#include "src/net/ota.h"
#include "src/core/board_log.h"
#include "src/core/command.h"
#include "src/ai/ai_client.h"
#include "src/exec/direct_exec.h"
#include "src/net/ble.h"

#include <ArduinoOTA.h>
#include <Update.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <esp_ota_ops.h>
#include <esp_app_desc.h>
#include <stdarg.h>
#include <lwip/sockets.h>   // lwip_socket/lwip_bind/lwip_close：探测 UDP 3232 是否真被 ArduinoOTA 占住
// 同 app_httpd.cpp / command.cpp：lwip 的 inet.h（经 sockets.h 引入）把 INADDR_NONE/IPADDR_NONE
// 定义成宏，与 Arduino core IPAddress.h 里的同名全局对象声明冲突。此处撤销，防后续 include 再踩。
#undef INADDR_NONE
#undef IPADDR_NONE

// ArduinoOTA 主机名（IDE 网络端口列表里显示 "<主机名> at <ip>"，mDNS 名同此）与上传口令。
// 主机名与 BLE 广播名一致，便于在 IDE 端口列表里认出这块板；口令用于挡同网段的误刷，
// 留空即不校验（IDE 检测到有口令，会在上传时提示输入）。
static const char* k_ota_host = "VisionS3";
static const char* k_ota_pass = "visions3";
static const uint16_t k_ota_port = 3232;

// 固件标记：手写标签（**升版本时改这里**）+ 自动的构建指纹，用来回答"板上跑的、以及刚 OTA
// 上去的到底是哪一份"。它出现在开机横幅、OTA 日志和 /update 页面。
static const char* k_fw_ver = "v3";   // v2: log 回执附带运行体征; v3: 体征再加 lwIP socket 表余量

// 指纹取 app 描述符里的 ELF SHA-256 前 4 字节（esptool 打包 .bin 时按 platform.txt 的
// `--elf-sha256-offset 0xb0` 打进去，正是该字段在镜像里的偏移）——换个源就是另一份 .bin，
// 指纹也就跟着变，这是判断"是不是新版本"唯一可靠的自动依据。
// ⚠️ 别改用描述符里的 date/time/version/project_name 当构建标记：本工程这几项是**冻住的**，
// 它们来自预编译核心库（esp32s3-libs 的 esp_app_desc.c.obj，值是库自己那次的
// "Sep 20 2026 / arduino-lib-builder"），每个 sketch 都一模一样——拿它做对比只会得出
// "OTA 没生效"的错觉。
static void fp_hex(const esp_app_desc_t* d, char* out, size_t n) {
  if (!d) {
    snprintf(out, n, "????????");
    return;
  }
  snprintf(out, n, "%02x%02x%02x%02x", d->app_elf_sha256[0], d->app_elf_sha256[1],
           d->app_elf_sha256[2], d->app_elf_sha256[3]);
}

// 运行中固件的标记，形如 "v1 指纹 fef509d0"：日志、网页、指令回执共用这一处渲染。
const char* ota::fw_stamp() {
  static char s[40];
  char fp[16];
  fp_hex(esp_app_get_description(), fp, sizeof(fp));
  snprintf(s, sizeof(s), "%s 指纹 %s", k_fw_ver, fp);
  return s;
}

// 读**分区里那份镜像**的标记（不是正在运行的那份）：OTA 刚写完就能报出"这次写进去的是哪份
// 构建"，不必等重启、也不必比对自己 PC 上的 .bin。分区为空/不是 app 镜像则返回 false。
static bool image_stamp(const esp_partition_t* part, char* out, size_t n) {
  if (!part) return false;
  esp_app_desc_t d;
  if (esp_ota_get_partition_description(part, &d) != ESP_OK) return false;
  char fp[16];
  fp_hex(&d, fp, sizeof(fp));
  snprintf(out, n, "指纹 %s", fp);
  return true;
}

static volatile bool s_active = false;   // 有 OTA 会话正在写固件（两条入口互斥，避免并发写同一分区）
// 本次 OTA 的落点分区，在写入**之前**记下来：Update.end(true) 会立刻把启动分区切过去，
// 那之后再调 esp_ota_get_next_update_partition() 拿到的已是旧槽，读它等于读老固件。
static const esp_partition_t* s_target = nullptr;

// 进入 OTA 前静默：把抢射频 / 抢 flash / 会让车乱动的东西全停掉。升级期间 CPU、射频、flash
// 都让给固件写入（写 flash 会短暂挂起另一个核，图传/AI/软解在跑只会互相拖慢，链路还抢时间片）。
static void ota_quiesce() {
  ai::cancel(ai::StopMode::All);  // 打断 AI 闭环，并中止在途 TLS 请求
  cmd::set_streaming(false);      // 停 UDP 图传（ws_stream_task 随即释放帧缓冲副本）
  JsonDocument doc;
  doc["scope"] = "all";
  // 停四轮 + 清机械臂连续动作；顺带覆盖 mvfy::end()（exec 的 stop 分支里已调，运动到位验证任务
  // 随即回到 IDLE 停止抓帧软解），故这里不必再单独停一次运动验证。
  exec::act("stop", doc.as<JsonObjectConst>());
  ble::set_quiet(true);  // 停 BLE 广播：与 WiFi 共用 2.4G 射频，广播开着 WiFi 吞吐掉到约 1/4
}

// 解除静默：只有 BLE 需要显式恢复——图传在静默前已被关掉，广播会按其状态自行决定；
// 升级成功路径紧接重启，广播随开机恢复，不需要这一步。
static void ota_restore() { ble::set_quiet(false); }

// 打印本次 OTA 的落点分区：有第二个 app 槽才会打印出与当前运行分区不同的槽位。
// 顺手把落点记进 s_target（写入前记，理由见其声明），并报出替换前板上跑的是哪份固件。
static void ota_log_target(const char* tag) {
  s_target = esp_ota_get_next_update_partition(nullptr);
  const char* cur = ota::fw_stamp();  // 本函数是文件级自由函数，不在 namespace ota 内，必须限名
  if (s_target) {
    blog::logf(blog::SYS, "%s：写入 %s @0x%06x（%u KB）；当前运行 %s", tag, s_target->label,
               (unsigned)s_target->address, (unsigned)(s_target->size / 1024), cur);
  } else {
    blog::logf(blog::SYS, "%s：找不到可写分区（分区表缺第二个 app 槽？）；当前运行 %s", tag, cur);
  }
}

static const char* ota_err_str(ota_error_t e) {
  switch (e) {
    case OTA_AUTH_ERROR:    return "认证失败（口令不符）";
    case OTA_BEGIN_ERROR:   return "启动失败（分区缺失/空间不足）";
    case OTA_CONNECT_ERROR: return "回连 PC 失败（PC 防火墙拦了回连端口？）";
    case OTA_RECEIVE_ERROR: return "接收失败（链路中断）";
    case OTA_END_ERROR:     return "收尾失败（长度/MD5 校验不过）";
    default:                return "未知";
  }
}

void ota::init() {
  const esp_partition_t* run = esp_ota_get_running_partition();
  // 开机横幅里报一次当前固件的指纹。与 OTA 完成时那行报的指纹一比对，就等于确认
  // "刚写进去的那份确实跑起来了"（两者不同 = 还在跑老固件）。
  // 注：这行发生在上电早期，手机/电脑的 WS 客户端此时还没接上，所以只进串口；
  // 带外获取见 /update 页面与 /log 指令回执（后者会同时到手机和 PC 脚本）。
  blog::logf(blog::SYS, "固件 %s", ota::fw_stamp());
  blog::logf(blog::SYS, "运行分区 %s @0x%06x；OTA 入口 %s.local:%u%s",
             run ? run->label : "?", (unsigned)(run ? run->address : 0), k_ota_host,
             (unsigned)k_ota_port, k_ota_pass[0] ? "（需口令）" : "（无口令）");

  ArduinoOTA.setHostname(k_ota_host);
  ArduinoOTA.setPort(k_ota_port);
  if (k_ota_pass[0]) ArduinoOTA.setPassword(k_ota_pass);

  ArduinoOTA.onStart([]() {
    s_active = true;
    ota_log_target("OTA 开始");
    ota_quiesce();
  });
  // 每 10% 打一行：该回调在收包循环里，逐包打印会严重拖慢上传。
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static uint8_t last = 0;
    if (!total) return;
    uint8_t pct = (uint8_t)((uint64_t)done * 100 / total);
    if (pct / 10 != last / 10) {
      last = pct;
      blog::logf(blog::SYS, "OTA %u%%（%u/%u KB）", (unsigned)pct, (unsigned)(done / 1024),
                 (unsigned)(total / 1024));
    }
  });
  ArduinoOTA.onEnd([]() {
    s_active = false;
    ota_restore();
    char written[96];  // 同 HTTP 路径：报出这次写进去的是哪次构建
    if (image_stamp(s_target, written, sizeof(written))) {
      blog::logf(blog::SYS, "OTA 写入完成，写入的固件 %s；重启中…", written);
    } else {
      blog::logf(blog::SYS, "OTA 写入完成（读不出新镜像标记），重启中…");
    }
  });
  ArduinoOTA.onError([](ota_error_t e) {
    s_active = false;
    ota_restore();  // 失败要解除静默：否则广播一直停着，手机既连不上也兜底不了
    blog::logf(blog::SYS, "OTA 失败[%u] %s；原固件继续运行", (unsigned)e, ota_err_str(e));
  });
}

// ==================== OTA 就绪判定：为什么是"探测"而不是 begin 一次就认定成功 ====================
// ArduinoOTA.begin() 在 3.3.11 里返回 void，且它唯一的失败早退是 `_udp_ota.begin(_port)` 起不来
// （内部堆紧到开不出 socket 或那 1460B 的 tx_buffer），那一步只打 IDF 的 log_e —— 走串口，进不了
// WS/BLE 日志通道；板子固定后看不到串口就等于没有。而原实现在 begin 之后**无条件**认定已启动，
// 于是这么一次失败会永久锁死 OTA（只能重启板子恢复），用户侧只表现为"OTA 连不上"，与网络卡死
// 无从区分——正是这一轮要修的那类静默闩锁。
// 判据反过来取：本板只有 ArduinoOTA 绑 3232（httpd 是 80/81、图传 UDP 端口由手机指定），故
//   bind 成功（抢到端口）⇒ 没有任何 PCB ⇒ begin 其实失败了；
//   bind 失败（EADDRINUSE）⇒ PCB 在 ⇒ begin 成功了。
// ArduinoOTA 自己的 socket 设了 SO_REUSEADDR、探测 socket 故意不设——lwIP 要求两端都设才允许复用，
// 故端口被占时我这侧必定失败，判据双向成立。
// 三态而非两态：连探测 socket 都开不出来（socket 表满 / 内存紧，正是卡死时的样子）时不下结论，
// 免得凭一次失败的探测就去 end() 掉一个其实好好的监听。
enum OtaPort { OTA_PORT_HELD, OTA_PORT_FREE, OTA_PORT_UNKNOWN };

static OtaPort ota_port_probe() {
  int fd = lwip_socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return OTA_PORT_UNKNOWN;
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(k_ota_port);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  OtaPort r = (lwip_bind(fd, (sockaddr*)&a, sizeof(a)) != 0) ? OTA_PORT_HELD : OTA_PORT_FREE;
  lwip_close(fd);  // 抢到的端口立刻还回去，别自己占着 3232
  return r;
}

static bool s_ready = false;      // 上次探测结论：3232 有人在听 = ArduinoOTA 真的在跑
static uint32_t s_check_at = 0;   // 上次探测/重试时刻（0 = 还没查过，boot 后第一轮就查）
static uint32_t s_fail_log_at = 0;  // 上次"未就绪"日志时刻（持续失败时限流，别刷屏）
static constexpr uint32_t k_check_ms = 30000;     // 探测周期（兼作失败重试间隔）
static constexpr uint32_t k_fail_log_ms = 60000;  // 持续失败时最多一分钟报一次

void ota::update() {
  // 未联网不起：ArduinoOTA 靠 UDP 收邀请、再回连 PC 取固件，没有 IP 无从谈起（配网后自动就绪）。
  if (WiFi.status() != WL_CONNECTED) return;
  // 升级进行中一律不碰：espota 那条路会阻塞在下面的 handle() 里（loop 根本回不到这），能走到这里的
  // 只有 HTTP /update —— 它在 httpd 任务里跑，loop 照常转。此刻既不该让探测去 end()/begin() 折腾
  // 射频与内部堆（写 flash 期间正是最紧的时候），也不该让第二个会话（espota 邀请）插进来并发写
  // 同一个分区。
  if (s_active) return;
  // 每 k_check_ms 用 3232 复核一次，没人听就重试 begin：这样一次性的启动失败、以及 WiFi 重连后
  // ArduinoOTA 被 end 掉之类，都会在几十秒内自愈，不再需要重启板子。
  if (s_check_at == 0 || millis() - s_check_at >= k_check_ms) {
    s_check_at = millis();
    OtaPort port = ota_port_probe();
    if (port == OTA_PORT_FREE) {  // 没人在听：重新起来
      // end() 先把半初始化状态清干净再 begin。这一步不只是洁癖：NetworkUDP::begin 的 socket 失败
      // 路径**不调 stop()**，会把刚 malloc 的 1460B tx_buffer 漏掉，只有 _udp_ota.stop()（= end 里
      // 那步）才 free —— 少了这行，重试每轮都漏 1.4KB。
      ArduinoOTA.end();
      ArduinoOTA.begin();
      port = ota_port_probe();
      if (port == OTA_PORT_HELD) {
        blog::logf(blog::SYS, "ArduinoOTA 就绪：%s.local:%u（IP %s）", k_ota_host,
                   (unsigned)k_ota_port, WiFi.localIP().toString().c_str());
      } else if (port == OTA_PORT_FREE &&
                 (!s_fail_log_at || millis() - s_fail_log_at >= k_fail_log_ms)) {
        s_fail_log_at = millis();
        blog::logf(blog::SYS,
                   "ArduinoOTA 未就绪：3232 没人听（begin 失败？）｜堆=%uk 最低=%uk 块=%uk｜%us 后重试",
                   (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMinFreeHeap() / 1024),
                   (unsigned)(ESP.getMaxAllocHeap() / 1024), (unsigned)(k_check_ms / 1000));
      }
      // 第二次探测若是 UNKNOWN（刚建过 socket，多半只是探测本身失败）则保留原状态：
      // handle() 内部对未初始化是空转，乐观放行不会出错。
    }
    if (port != OTA_PORT_UNKNOWN) s_ready = (port == OTA_PORT_HELD);
  }
  if (!s_ready) return;  // 明确没起来就别 handle（内部 _initialized=false，handle 会空转）
  // ⚠️ 上传期间这个 handle() 会一直阻塞到收完固件（数十秒），loop 里其它 update 在此期间停摆
  // ——这正是 onStart 要先静默小车、停图传的原因。
  ArduinoOTA.handle();
}

bool ota::active() { return s_active; }

// ==================== HTTP POST /update：浏览器/手机直传固件 ====================
// 请求体即 .bin 原始字节（页面上用 XHR 直接发 File，不走 multipart），免去解析表单。
// 同一分区同一时刻只允许一个会话（s_active）。

static const char k_runtime_mark[] = "<!--RUNTIME-->";  // 页面里的占位标记，见 update_get_handler

static const char k_update_page[] = R"HTML(<!doctype html><meta charset=utf-8>
<title>VisionS3 固件升级</title>
<style>body{font:14px/1.6 system-ui,sans-serif;margin:24px;max-width:640px}
code{background:#eee;padding:1px 4px;border-radius:3px}
#p{height:10px;background:#ddd;border-radius:5px;overflow:hidden;margin:12px 0}
#b{height:100%;width:0;background:#2b8a3e;transition:width .2s}
#s{white-space:pre-wrap;color:#333}
.rt{background:#f1f3f5;padding:10px 12px;border-radius:6px}
.warn{background:#fff3bf;padding:10px 12px;border-radius:6px}</style>
<h3>VisionS3 固件升级（HTTP OTA）</h3>
<!--RUNTIME-->
<p>选择编译出的 <code>Stm32-Vision.ino.bin</code> 上传。写入过程中车会停住、图传与 AI 会被
打断；成功后板子自动重启，失败则原固件继续运行。</p>
<input type=file id=f accept=".bin"><button id=go>上传并升级</button>
<div id=p><div id=b></div></div><div id=s></div>
<script>
const f=document.getElementById('f'),b=document.getElementById('b'),
s=document.getElementById('s'),go=document.getElementById('go');
go.onclick=()=>{
 if(!f.files[0]){s.textContent='先选 .bin 文件';return;}
 go.disabled=true;const x=new XMLHttpRequest();
 x.upload.onprogress=e=>{if(e.lengthComputable){const p=e.loaded/e.total*100;
  b.style.width=p.toFixed(1)+'%';s.textContent='上传中 '+p.toFixed(1)+'%';}};
 x.onload=()=>{s.textContent='板端返回 '+x.status+'：'+x.responseText+
  '\n已切换分区，板子重启中——等 5 秒后重新连接。';};
 x.onerror=()=>{s.textContent='连接中断（板子多半已开始重启）';go.disabled=false;};
 x.open('POST','/update');
 x.setRequestHeader('Content-Type','application/octet-stream');
 x.send(f.files[0]);};
</script>)HTML";

// 往 out 的 at 处续写一行（写满即截断，返回新的 at；绝不越界）。
static size_t appf(char* out, size_t n, size_t at, const char* fmt, ...) {
  if (at >= n) return n;  // 已满：后续续写一律丢弃（返回值停在 n 不再增长）
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(out + at, n - at, fmt, ap);
  va_end(ap);
  if (w < 0) return at;
  size_t nw = at + (size_t)w;
  return nw < n ? nw : n;  // vsnprintf 总会留终止符，截断也不会写出界
}

// 页面上"此刻板上是什么"那一段：运行分区 + 固件标记 + 下次 OTA 的落点及其内容。
// 每次请求现算，所以刷一下页面看到的就是当前状态（不是开机时的快照）。
static void runtime_info_html(char* out, size_t n) {
  const esp_partition_t* run = esp_ota_get_running_partition();
  const esp_partition_t* nxt = esp_ota_get_next_update_partition(nullptr);
  const char* part = run ? run->label : "?";
  char tgt[96];

  size_t at = appf(out, n, 0, "<p class=rt>当前运行：<b>%s</b><br>运行分区 <code>%s</code> @0x%06x",
                   ota::fw_stamp(), part, (unsigned)(run ? run->address : 0));
  if (!nxt) {
    appf(out, n, at,
         "</p><p class=warn>⚠ 找不到第二个 app 槽，OTA 无处可写——分区表须为双 app 槽方案"
         "（huge_app 是单槽，OTA 会失败）。</p>");
    return;
  }
  at = appf(out, n, at, "<br>下次 OTA 写入 <code>%s</code> @0x%06x（%u KB）",
            nxt->label, (unsigned)nxt->address, (unsigned)(nxt->size / 1024));
  if (image_stamp(nxt, tgt, sizeof(tgt))) {
    appf(out, n, at, "<br>该槽现有固件：%s</p>", tgt);
  } else {
    appf(out, n, at, "<br>该槽现有固件：空或非 app 镜像</p>");
  }
}

static esp_err_t update_get_handler(httpd_req_t* req) {
  // 运行时信息块现算后插到占位标记处：分块发出（标记前 → 信息块 → 标记后 → 收尾的空块）。
  char info[512];
  runtime_info_html(info, sizeof(info));

  const char* page = k_update_page;
  const char* mark = strstr(page, k_runtime_mark);
  httpd_resp_set_type(req, "text/html");
  if (!mark) {  // 标记万一被删，也得出得了页面，只是少了这段信息
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
  }
  esp_err_t err = httpd_resp_send_chunk(req, page, mark - page);
  if (err == ESP_OK) err = httpd_resp_send_chunk(req, info, HTTPD_RESP_USE_STRLEN);
  if (err == ESP_OK) {
    err = httpd_resp_send_chunk(req, mark + strlen(k_runtime_mark), HTTPD_RESP_USE_STRLEN);
  }
  if (err == ESP_OK) err = httpd_resp_send_chunk(req, NULL, 0);  // 空块 = 分块结束
  return err;
}

static esp_err_t update_post_handler(httpd_req_t* req) {
  if (s_active) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "已有 OTA 会话在进行");
  }
  size_t total = req->content_len;
  if (total == 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "请求体为空：需 POST 固件 .bin");
  }

  s_active = true;
  ota_quiesce();
  ota_log_target("HTTP OTA 开始");

  // 先按 Content-Length 开分区：超过分区容量在这里就失败，不必等传完。
  if (!Update.begin(total)) {
    blog::logf(blog::SYS, "HTTP OTA 失败：begin - %s", Update.errorString());
    s_active = false;
    ota_restore();
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, Update.errorString());
  }

  static uint8_t buf[4096];  // httpd 任务栈 8192，大缓冲放静态区不占栈
  size_t got = 0;
  while (got < total) {
    size_t want = total - got;
    if (want > sizeof(buf)) want = sizeof(buf);
    int n = httpd_req_recv(req, (char*)buf, want);
    if (n <= 0) {  // 超时/对端断开：别把半截固件留在分区里
      Update.abort();
      s_active = false;
      ota_restore();
      blog::logf(blog::SYS, "HTTP OTA 中断：已收 %u/%u KB（recv=%d）", (unsigned)(got / 1024),
                 (unsigned)(total / 1024), n);
      return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "接收中断，本次升级已放弃");
    }
    if (Update.write(buf, n) != (size_t)n) {
      blog::logf(blog::SYS, "HTTP OTA 写失败：%s", Update.errorString());
      Update.abort();
      s_active = false;
      ota_restore();
      return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, Update.errorString());
    }
    got += n;
  }

  if (!Update.end(true)) {
    blog::logf(blog::SYS, "HTTP OTA 收尾失败：%s", Update.errorString());
    s_active = false;
    ota_restore();
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, Update.errorString());
  }

  // 读完再报：把刚落进分区那份镜像自己的指纹读回来，答案就不依赖"我 PC 上刚编的是哪个文件"。
  // 拿它与重启后的开机横幅（或 /update 页面）比对，两边一致即确认新固件真的跑起来了。
  char written[96];
  if (image_stamp(s_target, written, sizeof(written))) {
    blog::logf(blog::SYS, "HTTP OTA 完成 %u KB，写入的固件 %s；重启中…", (unsigned)(got / 1024),
               written);
  } else {
    blog::logf(blog::SYS, "HTTP OTA 完成 %u KB（读不出新镜像标记），重启中…", (unsigned)(got / 1024));
  }
  httpd_resp_sendstr(req, "OK，已写入并切换分区");
  delay(300);  // 让响应先出网卡再重启
  ESP.restart();
  return ESP_OK;
}

void ota::http_register(httpd_handle_t server) {
  if (!server) return;
  httpd_uri_t get_uri = {
      .uri = "/update",
      .method = HTTP_GET,
      .handler = update_get_handler,
      .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
      ,
      .is_websocket = false,
      .handle_ws_control_frames = false,
      .supported_subprotocol = NULL
#endif
  };
  httpd_uri_t post_uri = {
      .uri = "/update",
      .method = HTTP_POST,
      .handler = update_post_handler,
      .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
      ,
      .is_websocket = false,
      .handle_ws_control_frames = false,
      .supported_subprotocol = NULL
#endif
  };
  httpd_register_uri_handler(server, &get_uri);
  httpd_register_uri_handler(server, &post_uri);
  blog::logf(blog::SYS, "HTTP OTA 入口已注册：http://<板IP>/update（浏览器打开即可升级）");
}
