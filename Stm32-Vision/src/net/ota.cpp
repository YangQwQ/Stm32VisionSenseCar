#include "src/net/ota.h"
#include "src/core/board_log.h"
#include "src/core/command.h"
#include "src/ai/ai_client.h"
#include "src/exec/direct_exec.h"
#include "src/exec/motion_verify.h"
#include "src/net/ble.h"

#include <ArduinoOTA.h>
#include <Update.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <esp_ota_ops.h>

// ArduinoOTA 主机名（IDE 网络端口列表里显示 "<主机名> at <ip>"，mDNS 名同此）与上传口令。
// 主机名与 BLE 广播名一致，便于在 IDE 端口列表里认出这块板；口令用于挡同网段的误刷，
// 留空即不校验（IDE 检测到有口令，会在上传时提示输入）。
static const char* k_ota_host = "VisionS3";
static const char* k_ota_pass = "visions3";
static const uint16_t k_ota_port = 3232;

static bool s_started = false;           // ArduinoOTA 已 begin（UDP 已绑定）
static volatile bool s_active = false;   // 有 OTA 会话正在写固件（两条入口互斥，避免并发写同一分区）

// 进入 OTA 前静默：把抢射频 / 抢 flash / 会让车乱动的东西全停掉。升级期间 CPU、射频、flash
// 都让给固件写入（写 flash 会短暂挂起另一个核，图传/AI/软解在跑只会互相拖慢，链路还抢时间片）。
static void ota_quiesce() {
  ai::cancel(ai::StopMode::All);  // 打断 AI 闭环，并中止在途 TLS 请求
  cmd::set_streaming(false);      // 停 UDP 图传（ws_stream_task 随即释放帧缓冲副本）
  JsonDocument doc;
  doc["scope"] = "all";
  exec::act("stop", doc.as<JsonObjectConst>());  // 停四轮 + 清机械臂连续动作
  ble::set_quiet(true);  // 停 BLE 广播：与 WiFi 共用 2.4G 射频，广播开着 WiFi 吞吐掉到约 1/4
}

// 解除静默：只有 BLE 需要显式恢复——图传在静默前已被关掉，广播会按其状态自行决定；
// 升级成功路径紧接重启，广播随开机恢复，不需要这一步。
static void ota_restore() { ble::set_quiet(false); }

// 打印本次 OTA 的落点分区：有第二个 app 槽才会打印出与当前运行分区不同的槽位。
static void ota_log_target(const char* tag) {
  const esp_partition_t* p = esp_ota_get_next_update_partition(nullptr);
  if (p) {
    blog::logf(blog::SYS, "%s：写入 %s @0x%06x（%u KB）", tag, p->label, (unsigned)p->address,
               (unsigned)(p->size / 1024));
  } else {
    blog::logf(blog::SYS, "%s：找不到可写分区（分区表缺第二个 app 槽？）", tag);
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
    blog::logf(blog::SYS, "OTA 写入完成，重启中…");
  });
  ArduinoOTA.onError([](ota_error_t e) {
    s_active = false;
    ota_restore();  // 失败要解除静默：否则广播一直停着，手机既连不上也兜底不了
    blog::logf(blog::SYS, "OTA 失败[%u] %s；原固件继续运行", (unsigned)e, ota_err_str(e));
  });
}

void ota::update() {
  // 未联网不起：ArduinoOTA 靠 UDP 收邀请、再回连 PC 取固件，没有 IP 无从谈起（配网后自动就绪）。
  if (WiFi.status() != WL_CONNECTED) return;
  if (!s_started) {
    // 只 begin 一次：内部对已初始化的重复调用直接返回。若日志出现 "udp bind failed"，
    // 说明 3232 被占，重启本板即可恢复。
    ArduinoOTA.begin();
    s_started = true;
    blog::logf(blog::SYS, "ArduinoOTA 就绪：%s.local:%u（IP %s）", k_ota_host, (unsigned)k_ota_port,
               WiFi.localIP().toString().c_str());
  }
  // ⚠️ 上传期间这个 handle() 会一直阻塞到收完固件（数十秒），loop 里其它 update 在此期间停摆
  // ——这正是 onStart 要先静默小车、停图传的原因。
  ArduinoOTA.handle();
}

bool ota::active() { return s_active; }

// ==================== HTTP POST /update：浏览器/手机直传固件 ====================
// 请求体即 .bin 原始字节（页面上用 XHR 直接发 File，不走 multipart），免去解析表单。
// 同一分区同一时刻只允许一个会话（s_active）。

static const char k_update_page[] = R"HTML(<!doctype html><meta charset=utf-8>
<title>VisionS3 固件升级</title>
<style>body{font:14px/1.6 system-ui,sans-serif;margin:24px;max-width:640px}
code{background:#eee;padding:1px 4px;border-radius:3px}
#p{height:10px;background:#ddd;border-radius:5px;overflow:hidden;margin:12px 0}
#b{height:100%;width:0;background:#2b8a3e;transition:width .2s}
#s{white-space:pre-wrap;color:#333}</style>
<h3>VisionS3 固件升级（HTTP OTA）</h3>
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

static esp_err_t update_get_handler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, k_update_page, HTTPD_RESP_USE_STRLEN);
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

  blog::logf(blog::SYS, "HTTP OTA 完成 %u KB，重启中…", (unsigned)(got / 1024));
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
