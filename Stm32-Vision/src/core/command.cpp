#include "src/core/command.h"
#include "src/net/config.h"
#include "src/net/wifi_net.h"
#include "src/ai/ai_client.h"
#include "src/exec/direct_exec.h"
#include "src/net/ping_svc.h"
#include "src/exec/nezha_direct.h"
#include "src/core/board_log.h"
#include "src/core/heap_watch.h"   // 体征行里的 DMA 块最低水位（见 heap_watch.h）
#include "src/net/ota.h"
#include <esp_heap_caps.h>   // /mem：内部堆各 region 的空闲/最大连续块（定位碎片来源）

#include <lwip/sockets.h>   // lwip_socket / lwip_close（socket 表自检，见 net_free_sockets）
// 同 app_httpd.cpp：lwip 的 inet.h（经 sockets.h 引入）把 INADDR_NONE/IPADDR_NONE 定义成宏，
// 与 Arduino core IPAddress.h 里的同名全局对象声明冲突。此处撤销，防后续 include 再踩。
#undef INADDR_NONE
#undef IPADDR_NONE

// 应答格式遵循架构 §5.1：板 → 手机文本 = {type:status/pong, params:{...}, id:<回填>}。
// move/stop/arm 是高频手动指令，只在 UART 层记录，不回文本（避免刷屏）。

static bool g_streaming = false;         // 图传开关全局状态（WS 推流任务读取）

static bool has_id(const JsonDocument& doc) {
  return doc["id"].is<int>() || doc["id"].is<long>();
}

// 合成板端状态位字节，供 get_state 与 reply_status 统一附带，手机端按位解析同步按钮。
// bit 布局与 Mobile-RemoteCtrl/Main.gd（_apply_state_bits）逐位 mirror，改一侧必改另一侧：
//   bit0 前灯 / bit1 震灯 / bit2 背灯 / bit3 夹爪夹紧 / bit4 AI busy；bit5-7 留空。
uint8_t cmd::state_bits() {
  uint8_t b = 0;
  if (exec::light_on("front")) b |= 1u << 0;
  if (exec::light_on("vibe"))  b |= 1u << 1;
  if (exec::light_on("back"))  b |= 1u << 2;
  if (exec::grip_closing())    b |= 1u << 3;
  if (ai::busy())              b |= 1u << 4;
  return b;
}

// 本次开机是怎么来的（log 回执随体征一起回报，见下）：直接区分"板子自己复位了"
// （崩溃/看门狗/掉电）与"人手动断电"（上电）。板上跑挂后网络同时没了，重启后这行字
// 就是唯一能说明上一次运行怎么结束的东西 —— 不需要串口。
static const char* reset_text() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "上电";
    case ESP_RST_EXT:       return "外部";
    case ESP_RST_SW:        return "软件";
    case ESP_RST_PANIC:     return "崩溃";
    case ESP_RST_INT_WDT:   return "中断狗";
    case ESP_RST_TASK_WDT:  return "任务狗";
    case ESP_RST_WDT:       return "看门狗";
    case ESP_RST_DEEPSLEEP: return "深睡";
    case ESP_RST_BROWNOUT:  return "掉电";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "未知";
  }
}

// 数一数 lwIP 的 socket 表还剩几个空位（连开裸 socket 直到失败，再全部关掉）。开的是未连接的
// socket，不占 TCP PCB、不发一个包，代价就是十几个 netconn 的瞬时分配。
// 为什么值得每 30s 报一次：卡死时 WiFi 仍关联、BLE 仍活，却所有 TCP 都建不起来——两种可能
// （a）连接泄漏把 16 个槽位占满（脚本/手机反复重连、httpd 的会话没回收），于是出站 connect 直接
// 失败（正是 ai_http 打的 code=-1）、httpd 无法 accept（手机侧表现为"握手超时"）；（b）WiFi TX
// 整体哑了，跟 socket 表无关。这一行 + 手敲 /ping <网关> 就能把两者分开：ICMP 走 raw pcb、不经
// socket 表——`套接字余=0` 而 ping 通 ⇒ (a)；两者皆死 ⇒ (b)。平时它则是一条泄漏曲线。
static int net_free_sockets() {
  int fd[32];
  int n = 0;
  while (n < 32 && (fd[n] = lwip_socket(AF_INET, SOCK_STREAM, 0)) >= 0) n++;
  for (int i = 0; i < n; i++) lwip_close(fd[i]);
  return n;
}

// 组一段带原 id 的应答文本并发出（reply 可空）。
static void reply_status(JsonDocument& src, cmd::ReplyFn reply, void* ctx,
                         const char* reason) {
  if (!reply) {
    blog::logf(blog::CMD, "(无回复通道) %s", reason);
    return;
  }
  JsonDocument out;
  out["type"] = "status";
  JsonObject params = out["params"].to<JsonObject>();
  params["reason"] = reason;
  params["bits"] = cmd::state_bits();  // 附带状态位，让"会触发动作重置"的回执驱动手机端自动同步按钮
  if (has_id(src)) out["id"] = src["id"].as<long>();
  String s;
  serializeJson(out, s);
  reply(ctx, s.c_str());
}

// 与手机 /ping 对齐：回 {type:pong}（手机 WSCarClient 对 pong 直接读文本）。
// 附带状态位（与 status/get_state 同一个字节，见 state_bits）：保活是手机**自己每几秒就会发**的，
// 于是"AI 任务在不在跑"不必等一次动作回执。任务由另一侧起停（另一台手机 / 另一个客户端发的
// ai_goal）、或手机漏收了一次 ai_result 时，按钮能在一个保活周期内自行纠正回来 —— 这正是
// "发送按钮没在任务运行时变成中止"那类不同步的兜底来源。每次 pong 都是**当场现测**的板端状态，
// 不是某条陈旧的回执，所以手机端可以放心用它覆盖本地 AI 运行态。
static void reply_pong(JsonDocument& src, cmd::ReplyFn reply, void* ctx) {
  if (!reply) return;
  JsonDocument out;
  out["type"] = "pong";
  JsonObject params = out["params"].to<JsonObject>();
  params["bits"] = cmd::state_bits();
  if (has_id(src)) out["id"] = src["id"].as<long>();
  String s;
  serializeJson(out, s);
  reply(ctx, s.c_str());
}

// 手动指令串口日志：只有指令类型切换时才打一行当作"确认收到"；
// 同一类型连续重复（摇杆高频帧）完全不输出，避免刷屏、也避免拖慢后续指令处理。
static void log_manual_throttled(const char* type) {
  static String s_last;
  if (s_last != type) {
    s_last = type;
    blog::logf(blog::CMD, "收到手动指令 %s", type);
  }
}

// OTA 升级期间的硬闸门：只放行不占射频/CPU 的三类——活体探测、状态查询、日志开关
// （手机 6s 单轮收不到 pong 就判死并重连，断链风暴比老老实实回一句更吵，故必须留活口）。
// 其余一律回绝并说明原因：升级正在写 flash，任何把图传/AI/电机重新开起来的指令都在抢射频
// 与 CPU，会把固件传输拖成涓流（实测因此超时、上传失败）。升级开始时的"静默一次"不够——
// 手机自动重连会把 stream on 之类原样重放回来，闸门必须是持续的。
static bool ota_gate_blocks(const char* type, JsonObject params) {
  if (!ota::active()) return false;
  if (!strcmp(type, "pong") || !strcmp(type, "get_state") || !strcmp(type, "log")) return false;
  if (!strcmp(type, "ping")) {  // 无目标 = 就地回 pong（活体探测）；带目标要发 ICMP，占射频
    const char* tgt = params["target"] | "";
    return tgt[0] != 0;
  }
  return true;
}

void cmd::handle(const char* json, bool has_frames, ReplyFn reply, void* reply_ctx) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) {
    blog::logf(blog::CMD, "bad json: %s", json);
    return;
  }
  const char* type = doc["type"] | "";
  JsonObject params = doc["params"].as<JsonObject>();
  if (ota_gate_blocks(type, params)) {
    reply_status(doc, reply, reply_ctx, "固件升级中，指令已忽略（数十秒后自动恢复）");
    return;
  }
  // 手动接管类型：move/stop/arm 之外，摇杆/直控面板的直接驱动指令（drive/spin/servo/motor/
  // arm_pose/reset）同样会接管小车运动，必须打断板载 AI 闭环，否则 AI 与手动抢控制权。
  bool manual = !strcmp(type, "move") || !strcmp(type, "stop") || !strcmp(type, "arm") ||
                !strcmp(type, "drive") || !strcmp(type, "spin") || !strcmp(type, "servo") ||
                !strcmp(type, "motor") || !strcmp(type, "arm_pose") || !strcmp(type, "reset");
  if (manual) {
    // 手动高频指令：先立即打断 AI 闭环并直接驱动哪吒扩展板（电机控制优先），
    // 串口日志只在类型切换时打一行，绝不让日志阻塞控制时序。
    // move/stop/arm 由本板直驱，不再经执行板。arm/arm_pose 打断只停轮子（机械臂指令即接管）。
    ai::cancel(!strcmp(type, "arm") || !strcmp(type, "arm_pose") ? ai::StopMode::Wheels : ai::StopMode::None);
    log_manual_throttled(type);
    if (!strcmp(type, "move") || !strcmp(type, "stop") || !strcmp(type, "arm")) {
      exec::act(type, params);
      return;
    }
    // drive/spin/servo/motor/arm_pose/reset 继续走下方各自分支（取消后落各自直驱逻辑）。
  } else if (strcmp(type, "ping") && strcmp(type, "pong") && strcmp(type, "get_state")) {
    // 保活/纯查询三件套（ping / pong / get_state）不打这一行：手机每几秒就一来一回，打出来只是刷屏，
    // 而它们的应答本身就是自描述的状态（pong 带 bits、get_state 回 bits）。其余类型（ai_goal / config /
    // goto…）仍留这一行当"收到过"的凭据。
    blog::logf(blog::CMD, "type=%s has_frames=%u", type, has_frames);
  }

  if (!strcmp(type, "config")) {
    // 配网（BLE 或 WS 同结构）：落 NVS → 在线重建 STA（不重启，BLE 保活）→ 上线后 BLE 上报 IP
    const char* ssid = params["ssid"] | "";
    const char* pass = params["password"] | "";
    if (!ssid[0]) {
      reply_status(doc, reply, reply_ctx, "config: SSID 为空");
      return;
    }
    if (!cfg::set_wifi(ssid, pass)) {
      reply_status(doc, reply, reply_ctx, "WiFi 配置未变化，跳过");
      return;
    }
    apply_network();
    reply_status(doc, reply, reply_ctx, "WiFi 配置已保存，正在连接…");
    return;
  }

  if (!strcmp(type, "ping")) {
    // 无 target = 测小车连通性（回 pong）；带 target（IP/域名）= 板子去 ping 并回报延迟。
    // ⚠️ 排查卡死时用 IP 字面量（域名要先做 DNS，tcpip 线程卡住时连解析都不返回）。
    const char* target = params["target"] | "";
    if (target[0]) {
      // 起不来必须出声：ping 会话要一段连续 16KB 的栈（已挪 PSRAM）加若干分配，内部堆紧时仍可能
      // 失败。以前这里把返回值丢掉 —— 用户只看到"正在 ping…"然后永远没有下文，跟"网络不通"完全
      // 分不开，等于把一次内存故障误读成链路故障。带上体征就能当场分清（详细原因另由 ping_svc 的
      // log_fail 走日志转发报出）。
      if (!ping::start(target, 5, reply, reply_ctx)) {
        char buf[224];
        snprintf(buf, sizeof(buf),
                 "ping %s: 未发起（上一个 ping 未结束 或 内部堆不足）｜堆=%uk 最低=%uk 块=%uk psram=%uk",
                 target, (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMinFreeHeap() / 1024),
                 (unsigned)(ESP.getMaxAllocHeap() / 1024), (unsigned)(ESP.getFreePsram() / 1024));
        reply_status(doc, reply, reply_ctx, buf);
        return;
      }
      String tip = "正在 ping " + String(target) + "…";
      reply_status(doc, reply, reply_ctx, tip.c_str());
    } else {
      reply_pong(doc, reply, reply_ctx);
    }
    return;
  }

  if (!strcmp(type, "reboot")) {
    // 远程重启：链路卡死（WiFi/IP 死而 BLE 还活着）时唯一能远程按下的那一下，值得留着。
    // 升级期间被上面的 ota_gate_blocks 拦下——写 flash 时重启等于砖，那不是后门是坑。
    // 报两遍是为覆盖两类观察者：回执只到发起方那一个 fd，而日志转发是广播给所有 WS 客户端
    // + BLE 的，于是"别人按的重启"旁边的人也能看见（不必经手机端中转）。
    reply_status(doc, reply, reply_ctx, "正在重启…");
    blog::logf(blog::SYS, "收到 reboot：重启中（卡死时跑的 %s）", ota::fw_stamp());
    // 回执是同步发出的，但日志转发得等 blog 的转发任务醒来再经 WS/BLE 送出去：300ms 只够前者
    // （OTA 那条路就是这么写的），这里留一倍余量把最后一行送完再断电。多这几百毫秒不影响
    // "能远程重启"这件事本身——重启后日志通道本来也要重连。
    delay(600);
    ESP.restart();
    return;
  }

  if (!strcmp(type, "pong")) {
    return;  // 板子 WS 活体探测的应答：仅当上行续活用，静默不回复
  }

  if (!strcmp(type, "stream")) {
    // 图传开关：更新全局状态，WS 推流任务读取。BLE 无帧通道也能开/关图传。
    bool on = params["on"] | false;
    set_streaming(on);
    reply_status(doc, reply, reply_ctx, on ? "图传已开启" : "图传已关闭");
    return;
  }

  if (!strcmp(type, "log")) {
    // 统一日志转发开关：/log <exec|ai|all> on|off。
    // 默认全关；exec = 直驱执行日志+周期状态推送，ai = AI 调试日志，
    // all = 板端全部类别日志转发手机（经统一日志队列）。三者单选（见 board_log.h）。
    const char* cat = params["cat"] | "";
    bool on = params["on"] | false;
    const char* what = nullptr;
    // 施加前后各取一次"当前生效类别"，两次相同即本次是**幂等重申**：PC 脚本每 30s 会重申一次
    // /log on（防手机改掉那个全局单选），若照旧回一整句，日志里就每 30s 多一条一模一样的
    // "已开启（当前…）+固件"，看着像板子在反复重开日志——其实什么都没变。
    char st_before[48], st_after[48];
    blog::state_text(st_before, sizeof(st_before));
    if (!strcmp(cat, "all")) { blog::set_all(on); what = "全部"; }
    else if (!strcmp(cat, "exec")) { blog::enable(blog::EXEC, on); what = "执行"; }
    else if (!strcmp(cat, "ai")) { blog::enable(blog::AI, on); what = "AI"; }
    else {
      reply_status(doc, reply, reply_ctx, "log: cat 需 exec|ai|all");
      return;
    }
    blog::state_text(st_after, sizeof(st_after));
    // 回执附带当前生效类别：只回报本次开关时，残留的 all（或 all 期内的降级）用户无从察觉，
    // 会出现"我只开了 ai 却全类别都在发"和"这条 off 到底生效没有"两类困惑。
    // 再附上当前固件标记：开日志是手机/PC 脚本每次接入的必经一步，顺路把"板上跑的是哪份固件"
    // 送到两边（OTA 后正是靠这行确认新固件到底生效没有），不必另开串口或网页。
    // 再顺路回报运行体征（内部堆三口径 + 复位原因 + 运行秒数）：开日志是手机/PC 每次接入的必经
    // 一步，于是这行在日志里天然形成一条体征曲线。板上跑挂时 WiFi/WS 一起没了、串口又拆不下来，
    // 这行就是事后唯一的现场 —— 而 BLE 有自己预分配的控制器缓冲，卡死时照常可连（实测指令照收），
    // 卡死后用手机走 BLE 敲一次 /log on 即可当场取回：
    //   `最低`/`块` 逼近 0 ⇒ 内部堆耗尽（lwIP / WiFi-TX 分不到缓冲 ⇒ ARP 与 TCP 全哑，而 WiFi 关联
    //   和 BLE 都还活着）——症状正是"连着热点却谁都不通、蓝牙却还能用"；
    //   `运行` 秒数在某次重连后归零 ⇒ 这中间其实复位过，再看`复位`是哪一种（崩溃/看门狗/掉电）；
    //   `套接字余` 逼近 0 ⇒ lwIP 的 16 个 socket 槽位被泄漏的连接占满（判读见 net_free_sockets）。
    // 卡死时除了这行，再手敲一条 `/ping <网关>`：ICMP 不经 socket 表，两者一对照即可定位层级。
    // 再加一个 `DMA块最低`：这是**唯一**能预报"WiFi 收不进包"的指标。WiFi 的 RX 缓冲必须落
    // DMA 可达的内部 RAM，闸门是那块最大连续块——它逼近 0 时收包必失败（ARP 都不回、ping 全灭），
    // 而此时 `堆=`（总空闲）可能看着还挺富余，`最低=` 也只是个含 PSRAM 无关的总量口径。
    // 实测：健康基线 `DMA块最低≈7k`；AI 任务一轮能把内部堆压到 `最低=1k`，链路随即全哑且不自愈。
    // 该值由 heap_watch 的 20ms 哨兵记录（AI 每轮任务起点清零，故它=最近一轮任务的最深点）。
    // ⚠️ ESP.getFreeHeap / getMinFreeHeap / getMaxAllocHeap 三者同为 MALLOC_CAP_INTERNAL 口径
    // （cores/esp32/Esp.cpp），与 ai_http 打的 `heap=` 可直接对照；别改成含 PSRAM 的口径。
    // `DMA块最低` 的 0 有两义：**真触底**与**哨兵没起来**（heap_watch 的栈没分配上）。
    // 两者含义相反、严重性也相反，印成同一个 `0k` 就等着被误读；未起来时印 `--`。
    // DMA 块给**两个**数：`现`=当前最大连续块，`低`=谷底（自 AI 任务起点清零）。
    // 缺一个就判不了 —— 两者的结论相反：`现` 就很小 ⇒ 这个池**结构性**贴底（与任务无关，是内存
    // 布局问题，告警等于常亮噪声）；`现` 正常而 `低`=0 ⇒ 是被某一轮任务压下去的（那才是有信息量的
    // 那次事件）。此前只印谷底，于是"啥也没干也一直弹"这件事在网侧完全看不见（见 heap_watch 注释）。
    // 与上一行的 `块=`（MALLOC_CAP_INTERNAL 口径）对照着看更关键：两者口径不同，`块=11k` 而
    // `DMA块现` 只有几 k 是常见组合，说明吃紧的是 DMA 可达的那个子集，不是内部 RAM 总量。
    char dma_buf[32];
    const char* dma_txt = "--";
    if (hwatch::ready()) {
      snprintf(dma_buf, sizeof(dma_buf), "%uk/低%uk", (unsigned)(hwatch::cur_largest_dma() / 1024),
               (unsigned)(hwatch::min_largest_dma() / 1024));
      dma_txt = dma_buf;
    }
    char msg[320];  // 命名避开 cfg（本文件另有 cfg:: 命名空间）；加 DMA块后需比原 224 宽
    if (!strcmp(st_before, st_after)) {
      // 幂等重申：只回一行体征当心跳。类别与固件都没变，重复报它们没有信息量；而体征必须照给
      // ——板子跑挂、WiFi 也断了时，这行是唯一还能出来的现场（走 BLE，见上）。
      snprintf(msg, sizeof(msg),
               "日志转发未变（当前: %s）｜复位=%s 堆=%uk 最低=%uk 块=%uk DMA块(现/低)=%s 套接字余=%d 运行=%us",
               st_after, reset_text(), (unsigned)(ESP.getFreeHeap() / 1024),
               (unsigned)(ESP.getMinFreeHeap() / 1024), (unsigned)(ESP.getMaxAllocHeap() / 1024),
               dma_txt, net_free_sockets(), (unsigned)(millis() / 1000));
    } else {
      snprintf(msg, sizeof(msg),
               "%s日志转发已%s（当前: %s）｜固件 %s｜复位=%s 堆=%uk 最低=%uk 块=%uk DMA块(现/低)=%s psram=%uk 套接字余=%d 运行=%us",
               what, on ? "开启" : "关闭", st_after, ota::fw_stamp(), reset_text(),
               (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMinFreeHeap() / 1024),
               (unsigned)(ESP.getMaxAllocHeap() / 1024),
               dma_txt, (unsigned)(ESP.getFreePsram() / 1024),
               net_free_sockets(), (unsigned)(millis() / 1000));
    }
    reply_status(doc, reply, reply_ctx, msg);
    return;
  }

  if (!strcmp(type, "nz_read")) {
    // I2C 诊断（只读，不下发指令）：探测哪吒从机是否在线。写地址 ACK=从机活着且总线可用；
    // 两项皆无 = 总线/从机掉电或拉死（常见硬件故障，此时回正与控制全无响应）。
    uint8_t lb = 0;
    uint8_t st = nezha::probe(&lb);
    char buf[120];
    const char* core = (st & 1u) ? ((st & 2u) ? "哪吒在线(写ACK✓ 读ACK✓)" : "哪吒在位 读应答✗")
                                 : "哪吒无响应(写地址无ACK)";
    if ((st & 3u) == 3u) snprintf(buf, sizeof(buf), "%s 首字节=0x%02X -> I2C/从机正常", core, lb);
    else snprintf(buf, sizeof(buf), "%s -> 检查I2C连线/从机供电/总线是否被拉死", core);
    reply_status(doc, reply, reply_ctx, buf);
    return;
  }

  if (!strcmp(type, "get_state")) {
    // 主动查询当前状态（供手机重连后同步控制按钮）：回状态位字节，手机端按位解析灯/夹爪/AI 运行态。
    JsonDocument out;
    out["type"] = "state";
    JsonObject params = out["params"].to<JsonObject>();
    params["bits"] = cmd::state_bits();
    if (has_id(doc)) out["id"] = doc["id"].as<long>();
    String s;
    serializeJson(out, s);
    if (reply) { reply(reply_ctx, s.c_str()); }
    return;
  }

  if (!strcmp(type, "mem")) {
    // 内部堆 region 全览（诊断，只读）：回各池空闲/最大连续块/谷底，判 DMA 池是被固定大块占住
    // 还是运行时 churn 碎出的（前者 largest 恒小、后者谷底为 0）。串口再留一份 region 映射明细。
    heap_caps_dump_all();
    auto fill = [](JsonObject o, uint32_t caps) {
      multi_heap_info_t hi;
      heap_caps_get_info(&hi, caps);
      o["free"] = (uint32_t)hi.total_free_bytes;
      o["largest"] = (uint32_t)hi.largest_free_block;
      o["min"] = (uint32_t)hi.minimum_free_bytes;
      o["blocks"] = (uint32_t)hi.free_blocks;
    };
    JsonDocument out;
    out["type"] = "mem";
    JsonObject params = out["params"].to<JsonObject>();
    { JsonObject o = params["internal"].to<JsonObject>();        fill(o, MALLOC_CAP_INTERNAL); }
    { JsonObject o = params["dma_internal"].to<JsonObject>();    fill(o, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA); }
    { JsonObject o = params["psram"].to<JsonObject>();           fill(o, MALLOC_CAP_SPIRAM); }
    { JsonObject o = params["dma_external"].to<JsonObject>();    fill(o, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA); }
    String s;
    serializeJson(out, s);
    blog::logf(blog::NET, "mem: %s", s.c_str());
    if (reply) { reply(reply_ctx, s.c_str()); }
    return;
  }

  if (!strcmp(type, "light")) {
    // 直驱灯光（绕过执行板）：/light <front|vibe|back> <0|1>。
    bool on = params["on"] | false;
    if (exec::act("light", params)) {
      reply_status(doc, reply, reply_ctx, on ? "灯已开" : "灯已关");
    } else {
      reply_status(doc, reply, reply_ctx, "light: kind 需 front|vibe|back");
    }
    return;
  }

  if (!strcmp(type, "reset")) {
    // 回正：机械臂四舵机回中 + 电机停（绕过执行板，直驱）。
    exec::reset();
    reply_status(doc, reply, reply_ctx, "已回正");
    return;
  }

  if (!strcmp(type, "ai_goal") || !strcmp(type, "ai_oneshot")) {
    // DIRECT：手机下发目标 → 板载 ai_client 调 AI。ai_goal=迭代闭环；ai_oneshot=只执行一轮收尾。
    if (cfg::ai_key().isEmpty()) {
      reply_status(doc, reply, reply_ctx, "AI 未配置（ai_key 为空），未调用云端");
      return;
    }
    const char* msg = params["message"] | "";
    bool use_image = params["use_image"] | false;
    bool one_shot = !strcmp(type, "ai_oneshot");

    // 组标注字符串（供 AI 观察近似意图区域）
    String ann;
    if (params["annotation"].is<JsonObject>()) {
      serializeJson(params["annotation"].as<JsonObjectConst>(), ann);
    }

    // WS 异步回复需堆拷贝 fd（ai_client 任务结束时释放）；BLE 传 nullptr。
    void* actx = nullptr;
    if (reply_ctx) actx = new int(*(int*)reply_ctx);
    long id = has_id(doc) ? doc["id"].as<long>() : 0;
    ai::set_goal(msg, use_image, ann.length() ? ann.c_str() : nullptr, id, reply, actx, one_shot);
    reply_status(doc, reply, reply_ctx, one_shot ? "已收到单轮 AI 目标" : "已收到目标，AI 处理中");
    return;
  }

  if (!strcmp(type, "ai_cancel")) {
    // 显式取消 AI 任务：残留持续指令会在任务出口补停（≠强制停车）。
    ai::cancel(ai::StopMode::All);
    reply_status(doc, reply, reply_ctx, "AI 任务已取消");
    return;
  }

  if (!strcmp(type, "ai_chat")) {
    // AI 任务进行中"插话"：把用户补充喂给当前任务，不打断（区别于 ai_goal/ai_cancel）。
    // 无任务在跑时忽略，仅回执提示——不当作新目标接管。
    const char* msg = params["message"] | "";
    if (!msg[0]) {
      reply_status(doc, reply, reply_ctx, "ai_chat: message 为空");
      return;
    }
    bool fed = ai::append_chat(msg);
    reply_status(doc, reply, reply_ctx, fed ? "已补充给 AI（任务继续）" : "当前无进行中的 AI 任务，补充被忽略");
    return;
  }

  if (!strcmp(type, "goto")) {
    // /move to x y [global|local]：板端本地巡航到坐标（不调 AI，手动接管类）。
    // 需 x,y；frame=local（默认，原点=当前位姿，y向前 x向右）/global（沿用全局系）。
    bool has_x = params["x"].is<float>() || params["x"].is<int>();
    bool has_y = params["y"].is<float>() || params["y"].is<int>();
    if (!has_x || !has_y) {
      reply_status(doc, reply, reply_ctx, "goto: 需 x,y 坐标（local 原点=当前位姿，y向前 x向右；frame=global 沿用全局系）");
      return;
    }
    float x = params["x"] | 0.f;
    float y = params["y"] | 0.f;
    bool global = !strcmp((const char*)(params["frame"] | "local"), "global");
    void* actx = reply_ctx ? new int(*(int*)reply_ctx) : nullptr;
    long id = has_id(doc) ? doc["id"].as<long>() : 0;
    ai::goto_target(x, y, global, id, reply, actx);
    reply_status(doc, reply, reply_ctx, "开始导航到坐标");
    return;
  }

  if (!strcmp(type, "servo")) {
    // 调试直驱：绕过执行板，本板软件 I2C 直接驱动哪吒舵机（逻辑通道 n=0转向/1左(前后)/2右(抬落)/3前(夹爪)）。
    // 直接写原始 pwm（50..250），不过标定限位，用于探机械极限/标定；走 exec::set_servo 同步内部状态。
    int n = params["n"] | -1;
    int p = params["pwm"] | -1;
    if (n >= 0 && n <= 3 && p >= 50 && p <= 250) {
      exec::set_move_cap_ms(0);  // 手动接管轮子/转向：先清 AI move 兜底，防旧时限在写入窗口误停
      exec::set_servo((uint8_t)n, (uint16_t)p);
      // 成功不回执：摇杆/按钮高频下发，状态看 exec_log 即知。
    } else {
      reply_status(doc, reply, reply_ctx, "servo: n=0转向/1左/2右/3前 pwm=50..250(可超标定限位)");
    }
    return;
  }

  if (!strcmp(type, "arm_pose")) {
    // 二连杆 IK：给末端位姿(x=车头系前方cm, h=离地高度cm)，联动算左右两舵机 pwm 一并下发。
    // 校准用：不限制数值范围（可为负/超界），不可达由 exec::arm_pose 可达域检查拦截并回执。
    bool has_x = params["x"].is<float>() || params["x"].is<int>();
    bool has_h = params["h"].is<float>() || params["h"].is<int>();
    if (has_x && has_h) {
      float x = params["x"] | 0.f;
      float h = params["h"] | 0.f;
      exec::set_move_cap_ms(0);  // 手动接管轮子/转向：先清 AI move 兜底，防旧时限误停
      if (!exec::arm_pose(x, h))
        reply_status(doc, reply, reply_ctx, "arm_pose: 目标不可达(xh 超出机械臂可达范围)");
      // 成功不回执：状态看 exec_log 即知。
    } else {
      reply_status(doc, reply, reply_ctx, "arm_pose: 需要 x=车头系前方cm h=离地高度cm");
    }
    return;
  }

  if (!strcmp(type, "motor")) {
    // 调试直驱：绕过执行板，本板直接驱动哪吒单轮电机（隔离执行板是否异常）。
    // 用法 /motor <n=1..4> <a=0..1000> <b=0..1000>；a=正转 b=反转，都 0 即停。
    int n = params["n"] | -1;
    int a = params["a"] | -1;
    int b = params["b"] | -1;
    if (n >= 1 && n <= 4 && a >= 0 && b >= 0 && a <= 1000 && b <= 1000) {
      exec::set_move_cap_ms(0);  // 手动接管轮子/转向：先清 AI move 兜底，防旧时限在写入窗口误停
      nezha::set_motor((uint8_t)n, (uint16_t)a, (uint16_t)b);
      // 成功不回执：摇杆/按钮高频下发，状态看 exec_log 即知。
    } else {
      reply_status(doc, reply, reply_ctx, "motor: 参数需 n=1..4 a,b=0..1000");
    }
    return;
  }

  if (!strcmp(type, "drive")) {
    // 调试直驱：一键发全车四轮前进/后退/停（单条命令保证四轮同时起转）。
    // 用法 /drive <speed=-1000..1000>；>0 前进 <0 后退 0 停车。
    int spd = params["speed"] | 0;
    if (spd < -1000 || spd > 1000) {
      reply_status(doc, reply, reply_ctx, "drive: speed 需在 -1000..1000");
      return;
    }
    int S = spd < 0 ? -spd : spd;
    uint16_t u = (uint16_t)S;
    bool rev = spd < 0;
    // 手动接管轮子/转向：先清 AI move 兜底，防旧时限在写入窗口误停
    exec::set_move_cap_ms(0);
    // 轮map：M1左后 M2右后 M3右前 M4左前；左轮 a 正前，右轮 b 正前。
    // 前进：M1(S,0) M2(0,S) M3(0,S) M4(S,0)；后退：四轮 a/b 对调。
    nezha::set_motor(1, rev ? 0 : u, rev ? u : 0);
    nezha::set_motor(2, rev ? u : 0, rev ? 0 : u);
    nezha::set_motor(3, rev ? u : 0, rev ? 0 : u);
    nezha::set_motor(4, rev ? 0 : u, rev ? u : 0);
    // 成功不回执：摇杆/按钮高频下发，状态看 exec_log 即知。
    return;
  }

  if (!strcmp(type, "spin")) {
    // 原地转向（普通四轮滑移式，无需特殊轮子）：靠左/右侧轮反向拖胎绕中心旋转。
    // 前提：转向舵回正（steer 居中），前轮保持直行位。
    // dir=+1 左进右退 / -1 左退右进 / 0 停；speed=单车轮 pwm 0..1000（默认 500）。
    // 落地统一走 exec 保持马达映射与内部状态一致（持续判定/回正）。
    int dir = params["dir"] | 0;
    int spd = params["speed"] | 500;
    if (dir < -1 || dir > 1 || spd < 0 || spd > 1000) {
      reply_status(doc, reply, reply_ctx, "spin: dir=±1/0, speed=0..1000");
      return;
    }
    exec::act("spin", params);
    // 成功不回执：摇杆/按钮高频下发，状态看 exec_log 即知。
    return;
  }

  reply_status(doc, reply, reply_ctx, "未知指令类型");
}

void cmd::apply_network() {
  net::reconnect();  // 在线重建 STA，不整板重启（BLE 保活，配网后无需重连）
  blog::logf(blog::NET, "已在线上网生效（未重启）");
}

bool cmd::streaming() { return g_streaming; }

void cmd::set_streaming(bool on) { g_streaming = on; }