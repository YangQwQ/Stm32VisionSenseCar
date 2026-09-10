#include "command.h"
#include "config.h"
#include "wifi_net.h"
#include "ai_client.h"
#include "direct_exec.h"
#include "ping_svc.h"
#include "nezha_direct.h"

// 应答格式遵循架构 §5.1：板 → 手机文本 = {type:status/pong, params:{...}, id:<回填>}。
// move/stop/arm 是高频手动指令，只在 UART 层记录，不回文本（避免刷屏）。

static bool g_streaming = false;         // 图传开关全局状态（WS 推流任务读取）
static bool g_exec_log = false;          // exec_log 开关（默认关；开启后周期性推本地状态）

static bool has_id(const JsonDocument& doc) {
  return doc["id"].is<int>() || doc["id"].is<long>();
}

// 组一段带原 id 的应答文本并发出（reply 可空）。
static void reply_status(JsonDocument& src, cmd::ReplyFn reply, void* ctx,
                         const char* reason) {
  if (!reply) {
    Serial.printf("[cmd] (无回复通道) %s\n", reason);
    return;
  }
  JsonDocument out;
  out["type"] = "status";
  JsonObject params = out["params"].to<JsonObject>();
  params["reason"] = reason;
  if (has_id(src)) out["id"] = src["id"].as<long>();
  String s;
  serializeJson(out, s);
  reply(ctx, s.c_str());
}

// 与手机 /ping 对齐：回 {type:pong}（手机 WSCarClient 对 pong 直接读文本）。
static void reply_pong(JsonDocument& src, cmd::ReplyFn reply, void* ctx) {
  if (!reply) return;
  JsonDocument out;
  out["type"] = "pong";
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
    Serial.printf("[cmd] 收到手动指令 %s\n", type);
  }
}

void cmd::handle(const char* json, bool has_frames, ReplyFn reply, void* reply_ctx) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) {
    Serial.printf("[cmd] bad json: %s\n", json);
    return;
  }
  const char* type = doc["type"] | "";
  JsonObject params = doc["params"].as<JsonObject>();
  bool manual = !strcmp(type, "move") || !strcmp(type, "stop") || !strcmp(type, "arm");
  if (manual) {
    // 手动高频指令：先立即打断 AI 闭环并直接驱动哪吒扩展板（电机控制优先），
    // 串口日志只在类型切换时打一行，绝不让日志阻塞控制时序。
    // move/stop/arm 由本板直驱，不再经执行板。arm 打断只停轮子（机械臂指令即接管）。
    ai::cancel(!strcmp(type, "arm") ? ai::StopMode::Wheels : ai::StopMode::None);
    exec::act(type, params);
    log_manual_throttled(type);
    return;
  }

  Serial.printf("[cmd] type=%s has_frames=%u\n", type, has_frames);

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
    const char* target = params["target"] | "";
    if (target[0]) {
      ping::start(target, 5, reply, reply_ctx);
      String tip = "正在 ping " + String(target) + "…";
      reply_status(doc, reply, reply_ctx, tip.c_str());
    } else {
      reply_pong(doc, reply, reply_ctx);
    }
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

  if (!strcmp(type, "exec_log")) {
    // 本地直驱状态实时推送开关（默认关，避免空闲刷屏）：开启后 app_httpd 周期性推
    // exec::read_state 合成的状态文本给手机，替代原执行板上行帧镜像。
    bool on = params["on"] | false;
    set_exec_log(on);
    reply_status(doc, reply, reply_ctx, on ? "状态实时推送已开启" : "状态实时推送已关闭");
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

  if (!strcmp(type, "servo")) {
    // 调试直驱：绕过执行板，本板软件 I2C 直接驱动哪吒舵机（逻辑通道 n=0转向/1左(前后)/2右(抬落)/3前(夹爪)）。
    // 直接写原始 pwm（50..250），不过标定限位，用于探机械极限/标定；走 exec::set_servo 同步内部状态。
    int n = params["n"] | -1;
    int p = params["pwm"] | -1;
    if (n >= 0 && n <= 3 && p >= 50 && p <= 250 && exec::set_servo((uint8_t)n, (uint16_t)p)) {
      reply_status(doc, reply, reply_ctx, "servo ok");
    } else {
      reply_status(doc, reply, reply_ctx, "servo: n=0转向/1左/2右/3前 pwm=50..250(可超标定限位)");
    }
    return;
  }

  if (!strcmp(type, "motor")) {
    // 调试直驱：绕过执行板，本板直接驱动哪吒单轮电机（隔离执行板是否异常）。
    // 用法 /motor <n=1..4> <a=0..1000> <b=0..1000>；a=正转 b=反转，都 0 即停。
    int n = params["n"] | -1;
    int a = params["a"] | -1;
    int b = params["b"] | -1;
    if (n >= 1 && n <= 4 && a >= 0 && b >= 0 && a <= 1000 && b <= 1000 &&
        nezha::set_motor((uint8_t)n, (uint16_t)a, (uint16_t)b)) {
      reply_status(doc, reply, reply_ctx, "motor ok");
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
    // 轮map：M1左后 M2右后 M3右前 M4左前；左轮 a 正前，右轮 b 正前。
    // 前进：M1(S,0) M2(0,S) M3(0,S) M4(S,0)；后退：四轮 a/b 对调。
    nezha::set_motor(1, rev ? 0 : u, rev ? u : 0);
    nezha::set_motor(2, rev ? u : 0, rev ? 0 : u);
    nezha::set_motor(3, rev ? u : 0, rev ? 0 : u);
    nezha::set_motor(4, rev ? 0 : u, rev ? u : 0);
    reply_status(doc, reply, reply_ctx, "drive ok");
    return;
  }

  reply_status(doc, reply, reply_ctx, "未知指令类型");
}

void cmd::apply_network() {
  net::reconnect();  // 在线重建 STA，不整板重启（BLE 保活，配网后无需重连）
  Serial.println("[cmd] 已在线上网生效（未重启）");
}

bool cmd::streaming() { return g_streaming; }

void cmd::set_streaming(bool on) { g_streaming = on; }

bool cmd::exec_log() { return g_exec_log; }

void cmd::set_exec_log(bool on) { g_exec_log = on; }