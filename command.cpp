#include "command.h"
#include "config.h"
#include "uart.h"
#include "ai_client.h"

// 应答格式遵循架构 §5.1：板 → 手机文本 = {type:status/pong, params:{...}, id:<回填>}。
// move/stop/arm 是高频手动指令，只在 UART 层记录，不回文本（避免刷屏）。

static unsigned long g_restart_at = 0;  // 配置变更后的重启时刻（0=未调度）
static bool g_streaming = false;         // 图传开关全局状态（WS 推流任务读取）

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

void cmd::handle(const char* json, bool has_frames, ReplyFn reply, void* reply_ctx) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) {
    Serial.printf("[cmd] bad json: %s\n", json);
    return;
  }
  const char* type = doc["type"] | "";
  JsonObject params = doc["params"].as<JsonObject>();
  bool manual = !strcmp(type, "move") || !strcmp(type, "stop") || !strcmp(type, "arm");
  if (!manual)  // 手动指令高频，不逐条打印
    Serial.printf("[cmd] type=%s has_frames=%u\n", type, has_frames);

  if (manual) {
    // 手动/词表动作：优先打断 AI 闭环，再立即译帧下发执行板（不文本应答）。
    // arm 打断只停轮子（机械臂指令即接管）；move/stop 由用户指令覆盖，不补停。
    ai::cancel(!strcmp(type, "arm") ? ai::StopMode::Wheels : ai::StopMode::None);
    uart::act(type, params);
    return;
  }

  if (!strcmp(type, "config")) {
    // 配网（BLE 或 WS 同结构）：落 NVS → 重启连 WiFi → 上线后 BLE 上报 IP
    const char* ssid = params["ssid"] | "";
    const char* pass = params["password"] | "";
    if (!ssid[0]) {
      reply_status(doc, reply, reply_ctx, "config: SSID 为空");
      return;
    }
    cfg::set_wifi(ssid, pass);
    if (g_restart_at == 0) g_restart_at = millis() + 1000;
    reply_status(doc, reply, reply_ctx, "WiFi 配置已保存，重启连接…");
    return;
  }

  if (!strcmp(type, "ping")) {
    reply_pong(doc, reply, reply_ctx);
    return;
  }

  if (!strcmp(type, "stream")) {
    // 图传开关：更新全局状态，WS 推流任务读取。BLE 无帧通道也能开/关图传。
    bool on = params["on"] | false;
    set_streaming(on);
    reply_status(doc, reply, reply_ctx, on ? "图传已开启" : "图传已关闭");
    return;
  }

  if (!strcmp(type, "snapshot")) {
    // 抓帧需 WS 通道（带帧/权限）；BLE 无此能力，如实引导。
    if (has_frames) {
      Serial.printf("[cmd] snapshot 由 WS 层处理\n");
    } else {
      reply_status(doc, reply, reply_ctx, "截图需 WiFi（BLE 仅为兜底控制），请连上 WS 后使用");
    }
    return;
  }

  if (!strcmp(type, "ai_goal")) {
    // DIRECT：手机下发目标 → 板载 ai_client 调 AI（迭代闭环，中途可被新目标/手动打断）。
    if (cfg::ai_key().isEmpty()) {
      reply_status(doc, reply, reply_ctx, "AI 未配置（ai_key 为空），未调用云端");
      return;
    }
    const char* msg = params["message"] | "";
    bool use_image = params["use_image"] | false;

    // 组标注字符串（供 AI 观察近似意图区域）
    String ann;
    if (params["annotation"].is<JsonObject>()) {
      serializeJson(params["annotation"].as<JsonObjectConst>(), ann);
    }

    // WS 异步回复需堆拷贝 fd（ai_client 任务结束时释放）；BLE 传 nullptr。
    void* actx = nullptr;
    if (reply_ctx) actx = new int(*(int*)reply_ctx);
    long id = has_id(doc) ? doc["id"].as<long>() : 0;
    ai::set_goal(msg, use_image, ann.length() ? ann.c_str() : nullptr, id, reply, actx);
    reply_status(doc, reply, reply_ctx, "已收到目标，AI 处理中");
    return;
  }

  if (!strcmp(type, "ai_cancel")) {
    // 显式取消 AI 任务：残留持续指令会在任务出口补停（≠强制停车）。
    ai::cancel(ai::StopMode::All);
    reply_status(doc, reply, reply_ctx, "AI 任务已取消");
    return;
  }

  reply_status(doc, reply, reply_ctx, "未知指令类型");
}

void cmd::schedule_restart() {
  if (g_restart_at == 0) g_restart_at = millis() + 1000;
}

bool cmd::streaming() { return g_streaming; }

void cmd::set_streaming(bool on) { g_streaming = on; }

void cmd::update() {
  if (g_restart_at != 0 && (long)(millis() - g_restart_at) >= 0) {
    g_restart_at = 0;
    Serial.printf("[cmd] 重启生效 WiFi 配置\n");
    ESP.restart();
  }
}
