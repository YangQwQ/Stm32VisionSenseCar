#include "command.h"
#include "config.h"
#include "uart.h"

// 应答格式遵循架构 §5.1：板 → 手机文本 = {type:status/pong, params:{...}, id:<回填>}。
// move/stop/arm 是高频手动指令，只在 UART 层记录，不回文本（避免刷屏）。

static unsigned long g_restart_at = 0;  // 配置变更后的重启时刻（0=未调度）

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
    // 手动/词表动作：立即译帧下发执行板（含 AI 返回的动作）。不文本应答。
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

  if (!strcmp(type, "snapshot") || !strcmp(type, "stream")) {
    // WS 通道在 app_httpd 已就地处理（带 fd/帧）；这里兜住 BLE 等无帧通道
    if (has_frames) {
      Serial.printf("[cmd] snapshot/stream 由 WS 层处理\n");
    } else {
      reply_status(doc, reply, reply_ctx,
                   "图传/截图需 WiFi（BLE 仅为兜底控制），请连上 WS 后使用");
    }
    return;
  }

  if (!strcmp(type, "ai_goal")) {
    // DIRECT：手机下发目标文本 → 本应经 ai_client 调板载云端 AI。
    // 本轮云端 AI 未接线（ai_url/key 空或未实现），如实回状态，不假装执行。
    const char* msg = params["message"] | "";
    String reason;
    if (cfg::ai_url().isEmpty() || cfg::ai_key().isEmpty()) {
      reason = "收到目标，但 AI 未配置（ai_url/ai_key 为空），未调用云端";
    } else {
      reason = "收到目标，板载 AI 尚未接通（本轮桩）";
    }
    Serial.printf("[cmd] ai_goal: %s\n", msg);
    reply_status(doc, reply, reply_ctx, reason.c_str());
    return;
  }

  reply_status(doc, reply, reply_ctx, "未知指令类型");
}

void cmd::schedule_restart() {
  if (g_restart_at == 0) g_restart_at = millis() + 1000;
}

void cmd::update() {
  if (g_restart_at != 0 && (long)(millis() - g_restart_at) >= 0) {
    g_restart_at = 0;
    Serial.printf("[cmd] 重启生效 WiFi 配置\n");
    ESP.restart();
  }
}
