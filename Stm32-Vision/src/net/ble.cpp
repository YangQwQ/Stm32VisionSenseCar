#include "src/net/ble.h"
#include "src/core/command.h"
#include "src/core/board_log.h"
#include "src/net/config.h"
#include "src/net/wifi_net.h"
#include "src/ai/ai_client.h"

#include <ArduinoJson.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// xQueue* / QueueHandle_t：Bluedroid 头会间接引入，但显式声明避免跨核心差异
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// 放开本地 MTU：本 core 用 NimBLE 后端（sdkconfig CONFIG_BT_NIMBLE_ENABLED=y，preferred MTU=256）。
// 手机 gdble 连接后即请求协商较大 MTU，若不把 server 侧 preferred MTU 显式设上，NimBLE 默认响应
// 到 23 → 手机 status=133 "MTU negotiation failed" → GATT 未就绪 → 连接判失败断开。
// 头文件已被 BLEDevice.h 间接引入（ble_att_set_preferred_mtu 返回 int，忽略返回值即可）。
#include "host/ble_att.h"

// UUID（与 Ctrl-App net/ble/BleProfile.gd 逐字 mirror——改一侧必须同步另一侧）。
// 服务 0000C0DE-…，特征 c0e0(ssid) c0e1(pass) c0e2(ai_url) c0e3(ai_key)
//       c0e4(ai_model) c0e5(cmd) c0e6(status: 读/通知)。广播名 VisionS3。
static const char* k_svc = "0000C0DE-0000-1000-8000-00805F9B34FB";
static const char* k_ssid = "0000C0E0-0000-1000-8000-00805F9B34FB";
static const char* k_pass = "0000C0E1-0000-1000-8000-00805F9B34FB";
static const char* k_ai_url = "0000C0E2-0000-1000-8000-00805F9B34FB";
static const char* k_ai_key = "0000C0E3-0000-1000-8000-00805F9B34FB";
static const char* k_ai_model = "0000C0E4-0000-1000-8000-00805F9B34FB";
static const char* k_cmd = "0000C0E5-0000-1000-8000-00805F9B34FB";
static const char* k_status = "0000C0E6-0000-1000-8000-00805F9B34FB";

static BLEServer* g_server = nullptr;
static BLECharacteristic* g_status_char = nullptr;
static bool g_ws_connected = false;   // 仅 WS 客户端在位（status.ws 字段）
static bool g_transmission = false;   // 任一图传通道活跃（WS 客户端/MJPEG/streaming 标志）
static bool g_last_net = false;

// cmd JSON 队列：GATT 写回调入队，ble::update()（loop 上下文）取出统一派发。
// 这样 cmd 处理 / uart 发送 / 延迟重启都在主循环上下文执行，与 WS 通道串行。
static QueueHandle_t g_q = nullptr;

static void enqueue(const char* text) {
  if (!g_q) return;
  size_t n = strlen(text);
  char* copy = (char*)malloc(n + 1);
  if (!copy) return;
  memcpy(copy, text, n + 1);
  // 队列满则丢弃（写频率极低，理论不触发）
  if (xQueueSend(g_q, &copy, 0) != pdTRUE) free(copy);
}

// ---------------- 应答：status 特征通知 ----------------

// 用 ArduinoJson 构造，自动转义 reply（内含引号/反斜杠的 JSON 字符串）。先前手拼 String
// 原样拼接 reply 导致其内双引号破坏最外层 JSON，手机端 parse 失败 → 纯蓝牙下所有经 reply
// 回执的指令（config/exec_log/light/reset/ai_cancel…）都"没有反馈"。
static String build_status(const char* reply) {
  JsonDocument d;
  d["ip"] = net::is_connected() ? net::local_ip().toString().c_str() : "";
  d["wifi_ssid"] = cfg::wifi_ssid();
  d["ws"] = g_ws_connected;
  d["ai_busy"] = ai::busy();
  if (reply && reply[0]) d["reply"] = reply;
  String s;
  serializeJson(d, s);
  return s;
}

// 按 ≤MTU 的片多次 notify（手机端按"能解析出完整 JSON"拼接）。NimBLE 单包 notify 只发
// MTU-3 字节，超长会截断导致手机 Parse JSON failed。配合 ble_att_set_preferred_mtu(256)
// 后单片 200B 安全；若协议栈仍未升 MTU（保守 23），长 status 仍会被截——需先烧含该调用的固件。
static void notify_raw(const char* s) {
  if (!g_status_char) return;
  const size_t chunk = 200;
  size_t len = strlen(s), off = 0;
  if (len == 0) { g_status_char->setValue((uint8_t*)"{}", 2); g_status_char->notify(); return; }
  while (off < len) {
    size_t n = (len - off) > chunk ? chunk : (len - off);
    g_status_char->setValue((uint8_t*)s + off, n);
    g_status_char->notify();
    off += n;
    if (off < len) vTaskDelay(pdMS_TO_TICKS(2));  // 给收端缓冲时间，避免连发丢片
  }
}

static void notify_status(const char* reply) {
  String s = build_status(reply);
  notify_raw(s.c_str());
}

// ---------------- GATT 回调 ----------------

class CharCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();  // core 3.x：getValue() 返回 Arduino String
    const char* data = v.c_str();
    if (BLEUUID(c->getUUID()).equals(BLEUUID(k_cmd))) {
      enqueue(data);  // 词表 JSON，交给 cmd（loop 上下文统一处理）
    } else if (BLEUUID(c->getUUID()).equals(BLEUUID(k_ssid)) ||
               BLEUUID(c->getUUID()).equals(BLEUUID(k_pass))) {
      // 配网（BleProfile provision 走 SSID/PASS 两特征）：先收齐再落 NVS
      static String s_ssid, s_pass;
      if (BLEUUID(c->getUUID()).equals(BLEUUID(k_ssid))) s_ssid = v;
      else s_pass = v;
      if (!s_ssid.isEmpty() && !s_pass.isEmpty()) {
        bool changed = cfg::set_wifi(s_ssid, s_pass);
        s_ssid = "";
        s_pass = "";
        if (changed) {
          cmd::apply_network();  // 在线重建 STA（不重启，BLE 保持连接）
          blog::logf(blog::BLE, "wifi 已更新，正在连接");
        } else {
          blog::logf(blog::BLE, "wifi 未变化，跳过");
        }
      }
    } else if (BLEUUID(c->getUUID()).equals(BLEUUID(k_ai_url)) ||
               BLEUUID(c->getUUID()).equals(BLEUUID(k_ai_key)) ||
               BLEUUID(c->getUUID()).equals(BLEUUID(k_ai_model))) {
      // 手机只写非空字段（BleProfile.write_ai_config 逐特征写）。
      // 用 cfg 现值做底、逐字段覆盖，避免用空串清掉其余已存配置。
      static String a_url, a_key, a_model;
      static bool seeded = false;
      if (!seeded) {
        a_url = cfg::ai_url();
        a_key = cfg::ai_key();
        a_model = cfg::ai_model();
        seeded = true;
      }
      if (BLEUUID(c->getUUID()).equals(BLEUUID(k_ai_url))) a_url = v;
      else if (BLEUUID(c->getUUID()).equals(BLEUUID(k_ai_key))) a_key = v;
      else a_model = v;
      cfg::set_ai(a_url, a_key, a_model);
      blog::logf(blog::BLE, "ai 配置已写");
    }
  }
  void onRead(BLECharacteristic* c) override {
    if (BLEUUID(c->getUUID()).equals(BLEUUID(k_status))) {
      String s = build_status("");
      c->setValue((uint8_t*)s.c_str(), s.length());
    }
  }
};

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    blog::logf(blog::BLE, "手机已连接");
    // 连上后主动推一次状态（含 IP），供手机配网闭环 / 兜底观察
    notify_status("");
  }
  void onDisconnect(BLEServer* s) override {
    bool adv = BLEDevice::getAdvertising()->isAdvertising();
    // 任一图传通道活跃（WS 客户端/MJPEG/streaming）时保持低调，不恢复广播以免抢 WiFi 射频；
    // 否则(纯兜底/配网)恢复可发现，供再次连接。
    if (s && !g_transmission) s->startAdvertising();
    blog::logf(blog::BLE, "手机断开 ws=%d adv=%d", g_ws_connected ? 1 : 0, adv);
  }
};

// ---------------- 对外 ----------------

void ble::init() {
  if (g_server) return;
  BLEDevice::init("VisionS3");  // 广播名，手机扫描按 name 过滤
  BLEDevice::setPower(ESP_PWR_LVL_P9);  // core 3.x 发射功率枚举改为 _P9 封顶（对应 +9dBm）
  // 放开本地 MTU：手机 gdble 连接后即请求协商较大 MTU，不设则 NimBLE 响应到 23，
  // 手机收到 status=133 → "MTU negotiation failed" → GATT 未就绪 → 连接判失败断开。
  ble_att_set_preferred_mtu(256);

  g_q = xQueueCreate(8, sizeof(char*));

  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());
  BLEService* svc = g_server->createService(BLEUUID(k_svc));

  // 配网/兜底写特征（wifi / ai / cmd）
  auto addWriteChar = [&](const char* uuid) {
    BLECharacteristic* c = svc->createCharacteristic(
        BLEUUID(uuid), BLECharacteristic::PROPERTY_WRITE);
    c->setCallbacks(new CharCB());
  };
  addWriteChar(k_ssid);
  addWriteChar(k_pass);
  addWriteChar(k_ai_url);
  addWriteChar(k_ai_key);
  addWriteChar(k_ai_model);
  addWriteChar(k_cmd);

  // 状态特征：读 + 通知（需 CCCD 描述符）
  g_status_char = svc->createCharacteristic(
      BLEUUID(k_status),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  g_status_char->addDescriptor(new BLE2902());
  g_status_char->setCallbacks(new CharCB());

  svc->start();

  // 广播：带服务 UUID + 设备名，持续可发现
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID(k_svc));
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  blog::logf(blog::BLE, "GATT server 就绪，广播中 (VisionS3)");
  g_last_net = net::is_connected();
}

void ble::reply(const char* text) {
  notify_status(text);
}

void ble::send_status(const char* text) {
  notify_raw(text);  // 直接推一段独立 JSON（周期状态 /log exec），不套 build_status 的 reply 包装
}

void ble::set_ws_connected(bool on) {
  // 仅维护 status.ws 上报；广播开关统一由 set_transmission 控制（图传活跃即停广播）。
  if (g_ws_connected == on) return;
  g_ws_connected = on;
  notify_status("");  // 状态变化即上报（ws 字段刷新）
}

void ble::set_transmission(bool on) {
  // BLE 与 WiFi 共用 2.4G 射频：广播开启会明显压低 WiFi 吞吐（手机离线但电脑 MJPEG 在推时
  // 单帧能从 ~100ms 恶化到 ~400ms）。因此任一图传通道活跃即停广播，全部安静再恢复可发现。
  if (g_transmission == on) return;
  g_transmission = on;
  bool before = BLEDevice::getAdvertising()->isAdvertising();
  if (on) { if (before) BLEDevice::stopAdvertising(); }
  else    { if (!before) BLEDevice::startAdvertising(); }
  bool after = BLEDevice::getAdvertising()->isAdvertising();
  if (before != after)
    blog::logf(blog::BLE, "图传=%s 广播%s", on ? "on" : "off", after ? "已启动" : "已停止");
}

void ble::update() {
  // 1) 处理 GATT 写回调积压的 cmd JSON
  char* item = nullptr;
  while (g_q && xQueueReceive(g_q, &item, 0) == pdTRUE) {
    if (item) {
      cmd::handle(item, false, [](void*, const char* t) { ble::reply(t); }, nullptr);
      free(item);
    }
  }
  // 2) WiFi 连接状态变化 → 重报状态（配网后上线 / 断线兜底）
  bool on = net::is_connected();
  if (on != g_last_net) {
    g_last_net = on;
    notify_status(on ? "WiFi 已连接" : "WiFi 已断开");
  }
}
