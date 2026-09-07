#include "uart.h"
#include "config.h"

// 置 1 时状态帧打印逐字节 payload hex（调试报文用）；默认单行短摘要，避免抢串口。
#define UART_STATE_HEX_DUMP 0

// UART 口：Serial2（实例）。引脚因板而异——若与执行板接线不符，在此 setPins 覆盖：
//   #define UART_TX_PIN 17
//   #define UART_RX_PIN 18
static HardwareSerial& u = Serial2;

// 速度/距离为执行板标定项（架构 §5.5「通用约定」）。词表不带 speed 的 arm 动作用此默认档。
static const uint8_t k_arm_speed = 0x80;

// ---------------- CRC16 (MODBUS, poly 0x8005 reflected, init 0xFFFF) ----------------
// 帧头之后 LEN..PAYLOAD 的 CRC；低字节在前。多项式/覆盖范围以执行板固件联调为准（§5.4）。
static uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

// 执行板状态缓存（0x0A 解析写入，AI 拼接读取；loop 写入 / worker 读取，互斥保护）。
struct ExecState { uint8_t dev, state, speed, grip; uint16_t param; uint8_t flag; bool valid; };
static ExecState g_exec = {};
static SemaphoreHandle_t g_exec_mtx = nullptr;

void uart::send_raw(uint8_t dev, uint8_t cmd, const uint8_t* payload, size_t len) {
  uint8_t frame[2 + 1 + 1 + 1 + 64 + 2];  // 头2 + LEN + DEV + CMD + PAYLOAD(≤64) + CRC2
  if (len > 64) len = 64;
  size_t n = 0;
  frame[n++] = 0xAA;
  frame[n++] = 0x55;
  uint8_t len_byte = 1 + 1 + (uint8_t)len;  // DEV+CMD+PAYLOAD
  frame[n++] = len_byte;
  frame[n++] = dev;
  frame[n++] = cmd;
  memcpy(frame + n, payload, len);
  n += len;
  uint16_t crc = crc16(frame + 2, n - 2);  // 从 LEN 起
  frame[n++] = crc & 0xFF;                 // 低位在前
  frame[n++] = (crc >> 8) & 0xFF;
  u.write(frame, n);
}

// ---------------- 词表 → UART 翻译（§5.3/§5.5） ----------------

static inline float fabsf_(float x) { return x < 0 ? -x : x; }
static inline uint8_t dir_of(float v, uint8_t positive, uint8_t negative) {
  return v >= 0 ? positive : negative;
}
// [-1,1] → [0,255]
static inline uint8_t speed_of(float v) {
  float a = fabsf_(v);
  if (a > 1.0f) a = 1.0f;
  return (uint8_t)(a * 255.0f + 0.5f);
}
static inline uint8_t u16lo(uint16_t v) { return v & 0xFF; }
static inline uint8_t u16hi(uint16_t v) { return (v >> 8) & 0xFF; }

// 小车：throttle / steering 各自按"持续 or 指定距离/角度"编码。二者同非零时分两帧发。
static void send_move(const JsonObjectConst& p) {
  float throttle = p["throttle"] | 0.0f;
  float steering = p["steering"] | 0.0f;
  bool has_dist = p["distance_cm"].is<int>();
  bool has_angle = p["angle_deg"].is<int>();

  if (fabsf_(throttle) > 0.001f) {
    if (has_dist) {
      uint16_t d = (uint16_t)((int)p["distance_cm"] | 0);
      uint8_t pay[] = {dir_of(throttle, 0x01, 0x02), u16lo(d), u16hi(d), speed_of(throttle)};
      uart::send_raw(0x01, 0x02, pay, sizeof(pay));  // 移动指定距离
    } else {
      uint8_t pay[] = {dir_of(throttle, 0x01, 0x02), speed_of(throttle)};
      uart::send_raw(0x01, 0x01, pay, sizeof(pay));  // 持续移动
    }
  }
  if (fabsf_(steering) > 0.001f) {
    if (has_angle) {
      uint16_t a = (uint16_t)((int)p["angle_deg"] | 0);
      uint8_t pay[] = {dir_of(steering, 0x01, 0x02), u16lo(a), u16hi(a), speed_of(steering)};
      uart::send_raw(0x01, 0x04, pay, sizeof(pay));  // 转动指定角度
    } else {
      uint8_t pay[] = {dir_of(steering, 0x01, 0x02), speed_of(steering)};
      uart::send_raw(0x01, 0x03, pay, sizeof(pay));  // 持续转动
    }
  }
}

// 停止（scope: all/wheels/arm → 0x00/0x01/0x02）。架构把 scope 枚举定义在小车 CMD 0x05，
// 以执行板固件对 DEV/scope 的最终解释为准（联调项）。
static void send_stop(const JsonObjectConst& p) {
  const char* scope = p["scope"] | "all";
  uint8_t s = 0;
  if (!strcmp(scope, "wheels")) s = 0x01;
  else if (!strcmp(scope, "arm")) s = 0x02;
  else s = 0x00;  // all
  uint8_t pay[] = {s};
  uart::send_raw(0x01, 0x05, pay, sizeof(pay));
}

// 机械臂：act → 持续/指定距离 升降或移爪、夹取。dist_cm 决定是否"指定距离"版。
static void send_arm(const JsonObjectConst& p) {
  const char* act = p["act"] | "";
  bool has_dist = p["dist_cm"].is<int>();
  uint8_t pay[4];

  if (!strcmp(act, "lift_up") || !strcmp(act, "lift_down")) {
    uint8_t dir = !strcmp(act, "lift_up") ? 0x01 : 0x02;
    if (has_dist) {
      uint16_t d = (uint16_t)((int)p["dist_cm"] | 0);
      pay[0] = dir; pay[1] = u16lo(d); pay[2] = u16hi(d); pay[3] = k_arm_speed;
      uart::send_raw(0x02, 0x02, pay, 4);  // 升降指定距离
    } else {
      pay[0] = dir; pay[1] = k_arm_speed;
      uart::send_raw(0x02, 0x01, pay, 2);  // 持续升降
    }
  } else if (!strcmp(act, "reach_forward") || !strcmp(act, "reach_backward")) {
    uint8_t dir = !strcmp(act, "reach_forward") ? 0x01 : 0x02;
    if (has_dist) {
      uint16_t d = (uint16_t)((int)p["dist_cm"] | 0);
      pay[0] = dir; pay[1] = u16lo(d); pay[2] = u16hi(d); pay[3] = k_arm_speed;
      uart::send_raw(0x02, 0x04, pay, 4);  // 移爪指定距离
    } else {
      pay[0] = dir; pay[1] = k_arm_speed;
      uart::send_raw(0x02, 0x03, pay, 2);  // 持续移爪
    }
  } else if (!strcmp(act, "clip") || !strcmp(act, "release")) {
    pay[0] = !strcmp(act, "clip") ? 0x01 : 0x02;
    uart::send_raw(0x02, 0x05, pay, 1);  // 夹取/松夹
  } else {
    Serial.printf("[uart] arm: 未知 act=%s\n", act);
  }
}

bool uart::act(const char* type, const JsonObjectConst& params) {
  if (!strcmp(type, "move")) { send_move(params); return true; }
  if (!strcmp(type, "stop")) { send_stop(params); return true; }
  if (!strcmp(type, "arm")) { send_arm(params); return true; }
  return false;
}

bool uart::is_continuous(const char* type, const JsonObjectConst& p) {
  if (!strcmp(type, "move")) {
    float th = p["throttle"] | 0.0f;
    float st = p["steering"] | 0.0f;
    bool has_dist = p["distance_cm"].is<int>();
    bool has_angle = p["angle_deg"].is<int>();
    return (fabsf_(th) > 0.001f && !has_dist) || (fabsf_(st) > 0.001f && !has_angle);
  }
  if (!strcmp(type, "arm")) {
    const char* act_ = p["act"] | "";
    bool cont_act = !strcmp(act_, "lift_up") || !strcmp(act_, "lift_down") ||
                    !strcmp(act_, "reach_forward") || !strcmp(act_, "reach_backward");
    return cont_act && !p["dist_cm"].is<int>();
  }
  return false;
}

// ---------------- 初始化 / RX（状态帧解析骨架） ----------------

void uart::init() {
  g_exec_mtx = xSemaphoreCreateMutex();
  uint32_t baud = cfg::uart_baud();
#if defined(UART_TX_PIN) && defined(UART_RX_PIN)
  u.begin(baud, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
#else
  u.begin(baud);
#endif
  Serial.printf("[uart] Serial2 @ %lu\n", (unsigned long)baud);
}

// 帧接收：攒 `AA 55 LEN` → 收满 LEN+CRC → 校验 → 若是状态帧(CMD 0x0A)记日志（骨架）。
// 上行解析供手机查询 / AI 观察用；执行板未接入时无输出。
void uart::update() {
  static uint8_t buf[70];
  static size_t n = 0;
  while (u.available()) {
    uint8_t b = (uint8_t)u.read();
    if (n == 0 && b != 0xAA) continue;      // 等帧头
    if (n == 1 && b != 0x55) { n = 0; continue; }
    if (n < sizeof(buf)) buf[n++] = b;

    if (n >= 3 && buf[1] == 0x55) {
      uint8_t len = buf[2];                 // DEV+CMD+PAYLOAD
      size_t total = 3 + len + 2;           // 头(AA55LEN) + 内容 + CRC2
      if (n >= total) {
        uint16_t got = buf[total - 2] | (buf[total - 1] << 8);
        uint16_t calc = crc16(buf + 2, len + 1);
        if (got == calc) {
          uint8_t dev = buf[3], cmd = buf[4];
          if (cmd == 0x0A) {
#if UART_STATE_HEX_DUMP
            Serial.printf("[uart] 状态帧 dev=%02X payload=", dev);
            for (size_t i = 5; i < 3 + 1 + len; i++) Serial.printf("%02X ", buf[i]);
            Serial.println();
#else
            // 单行短摘要：状态帧周期性上报，逐字节 hex 会占满串口缓冲、阻塞并发的手动 ack 打印。
            // 需看 payload 时临时把 UART_STATE_HEX_DUMP 置 1。
            Serial.printf("[uart] 状态帧 dev=%02X state=%u\n", dev, (unsigned)buf[5]);
#endif
          } else {
            Serial.printf("[uart] 收到帧 dev=%02X cmd=%02X（非状态帧，忽略）\n", dev, cmd);
          }
        } else {
          Serial.printf("[uart] CRC 校验失败\n");
        }
        n = 0;
      }
    }
  }
}

// 读取最近一条执行板状态帧并拼成一行文本（供 AI 上下文）。无数据返回 false。
bool uart::read_state(char* buf, size_t cap) {
  ExecState s;
  if (g_exec_mtx) xSemaphoreTake(g_exec_mtx, portMAX_DELAY);
  s = g_exec;
  if (g_exec_mtx) xSemaphoreGive(g_exec_mtx);
  if (!s.valid) return false;
  if (s.dev == 0x01) {  // 小车：state 0停止/1移动中/2转动中
    const char* st = s.state == 1 ? "移动中" : (s.state == 2 ? "转动中" : "停止");
    snprintf(buf, cap, "小车:%s 距%ucm%s", st, (unsigned)s.param, (s.flag & 1) ? " 打滑" : "");
  } else {              // 机械臂：state 0空闲/1升降中/2移爪中；grip 0松/1夹
    const char* st = s.state == 1 ? "升降中" : (s.state == 2 ? "移爪中" : "空闲");
    snprintf(buf, cap, "机械臂:%s 夹爪:%s 距%ucm%s",
             st, (s.grip & 1) ? "夹住" : "松开", (unsigned)s.param, (s.flag & 1) ? " 故障" : "");
  }
  return true;
}
