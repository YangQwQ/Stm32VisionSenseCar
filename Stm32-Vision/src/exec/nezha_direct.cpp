#include "src/exec/nezha_direct.h"

// 哪吒 I2C 引脚（本板空闲脚，见 CLAUDE.md 引脚占用）。SCL=47 / SDA=14。
#define NZ_SCL  47
#define NZ_SDA  14

#define NZ_ADDR     0x80
#define NZ_UNIT_US  3   // ≥2us，确保 I2C ≤200kHz（哪吒上限）

// 各舵机 SET 命令字节（NeZha.h）：Servo1..4 → 0x22/0x25/0x28/0x2B
static const uint8_t k_set_cmd[4] = { 0x22, 0x25, 0x28, 0x2B };
// 各电机 SET 命令字节（NeZha.h）：Motor1..4 → 0x05/0x09/0x0D/0x11
static const uint8_t k_motor_set[4] = { 0x05, 0x09, 0x0D, 0x11 };

static void sda_hi(void) { digitalWrite(NZ_SDA, 1); }
static void sda_lo(void) { digitalWrite(NZ_SDA, 0); }
static void scl_hi(void) { digitalWrite(NZ_SCL, 1); }
static void scl_lo(void) { digitalWrite(NZ_SCL, 0); }

static void i2c_start(void) {
  sda_hi(); scl_hi();
  sda_lo(); delayMicroseconds(NZ_UNIT_US);
  scl_lo();
}

static void i2c_stop(void) {
  sda_lo(); scl_hi();
  delayMicroseconds(NZ_UNIT_US);
  sda_hi();
}

// 逐位发一字节；末位后再给一个时钟位（从机可能应 ACK，这里不校验，照抄执行板软 I2C）。
static void send_byte(uint8_t b) {
  for (int i = 0; i < 8; i++) {
    if (b & (0x80 >> i)) sda_hi(); else sda_lo();
    scl_hi(); delayMicroseconds(NZ_UNIT_US);
    scl_lo(); delayMicroseconds(NZ_UNIT_US);
  }
  scl_hi(); delayMicroseconds(NZ_UNIT_US);
  scl_lo(); delayMicroseconds(NZ_UNIT_US);
}

static bool s_inited = false;

void nezha::init(void) {
  if (s_inited) return;
  pinMode(NZ_SCL, OUTPUT);
  pinMode(NZ_SDA, OUTPUT);
  scl_hi();
  sda_hi();
  delayMicroseconds(500);   // 上电等哪吒起稳
  s_inited = true;
}

bool nezha::set_servo(uint8_t channel, uint16_t pwm) {
  if (channel < 1 || channel > 4 || pwm < 50 || pwm > 250) return false;
  init();

  uint8_t cmd = k_set_cmd[channel - 1];
  // 与执行板 NeZha_ServoN_SetPwm 完全一致：先 [0x80,0x00,cmd]，再 [0x80,cmd,hi,lo]。
  i2c_start(); send_byte(NZ_ADDR); send_byte(0x00); send_byte(cmd); i2c_stop();
  delayMicroseconds(10);
  i2c_start(); send_byte(NZ_ADDR); send_byte(cmd);
  send_byte((uint8_t)(pwm >> 8)); send_byte((uint8_t)(pwm & 0xFF));
  i2c_stop();
  return true;
}

static bool s_motor_inited = false;

bool nezha::set_motor(uint8_t channel, uint16_t a, uint16_t b) {
  if (channel < 1 || channel > 4 || a > 1000 || b > 1000) return false;
  init();

  // 首次驱动电机前发一次 MOTOR_INIT（0x01），与执行板上电初始化对齐。
  if (!s_motor_inited) {
    i2c_start(); send_byte(NZ_ADDR); send_byte(0x00); send_byte(0x01); i2c_stop();
    delayMicroseconds(10);
    s_motor_inited = true;
  }

  uint8_t cmd = k_motor_set[channel - 1];
  // 与执行板 NeZha_MotorN_SetPwm 完全一致：先 [0x80,0x00,cmd]，再 [0x80,cmd,aH,aL,bH,bL]。
  i2c_start(); send_byte(NZ_ADDR); send_byte(0x00); send_byte(cmd); i2c_stop();
  delayMicroseconds(10);
  i2c_start(); send_byte(NZ_ADDR); send_byte(cmd);
  send_byte((uint8_t)(a >> 8)); send_byte((uint8_t)(a & 0xFF));
  send_byte((uint8_t)(b >> 8)); send_byte((uint8_t)(b & 0xFF));
  i2c_stop();
  return true;
}

// 灯光：与执行板 WriteCommand 一致，单帧 [0x80,0x00,cmd]。
static void led_frame(uint8_t cmd) {
  init();
  i2c_start(); send_byte(NZ_ADDR); send_byte(0x00); send_byte(cmd); i2c_stop();
}

// ================= I2C 只读探测（诊断用） =================
// 发地址字节（写 0x80 / 读 0x81）的第 9 个时钟：把 SDA 转输入上拉读 ACK 位（从机应答拉低=ACK）。
static bool send_addr_ack(uint8_t ab) {
  for (int i = 0; i < 8; i++) {
    if (ab & (0x80 >> i)) sda_hi(); else sda_lo();
    scl_hi(); delayMicroseconds(NZ_UNIT_US);
    scl_lo(); delayMicroseconds(NZ_UNIT_US);
  }
  pinMode(NZ_SDA, INPUT_PULLUP);          // 第 9 时钟释放 SDA，从机可应答
  scl_hi(); delayMicroseconds(NZ_UNIT_US);
  bool ack = (digitalRead(NZ_SDA) == LOW); // 低=ACK
  scl_lo(); delayMicroseconds(NZ_UNIT_US);
  pinMode(NZ_SDA, OUTPUT); sda_hi();       // 恢复驱动
  return ack;
}

// 读 1 字节（SDA 保持输入，主控 NACK 收尾）：尽力返回 8 位值。
static bool read_byte_nack(uint8_t* out) {
  pinMode(NZ_SDA, INPUT_PULLUP);
  uint8_t v = 0;
  for (int i = 0; i < 8; i++) {
    scl_hi(); delayMicroseconds(NZ_UNIT_US);
    v = (uint8_t)((v << 1) | (digitalRead(NZ_SDA) ? 1u : 0u));
    scl_lo(); delayMicroseconds(NZ_UNIT_US);
  }
  pinMode(NZ_SDA, OUTPUT); sda_hi();       // 主控发 NACK（保持高）
  scl_hi(); delayMicroseconds(NZ_UNIT_US);
  scl_lo(); delayMicroseconds(NZ_UNIT_US);
  pinMode(NZ_SDA, OUTPUT); sda_hi();
  *out = v;
  return true;
}

uint8_t nezha::probe(uint8_t* lb) {
  init();
  bool wack = false, rack = false;
  uint8_t data = 0xFF;
  i2c_start();
  wack = send_addr_ack(NZ_ADDR);           // 写地址探测
  i2c_stop();
  i2c_start();
  rack = send_addr_ack(NZ_ADDR | 0x01);    // 读地址探测
  if (rack) read_byte_nack(&data);
  i2c_stop();
  pinMode(NZ_SDA, OUTPUT); sda_hi();       // 复位总线到空闲高
  pinMode(NZ_SCL, OUTPUT); scl_hi();
  if (lb) *lb = data;
  return (uint8_t)((wack ? 1u : 0u) | (rack ? 2u : 0u));
}

// 前灯 0x2D/0x2E；氛围灯 0x36/0x37；尾灯=左右一起 0x30/0x31 + 0x33/0x34。
bool nezha::led(const char* kind, bool on) {
  if (!strcmp(kind, "front")) { led_frame(on ? 0x2D : 0x2E); return true; }
  if (!strcmp(kind, "vibe"))  { led_frame(on ? 0x36 : 0x37); return true; }
  if (!strcmp(kind, "back")) {
    led_frame(on ? 0x30 : 0x31);
    delayMicroseconds(5);
    led_frame(on ? 0x33 : 0x34);
    return true;
  }
  return false;
}