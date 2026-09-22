#include <Arduino.h>  // psramFound / Serial / pinMode
#include "src/cam/camera.h"    // 内含 CAMERA_MODEL_* 与 camera_pins.h
#include "src/cam/camera_pins.h"
#include "src/core/board_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace cam {

static bool s_ready = false;

// JPEG 软解码互斥（见 camera.h 说明）。惰性创建：首次解码器调用前任何时刻可用。
static SemaphoreHandle_t s_jpeg_dec_mtx = nullptr;

static void ensure_jpeg_dec_mtx() {
  if (!s_jpeg_dec_mtx) s_jpeg_dec_mtx = xSemaphoreCreateMutex();
}

void lock_jpeg_dec() {
  if (!s_jpeg_dec_mtx) ensure_jpeg_dec_mtx();
  if (s_jpeg_dec_mtx) xSemaphoreTake(s_jpeg_dec_mtx, portMAX_DELAY);
}

void unlock_jpeg_dec() {
  if (s_jpeg_dec_mtx) xSemaphoreGive(s_jpeg_dec_mtx);
}

bool init() {
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  // 图传 ws_stream_task 与 AI ai_worker 两个消费者并发抓帧：GRAB_LATEST 只认"最新帧"，
  // 在双缓冲下遇到多消费者会反复交出同一旧缓冲、并把 DMA 生产端卡死（图传冻结 + AI 反复收到同一张图）。
  // WHEN_EMPTY 双缓冲轮流交出，及时归还即不死锁，两者各取新鲜且不同的帧。
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 3;
  // 新库字段：jpeg 帧缓冲上限。经典版旧库无此字段，默认 recv_size=width*height/5(VGA≈60KB)，
  // VGA+jpeg_quality=10 的高熵画面单帧 JPEG 常超 60KB，cam_hal 判定 fb 溢出(FB-OVF) → ll_cam_stop
  // 硬停 DCMI → 图传冻结。抬到 128KB 消除溢出。依赖重编后的新版 esp32-camera 库，旧库编译会报错。
  config.jpeg_buffer_size = 128 * 1024;

  // PSRAM 缺失时降级
  if (!psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    blog::logf(blog::CAM, "init failed: 0x%x", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  // 关 aec2（night mode）减少低照度时长曝光跳变；
  // 固定快门对齐 50Hz 灯光闪烁周期（10ms），平均掉频闪横纹；亮度交给自动增益
  if (s->id.PID == OV3660_PID) {
    s->set_brightness(s, 1);
    // 饱和度不再下调：本板是机器视觉用途，黄/红这类颜色是识别小目标最强的线索，
    // 为"观感自然"压饱和度会直接吃掉它与地面的色差（量色差请取目标像素的分位数，
    // 别用 bbox 均值——均值会被框进来的背景稀释，框一大一小就不可比）。
    s->set_saturation(s, 0);
    s->set_aec2(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec_value(s, 230);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 0);
  s->set_hmirror(s, 1);
#endif

  s_ready = true;
  return true;
}

camera_fb_t* grab() {
  if (!s_ready) return nullptr;
  return esp_camera_fb_get();
}

// 真实 JPEG 长度。本板 fb_location=PSRAM + JPEG，驱动在这条分支下 **不是** 按实际字节数报 len，
// 而是按"消耗的 DMA 半缓冲数 × 半缓冲大小"算（cam_hal.c 的 psram_mode 分支），本身是向上取整的；
// 只有靠 ll_cam 里那处 EOI 截断（dma_buffer->len = offset_e + 2）才被修回真值。那处探测一旦没命中，
// len 就保留整个半缓冲 —— 于是把上一帧的残留、甚至下一帧整帧裹进来。实测（2026-09-21）：
// 缓冲里能拼到 2/4/6 张各自完整有效的 JPEG，len 最大 113363 而首帧只有 17.4KB；关掉第三个取帧方
// （mvfy）后仍余约 13%，说明单靠消费者数量解释不完，属驱动侧问题。
//
// 危害是连锁的：虚高的 len 会被 AI 原样 base64 进请求体，把 body 从 ~35KB 顶到 58K/139K/190K，
// 持续的大 TLS POST 压垮 WiFi 的发送路径（静态 TX 缓冲仅 8 个）→ 整块板子发不出任何帧：ARP 不应答、
// ping 不应答、SYN 发不出，而 CPU/BLE/exec 全活，几分钟后才自愈。
//
// 修法依据：JPEG 熵编码段中 0xFF 一律字节填充为 FF00，所以 FFD9 **只可能**是真正的 EOI 标记，
// 取"首个 EOI+2"必为真实长度，绝不误伤正文。这里不去改驱动（那是另一条线），只做长度校正。
size_t jpeg_len(const camera_fb_t* fb) {
  if (!fb || !fb->buf || fb->len == 0) return 0;
  const uint8_t* b = fb->buf;
  for (size_t i = 0; i + 1 < fb->len; i++) {
    if (b[i] == 0xFF && b[i + 1] == 0xD9) return i + 2;
  }
  return fb->len;   // 没找到 EOI（不该发生）：退回原值，至少不比现在更糟
}

bool available() { return s_ready; }

void return_frame(camera_fb_t* fb) {
  if (fb) esp_camera_fb_return(fb);
}

}
