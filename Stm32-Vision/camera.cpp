#include <Arduino.h>  // psramFound / Serial / pinMode
#include "camera.h"    // 内含 CAMERA_MODEL_* 与 camera_pins.h
#include "camera_pins.h"

namespace cam {

static bool s_ready = false;

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
  config.jpeg_quality = 14;
  config.fb_count = 2;

  // PSRAM 缺失时降级
  if (!psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[cam] init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  // OV3660 饱和度偏高微调；关 aec2（night mode）减少低照度时长曝光跳变；
  // 固定快门对齐 50Hz 灯光闪烁周期（10ms），平均掉频闪横纹；亮度交给自动增益
  if (s->id.PID == OV3660_PID) {
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
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

bool available() { return s_ready; }

void return_frame(camera_fb_t* fb) {
  if (fb) esp_camera_fb_return(fb);
}

}
