#include <Arduino.h>  // psramFound / Serial / pinMode
#include "src/cam/camera.h"    // 内含 CAMERA_MODEL_* 与 camera_pins.h
#include "src/cam/camera_pins.h"
#include "src/core/board_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>         // memcpy：高清帧拷入快照
#include <esp_heap_caps.h>  // heap_caps_malloc(MALLOC_CAP_SPIRAM): 快照缓冲

namespace cam {

static bool s_ready = false;

// ---- 高清快照（request_hires 返回物） ----
// request_hires 抓到有效高清帧后，必须**在切回 VGA / deinit 之前**把字节拷进这块 PSRAM：
// deinit 会释放整套帧缓冲池，不拷就返回的是一个悬垂指针（buf/width/len 全被清零）。
// 快照 + 快照锁：request_hires 持锁拷贝后返回（锁**不**放），调用方在 return_frame() 归还时
// 识别快照并放锁。真正的高清驱动帧不返回给调用方。
static uint8_t* s_snap = nullptr;                // 高清 JPEG 字节(常驻 PSRAM, 尺寸与 jpeg_buffer 一致)
static camera_fb_t s_hires_fb = {};              // 返回给调用方的"伪 fb"，buf 指向 s_snap
static SemaphoreHandle_t s_snap_mtx = nullptr;   // 快照锁：同一块缓冲须串行读（上一轮读完才允许下轮覆写）

// 高清快照容量：SVGA(800×600) 高熵 JPEG 单帧可能逼近 jpeg_buffer_size(256K)。放大镜只裁中央带，
// 但驱动抓的是整幅高清, 快照必须罩得住整帧字节, 否则高清 JPEG 溢出被截(EOI 之前)废帧。
static size_t get_hires_snap_cap() { return 256 * 1024; }

// JPEG 软解码互斥（见 camera.h 说明）。惰性创建：首次解码器调用前任何时刻可用。
static SemaphoreHandle_t s_jpeg_dec_mtx = nullptr;
// 相机访问互斥锁：grab/request_hires 共用。抓帧与"重开相机切分辨率"在同一把锁里串行，否则
// esp_camera_fb_get 与 esp_camera_deinit/init 并发操作驱动 → 死锁（实测：开着图传时放大取帧必卡死）。
static SemaphoreHandle_t s_cam_mtx = nullptr;
// 相机正在"deinit→重开"重建窗口（request_hires 内）。置位后 grab() 不再分发 fb，防止重建期间
// 别处拿到的帧缓冲是旧 config 分配的、重建后被释放的悬垂指针。
static volatile bool s_reconfig = false;

// 生成摄像头 config：各传感器/板型 pin 唯一配置点，VGA 常态与 SVGA 高清重建共用同一份骨架，
// 只换 frame_size。调用方不得改里面的帧缓冲策略（WHEN_EMPTY/fb_count=3/PSRAM/jpeg_buffer 256K）。
static camera_config_t make_config(framesize_t fs) {
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
  config.frame_size = fs;
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
  // 硬停 DCMI → 图传冻结。抬到 256KB 同时罩住"高清重开的 SVGA"（800×600 高熵 JPEG 可能 >128K），
  // 避免重建高清瞬间触发 FB-OVF。依赖重编后的新版 esp32-camera 库，旧库编译会报错。
  config.jpeg_buffer_size = 256 * 1024;

  // PSRAM 缺失时降级（正常 N16R8 恒有 PSRAM，仅兜底）。
  if (!psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  return config;
}

// 摄像头初始化后统一应用传感器/板型的校准参数（亮度/饱和度/快门/镜像等）。init 与高清重建
// （esp_camera_init 返回后）各调一次，保证重建后校准不被清零。放在被调方的私处，避免重开丢失。
static void apply_sensor_calib() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
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
}

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

  camera_config_t config = make_config(FRAMESIZE_VGA);
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    blog::logf(blog::CAM, "init failed: 0x%x", err);
    return false;
  }

  apply_sensor_calib();

  if (!s_cam_mtx) s_cam_mtx = xSemaphoreCreateMutex();   // 一进 init 就先建锁（grab/request_hires 都要用）
  if (!s_snap_mtx) s_snap_mtx = xSemaphoreCreateMutex();
  if (s_snap) { heap_caps_free(s_snap); s_snap = nullptr; }   // 重入 init 先清旧快照
  s_snap = (uint8_t*)heap_caps_malloc(get_hires_snap_cap(), MALLOC_CAP_SPIRAM);   // 高清快照常驻 PSRAM
  blog::logf(blog::CAM, "[cam] 高清快照 %s", s_snap ? "就绪" : "分配失败(降级: 放大切高清将失效)");
  s_ready = true;
  return true;
}

camera_fb_t* grab() {
  if (!s_ready) return nullptr;
  // 抓帧持锁：与 request_hires 的重开相机串行（见 s_cam_mtx 注释）。
  if (s_cam_mtx) xSemaphoreTake(s_cam_mtx, portMAX_DELAY);
  // 重建窗口内（deinit/init 之间）暂停分发 fb：此时缓冲是旧 config 分配的，返回即悬垂。丢这一帧，各消费方本容错。
  camera_fb_t* fb = s_reconfig ? nullptr : esp_camera_fb_get();
  if (s_cam_mtx) xSemaphoreGive(s_cam_mtx);
  return fb;
}

// 切高清真拍一帧（放大镜用）：用库唯一支持的正规切换路径——deinit 相机 → 以 hires 分辨率重新 init →
// 抓一帧高清 → 再 deinit 回 VGA init 恢复常态。全程持锁，窗口内 grab() 暂停分发（s_reconfig）。
// 相比 set_framesize 热切：后者不改帧缓冲大小，SVGA 帧会写进 VGA 的缓冲溢出 → ll_cam_stop 卡死；
// 而 reconfigure 重分配全套帧缓冲，是驱动认可的大小匹配路径。返回高清 fb，调用方用完 return_frame() 归还。
camera_fb_t* request_hires(framesize_t hires, int* ok) {
  if (ok) *ok = 0;
  if (!s_ready || !s_cam_mtx) return nullptr;
  xSemaphoreTake(s_cam_mtx, portMAX_DELAY);   // 独占相机（等当前任何抓帧用完）
  s_reconfig = true;
  camera_fb_t* fb = nullptr;
  const int HI_WARM = 4;            // 预热帧数：先喂 AWB/AEC 收敛，前几帧丢弃
  camera_fb_t* d = nullptr;
  // ① 重开为高清（deinit→延时→init hires）并重应用校准。
  // 根因(源码实证 esp_camera.c fb_get L352)：fb->width 按 resolution[sensor.status.framesize] 无边界填；
  // deinit 后传感器未掉电、SCCB I2C 拆建后停在半事务，立即 init 会 I2C 静默失败 → framesize 变非法值 → 越界读
  // resolution[] → width=0 且驱动卡死。修法：deinit 后延时让总线释放；init 后跳过前几帧直到拿到有效帧(fb->width
  // 与 len 合法)才认定成功；拿不到回卫 VGA 且整条失败。SXGA 时序更紧必卡 → 调用方只应用于 SVGA。
  camera_config_t hc = make_config(hires);
  esp_camera_return_all();                // 清在途帧，避免 deinit 撞上未归还的缓冲
  esp_err_t e_de = esp_camera_deinit();
  vTaskDelay(pdMS_TO_TICKS(50));          // 等 SCCB 总线释放、XCLK 停稳
  esp_err_t e_hi_init = esp_camera_init(&hc);
  blog::logf(blog::CAM, "[cam] reconfig hires: deinit=%d init=%d", (int)e_de, (int)e_hi_init);
  if (e_hi_init != ESP_OK) goto restore;
  apply_sensor_calib();
  // 重开相机后 AWB/AEC 需要几帧才收敛，立即取帧会拿到偏色/偏曝的高清图（真实机验证：切完就拍明显
  // 偏青绿）。AWB 增益只在出帧时才更新——光 sleep 不取帧，收敛根本不发生。故取消原 400ms 睡眠，
  // 改为连续取帧喂 AWB/AEC 收敛，跳过前几帧后取到较稳的高清帧入快照（这段在 s_cam_mtx 锁内，
  // 图传会停一拍，放大镜本就偶尔一下）。
  // 跳到有效帧后，**在 deinit 回 VGA 之前**把字节拷进快照：deinit 释放整套高清帧缓冲池，
  // 不拷则下面 return 的是一个悬垂指针（buf/width/len 清零）。真正的高清驱动帧不返回给调用方。
  for (int i = 0; i < HI_WARM + 4; i++) {
    camera_fb_t* cv = esp_camera_fb_get();
    if (!cv) break;
    bool valid = cv->width > 0 && cv->height > 0 && cv->len > 0;
    blog::logf(blog::CAM, "[cam] reconfig hires: 候选帧%d w=%u h=%u len=%u%c", i,
               (unsigned)cv->width, (unsigned)cv->height, (unsigned)cam::jpeg_len(cv), valid ? '+' : '-');
    if (i < HI_WARM || !valid) { esp_camera_fb_return(cv); continue; }   // 预热帧/无效帧作废
    d = cv;
    break;
  }
  if (d && d->width > 0 && d->height > 0 && d->len > 0) {
    // 快照锁：等上一轮调用方读完 s_snap 再覆写（否则会把别人正读的缓冲写花）。
    if (s_snap) {
      size_t rl = cam::jpeg_len(d);
      if (rl > get_hires_snap_cap()) rl = get_hires_snap_cap();
      xSemaphoreTake(s_snap_mtx, portMAX_DELAY);
      memcpy(s_snap, d->buf, rl);
      s_hires_fb.buf = s_snap;
      s_hires_fb.len = rl;
      s_hires_fb.width = d->width;
      s_hires_fb.height = d->height;
      s_hires_fb.format = (pixformat_t)PIXFORMAT_JPEG;
      fb = &s_hires_fb;
      blog::logf(blog::CAM, "[cam] reconfig hires: 高清已入快照 %uB(率%.1f)", (unsigned)rl,
                 (float)d->width * d->height / (rl ? rl : 1));
    }
    esp_camera_fb_return(d);
  }
  // ② 无论成败，回 VGA 常态。★ fb 若指向快照(非驱动帧)，不能 return_all —— 那会连快照一起放不到，
  // 且 restore 的 deinit 还会把驱动缓冲池释放；快照是独立 PSRAM，安全。
  restore:
  esp_camera_return_all();
  e_de = esp_camera_deinit();
  vTaskDelay(pdMS_TO_TICKS(50));
  {
    camera_config_t vc = make_config(FRAMESIZE_VGA);
    esp_err_t e_lo_init = esp_camera_init(&vc);
    blog::logf(blog::CAM, "[cam] reconfig 回VGA: deinit=%d init=%d", (int)e_de, (int)e_lo_init);
    if (e_lo_init == ESP_OK) apply_sensor_calib();
  }
  s_reconfig = false;
  xSemaphoreGive(s_cam_mtx);   // 注意: s_snap_mtx **不放**，快照由调用方 return_frame() 释放
  if (fb) { if (ok) *ok = 1;
    return fb;                 // 指向快照的"伪 fb"，调用方用完 return_frame() 归还
  }
  return nullptr;
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
  // 快照"伪 fb"（request_hires 的返回物）：不归还给驱动（它本来就不是驱动帧），只释放快照锁，
  // 让下一轮 request_hires 能覆写 s_snap。普通驱动帧走 esp_camera_fb_return。
  if (fb == &s_hires_fb) {
    if (s_snap_mtx) xSemaphoreGive(s_snap_mtx);
    return;
  }
  if (fb) esp_camera_fb_return(fb);
}

}
