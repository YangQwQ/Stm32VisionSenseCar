#include <Arduino.h>  // psramFound / Serial / pinMode
#include "src/cam/camera.h"    // 内含 CAMERA_MODEL_* 与 camera_pins.h
#include "src/cam/camera_pins.h"
#include "src/core/board_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>         // memcpy：高清帧拷入快照
#include <esp_heap_caps.h>  // heap_caps_malloc(MALLOC_CAP_SPIRAM): 快照缓冲
#include <img_converters.h> // frame2jpg_cb: 非 JPEG 帧软编码（图传 / AI 请求体）

namespace cam {

static bool s_ready = false;

// AWB/AEC 预热帧数：相机上电/切分辨率后，内部 ISP 增益从默认值起、仅在**出帧**时更新统计收敛
// （光 sleep 无效）。固定取这么多帧丢弃，喂收敛后再把帧分发给消费者，避免首个业务帧偏色/偏曝。
static const int HI_WARM = 4;
// 切分辨率（hires ↔ VGA）是重开相机、AWB 从零重来，比上电那次更难收敛（实测偶尔仍偏绿）。
// 故切配置的预热比上电多喂几帧，不与上电共用一份常量。
static const int HI_WARM_RECONFIG = HI_WARM + 3;

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
// 相机正在"deinit→重开"重建窗口（request_hires / set_track_mode 内）。置位后 grab() 不再分发 fb，
// 防止重建期间别处拿到的帧缓冲是旧 config 分配的、重建后被释放的悬垂指针。
static volatile bool s_reconfig = false;
// 当前 VGA 常态的像素格式：JPEG=常态（图传/AI 直传，不软编）/ RGB565=跟踪模式（track 直读）。
// request_hires 回 VGA 时据此恢复，否则跟踪期切高清后会把相机错误地留回 JPEG。只由 init()/set_track_mode() 写。
static pixformat_t s_vga_fmt = PIXFORMAT_JPEG;

// ---- 最新一帧发布（"随手要一张"的消费者读这里，不必自己 grab）----
// 图传任务是唯一的 grab 者，它每拿到一帧就把 JPEG 副本发到这儿。拷贝在锁内完成，读者不会读到半帧。
static const size_t kLatestCap = 96 * 1024;      // VGA JPEG 实测 20~60KB，96K 足够
static uint8_t*            s_latest = nullptr;
static size_t              s_latest_len = 0;
static uint32_t            s_latest_ms = 0;
static SemaphoreHandle_t   s_latest_mtx = nullptr;

// 生成摄像头 config：各传感器/板型 pin 唯一配置点，VGA 常态与 SVGA 高清重建共用同一份骨架，
// 只换 frame_size 与像素格式。调用方不得改里面的帧缓冲策略（WHEN_EMPTY/fb_count=3/PSRAM）。
// fmt：VGA 常态默认 JPEG——图传/AI 直接吃 sensor 硬编帧，省掉每帧软编；
//      跟踪模式（set_track_mode(true)）才把 VGA 换成 RGB565——track 直读 luma/chroma，省掉软解；
//      高清重建仍用 JPEG——放大镜只要中央带，整幅软解不划算，沿用驱动硬编。
static camera_config_t make_config(framesize_t fs, pixformat_t fmt = PIXFORMAT_JPEG) {
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
  config.pixel_format = fmt;
  // 图传 ws_stream_task 与 AI ai_worker 两个消费者并发抓帧：GRAB_LATEST 只认"最新帧"，
  // 在双缓冲下遇到多消费者会反复交出同一旧缓冲、并把 DMA 生产端卡死（图传冻结 + AI 反复收到同一张图）。
  // WHEN_EMPTY 双缓冲轮流交出，及时归还即不死锁，两者各取新鲜且不同的帧。
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 3;
  // jpeg_quality / jpeg_buffer_size 只对 JPEG 输出有意义：RGB565 下驱动按 width×height×2 自算帧长。
  // jpeg_buffer_size：经典版旧库无此字段，默认 recv_size=width*height/5(VGA≈60KB)，
  // VGA+jpeg_quality=10 的高熵画面单帧 JPEG 常超 60KB，cam_hal 判定 fb 溢出(FB-OVF) → ll_cam_stop
  // 硬停 DCMI → 图传冻结。抬到 256KB 同时罩住"高清重开的 SVGA"（800×600 高熵 JPEG 可能 >128K），
  // 避免重建高清瞬间触发 FB-OVF。依赖重编后的新版 esp32-camera 库，旧库编译会报错。
  const bool is_jpeg = (fmt == PIXFORMAT_JPEG);
  config.jpeg_quality = is_jpeg ? 10 : 0;
  config.jpeg_buffer_size = is_jpeg ? (256 * 1024) : 0;

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
    // 饱和度**上调**：本板是机器视觉用途，彩色小目标（不同颜色小方块）与灰地面/灰机械臂的判别
    // 全靠色度。实测 set_saturation(0) 时一个小黄块在解码图里色度 std 只有 3.6（≈无色），
    // 追踪只能退化成低对比的亮度匹配、频繁丢目标。提到 +2 让色度真正拉开。
    s->set_saturation(s, 2);
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

// 一律不用 psram 直写。三条实证：① RGB565 整帧 614KB 直写 PSRAM 会丢字节 → 画面被横切成条；
// ② SVGA 高清（约 130KB）同样丢字节 → 一帧都拿不到；③ psram 与非 psram 来回切之后，VGA 那路会
// "init 报 OK 却再也取不到帧"（zoomshot 后再抓帧必失败，连自愈重开都救不回）。
// 代价是常驻一块约 16KB 的内部 DMA（dma_buffer）—— 门槛已按 CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX=16384
// 压低，内部池够用；换来"相机任何时刻都能重开、画面干净"。参数保留仅为调用点可读。
static void set_psram_for(pixformat_t) {
  esp_camera_set_psram_mode(false);
}

// 切 RGB565 前等内部 DMA 池长出够一块连续缓冲（= 驱动的 dma_buffer；本板 CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX=16384 ⇒ 15360）。
// 图传刚开那几秒池子可能还没回收够，硬切会 init 失败并把相机留在 deinit 态。
static void wait_dma_block(size_t need, int max_ms) {
  for (int waited = 0; waited < max_ms &&
       heap_caps_get_largest_free_block(MALLOC_CAP_DMA) < need; waited += 100) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

bool init() {
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  set_psram_for(PIXFORMAT_JPEG);   // 常态 JPEG = psram 直写

  camera_config_t config = make_config(FRAMESIZE_VGA, PIXFORMAT_JPEG);
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    blog::logf(blog::CAM, "init failed: 0x%x", err);
    return false;
  }
  s_vga_fmt = PIXFORMAT_JPEG;   // 常态 = JPEG 直出（跟踪模式由 set_track_mode 切）

  apply_sensor_calib();

  // 上电后 AWB/AEC 增益从默认值开始，前几帧必然偏色（本板 LED 冷白光源下偏绿最明显）。
  // AWB 只在出帧时更新增益——sleep 等待无效——固定取几帧丢弃喂收敛，让首个被消费的帧就是稳的。
  // 图传/AI 都在 init 之后才抓帧，此处预热不丢业务帧。
  for (int i = 0; i < HI_WARM; i++) {
    camera_fb_t* w = esp_camera_fb_get();
    if (!w) break;
    esp_camera_fb_return(w);
  }

  if (!s_cam_mtx) s_cam_mtx = xSemaphoreCreateMutex();   // 一进 init 就先建锁（grab/request_hires 都要用）
  if (!s_snap_mtx) s_snap_mtx = xSemaphoreCreateMutex();
  if (!s_latest_mtx) s_latest_mtx = xSemaphoreCreateMutex();
  if (s_snap) { heap_caps_free(s_snap); s_snap = nullptr; }   // 重入 init 先清旧快照
  s_snap = (uint8_t*)heap_caps_malloc(get_hires_snap_cap(), MALLOC_CAP_SPIRAM);   // 高清快照常驻 PSRAM
  if (!s_latest) {
    s_latest = (uint8_t*)heap_caps_malloc(kLatestCap, MALLOC_CAP_SPIRAM);
    if (s_latest) s_latest_len = 0;
  }
  blog::logf(blog::CAM, "[cam] 高清快照 %s", s_snap ? "就绪" : "分配失败(降级: 放大切高清将失效)");
  s_ready = true;
  return true;
}

// 相机自愈：连续抓帧失败到阈值就自动重开一次（deinit→init，用当前 framesize/格式）。只在 grab() 里调
// （已持 s_cam_mtx，不自取锁）。实测有两类"init 报 OK 却不出帧"：① 高清往返之后 VGA 十几秒不出帧
// （此时内存很宽裕，不是分配问题）；② 切格式失败把相机留在 deinit 态。两条都靠这里兜住 ——
// 否则 AI 每次被卡都要干等（一次失败先在驱动里等 4s 超时），任务看起来就是"每轮几十秒"。
static void reinit_current() {
  // 非 psram 的 VGA JPEG 要一块 16KB 连续内部 DMA：先等池子够，免得重开失败又转一圈
  wait_dma_block(16384, 1500);
  s_reconfig = true;
  camera_config_t c = make_config(FRAMESIZE_VGA, s_vga_fmt);
  esp_camera_return_all();
  esp_camera_deinit();
  set_psram_for(s_vga_fmt);
  vTaskDelay(pdMS_TO_TICKS(50));
  esp_err_t e = esp_camera_init(&c);
  if (e == ESP_OK) {
    apply_sensor_calib();
    for (int i = 0; i < HI_WARM_RECONFIG; i++) {   // 喂 AWB/AEC 收敛，别把偏色帧放出去
      camera_fb_t* w = esp_camera_fb_get();
      if (!w) break;
      esp_camera_fb_return(w);
    }
  }
  blog::logf(blog::CAM, "[cam] 连续抓帧失败 → 自愈重开: init=%d(%s)", (int)e, esp_err_to_name(e));
  s_reconfig = false;
}

static int s_grab_fail = 0;              // 连续抓帧失败计数（自愈用）
static const int kGrabFailReinit = 2;    // 到这个数就重开相机（一次失败已含驱动侧 4s 超时）

camera_fb_t* grab() {
  if (!s_ready) return nullptr;
  // 抓帧持锁：与 request_hires / set_track_mode 的重开相机串行（见 s_cam_mtx 注释）。
  // ⚠️ 用**带超时**的取锁而非 portMAX_DELAY：真机上遇到过"抓帧永远拿不到锁 → 整条闭环静默僵死、
  // 只能断电"。超时后记一条限频日志并丢这一帧，让图传/AI/grasp 自愈而不是一起僵住（日志会点名是谁）。
  if (s_cam_mtx && xSemaphoreTake(s_cam_mtx, pdMS_TO_TICKS(1200)) != pdTRUE) {
    static uint32_t s_warn_ms = 0;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if ((uint32_t)(now - s_warn_ms) > 2000) {
      s_warn_ms = now;
      blog::logf(blog::CAM, "[cam] grab 等相机锁超时(1200ms)：有任务长期持锁(重开相机/切格式卡住?)");
    }
    return nullptr;
  }
  // 重建窗口内（deinit/init 之间）暂停分发 fb：此时缓冲是旧 config 分配的，返回即悬垂。丢这一帧，各消费方本容错。
  camera_fb_t* fb = nullptr;
  if (s_reconfig) {
    static uint32_t s_warn2_ms = 0;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if ((uint32_t)(now - s_warn2_ms) > 2000) {   // 正常重建是百 ms 级；反复出现=重建卡住了
      s_warn2_ms = now;
      blog::logf(blog::CAM, "[cam] grab 被 s_reconfig 挡下：重建窗口没关(卡在 deinit/init?)");
    }
  } else {
    fb = esp_camera_fb_get();
    if (!fb) {
      // ★ 抓帧失败时把内部/DMA 水位打出来（限频 3s，免得每帧刷屏）。
      static uint32_t s_fail_ms = 0;
      const uint32_t now2 = (uint32_t)(esp_timer_get_time() / 1000);
      if ((uint32_t)(now2 - s_fail_ms) > 3000) {
        s_fail_ms = now2;
        blog::logf(blog::CAM, "[cam] 抓帧失败! 内部堆=%u 最大块=%u | DMA 最大块=%u",
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                   (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
      }
      if (++s_grab_fail >= kGrabFailReinit) {
        s_grab_fail = 0;
        reinit_current();
        fb = esp_camera_fb_get();     // 重开后立刻再试一次；拿不到就交给下一次调用
      }
    } else {
      s_grab_fail = 0;
    }
  }
  if (s_cam_mtx) xSemaphoreGive(s_cam_mtx);
  return fb;
}

// 排空取"当前"一帧：连续 grab+return 把队列倒空，最后一次 grab 就会阻塞等一帧新拍的。
// 队列满时前几次是立刻返回的，只有最后一次等一个帧周期 —— 代价就是这么一点。
camera_fb_t* grab_fresh(int drop) {
  if (!s_ready) return nullptr;
  if (drop < 0) drop = 0; else if (drop > 6) drop = 6;
  for (int k = 0; k < drop; k++) {
    camera_fb_t* stale = grab();
    if (!stale) break;          // 拿不到就别空转，直接进最后一次
    return_frame(stale);
  }
  return grab();
}

void publish_latest(const camera_fb_t* fb) {
  if (!fb || !s_latest || !s_latest_mtx) return;
  if (fb->format != PIXFORMAT_JPEG) return;   // RGB565 全帧 614KB，不每帧拷；那种模式按需方走 grab_fresh 软编
  const size_t n = jpeg_len(fb);
  if (n == 0 || n > kLatestCap) return;
  if (xSemaphoreTake(s_latest_mtx, pdMS_TO_TICKS(50)) != pdTRUE) return;
  memcpy(s_latest, fb->buf, n);
  s_latest_len = n;
  s_latest_ms = (uint32_t)(esp_timer_get_time() / 1000);
  xSemaphoreGive(s_latest_mtx);
}

size_t latest_copy(uint8_t* out, size_t cap, uint32_t* age_ms) {
  if (!s_latest || !s_latest_mtx || !out || cap == 0) return 0;
  if (xSemaphoreTake(s_latest_mtx, pdMS_TO_TICKS(50)) != pdTRUE) return 0;
  const size_t n = s_latest_len;
  const bool ok = (n > 0 && n <= cap);
  if (ok) {
    memcpy(out, s_latest, n);
    if (age_ms) *age_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_latest_ms;
  }
  xSemaphoreGive(s_latest_mtx);
  return ok ? n : 0;
}

// 切高清真拍一帧（放大镜用）：用库唯一支持的正规切换路径——deinit 相机 → 以 hires 分辨率重新 init →
// 抓一帧高清 → 再 deinit 回 VGA init 恢复常态。全程持锁，窗口内 grab() 暂停分发（s_reconfig）。
// 相比 set_framesize 热切：后者不改帧缓冲大小，SVGA 帧会写进 VGA 的缓冲溢出 → ll_cam_stop 卡死；
// 而 reconfigure 重分配全套帧缓冲，是驱动认可的大小匹配路径。返回高清 fb，调用方用完 return_frame() 归还。
camera_fb_t* request_hires(framesize_t hires, int* ok) {
  if (ok) *ok = 0;
  if (!s_ready || !s_cam_mtx) return nullptr;
  // 高清段（非 psram）重开要一块 16KB 连续内部 DMA：先等池子够再取锁，别拿着相机锁干等（grab 只等 1200ms）
  wait_dma_block(16384, 1500);
  // 带超时取锁（同 grab 的说明）：拿不到就整条失败，别把自己僵在 portMAX_DELAY 上。
  if (xSemaphoreTake(s_cam_mtx, pdMS_TO_TICKS(1500)) != pdTRUE) {
    blog::logf(blog::CAM, "[cam] request_hires 放弃: 等相机锁超时(1500ms)");
    return nullptr;
  }
  s_reconfig = true;
  camera_fb_t* fb = nullptr;
  camera_fb_t* d = nullptr;
  // ① 重开为高清（deinit→延时→init hires）并重应用校准。
  // 根因(源码实证 esp_camera.c fb_get L352)：fb->width 按 resolution[sensor.status.framesize] 无边界填；
  // deinit 后传感器未掉电、SCCB I2C 拆建后停在半事务，立即 init 会 I2C 静默失败 → framesize 变非法值 → 越界读
  // resolution[] → width=0 且驱动卡死。修法：deinit 后延时让总线释放；init 后跳过前几帧直到拿到有效帧(fb->width
  // 与 len 合法)才认定成功；拿不到回卫 VGA 且整条失败。SXGA 时序更紧必卡 → 调用方只应用于 SVGA。
  camera_config_t hc = make_config(hires, PIXFORMAT_JPEG);
  esp_camera_return_all();                // 清在途帧，避免 deinit 撞上未归还的缓冲
  esp_err_t e_de = esp_camera_deinit();
  // 高清段走内部乒乓缓冲：psram 直写 SVGA（约 130KB/帧）和 RGB565 一样会丢字节 ⇒ 一帧都拿不到
  // （实测 /zoomshot 必失败、且之后相机取不到帧）。
  esp_camera_set_psram_mode(false);
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
  for (int i = 0; i < HI_WARM_RECONFIG + 4; i++) {
    camera_fb_t* cv = esp_camera_fb_get();
    if (!cv) break;
    bool valid = cv->width > 0 && cv->height > 0 && cv->len > 0;
    blog::logf(blog::CAM, "[cam] reconfig hires: 候选帧%d w=%u h=%u len=%u%c", i,
               (unsigned)cv->width, (unsigned)cv->height, (unsigned)cam::jpeg_len(cv), valid ? '+' : '-');
    if (i < HI_WARM_RECONFIG || !valid) { esp_camera_fb_return(cv); continue; }   // 预热帧/无效帧作废
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
  {
    // 回 VGA 也是一次"重开相机"：deinit 后 init。cam_hal 的重开全程走 MALLOC_CAP_DMA
    // (cam_obj / lldesc 描述符)，另有 frames 数组走内部堆；内部 DMA 块被切碎时 init 会失败并
    // 返回 ESP_FAIL(日志里的 -1)。失败后直接收手会把相机永久留在 deinit 态 —— 之后每次取帧都失败，
    // AI 空转多轮后任务夭折。故重试若干次：每次前先 deinit 回干净态(库对 cam_obj==NULL 安全)，
    // 并打出具体 esp_err 便于分诊。
    camera_config_t vc = make_config(FRAMESIZE_VGA, s_vga_fmt);   // 回到"当前常态格式"（JPEG 或跟踪模式 RGB565）
    esp_err_t e_lo_init = ESP_FAIL;
    for (int attempt = 1; attempt <= 3; attempt++) {
      esp_camera_return_all();
      e_de = esp_camera_deinit();
      set_psram_for(s_vga_fmt);         // 回常态格式：JPEG=psram / RGB565=内部缓冲
      vTaskDelay(pdMS_TO_TICKS(50));     // 等 SCCB 总线释放、XCLK 停稳（同高清段）
      e_lo_init = esp_camera_init(&vc);
      blog::logf(blog::CAM, "[cam] reconfig 回VGA: 第%d次 deinit=%d init=%d(%s)", attempt,
                 (int)e_de, (int)e_lo_init, esp_err_to_name(e_lo_init));
      if (e_lo_init != ESP_OK) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
      apply_sensor_calib();
      // ★ init 报 OK ≠ 出帧：实测高清往返后 VGA 会"静默"十几秒（内存很宽裕，纯驱动/传感器侧）。
      //   必须**取到帧**才算恢复，否则重开一次 —— 比干等那十几秒快。顺便喂 AWB/AEC 收敛。
      int got = 0;
      for (int i = 0; i < HI_WARM_RECONFIG; i++) {
        camera_fb_t* w = esp_camera_fb_get();
        if (!w) break;
        got++;
        esp_camera_fb_return(w);
      }
      if (got > 0) break;
      blog::logf(blog::CAM, "[cam] reconfig 回VGA: init OK 但一帧都取不到, 重开");
      e_lo_init = ESP_FAIL;
      vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (e_lo_init != ESP_OK) {
      blog::logf(blog::CAM, "[cam] 回VGA 连续失败, 相机停在 deinit 态: 后续取帧会一直失败");
    }
  }
  s_reconfig = false;
  xSemaphoreGive(s_cam_mtx);   // 注意: s_snap_mtx **不放**，快照由调用方 return_frame() 释放
  if (fb) { if (ok) *ok = 1;
    return fb;                 // 指向快照的"伪 fb"，调用方用完 return_frame() 归还
  }
  return nullptr;
}

// ---- 跟踪模式硬切换（track 起停时各切一次，不每帧切） ----
// 常态 VGA = sensor 直出 JPEG（图传/AI 白嫖硬编，不软编）；track 只认 RGB565 帧缓冲，故进入跟踪时切
// RGB565、结束切回 JPEG。切换必须走 deinit→init：两种格式的帧缓冲大小不同，热切 set_pixformat 会把帧
// 写进不匹配的缓冲 → ll_cam_stop 卡死（与 set_framesize 同源的坑）。
// 幂等：已是目标格式直接返回，不无谓重开（跟踪期反复调用、grasp/AI 两条路径叠加调用都安全）。
// 失败重试 3 次（同 request_hires restore），绝不把相机留在 deinit 态。
bool set_track_mode(bool on) {
  if (!s_ready || !s_cam_mtx) return false;
  const pixformat_t want = on ? PIXFORMAT_RGB565 : PIXFORMAT_JPEG;
  // 带超时取锁：拿不到就放弃（相机保持当前格式），绝不把自己僵在这里 —— 跟踪起停只是"尽量切"，卡死代价大得多。
  if (xSemaphoreTake(s_cam_mtx, pdMS_TO_TICKS(1500)) != pdTRUE) {
    blog::logf(blog::CAM, "[cam] 切格式放弃: 等相机锁超时(1500ms)");
    return false;
  }
  if (s_vga_fmt == want) { xSemaphoreGive(s_cam_mtx); return true; }   // 幂等：已是目标模式
  s_reconfig = true;                          // 重建窗口：grab() 暂停分发（旧缓冲悬垂）
  camera_config_t nc = make_config(FRAMESIZE_VGA, want);
  if (on) wait_dma_block(15360, 3000);        // 切 RGB565 要连续内部 DMA，先等池子够（见 wait_dma_block）
  esp_err_t e_init = ESP_FAIL;
  for (int attempt = 1; attempt <= 3; attempt++) {
    esp_camera_return_all();
    esp_err_t e_de = esp_camera_deinit();
    set_psram_for(want);                      // RGB565=内部缓冲 / JPEG=psram
    vTaskDelay(pdMS_TO_TICKS(50));            // 等 SCCB 总线释放、XCLK 停稳（同 request_hires）
    e_init = esp_camera_init(&nc);
    blog::logf(blog::CAM, "[cam] 切%s: 第%d次 deinit=%d init=%d(%s)",
               on ? "RGB565(跟踪)" : "JPEG(常态)", attempt, (int)e_de, (int)e_init, esp_err_to_name(e_init));
    if (e_init == ESP_OK) break;
    vTaskDelay(pdMS_TO_TICKS(200));           // 给刚释放的内部 DMA 块回收/合并留窗口，再试
  }
  const bool ok = (e_init == ESP_OK);
  if (ok) {
    s_vga_fmt = want;
    apply_sensor_calib();
    // 重开相机后 AWB/AEC 从零收敛（只睡不取帧不收敛）：喂预热帧再放行，避免跟踪/AI 头几帧吃偏色。
    for (int i = 0; i < HI_WARM_RECONFIG; i++) {
      camera_fb_t* w = esp_camera_fb_get();
      if (!w) break;
      esp_camera_fb_return(w);
    }
  } else {
    blog::logf(blog::CAM, "[cam] 切格式失败, 相机停在 deinit 态: 后续取帧会一直失败");
  }
  s_reconfig = false;
  xSemaphoreGive(s_cam_mtx);
  return ok;
}

// 真实 JPEG 长度。本板 fb_location=PSRAM + JPEG，驱动在这条分支下 **不是** 按实际字节数报 len，
// 而是按"消耗的 DMA 半缓冲数 × 半缓冲大小"算（cam_hal.c 的 psram_mode 分支），本身是向上取整的；
// 只有靠 ll_cam 里那处 EOI 截断（dma_buffer->len = offset_e + 2）才被修回真值。那处探测一旦没命中，
// len 就保留整个半缓冲 —— 于是把上一帧的残留、甚至下一帧整帧裹进来。实测（2026-09-21）：
// 缓冲里能拼到 2/4/6 张各自完整有效的 JPEG，len 最大 113363 而首帧只有 17.4KB；
// 减小取帧方数量后仍余约 13%，说明单靠消费者数量解释不完，属驱动侧问题。
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

// ---- 非 JPEG 帧 → JPEG 软编码（图传 / AI 请求体） ----
// jpge 的输出回调：直接写进调用方缓冲（省一次 malloc+拷贝）。`index` 语义本地无源码可查，两种约定
// 都兼容（仅当 index==已写长度时按 index 写，否则追加）；缓冲不够返回 0 并置 over。
namespace {
struct JpgOut { uint8_t* buf; size_t cap; size_t written; bool over; };

size_t jpg_write_cb(void* arg, size_t index, const void* data, size_t len) {
  JpgOut* o = (JpgOut*)arg;
  // `!data` 是 jpge 收尾的 put_buf(NULL,0)，不是错误/溢出（上游契约：pBuf 空 ⇒ 收尾返回 true）。
  if (!data) return 0;
  size_t at = (index == o->written) ? index : o->written;
  if (at + len > o->cap) { o->over = true; return 0; }
  memcpy(o->buf + at, data, len);
  o->written = at + len;
  return len;
}
}  // namespace

size_t encode_jpeg(const camera_fb_t* fb, int quality, uint8_t* out, size_t cap) {
  if (!fb || !fb->buf || !out || cap == 0) return 0;
  // JPEG 帧（高清快照）本就是这个格式，按真实长度直接拷出去。
  if (fb->format == PIXFORMAT_JPEG) {
    size_t rl = jpeg_len(fb);
    if (rl == 0 || rl > cap) return 0;
    memcpy(out, fb->buf, rl);
    return rl;
  }
  // 非 JPEG（跟踪模式 = RGB565）：jpge 与 Tjpgd 共用静态上下文（见 camera.h 说明），持锁串行。
  JpgOut o = { out, cap, 0, false };
  lock_jpeg_dec();
  bool ok = frame2jpg_cb((camera_fb_t*)fb, (uint8_t)quality, jpg_write_cb, &o);
  unlock_jpeg_dec();
  if (!ok || o.over || o.written == 0) return 0;
  return o.written;
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
