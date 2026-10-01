#pragma once

// ===================
// 选择摄像头型号（引脚定义见 camera_pins.h，按 CAMERA_MODEL_* 宏分支）。
// 所有需要引脚定义的编译单元都经 camera.h 间接获得该宏，此处为唯一配置点；
// 换板只需改下面这一行（与 Stm32-Vision.ino 不再各自定义，避免不一致）。
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
//#define CAMERA_MODEL_AI_THINKER // 经典 ESP32-CAM 走线，在 S3 上会导致 ll_cam_set_pin 空指针崩溃，勿用
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM（当前板=ESP32-S3-CAM N16R8 + OV3660，SIOD/SIOC 已真机确认）
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "src/cam/camera_pins.h"

#include "esp_camera.h"

namespace cam {

// 初始化摄像头（常态 VGA = JPEG 输出，双缓冲 + WHEN_EMPTY：图传与 AI 并发抓帧不死锁）
bool init();

// 取一帧（同步接口），使用后必须调用 return_frame() 归还缓冲
camera_fb_t* grab();

// ---- 排空取"当前"一帧（凡是要"此刻画面"的调用点都必须走这里）----
// 相机 fb_count=3，传感器是自由跑的：消费者比它慢时队列里**一直压着 2 帧旧的**，直接 grab 拿到的
// 是"动作前"的画面（WHEN_EMPTY 的驱动文档就写了 "first fb_count frames might be old"）。
// 排空（连续 grab+return）到队列空之后，下一次 grab 会**阻塞等一帧新拍的** —— 那才是当前画面。
// 队列满时排空几乎不耗时，只有最后一次 grab 等一个帧周期。drop=建议 2~4。
// 真机症状：/capture 量到的目标位置与追踪器看到的不是同一帧；夹取闭环读数滞后一步 ⇒ 转过头。
camera_fb_t* grab_fresh(int drop);

// ---- 最新一帧发布/读取（"随手要一张"的消费者走这里，别自己 grab）----
// 图传任务是**唯一的喂帧者**，每拿到一帧就把 JPEG 副本 publish 到这里；/capture 之类按需读取。
// 这样"抓帧者"只剩一个，不再有两个消费者各排各的队、互相错位（那才是 staleness 的结构性来源）。
// 只发 JPEG 副本：RGB565 全帧 614KB，不值得每帧拷；那种模式下按需方走 grab_fresh 自己软编。
void publish_latest(const camera_fb_t* fb);
// 拷出最新一帧到 out（cap 字节），返回长度；0 = 还没有 / 缓冲不够。age_ms 回填该帧有多旧(ms)。
size_t latest_copy(uint8_t* out, size_t cap, uint32_t* age_ms);

// ---- 高频取帧仲裁：切高清真拍一帧（放大镜用） ----
// 一次性完成"切到 hires → 抓一帧高清 → 立即切回 VGA → 放锁"，取帧与切分辨率在同一把互斥锁里串行，
// 与 grab() 不并发，杜绝 set_framesize 与 esp_camera_fb_get 同时操作驱动导致的死锁。
// 返回高清 fb，调用方用完必须 return_frame() 归还（归还不涉及切回，高清帧抓完锁已放）。
// *ok: 成功=1；失败(传感器不支持/取帧失败)=0 且返回 nullptr。
camera_fb_t* request_hires(framesize_t hires, int* ok);

// ---- 跟踪模式硬切换 ----
// 本模块常态 VGA = sensor 直出 JPEG（图传/AI 白嫖，省软编）；但本地追踪 track 只认 RGB565 帧缓冲，
// 故需"进入跟踪时切 RGB565、结束切回 JPEG"。切换必须走 deinit→init（两种格式的帧缓冲大小不同，
// 热切 set_pixformat 会把帧写进不匹配的缓冲 → ll_cam_stop 卡死，与切分辨率同源的坑），
// 全程持相机锁（图传/AI 抓帧让路，窗口内置 s_reconfig 暂停分发）。幂等：已是目标格式直接返回。
// ⚠️ 阻塞百 ms～秒级；调用点**不得**持有相机锁（会在内部取锁），且不应高频调用（只在跟踪起停各一次）。
// 返回 true = 切换成功（或本就处于目标模式）。
bool set_track_mode(bool on);

// 摄像头是否可用（init 后即定），供调用方做"无画面降级"判断
bool available();

// 归还帧缓冲
void return_frame(camera_fb_t* fb);

// 真实 JPEG 长度（见 camera.cpp 的说明）：fb->len 在本板配置下可能被驱动报大，
// 凡是要把帧字节**喂给别处**（base64 进 AI 请求体 / 走 HTTP 发出去 / 存留档）都必须用它，
// 不要直接用 fb->len。返回 0 = 帧不可用。
size_t jpeg_len(const camera_fb_t* fb);

// 把一帧编码为 JPEG 字节写进 out（cap 字节），返回写入长度；0 = 失败 / 缓冲不足。
// 常态帧本就是 sensor 直出 JPEG（直接按真实长度拷出）；跟踪模式帧是 RGB565（图传/AI 仍要 JPEG），
// 此时在此软编。若 fb 是 JPEG（常态帧 / 高清快照）则直拷，不重编。
// quality 是 jpge 刻度 0~100（越大越好、帧越大），与 sensor 的 jpeg_quality(0~63) 不同刻度。
size_t encode_jpeg(const camera_fb_t* fb, int quality, uint8_t* out, size_t cap);

// JPEG 软解码互斥（Tjpgd 非线程安全）：jpeg 软解可能在 AI worker（放大镜裁图）与其它调用方
// 上并发发生，库内部用全局静态上下文传参，互相踩会解出"Y 结构在、Cb/Cr 错乱 + 8×8 块状"的
// 坏图或解码失败。所有 jpg2rgb565 / fmt2jpg_cb（jpge 编码同为静态上下文）调用点都必须先
// 取锁，用完即放。持锁期是库调用粒度，软的 ms 级，不影响实时性。
void lock_jpeg_dec();
void unlock_jpeg_dec();

}
