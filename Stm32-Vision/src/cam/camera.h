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

// 初始化摄像头（JPEG 输出，双缓冲 + WHEN_EMPTY：图传与 AI 并发抓帧不死锁）
bool init();

// 取一帧（同步接口），使用后必须调用 return_frame() 归还缓冲
camera_fb_t* grab();

// ---- 高频取帧仲裁：切高清真拍一帧（放大镜用） ----
// 一次性完成"切到 hires → 抓一帧高清 → 立即切回 VGA → 放锁"，取帧与切分辨率在同一把互斥锁里串行，
// 与 grab() 不并发，杜绝 set_framesize 与 esp_camera_fb_get 同时操作驱动导致的死锁。
// 返回高清 fb，调用方用完必须 return_frame() 归还（归还不涉及切回，高清帧抓完锁已放）。
// *ok: 成功=1；失败(传感器不支持/取帧失败)=0 且返回 nullptr。
camera_fb_t* request_hires(framesize_t hires, int* ok);

// 摄像头是否可用（init 后即定），供调用方做"无画面降级"判断
bool available();

// 归还帧缓冲
void return_frame(camera_fb_t* fb);

// 真实 JPEG 长度（见 camera.cpp 的说明）：fb->len 在本板配置下可能被驱动报大，
// 凡是要把帧字节**喂给别处**（base64 进 AI 请求体 / 走 HTTP 发出去 / 存留档）都必须用它，
// 不要直接用 fb->len。返回 0 = 帧不可用。
size_t jpeg_len(const camera_fb_t* fb);

// JPEG 软解码互斥（Tjpgd 非线程安全）：jpeg 软解在 mvfy 任务（core0 常驻采样）与 AI worker
// （放大镜裁图）两个任务上并发调用，库内部用全局静态上下文传参，互相踩会解出"Y 结构在、
// Cb/Cr 错乱 + 8×8 块状"的坏图或解码失败。所有 jpg2rgb565 / fmt2jpg_cb（jpge 编码同为静态
// 上下文）调用点都必须先取锁，用完即放。持锁期是库调用粒度，软的 ms 级，不影响实时性。
void lock_jpeg_dec();
void unlock_jpeg_dec();

}
