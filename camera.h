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
#define CAMERA_MODEL_AI_THINKER // Has PSRAM（当前板，沿用出厂默认）
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
//#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3 // Has PSRAM
//#define CAMERA_MODEL_DFRobot_Romeo_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#include "esp_camera.h"

namespace cam {

// 初始化摄像头（JPEG 输出，双缓冲 + GRAB_LATEST）
bool init();

// 取一帧（同步接口），使用后必须调用 return_frame() 归还缓冲
camera_fb_t* grab();

// 归还帧缓冲
void return_frame(camera_fb_t* fb);

}
