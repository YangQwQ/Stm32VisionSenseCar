#pragma once

#include "esp_camera.h"

namespace cam {

// 初始化摄像头（JPEG 输出，双缓冲 + GRAB_LATEST）
bool init();

// 取一帧（同步接口），使用后必须调用 return_frame() 归还缓冲
camera_fb_t* grab();

// 归还帧缓冲
void return_frame(camera_fb_t* fb);

}
