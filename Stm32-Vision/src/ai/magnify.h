#pragma once
// 放大镜: 把一帧 JPEG 的局部裁出来放大重编码, 让 AI 能"凑近看"目标与夹爪的相对位置。
//
// 为什么是它而不是颜色检测(见 ai_client 调用处的长注释): AI 用眼睛在全幅里估绝对像素不可靠
// (目标只占二十几像素宽), 单应标定本身又有厘米级残差, 两者叠加后"它以为对准了"和"真对准了"
// 分不开。放大镜不引入任何颜色/形状/目标的假设 —— 它只是把 AI 自己的眼睛凑近, 裁框由程序记账,
// 于是 AI 报的像素换回全幅是**精确算术**而非估计。换任何目标都成立。
#include <stdint.h>
#include <stddef.h>

namespace magnify {

// jpg: 源帧(相机原始 JPEG); [x0,x1)×[y0,y1) 为**全幅归一化**裁框(自动夹到画面内并保证不小于
// 最小边长); 输出重编码为 out_w×out_h 的 JPEG 写进 out(cap 字节上限)。失败返回 false。
bool crop_to_jpg(const uint8_t* jpg, size_t len, int src_w, int src_h,
                 float x0, float y0, float x1, float y1,
                 uint8_t* out, size_t cap, size_t* out_len,
                 int out_w, int out_h, int quality);

int last_cost_ms();   // 上次 crop_to_jpg 的总耗时(解码+放大+编码)
int last_src_px();    // 上次解码出的源像素数(诊断用; 0 = 没解码成功)

}  // namespace magnify
