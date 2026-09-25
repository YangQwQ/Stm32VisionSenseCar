#pragma once
// 放大镜: 把一帧 JPEG 的中央裁出重编码, 让 AI 能"凑近看"目标与夹爪的相对位置。
// 裁框固定中央由程序记账, AI 报的像素换回全幅是精确算术, 不引入任何颜色/形状/目标的假设。
#include <stdint.h>
#include <stddef.h>

namespace magnify {

// TJpgDec 部分解码裁中央: 只解出中央 (0.25,0.25)-(0.75,0.75) 区域并重编码为 JPEG。
// workbuf 与输出缓冲全走 MALLOC_CAP_SPIRAM, 不占内部 DMA/堆(规避 fmt2rgb888 整幅软解卡死); 失败返回 false。
bool crop_center_jpg(const uint8_t* jpg, size_t len, int src_w, int src_h,
                     uint8_t* out, size_t cap, size_t* out_len,
                     int out_w, int out_h, int quality);

int last_cost_ms();   // 上次放大镜的总耗时(解码+重编码)，诊断用；0 = 上次没成功

}  // namespace magnify