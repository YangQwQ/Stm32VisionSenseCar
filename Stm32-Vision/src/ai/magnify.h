#pragma once
// 放大镜: 把一帧 JPEG 的中央裁出来重编码, 让 AI 能"凑近看"目标与夹爪的相对位置。
// 为什么是它而不是颜色检测(见 ai_client 调用处的长注释): AI 用眼睛在全幅里估绝对像素不可靠
// (目标只占二十几像素宽), 单应标定本身又有厘米级残差, 两者叠加后"它以为对准了"和"真对准了"
// 分不开。放大镜不引入任何颜色/形状/目标的假设 —— 它只是把 AI 自己的眼睛凑近, 裁框固定中央
// 由程序记账, 于是 AI 报的像素换回全幅是**精确算术**而非估计。换任何目标都成立。
#include <stdint.h>
#include <stddef.h>

namespace magnify {

// TJpgDec 部分解码裁中央：对整幅 JPEG 只解出**中央 (0.25,0.25)-(0.75,0.75) 区域**并重编码为 JPEG。
// 解码 workbuf 与输出缓冲全部 MALLOC_CAP_SPIRAM(PSRAM)，不占用内部 DMA/堆 —— 规避 fmt2rgb888
// 软解整幅吃内部堆的卡死。TJpgDec 熵解码仍需整幅串行跑完(省内存不省 CPU)，但只把落在中央的 MCU
// 像素落进缓冲，PSRAM 只划中央带那么大，输出为 out_w×out_h 的 JPEG 写进 out(cap 上限)。失败返回 false。
bool crop_center_jpg(const uint8_t* jpg, size_t len, int src_w, int src_h,
                     uint8_t* out, size_t cap, size_t* out_len,
                     int out_w, int out_h, int quality);

int last_cost_ms();   // 上次放大镜的总耗时(解码+重编码)，诊断用；0 = 上次没成功

}  // namespace magnify