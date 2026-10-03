#pragma once
// 板端本地目标追踪（namespace track）：从相机帧抽 luma + chroma，在其上做 ZNCC 模板匹配，
// 逐帧给出目标中心（归一化 0..1）。供"AI/手机标一个范围 → 板端自动锁定跟住 → 闭环夹取"用。
//
// 相机常态是 JPEG 直出，而本模块只认 RGB565 帧缓冲（逐像素直读，省解码）；故 seed()/stop() 会
// 经 cam::set_track_mode() 把相机在"跟踪模式 RGB565 / 常态 JPEG"之间硬切一次（幂等，只在起停各切一次）。
// 降采样用抽点（1/2^scale），只算需要的分辨率。所有像素缓冲走 MALLOC_CAP_SPIRAM（内部 DMA 堆紧张）。
#include <stdint.h>
#include <stddef.h>
#include "esp_camera.h"   // camera_fb_t：本模块 API 直接收帧

namespace track {

enum class State : uint8_t { Idle, Locking, Tracking, Lost };

struct Result {
  bool  ok;      // 本帧是否得到有效位置
  float u, v;    // 目标中心（全幅归一化 0..1）
  float conf;    // ZNCC 置信度（截断到 ≥0）
  State st;      // 本帧结束后的状态
};

const char* state_name(State s);

// ---- 诊断/内部：从一帧抽降采样 luma / chroma ----
// scale ∈ 0..3 = 1/1,1/2,1/4,1/8（抽点）；输出尺寸 = ceil(W/2^scale) × ceil(H/2^scale)。
// out 容量须 ≥ 该乘积字节。成功返回 0 并写 *gw/*gh；失败返回负值。
int decode_gray(const camera_fb_t* fb, int scale, uint8_t* out, int out_cap, int* gw, int* gh);
// 同上，但取**色度**(max(R,G,B)-min(R,G,B))——诊断用：看彩色目标在色度里是否真拉得开。
int decode_chroma(const camera_fb_t* fb, int scale, uint8_t* out, int out_cap, int* gw, int* gh);
int last_decode_ms();   // 上次取图(luma/chroma)耗时(ms)，诊断用
int last_track_ms();    // 上次 update 的"取窗+相关滤波"耗时(ms)，诊断用
// 上次 update 的**分段耗时(us)**：prep=取窗/加窗准备，search=相关滤波搜索。
// 用来定位"搜索变慢"到底慢在哪一段（真机实测曾出现 trk≈966ms/帧，需分段才能定位）。
void last_phases_us(int* prep, int* search);

// ---- 跟踪 ----
// 给一个目标：name + 中心(归一化) + 可选框宽高(归一化；0 = 用 TRACK_SEED_W/H 缺省)。
// 下一帧 update_from_fb 会以此为模板起跟。返回 false = 参数非法（未进入锁定）。
bool seed(const char* name, float cx, float cy, float w = 0, float h = 0);

// 用一帧更新位置（内部抽 luma/chroma + ZNCC 搜索）。无目标时返回 ok=false。
// light=true = "仅刷新显示位置"的轻量更新（供图传任务顺带调用，把手机标记刷新率提到图传帧率）：
//   ① 只在 Tracking 态生效；② try-lock 拿不到就立即返回（不阻塞控制线程）；③ 匹配失败**不**推进
//   miss/Lost、不改模板与计数器 —— 故不干扰 grasp/AI 对"新鲜帧"的丢目标判定。
Result update_from_fb(const camera_fb_t* fb, bool light = false);

// ---- 控制量先验（夹取闭环用）----
// 调用方已知自己刚下达的动作，把"目标下一帧大致会挪到哪"（归一化位移）告诉追踪器，直接写进已有的
// 运动预测门控（决定下一帧搜索窗心）。**比追踪器自己的图像预测可靠得多**：后者基于"动作前"那帧的位移，
// 动作后必然偏差；例如前进 4cm 若按图像预测搜，窗心还停在原地，而目标已经朝画面里挪了。
// 只在紧随其后的那一次 update 生效（update 末尾会用实测位移重写 pred）。拿不到锁就丢弃（不影响正确性）。
void hint_motion(float du, float dv);
// 同 hint_motion，但之后两帧用**宽窗多尺度**（重捕那套窗/门）：给"预期会有大位移"的动作后用
// （如超近距离的后退，1cm 的画面位移就能超出正常窗）。窗心仍在 hint 位置 —— 是预期位移补偿，非丢后重捕。
void hint_wide(float du, float dv);

void  stop();                 // 取消跟踪，回 Idle
// 跟丢后是否允许"放宽搜索窗重捕"(默认允许)。grasp 闭环里关掉: 重捕是宽搜, 抓到什么都可能。
void  set_reacquire(bool on);
bool  active();               // 是否在跟（非 Idle）
State state();
const char* target_name();
bool  last_center(float* u, float* v);   // 上次有效中心（平滑后；无则 false）——控制用
bool  last_raw_center(float* u, float* v); // 上次**未平滑**亚像素中心——手机叠加显示用（无拖尾；无则 false）
float last_conf();                       // 上次置信度（供状态/可视化上报）
float last_scale();                      // 上次搜索胜出的倍率（诊断：判断"目标变大超出尺度范围"）
// 最近一次 update 完成时的 millis()。夹取闭环靠它判断"有没有拿到**动作之后**拍的那帧" ——
// 位置不再由夹取循环自己抓帧（那要等相机出帧，实测每步 ~600ms），而是图传任务持续喂帧、这里读最新值。
unsigned long last_update_ms();
unsigned long last_capture_ms();   // 最近一次被处理帧的"拍摄时刻"（进入处理前的 millis）：夹取闭环据此要求"画面拍摄于动作结束之后"，结果到达时刻会被流水线延迟骗过
// 最近一帧是否**被采纳**（PSR 达标）。夹取闭环必须用它判断"手上这个位置是不是新鲜的"：
// 光看 state==Tracking 不够 —— 被拒的帧状态仍是 Tracking、位置还是上一帧的旧值（真机实测会照着旧位置决策）。
bool last_ok();
// 当前搜索半径（归一化 u/v，转发自相关滤波内核）——夹取闭环用它限制单步动作幅度。见 dcf.h。
void search_radius(float* ru, float* rv);
// ---- 诊断（转发自相关滤波内核，不参与决策）----
float last_obj_px();    // 本帧用的目标边长(px)；明显小于实际目标 ⇒ 窗太小、峰会在目标内部乱跳
float last_off_frac();  // 响应峰离窗心的距离 ÷ 搜索半径；接近/超过 1 ⇒ 目标贴窗边（窗小 或 单步太大）
float last_appear();    // 与**起跟那一帧**的窗的 NCC（1=一模一样）。低 ⇒ 已经不是那个物体了（防"滑到夹爪上"）
bool  dcf_mask_on();    // 相关滤波本次是否启用了"车身 mask"（诊断）
// 合爪自检探针：设两个点，下一次 update 顺便算它们"与种子的外观分"（不动跟踪状态）。
// 用途见 grasp.cpp 的 verify_grasp：比较"地面原位"与"夹持位"哪边更像目标。
void  probe_set(float u0, float v0, float u1, float v1);
void  probe_off();
float probe_appear0();
float probe_appear1();

}  // namespace track
