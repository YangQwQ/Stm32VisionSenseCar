#pragma once
#include <Arduino.h>

// 本地自动夹取（namespace grasp）：手机标一个物体在画面上的位置 → 板端自己完成
// "arm low → 转正对准 u → 边前进边对准直到 v → 合爪"。**纯画面闭环，不经 AI、不用单应/全局坐标。**
//   1) 先降到低姿准备位
//   2) 原地小角对准，把物体转到画面 u≈GRASP_U_TGT
//   3) 前进（每步后重新对准），直到物体到画面 v≈GRASP_V_TGT
//   4) 合爪（exec 的 grasp 含合后抬臂）
// 全程用 track（本地 ZNCC 追踪）提供目标的实时 (u,v)；跟丢/被打断/超时即中止。
namespace grasp {

// 启动 worker 任务（setup 早期调一次，须在 cam/exec 之后）。
void init();

// 请求一次自动夹取。x,y = 物体在画面上的位置（归一化 0~1）；w,h = 可选框宽高（0 = 用缺省框）；
// name = 可选目标名（空 = "目标"）。返回 false = 已有一次在跑 / 参数非法。
bool request(float x, float y, float w, float h, const char* name);

bool busy();     // 是否有一次夹取在跑（含排队）
void cancel();   // 中止当前夹取（内部走 ai::cancel → 代际号自增，循环下一拍即退出）

// 上一次夹取的**结果文本**（给 AI 侧当回执用）。取值：
//   "已夹取(可能)" / "确定未夹住(方块仍在地面原位)" / "跟丢目标" / "目标不随动作移动/后退超限, 中止" …
// ⚠️ "已夹取(可能)" 只表示**没抓到"方块还在地上"的证据**，不等于一定夹住了；判定规则见 grasp.cpp 的 verify_grasp。
const char* last_result();

}  // namespace grasp
