#pragma once
#include <Arduino.h>     // size_t / uint32_t

// 单帧留档上限（超过的帧不留档，极少见）。取图端 /ai_frame 按此申请接收缓冲，故对外可见。
#define AI_DUMP_FRAME_MAX (96 * 1024)

// AI 抓帧留档（调试用，纯只读旁路，不参与任何决策）。
//
// 动机：AI 每轮据一帧画面决策，但板端只把"决策/坐标"写进日志，画面本身转瞬即弃 —— 事后复盘
// "它到底看见了什么"无从查起，于是"我以为它看见了"和"它实际收到的那张图"永远对不上账。
// 本模块把**实际发往云端的那一帧**（原始 JPEG 字节）+ 这一轮的简短标注留在 PSRAM 环形缓冲里，
// 经 HTTP 取回本地（见 app_httpd 的 /ai_dump 与 /ai_frame?seq=N），由 tools/car_logcat.py 收工时落盘
// （它同时收 WS 日志，归档目录与那份日志同名，图与文字日志因此天然对得上）。
//
// 开关：沿用现有的 `/log ai on`（blog 的 AI 类别），**不动协议**。关着时 push 立即返回、不做一次
// memcpy、并在关掉的那一刻把已占的 PSRAM 全部还给堆 —— 平时不给 AI 链路添内存压力（这条链路对
// 内存敏感，见 ai_client 顶部注释）。开着时上限 AI_DUMP_SLOTS × 单帧实际大小（VGA q10 约 40~70KB）。
//
// 取帧按**序号**而非"第几个"：环形缓冲会滚动，按位置取会在并发下错位（读到相邻那轮的画面，
// 而标注是这一轮的 —— 复盘时最怕这种错配）。故 seq 只在标注落定后才生效，清单里只出现"已定案"的帧。
namespace ai {


// 留存一帧（由 ai_worker 在组包前调用）。img=JPEG 原始字节（可为空表示本轮无画面）。
// with_prev=本轮除这帧外**还带了上一帧**（AI 上轮要了 carry_image:"full" 做运动对比）—— 上一帧就是上一个
// 槽位，所以这一个标志就能还原"它当时看到的是几张图"。留档范围仅此两种：操作者下发的参考图
// （只首轮带一次）不留档，需要时从手机侧那份原图对。
// 同一轮内重复调用（组包失败重试）只占一个槽位、就地覆盖。
void dump_push(const uint8_t* img, size_t n, bool with_prev);

// 给最近一次 push 的那帧补上标注（这一轮做了什么/被谁拒了），并让它对 HTTP 可见。
// note 超长会在 UTF-8 边界截断（截半个中文字节会让清单变非法 UTF-8）。
void dump_note(const char* note);

// 清单（JSON 文本，最新帧在前）：
//   {"slots":N,"n":可用帧数,"frames":[{"seq":..,"ms":..,"len":..,"prev":true,"note":".."}]}
//   len=0 表示那轮没画面（抓帧失败/过大），仍占一条以便看出"某一轮 AI 是瞎着决策的"。
// 返回 false 只在真出错时（取锁超时 / 缓冲装不下）；"还没留过档"返回 true + 空清单。
bool dump_manifest(char* buf, size_t cap);

// 按序号取帧：把帧字节**拷进调用方给的缓冲**（持锁仅限这次 memcpy，不阻塞 AI 线程做 TLS）。
// 返回拷贝字节数；序号不存在/本轮无画面/缓冲不足返回 0。ms 回填该帧的 millis()（可空）。
size_t dump_copy(uint32_t seq, uint8_t* dst, size_t cap, uint32_t* ms);

// 当前最新的**已定案**帧序号（0 = 一帧都没定案 / 取锁超时 —— 两者对调用方同义："还没有新的"）。
// 供 /ai_dump?after=N 长轮询判断"有没有新东西可报"，故必须是廉价且不加锁等待的查询。
uint32_t dump_newest_seq(void);

}  // namespace ai
