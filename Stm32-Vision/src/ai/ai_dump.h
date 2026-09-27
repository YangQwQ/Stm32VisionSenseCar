#pragma once
#include <Arduino.h>     // size_t / uint32_t

// 单帧留档上限（超过的帧不留档，极少见）。取图端 /ai_frame 按此申请接收缓冲，故对外可见。
#define AI_DUMP_FRAME_MAX (96 * 1024)

// AI 抓帧留档(调试用只读旁路, 不参与决策): 把实际发往云端的帧(JPEG)+本轮标注留在 PSRAM 环形,
// 经 HTTP(/ai_dump、/ai_frame?seq=N)取回; 开关沿用 /log ai on, 关时立即把已占 PSRAM 还给堆。
namespace ai {


// 留存一帧(组包前调用)。img=JPEG 原始字节(空=本轮无画面); with_prev=本轮除这帧外还带了另一张图
// (look 一次给了两张做对比)。同一轮重复调用(组包重试)只占一个槽位、就地覆盖。
void dump_push(const uint8_t* img, size_t n, bool with_prev);

// 给最近一次 push 的那帧补上标注(这一轮做了什么/被谁拒了), 并让它对 HTTP 可见。
// note 超长会在 UTF-8 边界截断(截半个中文字节会让清单变非法 UTF-8)。
void dump_note(const char* note);

// 清单(JSON 文本, 最新帧在前): {"slots":N,"n":可用帧数,"frames":[{seq,ms,len,prev,note},...]};
// len=0 表示那轮没画面, 仍占一条以便看出"某轮 AI 是瞎着决策的"。返回 false 仅在真出错时。
bool dump_manifest(char* buf, size_t cap);

// 按序号取帧: 把帧字节拷进调用方给的缓冲(持锁仅限这次 memcpy, 不阻塞 AI 线程做 TLS)。
// 返回拷贝字节数; 序号不存在/本轮无画面/缓冲不足返回 0。ms 回填该帧 millis()(可空)。
size_t dump_copy(uint32_t seq, uint8_t* dst, size_t cap, uint32_t* ms);

// 当前最新的已定案帧序号(0=一帧都没定案/取锁超时, 对调用方同义"还没有新的"); 供 /ai_dump?after=N 长轮询。
uint32_t dump_newest_seq(void);

}  // namespace ai
