#pragma once
#include <Arduino.h>
#include <Stream.h>      // PieceStream: 喂给 HTTPClient::sendRequest
#include <stdio.h>
#include <stdlib.h>      // free
#include <string.h>      // strlen/memcpy
#include <esp_heap_caps.h>  // heap_caps_realloc / MALLOC_CAP_SPIRAM(PsaBuf ensure)

// ================ PSRAM 增长缓冲(全局共享, 组请求 body 用) ================
struct PsaBuf {
  char* p = nullptr;
  size_t len = 0, cap = 0;
  bool ok = true;
  ~PsaBuf() { if (p) free(p); }
  bool ensure(size_t need) {
    if (len + need + 1 <= cap) return true;
    size_t nc = cap ? cap * 2 : 16 * 1024;
    while (nc < len + need + 1) nc *= 2;
    char* np = (char*)heap_caps_realloc(p, nc, MALLOC_CAP_SPIRAM);
    if (!np) { ok = false; return false; }
    p = np; cap = nc; return true;
  }
  void put(const char* s) { size_t n = strlen(s); if (!ok || !ensure(n)) { ok = false; return; } memcpy(p + len, s, n); len += n; p[len] = 0; }
  void put(char c) { if (!ok || !ensure(1)) { ok = false; return; } p[len++] = c; p[len] = 0; }
  // 批量追加原始字节(响应正文收包用): 长度显式给出, 不按 C 串处理。
  void append(const uint8_t* d, size_t n) { if (!ok || !ensure(n)) { ok = false; return; } memcpy(p + len, d, n); len += n; p[len] = 0; }
  // 清空但**保留容量**: 每轮响应都从零开始, 复用同一块可彻底免掉反复 realloc。
  void clear() { len = 0; if (p) p[0] = 0; }
  const char* c_str() const { return p ? p : ""; }
};

// 帧字节引用: JPEG(指针, 长度)必须成对传递, 拆成两个位置参数极易调错
struct ImgRef {
  const uint8_t* p = nullptr;
  size_t n = 0;
  bool hi = false;   // true=detail:"high"(放大/精判图); false=detail:"low"(全幅, 省 token/省 prefill)
};

// ================ agent 循环的历史视图 ================
// 历史按"回合"回喂(tool 协议): 每回合 1 条 assistant(tool_calls, 可含多个调用) + 每个调用 1 条 tool 结果;
// 用户发言(初始目标/后续补充)是独立的一条 user 消息。⚠️ 字符串全在 PSRAM, 由 ai_round 分配/释放, 这里只借用指针。
// 图**只对"最新一组实图"注入字节**(调用方把 imgs 填好); 更早的按全局编号渲染成占位说明(见 build_body)。
#define AI_HIST_CALL_MAX 6      // 单回合最多回喂几个调用(mem/task/say/car/look/compact/goal 已是峰值)
// 先前帧环: 保留最近几张"发出去给模型看过的实景", 供 look(回看 ImageN) 回看。
// 环由 ai_round 维护; 尺寸即为环槽数(历史占位只按编号写死, 不再每轮现算还能回看哪张)。
#define AI_PREV_SLOTS 3

struct HistCall {
  char* strbuf = nullptr;    // id/name/args/result 四段字符串同块 PSRAM 分配, 一次释放; 下列指针指入其中
  char* id = nullptr;        // tool_call id(assistant 与 tool 配对)
  char* name = nullptr;      // 工具名(car/mem/task/goal/look/say/compact)
  char* args = nullptr;      // 模型给的参数原文(回喂 = AI 的实际输出)
  char* result = nullptr;    // 我方执行结果文本
  ImgRef imgs[2];            // 该结果所带的图(最多两张); 只有"最新一组"才填字节
  uint32_t img_id[2] = {};   // 各图的全局编号(ImageN): 字节已省略时按它渲染占位说明
  uint8_t img_n = 0;         // 当时带了几张图(0~2)
};

struct HistTurn {
  bool chat = false;                   // true = 用户消息(整条渲染成 user 文本)
  bool origin = false;                 // true = 任务起点的用户目标(原话已常驻目标消息, 淘汰时不必提醒更新)
  char* text = nullptr;                // chat 时用
  char* reasoning = nullptr;           // 该回合模型的思考原文(reasoning_content); 带 tools 的请求必须回传, 不回传模型每轮都得从头重推
  HistCall calls[AI_HIST_CALL_MAX];    // chat=false 时用
  int ncall = 0;
  uint16_t img_tag = 0;                // 图片归属标记: 与 RoundCtx::img_owner 配对, 认出本回合挂的图是否还没发出去
};

// 构建请求 body 的入参(agent 循环)。字段多且含多组同类型指针, 用具名字段代替位置参数。
struct BodyReq {
  const HistTurn* turns = nullptr;   // 历史回合(含用户消息), 按时间序
  int turn_n = 0;
  const char* state_block = nullptr; // 每轮现拼的任务状态块(目标/列表/笔记), 挂最新一条 tool 结果尾部; ⚠️ 不落历史
  const char* tail_hint = nullptr;   // 一次性提示, 挂在同一条尾部
  ImgRef frame;                      // 尾部 user 画面(首轮/本回合没有 look 时由程序注入)
  bool use_frame = false;
  const char* frame_note = nullptr;  // 画面文本前缀(无画面时也用它给出警告文案)
};

// ================ 请求体碎片流 ================
// 请求体不再整块拼接, 而是"碎片序列": 固定部分(系统提示词/tools 声明)直接引用静态字节, JPEG 走 B64
// 边发边编码。total = 各碎片输出字节之和 = Content-Length, 直接交给 HTTPClient::sendRequest。
struct Piece {
  enum Kind : uint8_t { TEXT, B64 };
  Kind kind = TEXT;
  const uint8_t* p = nullptr;   // TEXT: 文本字节; B64: JPEG 原始字节
  size_t n = 0;                 // 原始字节数(TEXT=文本长; B64=JPEG 长)
  size_t out_n = 0;             // 输出字节数(TEXT=n; B64=4*((n+2)/3))
  bool own = false;             // p 由本列表分配(渲染出来的文本), 析构时释放
};

struct PieceList {
  Piece* it = nullptr;
  int n = 0, cap = 0;
  size_t total = 0;             // 输出字节总数(= Content-Length)
  bool ok = true;

  PieceList() = default;
  PieceList(const PieceList&) = delete;
  PieceList& operator=(const PieceList&) = delete;
  ~PieceList();

  void add_text(const char* s, size_t len);      // 借用静态/外部缓冲(须活到发送完成)
  void add_b64(const uint8_t* jpg, size_t len);  // JPEG 原始字节, 发送时编码
  void take(PsaBuf& b);                          // 接管 b 的缓冲作为 TEXT 碎片, 并把 b 置空

private:
  Piece& push();
  Piece scratch;                // push 失败时的占位返回
};

// 把碎片序列喂给 HTTPClient: readBytes 逐碎片取, B64 碎片当场编码。
// rewind() 可重播(pass1 重连后重发同一 body), 因每个碎片的输出都由固定的 (指针, 偏移) 决定, 字节必然一致。
class PieceStream : public Stream {
public:
  explicit PieceStream(PieceList& l) : l_(l) { rewind(); }
  void rewind() { idx_ = 0; off_ = 0; done_ = 0; }
  int available() override { return (int)(l_.total - done_); }
  int read() override;
  int peek() override;
  size_t readBytes(char* buf, size_t len) override;
  size_t write(uint8_t) override { return 0; }   // 只读流: 不实现写入

private:
  PieceList& l_;
  int idx_ = 0;        // 当前碎片
  size_t off_ = 0;     // 当前碎片已产出的输出字节
  size_t done_ = 0;    // 已产出的总输出字节(available 用)
};

// 系统提示词(角色+规则+标定) + 历史回合 + 尾部(状态块/画面), 组进碎片列表。
void build_body(PieceList& l, const BodyReq& r);