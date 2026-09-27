#pragma once
#include <Arduino.h>
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
    size_t nc = cap ? cap * 2 : 1024 * 1024;
    while (nc < len + need + 1) nc *= 2;
    char* np = (char*)heap_caps_realloc(p, nc, MALLOC_CAP_SPIRAM);
    if (!np) { ok = false; return false; }
    p = np; cap = nc; return true;
  }
  void put(const char* s) { if (!ok || !ensure(strlen(s)) ) { ok = false; return; } memcpy(p + len, s, strlen(s)); len += strlen(s); p[len] = 0; }
  void put(char c) { if (!ok || !ensure(1)) { ok = false; return; } p[len++] = c; p[len] = 0; }
};

// 帧字节引用: JPEG(指针, 长度)必须成对传递, 拆成两个位置参数极易调错
struct ImgRef {
  const uint8_t* p = nullptr;
  size_t n = 0;
};

// ================ agent 循环的历史视图 ================
// 历史按"回合"回喂(tool 协议): 每回合 1 条 assistant(tool_calls, 可含多个调用) + 每个调用 1 条 tool 结果;
// 用户发言(初始目标/后续补充)是独立的一条 user 消息。⚠️ 字符串全在 PSRAM, 由 ai_round 分配/释放, 这里只借用指针。
// 图**只对"最新一组实图"注入字节**(调用方把 imgs 填好); 更早的按 note 渲染成占位说明(见 build_body)。
#define AI_HIST_CALL_MAX 6      // 单回合最多回喂几个调用(mem+say+car+look+compact+finish 已是峰值)
// 先前帧环: 保留最近几张"发出去给模型看过的实景", 供 look(prev:N) 回看(N=1 即上一张看过的)。
// 环由 ai_round 维护; 历史占位要按它标注"此刻还能回看的编号", 故两处共用这个尺寸。
#define AI_PREV_SLOTS 3

struct HistCall {
  char* id = nullptr;        // tool_call id(assistant 与 tool 配对)
  char* name = nullptr;      // 工具名(car/look/mem/say/compact/finish)
  char* args = nullptr;      // 模型给的参数原文(回喂 = AI 的实际输出)
  char* result = nullptr;    // 我方执行结果文本
  ImgRef imgs[2];            // 该结果所带的图(最多两张: 新拍/上一张实景/用户参考图 的组合); 只有"最新一组"才填字节
  uint32_t img_id[2] = {};   // 各图在先前帧环里的帧序号(0=非环内实景, 如用户参考图): 占位按它标出可回看的编号
  uint8_t img_n = 0;         // 当时带了几张图(0~2); 字节已省略时按 note 渲染占位说明
  char note[96] = {0};       // 该结果所带图的性质(如"新拍全幅 + 用户第1张参考图"); 空=不带图
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
  const uint32_t* prev_ids = nullptr;// 先前帧环各槽的帧序号(新→旧; 0=空槽): 历史占位据此算出"还能用 prev:N 回看"
  int prev_idn = 0;
  uint32_t prev_new_id = 0;          // 本回合正要发出去的实景帧序号(0=本回合不滚新帧): 组包后它成为 prev1, 占位偏移要按滚过之后的环算
};

// 系统提示词(角色+规则+标定) + 历史回合 + 尾部(状态块/画面), 组进 PSRAM。
void build_body(PsaBuf& b, const BodyReq& r);