#pragma once
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>      // free
#include <string.h>      // strlen/memcpy
#include <esp_heap_caps.h>  // heap_caps_realloc / MALLOC_CAP_SPIRAM(PsaBuf ensure)

// ================ PSRAM 增长缓冲(全局可用: ai_client 的 worker 亦用它组 body) ================
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

// 构建请求 body 的结构说明
// goal 当前任务目标(可被插话/ task_goal 热替换); hrole/htext/hn = 历史环条目(角色+文本,
// 同时含 assistant=AI 决策 与 user=插话), 逐条作为独立消息回喂, 构成真多轮对话记录;
// exec_state 执行板状态一行文本(无数据为空串)及"距上次执行"秒数喂当前 user。
// 系统提示词？(角色+规则+JSON格式+标定, 不含目标)+ 独立 user(目标) 消息先组进 PSRAM。
// 图预算 ≤2: carry 帧(prev/放大/用户图)优先(放弃参考图), 否则 参考图(首轮)+当前帧。
void build_body(PsaBuf& b, const char* goal, const char* ann, const char* hint,
                const char* const* hrole, const char* const* htext, int hn,
                const char* exec_state, unsigned last_age_s,
                const char* note, const char* prog,
                const uint8_t* frame, size_t frame_len,
                const uint8_t* prev, size_t prev_len,
                bool use_prev, bool use_edited, const uint8_t* edited, size_t edited_len);