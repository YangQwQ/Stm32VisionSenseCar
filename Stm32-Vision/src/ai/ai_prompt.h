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

// 构建请求 body 的入参。字段多且含多组同类型指针(三组"图+长度"), 用具名字段代替位置参数。
struct BodyReq {
  const char* goal = nullptr;        // 当前任务目标(可被插话/ task_goal 热替换)
  const char* ann = nullptr;         // 操作者标注
  const char* hint = nullptr;        // 本轮"注意:"提示
  const char* const* hrole = nullptr;  // 历史环: 角色数组(assistant=AI决策 / user=插话)
  const char* const* htext = nullptr;  // 历史环: 文本数组
  int hn = 0;                          // 历史环条数
  const char* exec_state = nullptr;  // 执行板状态一行文本(无数据为空串)
  unsigned last_age_s = 0;           // 距上次执行秒数
  const char* note = nullptr;        // 任务笔记
  const char* prog = nullptr;        // 已渲染的"任务列表: ..."文本
  ImgRef frame;                      // 本轮实时帧(系统画面)
  ImgRef prev;                       // 上一帧/放大帧(carry 通道)
  bool use_prev = false;
  ImgRef edited;                     // 用户发送的参考图
  bool use_edited = false;
};

// 系统提示词(角色+规则+JSON格式+标定, 不含目标)+ 独立 user(目标) 消息先组进 PSRAM。
void build_body(PsaBuf& b, const BodyReq& r);