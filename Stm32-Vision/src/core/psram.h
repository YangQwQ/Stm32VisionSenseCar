#pragma once
#include <esp_heap_caps.h>
#include <stdlib.h>
#include <string.h>

// PSRAM 上的**变长字符串**。
// 用于"长度由模型/用户当场决定"的那几段文本(任务名 / 任务笔记 / 当前目标 / 物体名): 定长缓冲要么
// 写死一个上限(超过就静默切断), 要么按字节切断(残尾再被各处出口消毒换成 '?')。这类文本的长度在
// 写入那一刻是**已知**的, 所以没理由写死 —— 一律走这里按实际长度分配。
// 放 PSRAM 而不是内部堆: 内部 RAM 才是这块板真正稀缺的(DMA 块/碎片是头号闸门), PSRAM 有 8MB。

// 复制一份到 PSRAM。s 为空 → nullptr; 分配失败 → nullptr(调用方负责决定怎么降级)。
inline char* ps_dup(const char* s) {
  if (!s) return nullptr;
  size_t n = strlen(s);
  char* p = (char*)heap_caps_malloc(n + 1, MALLOC_CAP_SPIRAM);
  if (!p) return nullptr;
  memcpy(p, s, n + 1);
  return p;
}

// 释放并置空(可反复调用)。
inline void ps_free(char*& p) {
  if (p) { free(p); p = nullptr; }
}

// 替换一段文本。**先分配后释放**: 分配失败时保留旧值并返回 false —— 不把已经有的内容弄丢。
// s 为空串视同"清空"。
inline bool ps_set(char*& dst, const char* s) {
  if (!s || !s[0]) { ps_free(dst); return true; }
  char* np = ps_dup(s);
  if (!np) return false;
  ps_free(dst);
  dst = np;
  return true;
}

// 读取: 空指针一律当空串, 调用方不必到处判 null。
inline const char* ps_str(const char* p) { return p ? p : ""; }
