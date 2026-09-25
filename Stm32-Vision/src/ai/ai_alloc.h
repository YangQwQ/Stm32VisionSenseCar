#pragma once
#include <ArduinoJson.h>
#include <esp_heap_caps.h>

// ArduinoJson 内存池改用 PSRAM, 避免小分配与 TLS 缓冲交错把内部堆切成碎块(致握手失败)。
// 定义(唯一处)在 ai_client.cpp; 各模块 include 本头取用。
struct PsramAllocator : public ArduinoJson::Allocator {
  void* allocate(size_t n) override { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM); }
  void deallocate(void* p) override { if (p) heap_caps_free(p); }
  void* reallocate(void* p, size_t n) override { return heap_caps_realloc(p, n, MALLOC_CAP_SPIRAM); }
};
extern PsramAllocator g_js_alloc;