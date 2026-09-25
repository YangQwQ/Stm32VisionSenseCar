#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// 作用域锁守卫: 构造取锁、析构放锁 —— 中途 return/break 就再也不会漏 unlock。
// 只有真取到锁时 held()==true 并在析构时释放; 句柄为空或超时未取到则 held()==false, 什么都不做。
// 只圈住需要互斥的那几行: 用 `{ ScopedLock lk(m); ... }` 显式限界, 别把慢操作圈进临界区。
struct ScopedLock {
  explicit ScopedLock(SemaphoreHandle_t m, TickType_t wait = portMAX_DELAY) : h(m) {
    if (h && xSemaphoreTake(h, wait) != pdTRUE) h = nullptr;
  }
  ~ScopedLock() { if (h) xSemaphoreGive(h); }
  ScopedLock(const ScopedLock&) = delete;
  ScopedLock& operator=(const ScopedLock&) = delete;

  bool held() const { return h != nullptr; }

 private:
  SemaphoreHandle_t h;
};