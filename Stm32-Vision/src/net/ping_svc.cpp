#include "src/net/ping_svc.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdlib.h>        // calloc/free
#include <esp_heap_caps.h> // MALLOC_CAP_SPIRAM：任务栈放外存（内部堆紧，见下）

#include "src/core/board_log.h"    // 结果同时进日志转发：回执只发发起方，看别的接收端会误判成"没输出"

#include "apps/ping/ping_sock.h"   // esp_ping_new_session 等
#include "lwip/api.h"              // netconn_gethostbyname_addrtype（域名解析）
#include "lwip/dns.h"              // LWIP_DNS_ADDRTYPE_IPV4（dns_addrtype 常量）
#include "lwip/ip_addr.h"          // ip4addr_aton / ip_addr_t
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// 方案：每次 start 起一个一次性任务做「域名解析（阻塞，独立任务内可接受）+ 建会话」，
// esp_ping 内部自带 ping 任务跑 count 次探测；on_ping_end 回调里收尾并发结果，
// 任务等信号量后释放并自杀——全程异步，不阻塞主 loop。

namespace ping {

// 任务栈从 PSRAM 分配（与 mvfy 同一套路，见 exec/motion_verify.cpp）。内部堆在 AI/TLS/网络起来后
// 常年只剩 4~7KB 最大块，16384 字节的**连续内部**栈走 xTaskCreate 必然失败——而那是静默失败：
// 用户只看到"正在 ping…"之后再无下文，与"网络不通"完全分不开（真机上已踩过，白排查一轮）。
// PSRAM 还有 6MB+ 空闲，ping 任务不做 TLS、无实时性要求，放外存安全。
// 同 mvfy 的硬约束：TCB 必须在内部 RAM（port 的 xPortCheckValidTCBMem 断言会拦），故 TCB 用内部
// 静态变量、栈手动分配。静态栈只有一份 ⇒ s_busy 串行化：同时只允许一个 ping 会话。
static StackType_t* s_task_stack = nullptr;
static StaticTask_t s_task_tcb;   // 内部 RAM（.bss）
static bool s_busy = false;
static constexpr uint32_t k_task_stack = 16384;

// 起不来必须出声：失败原因连体征一起报。ping 起不来几乎都是内部堆紧到分不出连续 16KB（也可能是
// 上一个会话没结束），只写"失败"看不出是哪一类；带上 块/最低 就能当场判断。
static void log_fail(const char* target, const char* why) {
  blog::logf(blog::CMD, "ping %s 未发起: %s｜堆=%uk 最低=%uk 块=%uk psram=%uk", target, why,
             (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getMinFreeHeap() / 1024),
             (unsigned)(ESP.getMaxAllocHeap() / 1024), (unsigned)(ESP.getFreePsram() / 1024));
}

struct Job {
  char target[96];
  int count;
  cmd::ReplyFn fn;
  void* ctx;              // 回复通道参数（WS=堆拷贝 int*；BLE=nullptr），回调后释放
  SemaphoreHandle_t done; // esp_ping 会话结束信号（on_ping_end → 任务）
  int sent = 0;           // 已发出探测数
  int ok = 0;             // 收到回复数
  long sum_ms = 0, min_ms = 0, max_ms = 0;  // 延迟统计（ms）
};

static void on_success(esp_ping_handle_t hdl, void* args) {
  Job* j = (Job*)args;
  uint32_t gap = 0;
  if (esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &gap, sizeof(gap)) == ESP_OK) {
    j->sent++;
    j->ok++;
    j->sum_ms += gap;
    if (!j->min_ms || gap < j->min_ms) j->min_ms = gap;
    if (gap > j->max_ms) j->max_ms = gap;
  }
}

static void on_timeout(esp_ping_handle_t hdl, void* args) {
  Job* j = (Job*)args;
  j->sent++;
}

// 会话结束：**只放信号**。
// ⚠️ 这个回调跑在 esp_ping 自己那个任务上，栈是 IDF 默认的 `ESP_TASK_PING_STACK`
// （esp_task.h 里 = 2048 + 扩展，约 2KB）——**不是**我们的 16KB PSRAM 栈。
// 原先这里就地组 JSON + snprintf(192) + 调回执 + blog::logf，几百字节的栈开销直接踩穿 2KB；
// 而本板 `FREERTOS_WATCHPOINT_END_OF_STACK=y`，溢出由硬件观察点**即时**触发 panic，
// 一行日志都来不及吐 ⇒ 现场只剩「ping 永远没下文 + 板子悄悄重启（复位=崩溃）」。
// 这就是长期看到的"ping 静默"，与内存不足/链路不通都无关，别再往那两个方向查。
// 结论：此处只允许几十字节量级的操作，组包/回执一律交给 session_task 的 16KB PSRAM 栈。
static void on_end(esp_ping_handle_t hdl, void* args) {
  Job* j = (Job*)args;
  esp_ping_delete_session(hdl);  // 清 INIT 标志 → ping 任务随后自行退出并释放会话
  xSemaphoreGive(j->done);       // 本句之后不得再碰 j：session_task 会立刻 free(j)
}

// 回执（status 帧）只发给发起方那一个 fd —— 手机发的 /ping，电脑那侧永远看不到。
// 排查时这等于没有观察窗口，故每条结果额外走一遍 blog 转发（cat=all 时 WS+BLE 都能收到）。
static void send_status(cmd::ReplyFn fn, void* ctx, const char* reason) {
  JsonDocument out;
  out["type"] = "status";
  out["params"]["reason"] = reason;
  String s;
  serializeJson(out, s);
  blog::logf(blog::CMD, "%s", reason);
  if (fn) fn(ctx, s.c_str());
}

// 会话任务的每条出口都要清 s_busy（否则后续 ping 全被判"上一个还在进行"）。
// 自杀式 vTaskDelete(NULL) 之前清即可：此时本函数已不再需要任务本身存活。
static void session_task(void* arg) {
  Job* j = (Job*)arg;

  // 目标解析：IPv4 直用；域名走 netconn 阻塞解析（带超时，独立任务内可接受）。
  ip_addr_t target = {};
  target.type = IPADDR_TYPE_V4;
  if (!ip4addr_aton(j->target, &target.u_addr.ip4)) {
    ip_addr_t res;
    if (netconn_gethostbyname_addrtype(j->target, &res, LWIP_DNS_ADDRTYPE_IPV4) != ERR_OK) {
      char text[128];
      snprintf(text, sizeof(text), "ping %s: 域名解析失败", j->target);
      send_status(j->fn, j->ctx, text);
      if (j->ctx) delete (int*)j->ctx;
      free(j);
      s_busy = false;
      vTaskDelete(NULL);
      return;
    }
    target = res;
  }

  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.count = j->count;
  cfg.timeout_ms = 3000;   // 单次探测超时 3s（与项目既有 ping 语义一致）
  cfg.interval_ms = 1000;
  cfg.data_size = 64;
  cfg.target_addr = target;

  esp_ping_callbacks_t cbs = {};
  cbs.cb_args = j;
  cbs.on_ping_success = on_success;
  cbs.on_ping_timeout = on_timeout;
  cbs.on_ping_end = on_end;

  esp_ping_handle_t hdl = nullptr;
  if (esp_ping_new_session(&cfg, &cbs, &hdl) != ESP_OK) {
    send_status(j->fn, j->ctx, "ping 会话创建失败");
    if (j->ctx) delete (int*)j->ctx;
    free(j);
    s_busy = false;
    vTaskDelete(NULL);
    return;
  }
  esp_ping_start(hdl);
  xSemaphoreTake(j->done, portMAX_DELAY);  // 等 on_ping_end 放行；统计量已由回调累加进 j
  // 组包与回执放**这里**、不放 on_end：本任务栈是自己分的 16KB PSRAM，而 on_end 跑在
  // esp_ping 的 ~2KB 栈上（详见 on_end 注释）。send_status 兼管 JSON 组包 + 日志转发 + 回执。
  char text[192];
  if (j->ok > 0) {
    int timeout = j->sent - j->ok;
    if (timeout > 0) {
      snprintf(text, sizeof(text), "ping %s: %d/%d 回复（%d 超时）, 平均 %lums, 最小 %lums, 最大 %lums",
               j->target, j->ok, j->sent, timeout, j->sum_ms / j->ok, j->min_ms, j->max_ms);
    } else {
      snprintf(text, sizeof(text), "ping %s: %d 回复, 平均 %lums, 最小 %lums, 最大 %lums",
               j->target, j->ok, j->sum_ms / j->ok, j->min_ms, j->max_ms);
    }
  } else {
    snprintf(text, sizeof(text), "ping %s: 超时（%d 次均无响应）", j->target, j->sent);
  }
  send_status(j->fn, j->ctx, text);
  if (j->ctx) delete (int*)j->ctx;
  free(j);
  s_busy = false;
  vTaskDelete(NULL);
}

bool start(const char* target, int count, cmd::ReplyFn reply, void* reply_ctx) {
  if (!target || !target[0] || count <= 0) return false;
  // 上一个会话还没结束。正常最多几十秒；若这条一直出现，说明上一会话卡住了（域名解析那条路在
  // lwIP 的 tcpip 线程卡死时永远不返回——这也是排查卡死时要用 IP 字面量的原因）。
  if (s_busy) {
    log_fail(target, "上一个 ping 仍在进行（若已过很久 = 上一会话卡住）");
    return false;
  }
  Job* j = (Job*)calloc(1, sizeof(Job));
  if (!j) {
    log_fail(target, "Job 分配失败");
    return false;
  }
  strncpy(j->target, target, sizeof(j->target) - 1);
  j->target[sizeof(j->target) - 1] = 0;
  j->count = count;
  j->fn = reply;
  // WS 场景 reply_ctx 指向 ws_handler 栈上 fd：必须堆拷贝供异步回调使用；BLE 为 nullptr。
  j->ctx = reply_ctx ? new int(*(int*)reply_ctx) : nullptr;
  j->done = xSemaphoreCreateBinary();
  if (!j->done) {
    if (j->ctx) delete (int*)j->ctx;
    free(j);
    log_fail(target, "信号量创建失败");
    return false;
  }
  // 栈只分一次、长期持有（PSRAM 不会碎片化到分不出 16KB）。
  if (!s_task_stack) s_task_stack = (StackType_t*)heap_caps_malloc(k_task_stack, MALLOC_CAP_SPIRAM);
  if (!s_task_stack) {
    if (j->ctx) delete (int*)j->ctx;
    vSemaphoreDelete(j->done);
    free(j);
    log_fail(target, "PSRAM 栈分配失败");
    return false;
  }
  s_busy = true;   // xTaskCreateStatic 起不来时要回滚（没有任务会替我们清）
  if (!xTaskCreateStaticPinnedToCore(session_task, "ping_svc", k_task_stack, j, 2, s_task_stack,
                                     &s_task_tcb, 1)) {
    s_busy = false;
    if (j->ctx) delete (int*)j->ctx;
    vSemaphoreDelete(j->done);
    free(j);
    log_fail(target, "任务创建失败");
    return false;
  }
  return true;
}

}  // namespace ping
