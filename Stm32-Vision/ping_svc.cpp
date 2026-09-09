#include "ping_svc.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <stdlib.h>  // calloc/free

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

// 会话结束：组 status 文本回报（手机端聊天区直接显示 reason），释放资源、通知任务自杀。
static void on_end(esp_ping_handle_t hdl, void* args) {
  Job* j = (Job*)args;
  esp_ping_delete_session(hdl);
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
  JsonDocument out;
  out["type"] = "status";
  out["params"]["reason"] = text;
  String s;
  serializeJson(out, s);
  if (j->fn) j->fn(j->ctx, s.c_str());
  if (j->ctx) delete (int*)j->ctx;
  xSemaphoreGive(j->done);
}

static void send_status(cmd::ReplyFn fn, void* ctx, const char* reason) {
  JsonDocument out;
  out["type"] = "status";
  out["params"]["reason"] = reason;
  String s;
  serializeJson(out, s);
  if (fn) fn(ctx, s.c_str());
}

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
    vTaskDelete(NULL);
    return;
  }
  esp_ping_start(hdl);
  xSemaphoreTake(j->done, portMAX_DELAY);  // 等 on_ping_end 发结果并给信号
  free(j);
  vTaskDelete(NULL);
}

bool start(const char* target, int count, cmd::ReplyFn reply, void* reply_ctx) {
  if (!target || !target[0] || count <= 0) return false;
  Job* j = (Job*)calloc(1, sizeof(Job));
  if (!j) return false;
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
    return false;
  }
  if (xTaskCreatePinnedToCore(session_task, "ping_svc", 4096, j, 2, nullptr, 1) != pdPASS) {
    if (j->ctx) delete (int*)j->ctx;
    vSemaphoreDelete(j->done);
    free(j);
    return false;
  }
  return true;
}

}  // namespace ping
