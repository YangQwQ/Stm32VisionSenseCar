#include "src/ai/ai_http.h"
#include "src/ai/ai_client.h"      // ai::logf(AI 调试日志)
#include "src/core/board_log.h"    // blog::logf

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>      // 统一处理 TLS/content-length/chunked, 替代手写 http_exchange
#include <esp_heap_caps.h>   // MALLOC_CAP_SPIRAM / heap_caps_calloc
#include <mbedtls/platform.h>  // mbedtls_platform_set_calloc_free(TLS 内存搬到 PSRAM)相关包含保留
#include <stdlib.h>          // strtol / malloc/free
#include <string.h>

#define AI_CONNECT_TIMEOUT_MS 10000
#define AI_HTTP_TIMEOUT_MS 30000

// TLS 客户端(worker 唯一实例): cancel/set_goal 可从其他任务 http_stop() 中止在途请求。
static WiFiClientSecure g_client;

// 最近一次响应的 HTTP 状态码(HTTPClient 写入, 供上层 4xx 快速失败判定; 0=未知)。
static int g_last_status = 0;

// ---------------- HTTPS POST(keep-alive 复用连接) ----------------
// 直接走 TLS socket 裸写 HTTP/1.1 的旧方案已废弃(见原 http_exchange, 现改用 HTTPClient)。
// 连接策略: 同一 host 复用 TLS 连接(Connection: keep-alive), 省掉每轮 ~500ms 握手;
// 复用连接被服务端空闲断开/半开时, http_post 检测后重建一次立即重发。
// 响应按块严格消费(chunked 终止块后吞 trailer 到空行), 保证流干净可复用。

// 手动读取 chunked 响应体。HTTPClient 的 chunked 解码在 TLS 明文空窗期(响应跨多个 TCP 段、
// 段间隙数据未到)会因 readBytes 遇 read()==-1 立即判死 → READ_TIMEOUT → 断连, 实测正文只
// 剩一个 TCP 段(1448B)被截断。这里改用"读不到就短延时重试"直到超时/连接关闭, 根除截断。
static bool read_chunked_body(WiFiClientSecure& c, String& body, unsigned long timeout_ms) {
  unsigned long t0 = millis();
  auto rd = [&]() -> int {                    // 读 1 字节; 无明文则等待(-1 = 超时/断连)
    while (millis() - t0 < timeout_ms) {
      if (c.available()) {
        int b = c.read();
        if (b >= 0) return b;
      }
      if (!c.connected()) return -1;
      delay(2);
    }
    return -1;
  };
  for (;;) {
    String hdr;                               // chunk size 行(可能带 ";扩展")
    int ch;
    while ((ch = rd()) >= 0 && ch != '\n') hdr += (char)ch;
    if (ch < 0) return false;
    hdr.trim();
    long sz = strtol(hdr.c_str(), nullptr, 16);
    if (sz <= 0) return true;                 // 末块; 忽略可能存在的 trailer
    for (long i = 0; i < sz; i++) {
      ch = rd();
      if (ch < 0) return false;
      body += (char)ch;
    }
    if (rd() < 0 || rd() < 0) return false;   // 块尾 \r\n
  }
}

// mbedTLS 内存搬到 PSRAM 的预留实现: 把 mbedTLS 内部所有分配改为优先 PSRAM(8MB 充足),
// 内部堆只留少量连续块即可握手。当前全仓库无 mbedtls_platform_set_calloc_free 调用点,
// 二者不会被 mbedTLS 使用, 属预留。
static void* ai_tls_calloc(size_t n, size_t s) {
  void* p = heap_caps_calloc(n, s, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_calloc(n, s, MALLOC_CAP_INTERNAL);
  return p;
}
static void ai_tls_free(void* p) { heap_caps_free(p); }

namespace ai {

bool http_post(const char* url, const char* key, const char* body, String& resp,
               unsigned long gen) {
  (void)gen;  // 代际/中断判断由调用方(worker 循环)负责; cancel 走 http_stop()
  g_last_status = 0;  // 入口重置, 避免沿用上一轮 4xx/429 误判
//   ai::logf("[ai] TLS前 freeHeap=%u maxBlock=%u freePsram=%u",
//            ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getFreePsram());
  unsigned long t0 = millis();
  // 连接策略: 尽量复用同一条 TLS 连接(省握手、降延迟)。复用的风险是上一次成功响应在
  // ssl_ctx/HTTPClient 留下未读尽的残留, 或连接在决策间隙被服务端/半开关掉, 导致下一次
  // mbedtls_ssl_write 报 -0x7100 BAD_INPUT_DATA(→-3)。处理: pass0 直接复用当前 g_client;
  // 一旦发送失败立刻 stop 掉, pass1 用全新握手重发同一 body(只多这一轮), 把多轮 -3 阵发
  // 收敛成"一次复用失败 + 一次新握手成功"。success 时保留连接供下轮复用。
  static bool s_tls_cfg = false;
  if (!s_tls_cfg) {
    g_client.setInsecure();                 // 开发期信任自签; 上线建议改 CA 校验
    g_client.setConnectionTimeout(AI_CONNECT_TIMEOUT_MS);
    g_client.setTimeout(AI_HTTP_TIMEOUT_MS);
    s_tls_cfg = true;
  }
  for (int pass = 0; pass < 2; pass++) {
    resp = "";                                              // 清残留: 失败重试时防上次部分字节叠加
    if (pass == 1 && g_client.connected()) { g_client.stop(); }  // 复用失败: 彻底断开, 全新握手
    HTTPClient http;
    http.setReuse(true);                    // 成功即保留连接供下轮复用
    http.collectAllHeaders(true);           // 保存响应头, 供判断 Transfer-Encoding(默认不收集)
    if (!http.begin(g_client, url)) { blog::logf(blog::AI, "HTTPClient begin 失败"); return false; }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + String(key));
    int code = http.POST(body);             // body 为 PSRAM C 串; 流式发送、不整体拷内部堆
    g_last_status = code;
    // ai::logf("[ai] POST 完成 code=%d ~%u ms rssi=%d", code, (unsigned)(millis() - t0), (int)WiFi.RSSI());
    if (code <= 0) {
      ai::logf("[ai] HTTP POST 失败 code=%d pass%d rssi=%d", code, pass + 1, (int)WiFi.RSSI());
      if (pass == 0) { g_client.stop(); http.end(); continue; }   // 复用失败: 断开, pass1 全新握手
      g_client.stop(); http.end();
      return false;
    }
    if (code != 200) {
      String eb = http.getString();
      if (eb.length() > 128) eb = eb.substring(0, 128);
      ai::logf("[ai] HTTP 非200 %d 响应体:%.100s", code, eb.c_str());
      http.end();
      return false;
    }
    // 读响应体: chunked 走手动解码(规避上述 TLS 明文空窗截断); Content-Length 路径
    // writeToStreamDataBlock 外层循环会重试短读, getString 安全。
    if (http.header("Transfer-Encoding").indexOf("chunked") >= 0) {
      if (!read_chunked_body(g_client, resp, AI_HTTP_TIMEOUT_MS)) {
        g_client.stop();                      // 截断/超时: 弃用复用连接
        http.end();
        if (pass == 0) continue;              // pass1 全新握手重发同 body
        return false;
      }
    } else {
      resp = http.getString();                // CL 路径: 模型简短, 落堆可接受
    }
    http.end();                               // 成功即保留连接, 下轮复用
    return resp.length() > 0;
  }
  return false;
}

int http_last_status() { return g_last_status; }

void http_stop() { g_client.stop(); }

}  // namespace ai