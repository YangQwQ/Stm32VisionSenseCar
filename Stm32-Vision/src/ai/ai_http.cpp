#include "src/ai/ai_http.h"
#include "src/ai/ai_client.h"      // ai::logf(AI 调试日志)
#include "src/ai/ai_alloc.h"       // g_js_alloc(响应解析的 PSRAM JSON 池)
#include "src/core/board_log.h"    // blog::logf
#include "src/core/lock_guard.h"   // ScopedLock(s_tls_mtx 的取放)

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>      // 负责 TLS/状态行/响应头; 正文自读(见 rx_body 注释)
#include <esp_heap_caps.h>   // MALLOC_CAP_SPIRAM / heap_caps_calloc
#include <freertos/semphr.h>  // s_tls_mtx: 串行化 g_client 的跨任务使用(见其注释)
#include <mbedtls/platform.h>  // mbedtls_platform_set_calloc_free(TLS 内存搬到 PSRAM)相关包含保留
#include <stdlib.h>          // strtol / malloc/free
#include <string.h>

#define AI_CONNECT_TIMEOUT_MS 10000   // TCP 建连 + TLS 握手上限
#define AI_HTTP_TIMEOUT_MS 30000      // HTTPClient 自身超时(建连/发请求/读状态行与响应头)
// 正文读取三道闸(按"有无进展"判定, 而非整段一票否决):
// FIRST=等正文首字节(非流式下=服务端思考耗时, 主项); IDLE=已开始后无新字节即卡死; MAX=正文阶段整体上限。
#define AI_FIRST_BYTE_MS 60000
#define AI_BODY_IDLE_MS 8000
#define AI_BODY_MAX_MS 60000

// ---------------- 传输诊断(只加计数器/日志, 不改发送行为) ----------------
// 用计数器区分"云端慢"还是"链路卡": 写@=首次写入(≈握手耗时)、发完@、发=已写字节、空=写不进次数、首收=首个响应字节。
static unsigned long s_req_t0 = 0;   // 本轮请求发起时刻(DiagClient 换算相对时间用)

class DiagClient : public WiFiClientSecure {
public:
  using WiFiClientSecure::read;
  uint32_t written = 0;      // 已写入的明文字节(请求头 + body)
  uint16_t zero_wr = 0;      // write() 返回 0 的次数(卡住/连接已死)
  uint32_t rx_bytes = 0;     // 已读到的响应字节
  uint32_t first_wr_ms = 0;  // 首次写入距本轮发起 ms(0=还没写)
  uint32_t last_wr_ms = 0;   // 最后一次写入完成距本轮发起 ms(0=一个字节都没写出去)
  uint32_t first_rx_ms = 0;  // 首个响应字节距本轮发起 ms(0=一个字节都没收到)
  void diag_reset() { written = zero_wr = rx_bytes = first_wr_ms = last_wr_ms = first_rx_ms = 0; }
  size_t write(const uint8_t* buf, size_t size) override {
    if (!first_wr_ms) first_wr_ms = millis() - s_req_t0;
    size_t n = WiFiClientSecure::write(buf, size);
    written += n;
    if (size && !n) zero_wr++;              // 整块没写进去: 多半是 socket 超时/连接断开
    else if (n) last_wr_ms = millis() - s_req_t0;
    return n;
  }
  int read(uint8_t* buf, size_t size) override {
    int n = WiFiClientSecure::read(buf, size);
    if (n > 0) {
      rx_bytes += n;
      if (!first_rx_ms) first_rx_ms = millis() - s_req_t0;
    }
    return n;
  }
};

// TLS 客户端(worker 唯一实例): cancel/set_goal 可从其他任务 http_stop() 中止在途请求。
static DiagClient g_client;

// g_client 的独占锁。HTTPClient / NetworkClientSecure / mbedTLS 均非线程安全, 故"谁碰 g_client 谁持锁"。
// http_stop() 直接 stop() 会把在途握手上下文从脚下抽掉而整板 panic, 必须靠它串行化。
static SemaphoreHandle_t s_tls_mtx = nullptr;

// WiFi 断线事件: 记录断线原因码(如 200=BEACON_TIMEOUT / 15=四次握手超时), 便于判根因在链路还是云端。
// ⚠️ 回调在 arduino_events 任务里且持有 NetworkEvents 锁, 只能调纯函数, 不碰会回调进该锁的接口。
static void on_wifi_event(WiFiEvent_t ev, WiFiEventInfo_t info) {
  if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    uint8_t r = info.wifi_sta_disconnected.reason;
    ai::logf("[ai] WiFi 断线 reason=%u(%s)", (unsigned)r,
             WiFi.disconnectReasonName((wifi_err_reason_t)r));
  } else if (ev == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    ai::logf("[ai] WiFi 重新取到 IP");
  }
}

// 最近一次响应的 HTTP 状态码(HTTPClient 写入, 供上层 4xx 快速失败判定; 0=未知)。
static int g_last_status = 0;
// 最近一次非 200 响应的错误体(前若干字节)。留档是为了让上层的 4xx 快速失败能报出**云端给的具体原因**
// (如 "Failed to parse the request body as JSON: trailing characters at ..."), 而不是笼统的"疑似参数或限流"。
static char s_last_err[160];
// http_stop() 置位: 本轮被主动中止, 不该再走 pass1 重发(否则白传一遍整包)。
static volatile bool s_abort = false;

// 常驻 HTTPClient(让 keep-alive 生效): 响应读尽后直接复用, 服务端已关则自动回落新握手。
// ⚠️ end() 会清空请求头(_headers), 每轮 POST 前必须重加 Authorization; 该实例为常驻单例, 不 delete。
static HTTPClient* s_http = nullptr;
static String s_http_url, s_http_key;

static bool http_ready(const char* url, const char* key) {
  if (s_http && s_http_url == url && s_http_key == key) return true;   // 同 URL/key: 复用实例
  if (!s_http) s_http = new HTTPClient();
  s_http_url = url;
  s_http_key = key;
  // ⚠️ begin() 会重置超时, 故配置须跟在它后面; HTTPClient 上的 setTimeout 才真正生效(会盖掉 g_client 的)。
  if (!s_http->begin(g_client, url)) {
    blog::logf(blog::AI, "HTTPClient begin 失败");
    return false;
  }
  s_http->setReuse(true);        // 成功即保留连接供下轮复用
  // 只收集真正要用的那一个响应头。⚠️ 绝不能图省事开 collectAllHeaders(true):
  // HTTPClient 只在 sendRequest(type, uint8_t*, size) 开头清 _currentHeaders, 而我们发 body 走的是
  // sendRequest(type, Stream*, size) 重载, 它不清; 开了全收集后 handleHeaderResponse() 会对每个响应头
  // push_back 一条, 而 s_http 是常驻单例且 end()/clear() 都不清 _currentHeaders ⇒ 每轮永久漏掉一整份
  // 响应头(数百字节内部堆)。实测一轮 AI 58 轮请求 ≈ 漏 25KB, 内部堆被榨到 3k/最大块 0k, WS 随即反复断链。
  static const char* kRespHdrKeys[] = {"Transfer-Encoding"};
  s_http->collectHeaders(kRespHdrKeys, 1);
  s_http->setConnectTimeout(AI_CONNECT_TIMEOUT_MS);
  s_http->setTimeout(AI_HTTP_TIMEOUT_MS);
  return true;
}

static void http_headers() {
  s_http->addHeader("Content-Type", "application/json");
  s_http->addHeader("Authorization", String("Bearer ") + s_http_key);
}

// 正文读取: HTTPClient 自带 chunked 解码会在 TLS 明文空窗期(read()==-1)误判断连, 故自读正文。
// 非阻塞 socket 读空即返 -1; TLS 记录整条落地, 故按 512B 批量取, 逐字节会引发大量 String 重分配。
static uint8_t s_rxbuf[512];   // 静态: worker 任务栈只有 16KB, 别在栈上再吃 512B

struct RxStat {
  unsigned long t0 = 0;          // 本轮(pass)发起时刻 —— 只作 first_ms/done_ms 的报时基准
  unsigned long rx_t0 = 0;       // 开始读正文的时刻 —— 超时预算从这儿算(见下)
  unsigned long last_ok_ms = 0;  // 最近一次成功读到数据的时刻
  unsigned long gap_t0 = 0;      // 当前这段"无数据"的起点(0=不在停顿中)
  unsigned long first_ms = 0;    // 首个正文字节距本轮发起
  unsigned long first_abs = 0;   // 首个正文字节的绝对时刻(正文阶段预算起点, 与报时基准无关)
  unsigned long done_ms = 0;     // 正文读完距本轮发起
  uint32_t max_gap_ms = 0;       // 最长停顿
  uint16_t stalls = 0;           // >200ms 的停顿次数
  bool any = false;              // 是否已收到过正文字节
  const char* why = "";          // 放弃原因(失败日志用): 断流/首字节超时/停顿超时/正文超时/中止
};

// 取一块数据(≤want 字节)。返回 false = 放弃(链路断/停顿超 IDLE/整体超 MAX)。
static bool rx_read(WiFiClientSecure& c, uint8_t* buf, size_t want, size_t& got, RxStat& st) {
  for (;;) {
    if (s_abort) { st.why = "中止"; return false; }   // 被 http_stop() 打断: 立刻收手, 不等超时
    int n = c.read(buf, want);
    if (n > 0) {
      unsigned long now = millis();
      if (st.gap_t0) {                       // 上一段停顿到此结束, 记账
        unsigned long g = now - st.gap_t0;
        if (g > st.max_gap_ms) st.max_gap_ms = g;
        if (g > 200) st.stalls++;
        st.gap_t0 = 0;
      }
      if (!st.first_ms) { st.first_ms = now - st.t0; st.first_abs = now; }
      st.last_ok_ms = now;
      st.any = true;
      got = (size_t)n;
      return true;
    }
    if (!c.connected()) { st.why = "断流"; return false; }  // 链路断了(服务端关连接/被 http_stop 中止)
    unsigned long now = millis();
    // 预算从开始读正文(rx_t0)算, 不从本轮发起算: 否则上传慢会挤掉服务端思考的额度。
    if (st.any) {
      if (now - st.last_ok_ms >= AI_BODY_IDLE_MS) { st.why = "停顿超时"; return false; }  // 已开始但卡住
      if (now - st.first_abs >= AI_BODY_MAX_MS) { st.why = "正文超时"; return false; }    // 正文阶段上限
    } else if (now - st.rx_t0 >= AI_FIRST_BYTE_MS) {
      st.why = "首字节超时";                    // 等首字节(服务端思考)超预算
      return false;
    }
    if (!st.gap_t0) st.gap_t0 = now;
    delay(2);                                // 让出 CPU: worker 只占 1 个 tick 粒度
  }
}

static bool rx_read1(WiFiClientSecure& c, int& out, RxStat& st) {
  size_t got = 0;
  if (!rx_read(c, s_rxbuf, 1, got, st)) return false;
  out = s_rxbuf[0];
  return got == 1;
}

// 按 chunked 取一段定长数据。返回 false = 放弃。
static bool rx_block(WiFiClientSecure& c, PsaBuf& body, long len, RxStat& st) {
  while (len > 0) {
    size_t want = ((size_t)len > sizeof(s_rxbuf)) ? sizeof(s_rxbuf) : (size_t)len;
    size_t got = 0;
    if (!rx_read(c, s_rxbuf, want, got, st)) return false;
    body.append(s_rxbuf, got);   // 批量追加进 PSRAM 缓冲, 不再逐字节
    if (!body.ok) { st.why = "PSRAM 不足"; return false; }
    len -= (long)got;
  }
  return true;
}

// 读正文。clen>0: Content-Length 定长; clen<0: chunked 分块。
static bool rx_body(WiFiClientSecure& c, PsaBuf& body, long clen, RxStat& st) {
  if (clen > 0) {
    body.ensure((size_t)clen + 16);          // 一次到位, 免掉一路上反复 realloc/memcpy
    return rx_block(c, body, clen, st);
  }
  // chunked: 长度事先未知, 靠 PsaBuf 的倍增 ensure 自己长(PSRAM 上翻倍几乎免费)。
  for (;;) {
    char hdr[36];                            // chunk size 行(可能带 ";扩展"); 栈缓冲, 不逐字符建 String
    size_t hn = 0;
    int ch;
    for (;;) {
      if (!rx_read1(c, ch, st)) return false;
      if (ch == '\n') break;
      if (hn < sizeof(hdr) - 1) hdr[hn++] = (char)ch;
    }
    hdr[hn] = 0;
    char* end = nullptr;
    long sz = strtol(hdr, &end, 16);         // strtol 自动跳过前导空白; 尾部 ";扩展" 由 end 停住
    if (end == hdr) return false;            // 帧头不是十六进制数: 流已错位, 交上层(重发)处理
    if (sz <= 0) {
      // 末块后可能有 trailer, 必须吞到空行为止; 残留的 \r\n 会被下一轮当成状态行读, 导致流水线错位。
      for (;;) {
        char line[4]; size_t ln = 0;         // 只关心是否空行(或单个 '\r')
        for (;;) {
          if (!rx_read1(c, ch, st)) return true;   // 读不到就算了: 正文已收全, 不值得判失败
          if (ch == '\n') break;
          if (ln < sizeof(line)) line[ln] = (char)ch;
          ln++;
        }
        if (ln == 0 || (ln == 1 && line[0] == '\r')) return true;
      }
    }
    if (!rx_block(c, body, sz, st)) return false;
    if (!rx_read1(c, ch, st)) return false;  // 块尾 \r\n
    if (!rx_read1(c, ch, st)) return false;
  }
}

// mbedTLS 内存优先走 PSRAM 的预留实现: 内部堆只留少量连续块即可握手。
// 现无 mbedtls_platform_set_calloc_free 调用点(故标 unused), 属预留, 别删。
__attribute__((unused)) static void* ai_tls_calloc(size_t n, size_t s) {
  void* p = heap_caps_calloc(n, s, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_calloc(n, s, MALLOC_CAP_INTERNAL);
  return p;
}
__attribute__((unused)) static void ai_tls_free(void* p) { heap_caps_free(p); }

// 早拒探针: body 发不出去(code=-3)时服务端其实早已回拒, 只是状态码读不到(请求体把 TCP 窗口写满)。
// 用同一 URL/key、2 字节 body 再问一次即可拿到真实状态码(不外发任何画面/记忆数据)。
static bool split_url(const char* url, String& host, int& port, String& path) {
  String u(url);
  int p = u.indexOf("://");
  if (p < 0) return false;
  bool tls = u.startsWith("https");
  u = u.substring(p + 3);
  int slash = u.indexOf('/');
  String hp = (slash < 0) ? u : u.substring(0, slash);
  path = (slash < 0) ? String("/") : u.substring(slash);
  int colon = hp.indexOf(':');
  port = tls ? 443 : 80;
  if (colon >= 0) { port = hp.substring(colon + 1).toInt(); hp = hp.substring(0, colon); }
  host = hp;
  return host.length() > 0 && port > 0;
}

// 返回探针读到的 HTTP 状态码(0=连状态行都没读到)
static int probe_early_reject(const char* url, const char* key) {
  String host, path;
  int port = 0;
  if (!split_url(url, host, port, path)) return 0;
  g_client.stop();                       // 上一轮这条连接已废, 重新握手
  if (!g_client.connect(host.c_str(), port)) {
    blog::logf(blog::AI, "[ai] 探针: 建连失败 %s:%d", host.c_str(), port);
    return 0;
  }
  String req = String("POST ") + path + " HTTP/1.1\r\nHost: " + host +
               "\r\nContent-Type: application/json\r\nAuthorization: Bearer " + key +
               "\r\nContent-Length: 2\r\nConnection: close\r\n\r\n{}";
  g_client.write((const uint8_t*)req.c_str(), req.length());
  static char pb[256];                   // 静态: worker 栈只有 16KB
  int n = 0;
  unsigned long t0 = millis(), last = t0;
  while (n < (int)sizeof(pb) - 1 && millis() - t0 < 6000) {
    int r = g_client.read((uint8_t*)pb + n, sizeof(pb) - 1 - n);
    if (r > 0) { n += r; last = millis(); }
    else if (millis() - last > 1500) break;   // 非阻塞 socket: 读空就短延时再试
    else delay(10);
  }
  pb[n] = 0;
  int code = 0;
  if (n > 8 && !strncmp(pb, "HTTP/", 5)) code = atoi(pb + 8);   // "HTTP/1.1 418 ..."
  for (int i = 0; i < n; i++) {          // 折成一行: 换行会把日志拆散
    unsigned char c = (unsigned char)pb[i];
    if (c < 32 || c == 127) pb[i] = ' ';
  }
  blog::logf(blog::AI, "探针(2B body) → 状态=%d 收%uB | %.190s", code, (unsigned)n, pb);
  g_client.stop();
  return code;
}

namespace ai {

bool http_post(const char* url, const char* key, PieceList& body, PsaBuf& resp,
               unsigned long gen) {
  (void)gen;  // 代际/中断判断由调用方(worker 循环)负责; cancel 走 http_stop()
  // 从入口持锁到本轮结束(POST 与 rx_read 都在用 g_client); http_stop() 只以 0 超时试取, 不会死锁。
  if (!s_tls_mtx) s_tls_mtx = xSemaphoreCreateMutex();   // 只有 worker 调用本函数 ⇒ 无建锁竞态
  ScopedLock tls_lk(s_tls_mtx);
  g_last_status = 0;  // 入口重置, 避免沿用上一轮 4xx/429 误判
  s_last_err[0] = 0;  // 同上: 错误体也要清, 免得本轮网络失败时报出上一轮的错误原因
  s_abort = false;    // 本轮开始: 清掉上一次打断的标记(跨轮的取消由 worker 自己判 gen)
  static bool s_tls_cfg = false;
  if (!s_tls_cfg) {
    g_client.setInsecure();                 // 开发期信任自签; 上线建议改 CA 校验
    g_client.setConnectionTimeout(AI_CONNECT_TIMEOUT_MS);
    g_client.setTimeout(AI_HTTP_TIMEOUT_MS);
    WiFi.onEvent(on_wifi_event);            // 断线/取 IP 记日志(排查链路问题)
    s_tls_cfg = true;
    ai::logf("[ai] TLS 就绪 connect=%dms rw=%dms heap=%uk psram=%uk", AI_CONNECT_TIMEOUT_MS,
             AI_HTTP_TIMEOUT_MS, (unsigned)(ESP.getFreeHeap() / 1024),
             (unsigned)(ESP.getFreePsram() / 1024));
  }
  if (!http_ready(url, key)) return false;
  // 请求体以碎片流交给 HTTPClient: sendRequest 会把 total(各碎片输出字节之和)自己填进 Content-Length,
  // 再按 1460B 从流里取着发 —— 不需要先拼出一整块 body, 图片 base64 也边发边编码。
  PieceStream ps(body);
  // 连接策略: 尽量复用同一条 TLS 连接(省握手); 复用可能因残留/半关连接导致写失败(-3),
  // 故 pass0 复用、失败即 pass1 全新握手重发同一 body; 成功后保留连接供下轮。
  for (int pass = 0; pass < 2; pass++) {
    resp.clear();                                           // 清残留: 失败重试时防上次部分字节叠加(保留容量)
    unsigned long tp = millis();                            // 本轮(pass)计时, 与整次 t0 区分
    s_req_t0 = tp;
    g_client.diag_reset();
    RxStat st;
    st.t0 = tp;
    bool reused = g_client.connected();                     // pass0: 上次留下的连接是否还在(复用)
    if (pass == 1 && reused) { g_client.stop(); reused = false; }   // 复用失败: 断开, 全新握手
    http_headers();                          // end() 每轮会清掉请求头, 这里重新加上
    // 重播同一份碎片(输出由固定的指针+偏移决定, 字节与 pass0 完全一致, total 也就一致)。
    ps.rewind();
    int code = s_http->sendRequest("POST", &ps, body.total);
    g_last_status = code;
    if (code <= 0) {
      // 字段: 写@≈握手耗时(复用≈0); 发=0/只约 200B 说明 body 没发出去; 空=整块写不进次数; 收/首收=已读字节/首字节。
      ai::logf("[ai] HTTP POST 失败 code=%d pass%d 复用%d %ums 写@%ums 发完@%ums 发%u空%u 收%u 首收%ums body=%uB rssi=%d wifi=%d heap=%uk psram=%uk%s",
               code, pass + 1, reused ? 1 : 0, (unsigned)(millis() - tp), (unsigned)g_client.first_wr_ms,
               (unsigned)g_client.last_wr_ms, (unsigned)g_client.written, (unsigned)g_client.zero_wr,
               (unsigned)g_client.rx_bytes, (unsigned)g_client.first_rx_ms, (unsigned)body.total,
               (int)WiFi.RSSI(), (int)WiFi.status(), (unsigned)(ESP.getFreeHeap() / 1024),
               (unsigned)(ESP.getFreePsram() / 1024), s_abort ? " 中止" : "");
      g_client.stop();                       // 发失败/连接已死: 一律弃用, 下一轮重新握手
      s_http->end();
      if (pass == 0 && !s_abort) {
        // 先用探针问一次: 若非 2xx 即服务端在拒我们, 直接带真实状态码失败, 省掉一次无谓重发。
        int pc = probe_early_reject(url, key);
        if (pc >= 300) { g_last_status = pc; return false; }
        continue;                             // 2xx/没读到: 端点本身是好的, 按老路重发一次
      }
      return false;
    }
    if (code != 200) {
      String eb = s_http->getString();
      // 不再 substring(0,128) 出第二份副本: s_last_err 只有 160B, 交给 snprintf 的精度直接截断即可。
      snprintf(s_last_err, sizeof(s_last_err), "%.159s", eb.c_str());   // 留档供上层带出去
      ai::logf("[ai] HTTP 非200 %d 响应体:%.100s", code, eb.c_str());
      // 错误体是否读尽无保证, 且 4xx/429 后多半要换 key: 一律断开, 别把可能错位的流留给下轮复用。
      g_client.stop();
      s_http->end();
      return false;
    }
    // 读正文: chunked 与定长都走 rx_body; 定长也不能用 getString(内部 readBytes 遇 read()==-1 会提前截断)。
    long clen = s_http->getSize();           // Content-Length; -1 = 无(即 chunked)
    // Stream 版 sendRequest 连 header 的 value.clear() 都不做(只有 uint8_t* 版会清), 故某轮响应没带
    // Transfer-Encoding 时会残留上一轮的值; 有 Content-Length 就按定长读, 只有没有 CL 时才信这个标志。
    bool chunked = (clen <= 0) && (s_http->header("Transfer-Encoding").indexOf("chunked") >= 0);
    st.rx_t0 = millis();                     // 读正文的预算起点(等首字节的那道闸从这算)
    if (chunked || clen > 0) {
      if (!rx_body(g_client, resp, chunked ? -1 : clen, st)) {
        ai::logf("[ai] 响应体读取失败(%s) ~%ums 已收%uB 正文%uB 首收@%ums 正文@%ums 停顿%u次/最长%ums",
                 st.why[0] ? st.why : "未知",
                 (unsigned)(millis() - tp), (unsigned)g_client.rx_bytes, (unsigned)resp.len,
                 (unsigned)g_client.first_rx_ms, (unsigned)st.first_ms, (unsigned)st.stalls,
                 (unsigned)st.max_gap_ms);
        g_client.stop();                      // 截断/超时: 流已错位, 弃用复用连接
        s_http->end();
        if (pass == 0 && !s_abort) continue;  // pass1 全新握手重发同 body
        return false;
      }
    } else {
      // 既无 CL 也无 chunked: 极罕见。这条路没有长度可依, 退回 HTTPClient 自带的读法再灌进 PSRAM。
      String raw = s_http->getString();
      resp.append((const uint8_t*)raw.c_str(), raw.length());
    }
    st.done_ms = millis() - tp;
    s_http->end();                            // 成功即保留连接(_reuse && _canReuse), 下轮复用
    // (每轮"POST 成功"的延迟分段日志默认关闭(常年刷屏): 需排查每轮延迟时再打开。)
    return resp.len > 0;
  }
  return false;
}

int http_last_status() { return g_last_status; }

// 最近一次非 200 响应的错误体(空串=该次没读到)。仅 http_post() 返回 false 后立即读取才有意义。
const char* http_last_error() { return s_last_err; }

void http_stop() {
  // 只置标志, 不在这里无条件 stop() —— 它可能正被 worker 用在 mbedtls 里, 那样等于从脚下抽上下文。
  // s_abort 由 rx_read 每轮检查, 中止延迟毫秒级; POST 内部握手/发送那段不可中断, 只能等它返回。
  s_abort = true;      // 标成主动中止: 让在途的这一轮失败后不再自动重发
  // 能立刻拿到锁 = worker 此刻不在 TLS 里 ⇒ 断开是安全的，顺手断掉还能让下一轮重新握手。
  // 拿不到（worker 正在 POST/读正文）就什么都不做：s_abort 会把它带出来。
  ScopedLock lk(s_tls_mtx, 0);   // 0 超时: 拿不到就算了, 绝不在这里等 worker
  if (lk.held()) g_client.stop();
}

// ---------------- 响应解析(content / tool_calls 两入口共用) ----------------
// 响应体 → JSON 文档: 先处理 SSE(data: 拼接)与 chunked 泄漏前缀, 再反序列化。
// 返回 false = 反序列化失败(置 broken; 多半是传输层截断/复用残留 ⇒ 调用方应弃用复用连接)。
static bool parse_resp(const PsaBuf& resp, JsonDocument& doc, bool* broken) {
  // resp 已在 PSRAM 里(见 http_post)。非 SSE 时 payload 只是它的一个视图, 不再整份复制成另一个
  // String: 响应体可达几十 KB, 而这条路径每轮都走 —— 白搭一份内部堆既是占用也是碎片源。
  // 只有真要改写(SSE 拆帧 / 剥 size 行)时才需要实体副本或指针前进。
  String sse_buf;                              // 仅 SSE 时持有拼接结果
  const char* payload = resp.c_str();
  if (!strncmp(payload, "data:", 5) || strstr(payload, "\ndata:")) {
    sse_buf = "";                              // SSE(chunked-SSE 解码产物): 按行取 data: 负载拼成最终 JSON
    const char* q = payload;
    while (*q) {
      const char* eol = strchr(q, '\n');
      size_t n = eol ? (size_t)(eol - q) : strlen(q);
      const char* a = q;
      while (n && (unsigned char)*a <= ' ') { a++; n--; }              // 去首尾空白(等价原 ln.trim())
      while (n && (unsigned char)a[n - 1] <= ' ') n--;
      if (n >= 5 && !strncmp(a, "data:", 5)) {
        const char* d = a + 5;
        size_t dn = n - 5;
        while (dn && (unsigned char)*d <= ' ') { d++; dn--; }
        while (dn && (unsigned char)d[dn - 1] <= ' ') dn--;
        if (dn == 6 && !strncmp(d, "[DONE]", 6)) break;
        sse_buf.concat(d, (unsigned)dn);       // concat(const char*, 长度): 不必先切出子串
      }
      if (!eol) break;
      q = eol + 1;
    }
    payload = sse_buf.c_str();
  }
  // 剥离复用连接偶发泄漏的 chunked size 行("hex+换行+{"): JSON 永不始于 hex, 故不会误伤正文。
  // 直接把指针往前推, 不再 substring 出一份新 String。
  {
    auto ishex = [](char c){ return (c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F'); };
    for (int s = 0; s < 4; s++) {              // 最多剥 4 段(防多块错位叠加)
      const char* q = payload;
      int i = 0;
      while (i < 8 && ishex(q[i])) i++;        // 扫描十六进制前缀
      if (i == 0) break;                       // 开头非 hex, 非泄漏
      int j = i;
      if (q[j] == '\r') j++;
      if (q[j] == '\n') j++;
      if (q[j] != '{') break;                  // 后随非 '{', 按正文处理
      payload += j;                            // 剥掉这段 size 行
    }
  }
  if (deserializeJson(doc, payload)) {
    if (broken) *broken = true;  // 解析失败即视为连接可疑: 截断/残留污染, 调用方弃用复用连接
    // 解析失败诊断: 打印 payload 开头 + 结构判据(判断是状体被拼断/chunked 泄漏/SSE 多段拼接)。
    // ⚠️ 只取开头一段: logf 自己会把输出截到 256B, 为此 sanitize 整份 payload(再来一份几十 KB 的副本)
    // 纯属浪费 —— 而这恰好发生在内存最紧张的时候。
    size_t plen = strlen(payload);
    char head[200];
    size_t hl = plen < sizeof(head) - 1 ? plen : sizeof(head) - 1;
    memcpy(head, payload, hl);
    head[hl] = 0;
    for (size_t i = 0; i < hl; i++) {          // 控制符→'?'(串口/WS 都要能看)
      unsigned char ch = (unsigned char)head[i];
      if (ch < 0x20 || ch == '\x7f') head[i] = '?';
    }
    // 结构判据: 是否含 "\ndata:"(SSE 多帧)、是否含多个 "{"id"..."}(多 JSON 拼接)、是否含 chunk 大小泄漏前缀。
    bool sse = strstr(payload, "\ndata:") != nullptr || !strncmp(payload, "data:", 5);
    int multi_json = 0;
    for (const char* q = strstr(payload, "{\"id\""); q; q = strstr(q + 1, "{\"id\"")) multi_json++;
    ai::logf("[ai] JSON失败 len=%d sse=%d 多json=%d 截断=%d 全文:%s",
             (int)plen, sse ? 1 : 0, multi_json,
             (plen > 0 && payload[plen - 1] != '}' && payload[plen - 1] != ']') ? 1 : 0,
             head);
    return false;
  }
  return true;
}

// 思考内容: 完整打串口, /log ai on 时也转发手机(仅 WS)。此处不落库; 回投模型由调用方负责
// (extract_tool_calls 把它拷进 out["reasoning"], 随回合存历史, 组包时原样回传)。
static void log_reasoning(const JsonDocument& doc) {
  const char* rc = doc["choices"][0]["message"]["reasoning_content"] | "";
  if (!rc[0]) return;
  Serial.print("[ai] 思考: "); Serial.println(rc);
  blog::forward_text(blog::AI, rc);
}

// 从响应提取工具调用, 规范化进 out["calls"]: [{"id","name","args"}(args 为模型给的参数 JSON 原文)...]。
// 返回调用数(0 = 没调工具)。⚠️ 返回的字符串是 out 这份 PSRAM 池文档里的视图, 调用方用完即弃, 别留指针。
// 同时打印 reasoning_content; 无调用时打诊断(区分 finish=length 预算被思考吃光 / 模型只回了文本)。
int extract_tool_calls(const PsaBuf& resp, JsonDocument& out, bool* broken) {
  JsonDocument doc(&g_js_alloc);
  if (!parse_resp(resp, doc, broken)) return 0;
  log_reasoning(doc);
  // 思考原文交给调用方(随回合存进历史): 带 tools 的请求必须原样回传, 否则模型每轮都得从头重推。
  // ⚠️ 用 String 赋值才会拷进 out 这份池; 直接塞 const char* 只存指针, doc 一析构就悬空。
  const char* rcx = doc["choices"][0]["message"]["reasoning_content"] | "";
  if (rcx[0]) out["reasoning"] = String(rcx);
  JsonArrayConst tcs = doc["choices"][0]["message"]["tool_calls"].as<JsonArrayConst>();
  int cnt = 0;
  JsonArray dst = out["calls"].to<JsonArray>();
  for (JsonObjectConst tc : tcs) {
    const char* nm = tc["function"]["name"] | "";
    if (!nm[0]) continue;
    JsonObject d = dst.add<JsonObject>();
    d["id"] = tc["id"] | "";
    d["name"] = nm;
    d["args"] = tc["function"]["arguments"] | "";
    cnt++;
  }
  if (cnt == 0) {
    const char* c = doc["choices"][0]["message"]["content"] | "";
    const char* rc = doc["choices"][0]["message"]["reasoning_content"] | "";
    const char* em = doc["error"]["message"] | "";
    ai::logf("[ai] 无工具调用: finish=%s content=%uB reasoning=%uB 补全/总=%u/%u%s%s",
             doc["choices"][0]["finish_reason"] | "(无)", (unsigned)strlen(c), (unsigned)strlen(rc),
             (unsigned)(doc["usage"]["completion_tokens"] | 0), (unsigned)(doc["usage"]["total_tokens"] | 0),
             em[0] ? " err=" : "", em);
  }
  return cnt;
}

}  // namespace ai
