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
  s_http->collectAllHeaders(true);  // 保存响应头, 供判断 Transfer-Encoding(默认不收集)
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
static bool rx_block(WiFiClientSecure& c, String& body, long len, RxStat& st) {
  while (len > 0) {
    size_t want = ((size_t)len > sizeof(s_rxbuf)) ? sizeof(s_rxbuf) : (size_t)len;
    size_t got = 0;
    if (!rx_read(c, s_rxbuf, want, got, st)) return false;
    body.concat((const char*)s_rxbuf, got);   // 批量追加, 不再逐字节
    len -= (long)got;
  }
  return true;
}

// 读正文。clen>0: Content-Length 定长; clen<0: chunked 分块。
static bool rx_body(WiFiClientSecure& c, String& body, long clen, RxStat& st) {
  if (clen > 0) {
    body.reserve((unsigned)clen + 16);       // 一次到位, 免掉一路上反复 realloc/memcpy
    return rx_block(c, body, clen, st);
  }
  for (;;) {
    String hdr;                              // chunk size 行(可能带 ";扩展")
    int ch;
    for (;;) {
      if (!rx_read1(c, ch, st)) return false;
      if (ch == '\n') break;
      if (hdr.length() < 32) hdr += (char)ch;
    }
    hdr.trim();
    char* end = nullptr;
    long sz = strtol(hdr.c_str(), &end, 16);
    if (end == hdr.c_str()) return false;    // 帧头不是十六进制数: 流已错位, 交上层(重发)处理
    if (sz <= 0) {
      // 末块后可能有 trailer, 必须吞到空行为止; 残留的 \r\n 会被下一轮当成状态行读, 导致流水线错位。
      for (;;) {
        String line;
        for (;;) {
          if (!rx_read1(c, ch, st)) return true;   // 读不到就算了: 正文已收全, 不值得判失败
          if (ch == '\n') break;
          line += (char)ch;
        }
        if (line.length() == 0 || line == "\r") return true;
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

bool http_post(const char* url, const char* key, const char* body, String& resp,
               unsigned long gen) {
  (void)gen;  // 代际/中断判断由调用方(worker 循环)负责; cancel 走 http_stop()
  // 从入口持锁到本轮结束(POST 与 rx_read 都在用 g_client); http_stop() 只以 0 超时试取, 不会死锁。
  if (!s_tls_mtx) s_tls_mtx = xSemaphoreCreateMutex();   // 只有 worker 调用本函数 ⇒ 无建锁竞态
  ScopedLock tls_lk(s_tls_mtx);
  g_last_status = 0;  // 入口重置, 避免沿用上一轮 4xx/429 误判
  s_last_err[0] = 0;  // 同上: 错误体也要清, 免得本轮网络失败时报出上一轮的错误原因
  s_abort = false;    // 本轮开始: 清掉上一次打断的标记(跨轮的取消由 worker 自己判 gen)
  const size_t body_len = strlen(body);
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
  // 连接策略: 尽量复用同一条 TLS 连接(省握手); 复用可能因残留/半关连接导致写失败(-3),
  // 故 pass0 复用、失败即 pass1 全新握手重发同一 body; 成功后保留连接供下轮。
  for (int pass = 0; pass < 2; pass++) {
    resp = "";                                              // 清残留: 失败重试时防上次部分字节叠加
    unsigned long tp = millis();                            // 本轮(pass)计时, 与整次 t0 区分
    s_req_t0 = tp;
    g_client.diag_reset();
    RxStat st;
    st.t0 = tp;
    bool reused = g_client.connected();                     // pass0: 上次留下的连接是否还在(复用)
    if (pass == 1 && reused) { g_client.stop(); reused = false; }   // 复用失败: 断开, 全新握手
    http_headers();                          // end() 每轮会清掉请求头, 这里重新加上
    // 直接把 body 指针交给 HTTPClient: POST(String) 会先把整包拷成临时 String, 白费一次分配+拷贝。
    int code = s_http->POST((uint8_t*)body, body_len);
    g_last_status = code;
    if (code <= 0) {
      // 字段: 写@≈握手耗时(复用≈0); 发=0/只约 200B 说明 body 没发出去; 空=整块写不进次数; 收/首收=已读字节/首字节。
      ai::logf("[ai] HTTP POST 失败 code=%d pass%d 复用%d %ums 写@%ums 发完@%ums 发%u空%u 收%u 首收%ums body=%uB rssi=%d wifi=%d heap=%uk psram=%uk%s",
               code, pass + 1, reused ? 1 : 0, (unsigned)(millis() - tp), (unsigned)g_client.first_wr_ms,
               (unsigned)g_client.last_wr_ms, (unsigned)g_client.written, (unsigned)g_client.zero_wr,
               (unsigned)g_client.rx_bytes, (unsigned)g_client.first_rx_ms, (unsigned)body_len,
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
      if (eb.length() > 128) eb = eb.substring(0, 128);
      snprintf(s_last_err, sizeof(s_last_err), "%s", eb.c_str());   // 留档供上层带出去
      ai::logf("[ai] HTTP 非200 %d 响应体:%.100s", code, eb.c_str());
      // 错误体是否读尽无保证, 且 4xx/429 后多半要换 key: 一律断开, 别把可能错位的流留给下轮复用。
      g_client.stop();
      s_http->end();
      return false;
    }
    // 读正文: chunked 与定长都走 rx_body; 定长也不能用 getString(内部 readBytes 遇 read()==-1 会提前截断)。
    long clen = s_http->getSize();           // Content-Length; -1 = 无(即 chunked)
    bool chunked = s_http->header("Transfer-Encoding").indexOf("chunked") >= 0;
    st.rx_t0 = millis();                     // 读正文的预算起点(等首字节的那道闸从这算)
    if (chunked || clen > 0) {
      if (!rx_body(g_client, resp, chunked ? -1 : clen, st)) {
        ai::logf("[ai] 响应体读取失败(%s) ~%ums 已收%uB 正文%uB 首收@%ums 正文@%ums 停顿%u次/最长%ums",
                 st.why[0] ? st.why : "未知",
                 (unsigned)(millis() - tp), (unsigned)g_client.rx_bytes, (unsigned)resp.length(),
                 (unsigned)g_client.first_rx_ms, (unsigned)st.first_ms, (unsigned)st.stalls,
                 (unsigned)st.max_gap_ms);
        g_client.stop();                      // 截断/超时: 流已错位, 弃用复用连接
        s_http->end();
        if (pass == 0 && !s_abort) continue;  // pass1 全新握手重发同 body
        return false;
      }
    } else {
      resp = s_http->getString();             // 既无 CL 也无 chunked: 极罕见, 保留原路径
    }
    st.done_ms = millis() - tp;
    s_http->end();                            // 成功即保留连接(_reuse && _canReuse), 下轮复用
    // (每轮"POST 成功"的延迟分段日志默认关闭(常年刷屏): 需排查每轮延迟时再打开。)
    return resp.length() > 0;
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
static bool parse_resp(const String& resp, JsonDocument& doc, bool* broken) {
  // 若响应是 SSE 文本(chunked-SSE 解码产物): 按行取 data: 负载拼成最终 JSON
  String payload = resp;
  if (resp.startsWith("data:") || resp.indexOf("\ndata:") >= 0) {
    payload = "";
    int i = 0, n = resp.length();
    while (i <= n) {
      int j = resp.indexOf('\n', i);
      if (j < 0) j = n;
      String ln = resp.substring(i, j);
      i = j + 1;
      ln.trim();
      if (ln.startsWith("data:")) {
        String d = ln.substring(5); d.trim();
        if (d == "[DONE]") break;
        payload += d;
      }
    }
  }
  // 剥离复用连接偶发泄漏的 chunked size 行("hex+换行+{"): JSON 永不始于 hex, 故不会误伤正文。
  {
    auto ishex = [](char c){ return (c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F'); };
    for (int s = 0; s < 4; s++) {              // 最多剥 4 段(防多块错位叠加)
      const char* q = payload.c_str();
      int i = 0;
      while (i < 8 && ishex(q[i])) i++;        // 扫描十六进制前缀
      if (i == 0) break;                       // 开头非 hex, 非泄漏
      int j = i;
      if (q[j] == '\r') j++;
      if (q[j] == '\n') j++;
      if (q[j] != '{') break;                  // 后随非 '{', 按正文处理
      payload = payload.substring(j);          // 剥掉这段 size 行
    }
  }
  if (deserializeJson(doc, payload)) {
    if (broken) *broken = true;  // 解析失败即视为连接可疑: 截断/残留污染, 调用方弃用复用连接
    // 解析失败诊断: 打印完整 payload + 结构判据(判断是状体被拼断/chunked 泄漏/SSE 多段拼接)。
    auto sanitize = [](String s) {
      for (int i = 0; i < s.length(); i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch < 0x20 || ch == '\x7f') s[i] = '?';
      }
      return s;
    };
    // 结构判据: 是否含 "\ndata:"(SSE 多帧)、是否含多个 "{"id"..."}(多 JSON 拼接)、是否含 chunk 大小泄漏前缀。
    bool sse = payload.indexOf("\ndata:") >= 0 || payload.startsWith("data:");
    int multi_json = 0;
    for (int i = payload.indexOf("{\"id\""); i >= 0 && i < payload.length(); i = payload.indexOf("{\"id\"", i + 1)) multi_json++;
    String full = sanitize(payload);
    ai::logf("[ai] JSON失败 len=%d sse=%d 多json=%d 截断=%d 全文:%s",
             (int)payload.length(), sse ? 1 : 0, multi_json,
             (payload.length() > 0 && payload[payload.length() - 1] != '}' && payload[payload.length() - 1] != ']') ? 1 : 0,
             full.c_str());
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
int extract_tool_calls(const String& resp, JsonDocument& out, bool* broken) {
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
