#include "src/ai/ai_http.h"
#include "src/ai/ai_client.h"      // ai::logf(AI 调试日志)
#include "src/core/board_log.h"    // blog::logf

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
// 正文读取的三道闸。旧实现只有"整段 30s 一票否决", 于是慢但一直在进数据的响应会被判死并
// 白重发一次整包(日志: ~32915ms 已收489B 之后紧跟着一次重发)。现在改为按"有无进展"判定:
//   FIRST: 等正文首字节。非流式响应下这一段 = 服务端把整段回答生成完的时间(思考), 是整轮的
//          主项 —— 实测正常请求就能到 29.4s, 所以最初照搬 HTTPClient 的 30s 会误杀(日志:
//          "响应体读取失败 ~32063ms 正文0B 停顿0次/最长0ms" = 一个字节都没来就被判死),
//          误杀代价是白传一次整包再赔一整轮。给到 60s。
//   IDLE : 正文已开始后, 连续这么多 ms 没新字节 = 卡死 → 放弃(实测分片间隙约 0.3~0.4s, 8s 有 20 倍余量)
//   MAX  : 正文阶段(自首字节起算)的整体上限, 只要还有数据在进来就允许读完
#define AI_FIRST_BYTE_MS 60000
#define AI_BODY_IDLE_MS 8000
#define AI_BODY_MAX_MS 60000

// ---------------- 传输诊断(只加计数器/日志, 不改发送行为) ----------------
// 两类失败都表现为"HTTPClient 的某个超时到了", 必须靠计数器区分是**云端慢**还是**链路卡**:
//   -3 SEND_PAYLOAD_FAILED: 发 body 时写不动 → 看"发X空Y": 发出去的字节数(请求头约 200B)+0 字节写次数
//   -11 READ_TIMEOUT:      请求发出后收不到响应 → 看"首收@"(首字节迟到多少 ms)
// 关键字段: 写@=首次写入距本轮发起多少 ms(≈TCP+TLS 握手耗时, 复用连接时≈0);
//           发完@=最后一次写入完成的时刻(≈body 传完); 发=已写入明文字节; 空=write() 一次都没写进去的次数;
//           首收=首个响应字节时刻。成功路径把它们一起打出来, 就能把一轮拆成
//           连接/上传/等首包/收正文 四段(见 http_post 末尾日志)。
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

// g_client 的独占锁。HTTPClient / NetworkClientSecure / mbedTLS **没有一处是线程安全的**，
// 所以"谁碰 g_client 谁持锁"。
//
// 为什么必须有它（一次真实的 panic，不是理论担忧）：旧 http_stop() 直接 g_client.stop()
// ⇒ stop_ssl_socket() ⇒ mbedtls_ssl_free(&ssl_ctx)。若此时 worker 恰在 s_http->POST() 里做
// 全新握手（NetworkClientSecure::connect → start_ssl_client → ssl_starttls_handshake →
// mbedtls_ssl_handshake），上下文就被从脚下抽掉，在途握手解引用 ssl->conf（结构里偏移 8）
// 得到地址 0x8 ⇒ LoadProhibited ⇒ `复位=崩溃`。
// 现场证据（由 /coredump 取回 + addr2line 解开，见 tools/carctl.py coredump）：
//   任务 ai_worker  cause=28(LoadProhibited)  vaddr=0x8  回溯落在
//   mbedtls ssl_tls.c:4594/4626 + Arduino NetworkClientSecure/ssl_client.cpp:340(握手那一行)
//   ← HTTPClient::connect ← src/ai/ai_http.cpp:294 的 s_http->POST。
// 触发条件普通得可怕：**上一轮请求还在途时又来一个 ai_oneshot / 手动指令**即可 ——
// 连跑多轮 AI 任务时每轮都在掷这个骰子。
static SemaphoreHandle_t s_tls_mtx = nullptr;

// RAII 取锁：http_post 里 return 路径多，手动 give 必漏一处。
struct TlsLock {
  bool held = false;
  TlsLock() { if (s_tls_mtx) held = (xSemaphoreTake(s_tls_mtx, portMAX_DELAY) == pdTRUE); }
  ~TlsLock() { if (held) xSemaphoreGive(s_tls_mtx); }
};

// WiFi 断线事件: 记录断线原因码(比 RSSI 有用得多: 200=BEACON_TIMEOUT 信号丢 / 15=四次握手超时 等)。
// 请求超时若恰好伴随断线, 根因就在链路而非云端; 之前全仓库无任何断线日志, 断线只能靠猜。
// 注: 回调在 arduino_events 任务(栈 4096)里、且持有 NetworkEvents 锁, 故只调纯函数(原因码→名字),
// 不碰会回调进 NetworkEvents 的接口(所以这里不取 IP, IP 由 net::update 连上时打)。
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
// http_stop()(手动指令打断/换目标)置位: 本轮是被主动中止的, 不该再走 pass1 重发。
// 光靠 g_client.stop() 只是让这一轮失败, 旧代码接着 `if (pass == 0) continue;` 会把 26~87KB
// 的 body 原样再传一遍 —— 打断反而多花一次上传时间。
static volatile bool s_abort = false;

// ---------------- 常驻 HTTPClient: 让 keep-alive 真正生效 ----------------
// 旧写法是在 pass 循环里 `HTTPClient http;` —— 它在函数返回前析构, 而
// HTTPClient::~HTTPClient() 无条件 _client->stop()(HTTPClient.cpp:89), 于是 setReuse(true)
// 和 end() 里刚保住的 TLS 会话每轮都被拆掉: 每轮都白付一次 TCP+TLS 握手。日志里的 "复用0"
// 就是它。begin() 也不能重调 —— _client 非空时它会先 _canReuse=false; end()(HTTPClient.cpp:118)。
// 所以: 实例常驻, 只在 url/key 变化时 begin() 一次, 之后反复 POST。
// 复用安全性: 响应读尽后 available()==0, connect() 里的 connected() 判定走 _connected 分支
// (socket 在 ssl_client.cpp:100 被置为 O_NONBLOCK, 而 lwip 对非阻塞 socket 立即返回
//  ERR_WOULDBLOCK、不看 recv_timeout, 所以 available() 不会卡住) → 直接复用; 若服务端已关连接,
// data_to_read 会收到 PEER_CLOSE_NOTIFY/CONN_EOF 并 stop(), connected() 转 false → 自动回落到
// 新握手, 不需要额外探测。
// ⚠️ end() 内部调 clear(), 而 clear() 会把请求头串 _headers 清空(HTTPClient.cpp:104), 所以每轮
// POST 前都要重加头 —— 否则第二轮起请求里就没有 Authorization 了(addHeader 默认 replace, 幂等)。
// 注: 这个 new 是故意的常驻单例(不做 delete), 一次性分配, 不随请求增长。
static HTTPClient* s_http = nullptr;
static String s_http_url, s_http_key;

static bool http_ready(const char* url, const char* key) {
  if (s_http && s_http_url == url && s_http_key == key) return true;   // 同 URL/key: 复用实例
  if (!s_http) s_http = new HTTPClient();
  s_http_url = url;
  s_http_key = key;
  // begin() 会把超时设回 HTTPClient 默认值, 故三项配置必须跟在它后面:
  // ⚠️ connect() 内部还会用 _tcpTimeout 覆盖客户端的 socket 超时, 所以 HTTPClient 上这份
  // setTimeout 才是真正生效的那份(g_client.setTimeout 会被它盖掉)。
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

// ---------------- 正文读取 ----------------
// HTTPClient 自带的 chunked 解码在 TLS 明文空窗期(响应跨多个 TCP 段、段间隙数据未到)会因
// readBytes 遇 read()==-1 立即判死 → READ_TIMEOUT → 断连, 实测正文只剩一个 TCP 段被截断。
// 这里自读正文: 读不到就短延时重试, 只在"持续无进展"或整体超时才放弃。
// 非阻塞 socket 保证 c.read() 无数据时立即返回 -1(不会卡在 socket 上), 而一旦某个 TLS 记录
// 落地, 整条记录会一次性可读 —— 所以按 512B 批量取, 不再逐字节(旧实现每字节要过 3 次
// mbedtls 入口, 且每个字节都往 String 上追加一次; String 的 changeBuffer 每次只长
// (len+16)&~0xf, 22KB 正文 ≈ 1400 次 realloc + 反复整段 memcpy)。
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
    // 预算从 st.rx_t0(开始读正文)算, 不从本轮发起算: 否则上传慢时会把服务端思考的额度挤掉
    // (旧实现就是各读各的 t0, 这是它唯一没出错的地方, 别改坏)。
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
      // 末块(0\r\n)。后面还可能有 trailer("名: 值\r\n" 若干), 必须一直吞到空行为止 ——
      // 旧实现这里直接 return true, 把终止块的 \r\n 留在流里: 今天因为连接每轮都被拆掉所以
      // 没暴露, 一旦连接真复用, 残留会被下一轮当成状态行读, 整条流水线就错位了。
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

// mbedTLS 内存搬到 PSRAM 的预留实现: 把 mbedTLS 内部所有分配改为优先 PSRAM(8MB 充足),
// 内部堆只留少量连续块即可握手。当前全仓库无 mbedtls_platform_set_calloc_free 调用点,
// 二者不会被 mbedTLS 使用, 属预留。
static void* ai_tls_calloc(size_t n, size_t s) {
  void* p = heap_caps_calloc(n, s, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_calloc(n, s, MALLOC_CAP_INTERNAL);
  return p;
}
static void ai_tls_free(void* p) { heap_caps_free(p); }

// ---------------- 早拒探针 ----------------
// 为什么需要它: body 一个字节都发不出去(典型 code=-3)时, 服务端其实**早就回了一个拒绝**
// (401 / 429 / 418 / 400 ...), 只是它不读请求体: 板子几十 KB 的 body 灌过去把 TCP 窗口写满,
// mbedtls_ssl_write 就卡到 socket 超时为止 —— 任务从没走到"读响应"那一步, 那个状态码于是永远
// 看不见, 现场只剩一句"发不出去", 谁也判不出到底是 key、限流、还是被 CDN 挡了。
// 探针用同一个 URL/key、**2 字节**的 body 再问一次: 请求小到不会被排空问题卡住, 服务端说什么
// 就读得到什么。同一端点、同一凭据, 不外发任何画面/记忆数据。
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
  // 从入口就持锁，直到本轮(含正文自读)结束：POST 与后续 rx_read 都在用 g_client，
  // 中途松开就等于把窗口留给 http_stop()。锁只被 http_stop() 以 0 超时试取(见那里)，不会反锁死。
  if (!s_tls_mtx) s_tls_mtx = xSemaphoreCreateMutex();   // 只有 worker 调用本函数 ⇒ 无建锁竞态
  TlsLock tls_lk;
  g_last_status = 0;  // 入口重置, 避免沿用上一轮 4xx/429 误判
  s_abort = false;    // 本轮开始: 清掉上一次打断的标记(跨轮的取消由 worker 自己判 gen)
  unsigned long t0 = millis();
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
  // 连接策略: 尽量复用同一条 TLS 连接(省掉每轮 ~TCP+TLS 握手)。复用的风险是上一次响应在
  // ssl_ctx 里留下未读尽的残留, 或连接在决策间隙被服务端/半开关掉, 导致下一次
  // mbedtls_ssl_write 报错(→-3)。处理: pass0 直接复用当前 g_client; 一旦这一轮失败立刻 stop
  // 掉, pass1 用全新握手重发同一 body(只多这一轮), 把多轮 -3 阵发收敛成"一次复用失败 +
  // 一次新握手成功"。success 时保留连接供下轮复用。
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
    // 直接把 PSRAM 里的 body 指针交给 HTTPClient: POST(String) 会把整包(26~87KB)先拷成一份
    // 临时 String(内部堆只有几十 KB, 只能落 PSRAM)再发, 白费一次分配+拷贝。
    int code = s_http->POST((uint8_t*)body, body_len);
    g_last_status = code;
    if (code <= 0) {
      // 字段: 写@=首次写入距今(≈TCP+TLS 握手耗时, 复用≈0; 发=0 时表示整轮一个字节都没写出去)
      //       发=已写字节(只发出请求头约 200B = body 没发出去) 空=整块写不进去的次数
      //       收=已读字节 首收=首个响应字节距今(0=没收到任何响应)
      ai::logf("[ai] HTTP POST 失败 code=%d pass%d 复用%d %ums 写@%ums 发完@%ums 发%u空%u 收%u 首收%ums body=%uB rssi=%d wifi=%d heap=%uk psram=%uk%s",
               code, pass + 1, reused ? 1 : 0, (unsigned)(millis() - tp), (unsigned)g_client.first_wr_ms,
               (unsigned)g_client.last_wr_ms, (unsigned)g_client.written, (unsigned)g_client.zero_wr,
               (unsigned)g_client.rx_bytes, (unsigned)g_client.first_rx_ms, (unsigned)body_len,
               (int)WiFi.RSSI(), (int)WiFi.status(), (unsigned)(ESP.getFreeHeap() / 1024),
               (unsigned)(ESP.getFreePsram() / 1024), s_abort ? " 中止" : "");
      g_client.stop();                       // 发失败/连接已死: 一律弃用, 下一轮重新握手
      s_http->end();
      if (pass == 0 && !s_abort) {
        // 先用探针问一次"服务端到底怎么说"(见 probe_early_reject 的注释)。若它明确回了个非 2xx,
        // 那就是它在拒我们 —— 换条新连接重发同一份 body 只会再赔一次同样的上传时间, 且结果一样。
        // 直接带着真实状态码失败, 把 20 秒的死等压成一次握手(4xx 快速失败也能据此生效)。
        int pc = probe_early_reject(url, key);
        if (pc >= 300) { g_last_status = pc; return false; }
        continue;                             // 2xx/没读到: 端点本身是好的, 按老路重发一次
      }
      return false;
    }
    if (code != 200) {
      String eb = s_http->getString();
      if (eb.length() > 128) eb = eb.substring(0, 128);
      ai::logf("[ai] HTTP 非200 %d 响应体:%.100s", code, eb.c_str());
      // 错误体由 getString 读, 是否读尽无保证; 且 4xx/429 后上层多半要换 key 重来 ——
      // 一律断开, 别把可能错位的流留给下一轮复用(以前靠 ~HTTPClient 顺手停, 现在析构不管了)。
      g_client.stop();
      s_http->end();
      return false;
    }
    // 读正文: chunked 自定义分块 / Content-Length 定长, 都走同一套"按进展判定"的读取。
    // (两者都可能跨多个 TCP 段, 所以定长路径也不能用 getString —— 它内部 readBytes 遇
    //  read()==-1 会提前收工, 同样是截断风险。)
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
    // 成功也记一行, 且把一轮拆成四段(这是"延迟到底花在哪"的唯一直接证据):
    //   写@    首次写入 ≈ TCP 建连+TLS 握手结束(复用时应≈0)
    //   发完@  最后一次写入完成 ≈ body 上传结束       → 上传耗时 = 发完@ - 写@
    //   首收@  首个响应字节(状态行)                    → 服务端首包延迟 = 首收@ - 发完@
    //   正文@  首个正文字节                            → 收完@ - 正文@ = 下载正文耗时
    //   停顿N次/最长Mms: 正文读取中">200ms 没新字节"的次数与最长间隔 —— 数它就能判断
    //                    "慢"是服务端在挤牙膏(停顿多且长)还是链路/客户端(停顿少但密集)
    //   sse=1: 正文以 "data:" 开头, 说明服务端按 SSE 分帧回(那 ai_client 的 SSE 分支就会走)
    // ⚠️ 下面这条"POST 成功"日志常年在刷屏(每轮都有), 对日常分析价值不大; 要排查每轮延迟,
    //    打开下面注释即可(信号就在那几个时间戳里)。平时关着, 让 /log ai on 的日志更干净。
    // ai::logf("[ai] POST 成功 %d 总%ums 复用%d 写@%ums 发完@%ums 首收@%ums 正文@%ums 收完@%ums 收%uB 正文%uB 停顿%u次/最长%ums sse=%d body=%uB",
    //          code, (unsigned)(millis() - t0), reused ? 1 : 0, (unsigned)g_client.first_wr_ms,
    //          (unsigned)g_client.last_wr_ms, (unsigned)g_client.first_rx_ms, (unsigned)st.first_ms,
    //          (unsigned)st.done_ms, (unsigned)g_client.rx_bytes, (unsigned)resp.length(),
    //          (unsigned)st.stalls, (unsigned)st.max_gap_ms, resp.startsWith("data:") ? 1 : 0,
    //          (unsigned)body_len);
    return resp.length() > 0;
  }
  return false;
}

int http_last_status() { return g_last_status; }

void http_stop() {
  // 只置标志。**不要**在这里无条件 g_client.stop() —— 它可能正被 worker 用在 mbedtls 里面，
  // 那样就是从脚下抽上下文（0x8 那次 panic 的成因，见 s_tls_mtx 上方注释）。
  //
  // 标志本身足够快：读正文的 rx_read 每轮循环都查 s_abort，而 socket 是非阻塞的（每次无数据
  // 立即返回 -1 再 delay(2)），所以中止延迟是毫秒级。唯一查不到的空档是 POST 内部的
  // 握手/发送，那一段本来就不可中断，只能等它自己返回（上限 AI_CONNECT_TIMEOUT_MS /
  // AI_HTTP_TIMEOUT_MS）；用"拆上下文"去抢那几秒，代价是随机的整板 panic，不值。
  // 而且 worker 在失败路径上本来就会 g_client.stop()（http_post 里 code<=0 与 rx_body 失败两处），
  // 连接该断的照样会断 —— 这里不断，只是把"什么时候断"交回给持有它的那个任务。
  s_abort = true;      // 标成主动中止: 让在途的这一轮失败后不再自动重发
  // 能立刻拿到锁 = worker 此刻不在 TLS 里 ⇒ 断开是安全的，顺手断掉还能让下一轮重新握手。
  // 拿不到（worker 正在 POST/读正文）就什么都不做：s_abort 会把它带出来。
  if (s_tls_mtx && xSemaphoreTake(s_tls_mtx, 0) == pdTRUE) {
    g_client.stop();
    xSemaphoreGive(s_tls_mtx);
  }
}

}  // namespace ai
