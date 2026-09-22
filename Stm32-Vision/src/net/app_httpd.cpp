// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "esp_http_server.h"
#include "esp_timer.h"
#include <freertos/semphr.h>   // WS 帧发送串行化用的互斥量
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include <string.h>
#include <stdlib.h>
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "lwip/sockets.h"
// lwip 的 inet.h（经上行 sockets.h:53 引入）把 INADDR_NONE/IPADDR_NONE 定义成宏，而 Arduino
// core 的 IPAddress.h 声明同名全局对象（extern const IPAddress INADDR_NONE），宏会在声明处
// 展开破坏语法；lwip 为预编译库，此处撤销宏不影响其编译期使用。
// ⚠️ 这两行必须紧跟 lwip/sockets.h。IPAddress.h 由 Arduino.h:198 引入，因此任何直接或间接
//    拉进 Arduino.h 的头都只能排在其后，否则报 "expected ')' before numeric constant"
//    （踩过的实例：BLEDevice.h → BLEServer.h:45 → Arduino.h → IPAddress.h）。
#undef INADDR_NONE
#undef IPADDR_NONE
#include "esp_heap_caps.h"
#include "esp_core_dump.h"   // /coredump：panic 现场（原因串 + 任务 + 回溯 PC）
#include "src/cam/camera.h"       // cam::grab / cam::jpeg_len（后者见 camera.cpp 的说明）
#include "src/cam/camera_index.h"
#include "src/net/ota.h"   // HTTP OTA 入口（/update）注册
#include "src/ai/ai_dump.h"   // AI 抓帧留档清单/取图（/ai_dump, /ai_frame 调试用）
#include "src/ai/magnify.h"   // /zoomshot：手动对当帧目标区域裁出放大图（等价于 AI 的 zoom）
#include "src/ai/ai_client.h"   // ai::busy()：AI 任务进行中时图传 FPS 降半，让 CPU 与内部 DMA 池给 AI 让路

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// 板载人脸检测/识别已关闭：其依赖 Espressif esp-face 的模型头文件，
// 且本项目视觉识别交由云端多模态大模型完成，板载人脸检测非必需。
// 若日后需要启用：引入 espressif/esp-face 模型头文件后，将下面两宏置 1 即可。
#define CONFIG_ESP_FACE_DETECT_ENABLED 0
#define CONFIG_ESP_FACE_RECOGNITION_ENABLED 0

#if CONFIG_ESP_FACE_DETECT_ENABLED

#include <vector>
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"

#define TWO_STAGE 1 /*<! 1: detect by two-stage which is more accurate but slower(with keypoints). */
                    /*<! 0: detect by one-stage which is less accurate but faster(without keypoints). */

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
#include "face_recognition_tool.hpp"
#include "face_recognition_112_v1_s16.hpp"
#include "face_recognition_112_v1_s8.hpp"

#define QUANT_TYPE 0 //if set to 1 => very large firmware, very slow, reboots when streaming...

#define FACE_ID_SAVE_NUMBER 7
#endif

#define FACE_COLOR_WHITE 0x00FFFFFF
#define FACE_COLOR_BLACK 0x00000000
#define FACE_COLOR_RED 0x000000FF
#define FACE_COLOR_GREEN 0x0000FF00
#define FACE_COLOR_BLUE 0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)
#endif

// Enable LED FLASH setting
#define CONFIG_LED_ILLUMINATOR_ENABLED 1

// LED FLASH setup
#if CONFIG_LED_ILLUMINATOR_ENABLED

#define CONFIG_LED_MAX_INTENSITY 255

int led_duty = 0;
bool isStreaming = false;
static uint8_t s_led_pin = 0;  // core 3.x LEDC 改引脚式 API，不再手动分 channel（与摄像头 timer 冲突由驱动内部处理）

#endif

typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// 连接建立后关 Nagle（指令/结果小报文低延迟）。
// 图传帧已改走 UDP，WS 不再承载大帧，故发送缓冲不再放大；
// 之前 64KB 的 SO_SNDBUF 会吃掉内部 RAM 连续块，反而挤占 ai_client 的 TLS 握手堆。
static void tune_socket(int fd)
{
    int sndbuf = 8 * 1024;
    lwip_setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    int nodelay = 1;
    lwip_setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
}

#ifdef CONFIG_HTTPD_WS_SUPPORT
// =================== WebSocket（端口 81 根路径，手机 App WSCarClient 通道） ===================
#include "ArduinoJson.h"
#include "src/cam/camera.h"
#include "src/core/command.h"
#include "src/net/ble.h"
#include "src/ai/ai_client.h"
#include "src/exec/direct_exec.h"
#include "src/core/board_log.h"
#include "src/net/wifi_net.h"   // net::kWanMtu（UDP 分片大小由它派生）

#define WS_STREAM_FPS 10
// UDP 图传整帧上限帧率：板子按链路吞吐能推多快就推多快，但接收端解析能力有限，
// 超出部分只会变成接收端积压（延迟累积、并挤占控制面应答），故在此封顶。
#define UDP_STREAM_MAX_FPS 18
#define WS_EDIT_IMG_MAX (128 * 1024)  // 编辑图（二进制上行）上限，与 ai_client 一致

// WS 对端（手机）活体探测：连续此时间无任何上行 → 发一次探测 ping；再此时间无响应判死。
// 用于感知"静默断链"（如手机重启），恢复 BLE 广播供再次配网/兜底。探测基于应用层
// {"type":"ping"}，对端协议栈/客户端回 {"type":"pong"}，均走 WS 文本帧，无附加心跳。
// 手机侧无条件按固定周期发 ping（上行，与图传下行无关），故 idle 阈值只要大于该周期
// 就不会误伤健康客户端。⚠️ 曾误伤：手机侧一度改成"仅下行空闲时才发探测"，图传下行不断
// → 上行 ping 永不发、idle 必然到期；且单轮无应答即踢，把"手机这一拍忙"当成真断链。
static const uint32_t WS_IDLE_PING_MS = 6000;     // 连续无上行时长，达到则发探测
static const uint32_t WS_PING_TIMEOUT_MS = 2500;  // 单轮探测可容忍的无上行时长
static const uint32_t WS_PING_MAX_TRY = 3;        // 最多探测轮数，连续这么多轮无应答才判死
static volatile uint32_t s_ws_last_rx_ms = 0;      // 最近收到任一 WS 文本/binary 上行（ms）
static volatile bool s_ws_ping_pending = false;    // 已发探测、等 pong（由 httpd 任务写入）
static uint32_t s_ws_ping_at_ms = 0;
static uint32_t s_ws_ping_try = 0;                 // 本轮已发出的探测次数（判死计数）
static bool s_ws_had_client = false;               // WS 客户端曾经在位（有→无即判离线）

// httpd 活跃 fd 表上限，须 >= 服务器 max_open_sockets（给足余量，过长时跳过）
#define WS_MAX_CLIENTS 32

// =================== UDP 图传（手机 App UDPVideoClient 接收端） ===================
// 图传视频帧改用 UDP 承载，替代 WS 的 TCP 推流：规避 TCP 慢启动/Nagle/重传引发的
// 延迟时高时低。指令/状态仍走 WS（控制面，低带宽可靠通道）。
// 握手：手机绑定本地 UDP 端口并上报本机 IP，随 WS 的 stream on 指令发(udp_port, src_ip)；
// 板据此建立 UDP 会话（同网段，无 NAT）。整帧 JPEG 切成 ≤UDP_JPG_CHUNK 的分片推送。
#define UDP_FRAME_HDR 14       // 分片头固定字节：magic(2) frame_id(4) seq(2) count(2) total(4) 全大端
// 每数据报 JPEG 分片负载：网卡 MTU 扣 IP(20)+UDP(8)+分片头，保证**不会被 IP 分片**。
// 必须由 net::kWanMtu 派生，不能写死：板子为修 AI 通路把网卡 MTU 压到了公网路径 MTU 之下
// （见 wifi_net.h），此处若仍是 1500 时代的数值，每个数据报都会被切成两片 —— 图传的
// 报文数、驱动 TX 队列压力与"丢一片=废一帧"的概率都会变差。接收端按包头 seq/count 重组，
// 与分片大小无关，故改小完全兼容。
#define UDP_JPG_CHUNK (net::kWanMtu - 28 - UDP_FRAME_HDR)
#define UDP_MAGIC0 0x56
#define UDP_MAGIC1 0x44
static int s_udp_fd = -1;                    // UDP 会话 fd（懒创建）
static volatile bool s_udp_peer_valid = false; // 已注册手机 UDP 对端
static struct sockaddr_in s_udp_peer;        // 目标：手机 IP:udp_port（WS 处理任务写、推流任务读）
static uint32_t s_udp_frame_id = 0;          // 帧序号，接收端按它区分新旧帧

// 建立/更新 UDP 对端：ip_s_addr 为网络字节序、port 为主机序（内部 htons）。懒创建非阻塞 socket（缓冲满即跳帧）。
static void udp_peer_set(uint32_t ip_s_addr, uint16_t port) {
    if (s_udp_fd < 0) {
        s_udp_fd = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s_udp_fd < 0) { log_e("UDP socket fail"); return; }
        int nb = 1;
        // 非阻塞：拿不到发送机会就立刻返回，由 udp_pump 停手、下个 tick 接着续，不在这里等。
        // 注：lwip 的 UDP 出站不实现 SO_SNDBUF（无 socket 级发送缓冲，setsockopt 无效），
        // 背压完全来自 WiFi 驱动 TX 队列 —— sendto 返回 <0 即"队列满"，属正常节流信号。
        lwip_fcntl(s_udp_fd, F_SETFL, O_NONBLOCK);
    }
    memset(&s_udp_peer, 0, sizeof(s_udp_peer));
    s_udp_peer.sin_family = AF_INET;
    s_udp_peer.sin_addr.s_addr = ip_s_addr;
    s_udp_peer.sin_port = htons(port);
    s_udp_peer_valid = true;
}

// 渐进式图传状态：整帧缓存在 PSRAM，按驱动队列空闲逐分片续传，队列满即停、下 tick 再续，
// 直到整帧发完才抓下一帧。链路能承受多少就稳发多少(自然降帧)，不再出现"整帧发一半被弃"——
// 半帧到手机端必然重组不出来，那些字节纯属白占链路，还会把接收端卡在旧画面上。
// 帧率天然自适应：吞吐 ÷ 单帧字节，无需调参；代价是慢链路下画面变陈旧，故设上限弃帧重来。
#define OUT_FRAME_MAX_US   600000  // 单帧在途时长上限(us)：超时弃帧，避免画面无限陈旧
#define OUT_FRAME_STALL_US 400000  // 连续此期间一个分片都发不出去 → 判链路死，弃帧重试
static uint8_t* s_out_jpeg = nullptr;  // 当前待发帧副本(PSRAM)
static size_t   s_out_cap = 0;         // s_out_jpeg 已分配容量
static size_t   s_out_len = 0;         // 本帧字节数
static uint16_t s_out_count = 0;       // 本帧分片总数
static uint16_t s_out_seq_next = 0;    // 下一个待发分片序号
static uint32_t s_out_fid = 0;         // 本帧号
static bool     s_out_active = false;  // 有帧在续传中
static uint64_t s_out_start = 0;       // 本帧开始续传时刻(us)
// 最近一次成功发出分片的时刻(us)。弃帧判据用它而非"tick 计数"：慢链路上帧尾会把驱动
// 缓冲池占满，新帧开头几拍可能一片都发不出去，按 tick 数判会把这些帧误杀。
static uint64_t s_out_last_ok = 0;
static uint64_t s_out_last_grab_us = 0;  // 上帧开抓时刻(us)，用于上限帧率节流
// 弃帧时的背压归因：sendto 失败原因(errno)与本帧累计失败次数。ENOBUFS=驱动 TX 池满(对端/空口不消化)、
// EAGAIN=本端 socket 发送缓冲满、其余多为 netif 异常 —— 三者处置完全不同，故随弃帧一并印出，
// 免得只能从"无进展多少 ms"反推。（空口不消化时池子被在途包占满，重试再多次也发不出去。）
static int      s_out_errno = 0;
static uint32_t s_out_fail = 0;

static void udp_peer_clear(void) {
    s_udp_peer_valid = false;
    s_out_active = false;
}

static void udp_out_free(void) {
    if (s_out_jpeg) { heap_caps_free(s_out_jpeg); s_out_jpeg = nullptr; s_out_cap = 0; }
}

// 把一帧 JPEG 拷入 PSRAM 副本作为"当前待发帧"，随后由 udp_pump() 逐分片续传。
static bool udp_start_frame(const uint8_t *jpeg, size_t len) {
    if (!len || len > 256 * 1024) return false;
    if (!s_out_jpeg || len > s_out_cap) {
        uint8_t *nb = (uint8_t*)heap_caps_malloc(len, MALLOC_CAP_SPIRAM);
        if (!nb) return false;
        udp_out_free();
        s_out_jpeg = nb; s_out_cap = len;
    }
    memcpy(s_out_jpeg, jpeg, len);
    s_out_len = len;
    s_out_count = (uint16_t)((len + UDP_JPG_CHUNK - 1) / UDP_JPG_CHUNK);
    s_out_fid = s_udp_frame_id++;
    s_out_seq_next = 0;
    s_out_start = esp_timer_get_time();
    s_out_last_ok = s_out_start;   // 从"刚起步"起算无进展时长
    s_out_errno = 0;
    s_out_fail = 0;
    s_out_active = true;
    return true;
}

// 单 tick 内续传的时间预算：队列满时让出片刻等驱动排空再续。
// ⚠️ 撞满不能立刻收手：WiFi 驱动的 TX 缓冲池只有十几个 buffer(~24KB)，每 tick 只填满一次的话，
// 吞吐会被钉死在「池容量 × 循环频率」≈ 240KB/s，远低于链路实际能力（单帧 >60KB 时只剩几帧）；
// 预算到期才收手，剩余分片交给下一 tick，既解开自设上限又不霸占本任务。
#define OUT_PUMP_BUDGET_US 40000
// 推流时的循环节拍(ms)。这个延时在每轮循环末尾无条件执行，而一帧通常要跨多轮 pump 才发完，
// 于是它既是「每帧固定多付的延迟」，也是「帧间隔被量化的台阶」（帧率忽高忽低的一部分来源）。
// 取 5ms：远小于一帧的空中时间，兼顾"队列一空就赶紧续片"与"不空转烧 CPU"。
// （pump 自身已按 40ms 预算与队列状态自行限流，不靠这个节拍限速。）
#define OUT_PUMP_TICK_MS   5

// 续传当前帧：按底层队列空闲连续发送分片；整帧发完置 s_out_active=false。
// lwip 的 UDP 出站没有发送队列，sendto 失败反映的是 WiFi 驱动 TX 队列满 —— 属背压而非错误，
// 因此绝不据此废弃整帧（旧实现 break 后整帧丢弃，是"帧一直被丢弃"的直接原因）。
// 帧率自适应为「吞吐 ÷ 单帧字节」，无需调参。
static void udp_pump(void) {
    if (!s_out_active || !s_udp_peer_valid || s_udp_fd < 0) return;
    uint8_t hdr[UDP_FRAME_HDR];
    uint8_t buf[UDP_JPG_CHUNK + UDP_FRAME_HDR];
    uint64_t t0 = esp_timer_get_time();
    while (s_out_seq_next < s_out_count) {
        size_t off = (size_t)s_out_seq_next * UDP_JPG_CHUNK;
        size_t chunk = s_out_len - off;
        if (chunk > UDP_JPG_CHUNK) chunk = UDP_JPG_CHUNK;
        uint16_t seq = s_out_seq_next;
        hdr[0] = UDP_MAGIC0; hdr[1] = UDP_MAGIC1;
        hdr[2] = (uint8_t)(s_out_fid >> 24); hdr[3] = (uint8_t)(s_out_fid >> 16);
        hdr[4] = (uint8_t)(s_out_fid >> 8);  hdr[5] = (uint8_t)(s_out_fid);
        hdr[6] = (uint8_t)(seq >> 8);        hdr[7] = (uint8_t)(seq);
        hdr[8] = (uint8_t)(s_out_count >> 8); hdr[9] = (uint8_t)(s_out_count);
        hdr[10] = (uint8_t)(s_out_len >> 24); hdr[11] = (uint8_t)(s_out_len >> 16);
        hdr[12] = (uint8_t)(s_out_len >> 8);  hdr[13] = (uint8_t)(s_out_len);
        memcpy(buf, hdr, UDP_FRAME_HDR);
        memcpy(buf + UDP_FRAME_HDR, s_out_jpeg + off, chunk);
        if (lwip_sendto(s_udp_fd, buf, UDP_FRAME_HDR + chunk, 0,
                        (struct sockaddr *)&s_udp_peer, sizeof(s_udp_peer)) < 0) {
            s_out_errno = errno;  // 归因用（见 s_out_errno 注释）
            s_out_fail++;
            if (esp_timer_get_time() - t0 >= OUT_PUMP_BUDGET_US) break;  // 本 tick 预算用尽，余片下 tick 续
            vTaskDelay(1);  // 让出约 1ms 给驱动排空，同一分片稍后重试
            continue;
        }
        s_out_last_ok = esp_timer_get_time();
        s_out_seq_next++;
    }
    if (s_out_seq_next >= s_out_count) s_out_active = false;  // 整帧发完
}

// 全量 WS fd 探测/踢除辅助：httpd 仅暴露活跃 fd 表，无 per-fd 状态管理，故按需遍历。
static esp_err_t ws_send_text(int fd, const char *text);  // 前向声明（定义在下方）
static void ws_send_text_to_ws_clients(const char *text); // 前向声明（定义在下方）
static void ws_ping_all(void) {
    int fds[WS_MAX_CLIENTS]; size_t n = WS_MAX_CLIENTS;
    if (httpd_get_client_list(stream_httpd, &n, fds) != ESP_OK) return;
    for (size_t i = 0; i < n; i++) {
        if (httpd_ws_get_fd_info(stream_httpd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        ws_send_text(fds[i], "{\"type\":\"ping\"}");  // 探测帧，对端回 pong
    }
}

static void ws_kick_all(void) {
    int fds[WS_MAX_CLIENTS]; size_t n = WS_MAX_CLIENTS;
    if (httpd_get_client_list(stream_httpd, &n, fds) != ESP_OK) return;
    for (size_t i = 0; i < n; i++) {
        if (httpd_ws_get_fd_info(stream_httpd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        httpd_sess_trigger_close(stream_httpd, fds[i]);  // 关闭死 fd，下轮剔出 client list
    }
}

// WS 文本统一出口消毒(残缺/非法 UTF-8 与控制字符→'?')，防 Godot 1007 断链
static void ws_sanitize_utf8(char* s) {
    char* w = s;
    const unsigned char* p = (const unsigned char*)s;
    while (*p) {
        unsigned char c = *p;
        int need = 0;
        if (c < 0x20 || c == 0x7f) { *w++ = '?'; p++; continue; }
        if (c < 0x80) { *w++ = (char)c; p++; continue; }
        if (c >= 0xC2 && c <= 0xDF) need = 1;
        else if (c >= 0xE0 && c <= 0xEF) need = 2;
        else if (c >= 0xF0 && c <= 0xF4) need = 3;
        bool ok = need > 0;
        for (int i = 1; ok && i <= need; i++) {
            unsigned char cc = p[i];
            if (!cc || !(cc >= 0x80 && cc <= 0xBF)) ok = false;
        }
        if (ok) { for (int i = 0; i <= need; i++) *w++ = (char)p[i]; p += need + 1; }
        else    { *w++ = '?'; p += 1; }
    }
    *w = 0;
}

// WS 帧发送串行化（与下面 ws_send_text / ws_send_jpeg 共用）：帧由多个任务并发发出，而
// httpd_ws_send_frame_async 是"低层直接写 socket"（帧头与载荷分次写，无内部排队、
// 无线程安全保证）。并发调用会把两个帧的字节交错，接收端只能按协议错误断链——手机端
// Godot 报的就是 1007 的 RFC 描述文本 "Invalid frame payload data"（看着像 UTF-8 问题，
// 实为帧被写坏），表现为"日志一切正常却莫名掉线"，且必在日志/状态推送最密时发作。
static SemaphoreHandle_t s_ws_tx_mtx = nullptr;
static StackType_t*  s_ws_stack = nullptr;  // ws_stream 任务栈：放 PSRAM，抬内部 DMA 块水位
static StaticTask_t  s_ws_tcb;
#define WS_TX_LOCK_TIMEOUT_MS 500   // 等锁上限：对端卡住时宁可丢这一帧，绝不拖住调用者

static esp_err_t ws_send_text(int fd, const char *text)
{
    // 统一出口消毒，堆拷贝发(1:1)，不动入参、不占发送任务大栈
    size_t n = strlen(text);
    char* buf = (char*)malloc(n + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    memcpy(buf, text, n + 1);
    ws_sanitize_utf8(buf);
    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t *)buf;
    frame.len = strlen(buf);
    if (s_ws_tx_mtx && xSemaphoreTake(s_ws_tx_mtx, pdMS_TO_TICKS(WS_TX_LOCK_TIMEOUT_MS)) != pdTRUE) {
        free(buf);
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t r = httpd_ws_send_frame_async(stream_httpd, fd, &frame);
    if (s_ws_tx_mtx) xSemaphoreGive(s_ws_tx_mtx);
    free(buf);
    return r;
}

static esp_err_t ws_send_jpeg(int fd, camera_fb_t *fb)
{
    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_BINARY;
    frame.payload = fb->buf;
    frame.len = cam::jpeg_len(fb);   // 勿用 fb->len：可能被驱动报大（裹进后续帧）
    if (s_ws_tx_mtx && xSemaphoreTake(s_ws_tx_mtx, pdMS_TO_TICKS(WS_TX_LOCK_TIMEOUT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    esp_err_t r = httpd_ws_send_frame_async(stream_httpd, fd, &frame);
    if (s_ws_tx_mtx) xSemaphoreGive(s_ws_tx_mtx);
    return r;
}

// cmd::Reply 适配：把应答文本发回给指定 fd。
// fd 已不是活跃 WS 客户端时回退广播：AI 闭环这类**长任务**在任务起点就把 fd 深拷贝进任务
// （见 ai_client 的 enqueue_result），手机 WS 断线重连换了 fd 之后，原 fd 不再是 WS 客户端，
// 定向发送会被 httpd 丢弃（ws_send_text 只回错误码，无人重试），手机端表现为"任务还在跑、
// 但 AI 的答复突然不再出现"（AI 调试日志走广播通道，故照常到达，容易误判成日志模块的问题）。
// 本链路假定单手机客户端，失效即广播与定向等价，消息不会再丢。
static void ws_cmd_reply(void *ctx, const char *text)
{
    int fd = *(int *)ctx;
    if (httpd_ws_get_fd_info(stream_httpd, fd) == HTTPD_WS_CLIENT_WEBSOCKET)
        ws_send_text(fd, text);
    else
        ws_send_text_to_ws_clients(text);
}

// 文本帧 = 指令 JSON；stream 就地建/拆 UDP 会话，其余统一移交 command 模块
static void ws_handle_text(const char *json, int fd)
{
    JsonDocument doc;
    if (deserializeJson(doc, json)) {
        log_w("[ws] bad json: %s", json);
        return;
    }
    const char *type = doc["type"] | "";

    // UDP 图传握手：stream on 且携带有效 udp_port → 结合 WS 对端 IP 建立 UDP 会话；
    // 其余（on 无有效端口 / off）→ 拆除旧会话，避免向已失效端口空推。
    if (!strcmp(type, "stream")) {
        bool on = doc["params"]["on"] | false;
        int port = doc["params"]["udp_port"] | 0;
        const char *src_ip = doc["params"]["src_ip"] | "";
        blog::logf(blog::WS, "stream on=%d port=%d src_ip=%s", on, port, src_ip);
        // 升级中不建/不拆 UDP 会话：手机自动重连会把"开图传"原样重放回来（掉线时它的开关
        // 并不会复位），一旦重建对端 ws_stream_task 立刻恢复抓帧推流，与固件写入抢射频和
        // core1（实测把 OTA 拖到超时）。command 侧另有同样的闸门，这里挡的是它之前的建会话。
        if (ota::active()) {
            blog::logf(blog::WS, "固件升级中：忽略图传开关（保持关闭）");
        } else if (on && port > 0) {
            struct sockaddr_in src;
            socklen_t sl = sizeof(src);
            uint32_t peer_ip = 0;
            // 优先采用手机上报的本机 IP（实测 lwip_getpeername 对 httpd fd 回 0.0.0.0，不可靠）
            if (src_ip[0]) {
                uint32_t a = 0, b = 0, c = 0, d = 0;
                if (sscanf(src_ip, "%u.%u.%u.%u", &a, &b, &c, &d) == 4)
                    peer_ip = a | (b << 8) | (c << 16) | ((uint32_t)d << 24);  // 首字节在低址 = 网络字节序
                else
                    peer_ip = 0;
            }
            if (peer_ip) {
                udp_peer_set(peer_ip, (uint16_t)port);
            } else if (lwip_getpeername(fd, (struct sockaddr *)&src, &sl) == 0) {
                udp_peer_set(src.sin_addr.s_addr, (uint16_t)port);
            }
            blog::logf(blog::WS, "peer set -> %u.%u.%u.%u:%d",
                        (uint8_t)(peer_ip), (uint8_t)(peer_ip >> 8),
                        (uint8_t)(peer_ip >> 16), (uint8_t)(peer_ip >> 24), port);
        } else {
            udp_peer_clear();
            blog::logf(blog::WS, "peer cleared (on=%d port=%d)", on, port);
        }
    }
    // 统一词表：move/stop/arm/config/stream/ai_goal/ai_cancel 等交给 command
    // （stream 由 command 更新全局图传开关；ai_goal 异步结果回传）。
    // 摇杆高频帧同指令连续重复时只记首条，避免刷屏；换指令再记。
    // has_frames=false：图传已走 UDP，WS 文本帧不内嵌 JPEG（该参数仅用于日志）。
    static String s_last_cmd;
    if (s_last_cmd != type) {
        log_i("[ws] forward to cmd: %s", type);
        s_last_cmd = type;
    }
    cmd::handle(json, false, ws_cmd_reply, &fd);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        // 非 WS 升级的浏览器访问根路径 → 重定向到 /stream
        if (httpd_req_get_hdr_value_len(req, "Upgrade") == 0) {
            httpd_resp_set_status(req, "302 Found");
            httpd_resp_set_hdr(req, "Location", "/stream");
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }
        tune_socket(httpd_req_to_sockfd(req)); // WS 握手 fd，图传前先调好窗口/Nagle
        return ESP_OK; // WS 握手，101 由 httpd 自动完成
    }

    httpd_ws_frame_t pkt = {0};
    pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
    if (ret != ESP_OK) return ret;
    if (pkt.len == 0) return ESP_OK;
    s_ws_last_rx_ms = (uint32_t)(esp_timer_get_time() / 1000);  // 任何上行都视为有活动，续活判定准
    s_ws_ping_pending = false;                                   // 收到上行即撤销探测等待，防误杀

    // 二进制帧 = 编辑图（裸 JPEG，手机→板）：覆盖暂存供 ai_goal{use_image}消费。
    if (pkt.type == HTTPD_WS_TYPE_BINARY) {
        if (pkt.len > WS_EDIT_IMG_MAX) return ESP_FAIL;  // 超大帧，断开
        char *ibuf = (char *)malloc(pkt.len);
        if (!ibuf) return ESP_ERR_NO_MEM;
        pkt.payload = (uint8_t *)ibuf;
        ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
        if (ret == ESP_OK && pkt.type == HTTPD_WS_TYPE_BINARY) {
            ai::set_edited_image((uint8_t *)ibuf, pkt.len);
        }
        free(ibuf);
        return ret;
    }

    // 文本帧 = 指令 JSON（≤512B，异常长帧断开）
    if (pkt.len > 512) return ESP_FAIL;
    char *buf = (char *)malloc(pkt.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    pkt.payload = (uint8_t *)buf;
    ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
    if (ret == ESP_OK && pkt.type == HTTPD_WS_TYPE_TEXT) {
        buf[pkt.len] = 0;
        ws_handle_text(buf, httpd_req_to_sockfd(req));
    }
    free(buf);
    return ret;
}

// core 3.x / IDF 无 httpd_ws_client_iterate：改由 httpd_get_client_list 取全部活跃
// fd，再用 httpd_ws_get_fd_info 筛出 WEBSOCKET 会话（HTTP 会话不计入）。

static bool ws_send_jpeg_to_ws_clients(camera_fb_t *fb, bool *has_client)
{
    bool any = false;
    int fds[WS_MAX_CLIENTS];
    size_t n = WS_MAX_CLIENTS;
    if (httpd_get_client_list(stream_httpd, &n, fds) != ESP_OK) return false;
    for (size_t i = 0; i < n; i++) {
        if (httpd_ws_get_fd_info(stream_httpd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        any = true;
        if (has_client) *has_client = true;
        if (fb) ws_send_jpeg(fds[i], fb);
    }
    return any;
}

// 文本广播给全部 WS 客户端（本直驱状态推送，async 安全）。
static void ws_send_text_to_ws_clients(const char *text)
{
    int fds[WS_MAX_CLIENTS];
    size_t n = WS_MAX_CLIENTS;
    if (httpd_get_client_list(stream_httpd, &n, fds) != ESP_OK) return;
    for (size_t i = 0; i < n; i++) {
        if (httpd_ws_get_fd_info(stream_httpd, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) continue;
        ws_send_text(fds[i], text);
    }
}

// 统一日志模块的转发器：WS 广播 + BLE status 通知（在 blog 转发任务线程执行）。
// 由 startCameraServer 注册，供 blog::logf 排队后统一发手机。
static void send_log_to_phone(const char *json)
{
    ws_send_text_to_ws_clients(json);
    ble::send_status(json);
}

// 图传推流任务：stream 开启时按帧率向所有 WS 客户端推 JPEG 帧；
// 同时探测 WS 客户端存在性喂给 ble::set_ws_connected（status.ws），并做对端活体探测：
// 连续 WS_IDLE_PING_MS 无上行 → 发探测 ping；再 WS_PING_TIMEOUT_MS 无任意上行 → 判死，
// 踢掉全部 WS fd 并恢复 BLE 广播（供再次配网/兜底）。assume 单手机客户端（全局 last_rx）。
static void ws_stream_task(void *arg)
{
    while (true) {
        bool has_client = false;
        // WS 客户端在位探测：必须每个 tick 无条件执行。它是 has_client 的唯一来源，
        // 而 has_client 既喂 ble::set_ws_connected，又决定下面「客户端离线」的判定。
        // ⚠️ 曾经只在"抓到新帧"分支里探测：续传跨 tick 那几拍 has_client 恒为 false，
        // 于是刚开播就被判成客户端离线 → 停推流 + 清 UDP 对端（画面卡在首帧、连接显示掉线）。
        ws_send_jpeg_to_ws_clients(nullptr, &has_client);

        // 升级中一律走"停推流"分支：闸门在 command/ws_handle_text 已挡住重新开启，这里兜底，
        // 顺带把在途帧的 PSRAM 副本放掉，把内存与 core1 让给固件写入。
        if (cmd::streaming() && !ota::active()) {
            if (s_out_active) {
                udp_pump();              // 上一帧还在续传：按队列空闲续片，不抢新帧
                // 弃帧只针对"发不动"：链路彻底不消化(持续一段时间一片都没出去)，
                // 或单帧拖太久(超过在途上限，画面已陈旧到没意义)。慢链路只要有进展就不会被弃——
                // 帧率自适应到吞吐，而不是把帧丢一半。
                uint64_t now_us = esp_timer_get_time();
                bool stall    = (now_us - s_out_last_ok > OUT_FRAME_STALL_US);
                bool overtime = (now_us - s_out_start   > OUT_FRAME_MAX_US);
                if (s_out_active && (stall || overtime)) {
                    // 必须打印命中的是哪条判据：超时弃帧时"无进展"往往远小于其阈值(400ms)，
                    // 只印它会把排查方向带偏（曾据此误判链路无进展）。
                    blog::logf(blog::WS, "[udp] 弃帧 fid=%u seq=%u/%u len=%u 判据=%s 无进展=%dms 在途=%dms errno=%d 失败=%u",
                               s_out_fid, s_out_seq_next, s_out_count, (unsigned)s_out_len,
                               stall ? "无进展" : "在途超时",
                               (int)((now_us - s_out_last_ok) / 1000),
                               (int)((now_us - s_out_start) / 1000),
                               s_out_errno, (unsigned)s_out_fail);
                    s_out_active = false;
                }
            } else {
                uint64_t grab_us = esp_timer_get_time();
                // 上限帧率：整帧间隔不小于 1/max_fps。不封顶则"链路能推多快推多快"，接收端吃不下只会
                // 积压成延迟，反过来挤占控制面（pong 应答）的时序。
                // AI 任务进行中降到一半 FPS：高帧推流压 CPU + 碎内部 DMA 池（TX pbuf 与 RX 同池互抢），
                // 实测开着图传会让 DMA 块告警连发、并发 AI 被拖慢；降帧把两条都让给 AI。
                int max_fps = ai::busy() ? (UDP_STREAM_MAX_FPS / 2) : UDP_STREAM_MAX_FPS;
                if (grab_us - s_out_last_grab_us >= (1000000ULL / max_fps)) {
                    camera_fb_t *fb = cam::grab();    // 图传高频：直接抓帧，拷入 PSRAM 后立刻还缓冲
                    if (fb) {
                        s_out_last_grab_us = grab_us;
                        size_t flen = cam::jpeg_len(fb);   // 勿用 fb->len：虚高会把后续帧当本帧推出去
                        bool started = udp_start_frame(fb->buf, flen);
                        cam::return_frame(fb);
                        if (started) udp_pump();
                    }
                }
            }
        } else {
            // 停推流：中断在途帧并释放 PSRAM 帧副本（下次开播重新分配）
            s_out_active = false;
            udp_out_free();
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (has_client) {
            if (s_ws_last_rx_ms == 0) s_ws_last_rx_ms = now;  // 首个挂上的时刻作为 idle 起点
            if (!s_ws_ping_pending && (int32_t)(now - s_ws_last_rx_ms) >= (int32_t)WS_IDLE_PING_MS) {
                s_ws_ping_pending = true;
                s_ws_ping_at_ms = now;
                s_ws_ping_try = 1;
                ws_ping_all();
                blog::logf(blog::WS, "idle 超时，发探测 ping (#%u)", (unsigned)s_ws_ping_try);
            } else if (s_ws_ping_pending &&
                       (int32_t)(now - s_ws_ping_at_ms) >= (int32_t)WS_PING_TIMEOUT_MS) {
                // 单轮无应答不立刻判死：手机主线程可能只是这一拍被图传解码占住（同步解码、
                // 与 WS 读包同线程），重发一轮给它机会；连续多轮都没上行才认定真断。
                if (s_ws_ping_try >= WS_PING_MAX_TRY) {
                    // 判死：踢 fd，下轮 has_client 回落 → 尾部恢复广播
                    s_ws_ping_pending = false;
                    s_ws_ping_try = 0;
                    s_ws_last_rx_ms = 0;
                    has_client = false;
                    ws_kick_all();
                    blog::logf(blog::WS, "探测 %u 轮无应答，判定断线，恢复广播", (unsigned)WS_PING_MAX_TRY);
                } else {
                    s_ws_ping_at_ms = now;
                    s_ws_ping_try++;
                    ws_ping_all();
                    blog::logf(blog::WS, "探测无应答，重发 ping (#%u/%u)",
                               (unsigned)s_ws_ping_try, (unsigned)WS_PING_MAX_TRY);
                }
            }
        } else {
            s_ws_ping_pending = false;  // 无 WS 挂载时复位探测态
            s_ws_ping_try = 0;
        }

        // 客户端从有到无（手机退出且未先发 stream off）：停掉它发起的推流，
        // 否则 ws 任务持续白抓帧，BLE 广播也无机会恢复。
        if (has_client) {
            s_ws_had_client = true;
        } else if (s_ws_had_client) {
            s_ws_had_client = false;
            cmd::set_streaming(false);
            udp_peer_clear();  // WS 会话失效，随普通图传一起停掉 UDP 对端
            blog::logf(blog::WS, "客户端离线，停止推流");
        }

        ble::set_ws_connected(has_client);
        // 本地直驱状态 → 手机（/log exec on 开启时实时推送；替代原执行板上行帧镜像）。
        // 不依赖 WS 客户端在位：纯蓝牙（WS 未连）时经 BLE status 通知也可直达手机。
        // 默认关：空闲不刷屏。开启后约 400ms 推一条 exec::read_state 合成的状态文本；
        // 状态文本与上次完全相同时跳过（防刷屏，只有变化才推）。
        static uint32_t s_last_status_ms = 0;
        static char s_last_state[192] = {0};
        // OTA 升级期间停推周期状态：每 400ms 一条 WS+BLE 通知纯属抢射频的杂音
        if (!ota::active() && blog::enabled(blog::EXEC) &&
            (int32_t)(now - s_last_status_ms) >= (int32_t)400) {
            s_last_status_ms = now;
            char st[256];  // 状态含抓手前端 XZ 与 PWM + 撞边界/位姿没到位诊断，需足量避免截断
            if (exec::read_state(st, sizeof(st)) && strcmp(st, s_last_state)) {
                strncpy(s_last_state, st, sizeof(s_last_state) - 1);
                s_last_state[sizeof(s_last_state) - 1] = 0;
                JsonDocument sdoc;
                sdoc["type"] = "exec_status";
                JsonObject sp = sdoc["params"].to<JsonObject>();
                sp["text"] = st;
                String js;
                serializeJson(sdoc, js);
                if (has_client) ws_send_text_to_ws_clients(js.c_str());
                ble::send_status(js.c_str());
            }
        }
        // 图传已改走 UDP：WS 客户端在位仅作指令/状态通道，几乎不占射频；
        // 只有"真在推帧"（UDP 图传 / MJPEG 推流）才停 BLE 广播，否则手机随时可发现
        // VisionS3 重连（此前 WS 常挂/半开会让广播永久关闭，导致手机不重启连不上）。
        ble::set_transmission(isStreaming || cmd::streaming());
        // 空转降频：无 WS 客户端且非推流时，此任务只需维持 BLE 广播/心跳探测，无需高频轮询。
        // 降低空转 CPU，让 core1 让给图传/AI。
        // 推流中用短节拍：udp_pump 已按驱动队列排空速度自行限流，这里只需尽快回到 pump；
        // 若仍用 100ms 长节拍，吞吐会再被钉在「驱动队列容量 × 10Hz」上（见 udp_pump 注释）。
        if (cmd::streaming())
            vTaskDelay(pdMS_TO_TICKS(OUT_PUMP_TICK_MS));
        else if (has_client)
            vTaskDelay(pdMS_TO_TICKS(1000 / WS_STREAM_FPS));
        else
            vTaskDelay(pdMS_TO_TICKS(500));
    }
}
#endif // CONFIG_HTTPD_WS_SUPPORT

#if CONFIG_ESP_FACE_DETECT_ENABLED

static int8_t detection_enabled = 0;

// #if TWO_STAGE
// static HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
// static HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
// #else
// static HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
// #endif

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static int8_t recognition_enabled = 0;
static int8_t is_enrolling = 0;

#if QUANT_TYPE
    // S16 model
    FaceRecognition112V1S16 recognizer;
#else
    // S8 model
    FaceRecognition112V1S8 recognizer;
#endif
#endif

#endif

typedef struct
{
    size_t size;  //number of values used for filtering
    size_t index; //current value index
    size_t count; //value count
    int sum;
    int *values; //array to be filled with values
} ra_filter_t;

static ra_filter_t ra_filter;

static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size)
{
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if (!filter->values)
    {
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
static int ra_filter_run(ra_filter_t *filter, int value)
{
    if (!filter->values)
    {
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size)
    {
        filter->count++;
    }
    return filter->sum / filter->count;
}
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static void rgb_print(fb_data_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(fb_data_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
    }
    vsnprintf(temp, len + 1, format, arg);
    va_end(arg);
    rgb_print(fb, color, temp);
    if (len > 64)
    {
        free(temp);
    }
    return len;
}
#endif
static void draw_face_boxes(fb_data_t *fb, std::list<dl::detect::result_t> *results, int face_id)
{
    int x, y, w, h;
    uint32_t color = FACE_COLOR_YELLOW;
    if (face_id < 0)
    {
        color = FACE_COLOR_RED;
    }
    else if (face_id > 0)
    {
        color = FACE_COLOR_GREEN;
    }
    if(fb->bytes_per_pixel == 2){
        //color = ((color >> 8) & 0xF800) | ((color >> 3) & 0x07E0) | (color & 0x001F);
        color = ((color >> 16) & 0x001F) | ((color >> 3) & 0x07E0) | ((color << 8) & 0xF800);
    }
    int i = 0;
    for (std::list<dl::detect::result_t>::iterator prediction = results->begin(); prediction != results->end(); prediction++, i++)
    {
        // rectangle box
        x = (int)prediction->box[0];
        y = (int)prediction->box[1];
        w = (int)prediction->box[2] - x + 1;
        h = (int)prediction->box[3] - y + 1;
        if((x + w) > fb->width){
            w = fb->width - x;
        }
        if((y + h) > fb->height){
            h = fb->height - y;
        }
        fb_gfx_drawFastHLine(fb, x, y, w, color);
        fb_gfx_drawFastHLine(fb, x, y + h - 1, w, color);
        fb_gfx_drawFastVLine(fb, x, y, h, color);
        fb_gfx_drawFastVLine(fb, x + w - 1, y, h, color);
#if TWO_STAGE
        // landmarks (left eye, mouth left, nose, right eye, mouth right)
        int x0, y0, j;
        for (j = 0; j < 10; j+=2) {
            x0 = (int)prediction->keypoint[j];
            y0 = (int)prediction->keypoint[j+1];
            fb_gfx_fillRect(fb, x0, y0, 3, 3, color);
        }
#endif
    }
}

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
static int run_face_recognition(fb_data_t *fb, std::list<dl::detect::result_t> *results)
{
    std::vector<int> landmarks = results->front().keypoint;
    int id = -1;

    Tensor<uint8_t> tensor;
    tensor.set_element((uint8_t *)fb->data).set_shape({fb->height, fb->width, 3}).set_auto_free(false);

    int enrolled_count = recognizer.get_enrolled_id_num();

    if (enrolled_count < FACE_ID_SAVE_NUMBER && is_enrolling){
        id = recognizer.enroll_id(tensor, landmarks, "", true);
        log_i("Enrolled ID: %d", id);
        rgb_printf(fb, FACE_COLOR_CYAN, "ID[%u]", id);
    }

    face_info_t recognize = recognizer.recognize(tensor, landmarks);
    if(recognize.id >= 0){
        rgb_printf(fb, FACE_COLOR_GREEN, "ID[%u]: %.2f", recognize.id, recognize.similarity);
    } else {
        rgb_print(fb, FACE_COLOR_RED, "Intruder Alert!");
    }
    return recognize.id;
}
#endif
#endif

#if CONFIG_LED_ILLUMINATOR_ENABLED
void enable_led(bool en)
{ // Turn LED On or Off
    int duty = en ? led_duty : 0;
    if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY))
    {
        duty = CONFIG_LED_MAX_INTENSITY;
    }
    ledcWrite(s_led_pin, duty);
    log_i("Set LED intensity to %d", duty);
}
#endif

static esp_err_t bmp_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint64_t fr_start = esp_timer_get_time();
#endif
    fb = cam::grab();
    if (!fb)
    {
        log_e("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/x-windows-bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char ts[32];
    snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);


    uint8_t * buf = NULL;
    size_t buf_len = 0;
    bool converted = frame2bmp(fb, &buf, &buf_len);
    esp_camera_fb_return(fb);
    if(!converted){
        log_e("BMP Conversion failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    res = httpd_resp_send(req, (const char *)buf, buf_len);
    free(buf);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint64_t fr_end = esp_timer_get_time();
#endif
    log_i("BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);
    return res;
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index)
    {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
    {
        return 0;
    }
    j->len += len;
    return len;
}

// /zoomshot: 对当帧按目标区域裁出放大 JPEG —— 手动"凑近看"两指与目标，等价于 AI 的 zoom。
// 参数(GET query)：px,py=框中心(归一化 0~1，默认 0.5)；scale=倍率(框=全幅 1/scale，默认 2，上限 8)；
// out_w/out_h=输出尺寸(像素，默认 320×240)；quality=JPEG 质量(默认 80)。
// 复用 magnify::crop_to_jpg：它内部自持 cam 解码/编码共享锁，与 AI worker/mvfy 的软解并发互斥，安全。
static esp_err_t zoomshot_handler(httpd_req_t *req)
{
    camera_fb_t *fb = cam::grab();
    if (!fb || fb->format != PIXFORMAT_JPEG) {
        if (fb) esp_camera_fb_return(fb);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    const size_t w = fb->width, h = fb->height;
    float px = 0.5f, py = 0.5f, sc = 2.0f;
    int outw = 320, outh = 240, quality = 80;
    char qs[160];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK) {
        char p[16];
        if (httpd_query_key_value(qs, "px", p, sizeof(p)) == ESP_OK) px = constrain(atof(p), 0.0f, 1.0f);
        if (httpd_query_key_value(qs, "py", p, sizeof(p)) == ESP_OK) py = constrain(atof(p), 0.0f, 1.0f);
        if (httpd_query_key_value(qs, "scale", p, sizeof(p)) == ESP_OK) sc = constrain(atof(p), 1.0f, 8.0f);
        if (httpd_query_key_value(qs, "out_w", p, sizeof(p)) == ESP_OK) { outw = atoi(p); if (outw <= 0 || outw > 4096) outw = 320; }
        if (httpd_query_key_value(qs, "out_h", p, sizeof(p)) == ESP_OK) { outh = atoi(p); if (outh <= 0 || outh > 4096) outh = 240; }
        if (httpd_query_key_value(qs, "quality", p, sizeof(p)) == ESP_OK) { quality = atoi(p); if (quality < 1 || quality > 100) quality = 80; }
    }
    float half = 0.5f / sc;   // 框 = 全幅的 1/scale（宽高各自）
    float x0 = constrain(px - half, 0.0f, 1.0f), x1 = constrain(px + half, 0.0f, 1.0f);
    float y0 = constrain(py - half, 0.0f, 1.0f), y1 = constrain(py + half, 0.0f, 1.0f);
    size_t cap = (size_t)outw * outh + 512;
    uint8_t *out = (uint8_t *)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
    if (!out) { esp_camera_fb_return(fb); httpd_resp_send_500(req); return ESP_FAIL; }
    size_t olen = 0;
    bool ok = magnify::crop_to_jpg(fb->buf, cam::jpeg_len(fb), (int)w, (int)h,
                                   x0, y0, x1, y1, out, cap, &olen, outw, outh, quality);
    esp_camera_fb_return(fb);
    if (!ok || olen == 0) { heap_caps_free(out); httpd_resp_send_500(req); return ESP_FAIL; }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char *)out, (int)olen);
    heap_caps_free(out);
    return res;
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    int64_t fr_start = esp_timer_get_time();
#endif

#if CONFIG_LED_ILLUMINATOR_ENABLED
    enable_led(true);
    vTaskDelay(150 / portTICK_PERIOD_MS); // The LED needs to be turned on ~150ms before the call to esp_camera_fb_get()
    fb = cam::grab();             // or it won't be visible in the frame. A better way to do this is needed.
    enable_led(false);
#else
    fb = cam::grab();
#endif

    if (!fb)
    {
        log_e("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char ts[32];
    snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

    // X-Raw-Len = 驱动自己报的 fb->len（**未**经 jpeg_len 校正）。正文只发校正后的真实长度，
    // 于是"驱动是否还在把多帧拼进一格"从正文里就看不出来了 —— 拿它跟本次实际发出的字节数一比
    // 即可判定（不等即驱动侧拼帧，差值就是被截掉的尾巴）。诊断用，别当数据用。
    char raw[24];
    snprintf(raw, sizeof(raw), "%u", (unsigned)fb->len);
    httpd_resp_set_hdr(req, "X-Raw-Len", (const char *)raw);

#if CONFIG_ESP_FACE_DETECT_ENABLED
    size_t out_len, out_width, out_height;
    uint8_t *out_buf;
    bool s;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    bool detected = false;
#endif
    int face_id = 0;
    if (!detection_enabled || fb->width > 400)
    {
#endif
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        size_t fb_len = 0;
#endif
        if (fb->format == PIXFORMAT_JPEG)
        {
            // 勿用 fb->len：驱动可能报大（缓冲里裹了后续帧），发出去就是"一张图带尾巴"
            size_t jlen = cam::jpeg_len(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            fb_len = jlen;
#endif
            res = httpd_resp_send(req, (const char *)fb->buf, jlen);
        }
        else
        {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            fb_len = jchunk.len;
#endif
        }
        esp_camera_fb_return(fb);
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        int64_t fr_end = esp_timer_get_time();
#endif
        log_i("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
        return res;
#if CONFIG_ESP_FACE_DETECT_ENABLED
    }

    jpg_chunking_t jchunk = {req, 0};

    if (fb->format == PIXFORMAT_RGB565
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
     && !recognition_enabled
#endif
     ){
#if TWO_STAGE
        HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
        HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
        std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
        std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3}, candidates);
#else
        HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
        std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
#endif
        if (results.size() > 0) {
            fb_data_t rfb;
            rfb.width = fb->width;
            rfb.height = fb->height;
            rfb.data = fb->buf;
            rfb.bytes_per_pixel = 2;
            rfb.format = FB_RGB565;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            detected = true;
#endif
            draw_face_boxes(&rfb, &results, face_id);
        }
        s = fmt2jpg_cb(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 90, jpg_encode_stream, &jchunk);
        esp_camera_fb_return(fb);
    } else
    {
        out_len = fb->width * fb->height * 3;
        out_width = fb->width;
        out_height = fb->height;
        out_buf = (uint8_t*)malloc(out_len);
        if (!out_buf) {
            log_e("out_buf malloc failed");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
        esp_camera_fb_return(fb);
        if (!s) {
            free(out_buf);
            log_e("To rgb888 failed");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        fb_data_t rfb;
        rfb.width = out_width;
        rfb.height = out_height;
        rfb.data = out_buf;
        rfb.bytes_per_pixel = 3;
        rfb.format = FB_BGR888;

#if TWO_STAGE
        HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
        HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
        std::list<dl::detect::result_t> &candidates = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
        std::list<dl::detect::result_t> &results = s2.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3}, candidates);
#else
        HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
        std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
#endif

        if (results.size() > 0) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            detected = true;
#endif
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
            if (recognition_enabled) {
                face_id = run_face_recognition(&rfb, &results);
            }
#endif
            draw_face_boxes(&rfb, &results, face_id);
        }

        s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
        free(out_buf);
    }

    if (!s) {
        log_e("JPEG compression failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    int64_t fr_end = esp_timer_get_time();
#endif
    log_i("FACE: %uB %ums %s%d", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start) / 1000), detected ? "DETECTED " : "", face_id);
    return res;
#endif
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    // 升级中直接拒绝开新流：整条 MJPEG 是持续抓帧+编码的 CPU 大户，会与固件写入抢 core1、
    // 射频与 PSRAM；只靠下面循环里的 break 会让发起方以为流已建立（画面僵住却不见报错）。
    if (ota::active())
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "固件升级中，暂不提供视频流");

    camera_fb_t *fb = NULL;
    struct timeval _timestamp;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[128];
#if CONFIG_ESP_FACE_DETECT_ENABLED
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        bool detected = false;
        int64_t fr_ready = 0;
        int64_t fr_recognize = 0;
        int64_t fr_encode = 0;
        int64_t fr_face = 0;
        int64_t fr_start = 0;
    #endif
    int face_id = 0;
    size_t out_len = 0, out_width = 0, out_height = 0;
    uint8_t *out_buf = NULL;
    bool s = false;
#if TWO_STAGE
    HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
    HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
#else
    HumanFaceDetectMSR01 s1(0.3F, 0.5F, 10, 0.2F);
#endif
#endif

    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    // 调大发送缓冲 + 关 Nagle，避免新连接慢启动期吞吐爬坡慢（图传前慢后快）
    tune_socket(httpd_req_to_sockfd(req));

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "60");

#if CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = true;
    enable_led(true);
#endif

    while (true)
    {
        // 有 OTA 会话在写固件：本次 MJPEG 推流就地结束，把射频与 CPU 让给固件传输
        // （流退出时下方 isStreaming=false 会复位，OTA 结束后客户端重连即可恢复）
        if (ota::active()) break;
#if CONFIG_ESP_FACE_DETECT_ENABLED
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        detected = false;
    #endif
        face_id = 0;
#endif

        fb = cam::grab();
        if (!fb)
        {
            log_e("Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            _timestamp.tv_sec = fb->timestamp.tv_sec;
            _timestamp.tv_usec = fb->timestamp.tv_usec;
#if CONFIG_ESP_FACE_DETECT_ENABLED
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            fr_face = fr_start;
    #endif
            if (!detection_enabled || fb->width > 400)
            {
#endif
                if (fb->format != PIXFORMAT_JPEG)
                {
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if (!jpeg_converted)
                    {
                        log_e("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                }
                else
                {
                    _jpg_buf_len = cam::jpeg_len(fb);   // 勿用 fb->len：虚高会让一个 part 里塞进多帧
                    _jpg_buf = fb->buf;
                }
#if CONFIG_ESP_FACE_DETECT_ENABLED
            }
            else
            {
                if (fb->format == PIXFORMAT_RGB565
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
                    && !recognition_enabled
#endif
                ){
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    fr_ready = esp_timer_get_time();
#endif
#if TWO_STAGE
                    std::list<dl::detect::result_t> &candidates = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
                    std::list<dl::detect::result_t> &results = s2.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3}, candidates);
#else
                    std::list<dl::detect::result_t> &results = s1.infer((uint16_t *)fb->buf, {(int)fb->height, (int)fb->width, 3});
#endif
#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    fr_face = esp_timer_get_time();
                    fr_recognize = fr_face;
#endif
                    if (results.size() > 0) {
                        fb_data_t rfb;
                        rfb.width = fb->width;
                        rfb.height = fb->height;
                        rfb.data = fb->buf;
                        rfb.bytes_per_pixel = 2;
                        rfb.format = FB_RGB565;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                        detected = true;
#endif
                        draw_face_boxes(&rfb, &results, face_id);
                    }
                    s = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if (!s) {
                        log_e("fmt2jpg failed");
                        res = ESP_FAIL;
                    }
#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                    fr_encode = esp_timer_get_time();
#endif
                } else
                {
                    out_len = fb->width * fb->height * 3;
                    out_width = fb->width;
                    out_height = fb->height;
                    out_buf = (uint8_t*)malloc(out_len);
                    if (!out_buf) {
                        log_e("out_buf malloc failed");
                        res = ESP_FAIL;
                    } else {
                        s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
                        esp_camera_fb_return(fb);
                        fb = NULL;
                        if (!s) {
                            free(out_buf);
                            log_e("To rgb888 failed");
                            res = ESP_FAIL;
                        } else {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                            fr_ready = esp_timer_get_time();
#endif

                            fb_data_t rfb;
                            rfb.width = out_width;
                            rfb.height = out_height;
                            rfb.data = out_buf;
                            rfb.bytes_per_pixel = 3;
                            rfb.format = FB_BGR888;

#if TWO_STAGE
                            std::list<dl::detect::result_t> &candidates = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
                            std::list<dl::detect::result_t> &results = s2.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3}, candidates);
#else
                            std::list<dl::detect::result_t> &results = s1.infer((uint8_t *)out_buf, {(int)out_height, (int)out_width, 3});
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                            fr_face = esp_timer_get_time();
                            fr_recognize = fr_face;
#endif

                            if (results.size() > 0) {
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                                detected = true;
#endif
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
                                if (recognition_enabled) {
                                    face_id = run_face_recognition(&rfb, &results);
    #if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                                    fr_recognize = esp_timer_get_time();
    #endif
                                }
#endif
                                draw_face_boxes(&rfb, &results, face_id);
                            }
                            s = fmt2jpg(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);
                            free(out_buf);
                            if (!s) {
                                log_e("fmt2jpg failed");
                                res = ESP_FAIL;
                            }
#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
                            fr_encode = esp_timer_get_time();
#endif
                        }
                    }
                }
            }
#endif
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            log_e("Send frame failed");
            break;
        }
        int64_t fr_end = esp_timer_get_time();

#if CONFIG_ESP_FACE_DETECT_ENABLED && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        int64_t ready_time = (fr_ready - fr_start) / 1000;
        int64_t face_time = (fr_face - fr_ready) / 1000;
        int64_t recognize_time = (fr_recognize - fr_face) / 1000;
        int64_t encode_time = (fr_encode - fr_recognize) / 1000;
        int64_t process_time = (fr_encode - fr_start) / 1000;
#endif

        int64_t frame_time = fr_end - last_frame;
        frame_time /= 1000;
        last_frame = fr_end;  // 修复：原未更新，fps 恒为 0.0
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
        uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
        log_i("MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)"
#if CONFIG_ESP_FACE_DETECT_ENABLED
                      ", %u+%u+%u+%u=%u %s%d"
#endif
                 ,
                 (uint32_t)(_jpg_buf_len),
                 (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time,
                 avg_frame_time, 1000.0 / avg_frame_time
#if CONFIG_ESP_FACE_DETECT_ENABLED
                 ,
                 (uint32_t)ready_time, (uint32_t)face_time, (uint32_t)recognize_time, (uint32_t)encode_time, (uint32_t)process_time,
                 (detected) ? "DETECTED " : "", face_id
#endif
        );
    }

#if CONFIG_LED_ILLUMINATOR_ENABLED
    isStreaming = false;
    enable_led(false);
#endif

    return res;
}

static esp_err_t parse_get(httpd_req_t *req, char **obuf)
{
    char *buf = NULL;
    size_t buf_len = 0;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char *)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            *obuf = buf;
            return ESP_OK;
        }
        free(buf);
    }
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

// ── AI 抓帧留档（调试用，见 ai_dump.h）──────────────────────────────────────
// /ai_dump         → JSON 清单：这一轮 AI 收到的是哪张图、它据此做了什么（含被闸门拒的原因）
// /ai_dump?after=N → 长轮询：挂住不返回，等出现序号 >N 的定案帧就立刻回清单（最多等 AI_DUMP_FOLLOW_MS）
// /ai_frame?seq=N  → 该轮的原始 JPEG（就是发往云端的那份字节）
// 只在 `/log ai on` 期间有内容：留档由 AI worker 按 blog 开关驱动，关着时不占一个字节。
//
// 为什么要长轮询：抓取端若靠定时轮询，快了空转、慢了漏帧（环形只 6 槽，一轮 AI 约 2s，十几秒的
// 卡顿就挤掉了）。挂住请求=把"什么时候有"交给板端说，抓取端不必猜周期，也不会有轮询间隙。
// 代价只在这条 HTTP 会话自己的任务里：不碰 WS/UDP，AI 线程也不为调试工具多等一毫秒。
#define AI_DUMP_FOLLOW_MS 15000      // 单次挂起上限（抓取端的读超时必须大于它）
#define AI_DUMP_FOLLOW_TICK_MS 200   // 检查间隔 ≈ "定案 → 抓取端看见" 的延迟

static esp_err_t ai_dump_handler(httpd_req_t *req)
{
    // after=N → 长轮询模式。缺参则立即回（抓取端取历史/老用法）。
    uint32_t after = 0;
    bool follow = false;
    char q[64];
    size_t ql = httpd_req_get_url_query_len(req);
    if (ql > 0 && ql + 1 <= sizeof(q) && httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[16];
        if (httpd_query_key_value(q, "after", v, sizeof(v)) == ESP_OK) {
            after = (uint32_t)strtoul(v, NULL, 10);
            follow = true;
        }
    }
    if (follow) {
        uint32_t t0 = (uint32_t)millis();
        while ((uint32_t)(millis() - t0) < AI_DUMP_FOLLOW_MS) {
            if (ai::dump_newest_seq() > after) break;               // 有新定案帧：立刻回
            // 留档关着 / 正在升级固件：不会再产出新帧，挂着只是白占一条会话（也给抓取端一个即时答复，
            // 好让它把"板端没在留档"这句话说出口，而不是干等 15 秒）。
            if (!blog::enabled(blog::AI) || ota::active()) break;
            vTaskDelay(pdMS_TO_TICKS(AI_DUMP_FOLLOW_TICK_MS));
        }
    }
    // 每条 ~180 字节 × 槽数；堆上开（httpd 栈 8192，清单虽小也别占它），
    // 且不能 static —— httpd 多请求并发，静态缓冲会被另一个请求盖掉。
    // 放在等待之后申请：挂起期间不占堆（AI 链路的内部堆很紧张，见 ai_client 顶部注释）。
    const size_t cap = 2048;
    char *buf = (char *)malloc(cap);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    ai::dump_manifest(buf, cap);   // 失败也回合法空清单，前端不必区分错误形状
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    free(buf);
    return r;
}

static esp_err_t ai_frame_handler(httpd_req_t *req)
{
    uint32_t seq = 0;
    char q[64];
    if (httpd_req_get_url_query_len(req) + 1 <= sizeof(q) &&
        httpd_req_get_url_query_len(req) > 0 &&
        httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[16];
        if (httpd_query_key_value(q, "seq", v, sizeof(v)) == ESP_OK) seq = (uint32_t)strtoul(v, NULL, 10);
    }
    if (seq == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need seq (see /ai_dump)");
        return ESP_FAIL;
    }
    // 接收缓冲走 PSRAM：单帧可达 96KB，与相机缓冲同池，不挤内部堆。
    uint8_t *buf = (uint8_t *)heap_caps_malloc(AI_DUMP_FRAME_MAX, MALLOC_CAP_SPIRAM);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    uint32_t ms = 0;
    size_t n = ai::dump_copy(seq, buf, AI_DUMP_FRAME_MAX, &ms);
    if (n == 0) {
        free(buf);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such seq (or 0-byte frame)");
        return ESP_FAIL;
    }
    char hdr[48];
    snprintf(hdr, sizeof(hdr), "seq=%u ms=%u", (unsigned)seq, (unsigned)ms);
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-AiFrame", hdr);
    esp_err_t r = httpd_resp_send(req, (const char *)buf, n);
    free(buf);
    return r;
}

// ── panic 现场读取（/coredump）──────────────────────────────────────────────
// 为什么需要这条路：板子跑挂后网络同时没了、串口也不在手边（车架在车上），于是「复位=崩溃」
// 四个字就是全部线索。panic 的现场其实**已经**落在 coredump 分区里（partitions.csv 里那 512KB +
// CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y），但分区开了却没有读取通路，等于没有。
// 这条 GET 把它变成能远程取回的东西：
//   GET /coredump          → 文本：panic 原因串 + 崩在哪个任务 + PC + 回溯 PC 数组 + 固件 sha
//   GET /coredump?erase=1  → 清掉现场（panic 一次覆写一次；想留旧的先取再清）
// 拿到 bt 的 PC 后，在 PC 侧对**本次固件的 .elf** 做 addr2line 即得文件:行号。先核对回来的 sha
// 与手头 .elf 的 sha256 是否一致，否则解出来的是别人的代码。工具：`carctl.py coredump`。
// ⚠️ 依赖 CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH + DATA_FORMAT_ELF；库与头文件两侧都得开
// （已在 librebuild 的配置里核实一致），否则下面两个符号链接不上——那是响亮的失败，不是静默。
static esp_err_t coredump_handler(httpd_req_t *req)
{
    // erase=1：取完现场顺手清掉，免得下次 panic 的新现场跟旧的混在一起看。
    char q[32];
    size_t ql = httpd_req_get_url_query_len(req);
    if (ql > 0 && ql + 1 <= sizeof(q) && httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        char v[8];
        if (httpd_query_key_value(q, "erase", v, sizeof(v)) == ESP_OK && v[0] == '1') {
            esp_err_t e = esp_core_dump_image_erase();
            char m[48];
            snprintf(m, sizeof(m), "erase=%d\n", (int)e);
            httpd_resp_set_type(req, "text/plain");
            return httpd_resp_send(req, m, HTTPD_RESP_USE_STRLEN);
        }
    }

    // summary 结构 ~200 字节、reason 上限 256：都走堆。httpd 栈只有 8192 且已知偏紧
    // （见 startCameraServer 里那条注释），不能在这条链上加大局部变量。
    const size_t cap = 1024;
    esp_core_dump_summary_t *sum = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
    char *reason = (char *)malloc(256);
    char *out = (char *)malloc(cap);
    if (!sum || !reason || !out) {
        free(sum); free(reason); free(out);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    reason[0] = '\0';
    esp_err_t er = esp_core_dump_get_panic_reason(reason, 256);
    esp_err_t es = esp_core_dump_get_summary(sum);

    // reason 是 panic 处理器写的自由文本，可能带引号/换行；这里按行输出，把控制字符压成空格，
    // 免得一条乱字符把整段可读性毁掉（也让 PC 侧按行切片就够，不必上 JSON 转义）。
    for (char *p = reason; *p; p++)
        if ((unsigned char)*p < 0x20) *p = ' ';

    size_t n = 0;
#define CD_ADD(...) do { \
        if (n < cap) { int w = snprintf(out + n, cap - n, __VA_ARGS__); \
                       if (w > 0) n += (size_t)w; if (n > cap) n = cap; } \
    } while (0)
    CD_ADD("ok=%d\n", (int)(es == ESP_OK));
    CD_ADD("reason_err=%d\n", (int)er);
    CD_ADD("reason=%s\n", reason);
    if (es == ESP_OK) {
        CD_ADD("task=%s\n", sum->exc_task);
        CD_ADD("pc=0x%08x\n", (unsigned)sum->exc_pc);
        CD_ADD("cause=%u vaddr=0x%08x\n",
               (unsigned)sum->ex_info.exc_cause, (unsigned)sum->ex_info.exc_vaddr);
        CD_ADD("depth=%u corrupted=%d\n",
               (unsigned)sum->exc_bt_info.depth, (int)sum->exc_bt_info.corrupted);
        CD_ADD("bt=");
        for (uint32_t i = 0; i < sum->exc_bt_info.depth && i < 16; i++)
            CD_ADD("0x%08x ", (unsigned)sum->exc_bt_info.bt[i]);
        CD_ADD("\n");
        CD_ADD("sha=%s\n", (const char *)sum->app_elf_sha256);
    } else {
        CD_ADD("summary_err=%d\n", (int)es);
    }
#undef CD_ADD

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t r = httpd_resp_send(req, out, n);
    free(sum); free(reason); free(out);
    return r;
}

static esp_err_t cmd_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char variable[32];
    char value[32];

    if (parse_get(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }
    if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK ||
        httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int val = atoi(value);
    log_i("%s = %d", variable, val);
    sensor_t *s = esp_camera_sensor_get();
    int res = 0;

    if (!strcmp(variable, "framesize")) {
        if (s->pixformat == PIXFORMAT_JPEG) {
            res = s->set_framesize(s, (framesize_t)val);
        }
    }
    else if (!strcmp(variable, "quality"))
        res = s->set_quality(s, val);
    else if (!strcmp(variable, "contrast"))
        res = s->set_contrast(s, val);
    else if (!strcmp(variable, "brightness"))
        res = s->set_brightness(s, val);
    else if (!strcmp(variable, "saturation"))
        res = s->set_saturation(s, val);
    else if (!strcmp(variable, "gainceiling"))
        res = s->set_gainceiling(s, (gainceiling_t)val);
    else if (!strcmp(variable, "colorbar"))
        res = s->set_colorbar(s, val);
    else if (!strcmp(variable, "awb"))
        res = s->set_whitebal(s, val);
    else if (!strcmp(variable, "agc"))
        res = s->set_gain_ctrl(s, val);
    else if (!strcmp(variable, "aec"))
        res = s->set_exposure_ctrl(s, val);
    else if (!strcmp(variable, "hmirror"))
        res = s->set_hmirror(s, val);
    else if (!strcmp(variable, "vflip"))
        res = s->set_vflip(s, val);
    else if (!strcmp(variable, "awb_gain"))
        res = s->set_awb_gain(s, val);
    else if (!strcmp(variable, "agc_gain"))
        res = s->set_agc_gain(s, val);
    else if (!strcmp(variable, "aec_value"))
        res = s->set_aec_value(s, val);
    else if (!strcmp(variable, "aec2"))
        res = s->set_aec2(s, val);
    else if (!strcmp(variable, "dcw"))
        res = s->set_dcw(s, val);
    else if (!strcmp(variable, "bpc"))
        res = s->set_bpc(s, val);
    else if (!strcmp(variable, "wpc"))
        res = s->set_wpc(s, val);
    else if (!strcmp(variable, "raw_gma"))
        res = s->set_raw_gma(s, val);
    else if (!strcmp(variable, "lenc"))
        res = s->set_lenc(s, val);
    else if (!strcmp(variable, "special_effect"))
        res = s->set_special_effect(s, val);
    else if (!strcmp(variable, "wb_mode"))
        res = s->set_wb_mode(s, val);
    else if (!strcmp(variable, "ae_level"))
        res = s->set_ae_level(s, val);
#if CONFIG_LED_ILLUMINATOR_ENABLED
    else if (!strcmp(variable, "led_intensity")) {
        led_duty = val;
        if (isStreaming)
            enable_led(true);
    }
#endif

#if CONFIG_ESP_FACE_DETECT_ENABLED
    else if (!strcmp(variable, "face_detect")) {
        detection_enabled = val;
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
        if (!detection_enabled) {
            recognition_enabled = 0;
        }
#endif
    }
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    else if (!strcmp(variable, "face_enroll")){
        is_enrolling = !is_enrolling;
        log_i("Enrolling: %s", is_enrolling?"true":"false");
    }
    else if (!strcmp(variable, "face_recognize")) {
        recognition_enabled = val;
        if (recognition_enabled) {
            detection_enabled = val;
        }
    }
#endif
#endif
    else {
        log_i("Unknown command: %s", variable);
        res = -1;
    }

    if (res < 0) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static int print_reg(char * p, sensor_t * s, uint16_t reg, uint32_t mask){
    return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));
}

static esp_err_t status_handler(httpd_req_t *req)
{
    static char json_response[1024];

    sensor_t *s = esp_camera_sensor_get();
    char *p = json_response;
    *p++ = '{';

    if(s->id.PID == OV5640_PID || s->id.PID == OV3660_PID){
        for(int reg = 0x3400; reg < 0x3406; reg+=2){
            p+=print_reg(p, s, reg, 0xFFF);//12 bit
        }
        p+=print_reg(p, s, 0x3406, 0xFF);

        p+=print_reg(p, s, 0x3500, 0xFFFF0);//16 bit
        p+=print_reg(p, s, 0x3503, 0xFF);
        p+=print_reg(p, s, 0x350a, 0x3FF);//10 bit
        p+=print_reg(p, s, 0x350c, 0xFFFF);//16 bit

        for(int reg = 0x5480; reg <= 0x5490; reg++){
            p+=print_reg(p, s, reg, 0xFF);
        }

        for(int reg = 0x5380; reg <= 0x538b; reg++){
            p+=print_reg(p, s, reg, 0xFF);
        }

        for(int reg = 0x5580; reg < 0x558a; reg++){
            p+=print_reg(p, s, reg, 0xFF);
        }
        p+=print_reg(p, s, 0x558a, 0x1FF);//9 bit
    } else if(s->id.PID == OV2640_PID){
        p+=print_reg(p, s, 0xd3, 0xFF);
        p+=print_reg(p, s, 0x111, 0xFF);
        p+=print_reg(p, s, 0x132, 0xFF);
    }

    p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);
    p += sprintf(p, "\"pixformat\":%u,", s->pixformat);
    p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p += sprintf(p, "\"quality\":%u,", s->status.quality);
    p += sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p += sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p += sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p += sprintf(p, "\"awb\":%u,", s->status.awb);
    p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p += sprintf(p, "\"aec\":%u,", s->status.aec);
    p += sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p += sprintf(p, "\"agc\":%u,", s->status.agc);
    p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p += sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p += sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p += sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p += sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);
#if CONFIG_LED_ILLUMINATOR_ENABLED
    p += sprintf(p, ",\"led_intensity\":%u", led_duty);
#else
    p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
#if CONFIG_ESP_FACE_DETECT_ENABLED
    p += sprintf(p, ",\"face_detect\":%u", detection_enabled);
#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    p += sprintf(p, ",\"face_enroll\":%u,", is_enrolling);
    p += sprintf(p, "\"face_recognize\":%u", recognition_enabled);
#endif
#endif
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

static esp_err_t xclk_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _xclk[32];

    if (parse_get(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }
    if (httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK) {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int xclk = atoi(_xclk);
    log_i("Set XCLK: %d MHz", xclk);

    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_xclk(s, LEDC_TIMER_0, xclk);
    if (res) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t reg_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _reg[32];
    char _mask[32];
    char _val[32];

    if (parse_get(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }
    if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK ||
        httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK ||
        httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK) {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int reg = atoi(_reg);
    int mask = atoi(_mask);
    int val = atoi(_val);
    log_i("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_reg(s, reg, mask, val);
    if (res) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t greg_handler(httpd_req_t *req)
{
    char *buf = NULL;
    char _reg[32];
    char _mask[32];

    if (parse_get(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }
    if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK ||
        httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK) {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    free(buf);

    int reg = atoi(_reg);
    int mask = atoi(_mask);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->get_reg(s, reg, mask);
    if (res < 0) {
        return httpd_resp_send_500(req);
    }
    log_i("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

    char buffer[20];
    const char * val = itoa(res, buffer, 10);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, val, strlen(val));
}

static int parse_get_var(char *buf, const char * key, int def)
{
    char _int[16];
    if(httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK){
        return def;
    }
    return atoi(_int);
}

static esp_err_t pll_handler(httpd_req_t *req)
{
    char *buf = NULL;

    if (parse_get(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }

    int bypass = parse_get_var(buf, "bypass", 0);
    int mul = parse_get_var(buf, "mul", 0);
    int sys = parse_get_var(buf, "sys", 0);
    int root = parse_get_var(buf, "root", 0);
    int pre = parse_get_var(buf, "pre", 0);
    int seld5 = parse_get_var(buf, "seld5", 0);
    int pclken = parse_get_var(buf, "pclken", 0);
    int pclk = parse_get_var(buf, "pclk", 0);
    free(buf);

    log_i("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", bypass, mul, sys, root, pre, seld5, pclken, pclk);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);
    if (res) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t win_handler(httpd_req_t *req)
{
    char *buf = NULL;

    if (parse_get(req, &buf) != ESP_OK) {
        return ESP_FAIL;
    }

    int startX = parse_get_var(buf, "sx", 0);
    int startY = parse_get_var(buf, "sy", 0);
    int endX = parse_get_var(buf, "ex", 0);
    int endY = parse_get_var(buf, "ey", 0);
    int offsetX = parse_get_var(buf, "offx", 0);
    int offsetY = parse_get_var(buf, "offy", 0);
    int totalX = parse_get_var(buf, "tx", 0);
    int totalY = parse_get_var(buf, "ty", 0);
    int outputX = parse_get_var(buf, "ox", 0);
    int outputY = parse_get_var(buf, "oy", 0);
    bool scale = parse_get_var(buf, "scale", 0) == 1;
    bool binning = parse_get_var(buf, "binning", 0) == 1;
    free(buf);

    log_i("Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
    sensor_t *s = esp_camera_sensor_get();
    int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);
    if (res) {
        return httpd_resp_send_500(req);
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t *s = esp_camera_sensor_get();
    if (s != NULL) {
        if (s->id.PID == OV3660_PID) {
            return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
        } else if (s->id.PID == OV5640_PID) {
            return httpd_resp_send(req, (const char *)index_ov5640_html_gz, index_ov5640_html_gz_len);
        } else {
            return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
        }
    } else {
        log_e("Camera sensor not found");
        return httpd_resp_send_500(req);
    }
}

void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    // 20 个槽：现 14（camera_httpd 上 12 个 + /zoomshot + ota 的 /update GET/POST 2 个）。
    // 再加 URI 前先数一遍，超了 httpd_register_uri_handler 会静默失败（那个路径直接 404）。
    config.max_uri_handlers = 20;
    // httpd 默认栈偏小，WS 指令处理链（cmd::handle → 统一日志转发）加深易触发栈 canary 崩溃
    // （实测收到 spin 时 httpd 栈溢出），调大与 ws_stream 同级避免 WS 指令线程爆栈。
    config.stack_size = 8192;

    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t cmd_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = cmd_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t zoomshot_uri = {
        .uri = "/zoomshot",
        .method = HTTP_GET,
        .handler = zoomshot_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t bmp_uri = {
        .uri = "/bmp",
        .method = HTTP_GET,
        .handler = bmp_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

#ifdef CONFIG_HTTPD_WS_SUPPORT
    // WS 通道挂在端口 81（流服务器）根路径，与 /stream 共存
    httpd_uri_t ws_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
    };
#endif

    httpd_uri_t xclk_uri = {
        .uri = "/xclk",
        .method = HTTP_GET,
        .handler = xclk_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t reg_uri = {
        .uri = "/reg",
        .method = HTTP_GET,
        .handler = reg_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t greg_uri = {
        .uri = "/greg",
        .method = HTTP_GET,
        .handler = greg_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t pll_uri = {
        .uri = "/pll",
        .method = HTTP_GET,
        .handler = pll_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    httpd_uri_t win_uri = {
        .uri = "/resolution",
        .method = HTTP_GET,
        .handler = win_handler,
        .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
        ,
        .is_websocket = true,
        .handle_ws_control_frames = false,
        .supported_subprotocol = NULL
#endif
    };

    // AI 抓帧留档（调试）：清单 + 按序号取图。纯 GET，非 WS。
    httpd_uri_t ai_dump_uri = {
        .uri = "/ai_dump",
        .method = HTTP_GET,
        .handler = ai_dump_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ai_frame_uri = {
        .uri = "/ai_frame",
        .method = HTTP_GET,
        .handler = ai_frame_handler,
        .user_ctx = NULL
    };

    // panic 现场（调试）：纯 GET，取 coredump 分区的摘要，见 coredump_handler 顶部注释。
    httpd_uri_t coredump_uri = {
        .uri = "/coredump",
        .method = HTTP_GET,
        .handler = coredump_handler,
        .user_ctx = NULL
    };

    ra_filter_init(&ra_filter, 20);

    // 统一日志模块转发器：WS 广播 + BLE 通知，供 /log 开启后把板端日志发手机。
    blog::set_forwarder(send_log_to_phone);

    // WS 发送锁：须在任何发送方（blog 转发任务、指令应答、ws_stream_task）启动前建好，
    // 否则那几路发送会在无锁状态下并发写 socket（见 s_ws_tx_mtx 注释）。
    if (!s_ws_tx_mtx) s_ws_tx_mtx = xSemaphoreCreateMutex();

#if CONFIG_ESP_FACE_RECOGNITION_ENABLED
    recognizer.set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");

    // load ids from flash partition
    recognizer.set_ids_from_flash();
#endif
    log_i("Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &zoomshot_uri);
        httpd_register_uri_handler(camera_httpd, &bmp_uri);

        httpd_register_uri_handler(camera_httpd, &xclk_uri);
        httpd_register_uri_handler(camera_httpd, &reg_uri);
        httpd_register_uri_handler(camera_httpd, &greg_uri);
        httpd_register_uri_handler(camera_httpd, &pll_uri);
        httpd_register_uri_handler(camera_httpd, &win_uri);
        httpd_register_uri_handler(camera_httpd, &ai_dump_uri);
        httpd_register_uri_handler(camera_httpd, &ai_frame_uri);
        httpd_register_uri_handler(camera_httpd, &coredump_uri);

        // 固件升级入口（GET 上传页 / POST 固件流），实现在 src/net/ota.cpp
        ota::http_register(camera_httpd);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    log_i("Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK)
    {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
#ifdef CONFIG_HTTPD_WS_SUPPORT
        httpd_register_uri_handler(stream_httpd, &ws_uri);
        // 栈先试 PSRAM（内部堆紧，见 heap_watch）：把 8KB 让回内部 DMA 池；失败退回内部栈保推流可用。
        if (!s_ws_stack) s_ws_stack = (StackType_t*)heap_caps_malloc(8192, MALLOC_CAP_SPIRAM);
        if (s_ws_stack) {
            xTaskCreateStaticPinnedToCore(ws_stream_task, "ws_stream", 8192, NULL, 5, s_ws_stack, &s_ws_tcb, 1);
        } else {
            xTaskCreatePinnedToCore(ws_stream_task, "ws_stream", 8192, NULL, 5, NULL, 1);
        }
#endif
    }
}

void setupLedFlash(int pin)
{
    #if CONFIG_LED_ILLUMINATOR_ENABLED
    s_led_pin = (uint8_t)pin;
    ledcAttach(pin, 5000, 8);  // core 3.x：引脚式绑定（频率 5kHz、8bit 分辨率）
    #else
    log_i("LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
    #endif
}
