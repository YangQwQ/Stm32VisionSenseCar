#include "src/exec/motion_verify.h"
#include "src/cam/camera.h"         // cam::grab/return_frame/available
#include "src/ai/ground_proj.h"     // ground::screen_to_world/ready：单应反投影
#include "src/core/board_log.h"
#include "Calibration.h"            // MVFY_* 阈值与缩放档

#include <esp_heap_caps.h>          // MALLOC_CAP_SPIRAM 软解缓冲
#include <esp_timer.h>              // esp_timer_get_time：与帧时间戳比出"帧龄"
#include <img_converters.h>         // jpg2rgb565：JPEG→RGB565 软解 + 缩放
#include <string.h>                 // memcpy
#include <math.h>                   // atan2f/fabsf
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// mvfy 任务栈：从 PSRAM 分配（外存 8MB 充足）。内部堆在 ai/TLS/网络初始化后很紧，
// 16384 字节栈从内部堆 xTaskCreate 分配极易失败（曾实测"任务创建失败"→ mvfy 全程不采样）。
// mvfy 不做 TLS/网络、无硬实时栈深要求，放 PSRAM 安全；软解缓冲本就分配在 PSRAM。
// 注意：TCB(pxTaskBuffer) 必须放内部 RAM（xTaskCreateStaticPinnedToCore 断言
// xPortCheckValidTCBMem，放 PSRAM 会 assert 崩溃），故用内部静态变量；栈走 PSRAM 手动分配。
static StackType_t* s_task_stack = nullptr;
static StaticTask_t s_task_tcb;   // 内部 RAM（.bss）
static TaskHandle_t s_task_handle = nullptr;

// 运动到位验证（namespace mvfy）实现。
// 线程模型：软解/采样/匹配全部在独立任务（MVFY_TASK_CORE）内跑，exec 调用方与任务之间
// 经 g_mtx 保护的共享状态通信（begin/should_stop/consume/spin_delta）。loop 不碰软解，时序不被阻塞。
namespace mvfy {

static constexpr float PI_F = 3.14159265358979f;

// 缩略图尺寸：源分辨率实际以 fb->width/height 为准。
static constexpr int SCALE = (MVFY_SCALE >= 0 && MVFY_SCALE <= 3) ? MVFY_SCALE : 2;
static int TW = 0, TH = 0;          // 缩略宽高（采样时按源 fb 计算）
static int TN = 0;                  // TW*TH（灰图字节数）

// ---- 共享状态（g_mtx 保护，exec 与任务双方访问） ----
static SemaphoreHandle_t g_mtx = nullptr;    // 保护下方状态
static SemaphoreHandle_t g_start = nullptr;  // 任务睡眠唤醒（begin 置位后发信，end 归零任务自行退出）
static Mode s_mode = IDLE;          // begin/end(core1) 写、任务与 loop(core0/1) 读；单字节类型，volatile 防缓存
static volatile bool s_need_stop = false;    // 有「受阻/转不大」待停机请求；loop 无锁读/consume 无锁清，volatile 防缓存
static int s_still = 0;             // 连续"没动/匹配失败"拍数
static unsigned long s_grace_until = 0; // 起步缓冲截止
static volatile bool s_spin_hold = false; // spin 已停轮（沉降窗口内）：暂停"卡死"判定，保留累计
static volatile bool s_settled = false;   // spin 停轮后已测到"画面静止"=滑行结束、累计角已定（exec 据此提前收窗口）

// ---- 任务线程私有（仅 mvfy_task 访问，无需锁） ----
static uint8_t* s_prev = nullptr;   // 上—缩略灰图（PSRAM，任务线程）
static uint8_t* s_cur  = nullptr;   // 当前缩略灰图（PSRAM，任务线程）
static uint16_t* s_rgb = nullptr;   // 软解中间 RGB565 缓冲（PSRAM，任务线程）
static uint8_t* s_jpg = nullptr;   // 当前帧 JPEG 副本（PSRAM，任务线程）：抓帧后立即拷入并归还缓冲再软解
static size_t s_jpg_cap = 0;       // s_jpg 已分配容量（字节）
static bool s_have_prev = false;    // 是否已有基准帧（任务线程）
static unsigned long s_last_ms = 0; // 上一拍采样时刻（任务线程）：日志里出真实采样间隔用
static int s_frame_age_ms = -1;     // 最近一拍的"帧龄"：该帧曝光 → 本拍取用之间隔了多久（-1=取不到时间戳）
static bool s_frame_stale = false;  // 最近一拍的帧是否已过旧（超 MVFY_FRAME_AGE_MAX → 该拍作废）
static int64_t s_cur_cap_us = 0;    // 当前帧（s_cur）的曝光时刻 µs
static int64_t s_prev_cap_us = 0;   // 基准帧（s_prev）的曝光时刻 µs：两者之差 = 该帧对真实覆盖的时长
static int s_cover_ms = 0;          // 已计入累计的各帧对覆盖时长之和（≈实测角实际覆盖的旋转时长）
static long s_grab_ms = 0;          // 上一拍抓帧（含排空旧帧）耗时
static int  s_grab_n  = 0;          // 上一拍抓了几帧（排空深度：判"队列里积压了多少旧帧"）
static long s_dec_ms = 0;           // 上一拍软解+灰化耗时（判"采样为何慢"：抓帧、软解还是匹配）
static long s_match_ms = 0;         // 上一拍块匹配耗时（全搜索 + 第二优，与软解并列的两个大头之一）
static volatile float s_angle = 0;  // spin：累计实测转角（deg，有符号）；loop 无锁读，volatile 防缓存
static volatile int s_angle_n = 0;  // spin：累计所用的有效样本数（判测量可信度）；同上无锁读
static float s_target = 0;          // spin：计划转角（deg，调试日志对照用，begin 锁内写、任务锁内读）

// ---- 临界区封装 ----
static void lock()   { if (g_mtx) xSemaphoreTake(g_mtx, portMAX_DELAY); }
static void unlock() { if (g_mtx) xSemaphoreGive(g_mtx); }

static bool ensure_bufs(int w, int h) {
  // 尺寸未变且三者都就绪则直接复用（防「部分分配失败」留下半就绪被快速路径误判）。
  if (s_prev && s_cur && s_rgb && TW == w) return true;
  if (s_prev) heap_caps_free(s_prev);
  if (s_cur)  heap_caps_free(s_cur);
  if (s_rgb)  heap_caps_free(s_rgb);
  s_prev = nullptr; s_cur = nullptr; s_rgb = nullptr;
  TW = w; TH = h; TN = w * h;
  s_prev = (uint8_t*)heap_caps_malloc(TN, MALLOC_CAP_SPIRAM);
  s_cur  = (uint8_t*)heap_caps_malloc(TN, MALLOC_CAP_SPIRAM);
  s_rgb  = (uint16_t*)heap_caps_malloc((size_t)TN * 2, MALLOC_CAP_SPIRAM);
  // 任何一者失败：把三者都置 null 并复位尺寸，返回 false；下次采样重新尝试（不留半就绪悬垂态）。
  if (!(s_prev && s_cur && s_rgb)) {
    if (s_prev) { heap_caps_free(s_prev); s_prev = nullptr; }
    if (s_cur)  { heap_caps_free(s_cur);  s_cur = nullptr; }
    if (s_rgb)  { heap_caps_free(s_rgb);  s_rgb = nullptr; }
    TW = TH = TN = 0;
    return false;
  }
  return true;
}

// 该帧的曝光时刻（µs）：相机在每帧起始打的时间戳。
static int64_t frame_cap_us(const camera_fb_t* fb) {
  return (int64_t)fb->timestamp.tv_sec * 1000000LL + (int64_t)fb->timestamp.tv_usec;
}

// 帧龄（ms）：抓帧时相机打的时间戳与当前时刻之差 = 这帧从曝光到本拍手上有"多旧"。
// 判读：正常（一个出帧周期内）说明画面是新的，"匹配到零位移"就是画面真没动；过大说明是积压/停摆的旧帧。
// ⚠️ 只把"负值"（当前时刻早于时间戳 = 时钟对不上）当不可用回 -1；**大值照原样回**——
// 实测见过 109602ms（队列在空闲期冻结、下一轮验证拿到的还是百秒前的帧），那种帧必须能被
// 判成过旧而丢弃；以前在这里封顶 60s 回 -1，恰好把它伪装成"判不了"，于是被放行当基准用了。
static int frame_age_ms(const camera_fb_t* fb) {
  long age = (long)((esp_timer_get_time() - frame_cap_us(fb)) / 1000);
  return (age >= 0) ? (int)age : -1;
}

// 抓一帧并软解成缩略灰图，结果写到全局 s_cur（任务线程内调用）。成功返回 true。
// 抓帧后【立即】把 JPEG 字节拷入 s_jpg 并归还相机缓冲，再在副本上软解——软解耗时几十 ms，
// 若不及时归还会长期占 fb 缓冲池，与图传/AI 并发抓帧触发 cam_hal FB-OVF。grab 占用仅一次 memcpy。
// 取的是"队列里最新的一帧"——本拍画面的时间必须尽量贴近现在，否则测出的转角覆盖不到整段旋转。
// 队列 FIFO（先出最旧），所以反复抓、留最后一帧即可；难在"抓到什么程度才算最新"：
// 空闲期队列会被填满并冻结（帧龄可上百秒），起转后又只按取走的速度催生新帧，
// 于是"设一个新鲜的阈值、够了就停"会被队首那帧恰好卡在阈值内而失效（实测间隔只有 dt 的 1/6）。
// 改为排到**队尾**：相邻两抓的曝光差 = 相机出帧周期，帧龄落进一个周期内就说明它已是队列最新；
// 时间戳不可用时判不了，一路抓到上限（同样取 FIFO 队尾）。至少抓两帧，免得第一帧就误判成"到手了"。
// 顺带把积压缓冲还回相机，这正是抓帧模式 WHEN_EMPTY 下相机恢复供帧的前提。
static bool grab_thumb() {
  unsigned long t0 = millis();
  camera_fb_t* fb = nullptr;
  int age = -1, prev_age = -1, period = 0, tries = 0;
  for (int i = 0; i < MVFY_GRAB_TRIES; i++) {
    camera_fb_t* f = cam::grab();
    if (!f || !f->buf || cam::jpeg_len(f) == 0) { if (f) cam::return_frame(f); break; }
    if (fb) cam::return_frame(fb);   // 换新：队列 FIFO，后拿到的更晚曝光
    fb = f; tries = i + 1;
    age = frame_age_ms(f);
    if (age >= 0 && prev_age > age) period = prev_age - age;  // 相邻两抓的曝光差 ≈ 相机出帧周期
    prev_age = age;
    if (i > 0 && age >= 0) {
      // 停手门槛 = min(实测出帧周期, 兜底值)：周期测出来是几就按几判"已到队尾"，
      // 但封顶在 FRESH——周期被量大了（队列里恰好只差两帧、间距偏大）时也不至于提早收手。
      int bar = (period > 0 && period < MVFY_FRAME_AGE_FRESH) ? period : MVFY_FRAME_AGE_FRESH;
      if (age <= bar) break;
    }
  }
  if (!fb) return false;
  s_grab_ms = (long)(millis() - t0);
  s_grab_n = tries;
  s_cur_cap_us = frame_cap_us(fb);
  s_frame_age_ms = age;
  s_frame_stale = (age >= 0 && age > MVFY_FRAME_AGE_MAX);
  // 源帧宽高必须先取，归还后便无从得知。
  int sw = fb->width, sh = fb->height;
  // 真实 JPEG 字节数(EOI 截断, 见 cam::jpeg_len)：fb->len 是缓冲容量, 会把上一帧残留一起裹进来,
  // 直接喂 jpg2rgb565 会解进垃圾字节 —— 放大镜那种"结构在、颜色毁"坏图的前置条件之一。
  size_t jlen = cam::jpeg_len(fb);
  // JPEG 副本：按需扩容到帧大小（VGA JPEG 通常 <100KB，留余量避免高频内存搬移）。
  // ⚠️ 守卫量与扩容档位必须同源：都基于"帧长+余量"向上取整。曾把帧长单独取整，
  // 结果档位可能仍不满足守卫（帧长落在档位边界下方时 nc 恰等于旧 cap）→ 每拍重分配。
  size_t need = jlen + 4096;
  if (!s_jpg || need > s_jpg_cap) {
    size_t nc = (need + 16383) & ~(size_t)16383;  // 对齐到 16KB，减少重分配
    if (nc < 1024) nc = 1024;
    uint8_t* nb = (uint8_t*)heap_caps_malloc(nc, MALLOC_CAP_SPIRAM);
    if (!nb) { cam::return_frame(fb); return false; }
    if (s_jpg) heap_caps_free(s_jpg);
    s_jpg = nb; s_jpg_cap = nc;
  }
  memcpy(s_jpg, fb->buf, jlen);
  cam::return_frame(fb);   // 立即归还，软解在副本上做，不占 fb

  int ow = sw >> SCALE, oh = sh >> SCALE;
  if (ow < 8 || oh < 8 || !ensure_bufs(ow, oh)) return false;
  unsigned long td = millis();
  // Tjpgd 软解非线程安全（全局静态上下文）：与放大镜的软解并发会互相踩出坏图，
  // 经 cam 共享锁串行化（见 camera.h lock_jpeg_dec）。
  cam::lock_jpeg_dec();
  bool ok = jpg2rgb565(s_jpg, jlen, (uint8_t*)s_rgb, (esp_jpeg_image_scale_t)SCALE);
  cam::unlock_jpeg_dec();
  if (!ok) { s_dec_ms = (long)(millis() - td); return false; }
  const uint8_t* p = (const uint8_t*)s_rgb;
  for (int i = 0; i < TN; i++) s_cur[i] = (uint8_t)((p[2*i] + p[2*i+1]) >> 1);
  s_dec_ms = (long)(millis() - td);   // 软解+灰化：本拍图像侧的固定开销（TN 像素要读 PSRAM 的 RGB565）
  return true;
}

// 整图灰差的平均绝对值（中心 ROI，避开边缘与顶部机械臂静止区）。
static float mean_abs_diff(const uint8_t* a, const uint8_t* b) {
  int x0 = TW / 4, x1 = TW * 3 / 4;
  int y0 = TH / 3, y1 = TH;
  if (x1 <= x0) { x0 = 0; x1 = TW; }
  long sum = 0, cnt = 0;
  for (int y = y0; y < y1; y++) {
    const uint8_t* pa = a + y * TW;
    const uint8_t* pb = b + y * TW;
    for (int x = x0; x < x1; x++) {
      int d = pa[x] - pb[x]; if (d < 0) d = -d;
      sum += d; cnt++;
    }
  }
  return cnt ? (float)sum / (float)cnt : 0.f;
}

// 在 cur 中找与 prev 中心小块最相似的偏移（SSD 全搜索，范围 MVFY_MATCH_SR，再受图边界裁剪）。
// 回传每像素残差 cost 与基准块纹理能量 var（灰度方差），由调用方按用途选门槛：
//   判"没转/卡死"用宽松档（只要有纹理 + 残差不高 + 位移在像素容差内）；
//   算转角用严格档（还要求残差不顶窗边、残差远小于纹理能量），乱匹配喂进累计会算歪角度。
// out_clipped：最佳点落在**可搜索范围**的边界上（真实位移更大、值只是被削顶的下界）。
// ⚠️ 不能拿 |dx|>=MVFY_MATCH_SR 代替它：块在 ROI（TH/3 处）上边留白少于下边，上下限本就不对称，
// 被图边界削顶的那拍同样不可信，用搜索半径判会漏掉（这种拍记进累计会让实测角系统性偏小）。
// out_cost2：基准块邻域之外的最优残差（"第二好"的位置）。与 cost 接近 = 画面里有多处长得一样，
// argmin 落在哪纯属偶然（重复花纹混叠，块小于花纹周期时必现）。纯诊断量，不参与判定。
// 返回 false = 窗内无可搜索位置或没找到（无纹理时提前返回）。
static bool match_shift(const uint8_t* prev, const uint8_t* cur,
                        int* out_dx, int* out_dy, float* out_cost, float* out_var,
                        bool* out_clipped, float* out_cost2 = nullptr) {
  const int PH = MVFY_PATCH_PX, PW = MVFY_PATCH_PX;
  const int sr = MVFY_MATCH_SR;
  int cx = TW / 2, cy = TH / 3;
  int tx = cx - PW / 2, ty = cy - PH / 2;
  *out_dx = 0; *out_dy = 0; *out_cost = 1e9f; *out_var = 0.f; *out_clipped = false;
  if (out_cost2) *out_cost2 = 0.f;
  // "第二好"的排除半径：半个块宽。位移不足半个块宽的位置与基准块大面积重叠，属同一个匹配峰；
  // 要另算一处匹配，必须离得足够远才算数。
  const int excl = PW / 2;
  // 可搜索偏移范围：先按搜索半径，再收窄到"基准块完整落在图内"。
  int xlo = -sr, xhi = sr, ylo = -sr, yhi = sr;
  if (tx + xlo < 0) xlo = -tx;
  if (tx + xhi + PW > TW) xhi = TW - PW - tx;
  if (ty + ylo < 0) ylo = -ty;
  if (ty + yhi + PH > TH) yhi = TH - PH - ty;
  if (xlo > xhi || ylo > yhi) return false;   // 图太小/块超界：无可搜索位置
  // 基准块均值与方差（纹理能量）：纯色地面上任何偏移都"匹配得上"，位移无意义，先算出来备判。
  float mean = 0;
  for (int py = 0; py < PH; py++) {
    const uint8_t* pa = prev + (ty + py) * TW + tx;
    for (int px = 0; px < PW; px++) mean += pa[px];
  }
  mean /= (float)(PH * PW);
  for (int py = 0; py < PH; py++) {
    const uint8_t* pa = prev + (ty + py) * TW + tx;
    for (int px = 0; px < PW; px++) { float d = pa[px] - mean; *out_var += d * d; }
  }
  *out_var /= (float)(PH * PW);
  int best_x = tx, best_y = ty;
  long best = -1, best2 = -1;
  for (int dy = ylo; dy <= yhi; dy++) {
    for (int dx = xlo; dx <= xhi; dx++) {
      int sx = tx + dx, sy = ty + dy;
      long s = 0;
      for (int py = 0; py < PH; py++) {
        const uint8_t* pa = prev + (ty + py) * TW + tx;
        const uint8_t* pb = cur + (sy + py) * TW + sx;
        for (int px = 0; px < PW; px++) { int d = pa[px] - pb[px]; s += d * d; }
      }
      if (best < 0 || s < best) { best = s; best_x = sx; best_y = sy; }
      // 邻域外的最优（诊断用）：只多一次比较，不额外扫像素。
      int ax = dx < 0 ? -dx : dx, ay = dy < 0 ? -dy : dy;
      if ((ax > excl || ay > excl) && (best2 < 0 || s < best2)) best2 = s;
    }
  }
  if (best < 0) return false;
  int bdx = best_x - tx, bdy = best_y - ty;
  *out_dx = bdx; *out_dy = bdy;
  *out_cost = (float)best / (float)(PH * PW);
  if (out_cost2 && best2 >= 0) *out_cost2 = (float)best2 / (float)(PH * PW);
  *out_clipped = (bdx <= xlo || bdx >= xhi || bdy <= ylo || bdy >= yhi);
  return true;
}

static void push_stop(const char* why) {
  lock(); s_need_stop = true; unlock();
  blog::logf(blog::EXEC, "mvfy 判定: %s", why);
}

// ---- 采样 + 判定循环（任务线程） ----
static void sample_tick(Mode m) {
  if (!grab_thumb()) {
    // 取帧/软解失败：低频计数日志，避免每拍刷屏又能定位「一直拿不到帧」的诊断盲区。
    static int s_gfail = 0;
    if ((s_gfail++ & 31) == 0)
      blog::logf(blog::EXEC, "mvfy %s 取帧/软解失败 ×%d（相机无帧或软解未过）",
                 m == VERIFY_SPIN ? "spin" : "move", s_gfail);
    return;
  }
  unsigned long now = millis();
  unsigned long dt = s_last_ms ? (unsigned long)(now - s_last_ms) : 0;  // 与上一拍的真实间隔
  s_last_ms = now;
  bool grace = now < s_grace_until;

  if (s_frame_stale) {
    // 帧过旧：内容对应的是几十秒前的场景，与"现在车转到哪了"无关。旧帧之间画面自然"没动"，
    // 喂进匹配必得"没转"的假结论，还会误置 s_settled 让 exec 提前收下沉降窗口、误清卡死计数。
    // 故本拍只记录、不判不计、**也不滚动基准**（旧帧当基准会污染下一拍；宁可让下一拍跟更早的
    // 基准比，位移偏大时自有"位移顶搜索窗"兜住）。帧为何会旧见 frame_age_ms 的说明。
    blog::logf(blog::EXEC, "mvfy %s 帧过旧 帧龄=%dms → 本拍跳过",
               m == VERIFY_SPIN ? "spin" : "move", s_frame_age_ms);
    return;
  }

  if (!s_have_prev) {                  // 首次采样只存基准
    memcpy(s_prev, s_cur, TN);
    s_prev_cap_us = s_cur_cap_us;      // 基准帧的曝光时刻：与后续帧比出"帧间隔"
    s_cover_ms = 0;                    // 覆盖时长从基准帧起算
    s_have_prev = true;
    return;
  }

  if (m == VERIFY_MOVE) {
    if (!grace) {
      float d = mean_abs_diff(s_prev, s_cur);
      blog::logf(blog::EXEC, "mvfy 画面差 d=%.1f%s", d, (d < MVFY_DIFF_LO) ? " (<阈值,疑似没动)" : "");
      lock(); bool below = d < MVFY_DIFF_LO; if (below) s_still++; else s_still = 0; int st = s_still; unlock();
      if (below && st >= MVFY_BLOCKED_N) push_stop("画面无变化,疑似受阻");
    }
  } else {                             // VERIFY_SPIN
    int dx = 0, dy = 0; float cost = 0, var = 0, cost2 = 0; bool clipped = false;
    unsigned long tm = millis();
    bool found = match_shift(s_prev, s_cur, &dx, &dy, &cost, &var, &clipped, &cost2);
    s_match_ms = (long)(millis() - tm);
    // 本拍配对的两帧（基准/当前）真实曝光间隔：这一拍的角增量覆盖的就是这段时长。
    // 它应约等于 dt（节流 + 抓帧/软解耗时）；明显小于 dt 说明两帧的队列积压程度不一致
    // （基准偏新、当前偏旧），这段旋转没被任何一拍测到 → 累计角系统性偏小。
    long dframe = (s_prev_cap_us > 0 && s_cur_cap_us > s_prev_cap_us)
                    ? (long)((s_cur_cap_us - s_prev_cap_us) / 1000) : -1;
    // 合理性下限：两帧曝光差不可能超过这两拍之间的墙钟间隔（抓帧排空至多多等一个出帧周期）。
    // 越界即时间戳对不上（实测见过 109602ms——空闲期冻结的旧帧，本已由帧龄挡掉，这里再兜一层），
    // 当未知处理，免得脏值污染"覆盖"。
    if (dframe > (long)dt + 200) dframe = -1;
    // 有纹理 + 残差不高于纹理能量，才谈得上"动没动/转多少"：纯色地面、或快转糊成一片时，
    // 任何偏移都"差不多匹配"，argmin 会飘到近零位移上，被误读成"没转"（进而误判卡死提前停轮）。
    bool textured = found && var >= MVFY_PATCH_VAR_LO;
    bool good     = textured && cost <= MVFY_MATCH_COST * var;
    bool still    = good && abs(dx) <= MVFY_SPIN_PX_TOL && abs(dy) <= MVFY_SPIN_PX_TOL;
    if (still) {
      // 画面无位移 = 完全没转：不依赖单应（静止画面/车没动/没接车台时也能判"没转/被卡"）。
      // 用像素容差而非严格 ==0：静止画面下 SSD 匹配会因噪声漂移出 1~2px，严格 ==0 会把静止误判成"在转"。
      // grace 内不数（起转瞬间糊/抖，先别判）；停轮沉降期（s_spin_hold）画面本就不动，也不判。
      if (s_spin_hold) s_settled = true;   // 停轮后画面静止 = 滑行结束，累积角已定，exec 可提前收窗口
      if (!grace && !s_spin_hold) {
        lock(); s_still++; int st = s_still; unlock();
        blog::logf(blog::EXEC, "mvfy 画面无位移(dx=%d dy=%d)", dx, dy);
        if (st >= MVFY_SPIN_N) push_stop("旋转特征不动,疑似卡死");
      }
    } else if (found) {
      // 有位移：清零停滞计数。⚠️ 累计不受 grace 限制——起转那段同样是真实转角，
      // 漏掉它实测角就系统性偏小，exec 会据此"还差一截"多补一段（旋转被拖长且转过头）。
      lock(); s_still = 0; unlock();
      // 算转角用严格档：残差须远小于纹理能量（乱匹配会算歪角度），且位移不能顶到可搜索范围边界
      // （顶边说明真实位移超出可测范围，值只是被削顶的下界）。不满足就当这拍没测到，不计入累计。
      bool trusted = good && !clipped;
      if (trusted && ground::ready()) {
        float cu = (float)(TW / 2 + dx) / (float)TW;
        float cv = (float)(TH / 3 + dy) / (float)TH;
        float pu = (float)(TW / 2) / (float)TW;
        float pv = (float)(TH / 3) / (float)TH;
        float x1, y1, x2, y2;
        if (ground::screen_to_world(pu, pv, &x1, &y1) && ground::screen_to_world(cu, cv, &x2, &y2)) {
          float a1 = atan2f(x1, y1), a2 = atan2f(x2, y2);
          float dth = a1 - a2;
          while (dth >  PI_F) dth -= 2.f * PI_F;
          while (dth < -PI_F) dth += 2.f * PI_F;
          float deg = dth * 180.0f / PI_F;
          s_angle += deg;                // 任务私有累计，无需锁
          s_angle_n++;                   // 有效样本计数（exec 判可信度用）
          if (dframe > 0) s_cover_ms += (int)dframe;   // 累计角实际覆盖的时长（帧间隔之和）
          lock(); float tgt = s_target; unlock();  // 日志用 target 在锁内取值
          blog::logf(blog::EXEC, "mvfy 转角 deg=%+.1f 累计=%.1f n=%d 目标=%.0f",
                     deg, s_angle, s_angle_n, tgt);
        }
      }
    }
    // 每拍原始结果：调参全靠它——单拍位移有多大（判断搜索窗够不够）、这拍被哪条门槛挡掉、
    // 真实采样间隔是多少（单拍位移的估算基准）。定参期置 MVFY_LOG_SAMPLES=1，稳定后置 0。
    if (MVFY_LOG_SAMPLES) {
      const char* why = !found    ? "窗内无可搜索位"
                      : !textured ? "纹理弱"
                      : !good     ? "残差大(乱匹配)"
                      : still     ? "无位移"
                      : clipped   ? "位移顶搜索窗"
                                  : "计入累计";
      // 三条独立证据合看，专门用来定性"旋转期间却测到零位移"：
      //  md     宽 ROI 平均灰差：画面到底变没变（≥ 移动受阻阈值 = 确实变了）；
      //  cost2  基准块邻域外的最优残差：若也"合格"（≤ 同一残差门槛）说明画面里有多处长得一样，
      //         argmin 落在哪纯属偶然——重复花纹混叠（块小于花纹周期时必现）；
      //  帧龄   这帧有多旧：偏大即拿到的是积压/停摆的旧帧，那时"无位移"与车转没转无关。
      // 另几个时长/计数（判"采样为何这么慢/累计为何偏小"）：
      //  dt     本拍与上一拍的墙钟间隔；间隔/覆盖 见各自说明；
      //  抓     抓帧耗时/抓了几帧（排空深度），解 = 软解+灰化，匹配 = 块匹配：
      //         三者是本拍处理耗时的全部，dt 减掉它们（再减采样节流）就是调度/日志/UART 的尾巴。
      //         ⚠️ 每拍转角上限 ≈ 转速 × dt，dt 一大单拍就顶出匹配量程（见"位移顶搜索窗"）。
      float md = mean_abs_diff(s_prev, s_cur);
      bool alias = found && cost2 > 0.f && cost2 <= MVFY_MATCH_COST * var;  // 邻域外还有一个"也合格"的匹配
      const char* hint = "";
      // 帧过旧的拍已在上面整拍跳过，这里再出现"无位移 + 画面已变"就只能是匹配本身的问题（混叠）。
      if (still && md >= MVFY_DIFF_LO)   hint = "⚠画面已变却报零位移,疑重复花纹";
      else if (still && alias)           hint = "⚠匹配不唯一,疑重复花纹";
      blog::logf(blog::EXEC, "mvfy spin拍 dt=%lums 帧龄=%d 间隔=%ld 覆盖=%d 抓=%ld/%d 解=%ld 匹配=%ld "
                             "md=%.1f dx=%d dy=%d cost=%.0f cost2=%.0f var=%.0f 判定=%s%s 累计=%.1f",
                 dt, s_frame_age_ms, dframe, s_cover_ms, s_grab_ms, s_grab_n, s_dec_ms, s_match_ms,
                 md, dx, dy, cost, cost2, var, why, hint, s_angle);
    }
    // found==false / 无纹理：既不计成绩也不算"没转"——快转到超出可测范围时若按"没转"处理，
    // 会误判成卡死而中途停轮。
  }

  // 滚动：当前帧作为下一拍基准（曝光时刻随之滚动，否则下一拍的"间隔"会拿错起点）。
  uint8_t* tmp = s_prev; s_prev = s_cur; s_cur = tmp;
  int64_t tc = s_prev_cap_us; s_prev_cap_us = s_cur_cap_us; s_cur_cap_us = tc;
}

// ---- 任务主体 ----
static void mvfy_task(void* arg) {
  Mode seen = IDLE;                    // 任务已处理过的模式（用于识别切换→重置角度）
  for (;;) {
    lock(); Mode m = s_mode; unlock();
    if (m == IDLE) { seen = IDLE; xSemaphoreTake(g_start, portMAX_DELAY); continue; }
    if (seen != m) {                   // 模式切换：任务侧重置角度与基准（单写者，避免跨线程竞争）
      s_angle = 0; s_angle_n = 0; s_have_prev = false; seen = m;
      blog::logf(blog::EXEC, "mvfy 任务进入 mode=%d", (int)m);
    }

    unsigned long last = 0;
    for (;;) {
      lock(); Mode cur = s_mode; unlock();
      if (cur == IDLE) break;          // 被 end 复位 → 退出本次验证
      if (cur != seen) break;          // 模式变了 → 回到外层重置角度
      unsigned long now = millis();
      if ((long)(now - last) >= (long)MVFY_SAMPLE_MS) {
        last = now;
        sample_tick(cur);
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

bool available() { return cam::available(); }

void init() {
#if MVFY_ENABLED
  // 版本标签：调试时用来确认烧录的固件确实包含本模块改动。
  static const char kMvfyTag[] = "mvfy-v2-debug";
  blog::logf(blog::EXEC, "%s init 调用", kMvfyTag);
  if (g_start) { blog::logf(blog::EXEC, "%s 已初始化幂等返回", kMvfyTag); return; }
  g_mtx = xSemaphoreCreateMutex();
  g_start = xSemaphoreCreateBinary();
  if (!g_mtx || !g_start) {
    blog::logf(blog::EXEC, "%s 信号量创建失败 g_mtx=%d g_start=%d（mtx 或 start 为 0 → 任务无法启动）",
               kMvfyTag, g_mtx != nullptr, g_start != nullptr);
    return;
  }
  s_angle = 0; s_have_prev = false;
  // 任务栈从 PSRAM 分配（内部堆紧，xTaskCreate 动态分配栈易失败）；TCB 用内部静态变量
  // （FreeRTOS 断言 TCB 必须在内部 RAM，不能放 PSRAM）。
  if (!s_task_stack) s_task_stack = (StackType_t*)heap_caps_malloc(MVFY_TASK_STACK, MALLOC_CAP_SPIRAM);
  if (!s_task_stack) {
    blog::logf(blog::EXEC, "%s PSRAM 栈分配失败（PSRAM=%d）", kMvfyTag, psramFound());
    return;
  }
  s_task_handle = xTaskCreateStaticPinnedToCore(
      mvfy_task, "mvfy", MVFY_TASK_STACK, nullptr, 2, s_task_stack, &s_task_tcb, MVFY_TASK_CORE);
  blog::logf(blog::EXEC, "%s 任务创建 %s（PSRAM 栈=%d bytes / 内部TCB）", kMvfyTag,
             s_task_handle ? "成功" : "失败", MVFY_TASK_STACK);
#endif
}

void begin(const char* type, float target_deg, bool keep_grace) {
  if (!available()) { lock(); s_mode = IDLE; unlock(); return; }
  Mode nm = (!strcmp(type, "spin")) ? VERIFY_SPIN : VERIFY_MOVE;
  lock();
  s_spin_hold = false;   // 轮子又在动了：恢复"卡死"判定
  s_settled = false;     // 又动起来了：上一次的"已静止"作废（否则补偿段会立刻以为滑行测完）
  s_still = 0;
  // 已处于同一模式（摇杆重发 / 旋转补偿续测）：
  if (s_mode == nm) {
    // 补偿续测传 0（只表示"接着测"，不代表目标变 0）：保留原目标，日志里"目标"才始终可读。
    if (target_deg > 0) s_target = target_deg;
    // 补偿续测（keep_grace）不重置起步 grace，防止补偿段因 grace 未过而测不到转角；摇杆重发刷新 grace。
    if (!keep_grace) s_grace_until = millis() + (unsigned long)MVFY_START_GRACE;
    unlock();
    return;
  }
  // 模式切换（含 IDLE→X）：只改 mode 与目标/节流，角度由任务在下一拍检测到切换时重置（单写者）。
  s_mode = nm;
  s_need_stop = false;
  s_target = target_deg;
  s_grace_until = millis() + (unsigned long)MVFY_START_GRACE;
  unlock();
  if (g_start) xSemaphoreGive(g_start);
  blog::logf(blog::EXEC, "mvfy begin mode=%d tgt=%.0f", (int)nm, target_deg);
}

// spin 到点停轮：轮子已停、画面本就不动，暂停"卡死"判定；采样与转角累计照旧，
// 供 exec 在沉降窗口内把停轮后的滑行、以及滞后一拍的那段一并测进累计值。
void spin_stop() {
  lock();
  s_spin_hold = true;
  s_settled = false;     // 进入沉降窗口：滑行还没测完，等任务拍到"画面静止"才置位
  s_still = 0;           // 清停滞计数：别把停轮期间的"没动"攒到下一次续测去
  unlock();
}

void end() {
  lock();
  s_mode = IDLE;
  s_need_stop = false;
  s_still = 0;
  s_spin_hold = false;
  s_settled = false;
  s_target = 0;
  unlock();
}

// 高频只读接口：loop 每拍(update_tick)轮询，为避免每拍取互斥锁(关中断)抢占 core1，
// 这些单 32 位变量的读/写在 ESP32-S3(Xtensa) 上天然原子，直接无锁读写即可。
// begin()/end() 这类多字段一致性写仍走锁；s_still 仅任务线程写、consume 无锁清。
bool should_stop() { return s_need_stop; }
void consume() { s_need_stop = false; s_still = 0; }
Mode mode() { return s_mode; }
float spin_delta_deg() { return s_angle; }
int spin_delta_n() { return s_angle_n; }
bool spin_settled() { return s_settled; }

}  // namespace mvfy