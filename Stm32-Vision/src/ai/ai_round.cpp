#include "src/ai/ai_round.h"
#include "src/ai/ai_client.h"   // ai::logf / enqueue_result 等协作入口 + ai::generation
#include "src/ai/ai_result.h"   // AI_EDITED_* + edited_snapshot
#include "src/ai/ai_prompt.h"   // PsaBuf + build_body
#include "src/ai/ai_mem.h"      // 空间记忆/车姿态: mem_reset/mem_feed/mem_find/...
#include "src/ai/ai_http.h"     // http_post/http_last_status/http_stop/extract_content
#include "src/ai/ai_validate.h" // validate_cmd/safe_append
#include "src/ai/ai_nav.h"      // navigate_to/settle_*/NavR
#include "src/ai/ai_dump.h"     // 抓帧留档(调试旁路, 只在 /log ai on 时留存本帧画面)
#include "src/ai/ai_alloc.h"    // g_js_alloc(共享 PSRAM JSON 池)
#include "src/ai/magnify.h"     // 放大镜: 把目标附近裁出来放大重编码(AI 的"凑近看")
#include "src/ai/ground_proj.h" // 屏幕→地面单应(观测解算)
#include "src/net/config.h"
#include "src/net/wifi_net.h"
#include "src/cam/camera.h"
#include "src/exec/direct_exec.h"
#include "src/core/board_log.h"
#include "src/core/heap_watch.h"   // 内部堆水位哨兵(诊断 WiFi 收发哑掉的第一现场)

#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <stdio.h>
#include <stdlib.h>  // malloc/free/strtol/atoi
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 决策频率 / 步数上限
#define AI_INTERVAL_MS 1500
#define AI_MAX_STEPS_PER_GOAL 120
// goal=abort = AI 请求中止并等待用户输入。等待期间**不取帧、不发云端请求**(不耗额度、不烧步数), 只轮询
// 用户插话(→继续执行)与超时; 超时后给 AI 一句"用户未回复"再继续, 免得它无限等下去。
#define AI_WAIT_USER_MS 60000
#define AI_WAIT_USER_POLL_MS 200   // 等待期的轮询步长(只查插话/代际/超时)
#define AI_WAIT_FB_MIN_MS 10000   // wait 反馈节流: 同一动作少于此间隔只回一条
#define AI_MAX_NET_FAIL 4         // 连续"无有效输出"轮数上限: 超过即中止任务并回报(防云端持续无响应时无限空转)
#define AI_HIST_N 20               // 历史环条数(AI 决策 + 插话共用, 满员淘汰最旧)

#define AI_FRAME_RETRY 4          // 单轮抓帧重试次数(推流并发占缓冲时会偶发取不到)

// ---------------- 放大镜(命令 zoom) ----------------
// 放大镜: 把 AI 的"眼睛"凑近(裁块放大回喂), 让它在放大图里跟**看得见的夹爪**比相对位置; 裁框由程序记账 ⇒ 换回全幅是精确算术。
#define AI_ZOOM_QUALITY 80          // 比图传略高(这是用来看细节的), 一帧几十 KB
#define AI_ZOOM_JPG_MAX (128 * 1024) // 放大图单帧上限: 640×480 高清近景高熵 JPEG 可超 48K, 抬到 128K; 超了弃用回全幅
// 放大镜=切高清真拍(request_hires): 切 SVGA 抓一帧, TJpgDec 部分解码只裁中央 (0.25,0.25)-(0.75,0.75),
// 中央 1/4 源 1:1 回喂。⚠️ 只用 SVGA: SXGA 的 reconfigure 重建会卡死驱动。
#define AI_HIRES_FRAME   FRAMESIZE_SVGA   // 高清目标分辨率（800×600）
#define AI_HIRES_OUT_W   400              // 放大输出＝高清源中央 1/4（1:1，同源像素密度）
#define AI_HIRES_OUT_H   300
#define AI_ZOOM_MAX 8.0f            // 倍数上限: 再放也只是像素块, 而视野小到看不见夹爪就失去参照
#define AI_ZOOM_DEF 2.0f            // AI 没给 scale 时的默认倍数(取小些: 放大后要同时看得见目标和夹爪, 开太大爪会被裁出画面就失去参照)
// 同一块画面在车与臂都没动的前提下连要几次后, 强制回全幅并讲死(见"放大镜"段)。
// ⚠️ 判"重复"必须带上"这期间有没有动作": 裁框是全幅归一化坐标, 挪车后同一个框里已是另一片世界。
#define AI_ZOOM_NOOP_MAX 2
// 连续这么多轮没有任何**实际动作**(只有 zoom/wait 这类本地命令)就认定空转, 强制把放大镜退回
// 全幅并点名: 放大的局部画面里找不到画面外的目标, 搜索只能在全幅做。
#define AI_IDLE_ROUNDS 5

// arm pose 的 h ≤ 此值 = 把夹爪下探到方块高度去夹(低位 pose), 落地后臂会挡在镜头前下方遮住目标;
// 抬高/悬停与 fold/release 等一律算"让出视野", 不会遮。仅供 hide_arm 判定用。
#define AI_GRASP_H_CM 2.5f

#define AI_ZOOM_HINT \
  "本帧是放大图(只有画面中心区域), 看不到全幅, 若目标基本被完全遮挡则说明没对准" \

// 用户插话(ai_chat)已发给模型但那一轮没拿到有效响应时的补投提示: 插话在组包时被一次性消费,
// 响应报废则等于没执行, 故在它被真正落实前每轮额外点一次名(正文在历史环里, 这里只提醒认领)。
#define AI_CHAT_UNACK_HINT \
  "用户有一条插话建议你还没落实(那一轮的响应失败了): 见对话历史里最后那条 user 插话, " \
  "本轮请优先按它的思路做, 或明确说明为什么不采纳。"

// 本地巡航(approach / /move to)到位距离。approach 是开环死航(无里程计, 靠姿态累积+定距时长近似),
// 目标坐标又来自单应高报 ⇒ 停得近会**越过爪口把方块压到车底/机械臂下**; 留余量停远些, 宁让 AI 多走几步微调。
#define AI_APPROACH_STOP_CM 20

// ---------------- 文件级状态(仅本模块用) ----------------
// 最近一条下发执行板的是否持续型(sink: stop 兜底判定)。任务起点复位。
static volatile bool g_last_continuous = false;
// 最近一条持续指令的类型(0=无/1=move/2=arm)。兜底 stop 时 Wheels 模式只适用于轮子残留。
static volatile int g_last_cont_type = 0;
static volatile uint64_t g_last_wait_fb_ms = 0;  // wait 反馈节流时间戳

// ---------------- 任务上下文 ----------------
// 跨轮/跨阶段的全部状态。按值建在 round_run_task 的栈上(worker 那 16KB PSRAM 栈帧内); 只放指针/PSRAM 句柄, 不新增任何堆分配。
struct RoundCtx {
  ai::TaskLocal& t;
  explicit RoundCtx(ai::TaskLocal& tt) : t(tt) {}

  bool cam_ok = false;              // 摄像头可用性(任务起点判定, init 后即定)

  // 任务级终止/出口
  bool done = false;
  bool got = false;                 // 本轮 attempt 是否产出了有效决策
  bool sent_done = false;           // 是否已确报过终态 done
  bool interrupted = false;         // 是否因新目标/手动中断退出(此时不发补发 done)
  const char* fail = nullptr;       // ⚠️ 可能指向 err_buf(validate 失败时), 故二者同生命周期
  char err_buf[160];

  // 计数/防线
  unsigned long steps = 0;
  char last_cmd[96] = {0};          // 上一轮"合并指令串"(死循环判据)
  unsigned long last_act_ms = 0;    // 上次真正下执行/微操指令的时刻(ms)
  int net_fail = 0;                 // 连续"无有效输出"轮数
  int stall = 0;
  bool stall_hint = false;
  int idle_rounds = 0;              // 连续"无实际动作"轮数
  bool acted = false;               // 本轮是否有实际动作落地(exec 接受)

  // carry_image 一次性投递 + PSRAM 帧缓存
  bool want_prev = false;           // 上轮 carry_image:"full"
  bool want_prev_zoom = false;      // 上轮 carry_image:"zoom"
  int want_prev_img = -1;           // 上轮 carry_image:"imageN"(-1=无)
  bool want_prev_grasp = false;     // 上轮执行过 grasp/clip → 自动带上"合爪前放大特写"
  bool prev_preset = false;         // 本轮是否已由 carry_image 预取帧进 prev
  uint8_t* cur = nullptr; size_t cur_len = 0;            // 本轮帧 PSRAM 副本
  uint8_t* prev = nullptr; size_t prev_len = 0;          // 上一帧 PSRAM 副本
  uint8_t* grasp_prev = nullptr; size_t grasp_prev_len = 0;  // 夹取前现抓的放大特写
  uint8_t* zoom_jpg = nullptr;      // 放大图缓冲(PSRAM, 首次用到时分配)

  // 本轮帧(round_prepare 产出 → round_attempt 消费)
  const uint8_t* frame = nullptr;
  size_t frame_len = 0;
  bool frame_moving = false;

  // 放大镜状态(裁框一律以**全幅归一化**坐标记账: 这是"换回全幅是算术而非估计"的全部依据)
  bool zoom_on = false;             // 下一帧起是否发放大图
  bool zoom_one_shot = false;       // bool `{"zoom":true}` 单次放大: 发完这一帧自动回全幅
  bool sent_zoomed = false;         // 本轮实际发出去的**是不是**放大图(observe 反算要用)
  float sent_x0 = 0, sent_y0 = 0, sent_x1 = 1, sent_y1 = 1;   // 本轮实际发出的裁框
  float zoom_x0 = 0, zoom_y0 = 0, zoom_x1 = 1, zoom_y1 = 1;   // 待用裁框
  float zoom_scale = AI_ZOOM_DEF;   // 待用倍数(日志/回告用)
  char zoom_warn[256] = {0};        // 放大相关的**一次性**告知(生成失败/被强制回全幅)
  char zoom_repeat[224] = {0};      // 重复请求放大镜的**每轮**告知
  int zoom_noop_n = 0;              // 连续"要了手上已经有的那块"的次数
  char zoom_pose[48] = {0};         // 上次放大时的车位姿+臂姿(判"重复请求"要看这期间动过没有)

  // 提示缓冲(一次性告知)
  bool last_round_hide_target = false;  // 上轮是否执行了"可能把目标遮挡/带出画面"的动作
  char obs_warn[160] = {0};         // 观测未记成的回告
  char task_remind[160] = {0};      // 插话即将被冲掉前的提醒
  char wait_note[160] = {0};        // abort 等待超时的一次性告知

  // 等待/插话
  bool wait_user = false;           // goal=abort 的等待态
  uint64_t wait_until = 0;          // 等待截止(ms, esp_timer)
  bool chat_unacked = false;        // 插话尚未落实(响应报废过)

  // 备注/列表/目标
  char task_note[192] = {0};        // AI 写入的任务笔记, 每轮喂回
  struct TaskItem { char name[48]; bool done; };
  TaskItem s_tasks[8] = {};         // AI 维护的任务列表
  int s_task_n = 0;
  char goal_now[256] = {0};         // 当前任务目标(可被 task_goal 热替换)

  // 历史环(PSRAM 动态分配): AI 决策(assistant) + 插话(user) 共用一条管理, 满员淘汰最旧
  char* hist_text[AI_HIST_N] = {nullptr};
  const char* hist_role[AI_HIST_N] = {nullptr};
  int hist_n = 0;

  // 用户编辑图快照(供整轮任务复用): 按"新→旧"把槽位记入 ed_order, image1=最新
  uint8_t* ed_img[AI_EDITED_SLOTS] = {};
  size_t ed_len[AI_EDITED_SLOTS] = {};
  int ed_order[AI_EDITED_SLOTS] = {};
  int ed_n = 0;

  // ---- 成员函数(与调用者同 TU ⇒ 仍可内联) ----
  // AI 报的像素(它看到的那张图) → 全幅归一化: 上一轮发全幅就直通, 是放大图就按当时记下的裁框反算(裁框由程序记账, 换回全幅是算术、不漂)。
  float to_full_x(float px) const { return sent_zoomed ? sent_x0 + px * (sent_x1 - sent_x0) : px; }
  float to_full_y(float py) const { return sent_zoomed ? sent_y0 + py * (sent_y1 - sent_y0) : py; }
  void set_zoom_box(float fx, float fy, float sc);
  void hist_add(const char* role, const char* text);
};

// 以 (fx,fy) 为中心、取**全幅**的 1/scale 作为新框(scale 为相对全幅的绝对倍数); px/py 用它当前那张图的
// 0~1 坐标(sent_* 反算)。⚠️ 必须绝对, 不能"相对当前视图"累乘(倍数会失控、夹爪被裁出画面即失去参照)。
void RoundCtx::set_zoom_box(float fx, float fy, float sc) {
  if (sc > AI_ZOOM_MAX) sc = AI_ZOOM_MAX;
  if (sc < 1.0f) sc = 1.0f;   // "放大"却把范围放大回去没有意义
  float hx = 0.5f / sc, hy = 0.5f / sc;
  float a = fx - hx, b = fx + hx, c = fy - hy, d = fy + hy;
  if (a < 0) { b -= a; a = 0; }
  if (b > 1) { a -= b - 1; b = 1; if (a < 0) a = 0; }
  if (c < 0) { d -= c; c = 0; }
  if (d > 1) { c -= d - 1; d = 1; if (c < 0) c = 0; }
  zoom_x0 = a; zoom_x1 = b; zoom_y0 = c; zoom_y1 = d;
  zoom_scale = sc;
}

// 拷贝到 PSRAM(供历史条用)
static char* ps_dup(const char* s) {
  size_t n = strlen(s);
  char* p = (char*)heap_caps_malloc(n + 1, MALLOC_CAP_SPIRAM);
  if (p) memcpy(p, s, n + 1);
  return p;
}

// 推入历史环(满则冲掉最旧)
void RoundCtx::hist_add(const char* role, const char* text) {
  char* dup = ps_dup(text);
  if (!dup) return;
  if (hist_n < AI_HIST_N) { hist_role[hist_n] = role; hist_text[hist_n] = dup; hist_n++; }
  else {
    if (hist_role[0] && !strcmp(hist_role[0], "user"))   // 被冲掉的恰是插话: 提醒先 task_goal 更新目标
      snprintf(task_remind, sizeof(task_remind),
               "较早的一条用户插话即将被历史丢弃; 若它表达了新任务/新目标而你还未用 task_goal 更新, 请现在更新。");
    free(hist_text[0]);
    memmove(hist_text, hist_text + 1, (AI_HIST_N - 1) * sizeof(hist_text[0]));
    memmove(hist_role, hist_role + 1, (AI_HIST_N - 1) * sizeof(hist_role[0]));
    hist_text[AI_HIST_N - 1] = dup; hist_role[AI_HIST_N - 1] = role;
  }
}

// 裁剪字符串尾部的残缺 UTF-8 序列: 固定缓冲截断常切在汉字中间, 残留半个字节会让云端判
// "invalid unicode code point" 400。原地修改; 合法 UTF-8 输入不受影响。
static void utf8_clamp_tail(char* buf) {
  size_t n = strlen(buf), e = n, ncont = 0;
  while (e > 0) {
    unsigned char ch = (unsigned char)buf[e - 1];
    if ((ch & 0xC0) == 0x80) { e--; ncont++; continue; }        // 续字节: 继续回退
    int need;
    if (ch < 0x80) break;                                       // ASCII 结尾: 完整
    else if ((ch & 0xE0) == 0xC0) need = 1;
    else if ((ch & 0xF0) == 0xE0) need = 2;
    else if ((ch & 0xF8) == 0xF0) need = 3;
    else break;                                                 // 非法引导字节: 不动
    if (ncont < need) e--;                                      // 续字节不足: 连同引导字节一起删
    break;
  }
  if (e != n) buf[e] = 0;
}

// 剥掉状态行里跟在 "cm" 后的舵机 PWM(如 "前10cm(200)" → "前10cm"): 这些 PWM 只给人工校准机械臂
// 用, 对 AI 是纯噪声且紧挨真实距离易被误读。只认 "cm(" 前缀, 不碰诊断里的坐标括号。原地压缩。
static void strip_pwm_hints(char* s) {
  char* w = s;
  for (const char* p = s; *p; ) {
    if (*p == '(' && p > s && p[-1] == 'm') {
      const char* q = p + 1;
      while (*q >= '0' && *q <= '9') q++;
      if (q > p + 1 && *q == ')') { p = q + 1; continue; }   // 整段 "cm(数字)" 丢弃
    }
    *w++ = *p++;
  }
  *w = 0;
}

// 向提示缓冲追加一段(以 "; " 分隔; 放不下就整段丢弃)。
static void hpush(char* buf, size_t cap, const char* s) {
  size_t l = strlen(buf);
  if (l) { if (l + 2 >= cap) return; buf[l++] = ';'; buf[l++] = ' '; }
  snprintf(buf + l, cap - l, "%s", s);
}

// ---------------- 结果文本构建 ----------------
static String build_feedback(unsigned long id, const JsonDocument& cmd) {
  JsonDocument out(&g_js_alloc);
  out["type"] = "ai_result";
  if (id) out["id"] = (long)id;
  JsonObject p = out["params"].to<JsonObject>();
  const char* reason = cmd["reason"] | "";
  p["reason"] = reason;
  if (cmd["error"]) p["error"] = cmd["error"].as<const char*>();
  if (cmd["done"].is<bool>()) p["done"] = cmd["done"].as<bool>();
  // 内嵌词表指令(供 Godot 显示)
  if (cmd["type"] ) {
    JsonObject inner = p["command"].to<JsonObject>();
    inner["type"] = cmd["type"].as<const char*>();
    // ⚠️ 这里收的是 **const** JsonDocument ⇒ `cmd["params"]` 是 JsonVariantConst, 类型检查必须用
    //    `is<JsonObjectConst>()`。写成 `is<JsonObject>()` 会**静默恒为 false**(ArduinoJson v7:
    //    Converter<JsonObject>::fromJson 收非 const JsonVariant, 在 JsonVariantConst 上落到
    //    "unsupported types" 那个重载, 直接 return false) —— 曾因此把内嵌指令的 params 一直丢掉。
    if (cmd["params"].is<JsonObjectConst>()) inner["params"] = cmd["params"].as<JsonObjectConst>();
    const char* rs = cmd["reason"] | "";
    if (rs[0]) inner["reason"] = rs;
  }
  String s;
  serializeJson(out, s);
  return s;
}

// 兜底 stop(任务终结出口统一解析一次)。stop_mode 由打断方写入: None=手动 move/stop 接管(不补停);
// Wheels=手动 arm(只停轮子); All=其余。仅当存在持续指令残留才补。
static void resolve_stop() {
  if (ai::stop_mode() == (int)ai::StopMode::None) return;
  if (!g_last_continuous) return;
  // Wheels 模式只对轮子持续残留停轮子; 臂持续残留(或未知)必须全停。
  const char* scope = (ai::stop_mode() == (int)ai::StopMode::Wheels && g_last_cont_type == 1) ? "wheels" : "all";
  JsonDocument d; d["scope"] = scope;   // d 即 stop 的 params 对象
  exec::act("stop", d.as<JsonObjectConst>());
  blog::logf(blog::AI, "兜底 stop scope=%s", scope);
  g_last_continuous = false;
}

// ---------------- 单轮驱动出口 ----------------
enum class PrepR  : uint8_t { Ok, SkipRound, ExitTask, Interrupted };
enum class AttR   : uint8_t { Got, Retry, Break };
enum class RoundR : uint8_t { NextRound, ExitTask };

// 落地(分组通道 → 执行 / 元数据 / 反馈)。content 为 AI 原样正文(工具日志要转发它)。
static void round_land(RoundCtx& c, JsonDocument& cmdD, const String& content);

// ---------------- 任务初始化 ----------------
static void round_task_init(RoundCtx& c) {
  g_last_continuous = false;  // 本任务尚未下发过持续指令(防上一任务残留标志误判)
  g_last_cont_type = 0;
  ai::set_busy(true);
  // 新任务 = 新坐标系: 车位置=原点、初始车头=0°, 清空上一任务的空间记忆。
  ai::mem_reset();
  // 水位清零: 本任务的低点从此算起。一轮 AI 就是内部堆被压得最深的时候
  // (TLS 组包/收响应 + 双帧 PSRAM 副本 + 历史环), 低点落到哪正是要量的事。
  hwatch::reset_min();
  // 摄像头可用性在任务起点判定(init 后即定): 不可用则整轮走无画面降级
  c.cam_ok = cam::available();
  ai::logf("[ai] 任务开始 gen=%lu text=%s%s", c.t.generation, c.t.text, c.cam_ok ? "" : "(无摄像头→无画面模式)");
  // 单应状态诊断: 若未就绪, 所有 px/py 观测都会被拒, 直接可看出问题
  if (!ground::ready()) ai::logf("[ai] 警告: 单应未就绪(像素观测将全部被拒绝)");
  else {
    float ex, ey;
    ground::screen_to_world(0.5f, 0.5f, &ex, &ey);
    ai::logf("[ai] 单应OK 中心→(%.0f,%.0f)", ex, ey);
  }
  // 打印实际端点/模型, 便于排查 404/401 等云端拒绝(配错路径是常见原因)
  ai::logf("[ai] 端点=%s 模型=%s key=%s", cfg::ai_url().c_str(), cfg::ai_model().c_str(),
                cfg::ai_key().isEmpty() ? "空" : "已配置");
  // 当前任务目标(独立 user 消息展示; 可被 task_goal 热替换)
  snprintf(c.goal_now, sizeof(c.goal_now), "%s", c.t.text ? c.t.text : "");
  // 用户编辑图快照(供整轮任务复用, 避免中途被覆盖): 暂存里最近 ≤3 张有效图各留一份独立 PSRAM
  // 副本, 并按"新→旧"把槽位记入 ed_order, 供 carry_image:"image1~3" 按需带出(image1=最新)。
  c.ed_n = ai::edited_snapshot(c.ed_img, c.ed_len, c.ed_order, AI_EDITED_SLOTS);
}

// ---------------- 取帧 + 放大镜(本轮画面的产出) ----------------
static PrepR round_prepare(RoundCtx& c) {
  // 中止检查(代际号变化即本任务作废); 兜底停统一在任务出口解析。
  if (c.t.generation != ai::generation()) { blog::logf(blog::AI, "被新目标/手动中断"); c.interrupted = true; return PrepR::Interrupted; }
  if (!net::is_connected()) { c.fail = "WiFi 掉线"; return PrepR::ExitTask; }
  if (cfg::ai_key().isEmpty()) { c.fail = "未配置 AI Key"; return PrepR::ExitTask; }

  // goal=abort 的等待态: 不取帧、不发云端请求, 只等用户插话(→继续执行)或超时(→告知"未回复"再继续)。
  // 用户改发新目标 / 手动接管 / ai_cancel 都会换代际号, 由上面的中断检查与本块内的检查兜住。
  while (c.wait_user) {
    if (c.t.generation != ai::generation()) { c.interrupted = true; break; }
    bool replied = ai::chat_pending();   // 只看不动: 插话留给正常流程消费成 user 消息
    if (replied) { c.wait_user = false; break; }
    if ((uint64_t)(esp_timer_get_time() / 1000) >= c.wait_until) {
      snprintf(c.wait_note, sizeof(c.wait_note),
               "你上轮请求中止并等待用户输入, 已超时(用户未回复); 请继续执行原任务");
      blog::logf(blog::AI, "等待用户输入超时(用户未回复), 告知 AI 继续执行");
      c.wait_user = false;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(AI_WAIT_USER_POLL_MS));
  }
  if (c.interrupted) { blog::logf(blog::AI, "等待用户输入期间被新目标/手动中断"); return PrepR::Interrupted; }

  // 取当前帧: 先等停稳(画面清晰) → 失败重试; 无摄像头则跳过, 走无画面降级
  c.frame_moving = false;
  camera_fb_t* fb = nullptr;
  if (c.cam_ok) {
    ai::settle_wheels(c.t.generation, AI_SETTLE_MAX_MS);
    ai::settle_arm(c.t.generation, AI_SETTLE_MAX_MS);   // 等机械臂(离散定位/持续步进/grasp 抬臂)也到位再出帧
    c.frame_moving = exec::wheels_moving();   // 仍在动: 本帧仅供粗略参考, 下轮提示 AI 别硬信
    for (int fr = 0; fr < AI_FRAME_RETRY && !fb; fr++) {
      fb = cam::grab();
      if (!fb && fr < AI_FRAME_RETRY - 1) vTaskDelay(pdMS_TO_TICKS(80));
    }
    // 取帧失败不再一次即判死整个任务(并发推流占满 fb_count 时属偶发): 计入失败轮次,
    // 与"云端无响应"共用同一条连续失败上限, 退避后重试; 连续多轮取不到才收尾。
    if (!fb) {
      if (++c.net_fail >= AI_MAX_NET_FAIL) {
        c.fail = "持续取帧失败(摄像头/缓冲异常)";
        return PrepR::ExitTask;
      }
      ai::logf("[ai] 取帧失败(第%d次), 稍后重试", c.net_fail);
      vTaskDelay(pdMS_TO_TICKS(300 * c.net_fail));
      return PrepR::SkipRound;   // 轮尾(计数/idle/prev 滚动)一律跳过
    }
  }
  const uint8_t* frame = fb ? fb->buf : nullptr;
  // 必须用 cam::jpeg_len 而非 fb->len: 驱动会把 len 报大(缓冲里可能拼了多帧), 虚高的 len 会被原样
  // base64 进请求体、压垮 WiFi 发送路径(详见 camera.cpp 的 jpeg_len 说明)。
  size_t frame_len = cam::jpeg_len(fb);
  // 源帧分辨率必须**在归还相机缓冲之前**取: fb 归还后其字段即失效(见下)。
  int src_w = 0, src_h = 0;
  if (fb) { src_w = fb->width; src_h = fb->height; }

  // 抓帧后立即拷入 PSRAM 副本并归还相机缓冲: AI 的 HTTPS 慢则数秒, 期间一直占着 fb 会把
  // fb_count=2 的缓冲池耗尽、饿死并行推流; 帧数据后续一律用这份副本。
  if (fb && frame_len > 0 && frame_len <= AI_EDITED_IMG_MAX) {
    if (!c.cur) c.cur = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (c.cur) { memcpy(c.cur, frame, frame_len); c.cur_len = frame_len; }
    else c.cur_len = 0;
  }
  cam::return_frame(fb);
  // 副本就绪才认本帧(fb 已归还, 直接指向裸 fb->buf 即为悬垂): 过大或拷入失败则按无画面处理。
  if (c.cur_len > 0) { frame = c.cur; frame_len = c.cur_len; }
  else { frame = nullptr; frame_len = 0; }

  // ---- 放大镜: 把 AI 上一轮点名的那块裁出来放大, 本轮就发这张 ----
  // 必须裁 **cur 这份字节**(AI 收到的就是它, 只是裁小): 另抓一帧会与它决策依据的那帧对不上账。
  c.sent_zoomed = false;
  // 本帧若发全幅, sent_* 必须跟着复位: sent_* 的语义 = AI 此刻收到的画面框, 不清则 AI 再要
  // 同一块会被判"无变化"、再也拿不到想要的近景。
  c.sent_x0 = 0; c.sent_y0 = 0; c.sent_x1 = 1; c.sent_y1 = 1;
  if (c.zoom_on && frame && frame_len > 0 && src_w > 0 && src_h > 0) {
    // 放大镜 = 切高清真拍再裁中央框：request_hires 独占相机切到 SVGA，抓一帧高清后由调用方裁(1:1 不缩放)。
    int hok = 0;
    camera_fb_t* hfb = cam::request_hires(AI_HIRES_FRAME, &hok);
    if (!c.zoom_jpg) c.zoom_jpg = (uint8_t*)heap_caps_malloc(AI_ZOOM_JPG_MAX, MALLOC_CAP_SPIRAM);
    size_t zl = 0;
    // 用 TJpgDec 部分解码裁中央的 crop_center_jpg（固定 (0.25,0.25)-(0.75,0.75)）：
    // workbuf/RGB 全走 PSRAM，不碰内部堆/DMA —— 内部堆被软解吃穿会导致整板卡死。
    bool ok_zoom = hfb && hok && c.zoom_jpg &&
                   magnify::crop_center_jpg(hfb->buf, cam::jpeg_len(hfb),
                                            hfb->width, hfb->height,
                                            c.zoom_jpg, AI_ZOOM_JPG_MAX, &zl,
                                            AI_HIRES_OUT_W, AI_HIRES_OUT_H, AI_ZOOM_QUALITY) && zl > 0;
    if (hfb) cam::return_frame(hfb);   // 高清帧用完归还（request_hires 内部已切回 VGA 放锁）
    if (ok_zoom) {
      frame = c.zoom_jpg; frame_len = zl;
      c.sent_zoomed = true;
      c.sent_x0 = 0.25f; c.sent_x1 = 0.75f; c.sent_y0 = 0.25f; c.sent_y1 = 0.75f;
      ai::logf("[放大镜] 高清(源%dx%d) 中央框(0.25,0.25)-(0.75,0.75) → %uKB %dms",
               hfb->width, hfb->height, (unsigned)(zl / 1024), magnify::last_cost_ms());
    } else {
      // 切高清/裁图失败不能让本轮瞎: 退回全幅(不会走高清), 并把这事说给 AI —— 它以为在看放大图。
      ai::logf("[放大镜] 高清放大失败(%s 源%dx%d), 本轮回全幅",
               hfb ? "裁图失败" : (hok ? "取帧失败" : "切分辨率失败"),
               hfb ? hfb->width : src_w, hfb ? hfb->height : src_h);
      snprintf(c.zoom_warn, sizeof(c.zoom_warn), "放大图生成失败, 本帧仍是全幅(按全幅读)");
    }
  }
  // 单次放大(`{"zoom":true}`)仅喷出一张放大图: 无论本轮放大成功(发出了)还是失败(退回全幅),
  // 消费完这次请求就自动回全幅 —— AI 不必再发 false 关闭。
  if (c.zoom_one_shot) { c.zoom_one_shot = false; c.zoom_on = false; }

  c.frame = frame;
  c.frame_len = frame_len;
  return PrepR::Ok;
}

// ---------------- 组包 + 云端 + 校验 ----------------
static AttR round_attempt(RoundCtx& c, int attempt) {
  // 引导语: 重试纠正 / 死循环打断(连续多轮相同指令, 带重复指令名便于 AI 自我纠正)
  char hint_buf[320];
  const char* hint;
  if (attempt) {
    // "没给正式回答(content 为空, 只有思考)"与"输出非法 JSON"是两种病, 提示要分开:
    // 前者再说"只输出合法 JSON"没用(它根本没输出), 得让它别再长篇思考、直接给指令。
    hint = (c.fail && !strcmp(c.fail, "AI 响应无内容"))
             ? "上轮你只给了思考过程、没有给出正式回答(content 为空); 本轮请勿再长篇思考, 直接输出一个合法 JSON 词表指令。"
             : "上次输出非法, 请只输出合法 JSON 词表指令。";
  } else if (c.stall_hint) {
    // 长度须留够 last_cmd 的**声明**宽度(96B, 编译器按声明判截断, 不看实际内容):
    // 22(前缀) + 96 + 122(后缀) ≈ 240, 故缓冲从 192 提到 320。
    snprintf(hint_buf, sizeof(hint_buf),
             "连续多轮重复「%s」无进展: 换个观察角度重认目标(抬/落机械臂、后退或环视), "
             "若已夹紧或达不成就交 done。", c.last_cmd);
    hint = hint_buf;
  } else {
    hint = "";
  }
  PsaBuf body;
  char st[256];  // exec 状态缓冲(含撞边界/位姿没到位诊断, 需足量避免截断)
  const char* stp = exec::read_state(st, sizeof(st)) ? st : "";  // 本地直驱状态(无执行板, 状态本地合成)
  if (stp == st) strip_pwm_hints(st);   // 剥掉给人工校准用的舵机 PWM, 只留 AI 能用的 cm 值
  // 每轮取一次插话(一次性消费, 读完清空): 推入历史环作 user 消息, 随环一起保留/冲掉。
  char chat_now[256] = {0};
  if (ai::chat_take(chat_now, sizeof(chat_now))) utf8_clamp_tail(chat_now);
  if (chat_now[0]) {   // 插话进入共享历史(用户话语), 落实前一直点名
    c.hist_add("user", chat_now); c.chat_unacked = true;
    ai::logf("[ai] 插话已喂给 AI: %.96s", chat_now);   // 落一条账: "AI 以为用户说了什么"必须可回溯
  }
  // 上一帧是否带上: 由 AI 上轮 carry_image 决定 —— "full"=带全幅(锁定/追踪/运动对比),
  // "zoom"=带放大帧, grasp 后自动带"合爪前放大特写"(见声明)。
  bool use_prev = c.prev_len > 0 && (c.want_prev || c.want_prev_zoom || c.want_prev_grasp);
  c.want_prev = false; c.want_prev_zoom = false; c.want_prev_grasp = false;  // 一次性消费: 只影响本轮
  // 用户编辑图带出(与 prev 通道互斥, 见 build_body 图预算): AI 上轮 carry_image:"imageN"
  // 指定一张则本轮带那张; 否则首轮强制带最新一张(用户刚插入的, 即"替换附带图片")。
  bool use_edited_now = false;
  const uint8_t* ed_show = nullptr; size_t ed_show_len = 0;
  if (c.want_prev_img >= 0 && c.want_prev_img < AI_EDITED_SLOTS && c.ed_len[c.want_prev_img] > 0) {
    use_edited_now = true; ed_show = c.ed_img[c.want_prev_img]; ed_show_len = c.ed_len[c.want_prev_img];
  } else if (c.steps == 0 && c.t.use_image && c.ed_n > 0) {
    use_edited_now = true; ed_show = c.ed_img[c.ed_order[0]]; ed_show_len = c.ed_len[c.ed_order[0]];
  }
  c.want_prev_img = -1;   // 一次性消费
  // 抓帧留档(调试旁路, 见 ai_dump.h): 把**本轮原样发出去的那帧**留一份供事后复盘"它看见了什么",
  // 关着时零开销。放在这里: 此刻才算得出 use_prev, 标注才说得清它看到几张图。
  ai::dump_push(c.frame, c.frame_len, use_prev);
  if (c.sent_zoomed) {   // 标注这帧是放大图(裁框/倍数), 否则复盘时容易把局部图当全幅读
    char zb[72];
    snprintf(zb, sizeof(zb), "[放大镜%.1f×] 全幅(%.2f,%.2f)-(%.2f,%.2f)",
             c.zoom_scale, c.sent_x0, c.sent_y0, c.sent_x1, c.sent_y1);
    ai::dump_note(zb);
  }
  // 渲染任务列表喂回: 任务列表: 1.出门[完成] 2.右转[未完成] ...
  char task_s[320] = {0};
  if (c.s_task_n > 0) {
    int tp2 = snprintf(task_s, sizeof(task_s), "任务列表: ");
    for (int ti = 0; ti < c.s_task_n && tp2 < (int)sizeof(task_s) - 48; ti++)
      tp2 += snprintf(task_s + tp2, sizeof(task_s) - tp2, "%d.%s[%s] ",
                      ti + 1, c.s_tasks[ti].name, c.s_tasks[ti].done ? "完成" : "未完成");
  }
  // 合并本轮提示(在插话 hist_add 之后构建, 确保其触发的 task_remind 本轮可见)。按"本轮最相关"到
  // "兜底引导"排序, 因为缓冲区满时会截断尾部; 只引导、不给判据。
  char no_tgt_warn[224] = {0};
  if (!ai::mem_have_any()) snprintf(no_tgt_warn, sizeof(no_tgt_warn), "记忆里还没有可用目标");
  // 记忆里**有**这个物体、但最近几轮没再看到(见 ai_mem::mem_have_lost) → 与"记忆空"不同:
  // 物体大概率还在老地方, 只是被挡在镜头外。仅在上轮做了可能遮挡的动作时推, 与 no_tgt_warn 互斥。
  char lost_tgt_warn[224] = {0};
  if (ai::mem_have_lost() && c.last_round_hide_target)
    snprintf(lost_tgt_warn, sizeof(lost_tgt_warn), "注意, 目标物体可能被机械臂遮挡");
  char hint_cb[640] = {0};
  // 放大镜提示排在最前: 它是本轮画面的性质, 读错整帧就白看了; 只给事实不给道理("该怎么用"在
  // 系统提示词里)。重复要同一块与下面**并列**而非互斥: 空操作时 sent_zoomed 必为 true, 写 else 就推不出去。
  if (c.zoom_repeat[0]) hpush(hint_cb, sizeof(hint_cb), c.zoom_repeat);
  if (c.sent_zoomed) hpush(hint_cb, sizeof(hint_cb), AI_ZOOM_HINT);
  else if (c.zoom_warn[0]) { hpush(hint_cb, sizeof(hint_cb), c.zoom_warn); c.zoom_warn[0] = 0; }
  // 用户一次发了多张图(首轮只强制带最新一张): 明说张数与查看入口, 否则 AI 不知道其余图存在
  if (c.steps == 0 && use_edited_now && c.ed_n > 1) {
    char mimg[160];
    snprintf(mimg, sizeof(mimg),
             "用户共发送了 %d 张图片, 本轮仅显示最新一张; 其余可用 carry_image:\"image%d\"~\"image%d\" 查看",
             c.ed_n, 2, c.ed_n);
    hpush(hint_cb, sizeof(hint_cb), mimg);
  }
  if (c.frame_moving)
    hpush(hint_cb, sizeof(hint_cb), "本帧是车/臂仍在移动时拍摄的, 画面可能模糊位移, 方位判断不可靠; 宜先 wait 待停稳再据画面决策");
  if (c.obs_warn[0]) { hpush(hint_cb, sizeof(hint_cb), c.obs_warn); c.obs_warn[0] = 0; }
  // "还没锁定任何目标"排在近场那几条之后: 它与它们互斥(那几条都要有观测依据才成立, 记忆空着时
  // 必然一条都推不出来), 排后面只是为了让"本轮硬事实"优先占位。排在插话提醒之前。
  if (no_tgt_warn[0]) hpush(hint_cb, sizeof(hint_cb), no_tgt_warn);
  // 目标刚变得不可见(记忆里有它、但最近没看到)+ 上轮遮挡动作 → 提示"可能被挡而非消失", 排在
  // 插话提醒之前, 与 no_tgt_warn 互斥(记忆非空才会走到)。
  if (lost_tgt_warn[0]) hpush(hint_cb, sizeof(hint_cb), lost_tgt_warn);
  if (c.chat_unacked) hpush(hint_cb, sizeof(hint_cb), AI_CHAT_UNACK_HINT);   // 用户插话尚未落实
  if (c.task_remind[0]) { hpush(hint_cb, sizeof(hint_cb), c.task_remind); c.task_remind[0] = 0; }
  if (c.wait_note[0]) { hpush(hint_cb, sizeof(hint_cb), c.wait_note); c.wait_note[0] = 0; }   // abort 等待超时(一次性)
  if (hint && hint[0]) hpush(hint_cb, sizeof(hint_cb), hint);
  // 历史环逐条喂给 build_body(assistant=AI决策 / user=插话, 真多轮对话)
  BodyReq br;
  br.goal = c.goal_now;
  br.ann = c.t.ann;
  br.hint = hint_cb;
  br.hrole = (const char* const*)c.hist_role;
  br.htext = (const char* const*)c.hist_text;
  br.hn = c.hist_n;
  br.exec_state = stp;
  br.last_age_s = c.last_act_ms ? (unsigned)((esp_timer_get_time() / 1000 - c.last_act_ms) / 1000) : 0u;
  br.note = c.task_note;
  br.prog = task_s;
  br.frame = { c.frame, c.frame_len };
  br.prev = { c.prev, c.prev_len };
  br.use_prev = use_prev;
  br.edited = { ed_show, ed_show_len };
  br.use_edited = use_edited_now;
  build_body(body, br);
  if (!body.ok) { c.fail = "组装请求 body 失败"; return AttR::Break; }

  String resp;
  bool http_ok = false;
  for (int nr = 0; nr < 3 && !http_ok; nr++) {   // 网络失败指数退避重试(任务串行, 代价可控)
    if (ai::http_post(cfg::ai_url().c_str(), cfg::ai_key().c_str(), body.p, resp, c.t.generation)) { http_ok = true; break; }
    if (c.t.generation != ai::generation()) { c.done = true; c.interrupted = true; break; }  // 被中止, 静默作废
    if (ai::http_last_status() >= 400 && ai::http_last_status() < 500) {  // 4xx 重发同 body 必然再拒, 快速失败
      c.fail = "云端拒绝(4xx), 疑似参数或限流";
      break;
    }
    if (nr < 2) { vTaskDelay(pdMS_TO_TICKS(500 << nr)); blog::logf(blog::AI, "网络失败重试 %d", nr + 1); }
  }
  if (c.done) return AttR::Break;
  if (!http_ok) { if (!c.fail) c.fail = "AI 请求失败"; return AttR::Break; }
  if (c.t.generation != ai::generation()) { c.done = true; c.interrupted = true; return AttR::Break; }  // 在途结果作废

  String content;
  bool body_broken = false;
  if (!ai::extract_content(resp, content, &body_broken)) {
    // 解码成功但无有效内容(瞬态错误体/空 content 等): 打印原始片段便于定位
    blog::logf(blog::AI, "响应无内容, 原始(前120B): %s", resp.substring(0, 120).c_str());
    c.fail = "AI 响应无内容";
    if (body_broken) ai::http_stop();  // 传输层截断/残留: 弃用复用连接, 下次全新握手防污染
    return AttR::Retry;  // 空内容→重试, 不终止
  }

  JsonDocument cmdD(&g_js_alloc);  // PSRAM 池: 避免每轮在内部堆分配/释放制造碎片(freeHeap 泄漏嫌疑)
  const char* verr = ai::validate_cmd(content.c_str(), cmdD, c.err_buf, sizeof(c.err_buf));
  if (!verr) {
    round_land(c, cmdD, content);
    return AttR::Got;   // 落地成功路径一律置 got(或 done)
  }
  blog::logf(blog::AI, "校验失败(%s), AI 原样返回: %.240s", verr, content.c_str());
  // 校验失败常因 AI 输出非纯 JSON, 只看 err_buf 前80B 看不出它写了什么: 把原样 content 全文也
  // 转发出来(任意长), 复盘"什么导致被拒"才有东西可看; 仅在校验失败时打, 不刷屏。
  blog::forward_text(blog::AI, content.c_str());
  c.fail = verr;  // 重试一次前暂存
  return AttR::Retry;
}

// ---- carry_image: 决定"下一轮额外携带哪张图"并就地预取进 prev ----
// ⚠️ 位置契约: 必须在**本轮任何动作落地之前**调用。AI 要的是"动作之前"的画面做前后对比; 排在动作后面
//    就会取到还在移动的帧(approach 尤其致命 —— 它自己就挪车, 且 navigate_to 到点只发一条 stop、不等
//    车身停稳), AI 拿一张糊帧等于白费一轮。
// 与本轮 zoom 通道**无先后要求**: zoom 分支只认 round_prepare 已定案的 sent_zoomed/frame(本轮发的
//    就是放大帧就直接复用), 否则另抓一张高清重新裁中心框 —— 两条都不读 round_land 里 zoom 通道设的框。
static void land_carry_image(RoundCtx& c, JsonDocument& cmdD) {
  const char* cimg = cmdD["carry_image"] | "";
  c.want_prev = false; c.want_prev_zoom = false; c.want_prev_img = -1;
  c.prev_preset = false;   // 本轮尚未预取(若本轮声明了 carry zoom/full 下面会置位)
  if (!strcmp(cimg, "zoom")) {
    c.want_prev_zoom = true;
    if (!c.prev) c.prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (c.prev) {
      size_t pl = 0;
      if (c.sent_zoomed && c.frame && c.frame_len > 0 && c.frame_len <= AI_EDITED_IMG_MAX) {
        memcpy(c.prev, c.frame, c.frame_len); pl = c.frame_len;   // 本轮发的就是放大帧, 直接复用
      } else {
        int hok = 0;
        camera_fb_t* hfb = cam::request_hires(AI_HIRES_FRAME, &hok);
        bool ok_g = hfb && hok &&
                    magnify::crop_center_jpg(hfb->buf, cam::jpeg_len(hfb),
                                             hfb->width, hfb->height,
                                             c.prev, AI_EDITED_IMG_MAX, &pl,
                                             AI_HIRES_OUT_W, AI_HIRES_OUT_H, AI_ZOOM_QUALITY) && pl > 0;
        if (hfb) cam::return_frame(hfb);
        if (!ok_g) pl = 0;
      }
      c.prev_len = pl;
      if (c.prev_len > 0) c.prev_preset = true;
      ai::logf("[ai] carry zoom 预取放大帧: %s(%uKB)", c.prev_len ? "已取帧" : "取帧失败", (unsigned)(c.prev_len / 1024));
    } else { c.prev_len = 0; }
  } else if (!strncmp(cimg, "image", 5) || !strncmp(cimg, "Image", 5)) {
    int n = atoi(cimg + 5);
    if (n >= 1 && n <= AI_EDITED_SLOTS && n <= c.ed_n && c.ed_order[n - 1] >= 0) c.want_prev_img = c.ed_order[n - 1];
  } else if (!strcmp(cimg, "full")) {
    c.want_prev = true;
    // 重新取一帧全幅, **不复用 c.cur**: "full" 的语义是"下一轮要一张全幅", 不能取决于 c.cur 此刻
    // 恰好装的是什么 —— 哪天有别的路径改成往 c.cur 里写放大图, 复用就会静默把放大图当全幅回给 AI。
    // 走 cam::grab()(与普通轮同源同分辨率: 前后对比的两张必须可比), 不用 request_hires 那条高清路。
    // 车已在 round_prepare 停稳、本轮动作尚未落地 ⇒ 取到的正是"本轮动作之前"的画面, 即对比基准。
    if (!c.prev) c.prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    size_t pl = 0;
    if (c.prev && c.cam_ok) {
      camera_fb_t* pfb = cam::grab();
      if (pfb) {
        size_t fl = cam::jpeg_len(pfb);
        if (fl > 0 && fl <= AI_EDITED_IMG_MAX) { memcpy(c.prev, pfb->buf, fl); pl = fl; }
        cam::return_frame(pfb);   // 立即归还: 后面是慢 HTTPS, 占着会把 fb_count=2 的池子耗尽饿死推流
      }
    }
    c.prev_len = pl;
    if (c.prev_len > 0) c.prev_preset = true;
    ai::logf("[ai] carry full 重新取帧: %s(%uKB)",
             c.prev_len ? "已取帧" : "取帧失败", (unsigned)(c.prev_len / 1024));
  }
}

// ---------------- 落地(分组通道 → 执行 / 元数据 / 反馈) ----------------
static void round_land(RoundCtx& c, JsonDocument& cmdD, const String& content) {
  // ---- 分组通道提取(新协议): move/arm/light/zoom 可选, 缺席=该子系统不动 ----
  // move 内部单选 throttle/spin/approach; arm 内部单选各固定动作; light/zoom 可任意组合。
  JsonObjectConst mv = cmdD["move"].is<JsonObject>() ? cmdD["move"].as<JsonObjectConst>() : JsonObjectConst();
  JsonObjectConst ar = cmdD["arm"].is<JsonObject>()  ? cmdD["arm"].as<JsonObjectConst>()  : JsonObjectConst();
  bool has_move  = cmdD["move"].is<JsonObject>();
  bool has_arm   = cmdD["arm"].is<JsonObject>();
  bool has_light = cmdD["light"].is<JsonObject>();
  bool has_zoom = cmdD["zoom"].is<bool>();   // `{"zoom":true}`(单次放大) / `{"zoom":false}`(回全幅)
  bool has_any   = has_move || has_arm || has_light || has_zoom;
  const char* mtype = mv["type"] | "";
  bool is_mv = has_move && !strcmp(mtype, "throttle");
  bool is_sp = has_move && !strcmp(mtype, "spin");
  bool is_ap = has_move && !strcmp(mtype, "approach");
  const char* act = ar["type"] | "";   // arm 子型(low/raise/fold/grasp/clip/release/pose), 无 arm 为空
  // 重置"上轮是否可能遮挡目标"标记: 本轮的提示已用上一轮的值构建完, 这里起重新累计供下一轮。
  // 只有**真正的遮挡源**(收臂进视野/后退)会在下方重标为 true; spin/前进是主动去找、zoom/wait 空动作轮仍为 false。
  c.last_round_hide_target = false;
  // 空间记忆: 先解析 AI observe, 且必须用**动作前**的车位姿把观测转全局 —— 位置在所有动作之前:
  // 本轮动作落不落地都要收下这一帧观测(观测是事实), 而坐标换算基准是动作前的车姿态。
  if (cmdD["observe"].is<JsonObject>()) {
    JsonObjectConst ob = cmdD["observe"].as<JsonObjectConst>();
    const char* nm = ob["name"] | "";
    // 只要 AI 提交 observe(写明了名字+位置)就说明它这一帧看到了该物体、要记录 → 一律视为可见;
    // 它不对某物 observe, 该物就不会被刷新(新旧由 mem 的过期轮数体现)。
    bool vis = true;
    if (!nm[0]) {
      // name 必填: 无名观测无法归属到任何物体, 只能丢弃。明确回告(否则 AI 以为已记住)
      if (vis) snprintf(c.obs_warn, sizeof(c.obs_warn),
                        "上轮 observe 没给 name, 该观测已被忽略(记忆里没有它); 每次 observe 都要带 name");
    } else {
      // 优先用屏幕像素 px/py(单应解算, 精度高); 越界/解算失败或没报像素则回退 rel_deg/dist。
      float px = ob["px"] | 0.0f, py = ob["py"] | 0.0f;
      // 本轮 AI 看的是放大图时, 它的 px/py 是相对那块裁框的: 先换回全幅再交给单应。
      // 这一步是精确算术(裁框是程序记账) ⇒ AI 只需在局部指认目标, 不必在全幅估绝对位置。
      if (c.sent_zoomed) {
        float lx = px, ly = py;
        px = c.to_full_x(px); py = c.to_full_y(py);
        if (lx >= 0.0f && lx <= 1.0f && ly >= 0.0f && ly <= 1.0f)
          ai::logf("[放大镜] 「%s」局部(%.2f,%.2f) → 全幅(%.2f,%.2f)", nm, lx, ly, px, py);
      }
      bool has_px = ob["px"].is<float>() || ob["px"].is<int>();
      bool has_py = ob["py"].is<float>() || ob["py"].is<int>();
      bool rec;
      if (has_px && has_py) {
        rec = ai::mem_observe_xy(nm, vis, px, py);
        if (!rec) {  // 像素越界: 若另给了角度+距离, 退回粗测(距离为 0 则同样记不成)
          ai::logf("[ai] 观测「%s」像素(%.2f,%.2f)不可用 → 退回模型自估 rel=%.0f° dist=%.0fcm",
                   nm, px, py, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
          rec = ai::mem_observe(nm, vis, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
        }
      } else {
        // 没给像素 = 位置只能记模型自估的厘米/角度。模型没有测距能力, 自估偏小, 照它对准会把车
        // 往错的方向指挥; 记还是照记(approach/goto 用粗值够用), 但要把话说清并要回像素指认。
        ai::logf("[ai] 观测「%s」未给 px/py → 只能记模型自估 rel=%.0f° dist=%.0fcm(非实测)",
                 nm, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
        if (vis && !c.obs_warn[0])
          snprintf(c.obs_warn, sizeof(c.obs_warn),
                   "上轮 observe 没给 px/py: 位置只能按你自报的厘米记(自估不准); 请用 px/py 指认画面里的目标再对准");
        rec = ai::mem_observe(nm, vis, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
      }
      if (vis && !rec)   // 可见却没记成: 回告, 否则 AI 以为已锁定而实际记忆为空
        snprintf(c.obs_warn, sizeof(c.obs_warn),
                 "上轮 observe 的「%s」没记进记忆(px/py 越界且缺可用距离), 位置仍未知; 请据当前画面重新确认后再 observe",
                 nm);
    }
  }
  // ---- 主动作/并行辅助标记(供末尾统一落地与反馈); acted 用 RoundCtx, 勿在此重复遮蔽 ----
  bool nav_done = false;                 // approach 已执行导航
  const char* nav_why = nullptr;         // approach 结果说明(供主动作反馈/历史)
  const char* nav_tgt = mv["target"] | "";
  // ---- 带图请求: ⚠️ 提到所有动作之前(approach 会挪车) ⇒ 预取到的必是"本轮动作前"的画面 ----
  land_carry_image(c, cmdD);
  // ---- approach(轮子自动靠近): 本地巡航, 不点云端。与合爪不同轮(导航后车动、坐标过时) ----
  if (is_ap) {
    // ⚠️ 只拦 approach+**合爪**(grasp/clip): approach 会挪车、坐标过时, 拿旧坐标合爪必空夹。
    //   approach+**降爪**(arm low)不拦 —— 降爪落到标定固定点, 坐标过时只影响之后的横向微调。
    if (has_arm && (!strcmp(act, "grasp") || !strcmp(act, "clip"))) {
      JsonDocument cmdF(&g_js_alloc);
      cmdF["type"] = "approach";
      cmdF["reason"] = "approach 与合爪不能同轮: 先 approach, 下轮据画面观察确定是否对齐后再决定夹取; arm low/fold/raise/light 可与 approach 并行";
      cmdF["params"]["target"] = nav_tgt[0] ? nav_tgt : "(最近)";
      String fb = build_feedback(c.t.id, cmdF);
      ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
      {
        const char* r = cmdD["reason"] | "";
        PsaBuf ld;
        ld.put("approach 被拦(与合爪同轮), 原因: "); ld.put(r);
        utf8_clamp_tail(ld.p);
        if (ld.len) c.hist_add("assistant", ld.p);
      }
      c.got = true; c.net_fail = 0;
      return;
    }
    float tx, ty;
    bool found = ai::mem_find(nav_tgt[0] ? nav_tgt : nullptr, &tx, &ty);
    if (!found) {
      nav_why = nav_tgt[0] ? "approach 目标不在记忆里, 请先 observe 锁定" : "approach 无可用目标记忆, 请先 observe";
    } else {
      ai::logf("[ai] approach 目标 全局(%.0f,%.0f)cm 停距%dcm", tx, ty, AI_APPROACH_STOP_CM);
      ai::NavR r = ai::navigate_to(c.t.generation, tx, ty, AI_APPROACH_STOP_CM);
      if (r == ai::NavR::Interrupted) { c.interrupted = true; c.done = true; return; }   // 被接管: 本任务作废
      nav_done = true;
      nav_why = (r == ai::NavR::Reached) ? "已自动靠近目标, 交还你微操" : "靠近收敛结束, 由你继续";
      c.last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
    }
    c.acted = true;   // approach 算一次有意推进(空转兜底/死循环都销账)
  }
  // ---- zoom(放大镜): 通道 `{"zoom":true}`(bool=单次要放大, 发完自动回全幅) / `{"zoom":false}`(回全幅) ----
  if (has_zoom) {
    const bool want_zoom = cmdD["zoom"].as<bool>();
    // bool 形态 = 一次性放大: 发完这一帧(下轮组帧时消费)自动回全幅, AI 不必再发 on:false。
    if (want_zoom) c.zoom_one_shot = true;
    if (want_zoom) {
      // 放大: 固定中央框(忽略 AI 的 px/py/scale, 见下)。
      c.set_zoom_box(0.5f, 0.5f, AI_ZOOM_DEF);
      c.zoom_on = true;
      bool zoom_changed = (c.zoom_x0 != c.sent_x0 || c.zoom_x1 != c.sent_x1 ||
                           c.zoom_y0 != c.sent_y0 || c.zoom_y1 != c.sent_y1);
      float zp_x = 0, zp_h = 0; exec::arm_pos(&zp_x, &zp_h);
      char zp_now[48];
      snprintf(zp_now, sizeof(zp_now), "%.1f,%.1f,%d,%.1f,%.1f",
               (double)ai::s_car_x, (double)ai::s_car_y, (int)ai::s_car_heading,
               (double)zp_x, (double)zp_h);
      bool same_place = !strcmp(zp_now, c.zoom_pose);
      snprintf(c.zoom_pose, sizeof(c.zoom_pose), "%s", zp_now);
      ai::logf("[ai] 放大镜 %.1f× 中心(全幅%.2f,%.2f) 框(%.2f,%.2f)-(%.2f,%.2f)%s%s",
               AI_ZOOM_DEF, 0.5f, 0.5f, c.zoom_x0, c.zoom_y0, c.zoom_x1, c.zoom_y1,
               zoom_changed ? "" : " 无变化", same_place ? "" : " 位姿已变");
      if (!zoom_changed && same_place) {   // 连续要同一块且车臂未动 = 复读, 连要多半就回全幅
        c.zoom_noop_n++;
        if (c.zoom_noop_n > AI_ZOOM_NOOP_MAX) {
          c.zoom_on = false;
          snprintf(c.zoom_repeat, sizeof(c.zoom_repeat),
                   "同一块画面已连要 %d 次, 而这期间车和臂都没动过 —— 画面一字未变, 再要也不会有"
                   "新信息。已回全幅: 下一步必须是实体动作(move/arm/spin/wait)", c.zoom_noop_n);
          ai::logf("[ai] 放大镜复读 %d 次(车臂未动), 强制回全幅", c.zoom_noop_n);
          c.zoom_noop_n = 0;
        } else {
          snprintf(c.zoom_repeat, sizeof(c.zoom_repeat),
                   "这张图就是你手上那张(画面不会变), 第%d次重复请求: 别再发同一条 zoom, 下一步给实体动作",
                   c.zoom_noop_n);
          ai::logf("[ai] 放大镜重复请求(画面已是这块), 保持不动");
        }
      } else {
        c.zoom_noop_n = 0; c.zoom_repeat[0] = 0;   // 画面真变了(或车/臂动过): 重复的账销掉
      }
    } else {
      // 回全幅
      c.zoom_on = false;
      c.zoom_one_shot = false;
      c.zoom_noop_n = 0; c.zoom_repeat[0] = 0;
      ai::logf("[ai] 放大镜关闭, 回全幅");
    }
    JsonDocument cmdF(&g_js_alloc);
    cmdF["type"] = "zoom";
    cmdF["reason"] = c.zoom_on ? "已放大本轮(中央框): 本帧是放大图, 下一轮自动回全幅"
                               : "已回全幅";
    cmdF["params"]["on"] = c.zoom_on;
    String fb = build_feedback(c.t.id, cmdF);
    ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
    {
      const char* r = cmdD["reason"] | "";
      PsaBuf ld;
      ld.put(c.zoom_on ? "zoom 放大镜" : "zoom 回全幅"); ld.put(", 原因: "); ld.put(r);
      utf8_clamp_tail(ld.p);
      if (ld.len) c.hist_add("assistant", ld.p);
    }
    // 换视角是有意推进, 但空操作(要一张手上就有的图)不算:
    if (c.zoom_on) { c.stall = 0; c.stall_hint = false; }
  }
  // move/spin 的默认段长与默认角由 `parse_move` 在规范化时补写(见 `t_move.cpp`): 本文件不再改写指令。
  // ---------- 任务记账(task_note / tasks / task_done / task_goal): 纯元数据, 不含硬件动作 ----------
  { const char* tn = cmdD["task_note"] | "";
    if (tn[0] && strcmp(tn, c.task_note)) {
      strncpy(c.task_note, tn, sizeof(c.task_note) - 1); c.task_note[sizeof(c.task_note) - 1] = 0;
      ai::logf("[ai] 任务笔记: %s", c.task_note);
    } }
  if (cmdD["tasks"].is<JsonArray>()) {
    JsonArrayConst ta = cmdD["tasks"].as<JsonArrayConst>();
    // tasks = "重写整个任务列表", 提示词只给名字 ⇒ done 不从 JSON 读: 重写即从"全部未完成"重来,
    // 进度一律由 task_done 标记(渲染时才看得出来谁完成)。
    int n = 0;
    for (JsonObjectConst it : ta) {
      if (n >= 8) break;
      const char* nm = it["name"] | "";
      if (!nm[0]) continue;
      strncpy(c.s_tasks[n].name, nm, 47); c.s_tasks[n].name[47] = 0;
      c.s_tasks[n].done = false;
      n++;
    }
    if (n > 0 || c.s_task_n > 0) { c.s_task_n = n; ai::logf("[ai] 任务列表更新(%d项)", c.s_task_n); }
  }
  if (cmdD["task_done"].is<JsonObject>()) {
    int idx = (cmdD["task_done"]["index"] | 0) - 1;
    if (idx >= 0 && idx < c.s_task_n) {
      c.s_tasks[idx].done = cmdD["task_done"]["done"] | false;
      ai::logf("[ai] 任务%d → %s", idx + 1, c.s_tasks[idx].done ? "完成" : "未完成");
    }
  }
  { const char* tgoal = cmdD["task_goal"] | "";
    if (tgoal[0] && strcmp(tgoal, c.goal_now)) {
      snprintf(c.goal_now, sizeof(c.goal_now), "%s", tgoal);
      utf8_clamp_tail(c.goal_now);
      ai::logf("[ai] 目标更新: %s", c.goal_now);
    } }

  // ---------- 逐通道落地(move/arm/light), 汇总主动作/合并串 ----------
  char merge_str[96] = {0};          // 本轮合并指令串(死循环判据 + 历史 + 留档)
  JsonDocument acmd(&g_js_alloc);     // 主动作(手机端 command 显示), 优先级 move>spin>arm>light>approach
  bool has_main = false;
  bool any_ok = false, any_attempted = false;
  if (is_mv) {
    JsonObjectConst mcv = cmdD["move"].as<JsonObjectConst>();
    bool ok = exec::act("move", mcv);
    c.acted |= ok; any_ok |= ok; any_attempted = true;
    ai::car_update_pose("move", mcv);   // 定距 move 累积位移(approach 由 navigate_to 内部自累积, 勿重复)
    if (!has_main) { acmd["type"] = "move"; acmd["params"] = mcv; has_main = true; }
    int dc = mcv["distance_cm"] | 0; float mth = mcv["throttle"] | 0.0f;
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
             "move %dcm th=%.1f; ", dc, mth);
    // 后退(th<0)会把目标甩出视野; 前进不遮。仅在确认落地时标, 供下轮"没锁定目标"提示判断。
    if (ok && mth < -0.001f) c.last_round_hide_target = true;
  } else if (is_sp) {
    JsonObjectConst mcv = cmdD["move"].as<JsonObjectConst>();
    bool ok = exec::act("spin", mcv);
    c.acted |= ok; any_ok |= ok; any_attempted = true;
    ai::car_update_pose("spin", mcv);
    if (!has_main) { acmd["type"] = "spin"; acmd["params"] = mcv; has_main = true; }
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
             "spin %d %d°; ", (mcv["dir"] | 0), (mcv["angle_deg"] | 0));
    // 转向(spin)是**主动去找**目标、改变视线的动作, 不是遮挡源: 转了还看不到就说明目标真不在
    // 当前方向上, 下轮不该再提示"被遮挡"。故不标 true(保持上方重置的 false), 有别于后退 move。
  }
  if (has_arm) {
    JsonObjectConst acv = cmdD["arm"].as<JsonObjectConst>();
    bool ok = false;
    if (!strcmp(act, "low"))        ok = exec::arm_low();
    else if (!strcmp(act, "raise")) ok = exec::arm_raise();
    else if (!strcmp(act, "pose"))  ok = exec::arm_pose(acv["x"] | 0.0f, acv["h"] | 0.0f);
    else {   // grasp/clip/release/fold
      // 夹取前先放大取一帧"合爪前特写"(下轮自动带上作前后对比): 本轮已发放大图则复用那帧, 否则
      // 现抓一张; **抓完才开始合爪抬臂**, 下一轮 AI 才能判断"原本在两指间 → 现在夹住随爪离地"。
      if (!strcmp(act, "grasp") || !strcmp(act, "clip")) {
        c.want_prev_grasp = true;
        if (!c.grasp_prev) c.grasp_prev = (uint8_t*)heap_caps_malloc(AI_ZOOM_JPG_MAX, MALLOC_CAP_SPIRAM);
        if (c.sent_zoomed && c.frame && c.frame_len > 0 && c.grasp_prev && c.frame_len <= AI_ZOOM_JPG_MAX) {
          memcpy(c.grasp_prev, c.frame, c.frame_len); c.grasp_prev_len = c.frame_len;   // 本轮发的就是放大图
        } else if (c.grasp_prev) {
          // ⚠️ 同轮若还有 move/spin, 它们已在本分支之前落地、车正在走 ⇒ 此刻抓的是糊帧, 而这张恰恰是
          //    下轮判"原本在两指间 → 现在夹住"的**基准图**, 最不能糊 ⇒ 先等轮子停稳再抓。
          //    (走上面复用 c.frame 那条路不必等: c.frame 是 round_prepare 抓的, 当时车已停稳。)
          //    车本就没动时 settle 的等待循环立刻退出, 只多一次固定 dwell。
          ai::settle_wheels(c.t.generation, AI_SETTLE_MAX_MS);
          int hok = 0;
          camera_fb_t* hfb = cam::request_hires(AI_HIRES_FRAME, &hok);
          size_t gl = 0;
          bool ok_g = hfb && hok &&
                      magnify::crop_center_jpg(hfb->buf, cam::jpeg_len(hfb),
                                               hfb->width, hfb->height,
                                               c.grasp_prev, AI_ZOOM_JPG_MAX, &gl,
                                               AI_HIRES_OUT_W, AI_HIRES_OUT_H, AI_ZOOM_QUALITY) && gl > 0;
          if (hfb) cam::return_frame(hfb);
          c.grasp_prev_len = ok_g ? gl : 0;
          ai::logf("[ai] 夹取前放大特写: %s(%uKB)", ok_g ? "已取帧" : "取帧失败", (unsigned)(gl / 1024));
        }
      } else if (!strcmp(act, "release")) { c.want_prev_grasp = false; }
      JsonDocument a(&g_js_alloc); a["act"] = act;
      ok = exec::act("arm", a.as<JsonObjectConst>());
    }
    c.acted |= ok; any_ok |= ok; any_attempted = true;
    if (!has_main) { acmd["type"] = "arm"; acmd["params"] = acv; has_main = true; }
    // 机械臂"放进视野"的动作(low/clip/grasp 或低位 pose)落地后, 下一帧臂会在镜头前下方挡住目标;
    // raise/fold/release 是收臂/让出视野, 不遮。与 move 后退同义 —— 都解释"画面里找不到目标"。
    bool hide_arm = ok && (!strcmp(act, "low") || !strcmp(act, "clip") ||
                           !strcmp(act, "grasp") ||
                           (!strcmp(act, "pose") && (acv["h"] | 0.0f) <= AI_GRASP_H_CM));
    if (hide_arm) c.last_round_hide_target = true;
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str), "arm %s; ", act);
  }
  if (has_light) {
    JsonObjectConst lcv = cmdD["light"].as<JsonObjectConst>();
    bool ok = exec::act("light", lcv);
    c.acted |= ok; any_ok |= ok;
    if (!has_main) { acmd["type"] = "light"; acmd["params"] = lcv; has_main = true; }
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
             "light %s %s; ", (lcv["kind"] | ""), (lcv["on"] | false) ? "on" : "off");
  }
  if (nav_done && !has_main) {
    acmd["type"] = "approach"; acmd["params"]["target"] = nav_tgt[0] ? nav_tgt : "(最近)"; has_main = true;
  }
  if (any_attempted || nav_done) c.last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
  // AI 的 move/spin 全有界(throttle 补距离 / spin 补默认角)、arm 全离散 → 无持续残留。
  g_last_continuous = false; g_last_cont_type = 0;

  // ---------- 反馈 / 历史 / 死循环 ----------
  if (!has_any && !nav_done) {
    // 空动作轮(只带 reason/observe/task_* 等元数据): wait 语义自理。
    c.stall = 0; c.stall_hint = false;
    uint64_t now = (uint64_t)(esp_timer_get_time() / 1000);
    // goal=abort 是本轮的状态变化(开始等用户), 不能等节流窗口 —— 用户不知道 AI 在等他回复,
    // 这功能就等于没开。其余空动作轮仍按 AI_WAIT_FB_MIN_MS 节流防刷屏。
    if (!strcmp(cmdD["goal"] | "", "abort") || now - g_last_wait_fb_ms >= AI_WAIT_FB_MIN_MS) {
      g_last_wait_fb_ms = now;
      String fb = build_feedback(c.t.id, cmdD);
      ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
    }
  } else {
    if (has_main && (any_ok || nav_done)) {
      acmd["reason"] = cmdD["reason"] | "";
      String fb = build_feedback(c.t.id, acmd);
      ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
    } else if (has_main) {
      ai::logf("[ai] 执行失败(exec 拒绝) %s", merge_str);
      JsonDocument cmdF(&g_js_alloc);
      cmdF["type"] = acmd["type"] | "none";
      cmdF["params"] = acmd["params"];
      cmdF["reason"] = "执行层拒绝, 动作未落地";
      String fb = build_feedback(c.t.id, cmdF);
      ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
    }
    // 死循环防线: 合并串比对(空动作轮已在上文走 wait, 不误伤)
    if (strcmp(merge_str, c.last_cmd)) { c.stall = 0; c.stall_hint = false; }
    else if (++c.stall >= 3 && !c.stall_hint) { c.stall_hint = true; blog::logf(blog::AI, "多轮无进展, 注入引导"); }
    snprintf(c.last_cmd, sizeof(c.last_cmd), "%s", merge_str);
    if (merge_str[0]) {   // 留档
      char nb[128];
      snprintf(nb, sizeof(nb), "执行%s%s", merge_str, any_ok ? "" : "(exec拒绝)");
      ai::dump_note(nb);
    }
  }
  // 历史回喂: 合并串(或 approach/等待说明) + 完整 reason
  {
    const char* r = cmdD["reason"] | "";
    const char* goal_v = cmdD["goal"] | "";
    const char* what = merge_str[0] ? merge_str
                       : (nav_done ? (nav_why ? nav_why : "approach") : "wait");
    if (!strcmp(goal_v, "abort")) what = "中止并等待用户输入";   // 别让历史里只写个 wait
    PsaBuf ld;
    ld.put(what);
    ld.put(", 原因: "); if (r) ld.put(r);
    utf8_clamp_tail(ld.p);
    if (ld.len) c.hist_add("assistant", ld.p);
  }

  // ---------- AI 决策日志: 本轮"用了哪些工具 + 完整 JSON" ----------
  // 复盘靠原样 JSON、不能截断: 用 blog::forward_text 走 PSRAM 任意长转发(不占 logf 的栈缓冲); 另打短摘要。
  if (content.length() > 0) {
    char tob[160] = {0};   // 短摘要: JSON 里的顶层工具 + 主动作
    bool first = true;
    if (cmdD["observe"].is<JsonObject>()) {
      JsonObjectConst ob = cmdD["observe"].as<JsonObjectConst>();
      snprintf(tob, sizeof(tob), "observe(%s,px%.2f,py%.2f)",
               ob["name"] | "", ob["px"] | 0.0f, ob["py"] | 0.0f);
      first = false;
    }
    if (has_zoom) first = ai::safe_append(tob, sizeof(tob), &first, "zoom");
    if (has_move && is_mv)  ai::safe_append(tob, sizeof(tob), &first, is_ap ? "approach" : "move");
    if (has_move && is_sp)  ai::safe_append(tob, sizeof(tob), &first, "spin");
    if (has_arm)            ai::safe_append(tob, sizeof(tob), &first, "arm");
    if (cmdD["light"].is<JsonObject>()) ai::safe_append(tob, sizeof(tob), &first, "light");
    if (cmdD["done"].is<bool>() && cmdD["done"].as<bool>())
      ai::safe_append(tob, sizeof(tob), &first, "done");
    { const char* gs = cmdD["goal"] | "";
      if (gs[0]) { char gb[16]; snprintf(gb, sizeof(gb), "goal=%s", gs); ai::safe_append(tob, sizeof(tob), &first, gb); } }
    ai::logf("[ai工具] 本轮: %s", tob[0] ? tob : "(纯 wait/reason)");
    blog::forward_text(blog::AI, content.c_str());   // 完整 JSON(任意长), 供复盘
  }

  // ---------- done 门: done:true = 任务完成(含停车) ----------
  if (cmdD["done"].is<bool>() && cmdD["done"].as<bool>()) {
    c.done = true;
    c.sent_done = true;   // 已向手机确报终态, 任务出口不再补发
    blog::logf(blog::AI, "AI 判定任务完成(done)");
    JsonDocument st(&g_js_alloc); st["scope"] = "all";
    exec::act("stop", st.as<JsonObjectConst>());   // 停机收尾
  }

  // ---------- goal 门: finish=完成收尾 / fail=失败收尾 / abort=中止并等用户输入(任务不结束) ----------
  {
    const char* goal_v = cmdD["goal"] | "";
    if (goal_v[0]) {
      JsonDocument st(&g_js_alloc); st["scope"] = "all";
      exec::act("stop", st.as<JsonObjectConst>());   // 终态前先急停, 不留残留动作
      if (!strcmp(goal_v, "abort")) {
        c.wait_user = true;   // 下一轮起进入等待态(不取帧/不发请求), 见 round_prepare 头部
        c.wait_until = (uint64_t)(esp_timer_get_time() / 1000) + AI_WAIT_USER_MS;
        blog::logf(blog::AI, "AI 请求中止, 等待用户输入(最多%us)",
                   (unsigned)(AI_WAIT_USER_MS / 1000));
      } else if (!strcmp(goal_v, "fail")) {
        c.fail = "AI 判定执行失败";
        c.done = true;
        blog::logf(blog::AI, "AI 判定执行失败(goal=fail)");
      } else {   // finish
        c.done = true;
        c.sent_done = true;   // 已向手机确报终态, 任务出口不再补发
        blog::logf(blog::AI, "AI 判定任务完成(goal=finish)");
      }
    }
  }
  c.got = true;
}

// ---------------- 单轮驱动 ----------------
static RoundR round_step(RoundCtx& c) {
  uint64_t step_ts = esp_timer_get_time();  // 本轮起点(周期控制基准)
  c.fail = nullptr;  // 每轮重置, 避免沿用上轮错误文本误导日志/回报
  c.got = false;

  PrepR pr = round_prepare(c);
  if (pr != PrepR::Ok) return pr == PrepR::SkipRound ? RoundR::NextRound : RoundR::ExitTask;

  for (int attempt = 0; attempt < 2 && !c.done; attempt++) {
    if (round_attempt(c, attempt) != AttR::Retry) break;
  }
  if (c.done) return RoundR::ExitTask;
  // 单轮模式(/ai oneshot): 一轮决策即收尾。本轮无有效输出(网络/解码/校验失败)
  // 时把原因作为 error 回报, 不跳 3s 继续观察; 持续指令残留由任务出口 resolve_stop 补停。
  if (c.t.one_shot) {
    if (!c.got && c.fail) {
      JsonDocument e(&g_js_alloc);
      e["error"] = c.fail;
      String s = build_feedback(c.t.id, e);
      ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
    }
    // 单轮收尾: 补一条带 done 的完成通知, 让手机端复位「发送」并显示任务结束。
    JsonDocument e(&g_js_alloc);
    e["done"] = true;
    String s = build_feedback(c.t.id, e);
    ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
    c.sent_done = true;   // 单轮已确报终态
    c.done = true;
    return RoundR::ExitTask;
  }
  if (c.got) c.chat_unacked = false;   // 本轮模型给出了有效决策(已进入它的上下文): 插话不必再点名
  if (!c.got) {
    // 网络/解析持续失败: 逐轮指数退避(3s→6s→12s→24s 封顶), 避免每轮狂建连接把云端打得更紧;
    // 429 限流给更长喘息(30s)再继续; 失败原因同步推手机。
    if (++c.net_fail >= AI_MAX_NET_FAIL) {
      const char* reason = "云端持续无响应(疑似限流), 任务已中止, 请稍后重试";
      JsonDocument e(&g_js_alloc);
      e["error"] = reason;
      e["done"] = true;   // 终结必带 done, 让手机端把「中止」复位为「发送」
      c.sent_done = true;
      String s = build_feedback(c.t.id, e);
      ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
      blog::logf(blog::AI, "连续网络失败超限, 任务中止");
      return RoundR::ExitTask;
    }
    int wait = ai::http_last_status() == 429 ? 30000 : (3000 << (c.net_fail > 3 ? 3 : c.net_fail - 1));
    ai::logf("[ai] 本轮无有效输出(%s), %ds 后重试", c.fail ? c.fail : "未知", wait / 1000);
    vTaskDelay(pdMS_TO_TICKS(wait));
  } else {
    c.net_fail = 0;
  }
  // 空转兜底(见 AI_IDLE_ROUNDS): 连续多轮没有任何实际动作 → 大概率被放大镜锁在局部画面里空转。
  // 搜索只能在全幅做, 所以强制回全幅, 并把原因明说 —— 只清状态不说清, AI 下一轮还会再放大。
  if (c.acted) { c.idle_rounds = 0; }
  else if (++c.idle_rounds >= AI_IDLE_ROUNDS) {
    c.idle_rounds = 0;
    if (c.zoom_on) {
      c.zoom_on = false;
      c.zoom_one_shot = false;
      c.zoom_noop_n = 0; c.zoom_repeat[0] = 0;
      snprintf(c.zoom_warn, sizeof(c.zoom_warn),
               "连续 %d 轮没动作, 放大镜已自动回全幅: 画面里没目标时放大没有意义。"
               "先 fold 收臂, 再用 spin 小幅环视找(单步≤60°), 每转一次停下看画面。",
               AI_IDLE_ROUNDS);
      ai::logf("[ai] 连续 %d 轮无动作, 放大镜强制回全幅", AI_IDLE_ROUNDS);
    }
  }
  c.acted = false;

  ai::mem_tick_stale();   // 每轮结束: 未观测的物体过期轮数 +1

  // 滚动缓存上一帧: 供下一轮 carry_image / grasp 自动带图作对比。zoom/full 已在解析时预取进 prev;
  // 这里只补 grasp 自动特写(优先覆盖预取)与无预取时的滚动存帧。
  if (c.want_prev_grasp && c.grasp_prev_len > 0 && c.grasp_prev_len <= AI_EDITED_IMG_MAX) {
    if (!c.prev) c.prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (c.prev) { memcpy(c.prev, c.grasp_prev, c.grasp_prev_len); c.prev_len = c.grasp_prev_len; } else c.prev_len = 0;
    c.grasp_prev_len = 0;   // 特写已消费
  } else if (!c.prev_preset && c.cur_len > 0) {
    if (!c.prev) c.prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (c.prev && c.cur_len <= AI_EDITED_IMG_MAX) { memcpy(c.prev, c.cur, c.cur_len); c.prev_len = c.cur_len; }
    else c.prev_len = 0;
  }

  if (++c.steps >= AI_MAX_STEPS_PER_GOAL) {
    JsonDocument e(&g_js_alloc); e["done"] = true; String s = build_feedback(c.t.id, e); ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
    c.sent_done = true;
    return RoundR::ExitTask;
  }
  // 周期控制: 以 AI_INTERVAL_MS 为下限节奏, 扣掉本轮已耗时(含抓帧/HTTP/校验),
  // 服务端快(单帧 ~2s)时立即进入下一轮, 慢时由服务端耗时主导。
  uint64_t el = esp_timer_get_time() - step_ts;
  long rem = (long)AI_INTERVAL_MS - (long)(el / 1000);
  if (rem > 0) vTaskDelay(pdMS_TO_TICKS(rem));
  return RoundR::NextRound;
}

// ---------------- 任务收尾 ----------------
static void round_task_finish(RoundCtx& c) {
  // 插话到任务结束都没被落实: 明说一句 —— 否则用户只看到"提醒没反应", 无从知道是它被
  // 报废的那几轮吃掉了(历史环随任务清空, 下一个任务不会带上它, 也不该带)。
  if (c.chat_unacked) blog::logf(blog::AI, "任务结束: 用户的插话始终未被落实(随本任务作废)");

  // 任务终结补发 done: 覆盖失败/掉线等"只报 error 不带 done"的终态, 让手机端把「中止」复位为
  // 「发送」。被中断时跳过(避免误复位下一任务); 已置 sent_done 的不再补发。
  if (!c.interrupted && !c.sent_done) {
    JsonDocument e(&g_js_alloc);
    e["done"] = true;
    if (c.fail) e["error"] = c.fail;
    String s = build_feedback(c.t.id, e);
    ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
  }

  for (int i = 0; i < AI_EDITED_SLOTS; i++) if (c.ed_img[i]) free(c.ed_img[i]);  // 用户编辑图快照
  if (c.cur) free(c.cur);          // 本轮帧 PSRAM 副本
  if (c.prev) free(c.prev);        // 上一帧 PSRAM 副本
  if (c.grasp_prev) free(c.grasp_prev);  // 夹取前特写缓冲(任务期复用, 不跨任务留着占 PSRAM)
  if (c.zoom_jpg) free(c.zoom_jpg);   // 放大图缓冲(任务期复用, 不跨任务留着占 PSRAM)
  for (int i = 0; i < AI_HIST_N; i++) free(c.hist_text[i]);   // 历史环 PSRAM
  if (c.t.ctx) delete (int*)c.t.ctx;  // 任务期 sink fd
  ai::logf("[ai] 任务结束 gen=%lu", c.t.generation);
  // 本任务的水位低点: 串口一行(开了 /log ai 也会转发手机)。真正的价值在
  // **没开日志也读得到** —— 同一组数字也在 /log 体征行里, 走 BLE 随时可取(见 command.cpp)。
  hwatch::report("AI任务");
  free(c.t.text); free(c.t.ann);
  resolve_stop();   // 统一兜底: 持续指令残留即补停
  ai::set_busy(false);
}

void ai::round_run_task(ai::TaskLocal& t) {
  RoundCtx c(t);   // 按值建在 worker 栈上(不额外堆分配)
  round_task_init(c);
  while (round_step(c) == RoundR::NextRound) {}
  round_task_finish(c);
}