#include "src/ai/ai_round.h"
#include "src/ai/ai_client.h"   // ai::logf / enqueue_result 等协作入口 + ai::generation
#include "src/ai/ai_result.h"   // AI_EDITED_* + edited_snapshot
#include "src/ai/ai_prompt.h"   // PsaBuf/PieceList + build_body
#include "src/ai/ai_mem.h"      // 空间记忆/车姿态: mem_reset/mem_feed/mem_find/...
#include "src/ai/ai_http.h"     // http_post/http_last_status/http_stop/extract_tool_calls
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
#include "src/core/utf8.h"         // utf8_clamp_tail: 定长缓冲按字节截断后的边界回退
#include "src/core/psram.h"        // ps_dup/ps_set/ps_free/ps_str: 变长文本按实际长度分配

#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <stdio.h>
#include <stdlib.h>  // malloc/free/strtol/atoi
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 决策频率
#define AI_INTERVAL_MS 1500
// 任务不设回合数上限: 一个任务要连续做几十个动作很常见, 到顶就掐会让长任务半途而废。
// 任务终止只由这些决定: 模型自己 finish/等待用户、用户中止、连续无有效输出(AI_MAX_NET_FAIL)。
// goal=abort(工具 finish result="wait") = AI 请求中止并等待用户输入。等待期间**不取帧、不发云端请求**
// (不耗额度、不推进任务回合), 只轮询用户回复(→继续执行)与超时; 超时后给 AI 一句"用户未回复"再继续。
#define AI_WAIT_USER_MS 60000
#define AI_WAIT_USER_POLL_MS 200   // 等待期的轮询步长(只查用户回复/代际/超时)
#define AI_MAX_NET_FAIL 4         // 连续"无有效输出"回合数上限: 超过即中止任务并回报(防云端持续无响应时空转)
// 历史环回合数(AI 工具调用 + 用户消息共用)。
// 语义从旧的"条数"改为"回合数": 一回合 = 1 条 assistant(tool_calls, 最多 AI_HIST_CALL_MAX 个) + N 条 tool 结果,
// 淘汰整回合才不会留下"孤儿 tool"(配对断了云端会拒)。旧 20 条 ≈ 新 10 回合。
// ⚠️ 带 tools 时历史轮次的 reasoning_content 要**全程**回传并被拼进上下文 ⇒ 每轮请求体随轮数线性变大
// (实测一轮文本 ~5KB, 30 轮约 150KB, 每轮都要重传一次), 故设软上限, 到线后催 AI 用 compact 压缩。
// 但图片只发"最新一组"、系统提示词/tools 每轮固定, 不随轮数涨 —— 故上限可以放得比"纯文本直觉"更宽。
#define AI_HIST_SOFT_TURNS 30   // 软上限: 达到后每轮在状态块里催 AI 压缩
#define AI_HIST_MAX_TURNS 200   // 兜底上限: 到顶才丢最旧整回合(防 AI 始终不压缩把请求体撑爆)

#define AI_FRAME_RETRY 4          // 单轮抓帧重试次数(推流并发占缓冲时会偶发取不到)

// ---------------- 放大镜(look(zoom=true)) ----------------
// 放大镜: 把 AI 的"眼睛"凑近(裁块放大回喂), 让它在放大图里跟**看得见的夹爪**比相对位置; 裁框由程序记账 ⇒ 换回全幅是精确算术。
#define AI_ZOOM_QUALITY 80          // 比图传略高(这是用来看细节的), 一帧几十 KB
// 放大镜=切高清真拍(request_hires): 切 SVGA 抓一帧, TJpgDec 部分解码只裁中央 (0.25,0.25)-(0.75,0.75),
// 中央 1/4 源 1:1 回喂。⚠️ 只用 SVGA: SXGA 的 reconfigure 重建会卡死驱动。
#define AI_HIRES_FRAME   FRAMESIZE_SVGA   // 高清目标分辨率（800×600）
#define AI_HIRES_OUT_W   400              // 放大输出＝高清源中央 1/4（1:1，同源像素密度）
#define AI_HIRES_OUT_H   300
#define AI_ZOOM_DEF 2.0f            // 倍数(与固定中央框等价, 只用于图注)
// 同一块画面在车与臂都没动的前提下连要几次后点名(见 look 分支)。
// ⚠️ 判"重复"必须带上"这期间有没有动作": 裁框是全幅归一化坐标, 挪车后同一个框里已是另一片世界。
#define AI_ZOOM_NOOP_MAX 2
// 连续这么多回合没有任何**实际动作**(只有 look/mem/wait 这类只读调用)就认定空转, 点名让 AI 改为搜目标:
// 放大的局部画面里找不到画面外的目标, 搜索只能在全幅做。
#define AI_IDLE_ROUNDS 5

#define AI_ZOOM_HINT \
  "本帧是放大图(只有画面中心区域), 看不到全幅, 若目标基本被完全遮挡则说明没对准"

// 出图时车/机械臂仍在动: 本帧只作粗略参考(定距/定角靠时长近似, 到点后还会滑行), 别据此精确判位置。
#define AI_FRAME_MOVING_HINT \
  "本帧是车/机械臂尚未停稳时拍摄的, 可能模糊或偏位, 不宜据此精确判断位置与对准"

// 本地巡航(approach / /move to)到位距离。approach 是开环死航(无里程计, 靠姿态累积+定距时长近似),
// 目标坐标又来自单应高报 ⇒ 停得近会**越过爪口把方块压到车底/机械臂下**; 留余量停远些, 宁让 AI 多走几步微调。
#define AI_APPROACH_STOP_CM 20

// ---------------- 文件级状态(仅本模块用) ----------------
// 最近一条下发执行板的是否持续型(sink: stop 兜底判定)。任务起点复位。
static volatile bool g_last_continuous = false;
// 最近一条持续指令的类型(0=无/1=move/2=arm)。兜底 stop 时 Wheels 模式只适用于轮子残留。
static volatile int g_last_cont_type = 0;

// ---------------- 图片全局编号 ----------------
// 每张发给模型的图(实景帧 + 用户参考图)各发一个全局递增号 ImageN, **跨任务不重置**: 模型据此回看,
// 号只增不减 ⇒ 历史里的占位文本可以写死编号, 不必每轮按环滚动现算"还能回看第几张"(旧写法即由此
// 让模型误判)。只有本模块发号(worker 单线程), 无需加锁。
static uint32_t g_img_seq = 0;
static uint32_t img_next_id() { return ++g_img_seq; }

// ---------------- 任务上下文 ----------------
// 跨回合/跨阶段的全部状态。按值建在 round_run_task 的栈上(worker 那 16KB PSRAM 栈帧内);
// 历史环**表本身**另在 PSRAM(见 hist_new_turn), 这里只放指针与中小缓冲, 不为它吃栈。
struct RoundCtx {
  ai::TaskLocal& t;
  explicit RoundCtx(ai::TaskLocal& tt) : t(tt) {}

  bool cam_ok = false;              // 摄像头可用性(任务起点判定, init 后即定)

  // 任务级终止/出口
  bool done = false;
  bool got = false;                 // 本回合是否产出了有效决策(拿到工具调用)
  bool sent_done = false;           // 是否已确报过终态 done
  bool interrupted = false;         // 是否因新目标/手动中断退出(此时不发补发 done)
  const char* fail = nullptr;       // ⚠️ 可能指向 err_buf, 故二者同生命周期
  char err_buf[160];

  // 计数/防线
  unsigned long steps = 0;
  char last_cmd[96] = {0};          // 上一回合"合并动作串"(死循环判据)
  unsigned long last_act_ms = 0;    // 上次真正下执行/微操指令的时刻(ms)
  int net_fail = 0;                 // 连续"无有效输出"回合数
  int stall = 0;
  bool stall_hint = false;
  int idle_rounds = 0;              // 连续"无实际动作"回合数
  bool acted = false;               // 本回合是否有实际动作落地(exec 接受)

  // 帧缓冲(PSRAM, 任务期复用): cur = 最近一张发给模型的实景; prev[] = 更早几张(look 回看用), [0]=最新
  uint8_t* cur = nullptr; size_t cur_len = 0;
  uint32_t cur_id = 0;              // cur 的全局图片编号(0=无): 帧发出去后随 prev_roll 进环
  uint8_t* prev[AI_PREV_SLOTS] = {};
  size_t prev_len[AI_PREV_SLOTS] = {};
  uint32_t prev_id[AI_PREV_SLOTS] = {};   // 各槽全局图片编号(0=空槽); 与 prev/prev_len 同生同灭
  uint32_t look_ids[2] = {};        // 本条 look 清单里各图的全局编号(与 look_cnt 配对)
  bool frame_moving = false;        // 首帧抓取时车/臂仍在动 → 提示别硬信本帧
  const uint8_t* frame = nullptr;   // 本回合由程序注入尾部 user 的那张图(首轮; 发过一次即清)
  size_t frame_len = 0;
  char frame_note[128] = {0};       // 尾部画面的文本前缀(空=本回合不注入尾部画面); 固定文案本身就 ~65B
  bool img_sent = false;            // 本回合 look 出的一组图是否已随请求发出去过(发过就只留占位文本)
  JsonDocument calls{&g_js_alloc};  // 本回合从响应里取出的工具调用(raw: id/name/args)

  // look 出图的清单(下一回合组包时注入到 look 那条 tool 结果): 顺序 = 给图顺序, 最多 2 张(硬上限)。
  // 指针指向任务期常驻缓冲(cur/prev/ed_img), 任务中途不会被释放。
  struct LookImg { const uint8_t* p = nullptr; size_t n = 0; };
  LookImg look_set[2];
  uint8_t look_cnt = 0;             // 清单张数(0=本回合没出图)
  bool look_live = false;           // 清单含"新拍的实景" → 才算 observe 基准 / 才滚动 prev

  // 图基准: 模型这回合报的 px/py 指的是"它最后看到的那张图"的裁框 ⇒ 组包时把 pend_* 提交进 sent_*,
  // look 新拍后写 pend_* 供**下一回合**用(同回合 car{observe}+look 时 observe 读到的仍是旧基准)。
  bool sent_zoomed = false;         // 模型此刻看的是不是放大图
  float sent_x0 = 0, sent_y0 = 0, sent_x1 = 1, sent_y1 = 1;   // 那张图的框(全幅归一化)
  float sent_car_x = 0, sent_car_y = 0;   // 那张图**拍摄当时**的车位姿: 单应解算必须用它, 不能用车此刻的位姿(否则移动后 observe 会整体偏移)
  int16_t sent_car_hd = 0;
  bool pend_valid = false, pend_zoomed = false;   // 纯对照图(prev/user)不更新基准, 故不设 pend
  float pend_x0 = 0, pend_y0 = 0, pend_x1 = 1, pend_y1 = 1;
  float pend_car_x = 0, pend_car_y = 0;   // 位姿快照与 pend 同生同灭
  int16_t pend_car_hd = 0;
  uint16_t img_owner = 0;           // 清单属于哪个回合(0=无): 组包只给拥有者的字节
  char zoom_pose[80] = {0};         // 上次放大时的车位姿+臂姿(判"重复要同一块"要看这期间动过没有)
  int zoom_noop_n = 0;

  // 提示缓冲(一次性; 组成结果文本/组包时即清)
  char pend_hint[640] = {0};        // 待挂到"下一条工具结果/本轮尾部画面"的提示
  char obs_warn[160] = {0};         // 观测未记成的回告
  char obs_echo[200] = {0};         // 本轮记成的观测读数(车头系带方向), 随 car 结果当场回执
  char task_remind[160] = {0};      // 用户消息即将被冲掉前的提醒
  char wait_note[160] = {0};        // finish result="wait" 等待超时的一次性告知
  bool img_purged = false;          // 本回合刚做过历史压缩(画面已作废): 下回合状态块里点名一次

  // 等待用户回复
  bool wait_user = false;           // finish result="wait" 的等待态
  uint64_t wait_until = 0;          // 等待截止(ms, esp_timer)

  // 备注/列表/目标 —— 这三段长度全是"写入那一刻才知道"(模型当场给), 所以一律按实际长度分配到
  // PSRAM(见 src/core/psram.h), **不设字数上限**, 也就不存在切断残尾变 '?' 的问题;
  // 收尾时在 round_task_finish 统一释放。RoundCtx 本身反而因此变小了(定长数组 → 指针)。
  char* task_note = nullptr;        // AI 写入的任务笔记, 随 car 结果喂回
  struct TaskItem { char* name; bool done; };
  TaskItem* s_tasks = nullptr;      // 任务列表: 单块 PSRAM(数组 + 名字区), 每次 todo 重写整体替换
  int s_task_n = 0;
  char* goal_now = nullptr;         // 当前任务目标(可被 goal.set 热替换)
  bool goal_explicit = false;       // AI 是否已显式 goal.set 过(≠"文案变了": 它常把用户原话原样写回来)
  char compact_sum[512] = {0};      // compact 工具给的进展摘要: 本回合落地跑完后据此压缩历史(见 compact_history)

  // 历史环(PSRAM 表, 任务起点分配 / 收尾释放): 一回合一条, 内含 1~AI_HIST_CALL_MAX 个工具调用
  HistTurn* hist = nullptr;
  int hist_n = 0;
  uint16_t img_tag_n = 0;           // 图片归属标记发号器: 每回合 +1(0 保留给"无"), 与 img_owner 配对

  // 用户参考图快照(供整任务复用): 与实景帧统一编号, 各槽记下自己的全局编号 ed_num(0=空槽)
  uint8_t* ed_img[AI_EDITED_SLOTS] = {};
  size_t ed_len[AI_EDITED_SLOTS] = {};
  uint32_t ed_num[AI_EDITED_SLOTS] = {};
  int ed_n = 0;

  // ---- 成员函数(与调用者同 TU ⇒ 仍可内联) ----
  // AI 报的像素(它看到的那张图) → 全幅归一化: 上一轮发全幅就直通, 是放大图就按当时记下的裁框反算。
  float to_full_x(float px) const { return sent_zoomed ? sent_x0 + px * (sent_x1 - sent_x0) : px; }
  float to_full_y(float py) const { return sent_zoomed ? sent_y0 + py * (sent_y1 - sent_y0) : py; }
};

// ps_dup / ps_set / ps_free / ps_str 见 src/core/psram.h(ai_mem 也要用同一套, 见那里的头注)。
// 本文件的历史环、任务列表、笔记、目标都靠它按实际长度分配。

// ---------------- 任务列表(PSRAM 单块) ----------------
// 整块布局 = [TaskItem 数组][各名字的字符区], 一次 malloc / 一次 free。
// 不按条分配: N 条 N 次小分配会把 PSRAM 也切碎(和内部堆一个道理)。
static void task_list_free(RoundCtx& c) {
  free(c.s_tasks);          // 名字就在同一块尾部, 一次放掉
  c.s_tasks = nullptr;
  c.s_task_n = 0;
}

// 用 todo 数组整份替换任务列表(空数组 = 清空)。**项数不设上限**。
// 分配失败返回 false 且保留原列表 —— 调用方据此回一句"内存不足, 列表未更新", 让模型精简后重写;
// 绝不能默默丢掉后半截(那会让模型以为自己写了 10 条、实际只跑 8 条)。
static bool task_list_replace(RoundCtx& c, JsonArrayConst ta) {
  int n = 0;
  size_t names = 0;
  for (JsonVariantConst it : ta) {
    if (!it.is<const char*>()) continue;      // 只认字符串项
    const char* nm = it.as<const char*>();
    if (!nm || !nm[0]) continue;
    n++;
    names += strlen(nm) + 1;
  }
  if (n == 0) { task_list_free(c); return true; }   // 重写成空表: 合法
  char* blob = (char*)heap_caps_malloc((size_t)n * sizeof(RoundCtx::TaskItem) + names, MALLOC_CAP_SPIRAM);
  if (!blob) return false;
  RoundCtx::TaskItem* arr = (RoundCtx::TaskItem*)blob;
  char* pool = blob + (size_t)n * sizeof(RoundCtx::TaskItem);
  int k = 0;
  for (JsonVariantConst it : ta) {
    if (!it.is<const char*>()) continue;
    const char* nm = it.as<const char*>();
    if (!nm || !nm[0]) continue;
    size_t l = strlen(nm);
    memcpy(pool, nm, l + 1);
    arr[k].name = pool;
    arr[k].done = false;
    pool += l + 1;
    k++;
  }
  task_list_free(c);   // 新的已建好, 再放旧的(失败时不动旧列表)
  c.s_tasks = arr;
  c.s_task_n = k;
  return true;
}

// 释放一个回合里的所有字符串(淘汰/收尾共用)
static void hist_free_turn(HistTurn& tn) {
  free(tn.reasoning); tn.reasoning = nullptr;   // 两条路径共用(free(nullptr) 安全)
  if (tn.chat) { free(tn.text); tn.text = nullptr; return; }
  for (int i = 0; i < tn.ncall; i++) {
    free(tn.calls[i].strbuf);   // id/name/args/result 同块分配, 一次释放
    tn.calls[i] = HistCall();
  }
  tn.ncall = 0;
}

// 开一个新回合(满则淘汰最旧**整回合** —— 永不出现孤儿 tool)。返回其指针供追加调用。
static HistTurn* hist_new_turn(RoundCtx& c) {
  if (!c.hist) return nullptr;
  if (c.hist_n >= AI_HIST_MAX_TURNS) {   // 兜底淘汰最旧整回合(正常路径应由 compact 先清)
    if (c.hist[0].chat && !c.hist[0].origin && c.hist[0].text) {  // 任务起点的目标不提醒(原话常驻目标消息)
      snprintf(c.task_remind, sizeof(c.task_remind),
               "较早的一条用户消息即将被历史丢弃; 若它表达了新任务/新目标而你还未用 goal 的 set 更新, 请现在更新。");
      utf8_clamp_tail(c.task_remind);
    }
    hist_free_turn(c.hist[0]);
    memmove(c.hist, c.hist + 1, (AI_HIST_MAX_TURNS - 1) * sizeof(HistTurn));
    memset(&c.hist[AI_HIST_MAX_TURNS - 1], 0, sizeof(HistTurn));
    c.hist_n = AI_HIST_MAX_TURNS - 1;
  }
  HistTurn& tn = c.hist[c.hist_n];
  memset(&tn, 0, sizeof(tn));
  c.hist_n++;
  tn.img_tag = ++c.img_tag_n;
  return &tn;
}

// 追加一条用户消息(独立 user 消息, 不参与 tool 配对)。origin=任务起点的用户目标。
static void hist_add_chat(RoundCtx& c, const char* text, bool origin = false) {
  HistTurn* tn = hist_new_turn(c);
  if (!tn) return;
  tn->chat = true;
  tn->origin = origin;
  tn->text = ps_dup(text);
}

// 向当前回合追加一次工具调用(id/name/args/result 拷进**同一块** PSRAM; 图只存指针与张数)
static void hist_add_call(RoundCtx& c, const char* id, const char* name, const char* args,
                          const char* result) {
  if (!c.hist || c.hist_n <= 0) return;
  HistTurn& tn = c.hist[c.hist_n - 1];
  if (tn.chat || tn.ncall >= AI_HIST_CALL_MAX) return;
  const char* s0 = id ? id : "";
  const char* s1 = name ? name : "";
  const char* s2 = args ? args : "{}";
  const char* s3 = result ? result : "";
  size_t n0 = strlen(s0) + 1, n1 = strlen(s1) + 1, n2 = strlen(s2) + 1, n3 = strlen(s3) + 1;
  char* buf = (char*)heap_caps_malloc(n0 + n1 + n2 + n3, MALLOC_CAP_SPIRAM);
  if (!buf) return;   // 分配失败: 不记这次调用(与 ps_dup 失败时留空指针的既有行为一致)
  HistCall& hc = tn.calls[tn.ncall];
  char* w = buf;
  hc.id = w;     memcpy(w, s0, n0); w += n0;
  hc.name = w;   memcpy(w, s1, n1); w += n1;
  hc.args = w;   memcpy(w, s2, n2); w += n2;
  hc.result = w; memcpy(w, s3, n3);
  hc.strbuf = buf;
  tn.ncall++;
}

// compact 落地: 释放除"本回合"外的全部回合, 在最前插一条 AI 自述的进展摘要。
// 目标/任务列表/任务笔记/物体记忆/车位姿都在历史环之外常驻 ⇒ 压缩只丢"AI 未落成笔记的旧结论"与旧插话。
static void compact_history(RoundCtx& c) {
  if (!c.hist || c.hist_n < 1) return;
  HistTurn cur = c.hist[c.hist_n - 1];                          // 本回合(刚记完的思考+调用+结果): 原文保留
  for (int i = 0; i < c.hist_n - 1; i++) hist_free_turn(c.hist[i]);
  memset(&c.hist[0], 0, sizeof(HistTurn));
  c.hist[0].chat = true;
  // 摘要渲染成一条 user 消息: 加固定抬头, 免得模型把它当成用户刚下的新要求。
  // +256 是给抬头留的余量: 摘要满 511B 时, 余量不足会把摘要尾部(甚至半个汉字)截掉, 云端会以
  // "invalid unicode code point" 400 拒整份请求(抬头约 215B, 511+215 < 512+256)。
  char s[sizeof(c.compact_sum) + 256];
  snprintf(s, sizeof(s),
           "【历史已压缩】以下是你自己写的此前进展摘要(更早的对话已清空; 目标/任务列表/任务笔记/物体记忆/车位姿仍在; 可回看的旧画面已清空, 要看东西请新拍):\n%s",
           c.compact_sum);
  c.hist[0].text = ps_dup(s);
  c.hist[1] = cur;                                               // 裸指针所有权随之转移
  c.hist_n = 2;
  c.compact_sum[0] = 0;
  // 更早的工具调用已随历史清掉, 那些画面(实景帧环 + 用户参考图)再也无从对应 ⇒ 一并作废: 否则
  // 「当前可查看图片」里会列出一串 AI 自己认不出的编号, 它只能对着空编号凭空猜。编号本身继续递增
  // (不重置, 免得新图号与摘要里提到的旧号撞车)。缓冲此处只摘号、不释放 —— 同回合 look 可能刚把
  // look_set 指进这些缓冲, 现在 free 会留悬空指针; 帧环槽位本就复用, 用户图则随任务收尾统一释放。
  for (int i = 0; i < AI_PREV_SLOTS; i++) { c.prev_len[i] = 0; c.prev_id[i] = 0; }
  for (int i = 0; i < AI_EDITED_SLOTS; i++) { c.ed_len[i] = 0; c.ed_num[i] = 0; }
  c.img_purged = true;   // 下回合在状态块里点名(那正是 AI 会去引用旧编号的时候)
  ai::logf("[ai] 历史已压缩, 保留摘要与本回合共 2 条; 可回看画面已作废");
}

// utf8_clamp_tail 已提到 src/core/utf8.h(ai_client / ai_mem 也要用, 否则各写各的必漏 —— 插话缓冲与
// 物体名两处就是这么漏掉 clamp 的)。这里只 include, 调用点写法不变。

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
  utf8_clamp_tail(buf);   // 放不下时 snprintf 会切在半个汉字上 → 回退到边界(否则喂给模型的是个 '?')
}

// ---------------- 结果文本构建 ----------------
static String build_feedback(unsigned long id, const JsonDocument& cmd) {
  JsonDocument out(&g_js_alloc);
  out["type"] = "ai_result";
  if (id) out["id"] = (long)id;
  JsonObject p = out["params"].to<JsonObject>();
  const char* reason = cmd["reason"] | "";
  if (reason[0]) p["reason"] = reason;   // 没话时不留空字段(动作回执/终态推送不带模型措辞)
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

// 推一条「工具使用」提示到手机(author 显示为 AI工具)。与 `/log` 开关无关: AI 往往闷头干完才 say,
// 不开日志时手机上看不到任何执行轨迹; 这条给它一个始终可见的进度行(如 "car spin_left 15°")。
// 复用结果队列与任务回传通道, 消息形状 {type:"ai_tool",params:{text}}。
static void notify_tool(RoundCtx& c, const char* text) {
  if (!text || !text[0]) return;
  JsonDocument d(&g_js_alloc);
  d["type"] = "ai_tool";
  d["params"]["text"] = text;
  // 栈缓冲序列化: text 是 ≤200B 的进度行(见各调用点的 char nb[80..200]), 512B 含转义有充分余量。
  // 免掉 String 那次内部堆分配 —— enqueue_result 反正立刻把文本拷进自己的那一块里。
  char buf[512];
  serializeJson(d, buf, sizeof(buf));
  ai::enqueue_result(buf, c.t.fn, c.t.ctx);
}

// 推一份「任务面板」快照给手机(聊天区顶部悬浮任务条的数据源):
// {type:"ai_task",params:{state,round,goal?,note?,tasks:[{name,done}]}}。
// 与 ai_tool 同一条结果队列, 但**只在列表/目标/终态变化时推** —— 任务列表只进过模型上下文和日志,
// 手机端此前拿不到; 逐回合推则会把深度 8 的结果队列挤满, 也把面板刷成流水。
// state: running | wait(等你输入) | done | fail | abort —— 手机端据此定标题色与"结束后是否保留"。
// ⚠️ 纯 BLE(无 WS)时这条会撞 GATT 通知的长度上限被截断 —— 面板那时不更新, 任务本身照跑(与 ai_tool 同命)。
static void notify_tasks(RoundCtx& c, const char* state) {
  JsonDocument d(&g_js_alloc);
  d["type"] = "ai_task";
  d["params"]["state"] = state;
  d["params"]["round"] = (long)c.steps;
  if (ps_str(c.goal_now)[0]) d["params"]["goal"] = c.goal_now;
  if (ps_str(c.task_note)[0]) d["params"]["note"] = c.task_note;
  JsonArray ta = d["params"]["tasks"].to<JsonArray>();
  for (int i = 0; i < c.s_task_n; i++) {
    JsonObject it = ta.add<JsonObject>();
    it["name"] = c.s_tasks[i].name;
    it["done"] = c.s_tasks[i].done;
  }
  // 按实际长度在 PSRAM 上开临时块序列化, 不用定长栈缓冲: 长度随列表/笔记增长, 固定缓冲迟早会切在
  // 半个字或半个 JSON 上(前者变 '?', 后者直接让手机端 **整条丢弃**, 表现为面板忽然不更新)。
  // enqueue_result 内部立刻拷走一份, 所以这里序列化完就可以 free。
  size_t need = measureJson(d) + 1;
  char* buf = (char*)heap_caps_malloc(need, MALLOC_CAP_SPIRAM);
  if (!buf) return;
  if (serializeJson(d, buf, need) > 0) ai::enqueue_result(buf, c.t.fn, c.t.ctx);
  free(buf);
}

// 兜底 stop(任务终结出口统一解析一次)。stop_mode 由打断方写入: None=手动 move/stop 接管(不补停);
// Wheels=手动 arm(只停轮子); All=其余。仅当存在持续指令残留才补。
static void resolve_stop() {
  if (ai::stop_mode() == (int)ai::StopMode::None) return;
  if (!g_last_continuous) return;
  // Wheels 模式只对轮子持续残留停轮子; 臂持续残留(或未知)必须全停。
  const char* scope = (ai::stop_mode() == (int)ai::StopMode::Wheels && g_last_cont_type == 1) ? "wheels" : "all";
  JsonDocument d(&g_js_alloc); d["scope"] = scope;   // d 即 stop 的 params 对象
  exec::act("stop", d.as<JsonObjectConst>());
  blog::logf(blog::AI, "兜底 stop scope=%s", scope);
  g_last_continuous = false;
}

// ---------------- 单轮驱动出口 ----------------
enum class PrepR  : uint8_t { Ok, SkipRound, ExitTask, Interrupted };
enum class AttR   : uint8_t { Got, Retry, Break };
enum class RoundR : uint8_t { NextRound, ExitTask };

// `car` 工具的落地(move/arm/light 分组通道 → 执行 / 反馈): 执行动作并把"这一回合发生了什么"写成结果文本(out)。
// 结果文本会作为该次 tool 调用的返回值回喂给模型 —— 这是它唯一的执行反馈通道。
static void land_car(RoundCtx& c, JsonDocument& cmdD, char* out, size_t out_cap);

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
  // 当前任务目标(独立 user 消息展示; 可被 goal.set 热替换)。按原话长度分配, 不设上限;
  // 分配失败就留空(手机端标题退化成"AI 任务"), 不影响任务本身继续跑。
  if (!ps_set(c.goal_now, c.t.text))
    ai::logf("[ai] 警告: 目标文本分配失败(PSRAM 不足), 状态块里将没有目标");
  // 历史环表(PSRAM): 一回合一条, 内含 1~AI_HIST_CALL_MAX 个工具调用。放 PSRAM 是因为它比 RoundCtx
  // 本身还大(数十回合 × 数个调用), 而 RoundCtx 按值建在 worker 那 16KB 栈上, 塞不下。
  c.hist = (HistTurn*)heap_caps_malloc(AI_HIST_MAX_TURNS * sizeof(HistTurn), MALLOC_CAP_SPIRAM);
  if (c.hist) {
    memset(c.hist, 0, AI_HIST_MAX_TURNS * sizeof(HistTurn));
    // 用户最初的发言也进历史环(一条 user 消息): 目标消息每轮重发之外, 对话流里也留一份原话。
    if (c.t.text && c.t.text[0]) hist_add_chat(c, c.t.text, /*origin=*/true);
  } else {
    ai::logf("[ai] 历史环分配失败(PSRAM 不足): 本任务无历史, 模型看不到自己的前几回合");
  }
  // 用户参考图快照(供整轮任务复用, 避免中途被覆盖): 暂存里最近 ≤3 张有效图各留一份独立 PSRAM
  // 副本, 并与实景帧统一编号(各槽 ed_num), 供 look 按编号回看。
  c.ed_n = ai::edited_snapshot(c.ed_img, c.ed_len, AI_EDITED_SLOTS);
  for (int i = 0; i < AI_EDITED_SLOTS; i++) if (c.ed_img[i] && c.ed_len[i]) c.ed_num[i] = img_next_id();
  // 任务起点先挂一张空面板: 此刻只有目标(用户原话), 列表等 AI 调 task 补。手机端由此知道"有任务在跑"。
  notify_tasks(c, "running");
}

// ---------------- 回合入口检查(中断 / 掉线 / 等待态) ----------------
static PrepR prep_gate(RoundCtx& c) {
  // 中止检查(代际号变化即本任务作废); 兜底停统一在任务出口解析。
  if (c.t.generation != ai::generation()) { blog::logf(blog::AI, "被新目标/手动中断"); c.interrupted = true; return PrepR::Interrupted; }
  if (!net::is_connected()) { c.fail = "WiFi 掉线"; return PrepR::ExitTask; }
  if (cfg::ai_key().isEmpty()) { c.fail = "未配置 AI Key"; return PrepR::ExitTask; }

  // finish result="wait" 的等待态: 不取帧、不发云端请求, 只等用户回复(→继续执行)或超时(→告知"未回复"再继续)。
  // 用户改发新目标 / 手动接管 / ai_cancel 都会换代际号, 由上面的中断检查与本块内的检查兜住。
  bool was_wait = c.wait_user;
  while (c.wait_user) {
    if (c.t.generation != ai::generation()) { c.interrupted = true; break; }
    bool replied = ai::chat_pending();   // 只看不动: 回复留给正常流程消费成 user 消息
    if (replied) { c.wait_user = false; break; }
    if ((uint64_t)(esp_timer_get_time() / 1000) >= c.wait_until) {
      snprintf(c.wait_note, sizeof(c.wait_note),
               "你上回合请求中止并等待用户输入, 已超时(用户未回复); 请继续执行原任务");
      utf8_clamp_tail(c.wait_note);
      blog::logf(blog::AI, "等待用户输入超时(用户未回复), 告知 AI 继续执行");
      c.wait_user = false;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(AI_WAIT_USER_POLL_MS));
  }
  // 等待结束(用户回话 / 超时): 面板从"等你输入"回到"进行中" —— 否则它会一直挂着等你, 而 AI 其实已继续。
  if (was_wait && !c.wait_user && !c.interrupted) notify_tasks(c, "running");
  if (c.interrupted) { blog::logf(blog::AI, "等待用户输入期间被新目标/手动中断"); return PrepR::Interrupted; }
  return PrepR::Ok;
}

// ---------------- 取一帧进 cur(首帧与 look 共用) ----------------
// 先等车轮/机械臂停稳再抓帧; 取帧失败退避重试(并发推流偶发占满缓冲), 连续失败计入 net_fail。
// zoom=true 走高清真拍 + 中央固定框放大(裁框绝对, 不按倍数缩放)。note 回填这张图的性质, 供历史占位与结果文本;
// 放大失败会**退回全幅并在 note 里说明**(不能让 AI 以为在看放大图)。out_zoomed 回填"这张到底是不是放大图"
// (调用方据此写 pend_zoomed —— 失败的放大若被当成放大图, observe 的坐标换算会整块错位)。
// 返回 false = 本回合没取到图。
static bool fetch_image(RoundCtx& c, bool zoom, char* note, size_t note_cap, bool* out_zoomed) {
  if (out_zoomed) *out_zoomed = false;
  // 等车轮/机械臂停稳**再**抓帧: 放大走的高清真拍同样要先停稳(否则高速运动下拍出来是糊的),
  // 故停稳与"仍在动"判定提到放大分支之前, 两条路径共用。
  ai::settle_wheels(c.t.generation, AI_SETTLE_MAX_MS);
  ai::settle_arm(c.t.generation, AI_SETTLE_MAX_MS);
  c.frame_moving = exec::wheels_moving();
  bool zoom_failed = false;
  if (zoom) {
    int hok = 0;
    camera_fb_t* hfb = cam::request_hires(AI_HIRES_FRAME, &hok);
    if (!c.cur) c.cur = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    size_t zl = 0;
    // 用 TJpgDec 部分解码裁中央的 crop_center_jpg（固定 (0.25,0.25)-(0.75,0.75)），直接输出到 cur
    // （统一由 cur 承载"最近一张图", 环做回看）：
    // workbuf/RGB 全走 PSRAM，不碰内部堆/DMA —— 内部堆被软解吃穿会导致整板卡死。
    bool ok = hfb && hok && c.cur &&
              magnify::crop_center_jpg(hfb->buf, cam::jpeg_len(hfb), hfb->width, hfb->height,
                                       c.cur, AI_EDITED_IMG_MAX, &zl,
                                       AI_HIRES_OUT_W, AI_HIRES_OUT_H, AI_ZOOM_QUALITY) && zl > 0;
    if (hfb) cam::return_frame(hfb);   // 高清帧用完归还（request_hires 内部已切回 VGA 放锁）
    if (ok) {
      c.cur_len = zl;
      c.cur_id = img_next_id();
      if (note && note_cap) snprintf(note, note_cap, "放大%.1f× 全幅(0.25,0.25)-(0.75,0.75)", (double)AI_ZOOM_DEF);
      if (out_zoomed) *out_zoomed = true;
      ai::logf("[放大镜] 高清中央框(0.25,0.25)-(0.75,0.75) → %uKB %dms",
               (unsigned)(zl / 1024), magnify::last_cost_ms());
      return true;
    }
    ai::logf("[放大镜] 高清放大失败(%s), 退回全幅",
             hfb ? "裁图失败" : (hok ? "取帧失败" : "切分辨率失败"));
    zoom_failed = true;   // 下面按全幅出图, 但要在 note 里讲清"这不是放大图"
  }
  camera_fb_t* fb = nullptr;
  for (int fr = 0; fr < AI_FRAME_RETRY && !fb; fr++) {
    fb = cam::grab();
    if (!fb && fr < AI_FRAME_RETRY - 1) vTaskDelay(pdMS_TO_TICKS(80));
  }
  if (!fb) {
    if (++c.net_fail >= AI_MAX_NET_FAIL) { c.fail = "持续取帧失败(摄像头/缓冲异常)"; return false; }
    ai::logf("[ai] 取帧失败(第%d次), 稍后重试", c.net_fail);
    vTaskDelay(pdMS_TO_TICKS(300 * c.net_fail));
    return false;
  }
  // 必须用 cam::jpeg_len 而非 fb->len: 驱动会把 len 报大(缓冲里可能拼了多帧), 虚高的 len 会被原样
  // base64 进请求体、压垮 WiFi 发送路径(详见 camera.cpp 的 jpeg_len 说明)。
  size_t fl = cam::jpeg_len(fb);
  bool copied = false;
  if (fl > 0 && fl <= AI_EDITED_IMG_MAX) {
    if (!c.cur) c.cur = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (c.cur) { memcpy(c.cur, fb->buf, fl); c.cur_len = fl; c.cur_id = img_next_id(); copied = true; }
  }
  // 抓帧后立即归还相机缓冲: AI 的 HTTPS 慢则数秒, 期间一直占着 fb 会把 fb_count=2 的缓冲池耗尽、
  // 饿死并行推流; 帧数据后续一律用这份 PSRAM 副本。
  cam::return_frame(fb);
  if (!copied) {
    c.cur_len = 0; c.cur_id = 0;
    if (++c.net_fail >= AI_MAX_NET_FAIL) { c.fail = "帧过大或拷贝失败"; return false; }
    return false;
  }
  if (note && note_cap)
    snprintf(note, note_cap, zoom_failed ? "全幅(放大失败, 按全幅读)" : "全幅");
  return true;
}

// 组包前提交"图基准": 模型这回合报的 px/py 指的是它**上一条看到**的图(look 产图后写 pend, 下一回合成基准)。
static void commit_pend(RoundCtx& c) {
  if (!c.pend_valid) return;
  c.sent_zoomed = c.pend_zoomed;
  c.sent_x0 = c.pend_x0; c.sent_y0 = c.pend_y0; c.sent_x1 = c.pend_x1; c.sent_y1 = c.pend_y1;
  // 位姿快照随图一起提交: 之后无论车走到哪, observe 都按"拍这张图时"的位姿解算
  c.sent_car_x = c.pend_car_x; c.sent_car_y = c.pend_car_y; c.sent_car_hd = c.pend_car_hd;
  c.pend_valid = false;
}

// 把刚发出去的实景(cur)滚动进先前帧环: 最旧的槽被丢弃(缓冲回收给最新帧, 免得反复分配)。
// 只在**请求确实带了这张图**之后调用 —— 没发出去的帧模型没见过, 不该占环。
static void prev_roll(RoundCtx& c) {
  if (!c.cur || c.cur_len == 0 || c.cur_len > AI_EDITED_IMG_MAX || !c.cur_id) return;
  uint8_t* slot = c.prev[AI_PREV_SLOTS - 1];   // 最旧槽的缓冲拿来装最新帧
  if (!slot) {
    slot = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
    if (!slot) return;                          // 分配失败: 本帧不进环, 其余槽保持原样
  }
  for (int i = AI_PREV_SLOTS - 1; i > 0; i--) {
    c.prev[i] = c.prev[i - 1]; c.prev_len[i] = c.prev_len[i - 1]; c.prev_id[i] = c.prev_id[i - 1];
  }
  c.prev[0] = slot;
  memcpy(slot, c.cur, c.cur_len);
  c.prev_len[0] = c.cur_len;
  c.prev_id[0] = c.cur_id;
}

// 每轮现拼的任务状态块(定义在 render_tasks 之后, 此处先声明)
static void build_state_block(RoundCtx& c, PsaBuf& b);

// ---------------- 组包 + 云端 + 取工具调用 ----------------
// 一次请求: 收用户消息 → 定案"哪张图带字节" → 组包 → POST → 解出 tool_calls。
// 返回 Got=拿到工具调用; Retry=没拿到(网络/解析/模型没调工具), 调用方退避重试; Break=任务终止。
static AttR round_attempt(RoundCtx& c) {
  // 用户消息(一次性消费): 作为独立 user 消息进历史。wait 等待期的回复走的也是这里, 一律当普通
  // 用户消息, 不额外标注成"插话" —— 那会让 AI 以为用户在打岔, 而不是在回答它刚提出的问题。
  char chat_now[256] = {0};
  if (ai::chat_take(chat_now, sizeof(chat_now))) utf8_clamp_tail(chat_now);
  if (chat_now[0]) {
    hist_add_chat(c, chat_now);
    ai::logf("[ai] 用户消息已喂给 AI: %.96s", chat_now);   // 落一条账: "AI 以为用户说了什么"必须可回溯
  }
  // 图只给"最新且还没发出去"的那组字节: 其余条目按 note 渲染成占位说明(老图不重复携带, 省 token)。
  bool carried = false;
  bool with_prev = false;   // 本回合带了不止一张图(多图对照): 只用于抓帧留档标注
  int owner_ti = -1;
  for (int ti = 0; ti < c.hist_n; ti++) {
    HistTurn& tn = c.hist[ti];
    if (tn.chat) continue;
    bool owner = (tn.img_tag == c.img_owner) && !c.img_sent && c.look_cnt > 0;
    for (int ci = 0; ci < tn.ncall; ci++) {
      HistCall& hc = tn.calls[ci];
      hc.imgs[0] = ImgRef(); hc.imgs[1] = ImgRef();
      if (!hc.img_n || !owner) continue;
      // 清单顺序 = 给图顺序(新拍 → prev → user)
      hc.imgs[0] = ImgRef{ c.look_set[0].p, c.look_set[0].n };
      if (hc.img_n >= 2 && c.look_set[1].p) { hc.imgs[1] = ImgRef{ c.look_set[1].p, c.look_set[1].n }; with_prev = true; }
      carried = true;
      owner_ti = ti;
    }
  }
  if (c.frame && c.frame_len > 0) carried = true;   // 尾部 user 画面(首轮)也算"发出去一张"

  PieceList body;
  PsaBuf state;
  build_state_block(c, state);   // 目标/列表/笔记: 每轮现拼, 挂最新一条工具结果尾部(不落历史)
  BodyReq br;
  br.state_block = state.p ? state.p : nullptr;
  br.turns = c.hist;
  br.turn_n = c.hist_n;
  br.tail_hint = c.pend_hint[0] ? c.pend_hint : nullptr;   // 一次性提示: 挂最新一条 tool 结果/尾部画面
  br.frame = { c.frame, c.frame_len };
  br.use_frame = c.frame_note[0] != 0;
  br.frame_note = c.frame_note;
  build_body(body, br);
  c.pend_hint[0] = 0;   // 一次性消费
  c.task_remind[0] = 0; // 已随状态块挂出
  if (!body.ok) { c.fail = "组装请求 body 失败"; return AttR::Break; }

  PsaBuf resp;   // 响应正文收进 PSRAM(内部堆才是瓶颈; 见 ai_http.h 的说明)
  bool http_ok = false;
  for (int nr = 0; nr < 3 && !http_ok; nr++) {   // 网络失败指数退避重试(任务串行, 代价可控)
    if (ai::http_post(cfg::ai_url().c_str(), cfg::ai_key().c_str(), body, resp, c.t.generation)) { http_ok = true; break; }
    if (c.t.generation != ai::generation()) { c.done = true; c.interrupted = true; break; }  // 被中止, 静默作废
    if (ai::http_last_status() >= 400 && ai::http_last_status() < 500) {  // 4xx 重发同 body 必然再拒, 快速失败
      int st = ai::http_last_status();
      const char* eb = ai::http_last_error();   // 带上云端给的具体原因, 别只说"疑似参数或限流"
      if (eb && eb[0]) snprintf(c.err_buf, sizeof(c.err_buf), "云端拒绝(%d): %.110s", st, eb);
      else             snprintf(c.err_buf, sizeof(c.err_buf), "云端拒绝(%d), 响应体为空(疑似限流或网关拒绝)", st);
      c.fail = c.err_buf;
      break;
    }
    if (nr < 2) { vTaskDelay(pdMS_TO_TICKS(500 << nr)); blog::logf(blog::AI, "网络失败重试 %d", nr + 1); }
  }
  if (c.done) return AttR::Break;
  if (!http_ok) { if (!c.fail) c.fail = "AI 请求失败"; return AttR::Break; }
  if (c.t.generation != ai::generation()) { c.done = true; c.interrupted = true; return AttR::Break; }  // 在途结果作废

  // 图已随请求发出: 新拍的实景滚进先前帧环(供之后 look(image=[N]) 按编号回看), 且不再重复携带字节。
  if (carried) {
    if (c.cur_len > 0 && (c.frame || c.look_live)) prev_roll(c);   // 首帧那条也滚一次(尾部画面发完就清)
    c.img_sent = true;
    c.look_cnt = 0; c.look_live = false;   // 清单已消费
  }
  c.frame = nullptr; c.frame_len = 0; c.frame_note[0] = 0;   // 尾部画面只发一次
  // 抓帧留档(调试旁路, 见 ai_dump.h): 只在**真正组出带图那条消息**时留一份供事后复盘, 关着时零开销。
  if (carried && c.cur && c.cur_len > 0) ai::dump_push(c.cur, c.cur_len, with_prev);

  c.calls.clear();
  bool body_broken = false;
  int nc = ai::extract_tool_calls(resp, c.calls, &body_broken);
  if (nc <= 0) {
    // 无工具调用: 模型只回了文本 / 思考把预算吃光被 length 截断 / 传输层截断。打原始片段便于定位。
    blog::logf(blog::AI, "无工具调用, 原始(前120B): %.120s", resp.c_str());
    c.fail = "AI 未调用工具(只回了文本或被 length 截断)";
    if (body_broken) ai::http_stop();  // 传输层截断/残留: 弃用复用连接, 下次全新握手防污染
    return AttR::Retry;
  }
  c.got = true;
  return AttR::Got;
}

// ---- observe: 收下数组里的一条观测 ----
// 名字必填; 位置只认屏幕像素 px/py(单应解算)。模型自报的角度/距离一律不记 —— 它没有测距能力, 角度
// 报偏了会把车指挥到错方向, 缺 px/py 时宁可判"没记成"并回告。⚠️ obs_warn 是单条缓冲: 一轮报多个物体
// 时只留**第一条**回告, 免得后面的覆盖前面的。
static void land_observe_one(RoundCtx& c, JsonObjectConst ob) {
  const char* nm = ob["name"] | "";
  // 只要 AI 提交 observe(写明了名字+位置)就说明它这一帧看到了该物体、要记录 → 一律视为可见;
  // 它不对某物 observe, 该物就不会被刷新(新旧由 mem 的过期轮数体现)。
  if (!nm[0]) {
    // name 必填: 无名观测无法归属到任何物体, 只能丢弃。明确回告(否则 AI 以为已记住)
    if (!c.obs_warn[0]) snprintf(c.obs_warn, sizeof(c.obs_warn),
                                 "上轮 observe 没给 name, 该观测已被忽略(记忆里没有它); 每次 observe 都要带 name");
    return;
  }
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
  bool rec = false;
  if (has_px && has_py) {
    float right = 0.0f, fwd = 0.0f;
    // 位姿用它**看的那张图拍摄时**的快照, 不是此刻: 车若在"看图"与"报坐标"之间动过, 用实时位姿会把坐标整体平移。
    ai::CarPose pose{ c.sent_car_x, c.sent_car_y, c.sent_car_hd };
    rec = ai::mem_observe_xy(nm, true, px, py, &right, &fwd, &pose);
    if (rec) {
      // 当场回执解算出的车头系读数(省去 AI 再调一次 mem 去看): 带"左/右、前/后"方向词, 免它猜正负号。
      char one[80];
      snprintf(one, sizeof(one), "%s%s (%s%.0f, %s%.0f)", c.obs_echo[0] ? ", " : "", nm,
               right >= 0 ? "右" : "左", right >= 0 ? right : -right,
               fwd >= 0 ? "前" : "后", fwd >= 0 ? fwd : -fwd);
      size_t used = strlen(c.obs_echo);
      if (used + strlen(one) < sizeof(c.obs_echo)) memcpy(c.obs_echo + used, one, strlen(one) + 1);
    } else {
      ai::logf("[ai] 观测「%s」像素(%.2f,%.2f)不可用(越界/解算失败), 未记录", nm, px, py);
    }
  } else {
    ai::logf("[ai] 观测「%s」未给 px/py, 未记录位置", nm);
    if (!c.obs_warn[0]) {
      snprintf(c.obs_warn, sizeof(c.obs_warn),
               "上轮 observe 的「%s」没给 px/py, 位置未记录; px/py 填目标底部中心在画面上的坐标(0~1)后再对准",
               nm);
      utf8_clamp_tail(c.obs_warn);   // 名字顶到上限时会把尾巴切在半个字上
    }
  }
  if (!rec && !c.obs_warn[0]) {  // 可见却没记成: 回告, 否则 AI 以为已锁定而实际记忆为空
    snprintf(c.obs_warn, sizeof(c.obs_warn),
             "上轮 observe 的「%s」没记进记忆(px/py 越界或解算失败), 位置仍未知; 请据当前画面重新确认后再 observe",
             nm);
    utf8_clamp_tail(c.obs_warn);
  }
}

// 渲染任务列表: "任务列表: 1.出门[完成] 2.右转[未完成] " (空列表则不写)。
// 直接写进状态块缓冲, 不再过定长中转 —— 中转一满就会**静默丢掉后面的项**, 模型只看到自己列表的
// 前半截, 却以为整份都在, 后面的步骤就不做了(名字本身在写入时已做过边界回退, 这里可整段直拼)。
static void render_tasks(RoundCtx& c, PsaBuf& b) {
  if (c.s_task_n <= 0) return;
  b.put("任务列表: ");
  for (int ti = 0; ti < c.s_task_n; ti++) {
    char idx[8];
    snprintf(idx, sizeof(idx), "%d.", ti + 1);
    b.put(idx);
    b.put(c.s_tasks[ti].name);
    b.put(c.s_tasks[ti].done ? "[完成] " : "[未完成] ");
  }
}

// 每轮现拼的"任务状态块"(车/臂基础状态 + 目标/列表/笔记/提醒): 组包时挂到本轮最后一条工具结果尾部。
// ⚠️ 只进请求、不写历史 —— 否则会像旧的 car 结果那样逐轮刻一份快照、在历史里重复。
static void build_state_block(RoundCtx& c, PsaBuf& b) {
  // 车/机械臂基础状态: 每轮现拼, 供 AI 随时查看; 动作类工具的返回值只留"这次动作发生了什么", 不再附带姿态。
  char st[256];
  const char* sp = exec::read_state(st, sizeof(st)) ? st : "";
  if (sp[0]) {
    if (sp == st) strip_pwm_hints(st);   // 剥掉人工校准用的舵机 PWM, 只留 AI 能用的 cm 值
    b.put("\n"); b.put(st);
  }
  bool has_origin = false;
  for (int i = 0; i < c.hist_n; i++) if (c.hist[i].origin) { has_origin = true; break; }
  b.put("\n任务目标: ");
  if (!c.goal_explicit && has_origin) b.put("暂未设置, 请根据用户发言更新");
  else b.put(ps_str(c.goal_now));
  if (c.t.ann && c.t.ann[0]) { b.put(" (操作者标注: "); b.put(c.t.ann); b.put(")"); }
  if (c.s_task_n > 0) { b.put("\n"); render_tasks(c, b); }
  if (ps_str(c.task_note)[0]) { b.put("\n任务笔记: "); b.put(c.task_note); }
  if (c.task_remind[0]) { b.put("\n注意: "); b.put(c.task_remind); }
  // 提示与工具无关, 统一挂在这里(动作类工具的结果只讲"这次动作发生了什么")。
  if (c.stall_hint) {
    b.put("\n注意: 连续几回合重复同一批动作、没有进展时可以换个角度重新观察目标");
    c.stall_hint = false;
  }
  if (c.wait_note[0]) { b.put("\n注意: "); b.put(c.wait_note); c.wait_note[0] = 0; }
  if (c.img_purged) {   // compact 之后: 旧编号全作废, 此刻正是 AI 想引用它们的时候
    b.put("\n注意: 可回看的画面(含用户参考图)已随历史压缩清空, 不要再引用更早的图片编号; 需要看图请重新 look");
    c.img_purged = false;
  }
  // 历史达软上限: 每轮催一次, 直到 AI 调 compact(硬上限兜底前它会先被提醒很多轮)。
  if (c.hist_n >= AI_HIST_SOFT_TURNS) {
    char hb[160];
    snprintf(hb, sizeof(hb),
             "历史上下文已 %d 轮(建议不超过 %d 轮), 请尽快调用 compact 用一段摘要压缩此前对话",
             c.hist_n, AI_HIST_SOFT_TURNS);
    b.put("\n注意: "); b.put(hb);
  }
  // 末尾固定一句中文约束: 模型上轮的英文 reasoning 会**原样回喂**历史(见 ai_prompt.cpp 回传 reasoning_content),
  // 形成语言自我强化, system 里那条弱约束压不住。贴在本轮末尾(recency 最强)才有分量。
  b.put("\n注意: 思考过程与发言都使用简体中文");
}

// 终态映射(goal.finish 的唯一出口): done→收尾; wait→等用户输入; fail→失败收尾。
static void apply_end(RoundCtx& c, const char* fin_v) {
  if (!fin_v || !fin_v[0]) return;
  JsonDocument st(&g_js_alloc); st["scope"] = "all";
  exec::act("stop", st.as<JsonObjectConst>());   // 终态前先急停, 不留残留动作
  if (!strcmp(fin_v, "wait")) {
    c.wait_user = true;   // 下一回合起进入等待态(不取帧/不发请求), 见 prep_gate 头部
    c.wait_until = (uint64_t)(esp_timer_get_time() / 1000) + AI_WAIT_USER_MS;
    blog::logf(blog::AI, "AI 请求中止, 等待用户输入(最多%us)", (unsigned)(AI_WAIT_USER_MS / 1000));
    // 等待态必须让用户知道(否则他不知道 AI 在等他回话) —— 文案由程序给, 不依赖模型调没调 say。
    char wt[96];
    snprintf(wt, sizeof(wt), "AI 已中止, 等待你的输入(最多%us; 直接发消息即可继续)",
             (unsigned)(AI_WAIT_USER_MS / 1000));
    JsonDocument f(&g_js_alloc); f["reason"] = wt;
    String fb = build_feedback(c.t.id, f);
    ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
    notify_tasks(c, "wait");   // 面板转"等你输入": 不提示的话用户不知道 AI 在等他回话
  } else if (!strcmp(fin_v, "fail")) {
    c.fail = "AI 判定执行失败";
    c.done = true;
    blog::logf(blog::AI, "AI 判定执行失败");
  } else {   // done
    c.done = true;
    // 不置 sent_done: 终态 done 统一由 round_task_finish 补发, 这样"完成"与"失败"走同一条出口。
    blog::logf(blog::AI, "AI 判定任务完成");
  }
}

// 追加一段"[工具] 执行结果"到 car 回执(段间 " | ")。detail 为空 = 该键本轮没调用/没产生结果, 不追加。
static void echo_seg(PsaBuf& b, int* n, const char* key, const char* detail) {
  if (!detail[0]) return;
  b.put(" | ["); b.put(key); b.put("] "); b.put(detail);
  (*n)++;
}

// ---------------- 落地(car 工具) ----------------
static void land_car(RoundCtx& c, JsonDocument& cmdD, char* out, size_t out_cap) {
  // ---- 分组通道提取: move/arm/light 可选, 缺席=该子系统不动(取画面/放大已归 look 工具) ----
  // move 内部单选 throttle/spin/approach; arm 内部单选各固定动作; light 可任意组合。
  JsonObjectConst mv = cmdD["move"].is<JsonObject>() ? cmdD["move"].as<JsonObjectConst>() : JsonObjectConst();
  JsonObjectConst ar = cmdD["arm"].is<JsonObject>()  ? cmdD["arm"].as<JsonObjectConst>()  : JsonObjectConst();
  bool has_move  = cmdD["move"].is<JsonObject>();
  bool has_arm   = cmdD["arm"].is<JsonObject>();
  bool has_light = cmdD["light"].is<JsonObject>();
  bool has_any   = has_move || has_arm || has_light;
  // 逐工具执行回执: 每个键把自己的"这次结果"写进对应段, 末尾拼成一条(见下方"结果文本")。
  int  seg_n = 0;                     // 有结果的段数(0 = 本轮纯空转)
  char emv[80] = {0}, earm[48] = {0}, elight[40] = {0};
  const char* mtype = mv["type"] | "";
  bool is_mv = has_move && !strcmp(mtype, "throttle");
  bool is_sp = has_move && !strcmp(mtype, "spin");
  bool is_ap = has_move && !strcmp(mtype, "approach");
  const char* act = ar["type"] | "";   // arm 子型(low/raise/fold/grasp/clip/release/pose), 无 arm 为空
  // ---- 主动作/并行辅助标记(供末尾统一落地与反馈); acted 用 RoundCtx, 勿在此重复遮蔽 ----
  bool nav_done = false;                 // approach 已执行导航
  const char* nav_tgt = mv["target"] | "";
  // ---- approach(轮子自动靠近): 本地巡航, 不点云端。与合爪同一回合时拦下(导航后车动、坐标过时) ----
  if (is_ap) {
    // ⚠️ 只拦 approach+**合爪**(grasp/clip): approach 会挪车、坐标过时, 拿旧坐标合爪必空夹。
    //   approach+**降爪**(arm low)不拦 —— 降爪落到标定固定点, 坐标过时只影响之后的横向微调。
    if (has_arm && (!strcmp(act, "grasp") || !strcmp(act, "clip"))) {
      JsonDocument cmdF(&g_js_alloc);
      cmdF["type"] = "approach";
      cmdF["reason"] = "approach 与合爪不能同一条 car 里: 先只 approach, 下一回合 look 看画面确认对齐后再夹; arm low/fold/raise/light 可与 approach 并行";
      cmdF["params"]["target"] = nav_tgt[0] ? nav_tgt : "(最近)";
      String fb = build_feedback(c.t.id, cmdF);
      ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
      snprintf(emv, sizeof(emv), "approach %s", nav_tgt[0] ? nav_tgt : "(最近)");
      snprintf(out, out_cap, "[执行结果] | [move] %s 被拦: 与合爪(grasp/clip)不能同一条 car 里 —— 先只做 approach, "
                             "下一回合 look 看画面确认是否对准, 再决定夹取。本轮其它动作也未执行。", emv);
      notify_tool(c, "car approach 被拦(与合爪冲突, 本轮未执行)");
      return;
    }
    float tx, ty;
    bool found = ai::mem_find(nav_tgt[0] ? nav_tgt : nullptr, &tx, &ty);
    if (!found) {
      ai::logf("[ai] %s", nav_tgt[0] ? "approach 目标不在记忆里(得先 observe)" : "approach 无可用目标记忆");
      snprintf(emv, sizeof(emv), "approach %s 未执行: 目标不在记忆里, 需先 observe",
               nav_tgt[0] ? nav_tgt : "(最近)");
    } else {
      ai::logf("[ai] approach 目标 全局(%.0f,%.0f)cm 停距%dcm", tx, ty, AI_APPROACH_STOP_CM);
      ai::NavR r = ai::navigate_to(c.t.generation, tx, ty, AI_APPROACH_STOP_CM);
      if (r == ai::NavR::Interrupted) { c.interrupted = true; c.done = true; return; }   // 被接管: 本任务作废
      nav_done = true;
      ai::logf("[ai] %s", r == ai::NavR::Reached ? "已自动靠近目标, 交还微操" : "靠近收敛结束");
      snprintf(emv, sizeof(emv), "approach %s", nav_tgt[0] ? nav_tgt : "(最近)");
      c.last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
    }
    c.acted = true;   // approach 算一次有意推进(空转兜底/死循环都销账)
  }
  // move/spin 的默认段长与默认角由 `parse_move` 在规范化时补写(见 `t_move.cpp`): 本文件不再改写指令。
  // ---------- 逐通道落地(move/arm/light), 汇总主动作/合并串 ----------
  char merge_str[96] = {0};          // 本轮指令回显串(死循环判据 + 结果文本 + 留档), 段间 " / "
  int  merge_n = 0;                  // 已回显几段(决定要不要补分隔符)
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
    // 回显用模型自己的词表(forward/backward), 不用内部规范形(th/t距离): AI 不必反查, 死循环自查也更准。
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
             "%s%s %dcm", merge_n++ ? " / " : "", mth < 0 ? "backward" : "forward", dc);
    snprintf(emv, sizeof(emv), "%s %dcm%s", mth < 0 ? "backward" : "forward", dc, ok ? "" : " (未落地)");
  } else if (is_sp) {
    JsonObjectConst mcv = cmdD["move"].as<JsonObjectConst>();
    bool ok = exec::act("spin", mcv);
    c.acted |= ok; any_ok |= ok; any_attempted = true;
    ai::car_update_pose("spin", mcv);
    if (!has_main) { acmd["type"] = "spin"; acmd["params"] = mcv; has_main = true; }
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
             "%s%s %d°", merge_n++ ? " / " : "", (mcv["dir"] | 0) < 0 ? "spin_left" : "spin_right",
             (mcv["angle_deg"] | 0));
    snprintf(emv, sizeof(emv), "%s %d°%s", (mcv["dir"] | 0) < 0 ? "spin_left" : "spin_right",
             (mcv["angle_deg"] | 0), ok ? "" : " (未落地)");
  }
  if (has_arm) {
    JsonObjectConst acv = cmdD["arm"].as<JsonObjectConst>();
    bool ok = false;
    if (!strcmp(act, "low"))        ok = exec::arm_low();
    else if (!strcmp(act, "raise")) ok = exec::arm_raise();
    else if (!strcmp(act, "pose"))  ok = exec::arm_pose(acv["x"] | 0.0f, acv["h"] | 0.0f);
    else {   // grasp/clip/release/fold
      JsonDocument a(&g_js_alloc); a["act"] = act;
      ok = exec::act("arm", a.as<JsonObjectConst>());
    }
    c.acted |= ok; any_ok |= ok; any_attempted = true;
    if (!has_main) { acmd["type"] = "arm"; acmd["params"] = acv; has_main = true; }
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
             "%sarm %s", merge_n++ ? " / " : "", act);
    if (!strcmp(act, "pose")) snprintf(earm, sizeof(earm), "pose (%.0f, %.0f)", acv["x"] | 0.0f, acv["h"] | 0.0f);
    else snprintf(earm, sizeof(earm), "%s", act);
    if (!ok) strncat(earm, " (未落地)", sizeof(earm) - strlen(earm) - 1);
  }
  if (has_light) {
    JsonObjectConst lcv = cmdD["light"].as<JsonObjectConst>();
    bool ok = exec::act("light", lcv);
    c.acted |= ok; any_ok |= ok;
    if (!has_main) { acmd["type"] = "light"; acmd["params"] = lcv; has_main = true; }
    snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
             "%slight %s %s", merge_n++ ? " / " : "", (lcv["kind"] | ""), (lcv["on"] | false) ? "on" : "off");
    snprintf(elight, sizeof(elight), "%s %s%s", lcv["kind"] | "", (lcv["on"] | false) ? "on" : "off",
             ok ? "" : " (未落地)");
  }
  if (nav_done && !has_main) {
    acmd["type"] = "approach"; acmd["params"]["target"] = nav_tgt[0] ? nav_tgt : "(最近)"; has_main = true;
  }
  if (any_attempted || nav_done) c.last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
  // AI 的 move/spin 全有界(throttle 补距离 / spin 补默认角)、arm 全离散 → 无持续残留。
  g_last_continuous = false; g_last_cont_type = 0;

  // ---------- 反馈 / 历史 / 死循环 ----------
  if (!has_any && !nav_done) {
    // 空动作轮(没给 move/arm/light 任一键): 不推手机 —— 模型没话说就不打扰用户, 要说话请走 say;
    // 中止/等待态的告知由 apply_end 兜底, 不依赖模型开不开口。
    c.stall = 0; c.stall_hint = false;
  } else {
    if (has_main && (any_ok || nav_done)) {
      String fb = build_feedback(c.t.id, acmd);   // 动作回执(指令本体, 无模型措辞)
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
  // ---------- AI 决策日志(car 的键) ----------
  {
    char tob[160] = {0};   // 短摘要: 用了哪些键
    bool first = true;
    if (has_move && is_mv)  ai::safe_append(tob, sizeof(tob), &first, is_ap ? "approach" : "move");
    if (has_move && is_sp)  ai::safe_append(tob, sizeof(tob), &first, "spin");
    if (has_arm)            ai::safe_append(tob, sizeof(tob), &first, "arm");
    if (has_light)          ai::safe_append(tob, sizeof(tob), &first, "light");
    ai::logf("[ai工具] car: %s", tob[0] ? tob : "(无动作键)");
  }
  {
    char nb[176];
    if (merge_str[0]) snprintf(nb, sizeof(nb), "car %s%s", merge_str, any_ok ? "" : " (未落地)");
    else if (nav_done) snprintf(nb, sizeof(nb), "car approach %s", nav_tgt[0] ? nav_tgt : "(最近)");
    else nb[0] = 0;
    if (nb[0]) notify_tool(c, nb);
  }

  // ---------- 结果文本(car 的返回值) ----------
  // 车与臂的当前姿态不在这里 —— 由组包时的状态块统一给出。
  PsaBuf rt;
  rt.put("[执行结果]");
  echo_seg(rt, &seg_n, "move", emv);
  echo_seg(rt, &seg_n, "arm", earm);
  echo_seg(rt, &seg_n, "light", elight);
  if (seg_n == 0) rt.put(" 本轮无动作");
  snprintf(out, out_cap, "%s", rt.p ? rt.p : "[执行结果] 本轮无动作");
}

// ---------------- 工具落地(mem / car / look / finish) ----------------
// 每个 handler 把"这次调用发生了什么"写成结果文本(out) —— 该文本作为 tool 结果回喂给模型, 是它唯一的
// 执行反馈通道(动作有没有落地、为什么被拒、现在什么姿态, 全靠这一段说清)。

// 刚追加到当前回合的最后一条调用(供回填图信息)。无则 nullptr。
static HistCall* hist_last_call(RoundCtx& c) {
  if (!c.hist || c.hist_n <= 0) return nullptr;
  HistTurn& tn = c.hist[c.hist_n - 1];
  if (tn.chat || tn.ncall <= 0) return nullptr;
  return &tn.calls[tn.ncall - 1];
}

// say: 对用户说一句话(推一条 ai_result 到手机)。没什么可说就不调 —— 这是"闷声干活"的出口。
// 复用 ai_result 的 reason 字段承载正文, 手机端无需改动。
static void do_say(RoundCtx& c, const char* args, char* out, size_t cap) {
  JsonDocument d(&g_js_alloc);
  if (args[0] && deserializeJson(d, args)) {
    snprintf(out, cap, "say: 参数不是合法 JSON(需要 text), 未发送。");
    return;
  }
  const char* tx = d["text"] | "";
  if (!tx[0]) { snprintf(out, cap, "say: text 为空, 未发送。"); return; }
  JsonDocument f(&g_js_alloc); f["reason"] = tx;
  String fb = build_feedback(c.t.id, f);
  ai::enqueue_result(fb.c_str(), c.t.fn, c.t.ctx);
  ai::logf("[ai工具] say: %s", tx);
  // 历史里复述原话(而非"已发送"): say 是模型唯一的自然语言输出, 回喂它是持续的中文**句子级**示范 ——
  // 否则历史里"模型自己说过的话"只剩英文 JSON 参数与英文 reasoning, 中文示范缺失。
  snprintf(out, cap, "say: 已对用户说「%s」", tx);
}

// mem: 物体记忆与车姿态 —— observe 记录 / delete 删除 / 两者都不给 = 查询。
// 观测必须用**那张图拍摄时**的车位姿换算(基准快照见 RoundCtx::sent_car_*); 查询则给出小车全局位姿与全部记忆。
static void do_mem(RoundCtx& c, const char* args, char* out, size_t cap) {
  JsonDocument cmdD(&g_js_alloc);
  char verr[128];
  const char* e = ai::validate_cmd(args[0] ? args : "{}", cmdD, verr, sizeof(verr));   // 空参数视作空对象(=查询)
  if (e) { snprintf(out, cap, "mem: 参数未通过校验(%s), 未执行。", e); return; }
  bool has_obs = cmdD["observe"].is<JsonArray>();
  bool has_del = cmdD["delete"].is<JsonArray>();
  if (!has_obs && !has_del) {   // 空 = 查询: 位姿 + 物体记忆
    int l = snprintf(out, cap, "mem: ");
    if (l < 0) l = 0;
    size_t rest = ((size_t)l + 8 < cap) ? cap - (size_t)l : 1;
    if (rest < 1) rest = 1;     // mem_feed 至少要一个字节写结束符
    ai::mem_feed(out + l, rest);
    return;
  }
  char edel[160] = {0};
  if (has_obs)
    for (JsonObjectConst ob : cmdD["observe"].as<JsonArrayConst>()) land_observe_one(c, ob);
  if (has_del)
    for (JsonVariantConst it : cmdD["delete"].as<JsonArrayConst>()) {
      if (!it.is<const char*>()) continue;   // 只认字符串名字
      const char* nm = it.as<const char*>();
      if (!nm || !nm[0]) continue;
      bool gone = ai::mem_forget(nm);
      ai::logf("[ai] 删除物体记忆: %s%s", nm, gone ? "" : "(记忆里没有该物体)");
      snprintf(edel + strlen(edel), sizeof(edel) - strlen(edel), "%s%s%s%s",
               edel[0] ? ", " : "", gone ? "已删除" : "记忆里没有", nm, gone ? "的记忆" : "");
    }
  // AI 决策日志
  {
    char tob[160] = {0}; bool first = true;
    if (has_obs) {
      JsonArrayConst oa = cmdD["observe"].as<JsonArrayConst>();
      JsonObjectConst ob = oa.size() > 0 ? oa[0].as<JsonObjectConst>() : JsonObjectConst();
      char b1[96];
      snprintf(b1, sizeof(b1), "observe(%s,px%.2f,py%.2f%s)", ob["name"] | "", ob["px"] | 0.0f, ob["py"] | 0.0f,
               oa.size() > 1 ? ",..." : "");
      ai::safe_append(tob, sizeof(tob), &first, b1);
    }
    if (has_del) {
      char b2[24];
      snprintf(b2, sizeof(b2), "delete(%u项)", (unsigned)cmdD["delete"].as<JsonArrayConst>().size());
      ai::safe_append(tob, sizeof(tob), &first, b2);
    }
    ai::logf("[ai工具] mem: %s", tob[0] ? tob : "(查询)");
    char nb[200];
    snprintf(nb, sizeof(nb), "mem %s", tob[0] ? tob : "(查询记忆/位姿)");
    notify_tool(c, nb);
  }
  // 回执: 本轮观测到的读数 / 没记成的告警 / 删除结果(一次性, 用完即清)
  PsaBuf rt;
  rt.put("mem:");
  int n = 0;
  echo_seg(rt, &n, "observe", c.obs_echo);
  echo_seg(rt, &n, "delete", edel);
  if (c.obs_warn[0]) { rt.put(" | "); rt.put(c.obs_warn); n++; }
  if (n == 0) rt.put(" 未记录任何内容");
  c.obs_echo[0] = 0;
  c.obs_warn[0] = 0;
  snprintf(out, cap, "%s", rt.p ? rt.p : "mem: 未记录任何内容");
}

// task: 任务记账(纯记录, 不含动作) —— note 记要点 / todo 重写任务列表 / done 标记已完成项。
static void do_task(RoundCtx& c, const char* args, char* out, size_t cap) {
  JsonDocument cmdD(&g_js_alloc);
  char verr[128];
  const char* e = ai::validate_cmd(args[0] ? args : "{}", cmdD, verr, sizeof(verr));
  if (e) { snprintf(out, cap, "task: 参数未通过校验(%s), 未记录。", e); return; }
  char etnote[24] = {0}, etasks[40] = {0}, etdone[56] = {0};
  const char* tn = cmdD["note"] | "";
  if (tn[0]) {
    if (strcmp(tn, ps_str(c.task_note))) {
      if (ps_set(c.task_note, tn)) {   // 按实际长度分配, 无上限
        ai::logf("[ai] 任务笔记: %s", c.task_note);
        snprintf(etnote, sizeof(etnote), "已更新");
      } else snprintf(etnote, sizeof(etnote), "内存不足");   // 明说, 让模型精简后重写
    } else snprintf(etnote, sizeof(etnote), "无变化");
  }
  if (cmdD["todo"].is<JsonArray>()) {
    // todo = "重写整个任务列表", 每项就是一个名字 ⇒ done 不从 JSON 读: 重写即从"全部未完成"重来。
    // 项数与各项长度当场已知 ⇒ 一次分配到 PSRAM, 既无项数上限也无字数上限。
    if (task_list_replace(c, cmdD["todo"].as<JsonArrayConst>())) {
      ai::logf("[ai] 任务列表更新(%d项)", c.s_task_n);
      snprintf(etasks, sizeof(etasks), "已重写(%d项)", c.s_task_n);
    } else snprintf(etasks, sizeof(etasks), "内存不足, 未更新");
  }
  if (cmdD["done"].is<JsonArray>()) {
    char idxs[32] = {0};   // 命中的编号列表: "1、3"
    for (JsonVariantConst it : cmdD["done"].as<JsonArrayConst>()) {
      int idx = (it | 0) - 1;
      if (idx < 0 || idx >= c.s_task_n) continue;
      if (!c.s_tasks[idx].done) ai::logf("[ai] 任务%d → 完成", idx + 1);
      c.s_tasks[idx].done = true;
      snprintf(idxs + strlen(idxs), sizeof(idxs) - strlen(idxs), "%s%d", idxs[0] ? "、" : "", idx + 1);
    }
    if (idxs[0]) snprintf(etdone, sizeof(etdone), "已完成第%s项", idxs);
  }
  {
    char tob[128] = {0}; bool first = true;
    if (etnote[0]) ai::safe_append(tob, sizeof(tob), &first, "note");
    if (etasks[0]) ai::safe_append(tob, sizeof(tob), &first, "todo");
    if (etdone[0]) ai::safe_append(tob, sizeof(tob), &first, "done");
    ai::logf("[ai工具] task: %s", tob[0] ? tob : "(无有效键)");
    if (tob[0]) {
      char nb[128];
      snprintf(nb, sizeof(nb), "task%s%s%s", etnote[0] ? " note" : "",
               etasks[0] ? " todo" : "", etdone[0] ? " done" : "");
      notify_tool(c, nb);
      // 列表/笔记真变了才推快照: pure note 不重排列表, 但面板上笔记那行也要跟着换。
      // 注意 todo 是"整段重写"(旧项全清、done 全清) —— 手机端会看到进度回退, 这是模型行为不是 bug。
      notify_tasks(c, "running");
    }
  }
  PsaBuf rt;
  rt.put("task:");
  int n = 0;
  echo_seg(rt, &n, "note", etnote);
  echo_seg(rt, &n, "todo", etasks);
  echo_seg(rt, &n, "done", etdone);
  if (n == 0) rt.put(" 未记录任何内容");
  snprintf(out, cap, "%s", rt.p ? rt.p : "task: 未记录任何内容");
}

// goal: set 更新最终目标 / finish 结束本次任务(结束语先用 say 发送)。同轮先 set 后 finish。
static void do_goal(RoundCtx& c, const char* args, char* out, size_t cap) {
  JsonDocument cmdD(&g_js_alloc);
  char verr[128];
  const char* e = ai::validate_cmd(args[0] ? args : "{}", cmdD, verr, sizeof(verr));
  if (e) { snprintf(out, cap, "goal: 参数未通过校验(%s), 未执行。", e); return; }
  char eset[24] = {0};
  const char* g = cmdD["set"] | "";
  if (g[0]) {
    c.goal_explicit = true;   // ⚠️ 与文案是否变化无关: 原话常与用户发言一字不差, 用 strcmp 判会把它整段吞掉
    if (strcmp(g, ps_str(c.goal_now))) {
      if (ps_set(c.goal_now, g)) {   // 按实际长度分配, 无上限
        ai::logf("[ai] 目标更新: %s", c.goal_now);
        snprintf(eset, sizeof(eset), "已更新");
        notify_tasks(c, "running");   // 目标被改写: 面板标题跟着变(AI 常把用户原话重述成更具体的目标)
      } else snprintf(eset, sizeof(eset), "内存不足");
    } else snprintf(eset, sizeof(eset), "无变化");
  }
  const char* fin = cmdD["finish"] | "";
  ai::logf("[ai工具] goal: %s%s", eset[0] ? "set" : "(无set)", fin[0] ? fin : "(未 finish)");
  if (eset[0] || fin[0]) {
    char nb[80];
    snprintf(nb, sizeof(nb), "goal%s%s%s", eset[0] ? " set" : "", fin[0] ? " finish(" : "", fin[0] ? fin : "");
    if (fin[0]) strncat(nb, ")", sizeof(nb) - strlen(nb) - 1);
    notify_tool(c, nb);
  }
  PsaBuf rt;
  rt.put("goal:");
  int n = 0;
  echo_seg(rt, &n, "set", eset);
  if (fin[0]) { rt.put(" | [finish] 已结束任务("); rt.put(fin); rt.put(")"); n++; }
  if (n == 0) rt.put(" 未做任何变更");
  snprintf(out, cap, "%s", rt.p ? rt.p : "goal: 未做任何变更");
  if (fin[0]) apply_end(c, fin);
}

// car: 一批动作。沿用词表校验 + land_car 的全部落地逻辑(结果文本也由它写)。
static void do_car(RoundCtx& c, const char* args, char* out, size_t cap) {
  JsonDocument cmdD(&g_js_alloc);
  char verr[128];
  const char* e = ai::validate_cmd(args[0] ? args : "{}", cmdD, verr, sizeof(verr));
  if (e) {
    snprintf(out, cap, "[执行结果] | [car] 参数未通过校验(%s) —— 本次调用没有执行任何动作, 请修正后重新调用 car。", e);
    return;
  }
  land_car(c, cmdD, out, cap);
}

// 渲染「当前可查看图片」一行: 实景帧环 + (本回合新拍、即将进环的 cur) + 用户参考图池, 按编号升序去重。
// 模型据此挑 look(image=[...]) 的编号 —— 号写死, 不必自己数"还剩几张"。
static void render_viewable(RoundCtx& c, PsaBuf& rt) {
  uint32_t ids[AI_PREV_SLOTS + AI_EDITED_SLOTS + 1]; int n = 0;
  // 本回合新拍时 cur 会挤进环首 ⇒ 最旧一槽随即被淘汰(见 prev_roll), 别把它列进来: 列了 AI 下回合
  // 真去用就"超出保留范围"了。纯回看不滚环, 故三槽都留得住。
  int keep = c.look_live ? AI_PREV_SLOTS - 1 : AI_PREV_SLOTS;
  for (int s = 0; s < keep; s++) if (c.prev_id[s] && c.prev_len[s] > 0) ids[n++] = c.prev_id[s];
  if (c.look_live && c.cur_id) ids[n++] = c.cur_id;   // 本回合新拍的: 组包后会滚进环, 下回合起可回看
  for (int s = 0; s < AI_EDITED_SLOTS; s++) if (c.ed_num[s] && c.ed_len[s] > 0) ids[n++] = c.ed_num[s];
  for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++)
    if (ids[j] < ids[i]) { uint32_t t = ids[i]; ids[i] = ids[j]; ids[j] = t; }
  rt.put("当前可查看图片: [");
  int w = 0;
  for (int i = 0; i < n; i++) {
    if (i && ids[i] == ids[i - 1]) continue;   // 排序后去重
    char t[16];
    snprintf(t, sizeof(t), "%sImage%u", w++ ? ", " : "", (unsigned)ids[i]);
    rt.put(t);
  }
  rt.put("]");
}

// look: 取 1~2 张画面回喂(看画面的唯一入口)。缺省(不给任何参数)=新拍一张当前全幅; zoom=新拍是否走高清中央放大;
// image=[N,...]=回看先前给过的画面(含用户发送的参考图), N 见上一次 look 结果里的「当前可查看图片」。
// 可组合, 清单最多 2 张(按 "新拍 → image 顺序" 取, 多的丢掉)。出图后把裁框写进 pend_* 供下一回合当
// observe 基准 —— 只有**新拍的实景**才是基准, 纯回看不动基准。字节归属记到本回合。
static bool do_look(RoundCtx& c, const char* args, char* out, size_t cap) {
  JsonDocument d(&g_js_alloc);
  if (args[0] && deserializeJson(d, args)) {
    snprintf(out, cap, "look: 参数不是合法 JSON, 未取画面(可用 {\"zoom\":bool,\"image\":[编号,...]} 的组合)。");
    return false;
  }
  bool has_zoom = !d["zoom"].isNull();
  bool zoom = d["zoom"] | false;
  // 回看清单: 按给定顺序的全局编号(去重, 只认正整数)。image 存在但不是数组时明说, 别静默当成"新拍全幅"。
  uint32_t want[AI_PREV_SLOTS + AI_EDITED_SLOTS]; int wn = 0;
  bool truncated = false;
  bool img_bad = false;
  if (!d["image"].isNull()) {
    JsonArrayConst ia = d["image"].as<JsonArrayConst>();
    if (ia.isNull()) img_bad = true;
    else for (JsonVariantConst it : ia) {
      if (!it.is<int>()) { img_bad = true; continue; }   // 非整数项(字符串/对象/小数)一律不认
      uint32_t k = (uint32_t)(it | 0);
      if (k == 0) continue;
      bool dup = false;
      for (int j = 0; j < wn; j++) if (want[j] == k) { dup = true; break; }
      if (dup) continue;
      if (wn >= (int)(sizeof(want) / sizeof(want[0]))) { truncated = true; break; }   // 清单容量兜底
      want[wn++] = k;
    }
  }
  bool do_live = has_zoom || wn == 0;   // 没点名要回看别的图, 就默认新拍一张当前画面

  // 清单: 顺序 = 结果里的给图顺序(新拍 → image), 最多 2 张。
  c.look_cnt = 0; c.look_live = false;
  c.look_ids[0] = 0; c.look_ids[1] = 0;
  char dsc[2][56] = {};
  bool zoomed = false;
  if (do_live) {
    char ln[48] = {0};
    if (!fetch_image(c, zoom, ln, sizeof(ln), &zoomed)) {
      snprintf(out, cap, "look: 取画面失败(摄像头/缓冲异常), 稍后再试。");
      return false;
    }
    c.look_set[c.look_cnt] = RoundCtx::LookImg{ c.cur, c.cur_len };
    c.look_ids[c.look_cnt] = c.cur_id;
    snprintf(dsc[c.look_cnt], sizeof(dsc[0]), "新拍的当前画面(%s)", ln);
    c.look_cnt++; c.look_live = true;
  }
  for (int i = 0; i < wn && c.look_cnt < 2; i++) {
    uint32_t k = want[i];
    const uint8_t* p = nullptr; size_t n = 0; const char* who = nullptr;
    for (int s = 0; s < AI_PREV_SLOTS; s++)       // 实景帧环
      if (c.prev_id[s] == k && c.prev_len[s] > 0) { p = c.prev[s]; n = c.prev_len[s]; who = "先前看过的实景"; break; }
    if (!p)
      for (int s = 0; s < AI_EDITED_SLOTS; s++)   // 用户参考图池
        if (c.ed_num[s] == k && c.ed_len[s] > 0) { p = c.ed_img[s]; n = c.ed_len[s]; who = "用户发送的图片"; break; }
    if (!p || n == 0 || n > AI_EDITED_IMG_MAX) continue;   // 号不在保留范围(末尾按"要而没给"统一提示)
    c.look_set[c.look_cnt] = RoundCtx::LookImg{ p, n };
    c.look_ids[c.look_cnt] = k;
    snprintf(dsc[c.look_cnt], sizeof(dsc[0]), "%s", who);
    c.look_cnt++;
  }
  // "要而没给"的判定: 请求张数(新拍 1 张 + 点名编号数) > 实给张数即截断。比"数组项数"准 ——
  // 能覆盖"点了 3 个号只给 2 张"这类被静默丢弃的情形(旧写法在 3 个互异合法号时漏报)。
  if (wn + (do_live ? 1 : 0) > (int)c.look_cnt) truncated = true;
  if (c.look_cnt == 0) {
    snprintf(out, cap, "look: 没有可给的画面(指定编号不在保留范围), 未取画面。");
    return false;
  }

  // 字节归属本回合: 下一回合组包时把清单里的图注入这条 tool 结果(老图只留占位文本)。
  c.img_owner = c.img_tag_n;
  c.img_sent = false;
  // observe 基准: 只有"新拍的实景"才是基准; 纯回看不更新(沿用上一次)
  if (c.look_live) {
    c.pend_valid = true;
    c.pend_zoomed = zoomed;
    if (zoomed) { c.pend_x0 = 0.25f; c.pend_y0 = 0.25f; c.pend_x1 = 0.75f; c.pend_y1 = 0.75f; }
    else { c.pend_x0 = 0; c.pend_y0 = 0; c.pend_x1 = 1; c.pend_y1 = 1; }
    c.pend_car_x = ai::s_car_x; c.pend_car_y = ai::s_car_y; c.pend_car_hd = ai::s_car_heading;   // 拍这张图时的车位姿
  }
  // 复读保护: 车与臂都没动的前提下连着要同一块画面 —— 画面里不会有新信息(裁框是全幅归一化坐标,
  // 挪过车之后同一个框里已是另一片世界, 所以判"重复"必须带上"这期间动过没有")。
  if (zoomed) {
    char st[256];
    const char* sp = exec::read_state(st, sizeof(st)) ? st : "";
    char pose[80];
    // 取够长: 状态串里的"位置(x,h)"排在 姿态 之后, 截太短会把臂真的挪过的信息切掉(重现"没动"误判)。
    snprintf(pose, sizeof(pose), "%d,%d,%d|%.64s", (int)ai::s_car_x, (int)ai::s_car_y,
             (int)ai::s_car_heading, sp);
    if (!strcmp(pose, c.zoom_pose)) { if (c.zoom_noop_n < 32000) c.zoom_noop_n++; }
    else { snprintf(c.zoom_pose, sizeof(c.zoom_pose), "%s", pose); c.zoom_noop_n = 1; }
  }
  // 结果文本: 给了哪几张 + 拍新图时的提示 + 可回看的编号清单
  PsaBuf rt;
  {
    char t[12];
    snprintf(t, sizeof(t), "look: 本条给 %d 张画面: ", (int)c.look_cnt);
    rt.put(t);
  }
  for (int i = 0; i < c.look_cnt; i++) {
    char t[12];
    snprintf(t, sizeof(t), "第%d张=", i + 1);
    rt.put(t); rt.put(dsc[i]); rt.put("; ");
  }
  if (img_bad) rt.put("image 需要编号数组(如 [3,5]), 非法项已忽略; ");
  if (truncated) rt.put("(未全部给出: 最多 2 张, 超出或不在保留范围的已忽略); ");
  if (c.frame_moving) { rt.put(AI_FRAME_MOVING_HINT); rt.put("; "); }
  if (zoomed && c.zoom_noop_n > AI_ZOOM_NOOP_MAX)
    rt.put("车与机械臂都没动, 而你要的又是同一块放大画面: 画面里不会再出现新信息, 不妨试试转动一下视角; ");
  if (zoomed) { rt.put(AI_ZOOM_HINT); rt.put("; "); }
  if (c.look_live) rt.put("新拍的当前画面可以作为 observe 的基准。");
  else rt.put("这些是参照图不是实景: 不可根据它们使用 observe。");
  rt.put("\n");
  render_viewable(c, rt);
  snprintf(out, cap, "%s", rt.p ? rt.p : "look: 已取画面");
  {
    char nb[128];
    snprintf(nb, sizeof(nb), "look %d张%s%s", (int)c.look_cnt, c.look_live ? "(新拍)" : "(回看)",
             zoomed ? " 放大" : "");
    notify_tool(c, nb);
  }
  return true;
}

// compact: 压缩历史上下文。此处只记摘要, 真正的清空在本回合的调用全部落地后统一做
// (见 compact_history)—— 此刻本回合的 assistant/tool 回合还没记完, 现在清会把它一起清掉。
static void do_compact(RoundCtx& c, const char* args, char* out, size_t cap) {
  JsonDocument d(&g_js_alloc);
  if (!args[0] || deserializeJson(d, args)) {
    snprintf(out, cap, "compact: 参数不是合法 JSON(需要 summary), 未压缩。");
    return;
  }
  const char* sm = d["summary"] | "";
  if (!sm[0]) { snprintf(out, cap, "compact: summary 为空, 未压缩。"); return; }
  // 摘要原文完整转发日志通道(未截断的那份): 摘要用什么语言写、总结成什么样, 直接决定压缩后的行为走向。
  ai::logf("[ai工具] compact: 摘要 %u 字节", (unsigned)strlen(sm));
  blog::forward_text(blog::AI, sm);
  snprintf(c.compact_sum, sizeof(c.compact_sum), "%s", sm);
  utf8_clamp_tail(c.compact_sum);   // 定长缓冲截断可能切在汉字中间
  notify_tool(c, "compact 压缩历史(旧画面随之清空)");
  snprintf(out, cap, "compact: 已记录摘要, 本回合结束后清空此前对话(目标/任务列表/笔记/记忆/位姿仍在; 可回看的旧画面将一并清空)。");
}

// ---------------- 本回合的工具分发 ----------------
// 按固定次序 mem → task → say → car → look → compact → goal 落地(不信任模型给的顺序): mem 的 observe 必须用
// **动作前**的车位姿换算, 故排最前; task 记账与 say 一样是纯文案(先说后做, 手机上就是"话在前、动作回执在后");
// car 动作居中; look 排在 car 之后是有意的(模型同回合要图是想看刚做完动作的效果); compact 收在动作之后、
// goal 之前(goal 的 finish 终结任务, 必须最后)。
// ⚠️ 模型给出的**每个** tool_call 都必须在历史里留下一条对应结果 —— 少一条就是"孤儿调用"(配对断了), 云端会拒整个请求。
static void dispatch_calls(RoundCtx& c) {
  JsonArrayConst arr = c.calls["calls"].as<JsonArrayConst>();
  const int n = (int)arr.size();
  if (n <= 0) return;
  // 历史环不可用(PSRAM 不足)时仍执行工具: 否则本回合所有调用被静默丢弃, 车不动、手机也无反馈。
  // hist_add_call 内部对 hist_n<=0 有守卫, 会安全跳过写入(只丢历史、不丢动作)。
  HistTurn* tn = hist_new_turn(c);
  if (tn) {   // 思考原文随回合存一份(ps_dup 成历史自己的副本): 后续请求要原样回传
    const char* rc = c.calls["reasoning"] | "";
    if (rc[0]) tn->reasoning = ps_dup(rc);
  } else ai::logf("[ai] 历史环不可用, 本回合的调用不记入历史");
  static const char* kOrder[] = { "mem", "task", "say", "car", "look", "compact", "goal" };
  const int kOrderN = (int)(sizeof(kOrder) / sizeof(kOrder[0]));
  const int cap_n = n < 16 ? n : 16;   // 超出部分直接不记(assistant 只列我们记下的, 不会产生孤儿)
  bool handled[16] = {false};
  bool exec_seen[kOrderN] = {false};   // 该工具本回合是否真的执行过(区分"重复调用"与"任务已结束被跳过")
  char res[1024];
  for (int oi = 0; oi < kOrderN && !c.done; oi++) {
    int pick = -1;
    for (int i = 0; i < cap_n; i++) {
      if (handled[i]) continue;
      const char* nm = arr[i]["name"] | "";
      if (!strcmp(nm, kOrder[oi])) { pick = i; break; }
    }
    if (pick < 0) continue;   // 本回合没调这个工具
    handled[pick] = true;
    exec_seen[oi] = true;
    const char* id = arr[pick]["id"] | "";
    const char* args = arr[pick]["args"] | "";
    res[0] = 0;
    bool is_look = !strcmp(kOrder[oi], "look");
    bool ok = true;
    if (!strcmp(kOrder[oi], "mem"))       do_mem(c, args, res, sizeof(res));
    else if (!strcmp(kOrder[oi], "task")) do_task(c, args, res, sizeof(res));
    else if (!strcmp(kOrder[oi], "say"))  do_say(c, args, res, sizeof(res));
    else if (!strcmp(kOrder[oi], "car"))  do_car(c, args, res, sizeof(res));
    else if (is_look)                     ok = do_look(c, args, res, sizeof(res));
    else if (!strcmp(kOrder[oi], "compact")) do_compact(c, args, res, sizeof(res));
    else                                  do_goal(c, args, res, sizeof(res));
    hist_add_call(c, id, kOrder[oi], args, res);
    if (is_look && ok) {   // 图信息补进刚追加的那条(张数/全局编号); 字节下一回合组包时注入
      HistCall* hc = hist_last_call(c);
      if (hc) {
        hc->img_n = c.look_cnt ? c.look_cnt : 1;
        for (int k = 0; k < 2; k++) hc->img_id[k] = c.look_ids[k];
      }
    }
  }
  // 未执行的调用(同回合重复同名 / 因任务已结束被跳过 / 未知工具): 也要各回一条结果,
  // 否则配对断了云端会拒。⚠️ 因 c.done 提前退出而没轮到的**不同名**工具不算"重复", 别扣错帽子。
  for (int i = 0; i < cap_n; i++) {
    if (handled[i]) continue;
    const char* id = arr[i]["id"] | "";
    const char* nm = arr[i]["name"] | "";
    const char* args = arr[i]["args"] | "";
    int hit = -1;
    for (int oi = 0; oi < kOrderN; oi++) if (!strcmp(nm, kOrder[oi])) { hit = oi; break; }
    if (hit >= 0 && exec_seen[hit])
      snprintf(res, sizeof(res), "%s: 本回合重复调用了 %s, 只执行了第一条; 这一条已忽略。", nm, nm);
    else if (hit >= 0)
      snprintf(res, sizeof(res), "%s: 本回合任务已结束, 这一条未执行。", nm);
    else
      snprintf(res, sizeof(res), "%s: 未知工具, 未执行(可用: mem/task/say/car/look/compact/goal)。", nm[0] ? nm : "(无名)");
    hist_add_call(c, id, nm, args, res);
  }
  // 压缩放在所有调用落地之后: 本回合的回合已完整记进历史, 才能安全清掉更早的。
  if (c.compact_sum[0] && !c.done) compact_history(c);
}

// ---------------- 单回合驱动 ----------------
// 一回合 = 一次请求: 提交图基准 → 入口检查 → (首回合注入一张画面) → 组包/发送/取工具调用 → 落地 → 节奏控制。
static RoundR round_step(RoundCtx& c) {
  uint64_t step_ts = esp_timer_get_time();  // 本回合起点(周期控制基准)
  c.fail = nullptr;  // 每回合重置, 避免沿用上回合错误文本误导日志/回报
  c.got = false;

  commit_pend(c);   // 上一回合产出的图, 从这一回合起成为 observe 的换算基准
  PrepR pr = prep_gate(c);
  if (pr != PrepR::Ok) return pr == PrepR::SkipRound ? RoundR::NextRound : RoundR::ExitTask;

  // 首回合(以及无有效输出重试回到这里时)由程序注入一张当前画面 —— agent 循环里唯一一次自动给图,
  // 之后看不看全由模型自己调 look 决定。
  if (c.steps == 0) {
    if (!c.cam_ok) {
      snprintf(c.frame_note, sizeof(c.frame_note), "当前无摄像头画面(无画面模式, 只能靠 mem 与动作结果判断): ");
    } else {
      char note[64] = {0}; bool zm = false;
      if (fetch_image(c, false, note, sizeof(note), &zm)) {
        c.frame = c.cur; c.frame_len = c.cur_len;
        snprintf(c.frame_note, sizeof(c.frame_note), "当前画面如下(系统实时画面, 非用户发送): ");
        c.img_owner = 0; c.img_sent = true;   // 首帧走尾部 user 消息, 不占历史条目的图位
        c.pend_valid = true; c.pend_zoomed = false;
        c.pend_x0 = 0; c.pend_y0 = 0; c.pend_x1 = 1; c.pend_y1 = 1;
        c.pend_car_x = ai::s_car_x; c.pend_car_y = ai::s_car_y; c.pend_car_hd = ai::s_car_heading;   // 拍这张图时的车位姿
        if (c.frame_moving) hpush(c.pend_hint, sizeof(c.pend_hint), AI_FRAME_MOVING_HINT);
        if (c.ed_n > 0) {
          // 注入的是**实景相机帧**, 用户发来的参考图并不在这里 —— 它们已统一编号, 要用得自己去 look 取。
          char eb[192];
          snprintf(eb, sizeof(eb),
                   "用户本次还发来 %d 张参考图(未自动注入此处); 需要时用 look(image=[编号]) 查看, 编号见 look 结果的「当前可查看图片」",
                   c.ed_n);
          hpush(c.pend_hint, sizeof(c.pend_hint), eb);
        }
      } else if (c.fail) {
        c.done = true;   // 取帧失败已到上限: 收尾(round_task_finish 报 error)
        return RoundR::ExitTask;
      } else {
        snprintf(c.frame_note, sizeof(c.frame_note), "本回合未取到画面(取景失败, 请谨慎行动): ");
      }
    }
  }

  AttR ar = round_attempt(c);
  if (ar == AttR::Break) return RoundR::ExitTask;
  if (ar == AttR::Retry) {
    // 无有效输出(网络/解析失败, 或模型整回合没调工具): 指数退避重试, 连续超限即中止。
    // ⚠️ oneshot 只跑一回合: 不在这里空等重试, 直接把原因报回去。
    if (c.t.one_shot) {
      JsonDocument e(&g_js_alloc);
      e["error"] = c.fail ? c.fail : "AI 未调用工具";
      e["done"] = true;
      c.sent_done = true;
      String s = build_feedback(c.t.id, e);
      ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
      return RoundR::ExitTask;
    }
    if (++c.net_fail >= AI_MAX_NET_FAIL) {
      const char* reason = "云端持续请求失败, 任务已中止, 请稍后重试";
      JsonDocument e(&g_js_alloc);
      e["error"] = reason;
      e["done"] = true;   // 终结必带 done, 让手机端把「中止」复位为「发送」
      c.sent_done = true;
      String s = build_feedback(c.t.id, e);
      ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
      blog::logf(blog::AI, "连续无有效输出超限, 任务中止");
      return RoundR::ExitTask;
    }
    int wait = ai::http_last_status() == 429 ? 30000 : (3000 << (c.net_fail > 3 ? 3 : c.net_fail - 1));
    ai::logf("[ai] 本回合无有效输出(%s), %ds 后重试", c.fail ? c.fail : "未知", wait / 1000);
    vTaskDelay(pdMS_TO_TICKS(wait));
    return RoundR::NextRound;
  }
  c.net_fail = 0;

  dispatch_calls(c);
  if (c.done) return RoundR::ExitTask;
  // 单回合模式(/ai oneshot): 一个回合决策即收尾, 让手机端复位「发送」并显示任务结束。
  if (c.t.one_shot) {
    JsonDocument e(&g_js_alloc);
    e["done"] = true;
    String s = build_feedback(c.t.id, e);
    ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
    c.sent_done = true;
    c.done = true;
    return RoundR::ExitTask;
  }
  // 空转兜底(见 AI_IDLE_ROUNDS): 连续多回合只有 look/mem 这类只读调用 → 大概率被局部画面困住。
  // 只清状态不说清, AI 下回合还会照样放大; 所以把"怎么办"一并写进下一条工具结果的尾部。
  if (c.acted) { c.idle_rounds = 0; }
  else if (++c.idle_rounds >= AI_IDLE_ROUNDS) {
    c.idle_rounds = 0;
    hpush(c.pend_hint, sizeof(c.pend_hint),
          "连续多个回合没有实际动作了: 若画面里找不到目标, 建议重新环视搜索");
    ai::logf("[ai] 连续 %d 回合无动作, 提示环视", AI_IDLE_ROUNDS);
  }
  c.acted = false;
  ai::mem_tick_stale();   // 每回合结束: 未观测的物体过期轮数 +1
  c.steps++;              // 仅用于标记"是否首回合"(见 round_task 头部), 不再作任务终止依据
  // 周期控制: 以 AI_INTERVAL_MS 为下限节奏, 扣掉本回合已耗时(含抓帧/HTTP/落地),
  // 服务端快时立即进入下一回合, 慢时由服务端耗时主导。
  uint64_t el = esp_timer_get_time() - step_ts;
  long rem = (long)AI_INTERVAL_MS - (long)(el / 1000);
  if (rem > 0) vTaskDelay(pdMS_TO_TICKS(rem));
  return RoundR::NextRound;
}

// ---------------- 任务收尾 ----------------
static void round_task_finish(RoundCtx& c) {
  // 终态快照: 面板定格成"已完成/失败/已中止"(不再随时间变化), 手机端据此保留列表供回看。
  // 中断(abort)也发: 手机端点「中止」时本地已乐观置灰, 这条是板端的确认。
  notify_tasks(c, c.interrupted ? "abort" : (c.fail ? "fail" : "done"));
  // 任务终结补发 done: 覆盖失败/掉线等"只报 error 不带 done"的终态, 让手机端把「中止」复位为
  // 「发送」。被中断时跳过(避免误复位下一任务); 已置 sent_done 的不再补发。
  if (!c.interrupted && !c.sent_done) {
    JsonDocument e(&g_js_alloc);
    e["done"] = true;
    if (c.fail) e["error"] = c.fail;
    String s = build_feedback(c.t.id, e);
    ai::enqueue_result(s.c_str(), c.t.fn, c.t.ctx);
  }

  task_list_free(c);               // 任务列表(单块 PSRAM)
  ps_free(c.task_note);            // 变长文本: 收尾统一放掉(与 hist/cur/prev 同一条出口)
  ps_free(c.goal_now);
  for (int i = 0; i < AI_EDITED_SLOTS; i++) if (c.ed_img[i]) free(c.ed_img[i]);  // 用户编辑图快照
  if (c.cur) free(c.cur);          // 最近一张帧 PSRAM 副本
  for (int i = 0; i < AI_PREV_SLOTS; i++) if (c.prev[i]) free(c.prev[i]);   // 先前帧环各槽副本
  if (c.hist) {                    // 历史环: 逐回合放掉各调用的字符串, 再放表本身
    for (int i = 0; i < c.hist_n; i++) hist_free_turn(c.hist[i]);
    free(c.hist);
  }
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