#include "src/ai/ai_client.h"
#include "src/ai/ai_result.h"   // 结果队列/日志出口/编辑图暂存 + result_init
#include "src/ai/ai_round.h"    // round_run_task(AI 任务主体)
#include "src/ai/ai_nav.h"      // navigate_to / NavR / NAV_STOP_CM_GOTO(纯导航任务)
#include "src/ai/ai_http.h"     // http_stop(打断在途请求)
#include "src/ai/ai_mem.h"      // s_car_x/y/heading(纯导航起点复位)
#include "src/ai/ground_proj.h" // ground::init(单应拟合)
#include "src/ai/ai_alloc.h"    // g_js_alloc(共享 PSRAM JSON 池)
#include "src/ai/tools/tool.h"  // tools_selfcheck(工具表完整性自检)
#include "src/core/board_log.h" // blog::logf
#include "src/core/lock_guard.h" // ScopedLock(g_mtx 的取放)

#include <ArduinoJson.h>
#include <esp_heap_caps.h>      // MALLOC_CAP_SPIRAM(worker 栈)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdlib.h>             // strdup/malloc/free
#include <string.h>             // strncpy

// PsramAllocator 见 src/ai/ai_alloc.h; 这里只放定义(唯一处), 各模块 include 该头取用。
PsramAllocator g_js_alloc;

// 代际号: 每次 set_goal/goto_target/cancel 自增; worker 与 ai_nav/ai_round 都比它判"是否被打断"。
// 只在本文件写(持 g_mtx), 外部一律经 ai::generation() 读。
static volatile unsigned long s_generation = 0;

// ---------------- worker / 槽 / 队列 ----------------

static ai::TaskLocal g_slot;
static SemaphoreHandle_t g_mtx = nullptr;   // 保护 g_slot / s_generation / g_chat
static SemaphoreHandle_t g_notify = nullptr; // 唤醒 worker 的二进制信号量
static volatile bool m_busy = false;
static TaskHandle_t g_worker = nullptr;

// 本轮任务终结时的兜底 stop 模式, 由打断方写入; worker 出口统一解析一次(见 ai_round 的 resolve_stop)。
static volatile int g_stop_mode = (int)ai::StopMode::All;

// AI 任务进行中"插话"缓冲(ai_chat 写入、worker 每轮消费一次, 受 g_mtx 保护)。
// 一次性消费: 喂进下一轮 prompt 后清空, 避免重复塞给 AI。
static char g_chat[256] = {0};
static bool g_chat_has = false;

static void ai_worker(void*) {
  for (;;) {
    xSemaphoreTake(g_notify, portMAX_DELAY);

    ai::TaskLocal t;
    {
      ScopedLock lk(g_mtx);   // 只圈住取槽: 拷完即放锁, 后面的导航/闭环不进临界区
      if (g_slot.active) {
        t = g_slot;
        g_slot.text = nullptr; g_slot.ann = nullptr; g_slot.ctx = nullptr; g_slot.active = false;
      }
    }

    if (!t.text && !t.nav) continue;

    // 立刻置忙(而不是等进到任务主体里再置): session_clear 用 busy() 判断能否直接清会话，
    // 若留着"槽已取走、还没置忙"的窗口，/clear 会在 worker 正读会话状态时把它释放掉。
    m_busy = true;

    // 纯导航任务(/move to): 不调云端, 直接本地巡航到目标坐标即回报。
    if (t.nav) {
      if (!t.nav_global) { ai::s_car_x = 0; ai::s_car_y = 0; ai::s_car_heading = 0; }  // local=以当前位姿为新原点
      ai::NavR r = ai::navigate_to(t.generation, t.nav_x, t.nav_y, NAV_STOP_CM_GOTO);
      JsonDocument f(&g_js_alloc);
      f["type"] = "ai_result";
      if (t.id) f["id"] = (long)t.id;
      // 三种出口分开报, 免得把"没收敛停在半路"说成"被打断"。
      f["params"]["reason"] = r == ai::NavR::Reached     ? "已到达目标坐标"
                            : r == ai::NavR::Interrupted ? "导航被中断"
                                                         : "未收敛(已停在最近处, 见导航日志)";
      f["params"]["done"] = (r == ai::NavR::Reached);
      String s; serializeJson(f, s);
      ai::enqueue_result(s.c_str(), t.fn, t.ctx);
      blog::logf(blog::AI, "导航结束 rel=%d", (int)r);
      if (t.ctx) delete (int*)t.ctx;
      m_busy = false;
      continue;
    }

    // AI 任务级闭环(取帧→组包→云端→校验→落地→下一轮), 全部状态与逻辑在 ai_round。
    ai::round_run_task(t);
  }
}

// ---------------- 对外 ----------------

void ai::init() {
  if (g_worker) return;
  ground::init();   // 屏幕→地面单应拟合 + 诊断日志(见 ground_proj)
  ai::tools_selfcheck();   // 工具表自检(key 唯一/parse 非空): 表是编译期常量, 出错只可能是手误
  // 注: 内部堆水位哨兵 hwatch 已在 setup 里随 blog 一起拉起(要覆盖整段运行, 不能等 AI 起来才采)。
  g_mtx = xSemaphoreCreateMutex();
  g_notify = xSemaphoreCreateBinary();
  ai::result_init();   // 结果队列 + 编辑图互斥(见 ai_result)
  // worker 栈从 PSRAM 出: 内部堆长期贴红线(WiFi RX 缓冲只能落内部 RAM)。AI 模块不碰 NVS/flash,
  // 故不存在"写 flash 时 PSRAM 栈取不到"的风险。
  static StackType_t* s_ai_stack = nullptr;
  static StaticTask_t s_ai_tcb;      // TCB 必须留内部 RAM(FreeRTOS 断言)
  // ⚠️ xTaskCreate* 的 usStackDepth 单位是**字**(StackType_t=4B)：下面声明 16384 字 ⇒ 必须分配
  // 16384*4=64KB。旧代码只 malloc(16384 字节)，任务却以为有 64KB ⇒ 溢出 48KB 写进相邻 PSRAM
  // （栈哨兵在 64KB 处、检查不到），静默踩坏堆 —— 症状就是"随机"故障。
  if (!s_ai_stack) s_ai_stack = (StackType_t*)heap_caps_malloc(16384 * sizeof(StackType_t), MALLOC_CAP_SPIRAM);
  if (s_ai_stack) {
    g_worker = xTaskCreateStaticPinnedToCore(ai_worker, "ai_worker", 16384, nullptr, 2,
                                             s_ai_stack, &s_ai_tcb, 1);
  } else {
    blog::logf(blog::AI, "worker: PSRAM 栈分配失败, 退回内部堆栈");
    xTaskCreatePinnedToCore(ai_worker, "ai_worker", 16384, nullptr, 2, &g_worker, 1);
  }
  blog::logf(blog::AI, "worker 就绪(%s)", s_ai_stack ? "PSRAM 栈" : "内部栈");
}

void ai::set_goal(const char* text, bool use_image, const char* annotation, long id,
                  cmd::ReplyFn reply, void* reply_ctx, bool one_shot) {
  {
    ScopedLock lk(g_mtx);
    s_generation = s_generation + 1;
    g_chat_has = false;   // 新目标清除上一任务的残留插话, 避免串任务
    // 替换旧槽(旧字符串为空因 worker 已取走; 残余则释放)
    if (g_slot.text) free(g_slot.text);
    if (g_slot.ann) free(g_slot.ann);
    if (g_slot.ctx) delete (int*)g_slot.ctx;
    g_slot.text = strdup(text ? text : "");
    g_slot.ann = (annotation && annotation[0]) ? strdup(annotation) : nullptr;
    g_slot.use_image = use_image;
    g_slot.one_shot = one_shot;
    g_slot.id = id;
    g_slot.generation = s_generation;
    g_slot.fn = reply;
    g_slot.ctx = reply_ctx;   // WS: 堆 int*; BLE: nullptr
    g_slot.nav = false;       // 新 AI 目标覆盖可能的纯导航残留
    g_slot.active = true;
  }
  // 尝试掐断在途请求, 让 worker 尽快回到循环取新槽
  ai::http_stop();
  g_stop_mode = (int)StopMode::All;   // 新目标打断旧任务: 残留持续指令在旧任务出口补停
  xSemaphoreGive(g_notify);
}

// /move to x y: 板端本地巡航到坐标(不调 AI)。local=以当前位姿为新原点;
// global=沿用当前全局系(可与 AI/历史导航共用坐标系)。手动接管类: 打断在途任务。
void ai::goto_target(float x, float y, bool frame_global, long id, cmd::ReplyFn reply, void* reply_ctx) {
  {
    ScopedLock lk(g_mtx);
    s_generation = s_generation + 1;               // 新导航接管: 在途 AI/导航作废
    g_chat_has = false;           // 清上一任务残留插话
    if (g_slot.text) free(g_slot.text);
    if (g_slot.ann) free(g_slot.ann);
    if (g_slot.ctx) delete (int*)g_slot.ctx;
    g_slot.nav = true;
    g_slot.nav_x = x; g_slot.nav_y = y;
    g_slot.nav_global = frame_global;
    g_slot.text = nullptr; g_slot.ann = nullptr;
    g_slot.use_image = false; g_slot.one_shot = false;
    g_slot.id = id;
    g_slot.generation = s_generation;
    g_slot.fn = reply;
    g_slot.ctx = reply_ctx;   // 直接接管 command.cpp 预建的堆 fd(对齐 set_goal), 由 nav 分支释放
    g_slot.active = true;
  }
  ai::http_stop();              // 掐断在途 AI 请求, worker 尽快回到循环取导航槽
  g_stop_mode = (int)StopMode::All;
  xSemaphoreGive(g_notify);
}

void ai::cancel(StopMode m) {
  bool was_active;   // 仅当确有任务在跑才上报, 避免手动指令刷屏
  {
    ScopedLock lk(g_mtx);
    was_active = g_slot.active;
    s_generation = s_generation + 1;         // 使在途结果作废
    if (g_slot.text) { free(g_slot.text); g_slot.text = nullptr; }
    if (g_slot.ann) { free(g_slot.ann); g_slot.ann = nullptr; }
    if (g_slot.ctx) { delete (int*)g_slot.ctx; g_slot.ctx = nullptr; }
    g_slot.nav = false;
    g_slot.active = false;
  }
  ai::http_stop();
  g_stop_mode = (int)m;   // 手动 move/stop 接管=None(不补停); arm=Wheels; ai_cancel=All
  if (was_active) blog::logf(blog::AI, "cancel");
}

bool ai::busy() { return m_busy; }

bool ai::append_chat(const char* text) {
  if (!text || !text[0]) return false;
  bool fed;   // 仅当有任务在跑才接收插话
  {
    ScopedLock lk(g_mtx);
    fed = m_busy;
    if (fed) {
      strncpy(g_chat, text, sizeof(g_chat) - 1);
      g_chat[sizeof(g_chat) - 1] = 0;
      g_chat_has = true;
      blog::logf(blog::AI, "用户消息入队: %s", text);
    }
  }
  return fed;
}

// ---------------- 模块内部协作入口(见 ai_client.h) ----------------

bool ai::chat_pending() {
  ScopedLock lk(g_mtx);
  return g_chat_has;   // 只看不动: 插话留给正常流程消费成 user 消息
}

bool ai::chat_take(char* dst, size_t cap) {
  ScopedLock lk(g_mtx);   // RAII: 提前 return 也不会漏放锁
  if (!g_chat_has) return false;
  strncpy(dst, g_chat, cap - 1);
  dst[cap - 1] = 0;
  g_chat_has = false;
  return true;
}

void ai::set_busy(bool v) { m_busy = v; }

int ai::stop_mode() { return g_stop_mode; }

unsigned long ai::generation() { return s_generation; }