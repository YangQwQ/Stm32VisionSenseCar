#include "src/ai/ai_mem.h"
#include "src/ai/ai_client.h"      // ai::logf(AI 调试日志)
#include "src/ai/ground_proj.h"    // ground::screen_to_world(mem_observe_xy 用)
#include "src/core/board_log.h"    // blog 类别(AI 日志经 ai::logf 内部使用)
#include "src/core/utf8.h"         // utf8_clamp_tail(唯一还在用的地方: mem_feed 头一句被截断时)
#include "src/core/psram.h"        // ps_dup/ps_free/ps_str: 物体名按实际长度分配
#include "Calibration.h"           // SPIN_PIVOT_BEHIND_CM(原地旋转车头位移折算)

#include <math.h>        // cosf/sinf/fmodf/fabsf
#include <string.h>      // memset/memcpy/strlen/strcmp/memcmp

// 物体记忆表(通用)。程序存全局坐标, 喂给 AI 时换算成车头局部系, AI 零换算。stale 每轮未观测 +1。
// 记忆表条数(小车最多同时记住几个物体; 满了覆盖最旧的, 见 mem_store)。
// ⚠️ 这**不是**文本长度限制, 是记忆模型的设计值: 多一条 = 每轮多渲染一行喂给模型(mem_feed) + 多一个
// 同名匹配的候选。长度限制的问题在下面的 name 上, 那个已经改成按实际长度分配了。
#define AI_MEM_MAX 16
// 记忆行里"未更新"提醒的轮数: 到这一轮补一次"看到就更新"的提醒(只此一次, stale 再涨不再重复)。
#define AI_MEM_REMIND_STALE 5
static struct {
  // 物体名: PSRAM 上按**实际长度**分配(名字是 observe/mem_find 的匹配键, 也是喂回模型的记忆行内容,
  // 写死长度就会认错物体、还会让模型看到被切断的残字)。换物体/遗忘/重置时释放, 见 mem_store 等。
  char* name;
  float gx, gy;        // 全局坐标 cm(每次观测直接覆盖)
  uint32_t t_ms;       // 最近观测时刻
  int16_t stale;       // 0=新鲜; 每轮未观测 +1(喂回时按它排新鲜度、报"N轮未更新")
  bool valid;
} g_mem[AI_MEM_MAX];

namespace ai {

float s_car_x = 0, s_car_y = 0;
int16_t s_car_heading = 0;

// 同一物体判定: 精确同名; 或首字相同且一者名字包含另一者余部(容忍加减修饰词)。
// 收紧为包含关系, 宁可拆两条也不把同首字的不同物体混成一条。
static bool name_same_obj(const char* a, const char* b) {
  int la = (int)strlen(a), lb = (int)strlen(b);
  if (la == 0 || lb == 0) return false;
  if (la == lb && !strcmp(a, b)) return true;
  if (la < 4 || lb < 4) return false;            // 首字(3 字节)+至少 1 字节余量才谈包含
  if (memcmp(a, b, 3)) return false;             // 首字不同: 视为不同物体
  return (la > lb && strstr(a + 3, b + 3)) || (lb > la && strstr(b + 3, a + 3));
}

// 把一次"车头系坐标"观测写入物体记忆表: 用**拍那张图时**的车位姿转成全局坐标, 每次观测无条件覆盖。
// ⚠️ 位姿必须取自观测所依据的那张图(调用方传快照), 不能用当前实时位姿 —— 车在"看图"与"报坐标"之间
// 动过时, 用实时位姿解算会把这条记忆整体平移一个位移量。
// 返回 false = 这条没记进去(名字分配失败): 调用方须据此回告 AI, 不能报成"已记录"。
static bool mem_store(const char* name, float wx, float wy, const CarPose& pose) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (g_mem[i].valid && ps_str(g_mem[i].name)[0] && name_same_obj(g_mem[i].name, name)) { slot = i; break; }
    if (!g_mem[i].valid) { slot = i; break; }
    if (g_mem[i].t_ms < g_mem[oldest].t_ms) oldest = i;
  }
  if (slot < 0) slot = oldest;                        // 满: 覆盖最旧
  // 名字按实际长度分配(无上限)。**先分配再释放旧的**: 分配失败就整条不记(明说), 不要把旧名字弄没 ——
  // 名字空着会被后续 mem_find 当成"未命名槽"误用, 比不记更糟。
  char* nn = ps_dup(name);
  if (!nn) { ai::logf("[ai] 观测 %s 未记入: 名字分配失败(PSRAM 不足)", name); return false; }
  ps_free(g_mem[slot].name);                       // 非按名命中的槽(空槽/复用最旧)名字属于别的物体, 放掉
  g_mem[slot].name = nn;
  // 车头系 (wx右+, wy前+) → 全局: heading 逆时针正(左转+), 前=(-sin,cos)、右=(cos,sin)
  float h = pose.hd * AI_PI / 180.0f, ch = cosf(h), sh = sinf(h);
  float gx0 = pose.x + wx * ch - wy * sh;
  float gy0 = pose.y + wx * sh + wy * ch;
  // 无条件采纳: 每次观测直接覆盖, 画面(单应)相对最可信, 让新观测纠正旧估计。
  g_mem[slot].gx = gx0;
  g_mem[slot].gy = gy0;
  g_mem[slot].t_ms = millis();
  g_mem[slot].stale = 0;
  g_mem[slot].valid = true;
  ai::logf("[ai] 观测 %s 车头系(%.0f,%.0f) → 全局(%.0f,%.0f) [车位姿%d,%d@%d°]", name, wx, wy,
           g_mem[slot].gx, g_mem[slot].gy, (int)pose.x, (int)pose.y, (int)pose.hd);
  return true;
}

// AI 每步 move/spin 后调用: 按定距/定角近似累积车姿态; 无定距/定角的持续移动位移未知, 不改姿态。
void car_update_pose(const char* type, const JsonObjectConst& p) {
  if (!strcmp(type, "move")) {
    float th = p["throttle"] | 0.0f;
    int cm = p["distance_cm"] | 0;
    if (cm > 0 && fabsf(th) > 0.001f) {
      float d = cm * (th < 0 ? -1.0f : 1.0f);        // 后退反向
      float h = s_car_heading * AI_PI / 180.0f;
      s_car_x -= sinf(h) * d;
      s_car_y += cosf(h) * d;
      ai::logf("[ai] 车位置(%.0f,%.0f) 车向%d°", s_car_x, s_car_y, (int)s_car_heading);
    }
  } else if (!strcmp(type, "spin")) {
    int dir = p["dir"] | 0;
    int ang = p["angle_deg"] | 0;
    if (dir != 0 && ang > 0) {
      float h0 = s_car_heading * AI_PI / 180.0f;        // 转前朝向
      int d = dir > 0 ? -ang : ang;                     // 右转=顺时针=heading 减小
      s_car_heading += d;
      if (s_car_heading > 180) s_car_heading -= 360;
      else if (s_car_heading < -180) s_car_heading += 360;
      // 原地旋转绕车几何中心(SPIN_PIVOT_BEHIND_CM)而非车头原点: 车头原点本身会移动, 只改朝向
      // 会让记忆坐标跟着漂。车头恒在枢轴前方 c, 转 d° 后位移 = c·(sin h0 - sin h1, cos h1 - cos h0)。
      float h1 = s_car_heading * AI_PI / 180.0f;        // 转后朝向
      float c = SPIN_PIVOT_BEHIND_CM;
      s_car_x += c * (sinf(h0) - sinf(h1));
      s_car_y += c * (cosf(h1) - cosf(h0));
      ai::logf("[ai] 车位置(%.0f,%.0f) 车向%d°", s_car_x, s_car_y, (int)s_car_heading);
    }
  }
}

// AI 观测(屏幕归一化像素 px,py): 单应解算车头系坐标后入表。pose=拍图时的车位姿(空=用当前实时位姿)。
// 返回 false = 像素越界/解算失败, 调用方须回告 AI(否则它会以为记下了)。
bool mem_observe_xy(const char* name, bool visible, float px, float py,
                    float* out_right, float* out_fwd, const CarPose* pose) {
  if (!visible || !name[0]) {   // 未见/缺名字: 不算失败(由 mem_tick_stale 每轮 +1)
    if (name[0] && !visible) ai::logf("[ai] 观测 %s 不可见", name);
    return true;
  }
  float wx, wy;
  if (!ground::screen_to_world(px, py, &wx, &wy)) {
    ai::logf("[ai] 观测 %s 像素(%.2f,%.2f) 越界/解算失败", name, px, py);
    return false;
  }
  if (out_right) *out_right = wx;   // 本文件口径: wx=右+左-, wy=前+后-
  if (out_fwd) *out_fwd = wy;
  CarPose live{ s_car_x, s_car_y, s_car_heading };
  return mem_store(name, wx, wy, pose ? *pose : live);   // 记不进去要如实回 false(调用方会回告 AI)
}

// 每轮结束: 所有物体未观测则过期轮数 +1(过期不清空, 仅标记)。
void mem_tick_stale(void) {
  for (int i = 0; i < AI_MEM_MAX; i++)
    if (g_mem[i].valid) g_mem[i].stale++;
}

// 导出车姿态 + 物体记忆为 JSON: 手机端「画面源 = 记忆」时据此画车头系俯视图。
// 坐标与 mem_feed 完全同一套逆变换(车头系: 右+/前+, cm), 只读。
void mem_export(JsonObject out) {
  out["x"] = s_car_x;
  out["y"] = s_car_y;
  out["hd"] = (int)s_car_heading;
  JsonArray objs = out["objs"].to<JsonArray>();
  float h = s_car_heading * AI_PI / 180.0f;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (!g_mem[i].valid || !ps_str(g_mem[i].name)[0]) continue;   // 空槽/无名不导出
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    JsonObject o = objs.add<JsonObject>();
    o["name"] = g_mem[i].name;
    o["r"] = cosf(h) * dx + sinf(h) * dy;    // 车头系: 右+ 左-
    o["f"] = -sinf(h) * dx + cosf(h) * dy;   // 车头系: 前+ 后-
    o["stale"] = (int)g_mem[i].stale;
  }
}

// 生成喂给 AI 的空间记忆文本: 小车的全局 (x,y) 与朝向, 再按新鲜度(最近更新的在前)列出各物体。
// ⚠️ 两种坐标系别混: 小车报的是**全局** (x,y); 物体位置一栏是**车头系**(按提示词的定义 x=车正前, y=车正右, cm)。
// 久未更新的条目照旧列出(不设喂回上限, 免得找早先记下的物体时找不着), 过期程度由"N轮未更新"表达。
void mem_feed(char* buf, size_t cap) {
  // 朝向: 说成"相对初始转了多少"(左正右负), 供模型判断转向量, 不是物体方位依据。
  int hd = (int)s_car_heading;
  int n = (hd == 0) ? snprintf(buf, cap, "小车 (%.0f, %.0f) 朝向: 与初始时同向", s_car_x, s_car_y)
                    : snprintf(buf, cap, "小车 (%.0f, %.0f) 朝向: 相对初始时%s%d°",
                               s_car_x, s_car_y, hd > 0 ? "左转" : "右转", hd > 0 ? hd : -hd);
  if (n < 0) n = 0;
  if (n > (int)cap - 1) {                      // 头一句就被截断(缓冲过小): 别让下面的写入越界
    buf[cap - 1] = 0;
    utf8_clamp_tail(buf);                      // 截断处回退到字符边界, 免得中间留半个汉字变 '?'
    n = (int)strlen(buf);
  }
  // 每条先写进本地缓冲, 整条装得下才收: 半句会切掉末尾的"未更新轮数", 甚至切在多字节字符中间。
  // rec 要容下一条(名字无长度上限了, 见 g_mem 声明): 512 ≈ 名字 130 汉字 + 坐标 + 未更新提醒。
  // 再长的条目直接跳过不列(下一行会打日志), 且调用方缓冲本来也装不下 —— 不在这里切半条。
  char rec[512];
  bool used[AI_MEM_MAX] = {false};
  bool first = true;
  for (int pass = 0; pass < AI_MEM_MAX; pass++) {
    int i = -1;                                // 每趟挑最"新鲜"的一条: 未观测轮数最少者在前
    for (int k = 0; k < AI_MEM_MAX; k++) {
      if (!g_mem[k].valid || !ps_str(g_mem[k].name)[0] || used[k]) continue;   // 空槽/无名不参与渲染
      if (i < 0 || g_mem[k].stale < g_mem[i].stale) i = k;
    }
    if (i < 0) break;
    used[i] = true;
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    float h = s_car_heading * AI_PI / 180.0f;
    float wx = cosf(h) * dx + sinf(h) * dy;    // 车头系: 右+ 左-(mem_store 的逆变换)
    float wy = -sinf(h) * dx + cosf(h) * dy;   // 车头系: 前+ 后-
    char pos[32];
    snprintf(pos, sizeof(pos), "[%.1f, %.1f]", wy, wx);   // (前, 右) —— 恒给真实车头系坐标
    // 未更新轮数: 见过即清零; 到 AI_MEM_REMIND_STALE 轮时补一次"看到就更新"的提醒(只此一次)。
    char tail[72] = {0};
    if (g_mem[i].stale > 0) {
      if (g_mem[i].stale == AI_MEM_REMIND_STALE)
        snprintf(tail, sizeof(tail), " (%d轮未更新, 看到请用 observe 更新位置)", (int)g_mem[i].stale);
      else
        snprintf(tail, sizeof(tail), " (%d轮未更新)", (int)g_mem[i].stale);
    }
    int rl = snprintf(rec, sizeof(rec), "%s%s: %s%s; ",
                      first ? " | " : "", ps_str(g_mem[i].name), pos, tail);
    if (rl <= 0) continue;                     // 没写出东西, 不必占位
    // 单条自己就超了 rec: 整条跳过, 不要"截半条"—— 半条不仅会切掉末尾的未更新轮数, 还会切在
    // 多字节字符中间(残尾会被出口消毒变成 '?')。留一条日志, 别让"某个物体莫名不在列表里"。
    if (rl >= (int)sizeof(rec)) {
      ai::logf("[ai] 记忆条目过长未列出(%d 字节约): %s", rl, ps_str(g_mem[i].name));
      continue;
    }
    if (n + rl >= (int)cap - 1) {              // 装不下: 明说后面还有, 而不是把某条切成半句
      const char* more = " | (空间不足, 后面还有物体记忆未列出)";
      int ml = (int)strlen(more);
      if (n + ml < (int)cap - 1) { memcpy(buf + n, more, ml); n += ml; }
      break;
    }
    memcpy(buf + n, rec, rl);
    n += rl;
    first = false;
  }
  buf[n] = '\0';
  ai::logf("[ai] 记忆 %s", buf);   // 喂回内容同步到 ai_log, 便于观察 AI 看到的物体位置理解
}

// 定位目标全局坐标: name 非空=按名匹配(找不到返回 false); name 空=取离车最近的合法记忆。
bool mem_find(const char* name, float* tx, float* ty) {
  int best = -1; float bd = 0.0f;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (!g_mem[i].valid) continue;
    if (name && name[0]) {
      if (name_same_obj(ps_str(g_mem[i].name), name)) { *tx = g_mem[i].gx; *ty = g_mem[i].gy; return true; }
      continue;
    }
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    float d = dx * dx + dy * dy;
    if (best < 0 || d < bd) { bd = d; best = i; }
  }
  if (name && name[0]) return false;   // 指定的名字不在记忆里
  if (best < 0) return false;          // 记忆里没有任何物体
  *tx = g_mem[best].gx; *ty = g_mem[best].gy;
  return true;
}

// 删除一个物体记忆(按名匹配, 判据与 mem_store/mem_find 同一套)。返回是否真删掉了。
// ⚠️ 匹配上的槽**全部**清掉: 同一物体在记忆里可能占了两条(名字差一个修饰词时入库也不并), 只清一条
// 会留下"删了却还在"。这也正是本功能要治的"重复记录同一物体"。
bool mem_forget(const char* name) {
  if (!name || !name[0]) return false;
  bool any = false;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (!g_mem[i].valid || !ps_str(g_mem[i].name)[0]) continue;
    if (!name_same_obj(g_mem[i].name, name)) continue;
    g_mem[i].valid = false;                 // 槽位就此空闲(各读口都先看 valid), 留给 mem_store 复用
    ps_free(g_mem[i].name);                 // 名字在 PSRAM 上, 必须显式放(不是数组了)
    g_mem[i].stale = 0;
    g_mem[i].gx = 0; g_mem[i].gy = 0;
    any = true;
  }
  return any;
}

// 新任务起点: 车位置=原点、车头=0°, 清空上一任务的物体记忆。
void mem_reset() {
  s_car_x = 0; s_car_y = 0; s_car_heading = 0;
  for (int i = 0; i < AI_MEM_MAX; i++) ps_free(g_mem[i].name);   // 名字在 PSRAM: 先逐个放掉再清零
  memset(g_mem, 0, sizeof(g_mem));
}

}  // namespace ai