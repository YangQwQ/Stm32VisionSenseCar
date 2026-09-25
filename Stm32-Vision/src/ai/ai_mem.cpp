#include "src/ai/ai_mem.h"
#include "src/ai/ai_client.h"      // ai::logf(AI 调试日志)
#include "src/ai/ground_proj.h"    // ground::screen_to_world(mem_observe_xy 用)
#include "src/core/board_log.h"    // blog 类别(AI 日志经 ai::logf 内部使用)
#include "Calibration.h"           // SPIN_PIVOT_BEHIND_CM(原地旋转车头位移折算)

#include <math.h>        // cosf/sinf/fmodf/fabsf
#include <stdarg.h>      // va_list/vsnprintf(mem_feed 整条装得下才收)
#include <string.h>      // memset/strncpy/memcpy/strlen

// 物体记忆表(通用)。程序存全局坐标, 喂给 AI 时换算成车头局部系, AI 零换算。stale 每轮未观测 +1。
#define AI_MEM_MAX 8
// 记忆喂回时效: 未观测轮数 ≤AI_MEM_COORD_STALE 给精确坐标, 到 AI_MEM_FEED_STALE 为止只提示"最近未见", 再久不再喂。
#define AI_MEM_FEED_STALE 10
// 给精确坐标的时效: 取 ai_mem.h 的 AI_GRASP_STALE, 与记忆行/夹取口径一一对应。
#define AI_MEM_COORD_STALE AI_GRASP_STALE
static struct {
  char name[16];
  float gx, gy;        // 全局坐标 cm(每次观测直接覆盖)
  uint32_t t_ms;       // 最近观测时刻
  int16_t stale;       // 0=新鲜; 每轮未观测 +1; 超喂回线不再喂
  bool valid;
  int32_t obs_rev;                   // 最近一次观测时的车姿态版本号(见 s_pose_rev)
  float odo_cm, odo_deg;             // 最近一次观测时的路程/转角累积(差值=该坐标被推算推了多远)
} g_mem[AI_MEM_MAX];

namespace ai {

float s_car_x = 0, s_car_y = 0;
int16_t s_car_heading = 0;

// 车姿态版本号: 每累积一次实际位移/转向 +1, 观测时记入记忆条目。
static int32_t s_pose_rev = 0;

// 路程/转角累积(绝对值累加, 不抵消): 与观测时的值相减, 判这条记忆的坐标是否还配得上"车没动过"。
static float s_odo_cm = 0, s_odo_deg = 0;

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

// 把一次"车头系坐标"观测写入物体记忆表: 用观测时刻的车位姿转成全局坐标, 每次观测无条件覆盖。
static void mem_store(const char* name, float wx, float wy) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (g_mem[i].valid && g_mem[i].name[0] && name_same_obj(g_mem[i].name, name)) { slot = i; break; }
    if (!g_mem[i].valid) { slot = i; break; }
    if (g_mem[i].t_ms < g_mem[oldest].t_ms) oldest = i;
  }
  if (slot < 0) slot = oldest;                        // 满: 覆盖最旧
  // 非按名命中(空槽/复用最旧槽)说明内容属于别的物体: 当新条目用, 下面各字段整体覆盖。
  strncpy(g_mem[slot].name, name, 15); g_mem[slot].name[15] = 0;
  // 车头系 (wx右+, wy前+) → 全局: heading 逆时针正(左转+), 前=(-sin,cos)、右=(cos,sin)
  float h = s_car_heading * AI_PI / 180.0f, ch = cosf(h), sh = sinf(h);
  float gx0 = s_car_x + wx * ch - wy * sh;
  float gy0 = s_car_y + wx * sh + wy * ch;
  // 无条件采纳: 每次观测直接覆盖, 画面(单应)相对最可信, 让新观测纠正旧估计。
  g_mem[slot].gx = gx0;
  g_mem[slot].gy = gy0;
  g_mem[slot].t_ms = millis();
  g_mem[slot].stale = 0;
  g_mem[slot].valid = true;
  g_mem[slot].obs_rev = s_pose_rev;
  g_mem[slot].odo_cm = s_odo_cm;                     // 记下"看到它时车走了多远", 供记忆行判这条坐标还算不算数
  g_mem[slot].odo_deg = s_odo_deg;
  ai::logf("[ai] 观测 %s 车头系(%.0f,%.0f) → 全局(%.0f,%.0f)", name, wx, wy,
           g_mem[slot].gx, g_mem[slot].gy);
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
      s_odo_cm += fabsf(d);                          // 路程(绝对值累加): 往返不抵消, 误差两段都累积了
      s_pose_rev++;                                  // 位姿变过 → 此前所有观测的坐标都成了推算值
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
      s_odo_deg += fabsf((float)ang);                   // 转角累积(同路程: 转向误差会留在朝向里)
      s_pose_rev++;
      ai::logf("[ai] 车位置(%.0f,%.0f) 车向%d°", s_car_x, s_car_y, (int)s_car_heading);
    }
  }
}

// AI 观测(rel_deg 相对车头, 正=右、负=左): 换算车头系后入表。
// 返回 false = "可见却没记成"(缺距离), 调用方须回告 AI, 否则它会以为记下了。
bool mem_observe(const char* name, bool visible, float rel_deg, float dist_cm) {
  if (!visible) {   // 未见: 由 mem_tick_stale 每轮 +1, 非错误
    if (name[0]) ai::logf("[ai] 观测 %s 不可见", name);
    return true;
  }
  if (!name[0]) return true;                    // 缺名字: 由调用方单独回告
  if (dist_cm <= 0) {                           // 可见但没给距离: 无法定位, 丢弃并回告
    ai::logf("[ai] 观测 %s 无可用距离(%.0f), 未记录", name, dist_cm);
    return false;
  }
  float r = rel_deg * AI_PI / 180.0f;
  mem_store(name, sinf(r) * dist_cm, cosf(r) * dist_cm);   // rel 正=右 → wx 右+
  return true;
}

// AI 观测(屏幕归一化像素 px,py): 单应解算车头系坐标后入表(精度远高于目测距离)。
// 返回 false = 像素越界/解算失败, 调用方应回退 rel_deg 路径。
bool mem_observe_xy(const char* name, bool visible, float px, float py) {
  if (!visible || !name[0]) {   // 未见/缺名字: 不算失败(由 mem_tick_stale 每轮 +1)
    if (name[0] && !visible) ai::logf("[ai] 观测 %s 不可见", name);
    return true;
  }
  float wx, wy;
  if (!ground::screen_to_world(px, py, &wx, &wy)) {
    ai::logf("[ai] 观测 %s 像素(%.2f,%.2f) 越界/解算失败", name, px, py);
    return false;
  }
  mem_store(name, wx, wy);
  return true;
}

// 每轮结束: 所有物体未观测则过期轮数 +1(过期不清空, 仅标记)。
void mem_tick_stale(void) {
  for (int i = 0; i < AI_MEM_MAX; i++)
    if (g_mem[i].valid) g_mem[i].stale++;
}

// 生成喂给 AI 的空间记忆文本: 当前车头局部系坐标(右+/左-、前+/后-, cm), AI 零换算。
void mem_feed(char* buf, size_t cap) {
  // 朝向: 说成"相对初始转了多少"(左正右负), 供模型判断转向量, 不是物体方位依据。
  int hd = (int)s_car_heading;
  int n = (hd == 0)? snprintf(buf, cap, "朝向: 与初始时同向"):snprintf(buf, cap, "朝向: 相对初始时%s%d°", hd > 0 ? "左转" : "右转", hd > 0 ? hd : -hd);
  // 每条先写进本地缓冲, 整条装得下才收: 半句会切掉末尾的"该怎么做", 甚至切在多字节字符中间。
  // rec 容量须容下最长一条; 栈上这点开销无妨(调用方 ai_worker 栈 16384)。
  char rec[384];
  auto add = [&](const char* fmt, ...) -> bool {
    va_list ap;
    va_start(ap, fmt);
    int rl = vsnprintf(rec, sizeof(rec), fmt, ap);
    va_end(ap);
    if (rl <= 0) return true;                       // 没写出东西, 不必占位
    if (rl > (int)sizeof(rec) - 1) rl = sizeof(rec) - 1;   // 仅超长物体名会走到(名字本身另有上限)
    if (n + rl >= (int)cap - 1) return false;       // 装不下: 交给调用处收尾
    memcpy(buf + n, rec, rl);
    n += rl;
    return true;
  };
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (!g_mem[i].valid || g_mem[i].stale > AI_MEM_FEED_STALE) continue;   // 久未观测不再喂, 防拿旧坐标瞎猜
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    float h = s_car_heading * AI_PI / 180.0f;
    float wx = cosf(h) * dx + sinf(h) * dy;   // 车头系: 右+ 左-(mem_store 的逆变换)
    float wy = -sinf(h) * dx + cosf(h) * dy;  // 车头系: 前+ 后-
    bool ok;
    if (g_mem[i].stale <= AI_MEM_COORD_STALE) {
      if (wy > 0 && wy <= AI_NEAR_FWD_CM) {
        // 近场(前距 ≤AI_NEAR_FWD_CM): 一带 cm 高报且抖, 只给远近档位不给坐标, 对准按画面两指判断。
        // 措辞与提示词同款: wy≤AI_ARM_UNDER_CM 为 [过近], 其上到近场线为 [接近]。
        const char* nstate = (wy <= AI_ARM_UNDER_CM) ? "[过近]" : "[接近]";
        ok = add("; %s 已%s", g_mem[i].name, nstate);
      } else {
        // 最近观测过且不在近场: 给精确坐标(供快速找回当前方位)
        ok = add("; %s(%s%.1fcm,%s%.1fcm)",
                 g_mem[i].name, wx >= 0 ? "右" : "左", fabsf(wx),
                 wy >= 0 ? "前" : "后", fabsf(wy));
        // 附实时反投影的屏幕位置, 供 AI 去该处核对目标: 车一动目标画面位置就变, 存观测时的 px/py 会误导。
        if (ok) {
          float su, sv;
          if (ground::world_to_screen(wx, wy, &su, &sv)) {
            ai::logf("[ai] 目标 %s 车头(%.0f,%.0f) → 屏幕(%.2f,%.2f)", g_mem[i].name, wx, wy, su, sv);
            ok = add(" 屏幕[%.2f,%.2f]", su, sv);
          }
        }
      }
    } else if (s_odo_cm - g_mem[i].odo_cm <= AI_GRASP_MOVE_TOL_CM &&
               s_odo_deg - g_mem[i].odo_deg <= AI_GRASP_TURN_TOL_DEG) {
      {
        const char* st;
        if (wy <= AI_ARM_UNDER_CM) st = "已[过近]";
        else if (wy <= AI_NEAR_FWD_CM) st = "已[接近]";
        else st = "[较远]";
        ok = add("; %s %s(%d轮前看到, 车没动; 看不见先动臂查遮挡, 还看不见才小步后退, 别一丢就退)",
                 g_mem[i].name, st, (int)g_mem[i].stale);
      }
    } else {
      // 久未观测: 不给坐标, 只提醒位置已过时(这种时效下夹取会被拒, 先说清省得白试)。
      ok = add("; %s(已%d轮未见, 位置过时: 不能凭它夹)", g_mem[i].name, (int)g_mem[i].stale);
    }
    if (!ok) {   // 空间不足: 一句话说清后面还有, 而不是把某条切成半句
      if (n < (int)cap - 32) {
        const char* more = "; (空间不足, 后面还有物体记忆未列出)";
        int ml = (int)strlen(more);
        if (n + ml < (int)cap - 1) { memcpy(buf + n, more, ml); n += ml; }
      }
      break;
    }
  }
  buf[n] = '\0';
  ai::logf("[ai] 记忆 %s", buf);   // 喂回内容同步到 ai_log, 便于观察 AI 看到的物体位置理解
}

// 记忆里有没有可用物体: 判据与 mem_feed 同源, 免得程序说"没有目标"而记忆行里明明列着。
bool mem_have_any() {
  for (int i = 0; i < AI_MEM_MAX; i++)
    if (g_mem[i].valid && g_mem[i].stale <= AI_MEM_FEED_STALE) return true;
  return false;
}

// 是否存在"曾被观测、现已不可见"的物体: 超新鲜线(AI_GRASP_STALE)但未过喂回线的这一档 ——
// 即"还记得它、但最近没在画面里看到", 供"目标可能被机械臂遮挡"类提示判断。
bool mem_have_lost() {
  for (int i = 0; i < AI_MEM_MAX; i++)
    if (g_mem[i].valid && g_mem[i].stale > AI_GRASP_STALE && g_mem[i].stale <= AI_MEM_FEED_STALE)
      return true;
  return false;
}

// 定位目标全局坐标: name 非空=按名匹配(找不到返回 false); name 空=取离车最近的合法记忆, 均跳过过期。
bool mem_find(const char* name, float* tx, float* ty) {
  int best = -1; float bd = 0.0f;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (!g_mem[i].valid || g_mem[i].stale > AI_MEM_FEED_STALE) continue;   // 久未观测的目标不据旧坐标导航
    if (name && name[0]) {
      if (name_same_obj(g_mem[i].name, name)) { *tx = g_mem[i].gx; *ty = g_mem[i].gy; return true; }
      continue;
    }
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    float d = dx * dx + dy * dy;
    if (best < 0 || d < bd) { bd = d; best = i; }
  }
  if (name && name[0]) return false;   // 指定的名字不在记忆里
  if (best < 0) return false;          // 无任何可用记忆
  *tx = g_mem[best].gx; *ty = g_mem[best].gy;
  return true;
}

// 新任务起点: 车位置=原点、车头=0°, 清空上一任务的物体记忆。
void mem_reset() {
  s_car_x = 0; s_car_y = 0; s_car_heading = 0;
  s_pose_rev = 0;
  s_odo_cm = 0; s_odo_deg = 0;
  memset(g_mem, 0, sizeof(g_mem));
}

}  // namespace ai