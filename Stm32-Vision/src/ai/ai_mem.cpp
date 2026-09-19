#include "src/ai/ai_mem.h"
#include "src/ai/ai_client.h"      // ai::logf(AI 调试日志)
#include "src/ai/ground_proj.h"    // ground::screen_to_world(mem_observe_xy 用)
#include "src/core/board_log.h"    // blog 类别(AI 日志经 ai::logf 内部使用)

#include <math.h>        // cosf/sinf/fmodf/fabsf
#include <string.h>      // memset/strncpy/memcpy/strlen

// 物体记忆表(通用: AI 觉得值得记的都记)。程序存全局坐标, 喂给 AI 时一律换算成
// "当前车头局部系"(相对车头角度+距离), AI 零换算。stale=每轮未观测+1(过期不清空)。
// 每条保留最近 AI_OBS_N 次观测, 取中位数融合——DeepSeek 单次报的像素/角度方差大
// (轮间跳变、reason 与 observe 字段不一致), 中位数对单次离谱值鲁棒, 防记忆被污染。
#define AI_MEM_MAX 8
#define AI_OBS_N 5
// 记忆喂回时效: 未观测轮数 ≤前者仍给精确坐标(供快速找回); 之间只提示"最近未见"不给坐标,
// 避免 AI 长期拿过时坐标去猜; 超过后者彻底不再喂。辨识: 活跃目标一轮一刷 stale≈1。
#define AI_MEM_COORD_STALE 3
#define AI_MEM_FEED_STALE 10
static struct {
  char name[16];
  float gx, gy;        // 全局坐标 cm(由观测中位数合成)
  uint32_t t_ms;       // 最近观测时刻
  int16_t stale;       // 0=新鲜; 每轮未观测 +1; >20 时不再喂回
  bool valid;
  float hx[AI_OBS_N], hy[AI_OBS_N];  // 各次观测的全局坐标环形缓冲(入表时已按当时车位姿转换)
  uint8_t hn, hi;                    // 已存数量 / 写指针
} g_mem[AI_MEM_MAX];

namespace ai {

float s_car_x = 0, s_car_y = 0;
int16_t s_car_heading = 0;

// 同一物体判定: 精确同名, 或首字相同且长度差≤3。
static bool name_same_obj(const char* a, const char* b) {
  int la = (int)strlen(a), lb = (int)strlen(b);
  if (la == 0 || lb == 0) return false;
  if (!strcmp(a, b)) return true;
  return a[0] == b[0] && abs(la - lb) <= 3;
}

// 取数组前 n 个元素的中位数(n≤AI_OBS_N, 插入排序后取中间, 仅用于观测融合)
static float median_n(const float* a, int n) {
  float t[AI_OBS_N];
  memcpy(t, a, n * sizeof(float));
  for (int i = 1; i < n; i++) {
    float v = t[i]; int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  return t[n / 2];
}

// 把一次"车头系坐标"观测写入物体记忆表: 入表时立即用**观测时刻**的车位姿转成全局坐标,
// 历史观测各自已是全局坐标, 融合直接取中位数。不再依赖"当前"车位姿, 避免随车移动漂移。
static void mem_store(const char* name, float wx, float wy) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (g_mem[i].valid && g_mem[i].name[0] && name_same_obj(g_mem[i].name, name)) { slot = i; break; }
    if (!g_mem[i].valid) { slot = i; break; }
    if (g_mem[i].t_ms < g_mem[oldest].t_ms) oldest = i;
  }
  if (slot < 0) slot = oldest;                        // 满: 覆盖最旧
  strncpy(g_mem[slot].name, name, 15); g_mem[slot].name[15] = 0;
  // 车头系 (wx右+, wy前+) → 全局: heading 逆时针正(左转+), 前=(-sin,cos)、右=(cos,sin)
  float h = s_car_heading * AI_PI / 180.0f, ch = cosf(h), sh = sinf(h);
  float gx0 = s_car_x + wx * ch - wy * sh;
  float gy0 = s_car_y + wx * sh + wy * ch;
  // 压入本次观测(全局坐标)
  g_mem[slot].hx[g_mem[slot].hi] = gx0;
  g_mem[slot].hy[g_mem[slot].hi] = gy0;
  g_mem[slot].hi = (g_mem[slot].hi + 1) % AI_OBS_N;
  if (g_mem[slot].hn < AI_OBS_N) g_mem[slot].hn++;
  // 历史观测(已是全局)直接取中位数
  float gxl[AI_OBS_N], gyl[AI_OBS_N];
  for (int i = 0; i < g_mem[slot].hn; i++) {
    int idx = (g_mem[slot].hi - g_mem[slot].hn + i + AI_OBS_N) % AI_OBS_N;  // 最旧→最新
    gxl[i] = g_mem[slot].hx[idx];
    gyl[i] = g_mem[slot].hy[idx];
  }
  g_mem[slot].gx = median_n(gxl, g_mem[slot].hn);
  g_mem[slot].gy = median_n(gyl, g_mem[slot].hn);
  g_mem[slot].t_ms = millis();
  g_mem[slot].stale = 0;
  g_mem[slot].valid = true;
  ai::logf("[ai] 观测 %s 车头系(%.0f,%.0f) 融合n=%d → 全局(%.0f,%.0f)", name, wx, wy,
           g_mem[slot].hn, g_mem[slot].gx, g_mem[slot].gy);
}

// AI 每步执行 move/spin 后调用: 按定距/定角近似累积车姿态。持续(无定距/定角)移动
// 位移未知, 不改姿态(记忆会随每轮 stale++ 表不可靠)。
void car_update_pose(const char* type, const JsonObjectConst& p) {
  if (!strcmp(type, "move")) {
    float th = p["throttle"] | 0.0f;
    int cm = p["distance_cm"] | 0;
    if (cm > 0 && fabsf(th) > 0.001f) {
      float d = cm * (th < 0 ? -1.0f : 1.0f);        // 后退反向
      float h = s_car_heading * AI_PI / 180.0f;
      s_car_x += sinf(h) * d;                        // heading=0 → 朝 Y+
      s_car_y += cosf(h) * d;
      ai::logf("[ai] 车位置(%.0f,%.0f) 车向%d°", s_car_x, s_car_y, (int)s_car_heading);
    }
  } else if (!strcmp(type, "spin")) {
    int dir = p["dir"] | 0;
    int ang = p["angle_deg"] | 0;
    if (dir != 0 && ang > 0) {
      int d = dir > 0 ? -ang : ang;                  // 右转=顺时针=heading 减小
      s_car_heading += d;
      if (s_car_heading > 180) s_car_heading -= 360;
      else if (s_car_heading < -180) s_car_heading += 360;
      ai::logf("[ai] 车向%d°", (int)s_car_heading);
    }
  }
}

// AI 观测(rel_deg 相对车头, 正=右、负=左): 换算车头系后入表。
void mem_observe(const char* name, bool visible, float rel_deg, float dist_cm) {
  if (!visible || !name[0] || dist_cm <= 0) {   // 未见: 由 mem_tick_stale 每轮 +1
    if (name[0]) ai::logf("[ai] 观测 %s 不可见", name);
    return;
  }
  float r = rel_deg * AI_PI / 180.0f;
  mem_store(name, sinf(r) * dist_cm, cosf(r) * dist_cm);   // rel 正=右 → wx 右+
}

// AI 观测(屏幕归一化像素 px,py): 单应解算车头系坐标后入表(精度远高于目测距离)。
// 返回 false = 像素越界/解算失败, 调用方应回退 rel_deg 路径。
bool mem_observe_xy(const char* name, bool visible, float px, float py) {
  if (!visible || !name[0]) {   // 未见: 由 mem_tick_stale 每轮 +1
    if (name[0]) ai::logf("[ai] 观测 %s 不可见", name);
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

// 生成喂给 AI 的空间记忆文本: 一律当前车头局部系精确坐标(右+ / 左-、前+ / 后-, 单位 cm), AI 零换算。
// 用精确厘米坐标替代粗略角度+距离: 目测角度在近距离误差会被放大(偏角显示会失真), 厘米更准。
void mem_feed(char* buf, size_t cap) {
  int n = snprintf(buf, cap, "车向:%d°", (int)s_car_heading);
  for (int i = 0; i < AI_MEM_MAX && n < (int)cap - 64; i++) {
    if (!g_mem[i].valid || g_mem[i].stale > AI_MEM_FEED_STALE) continue;   // 久未观测不再喂, 防拿旧坐标瞎猜
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    float h = s_car_heading * AI_PI / 180.0f;
    float wx = cosf(h) * dx + sinf(h) * dy;   // 车头系: 右+ 左-(mem_store 的逆变换)
    float wy = -sinf(h) * dx + cosf(h) * dy;  // 车头系: 前+ 后-
    if (g_mem[i].stale <= AI_MEM_COORD_STALE) {
      // 最近观测过: 给精确坐标(供快速找回当前方位)
      n += snprintf(buf + n, cap - n, "; %s(%s%.1fcm,%s%.1fcm)",
                    g_mem[i].name, wx >= 0 ? "右" : "左", fabsf(wx),
                    wy >= 0 ? "前" : "后", fabsf(wy));
    } else {
      // 久未观测: 不再给坐标, 只提醒它存在但位置已过时, 别据此猜
      n += snprintf(buf + n, cap - n, "; %s(已%d轮未见, 位置过时勿据此猜)",
                    g_mem[i].name, (int)g_mem[i].stale);
    }
  }
  ai::logf("[ai] 记忆 %s", buf);   // 喂回内容同步到 ai_log, 便于观察 AI 看到的物体位置理解
}

// 在物体记忆表里定位目标全局坐标: name 非空=精确/近似名匹配(找不到返回 false);
// name 空=取当前最近的合法记忆。均跳过过期(>20轮)。
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
  memset(g_mem, 0, sizeof(g_mem));
}

}  // namespace ai