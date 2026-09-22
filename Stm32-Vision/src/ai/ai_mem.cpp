#include "src/ai/ai_mem.h"
#include "src/ai/ai_client.h"      // ai::logf(AI 调试日志)
#include "src/ai/ground_proj.h"    // ground::screen_to_world(mem_observe_xy 用)
#include "src/core/board_log.h"    // blog 类别(AI 日志经 ai::logf 内部使用)
#include "Calibration.h"           // SPIN_PIVOT_BEHIND_CM(原地旋转车头位移折算)

#include <math.h>        // cosf/sinf/fmodf/fabsf
#include <stdarg.h>      // va_list/vsnprintf(mem_feed 整条装得下才收)
#include <string.h>      // memset/strncpy/memcpy/strlen

// 物体记忆表(通用: AI 觉得值得记的都记)。程序存全局坐标, 喂给 AI 时一律换算成
// "当前车头局部系"(相对车头角度+距离), AI 零换算。stale=每轮未观测+1(过期不清空)。
// 每条保留最近 AI_OBS_N 次观测, 取中位数融合——DeepSeek 单次报的像素/角度方差大
// (轮间跳变、reason 与 observe 字段不一致), 中位数对单次离谱值鲁棒, 防记忆被污染。
#define AI_MEM_MAX 8
#define AI_OBS_N 5
// 记忆喂回时效: 未观测轮数 ≤前者仍给精确坐标(供快速找回); 之间只提示"最近未见"不给坐标,
// 避免 AI 长期拿过时坐标去猜; 超过后者彻底不再喂。辨识: 活跃目标一轮一刷 stale≈1。
#define AI_MEM_FEED_STALE 10
// 给精确坐标的时效: 直接取夹取闸门那个时效(ai_mem.h 的 AI_GRASP_STALE), 于是 AI 看到记忆行给了
// 厘米坐标就一定能夹、看到"已N轮未见"就一定会被拒, 提示词/记忆行/闸门三者一一对应。
#define AI_MEM_COORD_STALE AI_GRASP_STALE
// 本次观测与融合坐标差超过此值就进**离群判定**(cm): 取一个明显大于单次观测噪声、又明显小于
// "夹不住"的量级 —— 方块才 2~3cm 宽, 差 5cm 已经是"记忆说在这边、画面说在那边"了。
#define AI_MEM_DRIFT_WARN_CM 5.0f
// 判"连续两次被拒的观测互相吻合"的距离(cm): 两次都离谱、但彼此落在这么近, 才认它俩说的是同一个
// 新位置(方块被爪推动时正是这个签名)。取略大于方块尺寸、又远小于离群阈值的值 —— 实测坏观测的散布
// 能到 24cm(同一方块、车没动, 前距报出 3.7~27.9cm), 所以"两次都离谱且彼此差得远"不算吻合, 继续拒。
#define AI_MEM_REJECT_AGREE_CM 3.0f
static struct {
  char name[16];
  float gx, gy;        // 全局坐标 cm(由观测中位数合成)
  uint32_t t_ms;       // 最近观测时刻
  int16_t stale;       // 0=新鲜; 每轮未观测 +1; >20 时不再喂回
  bool valid;
  float hx[AI_OBS_N], hy[AI_OBS_N];  // 各次观测的全局坐标环形缓冲(入表时已按当时车位姿转换)
  uint8_t hn, hi;                    // 已存数量 / 写指针
  int32_t obs_rev;                   // 最近一次观测时的车姿态版本号(见 s_pose_rev)
  float odo_cm, odo_deg;             // 最近一次观测时的路程/转角累积值(见 s_odo_*): 两个差值就是
                                     // "这条坐标从看到到现在被推算推了多远", 闸门据此判它还算不算数
  uint8_t out_n;                     // 连续被判离群的观测数(见 mem_store 的离群判定); 正常观测清零
  float rej_gx, rej_gy;              // 上一次**被拒**观测的全局坐标(判"连续两次是否互相吻合")
} g_mem[AI_MEM_MAX];

namespace ai {

float s_car_x = 0, s_car_y = 0;
int16_t s_car_heading = 0;

// 车姿态版本号: 每累积一次实际位移/转向 +1。观测时把当时的版本号记进记忆, 闸门据此判断"这条坐标
// 之后车还动过没有" —— 动过就意味着它是推算出来的, 不能再当画面用(见 mem_grasp_evidence)。
static int32_t s_pose_rev = 0;

// 路程/转角累积(绝对值累加, 不抵消): 与版本号同时记录, 但比它多带一个"推了多远"的尺度 ——
// "动过没有"是一刀切(挪 1cm 和转 40° 同样作废), 而推算误差是**随位移增长**的, 1cm 内那条坐标
// 其实还准。闸门用差值判"还在容差内"(见 ai_mem.h AI_GRASP_MOVE_TOL_CM)。
static float s_odo_cm = 0, s_odo_deg = 0;

// 同一物体判定: 精确同名; 或"首字相同 + 一者名字整体包含另一者的剩余部分"——容忍 AI 给同一物体
// 加/减修饰词(如"红方块"↔"红色方块")。
// 旧规则为"首字相同且长度差≤3", 会把"水瓶/水杯"这类同首字但**不同**的物体并成同一条记忆:
// 位置被互相覆盖, AI 于是记错物体/走向错误位置(靠近后怎么都对不上)。故收紧为包含关系,
// 宁可拆成两条(名字各自稳定时仍按名精确命中), 也不能把两个物体混成一条。
static bool name_same_obj(const char* a, const char* b) {
  int la = (int)strlen(a), lb = (int)strlen(b);
  if (la == 0 || lb == 0) return false;
  if (la == lb && !strcmp(a, b)) return true;
  if (la < 4 || lb < 4) return false;            // 首字(3 字节)+至少 1 字节余量才谈包含
  if (memcmp(a, b, 3)) return false;             // 首字不同: 视为不同物体
  return (la > lb && strstr(a + 3, b + 3)) || (lb > la && strstr(b + 3, a + 3));
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
  // ---------------- 离群判定 ----------------
  // 排在入表**之前**: "拒收"就是这条压根不进环缓冲, 融合值与记账一概不动。
  // gap 比的是**本条 vs 入表前的融合值**。旧代码先压入再算, 拿"含本条的中位数"当基准 —— 基准
  // 本身已被这条拉偏, 判据因此偏松(这是"检出异常后反而采纳了异常"的一半原因), 故顺序一并纠正。
  float gap = sqrtf((gx0 - g_mem[slot].gx) * (gx0 - g_mem[slot].gx) +
                    (gy0 - g_mem[slot].gy) * (gy0 - g_mem[slot].gy));
  if (g_mem[slot].hn >= 2 && gap > AI_MEM_DRIFT_WARN_CM) {
    // 自这条记忆上次被看到以来, 车挪了多远/转了多少 —— 容差直接取闸门那一条(ai_mem.h), 保持同源:
    // "这条坐标还算不算看到的位置"与"大落差是否可解释"用的是同一个尺度。
    float moved_cm  = s_odo_cm  - g_mem[slot].odo_cm;
    float moved_deg = s_odo_deg - g_mem[slot].odo_deg;
    // 大落差有两种**截然不同**的成因, 必须分开:
    //  ① 车动过 ⇒ 历史全局坐标是"当时位姿 + 当时数值"的产物, 位姿累积误差已让它整体错位, 而新观测
    //     来自画面(单应), 是相对最可信的证据 ⇒ 采信新样本、重置历史(原行为, 保留)。
    bool car_moved = moved_cm > AI_GRASP_MOVE_TOL_CM || moved_deg > AI_GRASP_TURN_TOL_DEG;
    //  ② 车**没动** ⇒ 画面根本没变, 同一个方块还在同一个屏幕位置上, 十几厘米的落差只可能来自
    //     **这一次观测自己报错**。硬证据(2026-09-22): 两帧同一屏幕位置(0.632,0.403)、车一步没动,
    //     喂回的前距却从 18.3cm 变成 3.4cm, 而 kGroundCal 复算真值约 27cm ⇒ 错的是模型那一次报的
    //     px/py, **不是**位姿漂移(旧日志把它归因成"车姿态累积疑已漂移", 归因本身也错了)。
    //     ⇒ **拒收**。但**连续两次互相吻合**的离群观测要认账 —— 方块被爪推动时正是这个签名(车没动、
    //     画面真变了): 两次都离谱且彼此落在 AI_MEM_REJECT_AGREE_CM 内 ⇒ 判历史已失效, 走①的采信+重置。
    bool confirmed = g_mem[slot].out_n >= 1 &&
                     sqrtf((gx0 - g_mem[slot].rej_gx) * (gx0 - g_mem[slot].rej_gx) +
                           (gy0 - g_mem[slot].rej_gy) * (gy0 - g_mem[slot].rej_gy))
                       <= AI_MEM_REJECT_AGREE_CM;
    if (!car_moved && !confirmed) {
      // 拒收: 坐标/融合值原样保留(它才是画面与历史都支持的那个), 但"这一轮看到过它"照记 ——
      // 否则 AI 正看着它、记忆行却报"已N轮未见", 两边说法打架(同源不变量)。
      // odo 也**不动**: 保留的坐标仍来自那一次旧观测, "此后车挪了多远"要从那时候算起。
      g_mem[slot].stale  = 0;
      g_mem[slot].t_ms   = millis();
      g_mem[slot].out_n++;
      g_mem[slot].rej_gx = gx0; g_mem[slot].rej_gy = gy0;
      ai::logf("[ai] 拒收 %s 本次观测: 与融合坐标差 %.0fcm, 而车此后只挪了 %.0fcm/转 %.0f°(画面没变)"
               " ⇒ 判**本次观测**不可信(非姿态漂移), 保留原估计(全局 %.0f,%.0f; 连续第%d次)",
               name, gap, moved_cm, moved_deg, g_mem[slot].gx, g_mem[slot].gy, (int)g_mem[slot].out_n);
      return;
    }
    if (car_moved)
      ai::logf("[ai] %s 观测与融合坐标差 %.0fcm, 但车此后挪了 %.0fcm/转 %.0f° ⇒ 落差可解释, 采信本次并重置历史",
               name, gap, moved_cm, moved_deg);
    else
      ai::logf("[ai] %s 连续两次观测互相吻合、且都与历史差 %.0fcm(车未动) ⇒ 判历史已失效, 采信新观测并重置历史",
               name, gap);
    // 重置: 只留本条观测。hn=1 且 gx/gy=本条, 后续观测从干净的起点重新融合。
    g_mem[slot].hn = 1;
    g_mem[slot].hx[0] = gx0;
    g_mem[slot].hy[0] = gy0;
    g_mem[slot].hi = 1;
    g_mem[slot].gx = gx0;
    g_mem[slot].gy = gy0;
    g_mem[slot].t_ms = millis();
    g_mem[slot].stale = 0;
    g_mem[slot].valid = true;
    g_mem[slot].obs_rev = s_pose_rev;
    g_mem[slot].odo_cm = s_odo_cm;
    g_mem[slot].odo_deg = s_odo_deg;
    g_mem[slot].out_n = 0;
    return;
  }
  g_mem[slot].out_n = 0;      // 正常观测: 清掉离群计数(见上)
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
  g_mem[slot].obs_rev = s_pose_rev;
  g_mem[slot].odo_cm = s_odo_cm;                     // 记下"看到它时车走了多远", 供闸门算推算误差
  g_mem[slot].odo_deg = s_odo_deg;
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
      // 前进方向 = mem_store 记录的车头系前向 (-sin, cos)(即 heading=0 时朝 Y+)。
      // 注意 X 是减号: 车头右转(heading 减小/顺时针)时前进应朝 +X 走, -sin(h) 在 h<0 时为正。
      // 曾误写 += sinf(h), 使"先左转-前进-再右转回正"这类往返运动在 X 上累积出约 2×行进距离的
      // 镜像误差, 喂回给 AI 的左右方位整体反相(实测表现为"物体在右却说在左"), 并让 approach/goto
      // 朝镜像坐标导航 → 冲过头/目标进死角。勿改回。
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
      // 原地旋转绕车几何中心(SPIN_PIVOT_BEHIND_CM)而非车头原点: 旋转时车头原点本身会移动,
      // 只改朝向不改位置会让记忆坐标跟着"没动的原点"整体漂移(实测转90°漂到(5,-5)~(5,-6.5))。
      // 车头相对枢轴恒在"前方 c cm", 转 d° 后其新位置 = R(d°)(车头矢量), 全局换算得:
      //   车头位移 = c·(sin h0 - sin h1, cos h1 - cos h0)   (x右+ / y前+)
      // 右转(h1<h0)→ 车头向右后漂, 左转→向左后漂 —— 与实测方向一致。
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
// 返回 false = "可见却没记成"(缺距离), 调用方据此回告 AI —— 否则 AI 以为记下了,
// 却在下一轮的记忆里找不到该物体, 于是反复重新 observe 或凭猜乱走。
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

// 生成喂给 AI 的空间记忆文本: 一律当前车头局部系精确坐标(右+ / 左-、前+ / 后-, 单位 cm), AI 零换算。
// 用精确厘米坐标替代粗略角度+距离: 目测角度在近距离误差会被放大(偏角显示会失真), 厘米更准。
void mem_feed(char* buf, size_t cap) {
  // 车头朝向: 说成"相对任务开始转了多少"(左正右负), 而不是裸角度(如"车向:-90°")——模型无法把
  // 裸角度对应到画面所见, 只会徒增困惑/被它当方位依据。下面每个物体的坐标**已经是当前车头系**,
  // 模型零换算, 此项仅供其判断"我转向了多少 / 起点在哪个方位", 不是物体方位的依据。
  int hd = (int)s_car_heading;
  int n = (hd == 0)
            ? snprintf(buf, cap, "车头: 与任务开始时同向")
            : snprintf(buf, cap, "车头: 相对任务开始时%s%d°",
                       hd > 0 ? "左转" : "右转", hd > 0 ? hd : -hd);
  // 每条先写进本地缓冲, 再整体决定收不收: **整条装得下才收**, 装不下就停手并说一句"还有未列出"。
  // 原先直接 snprintf 进 buf, 一条写不下就被切成半句 —— 而被切掉的恰是末尾那半句"该怎么做"
  // (实测末尾只剩"...近处被自身夹爪挡住看不见是常"), 等于把最关键的指令吃掉, 比不写还坏
  // (它读到一个没说完的句子, 只能自己猜)。半句还可能切在多字节字符中间。
  char rec[256];
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
      // 最近观测过: 给精确坐标(供快速找回当前方位)
      ok = add("; %s(%s%.1fcm,%s%.1fcm)",
               g_mem[i].name, wx >= 0 ? "右" : "左", fabsf(wx),
               wy >= 0 ? "前" : "后", fabsf(wy));
    } else if (s_odo_cm - g_mem[i].odo_cm <= AI_GRASP_MOVE_TOL_CM &&
               s_odo_deg - g_mem[i].odo_deg <= AI_GRASP_TURN_TOL_DEG) {
      // 这几轮没再看到它, 但车几乎没动: 那条坐标仍是"看到的位置"(与闸门同一判据, 见 ai_mem.h)。
      // 要说清两件事, 少一件都会被 AI 走偏:
      //  (1) **不等于对准** —— 这句以前写的是"照它夹", 实测 AI 拿它当合爪依据(前场坐标有 1~2cm
      //      级系统偏差, 实测差 1.3cm 时方块还在两指指尖前方两三厘米), 连夹两次全空。
      //  (2) **别只为看它而后退** —— 实测 AI 为找回画面会后退, 退出去目标又变远, 来回摆(整轮 90s
      //      没走到过 arm low)。近处被两指/车头挡住本就是常态。
      // 所以: 坐标只用来判"方位没变", 对准一律看画面。措辞要短: 这条最长, 而 mem_s 预算还得分给别人。
      ok = add("; %s(%s%.1fcm,%s%.1fcm; %d轮前看到, 车只挪%.0fcm/转%.0f° → 坐标没过时但**≠对准**; "
               "看不见多是被两指挡住(正常, 别后退找); 降臂后 zoom 看两指之间, 在里面就 clip, 不在就小步补)",
               g_mem[i].name, wx >= 0 ? "右" : "左", fabsf(wx),
               wy >= 0 ? "前" : "后", fabsf(wy), (int)g_mem[i].stale,
               s_odo_cm - g_mem[i].odo_cm, s_odo_deg - g_mem[i].odo_deg);
    } else {
      // 久未观测: 不再给坐标, 只提醒它存在但位置已过时。措辞与闸门对应: 这种时效下 arm_pose
      // 下探/arm clip 会被程序直接拒绝(见 mem_grasp_evidence), 先说清楚省得它白试一轮。
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

// 夹取依据(见 ai_mem.h): 取时效内"最新鲜"的那条记忆(不判远近/方位)。新鲜度优先, 同新鲜度再取
// 最近观测的 —— AI 要拿它的坐标去定位夹爪, 取到最旧那条等于自己给自己加误差。
bool mem_grasp_evidence(char* name, size_t cap, int* age_rounds, float* fwd_cm, float* lat_cm,
                        float* moved_cm, float* turned_deg) {
  int best = -1;
  float h = s_car_heading * AI_PI / 180.0f, ch = cosf(h), sh = sinf(h);
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (!g_mem[i].valid) continue;
    // 时效判据 = "轮数" **或** "车此后挪了多远", 两者取宽 —— 必须与 mem_feed 的措辞同源:
    // 那边写着"坐标没过时"的记录, 这边就不能拒(否则又是 AI 无论如何满足不了的指令)。
    // 物理上定坐标还准不准的是**车挪了多少**: 车没动就等于画面没变, 那条坐标和刚看到时一样准,
    // 过几轮不影响; 一旦挪出容差(见 AI_GRASP_*_TOL_*), 它才退化成"看到的位置 + 一段推算"。
    // 轮数只留一条上限: 连喂都不喂给 AI 的记录(> AI_MEM_FEED_STALE)不许当依据 —— 它自己都不知道
    // 有这个坐标, 拿它去夹等于程序替它瞎猜。
    if (g_mem[i].stale > AI_MEM_FEED_STALE) continue;
    bool fresh = g_mem[i].stale <= AI_GRASP_STALE;
    bool still = s_odo_cm - g_mem[i].odo_cm <= AI_GRASP_MOVE_TOL_CM &&
                 s_odo_deg - g_mem[i].odo_deg <= AI_GRASP_TURN_TOL_DEG;
    if (!fresh && !still) continue;
    if (best < 0 || g_mem[i].stale < g_mem[best].stale ||
        (g_mem[i].stale == g_mem[best].stale && g_mem[i].t_ms > g_mem[best].t_ms)) best = i;
  }
  if (best < 0) return false;
  if (name && cap) { strncpy(name, g_mem[best].name, cap - 1); name[cap - 1] = 0; }
  // ⚠️ 坐标必须与 mem_feed 喂给 AI 的**是同一个数**(融合中位数 gx/gy), 这是硬不变量, 别改成"最近
  // 一次观测": 闸门判的是"AI 照它拿到的坐标算出来的动作", 两边不同源就会造出**AI 无论如何都满足
  // 不了**的指令。实测: 同一轮喂回 8.7cm、闸门按最新观测的 4cm 判, AI 依 8.7 算出的 arm low(夹心
  // 落 9cm, 差 0.3cm)被按 4cm 拒掉, 回告还叫它"往前走 5cm"(走完差得更多), 连续 4 轮全被拒、任务
  // 锁死 —— 这种闸门比没有闸门更糟: 连"试一次、夹空了看得见"的机会都被剥夺了。
  // 融合值抗单次噪声, 但会继承历史观测里按无里程计位姿推算的漂移, "准不准"由抬臂复看兜底。
  float nx = g_mem[best].gx, ny = g_mem[best].gy;
  float dx = nx - s_car_x, dy = ny - s_car_y;
  if (lat_cm) *lat_cm = ch * dx + sh * dy;
  if (fwd_cm) *fwd_cm = -sh * dx + ch * dy;
  if (age_rounds) *age_rounds = (int)g_mem[best].stale;
  // 自那次观测以来车走了多远/转了多少(调用方拿它跟容差比, 判这条坐标还算不算数)
  if (moved_cm) *moved_cm = s_odo_cm - g_mem[best].odo_cm;
  if (turned_deg) *turned_deg = s_odo_deg - g_mem[best].odo_deg;
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