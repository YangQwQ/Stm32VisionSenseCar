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
#define AI_MEM_MAX 8
// 记忆喂回时效: 未观测轮数 ≤前者仍给精确坐标(供快速找回); 之间只提示"最近未见"不给坐标,
// 避免 AI 长期拿过时坐标去猜; 超过后者彻底不再喂。辨识: 活跃目标一轮一刷 stale≈1。
#define AI_MEM_FEED_STALE 10
// 给精确坐标的时效: 直接取夹取闸门那个时效(ai_mem.h 的 AI_GRASP_STALE), 于是 AI 看到记忆行给了
// 厘米坐标就一定能夹、看到"已N轮未见"就一定会被拒, 提示词/记忆行/闸门三者一一对应。
#define AI_MEM_COORD_STALE AI_GRASP_STALE
static struct {
  char name[16];
  float gx, gy;        // 全局坐标 cm(由观测**直接采纳**合成; 每次观测即覆盖, 不作离群拒收)
  uint32_t t_ms;       // 最近观测时刻
  int16_t stale;       // 0=新鲜; 每轮未观测 +1; >20 时不再喂回
  bool valid;
  int32_t obs_rev;                   // 最近一次观测时的车姿态版本号(见 s_pose_rev)
  float odo_cm, odo_deg;             // 最近一次观测时的路程/转角累积值(见 s_odo_*): 两个差值就是
                                     // "这条坐标从看到到现在被推算推了多远", 闸门据此判它还算不算数
  uint8_t just_reset;                // 曾"判历史失效⇒整条重置"来的一次性标志(已不再触发, 恒 0; 为兼容
                                     // mem_feed/mem_untrusted 旧读取保留字段, 但永不置位)
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

// （观测融合已改为无条件采纳, 不再取中位数, median_n 已移除。）

// 把一次"车头系坐标"观测写入物体记忆表: 入表时立即用**观测时刻**的车位姿转成全局坐标,
// 每次观测**无条件采纳**(不作离群拒收/锚定/中位数) —— 用户决定: 旧估计一旦错了, 拒收只会把
// 错坐标冻住、新观测永远纠正不回。
static void mem_store(const char* name, float wx, float wy) {
  int slot = -1, oldest = 0;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (g_mem[i].valid && g_mem[i].name[0] && name_same_obj(g_mem[i].name, name)) { slot = i; break; }
    if (!g_mem[i].valid) { slot = i; break; }
    if (g_mem[i].t_ms < g_mem[oldest].t_ms) oldest = i;
  }
  if (slot < 0) slot = oldest;                        // 满: 覆盖最旧
  // 这个槽是不是"按名字命中"的: 不是(空槽/最旧槽复用)就意味着里面的内容属于**别的物体** ——
  // 必须清干净再当新条目用, 避免新物体带着前一个物体的旧状态(标记位)入账。
  bool named = g_mem[slot].valid && g_mem[slot].name[0] && name_same_obj(g_mem[slot].name, name);
  if (!named) { g_mem[slot].just_reset = 0; }
  strncpy(g_mem[slot].name, name, 15); g_mem[slot].name[15] = 0;
  // 车头系 (wx右+, wy前+) → 全局: heading 逆时针正(左转+), 前=(-sin,cos)、右=(cos,sin)
  float h = s_car_heading * AI_PI / 180.0f, ch = cosf(h), sh = sinf(h);
  float gx0 = s_car_x + wx * ch - wy * sh;
  float gy0 = s_car_y + wx * sh + wy * ch;
  // ---------------- 无条件采纳 ----------------
  // 每次观测**直接覆盖**融合值, 不做离群拒收、不做"新名字锚定"、不取中位数。原因(用户拍板):
  //   · 旧坐标一旦错了, 离群门会把"每次都差好远"的新观测一遍遍拒掉, 把错估计冻住 —— 而画面(单应)
  //     才相对最可信, 应该让新观测把它纠正回来, 而不是让旧估计永远压着新观测。
  //   · 锚定"新名字贴已有物体"同样会按旧错坐标把新观测拉偏, 一并移除。
  // 代价: 单次离群实测(模型一次报错 px/py)会立刻写进记忆 —— 但下一轮画面观测又会把它纠正, 不锁死。
  g_mem[slot].gx = gx0;
  g_mem[slot].gy = gy0;
  g_mem[slot].t_ms = millis();
  g_mem[slot].stale = 0;
  g_mem[slot].valid = true;
  g_mem[slot].obs_rev = s_pose_rev;
  g_mem[slot].odo_cm = s_odo_cm;                     // 记下"看到它时车走了多远", 供闸门算推算误差
  g_mem[slot].odo_deg = s_odo_deg;
  g_mem[slot].just_reset = 0;
  ai::logf("[ai] 观测 %s 车头系(%.0f,%.0f) → 全局(%.0f,%.0f)", name, wx, wy,
           g_mem[slot].gx, g_mem[slot].gy);
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
  // 384 而不是 256: 最长的那条(丢失分流里"已被两指挡住"档)整句实测已到 ~280B(三位数厘米值 +
  // 满长物体名), 顶到 256 的截断线就会把句尾"该怎么做"吃掉 —— 那正是本块开头记的那条教训。
  // 栈上多这 128B 无所谓: 调用方是 ai_worker, 栈 16384。
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
        // 近场(前距 ≤AI_NEAR_FWD_CM, 与 approach 停距同值): 这一带的 cm 是单应高报解算值、近场每轮 ±5cm 抖,
        // 拿它判"进没进两指"就是拿噪声开车 —— 近场**只给远近档位、不给坐标**, 对准按画面两根手指判断。
        // 措辞用提示词同款的 [接近]: wy≤AI_ARM_UNDER_CM 且明显不在两指间是 [过近], 其上到近场线是 [接近]。
        const char* nstate = (wy <= AI_ARM_UNDER_CM) ? "[过近]" : "[接近]";
        ok = add("; %s 已%s", g_mem[i].name, nstate);
      } else {
        // 最近观测过且不在近场: 给精确坐标(供快速找回当前方位)
        ok = add("; %s(%s%.1fcm,%s%.1fcm)",
                 g_mem[i].name, wx >= 0 ? "右" : "左", fabsf(wx),
                 wy >= 0 ? "前" : "后", fabsf(wy));
        // 附上该全局坐标按**当前**车位姿反投影的屏幕位置, 供 AI 去画面那个位置核对目标是否还在。
        // 用实时反解而非存观测时的 px/py: 车一 move/spin, 目标在画面里的位置就变了, 存旧的反而误导;
        // 反投影随车姿每轮更新, 始终指示"此刻该往画面哪看"。world_to_screen 越出标定区返回 false 不显示。
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
      // 这几轮没再看到它, 但车几乎没动: 那条坐标仍是"看到的位置"(与闸门同一判据, 见 ai_mem.h)。
      // 要说清两件事, 少一件都会被 AI 走偏:
      //  (1) **不等于对准** —— 这句以前写的是"照它夹", 实测 AI 拿它当合爪依据(前场坐标有 1~2cm
      //      级系统偏差, 实测差 1.3cm 时方块还在两指指尖前方两三厘米), 连夹两次全空。
      //  (2) **丢失的尝试次序** —— 标准流程: 先动臂查遮挡(降臂/抬臂), 还看不见才小步后退找。这里曾经
      //      只写"别后退找": 实测 AI 会为找回画面一路后退、退出去目标更远, 来回摆(整轮 90s 没走到过
      //      arm low) —— 但把后退**整个划掉**又违背标准(后退是第三步), 且低姿下被两指挡住本来就查不出
      //      来。故改成有序三步并保留那句告诫: 先动臂, 后小步退, 且别"一丢就退"。
      //  (3) **但"先动臂查遮挡"不能无条件套** —— 实测 2026-09-22 20:33:44~20:35:18: AI 反复
      //      low→看不见→fold→看见→low, 90 秒一次夹取都没试。查画面留档(frame 0019 等)后确认
      //      **不是模型漏检**: 低姿下小车前挪两三厘米后, 方块真的被爪/臂挡出画面(同一位置
      //      前一帧还清清楚楚)。机制是结构性的: 可见≈前13~14cm, 被挡≈前10~11cm, 而夹取要前8cm
      //      ⇒ **最后那段永远看不见**。此时"再抬臂"等于退回起点, 下一轮必然重演, 死循环。
      //      所以按离爪口的远近分三种说法(与闸门同一条线, 值都在 ai_mem.h):
      //        · 前距 ≤ AI_ARM_UNDER_EXIT_CM: 已在臂下/车头前 —— 用户的纠正1, 抬臂 + 后退重来;
      //        · 那条线 ~ AI_NEAR_FWD_CM: 刚贴近就被挡 ⇒ 这是**到了**, 不是丢了:
      //          别抬臂, 小步顶进送进两指之间再夹;
      //        · > AI_NEAR_FWD_CM: 真远场丢失 ⇒ 原三步。
      // ⚠️ 但**臂下那一档必须再按横向分一次**, 否则它会专门在最该扣扳机的那一刻喊"抬臂后退":
      //    低姿爪口前 8cm 处的报值正是 10 上下(单应高报, AI_NEAR_SCALE 折算回 8) —— 与出门线
      //    重合。实测 2026-09-22 22:29~22:33 整轮: 闸门自己打出「前10=折算8 横向差0.7 前后差
      //    0.3cm」(几何完全就位、合爪会被放行), 而同一轮记忆行喊的是"已在臂下 ⇒ fold 抬臂再
      //    后退再降臂" ⇒ AI 只 low→fold→low 打转, 240 秒里 arm grasp **一次都没发**。横向又是
      //    爪子唯一补不了的一轴(也只有它决定夹不夹得住), 故拿它分档: 横向在容差内(与闸门同一条
      //    硬拦线同源, 值在 ai_mem.h) ⇒ 这就是夹取位, 直接夹; 偏出去才走用户的纠正1。
      // ⚠️ 第一档取的是**出门线**(10)而不是进门线(7): 闸门那边的迟滞只要开着, ufwd 就一定 ≤ 出门线,
      //    所以"程序判在臂下"与"记忆行判在臂下"永远同时成立。若这里用 7, 就会在 7~10cm 那段出现
      //    闸门说"抬臂+后退"、记忆行说"别抬臂、顶进"的**相反处方** —— AI 能看见的只有这些字, 它
      //    只会来回蹭(这正是这次要修的病, 不能换个地方再犯一遍)。
      // 所以: 坐标只用来判"方位没变", 对准一律看画面。措辞要短: 这条最长, 而 mem_s 预算还得分给别人。
      // 这几轮没再看到它、但车几乎没动: 那条坐标仍是"看到的位置"。近场**不报坐标、不给长处方**:
      // 坐标是单应高报解算值, 长处方(抬臂/后退/分横向档)之前实测正是"该扣扳机时反向拆台"的源头。
      // 这里只给一个对齐提示词远近档位的简短状态 + 一句"看不见先动臂查遮挡"的通用指引, 让 AI 自己看画面。
      {
        const char* st;
        if (wy <= AI_ARM_UNDER_CM) st = "已[过近]";
        else if (wy <= AI_NEAR_FWD_CM) st = "已[接近]";
        else st = "[较远]";
        ok = add("; %s %s(%d轮前看到, 车没动; 看不见先动臂查遮挡, 还看不见才小步后退, 别一丢就退)",
                 g_mem[i].name, st, (int)g_mem[i].stale);
      }
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
  buf[n] = '\0';
  ai::logf("[ai] 记忆 %s", buf);   // 喂回内容同步到 ai_log, 便于观察 AI 看到的物体位置理解
}

// 记忆里有没有可用的物体。判据与 mem_feed 的过滤条件**同源**(valid + stale 未过喂回线):
// 两处若不同步, 程序会在记忆行明列着目标的那一轮说"你还没锁定任何目标"。
bool mem_have_any() {
  for (int i = 0; i < AI_MEM_MAX; i++)
    if (g_mem[i].valid && g_mem[i].stale <= AI_MEM_FEED_STALE) return true;
  return false;
}

// 是否存在"曾被正确观测、现已不可见"的物体: stale 已超新鲜线(因而夹取依据也过期、闸门会拒), 但记录
// 还在(valid)。stale=1..新鲜线内的是近期正看着的对象, 不算"丢失"; stale 超喂回线(>AI_MEM_FEED_STALE)
// 的早已连喂都不喂, 那属于 mem_have_any 的"空"。中间这档(新鲜线<stale≤喂回线)正是"还记得它、但当前
// 这一两轮没在画面里看到"—— AI 若同时上一轮回收了机械臂/后退, 目标很可能被挡在镜头外, 见 ai_client。
bool mem_have_lost() {
  for (int i = 0; i < AI_MEM_MAX; i++)
    if (g_mem[i].valid && g_mem[i].stale > AI_GRASP_STALE && g_mem[i].stale <= AI_MEM_FEED_STALE)
      return true;
  return false;
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

// 这条坐标现在还站得住吗(见 ai_mem.h)。**挑条目的口径必须与 mem_find 逐字同源**(同一批 slot、同一条
// 过期线、不点名时同样挑离车最近的那条)—— 否则会出现"mem_find 说导航目标是它、mem_untrusted 却查的
// 是另一条"的两边打架, 那种不一致正是本仓库反复踩过的同源不变量。
bool mem_untrusted(const char* name) {
  int best = -1; float bd = 0.0f;
  for (int i = 0; i < AI_MEM_MAX; i++) {
    if (!g_mem[i].valid || g_mem[i].stale > AI_MEM_FEED_STALE) continue;
    if (name && name[0]) {
      if (name_same_obj(g_mem[i].name, name)) return g_mem[i].just_reset != 0;
      continue;
    }
    float dx = g_mem[i].gx - s_car_x, dy = g_mem[i].gy - s_car_y;
    float d = dx * dx + dy * dy;
    if (best < 0 || d < bd) { bd = d; best = i; }
  }
  return best >= 0 && g_mem[best].just_reset != 0;
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