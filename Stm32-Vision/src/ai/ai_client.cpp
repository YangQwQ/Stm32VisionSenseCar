#include "src/ai/ai_client.h"
#include "src/ai/ai_prompt.h"   // PsaBuf + build_body(请求体组装)
#include "src/ai/ai_mem.h"     // 空间记忆/车姿态: mem_reset/mem_feed/mem_find/...
#include "src/ai/ai_http.h"    // TLS 发送层: http_post/http_last_status/http_stop
#include "src/ai/ai_dump.h"    // 抓帧留档(调试旁路, 只在 /log ai on 时留存本帧画面)
#include "src/ai/magnify.h"    // 放大镜: 把目标附近裁出来放大重编码(AI 的"凑近看")
#include "src/net/config.h"
#include "src/cam/camera.h"
#include "src/exec/direct_exec.h"
#include "src/net/wifi_net.h"
#include "src/ai/ground_proj.h"
#include "src/core/board_log.h"
#include "src/core/heap_watch.h"   // 内部堆水位哨兵(诊断 WiFi 收发哑掉的第一现场)
#include "Calibration.h"   // ARM_LOW_X_CM(低姿落点): 闸门要拿它跟目标前距比, 见"夹取闸门"段

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>   // 统一处理 TLS/content-length/chunked, 替代手写 http_exchange
#include <esp_timer.h>
#include <stdlib.h>  // malloc/free/strtol
#include <math.h>    // fabsf
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <ArduinoJson.h>
#include <esp_heap_caps.h>  // MALLOC_CAP_SPIRAM
#include <mbedtls/platform.h>  // mbedtls_platform_set_calloc_free(TLS 内存搬到 PSRAM)
#include <string.h>         // memset/strncpy(空间记忆表)
#include <stdarg.h>         // vsnprintf(ai::logf)

// 决策频率 / 步数上限
#define AI_INTERVAL_MS 1500
#define AI_MAX_STEPS_PER_GOAL 120
#define AI_EDITED_IMG_MAX (128 * 1024)
#define AI_EDITED_IMG_TTL_MS 60000
#define AI_WAIT_FB_MIN_MS 10000   // wait 反馈节流: 同一动作少于此间隔只回一条
#define AI_MOVE_CAP_MS 2000       // AI 持续 move 单次行驶时限(无里程计兜底, 防决策间隔内盲走撞墙)
#define AI_MAX_NET_FAIL 4         // 连续"无有效输出"轮数上限: 超过即中止任务并回报(防云端持续无响应时无限空转)
#define AI_HIST_N 20               // 历史环条数(AI 决策 + 插话共用, 满员淘汰最旧)
// AI 未给 distance_cm 的 move 一律按此段长执行(而不是放开成"持续"盲走)。理由: 持续 move 只能靠
// AI_MOVE_CAP_MS 兜底, 2s×最多约 25cm/s ≈ 25cm 且**位移未知**(car_update_pose 不累积无定距的移动
// → 记忆里车位置从此失同步, 之后喂回的物体坐标全部偏移)。补成有限段长后, 走了多远始终已知:
// 既防"冲过头", 也让记忆坐标持续可用。取值贴 AI_APPROACH_STOP_CM: 从典型停靠距离再走一步正好落进
// 贴地夹取带(x≈9cm)。不要大于它 —— 靠近到 10cm 后再来一步 12cm 会直接冲过目标, 把方块顶进车体
// 下方死角(既够不着又看不见, 只能后退重来)。
#define AI_MOVE_DEFAULT_CM 8
// AI 未给 angle_deg 的 spin 一律按此角度执行(而不是放开成"持续旋转")。同样的道理: 持续旋转期间
// car_update_pose 不累积车向(angle 未知), 于是车头朝向估计停滞, 而所有喂回的物体坐标都是用这个
// 朝向从全局系换算到车头系的 —— 朝向一错, 喂回的左右/前后全错(这是"左右说反"的另一条来路)。
// 补成有限角度后每步朝向已知; 且定角旋转有 mvfy 视觉到位补偿, 实际转角比"盲转"更准。
#define AI_SPIN_DEFAULT_DEG 30
#define AI_MOVE_MAX_CM 40         // AI 单步 move 距离硬上限(硬钳制, 覆盖提示词的"≤30cm"建议)
// 近场步幅钳制(只作用于**前进**): 依据里的目标已在前方 AI_NEAR_FWD_CM 以内时, 单次前进步长压到
// AI_NEAR_STEP_CM。病根(实测): 方块进入两指之间只差几厘米, 而 AI 会按"画面中线附近=还远"一路 4~8cm
// 地顶 —— 两三轮就把方块压到车底(那里既看不见也够不着), 之后整轮都在盲区里找。它自己的判据并不错
// (方块确实还在两指前方), 错的是步长: 近场每 1cm 都在改变"夹得住/夹不到"(方块才 2~3cm 宽)。
// 判据只认"最近一次亲眼看到的前距"(mem_grasp_evidence, 已换算到当前车头系), 不依赖定标精度。
// 阈值取 14 而不是 12: 前场坐标实测高报约两三成, 报 14cm 时真距可能只剩 10cm —— 那已经是一步就能
// 顶进盲区的距离(实测: 报 13.7cm 时 AI 顶了 3cm, 方块当场从画面里消失、再没找回来)。
#define AI_NEAR_FWD_CM 14
#define AI_NEAR_STEP_CM 2
// 近场"够不够得着"的裁决线: 目标前距缩到这个值时, 它要么已经在两指之间, 要么已被顶进车头下沿的盲区
// —— 这两者在画面里长得完全不同, 而 AI 实测在"看不见它"时会无限 zoom 空转(它手里没有能选下一步的
// 判据)。所以这一档不是钳制, 是**每轮都推**的裁决提示(见 hint_cb 处)。
// 取 12 = 前场坐标报 12cm 时真距大约 9cm, 正好是低姿夹爪口的距离: 越过这条线, "看不见"就不再是
// "还差一点"而是"已经过了/钻到车头下面了"。定得太小会漏掉实测那种情形(报 10.7cm、实则已进盲区)。
#define AI_NEAR_BLIND_CM 12
// 近场(≤AI_NEAR_BLIND_CM)目标坐标的折算系数: 前场坐标由单应解算, 在近处系统性**高报**(实测两三成,
// 地砖校验指向约 30%), 而夹心/爪口位置是正运动学(FK)算出的**真实几何** —— 拿高报值直接减真值,
// 差出来的厘米数没有物理意义(闸门据此报"差1.6cm"、还按 gfwd 与 x_tgt 的大小给出"前进/后退",
// 方向会整个反过来: 坐标报 11cm 而真距约 8cm 时, 目标其实比爪口(9cm)更近, 该后退却报前进)。
// 用途只有一个: 把坐标换成"真距的量级估计"给 AI 与闸门**同一个口径**, 任何一档都仍以画面为准
// (那一档给的动作是"先 zoom 确认")。⚠️ 它是估计值不是精测值, 只做量级取舍, 永不作单独的一票。
#define AI_NEAR_SCALE 0.75f
// "低姿"的高度线(见 spin 处的"低姿转身"告警): 低于它就是两指已经贴地前伸的状态。
// 贴 ARM_LOW_H_CM(=1.0，见 Calibration.h)取 2.5：留出 FK 回读与单应噪声的余量，又明显低于
// 收臂折叠位(fold 的 h≈7)—— 判"爪还没降下来"时不会把低姿误判成高位。
#define AI_ARM_LOW_H_CM 2.5f
#define AI_SPIN_MAX_DEG 180       // AI 单步 spin 角度硬上限: 大幅盲转会把画面参照全丢, 且无里程计下角度误差随幅度放大
// "对准"档的单步上限(度): 记忆里有**新鲜观测**(目标刚被画面看到)时, 这次旋转是"对准"而不是"搜索",
// 而实测一次 47° 的对准转就把方块甩出了视野(2026-09-22 10:18 那次之后连 7 轮没再看见它, 再没找回,
// 任务被超时收掉)。提示词规则5 本就写"对准 spin ≤15°", 但模型要在"搜索≤60"(规则3)与"对准≤15"之间
// 自己选档, 实测选错 —— 改由程序按"有没有新鲜观测"定档, 比让模型猜可靠。真在搜索(无新鲜观测)时不钳,
// 仍受 AI_SPIN_MAX_DEG 与累计预算约束。
#define AI_SPIN_TRACK_MAX_DEG 15
// 近场档(度): 目标已在前方 AI_NEAR_FWD_CM 以内时, 对准单步再收紧 —— 例: 前距 10cm、转 10° 横向
// 移动 ≈1.7cm, 已接近方块宽度(2cm), 一步就可能跳过目标(实测: 目标已到(0.1,10.8)完美位置, AI 连转
// 10°×2 把它甩出视野)。近场每 1° 都是夹得住/夹不到的差别, 宁小勿大。
#define AI_SPIN_NEAR_MAX_DEG 6
// 单个任务累计旋转预算(度, 绝对值累加): 单步上限管不住"连发多步"(180°×N 就是好几圈), 而车尾还接着
// 充电线(缠住就走不动), 且每转一次朝向估计都要跟着累积误差。一圈半够从任何起始朝向找到目标;
// 用满后不再放行旋转, 并把话说明白(见 spin_warn) —— 这时正确的动作是收臂回画面重新定位。
#define AI_SPIN_TASK_BUDGET_DEG 540
// "对准转向"的独立额度(度): 搜索用的旋转预算与"有依据的横向对准"必须**分账**。实测: AI 为找目标一路
// 搜索把 540° 转光, 之后它自己按画面算出"朝右 spin 9° 把目标挪进爪口"照做, 同一份预算却把这 9° 降级
// 成停(日志: "旋转预算用满 → 本次 spin 降级为停" 紧跟 "执行 spin 0"), 车纹丝不动 ⇒ 横向误差永远修不
// 掉, 每一轮都白费 —— "程序给出自己满足不了的指令"的同源违规。
// 分账判据是**手上有没有新鲜观测**(目标刚被画面看见): 有 ⇒ 这次转是在对准, 它看得见结果、转错了自己
// 能修, 不占搜索预算; 没有 ⇒ 那是盲搜, 吃 540° 的搜索预算。额度用尽说明坐标精度已到极限, 那时该由
// 画面说了算, 而不是继续拿一个程序自己都补不了的判据把 AI 卡死。
#define AI_SPIN_ALIGN_BUDGET_DEG 180
// AI 照提示的度数转向时常自己再加几度微调, 容这点余量仍算"对准转向"
#define AI_SPIN_ALIGN_SLACK_DEG 8
#define AI_SETTLE_MAX_MS 700      // 出帧前等轮子/补转停稳的上限(定距到点自停后仍有余速滑行与 mvfy 补转)
#define AI_FRAME_RETRY 4          // 单轮抓帧重试次数(推流并发占缓冲时会偶发取不到)

// ---------------- 放大镜(命令 zoom, 见 worker 里"放大镜"段) ----------------
// 病根: AI 在全幅(VGA)里估绝对像素不可靠 —— 方块只占 ~35px 宽(画面 640), 而它自报的 px 中位偏
// 37px、22% 的轮次超 100px; 单应标定本身又带厘米级残差(地砖校验还指向它高报约 30%), 两者叠加后
// "它以为对准了"和"真对准了"分不开, 而夹取要的正是 1.5cm 级的判断。
// 放大镜不引入任何颜色/形状/目标的假设(那类硬编码换个光照/换个目标就废): 它只是把 AI 的眼睛凑近,
// 让它在放大图里跟**看得见的夹爪**比相对位置(它的强项), 而裁框是程序记的账 ⇒ 它报的像素换回全幅
// 是精确算术, 不是估计。顺带给了"夹住没有"的判据: 合爪后看目标有没有跟着爪起来。
#define AI_ZOOM_OUT_W 320           // 放大图输出宽(与全幅同量级: 放大靠"取景框小", 不靠输出像素多)
#define AI_ZOOM_OUT_H 240
#define AI_ZOOM_QUALITY 80          // 比图传略高(这是用来看细节的), 一帧几十 KB
#define AI_ZOOM_JPG_MAX (48 * 1024) // 单帧上限: 超了说明裁框异常, 弃用回全幅
#define AI_ZOOM_MIN 1.5f            // 倍数下限: 再小与全幅几乎一样, 白搭一次解码+编码
#define AI_ZOOM_MAX 8.0f            // 倍数上限: 再放也只是像素块, 而视野小到看不见夹爪就失去参照
#define AI_ZOOM_DEF 2.0f            // AI 没给 scale 时的默认倍数(取小些: 放大后要同时看得见目标和夹爪, 开太大爪会被裁出画面就失去参照)
// 同一块画面**在车与臂都没动过**的前提下连要几次后, 改回全幅并讲死(见"放大镜"段)。
// ⚠️ 判"重复"必须带上"这期间有没有动作": 裁框是全幅归一化坐标, 挪车之后同一个框里已是另一片
// 世界 —— 那时 AI 要的是"再看一眼刚动过的地方", 是正当的重看, 不是复读。曾按"框一样就是重复"
// 判, 于是把正当重看也一并算账(还扣了进展), 而真复读只靠提示语又拦不住(实测同一条连发 31 次)。
#define AI_ZOOM_NOOP_MAX 2
// 连续多少轮没有任何**实际动作**(只有 zoom/wait 这类本地命令)就认定它空转了, 强制把放大镜退回
// 全幅并点名。弱模型"看不见目标"时的第一反应是反复放大, 而放大的局部画面里永远找不到画面外的
// 目标 —— 实测就这样锁死在 8×(视场只剩 12%)空转到超步数。搜索只能在全幅做。
#define AI_IDLE_ROUNDS 5

// ---------------- 夹取闸门(见 worker 里"夹取闸门"段) ----------------
// h ≤ 此值的 arm_pose 视为"把夹爪下探到方块高度去夹", 必须持有近期画面依据才放行(且夹爪是张开的
// —— 合着爪下探是去放东西, 不拦, 否则搬运类任务永远放不下)。
// 抬高/悬停(h 更大)、arm fold/release/lift_* 一律无条件放行: 回撤与让出视野的动作永远不能拦,
// 否则 AI 会被关在一个它进不去也退不出的位姿上。
// 同 AI_ARM_LOW_H_CM 取 2.5：夹取前爪必须先 arm low 降到贴地位，半空中合爪必然空夹。
#define AI_GRASP_H_CM 2.5f
// 合爪时"夹心 vs 目标前距"的最大容许偏差 cm(约一个方块的半宽)。超了就是两指之间没东西:
// 目标 2~3cm 宽, 夹心偏出这么多物理上不可能夹住, 所以据此拒绝绝不会误杀真能夹住的动作。
// 拒绝的理由要说清往哪挪车 —— 实测的失败正是"目标在前 12cm、爪停在 10cm 就直接合爪",
// 而现有闸门只看"目标在车前 0~22cm 内", 差 2cm 照样放行, 于是空夹且没有任何日志。
#define AI_GRASP_X_TOL_CM 1.5f
// 近场"往前/往后"方向建议的**噪声死区** cm: 差得没超过它就别点名方向。
// 病根(2026-09-22 实测 grasp-newparadigm-1): 同一个方块、车几乎没动, 程序解算的"前距"却在
// 3.7~27.9cm 之间跳(样本 5.0 8.0 14.7 5.2 9.4 10.7 7.4 10.8 12.2 15.2 3.7 27.9), 而爪口固定
// 在车前 8cm。噪声**远大于**方块半宽(AI_GRASP_X_TOL_CM) ⇒ 拿它点名"前进/后退"就是用噪声开车:
// 顶一下 → 坐标报远 → 再顶 → 再报远, 实测就是这个前后循环(操作者复述"对准到完全能夹住时又自己
// 后退, 然后一直循环前后移动")。故只在远超噪声时才给方向; 带内改为"别再挪车, 看画面确认后夹"。
#define AI_NEAR_DEADBAND_CM 5.0f
// 低姿前顶之后的轮数窗口(见 worker 里的记账点): 期间闸门给"退1cm再夹"这条配方, 而不是按坐标点名方向。
// 操作者实测的机械经验: 爪子顶得动方块之后要**稍微退后约 1cm** 才夹得住 —— 顶着的时候两指被方块撑住
// 使不上劲, 退开一点才有闭合行程去咬住它。这条只有操作者能提供, 程序与模型都推不出来。
#define AI_LOW_PUSH_TTL 3
// 刚把爪降到位后的那一轮, 提示 AI 先据"落位后第一帧"确认方块是否已在两指之间再合爪。
// 只提示不拦截: 夹爪伸出去后目标可能确实完全被自身挡住(此时无画面依据可用), 硬拦会把夹取卡死。
#define AI_GRASP_CHECK_HINT \
  "上一步刚把爪降到贴地低姿: 本帧是落位后的第一帧, 先据它确认方块是否已在两指之间" \
  "(哪怕只露出一部分), 用 zoom 放大看最清楚。看不到或明显偏在一边 → 用小步 move/spin 微调" \
  "(程序会提示方向), 不要就此合爪; 确实被自身夹爪完全挡住时, 才用落位前那一帧看到的画面判断," \
  "不要再拖轮次。"

// 本轮发的是放大图时的提示。必须说, 否则 AI 会拿放大图当整幅去估距离/方位 —— 那正是它本就不擅长的
// 绝对估计, 而且这次连参照系都错了。怎么退出(zoom reset)在系统提示词里, 这里省字节。
#define AI_ZOOM_HINT \
  "本帧是放大图(只有目标附近一小块), 看不到全幅: 别据此估距离/方位, 只用来确认目标与夹爪的相对位置" \
  "(尤其合爪前看目标是否在两指之间、抬臂后看它有没有跟着起来); 要回全幅或要挪车, 先输出 zoom reset:true"

// 操作者插话(ai_chat)已发给模型但那一轮没拿到有效响应(超时/空 content)时的补投提示。
// 插话在组包时就被一次性消费, 那轮的响应若报废, 建议等于没被执行——只在历史环里留着, 模型未必
// 回头认领。故在它被真正落实前, 每轮额外点一次名(正文在历史环里, 这里只提醒去认领, 不重复正文)。
#define AI_CHAT_UNACK_HINT \
  "操作者有一条插话建议你还没落实(那一轮的响应失败了): 见对话历史里最后那条 user 插话, " \
  "本轮请优先按它的思路做, 或明确说明为什么不采纳。"

// 本地巡航(approach / /move to)参数: 定距段长、初对齐阈值、单次转向上限等。
// 到位距离 10cm: 别拿可达域的**外接矩形**(4~15cm)当中段用 —— 真实可达域是条下窄上宽的弯带,
// 贴地夹取(h≈1.5~2cm)时 x 只能伸到约 8~10.5cm。所以停在 12cm 会让贴地的方块永远差 1.5~2cm
// 够不着(实测就是这么空夹的: 方块在前 12cm, 爪停在 10cm 就直接合爪, 全程无任何报错)。
// 10cm 落在低处弯带中间, 滑行(COAST)后再近一点也仍在带内; 目标此时在画面下半部, 既看得见又
// 夹得到(具体落在哪个 v 由 kGroundCal 单应推出, 别在这儿再抄一份 —— 摄像头一动它就过期)。
// 与 Calibration.h 的 ARM_LOW_X_CM 是一对: 靠近停到这儿, 爪的固定准备位也在附近, 之后只需
// 厘米级微调。两者的复核时机见 Calibration.h ARM_LOW_* 处那条告警。
#define AI_APPROACH_STOP_CM 10   // AI 自动靠近的到位距离(之后交给 AI 微操)
#define NAV_STOP_CM_GOTO 6       // /move to 的到位容差 cm(无里程计开环, 留一点松量)
#define NAV_ALIGN_DEG 25         // 目标偏角超过此值先原地转向对齐, 否则直行推进
#define NAV_MAX_SPIN_DEG 60      // 单次原地转向的角度上限(分步收敛)
#define NAV_MAX_SEG_CM 25        // 单次定距推进上限 cm(分步收敛、防冲)
#define NAV_MAX_ITERS 200        // 巡航最大迭代步数(防死循环)
#define NAV_WAIT_LOOPS 120       // 每段等待轮子停下的检测循环数(约 50ms 一拍)

// ArduinoJson 内存池改用 PSRAM, 避免其小分配每轮在内部堆上反复申请/释放, 
// 与 TLS 缓冲交错把内部堆切成碎块(导致握手 -17040/-32512 失败)。
struct PsramAllocator : public ArduinoJson::Allocator {
  void* allocate(size_t n) override { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM); }
  void deallocate(void* p) override { if (p) heap_caps_free(p); }
  void* reallocate(void* p, size_t n) override { return heap_caps_realloc(p, n, MALLOC_CAP_SPIRAM); }
};
static PsramAllocator g_js_alloc;

// ---------------- worker / 槽 / 队列 ----------------

struct TaskLocal {
  char* text = nullptr;   // 目标文本(堆)
  char* ann = nullptr;    // 标注 JSON 或 {x,y,w,h,label}(堆)
  bool use_image = false;
  bool one_shot = false;  // 单轮模式: 只执行一轮决策即收尾(/ai oneshot)
  long id = 0;            // 对应 ai_goal 的词表 id(回填 ai_result)
  unsigned long generation = 0;
  cmd::ReplyFn fn = nullptr;
  void* ctx = nullptr;    // 任务期 sink ctx(WS=堆 int*, BLE=nullptr)
  bool active = false;    // 槽是否已被 set_goal 激活
  float nav_x = 0, nav_y = 0;  // goto_target 目标坐标 / frame
  bool nav = false;             // 纯导航任务(/move to), 不走 AI 闭环
  bool nav_global = false;      // true=沿用全局系; false=以当前车位姿为新原点
};

// 结果队列项: 每项独立持有消息文本与恢复用的(fn, 本次堆拷贝 ctx)。
struct ResultItem {
  char* text = nullptr;
  cmd::ReplyFn fn = nullptr;
  void* ctx = nullptr;    // 每次入队新建的 int*(WS); BLE nullptr
};

static TaskLocal g_slot;
static SemaphoreHandle_t g_mtx = nullptr;   // 保护 g_slot / m_generation
static SemaphoreHandle_t g_notify = nullptr; // 唤醒 worker 的二进制信号量
static QueueHandle_t g_result_q = nullptr;
static volatile unsigned long m_generation = 0;  // 代际号: 每次 set_goal/cancel 自增
static volatile bool m_busy = false;
static TaskHandle_t g_worker = nullptr;

// 编辑图暂存(PSRAM)+ 时间戳; worker 内只读快照由 set_edited_image/取图互斥。
static uint8_t* g_edited = nullptr;
static size_t g_edited_len = 0;
static uint64_t g_edited_ts = 0;
static SemaphoreHandle_t g_img_mtx = nullptr;

// AI 任务进行中"插话"缓冲(ai_chat 写入、worker 每轮消费一次, 受 g_mtx 保护)。
// 一次性消费: 喂进下一轮 prompt 后清空, 避免重复塞给 AI。
static char g_chat[256] = {0};
static bool g_chat_has = false;

// 角度归一化到 (-180,180](方向计算统一口径)。
static float wrap180f(float a) {
  a = fmodf(a, 360.0f);
  if (a > 180.0f) a -= 360.0f;
  else if (a < -180.0f) a += 360.0f;
  return a;
}

// 最近一条下发执行板的是否持续型(sink: stop 兜底判定)。任务起点复位。
static volatile bool g_last_continuous = false;

// 最近一条持续指令的类型(0=无/1=move/2=arm)。兜底 stop 时 Wheels 模式只适用于轮子残留。
static volatile int g_last_cont_type = 0;

// 本轮任务终结时的兜底 stop 模式, 由打断方写入; worker 出口统一解析一次。
static volatile int g_stop_mode = (int)ai::StopMode::All;
static volatile uint64_t g_last_wait_fb_ms = 0;  // wait 反馈节流时间戳

static void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx);

// ---------------- 结果队列(worker → loop) ----------------

// 就地把一条 WS 文本转为合法 UTF-8(RFC6455 文本帧必须为 UTF-8)。
// 云端 AI 响应/日志偶发混入残缺 UTF-8 序列或非法字节, 若原样塞进 WS TEXT 帧, 
// 手机 Godot 会以关闭码 1007(Invalid frame payload data)断链。此时整条链路
// 的中文保持不变, 仅把控制字符与非法/残缺序列替换为 '?'(1:1, 不扩容)。
static void sanitize_ws_utf8(char* s) {
  char* w = s;
  const unsigned char* p = (const unsigned char*)s;
  while (*p) {
    unsigned char c = *p;
    int need = 0;
    if (c < 0x20 || c == 0x7f) { *w++ = '?'; p++; continue; }   // 控制字符
    if (c < 0x80) { *w++ = (char)c; p++; continue; }             // 合法 ASCII
    if (c >= 0xC2 && c <= 0xDF) need = 1;
    else if (c >= 0xE0 && c <= 0xEF) need = 2;
    else if (c >= 0xF0 && c <= 0xF4) need = 3;                   // 其余字节非法
    bool ok = need > 0;
    for (int i = 1; ok && i <= need; i++) {
      unsigned char cc = p[i];
      if (!cc || !(cc >= 0x80 && cc <= 0xBF)) ok = false;        // continuation 缺失/越界(含被截断的串尾)
    }
    if (ok) { for (int i = 0; i <= need; i++) *w++ = (char)p[i]; p += need + 1; }
    else { *w++ = '?'; p += 1; }                                 // 非法首字节/残缺序列: 单字节替换
  }
  *w = 0;
}

void enqueue_result(const char* text, cmd::ReplyFn fn, void* ctx) {
  ResultItem* it = (ResultItem*)malloc(sizeof(ResultItem));
  if (!it) return;
  size_t n = strlen(text);
  it->text = (char*)malloc(n + 1);
  if (!it->text) { free(it); return; }
  memcpy(it->text, text, n + 1);
  sanitize_ws_utf8(it->text);   // 统一 WS 文本消毒: 任何 enqueue 出口都走这里, 防 1007 断链
  // WS: 为本次结果单独堆拷贝 fd(loop 发送后释放); BLE ctx 已为 nullptr。
  it->fn = fn;
  it->ctx = ctx ? new int(*(int*)ctx) : nullptr;
  if (xQueueSend(g_result_q, &it, 0) != pdTRUE) { free(it->text); free(it->ctx); free(it); }
}

// AI 调试日志: 经 board_log(blog::AI)统一输出; /log ai(或 all)时转发手机。
void ai::logf(const char* fmt, ...) {
  char buf[256];   // 栈缓冲保持小，防小栈任务(WS/httpd/BLE)里大局部压栈溢出崩溃
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  buf[sizeof(buf) - 1] = 0;  // 截断防越界
  va_end(ap);
  // 统一走板端日志模块: 始终写串口(带 [ai] 前缀); /log ai on 时经日志队列转发手机。
  // 消息内统一以 "[ai] " 开头, 与 blog::logf 的类别前缀重合 → 剥掉一段防显示成 "[ai] [ai]"。
  const char* p = buf;
  if (strncmp(p, "[ai] ", 5) == 0) p += 5;
  blog::logf(blog::AI, "%s", p);
}

void ai::update() {
  ResultItem* it = nullptr;
  while (g_result_q && xQueueReceive(g_result_q, &it, 0) == pdTRUE) {
    if (it) {
      if (it->fn) it->fn(it->ctx, it->text);
      free(it->text);
      if (it->ctx) delete (int*)it->ctx;
      free(it);
    }
  }
}

// ---------------- 编辑图暂存 ----------------

void ai::set_edited_image(const uint8_t* data, size_t len) {
  if (!data || len == 0 || len > AI_EDITED_IMG_MAX) return;
  xSemaphoreTake(g_img_mtx, portMAX_DELAY);
  if (!g_edited) g_edited = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
  if (g_edited) {
    memcpy(g_edited, data, len);
    g_edited_len = len;
    g_edited_ts = esp_timer_get_time();
  }
  xSemaphoreGive(g_img_mtx);
  blog::logf(blog::AI, "收到编辑图 %u B", (unsigned)len);
}

static bool take_edited(uint8_t* buf, size_t cap, size_t* out_len) {
  bool ok = false;
  xSemaphoreTake(g_img_mtx, portMAX_DELAY);
  if (g_edited && g_edited_len > 0 &&
      (esp_timer_get_time() - g_edited_ts) < (uint64_t)AI_EDITED_IMG_TTL_MS * 1000 &&
      g_edited_len <= cap) {
    memcpy(buf, g_edited, g_edited_len);
    *out_len = g_edited_len;
    ok = true;
  }
  xSemaphoreGive(g_img_mtx);
  return ok;
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

// 剥掉状态行里跟在 "cm" 后的舵机 PWM(如 "前10cm(200)" → "前10cm")。这些 PWM 是给人工校准机械臂
// 坐标用的(见 exec::read_state 注释/exec_log), 对 AI 是纯噪声, 且紧挨着真实距离极易被误读成另一个
// 距离/高度值。只认 "cm(" 前缀, 因此不会碰到诊断里的坐标括号(如 "目标(12.0,3.0)不可达")。
// 原地压缩(只减不增), 调用方缓冲复用即可。
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

// 等待本轮定距/定角到段自停(loop 的 update_tick 会按时长停轮); 中断或超时即退出。
static void wait_wheels(unsigned long gen) {
  for (int i = 0; i < NAV_WAIT_LOOPS; i++) {
    if (gen != m_generation) return;
    if (!exec::wheels_moving()) return;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// 出帧前限时等停稳(最多 max_ms, 超时照常出帧, 绝不长时间阻塞决策)。必要性: 定距 move 到点自停后
// 仍有余速滑行(MV_COAST), 定角 spin 到点后还有 mvfy 视觉补转, 两者都让摄像头在**运动/带模糊**中
// 取帧 —— AI 据这种帧报出的 px/py 会把物体方位算偏(近距离尤其明显), 也是"近处左右说反、靠近后
// 反而找不到目标"的一大来源。停稳后再看画面, 单应换算才有意义。
static void settle_wheels(unsigned long gen, int max_ms) {
  for (int t = 0; t < max_ms; t += 20) {
    if (gen != m_generation || !exec::wheels_moving()) return;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// 本地巡航核心(纯本地, 不调云端): 把车从当前位姿导航到全局目标 (tx,ty), 距目标 ≤stop_cm 停。
// 无里程计: 按定距/定角时长近似 + 姿态累积做开环死航; 到位后剩余误差交给 AI 视觉微操兜底。
// 返回 Reached(到位)/ Interrupted(被新任务/手动打断)/ TimedOut(迭代超限收敛)。
enum class NavR : uint8_t { Reached, Interrupted, TimedOut };
static NavR navigate_to(unsigned long gen, float tx, float ty, float stop_cm) {
  const float throttle = 0.5f;   // 巡航推进油门(中速)
  for (int it = 0; it < NAV_MAX_ITERS; it++) {
    if (gen != m_generation) return NavR::Interrupted;
    float dx = tx - ai::s_car_x, dy = ty - ai::s_car_y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= stop_cm) { JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>()); return NavR::Reached; }
    float thg = atan2f(dx, dy) * 180.0f / ai::AI_PI;        // 目标全局方位(heading=0 时朝 Y+)
    float rel = wrap180f(ai::s_car_heading + thg);          // 目标相对车头偏角, 正=右
    if (fabsf(rel) > NAV_ALIGN_DEG) {                   // 偏太多先原地转向对齐
      int ang = (int)fminf(fabsf(rel), (float)NAV_MAX_SPIN_DEG);
      JsonDocument p(&g_js_alloc); p["dir"] = rel > 0 ? 1 : -1; p["speed"] = 850; p["angle_deg"] = ang;
      exec::act("spin", p.as<JsonObjectConst>()); ai::car_update_pose("spin", p.as<JsonObjectConst>());
      wait_wheels(gen);
    } else {                                            // 否则定距直行一段(分步收敛)
      float seg = fminf(dist - stop_cm, (float)NAV_MAX_SEG_CM);
      if (seg < 1.0f) seg = 1.0f;
      JsonDocument p(&g_js_alloc); p["throttle"] = throttle; p["steering"] = 0; p["distance_cm"] = (int)seg;
      exec::act("move", p.as<JsonObjectConst>()); ai::car_update_pose("move", p.as<JsonObjectConst>());
      wait_wheels(gen);
    }
  }
  JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
  return NavR::TimedOut;
}

// (请求体组装 build_body 已拆分至 src/ai/ai_prompt.cpp)

// 指令短描述(供"上一步已下发"拼接与死循环判定; 含运动数值, 便于识别"相同指令")。
static void fmt_last(char* buf, size_t cap, const char* type, const JsonObjectConst& p) {
  if (!strcmp(type, "move")) {
    float th = p["throttle"] | 0.0f;
    float st = p["steering"] | 0.0f;
    int dc = p["distance_cm"] | 0;
    int ad = p["angle_deg"] | 0;
    if (dc) snprintf(buf, cap, "move %dcm th=%.1f", dc, th);
    else if (ad) snprintf(buf, cap, "move %d度 th=%.1f", ad, th);
    else snprintf(buf, cap, "move 持续 th=%.1f st=%.1f", th, st);
  } else if (!strcmp(type, "arm")) {
    const char* act = p["act"] | "";
    int pd = p["dist_cm"] | 0;
    // 只有 抬落/前后伸 是持续型(与 exec::is_continuous 一致); clip/release/fold/low 是一次性离散。
    // 给离散动作也标"持续"会让复盘时误读成"这条动作还挂着没停"(旧日志里的 "arm clip 持续" 即此)。
    bool cont = !strcmp(act, "lift_up") || !strcmp(act, "lift_down") ||
                !strcmp(act, "reach_forward") || !strcmp(act, "reach_backward");
    if (pd) snprintf(buf, cap, "arm %s %dcm", act, pd);
    else if (cont) snprintf(buf, cap, "arm %s 持续", act);
    else snprintf(buf, cap, "arm %s", act);
  } else if (!strcmp(type, "arm_pose")) {
    float px = p["x"] | 0.0f;
    float ph = p["h"] | 0.0f;
    snprintf(buf, cap, "arm_pose x=%.0f h=%.0f", px, ph);
  } else if (!strcmp(type, "spin")) {
    int dd = p["dir"] | 0;
    int ad = p["angle_deg"] | 0;
    // 必须带上角度: 否则 "spin -1" 对任何角度都长一样 —— ①复盘时看不出 AI 打算转多少
    // (实测为了查这一点只能靠反推); ②死循环判定(cur_cmd/last_cmd 字面比对)会把
    // "每次加大角度的连续修正转"当成同一条指令重复, 满 3 轮就误注入"多轮无进展"引导语。
    if (ad) snprintf(buf, cap, "spin %d %d度", dd, ad);
    else snprintf(buf, cap, "spin %d", dd);   // 无角度=持续旋转(exec 侧另一支)
  } else if (!strcmp(type, "light")) {
    // 必须单列: 落进下面那句 else 会被写成 "stop" —— 日志里一条开灯会读成"执行 stop",
    // 复盘时正好把"灯开着"误读成"已停车"(同源不一致)。
    snprintf(buf, cap, "light %s %s", p["kind"] | "", (p["on"] | false) ? "on" : "off");
  } else {
    snprintf(buf, cap, "stop");
  }
}

// ---------------- 输出校验(防误动作) ----------------
// 校验并规范化 AI 输出到 out{type,params,reason}。返回 nullptr 通过; 否则返回错误字符串。
static const char* validate_cmd(const char* content, JsonDocument& out, char* err_buf, size_t err_cap) {
  // 剥代码块/首尾空白(Arduino String 无 find/left, 用 indexOf/substring)
  String c = content;
  c.trim();
  if (c.startsWith("```")) {
    int e = c.indexOf('\n');
    if (e >= 0) c = c.substring(e + 1);
    c.trim();
    if (c.endsWith("```")) c = c.substring(0, c.length() - 3);
    c.trim();
  }

  JsonDocument doc(&g_js_alloc);  // PSRAM 池, 避免内部堆碎片
  if (deserializeJson(doc, c)) {
    snprintf(err_buf, err_cap, "AI 返回非 JSON: %s", c.substring(0, 80).c_str());
    return err_buf;
  }
  const char* type = doc["type"] | "";
  if (strcmp(type, "move") && strcmp(type, "stop") && strcmp(type, "arm") &&
      strcmp(type, "wait") && strcmp(type, "spin") && strcmp(type, "arm_pose") &&
      strcmp(type, "approach") && strcmp(type, "zoom") && strcmp(type, "light")) {
    return "AI 输出非法 type";
  }
  if (!doc["params"].is<JsonObject>() && strcmp(type, "stop") && strcmp(type, "wait") &&
      strcmp(type, "approach") && strcmp(type, "zoom")) {
    return "AI 输出缺 params";
  }
  if (!strcmp(type, "arm")) {
    const char* act = doc["params"]["act"] | "";
    static const char* acts[] = {"lift_up","lift_down","reach_forward","reach_backward","clip","release","fold","low","grasp", nullptr};
    bool good = false;
    for (int i = 0; acts[i]; i++) if (!strcmp(act, acts[i])) { good = true; break; }
    if (!good) { snprintf(err_buf, err_cap, "AI arm 非法 act=%s", act); return err_buf; }
  }
  if (!strcmp(type, "light")) {
    // 与 arm 的 act 同样"非法值直接拒": 让 AI 收到一句明确的错, 而不是"执行了却没反应"。
    const char* kind = doc["params"]["kind"] | "";
    if (strcmp(kind, "front") && strcmp(kind, "vibe") && strcmp(kind, "back")) {
      snprintf(err_buf, err_cap, "AI light 非法 kind=%s", kind);
      return err_buf;
    }
  }

  out["type"] = type;
  JsonObject p = out["params"].to<JsonObject>();
  if (doc["params"].is<JsonObject>()) {
    JsonVariantConst src = doc["params"];
    // ⚠️ 本块**逐个显式拷贝**白名单键, 下游读的键只要漏在这里, 拿到的就永远是默认值且不报错。
    // 已踩过两次: light 漏 kind/on、zoom 漏 px/py/scale/reset —— 后者让提示词里四处"用 zoom 看
    // 目标位置"的说明全部落空(放大镜无论 AI 报哪儿都只裁画面正中心)。**新增可读键必须同步加进
    // 本块**; 下面的兜底会替你把漏掉的喊出来, 免得只能靠"现象反推"。
    static const char* kParamKeys[] = {"throttle", "steering", "distance_cm", "angle_deg", "dist_cm",
                                       "dir", "speed", "act", "scope", "x", "h", "target",
                                       "kind", "on", "px", "py", "scale", "reset",
                                       "done", "carry_prev", nullptr};
    for (JsonPairConst kv : src.as<JsonObjectConst>()) {
      const char* k = kv.key().c_str();
      bool known = false;
      for (int i = 0; kParamKeys[i]; i++) if (!strcmp(k, kParamKeys[i])) { known = true; break; }
      if (!known) ai::logf("[ai] 未识别的 params 键 %s(未透传, 下游只会拿到默认值)", k);
    }
    // 数值钳制
    float th = doc["params"]["throttle"] | 0.0f;
    float st = doc["params"]["steering"] | 0.0f;
    p["throttle"] = constrain(th, -1.0f, 1.0f);
    p["steering"] = constrain(st, -1.0f, 1.0f);
    int dc = doc["params"]["distance_cm"] | 0;
    int ad = doc["params"]["angle_deg"] | 0;
    int pd = doc["params"]["dist_cm"] | 0;
    // 距离/角度一律硬钳制(提示词里的"建议值"不能当防线): 无里程计下大步长 = 冲过头/撞墙,
    // 而大幅盲转会把画面参照全部丢失。钳制后的值会进状态反馈, AI 能看到实际执行了多少。
    if (src["distance_cm"].is<int>() && dc) p["distance_cm"] = constrain(dc, 0, AI_MOVE_MAX_CM);
    if (src["dist_cm"].is<int>() && pd) p["dist_cm"] = constrain(pd, 0, 30);   // 臂: 可达域仅 4~15cm
    int sd = doc["params"]["dir"] | 0;
    int ss = doc["params"]["speed"] | 900;  // 原地旋转默认转速(实测 <700 拖不动, 须给足)
    p["dir"] = constrain(sd, -1, 1);     // 原地旋转方向: ±1/0
    p["speed"] = constrain(ss, 0, 1000); // 原地旋转单轮 pwm
    // 定角微操(上限见 AI_SPIN_MAX_DEG)。⚠️ 这里必须保证"带方向的旋转一定带**正的**角度":
    // 原先写 `if (... && sa) p["angle_deg"] = constrain(sa, 0, AI_SPIN_MAX_DEG)` —— 模型把方向写进
    // 角度(给负数)或给 0 时, 这个键仍会被写进去(值钳成 0), 于是 exec 见 ang==0 落进"持续旋转"那一支
    // (无时限兜底), 车轮一直转到下一条指令; 同时 car_update_pose 因 ang==0 **不累积车向**, 车真转了
    // 好几圈而程序以为没转 —— 之后所有物体坐标的朝向基准全错。实测现象正是"原地连转好几圈, 转完
    // 再也找不到方块"。改为: 取绝对值(方向由 dir 定); 为 0 就**不写这个键**, 交给下游"未给
    // angle_deg 补 AI_SPIN_DEFAULT_DEG"那条兜底(它只在键缺失时生效, 补出来是有限角度)。
    if (src["angle_deg"].is<int>()) {
      int mag = ad < 0 ? -ad : ad;
      if (mag > 0) p["angle_deg"] = constrain(mag, 1, AI_SPIN_MAX_DEG);
    }
    const char* act = doc["params"]["act"] | "";
    if (act[0]) p["act"] = act;
    const char* scope = doc["params"]["scope"] | "all";
    p["scope"] = !strcmp(scope, "wheels") ? "wheels" : (!strcmp(scope, "arm") ? "arm" : "all");
    // arm_pose 指定位姿: x=轴前方 cm(可达 4..15), h=夹爪中心离地高度 cm。arm_pose 内部还有可达域检查。
    float px = doc["params"]["x"] | 0.0f;
    float ph = doc["params"]["h"] | 0.0f;
    if (src["x"].is<float>() || src["x"].is<int>()) p["x"] = constrain(px, 0.0f, 20.0f);
    if (src["h"].is<float>() || src["h"].is<int>()) p["h"] = constrain(ph, -2.0f, 25.0f);
    const char* tgt = doc["params"]["target"] | "";
    if (tgt[0]) p["target"] = tgt;
    // light: 上面已校验过 kind(必是三者之一), 这里原样透传; on 缺省 false(与 exec 侧一致)。
    if (!strcmp(type, "light")) {
      p["kind"] = doc["params"]["kind"] | "";
      p["on"] = doc["params"]["on"] | false;
    }
    // zoom: 瞄准点/倍数/复位同样必须透传(见本块开头的告警)。px/py 是**归一化画面坐标**,
    // 硬钳到 [0,1]; 缺省不写键, 让 worker 用自己的中心默认值。scale 由 worker 按上下限钳制。
    if (!strcmp(type, "zoom")) {
      if (src["px"].is<float>() || src["px"].is<int>())
        p["px"] = constrain(doc["params"]["px"] | 0.5f, 0.0f, 1.0f);
      if (src["py"].is<float>() || src["py"].is<int>())
        p["py"] = constrain(doc["params"]["py"] | 0.5f, 0.0f, 1.0f);
      if (src["scale"].is<float>() || src["scale"].is<int>())
        p["scale"] = doc["params"]["scale"] | AI_ZOOM_DEF;
      if (doc["params"]["reset"].is<bool>() && doc["params"]["reset"].as<bool>()) p["reset"] = true;
    }
    // 顶层字段被模型偶尔塞进 params 的容错(实测漏过一次 `done`): 这两个键语义没有歧义,
    // 且都到不了 exec(无安全面), 所以照顶层一样处理 —— 否则它以为标了"完成"而程序没收到,
    // 任务在该结束的轮次继续空跑。kParamKeys 里登记它们, 免得再报成"未识别"。
    if (doc["params"]["done"].is<bool>() && doc["params"]["done"].as<bool>()) out["done"] = true;
    if (doc["params"]["carry_prev"].is<bool>() && doc["params"]["carry_prev"].as<bool>())
      out["carry_prev"] = true;
  }
  const char* reason = doc["reason"] | "";
  if (reason[0]) out["reason"] = reason;
  const char* tg = doc["task_goal"] | "";
  if (tg[0]) out["task_goal"] = tg;   // 选项: 把插话/新意图提升为当前任务目标(AI 显式标记)
  if (doc["done"].is<bool>() && doc["done"].as<bool>()) out["done"] = true;  // 任务完结标记
  // carry_prev:true = 下一轮希望同时收到本轮画面做对比(目标锁定/追踪、判断移动后目标方位)。
  // 本字段不进 params, 仅作 worker 决策是否带 prev 的信号。
  if (doc["carry_prev"].is<bool>() && doc["carry_prev"].as<bool>()) out["carry_prev"] = true;
  // observe:true = AI 的空间观测(name/rel_deg/dist_cm/visible), 透传给 worker 更新物体记忆表。
  if (doc["observe"].is<JsonObject>()) out["observe"] = doc["observe"].as<JsonObjectConst>();
  return nullptr;
}

// ---------------- HTTPS POST / chunked 解码已拆分至 src/ai/ai_http.cpp ----------------
// (http_post / read_chunked_body / ai_tls_calloc / ai_tls_free 及已废弃的手写 http_exchange
// 均移入独立 TLS 传输模块; worker 经 ai::http_post / ai::http_last_status / ai::http_stop 调用)

// 从响应提取 choices[0].message.content; 同时打印 reasoning_content(思考过程, 截断防刷屏)。
// broken: 解析失败时置 true(多半是传输层截断/复用残留, 调用方应弃用复用连接)
static bool extract_content(const String& resp, String& content, bool* broken = nullptr) {
  // 若响应是 SSE 文本(chunked-SSE 解码产物): 按行取 data: 负载拼成最终 JSON
  String payload = resp;
  if (resp.startsWith("data:") || resp.indexOf("\ndata:") >= 0) {
    payload = "";
    int i = 0, n = resp.length();
    while (i <= n) {
      int j = resp.indexOf('\n', i);
      if (j < 0) j = n;
      String ln = resp.substring(i, j);
      i = j + 1;
      ln.trim();
      if (ln.startsWith("data:")) {
        String d = ln.substring(5); d.trim();
        if (d == "[DONE]") break;
        payload += d;
      }
    }
  }
  // 剥离开头泄漏的 chunked size 行(形如 "XX\r\n{" / "XX\n{", XX 为 1-8 位十六进制长度)。
  // 复用连接偶发把某次响应的 size 行错位拼进下一条响应体顶部, deserializeJson 直接失败; 
  // JSON 响应永远以 '{' 起始、永不始于 hex, 故只在"hex+换行+{" 时剥离, 绝不误伤正文。
  {
    auto ishex = [](char c){ return (c>='0'&&c<='9')||(c>='a'&&c<='f')||(c>='A'&&c<='F'); };
    for (int s = 0; s < 4; s++) {              // 最多剥 4 段(防多块错位叠加)
      const char* q = payload.c_str();
      int i = 0;
      while (i < 8 && ishex(q[i])) i++;        // 扫描十六进制前缀
      if (i == 0) break;                       // 开头非 hex, 非泄漏
      int j = i;
      if (q[j] == '\r') j++;
      if (q[j] == '\n') j++;
      if (q[j] != '{') break;                  // 后随非 '{', 按正文处理
      payload = payload.substring(j);          // 剥掉这段 size 行
    }
  }
  JsonDocument doc(&g_js_alloc);
  if (deserializeJson(doc, payload)) {
    if (broken) *broken = true;  // 解析失败即视为连接可疑: 截断/残留污染, 调用方弃用复用连接
    // 解析失败诊断: 打印完整 payload + 结构判据(判断是状体被拼断/chunked 泄漏/SSE 多段拼接)。
    auto sanitize = [](String s) {
      for (int i = 0; i < s.length(); i++) {
        unsigned char ch = (unsigned char)s[i];
        if (ch < 0x20 || ch == '\x7f') s[i] = '?';
      }
      return s;
    };
    // 结构判据: 是否含 "\ndata:"(SSE 多帧)、是否含多个 "{"id"..."}(多 JSON 拼接)、是否含 chunk 大小泄漏前缀。
    bool sse = payload.indexOf("\ndata:") >= 0 || payload.startsWith("data:");
    int multi_json = 0;
    for (int i = payload.indexOf("{\"id\""); i >= 0 && i < payload.length(); i = payload.indexOf("{\"id\"", i + 1)) multi_json++;
    String full = sanitize(payload);
    ai::logf("[ai] JSON失败 len=%d sse=%d 多json=%d 截断=%d 全文:%s",
             (int)payload.length(), sse ? 1 : 0, multi_json,
             (payload.length() > 0 && payload[payload.length() - 1] != '}' && payload[payload.length() - 1] != ']') ? 1 : 0,
             full.c_str());
    return false;
  }
  const char* rc = doc["choices"][0]["message"]["reasoning_content"] | "";
  if (rc[0]) {
    // 完整思考只打串口（流式，不申请缓冲/不占栈，也不转发手机）；调试/评估用，不回投模型。
    Serial.print("[ai] 思考: "); Serial.println(rc);
  }
  const char* c = doc["choices"][0]["message"]["content"] | "";
  // 注意: content 为空但 reasoning 有值 = 模型只给了思考没给正式回答(安全终止/条件触达)
  if (!c[0]) {
    // 空 content 诊断: 必须打出 finish_reason 与用量 —— 二者都在响应**尾部**, 而旧的
    // "响应无内容, 原始(前120B)" 只打头部, 永远够不着它们, 于是两种完全不同的病因在日志里
    // 长得一模一样: (a) 模型把 max_tokens 预算全花在思考上(finish=length, 正式回答还没开始);
    // (b) 端点返回空/被过滤的 choices 或把 content 返成数组(取不到字符串)。
    // content 的类型也一并打: null / 字符串 / 数组 三种取不到值的情况要分得开。
    JsonVariant cv = doc["choices"][0]["message"]["content"];
    const char* ct = cv.isNull() ? "null" : cv.is<const char*>() ? "str" : cv.is<JsonArray>() ? "arr" : "其它";
    const char* em = doc["error"]["message"] | "";
    ai::logf("[ai] 空content: choices=%d finish=%s content=%s(%uB) reasoning=%uB 补全/总=%u/%u%s%s",
             (int)doc["choices"].size(), doc["choices"][0]["finish_reason"] | "(无)",
             ct, (unsigned)strlen(c), (unsigned)strlen(rc),
             (unsigned)(doc["usage"]["completion_tokens"] | 0), (unsigned)(doc["usage"]["total_tokens"] | 0),
             em[0] ? " err=" : "", em);
    if (rc[0]) { Serial.print("[ai] 思考: "); Serial.println(rc); }   // 完整思考只打串口, 不转发手机
  }
  content = c;
  return content.length() > 0;
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
    if (cmd["params"].is<JsonObject>()) inner["params"] = cmd["params"].as<JsonObjectConst>();
    const char* rs = cmd["reason"] | "";
    if (rs[0]) inner["reason"] = rs;
  }
  String s;
  serializeJson(out, s);
  return s;
}

// ---------------- worker 任务 ----------------

static void ai_worker(void*) {
  for (;;) {
    xSemaphoreTake(g_notify, portMAX_DELAY);

    TaskLocal t;
    xSemaphoreTake(g_mtx, portMAX_DELAY);
    if (g_slot.active) {
      t = g_slot;
      g_slot.text = nullptr; g_slot.ann = nullptr; g_slot.ctx = nullptr; g_slot.active = false;
    }
    xSemaphoreGive(g_mtx);
    if (!t.text && !t.nav) continue;

    // 纯导航任务(/move to): 不调云端, 直接本地巡航到目标坐标即回报。
    if (t.nav) {
      m_busy = true;
      if (!t.nav_global) { ai::s_car_x = 0; ai::s_car_y = 0; ai::s_car_heading = 0; }  // local=以当前位姿为新原点
      NavR r = navigate_to(t.generation, t.nav_x, t.nav_y, NAV_STOP_CM_GOTO);
      JsonDocument f(&g_js_alloc);
      f["type"] = "ai_result";
      if (t.id) f["id"] = (long)t.id;
      f["params"]["reason"] = r == NavR::Reached ? "已到达目标坐标" : "导航被中断";
      f["params"]["done"] = (r == NavR::Reached);
      String s; serializeJson(f, s);
      enqueue_result(s.c_str(), t.fn, t.ctx);
      blog::logf(blog::AI, "导航结束 rel=%d", (int)r);
      if (t.ctx) delete (int*)t.ctx;
      m_busy = false;
      continue;
    }

    g_last_continuous = false;  // 本任务尚未下发过持续指令(防上一任务残留标志误判)
    g_last_cont_type = 0;
    m_busy = true;
    // 新任务 = 新坐标系: 车位置=原点、初始车头=0°, 清空上一任务的空间记忆。
    ai::mem_reset();
    // 水位清零: 本任务的低点从此算起。一轮 AI 就是内部堆被压得最深的时候
    // (TLS 组包/收响应 + 双帧 PSRAM 副本 + 历史环), 低点落到哪正是要量的事。
    hwatch::reset_min();
    // 摄像头可用性在任务起点判定(init 后即定): 不可用则整轮走无画面降级
    bool cam_ok = cam::available();
    ai::logf("[ai] 任务开始 gen=%lu text=%s%s", t.generation, t.text, cam_ok ? "" : "(无摄像头→无画面模式)");
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

    unsigned long steps = 0;
    bool done = false;
    bool sent_done = false;      // 是否已确报过终态 done(正常/单轮/超步数/超连续失败)
    bool interrupted = false;    // 是否因新目标/手动中断退出(代际变更), 此时不发补发 done
    const char* fail = nullptr;
    char err_buf[160];
    // 死循环防线状态: 上一步指令短描述 / 连续相同指令计数 / 已注入引导标记。
    char last_cmd[48] = {0}, cur_cmd[48] = {0};
    unsigned long last_act_ms = 0;  // 上次真正下执行/微操指令的时刻(ms), 供给 AI 算"距上次决策多久"
    int net_fail = 0;           // 连续"无有效输出"轮数(网络/解析失败), 用于退避与上限收尾
    int stall = 0;
    bool stall_hint = false;
    // 夹取闸门状态(见下面"夹取闸门"段):
    char grasp_warn[224] = {0};  // 动作被拒的原因, 下一轮当引导语喂回(一次性; 与 obs_warn 同理)
    bool since_release = false;  // 上次臂动作是 release 且此后位姿/位置没变过(原样重夹必再空夹)
    bool pose_low = false;       // 刚把夹爪下探到位: 下一轮提示先看画面确认, 再决定夹还是收臂重对
    // 刚在低姿前顶过方块(见 AI_LOW_PUSH_TTL): 期间闸门给"退1cm再夹"这条配方, 不按坐标点名前后方向 ——
    // 顶完这一下坐标就不代表"方块在哪"了, 而实测的循环恰恰是"顶→坐标报远→再顶→…"。
    int  low_push_ttl = 0;
    bool want_prev = false;     // AI 上轮 carry_prev=true → 本轮带上 prev 帧做对比
    // 放大镜状态(见"放大镜"段)。裁框一律以**全幅归一化**坐标记账: 这是"换回全幅是算术而非估计"
    // 的全部依据, 所以每次发出去都留一份 sent_*(而不是拿待用值当已用值)。
    bool  zoom_on = false;              // 下一帧起是否发放大图(AI 用 zoom 命令开/关)
    bool  sent_zoomed = false;          // 本轮实际发出去的**是不是**放大图(observe 反算要用)
    float zoom_x0 = 0, zoom_y0 = 0, zoom_x1 = 1, zoom_y1 = 1;   // 待用裁框
    float sent_x0 = 0, sent_y0 = 0, sent_x1 = 1, sent_y1 = 1;   // 本轮实际发出的裁框
    float zoom_scale = AI_ZOOM_DEF;     // 待用倍数(日志/回告用)
    uint8_t* zoom_jpg = nullptr;        // 放大图缓冲(PSRAM, 首次用到时分配)
    char zoom_warn[192] = {0};          // 放大相关的**一次性**告知(生成失败/被强制回全幅), 见 hpush 处
    // 重复请求放大镜的**每轮**告知(不是一次性的): 空操作时画面本就是放大图(sent_zoomed=true),
    // 而 zoom_warn 只在 !sent_zoomed 时才推 —— 于是"别再重复发"这句从来没到过 AI 手上, 它拿到
    // 一张没变的图 + 一句"这是放大图", 只能猜命令生效没有, 实测原地复读 6~10 次。改成本缓冲按
    // 重复次数累加、每轮照推, 画面真的变了才清零。
    char zoom_repeat[224] = {0};
    int  zoom_noop_n = 0;               // 连续"要了手上已经有的那块"的次数
    char zoom_pose[48] = {0};           // 上次放大时的车位姿+臂姿: 判"重复请求"时要看这期间动过没有
    char spin_warn[224] = {0};          // 旋转预算相关的一次性告知(见 AI_SPIN_TASK_BUDGET_DEG), 同 hpush 处
    char near_warn[224] = {0};          // 近场步长钳制的一次性告知(见 AI_NEAR_FWD_CM), 同 hpush 处
    char near_blind[224] = {0};         // 近场裁决的**每轮**提示(见 AI_NEAR_BLIND_CM), 同 hpush 处
    char arm_low_warn[224] = {0};       // "低姿转身"告警(见 AI_ARM_LOW_H_CM), 同 hpush 处
    int  spin_total_deg = 0;            // 本任务累计已转角度(绝对值累加; 局部量, 随任务重建)
    int  spin_align_deg = 0;            // 其中"对准转向"的累计用量(走独立额度, 见 AI_SPIN_ALIGN_BUDGET_DEG)
    bool spin_align_now = false;        // 本轮这次 spin 是否算"对准"(见下面档位钳制处): 有新鲜观测
                                        // ⇒ 它是在对准而不是在搜索, 走独立额度、不占搜索预算
    int  idle_rounds = 0;               // 连续"无实际动作"轮数(见 AI_IDLE_ROUNDS)
    bool acted = false;                 // 本轮是否有实际动作落地(exec 接受)
    // AI 报的像素(它看到的那张图) → 全幅归一化。上一轮发的是全幅就直通; 是放大图就按当时记下的裁框
    // 反算 —— 这正是"裁框由程序记账"的用处: 换回全幅是算术, 不含任何估计, 于是连续放大也不会漂。
    auto to_full_x = [&](float px) { return sent_zoomed ? sent_x0 + px * (sent_x1 - sent_x0) : px; };
    auto to_full_y = [&](float py) { return sent_zoomed ? sent_y0 + py * (sent_y1 - sent_y0) : py; };
    // 以 (fx,fy) 为中心、取**全幅**的 1/scale 作为新框(scale 是"相对全幅的绝对倍数")。
    // ⚠️ 必须是绝对倍数, 不能按"相对当前视图"累乘: 契约(提示词写的是"放大倍数…约1.5~8")和 AI 的
    // 预期都是绝对的 —— 它连发三条 scale=2 想要的就是"比全幅大 2 倍", 累乘却给了 2×→4×→8×,
    // 而 8× 时视场只剩 12%, 夹爪早被裁出画面, 它据此判断"方块在不在两指之间"就没有参照了
    // (实测: 它以为在看 2×, 拿到的是 8×, 且每次都白烧一轮才被强制回全幅)。
    // px/py 仍用**它当前那张图**的 0~1 坐标(sent_* 反算回全幅), 于是反复放大/挪动都不漂。
    // 贴边时整体平移而**不缩小**: 缩小会改变倍数, 让 AI 对"方块在画面里该多大"的预期失准。
    // 想看得更近就直接把 scale 填大(上限 AI_ZOOM_MAX), 想回全幅用 reset。
    auto set_zoom_box = [&](float fx, float fy, float sc) {
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
    };
    char task_note[192] = {0};  // AI 写入的任务笔记(目标外观/计划), 每轮喂回
    struct { char name[48]; bool done; } s_tasks[8] = {{0}};  // AI 维护的任务列表(JSON 更新)
    int s_task_n = 0;
    // 当前任务目标(独立 user 消息展示; 可被 task_goal 热替换)
    char goal_now[256];
    snprintf(goal_now, sizeof(goal_now), "%s", t.text ? t.text : "");
    // 历史环(PSRAM 动态分配): AI 决策(assistant) + 插话(user) 共用一条管理, 满员淘汰最旧。
    char* hist_text[AI_HIST_N] = {nullptr};
    const char* hist_role[AI_HIST_N] = {nullptr};   // "assistant" / "user"
    int hist_n = 0;
    char task_remind[160] = {0};  // 插话即将被冲掉前的提醒(提示补 task_goal 更新目标), 一次性消费
    // 观测未记成的回告(一次轮消费): observe 被丢弃时 AI 无从得知, 会以为已记住该物体,
    // 于是反复 observe 或凭猜导航。明确回告让它改用画面重新定位。
    char obs_warn[160] = {0};
    auto ps_dup = [](const char* s) -> char* {   // 拷贝到 PSRAM(供历史条用)
      size_t n = strlen(s);
      char* p = (char*)heap_caps_malloc(n + 1, MALLOC_CAP_SPIRAM);
      if (p) memcpy(p, s, n + 1);
      return p;
    };
    auto hist_add = [&](const char* role, const char* text) {   // 推入历史环(满则冲掉最旧)
      char* dup = ps_dup(text);
      if (!dup) return;
      if (hist_n < AI_HIST_N) { hist_role[hist_n] = role; hist_text[hist_n] = dup; hist_n++; }
      else {
        if (hist_role[0] && !strcmp(hist_role[0], "user"))   // 被冲掉的恰是插话: 提醒先 task_goal 更新目标
          snprintf(task_remind, sizeof(task_remind),
                   "较早的一条操作者插话即将被历史丢弃; 若它表达了新任务/新目标而你还未用 task_goal 更新, 请现在更新。");
        free(hist_text[0]);
        memmove(hist_text, hist_text + 1, (AI_HIST_N - 1) * sizeof(hist_text[0]));
        memmove(hist_role, hist_role + 1, (AI_HIST_N - 1) * sizeof(hist_role[0]));
        hist_text[AI_HIST_N - 1] = dup; hist_role[AI_HIST_N - 1] = role;
      }
    };

    // 编辑图一次性取快照(供整轮任务复用, 避免中途被覆盖)。
    uint8_t* edited = nullptr; size_t edited_len = 0;
    if (t.use_image) {
      edited = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
      if (edited && !take_edited(edited, AI_EDITED_IMG_MAX, &edited_len)) { free(edited); edited = nullptr; }
    }
    // 本轮帧的 PSRAM 副本(任务期复用): 抓帧后立即把字节拷进来并归还相机缓冲, 
    // 避免 AI 在做慢速 TLS 请求期间长时间占用 fb_count=2 的缓冲池把推流饿死。
    uint8_t* cur = nullptr; size_t cur_len = 0;
    // 上一帧 PSRAM 副本(周期性双帧运动对比用; 任务期复用)。
    uint8_t* prev = nullptr; size_t prev_len = 0;

    // 兜底 stop(任务终结出口统一解析一次)。g_stop_mode 由打断方写入: 
    // None=手动 move/stop 接管(用户指令已覆盖, 不补停); Wheels=手动 arm(只停轮子); All=其余。
    // 仅当存在持续指令残留才补。
    auto resolve_stop = [&]() {
      if (g_stop_mode == (int)ai::StopMode::None) return;
      if (!g_last_continuous) return;
      // Wheels 模式只对轮子持续残留停轮子; 臂持续残留(或未知)必须全停。
      const char* scope = (g_stop_mode == (int)ai::StopMode::Wheels && g_last_cont_type == 1) ? "wheels" : "all";
      JsonDocument d; d["scope"] = scope;   // d 即 stop 的 params 对象
      exec::act("stop", d.as<JsonObjectConst>());
      blog::logf(blog::AI, "兜底 stop scope=%s", scope);
      g_last_continuous = false;
    };

    // 操作者插话"未落实"标记: 插话在组包时即被消费, 若那轮响应报废(超时/空 content)就等于
    // 没被执行过 —— 一轮轮点它的名, 直到某一轮真跑成了一个动作(见下方的清除点)。仅本任务内有效。
    bool chat_unacked = false;
    while (!done) {
      uint64_t step_ts = esp_timer_get_time();  // 本轮起点(周期控制基准)
      fail = nullptr;  // 每轮重置, 避免沿用上轮错误文本误导日志/回报
      // 中止检查(代际号变化即本任务作废); 兜底停统一在任务出口解析。
      if (t.generation != m_generation) { blog::logf(blog::AI, "被新目标/手动中断"); interrupted = true; break; }
      if (!net::is_connected()) { snprintf(err_buf, sizeof(err_buf), "WiFi 掉线"); fail = err_buf; break; }
      if (cfg::ai_key().isEmpty()) { snprintf(err_buf, sizeof(err_buf), "未配置 AI Key"); fail = err_buf; break; }

      // 取当前帧: 先等停稳(画面清晰) → 失败重试; 无摄像头则跳过, 走无画面降级
      bool frame_moving = false;
      camera_fb_t* fb = nullptr;
      if (cam_ok) {
        settle_wheels(t.generation, AI_SETTLE_MAX_MS);
        frame_moving = exec::wheels_moving();   // 仍在动: 本帧仅供粗略参考, 下轮提示 AI 别硬信
        for (int fr = 0; fr < AI_FRAME_RETRY && !fb; fr++) {
          fb = cam::grab();
          if (!fb && fr < AI_FRAME_RETRY - 1) vTaskDelay(pdMS_TO_TICKS(80));
        }
        // 取帧失败不再一次即判死整个任务(并发推流占满 fb_count 时属偶发): 计入失败轮次,
        // 与"云端无响应"共用同一条连续失败上限, 退避后重试; 连续多轮取不到才收尾。
        if (!fb) {
          if (++net_fail >= AI_MAX_NET_FAIL) {
            snprintf(err_buf, sizeof(err_buf), "持续取帧失败(摄像头/缓冲异常)");
            fail = err_buf;
            break;
          }
          ai::logf("[ai] 取帧失败(第%d次), 稍后重试", net_fail);
          vTaskDelay(pdMS_TO_TICKS(300 * net_fail));
          continue;
        }
      }
      const uint8_t* frame = fb ? fb->buf : nullptr;
      // 必须用 cam::jpeg_len 而非 fb->len: 驱动会把 len 报大(缓冲里可能拼了 2/4/6 帧),
      // 虚高的 len 会被原样 base64 进请求体 → body 从 ~35KB 顶到 190KB → 压垮 WiFi 发送路径。
      // 详见 camera.cpp 的 jpeg_len 说明。
      size_t frame_len = cam::jpeg_len(fb);
      // 源帧分辨率必须**在归还相机缓冲之前**取: fb 归还后其字段即失效(见下)。
      int src_w = 0, src_h = 0;
      if (fb) { src_w = fb->width; src_h = fb->height; }

      // 抓帧后立即拷入 PSRAM 副本并归还相机缓冲: AI 的 HTTPS(TLS 握手+请求)
      // 慢则数秒, 期间一直占着 fb 会把 fb_count=2 的缓冲池耗尽, 饿死并行推流。
      // 帧数据后续(body 组装)一律用这份副本。
      if (fb && frame_len > 0 && frame_len <= AI_EDITED_IMG_MAX) {
        if (!cur) cur = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
        if (cur) { memcpy(cur, frame, frame_len); cur_len = frame_len; }
        else cur_len = 0;
      }
      cam::return_frame(fb);
      // 副本就绪才认本帧(fb 已归还, 直接指向裸 fb->buf 即为悬垂): 过大或拷入失败则按无画面处理。
      if (cur_len > 0) { frame = cur; frame_len = cur_len; }
      else { frame = nullptr; frame_len = 0; }

      // ---- 放大镜: 把 AI 上一轮点名的那块裁出来放大, 本轮就发这张 ----
      // AI 要的不是"更准的坐标", 而是"能看清": 在放大图里它可以拿目标跟**看得见的夹爪**比相对位置
      // (它的强项), 而裁框是程序记的账 ⇒ 它报回的全幅坐标是算术, 不是估计。任何目标都适用, 程序不
      // 需要认识那是什么东西 —— 这是它跟"按颜色量尺寸"那类硬编码工具的根本区别。
      // 必须裁 **cur 这份字节**(AI 收到的就是它, 只是裁小): 另抓一帧会在近距裁到"更新的一帧",
      // 与它决策依据的那帧对不上账(这一带帧龄折算到地面是厘米级)。
      sent_zoomed = false;
      // 本帧若发的是全幅, "AI 手上那张图"就是全幅 —— sent_* 必须跟着复位。原先只在裁图成功时写、
      // 回全幅时不清, 于是 sent_* 永远停在上一次放大那块: AI 再要同一块就被判"无变化", 画面却已在
      // 全幅 → **它再也拿不到想要的近景**(实测整轮 13 次 zoom 全被吞掉, 而近场对准恰恰全靠这个:
      // 它得在放大图里看清"目标在不在两指之间")。sent_* 的语义 = AI 此刻收到的画面框。
      sent_x0 = 0; sent_y0 = 0; sent_x1 = 1; sent_y1 = 1;
      if (zoom_on && frame && frame_len > 0 && src_w > 0 && src_h > 0) {
        if (!zoom_jpg) zoom_jpg = (uint8_t*)heap_caps_malloc(AI_ZOOM_JPG_MAX, MALLOC_CAP_SPIRAM);
        size_t zl = 0;
        if (zoom_jpg && magnify::crop_to_jpg(frame, frame_len, src_w, src_h,
                                             zoom_x0, zoom_y0, zoom_x1, zoom_y1,
                                             zoom_jpg, AI_ZOOM_JPG_MAX, &zl,
                                             AI_ZOOM_OUT_W, AI_ZOOM_OUT_H, AI_ZOOM_QUALITY) && zl > 0) {
          frame = zoom_jpg; frame_len = zl;
          sent_zoomed = true;
          sent_x0 = zoom_x0; sent_x1 = zoom_x1; sent_y0 = zoom_y0; sent_y1 = zoom_y1;
          ai::logf("[放大镜] %.1f× 裁框(%.2f,%.2f)-(%.2f,%.2f) → %uKB %dms",
                   zoom_scale, zoom_x0, zoom_y0, zoom_x1, zoom_y1,
                   (unsigned)(zl / 1024), magnify::last_cost_ms());
        } else {
          // 生成失败(解码/编码/分配)不能让本轮瞎: 退回全幅, 并把这事说给 AI —— 它以为在看放大图。
          ai::logf("[放大镜] 生成失败(%dms 源%dx%d), 本轮回全幅", magnify::last_cost_ms(), src_w, src_h);
          snprintf(zoom_warn, sizeof(zoom_warn), "放大图生成失败, 本帧仍是全幅(按全幅读)");
        }
      }

      bool got = false;
      for (int attempt = 0; attempt < 2 && !done; attempt++) {
        // 引导语: 重试纠正 / 死循环打断(连续多轮相同指令, 带重复指令名便于 AI 自我纠正)
        char hint_buf[192];
        const char* hint;
        if (attempt) {
          // "没给正式回答(content 为空, 只有思考)"与"输出非法 JSON"是两种病, 提示要分开:
          // 前者再说"只输出合法 JSON"没用(它根本没输出), 得让它别再长篇思考、直接给指令。
          hint = (fail && !strcmp(fail, "AI 响应无内容"))
                   ? "上轮你只给了思考过程、没有给出正式回答(content 为空); 本轮请勿再长篇思考, 直接输出一个合法 JSON 词表指令。"
                   : "上次输出非法, 请只输出合法 JSON 词表指令。";
        } else if (stall_hint) {
          snprintf(hint_buf, sizeof(hint_buf),
                   "连续多轮重复「%s」无进展: 先改变观察角度重新确认目标(抬/落机械臂、后退或环视), 若夹爪已夹紧或无法达成则输出 stop。", last_cmd);
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
        xSemaphoreTake(g_mtx, portMAX_DELAY);
        if (g_chat_has) { strncpy(chat_now, g_chat, sizeof(chat_now) - 1); utf8_clamp_tail(chat_now); g_chat_has = false; }
        xSemaphoreGive(g_mtx);
        if (chat_now[0]) { hist_add("user", chat_now); chat_unacked = true; }   // 插话进入共享历史(操作者话语), 落实前一直点名
        // 上一帧是否带上: 仅由 AI 上轮 carry_prev=true 决定(锁定/追踪意图), 其余保持单帧省开销。
        // 放大镜下不带上一帧: prev 存的是全幅, 两张图参照系不同(同一像素在两图里指的不是一处),
        // 对比运动只会误导。放大镜本来就是用来"静态看清相对位置"的, 不需要跨帧运动对比。
        bool use_prev = want_prev && prev_len > 0 && !sent_zoomed;
        // 操作者参考图: 仅首轮带一次(初始目标外观参考); 之后不续带(已去掉 carry_user)。
        bool use_edited_now = (steps == 0) && edited != nullptr;
        // 抓帧留档(调试旁路, 见 ai_dump.h): 把**本轮原样发出去的那帧**留一份, 供事后复盘
        // "它当时到底看见了什么" —— 否则只能对着日志里的坐标猜它看见了什么。关着时零开销。
        // 放在这里而不是抓帧处: 此刻才算出 use_prev(本轮是否双帧), 标注才说得清它看到几张图。
        ai::dump_push(frame, frame_len, use_prev);
        if (sent_zoomed) {   // 标注这帧是放大图(裁框/倍数), 否则复盘时容易把局部图当全幅读
          char zb[72];
          snprintf(zb, sizeof(zb), "[放大镜%.1f×] 全幅(%.2f,%.2f)-(%.2f,%.2f)",
                   zoom_scale, sent_x0, sent_y0, sent_x1, sent_y1);
          ai::dump_note(zb);
        }
        // 渲染任务列表喂回: 任务列表: 1.出门[完成] 2.右转[未完成] ...
        char task_s[320] = {0};
        if (s_task_n > 0) {
          int tp2 = snprintf(task_s, sizeof(task_s), "任务列表: ");
          for (int ti = 0; ti < s_task_n && tp2 < (int)sizeof(task_s) - 48; ti++)
            tp2 += snprintf(task_s + tp2, sizeof(task_s) - tp2, "%d.%s[%s] ",
                            ti + 1, s_tasks[ti].name, s_tasks[ti].done ? "完成" : "未完成");
        }
        // 合并本轮提示(在插话 hist_add 之后构建, 确保其触发的 task_remind 本轮可见)。
        // 按"本轮最相关"到"兜底引导"排序, 因为缓冲区满时会截断尾部:
        // 画面在动/观测被丢(本轮事实) → 插话将冲掉 → 死循环引导。
        // 近场但爪还没降下来(见 AI_NEAR_FWD_CM): 目标已进车前 14cm 内, 而爪还折着/高悬 —— 放大图里
        // 没有两指参照, AI 永远确认不了对准, 只会反复转车空转(实测: 目标已到(0.1,10.8)接近完美位置,
        // AI 仍连转 10°×2 把目标甩丢, 全程一次 arm 都没发)。每轮提示先 arm low, 无目标依据时不推。
        static char near_hover_warn[224] = {0};
        near_hover_warn[0] = 0;
        {
          char nname[16] = {0}; int nage = 0;
          float nfwd = 0, nlat = 0, nmv = 0, ntn = 0;
          float hx = 0, hh = 0;
          if (ai::mem_grasp_evidence(nname, sizeof(nname), &nage, &nfwd, &nlat, &nmv, &ntn) &&
              nfwd > 0 && nfwd <= AI_NEAR_FWD_CM &&
              exec::arm_pos(&hx, &hh) && hh > AI_ARM_LOW_H_CM) {
            // 爪口还在"贴地低姿"线以上 ⇒ 基本是仍折着/高悬: 两指不在画面里, 没法定横向。
            snprintf(near_hover_warn, sizeof(near_hover_warn),
                     "目标已在前 %.0fcm(近场)但爪还在高位(高%.0fcm, 两指不在画面里): "
                     "先 arm low 把爪降到贴地低姿, 之后高度就不用再管, 只需小步 move 把方块送进两指之间 ——"
                     "别在爪没降下来时反复转车, 那只会把目标甩出视野",
                     nfwd, hh);
            ai::logf("[ai] 近场未降爪提醒(爪高%.0fcm, 目标前%.0fcm)", hh, nfwd);
          }
        }
        // 近场裁决 (见 AI_NEAR_BLIND_CM): 每轮按最新依据重算; 目标离开近场或依据失效就把这条撤掉。
        // ⚠️ 这张表曾经是**死代码**: bfwd 声明后再没被赋值过, 于是它每轮都对 AI 说"目标车前 0cm,
        // 真距约 0cm"—— 而提示词当时把这一行定为夹取步的唯一判据。所有依据都从 mem_grasp_evidence
        // 现取, 任一为空就不推这一条(宁可不说, 也不给一句假事实)。
        {
          char bname[16] = {0}; int bage = 0;
          float bfwd = 0, blat = 0, bmv = 0, btn = 0;
          near_blind[0] = 0;
          if (ai::mem_grasp_evidence(bname, sizeof(bname), &bage, &bfwd, &blat, &bmv, &btn) &&
              bfwd > 0 && bfwd <= AI_NEAR_BLIND_CM) {
            // 用**当前爪口前距**(低姿下就是夹心真值)做对照: 坐标是高报的解算值, 爪口是 FK 真值,
            // 两者只能比量级, 不能相减, 所以这里只给"明显更远/差不多/明显更近"三档。
            float gx = 0, gh = 0;
            bool has_g = exec::arm_pos(&gx, &gh);
            float true_d = bfwd * AI_NEAR_SCALE;
            const char* verdict = !has_g ? "以画面为准"
                                 : (true_d > gx + 1.5f ? "还差一点→小步 move 前进(≤2cm)顶到两指之间"
                                 : (true_d < gx - 1.5f ? "已过冲→小步 move 后退, 别继续顶"
                                                       : "真距≈爪口: zoom 确认方块在两指之间就 arm grasp"));
            snprintf(near_blind, sizeof(near_blind),
                     "近场裁决: 目标坐标车前%.0fcm(高报⇒真距约%.0fcm), 爪口%s: %s",
                     bfwd, true_d, has_g ? "在车前同量级处" : "位置未知", verdict);
          }
        }
        char hint_cb[640] = {0};
        auto hpush = [&](const char* s) {
          size_t l = strlen(hint_cb);
          if (l) { if (l + 2 >= sizeof(hint_cb)) return; hint_cb[l++] = ';'; hint_cb[l++] = ' '; }
          snprintf(hint_cb + l, sizeof(hint_cb) - l, "%s", s);
        };
        // 放大镜提示排在最前: 它是本轮画面的性质(而不是某条建议), 读错整帧就白看了。
        // 只给事实不给道理 —— "该怎么用"写在系统提示词里, 这里省下的字节留给后面的提示
        // (hint_cb 满时截尾, 而插话/拒因/下探复看那几条本轮的硬事实更不能丢)。
        // 重复要同一块排在最前(它比"这是放大图"更该先被读到): 与下面**并列**而不是互斥 ——
        // 空操作时 sent_zoomed 一定是 true, 写成 else 就永远推不出去(见 zoom_repeat 声明处)。
        if (zoom_repeat[0]) hpush(zoom_repeat);
        if (sent_zoomed) hpush(AI_ZOOM_HINT);
        else if (zoom_warn[0]) { hpush(zoom_warn); zoom_warn[0] = 0; }
        if (frame_moving)
          hpush("本帧是车/臂仍在移动时拍摄的, 画面可能模糊位移, 方位判断不可靠; 宜先 wait 待停稳再据画面决策");
        if (obs_warn[0]) { hpush(obs_warn); obs_warn[0] = 0; }
        if (grasp_warn[0]) { hpush(grasp_warn); grasp_warn[0] = 0; }
        if (spin_warn[0]) { hpush(spin_warn); spin_warn[0] = 0; }
        if (near_warn[0]) { hpush(near_warn); near_warn[0] = 0; }
        // 近场裁决(每轮都推, 只要依据里的目标还在 AI_NEAR_BLIND_CM 以内; 目标离开近场/依据失效即停):
        // 这是"看不见目标时该干什么"的唯一事实来源 —— 原先那条"看不见多是被两指挡住"只说了一半,
        // AI 分不清"在两指之间(该夹)"和"已顶进盲区(该退)", 于是拿 zoom 反复试探到超时(实测)。
        if (near_blind[0]) hpush(near_blind);
        // 近场但爪还在高位(每轮都推, 只要条件成立): 让 AI 先 arm low 降爪, 而不是反复转车 ——
        // 排 near_blind 之后: 两条说的都是"先把爪降到位再谈对准", 不冲突; 若没到裁决线,
        // 这条仍能把"降爪"这个动作顶到 AI 面前(实测空转的根因就是它全程没降爪)。
        if (near_hover_warn[0]) hpush(near_hover_warn);
        if (arm_low_warn[0]) { hpush(arm_low_warn); arm_low_warn[0] = 0; }
        if (chat_unacked) hpush(AI_CHAT_UNACK_HINT);   // 操作者插话尚未落实(它的那一轮响应报废了)
        // 下探到位后反复提示(直到它据落位后的画面 observe 一次, 或改变位姿/位置为止):
        // "夹爪放下后不重新对准就夹"是这条链路最贵的错误 —— 空夹一轮 = 一整次云端往返 + 一次
        // 抬起验证 + 一次重试。提示排在观测/拒因之后(前两条是本轮硬事实), 排在插话提醒之前。
        if (pose_low) hpush(AI_GRASP_CHECK_HINT);
        if (task_remind[0]) { hpush(task_remind); task_remind[0] = 0; }
        if (hint && hint[0]) hpush(hint);
        // 历史环逐条喂给 build_body(assistant=AI决策 / user=插话, 真多轮对话)
        build_body(body, goal_now, t.ann, hint_cb,
                   (const char* const*)hist_role, (const char* const*)hist_text, hist_n,
                   stp,
                   last_act_ms ? (unsigned)((esp_timer_get_time() / 1000 - last_act_ms) / 1000) : 0u,
                   task_note, task_s,
                   frame, frame_len, prev, use_prev ? prev_len : 0,
                   use_prev, use_edited_now, edited, edited_len);
        if (!body.ok) { fail = "组装请求 body 失败"; break; }

        String resp;
        bool http_ok = false;
        for (int nr = 0; nr < 3 && !http_ok; nr++) {   // 网络失败指数退避重试(任务串行, 代价可控)
          if (ai::http_post(cfg::ai_url().c_str(), cfg::ai_key().c_str(), body.p, resp, t.generation)) { http_ok = true; break; }
          if (t.generation != m_generation) { done = true; interrupted = true; break; }  // 被中止, 静默作废
          if (ai::http_last_status() >= 400 && ai::http_last_status() < 500) {  // 4xx 重发同 body 必然再拒, 快速失败
            fail = "云端拒绝(4xx), 疑似参数或限流";
            break;
          }
          if (nr < 2) { vTaskDelay(pdMS_TO_TICKS(500 << nr)); blog::logf(blog::AI, "网络失败重试 %d", nr + 1); }
        }
        if (done) break;
        if (!http_ok) { if (!fail) fail = "AI 请求失败"; break; }
        if (t.generation != m_generation) { done = true; interrupted = true; break; }  // 在途结果作废

        String content;
        bool body_broken = false;
        if (!extract_content(resp, content, &body_broken)) {
          // 解码成功但无有效内容(瞬态错误体/空 content 等): 打印原始片段便于定位
          blog::logf(blog::AI, "响应无内容, 原始(前120B): %s", resp.substring(0, 120).c_str());
          fail = "AI 响应无内容";
          if (body_broken) ai::http_stop();  // 传输层截断/残留: 弃用复用连接, 下次全新握手防污染
          continue;  // 空内容→重试, 不终止
        }

        JsonDocument cmdD(&g_js_alloc);  // PSRAM 池: 避免每轮在内部堆分配/释放制造碎片(freeHeap 泄漏嫌疑)
        const char* verr = validate_cmd(content.c_str(), cmdD, err_buf, sizeof(err_buf));
        if (!verr) {
          const char* type = cmdD["type"] | "";
          JsonObjectConst params = cmdD["params"].as<JsonObjectConst>();
          if (!strcmp(type, "approach")) {
            // 本地自动靠近: 不点云端, 直接按记忆目标巡航到近距, 交还 AI 继续微操。
            const char* tgt = params["target"] | "";
            float tx, ty;
            bool found = ai::mem_find(tgt[0] ? tgt : nullptr, &tx, &ty);
            const char* why = nullptr;
            bool arrived = false;
            if (!found) {
              why = tgt[0] ? "approach 目标不在记忆里, 请先 observe 锁定" : "approach 无可用目标记忆, 请先 observe";
            } else {
              // navigate_to 本身不产生任何日志(见其注释: 纯本地巡航), 于是"approach 后车停在哪、
              // 为什么停那儿"在日志里完全无迹可寻 —— 复盘"目标在前 12cm、爪却只到 10cm"时,
              // 唯一能定位停靠距离的就是这一行。停距决定了贴地夹取够不够得着, 必须可见。
              ai::logf("[ai] approach 目标 全局(%.0f,%.0f)cm 停距%dcm", tx, ty, AI_APPROACH_STOP_CM);
              NavR r = navigate_to(t.generation, tx, ty, AI_APPROACH_STOP_CM);
              if (r == NavR::Interrupted) { interrupted = true; done = true; break; }   // 被接管: 本任务作废
              arrived = (r == NavR::Reached);
              why = arrived ? "已自动靠近目标, 交还你微操" : "靠近收敛结束, 由你继续";
              last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
            }
            JsonDocument cmdF(&g_js_alloc);
            cmdF["type"] = "approach";
            cmdF["reason"] = why;
            cmdF["params"]["target"] = tgt[0] ? tgt : "(最近)";
            String fb = build_feedback(t.id, cmdF);
            enqueue_result(fb.c_str(), t.fn, t.ctx);
            stall = 0; stall_hint = false;    // 本地巡航视为有意推进, 不复位死循环判据
            since_release = false; pose_low = false;   // 挪了车 = 重新对准过(闸门的账销掉)
            got = true;
            net_fail = 0;
            break;
          }
          if (!strcmp(type, "zoom")) {
            // 本地处理(不点云端): 记下裁框, 下一帧起发给 AI 的就是那块放大图。
            // px/py 用**它刚看到那张图自己的坐标**(0~1), 由 sent_* 反算回全幅 —— 于是反复放大/挪动
            // 都按同一套账走, 不会一轮轮漂。reset 回全幅。
            bool zoom_changed = true;   // 这次放大是否真的改变了画面(见下面的"空操作要报")
            bool zoom_noop = false;     // 已在倍数上限、又要同一块: 画面与手上一字不差
            if (params["reset"] | false) {
              zoom_on = false;
              zoom_noop_n = 0; zoom_repeat[0] = 0;
              ai::logf("[ai] 放大镜关闭, 回全幅");
            } else {
              float fx = to_full_x(params["px"] | 0.5f), fy = to_full_y(params["py"] | 0.5f);
              float sc = params["scale"] | AI_ZOOM_DEF;
              if (sc < AI_ZOOM_MIN) sc = AI_ZOOM_MIN;
              if (sc > AI_ZOOM_MAX) sc = AI_ZOOM_MAX;
              set_zoom_box(fx, fy, sc);   // 参照当前视图定框, 并按 AI_ZOOM_MAX 给累积倍数封顶
              zoom_on = true;
              // 与**当前视图**比: 框一样, 且这期间车与臂都没动过, 才是真复读(见 AI_ZOOM_NOOP_MAX 处)。
              zoom_changed = (zoom_x0 != sent_x0 || zoom_x1 != sent_x1 ||
                              zoom_y0 != sent_y0 || zoom_y1 != sent_y1);
              float zp_x = 0, zp_h = 0; exec::arm_pos(&zp_x, &zp_h);
              char zp_now[48];
              snprintf(zp_now, sizeof(zp_now), "%.1f,%.1f,%d,%.1f,%.1f",
                       (double)ai::s_car_x, (double)ai::s_car_y, (int)ai::s_car_heading,
                       (double)zp_x, (double)zp_h);
              bool same_place = !strcmp(zp_now, zoom_pose);
              snprintf(zoom_pose, sizeof(zoom_pose), "%s", zp_now);
              ai::logf("[ai] 放大镜 %.1f× 中心(全幅%.2f,%.2f) 框(%.2f,%.2f)-(%.2f,%.2f)%s%s",
                       sc, fx, fy, zoom_x0, zoom_y0, zoom_x1, zoom_y1,
                       zoom_changed ? "" : " 无变化", same_place ? "" : " 位姿已变");
              // 空操作(它要的正是它手上这张图、且这期间什么都没动)不能"说一声"就当进展 —— 但它更
              // 不该**改变画面**: 这里曾经"替它退回全幅", 那是针对旧语义(scale 累乘: 连发三次 2× 会
              // 变成 8×、夹爪被裁出画面)打的补丁。改成绝对倍数之后, 重复请求是**可满足的幂等请求**
              // (它想接着看那一块), 退回全幅反而把刚要到手的近景毁掉。但只讲道理拦不住复读(实测同一条
              // 连发 31 次), 所以连要 AI_ZOOM_NOOP_MAX 次以上就改回全幅 —— 那时车与臂都没动过, 画面
              // 一字未变, 换个视野至少给了新信息, 也把"同一输入→同一请求"这个环打断。
              if (!zoom_changed && same_place) {
                zoom_noop = true;
                zoom_noop_n++;
                if (zoom_noop_n > AI_ZOOM_NOOP_MAX) {
                  zoom_on = false;   // 改回全幅(裁框复位, 上面 zoom_warn 那条路径照旧可用)
                  snprintf(zoom_repeat, sizeof(zoom_repeat),
                           "同一块画面已连要 %d 次, 而这期间车和臂都没动过 —— 画面一字未变, 再要也不会有"
                           "新信息。已改回全幅: 下一步必须是实体动作(move/arm/spin/wait), 或改 px/py 看别处",
                           zoom_noop_n);
                  ai::logf("[ai] 放大镜复读 %d 次(车臂未动), 强制回全幅", zoom_noop_n);
                  zoom_noop_n = 0;
                } else {
                  snprintf(zoom_repeat, sizeof(zoom_repeat),
                           "这张图就是你手上那张(同倍数同位置, 第%d次重复请求), 画面不会变, 别再发同一条 zoom; "
                           "下一步给实体动作(move/arm/spin/wait)或改 px/py/scale, 回全幅填 reset:true",
                           zoom_noop_n);
                  ai::logf("[ai] 放大镜重复请求(画面已是这块), 保持不动");
                }
              } else {
                zoom_noop_n = 0; zoom_repeat[0] = 0;   // 画面真变了(或车/臂动过): 重复的账销掉
              }
            }
            JsonDocument cmdF(&g_js_alloc);
            cmdF["type"] = "zoom";
            // 只剩两支: 非 reset 时 zoom_noop == !zoom_changed, 原先那支"已到最大放大倍数"永远走不到。
            cmdF["reason"] = zoom_noop ? (zoom_on ? "画面已经是这块(保持不动, 别重复发)"
                                                  : "同一块连要多次且车臂未动, 已改回全幅")
                           : (!zoom_on ? "已回全幅"
                                       : "已放大: 下一帧起画面只剩目标附近这一块(px/py 照旧按 0~1 填)");
            cmdF["params"]["on"] = zoom_on;
            String fb = build_feedback(t.id, cmdF);
            enqueue_result(fb.c_str(), t.fn, t.ctx);
            // 本地命令也要进历史环。原先 zoom 在这里 break, 走不到下面 exec 分支的历史写入 ——
            // 于是喂回模型的历史里**没有"我已经放过 zoom"**, 下一轮 prompt 与上一轮一字不差,
            // 它必然再放一条同样的 zoom(实测空转 28 轮 / 0 动作)。历史里的形态与 exec 分支一致
            // (它自己的决策 + 自己的 reason), 免得把板子的回执文案喂回去教它复读。
            {
              const char* r = cmdD["reason"] | "";
              PsaBuf ld;
              ld.put(zoom_on ? "zoom 放大镜" : "zoom 回全幅"); ld.put(", 原因: "); ld.put(r);
              utf8_clamp_tail(ld.p);
              if (ld.len) hist_add("assistant", ld.p);
            }
            // 换视角是有意推进, 与 wait 同理, 不算"无进展" —— 但空操作(要一张手上就有的图)不算:
            // 它什么都没换来, 别把死循环/空转的账销掉。
            if (!zoom_noop) { stall = 0; stall_hint = false; }
            got = true;
            net_fail = 0;
            break;
          }
          if (!strcmp(type, "wait")) {
            // wait=空操作: 不下发执行板, 保持当前动作, 任务继续观察。
            // 视为有意进展: 复位死循环计数, 避免"等待"被当成无进展注入引导。
            stall = 0; stall_hint = false;
            uint64_t now = (uint64_t)(esp_timer_get_time() / 1000);
            if (now - g_last_wait_fb_ms >= AI_WAIT_FB_MIN_MS) {  // 反馈节流, 防每轮刷屏
              g_last_wait_fb_ms = now;
              String fb = build_feedback(t.id, cmdD);
              enqueue_result(fb.c_str(), t.fn, t.ctx);
            }
            got = true;
            break;
          }
          // move 未给 distance_cm 时补一个有限段长(见 AI_MOVE_DEFAULT_CM): 否则执行层按"持续"处理,
          // 只能靠 AI_MOVE_CAP_MS 盲走, 且 car_update_pose 不累积位移 → 记忆里的车位置从此失同步,
          // 后续喂回的物体坐标整体偏移(表现为"明明记过却在错的地方找")。补在**执行前**, 让执行、
          // 姿态累积、状态文本三者看到的是同一个距离。补完 params 重新指向同一对象。
          if (!strcmp(type, "move") && !params["distance_cm"].is<int>() &&
              fabsf(params["throttle"] | 0.0f) > 0.001f) {
            cmdD["params"]["distance_cm"] = AI_MOVE_DEFAULT_CM;
            params = cmdD["params"].as<JsonObjectConst>();
            ai::logf("[ai] move 未给 distance_cm, 补为 %dcm", AI_MOVE_DEFAULT_CM);
          }
          // 近场步幅钳制(见 AI_NEAR_FWD_CM): 目标已经在前方十几厘米内时, 前进一步最多 2cm。
          // 只钳前进 —— 后退是脱离死角的方向, 拦它只会把车锁在"够不着又退不出"的位姿里。
          if (!strcmp(type, "move") && (params["throttle"] | 0.0f) > 0.001f) {
            char nname[16] = {0}; int nage = 0;
            float nfwd = 0, nlat = 0, nmv = 0, ntn = 0;
            if (ai::mem_grasp_evidence(nname, sizeof(nname), &nage, &nfwd, &nlat, &nmv, &ntn) &&
                nfwd > 0 && nfwd <= AI_NEAR_FWD_CM) {
              int want = params["distance_cm"] | 0;
              if (want > AI_NEAR_STEP_CM) {
                cmdD["params"]["distance_cm"] = AI_NEAR_STEP_CM;
                params = cmdD["params"].as<JsonObjectConst>();
                ai::logf("[ai] 目标已在前%.0fcm(近场), 前进步长 %dcm → %dcm", nfwd, want,
                         AI_NEAR_STEP_CM);
                if (!near_warn[0])
                  snprintf(near_warn, sizeof(near_warn),
                           "目标已在前 %.0fcm(近场): 这次前进已限为 %dcm。近处每 1cm 都决定夹得住夹不住, "
                           "别再大步顶(几轮就能把方块压进车底死角, 那里既看不见也够不着)。"
                           "改用 zoom 放大看目标是否已进两指之间, 进了就 arm clip",
                           nfwd, AI_NEAR_STEP_CM);
              }
            }
          }
          // "低姿转身会拨动方块"告警(通用, 不针对某个目标): 爪贴地时原地转, 两指会横扫过爪口下方
          // 那一带 —— 方块正好在那儿时会被推到爪口外侧(实测两次空夹的共同前兆都是贴地后才动)。
          // 这是**物理**后果, 画面里看不出来(转的时候画面本来就糊), 只能由程序按真实臂位提醒。
          // 只告警不拦: 低姿下的横向微调**正是**新范式里该做的事(靠车把方块送进两指之间), 拦下来就没
          // 法对准了。要说的只是"幅度小、转完重看、方块被拨走就重新 observe"。
          // 每轮重推(与 near_blind 同): 只要臂还在低姿, 这一轮就再说一次。
          if (!strcmp(type, "spin") && (params["dir"] | 0) != 0) {
            float ax = 0, ah = 0;
            if (exec::arm_pos(&ax, &ah) && ah <= AI_ARM_LOW_H_CM) {
              snprintf(arm_low_warn, sizeof(arm_low_warn),
                       "爪正贴着地(车前%.0fcm/高%.0fcm): 这样转向, 两指会横扫过爪口下方把方块拨出去。"
                       "低姿微调就靠它, 但**幅度要小**(几度一级), 转完立刻重看画面 zoom 确认方块还在不在两指之间; "
                       "被拨走了就重新 observe、重走一遍对位",
                       ax, ah);
              ai::logf("[ai] 低姿转身告警(爪高%.0fcm)", ah);
            } else {
              arm_low_warn[0] = 0;
            }
          } else {
            arm_low_warn[0] = 0;   // 只在这一条 spin 上说话, 别的回合不带
          }
          // spin 同理: 未给 angle_deg 的持续旋转不累积车向 → 记忆换算基准失同步(见 AI_SPIN_DEFAULT_DEG)。
          if (!strcmp(type, "spin") && !params["angle_deg"].is<int>() &&
              (params["dir"] | 0) != 0) {
            cmdD["params"]["angle_deg"] = AI_SPIN_DEFAULT_DEG;
            params = cmdD["params"].as<JsonObjectConst>();
            ai::logf("[ai] spin 未给 angle_deg, 补为 %d°", AI_SPIN_DEFAULT_DEG);
          }
          // 档位钳制(见 AI_SPIN_TRACK_MAX_DEG): 记忆里有新鲜观测 ⇒ 目标刚在画面里, 这是"对准"档,
          // 单步收紧到 15°。放宽的条件是"真没有新鲜观测"(那才是搜索, 按规则的 60°/步放行)。
          // 排在下面累计预算之前: 让预算账本记的是**实际要转的**角度, 不虚耗余额。
          if (!strcmp(type, "spin") && (params["dir"] | 0) != 0 &&
              (params["angle_deg"] | 0) > AI_SPIN_TRACK_MAX_DEG) {
            char tnm[16] = {0};
            int tage = 0; float tf = 0, tl = 0, tmv = 0, ttd = 0;
            if (ai::mem_grasp_evidence(tnm, sizeof(tnm), &tage, &tf, &tl, &tmv, &ttd)) {
              // 近场再收紧: 目标已在前方十几厘米内时, 单步 ≤ AI_SPIN_NEAR_MAX_DEG —— 前距 10cm 转 6°
              // 横向约 1cm, 还在方块宽度内, 一步不会跳过; 前方没目标(仍在搜索)才回到 15° 档。
              int cap = (tf > 0 && tf <= AI_NEAR_FWD_CM) ? AI_SPIN_NEAR_MAX_DEG : AI_SPIN_TRACK_MAX_DEG;
              spin_align_now = true;   // 有新鲜观测 ⇒ 本次是对准, 见下面预算分账
              ai::logf("[ai] spin %d° 限为 %d°(「%s」%d轮内刚见过%s ⇒ 这是对准不是搜索; 转大了会把它甩出视野)",
                       params["angle_deg"] | 0, cap, tnm, tage,
                       (cap == AI_SPIN_NEAR_MAX_DEG) ? " 且已近场" : "");
              cmdD["params"]["angle_deg"] = cap;
              params = cmdD["params"].as<JsonObjectConst>();
            }
          }
          // 本任务累计旋转预算: 单步有硬上限(AI_SPIN_MAX_DEG), 但连发多步就能转好几圈 —— 车尾还挂着
          // 充电线(缠住就走不动), 且大幅盲转会**同时**丢掉画面参照与朝向基准(见 AI_SPIN_MAX_DEG 注释)。
          // 到顶不再放行: 把这次 spin 的 dir 改成 0(= 停, 一点都不转), 并明确要它回画面定位。
          // 用 dir=0 而不是跳过执行: 语义仍是"spin = 停下", 状态行/姿态累积都自洽。
          if (!strcmp(type, "spin") && (params["dir"] | 0) != 0) {
            int want = params["angle_deg"] | 0;
            // "对准"与"搜索"分账(见 AI_SPIN_ALIGN_BUDGET_DEG): 判据是**记忆里有没有新鲜观测** ——
            // 目标刚在画面里, 这次转就是在对准(它自己看得见结果), 走独立额度、不占搜索预算; 没有观测
            // 的才是在盲搜, 吃 540° 的搜索预算。
            // ⚠️ 这条曾经挂在"闸门点名要求的那次转向"上(align_need_dir/deg/ttl)。那套闸门拒绝已经去掉
            // (它是死亡螺旋的源头), 判据随之改为按观测新鲜度判 —— 否则闸门不再点名后, 对准转向会全部
            // 落进搜索预算, 预算一满就被降级成停, 又回到"程序叫它转、程序不让它转"的同源违规。
            bool is_align = spin_align_now && want <= AI_SPIN_TRACK_MAX_DEG + AI_SPIN_ALIGN_SLACK_DEG &&
                            spin_align_deg < AI_SPIN_ALIGN_BUDGET_DEG;
            if (is_align) {
              spin_align_deg += want;
              ai::logf("[ai] 对准转向 %d°(不占搜索预算; 对准额度 %d°/%d°, 累计搜索转 %d°)",
                       want, spin_align_deg, AI_SPIN_ALIGN_BUDGET_DEG, spin_total_deg);
            } else {
              int left = AI_SPIN_TASK_BUDGET_DEG - spin_total_deg;
              if (left <= 0) {
                cmdD["params"]["dir"] = 0;   // 降级为"停": 这一次不转
                params = cmdD["params"].as<JsonObjectConst>();
                ai::logf("[ai] 旋转预算用满(累计%d°/%d°) → 本次 spin 降级为停", spin_total_deg,
                         AI_SPIN_TASK_BUDGET_DEG);
                if (!spin_warn[0])
                  snprintf(spin_warn, sizeof(spin_warn),
                           "旋转预算已用满(本任务累计转了%d°): 这次 spin 已改为停下。别再盲转 —— 大幅旋转会把画面参照和朝向基准一起弄丢(转了却以为没转), 改用 arm fold 收臂后看画面/observe 重新定位(注: 对准转向——手上有本轮新鲜观测的那种——走独立额度, 不受此限)",
                           spin_total_deg);
              } else {
                if (want > left) {
                  cmdD["params"]["angle_deg"] = left;   // 只给余额, 别一次超支
                  params = cmdD["params"].as<JsonObjectConst>();
                  ai::logf("[ai] spin %d° 超出旋转预算余额, 限为 %d°(累计%d°/%d°)", want, left,
                           spin_total_deg, AI_SPIN_TASK_BUDGET_DEG);
                }
                spin_total_deg += params["angle_deg"] | 0;
              }
            }
          }
          // 空间记忆: 先解析 AI observe(用**动作前**的车位姿把观测转全局), 再按 move/spin 更新车姿态。
          // 必须排在闸门与 exec::act 之前: 动作被闸门拒掉时, 这一轮的观测同样要收下。
          bool obs_visible = false;   // 本轮 AI 报"看得见且记成了"→ 夹取依据/落位复看都算销账
          char obs_vis_name[16] = {0};   // 记成的那条是谁 —— 闸门放行时要报"依据是本轮亲眼看的吗"
          if (cmdD["observe"].is<JsonObject>()) {
            JsonObjectConst ob = cmdD["observe"].as<JsonObjectConst>();
            const char* nm = ob["name"] | "";
            bool vis = ob["visible"] | true;
            if (!nm[0]) {
              // name 必填: 无名观测无法归属到任何物体, 只能丢弃。明确回告(否则 AI 以为已记住)
              if (vis) snprintf(obs_warn, sizeof(obs_warn),
                                "上轮 observe 没给 name, 该观测已被忽略(记忆里没有它); 每次 observe 都要带 name");
            } else {
              // 优先用屏幕像素 px/py(单应解算, 精度高); 越界/解算失败或没报像素则回退 rel_deg/dist。
              float px = ob["px"] | 0.0f, py = ob["py"] | 0.0f;
              // 本轮 AI 看的是放大图时, 它的 px/py 是相对那块裁框的: 先换回全幅再交给单应。
              // 这一步是精确算术(裁框是程序自己记的账) —— 放大镜的全部价值就在这儿: 它只需在局部
              // 里指认目标, 而不必在全幅里估绝对位置(那正是它一直做不好的事)。
              if (sent_zoomed) {
                float lx = px, ly = py;
                px = to_full_x(px); py = to_full_y(py);
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
                // 没给像素 = 位置只能记模型**嘴里估的**厘米/角度。模型没有测距能力, 它自估的数一律
                // 偏小(实测同一个方块: 自估 4~5cm, 而画面单应解算是十来厘米), 照它对准会把车往错的
                // 方向指挥。记还是照记(approach/goto 用粗值够用), 但必须把话说清并要回像素指认 ——
                // 指认是模型做得好的那件事(它看得见), 估 cm 是它做不好的那件事。
                ai::logf("[ai] 观测「%s」未给 px/py → 只能记模型自估 rel=%.0f° dist=%.0fcm(非实测)",
                         nm, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
                if (vis && !obs_warn[0])
                  snprintf(obs_warn, sizeof(obs_warn),
                           "上轮 observe 没给 px/py: 位置只能按你自报的厘米记(自估不准); 请用 px/py 指认画面里的目标再对准");
                rec = ai::mem_observe(nm, vis, ob["rel_deg"] | 0.0f, ob["dist_cm"] | 0.0f);
              }
              if (vis && !rec)   // 可见却没记成: 回告, 否则 AI 以为已锁定而实际记忆为空
                snprintf(obs_warn, sizeof(obs_warn),
                         "上轮 observe 的「%s」没记进记忆(px/py 越界且缺可用距离), 位置仍未知; 请据当前画面重新确认后再 observe",
                         nm);
              obs_visible = (vis && rec);
              if (obs_visible) { strncpy(obs_vis_name, nm, sizeof(obs_vis_name) - 1); obs_vis_name[sizeof(obs_vis_name) - 1] = 0; }
            }
          }
          // ---------------- 夹取闸门 ----------------
          // 病根: arm_pose/arm clip 的入参只有 x/h 两个数, AI 完全可以拿记忆里的旧坐标"算"出一个
          // 位姿直接合爪 —— 中间没有任何一步需要它真的看画面(实测它就是这么干的: 记忆行明明写着
          // "已4~9轮未见", 照夹不误, 夹空 → 抬臂验证没夹住 → release → 原样再夹一遍)。
          // 这里把"必须先看到"变成程序约束: 下探去夹(h≤AI_GRASP_H_CM 且夹爪是张开的)与合爪, 都要
          // 有一条近期画面依据(mem_grasp_evidence: AI_GRASP_STALE 轮内看到过且在车前)。
          // 被拒不执行原动作, 改为自动收臂让出视野 + 把原因喂回下一轮 —— 拒绝是为了让它重新对准,
          // 不是终点, 所以绝不能拦收臂/张开/抬落这些回撤动作(否则会把它关在一个进不去也退不出的位姿)。
          const char* act = params["act"] | "";
          bool jaw_open = !exec::grip_closing();   // 合着爪下探 = 去放东西(不是去夹), 不拦
          bool is_clip = !strcmp(type, "arm") && (!strcmp(act, "clip") || !strcmp(act, "grasp"));
          bool is_low_pose = (!strcmp(type, "arm_pose") && (params["h"] | 0.0f) <= AI_GRASP_H_CM) ||
                             (!strcmp(type, "arm") && !strcmp(act, "low"));
          bool gwhy_hit = false;
          char gwhy[224] = {0};
          // 只告警不拦截的那类判据(见下面"细判据"分支): 动作照常执行, 这句作为下轮引导语喂回。
          char gadv[224] = {0};
          if (is_clip || (is_low_pose && jaw_open)) {
            char gname[16] = {0}; int gage = 0; float gfwd = 0, glat = 0;
            const char* what = !strcmp(type, "arm") && !strcmp(act, "grasp") ? "arm grasp"
                               : is_clip      ? "arm clip"
                               : !strcmp(type, "arm") ? "arm low"
                                                      : "arm_pose 下探";
            // 夹爪实际停在哪(正运动学回读, 与状态行同源): 合爪前要拿它跟目标前距比 ——
            // 只判"目标在车前 0~22cm 内"是不够的, 差 1.5cm 以上就已经夹不到方块了(方块才 2~3cm 宽)。
            float cx = 0, ch = 0;
            bool has_pos = exec::arm_pos(&cx, &ch);
            // 这次动作会把夹心送到车前多少 cm: arm low 落点是标定的固定值(AI 管不了它), clip 就是爪
            // 当前实际位置。**不判 arm_pose 下探**: 它的 x 是 AI 照记忆坐标填的, 而记忆喂回用的是融合
            // 值、闸门用的是最新观测, 两者本来就可能差几厘米 —— 在那儿比会拒掉一个它照做无误的动作。
            float x_tgt = 0;
            bool has_x_tgt = false;
            if (!strcmp(type, "arm") && !strcmp(act, "low")) { x_tgt = ARM_LOW_X_CM; has_x_tgt = true; }
            else if (is_clip && has_pos) { x_tgt = cx; has_x_tgt = true; }
            float gmove_cm = 0, gturn_deg = 0;
            // 半空合爪拦截(实测: AI 在悬停位 h=5 直接 clip, 爪口离地 5cm, 贴地的方块根本不在爪口里
            // → 必空夹, 抬臂验证才发现)。clip/grasp 本身不携带高度, 但爪口必须先降到贴地低姿
            // (AI_GRASP_H_CM)才能套住方块 —— 超过说明爪还悬在半空。这是新范式里唯一一条**必须先做**的
            // 动作顺序(②降爪 在 ⑤夹取 之前), 所以照旧拦。⚠️ 离地目标(堆叠场景)会被此判据误伤, 目前
            // 任务是贴地方块, 以它为准; 真正支持离地夹取时须把"目标高度"传进闸门而不是用固定线。
            if (is_clip && has_pos && ch > AI_GRASP_H_CM) {
              snprintf(gwhy, sizeof(gwhy),
                       "%s 被拒: 爪口还悬在半空(离地高%.0fcm), 没降下来就合爪 —— "
                       "贴地的方块在爪口下方好几厘米, 合上空爪必空夹。先 arm low 把爪降到贴地低姿"
                       "(之后高度就不用再管), 再小步 move 把方块送进两指之间, 然后 %s",
                       what, ch, what);
              gwhy_hit = true;
            } else if (!ai::mem_grasp_evidence(gname, sizeof(gname), &gage, &gfwd, &glat, &gmove_cm, &gturn_deg)) {
              // 依据缺失: 记忆是空的, 或最近一次观测已过期。这种坐标下夹=盲夹。
              snprintf(gwhy, sizeof(gwhy),
                       "%s 被拒: 记忆里没有近%d轮内被画面看到过的目标, 拿旧坐标去夹必然空夹。先收臂让出视野, 用 observe 记下目标当前实际位置再对准",
                       what, ai::AI_GRASP_STALE);
              gwhy_hit = true;
            } else if (gmove_cm > ai::AI_GRASP_MOVE_TOL_CM || gturn_deg > ai::AI_GRASP_TURN_TOL_DEG) {
              // "用坐标代替画面"的主路径, 也是实测最后一次空夹前的状态: 目标那次看到时在画面中央, 之后
              // move 了 5cm, 记忆里仍旧报"居中", 而同一次观测的画面里它已经偏到左边约 9cm —— 爪不能
              // 左右移动, 这一轴错了必空夹, 且合爪时不会报任何错。这类坐标已经不是"看到的位置", 而是
              // "看到的位置 + 一段按标定时长推算的位移"; 无里程计谁也算不准这段位移, 只能重新看。
              // 文案控制在 grasp_warn[224] 之内(超了截断丢的正好是末尾那句怎么改)。
              snprintf(gwhy, sizeof(gwhy),
                       "%s 被拒: 依据是车挪了%.0fcm/转%.0f°之前看到的, 那段位移只是推算(无里程计), 已与当前画面对不上。请用本帧 observe(px/py)并和该动作写进同一次回复",
                       what, gmove_cm, gturn_deg);
              gwhy_hit = true;
            } else if (low_push_ttl > 0) {
              // 低姿前顶之后的专用配方(见 AI_LOW_PUSH_TTL): 这里**不按坐标点名前后方向**。
              // 操作者实测的机械经验: 爪子顶得动方块之后, 要**稍微退后约 1cm** 才夹得住 —— 顶着的时候
              // 两指被方块撑住、使不上劲, 退开一点才有闭合行程去咬住它。这条程序与模型都推不出来,
              // 只能由人给。而顶完之后的坐标也不再代表"方块在哪"(被推动了 + 被爪面挡住), 实测同一
              // 方块的前距会在 4~28cm 之间乱跳 —— 按它点名方向就是实测那个"顶→报远→再顶"的循环。
              float elat2 = glat - ARM_LOW_LAT_CM;
              if (fabsf(elat2) > ai::AI_GRASP_LAT_MAX) {
                snprintf(gadv, sizeof(gadv),
                         "%s 已放行: 刚在低姿顶过方块(就在爪口前, 顶完坐标不可信), 别按它再往前顶。"
                         "先后退约1cm让两指脱开, 同时朝%s小幅 spin 修横向, 再 zoom 确认就夹",
                         what, elat2 >= 0 ? "右" : "左");
              } else {
                snprintf(gadv, sizeof(gadv),
                         "%s 已放行: 刚在低姿顶过方块(就在爪口前, 顶完坐标不可信), 别按它再往前顶。"
                         "先后退约1cm让两指脱开(顶着夹不紧), 再 zoom 确认就 arm grasp",
                         what);
              }
            } else if (is_clip && fabsf(glat - ARM_LOW_LAT_CM) > ai::AI_GRASP_LAT_HARD_CM) {
              // 横向**硬**闸门(与下面那条提醒线的分工见 ai_mem.h 的 AI_GRASP_LAT_HARD_CM): 提醒线只管
              // "说一句"、动作照做; 这一条是"物理上夹不到", 合爪一律拦下。依据(实测): 那次空夹的记忆读数
              // 是"左5.1cm", 减掉爪口自身偏置 ARM_LOW_LAT_CM(1.0) ⇒ 目标偏出爪口 6.1cm, 是方块身位
              // (2.5cm)的两倍多, 却从 1.5cm 的提醒线里放行 —— 合上空爪、**不报任何错**, 只能靠下一轮抬臂
              // 复看才发现, 白烧一轮还把方块压走(随后连轮不可见 ⇒ 跟丢)。
              // ⚠️ 位置刻意排在 low_push_ttl 那条**之后**(而不是最前): 低姿前顶之后程序自己就对 AI 声明
              // "坐标不再代表方块在哪"(顶完的读数实测在 4~28cm 之间乱跳)。拿一个程序刚说过不可信的坐标去
              // 硬拦, 既违反同源不变量, 又会在配方(退约1cm再夹)执行到一半时强行收臂 —— 把唯一能成的那条
              // 路堵死。所以硬拦只在"坐标还算数"的窗口里生效; 顶过的窗口仍交给上面那条配方。
              // ⚠️ 补救动作与提示词的小角度上限**同源**: 只说"最小步 spin + 转完看画面", **不照 atan2
              // 点名大角度** —— 旧闸门点名 47°/56° 与提示词"≤15°"打架, 是跟丢的直接成因(见 CLAUDE.md 4)。
              float elat_h = glat - ARM_LOW_LAT_CM;
              snprintf(gwhy, sizeof(gwhy),
                       "%s 被拒: 目标偏出爪口%s%.1fcm, 超一个方块身位, 这样合爪必空夹且不报错。"
                       "夹爪不能左右移动, 用最小步 spin 朝%s转一点, 转完 zoom 看方块进两指之间没有",
                       what, elat_h >= 0 ? "右" : "左", fabsf(elat_h), elat_h >= 0 ? "右" : "左");
              gwhy_hit = true;
            } else if (fabsf(glat - ARM_LOW_LAT_CM) > ai::AI_GRASP_LAT_MAX) {
              // 横向是机械臂**唯一无法补偿**的一轴(只有前后+抬落两个自由度), 差一点就必空夹, 而合爪时
              // 不会报任何错 —— 所以这条要说得早、说得具体。
              // 判据要**减掉爪口自身的横向偏置** ARM_LOW_LAT_CM: 爪口不在车头轴线上, 拿 |glat| 判等于
              // 把"爪口偏右"读成"目标偏右"。
              // ⚠️ 这条**曾经是拒绝**, 而且拒绝里给的动作是"先 arm_pose 抬回悬停位(h≈目标高度+3), 再朝
              // X 侧 spin 约 N°" —— 那正是把 AI 赶出低姿、逼它走"悬停对准→垂直下探"这条已被判定为错的
              // 主线, 而低姿下的横向微调本来就该靠车身转动完成(新范式的第③步)。**降级为放行+提醒**:
              // 坐标本身是厘米级解算值, 拿它一票否决会拒掉其实夹得住的动作, 而实测的死亡螺旋正是
              // "闸门按坐标点名要大转向 → 与提示词的小角度上限打架 → 转过头 → 方块被扫出视野跟丢"。
              // 现在只说"偏哪边、大概几度、转完自己看画面确认"。
              // ⚠️ 措辞**必须压在 grasp_warn[224] 之内**(经 hpush 进 hint_cb)。本条原先 311 字节、被
              // snprintf 截断, 丢掉的正好是末尾的补救动作 —— 实测值, 别再写长(超了等于白说)。
              float elat = glat - ARM_LOW_LAT_CM;
              int   edeg = (int)lroundf(atan2f(fabsf(elat), gfwd > 1.0f ? gfwd : 1.0f) * 180.0f / ai::AI_PI);
              if (edeg < 2) edeg = 2;
              if (edeg > 20) edeg = 20;   // 只是提醒幅度: 低姿下大角度转向会横扫方块, 宁小勿大
              // 目标在爪口右侧 ⇒ 要把目标相对车头挪向左 ⇒ 车右转(dir=+1, 见 spin 的 dir 定义)
              snprintf(gadv, sizeof(gadv),
                       "%s 已放行, 但横向没对准: 目标在爪口%s约%.1fcm(车头系 前%.0fcm)。夹爪不能左右移动, "
                       "微调靠小角度 spin 朝%s转约%d°(一次别多转), 转完立刻 zoom 看方块进两指之间没有",
                       what, elat >= 0 ? "右" : "左", fabsf(elat), gfwd, elat >= 0 ? "右" : "左", edeg);
            } else if (gmove_cm > 0.05f || gturn_deg > 0.5f) {
              // 车动过, 但只挪了容差以内的一点点: 那条坐标仍是"看到的位置", 推算误差小于夹取容差。
              // 这类位移正是**配方里的那一小步** —— 悬停对准之后补一小步(≤2cm)把目标挪进两指正下方,
              // 而那一小步必然让车位姿变化、且目标这时已被夹爪挡住**没法重新 observe**。
              // 判"车动过就拒"等于把配方里最关键的一步永久拒之门外: 实测 AI 因此卡在"目标太近→看不见
              // →后退找画面→又变远"的极限环里, 整轮 90s 一次都没走到对准(旧范式的 arm low)。
              // 同理, "看不见"本身不是问题: 近处被自身夹爪/车体挡住是常态(提示词的规则 8(d) 也这么写),
              // 该夹就夹 —— 夹空了抬臂复看能立刻发现, 比在盲区外反复打转便宜得多。
              // ⚠️ 这条曾经写着"依据仍然有效, 照这个坐标夹就行" —— 实测它把 AI 推去**照坐标合爪**:
              // 差 1.3cm 就合, 而前场坐标有 1~2cm 系统偏差(画面里方块还在指尖前方两三厘米), 连夹两次
              // 都空。容差只说明"坐标没矛盾", 不是对准依据; 对准只能看画面(见提示词规则 8(d))。
              snprintf(gadv, sizeof(gadv),
                       "%s 已放行(容差内≠对准): 看不见多半是爪挡住(正常), 别为看它而后退; "
                       "合爪前用 zoom 确认方块已到两指之间, 还在指尖前方就回悬停位小步前顶",
                       what);
            } else if (gfwd < ai::AI_GRASP_FWD_MIN || gfwd > ai::AI_GRASP_FWD_MAX) {
              // 横向已在上一条判过, 这里只剩"够不着": 与"没看见"/"没对准"分开说 —— 否则 AI 会以为
              // 是自己没记, 反复 observe 再试同一动作, 而真正该做的是先把车推近(它并不会自己想到)。
              // 文案必须压在 grasp_warn[224] 之内: 经 hpush 进 hint_cb 有容量, 超了截断丢的正好是
              // 末尾那句"该怎么改"。
              // 新范式的"可夹带"不再是固定的贴地爪口: 悬停位(h=目标高度+3)能到的 x 带更宽(约 8~10.5cm
              // 落到贴地时要收窄), 文案给量级即可, 让 AI 用 move 把目标送到爪口附近再下探。
              snprintf(gwhy, sizeof(gwhy),
                       "%s 被拒: 够不着 —— 目标 %s 在车头系 前%.0fcm, 而爪口当前位置在车前约%.0fcm。用 move 把它送到车前约%.0fcm 附近(悬停位对准后)再下探",
                       what, gname, gfwd, (double)ARM_LOW_X_CM, (double)ARM_LOW_X_CM);
              gwhy_hit = true;
            } else if (has_x_tgt && fabsf(x_tgt - gfwd * AI_NEAR_SCALE) > AI_NEAR_DEADBAND_CM) {
              // 依据新鲜、也在车前, 但按**坐标**看夹心不在目标身上(目标 2~3cm 宽, 差出半宽就夹不住)。
              // 这条只告警、不拦截 —— 判据本身依赖厘米坐标, 那个数带厘米级误差(标定换算 + 模型自估,
              // 实测同一物体被估成 4cm 而画面解算是十几厘米), 拿它一票否决等于让"其实夹得住"的动作
              // 永远进不来(见 ai_mem.h 的同源不变量)。真正能一票否决的是三条与坐标精度无关的判据:
              // 本轮没看过画面 / 车动过之后再没看 / 够不着 —— 它们才拦得住"用坐标代替画面"。
              // 如实把差的这几厘米告诉 AI, 并把它推回画面判据(规则8(c): 小步前进到卡在两指之间),
              // 因为它看得见画面, 而程序看不见。
              // 差的算法与"前进/后退"的方向都必须用**同一尺度**: x_tgt 是 FK 真值, gfwd 是单应解算
              // 的高报值, 直接相减(以及拿原始值比大小定方向)会把方向指反(见 AI_NEAR_SCALE)。
              // ⚠️ 方向的写法也别照直觉抄: 爪口是**固定在车前 x_tgt** 的, 目标比它远(gfwd>x_tgt)才要
              // 前进把它拉近; 目标比它近(gfwd<x_tgt)是**已过冲**, 必须后退。这里曾经写成
              // `x_tgt > gfwd ? "前进" : "后退"` —— 正好反了, 于是闸门在 AI 已经顶过头的每一轮都
              // 叫它"再前进", 越顶越深、合爪必空(实测: 真距7cm 而爪口9cm 时仍报"前进")。
              float gfwd_true = gfwd * AI_NEAR_SCALE;
              snprintf(gadv, sizeof(gadv),
                       "%s 已放行, 但坐标说夹心落车前%.0fcm、%s在%.0fcm(高报⇒真距约%.0fcm, 差%.1fcm)。"
                       "以画面为准: zoom 看目标是否在两指之间, 不在就小步 move %s",
                       what, x_tgt, gname, gfwd, gfwd_true, fabsf(x_tgt - gfwd_true),
                       gfwd_true > x_tgt ? "前进" : "后退");
            } else if (has_x_tgt && fabsf(x_tgt - gfwd * AI_NEAR_SCALE) > AI_GRASP_X_TOL_CM) {
              // 差得**没超过噪声带** ⇒ 不点名方向(见 AI_NEAR_DEADBAND_CM)。旧写法在这里也会说
              // "小步 move 前进/后退", 而实测同一个方块(车与臂都没动)的前距就能报出差好几厘米 ——
              // 等于拿噪声给 AI 下左右脚的命令, 这就是那个前后极限环。带内改为把它推回画面判据。
              snprintf(gadv, sizeof(gadv),
                       "%s 已放行: 差%.1fcm, 还在近场坐标的噪声带内(实测前距会乱跳好几厘米), 据此挪车只会"
                       "来回顶。以画面为准: zoom 看方块在不在两指之间, 在就夹, 不在再小步挪一次",
                       what, fabsf(x_tgt - gfwd * AI_NEAR_SCALE));
            } else if (is_clip && since_release) {
              snprintf(gwhy, sizeof(gwhy),
                       "arm clip 被拒: 上一步 arm release 之后夹爪位姿与车位都没变过, 原样再夹一次结果不会有任何不同(依据: %s 近%d轮 前%.0fcm)",
                       gname, gage, gfwd);
              gwhy_hit = true;
            } else {
              // 放行也记一行: 这一夹的依据到底是"本轮亲眼看的"还是"沿用几轮前的坐标", 以及夹心与目标
              // 差多少 —— 这是复盘"为什么夹空"的唯一线索(此前只有坐标, 夹空时无法区分"没对准"和
              // "对准了但爪没夹住", 只能重跑一遍才能定位)。
              const char* src = (obs_vis_name[0] && !strcmp(obs_vis_name, gname)) ? "本轮亲眼"
                                                                                  : "沿用旧观测";
              if (has_x_tgt)
                ai::logf("[ai] 夹取依据 %s(%s, 已%d轮未刷, 车此后挪%.0fcm/转%.0f°) 目标车头系(%s%.0f,前%.0f=折算%.0f)cm 爪口偏置%.1f 横向差%.1f 夹心落车前%.0fcm 前后差%.1fcm",
                         gname, src, gage, gmove_cm, gturn_deg, glat >= 0 ? "右" : "左", fabsf(glat),
                         gfwd, gfwd * AI_NEAR_SCALE, (double)ARM_LOW_LAT_CM, fabsf(glat - ARM_LOW_LAT_CM), x_tgt,
                         fabsf(x_tgt - gfwd * AI_NEAR_SCALE));
              else
                ai::logf("[ai] 夹取依据 %s(%s, 已%d轮未刷, 车此后挪%.0fcm/转%.0f°) 目标车头系(%s%.0f,前%.0f)cm 爪口偏置%.1f 横向差%.1f 夹心落点未知",
                         gname, src, gage, gmove_cm, gturn_deg, glat >= 0 ? "右" : "左", fabsf(glat),
                         gfwd, (double)ARM_LOW_LAT_CM, fabsf(glat - ARM_LOW_LAT_CM));
            }
          }
          // 只告警的那类判据: 动作照常执行, 这句当下轮引导语喂回(与"被拒"互斥, 被拒走下面那条路径)。
          if (gadv[0]) {
            ai::logf("[ai] 夹取闸门放行(带提醒) → %s", gadv);
            snprintf(grasp_warn, sizeof(grasp_warn), "%s", gadv);
          }
          if (gwhy_hit) {
            char rej[48]; fmt_last(rej, sizeof(rej), type, params);
            ai::logf("[ai] 夹取闸门拒绝 %s → %s", rej, gwhy);
            // 留档标注: 这一帧正是"它以为对准了、其实没有"的那一帧, 被拒原因一并记上
            // (事后复盘最有价值的一类画面 —— 日志说"居中", 图上目标却在旁边)。
            char nb[160];
            snprintf(nb, sizeof(nb), "被闸门拒(未执行) %s: %s", rej, gwhy);
            ai::dump_note(nb);
            JsonDocument fd(&g_js_alloc);
            // act 要在顶层: exec::act("arm") 读的是 params["act"], 嵌进 params 子对象会变成空动作
            fd["act"] = "fold";
            exec::act("arm", fd.as<JsonObjectConst>());   // 安全回退: 收臂让出视野(爪状态不变, 不会掉物)
            since_release = false; pose_low = false;
            JsonDocument cmdF(&g_js_alloc);
            cmdF["type"] = type;
            cmdF["reason"] = gwhy;
            cmdF["params"]["act"] = "fold";
            String fb = build_feedback(t.id, cmdF);
            enqueue_result(fb.c_str(), t.fn, t.ctx);
            snprintf(grasp_warn, sizeof(grasp_warn), "%s", gwhy);   // 下轮引导语(一次性)
            stall = 0; stall_hint = false;   // 被拒是有意纠偏, 不算"无进展"
            got = true;                      // 已给 AI 明确答复, 不计入网络失败退避
            break;
          }
          // 校验通过 → 执行 move/arm/stop(本板直驱, 不再经执行板), 记录时刻供时间感知。
          // 返回值必须接: exec 拒绝时(反解失败/硬件未就绪)动作根本没落地, 以前丢掉它等于回告
          // "已执行"、AI 与操作者都只能从画面上猜为什么没动(见下方 cmdF 回告)。
          bool act_ok = exec::act(type, params);
          if (act_ok) acted = true;   // 有实际动作落地 → 空转计数清零(见 AI_IDLE_ROUNDS)
          last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
          // 闸门状态维护(仅对真执行了的动作): obs_visible=AI 本轮据画面确认过目标;
          // pose_changed=换过位姿/位置(含收臂)——两者都算"重新对准过", 落位复看的账就此销掉。
          bool pose_changed = !strcmp(type, "move") || !strcmp(type, "spin") ||
                              !strcmp(type, "arm") || !strcmp(type, "arm_pose");
          if (obs_visible || pose_changed) pose_low = false;
          if (is_low_pose && jaw_open) pose_low = true;   // 下探到位: 下轮起提示先看落位后的画面再夹
          if (!strcmp(type, "arm") && !strcmp(act, "release")) since_release = true;
          else if (pose_changed) since_release = false;
          // 低姿前顶记账(见 AI_LOW_PUSH_TTL): 爪口已贴地还往前开 = 用爪面推方块, 此后坐标不再代表
          // "方块在哪"(它被推动了, 且多半已被爪面挡住)。判"低姿"用正运动学回读的实际高度, 而不是
          // 命令里写的 h —— 命令与落点是两回事(见 AI_ARM_LOW_H_CM 那条注释)。
          // 后退即销账: 配方里那"退约1cm"一旦做了, 提醒就该停, 否则它每轮都照说一遍"先后退"。
          {
            float pp_x = 0, pp_h = 0;
            bool  low_now = exec::arm_pos(&pp_x, &pp_h) && pp_h <= AI_ARM_LOW_H_CM;
            float throttle = params["throttle"] | 0.0f;
            bool  is_grasp = !strcmp(type, "arm") && (!strcmp(act, "clip") || !strcmp(act, "grasp"));
            if (!strcmp(type, "move") && throttle > 0.001f && low_now) low_push_ttl = AI_LOW_PUSH_TTL;
            else if (is_grasp || (!strcmp(type, "move") && throttle < -0.001f) || !strcmp(type, "spin"))
              low_push_ttl = 0;
            else if (low_push_ttl > 0) low_push_ttl--;
          }
          ai::car_update_pose(type, params);
          // 无里程计兜底: AI 无定距的持续 move 单次最多行驶 AI_MOVE_CAP_MS, 到期由 exec 自动停轮; 
          // 带 distance_cm 的定距 move 已在 send_move 按时长近似自停, 不叠加持续兜底。
          if (!strcmp(type, "move") && !params["distance_cm"].is<int>() &&
              fabsf(params["throttle"] | 0.0f) > 0.001f)
            exec::set_move_cap_ms(AI_MOVE_CAP_MS);
          g_last_continuous = exec::is_continuous(type, params);
          g_last_cont_type = g_last_continuous ? (!strcmp(type, "move") ? 1 : 2) : 0;
          // 用带参数的短描述日志(如 "arm_pose x=10 h=3" / "arm lift_up / move 6cm"), 供对动作看得清。
          fmt_last(cur_cmd, sizeof(cur_cmd), type, params);
          ai::logf("[ai] 执行 %s%s", cur_cmd, g_last_continuous ? "(持续)" : "");
          {
            char nb[128];
            snprintf(nb, sizeof(nb), "%s%s%s", act_ok ? "执行 " : "执行失败(exec拒绝) ",
                     cur_cmd, g_last_continuous ? "(持续)" : "");
            ai::dump_note(nb);
          }
          String fb;
          if (act_ok) {
            fb = build_feedback(t.id, cmdD);
          } else {
            // 动作被 exec 拒绝: 回告里必须说清"没落地", 否则下游看到的还是一句已执行的应答。
            ai::logf("[ai] 执行失败(exec 拒绝) %s", cur_cmd);
            JsonDocument cmdF(&g_js_alloc);
            cmdF["type"] = type;
            cmdF["params"] = params;
            cmdF["reason"] = "执行层拒绝, 动作未落地";
            fb = build_feedback(t.id, cmdF);
          }
          enqueue_result(fb.c_str(), t.fn, t.ctx);
          // 死循环防线: 连续多轮下发相同指令 → 下轮注入引导语让 AI 主动变化
          if (strcmp(cur_cmd, last_cmd)) { stall = 0; stall_hint = false; }
          else if (++stall >= 3 && !stall_hint) { stall_hint = true; blog::logf(blog::AI, "多轮无进展, 注入引导"); }
          snprintf(last_cmd, sizeof(last_cmd), "%s", cur_cmd);
          // AI 显式要求保留上一帧(锁定/追踪): 下轮带上 prev; 否则按兜底节奏走
          want_prev = cmdD["carry_prev"].is<bool>() && cmdD["carry_prev"].as<bool>();
          // 任务笔记/进度: AI 写入并持续喂回(目标外观/计划/进行到哪)
          const char* tn = cmdD["task_note"] | "";
          if (tn[0] && strcmp(tn, task_note)) {
            strncpy(task_note, tn, sizeof(task_note) - 1); task_note[sizeof(task_note) - 1] = 0;
            ai::logf("[ai] 任务笔记: %s", task_note);
          }
          // 任务列表: 全量重写(新建/重组时用, 低频); AI 每轮只看到当前状态喂回。
          if (cmdD["tasks"].is<JsonArray>()) {
            JsonArrayConst ta = cmdD["tasks"].as<JsonArrayConst>();
            int n = 0;
            for (JsonObjectConst it : ta) {
              if (n >= 8) break;
              const char* nm = it["name"] | "";
              if (!nm[0]) continue;
              strncpy(s_tasks[n].name, nm, 47); s_tasks[n].name[47] = 0;
              s_tasks[n].done = it["done"] | false;
              n++;
            }
            if (n > 0 || s_task_n > 0) { s_task_n = n; ai::logf("[ai] 任务列表更新(%d项)", s_task_n); }
          }
          // 任务状态标记: 增量(第 N 项完成/未完成, index 从 1 起)
          if (cmdD["task_done"].is<JsonObject>()) {
            int idx = (cmdD["task_done"]["index"] | 0) - 1;
            if (idx >= 0 && idx < s_task_n) {
              s_tasks[idx].done = cmdD["task_done"]["done"] | false;
              ai::logf("[ai] 任务%d → %s", idx + 1, s_tasks[idx].done ? "完成" : "未完成");
            }
          }
          // 发给 AI 的"上一步已下发": 指令 + 完整 reason(含目标方位), 替代双帧供跨轮衔接。
          // 末尾整字符钳制(utf8_clamp_tail)防半个中文字节被云端判 400, 但不做长度截断。
          const char* r = cmdD["reason"] | "";
          {
            PsaBuf ld;   // 指令+完整 reason
            ld.put(cur_cmd); ld.put(", 原因: "); ld.put(r);
            utf8_clamp_tail(ld.p);
            // AI 决策(assistant) 直接滚入历史环(省去中转拷贝一次); ld.p 作用域末自动释放
            if (ld.len) hist_add("assistant", ld.p);
          }
          // AI 显式 task_goal: 把插话/新意图提升为目标(只替换目标 message, 不碰 system → 保缓存)
          const char* tg = cmdD["task_goal"] | "";
          if (tg[0] && strcmp(tg, goal_now)) {
            snprintf(goal_now, sizeof(goal_now), "%s", tg);
            utf8_clamp_tail(goal_now);
            ai::logf("[ai] 目标更新: %s", goal_now);
          }
          // stop + done:true = AI 自判任务完成: 停车并结束任务(build_feedback 已带 done 上报手机端); 
          // 不带 done 的 stop 仅临时停车观察, 任务继续下一轮。严格按 bool 判定, 防模型输出字符串 "true"。
          if (!strcmp(type, "stop") && cmdD["done"].is<bool>() && cmdD["done"].as<bool>()) {
            done = true;
            sent_done = true;   // 已向手机确报终态, 任务出口不再补发
            blog::logf(blog::AI, "AI 判定任务完成(stop+done)");
          }
          got = true;
          break;
        } else {
          blog::logf(blog::AI, "校验: %s", verr);
          fail = verr;  // 重试一次前暂存
        }
      }
      if (done) break;
      // 单轮模式(/ai oneshot): 一轮决策即收尾。本轮无有效输出(网络/解码/校验失败)
      // 时把原因作为 error 回报, 不跳 3s 继续观察; 持续指令残留由任务出口 resolve_stop 补停。
      if (t.one_shot) {
        if (!got && fail) {
          JsonDocument e(&g_js_alloc);
          e["error"] = fail;
          String s = build_feedback(t.id, e);
          enqueue_result(s.c_str(), t.fn, t.ctx);
        }
        // 单轮收尾: 补一条带 done 的完成通知, 让手机端复位「发送」并显示任务结束。
        JsonDocument e(&g_js_alloc);
        e["done"] = true;
        String s = build_feedback(t.id, e);
        enqueue_result(s.c_str(), t.fn, t.ctx);
        sent_done = true;   // 单轮已确报终态
        done = true;
        break;
      }
      if (got) chat_unacked = false;   // 本轮模型给出了有效决策(已进入它的上下文): 插话不必再点名
      if (!got) {
        // 网络/解析持续失败: 逐轮指数退避(3s→6s→12s→24s 封顶), 避免每轮狂建连接把云端打得更紧; 
        // 429 限流给更长喘息(30s)再继续; 失败原因同步推手机。
        if (++net_fail >= AI_MAX_NET_FAIL) {
          const char* reason = "云端持续无响应(疑似限流), 任务已中止, 请稍后重试";
          JsonDocument e(&g_js_alloc);
          e["error"] = reason;
          e["done"] = true;   // Bug1: 终结必带 done, 让手机端把「中止」复位为「发送」
          sent_done = true;
          String s = build_feedback(t.id, e);
          enqueue_result(s.c_str(), t.fn, t.ctx);
          blog::logf(blog::AI, "连续网络失败超限, 任务中止");
          break;
        }
        int wait = ai::http_last_status() == 429 ? 30000 : (3000 << (net_fail > 3 ? 3 : net_fail - 1));
        ai::logf("[ai] 本轮无有效输出(%s), %ds 后重试", fail ? fail : "未知", wait / 1000);
        vTaskDelay(pdMS_TO_TICKS(wait));
      } else {
        net_fail = 0;
      }
      // 空转兜底(见 AI_IDLE_ROUNDS): 连续多轮没有任何实际动作 → 大概率被放大镜锁在局部画面里空转。
      // 搜索只能在全幅做, 所以强制回全幅, 并把原因明说 —— 只清状态不说清, AI 下一轮还会再放大。
      if (acted) { idle_rounds = 0; }
      else if (++idle_rounds >= AI_IDLE_ROUNDS) {
        idle_rounds = 0;
        if (zoom_on) {
          zoom_on = false;
          zoom_noop_n = 0; zoom_repeat[0] = 0;
          snprintf(zoom_warn, sizeof(zoom_warn),
                   "连续 %d 轮没有实际动作, 放大镜已自动回全幅 —— 画面里没有目标时放大没有意义"
                   "(放大一块空地不会变出目标)。先 fold 收臂再用 spin 小幅环视找目标(单步≤60度), "
                   "每转一次停下看一次。", AI_IDLE_ROUNDS);
          ai::logf("[ai] 连续 %d 轮无动作, 放大镜强制回全幅", AI_IDLE_ROUNDS);
        }
      }
      acted = false;

      ai::mem_tick_stale();   // 每轮结束: 未观测的物体过期轮数 +1

      // 滚动缓存上一帧: 仅当 AI 下一轮 carry_prev=true 时才发送作对比(锁定/追踪场景), 
      // 其余单帧以节省云端处理开销。AI 不要求即永不发送此帧。
      // 一律存**全幅**(cur): 放大图只在本轮有意义, 而下一轮若已回全幅/或另一轮开了放大, prev 必须
      // 与那时的 frame 同参照系, 否则"两帧对比"比的是两块不同的地方。
      if (cur_len > 0) {
        if (!prev) prev = (uint8_t*)heap_caps_malloc(AI_EDITED_IMG_MAX, MALLOC_CAP_SPIRAM);
        if (prev && cur_len <= AI_EDITED_IMG_MAX) { memcpy(prev, cur, cur_len); prev_len = cur_len; }
        else prev_len = 0;
      }

      if (++steps >= AI_MAX_STEPS_PER_GOAL) {
        JsonDocument e(&g_js_alloc); e["done"] = true; String s = build_feedback(t.id, e); enqueue_result(s.c_str(), t.fn, t.ctx);
        sent_done = true;
        break;
      }
      // 周期控制: 以 AI_INTERVAL_MS 为下限节奏, 扣掉本轮已耗时(含抓帧/HTTP/校验), 
      // 服务端快(单帧 ~2s)时立即进入下一轮, 慢时由服务端耗时主导。
      uint64_t el = esp_timer_get_time() - step_ts;
      long rem = (long)AI_INTERVAL_MS - (long)(el / 1000);
      if (rem > 0) vTaskDelay(pdMS_TO_TICKS(rem));
    }

    // 插话到任务结束都没被落实: 明说一句 —— 否则操作者只看到"提醒没反应", 无从知道是它被
    // 报废的那几轮吃掉了(历史环随任务清空, 下一个任务不会带上它, 也不该带)。
    if (chat_unacked) blog::logf(blog::AI, "任务结束: 操作者的插话始终未被落实(随本任务作废)");

    // 任务终结补发 done: 覆盖失败/掉线等"只报 error 不带 done"的终态(Bug1),
    // 让手机端把「中止」复位为「发送」。被新目标/手动中断(interrupted)时跳过, 
    // 避免把下一任务的按钮状态误复位。正常 stop+done / 单轮 / 超步数已置 sent_done, 不再补发。
    if (!interrupted && !sent_done) {
      JsonDocument e(&g_js_alloc);
      e["done"] = true;
      if (fail) e["error"] = fail;
      String s = build_feedback(t.id, e);
      enqueue_result(s.c_str(), t.fn, t.ctx);
    }

    if (edited) free(edited);
    if (cur) free(cur);          // 本轮帧 PSRAM 副本
    if (prev) free(prev);        // 上一帧 PSRAM 副本
    if (zoom_jpg) free(zoom_jpg);   // 放大图缓冲(任务期复用, 不跨任务留着占 PSRAM)
    for (int i = 0; i < AI_HIST_N; i++) free(hist_text[i]);   // 历史环 PSRAM
    if (t.ctx) delete (int*)t.ctx;  // 任务期 sink fd
    ai::logf("[ai] 任务结束 gen=%lu", t.generation);
    // 本任务的水位低点: 串口一行(开了 /log ai 也会转发手机)。真正的价值在
    // **没开日志也读得到** —— 同一组数字也在 /log 体征行里, 走 BLE 随时可取(见 command.cpp)。
    hwatch::report("AI任务");
    free(t.text); free(t.ann);
    resolve_stop();   // 统一兜底: 持续指令残留即补停
    m_busy = false;
  }
}

// ---------------- 对外 ----------------

void ai::init() {
  if (g_worker) return;
  ground::init();   // 屏幕→地面单应拟合 + 诊断日志(见 ground_proj)
  // 注: 内部堆水位哨兵 hwatch 已在 setup 里随 blog 一起拉起(要覆盖整段运行, 不能等 AI 起来才采)。
  g_mtx = xSemaphoreCreateMutex();
  g_notify = xSemaphoreCreateBinary();
  g_img_mtx = xSemaphoreCreateMutex();
  g_result_q = xQueueCreate(8, sizeof(ResultItem*));
  // worker 的 16KB 栈改从 PSRAM 出：WiFi 的 RX 缓冲只能落内部 RAM，
  // 而这块板的内部堆长期贴红线（motion_verify/ping_svc 都栽过跟头，同样搬了栈）。
  // AI 栈每让出 1KB，收包缓冲就多 1KB 余量。AI 模块不碰 NVS/flash，
  // 所以不存在"写 flash 时 PSRAM 栈取不到"的风险。
  static StackType_t* s_ai_stack = nullptr;
  static StaticTask_t s_ai_tcb;      // TCB 必须留内部 RAM(FreeRTOS 断言)
  if (!s_ai_stack) s_ai_stack = (StackType_t*)heap_caps_malloc(16384, MALLOC_CAP_SPIRAM);
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
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  ++m_generation;
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
  g_slot.generation = m_generation;
  g_slot.fn = reply;
  g_slot.ctx = reply_ctx;   // WS: 堆 int*; BLE: nullptr
  g_slot.nav = false;       // 新 AI 目标覆盖可能的纯导航残留
  g_slot.active = true;
  xSemaphoreGive(g_mtx);
  // 尝试掐断在途请求, 让 worker 尽快回到循环取新槽
  ai::http_stop();
  g_stop_mode = (int)StopMode::All;   // 新目标打断旧任务: 残留持续指令在旧任务出口补停
  xSemaphoreGive(g_notify);
}

// /move to x y: 板端本地巡航到坐标(不调 AI)。local=以当前位姿为新原点; 
// global=沿用当前全局系(可与 AI/历史导航共用坐标系)。手动接管类: 打断在途任务。
void ai::goto_target(float x, float y, bool frame_global, long id, cmd::ReplyFn reply, void* reply_ctx) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  ++m_generation;               // 新导航接管: 在途 AI/导航作废
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
  g_slot.generation = m_generation;
  g_slot.fn = reply;
  g_slot.ctx = reply_ctx;   // 直接接管 command.cpp 预建的堆 fd(对齐 set_goal), 由 nav 分支释放
  g_slot.active = true;
  xSemaphoreGive(g_mtx);
  ai::http_stop();              // 掐断在途 AI 请求, worker 尽快回到循环取导航槽
  g_stop_mode = (int)StopMode::All;
  xSemaphoreGive(g_notify);
}

void ai::cancel(StopMode m) {
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  bool was_active = g_slot.active;  // 仅当确有任务在跑才上报, 避免手动指令刷屏
  ++m_generation;         // 使在途结果作废
  if (g_slot.text) { free(g_slot.text); g_slot.text = nullptr; }
  if (g_slot.ann) { free(g_slot.ann); g_slot.ann = nullptr; }
  if (g_slot.ctx) { delete (int*)g_slot.ctx; g_slot.ctx = nullptr; }
  g_slot.nav = false;
  g_slot.active = false;
  xSemaphoreGive(g_mtx);
  ai::http_stop();
  g_stop_mode = (int)m;   // 手动 move/stop 接管=None(不补停); arm=Wheels; ai_cancel=All
  if (was_active) blog::logf(blog::AI, "cancel");
}

bool ai::busy() { return m_busy; }

bool ai::append_chat(const char* text) {
  if (!text || !text[0]) return false;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  bool fed = m_busy;  // 仅当有任务在跑才接收插话
  if (fed) {
    strncpy(g_chat, text, sizeof(g_chat) - 1);
    g_chat[sizeof(g_chat) - 1] = 0;
    g_chat_has = true;
    blog::logf(blog::AI, "插话入队: %s", text);
  }
  xSemaphoreGive(g_mtx);
  return fed;
}