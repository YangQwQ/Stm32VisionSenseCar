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
// 既防"冲过头", 也让记忆坐标持续可用。取值贴 AI_APPROACH_STOP_CM: 从典型停靠距离(approach 停在
// 报 15cm 处)再走一步正好落进爪口前那一段(报 7cm, 真距约 5cm)而**不越过**它 —— 越过(报 ≤7cm)
// 方块就进了机械臂下方那个死角(既够不着又看不见, 只能抬臂后退重来, 见 AI_ARM_UNDER_CM)。
#define AI_MOVE_DEFAULT_CM 8
// AI 未给 angle_deg 的 spin 一律按此角度执行(而不是放开成"持续旋转")。同样的道理: 持续旋转期间
// car_update_pose 不累积车向(angle 未知), 于是车头朝向估计停滞, 而所有喂回的物体坐标都是用这个
// 朝向从全局系换算到车头系的 —— 朝向一错, 喂回的左右/前后全错(这是"左右说反"的另一条来路)。
// 补成有限角度后每步朝向已知; 且定角旋转有 mvfy 视觉到位补偿, 实际转角比"盲转"更准。
#define AI_SPIN_DEFAULT_DEG 30
#define AI_MOVE_MAX_CM 40         // AI 单步 move 距离硬上限(硬钳制, 覆盖提示词的"≤30cm"建议)
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
#define AI_SETTLE_MAX_MS 700      // 出帧前等轮子/补转停稳的上限(定距到点自停后仍有余速滑行与 mvfy 补转)
#define AI_SETTLE_DWELL_MS 60     // 轮子 PWM 停稳后再固定缓冲这些毫秒: 余速滑行(MV_COAST)/补转已结束但
                                  // 车身惯性还在缓慢漂(轮子已停, wheels_moving 提前返回 false), 立即抓帧仍会
                                  // 取到模糊帧 → AI 空等稳定浪费一轮。停完再等这一小段, 画面才真正静止。
#define AI_ARM_DWELL_MS 50        // 机械臂静止后再固定缓冲这些毫秒(S 形缓动/持续步进/grasp 合爪已收敛后),
                                  // 让舵机把动作走完收尾 —— 只在赛后留个小间隙, 不做近似, 出帧即"动作已完成"。
                                  // ⚠️ 不把 grasp 当作"即时"处理: 它有两步(合爪→定时抬臂), 只等缓动收敛等于
                                  // 没等抬臂, 这里的是 settle_arm 里 arm_moving 收敛后的收尾缓冲, 不是动作时长。
#define AI_FRAME_RETRY 4          // 单轮抓帧重试次数(推流并发占缓冲时会偶发取不到)

// ---------------- 放大镜(命令 zoom, 见 worker 里"放大镜"段) ----------------
// 病根: AI 在全幅(VGA)里估绝对像素不可靠 —— 方块只占 ~35px 宽(画面 640), 而它自报的 px 中位偏
// 37px、22% 的轮次超 100px; 单应标定本身又带厘米级残差(地砖校验还指向它高报约 30%), 两者叠加后
// "它以为对准了"和"真对准了"分不开, 而夹取要的正是 1.5cm 级的判断。
// 放大镜不引入任何颜色/形状/目标的假设(那类硬编码换个光照/换个目标就废): 它只是把 AI 的眼睛凑近,
// 让它在放大图里跟**看得见的夹爪**比相对位置(它的强项), 而裁框是程序记的账 ⇒ 它报的像素换回全幅
// 是精确算术, 不是估计。顺带给了"夹住没有"的判据: 合爪后看目标有没有跟着爪起来。
#define AI_ZOOM_QUALITY 80          // 比图传略高(这是用来看细节的), 一帧几十 KB
#define AI_ZOOM_JPG_MAX (128 * 1024) // 放大图单帧上限: 640×480 高清近景高熵 JPEG 可超 48K, 抬到 128K; 超了弃用回全幅
// 放大镜=切高清真拍（见 camera request_hires）：切到 SVGA 高清抓一帧，TJpgDec 部分解码只裁中央
// (0.25,0.25)-(0.75,0.75)。⚠️ 只用 SVGA 800×600：SXGA 1280×960 的 reconfigure 重建会卡死驱动(实测)。
// SVGA 中央 1/4 = 400×300 源，1:1 不缩放直接回喂，像素密度高于 VGA 全幅裁。
#define AI_HIRES_FRAME   FRAMESIZE_SVGA   // 高清目标分辨率（800×600）
#define AI_HIRES_OUT_W   400              // 放大输出＝高清源中央 1/4（1:1，同源像素密度）
#define AI_HIRES_OUT_H   300
#define AI_ZOOM_QUALITY 80          // 比图传略高(这是用来看细节的), 一帧几十 KB
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
// 顶一下 → 坐标报远 → 再顶 → 再报远, 实测就是这个前后循环(用户复述"对准到完全能夹住时又自己
// 后退, 然后一直循环前后移动")。故只在远超噪声时才给方向; 带内改为"别再挪车, 看画面确认后夹"。
#define AI_NEAR_DEADBAND_CM 5.0f
// 刚把爪降到位后的那一轮, 提示 AI 先据"落位后第一帧"确认方块是否已在两指之间再合爪。
// 只提示不拦截: 夹爪伸出去后目标可能确实完全被自身挡住(此时无画面依据可用), 硬拦会把夹取卡死。
// ⚠️ 它曾经写着"用 zoom 放大看最清楚" —— 那是在**对位阶段**就推 zoom, 与标准流程(zoom 只留给夹取
// 前的那一次确认)正好相反, 用户实测的"还没进两指之间、甚至还没对齐就反复 zoom"里, 这一句是程序
// 自己的那一半成因(另一半是提示词流程 III 里的措辞)。现在改成: 对位阶段看全幅、靠小步微调;
// zoom 等到方块已进两指之间再用(与下面 zoom 的远场闸门同源)。

#define AI_ZOOM_HINT \
  "本帧是放大图(只有画面中心区域), 看不到全幅, 若目标基本被完全遮挡则说明没对准" \

// 用户插话(ai_chat)已发给模型但那一轮没拿到有效响应(超时/空 content)时的补投提示。
// 插话在组包时就被一次性消费, 那轮的响应若报废, 建议等于没被执行——只在历史环里留着, 模型未必
// 回头认领。故在它被真正落实前, 每轮额外点一次名(正文在历史环里, 这里只提醒去认领, 不重复正文)。
#define AI_CHAT_UNACK_HINT \
  "用户有一条插话建议你还没落实(那一轮的响应失败了): 见对话历史里最后那条 user 插话, " \
  "本轮请优先按它的思路做, 或明确说明为什么不采纳。"

// 本地巡航(approach / /move to)参数: 定距段长、初对齐阈值、单次转向上限等。
// 到位距离 10cm: 别拿可达域的**外接矩形**(4~15cm)当中段用 —— 真实可达域是条下窄上宽的弯带,
// 贴地夹取(h≈1.5~2cm)时 x 只能伸到约 8~10.5cm。所以停在 12cm 会让贴地的方块永远差 1.5~2cm
// 够不着(实测就是这么空夹的: 方块在前 12cm, 爪停在 10cm 就直接合爪, 全程无任何报错)。
// 10cm 落在低处弯带中间, 滑行(COAST)后再近一点也仍在带内; 目标此时在画面下半部, 既看得见又
// 夹得到(具体落在哪个 v 由 kGroundCal 单应推出, 别在这儿再抄一份 —— 摄像头一动它就过期)。
// 与 Calibration.h 的 ARM_LOW_X_CM 是一对: 靠近停到这儿, 爪的固定准备位也在附近, 之后只需
// 厘米级微调。两者的复核时机见 Calibration.h ARM_LOW_* 处那条告警。
// AI 自动靠近的到位距离(之后交给 AI 微操)。**取 20 而非与近场同值的 15**: approach 是开环死航
// (无里程计, 靠姿态累积 + 定距时长近似), 目标全局坐标又来自单应(约高报 30%——报15cm真距仅约11cm),
// 停到"报15cm"时真距已经压到爪口(8cm)附近, 高报波动下一不小心就**越过爪口把方块压到车底/机械臂下**
// (实测日志: approach 一停目标即[过近]、爪下降直接压住方块)。留 5cm 余量停在更远的报20cm处, 宁可
// 让 AI 再多走一两步视觉微调, 也别让开环死航一次冲过头。⚠️ 这打破原"approach 停距=近场钳制线"的
// 同源绑定: 停 20 后目标不在近场档(≤15)内, AI 若一下给默认 8cm 的大步仍可能顶进盲区 —— 靠近场步长
// 钳制(mem_grasp_evidence 的实时读数)兜住, 而非靠 approach 停得近。
#define AI_APPROACH_STOP_CM 20
// "已在爪口之后"的前距线(cm, 与 AI_NEAR_SCALE 的估算口径一致 = mem_grasp_evidence 报的原始值):
// 报 7cm 时真距约 5cm, 已经**越过**低姿爪口(ARM_LOW_X_CM=8) ⇒ 方块在夹爪与车头之间的臂下。这个
// 区域里: 前进会把它推到车底, 低姿后退/转身会拖着它走或把两指扫过去把它拨开 —— 三种动作都坏事,
// 唯一安全的第一步是**先抬臂**(arm fold)。用户实测的"很容易把方块卡在机械臂下面, 然后不抬臂后退,
// 退到哪都不对"就是这个区域。判据只用于**提醒**(见 hint_cb 处), 不拦任何动作 —— 回撤永远不能拦。
// 至于"听了错的那条记忆"(同一方块被起了第二个名字、head 报前0.6cm 而另一条报前10.1cm) —— 那是
// **记忆侧**的病, 由 ai_mem 的新名字锚定 + 门限去治, 不靠去抖兜。
// ⚠️ 进/出门线两条值都搬去了 ai_mem.h(AI_ARM_UNDER_CM / AI_ARM_UNDER_EXIT_CM): 记忆行的丢失文案
// 要按同一条线分流"卡在臂下"与"贴到爪口了", 两处各写一份迟早对不上。
// 同一条记忆连续这么多轮都报"已在爪口之后" ⇒ 可疑的已经不是方块, 而是这条坐标本身(处方照做了却
// 毫无变化): 改口为"别再按它来回撤, 抬臂去看画面", 免得处方被无脑重复成空转。
#define AI_ARM_UNDER_REPEAT 4
// "目标在夹子侧面 ⇒ 退出去重来"处方的**横向**进门线(cm, 与 mem_grasp_evidence 的原始口径一致)。
// 深度那一半另有判据、用 Calibration.h 的 ARM_LOW_X_CM(低姿爪口在车前 8cm): 方块要到爪口那一带
// **或更后**才算"在夹子侧面或更后面"(用户原话), 在爪口**前方**只是偏左/偏右的目标完全够得着, 不该
// 被赶去后退。横向取 6cm 而不是沿用 AI_GRASP_LAT_HARD_CM(3.0): 近场横向读数实测每轮自跳 ±5cm
// (见 ai_mem 离群判定的底噪项), 3cm 的门等于在噪声里掷硬币。
// 实测教训(2026-09-22 23:58, 这是本人上一版只按横向判的后果): 把"前 10cm(在爪口前方 2cm)、横向
// 偏左 5.9cm"这个**够得着**的目标赶去"fold → 后退到 >17cm → 重新 approach", 一轮 40s; 退出期间
// 坐标要重取、又被离群门拒收 ⇒ 整轮 6 分钟 52 步、一次没夹住。而同一晚老固件对**更大**的横向差
// (5.7cm)只发一句附在放行上的提醒, AI 照样夹住。教训与上面 arm_under 那条同源: 近场坐标不可信,
// 别拿它堵路 —— 判断"该不该退"的是**深度**, 不是横向。
#define AI_LAT_SIDE_CM 6.0f
#define NAV_STOP_CM_GOTO 6       // /move to 的到位容差 cm(无里程计开环, 留一点松量)
#define NAV_ALIGN_DEG 15         // 目标偏角超过此值先原地转向对齐, 否则直行推进
#define NAV_MAX_SPIN_DEG 60      // 单次原地转向的角度上限(分步收敛)
#define NAV_MAX_SEG_CM 25        // 单次定距推进上限 cm(分步收敛、防冲)
#define NAV_MAX_ITERS 200        // 巡航最大迭代步数(防死循环)
#define NAV_WAIT_LOOPS 120       // 每段等待轮子停下的检测循环数(约 50ms 一拍)
// 转向预算: "左右来回转"是**发散**的签名(机制见 navigate_to 里那段), 光靠 NAV_MAX_ITERS 兜底太久
// —— 实测一次 spin 约 1.5~3s, 200 次迭代 = 好几分钟原地打转, 只能手动 reset 打断。这两条让它在
// 几秒内就认输、把控制权交回 AI(/move to 则回报未收敛)。
#define NAV_MAX_SPINS 8          // 单次巡航的原地转向次数上限
#define NAV_MAX_FLIPS 2          // 转向方向反转次数上限(超过 = 在来回蹭)
#define NAV_BACKOFF_CM 8         // "已够近却没正对"时先拉开这么多再对准 cm(见 navigate_to)
#define NAV_BACKOFF_TRIES 2      // 最多拉几次(方向按目标所在半球选; 退/进自己也会成极限环)

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
// ⚠️ wheels_moving 只认 PWM 命令(s_car_motion/s_spin), 命令一停就返回 false —— 但命令停 ≠ 车身停:
// 余速滑行/补转走完之后车身仍会因惯性缓慢漂移一小段, 此刻立即抓帧照样模糊。故在等停后再固定
// dwell AI_SETTLE_DWELL_MS, 让残余慢漂走完、画面真正静止再返回。
static void settle_wheels(unsigned long gen, int max_ms) {
  for (int t = 0; t < max_ms; t += 20) {
    if (gen != m_generation || !exec::wheels_moving()) break;   // PWM 停稳即跳出(不等满 max_ms)
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  for (int d = 0; d < AI_SETTLE_DWELL_MS && gen == m_generation; d += 20)  // 停稳后缓冲残余慢漂
    vTaskDelay(pdMS_TO_TICKS(20));
}

// 出帧前等机械臂动作到位（上限 max_ms，超时照常出帧，绝不长时间阻塞决策）。
// 必要性: wheels_moving 只盯轮子 PWM（s_car_motion/s_spin），机械臂的运动（离散 S 形缓动 / 持续步进 /
// grasp 的"合爪后自动抬臂"）它一概不覆盖 —— 于是 grasp 后下一轮立刻取帧，拍到的是"夹起过程"的中间态
// 画面（方块刚被夹起、爪子还停在低处或半途），与上一帧几乎相同 → AI 判"画面没变化"，白白多等一轮。
// 在出帧前把臂也等到静止（arm_moving 收敛到 false），grasp 的夹取+抬臂闭环才真正落到两帧的间隙里。
static void settle_arm(unsigned long gen, int max_ms) {
  for (int t = 0; t < max_ms; t += 20) {
    if (gen != m_generation || !exec::arm_moving()) break;   // 臂到位即跳出(不等满 max_ms)
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  for (int d = 0; d < AI_ARM_DWELL_MS && gen == m_generation; d += 20)  // 臂静止后再缓冲动作收尾
    vTaskDelay(pdMS_TO_TICKS(20));
}

// 本地巡航核心(纯本地, 不调云端): 把车从当前位姿导航到全局目标 (tx,ty), 距目标 ≤stop_cm 且**正对**
// 目标时停。无里程计: 按定距/定角时长近似 + 姿态累积做开环死航; 到位后剩余误差交给 AI 视觉微操兜底。
// 返回 Reached(到位)/ Interrupted(被新任务/手动打断)/ TimedOut(迭代超限收敛)。
enum class NavR : uint8_t { Reached, Interrupted, TimedOut };
static NavR navigate_to(unsigned long gen, float tx, float ty, float stop_cm) {
  const float throttle = 0.5f;   // 巡航推进油门(中速)
  int spins = 0, flips = 0, last_dir = 0, backs = 0;
  for (int it = 0; it < NAV_MAX_ITERS; it++) {
    if (gen != m_generation) return NavR::Interrupted;
    float dx = tx - ai::s_car_x, dy = ty - ai::s_car_y;
    float dist = sqrtf(dx * dx + dy * dy);
    float thg = atan2f(dx, dy) * 180.0f / ai::AI_PI;        // 目标全局方位(heading=0 时朝 Y+)
    float rel = wrap180f(ai::s_car_heading + thg);          // 目标相对车头偏角, 正=右
    if (dist <= stop_cm && fabsf(rel) <= NAV_ALIGN_DEG) {   // 到位 = 距离够近 **且** 正对目标
      JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
      return NavR::Reached;
    }
    if (fabsf(rel) > NAV_ALIGN_DEG) {                   // 偏太多先原地转向对齐
      // ★ 但"已在停距内却没正对"**绝不能原地转** —— 这是本函数唯一的发散路径。
      // 原地转是绕"车后 SPIN_PIVOT_BEHIND_CM 处"的枢轴做的, car_update_pose 会把这份平移记进车位置
      // (转 d° 位移约 2·c·sin(d/2): 40° 就有 4cm)。当 dist 只剩几厘米时, **平移改掉的方位比转动本身
      // 还多**: 转完朝另一边偏, 下一轮转回来, rel 每轮反号 ⇒ 左右来回死转。实测 2026-09-22 21:04
      // (`/move to -10,20` 之后): 两个位姿 (-7,15)/车向-7° 与 (-11,14)/车向34° 反复对跳、40+ 次原地
      // 转、持续好几秒, 只能手动 reset 打断 —— 用户看到的"移动完就一直左右来回循环转向"即此。
      // 物理上更坏: 车体绕着目标扫一圈, 低姿的两指会把方块拨走。
      // 范式给的处方正是"目标在侧面/偏后 ⇒ 抬臂 + 后退": 退开几厘米, dist 离开枢轴量级, 几何就不再
      // 退化, 下一轮的转才收敛。只放行一次(退/进自己也能变成极限环), 之后认输交回画面。
      if (dist <= stop_cm) {
        // 退开的方向**取决于目标在哪个半球**：目标在前半球 ⇒ 倒车拉距离；已在侧后方(|rel|>90°)
        // ⇒ 倒车只会更靠近它（甚至压上去），必须往**前**开才拉得开。此前这一支对 |rel|>90° 直接
        // 落进下面的"坐标可疑"分支停车返回——用户看到的"停到目标侧面偏后、程序却说到了"就是它。
        // 最多退 NAV_BACKOFF_TRIES 次（退/进自己也能变成极限环），仍不正对才认输交回画面。
        if (backs++ < NAV_BACKOFF_TRIES) {
          float ax0 = 0, ah0 = 0;
          bool low = exec::arm_pos(&ax0, &ah0) && ah0 <= AI_ARM_LOW_H_CM;
          if (low) {                                     // 低姿别拖着方块动
            JsonDocument f(&g_js_alloc); f["act"] = "fold";
            exec::act("arm", f.as<JsonObjectConst>());
            wait_wheels(gen);
          }
          bool behind = fabsf(rel) > 90.0f;
          JsonDocument p(&g_js_alloc);
          p["throttle"] = behind ? throttle : -throttle; p["steering"] = 0;
          p["distance_cm"] = NAV_BACKOFF_CM;
          exec::act("move", p.as<JsonObjectConst>()); ai::car_update_pose("move", p.as<JsonObjectConst>());
          ai::logf("[ai] 导航: 已在 %.0fcm 内却偏 %d°(此处原地转只会把车绕枢轴平移、越转越偏), "
                   "先%s %dcm 再对准", dist, (int)rel, behind ? "前进拉距离(目标已在侧后方)" : "退",
                   NAV_BACKOFF_CM);
          wait_wheels(gen);
          continue;
        }
        // 退开重对后仍不正对(或目标已到车后) ⇒ 可疑的已经不是转向, 而是**这条坐标本身**。
        JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
        ai::logf("[ai] 导航: 车已在 %.0fcm 内却仍偏 %d° ⇒ 这条坐标可疑(真在停距内不该偏这么多), "
                 "停车交回画面微操", dist, (int)rel);
        return NavR::Reached;
      }
      int dir = rel > 0 ? 1 : -1;
      if (last_dir && dir != last_dir) flips++;
      last_dir = dir;
      if (++spins > NAV_MAX_SPINS || flips > NAV_MAX_FLIPS) {
        JsonDocument s(&g_js_alloc); s["scope"] = "all"; exec::act("stop", s.as<JsonObjectConst>());
        ai::logf("[ai] 导航中止: 转向 %d 次/换向 %d 次仍未对准(目标还在 %.0fcm 外) ⇒ 停止打转, 交回 AI",
                 spins, flips, dist);
        return NavR::TimedOut;
      }
      int ang = (int)fminf(fabsf(rel), (float)NAV_MAX_SPIN_DEG);
      JsonDocument p(&g_js_alloc); p["dir"] = dir; p["speed"] = 800; p["angle_deg"] = ang;   // 全系统统一 800 档
      exec::act("spin", p.as<JsonObjectConst>()); ai::car_update_pose("spin", p.as<JsonObjectConst>());
      wait_wheels(gen);
    } else {                                            // 已正对: 定距直行一段(分步收敛)
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



// ---------------- 输出校验(防误动作) ----------------
// 向 char 缓冲安全追加一段(空串跳过; 首个不清分隔、后续加 "; ")。返回 ref=false 表示缓冲已满。
// 供"AI 工具摘要"拼多个通道用: 逐段 append, 防 snprintf 回写越界(snprintf 不会, 但要自己算长度)。
static bool safe_append(char* buf, size_t cap, bool* first, const char* part) {
  size_t used = strlen(buf);
  const char* sep = *first ? "" : "; ";
  size_t need = strlen(sep) + strlen(part);
  if (used + need + 1 > cap) return false;   // 放不下: 跳过本段, 不硬塞
  snprintf(buf + used, cap - used, "%s%s", sep, part);
  *first = false;
  return true;
}

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
  // 顶层出现旧单指令格式 type → 拒(给处方): 分组协议已取代它, 本轮要动的子系统用 move/arm/light/zoom 通道。
  if (doc["type"]) {
    snprintf(err_buf, err_cap,
             "AI 输出了旧版单指令格式(type=%s); 请改用分组 JSON: move/arm/light/zoom 通道(缺席=不动)",
             doc["type"].as<const char*>());
    return err_buf;
  }
  // 分组通道校验: 出现=本轮该子系统动作, 缺席=不动; move/arm 内部单选, light/zoom 可任意组合。
  // move 通道(轮子): throttle=直行 / spin=原地转 / approach=自动靠近, 三选一。
  if (doc["move"].is<JsonObject>()) {
    JsonObjectConst src = doc["move"].as<JsonObjectConst>();
    const char* mt = src["type"] | "";
    if (strcmp(mt, "throttle") && strcmp(mt, "spin") && strcmp(mt, "approach")) {
      snprintf(err_buf, err_cap, "AI move 通道非法 type=%s(须 throttle/spin/approach)", mt);
      return err_buf;
    }
    JsonObject m = out["move"].to<JsonObject>();
    m["type"] = mt;
    if (!strcmp(mt, "throttle")) {
      float th = src["throttle"] | 0.0f;
      float st = src["steering"] | 0.0f;
      m["throttle"] = constrain(th, -1.0f, 1.0f);
      m["steering"] = constrain(st, -1.0f, 1.0f);
      int dc = src["distance_cm"] | 0;
      if (src["distance_cm"].is<int>() && dc) m["distance_cm"] = constrain(dc, 0, AI_MOVE_MAX_CM);
    } else if (!strcmp(mt, "spin")) {
      int sd = src["dir"] | 0;
      int ss = src["speed"] | 800;   // 原地旋转默认转速, 全系统统一 800 档(实测 <700 拖不动, 须给足)
      m["dir"] = constrain(sd, -1, 1);
      m["speed"] = constrain(ss, 0, 1000);
      // 定角取绝对值、为 0 不写键(下游补默认角): 方向由 dir 定(见旧注释, 防"持续旋转不累积车向")。
      if (src["angle_deg"].is<int>()) {
        int ad = src["angle_deg"] | 0;
        int mag = ad < 0 ? -ad : ad;
        if (mag > 0) m["angle_deg"] = constrain(mag, 1, AI_SPIN_MAX_DEG);
      }
    } else {  // approach
      const char* tgt = src["target"] | "";
      if (tgt[0]) m["target"] = tgt;
    }
  }
  // arm 通道(机械臂): low/raise/fold/grasp/clip/release/pose 单选。持续抬落/伸缩(lift_*/reach_*)
  // 已从 AI 词表移除 —— 固定抬臂用 raise(一次到位不抽搐), 显式坐标用 pose。
  if (doc["arm"].is<JsonObject>()) {
    JsonObjectConst src = doc["arm"].as<JsonObjectConst>();
    const char* act = src["type"] | "";
    static const char* acts[] = {"low","raise","fold","grasp","clip","release","pose", nullptr};
    bool good = false;
    for (int i = 0; acts[i]; i++) if (!strcmp(act, acts[i])) { good = true; break; }
    if (!good) {
      snprintf(err_buf, err_cap,
               "AI arm 非法 type=%s(可用 low/raise/fold/grasp/clip/release/pose; 持续抬落/伸缩已移除)", act);
      return err_buf;
    }
    JsonObject a = out["arm"].to<JsonObject>();
    a["type"] = act; a["act"] = act;   // act 供 exec::act("arm") 与 fmt_last 复用旧键
    if (!strcmp(act, "pose")) {
      float px = src["x"] | 0.0f;
      float ph = src["h"] | 0.0f;
      if (src["x"].is<float>() || src["x"].is<int>()) a["x"] = constrain(px, 0.0f, 20.0f);
      if (src["h"].is<float>() || src["h"].is<int>()) a["h"] = constrain(ph, -2.0f, 25.0f);
    }
  }
  // light 通道: kind 合法, on 必须显式给 bool(缺省拒, 防 AI 漏写 on 把灯误关)。
  if (doc["light"].is<JsonObject>()) {
    JsonObjectConst src = doc["light"].as<JsonObjectConst>();
    const char* kind = src["kind"] | "";
    if (strcmp(kind, "front") && strcmp(kind, "vibe") && strcmp(kind, "back")) {
      snprintf(err_buf, err_cap, "AI light 非法 kind=%s", kind);
      return err_buf;
    }
    if (!src["on"].is<bool>()) {
      snprintf(err_buf, err_cap, "AI light 缺 on(bool): 必须显式给 on:true/false(缺省会把灯误关)");
      return err_buf;
    }
    out["light"]["kind"] = kind;
    out["light"]["on"] = src["on"].as<bool>();
  }
  // zoom 通道: 固定中央框放大, 旧 px/py/scale/reset 键已废弃(见 dispatch 固定框注释)。
  // 新协议 `{"zoom":true}`=要一张放大图(单次, 发完自动回全幅); `{"zoom":{"on":bool}}` 兼容旧形态
  // (on:true 放大 / on:false 回全幅, 即显式开关)。两者并存。
  if (doc["zoom"].is<bool>()) {
    out["zoom"] = doc["zoom"].as<bool>();   // 布尔: true=触发单次放大
  } else if (doc["zoom"].is<JsonObject>()) {
    JsonObjectConst src = doc["zoom"].as<JsonObjectConst>();
    if (!src["on"].is<bool>()) {
      snprintf(err_buf, err_cap, "AI zoom 需 bool 或 on(bool): 要张放大图填 {\"zoom\":true}");
      return err_buf;
    }
    out["zoom"]["on"] = src["on"].as<bool>();
  }
  // 顶层元数据: 与动作通道并行透传, 任意轮都可用。
  const char* reason = doc["reason"] | "";
  if (reason[0]) out["reason"] = reason;
  const char* tg = doc["task_goal"] | "";
  if (tg[0]) out["task_goal"] = tg;   // 把插话/新意图提升为当前任务目标(AI 显式标记)
  if (doc["done"].is<bool>() && doc["done"].as<bool>()) out["done"] = true;  // 任务完结标记
  // carry_prev:true = 下一轮希望同时收到本轮画面做对比(目标锁定/追踪、判断移动后目标方位)。
  if (doc["carry_prev"].is<bool>() && doc["carry_prev"].as<bool>()) out["carry_prev"] = true;
  // observe: AI 的空间观测(name/px/py/visible), 透传给 worker 更新物体记忆表。
  if (doc["observe"].is<JsonObject>()) out["observe"] = doc["observe"].as<JsonObjectConst>();
  // 任务笔记/列表/状态: 原样透传(dispatch 读取处理)。
  const char* tn = doc["task_note"] | "";
  if (tn[0]) out["task_note"] = tn;
  if (doc["tasks"].is<JsonArray>()) out["tasks"] = doc["tasks"].as<JsonArrayConst>();
  if (doc["task_done"].is<JsonObject>()) out["task_done"] = doc["task_done"].as<JsonObjectConst>();
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
    // 完整思考已打串口; /log ai on 时也转发手机(副本走 PSRAM, 见 blog::forward_text; 仅 WS)。
    // 不回投模型。L843 空 content 分支的同名串口打印保留, 这里只转发一次。
    Serial.print("[ai] 思考: "); Serial.println(rc);
    blog::forward_text(blog::AI, rc);
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
      // 三种出口分开报: 混成一句"被中断"会把"没收敛、停在半路"说成"被打断", 复盘时等于没有信息。
      f["params"]["reason"] = r == NavR::Reached     ? "已到达目标坐标"
                            : r == NavR::Interrupted ? "导航被中断"
                                                     : "未收敛(已停在最近处, 见导航日志)";
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
    char last_cmd[96] = {0};   // 上一轮"合并指令串"(死循环判据; 分组协议下可含多通道, 需比原先宽)
    unsigned long last_act_ms = 0;  // 上次真正下执行/微操指令的时刻(ms), 供给 AI 算"距上次决策多久"
    int net_fail = 0;           // 连续"无有效输出"轮数(网络/解析失败), 用于退避与上限收尾
    int stall = 0;
    bool stall_hint = false;
    bool did_grasp = false;      // 本任务已执行过夹取(arm grasp/clip): 完成判定需先经过抬臂画面核验
    bool grasp_verify_warned = false;  // 已就"夹取完成须画面核验"提醒过 AI(只催一次, 不反复纠缠)
    bool want_prev = false;     // AI 上轮 carry_prev=true → 本轮带上 prev 帧做对比
    bool want_prev_grasp = false;  // 上轮执行过 grasp/合爪 → 本轮自动带上前帧对比(夹爪前后变化=判夹住)：
                                   // AI 常发 grasp 后不自带 carry_prev, 而"合爪前 vs 合爪后"的两帧对比正是
                                   // 判断"方块有没有被夹住/随爪离地"的最强视觉信号, 程序自动给它请求上, 不依赖
                                   // 它记得要 carry_prev。
    // 放大镜状态(见"放大镜"段)。裁框一律以**全幅归一化**坐标记账: 这是"换回全幅是算术而非估计"
    // 的全部依据, 所以每次发出去都留一份 sent_*(而不是拿待用值当已用值)。
    bool  zoom_on = false;              // 下一帧起是否发放大图(AI 用 zoom 命令开/关)
    bool  zoom_one_shot = false;        // bool `{"zoom":true}` 单次放大: 发完这一帧自动回全幅
    bool  sent_zoomed = false;          // 本轮实际发出去的**是不是**放大图(observe 反算要用)
    float zoom_x0 = 0, zoom_y0 = 0, zoom_x1 = 1, zoom_y1 = 1;   // 待用裁框
    float sent_x0 = 0, sent_y0 = 0, sent_x1 = 1, sent_y1 = 1;   // 本轮实际发出的裁框
    float zoom_scale = AI_ZOOM_DEF;     // 待用倍数(日志/回告用)
    uint8_t* zoom_jpg = nullptr;        // 放大图缓冲(PSRAM, 首次用到时分配)
    char zoom_warn[256] = {0};          // 放大相关的**一次性**告知(生成失败/被强制回全幅), 见 hpush 处
    // 重复请求放大镜的**每轮**告知(不是一次性的): 空操作时画面本就是放大图(sent_zoomed=true),
    // 而 zoom_warn 只在 !sent_zoomed 时才推 —— 于是"别再重复发"这句从来没到过 AI 手上, 它拿到
    // 一张没变的图 + 一句"这是放大图", 只能猜命令生效没有, 实测原地复读 6~10 次。改成本缓冲按
    // 重复次数累加、每轮照推, 画面真的变了才清零。
    char zoom_repeat[224] = {0};
    int  zoom_noop_n = 0;               // 连续"要了手上已经有的那块"的次数
    char zoom_pose[48] = {0};           // 上次放大时的车位姿+臂姿: 判"重复请求"时要看这期间动过没有
    // "还没锁定任何目标"的**每轮**提示: 记忆里一个可用物体都没有时推(见 hpush 处)。实测(2026-09-22)
    // AI 开局看见画面右上的远处方块后, 既不 observe 也不 approach, 朝反方向 spin 环视, 转到背对目标
    // 时把桌角的插排当成目标记了下来, 整轮都在夹插排 —— 它缺的正是"先锁定"这一步, 而记忆空着时
    // 程序一句提示都没有(记忆行只剩"车头: 与任务开始时同向"这一行)。
    char no_tgt_warn[224] = {0};
    // "目标刚变得不可见"的提示: 记忆里**有**这个物体(曾正确观测过)、但最近几轮没再看到(已超新鲜线,
    // 见 ai_mem::mem_have_lost)。这跟"记忆空"是两回事 —— 物体还在原处, 只是当前画面可能被机械臂/
    // 车身挡住。仅当上一轮做过**真正的遮挡动作**(收臂进视野 low/clip/grasp/低位 pose, 或后退 move)时
    // 才推; 转向(spin)/前进/raise/fold/release 这类"主动找"的动作会清掉它(见下方落地处), 避免 AI
    // 总以为目标只是被挡而反复试探。
    char lost_tgt_warn[224] = {0};
    // 上一轮是否执行了"可能把目标遮挡/带出画面"的动作: 机械臂操(low/clip/grasp/低位 pose 这类把臂
    // 放进视野的)或后退 move。这两者一起解释"明明物体在桌上、画面里却找不到"的最常见成因 ——
    // grasp 抬臂后臂会挡在摄像头前下方, 后退则把目标甩出视野。每轮响应解析开头重置为 false, 只在这
    // 两类动作落地时置 true; 转向/前进是主动去找、收臂让视野, 均清掉。供"没锁定目标"与"已不可见"
    // 两条引导带上"目标可能被遮挡"的判断, 而不是让 AI 盲目 spin 环视。
    bool last_round_hide_target = false;
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
                   "较早的一条用户插话即将被历史丢弃; 若它表达了新任务/新目标而你还未用 task_goal 更新, 请现在更新。");
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

    // 用户插话"未落实"标记: 插话在组包时即被消费, 若那轮响应报废(超时/空 content)就等于
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
        settle_arm(t.generation, AI_SETTLE_MAX_MS);   // 等机械臂(离散定位/持续步进/grasp 抬臂)也到位再出帧
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
        // 放大镜 = 切高清真拍再裁中央框：request_hires 独占相机切到 SVGA，抓一帧高清后由调用方裁。
        // 相比旧的"VGA 全幅裁中央 1/4 放大"，这次源是高清原生像素，裁出的图 1:1 不缩放、细节真实。
        int hok = 0;
        camera_fb_t* hfb = cam::request_hires(AI_HIRES_FRAME, &hok);
        if (!zoom_jpg) zoom_jpg = (uint8_t*)heap_caps_malloc(AI_ZOOM_JPG_MAX, MALLOC_CAP_SPIRAM);
        size_t zl = 0;
        // 用 TJpgDec 部分解码裁中央的 crop_center_jpg（固定 (0.25,0.25)-(0.75,0.75)）：
        // workbuf/RGB 全走 PSRAM，不碰内部堆/DMA —— 规避旧 crop_to_jpg 全幅软解吃内部堆导致的整板卡死。
        bool ok_zoom = hfb && hok && zoom_jpg &&
                       magnify::crop_center_jpg(hfb->buf, cam::jpeg_len(hfb),
                                                hfb->width, hfb->height,
                                                zoom_jpg, AI_ZOOM_JPG_MAX, &zl,
                                                AI_HIRES_OUT_W, AI_HIRES_OUT_H, AI_ZOOM_QUALITY) && zl > 0;
        if (hfb) cam::return_frame(hfb);   // 高清帧用完归还（request_hires 内部已切回 VGA 放锁）
        if (ok_zoom) {
          frame = zoom_jpg; frame_len = zl;
          sent_zoomed = true;
          sent_x0 = 0.25f; sent_x1 = 0.75f; sent_y0 = 0.25f; sent_y1 = 0.75f;
          ai::logf("[放大镜] 高清(源%dx%d) 中央框(0.25,0.25)-(0.75,0.75) → %uKB %dms",
                   hfb->width, hfb->height, (unsigned)(zl / 1024), magnify::last_cost_ms());
        } else {
          // 切高清/裁图失败不能让本轮瞎: 退回全幅(不会走高清), 并把这事说给 AI —— 它以为在看放大图。
          ai::logf("[放大镜] 高清放大失败(%s 源%dx%d), 本轮回全幅",
                   hfb ? "裁图失败" : (hok ? "取帧失败" : "切分辨率失败"),
                   hfb ? hfb->width : src_w, hfb ? hfb->height : src_h);
          snprintf(zoom_warn, sizeof(zoom_warn), "放大图生成失败, 本帧仍是全幅(按全幅读)");
        }
      }
      // 单次放大(`{"zoom":true}`)仅喷出一张放大图: 无论本轮放大成功(发出了)还是失败(退回全幅),
      // 消费完这次请求就自动回全幅 —— AI 不必再发 on:false 关闭。旧显式开关不受影响。
      if (zoom_one_shot) { zoom_one_shot = false; zoom_on = false; }

      bool got = false;
      for (int attempt = 0; attempt < 2 && !done; attempt++) {
        // 引导语: 重试纠正 / 死循环打断(连续多轮相同指令, 带重复指令名便于 AI 自我纠正)
        char hint_buf[320];
        const char* hint;
        if (attempt) {
          // "没给正式回答(content 为空, 只有思考)"与"输出非法 JSON"是两种病, 提示要分开:
          // 前者再说"只输出合法 JSON"没用(它根本没输出), 得让它别再长篇思考、直接给指令。
          hint = (fail && !strcmp(fail, "AI 响应无内容"))
                   ? "上轮你只给了思考过程、没有给出正式回答(content 为空); 本轮请勿再长篇思考, 直接输出一个合法 JSON 词表指令。"
                   : "上次输出非法, 请只输出合法 JSON 词表指令。";
        } else if (stall_hint) {
          // 长度须留够 last_cmd 的**声明**宽度(96B, 编译器按声明判截断, 不看实际内容):
          // 22(前缀) + 96 + 122(后缀) ≈ 240, 故缓冲从 192 提到 320。
          snprintf(hint_buf, sizeof(hint_buf),
                   "连续多轮重复「%s」无进展: 换个观察角度重认目标(抬/落机械臂、后退或环视), "
                   "若已夹紧或达不成就交 done。", last_cmd);
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
        if (chat_now[0]) { hist_add("user", chat_now); chat_unacked = true; }   // 插话进入共享历史(用户话语), 落实前一直点名
        // 上一帧是否带上: 仅由 AI 上轮 carry_prev=true 决定(锁定/追踪意图), 其余保持单帧省开销。
        // 放大镜下不带上一帧: prev 存的是全幅, 两张图参照系不同(同一像素在两图里指的不是一处),
        // 对比运动只会误导。放大镜本来就是用来"静态看清相对位置"的, 不需要跨帧运动对比。
        // 带前帧的条件: AI 显式 carry_prev, **或**上轮执行过 grasp(自动请求合爪前后对比, 见
        // want_prev_grasp 声明)。消费掉这次自动对比后清标志(只影响本轮, 不持续)。
        bool use_prev = prev_len > 0 && !sent_zoomed && (want_prev || want_prev_grasp);
        want_prev_grasp = false;   // 一次性消费: 自动带的对比只给一轮
        // 用户参考图: 仅首轮带一次(初始目标外观参考); 之后不续带(已去掉 carry_user)。
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
        // "还没锁定任何目标"(见 no_tgt_warn 声明处): 每轮按记忆现算, 一有可用目标就自动停。
        // 只引导、不给判据 —— 提示词规则 3 本就有"看到目标就 observe", 这里只是把它顶到开局那一轮
        // 面前: 弱模型在"目标又小又远"时会跳过锁定直接环视, 而环视是**无依据**的, 转到哪算哪。
        if (!ai::mem_have_any()) {
          snprintf(no_tgt_warn, sizeof(no_tgt_warn),
                   "记忆里还没有可用目标: 目标若在画面里(哪怕很小/很远)就先 observe 记下 px/py, "
                   "再 approach 靠近; 画面里确实没有它, 才用 spin 小幅环视去找");
        } else {
          no_tgt_warn[0] = 0;
        }
        // 记忆里**有**这个物体、但最近几轮没再看到(已超新鲜线, 见 ai_mem::mem_have_lost) → 与"记忆
        // 空"不同: 物体大概率还在老地方, 只是被机械臂/车身挡在镜头外。此时若上一轮恰好做了可能遮挡
        // 的动作(收臂/后退/旋转), 就点名这一点 —— 避免 AI 把它当"消失"去盲目 spin 环视。只在记忆非空
        // 且上轮有遮挡动作时推, 与 no_tgt_warn 互斥。
        if (ai::mem_have_lost() && last_round_hide_target) {
          snprintf(lost_tgt_warn, sizeof(lost_tgt_warn),
                   "注意, 目标物体可能被机械臂遮挡");
        } else {
          lost_tgt_warn[0] = 0;
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
        // "还没锁定任何目标"排在近场那几条之后: 它与它们互斥(那几条都要有观测依据才成立, 记忆空着时
        // 必然一条都推不出来), 排后面只是为了让"本轮硬事实"优先占位。排在插话提醒之前。
        if (no_tgt_warn[0]) hpush(no_tgt_warn);
        // 目标刚变得不可见(记忆里有它、但最近没看到)+ 上轮遮挡动作 → 提示"可能被挡而非消失", 排在
        // 插话提醒之前, 与 no_tgt_warn 互斥(记忆非空才会走到)。
        if (lost_tgt_warn[0]) hpush(lost_tgt_warn);
        if (chat_unacked) hpush(AI_CHAT_UNACK_HINT);   // 用户插话尚未落实(它的那一轮响应报废了)
        // 下探到位后反复提示(直到它据落位后的画面 observe 一次, 或改变位姿/位置为止):
        // "夹爪放下后不重新对准就夹"是这条链路最贵的错误 —— 空夹一轮 = 一整次云端往返 + 一次
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
          // ---- 分组通道提取(新协议): move/arm/light/zoom 可选, 缺席=该子系统不动 ----
          // move 内部单选 throttle/spin/approach; arm 内部单选各固定动作; light/zoom 可任意组合。
          JsonObjectConst mv = cmdD["move"].is<JsonObject>() ? cmdD["move"].as<JsonObjectConst>() : JsonObjectConst();
          JsonObjectConst ar = cmdD["arm"].is<JsonObject>()  ? cmdD["arm"].as<JsonObjectConst>()  : JsonObjectConst();
          bool has_move  = cmdD["move"].is<JsonObject>();
          bool has_arm   = cmdD["arm"].is<JsonObject>();
          bool has_light = cmdD["light"].is<JsonObject>();
          // zoom 协议允许 `{"zoom":true}`(bool=单次要放大) 或 `{"zoom":{"on":bool}}`(显式开关)。
          bool has_zoom = cmdD["zoom"].is<JsonObject>() || cmdD["zoom"].is<bool>();
          bool has_any   = has_move || has_arm || has_light || has_zoom;
          const char* mtype = mv["type"] | "";
          bool is_mv = has_move && !strcmp(mtype, "throttle");
          bool is_sp = has_move && !strcmp(mtype, "spin");
          bool is_ap = has_move && !strcmp(mtype, "approach");
          const char* act = ar["type"] | "";   // arm 子型(low/raise/fold/grasp/clip/release/pose), 无 arm 为空
          // 重置"上轮是否可能遮挡目标"标记 = false: 本轮的提示(no_tgt_warn/lost_tgt_warn)已在上方用上
          // 一轮的值构建完, 这里起重新累计供下一轮; 默认清掉, 只把**真正的遮挡源**(收臂进视野 / 后退)
          // 在下方落地处重标为 true。转向(spin)与前进(move 前)是**主动去找**而非遮挡: 做了它们还看不到
          // 目标 → 下轮就不该再提示"被遮挡", 否则 AI 会一直觉得"只是被挡住了"而反复试探, 故它们保持
          // false。空动作轮(zoom/wait)同样清掉, 避免把很久前的遮挡一直挂在嘴上。
          last_round_hide_target = false;
          // 空间记忆: 先解析 AI observe(用**动作前**的车位姿把观测转全局)。
          // ⚠️ 位置在**所有闸门之前**(曾经夹在近场步长钳制与夹取闸门之间): 闸门判的"记忆里的坐标"
          // 必须含**本轮这一帧**的观测 —— AI 习惯把 observe 与动作写在同一次回复里, 而上一轮的记忆在
          // 刚进近场的那个转折点恰好还在近场线之外。实测(2026-09-22): 本轮 observe 报前 13.1cm(该钳),
          // 闸门看到的却是上一轮的前 17.1cm(不钳) ⇒ 那条 `move 4cm th=0.4` 原样满油门执行, 直接把
          // 方块顶进机械臂下方, 触发了随后两分多钟的"抬臂→后退→降爪"空转。保护恰好漏掉了它最该
          // 保护的那一轮。同理受益的还有放大镜时机闸门与近场旋转钳制。
          // 被闸门拒掉时这一轮的观测同样要收下(下面的 break 不跳过这里) —— 观测是事实, 与动作是否
          // 被允许无关。写在 `car_update_pose` 之前即可, 本位置早于它。
          bool obs_visible = false;   // 本轮 AI 报"看得见且记成了"→ 夹取依据/落位复看都算销账
          char obs_vis_name[16] = {0};   // 记成的那条是谁 —— 闸门放行时要报"依据是本轮亲眼看的吗"
          if (cmdD["observe"].is<JsonObject>()) {
            JsonObjectConst ob = cmdD["observe"].as<JsonObjectConst>();
            const char* nm = ob["name"] | "";
            // visible 已废弃(提示词不再让 AI 标): 只要 AI 提交 observe(写明了名字+位置)就说明它
            // 这一帧看到了该物体、要记录 → 一律视为可见。哪些"没看到"由 AI 自己把握 —— 它不对某物
            // observe, 该物就不会被刷新(新旧由 mem 的过期轮数体现)。
            bool vis = true;
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
          // ---- 主动作/并行辅助标记(供末尾统一落地与反馈); acted 用外部 while 级声明, 勿在此重复遮蔽 ----
          bool nav_done = false;                 // approach 已执行导航
          const char* nav_why = nullptr;         // approach 结果说明(供主动作反馈/历史)
          const char* nav_tgt = mv["target"] | "";
          // ---- approach(轮子自动靠近): 本地巡航, 不点云端。与合爪不同轮(导航后车动、坐标过时) ----
          if (is_ap) {
            // ⚠️ 只拦 approach+**合爪**(grasp/clip): approach 会挪车、坐标过时, 拿旧坐标合爪必空夹;
            //   而 approach+**降爪**(arm low)不再拦 —— 降爪落到标定固定点、不影响"夹没夹对", 坐标过时
            //   只影响之后的横向微调, 那恰好靠"降爪后重新 observe、凭画面小步对位"解决(与夹取闸门里
            //   "arm low 不按坐标拒"同源)。AI 常在 approach 前先把爪降好, 为贴近地面方块做准备,
            //   拦它只会逼 AI 多耗一轮(实测日志里 AI 因此反复被顶回, 停在"较远→approach"循环里)。
            if (has_arm && (!strcmp(act, "grasp") || !strcmp(act, "clip"))) {
              // approach 与夹取由两个动作通道组成、车会先挪动 ⇒ 夹取判据的坐标过时, 整轮拦截。
              JsonDocument cmdF(&g_js_alloc);
              cmdF["type"] = "approach";
              cmdF["reason"] = "approach 与合爪不能同轮: 先 approach, 下轮据画面观察确定是否对齐后再决定夹取; arm low/fold/raise/light 可与 approach 并行";
              cmdF["params"]["target"] = nav_tgt[0] ? nav_tgt : "(最近)";
              String fb = build_feedback(t.id, cmdF);
              enqueue_result(fb.c_str(), t.fn, t.ctx);
              {
                const char* r = cmdD["reason"] | "";
                PsaBuf ld;
                ld.put("approach 被拦(与降爪/合爪同轮), 原因: "); ld.put(r);
                utf8_clamp_tail(ld.p);
                if (ld.len) hist_add("assistant", ld.p);
              }
              got = true; net_fail = 0;
              break;
            }
            float tx, ty;
            bool found = ai::mem_find(nav_tgt[0] ? nav_tgt : nullptr, &tx, &ty);
            // 坐标可信度闸: 上一轮若走了"判历史已失效 ⇒ 整条重置", 这条坐标此刻就只由**那一条曾被判
            // 离群的观测**撑着。赶路是全盲死航, 拿它冲过去不是"顶偏一点"而是**直接顶到方块上**。
            char awarn[384] = {0};
            bool areset = found && ai::mem_untrusted(nav_tgt[0] ? nav_tgt : nullptr);
            if (!found) {
              nav_why = nav_tgt[0] ? "approach 目标不在记忆里, 请先 observe 锁定" : "approach 无可用目标记忆, 请先 observe";
            } else if (areset) {
              int wn = snprintf(awarn, sizeof(awarn),
                       "「%s」的位置刚被整条重置(上两轮观测都与历史冲突、且彼此吻合 ⇒ 程序判了历史失效), "
                       "此刻它只由那一次观测撑着 —— 照它开过去会直接顶到方块。本轮别赶路: 用全幅看画面"
                       "确认它在不在那儿(在就 observe 把它记回来), 下一轮再 approach",
                       nav_tgt[0] ? nav_tgt : "目标");
              if (wn >= (int)sizeof(awarn))
                ai::logf("[ai] ⚠ 挡下文案被截断(%dB>%dB): 句尾处方已丢, 加宽 awarn", wn, (int)sizeof(awarn));
              nav_why = awarn;
              ai::logf("[ai] approach 挡下(目标坐标刚被整条重置): 本轮不赶路, 先看画面确认位置");
            } else {
              ai::logf("[ai] approach 目标 全局(%.0f,%.0f)cm 停距%dcm", tx, ty, AI_APPROACH_STOP_CM);
              NavR r = navigate_to(t.generation, tx, ty, AI_APPROACH_STOP_CM);
              if (r == NavR::Interrupted) { interrupted = true; done = true; break; }   // 被接管: 本任务作废
              nav_done = true;
              nav_why = (r == NavR::Reached) ? "已自动靠近目标, 交还你微操" : "靠近收敛结束, 由你继续";
              last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
            }
            acted = true;   // approach 算一次有意推进(空转兜底/死循环都销账)
          }
          // ---- zoom(放大镜): 通道 `{"zoom":true}`(bool=单次要放大, 发完自动回全幅) / `{"zoom":false}`(回全幅)
          //      或旧形态 `{"zoom":{"on":bool}}`(显式开关)。挡下则整轮 break ----
          if (has_zoom) {
            const bool zoom_direct = cmdD["zoom"].is<bool>();
            const bool want_zoom = zoom_direct ? cmdD["zoom"].as<bool>()
                                               : (cmdD["zoom"]["on"] | false);
            // bool 形态 = 一次性放大: 发完这一帧(下轮组帧时消费)自动回全幅, AI 不必再发 on:false。
            // 旧 object 形态(显式开关)不标单次, 维持跨轮直到 on:false —— 兼容旧客户端/旧 prompt。
            if (zoom_direct && want_zoom) zoom_one_shot = true;
            if (want_zoom) {
              // 放大: 固定中央框(忽略 AI 的 px/py/scale, 见下)。
              set_zoom_box(0.5f, 0.5f, AI_ZOOM_DEF);
              zoom_on = true;
              bool zoom_changed = (zoom_x0 != sent_x0 || zoom_x1 != sent_x1 ||
                                   zoom_y0 != sent_y0 || zoom_y1 != sent_y1);
              float zp_x = 0, zp_h = 0; exec::arm_pos(&zp_x, &zp_h);
              char zp_now[48];
              snprintf(zp_now, sizeof(zp_now), "%.1f,%.1f,%d,%.1f,%.1f",
                       (double)ai::s_car_x, (double)ai::s_car_y, (int)ai::s_car_heading,
                       (double)zp_x, (double)zp_h);
              bool same_place = !strcmp(zp_now, zoom_pose);
              snprintf(zoom_pose, sizeof(zoom_pose), "%s", zp_now);
              ai::logf("[ai] 放大镜 %.1f× 中心(全幅%.2f,%.2f) 框(%.2f,%.2f)-(%.2f,%.2f)%s%s",
                       AI_ZOOM_DEF, 0.5f, 0.5f, zoom_x0, zoom_y0, zoom_x1, zoom_y1,
                       zoom_changed ? "" : " 无变化", same_place ? "" : " 位姿已变");
              if (!zoom_changed && same_place) {   // 连续要同一块且车臂未动 = 复读, 连要多半就回全幅
                zoom_noop_n++;
                if (zoom_noop_n > AI_ZOOM_NOOP_MAX) {
                  zoom_on = false;
                  snprintf(zoom_repeat, sizeof(zoom_repeat),
                           "同一块画面已连要 %d 次, 而这期间车和臂都没动过 —— 画面一字未变, 再要也不会有"
                           "新信息。已回全幅: 下一步必须是实体动作(move/arm/spin/wait)", zoom_noop_n);
                  ai::logf("[ai] 放大镜复读 %d 次(车臂未动), 强制回全幅", zoom_noop_n);
                  zoom_noop_n = 0;
                } else {
                  snprintf(zoom_repeat, sizeof(zoom_repeat),
                           "这张图就是你手上那张(画面不会变), 第%d次重复请求: 别再发同一条 zoom, 下一步给实体动作",
                           zoom_noop_n);
                  ai::logf("[ai] 放大镜重复请求(画面已是这块), 保持不动");
                }
              } else {
                zoom_noop_n = 0; zoom_repeat[0] = 0;   // 画面真变了(或车/臂动过): 重复的账销掉
              }
            } else {
              // 回全幅
              zoom_on = false;
              zoom_one_shot = false;
              zoom_noop_n = 0; zoom_repeat[0] = 0;
              ai::logf("[ai] 放大镜关闭, 回全幅");
            }
            JsonDocument cmdF(&g_js_alloc);
            cmdF["type"] = "zoom";
            cmdF["reason"] = zoom_on ? "已放大本轮(中央框): 本帧是放大图, 下一轮自动回全幅"
                                     : "已回全幅";
            cmdF["params"]["on"] = zoom_on;
            String fb = build_feedback(t.id, cmdF);
            enqueue_result(fb.c_str(), t.fn, t.ctx);
            {
              const char* r = cmdD["reason"] | "";
              PsaBuf ld;
              ld.put(zoom_on ? "zoom 放大镜" : "zoom 回全幅"); ld.put(", 原因: "); ld.put(r);
              utf8_clamp_tail(ld.p);
              if (ld.len) hist_add("assistant", ld.p);
            }
            // 换视角是有意推进, 但空操作(要一张手上就有的图)不算:
            if (zoom_on) { stall = 0; stall_hint = false; }
          }
          if (is_mv) {
            // 未给 distance_cm 时补有限段长: 否则执行层按"持续"处理、无法累积位移, 见原 move 补段长注释。
            if (!mv["distance_cm"].is<int>() && fabsf(mv["throttle"] | 0.0f) > 0.001f) {
              cmdD["move"]["distance_cm"] = AI_MOVE_DEFAULT_CM;
              ai::logf("[ai] move 未给 distance_cm, 补为 %dcm", AI_MOVE_DEFAULT_CM);
            }
          }
          // spin 同理: 未给 angle_deg 时补默认角(持续旋转不累积车向 → 记忆基准失同步)。
          if (is_sp && !mv["angle_deg"].is<int>() && (mv["dir"] | 0) != 0) {
            cmdD["move"]["angle_deg"] = AI_SPIN_DEFAULT_DEG;
            ai::logf("[ai] spin 未给 angle_deg, 补为 %d°", AI_SPIN_DEFAULT_DEG);
          }
          // ---------- 统一元数据处理: 任意轮(含空动作/approach/zoom)都生效 ----------
          want_prev = cmdD["carry_prev"].is<bool>() && cmdD["carry_prev"].as<bool>();
          { const char* tn = cmdD["task_note"] | "";
            if (tn[0] && strcmp(tn, task_note)) {
              strncpy(task_note, tn, sizeof(task_note) - 1); task_note[sizeof(task_note) - 1] = 0;
              ai::logf("[ai] 任务笔记: %s", task_note);
            } }
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
          if (cmdD["task_done"].is<JsonObject>()) {
            int idx = (cmdD["task_done"]["index"] | 0) - 1;
            if (idx >= 0 && idx < s_task_n) {
              s_tasks[idx].done = cmdD["task_done"]["done"] | false;
              ai::logf("[ai] 任务%d → %s", idx + 1, s_tasks[idx].done ? "完成" : "未完成");
            }
          }
          { const char* tgoal = cmdD["task_goal"] | "";
            if (tgoal[0] && strcmp(tgoal, goal_now)) {
              snprintf(goal_now, sizeof(goal_now), "%s", tgoal);
              utf8_clamp_tail(goal_now);
              ai::logf("[ai] 目标更新: %s", goal_now);
            } }

          // ---------- 逐通道落地(move/arm/light), 汇总主动作/合并串 ----------
          char merge_str[96] = {0};          // 本轮合并指令串(死循环判据 + 历史 + 留档)
          JsonDocument acmd(&g_js_alloc);     // 主动作(手机端 command 显示), 优先级 move>spin>arm>light>approach
          bool has_main = false;
          bool any_ok = false, any_attempted = false;
          bool pose_changed = false;          // 本轮是否动过车/臂(销对准账)
          if (is_mv) {
            JsonObjectConst mcv = cmdD["move"].as<JsonObjectConst>();
            bool ok = exec::act("move", mcv);
            acted |= ok; any_ok |= ok; any_attempted = true;
            ai::car_update_pose("move", mcv);   // 定距 move 累积位移(approach 由 navigate_to 内部自累积, 勿重复)
            pose_changed = true;
            if (!has_main) { acmd["type"] = "move"; acmd["params"] = mcv; has_main = true; }
            int dc = mcv["distance_cm"] | 0; float mth = mcv["throttle"] | 0.0f;
            snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
                     "move %dcm th=%.1f; ", dc, mth);
            // 后退(th<0)会把目标甩出视野; 前进不遮。仅在确认落地时标, 供下轮"没锁定目标"提示判断。
            if (ok && mth < -0.001f) last_round_hide_target = true;
          } else if (is_sp) {
            JsonObjectConst mcv = cmdD["move"].as<JsonObjectConst>();
            bool ok = exec::act("spin", mcv);
            acted |= ok; any_ok |= ok; any_attempted = true;
            ai::car_update_pose("spin", mcv);
            pose_changed = true;
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
              JsonDocument a(&g_js_alloc); a["act"] = act;
              ok = exec::act("arm", a.as<JsonObjectConst>());
            }
            acted |= ok; any_ok |= ok; any_attempted = true; pose_changed = true;
            if (!has_main) { acmd["type"] = "arm"; acmd["params"] = acv; has_main = true; }
            if (!strcmp(act, "grasp") || !strcmp(act, "clip")) { did_grasp = true; want_prev_grasp = true; }
            else if (!strcmp(act, "release")) { want_prev_grasp = false; }
            // 机械臂"放进视野"的动作(low/clip/grasp 或低位 pose)落地后, 下一帧臂会在镜头前下方挡住目标;
            // raise/fold/release 是收臂/让出视野, 不遮。与 move 后退同义 —— 都解释"画面里找不到目标"。
            bool hide_arm = ok && (!strcmp(act, "low") || !strcmp(act, "clip") ||
                                   !strcmp(act, "grasp") ||
                                   (!strcmp(act, "pose") && (acv["h"] | 0.0f) <= AI_GRASP_H_CM));
            if (hide_arm) last_round_hide_target = true;
            snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str), "arm %s; ", act);
          }
          if (has_light) {
            JsonObjectConst lcv = cmdD["light"].as<JsonObjectConst>();
            bool ok = exec::act("light", lcv);
            acted |= ok; any_ok |= ok;
            if (!has_main) { acmd["type"] = "light"; acmd["params"] = lcv; has_main = true; }
            snprintf(merge_str + strlen(merge_str), sizeof(merge_str) - strlen(merge_str),
                     "light %s %s; ", (lcv["kind"] | ""), (lcv["on"] | false) ? "on" : "off");
          }
          if (nav_done && !has_main) {
            acmd["type"] = "approach"; acmd["params"]["target"] = nav_tgt[0] ? nav_tgt : "(最近)"; has_main = true;
          }
          if (any_attempted || nav_done) last_act_ms = (unsigned long)(esp_timer_get_time() / 1000);
          // AI 的 move/spin 全有界(throttle 补距离 / spin 补默认角)、arm 全离散 → 无持续残留, 无需 move_cap 兜底。
          g_last_continuous = false; g_last_cont_type = 0;

          // ---------- 反馈 / 历史 / 死循环 ----------
          if (!has_any && !nav_done) {
            // 空动作轮(只带 reason/observe/task_* 等元数据): wait 语义自理。
            stall = 0; stall_hint = false;
            uint64_t now = (uint64_t)(esp_timer_get_time() / 1000);
            if (now - g_last_wait_fb_ms >= AI_WAIT_FB_MIN_MS) {
              g_last_wait_fb_ms = now;
              String fb = build_feedback(t.id, cmdD);
              enqueue_result(fb.c_str(), t.fn, t.ctx);
            }
          } else {
            if (has_main && (any_ok || nav_done)) {
              acmd["reason"] = cmdD["reason"] | "";
              String fb = build_feedback(t.id, acmd);
              enqueue_result(fb.c_str(), t.fn, t.ctx);
            } else if (has_main) {
              ai::logf("[ai] 执行失败(exec 拒绝) %s", merge_str);
              JsonDocument cmdF(&g_js_alloc);
              cmdF["type"] = acmd["type"] | "none";
              cmdF["params"] = acmd["params"];
              cmdF["reason"] = "执行层拒绝, 动作未落地";
              String fb = build_feedback(t.id, cmdF);
              enqueue_result(fb.c_str(), t.fn, t.ctx);
            }
            // 死循环防线: 合并串比对(空动作轮已在上文走 wait, 不误伤)
            if (strcmp(merge_str, last_cmd)) { stall = 0; stall_hint = false; }
            else if (++stall >= 3 && !stall_hint) { stall_hint = true; blog::logf(blog::AI, "多轮无进展, 注入引导"); }
            snprintf(last_cmd, sizeof(last_cmd), "%s", merge_str);
            if (merge_str[0]) {   // 留档
              char nb[128];
              snprintf(nb, sizeof(nb), "执行%s%s", merge_str, any_ok ? "" : "(exec拒绝)");
              ai::dump_note(nb);
            }
          }
          // 历史回喂: 合并串(或 approach/等待说明) + 完整 reason
          {
            const char* r = cmdD["reason"] | "";
            PsaBuf ld;
            ld.put(merge_str[0] ? merge_str
                                : (nav_done ? (nav_why ? nav_why : "approach") : "wait"));
            ld.put(", 原因: "); if (r) ld.put(r);
            utf8_clamp_tail(ld.p);
            if (ld.len) hist_add("assistant", ld.p);
          }

          // ---------- AI 决策日志: 本轮"用了哪些工具 + 完整 JSON" ----------
          // 复盘靠的是 AI 给的原样 JSON, 不能截断 —— 用 blog::forward_text 走 PSRAM 任意长转发
          // (与 reasoning 同一通道, 只转发手机、不占 256B 的 logf 栈缓冲); 另打一条短摘要方便扫。
          if (content.length() > 0) {
            char tob[160] = {0};   // 短摘要: JSON 里的顶层工具 + 主动作
            bool first = true;
            if (cmdD["observe"].is<JsonObject>()) {
              JsonObjectConst ob = cmdD["observe"].as<JsonObjectConst>();
              int t = snprintf(tob, sizeof(tob), "observe(%s,px%.2f,py%.2f)",
                               ob["name"] | "", ob["px"] | 0.0f, ob["py"] | 0.0f);
              first = false;
            }
            if (has_zoom) first = safe_append(tob, sizeof(tob), &first, "zoom");
            if (has_move && is_mv)  safe_append(tob, sizeof(tob), &first, is_ap ? "approach" : "move");
            if (has_move && is_sp)  safe_append(tob, sizeof(tob), &first, "spin");
            if (has_arm)            safe_append(tob, sizeof(tob), &first, "arm");
            if (cmdD["light"].is<JsonObject>()) safe_append(tob, sizeof(tob), &first, "light");
            if (cmdD["done"].is<bool>() && cmdD["done"].as<bool>())
              safe_append(tob, sizeof(tob), &first, "done");
            ai::logf("[ai工具] 本轮: %s", tob[0] ? tob : "(纯 wait/reason)");
            blog::forward_text(blog::AI, content.c_str());   // 完整 JSON(任意长), 供复盘
          }

          // ---------- done 门: done:true = 任务完成(含停车); 保留夹取画面核验闸门 ----------
          if (cmdD["done"].is<bool>() && cmdD["done"].as<bool>()) {
            if (did_grasp && !grasp_verify_warned) {
              // 夹取动作已落地, 但 AI 还没在"抬起后"这一轮明确核验过 —— 拦下这次 done, 要求下一轮
              // **据当前画面**(而非记忆坐标)核验, 防"举起看不见却报成功"。只拦一次。
              grasp_verify_warned = true;
              hist_add("user", "夹取动作已执行。在宣布任务完成前, 请据**当前画面**核验确实夹住了: "
                               "方块的黄色应仍在两指之间并随机械臂升离地面、原地面该位置已空出; "
                               "若看不到方块在爪中或不确定, 不算夹住 —— 应 arm release 松开后重新对位再试, "
                               "或明确说明。确认夹住后再交 done。");
              ai::logf("[ai] 夹取验证: 拦下本次完成, 已要求画面核验(抬升看爪中)");
            } else {
              done = true;
              sent_done = true;   // 已向手机确报终态, 任务出口不再补发
              blog::logf(blog::AI, "AI 判定任务完成(done)");
            }
            JsonDocument st(&g_js_alloc); st["scope"] = "all";
            exec::act("stop", st.as<JsonObjectConst>());   // 停机收尾
          }
          got = true;
          break;
        } else {
          blog::logf(blog::AI, "校验失败(%s), AI 原样返回: %.240s", verr, content.c_str());
          // 校验失败常因 AI 输出非纯 JSON(带多余正文/残缺字段), 只看 err_buf 前80B 根本看不出它写了
          // 什么。把**原样 content 全文**也转发出来(任意长, 与 reasoning 同通道) —— 复盘"它到底返回了
          // 什么导致被拒"才有东西可看。仅在校验失败时打, 不刷屏。
          blog::forward_text(blog::AI, content.c_str());
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
          zoom_one_shot = false;
          zoom_noop_n = 0; zoom_repeat[0] = 0;
          snprintf(zoom_warn, sizeof(zoom_warn),
                   "连续 %d 轮没动作, 放大镜已自动回全幅: 画面里没目标时放大没有意义。"
                   "先 fold 收臂, 再用 spin 小幅环视找(单步≤60°), 每转一次停下看画面。",
                   AI_IDLE_ROUNDS);
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

    // 插话到任务结束都没被落实: 明说一句 —— 否则用户只看到"提醒没反应", 无从知道是它被
    // 报废的那几轮吃掉了(历史环随任务清空, 下一个任务不会带上它, 也不该带)。
    if (chat_unacked) blog::logf(blog::AI, "任务结束: 用户的插话始终未被落实(随本任务作废)");

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