#include "src/ai/ai_prompt.h"
#include "src/net/config.h"   // cfg::ai_model
#include "src/ai/ai_mem.h"    // ai::mem_feed(空间记忆喂回)

// ---------------- AI 请求构建 ----------------

// JSON 字符串转义: 引号/反斜杠 + 控制字符(\n \r \t 等)。模型 reason 可能含换行, 裸发会让云端
// 400 "invalid unicode/character"; 中文等多字节字节序原样透传(UTF-8 合法)。
static void esc_append(PsaBuf& b, const char* s) {
  b.put('"');
  const unsigned char* p = (const unsigned char*)s;
  while (*p) {
    unsigned char ch = *p;
    if (ch == '"' || ch == '\\') { b.put('\\'); b.put((char)ch); p++; }
    else if (ch == '\n') { b.put('\\'); b.put('n'); p++; }
    else if (ch == '\r') { b.put('\\'); b.put('r'); p++; }
    else if (ch == '\t') { b.put('\\'); b.put('t'); p++; }
    else if (ch < 0x20) { char h[7]; snprintf(h, sizeof(h), "\\u%04X", ch); b.put(h); p++; }
    else if (ch < 0x80) { b.put((char)ch); p++; }
    else {
      // 多字节 UTF-8: 校验续字节, 非法/残缺则替换为 '?'(防云端判 invalid unicode code point, 400)
      // 历史环回喂的模型 reason 偶发携非良构多字节(如不合法的 unicode escape), 只能防御。
      int need;
      if (ch >= 0xC2 && ch <= 0xDF) need = 1;
      else if (ch >= 0xE0 && ch <= 0xEF) need = 2;
      else if (ch >= 0xF0 && ch <= 0xF4) need = 3;
      else { b.put('?'); p++; continue; }              // 非法首字节
      bool ok = true;
      for (int i = 1; i <= need; i++)
        if (!(p[i] >= 0x80 && p[i] <= 0xBF)) { ok = false; break; }
      if (ok) { for (int i = 0; i <= need; i++) b.put((char)p[i]); p += need + 1; }
      else { b.put('?'); p++; }                        // 残缺/非法序列: 单字节替换
    }
  }
  b.put('"');
}

static void b64_append(PsaBuf& b, const uint8_t* in, size_t inlen) {
  static const char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t i = 0;
  while (i + 2 < inlen) {
    uint32_t v = (in[i] << 16) | (in[i+1] << 8) | in[i+2];
    b.put(T[(v >> 18) & 63]); b.put(T[(v >> 12) & 63]); b.put(T[(v >> 6) & 63]); b.put(T[v & 63]);
    i += 3;
  }
  if (i + 1 == inlen) {
    uint32_t v = in[i] << 16;
    b.put(T[(v >> 18) & 63]); b.put(T[(v >> 12) & 63]); b.put('='); b.put('=');
  } else if (i + 2 == inlen) {
    uint32_t v = (in[i] << 16) | (in[i+1] << 8);
    b.put(T[(v >> 18) & 63]); b.put(T[(v >> 12) & 63]); b.put(T[(v >> 6) & 63]); b.put('=');
  }
}

static void img_block(PsaBuf& b, const uint8_t* data, size_t len) {
  b.put("{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,");
  b64_append(b, data, len);
  b.put("\",\"detail\":\"high\"}}");   // DeepSeek 只认 low/high/original/auto; medium 会被 422 拒(夹取小目标需保细节)
}

// 屏幕像素 → 地面坐标(单应投影)由独立模块 ground_proj 负责: ground::screen_to_world。

// 构建请求 body 的结构说明
// goal 当前任务目标(可被插话/ task_goal 热替换); hrole/htext/hn = 历史环条目(角色+文本,
// 同时含 assistant=AI 决策 与 user=插话), 逐条作为独立消息回喂, 构成真多轮对话记录;
// exec_state 执行板状态一行文本(无数据为空串)及"距上次执行"秒数喂当前 user。
// 系统提示词？(角色+规则+JSON格式+标定, 不含目标)+ 独立 user(目标) 消息先组进 PSRAM。
// 图预算 ≤2: carry_prev 双帧优先(放弃参考图), 否则 参考图(首轮)+当前帧。
void build_body(PsaBuf& b, const char* goal, const char* ann, const char* hint,
                const char* const* hrole, const char* const* htext, int hn,
                const char* exec_state, unsigned last_age_s,
                const char* note, const char* prog,
                const uint8_t* frame, size_t frame_len,
                const uint8_t* prev, size_t prev_len,
                bool use_prev, bool use_edited, const uint8_t* edited, size_t edited_len) {
  PsaBuf sys;
  sys.put("你带有机械臂的小车的眼睛, 负责以第一人称视角控制小车。画面中若有红色标注(方框/圆圈)需根据要求优先遵循。");
  sys.put("每次只输出一个合法 JSON(方括号内为值说明): ");
  sys.put("1. {\"type\":\"move\",\"params\":{\"throttle\":[油门, [-1,1], 正为前进],\"distance_cm\":[本次推进距离, 单位cm, 建议≤10, 省略时程序按 8cm 执行]}} 沿当前车头方向直线推进/后退; ");
  sys.put("2. {\"type\":\"spin\",\"params\":{\"dir\":[方向, +1右转/-1左转],\"angle_deg\":[旋转角度, 单位度, 建议≤60]}} 微调朝向、环视找目标; ");
  sys.put("3. {\"type\":\"arm\",\"params\":{\"act\":\"[动作]\",\"dist_cm\":[可选, 仅 grasp 用: 抬臂高度cm]}} 动作: low=降爪到贴地低姿(夹取前必做, 程序自动落到标定好的位置, 不用你给坐标) / grasp=合爪并自动抬臂(夹取, 一步到位) / release=张开 / clip=只合爪不抬臂 / fold=收臂折叠(视野最大) / lift_up·lift_down=升降 / reach_forward·reach_backward=前后伸缩; ");
  sys.put("4. {\"type\":\"arm_pose\",\"params\":{\"x\":[夹心在车头前方cm, 可达约4~15],\"h\":[夹心离地cm, 可达约1~12]}} 直接把夹爪移到指定位姿(一般用不到: 夹取主线由 low/grasp 覆盖)。机械臂只能前后伸+升降, 不能左右移动, 左右对准只能靠车; ");
  sys.put("5. {\"type\":\"stop\",\"done\":[bool值, 是否结束任务]} 停车/终止任务(完成、或确定无法继续时); ");
  sys.put("6. {\"type\":\"wait\"} 空操作, 不动车/臂(画面还在动、想先停稳再看时); ");
  sys.put("7. {\"type\":\"approach\",\"params\":{\"target\":\"[observe记录过的名字, 留空=最近目标]\"}} 按记忆自动靠近目标到约10cm; ");
  sys.put("8. {\"type\":\"zoom\",\"params\":{\"px\":[目标x, [0,1]],\"py\":[目标y, [0,1]],\"scale\":[倍数, 默认2, 约1.5~8; 放大后画面里要能同时看见目标和夹爪两指]}} 放大目标附近那块画面看清细节(带 \"reset\":true 回全幅)。画面里没有目标就别 zoom(放大空地不会变出目标); ");
  sys.put("9. {\"type\":\"light\",\"params\":{\"kind\":\"back\",\"on\":[bool值]}} 开/关照明。kind 只用 back(中性白光); front/vibe 是绿光会把画面染绿、让判色失效; ");
  sys.put("附加字段(加在任一指令 JSON 里): \
	\"reason\":\"[行动原因]\" 每轮行动必带, 一句话简要说明决策原因\
	\"carry_prev\":[是否带上本帧画面, bool值] 下轮是否带上本帧做前后对比, 发现目标且需要防止跟丢时使用; \
	\"task_goal\":\"[新目标]\" 更新当前任务目标; \
	\"observe\":{\"name\":\"[物体名字, 必填——缺名字的观测会被忽略, 同一物体固定沿用同一名字]\",\"px\":[屏幕x, [0,1], 画面最左=0],\"py\":[屏幕y, [0,1], 画面最上=0],\"visible\":[目标是否在画面内可见, bool值]} 记录/刷新物体位置: px/py 必须是你这一帧在画面里看到的实际位置(放大图里也照常填 0~1, 程序会换算回全幅); \
	\"task_note\":\"[目标外观/任务备注]\"; \
	\"tasks\":[{\"name\":\"[任务名称]\",\"done\":[bool值]}] 更新任务列表; \
	\"task_done\":{\"index\":[任务编号, 首项为1],\"done\":[bool值]} 标记第N项完成/未完成");
  sys.put("画面: 横向 u 0=画面最左(你的左边) 1=画面最右(你的右边), 没有镜像; 纵向 v 0=最远, 越大越近。左右一律以当前画面为准, 不要凭记忆或推理猜。程序喂回的物体坐标(右+/左-, 前+, 单位cm)已是当前车头系, 直接读、不要再换算; 但它由标定解算、只作粗参考 —— 近场尤其不可信: 同一个目标物体、车没动, 喂回的前距也可能差好几厘米, 所以别拿它决定往前还是退后(要挪就挪, 挪完看画面) —— 判断“对准没有”“夹住没有”一律以画面为准。\n");
  sys.put("夹地上的小目标物体统一走这条线, 不要自创流程: \
    I.	靠近: 目标远就用 approach(有记忆或标记目标时)或小步 move。目标进到画面 2/3 高及以下就该停下准备降爪了, 如果目标位置和夹爪平行则还需后退。够不够得着一律看画面里目标相对夹爪的位置, 夹爪需要提前降到低点; \
    II.	降爪: arm low —— 一次把爪降到贴地低姿(程序自动落到标定好的位置, 不用你算坐标)。降到这儿以后高度就不用再管了, 后面只调角度; \
    III.对位(全靠画面, 不看坐标): 没对准就先用小角度 spin 修, 对准后用小步 move 前进, 把目标物体送进两指之间。每走一步停下看画面(拿不准就 zoom 放大), 直到目标物体卡在两指之间; 如果已在两指之间, 且顶到目标物体, 别再往前, 此时夹子使不上劲, 先小步后退2cm让再尝试 夹取; 目标物体与夹爪的相对位置在前进后没有变化时需要小幅后退检查是否压住目标物体或角度不对, 若后退后目标仍跟随夹爪, 需要抬起夹爪并后退避免卡住物体, 随后调整角度重新对准目标物体; 在上轮目标物体已经接触夹爪且尝试前进后, 本轮不得继续前进\
    IV.	确认: zoom 到能同时看清目标物体和两指, 确认目标物体确实在两指之间(而不是还在指尖前方), 若无法确认可以标记目标物体并检查距离是否接近15cm, 在先前已经靠近并对准的情况下, 成功抬起目标会使其距离像是变远; \
    VI.	夹取: arm grasp —— 合爪并自动抬臂, 一步完成; \
    VII.验证: wait 一拍等画面静止, 再看目标物体是否明显跟着升起来。跟着起来了=夹住; 还留在原地/回到地面/从画面下方消失=空夹, 按规则6重来。\n");
  sys.put("规则: \
    1. 只输出一个合法 JSON, 每次规划一步; 无需马上行动可 wait。reason 每轮必带, 一句话说明原因; \
    2. 判“对准/夹住”的唯一依据是画面: 目标物体在两指之间 / 目标物体跟着抬起来。坐标的厘米数不能当依据; 状态行的“爪:合”只表示伺服闭合到位, 不代表夹住了物体, 一律以“抬起来目标物体是否跟着走”为准; \
    3. 看到目标就用 observe 记它, 并和本体动作写在同一次回复里(不必单独占一轮), px/py 填本帧看到的位置。移动之后旧坐标作废、要重新 observe; 但“对准没有”永远看画面; \
    4. 低姿下爪口贴着地面, 转向和前进都会拨动目标物体 —— 微操幅度要小(spin 几度、move 一两厘米), 每次动完重新看画面。目标物体被拨走了就重新 observe、重走一遍对位; \
    5. 放大图看不到全幅, 别据它估距离/方位; 挪车前先 zoom reset 回全幅。车或臂没动过时反复要同一块画面是复读, 程序会改回全幅; \
    6. 空夹了不要原样重试: 先 release, 再回到 I 重走一遍, 第二次朝同一个方向多挪一点(spin 几度), 累计约 10° 仍夹不住就回中、朝反方向同样小步试。两侧都试过还不行, 说明偏差更大, 换个距离/角度重新对位。同一动作连续两次没进展就换做法, 不要试第三次; \
    7. 画面里没有目标时用 spin 小幅环视(单步≤60°), 每转一次停下看一次, 不要连续盲转; 也可以后退一点扩大视野。别靠一进一退来回摆动去碰运气; \
    8. 画面偏暗就开 back 灯照亮, 看清后随手关; \
    9. 夹爪挡着看不清目标时(比如刚顶得太近), 先 move 后退两三厘米或 fold 收臂让出视野再看, 不要在遮挡状态下反复 observe 一个看不见的目标; \
    10. 接近目标时前方有障碍且不确定会不会撞上就先绕开; move 之后画面几乎没变化, 要考虑是否被挡住或轮子打滑");

  b.put("{\"model\":");
  esc_append(b, cfg::ai_model().c_str());
  b.put(",\"messages\":[{\"role\":\"system\",\"content\":");
  esc_append(b, sys.p ? sys.p : "");
  
  // user(目标): 独立持久消息; 目标被 task_goal 热替换时只换这条、不碰 system(保前缀缓存)
  b.put("},{\"role\":\"user\",\"content\":");
  {
    PsaBuf gt;
    gt.put("任务目标: "); gt.put(goal ? goal : "");
    if (ann && ann[0]) { gt.put("(操作者标注: "); gt.put(ann); gt.put(")"); }
    esc_append(b, gt.p ? gt.p : "");
  }
  b.put("}");
  // 历史环: 逐条独立消息(assistant=自己之前的决策 / user=操作者插话), 构成真多轮对话记录。
  // 角色交替保护: 若末条是 user 插话, 把它并入当前 user 文本, 避免"连续两条 user"被严格端点拒。
  const char* merge_tail = (hn > 0 && !strcmp(hrole[hn - 1], "user")) ? htext[hn - 1] : nullptr;
  int hn_out = merge_tail ? hn - 1 : hn;
  for (int i = 0; i < hn_out; i++) {
    b.put(",{\"role\":");
    esc_append(b, hrole[i]);
    b.put(",\"content\":");
    esc_append(b, htext[i]);
    b.put("}");
  }
  // 当前 user: 状态 + 画面(整体转义一次), 作为本轮模型的输入
  b.put(",{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":");
  PsaBuf ut;
  if (exec_state && exec_state[0]) { ut.put(exec_state); ut.put("; "); }
  if (last_age_s > 0) {
    char age[32]; snprintf(age, sizeof(age), "上一指令约%us前执行; ", last_age_s);
    ut.put(age);
  }
  if (merge_tail) { ut.put("(操作者前一条插话: " ); ut.put(merge_tail); ut.put(")"); }
  // 任务笔记/任务列表: AI 自己写入并持续喂回(目标外观/计划/各任务状态), 无需每轮重新推断。
  if (note && note[0]) { ut.put("任务笔记: "); ut.put(note); ut.put("; "); }
  if (prog && prog[0]) { ut.put(prog); ut.put("; "); }  // prog 为已渲染的"任务列表: ..."文本
  if (hint && hint[0]) { ut.put("注意: "); ut.put(hint); ut.put("。"); }
  { // 空间记忆喂回(车向 + 已记物体, 当前车头局部系)
    // 容两三条物体记忆(含"依据仍有效/别后退"那种长式) —— 满了 mem_feed 会整条停收并说明,
    // 不会再切半句(见 mem_feed 里 add() 的注释)。
    char mem_s[320];
    ai::mem_feed(mem_s, sizeof(mem_s));
    if (mem_s[0]) { ut.put(mem_s); ut.put("; "); }
  }
  if (frame) {
    if (use_prev && prev && prev_len > 0) {
      ut.put("下面按顺序给出: 上一帧、当前帧。请对比两帧, 判断画面中移动的人手/物体大致朝哪个方向移动; 若上一步动作已让目标消失, 据两帧差异推断目标方位与盲区。");
    } else if (use_edited && edited) {
      ut.put("下面按顺序给出: 操作者参考图、当前帧。参考图用于辨识目标外观/位置, 请在当前帧中寻找匹配的目标。");
    } else {
      ut.put("当前画面如下: ");
    }
  } else {
    ut.put("警告: 摄像头不可用, 当前无实时画面可分析。");
  }
  esc_append(b, ut.p ? ut.p : "");
  b.put("}");
  if (frame) {
    // 图片块间需逗号分隔; 首个(text 之后)不加。修 multi-image 缺逗号导致的 400。
    b.put(",");
    bool first = true;
    auto img = [&](const uint8_t* d, size_t n) {
      if (!first) b.put(',');
      img_block(b, d, n);
      first = false;
    };
    // 图预算 ≤2: carry_prev 双帧优先(此时放弃参考图); 否则 参考图(首轮)+当前帧
    if (use_prev && prev && prev_len > 0) img(prev, prev_len);
    else if (use_edited && edited) img(edited, edited_len);
    img(frame, frame_len);
  }
  // reasoning_effort: 曾提到 medium 想让它少选错左右/少重复动作, 但实测代价明确 —— 生成变慢到
  // 正文首字节 23~32s(卡在 30s 等正文闸上直接丢一整轮)、且出现过"正文收全却没有 content"
  // (思考把 token 吃光)。夹取闭环丢一轮 = 操作者的插话/上一步判断全部作废, 比"想得浅"贵得多,
  // 故回到 low。要再试 medium, 必须同时放宽 ai_http 的等正文闸, 别只改这一处。
  b.put("]}],\"temperature\":0.3,\"max_tokens\":8192,\"reasoning_effort\":\"low\",\"response_format\":{\"type\":\"json_object\"}}");
}