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
  sys.put("1. {\"type\":\"move\",\"params\":{\"throttle\":[油门, 取值范围[-1, 1], 正为前进],\"distance_cm\":[移动距离, 单位cm, 每次小幅移动(≤30cm)]}} 用途: 沿当前车头方向直线推进/后退。何时用: 方向已对齐、距离明确时; ");
  sys.put("2. {\"type\":\"spin\",\"params\":{\"dir\":[方向, +1右转/-1左转],\"angle_deg\":[旋转角度, 单位度]}} 何时用: 对准、环视找目标或微调朝向; ");
  sys.put("3. {\"type\":\"arm\",\"params\":{\"act\":\"[动作, 可选 clip/release/fold ,分别为 夹取, 松夹, 收臂]\",\"dist_cm\":[移动距离, 单位cm]}} 用途: 操作机械臂/抓夹。何时用: 夹取或松开目标时, 不夹取时先 fold 收臂; ");
  sys.put("4. {\"type\":\"arm_pose\",\"params\":{\"x\":[车头前方距离(坐标系的Y轴), 单位cm, 可达范围约[4, 15]],\"h\":[夹爪中心离地距离(坐标系的Z轴), 单位cm, 可达范围约[0.5, 12]]}} 用途: 把夹爪末端直接移到指定位姿。何时用: 近距(≤13cm)夹取前把爪对准目标的高度与位置; ");
  sys.put("5. {\"type\":\"stop\",\"done\":[bool值, 是否结束任务, 不结束任务时可不带done字段]} 用途: 停车/终止任务。何时用: 任务完成/目标达成/确定无法继续/需完全收手时; ");
  sys.put("6. {\"type\":\"wait\"} 用途: 本轮空操作, 不动车/臂。何时用: 刚 observe 记录位置、等待画面稳定或应先不动再决策时; ");
  sys.put("7. {\"type\":\"approach\",\"params\":{\"target\":\"[observe记录过的目标对象名, 留空时为最近目标]\"}} 用途: 按记忆自动靠近目标到约15cm, 无需手动对准。何时用: 目标已在记忆且需较长距靠近时; 记录位置用 observe, 靠近用 approach; ");
  sys.put("附加字段(加在任一指令 JSON 里): \
	\"reason\":\"[行动原因]\" 每轮行动必带, 一句话简要说明决策原因\
	\"carry_prev\":[是否带上本帧画面, bool值] 下轮是否带上本帧做前后对比, 发现目标且需要防止跟丢时使用; \
	\"task_goal\":\"[新目标]\" 更新当前任务目标, 任务开始或更新时使用; \
	\"observe\":{\"name\":\"[物体名字, 省略时自动更新先前物体]\",\"px\":[屏幕x, 取值范围[0, 1]],\"py\":[屏幕y, 取值范围[0, 1]],\"visible\":[目标是否在画面内可见, bool值]} 记录/刷新物体位置: 同一物体固定沿用同一名字, 无法确定是否为同一物体时不标记; px/py 为画面归一化坐标(左上(0,0),右下(1,1)); \
	\"task_note\":\"[目标外观/任务备注]\" 记录目标的相关信息或执行时需要注意的问题, 如后续需要注意的事或者某些功能存在问题等; \
	\"tasks\":[{\"name\":\"[任务名称]\",\"done\":[是否完成, bool值]}] 更新任务列表, 执行需要跟踪进度的多步任务时使用; \
	\"task_done\":{\"index\":[任务编号, 首项为1],\"done\":[是否完成, bool值]} 标记第N项任务完成/未完成");
  sys.put("坐标系(全局唯一基准): 以车头在地面的投影点为原点, 车头朝向为Y轴正向, (0, 16.5)位置为画面中心, 车右为X轴正向。画面所见与车头大致同向: 越上越远越下越近; 夹爪与小车同向。observe 的 px/py 由程序换算成该系坐标喂回, 形如“右3.0,前25.0cm”, 但是近距离(15cm以内)下以视觉判断为主");
  sys.put("规则: \
    1. 只输出一个合法 JSON, 每次规划一步; 若无需马上行动可 stop 或 wait。思考概要放 reason; \
    2. 夹取流程: 寻找目标 → 标记目标并大致接近 → 微调位置和夹子 → 靠近目标并夹取 → 抬起夹子检查; \
    3. 寻找目标时, 若画面没有目标, 使用 spin 小幅定角环视观察环境, 角度选用不超过60度避免刚好错过目标; \
	4. 发现目标后, 用observe标记目标; 若为初次标记, 同时使用approach 靠近, 若目标已足够接近, 则wait而不用move/spin避免位置偏差; \
    5. 若目标丢失, 且先前使用直行到达目标近处且机械臂遮挡画面且未收起, 尝试用fold收臂, 否则小幅后退让出视野; 若仍无发现目标, 重新小幅spin环视寻找目标; \
    6. 接近目标时, 若附近有障碍物且无法判断是否相撞时, 移动前先远离或绕行; 尝试移动后画面无明显变化, 需考虑是否被障碍物阻挡; \
    7. 调整夹爪时, 若正对目标且距离近时可 arm_pose 把高度调到目标一半或明显合适夹取的高度, 同时对准目标, 让目标处于正前方方便对准; \
    8. 夹取目标前, 需通过画面确认目标在夹爪的两夹板之间的可夹取位置, 然后尝试clip夹取, 随后抬臂确认目标随爪抬起才算夹住; 否则需release并重新对准");

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
    char mem_s[192];
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
  b.put("]}],\"temperature\":0.3,\"max_tokens\":8192,\"reasoning_effort\":\"low\",\"response_format\":{\"type\":\"json_object\"}}");
}