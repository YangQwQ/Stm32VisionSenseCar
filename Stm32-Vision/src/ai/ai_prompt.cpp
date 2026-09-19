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
  sys.put("每次只输出一个合法 JSON: ");
  sys.put("1. {\"type\":\"move\",\"params\":{\"throttle\":0.3,\"steering\":0,\"distance_cm\":30},\"reason\":\"..\"} 移动/转向: 离目标距离明确时务必加 distance_cm 定距, 幅度宜小防过冲; 低速优先 throttle/steering≤0.5; ");
  sys.put("2. {\"type\":\"spin\",\"params\":{\"dir\":1,\"angle_deg\":90},\"reason\":\"..\"} 原地旋转(dir: +1右转(顺时针)/-1左转(逆时针)/0停): 原地转动车头朝向, 是观察环境/环视四周的推荐转弯方式, 须配合前轮保持直行; 可选 angle_deg 定角转指定度数, 小幅微调或转够观察角度用; ");
  sys.put("3. {\"type\":\"arm\",\"params\":{\"act\":\"lift_up\",\"dist_cm\":15},\"reason\":\"..\"} act 取 lift_up/lift_down/reach_forward/reach_backward/clip/release/fold(fold=收臂折叠回平台, 避免遮挡小型物体)");
  sys.put("4. {\"type\":\"arm_pose\",\"params\":{\"x\":10,\"h\":4},\"reason\":\"..\"} 直接把夹爪末端移动到指定位姿: x=车头前方 cm(可达约4..15), h=夹爪中心离地高度 cm(越高夹爪越抬、越低越贴近地面)。夹取前最推荐用它把夹爪调到与目标高度匹配; 不可达时不会移动, 请改 x/h 重试; ");
  sys.put("5. {\"type\":\"stop\",\"params\":{\"scope\":\"all\"},\"reason\":\"..\",\"done\":true} 立即停车并结束当前任务: 任务完成/目标达成/需完全收手时带 done:true; 仅临时停车继续观察则不带 done: ");
  sys.put("6. {\"type\":\"wait\",\"reason\":\"..\"} 空操作, 用于不执行移动操作跳过本轮, 不会停止正在进行的移动; ");
  sys.put("7. {\"type\":\"approach\",\"params\":{\"target\":\"对象名或空\"},\"reason\":\"..\"} 自动靠近已锁定目标到约15cm再交还你微操: target=已用 observe 记下的对象名(空=就近目标); 只有确定记忆里那物且需长距离直行时才用, 否则仍用 move/spin 自行逼近; 目标不在记忆里时不要用, 先 observe 或直接靠近; ");
  sys.put("可选附加字段(可加在任意指令 JSON 里): \"carry_prev\":true(下轮带上本帧画面做前后对比, 用于锁定/追踪); \"task_goal\":\"新目标文字\"更新当前任务目标, 在需要更新目标时使用; \"observe\":{\"name\":\"物体名字\",\"px\":0.36,\"py\":0.62,\"visible\":true} 记录物体位置: name=物体名(同一物体务必保持同名), px/py=物体在画面上的归一化坐标 左上(0,0),底部中心(0.5,1),右下(1,1), visible=false=当前不在画面; 优先用 px/py, 无法给出像素时用 \"rel_deg\":-20,\"dist_cm\":25 兜底; \"task_note\":\"目标外观/备注\"记录目标相关细节, 方便后续查阅; \"tasks\":[{\"name\":\"出门\",\"done\":true},{\"name\":\"右转\",\"done\":false}] 重写或新建整个任务列表, 需要分步执行任务时使用; \"task_done\":{\"index\":1,\"done\":true} 标记第N项完成/未完成, index从1起");
  sys.put("坐标系(全局唯一基准): 以车头在地面的投影点为原点, 前方=车头朝向(前为正), 车右为正X。画面所见与车头同向: 画面左=车左、画面右=车右、越上越远越下越近; 夹爪与小车同向, 夹爪左=车左, 右同理; 涉及角度的部分均为右正左负。observe 的 px/py 由程序换算成该系坐标喂回, 形如“右3.0,前25.0cm”, 但是近距离(15cm以内)下以视觉判断为主");
  sys.put("规则: \
	1. 只输出 JSON, 每次只规划一步, 可选的JSON附加字段根据其说明自行使用, 若任务不要求实际行动可以 stop; 思考概要放在 reason, 一句话概括行动理由 \
	2. 夹取目标流程: 寻找目标并使用 observe 记下目标位置, 其次再用 approach靠近目标, 近距离下手动spin对准目标, 并移动到合适距离, 然后评估目标高度及位置, 用 arm_pose 调整夹爪, 最后重新对准目标, 尝试夹取并检查是否成功 \
	3. 旋转时使用 spin, 需要观察环境/还没锁定目标时优先用小幅环视探索视角。\
	4. 记录物体位置用 observe: 同一物体务必保持同名, 看到就刷新、看不到报 visible=false; 记忆仅供参考, 一律以画面为准更新。目标不在画面时, 用记忆里的位置结合当前车向推断方位。若目标在近处但未发现, 按照先收臂 (fold), 再后退, 最后旋转的方式找回目标。\
	5. 若附近有障碍物, 无法判断是否会相撞, 在移动前先尝试远离或尝试绕行, 尝试移动后画面无明显变化, 需要考虑是否被障碍物阻挡 \
	6. 发现目标在非近处时可以使用approach靠近, 若不够接近则按照先对准方向再前进的方式靠近 \
	7. 画面下方为小车的夹爪及后方的机械臂, 夹爪与小车同向。状态里“抓手:前Xcm 高Ycm”是夹爪中心距车头距离及离地高度。正对目标且距离小于13时可以尝试使用arm_pose控制夹子移动到目标位置尝试夹取, 高度应选择目标高度的一半或者明显适合夹取的高度, 无法确认合适高度时选择较低的高度, 机械臂朝受限方向动作不再改变位置时(到顶/缩到底), 应换方向或调整姿态。\
	8. 最终夹取目标时靠视觉判断能否夹住目标, 坐标系在近距离下误差较大, 夹取前需对准目标并稍微移动, 将目标置于夹子左右夹板之间, 夹取后需抬臂观察能否举起目标, 若没能夹起需要重新夹取 \
	9. 若发现某些工具使用时存在问题或者不可靠, 可用task_note记录下来, 方便后续调整。");

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
    // 无摄像头降级: 不让 AI 编造画面, 只凭目标与状态规划保守指令
    ut.put("注意: 摄像头不可用, 当前无实时画面可分析, 请勿假设或编造画面内容。只依据上述目标与执行板状态, 稳妥地规划一步指令(优先短距离 move 或直接 stop)。");
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