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
// 同时含 assistant=AI 决策 与 user=插话), 逐条作为独立消息回喂, 构成真多轮对话记录。
// 消息通道: system(固定规则) → user(目标=用户输入) → 历史环 → 独立 system 诊断消息
// (板状态/任务笔记/任务列表/注意: 提示/空间记忆, 承载所有**非用户输入**) → user(画面描述+图)。
// 图预算 ≤2: carry 帧(prev/放大/用户图)优先(放弃参考图), 否则 参考图(首轮)+当前帧。
void build_body(PsaBuf& b, const char* goal, const char* ann, const char* hint,
                const char* const* hrole, const char* const* htext, int hn,
                const char* exec_state, unsigned last_age_s,
                const char* note, const char* prog,
                const uint8_t* frame, size_t frame_len,
                const uint8_t* prev, size_t prev_len,
                bool use_prev, bool use_edited, const uint8_t* edited, size_t edited_len) {
  PsaBuf sys;
sys.put(R"PROMPT(
你是一个小车车手, 负责坐在小车左后方根据画面快速决策控制小车完成目标
# 规则
	- 每轮只输出一个合法 JSON, 可组合多个行为; 每种行为只能出现一次, 例如不能同时选 move throttle 和 move spin。除 reason 外每一项都非必须, 按需选用
	- 信息来源的优先级: 用户发言 > 系统提示 > 系统规则
# 规则中的用词规范
	- 规则中的方括号用于标记有明确定义的内容、强调或描述内容
	- 当强调[画面上]只考虑物体在画面上的上下左右关系
# 控制指令
## 小车方位移动: 
	- {"move":{"type":"throttle","throttle":-1|0|1,"distance_cm":[推进距离, 默认8cm]}} 前进(1)/后退(-1) 指定距离, 在前进前先对准
	- {"move":{"type":"spin","dir":-1|1,"angle_deg":[角度, 建议≤60]}} 原地旋转, 左转(-1)/右转(1) 指定角度
	- {"move":{"type":"approach","target":"[observe记过的名字, 留空=最近目标]"}} 按标记位置靠近目标, 目标距离[较远]时使用, 可能无法直接到达[接近]距离, 只在初次接近目标时使用
## 机械臂/夹爪移动:
	- {"arm":{"type":"low|raise|grasp|clip|release|fold|pose","x":[4~15cm],"h":[1~12cm]}}; (x, h)参数仅pose动作有效, 其它时候不写, low=夹爪移动至贴地位置 / raise=抬起夹爪到高位 / grasp=合爪并抬臂, 同时传递图片给下轮对比验证 / clip=合上夹爪 / release=松开夹爪 / fold=收臂折叠, 避免遮挡 / pose=移动夹爪到指定坐标
## 其它行为:
	- {"reason":"[行动原因]"} 每轮必带, 一句话概括本轮决策及原因, 此内容用户可见
	- {"goal":finish|abort|fail} 最终目标 已完成, 可以结束会话/需要中止并等待用户输入/目标已不可能达成 时使用, 若中止后用户未输入则继续进行原先任务
	- {"zoom":true}: 下一轮放大画面中央区域((0.25, 0.25)至(0.75, 0.75)区域), 检查[目标物体]是否进入[两指之间]时才使用
	- {"carry_image":"zoom|full|image1~3"} 下轮可以额外查看的一张图片: zoom/full 带上本帧的 放大/全幅 画面, 给下一轮做前后对比, 发现目标且距离接近, 需要防止跟丢用full, 验证动作/观察夹爪附近 用zoom; 行动可能导致目标不可见时才使用; image1|image2|image3 查看用户传入的图片
	- {"light":{"kind":"front|back|vibe","on":true|false}} 画面偏暗, 照亮/判断颜色用 front(前灯, 白色); back(尾灯, 红色); vibe(侧面氛围灯,深蓝色)
	- {"task_goal":"[新目标]"} 更新当前任务的最终目标, 目标需要变更时使用
	- {"observe":{"name":"[物体名字]","px":[0.0~1.0],"py":[0.0~1.0]}} 记录/刷新物体位置, 目标可见时总是使用, px/py 统一填物体的底部中心在本帧画面上的位置，放大图也照常填 0~1
	- {"task_note":"[目标外观/任务备注]"} 可用于记录目标特征, 不过记录物体位置可能会对后续造成误导; 也可以用于备注信息, 例如备注用户发送的图片内容等
	- {"tasks":[{"name":"[任务名称]"}]} 重写整个任务列表, 需要执行分步任务时使用
	- {"task_done":{"index":[任务编号, 首项为1],"done":true|false}} 标记第 N 项完成/未完成
## 输出示例
	- {"observe":{"name":"[目标]","px":0.75,"py":0.75},"tasks":[{"name":"找到[目标]"},{"name":"夹取"},{"name":"送到A点"},{"name":"放下"}]"reason":"目标距离不好判断, 先创建任务, 观测并等待一轮"}
	- {"move":{"type":"approach","target":"[目标]"},"observe":{"name":"[目标]","px":0.5,"py":0.25},"reason":"目标较远, 先尝试靠近"}
	- {"arm":{"type":"raise"},"move":{"type":"throttle","throttle":-1},"reason":"目标卡在夹爪与小车之间, 后退并抬臂避让"}

# 基础定义
## 方位描述及旋转
	- (px, py)为基于屏幕的归一化坐标, 左上(0,0), 右下(1,1)。小车朝向在[画面上]表现为从(0.625,1)朝向(0.375,0), 画面中心点约小车正前14cm
	- 现实坐标系以车头为原点, 车正前为x轴正向, 车正右为y轴正向, h 为离地高度。系统表示小车, 物体位置用(x,y), 表示夹爪位置用(x,h)
	- 旋转时画面大致以底部中心为圆心旋转, 车及机械臂的部分保持不动, 可以借此判断旋转是否会撞到物体
## 外观描述
	- [画面上]右下可见小车主体的前半部分, 顶部的机械臂结构连接至夹爪
	- [画面上]总是可见夹爪左指, 夹爪左前端向左上伸出的黑色细棍的平直段为左指, 长约2.5cm
	- 左指右边的黑色立方体是夹爪的舵机, 右指被其遮挡。[两指之间]是左右指之间的区域, 画面上表现为左指与舵机间的空隙, 松爪时该区域宽约3cm
## 状态判定
	- 如何准确判断物体位置
		+ if (系统显示物体的屏幕坐标与观察到的画面基本一致): 系统提供的距离数字准确
		+ elif (目标[较远]): 自己观测的屏幕坐标可靠
		+ else: 无需考虑坐标, 用[画面上]的物体位置关系判断
	- 距离判定
		+ if ([目标物体] py>=0.3 || 系统提示[目标物体]已[接近]):
			* if ([目标物体]在[画面上]处于左指上方位置, 水平方向上不相交): [目标物体]距离[接近], 需要对准
			* else: [目标物体]处于[过近]距离
		+ else: [目标物体]仍处于[较远]距离
	- 对准判定
		+ if ([目标物体]在[画面上]不可见): 没有[对准]
		+ elif (未arm low 或夹爪高度大于2cm): 无法[对准], 需要先arm low
		+ else:
			* if ([目标物体]不处于[过近]距离):
				- if ([目标物体]在[画面上]处于左指正上方): 此时[目标物体]已对准, 可以继续接近
			* else:
				- if ([目标物体]在[画面上]处于左指左侧): [目标物体]没有[对准], 位置偏左, 需要稍微后退并左转对准
				- elif ([目标物体]在[画面上]处于夹爪舵机右侧或被机械臂结构遮挡)): [目标物体]没有[对准], 位置偏右, 需要稍微后退并右转对准
				- elif ([目标物体]在[画面上]的左侧与左指右侧紧贴)): [目标物体]进入[两指之间], 可以用 grasp/clip 夹取
				- elif ([目标物体]被遮挡, 难以辨认或基本不可见): [目标物体]可能已被卡住, 抬臂并后退会比较合适
				- else: [目标物体]可能已经进入[两指之间], zoom后无法确认可以尝试夹取
	- 夹取结果判定
		+ if (夹爪未clip): 未执行夹取, 不可能[夹住]
		+ elif([目标物体]在[画面上]不可见): [目标物体]被遮挡或丢失
		+ elif ([画面上]夹爪右指图层在[目标物体]上, 即遮挡[目标物体]): [目标物体]仍在地面上, 未[夹住]
		+ else:
			* if ([目标物体]进入[两指之间]): 已[夹住]
	- 目标状态判定
		+ if (曾经发现过目标):
			* if (上一步为前进 || 上一步为机械臂动作): 可能被遮挡或过于靠近
			* elif (上一步为旋转): 可能旋转过头
			* else: 目标可能已被移动, 考虑重新搜索
		+ else: 需要搜索目标, 可以每步60度旋转搜索, 同时标记较开阔区域, 搜索不到目标时可以前往该区域重新搜索

# 工作流程
	1. 确认目标: 确认当前目标以及是否需要更新
	2. 场景理解: 描述画面中的关键物体的当前状态
	3. 行为决策: 根据正在执行的任务和当前状态, 选择合理的动作
	4. 最终校验: 重新审查动作决策是否可靠, 是否使用了不可靠的信息或可能会发生碰撞、卡住目标物体或导致距离过近 并提出修正方案, 大部分情况下的可行方案是抬臂并后退
)PROMPT");

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
  for (int i = 0; i < hn; i++) {
    b.put(",{\"role\":");
    esc_append(b, hrole[i]);
    b.put(",\"content\":");
    esc_append(b, htext[i]);
    b.put("}");
  }
  // 本轮诊断: 独立 system 消息承载所有**非用户输入**(板状态/任务笔记/任务列表/注意: 提示/空间记忆),
  // 不再挂在 user 通道冒充用户发言。因诊断与画面分成两条消息, 末条 user 插话也无需再并入当前 user。
  PsaBuf dia;
  if (exec_state && exec_state[0]) { dia.put(exec_state); dia.put("; "); }
  if (last_age_s > 0) {
    char age[40]; snprintf(age, sizeof(age), "上一指令约%us前执行; ", last_age_s);
    dia.put(age);
  }
  // 任务笔记/任务列表: AI 自己写入并持续喂回(目标外观/计划/各任务状态), 无需每轮重新推断。
  if (note && note[0]) { dia.put("任务笔记: "); dia.put(note); dia.put("; "); }
  if (prog && prog[0]) { dia.put(prog); dia.put("; "); }  // prog 为已渲染的"任务列表: ..."文本
  if (hint && hint[0]) { dia.put("注意: "); dia.put(hint); dia.put("。"); }
  { // 空间记忆喂回(车向 + 已记物体, 当前车头局部系)
    char mem_s[448];
    ai::mem_feed(mem_s, sizeof(mem_s));
    if (mem_s[0]) { dia.put(mem_s); dia.put("; "); }
  }
  if (dia.p && dia.p[0]) {
    b.put(",{\"role\":\"system\",\"content\":");
    esc_append(b, dia.p);
    b.put("}");
  }
  // 当前 user: 画面(文本描述整体转义一次 + 图块), 只承载图像输入。
  // ⚠️ 图片受 API 限制只能走 user 消息(放 system 会被 400), 于是"系统实时画面"也以 user 身份到达 ——
  // 就地声明来源, 否则模型会把画面当成用户发来的东西。
  b.put(",{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":");
  PsaBuf ut;
  if (frame) {
    if (use_prev && prev && prev_len > 0) {
      ut.put("下面按顺序给出两张系统实时画面(非用户发送): 上一帧、当前帧: ");
    } else if (use_edited && edited) {
      ut.put("下面按顺序给出: 用户发送的参考图、当前帧(系统实时画面, 非用户发送): ");
    } else {
      ut.put("当前画面如下(系统实时画面, 非用户发送): ");
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
    // 图预算 ≤2: carry 帧(prev/放大/用户图)优先(此时放弃参考图); 否则 参考图(首轮)+当前帧
    if (use_prev && prev && prev_len > 0) img(prev, prev_len);
    else if (use_edited && edited) img(edited, edited_len);
    img(frame, frame_len);
  }
  b.put(R"CFG(]}],"temperature":0.3,"max_tokens":8192,"reasoning_effort":"low","response_format":{"type":"json_object"}})CFG");
}