#include "src/ai/ai_prompt.h"
#include "src/net/config.h"      // cfg::ai_model
#include "src/ai/tools/tool.h"   // ai::tools_schema(请求尾的工具声明)

// ---------------- AI 请求构建 ----------------

// JSON 字符串转义: 引号/反斜杠 + 控制字符(\n \r \t 等)。模型文本(工具参数/名称)可能含换行, 裸发会让云端
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
  b.put("\",\"detail\":\"high\"}}");   // DeepSeek 只认 low/high/original/auto; medium 会被 422 拒
}

// 屏幕像素 → 地面坐标(单应投影)由独立模块 ground_proj 负责: ground::screen_to_world。

// 请求体尾部的工具声明(agent 循环): `car` 批量动作 / `mem` 物体记忆(记录/删除/查询) / `task` 任务记账 /
// `goal` 目标与收尾 / `look` 取画面 / `say` 对用户说话 / `compact` 压缩历史。
// 与上面的 system 提示词是一套东西的两半: 结构(键名/枚举)在这里, 行为与判据在提示词里;
// 改键名/枚举必须两处同时改(现在同文件, 改起来方便)。
// 取值合法性不靠这里的 schema(没用 strict: DeepSeek 的 strict 要求所有属性 required+additionalProperties,
// 与我们"car 的键基本全可选"冲突), 仍由 registry.cpp 各 parse 兜底校验。
// 放在请求体尾部(messages 之后): 前缀缓存吃的是 system+历史那一段, 工具声明放后面不破坏它。
// ⚠️ 尾巴上的 "tool_choice":"auto" 是风险点: DeepSeek 官方接入说明称 V4 思考模式不接受 tool_choice,
// 严格时会 400。当前实测能过, 故先留着; 若哪天开始 400, 第一件事就是摘掉它。
const char* ai::tools_schema() {
  return R"TOOLS("tools":[
{"type":"function","function":{"name":"car","description":"对小车/机械臂下达一批动作。\n每轮至少调用一个工具, 同一种工具最多一次。\n一次 car 调用可同时给多个键(如 move + light 一起做); 各键都可不写, 只给本轮要用的即可。","parameters":{"type":"object","properties":{
"move":{"type":"object","description":"轮子, 五选一且每次只用一个, 方向已含在 type 里: forward 前进 / backward 后退 → value 填距离 cm, 前进前先确保目标对准; spin_left 原地左转 / spin_right 原地右转 → value 填角度; 微调对准先用比较接近的角度, 如果过头再逐轮砍半角度反向转;\n approach 靠近 → target(物体记忆里的名字, 缺省=最近目标, 如果还未observe可以两个同时使用直接前往目标附近); 只在初次接近较远目标时用, 物体过近时用可能大幅转向","properties":{
"type":{"type":"string","enum":["forward","backward","spin_left","spin_right","approach"]},

"value":{"type":"integer","description":"forward/backward: 移动距离, 单位cm(可省略, 默认8); spin_left/spin_right: 旋转角度(可省略, 默认30)"},
"target":{"type":"string","description":"approach: 记忆里的目标名(可省略, 缺省=最近目标)"}}},

"arm":{"type":"object","description":"机械臂/夹爪, 每次只用一个 type(x/h 仅 pose 有效, 其它时候不写):\nlow 夹爪降到贴地准备位; 目标在画面上不处于[左指]上方时需先后退, 避免压住目标\nraise 抬到高位; 压住物体或物体可能在车头近处时用, 找回目标后应先旋转对准\ngrasp 合爪并抬臂; 夹取目标时用, 之后记得 look 确认是否夹住\nclip / release 合上 / 松开夹爪; 需要手动控制夹取流程时用\nfold 收臂折叠, 避免遮挡; 疑似压住物体或机械臂遮挡视野时用\npose 移动夹爪到指定坐标","properties":{
"type":{"type":"string","enum":["low","raise","fold","grasp","clip","release","pose"]},"x":{"type":"number","description":"pose: 轴前 cm(4~15)(可省略)"},"h":{"type":"number","description":"pose: 离地 cm(1~12)(可省略)"}}},

"light":{"type":"object","description":"车灯: front=前灯(白色, 照亮/判断颜色用它) / back=尾灯(红) / vibe=氛围灯(深蓝)","properties":{"kind":{"type":"string","enum":["front","back","vibe"]},"on":{"type":"boolean","description":"开启/关闭"}}}
}}}},

{"type":"function","function":{"name":"mem","description":"物体记忆与小车姿态: 记录(observe)/删除(delete)目标位置, 或查询当前记忆。\n只给 observe/delete 时按给的内容返回执行回执; 什么都不给(或空对象)=查询小车全局坐标/朝向与已记忆的物体坐标。","parameters":{"type":"object","properties":{
"observe":{"type":"array","description":"记录/刷新物体记忆的位置(即查询结果里的物体位置), 查看新画面且目标可见时总是使用; px/py 统一填物体底部中心在最新画面上的屏幕坐标(放大图也照常填 0~1, 程序会自动换算); 请勿对着用户参考图或回看的旧图使用observe","items":{"type":"object","properties":{"name":{"type":"string"},"px":{"type":"number"},"py":{"type":"number"}}}},
"delete":{"type":"array","description":"删除已记忆的物体; 发现重复记录同一物体、或记忆已无用时的清理","items":{"type":"string"}}
}}}},

{"type":"function","function":{"name":"task","description":"任务记账(纯记录, 不含动作): note 记要点 / todo 重写任务列表 / done 标记已完成项。","parameters":{"type":"object","properties":{
"note":{"type":"string","description":"记录目标特征(如大小、形状), 避免后续误认, 位置尽量不记; 也可备注用户发送图片的内容"},
"todo":{"type":"array","description":"重写整个任务列表, 需要分步任务时用","items":{"type":"string"}},
"done":{"type":"array","description":"标记第N项完成(首项为1, 可同时标多项; 取消标记需重写任务列表)","items":{"type":"integer"}}
}}}},

{"type":"function","function":{"name":"goal","description":"任务目标与收尾。set 更新最终目标; finish 结束本次任务(结束语或总结先用 say 发送)。","parameters":{"type":"object","properties":{
"set":{"type":"string","description":"更新当前任务的最终目标, 目标需要变更时使用"},
"finish":{"type":"string","enum":["done","fail","wait"],"description":"结束本次任务: done=已完成 / fail=目标已不可能达成 / wait=中止并等待用户输入(用户未回复则继续原任务)"}
}}}},

{"type":"function","function":{"name":"look","description":"查看最多两个画面: 默认新拍一张当前全幅画面, 可叠加放大或回看先前给过的画面, 最多两张(超出的忽略); 回看编号见结果里的「当前可查看图片」; 历史消息中的旧图会退化成占位符; 动作结果与预期一致时直接继续。查询记忆用 mem, 本工具只管取画面。","parameters":{"type":"object","properties":{
"zoom":{"type":"boolean","description":"新拍一张当前实景画面, 并选择是否为放大版, 放大画面只能看见(0.25,0.25)至(0.75,0.75)的中央区域; 放大画面仅在检查[目标物体]是否可以被夹取时用, 普通的对准及其它场景用普通画面已足够; 目标不在夹爪附近时使用不放大的画面更合适"},
"image":{"type":"array","description":"回看先前给过的画面(含用户发送的参考图), 填编号数组; 编号见上一次 look 结果里的「当前可查看图片」, 不要凭空写编号; 与新拍画面合计最多 2 张, 超出的会被忽略, 编号已超出保留范围的会被告知","items":{"type":"integer"}}
}}}},

{"type":"function","function":{"name":"compact","description":"压缩历史上下文: 对话轮数变多时(状态块会提醒)用一段摘要概括此前进展; 调用后更早的对话被清空, 只留这条摘要开始的后续部分。目标/任务列表/任务笔记/物体记忆/车位姿都不受影响, 但可回看的旧画面(含用户参考图)会一并清空, 要看东西需重新 look","parameters":{"type":"object","properties":{
"summary":{"type":"string","description":"用中文写给之后的自己看: 摘要只写已确认事实、后续步骤、需要注意的事或总结出的经验, 不写未证实的猜测; 清空后你只能靠这段文字回忆之前做过什么"}},
"required":["summary"]}}},

{"type":"function","function":{"name":"say","description":"对用户说话, 也可用来回答用户的提问; 有值得汇报的进展、结论或要解释的事时使用; 没什么可说可以不使用, 认为没有必要的话允许不说话, 但是建议在每个任务阶段, 或者执行一定次数后说话一下","parameters":{"type":"object","properties":{
"text":{"type":"string","description":"要说的话, 一句话, 用户可见; 执行任务时用于写接下来准备干什么等; 结束时可以用来总结、回答用户问题或向用户提问"}},
"required":["text"]}}}
],"tool_choice":"auto")TOOLS";
}

// 构建请求 body(入参见 BodyReq)。消息通道: system(规则) → 历史环 → 尾部(状态块挂最新工具结果 / user 画面)。
void build_body(PsaBuf& b, const BodyReq& r) {
  PsaBuf sys;
sys.put(R"PROMPT(
你是Caris, 一个带猫娘气质的小车驾驶助手。你的首要目标是准确、高效地帮助用户, 猫娘语气只是轻微调味, 不能影响信息传达

# 人设
##【身份】
	- 名字: Caris
	- 自称: 我; 偶尔“本喵”, 每10句不超过1次
	- 称呼用户: 默认用“你”
	- 关系: 亲近但不过度黏人，像熟悉的搭档

##【语气】
	- 默认自然、简洁、直接
	- 猫娘感来自轻微语气词、少量猫相关动作比喻, 而不是每句加"喵"

##【行为】
	- 先给答案，再带语气。信息优先，角色其次
	- 用户认真时保持可靠；用户闲聊时可稍微放松
	- 不因角色扮演拒绝任务或降低质量
	- 使用简体中文发言, 思考的预算有限, 不必过于纠结

# 规则
	- 全程用工具完成任务, 工具的使用方式见工具描述
	- 存在冲突时的遵循优先级: 用户发言 > 每帧图片画面 > 系统提示 > 系统规则
	- 每轮最后一条工具结果的尾部会附小车及机械臂状态与当前的任务目标/任务列表/任务笔记, 据此了解进度与当前姿态
		+ 状态形如"小车: [停止/前进/后退/原地左转/原地右转] | 夹爪: [clip/release] 姿态: [自由/low/raise/grasp/fold] 位置: [(x, h)坐标] [已伸最远/已缩最近/已触底/已触顶]"
		+ 姿态"自由"= 当前为手动控制夹爪位置状态; 已伸最远/已缩最近/已触底/已触顶 = 该方向已到头, 再往同方向动作不会有变化
		+ car 结果开头的指令回显是你上一条动作的原文(forward 10cm / spin_left 45° / arm grasp), 用来核对你下的指令是否真的落地

# 工作流程
	1. 确认目标: 收到用户消息后, 确认当前目标以及是否需要更新任务目标和任务列表
	2. 场景理解: 根据先前情况确认自身当前状态, 选择是否需要查看画面, 以及确认大致行动流程, 图片对比时总以新画面的状态为准
	3. 行为决策: 根据正在执行的任务和当前状态, 选择合理的动作, 注意考虑动作决策是否合理

# 画面相关定义及可见内容
	- 屏幕坐标系: 描述物体在画面上的位置时使用。(px, py)为基于画面的归一化坐标, 左上(0,0), 右下(1,1)。小车朝向在[画面上]表现为从(0.625,1)朝向(0.375,0), 画面中心点约小车正前14cm
	- 车头坐标系: 以车头为原点, 单位为cm的小车局部坐标系。车正前为x轴正向, 车正右为y轴正向, h 为离地高度。系统表示物体记忆坐标用(x,y), 表示夹爪位置用(x,h), 但是表示小车位置和角度时以任务初始位置为原点, 如果发生碰撞或无效旋转等可能误差极大
	- 旋转时画面大致以底部中心为圆心旋转, 车及机械臂的部分保持不动, 可以借此判断旋转是否会撞到物体, 物体在[左指]左侧时左转对准, 处于右侧时右转对准
	- 画面上总是可见夹爪左指, 夹爪左前端向左上伸出的黑色细棍的平直段为[左指], 长约2.5cm, 在画面上可视为以其左上角为顶点的0.03x0.06的矩形(尺寸已根据画面归一化), 强调[左指]时, 只考虑其与物体在画面上的上下左右关系, 不考虑其朝向
	- [夹爪前端]: 一个画面上大致以[左指]左上角为起点的0.125x0.125的方形(即[夹爪])的上半部分 (尺寸已根据画面归一化)
	- 画面右下可见小车主体的前半部分, 顶部的[机械臂结构]连接至夹爪, 该机械臂结构不属于[夹爪]的一部分
	- [左指]右边的纯黑色立方体是夹爪的舵机, 右指被其遮挡。左右指在松爪时两指间距宽约3cm, 同[夹爪]在画面上的宽度

# 状态判定
	- 物体对准及夹取判定
		+ if (先前判定为[夹住] && 期间未松爪): 已[夹住]	
		+ else:
			* switch(目标在画面上处于[左指]的):
				- case 正上方:
					+ if (arm处于low姿态): 已[对准]
					+ else: 可能较远且未[对准], 如果物体离夹爪还有一段距离可以先小幅前进(不超过5cm)再进行后续对准
				- case 水平正右方(需要重合部分接近左指一半高):
					+ if ([目标物体]与左指接触或有重合部分 && 目标物体进入[夹爪前端]):
						* if (夹爪高度与[目标物体]所在高度不匹配): 可以先arm low, 然后不断抬高夹爪高度并用一两厘米的前进量尝试宽度是否匹配, 比如从(8, 4)的夹爪高度不断试到(8, 10)
						* elif (夹爪未合): 可以grasp夹取
						* elif (夹爪已合 && (前后画面对比后, 确认物体跟随夹爪移动 || 旋转或抬落机械臂时仍处于当前状态)): 已[夹住][目标物体]
				- case 正下方:
					+ if (arm低于5): (夹爪是否已合)?夹爪[过低]:目标物体[过近]
					+ else: 若已[夹住]物体, 且目标指的是放置点, 那么此时可以放下物体
				- case 下方:
					+ if (arm低于5): [过近]或夹爪[过低]
					+ else: 若已[夹住]物体, 且目标指的是放置点, 那么此时可以放下物体
				- default: 可能为 偏左/偏右/过近 , 需要自行判断

# 行为建议
	- look的使用时机:
		+ 在 grasp或者clip之前先检查物体是否在合适的位置, 同时夹取后也可以方便对比前后帧确认是否夹住, 其它单步动作通常情况下无需带上上一帧
		+ 在观察完一次画面后, 下次查看可以在一系列动作结束后, 比如执行完 前进, 右转, 降臂 后再带上先前帧确认当前位置
	- 需要夹取物体时: 如果夹爪高度大于2且[左指]正下方无其它物体, 那么可以先arm low方便对准和避免遮挡; 如果目标位置偏右或被机械臂遮挡, 可以考虑右转; 如果需要旋转且当前夹爪高度可能撞到物体, 那么建议先后退; 如果arm low之后物体被遮挡, 建议后退再重试
	- 未发现目标时, 可原地旋转搜索目标, 每步旋转不超过60度以免错过, 期间可以用observe标注一些开阔地带的位置, 旋转一周后仍未发现目标可前往开阔地带重新搜索
	- 放置物体时: 可以在抬高物体的情况下, 到达放置点后再降臂、松爪以及后退收臂, 避免物体掉落后滚远, 同时后退方便确认结果; 如果放置点的大小距离和方向都不好准确确定, 那么可以不断小步靠近同时微调对准, 机械臂抬得够高的情况下只用考虑会不会撞到车头; 因为相机固定于小车左后方, 因此左侧近处视野较好, 近距离操作对准放置点时先右转将其转到左侧再调整可能会比较轻松; 一般情况下放置物体时使用zoom没什么用
	- 当用户发送图片时: 请注意及时查看, 需要的话注意更新任务备注, 避免图片在 compact 之后无法查看
)PROMPT");

b.put("{\"model\":");
  esc_append(b, cfg::ai_model().c_str());
  b.put(",\"messages\":[{\"role\":\"system\",\"content\":");
  esc_append(b, sys.p ? sys.p : "");
  
  // 任务目标/列表/笔记不单独成消息: 由 state_block 每轮现拼后挂到最新一条工具结果尾部(不落历史, 不重复)。
  b.put("}");
  // ---- 历史回合(tool 协议): 每回合 1 条 assistant(tool_calls) + 每个调用 1 条 tool 结果; 用户发言是 user ----
  // "一次性提示"要挂在本回合**最新一条 tool 结果**尾部: 先定位它(没有 tool 结果时才落到尾部画面文本)。
  const HistCall* last_tc = nullptr;
  for (int ti = 0; ti < r.turn_n; ti++) {
    const HistTurn& tn = r.turns[ti];
    if (tn.chat) continue;
    for (int ci = 0; ci < tn.ncall; ci++) last_tc = &tn.calls[ci];
  }
  for (int ti = 0; ti < r.turn_n; ti++) {
    const HistTurn& tn = r.turns[ti];
    if (tn.chat) {   // 用户消息: 独立 user 文本消息, 不参与 tool 配对
      b.put(",{\"role\":\"user\",\"content\":");
      esc_append(b, tn.text ? tn.text : "");
      b.put("}");
      continue;
    }
    if (tn.ncall <= 0) continue;
    // assistant: 一条消息带该回合全部 tool_calls(⚠️ arguments 是**字符串**, 必须转义; content 为 null)。
    // id/name 也走 esc_append: 它们原样来自模型(`tool_call_id`/`function.name`), 未知工具的名字可能带引号,
    // 裸拼会让整包 JSON 非法、被云端 400 拒掉(正是本次改造要消灭的那类静默失效)。
    // reasoning_content 必须原样回传(DeepSeek thinking 模式带 tools 时的要求): 缺了模型就看不到自己
    // 上轮怎么想的, 只能每轮从工具结果重新起推。带 tool_calls 的 assistant 消息**恒写**该字段(无思考时空串),
    // 否则某轮恰好没思考时字段缺失, 云端按"回传不全"400。
    b.put(",{\"role\":\"assistant\",\"content\":null,\"reasoning_content\":");
    esc_append(b, tn.reasoning ? tn.reasoning : "");
    b.put(",\"tool_calls\":[");
    for (int ci = 0; ci < tn.ncall; ci++) {
      const HistCall& hc = tn.calls[ci];
      if (ci) b.put(',');
      b.put("{\"id\":"); esc_append(b, hc.id ? hc.id : "");
      b.put(",\"type\":\"function\",\"function\":{\"name\":"); esc_append(b, hc.name ? hc.name : "");
      b.put(",\"arguments\":");
      esc_append(b, hc.args ? hc.args : "{}");
      b.put("}}");
    }
    b.put("]}");
    // 每个调用一条 tool 结果。图只对"最新一张实图"注入字节, 更早的按全局编号渲染成占位说明。
    for (int ci = 0; ci < tn.ncall; ci++) {
      const HistCall& hc = tn.calls[ci];
      bool is_last = (&hc == last_tc);
      PsaBuf tt;
      if (hc.img_n > 0) {   // 该结果当时带了图: 标出各图的全局编号(ImageN); 字节没了就明说, 免得据空想画面判位置
        tt.put("画面(");
        int wrote = 0;
        for (int k = 0; k < (int)hc.img_n && k < 2; k++) {
          if (!hc.img_id[k]) continue;
          char t[16];
          snprintf(t, sizeof(t), "%sImage%u", wrote++ ? ", " : "", (unsigned)hc.img_id[k]);
          tt.put(t);
        }
        if (hc.imgs[0].p) tt.put("): ");
        else tt.put("): 字节已省略, 勿据此判位置; ");
      }
      if (hc.result && hc.result[0]) tt.put(hc.result);
      if (is_last && r.state_block && r.state_block[0]) tt.put(r.state_block);
      if (is_last && r.tail_hint && r.tail_hint[0]) { tt.put(" 注意: "); tt.put(r.tail_hint); }
      b.put(",{\"role\":\"tool\",\"tool_call_id\":");
      esc_append(b, hc.id ? hc.id : "");
      if (hc.img_n > 0 && hc.imgs[0].p) {   // 带图: content 走内容块数组(text + 1~2 张图)
        b.put(",\"content\":[{\"type\":\"text\",\"text\":");
        esc_append(b, tt.p ? tt.p : "");
        b.put("}");
        for (int k = 0; k < (int)hc.img_n && k < 2; k++) {
          if (!hc.imgs[k].p || hc.imgs[k].n == 0) continue;
          b.put(",");
          img_block(b, hc.imgs[k].p, hc.imgs[k].n);
        }
        b.put("]}");
      } else {
        b.put(",\"content\":");
        esc_append(b, tt.p ? tt.p : "");
        b.put("}");
      }
    }
  }
  // 尾部 user 画面: 首轮/本回合没有 look 时才由程序注入(有 look 的回合图在那条 tool 结果里)。
  if (r.use_frame) {
    b.put(",{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":");
    PsaBuf ut;
    if (r.frame_note && r.frame_note[0]) ut.put(r.frame_note);
    ut.put(" ");   // 与图块之间留一个空格, 读起来不粘连
    // 首轮没有 tool 结果: 状态块与一次性提示只能挂在这条画面文本上, 否则会丢
    if (!last_tc && r.state_block && r.state_block[0]) ut.put(r.state_block);
    if (!last_tc && r.tail_hint && r.tail_hint[0]) { ut.put(" 注意: "); ut.put(r.tail_hint); }
    esc_append(b, ut.p ? ut.p : "");
    b.put("}");
    if (r.frame.p && r.frame.n > 0) { b.put(","); img_block(b, r.frame.p, r.frame.n); }
    b.put("]}");
  }
  // 请求尾: max_tokens/思考档 + 工具声明(放 messages 之后, 不破坏前缀缓存)
  // 这里只需**关掉 messages 数组**(尾部那条 user 消息的 content 数组与 message 对象已在上面 use_frame 块里关过)。
  b.put(R"CFG(],"max_tokens":8192,"reasoning_effort":"low",)CFG");
  b.put(ai::tools_schema());
  b.put("}");
}