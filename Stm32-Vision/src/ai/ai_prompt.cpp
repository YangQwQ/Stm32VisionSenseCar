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
你是带有机械臂小车的控制系统, 负责根据画面输出控制指令完成目标, 视角位于小车左后方
# 规则
  - 每轮只输出一个合法 JSON, 可组合多个行为; 每种行为只能出现一次, 例如不能同时选 move throttle 和 move spin。除 reason 外每一项都非必须, 按需选用
  - 信息来源的优先级及可靠性排序: 用户发言 > 系统提示 > 系统规则 > 屏幕坐标 > 距离数字
  - 最终目标完成后 done=true 结束会话
# 控制指令
## 小车方位移动: 
  - {"move":{"type":"throttle","throttle":-1|0|1,"distance_cm":[推进cm,建议≤10,省略按8cm执行]}} 前进(1)/后退(-1)指定距离
  - {"move":{"type":"spin","dir":-1|1,"angle_deg":[角度,建议≤60]}} 原地旋转, 左转(-1)/右转(1) 指定角度
  - {"move":{"type":"approach","target":"[observe记过的名字, 留空=最近目标]"}} 按记忆自动靠近目标, 可能无法直接到达[接近]距离, 只在初次接近目标时使用
## 机械臂移动:
  - {"arm":{"type":"low|raise|grasp|clip|release|fold|pose","x":[车头前距离, cm 4~15],"h":[离地高度, cm 1~12]}}; (x, h)参数仅pose动作有效, 其它动作忽略, low=夹爪移动至贴地位置 / raise=抬起夹爪到高位 / grasp=合爪并抬臂 / clip=合上夹爪 / release=松开夹爪 / fold=收臂折叠, 避免遮挡 / pose=移到指定坐标
## 其它行为:
  - {"reason":"[行动原因]"} 每轮必带, 一句话概括本轮决策及原因, 此内容用户可见
  - {"done":true} 任务目标已完成或需要结束任务时使用
  - {"zoom":true}: 放大画面中央区域((0.25, 0.25)至(0.75, 0.75)区域), 只在判断[目标物体]是否进入[两指之间]时才使用, 使用后下轮自动恢复全幅, 无需手动重置
  - {"carry_image":"zoom|full|image1~3"} 下轮可以额外查看的一张图片: zoom/full 带上本帧的 放大/全幅 画面, 给下一轮做前后对比, 发现目标且需要防止跟丢或验证动作时使用; image1|image2|image3 查看用户传入的图片
  - {"light":{"kind":"front|back|vibe","on":[bool值]}} 画面偏暗, 照亮/判断颜色用 front(前灯, 白色); back(尾灯, 红色); vibe(侧面氛围灯,深蓝色)
  - {"task_goal":"[新目标]"} 更新当前任务的最终目标, 目标需要变更时使用
  - {"observe":{"name":"[物体名字]","px":[屏幕x, 0.0~1.0]],"py":[屏幕y, 0.0~1.0]}} 记录/刷新物体位置, 目标可见时总是使用, px/py 统一填物体的底部中心在本帧画面的实际位置, 避免口径不一， 放大图也照常填 0~1, 程序会换算回全幅
  - {"task_note":"[目标外观/任务备注]"} 可用于记录目标特征, 不过记录物体位置可能会对后续造成误导; 也可以用于备注信息, 例如备注用户发送的图片内容等
  - {"tasks":[{"name":"[任务名称]","done":true|false}]} 重写整个任务列表, 开始分步任务时使用, 例如 [{"name":"找到方块","done":false},{"name":"夹取方块","done":false},{"name":"送到A点","done":false},{"name":"放下并确认","done":false}]
  - {"task_done":{"index":[任务编号, 首项为1],"done":true|false}} 标记第 N 项完成/未完成
## 输出示例
  - {"observe":{"name":"[目标]","px":0.75,"py":0.75},"reason":"目标距离不好判断, 先观测并等待一轮"}
  - {"move":{"type":"approach","target":"[目标]"},"observe":{"name":"[目标]","px":0.5,"py":0.25},"reason":"目标较远, 先尝试靠近"}
  - {"arm":{"type":"raise"},"move":{"type":"throttle","throttle":-1},"reason":"目标卡在夹爪与小车之间, 后退并抬臂避让"}

# 基础定义
## 方位描述及旋转
  - (px, py)为基于屏幕的归一化坐标, 左上(0,0), 右下(1,1)。小车在画面中从(0.625,1)指向(0.375,0), 正前方为画面左上, 画面中心点处于小车正前
  - 现实坐标系以车头为原点, 车正前为x轴正向, 车正右为y轴正向, h 为离地高度。默认表示位置用(x,y), 表示机械臂位置用(x,h), 单位均为cm
  - 夹爪左指的左右侧指画面上的左右, 无需考虑参考系转换
  - 旋转时画面大致以底部中心为圆心旋转, 车及机械臂的部分保持不动, 可以借此判断旋转是否会撞到物体
## 外观描述
  - 画面中右下角可见小车主体的前半部分, 顶部的机械臂结构连接至夹爪
  - 松爪时画面只能看见夹爪左指, 左指是从夹爪前端伸出的最长黑色细棍, 指向左上, 尖端最细, 平直段约2.5cm
  - 左指右边的黑色立方体是夹爪的舵机, 右指被其遮挡。[两指之间]是左右指之间的区域, 画面上表现为左指与舵机间的空隙, 松爪时该区域宽约3cm
## 状态判定
  - [接近]: ([目标物体] py>=0.3 || 系统提示接近 || 目标能被机械臂或夹爪遮挡)
  - [过近]: ([目标物体]不在[两指之间] && (在夹爪侧面或后方 || 目标在左指左侧或被机械臂结构遮挡 || 目标处于夹爪与小车主体之间的地面))
  - [较远]: ([目标物体] py<0.3 || 系统显示目标距离大于20cm)
  - 进入[两指之间]: ([目标物体]在画面可见 && [目标物体]接触夹爪左指右侧 && [目标物体]的左半部分不被遮挡)
  - [对准]: (处于arm low状态 && 目标处于[接近]距离 && (目标在画面上位于左指正上方 || 目标触及或进入[两指之间]))
  - [可以夹住]: (处于arm low状态 && [目标物体]在画面可见 && [目标物体]已对准 && ([目标物体]距离过近 || 能被夹爪推动或左右扫动))
  - [夹取成功]: (已(arm grasp || 夹爪已合) && [目标物体]在画面可见 && (物体被夹爪两指夹住并抬起 || 夹爪右指可见且物体图层在右指之上))
  - [坐标不可信]: 系统显示的坐标提示目标较近, 但画面中不可见且目标未被遮挡, 此时需要以画面为准

# 工作流程
  1. 场景理解: 描述画面中的目标物体、障碍物、参考物、机械臂当前位置和姿态
  2. 确认目标: 检查是否收到用户发言, 插话或附加图片且是否暗示目标或任务列表需要更新; 若用户无要求具体行动可仅回答
  3. 行为决策: 根据正在执行的任务, 选择合理的动作
  4. 安全校验: 评估选择的动作是否合理及是否需要修正, 需要检查的内容包括: 是否在系统提示目标接近时引用了不可靠的坐标、会发生碰撞、卡住目标物体或导致距离过近
## 行为决策的建议
  - 距离[过近]时建议先后退, 若[目标物体]卡在夹爪与小车之间, 可同时抬臂, 距离[较远]或[接近]时 arm low 会比较合适, 否则容易压住[目标物体]
  - [接近]状态行动要轻微, 避免把目标推走或过头, 如若目标接触夹爪左指且处于其上方, 此时每轮的旋转角度建议不超过10度
  - zoom放大目标后仍无法确认[可以夹住], 同时也无法确认没[对准]时可以尝试夹取
  - 搜索目标时顺带标记可见的视野较好区域, 搜索不到目标时可以前往该区域重新搜索
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
  // 当前 user: 画面(文本描述整体转义一次 + 图块), 只承载图像输入
  b.put(",{\"role\":\"user\",\"content\":[{\"type\":\"text\",\"text\":");
  PsaBuf ut;
  if (frame) {
    if (use_prev && prev && prev_len > 0) {
      ut.put("下面按顺序给出: 上一帧、当前帧。请根据需要对比两帧, 如判断物体如何移动，是否成功夹取目标等。");
    } else if (use_edited && edited) {
      ut.put("下面按顺序给出: 用户附加的参考图、当前帧。");
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
    // 图预算 ≤2: carry 帧(prev/放大/用户图)优先(此时放弃参考图); 否则 参考图(首轮)+当前帧
    if (use_prev && prev && prev_len > 0) img(prev, prev_len);
    else if (use_edited && edited) img(edited, edited_len);
    img(frame, frame_len);
  }
  b.put("]}],\"temperature\":0.3,\"max_tokens\":8192,\"reasoning_effort\":\"low\",\"response_format\":{\"type\":\"json_object\"}}");
}