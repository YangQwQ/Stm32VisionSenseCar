extends RefCounted
class_name CommandProto
## 统一命令词表：手机手动控制 / 手机中转审批 / 小车直连 统一经此编解码。

static func move(throttle: float, steering: float) -> Dictionary:
	return {"type": "move", "params": {"throttle": throttle, "steering": steering}, "id": _new_id()}

static func stop(scope: String = "all") -> Dictionary:
	# scope: all / wheels / arm
	return {"type": "stop", "params": {"scope": scope}, "id": _new_id()}

static func arm(act: String, duration_ms: int = 0) -> Dictionary:
	# act: lift_up / lift_down / clip / release / reach_forward / reach_backward / fold(收臂折叠回平台)
	#      / low(降到板端标定的贴地夹取准备位，再靠挪车把目标送进两指之间)
	# 持续型只有 lift_up/lift_down/reach_forward/reach_backward；其余为一次性离散动作。
	# duration_ms == 0 表示持续移动，直到收到 stop(scope="arm")
	return {"type": "arm", "params": {"act": act, "duration_ms": duration_ms}, "id": _new_id()}

static func stream(on: bool, udp_port: int = 0, src_ip: String = "") -> Dictionary:
	# udp_port：图传走 UDP 时手机本地接收端口（>0 才带上）；src_ip：手机本机 IP（板子据此建 UDP 会话，
	# 因实测板端 lwip_getpeername 对 httpd fd 取 peer 会回 0.0.0.0，依赖上报更可靠）。
	var params: Dictionary = {"on": on}
	if udp_port > 0:
		params["udp_port"] = udp_port
	if not src_ip.is_empty():
		params["src_ip"] = src_ip
	return {"type": "stream", "params": params, "id": _new_id()}

static func log_switch(cat: String, on: bool) -> Dictionary:
	# 统一日志转发开关：/log <exec|ai|all> on|off。默认全关。
	#   exec = 直驱执行日志+周期状态推送；ai = AI 调试日志；all = 板端串口全部输出转发手机。
	return {"type": "log", "params": {"cat": cat, "on": on}, "id": _new_id()}

static func get_state() -> Dictionary:
	# 主动查询当前状态（车灯/夹爪等），用于重连后同步控制按钮。板端回 {"type":"state",params:{...}}。
	return {"type": "get_state", "params": {}, "id": _new_id()}

static func nz_read() -> Dictionary:
	# 诊断：让板子经 I2C 探测哪吒从机是否在线，回执写/读 ACK 与读到的首字节。
	# 用于区分「从机/总线问题」vs「软件控制问题」；仅探测，不改从机状态。
	return {"type": "nz_read", "params": {}, "id": _new_id()}



static func config_wifi(ssid: String, password: String) -> Dictionary:
	return {"type": "config", "params": {"ssid": ssid, "password": password}, "id": _new_id()}

static func ping(target: String = "") -> Dictionary:
	# target 为空 = 测小车连通性（板子直接回 pong）；带目标（IP/域名）= 板子去 ping 并回报延迟。
	var params: Dictionary = {}
	if not target.is_empty():
		params["target"] = target
	return {"type": "ping", "params": params, "id": _new_id()}

static func reboot() -> Dictionary:
	# 远程重启板子：链路卡死（WiFi/IP 死、BLE 还活着）时唯一能远程按下的那一下。
	# 板端先回 status「正在重启…」再延时重启；同一句另经日志转发广播（所有 WS 客户端 + BLE），
	# 所以旁观的手机也能看到，不依赖发起方。注意板端升级期间会拒收此指令。
	return {"type": "reboot", "params": {}, "id": _new_id()}

static func ai_goal(message: String, annotation: Dictionary = {}, use_image: bool = false) -> Dictionary:
	# DIRECT 链路目标下发：手机 → 板子。annotation 为可选圈选区域 {x,y,w,h,label}（坐标相对手机画面）。
	# use_image=true 表示"已先上行编辑图（WS 二进制），板侧以其为意图锚点"。
	var params: Dictionary = {"message": message}
	if not annotation.is_empty():
		params["annotation"] = annotation
	if use_image:
		params["use_image"] = true
	return {"type": "ai_goal", "params": params, "id": _new_id()}

static func ai_cancel() -> Dictionary:
	# 取消当前 AI 任务（板侧须手动/新目标也能中止；此指令离线经 BLE 兜底也可用）。
	return {"type": "ai_cancel", "params": {}, "id": _new_id()}

static func ai_chat(message: String) -> Dictionary:
	# AI 任务进行中「插话」：把补充文本喂给正在运行的任务，不打断（区别于 ai_goal 的接管）。
	# 无任务在跑时板端忽略并回执提示。非 BLACKLISTED 类型，支持 BLE 兜底。
	return {"type": "ai_chat", "params": {"message": message}, "id": _new_id()}

static func ai_oneshot(message: String, annotation: Dictionary = {}, use_image: bool = false) -> Dictionary:
	# 单轮 AI：板侧只执行一次决策即自动收尾（区别于 ai_goal 的迭代闭环）。参数同 ai_goal。
	var params: Dictionary = {"message": message}
	if not annotation.is_empty():
		params["annotation"] = annotation
	if use_image:
		params["use_image"] = true
	return {"type": "ai_oneshot", "params": params, "id": _new_id()}

static func light(kind: String, on: bool) -> Dictionary:
	# 直驱灯光：kind = front/vibe/back → 前灯/氛围灯/尾灯(左右一起)
	return {"type": "light", "params": {"kind": kind, "on": on}, "id": _new_id()}

static func reset() -> Dictionary:
	# 回正：转向+机械臂四个舵机全部回中+电机停（直驱专用）
	return {"type": "reset", "id": _new_id()}

static func servo(n: int, pwm: int) -> Dictionary:
	# 调试直驱：大脑板直接驱动哪吒舵机（n=0转向/1左(前后)/2右(抬落)/3前(夹爪), pwm=50..250）
	return {"type": "servo", "params": {"n": n, "pwm": pwm}, "id": _new_id()}

static func arm_pose(x: float, h: float) -> Dictionary:
	# 二连杆 IK：末端位姿(x=轴前方cm, h=地面以上cm) → 大脑板联动算左右两舵机 pwm
	return {"type": "arm_pose", "params": {"x": x, "h": h}, "id": _new_id()}

static func motor(n: int, a: int, b: int) -> Dictionary:
	# 调试直驱：绕过执行板，大脑板直接驱动哪吒单轮电机（n=1..4, a=正转 b=反转, 0..1000）
	return {"type": "motor", "params": {"n": n, "a": a, "b": b}, "id": _new_id()}

static func drive(speed: int) -> Dictionary:
	# 调试直驱：一键全车前进/后退/停（单条命令；speed=-1000..1000, 0=停）
	return {"type": "drive", "params": {"speed": speed}, "id": _new_id()}

static func spin(dir: int, speed: int = 500, angle_deg: int = 0) -> Dictionary:
	# 原地转向：普通四轮滑移式。dir=+1左进右退 / -1左退右进 / 0停；speed=单轮pwm 0..1000。
	# angle_deg>0 时按板端时长近似"转指定度数"到点自停（无里程计，粗略，供微操/标定）。
	# 需要转向舵回正前轮直行才转得正。
	var params: Dictionary = {"dir": dir, "speed": speed}
	if angle_deg > 0:
		params["angle_deg"] = angle_deg
	return {"type": "spin", "params": params, "id": _new_id()}

static func move_dist(throttle: float, cm: int, steering: float = 0.0) -> Dictionary:
	# 定距移动：油门+距离cm，板端按时长近似到时自停（无里程计，粗略，供微操/标定）。
	return {"type": "move", "params": {"throttle": throttle, "steering": steering, "distance_cm": cm}, "id": _new_id()}

static func goto(x: float, y: float, frame: String = "local") -> Dictionary:
	# 本地巡航到坐标（板端不调 AI 自动执行）：frame=local（默认，原点=当前位姿，y向前 x向右）/
	# global（沿用全局系）。无里程计开环，近点到停，用于测导航。
	var params: Dictionary = {"x": x, "y": y}
	if frame == "global":
		params["frame"] = "global"
	return {"type": "goto", "params": params, "id": _new_id()}

## /help 文案：由 COMMAND_HINTS 生成（唯一事实源，避免重复维护）；特殊说明在此追加。
static func help_lines() -> PackedStringArray:
	var out := PackedStringArray()
	for cmd: String in COMMAND_HINTS.keys():
		out.append("%s   %s" % [cmd, COMMAND_HINTS[cmd]])
	out.append("直接输入文字 = 以下发 AI 目标; 框选后发文字 = 带区域目标")
	return out

## 指令提示表：完整指令（语法） → 说明。/help 与输入 / 时的匹配提示共用。
const COMMAND_HINTS := {
	"/ping [IP|域名]": "连通性测试（不带参数=测小车）",
	"/clear": "清空消息区(仅本机)",
	"/stream [on|off]": "图传开关",
	"/grid [on|off]": "图传叠加标定网格（本地，不下发板子）",
	"/stop [wheels|arm]": "停车",
	"/log <exec|ai|all> [on|off]": "统一日志转发开关（默认关；exec=执行日志+周期状态, ai=AI日志, all=板端全部输出）",
	"/nz_read": "I2C诊断：探测哪吒从机在线状态(写/读ACK)",
	"/reboot": "重启板子(链路卡死时的解药; 升级中会被拒)",
	"/light <front|vibe|back> <0|1>": "直驱灯开关(前/氛围/尾)",
	"/config <WiFi名> <密码>": "配网",
	"/ai [goal|oneshot|cancel] <目标>": "AI 目标 / 单轮 / 取消",
	"/snapshot": "保存当前图传画面(本地)",
	"/append": "从图库选一张图，标注后作为附件",
	"/ws [connect [IP]|disconnect|status]": "WS 手动连接/断开/状态",
	"/connect <IP>": "不经蓝牙直连 WS",
	"/move rotate <dir=-1/0/1>": "转向舵三档(左/回正/右)",
	"/move spin <角度> [speed]": "原地旋转(正=右转, 负=左转; 0=停; 板端时长近似到点自停)",
	"/move fore|back <距离cm> [油门%]": "定距前进/后退(时长近似到点自停)",
	"/move to <x> <y> [global]": "板端本地导航到坐标(local原点=当前位姿,y向前x向右; global=沿用全局系)",
	"/move arm <x> <h>": "机械臂末端到指定位姿(车头系前方cm, 离地高度cm)",
	"/drive motor <n=1..4|0=all> <pwm>": "直驱单轮电机(n=0 全车drive)",
	"/drive servo <n=0转向/1左/2右/3前> <pwm=50..250>": "直驱舵机(可超标定限位)",
	}

## 指令提示最多展示条数（超出截断，避免挡住聊天区）。
const MAX_HINTS := 10

## 按已敲的 / 指令片段过滤可匹配项，返回"指令 - 说明"行。text 以 / 开头才匹配；删到空则返回空。
## 只按空格前的指令词匹配：打 /ping 1.2.3.4 这类带参数输入时依旧能匹配到 /ping。
static func command_hints(text: String) -> PackedStringArray:
	var t := text.strip_edges()
	if not t.begins_with("/"):
		return PackedStringArray()
	# 只按空格前的指令词匹配：打 /ping 1.2.3.4 这类带参数输入时依旧能匹配到 /ping。
	# 只打 / 时 tok 为空 → 显示全部（按 COMMAND_HINTS 定义顺序）。
	var rest := t.trim_prefix("/")
	var tok := rest.split(" ", false)[0].to_lower() if not rest.is_empty() else ""
	var out := PackedStringArray()
	for cmd: String in COMMAND_HINTS.keys():
		var verb := cmd.to_lower().split(" ", true, 1)[0].lstrip("/")
		if tok.is_empty() or verb.begins_with(tok):
			out.append("%s - %s" % [cmd, COMMAND_HINTS[cmd]])
			if out.size() >= MAX_HINTS:
				break
	return out

static func raw(type: String, params: Dictionary = {}) -> Dictionary:
	# 通用出口：未预置 builder 的词表指令透传用。
	return {"type": type, "params": params, "id": _new_id()}

static func encode(cmd: Dictionary) -> String:
	return JSON.stringify(cmd)

static func decode(text: String) -> Dictionary:
	var data = JSON.parse_string(text)
	return data if data is Dictionary else {}

static var _seq := 0

static func _new_id() -> int:
	_seq += 1
	return _seq
