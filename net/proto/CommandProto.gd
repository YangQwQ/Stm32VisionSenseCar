extends RefCounted
class_name CommandProto
## 统一命令词表：手机手动控制 / 手机中转审批 / 小车直连 统一经此编解码。

static func move(throttle: float, steering: float) -> Dictionary:
	return {"type": "move", "params": {"throttle": throttle, "steering": steering}, "id": _new_id()}

static func stop(scope: String = "all") -> Dictionary:
	# scope: all / wheels / arm
	return {"type": "stop", "params": {"scope": scope}, "id": _new_id()}

static func arm(act: String, duration_ms: int = 0) -> Dictionary:
	# act: lift_up / lift_down / clip / release / reach_forward / reach_backward
	# duration_ms == 0 表示持续移动，直到收到 stop(scope="arm")
	return {"type": "arm", "params": {"act": act, "duration_ms": duration_ms}, "id": _new_id()}

static func snapshot(quality: int = 82) -> Dictionary:
	return {"type": "snapshot", "params": {"quality": quality}, "id": _new_id()}

static func stream(on: bool) -> Dictionary:
	return {"type": "stream", "params": {"on": on}, "id": _new_id()}

static func config_wifi(ssid: String, password: String) -> Dictionary:
	return {"type": "config", "params": {"ssid": ssid, "password": password}, "id": _new_id()}

static func ping() -> Dictionary:
	# 手动连通性测试（/ping）。无周期心跳。
	return {"type": "ping", "params": {}, "id": _new_id()}

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

## /help 文案：可用指令说明（仅供本地展示，不下发板子）。
static func help_lines() -> PackedStringArray:
	return PackedStringArray([
		"/ping  连通性测试",
		"/snapshot  截图",
		"/stream [on|off]  图传开关",
		"/stop [wheels|arm]  停车",
		"/config <WiFi名> <密码>  配网",
		"/goal <目标>  下发 AI 目标(DIRECT)",
		"/cancel  取消当前 AI 任务",
		"/ws connect|disconnect|status  WS 手动连接/断开/状态",
		"直接输入文字 = 以下发 AI 目标; 框选后发文字 = 带区域目标",
	])

## 指令提示表：完整指令（语法） → 说明。/help 与输入 / 时的匹配提示共用。
const COMMAND_HINTS := {
	"/ping": "连通性测试",
	"/snapshot": "截图",
	"/stream [on|off]": "图传开关",
	"/stop [wheels|arm]": "停车",
	"/config <WiFi名> <密码>": "配网",
	"/goal <目标>": "下发 AI 目标(DIRECT)",
	"/cancel": "取消当前 AI 任务",
	"/ws [connect|disconnect|status]": "WS 手动连接/断开/状态",
}

## 按已敲的 / 指令片段过滤可匹配项，返回"指令 - 说明"行。text 以 / 开头才匹配；删到空则返回空。
static func command_hints(text: String) -> PackedStringArray:
	var t := text.strip_edges()
	if not t.begins_with("/"):
		return PackedStringArray()
	var tok := t.trim_prefix("/").strip_edges().to_lower()
	var out := PackedStringArray()
	for cmd: String in COMMAND_HINTS.keys():
		var verb := cmd.to_lower().split(" ", true, 1)[0].lstrip("/")
		if tok.is_empty() or verb.begins_with(tok):
			out.append("%s - %s" % [cmd, COMMAND_HINTS[cmd]])
	return out

static func raw(type: String, params: Dictionary = {}) -> Dictionary:
	# 通用出口：给聊天 / 解析到未预置 builder 的词表指令透传用
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