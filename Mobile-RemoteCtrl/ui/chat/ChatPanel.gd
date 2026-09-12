extends VBoxContainer
## 聊天区视图：消息日志、指令提示、待上传图片列表、输入发送。
## 职责（Main 只做连接编排，聊天区逻辑全部收口在此）：
##   - 消息日志展示（chat）
##   - 指令提示（最多 MAX_HINTS 条，只按空格前的指令词匹配）
##   - 附件列表（编辑图最多 3 张，超限自动顶掉最早一张；删除走 delBtn）
##   - 附件与指令提示共用底部区域：有图时优先显示图片，指令提示隐藏
##   - 指令解析（/ai、/ping、/ws、/connect 等）与发送
## 与 Main 的交互：图传开关等旁路动作经 stream_requested 信号交给 Main 统一处理。

signal stream_requested(on: bool)
signal grid_requested(on: bool)

const CP := preload("res://net/proto/CommandProto.gd")
const MAX_IMAGES := 3

## 底部面板（TabContainer）两种内容各自需要的面板高度（anchored 于 ChatLog 底部）。
const _HINT_AREA_H := 43.0
const _IMAGE_AREA_H := 310.0

@onready var _chat_log: RichTextLabel = $ChatLog
@onready var _bottom: TabContainer = $ChatLog/BottomPanl
@onready var _cmd_hint: Label = $ChatLog/BottomPanl/CommandHint
@onready var _message_input: LineEdit = $InputRow/MessageInput
@onready var _send_btn: Button = $InputRow/SendBtn
@onready var _panels: Array = [
	$ChatLog/BottomPanl/ImageList/Image1,
	$ChatLog/BottomPanl/ImageList/Image2,
	$ChatLog/BottomPanl/ImageList/Image3,
]

## AI 任务执行中：发送按钮切换为「中止」（急停），按下打断任务并停车。
var _ai_running := false

## 待上传的编辑图附件 [{image: Image, annotation: Dictionary}]，按加入顺序展示。
var _attachments: Array = []
## 输入历史（仅纯文本）：上/下键翻阅，_history_idx 指向当前展示项。
## _history_idx == size() 表示停在"当前草稿位"；_draft 存首次上翻前未发送的输入，供下键恢复。
var _input_history: PackedStringArray = []
var _history_idx: int = -1
var _draft: String = ""

func _ready() -> void:
	# 删除按钮在脚本里连接（tscn 里逐个连太啰嗦）。
	for i in _panels.size():
		_panels[i].get_node("delBtn").pressed.connect(_on_del_pressed.bind(i))
	set_process_input(true)
	_refresh_bottom()

# ============================== 消息日志（Main 外部调用） ==============================

## 追加一条聊天消息。who 取值：本机 / 板 / AI / 提示 / 执行板。
func chat(who: String, msg: String) -> void:
	if who == "本机":
		_chat_log.append_text("[b]本机[/b]: %s\n" % msg)
	elif who == "板":
		_chat_log.append_text("[color=#6fc3ff]小车[/color]: %s\n" % msg)
	elif who == "AI":
		_chat_log.append_text("[color=#c9f7a8]AI[/color]: %s\n" % msg)
	elif who == "AI日志":
		_chat_log.append_text("[color=#9ad0ff]AI日志[/color]: %s\n" % msg)
	elif who == "提示":
		_chat_log.append_text("[color=#ffd75e]系统[/color]: %s\n" % msg)
	elif who == "执行板":
		_chat_log.append_text("[color=#b39ddb]执行板[/color]: %s\n" % msg)
	else:
		_chat_log.append_text(msg + "\n")
	AppLog.write(who, msg)  # 聊天区出现的内容统一落盘（启动已建好文件）

## 展示一条 ai_result：{type:"ai_result", id, params:{error?, reason?, done?, command:{type,params,reason}}}。
func show_ai_result(data: Dictionary) -> void:
	var params: Variant = data.get("params")
	# 仅终轮（done:true）复位按钮：多轮任务每轮都回 ai_result，中途复位会让「中止」失效。
	var is_final := false
	if params is Dictionary:
		var dv: Variant = (params as Dictionary).get("done")
		is_final = dv is bool and (dv as bool)
	if is_final:
		set_ai_running(false)
	var line := ""
	if params is Dictionary:
		var pd: Dictionary = params as Dictionary
		var e: Variant = pd.get("error")
		if e is String and not (e as String).is_empty():
			line = "错误：%s" % (e as String)
		else:
			var r: Variant = pd.get("reason")
			if r is String and not (r as String).is_empty():
				line = r as String
			var inner: Variant = pd.get("command")
			if inner is Dictionary:
				var cs := _cmd_text(inner as Dictionary)
				if cs != "":
					line = "%s → %s" % [line, cs] if line != "" else cs
			var dv: Variant = pd.get("done")
			if dv is bool and (dv as bool):
				line = "%s(任务结束)" % line if line != "" else "任务结束"
	if line == "":
		line = "已收到 AI 输出"
	chat("AI", line)

func _cmd_text(cmd: Dictionary) -> String:
	var t: String = str(cmd.get("type", ""))
	var p: Variant = cmd.get("params")
	var parts := PackedStringArray()
	if p is Dictionary:
		for k: Variant in (p as Dictionary).keys():
			parts.append("%s=%s" % [str(k), str((p as Dictionary).get(k))])
	return "%s (%s)" % [t, ", ".join(parts)] if parts.size() > 0 else t

# ============================== 底部区域（指令提示 / 图片列表） ==============================
# 两者共用 ChatLog 底部的 BottomPanl：有附件时切到图片页（更高），否则按 / 指令显示提示。

func _refresh_bottom() -> void:
	if not _attachments.is_empty():
		# 有图优先显示图片，不显示指令提示
		_bottom.visible = true
		_bottom.current_tab = 1
		_bottom.offset_top = -_IMAGE_AREA_H
		return
	var lines := CP.command_hints(_message_input.text)
	if lines.is_empty():
		_bottom.visible = false
		return
	_cmd_hint.text = "\n".join(lines)
	_bottom.visible = true
	_bottom.current_tab = 0
	_bottom.offset_top = -_HINT_AREA_H

# ============================== 附件（图片列表） ==============================

## 编辑器「采用」：把编辑图作为附件加入列表（最多 3 张，超限顶掉最早一张）。
## 指令不能和图片一起发：若输入框正打着 / 指令，清空输入。
func _on_image_sent(img: Image, annotation: Dictionary) -> void:
	if img == null:
		chat("提示", "未能导出编辑图（无可用画面），请先框选再采用")
		return
	if _attachments.size() >= MAX_IMAGES:
		_attachments.pop_front()
	_attachments.append({"image": img, "annotation": annotation})
	# /ai 指令可与图片搭配（AI 需要参考画面），保留输入；其余 / 指令与图冲突，清空。
	if _message_input.text.begins_with("/") and not _message_input.text.begins_with("/ai"):
		_message_input.text = ""
	_render_images()
	_refresh_bottom()
	_message_input.grab_focus()

func _on_del_pressed(index: int) -> void:
	if index < _attachments.size():
		_attachments.remove_at(index)
	_render_images()
	_refresh_bottom()
	_message_input.grab_focus()

## 把附件列表渲染进 3 个图片槽位（Image1..3，按加入顺序），多余的槽位隐藏。
func _render_images() -> void:
	for i in _panels.size():
		var panel: Panel = _panels[i]
		if i < _attachments.size():
			var att: Dictionary = _attachments[i]
			var img: Image = att.get("image")
			var tex: TextureRect = panel.get_node("img")
			tex.texture = ImageTexture.create_from_image(img)
			panel.visible = true
		else:
			panel.visible = false

# ============================== 输入 / 发送 ==============================

func _on_input_text_changed(_new_text: String) -> void:
	_refresh_bottom()

func _on_send_pressed(_new_text: String = "") -> void:
	# AI 任务执行中：按钮语义已切换为「中止」，按下即打断任务并急停，不发输入内容。
	if _ai_running:
		_abort_ai()
		return
	if _message_input.text.strip_edges().is_empty() and _attachments.is_empty():
		return
	if not _attachments.is_empty():
		var plain: String = _message_input.text.strip_edges()
		# 仅 /ai 指令支持带图；其他 / 指令与图冲突，提示后保留输入与附件。
		if plain.begins_with("/") and not plain.begins_with("/ai"):
			chat("提示", "该指令不支持带图（图片仅支持 /ai 或直接输入文字目标）")
			return
		var batch: Array = _attachments
		_message_input.text = ""
		_attachments = []
		_render_images()
		_refresh_bottom()
		if plain.begins_with("/ai"):
			_send_ai_with_images(batch, plain)
		else:
			_send_image_goal(batch, plain)
		return
	var text: String = _message_input.text.strip_edges()
	_message_input.text = ""
	_push_to_history(text)
	_refresh_bottom()  # 输入清空后指令提示隐藏（无图时）
	if text.begins_with("/"):
		_handle_slash(text)
	else:
		_send_ai_goal(text)

## 切换 AI 执行中状态，同步发送按钮文字（发送 ↔ 中止）。Main 摇杆手动接管时也会调用。
func set_ai_running(run: bool) -> void:
	if _ai_running == run:
		return
	_ai_running = run
	_send_btn.button_pressed = run
	_send_btn.text = "中止" if run else "发送"

## 中止 AI：打断进行中的任务并急停（板端 ai_cancel 打断闭环并补停残留持续指令）。
func _abort_ai() -> void:
	chat("本机", "/ai cancel（中止）")
	if not AppState.send_command(CP.ai_cancel()):
		chat("提示", "中止指令未发送(当前离线)")
	set_ai_running(false)

func _send_ai_goal(text: String) -> void:
	chat("本机", text)
	var cmd: Dictionary = CP.ai_goal(text)
	if not AppState.send_command(cmd):
		chat("提示", "目标未发送：AI 目标走 WiFi（当前离线）")
		return
	set_ai_running(true)

func _send_image_goal(items: Array, message: String) -> void:
	chat("本机", ("发图·%s" % message) if message.strip_edges() != "" else "发图")
	if not _upload_images(items):
		return
	var ann: Dictionary = (items[0] as Dictionary).get("annotation", {})
	var cmd: Dictionary = CP.ai_goal(message, ann, true)
	if not AppState.send_command(cmd):
		chat("提示", "AI 目标未发送（WS 掉线？）")
		return
	set_ai_running(true)

## /ai 指令 + 图：按子命令（goal/oneshot/cancel）解析，图作为意图锚点随指令上行。
func _send_ai_with_images(items: Array, text: String) -> void:
	chat("本机", ("发图·%s" % text) if text.strip_edges() != "" else "发图")
	if not _upload_images(items):
		return
	var ann: Dictionary = (items[0] as Dictionary).get("annotation", {})
	var pieces := text.split(" ", true, 2)
	var mode := pieces[1].strip_edges().to_lower() if pieces.size() > 1 else ""
	var msg := pieces[2].strip_edges() if pieces.size() > 2 else ""
	var cmd: Dictionary = {}
	match mode:
		"cancel":
			cmd = CP.ai_cancel()
		"oneshot":
			if msg.is_empty():
				chat("提示", "用法: /ai oneshot <目标文本>")
				return
			cmd = CP.ai_oneshot(msg, ann, true)
		_:
			if msg.is_empty():
				chat("提示", "用法: /ai goal <目标文本>")
				return
			cmd = CP.ai_goal(msg, ann, true)
	if not AppState.send_command(cmd):
		chat("提示", "AI 指令未发送（WS 掉线？）")
		return
	set_ai_running(mode != "cancel")

## 逐张上行编辑图（WS 二进制），任一张失败即中断并返回 false。
func _upload_images(items: Array) -> bool:
	for it in items:
		var img: Image = (it as Dictionary).get("image", null)
		if img == null or not AppState.send_image(img):
			chat("提示", "编辑图未发送: 需先连上 WS 图传")
			return false
	return true

# ============================== 输入历史 ==============================

## 记录一条纯文本输入到历史（去重相邻重复），供上/下键回填。
## 发送后无论是否去重，都回到草稿位，保证下次上翻总是从最近一条开始。
func _push_to_history(text: String) -> void:
	if text.is_empty():
		return
	if _input_history.is_empty() or _input_history[-1] != text:
		_input_history.append(text)
	_history_idx = _input_history.size()  # 指向"末尾之后"=草稿位
	_draft = ""                            # 发送后重置待恢复的草稿

## 上/下键回退输入历史：dir=-1 上翻、+1 下翻。下键翻过最旧一条后回到"草稿位"，恢复上翻前的编辑内容。
func _recall_history(dir: int) -> bool:
	if _input_history.is_empty():
		return false
	if _history_idx < 0 or _history_idx > _input_history.size():
		_history_idx = _input_history.size()
	var from_draft := _history_idx == _input_history.size()
	var new_idx: int = _history_idx + dir
	if from_draft and not _input_history.is_empty() and dir > 0:
		return false  # 已在草稿位还往下翻：无更新内容
	if from_draft and dir < 0:
		_draft = _message_input.text  # 首次离开草稿位，先把未发送输入存下来
	if new_idx < 0 or new_idx > _input_history.size():
		return false
	_history_idx = new_idx
	_message_input.text = _draft if new_idx == _input_history.size() else _input_history[new_idx]
	_message_input.caret_column = _message_input.text.length()
	_message_input.grab_focus()
	return true

func _input(event: InputEvent) -> void:
	if not (event is InputEventKey):
		return
	var k := event as InputEventKey
	if not k.pressed or k.echo or not _message_input.has_focus():
		return
	match k.keycode:
		KEY_UP:
			if _recall_history(-1):
				get_viewport().set_input_as_handled()
		KEY_DOWN:
			if _recall_history(1):
				get_viewport().set_input_as_handled()

# ============================== 指令 ==============================

func _handle_slash(text: String) -> void:
	var pieces := text.split(" ", true, 1)  # 最多拆一次，保住剩余文本原样
	var verb: String = pieces[0].to_lower()
	var cmd: Dictionary = {}
	match verb:
		"/ping":
			var target := pieces[1].strip_edges() if pieces.size() > 1 else ""
			cmd = CP.ping(target)
		"/help", "/h", "?":
			_show_help()
			return
		"/clear":
			# 仅本地清理聊天区，不下发板子。
			_chat_log.clear()
			return
		"/stream":
			var on := true
			if pieces.size() > 1:
				var arg: String = pieces[1].strip_edges().to_lower()
				on = arg != "off" and arg != "0" and arg != "false"
			chat("本机", text)
			stream_requested.emit(on)  # 统一出口：Main 同步开关并起停 UDP 接收
			return
		"/grid":
			# 图传标定网格叠加开关（本地显示层，不下发板子）：/grid [on|off]
			var g_on := true
			if pieces.size() > 1:
				var ga: String = pieces[1].strip_edges().to_lower()
				g_on = ga != "off" and ga != "0" and ga != "false"
			chat("本机", text)
			grid_requested.emit(g_on)
			return
		"/exec_log":
			# 本地直驱状态实时推送开关（默认关）：/exec_log [on|off]
			var el_on := true
			if pieces.size() > 1:
				var ea: String = pieces[1].strip_edges().to_lower()
				el_on = ea == "on" or ea == "1" or ea == "true"
			cmd = CP.exec_log(el_on)
		"/ai_log":
			# AI 调试/延迟日志回推开关（默认关）：/ai_log [on|off]
			var al_on := true
			if pieces.size() > 1:
				var aa: String = pieces[1].strip_edges().to_lower()
				al_on = aa == "on" or aa == "1" or aa == "true"
			cmd = CP.ai_log(al_on)
		"/light":
			# 直驱灯光：/light <front|vibe|back> <0|1>
			var lt_kind := "front"
			var light_on := false
			if pieces.size() > 2:
				lt_kind = pieces[1].strip_edges().to_lower()
				var lt_arg: String = pieces[2].strip_edges().to_lower()
				light_on = lt_arg == "on" or lt_arg == "1" or lt_arg == "true"
			elif pieces.size() > 1:
				lt_kind = "front"
				var la: String = pieces[1].strip_edges().to_lower()
				light_on = la == "on" or la == "1" or la == "true"
			cmd = CP.light(lt_kind, light_on)
		"/stop":
			var scope := "all"
			if pieces.size() > 1 and pieces[1].strip_edges().to_lower() in ["wheels", "arm"]:
				scope = pieces[1].strip_edges().to_lower()
			cmd = CP.stop(scope)
			set_ai_running(false)  # /stop 手动接管：板端打断 AI 闭环，按钮恢复
		"/config":
			var rest := pieces[1] if pieces.size() > 1 else ""
			var kv := rest.strip_edges().split(" ", true, 1)
			if kv.size() < 2 or kv[0].is_empty():
				chat("提示", "用法: /config <WiFi名> <密码>")
				return
			cmd = CP.config_wifi(kv[0], kv[1])
		"/ai":
			_handle_ai_slash(text)
			return
		"/goal":  # 兼容旧写法，等同 /ai goal
			var gmsg := pieces[1].strip_edges() if pieces.size() > 1 else ""
			if gmsg.is_empty():
				chat("提示", "用法: /goal <目标文本>")
				return
			chat("本机", text)
			_send_ai_goal(gmsg)
			return
		"/cancel", "/stopai":  # 兼容旧写法，等同 /ai cancel
			chat("本机", text)
			cmd = CP.ai_cancel()
			set_ai_running(false)
		"/ws":
			_handle_ws_slash(text)
			return
		"/connect":  # 不经蓝牙直连 WS：等同 /ws connect <IP>
			_handle_ws_slash("/ws connect %s" % (pieces[1].strip_edges() if pieces.size() > 1 else ""))
			return
		"/servo":  # 调试直驱：绕过执行板，大脑板直接驱动哪吒机械臂舵机
			var sp := pieces[1].strip_edges() if pieces.size() > 1 else ""
			var kv := sp.split(" ", true, 1)
			if kv.size() < 2 or not kv[0].is_valid_int() or not kv[1].is_valid_int():
				chat("提示", "用法: /servo <n=0转向/1左/2右/3前> <pwm=50..250>")
				return
			var serv_n := kv[0].to_int()
			var serv_pwm := kv[1].to_int()
			if serv_n < 0 or serv_n > 3 or serv_pwm < 50 or serv_pwm > 250:
				chat("提示", "用法: /servo <n=0转向/1左/2右/3前> <pwm=50..250>")
				return
			cmd = CP.servo(serv_n, serv_pwm)
		"/arm_pose":  # 二连杆 IK：给末端位姿，让大脑板联动算左右两舵机
			var sp := pieces[1].strip_edges() if pieces.size() > 1 else ""
			var pv := sp.split(" ", true, 1)
			if pv.size() < 2 or not pv[0].is_valid_float() or not pv[1].is_valid_float():
				chat("提示", "用法: /arm_pose <x=轴前方cm> <h=地面以上cm>")
				return
			var pose_x := pv[0].to_float()
			var pose_h := pv[1].to_float()
			if pose_x < 0.0 or pose_h < 0.0:
				chat("提示", "用法: /arm_pose <x=轴前方cm> <h=地面以上cm>")
				return
			cmd = CP.arm_pose(pose_x, pose_h)
		"/spin":  # 原地转向（普通四轮滑移式）；第三参 angle_deg 定角微操（板端时长近似）
			var sp := pieces[1].strip_edges() if pieces.size() > 1 else ""
			var sv := sp.split(" ", true, 2)
			if sv.size() < 1 or not sv[0].is_valid_int():
				chat("提示", "用法: /spin <dir=+1/-1/0> [speed 0..1000] [angle_deg]")
				return
			var spin_dir := sv[0].to_int()
			var spin_speed := 500
			var spin_angle := 0
			if sv.size() > 1 and sv[1].is_valid_int():
				spin_speed = clampi(sv[1].to_int(), 0, 1000)
			if sv.size() > 2 and sv[2].is_valid_int():
				spin_angle = clampi(sv[2].to_int(), 0, 500)
			if spin_dir < -1 or spin_dir > 1:
				chat("提示", "用法: /spin <dir=+1/-1/0> [speed 0..1000] [angle_deg]")
				return
			cmd = CP.spin(spin_dir, spin_speed, spin_angle)
		"/move":  # 微操/标定测试：定距移动（板端时长近似到点自停）
			# /move <油门 -100..100> <距离 cm 1..500>（油门 50=throttle 0.5）
			var sp := pieces[1].strip_edges() if pieces.size() > 1 else ""
			var mv := sp.split(" ", true, 1)
			if mv.size() < 2 or not mv[0].is_valid_int() or not mv[1].is_valid_int():
				chat("提示", "用法: /move <油门 -100..100> <距离 cm 1..500>（油门50=throttle0.5）")
				return
			var mv_thr := float(mv[0].to_int()) / 100.0
			var mv_cm := mv[1].to_int()
			if mv_thr < -1.0 or mv_thr > 1.0 or mv_cm < 1 or mv_cm > 500:
				chat("提示", "用法: /move <油门 -100..100> <距离 cm 1..500>（油门50=throttle0.5）")
				return
			cmd = CP.move_dist(mv_thr, mv_cm)
		"/motor":  # 调试直驱：绕过执行板，大脑板直接驱动哪吒单轮电机
			var sp := pieces[1].strip_edges() if pieces.size() > 1 else ""
			var mv := sp.split(" ", true, 2)
			if mv.size() < 3 or not mv[0].is_valid_int() or not mv[1].is_valid_int() or not mv[2].is_valid_int():
				chat("提示", "用法: /motor <n=1..4> <a=0..1000> <b=0..1000>")
				return
			var m_n := mv[0].to_int()
			var m_a := mv[1].to_int()
			var m_b := mv[2].to_int()
			if m_n < 1 or m_n > 4 or m_a < 0 or m_a > 1000 or m_b < 0 or m_b > 1000:
				chat("提示", "用法: /motor <n=1..4> <a=0..1000> <b=0..1000>")
				return
			cmd = CP.motor(m_n, m_a, m_b)
		"/drive":  # 调试直驱：一键全车前进/后退/停
			var sp := pieces[1].strip_edges() if pieces.size() > 1 else "0"
			if not sp.is_valid_int():
				chat("提示", "用法: /drive <speed=-1000..1000> (0=停)")
				return
			var d_spd := sp.to_int()
			if d_spd < -1000 or d_spd > 1000:
				chat("提示", "用法: /drive <speed=-1000..1000> (0=停)")
				return
			cmd = CP.drive(d_spd)
		_:
			chat("提示", "未知指令: %s(/help 查看可用指令)" % verb)
			return
	chat("本机", text)
	if not AppState.send_command(cmd):
		chat("提示", "指令未发送(当前离线)")

## /ai 统一入口：/ai goal <目标>（迭代闭环）/ ai oneshot <目标>（单轮）/ ai cancel（取消）。
func _handle_ai_slash(text: String) -> void:
	var pieces := text.split(" ", true, 2)
	var mode := pieces[1].strip_edges().to_lower() if pieces.size() > 1 else ""
	var msg := pieces[2].strip_edges() if pieces.size() > 2 else ""
	var cmd: Dictionary = {}
	match mode:
		"cancel":
			cmd = CP.ai_cancel()
		"oneshot":
			if msg.is_empty():
				chat("提示", "用法: /ai oneshot <目标文本>")
				return
			cmd = CP.ai_oneshot(msg)
		_:
			if msg.is_empty():
				chat("提示", "用法: /ai goal <目标文本>")
				return
			cmd = CP.ai_goal(msg)
	chat("本机", text)
	if not AppState.send_command(cmd):
		chat("提示", "指令未发送(当前离线)")
		return
	set_ai_running(mode != "cancel")

func _show_help() -> void:
	var lines := CP.help_lines()
	chat("提示", "可用指令:\n" + "\n".join(lines))

## /ws 手动控制：connect [IP] 开启自动重连并重连（可带 IP 直连，不经蓝牙）；disconnect 暂停自动重连并断开；status 查状态。
func _handle_ws_slash(text: String) -> void:
	var pieces := text.split(" ", true, 2)
	var arg := pieces[1].strip_edges().to_lower() if pieces.size() > 1 else "status"
	match arg:
		"connect":
			var ip := pieces[2].strip_edges() if pieces.size() > 2 else ""
			DeviceConn.connect_ws(ip)
			chat("提示", ("已发起 WS 连接 %s（自动重连已开启）" % ip) if not ip.is_empty() else "已发起 WS 连接（自动重连已开启）")
		"disconnect":
			DeviceConn.disconnect_ws()
			chat("提示", "已手动断开 WS（暂停自动重连）")
		_:
			var auto := "自动重连" if DeviceConn.ws_is_auto() else "无自动重连"
			chat("提示", "WS:%s（%s）" % [DeviceConn.get_ws_state(), auto])
