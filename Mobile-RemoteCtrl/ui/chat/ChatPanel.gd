extends VBoxContainer
## 聊天区视图：消息日志、指令提示、待上传图片列表、输入发送。
## 职责（Main 只做连接编排，聊天区逻辑全部收口在此）：
##   - 消息日志展示（chat）
##   - 指令提示与附件列表（共用底部区域：有图优先显示图片）
##   - 指令解析经 SlashCommands 完成，本类只做发送与本地副作用
## 与 Main 的交互：图传开关等旁路动作经 stream_requested 信号交给 Main 统一处理。

signal stream_requested(on: bool)
signal grid_requested(on: bool)
## /append 请求：Main 弹出系统文件选择器选图，读图后打开标注编辑器。
signal image_pick_requested()

const CP := preload("res://net/proto/CommandProto.gd")
const SC := preload("res://ui/chat/SlashCommands.gd")
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
	# 删除按钮在脚本里连接（tscn 逐个连太啰嗦）。
	for i in _panels.size():
		_panels[i].get_node("delBtn").pressed.connect(_on_del_pressed.bind(i))
	set_process_input(true)
	_refresh_bottom()

# ============================== 消息日志（Main 外部调用） ==============================

## 追加一条聊天消息。who 取值：本机 / 板 / AI / AI日志 / 提示 / 日志 / 执行板。
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
	elif who == "日志":
		_chat_log.append_text("[color=#c8c8c8]日志[/color]: %s\n" % msg)
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

## 保存当前图传画面（/snapshot，本地操作，不下发板子）。
## 优先写相册 Pictures 目录；失败或 Android 分区存储限制时兜底写应用 user:// 目录。
func _snapshot() -> void:
	var img: Image = DeviceConn.current_image
	if img == null or img.is_empty():
		chat("提示", "暂无可保存的画面（先开启图传、等画面出现）")
		return
	var dirs := PackedStringArray()
	var pictures: String = OS.get_system_dir(OS.SYSTEM_DIR_PICTURES)
	if not pictures.is_empty():
		dirs.append("%s/CarVisionSnapShot" % pictures)
	dirs.append("user://CarVisionSnapShot")
	for d: String in dirs:
		if d.is_empty():
			continue
		DirAccess.make_dir_recursive_absolute(d)
		var path: String = "%s/car_%d.jpg" % [d, Time.get_unix_time_from_system()]
		if img.save_jpg(path) == OK:
			chat("提示", "已保存画面: %s" % path)
			return
	chat("提示", "截图保存失败（无写入权限）")

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
	_refresh_send_btn()

func _on_send_pressed(_new_text: String = "") -> void:
	var text: String = _message_input.text.strip_edges()
	# AI 任务执行中：决定是"插话/发指令"还是"中止"。
	#  - 输入框非空 → 临时为发送语义：插话（纯文本）/ 指令（/ 前缀，板端自决定是否打断）。
	#  - 输入框空 → 中止（打断任务并急停）。
	if _ai_running:
		# AI 运行中：仅"有输入文本"才是插话/发指令语义；空文本一律中止（避免发空插话、静默丢弃附件）。
		if text.is_empty():
			_abort_ai()
		else:
			_send_during_ai(text)
		return
	# 非 AI 态：Button 用 toggle_mode 承载，按下后立即复位按下态（假装普通按钮）。
	_send_btn.set_pressed_no_signal(false)
	if text.is_empty() and _attachments.is_empty():
		return
	if not _attachments.is_empty():
		var plain: String = text
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
	_message_input.text = ""
	_push_to_history(text)
	_refresh_bottom()  # 输入清空后指令提示隐藏（无图时）
	if text.begins_with("/"):
		_handle_slash(text)
	else:
		_send_ai_goal(text)

## AI 任务执行中发送：纯文本→插话补信息（不打断），/ 指令→正常下发（板端自行决定是否接管打断）。
func _send_during_ai(text: String) -> void:
	_message_input.text = ""
	_push_to_history(text)
	_refresh_bottom()
	_refresh_send_btn()
	if text.begins_with("/"):
		# 手动接管类指令会打断 AI（SlashCommands 已标注 interrupts_ai），分发时同步复位按钮。
		_handle_slash(text)
		return
	chat("本机", "插话·%s" % text)
	if not DeviceConn.send_command(CP.ai_chat(text)):
		chat("提示", "插话未发送（当前离线）")

## 切换 AI 执行中状态：Main 摇杆手动接管 / 板端 done / get_state.ai_busy 同步都会调用。
func set_ai_running(run: bool) -> void:
	_ai_running = run
	_refresh_send_btn()

## 刷新发送按钮形态：
##  - AI 运行且无输入 → 「中止」按下态（toggle 卡住）；
##  - AI 运行且有输入 → 临时「发送」（插话），取消按下态；
##  - 非 AI → 「发送」普通按钮（按下态永远复位）。
func _refresh_send_btn() -> void:
	var typing: bool = not _message_input.text.strip_edges().is_empty()
	if _ai_running and not typing:
		_send_btn.text = "中止"
		if not _send_btn.button_pressed:
			_send_btn.set_pressed_no_signal(true)
	else:
		_send_btn.text = "发送"
		if _send_btn.button_pressed:
			_send_btn.set_pressed_no_signal(false)

## 中止 AI：打断进行中的任务并急停（板端 ai_cancel 打断闭环并补停残留持续指令）。
func _abort_ai() -> void:
	chat("本机", "/ai cancel（中止）")
	if not DeviceConn.send_command(CP.ai_cancel()):
		chat("提示", "中止指令未发送(当前离线)")
	set_ai_running(false)

func _send_ai_goal(text: String) -> void:
	chat("本机", text)
	var cmd: Dictionary = CP.ai_goal(text)
	if not DeviceConn.send_command(cmd):
		chat("提示", "目标未发送：AI 目标走 WiFi（当前离线）")
		return
	set_ai_running(true)

func _send_image_goal(items: Array, message: String) -> void:
	chat("本机", ("发图·%s" % message) if message.strip_edges() != "" else "发图")
	if not _upload_images(items):
		return
	var ann: Dictionary = (items[0] as Dictionary).get("annotation", {})
	var cmd: Dictionary = CP.ai_goal(message, ann, true)
	if not DeviceConn.send_command(cmd):
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
	if not DeviceConn.send_command(cmd):
		chat("提示", "AI 指令未发送（WS 掉线？）")
		return
	set_ai_running(mode != "cancel")

## 逐张上行编辑图（WS 二进制），任一张失败即中断并返回 false。
func _upload_images(items: Array) -> bool:
	for it in items:
		var img: Image = (it as Dictionary).get("image", null)
		if img == null or not DeviceConn.send_image(img):
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

## 指令分发：SlashCommands 解析 → 词表指令发送 / 本地动作执行。
func _handle_slash(text: String) -> void:
	var r: Dictionary = SC.parse(text)
	if not bool(r.get("ok", false)):
		chat("提示", str(r.get("hint", "未知指令")))
		return
	chat("本机", text)
	match str(r.get("kind", "")):
		"cmd":
			_send_slash_cmd(r)
		"local":
			_apply_slash_local(r)

## 词表指令：发送并复位发送按钮（手动接管类打断 AI，/ai 类按模式切换）。
func _send_slash_cmd(r: Dictionary) -> void:
	var cmd: Dictionary = r.get("cmd", {})
	var sent: bool = DeviceConn.send_command(cmd)
	if not sent:
		chat("提示", "指令未发送(当前离线)")
	var mode: String = str(r.get("ai_mode", ""))
	if mode != "":
		# /ai 类仅在发送成功后才切换运行态（失败保持原状）。
		if sent:
			set_ai_running(mode != "cancel")
	elif bool(r.get("interrupts_ai", false)):
		# 手动接管类无条件复位（即使离线也还原按钮，避免残留「中止」）。
		set_ai_running(false)

## 本地动作：不经板子，直接处理。
func _apply_slash_local(r: Dictionary) -> void:
	match str(r.get("local", "")):
		"help":
			chat("提示", "可用指令:\n" + "\n".join(CP.help_lines()))
		"clear":
			_chat_log.clear()
			AppLog.clear()
		"stream":
			stream_requested.emit(bool(r.get("on", true)))
		"grid":
			grid_requested.emit(bool(r.get("on", true)))
		"snapshot":
			_snapshot()
		"append":
			image_pick_requested.emit()
		"ws":
			_apply_ws_local(r)

## /ws 手动控制：connect [IP]（可带 IP 直连，不经蓝牙）/ disconnect / status。
func _apply_ws_local(r: Dictionary) -> void:
	var verb: String = str(r.get("arg", "status"))
	var ip: String = str(r.get("ip", ""))
	match verb:
		"connect":
			DeviceConn.connect_ws(ip)
			chat("提示", ("已发起 WS 连接 %s（自动重连已开启）" % ip) if not ip.is_empty() else "已发起 WS 连接（自动重连已开启）")
		"disconnect":
			DeviceConn.disconnect_ws()
			chat("提示", "已手动断开 WS（暂停自动重连）")
		_:
			var auto := "自动重连" if DeviceConn.ws_is_auto() else "无自动重连"
			chat("提示", "WS:%s（%s）" % [DeviceConn.get_ws_state(), auto])
