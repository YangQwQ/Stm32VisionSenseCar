extends Control
## App 壳：组装子视图，维护连接对象与交互。
## 传输语义（与用户确认）：/ 前缀=指令；纯文本=AI 目标（DIRECT ai_goal）。
## 所有指令经 AppState.send_command：WS 优先、BLE 兜底。

const CP := preload("res://net/proto/CommandProto.gd")

@onready var _video: Control = $Body/Video
@onready var _joystick: VirtualJoystick = $Body/CtrlArea/Joystick
@onready var _wifi_popup: PanelContainer = $WifiPopup
@onready var _editor: Control = $ImageEditor

@onready var _device_list: OptionButton = $TopBar/HBox/DeviceList
@onready var _ble_dot: Label = $TopBar/HBox/DeviceList/BleStat/BleDot
@onready var _ble_stat: Label = $TopBar/HBox/DeviceList/BleStat
@onready var _ws_dot: Label = $TopBar/HBox/ConnectionStat/WsStat/WsDot
@onready var _ws_stat: Label = $TopBar/HBox/ConnectionStat/WsStat
@onready var _conn_stat: Label = $TopBar/HBox/ConnectionStat

@onready var _chat_log: RichTextLabel = $Body/ChatPanel/ChatLog
@onready var _message_input: LineEdit = $Body/ChatPanel/InputRow/MessageInput
@onready var _stream_toggle: CheckButton = $Body/VidControls/StreamToggle
@onready var _box_hint: Label = $Body/VidControls/BoxHint

var _joy_held := false
var _last_joy := Vector2.ZERO
## 编辑器「采用」后暂存的编辑图与区域，随发送以 [Image N] 标记上行给 AI。
var _attachments: Array = []

func _ready() -> void:
	AppState.ble = $Net/BLE
	AppState.ws = $Net/WS

	$Net/BLE.ble_state_changed.connect(_on_ble_state)
	$Net/BLE.scan_finished.connect(_on_scan_finished)
	$Net/BLE.device_connected.connect(_on_device_connected)
	$Net/BLE.device_disconnected.connect(_on_device_disconnected)
	$Net/BLE.status_received.connect(_on_ble_status)
	$Net/WS.connected.connect(_on_ws_connected)
	$Net/WS.disconnected.connect(_on_ws_disconnected)
	$Net/WS.frame_received.connect(_on_frame)
	$Net/WS.text_received.connect(_on_ws_text)
	# 编辑器信号是脚本自定义信号，对基类不可静态访问，用字符串 connect
	_editor.connect("cancelled", Callable(self, "_on_editor_cancelled"))
	_editor.connect("image_sent", Callable(self, "_on_image_sent"))
	_message_input.text_changed.connect(_on_input_text_changed)
	_wifi_popup.connect("confirmed", Callable(self, "_on_wifi_confirmed"))
	_device_list.item_selected.connect(_on_device_selected)

	AppState.hook_auto_ws()
	_request_ble_permissions()
	# 启动即尝试连接小车（默认软 AP 地址；离线只显示未连接，不阻塞 UI）
	$Net/WS.connect_car()
	_update_status()
	_update_attach_hint()
	set_process_input(true)

# Android 运行时权限：BLE 扫描/连接 + 定位。声明在 export_presets（BLUETOOTH_* 等），
# 这里启动即申请，新装手机首次打开会弹窗，无需 adb pm grant。
# 注：ACCESS_FINE_LOCATION 要 toggle=true 与 custom_permissions 双声明，才会额外带一条
# 无 maxSdkVersion 的声明（toggle 单独那条被写死 maxSdk=30，API>=31 实为未声明）。MIUI 门禁认
# FINE，无 cap 条存在即可弹窗授权，见 CLAUDE.md「已知坑」。无需 apktool / pm grant。
func _request_ble_permissions() -> void:
	if OS.get_name() != "Android":
		return
	for p: String in [
			"android.permission.BLUETOOTH_SCAN",
			"android.permission.BLUETOOTH_CONNECT",
			"android.permission.ACCESS_FINE_LOCATION",
		]:
		if not OS.get_granted_permissions().has(p):
			OS.request_permission(p)

# ============================== BLE ==============================

func _on_ble_state(_s: String) -> void:
	_update_status()

func _on_scan_finished(devices: Array) -> void:
	_device_list.clear()
	for d: Variant in devices:
		if not (d is Dictionary):
			continue
		var dd: Dictionary = d as Dictionary
		var raw_name: Variant = dd.get("name")
		var raw_addr: Variant = dd.get("address")
		if not (raw_name is String) or not (raw_addr is String):
			continue
		var name: String = raw_name as String
		_device_list.add_item(name)
		_device_list.set_item_metadata(_device_list.item_count - 1, dd)
	if _device_list.item_count == 0:
		_device_list.add_item("（未发现设备，点刷新）")
		_device_list.set_item_metadata(0, {})
		if OS.get_name() == "Android" and not OS.get_granted_permissions().has("android.permission.BLUETOOTH_SCAN"):
			_chat("提示", "未授予蓝牙/附近设备权限，扫描不到设备——请到系统设置允许本 App 权限后刷新")

func _on_device_selected(index: int) -> void:
	var meta: Variant = _device_list.get_item_metadata(index)
	if not (meta is Dictionary):
		return
	var addr: String = str((meta as Dictionary).get("address", ""))
	if addr.is_empty():
		return
	var name: String = str((meta as Dictionary).get("name", addr))
	_chat("提示", "连接 %s …" % name)
	$Net/BLE.connect_device(addr, name)

func _on_device_connected(_address: String, name: String) -> void:
	_device_list.text = name
	_update_status()
	_chat("提示", "已连接设备 %s；可在聊天框用 / 指令或直接发文字目标" % name)

func _on_device_disconnected(reason: String) -> void:
	_update_status()
	_chat("提示", "设备已断开：%s" % reason)

func _on_ble_status(data: Dictionary) -> void:
	# 板子 BLE status：含 reply 时展示（如配网/指令应答）；自动连 WS 由 AppState 处理。
	# 板子 reply 可能是词表应答 JSON（{"type":status,pong,"params":{reason}}），解析出可读文本。
	var reply: Variant = data.get("reply")
	if reply is String and not (reply as String).is_empty():
		var txt: String = reply as String
		var parsed: Variant = JSON.parse_string(txt)
		if parsed is Dictionary:
			var t: String = str((parsed as Dictionary).get("type", ""))
			if t == "pong":
				txt = "pong"
			elif (parsed as Dictionary).has("params"):
				var pm: Variant = (parsed as Dictionary).get("params")
				if pm is Dictionary and (pm as Dictionary).has("reason"):
					txt = str((pm as Dictionary).get("reason"))
		_chat("板", txt)
	_update_status()

func _on_refresh_pressed() -> void:
	# 立即让下拉框进入"扫描中"态，点一次必有可见反应；结果由 scan_finished 覆盖
	_device_list.clear()
	_device_list.add_item("扫描中…")
	_device_list.set_item_metadata(0, {})
	$Net/BLE.scan()

func _on_provision_pressed() -> void:
	_wifi_popup.call("popup")

func _on_wifi_confirmed(ssid: String, password: String) -> void:
	if AppState.ble == null or not AppState.ble.is_device_connected():
		_chat("提示", "配网需先连接小车蓝牙（在顶部列表选中设备）")
		return
	if AppState.ble.provision(ssid, password):
		_chat("提示", "已下发 WiFi: %s；板子将重启连网，稍后会自动重连并显示 IP" % ssid)
	else:
		_chat("提示", "配网下发失败")

# ============================== WS / 视频 ==============================

func _on_ws_connected() -> void:
	_chat("板", "WS 已连接")
	_update_status()

func _on_ws_disconnected(_reason: String) -> void:
	_video.call("show_no_signal", true)
	_update_status()

func _on_frame(img: Image) -> void:
	_video.call("set_frame", img)
	AppState.current_image = img

func _on_ws_text(data: Dictionary) -> void:
	var t: String = str(data.get("type", ""))
	if t == "pong":
		_chat("板", "pong")
		return
	if t == "ai_result":
		_show_ai_result(data)
		return
	if t != "status":
		return
	# status：尽量展示人类可读字段（reason / reply），纯机器状态略
	var params: Variant = data.get("params")
	var line := ""
	if params is Dictionary:
		var r: Variant = (params as Dictionary).get("reason")
		if r is String and not (r as String).is_empty():
			line = r as String
	elif data.has("reply"):
		var rp: Variant = data.get("reply")
		if rp is String and not (rp as String).is_empty():
			line = rp as String
	if line != "":
		_chat("板", line)

func _on_stream_toggled(on: bool) -> void:
	AppState.send_command(CP.stream(on))
	_video.visible = on

# ============================== 框选（编辑器） ==============================

func _on_annotate_pressed() -> void:
	# _video 以基类 Control 持有，脚本成员只能动态取
	var tex: Variant = _video.get("current_texture")
	if tex == null:
		_chat("提示", "先开启图传、等画面出现再框选目标")
		return
	_editor.call("open", tex)

func _on_editor_cancelled() -> void:
	pass  # 取消 = 放弃这张图，不影响输入框与已附图

## 编辑器「采用」：把编辑图作为附件以 [Image N] 标记附到输入框，供发送时上行给 AI。
func _on_image_sent(img: Image, annotation: Dictionary) -> void:
	if img == null:
		_chat("提示", "未能导出编辑图（无可用画面），请先框选再采用")
		return
	_attachments.append({"image": img, "annotation": annotation})
	_message_input.text += (_token_text(_attachments.size()) if _message_input.text.is_empty() else " " + _token_text(_attachments.size()))
	_message_input.caret_column = _message_input.text.length()
	_message_input.grab_focus()
	_update_attach_hint()

## 文本每次变化都重建附件与标记的对应：删除某段 [Image N] 时同步移除对应图并重编号，不会错位。
func _on_input_text_changed(_new_text: String) -> void:
	_reconcile_attachments()

func _token_text(i: int) -> String:
	return "[Image %d]" % i

func _strip_tokens(txt: String) -> String:
	var re := RegEx.new()
	re.compile("\\[Image \\d+\\]")
	return re.sub(txt, "", true).strip_edges()

## 依据当前输入框内实际存在的标记，重建附件列表并以 1..N 重编号；无标记则清空附件。
func _reconcile_attachments() -> void:
	var txt := _message_input.text
	var source: Array = _attachments
	var re := RegEx.new()
	re.compile("\\[Image \\d+\\]")
	var matches := re.search_all(txt)
	var kept: Array = []
	var seen := {}
	for m in matches:
		var idx: int = int(m.get_string().trim_prefix("[Image ").trim_suffix("]")) - 1
		if idx >= 0 and idx < source.size() and not seen.has(idx):
			kept.append(source[idx])
			seen[idx] = true
	if not matches.is_empty() and kept.size() == source.size():
		return  # 一一对应，无需改
	# 重建文本：保留标记外的输入，按顺序重贴 1..kept.size()
	var final := ""
	var mi := 0
	for m in matches:
		final += txt.substr(0, m.get_start())
		if mi < kept.size():
			final += _token_text(mi + 1)
			mi += 1
		txt = txt.substr(m.get_end())
	final += txt
	_message_input.text = final
	_message_input.caret_column = final.length()
	_attachments = kept
	_update_attach_hint()

func _update_attach_hint() -> void:
	var n := _attachments.size()
	if n == 0:
		_box_hint.text = "无附图"
	else:
		_box_hint.text = "已附图 %d 张" % n

func _show_ai_result(data: Dictionary) -> void:
	# ai_result：{type:"ai_result", id, params:{error?, reason?, done?, command:{type,params,reason}}}
	var params: Variant = data.get("params")
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
				line = "%s（任务结束）" % line if line != "" else "任务结束"
	if line == "":
		line = "已收到 AI 输出"
	_chat("AI", line)

func _cmd_text(cmd: Dictionary) -> String:
	var t: String = str(cmd.get("type", ""))
	var p: Variant = cmd.get("params")
	var parts := PackedStringArray()
	if p is Dictionary:
		for k: Variant in (p as Dictionary).keys():
			parts.append("%s=%s" % [str(k), str((p as Dictionary).get(k))])
	return "%s (%s)" % [t, ", ".join(parts)] if parts.size() > 0 else t

# ============================== 聊天 / 指令 ==============================

func _on_send_pressed() -> void:
	_reconcile_attachments()
	if _message_input.text.strip_edges().is_empty() and _attachments.is_empty():
		return
	if not _attachments.is_empty():
		var plain: String = _strip_tokens(_message_input.text)
		var batch: Array = _attachments
		_message_input.text = ""
		_attachments = []
		_update_attach_hint()
		_send_image_goal(batch, plain)
		return
	var text: String = _message_input.text.strip_edges()
	_message_input.text = ""
	if text.begins_with("/"):
		_handle_slash(text)
	else:
		_send_ai_goal(text)

func _send_ai_goal(text: String) -> void:
	_chat("我", text)
	var cmd: Dictionary = CP.ai_goal(text)
	if not AppState.send_command(cmd):
		_chat("提示", "目标未发送：AI 目标走 WiFi（当前离线）")

func _send_image_goal(items: Array, message: String) -> void:
	_chat("我", ("发图·%s" % message) if message.strip_edges() != "" else "发图")
	# 先逐张上行编辑图（WS 二进制），再发文本 ai_goal{use_image:true}——板侧以最后一张为意图锚点。
	for it in items:
		var img: Image = (it as Dictionary).get("image", null)
		if img == null or not AppState.send_image(img):
			_chat("提示", "编辑图未发送：需先连上 WS 图传")
			return
	var ann: Dictionary = (items[0] as Dictionary).get("annotation", {})
	var cmd: Dictionary = CP.ai_goal(message, ann, true)
	if not AppState.send_command(cmd):
		_chat("提示", "AI 目标未发送（WS 掉线？）")

func _handle_slash(text: String) -> void:
	var pieces := text.split(" ", true, 1)  # 最多拆一次，保住剩余文本原样
	var verb: String = pieces[0].to_lower()
	var cmd: Dictionary = {}
	match verb:
		"/ping":
			cmd = CP.ping()
		"/help", "/h", "?":
			_show_help()
			return
		"/snapshot", "/snap":
			cmd = CP.snapshot()
		"/stream":
			var on := true
			if pieces.size() > 1:
				var arg: String = pieces[1].strip_edges().to_lower()
				on = arg != "off" and arg != "0" and arg != "false"
			cmd = CP.stream(on)
			_stream_toggle.set_pressed_no_signal(on)
		"/stop":
			var scope := "all"
			if pieces.size() > 1 and pieces[1].strip_edges().to_lower() in ["wheels", "arm"]:
				scope = pieces[1].strip_edges().to_lower()
			cmd = CP.stop(scope)
		"/config":
			var rest := pieces[1] if pieces.size() > 1 else ""
			var kv := rest.strip_edges().split(" ", true, 1)
			if kv.size() < 2 or kv[0].is_empty():
				_chat("提示", "用法: /config <WiFi名> <密码>")
				return
			cmd = CP.config_wifi(kv[0], kv[1])
		"/goal":
			var rest := pieces[1] if pieces.size() > 1 else ""
			var msg: String = rest.strip_edges()
			if msg.is_empty():
				_chat("提示", "用法: /goal <目标文本>")
				return
			cmd = CP.ai_goal(msg)
		"/cancel", "/stopai":
			cmd = CP.ai_cancel()
		_:
			_chat("提示", "未知指令: %s（/help 查看可用指令）" % verb)
			return
	_chat("我", text)
	if not AppState.send_command(cmd):
		_chat("提示", "指令未发送（当前离线）")

func _show_help() -> void:
	var lines := CP.help_lines()
	_chat("提示", "可用指令：\n" + "\n".join(lines))

func _chat(who: String, msg: String) -> void:
	if who == "我":
		_chat_log.append_text("[b]我[/b]: %s\n" % msg)
	elif who == "板":
		_chat_log.append_text("[color=#6fc3ff]小车[/color]: %s\n" % msg)
	elif who == "AI":
		_chat_log.append_text("[color=#c9f7a8]AI[/color]: %s\n" % msg)
	elif who == "提示":
		_chat_log.append_text("[color=#ffd75e]系统[/color]: %s\n" % msg)
	else:
		_chat_log.append_text(msg + "\n")

# ============================== 附件标记整段删除 ==============================
# 输入框聚焦时，按 Backspace / Delete 若光标落在 [Image N] 上，整段删除该标记
# （_input 先于 LineEdit 处理，拦截后置为已处理，避免只删一个字符）。

func _input(event: InputEvent) -> void:
	if not (event is InputEventKey):
		return
	var k := event as InputEventKey
	if not k.pressed or k.echo or (k.keycode != KEY_BACKSPACE and k.keycode != KEY_DELETE):
		return
	if not _message_input.has_focus():
		return
	if _delete_input_token(k.keycode == KEY_BACKSPACE):
		get_viewport().set_input_as_handled()

func _delete_input_token(is_backspace: bool) -> bool:
	var txt: String = _message_input.text
	if txt.is_empty():
		return false
	var pos: int = (_message_input.caret_column - 1) if is_backspace else _message_input.caret_column
	if pos < 0:
		return false
	var re := RegEx.new()
	re.compile("\\[Image \\d+\\]")
	for m in re.search_all(txt):
		var s: int = m.get_start()
		var e: int = m.get_end()
		if pos >= s and pos < e:
			_message_input.text = txt.erase(s, e - s)
			_message_input.caret_column = s
			return true
	return false

# ============================== 手动控制 ==============================

func _process(_delta: float) -> void:
	if _joy_held:
		_update_joystick()

func _update_joystick() -> void:
	var v: Vector2 = _joystick.get_value()
	# 上=前进：推力取 -y；左右取 x
	var throttle: float = clampf(-v.y, -1.0, 1.0)
	var steering: float = clampf(v.x, -1.0, 1.0)
	if v.distance_to(_last_joy) < 0.001:
		return
	_last_joy = v
	AppState.send_command(CP.move(throttle, steering))

func _on_joystick_pressed(_v: Variant = null) -> void:
	_joy_held = true

func _on_joystick_release(_v: Variant = null) -> void:
	_joy_held = false
	_last_joy = Vector2.ZERO
	AppState.send_command(CP.stop("wheels"))

# ============================== 状态 ==============================

func _update_status() -> void:
	var ble: String = $Net/BLE.get_ble_state()
	var ws_state: String = $Net/WS.get_state()
	var online: bool = ws_state == "connected"
	var ble_on: bool = ble != "off" and ble != "unavailable"
	_ble_dot.modulate = Color.GREEN if ble_on else Color(1, 1, 1, 0.3)
	_ws_dot.modulate = Color.GREEN if online else Color(1, 1, 1, 0.3)
	_ble_stat.text = "BLE:%s" % ble
	_ws_stat.text = "WS:%s" % ("已连接" if online else "未连接")
	var transport: String = AppState.best_transport_name() if AppState.has_method("best_transport_name") else "离线"
	_conn_stat.text = "已连: %s" % transport
