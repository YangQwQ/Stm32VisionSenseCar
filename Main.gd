extends Control
## App 壳：组装子视图，维护连接对象与交互。
## 传输语义（与用户确认）：/ 前缀=指令；纯文本=AI 目标（DIRECT ai_goal）。
## 所有指令经 AppState.send_command：WS 优先、BLE 兜底。

const CP := preload("res://net/proto/CommandProto.gd")
const BT_ITEM := preload("res://ui/bluetooth/BTDeviceListItem.tscn")

@onready var _video: Control = $BodyControl/Video
@onready var _joystick: VirtualJoystick = $BodyControl/CtrlArea/Joystick
@onready var _wifi_popup: PanelContainer = $WifiPopup
@onready var _editor: Control = $ImageEditor

@onready var _refresh_btn: Button = $BodyBTScan/RefreshBtn
@onready var _device_vbox: VBoxContainer = $BodyBTScan/DeviceList/VBox
@onready var _ble_dot: Label = $TopBar/HBox/VBoxContainer/BleStat/BleDot
@onready var _ble_stat: Label = $TopBar/HBox/VBoxContainer/BleStat
@onready var _ws_dot: Label = $TopBar/HBox/VBoxContainer/WsStat/WsDot
@onready var _ws_stat: Label = $TopBar/HBox/VBoxContainer/WsStat
@onready var _conn_stat: Label = $TopBar/HBox/ConnectionStat

@onready var _chat_log: RichTextLabel = $BodyControl/ChatPanel/ChatLog
@onready var _message_input: LineEdit = $BodyControl/ChatPanel/InputRow/MessageInput
@onready var _stream_toggle: CheckButton = $BodyControl/VidControls/StreamToggle
@onready var _box_hint: Label = $BodyControl/VidControls/BoxHint

@onready var _body_bt: Control = $BodyBTScan
@onready var _body_ctrl: Control = $BodyControl
@onready var _body_about: Control = $BodyAbout
@onready var _nav_bt: TextureButton = $NaviBar/HBox/BTScan
@onready var _nav_ctrl: TextureButton = $NaviBar/HBox/Control
@onready var _nav_about: TextureButton = $NaviBar/HBox/About

var _joy_held := false
var _last_joy_cmd := Vector2.ZERO  # 上一次真正下发的摇杆指令（模拟值量化后对比用）
# 摇杆量化档：死区内不动作；速度分 低速/高速 两档；转向固定档。仅档位变化才发 move，松手发 stop。
const _JOY_DEADZONE := 0.15
const _JOY_SLOW := 0.5
const _JOY_FAST := 1.0
const _JOY_FAST_THRESH := 0.7
const _JOY_STEER := 0.8
## 最近一次成功连接的设备名，用于顶栏「已连接: xxx」。
var _device_name := ""
var _page_tween: Tween = null
const _PAGE_DURATION := 0.28
## 刷新扫描动画 tween；待配网设备 + 待下发 WiFi。
var _refresh_tween: Tween = null
var _pending_addr := ""
var _pending_name := ""
var _pending_provision := {}
var _pending_ai := {}
## 编辑器「采用」后暂存的编辑图与区域，随发送以 [Image N] 标记上行给 AI。
var _attachments: Array = []
## 边扫边显示用：本趟已展示的 address 去重表 + "未发现设备"占位 Label。
var _device_seen: Dictionary = {}
var _empty_hint: Label = null
## 输入历史（仅纯文本，不含图片附件）：上/下键翻阅，_history_idx 指向当前展示项。
## _history_idx == size() 表示停在"当前草稿位"；_draft 存首次上翻前未发送的输入，供下键恢复。
var _input_history: PackedStringArray = []
var _history_idx: int = -1
var _draft: String = ""

@onready var _cmd_hint: Label = $BodyControl/ChatPanel/ChatLog/CommandHint

func _ready() -> void:
	AppState.ble = $Net/BLE
	AppState.ws = $Net/WS

	# 用 toggled + bind 页码；按钮同属一个 ButtonGroup，互斥单选。
	_nav_bt.toggled.connect(_on_nav_toggled.bind(0))
	_nav_ctrl.toggled.connect(_on_nav_toggled.bind(1))
	_nav_about.toggled.connect(_on_nav_toggled.bind(2))

	AppState.hook_auto_ws()
	_request_ble_permissions()
	# WS 由 BLE 会话驱动：不在启动时自连/心跳（避免"未连接设备也在连 WS"），
	# 等 _on_device_connected / 板子上报 IP（AppState._on_ble_status）再连。
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

# ============================== 页面切换 ==============================
# 底部导航点击时把三个 body 横向滑移：目标页 ratio.x=0，其余沿左右各摆一屏。
# body 各自持有固定的相对序号（BTScan=0 / Control=1 / About=2）。

func _on_nav_toggled(pressed_on: bool, page: int) -> void:
	if not pressed_on:
		return
	_switch_page(page)

func _switch_page(page: int) -> void:
	if _page_tween != null:
		_page_tween.kill()
	var tween = create_tween().set_parallel(true).set_trans(Tween.TRANS_CUBIC).set_ease(Tween.EASE_OUT)
	tween.tween_property(_body_bt, "offset_transform_position", Vector2(-20 * page, 0), _PAGE_DURATION)
	tween.tween_property(_body_ctrl, "offset_transform_position", Vector2(-20 * page + 20, 0), _PAGE_DURATION)
	tween.tween_property(_body_about, "offset_transform_position", Vector2(-20 * page + 40, 0), _PAGE_DURATION)
	tween.tween_property(_body_bt, "offset_transform_position_ratio", Vector2(0 - page, 0), _PAGE_DURATION)
	tween.tween_property(_body_ctrl, "offset_transform_position_ratio", Vector2(1 - page, 0), _PAGE_DURATION)
	tween.tween_property(_body_about, "offset_transform_position_ratio", Vector2(2 - page, 0), _PAGE_DURATION)
	_page_tween = tween

# ============================== BLE ==============================

func _on_ble_state(_s: String) -> void:
	_update_status()

func _on_scan_finished(devices: Array) -> void:
	# 扫描结束：复位刷新按钮（取消按下态 + 停止旋转动画）
	if _refresh_btn.button_pressed:
		_refresh_btn.set_pressed_no_signal(false)
	_stop_scan_animation()
	# 设备在扫描中已逐台加进列表（边扫边显示），这里兜底补漏（按地址去重），并处理"整轮一个都没发现"。
	for d: Variant in devices:
		if not (d is Dictionary):
			continue
		var dd: Dictionary = d as Dictionary
		var raw_addr: Variant = dd.get("address")
		if not (raw_addr is String) or (raw_addr as String).is_empty():
			continue
		var addr: String = raw_addr as String
		if _device_seen.has(addr):
			continue
		var nm: String = str(dd.get("name", addr))
		if nm.is_empty():
			nm = addr
		_device_seen[addr] = nm
		_add_device_card(nm, addr)
	if _device_seen.is_empty():
		_show_empty_hint()

## 扫描中逐台发现（device_found）：去重后立刻补一张卡片，实现"边扫边显示"。
func _on_device_found(device: Dictionary) -> void:
	var raw_addr: Variant = device.get("address")
	if not (raw_addr is String) or (raw_addr as String).is_empty():
		return
	var addr: String = raw_addr as String
	if _device_seen.has(addr):
		return
	var nm: String = str(device.get("name", addr))
	if nm.is_empty():
		nm = addr
	_device_seen[addr] = nm
	_add_device_card(nm, addr)

func _add_device_card(name: String, address: String) -> void:
	if _empty_hint != null:
		_empty_hint.queue_free()
		_empty_hint = null
	var item: Node = BT_ITEM.instantiate()
	item.call("setup", name, address)
	item.connect("selected", Callable(self, "_on_device_item_selected"))
	_device_vbox.add_child(item)

func _show_empty_hint() -> void:
	if _empty_hint != null:
		return
	var hint := Label.new()
	hint.text = "未发现设备，点击刷新"
	hint.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	hint.add_theme_font_size_override("font_size", 36)
	hint.add_theme_color_override("font_color", Color(0.6, 0.64, 0.72, 1))
	hint.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	_device_vbox.add_child(hint)
	_empty_hint = hint
	if OS.get_name() == "Android" and not OS.get_granted_permissions().has("android.permission.BLUETOOTH_SCAN"):
		_chat("提示", "未授予蓝牙/附近设备权限，扫描不到设备——请到系统设置允许本 App 权限后刷新")

func _clear_device_list() -> void:
	for child: Node in _device_vbox.get_children():
		child.queue_free()
	_empty_hint = null
	_device_seen.clear()

func _on_device_item_selected(name: String, address: String) -> void:
	# 点击设备卡片：暂存目标，弹出配网/连接窗口（连接 + 可选下发 WiFi/AI，推荐用已存配置）
	_pending_addr = address
	_pending_name = name
	_wifi_popup.call("set_device", name)
	_wifi_popup.call("popup")

func _on_device_connected(_address: String, name: String) -> void:
	_device_name = name
	_update_status()
	# BLE 已连接（会话开始）：发起 WS（默认软 AP 地址；板子上报 IP 时 AppState 会用 ws://ip 覆盖）。
	if AppState.ws != null and not AppState.ws.is_connected_car():
		AppState.ws.connect_car()
	_chat("提示", "已连接设备 %s" % name)
	# 携带配网/AI 请求（设备卡片 → 连接窗口 → 确认），连上且 GATT 就绪后下发。
	var wifi: Dictionary = _pending_provision
	var ai: Dictionary = _pending_ai
	_pending_provision = {}
	_pending_ai = {}
	if wifi.is_empty() and ai.is_empty():
		return
	await _wait_gatt_ready()
	if not wifi.is_empty():
		if AppState.ble.provision(str(wifi.get("ssid", "")), str(wifi.get("password", ""))):
			_chat("提示", "已下发 WiFi: %s | 板子可能重启，稍后会自动重连" % str(wifi.get("ssid", "")))
		else:
			_chat("提示", "配网下发失败")
	if not ai.is_empty():
		if AppState.ble.write_ai_config(str(ai.get("url", "")), str(ai.get("key", "")), str(ai.get("model", ""))):
			_chat("提示", "已下发 AI 配置")
		else:
			_chat("提示", "AI 配置下发失败")

func _on_device_disconnected(reason: String) -> void:
	_update_status()
	# BLE 会话结束 → 结束 WS 会话（关掉自动重连/心跳），避免"无设备也在连 WS"。
	# 例外：WS_ONLY 模式下断开 BLE 是"让出射频"（on_ws_ready 主动断开），此时 WS 仍要继续，不随之断开。
	if AppState.ws != null and not AppState.is_ws_only():
		AppState.ws.disconnect_car()
	_chat("提示", "设备已断开: %s" % reason)

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

func _on_refresh_toggled(pressed_on: bool) -> void:
	# toggle 按下 → 清空旧列表、开始扫描并播放旋转动画；松开 → 停止扫描并复位。
	if pressed_on:
		_clear_device_list()
		_start_scan_animation()
		$Net/BLE.scan()
	else:
		_stop_scan_animation()
		$Net/BLE.stop_scan()

## 扫描旋转动画：先左旋两圈、再右旋两圈，往复循环；松开/结束由 _stop 复位。
func _start_scan_animation() -> void:
	if _refresh_tween != null:
		_refresh_tween.kill()
	_refresh_btn.offset_transform_rotation = 0.0
	var tw := create_tween().set_loops()
	tw.set_trans(Tween.TRANS_CUBIC).set_ease(Tween.EASE_IN_OUT)
	tw.tween_property(_refresh_btn, "offset_transform_rotation", -TAU * 2.0, 1.5)
	tw.tween_property(_refresh_btn, "offset_transform_rotation", TAU * 2.0, 1.5)
	_refresh_tween = tw

func _stop_scan_animation() -> void:
	if _refresh_tween != null:
		_refresh_tween.kill()
		_refresh_tween = null
	create_tween().tween_property(_refresh_btn, "offset_transform_rotation", 0.0, 0.3)\
		.set_trans(Tween.TRANS_CUBIC).set_ease(Tween.EASE_OUT)

func _on_provision_pressed() -> void:
	_wifi_popup.call("popup")

func _on_wifi_confirmed(ssid: String, password: String, url: String, key: String, model: String) -> void:
	# 连接窗口「连接」确认：连接暂存设备；WiFi/AI 留空则仅连接不下发。
	if _pending_addr.is_empty():
		_chat("提示", "请先在列表中选中一个蓝牙设备")
		return
	_pending_provision = {}
	if not ssid.is_empty():
		_pending_provision = {"ssid": ssid, "password": password}
	_pending_ai = {}
	if not url.is_empty():
		_pending_ai = {"url": url, "key": key, "model": model}
	# 持久化本次配置（留空保留旧值），下次打开弹窗自动预填；并记录最近设备供启动自连。
	Store.set_wifi(ssid, password)
	Store.set_ai(url, key, model)
	Store.set_last_device(_pending_addr, _pending_name)
	_chat("提示", "连接 %s …" % _pending_name)
	$Net/BLE.connect_device(_pending_addr, _pending_name)

## 轮询等待 BLE 服务发现完成（_gatt_ready），之后才能写 GATT 特征。
func _wait_gatt_ready() -> void:
	for i in 60:
		if AppState.ble.is_device_connected():
			return
		await get_tree().process_frame

# ============================== WS / 视频 ==============================

func _on_ws_connected() -> void:
	_chat("板", "WS 已连接")
	_update_status()
	# WS 建立：进入 WS_ONLY，让出 BLE 射频（若 BLE 仍在连接则断开）
	AppState.on_ws_ready()

func _on_ws_disconnected(reason: String) -> void:
	_video.call("show_no_signal", true)
	_update_status()
	_chat("板", "WS 已断开:%s" % reason)

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
func _on_input_text_changed(new_text: String) -> void:
	_reconcile_attachments()
	# 打 / 时在聊天区列出可匹配指令及语法；删到不以 / 开头即隐藏。
	if _cmd_hint != null:
		var lines := CP.command_hints(new_text)
		if lines.is_empty():
			_clear_command_hint()
		else:
			_cmd_hint.text = "\n".join(lines)
			_cmd_hint.visible = true

## 清除 / 指令提示。发送、输入清空/不以 / 开头时统一走这里。
func _clear_command_hint() -> void:
	if _cmd_hint != null:
		_cmd_hint.text = ""
		_cmd_hint.visible = false

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
				line = "%s(任务结束)" % line if line != "" else "任务结束"
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

func _on_send_pressed(_new_text: String = "") -> void:
	_reconcile_attachments()
	_clear_command_hint()  # 发送清掉可能的 / 指令提示（输入清空不一定触发 text_changed）
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
	_push_to_history(text)
	if text.begins_with("/"):
		_handle_slash(text)
	else:
		_send_ai_goal(text)

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
## 只回填纯文本，不涉及图片附件。成功返回 true。
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
			_chat("提示", "编辑图未发送: 需先连上 WS 图传")
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
		"/ws":
			_handle_ws_slash(pieces[1].strip_edges().to_lower() if pieces.size() > 1 else "status")
			return
		_:
			_chat("提示", "未知指令: %s(/help 查看可用指令)" % verb)
			return
	_chat("我", text)
	if not AppState.send_command(cmd):
		_chat("提示", "指令未发送(当前离线)")

func _show_help() -> void:
	var lines := CP.help_lines()
	_chat("提示", "可用指令:\n" + "\n".join(lines))

## /ws 手动控制：connect 开启自动重连并重连；disconnect 暂停自动重连并断开；status 查状态。
func _handle_ws_slash(arg: String) -> void:
	var ws := $Net/WS
	match arg:
		"connect":
			AppState.ws.connect_car()
			_chat("提示", "已发起 WS 连接（自动重连已开启）")
		"disconnect":
			AppState.ws.disconnect_car()
			_chat("提示", "已手动断开 WS（暂停自动重连）")
		_:
			var auto := "自动重连" if ws.is_auto_reconnect() else "无自动重连"
			_chat("提示", "WS:%s（%s）" % [ws.get_state(), auto])

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
	if not k.pressed or k.echo or not _message_input.has_focus():
		return
	match k.keycode:
		KEY_BACKSPACE, KEY_DELETE:
			if _delete_input_token(k.keycode == KEY_BACKSPACE):
				get_viewport().set_input_as_handled()
		KEY_UP:
			if _recall_history(-1):
				get_viewport().set_input_as_handled()
		KEY_DOWN:
			if _recall_history(1):
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
	# 内置 VirtualJoystick 把分量写入 4 个 vjoy_* action，据此还原方向向量
	var x: float = Input.get_action_strength("vjoy_right") - Input.get_action_strength("vjoy_left")
	var y: float = Input.get_action_strength("vjoy_down") - Input.get_action_strength("vjoy_up")
	var v: Vector2 = Vector2(x, y)
	# 上=前进：推进取 -y；左右取 x。量化成：速度 低速/高速 两档 + 固定转向，死区内归零
	var throttle := 0.0
	if absf(v.y) >= _JOY_DEADZONE:
		var sp: float = _JOY_FAST if absf(v.y) > _JOY_FAST_THRESH else _JOY_SLOW
		throttle = signf(v.y) * -sp
	var steering := 0.0
	if absf(v.x) >= _JOY_DEADZONE:
		steering = signf(v.x) * _JOY_STEER
	var cmd := Vector2(throttle, steering)
	if cmd == _last_joy_cmd:
		return
	_last_joy_cmd = cmd
	if cmd == Vector2.ZERO:
		AppState.send_command(CP.stop("wheels"))
		return
	AppState.send_command(CP.move(throttle, steering))

func _on_joystick_pressed(_v: Variant = null) -> void:
	_joy_held = true
	_last_joy_cmd = Vector2.ZERO

func _on_joystick_release(_v: Variant = null) -> void:
	_joy_held = false
	_last_joy_cmd = Vector2.ZERO
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
	# 顶栏大字：连接中 / 已连接 / 未连接 三态反馈。
	var ble_ok: bool = $Net/BLE.is_device_connected()
	if ble_ok and _device_name != "":
		_conn_stat.text = "已连接: %s" % _device_name
	elif ble == "connecting":
		_conn_stat.text = ("连接中: %s" % _pending_name) if _pending_name != "" else "连接中…"
	else:
		_conn_stat.text = "设备未连接"
