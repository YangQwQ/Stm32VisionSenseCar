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

@onready var _chat_panel = $BodyControl/ChatPanel
@onready var _stream_toggle: CheckButton = $BodyControl/VidControls/StreamToggle

@onready var _body_bt: Control = $BodyBTScan
@onready var _body_ctrl: Control = $BodyControl
@onready var _body_about: Control = $BodyAbout
@onready var _auto_conn_btn: CheckButton = $BodyAbout/Options/AutoConnOnStart
@onready var _disable_ws_btn: CheckButton = $BodyAbout/Options/DisableAutoConnWS
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
## 直接驱动（绕过执行板）时的摇杆映射参数。
const _DRIVE_MAX := 1000       # 油门满量程 PWM（低速档 500 / 高速档 1000）
const _SERVO_CENTER := 150     # 转向舵中位（/servo 1 150 = 正前）
const _SERVO_RANGE := 30       # 转向舵单侧偏转量（右 +30→180 / 左 -30→120）
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
## 启动自连的待匹配地址（小写）。扫描中发现该地址即自动连接；轮结束仍未出现则提示手动连。
var _startup_connect_addr := ""
## 边扫边显示用：本趟已展示的 address 去重表 + "未发现设备"占位 Label。
var _device_seen: Dictionary = {}
var _empty_hint: Label = null

func _ready() -> void:
	# 只对统一设备连接层 DeviceConn 说话：连接其统一信号（传输事件由 DeviceConn 收口）。
	DeviceConn.scan_finished.connect(_on_scan_finished)
	DeviceConn.device_found.connect(_on_device_found)
	DeviceConn.scan_started.connect(_on_device_scan_started)
	DeviceConn.device_connected.connect(_on_device_connected)
	DeviceConn.device_disconnected.connect(_on_device_disconnected)
	DeviceConn.ws_connected.connect(_on_ws_connected)
	DeviceConn.ws_disconnected.connect(_on_ws_disconnected)
	DeviceConn.status_received.connect(_on_ble_status)
	DeviceConn.text_received.connect(_on_ws_text)
	DeviceConn.frame_received.connect(_on_frame)
	DeviceConn.state_changed.connect(_on_ble_state)

	# 用 toggled + bind 页码；按钮同属一个 ButtonGroup，互斥单选。
	_nav_bt.toggled.connect(_on_nav_toggled.bind(0))
	_nav_ctrl.toggled.connect(_on_nav_toggled.bind(1))
	_nav_about.toggled.connect(_on_nav_toggled.bind(2))

	_request_ble_permissions()
	# WS 由 BLE 会话驱动（DeviceConn 已在连接态上收自动连 WS）：不在启动时自连/心跳，
	# 等设备连接 / 板子上报 IP（DeviceConn._on_ble_status）再连。
	_update_status()
	# 设置项：读取本地配置并同步两个开关状态；开启启动自连时按最近设备重连。
	_auto_conn_btn.set_pressed_no_signal(Store.get_auto_conn())
	_disable_ws_btn.set_pressed_no_signal(Store.get_disable_auto_ws())
	if Store.get_auto_conn():
		# 启动即自动连接：直接进控制页（页 1，触发 _on_nav_toggled → _switch_page），不再停在蓝牙扫描页。
		_nav_ctrl.button_pressed = true
		_try_startup_connect()

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

## 设置项：启动时自动连接上次设备。持久化开关；开启时立即尝试自连（含等待蓝牙就绪）。
func _on_auto_conn_toggled(on: bool) -> void:
	Store.set_auto_conn(on)
	if on:
		_try_startup_connect()

## 设置项：关闭自动建立 WS 连接（纯蓝牙控制）。仅持久化，连连接态下一次设备连接生效。
func _on_disable_ws_toggled(on: bool) -> void:
	Store.set_disable_auto_ws(on)

## 启动（或开启自连开关）时重连上次设备。Android 栈要求设备必须先被扫描到才能连接，
## 故这里先扫描、等 `_on_device_found` 中目标地址出现再连（不能像 RECOVERY 那样按地址直连）。
func _try_startup_connect() -> void:
	if DeviceConn.get_ble_state() == "unavailable":
		return
	if not await _await_ble_ready():
		_chat_panel.chat("提示", "蓝牙不可用，未自动连接设备")
		return
	var last: Dictionary = Store.get_last_device()
	var addr: String = str(last.get("address", ""))
	if addr.is_empty():
		return
	_startup_connect_addr = addr.to_lower()
	_pending_addr = addr
	_pending_name = str(last.get("name", addr))
	_chat_panel.chat("提示", "启动自连：扫描并连接 %s …" % _pending_name)
	DeviceConn.scan()

## 轮询等待蓝牙适配器进入可用状态（Android 含运行时授权弹窗）。超时或不可用则放弃。
func _await_ble_ready() -> bool:
	for i in 600:  # 至多约 10s
		var s: String = DeviceConn.get_ble_state()
		if s in ["idle", "scanning", "connecting", "connected"]:
			return true
		if s == "unavailable":
			return false
		await get_tree().process_frame
	return false

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
	# 启动自连兜底：整轮扫描没出现目标地址则提示，等待用户手动连接。
	if _startup_connect_addr != "":
		_startup_connect_addr = ""
		_chat_panel.chat("提示", "未扫描到上次设备 %s，请手动连接" % _pending_name)

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
	# 启动自连：发现目标地址即在卡片弹出同时自动连接（清空待匹配防重复）。
	if _startup_connect_addr != "" and addr.to_lower() == _startup_connect_addr:
		_startup_connect_addr = ""
		_chat_panel.chat("提示", "启动自连：发现 %s，正在连接…" % nm)
		DeviceConn.connect_device(addr, nm)

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
		_chat_panel.chat("提示", "未授予蓝牙/附近设备权限，扫描不到设备——请到系统设置允许本 App 权限后刷新")

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
	# BLE 已连接（会话开始）：发起 WS（默认软 AP 地址；板子上报 IP 时 DeviceConn 会用 ws://ip 覆盖）。
	# 设置了「关闭自动建立WS连接」= 纯蓝牙控制，则跳过自动连 WS（仍可用 /ws connect 手动连）。
	if not DeviceConn.get_ws_state() in ["connected", "connecting"] and not Store.get_disable_auto_ws():
		DeviceConn.connect_ws()
	_chat_panel.chat("提示", "已连接设备 %s" % name)
	# 携带配网/AI 请求（设备卡片 → 连接窗口 → 确认），连上且 GATT 就绪后下发。
	var wifi: Dictionary = _pending_provision
	var ai: Dictionary = _pending_ai
	_pending_provision = {}
	_pending_ai = {}
	if wifi.is_empty() and ai.is_empty():
		return
	await _wait_gatt_ready()
	if not wifi.is_empty():
		if DeviceConn.provision(str(wifi.get("ssid", "")), str(wifi.get("password", ""))):
			_chat_panel.chat("提示", "已下发 WiFi: %s | 板子可能重启，稍后会自动重连" % str(wifi.get("ssid", "")))
		else:
			_chat_panel.chat("提示", "配网下发失败")
	if not ai.is_empty():
		if DeviceConn.write_ai_config(str(ai.get("url", "")), str(ai.get("key", "")), str(ai.get("model", ""))):
			_chat_panel.chat("提示", "已下发 AI 配置")
		else:
			_chat_panel.chat("提示", "AI 配置下发失败")

func _on_device_disconnected(reason: String) -> void:
	_update_status()
	# 会话结束：WS 重连/双断恢复等策略由 DeviceConn 收口，这里仅提示 UI。
	# 注意：WS_ONLY 让出 BLE 射频时的 BLE 断开不触发本信号（DeviceConn 在此场景不抛 device_disconnected）。
	_chat_panel.chat("提示", "设备已断开: %s" % reason)

func _on_ble_status(data: Dictionary) -> void:
	# 板子 BLE status：含 reply 时展示（如配网/指令应答）；自动连 WS 由 AppState 处理。
	# 板子 reply 可能是词表应答 JSON（{"type":status,pong,"params":{reason}}），解析出可读文本。
	# 也可能是纯文本（如 "WiFi ..."）——用 JSON.new().parse() 拿错误码而不是 parse_string 打 C++ 错误。
	var reply: Variant = data.get("reply")
	if reply is String and not (reply as String).is_empty():
		var txt: String = reply as String
		var json := JSON.new()
		var parsed: Variant = {}
		if json.parse(txt) == OK and json.data is Dictionary:
			parsed = json.data
			var t: String = str(parsed.get("type", ""))
			if t == "pong":
				txt = "pong"
			elif parsed.has("params"):
				var pm: Variant = parsed.get("params")
				if pm is Dictionary and (pm as Dictionary).has("reason"):
					txt = str((pm as Dictionary).get("reason"))
		_chat_panel.chat("板", txt)
	_update_status()

func _on_refresh_toggled(pressed_on: bool) -> void:
	# toggle 按下 → 打断启动自连、清空旧列表，交由 DeviceConn 统一开扫（scan_started 会按上按钮+动画）；
	# 松开 → 统一停扫（恢复扫描也会一并打断）。
	if pressed_on:
		_cancel_startup_connect()
		_clear_device_list()
		DeviceConn.scan()
	else:
		DeviceConn.stop_scan()

## 任何一次扫描（手动 / 双断恢复）都由 DeviceConn 的上报驱动同一套扫描按钮 + 动画，保证统一、可打断。
func _on_device_scan_started() -> void:
	_refresh_btn.set_pressed_no_signal(true)
	_start_scan_animation()

## 手动按扫描按钮：打断启动自连（清掉待匹配地址与提示名），之后按普通手动扫描流程走，
## 不再对扫到的目标做自动连接、也不再在整轮没出现时弹"请手动连接"。
func _cancel_startup_connect() -> void:
	_startup_connect_addr = ""
	_pending_addr = ""
	_pending_name = ""

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
		_chat_panel.chat("提示", "请先在列表中选中一个蓝牙设备")
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
	_chat_panel.chat("提示", "连接 %s …" % _pending_name)
	DeviceConn.connect_device(_pending_addr, _pending_name)

## 轮询等待 BLE 服务发现完成（_gatt_ready），之后才能写 GATT 特征。
func _wait_gatt_ready() -> void:
	for i in 60:
		if DeviceConn.is_device_connected():
			return
		await get_tree().process_frame

# ============================== WS / 视频 ==============================

func _on_ws_connected() -> void:
	_chat_panel.chat("板", "WS 已连接")
	_update_status()
	# WS 建立即进入 WS_ONLY（让出 BLE 射频）由 DeviceConn 在内部处理。
	# 重连/复线后按图传开关当前状态重发一次开启指令：断连期间开关仍保持「开」而板子画面已断，
	# 若不重发需要用户手动再拨一次。UDP VideoClient 每次重连需换新端口并随指令重新上报。
	if _stream_toggle.button_pressed:
		_apply_stream(true)

func _on_ws_disconnected(reason: String) -> void:
	_video.call("show_no_signal", true)
	DeviceConn.stop_video()  # WS 掉线：UDP 对端随之失效，停接收
	_chat_panel.set_ai_running(false)  # 掉线即任务中断：按钮复位「发送」，避免重连后残留「中止」
	_update_status()
	var r := reason
	if r.is_empty():
		r = "连接中断"
	_chat_panel.chat("板", "WS 已断开:%s" % r)

func _on_frame(img: Image) -> void:
	_video.call("set_frame", img)
	AppState.current_image = img

func _on_ws_text(data: Dictionary) -> void:
	var t: String = str(data.get("type", ""))
	if t == "pong":
		_chat_panel.chat("板", "pong")
		return
	if t == "ai_result":
		_chat_panel.show_ai_result(data)
		return
	if t == "exec_status":
		# 执行板日志镜像：板子把执行板上行帧转发过来（状态帧已解码成可读文本 text）。
		# 无 text 的非状态帧回退显示原始 payload hex。
		var p: Variant = data.get("params")
		var line := ""
		if p is Dictionary:
			var pm := p as Dictionary
			var txt: Variant = pm.get("text")
			if txt is String and not (txt as String).is_empty():
				line = txt as String
			else:
				line = "%02X" % int(pm.get("cmd", 0))
				var hx: Variant = pm.get("hex")
				if hx is Array:
					var parts := PackedStringArray()
					for b in hx:
						parts.append("%02X" % int(b))
					line += " " + " ".join(parts)
		_chat_panel.chat("执行板", line)
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
		_chat_panel.chat("板", line)

func _on_stream_toggled(on: bool) -> void:
	_apply_stream(on)

## 图传开关统一出口：开 → 先起 UDP 接收拿本地端口，再发 stream(udp_port) 让板子向该端口推 JPEG；
## 关 → 停 UDP 接收并发 stream off。_on_ws_connected / /stream 均走这里，保证端口上报一致。
func _apply_stream(on: bool) -> void:
	var port := -1
	if on:
		port = DeviceConn.start_video()
		if port < 0:
			_chat_panel.chat("提示", "UDP 图传初始化失败")
	AppState.send_command(CP.stream(on, port if port > 0 else 0, _my_ipv4()))
	_video.visible = on

## 取手机非回环 IPv4 本机地址（板端建 UDP 会话用，见 CommandProto.stream）。
## 优先挑与板子（WS 对端）同网段的地址：手机可能带 VPN/虚拟网卡（如 tun0 172.19.0.1），
## 若直接取首个非回环地址，可能把隧道 IP 报给板子，板端 UDP 发到该地址不可达 → 手机收不到画面。
func _my_ipv4() -> String:
	var host: String = DeviceConn.board_ip()
	for a in IP.get_local_addresses():
		if _is_usable_ipv4(a) and not host.is_empty() and _same_subnet(a, host):
			return a
	# 兜底：无匹配网段时任取一个可用 IPv4（无 VPN 环境与旧行为一致）
	for a in IP.get_local_addresses():
		if _is_usable_ipv4(a):
			return a
	return ""

func _is_usable_ipv4(a: String) -> bool:
	return a.find(".") != -1 and not a.begins_with("127.") and a != "0.0.0.0"

## 板子与手机同接一个 WiFi/LAN，一般 /24：前 3 段一致即视为同网段。
func _same_subnet(a: String, b: String) -> bool:
	var sa := a.split(".")
	var sb := b.split(".")
	return sa.size() >= 3 and sb.size() >= 3 \
		and sa[0] == sb[0] and sa[1] == sb[1] and sa[2] == sb[2]

# ============================== 框选（编辑器） ==============================

func _on_annotate_pressed() -> void:
	# _video 以基类 Control 持有，脚本成员只能动态取
	var tex: Variant = _video.get("current_texture")
	if tex == null:
		_chat_panel.chat("提示", "先开启图传、等画面出现再框选目标")
		return
	_editor.call("open", tex)

func _on_editor_cancelled() -> void:
	pass  # 取消 = 放弃这张图，不影响输入框与已附图

## 聊天区「图传」旁路请求（/stream 由 ChatPanel 解析后交给 Main 统一起停 UDP 接收）。
func _on_chat_stream_requested(on: bool) -> void:
	_stream_toggle.set_pressed_no_signal(on)
	_apply_stream(on)

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
	# 摇杆操作 = 手动接管：打断板端 AI 闭环，聊天发送按钮恢复「发送」
	_chat_panel.set_ai_running(false)
	if cmd == Vector2.ZERO:
			# 松手/居中：停四轮 + 转向回正
			AppState.send_command(CP.drive(0))
			AppState.send_command(CP.servo(0, _SERVO_CENTER))
			return
		# 直接驱动（绕过执行板）：油门 → 全车 drive，左右 → 转向舵（逻辑 0）。
	var drive_spd := int(round(absf(throttle) * _DRIVE_MAX))
	if throttle < 0:
		drive_spd = -drive_spd
	AppState.send_command(
		CP.drive(clampi(drive_spd, -1000, 1000)))
	AppState.send_command(CP.servo(0, clampi(
		_SERVO_CENTER + int(round(steering / _JOY_STEER * _SERVO_RANGE)), 50, 250)))

func _on_joystick_pressed(_v: Variant = null) -> void:
	_joy_held = true
	_last_joy_cmd = Vector2.ZERO

func _on_joystick_release(_v: Variant = null) -> void:
	_joy_held = false
	_last_joy_cmd = Vector2.ZERO
	_chat_panel.set_ai_running(false)
	AppState.send_command(CP.drive(0))
	AppState.send_command(CP.servo(1, _SERVO_CENTER))

# ============================== 状态 ==============================

func _update_status() -> void:
	var ble: String = DeviceConn.get_ble_state()
	var ws_state: String = DeviceConn.get_ws_state()
	var online: bool = ws_state == "connected"
	var ble_on: bool = ble != "off" and ble != "unavailable"
	_ble_dot.modulate = Color.GREEN if ble_on else Color(1, 1, 1, 0.3)
	_ws_dot.modulate = Color.GREEN if online else Color(1, 1, 1, 0.3)
	_ble_stat.text = "BLE:%s" % ble
	_ws_stat.text = "WS:%s" % ("已连接" if online else "未连接")
	# 顶栏大字：连接中 / 已连接 / 未连接 三态反馈。
	# 连接判定：WS 在线优先（WS_ONLY 模式下 BLE 已让出射频、主动断开），故只要 WS 连着就算已连接，
	# 即使 BLE 断开也不显示"未连接"；BLE 断开会先看 WS——WS 也没连才落到"设备未连接"。
	var ble_ok: bool = DeviceConn.is_device_connected()
	if online:
		_conn_stat.text = ("已连接: %s" % _device_name) if _device_name != "" else "已连接(WS)"
	elif ble_ok and _device_name != "":
		_conn_stat.text = "已连接: %s" % _device_name
	elif ble == "connecting":
		_conn_stat.text = ("连接中: %s" % _pending_name) if _pending_name != "" else "连接中…"
	else:
		_conn_stat.text = "设备未连接"
