extends Control
## App 壳：组装子视图，维护连接编排与交互。
## 传输语义：/ 前缀=指令；纯文本=AI 目标（DIRECT ai_goal）。
## 所有指令经 DeviceConn.send_command：WS 优先、BLE 兜底。

const CP := preload("res://net/proto/CommandProto.gd")

@onready var _video: Control = $BodyControl/Video
@onready var _joystick: VirtualJoystick = $BodyControl/CtrlArea/Joystick
@onready var _wifi_popup: PanelContainer = $WifiPopup
@onready var _editor: Control = $ImageEditor
var _pick_dialog: FileDialog = null   # /append 选图对话框（懒建）

@onready var _scan_panel: Control = $BodyBTScan
@onready var _ble_dot: Label = $TopBar/HBox/VBoxContainer/BleStat/BleDot
@onready var _ble_stat: Label = $TopBar/HBox/VBoxContainer/BleStat
@onready var _ws_dot: Label = $TopBar/HBox/VBoxContainer/WsStat/WsDot
@onready var _ws_stat: Label = $TopBar/HBox/VBoxContainer/WsStat
@onready var _conn_stat: Label = $TopBar/HBox/ConnectionStat

@onready var _chat_panel = $BodyControl/ChatPanel
@onready var _stream_toggle: CheckButton = $BodyControl/VidControls/StreamToggle

@onready var _body_ctrl: Control = $BodyControl
@onready var _body_about: Control = $BodyAbout
@onready var _auto_conn_btn: CheckButton = $BodyAbout/Options/AutoConnOnStart
@onready var _disable_ws_btn: CheckButton = $BodyAbout/Options/DisableAutoConnWS
@onready var _spin_mode_btn: CheckButton = $BodyAbout/Options/SpinMode
@onready var _nav_bt: TextureButton = $NaviBar/HBox/BTScan
@onready var _nav_ctrl: TextureButton = $NaviBar/HBox/Control
@onready var _nav_about: TextureButton = $NaviBar/HBox/About

var _joy_held := false
var _last_joy_cmd := Vector2.ZERO  # 上一次真正下发的摇杆指令（模拟值量化后对比用）
var _last_spin_active := false     # 原地旋转模式下当前是否在转（避免斜向切换时漏停残余自转）
# 摇杆量化档：死区内不动作；速度分 低速/高速 两档；转向固定档。仅档位变化才发 move，松手发 stop。
const _JOY_DEADZONE := 0.15
const _JOY_SLOW := 0.5
const _JOY_FAST := 1.0
const _JOY_FAST_THRESH := 0.7
const _JOY_STEER := 0.8
## 直接驱动时的摇杆映射参数。
const _DRIVE_MAX := 1000       # 油门满量程 PWM
const _SERVO_CENTER := 150     # 转向舵中位
const _SERVO_RANGE := 30       # 转向舵单侧偏转量
const _SPIN_SPEED := 900       # 原地旋转模式下左右推摇杆的单轮 PWM
## 最近一次成功连接的设备名，用于顶栏「已连接: xxx」。
var _device_name := ""
var _page_tween: Tween = null
const _PAGE_DURATION := 0.28
const _PAGE_TRIGGER := 24.0   # 横向滑动达到该位移才判定为切页手势（区分纵向滚动）
const _PAGE_FLING := 600.0    # 拖动中采样的瞬时横向速度超过该值按"甩动"吸附到下/上一页（px/s）
const _VEL_SAMPLE_MS := 33    # 拖动中每隔该时长采一次速（镜像 ScrollContainer 的位移差/时长 采样）
# 横向切页手势状态：当前页 / 拖动基准 / 是否已进入切页。
var _cur_page := 0
var _swipe_start := Vector2.ZERO
var _drag_base := 0.0
var _drag_active := false
var _drag_float := 0.0
var _mouse_held := false   # 桌面兜底：仅在按住左键拖动时响应横移（避免悬停误触发）
var _swipe_skip := false   # 手势落在摇杆区域内时置真：整段不响应，避免和转向拖动冲突
var _drag_accum := 0.0      # 累计手指横向位移（当前 x - 起点 x）
var _last_sample := 0.0     # 上次采速时的累计位移
var _last_sample_tick := -1 # 上次采速时刻（ms），<0 表示尚未采速
var _vel := 0.0             # 拖动中最近一次采到的瞬时横向速度（px/s）
## 待配网设备 + 待下发 WiFi/AI（设备卡片 → 连接窗口 → 确认）。
var _pending_addr := ""
var _pending_name := ""
var _pending_provision := {}
var _pending_ai := {}

func _ready() -> void:
	# 只对统一设备连接层 DeviceConn 说话：连接其统一信号（传输事件由 DeviceConn 收口）。
	DeviceConn.scan_finished.connect(_on_scan_finished)
	DeviceConn.device_found.connect(_on_device_found)
	DeviceConn.scan_started.connect(_on_device_scan_started)
	DeviceConn.auto_scan_requested.connect(_start_scan.bind(true))
	DeviceConn.device_connected.connect(_on_device_connected)
	DeviceConn.device_disconnected.connect(_on_device_disconnected)
	DeviceConn.ws_connected.connect(_on_ws_connected)
	DeviceConn.ws_disconnected.connect(_on_ws_disconnected)
	DeviceConn.status_received.connect(_on_ble_status)
	DeviceConn.text_received.connect(_on_ws_text)
	DeviceConn.frame_received.connect(_on_frame)
	DeviceConn.state_changed.connect(_on_ble_state)
	# 扫描页设备卡片被选中 → 走连接编排。
	_scan_panel.device_selected.connect(_on_device_item_selected)
	# /append：聊天区发起的"从图库选图"由 Main 弹出系统文件选择器并打开标注编辑器。
	_chat_panel.image_pick_requested.connect(_on_chat_image_pick_requested)

	# 用 toggled + bind 页码；按钮同属一个 ButtonGroup，互斥单选。
	_nav_bt.toggled.connect(_on_nav_toggled.bind(0))
	_nav_ctrl.toggled.connect(_on_nav_toggled.bind(1))
	_nav_about.toggled.connect(_on_nav_toggled.bind(2))

	_request_ble_permissions()
	# WS 由 BLE 会话驱动（DeviceConn 在连接态自动连 WS）：不在启动时自连/心跳，
	# 等设备连接 / 板子上报 IP 再连。
	_update_status()
	# 设置项：读取本地配置并同步两个开关状态；开启启动自连时按最近设备重连。
	_auto_conn_btn.set_pressed_no_signal(Store.get_auto_conn())
	_disable_ws_btn.set_pressed_no_signal(Store.get_disable_auto_ws())
	_spin_mode_btn.set_pressed_no_signal(Store.get_spin_mode())
	if Store.get_auto_conn():
		# 启动即自动连接：直接进控制页（页 1，触发 _on_nav_toggled → _switch_page），不再停在蓝牙扫描页。
		_nav_ctrl.button_pressed = true
		_try_startup_connect()

## Android 运行时权限：BLE 扫描/连接 + 定位，启动即申请（声明见 export_presets）。
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
	_cur_page = clampi(page, 0, 2)
	_snap_to(_cur_page, true)

## 把三页各自摆到对应"页浮点" f 处（f=页序号，含拖动中的小数）：BTScan=0 / Control=1 / About=2。
func _apply_page_offset(f: float) -> void:
	_scan_panel.offset_transform_position = Vector2(-20.0 * f, 0)
	_scan_panel.offset_transform_position_ratio = Vector2(0.0 - f, 0)
	_body_ctrl.offset_transform_position = Vector2(-20.0 * f + 20, 0)
	_body_ctrl.offset_transform_position_ratio = Vector2(1.0 - f, 0)
	_body_about.offset_transform_position = Vector2(-20.0 * f + 40, 0)
	_body_about.offset_transform_position_ratio = Vector2(2.0 - f, 0)

## 吸附到最近的整页（拖动松手 / 点导航），并同步当前页与底部导航选中态。
func _snap_to(page: int, animate: bool) -> void:
	_cur_page = clampi(page, 0, 2)
	if _page_tween != null:
		_page_tween.kill()
	if not animate:
		_apply_page_offset(float(page))
	else:
		var tween = create_tween().set_parallel(true).set_trans(Tween.TRANS_CUBIC).set_ease(Tween.EASE_OUT)
		tween.tween_property(_scan_panel, "offset_transform_position", Vector2(-20.0 * page, 0), _PAGE_DURATION)
		tween.tween_property(_scan_panel, "offset_transform_position_ratio", Vector2(0.0 - page, 0), _PAGE_DURATION)
		tween.tween_property(_body_ctrl, "offset_transform_position", Vector2(-20.0 * page + 20, 0), _PAGE_DURATION)
		tween.tween_property(_body_ctrl, "offset_transform_position_ratio", Vector2(1.0 - page, 0), _PAGE_DURATION)
		tween.tween_property(_body_about, "offset_transform_position", Vector2(-20.0 * page + 40, 0), _PAGE_DURATION)
		tween.tween_property(_body_about, "offset_transform_position_ratio", Vector2(2.0 - page, 0), _PAGE_DURATION)
		_page_tween = tween
	_sync_nav(page)

## 底部导航只读同步（set_pressed_no_signal 避免回灌 toggled → _switch_page 造成重复）。
func _sync_nav(page: int) -> void:
	_nav_bt.set_pressed_no_signal(page == 0)
	_nav_ctrl.set_pressed_no_signal(page == 1)
	_nav_about.set_pressed_no_signal(page == 2)

## 画面上左右滑动切页：直接在 _input 里全量处理（不依赖 unhandled 传播，保证任何位置都响应）。
## 横移超过阈值且横向占主导才切页；落在摇杆区内整段跳过，避免和转向拖动冲突。
func _input(event: InputEvent) -> void:
	if event is InputEventScreenTouch:
		var t := event as InputEventScreenTouch
		if t.pressed:
			_swipe_start = t.position
			_drag_base = float(_cur_page)
			_drag_active = false
			_drag_float = _drag_base
			_reset_velocity()
			_swipe_skip = _joystick.get_global_rect().has_point(t.position)
		else:
			if _drag_active:
				_end_drag()
			_drag_active = false
	elif event is InputEventScreenDrag:
		if _swipe_skip:
			return
		var d := event as InputEventScreenDrag
		_track_velocity(d.position.x)
		if not _drag_active:
			var dx: float = d.position.x - _swipe_start.x
			var dy: float = d.position.y - _swipe_start.y
			if absf(dx) < _PAGE_TRIGGER or absf(dx) < absf(dy):
				return
			_drag_active = true
			_drag_base = float(_cur_page)
			if _page_tween != null:
				_page_tween.kill()
		_drag_float = clampf(_drag_base - (d.position.x - _swipe_start.x) / size.x, 0.0, 2.0)
		_apply_page_offset(_drag_float)
		get_viewport().set_input_as_handled()
	elif OS.get_name() != "Android" and event is InputEventMouseButton \
			and (event as InputEventMouseButton).button_index == MOUSE_BUTTON_LEFT:
		var mb := event as InputEventMouseButton
		if mb.pressed:
			_mouse_held = true
			_swipe_start = mb.position
			_drag_base = float(_cur_page)
			_drag_active = false
			_drag_float = _drag_base
			_reset_velocity()
			_swipe_skip = _joystick.get_global_rect().has_point(mb.position)
		else:
			_mouse_held = false
			if _drag_active:
				_end_drag()
			_drag_active = false
	elif OS.get_name() != "Android" and event is InputEventMouseMotion \
			and _mouse_held and not _swipe_skip:
		var m := event as InputEventMouseMotion
		_track_velocity(m.position.x)
		if not _drag_active:
			var dx2: float = m.position.x - _swipe_start.x
			var dy2: float = m.position.y - _swipe_start.y
			if absf(dx2) < _PAGE_TRIGGER or absf(dx2) < absf(dy2):
				return
			_drag_active = true
			_drag_base = float(_cur_page)
			if _page_tween != null:
				_page_tween.kill()
		_drag_float = clampf(_drag_base - (m.position.x - _swipe_start.x) / size.x, 0.0, 2.0)
		_apply_page_offset(_drag_float)
		get_viewport().set_input_as_handled()

## 松手吸附：默认吸附到最近整页；若拖动中采样到的横向速度很高（甩动）则顶掉速度方向相邻那一页。
func _end_drag() -> void:
	var target := clampi(roundi(_drag_float), 0, 2)
	if absf(_vel) > _PAGE_FLING:
		target = clampi(int(_drag_base) + (1 if _vel < 0.0 else -1), 0, 2)
	_snap_to(target, true)

## 按下时重置速度采样。
func _reset_velocity() -> void:
	_drag_accum = 0.0
	_last_sample = 0.0
	_last_sample_tick = -1
	_vel = 0.0

## 拖动中采速（镜像 ScrollContainer）：位移差 / 采样时长，间隔离散采样防噪声。
func _track_velocity(px: float) -> void:
	_drag_accum = px - _swipe_start.x
	var now: int = Time.get_ticks_msec()
	if _last_sample_tick >= 0:
		var dms: int = now - _last_sample_tick
		if dms >= _VEL_SAMPLE_MS:
			_vel = (_drag_accum - _last_sample) / (float(dms) / 1000.0)
			_last_sample = _drag_accum
			_last_sample_tick = now
	else:
		_last_sample = _drag_accum
		_last_sample_tick = now

# ============================== BLE ==============================

## 设置项：启动时自动连接上次设备。持久化开关；开启时立即尝试自连（含等待蓝牙就绪）。
func _on_auto_conn_toggled(on: bool) -> void:
	Store.set_auto_conn(on)
	if on:
		_try_startup_connect()

## 设置项：关闭自动建立 WS 连接（纯蓝牙控制）。仅持久化，连连接态下一次设备连接生效。
func _on_disable_ws_toggled(on: bool) -> void:
	Store.set_disable_auto_ws(on)

## 设置项：原地旋转模式——开启后左右推摇杆改为发送原地旋转(spin)，取代转向舵。
func _on_spin_mode_toggle(on: bool) -> void:
	Store.set_spin_mode(on)

## 启动（或开启自连开关）时重连上次设备。实际扫描由 DeviceConn.prepare_auto_scan(true)
## 内部门控「启动自连」并解析目标；这里仅读上次设备拿提示文案。
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
	_chat_panel.chat("提示", "启动自连：扫描并连接 %s …" % str(last.get("name", addr)))
	DeviceConn.prepare_auto_scan(true)   # 首扫：内部门控 Store.get_auto_conn()

## 轮询等待蓝牙适配器进入可用状态（Android 含运行时授权弹窗）。超时或不可用则放弃。
func _await_ble_ready() -> bool:
	for i in 600:  # 轮询等待，带超时上限
		var s: String = DeviceConn.get_ble_state()
		if s in ["idle", "scanning", "connecting", "connected"]:
			return true
		if s == "unavailable":
			return false
		await get_tree().process_frame
	return false

func _on_ble_state(_s: String) -> void:
	_update_status()

## 扫描结束：列表收尾（复位/补漏/空提示）交给扫描页，本处只做权限提示。
func _on_scan_finished(devices: Array) -> void:
	var empty: bool = _scan_panel.call("on_scan_finished", devices)
	# 整轮空列表且 Android 未授权：提示用户（权限交叉逻辑留在编排层）。
	if empty and OS.get_name() == "Android" \
			and not OS.get_granted_permissions().has("android.permission.BLUETOOTH_SCAN"):
		_chat_panel.chat("提示", "未授予蓝牙/附近设备权限，扫描不到设备——请到系统设置允许本 App 权限后刷新")

## 扫描中逐台发现：交给扫描页"边扫边显示"。
func _on_device_found(device: Dictionary) -> void:
	_scan_panel.call("on_device_found", device)

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
	# 连上后拉一次状态，同步灯/夹爪 + AI 运行态（覆盖纯蓝牙、无 WS 场景；与 _on_ws_connected 幂等）。
	await _wait_gatt_ready()
	DeviceConn.send_command(CP.get_state())
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
	# 板端 BLE status 两种形态：
	#  1) build_status 包装：{"ip",...,"reply":"<JSON>"}（词表应答/状态），无顶层 type → 解包 reply 再路由；
	#  2) send_status 直推的独立 JSON：顶层带 type（如 exec_status）。
	# 统一解出消息体后交给公共处理器（_handle_board_msg），与 WS 通道同一套展示逻辑。
	var msg := data
	if not data.has("type") and data.has("reply"):
		var r: Variant = data.get("reply")
		var j := JSON.new()
		if r is String and j.parse(r as String) == OK and j.data is Dictionary:
			msg = j.data as Dictionary
		else:
			_update_status()  # 无 reply 的纯状态 notify：只刷新 UI，无可展示文本
			return
	if msg.has("type"):
		_handle_board_msg(msg)
	_update_status()

## 从 exec_status 的 params 提取可读状态文本；无 text 回退显示原始 cmd/hex。
## 供 WS（_on_ws_text）与 BLE（_on_ble_status）两条通道共用。
func _exec_status_text(p: Variant) -> String:
	if p is Dictionary:
		var t1: Variant = (p as Dictionary).get("text")
		if t1 is String and not (t1 as String).is_empty():
			return t1 as String
		var line := "%02X" % int((p as Dictionary).get("cmd", 0))
		var hx: Variant = (p as Dictionary).get("hex")
		if hx is Array:
			var parts := PackedStringArray()
			for b in hx:
				parts.append("%02X" % int(b))
			line += " " + " ".join(parts)
		return line
	return ""

## 统一扫描入口。auto=true（启动/恢复）：同步自动目标名；
## auto=false（手动）：打断自动重连 + 复位连接占位 + 开扫。
## 列表不在开扫时清空：由扫描页在真正显示新设备时才清旧列表（见 ScanPanel），避免扫描间隙空白。
func _start_scan(auto: bool) -> void:
	if auto:
		_pending_name = DeviceConn.auto_target_name()   # 顶栏 "连接中: xxx"
	else:
		DeviceConn.cancel_auto_reconnect()
		_pending_addr = ""
		_pending_name = ""
	DeviceConn.scan()

func _on_refresh_toggled(pressed_on: bool) -> void:
	# toggle 按下 → 统一入口（打断自动重连 + 开扫），松开 → 统一停扫（恢复扫描一并打断）。
	if pressed_on:
		_start_scan(false)
	else:
		DeviceConn.stop_scan()

## 任何一次扫描（手动 / 双断恢复）都由 DeviceConn 的上报驱动同一套扫描按钮 + 动画，保证统一、可打断。
func _on_device_scan_started() -> void:
	_scan_panel.call("show_scanning")

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

## get_state 回传（type:"state"）：params.bits 为板端状态位字节，按位同步直控按钮（灯/夹爪/AI 运行态）。
func _apply_state(data: Dictionary) -> void:
	var st: Variant = data.get("params")
	if st is Dictionary:
		var b: Variant = (st as Dictionary).get("bits")
		# Godot JSON 解析把数字存成 float（typeof=3），兼容 int/float。
		if b is int or b is float:
			_apply_state_bits(int(b))

## 按板端状态位字节同步直控按钮（灯/夹爪/AI 运行态）。WS 与 BLE 通道共用。
## bit 布局与 Stm32-Vision/command.cpp（make_state_bits）逐位 mirror，改一侧必改另一侧：
##   bit0 前灯 / bit1 震灯 / bit2 背灯 / bit3 夹爪夹紧 / bit4 AI busy；bit5-7 留空。
func _apply_state_bits(bits: int) -> void:
	$BodyControl/CtrlArea.call("sync_state", {
		"front": (bits & 1) != 0,
		"vibe": (bits & 2) != 0,
		"back": (bits & 4) != 0,
	}, (bits & 8) != 0)
	_chat_panel.set_ai_running((bits & 16) != 0)

func _on_ws_connected() -> void:
	_chat_panel.chat("板", "WS 已连接")
	_update_status()
	# 连上后主动拉一次当前状态，同步直控面板按钮（灯/夹爪），避免重连后状态不一致。
	DeviceConn.send_command(CP.get_state())
	# WS 建立即进入 WS_ONLY（让出 BLE 射频）由 DeviceConn 在内部处理。
	# 重连后按图传开关当前状态重发开启指令：断连期间开关保持「开」但板子画面已断，需重发恢复。
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
	DeviceConn.current_image = img

func _on_ws_text(data: Dictionary) -> void:
	_handle_board_msg(data)

## 板端上行消息统一处理（AI 结果/日志/状态/词表回执），WS 与 BLE 两条通道共用：
## 按顶层 type 路由展示；返回 true 表示已消费，false 表示未知类型（留给调用方兜底）。
func _handle_board_msg(data: Dictionary) -> bool:
	var t: String = str(data.get("type", ""))
	match t:
		"pong":
			_chat_panel.chat("板", "pong")
		"ai_result":
			_chat_panel.show_ai_result(data)
		"log":
			# 板端统一日志模块回推（/log <exec|ai|all> on 开启）：来源+文本，展示并落盘。
			# 板端发 {type:"log", params:{src:"exec|ai|...", text:"..."}}。
			var lp: Variant = data.get("params")
			if lp is Dictionary:
				var lsrc := str((lp as Dictionary).get("src", ""))
				var ltv: Variant = (lp as Dictionary).get("text")
				if ltv is String:
					# ai 类别用「AI日志」样式单列（ChatPanel 已有该分支，颜色与「日志」区分）：
					# 全类别混进同一个灰色「日志」区时，AI 行为被 cmd/exec 的噪声淹掉，
					# 看起来就像"AI 日志不发了"。
					if lsrc == "ai":
						_chat_panel.chat("AI日志", ltv as String)
					else:
						_chat_panel.chat("日志", "[%s] %s" % [lsrc, ltv as String])
		"state":
			# get_state 回传：同步直控按钮（灯/夹爪/AI 运行态）。
			_apply_state(data)
		"exec_status":
			_chat_panel.chat("执行板", _exec_status_text(data.get("params")))
		"status":
			# status 回执：展示可读 reason，并按其附带的状态位（bits，若存在）同步直控按钮
			# （reset / ai_cancel / light 等"会触发动作重置"的回执都自动带上 bits）。
			var line2 := ""
			var pm: Variant = data.get("params")
			if pm is Dictionary:
				var pd := pm as Dictionary
				if pd.has("reason"):
					line2 = str(pd.get("reason"))
				var b: Variant = pd.get("bits")
				# Godot JSON 解析把数字存成 float（typeof=3），兼容 int/float。
				if b is int or b is float:
					_apply_state_bits(int(b))
			if line2 != "":
				_chat_panel.chat("板", line2)
		_:
			return false
	return true

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
	DeviceConn.send_command(CP.stream(on, port if port > 0 else 0, _my_ipv4()))
	_video.visible = on

## 取手机非回环 IPv4 本机地址（板端建 UDP 会话用，见 CommandProto.stream）。
## 优先挑与板子（WS 对端）同网段的地址：手机可能带 VPN/虚拟网卡，直接取首个非回环地址
## 可能把隧道 IP 报给板子，板端 UDP 发到该地址不可达 → 手机收不到画面。
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

## /append：打开系统文件选择器选一张图片（懒建 FileDialog）。
## 选择后压缩到目标大小、用现有标注编辑器标注，采用后进入附件列表（Main.tscn 已连 image_sent）。
func _on_chat_image_pick_requested() -> void:
	if _pick_dialog == null:
		_pick_dialog = FileDialog.new()
		_pick_dialog.title = "选择图片"
		_pick_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
		_pick_dialog.access = FileDialog.ACCESS_FILESYSTEM
		_pick_dialog.use_native_dialog = true  # Android 上走系统 SAF 文件选择器（4.6+ 内置，无需插件）
		_pick_dialog.filters = PackedStringArray(["*.png ; PNG 图片", "*.jpg ; JPG 图片", "*.jpeg ; JPEG 图片"])
		_pick_dialog.file_selected.connect(_on_pick_file)
		var pics: String = OS.get_system_dir(OS.SYSTEM_DIR_PICTURES)
		if not pics.is_empty():
			_pick_dialog.current_dir = pics
		add_child(_pick_dialog)
	_pick_dialog.popup_centered()

func _on_pick_file(path: String) -> void:
	var img := Image.new()
	if img.load(path) != OK:
		_chat_panel.chat("提示", "图片读取失败: %s" % path)
		return
	img = _compress_to_target(img, 20480)  # 压缩到目标大小，避免把大图发给板端/AI
	if img == null:
		_chat_panel.chat("提示", "图片压缩失败（未能压到目标大小内）")
		return
	_editor.call("open", ImageTexture.create_from_image(img))

## 把图压到 ≤ max_bytes（JPEG）：先降质量，仍超则等比缩宽后再降质，返回压缩后 Image。
## 0=用原宽（仅降质）；宽度从大到小、质量从高到低，命中即返回（尽量清晰）。
func _compress_to_target(img: Image, max_bytes: int) -> Image:
	var widths := [0, 1600, 1280, 1024, 800, 640, 480, 360]
	var qualities := [0.88, 0.76, 0.64, 0.52, 0.40, 0.30]
	for w: int in widths:
		var cur: Image = img
		if w > 0 and img.get_width() > w:
			cur = img.duplicate()
			cur.resize(w, maxi(int(round(img.get_height() * float(w) / float(img.get_width()))), 1))
		for q: float in qualities:
			var buf: PackedByteArray = cur.save_jpg_to_buffer(q)
			if buf.size() > 0 and buf.size() <= max_bytes:
				var out := Image.new()
				if out.load_jpg_from_buffer(buf) == OK:
					return out
	return null

func _on_editor_cancelled() -> void:
	pass  # 取消 = 放弃这张图，不影响输入框与已附图

## 聊天区「图传」旁路请求（/stream 由 ChatPanel 解析后交给 Main 统一起停 UDP 接收）。
func _on_chat_stream_requested(on: bool) -> void:
	_stream_toggle.set_pressed_no_signal(on)
	_apply_stream(on)

## 图传标定网格叠加（/grid 本地开关，配合单应标定测量）。
func _on_chat_grid_requested(on: bool) -> void:
	_video.call("show_grid", on)

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
	# 原地旋转模式下摇杆斜向（左右+前后同时有效）：原地转与前进物理互斥，忽略转向、只前进/后退。
	# 普通（转向舵）模式保留"一边转弯一边前进"的原有行为。
	var spin_mode: bool = _spin_mode_btn.button_pressed
	var diagonal: bool = spin_mode and absf(v.x) >= _JOY_DEADZONE and absf(v.y) >= _JOY_DEADZONE
	# cmd 仅做"是否变化"的去重；斜向时 steering 归零，避免单独触发自转。
	var cmd := Vector2(throttle, 0.0 if diagonal else steering)
	if cmd == _last_joy_cmd:
		return
	_last_joy_cmd = cmd
	# 摇杆操作 = 手动接管：打断板端 AI 闭环，聊天发送按钮恢复「发送」
	_chat_panel.set_ai_running(false)
	if cmd == Vector2.ZERO:
		# 松手/居中：停四轮；原地旋转模式下停旋转（复位自转态），否则转向回正
		DeviceConn.send_command(CP.drive(0))
		if spin_mode:
			DeviceConn.send_command(CP.spin(0))
			_last_spin_active = false
		else:
			DeviceConn.send_command(CP.servo(0, _SERVO_CENTER))
		return
	# 直接驱动：油门 → 全车 drive；左右 → 转向舵；原地旋转模式下左右推改为原地旋转。
	var drive_spd := int(round(absf(throttle) * _DRIVE_MAX))
	if throttle < 0:
		drive_spd = -drive_spd
	DeviceConn.send_command(CP.drive(clampi(drive_spd, -1000, 1000)))
	if spin_mode:
		# 原地旋转模式：仅纯左右（非斜向）才自转；斜向/前进时若此前在转则补停残余自转。
		var want_spin: bool = (not diagonal) and absf(steering) >= _JOY_DEADZONE
		var want_dir: int = 1 if steering > 0 else -1
		if want_spin != _last_spin_active:
			DeviceConn.send_command(CP.spin(want_dir if want_spin else 0, _SPIN_SPEED))
			_last_spin_active = want_spin
	else:
		DeviceConn.send_command(CP.servo(0, clampi(
			_SERVO_CENTER + int(round(steering / _JOY_STEER * _SERVO_RANGE)), 50, 250)))

func _on_joystick_pressed(_v: Variant = null) -> void:
	_joy_held = true
	_last_joy_cmd = Vector2.ZERO
	_last_spin_active = false

func _on_joystick_release(_v: Variant = null) -> void:
	_joy_held = false
	_last_joy_cmd = Vector2.ZERO
	_last_spin_active = false
	_chat_panel.set_ai_running(false)
	DeviceConn.send_command(CP.drive(0))
	if _spin_mode_btn.button_pressed:
		DeviceConn.send_command(CP.spin(0))
	else:
		DeviceConn.send_command(CP.servo(1, _SERVO_CENTER))

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
	# 顶栏大字三态：WS 在线优先判定（WS_ONLY 下 BLE 已让出射频），WS 连着即算已连接。
	var ble_ok: bool = DeviceConn.is_device_connected()
	if online:
		_conn_stat.text = ("已连接: %s" % _device_name) if _device_name != "" else "已连接(WS)"
	elif ble_ok and _device_name != "":
		_conn_stat.text = "已连接: %s" % _device_name
	elif ble == "connecting":
		_conn_stat.text = ("连接中: %s" % _pending_name) if _pending_name != "" else "连接中…"
	else:
		_conn_stat.text = "设备未连接"
