extends Control
## App 壳：组装子视图，维护连接编排与交互。
## 传输语义：/ 前缀=指令；纯文本=AI 目标（DIRECT ai_goal）。
## 所有指令经 DeviceConn.send_command：WS 优先、BLE 兜底。
## 布局：手动控制 / AI 接管 两模式，可见性统一由 _apply_layout 收口（不再散在各处改 visible）。

const CP := preload("res://net/proto/CommandProto.gd")

# ------------------------------ 视图引用 ------------------------------
@onready var _video: Control = $BodyControl/Video
@onready var _wifi_popup: PanelContainer = $PopupWindow
@onready var _editor: Control = $BodyControl/ImgEditToolbar
@onready var _annotate_btn: Button = $BodyControl/AIModeCtrl/AnnotateBtn
@onready var _dim: ColorRect = $BGDimSharder

@onready var _scan_panel: Control = $BodyBTScan
@onready var _ble_dot: Label = $TopBar/HBox/VBoxContainer/BleStat/BleDot
@onready var _ble_stat: Label = $TopBar/HBox/VBoxContainer/BleStat
@onready var _ws_dot: Label = $TopBar/HBox/VBoxContainer/WsStat/WsDot
@onready var _ws_stat: Label = $TopBar/HBox/VBoxContainer/WsStat
@onready var _conn_stat: Label = $TopBar/HBox/ConnectionStat

@onready var _chat_panel = $BodyControl/ChatPanel
@onready var _session_panel: Control = $BodyControl/ChatPanel/ChatLog/SessionPanel
@onready var _save_btn: Button = $BodyControl/ChatPanel/ChatLog/SessionPanel/VBox/Title/SaveBtn
@onready var _playback_bar: Control = $BodyControl/ChatPanel/ChatLog/BottomPanl/PlaybackCtrl
@onready var _stream_toggle: CheckButton = $BodyControl/GeneralCtrl/StreamToggle
@onready var _ctrl_mode_btn: Button = $BodyControl/GeneralCtrl/CtrlModeToggle
@onready var _source_btn: Button = $BodyControl/AIModeCtrl/StreamToggle
@onready var _ctrl_area: Control = $BodyControl/CtrlArea
@onready var _joystick: Control = $BodyControl/CtrlArea/Joystick
@onready var _bottom_padding: Panel = $BodyControl/BottomPadding

## AI 接管模式才显示的组件（批量显隐：以后增删组件只改这一处）。
@onready var _aimode_row: Control = $BodyControl/AIModeCtrl
@onready var _append_img_btn: Control = $BodyControl/ChatPanel/InputRow/MessageInput/AppendImgBtn
## 整栏：回放期间也要留着 —— SessionBtn 是回放列表的唯一入口，藏起来就退不出回放了。
@onready var _ai_only: Array[Control] = [_aimode_row]
## 其内的输入类组件：回放中只读，收起。
@onready var _ai_inputs: Array[Control] = [_append_img_btn]

## 手动控制模式才显示的组件（AI 接管时收起底部控制区，把屏幕让给图传/聊天/回放）。
@onready var _manual_only: Array[Control] = [_ctrl_area]

@onready var _body_ctrl: Control = $BodyControl
@onready var _body_about: Control = $BodyAbout
@onready var _auto_conn_btn: CheckButton = $BodyAbout/Options/AutoConnOnStart
@onready var _disable_ws_btn: CheckButton = $BodyAbout/Options/DisableAutoConnWS
@onready var _spin_mode_btn: CheckButton = $BodyAbout/Options/SpinMode
@onready var _nav_bt: TextureButton = $NaviBar/HBox/BTScan
@onready var _nav_ctrl: TextureButton = $NaviBar/HBox/Control
@onready var _nav_about: TextureButton = $NaviBar/HBox/About

# ------------------------------ 模式 / 画面源 / 图传 ------------------------------
var _ai_mode := false      # false=手动控制，true=AI 接管
var _map_on := false       # 画面源：false=图传，true=记忆地图
var _stream_on := false    # 图传开关（持久化）
var _editing := false      # 标注中：图传冻结，画面停在最后一帧
## 弹窗「确认」要执行的动作（如删除会话）；由 _on_popup_confirmed 消费。
var _confirm_action: Callable = Callable()

## 最近一次成功连接的设备名，用于顶栏「已连接: xxx」。
var _device_name := ""
var _page_tween: Tween = null
const _PAGE_DURATION := 0.28
const _PAGE_TRIGGER := 24.0   # 横向滑动达到该位移才判定为切页手势（区分纵向滚动）
const _PAGE_FLING := 600.0    # 拖动中采样的瞬时横向速度超过该值按"甩动"吸附到下/上一页（px/s）
const _VEL_SAMPLE_MS := 33    # 拖动中每隔该时长采一次速
# 横向切页手势状态：当前页 / 拖动基准 / 是否已进入切页。
var _cur_page := 0
var _swipe_start := Vector2.ZERO
var _drag_base := 0.0
var _drag_active := false
var _drag_float := 0.0
var _mouse_held := false   # 桌面兜底：仅在按住左键拖动时响应横移（避免悬停误触发）
var _swipe_skip := false   # 手势落在摇杆区域内时置真：整段不响应，避免和转向拖动冲突
var _drag_accum := 0.0     # 累计手指横向位移
var _last_sample := 0.0    # 上次采速时的累计位移
var _last_sample_tick := -1
var _vel := 0.0
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
	DeviceConn.frame_jpeg.connect(Recorder.record_frame)  # 原始 JPEG 直接进录制（不在录制态时 Recorder 自丢）
	DeviceConn.state_changed.connect(_on_ble_state)
	# 扫描页设备卡片被选中 → 走连接编排。
	_scan_panel.device_selected.connect(_on_device_item_selected)
	# 选图（/append 与输入框附图按钮）→ 打开标注编辑器。
	_chat_panel.image_picked.connect(_on_chat_image_picked)
	# 标注工具条（非模态，浮在图传上方）：采用/取消由编辑器回抛，Main 统一收尾。
	_editor.connect("image_sent", _on_editor_image_sent)
	_editor.connect("cancelled", _on_editor_cancelled)
	_editor.connect("grasp_requested", _on_editor_grasp_requested)
	# 直控面板：摇杆手动接管 / 板端 AI 运行态回抛。
	_ctrl_area.connect("manual_takeover", _on_manual_takeover)
	_ctrl_area.connect("ai_busy_changed", _on_ai_busy_changed)
	# 通用弹窗：配网确认 / 删除会话确认。
	_wifi_popup.connect("provision_confirmed", _on_wifi_confirmed)
	_wifi_popup.connect("confirmed", _on_popup_confirmed)
	# 会话回放：列表选择/删除 + 控制条 + 录制器信号。
	_session_panel.connect("session_selected", _on_session_selected)
	_session_panel.connect("delete_requested", _on_session_delete_requested)
	_playback_bar.connect("step_requested", _on_replay_step)
	_playback_bar.connect("toggle_pause_requested", _on_replay_toggle_pause)
	_playback_bar.connect("seek_requested", _on_replay_seek)
	_wifi_popup.connect("cancelled", _on_popup_cancelled)
	_chat_panel.session_cleared.connect(_on_session_cleared)
	Recorder.replay_event.connect(_on_replay_event)
	Recorder.replay_frame.connect(_on_replay_frame)
	Recorder.replay_seeked.connect(_on_replay_seeked)
	Recorder.replay_progress.connect(_on_replay_progress)
	Recorder.replay_finished.connect(_on_replay_finished)
	# 配网弹窗为模态：背景遮罩随其显隐同步（visibility_changed 已在 tscn 连接）。
	_dim.connect("tapped", _on_dim_tapped)
	_sync_modal_dim()

	# 用 toggled + bind 页码；按钮同属一个 ButtonGroup，互斥单选。
	_nav_bt.toggled.connect(_on_nav_toggled.bind(0))
	_nav_ctrl.toggled.connect(_on_nav_toggled.bind(1))
	_nav_about.toggled.connect(_on_nav_toggled.bind(2))

	_request_ble_permissions()
	_update_status()

	# 设置项：读取本地配置并同步开关状态；开启启动自连时按最近设备重连。
	_auto_conn_btn.set_pressed_no_signal(Store.get_auto_conn())
	_disable_ws_btn.set_pressed_no_signal(Store.get_disable_auto_ws())
	_spin_mode_btn.set_pressed_no_signal(Store.get_spin_mode())
	_ctrl_area.call("set_spin_mode", _spin_mode_btn.button_pressed)

	# 模式 / 图传 / 画面源：按上次退出时的状态恢复，再统一应用一次布局。
	_ai_mode = Store.get_ai_mode()
	_map_on = false
	_stream_on = Store.get_stream_on()
	_stream_toggle.set_pressed_no_signal(_stream_on)
	_apply_layout()
	# 图传真正的起流待 WS 连上后由 _on_ws_connected 重发（此刻还没有链路可发）。
	if Store.get_auto_conn():
		# 启动即自动连接：直接进控制页（页 1，触发 _on_nav_toggled → _switch_page）。
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

# ============================== 布局 / 模式 ==============================

## 一处收口"哪些组件该显示"：模式、回放、图传、标注、画面源。
## 任何会改变可见性的地方都只调它，别再各自改 visible。
func _apply_layout() -> void:
	var replaying: bool = Recorder.is_playing()
	for n in _ai_only:
		n.visible = _ai_mode
	var ai_inputs: bool = _ai_mode and not replaying
	for n in _ai_inputs:
		n.visible = ai_inputs
	# 底部控制区只在「手动控制」显示：AI 接管（和回放）时收起。
	for n in _manual_only:
		n.visible = not _ai_mode and not replaying
	if not _ctrl_area.visible:
		# 收起控制区时务必松开摇杆：隐藏正在拖拽的摇杆可能收不到 release，把车留在"前进"。
		_ctrl_area.call("release_all")
	_ctrl_mode_btn.text = "模式：AI接管" if _ai_mode else "模式：手动控制"
	_ctrl_mode_btn.set_pressed_no_signal(_ai_mode)
	var show_map: bool = _ai_mode and _map_on and not replaying
	_source_btn.set_pressed_no_signal(_map_on)
	_source_btn.text = "画面源：记忆" if _map_on else "画面源：图传"
	_video.call("show_map", show_map)
	_video.visible = _stream_on or _editing or replaying or show_map
	_chat_panel.call("set_input_enabled", not replaying)
	# 回放是只读态：下列控件都与「实时图传」或「live 记忆地图」相关，在回放里是 no-op，
	# 禁掉它们，免得点下去没反应（画面源切记忆、图传开关在回放里都无效）让人以为坏了。
	# 回放列表入口(SessionBtn)必须保持可用 —— 它是回放的唯一出口。
	_source_btn.disabled = replaying            # 画面源：回放只画录制帧，记忆地图无意义
	_stream_toggle.disabled = replaying or _editing   # 图传开关：回放时实时流无效；标注时冻结也禁
	_ctrl_mode_btn.disabled = replaying         # 模式切换：切到手动会把人踢出回放
	_annotate_btn.disabled = replaying          # 标注：在回放帧上框选无意义
	_sync_bottom_padding()

## 模式切换（tscn: GeneralCtrl/CtrlModeToggle）。
func _on_ctrl_mode_toggled(on: bool) -> void:
	_ai_mode = on
	Store.set_ai_mode(on)
	if not on:
		_map_on = false                  # 退出 AI 接管：画面源回到图传
		_session_panel.call("close")     # 回放列表只属于 AI 接管
		if Recorder.is_playing():
			Recorder.stop_play()          # 手动模式没有回放入口，别把人困在只读态
	_apply_layout()

## 画面源切换（tscn: AIModeCtrl/StreamToggle，仅 AI 接管模式可见）。
func _on_source_toggled(on: bool) -> void:
	_map_on = on
	_apply_layout()

## 会话回放列表入口（tscn: AIModeCtrl/SessionBtn）。
func _on_session_btn_pressed() -> void:
	_session_panel.call("toggle")

## 保存当前会话（tscn: ChatPanel/ChatLog/SessionPanel/VBox/Title/SaveBtn）。
## 与 /clear 等价（归档当前趟 + 开新会话）；但无可保存内容（没跑过任务/空趟）或回放中时弹窗提醒，不静默丢弃。
func _on_save_pressed() -> void:
	if Recorder.is_playing():
		_wifi_popup.call("open_confirm", "回放中无法保存", "正在回放历史会话，不能保存当前会话。退出回放后再试。", "知道了")
		_confirm_action = Callable()   # 纯提醒：确认即关闭，不执行任何动作
		return
	if not Recorder.has_session_to_save():
		_wifi_popup.call("open_confirm", "暂无可保存的会话", "当前会话还没有执行过任务，没有内容可以保存。", "知道了")
		_confirm_action = Callable()   # 同上：纯提醒
		return
	_chat_panel.call("save_current_session")

## 摇杆一动 = 手动接管：打断板端 AI 闭环，发送按钮复位「发送」。
func _on_manual_takeover() -> void:
	_chat_panel.set_ai_running(false)

## 板端 AI 运行态（状态位 bit4）回抛。
func _on_ai_busy_changed(busy: bool) -> void:
	_chat_panel.set_ai_running(busy)

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

## 把三页各自摆到对应"页浮点" f 处（f=页序号，含拖动中的小数）。
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

## 画面上左右滑动切页：走 _gui_input（Godot 的 GUI 派发，谁挡住谁消费）。
## 背景遮罩与标注画布都是 mouse_filter=STOP 且盖在上层，触点被它们吃掉，这里自然收不到。
## 横移超过阈值且横向占主导才切页；落在摇杆区内整段跳过，避免和转向拖动冲突。
func _gui_input(event: InputEvent) -> void:
	if event is InputEventScreenTouch:
		var t := event as InputEventScreenTouch
		if t.pressed:
			_swipe_start = t.position
			_drag_base = float(_cur_page)
			_drag_active = false
			_drag_float = _drag_base
			_reset_velocity()
			_swipe_skip = _ctrl_area.visible and _joystick.get_global_rect().has_point(t.position)
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
			_swipe_skip = _ctrl_area.visible and _joystick.get_global_rect().has_point(mb.position)
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

# ============================== 模态弹窗 ==============================
# 背景遮罩 BGDimSharder 挂在 Main 根下：显隐跟随弹窗的 visibility_changed（tscn 连接），
# 点空白处关闭交给遮罩自己上报。

func _modal_open() -> bool:
	return _wifi_popup.visible

func _sync_modal_dim() -> void:
	_dim.visible = _modal_open()

## 点遮罩空白处关闭 = 取消。
func _on_dim_tapped() -> void:
	_wifi_popup.call("close")

## 弹窗「确认」（Message 页）：执行挂起的动作。
func _on_popup_confirmed() -> void:
	var act := _confirm_action
	_confirm_action = Callable()
	if act.is_valid():
		act.call()

# ============================== BLE ==============================

## 设置项：启动时自动连接上次设备。
func _on_auto_conn_toggled(on: bool) -> void:
	Store.set_auto_conn(on)
	if on:
		_try_startup_connect()

## 设置项：关闭自动建立 WS 连接（纯蓝牙控制）。
func _on_disable_ws_toggled(on: bool) -> void:
	Store.set_disable_auto_ws(on)

## 设置项：原地旋转模式——开启后左右推摇杆改为发送原地旋转(spin)，取代转向舵。
func _on_spin_mode_toggle(on: bool) -> void:
	Store.set_spin_mode(on)
	_ctrl_area.call("set_spin_mode", on)

## 启动（或开启自连开关）时重连上次设备。
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
	for i in 600:
		var s: String = DeviceConn.get_ble_state()
		if s in ["idle", "scanning", "connecting", "connected"]:
			return true
		if s == "unavailable":
			return false
		await get_tree().process_frame
	return false

func _on_ble_state(_s: String) -> void:
	_update_status()

## 扫描结束：列表收尾交给扫描页，本处只做权限提示。
func _on_scan_finished(devices: Array) -> void:
	var empty: bool = _scan_panel.call("on_scan_finished", devices)
	if empty and OS.get_name() == "Android" \
			and not OS.get_granted_permissions().has("android.permission.BLUETOOTH_SCAN"):
		_chat_panel.chat("提示", "未授予蓝牙/附近设备权限，扫描不到设备——请到系统设置允许本 App 权限后刷新")

func _on_device_found(device: Dictionary) -> void:
	_scan_panel.call("on_device_found", device)

func _on_device_item_selected(name: String, address: String) -> void:
	# 点击设备卡片：暂存目标，弹出配网/连接窗口。
	_pending_addr = address
	_pending_name = name
	_wifi_popup.call("open_provision", name)

func _on_device_connected(_address: String, name: String) -> void:
	_device_name = name
	_update_status()
	# BLE 已连接（会话开始）：发起 WS（默认软 AP 地址；板子上报 IP 时 DeviceConn 会用 ws://ip 覆盖）。
	if not DeviceConn.get_ws_state() in ["connected", "connecting"] and not Store.get_disable_auto_ws():
		DeviceConn.connect_ws()
	_chat_panel.chat("提示", "已连接设备 %s" % name)
	# 携带配网/AI 请求（设备卡片 → 连接窗口 → 确认），连上且 GATT 就绪后下发。
	var wifi: Dictionary = _pending_provision
	var ai: Dictionary = _pending_ai
	_pending_provision = {}
	_pending_ai = {}
	# 连上后拉一次状态，同步灯/夹爪 + AI 运行态。
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
	_chat_panel.chat("提示", "设备已断开: %s" % reason)

func _on_ble_status(data: Dictionary) -> void:
	# 板端 BLE status 两种形态（build_status 包装 / send_status 直推），解包后走同一套展示逻辑。
	var _b: Variant = data.get("bits")
	if _b is int or _b is float:
		_apply_state_bits(int(_b))
	var msg := data
	if not data.has("type") and data.has("reply"):
		var r: Variant = data.get("reply")
		var j := JSON.new()
		if r is String and j.parse(r as String) == OK and j.data is Dictionary:
			msg = j.data as Dictionary
		else:
			_update_status()
			return
	if msg.has("type"):
		_handle_board_msg(msg)
	_update_status()

## 从 exec_status 的 params 提取可读状态文本；无 text 回退显示原始 cmd/hex。
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

## 统一扫描入口。auto=true（启动/恢复）；auto=false（手动）。
func _start_scan(auto: bool) -> void:
	if auto:
		_pending_name = DeviceConn.auto_target_name()
	else:
		DeviceConn.cancel_auto_reconnect()
		_pending_addr = ""
		_pending_name = ""
	DeviceConn.scan()

func _on_refresh_toggled(pressed_on: bool) -> void:
	if pressed_on:
		_start_scan(false)
	else:
		DeviceConn.stop_scan()

func _on_device_scan_started() -> void:
	_scan_panel.call("show_scanning")

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

## get_state 回传（type:"state"）：params.bits 为板端状态位字节，按位同步直控按钮。
func _apply_state(data: Dictionary) -> void:
	var st: Variant = data.get("params")
	if st is Dictionary:
		var b: Variant = (st as Dictionary).get("bits")
		if b is int or b is float:
			_apply_state_bits(int(b), true)

## 按板端状态位字节同步直控按钮（WS 与 BLE 通道共用）。
## apply_ai=true 仅在主动 get_state/reconnect 首同步时传：此时按 bit4 初始化「中止/发送」。
## bit 布局与 Stm32-Vision/command.cpp（state_bits）逐位 mirror，改一侧必改另一侧。
func _apply_state_bits(bits: int, apply_ai: bool = false) -> void:
	_ctrl_area.call("apply_state_bits", bits, apply_ai)

func _on_ws_connected() -> void:
	_chat_panel.chat("板", "WS 已连接")
	_update_status()
	DeviceConn.send_command(CP.get_state())
	# 重连后按图传开关当前状态重发开启指令：断连期间开关保持「开」但板子画面已断。
	if _stream_on:
		_apply_stream(true)

func _on_ws_disconnected(reason: String) -> void:
	_video.call("show_no_signal", true)
	_video.call("clear_track")  # 掉线：跟踪叠加一并清掉
	DeviceConn.stop_video()  # WS 掉线：UDP 对端随之失效，停接收
	_chat_panel.set_ai_running(false)  # 掉线即任务中断：按钮复位「发送」
	_update_status()
	var r := reason
	if r.is_empty():
		r = "连接中断"
	_chat_panel.chat("板", "WS 已断开:%s" % r)

func _on_frame(img: Image) -> void:
	# 标注中：画面冻结在进入标注时的那一帧；回放中：实时帧不许顶掉回放画面。
	if _editing or Recorder.is_playing():
		return
	_video.call("set_frame", img)
	DeviceConn.current_image = img

func _on_ws_text(data: Dictionary) -> void:
	_handle_board_msg(data)

## 板端上行消息统一处理（AI 结果/日志/状态/词表回执），WS 与 BLE 两条通道共用。
func _handle_board_msg(data: Dictionary) -> bool:
	# 回放中一律不展示：否则实时消息会插进回放的时间线里。
	if Recorder.is_playing():
		return true
	var t: String = str(data.get("type", ""))
	match t:
		"pong":
			# 板端保活应答：带当场状态位（bits），只同步不打印（否则每几秒一行噪声）。
			var pp: Variant = data.get("params")
			if pp is Dictionary:
				var pb: Variant = (pp as Dictionary).get("bits")
				if pb is int or pb is float:
					_apply_state_bits(int(pb), true)
		"ai_result":
			_chat_panel.show_ai_result(data)
		"ai_task":
			_chat_panel.show_task(data.get("params"))
		"ai_mem":
			# 板端物体记忆 + 车姿态快照：喂给记忆地图（画面源=记忆时画）。
			_video.call("show_map_data", data.get("params"))
		"track":
			# 板端跟踪器目标位置（归一化 u/v + 置信度 + 状态）：叠加层画十字/圆圈。
			# st=idle 表示跟踪已停：清掉叠加，避免残留旧十字。
			# novid=1：跟踪期板端**不推视频**（省掉整幅软编，真机实测夹取每步快 ~1.5×）→ 手机改显占位。
			var st_t := str(data.get("st", ""))
			_video.call("set_track_novid", bool(data.get("novid", false)))
			if st_t == "idle" or not data.has("u"):
				_video.call("clear_track")
			else:
				_video.call("set_track_target", float(data.get("u", 0.0)),
					float(data.get("v", 0.0)), float(data.get("conf", 0.0)), st_t)
		"ai_tool":
			var tp: Variant = data.get("params")
			if tp is Dictionary:
				var ttv: Variant = (tp as Dictionary).get("text")
				if ttv is String and not (ttv as String).is_empty():
					_chat_panel.chat("AI工具", ttv as String)
		"log":
			var lp: Variant = data.get("params")
			if lp is Dictionary:
				var lsrc := str((lp as Dictionary).get("src", ""))
				var ltv: Variant = (lp as Dictionary).get("text")
				if ltv is String:
					if lsrc == "ai":
						_chat_panel.chat("AI日志", ltv as String)
					else:
						_chat_panel.chat("日志", "[%s] %s" % [lsrc, ltv as String])
		"state":
			_apply_state(data)
		"exec_status":
			_chat_panel.chat("状态", _exec_status_text(data.get("params")))
		"status":
			var line2 := ""
			var pm: Variant = data.get("params")
			if pm is Dictionary:
				var pd := pm as Dictionary
				if pd.has("reason"):
					line2 = str(pd.get("reason"))
				var b: Variant = pd.get("bits")
				if b is int or b is float:
					_apply_state_bits(int(b))
			if line2 != "":
				_chat_panel.chat("板", line2)
		_:
			return false
	return true

func _on_stream_toggled(on: bool) -> void:
	Store.set_stream_on(on)
	_apply_stream(on)

## 控制区收起时输入框贴底、软键盘会盖住它：撑起底部占位把聊天区抬起来，键盘收起即还原。
## 弹窗/回放期间不抬（弹窗自带输入框，回放的输入本就禁用）。
func _sync_bottom_padding() -> void:
	var want: bool = not _modal_open() and not _ctrl_area.visible and not Recorder.is_playing() \
		and DisplayServer.has_feature(DisplayServer.FEATURE_VIRTUAL_KEYBOARD) \
		and DisplayServer.virtual_keyboard_get_height() > 0
	if _bottom_padding.visible != want:
		_bottom_padding.visible = want

## 图传开关统一出口：开 → 先起 UDP 接收拿本地端口，再发 stream(udp_port)；
## 关 → 停 UDP 接收并发 stream off。_on_ws_connected / /stream 均走这里。
func _apply_stream(on: bool) -> void:
	_stream_on = on
	var port := -1
	if on:
		port = DeviceConn.start_video()
		if port < 0:
			_chat_panel.chat("提示", "UDP 图传初始化失败")
	DeviceConn.send_command(CP.stream(on, port if port > 0 else 0, _my_ipv4()))
	_apply_layout()

## 取手机非回环 IPv4 本机地址（板端建 UDP 会话用）。优先挑与板子（WS 对端）同网段的地址。
func _my_ipv4() -> String:
	var host: String = DeviceConn.board_ip()
	for a in IP.get_local_addresses():
		if _is_usable_ipv4(a) and not host.is_empty() and _same_subnet(a, host):
			return a
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

## 「框选目标」开关：开 → 冻结图传取当前帧进标注；关 → 关闭工具条并收尾。
func _on_annotate_toggled(on: bool) -> void:
	if not on:
		_editor.call("close")
		_end_edit()
		return
	var img: Image = DeviceConn.current_image
	if img == null or img.is_empty():
		_chat_panel.chat("提示", "先开启图传、等画面出现再框选目标")
		_annotate_btn.set_pressed_no_signal(false)
		return
	_begin_edit(img)

## 进入标注：图传区临时可见（图传关着时也能在冻结帧上标注），冻结帧交给画布。
func _begin_edit(img: Image) -> void:
	_editing = true
	_annotate_btn.set_pressed_no_signal(true)
	_apply_layout()   # 图传开关的禁用由「replaying or _editing」统一在这里管
	_editor.call("open", img)

## 退出标注：解冻图传、复位「框选目标」按钮、放开图传开关。
func _end_edit() -> void:
	_editing = false
	_annotate_btn.set_pressed_no_signal(false)
	_apply_layout()

## 编辑器「采用」：编辑图进聊天区附件列表，然后收尾。
func _on_editor_image_sent(img: Image, annotation: Dictionary) -> void:
	_end_edit()
	_chat_panel.call("_on_image_sent", img, annotation)

## 选图（/append 与附图按钮）：ChatPanel 已压缩好，这里打开标注编辑器。
func _on_chat_image_picked(img: Image) -> void:
	_begin_edit(img)

func _on_editor_cancelled() -> void:
	_end_edit()  # 取消 = 放弃这张图，不影响输入框与已附图

## 编辑器「自动夹取」：把标注框换算成中心后下发 auto_grasp，板端自己完成夹取（不调云端 AI）。
## 框选产物是**左上角+宽高**，板端要的是**中心**（见 grasp.h / track.h），故此处换算。
func _on_editor_grasp_requested(ann: Dictionary) -> void:
	if ann.is_empty():
		_chat_panel.chat("提示", "请先拖出一个方框框住目标，再点「自动夹取」")
		return
	var w: float = float(ann.get("w", 0.0))
	var h: float = float(ann.get("h", 0.0))
	var cx: float = float(ann.get("x", 0.0)) + w * 0.5
	var cy: float = float(ann.get("y", 0.0)) + h * 0.5
	_end_edit()
	if not DeviceConn.send_command(CP.auto_grasp(cx, cy, w, h)):
		_chat_panel.chat("提示", "自动夹取未发送（当前离线）")

## 聊天区「图传」旁路请求（/stream 由 ChatPanel 解析后交给 Main 统一起停 UDP 接收）。
func _on_chat_stream_requested(on: bool) -> void:
	_stream_toggle.set_pressed_no_signal(on)
	Store.set_stream_on(on)
	_apply_stream(on)

## 图传标定网格叠加（/grid 本地开关，配合单应标定测量）。
func _on_chat_grid_requested(on: bool) -> void:
	_video.call("show_grid", on)

# ============================== 会话回放 ==============================

func _on_session_selected(id: int) -> void:
	_session_panel.call("close")
	if id < 0:
		Recorder.stop_play()   # 当前会话 = 退出回放（回 live）
		return
	if not Recorder.play(id):
		_session_panel.call("set_selected", -1)
		_chat_panel.chat("提示", "该会话没有可回放的内容")
		return
	_session_panel.call("set_selected", id)
	_chat_panel.call("begin_replay")
	_playback_bar.call("set_paused", false)
	_apply_layout()

func _on_session_delete_requested(id: int) -> void:
	_confirm_action = func(): Recorder.delete(id)
	_wifi_popup.call("open_confirm", "删除会话", "是否确认删除该会话？", "删除")

func _on_popup_cancelled() -> void:
	# 取消（按钮或点遮罩）也要清掉挂起的确认动作，免得下次弹确认框时误触发上一次的。
	_confirm_action = Callable()

## /clear 开了新会话：板端上下文已清，本地记忆图也一并清掉（板子不会主动推一份空快照）。
func _on_session_cleared() -> void:
	_video.call("clear_map")

func _on_replay_event(ev: Dictionary) -> void:
	_chat_panel.call("replay_apply", ev)

func _on_replay_frame(img: Image) -> void:
	_video.call("set_frame", img)

func _on_replay_seeked() -> void:
	# 拖动/回退：清场后由 Recorder 重新抛该点之前的事件。
	_chat_panel.call("begin_replay")

func _on_replay_progress(t_ms: int, dur_ms: int) -> void:
	_playback_bar.call("set_progress", 0.0 if dur_ms <= 0 else float(t_ms) / float(dur_ms))

func _on_replay_finished() -> void:
	_chat_panel.call("end_replay")
	_playback_bar.call("set_paused", true)
	_apply_layout()

func _on_replay_step(dir: int) -> void:
	Recorder.step(dir)
	_playback_bar.call("set_paused", true)

func _on_replay_toggle_pause() -> void:
	Recorder.toggle_pause()
	_playback_bar.call("set_paused", Recorder.is_paused())

func _on_replay_seek(ratio: float) -> void:
	Recorder.seek_ratio(ratio)

# ============================== 状态 ==============================

func _process(_delta: float) -> void:
	_sync_bottom_padding()

func _update_status() -> void:
	var ble: String = DeviceConn.get_ble_state()
	var ws_state: String = DeviceConn.get_ws_state()
	var online: bool = ws_state == "connected"
	var ble_on: bool = ble != "off" and ble != "unavailable"
	_ble_dot.modulate = Color.GREEN if ble_on else Color(1, 1, 1, 0.3)
	_ws_dot.modulate = Color.GREEN if online else Color(1, 1, 1, 0.3)
	_ble_stat.text = "BLE:%s" % ble
	_ws_stat.text = "WS:%s" % ("已连接" if online else "未连接")
	# 顶栏大字三态：WS 在线优先判定（WS_ONLY 下 BLE 已让出射频）。
	var ble_ok: bool = DeviceConn.is_device_connected()
	if online:
		_conn_stat.text = ("已连接: %s" % _device_name) if _device_name != "" else "已连接(WS)"
	elif ble_ok and _device_name != "":
		_conn_stat.text = "已连接: %s" % _device_name
	elif ble == "connecting":
		_conn_stat.text = ("连接中: %s" % _pending_name) if _pending_name != "" else "连接中…"
	else:
		_conn_stat.text = "设备未连接"
