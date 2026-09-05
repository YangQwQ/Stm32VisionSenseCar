extends Control
## App 壳：组装各子视图，维护连接对象与交互逻辑。

const CP := preload("res://net/proto/CommandProto.gd")

@onready var _video: Control = $Body/Video
@onready var _joystick: VirtualJoystick = $Body/CtrlArea/Joystick
@onready var _wifi_popup: PanelContainer = $WifiPopup
@onready var _editor: Control = $ImageEditor

@onready var _device_list: OptionButton = $TopBar/HBox/DeviceList
@onready var _provision_btn: Button = $TopBar/HBox/ProvisionBtn
@onready var _ble_dot: Label = $TopBar/HBox/DeviceList/BleStat/BleDot
@onready var _ble_stat: Label = $TopBar/HBox/DeviceList/BleStat
@onready var _ws_dot: Label = $TopBar/HBox/ConnectionStat/WsStat/WsDot
@onready var _ws_stat: Label = $TopBar/HBox/ConnectionStat/WsStat
@onready var _conn_stat: Label = $TopBar/HBox/ConnectionStat

@onready var _ai_log: RichTextLabel = $Body/AILog
@onready var _approval: PanelContainer = $Body/ApprovalCard
@onready var _approve_reason: Label = $Body/ApprovalCard/VBox/Reason
@onready var _approve_detail: Label = $Body/ApprovalCard/VBox/Detail
@onready var _relay_btn: Button = $Body/ModeBar/RelayBtn
@onready var _direct_btn: Button = $Body/ModeBar/DirectBtn
@onready var _chat_log: RichTextLabel = $Body/ChatPanel/ChatLog
@onready var _message_input: LineEdit = $Body/ChatPanel/InputRow/MessageInput
@onready var _send_btn: Button = $Body/ChatPanel/InputRow/SendBtn
@onready var _stream_toggle: CheckButton = $Body/VidControls/StreamToggle
@onready var _snapshot_btn: Button = $Body/VidControls/SnapshotBtn
@onready var _edit_btn: Button = $Body/VidControls/EditBtn

var _joy_held := false
var _last_joy := Vector2.ZERO

func _ready() -> void:
	# 通信层节点由 Main.tscn 挂在 $Net 下，注册到全局状态
	AppState.ble = $Net/BLE
	AppState.ws = $Net/WS
	AppState.ai = $Net/AI

	$Net/BLE.ble_state_changed.connect(_on_ble_state)
	$Net/BLE.scan_finished.connect(_on_scan_finished)
	$Net/WS.connected.connect(_on_ws_connected)
	$Net/WS.disconnected.connect(_on_ws_disconnected)
	$Net/WS.frame_received.connect(_on_frame)
	$Net/AI.ai_response.connect(_on_ai_response)

	# 启动即尝试连接小车（离线时显示"未连接"即可，不阻塞 UI）
	$Net/WS.connect_car()

	_update_status()
	set_process_input(true)

# ----- BLE -----
func _on_ble_state(_s: String) -> void:
	_update_status()

func _on_scan_finished(devices: Array) -> void:
	_device_list.clear()
	for d in devices:
		_device_list.add_item(d)

func _on_refresh_pressed() -> void:
	$Net/BLE.scan()

func _on_provision_pressed() -> void:
	_wifi_popup.call("popup")

func _on_wifi_confirmed(ssid: String, password: String) -> void:
	$Net/BLE.provision(ssid, password)
	_push_ai_log("已通过蓝牙下发配网参数: %s" % ssid)

# ----- WS / 视频 -----
func _on_ws_connected() -> void:
	_update_status()

func _on_ws_disconnected(_reason: String) -> void:
	_video.call("show_no_signal", true)
	_update_status()

func _on_frame(img: Image) -> void:
	_video.call("set_frame", img)
	AppState.current_image = img  # 保留最新一帧供(未编辑)发送

func _on_stream_toggled(on: bool) -> void:
	$Net/WS.send_command(CP.stream(on))

# ----- 截图 / 编辑 -----
func _on_snapshot_pressed() -> void:
	if _video.current_texture != null:
		_editor.call("open", _video.current_texture)

func _on_edit_pressed() -> void:
	if _video.current_texture != null:
		_editor.call("open", _video.current_texture)

func _on_ai_image_sent(image: Image) -> void:
	AppState.current_image = image
	_trigger_ai(image)

func _trigger_ai(image: Image) -> void:
	if AppState.ai_mode == AppState.AiMode.RELAY:
		_push_ai_log("发送给云端 AI 处理…")
		$Net/AI.ask(image, _message_input.text)
	else:
		_push_ai_log("小车直连模式：AI 由小车端处理，请在设备日志查看")

# ----- AI -----
func _on_ai_response(tool: Dictionary, text: String) -> void:
	AppState.pending_tool = tool
	_push_ai_log(text)
	_approval.visible = true
	_approve_reason.text = tool.get("reason", "")
	_approve_detail.text = CP.encode(tool)
	AnimationManager.fade_scale_in(_approval)

func _on_approve_execute() -> void:
	var tool: Dictionary = AppState.pending_tool
	if not tool.is_empty():
		$Net/WS.send_command(tool)
		_push_ai_log("已执行：%s" % tool.get("type", "?"))
	_approval.visible = false
	AppState.pending_tool = {}

func _on_approve_cancel() -> void:
	_approval.visible = false
	AppState.pending_tool = {}

func _set_ai_mode(mode) -> void:
	AppState.ai_mode = mode
	_relay_btn.button_pressed = (mode == AppState.AiMode.RELAY)
	_direct_btn.button_pressed = (mode == AppState.AiMode.DIRECT)

func _on_relay_pressed() -> void:
	_set_ai_mode(AppState.AiMode.RELAY)

func _on_direct_pressed() -> void:
	_set_ai_mode(AppState.AiMode.DIRECT)

# ----- 聊天 -----
func _on_send_pressed() -> void:
	var text: String = _message_input.text.strip_edges()
	if text.is_empty():
		return
	_chat_log.append_text("[b]我:[/b] %s\n" % text)
	_message_input.text = ""
	if AppState.ai_mode == AppState.AiMode.RELAY and AppState.current_image != null:
		_trigger_ai(AppState.current_image)

func _push_ai_log(msg: String) -> void:
	_ai_log.append_text(msg + "\n")

# ----- 手动控制 -----
func _process(_delta: float) -> void:
	if _joy_held:
		_update_joystick()

func _update_joystick() -> void:
	if not $Net/WS.is_connected_car():
		return
	var v: Vector2 = _joystick.get_value()
	# 上=前进：推力取 -y；左右取 x
	var throttle: float = clampf(-v.y, -1.0, 1.0)
	var steering: float = clampf(v.x, -1.0, 1.0)
	if v.distance_to(_last_joy) < 0.001:
		return
	_last_joy = v
	$Net/WS.send_command(CP.move(throttle, steering))

func _on_joystick_pressed(_v: Variant = null) -> void:
	_joy_held = true

func _on_joystick_release(_v: Variant = null) -> void:
	_joy_held = false
	_last_joy = Vector2.ZERO
	if $Net/WS.is_connected_car():
		$Net/WS.send_command(CP.stop("wheels"))

func _on_stop_pressed() -> void:
	_joystick.call("reset")
	$Net/WS.send_command(CP.stop())

# ----- 状态 -----
func _update_status() -> void:
	var ble: String = $Net/BLE.get_ble_state()
	var ws: String = $Net/WS.get_state()
	var online: bool = ws == "connected"
	_ble_dot.modulate = Color.GREEN if ble != "off" else Color(1, 1, 1, 0.3)
	_ws_dot.modulate = Color.GREEN if online else Color(1, 1, 1, 0.3)
	_ble_stat.text = "BLE:%s" % ble
	_ws_stat.text = "WS:%s" % ("已连接" if online else "未连接")
	var ctrl: String = "已连接" if online else "未连接"
	_conn_stat.text = "已连: %s" % ctrl
