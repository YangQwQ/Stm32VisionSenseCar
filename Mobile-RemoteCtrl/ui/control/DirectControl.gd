extends Panel

const CP := preload("res://net/proto/CommandProto.gd")

# 爪控经 AppState.send_command 统一发送：WS 优先、BLE 兜底（不再只认 WS）。

# 持续型（按住动、松开停）：升降 / 移爪。
const _HOLD_ACTIONS := {
	"ClawUpBtn": "lift_up",
	"ClawDownBtn": "lift_down",
	"ClawForwardBtn": "reach_forward",
	"ClawBackwardBtn": "reach_backward",
}

func _ready() -> void:
	# 持续型：按住下发动，松开发 stop。
	for path in _HOLD_ACTIONS:
		var btn: Button = get_node("Btns/" + path)
		var act: String = _HOLD_ACTIONS[path]
		btn.button_down.connect(_on_hold_down.bind(act))
		btn.button_up.connect(_on_hold_up)

	# 夹取/松夹合一：toggle → true 夹、false 松（离散，松开不补 stop）。
	var grip: Button = get_node("Btns/ClawClipReleaseBtn")
	grip.toggled.connect(_on_grip_toggled)

	# 回正：机械臂 + 转向一次性归位。
	var reset_btn: Button = get_node("Btns/ResetPosBtn")
	reset_btn.pressed.connect(_on_reset_pressed)

	# 三灯开关：kind 与按钮名映射。
	var lights := {"ForeLight": "front", "VibeLight": "vibe", "BackLight": "back"}
	for path in lights:
		var b: Button = get_node("LightCtrl/" + path)
		b.toggled.connect(_on_light_toggled.bind(lights[path]))

# 当前按住的动作；松开即停。
var _hold_act := ""

func _on_hold_down(act: String) -> void:
	AppState.send_command(CP.arm(act))
	_hold_act = act

func _on_hold_up() -> void:
	if not _hold_act.is_empty():
		AppState.send_command(CP.stop("arm"))
	_hold_act = ""

func _on_grip_toggled(on: bool) -> void:
	AppState.send_command(CP.arm("clip" if on else "release"))

func _on_reset_pressed() -> void:
	AppState.send_command(CP.reset())

func _on_light_toggled(on: bool, kind: String) -> void:
	AppState.send_command(CP.light(kind, on))

## 重连/查询后用板端状态同步按钮（不触发 toggled 回调，避免反向多下指令）。
func sync_state(lights: Dictionary, grip_close: bool) -> void:
	var map := {"front": "ForeLight", "vibe": "VibeLight", "back": "BackLight"}
	for kind: String in map:
		var b: Button = get_node_or_null("LightCtrl/" + map[kind])
		if b != null:
			b.set_pressed_no_signal(bool(lights.get(kind, false)))
	var grip: Button = get_node_or_null("Btns/ClawClipReleaseBtn")
	if grip != null:
		grip.set_pressed_no_signal(grip_close)
