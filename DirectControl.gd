extends Panel

const CP := preload("res://net/proto/CommandProto.gd")

# 爪控经 AppState.send_command 统一发送：WS 优先、BLE 兜底（不再只认 WS）。

const _ACTIONS := {
	"ClawUpBtn": "lift_up",
	"ClawDownBtn": "lift_down",
	"ClawClipBtn": "clip",
	"ClawReleaseBtn": "release",
	"ClawForwardBtn": "reach_forward",
	"ClawBackwardBtn": "reach_backward",
}

func _ready() -> void:
	for path in _ACTIONS:
		var btn: Button = get_node("Btns/" + path)
		var act: String = _ACTIONS[path]
		btn.button_down.connect(_on_claw_down.bind(act))
		btn.button_up.connect(_on_claw_up)

func _on_claw_down(act: String) -> void:
	AppState.send_command(CP.arm(act))

func _on_claw_up() -> void:
	AppState.send_command(CP.stop("arm"))
