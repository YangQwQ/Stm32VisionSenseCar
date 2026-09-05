extends Panel

const CP := preload("res://net/proto/CommandProto.gd")

@onready var _ws = $"../../Net/WS"

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
	if _ws.is_connected_car():
		_ws.send_command(CP.arm(act))

func _on_claw_up() -> void:
	if _ws.is_connected_car():
		_ws.send_command(CP.stop("arm"))
