extends ColorRect
## 弹窗背景遮罩：盖在下层之上，mouse_filter=STOP 吃掉下层触控（翻页手势 / 摇杆 / 按钮）。
## 自身只做一件事——点空白处关闭弹窗。遮罩不覆盖弹窗本体（弹窗在其上层、自己消费触点），
## 故这里收到的点击必在空白处，不必再判断是否落在弹窗区域内。

signal tapped   # 点在遮罩空白处（按下→抬起、几乎没移动）：请求关闭弹窗

const TAP_SLOP := 20.0   # 按下到抬起的位移超过它就不算"点"，按拖动忽略

var _press_pos := Vector2.ZERO
var _pressed := false

func _gui_input(event: InputEvent) -> void:
	var pos := Vector2.INF
	var down := false
	if event is InputEventScreenTouch:
		var t := event as InputEventScreenTouch
		pos = t.position
		down = t.pressed
	elif event is InputEventMouseButton \
			and (event as InputEventMouseButton).button_index == MOUSE_BUTTON_LEFT:
		var mb := event as InputEventMouseButton
		pos = mb.position
		down = mb.pressed
	else:
		return
	if down:
		_pressed = true
		_press_pos = pos
		return
	if _pressed and pos.distance_to(_press_pos) <= TAP_SLOP:
		tapped.emit()
	_pressed = false