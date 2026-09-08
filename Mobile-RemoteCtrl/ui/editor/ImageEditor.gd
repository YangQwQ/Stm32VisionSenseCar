extends Control
## 全屏模态标注面板：冻结帧 + 「方框 / 圆圈」标注。
## 取消 = 放弃这张图；采用 = 导出「底图+标注」，由 Main 以 [Image N] 附到聊天输入框。

signal cancelled
signal image_sent(img: Image, annotation: Dictionary)

@onready var _canvas: Control = $Panel/VBox/ViewportWrap/SubViewportContainer/Viewport/Canvas
@onready var _panel: PanelContainer = $Panel
@onready var _rect_btn: Button = $Panel/VBox/Toolbar/RectBtn
@onready var _circle_btn: Button = $Panel/VBox/Toolbar/CircleBtn

func open(texture: Texture2D) -> void:
	_canvas.call("set_base_image", texture)
	_set_tool("none")
	visible = true
	AnimationManager.fade_scale_in(_panel)

func close_modal() -> void:
	AnimationManager.fade_scale_out(_panel)
	await get_tree().create_timer(0.2).timeout
	visible = false

func _set_tool(t: String) -> void:
	_canvas.call("set_tool", t)
	_rect_btn.button_pressed = (t == "rect")
	_circle_btn.button_pressed = (t == "circle")

func _on_rect_pressed() -> void:
	_set_tool("rect" if _rect_btn.button_pressed else "none")

func _on_circle_pressed() -> void:
	_set_tool("circle" if _circle_btn.button_pressed else "none")

func _on_undo_pressed() -> void:
	_canvas.call("undo")

func _on_clear_pressed() -> void:
	_canvas.call("clear")

func _on_cancel_pressed() -> void:
	cancelled.emit()
	close_modal()

func _on_done_pressed() -> void:
	var img: Image
	if _canvas != null and _canvas.has_method("export_capture"):
		img = _canvas.call("export_capture")
	var ann: Dictionary = {}
	if _canvas != null and _canvas.has_method("first_region_norm"):
		var got: Variant = _canvas.call("first_region_norm")
		if got is Dictionary:
			ann = got
	close_modal()
	image_sent.emit(img, ann)
