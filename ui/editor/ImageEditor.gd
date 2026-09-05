extends Control
## 全屏模态标注面板：冻结帧 + 框选工具。完成时输出首个框的归一化区域
## {x,y,w,h}（供 DIRECT ai_goal.annotation），不在本面板发图/调云 AI。

signal annotated(annotation: Dictionary)
signal cancelled

@onready var _canvas: Control = $Panel/VBox/ViewportWrap/SubViewportContainer/Viewport/Canvas
@onready var _viewport: SubViewport = $Panel/VBox/ViewportWrap/SubViewportContainer/Viewport
@onready var _panel: PanelContainer = $Panel
@onready var _text_input: LineEdit = $Panel/VBox/Toolbar/TextInput
@onready var _rect_btn: Button = $Panel/VBox/Toolbar/RectBtn
@onready var _arrow_btn: Button = $Panel/VBox/Toolbar/ArrowBtn
@onready var _text_btn: Button = $Panel/VBox/Toolbar/TextBtn

func open(texture: Texture2D) -> void:
	_canvas.call("set_base_image", texture)
	_set_tool("none")
	_text_input.text = ""
	visible = true
	AnimationManager.fade_scale_in(_panel)

func close_modal() -> void:
	AnimationManager.fade_scale_out(_panel)
	await get_tree().create_timer(0.2).timeout
	visible = false

func _set_tool(t: String) -> void:
	_canvas.call("set_tool", t)
	_rect_btn.button_pressed = (t == "rect")
	_arrow_btn.button_pressed = (t == "arrow")
	_text_btn.button_pressed = (t == "text")

func _on_rect_pressed() -> void:
	_set_tool("rect" if _rect_btn.button_pressed else "none")

func _on_arrow_pressed() -> void:
	_set_tool("arrow" if _arrow_btn.button_pressed else "none")

func _on_text_pressed() -> void:
	_set_tool("text" if _text_btn.button_pressed else "none")

func _on_text_submitted(_new_text: String) -> void:
	_canvas.draft_text = _text_input.text

func _on_undo_pressed() -> void:
	_canvas.call("undo")

func _on_clear_pressed() -> void:
	_canvas.call("clear")

func _on_cancel_pressed() -> void:
	cancelled.emit()
	close_modal()

func _on_done_pressed() -> void:
	var ann: Dictionary = {}
	if _canvas != null and _canvas.has_method("first_rect_norm"):
		var got: Variant = _canvas.call("first_rect_norm")
		if got is Dictionary:
			ann = got
	close_modal()
	annotated.emit(ann)