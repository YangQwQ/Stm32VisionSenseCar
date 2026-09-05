extends Control
## 全屏模态图片编辑面板：冻结帧 + 标注工具，输出编辑后的 Image。

signal sent_to_ai(image: Image)
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

func _rasterize() -> Image:
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	await get_tree().process_frame
	var img: Image = _viewport.get_texture().get_image()
	if img == null:
		return null
	return img

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
	close_modal()

func _on_send_ai_pressed() -> void:
	var img := await _rasterize()
	if img != null:
		sent_to_ai.emit(img)
	close_modal()