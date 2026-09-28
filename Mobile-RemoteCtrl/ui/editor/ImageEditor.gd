extends HBoxContainer
## 图片标注工具条（原全屏模态面板改造）：点「框选目标」冻结图传取当前帧，
## 在此选工具 / 撤销 / 清除，标注直接画在叠加于图传画面的 EditorCanvas 上（无 SubViewport）。
## 「采用」导出「底图+标注」由 Main 转交聊天区作附图；「取消」放弃这张图。

signal cancelled
signal image_sent(img: Image, annotation: Dictionary)

const CanvasScript := preload("res://ui/editor/EditorCanvas.gd")

@onready var _rect_btn: Button = $RectBtn
@onready var _circle_btn: Button = $CircleBtn

var _canvas: Control = null

func _ready() -> void:
	# 画布叠在 Video 上（同 rect、最后加入 → 盖在画面与网格之上）；仅编辑时可见并吃触摸。
	_canvas = CanvasScript.new()
	_canvas.visible = false
	var video: Control = get_parent().get_node("Video")
	video.add_child(_canvas)
	_canvas.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_rect_btn.toggled.connect(_on_rect_toggled)
	_circle_btn.toggled.connect(_on_circle_toggled)
	$UndoBtn.pressed.connect(_on_undo_pressed)
	$ClearBtn.pressed.connect(_on_clear_pressed)
	$CancelBtn.pressed.connect(_on_cancel_pressed)
	$DoneBtn.pressed.connect(_on_done_pressed)

## 进入标注：以冻结帧为底图，工具复位为「不选」。
func open(img: Image) -> void:
	_canvas.call("set_base_image", ImageTexture.create_from_image(img))
	_set_tool("none")
	_canvas.visible = true
	visible = true

func close() -> void:
	_canvas.visible = false
	visible = false

func _set_tool(t: String) -> void:
	_canvas.call("set_tool", t)
	_rect_btn.set_pressed_no_signal(t == "rect")
	_circle_btn.set_pressed_no_signal(t == "circle")

func _on_rect_toggled(on: bool) -> void:
	_set_tool("rect" if on else "none")

func _on_circle_toggled(on: bool) -> void:
	_set_tool("circle" if on else "none")

func _on_undo_pressed() -> void:
	_canvas.call("undo")

func _on_clear_pressed() -> void:
	_canvas.call("clear")

func _on_cancel_pressed() -> void:
	close()
	cancelled.emit()

func _on_done_pressed() -> void:
	var img: Image = _canvas.call("export_capture")
	var ann: Dictionary = _canvas.call("first_region_norm")
	close()
	image_sent.emit(img, ann)
