class_name EditorCanvas
extends Control
## 图片编辑画布：底图 + 标注对象（框/箭头/文字），坐标与视口一致以便直接截图。

const ACCENT := Color(1.0, 0.45, 0.2, 1.0)
const FONT_SIZE := 34.0
const LINE_W := 4.0

var texture: Texture2D = null
var tool := "none"   # none / rect / arrow / text
var draft_text := ""
var annotations: Array = []

var _drag_origin := Vector2.ZERO
var _drag_current := Vector2.ZERO
var _dragging := false

func _ready() -> void:
	mouse_filter = MOUSE_FILTER_STOP

func set_base_image(tex: Texture2D) -> void:
	texture = tex
	annotations = []
	_dragging = false
	queue_redraw()

func set_tool(t: String) -> void:
	tool = t
	_dragging = false
	queue_redraw()

func undo() -> void:
	if not annotations.is_empty():
		annotations.pop_back()
	queue_redraw()

func clear() -> void:
	annotations = []
	queue_redraw()

## 取第一个 rect 标注 → 归一化区域 {x,y,w,h}（相对源图 0..1），供 ai_goal.annotation。
## 换算用与 _draw 相同的居中缩放矩形 ir（视口坐标 → 贴图坐标 → 归一化）。无 rect 返回空。
func first_rect_norm() -> Dictionary:
	for a: Variant in annotations:
		if not (a is Dictionary):
			continue
		if (a as Dictionary).get("kind") == "rect":
			var p: Vector2 = (a as Dictionary).get("a", Vector2.ZERO)
			var q: Vector2 = (a as Dictionary).get("b", Vector2.ZERO)
			return _norm_rect(p, q)
	return {}

func _norm_rect(p: Vector2, q: Vector2) -> Dictionary:
	if texture == null:
		return {}
	var ts := Vector2(texture.get_width(), texture.get_height())
	if ts.x <= 0.0 or ts.y <= 0.0:
		return {}
	var s: float = min(size.x / ts.x, size.y / ts.y)
	var ir := Rect2((size - ts * s) / 2.0, ts * s)
	var p0 := _clamp01((p - ir.position) / ir.size)
	var p1 := _clamp01((q - ir.position) / ir.size)
	var x0: float = min(p0.x, p1.x)
	var y0: float = min(p0.y, p1.y)
	return {"x": x0, "y": y0, "w": absf(p1.x - p0.x), "h": absf(p1.y - p0.y)}

func _clamp01(v: Vector2) -> Vector2:
	return Vector2(clampf(v.x, 0.0, 1.0), clampf(v.y, 0.0, 1.0))

func _draw() -> void:
	if texture == null:
		return
	var ts := Vector2(texture.get_width(), texture.get_height())
	var s: float = min(size.x / ts.x, size.y / ts.y)
	var ir := Rect2((size - ts * s) / 2.0, ts * s)
	draw_texture_rect(texture, ir, false)
	for a in annotations:
		_draw_annotation(a)
	if _dragging and (tool == "rect" or tool == "arrow"):
		_draw_shape(tool, _drag_origin, _drag_current)

func _draw_annotation(a: Dictionary) -> void:
	match a.get("kind"):
		"rect":
			draw_rect(Rect2(a.a, a.b - a.a), a.color, false, LINE_W)
		"arrow":
			_draw_shape("arrow", a.a, a.b)
		"text":
			var font := ThemeDB.fallback_font
			draw_string(font, a.pos + Vector2(0, 34), a.text, HORIZONTAL_ALIGNMENT_LEFT, -1, FONT_SIZE, a.color)

func _draw_shape(kind: String, a: Vector2, b: Vector2) -> void:
	if kind == "rect":
		draw_rect(Rect2(a, b - a), ACCENT, false, LINE_W)
		return
	draw_line(a, b, ACCENT, LINE_W)
	var dir := (b - a).normalized()
	if dir.is_zero_approx():
		return
	var head := 26.0
	var n1 := dir.rotated(0.45) * head
	var n2 := dir.rotated(-0.45) * head
	draw_line(b, b - n1, ACCENT, LINE_W)
	draw_line(b, b - n2, ACCENT, LINE_W)

func _gui_input(event: InputEvent) -> void:
	if texture == null:
		return
	if tool == "text":
		if _is_press(event):
			annotations.append({"kind": "text", "pos": _pos(event), "text": draft_text, "color": ACCENT})
			queue_redraw()
		return
	if not (tool == "rect" or tool == "arrow"):
		return
	if _is_press(event):
		_dragging = true
		_drag_origin = _pos(event)
		_drag_current = _drag_origin
		queue_redraw()
	elif _is_release(event):
		_commit()
	elif event is InputEventScreenDrag:
		_drag_current = event.position
		queue_redraw()
	elif event is InputEventMouseMotion:
		_drag_current = event.position
		queue_redraw()

func _commit() -> void:
	if _dragging:
		if _drag_origin.distance_to(_drag_current) > 8.0:
			annotations.append({"kind": tool, "a": _drag_origin, "b": _drag_current, "color": ACCENT})
		_dragging = false
	queue_redraw()

func _is_press(e: InputEvent) -> bool:
	return (e is InputEventMouseButton and e.pressed) or (e is InputEventScreenTouch and e.pressed)

func _is_release(e: InputEvent) -> bool:
	return (e is InputEventMouseButton and not e.pressed) or (e is InputEventScreenTouch and not e.pressed)

func _pos(e: InputEvent) -> Vector2:
	if e is InputEventMouseButton or e is InputEventMouseMotion:
		return (e as InputEventMouse).position
	if e is InputEventScreenTouch or e is InputEventScreenDrag:
		return (e as InputEventScreenTouch).position
	return Vector2.ZERO