class_name EditorCanvas
extends Control
## 图片编辑画布：底图 + 标注对象（方框/圆圈），坐标与视口一致以便直接截图导出。

const ACCENT := Color(1.0, 0.45, 0.2, 1.0)
const LINE_W := 4.0

var texture: Texture2D = null
var tool := "none"   # none / rect / circle
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

## 取首个标注（方框或圆圈）→ 归一化区域 {x,y,w,h}（相对源图 0..1），供 ai_goal.annotation。
## 换算用与 _draw 相同的居中缩放矩形 ir（视口坐标 → 贴图坐标 → 归一化）。无标注返回空。
func first_region_norm() -> Dictionary:
	for a: Variant in annotations:
		if not (a is Dictionary):
			continue
		var kind: String = str((a as Dictionary).get("kind", ""))
		if kind == "rect":
			var p: Vector2 = (a as Dictionary).get("a", Vector2.ZERO)
			var q: Vector2 = (a as Dictionary).get("b", Vector2.ZERO)
			return _norm_rect(p, q)
		elif kind == "circle":
			var c: Vector2 = (a as Dictionary).get("a", Vector2.ZERO)
			var edge: Vector2 = (a as Dictionary).get("b", c)
			return _norm_circle(c, c.distance_to(edge))
	return {}

## 导出编辑图：取视口渲染，按与 _draw 相同的居中缩放矩形 ir 裁剪出「底图+标注」，
## 宽高比=底图、无黑边。失败（无底图 / 取帧失败）返回 null。
func export_capture() -> Image:
	if texture == null:
		return null
	var vp := get_viewport()
	if vp == null or vp.get_texture() == null:
		return null
	var img: Image = vp.get_texture().get_image()
	if img == null or img.is_empty():
		return null
	var ts := Vector2(texture.get_width(), texture.get_height())
	if ts.x <= 0.0 or ts.y <= 0.0:
		return null
	var s: float = min(size.x / ts.x, size.y / ts.y)
	var ir := Rect2((size - ts * s) / 2.0, ts * s)
	# 画布（=视口渲染）坐标 → 视图纹理像素：等比缩放取整，夹取在图像内。
	var sc := Vector2(float(img.get_width()) / maxf(size.x, 1.0), float(img.get_height()) / maxf(size.y, 1.0))
	var x0: int = maxi(int(floor(ir.position.x * sc.x)), 0)
	var y0: int = maxi(int(floor(ir.position.y * sc.y)), 0)
	var x1: int = mini(int(ceil(ir.end.x * sc.x)), img.get_width())
	var y1: int = mini(int(ceil(ir.end.y * sc.y)), img.get_height())
	if x1 <= x0 or y1 <= y0:
		return null
	return img.get_region(Rect2(x0, y0, x1 - x0, y1 - y0))

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

func _norm_circle(c: Vector2, radius: float) -> Dictionary:
	if texture == null or radius <= 0.0:
		return {}
	var ts := Vector2(texture.get_width(), texture.get_height())
	if ts.x <= 0.0 or ts.y <= 0.0:
		return {}
	var s: float = min(size.x / ts.x, size.y / ts.y)
	var ir := Rect2((size - ts * s) / 2.0, ts * s)
	var r2 := Vector2(radius, radius)
	var tl := _clamp01((c - r2 - ir.position) / ir.size)
	var br := _clamp01((c + r2 - ir.position) / ir.size)
	return {"x": tl.x, "y": tl.y, "w": absf(br.x - tl.x), "h": absf(br.y - tl.y)}

func _draw() -> void:
	if texture == null:
		return
	var ts := Vector2(texture.get_width(), texture.get_height())
	var s: float = min(size.x / ts.x, size.y / ts.y)
	var ir := Rect2((size - ts * s) / 2.0, ts * s)
	draw_texture_rect(texture, ir, false)
	for a in annotations:
		_draw_annotation(a)
	if _dragging and (tool == "rect" or tool == "circle"):
		_draw_shape(tool, _drag_origin, _drag_current)

func _draw_annotation(a: Dictionary) -> void:
	var kind: String = str(a.get("kind", ""))
	if kind == "rect":
		draw_rect(Rect2(a.a, a.b - a.a), a.color, false, LINE_W)
	elif kind == "circle":
		_draw_shape("circle", a.a, a.b)

func _draw_shape(kind: String, a: Vector2, b: Vector2) -> void:
	if kind == "rect":
		draw_rect(Rect2(a, b - a), ACCENT, false, LINE_W)
		return
	# circle：以起点为圆心、终点定半径
	var radius: float = a.distance_to(b)
	if radius > 0.5:
		draw_arc(a, radius, 0.0, TAU, 64, ACCENT, LINE_W)

func _gui_input(event: InputEvent) -> void:
	if texture == null:
		return
	if not (tool == "rect" or tool == "circle"):
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