class_name EditorCanvas
extends Control
## 图片编辑画布：底图 + 标注对象（方框/椭圆，均由拖出的外接框定义）。直接叠在图传画面上绘制，
## 导出用手工把标注合成到底图 Image（不依赖视口，因此无需 SubViewport）。

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

## 取首个标注（方框 / 椭圆）→ 归一化区域 {x,y,w,h}（相对源图 0..1），供 ai_goal.annotation。
## 两者都由 a→b 拖出的矩形外接框定义，故共用同一套换算（画布坐标 → 贴图坐标 → 归一化）。无标注返回空。
func first_region_norm() -> Dictionary:
	for a: Variant in annotations:
		if not (a is Dictionary):
			continue
		var kind: String = str((a as Dictionary).get("kind", ""))
		if kind == "rect" or kind == "circle":
			var p: Vector2 = (a as Dictionary).get("a", Vector2.ZERO)
			var q: Vector2 = (a as Dictionary).get("b", Vector2.ZERO)
			return _norm_rect(p, q)
	return {}

## 导出编辑图：把「底图 + 标注」直接合成到 Image，不依赖任何视口（无 SubViewport）。
## 标注坐标（画布 px）→ 源图像素：先减去居中缩放矩形 ir 的偏移，再按显示比例放大。
## 宽高比=底图、无黑边。失败（无底图 / 取不到图像数据）返回 null。
func export_capture() -> Image:
	if texture == null:
		return null
	var base: Image = texture.get_image()
	if base == null or base.is_empty():
		return null
	var out: Image = Image.new()
	out.copy_from(base)
	if out.is_compressed():
		out.decompress()
	var ts := Vector2(out.get_width(), out.get_height())
	if ts.x <= 0.0 or ts.y <= 0.0:
		return null
	var ir: Rect2 = _image_rect(ts)
	var disp: float = ir.size.x / ts.x   # 画布 px 每源图 px
	if disp <= 0.0:
		return null
	var thick: int = maxi(int(round(LINE_W / disp)), 1)
	for a: Variant in annotations:
		if not (a is Dictionary):
			continue
		var d: Dictionary = a as Dictionary
		var kind: String = str(d.get("kind", ""))
		var col: Color = d.get("color", ACCENT)
		if kind == "rect":
			_fill_rect_outline(out, _to_image_rect(d.get("a", Vector2.ZERO), d.get("b", Vector2.ZERO), ir, disp), col, thick)
		elif kind == "circle":
			var box: Rect2 = _to_image_rect(d.get("a", Vector2.ZERO), d.get("b", Vector2.ZERO), ir, disp)
			_fill_ellipse_outline(out, box.get_center(), box.size.x * 0.5, box.size.y * 0.5, col, thick)
	return out

## 画布内「居中缩放矩形」：底图按 KEEP_ASPECT_CENTERED 摆进画布后的实际显示区（与 _draw 一致）。
func _image_rect(ts: Vector2) -> Rect2:
	var s: float = min(size.x / ts.x, size.y / ts.y)
	return Rect2((size - ts * s) / 2.0, ts * s)

func _to_image_pos(p: Vector2, ir: Rect2, disp: float) -> Vector2:
	return (p - ir.position) / disp

func _to_image_rect(a: Vector2, b: Vector2, ir: Rect2, disp: float) -> Rect2:
	var p: Vector2 = _to_image_pos(a, ir, disp)
	var q: Vector2 = _to_image_pos(b, ir, disp)
	return Rect2(Vector2(minf(p.x, q.x), minf(p.y, q.y)), Vector2(absf(q.x - p.x), absf(q.y - p.y)))

## Image 上画矩形描边（四条边填充），坐标夹取在图像内。
func _fill_rect_outline(out: Image, r: Rect2, col: Color, t: int) -> void:
	var w: int = out.get_width()
	var h: int = out.get_height()
	var x0: int = clampi(int(round(r.position.x)), 0, w)
	var y0: int = clampi(int(round(r.position.y)), 0, h)
	var x1: int = clampi(int(round(r.end.x)), 0, w)
	var y1: int = clampi(int(round(r.end.y)), 0, h)
	if x1 <= x0 or y1 <= y0:
		return
	var tw: int = mini(t, x1 - x0)
	var th: int = mini(t, y1 - y0)
	out.fill_rect(Rect2i(x0, y0, x1 - x0, th), col)
	out.fill_rect(Rect2i(x0, y1 - th, x1 - x0, th), col)
	out.fill_rect(Rect2i(x0, y0, tw, y1 - y0), col)
	out.fill_rect(Rect2i(x1 - tw, y0, tw, y1 - y0), col)

## Image 上画椭圆描边：沿椭圆采样、每个采样点盖一个实心圆点，
## 得到等宽的环（比逐像素求椭圆距离简单，且线宽不随长短轴变化）。
func _fill_ellipse_outline(out: Image, center: Vector2, rx: float, ry: float, col: Color, t: int) -> void:
	if rx <= 0.5 or ry <= 0.5:
		return
	var n: int = maxi(32, int(ceil(rx + ry)))
	var r: float = maxf(float(t) * 0.5, 0.5)
	for i in range(n + 1):
		var ang: float = TAU * float(i) / float(n)
		_stamp_disc(out, center + Vector2(cos(ang) * rx, sin(ang) * ry), r, col)

## Image 上盖一个半径 r 的实心圆点（椭圆描边采样用），越界自动丢弃。
func _stamp_disc(out: Image, p: Vector2, r: float, col: Color) -> void:
	var x0: int = maxi(int(floor(p.x - r)), 0)
	var y0: int = maxi(int(floor(p.y - r)), 0)
	var x1: int = mini(int(ceil(p.x + r)), out.get_width() - 1)
	var y1: int = mini(int(ceil(p.y + r)), out.get_height() - 1)
	var r2: float = r * r
	for y: int in range(y0, y1 + 1):
		var dy: float = float(y) - p.y
		for x: int in range(x0, x1 + 1):
			var dx: float = float(x) - p.x
			if dx * dx + dy * dy <= r2:
				out.set_pixel(x, y, col)

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
	if _dragging and (tool == "rect" or tool == "circle"):
		_draw_shape(tool, _drag_origin, _drag_current)

func _draw_annotation(a: Dictionary) -> void:
	var kind: String = str(a.get("kind", ""))
	if kind == "rect":
		draw_rect(Rect2(a.a, a.b - a.a), a.color, false, LINE_W)
	elif kind == "circle":
		_draw_shape("circle", a.a, a.b)

## 两种工具都是 a→b 拖出的矩形外接框：rect 画框本身，circle 画框的内切椭圆。
func _draw_shape(kind: String, a: Vector2, b: Vector2) -> void:
	var box := Rect2(a, b - a).abs()
	if kind == "rect":
		draw_rect(box, ACCENT, false, LINE_W)
		return
	_draw_ellipse(box.get_center(), box.size.x * 0.5, box.size.y * 0.5, ACCENT, LINE_W)

## 椭圆描边（draw_arc 只支持正圆，这里采样成折线）。
func _draw_ellipse(center: Vector2, rx: float, ry: float, color: Color, width: float) -> void:
	if rx <= 0.5 or ry <= 0.5:
		return
	var n: int = 64
	var pts := PackedVector2Array()
	pts.resize(n + 1)
	for i in range(n + 1):
		var ang: float = TAU * float(i) / float(n)
		pts[i] = center + Vector2(cos(ang) * rx, sin(ang) * ry)
	draw_polyline(pts, color, width, true)

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
