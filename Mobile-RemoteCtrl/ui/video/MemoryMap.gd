extends Control
## 记忆地图叠加层：把板端 ai_mem 上报的物体记忆画成车头系俯视图。
## 原点 = 车（车头朝上），+前 = 屏幕上方，+右 = 屏幕右方，单位 cm。
## 只在「画面源 = 记忆」时显示；绘制时铺满底色以盖住图传。
## 位置更新走补间（新目标 → 每帧插值），做出"执行动作时跟着动"的观感。

const _RANGE_CM := 100.0     # 视野半径(cm)：半屏对应这么多厘米
const _TICK_EVERY := 10.0    # 刻度线间距(cm)
const _LABEL_EVERY := 20.0   # 数字标签间距(cm)
const _DOT_R := 10.0
const _LERP_SPEED := 6.0     # 位置补间速度(1/s)
const _EPS := 0.3            # 补间收敛阈值(cm)

const _C_BG := Color(0.043, 0.063, 0.106, 1.0)
const _C_AXIS := Color(1, 1, 1, 0.22)
const _C_TICK := Color(1, 1, 1, 0.35)
const _C_TEXT := Color(0.72, 0.76, 0.84)
const _C_CAR := Color(0.42, 0.72, 1.0)
const _C_FRESH := Color(0.49, 0.95, 0.73)
const _C_STALE := Color(0.62, 0.62, 0.68)

var _active := false
var _objs: Array = []          # [{name:String, r:float, f:float, stale:int}]
var _disp: Dictionary = {}     # name → Vector2(右, 前)，当前显示位置(cm)，用于补间

func set_active(on: bool) -> void:
	if not on and not _active:
		return
	_active = on
	set_process(on)
	if on:
		_snap_disp()  # 重新显示时直接落位，不补间
	queue_redraw()

## 应用一份板端快照（params:{objs:[{name,r,f,stale}], hd:车头朝向}）。整份替换物体表。
## 车自身的全局 x/y 板端已不再上报（里程会漂，喂给 AI 反而误导），这里也用不到。
func apply(params: Variant) -> void:
	if not (params is Dictionary):
		return
	var p := params as Dictionary
	_objs = []
	var oa: Variant = p.get("objs")
	if oa is Array:
		for it in (oa as Array):
			if not (it is Dictionary):
				continue
			var d := it as Dictionary
			var nm: String = str(d.get("name", ""))
			if nm.is_empty():
				continue
			var r: float = float(d.get("r", 0.0))
			var f: float = float(d.get("f", 0.0))
			_objs.append({"name": nm, "r": r, "f": f, "stale": int(d.get("stale", 0))})
			if not _disp.has(nm):
				_disp[nm] = Vector2(r, f)  # 新物体直接落位
	# 丢掉板端已遗忘的物体：否则 _disp 会随会话越攒越大。
	var alive := {}
	for o in _objs:
		alive[str((o as Dictionary).get("name", ""))] = true
	for k: Variant in _disp.keys():
		if not alive.has(str(k)):
			_disp.erase(k)
	set_process(true)   # 即使不在显示态也跑一拍补间，切回来时位置已就位
	queue_redraw()

## 清空全部物体（/clear 开新会话后本地立刻清屏，不必等板端推空快照）。
func clear() -> void:
	_objs = []
	_disp = {}
	queue_redraw()

func _snap_disp() -> void:
	for o in _objs:
		var d := o as Dictionary
		_disp[str(d.get("name", ""))] = Vector2(float(d.get("r", 0.0)), float(d.get("f", 0.0)))

func _process(delta: float) -> void:
	var moving := false
	for o in _objs:
		var d := o as Dictionary
		var nm: String = str(d.get("name", ""))
		var target := Vector2(float(d.get("r", 0.0)), float(d.get("f", 0.0)))
		var cur: Vector2 = _disp.get(nm, target)
		if cur.distance_to(target) > _EPS:
			_disp[nm] = cur.lerp(target, clampf(_LERP_SPEED * delta, 0.0, 1.0))
			moving = true
	if moving or _active:
		queue_redraw()
	if not moving and not _active:
		set_process(false)

# ============================== 绘制 ==============================

func _draw() -> void:
	if not _active:
		return
	var rect := Rect2(Vector2.ZERO, size)
	if size.x <= 1.0 or size.y <= 1.0:
		return
	# 铺底：记忆图要完全盖住图传
	draw_rect(rect, _C_BG)
	var scale: float = minf(size.x, size.y) * 0.5 / _RANGE_CM

	# 刻度（先画，压在轴线下）
	var font: Font = ThemeDB.fallback_font
	var cm := _TICK_EVERY
	while cm <= _RANGE_CM:
		# 横轴（右向）上下两侧的刻度；纵轴（前向）左右两侧的刻度
		draw_line(_plot(Vector2(cm, 0.0), scale), _plot(Vector2(cm, 0.0), scale) + Vector2(0, -8), _C_TICK, 2.0)
		draw_line(_plot(Vector2(-cm, 0.0), scale), _plot(Vector2(-cm, 0.0), scale) + Vector2(0, -8), _C_TICK, 2.0)
		draw_line(_plot(Vector2(0.0, cm), scale), _plot(Vector2(0.0, cm), scale) + Vector2(-8, 0), _C_TICK, 2.0)
		draw_line(_plot(Vector2(0.0, -cm), scale), _plot(Vector2(0.0, -cm), scale) + Vector2(-8, 0), _C_TICK, 2.0)
		if fmod(cm, _LABEL_EVERY) < 0.001:
			var lbl := "%d" % int(cm)
			draw_string(font, _plot(Vector2(cm, 0.0), scale) + Vector2(4, -12), lbl, HORIZONTAL_ALIGNMENT_LEFT, -1, 16, _C_TEXT)
			draw_string(font, _plot(Vector2(0.0, cm), scale) + Vector2(6, 6), lbl, HORIZONTAL_ALIGNMENT_LEFT, -1, 16, _C_TEXT)
		cm += _TICK_EVERY

	# 十字轴
	var o := size * 0.5
	draw_line(Vector2(o.x - _RANGE_CM * scale, o.y), Vector2(o.x + _RANGE_CM * scale, o.y), _C_AXIS, 2.0)
	draw_line(Vector2(o.x, o.y - _RANGE_CM * scale), Vector2(o.x, o.y + _RANGE_CM * scale), _C_AXIS, 2.0)

	# 车（原点）：朝上的三角
	draw_colored_polygon(PackedVector2Array([
		o + Vector2(0, -16), o + Vector2(-11, 12), o + Vector2(11, 12),
	]), _C_CAR)

	# 物体点 + 名字
	for it in _objs:
		var d := it as Dictionary
		var nm: String = str(d.get("name", ""))
		var pos: Vector2 = _disp.get(nm, Vector2(float(d.get("r", 0.0)), float(d.get("f", 0.0))))
		var sp := _plot(pos, scale)
		var stale: int = int(d.get("stale", 0))
		var col: Color = _C_FRESH if stale <= 0 else _C_STALE
		draw_circle(sp, _DOT_R, col)
		if stale > 0:
			draw_arc(sp, _DOT_R + 4.0, 0.0, TAU, 20, col, 2.0)
		draw_string(font, sp + Vector2(_DOT_R + 6.0, 6.0), nm, HORIZONTAL_ALIGNMENT_LEFT, -1, 18, col)

## 车头系(cm) → 屏幕像素：原点是控件中心，+前向上、+右向右。
func _plot(p: Vector2, scale: float) -> Vector2:
	return size * 0.5 + Vector2(p.x * scale, -p.y * scale)
