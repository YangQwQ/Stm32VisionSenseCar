extends Control
## 可复用虚拟摇杆：拖拽返回方向向量(-1..1)，松开归零。

signal vector_changed(v: Vector2)
signal released

const KNOB_RADIUS := 34.0
const MAX_RADIUS := 62.0

var _center := Vector2.ZERO
var _knob := Vector2.ZERO
var _active := false
var _vector := Vector2.ZERO

func _ready() -> void:
	_center = size / 2.0
	_knob = _center

func reset() -> void:
	_active = false
	_knob = _center
	_vector = Vector2.ZERO
	queue_redraw()

func _draw() -> void:
	draw_circle(_center, MAX_RADIUS + 18.0, Color(1, 1, 1, 0.12))
	draw_arc(_center, MAX_RADIUS + 8.0, 0.0, TAU, 48, Color(1, 1, 1, 0.35), 3.0)
	draw_circle(_knob, KNOB_RADIUS, Color(1, 1, 1, 0.85))

func _gui_input(event: InputEvent) -> void:
	if event is InputEventScreenTouch:
		if event.pressed:
			_begin_if_inside(event.position)
		else:
			_end()
	elif event is InputEventScreenDrag:
		_drag(event.position)
	elif event is InputEventMouseButton and event.pressed:
		_begin_if_inside(event.position)
	elif event is InputEventMouseMotion and _active:
		_drag(event.position)

func _begin_if_inside(p: Vector2) -> void:
	if p.distance_to(_center) <= MAX_RADIUS + 18.0:
		_active = true
		_apply(p)

func _drag(p: Vector2) -> void:
	if _active:
		_apply(p)

func _apply(p: Vector2) -> void:
	var delta := p - _center
	if delta.length() > MAX_RADIUS:
		delta = delta.normalized() * MAX_RADIUS
	_knob = _center + delta
	_vector = delta / MAX_RADIUS
	vector_changed.emit(_vector)
	queue_redraw()

func _end() -> void:
	if not _active:
		return
	_active = false
	_knob = _center
	_vector = Vector2.ZERO
	vector_changed.emit(_vector)
	released.emit()
	queue_redraw()
