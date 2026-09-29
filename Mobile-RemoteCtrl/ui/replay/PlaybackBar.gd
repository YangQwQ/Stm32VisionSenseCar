extends VBoxContainer
## 回放控制条：上一步 / 暂停↔播放 / 下一步 + 进度条拖动。
## 只发意图信号；真正的回放推进在 Recorder。

signal step_requested(dir: int)
signal toggle_pause_requested
signal seek_requested(ratio: float)

@onready var _prev: Button = $Btns/Button
@onready var _pause: Button = $Btns/Button2
@onready var _next: Button = $Btns/Button3
@onready var _progress: HSlider = $Progress

var _dragging := false

func _ready() -> void:
	_prev.pressed.connect(func(): step_requested.emit(-1))
	_next.pressed.connect(func(): step_requested.emit(1))
	_pause.pressed.connect(func(): toggle_pause_requested.emit())
	_progress.min_value = 0.0
	_progress.max_value = 100.0
	_progress.step = 0.1
	_progress.drag_started.connect(func(): _dragging = true)
	_progress.drag_ended.connect(_on_drag_ended)
	_progress.value_changed.connect(_on_value_changed)

## 拖动期间不 seek（否则每像素都在重建回放）；松手才跳。
func _on_drag_ended(_value_changed: bool) -> void:
	_dragging = false
	seek_requested.emit(float(_progress.value) / 100.0)

## 点轨道（非拖动）也当一次跳转。
func _on_value_changed(v: float) -> void:
	if not _dragging:
		seek_requested.emit(v / 100.0)

func set_progress(ratio: float) -> void:
	if not _dragging:
		_progress.set_value_no_signal(clampf(ratio, 0.0, 1.0) * 100.0)

func set_paused(p: bool) -> void:
	_pause.text = "播放" if p else "暂停"
