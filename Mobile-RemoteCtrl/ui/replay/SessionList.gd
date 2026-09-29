extends PanelContainer
## 会话回放列表（悬浮在聊天区之上，由 AI 接管栏的 SessionBtn 开关）。
## Session1 = 当前会话（live，选中 = 退出回放；不可删），Session2..10 = 归档会话（最多 9 条）。
## 本面板只做"列 / 选中 / 请删"，真正的回放与删除由 Recorder 与 Main 执行。

## 选中某会话；id = -1 表示「当前会话」（回 live）。
signal session_selected(id: int)
## 请求删除某条归档会话（Main 弹确认后再真正删）。
signal delete_requested(id: int)

@onready var _rows: Array = [
	$VBox/Session1, $VBox/Session2, $VBox/Session3, $VBox/Session4, $VBox/Session5,
	$VBox/Session6, $VBox/Session7, $VBox/Session8, $VBox/Session9, $VBox/Session10,
]

var _archived: Array = []
var _selected_id := -1

func _ready() -> void:
	for i in _rows.size():
		var b: Button = _rows[i]
		b.pressed.connect(_on_row_pressed.bind(i))
		var del: TextureButton = b.get_node("DelBtn")
		del.pressed.connect(_on_del_pressed.bind(i))
	Recorder.sessions_changed.connect(refresh)
	refresh()

func toggle() -> void:
	if visible:
		close()
	else:
		open()

func open() -> void:
	refresh()
	visible = true
	AnimationManager.fade_scale_in(self)

func close() -> void:
	visible = false

## 高亮当前正在回放/录制的会话（Main 进入回放后调用）。
func set_selected(id: int) -> void:
	_selected_id = id
	_refresh_pressed()

func refresh() -> void:
	_archived = Recorder.list_sessions()
	for i in _rows.size():
		var b: Button = _rows[i]
		var del: TextureButton = b.get_node("DelBtn")
		if i == 0:
			b.visible = true
			del.visible = false
			b.text = "当前会话"
		else:
			var k := i - 1
			if k < _archived.size():
				var s: Dictionary = _archived[k]
				b.visible = true
				del.visible = true
				b.text = "%s  %s" % [str(s.get("title", "会话")), _fmt_time(int(s.get("created_ms", 0)))]
			else:
				b.visible = false
	_refresh_pressed()

func _refresh_pressed() -> void:
	for i in _rows.size():
		var b: Button = _rows[i]
		b.set_pressed_no_signal(_row_id(i) == _selected_id)

## 行号 → 会话 id：0 = 当前会话(-1)；1.. = 归档；空行返回 -2。
func _row_id(i: int) -> int:
	if i == 0:
		return -1
	var k := i - 1
	if k < 0 or k >= _archived.size():
		return -2
	return int((_archived[k] as Dictionary).get("id", -2))

func _on_row_pressed(i: int) -> void:
	var id := _row_id(i)
	if id == -2:
		return
	_selected_id = id
	session_selected.emit(id)

func _on_del_pressed(i: int) -> void:
	var id := _row_id(i)
	if id >= 0:
		delete_requested.emit(id)

func _fmt_time(ms: int) -> String:
	if ms <= 0:
		return ""
	var d := Time.get_datetime_dict_from_unix_time(int(ms / 1000.0))
	return "%02d-%02d %02d:%02d" % [d["month"], d["day"], d["hour"], d["minute"]]
