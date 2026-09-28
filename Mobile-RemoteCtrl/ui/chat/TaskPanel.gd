extends PanelContainer
## AI 任务面板：悬浮在聊天区（ChatLog）顶部的折叠条，点一下向下展开看完整任务列表。
## 数据源 = 板端 ai_task 上行（{state, round, goal?, note?, tasks:[{name,done}]}）；任务列表原本只进
## 模型上下文和串口日志，手机端拿不到，这条消息才把它结构化送上来。
## 本面板只渲染"最新一份快照"，不判断 AI 是否在跑（那套在 ChatPanel._ai_running / Main 的 bit4 同步里），
## 也不发任何指令 —— 展示与控制在两端是分开的。
## 悬浮在文字区之上，不参与 ChatPanel 的纵向排版，因此不与图传/摇杆抢高度。

## 折叠态标题里目标的字数上限：超出截断，保证后面的进度与状态不被挤掉（完整目标在展开态看）。
const _GOAL_MAX := 16
const _ROW_H := 44.0       # 任务行最小高（名字换行时按实测往上加，见 _label_height）
const _NOTE_H := 40.0      # 笔记测量的兜底值（拿不到字体时用）

const _STATE_TEXT := {
	"running": "进行中",
	"wait": "等你输入",
	"done": "已完成",
	"fail": "失败",
	"abort": "已中止",
}
const _STATE_COLOR := {
	"running": Color("#7fd0bb"),   # 与「AI工具」同色系：任务正在推进
	"wait": Color("#ffd75e"),      # 与「系统提示」同色：AI 在等你回话，必须显眼
	"done": Color("#c9f7a8"),
	"fail": Color("#ff8a80"),
	"abort": Color("#b0b0b0"),
}
const _C_DIM := Color("#8a8a8a")
const _C_TEXT := Color("#e6e6e6")
const _C_NOW := Color("#7fd0bb")

var _expanded := false
var _state := ""
var _goal := ""
var _note := ""
var _tasks: Array = []

@onready var _head: Button = $VBox/Head
## 展开区是 ScrollContainer：任务项没有数量上限（板端 `task_list_replace` 按实际项数分配），
## 条数一多就必须在面板里滚动，不能靠"没有上限所以不会超"。
@onready var _body: ScrollContainer = $VBox/Body
@onready var _note_lb: Label = $VBox/Body/Inner/Note
@onready var _rows: VBoxContainer = $VBox/Body/Inner/Todos

# ============================== 对外接口（ChatPanel 调用） ==============================

## 应用一份板端快照并刷新。整份替换（不做增量）：模型常用 todo 整段重写列表，
## 增量合并会把"重写"误算成"新增"，且任务切换时旧条目会残留。
func apply(params: Variant) -> void:
	if not (params is Dictionary):
		return
	var p := params as Dictionary
	_state = str(p.get("state", ""))
	_goal = str(p.get("goal", ""))
	_note = str(p.get("note", ""))
	_tasks = []
	var ta: Variant = p.get("tasks")
	if ta is Array:
		for it in (ta as Array):
			if not (it is Dictionary):
				continue
			var td := it as Dictionary
			_tasks.append({
				"name": str(td.get("name", "")),
				"done": bool(td.get("done", false)),
			})
	visible = true
	_render()

## 本地乐观置「已中止」：用户点中止 / 掉线时板端还没回终态，先把面板打成中止态。
## 只覆盖"活着"的状态（进行中 / 等你输入）—— 板端已报的 done/fail 终态不能被它盖掉。
func mark_abort_if_live() -> void:
	if _state == "running" or _state == "wait":
		_state = "abort"
		_render()

# ============================== 渲染 ==============================

func _on_head_pressed() -> void:
	_expanded = not _expanded
	_render()

func _render() -> void:
	var st: String = str(_STATE_TEXT.get(_state, "进行中"))
	var col: Color = _STATE_COLOR.get(_state, _STATE_COLOR["running"])
	var done_n := 0
	for t in _tasks:
		if bool((t as Dictionary).get("done", false)):
			done_n += 1
	var title := _goal if _goal != "" else "AI 任务"
	if title.length() > _GOAL_MAX:
		title = title.substr(0, _GOAL_MAX) + "…"
	var arrow := "▾" if _expanded else "▸"
	if _tasks.is_empty():
		_head.text = "%s %s · %s" % [arrow, title, st]
	else:
		_head.text = "%s %s · %d/%d · %s" % [arrow, title, done_n, _tasks.size(), st]
	_head.add_theme_color_override("font_color", col)

	_note_lb.text = ("笔记: %s" % _note) if _note != "" else ""
	_note_lb.visible = _note != ""
	_rebuild_rows()
	_body.visible = _expanded

	# 高度自算而不用容器测量：面板是悬浮层，锚在 ChatLog 顶部（anchor_top=bottom=0），
	# 得自己把展开后的高度写进 offset_bottom，否则永远只有一行高。
	# 板端把任务名/笔记改成按实际长度分配（无字数上限）之后，行与笔记都可能换行，所以高度按
	# **字体实测串宽折行数**算，而不是"一项一行"；再把展开区封顶到聊天区高度，超出的交给滚动条 ——
	# 面板是浮层，撑太高会盖住输入行/摇杆。
	var h := _head.custom_minimum_size.y + _v_margins()
	if _expanded:
		var content := 0.0
		if _note_lb.visible:
			content += _label_height(_note_lb, _avail_w())
		for row in _rows.get_children():
			content += maxf(_ROW_H, _label_height(row.get_child(1), _avail_w() - 46.0))  # 46 = 标记列 + 间隔
		var capped := minf(content, _max_body_h())
		_body.custom_minimum_size.y = capped
		h += capped
	custom_minimum_size.y = h
	offset_bottom = offset_top + h   # 场景里 offset_top 非 0，别把它当成高度起点

## 估一个自动换行 Label 占几行(按其字体实测串宽 / 可用宽度)。不追求绝对精确：
## 只用来给悬浮面板定高, 宁可略大(多几像素空白)也不能略小(文字被裁)。
func _label_height(lb: Label, avail_w: float) -> float:
	var f: Font = lb.get_theme_font("font")
	var fs: int = lb.get_theme_font_size("font_size")
	if f == null or lb.text.is_empty():
		return _NOTE_H
	var w: float = f.get_string_size(lb.text, HORIZONTAL_ALIGNMENT_LEFT, -1, fs).x
	var lines: int = maxi(1, int(ceil(w / maxf(1.0, avail_w))))
	return float(lines) * (f.get_height(fs) + 4.0) + 6.0

## 文本可用的横向宽度（面板宽 - 样式盒左右留白）。滚动条占掉的那点不扣：估宽了只会让面板略高，
## 估窄了才会裁字。
func _avail_w() -> float:
	return maxf(120.0, size.x - _h_margins())

func _h_margins() -> float:
	var sb := get_theme_stylebox("panel")
	return (sb.content_margin_left + sb.content_margin_right) if sb != null else 28.0

func _v_margins() -> float:
	var sb := get_theme_stylebox("panel")
	return (sb.content_margin_top + sb.content_margin_bottom) if sb != null else 16.0

## 展开区上限 = 聊天区高度（再高就会盖到输入行与摇杆）。内容超过它就出滚动条。
func _max_body_h() -> float:
	var host := get_parent() as Control
	var host_h := host.size.y if host != null else 0.0
	return maxf(240.0, host_h - 20.0)

## 重建行（项数少，整体重建比增量 diff 简单可靠）。
func _rebuild_rows() -> void:
	for c in _rows.get_children():
		_rows.remove_child(c)
		c.free()
	var next := true   # 第一个未完成项 = 当前正在做的那一步
	for t in _tasks:
		var td := t as Dictionary
		var done: bool = bool(td.get("done", false))
		var row := HBoxContainer.new()
		row.custom_minimum_size.y = _ROW_H
		row.add_theme_constant_override("separation", 10)

		var mark := Label.new()
		mark.custom_minimum_size.x = 36
		mark.add_theme_font_size_override("font_size", 26)
		row.add_child(mark)

		var nm := Label.new()
		nm.text = str(td.get("name", ""))
		nm.add_theme_font_size_override("font_size", 28)
		nm.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
		nm.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(nm)

		if done:
			mark.text = "✓"
			mark.add_theme_color_override("font_color", _C_DIM)
			nm.add_theme_color_override("font_color", _C_DIM)
		elif next:
			mark.text = "▶"
			mark.add_theme_color_override("font_color", _C_NOW)
			nm.add_theme_color_override("font_color", _C_NOW)
			next = false
		else:
			mark.text = "○"
			mark.add_theme_color_override("font_color", _C_TEXT)
			nm.add_theme_color_override("font_color", _C_TEXT)
		_rows.add_child(row)
