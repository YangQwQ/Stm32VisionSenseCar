extends Control
## 图传标定网格叠加层：/grid 开关。绘制在 Feed 之上（Overlay 为 Video 子层最后一个节点），
## 网格对齐图片内容（扣除 KEEP_ASPECT 黑边），顶边 u 刻度、左边 v 刻度 + 中心十字 + 对角线。
## 配合板端单应标定：把物体放到网格交点，读 (u,v) 与地面实测距离即得标定点。

@onready var _feed: TextureRect = $"../Feed"

var grid_on := false

func show_grid(on: bool) -> void:
	grid_on = on
	queue_redraw()

func _draw() -> void:
	if not grid_on or _feed == null or _feed.texture == null:
		return
	var fsz := size
	var tex: Texture2D = _feed.texture
	if fsz.x <= 0.0 or fsz.y <= 0.0 or tex.get_width() <= 0:
		return
	# Overlay 与 Feed 同锚全屏：本节点 rect 即图片容器；算内容 rect（KEEP_ASPECT_CENTERED）
	var ar_tex := float(tex.get_width()) / float(tex.get_height())
	var ar_box := fsz.x / fsz.y
	var cw := fsz.x
	var ch := fsz.y
	if ar_tex > ar_box:
		ch = fsz.x / ar_tex
	else:
		cw = fsz.y * ar_tex
	var rect := Rect2((fsz - Vector2(cw, ch)) / 2.0, Vector2(cw, ch))
	var col_grid := Color(1, 1, 1, 0.4)
	var col_major := Color(1, 0.45, 0.25, 0.92)
	var fracs := [0.25, 0.5, 0.75]
	# 网格线
	for f in fracs:
		draw_line(rect.position + Vector2(rect.size.x * f, 0.0),
			rect.position + Vector2(rect.size.x * f, rect.size.y), col_grid, 1.0)
		draw_line(rect.position + Vector2(0.0, rect.size.y * f),
			rect.position + Vector2(rect.size.x, rect.size.y * f), col_grid, 1.0)
	# 中心十字 + 对角线（主参考线）
	draw_line(rect.position + Vector2(rect.size.x * 0.5, 0.0),
		rect.position + Vector2(rect.size.x * 0.5, rect.size.y), col_major, 2.0)
	draw_line(rect.position + Vector2(0.0, rect.size.y * 0.5),
		rect.position + Vector2(rect.size.x, rect.size.y * 0.5), col_major, 2.0)
	draw_line(rect.position, rect.position + rect.size, col_major, 2.0)
	draw_line(rect.position + Vector2(rect.size.x, 0.0),
		rect.position + Vector2(0.0, rect.size.y), col_major, 2.0)
	# 刻度（顶边 = u，左边 = v；0.25/0.5/0.75）
	var font := ThemeDB.fallback_font
	var tc := Color(1, 1, 1, 0.95)
	for f in fracs:
		draw_string(font, rect.position + Vector2(rect.size.x * f + 3.0, 13.0),
			"%.2f" % f, HORIZONTAL_ALIGNMENT_LEFT, -1, 13, tc)
		draw_string(font, rect.position + Vector2(3.0, rect.size.y * f + 13.0),
			"%.2f" % f, HORIZONTAL_ALIGNMENT_LEFT, -1, 13, tc)
	draw_string(font, rect.position + Vector2(rect.size.x * 0.5 + 4.0, 13.0),
		"0.50", HORIZONTAL_ALIGNMENT_LEFT, -1, 13, col_major)
