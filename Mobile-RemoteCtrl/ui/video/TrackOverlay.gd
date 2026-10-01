extends Control
## 跟踪目标可视化叠加层：绘制在 Feed 之上（与 Overlay 同级），显示跟踪器锁定的位置与状态。
## 数据源：板端 WS 上行 {"type":"track","u":..,"v":..,"conf":..,"st":".."}（归一化坐标，st=idle 表示无目标）。
## 十字准星 + 圆圈 + 置信度；绿=track，黄=lock，红=lost。

@onready var _feed: TextureRect = $"../Feed"

var target_u := -1.0
var target_v := -1.0
var target_conf := 0.0
var target_st := ""

func set_target(u: float, v: float, conf: float, state: String) -> void:
	target_u = u
	target_v = v
	target_conf = conf
	target_st = state
	queue_redraw()

func clear_target() -> void:
	target_u = -1.0
	target_v = -1.0
	target_conf = 0.0
	target_st = ""
	queue_redraw()

func _draw() -> void:
	if target_u < 0.0 or target_v < 0.0 or target_st == "idle" or _feed == null:
		return
	var fsz := size
	if fsz.x <= 0.0 or fsz.y <= 0.0:
		return
	# 跟踪期板端不推流（novid）：没有画面纹理也要画十字 —— 按 VGA 4:3 兜底算内容矩形。
	var ar_tex := 4.0 / 3.0
	var tex: Texture2D = _feed.texture
	if tex != null and tex.get_width() > 0 and tex.get_height() > 0:
		ar_tex = float(tex.get_width()) / float(tex.get_height())
	# 本节点与 Feed 同锚全屏：算内容 rect（KEEP_ASPECT_CENTERED，与 GridOverlay 一致）
	var ar_box := fsz.x / fsz.y
	var cw := fsz.x
	var ch := fsz.y
	if ar_tex > ar_box:
		ch = fsz.x / ar_tex
	else:
		cw = fsz.y * ar_tex
	var rect := Rect2((fsz - Vector2(cw, ch)) / 2.0, Vector2(cw, ch))
	var pos := rect.position + Vector2(rect.size.x * target_u, rect.size.y * target_v)
	# 状态配色
	var col := Color(0, 1, 0, 0.9)          # 绿 = track
	if target_st == "lost":
		col = Color(1, 0.2, 0.2, 0.9)       # 红 = lost
	elif target_st == "lock":
		col = Color(1, 1, 0, 0.9)           # 黄 = locking
	# 十字准星（臂长 20px，线宽 2px）
	draw_line(pos + Vector2(-20, 0), pos + Vector2(20, 0), col, 2.0)
	draw_line(pos + Vector2(0, -20), pos + Vector2(0, 20), col, 2.0)
	# 圆圈（半径 12px）
	draw_arc(pos, 12.0, 0.0, TAU, 32, col, 1.5)
	# 置信度 + 坐标：把板端报的 (u,v) 直接打在十字旁边 —— 用于定死"对歪了"到底怪谁：
	# 若十字明显偏右而屏幕数字写着 u=0.49 ⇒ 是本文件的内容矩形映射错了（手机端 bug）；
	# 若屏幕上写着 u=0.8 ⇒ 板端确实测到 0.8，是闭环没把它拉到 0.48（板端/控制问题）。
	if target_conf > 0.0:
		var font := ThemeDB.fallback_font
		draw_string(font, pos + Vector2(16, -8), "%.2f" % target_conf,
			HORIZONTAL_ALIGNMENT_LEFT, -1, 12, col)
		draw_string(font, pos + Vector2(16, 6), "u%.2f v%.2f" % [target_u, target_v],
			HORIZONTAL_ALIGNMENT_LEFT, -1, 12, col)