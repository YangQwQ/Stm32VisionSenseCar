extends Control
## 图传显示区：承载 WS 传来的 JPEG 帧 TextureRect，无流时显示占位。
## 标定网格：/grid 开关，由子节点 Overlay（GridOverlay.gd）在 Feed 之上绘制。
## 画面源：图传 ↔ 记忆地图，由 OpMap（MemoryMap.gd）铺底绘制。

@onready var _feed: TextureRect = $Feed
@onready var _no_signal: Label = $NoSignal
@onready var _overlay: Control = $Overlay
@onready var _track: Control = $TrackOverlay
@onready var _map: Control = $OpMap

var current_texture: Texture2D = null
var _track_novid := false   # 跟踪期板端不推视频：显示"标尺网格 + 跟踪点"占位而非冻结的旧画面
var _grid_before := false   # 进跟踪前的网格开关（用户自己的设置），退出时原样还原

func set_frame(img: Image) -> void:
	current_texture = ImageTexture.create_from_image(img)
	_feed.texture = current_texture
	_no_signal.visible = false

func show_no_signal(on: bool) -> void:
	# 记忆图铺满时不该再叠「未连接 / 无信号」
	_no_signal.visible = on and not _map.visible
	if on:
		_feed.texture = null
		current_texture = null

func show_grid(on: bool) -> void:
	# 转发给 Overlay（绘制在 Feed 之上）；Overlay 无图时自动隐藏
	_overlay.call("show_grid", on)

## 跟踪目标位置（板端 WS 上行 type:"track"）：转发给 TrackOverlay 绘制十字/圆圈。
func set_track_target(u: float, v: float, conf: float, state: String) -> void:
	_track.call("set_target", u, v, conf, state)

func clear_track() -> void:
	_track.call("clear_target")

## auto_grasp 播种前：把 AI 标的框交给叠加层画在当前这一帧上（切跟踪画面之前那几帧视频还在推）。
func set_seed_box(u: float, v: float, w: float, h: float) -> void:
	_track.call("set_seed_box", u, v, w, h)

## 跟踪期板端不推视频（track 消息带 novid=1）：藏掉冻结的旧画面，自动开"标尺网格"，
## 由 TrackOverlay 在图内画跟踪点。跟踪结束（novid=0 / idle）恢复原状。
## 注：Feed 只是隐藏、texture 保留 —— 网格与十字都要用它算 KEEP_ASPECT 的内容矩形。
func set_track_novid(on: bool) -> void:
	if on == _track_novid:
		return
	_track_novid = on
	if on:
		# 记住用户自己的网格开关：退出时必须还原，否则"跟踪结束后网格被关掉"（用户实测的 bug）
		_grid_before = bool(_overlay.get("grid_on"))
		_feed.visible = false
		_no_signal.visible = false
		show_grid(true)
	else:
		_feed.visible = not _map.visible
		show_grid(_grid_before)
		_no_signal.visible = _feed.texture == null and not _map.visible

## 画面源切换：图传 ↔ 记忆地图（记忆图铺满底色，故同时藏掉 Feed）。
func show_map(on: bool) -> void:
	_map.visible = on
	_map.call("set_active", on)
	_feed.visible = not on
	if on:
		_no_signal.visible = false
	else:
		# 切回图传：没有画面时要把占位提示还原，否则空着也不提示
		_no_signal.visible = _feed.texture == null

## 清空记忆地图数据（/clear 开新会话时用；板端不会主动推一份空快照）。
func clear_map() -> void:
	_map.call("clear")

## 记忆地图数据（板端 ai_mem 上行）：整份替换。
func show_map_data(params: Variant) -> void:
	_map.call("apply", params)
