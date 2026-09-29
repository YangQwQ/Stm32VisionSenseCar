extends Control
## 图传显示区：承载 WS 传来的 JPEG 帧 TextureRect，无流时显示占位。
## 标定网格：/grid 开关，由子节点 Overlay（GridOverlay.gd）在 Feed 之上绘制。
## 画面源：图传 ↔ 记忆地图，由 OpMap（MemoryMap.gd）铺底绘制。

@onready var _feed: TextureRect = $Feed
@onready var _no_signal: Label = $NoSignal
@onready var _overlay: Control = $Overlay
@onready var _map: Control = $OpMap

var current_texture: Texture2D = null

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
