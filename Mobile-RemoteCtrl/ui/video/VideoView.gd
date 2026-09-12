extends Control
## 图传显示区：承载 WS 传来的 JPEG 帧 TextureRect，无流时显示占位。
## 标定网格：/grid 开关，由子节点 Overlay（GridOverlay.gd）在 Feed 之上绘制。

@onready var _feed: TextureRect = $Feed
@onready var _no_signal: Label = $NoSignal
@onready var _overlay: Control = $Overlay

var current_texture: Texture2D = null

func set_frame(img: Image) -> void:
	current_texture = ImageTexture.create_from_image(img)
	_feed.texture = current_texture
	_no_signal.visible = false

func show_no_signal(on: bool) -> void:
	_no_signal.visible = on
	if on:
		_feed.texture = null
		current_texture = null

func show_grid(on: bool) -> void:
	# 转发给 Overlay（绘制在 Feed 之上）；Overlay 无图时自动隐藏
	_overlay.call("show_grid", on)
