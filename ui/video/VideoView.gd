extends Control
## 图传显示区：承载 WS 传来的 JPEG 帧 TextureRect，无流时显示占位。

@onready var _feed: TextureRect = $Feed
@onready var _no_signal: Label = $NoSignal

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
