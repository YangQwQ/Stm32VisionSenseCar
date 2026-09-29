extends Node
## 通用动画管理：淡入+缩放滑入/滑出，供各视图统一调用。

func fade_scale_in(node: Control) -> void:
	if node == null:
		return
	node.pivot_offset = node.size / 2.0
	node.modulate.a = 0.0
	node.scale = Vector2(0.92, 0.92)
	var tw := node.create_tween().set_parallel(true)
	tw.set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_OUT)
	tw.tween_property(node, "modulate:a", 1.0, 0.2)
	tw.tween_property(node, "scale", Vector2.ONE, 0.35)
	tw.chain().tween_callback(func():
		node.pivot_offset = Vector2.ZERO)

func fade_scale_out(node: Control) -> void:
	if node == null:
		return
	node.pivot_offset = node.size / 2.0
	var tw := node.create_tween().set_parallel(true)
	tw.set_trans(Tween.TRANS_BACK).set_ease(Tween.EASE_IN)
	tw.tween_property(node, "modulate:a", 0.0, 0.2)
	tw.tween_property(node, "scale", Vector2(0.9, 0.9), 0.25)
	tw.chain().tween_callback(func():
		node.pivot_offset = Vector2.ZERO)
