extends Node
## 通用动画管理：淡入+缩放滑入/滑出、上下浮动循环，供各视图统一调用。

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

## 上下浮动循环动画。
func float_loop(node: Control, amount: float = 8.0, duration: float = 1.2) -> void:
	var base := node.position.y
	var tw := node.create_tween().set_loops()
	tw.tween_property(node, "position:y", base - amount, duration).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN_OUT)
	tw.tween_property(node, "position:y", base + amount, duration).set_trans(Tween.TRANS_SINE).set_ease(Tween.EASE_IN_OUT)