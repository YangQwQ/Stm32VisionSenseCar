extends VBoxContainer
## 蓝牙扫描页：设备列表渲染（边扫边显示）、空提示、刷新按钮旋转动画。
## 扫描/连接的编排由 Main 负责，本类只做页面显示；设备卡片复用 BTDeviceListItem.tscn。

## 设备卡片被选中：转发给 Main 做连接编排。
signal device_selected(name: String, address: String)

const BT_ITEM := preload("res://ui/bluetooth/BTDeviceListItem.tscn")

@onready var _refresh_btn: Button = $RefreshBtn
@onready var _device_vbox: VBoxContainer = $DeviceList/VBox

## 本趟已展示的 address 去重表 + "未发现设备"占位 Label。
var _device_seen: Dictionary = {}
var _empty_hint: Label = null
var _refresh_tween: Tween = null
## 新一轮扫描待首条新设备；出现时才清掉上一轮旧表（避免扫描间隙显示空白）。
var _need_flush := false

## 扫描结束收尾：复位按钮/动画、按地址去重补漏设备，整轮空则显示空提示。
## 返回是否整轮一个设备都没发现（供 Main 决定是否提示权限问题）。
func on_scan_finished(devices: Array) -> bool:
	if _refresh_btn.button_pressed:
		_refresh_btn.set_pressed_no_signal(false)
	_stop_scan_animation()
	# 本轮若有新设备经 on_device_found 已清旧表；这里兜底：仅当确要到条列表才清。
	if _need_flush and devices.size() > 0:
		_need_flush = false
		_flush()
	# 设备在扫描中已逐台加进列表（边扫边显示），这里兜底补漏（按地址去重）。
	for d: Variant in devices:
		if not (d is Dictionary):
			continue
		var dd: Dictionary = d as Dictionary
		var raw_addr: Variant = dd.get("address")
		if not (raw_addr is String) or (raw_addr as String).is_empty():
			continue
		var addr: String = raw_addr as String
		# 去重 key 统一小写：gdble 个别调用对地址大小写不一致，避免同一设备以不同 case 重复入列
		if _device_seen.has(addr.to_lower()):
			continue
		var nm: String = str(dd.get("name", addr))
		if nm.is_empty():
			nm = addr
		_device_seen[addr.to_lower()] = nm
		_add_device_card(nm, addr)
	if _device_seen.is_empty():
		_show_empty_hint()
		return true
	return false

## 扫描中逐台发现：本轮首条新设备先清上一轮旧表；再按地址去重立刻补一张卡片（边扫边显示）。
func on_device_found(device: Dictionary) -> void:
	if _need_flush:
		_need_flush = false
		_flush()
	var raw_addr: Variant = device.get("address")
	if not (raw_addr is String) or (raw_addr as String).is_empty():
		return
	var addr: String = raw_addr as String
	if _device_seen.has(addr.to_lower()):
		return
	var nm: String = str(device.get("name", addr))
	if nm.is_empty():
		nm = addr
	_device_seen[addr.to_lower()] = nm
	_add_device_card(nm, addr)

## 进入扫描：刷新按钮按下态 + 旋转动画；标记"待首条新设备再清旧表"。
func show_scanning() -> void:
	_need_flush = true
	_refresh_btn.set_pressed_no_signal(true)
	_start_scan_animation()

## 清空列表与去重表（上一轮的旧内容，仅在新一轮出现首条设备时才由 _flush 调用）。
func clear() -> void:
	_need_flush = true
	_flush()

## 真正清空：释放旧设备卡片与空提示占位、重置去重表。
func _flush() -> void:
	for child: Node in _device_vbox.get_children():
		child.queue_free()
	_empty_hint = null
	_device_seen.clear()

func device_count() -> int:
	return _device_seen.size()

func _add_device_card(name: String, address: String) -> void:
	if _empty_hint != null:
		_empty_hint.queue_free()
		_empty_hint = null
	var item: Node = BT_ITEM.instantiate()
	item.call("setup", name, address)
	item.connect("selected", Callable(self, "_on_card_selected"))
	_device_vbox.add_child(item)

func _on_card_selected(name: String, address: String) -> void:
	device_selected.emit(name, address)

func _show_empty_hint() -> void:
	if _empty_hint != null:
		return
	var hint := Label.new()
	hint.text = "未发现设备，点击刷新"
	hint.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	hint.add_theme_font_size_override("font_size", 36)
	hint.add_theme_color_override("font_color", Color(0.6, 0.64, 0.72, 1))
	hint.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	_device_vbox.add_child(hint)
	_empty_hint = hint

## 扫描旋转动画：先左旋两圈、再右旋两圈，往复循环；结束由 _stop 复位。
func _start_scan_animation() -> void:
	if _refresh_tween != null:
		_refresh_tween.kill()
	_refresh_btn.offset_transform_rotation = 0.0
	var tw := create_tween().set_loops()
	tw.set_trans(Tween.TRANS_CUBIC).set_ease(Tween.EASE_IN_OUT)
	tw.tween_property(_refresh_btn, "offset_transform_rotation", -TAU * 2.0, 1.5)
	tw.tween_property(_refresh_btn, "offset_transform_rotation", TAU * 2.0, 1.5)
	_refresh_tween = tw

func _stop_scan_animation() -> void:
	if _refresh_tween != null:
		_refresh_tween.kill()
		_refresh_tween = null
	create_tween().tween_property(_refresh_btn, "offset_transform_rotation", 0.0, 0.3)\
		.set_trans(Tween.TRANS_CUBIC).set_ease(Tween.EASE_OUT)
