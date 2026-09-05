extends Node
## 全局状态：持有连接对象引用 + 统一发送出口（WS 优先、BLE 兜底）。
## DIRECT 为主链路：AI 由小车端处理，手机只下发 ai_goal / 观察。

const BP := preload("res://net/ble/BleProfile.gd")

# 连接对象（BLEClient / WSCarClient 节点，由 Main._ready 赋值）。无类型以便动态调用子类方法。
var ble
var ws

var current_image: Image = null  # 最新一帧，供编辑/框选用

## 统一命令出口：所有手动控制 / 聊天 / 图传开关都经此。
## WS 在线走 WS；否则 BLE 已连且词表在兜底白名单内则走 BLE cmd 特征；
## 都不可用则告警丢弃。ai_goal / snapshot 图像类不走 BLE（需 WiFi）。
func send_command(cmd: Dictionary) -> bool:
	var t: String = str(cmd.get("type", ""))
	if _ws_ready():
		ws.send_command(cmd)
		return true
	if t in BP.FALLBACK_TYPES and _ble_ready():
		ble.write_cmd(cmd)
		return true
	push_warning("指令未发送（WS/BLE 均不可用）: %s" % t)
	return false

func best_transport_name() -> String:
	if _ws_ready():
		return "WS"
	if _ble_ready():
		return "BLE"
	return "离线"

func is_online() -> bool:
	return _ws_ready() or _ble_ready()

## 挂 BLE status 自动闭环：板子上报 ip 且 WS 未连时，自动连 WS。
## 由 Main._ready 在赋值 ble/ws 后调用一次。
func hook_auto_ws() -> void:
	if ble != null and not ble.status_received.is_connected(_on_ble_status):
		ble.status_received.connect(_on_ble_status)

func _on_ble_status(data: Dictionary) -> void:
	var ip: Variant = data.get("ip")
	if ip is String and not (ip as String).is_empty() and ws != null and not _ws_ready():
		print("[AppState] 板子上线 ip=%s，自动连 WS" % ip)
		ws.connect_car_ip(ip as String)

func _ws_ready() -> bool:
	return ws != null and ws.has_method("is_connected_car") and ws.is_connected_car()

func _ble_ready() -> bool:
	return ble != null and ble.has_method("is_device_connected") and ble.is_device_connected()
