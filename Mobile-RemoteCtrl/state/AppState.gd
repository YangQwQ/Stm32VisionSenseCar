extends Node
## 全局状态：统一发送出口（WS 优先、BLE 兜底）与连接状态名的门面。
## 连接策略/信道切换/重连已上收至 DeviceConn，本类只做委托，保持对上层调用签名兼容。

var current_image: Image = null  # 最新一帧，供编辑/框选用

## 统一命令出口：所有手动控制 / 聊天 / 图传开关都经此。路由与选路由 DeviceConn 负责。
## ai_goal / snapshot 图像类不走 BLE（需 WiFi），DeviceConn 仅在 WS 可用时下发。
func send_command(cmd: Dictionary) -> bool:
	return DeviceConn.send_command(cmd)

## 编辑图（JPEG 二进制）直通：仅走 WS（由 DeviceConn 选路）。调用方保证先 send_image 再 send_command。
func send_image(img: Image) -> bool:
	return DeviceConn.send_image(img)

func best_transport_name() -> String:
	return DeviceConn.best_transport_name()

func is_online() -> bool:
	return DeviceConn.is_online()

## 兼容旧出口：WS_ONLY 语义（主信道为 WS 时 BLE 已让出射频，判真）。由 DeviceConn 维护。
func is_ws_only() -> bool:
	return DeviceConn.is_ws_only()