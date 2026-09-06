extends RefCounted
## BLE 协议常量表——与固件 Stm32-Vision `ble.h` 逐字 mirror，改一侧必须同步另一侧。
## 服务 0000C0DE-…，特征 0000C0E0~C0E6（16-bit 派生 128-bit）。
## 用法：const BP := preload("res://net/ble/BleProfile.gd")，然后 BP.SVC_UUID / BP.uuid("c0e5")。

const SVC_SHORT := "c0de"
const SSID_SHORT := "c0e0"    # wifi_ssid  写
const PASS_SHORT := "c0e1"    # wifi_pass  写
const AI_URL_SHORT := "c0e2"  # ai_url     写
const AI_KEY_SHORT := "c0e3"  # ai_key     写
const AI_MODEL_SHORT := "c0e4" # ai_model  写
const CMD_SHORT := "c0e5"     # cmd        写（词表 JSON）
const STATUS_SHORT := "c0e6"  # status     读/通知

const SVC_UUID := "0000C0DE-0000-1000-8000-00805F9B34FB"
const SSID_UUID := "0000C0E0-0000-1000-8000-00805F9B34FB"
const PASS_UUID := "0000C0E1-0000-1000-8000-00805F9B34FB"
const AI_URL_UUID := "0000C0E2-0000-1000-8000-00805F9B34FB"
const AI_KEY_UUID := "0000C0E3-0000-1000-8000-00805F9B34FB"
const AI_MODEL_UUID := "0000C0E4-0000-1000-8000-00805F9B34FB"
const CMD_UUID := "0000C0E5-0000-1000-8000-00805F9B34FB"
const STATUS_UUID := "0000C0E6-0000-1000-8000-00805F9B34FB"

## 广播名，与固件一致（固件扫描/过滤可用它或 Service UUID c0de）
const ADVERT_NAME := "VisionS3"

## 兜底控制白名单：WS 掉线时这些词表指令允许经 BLE cmd 特征下发。
## 图传/ai_goal 需 WiFi（抓帧 + 云端 HTTP），不进白名单；ai_cancel 取消任务应离线可用。
const FALLBACK_TYPES := ["move", "stop", "arm", "ping", "config", "stream", "snapshot", "ai_cancel"]

## 16-bit 短值拼全量（小写输入，统一大写输出），如 uuid("c0e5")
static func uuid(short: String) -> String:
	return ("0000%s-0000-1000-8000-00805F9B34FB" % short.to_upper())
