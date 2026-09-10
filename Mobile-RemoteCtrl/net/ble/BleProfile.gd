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

## 兜底控制黑名单：WS 掉线时除列出的类型外，其余词表指令都允许经 BLE cmd 特征下发。
## 仅真正依赖 WiFi 的类型才需拦：stream 图传（需 UDP/抓帧）、ai_goal/ai_oneshot（板需 WiFi
## 直连云端）。其余控制/调试/状态类一律支持蓝牙兜底。
const BLOCKED_TYPES := ["stream", "ai_goal", "ai_oneshot"]

## 16-bit 短值拼全量（小写输入，统一大写输出），如 uuid("c0e5")
static func uuid(short: String) -> String:
	return ("0000%s-0000-1000-8000-00805F9B34FB" % short.to_upper())
