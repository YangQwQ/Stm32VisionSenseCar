extends RefCounted
class_name CommandProto
## 统一命令词表：手机手动控制 / 手机中转审批 / 小车直连 统一经此编解码。

static func move(throttle: float, steering: float) -> Dictionary:
	return {"type": "move", "params": {"throttle": throttle, "steering": steering}, "id": _new_id()}

static func stop(scope: String = "all") -> Dictionary:
	# scope: all / wheels / arm
	return {"type": "stop", "params": {"scope": scope}, "id": _new_id()}

static func arm(act: String, duration_ms: int = 0) -> Dictionary:
	# act: lift_up / lift_down / clip / release / reach_forward / reach_backward
	# duration_ms == 0 表示持续移动，直到收到 stop(scope="arm")
	return {"type": "arm", "params": {"act": act, "duration_ms": duration_ms}, "id": _new_id()}

static func snapshot(quality: int = 82) -> Dictionary:
	return {"type": "snapshot", "params": {"quality": quality}, "id": _new_id()}

static func stream(on: bool) -> Dictionary:
	return {"type": "stream", "params": {"on": on}, "id": _new_id()}

static func config_wifi(ssid: String, password: String) -> Dictionary:
	return {"type": "config", "params": {"ssid": ssid, "password": password}, "id": _new_id()}

static func ping() -> Dictionary:
	return {"type": "ping", "params": {}, "id": _new_id()}

static func encode(cmd: Dictionary) -> String:
	return JSON.stringify(cmd)

static func decode(text: String) -> Dictionary:
	var data = JSON.parse_string(text)
	return data if data is Dictionary else {}

static var _seq := 0

static func _new_id() -> int:
	_seq += 1
	return _seq