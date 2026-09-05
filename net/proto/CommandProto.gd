extends RefCounted
class_name CommandProto
## 统一命令词表：手机手动控制 / 手机中转审批 / 小车直连 统一经此编解码。

static func move(throttle: float, steering: float, distance_cm: float = 0.0, angle_deg: float = 0.0) -> Dictionary:
	# 省略 distance_cm/angle_deg（或为 0）即持续移动，直到 stop(scope="wheels")。
	var params := {"throttle": throttle, "steering": steering}
	if distance_cm > 0.0:
		params["distance_cm"] = distance_cm
	if angle_deg > 0.0:
		params["angle_deg"] = angle_deg
	return {"type": "move", "params": params, "id": _new_id()}

static func stop(scope: String = "all") -> Dictionary:
	# scope: all / wheels / arm
	return {"type": "stop", "params": {"scope": scope}, "id": _new_id()}

static func arm(act: String, duration_ms: int = 0, dist_cm: float = 0.0) -> Dictionary:
	# act: lift_up / lift_down / clip / release / reach_forward / reach_backward
	# duration_ms == 0 表示持续移动，直到收到 stop(scope="arm")。
	# dist_cm > 0 时表示指定距离（lift/reach 的方向由 act 决定）。
	var params := {"act": act, "duration_ms": duration_ms}
	if dist_cm > 0.0:
		params["dist_cm"] = dist_cm
	return {"type": "arm", "params": params, "id": _new_id()}

static func ai_goal(message: String, annotation: Dictionary = {}) -> Dictionary:
	# DIRECT 链路：手机下发任务目标文本 + 圈选近似区域，板子自采当前帧调用 AI。
	return {"type": "ai_goal", "params": {"message": message, "annotation": annotation}, "id": _new_id()}

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