extends Node
## 本地持久化（autoload `Store`）：last_device / wifi / ai 配置，JSON 存 user://。
## 目的：启动后可据此自动重连上次设备；弹窗预填已存配置，避免反复输入。
## 结构化存取，键不存在返回默认空；写入非空值才用新值覆盖，空值保留旧值。

const _PATH := "user://ctrl_app_settings.json"

## 存储的数据。set_* 会保留未提供的旧字段。
var _data: Dictionary = {
	"last_device": {"address": "", "name": ""},
	"wifi": {"ssid": "", "password": ""},
	"ai": {"url": "", "key": "", "model": ""},
	"settings": {"auto_conn": false, "disable_auto_ws": false, "spin_mode": false},
}

func _ready() -> void:
	_load()

# ============================== 读取 ==============================

func get_last_device() -> Dictionary:
	return _data.get("last_device", {})

func get_wifi() -> Dictionary:
	return _data.get("wifi", {})

func get_ai() -> Dictionary:
	return _data.get("ai", {})

func get_auto_conn() -> bool:
	return bool(_data.get("settings", {}).get("auto_conn", false))

func get_disable_auto_ws() -> bool:
	return bool(_data.get("settings", {}).get("disable_auto_ws", false))

func get_spin_mode() -> bool:
	return bool(_data.get("settings", {}).get("spin_mode", false))

# ============================== 写入 ==============================

## 记录最近成功选中的设备（用于启动自连）。
func set_last_device(address: String, name: String) -> void:
	var d: Dictionary = _data.get("last_device", {})
	if address != "":
		d["address"] = address
	if name != "":
		d["name"] = name
	_data["last_device"] = d
	_save()

func set_wifi(ssid: String, password: String) -> void:
	var w: Dictionary = _data.get("wifi", {})
	if ssid != "":
		w["ssid"] = ssid
	if password != "":
		w["password"] = password
	_data["wifi"] = w
	_save()

func set_ai(url: String, key: String, model: String) -> void:
	var a: Dictionary = _data.get("ai", {})
	if url != "":
		a["url"] = url
	if key != "":
		a["key"] = key
	if model != "":
		a["model"] = model
	_data["ai"] = a
	_save()

func set_auto_conn(v: bool) -> void:
	var s: Dictionary = _data.get("settings", {})
	s["auto_conn"] = v
	_data["settings"] = s
	_save()

func set_disable_auto_ws(v: bool) -> void:
	var s: Dictionary = _data.get("settings", {})
	s["disable_auto_ws"] = v
	_data["settings"] = s
	_save()

func set_spin_mode(v: bool) -> void:
	var s: Dictionary = _data.get("settings", {})
	s["spin_mode"] = v
	_data["settings"] = s
	_save()

# ============================== 文件 ==============================

func _load() -> void:
	var f: FileAccess = FileAccess.open(_PATH, FileAccess.READ)
	if f == null:
		return  # 首次运行无存档，保持默认
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	f.close()
	_merge_defaults()
	if parsed is Dictionary:
		# 只取已知键，忽略未知/异常字段
		var sd: Dictionary = parsed as Dictionary
		_merge_into(_data, sd)

## 保证每个分组至少带默认键，避免旧存档缺字段。
func _merge_defaults() -> void:
	for grp: String in ["last_device", "wifi", "ai", "settings"]:
		if not (_data.get(grp) is Dictionary):
			_data[grp] = _default_group(grp)

## 各分组的默认结构（旧存档缺字段时补）。
func _default_group(grp: String) -> Dictionary:
	match grp:
		"last_device":
			return {"address": "", "name": ""}
		"wifi":
			return {"ssid": "", "password": ""}
		"ai":
			return {"url": "", "key": "", "model": ""}
		"settings":
			return {"auto_conn": false, "disable_auto_ws": false, "spin_mode": false}
		_:
			return {}

func _merge_into(base: Dictionary, over: Dictionary) -> void:
	for k: Variant in over.keys():
		if base.has(k) and base[k] is Dictionary and over[k] is Dictionary:
			_merge_into(base[k], over[k])
		elif base.has(k):
			base[k] = over[k]

func _save() -> void:
	var f: FileAccess = FileAccess.open(_PATH, FileAccess.WRITE)
	if f == null:
		push_warning("本地存储写入失败: %s" % _PATH)
		return
	f.store_string(JSON.stringify(_data))
	f.close()