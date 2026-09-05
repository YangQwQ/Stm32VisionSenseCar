extends Node
## BLE 客户端层（GDBLE 实现）：真实扫描附近小车设备。
## 配网仍为桩：需固件侧定义 GATT 服务/特征后接入。

signal scan_finished(devices: Array)
signal ble_state_changed(state: String)

const SCAN_DURATION := 8.0

var _state := "off"
var _mgr: Node = null
var _initialized := false
var _pending_scan := false

func get_ble_state() -> String:
	return _state

func _set_state(s: String) -> void:
	if _state == s:
		return
	_state = s
	ble_state_changed.emit(s)

func _ready() -> void:
	# GDBLE 通过 ClassDB 注册蓝牙管理器；用 derive/即时化避免未加载时解析失败
	if not ClassDB.class_exists("BluetoothManager"):
		push_warning("GDBLE 未加载，蓝牙扫描不可用")
		_set_state("unavailable")
		return
	_mgr = ClassDB.instantiate("BluetoothManager")
	add_child(_mgr)
	_mgr.call("set_debug_mode", false)
	_mgr.adapter_initialized.connect(_on_adapter_initialized)
	_mgr.scan_started.connect(_on_scan_started)
	_mgr.scan_stopped.connect(_on_scan_stopped)
	_mgr.error_occurred.connect(_on_error)
	_call_if("initialize")

func _on_adapter_initialized(success: bool, error: String) -> void:
	_initialized = success
	if not success:
		push_warning("蓝牙初始化失败: %s" % error)
		_set_state("idle")
		_scan_finish_empty()
		return
	_set_state("idle")
	if _pending_scan:
		_pending_scan = false
		_start_real_scan()

## 扫描附近设备。首次调用会自动初始化蓝牙适配器。
func scan() -> void:
	if _mgr == null:
		_scan_finish_empty()
		return
	if _state == "scanning":
		return
	if not _initialized:
		_pending_scan = true
		if _state != "initializing":
			_call_if("initialize")
		return
	_start_real_scan()

func _start_real_scan() -> void:
	_set_state("scanning")
	_mgr.call("start_scan", SCAN_DURATION)

func _on_scan_started() -> void:
	_set_state("scanning")

func _on_scan_stopped() -> void:
	_set_state("idle")
	var devs: Array = _mgr.call("get_discovered_devices")
	scan_finished.emit(_labels(devs))

func _on_error(message: String) -> void:
	push_warning("蓝牙错误: %s" % message)
	_set_state("idle")

func _labels(devs: Array) -> Array:
	var out: Array = []
	for d in devs:
		var name: String = d.get("name", "")
		if name.is_empty():
			name = d.get("address", "未知设备")
		out.append(name)
	return out

func _scan_finish_empty() -> void:
	scan_finished.emit([])

func _call_if(method: String) -> void:
	if _mgr != null:
		_mgr.call(method)

## 经 GATT 下发配网参数到 ESP32（桩，待固件 GATT 协议确定后接入）。
func provision(ssid: String, password: String) -> void:
	push_warning("BLE 配网为桩实现，未真正发送: %s" % ssid)

func _exit_tree() -> void:
	if _mgr != null:
		_call_if("stop_scan")