extends Node
## BLE 客户端层（GDBLE 实现）：扫描附近小车 → 连接 → GATT 读写（配网 / 兜底控制 / 状态订阅）。
## GATT 服务/特征 UUID 见 BleProfile.gd，与固件 Stm32-Vision `ble.h` mirror。
##
## gdble API（Rust, D:\Downloads\Git\gdble，已核对源码）：
##   BluetoothManager: initialize / start_scan(t) / stop_scan / get_discovered_devices
##     / connect_device(addr)->BleDevice / disconnect_device(addr) / get_device(addr)
##   BleDevice (RefCounted): connect_async() / disconnect() / is_connected()
##     / discover_services() / get_name()
##     / write_characteristic(svc, char, data:PackedByteArray, with_response:bool)
##     / read_characteristic(svc, char) / subscribe_characteristic(svc, char)
##   BleDevice signals: connected / disconnected / connection_failed(err)
##     / services_discovered / characteristic_written(char) / characteristic_notified(char, data)

signal scan_finished(devices: Array)
## 扫描期间逐台上报（发现了就立刻弹，不必等整轮结束）。与 scan_finished 的差异：单台、实时。
signal device_found(device: Dictionary)
signal ble_state_changed(state: String)
signal device_connected(address: String, name: String)
signal device_disconnected(reason: String)
signal status_received(data: Dictionary)

const SCAN_DURATION := 8.0
const BP := preload("res://net/ble/BleProfile.gd")

var _state := "off"      # off / unavailable / initializing / idle / scanning / connecting / connected
var _mgr: Node = null
var _initialized := false
var _pending_scan := false
var _restart_scan := false   # 扫描中又点刷新：当前扫描结束后立即重扫一轮

# GATT 客户端状态
var _dev: Variant = null            # BleDevice (RefCounted)，connect_device 返回的句柄
var _dev_addr := ""
var _dev_name := ""
var _gatt_ready := false            # 服务发现完成、可读写

func get_ble_state() -> String:
	return _state

func is_device_connected() -> bool:
	return _state == "connected" and _gatt_ready and _dev != null

func _set_state(s: String) -> void:
	if _state == s:
		return
	_state = s
	ble_state_changed.emit(s)

func _ready() -> void:
	# GDBLE 通过 ClassDB 注册蓝牙管理器；用 derive/即时化避免未加载时解析失败
	if not ClassDB.class_exists("BluetoothManager"):
		push_warning("GDBLE 未加载，蓝牙不可用")
		_set_state("unavailable")
		return
	_mgr = ClassDB.instantiate("BluetoothManager")
	add_child(_mgr)
	_mgr.call("set_debug_mode", false)
	_mgr.adapter_initialized.connect(_on_adapter_initialized)
	_mgr.scan_started.connect(_on_scan_started)
	_mgr.scan_stopped.connect(_on_scan_stopped)
	_mgr.error_occurred.connect(_on_error)
	_mgr.device_discovered.connect(_on_device_discovered)
	# Android 12+：附近设备扫描/连接需要 BLUETOOTH_SCAN/CONNECT 运行时权限
	if OS.get_name() == "Android":
		OS.request_permissions()
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

# ============================== 扫描 ==============================

## 扫描附近设备。首次调用会自动初始化蓝牙适配器。
func scan() -> void:
	if _mgr == null:
		_scan_finish_empty()
		return
	if _state == "scanning":
		# 扫描中又点刷新：不吞掉，先停旧扫描，结束后 _on_scan_stopped 里立即重扫。
		# 也顺带自愈"上一轮没收到 scan_stopped 导致状态卡死 scanning"的情况。
		_restart_scan = true
		print("[BLE] scan() 遇扫描中，当前轮结束后重扫")
		_mgr.call("stop_scan")
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

## 提前结束扫描（不重扫）。扫描中由 UI 松开刷新按钮调用。
func stop_scan() -> void:
	_restart_scan = false   # 手动停，不重扫
	if _state == "scanning" and _mgr != null:
		_mgr.call("stop_scan")

func _on_scan_started() -> void:
	_set_state("scanning")

func _on_scan_stopped() -> void:
	_set_state("idle")
	var devs: Array = _mgr.call("get_discovered_devices")
	var named := _named_devices(devs)
	print("[BLE] 扫描结束，发现设备: ", str(named))
	scan_finished.emit(named)
	if _restart_scan:
		_restart_scan = false
		_start_real_scan()

func _on_error(message: String) -> void:
	push_warning("蓝牙错误: %s" % message)
	var was_scanning := _state == "scanning"
	_set_state("idle")
	if was_scanning:
		# 扫描失败也要收尾下拉框，否则停在"扫描中…"；顺带清掉排队重扫
		_restart_scan = false
		_scan_finish_empty()

func _named_devices(devs: Array) -> Array:
	# 收「广播了名字」的设备，并额外收「广播了我们服务 c0de」的目标设备（VisionS3 常只广播
	# UUID、把名字留在连接后才能拿到，name 为 null 时不能丢）。其余匿名设备（耳机/手环噪音）丢弃。
	# 返回仍带 address 的字典，供 DeviceList 选中后按地址连接。
	var out: Array = []
	for d: Variant in devs:
		if not (d is Dictionary):
			continue
		var kept: Dictionary = _filter_device(d as Dictionary)
		if not kept.is_empty():
			out.append(kept)
	return out

## 单台设备筛选：名字非空 或 广播了我们的服务 c0de 才保留。返回 {name,address}，丢弃返回 {}。
## gdble 字典 "name" 键可能为 null，先判型再取值（typed 直接赋 String 会在 Nil 时中断函数）。
func _filter_device(d: Dictionary) -> Dictionary:
	var raw_name: Variant = d.get("name")
	var raw_addr: Variant = d.get("address")
	if not (raw_addr is String) or (raw_addr as String).is_empty():
		return {}
	var keep := false
	if raw_name is String and not (raw_name as String).is_empty():
		keep = true
	elif _advertises_ours(d):
		keep = true
		# 名字拿到前先占位，连接成功后 BLEClient 会用 get_name() 补齐。
		raw_name = ""
	if not keep:
		return {}
	return {"name": raw_name, "address": raw_addr}

## gdble 扫描中逐台上报 device_discovered：过滤后只转发目标设备给 UI，实现"边扫边显示"。
func _on_device_discovered(info: Variant) -> void:
	if _state != "scanning" or not (info is Dictionary):
		return
	var kept: Dictionary = _filter_device(info as Dictionary)
	if not kept.is_empty():
		device_found.emit(kept)

## 广播记录里是否声明了我们的服务（SVC SHORT c0de，gdble 统一转小写）。
func _advertises_ours(dict: Dictionary) -> bool:
	var services: Variant = dict.get("services")
	if not (services is Array):
		return false
	var svc := BP.SVC_UUID.to_lower()
	for s: Variant in services:
		if s is String and (s as String).to_lower() == svc:
			return true
	return false

func _scan_finish_empty() -> void:
	scan_finished.emit([])

# ============================== 连接 ==============================

## 连接指定地址的设备（来自扫描结果）。成功开始连接流程返回 true；连接进度由信号上报。
func connect_device(address: String, display_name: String = "") -> bool:
	if _mgr == null or address.is_empty():
		return false
	_teardown_device()
	_dev_addr = address
	_dev_name = display_name
	_gatt_ready = false
	var dev: Variant = _mgr.call("connect_device", address)
	if dev == null:
		push_warning("BLE 连接失败：无法解析设备 %s" % address)
		_dev_addr = ""
		_dev_name = ""
		_set_state("idle")
		return false
	_dev = dev
	# gdble 按地址缓存同一 BleDevice，复用时旧 handler 可能仍挂着。BleDevice 覆盖了 Object 的
	# is_connected/disconnect（都是 0 参 GATT 方法），信号不能按 Object 语义断开；改为幂等连接：
	# handler 已在 get_signal_connection_list 里则跳过、不再重复挂，绕开 shadow 也避免重复回调。
	for sig: String in _DEV_SIGNALS:
		_connect_if_absent(sig, Callable(self, _DEV_SIGNALS[sig]))
	# RefCounted 动态连接信号：用字符串 connect 规避对 Variant 的静态成员访问
	_set_state("connecting")
	_dev.call("connect_async")
	return true

func disconnect_device() -> void:
	_teardown_device()

func _on_device_connected() -> void:
	if _dev_name.is_empty() and _dev != null:
		var n: String = _dev.call("get_name")
		_dev_name = n
	_set_state("connected")
	print("[BLE] 已连接: %s (%s)" % [_dev_name, _dev_addr])
	device_connected.emit(_dev_addr, _dev_name)
	# 连接成功即发现服务，之后才能读写特征
	if _dev != null:
		_dev.call("discover_services")

func _on_services_discovered(_services: Array) -> void:
	_gatt_ready = true
	print("[BLE] 服务发现完成，订阅状态特征")
	# 订阅 status 通知 + 主动读一次（配网后板子重启，重连时靠读拿 IP）
	if _dev != null:
		_dev.call("subscribe_characteristic", BP.SVC_UUID, BP.STATUS_UUID)
		_dev.call("read_characteristic", BP.SVC_UUID, BP.STATUS_UUID)

func _on_device_disconnected() -> void:
	var reason := "连接已断开"
	_set_state("idle")
	_teardown_device()
	device_disconnected.emit(reason)

func _on_device_connection_failed(error: String) -> void:
	push_warning("BLE 连接失败: %s" % error)
	_set_state("idle")
	_teardown_device()
	device_disconnected.emit("连接失败")

func _on_operation_failed(_operation: String, error: String) -> void:
	push_warning("BLE 操作失败: %s" % error)

# ============================== GATT 读写 ==============================

## 写词表 JSON 到 cmd 特征（兜底控制通道）。需服务发现完成。
func write_cmd(cmd: Dictionary) -> bool:
	if not is_device_connected():
		return false
	return _write_char(BP.CMD_UUID, JSON.stringify(cmd).to_utf8_buffer())

## 写 Wi-Fi 配置（配网）。两特征都下发成功才返回 true。
func write_wifi(ssid: String, password: String) -> bool:
	if not is_device_connected():
		return false
	var ok1 := _write_char(BP.SSID_UUID, ssid.to_utf8_buffer())
	var ok2 := _write_char(BP.PASS_UUID, password.to_utf8_buffer())
	return ok1 and ok2

## 写 AI 接口配置（DIRECT 链路用）。
func write_ai_config(url: String, key: String, model: String) -> bool:
	if not is_device_connected():
		return false
	var ok := true
	if not url.is_empty():
		ok = ok and _write_char(BP.AI_URL_UUID, url.to_utf8_buffer())
	if not key.is_empty():
		ok = ok and _write_char(BP.AI_KEY_UUID, key.to_utf8_buffer())
	if not model.is_empty():
		ok = ok and _write_char(BP.AI_MODEL_UUID, model.to_utf8_buffer())
	return ok

func _write_char(char_uuid: String, data: PackedByteArray) -> bool:
	if _dev == null or data.is_empty():
		return false
	# with_response=true：Android 默认 MTU 23（单包 20B），长指令会被切成多次 onWrite；
	# 固件 ble 层已按"攒到可解析 JSON"处理。命令 JSON 请保持简短。
	_dev.call("write_characteristic", BP.SVC_UUID, char_uuid, data, true)
	return true

## 配网入口：下发 SSID/PASS。板子收到后重启连网，连接会断开；重启完成后
## 手机重新连接该设备，BLEClient 读/订阅 status 拿到 {ip} 即触发 status_received。
func provision(ssid: String, password: String) -> bool:
	if not is_device_connected():
		push_warning("BLE 配网：未连接设备")
		return false
	var ok := write_wifi(ssid, password)
	if ok:
		print("[BLE] 配网参数已下发，板子将重启连接 WiFi；重启后请重新连接设备获取 IP")
	return ok

func _on_characteristic_notified(char_uuid: String, data: PackedByteArray) -> void:
	if _is_status_char(char_uuid):
		_handle_status_bytes(data)

func _on_characteristic_read(char_uuid: String, data: PackedByteArray) -> void:
	if _is_status_char(char_uuid):
		_handle_status_bytes(data)

func _on_characteristic_written(_char_uuid: String) -> void:
	pass

func _is_status_char(char_uuid: String) -> bool:
	# gdble 侧 uuid 可能大小写/长短不一致，统一小写全量比较
	return char_uuid.to_lower() == BP.STATUS_UUID.to_lower()

func _handle_status_bytes(data: PackedByteArray) -> void:
	var text := data.get_string_from_utf8()
	if text.is_empty():
		return
	var parsed = JSON.parse_string(text)
	if parsed is Dictionary:
		status_received.emit(parsed)
	else:
		print("[BLE] status 非 JSON: ", text)

# ============================== 清理 ==============================

## 连接失败等路径会在信号已被拆除后再次 teardown，只需 GATT 断开、不再动信号（信号幂等挂，无需断）。
const _DEV_SIGNALS := {
	"connected": "_on_device_connected",
	"disconnected": "_on_device_disconnected",
	"connection_failed": "_on_device_connection_failed",
	"services_discovered": "_on_services_discovered",
	"characteristic_written": "_on_characteristic_written",
	"characteristic_notified": "_on_characteristic_notified",
	"characteristic_read": "_on_characteristic_read",
	"operation_failed": "_on_operation_failed",
}

## 幂等连信号：handler 已挂则跳过。BleDevice 覆盖了 Object 的 is_connected/disconnect（0 参 GATT 方法），
## 信号不能按 Object 语义断开；改在连接前查 get_signal_connection_list 判重，避免 1337 与重复回调。
func _connect_if_absent(sig: String, handler: Callable) -> void:
	if _dev == null or not is_instance_valid(_dev):
		return
	for c: Variant in _dev.get_signal_connection_list(sig):
		var cd := c as Dictionary
		if cd.get("callable") == handler:
			return
	_dev.connect(sig, handler)

func _teardown_device() -> void:
	if _dev != null and is_instance_valid(_dev):
		_dev.call("disconnect")
	_dev = null
	_dev_addr = ""
	_dev_name = ""
	_gatt_ready = false

func _call_if(method: String) -> void:
	if _mgr != null:
		_mgr.call(method)

func _exit_tree() -> void:
	_teardown_device()
	if _mgr != null:
		_call_if("stop_scan")
