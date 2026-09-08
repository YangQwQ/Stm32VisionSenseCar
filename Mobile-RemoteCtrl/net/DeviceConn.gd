extends Node
## 统一「设备连接层」：自建并持有 BLE / WS 两个传输，收口全部连接策略。
## 职责（对 Main / AppState 只暴露统一信号与门面）：
##   - 连接状态：派生统一 state，单一事实源
##   - 信道选路：WS 优先、BLE 兜底（FALLBACK_TYPES 白名单）
##   - 信道切换：WS 上连让出 BLE 射频；WS 下连保 BLE / 双断转恢复
##   - 重连：双断自动扫描 BLE → 发现最近设备 → 自连 →「连上停扫」
## 传输本身（BLEClient / WSCarClient）作为纯通道，不做连接策略。

const BP := preload("res://net/ble/BleProfile.gd")
const BLE_SCRIPT := preload("res://net/ble/BLEClient.gd")
const WS_SCRIPT := preload("res://net/ws/WSCarClient.gd")

## WS 连续重连失败达该次（WSCarClient 每 3s 一次 ≈ 9s）→ 停掉 WS 自旋、转 BLE 恢复。
const WS_FAIL_LIMIT := 3

# --- 对外统一信号（Main / AppState 只订阅这些） ---
## 语义明确：device_* 是 BLE 物理链路，ws_* 是 WS 链路，state_changed 是统一状态。
signal state_changed(state: String)
signal device_connected(address: String, name: String)
signal device_disconnected(reason: String)
signal ws_connected()
signal ws_disconnected(reason: String)
signal device_found(device: Dictionary)
signal scan_finished(devices: Array)
signal scan_started()
signal status_received(data: Dictionary)
signal text_received(data: Dictionary)
signal frame_received(img: Image)

enum Channel { NONE, BLE, WS }

# --- 传输 & 状态 ---
var _ble  # BLEClient
var _ws   # WSCarClient
var _state := "off"          # off / unavailable / idle / scanning / connecting / connected
var _channel := Channel.NONE # 当前主信道
var _device_addr := ""
var _device_name := ""

# --- 双断恢复 ---
var _recovery_active := false
var _auto_target_addr := ""   # 恢复到目标地址（最近设备）
var _auto_target_name := ""
var _pending_target_addr := ""   # 扫描整批列表匹配到的待自连目标；交给 _process 在 manager 空闲帧连接（避开回调重入）
var _last_scan_ms := 0
var _rescan_cd_ms := 0
const _RESCAN_INTERVAL := 5.0        # 仍双断且未在扫时的周期重扫秒数
const _SCAN_WATCHDOG_MS := 12000     # 一轮扫描不该超过它；gdble 漏发 scan_stopped 时的自愈阈值

func _ready() -> void:
	_ble = BLE_SCRIPT.new()
	_ble.name = "BLE"
	_ws = WS_SCRIPT.new()
	_ws.name = "WS"

	# 传输信号 → 内部处理器 → 上收为统一信号。先在 add_child 前挂好，不遗漏初始化的早期 emit。
	_ble.scan_finished.connect(_on_scan_finished)
	_ble.device_found.connect(_on_device_found)
	_ble.ble_state_changed.connect(_on_ble_state)
	_ble.device_connected.connect(_on_ble_connected)
	_ble.device_disconnected.connect(_on_ble_disconnected)
	_ble.status_received.connect(_on_ble_status)

	_ws.connected.connect(_on_ws_connected)
	_ws.disconnected.connect(_on_ws_disconnected)
	_ws.reconnect_failed.connect(_on_ws_reconnect_failed)
	_ws.text_received.connect(text_received.emit)
	_ws.frame_received.connect(frame_received.emit)

	add_child(_ble)
	add_child(_ws)
	set_process(true)

# ============================== 统一状态 ==============================

func _set_state(s: String) -> void:
	if _state == s:
		return
	_state = s
	state_changed.emit(s)

## 派生统一状态：单一事实源。WS 优先判定（WS_ONLY 语义下 BLE 已让出射频仍算 connected）。
func _evaluate_state() -> void:
	if _ws.is_connected_car():
		_set_state("connected")
	elif _ble.is_device_connected():
		_set_state("connected")
	else:
		var bs: String = _ble.get_ble_state()
		if bs in ["scanning", "connecting"]:
			_set_state(bs)
		elif bs == "unavailable":
			_set_state("unavailable")
		else:
			_set_state("idle")

# ============================== 对外门面（Main 遥控 / UI 用） ==============================

func is_online() -> bool:
	return _ws.is_connected_car() or _ble.is_device_connected()

## BLE 物理链路已连且 GATT 就绪（配网等需在此时才能写特征）。
func is_device_connected() -> bool:
	return _ble.is_device_connected()

func get_ble_state() -> String:
	return _ble.get_ble_state()

func get_ws_state() -> String:
	return _ws.get_state()

func scan() -> void:
	_ble.scan()

func stop_scan() -> void:
	_recovery_active = false   # 用户手动打断：同时停掉恢复扫描
	_pending_target_addr = ""
	_ble.stop_scan()

func connect_device(address: String, display_name: String = "") -> bool:
	return _ble.connect_device(address, display_name)

func disconnect_device() -> void:
	_ble.disconnect_device()

func provision(ssid: String, password: String) -> bool:
	return _ble.provision(ssid, password)

func write_ai_config(url: String, key: String, model: String) -> bool:
	return _ble.write_ai_config(url, key, model)

## 手动连 WS（/ws connect）；自动重连由本层策略接管。
func connect_ws(ip: String = "") -> void:
	if ip.is_empty():
		_ws.connect_car()
	else:
		_ws.connect_car_ip(ip)

func disconnect_ws() -> void:
	_ws.disconnect_car()

func ws_is_auto() -> bool:
	return _ws.is_auto_reconnect()

func is_ws_only() -> bool:
	return _channel == Channel.WS

# ============================== 发送选路（WS 优先） ==============================

func send_command(cmd: Dictionary) -> bool:
	var t: String = str(cmd.get("type", ""))
	if _ws.is_connected_car():
		print("[SEND] %s via WS" % t)
		_ws.send_command(cmd)
		return true
	if t in BP.FALLBACK_TYPES and _ble.is_device_connected():
		print("[SEND] %s via BLE" % t)
		_ble.write_cmd(cmd)
		return true
	push_warning("指令未发送（WS/BLE 均不可用）: %s" % t)
	return false

## 编辑图（JPEG 二进制）：仅走 WS。调用方保证先 send_image 后 send_command(ai_goal) 保序。
func send_image(img: Image) -> bool:
	if _ws.is_connected_car():
		_ws.send_image(img)
		return true
	push_warning("编辑图未发送（WS 不可用）")
	return false

func best_transport_name() -> String:
	if _ws.is_connected_car():
		return "WS"
	if _ble.is_device_connected():
		return "BLE"
	return "离线"

# ============================== 传输事件处理 ==============================

func _on_ble_state(s: String) -> void:
	# 以「真实进入 scanning 态」为统一开扫信号源：手动、双断恢复、BLEClient 内部重扫（扫描中再点刷新
	# 的 _restart_scan 路径）都会走到这，保证刷新按钮/动画对任意扫描保持一致、可打断。
	if s == "scanning":
		scan_started.emit()
	_evaluate_state()

func _on_device_found(device: Dictionary) -> void:
	device_found.emit(device)
	var raw_addr: Variant = device.get("address")
	if not (raw_addr is String) or (raw_addr as String).is_empty():
		return
	# 双断恢复：发现目标地址 → 记录待自连并停扫，统一由 _process 在 manager 空闲帧连接。
	if _recovery_active and _auto_target_addr != "" \
			and (raw_addr as String).to_lower() == _auto_target_addr.to_lower():
		_pending_target_addr = _auto_target_addr
		_auto_target_addr = ""
		_ble.stop_scan()

func _on_scan_finished(devices: Array) -> void:
	scan_finished.emit(devices)
	# 双断恢复：gdble 常只在扫描结束才集中上报设备（未必逐台发 device_discovered），
	# 因此在整批列表里再匹配一次目标；命中先存到 _pending_target_addr，
	# 交由 _process 在 manager 空闲（非回调）帧执行自连，避免蓝牙回调栈内重入 1337。
	if _recovery_active and _auto_target_addr != "":
		for d: Variant in devices:
			if not (d is Dictionary):
				continue
			var ra: Variant = (d as Dictionary).get("address")
			if ra is String and not (ra as String).is_empty() \
					and (ra as String).to_lower() == _auto_target_addr.to_lower():
				_pending_target_addr = _auto_target_addr
				_auto_target_addr = ""
				break

func _on_ble_connected(address: String, name: String) -> void:
	_device_addr = address
	_device_name = name
	_recovery_active = false   # BLE 已连：停恢复
	_auto_target_addr = ""
	_pending_target_addr = ""
	_channel = Channel.BLE
	_evaluate_state()
	device_connected.emit(address, name)

func _on_ble_disconnected(reason: String) -> void:
	# BLE 话路结束：若 WS 还活着（WS_ONLY 让出射频场景），WS 仍为主信道，直接忽略不切换。
	if _channel == Channel.WS:
		return
	_channel = Channel.NONE
	_evaluate_state()
	device_disconnected.emit(reason)
	_maybe_recover()

func _on_ble_status(data: Dictionary) -> void:
	status_received.emit(data)
	# 拿到板子 IP → 自动连 WS（若开了「自动建立 WS」）。
	var ip: Variant = data.get("ip")
	if ip is String and not (ip as String).is_empty() and not _ws.is_connected_car():
		if Store.get_disable_auto_ws():
			return
		print("[DeviceConn] 板子上线 ip=%s，自动连 WS" % ip)
		_ws.connect_car_ip(ip as String)

func _on_ws_connected() -> void:
	# WS 上连：主信道转 WS，让出射频（WS_ONLY 语义）。恢复扫描若仍在跑则一并停掉，避免残留。
	_channel = Channel.WS
	_recovery_active = false
	_evaluate_state()
	if _ble.get_ble_state() == "scanning":
		_ble.stop_scan()        # 残留的恢复扫描：停掉（连上停扫兜底）
	elif _ble.is_device_connected():
		print("[DeviceConn] WS_ONLY：断开 BLE 让出射频")
		_ble.disconnect_device()
	ws_connected.emit()

func _on_ws_disconnected(reason: String) -> void:
	# WS 掉线（WSCarClient 已在「从已连接掉线」时去抖 emit）。
	var r := reason.strip_edges()
	if r.is_empty():
		r = "连接中断"
	ws_disconnected.emit(r)   # 视频等 WS 专属功能随之不可用，UI 需感知
	if _ble.is_device_connected():
		# BLE 兜底还活着：继续 WS 自动重连抢回主信道。
		_channel = Channel.BLE
		_evaluate_state()
		return
	_channel = Channel.NONE
	_evaluate_state()
	_maybe_recover()

## WS 连续重连失败达阈值：停掉 WS 自旋，转入 BLE 恢复。
func _on_ws_reconnect_failed(fails: int) -> void:
	if fails >= WS_FAIL_LIMIT:
		_ws.disconnect_car()   # 停自动重连自旋，不再刷「WS 已断开」
		print("[DeviceConn] WS 连续失败 %d 次，转 BLE 恢复" % fails)
		_maybe_recover()

# ============================== 双断恢复（自动扫描重连 + 连上停扫） ==============================

func _maybe_recover() -> void:
	if is_online() or _recovery_active:
		return   # 任一信道还活着或已在恢复：不重复触发
	var last: Dictionary = Store.get_last_device()
	var addr: String = str(last.get("address", ""))
	if addr.is_empty():
		return   # 无最近设备，留给用户手动扫描
	_recovery_active = true
	_auto_target_addr = addr.to_lower()
	_auto_target_name = str(last.get("name", addr))
	print("[DeviceConn] 双断恢复：扫描最近设备 %s" % _auto_target_name)
	_run_recovery_scan()

func _process(delta: float) -> void:
	if not _recovery_active:
		return   # 未在恢复，不抢系统资源
	if is_online():
		_recovery_active = false
		_pending_target_addr = ""
		if _ble.get_ble_state() == "scanning":
			_ble.stop_scan()   # 连上停扫
		return
	if _pending_target_addr != "" and _ble.get_ble_state() == "idle":
		# 上轮整批列表里已匹配目标：在 manager 空闲（非回调）帧执行自连，避开蓝牙回调栈重入 1337。
		_rescan_cd_ms = _RESCAN_INTERVAL * 1000.0   # 连接失败也留冷却再重试，不连帧硬啃
		var addr := _pending_target_addr
		_pending_target_addr = ""
		_ble.connect_device(addr, _auto_target_name)
		return
	if _ble.get_ble_state() == "scanning":
		# 扫描进行中：不攒冷却，等这轮自然结束（给手动按钮留出可打断的间隙）。
		# 兜底：若 gdble 漏发 scan_stopped 使状态长期卡在 scanning，超过阈值强制停扫，交给下轮重扫自愈。
		if Time.get_ticks_msec() - _last_scan_ms > _SCAN_WATCHDOG_MS:
			_ble.stop_scan()
		return
	# 未在扫时才开始攒空闲时长，冷却到期再重扫。
	_rescan_cd_ms -= delta * 1000.0
	if _rescan_cd_ms <= 0.0:
		_run_recovery_scan()

func _run_recovery_scan() -> void:
	_last_scan_ms = Time.get_ticks_msec()
	_rescan_cd_ms = _RESCAN_INTERVAL * 1000.0
	_ble.scan()