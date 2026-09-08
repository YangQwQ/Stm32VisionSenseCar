extends Node
## WebSocket 客户端：连接小车 ESP32，收发图传帧 / 控制指令 / AI 消息。
## 断线感知 = 连接事件的 get_ready_state() 关闭 + 应用层"疑似断线确认"。
## 不是周期心跳：仅在连续 3s 没有任何下行数据（含图传帧）——即"本要判定断开"的时刻——
## 发一次 ping 探测做单次确认；3s 内收到板子 pong 说明仍在线（仅临时无数据），不响应才真正判定断开。
## 探测 pong 由本类内部拦截，不会像 /ping 那样转发到消息区。避免长时间空闲时依赖系统 TCP 超时（可达 ~10s）。

const CP := preload("res://net/proto/CommandProto.gd")

signal connected
signal disconnected(reason: String)
signal text_received(data: Dictionary)
signal frame_received(img: Image)
## 连续重连失败（异常断线后每次自动重连触达都发，参数=连续失败次数）。供连接策略据此进入 BLE 恢复。
signal reconnect_failed(fails: int)

var url := "ws://192.168.4.1:81"

var _peer: WebSocketPeer = null
var _state := "disconnected"  # disconnected / connecting / connected

## 自动重连开关：connect_car（BLE 连接 / 上线上报 IP / 手动 /ws connect）置 true；disconnect_car 置 false。
var _auto := false
var _retry_left := 0.0          # 剩余自动重试时间
const _RETRY_SEC := 3.0         # 重试间隔
var _consecutive_fail := 0      # 连续重连失败次数，成功连接即清零

# 断线确认（非周期心跳，见文件头注释）：空闲/响应阈值均 3s。
const _PROBE_INTERVAL_MS := 3000  # 连续无下行数据时长，达到即"本要判定断开"，触发一次探测确认
const _PROBE_TIMEOUT_MS := 3000   # 探测发出后等待 pong 的时长，超时视为真正断开
const PING_FRAME := {"type": "ping", "params": {}}  # 探测载荷，板子回 {type:pong}
var _last_active_ms := 0        # 最近一次收到下行/握手完成的时间（毫秒）
var _probe_pending := false     # 已发探测、等待 pong
var _probe_sent_ms := 0

func get_state() -> String:
	return _state

func is_connected_car() -> bool:
	return _state == "connected"

func is_auto_reconnect() -> bool:
	return _auto

func connect_car(target_url: String = "") -> void:
	var want := url
	if target_url != "":
		want = target_url
	if _peer != null and _state == "connecting" and want == url:
		_auto = true  # 已在连接同一地址：去重，避免配网 IP 上报与连接信号重复触发抖动
		return
	url = want
	_auto = true
	_teardown_peer()
	_state = "connecting"
	_peer = WebSocketPeer.new()
	if _peer.connect_to_url(url) != OK:
		_state = "disconnected"
		push_warning("WS 连接失败: %s" % url)
		_schedule_retry()

## 配网闭环：板子经 BLE 上报 IP 后，据此连 WS（ws://<ip>:81）。
func connect_car_ip(ip: String) -> void:
	if ip.is_empty():
		return
	connect_car("ws://%s:81" % ip)

## 手动断开：关闭自动重连，仅本次生效；后续 connect_car 或 /ws connect 再恢复。
func disconnect_car() -> void:
	_auto = false
	_retry_left = 0.0
	_consecutive_fail = 0
	if _state == "disconnected" and _peer == null:
		return
	_teardown_peer()
	_state = "disconnected"
	disconnected.emit("手动断开")

func send_command(cmd: Dictionary) -> void:
	if not is_connected_car():
		return
	_peer.send_text(CP.encode(cmd))

func send_image(img: Image) -> void:
	if not is_connected_car() or img == null:
		return
	var jpg := img.save_jpg_to_buffer()
	if jpg.size() > 0:
		_peer.send_binary(jpg)

func _process(delta: float) -> void:
	# 自动重连倒计时（_peer 为空期间计时）
	if _auto and _retry_left > 0.0:
		_retry_left -= delta
		if _retry_left <= 0.0:
			_retry_left = 0.0
			connect_car()  # 保持当前 url 重试
	if _peer == null:
		return
	_peer.poll()
	var rs := _peer.get_ready_state()
	if rs == WebSocketPeer.STATE_OPEN:
		if _state != "connected":
			_state = "connected"
			_consecutive_fail = 0  # 连接成功，清零连续失败
			_last_active_ms = Time.get_ticks_msec()  # 握手完成即视为有活动，作为探测计时基准
			_probe_pending = false
			connected.emit()
	elif rs in [WebSocketPeer.STATE_CLOSED, WebSocketPeer.STATE_CLOSING]:
		var reason := ""
		if rs == WebSocketPeer.STATE_CLOSED:
			reason = _peer.get_close_reason()
		if _peer.get_close_code() == 1000:
			reason = "主动断开"
		_abnormal_disconnect(reason)
		return
	# 已连接时也继续读包（Open 分支不提前 return，否则收不到下行）。
	for i in _peer.get_available_packet_count():
		var pkt = _peer.get_packet()
		if pkt is String:
			_handle_text(pkt as String)
		else:
			# Godot WebSocketPeer 可能按二进制交付文本帧：先试按 JSON 文本解析，非文本再当 JPEG 图传。
			var bytes := pkt as PackedByteArray
			var text := bytes.get_string_from_utf8()
			var data: Dictionary = CP.decode(text)
			if not data.is_empty():
				_handle_text(text)
			else:
				var img := Image.new()
				if img.load_jpg_from_buffer(bytes) == OK:
					_last_active_ms = Time.get_ticks_msec()  # 图传帧也是"有活动"，续活防误探测
					frame_received.emit(img)
	# 读包完成后做断线确认（此时活跃时间已按本帧下行更新，判定更准）。
	if _state == "connected":
		_check_keepalive()

## 断线确认（非周期心跳，见文件头注释）：连续 _PROBE_INTERVAL_MS 无下行则发一次 ping 探测；
## 再 _PROBE_TIMEOUT_MS 无 pong 即判定断开触发重连。探测 pong 由 _handle_text 拦截，不进消息区。
func _check_keepalive() -> void:
	var now := Time.get_ticks_msec()
	if _probe_pending:
		if now - _probe_sent_ms >= _PROBE_TIMEOUT_MS:
			_abnormal_disconnect("连接中断")  # 探测无应答 → 真正断开
		return
	if now - _last_active_ms >= _PROBE_INTERVAL_MS:
		_probe_pending = true
		_probe_sent_ms = now
		_peer.send_text(CP.encode(PING_FRAME))

## 统一处理一条 WS 文本指令/应答：解码后派发。
func _handle_text(text: String) -> void:
	var data: Dictionary = CP.decode(text)
	if data.is_empty():
		return
	_last_active_ms = Time.get_ticks_msec()  # 任何下行文本都算有活动，续活防误探测
	if _probe_pending and str(data.get("type", "")) == "pong":
		# 探测应答：确认仍在线，撤销"疑似断开"，不转发给 UI（避免像 /ping 那样把 pong 刷进消息区）。
		_probe_pending = false
		return
	text_received.emit(data)

## 异常断线统一出口：断开并（若处于自动模式）调度重连。reason 为空时给个兜底文案，避免显示"已断开:"。
func _abnormal_disconnect(reason: String) -> void:
	var r := reason.strip_edges()
	if r.is_empty():
		r = "连接中断"
	_teardown_peer()
	_state = "disconnected"
	disconnected.emit(r)
	if _auto:
		_consecutive_fail += 1
		reconnect_failed.emit(_consecutive_fail)
		_schedule_retry()

func _schedule_retry() -> void:
	if _auto and _retry_left <= 0.0:
		_retry_left = _RETRY_SEC

func _teardown_peer() -> void:
	_probe_pending = false
	if _peer != null:
		_peer.close()
		_peer = null
	if _state != "disconnected":
		_state = "disconnected"
