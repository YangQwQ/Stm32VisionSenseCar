extends Node
## WebSocket 客户端：连接小车 ESP32，收发图传帧 / 控制指令 / AI 消息。

const CP := preload("res://net/proto/CommandProto.gd")

signal connected
signal disconnected(reason: String)
signal text_received(data: Dictionary)
signal frame_received(img: Image)

var url := "ws://192.168.4.1:81"

var _peer: WebSocketPeer = null
var _state := "disconnected"  # disconnected / connecting / connected

func get_state() -> String:
	return _state

func is_connected_car() -> bool:
	return _state == "connected"

func connect_car(target_url: String = "") -> void:
	if target_url != "":
		url = target_url
	_teardown()
	_state = "connecting"
	_peer = WebSocketPeer.new()
	if _peer.connect_to_url(url) != OK:
		_state = "disconnected"
		push_warning("WS 连接失败: %s" % url)

## 配网闭环：板子经 BLE 上报 IP 后，据此连 WS（ws://<ip>:81）。
func connect_car_ip(ip: String) -> void:
	if ip.is_empty():
		return
	connect_car("ws://%s:81" % ip)

func disconnect_car() -> void:
	_teardown()

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

func _process(_delta: float) -> void:
	if _peer == null:
		return
	_peer.poll()
	var rs := _peer.get_ready_state()
	if rs == WebSocketPeer.STATE_OPEN:
		if _state != "connected":
			_state = "connected"
			connected.emit()
	elif rs in [WebSocketPeer.STATE_CLOSED, WebSocketPeer.STATE_CLOSING]:
		var reason := ""
		if rs == WebSocketPeer.STATE_CLOSED:
			reason = _peer.get_close_reason()
		if _peer.get_close_code() == 1000:
			reason = "主动断开"
		_state = "disconnected"
		disconnected.emit(reason)
		_teardown()
		return
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
					frame_received.emit(img)

## 统一处理一条 WS 文本指令/应答：解码后派发；pong 打印便于调试。
func _handle_text(text: String) -> void:
	var data: Dictionary = CP.decode(text)
	if data.is_empty():
		return
	if data.get("type", "") == "pong":
		print("[WS] recv text: %s" % text)
	text_received.emit(data)

func _teardown() -> void:
	if _peer != null:
		_peer.close()
		_peer = null
	if _state != "disconnected":
		_state = "disconnected"
