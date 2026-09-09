extends Node
## UDP 图传接收端：用 Godot 内置 PacketPeerUDP 收板子推送的 JPEG 分片，重组整帧后发 frame_received。
## 图传视频帧现走 UDP（更稳的低延迟）；WS 仅保留控制/指令与 snapshot 单帧。
## 分片协议与板子 app_httpd.cpp 的 udp_send_frame 对齐：每数据报 14B 大端头 +
## [magic(2) frame_id(4) seq(2) count(2) total(4)] + ≤1400B JPEG 负载。
## 重组策略：按 frame_id 缓存分片，count 片齐则解码上抛；缺片/UDP 丢包则等下一帧（自愈），
## 超过 _TIMEOUT_MS 无新分片即丢弃半帧。

signal frame_received(img: Image)

const MAGIC0 := 0x56
const MAGIC1 := 0x44
const _HEADER := 14
const _CHUNK := 1400
const _TIMEOUT_MS := 600

var _peer: PacketPeerUDP = null
var _local_port := -1

var _frame_id := -1
var _total := 0
var _count := 0
var _chunks: Dictionary = {}
var _last_ms := 0

func is_active() -> bool:
	return _peer != null

func get_port() -> int:
	return _local_port

## 绑定随机本地 UDP 端口，供 stream 指令上报给板子。成功返回 true。
func start() -> bool:
	stop()
	_peer = PacketPeerUDP.new()
	if _peer.bind(0) != OK:
		_peer = null
		return false
	_local_port = _peer.get_local_port()
	set_process(true)
	return true

func stop() -> void:
	if _peer != null:
		_peer.close()
		_peer = null
	_local_port = -1
	_reset()
	set_process(false)

func _reset() -> void:
	_frame_id = -1
	_total = 0
	_count = 0
	_chunks.clear()
	_last_ms = 0

func _process(_delta: float) -> void:
	if _peer == null:
		return
	var peer: PacketPeerUDP = _peer
	while peer.get_available_packet_count() > 0:
		_consume(peer.get_packet())
	# 半帧卡死（剩余分片丢失/迟到）：丢弃，等下一帧序号从头重组
	if _frame_id != -1 and Time.get_ticks_msec() - _last_ms > _TIMEOUT_MS:
		_reset()

func _consume(pkt: PackedByteArray) -> void:
	if pkt.size() < _HEADER or pkt[0] != MAGIC0 or pkt[1] != MAGIC1:
		return
	var fid: int = _read_u32(pkt, 2)
	var seq: int = _read_u16(pkt, 6)
	var count: int = _read_u16(pkt, 8)
	var total: int = _read_u32(pkt, 10)
	if count <= 0 or seq >= count or total <= 0:
		return
	if fid != _frame_id:
		_reset()  # 新帧：丢弃未组完的旧帧
		_frame_id = fid
		_total = total
		_count = count
	if _total != total or _count != count:
		return  # 头字段不一致，丢弃
	if _chunks.has(seq):
		return  # 该片已收（UDP 重复）
	_chunks[seq] = pkt.slice(_HEADER)
	_last_ms = Time.get_ticks_msec()
	if _chunks.size() == _count:
		_assemble()

func _assemble() -> void:
	var img_bytes := PackedByteArray()
	for i in _count:
		var chunk: PackedByteArray = _chunks.get(i, PackedByteArray())
		img_bytes.append_array(chunk)
	if img_bytes.size() > _total:
		img_bytes = img_bytes.slice(0, _total)
	else:
		img_bytes.resize(_total)  # 保证与声明的 total 长度一致（缺片理论上不会走到这）
	_reset()
	var img := Image.new()
	if img.load_jpg_from_buffer(img_bytes) == OK and not img.is_empty():
		frame_received.emit(img)

func _read_u16(b: PackedByteArray, off: int) -> int:
	return (b[off] << 8) | b[off + 1]

func _read_u32(b: PackedByteArray, off: int) -> int:
	return (b[off] << 24) | (b[off + 1] << 16) | (b[off + 2] << 8) | b[off + 3]