extends Node
## UDP 图传接收端：用 Godot 内置 PacketPeerUDP 收板子推送的 JPEG 分片，重组整帧后发 frame_received。
## 分片协议与板子 app_httpd.cpp 的 udp_send_frame 对齐：每数据报 14B 大端头 +
## [magic(2) frame_id(4) seq(2) count(2) total(4)] + ≤1400B JPEG 负载。
## 重组策略：按 frame_id 缓存分片，count 片齐则整帧字节交给解码线程；缺片/丢包等下一帧（自愈），
## 超过 _TIMEOUT_MS 无新分片即丢弃半帧。
## 解码不在主线程做：VGA JPEG 解码（十几 ms 量级）与渲染、WS 心跳应答同在主线程时，每拍被解码顶住
## → 渲染掉帧、心跳应答延后（板端 idle 探测会判死重连），且内核缓冲里的整帧越积越多、下一拍连播出来
## （画面跑得比真实快）。故解码交给 WorkerThreadPool，主线程每拍只做「取包→拼帧→取结果→上抛」；
## 在途解码期间的整帧直接覆盖丢弃 —— 画面只追最新，显示顺序天然单调（不会把旧帧翻出来重播）。

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

# 解码线程交接：_pending / _task_id 只在主线程读写；worker 只写 _result，经 _mutex 交给主线程。
var _mutex := Mutex.new()
var _pending := PackedByteArray()  # 待解码的最新整帧（worker 取走前可被新来的整帧覆盖）
var _result: Image = null          # worker 解出的画面，主线程取走后置空
var _task_id := -1                 # 在途解码任务 id（-1 = 空闲）

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
	_drop_task()
	_reset()
	set_process(false)

## 停流/重开时收掉在途解码任务：等它跑完再清，不留未回收的任务引用。
func _drop_task() -> void:
	if _task_id != -1:
		WorkerThreadPool.wait_for_task_completion(_task_id)
		_task_id = -1
	_pending = PackedByteArray()
	_mutex.lock()
	_result = null
	_mutex.unlock()

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
	# 取包与拼帧都便宜（分片拷贝），故每拍把内核缓冲排空：积压越滚越大就是延迟越滚越大，
	# 且缓冲满了直接丢包。开销大的解码已交给 worker，排空不再拖长本拍。
	while peer.get_available_packet_count() > 0:
		_consume(peer.get_packet())
	# 半帧卡死（剩余分片丢失/迟到）：丢弃，等下一帧序号从头重组
	if _frame_id != -1 and Time.get_ticks_msec() - _last_ms > _TIMEOUT_MS:
		_reset()
	if _task_id != -1 and WorkerThreadPool.is_task_completed(_task_id):
		WorkerThreadPool.wait_for_task_completion(_task_id)
		_task_id = -1
	var img: Image = null
	_mutex.lock()
	if _result != null:
		img = _result
		_result = null
	_mutex.unlock()
	if img != null:
		frame_received.emit(img)
	# 只在无在途任务时起新解码，且只喂最新整帧：期间的整帧已被覆盖丢弃 —— 画面只追最新，
	# 显示顺序单调（不会把旧帧翻出来重播成"快放"）。
	if _task_id == -1:
		var bytes := _pending
		_pending = PackedByteArray()
		if bytes.size() > 0:
			_task_id = WorkerThreadPool.add_task(_decode.bind(bytes))

## 解码线程体（WorkerThreadPool）：只碰自己 new 的 Image，结果经 _mutex 交主线程。
func _decode(bytes: PackedByteArray) -> void:
	var img := Image.new()
	if img.load_jpg_from_buffer(bytes) != OK or img.is_empty():
		return
	_mutex.lock()
	_result = img
	_mutex.unlock()

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
	_pending = img_bytes  # 覆盖写：在途解码期间来的新整帧直接顶掉旧的（只追最新）

func _read_u16(b: PackedByteArray, off: int) -> int:
	return (b[off] << 8) | b[off + 1]

func _read_u32(b: PackedByteArray, off: int) -> int:
	return (b[off] << 24) | (b[off + 1] << 16) | (b[off + 2] << 8) | b[off + 3]