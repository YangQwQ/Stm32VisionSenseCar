extends Node
## 会话录制与回放（autoload `Recorder`）。
##
## 录制：常驻。AI 任务开始到用户 /clear 之间的事件按**逻辑时钟**记录 —— 时钟只在 AI 运行中前进，
## 于是"AI 暂停 + 用户插话"在回放里是一瞬间（等待期间自动被抹掉）。/clear 把这一趟归档成一条会话。
##
## 落盘（依次追加，不在内存里堆整趟）：
##   user://sessions/index.json          # [{id,title,created_ms,dur_ms,n_events,n_frames}] 新→旧, ≤9
##   user://sessions/<id>/meta.json
##   user://sessions/<id>/events.jsonl   # 每行 {t, k, ...}；k=chat|task|frame|track
##   user://sessions/<id>/frames/<seq>.jpg
## 当前趟先写 user://sessions/_cur/，归档时整体改名到 <id>/。
##
## 回放：只读。按逻辑时间轴推进，依次抛 replay_event / replay_frame；
## 期间 is_playing() 为真，DeviceConn 据此拒绝一切下发。

## 会话列表变化（归档/删除后）——SessionList 据此重绘。
signal sessions_changed
## 回放到某条事件（ChatPanel 据此续写聊天区 / 切任务面板）。
signal replay_event(ev: Dictionary)
## 回放被拖到新位置：调用方先清场（清聊天区/任务面板），随后会重新抛该点之前的事件。
signal replay_seeked
## 回放到某帧（已解码；Main 交给 VideoView）。
signal replay_frame(img: Image)
## 回放进度（t_ms / duration_ms）。
signal replay_progress(t_ms: int, duration_ms: int)
## 回放结束（播完或手动停止）。
signal replay_finished

const _DIR := "user://sessions"
const _CUR := "user://sessions/_cur"
const _MAX_SESSIONS := 9   # 归档条数上限（面板 Session1 = 当前会话，2..10 共 9 行给归档）
const _FRAME_MIN_GAP_MS := 100   # 录制帧率上限（≈10fps），与"隔帧存一帧"叠加
const _TRACK_MIN_GAP_MS := 100   # 跟踪点录制上限（板端 10Hz 上报，同频不丢动作）
const _ID_CUR := -1

# ============================== 录制状态 ==============================

var _ai_running := false
var _lt_ms := 0            # 逻辑时钟（只在 AI 运行中前进）
var _saw_ai := false       # 这一趟是否出现过 AI 任务（空的趟不归档）
var _title := ""
var _started_ms := 0
var _n_events := 0
var _n_frames := 0
var _cur_frames := 0
var _frame_parity := 0     # 隔帧计数
var _last_frame_lt := -100000
var _last_track_lt := -100000
var _track_novid := false   # 上次录下的跟踪期状态（用于识别 novid 翻转，不被限频丢掉）

# ============================== 回放状态 ==============================

var _playing := false
var _paused := false
var _pt_ms := 0
var _pt_f := 0.0           # 毫秒累加用浮点（整数逐帧截断会让长回放越走越慢）
var _dur_ms := 0
var _events: Array = []    # [{t, k, ...}] 按 t 升序
var _ev_idx := 0           # 下一个待抛事件
var _dec_idx := -1         # 当前已抛到的「AI 决策点」事件索引；-1=尚未落点(起点)。上一步/下一步以它为锚。
var _session_id := _ID_CUR # 正在回放的会话 id（删除它时要先停下来）
var _rf_dir := ""

# 回放帧解码（worker，单任务在途 + Mutex，镜像 UDPVideoClient 的做法）
var _dec_mutex := Mutex.new()
var _dec_result: Image = null
var _dec_task := -1

# 录制帧写盘（worker）：登记在途任务 id，归档/重置目录前必须等它们收尾。
var _write_mutex := Mutex.new()
var _write_ids: Array = []

func _ready() -> void:
	DirAccess.make_dir_recursive_absolute(_DIR)
	_reset_current()

func _process(delta: float) -> void:
	# 逻辑时钟：只在 AI 运行中前进（回放期间冻结）——空闲等待在回放里被抹掉。
	if _ai_running and not _playing:
		_lt_ms += int(delta * 1000.0)
	if _playing:
		_tick_playback(delta)
		_drain_decode()
	_prune_writes()

# ============================== 录制（ChatPanel 调用） ==============================

## AI 运行态（唯一入口：ChatPanel.set_ai_running）。只在运行中推进逻辑时钟。
func set_ai_running(run: bool) -> void:
	if run and not _ai_running:
		_saw_ai = true
	_ai_running = run

## 聊天区一条消息。回放中不记（防回灌）。
func record_chat(who: String, msg: String) -> void:
	if _playing:
		return
	_append({"t": _lt_ms, "k": "chat", "who": who, "msg": msg})

## AI 任务面板快照。
func record_task(params: Variant) -> void:
	if _playing or not (params is Dictionary):
		return
	_append({"t": _lt_ms, "k": "task", "params": params})

## 图传帧（原始 JPEG）：只在 AI 运行中记录；隔帧存一帧 + 帧率上限。
func record_frame(jpeg: PackedByteArray) -> void:
	if _playing or not _ai_running or jpeg.is_empty():
		return
	_frame_parity += 1
	if _frame_parity % 2 == 1:
		return  # 「保存一帧扔一帧」
	if _lt_ms - _last_frame_lt < _FRAME_MIN_GAP_MS:
		return
	_last_frame_lt = _lt_ms
	_cur_frames += 1
	var name := "%06d.jpg" % _cur_frames
	if _append({"t": _lt_ms, "k": "frame", "f": name}):
		_n_frames += 1
		_write_file_async(_CUR + "/frames/" + name, jpeg)

## 跟踪叠加层（板端 track 上行）：夹取/跟踪期板端**不推视频**，画面全靠这层
## "标尺网格 + 跟踪点"。不录它 ⇒ 回放时那段既没帧也没状态，画面冻在夹取前一张。
## 键名与板端 track 消息保持一致（u/v/conf/st/novid），回放端同一段代码两边通用。
func record_track(u: float, v: float, conf: float, st: String, novid: bool) -> void:
	var flip := novid != _track_novid
	_track_novid = novid
	# novid 翻转（跟踪开始/结束）必须落盘：它决定回放是显示跟踪画面还是真实画面，丢了就卡死在另一种。
	# 收尾那条（flip 到不 novid）即便已退出录制态也要录：中止/掉线时 AI 先停、板端后补 idle，
	# 漏掉它回放会永远停在跟踪画面。逻辑时钟已冻结 ⇒ t 就是最后一刻，正好接在夹取段末尾。
	if not _ai_running and not flip:
		return
	if not flip and _lt_ms - _last_track_lt < _TRACK_MIN_GAP_MS:
		return
	_last_track_lt = _lt_ms
	var ev := {"t": _lt_ms, "k": "track", "st": st, "novid": novid}
	if st != "idle":
		ev["u"] = u
		ev["v"] = v
		ev["conf"] = conf
	_append(ev)

## 归档当前趟（/clear 调用）。没跑过 AI 则整趟丢弃。
func archive_current() -> void:
	if _playing:
		return
	if not _saw_ai or _n_events == 0:
		_reset_current()
		return
	# 时间戳毫秒撞车(同一毫秒归档两次)时让 id 顺延，避免目录冲突。
	var id := int(Time.get_unix_time_from_system() * 1000.0)
	while _dir_exists("%s/%d" % [_DIR, id]):
		id += 1
	_finalize_meta(_CUR)
	var dst := "%s/%d" % [_DIR, id]
	# 写盘的帧任务必须先收尾：目录改名/删除时还有句柄开着，rename 会失败，
	# 兜底拷贝就可能拷到半截文件。
	_drain_writes()
	_move_dir(_CUR, dst)
	if not _dir_exists(dst):
		# 归档失败（目录被占/无权限）：保留当前趟，不清空 —— 免得录制白丢。
		push_warning("[Recorder] 会话归档失败, 已保留当前录制: %s" % dst)
		return
	_append_index({
		"id": id,
		"title": _title if not _title.is_empty() else "会话",
		"created_ms": _started_ms,
		"dur_ms": _lt_ms,
		"n_events": _n_events,
		"n_frames": _n_frames,
	})
	_prune_index()
	_reset_current()
	sessions_changed.emit()

## 当前趟是否值得保存：跑过 AI 任务且记录到事件才算（与 archive_current 的归档门槛一致）。
## 没跑过任务/空趟调用 save 时应弹窗提醒，而不是静默丢弃。
func has_session_to_save() -> bool:
	return _saw_ai and _n_events > 0

# ============================== 会话列表 ==============================

## 归档会话（新→旧）。
func list_sessions() -> Array:
	return _read_index()

func delete(id: int) -> void:
	if id == _session_id and _playing:
		_stop_internal(true)   # 正在回放的就是它：先停，别让游标指向已删目录
	var idx := _read_index()
	var out: Array = []
	for it in idx:
		if int((it as Dictionary).get("id", 0)) == id:
			continue
		out.append(it)
	_write_index(out)
	_remove_dir("%s/%d" % [_DIR, id])
	sessions_changed.emit()

# ============================== 回放 ==============================

func is_playing() -> bool:
	return _playing

func play(id: int) -> bool:
	if _playing:
		_stop_internal(false)   # 切换会话：不抛 finished（马上要开新的一段）
	var dir := _CUR if id == _ID_CUR else "%s/%d" % [_DIR, id]
	_events = _read_events(dir)
	if _events.is_empty():
		return false
	_dur_ms = 0
	for e in _events:
		_dur_ms = maxi(_dur_ms, int((e as Dictionary).get("t", 0)))
	_session_id = id
	_rf_dir = dir + "/frames/"
	_ev_idx = 0
	_dec_idx = -1
	_pt_ms = 0
	_pt_f = 0.0
	_paused = false
	_playing = true
	replay_progress.emit(0, _dur_ms)
	return true

func stop_play() -> void:
	if not _playing:
		return
	_stop_internal(true)

func toggle_pause() -> void:
	if _playing:
		_paused = not _paused

func is_paused() -> bool:
	return _paused

## 单步：dir=+1 跳到下一个「AI 决策点」（AI工具/AI 的发言），-1 回到上一个。
## 以决策点为锚而不是逐条日志跳：日志/状态行只是伴随输出，逐条跳很抽象。
## 锚点用 _dec_idx（已抛到的决策点索引）记录；前进从它之后搜下一个，回退在它之前找上一个。
## ⚠️ 按事件推进而不是按时间：空闲期的时间戳全相同，按时间回退会一步跳回原地。
func step(dir: int) -> void:
	if not _playing or _events.is_empty():
		return
	_paused = true
	if dir > 0:
		# 从当前决策点之后找下一个决策点；找不到就落到末尾（收尾态面板/日志照样抛完）。
		var j := _dec_idx + 1
		while j < _events.size() and not _is_decision(j):
			j += 1
		if j >= _events.size():
			j = _events.size() - 1   # 没有更多决策点：把剩余事件全部抛完并停在末条
		_drop_decode()   # 丢掉在途旧帧（含上一帧解码）：重建后只解"最新一帧"，否则会残留跳走前的画面
		_emit_index_upto(j + 1)
		_seek_to_ms(int((_events[j] as Dictionary).get("t", 0)))
	else:
		# 回退：清场后重建到「当前决策点之前」最近的一个决策点；开头之前没有就回第 0 条。
		if _dec_idx <= 0:
			_drop_decode()   # 丢掉在途的旧帧解码：时间轴跳走后不该再冒出跳走前的画面
			_seek_to_ms(0)
			replay_seeked.emit()
			_ev_idx = 0
			_dec_idx = -1
			replay_progress.emit(_pt_ms, _dur_ms)
			return
		var i := _dec_idx - 1
		while i > 0 and not _is_decision(i):
			i -= 1
		if not _is_decision(i) and i > 0:
			i = 0   # 之前没有决策点：回退到开头
		_drop_decode()   # 同上：时间轴跳走后旧帧不该再上屏
		_seek_to_ms(int((_events[i] as Dictionary).get("t", 0)))
		replay_seeked.emit()
		_ev_idx = 0
		_dec_idx = -1   # 先清锚，让 _emit_index_upto 按"已抛决策点"重新建立（到开头时即 -1）
		_emit_index_upto(i + 1)   # 抛完即把 _dec_idx 落回 i（i 本身是决策点）
	replay_progress.emit(_pt_ms, _dur_ms)

## 事件 idx 是否是「AI 决策点」（AI 的工具调用或发言）——上一步/下一步以它为锚。
func _is_decision(i: int) -> bool:
	var ev := _events[i] as Dictionary
	if str(ev.get("k", "")) != "chat":
		return false
	var w := str(ev.get("who", ""))
	return w == "AI工具" or w == "AI"

## 拖动进度条：ratio 0..1。先清场再重建到该点之前的所有事件。
func seek_ratio(ratio: float) -> void:
	if not _playing:
		return
	_paused = true
	_drop_decode()   # 同上：拖动后旧帧不该再上屏
	_seek_to_ms(clampi(int(ratio * float(_dur_ms)), 0, _dur_ms))
	replay_seeked.emit()
	_ev_idx = 0
	_emit_pending()
	replay_progress.emit(_pt_ms, _dur_ms)

# ============================== 内部：录制落盘 ==============================

func _reset_current() -> void:
	_drain_writes()   # 目录要删了，先等在途写盘收尾
	_remove_dir(_CUR)
	for d: String in ["", "/frames"]:
		DirAccess.make_dir_recursive_absolute(_CUR + d)
	# 先把事件文件建出来：READ_WRITE 追加要求文件已存在（否则 open 直接失败、事件被静默丢掉）。
	var f := FileAccess.open(_CUR + "/events.jsonl", FileAccess.WRITE)
	if f != null:
		f.close()
	_ai_running = false
	_lt_ms = 0
	_saw_ai = false
	_title = ""
	_started_ms = int(Time.get_unix_time_from_system() * 1000.0)
	_n_events = 0
	_n_frames = 0
	_cur_frames = 0
	_frame_parity = 0
	_last_frame_lt = -100000
	_last_track_lt = -100000
	_track_novid = false

## 追加一条事件。返回是否真的写进去了（写失败不计数，免得 meta 与内容对不上）。
func _append(ev: Dictionary) -> bool:
	if _title.is_empty() and str(ev.get("k", "")) == "chat" and str(ev.get("who", "")) == "本机":
		var m := str(ev.get("msg", ""))
		if not m.is_empty() and not m.begins_with("/"):
			_title = m.substr(0, 20)   # 用第一句 AI 目标当会话名（提示/连接文案不当名）
	var f := FileAccess.open(_CUR + "/events.jsonl", FileAccess.READ_WRITE)
	if f == null:
		f = FileAccess.open(_CUR + "/events.jsonl", FileAccess.WRITE)  # 兜底：文件缺失时新建
	if f == null:
		return false
	f.seek_end()
	f.store_line(JSON.stringify(ev))
	f.close()
	_n_events += 1
	return true

func _finalize_meta(dir: String) -> void:
	var meta := {
		"title": _title if not _title.is_empty() else "会话",
		"created_ms": _started_ms,
		"dur_ms": _lt_ms,
		"n_events": _n_events,
		"n_frames": _n_frames,
	}
	var f := FileAccess.open(dir + "/meta.json", FileAccess.WRITE)
	if f != null:
		f.store_string(JSON.stringify(meta))
		f.close()

func _read_index() -> Array:
	var f := FileAccess.open(_DIR + "/index.json", FileAccess.READ)
	if f == null:
		return []
	var parsed: Variant = JSON.parse_string(f.get_as_text())
	f.close()
	if parsed is Array:
		return parsed as Array
	return []

## 原子写：先写临时文件再改名，避免写一半崩掉把整个列表读成空。
func _write_index(arr: Array) -> void:
	var tmp := _DIR + "/index.json.tmp"
	var f := FileAccess.open(tmp, FileAccess.WRITE)
	if f == null:
		return
	f.store_string(JSON.stringify(arr))
	f.close()
	if DirAccess.rename_absolute(tmp, _DIR + "/index.json") != OK:
		DirAccess.remove_absolute(tmp)

func _append_index(entry: Dictionary) -> void:
	var idx := _read_index()
	idx.push_front(entry)   # 新→旧
	_write_index(idx)

func _prune_index() -> void:
	var idx := _read_index()
	while idx.size() > _MAX_SESSIONS:
		var old: Variant = idx.pop_back()
		if old is Dictionary:
			_remove_dir("%s/%d" % [_DIR, int((old as Dictionary).get("id", 0))])
	_write_index(idx)

func _read_events(dir: String) -> Array:
	var f := FileAccess.open(dir + "/events.jsonl", FileAccess.READ)
	if f == null:
		return []
	var out: Array = []
	while not f.eof_reached():
		var line := f.get_line().strip_edges()
		if line.is_empty():
			continue
		var parsed: Variant = JSON.parse_string(line)
		if parsed is Dictionary:
			out.append(parsed)
	f.close()
	return out

# ============================== 内部：回放推进 ==============================

## 停止回放（notify=false 用于"立刻切到另一段"，不抛 finished 免得调用方白清一次场）。
func _stop_internal(notify: bool) -> void:
	_playing = false
	_paused = false
	_drop_decode()
	_events = []
	_rf_dir = ""
	if notify:
		replay_finished.emit()

func _seek_to_ms(t: int) -> void:
	_pt_ms = t
	_pt_f = float(t)

func _tick_playback(delta: float) -> void:
	if _paused:
		return
	_pt_f += delta * 1000.0
	_pt_ms = int(_pt_f)
	_emit_pending()
	replay_progress.emit(mini(_pt_ms, _dur_ms), _dur_ms)
	if _pt_ms >= _dur_ms and _ev_idx >= _events.size():
		# 播完即退出回放（只读态一并解除）：否则 is_playing() 永远为真，指令再也发不出去。
		_stop_internal(true)

## 抛出一条事件（frame 交给解码，其余直接上抛）。
## 抛出一条非帧事件（帧的解码在调用方合并到"该范围内最新一帧"后统一起，避免回退时跳回首帧）。
func _emit_one(ev: Dictionary) -> void:
	replay_event.emit(ev)

## 抛出 t <= 当前游标时间且尚未抛过的事件。帧只解码该范围内**最新一帧**（其余会被顶掉）。
func _emit_pending() -> void:
	var last_frame := ""
	while _ev_idx < _events.size():
		var ev := _events[_ev_idx] as Dictionary
		if int(ev.get("t", 0)) > _pt_ms:
			break
		if _is_decision(_ev_idx):
			_dec_idx = _ev_idx
		if str(ev.get("k", "")) == "frame":
			last_frame = _rf_dir + str(ev.get("f", ""))
		else:
			_emit_one(ev)
		_ev_idx += 1
	if not last_frame.is_empty():
		_start_decode(last_frame)

## 抛到第 n 条为止（不含 n）——按条数推进用，不受"空闲期时间戳相同"影响。帧只解码最新一帧。
func _emit_index_upto(n: int) -> void:
	var last_frame := ""
	while _ev_idx < n and _ev_idx < _events.size():
		var ev := _events[_ev_idx] as Dictionary
		if _is_decision(_ev_idx):
			_dec_idx = _ev_idx
		if str(ev.get("k", "")) == "frame":
			last_frame = _rf_dir + str(ev.get("f", ""))
		else:
			_emit_one(ev)
		_ev_idx += 1
	if not last_frame.is_empty():
		_start_decode(last_frame)

# ============================== 内部：异步写盘 / 解码 ==============================

func _write_file_async(path: String, bytes: PackedByteArray) -> void:
	var id := WorkerThreadPool.add_task(_write_file.bind(path, bytes))
	_write_mutex.lock()
	_write_ids.append(id)
	_write_mutex.unlock()

func _write_file(path: String, bytes: PackedByteArray) -> void:
	var f := FileAccess.open(path, FileAccess.WRITE)
	if f != null:
		f.store_buffer(bytes)
		f.close()

## 丢掉已完成的写盘任务 id（防长时间录制把列表堆大）。
func _prune_writes() -> void:
	if _write_ids.is_empty():
		return
	_write_mutex.lock()
	var keep: Array = []
	for id in _write_ids:
		if not WorkerThreadPool.is_task_completed(id):
			keep.append(id)
	_write_ids = keep
	_write_mutex.unlock()

## 等在途写盘任务全部收尾（归档/重置目录前必须调）。
func _drain_writes() -> void:
	_write_mutex.lock()
	var ids := _write_ids
	_write_ids = []
	_write_mutex.unlock()
	for id in ids:
		WorkerThreadPool.wait_for_task_completion(id)

## 起一次解码：读文件也放 worker 里（主线程读整帧会卡回放）。
func _start_decode(path: String) -> void:
	if _dec_task != -1:
		return  # 有在途解码：丢掉这帧（回放只追最新，与图传一致）
	_dec_task = WorkerThreadPool.add_task(_decode_path.bind(path))

func _decode_path(path: String) -> void:
	var bytes := FileAccess.get_file_as_bytes(path)
	if bytes.is_empty():
		return
	var img := Image.new()
	if img.load_jpg_from_buffer(bytes) == OK and not img.is_empty():
		_dec_mutex.lock()
		_dec_result = img
		_dec_mutex.unlock()

func _drain_decode() -> void:
	if _dec_task != -1 and WorkerThreadPool.is_task_completed(_dec_task):
		WorkerThreadPool.wait_for_task_completion(_dec_task)
		_dec_task = -1
	var img: Image = null
	_dec_mutex.lock()
	if _dec_result != null:
		img = _dec_result
		_dec_result = null
	_dec_mutex.unlock()
	if img != null:
		replay_frame.emit(img)

func _drop_decode() -> void:
	if _dec_task != -1:
		WorkerThreadPool.wait_for_task_completion(_dec_task)
		_dec_task = -1
	_dec_mutex.lock()
	_dec_result = null
	_dec_mutex.unlock()

# ============================== 内部：目录工具 ==============================

func _dir_exists(path: String) -> bool:
	return DirAccess.dir_exists_absolute(path)

func _remove_dir(path: String) -> void:
	var d := DirAccess.open(path)
	if d == null:
		return
	d.list_dir_begin()
	var name := d.get_next()
	while name != "":
		if d.current_is_dir():
			_remove_dir(path + "/" + name)
		else:
			DirAccess.remove_absolute(path + "/" + name)
		name = d.get_next()
	d.list_dir_end()
	DirAccess.remove_absolute(path)

func _move_dir(src: String, dst: String) -> void:
	# 目标不存在时整目录改名；失败则逐项拷贝兜底。
	if DirAccess.rename_absolute(src, dst) == OK:
		return
	DirAccess.make_dir_recursive_absolute(dst)
	_copy_dir(src, dst)

func _copy_dir(src: String, dst: String) -> void:
	var d := DirAccess.open(src)
	if d == null:
		return
	d.list_dir_begin()
	var name := d.get_next()
	while name != "":
		if d.current_is_dir():
			DirAccess.make_dir_recursive_absolute(dst + "/" + name)
			_copy_dir(src + "/" + name, dst + "/" + name)
		else:
			DirAccess.copy_absolute(src + "/" + name, dst + "/" + name)
		name = d.get_next()
	d.list_dir_end()
