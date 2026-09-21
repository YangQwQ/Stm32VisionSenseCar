extends RefCounted
## /指令 解析器：把聊天输入文本解析成「词表指令」或「本地动作」。
## 纯解析、不碰 UI/网络，副作用由调用方（ChatPanel）统一执行。
## 返回：
##   {"ok": false, "hint": "..."}                     用法/未知指令提示
##   {"ok": true, "kind": "cmd", "cmd": {...},
##    "interrupts_ai": bool, "ai_mode": ""|"goal"|"oneshot"|"cancel"}
##   {"ok": true, "kind": "local",
##    "local": "stream"|"grid"|"snapshot"|"append"|"help"|"clear"|"ws",
##    "on": bool, "arg": "", "ip": ""}

const CP := preload("res://net/proto/CommandProto.gd")

## 转向舵三档参数（与 Main 摇杆映射一致）。
const _SERVO_CENTER := 150  # 中位
const _SERVO_RANGE := 30    # 单侧偏转量

static func parse(text: String) -> Dictionary:
	var pieces := text.split(" ", true, 1)  # 最多拆一次，保住剩余文本原样
	var verb: String = pieces[0].to_lower()
	match verb:
		"/ping":
			return _cmd(CP.ping(_arg(pieces)))
		"/help", "/h", "?":
			return _local("help")
		"/clear":
			return _local("clear")
		"/stream":
			return _local("stream", _off_unless(_arg(pieces)))
		"/grid":
			return _local("grid", _off_unless(_arg(pieces)))
		"/snapshot":
			return _local("snapshot")
		"/append", "/append_image", "/append image":
			return _local("append")
		"/exec_log":
			return _log_parse(_arg(pieces), "exec")
		"/ai_log":
			return _log_parse(_arg(pieces), "ai")
		"/log":
			return _log_parse(_arg(pieces), "")
		"/nz_read":
			return _cmd(CP.nz_read())
		"/reboot":  # 远程重启板子：卡死（WiFi/IP 死、BLE 还活着）时的唯一解药
			return _cmd(CP.reboot(), true)
		"/light":
			return _light(pieces)
		"/stop":
			var scope := "all"
			var sa := _arg(pieces).to_lower()
			if sa in ["wheels", "arm"]:
				scope = sa
			return _cmd(CP.stop(scope), true)  # 手动接管类：板端打断 AI，调用方复位「中止」
		"/reset":  # 机械臂+转向回正（直驱）
			return _cmd(CP.reset(), true)
		"/config":
			var kv := _arg(pieces).split(" ", true, 1)
			if kv.size() < 2 or kv[0].is_empty():
				return _hint("用法: /config <WiFi名> <密码>")
			return _cmd(CP.config_wifi(kv[0], kv[1]))
		"/ai":
			return _ai_parse(text)
		"/goal":  # 兼容旧写法，等同 /ai goal
			var gmsg := _arg(pieces)
			if gmsg.is_empty():
				return _hint("用法: /goal <目标文本>")
			return _cmd(CP.ai_goal(gmsg), false, "goal")
		"/cancel", "/stopai":  # 兼容旧写法，等同 /ai cancel
			return _cmd(CP.ai_cancel(), true)
		"/ws":
			var rest := pieces[1] if pieces.size() > 1 else "status"
			var wp := rest.split(" ", true, 1)
			var wv := wp[0].strip_edges().to_lower()
			var ip := wp[1].strip_edges() if wp.size() > 1 else ""
			return _local("ws", false, wv, ip)
		"/connect":  # 不经蓝牙直连 WS：等同 /ws connect <IP>
			return _local("ws", false, "connect", _arg(pieces))
		"/move":
			return _move_parse(_arg(pieces))
		"/drive":
			return _drive_parse(_arg(pieces))
		_:
			return _hint("未知指令: %s(/help 查看可用指令)" % verb)

## /move 移动控制族：rotate 转向舵 / spin 原地旋转 / fore|back 定距 / arm 机械臂位姿。
## 全部为手动接管类（打断 AI）。
static func _move_parse(args: String) -> Dictionary:
	var mv := args.split(" ", false) if args != "" else PackedStringArray()
	var sub := mv[0].strip_edges().to_lower() if mv.size() > 0 else ""
	match sub:
		"rotate":
			# /move rotate <dir=-1/0/1>：转向舵三档（左/回正/右）。
			if mv.size() < 2 or not mv[1].is_valid_int():
				return _hint("用法: /move rotate <dir=-1/0/1>")
			var dir := mv[1].to_int()
			if dir < -1 or dir > 1:
				return _hint("用法: /move rotate <dir=-1/0/1>")
			return _cmd(CP.servo(0, _SERVO_CENTER + dir * _SERVO_RANGE), true)
		"spin":
			# /move spin <角度> [speed 0..1000]：正=右转、负=左转；0 停。
			if mv.size() < 2 or not mv[1].is_valid_float():
				return _hint("用法: /move spin <角度> [speed 0..1000]")
			var ang := mv[1].to_float()
			if absf(ang) < 1.0:
				return _cmd(CP.spin(0), true)
			var spd := 500
			if mv.size() > 2 and mv[2].is_valid_int():
				spd = clampi(mv[2].to_int(), 0, 1000)
			var dir := 1 if ang > 0 else -1
			var deg := clampi(int(round(absf(ang))), 0, 500)
			return _cmd(CP.spin(dir, spd, deg), true)
		"fore", "back":
			# /move fore|back <距离cm 1..500> [油门% -100..100]：定距移动（板端时长近似）。
			if mv.size() < 2 or not mv[1].is_valid_int():
				return _hint("用法: /move fore|back <距离cm 1..500> [油门% -100..100]")
			var cm := mv[1].to_int()
			if cm < 1 or cm > 500:
				return _hint("用法: /move fore|back <距离cm 1..500> [油门% -100..100]")
			var thr := 0.5  # 默认半速
			if mv.size() > 2:
				if not mv[2].is_valid_float():
					return _hint("用法: /move fore|back <距离cm 1..500> [油门% -100..100]")
				thr = absf(mv[2].to_float()) / 100.0
				if thr > 1.0:
					return _hint("用法: /move fore|back <距离cm 1..500> [油门% -100..100]")
			if sub == "back":
				thr = -thr
			return _cmd(CP.move_dist(thr, cm), true)
		"arm":
			# /move arm <x> <h> 位姿；/move arm reset 机械臂+转向回正；/move arm fold 收臂折叠；
			# /move arm low 降到贴地夹取准备位。
			if mv.size() == 2:
				match mv[1].strip_edges().to_lower():
					"reset":
						return _cmd(CP.reset(), true)
					"fold":
						return _cmd(CP.arm("fold"), true)
					"low":
						# 板端标定过的贴地夹取准备位（固定低姿）：先降到位，再靠前后挪车把目标
						# 送进两指之间。手动验证该准备位/复现 AI 的贴地夹取流程时用。
						return _cmd(CP.arm("low"), true)
				return _hint("用法: /move arm <x> <h> | reset | fold | low")
			if mv.size() < 3 or not mv[1].is_valid_float() or not mv[2].is_valid_float():
				return _hint("用法: /move arm <x=车头系前方cm> <h=离地高度cm> | reset | fold | low")
			# 校准用：不限制数值范围（可为负/超界），不可达由板端可达域检查拦截。
			return _cmd(CP.arm_pose(mv[1].to_float(), mv[2].to_float()), true)
		"to":
			# /move to <x> <y> [global|local]：板端本地导航到坐标（手动接管，打断 AI）。
			# local 原点=当前位姿（y前正 x右正）；global=沿用全局系。
			if mv.size() < 3 or not mv[1].is_valid_float() or not mv[2].is_valid_float():
				return _hint("用法: /move to <x> <y> [global|local]")
			var fx := mv[1].to_float()
			var fy := mv[2].to_float()
			var frame := "local"
			if mv.size() > 3:
				frame = mv[3].strip_edges().to_lower()
				if frame != "local" and frame != "global":
					return _hint("用法: /move to <x> <y> [global|local]")
			return _cmd(CP.goto(fx, fy, frame), true)
		_:
			return _hint("用法: /move rotate|spin|fore|back|to|arm <参数>")

## /drive 直驱控制族：motor 电机 / servo 舵机。
static func _drive_parse(args: String) -> Dictionary:
	var dv := args.split(" ", false) if args != "" else PackedStringArray()
	var sub := dv[0].strip_edges().to_lower() if dv.size() > 0 else ""
	match sub:
		"motor":
			# /drive motor <n=1..4|0=all> <pwm -1000..1000>：单轮电机；n=0 全车 drive。
			if dv.size() < 3 or not dv[1].is_valid_int() or not dv[2].is_valid_int():
				return _hint("用法: /drive motor <n=1..4|0=all> <pwm -1000..1000>")
			var m_n := dv[1].to_int()
			var m_pwm := dv[2].to_int()
			if m_pwm < -1000 or m_pwm > 1000:
				return _hint("用法: /drive motor <n=1..4|0=all> <pwm -1000..1000>")
			if m_n == 0:
				return _cmd(CP.drive(m_pwm), true)
			if m_n < 1 or m_n > 4:
				return _hint("用法: /drive motor <n=1..4|0=all> <pwm -1000..1000>")
			return _cmd(CP.motor(m_n, maxi(m_pwm, 0), maxi(-m_pwm, 0)), true)
		"servo":
			# /drive servo <n=0转向/1左/2右/3前> <pwm=50..250>：直驱舵机（可超标定限位）。
			if dv.size() < 3 or not dv[1].is_valid_int() or not dv[2].is_valid_int():
				return _hint("用法: /drive servo <n=0转向/1左/2右/3前> <pwm=50..250>")
			var s_n := dv[1].to_int()
			var s_pwm := dv[2].to_int()
			if s_n < 0 or s_n > 3 or s_pwm < 50 or s_pwm > 250:
				return _hint("用法: /drive servo <n=0转向/1左/2右/3前> <pwm=50..250>")
			return _cmd(CP.servo(s_n, s_pwm), true)
		_:
			return _hint("用法: /drive motor|servo <参数>")

## /ai 统一入口：/ai goal <目标>（迭代闭环）/ /ai oneshot <目标>（单轮）/ /ai cancel（取消）。
static func _ai_parse(text: String) -> Dictionary:
	var ap := text.split(" ", true, 2)
	var mode := ap[1].strip_edges().to_lower() if ap.size() > 1 else ""
	var msg := ap[2].strip_edges() if ap.size() > 2 else ""
	match mode:
		"cancel":
			return _cmd(CP.ai_cancel(), false, "cancel")
		"oneshot":
			if msg.is_empty():
				return _hint("用法: /ai oneshot <目标文本>")
			return _cmd(CP.ai_oneshot(msg), false, "oneshot")
		_:
			if msg.is_empty():
				return _hint("用法: /ai goal <目标文本>")
			return _cmd(CP.ai_goal(msg), false, "goal")

## /light <kind=front|vibe|back> <0|1>：单参按 front 处理，两参拆分 kind 与开关。
static func _light(pieces: Array) -> Dictionary:
	var rest := _arg(pieces)
	var args := rest.split(" ", false) if rest != "" else PackedStringArray()
	var kind := "front"
	var on := false
	if args.size() >= 2:
		kind = args[0].to_lower()
		on = args[1].to_lower() in ["on", "1", "true"]
	elif args.size() == 1:
		on = args[0].to_lower() in ["on", "1", "true"]
	return _cmd(CP.light(kind, on))

## 取首个参数（空格后的剩余文本，去首尾空白）。
static func _arg(pieces: Array) -> String:
	return (pieces[1] as String).strip_edges() if pieces.size() > 1 else ""

## /log 统一日志开关：/log <exec|ai|all> [on|off]；缺省 on。旧 /exec_log /ai_log 映射到对应类别。
## 板端三个开关是**单选**（开某类别即收窄到该类、开 all 即全部），回执会带「当前: …」供确认。
static func _log_parse(args: String, forced_cat: String) -> Dictionary:
	var cat := forced_cat
	var on := true
	if forced_cat != "":
		# 别名 /exec_log|/ai_log：剩参仅开关词（on/off/空）。
		on = _on_explicit(args)
	else:
		var ap := args.split(" ", false) if args != "" else PackedStringArray()
		if ap.is_empty() or ap[0].is_empty() or ap[0].to_lower() not in ["exec", "ai", "all"]:
			return _hint("用法: /log <exec|ai|all> [on|off]")
		cat = ap[0].to_lower()
		if ap.size() > 1:
			var av := ap[1].strip_edges().to_lower()
			if av in ["off", "0", "false"]:
				on = false
			elif av not in ["on", "1", "true"]:
				return _hint("用法: /log <exec|ai|all> [on|off]")
	return _cmd(CP.log_switch(cat, on))

## /stream /grid 语义：缺省开；参数为 off/0/false 才关。
static func _off_unless(rest: String) -> bool:
	var a := rest.to_lower()
	return a != "off" and a != "0" and a != "false"

## /log 语义：显式 on/1/true 才开；无参数默认开。
static func _on_explicit(rest: String) -> bool:
	var a := rest.to_lower()
	return a.is_empty() or a in ["on", "1", "true"]

static func _cmd(cmd: Dictionary, interrupts_ai: bool = false, ai_mode: String = "") -> Dictionary:
	return {"ok": true, "kind": "cmd", "cmd": cmd,
		"interrupts_ai": interrupts_ai, "ai_mode": ai_mode}

static func _local(local_name: String, on: bool = false, arg: String = "", ip: String = "") -> Dictionary:
	return {"ok": true, "kind": "local", "local": local_name,
		"on": on, "arg": arg, "ip": ip}

static func _hint(text: String) -> Dictionary:
	return {"ok": false, "hint": text}
