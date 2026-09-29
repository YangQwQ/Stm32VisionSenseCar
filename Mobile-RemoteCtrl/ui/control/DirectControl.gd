extends Panel
## 手动控制面板：摇杆驱动映射 + 机械臂 / 灯光直控。
## 摇杆映射：内置 VirtualJoystick 把分量写入 4 个 vjoy_* action，本脚本据此还原方向向量、量化后下发。
## 所有下发经 DeviceConn.send_command 统一出口（WS 优先、BLE 兜底）。

const CP := preload("res://net/proto/CommandProto.gd")

## 手动接管：摇杆一动就打断板端 AI 闭环（Main 收到后把聊天发送按钮复位「发送」）。
signal manual_takeover
## 板端 AI 运行态（状态位 bit4）同步上来时通知；仅 apply_ai=true 时发。
signal ai_busy_changed(busy: bool)

# 持续型（按住动、松开停）：升降 / 移爪。
const _HOLD_ACTIONS := {
	"ClawUpBtn": "lift_up",
	"ClawDownBtn": "lift_down",
	"ClawForwardBtn": "reach_forward",
	"ClawBackwardBtn": "reach_backward",
}

# 摇杆量化档：死区内不动作；速度分 低速/高速 两档；转向固定档。仅档位变化才发指令，松手发 stop。
const _JOY_DEADZONE := 0.15
const _JOY_SLOW := 0.5
const _JOY_FAST := 1.0
const _JOY_FAST_THRESH := 0.7
const _JOY_STEER := 0.8
# 直接驱动时的摇杆映射参数。
const _DRIVE_MAX := 1000       # 油门满量程 PWM
const _SERVO_CENTER := 150     # 转向舵中位
const _SERVO_RANGE := 30       # 转向舵单侧偏转量
const _SPIN_SPEED := 800       # 原地旋转模式下左右推摇杆的单轮 PWM（全系统统一 800 档）

var _hold_act := ""
var _joy_held := false
var _last_joy_cmd := Vector2.ZERO  # 上一次真正下发的摇杆指令（模拟值量化后对比用）
var _last_spin_active := false     # 原地旋转模式下当前是否在转（避免斜向切换时漏停残余自转）
var _spin_mode := false

func _ready() -> void:
	# 持续型：按住下发动，松开发 stop。
	for path in _HOLD_ACTIONS:
		var btn: Button = get_node("Btns/" + path)
		var act: String = _HOLD_ACTIONS[path]
		btn.button_down.connect(_on_hold_down.bind(act))
		btn.button_up.connect(_on_hold_up)

	# 夹取/松夹合一：toggle 按下夹、松开松（离散动作，不补 stop）。
	var grip: Button = get_node("Btns/ClawClipReleaseBtn")
	grip.toggled.connect(_on_grip_toggled)

	# 回正：机械臂 + 转向一次性归位。
	var reset_btn: Button = get_node("Btns/ResetPosBtn")
	reset_btn.pressed.connect(_on_reset_pressed)

	# 三灯开关：kind 与按钮名映射。
	var lights := {"ForeLight": "front", "VibeLight": "vibe", "BackLight": "back"}
	for path in lights:
		var b: Button = get_node("LightCtrl/" + path)
		b.toggled.connect(_on_light_toggled.bind(lights[path]))

	# 摇杆 pressed/released 由 tscn 静态连到本面板（_on_joystick_pressed/_on_joystick_release）。

func _process(_delta: float) -> void:
	if _joy_held:
		_update_joystick()

# ============================== 机械臂 / 灯光 ==============================

func _on_hold_down(act: String) -> void:
	DeviceConn.send_command(CP.arm(act))
	_hold_act = act

func _on_hold_up() -> void:
	if not _hold_act.is_empty():
		DeviceConn.send_command(CP.stop("arm"))
	_hold_act = ""

func _on_grip_toggled(on: bool) -> void:
	DeviceConn.send_command(CP.arm("clip" if on else "release"))

func _on_reset_pressed() -> void:
	DeviceConn.send_command(CP.reset())

func _on_light_toggled(on: bool, kind: String) -> void:
	DeviceConn.send_command(CP.light(kind, on))

## 重连/查询后用板端状态同步按钮（不触发 toggled 回调，避免反向多下指令）。
func sync_state(lights: Dictionary, grip_close: bool) -> void:
	var map := {"front": "ForeLight", "vibe": "VibeLight", "back": "BackLight"}
	for kind: String in map:
		var b: Button = get_node_or_null("LightCtrl/" + map[kind])
		if b != null:
			b.set_pressed_no_signal(bool(lights.get(kind, false)))
	var grip: Button = get_node_or_null("Btns/ClawClipReleaseBtn")
	if grip != null:
		grip.set_pressed_no_signal(grip_close)

## 板端状态位 → 面板按钮。bit0-2 灯 / bit3 夹爪 / bit4 AI busy（仅 apply_ai 时回抛）。
## bit 布局与板端 command.cpp::state_bits() 逐位 mirror，改一侧必改另一侧。
func apply_state_bits(bits: int, apply_ai: bool = false) -> void:
	sync_state({
		"front": (bits & 1) != 0,
		"vibe": (bits & 2) != 0,
		"back": (bits & 4) != 0,
	}, (bits & 8) != 0)
	if apply_ai:
		ai_busy_changed.emit((bits & 16) != 0)

# ============================== 摇杆驱动映射 ==============================

## 转向行为：false=转向舵（需前轮回正），true=原地旋转。
func set_spin_mode(on: bool) -> void:
	_spin_mode = on

## 外部强制松开摇杆（切模式 / 收起控制区时调用）。
## 隐藏正在被拖拽的摇杆可能收不到 release 事件，会把车留在"前进" —— 必须显式收尾。
func release_all() -> void:
	if _joy_held:
		_on_joystick_release()

func _on_joystick_pressed() -> void:
	_joy_held = true
	_last_joy_cmd = Vector2.ZERO
	_last_spin_active = false

func _on_joystick_release() -> void:
	_joy_held = false
	_last_joy_cmd = Vector2.ZERO
	_last_spin_active = false
	manual_takeover.emit()
	DeviceConn.send_command(CP.drive(0))
	if _spin_mode:
		DeviceConn.send_command(CP.spin(0))
	else:
		DeviceConn.send_command(CP.servo(0, _SERVO_CENTER))

func _update_joystick() -> void:
	# 内置 VirtualJoystick 把分量写入 4 个 vjoy_* action，据此还原方向向量
	var x: float = Input.get_action_strength("vjoy_right") - Input.get_action_strength("vjoy_left")
	var y: float = Input.get_action_strength("vjoy_down") - Input.get_action_strength("vjoy_up")
	var v: Vector2 = Vector2(x, y)
	# 上=前进：推进取 -y；左右取 x。量化成：速度 低速/高速 两档 + 固定转向，死区内归零
	var throttle := 0.0
	if absf(v.y) >= _JOY_DEADZONE:
		var sp: float = _JOY_FAST if absf(v.y) > _JOY_FAST_THRESH else _JOY_SLOW
		throttle = signf(v.y) * -sp
	var steering := 0.0
	if absf(v.x) >= _JOY_DEADZONE:
		steering = signf(v.x) * _JOY_STEER
	# 原地旋转模式下摇杆斜向（左右+前后同时有效）：原地转与前进物理互斥，忽略转向、只前进/后退。
	# 普通（转向舵）模式保留"一边转弯一边前进"的原有行为。
	var diagonal: bool = _spin_mode and absf(v.x) >= _JOY_DEADZONE and absf(v.y) >= _JOY_DEADZONE
	# cmd 仅做"是否变化"的去重；斜向时 steering 归零，避免单独触发自转。
	var cmd := Vector2(throttle, 0.0 if diagonal else steering)
	if cmd == _last_joy_cmd:
		return
	_last_joy_cmd = cmd
	# 摇杆操作 = 手动接管：打断板端 AI 闭环，聊天发送按钮恢复「发送」
	manual_takeover.emit()
	if cmd == Vector2.ZERO:
		# 松手/居中：停四轮；原地旋转模式下停旋转（复位自转态），否则转向回正
		DeviceConn.send_command(CP.drive(0))
		if _spin_mode:
			DeviceConn.send_command(CP.spin(0))
			_last_spin_active = false
		else:
			DeviceConn.send_command(CP.servo(0, _SERVO_CENTER))
		return
	# 直接驱动：油门 → 全车 drive；左右 → 转向舵；原地旋转模式下左右推改为原地旋转。
	var drive_spd := int(round(absf(throttle) * _DRIVE_MAX))
	if throttle < 0:
		drive_spd = -drive_spd
	DeviceConn.send_command(CP.drive(clampi(drive_spd, -1000, 1000)))
	if _spin_mode:
		# 原地旋转模式：仅纯左右（非斜向）才自转；斜向/前进时若此前在转则补停残余自转。
		var want_spin: bool = (not diagonal) and absf(steering) >= _JOY_DEADZONE
		var want_dir: int = 1 if steering > 0 else -1
		if want_spin != _last_spin_active:
			DeviceConn.send_command(CP.spin(want_dir if want_spin else 0, _SPIN_SPEED))
			_last_spin_active = want_spin
	else:
		DeviceConn.send_command(CP.servo(0, clampi(
			_SERVO_CENTER + int(round(steering / _JOY_STEER * _SERVO_RANGE)), 50, 250)))
