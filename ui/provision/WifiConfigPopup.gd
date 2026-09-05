extends PanelContainer
## 配网弹窗：输入目标 WiFi SSID/密码，经 BLE 下发给小车。确认/取消信号。

signal confirmed(ssid: String, password: String)
signal cancelled

@onready var _ssid: LineEdit = $VBox/Ssid
@onready var _password: LineEdit = $VBox/Password
@onready var _status: Label = $VBox/Status

func popup() -> void:
	visible = true
	AnimationManager.fade_scale_in(self)

func close() -> void:
	AnimationManager.fade_scale_out(self)
	await get_tree().create_timer(0.2).timeout
	visible = false

func _on_confirm_pressed() -> void:
	var ssid: String = _ssid.text.strip_edges()
	var password: String = _password.text
	if ssid.is_empty():
		_status.text = "SSID 不能为空"
		return
	confirmed.emit(ssid, password)
	close()

func _on_cancel_pressed() -> void:
	cancelled.emit()
	close()