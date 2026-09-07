extends PanelContainer
## 连接/配网弹窗：输入 WiFi 与云端 AI 配置，经 BLE 下发给小车（DIRECT 链路用）。
## WiFi 与 AI 均可留空（仅连接不配网）；确认/取消信号。弹窗打开时从本地存储预填，避免反复输入。

signal confirmed(ssid: String, password: String, url: String, key: String, model: String)
signal cancelled

@onready var _device_label: Label = $VBox/DeviceLabel
@onready var _ssid: LineEdit = $VBox/Ssid
@onready var _password: LineEdit = $VBox/Password
@onready var _url: LineEdit = $VBox/Url
@onready var _key: LineEdit = $VBox/Key
@onready var _model: LineEdit = $VBox/Model
@onready var _status: Label = $VBox/Status

func set_device(name: String) -> void:
	_device_label.text = "设备：%s" % name

func popup() -> void:
	# 预填已存配置，留空即保持旧值。
	var w: Dictionary = Store.get_wifi()
	var a: Dictionary = Store.get_ai()
	_ssid.text = str(w.get("ssid", ""))
	_password.text = str(w.get("password", ""))
	_url.text = str(a.get("url", ""))
	_key.text = str(a.get("key", ""))
	_model.text = str(a.get("model", ""))
	_status.text = ""
	visible = true
	AnimationManager.fade_scale_in(self)

func close() -> void:
	AnimationManager.fade_scale_out(self)
	await get_tree().create_timer(0.2).timeout
	visible = false

## 允许全空直接连接：SSID/URL 留空=仅连接，不下发配网/AI。
func _on_confirm_pressed() -> void:
	confirmed.emit(_ssid.text.strip_edges(), _password.text, _url.text.strip_edges(), _key.text, _model.text.strip_edges())
	close()

func _on_cancel_pressed() -> void:
	cancelled.emit()
	close()