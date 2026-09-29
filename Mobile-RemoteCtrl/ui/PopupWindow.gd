extends PanelContainer
## 通用弹窗：一套标题/按钮，两种用途，靠 TabContainer 切页 ——
##  - WifiCfg 页：连接设备时输入 WiFi 与云端 AI 配置（可全空 = 仅连接）
##  - Message 页：一句话确认（如「是否确认删除该会话？」）
## 由调用方 open_provision() / open_confirm()；结果经信号回抛，弹窗不持有业务状态。
## 显隐由 Main 的 BGDimSharder 跟随（visibility_changed）。

## 配网/连接确认：把 WiFi 与 AI 配置原样回抛（各字段可空）。
signal provision_confirmed(ssid: String, password: String, url: String, key: String, model: String)
## Message 页确认。
signal confirmed
## 任意一页取消。
signal cancelled

const _TAB_WIFI := 0
const _TAB_MESSAGE := 1

@onready var _title: Label = $VBox/Title
@onready var _tabs: TabContainer = $VBox/TabContainer
@onready var _device_label: Label = $VBox/TabContainer/WifiCfg/DeviceLabel
@onready var _ssid: LineEdit = $VBox/TabContainer/WifiCfg/Ssid
@onready var _password: LineEdit = $VBox/TabContainer/WifiCfg/Password
@onready var _url: LineEdit = $VBox/TabContainer/WifiCfg/Url
@onready var _key: LineEdit = $VBox/TabContainer/WifiCfg/Key
@onready var _model: LineEdit = $VBox/TabContainer/WifiCfg/Model
@onready var _message: Label = $VBox/TabContainer/Message
@onready var _confirm_btn: Button = $VBox/Btns/Confirm

var _mode := _TAB_WIFI

# ============================== 对外接口 ==============================

## 连接设备：预填已存配置（留空即保持旧值），确认后发 provision_confirmed。
func open_provision(device_name: String) -> void:
	_mode = _TAB_WIFI
	_title.text = "连接设备配置确认"
	_device_label.text = "设备：%s" % device_name
	var w: Dictionary = Store.get_wifi()
	var a: Dictionary = Store.get_ai()
	_ssid.text = str(w.get("ssid", ""))
	_password.text = str(w.get("password", ""))
	_url.text = str(a.get("url", ""))
	_key.text = str(a.get("key", ""))
	_model.text = str(a.get("model", ""))
	_confirm_btn.text = "连接"
	_tabs.current_tab = _TAB_WIFI
	_open()

## 纯确认：确认后只发 confirmed，调用方自己记着要做什么。
func open_confirm(title: String, message: String, confirm_text: String = "确认") -> void:
	_mode = _TAB_MESSAGE
	_title.text = title
	_message.text = message
	_confirm_btn.text = confirm_text
	_tabs.current_tab = _TAB_MESSAGE
	_open()

func close() -> void:
	AnimationManager.fade_scale_out(self)
	await get_tree().create_timer(0.2).timeout
	visible = false

# ============================== 内部 ==============================

func _open() -> void:
	visible = true
	AnimationManager.fade_scale_in(self)

func _on_confirm_pressed() -> void:
	if _mode == _TAB_MESSAGE:
		confirmed.emit()
	else:
		# 允许全空直接连接：SSID/URL 留空 = 仅连接，不下发配网/AI。
		provision_confirmed.emit(_ssid.text.strip_edges(), _password.text,
			_url.text.strip_edges(), _key.text, _model.text.strip_edges())
	close()

func _on_cancel_pressed() -> void:
	cancelled.emit()
	close()
