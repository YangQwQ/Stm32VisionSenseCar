extends PanelContainer
## 通用弹窗：一套标题/按钮，两种用途，靠 TabContainer 切页 ——
##  - WifiCfg 页：连接设备时输入 WiFi 与云端 AI 配置（可全空 = 仅连接）；模型名从服务端拉列表后下拉选
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

## URL/Key 停输后才拉模型列表，避免逐字请求。
const _FETCH_DELAY := 0.6
const _FETCH_TIMEOUT := 8.0
## 拉不到模型时占位（列表为空 = 未选模型，回抛空串让板端沿用旧值）。
const _NO_MODEL := "（拉取失败，检查 URL/Key）"
const _NO_URL := "（请先填接口 URL）"

@onready var _title: Label = $VBox/Title
@onready var _tabs: TabContainer = $VBox/TabContainer
@onready var _device_label: Label = $VBox/TabContainer/WifiCfg/DeviceLabel
@onready var _ssid: LineEdit = $VBox/TabContainer/WifiCfg/Ssid
@onready var _password: LineEdit = $VBox/TabContainer/WifiCfg/Password
@onready var _url: LineEdit = $VBox/TabContainer/WifiCfg/Url
@onready var _key: LineEdit = $VBox/TabContainer/WifiCfg/Key
@onready var _model: OptionButton = $VBox/TabContainer/WifiCfg/Model
@onready var _model_http: HTTPRequest = $ModelHttp
@onready var _fetch_timer: Timer = $ModelFetchTimer
@onready var _message: Label = $VBox/TabContainer/Message
@onready var _confirm_btn: Button = $VBox/Btns/Confirm

var _mode := _TAB_WIFI
## 已存/已选的模型名，用于在列表刷新后仍能对回同一项。
var _cur_model := ""
## 下拉里是否只有占位项（有则说明没拿到真列表，回抛时置空模型名）。
var _model_placeholder := true

func _ready() -> void:
	_fetch_timer.wait_time = _FETCH_DELAY
	_model_http.timeout = _FETCH_TIMEOUT
	_model_http.request_completed.connect(_on_models_received)
	_fetch_timer.timeout.connect(_fetch_models)

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
	_cur_model = str(a.get("model", ""))
	_set_models(PackedStringArray(), _NO_URL)
	_confirm_btn.text = "连接"
	_tabs.current_tab = _TAB_WIFI
	_open()
	# 预填的 URL 也会触发一次拉取，让列表回到最新。
	if not _url.text.strip_edges().is_empty():
		_fetch_timer.start()

## 用户改选后记住新值，后续重拉列表时仍选它。
func _on_model_selected(idx: int) -> void:
	if idx >= 0:
		_cur_model = _model.get_item_text(idx)

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
			_url.text.strip_edges(), _key.text, _selected_model())
	close()

func _on_cancel_pressed() -> void:
	cancelled.emit()
	close()

# ============================== 模型列表 ==============================

## 取下拉当前项；下拉里只有占位项时视为未选模型（回抛空串，板端沿用旧值）。
func _selected_model() -> String:
	if _model_placeholder:
		return ""
	return _model.get_item_text(_model.get_selected())

## URL/Key 变化：防抖后重拉。Key 常在 URL 之后填，故两者都触发。
func _on_ai_endpoint_changed(_new_text: String) -> void:
	_fetch_timer.start()

func _fetch_models() -> void:
	var base := _url.text.strip_edges()
	if base.is_empty():
		_set_models(PackedStringArray(), _NO_URL)
		return
	if not base.begins_with("http"):
		base = "https://" + base
	# 与板端 normalize_ai_url 同规则：URL 允许只填到 /v1，或填完整 /chat/completions。
	var root := base.trim_suffix("/")
	if root.ends_with("/chat/completions"):
		root = root.trim_suffix("/chat/completions")
	var headers := PackedStringArray()
	var k := _key.text.strip_edges()
	if not k.is_empty():
		headers.append("Authorization: Bearer %s" % k)
	if _model_http.get_http_client_status() != HTTPClient.STATUS_DISCONNECTED:
		_model_http.cancel_request()
	if _model_http.request(root + "/models", headers) != OK:
		_set_models(PackedStringArray(), _NO_MODEL)

func _on_models_received(result: int, code: int, _headers: PackedStringArray, body: PackedByteArray) -> void:
	if result != HTTPRequest.RESULT_SUCCESS or code < 200 or code >= 300:
		_set_models(PackedStringArray(), _NO_MODEL)
		return
	var names := _parse_model_names(JSON.parse_string(body.get_string_from_utf8()))
	if names.is_empty():
		_set_models(PackedStringArray(), _NO_MODEL)
		return
	_set_models(names, "")

## 兼容 OpenAI 的 {"data":[{"id":...}]}、Ollama 的 {"models":[{"name":...}]} 与裸数组。
func _parse_model_names(parsed: Variant) -> PackedStringArray:
	var out := PackedStringArray()
	var arr: Variant = parsed
	if parsed is Dictionary:
		var d: Dictionary = parsed as Dictionary
		arr = d.get("data", d.get("models", []))
	if not (arr is Array):
		return out
	for it: Variant in arr as Array:
		var id := ""
		if it is String:
			id = it as String
		elif it is Dictionary:
			var e: Dictionary = it as Dictionary
			for k: String in ["id", "name", "model"]:
				if e.has(k):
					id = str(e[k])
					break
		if not id.is_empty() and not out.has(id):
			out.append(id)
	return out

## 重建下拉：placeholder 非空 = 本次没拿到可用列表（只留一句说明，且视为未选）。
## 否则铺开列表，已存/已选的模型即使不在列表里也留在首位，避免静默换成别的模型。
func _set_models(names: PackedStringArray, placeholder: String) -> void:
	_model.clear()
	var usable := PackedStringArray()
	if not _cur_model.is_empty() and not names.has(_cur_model):
		usable.append(_cur_model)
	usable.append_array(names)
	if not placeholder.is_empty() or usable.is_empty():
		_model.disabled = true
		_model_placeholder = true
		_model.add_item(placeholder if not placeholder.is_empty() else _NO_MODEL)
		_model.select(0)
		return
	for n: String in usable:
		_model.add_item(n)
	_model.disabled = false
	_model_placeholder = false
	var idx := usable.find(_cur_model)
	_model.select(idx if idx >= 0 else 0)
