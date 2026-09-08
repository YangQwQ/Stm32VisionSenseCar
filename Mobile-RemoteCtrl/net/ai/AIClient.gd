extends Node
## 云端多模态 AI 客户端（Stub）。真实调用待配置 API Key 后接入。

const CP := preload("res://net/proto/CommandProto.gd")

signal ai_response(tool_call: Dictionary, text: String)

## 中转模式：带图 + 消息请求 AI，桩返回模拟工具调用以走通审批 UI 流。
func ask(image: Image, message: String) -> void:
	var tool: Dictionary = CP.move(0.6, -0.2)
	tool["reason"] = "（桩）前方疑似障碍物，建议左转绕行"
	await get_tree().create_timer(1.0).timeout
	ai_response.emit(tool, "我在画面上看到了%s，建议如下。参数见审批卡。" % message)