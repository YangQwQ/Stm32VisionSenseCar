extends Node
## 全局状态：持有连接对象引用与当前模式，供各视图互通。

enum AiMode { RELAY, DIRECT }

var ble: Node
var ws: Node
var ai: Node

var ai_mode := AiMode.DIRECT
var current_image: Image = null  # 待发送的（可能已编辑）图片
var pending_tool: Dictionary = {}  # 待审批的工具调用