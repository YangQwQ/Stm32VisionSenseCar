extends Button
## 蓝牙设备列表项：显示设备名 + MAC，点击即触发 selected（选中该设备去配网）。

signal selected(name: String, address: String)

func setup(device_name: String, address: String) -> void:
	$Name.text = device_name
	$Mac.text = address

func _on_device_selected() -> void:
	if $Mac.text.is_empty():
		return
	selected.emit($Name.text, $Mac.text)
