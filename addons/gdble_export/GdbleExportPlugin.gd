@tool
extends EditorPlugin

## GDBLE 导出插件入口：注册 Android 导出插件，
## 把 gdble-release.aar 打包进 APK（机制同 miniaudio 示例）。

const GdbleAndroidExportPlugin := preload("GdbleAndroidExportPlugin.gd")

var export_plugin: EditorExportPlugin = GdbleAndroidExportPlugin.new()

func _enter_tree() -> void:
	add_export_plugin(export_plugin)

func _exit_tree() -> void:
	remove_export_plugin(export_plugin)