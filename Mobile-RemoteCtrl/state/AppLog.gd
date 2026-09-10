extends Node
## 本地日志落盘（autoload `AppLog`）：每次启动用固定文件名重新写入（覆盖旧文件），按行追加带时间戳的日志。
## 路径：user://logs/app.log（Android 即 /data/data/<包名>/files/logs/app.log）。

const _PATH := "user://logs/app.log"

func _ready() -> void:
	DirAccess.make_dir_recursive_absolute("user://logs")
	# 每次启动重新创建（WRITE 截断旧内容）：固定名文件，本会话从空开始写。
	var w := FileAccess.open(_PATH, FileAccess.WRITE)
	if w:
		w.close()
	var v := Engine.get_version_info()
	write("启动", "App %d.%d.%d (Godot %s)" % [
		v.get("major", 0), v.get("minor", 0), v.get("patch", 0),
		v.get("string", ""),
	])

## 追加一行日志：带当前时间戳与标签 tag。
func write(tag: String, msg: String) -> void:
	var now := Time.get_time_dict_from_system()
	var ts := "%02d:%02d:%02d" % [now["hour"], now["minute"], now["second"]]
	var line := "[%s][%s] %s\n" % [ts, tag, msg]
	var f := FileAccess.open(_PATH, FileAccess.READ_WRITE)
	if f == null:
		return
	f.seek_end()
	f.store_string(line)
	f.close()