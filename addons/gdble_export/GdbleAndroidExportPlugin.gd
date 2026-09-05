@tool
extends EditorExportPlugin

## GDBLE Android 导出插件
##
## 机制（官方 + 最小桥接）：
##  libgdble.so 是一个 godot-rust GDExtension。它有两个入口：
##    1) Java 插件 org.gdble.android.GDBLEPlugin 在启动时 System.loadLibrary("gdble")
##       把 lib/arm64-v8a/libgdble.so 加载进进程（并做权限/BLE Java 侧初始化）；
##    2) engine 还需 load_extension(gdble.gdextension) 调用 gdext_rust_init 注册
##       BluetoothManager 等类。官方 GDBLEPlugin 没覆写 getPluginGDExtensionLibrariesPaths，
##       所以这里用桥接插件 GDBLEBridgePlugin 返回 assets 里的 gdextension 相对路径，
##       让引擎 load_extension -> gdext_rust_init。
##
## 关键：gdble.gdextension 必须带 android_aar_plugin=true。这样导出时引擎会跳过把
## config 与 .so 打进 pck（gdextension_export_plugin.h），它们改由 gdble-release.aar
## 以真实 APK assets/ 与 lib/<abi>/ 提供，避免 sparse pack 读取失败。
## 引擎读到 assets/addons/gdble/gdble.gdextension 后，解析 android 库路径
## res://addons/gdble/libgdble.so -> 非 pck 文件 -> dlopen("libgdble.so") 命中已加载库。
##
## 打包三件事：
##   1) gdble-release.aar：含 assets/addons/gdble/gdble.gdextension、
##      jni/<abi>/libgdble.so、org.gdble.android.GDBLEPlugin（Java 插件类）
##   2) btleplug-release.aar：Rust 侧 JNI 依赖的 com.nonpolynomial.btleplug 类
##   3) gdble_bridge.jar：GDBLEBridgePlugin，覆写 getPluginGDExtensionLibrariesPaths
## 并在 manifest 里注册 GDBLE 与 GDBLEBridge 两个插件。

func _supports_platform(platform) -> bool:
	return platform is EditorExportPlatformAndroid

func _get_name() -> String:
	return "GDBLE"

func _get_android_libraries(_platform, _debug: bool) -> PackedStringArray:
	var libs := PackedStringArray([
		"res://addons/gdble/android/gdble_bridge.jar",
		"res://addons/gdble/android/gdble-release.aar",
		"res://addons/gdble/android/btleplug-release.aar",
	])
	for p: String in libs:
		if not FileAccess.file_exists(p):
			push_warning("[GDBLE] 文件不存在: %s" % p)
	return libs

func _get_android_manifest_application_element_contents(_platform, _debug: bool) -> String:
	# 注册官方 GDBLE 插件（负责加载 libgdble.so）与桥接插件（向引擎提供 gdextension 路径）
	return '<meta-data android:name="org.godotengine.plugin.v2.GDBLE" android:value="org.gdble.android.GDBLEPlugin" />' + \
		   '<meta-data android:name="org.godotengine.plugin.v2.GDBLEBridge" android:value="com.ctrlapp.gdble.GDBLEBridgePlugin" />'
# 注：曾尝试在此注入无 maxSdk 的 ACCESS_FINE_LOCATION（Godot 导出器会把该权限固定写成
# maxSdkVersion=30，API>=31 等于未声明，MIUI 仍用它门禁扫描结果），但根级 hook 在标准导出
# 里不生效，实测无效，已移除。规避方案见 CLAUDE.md「构建环境」段落（导出后 apktool 处理）。