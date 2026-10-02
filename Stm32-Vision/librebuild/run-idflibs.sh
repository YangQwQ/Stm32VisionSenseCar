#!/bin/bash
# 只跑 idf-libs 一步，产出 out/ 静态库（绕过 build.sh 里必失败的 srmodels/bootloader/mem-variant）。
# 配置与 build.sh 默认 esp32s3 全量一致：common;esp32s3;debug_default;qio;80m;qio_ram
#
# 路径不写死：**未显式配置会在开头交互提问**（回车即采用中括号里的默认值）。
# 想免提问就导出变量（也便于脚本化；非交互运行会自动跳过提问、直接用默认值）：
#   IDF_PATH    esp-idf 路径（默认 $HOME/esp-idf）
#   LIB_BUILDER esp32-arduino-lib-builder 路径（默认 $HOME/esp32-arduino-lib-builder）
#   IDFLIBS_LOG 编库日志落点（默认 $LIB_BUILDER/idflibs.log）
set -uo pipefail

# 交互取路径：有 TTY 才提问（提示走 stderr，回车用默认值）；无 TTY 直接用默认值
ask_path() { # <提示名> <默认值>
  local name=$1 def=$2 ans=
  if [ -t 0 ]; then
    printf '%s\n  [%s]: ' "$name" "$def" >&2
    read -r ans || ans=
  fi
  printf '%s\n' "${ans:-$def}"
}

# 从 Git Bash 直接调起 WSL 时 HOME 可能被指到 Windows 家目录（/mnt/c/...），
# 那样 esp-idf 工具链与默认路径都会错位。这里只报警不改写（不写死任何用户名）。
case "$HOME" in
  /mnt/*) echo "⚠️ HOME 指向 Windows 目录（$HOME）。请先 export HOME=/home/<你的用户名> 再重跑。" >&2 ;;
esac

[ -n "${IDF_PATH:-}" ]    || IDF_PATH=$(ask_path "esp-idf 路径" "$HOME/esp-idf")
[ -n "${LIB_BUILDER:-}" ] || LIB_BUILDER=$(ask_path "esp32-arduino-lib-builder 路径" "$HOME/esp32-arduino-lib-builder")
export IDF_PATH
export IDF_COMPONENT_OVERWRITE_MANAGED_COMPONENTS=1

cd "$LIB_BUILDER" || { echo "LIB_BUILDER_NOT_FOUND: $LIB_BUILDER"; exit 2; }
[ -f "$IDF_PATH/export.sh" ] && . "$IDF_PATH/export.sh" >/dev/null 2>&1
command -v idf.py >/dev/null || { echo "IDF_NOT_FOUND: $IDF_PATH（检查 esp-idf 路径，或先手动 . export.sh）"; exit 2; }
echo "idf.py: $(command -v idf.py)"

# esp_video 的 version 不设上界时，组件管理器会去评估新版 manifest：其中 esp_h264 依赖规则的 if 用了
# ESP_VIDEO_USE_CUSTOMIZED_ESP_H264_VERSION（该符号由 esp_h264 定义，S3 上不装它）⇒ kconfig missed_keys
# 非空 ⇒ 管理器永远 sys.exit(10) ⇒ CMake 重跑一次后 FATAL。钉到锁文件里已有的版本即收敛（本工程不用 esp_video）。
sed -i 's@version: ">=2.3.0"@version: "2.4.1"@' main/idf_component.yml
grep -A1 'espressif/esp_video:' main/idf_component.yml | grep -q '2\.4\.1' \
  || echo "⚠️ esp_video 版本没钉住（main/idf_component.yml 变了？），组件管理器可能 exit(10) 死循环" >&2

# esp32-camera 必须是**含 jpeg_buffer_size 的版本**：camera.cpp 用它把 VGA JPEG 的 recv_size 抬到 256KB
# 防 FB-OVF（旧版没这字段 ⇒ 草图编译报错；且默认 recv_size = w*h/5 ≈ 60KB，高熵帧溢出会 ll_cam_stop 停图传）。
# dependencies.lock 里可能钉着那个旧提交，检测到就删掉这条，让管理器按 manifest 的 master 重新解析。
CAM_HDR=managed_components/espressif__esp32-camera/driver/include/esp_camera.h
if [ -f "$CAM_HDR" ] && ! grep -q jpeg_buffer_size "$CAM_HDR"; then
  echo "== esp32-camera 是缺 jpeg_buffer_size 的旧版：删 lock 条目强制重解析 =="
  L=$(mktemp) && awk '/^  espressif\/esp32-camera:$/{s=1;next} s&&/^  [^ ]/{s=0} !s' dependencies.lock > "$L" && mv "$L" dependencies.lock
fi

CFG="configs/defconfig.common;configs/defconfig.esp32s3;configs/defconfig.debug_default;configs/defconfig.qio;configs/defconfig.80m;configs/defconfig.qio_ram"
LOG="${IDFLIBS_LOG:-$LIB_BUILDER/idflibs.log}"
O="$LIB_BUILDER/out/tools/esp32-arduino-libs/esp32s3"

for attempt in 1 2 3; do
  echo "== idf-libs attempt $attempt $(date) =="
  rm -rf build sdkconfig
  if idf.py -DIDF_TARGET=esp32s3 -DSDKCONFIG_DEFAULTS="$CFG" idf-libs 2>&1 | tee "$LOG"; then
    # 校验 out 已生成
    if [ -f "$O/lib/libmbedtls.a" ] && grep -q 'SSL_IN_CONTENT_LEN=8192' "$O/sdkconfig"; then
      echo "IDFLIBS_OK: out sdkconfig has 8192"
      exit 0
    else
      echo "IDFLIBS_DONE_BUT_NOT_CHECKED"
      exit 3
    fi
  fi
  echo "-- attempt $attempt failed (可能组件 git fetch 网络抖动)，重试 --"
  sleep 5
done
echo "IDFLIBS_FAILED"
exit 1
