#!/bin/bash
# 把 WSL 里编库产物覆盖到 Windows 侧 Arduino15 的 esp32s3-libs/<内核版本>。
#
# 路径不写死：**未显式配置会在开头交互提问**（回车即采用中括号里的默认值）。
# 想免提问就导出变量（也便于脚本化；非交互运行会自动跳过提问、直接用默认值）：
#   LIB_BUILDER      esp32-arduino-lib-builder 路径（默认 $HOME/esp32-arduino-lib-builder）
#   ESP32S3_LIBS_SRC 编库产物的 esp32s3 目录
#                    （默认 $LIB_BUILDER/out/tools/esp32-arduino-libs/esp32s3）
#   ESP32S3_LIBS_DST 目标目录（不设则先按 %LOCALAPPDATA% 自动推，推不出才提问）
#   ESP32_CORE_VER   已装 esp32 内核版本（默认 3.3.11，须与实际安装目录名一致）
set -e

# 交互取路径：有 TTY 才提问（提示走 stderr，回车用默认值）；无 TTY 直接用默认值
ask_path() { # <提示名> <默认值>
  local name=$1 def=$2 ans=
  if [ -t 0 ]; then
    printf '%s\n  [%s]: ' "$name" "$def" >&2
    read -r ans || ans=
  fi
  printf '%s\n' "${ans:-$def}"
}

: "${LIB_BUILDER:=$HOME/esp32-arduino-lib-builder}"
VER="${ESP32_CORE_VER:-3.3.11}"

# ---- 源：编库产物 ----
SRC_DEF="${ESP32S3_LIBS_SRC:-$LIB_BUILDER/out/tools/esp32-arduino-libs/esp32s3}"
if [ -n "${ESP32S3_LIBS_SRC:-}" ]; then S="$ESP32S3_LIBS_SRC"; else
  S=$(ask_path "编库产物目录（esp32s3）" "$SRC_DEF")
fi

# ---- 目标：Arduino15 的 esp32s3-libs/<版本> ----
DST_DEF="${ESP32S3_LIBS_DST:-}"
if [ -z "$DST_DEF" ]; then
  # WSL 互操作：向 Windows 问 %LOCALAPPDATA% 再转成 WSL 路径（不写死用户名）
  LA=$(cmd.exe /c echo %LOCALAPPDATA% 2>/dev/null | tr -d '\r\n' || true)
  if [ -n "$LA" ] && command -v wslpath >/dev/null 2>&1; then
    DST_DEF="$(wslpath "$LA" 2>/dev/null)/Arduino15/packages/esp32/tools/esp32s3-libs/$VER" || DST_DEF=
  fi
fi
if [ -n "${ESP32S3_LIBS_DST:-}" ]; then D="$ESP32S3_LIBS_DST"; else
  [ -n "$DST_DEF" ] && D=$(ask_path "Arduino15 目标目录" "$DST_DEF") || D=
fi
if [ -z "$D" ]; then
  {
    echo "未确定目标目录。请显式指定，例如："
    echo "  export ESP32S3_LIBS_DST=/mnt/c/Users/<你的用户名>/AppData/Local/Arduino15/packages/esp32/tools/esp32s3-libs/$VER"
    echo "（Windows 的 Arduino15 通常在 %LOCALAPPDATA%\\Arduino15）"
  } >&2
  exit 2
fi
[ -d "$S" ] || { echo "源目录不存在: $S（先跑 run-idflibs.sh）" >&2; exit 2; }

echo "源  : $S"
echo "目标: $D"
if [ -t 0 ]; then
  printf '确认覆盖？[Y/n]: ' >&2
  read -r ok || ok=
  case "${ok:-y}" in [Nn]*) echo "已取消" >&2; exit 0 ;; esac
fi

echo "== 清理顶层套嵌 =="
for d in lib include flags ld; do
  if [ -d "$D/$d/$d" ]; then
    echo "remove nesting: $d/$d"
    rm -rf "$D/$d/$d"
  fi
done

# 重建每个顶级目录
for d in lib include flags ld; do
  mkdir -p "$D/$d"
  echo "== cp $S/$d/* -> $D/$d/"
  cp -rf "$S/$d/"* "$D/$d/"
done

# 覆盖 sdkconfig
cp -f "$S/sdkconfig" "$D/sdkconfig"

# 最终统计
echo
echo "=== FINAL CHECK ==="
echo "lib  : $(ls "$D/lib" | wc -l) archives"
echo "include dirs: $(find "$D/include" -maxdepth 1 -type d | wc -l)"
echo "flags  : $(ls "$D/flags" | wc -l) files"
echo "ld  : $(ls "$D/ld" | wc -l) files"
echo "mbedtls size: $(ls -la "$D/lib/libmbedtls.a" | awk '{print $5}')"
ls -la "$D/sdkconfig" "$D/lib/libmbedtls.a"
