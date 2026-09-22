#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印固件**编译期真实展开**的宏值 —— 用于证实/证伪"某个 sdkconfig 补丁到底进没进固件"。

为什么需要它：Arduino/IDF 的很多"配置"不是运行时变量，而是**宏**（`WIFI_INIT_CONFIG_DEFAULT()`
把 `CONFIG_ESP_WIFI_*` 展开进 `wifi_init_config_t`）。这类改动"编译通过"完全不能证明生效
——头文件没被覆盖、或者改错了那一份（变体目录 vs 顶层）都会静默无效。本工具直接把预处理器的
宏表打出来，是强证据。

做法：从草图缓存的 `compile_commands.json` 里取一条**真实编译命令**的完整参数（`-I` / `-isystem`
/ `-D` 一个不差），换成 `-E -dM` 只做预处理并打印全部宏定义；探针文件只 include 指定头文件。

用法：
    uv run python tools/probe_macro.py                    # 默认看 WiFi 缓冲区那几个
    uv run python tools/probe_macro.py -p WIFI_ CONFIG_ESP_WIFI_
    uv run python tools/probe_macro.py -i esp_wifi.h -i lwipopts.h
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

DEFAULT_PATTERNS = (
    "WIFI_STATIC_RX_BUFFER_NUM", "WIFI_DYNAMIC_RX_BUFFER_NUM", "WIFI_RX_BA_WIN",
    "WIFI_STATIC_TX_BUFFER_NUM", "WIFI_DYNAMIC_TX_BUFFER_NUM", "WIFI_CACHE_TX_BUFFER_NUM",
    "WIFI_ENABLE_CACHE_TX_BUFFER", "WIFI_TX_BUFFER_TYPE",
)


def find_compile_commands(sketch: Path):
    """草图缓存目录按 (草图路径+fqbn) 哈希命名，所以扫全部挑出 entries 指向本草图的那份。"""
    sketches = Path(os.environ["LOCALAPPDATA"]) / "arduino" / "sketches"
    if not sketches.is_dir():
        return None, f"没有草图缓存目录：{sketches}"
    marker = sketch.name  # 如 "Stm32-Vision"
    best = None
    for p in sorted(sketches.glob("*/compile_commands.json"),
                    key=lambda x: x.stat().st_mtime, reverse=True):
        try:
            head = p.read_text(encoding="utf-8", errors="replace")[:8000]
        except OSError:
            continue
        if marker in head:
            best = p
            break
    if not best:
        return None, f"没有指向 {marker} 的 compile_commands.json（先编译一次草图）"
    return best, None


def split_args(entry):
    if "arguments" in entry:
        args = list(entry["arguments"])
    else:
        args = re.split(r'\s+(?=(?:[^"]*"[^"]*")*[^"]*$)', entry["command"])
    # 只留搜索路径与宏定义；丢掉输入文件/-o/-c 等
    drop_with_value = {"-o", "-MF", "-MT", "-MQ", "-x"}
    drop_alone = {"-c", "-E", "-S", "-MD", "-MMD", "-MP"}
    keep, i = [], 0
    while i < len(args):
        a = args[i]
        if a in drop_with_value:
            i += 2
            continue
        if a in drop_alone or a.startswith("-o"):
            i += 1
            continue
        if a in (entry.get("file"), entry.get("output")):
            i += 1
            continue
        keep.append(a)
        i += 1
    return keep[0], keep[1:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--sketch", default=str(Path(__file__).resolve().parent.parent),
                    help="草图目录（默认本 tools/ 的上一级）")
    ap.add_argument("-i", "--include", action="append",
                    default=["esp_wifi.h", "esp_wifi_default.h"],
                    help="探针要 include 的头文件，可重复")
    ap.add_argument("-p", "--pattern", action="append", default=None,
                    help="只打印含此子串的宏，可重复（默认 WiFi 缓冲区那几个）")
    a = ap.parse_args()

    sketch = Path(a.sketch).resolve()
    ccjson, err = find_compile_commands(sketch)
    if err:
        print(err)
        return 2
    entries = json.loads(ccjson.read_text(encoding="utf-8", errors="replace"))
    print(f"compile_commands.json: {ccjson}  （{len(entries)} 条）")

    pick = next((e for e in entries
                 if e.get("file", "").replace("\\", "/").endswith("WiFiGeneric.cpp")), None)
    if pick is None:
        pick = entries[0]
    compiler, flags = split_args(pick)
    print("编译器:", compiler)

    with tempfile.TemporaryDirectory(prefix="macroprobe-") as td:
        probe = Path(td) / "probe.cpp"
        probe.write_text("".join(f"#include <{h}>\n" for h in a.include), encoding="utf-8")
        r = subprocess.run([compiler, "-E", "-dM", *flags, str(probe)],
                           capture_output=True, text=True, errors="replace")
    if r.returncode != 0:
        print("预处理失败：")
        print((r.stderr or "")[:2000])
        return 1

    pats = tuple(a.pattern) if a.pattern else DEFAULT_PATTERNS
    print("\n固件编译期的真实展开值：")
    hit = 0
    for line in r.stdout.splitlines():
        if any(p in line for p in pats):
            print("  " + line)
            hit += 1
    if not hit:
        print("  （无匹配——检查 -p 或 -i）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
