# /// script
# requires-python = ">=3.10"
# dependencies = ["websockets>=13", "bleak>=0.22"]
# ///
"""carctl —— 直连小车的操作台：状态 / 日志 / 发指令 / ping / 重启 / 远程 OTA。

板子固定在车上，串口够不着，**所有诊断只能走网络**。这个脚本就是那只手：查板上跑的是哪份
固件、看实时日志、按词表发指令、把板子当跳板去 ping、卡死时远程重启、以及推固件并确认它
真的跑起来了。

全部经 uv 跑（依赖按内联元数据自动装，无需手动 pip）：

    uv run tools/carctl.py status                    # 板上跑的是哪份固件、OTA 会写到哪个槽
    uv run tools/carctl.py log -t 30                 # 实时看板端日志 30 秒
    uv run tools/carctl.py cmd spin dir=1 speed=500  # 发词表指令，收回复
    uv run tools/carctl.py frame -t before-lift      # 抓一张画面（手动操作时的眼睛）
    uv run tools/carctl.py ping 192.168.1.1          # 让板子去 ping（查"关联着但不通"）
    uv run tools/carctl.py reboot                    # 远程重启（链路卡死时的解药）
    uv run tools/carctl.py doctor                    # 分层体检：哪层断的，顺手 BLE 救活
    uv run tools/carctl.py fw                        # 列出本机编出的 .bin 及其指纹
    uv run tools/carctl.py ota <file.bin>            # 推固件 → 等板子回来 → 核对指纹
    uv run tools/carctl.py build --flash             # 编译 → 直接 OTA → 核对指纹（一条龙）
    uv run tools/carctl.py build --clean             # 全量重编（改了源文件或内核 sdkconfig.h）

地址解析顺序：`-H` 指定 → `~/.car_logcat/last_host` 缓存（与 car_logcat.py 共用）→ 扫本机
各 /24 网段的 81 端口并用板子指纹（`GET /` 回 302 `/stream`）确认。成功连上后会写回缓存。

## 两条链路各管什么

- **WS（端口 81）**：指令、状态、日志转发、重启。板端 `log` 开关是**全局单选**，所以本脚本
  在 `log` 模式里每 30s 重申一次，退出时按 `--leave-on` 决定要不要关掉。
- **HTTP（端口 80）**：`GET /update` 读运行分区/固件指纹/OTA 落点；`POST /update` 推固件。

`ota` 的收尾那步是本工具最值钱的地方：写完固件、板子重启回来后，再读一次 `/update`，把**板上
正在跑的指纹**和**你刚推的那个 .bin 的指纹**摆在一起比。指纹取 .bin 偏移 0xb0 的 4 字节
（esptool 按 `--elf-sha256-offset 0xb0` 打进去的 ELF SHA-256 前 4 字节），换个源就是另一份
.bin——所以两边一致就等于"新固件确实在跑"，不一致就是没有。

## 与 car_logcat.py 的分工

`car_logcat.py` 是**记录仪**（长时间抓日志落盘 + AI 每轮画面留档 + 断线自愈），本脚本是**操作
台**（一次性动作，随发随走）。少数基础设施（代理剥离、板子指纹、网段扫描、地址缓存）两边各有
一份拷贝——刻意不互相 import，好让每个脚本都能单独 `uv run` 起来；改动请顺手同步两个文件。
"""

from __future__ import annotations

import argparse
import http.client
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

try:
    from websockets.exceptions import ConnectionClosed
    from websockets.sync.client import connect
except ImportError:  # pragma: no cover - 提示装依赖而不是抛栈
    sys.exit("缺少依赖 websockets：请用 `uv run tools/carctl.py`（会按内联元数据自动装）")

WS_PORT = 81                    # 板端 WS 端口（指令/状态/日志）
HTTP_PORT = 80                  # 板端 HTTP（/update 读写固件信息）
HTTP_TIMEOUT_S = 8.0            # 普通 HTTP 请求上限（板端现算页面，秒回）
OTA_HTTP_TIMEOUT_S = 120.0      # OTA 的 POST：整包固件传完 + 板端写 flash 都在这个窗口里
PING_EVERY_S = 5.0              # 保活周期（< 板端 WS_IDLE_PING_MS 6s）
REASSERT_S = 30.0               # 重申 /log on 的周期（板端 log 开关全局单选，手机会改掉）
BIN_FP_OFFSET = 0xB0            # .bin 里 ELF SHA-256 前 4 字节的偏移（见 ota.cpp fp_hex）
BIN_CHUNK = 8192                # OTA 上传分片
HOST_CACHE = Path.home() / ".car_logcat" / "last_host"   # 与 car_logcat.py 共用同一个缓存
SKETCH_CACHE = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "arduino" / "sketches"

def _find_arduino_cli() -> Path:
    """定位 arduino-cli，按可信度从高到低：

    1. 环境变量 `ARDUINO_CLI` —— 显式指定，**换机器首选**；
    2. PATH 里已有 `arduino-cli`；
    3. Arduino IDE **内置**的那份（与 IDE 同版、缓存目录同源），它在 IDE 安装目录下的
       `resources/app/lib/backend/resources/`，**不在 PATH**，只能猜安装位置；
    4. 都找不到就返回第 3 步的第一个候选，由调用处报错并提示改用 `ARDUINO_CLI`。
    """
    env = os.environ.get("ARDUINO_CLI")
    if env:
        return Path(env)
    on_path = shutil.which("arduino-cli")
    if on_path:
        return Path(on_path)
    tail = "resources/app/lib/backend/resources/arduino-cli"
    names = [tail + ".exe", tail] if os.name == "nt" else [tail]
    bases = [Path(v) for v in (os.environ.get("PROGRAMFILES"),
                               os.environ.get("PROGRAMFILES(X86)"),
                               os.environ.get("ProgramW6432")) if v]
    la = os.environ.get("LOCALAPPDATA")
    if la:
        bases.append(Path(la) / "Programs")
    if os.name == "nt":
        # IDE 允许装到非系统盘（本机就在 D 盘），所以每个存在的盘符都看一眼
        for drive in "CDEFGHIJ":
            d = Path(f"{drive}:/")
            if d.exists():
                bases += [d / "Program Files", d / "Program Files (x86)"]
    else:
        bases.append(Path("/Applications"))          # macOS：下面单独拼 .app 结构
    cands = [b / "Arduino IDE" / n for b in bases for n in names]
    cands.append(Path("/Applications/Arduino IDE.app/Contents") / tail)
    for c in cands:
        if c.is_file():
            return c
    return cands[0]


ARDUINO_CLI = _find_arduino_cli()
SKETCH_DIR = Path(__file__).resolve().parent.parent          # Stm32-Vision/
# ⚠️ arch 段是 esp32 不是 esp32s3（写成 esp32:esp32s3: 会被当成"本平台未安装"）。
# 与 Stm32-Vision/CLAUDE.md §构建要点 逐字一致，改一处就要同步另一处。
FQBN = ("esp32:esp32:esp32s3:FlashSize=16M,FlashMode=dio,PartitionScheme=huge_app,"
        "DebugLevel=debug,PSRAM=opi,EraseFlash=none")

# 状态位（与 command.cpp make_state_bits / 手机 Main.gd 逐位 mirror）
BIT_NAMES = ("前灯", "震灯", "背灯", "夹爪夹紧", "AI忙碌")

# BLE（与 ble.cpp / 手机 BleProfile.gd 逐字 mirror）：WiFi 卡死时唯一还通的链路
BLE_SVC = "0000c0de-0000-1000-8000-00805f9b34fb"     # 服务
BLE_CMD = "0000c0e5-0000-1000-8000-00805f9b34fb"     # 收原始 JSON 指令
BLE_ST = "0000c0e6-0000-1000-8000-00805f9b34fb"      # 状态/日志通知
BLE_NAME = "VisionS3"


def log(msg: str) -> None:
    print(msg, flush=True)


# ---------------- 基础设施：代理 / 地址 / 板子指纹（与 car_logcat.py 同源） ----------------

def strip_proxy_env() -> None:
    """清掉代理环境变量。

    本机全局配了 HTTP(S)_PROXY=127.0.0.1:7897，而 websockets 15+ 会**默认读环境变量走代理**
    ——那会让发往局域网小车(192.168.x.x)的连接被塞进代理，直接连不上。这里统一清掉。
    """
    for k in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "http_proxy", "https_proxy", "all_proxy"):
        os.environ.pop(k, None)


def normalize_host(s: str) -> str:
    """接受 1.2.3.4 / ws://1.2.3.4:81/ / 主机名，统一成裸主机。"""
    s = s.strip()
    s = re.sub(r"^wss?://", "", s)
    s = s.split("/", 1)[0]
    if ":" in s and not s.startswith("["):
        s = s.rsplit(":", 1)[0]
    return s


def local_subnets() -> list[str]:
    """本机非回环 IPv4 所在的 /24 前缀（用于扫描候选）。"""
    addrs: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addrs.add(info[4][0])
    except OSError:
        pass
    try:  # 兜底：探默认出口地址（UDP connect 不发包，只让内核选路）
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            addrs.add(s.getsockname()[0])
    except OSError:
        pass
    out = []
    for a in addrs:
        if a.startswith(("127.", "169.254.")):
            continue
        pre = a.rsplit(".", 1)[0]
        if pre not in out:
            out.append(pre)
    return out


def probe_board(host: str, timeout: float = 1.0) -> bool:
    """是不是本项目的板子？

    板子的 ws_handler 对"非 WS 升级的 GET /"会 302 到 /stream，拿这个当指纹：81 端口上别的
    服务不会给出这条 302。板子卡死（WiFi/IP 死）时这里返回 False——正是要能分辨的那种状态。

    ⚠️ 必须读到报文头结束（`\\r\\n\\r\\n`）才算数，不能只 `recv(512)` 一次：板端是**逐段**把
    响应写出去的（实测首个 TCP 段只到 `Content-Length: 0\\r\\n` 共 64 字节，`Location: /stream`
    落在第二段），单次 recv 拿到哪一段全看时序 —— 只读一次会得出"这不是板子"的假结论，
    扫描会把唯一的板子整个滤掉（真机上已踩过）。
    """
    try:
        with socket.create_connection((host, WS_PORT), timeout=timeout) as s:
            s.settimeout(timeout)
            s.sendall(f"GET / HTTP/1.1\r\nHost: {host}\r\nConnection: close\r\n\r\n".encode())
            data = b""
            while len(data) < 512:
                chunk = s.recv(512 - len(data))
                if not chunk:            # 对端关连接：响应到此为止
                    break
                data += chunk
                if b"\r\n\r\n" in data:  # 报文头收完，后面的正文用不着
                    break
    except OSError:
        return False
    head = data.split(b"\r\n", 1)[0]
    return b" 302" in head and b"/stream" in data


def _port_open(host: str, timeout: float) -> bool:
    try:
        with socket.create_connection((host, WS_PORT), timeout=timeout):
            return True
    except OSError:
        return False


def scan_boards(timeout: float = 0.25) -> list[str]:
    """扫本机所在各 /24 的 81 端口，再用 302 指纹确认是板子。"""
    subs = local_subnets()
    if not subs:
        return []
    hosts = [f"{pre}.{i}" for pre in subs for i in range(1, 255)]
    log(f"[扫描] 网段 {', '.join(s + '.0/24' for s in subs)} 的 {WS_PORT} 端口…")
    with ThreadPoolExecutor(max_workers=128) as ex:
        opens = [h for h, ok in zip(hosts, ex.map(lambda h: _port_open(h, timeout), hosts)) if ok]
    return [h for h in opens if probe_board(h, timeout=max(timeout, 0.8))]


def resolve_host(args) -> str:
    """定位板子地址：命令行 → 上次成功的缓存 → 扫网段。"""
    if args.host:
        h = normalize_host(args.host)
        if not probe_board(h):
            log(f"[注意] {h} 上没探到板子指纹（81 端口没回 302 /stream）——仍按你给的地址试")
        return h
    if HOST_CACHE.exists():
        cached = HOST_CACHE.read_text(encoding="utf-8").strip()
        if cached and probe_board(cached):
            log(f"[定位] 用上次的地址 {cached}")
            return cached
        if cached:
            log(f"[定位] 缓存地址 {cached} 现在探不到板子（没上电？卡死？换网了？）")
    if args.no_scan:
        sys.exit(f"没有可用地址：缓存 {HOST_CACHE} 无效，且指定了 --no-scan（用 -H 指定板子 IP）")
    found = scan_boards()
    if not found:
        sys.exit("没扫到板子：没上电 / 不在同一网段 / 已卡死到 IP 都不通（后者只能现场断电）")
    if len(found) > 1:
        log(f"[定位] 扫到多块：{', '.join(found)}；用第一块，要指定请加 -H")
    return found[0]


def remember_host(host: str) -> None:
    try:
        HOST_CACHE.parent.mkdir(parents=True, exist_ok=True)
        HOST_CACHE.write_text(host, encoding="utf-8")
    except OSError:
        pass          # 缓存写不进去不影响主流程


# ---------------- HTTP：/update 的读（运行信息）与写（推固件） ----------------

def http_get(host: str, path: str, timeout: float = HTTP_TIMEOUT_S, port: int = HTTP_PORT) -> bytes:
    """GET 板端 HTTP。显式关掉代理：本机 127.0.0.1:7897 会把局域网请求截胡。"""
    req = urllib.request.Request(f"http://{host}:{port}{path}",
                                 headers={"Cache-Control": "no-store"})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=timeout) as r:
        return r.read()


def _strip_tags(s: str) -> str:
    return re.sub(r"<[^>]+>", "", s).strip()


def parse_runtime(html: str) -> dict:
    """从 /update 页面里抠出"此刻板上是什么"那一段。

    板端 runtime_info_html 现算这段（不是开机快照），所以刷一次页面拿到的就是当前状态。
    分区表只有一个 app 槽时它会改吐一段 warn（见下），那种情况下 OTA 根本无处可写。
    """
    out: dict = {}
    m = re.search(r'<p class=rt>(.*?)</p>', html, re.S)
    if m:
        out["block"] = _strip_tags(m.group(1).replace("<br>", "\n"))
    if m := re.search(r"当前运行：<b>(.*?)</b>", html):
        out["running_stamp"] = _strip_tags(m.group(1))
    if m := re.search(r"运行分区 <code>(.*?)</code> @(0x[0-9a-fA-F]+)", html):
        out["running_part"], out["running_addr"] = m.group(1), m.group(2)
    if m := re.search(r"下次 OTA 写入 <code>(.*?)</code> @(0x[0-9a-fA-F]+)（(\d+) KB）", html):
        out["next_part"], out["next_addr"], out["next_kb"] = m.group(1), m.group(2), int(m.group(3))
    if m := re.search(r"该槽现有固件：(.*?)</p>", html, re.S):
        out["next_stamp"] = _strip_tags(m.group(1))
    out["warn"] = "找不到第二个 app 槽" in html
    return out


def fingerprint_of(text: str) -> str | None:
    """从 "v3 指纹 f8e6f35f" 这类串里取 8 位指纹。"""
    m = re.search(r"指纹\s*([0-9a-fA-F]{8})", text or "")
    return m.group(1).lower() if m else None


def bin_fingerprint(path: Path) -> str | None:
    """读 .bin 自己的指纹（偏移 0xB0 起 4 字节）。不是 ESP32 app 镜像则返回 None。

    这与板端 fp_hex 读的是同一个字段（esptool 打包时按 --elf-sha256-offset 0xb0 写入），
    所以可以直接跟板子报出来的指纹逐字比。
    """
    try:
        with open(path, "rb") as f:
            head = f.read(4)
            if len(head) < 4 or head[0] != 0xE9:      # 0xE9 = ESP32 app 镜像的 magic
                return None
            f.seek(BIN_FP_OFFSET)
            fp = f.read(4)
    except OSError:
        return None
    return fp.hex() if len(fp) == 4 else None


def list_bins() -> list[Path]:
    """本机 Arduino 编译缓存里的候选 .bin，按修改时间新→旧。"""
    if not SKETCH_CACHE.is_dir():
        return []
    bins = list(SKETCH_CACHE.glob("*/Stm32-Vision.ino.bin"))
    return sorted(bins, key=lambda p: p.stat().st_mtime, reverse=True)


# ---------------- 固件归档：按指纹留住每份 .elf ----------------
# arduino-cli 把同一个 sketch 的 .elf **原地覆盖**在缓存目录里（同一个 sketch+fqbn 只对应一个
# 缓存哈希），一编新的一来，上一份就没了。可 panic 现场偏偏只能用它解——backtrace 里的 PC 要有
# **配对的** .elf 才能变成文件:行号。所以每编一份就按指纹留一份。
#
# 指纹 = app 镜像头里那 4 字节 = `app_elf_sha256` 的前 4 字节 = elf 文件 sha256 的前 8 位
# （已实测对上：elf sha256 = 696cc4657088…，板子 status 里也显示 696cc465）。所以归档目录名
# 就是板上显示的指纹，coredump 里回传的 sha 也能直接反查到这里。
# 放 LOCALAPPDATA（机器本地）：换机器本来也得重编，归档跟着走没意义；22MB/份，不入库。
FW_ARCHIVE = SKETCH_CACHE.parent / "carctl-fw"


def elf_sha256(p: Path) -> str | None:
    import hashlib
    try:
        h = hashlib.sha256()
        with open(p, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        return h.hexdigest()
    except OSError:
        return None


def find_addr2line() -> Path | None:
    """xtensa 版 addr2line。版本目录（esp-x32/<ver>/bin）随工具链升级变，故用 glob。"""
    base = (Path(os.environ.get("LOCALAPPDATA", "")) / "Arduino15" / "packages" / "esp32"
            / "tools" / "esp-x32")
    for pat in ("*/bin/xtensa-esp32s3-elf-addr2line.exe", "*/bin/xtensa-esp32-elf-addr2line.exe"):
        hit = sorted(base.glob(pat))
        if hit:
            return hit[-1]
    return None


def resolve_elf(sha: str | None, forced: str | None) -> tuple[Path | None, str]:
    """按 coredump 回传的 app elf sha256 找配对 .elf。返回 (路径, 说明)。

    ⚠️ 板子回传的 sha 是**截断**的：`CONFIG_APP_RETRIEVE_LEN_ELF_SHA=9` ⇒ 只有 9 个十六进制
    字符（板子 status 里显示的指纹就是它）。所以这里只能**前缀匹配**，拿它跟完整的 64 位比
    永远不等 —— 我第一版就是这么写的，结果明明归档在手却报"找不到 .elf"。
    """
    if forced:
        p = Path(forced)
        return (p, "指定") if p.is_file() else (None, f"指定的 {p} 不存在")
    sha = (sha or "").strip().lower()
    if len(sha) < 8:
        return None, f"coredump 里的 sha 太短（{sha!r}），无法反查归档（用 --elf 指定）"
    if not FW_ARCHIVE.is_dir():
        return None, f"归档目录还不存在：{FW_ARCHIVE}（归档功能是后加的，之前编的没存）"
    # 先按目录名（= 指纹 = sha 前 8 位）直接命中，再退化成全部前缀比对
    cand = FW_ARCHIVE / sha[:8] / "Stm32-Vision.ino.elf"
    if cand.is_file() and (elf_sha256(cand) or "").startswith(sha):
        return cand, "指纹目录命中"
    for p in sorted(FW_ARCHIVE.glob("*/Stm32-Vision.ino.elf")):
        if (elf_sha256(p) or "").startswith(sha):
            return p, f"前缀比对命中（{p.parent.name}）"
    return None, (f"归档里没有这份固件的 .elf（sha={sha}）；裸地址仍可看，但解不出文件:行号。"
                  f"归档里现有：{', '.join(sorted(d.name for d in FW_ARCHIVE.iterdir() if d.is_dir()))}")


# Xtensa 异常原因表：逐字抄自 esp-idf `components/esp_system/port/arch/xtensa/panic_arch.c`
# 的 `reason[]`（连 "res" 占位都对上了号，索引即 exccause，别自己重排）。
XTENSA_CAUSE = [
    "IllegalInstruction", "Syscall", "InstructionFetchError", "LoadStoreError",
    "Level1Interrupt", "Alloca", "IntegerDivideByZero", "PCValue",
    "Privileged", "LoadStoreAlignment", "res", "res",
    "InstrPDAddrError", "LoadStorePIFDataError", "InstrPIFAddrError", "LoadStorePIFAddrError",
    "InstTLBMiss", "InstTLBMultiHit", "InstFetchPrivilege", "res",
    "InstrFetchProhibited", "res", "res", "res",
    "LoadStoreTLBMiss", "LoadStoreTLBMultihit", "LoadStorePrivilege", "res",
    "LoadProhibited", "StoreProhibited", "res", "res",
    "Cp0Dis", "Cp1Dis", "Cp2Dis", "Cp3Dis",
    "Cp4Dis", "Cp5Dis", "Cp6Dis", "Cp7Dis",
]


def cause_name(c: int | None) -> str:
    if c is None:
        return "?"
    return XTENSA_CAUSE[c] if 0 <= c < len(XTENSA_CAUSE) else f"Unknown({c})"


# ---------------- WS：连接、收帧、发指令 ----------------

def fmt_bits(bits: int) -> str:
    on = [n for i, n in enumerate(BIT_NAMES) if bits >> i & 1]
    return f"0x{bits:02x} " + ("/".join(on) if on else "全关")


def fmt_frame(m: dict, show_log: bool, show_pong: bool) -> str | None:
    """板端一帧 → 一行人话。返回 None 表示这帧按开关应当吞掉。"""
    t = m.get("type")
    p = m.get("params") if isinstance(m.get("params"), dict) else {}
    if t == "log":
        if not show_log:
            return None
        src = p.get("src") or "log"
        return f"[{src}] {p.get('text', '')}"
    if t == "status":
        bits = p.get("bits")
        tail = f"   (bits {fmt_bits(bits)})" if isinstance(bits, int) else ""
        return f"[回复] {p.get('reason', '')}{tail}"
    if t == "state":
        bits = p.get("bits")
        return f"[状态] bits {fmt_bits(bits) if isinstance(bits, int) else '?'}"
    if t == "pong":
        return "[pong] 链路活着" if show_pong else None
    if t == "ping":
        return None                                   # 板端探测帧：上面已回 pong，不必刷屏
    return f"[{t}] {json.dumps(p, ensure_ascii=False)}"


def ws_open(host: str, timeout: float):
    """连板端 WS。

    ping_interval=None 是必须的：websockets 默认每 20s 发 WS 控制帧 PING 并等 PONG，而板端是
    handle_ws_control_frames=false（控制帧不一定回），库会误判超时后自己断开。活体改走板端自己
    的文本协议 {"type":"ping"} ↔ {"type":"pong"}（与手机端一致）。
    """
    return connect(f"ws://{host}:{WS_PORT}/", ping_interval=None, open_timeout=timeout,
                   close_timeout=2, max_size=2 ** 22)


def pump(ws, seconds: float, show_log: bool = True, show_pong: bool = False,
         keepalive: bool = False) -> int:
    """收帧 seconds 秒，边收边打印；板端 ping 一律回 pong。返回收到的帧数。

    keepalive=True 时按 PING_EVERY_S 主动发 {"type":"ping"}（长驻的 log 模式要它维持链路）。
    """
    got = 0
    end = time.monotonic() + seconds
    last_ping = time.monotonic()
    while True:
        left = end - time.monotonic()
        if left <= 0:
            return got
        if keepalive and time.monotonic() - last_ping >= PING_EVERY_S:
            last_ping = time.monotonic()
            try:
                ws.send('{"type":"ping"}')
            except (OSError, ConnectionClosed):
                return got
        try:
            # 收包粒度取小值：这样 keepalive 与 Ctrl-C 都不会被一个长阻塞拖住
            raw = ws.recv(timeout=min(left, 0.5))
        except TimeoutError:
            continue
        except (OSError, ConnectionClosed) as e:
            log(f"[断开] {type(e).__name__}: {e}")
            return got
        got += 1
        if isinstance(raw, (bytes, bytearray)):
            log(f"[二进制帧] {len(raw)} 字节（图传/编辑图，非文本）")
            continue
        try:
            m = json.loads(raw)
        except ValueError:
            log(f"[原始] {raw}")
            continue
        if not isinstance(m, dict):
            log(f"[原始] {raw}")
            continue
        if m.get("type") == "ping":        # 板端探测：回一条（板端把任何上行都算活动）
            try:
                ws.send('{"type":"pong"}')
            except (OSError, ConnectionClosed):
                return got
            continue
        if line := fmt_frame(m, show_log, show_pong):
            log(line)


def coerce(v: str):
    """命令行的 k=v 值转成 JSON 该有的类型：1 → int，1.5 → float，true → bool，其余留字符串。"""
    if v.lower() in ("true", "false"):
        return v.lower() == "true"
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        pass
    return v


def send_and_wait(host: str, payload: str, args, wait_s: float, label: str,
                  show_pong: bool = False) -> int:
    """连一次、发一条、收 wait_s 秒、断开。cmd/raw/ping/reboot 共用这条路。

    show_pong：无目标的 ping 回的就是 pong，那是它**唯一**的输出，必须显出来（默认静音是给
    别的子命令用的：那些场景下 pong 只是保活回声，刷屏没用）。返回收到的帧数。
    """
    log(f"[发出] {payload}")
    with ws_open(host, args.timeout) as ws:
        ws.send(payload)
        n = pump(ws, wait_s, show_log=not args.no_log, show_pong=show_pong or args.verbose)
    if n == 0:
        log(f"[注意] {wait_s:g}s 内板端一条都没回——链路卡住 / 板子正忙 / 指令被闸门挡了？")
    log(f"[完成] {label}")
    return n


# ---------------- 子命令 ----------------

def cmd_status(host: str, args) -> None:
    """板上现状：运行分区 + 固件指纹 + OTA 落点（HTTP），外加一次状态位（WS）。"""
    log(f"[板子] {host}  (HTTP {HTTP_PORT} / WS {WS_PORT})")
    try:
        html = http_get(host, "/update").decode("utf-8", "replace")
    except (urllib.error.URLError, OSError, TimeoutError) as e:
        sys.exit(f"读 /update 失败：{type(e).__name__}: {e}\n"
                 f"（HTTP 不通但 WS 可能还活着——先试 `carctl.py log`；都不通就 `carctl.py reboot` 也发不进去，只能现场断电）")
    info = parse_runtime(html)
    if info.get("block"):
        for line in info["block"].splitlines():
            log("  " + line)
    else:
        log("  （页面里没找到运行信息块：固件比本脚本旧？）")
    if info.get("warn"):
        log("  ⚠ 分区表只有一个 app 槽，OTA 无处可写")
    # 顺手问一次状态位：能答上来说明 WS 也活着（两条链路各自独立，分开报才分得清是哪条断）
    try:
        with ws_open(host, args.timeout) as ws:
            ws.send('{"type":"get_state"}')
            pump(ws, 3.0, show_log=False)
    except (OSError, ConnectionClosed, TimeoutError) as e:
        log(f"  [WS 不通] {type(e).__name__}: {e}")


def cmd_log(host: str, args) -> None:
    """实时日志：开板端转发 → 收 → 退出时按需关掉。"""
    want = json.dumps({"type": "log", "params": {"cat": args.cat, "on": True}})
    off = json.dumps({"type": "log", "params": {"cat": args.cat, "on": False}})
    with ws_open(host, args.timeout) as ws:
        ws.send(want)
        dur = "直到 Ctrl-C" if args.duration <= 0 else f"{args.duration:g}s"
        log(f"[日志] cat={args.cat}，{dur}（{'保留' if args.leave_on else '退出时关闭'}转发）")
        try:
            if args.duration > 0:
                pump(ws, args.duration, show_pong=args.verbose, keepalive=True)
            else:
                # 不退避地一直收：Ctrl-C 走 KeyboardInterrupt，由下面的 finally 收尾
                while True:
                    pump(ws, REASSERT_S, show_pong=args.verbose, keepalive=True)
                    ws.send(want)     # 周期重申：手机会改掉这个全局单选开关
        except KeyboardInterrupt:
            log("\n[中断] 停止收日志")
        finally:
            # 必须放在 finally：Ctrl-C 也走这条路。转发是**全局**开关，留着不管就会被手机的
            # 日志页面一直刷屏（板端并不知道是我这边不看了）。
            if not args.leave_on:
                try:
                    ws.send(off)
                    pump(ws, 0.5, show_log=False)     # 留一拍让板端收下再关
                except (OSError, ConnectionClosed):
                    pass                # 板子已经断了：日志开关本来就是全局的，下次连上重设即可
    log("[日志] 已退出")


def cmd_ping(host: str, args) -> None:
    """不带目标 = 测板子在不在（回 pong）；带目标 = 让板子去 ICMP 探测。"""
    if not args.target:
        # 回的就是 pong：显出来，"板子在不在"的答案就在这一行
        n = send_and_wait(host, '{"type":"ping"}', args, 3.0, "连通性探测", show_pong=True)
        if n == 0:
            log(f"[结论] {host} 的 WS 没回 pong —— 板子卡死或不在这个地址")
        return
    # 板端 5 次探测、每次超时 3s，域名还要先解析：窗口给足，否则会在结果出来前就退出
    payload = json.dumps({"type": "ping", "params": {"target": args.target}})
    send_and_wait(host, payload, args, args.wait, f"板子 ping {args.target}")


def cmd_reboot(host: str, args) -> None:
    """远程重启。板端回一条 status，同一句另经日志转发广播出去。"""
    send_and_wait(host, '{"type":"reboot"}', args, args.wait, "重启指令已发")
    if not args.wait_up:
        return
    log("[等待] 等板子掉线再回来…")
    if wait_down(host, 20.0) is None:
        log("  （没观察到掉线：可能早已重启完，或这条指令根本没送达）")
    up = wait_board_up(host, args.wait_up)
    if up is None:
        log(f"[重启] {args.wait_up:g}s 内板子没回来——没重启成，或重启后没连上 WiFi")
    else:
        log(f"[重启] 板子 {up:.0f}s 后回来了")


def wait_down(host: str, timeout_s: float) -> float | None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if not probe_board(host, 1.0):
            return time.monotonic() - t0
        time.sleep(0.5)
    return None


def wait_board_up(host: str, timeout_s: float) -> float | None:
    """等板子回来：WS 指纹 + HTTP 都得通（只探到 WS 可能还在开机早期）。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if probe_board(host, 1.5):
            try:
                http_get(host, "/update", timeout=4.0)
                return time.monotonic() - t0
            except (urllib.error.URLError, OSError, TimeoutError):
                pass
        time.sleep(1.0)
    return None


def cmd_build(args) -> Path | None:
    """编译草图，返回新产出的 .bin（失败返回 None）。加 --flash 就直接推上板。

    fqbn 写死在这里的理由：它必须与 IDE 里逐字一致（**缓存目录是按 fqbn 哈希分的**，
    差一个字就是另一份缓存 → 全量重编，还查不出原因），而这条命令长到没人能凭记忆打对。
    """
    if not ARDUINO_CLI.is_file():
        sys.exit(f"找不到 arduino-cli：{ARDUINO_CLI}\n"
                 "（已按 环境变量 ARDUINO_CLI → PATH → IDE 安装目录 依次找过。"
                 "请显式指定：`ARDUINO_CLI=<arduino-cli 全路径>` 或先把它加进 PATH。）")
    t0 = time.time()
    cmd = [str(ARDUINO_CLI), "compile", "--fqbn", FQBN]
    if args.clean:
        # --clean 交给 arduino-cli 自己清构建目录：比 rm -rf <缓存hash> 稳，
        # 不用知道那个哈希是怎么算的。**改/增/删源文件、或改了内核 sdkconfig.h 后必须加它**
        # ——后者在 74 个文件的 .d 依赖里，增量构建不保证认这个变更（静默不生效）。
        cmd.append("--clean")
    cmd.append(".")
    log(f"[编译] fqbn={FQBN}")
    if args.clean:
        log("[编译] --clean：全量重编（慢，但换来「改动确实进了固件」）")
    r = subprocess.run(cmd, cwd=str(SKETCH_DIR), capture_output=True, text=True,
                       errors="replace")
    out = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0:
        log("[编译] 失败，末尾输出：")
        for line in out.strip().splitlines()[-40:]:
            log("  " + line)
        return None
    # 成功时只留有用几行：占用统计 + 产物路径。arduino-cli 的完整输出是几十行编译进度
    for line in out.splitlines():
        s = line.strip()
        if s.startswith(("Sketch uses", "Global variables use", "Leaving", "Used platform")):
            log("  " + s)

    # 优先认"这次编出来的"，而不是"缓存里最新的"：缓存目录里可能躺着上一次的 .bin，
    # 照它推就等于推了旧固件、还核不出来。
    fresh = [p for p in list_bins() if p.stat().st_mtime >= t0 - 1]
    if fresh:
        bin_path = fresh[0]
    else:
        # 源码无变化时 arduino-cli 会跳过链接、不重写 .bin —— 这不是失败，但也**不能默认**
        # 拿旧的顶替（那正是上面要防的事），所以显式说明来历再交出去。
        cached = list_bins()
        if not cached:
            log(f"[编译] 退出码 0 但缓存里也没有 .bin：{SKETCH_CACHE}")
            return None
        bin_path = cached[0]
        stamp = datetime.fromtimestamp(bin_path.stat().st_mtime).strftime("%m-%d %H:%M")
        log(f"[产物] 本次没有新产出 .bin（源码未变时 arduino-cli 会跳过链接）——"
            f"用缓存里最新的那份，产于 {stamp}")
    fp = bin_fingerprint(bin_path)
    log(f"[产物] {bin_path}")
    log(f"       {bin_path.stat().st_size / 1024 / 1024:.2f}MB  指纹 {fp}  耗时 {time.time() - t0:.0f}s")
    archive_build(bin_path, fp)
    return bin_path


def archive_build(bin_path: Path, fp: str | None) -> None:
    """把这次的 .elf/.bin 按指纹存一份到 FW_ARCHIVE（理由见 FW_ARCHIVE 上方注释）。

    指纹相同就跳过：指纹是镜像内容哈希，同指纹 ⇒ 同一份字节，重拷没意义（22MB）。
    """
    if not fp:
        log("[归档] 读不出指纹，跳过（.elf 仍会随下次编译被覆盖）")
        return
    dst = FW_ARCHIVE / fp
    if (dst / "Stm32-Vision.ino.elf").is_file():
        log(f"[归档] 已有 {fp}，跳过")
        return
    elf = bin_path.with_suffix("").with_suffix(".ino.elf")   # Stm32-Vision.ino.bin → .ino.elf
    if not elf.is_file():
        log(f"[归档] 没找到配对的 .elf（{elf.name}），只存 .bin")
    try:
        dst.mkdir(parents=True, exist_ok=True)
        import shutil
        shutil.copy2(bin_path, dst / bin_path.name)
        if elf.is_file():
            shutil.copy2(elf, dst / elf.name)
        log(f"[归档] {dst}（含 .elf：panic 现场要靠它解）")
    except OSError as e:
        log(f"[归档] 失败（不影响出固件）：{e}")


def cmd_coredump(host: str, args) -> None:
    """取回 panic 现场，并配 .elf 解成 文件:行号。

    为什么值得有这条命令：板子跑挂后网络一起没了、串口也不在手边，`复位=崩溃` 四个字就是全部
    线索。现场其实一直落在 coredump 分区里（partitions.csv 那 512KB），但分区开着没通路等于没开。
    现在 `/coredump` 把它取出来（原因串 + 崩在哪个任务 + 16 层回溯 PC），这里再配**同一份固件**
    的 .elf 做 addr2line —— 配对靠 coredump 回传的 app elf sha256，它就是板上显示的指纹。
    """
    q = "?erase=1" if args.erase else ""
    try:
        txt = http_get(host, "/coredump" + q).decode("utf-8", "replace")
    except (urllib.error.URLError, OSError, TimeoutError) as e:
        sys.exit(f"取 /coredump 失败：{type(e).__name__}: {e}\n"
                 f"（HTTP 不通：先 `carctl.py status` 看板子活没活；固件比本脚本旧也会 404）")
    if "erase=" in txt:
        log(f"[现场] 已清空：{txt.strip()}（panic 一次覆写一次，清了就没了）")
        return

    kv = dict(re.findall(r"^(\w+)=(.*)$", txt, re.M))
    ok = kv.get("ok") == "1"
    log(f"[现场] {'有' if ok else '没有可读的 coredump'}"
        + ("" if ok else f"（reason_err={kv.get('reason_err')} summary_err={kv.get('summary_err')}）"))
    if kv.get("reason"):
        log(f"  原因: {kv['reason']}")
    if not ok:
        log("  没有现场通常是：上电后还没崩过、或上一次已取走并 erase 了。")
        return
    # 留档：coredump 分区只有一份，下一次 panic 就覆写；而这段文本是事后唯一能复盘的东西。
    # 存进归档目录（与配对 .elf 同处），文件名带时间戳。
    try:
        arch = FW_ARCHIVE / (kv.get("sha") or "unknown")
        arch.mkdir(parents=True, exist_ok=True)
        p = arch / f"panic-{datetime.now():%Y-%m-%d-%H%M%S}.txt"
        p.write_text(txt, encoding="utf-8")
        log(f"  已留档: {p}")
    except OSError as e:
        log(f"  （留档失败，不影响判读：{e}）")
    # 按字段名抓（不按整行 key=value）：cause 与 vaddr 印在同一行，按行拆会串味。
    def g(pat, s=txt):
        m = re.search(pat, s)
        return m.group(1) if m else None
    cause = int(g(r"cause=(\d+)")) if g(r"cause=(\d+)") else None
    vaddr = g(r"vaddr=(0x[0-9a-fA-F]+)")
    depth = g(r"depth=(\d+)")
    corrupt = g(r"corrupted=(\d+)")
    log(f"  任务: {kv.get('task')}   PC: {kv.get('pc')}")
    log(f"  异常: {cause_name(cause)}({cause})  访问地址: {vaddr}")
    # 判读提示：这三个数字组合起来基本就把"是什么错"说完了，别让人再去查表。
    if vaddr and int(vaddr, 16) < 0x1000:
        log(f"  ⇒ 访问的地址小到不像真地址（{vaddr}）：典型的**空指针 + 结构体偏移**解引用"
            f"（`p->field` 而 p=NULL）。不是内存不足，也不是栈溢出。")
    elif cause in (28, 29):
        log("  ⇒ 读/写了一个未映射或无权访问的地址：指针已损坏或指向已释放的对象。")
    log(f"  回溯: depth={depth} corrupted={corrupt}")
    pcs = re.findall(r"0x[0-9a-fA-F]{8}", kv.get("bt", ""))
    if not pcs:
        log("  （bt 为空：回溯没生成出来，只能看 PC 与原因串）")
        return

    elf, why = resolve_elf(kv.get("sha"), args.elf)
    if not elf:
        log(f"  ⚠ {why}")
        for i, pc in enumerate(pcs):
            log(f"    #{i:<2} {pc}")
        return
    log(f"  .elf: {elf.parent.name}/{elf.name}（{why}）")
    a2l = find_addr2line()
    if not a2l:
        log("  ⚠ 没找到 xtensa addr2line（Arduino15/packages/esp32/tools/esp-x32/*/bin）")
        return
    # -C 解 C++ 符号名；-i 展开内联帧；-p 一行一帧（比默认的成对输出好读，也好数）
    r = subprocess.run([str(a2l), "-C", "-i", "-p", "-e", str(elf)] + pcs,
                       capture_output=True, text=True, errors="replace")
    lines = [ln for ln in (r.stdout or "").splitlines() if ln.strip()]
    if r.returncode != 0 or not lines:
        log(f"  ⚠ addr2line 没输出：{(r.stderr or '').strip()[:200]}")
        return
    # 输出顺序与 pcs 一一对应，但展开内联后会多行 —— 按行打印，前面套上序号分块
    log("  ── 回溯（自上而下=由内到外）──")
    for ln in lines:
        log("    " + ln)
    log("  提示：`?? at ??:0` 多是 ROM 里的函数（不在本 .elf 内），属正常。")


# 风暴用的默认目标：**必须**带"别动"这两个字。小车电池上挂着充电线，轮子一转就把线拖住。
STRESS_GOAL = "不要移动小车、不要转圈、不要动机械臂。只看画面，用一句话说你看到了什么，然后立刻结束任务。"

# 体征探针：`复位=/堆=/最低=/DMA块最低=` 那一串**只**由 log 指令的回复带出来（command.cpp 的
# log 分支；get_state 只回 bits，别的指令的 reply_status 也不带）。而 log 是**幂等**的：要的
# 状态与当前相同时只回 "日志转发未变（当前: X）｜复位=…"，什么也不改 —— 正好当只读探针。
# ⚠️ 代价：若转发原本是开的，探针会把它关掉（回 "已关闭"），此时见下方还原逻辑。
VITALS_PROBE = '{"type":"log","params":{"on":false}}'


def cmd_stress(host: str, args) -> None:
    """abort 风暴：AI 轮次在途时反复发中止指令，专打跨任务竞态那个窗口。

    为什么要有它：`696cc465` 那次 panic（ai_worker 在 mbedtls 握手里解引用已被释放的 ssl_ctx，
    vaddr=0x8）成因是 http_stop() 从别的任务拆 g_client。修好没有，光靠"跑几轮 AI 没崩"证不了
    ——**得先能把它打出来，再看改完打不打得出**。

    触发路径（照源码，别照感觉）：
      · 手动类型指令（stop/ai_cancel…）→ command.cpp 的 `ai::cancel()` → http_stop()
      · 新目标（ai_goal/ai_oneshot）  → ai_client 的 `ai::set_goal()` → http_stop()
    两条都要求"上一轮请求还在途"，所以风暴是两条腿：一条不停起新轮次，一条不停发中止。

    ⚠️ 先看 `命中` 那个数：它是"回包时 bits 里带 AI忙碌"的次数，即中止**确实落在在途请求上**的
    次数。命中=0 说明 AI 根本没在忙、全打空了 —— 那次运行**什么也没证明**，不许当作"通过"。
    """
    AI_BUSY = 1 << 4                       # BIT_NAMES[4] = AI忙碌
    goal = json.dumps({"type": args.type, "params": {"message": args.goal, "use_image": True}},
                      ensure_ascii=False)
    # stop 走"手动类型"白名单分支，ai_cancel 走显式取消 —— 两条不同的调用点，交替发才都覆盖到。
    # 两条都不动轮子：stop 时车本来就没动（StopMode::None），ai_cancel 只清任务。
    aborts = ['{"type":"stop"}', '{"type":"ai_cancel"}']

    if not args.keep_panic:                # 不先清，事后就分不清现场是本轮崩的还是上次留的
        try:
            http_get(host, "/coredump?erase=1")
            log("[前置] 已清空旧 panic 现场（保证下面读到的只可能是本轮崩的）")
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            log(f"[前置] 清现场失败（固件太旧没有 /coredump？）：{type(e).__name__}: {e}")

    span = args.rounds * args.gap
    log(f"[风暴] {args.rounds} 轮 {args.type}（每 {args.gap:g}s）+ 中止每 {args.abort_ms:g}ms"
        f"（stop/ai_cancel 交替）· 预计 {span:.0f}s")
    t0 = time.monotonic()
    end = t0 + span
    # abort_ms=0 ⇒ 不发中止（只测"连跑多轮"）：**不能**写成 t0+0，那会让 next_abort 每轮迭代都
    # 落后于 now ⇒ 每个循环都发一条，等于自己造一场洪水（刚诊断过一次这类洪水把板子压死）。
    next_goal = t0
    next_abort = (t0 + args.abort_ms / 1000.0) if args.abort_ms > 0 else float("inf")
    # 风暴进行中定期采体征：**这是分层的唯一依据**。上次卡死时手里没有"死之前的堆水位"，
    # 于是分不清是内部堆触底（`堆/最低/DMA块最低`）还是 PCB 池耗尽（`套接字余`）——
    # 看 `DMA块最低` 而不是 `堆=`，碎片化才是真闸门。
    next_sample = (t0 + args.sample) if args.sample > 0 else float("inf")
    rounds = sent = hits = frames = 0
    vitals = ""
    died = None                    # 非 None = WS 异常断开（板子掉了），值是当时的异常
    asked, wait_dl = False, 0.0
    with ws_open(host, args.timeout) as ws:
        while True:
            now = time.monotonic()
            try:
                if now >= end:
                    if not asked:              # 收尾：**只问一次**（早先版本每轮迭代都发，
                        asked, wait_dl = True, now + 8.0   # 等于每秒几十条，把板子和回包全冲了）
                        ws.send('{"type":"get_state"}')
                    elif vitals or now >= wait_dl:
                        break
                else:
                    if now >= next_goal and rounds < args.rounds:
                        next_goal += args.gap
                        rounds += 1
                        ws.send(goal)
                        log(f"  [{rounds:>2}/{args.rounds}] {now - t0:6.1f}s 起一轮")
                    if now >= next_abort:
                        next_abort += args.abort_ms / 1000.0
                        ws.send(aborts[sent % len(aborts)])
                        sent += 1
                    if now >= next_sample:      # 采一次体征（下面回包处打时间序列）
                        next_sample = now + args.sample
                        ws.send(VITALS_PROBE)
                raw = ws.recv(timeout=0.02)
            except TimeoutError:
                continue
            except (OSError, ConnectionClosed) as e:
                # 这不是"出错"，这是**测试结果**：整板 panic→重启会把 TCP 直接掐掉，
                # 对端收不到 close 帧。所以只记下来，继续走查现场那条路（别 sys.exit）。
                died = e
                log(f"[断开] {now - t0:6.1f}s WS 断了且**没有 close 帧**"
                    f"（{type(e).__name__}）—— 正常收工会送 close 帧，这是整板重启/panic 的签名")
                break
            if isinstance(raw, (bytes, bytearray)):
                continue
            frames += 1                        # 原始收帧数：没它就没法区分"没帧"和"帧里没那个位"
            try:
                m = json.loads(raw)
            except ValueError:
                continue
            if not isinstance(m, dict):
                continue
            if m.get("type") == "ping":
                try:
                    ws.send('{"type":"pong"}')
                except (OSError, ConnectionClosed) as e:
                    died = e
                    break
                continue
            p = m.get("params") if isinstance(m.get("params"), dict) else {}
            b = p.get("bits")
            if isinstance(b, int) and b & AI_BUSY:
                hits += 1                      # 这一发确实落在"AI 在途"上 ⇒ 算一次真命中
            txt = str(p.get("reason") or p.get("text") or "")
            if "已关闭" in txt:
                # 探针把本来开着的转发关掉了：马上还原。只能还成 all —— 回包里没带上一个 cat。
                log("  ⚠ 探针发现日志转发原本是开的（已关掉），还原为 all（原类别未回传，无法精确还原）")
                try:
                    ws.send('{"type":"log","params":{"cat":"all","on":true}}')
                except (OSError, ConnectionClosed):
                    pass
            if "复位=" in txt:
                vitals = txt
                log(f"  [体征 {now - t0:5.1f}s] {txt}")
            # 打断类回包不逐条刷屏：风暴里几百条，全打出来没信息量（命中数已由 hits 计着）

    # 汇总**必须在查现场之前**打：板子崩了正是不该省掉这段的时候（早先版本异常直接冒泡，
    # 结果连命中数都没打出来，等于这轮白跑）。
    log(f"[风暴完] 起轮 {rounds} 次 · 发中止 {sent} 次 · **命中在途 {hits} 次** · 收帧 {frames} 条"
        f"{' · 板子中途掉了' if died else ''}")
    if frames == 0:
        log("  ⚠ 收帧 0：**一条回包都没收到**，这轮什么都没测到（不是「AI 不忙」而是「听不见」）。"
            "先 `carctl.py cmd get_state` 确认回包通路，再重跑。")
    elif sent == 0:
        # --abort-ms 0 = 故意不发中止，这轮只跑正常循环（验证 AI 闭环本身）。
        # 此时"命中 0"是设计如此，不是覆盖度不足——别报成警告，否则使用者会去调 --gap 白折腾。
        log("  （本轮未发中止：--abort-ms 0 ⇒ 只验证正常 AI 闭环，不看打断路径）")
    elif hits == 0:
        log("  ⚠ 命中 0：有回包但都没带 AI忙碌 ⇒ 中止全打在空处，这轮**没有验证效果**——"
            "把 --gap 调大（让轮次真的重叠）或 --abort-ms 调小再跑一次。")
    if vitals and args.sample <= 0:      # 开了采样的话上面已按时间序列打过，别重复
        log(f"[体征] 末次: {vitals}")
    elif not vitals and not died:
        log("[体征] 没等到体征回复（板子忙 / 帧被吞）")

    # 崩了就是崩了：等它重启回来再读现场，否则会把"网络还没起来"误判成"没崩"。
    up = wait_board_up(host, args.wait_up)
    if up is None:
        log(f"[结果] 等 {args.wait_up:g}s 板子也没回来。")
        if died:
            log("  ⇒ 崩溃后没自愈（WiFi 没起来）。`carctl.py doctor` 走 BLE 敲一次 reboot，"
                "或现场断电；起来后第一件事就是 `carctl.py coredump` 取现场。")
        else:
            log("  ⇒ 风暴中没见异常断开，但板子也不通：可能是网络侧（PC/路由器）问题，"
                "先 `carctl.py doctor` 分层确认。")
        sys.exit(1)
    log(f"[恢复] 板子 {up:.0f}s 后回来了")
    log("[现场] —— 下面若有 panic，且任务/回溯与预期一致，就说明这轮**复现成功** ——")
    cmd_coredump(host, argparse.Namespace(erase=False, elf=None))


def cmd_fw(args) -> None:
    """读 .bin 指纹。不带参数就列本机编译缓存里的候选。"""
    paths = [Path(p) for p in args.bin] if args.bin else list_bins()
    if not paths:
        sys.exit(f"没找到 .bin：{SKETCH_CACHE} 下没有 */Stm32-Vision.ino.bin（先在 IDE 里编译一次？）")
    for p in paths:
        fp = bin_fingerprint(p)
        if not p.exists():
            log(f"  {p}  —— 文件不存在")
            continue
        size = p.stat().st_size
        stamp = datetime.fromtimestamp(p.stat().st_mtime).strftime("%m-%d %H:%M")
        log(f"  {fp or '??（不是 app 镜像 / 太短）'}  {size / 1024 / 1024:5.2f}MB  {stamp}  {p}")


def cmd_ota(host: str, args) -> None:
    """推固件 → 等板子重启回来 → 核对板上跑的指纹 == 刚推的那份。"""
    path = Path(args.bin)
    if not path.is_file():
        sys.exit(f"找不到文件：{path}")
    data = path.read_bytes()
    fp = bin_fingerprint(path)
    if fp is None:
        sys.exit(f"{path} 不像 ESP32 app 镜像（首字节非 0xE9 或不足 0xC0 字节），拒绝上传")
    log(f"[固件] {path}")
    log(f"        {len(data) / 1024 / 1024:.2f}MB  指纹 {fp}")

    try:
        before = parse_runtime(http_get(host, "/update").decode("utf-8", "replace"))
    except (urllib.error.URLError, OSError, TimeoutError) as e:
        sys.exit(f"推之前读 /update 失败（板子不通就别推了）：{type(e).__name__}: {e}")
    log(f"[板上] 跑 {before.get('running_stamp', '?')}（分区 {before.get('running_part', '?')}）")
    if before.get("warn"):
        sys.exit("板端说找不到第二个 app 槽——分区表是单槽方案，OTA 无处可写")
    if before.get("next_part"):
        log(f"[落点] {before['next_part']} @{before.get('next_addr', '?')}"
            f"（{before.get('next_kb', '?')} KB）现有 {before.get('next_stamp', '?')}")
    if fingerprint_of(before.get("running_stamp", "")) == fp:
        log("[注意] 板上跑的已经是这份固件（指纹相同）——继续推会重写一遍，通常没意义")
        if not args.force:
            sys.exit("已中止（要强推请加 --force）")

    if args.dry_run:
        log("[演练] --dry-run：不推。上面就是推下去会发生的事。")
        return

    log("[上传] POST /update …")
    t0 = time.monotonic()
    try:
        body = post_firmware(host, data)
    except (OSError, http.client.HTTPException) as e:
        # 板端在收包途中 abort 会直接掐连接，这里就是它的表现；板端日志里有确切原因
        sys.exit(f"[上传] 连接断在中途：{type(e).__name__}: {e}\n"
                 f"（板端 /log all on 后重推可看到它写的那行原因）")
    log(f"[上传] 板端返回：{body}")
    log(f"[上传] 耗时 {time.monotonic() - t0:.1f}s")

    if args.no_wait:
        log("[重启] --no-wait：不等板子回来。稍后用 `carctl.py status` 自己核对指纹。")
        return

    log("[等待] 等板子重启…")
    if wait_down(host, 20.0) is None:
        log("  （没观察到掉线：可能已经重启完了）")
    up = wait_board_up(host, args.wait_up)
    if up is None:
        log(f"[重启] {args.wait_up:g}s 内板子没回来。"
            f"现场看一眼：不亮 = 新固件起不来（但分区已切，得靠串口/回滚救）")
        return
    log(f"[重启] 板子 {up:.0f}s 后回来了")

    after = parse_runtime(http_get(host, "/update").decode("utf-8", "replace"))
    now_fp = fingerprint_of(after.get("running_stamp", ""))
    log(f"[核对] 板上跑 {after.get('running_stamp', '?')}（分区 {after.get('running_part', '?')}）")
    if now_fp == fp:
        log(f"[成功] 指纹一致（{fp}）—— 新固件确实在跑")
    else:
        log(f"[失败] 指纹不一致：推的是 {fp}，板上跑的是 {now_fp or '?'}")
        log(f"       落点槽里现在是 {after.get('next_stamp', '?')}"
            f"（若那份等于 {fp}，说明写进去了但没切过去/没重启起来）")


def post_firmware(host: str, data: bytes) -> str:
    """把整包固件 POST 给 /update（请求体即裸 .bin），带进度。返回板端的响应正文。"""
    conn = http.client.HTTPConnection(host, HTTP_PORT, timeout=OTA_HTTP_TIMEOUT_S)
    try:
        conn.putrequest("POST", "/update")
        conn.putheader("Content-Type", "application/octet-stream")
        conn.putheader("Content-Length", str(len(data)))
        conn.endheaders()
        total = len(data)
        sent = 0
        mark = 0
        while sent < total:
            chunk = data[sent:sent + BIN_CHUNK]
            conn.send(chunk)
            sent += len(chunk)
            pct = sent * 100 // total
            if pct >= mark + 10:
                mark = pct - pct % 10
                log(f"        {mark:3d}%  {sent / 1024:6.0f}/{total / 1024:.0f} KB")
        resp = conn.getresponse()
        text = resp.read().decode("utf-8", "replace")
        if resp.status != 200:
            raise OSError(f"HTTP {resp.status} {resp.reason}: {text}")
        return f"{resp.status} {text}"
    finally:
        conn.close()


# ---------------- 分层体检（doctor）：分辨"板子射频死了"和"电脑的网络跑了" ----------------

def local_ipv4s() -> list[str]:
    """本机所有非回环 IPv4（含 169.254 自配地址）。"""
    out: list[str] = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            a = info[4][0]
            if not a.startswith("127.") and a not in out:
                out.append(a)
    except OSError:
        pass
    return out


def ble_probe(payload: str, wait_s: float = 4.0, scan_s: float = 10.0) -> dict:
    """走 BLE 发一条指令并收 status 通知。

    返回 {"state": ..., "msgs": [...]}：
      "ok"          —— 连上并收到回话（板子活着，CPU/固件都好）
      "nobroadcast" —— 没扫到 VisionS3 广播（没上电 / 电池没电 / 固件崩了）
      "unavailable" —— 蓝牙栈用不了（没装 bleak / 适配器不可用 / 会话出错）

    BLE 是与 WiFi **互相独立**的另一条射频：WiFi 卡死时（发不出去、不回 ARP）BLE 照样能通。
    所以"BLE 通 + IP 不通"这组证据正好把故障钉死在 WiFi 那一侧，而不是整机崩了 —— 这正是
    doctor 要回答的问题，也是远程重启必须走 BLE 的原因：WS 本身就架在死掉的那条链路上。
    """
    try:
        import asyncio
        import bleak
    except ImportError:
        return {"state": "unavailable", "msgs": ["没装 bleak（用 uv run 跑本脚本会自动装）"]}

    async def run() -> dict:
        dev = await bleak.BleakScanner.find_device_by_filter(
            lambda d, ad: (ad.local_name or "") == BLE_NAME
            or any(u.lower() == BLE_SVC for u in (ad.service_uuids or [])),
            timeout=scan_s,
        )
        if dev is None:
            return {"state": "nobroadcast", "msgs": []}
        got: list[str] = []
        async with bleak.BleakClient(dev, timeout=15.0) as cli:
            if BLE_SVC not in [s.uuid.lower() for s in cli.services]:
                return {"state": "unavailable",
                        "msgs": [f"连上了 {dev.address} 但没有 C0DE 服务"]}
            await cli.start_notify(
                BLE_ST, lambda _c, d: got.append(d.decode("utf-8", "replace")))
            await cli.write_gatt_char(BLE_CMD, payload.encode(), response=True)
            end = time.monotonic() + wait_s
            while time.monotonic() < end:      # 收满窗口才退：回执和日志是分几帧来的
                await asyncio.sleep(0.2)
            try:
                await cli.stop_notify(BLE_ST)
            except Exception:
                pass
        return {"state": "ok", "msgs": got}

    try:
        return asyncio.run(run())
    except Exception as e:                      # bleak 的异常类型很杂，判层不该被它打断
        return {"state": "unavailable", "msgs": [f"{type(e).__name__}: {e}"]}


def wait_board_any(host: str, timeout_s: float) -> str | None:
    """等板子回来，返回它**现在的**地址。

    先认原地址；逾时再扫网段 —— 重启后 DHCP 可能给个新地址，只盯原地址会误判成"没回来"。
    """
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s * 0.4:
        if probe_board(host, 1.5):
            try:
                http_get(host, "/update", timeout=4.0)
                return host
            except (urllib.error.URLError, OSError, TimeoutError):
                pass
        time.sleep(1.0)
    log(f"[等待] {host} 还没回来，扫网段看是不是换了地址…")
    while time.monotonic() - t0 < timeout_s:
        found = scan_boards(timeout=0.2)
        if found:
            return found[0]
        time.sleep(3.0)
    return None


def cmd_doctor(args) -> None:
    """分层体检：在哪一层断的，就在那一层给解药。

    把手工那套（看本机网段 → 探 IP → 探 BLE → 判层 → BLE 重启 → 等回来）压成一条命令。
    断链的现场只在"断着"的那一刻有意义，人走到车边时它多半已经自愈了 —— 所以要留痕。

    判层只有四种结论：
      该地址 IP 通            → 板子好着呢（顺带报固件/分区），什么都不用做
      原地址不通但扫到了       → 只是换了 IP，改缓存即可，**不用重启**
      IP 不通 + BLE 通        → WiFi 侧卡死（CPU/固件都好）→ 解药：BLE 重启
      IP 不通 + BLE 也没广播   → 没上电 / 电池没电 / 固件崩了 → 只能现场
    """
    log("[1/4] 本机网络")
    ips = local_ipv4s()
    if not ips:
        log("  ⚠ 没找到任何非回环 IPv4 —— 本机网络是全断的")
    for a in ips:
        tag = "   ⚠ 169.254 自配（DHCP 没拿到地址）" if a.startswith("169.254.") else ""
        log(f"  {a}{tag}")
    subs = local_subnets()
    log(f"  可扫网段: {', '.join(s + '.0/24' for s in subs) or '（无）'}")

    log("[2/4] 定位板子（-H → 缓存 → 扫网段）")
    host = ""
    if args.host:
        host = normalize_host(args.host)
        log(f"  用 -H 指定的 {host}")
    elif HOST_CACHE.exists():
        cached = HOST_CACHE.read_text(encoding="utf-8").strip()
        if cached:
            host = cached
            log(f"  用上次成功的地址 {host}")
    if host and probe_board(host):
        log("  ✓ 该地址上有板子指纹")
    else:
        if host:
            log(f"  ✗ {host} 上探不到板子")
        found = [] if args.no_scan else scan_boards()
        if found:
            log(f"  ✓ 扫到板子：{', '.join(found)}")
            if host:
                log(f"  ⇒ 判层：**只是换了 IP**（{host} → {found[0]}），改缓存即可，不用重启")
            host = found[0]
            remember_host(host)
        else:
            log("  ✗ 网段里也没扫到板子")

    log("[3/4] IP 链路（WS 81 + HTTP 80）")
    if host:
        if probe_board(host):
            try:
                info = parse_runtime(http_get(host, "/update", timeout=5.0).decode("utf-8", "replace"))
                fp = fingerprint_of(info.get("running_stamp", "")) or "?"
                log(f"  ✓ HTTP 也通 —— 板子在 {host}"
                    f"（固件 {fp}，运行分区 {info.get('running_part', '?')}）")
                log("[结论] 板子 IP 链路正常，没有需要救的东西。")
                return
            except (urllib.error.URLError, OSError, TimeoutError) as e:
                log(f"  ✗ WS 指纹有、HTTP 不通（{type(e).__name__}）—— 半死，值得重启")
        else:
            log(f"  ✗ {host} 的 {WS_PORT} 端口没回 302 /stream")

    log("[4/4] BLE 链路（独立射频：判 WiFi 死还是整机死）")
    r = ble_probe('{"type":"ping"}', wait_s=5.0)
    ble_ok = r["state"] == "ok"
    if ble_ok:
        log("  ✓ BLE 通，板子回话：")
        for m in r["msgs"]:
            log(f"      {m}")
    elif r["state"] == "nobroadcast":
        log("  ✗ 没扫到 VisionS3 广播")
    else:
        log(f"  ? 蓝牙用不了：{'; '.join(r['msgs'])}")

    log("[判层]")
    if ble_ok:
        log("  IP 不通 + BLE 通 ⇒ **WiFi 侧卡死**：CPU/固件都好，就是发不出去")
    elif r["state"] == "nobroadcast":
        log("  IP 不通 + BLE 也没广播 ⇒ 没上电 / 电池没电 / 固件崩了 —— 只能现场看")
    else:
        log("  本机蓝牙用不了，判不了层，只能按'IP 不通'处理")

    if args.no_heal:
        log("[救活] 已按 --no-heal 跳过")
        return
    if not ble_ok:
        log("[救活] BLE 都不通，重启指令发不进去 —— 现场断电吧")
        return
    log("[救活] 经 BLE 下发重启…")
    for m in ble_probe('{"type":"reboot"}', wait_s=4.0)["msgs"]:
        log(f"      {m}")
    old = host
    new = wait_board_any(old, args.wait_up) if old else None
    if new is None:
        log(f"[救活] ✗ {args.wait_up:g}s 内没等到板子回来")
        return
    log(f"[救活] ✓ 板子回来了：{new}"
        + (f"（地址从 {old} 变了）" if new != old else ""))
    remember_host(new)
    try:
        info = parse_runtime(http_get(new, "/update", timeout=6.0).decode("utf-8", "replace"))
        fp = fingerprint_of(info.get("running_stamp", "")) or "?"
        log(f"  运行中：固件 {fp}，分区 {info.get('running_part', '?')}")
    except (urllib.error.URLError, OSError, TimeoutError):
        log("  （HTTP 详情没读到，不影响链路已恢复）")


def parse_args():
    ap = argparse.ArgumentParser(
        description="直连小车的操作台：状态 / 日志 / 发指令 / ping / 重启 / 远程 OTA",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：\n"
               "  carctl.py status\n"
               "  carctl.py log -t 30 --cat all\n"
               "  carctl.py cmd spin dir=1 speed=500\n"
               "  carctl.py cmd ai_goal message=前进两秒\n"
               "  carctl.py ping 192.168.1.1\n"
               "  carctl.py reboot --wait-up 60\n"
               "  carctl.py doctor\n"
               "  carctl.py fw\n"
               "  carctl.py build --flash\n"
               "  carctl.py ota Stm32-Vision.ino.bin\n"
               "  carctl.py coredump            # 体征里见到「复位=崩溃」就来这条\n"
               "  carctl.py stress              # abort 风暴：改并发/修 panic 前后各跑一次对比\n")
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("-H", "--host", help="板子 IP/主机名（省略则用缓存或扫网段）")
    common.add_argument("--no-scan", action="store_true", help="缓存失效时不要扫网段，直接报错")
    common.add_argument("--timeout", type=float, default=5.0, help="WS 建连/HTTP 超时秒（默认 5）")
    common.add_argument("-v", "--verbose", action="store_true", help="连 pong 之类的保活帧也打出来")

    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("status", parents=[common], help="板上跑的是哪份固件、OTA 落点、状态位")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("log", parents=[common], help="实时看板端日志")
    p.add_argument("-t", "--for", dest="duration", type=float, default=0.0, metavar="SEC",
                   help="收多久（0 或不给 = 直到 Ctrl-C）")
    p.add_argument("--cat", default="all", choices=("all", "ai", "exec"), help="日志类别（默认 all）")
    p.add_argument("--leave-on", action="store_true", help="退出时不关板端转发（留给手机看）")
    p.set_defaults(func=cmd_log)

    p = sub.add_parser("cmd", parents=[common], help="发一条词表指令，如 cmd spin dir=1")
    p.add_argument("type", help="指令类型（move/stop/spin/arm/light/log/get_state/ai_goal/…）")
    p.add_argument("kv", nargs="*", metavar="k=v", help="参数，值自动转 int/float/bool/字符串")
    p.add_argument("--wait", type=float, default=3.0, metavar="SEC", help="收回复的时长（默认 3）")
    p.add_argument("--no-log", action="store_true", help="不显示顺带转发来的日志行")
    p.set_defaults(func=cmd_cmd)

    p = sub.add_parser("raw", parents=[common], help="直接发一段原始 JSON")
    p.add_argument("json", help="完整 JSON 文本")
    p.add_argument("--wait", type=float, default=3.0, metavar="SEC", help="收回复的时长（默认 3）")
    p.add_argument("--no-log", action="store_true", help="不显示顺带转发来的日志行")
    p.set_defaults(func=cmd_raw)

    p = sub.add_parser("ping", parents=[common], help="不带给目标=测板子；带目标=让板子去 ping")
    p.add_argument("target", nargs="?", default="", help="IP 或域名（省略则只探板子在不在）")
    p.add_argument("--wait", type=float, default=25.0, metavar="SEC", help="等结果的时长（默认 25）")
    p.add_argument("--no-log", action="store_true", help="不显示顺带转发来的日志行")
    p.set_defaults(func=cmd_ping)

    p = sub.add_parser("reboot", parents=[common], help="远程重启板子")
    p.add_argument("--wait", type=float, default=6.0, metavar="SEC", help="等回执的时长（默认 6）")
    p.add_argument("--no-log", action="store_true", help="不显示顺带转发来的日志行")
    p.add_argument("--wait-up", type=float, default=0.0, metavar="SEC",
                   help="重启后等它回来的上限（默认 0 = 不等；给 60 就等一分钟）")
    p.set_defaults(func=cmd_reboot)

    p = sub.add_parser("doctor", parents=[common],
                       help="分层体检：在哪层断的就在那层给解药（必要时走 BLE 救活）")
    p.add_argument("--no-heal", action="store_true", help="只诊断，不自动 BLE 重启")
    p.add_argument("--wait-up", type=float, default=75.0, metavar="SEC",
                   help="BLE 重启后等板子回来的上限（默认 75）")
    p.set_defaults(func=cmd_doctor)

    p = sub.add_parser("fw", help="读 .bin 指纹（不给文件则列本机编译缓存里的候选）")
    p.add_argument("bin", nargs="*", help=".bin 路径（可多个）")
    p.set_defaults(func=cmd_fw)

    p = sub.add_parser("build", parents=[common],
                       help="编译草图（--flash 则连编译带上板一条龙）")
    p.add_argument("--clean", action="store_true",
                   help="全量重编（改/增/删源文件、或改了内核 sdkconfig.h 后必须加）")
    p.add_argument("--flash", action="store_true", help="编完直接 OTA 上板并核对指纹")
    p.add_argument("--force", action="store_true", help="配合 --flash：板上已是这份固件也照推")
    p.add_argument("--wait-up", type=float, default=90.0, metavar="SEC",
                   help="配合 --flash：等板子重启回来的上限（默认 90）")
    p.set_defaults(func=None)          # build 不必先定位板子；只有 --flash 才需要，见 main()

    p = sub.add_parser("ota", parents=[common], help="推固件并核对新固件真的跑起来了")
    p.add_argument("bin", help="要推的 .bin（或直接给个目录里的文件名）")
    p.add_argument("--wait-up", type=float, default=90.0, metavar="SEC",
                   help="等板子重启回来的上限（默认 90）")
    p.add_argument("--no-wait", action="store_true", help="推完就不管，不等重启也不核对")
    p.add_argument("--dry-run", action="store_true", help="只报告会发生什么，不上传")
    p.add_argument("--force", action="store_true", help="板上已经是这份固件时也照推")
    p.set_defaults(func=cmd_ota)

    p = sub.add_parser("coredump", parents=[common],
                       help="取回 panic 现场（原因/任务/回溯）并解成文件:行号")
    p.add_argument("--erase", action="store_true", help="取完顺手清掉现场")
    p.add_argument("--elf", help="指定 .elf（默认按 coredump 回传的 sha 去归档里自动找）")
    p.set_defaults(func=cmd_coredump)

    p = sub.add_parser("stress", parents=[common],
                       help="abort 风暴：AI 在途时反复中止，逼出跨任务竞态（改并发/修 panic 前后各跑一次）")
    p.add_argument("--rounds", type=int, default=12, help="起多少轮 AI（默认 12）")
    p.add_argument("--gap", type=float, default=3.0, metavar="SEC", help="每轮间隔秒（默认 3）")
    p.add_argument("--abort-ms", type=float, default=200.0, metavar="MS",
                   help="中止指令间隔毫秒（stop/ai_cancel 交替，默认 200）")
    p.add_argument("--sample", type=float, default=2.0, metavar="SEC",
                   help="风暴中每几秒采一次体征（默认 2；0=不采）——卡死了也没这组数就没法分层")
    p.add_argument("--type", default="ai_oneshot", choices=("ai_oneshot", "ai_goal"),
                   help="起轮次用哪条指令（默认 ai_oneshot=单轮不闭环）")
    p.add_argument("--goal", default=STRESS_GOAL, help="喂给 AI 的目标文本（默认带「别动」）")
    p.add_argument("--keep-panic", action="store_true", help="不先清空旧现场")
    p.add_argument("--wait-up", type=float, default=60.0, metavar="SEC",
                   help="崩了之后等板子回来的上限（默认 60）")
    p.set_defaults(func=cmd_stress)

    p = sub.add_parser("frame", parents=[common],
                       help="抓板端单帧（/capture）：手动操作用的眼睛，不依赖 AI 任务/日志")
    p.add_argument("-o", "--out", help="输出路径（默认 tools/shots/<时刻或标签>.jpg）")
    p.add_argument("-t", "--tag", help="文件名标签（如 before-lift；默认用当前时刻）")
    p.add_argument("-n", type=int, default=1, help="连拍张数（默认 1，看运动用）")
    p.add_argument("--gap", type=float, default=0.5, metavar="SEC", help="连拍间隔秒（默认 0.5）")
    p.add_argument("--tries", type=int, default=4, help="单张失败重试次数（默认 4）")
    p.set_defaults(func=cmd_frame)

    p = sub.add_parser("step", parents=[common],
                       help="发指令+抓帧+记状态行（手动夹取/复盘用，一次一条可对照的记录）")
    p.add_argument("type", help="词表指令类型，如 arm / move / spin")
    p.add_argument("kv", nargs="*", help="参数，形如 act=low / distance_cm=3")
    p.add_argument("-t", "--tag", help="本次步骤的标签（文件名 + 记录条目标题）")
    p.add_argument("--wait", type=float, default=4.0, metavar="SEC", help="等落地秒数（默认 4）")
    p.set_defaults(func=cmd_step)
    return ap.parse_args()


SHOTS_DIR = Path(__file__).resolve().parent / "shots"
STEPS_LOG = SHOTS_DIR / "steps.txt"


def grab_still(host: str, timeout: float, tries: int = 4) -> bytes:
    """抓一张板端单帧，返回 JPEG 字节。取不到就抛最后一个错。"""
    err: Exception | str | None = None
    for _ in range(tries):
        try:
            blob = http_get(host, "/capture", timeout=timeout)
        except Exception as e:   # 推流/相机缓冲被占时偶发取不到：重试即可，不是错
            err = e
            time.sleep(0.3)
            continue
        if blob[:2] != b"\xff\xd8":
            err = f"不是 JPEG（前16B {blob[:16]!r}）"
            time.sleep(0.3)
            continue
        return blob
    raise RuntimeError(f"抓帧失败：{err}")


def cmd_frame(host: str, args) -> None:
    """抓板端单帧（HTTP /capture）。

    手动操作时的"眼睛"：不像 /ai_dump 那样必须先开 AI 日志、还得有 AI 任务在跑 —— 这里任何时候
    都能取到画面，于是"我自己动手夹一次、再跟 AI 的做法对比"才有可能（/capture 是板端本来就有的
    端点，这里只是把它变成一条命令）。连拍用来看**运动**：合爪/抬臂那几秒里目标有没有跟着动。
    """
    if args.out and args.n > 1:
        sys.exit("-o 与 -n 不能同用（连拍请用 -t 标签，会生成 <标签>-0/-1/...）")
    for i in range(args.n):
        if i:
            time.sleep(args.gap)
        try:
            blob = grab_still(host, args.timeout, args.tries)
        except RuntimeError as e:
            sys.exit(str(e))
        dst = Path(args.out) if args.out else None
        if dst is None:
            SHOTS_DIR.mkdir(parents=True, exist_ok=True)
            tag = args.tag or time.strftime("%H%M%S")
            dst = SHOTS_DIR / (f"{tag}.jpg" if args.n == 1 else f"{tag}-{i}.jpg")
        dst.write_bytes(blob)
        log(f"{dst}  {len(blob) / 1024:.1f}KB")


def fetch_state(host: str, timeout: float, wait: float = 2.5) -> str | None:
    """取一条板端状态行（`小车:… | 抓手:前Xcm 高Ycm 爪:…`）。

    独立开一次连接：板端 exec_status 是**按变化**推的，动作刚做完那一下未必推得及时；而转发一
    打开它总会先来一条当前状态，所以"另开一次拿第一条"比在动作窗口里等更稳。
    """
    state = None
    try:
        with ws_open(host, timeout) as ws:
            ws.send('{"type":"log","params":{"cat":"exec","on":true}}')
            end = time.monotonic() + wait
            while time.monotonic() < end:
                try:
                    raw = ws.recv(timeout=min(0.5, max(0.05, end - time.monotonic())))
                except TimeoutError:
                    continue
                except (OSError, ConnectionClosed):
                    break
                if isinstance(raw, (bytes, bytearray)):
                    continue
                try:
                    m = json.loads(raw)
                except ValueError:
                    continue
                if not isinstance(m, dict):
                    continue
                if m.get("type") == "exec_status":
                    state = (m.get("params") or {}).get("text") or state
                    break
                if m.get("type") == "ping":
                    ws.send('{"type":"pong"}')
    except (OSError, ConnectionClosed, TimeoutError):
        pass
    return state


def cmd_step(host: str, args) -> None:
    """发一条指令 → 等它落地 → 抓一张帧 → 三样东西一起记进 tools/shots/steps.txt。

    手动复现"夹一次"时，一次操作得同时留下：我发了什么、板子自报爪子在哪（状态行）、画面实际
    是什么。分开跑三条命令永远对不上账（帧比指令晚、状态又是另一时刻的），所以合成一条：帧与
    状态行取自同一时刻，且落在同一个会话文件里，事后能逐帧对着复盘。

    会临时打开 exec 日志转发拿状态行（板端在 WS 断开时自动关掉，不用手动收）。
    """
    params = {}
    for item in args.kv:
        if "=" not in item:
            sys.exit(f"参数得写成 k=v：{item}")
        k, v = item.split("=", 1)
        params[k.strip()] = coerce(v)
    payload = json.dumps({"type": args.type, "params": params}, ensure_ascii=False)
    state: str | None = None
    lines: list[str] = []
    with ws_open(host, args.timeout) as ws:
        ws.send('{"type":"log","params":{"cat":"exec","on":true}}')   # 要状态行就得开转发
        time.sleep(0.4)
        log(f"[发出] {payload}")
        ws.send(payload)
        end = time.monotonic() + args.wait
        while time.monotonic() < end:
            try:
                raw = ws.recv(timeout=min(0.5, max(0.05, end - time.monotonic())))
            except TimeoutError:
                continue
            except (OSError, ConnectionClosed):
                break
            if isinstance(raw, (bytes, bytearray)):
                continue
            try:
                m = json.loads(raw)
            except ValueError:
                continue
            if not isinstance(m, dict):
                continue
            if m.get("type") == "ping":
                ws.send('{"type":"pong"}')
                continue
            if m.get("type") == "exec_status":
                state = (m.get("params") or {}).get("text") or state
                continue
            if m.get("type") == "log":
                continue
            if line := fmt_frame(m, True, args.verbose):
                lines.append(line)
    state = state or fetch_state(host, args.timeout)   # 动作后另开一次取状态行(见其注释)
    SHOTS_DIR.mkdir(parents=True, exist_ok=True)
    tag = args.tag or time.strftime("%H%M%S")
    dst = SHOTS_DIR / f"{tag}.jpg"
    try:
        blob = grab_still(host, args.timeout)
    except RuntimeError as e:
        sys.exit(str(e))
    dst.write_bytes(blob)
    with STEPS_LOG.open("a", encoding="utf-8") as f:
        f.write(f"### {tag}  {time.strftime('%H:%M:%S')}\n")
        f.write(f"cmd:   {payload}\n")
        f.write(f"state: {state or '(没收到状态行)'}\n")
        for ln in lines:
            f.write(f"  {ln}\n")
        f.write("\n")
    if state:
        log(f"[状态] {state}")
    for ln in lines:
        log(ln)
    log(f"{dst}  {len(blob) / 1024:.1f}KB  （记录追加到 {STEPS_LOG}）")


def cmd_cmd(host: str, args) -> None:
    params = {}
    for item in args.kv:
        if "=" not in item:
            sys.exit(f"参数得写成 k=v：{item}")
        k, v = item.split("=", 1)
        params[k.strip()] = coerce(v)
    payload = json.dumps({"type": args.type, "params": params}, ensure_ascii=False)
    send_and_wait(host, payload, args, args.wait, f"{args.type} 已发")


def cmd_raw(host: str, args) -> None:
    try:
        json.loads(args.json)          # 本地先验一遍：别让板子去猜我手抖写坏的 JSON
    except ValueError as e:
        sys.exit(f"不是合法 JSON：{e}")
    send_and_wait(host, args.json, args, args.wait, "原始帧已发")


def main() -> None:
    strip_proxy_env()          # 必须在任何连接之前：见 strip_proxy_env 的注释
    args = parse_args()
    if args.cmd == "fw":       # fw 只读本机文件，不碰网络，不需要定位板子
        cmd_fw(args)
        return
    if args.cmd == "build":
        # 先编译再定位板子：编译不需要网络，板子不通时也该能编（编完再报连不上，
        # 而不是"因为找不到板子所以没编"）。--flash 才走下面这段。
        bin_path = cmd_build(args)
        if bin_path is None:
            sys.exit(1)
        if not args.flash:
            log("[提示] 上板：carctl.py build --flash   或   carctl.py ota <上面的产物>")
            return
        host = resolve_host(args)
        try:
            cmd_ota(host, argparse.Namespace(bin=str(bin_path), force=args.force,
                                             dry_run=False, no_wait=False,
                                             wait_up=args.wait_up))
        except KeyboardInterrupt:
            log("\n[中断] 已停止")
        except (OSError, ConnectionClosed, TimeoutError) as e:
            sys.exit(f"[连接失败] {type(e).__name__}: {e}")
        return
    if args.cmd == "doctor":   # doctor 自己判层：板子不通是它的**正常输入**，不能先 sys.exit
        try:
            cmd_doctor(args)
        except KeyboardInterrupt:
            log("\n[中断] 已停止")
        return
    host = resolve_host(args)
    try:
        args.func(host, args)
    except KeyboardInterrupt:
        log("\n[中断] 已停止")
    except (OSError, ConnectionClosed, TimeoutError) as e:
        sys.exit(f"[连接失败] {type(e).__name__}: {e}\n"
                 f"（板子卡死时 WS/HTTP 都不通，且 `reboot` 也发不进去——那种情况只能现场断电）")
    else:
        remember_host(host)


if __name__ == "__main__":
    main()
