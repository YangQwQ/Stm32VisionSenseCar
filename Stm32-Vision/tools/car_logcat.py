# /// script
# requires-python = ">=3.10"
# dependencies = ["websockets>=13", "bleak>=0.22"]
# ///
"""car_logcat —— 直连小车：一边把板端日志落盘，一边把 AI 每轮实际看到的画面留档。

一个脚本两件事：
  · WS(端口 81) 收板端日志  → logs/car-<时间戳>.log
  · HTTP(端口 80) 取板端留档的 AI 画面 → logs/aiframes/<上面那份日志的同名目录>/

**为什么要留画面**：AI 每轮据一帧画面决策，但板端日志只留**决策与坐标**，画面本身转瞬即弃。
于是复盘"它为什么夹空"时只能对着日志里的坐标猜它看见了什么 —— 而实测最有价值的那类证据恰恰是
"日志说居中、画面里目标却在旁边 9cm"，没有图就永远说不清。板端 ai_dump 模块把**实际发往云端的
那一帧**（原始 JPEG 字节）留在 PSRAM，本脚本经 HTTP 取回来配着标注落盘。

**为什么日志能这么收**（不需要改固件）：
  板子 blog 模块的日志转发是**板端广播**（app_httpd 的转发器 → 遍历所有 WS 客户端逐个发
  {type:"log",params:{src,text}}）。手机 App 只是其中一个接收方，所以电脑自己再接一条 WS
  就是第二个接收方：拿到的日志与手机完全一样，手机那边照旧控制、互不干扰。
  转发开关 /log 是板端**全局单选**状态（开 all 会清掉类别位，开类别会清掉 all，见 board_log.h），
  所以本脚本开一下开关，手机侧也会看到同样的日志——这与"在手机上 /log all on"效果一致。

  注意板端的活体探测是**全局**的：连续 6s 没有任何客户端上行就广播探测 ping，多轮无应答会把
  **所有**客户端一起踢（app_httpd WS_IDLE_PING_MS / ws_kick_all）。所以本脚本自己按 3s 周期发
  {"type":"ping"} 并等 pong：既是自己的保活，也顺带喂饱那个全局 idle 计时器（手机没连也不掉线）。

**为什么画面与日志并在一个脚本里**（原 ai_frame_dump.py 已并入）：
  分开跑时"这些图配哪份日志"只能靠文件 mtime 猜（起晚了、或中途重启过，就配错目录），定位也要
  各扫一遍网段。并在一个进程里，归档目录就是本进程**刚刚建的那份日志**的名字 —— 配对不再靠猜；
  一次定位两处共用；Ctrl+C 一次两者同时收工。代价是抓帧随日志会话起止（半途起脚本时，板端环形
  里还留着的几帧照样一次性回填，更早的本来也早被挤掉了）。

**抓帧怎么抓**：留档默认不开，跟着板端 `/log` 开关走（未改任何协议）——本脚本启动时自己开
--cat 指定的类别，AI 类别一开，板端才开始留画面。留档上限 6 帧环形，所以默认**长轮询**：请求
挂在板端不返回（最多 15s），一有新定案帧立刻应答 —— 既不必猜轮询周期，也没有轮询间隙（定时问的
话，一轮 AI 约 2s，十几秒的卡顿就把帧挤掉了）。抓帧跑在**自己的线程**里，与 WS 会话解耦：板子
断线重连、甚至 WS 完全连不上时，画面照收。

用法（uv 会按上面的内联元数据自动装 websockets，不必手动建环境）：
    uv run tools/car_logcat.py                    # 自动找车（缓存 → 扫本网段），日志+抓帧一起收
    uv run tools/car_logcat.py -H 192.168.1.23    # 指定板子 IP
    uv run tools/car_logcat.py --cat ai           # 只转发 AI 类日志（默认 all）；抓帧跟着 AI 开关
    uv run tools/car_logcat.py --no-frames        # 只记日志，不抓帧
    uv run tools/car_logcat.py -o log.txt         # 日志写到指定文件（抓帧落到 ./aiframes/log/）
    uv run tools/car_logcat.py --for 300 --open   # 记 300 秒后自动收工，并打开缩略图页
    uv run tools/car_logcat.py --no-ble           # 关掉 BLE 兜底通道（默认开）

**BLE 兜底通道**（默认开，`--no-ble` 关）：板端 blog 的转发器是 WS+BLE **双发**同一条日志，
所以本脚本让 BLE 待机、只在 WS 静默 6s 后才接上，WS 一回来就断开。于是日志里 `[蓝牙]` 开头的
那几行**就是**断开区间本身 —— 这正是不做兜底时永远拿不到的那段：WiFi 已经发不出去（`发0`、
不回 ARP、ping 都不通），日志却还能从蓝牙出来，说明 CPU/固件/日志队列都好着，坏的只是 WiFi。

日志输出格式与手机端导出的日志一致：`[HH:MM:SS][来源] 文本`（来源 = exec/ai/net/cam/ws/cmd/ble/sys）。
抓帧另在同一个文件里留 `[HH:MM:SS][抓帧] #7 62.3KB prev 执行 arm_low` —— 与板端那轮的文字日志
逐行对着看，就能定位"它说看到了什么"与"它因此做了什么"。
每个会话开转发时板端会回一条 status，里面带着**当前固件标记**（形如 `固件 v1 指纹 fef509d0`），
本脚本照常记成 `[板] …` 一行 —— 日志因此自带"采集时板上跑的是哪份固件"，比对 OTA 前后即可确认
新固件到底生效没有（指纹 = 该 .bin 的 ELF SHA-256 前 4 字节，见 Stm32-Vision/src/net/ota.cpp）。
默认写到本工程 `logs/car-<时间戳>.log`（锚定脚本位置，从哪调用都一样；该目录未被 .gitignore 覆盖）。
默认丢弃 ping/log 两类指令回显（板端会把收到的每条指令记一行 cmd 日志，本脚本自己的心跳与定期
重申都会刷这个，不收就是自噪声）；要看加 --keep-echo。

退出时默认把 /log 复位成关闭（否则手机端会一直刷你这次开的类别，板端留档的 PSRAM 也不释放）；
想保留用 --leave-on。

捕获类别以本脚本的 --cat 为准：运行期间每 30s 重申一次（板端 /log 是全局单选，在手机上敲
/log 会把它改成别的类别甚至关掉，那样脚本会"连着却什么都没记"）。想换类别请重启本脚本。

抓不到的东西（板端根本没往外发）：
  - 手机本地的行（`[本机] 插话·…`、`/ai cancel` 之类）；
  - `ai_client` 里走 Serial 直出的思考过程转储——这类只进 UART，不在 WS 广播里。
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import queue
import re
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path

try:
    from websockets.asyncio.client import connect
    from websockets.exceptions import ConnectionClosed
except ImportError:  # pragma: no cover - 提示装依赖而不是抛栈
    sys.exit("缺少依赖 websockets：请用 `uv run tools/car_logcat.py`（会按内联元数据自动装）")

WS_PORT = 81                    # 板端 WS/图传端口（HTTP 在 80）
# BLE 兜底通道（与 ble.cpp / 手机 BleProfile.gd 逐字 mirror）
BLE_SVC = "0000c0de-0000-1000-8000-00805f9b34fb"     # 服务
BLE_CMD = "0000c0e5-0000-1000-8000-00805f9b34fb"     # 收原始 JSON 指令
BLE_ST = "0000c0e6-0000-1000-8000-00805f9b34fb"      # 状态/日志通知
BLE_NAME = "VisionS3"
BLE_AFTER_S = 6.0               # 开局宽限：给 WS 这么久先连上；之后只看 ws_up，不再按静默时长判
PING_EVERY_S = 3.0              # 心跳周期：必须 < 板端 WS_IDLE_PING_MS(6s)
PONG_TIMEOUT_S = 3.0            # 单次 ping 等应答上限
MAX_MISS = 2                    # 连续这么多次无应答即判链断重连
RECV_TICK_S = 0.5               # 收包轮询粒度（用于插入心跳与超时判定）
STABLE_S = 10.0                 # 会话活过这么久算"稳定"，下次断开时退避从 1s 重来
REASSERT_S = 30.0               # 定期重申 /log on：板端转发是全局单选，手机端一敲 /log 就会改掉
HOST_CACHE = Path.home() / ".car_logcat" / "last_host"
QUEUE_MAX = 20000               # 落盘队列上限（主循环只入队，落盘线程再慢也不反压 socket）
CONSOLE_MAX = 2000              # 回显队列上限（控制台堵死时只丢回显，别在内存里堆日志）
HTTP_PORT = 80                  # 板端 HTTP（日志走 WS 的 81，抓帧走这里）
HTTP_TIMEOUT_S = 8.0            # 单次 HTTP 上限（取图只是 PSRAM 拷贝，应远快于此）
FOLLOW_TIMEOUT_S = 25.0         # 抓帧长轮询的请求上限：必须 > 板端挂起上限(AI_DUMP_FOLLOW_MS 15s)
HELD_MIN_S = 1.5                # 清单请求耗时超过它 = 板端确实挂起过（即支持长轮询）
FRAMES_MIN_RUN_S = 10.0         # 跑这么久才敢说"板端没有留档"（此前开关刚开上/首轮 AI 还没跑完）
FRAMES_BACKOFF_MAX_S = 10.0     # 板子问不到时（重启/掉线）的重试上限，免得每秒刷一行警告


# ---------------- 输出收口（落盘 / 回显各一条线程） ----------------

class Writer:
    """落盘与回显各走自己的队列+线程，主循环（读 socket）绝不碰 stdout/文件。

    为什么必须这样：Windows 传统控制台开着"快速编辑"，在窗口里**选中文字**（想把日志或报错
    复制给别人时顺手就做了）会让任何写控制台的线程阻塞在 print 上。若主循环自己 print/写文件，
    它就会卡在那里不再读 socket：板端往这个不读的客户端发送时被堵住（板端 tune_socket 只设了
    8KB SO_SNDBUF 与 TCP_NODELAY，没有 SO_SNDTIMEO），本脚本这一侧的记录与保活全停。

    两条队列都是 put_nowait：写线程再堵也绝不反压主循环（宁可丢行也不丢 socket）。
    落盘与回显**必须分开**——同一条线程里"先写文件再 print"的话，print 一堵该线程就再也不回来
    取下一条，文件会跟着一起停（实测：控制台 64KB 管道灌满后，落盘停在 240 行 / 已收 1400+）。
    分开后控制台卡死只丢回显（计入 lost_echo），落盘逐行 flush 照旧完整。
    """

    def __init__(self, out_path: Path, echo: bool) -> None:
        self.out_path = out_path
        self.echo = echo
        self.dropped = 0                          # 落盘队列满而丢的行
        self.lost_echo = 0                        # 控制台堵死/太慢而丢的回显
        self._fq: queue.Queue = queue.Queue(maxsize=QUEUE_MAX)
        self._cq: queue.Queue = queue.Queue(maxsize=CONSOLE_MAX)
        self._tf = threading.Thread(target=self._file_loop, name="writer-file", daemon=True)
        self._tc = threading.Thread(target=self._console_loop, name="writer-console", daemon=True)
        self._tf.start()
        self._tc.start()

    def _file_loop(self) -> None:
        try:  # 逐行 flush：掉电/被杀也不丢已落盘的日志
            f = open(self.out_path, "a", encoding="utf-8", buffering=1)  # noqa: SIM115 — 要在 open 处捕 OSError
        except OSError as e:
            self._console(f"[输出] 打不开 {self.out_path}: {e}")
            return                                # 之后入队的行会滞留在队列里，到上限后计入 dropped
        with f:
            while True:
                line = self._fq.get()
                if line is None:
                    break
                try:
                    f.write(line + "\n")
                except OSError:
                    pass

    def _console_loop(self) -> None:
        while True:
            msg = self._cq.get()
            if msg is None:
                break
            try:
                # 必须走 print/sys.stdout：它会把文本转成 UTF-16 再 WriteConsoleW，中文才正常；
                # 直接 os.write(1) 在 Windows 控制台是按代码页解字节的，中文会乱码。
                print(msg, flush=True)
            except (OSError, UnicodeEncodeError, ValueError):
                pass          # 控制台堵死/已关闭：丢这条；本线程卡在 print 上后，队列满了就丢新来的

    def _to_file(self, line: str) -> None:
        try:
            self._fq.put_nowait(line)
        except queue.Full:
            self.dropped += 1

    def _to_console(self, msg: str) -> None:
        if not self.echo:
            return
        try:
            self._cq.put_nowait(msg)
        except queue.Full:
            self.lost_echo += 1

    def emit_line(self, line: str) -> None:      # 日志正文：落盘 + 回显
        self._to_file(line)
        self._to_console(line)

    def note(self, msg: str) -> None:            # 状态提示：只回显
        self._to_console(msg)

    def file_only(self, line: str) -> None:      # 文件头注释：只落盘
        self._to_file(line)

    def close(self) -> None:
        """等落盘队列排空再退；回显只给 1s（它卡住时是控制台的锅，不拖着进程不退）。"""
        deadline = time.monotonic() + 15.0
        while not self._fq.empty() and time.monotonic() < deadline:
            time.sleep(0.05)
        try:
            self._fq.put_nowait(None)
        except queue.Full:
            pass
        self._tf.join(timeout=5.0)
        deadline = time.monotonic() + 1.0
        while not self._cq.empty() and time.monotonic() < deadline:
            time.sleep(0.02)
        try:
            self._cq.put_nowait(None)
        except queue.Full:
            pass
        self._tc.join(timeout=1.0)


_sink: Writer | None = None      # 建好 Writer 前的状态提示直接打印（那时还没连板子，卡住无碍）


def log(msg: str) -> None:
    if _sink is not None:
        _sink.note(msg)
    else:
        print(msg, flush=True)


def strip_proxy_env() -> None:
    """清掉代理环境变量。

    本机全局配了 HTTP(S)_PROXY=127.0.0.1:7897（见 ~/.bashrc），而 websockets 15+ 会**默认
    读环境变量走代理** —— 那会让发往局域网小车(192.168.x.x)的连接被塞进代理，直接连不上。
    这里统一清掉，保证直连板子。
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

    板子的 ws_handler 对"非 WS 升级的 GET /"会 302 到 /stream（见 app_httpd ws_handler），
    拿这个当指纹：端口 81 上别的服务不会给出这条 302。

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


# ---------------- 板端 HTTP（抓帧用；日志那条链路走 WS，互不相干） ----------------

def get(host: str, path: str, timeout: float = HTTP_TIMEOUT_S, port: int = HTTP_PORT) -> bytes:
    """GET 板端 HTTP。显式关掉代理：本机 127.0.0.1:7897 会把局域网请求截胡（见 strip_proxy_env）。"""
    req = urllib.request.Request(f"http://{host}:{port}{path}",
                                 headers={"Cache-Control": "no-store"})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=timeout) as r:
        return r.read()


def has_dump(host: str, port: int = HTTP_PORT) -> bool:
    """板上固件带不带抓帧留档？

    `/ai_dump` 是留档固件才有的入口，返回的 {"slots":…} 既是存在性也是能力判定。注意它**不依赖**
    /log 开关：留档没开时板端照样回一份合法的空清单（slots 在），所以这一问拿到的"能不能抓"是准的。
    不带 after 的清单请求是立刻返回的（长轮询只在带 after 时挂起），所以这一问不会把启动拖住。
    """
    try:
        return get(host, "/ai_dump", port=port).lstrip().startswith(b'{"slots"')
    except (urllib.error.URLError, OSError, TimeoutError, ValueError):
        return False


def frames_dir_for(log_path: Path, override: str | None = None) -> Path:
    """抓帧归档目录：默认在日志文件同级的 `aiframes/` 下，**目录名就是这份日志的名字**。

    不按 mtime 去"找最新的 car-*.log"配对（那是两个脚本分开跑时的将就办法，起晚了/中途重启过
    就配错），而是由本进程刚建的那份日志直接推出来 —— 图与日志一一对应是结构上保证的。
    """
    return Path(override) if override else log_path.parent / "aiframes" / log_path.stem


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
        return normalize_host(args.host)
    if HOST_CACHE.exists():
        cached = HOST_CACHE.read_text(encoding="utf-8").strip()
        if cached and probe_board(cached):
            log(f"[定位] 用上次的地址 {cached}（缓存 {HOST_CACHE}）")
            return cached
    if args.no_scan:
        sys.exit(f"没给 -H 且缓存不可用（{HOST_CACHE}）；用 -H <ip> 指定，或去掉 --no-scan 让它扫网段")
    found = scan_boards()
    if not found:
        sys.exit("没扫到板子：确认电脑与小车在同一网段、板子已联网，或用 -H <ip> 指定")
    if len(found) > 1:
        log(f"[定位] 扫到多个候选 {found}，用第一个（要指定请加 -H）")
    host = found[0]
    log(f"[定位] 找到板子 {host}")
    return host


def remember_host(host: str) -> None:
    try:
        HOST_CACHE.parent.mkdir(parents=True, exist_ok=True)
        HOST_CACHE.write_text(host + "\n", encoding="utf-8")
    except OSError:
        pass


# ---------------- 日志落盘 ----------------

class Stats:
    def __init__(self) -> None:
        self.lines = 0
        self.frames = 0
        self.filtered = 0
        self.sessions = 0
        self.enabled = False        # 是否真的开过转发（决定退出时要不要复位）
        self.deadline = None        # monotonic 截止时刻（--for），到点由会话内主动退出
        self.ws_last_rx = 0.0       # 最近一次 WS 下行/连上的时刻（仅供读数，别拿它判死活）
        # WS 是否处于**已连上**状态。BLE 兜底只认这个，不认"静默了多久"：
        # 板端心跳 5s 一次，按静默判会只剩 1s 余量，WS 好着也误报成断（本工具踩过）。
        self.ws_up = False
        self.ble_lines = 0          # 经 BLE 兜底通道落盘的行数（>0 即本次出现过断开区间）


def emit(wr: Writer, src: str, text: str, st: Stats) -> None:
    """一条记录写成一行：把内嵌换行转义掉，保证"一行一记录"（与手机导出格式一致）。"""
    text = text.replace("\r", "").replace("\n", "\\n")
    if text.startswith(f"[{src}] "):        # 板端文本多已自带 [类] 前缀，避免重复
        text = text[len(src) + 3:]
    ts = datetime.now().strftime("%H:%M:%S")
    wr.emit_line(f"[{ts}][{src}] {text}")
    st.lines += 1


def handle_frame(m: dict, raw: str, wr: Writer, args, st: Stats) -> None:
    t = m.get("type")
    p = m.get("params") or {}
    if t == "log":
        src = str(p.get("src") or "log")
        text = str(p.get("text") or "")
        # 自身回声默认丢掉：本脚本发的 ping/log 都会被板端记成一行 cmd 日志（command.cpp 的
        # "type=%s has_frames" 在 manual 分支之外，ping/log 都走它），不收就是每 30s 一条自噪声。
        # 手机端心跳/敲 /log 也是同一形状，一并丢掉。
        if src == "cmd" and re.search(r"type=(?:ping|log)\b", text) and not args.keep_echo:
            st.filtered += 1
            return
        emit(wr, src, text, st)
    elif t == "exec_status":
        # /log exec on 时板端约 400ms 推一条（内容变化才推）的直驱状态
        emit(wr, "exec", str(p.get("text") or ""), st)
    elif t == "status":
        # 指令回执（reply_status）：手机端显示成 [板] …，这里保持同一形状
        emit(wr, "板", str(p.get("reason") or ""), st)
    elif t == "ai_result":
        # 一条已下发的动作（build_feedback）：手机端显示成 [执行 <type> <k=v…>]，照此渲染
        c = p.get("command") or {}
        ctype = c.get("type")
        if ctype:
            kv = " ".join(f"{k}={v}" for k, v in (c.get("params") or {}).items())
            emit(wr, "执行", f"{ctype} {kv}".strip(), st)
        else:
            emit(wr, "执行", str(p.get("reason") or ""), st)
    elif t in ("pong", "ping"):
        st.frames += 1
    else:
        # state / 其它应答：原文留档（这些是"板子回了什么"的直接证据）
        emit(wr, "rx", raw, st)


# ---------------- 抓帧留档（板端 ai_dump → 本地 aiframes/） ----------------

_NOTHING_HINT = (
    "[提示] 板端当前没有留档。逐项确认：① 本脚本的 --cat 是否含 ai 类别（留档跟着 AI 这个类别走，"
    "改成 --cat exec 时板端一个字节都不留）；② 开关开过之后**跑过一次 AI 任务**吗（/ai goal …）；"
    "③ 上次任务是否已过去太久（环形只留 6 帧，更早的被挤掉了）。若固件还没有长轮询，本脚本会自动"
    "退化成定时问。"
)


def safe_name(s: str, cap: int = 64) -> str:
    """标注 → 文件名片段：去掉路径/引号类字符（Windows 上 `:` 等非法），按 UTF-8 安全截断。

    64 字节 ≈ 21 个汉字：再短的话"被闸门拒(未执行) arm clip:"这层前缀就把原因挤没了，
    而文件名正是复盘时列表里第一眼看到的东西（完整标注始终在 index.html 里）。
    """
    out = "".join(c for c in s if c not in '\\/:*?"<>|' and ord(c) >= 32).strip().replace(" ", "_")
    b = out.encode("utf-8")
    if len(b) > cap:
        out = b[:cap].decode("utf-8", "ignore")   # 按字节截会落下半个汉字，得从字符边界退回来
    return out or "note"


def is_jpeg(b: bytes) -> bool:
    return len(b) > 4 and b[:2] == b"\xff\xd8" and b[-2:] == b"\xff\xd9"


class Collector:
    """把板端留档的帧取回落盘，并维护 index.json / index.html。

    单写者：只有抓帧线程动 items/seen。主线程只在收工时再写一次索引（见 Frames.finalize），那时
    已经请抓帧线程停下 —— 最坏也只是"最后一帧没进索引"：每帧**落盘发生在记账之前**，.jpg 不会丢，
    下一趟并入索引时会把它补上。
    """

    def __init__(self, outdir: Path, host: str, report=None, log_ref: str = "",
                 port: int = HTTP_PORT) -> None:
        self.dir = outdir
        self.dir.mkdir(parents=True, exist_ok=True)   # 自己保证目录在（-o 给了二级路径也不怕）
        self.host = host
        self.port = port
        self.log_ref = log_ref
        self._report = report or log
        self.seen: set[int] = set()
        self.items: list[dict] = []
        self.after: int | None = None   # 结清水位：≤ 它的帧都已落盘。None = 还没做过首次清单
        self.held = False               # 上次清单请求是否被板端挂起过（= 支持长轮询）
        self.failed = False             # 上次清单请求是否失败（板子问不到 / 答非所问）
        self.skipped = 0                # 滚出环形、再也拿不回来的帧数（如实计数，不装作收全了）
        self._load_index()              # 同一场跑第二次时接着记，不把前一次的索引冲掉

    def say(self, msg: str) -> None:
        """抓帧侧的一行话：同时进控制台与日志文件（带 [抓帧] 来源），便于与板端日志对着看。"""
        self._report(msg)

    def _load_index(self) -> None:
        """并入已有 index.json：同一场重复跑（或上一趟被判链断）时接得上，而不是"只算我这次的"。"""
        p = self.dir / "index.json"
        if not p.exists():
            return
        try:
            old = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return
        for it in old:
            if not isinstance(it, dict) or "seq" not in it:
                continue
            self.items.append(it)
            self.seen.add(int(it["seq"]))
        if self.items:
            self.say(f"[归档] 目录里已有 {len(self.items)} 帧（并入本次索引，已落盘的不重复下载）")

    def poll(self, follow: bool) -> int:
        """取一轮清单并落盘新帧，返回本次新收的帧数（含"该轮无画面"的记账）。

        follow=True 且已做过首次清单时走长轮询：请求挂在板端，一有新定案帧立刻回来。
        """
        path = f"/ai_dump?after={self.after}" if (follow and self.after is not None) else "/ai_dump"
        t0 = time.monotonic()
        try:
            man = json.loads(get(self.host, path, port=self.port,
                                 timeout=FOLLOW_TIMEOUT_S if (follow and self.after is not None)
                                 else HTTP_TIMEOUT_S).decode("utf-8"))
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            self.held = False
            self.failed = True          # 板子问不到：调用方据此退避重试（held 只表示"板端挂起过"）
            self.say(f"[警告] 取清单失败：{e}")
            return 0
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            self.held = False
            self.failed = True
            self.say(f"[警告] 清单不是合法 JSON：{e}")
            return 0
        self.failed = False
        self.held = time.monotonic() - t0 >= HELD_MIN_S   # 耗时明显 = 板端真的挂起等了

        frames = [f for f in man.get("frames", []) if isinstance(f, dict) and f.get("seq")]
        frames.sort(key=lambda f: f["seq"])
        oldest, newest = (frames[0]["seq"], frames[-1]["seq"]) if frames else (0, 0)
        if self.after is None:
            # 首次清单：环形里现存的帧全部按序收下（这就是"半途起脚本也能拿到前几轮"）
            self.after = oldest - 1 if frames else 0
        elif frames and newest < self.after:
            # 板端序号倒退 = 板子重启/换过固件（序号从 1 重新发号）。不认这一步的话，水位压在旧序号
            # 上会**既收不到又看不出错**：长轮询一直空等到超时，画面明明在流动却一帧不进。这里当作
            # 新一场：覆盖得到的旧条目（同名文件马上会被新帧盖掉）先清出索引，免得同号两份。
            self.say(f"[警告] 板端序号退回到 {newest}（已收到 {self.after}）—— 板子重启或换了固件，"
                     f"画面从头发号。索引里序号 ≥ {oldest} 的旧条目已清掉（新场若不走到那些号，"
                     f"对应旧 .jpg 会留在目录里不再被引用）")
            self.items = [it for it in self.items if it["seq"] < oldest]
            self.seen = {it["seq"] for it in self.items}
            self.after = oldest - 1
        elif frames and oldest > self.after + 1:
            # 已经滚出环形、永远拿不到的那些：如实计数（复盘最怕"以为收全了"）
            self.skipped += oldest - self.after - 1
            self.after = oldest - 1

        new = 0
        for f in frames:
            seq = f["seq"]
            if seq <= self.after:
                continue
            if seq in self.seen:
                # 上一趟（或并入的索引）已经收过：水位照推 —— 否则长轮询会拿同一份清单反复空转
                self.after = seq
                continue
            if not self._save(f):
                break            # 这一帧没落成：水位停在它前面，下轮清单会把它再报一次
            self.after = seq
            new += 1
        return new

    def _save(self, f: dict) -> bool:
        seq, ms = f["seq"], int(f.get("ms", 0))
        n, prev, note = f.get("len", 0), f.get("prev", False), f.get("note", "")
        if n <= 0:
            self.say(f"#{seq}  <本轮无画面>  {note}")   # 那轮 AI 是瞎着决策的，如实记下
            self.items.append({"seq": seq, "ms": ms, "bytes": 0, "prev": prev,
                               "note": note, "file": None})
            self.seen.add(seq)
            return True
        try:
            data = get(self.host, f"/ai_frame?seq={seq}", port=self.port)
        except (urllib.error.URLError, OSError, TimeoutError) as e:
            self.say(f"#{seq}  取图失败：{e}")
            return False
        # 文件名前缀 = 板端序号：与日志同一场对号入座，跳号即漏收（或那一轮没画面）
        name = f"{seq:04d}-{safe_name(note)}.jpg"
        try:
            (self.dir / name).write_bytes(data)
        except OSError as e:
            self.say(f"#{seq}  写盘失败：{e}")
            return False
        bad = "" if is_jpeg(data) else "  ⚠不是完整 JPEG(截断?)"
        self.say(f"#{seq}  {len(data)/1024:5.1f}KB{'  prev' if prev else '      '}  {note}{bad}")
        self.items.append({"seq": seq, "ms": ms, "bytes": len(data), "prev": prev,
                           "note": note, "file": name})
        self.seen.add(seq)
        return True

    def write_index(self) -> None:
        items = sorted(self.items, key=lambda x: x["seq"])   # 快照：排序后不再读原表
        (self.dir / "index.json").write_text(
            json.dumps(items, ensure_ascii=False, indent=2), encoding="utf-8")
        (self.dir / "index.html").write_text(self._html(items), encoding="utf-8")

    def _html(self, items: list[dict]) -> str:
        cards = []
        for it in items:
            img = (f'<a href="{it["file"]}"><img src="{it["file"]}" loading="lazy"></a>'
                   if it["file"] else '<div class="noimg">（本轮无画面）</div>')
            head = f'#{it["seq"]} · 板端 {it["ms"]/1000:.1f}s · {it["bytes"]/1024:.1f}KB'
            if it["prev"]:
                head += " · 双帧"
            cards.append(
                f'<figure>{img}<figcaption><div class="h">{head}</div>'
                f'<div class="n">{_esc(it["note"])}</div></figcaption></figure>')
        warn = f" ⚠ 漏收 {self.skipped} 帧（滚出环形，拿不回来了）。" if self.skipped else ""
        log_line = f"同名日志：<code>{_esc(self.log_ref)}</code>。" if self.log_ref else ""
        return f"""<!doctype html><meta charset="utf-8">
<title>AI 抓帧留档 {self.dir.name}</title>
<style>
 body{{background:#14161a;color:#e6e6e6;font:14px/1.5 system-ui,"Microsoft YaHei",sans-serif;margin:24px}}
 h1{{font-size:16px;font-weight:600;margin:0 0 4px}}
 .sub{{color:#8b93a1;margin-bottom:20px}}
 .grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(360px,1fr));gap:18px}}
 figure{{margin:0;background:#1d2026;border:1px solid #2b3038;border-radius:8px;overflow:hidden}}
 img{{width:100%;display:block;background:#000}}
 .noimg{{padding:60px 0;text-align:center;color:#6b7280;background:#000}}
 figcaption{{padding:10px 12px}}
 .h{{color:#8b93a1;font-size:12px;margin-bottom:4px}}
 .n{{color:#e6e6e6;word-break:break-word}}
</style>
<h1>AI 抓帧留档 · {self.dir.name}</h1>
<div class="sub">共 {len(items)} 帧，按板端序号排列（文件名前缀即序号）。
图 = 该轮实际发往云端的字节；标注 = 它据此做了什么 / 为什么被拒。
{log_line}{warn}</div>
<div class="grid">{''.join(cards)}</div>
"""


def _esc(s: str) -> str:
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
             .replace('"', "&quot;"))


class Frames:
    """抓帧留档，跑在**自己的线程**里（与 WS 会话完全解耦）。

    为什么必须另起线程：长轮询是**故意挂住**的（板端最多 15s 才回），塞进 asyncio 主循环会把 WS
    的心跳与收包一起憋住 —— 而板端对"连续 6s 没有上行"的客户端会广播探测，多轮无应答就把**所有**
    客户端一起踢。放进线程后，板子断线重连、甚至 WS 完全连不上，画面都照收。
    """

    def __init__(self, host: str, outdir: Path, log_ref: str, args, wr: Writer, st: Stats) -> None:
        self.outdir = outdir
        self.st = st
        self.follow = not args.no_follow
        self.interval = args.interval
        self.deadline = None            # 与 --for 同一个截止点（由主流程设）
        self.stop = threading.Event()
        self.t0 = time.monotonic()
        self.fails = 0                  # 连续问不到板子的次数（用来退避）
        self.hinted = False
        self.col = Collector(outdir, host, report=lambda t: emit(wr, "抓帧", t, st),
                             log_ref=log_ref, port=args.http_port)
        self._th = threading.Thread(target=self._loop, name="frames", daemon=True)

    def start(self) -> None:
        self._th.start()

    def stop_and_join(self, timeout: float = 2.0) -> bool:
        """请抓帧线程收工，返回是否真停了。

        长轮询可能正挂在板端（最多 25s 才回），所以只等 2s：每收到一帧就已经落盘了，等不到的只是
        "最后一帧还没写进索引"，下一趟并入索引时会补上。之后进程就 os._exit，卡在 read 里的守护
        线程随之消失 —— 不为了它多等 20 秒才收工。
        """
        self.stop.set()
        self._th.join(timeout)
        return not self._th.is_alive()

    def finalize(self, stopped: bool) -> None:
        self.col.write_index()
        n_img = sum(1 for it in self.col.items if it.get("file"))
        tail = ""
        if self.col.skipped:
            tail += f"（漏收 {self.col.skipped} 帧：滚出环形了，本脚本一直挂着就不会漏）"
        if not stopped:
            tail += "（抓帧线程仍挂在长轮询里，索引按当前状态写出）"
        self.col.say(f"[完成] 抓帧 {n_img} 张 → {self.outdir}{tail}")
        if n_img:
            self.col.say(f"       缩略图页 {self.outdir / 'index.html'}")

    def _loop(self) -> None:
        while not self.stop.is_set():
            if self.deadline and time.monotonic() >= self.deadline:
                break
            n = self.col.poll(self.follow)
            self.col.write_index()
            if self.col.failed:
                # 板子问不到（重启中/掉线）：退避重试。警告只在这一路上打，退到 10s 就不会刷屏
                self.fails += 1
                self.stop.wait(min(2.0 ** (self.fails - 1), FRAMES_BACKOFF_MAX_S))
                continue
            self.fails = 0
            if (not self.hinted and not self.col.items and not self.col.held
                    and self.st.sessions > 0 and time.monotonic() - self.t0 >= FRAMES_MIN_RUN_S):
                # 板端"没挂起"就答没有：留档没开、或固件不支持长轮询。只说一次然后照常问 —— 后者
                # 要继续等，前者用户自己会去开开关。要等 st.sessions>0：开关是本脚本刚开的，连上之前
                # 就说"板端没有留档"是冤枉它。
                self.col.say(_NOTHING_HINT)
                self.hinted = True
            if not self.col.held and not self.stop.is_set():
                self.stop.wait(self.interval)   # 板端没挂起（旧固件/留档关着）：退化成定时问


# ---------------- WS 会话 ----------------

class LinkDead(Exception):
    pass


class TimeUp(Exception):
    pass


async def run_session(host: str, args, wr: Writer, st: Stats) -> None:
    url = f"ws://{host}:{WS_PORT}/"
    # ping_interval=None 是必须的：websockets 默认每 20s 发 WS 控制帧 PING 并等 PONG，而板端
    # 是 handle_ws_control_frames=false（控制帧由 WS 层自理，不一定回），库会误判超时后自己断开。
    # 活体改走板端自己的文本协议 {"type":"ping"} ↔ {"type":"pong"}（与手机端一致）。
    async with connect(url, ping_interval=None, open_timeout=args.timeout,
                       close_timeout=2, max_size=2 ** 22) as ws:
        log(f"\n[连上] {url}")
        st.ws_last_rx = time.monotonic()
        st.ws_up = True
        # 开板端日志转发（全局单选开关；重连后重发一次是幂等的）
        want_log = json.dumps({"type": "log", "params": {"cat": args.cat, "on": True}})
        await ws.send(want_log)
        st.enabled = True
        st.sessions += 1
        if args.header:
            stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            wr.file_only(f"# car_logcat {host}:{WS_PORT} cat={args.cat} 开始于 {stamp}")
        pending = None      # 已发 ping、还没等到任何下行的时刻
        miss = 0
        last_ping = 0.0
        last_assert = time.monotonic()
        while True:
            if st.deadline and time.monotonic() >= st.deadline:
                raise TimeUp()           # --for 到点：主动退出（不用 wait_for 取消, 收尾才不会被取消掉）
            if time.monotonic() - last_assert >= REASSERT_S:
                # 手机端动过 /log（全局单选）会把类别改掉/关掉，这里重申一次，避免"连着却什么都没记"
                last_assert = time.monotonic()
                await ws.send(want_log)
            if time.monotonic() - last_ping >= PING_EVERY_S:
                last_ping = time.monotonic()
                if pending is None:
                    pending = last_ping
                await ws.send('{"type":"ping"}')
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=RECV_TICK_S)
            except asyncio.TimeoutError:
                if pending is not None and time.monotonic() - pending >= PONG_TIMEOUT_S:
                    miss += 1
                    pending = None
                    if miss >= MAX_MISS:
                        raise LinkDead(f"连续 {miss} 次心跳无应答")
                    last_ping = 0.0      # 立刻补发一次
                continue
            pending = None               # 任何下行都算链路活着
            miss = 0
            st.ws_last_rx = time.monotonic()   # 兜底通道据此判"WS 还活着吗"
            if isinstance(raw, (bytes, bytearray)):
                st.frames += 1           # 二进制帧（图传/编辑图）不落日志
                continue
            st.frames += 1
            try:
                m = json.loads(raw)
            except ValueError:
                emit(wr, "raw", raw, st)
                continue
            if not isinstance(m, dict):
                emit(wr, "raw", raw, st)
                continue
            if m.get("type") == "ping":   # 板端探测帧：回一条（板端把任何上行都算活动）
                await ws.send('{"type":"pong"}')
                continue
            handle_frame(m, raw, wr, args, st)


async def send_once(host: str, payload: str, timeout: float = 3.0) -> bool:
    """短连接发一条就走（退出时复位 /log 用）。"""
    try:
        async with connect(f"ws://{host}:{WS_PORT}/", ping_interval=None,
                           open_timeout=timeout, close_timeout=2) as ws:
            await ws.send(payload)
            await asyncio.sleep(0.2)     # 留一拍让板端收下再关
        return True
    except (OSError, ConnectionClosed, asyncio.TimeoutError, TimeoutError):
        return False


# ---------------- 主流程 ----------------

async def ble_session(args, wr: Writer, st: Stats) -> None:
    """BLE 兜底日志通道：**只在 WS 静默时**才接上。

    板端 blog 的转发器是 WS+BLE 双发的（同一条 {type:"log"} 文本），同时收就是每行记两遍；
    所以这里做成"待机"——WS 一静默（断链 / 射频卡死）才连上，WS 一回来就断开。于是日志里
    `[蓝牙]` 开头那几行**就是**断开区间本身。

    这段正是只收 WS 时永远拿不到的证据：WiFi 已经发不出去了（`发0`、不回 ARP、ping 都不通），
    日志却还能从蓝牙出来 —— 那就说明 CPU/固件/日志队列都好着，坏的只是 WiFi 那一侧。
    卡死前后板子到底说了什么（含每轮的 heap= 数字），全留在这几行里。
    """
    try:
        import bleak
    except ImportError:
        emit(wr, "蓝牙", "没装 bleak，兜底通道不可用（用 uv run 跑本脚本会自动装）", st)
        return

    def hit(_d, ad) -> bool:
        return ((ad.local_name or "") == BLE_NAME
                or any(u.lower() == BLE_SVC for u in (ad.service_uuids or [])))

    # 开局先给 WS 一个连上的窗口：ws_up 要连上才为真，没有这段宽限会一上来就抢兜底通道
    t_boot = time.monotonic()
    while not st.ws_up and time.monotonic() - t_boot < BLE_AFTER_S:
        if st.deadline and time.monotonic() >= st.deadline:
            return
        await asyncio.sleep(0.5)

    while True:
        if st.deadline and time.monotonic() >= st.deadline:
            return
        if st.ws_up:                                         # WS 活着：待机，不连
            await asyncio.sleep(1.0)
            continue
        try:
            dev = await bleak.BleakScanner.find_device_by_filter(hit, timeout=args.ble_scan)
            if dev is None:
                emit(wr, "蓝牙", f"WS 静默但也没扫到 {BLE_NAME} 广播 —— 没上电 / 电池没电 / 固件崩了", st)
                await asyncio.sleep(5.0)
                continue

            def on_note(_c, data: bytearray) -> None:
                raw = data.decode("utf-8", "replace")
                try:
                    m = json.loads(raw)
                except ValueError:
                    m = None
                if not isinstance(m, dict):
                    st.ble_lines += 1
                    emit(wr, "蓝牙", raw, st)
                    return
                if m.get("type") == "ping":
                    return                  # 板端活体探测：BLE 上没有"回"的概念，忽略
                before = st.lines
                handle_frame(m, raw, wr, args, st)
                if st.lines > before:
                    st.ble_lines += 1

            async with bleak.BleakClient(dev, timeout=15.0) as cli:
                await cli.start_notify(BLE_ST, on_note)
                emit(wr, "蓝牙", f"WS 已断 —— 已接上兜底通道 {dev.address}"
                                 f"（以下行都是从蓝牙来的）", st)
                # 顺手重申一次转发开关：板端 /log 是全局单选的，WS 死着就没别人替它重申了
                try:
                    await cli.write_gatt_char(
                        BLE_CMD,
                        json.dumps({"type": "log",
                                    "params": {"cat": args.cat, "on": True}}).encode(),
                        response=True)
                except Exception:
                    pass                    # 重申失败不影响收：开关多半本来就是开的
                while True:
                    await asyncio.sleep(0.5)
                    if st.deadline and time.monotonic() >= st.deadline:
                        return
                    if st.ws_up:
                        emit(wr, "蓝牙", "WS 已恢复 —— 断开兜底通道（同一行日志别记两遍）", st)
                        break
        except Exception as e:              # bleak 异常类型很杂；兜底通道不该把整场会话带走
            emit(wr, "蓝牙", f"通道出错：{type(e).__name__}: {e}", st)
            await asyncio.sleep(5.0)


async def _ws_loop(host: str, args, wr: Writer, st: Stats) -> None:
    """WS 主通道：断了就退避重连，直到 --for 到点。"""
    delay = 1.0
    while True:
        started = time.monotonic()
        try:
            await run_session(host, args, wr, st)
            delay = 1.0                  # 板端主动关了连接：立刻重连
        except TimeUp:
            break                        # --for 到点：正常收工
        except (OSError, ConnectionClosed, LinkDead, asyncio.TimeoutError) as e:
            # 稳定跑过一阵再断的，退避从 1s 重来（否则一个长会话反复断几次后要等满 10s）
            if time.monotonic() - started >= STABLE_S:
                delay = 1.0
            log(f"[断开] {type(e).__name__}: {e}")
            if st.deadline and time.monotonic() + delay > st.deadline:
                break                    # 退避会跨过截止点：直接收工，别让 --for 超差
            await asyncio.sleep(delay)
            delay = min(delay * 2, 10.0)
        finally:
            # 无论正常收尾、超时收工还是异常退出，此刻都没有 WS 下行了。
            # 兜底通道据此立刻启用（断连瞬间就该接上，不必再等一个静默窗口）。
            st.ws_up = False
        log(f"[重连] {host}")


async def collect(host: str, args, wr: Writer, st: Stats) -> None:
    """WS 主通道 + BLE 兜底通道并行；两条链路的日志落进**同一份**文件。"""
    t0 = time.monotonic()
    st.deadline = (t0 + args.for_s) if args.for_s > 0 else None
    st.ws_last_rx = t0
    if args.ble:
        emit(wr, "蓝牙", f"兜底通道待机中（WS 一断就自动接上，--no-ble 可关）", st)
    else:
        emit(wr, "蓝牙", "按 --no-ble 关闭 —— WiFi 一断就再也收不到日志了", st)
    jobs = [_ws_loop(host, args, wr, st)]
    if args.ble:
        jobs.append(ble_session(args, wr, st))
    await asyncio.gather(*jobs)

    extra = ""
    if wr.dropped:
        extra += f" 落盘丢弃 {wr.dropped} 行"
    if wr.lost_echo:
        extra += f" 回显丢弃 {wr.lost_echo} 行(控制台堵塞)"
    summary = (f"时长 {time.monotonic() - t0:.0f}s 会话 {st.sessions} 次 收帧 {st.frames} 条 "
               f"落盘 {st.lines} 行 丢弃自身回声 {st.filtered} 行{extra}")
    wr.file_only(f"# 统计 {summary} (结束于 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')})")
    log(f"\n[统计] {summary}")   # 落盘留痕那份写在文件里：丢了行也要能从文件本身看出来


def parse_args():
    ap = argparse.ArgumentParser(
        description="直连小车：板端日志落盘 + AI 每轮画面留档（手机侧可同时照旧控制）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  uv run tools/car_logcat.py                    # 自动找车，日志+抓帧一起收\n"
               "  uv run tools/car_logcat.py -H 192.168.1.23 -o log.txt\n"
               "  uv run tools/car_logcat.py --cat ai --for 120 --open\n"
               "  uv run tools/car_logcat.py --no-frames        # 只记日志\n")
    ap.add_argument("-H", "--host", help="板子 IP/主机名（省略则用缓存或扫网段）")
    ap.add_argument("-o", "--out", help="日志文件（默认 <本工程>/logs/car-<时间戳>.log，追加写）")
    ap.add_argument("--cat", default="all", choices=("all", "ai", "exec"),
                    help="板端转发类别（全局单选，默认 all）；抓帧跟着其中的 ai 类别走")
    ap.add_argument("--for", dest="for_s", type=float, default=0,
                    help="只记这么多秒后自动收工（默认一直记到 Ctrl+C）")
    ap.add_argument("--timeout", type=float, default=5.0, help="WS 建连超时秒（默认 5）")
    ap.add_argument("--no-scan", action="store_true", help="缓存失效时不要扫网段，直接报错")
    ap.add_argument("--keep-echo", dest="keep_echo", action="store_true",
                    help="保留 ping/log 的指令回显（默认丢弃，否则每 30s 一条自噪声）")
    ap.add_argument("--leave-on", action="store_true", help="退出时不复位 /log（保留转发与留档开启）")
    ap.add_argument("--no-header", dest="header", action="store_false", help="不写文件头注释行")
    ap.add_argument("-q", "--quiet", action="store_true", help="不往控制台回显日志")
    ap.add_argument("--no-frames", dest="frames", action="store_false",
                    help="不取板端留档的 AI 画面，只记日志")
    ap.add_argument("--frames-dir", help="抓帧归档目录（默认 <日志同级>/aiframes/<日志名>）")
    ap.add_argument("--no-follow", action="store_true",
                    help="抓帧不用长轮询（改按 --interval 定时问）；板端很旧或长轮询异常时的退路")
    ap.add_argument("--interval", type=float, default=1.0, metavar="SEC",
                    help="抓帧不用长轮询时的轮询周期（默认 1s）")
    ap.add_argument("--open", action="store_true", help="收工后用浏览器打开缩略图页")
    ap.add_argument("--no-ble", dest="ble", action="store_false",
                    help="关掉 BLE 兜底通道（默认开：WiFi 卡死时只有它还能把日志带出来）")
    ap.add_argument("--ble-scan", type=float, default=10.0, metavar="SEC",
                    help="BLE 扫描窗口秒（默认 10）")
    ap.add_argument("--http-port", type=int, default=HTTP_PORT, metavar="PORT",
                    help=f"板端 HTTP 端口（默认 {HTTP_PORT}；只有拿桩板端在本地测本脚本时才要改）")
    return ap.parse_args()


def main() -> None:
    global _sink
    args = parse_args()
    try:  # Windows 控制台默认 GBK，板端中文/控制字符直接 print 会抛 UnicodeEncodeError
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, OSError):
        pass
    strip_proxy_env()

    host = resolve_host(args)
    remember_host(host)

    if args.out:
        out_path = Path(args.out)
    else:
        # 锚定到本脚本所在工程（Stm32-Vision/logs/），不管从哪个目录调用都落在同一处
        out_path = (Path(__file__).resolve().parent.parent / "logs"
                    / f"car-{datetime.now().strftime('%Y%m%d-%H%M%S')}.log")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    wr = Writer(out_path, echo=not args.quiet)
    _sink = wr
    log(f"[输出] {out_path.resolve()}")
    log("[提示] 手机可同时控制；Ctrl+C 收工")

    st = Stats()

    # ---- 抓帧留档：与日志同起同收，归档目录就是这份日志的名字 ----
    # 这几条（尤其"为什么这场没有图"）都用 emit 走日志文件：事后翻日志时，缺图的原因本身也是证据。
    fr = None
    frames_dir = None
    if not args.frames:
        emit(wr, "抓帧", "按 --no-frames 跳过（本次只记日志）", st)
    elif args.cat == "exec":
        # 留档跟着板端 AI 类别走：--cat exec 时板端根本不 memcpy，抓也只会一直空等
        emit(wr, "抓帧", "跳过：--cat exec 不含 ai 类别，板端不会留画面"
                        "（改用 --cat ai 或 all，或去掉 --no-frames）", st)
    elif not has_dump(host, args.http_port):
        emit(wr, "抓帧", f"跳过：{host} 的 /ai_dump 不可用（404 或超时）——多半是板上固件还没有"
                        "抓帧留档；本次只记日志", st)
    else:
        frames_dir = frames_dir_for(out_path, args.frames_dir)
        fr = Frames(host=host, outdir=frames_dir, args=args, wr=wr, st=st,
                    log_ref=str(args.out or f"logs/{out_path.stem}.log"))
        fr.deadline = (time.monotonic() + args.for_s) if args.for_s > 0 else None
        emit(wr, "抓帧", f"板子 {host}:{args.http_port}，归档 {frames_dir}"
                        f"（{'长轮询' if fr.follow else f'定时问 {args.interval}s'}）", st)
        fr.start()

    try:
        # 时长控制交给 collect 内部的 st.deadline：用 wait_for 取消任务会让 await 收尾
        # （复位 /log）一起被取消，Ctrl+C 同理 —— 所以收尾一律放到 asyncio.run 之外做。
        asyncio.run(collect(host, args, wr, st))
    except KeyboardInterrupt:
        pass

    if fr is not None:
        # 先停抓帧再复位 /log（复位会释放板端留档，停晚了会扑空）。--for 到点时抓帧线程自己也到点了。
        stopped = fr.stop_and_join()
        fr.finalize(stopped)
        if args.open and frames_dir is not None and (frames_dir / "index.html").exists():
            webbrowser.open((frames_dir / "index.html").as_uri())

    # 复位 /log：板端是全局单选的转发开关，不复位手机端会一直刷本脚本开的类别
    if st.enabled and not args.leave_on:
        ok = asyncio.run(send_once(host, json.dumps({"type": "log", "params": {"cat": args.cat, "on": False}})))
        log("[收尾] 已把 /log 复位为关闭" if ok else "[收尾] /log 复位失败（板子没应答，可在手机上发 /log off）")
    elif st.enabled:
        log("[收尾] 按 --leave-on 保留日志转发开启")

    wr.note(f"[完成] {out_path.resolve()}")   # 排进队列，控制台被堵死也不阻塞退出
    wr.close()          # 等落盘线程把队列排空（此时文件已完整，回显队列只给 1s）

    # 硬退：控制台若正被堵死，回显线程会卡在 print 里握着 sys.stdout 的缓冲锁，解释器收尾时
    # flush 抢不到锁就报 `Fatal Python error: _enter_buffered_busy`（难看且像崩溃）。要写的东西
    # 上面都已落盘/已 flush，直接跳过解释器收尾最干净。
    os._exit(0)


if __name__ == "__main__":
    main()
