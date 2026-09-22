# car_logcat 的宿主端测试：不用板子、不用联网，桩板端 + 真模块 + 真子进程跑一遍 CLI。
#   uv run tools/test_car_logcat.py
#
# 覆盖：回填现有环形、长轮询挂起与唤醒、漏收计数、旧固件退化、板子问不到的退避、板端序号倒退、
#       归档目录由日志名推出、Frames 线程端到端（帧行同时进日志文件）、CLI 端到端与开关。
#
# 为什么值得留着：抓帧这条路（板端 ai_dump → /ai_dump 长轮询 → 落盘）没有板子也能验，改动它之后
# 先跑这个，比车上一趟趟试快得多。
import importlib.util
import json
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from argparse import Namespace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
TOOL = HERE / "car_logcat.py"
PROJ = HERE.parent
TMP = Path(tempfile.gettempdir()) / "car_logcat_test"

# car_logcat 顶层 import websockets（真收日志要它）。本测试只测抓帧那半边，且不想为此联网装依赖，
# 所以塞个空壳进 sys.modules（子进程那几条用例跑的是真 uv run，依赖用缓存里的）。
if "websockets" not in sys.modules:
    import types
    _ws = types.ModuleType("websockets")
    _a = types.ModuleType("websockets.asyncio")
    _c = types.ModuleType("websockets.asyncio.client")
    _e = types.ModuleType("websockets.exceptions")

    class ConnectionClosed(Exception):
        pass

    _c.connect = lambda *a, **k: None
    _e.ConnectionClosed = ConnectionClosed
    _ws.asyncio, _a.client = _a, _c
    sys.modules.update({"websockets": _ws, "websockets.asyncio": _a,
                        "websockets.asyncio.client": _c, "websockets.exceptions": _e})

spec = importlib.util.spec_from_file_location("logcat", TOOL)
logcat = importlib.util.module_from_spec(spec)
spec.loader.exec_module(logcat)

JPEG = b"\xff\xd8" + b"z" * 300 + b"\xff\xd9"
fails = []


def check(cond, msg):
    print(("  OK  " if cond else "  !!  ") + msg)
    if not cond:
        fails.append(msg)


class Board:
    """桩板端。frames 按 seq 升序；note 为空串表示那一轮没画面。"""

    def __init__(self, support_follow=True, hold_s=2.0, alive=True, garbage=False):
        self.frames = []
        self.support_follow = support_follow
        self.hold_s = hold_s
        self.alive = alive
        self.garbage = garbage                      # 回一份不是 JSON 的东西（量退避用）
        self.frame_hits = 0
        self.dump_queries = []
        self.lock = threading.Lock()

    def set(self, *pairs):
        with self.lock:
            self.frames = [{"seq": s, "note": n, "prev": False, "len": 0 if not n else len(JPEG)}
                           for s, n in pairs]

    def add(self, seq, note):
        with self.lock:
            self.frames.append({"seq": seq, "note": note, "prev": False, "len": len(JPEG)})

    def newest(self):
        with self.lock:
            return max([f["seq"] for f in self.frames], default=0)

    def manifest(self):
        with self.lock:
            fr = sorted(self.frames, key=lambda f: -f["seq"])   # 板端是"最新在前"
            return {"slots": 6, "n": len(fr),
                    "frames": [{"seq": f["seq"], "ms": 1234 * f["seq"], "len": f["len"],
                                "prev": f["prev"], "note": f["note"]} for f in fr]}


class Handler(BaseHTTPRequestHandler):
    board = None

    def log_message(self, *a):
        pass

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        b = Handler.board                       # 每次请求现取，方便测试中途换板子
        if u.path == "/ai_dump":
            after = q.get("after", [None])[0]
            with b.lock:
                b.dump_queries.append(after)
            if not b.alive:                     # 模拟板子在重启：连接直接断
                self.close_connection = True
                return
            if after is not None and b.support_follow:
                t0 = time.monotonic()           # 板的挂起语义：等到有新帧或超时
                while b.newest() <= int(after) and time.monotonic() - t0 < b.hold_s:
                    time.sleep(0.05)
            body = b"<html>502 bad gateway</html>" if b.garbage else json.dumps(b.manifest()).encode()
        elif u.path == "/ai_frame":
            with b.lock:
                b.frame_hits += 1
            body = JPEG
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def files_of(d):
    return sorted(p.name for p in Path(d).glob("*.jpg"))


def index_of(d):
    return json.loads((Path(d) / "index.json").read_text(encoding="utf-8"))


def dead_port():
    """拿一个刚被释放的端口号当"板子不在"（连接必被拒）。"""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def main():
    if TMP.exists():
        shutil.rmtree(TMP)
    TMP.mkdir(parents=True)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[桩板端] HTTP 127.0.0.1:{port}（只实现 /ai_dump 与 /ai_frame；81 端口故意不提供）")

    # ---- 1. 新固件：首次回填 + 长轮询挂起 + 编号 ----
    print("[1] 新固件：回填现有环形 → 长轮询等到新帧")
    b = Board(support_follow=True, hold_s=2.0)
    Handler.board = b
    b.set((3, "执行 arm_pose x=9 h=1.5"),
          (4, '被闸门拒(未执行) arm clip: 还没对准 —— 目标 "黄色方块" 在左9cm 前13cm'),
          (5, ""))                              # 5 = 那轮没画面
    d1 = TMP / "car-20260921-170352"
    col = logcat.Collector(d1, "127.0.0.1", port=port)
    n = col.poll(follow=True)
    check(n == 3, f"首次回填 3 条记账（含无画面那轮），实际 {n}")
    check(col.after == 5, f"水位推到 5，实际 {col.after}")
    check(col.held is False and col.failed is False, "首次清单不带 after → 板端不挂起、无错")
    check(b.dump_queries[0] is None, "首次请求确实没带 after")
    names = files_of(d1)
    check(len(names) == 2, f"无画面那轮不落 jpg，落了 {len(names)} 个")
    check(names[0].startswith("0003-") and names[1].startswith("0004-"),
          f"文件名以板端序号编号且补零：{names}")
    check('"' not in names[1] and "还没对准" in names[1], f"标注进文件名且引号已净化：{names[1]}")
    check((d1 / names[0]).read_bytes() == JPEG, "落盘的正是板端那份字节")

    col.poll(follow=True)                        # 无新帧：板端挂 2s 后回同一份清单
    check(col.held is True, "板端挂起过 → held=True（判定支持长轮询）")
    check(col.after == 5, "无新帧时水位不动")
    threading.Timer(0.3, lambda: b.add(6, "执行 arm low")).start()
    t0 = time.monotonic()
    n = col.poll(follow=True)
    dt = time.monotonic() - t0
    check(n == 1, f"挂起期间新帧到达 → 收到 1 帧，实际 {n}")
    check(dt < 1.5, f"新帧一到就返回，没干等满 hold_s（{dt:.2f}s）")
    check(files_of(d1)[-1].startswith("0006-"), f"新帧同样按序号命名：{files_of(d1)}")
    check(b.dump_queries[-1] == "5", f"长轮询带上了水位 after=5，实际 {b.dump_queries[-1]}")
    col.write_index()
    check([i["seq"] for i in index_of(d1)] == [3, 4, 5, 6], f"索引按序号列出：{[i['seq'] for i in index_of(d1)]}")
    check(index_of(d1)[2]["file"] is None and index_of(d1)[2]["bytes"] == 0, "无画面那轮在索引里留痕")
    check("共 4 帧" in (d1 / "index.html").read_text(encoding="utf-8"), "缩略图页统计正确")

    # ---- 2. 漏收计数 ----
    print("[2] 漏收计数")
    hits_before = b.frame_hits
    b.set((8, "执行 arm low"), (9, "执行 arm clip"))   # 6 收过，7 已被挤掉
    col.skipped = 0
    n = col.poll(follow=True)
    check(col.skipped == 1, f"7 滚出环形 → 报漏收 1，实际 {col.skipped}")
    check(n == 2 and col.after == 9, f"剩下两帧照收并推水位，n={n} after={col.after}")
    check(b.frame_hits - hits_before == 2, "只取了两张图，没重复下载")
    b.set((12, "执行 arm low"), (13, "执行 arm fold"))   # 这次一口气挤掉 10、11 两帧
    col.skipped = 0
    col.poll(follow=True)
    check(col.skipped == 2, f"一次挤掉两帧也如实计数，实际 {col.skipped}")
    check(col.after == 13, f"水位推到 13，实际 {col.after}")

    # ---- 3. 旧固件（不认识 after）：不挂起，退化成定时问也能收 ----
    print("[3] 旧固件退化")
    b2 = Board(support_follow=False, hold_s=2.0)
    Handler.board = b2
    d3 = TMP / "car-20260921-180000"
    col3 = logcat.Collector(d3, "127.0.0.1", port=port)
    col3.poll(follow=True)                       # 首次回填（空环形）
    check(col3.after == 0, f"空环形时水位从 0 起算，实际 {col3.after}")
    t0 = time.monotonic()
    n = col3.poll(follow=True)
    dt = time.monotonic() - t0
    check(n == 0 and col3.held is False and dt < 1.0, f"旧固件立即应答（{dt:.2f}s）→ 调用方按 --interval 退避")
    b2.add(1, "执行 spin -1")
    check(col3.poll(follow=True) == 1, "退化路径下照样收到新帧")
    check(files_of(d3) == ["0001-执行_spin_-1.jpg"], f"文件名：{files_of(d3)}")

    # ---- 4. 同一场跑第二次：并入索引，已落盘的不重复下载 ----
    print("[4] 重复运行可加")
    Handler.board = b
    b.set((8, "执行 arm low"), (9, "执行 arm clip"), (10, "执行 arm fold"))
    col.write_index()                            # d1 索引：3,4,5,6,8,9,12,13
    col4 = logcat.Collector(d1, "127.0.0.1", port=port)
    check(len(col4.items) == len(col.items), f"并入已有索引 {len(col4.items)} 条")
    hits_before = b.frame_hits
    n = col4.poll(follow=True)
    check(n == 1, f"环形里 8、9 已收过，只补新来的 10，实际 {n}")
    check(b.frame_hits - hits_before == 1, "已收过的不重复取图")
    check(col4.after == 10, f"水位推到 10（已收过的也照推，否则长轮询会空转），实际 {col4.after}")
    b.add(11, "执行 arm clip")
    check(col4.poll(follow=True) == 1, "接着收新帧")
    check(b.dump_queries[-1] == "10", f"长轮询带上了 10，实际 {b.dump_queries[-1]}")
    col4.write_index()
    seqs = [i["seq"] for i in index_of(d1)]
    check(len(seqs) == len(col4.items) and 11 in seqs and seqs == sorted(seqs),
          f"索引把两趟的条目并在一起且有序：{seqs}")

    # ---- 5. 板子重启/换固件：序号倒退不能让脚本"既收不到又看不出错" ----
    print("[5] 板端序号倒退（重启/换固件）")
    b.set((1, "执行 arm low"), (2, "执行 arm clip"))     # 新固件重启，序号从 1 重新发号
    n = col4.poll(follow=True)
    check(n == 2, f"倒退后照收新时间线的帧，实际 {n}")
    check(col4.after == 2, f"水位重挂到新序号上，实际 {col4.after}")
    check(len([i for i in col4.items if i["seq"] >= 1]) == 2, "旧条目已清，不出现同号两份")
    b.add(3, "执行 arm fold")
    check(col4.poll(follow=True) == 1 and col4.after == 3, "之后继续正常跟")

    # ---- 6. 板子问不到（重启/掉线）：poll 不抛、failed 置位（调用方据此退避） ----
    print("[6] 板子问不到")
    b3 = Board(alive=False)
    Handler.board = b3
    col6 = logcat.Collector(TMP / "car-deadbeef", "127.0.0.1", port=port)
    n = col6.poll(follow=True)                   # 连接被板端关掉 → URLError/远端断开
    check(n == 0 and col6.failed is True and col6.held is False,
          f"连接被断：n=0 failed=True（不抛异常），实际 n={n} failed={col6.failed}")
    col7 = logcat.Collector(TMP / "car-noport", "127.0.0.1", port=dead_port())
    n = col7.poll(follow=True)
    check(n == 0 and col7.failed is True, "端口没人听：同样只是 failed，不炸")
    check(col7.items == [] and files_of(TMP / "car-noport") == [], "什么都没有时不留空文件")

    # ---- 7. 归档目录由日志名推出（不再靠 mtime 猜配对）+ has_dump 能力探测 ----
    print("[7] 目录同名与能力探测")
    d = logcat.frames_dir_for(Path("logs/car-20260921-170352.log"))
    check(str(d).replace("\\", "/") == "logs/aiframes/car-20260921-170352",
          f"归档目录 = 日志同级 aiframes/<日志名>：{d}")
    d2 = logcat.frames_dir_for(Path("logs/car-20260921-170352.log"), "C:/tmp/myframes")
    check(str(d2).replace("\\", "/") == "C:/tmp/myframes", f"--frames-dir 覆盖生效：{d2}")
    Handler.board = b
    check(logcat.has_dump("127.0.0.1", port) is True, "桩板端 /ai_dump 在 → has_dump=True")
    check(logcat.has_dump("127.0.0.1", dead_port()) is False, "端口没人听 → has_dump=False")
    check(logcat.safe_name("被闸门拒(未执行) arm clip: 还没对准") == "被闸门拒(未执行)_arm_clip_还没对准",
          "文件名净化：冒号/空格处理正确")

    # ---- 8. Frames 线程端到端：帧行进日志文件、索引写出、停得干净 ----
    print("[8] Frames 线程（与 WS 解耦）")
    b4 = Board(support_follow=True, hold_s=1.0)
    Handler.board = b4
    b4.set((1, "执行 arm low"), (2, ""))
    run_log = TMP / "sess.log"
    wr = logcat.Writer(run_log, echo=False)
    st = logcat.Stats()
    st.sessions = 1                              # 假装 WS 已连上（提示语的门槛）
    fdir = TMP / "aiframes" / "car-cli"
    args8 = Namespace(no_follow=False, interval=0.2, http_port=port)
    fr = logcat.Frames(host="127.0.0.1", outdir=fdir, log_ref="logs/car-cli.log",
                       args=args8, wr=wr, st=st)
    fr.start()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and len(fr.col.items) < 2:
        time.sleep(0.05)
    # seq=2 那轮板端没留画面(len=0)，所以按 .jpg 只数得到 1 个 —— 记账按 items 数
    check(len(fr.col.items) == 2 and len(files_of(fdir)) == 1,
          f"线程独立把两轮收了（WS 那侧根本没连）：记账 {len(fr.col.items)} 条 / 图 {files_of(fdir)}")
    b4.add(3, "执行 arm clip")                   # 线程正挂在长轮询里 → 应当很快被叫醒
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and len(fr.col.items) < 3:
        time.sleep(0.05)
    check(len(fr.col.items) == 3 and len(files_of(fdir)) == 2,
          f"挂起期间新帧到达，线程自己醒来收下：记账 {len(fr.col.items)} 条 / 图 {files_of(fdir)}")
    stopped = fr.stop_and_join()
    check(stopped is True, "stop_and_join 在 2s 内停干净（长轮询 hold 只有 1s）")
    fr.finalize(stopped)
    wr.close()
    txt = run_log.read_text(encoding="utf-8")
    check("[抓帧] #1" in txt and "执行 arm low" in txt, "帧行走进了日志文件（[抓帧] 来源，与板端日志同一份）")
    check("[抓帧] #2" in txt and "<本轮无画面>" in txt, "无画面那轮也在日志里留痕")
    check("[抓帧] [完成] 抓帧 2 张" in txt, f"收工统计只数真落盘的图：{[l for l in txt.splitlines() if '完成' in l]}")
    check("logs/car-cli.log" in (fdir / "index.html").read_text(encoding="utf-8"),
          "缩略图页指向同名日志路径")
    check(fdir.name == "car-cli" and (fdir / "index.json").exists(), "索引落在 aiframes/<日志名>/ 下")

    # ---- 9. 板子一直答非所问：退避重试，而不是把警告刷满屏 ----
    # 用"回垃圾 JSON"而不是"端口没人听"来量：本机对一个**被拒绝**的端口也要 ~2s 才返回
    # （socket.create_connection 裸测同样 2.07s，属这台机器的网络栈），那样 4 秒里跑不了几次，
    # 量不出退避。答非所问是立刻返回的，反而能把"每次失败后的等待在变长"清清楚楚量出来。
    print("[9] 板子答非所问时的退避")
    b9 = Board(garbage=True)
    Handler.board = b9
    wr2 = logcat.Writer(TMP / "dead.log", echo=False)
    st2 = logcat.Stats()
    st2.sessions = 1
    args9 = Namespace(no_follow=False, interval=0.05, http_port=port)
    fr9 = logcat.Frames(host="127.0.0.1", outdir=TMP / "aiframes" / "car-dead",
                        log_ref="", args=args9, wr=wr2, st=st2)
    fr9.start()
    time.sleep(4.0)
    fr9.stop_and_join()
    wr2.close()
    warn_lines = [l for l in (TMP / "dead.log").read_text(encoding="utf-8").splitlines()
                  if "不是合法 JSON" in l]
    # 4s 窗口: 退避 1s→2s→4s 只容得下 3 次尝试；若没有退避（interval 0.05s）会是几十上百行
    check(fr9.fails >= 3, f"连续失败在累加：fails={fr9.fails}")
    check(len(warn_lines) <= 4, f"4s 内警告只有 {len(warn_lines)} 行（退避 1s→2s→4s，没有刷屏）")
    check(not (TMP / "aiframes" / "car-dead" / "index.json").read_text(encoding="utf-8").strip("[]\n "),
          "一帧没收到时索引是空表，不编造条目")

    # ---- 10. CLI 端到端：一条命令同时收日志与画面 ----
    print("[10] CLI 端到端（真子进程）")
    b5 = Board(support_follow=True, hold_s=0.5)
    Handler.board = b5
    b5.set((7, "执行 arm_pose x=9 h=1.5"), (8, '被闸门拒(未执行) arm clip: 还没对准'))
    cli_log = TMP / "cli" / "car-cli.log"
    r = subprocess.run(["uv", "run", str(TOOL), "-H", "127.0.0.1", "--http-port", str(port),
                        "--for", "3", "-o", str(cli_log)],
                       cwd=PROJ, capture_output=True, text=True, timeout=120)
    check(r.returncode == 0, f"进程正常收工（rc={r.returncode}）")
    check("抓帧 2 张" in r.stdout, f"控制台报了抓帧数：{[l for l in r.stdout.splitlines() if '抓帧' in l][:3]}")
    cli_frames = cli_log.parent / "aiframes" / "car-cli"
    names = files_of(cli_frames)
    check(len(names) == 2 and names[0].startswith("0007-") and names[1].startswith("0008-"),
          f"画面按板端序号落在 <日志同级>/aiframes/<日志名>/：{names}")
    body = cli_log.read_text(encoding="utf-8")
    check("[抓帧] #7" in body and "[抓帧] [完成] 抓帧 2 张" in body, "同一份日志文件里有抓帧记录")
    check("[抓帧] 板子 127.0.0.1" in body, "开机那行抓帧说明也落进了日志")
    check((cli_frames / "index.html").exists(), "缩略图页生成")
    check("[断开]" in r.stdout, "WS 连不上（桩板端没有 81 端口）→ 如实报断开，而抓帧照收（下面这条）")

    # ---- 11. CLI：--no-frames 只记日志；--cat exec 自动跳过抓帧 ----
    print("[11] CLI 开关")
    cli2 = TMP / "cli2" / "only.log"
    r = subprocess.run(["uv", "run", str(TOOL), "-H", "127.0.0.1", "--http-port", str(port),
                        "--for", "2", "--no-frames", "-o", str(cli2)],
                       cwd=PROJ, capture_output=True, text=True, timeout=120)
    check(r.returncode == 0, f"--no-frames 正常收工（rc={r.returncode}）")
    check(not (cli2.parent / "aiframes").exists(), "--no-frames 不建抓帧目录")
    check("--no-frames" in r.stdout, f"并说清了为什么没抓帧：{[l for l in r.stdout.splitlines() if '抓帧' in l][:1]}")
    check("--no-frames" in cli2.read_text(encoding="utf-8"),
          "这句原因也落进日志文件（事后翻日志时缺图的原因也是证据）")
    cli3 = TMP / "cli3" / "exec.log"
    r = subprocess.run(["uv", "run", str(TOOL), "-H", "127.0.0.1", "--http-port", str(port),
                        "--cat", "exec", "--for", "2", "-o", str(cli3)],
                       cwd=PROJ, capture_output=True, text=True, timeout=120)
    check("不含 ai 类别" in r.stdout, f"--cat exec 时跳过抓帧并说明原因：{[l for l in r.stdout.splitlines() if '抓帧' in l][:1]}")
    check(not (cli3.parent / "aiframes").exists(), "--cat exec 不建抓帧目录")

    print()
    if fails:
        print(f"!! {len(fails)} 项未过：")
        for f in fails:
            print("   - " + f)
        sys.exit(1)
    print(f"全部通过（临时目录 {TMP}）")


if __name__ == "__main__":
    main()
