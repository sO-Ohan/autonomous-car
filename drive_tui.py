#!/usr/bin/env python3
"""
Terminal console for the drive harness: closed-loop rotation, speed control,
and motor calibration.

    python3 drive_tui.py [/dev/ttyUSB0]

The firmware holds position with a PID loop at 200 Hz. This is the operator
side: it sends commands, shows live position/velocity/error, and records the
settling error of every move so you can see whether a gain change actually
helped rather than guessing.
"""

import sys
import time
import threading
import queue
import os
import termios
import tty
import select
from collections import deque

import serial
from rich.console import Console, Group
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.align import Align

CPR = 4096.0

HELP = [
    ("tab", "select wheel"), ("h / l", "rotate -15 / +15"),
    ("j / k", "rotate -90 / +90"), ("J / K", "rotate -360 / +360"),
    ("w / s", "duty +10 / -10"), ("space", "STOP"),
    ("z", "zero position"), ("c", "calibrate"),
    (":", "command"), ("q", "quit"),
]


class Link(threading.Thread):
    """Serial reader/writer. Parses the firmware's line protocol."""

    def __init__(self, port, logdir="logs"):
        super().__init__(daemon=True)
        self.port = port
        self.out = queue.Queue()
        self.running = True
        self.ser = None
        self.connected = False

        # Every byte in and out goes to disk. Debugging a motion problem after
        # the fact is impossible from an in-memory ring buffer.
        os.makedirs(logdir, exist_ok=True)
        self.logpath = os.path.join(
            logdir, time.strftime("session-%Y%m%d-%H%M%S.log"))
        self.logf = open(self.logpath, "a", buffering=1)
        self.logf.write(f"# drive_tui session {time.asctime()} port={port}\n")

        self.d = {}                      # latest D, telemetry frame
        self.s = {}                      # latest S, status frame
        self.log = deque(maxlen=200)
        self.cal = {"L": {}, "R": {}}
        self.moves = {"L": deque(maxlen=12), "R": deque(maxlen=12)}
        self.pending = {"L": None, "R": None}
        self.rate = 0.0
        self._n = 0
        self._t0 = time.time()

    def write_log(self, kind, msg):
        try:
            self.logf.write(f"{time.time():.3f} {kind} {msg}\n")
        except Exception:
            pass

    def send(self, line):
        self.out.put(line)
        self.log.append(("tx", line))
        self.write_log("TX", line)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, 115200, timeout=0.1)
        except Exception as e:
            self.log.append(("err", f"open {self.port}: {e}"))
            return

        self.ser.setDTR(False)
        self.ser.setRTS(True)
        time.sleep(0.15)
        self.ser.setRTS(False)
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.log.append(("sys", "board reset, waiting for boot scan"))
        time.sleep(5.5)
        self.ser.reset_input_buffer()
        self.connected = True
        self.send("T")                   # telemetry on

        buf = b""
        while self.running:
            try:
                while not self.out.empty():
                    self.ser.write((self.out.get_nowait() + "\n").encode())
                chunk = self.ser.read(512)
            except Exception as e:
                self.log.append(("err", str(e)))
                break
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.parse(line.decode("utf-8", "replace").strip())

        try:
            self.ser.write(b"X\nT\n")
            self.ser.close()
        except Exception:
            pass

    def parse(self, line):
        if not line:
            return
        self.write_log("RX", line)
        p = line.split(",")
        tag = p[0]

        if tag == "D" and len(p) >= 13:
            try:
                self.d = {
                    "L": {"pos": int(p[1]), "tgt": int(p[2]), "vel": float(p[3]),
                          "duty": int(p[4]), "err": int(p[5]), "mode": int(p[11])},
                    "R": {"pos": int(p[6]), "tgt": int(p[7]), "vel": float(p[8]),
                          "duty": int(p[9]), "err": int(p[10]), "mode": int(p[12])},
                }
                self._n += 1
                dt = time.time() - self._t0
                if dt > 0:
                    self.rate = self._n / dt
            except ValueError:
                pass

        elif tag == "S" and len(p) >= 15:
            try:
                self.s = {
                    "Lagc": int(p[1]), "Lstat": int(p[2]),
                    "Ragc": int(p[3]), "Rstat": int(p[4]),
                    "kp": float(p[5]), "ki": float(p[6]), "kd": float(p[7]),
                    "tol": int(p[8]),
                    "Ldf": int(p[9]), "Ldr": int(p[10]),
                    "Rdf": int(p[11]), "Rdr": int(p[12]),
                    "Lslope": float(p[13]), "Rslope": float(p[14]),
                }
            except ValueError:
                pass

        elif tag == "DONE" and len(p) >= 4:
            w = p[1]
            if w in self.moves:
                tgt = self.pending.get(w)
                self.moves[w].append((tgt, float(p[3])))
                self.pending[w] = None
            self.log.append(("done", f"{w} settled, residual {p[3]}°"))

        elif tag == "ACK":
            if len(p) >= 4 and p[1] == "move":
                self.pending[p[2]] = float(p[3])
            self.log.append(("ack", line))

        elif tag.startswith("CAL"):
            w = p[1] if len(p) > 1 else "?"
            c = self.cal.setdefault(w, {})
            if tag == "CALSTART":
                c.clear()
                c["running"] = True
                c["pts"] = []
            elif tag == "CALDEAD" and len(p) >= 4:
                c["fwd"], c["rev"] = int(p[2]), int(p[3])
            elif tag == "CALPT" and len(p) >= 4:
                c.setdefault("pts", []).append((int(p[2]), float(p[3])))
            elif tag == "CALSLOPE" and len(p) >= 3:
                c["slope"] = float(p[2])
            elif tag in ("CALDONE", "CALABORT"):
                c["running"] = False
            self.log.append(("cal", line))

        elif tag == "ERR":
            self.log.append(("err", line))
        else:
            self.log.append(("rx", line))

    def stop(self):
        self.running = False


def bar(value, lo, hi, width=18, char="█"):
    """Signed bar centred on zero when lo is negative."""
    span = hi - lo
    if span <= 0:
        return " " * width
    if lo < 0:
        mid = width // 2
        n = int(round(abs(value) / hi * mid))
        n = max(0, min(mid, n))
        if value >= 0:
            return " " * mid + char * n + " " * (mid - n)
        return " " * (mid - n) + char * n + " " * mid
    n = int(round((value - lo) / span * width))
    return char * max(0, min(width, n)) + " " * (width - max(0, min(width, n)))


class App:
    def __init__(self, port):
        self.link = Link(port)
        self.sel = "L"
        self.duty = {"L": 0, "R": 0}
        self.cmd_mode = False
        self.cmd = ""
        self.quit = False
        self.port = port

    # ------------------------------------------------------------- rendering
    def wheel_panel(self, w):
        d = self.link.d.get(w)
        s = self.link.s
        t = Table.grid(padding=(0, 1))
        t.add_column(justify="right", style="grey58", min_width=9)
        t.add_column(justify="left", min_width=30)

        if not d:
            t.add_row("", Text("waiting for telemetry", style="grey42"))
            return Panel(t, title=f"[bold]{w}[/]", border_style="grey35")

        pos, tgt = d["pos"], d["tgt"]
        deg = pos * 360.0 / CPR
        err = tgt - pos
        errdeg = err * 360.0 / CPR
        mode = {0: "idle", 1: "duty", 2: "position", 3: "velocity"}.get(d["mode"], "?")

        ecol = "green" if abs(err) <= s.get("tol", 8) else ("yellow" if abs(err) < 60 else "red")
        t.add_row("position", Text(f"{pos:+9d} cts   {deg:+9.2f}°   {pos/CPR:+7.3f} turns",
                                   style="bright_white"))
        t.add_row("target", Text(f"{tgt:+9d} cts   {tgt*360.0/CPR:+9.2f}°", style="grey70"))
        t.add_row("error", Text(f"{err:+9d} cts   {errdeg:+9.2f}°", style=ecol))
        rpm = d["vel"] * 60.0 / CPR
        t.add_row("velocity", Text(f"{d['vel']:+9.0f} cts/s {rpm:+8.1f} rpm", style="cyan"))

        dcol = "green" if d["duty"] == 0 else "yellow"
        t.add_row("duty", Text(f"{d['duty']:+4d}  [{bar(d['duty'], -255, 255)}]", style=dcol))
        t.add_row("mode", Text(mode, style="magenta" if mode != "idle" else "grey58"))

        agc = s.get(f"{w}agc", -1)
        st = s.get(f"{w}stat", -1)
        if agc >= 0:
            acol = "green" if 40 <= agc <= 88 else ("yellow" if 24 <= agc <= 112 else "red")
            flags = []
            if st >= 0:
                if not st & 0x20:
                    flags.append("NO MAGNET")
                if st & 0x10:
                    flags.append("ML weak")
                if st & 0x08:
                    flags.append("MH strong")
            t.add_row("magnet", Text(f"AGC {agc:3d}  " + (" ".join(flags) if flags else "ok"),
                                     style=acol))
        if d["err"]:
            t.add_row("i2c err", Text(str(d["err"]), style="red"))

        hist = self.link.moves[w]
        if hist:
            errs = [abs(e) for _, e in hist]
            mae = sum(errs) / len(errs)
            worst = max(errs)
            t.add_row("", "")
            t.add_row("settling", Text(f"n={len(hist)}  mean |err| {mae:.3f}°  worst {worst:.3f}°",
                                       style="green" if mae < 0.5 else "yellow"))
            recent = "  ".join(f"{e:+.2f}" for _, e in list(hist)[-6:])
            t.add_row("last", Text(recent, style="grey58"))

        border = "cyan" if w == self.sel else "grey35"
        title = f"[bold]{'▸ ' if w == self.sel else ''}{w}[/]"
        return Panel(t, title=title, border_style=border)

    def cal_panel(self):
        t = Table.grid(padding=(0, 2))
        t.add_column(style="grey58", min_width=8)
        t.add_column(min_width=46)
        any_data = False
        for w in ("L", "R"):
            c = self.link.cal.get(w, {})
            if not c:
                continue
            any_data = True
            if c.get("running"):
                t.add_row(w, Text("calibrating… (x aborts)", style="yellow"))
                continue
            bits = []
            if "fwd" in c:
                bits.append(f"deadband fwd {c['fwd']}  rev {c['rev']}")
            if "slope" in c:
                bits.append(f"slope {c['slope']:.1f} cts/s per duty")
            t.add_row(w, Text("   ".join(bits), style="bright_white"))
            pts = c.get("pts") or []
            if pts:
                t.add_row("", Text("  ".join(f"{d}→{v:.0f}" for d, v in pts), style="grey58"))
        if not any_data:
            t.add_row("", Text("no calibration yet — press c to run it on the selected wheel",
                               style="grey42"))
        return Panel(t, title="[bold]calibration[/]", border_style="grey35")

    def log_panel(self):
        colors = {"tx": "blue", "rx": "grey42", "err": "red", "ack": "grey58",
                  "cal": "yellow", "done": "green", "sys": "magenta"}
        lines = []
        for kind, msg in list(self.link.log)[-8:]:
            prefix = {"tx": "→", "done": "✓", "err": "!"}.get(kind, " ")
            lines.append(Text(f"{prefix} {msg}", style=colors.get(kind, "grey58")))
        return Panel(Group(*lines) if lines else Text(""), title="[bold]log[/]",
                     border_style="grey35")

    def footer(self):
        if self.cmd_mode:
            return Panel(Text(": " + self.cmd + "▌", style="bright_white"),
                         border_style="cyan",
                         title="[bold]command[/]  (p <deg> · v <cts/s> · d <duty> · k <kp> <ki> <kd> · tol <n>)")
        s = self.link.s
        gains = (f"Kp {s.get('kp', 0):.3f}   Ki {s.get('ki', 0):.3f}   "
                 f"Kd {s.get('kd', 0):.4f}   tol {s.get('tol', 0)} cts") if s else "gains —"
        keys = "   ".join(f"[bold cyan]{k}[/] {v}" for k, v in HELP)
        return Panel(Group(Text.from_markup(f"[grey58]{gains}[/]"), Text.from_markup(keys)),
                     border_style="grey35")

    def render(self):
        lay = Layout()
        lay.split_column(
            Layout(name="head", size=3),
            Layout(name="wheels", size=16),
            Layout(name="cal", size=6),
            Layout(name="log", size=10),
            Layout(name="foot", size=5),
        )
        conn = ("[green]connected[/]" if self.link.connected else "[yellow]connecting…[/]")
        lay["head"].update(Panel(Align.center(Text.from_markup(
            f"[bold]DRIVE HARNESS[/]   {self.port}   {conn}   "
            f"[grey58]{self.link.rate:4.1f} Hz   selected [bold cyan]{self.sel}[/][/]")),
            border_style="grey35"))
        w = Layout()
        w.split_row(Layout(self.wheel_panel("L")), Layout(self.wheel_panel("R")))
        lay["wheels"].update(w)
        lay["cal"].update(self.cal_panel())
        lay["log"].update(self.log_panel())
        lay["foot"].update(self.footer())
        return lay

    # ------------------------------------------------------------- input
    def rotate(self, deg):
        self.link.send(f"P {self.sel} {deg}")

    def nudge_duty(self, step):
        self.duty[self.sel] = max(-255, min(255, self.duty[self.sel] + step))
        self.link.send(f"D {self.sel} {self.duty[self.sel]}")

    def key(self, ch):
        if self.cmd_mode:
            if ch in ("\r", "\n"):
                c = self.cmd.strip()
                self.cmd = ""
                self.cmd_mode = False
                if c:
                    self.run_command(c)
            elif ch == "\x1b":
                self.cmd, self.cmd_mode = "", False
            elif ch in ("\x7f", "\b"):
                self.cmd = self.cmd[:-1]
            elif ch.isprintable():
                self.cmd += ch
            return

        if ch == "q":
            self.quit = True
        elif ch == "\t":
            self.sel = "R" if self.sel == "L" else "L"
        elif ch == " ":
            self.duty = {"L": 0, "R": 0}
            self.link.send("X")
        elif ch == "z":
            self.link.send("Z")
        elif ch == "c":
            self.link.send(f"C {self.sel}")
        elif ch == "x":
            self.link.send("X")
        elif ch == "h":
            self.rotate(-15)
        elif ch == "l":
            self.rotate(15)
        elif ch == "j":
            self.rotate(-90)
        elif ch == "k":
            self.rotate(90)
        elif ch == "J":
            self.rotate(-360)
        elif ch == "K":
            self.rotate(360)
        elif ch == "w":
            self.nudge_duty(10)
        elif ch == "s":
            self.nudge_duty(-10)
        elif ch == ":":
            self.cmd_mode = True
            self.cmd = ""

    def run_command(self, c):
        p = c.split()
        k = p[0].lower()
        try:
            if k == "p" and len(p) >= 2:
                self.link.send(f"P {self.sel} {float(p[1])}")
            elif k == "pb" and len(p) >= 2:
                self.link.send(f"PB {float(p[1])}")
            elif k == "v" and len(p) >= 2:
                self.link.send(f"V {self.sel} {float(p[1])}")
            elif k == "d" and len(p) >= 2:
                self.duty[self.sel] = int(p[1])
                self.link.send(f"D {self.sel} {int(p[1])}")
            elif k == "k" and len(p) >= 4:
                self.link.send(f"K {float(p[1])} {float(p[2])} {float(p[3])}")
            elif k == "tol" and len(p) >= 2:
                self.link.send(f"TOL {int(p[1])}")
            else:
                self.link.send(c.upper())
        except ValueError:
            self.link.log.append(("err", f"bad command: {c}"))

    # ------------------------------------------------------------- loop
    def run(self):
        self.link.start()
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            with Live(self.render(), console=Console(), screen=True,
                      refresh_per_second=12, transient=False) as live:
                while not self.quit:
                    r, _, _ = select.select([sys.stdin], [], [], 0.08)
                    if r:
                        self.key(sys.stdin.read(1))
                    live.update(self.render())
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.link.send("X")
            time.sleep(0.2)
            self.link.stop()
            self.link.join(timeout=1.5)


if __name__ == "__main__":
    App(sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0").run()
