#!/usr/bin/env python3
"""
Rotate TUI — turn a wheel by a commanded angle, at a chosen speed.

Uses the gear ratio measured by wheel_cal.py (calibration.json) to convert
wheel degrees into encoder counts, and the board closes the loop: it drives,
tapers off near the target, and reports what it actually achieved.

    python rotate.py [--port /dev/ttyACM0]
"""

import argparse
import json
import os
import queue
import select
import sys
import termios
import threading
import time
import tty

import serial
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

HERE = os.path.dirname(os.path.abspath(__file__))
CAL_PATH = os.path.join(HERE, "calibration.json")

WHEELS = ("LEFT", "RIGHT", "BOTH")
SIDE_CODE = {"LEFT": "L", "RIGHT": "R"}

MIN_DUTY, MAX_DUTY = 58, 255      # 58 is just above the measured deadband


def load_cal():
    try:
        cal = json.load(open(CAL_PATH))
    except Exception:
        return None
    out = {}
    for name in ("LEFT", "RIGHT"):
        w = cal.get("wheels", {}).get(name) or cal.get("combined")
        if w:
            out[name] = w["counts_per_degree"]
    return out or None


class Reader(threading.Thread):
    daemon = True

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.counts = [0, 0]
        self.linked = False
        self.err = ""
        self.events = queue.Queue()
        self._tx = queue.Queue()
        self._stop = threading.Event()

    def send(self, data):
        self._tx.put(data)

    def stop(self):
        self._stop.set()

    def run(self):
        ser, buf = None, b""
        while not self._stop.is_set():
            if ser is None:
                try:
                    ser = serial.Serial(self.port, 115200, timeout=0.15)
                    ser.write(b"X\n")
                    self.linked, self.err, buf = True, "", b""
                except Exception as e:
                    self.linked, self.err = False, str(e)
                    time.sleep(0.5)
                    continue
            try:
                while not self._tx.empty():
                    ser.write(self._tx.get_nowait())
                chunk = ser.read(4096)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("utf-8", "replace").strip()
                        if line.startswith("ODO,"):
                            p = line.split(",")
                            if len(p) == 7:
                                try:
                                    self.counts = [int(p[5]), int(p[6])]
                                except ValueError:
                                    pass
                        elif line.startswith("# GODONE,"):
                            self.events.put(("done", line[9:].split(",")))
                        elif line.startswith("# GO,"):
                            self.events.put(("start", line[5:].split(",")))
            except Exception as e:
                self.linked, self.err = False, str(e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(0.4)


class RawKeys:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self):
        if not select.select([sys.stdin], [], [], 0)[0]:
            return None
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        # Arrow keys arrive as a short escape sequence; pull the rest of it.
        seq = ch
        while select.select([sys.stdin], [], [], 0.02)[0] and len(seq) < 6:
            seq += sys.stdin.read(1)
        return {"\x1b[A": "UP", "\x1b[B": "DOWN",
                "\x1b[C": "RIGHT", "\x1b[D": "LEFT"}.get(seq, "ESC")


def bar(frac, width=30):
    frac = max(0.0, min(1.0, frac))
    n = int(round(frac * width))
    return "█" * n + "░" * (width - n)


class App:
    def __init__(self, port):
        self.cal = load_cal()
        self.reader = Reader(port)
        self.reader.start()
        self.console = Console()

        self.wheel = 0
        self.angle = 90.0
        self.duty = 90
        self.typing = ""
        self.flash = ""
        self.flash_until = 0.0

        self.pending = []             # queued sides for a BOTH move
        self.moving_side = None
        self.move_start = 0
        self.move_target = 0.0
        self.last = None              # (side, commanded, actual, err, why)

    # ---------------------------------------------------------------- misc

    @property
    def w(self):
        return WHEELS[self.wheel]

    def cpd(self, side):
        if not self.cal:
            return None
        return self.cal.get(side)

    def say(self, msg, secs=3.0):
        self.flash, self.flash_until = msg, time.time() + secs

    def shown_angle(self):
        if self.typing:
            try:
                return float(self.typing)
            except ValueError:
                return self.angle
        return self.angle

    # ------------------------------------------------------------- actions

    def go(self):
        if not self.cal:
            self.say("no calibration.json — run wheel_cal.py first", 5)
            return
        if self.moving_side or self.pending:
            self.say("already moving — SPACE to stop")
            return
        angle = self.shown_angle()
        self.typing = ""
        self.angle = angle
        if abs(angle) < 0.1:
            self.say("angle is zero")
            return
        self.pending = (["LEFT", "RIGHT"] if self.w == "BOTH" else [self.w])
        self.next_move()

    def next_move(self):
        if not self.pending:
            return
        side = self.pending.pop(0)
        idx = 0 if side == "LEFT" else 1
        self.moving_side = side
        self.move_start = self.reader.counts[idx]
        self.move_target = abs(self.angle) * self.cpd(side)
        self.reader.send(
            f"G {SIDE_CODE[side]} {self.angle:.2f} {self.duty}\n".encode())

    def stop(self):
        self.pending = []
        self.reader.send(b"X\n")
        self.say("stopped")

    def drain_events(self):
        while True:
            try:
                kind, parts = self.reader.events.get_nowait()
            except queue.Empty:
                return
            if kind == "done":
                try:
                    self.last = (parts[0], float(parts[1]), float(parts[2]),
                                 float(parts[3]), parts[4])
                except (ValueError, IndexError):
                    self.last = None
                self.moving_side = None
                if self.pending:
                    self.next_move()

    def move_progress(self):
        if not self.moving_side:
            return None
        idx = 0 if self.moving_side == "LEFT" else 1
        moved = abs(self.reader.counts[idx] - self.move_start)
        if self.move_target <= 0:
            return None
        cpd = self.cpd(self.moving_side)
        return moved / self.move_target, moved / cpd

    # -------------------------------------------------------------- render

    def header(self):
        row = Table.grid(padding=(0, 1))
        for _ in range(len(WHEELS) + 1):
            row.add_column()
        cells = []
        for i, name in enumerate(WHEELS):
            sel = i == self.wheel
            cells.append(Text(f" {name} ",
                              style="bold black on green" if sel else "grey42"))
        cells.append(Text("  ● board" if self.reader.linked else "  ● NO BOARD",
                          style="green" if self.reader.linked else "bold red"))
        row.add_row(*cells)
        return Panel(row, title="[bold]WHEEL[/] — L / R / B",
                     border_style="grey35", padding=(0, 1))

    def command(self):
        ang = self.shown_angle()
        angle_txt = Text.assemble(
            ("  ANGLE  ", "grey58"),
            (f"{ang:+8.1f}°", "bold yellow" if not self.typing
             else "bold black on yellow"),
            ("    ← →  ±10    type digits, - flips sign", "grey42"))
        speed_txt = Text.assemble(
            ("  SPEED  ", "grey58"), (f"{self.duty:8d}", "bold cyan"),
            (f"    ↑ ↓  ±5     ({MIN_DUTY}–{MAX_DUTY})", "grey42"))
        rows = [angle_txt, speed_txt]
        if self.cal:
            cpd = self.cpd("LEFT") or 0
            rows.append(Text(f"  = {abs(ang) * cpd:,.0f} encoder counts",
                             style="grey42"))
        else:
            rows.append(Text("  no calibration.json — run wheel_cal.py",
                             style="bold red"))
        return Panel(Group(*rows), title="[bold]COMMAND[/]",
                     border_style="grey35", padding=(0, 1))

    def status(self):
        rows = []
        prog = self.move_progress()
        if prog:
            frac, deg = prog
            rows.append(Text.assemble(
                (bar(frac), "cyan"), (f"  {deg:6.1f}° of {abs(self.angle):.0f}°",
                                      "bold white")))
            rows.append(Text(f"  moving {self.moving_side}"
                             + (f"   then {', '.join(self.pending)}"
                                if self.pending else ""),
                             style="bold green"))
        else:
            rows.append(Text("  idle", style="grey42"))

        if self.last:
            side, cmd, act, err, why = self.last
            colour = "green" if abs(err) <= 1.0 else "yellow"
            rows.append(Text(""))
            rows.append(Text.assemble(
                ("  last:  ", "grey58"), (f"{side} ", "white"),
                (f"{cmd:+.1f}° commanded  ", "grey58"),
                (f"{act:+.2f}° actual  ", "bold white"),
                (f"error {err:+.2f}°", colour),
                (f"  [{why}]", "grey42")))
        return Panel(Group(*rows), title="[bold]MOVE[/]",
                     border_style="grey35", padding=(0, 1))

    def footer(self):
        keys = Text.assemble(
            (" ENTER", "bold yellow"), (" go  ", "grey58"),
            ("SPACE", "bold yellow"), (" stop  ", "grey58"),
            ("L/R/B", "bold yellow"), (" wheel  ", "grey58"),
            ("←→", "bold yellow"), (" angle  ", "grey58"),
            ("↑↓", "bold yellow"), (" speed  ", "grey58"),
            ("C", "bold yellow"), (" clear  ", "grey58"),
            ("Q", "bold yellow"), (" quit", "grey58"))
        msg = (Text(" " + self.flash, style="bold black on yellow")
               if time.time() < self.flash_until else Text(""))
        return Group(keys, msg)

    def screen(self):
        return Group(self.header(), self.command(),
                     self.status(), self.footer())

    # ---------------------------------------------------------------- loop

    def key(self, k):
        if k in ("q", "Q"):
            return False
        elif k in ("\r", "\n"):
            self.go()
        elif k == " ":
            self.stop()
        elif k in ("l", "L"):
            self.wheel = 0
        elif k in ("r", "R"):
            self.wheel = 1
        elif k in ("b", "B"):
            self.wheel = 2
        elif k == "RIGHT":
            self.typing = ""
            self.angle += 10
        elif k == "LEFT":
            self.typing = ""
            self.angle -= 10
        elif k == "UP":
            self.duty = min(MAX_DUTY, self.duty + 5)
        elif k == "DOWN":
            self.duty = max(MIN_DUTY, self.duty - 5)
        elif k in ("c", "C"):
            self.typing = ""
            self.angle = 0.0
        elif k == "-":
            self.typing = ""
            self.angle = -self.angle
        elif k.isdigit() or k == ".":
            self.typing += k
        elif k in ("\x7f", "\b"):
            self.typing = self.typing[:-1]
        return True

    def run(self):
        with RawKeys() as keys, Live(self.screen(), console=self.console,
                                     refresh_per_second=12,
                                     screen=True) as live:
            while True:
                self.drain_events()
                k = keys.get()
                if k is not None and not self.key(k):
                    break
                live.update(self.screen())
                time.sleep(0.04)
        self.reader.send(b"X\n")
        time.sleep(0.3)
        self.reader.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    App(ap.parse_args().port).run()


if __name__ == "__main__":
    main()
