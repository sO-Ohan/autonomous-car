#!/usr/bin/env python3
"""
Wheel calibration TUI.

The AS5600 encoders sit on the motor shafts, before the gearbox, so the board
counts motor turns and cannot know how far the wheel went. You turn the wheel
by hand to a known angle and press ENTER; this works out the gear ratio.

Nothing is ever powered. Every reading is saved the instant you take it, so
there is nothing to lose if you close the window.

    python wheel_cal.py [--port /dev/ttyACM0]
"""

import argparse
import csv
import math
import os
import queue
import select
import sys
import termios
import threading
import time
import tty

import serial
from rich.align import Align
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

COUNTS_PER_MOTOR_REV = 4096
WHEEL_DIAMETER_M = 0.120                  # 120 mm
TARGETS = (90, 180, 270, 360, 720, 1080)
GOAL_READINGS = 5                         # per wheel, for the progress bar

CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "wheel_cal.csv")

WHEELS = ("LEFT", "RIGHT")


# --------------------------------------------------------------- serial

class Reader(threading.Thread):
    """Keep `counts` fresh from the board's ODO stream."""

    daemon = True

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.counts = [0, 0]
        self.linked = False
        self.err = ""
        self._tx = queue.Queue()
        self._marks = queue.Queue()
        self._stop = threading.Event()

    def send(self, data):
        self._tx.put(data)

    def mark(self, timeout=1.0):
        """Ask for the counters right now; fall back to the stream."""
        while not self._marks.empty():
            self._marks.get_nowait()
        self.send(b"M\n")
        try:
            return self._marks.get(timeout=timeout)
        except queue.Empty:
            return tuple(self.counts)

    def stop(self):
        self._stop.set()

    def run(self):
        ser, buf = None, b""
        while not self._stop.is_set():
            if ser is None:
                try:
                    ser = serial.Serial(self.port, 115200, timeout=0.15)
                    ser.write(b"X\n")            # never leave a motor driving
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
                        elif line.startswith("# MARK,"):
                            p = line[7:].split(",")
                            try:
                                self._marks.put((int(p[0]), int(p[1])))
                            except (ValueError, IndexError):
                                pass
            except Exception as e:
                self.linked, self.err = False, str(e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(0.4)


# --------------------------------------------------------------- keyboard

class RawKeys:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


# --------------------------------------------------------------- maths

def fit_counts_per_rev(rows):
    """
    Least-squares fit of counts against angle, forced through the origin.

    Zero counts at zero degrees is exact, so the line must pass through it.
    Weighting by angle is what we want: misjudging 90 degrees by 5 is a 5.6%
    error, misjudging 720 by the same 5 is 0.7%, so long turns count most.
    """
    num = sum(a * c for a, c in rows if c > 0)
    den = sum(a * a for a, c in rows if c > 0)
    return 360.0 * num / den if den else None


def bar(frac, width=28, done="█", todo="░"):
    frac = max(0.0, min(1.0, frac))
    n = int(round(frac * width))
    return done * n + todo * (width - n)


# --------------------------------------------------------------- app

class App:
    def __init__(self, port):
        self.reader = Reader(port)
        self.reader.start()

        self.wheel = 0                      # index into WHEELS
        self.rows = {w: [] for w in WHEELS}
        self.base = {w: None for w in WHEELS}
        self.target_i = {w: 0 for w in WHEELS}
        self.flash = ""
        self.flash_until = 0.0
        self.done = False
        self.saved = 0
        # (byte offset, wheel, angle, counts) for each row appended this
        # session, newest last. Lets undo truncate the row back off the file
        # instead of leaving a bad reading in the saved data.
        self._appended = []

        self._recent = (0.0, 0)             # (time, counts) for motion detect
        self.console = Console()

    # ------------------------------------------------------------ helpers

    @property
    def w(self):
        return WHEELS[self.wheel]

    def counts_now(self):
        return self.reader.counts[self.wheel]

    def target(self):
        return TARGETS[min(self.target_i[self.w], len(TARGETS) - 1)]

    def turned(self):
        b = self.base[self.w]
        return 0 if b is None else abs(self.counts_now() - b)

    def best_n(self):
        """Best gear-ratio estimate so far: this wheel, else both, else none."""
        for rows in (self.rows[self.w],
                     self.rows[WHEELS[0]] + self.rows[WHEELS[1]]):
            per_rev = fit_counts_per_rev(rows)
            if per_rev:
                return per_rev / COUNTS_PER_MOTOR_REV
        return None

    def est_degrees(self):
        n = self.best_n()
        if not n:
            return None
        return self.turned() / (COUNTS_PER_MOTOR_REV * n) * 360.0

    def moving(self):
        now, c = time.time(), self.counts_now()
        t0, c0 = self._recent
        if now - t0 > 0.4:
            self._recent = (now, c)
            return abs(c - c0) > 20
        return abs(c - c0) > 20

    def say(self, msg, secs=2.5):
        self.flash, self.flash_until = msg, time.time() + secs

    # ------------------------------------------------------------ actions

    def set_base(self):
        self.base[self.w] = self.reader.mark()[self.wheel]
        self.target_i[self.w] = 0
        self.say(f"{self.w}: start point set — now turn the wheel")

    def record(self):
        if self.base[self.w] is None:
            self.set_base()
            return
        counts = self.turned()
        if counts < 100:
            self.say("barely moved — turn the wheel first", 3)
            return
        angle = self.target()
        self.rows[self.w].append((angle, counts))
        self.append_csv(self.w, angle, counts)
        self.target_i[self.w] += 1
        self.say(f"recorded {angle:g}° = {counts:,} counts")

    def undo(self):
        if not self.rows[self.w]:
            self.say("nothing to undo")
            return
        angle, counts = self.rows[self.w].pop()
        self.target_i[self.w] = max(0, self.target_i[self.w] - 1)
        if self.unappend_csv(self.w, angle, counts):
            self.say(f"removed {angle:g}° reading from the CSV too", 3)
        else:
            self.say(f"removed {angle:g}° from the list, but it is not the "
                     f"last CSV row — edit the file by hand", 4)

    def switch(self, idx):
        self.wheel = idx
        self.say(f"switched to {self.w} wheel")

    def append_csv(self, wheel, angle, counts):
        new = not os.path.exists(CSV_PATH)
        with open(CSV_PATH, "a", newline="") as f:
            if new:
                csv.writer(f).writerow(
                    ["timestamp", "wheel", "angle_deg", "counts"])
            offset = f.tell()
            csv.writer(f).writerow([time.strftime("%Y-%m-%d %H:%M:%S"),
                                    wheel, angle, counts])
        self._appended.append((offset, wheel, angle, counts))
        self.saved += 1

    def unappend_csv(self, wheel, angle, counts):
        """
        Truncate the last appended row back off the file.

        Only safe when the row being undone really is the final one -- if the
        user recorded on the other wheel afterwards, cutting the tail would
        take that reading with it, so refuse and say so.
        """
        if not self._appended:
            return False
        offset, w, a, c = self._appended[-1]
        if (w, a, c) != (wheel, angle, counts):
            return False
        try:
            with open(CSV_PATH, "r+", newline="") as f:
                f.truncate(offset)
        except OSError:
            return False
        self._appended.pop()
        self.saved = max(0, self.saved - 1)
        return True

    # ------------------------------------------------------------ render

    def header(self):
        row = Table.grid(padding=(0, 1))
        for _ in range(5):
            row.add_column()
        cells = []
        for i, name in enumerate(WHEELS):
            sel = i == self.wheel
            cells.append(Text(f" {'>' if sel else ' '} {name} ",
                              style="bold black on green" if sel else "grey42"))
            cells.append(Text(f"{len(self.rows[name])}",
                              style="white" if sel else "grey35"))
        cells.append(Text("  ● board" if self.reader.linked else "  ● NO BOARD",
                          style="green" if self.reader.linked else "bold red"))
        row.add_row(*cells)
        return Panel(row, title="[bold]WHEEL[/] — press L / R",
                     border_style="grey35", padding=(0, 1))

    def instruction(self):
        if self.base[self.w] is None:
            body = Text.assemble(
                ("Line the tyre mark up with your reference point, then press ",
                 "white"), ("ENTER", "bold yellow"), (" to zero it.", "white"))
            return Panel(body, title=f"[bold yellow]SET START — {self.w}[/]",
                         border_style="yellow", padding=(0, 1))

        exhausted = self.target_i[self.w] >= len(TARGETS)
        tgt = self.target()
        est = self.est_degrees()
        lines = []

        if exhausted:
            other = WHEELS[1 - self.wheel]
            lines.append(Text.assemble(
                (f"{self.w} has all the angles. ", "bold green"),
                ("Press ", "white"),
                ("R" if self.wheel == 0 else "L", "bold yellow"),
                (f" for the {other} wheel, or ", "white"),
                ("F", "bold yellow"), (" to finish.", "white")))
            lines.append(Text(f"(ENTER still records another {tgt:g}° reading)",
                              style="grey42"))
        else:
            lines.append(Text.assemble(
                ("Turn to ", "white"), (f"{tgt:g}°", "bold yellow"),
                (" total from the start", "white"),
                ("   — one direction only", "grey42")))

        if est is not None:
            lines.append(Text.assemble(
                (bar(est / tgt, 30), "cyan"),
                (f" ~{est:5.0f}° of {tgt:g}°", "bold white"),
                ("   ● TURNING" if self.moving() else "   ○ still",
                 "bold green" if self.moving() else "grey42")))
        else:
            lines.append(Text.assemble(
                (f"{self.turned():,} counts so far", "white"),
                ("   ● TURNING" if self.moving() else "   ○ still",
                 "bold green" if self.moving() else "grey42")))

        if not exhausted:
            lines.append(Text.assemble(
                ("Press ", "white"), ("ENTER", "bold yellow"),
                (f" when the mark reaches {tgt:g}°", "white")))

        return Panel(Group(*lines),
                     title=f"[bold yellow]TURN THE {self.w} WHEEL[/]",
                     border_style="yellow", padding=(0, 1))

    def progress(self, max_rows=3):
        got = len(self.rows[self.w])
        n = self.best_n()
        head = Text.assemble(
            (bar(got / GOAL_READINGS, 18), "green"),
            (f"  {got}/{GOAL_READINGS}", "white"),
            (f"    gear ratio so far  {n:.2f} : 1" if n else "", "bold green"))
        parts = [head]
        if self.rows[self.w] and max_rows > 0:
            tb = Table(box=None, pad_edge=False, padding=(0, 2), show_header=False)
            tb.add_column(justify="right", style="grey58")
            tb.add_column(justify="right", style="white")
            tb.add_column(justify="right", style="cyan")
            for a, c in self.rows[self.w][-max_rows:]:
                tb.add_row(f"{a:g}°", f"{c:,}",
                           f"N={c * 360.0 / a / COUNTS_PER_MOTOR_REV:.2f}")
            parts.append(tb)
        return Panel(Group(*parts), title="[bold]PROGRESS[/]",
                     border_style="grey35", padding=(0, 1))

    def footer(self):
        keys = Text.assemble(
            (" ENTER", "bold yellow"), (" record  ", "grey58"),
            ("L/R", "bold yellow"), (" wheel  ", "grey58"),
            ("N", "bold yellow"), (" restart  ", "grey58"),
            ("U", "bold yellow"), (" undo  ", "grey58"),
            ("F", "bold yellow"), (" finish  ", "grey58"),
            ("Q", "bold yellow"), (" quit", "grey58"))
        msg = (Text(" " + self.flash, style="bold black on yellow")
               if time.time() < self.flash_until else Text(""))
        return Group(keys, msg)

    def screen(self):
        if self.done:
            return self.done_screen()
        # Trim the readings table so the footer -- which carries the keys and
        # the confirmation message -- can never fall off the bottom.
        height = self.console.size.height
        max_rows = 3 if height >= 22 else (1 if height >= 18 else 0)
        return Group(self.header(), self.instruction(),
                     self.progress(max_rows), self.footer())

    def done_screen(self):
        circumference = math.pi * WHEEL_DIAMETER_M
        t = Table(box=None, padding=(0, 3))
        t.add_column("", style="grey58")
        t.add_column("counts / wheel rev", justify="right")
        t.add_column("gear ratio N", justify="right", style="bold cyan")
        t.add_column("counts / metre", justify="right")

        for name in WHEELS:
            per_rev = fit_counts_per_rev(self.rows[name])
            if per_rev:
                t.add_row(name, f"{per_rev:,.0f}",
                          f"{per_rev / COUNTS_PER_MOTOR_REV:.2f} : 1",
                          f"{per_rev / circumference:,.0f}")

        every = self.rows[WHEELS[0]] + self.rows[WHEELS[1]]
        per_rev = fit_counts_per_rev(every)

        parts = [Text("ALL DATA SAVED", style="bold black on green"),
                 Text(""),
                 Text(f"{self.saved} readings written to", style="white"),
                 Text(f"{CSV_PATH}", style="cyan"),
                 Text("")]
        if per_rev:
            cpm = per_rev / circumference
            parts += [
                t, Text(""),
                Text("COMBINED", style="grey58"),
                Text(f"  gear ratio N     :  "
                     f"{per_rev / COUNTS_PER_MOTOR_REV:.2f} : 1",
                     style="bold green"),
                Text(f"  COUNTS_PER_METRE :  {cpm:,.1f}", style="bold green"),
                Text(""),
                Text("paste into c3-webgui/src/main.cpp:", style="grey58"),
                Text(f"  static const float COUNTS_PER_METRE = {cpm:.1f}f;",
                     style="bold white")]
        else:
            parts.append(Text("no readings taken", style="yellow"))

        parts += [Text(""),
                  Text("You can close this window now.  (Q to quit)",
                       style="bold black on green")]
        return Panel(Align.center(Group(*parts)), border_style="green",
                     title="[bold green]DONE[/]")

    # ------------------------------------------------------------ loop

    def run(self):
        with RawKeys() as keys, Live(self.screen(), console=self.console,
                                     refresh_per_second=10,
                                     screen=True) as live:
            while True:
                k = keys.get()
                if k:
                    if k in ("q", "Q"):
                        break
                    elif self.done:
                        pass
                    elif k in ("\r", "\n"):
                        self.record()
                    elif k in ("l", "L"):
                        self.switch(0)
                    elif k in ("r", "R"):
                        self.switch(1)
                    elif k in ("n", "N"):
                        self.set_base()
                    elif k in ("u", "U"):
                        self.undo()
                    elif k in ("f", "F"):
                        self.done = True
                live.update(self.screen())
                time.sleep(0.05)
        self.reader.send(b"X\n")
        time.sleep(0.2)
        self.reader.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    args = ap.parse_args()
    App(args.port).run()
    print("data saved to", CSV_PATH)


if __name__ == "__main__":
    main()
