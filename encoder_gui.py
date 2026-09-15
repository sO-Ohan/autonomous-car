#!/usr/bin/env python3
"""
Magnetic encoder checker -- a GUI that answers one question per wheel:
is this AS5600 reporting distance correctly, yes or no?

Reads the `ODO,` telemetry stream the c3-webgui firmware prints over USB
serial at 10 Hz:

    ODO,<leftMetres>,<rightMetres>,<leftOk>,<rightOk>,<leftCounts>,<rightCounts>

A sensor passes only when it does BOTH things: answers on I2C, and actually
changes its count when you turn the wheel. An AS5600 with a missing or badly
spaced magnet still ACKs happily on the bus -- it just returns a value that
never moves -- so "the mux sees it" is not enough to call it good.

    python encoder_gui.py [port]
"""

import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import font as tkfont

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200

# How many counts a wheel must move before we believe it is really moving
# rather than showing magnetic noise on a stationary shaft.
MOVE_COUNTS = 40

BG      = "#15171a"
PANEL   = "#1e2126"
EDGE    = "#2c3138"
TEXT    = "#e7e9ec"
MUTED   = "#8b929c"
GREEN   = "#3ddc84"
RED     = "#ff5c5c"
AMBER   = "#ffb020"
BLUE    = "#5cc8ff"


class SerialReader(threading.Thread):
    """Own the serial port on a background thread, publish frames to a queue."""

    daemon = True

    def __init__(self, port, out_q):
        super().__init__()
        self.port = port
        self.out_q = out_q
        self.ser = None
        self._tx = queue.Queue()
        self._stop = threading.Event()

    def send(self, data: bytes):
        self._tx.put(data)

    def stop(self):
        self._stop.set()

    def run(self):
        buf = b""
        while not self._stop.is_set():
            if self.ser is None:
                try:
                    self.ser = serial.Serial(self.port, BAUD, timeout=0.2)
                    self.out_q.put(("link", True))
                    buf = b""
                except Exception:
                    self.out_q.put(("link", False))
                    time.sleep(0.4)
                    continue
            try:
                while not self._tx.empty():
                    self.ser.write(self._tx.get_nowait())

                chunk = self.ser.read(4096)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        self.out_q.put(("line", line.decode("utf-8", "replace").strip()))
            except Exception:
                # Board reset or cable pulled -- drop the handle and retry.
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                self.out_q.put(("link", False))
                time.sleep(0.4)


class Wheel:
    """Per-wheel verdict state."""

    def __init__(self, name):
        self.name = name
        self.metres = 0.0
        self.counts = 0
        self.online = False
        self.seen_counts = set()
        self.min_c = None
        self.max_c = None

    def update(self, metres, counts, online):
        self.metres, self.counts, self.online = metres, counts, online
        if online:
            self.min_c = counts if self.min_c is None else min(self.min_c, counts)
            self.max_c = counts if self.max_c is None else max(self.max_c, counts)

    @property
    def travel(self):
        if self.min_c is None:
            return 0
        return self.max_c - self.min_c

    def reset_travel(self):
        self.min_c = self.max_c = self.counts

    def verdict(self):
        """(text, colour) -- the yes/no answer for this sensor."""
        if not self.online:
            return "FAIL - no I2C reply", RED
        if self.travel >= MOVE_COUNTS:
            return "OK - counts track movement", GREEN
        return "turn this wheel...", AMBER


class App:
    def __init__(self, root):
        self.root = root
        root.title("Magnetic Encoder Check")
        root.configure(bg=BG)
        root.minsize(660, 430)

        self.q = queue.Queue()
        self.reader = SerialReader(PORT, self.q)
        self.reader.start()

        self.linked = False
        self.last_frame = 0.0
        self.wheels = {"LEFT": Wheel("LEFT"), "RIGHT": Wheel("RIGHT")}

        self.f_title = tkfont.Font(family="sans-serif", size=11, weight="bold")
        self.f_big   = tkfont.Font(family="monospace", size=34, weight="bold")
        self.f_mid   = tkfont.Font(family="monospace", size=13)
        self.f_small = tkfont.Font(family="sans-serif", size=10)

        self._build()
        self._tick()

    # ---------------------------------------------------------------- ui

    def _build(self):
        head = tk.Frame(self.root, bg=BG)
        head.pack(fill="x", padx=18, pady=(16, 6))

        tk.Label(head, text="MAGNETIC ENCODER CHECK", bg=BG, fg=TEXT,
                 font=self.f_title).pack(side="left")
        self.link_lbl = tk.Label(head, text="connecting...", bg=BG, fg=AMBER,
                                 font=self.f_small)
        self.link_lbl.pack(side="right")

        cards = tk.Frame(self.root, bg=BG)
        cards.pack(fill="both", expand=True, padx=18, pady=6)
        cards.columnconfigure(0, weight=1, uniform="c")
        cards.columnconfigure(1, weight=1, uniform="c")

        self.cards = {}
        for i, name in enumerate(("LEFT", "RIGHT")):
            self.cards[name] = self._card(cards, name, i)

        # difference row
        diff = tk.Frame(self.root, bg=PANEL, highlightbackground=EDGE,
                        highlightthickness=1)
        diff.pack(fill="x", padx=18, pady=6)
        tk.Label(diff, text="LEFT - RIGHT", bg=PANEL, fg=MUTED,
                 font=self.f_small).pack(side="left", padx=14, pady=10)
        self.diff_lbl = tk.Label(diff, text="--", bg=PANEL, fg=BLUE,
                                 font=self.f_mid)
        self.diff_lbl.pack(side="right", padx=14)

        btns = tk.Frame(self.root, bg=BG)
        btns.pack(fill="x", padx=18, pady=(6, 16))
        tk.Button(btns, text="ZERO", command=self.zero, bg=EDGE, fg=TEXT,
                  activebackground=BLUE, relief="flat", font=self.f_small,
                  padx=22, pady=9, bd=0, highlightthickness=0).pack(side="left")
        tk.Button(btns, text="RESET TEST", command=self.reset_test, bg=EDGE,
                  fg=TEXT, activebackground=BLUE, relief="flat",
                  font=self.f_small, padx=22, pady=9, bd=0,
                  highlightthickness=0).pack(side="left", padx=8)
        tk.Label(btns, text=f"turn each wheel by hand — {MOVE_COUNTS}+ counts to pass",
                 bg=BG, fg=MUTED, font=self.f_small).pack(side="right")

    def _card(self, parent, name, col):
        f = tk.Frame(parent, bg=PANEL, highlightbackground=EDGE,
                     highlightthickness=1)
        f.grid(row=0, column=col, sticky="nsew", padx=(0, 10) if col == 0 else (10, 0))

        tk.Label(f, text=name, bg=PANEL, fg=MUTED,
                 font=self.f_title).pack(anchor="w", padx=16, pady=(14, 0))

        val = tk.Label(f, text="--", bg=PANEL, fg=TEXT, font=self.f_big)
        val.pack(anchor="w", padx=16, pady=(2, 0))

        cnt = tk.Label(f, text="counts --   moved --", bg=PANEL, fg=MUTED,
                       font=self.f_small)
        cnt.pack(anchor="w", padx=16, pady=(2, 8))

        verd = tk.Label(f, text="waiting", bg=PANEL, fg=AMBER,
                        font=self.f_mid, wraplength=250, justify="left")
        verd.pack(anchor="w", padx=16, pady=(0, 16))

        return {"value": val, "counts": cnt, "verdict": verd}

    # ---------------------------------------------------------------- actions

    def zero(self):
        self.reader.send(b"Z\n")
        for w in self.wheels.values():
            w.min_c = w.max_c = 0

    def reset_test(self):
        for w in self.wheels.values():
            w.reset_travel()

    # ---------------------------------------------------------------- loop

    def _drain(self):
        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                return
            if kind == "link":
                self.linked = payload
            elif kind == "line" and payload.startswith("ODO,"):
                self._parse(payload)

    def _parse(self, line):
        parts = line.split(",")
        if len(parts) != 7:
            return
        try:
            lm, rm = float(parts[1]), float(parts[2])
            lok, rok = parts[3] == "1", parts[4] == "1"
            lc, rc = int(parts[5]), int(parts[6])
        except ValueError:
            return
        self.wheels["LEFT"].update(lm, lc, lok)
        self.wheels["RIGHT"].update(rm, rc, rok)
        self.last_frame = time.time()

    def _tick(self):
        self._drain()

        stale = (time.time() - self.last_frame) > 1.5
        if not self.linked:
            self.link_lbl.config(text=f"{PORT}: not connected", fg=RED)
        elif stale:
            self.link_lbl.config(text=f"{PORT}: no telemetry", fg=AMBER)
        else:
            self.link_lbl.config(text=f"{PORT}: streaming", fg=GREEN)

        for name, w in self.wheels.items():
            c = self.cards[name]
            live = self.linked and not stale
            if not live:
                c["value"].config(text="--", fg=MUTED)
                c["counts"].config(text="counts --   moved --")
                c["verdict"].config(text="no data", fg=MUTED)
                continue

            c["value"].config(text=f"{w.metres:+.3f} m",
                              fg=TEXT if w.online else RED)
            c["counts"].config(text=f"counts {w.counts}   moved {w.travel}")
            text, colour = w.verdict()
            c["verdict"].config(text=text, fg=colour)

        L, R = self.wheels["LEFT"], self.wheels["RIGHT"]
        if self.linked and not stale and L.online and R.online:
            self.diff_lbl.config(text=f"{L.metres - R.metres:+.3f} m")
        else:
            self.diff_lbl.config(text="--")

        self.root.after(80, self._tick)


def main():
    root = tk.Tk()
    app = App(root)
    try:
        root.mainloop()
    finally:
        app.reader.stop()


if __name__ == "__main__":
    main()
