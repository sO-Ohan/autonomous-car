#!/usr/bin/env python3
"""
Motor TUI -- drive both motors, watch position hold, edit every property.

This is the front end for `carbot.motor.MotorNode`: open-loop driving,
closed-loop rotates, station keeping, faults and the e-stop, plus a live
editor for the critical and optional properties of each motor.

    python motor_tui.py [--port /dev/ttyACM0] [--no-hold] [--track-width 0.30]

Driving is hold-to-move: a key held down keeps the wheels turning because the
terminal repeats it, and letting go stops the base within 350 ms. There is no
latched drive state to forget about.
"""

import argparse
import os
import select
import sys
import termios
import time
import tty

from rich.align import Align
from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from carbot import (CRITICAL, LEFT, OPTIONAL, MotorConfig, MotorNode,
                    PROPERTIES, RIGHT, WheelDriver)

BOTH = 2
FOCUS_NAME = {LEFT: "LEFT", RIGHT: "RIGHT", BOTH: "BOTH"}
SIDES_OF = {LEFT: (LEFT,), RIGHT: (RIGHT,), BOTH: (LEFT, RIGHT)}

# How long a drive command stays valid without another keypress.
#
# This has to clear the terminal's key-repeat DELAY, not its repeat interval:
# hold a key and the first character arrives at once, the second only after the
# delay -- 500 ms on a stock desktop (`xset q`, "auto repeat delay"). Anything
# shorter than that and a held key drives, stops, then stutters back in, which
# reads as the controls not working at all. After the delay the repeats come
# every ~30 ms, so this only ever has to bridge the first gap. Override with
# --key-hold if your keyboard delay is longer.
KEY_HOLD_S = 0.7

MODE_STYLE = {
    "holding": "bold green",
    "idle": "grey62",
    "driving": "bold cyan",
    "moving": "bold cyan",
    "recovering": "bold yellow",
    "disturbed": "bold yellow",
    "fault": "bold white on red",
    "disabled": "grey42",
}

EVENT_STYLE = {
    "disturbed": "yellow", "recovering": "yellow", "recovered": "green",
    "hold_armed": "cyan", "hold_released": "grey62", "hold_failed": "red",
    "stall": "red", "soft_limit": "red", "encoder_lost": "red",
    "wrong_direction": "bold white on red", "move_split": "cyan",
    "encoder_back": "green", "estop": "bold white on red",
    "estop_cleared": "green", "fault_cleared": "green", "move_done": "grey62",
}


class RawKeys:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read(self) -> str:
        if select.select([sys.stdin], [], [], 0)[0]:
            return os.read(self.fd, 32).decode("utf-8", "replace")
        return ""


def bar(value: float, span: float, width: int = 21) -> Text:
    """A centre-zero bar. `span` is what the full half-width represents."""
    half = width // 2
    span = max(span, 1e-6)
    n = int(round(max(-1.0, min(1.0, value / span)) * half))
    cells = ["─"] * width
    cells[half] = "┼"
    if n:
        step = 1 if n > 0 else -1
        for i in range(half + step, half + n + step, step):
            if 0 <= i < width:
                cells[i] = "█"
    return Text("".join(cells))


class MotorTUI:
    def __init__(self, node: MotorNode, port: str):
        self.node = node
        self.port = port
        self.focus = BOTH
        self.duty = 90
        self.step_deg = 90.0
        self.key_hold_s = KEY_HOLD_S
        self.mode = "drive"          # or "props"
        self.prop_index = 0
        self.show_help = False
        self.message = ""
        self.message_until = 0.0
        self.running = True

    # ------------------------------------------------------------- helpers

    def sides(self):
        return SIDES_OF[self.focus]

    def side_arg(self):
        """The `side` argument MotorNode wants: None means both."""
        return None if self.focus == BOTH else self.focus

    def say(self, text: str, seconds: float = 3.0) -> None:
        self.message = text
        self.message_until = time.time() + seconds

    # ---------------------------------------------------------------- keys

    def handle(self, keys: str) -> None:
        i = 0
        while i < len(keys):
            k = keys[i]
            if k == "\x1b" and keys[i:i + 3].startswith("\x1b["):
                self.key(keys[i:i + 3])
                i += 3
                continue
            self.key(k)
            i += 1

    def key(self, k: str) -> None:
        n = self.node
        if k in ("\x03", "\x04"):
            self.running = False
            return
        if self.mode == "props" and self.prop_key(k):
            return

        # ---- movement
        if k in ("w", "W", "\x1b[A"):
            self.drive(+1, +1)
        elif k in ("s", "S", "\x1b[B"):
            self.drive(-1, -1)
        elif k in ("a", "A", "\x1b[D"):
            self.drive(-1, +1)
        elif k in ("d", "D", "\x1b[C"):
            self.drive(+1, -1)
        elif k == " ":
            n.halt()
            self.say("stopped")
        elif k == "E":
            if n.estopped:
                n.clear_estop()
                self.say("e-stop cleared")
            else:
                n.estop("operator")
                self.say("E-STOP")

        # ---- duty and step
        elif k in ("]", "="):
            self.duty = min(255, self.duty + 5)
        elif k in ("[", "-"):
            self.duty = max(1, self.duty - 5)
        elif k == ">" or k == ".":
            self.step_deg = min(3600.0, self.step_deg * 2)
        elif k == "<" or k == ",":
            self.step_deg = max(1.0, self.step_deg / 2)

        # ---- closed-loop rotate
        elif k == "j":
            self.rotate(-self.step_deg)
        elif k == "l":
            self.rotate(+self.step_deg)

        # ---- hold
        elif k == "h":
            on = not all(n.motors[s].cfg.hold_position for s in self.sides())
            n.set_hold(self.side_arg(), on)
            self.say(f"hold {'on' if on else 'off'} for {FOCUS_NAME[self.focus]}")
        elif k == "m":
            n.hold_here(self.side_arg())
            self.say(f"hold target latched for {FOCUS_NAME[self.focus]}")
        elif k == "r":
            n.release(self.side_arg())
            self.say(f"hold released for {FOCUS_NAME[self.focus]}")

        # ---- state
        elif k == "n":
            on = not all(n.motors[s].cfg.enabled for s in self.sides())
            n.set_property(self.side_arg(), "enabled", on)
            self.say(f"{FOCUS_NAME[self.focus]} {'enabled' if on else 'disabled'}")
        elif k == "c":
            n.clear_faults()
            n.clear_estop()
            self.say("faults cleared")
        elif k == "z":
            n.zero()
            self.say("encoders zeroed, hold re-latched at 0")

        # ---- ui
        elif k == "\t":
            self.focus = {LEFT: RIGHT, RIGHT: BOTH, BOTH: LEFT}[self.focus]
        elif k == "1":
            self.focus = LEFT
        elif k == "2":
            self.focus = RIGHT
        elif k == "3":
            self.focus = BOTH
        elif k == "p":
            self.mode = "props" if self.mode == "drive" else "drive"
        elif k == "?":
            self.show_help = not self.show_help
        elif k in ("x", "q"):
            self.running = False

    def prop_key(self, k: str) -> bool:
        """Property-editor keys. Returns True if the key was consumed."""
        n = self.node
        p = PROPERTIES[self.prop_index]
        if k in ("\x1b[A", "k"):
            self.prop_index = (self.prop_index - 1) % len(PROPERTIES)
        elif k in ("\x1b[B", "j"):
            self.prop_index = (self.prop_index + 1) % len(PROPERTIES)
        elif k in ("\x1b[C", "+", "="):
            self.say(f"{p.name} = {n.bump_property(self.side_arg(), p.name, +1)}")
        elif k in ("\x1b[D", "-"):
            self.say(f"{p.name} = {n.bump_property(self.side_arg(), p.name, -1)}")
        elif k in ("\r", "\n"):
            if p.type is bool:
                self.say(f"{p.name} = {n.bump_property(self.side_arg(), p.name, +1)}")
            else:
                self.say(f"{p.name} is not a switch; use left/right")
        elif k == "0":
            n.set_property(self.side_arg(), p.name, p.default)
            self.say(f"{p.name} reset to {p.default}")
        elif k == "\x1b":
            self.mode = "drive"
        else:
            return False
        return True

    # ------------------------------------------------------------ actions

    def drive(self, left_sign: int, right_sign: int) -> None:
        if self.focus == LEFT:
            right_sign = 0
        elif self.focus == RIGHT:
            left_sign = 0
        self.node.drive(left_sign * self.duty, right_sign * self.duty,
                        hold_s=self.key_hold_s)

    def rotate(self, degrees: float) -> None:
        for s in self.sides():
            self.node.rotate(s, degrees, duty=self.duty)
        self.say(f"rotate {FOCUS_NAME[self.focus]} {degrees:+.0f} deg "
                 f"at duty {self.duty}")

    # ------------------------------------------------------------- drawing

    def render(self):
        parts = [self.header(), self.motors()]
        if self.mode != "props" or self.show_help:
            parts.append(self.body())
        parts += [self.lower(), self.footer()]
        return Group(*parts)

    def header(self):
        n = self.node
        st = n.driver.state
        t = Table.grid(expand=True)
        t.add_column(ratio=1)
        t.add_column(justify="center", ratio=1)
        t.add_column(justify="right", ratio=1)

        link = (Text("LINK OK", style="bold green") if st.fresh else
                Text("NO TELEMETRY", style="bold red"))
        if not st.connected:
            link = Text(f"NO PORT: {st.error[:40]}", style="bold red")

        if n.estopped:
            middle = Text(f"  E-STOP  {n.estop_reason}  ",
                          style="bold white on red")
        else:
            driving = any(n.motors[s].status.duty for s in (LEFT, RIGHT))
            middle = Text(f"focus {FOCUS_NAME[self.focus]}   duty {self.duty}"
                          f"   step {self.step_deg:g} deg", style="bold")
            if driving:
                middle = Text("  DRIVE  ", style="bold black on green") + middle
        t.add_row(Text(f"{self.port}  ", style="cyan") + link, middle,
                  Text("p properties   ? help   x quit", style="grey62"))
        return Panel(t, border_style="grey35", padding=(0, 1))

    def motors(self):
        t = Table.grid(expand=True, padding=(0, 1))
        t.add_column(ratio=1)
        t.add_column(ratio=1)
        t.add_row(self.motor_panel(LEFT), self.motor_panel(RIGHT))
        return t

    def motor_panel(self, side: int):
        m = self.node.motors[side]
        s, cfg = m.status, m.cfg
        focused = self.focus in (side, BOTH)

        g = Table.grid(expand=True)
        g.add_column(width=11, style="grey62")
        g.add_column(ratio=1)

        style = MODE_STYLE.get(s.mode, "white")
        mode = Text(s.mode.upper(), style=style)
        if s.fault:
            mode = Text(f"{s.mode.upper()}  {s.fault}", style=MODE_STYLE["fault"])
        g.add_row("state", mode)

        g.add_row("angle", Text(f"{s.position_deg:+11.2f} deg   "
                                f"({s.angle_deg:6.2f} of 360)"))
        if s.hold_armed:
            g.add_row("target", Text(f"{s.target_deg:+11.2f} deg"))
            err_style = ("green" if abs(s.error_deg) <= cfg.hold_tolerance_deg
                         else "yellow")
            g.add_row("error", Text(f"{s.error_deg:+11.2f} deg", style=err_style)
                      + Text(f"   tol {cfg.hold_tolerance_deg:g}", style="grey62"))
            g.add_row("", bar(s.error_deg, max(cfg.hold_tolerance_deg * 6,
                                               abs(s.error_deg) * 1.2)))
        else:
            g.add_row("target", Text("not armed  (m to latch here)",
                                     style="grey62"))
            g.add_row("error", Text("--", style="grey62"))
            g.add_row("", Text(""))

        g.add_row("speed", Text(f"{s.speed_dps:+8.1f} deg/s"
                                f"   duty {s.duty:+4d}"))
        g.add_row("hold", Text("ON " if cfg.hold_position else "OFF",
                               style="green" if cfg.hold_position else "grey62")
                  + Text(f"  corrections {s.corrections}/{cfg.hold_max_corrections}"
                         f"  recoveries {s.recoveries}", style="grey62"))
        g.add_row("peak off", Text(f"{s.peak_error_deg:+8.2f} deg", style="grey62"))
        enc = ("green" if s.magnet_ok else "yellow" if s.encoder_online else "red")
        g.add_row("encoder", Text(
            f"{'online' if s.encoder_online else 'silent'}, magnet "
            f"{'ok' if s.magnet_ok else 'marginal'}", style=enc))
        g.add_row("last move", Text(s.last_move or "--", style="grey62"))

        title = f"{s.label}" + ("" if cfg.enabled else "  [disabled]")
        return Panel(g, title=title, border_style="cyan" if focused else "grey35",
                     padding=(0, 1))

    def body(self):
        f = self.node.frame
        g = Table.grid(expand=True, padding=(0, 2))
        for _ in range(4):
            g.add_column()
        if f is None:
            return Panel(Text("waiting for telemetry", style="grey62"),
                         border_style="grey35", padding=(0, 1))
        pose = (f"x {f.x:+.3f}  y {f.y:+.3f}  th {f.theta_deg:+.1f}"
                if f.have_pose else "no track width set")
        g.add_row(Text(f"distance {f.distance_m:+.3f} m"),
                  Text(f"path {f.path_m:.3f} m"),
                  Text(pose, style="" if f.have_pose else "grey62"),
                  Text("odometry trustworthy" if f.trustworthy
                       else "odometry NOT trustworthy",
                       style="green" if f.trustworthy else "yellow"))
        return Panel(g, title="body", border_style="grey35", padding=(0, 1))

    def lower(self):
        if self.show_help:
            return self.help_panel()
        if self.mode == "props":
            return self.props_panel()
        return self.events_panel()

    def events_panel(self):
        evs = self.node.events[-8:]
        if not evs:
            body = Text("nothing has happened yet", style="grey62")
        else:
            t = Table.grid(expand=True, padding=(0, 1))
            t.add_column(width=8, style="grey50")
            t.add_column(width=6)
            t.add_column(ratio=1)
            for e in evs:
                t.add_row(time.strftime("%H:%M:%S", time.localtime(e.t)),
                          Text(e.side[:5], style="grey62"),
                          Text(e.message, style=EVENT_STYLE.get(e.kind, "white")))
            body = t
        return Panel(body, title="events", border_style="grey35", padding=(0, 1))

    def props_panel(self):
        n = self.node
        t = Table.grid(expand=True, padding=(0, 1))
        t.add_column(width=2)
        t.add_column(width=26)
        t.add_column(width=16, justify="right")
        t.add_column(ratio=1, style="grey62", overflow="ellipsis")

        last_kind = None
        for i, p in enumerate(PROPERTIES):
            if p.kind != last_kind:
                label = ("CRITICAL -- wrong values here damage the bot"
                         if p.kind == CRITICAL else
                         "OPTIONAL -- tuning and comfort")
                t.add_row("", "", "",
                          Text(label, style="bold "
                               + ("red" if p.kind == CRITICAL else "cyan")))
                last_kind = p.kind
            sel = i == self.prop_index
            vals = []
            for s in self.sides():
                v = n.motors[s].cfg.get(p.name)
                vals.append(("on" if v else "off") if p.type is bool
                            else (f"{v:g}" if isinstance(v, float) else str(v)))
            shown = vals[0] if len(set(vals)) == 1 else " / ".join(vals)
            if p.unit:
                shown += f" {p.unit}"
            style = "bold reverse" if sel else ""
            doc = p.doc if sel else p.doc.split(".")[0]
            t.add_row(Text(">" if sel else " ", style=style),
                      Text(p.name, style=style),
                      Text(shown, style=style or "bold"),
                      Text(doc, no_wrap=not sel))
        return Panel(t, padding=(0, 1), border_style="cyan",
                     title=f"properties -- editing {FOCUS_NAME[self.focus]}",
                     subtitle="up/down select   left/right change   enter "
                              "toggle   0 default   tab motor   esc back")

    def help_panel(self):
        rows = [
            ("w a s d / arrows", "drive -- hold the key, release to stop"),
            ("space", "stop now"),
            ("E", "e-stop / clear e-stop"),
            ("[ ]", "duty down / up"),
            ("< >", "rotate step size down / up"),
            ("j l", "closed-loop rotate the focused motor -step / +step"),
            ("h", "hold on/off for the focused motor"),
            ("m", "latch the hold target here"),
            ("r", "release the hold target"),
            ("n", "enable / disable the focused motor"),
            ("c", "clear faults and the e-stop"),
            ("z", "zero the encoders and re-latch"),
            ("tab / 1 2 3", "focus LEFT, RIGHT or BOTH"),
            ("p", "property editor"),
            ("? x", "this help / quit"),
        ]
        t = Table.grid(expand=True, padding=(0, 2))
        t.add_column(width=18, style="bold cyan")
        t.add_column(ratio=1)
        for k, v in rows:
            t.add_row(k, v)
        return Panel(t, title="keys", border_style="cyan", padding=(0, 1))

    def footer(self):
        if time.time() < self.message_until:
            return Panel(Text(self.message, style="bold yellow"),
                         border_style="grey35", padding=(0, 1))
        bad = self.node.invalid_properties()
        if bad:
            return Panel(Text("; ".join(bad), style="bold red"),
                         border_style="red", padding=(0, 1))
        hint = ("w/a/s/d drive   space stop   E e-stop   h hold   m latch   "
                "j/l rotate   z zero   c clear")
        return Panel(Text(hint, style="grey62"), border_style="grey35",
                     padding=(0, 1))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--track-width", type=float, default=None,
                    help="wheel centre-to-centre in metres, for body pose")
    ap.add_argument("--no-hold", action="store_true",
                    help="start with position hold switched off")
    ap.add_argument("--duty", type=int, default=90)
    ap.add_argument("--rate", type=float, default=25.0)
    ap.add_argument("--key-hold", type=float, default=KEY_HOLD_S,
                    help="how long a drive key stays live without a repeat; "
                         "must exceed your keyboard repeat delay (xset q)")
    args = ap.parse_args()

    cfg = MotorConfig(hold_position=not args.no_hold)
    console = Console()
    with WheelDriver(args.port) as bot:
        node = MotorNode(bot, config=cfg, rate_hz=args.rate,
                         track_width_m=args.track_width)
        ui = MotorTUI(node, args.port)
        ui.duty = max(1, min(255, args.duty))
        ui.key_hold_s = max(0.1, args.key_hold)
        node.start()
        try:
            with RawKeys() as keys, Live(ui.render(), console=console,
                                         refresh_per_second=15,
                                         screen=True) as live:
                while ui.running:
                    k = keys.read()
                    if k:
                        ui.handle(k)
                    live.update(ui.render())
                    time.sleep(0.03)
        except KeyboardInterrupt:
            pass
        finally:
            node.halt()
            node.stop()

    console.print("[grey62]motors stopped.[/grey62]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
