#!/usr/bin/env python3
"""
Hold position, and rotate a wheel by degrees. Plain terminal, no TUI.

Two jobs in one tool, because they are the two things you keep wanting while
the car is on the bench:

    HOLD    freeze both wheels where they are and actively resist a shove.
            This is the firmware's `H` -- a real closed-loop position hold,
            not just "motors off". It is exempt from the 300 ms command
            watchdog but expires on its own after about two minutes, so this
            script re-sends it to keep it alive for as long as you leave it up.

    WHEEL   rotate a wheel by N degrees of WHEEL rotation (firmware `G`).
            The board drives, brakes, settles and then trims, and reports the
            angle it actually achieved.

    python hold_rotate.py                      # interactive
    python hold_rotate.py hold
    python hold_rotate.py wheel L 90           # left wheel +90 deg
    python hold_rotate.py wheel B -180 --duty 110

Everything here is in the FORWARD-POSITIVE frame on both wheels -- WheelDriver
applies the mirror correction, so +90 means the same thing on both sides.

Wheel degrees only: nothing here depends on the track width, so nothing here
is affected by the fact that it is still an unmeasured placeholder.
"""

import argparse
import sys
import time

from carbot.driver import DEFAULT_PORT  # honours CARBOT_PORT
from carbot.driver import LEFT, RIGHT, MODE_HOLD, WheelDriver

MIN_DUTY, MAX_DUTY = 58, 255      # 58 is just above the measured deadband
HOLD_REFRESH_S = 60.0             # board drops the hold at ~120 s

# Runaway guard for `hold`. The firmware's hold has no deadband -- velSp is
# KP_POS * posErr straight into a feedforward whose FF_DEAD term is 49.3 duty,
# so any error past ~0.125 mm applies a ~50 duty kick. On a wheel whose sign
# or friction is not right, that does not settle: the error and the duty ramp
# together and the wheel spins up. Watch for exactly that shape and bail out.
RUNAWAY_ERR_MM = 5.0              # error this big is not a hold any more
RUNAWAY_DUTY = 120                # ... and this much duty means it is pushing
RUNAWAY_TICKS = 5                 # consecutive samples before calling it


# --------------------------------------------------------------------- hold

def _runaway(w) -> bool:
    """A wheel is running away when the error is large AND the loop is flooring
    it. Either alone is fine: a big error with no duty is just a stalled motor,
    and a big duty with a shrinking error is the hold doing its job."""
    return abs(w.track_error_mm) > RUNAWAY_ERR_MM and abs(w.duty) > RUNAWAY_DUTY


def hold(bot: WheelDriver) -> bool:
    """
    Engage position hold and keep it engaged until Ctrl-C.

    Returns False if it had to abort on a runaway. Push a wheel and watch the
    error and duty: the error should grow while you push and then come back to
    zero when you let go. What it must NOT do is grow while the duty grows --
    that is the loop driving the wheel further from where it was asked to be.
    """
    bot.hold_position()
    print("holding position -- push a wheel and it should push back."
          "\nCtrl-C to release.\n")
    last = time.time()
    bad = 0
    try:
        while True:
            if time.time() - last >= HOLD_REFRESH_S:
                bot.hold_position()          # beat the ~2 min board timeout
                last = time.time()
            s = bot.state
            engaged = "HOLD" if s.mode == MODE_HOLD else s.mode_name.upper()
            print(f"\r  {engaged:8s} "
                  f"L {s.left.fwd_metres * 1000:+8.2f} mm "
                  f"(err {s.left.track_error_mm:+6.2f}, duty {s.left.duty:+4d})   "
                  f"R {s.right.fwd_metres * 1000:+8.2f} mm "
                  f"(err {s.right.track_error_mm:+6.2f}, duty {s.right.duty:+4d})",
                  end="", flush=True)

            runaway = [n for n, w in (("LEFT", s.left), ("RIGHT", s.right))
                       if _runaway(w)]
            bad = bad + 1 if runaway else 0
            if bad >= RUNAWAY_TICKS:
                bot.stop()
                bot.release()
                print(f"\n\nABORTED: {' and '.join(runaway)} is running away --"
                      f" the error grew while the loop drove harder against it."
                      f"\nThat is positive feedback, not a hold. Motors stopped."
                      f"\nCheck that side's drive and encoder signs with the"
                      f" firmware spin test (`S`): a forward step must RAISE"
                      f" that side's forward count.")
                return False
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nreleasing")
        bot.release()
        return True


# -------------------------------------------------------------- wheel degrees

def rotate_wheel(bot: WheelDriver, side_name: str, degrees: float,
                 duty: int) -> bool:
    """One `G` move. Returns True if the board reported it reached the target."""
    side = LEFT if side_name == "L" else RIGHT
    print(f"   rotating {side_name} {degrees:+g}deg at duty {duty} ...")
    r = bot.rotate(side, degrees, duty)
    if r is None:
        print("   no result")
        return False
    if r.reason == "timeout":
        print("   timed out waiting for the board")
        return False
    print(f"   done: commanded {r.commanded_deg:+.1f}deg   "
          f"actual {r.actual_deg:+.2f}deg   "
          f"error {r.error_deg:+.2f}deg   ({r.reason})")
    return r.ok


def rotate_wheels(bot: WheelDriver, which: str, degrees: float,
                  duty: int) -> bool:
    """
    `which` is L, R or B. B runs the two sides one after the other, not
    together: the firmware services one `G` at a time and a second one would
    preempt the first.
    """
    if which != "B":
        return rotate_wheel(bot, which, degrees, duty)
    ok = rotate_wheel(bot, "L", degrees, duty)
    time.sleep(0.2)
    return rotate_wheel(bot, "R", degrees, duty) and ok


# -------------------------------------------------------------- interactive

def ask(prompt, default=None, cast=str, lo=None, hi=None, allow=None):
    while True:
        shown = prompt + (f" [{default}]" if default is not None else "")
        raw = input(shown + ": ").strip()
        if raw.lower() in ("q", "quit", "exit"):
            raise SystemExit(0)
        if not raw and default is not None:
            raw = str(default)
        if allow:
            if raw.upper() in allow:
                return raw.upper()
            print(f"   pick one of {', '.join(allow)}")
            continue
        try:
            v = cast(raw)
        except ValueError:
            print("   not a number")
            continue
        if lo is not None and v < lo:
            print(f"   minimum is {lo}")
            continue
        if hi is not None and v > hi:
            print(f"   maximum is {hi}")
            continue
        return v


def menu(bot: WheelDriver) -> None:
    last_duty = 90
    while True:
        print("  [h] hold position   [w] rotate a wheel by degrees   "
              "[e] release   [q] quit")
        what = ask("what", "w", allow=("H", "W", "E", "Q"))
        if what == "Q":
            return
        if what == "E":
            bot.release()
            print("   released -- wheels free\n")
            continue
        if what == "H":
            hold(bot)
            print()
            continue
        which = ask("wheel  L / R / B", "L", allow=("L", "R", "B"))
        deg = ask("wheel degrees (negative = reverse)", 90, float)
        last_duty = ask(f"duty {MIN_DUTY}-{MAX_DUTY}", last_duty, int,
                        MIN_DUTY, MAX_DUTY)
        rotate_wheels(bot, which, deg, last_duty)
        print()


# --------------------------------------------------------------------- main

def main() -> int:
    ap = argparse.ArgumentParser(
        description="Hold position, or rotate a wheel by degrees.")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--duty", type=int, default=90,
                    help=f"rotate duty, {MIN_DUTY}-{MAX_DUTY} (default 90)")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("hold", help="freeze both wheels until Ctrl-C")
    w = sub.add_parser("wheel", help="rotate a wheel by wheel degrees")
    w.add_argument("side", choices=["L", "R", "B", "l", "r", "b"])
    w.add_argument("degrees", type=float)
    sub.add_parser("release", help="disengage the loop, wheels coast free")
    a = ap.parse_args()

    if not MIN_DUTY <= a.duty <= MAX_DUTY:
        print(f"duty must be {MIN_DUTY}-{MAX_DUTY}", file=sys.stderr)
        return 2

    bot = WheelDriver(a.port)
    if not bot.wait_for_telemetry(timeout=6.0):
        print(f"no telemetry from {a.port}: {bot.state.error or 'silent'}",
              file=sys.stderr)
        bot.close()
        return 1
    print(f"connected on {a.port}   mode={bot.state.mode_name}   "
          f"L{'+' if bot.state.left.online else '-'} "
          f"R{'+' if bot.state.right.online else '-'}\n")

    rc = 0
    try:
        if a.cmd == "hold":
            rc = 0 if hold(bot) else 1
        elif a.cmd == "wheel":
            rc = 0 if rotate_wheels(bot, a.side.upper(), a.degrees,
                                    a.duty) else 1
        elif a.cmd == "release":
            bot.release()
            print("released -- wheels free")
        else:
            menu(bot)
    except KeyboardInterrupt:
        print("\nstopping motors")
    finally:
        if a.cmd != "hold":
            bot.stop()
        bot.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
