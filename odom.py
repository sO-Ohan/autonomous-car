#!/usr/bin/env python3
"""
Live wheel-odometry and straight-line tracking readout.

Shows what the closed loop is actually doing, from the firmware's FB stream:
per-wheel distance and speed, how far each wheel is from where the controller
says it should be, and SKEW -- the difference in distance travelled between
the two wheels, which is the number that says whether the car is going
straight. On a good run SKEW stays under a millimetre over metres of travel.

    python odom.py [port]                 # watch only, motors never touched
    python odom.py [port] --drive 0.2     # drive straight at 0.2 m/s
    python odom.py [port] --drive 0.2 --distance 1.0   # then stop and hold

Watch mode is completely passive: it sends nothing that can move a wheel, so
it is safe to leave running while you push the car around by hand.
"""

import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from carbot.driver import WheelDriver, MODE_NAME   # noqa: E402

CLR = "\033[2J\033[H"
HIDE, SHOW = "\033[?25l", "\033[?25h"


def bar(value, span, width=21):
    """Centre-zero bar. `span` is the half-range mapped to the full width."""
    half = width // 2
    n = int(round(max(-1.0, min(1.0, value / span)) * half))
    cells = ["-"] * width
    cells[half] = "|"
    if n:
        step = 1 if n > 0 else -1
        for i in range(half + step, half + n + step, step):
            if 0 <= i < width:
                cells[i] = "#"
    return "".join(cells)


def render(st, cmd, elapsed):
    L, R = st.left, st.right
    if not st.feedback_fresh:
        return (CLR + "waiting for the FB telemetry stream...\n\n"
                "If this never arrives the board is running the OLD open-loop\n"
                "firmware. Reflash from ~/drive-harness-diag/c3-webgui.\n")

    skew = st.skew_mm
    flag = "  <-- CURVING" if abs(skew) > 5.0 else ""
    gov = "  GOVERNED (a wheel is being held back)" if st.governed else ""

    out = [CLR]
    out.append(f"  WHEEL ODOMETRY            mode: {st.mode_name.upper():8s}"
               f"  t={elapsed:6.1f}s{gov}")
    out.append("  " + "=" * 68)
    out.append(f"  {'':10s}{'distance':>12s}{'speed':>11s}{'target':>11s}"
               f"{'track err':>12s}{'duty':>7s}")
    for name, w in (("LEFT", L), ("RIGHT", R)):
        live = "" if w.online else "  NO I2C"
        out.append(f"  {name:<10s}{w.fwd_metres:>10.4f} m{w.velocity:>9.3f} "
                   f"{w.setpoint:>10.3f} {w.track_error_mm:>+10.2f} mm"
                   f"{w.duty:>7d}{live}")
    out.append("")
    out.append(f"  body travel   {st.distance:>9.4f} m"
               f"      heading {st.theta * 57.2958:>+8.2f} deg")
    out.append(f"  pose          x {st.x:>+8.4f}  y {st.y:>+8.4f} m"
               f"      track {st.track_width:.3f} m")
    out.append("")
    out.append("  " + "-" * 68)
    out.append(f"  SKEW (left minus right travel)   {skew:>+9.3f} mm{flag}")
    out.append(f"     {bar(skew, 10.0)}   +-10 mm scale")
    if abs(st.distance) > 0.01:
        ppm = abs(skew) / (abs(st.distance) * 1000.0) * 100.0
        out.append(f"  as a fraction of distance travelled: {ppm:.4f}%")
    out.append("  " + "-" * 68)
    out.append("")
    if cmd:
        out.append(f"  driving: {cmd}")
    else:
        out.append("  watch only -- nothing is being commanded")
    out.append("  Ctrl-C to stop")
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser(description="Live wheel odometry readout.")
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--drive", type=float, metavar="MPS",
                    help="closed-loop velocity, both wheels, m/s")
    ap.add_argument("--distance", type=float, metavar="M",
                    help="stop and hold after this much travel")
    ap.add_argument("--zero", action="store_true",
                    help="zero the counters and pose before starting")
    args = ap.parse_args()

    bot = WheelDriver(args.port)
    if not bot.wait_for_telemetry(timeout=8.0):
        print("no telemetry from the board", file=sys.stderr)
        return 1
    if args.zero or args.drive is not None:
        bot.zero()
        time.sleep(0.4)

    t0 = time.time()
    cmd = ""
    holding = False
    sys.stdout.write(HIDE)
    try:
        while True:
            st = bot.state
            if args.drive is not None and not holding:
                if (args.distance is not None
                        and abs(st.distance) >= abs(args.distance)):
                    bot.hold_position()
                    holding = True
                    cmd = (f"reached {args.distance:.3f} m -- holding position")
                else:
                    bot.set_velocity(args.drive, args.drive)
                    cmd = f"V {args.drive:.3f} {args.drive:.3f} m/s"
            sys.stdout.write(render(st, cmd, time.time() - t0))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW + "\n")
        if args.drive is not None:
            bot.stop()
        bot.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
