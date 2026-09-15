#!/usr/bin/env python3
"""
Wheel odometry node -- absolute wheel angle, distance, and unintended rotation.

Runs standalone (no ROS needed) and prints a live readout; with --json it also
writes one JSON object per sample, which is the form a SLAM front end or a bag
replay wants.

    python wheel_odom_node.py                        # live table
    python wheel_odom_node.py /dev/ttyACM0 --track-width 0.30
    python wheel_odom_node.py --json odom.jsonl --quiet
    python wheel_odom_node.py --rate 50 --zero       # zero counters at start

Track width is the wheel centre-to-centre separation in metres. Without it the
per-wheel numbers are all still exact; only the body pose (x, y, heading) is
withheld, because it would be fiction. calibration.json ships with it unset --
measure it once and either pass --track-width or fill in the field.

Ctrl-C stops it and prints a summary of everything that moved unbidden.
"""

from __future__ import annotations

import argparse
import sys
import time

from carbot.driver import DEFAULT_PORT  # honours CARBOT_PORT
from carbot import WheelDriver
from carbot.odometry import WheelOdometryNode


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT)
    ap.add_argument("--rate", type=float, default=20.0, help="sample rate, Hz")
    ap.add_argument("--track-width", type=float, default=None,
                    help="wheel separation in metres; enables body pose")
    ap.add_argument("--json", metavar="FILE",
                    help="append one JSON frame per sample to FILE ('-' for stdout)")
    ap.add_argument("--zero", action="store_true",
                    help="zero the encoder counters before starting")
    ap.add_argument("--quiet", action="store_true", help="no live table")
    args = ap.parse_args()

    drv = WheelDriver(args.port)
    if not drv.wait_for_telemetry(timeout=6.0):
        print(f"no telemetry on {args.port}: {drv.state.error or 'silent'}",
              file=sys.stderr)
        drv.close()
        return 1

    node = WheelOdometryNode(drv, track_width_m=args.track_width)
    if args.zero:
        node.zero_encoders()

    sink = None
    if args.json:
        sink = sys.stdout if args.json == "-" else open(args.json, "a")

    def on_event(ev):
        line = f"[{time.strftime('%H:%M:%S')}] UNINTENDED {ev.kind}: {ev.message}"
        print(("\r" + line).ljust(100), file=sys.stderr)

    node.on_event = on_event
    if not args.quiet:
        print(f"port {args.port}   rate {args.rate:g} Hz   "
              f"track width {node.track_width or 'UNSET (no pose)'}")
        print("Ctrl-C to stop.\n")

    last_print = 0.0
    try:
        while True:
            t0 = time.time()
            f = node.update(t0)
            if sink:
                sink.write(f.as_json() + "\n")
                sink.flush()
            if not args.quiet and t0 - last_print > 0.1:
                last_print = t0
                sys.stdout.write("\r" + render(f) + "  ")
                sys.stdout.flush()
            time.sleep(max(0.0, 1.0 / args.rate - (time.time() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        summarise(node)
        if sink and sink is not sys.stdout:
            sink.close()
        drv.close()
    return 0


def render(f) -> str:
    def wheel(w):
        # 359.995 would print as "360.0", which reads as a wheel that is one
        # whole turn off rather than one that is a hair short of the origin.
        ang = w.wheel_angle_deg % 360.0
        if ang > 359.95:
            ang = 0.0
        return (f"{w.side[0]} ang {ang:6.1f}deg "
                f"({w.revolutions:+7.2f}rev) {w.distance_m:+7.3f}m "
                f"{w.speed_mps:+5.2f}m/s")
    body = (f"| pose {f.x:+6.3f},{f.y:+6.3f} hdg {f.theta_deg:+7.1f}deg"
            if f.have_pose else "| pose n/a")
    flag = "" if f.trustworthy else " !SLIP"
    live = "" if f.fresh else " [STALE]"
    return f"{wheel(f.left)} | {wheel(f.right)} {body}{flag}{live}"


def summarise(node) -> None:
    f = node.frame
    print("--- summary ---")
    for w in (f.left, f.right):
        print(f"{w.side:5s}  absolute angle {w.wheel_angle_deg:7.2f} deg   "
              f"{w.revolutions:+8.3f} rev   net {w.distance_m:+.4f} m   "
              f"path {w.path_m:.4f} m   motor shaft {w.motor_angle_deg:6.2f} deg")
    if f.have_pose:
        print(f"BODY   x {f.x:+.4f} m  y {f.y:+.4f} m  heading {f.theta_deg:+.2f} deg"
              f"   path {f.path_m:.4f} m")
    else:
        print(f"BODY   net travel {f.distance_m:+.4f} m   path {f.path_m:.4f} m"
              "   (no heading: track width unset)")
    if node.events:
        print(f"\n{len(node.events)} unintended-motion event(s):")
        for ev in node.events[-20:]:
            print(f"  {time.strftime('%H:%M:%S', time.localtime(ev.t))} "
                  f"{ev.kind:26s} {ev.message}")
        print("Odometry after the first event should be treated as suspect by SLAM.")
    else:
        print("\nNo unintended motion seen.")


if __name__ == "__main__":
    raise SystemExit(main())
