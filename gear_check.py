#!/usr/bin/env python3
"""
Measure the motor-to-wheel gear ratio, the one number the encoder cannot see.

WHY THIS TOOL EXISTS
--------------------
The AS5600 is mounted on the MOTOR shaft, before the gearbox. It counts motor
revolutions and nothing else. The gear ratio is exactly the quantity that sits
between the encoder and the wheel, so it can never be recovered from encoder
data alone -- it needs one observation of the WHEEL, from outside.

This tool makes that observation as precise as possible by amplifying it. It
commands a large, whole number of WHEEL revolutions using the ratio the
firmware currently believes, and you report how far the tyre mark landed from
where it started. Any error in the ratio shows up multiplied by the number of
revolutions, so ten turns makes a 1% error into a very visible 36 degrees.

    true_ratio = believed_ratio * commanded_deg / (commanded_deg - shortfall)

WHY NOT JUST HAND-TURN IT (wheel_cal.py)
----------------------------------------
Hand-turning is biased LOW and always in the same direction. You turn the
wheel to a mark, but the first part of that motion is absorbed taking up
gearbox backlash, during which the wheel moves and the motor shaft does not.
Those counts are missing from every reading, so counts-per-wheel-rev comes out
too small on both wheels alike. Two wheels agreeing closely does not rule this
out -- a systematic bias moves them both together.

Driving the wheel under power, as this tool does, loads the gear teeth against
one flank for the whole move, so backlash is taken up once at the start rather
than being counted as travel.

    python gear_check.py [port] [--side L|R] [--revs 10]
"""

import argparse
import json
import math
import os
import sys
import time

import serial

HERE = os.path.dirname(os.path.abspath(__file__))
CAL_PATH = os.path.join(HERE, "calibration.json")

# Ratios that actually exist as catalogue parts near this range. A measurement
# landing within a percent of one of these is almost certainly that part.
CATALOGUE = [
    (14.0, "14:1"), (15.0, "15:1"), (16.0, "16:1"), (18.75, "18.75:1 (75/4)"),
    (19.0, "19:1"), (20.0, "20:1"), (21.3, "21.3:1"), (25.0, "25:1"),
    (30.0, "30:1"), (34.0, "34:1"), (48.0, "48:1"),
]


class Board:
    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=0.4)
        time.sleep(2.5)                 # USB CDC re-enumerates on open
        self.s.reset_input_buffer()

    def send(self, line):
        self.s.write((line + "\n").encode())
        self.s.flush()

    def ask(self, line, prefix, timeout=8.0):
        """Send a command and return the first reply line starting with prefix."""
        self.s.reset_input_buffer()
        self.send(line)
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.s.readline().decode("utf-8", "replace").strip()
            if raw.startswith(prefix):
                return raw
        return None

    def wait_for(self, prefix, timeout=90.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.s.readline().decode("utf-8", "replace").strip()
            if raw.startswith(prefix):
                return raw
        return None

    def close(self):
        try:
            self.send("X")
            time.sleep(0.2)
        finally:
            self.s.close()


def nearest_catalogue(r):
    best, err = None, None
    for value, label in CATALOGUE:
        e = abs(value - r) / r * 100.0
        if err is None or e < err:
            best, err = (value, label), e
    return best, err


def main():
    ap = argparse.ArgumentParser(
        description="Measure the motor-to-wheel gear ratio.")
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--side", default="L", choices=list("LRlr"),
                    help="which wheel to turn (default L)")
    ap.add_argument("--revs", type=float, default=10.0,
                    help="whole WHEEL revolutions to command (default 10)")
    ap.add_argument("--duty", type=int, default=90)
    args = ap.parse_args()
    side = args.side.upper()

    b = Board(args.port)
    try:
        cur = b.ask("N", "# N,")
        if not cur:
            print("No reply to N -- is this the closed-loop firmware?",
                  file=sys.stderr)
            return 1
        fields = dict(kv.split("=", 1) for kv in cur[4:].split(",") if "=" in kv)
        believed = float(fields["gear"])
        wheel_mm = float(fields["wheel_mm"])
        print(f"firmware currently believes: gear {believed:.4f}:1, "
              f"wheel {wheel_mm:.1f} mm")

        commanded_deg = args.revs * 360.0
        print()
        print(f"About to turn the {side} wheel {args.revs:g} full revolutions "
              f"({commanded_deg:.0f} deg).")
        print("1. Mark the tyre and the frame so the two marks line up exactly.")
        print("2. Make sure the wheel can turn freely and nothing will snag.")
        input("   Press ENTER when the marks are aligned and you are ready... ")

        b.send(f"G {side} {commanded_deg:.1f} {args.duty}")
        print("   turning...", end="", flush=True)
        done = b.wait_for("# GODONE", timeout=180.0)
        print(" done.")
        if done:
            print(f"   firmware says: {done}")

        print()
        print("Now read the tyre mark against the frame mark.")
        print("  SHORT  = the tyre stopped BEFORE lining up (positive number)")
        print("  OVER   = the tyre went PAST the line      (negative number)")
        print("Estimate in degrees of WHEEL rotation. A quarter turn is 90.")
        raw = input("Shortfall in degrees [0 = landed perfectly]: ").strip()
        try:
            shortfall = float(raw or "0")
        except ValueError:
            print("Not a number.", file=sys.stderr)
            return 1

        actual_deg = commanded_deg - shortfall
        if abs(actual_deg) < 1.0:
            print("That shortfall is implausibly large.", file=sys.stderr)
            return 1

        # The wheel actually turned `actual_deg` while the firmware metered out
        # counts for `commanded_deg` at the believed ratio. Counts delivered are
        # fixed, so the true ratio scales by commanded/actual.
        true_ratio = believed * commanded_deg / actual_deg
        err_pct = (true_ratio - believed) / believed * 100.0

        print()
        print("-" * 62)
        print(f"  commanded        {commanded_deg:10.1f} deg of wheel")
        print(f"  actually turned  {actual_deg:10.1f} deg of wheel")
        print(f"  believed ratio   {believed:10.4f} : 1")
        print(f"  MEASURED ratio   {true_ratio:10.4f} : 1   ({err_pct:+.2f}%)")

        (cat_val, cat_label), cat_err = nearest_catalogue(true_ratio)
        print(f"  nearest catalogue part: {cat_label}  ({cat_err:+.2f}% away)")
        if cat_err < 1.0:
            print(f"  -> within 1%, so the real gearbox is almost certainly "
                  f"{cat_label}.")
            print(f"     Using the exact catalogue value {cat_val} is better "
                  f"than the\n     measurement, because the true ratio is an "
                  f"exact gear-tooth count.")
        else:
            print("  -> not close to a standard ratio; re-measure with more "
                  "revolutions\n     before trusting it.")

        # Resolution of this measurement: how much ratio error 1 degree of
        # reading error corresponds to. Tells the user if --revs was enough.
        per_deg = believed * commanded_deg / (commanded_deg - 1.0) - believed
        print(f"  reading resolution: 1 deg of eyeball error = "
              f"{per_deg:.4f} in the ratio ({per_deg/believed*100:.3f}%)")
        print("-" * 62)

        choice = cat_val if cat_err < 1.0 else round(true_ratio, 4)
        print()
        ans = input(f"Apply {choice} to the firmware and calibration.json? "
                    f"[y/N] ").strip().lower()
        if ans != "y":
            print("Left unchanged. Set it later with:  N " + str(choice))
            return 0

        reply = b.ask(f"N {choice}", "# N,")
        print("firmware:", reply)
        _write_calibration(choice, wheel_mm / 1000.0)
        print(f"calibration.json updated.")
        print("Re-run this tool to confirm the wheel now lands on the mark.")
        return 0
    finally:
        b.close()


def _write_calibration(ratio, wheel_d_m):
    """Fold the measured ratio back into calibration.json, keeping history."""
    try:
        with open(CAL_PATH) as f:
            cal = json.load(f)
    except Exception as e:
        print(f"could not read {CAL_PATH}: {e}", file=sys.stderr)
        return

    cpr = cal.get("counts_per_motor_rev", 4096)
    cwr = cpr * ratio
    circ = math.pi * wheel_d_m

    cal.setdefault("measured_gear_ratio_history", []).append({
        "ratio": cal.get("combined", {}).get("gear_ratio"),
        "replaced": time.strftime("%Y-%m-%d %H:%M:%S"),
    })
    cal["combined"] = {
        "counts_per_wheel_rev": round(cwr, 1),
        "gear_ratio": ratio,
        "counts_per_degree": round(cwr / 360.0, 3),
        "counts_per_metre": round(cwr / circ, 1),
    }
    cal["gear_ratio_note"] = (
        "Measured with gear_check.py by driving the wheel a whole number of "
        "revolutions under power and reading the tyre mark. This supersedes "
        "the per-wheel hand-turned figures, which are biased low by gearbox "
        "backlash. ONE ratio applies to both wheels -- they share a gearbox "
        "design, so a per-wheel difference is measurement error, and baking it "
        "in makes the car curve."
    )
    with open(CAL_PATH, "w") as f:
        json.dump(cal, f, indent=2)
        f.write("\n")


if __name__ == "__main__":
    sys.exit(main())
