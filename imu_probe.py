#!/usr/bin/env python3
"""
Read and decode a WitMotion IMU (HWT905-TTL and friends) over serial.

Run this the moment the IMU is plugged in. It finds the baud rate, decodes the
packet stream, and reports the things that actually decide whether the IMU is
worth fusing: the real output rate, and the YAW DRIFT while sitting still.

    python imu_probe.py                      # auto-detect port and baud
    python imu_probe.py /dev/ttyUSB0
    python imu_probe.py /dev/ttyUSB0 --baud 115200 --drift 60

WHY DRIFT IS THE NUMBER THAT MATTERS
------------------------------------
For this car the IMU has one job the wheels cannot do: measure how much the
BODY actually turned, independently of what the wheels did. Wheel odometry is
blind to slip, to being shoved sideways, and (because the encoder sits before
the gearbox) to backlash and to a spinning wheel. The gyro sees all of those.

But a gyro's yaw is an integral, so a constant bias becomes an error that grows
linearly forever. Sitting still for 60 s and watching how far yaw wanders tells
you directly how long you can trust it between corrections. Under about
0.5 deg/min is good enough to keep this car straight over a room; several
deg/min means you need continuous bias nulling (see --zupt below).

WITMOTION PACKET FORMAT
-----------------------
Fixed 11-byte frames: 0x55, TYPE, then four little-endian int16, then a
checksum that is the low byte of the sum of the first ten. Each TYPE carries a
different quartet:

    0x51 acceleration    ax ay az temp     +-16 g full scale
    0x52 angular rate    wx wy wz temp     +-2000 deg/s full scale
    0x53 euler angle     roll pitch yaw    +-180 deg
    0x54 magnetometer    hx hy hz temp     raw counts
    0x59 quaternion      q0 q1 q2 q3       normalised
"""

import argparse
import glob
import math
import statistics
import sys
import time

import serial

HDR = 0x55
SCALE_ACC = 16.0 * 9.80665 / 32768.0     # -> m/s^2
SCALE_GYR = 2000.0 / 32768.0             # -> deg/s
SCALE_ANG = 180.0 / 32768.0              # -> deg
COMMON_BAUDS = [115200, 9600, 230400, 921600, 460800, 57600, 38400, 19200]


def i16(lo, hi):
    v = lo | (hi << 8)
    return v - 65536 if v & 0x8000 else v


class WitDecoder:
    """Feeds on raw bytes, yields (type, [v0..v3]) for each valid frame."""

    def __init__(self):
        self.buf = bytearray()
        self.good = 0
        self.bad = 0

    def feed(self, data):
        self.buf.extend(data)
        out = []
        while len(self.buf) >= 11:
            if self.buf[0] != HDR:
                # Not aligned. Drop one byte and try again rather than
                # discarding the whole buffer -- a partial frame at the start
                # is normal on the first read.
                del self.buf[0]
                continue
            frame = self.buf[:11]
            if (sum(frame[:10]) & 0xFF) != frame[10]:
                self.bad += 1
                del self.buf[0]
                continue
            vals = [i16(frame[2 + 2 * i], frame[3 + 2 * i]) for i in range(4)]
            out.append((frame[1], vals))
            self.good += 1
            del self.buf[:11]
        return out


def try_baud(port, baud, seconds=1.2):
    """Return (frames_decoded, checksum_failures) at this baud."""
    try:
        s = serial.Serial(port, baud, timeout=0.2)
    except Exception:
        return 0, 0
    try:
        time.sleep(0.25)
        s.reset_input_buffer()
        d = WitDecoder()
        end = time.time() + seconds
        while time.time() < end:
            chunk = s.read(512)
            if chunk:
                d.feed(chunk)
        return d.good, d.bad
    finally:
        s.close()


def find_port():
    """Prefer USB-serial adapters; never return the ESP32's own port."""
    cands = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    out = []
    for p in cands:
        try:
            import os
            link = os.path.realpath(p)
            ids = glob.glob("/dev/serial/by-id/*")
            name = ""
            for i in ids:
                if os.path.realpath(i) == link:
                    name = os.path.basename(i)
                    break
            # The ESP32-C3 enumerates as an Espressif JTAG/serial debug unit.
            if "Espressif" in name or "JTAG" in name:
                continue
            out.append(p)
        except Exception:
            out.append(p)
    return out


def main():
    ap = argparse.ArgumentParser(description="Probe a WitMotion IMU.")
    ap.add_argument("port", nargs="?", help="serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, help="skip baud detection")
    ap.add_argument("--drift", type=float, default=30.0, metavar="SEC",
                    help="how long to sit still measuring yaw drift (0 to skip)")
    args = ap.parse_args()

    port = args.port
    if not port:
        cands = find_port()
        if not cands:
            print("No candidate serial port found.\n\n"
                  "The ESP32 is on ttyACM0 and is deliberately skipped. Plug the\n"
                  "IMU in through a USB-TTL adapter and it should appear as\n"
                  "/dev/ttyUSB0. Check with:  ls -l /dev/serial/by-id/",
                  file=sys.stderr)
            return 1
        port = cands[0]
        print(f"using port {port}"
              f"{'  (others: ' + ', '.join(cands[1:]) + ')' if len(cands) > 1 else ''}")

    # ---- baud ----------------------------------------------------------
    if args.baud:
        baud = args.baud
        good, bad = try_baud(port, baud)
        if not good:
            print(f"No valid frames at {baud} baud.", file=sys.stderr)
            return 1
    else:
        print("detecting baud", end="", flush=True)
        best = (0, 0, None)
        for b in COMMON_BAUDS:
            print(".", end="", flush=True)
            good, bad = try_baud(port, b)
            if good > best[0]:
                best = (good, bad, b)
            if good > 40 and bad == 0:
                break
        print()
        good, bad, baud = best
        if not baud or good == 0:
            print("\nNo WitMotion frames found at any common baud rate.\n"
                  "Check: TX of the IMU goes to RX of the adapter, both share a\n"
                  "ground, and the module is powered. If it was reconfigured in\n"
                  "the WitMotion app, pass --baud explicitly.", file=sys.stderr)
            return 1
        print(f"locked on: {baud} baud  ({good} frames, {bad} checksum failures)")

    # ---- live decode ---------------------------------------------------
    s = serial.Serial(port, baud, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()
    d = WitDecoder()

    seen = {}
    latest = {}
    t0 = time.time()
    print("\nreading for 3 s to see which packet types this unit emits...")
    while time.time() - t0 < 3.0:
        chunk = s.read(512)
        if chunk:
            for typ, vals in d.feed(chunk):
                seen[typ] = seen.get(typ, 0) + 1
                latest[typ] = vals

    names = {0x50: "time", 0x51: "acceleration", 0x52: "angular rate",
             0x53: "euler angle", 0x54: "magnetometer", 0x55: "port status",
             0x56: "pressure", 0x57: "lat/lon", 0x58: "gps", 0x59: "quaternion"}
    print(f"\n{'type':>6} {'name':<16} {'rate':>9}   values")
    for typ in sorted(seen):
        rate = seen[typ] / 3.0
        v = latest[typ]
        if typ == 0x51:
            txt = ("ax {:+7.3f}  ay {:+7.3f}  az {:+7.3f} m/s2"
                   .format(*[x * SCALE_ACC for x in v[:3]]))
        elif typ == 0x52:
            txt = ("wx {:+8.3f}  wy {:+8.3f}  wz {:+8.3f} deg/s"
                   .format(*[x * SCALE_GYR for x in v[:3]]))
        elif typ == 0x53:
            txt = ("roll {:+8.2f}  pitch {:+8.2f}  yaw {:+8.2f} deg"
                   .format(*[x * SCALE_ANG for x in v[:3]]))
        elif typ == 0x54:
            txt = "hx {:+6d}  hy {:+6d}  hz {:+6d}".format(*v[:3])
        elif typ == 0x59:
            txt = ("q {:+.4f} {:+.4f} {:+.4f} {:+.4f}"
                   .format(*[x / 32768.0 for x in v]))
        else:
            txt = str(v)
        print(f"  0x{typ:02X} {names.get(typ,'?'):<16} {rate:>6.1f} Hz   {txt}")

    if 0x52 not in seen:
        print("\nWARNING: no angular-rate (0x52) packets. That is the one this\n"
              "car actually needs -- yaw rate is what the wheels cannot measure.\n"
              "Enable it in the WitMotion config app.")

    # ---- drift ---------------------------------------------------------
    if args.drift > 0 and (0x52 in seen or 0x53 in seen):
        print(f"\n{'='*64}\nYAW DRIFT TEST -- keep the IMU COMPLETELY STILL for "
              f"{args.drift:.0f} s\n{'='*64}")
        input("press ENTER when it is sitting still and settled... ")
        s.reset_input_buffer()
        d = WitDecoder()
        gz, yaws, t = [], [], []
        start = time.time()
        while time.time() - start < args.drift:
            chunk = s.read(512)
            if not chunk:
                continue
            for typ, vals in d.feed(chunk):
                now = time.time() - start
                if typ == 0x52:
                    gz.append(vals[2] * SCALE_GYR)
                    t.append(now)
                elif typ == 0x53:
                    yaws.append((now, vals[2] * SCALE_ANG))
            done = int(40 * (time.time() - start) / args.drift)
            print(f"\r  [{'#'*done}{'.'*(40-done)}]", end="", flush=True)
        print()

        print()
        if gz:
            bias = statistics.mean(gz)
            noise = statistics.pstdev(gz)
            print(f"  gyro Z bias        {bias:+.4f} deg/s"
                  f"   -> {bias*60:+.2f} deg/min of yaw drift if uncorrected")
            print(f"  gyro Z noise (1SD) {noise:.4f} deg/s")
            print(f"  samples            {len(gz)} in {args.drift:.0f} s"
                  f"  ({len(gz)/args.drift:.1f} Hz)")
        if len(yaws) > 2:
            dy = yaws[-1][1] - yaws[0][1]
            if dy > 180: dy -= 360
            if dy < -180: dy += 360
            span = yaws[-1][0] - yaws[0][0]
            print(f"  reported yaw moved {dy:+.3f} deg in {span:.1f} s"
                  f"   ({dy/span*60:+.2f} deg/min)")
            drift = abs(dy / span * 60)
            print()
            if drift < 0.5:
                print("  VERDICT: excellent. Good enough to hold heading across a\n"
                      "           room with only occasional correction.")
            elif drift < 3.0:
                print("  VERDICT: usable, but bias-null it whenever the wheels say\n"
                      "           stopped (a zero-velocity update). That is free on\n"
                      "           this car -- the encoders already know when it is\n"
                      "           standing still.")
            else:
                print("  VERDICT: high. Re-run after a longer warm-up, and check the\n"
                      "           module is not being disturbed by vibration or a\n"
                      "           nearby motor. Bias nulling is mandatory.")
    s.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
