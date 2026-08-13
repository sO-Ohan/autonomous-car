#!/usr/bin/env python3
"""
Prove, from the encoders, that DRIVE goes straight and TURN rotates.

The encoders cannot see the robot, only the wheels -- but the *pattern* is
decisive. With the per-wheel direction constants applied:

  straight  ->  both wheels report the SAME signed forward distance
  turning   ->  they report OPPOSITE signed forward distances

So this converts each raw encoder delta to "forward metres" using DIR_L/DIR_R
and checks the signs agree with the command that was issued.

    python3 check_directions.py [10.42.7.1]
"""
import sys, time
from link import connect_and_sync

target = sys.argv[1] if len(sys.argv) > 1 else "10.42.7.1"
s, _ = connect_and_sync(target)
buf = b""


def pump(secs, want=None):
    global buf
    t0 = time.time()
    hit = None
    while time.time() - t0 < secs:
        buf += s.read(1024)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("utf-8", "replace").strip()
            if want and t.startswith(want):
                hit = t
    return hit


def cmd(c):
    s.write((c + "\n").encode())
    time.sleep(0.05)


def geom():
    cmd("GEOM")
    t0 = time.time()
    global buf
    while time.time() - t0 < 2:
        buf += s.read(1024)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("utf-8", "replace").strip()
            if t.startswith("GEOM,"):
                p = t.split(",")
                return {"cpm": float(p[1]), "track": float(p[2]),
                        "dirL": int(p[4]) if len(p) > 4 else 1,
                        "dirR": int(p[5]) if len(p) > 5 else 1}
    return {"cpm": 10905.0, "track": 0.0, "dirL": 1, "dirR": 1}


def pos():
    """Latest raw encoder counts for both wheels."""
    global buf
    cmd("T")
    t0 = time.time()
    last = None
    while time.time() - t0 < 1.5:
        buf += s.read(1024)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("utf-8", "replace").strip()
            if t.startswith("D,"):
                p = t.split(",")
                if len(p) >= 13:
                    last = (int(p[1]), int(p[6]))
    cmd("T")
    time.sleep(0.2)
    return last


g = geom()
print(f"geometry: cpm {g['cpm']:.0f}   track {g['track']:.0f} mm   "
      f"DIR_L {g['dirL']:+d}  DIR_R {g['dirR']:+d}\n")

TESTS = [
    ("DRIVE 0.10", "forward 10 cm", "same"),
    ("DRIVE -0.10", "back 10 cm", "same"),
    ("SPIN 45", "turn left", "opposite"),
    ("SPIN -45", "turn right", "opposite"),
]

print(f"{'command':13s} {'meaning':14s} {'left':>10s} {'right':>10s}   verdict")
print("-" * 68)
ok = True
for c, meaning, expect in TESTS:
    before = pos()
    cmd(c)
    pump(9, want="DONE")
    time.sleep(0.6)
    after = pos()
    if not before or not after:
        print(f"{c:13s} {meaning:14s}   no telemetry")
        ok = False
        continue

    # convert raw counts to signed FORWARD distance for each wheel
    dl = (after[0] - before[0]) * g["dirL"] / g["cpm"] * 100
    dr = (after[1] - before[1]) * g["dirR"] / g["cpm"] * 100
    same = (dl > 0) == (dr > 0)
    got = "same" if same else "opposite"
    good = (got == expect)
    ok &= good
    verdict = ("STRAIGHT" if same else "ROTATING") + ("  ok" if good else "  <-- WRONG")
    print(f"{c:13s} {meaning:14s} {dl:+8.2f}cm {dr:+8.2f}cm   {verdict}")

print("-" * 68)
print("PASS: w/s drive straight, a/d rotate" if ok else
      "FAIL: try  WDIR R -1  (or +1) to flip the right wheel convention")
cmd("X")
s.close()
