#!/usr/bin/env python3
"""
Measure move quality at the full control-loop rate.

For each commanded move this pulls the firmware's 200 Hz position trace and
reports peak overshoot past the target, how long settling took, and how much
the wheel dithered after arriving -- the numbers you cannot get from 50 Hz
telemetry, where the wheel moves ~70 deg between samples at top speed.

    python3 analyze_moves.py [port] [--csv DIR]
"""
import sys, time, os, serial

port = "/dev/ttyUSB0"
csvdir = None
args = sys.argv[1:]
for i, a in enumerate(args):
    if a == "--csv" and i + 1 < len(args):
        csvdir = args[i + 1]
    elif not a.startswith("--") and (i == 0 or args[i - 1] != "--csv"):
        port = a

CPR = 4096.0
s = serial.Serial(port, 115200, timeout=0.1)
s.setDTR(False); s.setRTS(True); time.sleep(0.15); s.setRTS(False)
time.sleep(5.5); s.reset_input_buffer()
buf = b""


def pump(secs, want=None, collect=None, out=None):
    global buf
    t0 = time.time()
    while time.time() - t0 < secs:
        buf += s.read(512)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("utf-8", "replace").strip()
            if not t:
                continue
            if collect and t.startswith(collect) and out is not None:
                out.append(t)
            if want and t.startswith(want):
                return t
    return None


def cmd(c):
    s.write((c + "\n").encode())
    time.sleep(0.04)


def get_trace():
    lines = []
    cmd("DUMP")
    pump(6, want="TRCEND", collect="T", out=lines)
    target, vals = None, []
    for t in lines:
        p = t.split(",")
        if p[0] == "TRC":
            target = int(p[2])
        elif p[0] == "TD":
            vals.extend(int(x) for x in p[2:])
    return target, vals


def analyse(target, vals):
    """Peak overshoot past target, settle time, and post-arrival dither."""
    if not vals or target is None or target == 0:
        return None
    sign = 1 if target > 0 else -1
    # overshoot = furthest travel beyond the target, in the direction of motion
    peak = max((v * sign for v in vals), default=0)
    over = max(0.0, peak - abs(target))

    # settle time: last moment it was outside a 3-count band around target
    band = 3
    last_out = 0
    for i, v in enumerate(vals):
        if abs(v - target) > band:
            last_out = i
    settle_s = last_out / 200.0

    # dither: total path travelled after first arrival, minus net movement
    first_in = next((i for i, v in enumerate(vals) if abs(v - target) <= band), None)
    dither = 0
    if first_in is not None:
        seg = vals[first_in:]
        dither = sum(abs(seg[i + 1] - seg[i]) for i in range(len(seg) - 1))
    return {
        "final": vals[-1] - target,
        "over_cts": over,
        "over_deg": over * 360.0 / CPR,
        "settle_s": settle_s,
        "dither_cts": dither,
        "n": len(vals),
    }


# Calibration is required: without it the deadband is zero, the stiction kick
# is disabled, and every move undershoots. Ask the firmware directly rather
# than watching for the boot banner -- the banner is printed before this script
# finishes waiting out the boot, so it gets flushed away with the input buffer.
lines = []
cmd("CALQ")
pump(1.0, collect="CALLOAD", out=lines)
have = {l.split(",")[1]: l.split(",")[2] == "1" for l in lines if len(l.split(",")) > 3}
if have and all(have.values()):
    print("using stored calibration: " + "  ".join(
        l for l in lines if l.startswith("CALLOAD")))
else:
    print("no stored calibration, running it now...")
    for w in ("L", "R"):
        cmd(f"C {w}")
        pump(70, want="CALDONE")

cmd("TRACE")               # arm tracing
pump(0.5)
print(f"{'move':>12}  {'overshoot':>16}  {'final err':>11}  {'settle':>8}  {'dither':>8}")
print("-" * 68)

results = []
for wheel, deg in (("L", 90), ("L", 360), ("R", 90), ("R", 360), ("L", -360), ("R", -90)):
    cmd(f"P {wheel} {deg}")
    done = pump(12, want="DONE")
    time.sleep(0.5)
    target, vals = get_trace()
    a = analyse(target, vals)
    if not a:
        print(f"{wheel} {deg:+5}    no trace captured")
        continue
    results.append((wheel, deg, a))
    # A move opposite to the approach direction deliberately runs past the
    # target to take up backlash, so its "overshoot" is planned, not a fault.
    planned = " (planned take-up)" if deg < 0 else ""
    print(f"{wheel} {deg:+6}°  {a['over_cts']:7.0f} cts {a['over_deg']:6.2f}°  "
          f"{a['final']:+5d} cts  {a['settle_s']:6.2f} s  {a['dither_cts']:6.0f} cts{planned}")
    if csvdir:
        os.makedirs(csvdir, exist_ok=True)
        fn = os.path.join(csvdir, f"move_{wheel}_{deg}.csv")
        with open(fn, "w") as f:
            f.write("sample,seconds,counts,degrees,target_counts\n")
            for i, v in enumerate(vals):
                f.write(f"{i},{i/200.0:.4f},{v},{v*360.0/CPR:.3f},{target}\n")

if results:
    print("-" * 68)
    mo = max(a["over_deg"] for _, _, a in results)
    mf = max(abs(a["final"]) for _, _, a in results) * 360.0 / CPR
    ms = max(a["settle_s"] for _, _, a in results)
    print(f"worst overshoot {mo:.2f}°   worst final error {mf:.3f}°   "
          f"worst settle {ms:.2f} s")
if csvdir:
    print(f"traces written to {csvdir}/")

# ---- repeatability -------------------------------------------------------
# The real test with backlash present: return to the same commanded position
# repeatedly and measure the spread. Lost motion shows up here as drift even
# when each individual move reports a small error.
print("\nrepeatability: 6 cycles of +90/-90, net drift from start")
cmd("TRACE")            # disarm tracing so DUMP is not in the way
pump(0.3)
cmd("Z")
time.sleep(0.3)
cmd("T")                # telemetry on, to read absolute position
pump(0.5)

def read_pos(wheel, timeout=2.0):
    global buf
    t0 = time.time()
    idx = 1 if wheel == "L" else 6
    while time.time() - t0 < timeout:
        buf += s.read(512)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("utf-8", "replace").strip()
            if t.startswith("D,"):
                p = t.split(",")
                if len(p) >= 13:
                    return int(p[idx])
    return None

for wheel in ("L", "R"):
    start = read_pos(wheel)
    drifts = []
    for _ in range(6):
        cmd(f"P {wheel} 90")
        pump(12, want="DONE")
        cmd(f"P {wheel} -90")
        pump(12, want="DONE")
        time.sleep(0.3)
        now = read_pos(wheel)
        if now is not None and start is not None:
            drifts.append((now - start) * 360.0 / CPR)
    if drifts:
        print(f"  {wheel}: " + "  ".join(f"{d:+.2f}°" for d in drifts))
        print(f"     final drift {drifts[-1]:+.3f}°   "
              f"spread {max(drifts)-min(drifts):.3f}°")

cmd("X")
cmd("T")
s.close()
