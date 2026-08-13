#!/usr/bin/env python3
"""Headless check of the closed-loop move and calibration protocol."""
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
s = serial.Serial(port, 115200, timeout=0.1)
s.setDTR(False); s.setRTS(True); time.sleep(0.15); s.setRTS(False)
time.sleep(5.5)
s.reset_input_buffer()

buf = b""
def pump(secs, want=None, show=()):
    """Read for secs, or until a line starting with `want` arrives."""
    global buf
    t0 = time.time()
    while time.time() - t0 < secs:
        buf += s.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            t = line.decode("utf-8", "replace").strip()
            if not t:
                continue
            if any(t.startswith(p) for p in show):
                print("   ", t)
            if want and t.startswith(want):
                return t
    return None

def cmd(c):
    s.write((c + "\n").encode())

print("=== closed-loop moves ===")
cmd("Z")
time.sleep(0.2)
for wheel, deg in (("L", 90), ("L", -90), ("R", 180), ("R", -180)):
    cmd(f"P {wheel} {deg}")
    r = pump(8, want="DONE", show=("ERR",))
    print(f"  P {wheel} {deg:+5}  ->  {r if r else 'NO DONE (timed out)'}")

print("\n=== calibration, LEFT ===")
cmd("C L")
r = pump(70, want="CALDONE", show=("CALDEAD", "CALPT", "CALSLOPE", "CALABORT"))
print("  ", r or "NO CALDONE (timed out)")

print("\n=== calibration, RIGHT ===")
cmd("C R")
r = pump(70, want="CALDONE", show=("CALDEAD", "CALPT", "CALSLOPE", "CALABORT"))
print("  ", r or "NO CALDONE (timed out)")

print("\n=== post-calibration moves (deadband kick now active) ===")
for wheel, deg in (("L", 45), ("R", 45), ("L", 10), ("R", 10)):
    cmd(f"P {wheel} {deg}")
    r = pump(8, want="DONE")
    print(f"  P {wheel} {deg:+5}  ->  {r if r else 'NO DONE (timed out)'}")

cmd("X")
time.sleep(0.3)
s.close()
