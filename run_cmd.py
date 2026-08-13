#!/usr/bin/env python3
"""Reset, wait for boot, send a command, capture the result."""
import sys, time, serial

port = sys.argv[1]
cmd = sys.argv[2]
wait = float(sys.argv[3]) if len(sys.argv) > 3 else 14.0

s = serial.Serial(port, 115200, timeout=0.2)
s.setDTR(False); s.setRTS(True); time.sleep(0.15)
s.setRTS(False); time.sleep(0.05)
s.reset_input_buffer()


def pump(secs):
    t0 = time.time()
    while time.time() - t0 < secs:
        d = s.read(4096)
        if d:
            sys.stdout.write(d.decode("utf-8", "replace"))
            sys.stdout.flush()


pump(6)
sys.stdout.write(f"\n>>> sending '{cmd}'\n")
sys.stdout.flush()
s.write((cmd + "\n").encode())
pump(wait)
s.write(b"s\n")
pump(1)
s.close()
