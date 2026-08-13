#!/usr/bin/env python3
"""Reset the board, let the scan finish, then spin each motor once."""
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
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


pump(6)                      # boot + full diagnostic sequence
for cmd, name in (("l", "LEFT"), ("k", "RIGHT")):
    print(f"\n>>> sending '{cmd}' ({name})", flush=True)
    s.write((cmd + "\n").encode())
    pump(5)

s.write(b"s\n")              # belt and braces: force everything low
pump(1)
s.close()
