#!/usr/bin/env python3
"""Reset the ESP32 and capture its serial output for N seconds."""
import sys, time, serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
send = sys.argv[3] if len(sys.argv) > 3 else None

s = serial.Serial(port, 115200, timeout=0.2)
# hardware reset: EN=DTR/RTS toggle used by esp32 dev boards
s.setDTR(False); s.setRTS(True); time.sleep(0.15)
s.setRTS(False); time.sleep(0.05)
s.reset_input_buffer()

t0 = time.time()
sent = False
while time.time() - t0 < secs:
    data = s.read(4096)
    if data:
        sys.stdout.write(data.decode("utf-8", "replace"))
        sys.stdout.flush()
    if send and not sent and time.time() - t0 > secs * 0.45:
        s.write((send + "\n").encode())
        sent = True
s.close()
