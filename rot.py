#!/usr/bin/env python3
"""
Plain terminal rotate. No TUI.

    python rot.py

Asks which wheel, how many degrees, what speed. Rotates. Repeats.
Ctrl-C or 'q' to quit.
"""

import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
MIN_DUTY, MAX_DUTY = 58, 255      # 58 is just above the measured deadband

last_speed = 90


def ask(prompt, default=None, cast=str, lo=None, hi=None, allow=None):
    while True:
        shown = f"{prompt}" + (f" [{default}]" if default is not None else "")
        raw = input(shown + ": ").strip()
        if raw.lower() in ("q", "quit", "exit"):
            raise SystemExit(0)
        if not raw and default is not None:
            raw = str(default)
        if allow:
            if raw.upper() in allow:
                return raw.upper()
            print(f"   pick one of {', '.join(allow)}")
            continue
        try:
            v = cast(raw)
        except ValueError:
            print("   not a number")
            continue
        if lo is not None and v < lo:
            print(f"   minimum is {lo}")
            continue
        if hi is not None and v > hi:
            print(f"   maximum is {hi}")
            continue
        return v


def main():
    global last_speed
    try:
        s = serial.Serial(PORT, 115200, timeout=0.2)
    except Exception as e:
        raise SystemExit(f"cannot open {PORT}: {e}")
    time.sleep(2.0)
    s.reset_input_buffer()
    s.write(b"X\n")

    print(f"connected on {PORT}   (q to quit)\n")

    while True:
        wheel = ask("wheel  L / R", "L", allow=("L", "R"))
        deg = ask("degrees (negative = reverse)", 90, float)
        speed = ask(f"speed {MIN_DUTY}-{MAX_DUTY}", last_speed, int,
                    MIN_DUTY, MAX_DUTY)
        last_speed = speed

        s.reset_input_buffer()
        s.write(f"G {wheel} {deg} {speed}\n".encode())
        print(f"   rotating {wheel} {deg:+g}deg at speed {speed} ...")

        # Wait for the board to report the finished move.
        buf, t0, done = b"", time.time(), False
        while time.time() - t0 < 40:
            try:
                chunk = s.read(4096)
            except Exception as e:
                print(f"   serial error: {e}")
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if line.startswith("# GODONE,"):
                    p = line[9:].split(",")
                    try:
                        print(f"   done: commanded {float(p[1]):+.1f}deg   "
                              f"actual {float(p[2]):+.2f}deg   "
                              f"error {float(p[3]):+.2f}deg   ({p[4]})")
                    except (ValueError, IndexError):
                        print("   " + line)
                    done = True
                elif line.startswith("# ERR"):
                    print("   " + line)
                    done = True
            if done:
                break
        else:
            print("   timed out waiting for the board")
        print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopping motors")
        try:
            serial.Serial(PORT, 115200, timeout=0.2).write(b"X\n")
        except Exception:
            pass
