#!/usr/bin/env python3
"""Talk to the Slamtec RPLIDAR on the CP2102 and print scans. No library needed.

    python3 lidar_probe.py                       # /dev/ttyUSB1 @ 256000, live sweep summary
    python3 lidar_probe.py --raw                 # print every (angle, distance) point
    python3 lidar_probe.py --port /dev/ttyUSB1 --baud 256000 --seconds 5

RPLIDAR wire protocol (Slamtec "Interface Protocol" doc):
  host -> lidar   A5 CMD           (GET_INFO 50, GET_HEALTH 52, SCAN 20, STOP 25, RESET 40)
  lidar -> host   A5 5A <len:30b|mode:2b> <type>   7-byte response descriptor, then payload
  SCAN payload    5-byte nodes: [q<<2|!S|S] [angle_q6<<1|1] [angle_q6>>7] [dist_q2 lo] [dist_q2 hi]
                  S=1 marks the first node of a new 360 deg sweep. angle = angle_q6/64 deg,
                  distance = dist_q2/4 mm (0 = no return).
Motor: on A1/A2 the USB adapter's DTR line drives MOTOCTL. DTR low = spin.
"""
import argparse, math, struct, sys, time
import serial

def cmd(s, c):
    s.write(bytes([0xA5, c]))

def descriptor(s):
    d = s.read(7)
    if len(d) != 7 or d[:2] != b"\xa5\x5a":
        raise RuntimeError(f"bad descriptor {d.hex(' ')!r}")
    ln = struct.unpack("<I", d[2:6])[0]
    return ln & 0x3FFFFFFF, ln >> 30, d[6]

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=256000)
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl-C)")
    ap.add_argument("--raw", action="store_true", help="print every point instead of a per-sweep summary")
    a = ap.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=1)
    s.dtr = False            # motor on
    cmd(s, 0x25); time.sleep(0.05); s.reset_input_buffer()   # STOP any old scan

    cmd(s, 0x50); ln, _, typ = descriptor(s); info = s.read(ln)
    print(f"model 0x{info[0]:02x}  firmware {info[2]}.{info[1]}  hw {info[3]}  serial {info[4:].hex()}")
    cmd(s, 0x52); ln, _, typ = descriptor(s); h = s.read(ln)
    print("health:", ["GOOD", "WARNING", "ERROR"][h[0]], f"code {struct.unpack('<H', h[1:3])[0]}")
    if h[0] == 2:
        print("lidar reports ERROR -- sending RESET, try again in 2 s"); cmd(s, 0x40); return 1

    time.sleep(1.0)          # let the motor reach speed
    cmd(s, 0x20); ln, mode, typ = descriptor(s)
    print(f"scanning (node {ln} B, type 0x{typ:02x}) -- Ctrl-C to stop\n")

    sweep, t0, t_sweep, n_sweeps = [], time.time(), time.time(), 0
    try:
        while not a.seconds or time.time() - t0 < a.seconds:
            b = s.read(5)
            if len(b) < 5: print("timeout -- no data"); continue
            if (b[0] & 1) == (b[0] >> 1 & 1) or not (b[1] & 1):   # resync on bad flags
                s.read(1); continue
            start = b[0] & 1
            q = b[0] >> 2
            ang = ((b[1] >> 1) | (b[2] << 7)) / 64.0
            dist = ((b[3]) | (b[4] << 8)) / 4.0
            if start and sweep:
                n_sweeps += 1
                good = [(an, d) for an, d, _ in sweep if d > 0]
                dt = time.time() - t_sweep; t_sweep = time.time()
                if not a.raw:
                    nearest = min(good, key=lambda p: p[1]) if good else (0, 0)
                    sectors = []
                    for c in (0, 90, 180, 270):
                        pts = [d for an, d in good if abs((an - c + 180) % 360 - 180) < 15]
                        sectors.append(f"{c:3d}deg:{(sum(pts)/len(pts))/1000 if pts else 0:5.2f}m")
                    print(f"sweep {n_sweeps:4d}  {len(sweep):4d} pts ({len(good)} valid)  {1/dt if dt else 0:4.1f} Hz  "
                          f"nearest {nearest[1]/1000:.2f} m @ {nearest[0]:.0f}deg  |  " + "  ".join(sectors))
                sweep = []
            sweep.append((ang, dist, q))
            if a.raw and dist > 0:
                print(f"{ang:7.2f} deg  {dist/1000:6.3f} m  q={q}")
    except KeyboardInterrupt:
        pass
    finally:
        cmd(s, 0x25); time.sleep(0.05); s.dtr = True; s.close()   # stop scan, motor off
        print(f"\nstopped after {n_sweeps} sweeps")
    return 0

if __name__ == "__main__":
    sys.exit(main())
