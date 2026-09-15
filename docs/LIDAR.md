# Lidar, IMU and the sensor web GUI

Bring-up notes for the perception side of the car, as measured on
2026-09-15. Everything here runs on the **car computer** (`mt10`, Ubuntu
22.04 x86-64, Python 3.10), not on the ESP32. The ESP32 motor board is a
separate device that the car computer talks to over Ethernet — see
`ETHERNET.md` in the car-computer checkout.

## What is plugged in

Two USB-serial adapters. Identified by sniffing both ports, not by assumption:

| Device node | Adapter | What it is | Baud | How it was identified |
|---|---|---|---|---|
| `/dev/ttyUSB0` | CH340 (`1a86:7523`) | WitMotion IMU | 115200 | streams unsolicited 11-byte `0x55 0x51/52/53` frames, checksum verifies |
| `/dev/ttyUSB1` | CP2102 (`10c4:ea60`) | Slamtec RPLIDAR | **256000** | silent until asked; answers `A5 50` (GET_INFO) with a valid `A5 5A` descriptor |

The `ttyUSBn` numbers depend on plug-in order. Scripts should use the stable
names under `/dev/serial/by-id/`:

```
usb-1a86_USB_Serial-if00-port0                                         -> IMU
usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_a220999c70a3904db2202db75d2bd39d-if00-port0  -> lidar
```

### Lidar identity

From the `RESET` banner and `GET_INFO`:

```
RP LIDAR System.
Firmware Ver 1.32 - rtm, HW Ver 6
Model: 2c
S/N: DD98ED93C0EA98C7A0E69BF54D464560
```

Measured with the SDK: **~16 500 points/s, 9.6 sweeps/s**, ~1 700 points per
sweep, of which roughly half have a valid return indoors. That data rate and
the 256000 baud put it in the A2M12/A3 class.

## The one thing that cost the most time

**This firmware does not stream in the legacy `A5 20` SCAN mode.** It
accepts the command — the response descriptor `A5 5A 05 00 00 40 81` comes
back — and then sends nothing, forever. Health reports GOOD. Every motor-start
variant (DTR either polarity, `SET_MOTOR_PWM` 660 and 1023) gives the same
silence. It looks exactly like a lidar whose motor is not spinning.

It is not. The Slamtec SDK negotiates one of the express/dense scan modes
and gets data immediately. So: **use the SDK**, or implement express scan.
`sensors/lidar_probe.py` is the legacy-mode reader kept as a record of this;
do not expect it to produce points on this unit.

## Slamtec SDK

Source: <https://github.com/Slamtec/rplidar_sdk> (`master`, SDK 2.1.0).
The car computer has no internet, so the tarball was downloaded on a laptop
and copied over with `scp`.

```sh
tar xzf rplidar_sdk.tar.gz && mv rplidar_sdk-master ~/rplidar_sdk
cd ~/rplidar_sdk && make -j4
ls output/Linux/Release/     # ultra_simple simple_grabber frame_grabber custom_baudrate libsl_lidar_sdk.a
```

Its Makefiles break on a path containing a space, which is why it lives at
`~/rplidar_sdk` and not inside `~/autonomus car`.

Quick test, which is also what the web GUI runs underneath:

```sh
~/rplidar_sdk/output/Linux/Release/ultra_simple --channel --serial /dev/ttyUSB1 256000
```

Output is one point per line, `S ` on the first point of each sweep:

```
S  theta: 0.16 Dist: 00000.00 Q: 0
   theta: 4.37 Dist: 02224.00 Q: 47
```

Ctrl-C stops the scan and parks the motor (`drv->stop()` then
`setMotorSpeed(0)`), so SIGINT is the correct way to stop it from another
program.

## IMU frame format

WitMotion, 11 bytes, fixed: `0x55 TYPE d0 d1 d2 d3 d4 d5 d6 d7 SUM` where
`SUM` is the low byte of the sum of the first ten. Four little-endian int16.

| TYPE | Content | Scale |
|---|---|---|
| `0x51` | accel x y z, temp | `/32768 × 16` g |
| `0x52` | gyro x y z, temp | `/32768 × 2000` °/s |
| `0x53` | roll pitch yaw, version | `/32768 × 180` ° |

Verified on the car: roll −4.4°, pitch 2.6°, yaw −71° with the car at rest.

## `sensors/lidar_web.py` — web GUI

A single-file server, **Python standard library only** (the car computer
cannot `pip install`), that wraps `ultra_simple`, cuts its stream into sweeps
and serves a live page plus a JSON API.

```sh
python3 lidar_web.py [--http-port 8080] [--lidar-port /dev/ttyUSB1] [--baud 256000] [--imu-port /dev/ttyUSB0] [--autostart]
```

Then open `http://<car-computer-ip>:8080` from anything on the LAN.

What the page shows:

- Live polar plot of the current sweep, range rings, 30° spokes, red marker on
  the nearest return. Scroll/pinch to zoom, drag to pan, double-click to reset.
- **Start scan / Stop.** Stop sends SIGINT to `ultra_simple`, so the motor
  parks — it is not just closing the pipe.
- Config: lidar port (populated from `/dev/serial/by-id`, labelled by adapter
  chip), baud, plot max range, minimum-quality filter, IMU port. *Apply*
  restarts the scan if one is running.
- Lidar identity and health, sweep counter, sweeps/s, points/s.
- IMU roll / pitch / yaw, accel, gyro, live.
- Header status dots: lidar, IMU, and the ESP32 motor board (pinged every 3 s).
- Last lines of `ultra_simple`'s output, for debugging.

JSON API, for `odom.py`, a ROS bridge, or anything else:

| Endpoint | Returns |
|---|---|
| `GET /api/scan` | `{sweep: [[deg, mm, quality], …], stats, info, running, error, cfg}` — the latest complete sweep |
| `GET /api/status` | lidar (without the sweep), imu, car ping, detected ports, SDK present, host |
| `GET /api/start` · `/api/stop` | start / stop scanning |
| `GET /api/config?port=…&baud=…&max_range_m=…&min_quality=…&imu_port=…` | change settings |

Deliberately absent: **motor control**. Driving stays in `motor_tui.py`,
which has a key-hold watchdog. A browser button with no dead-man's switch
would let one dropped Wi-Fi packet keep the car moving.

### Running it

Ad hoc, survives logout but not reboot:

```sh
./lidar_web.sh            # nohup, --autostart, log in /tmp/lidar_web.log
```

As a service (installed on the car computer):

```sh
cp lidar-web.service ~/.config/systemd/user/     # edit the paths first
systemctl --user daemon-reload
systemctl --user enable --now lidar-web
systemctl --user status lidar-web
journalctl --user -u lidar-web -f
sudo loginctl enable-linger mt10                 # once, so it starts at boot without a login
```

Without linger, user services start only when that user logs in — one `ssh`
after a reboot is enough.

## Gotchas met along the way

- **`pkill -f lidar_web.py` over ssh kills your own session.** The pattern
  matches the `bash -c '…'` that ssh is running. Use `pkill -f "[l]idar_web"`
  and keep the kill and the start in separate ssh invocations.
- **`'xterm-kitty': unknown terminal type`** on the car computer: kitty's
  terminfo is missing there. `infocmp -x xterm-kitty | ssh mt10@… 'tic -x -'`
  fixes it, or use `kitten ssh`.
- **The car computer has no internet.** `pip download` on a laptop, `scp`
  the wheels, `pip3 install --user --no-index *.whl`. `rich` needed
  `markdown-it-py`, `mdurl`, `pygments`.
- **`rot.py` took its port positionally** and opened `serial.Serial()`
  directly, so `tcp://…` failed with *No such file or directory*. It now goes
  through `carbot.driver._open_transport` like everything else.

## Status

Verified on hardware: lidar identified and streaming through the SDK; the
web GUI plotting the room (walls ~4 m out, a 0.47 m obstacle at 347°) from a
browser on another machine, with the IMU live beside it; start/stop parks
and restarts the motor; the systemd unit is active.

Not yet done: `sllidar_ros2` in `ros2_ws` to publish `/scan`; the ESP32 motor
board was unreachable at its documented `192.168.1.77` from the car computer
on the day (its segment has a DHCP server at `.1`, so the board may be taking
a lease instead of falling back to the static address).
