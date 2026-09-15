"""
Serial driver for the ESP32-C3 differential-drive base.

Pure Python, no ROS dependency, so the same class backs the ROS 2 node, the
terminal tools and anything else you bolt on.

    from carbot import WheelDriver

    with WheelDriver("/dev/ttyACM0") as bot:
        bot.set_duty(90, 90)          # keep calling: the firmware watchdog
        ...                           # stops the motors after 300 ms of silence
        print(bot.state.left_metres)

Threading model: one background thread owns the port, parses the telemetry
stream and updates `state`. Callers never touch the socket directly, so the
class is safe to use from a ROS executor, a GUI loop or a plain script.
"""

from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
from dataclasses import dataclass, field

import os
import socket

import serial

# Either a serial device or a network endpoint. The car is reachable both ways
# and the rest of this file does not care which: see _open_transport().
#
#   /dev/ttyACM0            USB CDC, the recovery path
#   tcp://192.168.1.77      W5500 -> switch -> Bullet M2, port 3333 implied
#   192.168.1.77:3333       same thing
#
# CARBOT_PORT overrides the default, so the whole toolchain (motor_tui.py,
# encoder_tune.py, odom.py, ...) moves to the radio link with one export and
# no code changes.
DEFAULT_PORT = os.environ.get("CARBOT_PORT", "/dev/ttyACM0")
BAUD = 115200
NET_PORT = 3333


class _TcpTransport:
    """A socket wearing pyserial's clothes.

    Only the three calls _run() makes are implemented -- write / read / close --
    with pyserial's semantics, notably: read() returns b"" when it merely times
    out with nothing to say, and raises when the link is actually gone. Getting
    that distinction wrong would either spin the reconnect loop forever or hide
    a dead car behind a quiet one.
    """

    def __init__(self, host: str, port: int = NET_PORT, timeout: float = 0.1):
        self._sock = socket.create_connection((host, port), timeout=3.0)
        self._sock.settimeout(timeout)
        # The console is line-oriented and latency matters more than packing:
        # a 'V' command sitting in a Nagle buffer is a wheel that starts late.
        # (The firmware batches the other direction on a 20 ms timer.)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def write(self, data: bytes) -> int:
        self._sock.sendall(data)
        return len(data)

    def read(self, n: int = 4096) -> bytes:
        try:
            chunk = self._sock.recv(n)
        except socket.timeout:
            return b""
        if not chunk:
            raise ConnectionResetError("car closed the console connection")
        return chunk

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


def _open_transport(port: str, baud: int):
    """Return a serial or TCP transport, chosen from the port string."""
    spec = port
    if spec.startswith("tcp://"):
        spec = spec[len("tcp://"):]
    elif spec.startswith("/") or spec.startswith("COM"):
        return serial.Serial(port, baud, timeout=0.1)

    if ":" in spec:
        host, _, portstr = spec.rpartition(":")
        return _TcpTransport(host, int(portstr))
    # A bare host or dotted quad is a network target, not a device node.
    if spec and not spec.startswith("/"):
        return _TcpTransport(spec, NET_PORT)
    return serial.Serial(port, baud, timeout=0.1)

# The firmware stops the motors this long after the last drive command, so a
# continuous controller has to re-send faster than this.
WATCHDOG_MS = 300

# Firmware clamp on a velocity command, m/s. Duty 255 works out to ~0.98 m/s.
VEL_MAX = 0.90

LEFT, RIGHT = 0, 1
SIDE_CODE = {LEFT: "L", RIGHT: "R"}


@dataclass
class WheelState:
    """Latest telemetry for one wheel."""

    counts: int = 0
    metres: float = 0.0
    online: bool = False

    # Closed-loop feedback, from the firmware's FB line. All of these are in
    # the FORWARD-POSITIVE frame already -- the board resolves the mirrored
    # motor mounting itself now, so nothing here needs `direction` applied.
    velocity: float = 0.0       # m/s, filtered, measured
    setpoint: float = 0.0       # m/s, commanded
    track_error_mm: float = 0.0  # reference position minus actual
    duty: int = 0               # what the controller actually applied
    fwd_metres: float = 0.0     # forward-positive travel
    raw_angle: int = -1         # AS5600 absolute shaft angle, 0-4095, -1 unknown
    agc: int = -1               # AS5600 gain, 0-128 on 3V3; ~64 is ideal
    magnitude: int = -1
    status: int = -1            # raw AS5600 STATUS register

    @property
    def magnet_detected(self) -> bool:
        return self.status >= 0 and bool(self.status & 0x20)

    @property
    def magnet_too_weak(self) -> bool:
        return self.status >= 0 and bool(self.status & 0x10)

    @property
    def magnet_too_strong(self) -> bool:
        return self.status >= 0 and bool(self.status & 0x08)

    @property
    def magnet_ok(self) -> bool:
        """Answering, magnet present, and the gain not pegged at either rail."""
        return (self.online and self.magnet_detected
                and not self.magnet_too_weak and not self.magnet_too_strong
                and 0 < self.agc < 128)


# Firmware control modes, as reported in the FB line.
MODE_OFF, MODE_VEL, MODE_HOLD = 0, 1, 2
MODE_NAME = {MODE_OFF: "off", MODE_VEL: "velocity", MODE_HOLD: "hold"}


@dataclass
class BaseState:
    left: WheelState = field(default_factory=WheelState)
    right: WheelState = field(default_factory=WheelState)
    connected: bool = False
    last_rx: float = 0.0
    error: str = ""

    # Pose, integrated on the board from both wheels. Yaw is only as good as
    # the track width, which is a placeholder until somebody measures it.
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0          # radians
    track_width: float = 0.0

    skew_mm: float = 0.0        # left travel minus right travel
    ref_scale: float = 1.0      # 1.0 = loop keeping up; < 1 = governed
    mode: int = MODE_OFF
    last_fb: float = 0.0

    @property
    def left_metres(self) -> float:
        return self.left.metres

    @property
    def right_metres(self) -> float:
        return self.right.metres

    @property
    def mode_name(self) -> str:
        return MODE_NAME.get(self.mode, "?")

    @property
    def closed_loop(self) -> bool:
        return self.mode != MODE_OFF

    @property
    def feedback_fresh(self) -> bool:
        """True while the FB stream is arriving -- i.e. the closed-loop build."""
        return (time.time() - self.last_fb) < 0.5

    @property
    def distance(self) -> float:
        """Straight-line travel of the body centre, metres, forward-positive."""
        return 0.5 * (self.left.fwd_metres + self.right.fwd_metres)

    @property
    def governed(self) -> bool:
        """A wheel is being held back and both references have been slowed."""
        return self.ref_scale < 0.999

    @property
    def fresh(self) -> bool:
        return self.connected and (time.time() - self.last_rx) < 1.0


@dataclass
class MoveResult:
    """Outcome of a closed-loop rotate."""

    side: str
    commanded_deg: float
    actual_deg: float
    error_deg: float
    reason: str                 # "reached", "timeout", "aborted", "preempted"

    @property
    def ok(self) -> bool:
        return self.reason == "reached"


def load_calibration(path: str | None = None) -> dict:
    """Load calibration.json, falling back to the values measured on this car."""
    if path is None:
        path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "calibration.json")
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return {
            "wheel_diameter_m": 0.120,
            "counts_per_motor_rev": 4096,
            "combined": {"gear_ratio": 18.518, "counts_per_degree": 210.69,
                         "counts_per_metre": 201196.0},
            "wheels": {"LEFT": {"counts_per_degree": 210.76, "direction": -1},
                       "RIGHT": {"counts_per_degree": 209.88, "direction": 1}},
            "duty_to_counts_per_sec": {"slope": 959.0, "intercept": -47316.0,
                                       "valid_duty_min": 58},
            "encoder_sign": -1,
        }


class WheelDriver:
    """Owns the serial link and speaks the firmware's line protocol."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = BAUD,
                 calibration: dict | None = None, auto_open: bool = True):
        self.port = port
        self.baud = baud
        self.cal = calibration or load_calibration()
        self.state = BaseState()

        # The two motors are mounted mirror-imaged, so wiring both the same
        # runs one of them backwards. `direction` flips that side's commands
        # AND its counts, so everything above this class -- odometry, the
        # motor node, the tools -- sees one forward, on both wheels.
        self.direction = {
            LEFT: self._direction(LEFT),
            RIGHT: self._direction(RIGHT),
        }

        self._ser: serial.Serial | None = None
        self._tx: queue.Queue[bytes] = queue.Queue()
        self._moves: queue.Queue[MoveResult] = queue.Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # Odometry needs to know what was asked for, to tell wanted motion from
        # unwanted. Every command that can move a wheel stamps these.
        self.last_command: tuple[float, int, int] | None = None   # (t, left, right)
        self._command_until = 0.0

        if auto_open:
            self.open()

    def _direction(self, side: int) -> int:
        name = "LEFT" if side == LEFT else "RIGHT"
        d = self.cal.get("wheels", {}).get(name, {}).get("direction", 1)
        return -1 if int(d or 1) < 0 else 1

    def encoder_sign(self, side: int) -> int:
        """
        Multiplier that turns this side's raw counts into forward-positive.

        Two things folded into one number: the firmware's counters run
        negative when driven forward (SETUP.md 7), and a mirror-wired motor
        turns its own encoder the other way as well.
        """
        return int(self.cal.get("encoder_sign", 1)) * self.direction[side]

    # ------------------------------------------------------------ lifecycle

    def open(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        try:
            self.stop()
            time.sleep(0.15)
        except Exception:
            pass
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def __enter__(self) -> "WheelDriver":
        self.wait_for_telemetry(timeout=6.0)
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def wait_for_telemetry(self, timeout: float = 6.0) -> bool:
        """Block until the board is actually streaming. Returns success."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.state.fresh:
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------- commands

    def _send(self, line: str) -> None:
        self._tx.put((line + "\n").encode())

    def set_duty(self, left: int, right: int) -> None:
        """
        Open-loop drive, -255..255 per side.

        The firmware stops after WATCHDOG_MS without a fresh command, so a
        continuous controller must call this faster than about 3 Hz. That is
        deliberate: it means a crashed controller cannot run the car away.
        """
        left = max(-255, min(255, int(left)))
        right = max(-255, min(255, int(right)))
        # Track what was asked for in the forward-positive frame; send it in
        # the board's frame.
        self._note_command(left, right, WATCHDOG_MS / 1000.0)
        self._send(f"D {left * self.direction[LEFT]} {right * self.direction[RIGHT]}")

    def set_velocity(self, left_mps: float, right_mps: float) -> None:
        """
        Closed-loop wheel velocity, m/s, forward-positive on both sides.

        This is the one to use for driving. The firmware servos each wheel to
        an integrated position reference rather than just matching speed, so
        commanding the same value on both sides holds them in lockstep -- any
        distance difference between the wheels is an error the loop actively
        removes, instead of drifting the way a velocity-only loop does.

        Same watchdog as set_duty: re-send faster than WATCHDOG_MS or the
        board stops on its own.

        No `direction` is applied here. The board resolves the mirrored motor
        mounting itself, so both sides are already forward-positive.
        """
        left_mps = max(-VEL_MAX, min(VEL_MAX, float(left_mps)))
        right_mps = max(-VEL_MAX, min(VEL_MAX, float(right_mps)))
        self._note_command(int(left_mps * 255), int(right_mps * 255),
                           WATCHDOG_MS / 1000.0)
        self._send(f"V {left_mps:.4f} {right_mps:.4f}")

    def drive(self, linear_mps: float, angular_rps: float = 0.0) -> None:
        """Unicycle command: forward m/s and yaw rad/s, mixed on the board."""
        self._note_command(int(linear_mps * 255), int(linear_mps * 255),
                           WATCHDOG_MS / 1000.0)
        self._send(f"W {linear_mps:.4f} {angular_rps:.4f}")

    def hold_position(self) -> None:
        """
        Freeze both wheels where they are and actively resist being moved.

        Exempt from the command watchdog -- it is a deliberate "stay put", so
        it survives without anyone re-sending. It does expire on its own after
        about two minutes so a forgotten hold cannot cook a stalled motor.
        """
        self._note_command(0, 0, 0.0)
        self._send("H")

    def release(self) -> None:
        """Disengage the closed loop and let the wheels coast free."""
        self._note_command(0, 0, 0.0)
        self._send("E")

    def set_gains(self, kp_vel=None, ki_vel=None, kp_pos=None,
                  soft_m=None, hard_m=None) -> None:
        """Retune the cascade live. Omitted values keep their current setting."""
        cur = [kp_vel, ki_vel, kp_pos, soft_m, hard_m]
        if all(v is None for v in cur):
            self._send("K")
            return
        d = [250.0, 900.0, 8.0, 0.020, 0.060]
        v = [d[i] if cur[i] is None else float(cur[i]) for i in range(5)]
        self._send("K {:.3f} {:.3f} {:.3f} {:.4f} {:.4f}".format(*v))

    def set_gear_ratio(self, ratio: float, wheel_mm: float | None = None) -> None:
        """
        Set the motor-to-wheel gear ratio the board derives everything from.

        The encoder is before the gearbox, so this is the one constant it can
        never measure for itself -- see gear_check.py. Changing it rescales
        counts-per-metre, counts-per-degree and the feedforward together.
        """
        if wheel_mm is None:
            self._send(f"N {float(ratio):.4f}")
        else:
            self._send(f"N {float(ratio):.4f} {float(wheel_mm):.2f}")

    def set_track_width(self, metres: float) -> None:
        """Wheel centre-to-centre. Needed for yaw and for drive()'s angular term."""
        self._send(f"B {float(metres):.4f}")

    def stop(self) -> None:
        self._note_command(0, 0, 0.0)
        self._send("X")

    def zero(self) -> None:
        """Reset both encoder counters to zero."""
        self._send("Z")

    def _note_command(self, left: int, right: int, hold_s: float) -> None:
        """Record that motion was asked for, and for how long it stays valid."""
        now = time.time()
        self.last_command = (now, left, right)
        if left or right:
            self._command_until = max(self._command_until, now + hold_s)
        else:
            self._command_until = now

    def command_active(self) -> bool:
        """True while a command that can move a wheel is still in force."""
        return time.time() < self._command_until

    def idle_seconds(self) -> float:
        """How long the base has been under no drive command at all."""
        return max(0.0, time.time() - self._command_until)

    def rotate(self, side: int, degrees: float, duty: int = 90,
               wait: bool = True, timeout: float = 40.0) -> MoveResult | None:
        """
        Closed-loop rotate one wheel by `degrees` of WHEEL rotation.

        The firmware closes the loop and tapers off near the target, so this
        does not need the caller to hold a control loop open.
        """
        while not self._moves.empty():          # discard any stale result
            self._moves.get_nowait()
        self._note_command(int(duty) if side == LEFT else 0,
                           int(duty) if side == RIGHT else 0, timeout)
        self._send(f"G {SIDE_CODE[side]} "
                   f"{degrees * self.direction[side]:.2f} {int(duty)}")
        if not wait:
            return None
        try:
            return self._moves.get(timeout=timeout)
        except queue.Empty:
            return MoveResult(SIDE_CODE[side], degrees, 0.0, 0.0, "timeout")

    def spin_test(self) -> None:
        """Fire the firmware's built-in forward/reverse diagnostic."""
        self._note_command(110, 110, 6.0)
        self._send("S")

    # -------------------------------------------------------------- helpers

    def counts_per_degree(self, side: int) -> float:
        name = "LEFT" if side == LEFT else "RIGHT"
        w = self.cal.get("wheels", {}).get(name)
        if w:
            return w["counts_per_degree"]
        return self.cal["combined"]["counts_per_degree"]

    def counts_per_metre(self, side: int | None = None) -> float:
        if side is not None:
            name = "LEFT" if side == LEFT else "RIGHT"
            w = self.cal.get("wheels", {}).get(name)
            if w and "counts_per_metre" in w:
                return w["counts_per_metre"]
        return self.cal["combined"]["counts_per_metre"]

    # ---------------------------------------------------------------- thread

    def _run(self) -> None:
        buf = b""
        while not self._stop.is_set():
            if self._ser is None:
                try:
                    self._ser = _open_transport(self.port, self.baud)
                    self._ser.write(b"X\n")     # never inherit a moving motor
                    self.state.connected = True
                    self.state.error = ""
                    buf = b""
                except Exception as e:
                    self.state.connected = False
                    self.state.error = str(e)
                    time.sleep(0.5)
                    continue
            try:
                while not self._tx.empty():
                    self._ser.write(self._tx.get_nowait())
                chunk = self._ser.read(4096)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        self._parse(raw.decode("utf-8", "replace").strip())
            except Exception as e:
                self.state.connected = False
                self.state.error = str(e)
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(0.4)

    def _parse(self, line: str) -> None:
        if line.startswith("ODO,"):
            p = line.split(",")
            if len(p) != 7:
                return
            try:
                self.state.left.metres = float(p[1])
                self.state.right.metres = float(p[2])
                self.state.left.online = p[3] == "1"
                self.state.right.online = p[4] == "1"
                self.state.left.counts = int(p[5])
                self.state.right.counts = int(p[6])
                self.state.last_rx = time.time()
            except ValueError:
                pass
        elif line.startswith("FB,"):
            self._parse_fb(line)
        elif line.startswith("T,"):
            p = line.split(",")
            if len(p) != 9:
                return
            try:
                v = [int(x) for x in p[1:]]
            except ValueError:
                return
            self.state.left.raw_angle = v[0]
            self.state.right.raw_angle = v[4]
            self.state.left.agc, self.state.left.magnitude = v[1], v[2]
            self.state.left.status = v[3]
            self.state.right.agc, self.state.right.magnitude = v[5], v[6]
            self.state.right.status = v[7]
        elif line.startswith("# GODONE,"):
            p = line[len("# GODONE,"):].split(",")
            try:
                # The board answers in its own frame; hand it back in ours.
                d = self.direction[LEFT if p[0].upper().startswith("L")
                                   else RIGHT]
                self._command_until = time.time()
                self._moves.put(MoveResult(p[0], float(p[1]) * d,
                                           float(p[2]) * d,
                                           float(p[3]) * d, p[4]))
            except (ValueError, IndexError):
                pass

    def _parse_fb(self, line: str) -> None:
        """
        FB,<ms>,<Lm>,<Rm>,<Lmps>,<Rmps>,<LspMps>,<RspMps>,<LerrMm>,<RerrMm>,
           <Lduty>,<Rduty>,<skewMm>,<x>,<y>,<thDeg>,<refScale>,<mode>,<track>

        Everything already forward-positive and in SI units, straight from the
        controller. This is the line to build odometry on; ODO is the older
        raw-board-frame one kept for the existing tools.
        """
        p = line.split(",")
        if len(p) != 19:
            return
        try:
            st = self.state
            st.left.fwd_metres = float(p[2])
            st.right.fwd_metres = float(p[3])
            st.left.velocity = float(p[4])
            st.right.velocity = float(p[5])
            st.left.setpoint = float(p[6])
            st.right.setpoint = float(p[7])
            st.left.track_error_mm = float(p[8])
            st.right.track_error_mm = float(p[9])
            st.left.duty = int(p[10])
            st.right.duty = int(p[11])
            st.skew_mm = float(p[12])
            st.x = float(p[13])
            st.y = float(p[14])
            st.theta = math.radians(float(p[15]))
            st.ref_scale = float(p[16])
            st.mode = int(p[17])
            st.track_width = float(p[18])
            st.last_fb = time.time()
            st.last_rx = st.last_fb
        except ValueError:
            pass
