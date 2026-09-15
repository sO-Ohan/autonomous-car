"""
Wheel odometry: absolute wheel angle, travelled distance, and detection of
rotation nobody asked for.

This is the layer between the raw encoder telemetry and a SLAM front end. It
answers three questions continuously, per wheel and for the body:

  1. Where is each wheel pointing?      -> absolute angle, 0-360 deg
  2. How far has it gone?               -> signed net distance and total path
  3. Did anything move that shouldn't?  -> UNINTENDED motion events

Everything here is forward-positive. The firmware's counters run negative when
driven forward (SETUP.md 7), so `encoder_sign` from calibration.json is applied
once, on the way in, and never thought about again.

What "absolute angle" means, precisely
--------------------------------------
The AS5600 is absolute over one MOTOR revolution (4096 counts). The gearbox is
18.5:1, so one wheel revolution is ~18.5 motor revolutions: the sensor alone
cannot tell you which of those 18.5 the wheel is in. Therefore:

  * `motor_angle_deg` is TRULY absolute -- it survives a power cycle, because
    the AS5600 reports shaft position, not motion.
  * `wheel_angle_deg` is absolute WITH RESPECT TO THE ORIGIN, i.e. wherever the
    wheel was when the counters were last zeroed. It is exact and drift-free
    across any number of turns, but its zero is a convention, not a physical
    landmark.

That distinction matters for SLAM: wheel angle is a perfectly good relative
measurement, it just is not a global one.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field, asdict

from .driver import LEFT, RIGHT, WheelDriver, load_calibration
from .kinematics import DifferentialOdometry, Pose

SIDE_NAME = {LEFT: "LEFT", RIGHT: "RIGHT"}

# A stationary AS5600 jitters by a few counts. Anything under this is noise,
# not motion -- the same threshold encoder_gui.py uses to call a wheel moving.
NOISE_COUNTS = 40


@dataclass
class WheelOdom:
    """Everything known about one wheel at one instant. Forward positive."""

    side: str = ""
    counts: int = 0                 # unwrapped, sign-corrected motor counts
    motor_angle_deg: float = 0.0    # 0-360, absolute shaft angle from AS5600
    wheel_angle_deg: float = 0.0    # 0-360, absolute since origin (see module docstring)
    wheel_angle_total_deg: float = 0.0   # unwrapped, grows without bound
    revolutions: float = 0.0        # signed wheel revolutions since origin
    distance_m: float = 0.0         # signed net travel since origin
    path_m: float = 0.0             # total travel, ignoring direction
    speed_mps: float = 0.0
    omega_dps: float = 0.0          # wheel angular rate, deg/s
    moving: bool = False
    healthy: bool = False           # encoder answering and magnet usable


@dataclass
class MotionEvent:
    """A wheel or the body moved without being commanded to."""

    t: float
    kind: str                       # see below
    side: str                       # "LEFT", "RIGHT" or "BODY"
    magnitude_deg: float
    distance_m: float
    message: str

    def as_dict(self) -> dict:
        return asdict(self)


# Event kinds:
#   unintended_wheel_rotation : wheel turned while the base was commanded idle
#                               (pushed, rolled down a slope, knocked)
#   unintended_yaw            : body heading changed while driving straight
#                               (one wheel slipping, dragging, uneven load)
#   wheel_disagreement        : wheels commanded together, travelled apart


@dataclass
class OdometryFrame:
    """One complete odometry sample: the thing a SLAM front end consumes."""

    t: float = 0.0
    left: WheelOdom = field(default_factory=WheelOdom)
    right: WheelOdom = field(default_factory=WheelOdom)

    # Body pose. Only meaningful when a track width was supplied.
    x: float = 0.0
    y: float = 0.0
    theta_deg: float = 0.0
    have_pose: bool = False

    distance_m: float = 0.0         # signed net body travel
    path_m: float = 0.0             # total body path length
    linear_mps: float = 0.0
    angular_dps: float = 0.0

    commanded: bool = False         # was the base under a drive command?
    trustworthy: bool = True        # False once unintended motion is seen
    events: list = field(default_factory=list)
    fresh: bool = False             # telemetry actually arriving

    def as_dict(self) -> dict:
        d = asdict(self)
        d["events"] = [e.as_dict() if isinstance(e, MotionEvent) else e
                       for e in self.events]
        return d

    def as_json(self) -> str:
        return json.dumps(self.as_dict(), separators=(",", ":"))


class WheelOdometryNode:
    """
    Turns `WheelDriver` telemetry into `OdometryFrame`s.

    Standalone by design -- no ROS import anywhere -- so it runs today from a
    plain script and drops into a ROS 2 node later as a member object whose
    frames map field-for-field onto nav_msgs/Odometry plus a diagnostics topic.

        node = WheelOdometryNode(driver, track_width_m=0.30)
        node.on_frame = lambda f: print(f.as_json())
        node.spin(rate_hz=20)

    Detecting unintended rotation
    -----------------------------
    "Unintended" is defined against what was commanded, so the node has to know
    that. `WheelDriver` timestamps every drive command it sends; a wheel that
    moves more than `idle_tolerance_deg` while no command has been in force for
    `idle_grace_s` raises an event. That catches a hand push, a roll on a
    slope, and a wheel still coasting long after the motors were cut.

    Body yaw is checked separately: when both wheels are commanded to the same
    speed, any heading change is by definition something going wrong -- a
    slipping tyre, a dragging brake, a carpet edge. That check needs the track
    width; without it the per-wheel disagreement check still runs, in metres.
    """

    def __init__(self, driver: WheelDriver, track_width_m: float | None = None,
                 calibration: dict | None = None,
                 idle_tolerance_deg: float = 1.0,
                 idle_grace_s: float = 0.6,
                 yaw_tolerance_dps: float = 8.0,
                 disagreement_m: float = 0.01,
                 velocity_window_s: float = 0.15,
                 yaw_window_s: float = 0.25):
        self.driver = driver
        self.cal = calibration or driver.cal or load_calibration()
        # Per side: the firmware's counters run negative when driven forward,
        # and a mirror-wired motor turns its encoder the other way too. The
        # driver folds both into one number.
        self.sign = {side: self._encoder_sign(side) for side in (LEFT, RIGHT)}

        cpr = self.cal["combined"]["counts_per_wheel_rev"] \
            if "counts_per_wheel_rev" in self.cal.get("combined", {}) \
            else self.cal["counts_per_motor_rev"] * self.cal["combined"]["gear_ratio"]
        self.counts_per_wheel_rev = {
            LEFT: self._wheel_cal(LEFT, "counts_per_wheel_rev", cpr),
            RIGHT: self._wheel_cal(RIGHT, "counts_per_wheel_rev", cpr),
        }
        self.counts_per_metre = {
            LEFT: driver.counts_per_metre(LEFT),
            RIGHT: driver.counts_per_metre(RIGHT),
        }
        self.counts_per_motor_rev = self.cal.get("counts_per_motor_rev", 4096)

        self.track_width = track_width_m or self.cal.get("track_width_m")
        self._diff = (DifferentialOdometry(self.track_width)
                      if self.track_width else None)

        self.idle_tolerance_deg = idle_tolerance_deg
        self.idle_grace_s = idle_grace_s
        self.yaw_tolerance_dps = yaw_tolerance_dps
        self.disagreement_m = disagreement_m
        # Rates are measured over a window, never over one sample pair. The
        # firmware streams ODO at 10 Hz; sampling faster than that would
        # otherwise divide a real count step by a near-zero dt and report a
        # wheel doing thousands of deg/s.
        self.velocity_window_s = velocity_window_s
        self.yaw_window_s = yaw_window_s

        self.frame = OdometryFrame()
        self.events: list[MotionEvent] = []
        self.on_frame = None            # optional callback(OdometryFrame)
        self.on_event = None            # optional callback(MotionEvent)

        self._origin = {LEFT: None, RIGHT: None}     # counts at zero
        self._prev = {LEFT: None, RIGHT: None}       # counts last sample
        self._prev_t = None
        self._path = {LEFT: 0.0, RIGHT: 0.0}
        self._idle_ref = {LEFT: None, RIGHT: None}   # counts when idling began
        self._idle_flagged = {LEFT: False, RIGHT: False}
        self._body_path = 0.0
        self._trustworthy = True
        self._stop = False
        self._vel_acc = {LEFT: [0, 0.0], RIGHT: [0, 0.0]}   # [counts, seconds]
        self._speed = {LEFT: 0.0, RIGHT: 0.0}               # m/s, windowed
        self._yaw_acc = [0.0, 0.0, 0.0]                     # [dl_m, dr_m, s]

    def _encoder_sign(self, side: int) -> int:
        fn = getattr(self.driver, "encoder_sign", None)
        if callable(fn):
            return int(fn(side))
        return int(self.cal.get("encoder_sign", 1))

    def _wheel_cal(self, side: int, key: str, default: float) -> float:
        w = self.cal.get("wheels", {}).get(SIDE_NAME[side], {})
        return float(w.get(key, default))

    # -------------------------------------------------------------- control

    def reset(self) -> None:
        """Re-origin here: angles, distances and pose all restart from zero."""
        self._origin = {LEFT: None, RIGHT: None}
        self._prev = {LEFT: None, RIGHT: None}
        self._prev_t = None
        self._path = {LEFT: 0.0, RIGHT: 0.0}
        self._body_path = 0.0
        self._idle_ref = {LEFT: None, RIGHT: None}
        self._idle_flagged = {LEFT: False, RIGHT: False}
        self._trustworthy = True
        self._vel_acc = {LEFT: [0, 0.0], RIGHT: [0, 0.0]}
        self._speed = {LEFT: 0.0, RIGHT: 0.0}
        self._yaw_acc = [0.0, 0.0, 0.0]
        self.events.clear()
        if self._diff:
            self._diff.reset()

    def origin_counts(self, side: int) -> int | None:
        """
        The counts this side's angles and distances are measured from.

        It changes when the counters are zeroed or absorbed after a jump, and
        every position measured against the old origin becomes meaningless at
        that instant. Anything holding a position target has to notice, which
        is why this is public rather than private.
        """
        return self._origin[side]

    def zero_encoders(self) -> None:
        """Zero the firmware counters as well, so counts and origin agree."""
        self.driver.zero()
        time.sleep(0.2)
        self.reset()

    # --------------------------------------------------------------- update

    def update(self, now: float | None = None) -> OdometryFrame:
        """Sample the driver once and produce a frame. Safe to call at any rate."""
        now = now or time.time()
        st = self.driver.state
        f = OdometryFrame(t=now, fresh=st.fresh)
        f.commanded = self.driver.command_active()

        raw = {LEFT: self.sign[LEFT] * st.left.counts,
               RIGHT: self.sign[RIGHT] * st.right.counts}

        # First sample, or a counter reset under us: adopt it as the origin
        # rather than integrating a meaningless jump into the pose.
        for side in (LEFT, RIGHT):
            if self._origin[side] is None or self._is_jump(side, raw[side], now):
                self._origin[side] = raw[side]
                self._prev[side] = raw[side]
                self._idle_ref[side] = raw[side]
                if self._diff:
                    self._diff.reset(self._diff.pose)

        dt = (now - self._prev_t) if self._prev_t else 0.0
        self._prev_t = now

        wheels = {}
        step = {}
        for side in (LEFT, RIGHT):
            step[side] = ((raw[side] - self._prev[side])
                          / self.counts_per_metre[side])
            wheels[side] = self._wheel_frame(side, raw[side], dt, st)
        step_l, step_r = step[LEFT], step[RIGHT]
        f.left, f.right = wheels[LEFT], wheels[RIGHT]

        # Body
        dl = f.left.distance_m
        dr = f.right.distance_m
        f.distance_m = (dl + dr) / 2.0
        self._body_path = (f.left.path_m + f.right.path_m) / 2.0
        f.path_m = self._body_path
        f.linear_mps = (f.left.speed_mps + f.right.speed_mps) / 2.0

        if self._diff:
            pose: Pose = self._diff.update(dl, dr)
            f.x, f.y = pose.x, pose.y
            f.theta_deg = math.degrees(pose.theta)
            f.have_pose = True
            f.angular_dps = math.degrees(
                (f.right.speed_mps - f.left.speed_mps) / self.track_width)

        # Unintended motion
        new_events = (self._check_idle(f, now)
                      + self._check_yaw(f, now, step_l, step_r, dt))
        for ev in new_events:
            self.events.append(ev)
            self._trustworthy = False
            if self.on_event:
                self.on_event(ev)
        f.events = new_events
        f.trustworthy = self._trustworthy

        self.frame = f
        if self.on_frame:
            self.on_frame(f)
        return f

    def _wheel_frame(self, side: int, counts: int, dt: float, st) -> WheelOdom:
        cpr = self.counts_per_wheel_rev[side]
        cpm = self.counts_per_metre[side]
        rel = counts - self._origin[side]
        d_counts = counts - self._prev[side]
        self._prev[side] = counts

        if abs(d_counts) > NOISE_COUNTS:
            self._path[side] += abs(d_counts) / cpm

        acc = self._vel_acc[side]
        acc[0] += d_counts
        acc[1] += dt
        if acc[1] >= self.velocity_window_s:
            self._speed[side] = acc[0] / cpm / acc[1]
            acc[0], acc[1] = 0, 0.0
        speed = self._speed[side]

        ws = st.left if side == LEFT else st.right
        # Prefer the AS5600's own reading for the motor shaft: it is the one
        # number here that is absolute in the physical sense.
        if ws.raw_angle >= 0:
            motor_deg = ws.raw_angle * 360.0 / self.counts_per_motor_rev
            if self.sign[side] < 0:
                motor_deg = (360.0 - motor_deg) % 360.0
        else:
            motor_deg = (counts % self.counts_per_motor_rev) * 360.0 / self.counts_per_motor_rev

        total_deg = rel * 360.0 / cpr
        return WheelOdom(
            side=SIDE_NAME[side],
            counts=rel,
            motor_angle_deg=motor_deg % 360.0,
            wheel_angle_deg=total_deg % 360.0,
            wheel_angle_total_deg=total_deg,
            revolutions=rel / cpr,
            distance_m=rel / cpm,
            path_m=self._path[side],
            speed_mps=speed,
            omega_dps=speed * cpm * 360.0 / cpr,
            moving=abs(d_counts) > NOISE_COUNTS,
            healthy=ws.magnet_ok,
        )

    def _is_jump(self, side: int, counts: int, now: float) -> bool:
        """A step no real wheel could have taken -- a `Z`, or a lost link."""
        prev = self._prev[side]
        if prev is None:
            return False
        # Top measured speed is ~58k counts/s; 200k in one sample is not motion.
        return abs(counts - prev) > 200_000

    # ------------------------------------------------- unintended motion

    def _check_idle(self, f: OdometryFrame, now: float) -> list[MotionEvent]:
        """Wheel movement while the base is not under command."""
        out = []
        idle_for = self.driver.idle_seconds()
        for side, wo in ((LEFT, f.left), (RIGHT, f.right)):
            if f.commanded or idle_for < self.idle_grace_s:
                self._idle_ref[side] = self._prev[side]
                self._idle_flagged[side] = False
                continue
            if self._idle_ref[side] is None:
                self._idle_ref[side] = self._prev[side]
                continue
            drift = self._prev[side] - self._idle_ref[side]
            deg = abs(drift) * 360.0 / self.counts_per_wheel_rev[side]
            if deg > self.idle_tolerance_deg and not self._idle_flagged[side]:
                self._idle_flagged[side] = True
                out.append(MotionEvent(
                    t=now, kind="unintended_wheel_rotation",
                    side=SIDE_NAME[side], magnitude_deg=math.copysign(deg, drift),
                    distance_m=drift / self.counts_per_metre[side],
                    message=(f"{SIDE_NAME[side]} wheel turned {deg:.2f} deg with "
                             f"no drive command for {idle_for:.1f} s")))
        return out

    def _check_yaw(self, f: OdometryFrame, now: float, dl: float, dr: float,
                   dt: float) -> list[MotionEvent]:
        """Heading change while the two sides were commanded the same."""
        cmd = self.driver.last_command
        left_duty, right_duty = (cmd[1], cmd[2]) if cmd else (0, 0)
        # A turn, or an idle base, is not a straight-line fault: drop the window.
        if not cmd or not f.commanded or left_duty != right_duty:
            self._yaw_acc = [0.0, 0.0, 0.0]
            return []

        self._yaw_acc[0] += dl
        self._yaw_acc[1] += dr
        self._yaw_acc[2] += dt
        if self._yaw_acc[2] < self.yaw_window_s:
            return []
        wdl, wdr, wdt = self._yaw_acc
        self._yaw_acc = [0.0, 0.0, 0.0]

        diff_m = (wdr - wdl) / wdt          # metres/second of differential
        out = []
        if self.track_width:
            yaw_dps = math.degrees(diff_m / self.track_width)
            if abs(yaw_dps) > self.yaw_tolerance_dps:
                out.append(MotionEvent(
                    t=now, kind="unintended_yaw", side="BODY",
                    magnitude_deg=yaw_dps, distance_m=0.0,
                    message=(f"heading turning at {yaw_dps:+.1f} deg/s while "
                             f"driving straight (duty {left_duty})")))
        elif abs(diff_m) > self.disagreement_m:
            out.append(MotionEvent(
                t=now, kind="wheel_disagreement", side="BODY",
                magnitude_deg=0.0, distance_m=diff_m,
                message=(f"wheels differ by {diff_m:+.3f} m/s while commanded "
                         f"together; set track_width to convert this to yaw")))
        return out

    # ----------------------------------------------------------------- spin

    def spin(self, rate_hz: float = 20.0, duration: float | None = None) -> None:
        """Sample forever (or for `duration` seconds), calling `on_frame`."""
        period = 1.0 / rate_hz
        end = (time.time() + duration) if duration else None
        self._stop = False
        while not self._stop and (end is None or time.time() < end):
            t0 = time.time()
            self.update(t0)
            time.sleep(max(0.0, period - (time.time() - t0)))

    def shutdown(self) -> None:
        self._stop = True
