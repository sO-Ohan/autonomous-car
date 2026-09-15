"""
Motor supervision: per-motor properties, position hold, and the faults that
have to stop a wheel.

`WheelDriver` moves a motor and `WheelOdometryNode` says what moved. Neither
decides what a motor is *allowed* to do, or what to do about motion nobody
asked for -- odometry only reports it. This node closes that gap.

Two things live here:

1. **Properties.** Every motor carries a `MotorConfig`, split into CRITICAL
   properties (get these wrong and the bot damages itself, runs away, or holds
   position blind) and OPTIONAL ones (tuning and comfort). The split is not
   decoration: `MotorNode` refuses to run with an invalid critical property,
   while an optional one is free to be anything in range. `PROPERTIES` carries
   the range, unit and reason for each, which is what the TUI edits.

2. **Position hold.** With `hold_position` on, a motor latches where it is once
   it goes idle, and comes back to that angle if anything moves it -- a hand,
   a slope, a knock. That is the difference between "the odometry noticed you
   pushed the wheel" and "the wheel put itself back".

How the hold actually returns the wheel
---------------------------------------
Not with a Python P-loop. The firmware's `G` rotate already drives, brakes,
settles, and trims to +-60 counts (~0.3 deg of wheel) -- far better than
anything reachable over a 10 Hz telemetry link with a duty deadband at 58. So
a recovery is one `G` for the measured error, and the loop here only decides
*when* to fire it:

  * wait `hold_settle_s` after the disturbance stops, so the wheel is not
    fought while a hand is still turning it, and coast has died out;
  * cap each attempt at `hold_max_step_deg`, so a wheel spun a long way back
    walks home instead of lurching;
  * give up after `hold_max_corrections` and raise `hold_failed` rather than
    hunting forever against something that is physically blocking the wheel.

Only one `G` can run at a time (the firmware has a single rotate state
machine), so every move -- recovery or commanded -- goes through one worker
queue. Nothing else may send `D` while one is in flight: `D` preempts `G`.

Sign convention is the same as everywhere else in this package: forward is
positive, in degrees of WHEEL rotation.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import asdict, dataclass, field

from .driver import LEFT, RIGHT, MoveResult, WheelDriver
from .odometry import SIDE_NAME, WheelOdometryNode

SIDES = (LEFT, RIGHT)
SIDE_OF_NAME = {"LEFT": LEFT, "RIGHT": RIGHT}

CRITICAL = "critical"
OPTIONAL = "optional"

# The firmware calls a rotate "capped" when it ran out of trim attempts. It
# still stopped where the encoder says, so it counts as a landed move.
MOVE_OK = ("reached", "capped")

# Motion bookkeeping. A wheel that has changed angle by more than MOVING_DEG
# counts as moving for MOVING_HOLD_S afterwards -- long enough to bridge the
# 100 ms gap between ODO frames, short enough to feel immediate.
MOVING_DEG = 0.05
MOVING_HOLD_S = 0.25
# After a move of our own, ignore the odometry catching up for this long. Must
# outlast MOVING_HOLD_S so the wheel does not look like it is still turning.
MOVE_QUIET_S = 0.3

# The firmware abandons a rotate after this long (GO_TIMEOUT_MS in main.cpp)
# and reports "timeout" wherever it got to. At duty 90 a wheel turns about
# 185 deg/s, so anything past ~5500 deg cannot finish inside it -- which is
# why a big move is split rather than sent whole.
GO_TIMEOUT_S = 30.0
GO_TIMEOUT_USE = 0.6            # only spend this much of the budget per chunk
GO_TAPER_COST = 1.2             # the firmware eases off over the last fifth


# --------------------------------------------------------------- properties

@dataclass(frozen=True)
class Property:
    """One tunable, with the range and the reason it matters."""

    name: str
    kind: str                   # CRITICAL or OPTIONAL
    type: type
    default: object
    lo: float | None = None
    hi: float | None = None
    step: float = 1.0
    unit: str = ""
    doc: str = ""


PROPERTIES: tuple[Property, ...] = (
    # ---------------------------------------------------------- critical
    Property("enabled", CRITICAL, bool, True, doc=(
        "Master switch. A disabled motor refuses every drive, every rotate "
        "and every hold correction.")),
    Property("hold_position", CRITICAL, bool, True, doc=(
        "Station keeping. Latch the angle when the motor goes idle and drive "
        "back to it if anything else moves the wheel.")),
    Property("hold_tolerance_deg", CRITICAL, float, 2.0, 0.3, 90.0, 0.5, "deg",
             "Deadband. Smaller than the firmware's own ~0.3 deg trim window "
             "and the hold will hunt forever."),
    Property("hold_settle_s", CRITICAL, float, 0.4, 0.05, 5.0, 0.05, "s",
             "How long the disturbance must be over before correcting. This "
             "is what stops the motor fighting a hand that is still turning "
             "it."),
    Property("hold_duty", CRITICAL, int, 90, 58, 255, 5, "duty",
             "Duty used for a recovery move. Below the 58 deadband nothing "
             "moves, and a standstill needs more torque to break away than a "
             "wheel already turning."),
    Property("hold_max_corrections", CRITICAL, int, 8, 1, 60, 1, "tries",
             "Consecutive attempts before giving up and raising hold_failed. "
             "Resets as soon as the wheel is back inside tolerance."),
    Property("min_duty", CRITICAL, int, 58, 0, 255, 1, "duty",
             "Measured deadband floor. A commanded duty between 1 and this "
             "only heats the motor, so it is treated as no motion wanted."),
    Property("max_duty", CRITICAL, int, 200, 58, 255, 5, "duty",
             "Hard ceiling on any duty this motor is given, from any source."),
    Property("stall_timeout_s", CRITICAL, float, 1.0, 0.0, 10.0, 0.1, "s",
             "Commanded above the deadband but the encoder is not moving for "
             "this long -> stall fault. 0 disables the check."),
    Property("estop_on_stall", CRITICAL, bool, True, doc=(
        "A stalled motor e-stops the whole base rather than only itself. On a "
        "two-wheel base, driving one side into a locked wheel is a spin.")),
    Property("check_direction", CRITICAL, bool, True, doc=(
        "Fault if the wheel turns the opposite way to the duty it was given. "
        "A mirror-wired motor makes the odometry, the pose and the hold all "
        "wrong in the same breath, and it is invisible until something is "
        "measured against it -- so it is checked, not assumed.")),
    Property("require_encoder", CRITICAL, bool, True, doc=(
        "Fault when the encoder stops answering. Position hold with no "
        "feedback is an open-loop motor with a plan, which is worse than a "
        "stopped one.")),

    # ---------------------------------------------------------- optional
    Property("hold_rearm_s", OPTIONAL, float, 0.6, 0.0, 10.0, 0.1, "s",
             "Idle time before the hold target is latched, so coast-down is "
             "not recorded as the target."),
    Property("hold_retry_s", OPTIONAL, float, 0.3, 0.0, 30.0, 0.25, "s",
             "Back-off after a recovery attempt that did NOT get closer. An "
             "attempt that made progress is followed immediately, so a long "
             "push comes back in one smooth motion instead of stuttering."),
    Property("hold_giveup_deg", OPTIONAL, float, 0.0, 0.0, 3600.0, 45.0, "deg",
             "Stop trying if the wheel is further than this from the target "
             "(someone is deliberately repositioning the bot). 0 = never "
             "give up on distance alone."),
    Property("hold_max_step_deg", OPTIONAL, float, 360.0, 5.0, 3600.0, 45.0, "deg",
             "Largest single recovery move. Below the distance of a typical "
             "hand push this turns one smooth return into several, so keep it "
             "generous unless a fast return is itself the hazard."),
    Property("soft_limit_deg", OPTIONAL, float, 0.0, 0.0, 100000.0, 90.0, "deg",
             "Fault if the wheel travels further than this from the odometry "
             "origin, either way. 0 = no limit."),
    Property("slew_duty_per_s", OPTIONAL, float, 0.0, 0.0, 2000.0, 50.0, "duty/s",
             "Ramp rate for open-loop duty changes. 0 = step instantly."),
    Property("arm_on_start", OPTIONAL, bool, True, doc=(
        "Latch a hold target as soon as the node starts, rather than waiting "
        "for the first commanded move to finish.")),
    Property("clear_firmware_reference", OPTIONAL, bool, True, doc=(
        "Drop the firmware's carried-forward rotate target before each "
        "recovery, so the move is measured from where the wheel actually is. "
        "See _forget_reference() for why this is needed.")),
    Property("label", OPTIONAL, str, "", doc="Name shown in the TUI."),
)

PROPERTY_BY_NAME = {p.name: p for p in PROPERTIES}


def _coerce(p: Property, value) -> object:
    if p.type is bool:
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "on", "yes", "y")
        return bool(value)
    if p.type is str:
        return str(value)
    v = float(value)
    if p.lo is not None:
        v = max(p.lo, v)
    if p.hi is not None:
        v = min(p.hi, v)
    return int(round(v)) if p.type is int else v


@dataclass
class MotorConfig:
    """Everything one motor is allowed and told to do. See `PROPERTIES`."""

    # critical
    enabled: bool = True
    hold_position: bool = True
    hold_tolerance_deg: float = 2.0
    hold_settle_s: float = 0.4
    hold_duty: int = 90
    hold_max_corrections: int = 8
    min_duty: int = 58
    max_duty: int = 200
    stall_timeout_s: float = 1.0
    estop_on_stall: bool = True
    check_direction: bool = True
    require_encoder: bool = True
    # optional
    hold_rearm_s: float = 0.6
    hold_retry_s: float = 0.3
    hold_giveup_deg: float = 0.0
    hold_max_step_deg: float = 360.0
    soft_limit_deg: float = 0.0
    slew_duty_per_s: float = 0.0
    arm_on_start: bool = True
    clear_firmware_reference: bool = True
    label: str = ""

    # ------------------------------------------------------------- access

    def set(self, name: str, value) -> object:
        """Set one property, coerced and clamped to its declared range."""
        p = PROPERTY_BY_NAME.get(name)
        if p is None:
            raise KeyError(f"no such motor property: {name}")
        v = _coerce(p, value)
        setattr(self, name, v)
        return v

    def get(self, name: str) -> object:
        if name not in PROPERTY_BY_NAME:
            raise KeyError(f"no such motor property: {name}")
        return getattr(self, name)

    def bump(self, name: str, direction: int) -> object:
        """Nudge a property one step; toggles a bool."""
        p = PROPERTY_BY_NAME[name]
        if p.type is bool:
            return self.set(name, not getattr(self, name))
        if p.type is str:
            return getattr(self, name)
        return self.set(name, getattr(self, name) + p.step * direction)

    def validate(self) -> list[str]:
        """Critical properties that are out of range. Empty list means good."""
        bad = []
        for p in PROPERTIES:
            if p.kind != CRITICAL or p.type in (bool, str):
                continue
            v = getattr(self, p.name)
            if (p.lo is not None and v < p.lo) or (p.hi is not None and v > p.hi):
                bad.append(f"{p.name}={v} outside {p.lo}..{p.hi} {p.unit}".strip())
        if self.max_duty < self.min_duty:
            bad.append(f"max_duty ({self.max_duty}) below min_duty ({self.min_duty})")
        if self.hold_duty > self.max_duty:
            bad.append(f"hold_duty ({self.hold_duty}) above max_duty ({self.max_duty})")
        return bad

    def copy(self, **over) -> "MotorConfig":
        d = asdict(self)
        d.update(over)
        return MotorConfig(**d)

    def as_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "MotorConfig":
        cfg = cls()
        for k, v in (d or {}).items():
            if k in PROPERTY_BY_NAME:
                cfg.set(k, v)
        return cfg

    @staticmethod
    def properties(kind: str | None = None) -> tuple[Property, ...]:
        return tuple(p for p in PROPERTIES if kind is None or p.kind == kind)


# ------------------------------------------------------------------- state

@dataclass
class MotorEvent:
    """Something happened to a motor that a human or a log should know."""

    t: float
    side: str
    kind: str
    message: str
    value: float = 0.0

    def as_dict(self) -> dict:
        return asdict(self)


# Event kinds:
#   hold_armed / hold_released   target latched or dropped
#   disturbed                    moved off target with nothing commanded
#   recovering / recovered       driving back / back inside tolerance
#   hold_failed                  out of attempts, or past hold_giveup_deg
#   stall / soft_limit           critical property tripped
#   encoder_lost / encoder_back  feedback gone / returned
#   fault / fault_cleared / estop / estop_cleared
#   move_done                    a commanded rotate finished


@dataclass
class MotorStatus:
    """Read-only snapshot of one motor. The TUI draws exactly this."""

    side: str = ""
    label: str = ""
    mode: str = "idle"          # see MODES
    fault: str = ""
    fault_since: float = 0.0

    hold_armed: bool = False
    target_deg: float = 0.0     # wheel angle the hold is defending
    position_deg: float = 0.0   # unwrapped wheel angle, forward positive
    angle_deg: float = 0.0      # the same wrapped into 0-360
    error_deg: float = 0.0      # position - target
    peak_error_deg: float = 0.0 # worst error of the current disturbance
    corrections: int = 0        # attempts on the current disturbance
    corrections_total: int = 0
    recoveries: int = 0         # disturbances brought back inside tolerance

    requested_duty: int = 0
    duty: int = 0               # after slew, clamps and gating
    speed_dps: float = 0.0
    moving: bool = False
    encoder_online: bool = False
    magnet_ok: bool = False
    last_move: str = ""

    def as_dict(self) -> dict:
        return asdict(self)


MODES = ("disabled", "fault", "driving", "moving", "recovering",
         "disturbed", "holding", "idle")


@dataclass
class _Move:
    side: int
    degrees: float
    duty: int
    tag: str                    # "user" or "recover"
    done: threading.Event = field(default_factory=threading.Event)
    result: MoveResult | None = None


class Motor:
    """One wheel: its properties, its hold target, and its faults."""

    def __init__(self, node: "MotorNode", side: int, config: MotorConfig):
        self.node = node
        self.side = side
        self.name = SIDE_NAME[side]
        self.cfg = config
        self.status = MotorStatus(side=self.name,
                                  label=config.label or self.name)

        self.target: float | None = None        # wheel degrees, or unarmed
        self.position = 0.0
        self._origin: int | None = None
        self._disturbed_since: float | None = None
        self._next_attempt = 0.0
        self._stall_since: float | None = None
        self._wrong_way_since: float | None = None
        self._last_motion = 0.0
        self._quiet_until = 0.0
        self._attempt_error: float | None = None   # error when the last try fired
        self._move_in_flight = False
        self._recovering = False
        self._started = False

    # ------------------------------------------------------------- control

    def arm(self, at: float | None = None) -> None:
        """Latch the hold target here (or at a given wheel angle)."""
        self.target = self.position if at is None else at
        self._disturbed_since = None
        self._next_attempt = 0.0
        self.status.corrections = 0
        self.status.peak_error_deg = 0.0
        self.node._emit(self.name, "hold_armed",
                        f"{self.name} holding {self.target:+.2f} deg",
                        self.target)

    def disarm(self, why: str = "released") -> None:
        if self.target is None:
            return
        self.target = None
        self._disturbed_since = None
        self.status.corrections = 0
        self.status.peak_error_deg = 0.0
        self.node._emit(self.name, "hold_released", f"{self.name} hold {why}")

    def fault(self, kind: str, message: str) -> None:
        if self.status.fault == kind:
            return
        self.status.fault = kind
        self.status.fault_since = time.time()
        self.disarm("dropped by fault")
        self.node._emit(self.name, kind, message)

    def clear_fault(self) -> None:
        if not self.status.fault:
            return
        kind = self.status.fault
        self.status.fault = ""
        self.status.fault_since = 0.0
        self._stall_since = None
        self.node._emit(self.name, "fault_cleared",
                        f"{self.name} {kind} cleared")

    # -------------------------------------------------------------- update

    def update(self, now: float, wo, ws, commanded: bool) -> None:
        """One control step. `wo` is the WheelOdom, `ws` the raw WheelState."""
        st = self.status
        st.label = self.cfg.label or self.name

        prev = self.position
        self.position = wo.wheel_angle_total_deg
        st.position_deg = self.position
        st.angle_deg = wo.wheel_angle_deg
        st.speed_dps = wo.omega_dps
        st.encoder_online = ws.online
        st.magnet_ok = ws.magnet_ok

        # `wo.moving` compares one sample against the last, but the board
        # streams ODO at 10 Hz while this loop runs at 25 Hz, so most samples
        # see no new data and report a turning wheel as stopped. Timestamp the
        # last real change instead and call the wheel moving for MOVING_HOLD_S
        # after it. Getting this wrong is what makes a hold fight a hand that
        # is still turning the wheel.
        # A move of our own is not a disturbance: the firmware brakes and
        # settles before it reports, so the wheel is already stopped by the
        # time the result arrives. Ignore the position catching up, or every
        # step of a long recovery pays the settle wait again and the wheel
        # walks home in visible lurches.
        if now < self._quiet_until:
            st.moving = False
        else:
            if abs(self.position - prev) > MOVING_DEG:
                self._last_motion = now
            st.moving = (now - self._last_motion) < MOVING_HOLD_S

        # The odometry re-origins when the counters are zeroed or a jump is
        # absorbed. Every angle measured against the old origin, the hold
        # target included, means nothing from that instant on -- so re-latch
        # here rather than drive back to a target that has moved underneath us.
        origin = self.node.odom.origin_counts(self.side)
        if origin != self._origin:
            if self._origin is not None and self.target is not None:
                self.target = self.position
                self.node._emit(self.name, "hold_armed",
                                f"{self.name} odometry re-origined; hold "
                                f"re-latched at {self.position:+.2f} deg",
                                self.position)
            self._origin = origin

        self._check_encoder(now, ws)
        self._check_soft_limit()
        self._check_stall(now)
        self._check_direction(now)

        st.hold_armed = self.target is not None
        st.target_deg = self.target if self.target is not None else 0.0
        st.error_deg = (self.position - self.target) if self.target is not None else 0.0

        if not self.cfg.enabled:
            self.disarm("motor disabled")
            st.mode = "disabled"
            return
        if st.fault or self.node.estopped:
            st.mode = "fault"
            return
        if self._move_in_flight:
            st.mode = "recovering" if self._recovering else "moving"
            return
        if commanded:
            self.disarm("open-loop drive")
            st.mode = "driving"
            return

        self._service_hold(now)

    # ---------------------------------------------------------- the checks

    def _check_encoder(self, now: float, ws) -> None:
        if not self.cfg.require_encoder:
            return
        fresh = self.node.driver.state.fresh
        if fresh and ws.online:
            if self.status.fault == "encoder_lost":
                self.clear_fault()
                self.node._emit(self.name, "encoder_back",
                                f"{self.name} encoder answering again")
            return
        self.fault("encoder_lost",
                   f"{self.name} encoder not answering; holding position "
                   f"without feedback is not safe, motor cut")

    def _check_soft_limit(self) -> None:
        limit = self.cfg.soft_limit_deg
        if limit <= 0:
            return
        if abs(self.position) > limit:
            self.fault("soft_limit",
                       f"{self.name} at {self.position:+.1f} deg, past the "
                       f"+-{limit:.0f} deg soft limit")

    def _check_stall(self, now: float) -> None:
        timeout = self.cfg.stall_timeout_s
        want = abs(self.status.duty) >= max(1, self.cfg.min_duty)
        if timeout <= 0 or not want or self._move_in_flight:
            self._stall_since = None
            return
        if self.status.moving:
            self._stall_since = None
            return
        if self._stall_since is None:
            self._stall_since = now
        elif now - self._stall_since > timeout:
            self.fault("stall",
                       f"{self.name} commanded at duty {self.status.duty} but "
                       f"not turning for {timeout:.1f} s")
            if self.cfg.estop_on_stall:
                self.node.estop(f"{self.name} stalled")

    def _check_direction(self, now: float) -> None:
        """
        The wheel must turn the way it was told to.

        Both motors are mounted mirror-imaged, so wiring them the same runs one
        backwards, and `calibration.json` carries a per-wheel `direction` to
        undo that. If that number is wrong, everything measured against it --
        odometry, pose, and the hold, which would drive *away* from its target
        -- is wrong with it, silently. So it is checked against what the
        encoder actually did rather than trusted.
        """
        st = self.status
        if not self.cfg.check_direction or self._move_in_flight:
            self._wrong_way_since = None
            return
        duty = st.duty
        if abs(duty) < max(1, self.cfg.min_duty) or abs(st.speed_dps) < 5.0:
            self._wrong_way_since = None
            return
        if (duty > 0) == (st.speed_dps > 0):
            self._wrong_way_since = None
            return
        # Ride out a direction reversal: the wheel is still coasting the old
        # way for a moment after the duty flips.
        if self._wrong_way_since is None:
            self._wrong_way_since = now
        elif now - self._wrong_way_since > 0.5:
            self.fault("wrong_direction",
                       f"{self.name} is turning {st.speed_dps:+.0f} deg/s on a "
                       f"duty of {duty:+d} -- backwards. Flip "
                       f"wheels.{self.name}.direction in calibration.json")

    # ------------------------------------------------------------ the hold

    def _service_hold(self, now: float) -> None:
        st = self.status
        cfg = self.cfg

        if not cfg.hold_position:
            self.disarm("hold switched off")
            st.mode = "idle"
            return

        if self.target is None:
            st.mode = "idle"
            ready = self._started or cfg.arm_on_start
            if ready and self.node.idle_seconds() >= cfg.hold_rearm_s \
                    and not st.moving:
                self.arm()
            return

        err = self.position - self.target
        st.error_deg = err

        if abs(err) <= cfg.hold_tolerance_deg:
            if self._disturbed_since is not None:
                st.recoveries += 1
                self.node._emit(self.name, "recovered",
                                f"{self.name} back on target "
                                f"({err:+.2f} deg) after {st.corrections} "
                                f"correction(s)", err)
            self._disturbed_since = None
            self._attempt_error = None
            st.corrections = 0
            st.peak_error_deg = 0.0
            st.mode = "holding"
            return

        st.mode = "disturbed"
        if abs(err) > abs(st.peak_error_deg):
            st.peak_error_deg = err
        if self._disturbed_since is None:
            self._disturbed_since = now
            self.node._emit(self.name, "disturbed",
                            f"{self.name} moved {err:+.2f} deg off target with "
                            f"nothing commanded", err)
        # Restart the settle clock for as long as the wheel is still turning:
        # correcting into a hand that has not let go yet just fights it.
        if st.moving:
            self._disturbed_since = now
            return

        if now - self._disturbed_since < cfg.hold_settle_s:
            return
        if now < self._next_attempt:
            return

        # An attempt that got closer is followed immediately -- that is what
        # turns a long push into one smooth return instead of a stutter. Only
        # an attempt that made no progress has to back off, and only that one
        # counts against hold_max_corrections, so a wheel walking home in
        # hold_max_step_deg-sized steps never runs out of attempts.
        made_progress = (self._attempt_error is not None
                         and abs(err) < abs(self._attempt_error) - cfg.hold_tolerance_deg)
        if made_progress:
            st.corrections = 0

        if cfg.hold_giveup_deg > 0 and abs(err) > cfg.hold_giveup_deg:
            self.fault("hold_failed",
                       f"{self.name} is {err:+.1f} deg off target, past the "
                       f"{cfg.hold_giveup_deg:.0f} deg give-up distance; "
                       f"assuming the move was deliberate")
            return
        if st.corrections >= cfg.hold_max_corrections:
            self.fault("hold_failed",
                       f"{self.name} still {err:+.2f} deg off target after "
                       f"{st.corrections} corrections; something is holding "
                       f"the wheel")
            return

        step = err
        if cfg.hold_max_step_deg > 0:
            step = max(-cfg.hold_max_step_deg,
                       min(cfg.hold_max_step_deg, err))
        st.corrections += 1
        st.corrections_total += 1
        self._attempt_error = err
        self._next_attempt = now + (0.0 if made_progress else cfg.hold_retry_s)
        st.mode = "recovering"
        self.node._emit(self.name, "recovering",
                        f"{self.name} returning {-step:+.2f} deg to target "
                        f"(attempt {st.corrections})", -step)
        self.node.submit_move(self.side, -step, cfg.hold_duty, tag="recover")


class MotorNode:
    """
    Supervises both motors: properties, position hold, faults and e-stop.

    Nothing else may talk to the driver while this is running -- it owns the
    open-loop duty stream (which has to be re-sent faster than the firmware's
    300 ms watchdog) and it serialises every closed-loop move.

        with WheelDriver("/dev/ttyACM0") as bot:
            node = MotorNode(bot)
            node.on_event = lambda e: print(e.message)
            node.start()
            ...
            node.stop()
    """

    def __init__(self, driver: WheelDriver,
                 odom: WheelOdometryNode | None = None,
                 config: MotorConfig | None = None,
                 left_config: MotorConfig | None = None,
                 right_config: MotorConfig | None = None,
                 rate_hz: float = 25.0,
                 track_width_m: float | None = None):
        self.driver = driver
        self.odom = odom or WheelOdometryNode(driver, track_width_m=track_width_m)
        self.rate_hz = rate_hz

        base = config or MotorConfig()
        self.motors: dict[int, Motor] = {
            LEFT: Motor(self, LEFT, left_config or base.copy(label="LEFT")),
            RIGHT: Motor(self, RIGHT, right_config or base.copy(label="RIGHT")),
        }

        self.estopped = False
        self.estop_reason = ""
        self.frame = None
        self.events: list[MotorEvent] = []
        self.max_events = 300
        self.on_event = None            # callback(MotorEvent)
        self.on_frame = None            # callback(OdometryFrame)

        self._drive = {LEFT: 0, RIGHT: 0}
        self._applied = {LEFT: 0.0, RIGHT: 0.0}
        self._drive_until = 0.0         # 0 means "until told otherwise"
        self._last_tx = 0.0
        self._tx_period = 0.1           # re-send at 10 Hz; watchdog is 300 ms

        self._moves: queue.Queue[_Move] = queue.Queue()
        self._move_side: int | None = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # ------------------------------------------------------------ lifecycle

    def start(self) -> "MotorNode":
        bad = self.invalid_properties()
        if bad:
            raise ValueError("critical motor properties out of range: "
                             + "; ".join(bad))
        if self._threads:
            return self
        self._stop.clear()
        for target in (self._control_loop, self._move_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()
        try:
            self.driver.stop()
        except Exception:
            pass

    def __enter__(self) -> "MotorNode":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def invalid_properties(self) -> list[str]:
        out = []
        for side in SIDES:
            out += [f"{SIDE_NAME[side]}: {m}"
                    for m in self.motors[side].cfg.validate()]
        return out

    # -------------------------------------------------------------- events

    def _emit(self, side: str, kind: str, message: str, value: float = 0.0) -> None:
        ev = MotorEvent(t=time.time(), side=side, kind=kind,
                        message=message, value=value)
        self.events.append(ev)
        if len(self.events) > self.max_events:
            del self.events[:-self.max_events]
        if self.on_event:
            self.on_event(ev)

    # ------------------------------------------------------------ commands

    def drive(self, left: int, right: int, hold_s: float | None = None) -> None:
        """
        Open-loop drive both sides. Re-sent automatically at 10 Hz.

        `hold_s` makes the command expire on its own, which is what a keyboard
        TUI wants: a held key refreshes it, releasing the key stops the bot.
        """
        for side, duty in ((LEFT, left), (RIGHT, right)):
            self._drive[side] = self._gate_duty(side, duty)
        self._drive_until = (time.time() + hold_s) if hold_s else 0.0
        if self._drive[LEFT] or self._drive[RIGHT]:
            self._cancel_moves("preempted by drive")

    def drive_side(self, side: int, duty: int, hold_s: float | None = None) -> None:
        """Change one side's duty, leaving the other where it is."""
        left = duty if side == LEFT else self._drive[LEFT]
        right = duty if side == RIGHT else self._drive[RIGHT]
        self.drive(left, right, hold_s)

    def halt(self) -> None:
        """Stop open-loop drive and abort any move in flight."""
        self._drive = {LEFT: 0, RIGHT: 0}
        self._applied = {LEFT: 0.0, RIGHT: 0.0}
        self._drive_until = 0.0
        self._cancel_moves("halted")
        try:
            self.driver.stop()
        except Exception:
            pass

    def estop(self, reason: str = "operator") -> None:
        if self.estopped:
            return
        self.estopped = True
        self.estop_reason = reason
        self.halt()
        for m in self.motors.values():
            m.disarm("e-stop")
        self._emit("BODY", "estop", f"E-STOP: {reason}")

    def clear_estop(self) -> None:
        if not self.estopped:
            return
        self.estopped = False
        self._emit("BODY", "estop_cleared",
                   f"e-stop cleared (was: {self.estop_reason})")
        self.estop_reason = ""

    def clear_faults(self, side: int | None = None) -> None:
        for s in (SIDES if side is None else (side,)):
            self.motors[s].clear_fault()

    def zero(self) -> None:
        """Zero the encoders and re-latch both hold targets at the new origin."""
        self.halt()
        self.odom.zero_encoders()
        for m in self.motors.values():
            m.position = 0.0
            if m.cfg.hold_position and m.cfg.enabled and not m.status.fault:
                m.arm(0.0)

    # ----------------------------------------------------------- hold API

    def hold_here(self, side: int | None = None) -> None:
        """Latch the current angle as the target, whatever the wheel is doing."""
        for s in (SIDES if side is None else (side,)):
            m = self.motors[s]
            if m.cfg.enabled and not m.status.fault:
                m.arm()

    def release(self, side: int | None = None) -> None:
        for s in (SIDES if side is None else (side,)):
            self.motors[s].disarm("released by operator")

    def set_hold(self, side: int | None, on: bool) -> None:
        for s in (SIDES if side is None else (side,)):
            self.motors[s].cfg.set("hold_position", on)
            if not on:
                self.motors[s].disarm("hold switched off")

    def set_property(self, side: int | None, name: str, value) -> object:
        out = None
        for s in (SIDES if side is None else (side,)):
            out = self.motors[s].cfg.set(name, value)
        return out

    def bump_property(self, side: int | None, name: str, direction: int) -> object:
        out = None
        for s in (SIDES if side is None else (side,)):
            out = self.motors[s].cfg.bump(name, direction)
        return out

    # ----------------------------------------------------------- moves

    def max_degrees_per_move(self, side: int, duty: int) -> float:
        """
        The largest single `G` that can finish before the firmware gives up.

        The board abandons a rotate after 30 s wherever it has got to and
        reports "timeout" -- so a 7200 deg command at duty 90 does not come up
        short because the maths is wrong, it comes up short because it ran out
        of clock. Sized from the measured duty/speed curve, with the taper over
        the last fifth paid for and a margin left over for a low battery.
        """
        fit = self.driver.cal.get("duty_to_counts_per_sec", {})
        slope = float(fit.get("slope", 959.0))
        intercept = float(fit.get("intercept", -47316.0))
        cps = slope * abs(duty) + intercept
        if cps <= 0:
            return 360.0
        cpd = self.driver.counts_per_degree(side)
        return max(90.0, cps * GO_TIMEOUT_S * GO_TIMEOUT_USE / GO_TAPER_COST / cpd)

    def rotate(self, side: int, degrees: float, duty: int | None = None,
               wait: bool = False, timeout: float = 40.0) -> MoveResult | None:
        """
        Commanded closed-loop rotate, split into as many `G` moves as it takes.

        The hold target follows the move, so the wheel defends where it was
        *asked* to end up, not where it started. Anything longer than the
        firmware can finish in 30 s is chunked; the chunks carry the firmware's
        ideal target forward between them, so splitting costs no accuracy.
        """
        cfg = self.motors[side].cfg
        duty = self._gate_duty(side, duty if duty is not None else cfg.hold_duty,
                               floor=True)
        if duty == 0:
            return None

        chunk = self.max_degrees_per_move(side, abs(duty))
        left = float(degrees)
        moves = []
        while abs(left) > 1e-3:
            step = max(-chunk, min(chunk, left))
            mv = self.submit_move(side, step, abs(duty), tag="user")
            if mv is None:
                break
            moves.append(mv)
            left -= step
        if not moves:
            return None
        if len(moves) > 1:
            self._emit(SIDE_NAME[side], "move_split",
                       f"{SIDE_NAME[side]} {degrees:+.0f} deg split into "
                       f"{len(moves)} moves of up to {chunk:.0f} deg -- the "
                       f"firmware abandons a rotate after {GO_TIMEOUT_S:.0f} s",
                       float(len(moves)))
        if not wait:
            return None
        for mv in moves:
            mv.done.wait(timeout)
        return moves[-1].result

    def submit_move(self, side: int, degrees: float, duty: int,
                    tag: str = "user") -> _Move | None:
        m = self.motors[side]
        if self.estopped or m.status.fault or not m.cfg.enabled:
            return None
        if abs(degrees) < 1e-3:
            return None
        mv = _Move(side=side, degrees=float(degrees),
                   duty=int(min(duty, m.cfg.max_duty)), tag=tag)
        self._moves.put(mv)
        return mv

    def _cancel_moves(self, why: str) -> None:
        while True:
            try:
                mv = self._moves.get_nowait()
            except queue.Empty:
                break
            mv.result = MoveResult(SIDE_NAME[mv.side], mv.degrees, 0.0, 0.0,
                                   "aborted")
            mv.done.set()
        if self._move_side is not None:
            try:
                self.driver.stop()          # X aborts the firmware's rotate
            except Exception:
                pass

    # ------------------------------------------------------------- status

    def idle_seconds(self) -> float:
        """How long since anything could have driven a wheel."""
        if self._move_side is not None:
            return 0.0
        return self.driver.idle_seconds()

    def status(self, side: int) -> MotorStatus:
        return self.motors[side].status

    def snapshot(self) -> dict:
        f = self.frame
        return {
            "t": time.time(),
            "connected": self.driver.state.connected,
            "fresh": self.driver.state.fresh,
            "estopped": self.estopped,
            "estop_reason": self.estop_reason,
            "left": self.motors[LEFT].status.as_dict(),
            "right": self.motors[RIGHT].status.as_dict(),
            "config": {"LEFT": self.motors[LEFT].cfg.as_dict(),
                       "RIGHT": self.motors[RIGHT].cfg.as_dict()},
            "odom": f.as_dict() if f is not None else None,
        }

    # --------------------------------------------------------------- loops

    def poll(self, now: float | None = None):
        """One supervision step. Called by the control thread; safe in tests."""
        now = now or time.time()
        f = self.odom.update(now)
        self.frame = f

        if self._drive_until and now > self._drive_until:
            self._drive = {LEFT: 0, RIGHT: 0}
            self._drive_until = 0.0

        st = self.driver.state
        for side in SIDES:
            m = self.motors[side]
            wo = f.left if side == LEFT else f.right
            ws = st.left if side == LEFT else st.right
            m.update(now, wo, ws, commanded=bool(self._drive[side]))

        self._push_duties(now)
        if self.on_frame:
            self.on_frame(f)
        return f

    def _gate_duty(self, side: int, duty, floor: bool = False) -> int:
        """Clamp to this motor's limits, and treat sub-deadband as zero."""
        cfg = self.motors[side].cfg
        d = int(duty)
        if not cfg.enabled or self.estopped or self.motors[side].status.fault:
            return 0
        if abs(d) < cfg.min_duty:
            if not floor or d == 0:
                return 0
            d = cfg.min_duty if d > 0 else -cfg.min_duty
        return max(-cfg.max_duty, min(cfg.max_duty, d))

    def _push_duties(self, now: float) -> None:
        """
        Own the open-loop duty stream.

        Two rules, both load-bearing: never send `D` while a closed-loop move
        is running (the firmware treats `D` as a preempt and kills the rotate),
        and re-send faster than the 300 ms firmware watchdog or the motors stop
        on their own.
        """
        if self._move_side is not None:
            return

        for side in SIDES:
            cfg = self.motors[side].cfg
            want = float(self._gate_duty(side, self._drive[side]))
            cur = self._applied[side]
            if cfg.slew_duty_per_s > 0 and want != cur:
                step = cfg.slew_duty_per_s * self._tx_period
                cur = max(cur - step, min(cur + step, want))
                if abs(cur) < cfg.min_duty and want == 0:
                    cur = 0.0
            else:
                cur = want
            self._applied[side] = cur
            self.motors[side].status.requested_duty = int(self._drive[side])
            self.motors[side].status.duty = int(round(cur))

        left = self.motors[LEFT].status.duty
        right = self.motors[RIGHT].status.duty
        moving = bool(left or right)

        if moving:
            if now - self._last_tx >= self._tx_period:
                self.driver.set_duty(left, right)
                self._last_tx = now
        elif self._last_tx:
            self.driver.stop()
            self._last_tx = 0.0

    def _control_loop(self) -> None:
        period = 1.0 / self.rate_hz
        while not self._stop.is_set():
            t0 = time.time()
            try:
                self.poll(t0)
            except Exception as e:                  # never let the loop die
                self._emit("BODY", "fault", f"control loop error: {e}")
            time.sleep(max(0.0, period - (time.time() - t0)))
        try:
            self.driver.stop()
        except Exception:
            pass

    def _move_loop(self) -> None:
        while not self._stop.is_set():
            try:
                mv = self._moves.get(timeout=0.15)
            except queue.Empty:
                continue
            self.run_move(mv)

    def run_move(self, mv: _Move) -> MoveResult | None:
        """Execute one closed-loop move. One at a time, by construction."""
        m = self.motors[mv.side]
        if self.estopped or m.status.fault or not m.cfg.enabled:
            mv.result = MoveResult(SIDE_NAME[mv.side], mv.degrees, 0.0, 0.0,
                                   "aborted")
            mv.done.set()
            return mv.result

        self._move_side = mv.side
        m._move_in_flight = True
        m._recovering = (mv.tag == "recover")
        try:
            # Only a recovery clears it. For a commanded move -- including
            # every chunk after the first of a long one -- the carried-forward
            # target is exactly what stops a run of moves accumulating error
            # (SETUP.md 4a), so clearing it there would undo that fix.
            if mv.tag == "recover" and m.cfg.clear_firmware_reference:
                self._forget_reference(mv.side)
            res = self.driver.rotate(mv.side, mv.degrees, mv.duty,
                                     wait=True, timeout=45.0)
        except Exception as e:
            res = MoveResult(SIDE_NAME[mv.side], mv.degrees, 0.0, 0.0,
                             f"error: {e}")
        finally:
            m._move_in_flight = False
            m._recovering = False
            m._quiet_until = time.time() + MOVE_QUIET_S
            self._move_side = None

        m.status.last_move = (f"{res.reason} {res.actual_deg:+.2f} deg "
                              f"(residual {res.error_deg:+.2f})") if res else ""
        m._started = True
        if res and res.reason not in MOVE_OK:
            self._emit(m.name, "move_done",
                       f"{m.name} rotate ended '{res.reason}' after "
                       f"{res.actual_deg:+.2f} deg")
        if mv.tag == "user":
            # Defend where the move was asked to finish, not where it began.
            if m.cfg.hold_position and m.cfg.enabled and not m.status.fault:
                base = m.target if m.target is not None else m.position
                m.arm(base + mv.degrees)
        mv.result = res
        mv.done.set()
        return res

    def _forget_reference(self, side: int) -> None:
        """
        Make the next `G` measure from where the wheel actually is.

        The firmware carries a per-side "ideal" target forward between rotates
        so a run of identical moves cannot accumulate error (SETUP.md 4a). That
        is right for a sequence of commanded moves and wrong for a recovery: a
        hand nudge of less than 45 deg leaves the ideal intact, so a corrective
        `G -e` would be measured from the ideal rather than from the wheel, and
        overshoot by e. The firmware drops the reference on any `D` with a
        non-zero duty, so a duty of 1 -- an order of magnitude under the 45-50
        deadband, incapable of turning the wheel -- clears it. The `X` after it
        clears the drive watchdog the `D` just armed, which would otherwise
        fire mid-rotate.
        """
        self.driver.set_duty(1 if side == LEFT else 0,
                             1 if side == RIGHT else 0)
        time.sleep(0.04)
        self.driver.stop()
        time.sleep(0.04)
