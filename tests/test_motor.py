#!/usr/bin/env python3
"""
Offline checks for MotorNode -- no hardware, no serial port.

A fake driver stands in for the board: `rotate()` moves the synthetic counters
the way the firmware would, so the position-hold loop can be driven all the way
from "a hand turned the wheel" to "the wheel is back on target".
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carbot.driver import BaseState, LEFT, RIGHT, MoveResult, load_calibration
from carbot.motor import (CRITICAL, GO_TIMEOUT_S, OPTIONAL, MotorConfig,
                          MotorNode, PROPERTIES, PROPERTY_BY_NAME)

CAL = load_calibration()
SIGN = CAL["encoder_sign"]
CPR_L = CAL["wheels"]["LEFT"]["counts_per_wheel_rev"]
CPD_L = CPR_L / 360.0                       # counts per wheel degree, LEFT

FAST = dict(hold_rearm_s=0.05, hold_settle_s=0.05, hold_retry_s=0.0,
            hold_tolerance_deg=1.0, clear_firmware_reference=False,
            stall_timeout_s=0.0)


class FakeDriver:
    """Same surface as WheelDriver, backed by variables instead of a port."""

    def __init__(self):
        self.cal = CAL
        self.state = BaseState()
        self.state.connected = True
        self.state.left.online = self.state.right.online = True
        self.state.left.status = self.state.right.status = 0x20
        self.state.left.agc = self.state.right.agc = 64
        self.state.last_rx = time.time()
        self.last_command = None
        self._command_until = 0.0
        self.sent = []
        self.moves = []

    # ---- driver API used by the node
    def counts_per_metre(self, side=None):
        return self.cal["wheels"]["LEFT" if side == LEFT else "RIGHT"]["counts_per_metre"]

    def counts_per_degree(self, side):
        return self.cal["wheels"]["LEFT" if side == LEFT else "RIGHT"]["counts_per_degree"]

    def encoder_sign(self, side):
        return SIGN

    def command_active(self):
        return time.time() < self._command_until

    def idle_seconds(self):
        return max(0.0, time.time() - self._command_until)

    def set_duty(self, l, r):
        self.sent.append(("D", l, r))
        self.last_command = (time.time(), l, r)
        self._command_until = time.time() + (0.3 if (l or r) else 0.0)

    def stop(self):
        self.sent.append(("X",))
        self.last_command = (time.time(), 0, 0)
        self._command_until = time.time()

    def zero(self):
        self.state.left.counts = self.state.right.counts = 0

    def rotate(self, side, degrees, duty=90, wait=True, timeout=40.0):
        """Move the wheel exactly as asked, minus a small residual."""
        self.moves.append((side, round(degrees, 3), duty))
        self._command_until = time.time() + 0.05
        self.turn(**({"left_deg": degrees} if side == LEFT
                     else {"right_deg": degrees}))
        self.state.last_rx = time.time()
        return MoveResult("LEFT" if side == LEFT else "RIGHT",
                          degrees, degrees, 0.0, "reached")

    # ---- test helpers
    def turn(self, left_deg=0.0, right_deg=0.0):
        """Turn the wheels forward by this many WHEEL degrees."""
        self.state.left.counts += int(SIGN * left_deg * CPD_L)
        self.state.right.counts += int(SIGN * right_deg * CPD_L)
        self.state.last_rx = time.time()

    def idle(self):
        self.last_command = (time.time(), 0, 0)
        self._command_until = time.time() - 10.0


def node(**over):
    cfg = MotorConfig(**{**FAST, **over})
    d = FakeDriver()
    n = MotorNode(d, config=cfg, rate_hz=50.0)
    return d, n


def settle(n, d, seconds=0.25, step=0.02):
    """Run the control loop for a while, executing any move it queues."""
    end = time.time() + seconds
    while time.time() < end:
        d.state.last_rx = time.time()
        n.poll()
        drain(n)
        time.sleep(step)


def drain(n):
    """Run at most one queued move, so the odometry gets sampled between them."""
    try:
        mv = n._moves.get_nowait()
    except Exception:
        return
    n.run_move(mv)


def check(name, cond, detail=""):
    print(f"  {'PASS' if cond else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    assert cond, f"{name} {detail}"


# --------------------------------------------------------------- properties

def test_properties():
    print("properties")
    names = {p.name for p in PROPERTIES}
    fields = set(MotorConfig().as_dict())
    check("PROPERTIES matches MotorConfig fields", names == fields,
          f"{names ^ fields or ''}")
    check("both kinds are populated",
          any(p.kind == CRITICAL for p in PROPERTIES)
          and any(p.kind == OPTIONAL for p in PROPERTIES),
          f"{len(MotorConfig.properties(CRITICAL))} critical, "
          f"{len(MotorConfig.properties(OPTIONAL))} optional")

    cfg = MotorConfig()
    check("values clamp to the declared range", cfg.set("hold_duty", 900) == 255)
    check("sub-deadband hold duty clamps up", cfg.set("hold_duty", 5) == 58)
    check("bool accepts strings", cfg.set("hold_position", "off") is False)
    check("bump toggles a bool", cfg.bump("hold_position", +1) is True)
    try:
        cfg.set("no_such_thing", 1)
        check("unknown property rejected", False)
    except KeyError:
        check("unknown property rejected", True)

    check("a sane config validates", MotorConfig().validate() == [])
    bad = MotorConfig(min_duty=200, max_duty=100)
    check("max_duty below min_duty is caught", len(bad.validate()) > 0,
          bad.validate()[0])
    d, n = node()
    n.motors[LEFT].cfg.max_duty = 10
    try:
        n.start()
        check("start refuses an invalid critical property", False)
    except ValueError as e:
        check("start refuses an invalid critical property", True, str(e)[:60])


# --------------------------------------------------------------- hold

def test_arms_when_idle():
    print("hold arms when idle")
    d, n = node()
    d.idle()
    settle(n, d, 0.2)
    check("target latched", n.motors[LEFT].status.hold_armed)
    check("no move needed", d.moves == [], str(d.moves))


def test_hand_push_returns():
    print("a hand push comes back")
    d, n = node()
    d.idle()
    settle(n, d, 0.2)
    target = n.motors[LEFT].target
    d.turn(left_deg=10.0)                       # someone turns the wheel
    settle(n, d, 0.5)
    check("a recovery move was issued", len(d.moves) == 1, str(d.moves))
    if d.moves:
        check("it drove back the right way", d.moves[0][0] == LEFT
              and -10.5 < d.moves[0][1] < -9.5, f"{d.moves[0][1]:+.2f} deg")
        check("it used hold_duty", d.moves[0][2] == n.motors[LEFT].cfg.hold_duty)
    err = n.motors[LEFT].status.error_deg
    check("wheel is back on target", abs(err) < 1.0, f"error {err:+.3f} deg")
    check("target unchanged", abs(n.motors[LEFT].target - target) < 1e-6)
    check("mode is holding", n.motors[LEFT].status.mode == "holding",
          n.motors[LEFT].status.mode)
    check("a recovery was counted", n.motors[LEFT].status.recoveries == 1)
    kinds = [e.kind for e in n.events]
    check("disturbed and recovered were reported",
          "disturbed" in kinds and "recovered" in kinds, str(kinds))
    check("the other wheel was left alone",
          all(m[0] == LEFT for m in d.moves))


def test_small_nudge_ignored():
    print("a nudge inside tolerance is left alone")
    d, n = node(hold_tolerance_deg=5.0)
    d.idle()
    settle(n, d, 0.2)
    d.turn(left_deg=2.0)
    settle(n, d, 0.4)
    check("no correction fired", d.moves == [], str(d.moves))
    check("still holding", n.motors[LEFT].status.mode == "holding")


def test_waits_for_the_hand_to_let_go():
    print("it does not fight a hand still turning the wheel")
    d, n = node(hold_settle_s=0.3)
    d.idle()
    settle(n, d, 0.2)
    for _ in range(8):                          # a slow, continuous push
        d.turn(left_deg=3.0)
        d.state.last_rx = time.time()
        n.poll()
        drain(n)
        time.sleep(0.03)
    check("nothing corrected while the wheel was moving", d.moves == [],
          str(d.moves))
    check("but the disturbance was noticed",
          n.motors[LEFT].status.mode == "disturbed")
    settle(n, d, 0.6)
    check("it corrected once the wheel stopped", len(d.moves) >= 1)
    check("wheel is back on target", abs(n.motors[LEFT].status.error_deg) < 1.0,
          f"{n.motors[LEFT].status.error_deg:+.3f} deg")


def test_hold_off_does_nothing():
    print("hold switched off")
    d, n = node(hold_position=False)
    d.idle()
    settle(n, d, 0.2)
    d.turn(left_deg=20.0)
    settle(n, d, 0.4)
    check("no target latched", not n.motors[LEFT].status.hold_armed)
    check("no correction", d.moves == [], str(d.moves))
    check("mode is idle", n.motors[LEFT].status.mode == "idle")


def test_big_push_walks_home():
    print("a big push walks home in steps")
    d, n = node(hold_max_step_deg=45.0, hold_max_corrections=20)
    d.idle()
    settle(n, d, 0.2)
    d.turn(left_deg=200.0)
    settle(n, d, 1.2)
    check("more than one step was used", len(d.moves) >= 4, str(len(d.moves)))
    check("no step exceeded hold_max_step_deg",
          all(abs(m[1]) <= 45.01 for m in d.moves),
          str([round(m[1], 1) for m in d.moves]))
    check("wheel is back on target", abs(n.motors[LEFT].status.error_deg) < 1.0,
          f"{n.motors[LEFT].status.error_deg:+.3f} deg")


def test_gives_up_rather_than_hunting():
    print("a blocked wheel gives up instead of hunting")
    d, n = node(hold_max_corrections=3)
    blocked = {"on": False}
    real_rotate = d.rotate

    def stuck(side, degrees, duty=90, wait=True, timeout=40.0):
        d.moves.append((side, round(degrees, 3), duty))   # commanded, no motion
        d._command_until = time.time() + 0.02
        return MoveResult("LEFT", degrees, 0.0, degrees, "timeout")
    d.idle()
    settle(n, d, 0.2)
    d.rotate = stuck
    d.turn(left_deg=15.0)
    settle(n, d, 0.8)
    check("it stopped after hold_max_corrections", len(d.moves) == 3,
          str(len(d.moves)))
    check("and raised hold_failed",
          n.motors[LEFT].status.fault == "hold_failed",
          n.motors[LEFT].status.fault)
    d.rotate = real_rotate
    n.clear_faults(LEFT)
    check("the fault clears", n.motors[LEFT].status.fault == "")


def test_giveup_distance():
    print("a deliberate repositioning is not fought")
    d, n = node(hold_giveup_deg=90.0)
    d.idle()
    settle(n, d, 0.2)
    d.turn(left_deg=180.0)
    settle(n, d, 0.4)
    check("no correction attempted", d.moves == [], str(d.moves))
    check("hold_failed raised instead",
          n.motors[LEFT].status.fault == "hold_failed")


# --------------------------------------------------------------- faults

def test_stall():
    print("stall detection")
    d, n = node(stall_timeout_s=0.15, estop_on_stall=True)
    n.drive(120, 0)
    settle(n, d, 0.4)                           # counts never change
    check("stall fault raised", n.motors[LEFT].status.fault == "stall",
          n.motors[LEFT].status.fault)
    check("and it e-stopped the base", n.estopped)
    check("duty was cut", n.motors[LEFT].status.duty == 0)
    n.clear_faults()
    n.clear_estop()
    check("both clear", not n.estopped and not n.motors[LEFT].status.fault)


def test_soft_limit():
    print("soft limits")
    d, n = node(soft_limit_deg=90.0)
    d.idle()
    settle(n, d, 0.15)
    d.turn(left_deg=120.0)
    settle(n, d, 0.2)
    check("soft limit fault", n.motors[LEFT].status.fault == "soft_limit")
    check("the other wheel is unaffected",
          n.motors[RIGHT].status.fault == "")


def test_encoder_loss():
    print("encoder loss")
    d, n = node()
    d.idle()
    settle(n, d, 0.15)
    d.state.left.online = False
    settle(n, d, 0.1)
    check("encoder_lost fault", n.motors[LEFT].status.fault == "encoder_lost")
    check("hold dropped", not n.motors[LEFT].status.hold_armed)
    d.state.left.online = True
    settle(n, d, 0.1)
    check("it recovers on its own", n.motors[LEFT].status.fault == "")


def test_estop_blocks_everything():
    print("e-stop")
    d, n = node()
    d.idle()
    settle(n, d, 0.15)
    n.estop("test")
    d.turn(left_deg=30.0)
    n.drive(200, 200)
    settle(n, d, 0.3)
    check("no duty is sent", n.motors[LEFT].status.duty == 0)
    check("no move is queued", d.moves == [], str(d.moves))
    check("mode is fault", n.motors[LEFT].status.mode == "fault")
    n.clear_estop()
    settle(n, d, 0.2)
    check("hold re-arms after the clear", n.motors[LEFT].status.hold_armed)


def test_disabled_motor():
    print("a disabled motor")
    d, n = node()
    n.set_property(LEFT, "enabled", False)
    n.drive(120, 120)
    settle(n, d, 0.2)
    check("left is dead", n.motors[LEFT].status.duty == 0
          and n.motors[LEFT].status.mode == "disabled")
    check("right still drives", n.motors[RIGHT].status.duty == 120,
          str(n.motors[RIGHT].status.duty))


def test_duty_gating():
    print("duty limits")
    d, n = node(max_duty=150, min_duty=58)
    n.drive(255, 30)
    settle(n, d, 0.1)
    check("clamped to max_duty", n.motors[LEFT].status.duty == 150,
          str(n.motors[LEFT].status.duty))
    check("sub-deadband is treated as stop", n.motors[RIGHT].status.duty == 0,
          str(n.motors[RIGHT].status.duty))


def test_commanded_move_moves_the_target():
    print("a commanded rotate carries the hold target with it")
    d, n = node()
    d.idle()
    settle(n, d, 0.2)
    start = n.motors[LEFT].target
    n.rotate(LEFT, 90.0, duty=90)
    drain(n)
    settle(n, d, 0.3)
    check("target advanced by the commanded angle",
          abs((n.motors[LEFT].target - start) - 90.0) < 0.5,
          f"{n.motors[LEFT].target - start:+.2f} deg")
    check("and it is holding there",
          n.motors[LEFT].status.mode == "holding"
          and abs(n.motors[LEFT].status.error_deg) < 1.0,
          f"{n.motors[LEFT].status.mode} {n.motors[LEFT].status.error_deg:+.2f}")


def test_drive_expiry():
    print("a drive command with hold_s expires on its own")
    d, n = node()
    n.drive(100, 100, hold_s=0.1)
    settle(n, d, 0.05)
    check("driving", n.motors[LEFT].status.duty == 100)
    time.sleep(0.12)
    n.poll()
    check("stopped by itself", n.motors[LEFT].status.duty == 0)
    check("an X was sent", ("X",) in d.sent)


def test_no_duty_while_moving():
    print("no open-loop duty is sent while a rotate is in flight")
    d, n = node()
    d.idle()
    settle(n, d, 0.15)
    n._move_side = LEFT
    before = len(d.sent)
    n._drive[LEFT] = 120
    n.poll()
    check("nothing was written to the port", len(d.sent) == before,
          str(d.sent[before:]))
    n._move_side = None


def test_long_move_is_split():
    print("a move too long for the firmware's timeout is split")
    d, n = node()
    d.idle()
    settle(n, d, 0.15)

    cap90 = n.max_degrees_per_move(LEFT, 90)
    check("the cap is sized from the duty curve", 2000 < cap90 < 4000,
          f"{cap90:.0f} deg at duty 90")
    check("a lower duty gets a shorter cap",
          n.max_degrees_per_move(LEFT, 60) < cap90,
          f"{n.max_degrees_per_move(LEFT, 60):.0f} deg at duty 60")
    check("the cap really is the firmware's clock",
          abs(cap90 * CPD_L / (959 * 90 - 47316) - GO_TIMEOUT_S * 0.6 / 1.2) < 0.1,
          f"{cap90 * CPD_L / (959 * 90 - 47316):.1f} s of driving")

    # 20 turns at duty 90 is nearly 39 s of driving, well past the 30 s the
    # firmware allows one rotate -- the case that came up short on hardware.
    n.rotate(LEFT, 7200.0, duty=90)
    queued = []
    while not n._moves.empty():
        queued.append(n._moves.get_nowait().degrees)
    check("7200 deg is split", len(queued) == 3, str(len(queued)))
    check("no chunk can time out",
          all(abs(x) <= cap90 + 0.01 for x in queued),
          str([round(x) for x in queued]))
    check("the chunks add up to the command",
          abs(sum(queued) - 7200.0) < 0.01, f"{sum(queued):.2f} deg")
    check("a split was reported",
          any(e.kind == "move_split" for e in n.events))

    # Run one end to end at a duty whose chunks the fake odometry can follow.
    d, n = node()
    d.idle()
    settle(n, d, 0.15)
    n.rotate(LEFT, 1800.0, duty=60)
    settle(n, d, 0.5)
    check("five turns landed",
          abs(n.motors[LEFT].status.position_deg - 1800.0) < 1.0,
          f"{n.motors[LEFT].status.position_deg:.2f} deg")
    check("and it holds there",
          abs(n.motors[LEFT].status.error_deg) < 1.0
          and n.motors[LEFT].status.mode == "holding",
          n.motors[LEFT].status.mode)


def test_short_move_is_not_split():
    print("a move that fits is sent whole")
    d, n = node()
    d.idle()
    settle(n, d, 0.15)
    n.rotate(LEFT, 90.0, duty=90)
    drain(n)
    check("one move", len(d.moves) == 1, str(d.moves))
    check("nothing else queued", n._moves.empty())
    check("no split reported",
          not any(e.kind == "move_split" for e in n.events))


def test_reference_cleared_only_for_recoveries():
    print("the firmware's carried-forward target survives commanded moves")
    d, n = node(clear_firmware_reference=True)
    d.idle()
    settle(n, d, 0.15)
    d.sent.clear()
    n.rotate(LEFT, 45.0, duty=90)
    drain(n)
    check("a commanded move does not clear it",
          ("D", 1, 0) not in d.sent, str(d.sent))
    d.sent.clear()
    d.turn(left_deg=10.0)
    settle(n, d, 0.5)
    check("a recovery does clear it", ("D", 1, 0) in d.sent, str(d.sent))


def test_wrong_direction():
    print("a mirror-wired motor is caught, not trusted")
    d, n = node(stall_timeout_s=0.0)
    n.drive(120, 120)
    for _ in range(30):                         # wheel turns the wrong way
        d.state.last_rx = time.time()
        n.poll()
        d.turn(left_deg=-4.0, right_deg=+4.0)
        time.sleep(0.03)
    check("the backwards wheel faults",
          n.motors[LEFT].status.fault == "wrong_direction",
          n.motors[LEFT].status.fault)
    check("the correct one does not",
          n.motors[RIGHT].status.fault == "", n.motors[RIGHT].status.fault)
    d, n = node(stall_timeout_s=0.0, check_direction=False)
    n.drive(120, 0)
    for _ in range(30):
        d.state.last_rx = time.time()
        n.poll()
        d.turn(left_deg=-4.0)
        time.sleep(0.03)
    check("the check can be switched off",
          n.motors[LEFT].status.fault == "")


def test_recovery_does_not_stutter():
    print("a long push comes back without stuttering")
    d, n = node(hold_max_step_deg=45.0, hold_max_corrections=4)
    d.idle()
    settle(n, d, 0.2)
    d.turn(left_deg=200.0)
    settle(n, d, 1.0)
    check("it walked home in capped steps", len(d.moves) == 5,
          str([round(m[1]) for m in d.moves]))
    check("progress did not count against hold_max_corrections",
          n.motors[LEFT].status.fault == "", n.motors[LEFT].status.fault)
    check("back on target", abs(n.motors[LEFT].status.error_deg) < 1.0,
          f"{n.motors[LEFT].status.error_deg:+.3f} deg")


def test_a_hand_still_turning_is_not_fought():
    print("motion is judged over a window, not one sample")
    d, n = node(hold_settle_s=0.3)
    d.idle()
    settle(n, d, 0.2)
    # A slow hand: less than the odometry's per-sample noise floor most polls,
    # which is exactly the case a per-sample "moving" flag gets wrong.
    for _ in range(14):
        d.turn(left_deg=1.5)
        d.state.last_rx = time.time()
        n.poll()
        drain(n)
        time.sleep(0.04)
    check("nothing was corrected mid-push", d.moves == [], str(d.moves))


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nall {len(tests)} groups passed")
