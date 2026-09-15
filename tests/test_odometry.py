#!/usr/bin/env python3
"""
Offline checks for WheelOdometryNode -- no hardware, no serial port.

A fake driver feeds synthetic counts, so the maths and the unintended-motion
logic can be checked against numbers worked out by hand.
"""
import math, sys, time, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from carbot.driver import BaseState, LEFT, RIGHT, load_calibration
from carbot.odometry import WheelOdometryNode

CAL = load_calibration()
CPR = CAL["combined"]["counts_per_wheel_rev"]      # counts per wheel rev
CPM_L = CAL["wheels"]["LEFT"]["counts_per_metre"]
CPM_R = CAL["wheels"]["RIGHT"]["counts_per_metre"]
CPR_L = CAL["wheels"]["LEFT"]["counts_per_wheel_rev"]
CPR_R = CAL["wheels"]["RIGHT"]["counts_per_wheel_rev"]
SIGN = CAL["encoder_sign"]


class FakeDriver:
    """Same surface as WheelDriver, backed by variables instead of a port."""
    def __init__(self):
        self.cal = CAL
        self.state = BaseState()
        self.state.connected = True
        self.state.last_rx = time.time()
        self.last_command = None
        self._command_until = 0.0

    # driver API used by the node
    def counts_per_metre(self, side=None):
        name = "LEFT" if side == LEFT else "RIGHT"
        return self.cal["wheels"][name]["counts_per_metre"]
    def command_active(self): return time.time() < self._command_until
    def idle_seconds(self): return max(0.0, time.time() - self._command_until)
    def zero(self): pass

    # test helpers
    def drive(self, left, right, hold=0.3):
        self.last_command = (time.time(), left, right)
        self._command_until = time.time() + hold
    def idle(self):
        self.last_command = (time.time(), 0, 0)
        self._command_until = time.time() - 10.0
    def turn(self, left_counts=0, right_counts=0):
        """Move the wheels forward by this many FORWARD counts."""
        self.state.left.counts += SIGN * left_counts
        self.state.right.counts += SIGN * right_counts
        self.state.last_rx = time.time()


def approx(a, b, tol):
    assert abs(a - b) < tol, f"{a} != {b} (tol {tol})"


def approx_deg(a, b, tol):
    """Compare wrapped angles: 359.9 and 0.0 are 0.1 apart, not 359.9."""
    d = (a - b + 180.0) % 360.0 - 180.0
    assert abs(d) < tol, f"{a} != {b} (tol {tol})"


def test_angle_and_distance():
    d = FakeDriver()
    n = WheelOdometryNode(d, track_width_m=0.30)
    n.update()
    d.drive(90, 90); d.turn(CPR, CPR)          # exactly one wheel revolution
    f = n.update()
    approx(f.left.revolutions, 1.0, 1e-3)
    approx_deg(f.left.wheel_angle_deg, 0.0, 0.5)   # wrapped back to the origin
    approx(f.left.wheel_angle_total_deg, 360.0, 0.5)   # per-wheel cal, not combined
    approx(f.left.distance_m, CAL["wheel_circumference_m"], 2e-3)
    approx(f.distance_m, CAL["wheel_circumference_m"], 2e-3)

    d.turn(CPR / 4, CPR / 4)                   # a further quarter turn
    f = n.update()
    approx_deg(f.left.wheel_angle_deg, 90.0, 0.5)
    approx(f.left.revolutions, 1.25, 1e-3)
    print("angle/distance ok")


def test_reverse_is_negative_and_path_grows():
    d = FakeDriver(); n = WheelOdometryNode(d, track_width_m=0.30)
    n.update(); d.drive(90, 90)
    d.turn(CPR, CPR); n.update()
    d.turn(-CPR, -CPR); f = n.update()
    approx(f.left.distance_m, 0.0, 2e-3)                     # back at the start
    approx(f.left.path_m, 2 * CAL["wheel_circumference_m"], 4e-3)  # but 2 revs travelled
    print("net vs path ok")


def test_pose_straight_and_spin():
    # Straight means equal DISTANCE, not equal counts: the two wheels differ by
    # 0.4% in counts per metre, which is exactly the sort of asymmetry odometry
    # exists to account for.
    d = FakeDriver(); n = WheelOdometryNode(d, track_width_m=0.30)
    n.update(); d.drive(90, 90)
    metre = 0.5
    d.turn(metre * CPM_L, metre * CPM_R); f = n.update()
    approx(f.theta_deg, 0.0, 0.05)
    approx(f.x, metre, 1e-3)
    # spin in place: wheels equal and opposite -> heading changes, x,y do not
    x0, y0 = f.x, f.y
    arc = 0.05
    d.turn(-arc * CPM_L, arc * CPM_R); f = n.update()
    expected = math.degrees(2 * arc / 0.30)
    approx(f.theta_deg, expected, 0.5)
    approx(f.x, x0, 2e-3); approx(f.y, y0, 2e-3)
    print(f"pose ok (spin gave {f.theta_deg:.1f} deg)")


def test_unintended_rotation_when_idle():
    d = FakeDriver(); n = WheelOdometryNode(d, track_width_m=0.30,
                                            idle_grace_s=0.0)
    d.idle(); n.update(); n.update()
    d.turn(left_counts=CPR / 4)                # someone pushes the car
    f = n.update()
    kinds = [e.kind for e in f.events]
    assert "unintended_wheel_rotation" in kinds, kinds
    ev = f.events[0]
    assert ev.side == "LEFT"
    approx(ev.magnitude_deg, 90.0, 1.0)
    assert not f.trustworthy
    print(f"idle-push detection ok: {ev.message}")


def test_no_event_when_commanded():
    d = FakeDriver(); n = WheelOdometryNode(d, track_width_m=0.30)
    n.update(); d.drive(90, 90)
    d.turn(CPR, CPR); f = n.update()
    assert f.events == [], f.events
    assert f.trustworthy
    print("commanded motion raises nothing ok")


def test_unintended_yaw_while_straight():
    # Rates are windowed, so this has to be played out over real time rather
    # than crammed into two back-to-back samples.
    d = FakeDriver(); n = WheelOdometryNode(d, track_width_m=0.30)
    n.update()
    events = []
    for _ in range(12):
        d.drive(90, 90)
        d.turn(0.04 * CPM_L, 0.005 * CPM_R)   # right wheel slipping badly
        f = n.update()
        events += f.events
        time.sleep(0.05)
    kinds = [e.kind for e in events]
    assert "unintended_yaw" in kinds, kinds
    print(f"slip/yaw detection ok: {events[0].message}")


def test_true_straight_raises_no_yaw_event():
    d = FakeDriver(); n = WheelOdometryNode(d, track_width_m=0.30)
    n.update()
    events = []
    for _ in range(12):
        d.drive(90, 90)
        d.turn(0.03 * CPM_L, 0.03 * CPM_R)    # equal distance, both wheels
        f = n.update()
        events += f.events
        time.sleep(0.05)
    assert events == [], events
    print("clean straight run stays quiet ok")


def test_counter_reset_is_not_integrated():
    d = FakeDriver(); n = WheelOdometryNode(d, track_width_m=0.30)
    n.update(); d.drive(90, 90); d.turn(CPR, CPR); n.update()
    d.state.left.counts = 0       # firmware `Z` under our feet
    d.state.right.counts = 0
    f = n.update()
    assert abs(f.left.speed_mps) < 5.0, f.left.speed_mps
    approx(f.left.distance_m, 0.0, 1e-6)
    print("counter reset re-origins instead of teleporting ok")


if __name__ == "__main__":
    for fn in [test_angle_and_distance, test_reverse_is_negative_and_path_grows,
               test_pose_straight_and_spin, test_unintended_rotation_when_idle,
               test_no_event_when_commanded, test_unintended_yaw_while_straight,
               test_true_straight_raises_no_yaw_event,
               test_counter_reset_is_not_integrated]:
        fn()
    print("\nall odometry tests passed")
