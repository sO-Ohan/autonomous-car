"""
Differential-drive kinematics and the measured duty/speed mapping.

Two separate jobs live here:

* turning a body twist (linear m/s, angular rad/s) into per-wheel motor duty,
  which needs the measured duty->speed curve; and
* integrating wheel travel into a pose, which needs the track width.

Neither depends on ROS, so a non-ROS controller can use them too.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


class DutyModel:
    """
    Measured open-loop duty -> shaft speed relationship.

        counts_per_sec = slope * duty + intercept

    Fitted from a duty sweep on this car (duty 60 -> 10,224 counts/s,
    duty 110 -> 58,170 counts/s). The fit crosses zero at duty 49.3, which is
    an independent match for the separately measured 45-50 deadband -- that
    agreement is the reason to trust it.

    This is FEEDFORWARD ONLY: the firmware has no velocity loop, so load,
    battery sag and the motor asymmetry all shift the real speed. Expect the
    commanded speed to be right to maybe 10-20% open loop. Close the loop with
    encoder feedback if you need better.
    """

    def __init__(self, slope: float = 959.0, intercept: float = -47316.0,
                 counts_per_metre: float = 201196.0,
                 min_duty: int = 58, max_duty: int = 255):
        self.slope = slope
        self.intercept = intercept
        self.counts_per_metre = counts_per_metre
        self.min_duty = min_duty
        self.max_duty = max_duty

    def speed_to_duty(self, mps: float) -> int:
        """Duty for a wanted wheel speed in m/s. 0 in gives a hard 0 out."""
        if abs(mps) < 1e-6:
            return 0
        counts_per_sec = abs(mps) * self.counts_per_metre
        duty = (counts_per_sec - self.intercept) / self.slope
        duty = max(self.min_duty, min(self.max_duty, duty))
        return int(round(math.copysign(duty, mps)))

    def duty_to_speed(self, duty: int) -> float:
        """Predicted wheel speed in m/s for a given duty."""
        if abs(duty) < self.min_duty:
            return 0.0
        counts_per_sec = self.slope * abs(duty) + self.intercept
        return math.copysign(max(0.0, counts_per_sec) / self.counts_per_metre,
                             duty)

    @property
    def max_speed(self) -> float:
        return self.duty_to_speed(self.max_duty)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0          # radians, CCW positive


class DifferentialOdometry:
    """
    Integrate per-wheel travel into a planar pose.

    `track_width` is the wheel centre-to-centre separation in metres. It only
    affects heading: straight-line distance is correct without it, but yaw is
    wrong in direct proportion to any error, so measure it rather than guess.
    """

    def __init__(self, track_width: float):
        if not track_width or track_width <= 0:
            raise ValueError(
                "track_width must be a positive measurement in metres -- "
                "see calibration.json, which ships with it unset")
        self.track_width = track_width
        self.pose = Pose()
        self._last = None       # (left_metres, right_metres)

    def reset(self, pose: Pose | None = None) -> None:
        self.pose = pose or Pose()
        self._last = None

    def update(self, left_metres: float, right_metres: float) -> Pose:
        """Fold in a new pair of absolute wheel odometers."""
        if self._last is None:
            self._last = (left_metres, right_metres)
            return self.pose

        dl = left_metres - self._last[0]
        dr = right_metres - self._last[1]
        self._last = (left_metres, right_metres)

        ds = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.track_width

        # Integrate about the arc midpoint rather than the start heading: for
        # a curved step, using the start heading biases the path outward.
        mid = self.pose.theta + dtheta / 2.0
        self.pose.x += ds * math.cos(mid)
        self.pose.y += ds * math.sin(mid)
        self.pose.theta = _wrap(self.pose.theta + dtheta)
        return self.pose


def twist_to_wheel_speeds(linear: float, angular: float,
                          track_width: float) -> tuple[float, float]:
    """Body twist (m/s, rad/s) -> (left, right) wheel speeds in m/s."""
    half = angular * track_width / 2.0
    return linear - half, linear + half


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a
