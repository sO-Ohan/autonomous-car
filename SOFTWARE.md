# Autonomous Car — Software

The Python side of the car. `SETUP.md` covers the hardware, the wiring and the
firmware protocol; this file covers the code that talks to it.

Last verified against hardware: 2026-08-24.

```
carbot/                 importable package, no ROS dependency anywhere
  driver.py             serial link, telemetry parsing, command tracking
  kinematics.py         duty<->speed model, differential pose integration
  odometry.py           wheel angle, distance, unintended-motion detection
  motor.py              motor properties, position hold, faults, e-stop
wheel_odom_node.py      odometry node / live readout / JSONL logger
motor_tui.py            drive both motors, watch the hold, edit properties
encoder_gui.py          per-wheel "is this encoder good?" dashboard
wheel_cal.py            gear-ratio calibration TUI, writes wheel_cal.csv
rotate.py, rot.py       drive one wheel a commanded angle
calibration.json        the measured numbers every tool reads
tests/test_odometry.py  offline checks, no hardware needed
tests/test_motor.py     offline checks for the motor node, no hardware
```

---

## 1. `carbot` package

### `WheelDriver` — the serial link

One background thread owns `/dev/ttyACM0`, parses the telemetry stream and
keeps `driver.state` current. Nothing else touches the port, so the same
object is safe to share between a control loop, a GUI and the odometry node.

```python
from carbot import WheelDriver

with WheelDriver("/dev/ttyACM0") as bot:
    bot.set_duty(90, 90)          # re-send faster than 3 Hz or the
    ...                           # firmware watchdog stops the motors
    print(bot.state.left_metres)
```

| Method | Does |
|---|---|
| `set_duty(l, r)` | open loop, −255..255 per side |
| `stop()` / `zero()` | `X` / `Z` |
| `rotate(side, deg, duty)` | closed-loop wheel rotation, returns `MoveResult` |
| `wait_for_telemetry(t)` | block until the board is actually streaming |
| `command_active()` | is a motion command in force right now? |
| `idle_seconds()` | how long since any command could move a wheel |

`state.left` / `state.right` carry counts, metres, `raw_angle` (the AS5600's
absolute 0–4095 shaft reading), AGC, magnitude and the STATUS bits, plus a
`magnet_ok` roll-up.

**Wheel direction (added 2026-08-25).** The two motors are mounted
mirror-imaged, so wiring both the same makes one of them run backwards — on
this car, the LEFT one. `calibration.json` carries a per-wheel `direction`
(`LEFT: -1`, `RIGHT: 1`) and the driver applies it to the duty it sends, the
degrees it asks a `G` for, and the counts it reads back. Everything above the
driver — odometry, the motor node, the tools — therefore sees **one forward, on
both wheels**, and `encoder_sign(side)` folds the mirror together with the
firmware's own negative-when-forward counters into a single number per side.

Fixing it here rather than in the firmware means no reflash, and one place to
be wrong. To check it: `python motor_tui.py`, press `w`, and both wheels should
roll forward with both angles climbing. If one does not, flip its `direction`
and re-run — the motor node also raises a `wrong_direction` fault on its own if
a wheel turns against the duty it was given.

**Command tracking (added 2026-08-24).** Every command that can move a wheel
now stamps `last_command` and a hold-until time: `set_duty` holds for the
firmware watchdog (300 ms), a `G` rotate holds until `# GODONE` arrives, `X`
releases immediately. That is what lets the odometry tell wanted motion from
unwanted — without it, "the wheel turned" carries no information about whether
anything is wrong.

### `kinematics.py`

`DutyModel` is the measured feed-forward curve, `counts_per_sec = 959·duty −
47316`. It is **feed-forward only** — there is no velocity loop in the
firmware, so expect 10–20% error under load.

`DifferentialOdometry` integrates two wheel odometers into a pose, about the
arc midpoint rather than the start heading. It requires a track width and
refuses to guess one.

### `odometry.py` — the odometry node

See §2.

### `motor.py` — the motor node

See §3.

---

## 2. Wheel odometry

```sh
cd ~/"autonomus car"
python wheel_odom_node.py                                  # live table
python wheel_odom_node.py /dev/ttyACM0 --track-width 0.30  # with body pose
python wheel_odom_node.py --json odom.jsonl --quiet        # log for SLAM
python wheel_odom_node.py --rate 50 --zero                 # zero counters first
```

It prints one line per sample and, on Ctrl-C, a summary plus every unintended
motion it saw. `--json` writes one `OdometryFrame` per line, which is the form
to hand to a SLAM front end or replay later.

### What it reports, per wheel

| Field | Meaning |
|---|---|
| `motor_angle_deg` | AS5600 absolute shaft angle, 0–360 |
| `wheel_angle_deg` | absolute wheel angle since the origin, 0–360 |
| `wheel_angle_total_deg` | the same unwrapped, grows without bound |
| `revolutions` | signed wheel revolutions since the origin |
| `distance_m` | signed **net** travel — forward then back cancels |
| `path_m` | **total** travel — forward then back adds up |
| `speed_mps`, `omega_dps` | windowed rates (see below) |
| `moving`, `healthy` | above the noise floor; encoder + magnet usable |

and for the body: `x`, `y`, `theta_deg`, `distance_m`, `path_m`,
`linear_mps`, `angular_dps`, `commanded`, `trustworthy`, `events`.

### Two senses of "absolute", and why only one is global

The AS5600 is absolute over one **motor** revolution. The gearbox is 18.5:1,
so one wheel turn is ~18.5 motor turns and the sensor cannot tell you which of
them the wheel is in. Therefore:

* `motor_angle_deg` is absolute in the physical sense. Power-cycle the board
  and it still reads the true shaft position.
* `wheel_angle_deg` is absolute **with respect to the origin** — wherever the
  wheel was when the counters were last zeroed. It is exact and drift-free
  across any number of turns, but its zero is a convention, not a landmark.

For SLAM that is fine: wheel angle is a relative measurement and is used as
one. It is worth being explicit about because "absolute encoder" invites the
assumption that the wheel angle survives a reboot, and it does not.

### Sign convention

Forward is positive, everywhere, in every field. The firmware's counters run
*negative* when driven forward (SETUP.md §7); `encoder_sign` from
`calibration.json` is applied once on the way in and never again.

### Rates are windowed, not per-sample

The firmware streams `ODO` at 10 Hz. Sampling faster than that — and the node
defaults to 20 Hz — means most samples see no new data, and dividing a real
count step by a near-zero `dt` reports a wheel doing thousands of deg/s.
Speeds are therefore accumulated over `velocity_window_s` (150 ms) and yaw
over `yaw_window_s` (250 ms). This is the reason `speed_mps` updates a few
times a second while the angles update every sample.

### Detecting rotation nobody asked for

Three checks, all comparing what moved against what was commanded:

| Event kind | Fires when | Typical cause |
|---|---|---|
| `unintended_wheel_rotation` | a wheel turns more than `idle_tolerance_deg` (1°) while no command has been in force for `idle_grace_s` (0.6 s) | pushed by hand, rolling on a slope, coasting long after the motors were cut |
| `unintended_yaw` | heading changes faster than `yaw_tolerance_dps` (8°/s) while both sides are commanded to the *same* duty | one tyre slipping, a dragging brake, a carpet edge |
| `wheel_disagreement` | as above but no track width is set, so it is reported in m/s instead of °/s | same, but unquantified in yaw |

A commanded turn is never flagged: the check only runs when both sides were
asked for the same thing, so asymmetric duty is treated as intent.

Once any event fires, `trustworthy` goes `False` and stays there until
`reset()`. That flag is the useful output for SLAM — it marks the point after
which dead reckoning stopped being evidence.

### Counter resets are absorbed, not integrated

If the counters jump by more than 200,000 counts in one sample (a `Z` issued
by another tool, or a serial reconnect), the node re-origins there instead of
folding an impossible step into the pose. Top measured speed is ~58,000
counts/s, so nothing real can clear that bar.

### Track width is still unmeasured

`calibration.json` ships `track_width_m: null`. Without it:

* every per-wheel number is exact — angle, distance, revolutions, speed;
* body `distance_m` and `path_m` are exact, since they are the wheel mean;
* `x`, `y`, `theta_deg` are **withheld** (`have_pose: false`), because heading
  error is directly proportional to track-width error and a guess would be
  fiction dressed as a measurement;
* the yaw slip check degrades to `wheel_disagreement` in m/s.

Measure the wheel centre-to-centre separation once, then either pass
`--track-width` or fill in the field. To sanity-check it: command a spin in
place through 360°, and if the car under- or over-rotates, the track width is
wrong in that proportion.

### Using it from code

```python
from carbot import WheelDriver, WheelOdometryNode

with WheelDriver("/dev/ttyACM0") as bot:
    odom = WheelOdometryNode(bot, track_width_m=0.30)
    odom.on_event = lambda e: print("UNINTENDED:", e.message)
    odom.on_frame = lambda f: publish(f)      # your SLAM sink
    odom.spin(rate_hz=20)
```

### Wiring it to ROS 2 later

There is no ROS import in this package, which is deliberate: the node runs
today, on a machine with no ROS installed, and becomes a ROS node by wrapping
rather than by rewriting. An `OdometryFrame` maps field-for-field:

| Frame field | ROS |
|---|---|
| `x`, `y`, `theta_deg` | `nav_msgs/Odometry` pose (yaw → quaternion) |
| `linear_mps`, `angular_dps` | `nav_msgs/Odometry` twist |
| `left/right.wheel_angle_total_deg` | `sensor_msgs/JointState.position` (→ radians) |
| `left/right.omega_dps` | `sensor_msgs/JointState.velocity` |
| `events`, `trustworthy` | `diagnostic_msgs/DiagnosticArray` |

`ros2_ws/src/carbot_driver/` is scaffolded but empty; ROS 2 is not installed
on this machine (no `/opt/ros`, no `rclpy`), so nothing there is built yet.

---

## 3. Motor node — properties, position hold, faults

`WheelDriver` moves a motor and the odometry says what moved. Neither decides
what a motor is *allowed* to do, or what to do about motion nobody asked for —
odometry only reports it. `carbot/motor.py` closes that gap.

```python
from carbot import WheelDriver, MotorNode, MotorConfig, LEFT

with WheelDriver("/dev/ttyACM0") as bot:
    node = MotorNode(bot, config=MotorConfig(hold_tolerance_deg=1.5))
    node.on_event = lambda e: print(e.message)
    node.start()
    node.drive(90, 90)          # re-sent for you, watchdog fed
    node.rotate(LEFT, 90)       # closed loop, hold target follows the move
    node.stop()
```

While it is running, **nothing else may talk to the driver**. It owns the
open-loop duty stream (which must be re-sent faster than the firmware's 300 ms
watchdog) and it serialises every closed-loop move, because the firmware has a
single rotate state machine and a `D` sent mid-rotate preempts it.

### 3.1 Properties, critical and optional

Every motor carries a `MotorConfig`. The split is not decoration: `start()`
refuses to run if a **critical** property is out of range, while an optional
one is free to be anything the range allows. `PROPERTIES` carries the type,
range, unit and reason for each — it is what the TUI editor draws.

**Critical** — wrong values here damage the bot, run it away, or make it hold
position blind:

| Property | Default | Does |
|---|---|---|
| `enabled` | on | master switch; a disabled motor refuses every drive, rotate and correction |
| `hold_position` | on | station keeping (§3.2) |
| `hold_tolerance_deg` | 2.0° | hold deadband; below the firmware's own ~0.3° trim window it hunts forever |
| `hold_settle_s` | 0.4 s | how long the disturbance must be over before correcting — this is what stops the motor fighting a hand still turning it |
| `hold_duty` | 90 | duty for a recovery move; a standstill needs more torque to break away than a wheel already turning |
| `hold_max_corrections` | 8 | attempts before `hold_failed`; resets once back inside tolerance |
| `min_duty` | 58 | measured deadband floor; anything under it only heats the motor |
| `max_duty` | 200 | hard ceiling on any duty from any source |
| `stall_timeout_s` | 1.0 s | commanded above the deadband but not turning this long → stall fault; 0 disables |
| `estop_on_stall` | on | a stalled motor e-stops the whole base — on a two-wheel base, driving one side into a locked wheel is a spin |
| `check_direction` | on | fault if a wheel turns against the duty it was given — a mirror-wired motor makes odometry, pose and hold wrong in the same breath, and it is invisible until something is measured against it |
| `require_encoder` | on | fault when the encoder goes silent; holding position with no feedback is an open-loop motor with a plan |

**Optional** — tuning and comfort:

| Property | Default | Does |
|---|---|---|
| `hold_rearm_s` | 0.6 s | idle time before the target is latched, so coast-down is not recorded as the target |
| `hold_retry_s` | 0.3 s | back-off after an attempt that got **no closer**; an attempt that made progress is followed immediately |
| `hold_giveup_deg` | 0 (off) | past this distance, assume the move was deliberate and stop fighting |
| `hold_max_step_deg` | 360° | largest single recovery move; below the distance of a typical hand push this turns one smooth return into several |
| `soft_limit_deg` | 0 (off) | fault if the wheel travels further than this from the odometry origin |
| `slew_duty_per_s` | 0 (off) | ramp rate for open-loop duty changes |
| `arm_on_start` | on | latch a target at start-up rather than after the first commanded move |
| `clear_firmware_reference` | on | see §3.3 |
| `label` | — | name shown in the TUI |

`cfg.set(name, value)` coerces and clamps to the declared range, `cfg.bump()`
nudges one step (and toggles a bool), `cfg.validate()` returns the critical
properties that are out of range.

### 3.2 Position hold — the wheel puts itself back

With `hold_position` on, a motor latches its wheel angle once the base has
been idle for `hold_rearm_s`, and drives back to that angle if anything else
moves the wheel: a hand, a slope, a knock. That is the difference between *the
odometry noticed you pushed the wheel* and *the wheel put itself back*.

The recovery is **not** a Python P-loop. The firmware's `G` rotate already
drives, brakes, settles and trims to ±60 counts (~0.3° of wheel) — far better
than anything reachable over a 10 Hz telemetry link with a duty deadband at
58. So a recovery is one `G` for the measured error, and the node only decides
*when* to fire it:

* wait `hold_settle_s` after the disturbance **stops** — while the wheel is
  still turning the settle clock keeps restarting, so the motor never fights a
  hand that has not let go;
* cap each attempt at `hold_max_step_deg`;
* stop after `hold_max_corrections` and raise `hold_failed` rather than
  hunting forever against something physically blocking the wheel.

Two details decide whether that feels like a return or a stutter, and both were
wrong in the first cut:

**"Still moving" is judged over a window, not one sample.** `WheelOdom.moving`
compares one sample against the last, but the board streams `ODO` at 10 Hz
while this loop runs at 25 Hz — so most samples see no new data and report a
turning wheel as stopped. The node timestamps the last real change instead and
calls the wheel moving for 250 ms after it. Without that, a slow hand push gets
corrected *while the hand is still on it*, which is exactly the juddering it
was supposed to prevent.

**A move of our own is not a disturbance.** The firmware brakes and settles
before it reports, so the wheel is already stopped when the result arrives. The
node ignores the odometry catching up for 300 ms afterwards; otherwise every
step of a multi-step recovery pays the settle wait again and the wheel walks
home in visible lurches. Alongside that, an attempt that got closer is followed
immediately and does not count against `hold_max_corrections` — only an attempt
that made *no* progress backs off by `hold_retry_s`.

A commanded `rotate()` carries the target with it — the wheel defends where it
was *asked* to end up, not where it started. `hold_here()`, `release()` and
`set_hold()` control it by hand; `zero()` re-latches both targets at the new
origin.

The hold target is an angle measured from the odometry origin, so when the
odometry re-origins (a `Z`, or an absorbed counter jump) every such angle
becomes meaningless at that instant. `WheelOdometryNode.origin_counts()` is
public for exactly this reason: the motor node watches it and re-latches
instead of driving back to a target that moved underneath it.

### 3.3 Long moves are split, because the firmware has a clock

`goService()` abandons a rotate after **30 s** wherever it has got to and
reports `timeout` (`GO_TIMEOUT_MS` in `main.cpp`). At duty 90 a wheel turns
about 185 °/s, so anything past roughly 5,500° cannot finish inside it. Ask for
20 turns — 7,200° — and the board drives for 30 s, gets about three quarters of
the way, and stops. Nothing about the maths is wrong; it ran out of clock.

`MotorNode.rotate()` therefore splits any command into chunks that fit, sized
from the measured duty/speed curve in `calibration.json` with the firmware's
taper over the last fifth paid for and a margin left for a low battery:

| Duty | Longest single `G` |
|---|---|
| 60 | 728° |
| 90 | 2,775° |
| 120 | 4,823° |
| 200 | 10,283° |

7,200° at duty 90 becomes three moves and a `move_split` event. Splitting costs
no accuracy: the chunks carry the firmware's ideal target forward between them,
so each one corrects the last one's residual (SETUP.md §4a). `python
motor_tui.py` shows the chunks going past in the event log.

If 20 turns still lands short after this, the remaining error is the gear
ratio, not the timeout — 7,200° is 20 wheel revolutions, and the 0.4% spread
between `counts_per_degree` for LEFT and RIGHT is ±29° over that distance.
`wheel_cal.py` is the fix for that, not this node.

### 3.4 Why a recovery clears the firmware's reference first

The firmware carries a per-side "ideal" target forward between rotates so a run
of identical moves cannot accumulate error (SETUP.md §4a). That is right for a
sequence of commanded moves and wrong for a recovery: a hand nudge under 45°
leaves the ideal intact, so a corrective `G -e` would be measured from the
ideal rather than from the wheel and overshoot by `e`. It converges — the
second correction lands it — but it takes two moves to do one move's work.

The firmware drops the reference on any `D` with a non-zero duty, so the node
sends a duty of **1** — an order of magnitude under the 45–50 deadband,
incapable of turning the wheel — followed by an `X` to clear the drive
watchdog that `D` just armed, which would otherwise fire mid-rotate. Set
`clear_firmware_reference` off to skip it.

**Only a recovery does this.** A commanded move — including every chunk after
the first of a long one — leaves the reference alone, because carrying it
forward is precisely what stops a run of moves accumulating error. Clearing it
there would undo §4a and put the drift back.

### 3.5 Faults and the e-stop

| Fault | Raised when |
|---|---|
| `stall` | commanded above `min_duty` but the encoder has not moved for `stall_timeout_s` |
| `soft_limit` | the wheel is past `±soft_limit_deg` from the odometry origin |
| `encoder_lost` | telemetry is stale or the encoder stopped answering (clears itself when it comes back) |
| `hold_failed` | out of correction attempts, or past `hold_giveup_deg` |
| `wrong_direction` | the wheel turned against its duty for more than 0.5 s — its `direction` in `calibration.json` is wrong |

A faulted motor is cut, its hold target dropped, and it stays that way until
`clear_faults()`. `estop()` cuts both, aborts any move in flight and refuses
every command until `clear_estop()`. Everything worth knowing is also pushed
to `on_event` as a `MotorEvent`, and kept in `node.events`.

### 3.6 The TUI

```sh
cd ~/"autonomus car"
python motor_tui.py                                  # both motors, hold on
python motor_tui.py --no-hold --duty 70
python motor_tui.py --track-width 0.30               # adds body pose
```

Driving is **hold-to-move**: a key held down keeps the wheels turning because
the terminal repeats it, and letting go stops the base within 700 ms. There is
no latched drive state to forget about.

That 700 ms is not arbitrary, and getting it wrong is why the controls did
nothing at first. A held key sends one character immediately and the next only
after the keyboard's **repeat delay** — 500 ms on a stock desktop, `xset q`,
"auto repeat delay". A drive command that expires sooner than that drives,
stops, and only then stutters back in, which reads as the keys not working. It
only ever has to bridge that first gap; after it, repeats arrive every ~30 ms.
If your delay is longer, pass `--key-hold`. The header shows a green `DRIVE`
badge whenever a duty is actually going to the motors.

| Key | Does |
|---|---|
| `w` `a` `s` `d` / arrows | drive — hold the key, release to stop |
| `space` | stop now |
| `E` | e-stop / clear e-stop |
| `[` `]` | duty down / up |
| `<` `>` | rotate step size down / up |
| `j` `l` | closed-loop rotate the focused motor −step / +step |
| `h` | hold on/off · `m` latch the target here · `r` release it |
| `n` | enable / disable the focused motor |
| `c` | clear faults and the e-stop · `z` zero the encoders and re-latch |
| `tab` / `1` `2` `3` | focus LEFT, RIGHT or BOTH |
| `p` | property editor — up/down select, left/right change, enter toggles a switch, `0` restores the default |
| `?` `x` | help / quit |

Each motor panel shows its mode, wheel angle, hold target and error (with a
centre-zero bar against the tolerance), speed and duty, correction and recovery
counts, encoder health and the last move's result. Everything the editor
changes applies to whichever side is focused, so `tab` then `p` edits one motor
without touching the other.

---

## 4. Tests

```sh
cd ~/"autonomus car"
python tests/test_odometry.py
python tests/test_motor.py
```

Seven checks against a fake driver — no serial port, no hardware. They cover
one exact wheel revolution landing back on 0°, net-versus-path distance, a
straight run and a spin in place, a hand push while idle, a commanded move
raising nothing, a slipping wheel raising `unintended_yaw`, a clean straight
run staying quiet, and a mid-run counter reset not teleporting the pose.

`test_motor.py` adds twenty-four groups against a fake driver whose `rotate()`
moves the synthetic counters the way the firmware would, so the hold loop can
be driven all the way from *a hand turned the wheel* to *the wheel is back on
target*. They cover the property table and its clamping, arming when idle, a
hand push returning to within a degree, a nudge inside tolerance being left
alone, not correcting while the wheel is still being turned, a 200° push
walking home in capped steps, a blocked wheel giving up instead of hunting,
stall / soft-limit / encoder-loss faults, the e-stop, duty gating, a commanded
rotate carrying the target with it, drive commands expiring on their own, and
no `D` being written while a rotate is in flight. Added 2026-08-25: 7,200°
splitting into three chunks that each fit the firmware's clock and adding back
up to the command, a 1,800° move landing end to end, the reference being
cleared for recoveries but not commanded moves, a backwards wheel raising
`wrong_direction`, a 200° push walking home in five back-to-back steps without
tripping `hold_max_corrections`, and a slow hand push — below the odometry's
per-sample noise floor, the case a one-sample `moving` flag gets wrong — not
being corrected mid-push.

One thing the tests make concrete: "straight" means equal **distance**, not
equal counts. LEFT and RIGHT differ by 0.4% in counts per metre, so feeding
both wheels the same count step is a slow curve, not a straight line — 0.3° of
heading per wheel revolution. That asymmetry is real, it is in
`calibration.json`, and accounting for it is most of what odometry is for.

---

## 4a. Closed-loop drive (2026-08-26)

Before this, the motors were open loop end to end: the joystick's `y` value
became a PWM duty and that was the whole story. The encoders were read, logged
and reported, but **nothing ever acted on them**. That is why the car curved,
why it responded slowly, and why the two wheels never stayed in step.

### Why it was slow

Encoder sampling lived in `loop()`, sharing the single ESP32-C3 core with WiFi,
the async web server and about 30 lines/s of USB serial. The sampling *interval*
was therefore whatever was left over — fine for reporting a distance, useless as
the input to a control law, which needs a **constant** period far more than it
needs a fast one. On top of that, every 50 ms the loop did six blocking I²C
transactions just to read the magnet-health registers, which no controller
would ever consume.

### What it does now

A dedicated FreeRTOS task at **500 Hz** owns the I²C bus outright and is the
only thing in the firmware that ever touches it. It runs off `vTaskDelayUntil`,
so its period is fixed no matter what WiFi and the web server are doing.
`loop()` has been demoted to command parsing and printing.

Three things made the bus cheap enough for that:

- **Burst reads.** `STATUS`, `ANGLE_HI` and `ANGLE_LO` are consecutive
  registers (0x0B/0x0C/0x0D), so one three-byte read gets the magnet status
  free with every angle sample. Same trick for AGC/MAGNITUDE at 0x1A.
- **Cached mux channel.** The `muxDisableAll()` after every sample cycle was a
  whole transaction spent achieving nothing; it is gone.
- **Magnet registers demoted.** They are a bench-tuning readout, never a
  control input, so they are polled at 5 Hz and skipped entirely while the
  wheels are turning. The `T` line still *emits* at 20 Hz from the cached
  values, so `encoder_tune.py` feels exactly as live as before.

Measured: **499.7 Hz against a 500 Hz target, 0 I²C errors, 6 overruns in
21,475 ticks (0.03%).**

### The controller

Two-level cascade, per wheel:

```
commanded velocity -> integrated into a POSITION REFERENCE
position error (reference - actual) -> trims the velocity setpoint
velocity error -> PI on top of a measured feedforward -> PWM duty
```

**Driving straight is a consequence of the structure, not a special case.**
Both wheels track their own position reference; command both the same velocity
and both references advance identically, so any difference between the wheels
is a position error the loop actively removes. A velocity-only loop cannot do
this — it nulls speed error but lets *distance* error integrate up forever,
which is exactly how a robot drives a slow arc while both wheels report the
correct speed.

The feedforward comes straight out of `calibration.json`'s measured
`duty_to_counts_per_sec` fit, inverted into m/s. Getting it roughly right is
what lets the PI gains stay small, which is what keeps the loop stable. The
integrator only accumulates while there is duty headroom, so it cannot wind up
against a wheel that physically cannot go faster and then lurch when the load
clears.

### "Straight no matter what" — the reference governor

The remaining failure mode is one wheel physically unable to keep up: a carpet
edge, a stalled gearbox, a hand on a tyre. Left alone, the position loop would
wind the reference away from a wheel that cannot follow it.

So the reference advance is **governed**. Once either wheel falls
`POS_ERR_SOFT` (20 mm) behind, *both* references slow down together; past
`POS_ERR_HARD` (60 mm) they stop entirely. The pair stays locked to each other
even when the base as a whole is being held back, and resumes in step once the
obstruction clears. The car goes slower than asked, but it does not turn.

Verified by forcing the thresholds down to 1 mm/3 mm and commanding a fast
move: `refScale` fell to 0.250, tracking error clamped at 2.50 mm under the
3.00 mm hard stop, and **skew stayed at 0.23 mm while being governed.**

### Position hold

The same code path with a zero velocity command. The reference stops advancing
and the loop drives whatever error appears back to zero, so a shoved wheel gets
pushed back where it was. It is exempt from the 300 ms command watchdog — it is
a deliberate "stay put" and nobody is obliged to keep talking to it — but it
expires after about two minutes so a forgotten hold cannot cook a stalled motor.

### Measured results (wheels on a stand, free-spinning)

| | open loop `D -110 110` | closed loop `V 0.20 0.20` |
|---|---|---|
| skew over ~0.85 m | **7.5 mm (0.86%)** | **0.05 mm (0.007%)** |
| worst skew during run | — | 0.79 mm |
| velocity vs command | n/a | 0.1996 / 0.1997 vs 0.2000 |
| rise to 90% of setpoint | — | **40 ms** |

Velocity accuracy, measured off the **board's own millisecond clock** in
steady state (1.5 s after the command, so spin-up is excluded — measuring from
the host over a short window instead makes this look like a 3.8% error that is
entirely the acceleration transient):

| commanded | measured | error | ripple | tracking error L/R |
|---|---|---|---|---|
| 0.05 m/s | 0.0499 | −0.18% | ±0.0018 | −0.05 / −0.03 mm |
| 0.15 m/s | 0.1498 | −0.11% | ±0.0042 | −0.16 / −0.06 mm |
| 0.25 m/s | 0.2498 | −0.07% | ±0.0092 | +0.05 / −0.08 mm |
| 0.40 m/s | 0.4002 | +0.05% | ±0.0102 | +0.02 / −0.05 mm |
| 0.60 m/s | 0.5999 | −0.02% | ±0.0104 | +0.08 / +0.02 mm |

**Within 0.2% across a 12:1 speed range**, with every wheel inside 0.2 mm of
where the controller intends it to be.

Ten-second straight run at 0.25 m/s: both wheels 2475.8 mm, final skew
**0.090 mm (0.0036% of distance)**, heading drift 0.14°. An eight-second run at
0.30 m/s gave 1.49 mm over 2.38 m. Low speed holds up too — 0.03 m/s commanded
gives 0.0294/0.0296 actual at duty 47–49, *below* the open-loop deadband of 50,
because the integrator finds the sticking point on its own.

Position hold against a 4.2 mm coast-down disturbance: settled to **0.21 / 0.36
mm** residual and stayed there, on a standing duty of ±3.

### Two bugs found and fixed during testing

- **Position hold died after 300 ms.** `ctrlEngage()` marked every mode as
  `driving`, so the velocity watchdog cancelled the hold almost immediately —
  it corrected a disturbance for 300 ms and then let go. Only `CTRL_VEL` arms
  the watchdog now.
- **Zeroing injected a phantom pose step.** `zeroEncoders()` reset the counters
  and the pose, but the control task's velocity and pose integrators kept their
  own differencing state. When the counters snapped to zero those saw a
  metre-sized negative delta and integrated it as real travel, and it spiked
  the velocity filter for one tick. A `ctrlResync` flag now re-primes them.

### What still needs a floor test

Everything above was measured with the wheels **up on a stand, free-spinning.**
Two things specifically need re-checking on the ground:

- **Straightness under real load.** Free-spinning removes exactly the load
  asymmetry the loop exists to reject, so the open-loop 0.86% figure is the
  *best* case for open loop — the earlier spin test showed 8.5% between the
  sides at equal duty. The closed-loop numbers should hold, but confirm.
- **Yaw is not independently validated.** A 1.0 rad/s spin for 2 s integrated
  to 115.8° against an expected 114.6°, but the *same* placeholder track width
  is used both to mix the command and to integrate the pose, so that agreement
  is self-consistency, not accuracy. Measure the track width, set it with `B`,
  and put it in `calibration.json` before trusting any heading number.
- **The `G` rotate primitive.** It now reports `capped` with 2–3° residual on a
  free-spinning wheel, because the trim pulses overshoot with no friction to
  stop them. The residual **bounces rather than accumulating** (0.11, −2.11,
  −2.88, −0.60, −1.86), so the anti-accumulation logic from 2026-08-24 is still
  working. Re-verify on the floor, where it previously landed inside tolerance.

## 5. Changes

### 2026-08-26 — closed-loop drive, sync, and position hold

- **Motors are now closed loop.** 500 Hz FreeRTOS control task, cascade
  position-over-velocity controller with a measured feedforward. See §4a.
- **Both wheels are synchronised by construction** — each tracks its own
  position reference, so straightness is structural rather than a trim. Skew
  fell from 7.5 mm to 0.05 mm over 0.85 m, a 123× improvement.
- **Reference governor** keeps the pair locked when one wheel is held back.
- **Position hold** (`H`) resists being pushed; 4.2 mm disturbance settles to
  0.2–0.4 mm.
- **Encoder sampling moved off `loop()`** and onto a fixed-period task; burst
  reads and demoted magnet polling cut the bus cost. 499.7 Hz measured,
  0 I²C errors.
- **All distance constants now derive from one gear ratio + wheel diameter**
  (`N` command). Per-wheel ratios are gone — they were measurement error and
  baking them in caused a permanent curve.
- **New `gear_check.py`** measures the gear ratio under power, the one constant
  the encoder physically cannot see.
- **New `odom.py`** live odometry and straightness dashboard.
- **New `FB` telemetry line** (50 Hz, forward-positive, SI) for odometry.
  `ODO` and `T` unchanged, so every existing tool still works.
- Fixed: position hold cancelled by the velocity watchdog after 300 ms.
- Fixed: `Z` injected a phantom metre-sized step into the pose.

### 2026-08-26 (later) — radios off, magnetometer, I²C scan

- **WiFi and Bluetooth off at boot** (`A 1` brings WiFi back at runtime).
  Removed all control-loop tick overruns: worst tick 3313 → 1537 µs, 6
  overruns → 0, and 63 kB of heap recovered. See SETUP.md §5a.
- **QMC5883L magnetometer** read at 100 Hz, new `MAG` telemetry line. Measured
  immune to the motor magnets and motor current — heading moves < 0.3° across
  a full wheel rotation and a 6× current range, under its own ±0.8° noise.
- **New `I` command** scans the I²C bus and every mux channel, naming known
  parts. It is what identified the "HMC5883L" as a QMC5883L at 0x0D.
- Fixed: the WebSocket odometry push was **dropped during the closed-loop
  rewrite**, so the phone GUI connected but displayed `--` forever. Restored,
  and it now carries wheel speeds as well.

### 2026-08-25 (later) — four fixes from the first hardware run

* **LEFT motor ran backwards.** The two motors are mounted mirror-imaged and
  were wired the same. `calibration.json` gains a per-wheel `direction`
  (`LEFT: -1`), applied in `WheelDriver` to duty, `G` degrees, `GODONE` replies
  and counts, so forward is forward on both wheels everywhere above it.
  `WheelDriver.encoder_sign(side)` replaces the single global sign, and
  `WheelOdometryNode` now takes its sign per side.
* **A `check_direction` critical property**, faulting `wrong_direction` if a
  wheel turns against its duty for more than 0.5 s — so a wrong `direction`
  announces itself instead of silently corrupting the odometry and the hold.
* **20 turns came up short.** The firmware abandons a rotate after 30 s;
  7,200° at duty 90 needs 39. `MotorNode.rotate()` now splits a command into
  chunks sized from the duty/speed curve, which carry the firmware's ideal
  target forward so the split costs no accuracy (§3.3).
* **The hold stuttered when a wheel was pushed by hand.** Three causes:
  `moving` was judged from one sample against a 10 Hz stream, so a slow push
  was corrected while the hand was still on it; the node's own moves were then
  read back as fresh disturbances, making every step pay the settle wait again;
  and `hold_retry_s` backed off after *every* attempt, including ones that made
  progress. All three fixed (§3.2), and the defaults moved with them —
  `hold_duty` 70 → 90, `hold_max_step_deg` 90 → 360, `hold_retry_s` 1.5 → 0.3.
* **`w`/`a`/`s`/`d` did nothing.** A drive command lived 350 ms, shorter than
  the 500 ms keyboard repeat delay, so a held key drove then stopped before the
  first repeat arrived. Now 700 ms, with `--key-hold` to tune it and a `DRIVE`
  badge in the header (§3.6).
* **A regression in the first cut**, found while fixing the above: recoveries
  and commanded moves both cleared the firmware's carried-forward rotate
  target, which undid the anti-drift fix in SETUP.md §4a. Only recoveries clear
  it now (§3.4).

### 2026-08-25

* **`carbot/motor.py`** — new. `MotorConfig` (11 critical + 9 optional
  properties with ranges and reasons), `MotorNode` (owns the duty stream,
  serialises closed-loop moves, position hold, stall / soft-limit /
  encoder-loss faults, e-stop), `Motor`, `MotorStatus`, `MotorEvent`.
* **`motor_tui.py`** — new. Drive both motors, watch the hold defend its
  target, edit every property live.
* **`carbot/odometry.py`** — `origin_counts()` made public, so anything
  holding a position target can tell a re-origin from motion.
* **`carbot/__init__.py`** — exports the motor types; version 1.1.0.
* **`tests/test_motor.py`** — new, offline.

Verified on hardware: the node was run against the board with hold armed on
both wheels for 5 s. Both latched at 0.00°, both reported `holding` with zero
error and healthy encoders throughout, and neither issued a correction — which
is the thing to check, because a hold that twitches when nothing has touched it
is worse than no hold at all.

**Not verified on hardware:** the recovery itself. Confirming it means pushing
a wheel by hand and watching it come back, and blocking a wheel to see
`hold_failed` fire. Both are exercised against the fake driver, and both are
worth doing before leaving the bot holding position unattended. `stall` has the
same status.

### 2026-08-24

* **`carbot/odometry.py`** — new. Absolute wheel angle, motor shaft angle,
  net and total distance, windowed speeds, differential pose, and the three
  unintended-motion checks.
* **`wheel_odom_node.py`** — new. CLI node: live readout, JSONL logging,
  Ctrl-C summary of everything that moved unbidden.
* **`carbot/driver.py`** — `WheelState.raw_angle` now parsed from the `T,`
  stream (it was being discarded); command tracking added
  (`last_command`, `command_active()`, `idle_seconds()`), which is what makes
  "unintended" a decidable question.
* **`carbot/__init__.py`** — exports the odometry types.
* **`tests/test_odometry.py`** — new, offline.

Verified on hardware: the node was run against the board at 20 Hz and read
live `motor_angle_deg` from both AS5600s. Both wheels report
`healthy: false`, which is correct and expected — AGC is pegged at 128 on both
encoders, the known airgap problem in SETUP.md §7. Fix the magnets and that
flag clears; until then the odometry works but has no margin.

**Not verified on hardware:** the unintended-motion detectors have only been
exercised against the fake driver, because confirming them for real means
pushing the car by hand while it idles, and slipping a wheel under command.
Both are worth doing before trusting the flags in a SLAM run.
