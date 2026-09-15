# Autonomous Car — Setup and As-Built Wiring

ESP32-C3 Super Mini, two BTS7960 half-bridges, two AS5600 magnetic encoders
behind a PCA9548A I²C switch.

This file is the hardware, the wiring and the firmware protocol. The Python
side — the `carbot` package, the odometry node and the tools — is documented
in `SOFTWARE.md`.

Firmware lives in `~/drive-harness-diag/c3-webgui`.
Last verified against hardware: 2026-08-23.

---

## 1. Mechanical

| Property | Value | Source |
|---|---|---|
| Wheel diameter | **120 mm** | given |
| Wheel circumference | **0.376991 m** | π × 0.120 |
| Encoder position | **on the motor shaft, before the gearbox** | given |
| Gear ratio | **18.52 : 1** (LEFT 18.524, RIGHT 18.447) | measured, `wheel_cal.py` |

The encoder being on the motor side of the gearbox is the single most
important fact here, because every counts-to-distance number depends on it.
One AS5600 revolution is one *motor* revolution, not one wheel revolution.

**The two motors are mounted mirror-imaged and wired the same**, so the LEFT
one runs backwards for a "forward" command — and its encoder, being on that
motor's shaft, counts backwards with it. This is corrected in software rather
than by rewiring or reflashing: `calibration.json` carries `wheels.LEFT.direction
= -1`, and `WheelDriver` applies it to every duty sent, every `G` commanded and
every count read. Nothing above the driver has to know. See SOFTWARE.md §1.

---

## 2. Pinout (as flashed)

| Subsystem | Signal | ESP32-C3 pin |
|---|---|---|
| LEFT BTS7960 | RPWM | GPIO 1 |
| LEFT BTS7960 | LPWM | GPIO 0 |
| RIGHT BTS7960 | RPWM | GPIO 4 |
| RIGHT BTS7960 | LPWM | GPIO 3 |
| I²C | SDA | GPIO 6 |
| I²C | SCL | GPIO 7 |
| USB serial | — | native USB, enumerates as `/dev/ttyACM0` |

Left free: GPIO 2 (boot strap), 20/21 (UART TX/RX).

### I²C bus

| Device | Address | Notes |
|---|---|---|
| PCA9548A (HW-617) I²C switch | 0x70 | verified ACK |
| AS5600 LEFT | 0x36 | behind mux **channel 6** (mask `0x40`) |
| AS5600 RIGHT | 0x36 | behind mux **channel 3** (mask `0x08`) |

Both encoders share address 0x36, which is the whole reason the mux exists.
Exactly one channel is enabled at a time, and all channels are disabled
between sample sweeps.

### Deliberate deviations

- **R_EN / L_EN strapped to VCC.** Both drivers are permanently enabled;
  there is no software e-stop. Zero PWM duty is what keeps the motors still.
  The BTS7960 inputs have internal pull-downs, which is what keeps the wheels
  still while the pins float during ESP boot.
- **W5500 Ethernet is not wired.** See the warning in §7 before adding it.

---

## 3. Counts, and how to turn them into distance

The AS5600 is a 12-bit absolute encoder: **4096 counts per motor revolution.**
Firmware unwraps that into an `int64_t`, so travel is unbounded across
revolutions.

With the encoder before the gearbox, for a gear ratio `N`:

```
counts per wheel revolution = 4096 × N
counts per metre            = 4096 × N / 0.376991
                            = 10864.7 × N
```

So at an assumed N = 25, that is **271,600 counts/m** — versus 10,865 if the
encoder were on the wheel. The gearbox is buying roughly N× the resolution,
which is exactly why this placement is worth it.

**Measured 2026-08-23**, and now set in the firmware:

| | value |
|---|---|
| gear ratio N | **18.52 : 1** |
| counts per wheel rev | 75,850 |
| counts per degree of wheel | 210.76 (LEFT), 209.88 (RIGHT) |
| `COUNTS_PER_METRE` | **201,196** |

LEFT and RIGHT agree to 0.4%, which is the main evidence the number is real.
The machine-readable copy lives in `calibration.json`, which the tools read.

### The gear ratio is ONE constant, and the encoder cannot measure it

Two things about this number, both of which matter more than they look.

**One constant, both wheels.** `calibration.json` lists per-wheel ratios (LEFT
18.524, RIGHT 18.447). That is not a real difference between the sides — both
wheels run the same gearbox, so the ratio is one number by construction. The
0.4% spread is measurement error: LEFT had 16 hand-turned readings and RIGHT
only 5, and weighting by reading count gives 18.506, within 0.07% of the
combined 18.518. **Programming the per-wheel figures into the controller would
bake that measurement artifact in as a permanent 0.4% curve to one side.** The
firmware therefore uses the combined constant for both wheels and has no
per-wheel scale factor anywhere. If the car still curves, look at the
mechanics or the wheel diameter, not at these numbers.

**The encoder is blind to it.** The AS5600 sits on the motor shaft, *before*
the gearbox, so it counts motor revolutions and nothing else. The gear ratio
is precisely the quantity sitting between the encoder and the wheel, so it can
never be recovered from encoder data alone — it needs one observation of the
wheel from outside. There is no firmware trick that avoids this.

Everything else is **derived** from the ratio and the wheel diameter, so those
two numbers are the only free parameters:

```
counts_per_wheel_rev = 4096 * gear_ratio
counts_per_wheel_deg = counts_per_wheel_rev / 360
counts_per_metre     = counts_per_wheel_rev / (pi * wheel_diameter)
ff_slope             = counts_per_metre / 959      # feedforward duty per m/s
```

Change the ratio with `N` and all four move together. Nothing is written twice,
so counts-per-metre and counts-per-degree cannot drift out of agreement.

### Measuring the gear ratio

**Preferred: `gear_check.py`.** It drives the wheel a whole number of
revolutions *under power* using the ratio the firmware currently believes, and
you read the tyre mark. Error shows up multiplied by the revolution count, so
ten turns turns a 1% error into a visible 45°:

```sh
cd ~/"autonomus car"
python gear_check.py /dev/ttyACM0 --side L --revs 10
```

At 10 revolutions, 1° of eyeball error is only 0.028% of the ratio — about 15×
tighter than the hand-turn method. It prints the measured ratio, names the
nearest catalogue part, and offers to write the result to both the firmware and
`calibration.json`.

**Why not hand-turning.** `wheel_cal.py` (§5) is unpowered — you turn the wheel
by hand to a known angle and it records the counter. That is biased **low, and
always in the same direction**: the first part of every turn is absorbed taking
up gearbox backlash, during which the wheel moves and the motor shaft does not,
so those counts are missing from every reading. Two wheels agreeing closely
does *not* rule this out, because a systematic bias moves them both together.
Driving under power loads the gear teeth against one flank for the whole move,
so backlash is taken up once at the start instead of being counted as travel.

This is why the true ratio is probably **above** the measured 18.518. The
nearest standard part is 18.75:1 (= 75/4, a common three-stage ratio) at 1.25%
above — comfortably inside hand-measurement error. A 20:1 gearbox would need an
8% bias, which backlash cannot explain, and would show up in `gear_check.py`
as the wheel landing 288° short over ten revolutions. **Run the tool and
settle it.**

Old method, kept because it needs no working drive:

Use `wheel_cal.py` (§5). **Nothing is powered** — you turn the wheel by hand
to a known angle, press ENTER, and it records the counter. The wheel is
stationary at every reading, so there is no reaction-time error; the only
error left is how well you eyeball the angle, and larger angles shrink that
proportionally. One 720-degree reading beats several 90-degree ones.

Always turn the wheel **one way only**. Backing up to correct an overshoot
takes up gearbox backlash, and the motor shaft (where the encoder is) will
not follow the wheel back.

Measured motor deadband on this car, for reference — not used by the
calibration any more, but it is what the drive commands have to clear:

| Duty | Result |
|---|---|
| ≤ 45 | no movement at all |
| 50 | ~250 counts/s — creeps, close to stall, heats the motor |
| 60 | ~10,200 counts/s — ~10 s per wheel rev, ~2.5 s per quarter turn |
| 110 | ~58,000 counts/s — ~852 motor RPM |

Duty 60 is the lowest that turns cleanly without hovering near stall.

---

## 4. Firmware commands (USB serial, 115200)

### Closed-loop driving (added 2026-08-26)

These are the commands to use for actual driving. Everything here is in the
**forward-positive frame**: `+1` means "this wheel rolls the bot forwards", on
both sides. The firmware resolves the mirrored motor mounting itself, so
nothing above it needs to flip a sign any more.

| Command | Action |
|---|---|
| `V <left> <right>` | closed-loop wheel velocity in **m/s**. Watchdog-armed: re-send faster than 300 ms |
| `W <linear> <angular>` | unicycle — forward m/s and yaw rad/s, mixed on the board |
| `H` | **position hold** — freeze both wheels here and resist being pushed |
| `E` | disengage the closed loop, wheels coast free |
| `K [kpv kiv kpp soft hard]` | read or set the cascade gains, no reflash |
| `N [ratio] [wheel_mm]` | read or set the gear ratio and wheel diameter |
| `B [metres]` | read or set the track width |
| `F [hz]` | `FB` telemetry rate, `0` turns it off |
| `A [0\|1]` | WiFi off/on. **Off at boot.** Bluetooth is always off |
| `I` | scan the I²C bus and every mux channel, naming known parts |
| `Q` | control-loop health: actual rate, worst tick, overruns, I²C errors |
| `?` | print the command list |

`V` and `W` are watchdog-armed, so a crashed controller cannot drive the car
away. `H` is deliberately **exempt** — it is a "stay put" request and nobody is
obliged to keep talking to it — but it still expires on its own after about
two minutes so a forgotten hold cannot cook a stalled motor.

### Legacy / bring-up commands

| Command | Action |
|---|---|
| `Z` | zero both encoder counters |
| `S` | spin test — each motor forward and reverse, reports both encoders |
| `S <duty>` | same, at a specific duty 0–255 (default 110) |
| `X` | stop both motors |
| `R <L\|R> <duty> [F\|B]` | run one motor continuously; auto-stops after 180 s |
| `M` | print `# MARK,<Lcounts>,<Rcounts>,<millis>` right now |
| `G <L\|R> <degrees> [duty]` | closed-loop rotate by wheel degrees; brakes, settles, then trims |
| `P [tol] [trimduty] [brakems] [settlems] [maxtrim]` | read or set the rotate settling parameters, no reflash |

`D` is the one command still in the **raw board frame** — it does not apply the
mirror correction, so `D 110 110` spins the car in place rather than driving it
forward. `D -110 110` is what drives both wheels forward. This is deliberate:
`carbot/driver.py` has always applied `direction` on the host side for `D`, and
changing it would break `wheel_cal.py`. Use `V` for anything new.

### Telemetry streams

```
FB,<ms>,<Lm>,<Rm>,<Lmps>,<Rmps>,<Lsp>,<Rsp>,<LerrMm>,<RerrMm>,
   <Lduty>,<Rduty>,<skewMm>,<x>,<y>,<thDeg>,<refScale>,<mode>,<track>   50 Hz
T,<Lang>,<Lagc>,<Lmag>,<Lstat>,<Rang>,<Ragc>,<Rmag>,<Rstat>              20 Hz
ODO,<Lmetres>,<Rmetres>,<Lok>,<Rok>,<Lcounts>,<Rcounts>                  10 Hz
```

**`FB` is the line to build odometry on.** 19 fields, forward-positive, SI
units, everything the controller can see:

| Field | Meaning |
|---|---|
| `Lm` `Rm` | per-wheel travel, metres, forward-positive |
| `Lmps` `Rmps` | measured wheel speed, m/s, filtered |
| `Lsp` `Rsp` | commanded wheel speed, m/s |
| `LerrMm` `RerrMm` | **tracking error** — how far each wheel is from where the controller says it should be |
| `Lduty` `Rduty` | duty the controller actually applied, −255…255 |
| `skewMm` | left travel minus right travel. **This is the straightness number** |
| `x` `y` `thDeg` | integrated pose |
| `refScale` | 1.0 = loop keeping up; below 1 = a wheel is being held back and both references have been slowed |
| `mode` | 0 off, 1 velocity, 2 hold |
| `track` | track width in use |

```
MAG,<x>,<y>,<z>,<headingDeg>,<field_gauss>,<reads>,<errors>              20 Hz
```

`MAG` only appears if the magnetometer answered at boot. Raw counts plus a
flat-mounted heading — **no tilt compensation** (that needs an accelerometer,
which the QMC5883L does not have) and no hard/soft-iron calibration.

`ODO` and `T` are unchanged and still in the **raw board frame**, because
`encoder_gui.py`, `wheel_odom_node.py` and `carbot/driver.py`'s legacy path all
parse them. Nothing that used to work was altered.

`T,` is the format `encoder_tune.py` parses. Status bits are the raw AS5600
STATUS register: `MD 0x20` magnet detected, `ML 0x10` too weak,
`MH 0x08` too strong.

### Rotate replies

```
# GO,<side>,<degrees>,<duty>,<counts to go>
# GODONE,<side>,<commanded deg>,<moved deg>,<residual deg>,<why>,<trims>
```

`why` is `reached`, `capped` (ran out of trim attempts), `timeout`, `aborted`
or `preempted`. `trims` is how many
correction pulses it needed.

**`residual` is the number that matters**, not `moved`. `moved` is how far
*this one command* travelled, which is deliberately not exactly 90 when it is
correcting the previous move's leftover. `residual` is the gap between where
the wheel ended up and where it should be, and it must stay near zero across a
long run of identical moves rather than growing. See §4a.

---

## 4a. Why repeated 90° moves used to drift (fixed 2026-08-24)

A single `G L 90` landed on 90°, but ten of them in a row walked off by tens
of degrees. Two causes, both in the firmware's rotate state machine:

**1. The move ended at the wrong moment.** `goService()` finished as soon as
the counter *crossed* the target, cut PWM, and read the encoder immediately
for its report. Both BTS7960 inputs low is **coast**, not brake — the wheel
was still spinning when the report was printed. So every move reported ~0.00°
error while physically sailing a degree or two past. The error was real, it
was just never measured.

**2. Each move re-based on wherever the wheel actually stopped.** `goStart_()`
set `goTarget = encoders[side].counts + delta`. The uncorrected overshoot from
move *n* silently became the zero point of move *n+1*, so a small repeatable
bias compounded instead of cancelling.

### What it does now

| Phase | What happens |
|---|---|
| `GO_RUN` | drive, tapering over the last fifth as before |
| `GO_BRAKE` | on crossing the target, drive **both** PWM inputs to 255 for 120 ms |
| `GO_SETTLE` | release to coast, wait 200 ms, *then* read the encoder |
| `GO_PULSE` | if outside tolerance, a short kick at duty 70 back toward the target, then brake and settle again — up to 10 times |

Both inputs high on a BTS7960 ties both motor terminals to Vcc and shorts the
winding through the high sides. The two half-bridges are independent, so this
is a defined brake state, not shoot-through. It is what removes most of the
coast in the first place; the trim pulses close whatever is left.

Alongside that, the firmware now keeps `goIdeal[side]` — where each wheel
*should* be if every move had been perfect — and aims the next move from that
rather than from the actual stopped position. **Residual is corrected, never
inherited.** The error stays bounded at the tolerance instead of accumulating.

`goIdeal` is dropped whenever something moves a wheel outside the rotate loop:
a `Z` zero, a `D` command, an `R` run, the web joystick, or a gap of more than
45° between ideal and actual (a hand nudge). The next `G` then starts clean.

### Tuning

Defaults are ±60 counts (≈0.3° of wheel) and roughly 0.4 s of settling per
move. Change them live with `P`, no reflash:

```
P                        # print the current values
P 30 70 120 200 10       # tol=30 counts (~0.14deg), trimduty 70,
                         # brake 120 ms, settle 200 ms, max 10 trims
```

Tighter than about ±20 counts is below the gearbox backlash floor and it will
hunt — the encoder is on the motor shaft, so it cannot see the wheel move
through backlash at all. If trims regularly hit the cap, raise `trimduty` a
little (a 25 ms kick at 70 barely clears the deadband when cold) or widen the
tolerance.

### Checking it

Run the same angle 10–20 times and watch the `residual` field, not `moved`:

```sh
cd ~/"autonomus car"
python rot.py /dev/ttyACM0      # answer L, 90, 90 repeatedly
```

Residual should bounce around zero inside the tolerance and show no trend. A
steady creep in one direction is no longer accumulation — it means
`COUNTS_PER_WHEEL_DEG` is biased, and the fix is to recalibrate over a large
angle (720°+, one direction only, per §3) rather than to touch the loop.

**The mechanical check still rules.** Mark the tyre, command 90° four times,
and see whether the mark is back where it started. The encoder is before the
gearbox, so it is blind to backlash and to a slipping wheel; it will happily
report a perfect residual while the tyre sits a few degrees off.

---

## 5. Running the tools

Magnet strength dashboard (AGC gauges, 0–128, target 64):

```sh
cd ~/"autonomus car"
python encoder_tune.py /dev/ttyACM0
```

Live odometry and straight-line tracking (the closed-loop dashboard):

```sh
cd ~/"autonomus car"
python odom.py /dev/ttyACM0                      # watch only, never moves a wheel
python odom.py /dev/ttyACM0 --drive 0.2          # drive straight at 0.2 m/s
python odom.py /dev/ttyACM0 --drive 0.2 --distance 1.0   # then stop and hold
```

Watch the **SKEW** figure — left travel minus right travel. Under a millimetre
over metres of travel is normal; more than about 5 mm means something is wrong
mechanically, because the controller cannot lose track by that much on its own.

Gear ratio, measured under power:

```sh
cd ~/"autonomus car"
python gear_check.py /dev/ttyACM0 --revs 10
```

Distance readout:

```sh
cd ~/"autonomus car"
python encoder_gui.py /dev/ttyACM0
```

Gear-ratio calibration:

```sh
cd ~/"autonomus car"
python wheel_cal.py
```

A full-screen TUI. Tape a mark on each tyre and pick a fixed reference on the
chassis, then follow the four numbered panels.

| Key | Action |
|---|---|
| `L` / `R` | pick which wheel you are turning |
| `ENTER` | set the start point, then record each reading |
| `N` | new start point for the current wheel |
| `U` | undo the last reading (also removes it from the CSV) |
| `F` | finish — shows results and the line to paste |
| `Q` | quit |

Angles are **cumulative from the start point**, and the live bar shows the
estimated angle turned so far once there is one reading to calibrate against.
Every reading is written to `wheel_cal.csv` the moment you take it, so closing
the window never loses data. Trials from different days accumulate.

Wheel odometry — absolute wheel angle, distance, and unintended rotation:

```sh
cd ~/"autonomus car"
python wheel_odom_node.py /dev/ttyACM0
```

See `SOFTWARE.md` §2. It needs `track_width_m`, still unmeasured in
`calibration.json`, before it will report a heading.

Rotate a wheel by a commanded angle:

```sh
cd ~/"autonomus car"
python rotate.py
```

`L`/`R`/`B` picks the wheel, arrows or typed digits set the angle, up/down set
the speed, ENTER goes, SPACE aborts. It shows a live progress bar during the
move and then the commanded angle, the actual angle and the error.

Measured accuracy before the settling fix: 90° → 90.34°, −90° → −90.06°,
180° → 180.00°. Those figures were read at the instant PWM was cut, with the
wheel still coasting, so they flattered the loop — see §4a. Re-measure them
against the `residual` field now that the wheel is stopped before it reports.

Reflash:

```sh
cd ~/drive-harness-diag/c3-webgui
pio run -t upload
```

The `/dev/ttyACM0` argument is required — both scripts default to
`/dev/ttyUSB0`, which was the old WROOM board and no longer exists. Only one
tool can hold the port at a time; close the GUI before reflashing.

`encoder_tune.py` was patched to skip its DTR/RTS hardware reset on `ACM`
ports. The C3's USB peripheral is inside the chip, so pulsing DTR/RTS resets
the chip and tears down the USB device underneath the open handle.

---

## 5a. Radios (changed 2026-08-26)

**WiFi and Bluetooth are off at boot.** On a single-core C3 the WiFi task runs
at priority 18–23, well above the 500 Hz control task at 5, so it preempts the
control loop whenever the radio needs servicing. That was the entire source of
the loop's tick overruns:

| | radio on | radio off |
|---|---|---|
| tick overruns | 6 in 21,475 | **0 in 9,172** |
| worst tick | 2067–3313 µs | **1482–1537 µs** |
| loop rate | 499.7 Hz | 500.0–500.2 Hz |
| free heap | 203,940 B | **267,264 B** |

The worst-case tick more than halved, leaving 23% headroom inside the 2 ms
budget instead of occasionally blowing it, and 63 kB of heap came back.

Bluetooth was never *started* by this firmware — no BLE library is linked — so
the controller was already idle. It is now explicitly disabled and its memory
released, so the state is a stated fact rather than an assumption.

The phone joystick still exists. Bring the radio up over USB serial when you
want it, and drop it again afterwards; nothing needs reflashing:

```
A 1     # WiFi up, connects, serves the joystick page
A       # report current state
A 0     # WiFi down (also disengages the closed loop, so a dead
        #            link can never leave a wheel driving)
```

To go back to radio-on-at-boot, flip `RADIO_ON_AT_BOOT` in `main.cpp`.

The long-term plan is a separate 2.4 GHz link ("bullet"), so the ESP32's own
radios are not expected to be used at all. They are left compiled in rather
than stripped because they currently cost nothing: with the radio down, flash
sits at 69.9% and 267 kB of heap is free, so neither is a constraint. Strip
`WiFi`/`ESPAsyncWebServer` only if flash actually becomes tight — until then
the escape hatch is worth more than the space.

### Free GPIO, for what comes next

Used: 0, 1, 3, 4 (motor PWM), 6, 7 (I²C). Because `ARDUINO_USB_CDC_ON_BOOT=1`
puts the console on USB, **UART0 is free** as well.

| GPIO | Notes |
|---|---|
| 20, 21 | UART0 RX/TX — free, and the natural home for a serial IMU or an SBUS receiver |
| 5, 10 | free general purpose |
| 2, 8, 9 | **strapping pins** — avoid (9 is BOOT) |

That is enough for both a serial IMU (1 pin, RX only, since you only read it)
and a 2.4 GHz receiver (1 pin for SBUS or PPM) without contention.

## 5b. Magnetometer — GY-271 (QMC5883L, **not** HMC5883L)

Wired on the **main I²C bus at 0x0D**, ahead of the mux, so it is readable
whatever mux channel is selected. Confirmed with the `I` scan command.

**It is not the chip the silkscreen claims.** Boards sold as "GY-271
HMC5883L" are almost always a QMC5883L — a different die, a different register
map, and address 0x0D instead of the genuine Honeywell part's 0x1E. HMC5883L
libraries will not work on it. The firmware drives it as a QMC5883L:
continuous mode, 200 Hz, ±8 gauss, 512× oversampling, with the SET/RESET
period register written to 0x01 (skip that and it returns garbage).

### It survives the motors, which was not obvious

The worry was that two AS5600 diametric magnets spinning on the motor shafts,
plus two brushed motors drawing amps through 20 kHz PWM, would swamp a compass.
Measured, they do not:

| condition | heading | vs baseline |
|---|---|---|
| still, motors off | 203.3° ± 0.80 | — |
| wheels turned 90 / 180 / 270 / 360° | 203.4 / 203.2 / 203.4 / 203.1 | ≤ 0.3° |
| driving 0.10 / 0.30 / 0.60 m/s | 203.3 / 203.6 / 203.5 | ≤ 0.3° |
| back to still | 203.2 | −0.1° net |

Rotating the encoder magnets through a full turn and running the motors at 6×
the current both move the heading by less than the sensor's own ±0.8° noise.
Whatever the mounting distance is, the dipole falloff is doing its job. Field
magnitude reads 0.60–0.62 G, a little above Earth's, which is the expected
hard-iron offset from the chassis and is calibratable.

### What it still cannot do

- **No tilt compensation.** Heading is only valid while the car is flat. A
  ramp or a bump tilts the sensor and the heading swings.
- **No gyro**, so no high-rate yaw rate and nothing to carry heading through a
  magnetic anomaly. Rebar, steel furniture and door frames will pull it.
- ±0.8° noise, reported at 20 Hz.

## 6. Measured results (2026-08-23)

Spin test, 0.7 s per direction, wheels off the ground. Counts are raw motor
counts; the metre figures the firmware printed alongside them were wrong per
§3 and are omitted here.

| Motor | Duty | FWD counts | REV counts | FWD/REV |
|---|---|---|---|---|
| LEFT | 110 | 34,804 (avg of 4) | 40,719 (avg of 4) | 0.855 |
| RIGHT | 110 | 39,028 (avg of 4) | 38,157 (avg of 4) | 1.023 |
| LEFT | 60 | 12,022 | 14,866 | 0.809 |
| RIGHT | 60 | 9,892 | 12,331 | 0.802 |

**Confirmed good:**

- **Motor↔encoder pairing is correct.** Driving LEFT moves only `encL`
  (`encR` stayed within ±2 counts), and driving RIGHT moves only `encR`.
  No mux cross-talk.
- **Both encoders track both directions**, and the sign reverses cleanly.
- **No counts are being lost.** Count-dropping gets worse with speed; the
  forward/reverse asymmetry here *shrinks* as speed rises, and at low duty it
  hits RIGHT (healthy magnet) as hard as LEFT (weak magnet). The asymmetry is
  brushed-motor behaviour near the stiction threshold, not an encoder fault.
- **Sampling headroom is ~5×.** The unwrap fails only if a shaft crosses 2048
  counts between samples. At ~7 ms sampling that ceiling is ~4,300 motor RPM;
  measured top speed at duty 110 is ~852 motor RPM.

---

## 7. Known issues

**Magnet airgap is marginal on both encoders.** Measured AGC = 128 on both,
pegged at the ceiling for a 3.3 V rail, target 64. LEFT also asserts its own
`ML` (magnet too weak) flag and has lower magnitude (1546 vs 2006).

AGC is the AS5600's internal gain, so it is an *inverse* proxy for field
strength — pegged means the chip ran out of gain and still cannot see enough
field. There is no software fix; it is an airgap problem. It is not currently
costing accuracy (see §6), but there is zero margin: vibration, heat, or a
knock and `MD` clears and the sensor drops out entirely. Move the magnets
closer and watch the gauge fall toward 64. LEFT needs it most.

**Backlash is invisible to the control loop.** The AS5600 is on the motor
shaft, before an 18.5:1 gearbox, so the rotate loop cannot see the wheel move
through gearbox backlash and cannot correct it. Repeat accuracy at the wheel
is therefore floored by mechanical slop no matter how tight `P` is set. Always
command a repeated angle in the *same direction* when it matters.

**Encoder direction is inverted vs the old harness convention.** RPWM
("forward") produces *negative* counts on both encoders; the WROOM README
says RPWM should increase counts. Consistent on both sides, so it is a
one-line sign flip when odometry gets wired up.

**LEFT keeps a 13% forward/reverse asymmetry at duty 110** where RIGHT sits
at 2%, repeatable across four runs. Ruled out as an encoder fault; suspect a
stiffer gearbox or a dragging brush on that motor.

**WARNING — do not wire W5500 MISO to GPIO 9.** GPIO 9 is the C3's boot
strapping pin. If the W5500 holds it low at reset the board enters download
mode instead of booting. Pick a different pin before adding Ethernet.

---

## 8. WiFi

The firmware tries `LPthinkpad` as a station client and **continues on
failure** rather than rebooting — the encoder readout works over USB with no
network at all. When it does associate it serves a joystick + distance page
on port 80, with a WebSocket at `/ws`.

The `LPthinkpad-Hotspot` NetworkManager profile exists on the laptop but is
not active; `wlan0` is on `Student-WiFi`, and there is one radio, so starting
the hotspot drops the laptop's internet.
