# Autonomous Car — Drive Harness

Bring-up and diagnostic firmware for a differential-drive autonomous car:
an ESP32 driving two brushed motors through two BTS7960 half-bridges, with
two AS5600 magnetic encoders for odometry sitting behind a PCA9548A I²C
switch.

Both encoders answer to the same address (0x36), which is the reason the
I²C switch exists — only one channel is ever enabled at a time.

## Hardware

| Part | Qty | Notes |
|---|---|---|
| ESP32-WROOM-32 | 1 | CP210x USB-serial, enumerates as `/dev/ttyUSB0` |
| BTS7960 half-bridge driver | 2 | 12 V motor rail |
| AS5600 magnetic encoder | 2 | both at 0x36, diametric magnets |
| PCA9548A I²C switch (HW-617) | 1 | address 0x70 |
| 3S LiPo / 12 V supply | 1 | motor rail |

## As-built wiring

Verified on hardware with the built-in mapping test. **These pin assignments
are deliberate and were measured, not assumed** — an earlier design document
had the two drivers on opposite GPIOs.

| ESP32 | Goes to |
|---|---|
| GPIO 32 | LEFT motor RPWM |
| GPIO 33 | LEFT motor LPWM |
| GPIO 25 | RIGHT motor RPWM |
| GPIO 26 | RIGHT motor LPWM |
| GPIO 21 | I²C SDA → PCA9548A |
| GPIO 22 | I²C SCL → PCA9548A |
| GPIO 5 | intended mux RESET — **not connected on this build** |

RPWM produces increasing encoder counts on both sides.

Encoders behind the mux:

| Wheel | Mux channel | Channel mask |
|---|---|---|
| LEFT | 6 | `0x40` |
| RIGHT | 3 | `0x08` |

### Deliberate deviations

- **R_EN / L_EN are strapped to VCC.** Both drivers are permanently enabled;
  there is no software e-stop, and the PWM pins float during ESP boot. The
  BTS7960 inputs have internal pull-downs, which is what keeps the motors
  still through reset. GPIO 27 is unused.
- **R_IS / L_IS are tied to GND.** No current sensing, and no driver fault
  flag — a BTS7960 reports over-temperature and over-current on that same pin.
- **GPIO 5 → mux RESET is absent.** Confirmed by test: pulsing the pin does
  not clear the mux control register. Recovery from a latched channel needs a
  software I²C recovery path instead of a hardware reset.

## Firmware

PlatformIO, `board = esp32dev`, Arduino framework.

```sh
pio run -t upload
```

On boot it runs the full check sequence, then accepts line commands and runs a
200 Hz control loop.

| Command | Action |
|---|---|
| `P <L\|R> <deg>` | closed-loop relative rotation |
| `PB <deg>` | same move on both wheels |
| `D <L\|R> <-255..255>` | open-loop duty |
| `V <L\|R> <counts/s>` | velocity control |
| `C <L\|R>` | calibrate — measures deadband and duty→speed slope |
| `K <kp> <ki> <kd>` | set PID gains live |
| `TOL <counts>` | set settling tolerance |
| `Z` / `X` / `T` | zero position / stop / toggle telemetry |
| `r` `m` `1`–`4` | diagnostics: full scan, PWM mapping, single-pin map |

`X` also aborts a running calibration, which is why calibration polls the
serial port itself rather than blocking the loop outright.

### Control design

Position is tracked by unwrapping the AS5600's 12-bit angle into an `int64_t`,
so travel is unbounded across revolutions. Four things beyond a plain PID earn
their place, each because a measurement forced it:

**Trapezoidal motion profile.** A step target makes the wheel run at full duty
until it is already at the target, and its own inertia carries it past — that
was 11.43° of overshoot on a 360° move. The controller instead chases a
setpoint that accelerates, cruises and decelerates into the target, arriving at
near-zero speed.

**Velocity feedback (`Kv`).** Both PWM inputs low is *coast*, not brake, so
without an active term nothing opposes the wheel's momentum. `Kv` commands
reverse duty whenever the wheel runs faster than the profile wants. It is
clamped at 0.070: a sweep measured 258° of overshoot at `Kv 0.14`, because the
term differentiates a quantised, filtered velocity and goes unstable.

**Stiction kick, gated on distance remaining.** Below the measured deadband the
motor makes no torque, so a small proportional term just hums. The gate is on
distance to target, not tracking error — during a small corrective move the
setpoint creeps and tracking error stays tiny, so a tracking-error gate never
fires and the move stalls. A direction test means the kick can only ever push
toward the target.

**Backlash handling.** There is roughly 35 counts (~3°) of lost motion between
the motor and the encoder: the motor turns without the geared output following,
so that region is invisible to the loop and cannot be tuned away. Two measures
address it. Every move finishes travelling in the same direction (`APPROACH`),
running past the target by `TAKEUP` counts and coming back if it would
otherwise arrive from the wrong side, so the same tooth face is loaded every
time. And the final counts are closed with **pulsed creep** — short pulses with
pauses — because the continuous duty needed to break stiction also carries the
wheel past.

A runaway guard cuts drive and reports `FAULT` above `VEL_LIMIT`, so an
unstable gain cannot spin a wheel up.

### Calibration persists

Calibration is stored in NVS and reloaded at boot. This matters more than it
sounds: with no calibration the deadband is zero, the stiction kick is disabled
entirely, and every move silently undershoots — which looks exactly like a
controller fault. The boot banner prints `CALLOAD` for each wheel and warns if
either is missing.

### What the check sequence proves

1. **Upstream bus** — with all channels off, only 0x70 should answer. An
   0x36 here means an encoder is wired to the upstream bus by mistake.
2. **Mux control register** — write a mask, read it back.
3. **RESET line** — select a channel, pulse GPIO 5, confirm the register clears.
4. **Per-channel scan** — all 8 channels, so a mis-plugged encoder is visible
   rather than merely absent. Then mask `0x00` to confirm isolation.
5. **Encoder health** — magnet placement read from STATUS/AGC registers.
6. **PWM mapping** — the only way to learn which GPIO reaches which driver,
   since a BTS7960 has no readback.

## Host tools

Python 3, `pyserial`, and `PyQt5` (or PyQt6) for the GUI.

### `encoder_tune.py` — live magnet air-gap tuning

```sh
python3 encoder_tune.py [/dev/ttyUSB0]
```

Real-time dashboard for physically positioning the encoder magnets. The
AS5600 raises its internal amplifier gain when the field is weak, so the AGC
register is an inverse proxy for field strength at the die:

- **low AGC** → strong field → magnet close
- **high AGC** → weak field → magnet far
- **pegged at 128** (the 3.3 V maximum; it is 255 at 5 V) → the sensor has run
  out of gain, and the ML status bit is set

Aim for the middle of the range, near 64, so the chip has headroom in both
directions. Shows an AGC arc gauge with the good band drawn into the scale, a
best-so-far marker, a 20 s history strip, MD/ML/MH status LEDs, and a shaft
dial with a turn counter.

### `drive_tui.py` — control console

```sh
python3 drive_tui.py [/dev/ttyUSB0]
```

Live position, target, error, velocity, duty and magnet health for both
wheels, plus calibration results and a rolling record of the settling error of
every move — so a gain change can be judged against measured error instead of
by feel.

| Key | Action |
|---|---|
| `tab` | select wheel |
| `h` / `l` | rotate −15° / +15° |
| `j` / `k` | rotate −90° / +90° |
| `J` / `K` | rotate −360° / +360° |
| `w` / `s` | duty +10 / −10 |
| `space` | stop |
| `z` | zero position |
| `c` | calibrate selected wheel |
| `:` | command line (`p 123.5`, `v 4000`, `k 0.35 0.4 0.004`, `tol 2`) |
| `q` | quit |

### Other scripts

| Script | Purpose |
|---|---|
| `capture.py <port> <secs>` | reset the board and capture serial output |
| `run_cmd.py <port> <cmd> <secs>` | reset, wait for boot, send a command, capture |
| `spin.py <port>` | reset, then spin each motor once |
| `selftest_control.py <port>` | headless check of moves and calibration |

Only one program can hold the serial port at a time.

## Measured results

Encoders, after tuning the air gap with `encoder_tune.py`:

| | AGC | ML flag | MAGNITUDE |
|---|---|---|---|
| LEFT | 70 | clear | 2129 |
| RIGHT | 45 | clear | 2124 |

Motor calibration:

| | Deadband fwd / rev | Slope | Top speed |
|---|---|---|---|
| LEFT | 34 / 32 | 70.7 cts/s per duty | 16089 cts/s (236 rpm) |
| RIGHT | 36 / 34 | 68.8 cts/s per duty | 15336 cts/s (225 rpm) |

Left runs about 5% faster than right at identical duty. The per-wheel
feedforward compensates for it; without that a straight-line command curves.

Settling error over 6 moves per setting:

| Setting | mean \|err\| | worst |
|---|---|---|
| tol 8 | 0.468° | 0.700° |
| tol 4 | 0.307° | 0.350° |
| **tol 2** (default) | **0.120°** | **0.180°** |

Raising Kp did not help — at Kp 0.55 the wheel hunted and failed to settle on
3 of 6 moves. Tolerance, not gain, was the limiting factor. One count is
0.088°, so tol 2 sits just above the encoder's quantisation floor.

### Overshoot and repeatability

Overshoot on a 360° move, as the controller was built up:

| | overshoot |
|---|---|
| step target, no profile | 11.43° |
| profile only | 6.42° |
| profile + velocity feedback + corrected kick gate | **0.18–0.79°** |

Repeatability over 6 cycles of +90°/−90°, which is the number that exposes
backlash — an individual move can report a small error while the pair still
loses ground:

| | before | after |
|---|---|---|
| LEFT drift | −23.64° | **−1.14°** |
| LEFT spread | 20.30° | **0.70°** |
| RIGHT drift | −22.24° | **−2.02°** |
| RIGHT spread | 18.90° | **1.32°** |

The important change is that drift is now bounded rather than accumulating.

Individual moves settle at 0.00°–0.18° with reason `tol` when calibrated:

```
L  +90:  DONE,L, 2, 0.18,tol
L  -90:  APPROACH,L,25872   ->  DONE,L, 0, 0.00,tol
R  -90:  APPROACH,R,27064   ->  DONE,R,-2,-0.18,tol
```

Note that a move opposite to the approach direction shows ~8° of apparent
overshoot in `analyze_moves.py`. That is the planned `TAKEUP` run-past, not a
control fault.

## Logs

`drive_tui.py` writes every line sent and received to
`logs/session-YYYYmmdd-HHMMSS.log` with timestamps. `analyze_moves.py --csv
DIR` writes each move's full 200 Hz position trace as CSV. Both are gitignored.

## Status

Verified on hardware: I²C switch and channel isolation, both encoders present
and counting, the full command → driver → motor → magnet → encoder loop on
both sides with under 1 count of cross-talk, closed-loop moves settling inside
0.18°, and calibration measuring deadband and speed slope on both wheels.

Not yet done: counts-per-metre calibration against a measured straight run,
which needs a tape measure and cannot be derived on the bench.

## Licence

MIT
