// drive-harness-diag -- bring-up, closed-loop control and calibration for the
// ESP32 / BTS7960 / AS5600 / PCA9548A drive harness.
//
// Safety: the motor rail is live and R_EN/L_EN are strapped to VCC, so both
// drivers are permanently enabled. Only the PWM duty being zero keeps the
// motors still. Every path that ends a move drives duty to zero.

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ---- output tee ----
// Everything the firmware prints goes to USB serial AND to the TCP client, so
// the wireless link is a peer of the cable rather than a replacement. A WiFi
// failure can never leave the board unreachable.
#define TCP_PORT 3333
static WiFiServer tcpServer(TCP_PORT);
static WiFiClient tcpClient;

class Tee : public Print {
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (tcpClient && tcpClient.connected()) tcpClient.write(c);
    return 1;
  }
  size_t write(const uint8_t *b, size_t n) override {
    Serial.write(b, n);
    if (tcpClient && tcpClient.connected()) tcpClient.write(b, n);
    return n;
  }
};
static Tee Out;

// ---- harness pin map ----
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_MUX_RESET   5

// AS BUILT, verified by the mapping test -- the two driver connectors are on
// the opposite GPIOs from the original wiring reference. Do not "correct"
// these back to 25/26 = left without re-running the mapping test.
//   GPIO 32/33 -> LEFT  driver   (encoder on mux channel 6)
//   GPIO 25/26 -> RIGHT driver   (encoder on mux channel 3)
// RPWM produces increasing encoder counts on both sides.
#define PIN_L_RPWM     32
#define PIN_L_LPWM     33
#define PIN_R_RPWM     25
#define PIN_R_LPWM     26

#define MUX_ADDR     0x70
#define ENC_ADDR     0x36
#define MASK_LEFT    0x40   // channel 6
#define MASK_RIGHT   0x08   // channel 3

// AS5600 registers
#define AS_STATUS    0x0B
#define AS_RAW_ANGLE 0x0C
#define AS_AGC       0x1A
#define AS_MAGNITUDE 0x1B

#define CPR          4096.0f          // counts per revolution
#define LOOP_US      5000             // 200 Hz control loop

// ---- PWM ----
// One LEDC channel per driver input, set up once and never detached, so the
// diagnostics and the control loop cannot fight over the GPIO matrix.
static const uint8_t PWM_PIN[4] = { PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM };
static const char   *PWM_NAME[4] = { "LEFT  RPWM", "LEFT  LPWM", "RIGHT RPWM", "RIGHT LPWM" };
static bool pwmReady = false;

static void pwmInit() {
  if (pwmReady) return;
  for (uint8_t i = 0; i < 4; i++) {
    ledcSetup(i, 15000, 8);
    ledcAttachPin(PWM_PIN[i], i);
    ledcWrite(i, 0);
  }
  pwmReady = true;
}

static inline void pwmSet(uint8_t idx, int duty) {
  ledcWrite(idx, constrain(duty, 0, 255));
}

static void killDrive() {
  if (!pwmReady) {                      // before LEDC exists, force the pins low
    for (uint8_t i = 0; i < 4; i++) { pinMode(PWM_PIN[i], OUTPUT); digitalWrite(PWM_PIN[i], LOW); }
    return;
  }
  for (uint8_t i = 0; i < 4; i++) ledcWrite(i, 0);
}

// ---- I2C primitives ----
static bool ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool muxWrite(uint8_t mask) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(mask);
  return Wire.endTransmission() == 0;
}

static int muxRead() {
  if (Wire.requestFrom((uint8_t)MUX_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

static int as5600Read8(uint8_t reg) {
  Wire.beginTransmission(ENC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)ENC_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

static int as5600Read12(uint8_t reg) {
  Wire.beginTransmission(ENC_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)ENC_ADDR, (uint8_t)2) != 2) return -1;
  int hi = Wire.read(), lo = Wire.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

// GPIO 5 -> mux RESET is not connected on this build, so a latched channel has
// no hardware recovery. This clocks SCL by hand to free a slave that is holding
// SDA low, then re-inits the peripheral -- the software stand-in for RESET.
static void i2cRecover() {
  Wire.end();
  pinMode(PIN_SCL, OUTPUT);
  pinMode(PIN_SDA, INPUT_PULLUP);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
  }
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(5);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  muxWrite(0x00);
}

// ---------------------------------------------------------------- wheels
struct Wheel {
  Wheel(const char *n, uint8_t f, uint8_t r, uint8_t m)
    : name(n), idxFwd(f), idxRev(r), mask(m) {}

  const char *name;
  uint8_t idxFwd, idxRev;     // LEDC channel indices: +duty and -duty
  uint8_t mask;

  int      lastRaw = -1;
  bool     haveRaw = false;
  int64_t  pos = 0;           // unwrapped counts, survives many revolutions
  int64_t  target = 0;
  float    vel = 0;           // counts/s, low-pass filtered
  int      duty = 0;          // signed, -255..255
  uint32_t errCount = 0;

  float    integ = 0, prevErr = 0;
  bool     settled = true;
  uint32_t settleStart = 0;

  // trapezoidal profile state -- the controller chases this, not the raw target
  double   spPos = 0;         // setpoint position, counts
  float    spVel = 0;         // setpoint velocity, counts/s
  bool     profActive = false;
  uint32_t holdStart = 0;
  int      retries = 0;       // corrective re-moves used on the current command
  uint8_t  creepState = 0;    // 0 start pulse, 1 pulsing, 2 pausing
  uint32_t creepUntil = 0;
  int      creepCount = 0;
  int64_t  finalTarget = 0;   // where the move really ends
  uint8_t  approachStage = 0; // 0 direct, 1 running past, 2 final approach
  uint32_t stallStart = 0;

  int      minDutyFwd = 0, minDutyRev = 0;   // measured stiction thresholds
  float    slope = 0;                        // counts/s per duty unit

  uint8_t  mode = 0;          // 0 idle, 1 raw duty, 2 position, 3 velocity
  float    velTarget = 0;
};

static Wheel WL("L", 0, 1, MASK_LEFT);
static Wheel WR("R", 2, 3, MASK_RIGHT);

// Gains shared by both wheels, tuned live from the TUI. These defaults were
// measured, not guessed: a sweep over Kp 0.35-0.80 found raising Kp made
// settling worse (Kp 0.55 hunted and failed to settle on 3 of 6 moves), while
// tightening POS_TOL from 8 to 2 cut mean settling error from 0.468 to 0.120
// degrees. One count is 0.088 deg, so tol 2 sits just above the sensor floor.
static float Kp = 0.35f, Ki = 0.40f, Kd = 0.004f;
static int   POS_TOL = 2;          // counts, ~0.18 deg

// Motion profile limits. Arriving at the target with near-zero velocity is
// what removes overshoot -- a step target makes the wheel run at full duty
// until it is already there, and its own inertia carries it past.
// Measured best of a VMAX/AMAX/Kv sweep: 0.18 deg overshoot, against 11.43 deg
// with a step target and no profile.
static float VMAX = 3000.0f;       // counts/s  (~44 rpm, top speed is ~16000)
static float AMAX = 9000.0f;       // counts/s^2 (reaches VMAX in 0.33 s)
static uint32_t HOLD_MS = 500;     // hold window before settling or retrying

// Velocity feedback. Both PWM low is coast, not brake, so without this the
// wheel's inertia carries it past a decelerating setpoint. This term commands
// reverse duty whenever the wheel is running faster than the profile wants.
// Scale reference: 1/slope ~ 0.014 duty per count/s.
// Kv above ~0.06 goes unstable: it differentiates a quantised, filtered
// velocity, and the sweep showed 258 deg of overshoot at Kv 0.14.
static float Kv = 0.060f;
static int   MAX_RETRY = 3;        // corrective re-moves for a large residual

// Pulsed creep for the last few counts. Continuous drive cannot win here: the
// duty needed to break stiction also carries the wheel past the target. Short
// pulses with pauses let it stop between each one, so the step size is set by
// pulse width rather than by friction.
// Budget matters: measured backlash is ~35 counts and each pulse moves ~2-3,
// so a 14-pulse budget was entirely consumed crossing the dead zone with
// nothing left to position with. 60 pulses crosses it and still converges.
static int   CREEP_MS = 12;        // pulse width -- ~2-3 counts per pulse
static int   CREEP_GAP = 45;       // pause, long enough to come to rest
static int   CREEP_BOOST = 6;      // duty above the measured deadband
static int   CREEP_MAX = 60;       // pulses before accepting the residual

// Backlash handling. There is lost motion between the motor and the encoder --
// the motor can turn without the geared output following, so that region is
// invisible to the control loop and cannot be tuned away. Instead every move
// finishes travelling in the same direction, which loads the same tooth face
// each time and makes final position repeatable. A move that would arrive from
// the wrong side runs past by TAKEUP counts and comes back.
static int   APPROACH_DIR = 1;     // 0 disables, +1 or -1 selects the side
static int   TAKEUP = 90;          // counts to run past -- must exceed the ~35
                                   // counts of measured backlash with margin

// Runaway guard: an unstable gain must never be able to spin a wheel up.
static float VEL_LIMIT = 18000.0f; // counts/s, above the measured 16089 top speed

// ---- robot geometry ----
// Wheel diameter 119.56 mm -> circumference 375.61 mm, and 4096 counts is one
// wheel revolution, giving 10905 counts/m. That is the theoretical value only:
// tyre compression and slip typically put the real figure a few percent off,
// so correct it with a measured run via CALDIST. If the encoder turns out to
// sit before a gear reduction, CALDIST absorbs that too.
static float CPM = 10905.0f;       // counts per metre
static float TRACK_MM = 0.0f;      // wheel centre-to-centre, MUST be measured

// Pose, integrated from both encoders. Standard differential-drive odometry.
static double poseX = 0, poseY = 0, poseTh = 0;
static int64_t lastPoseL = 0, lastPoseR = 0;
static float lastDriveM = 0;       // last commanded distance, for CALDIST
static bool  telem = false;

// ---- move trace ----
// Telemetry at 50 ms cannot see a transient: at top speed the wheel covers
// ~70 deg between samples. This records every control tick into RAM during a
// move so peak overshoot can be measured instead of eyeballed.
#define TRACE_N 1500                  // 7.5 s at 200 Hz
static int32_t  traceBuf[TRACE_N];
static uint16_t traceCount = 0;
static bool     traceArmed = false;
static Wheel   *traceW = nullptr;
static int64_t  traceStart = 0, traceTargetRel = 0;

static void applyDuty(Wheel &w, int d) {
  d = constrain(d, -255, 255);
  w.duty = d;
  if (d >= 0) { pwmSet(w.idxRev, 0); pwmSet(w.idxFwd, d); }
  else        { pwmSet(w.idxFwd, 0); pwmSet(w.idxRev, -d); }
}

static void stopWheel(Wheel &w) {
  w.mode = 0;
  w.integ = 0;
  applyDuty(w, 0);
}

static void stopAll() { stopWheel(WL); stopWheel(WR); }

// Read one encoder and fold the 12-bit wrap into an unbounded position.
static void updateWheel(Wheel &w, float dt) {
  if (!muxWrite(w.mask)) { w.errCount++; i2cRecover(); return; }
  int raw = as5600Read12(AS_RAW_ANGLE);
  if (raw < 0) { w.errCount++; return; }

  if (!w.haveRaw) { w.lastRaw = raw; w.haveRaw = true; return; }

  int d = raw - w.lastRaw;
  if (d >  2048) d -= 4096;           // wrapped backwards past zero
  if (d < -2048) d += 4096;           // wrapped forwards past full scale
  w.lastRaw = raw;
  w.pos += d;

  float vraw = (dt > 0) ? d / dt : 0;
  w.vel += 0.25f * (vraw - w.vel);    // light filter; raw is quantised and noisy
}

// Start a profiled move to an absolute target from wherever we are now.
static void startMove(Wheel &w, int64_t absTarget) {
  w.target = absTarget;
  w.integ = 0;
  w.prevErr = 0;
  w.settled = false;
  w.spPos = (double)w.pos;
  w.spVel = 0;
  w.profActive = true;
  w.holdStart = 0;
  w.retries = 0;
  w.creepState = 0;
  w.creepCount = 0;
  w.stallStart = 0;
  w.mode = 2;
}

// End of a move stage. If we only ran past the target to take up backlash,
// this starts the real approach instead of reporting completion.
static void finishMove(Wheel &w, const char *why) {
  applyDuty(w, 0);
  if (w.approachStage == 1) {
    w.approachStage = 2;
    startMove(w, w.finalTarget);
    Out.printf("APPROACH,%s,%lld\n", w.name, (long long)w.finalTarget);
    return;
  }
  w.approachStage = 0;
  w.settled = true;
  w.mode = 0;
  Out.printf("DONE,%s,%lld,%.2f,%s\n", w.name,
                (long long)(w.finalTarget - w.pos),
                (w.finalTarget - w.pos) * 360.0f / CPR, why);
}

// PID with a stiction kick: below the measured deadband the motor makes no
// torque at all, so a small proportional term would sit there humming.
static void controlWheel(Wheel &w, float dt) {
  // Runaway guard, checked before anything else can command duty.
  if (w.mode != 0 && fabsf(w.vel) > VEL_LIMIT) {
    stopWheel(w);
    w.approachStage = 0;
    Out.printf("FAULT,%s,overspeed,%.0f\n", w.name, w.vel);
    return;
  }
  if (w.mode == 2) {
    // ---- advance the trapezoidal profile one tick ----
    if (w.profActive) {
      double remain = (double)w.target - w.spPos;
      float dir = (remain >= 0) ? 1.0f : -1.0f;
      // distance needed to bleed off current setpoint speed at AMAX
      float stopDist = (w.spVel * w.spVel) / (2.0f * AMAX);

      if (fabs(remain) <= stopDist || fabs(remain) < 1.0) {
        w.spVel -= dir * AMAX * dt;                     // decelerate
        if (dir > 0 && w.spVel < 0) w.spVel = 0;
        if (dir < 0 && w.spVel > 0) w.spVel = 0;
      } else {
        w.spVel += dir * AMAX * dt;                     // accelerate / cruise
        w.spVel = constrain(w.spVel, -VMAX, VMAX);
      }
      w.spPos += w.spVel * dt;

      // profile has arrived: hand over to the hold phase
      if (fabs((double)w.target - w.spPos) < 1.0 && fabsf(w.spVel) < 30.0f) {
        w.spPos = (double)w.target;
        w.spVel = 0;
        w.profActive = false;
        w.holdStart = millis();
      }
    }

    float err = (float)(w.spPos - (double)w.pos);       // track the setpoint
    float finalErr = (float)(w.target - w.pos);

    // ---- finish conditions ----
    // Settle as soon as we are inside tolerance and actually stopped; and give
    // up after HOLD_MS regardless, so a 1-count miss can never become a dither.
    if (!w.profActive) {
      bool stopped = fabsf(w.vel) < 40;
      bool timeUp = (millis() - w.holdStart) > HOLD_MS;

      if (fabsf(finalErr) <= POS_TOL && stopped) {
        finishMove(w, "tol");
        return;
      }
      // A large residual means the wheel coasted well past. Re-run a short
      // profile at the remaining distance -- a controlled micro-move.
      if (fabsf(finalErr) > 20 && w.retries < MAX_RETRY && timeUp) {
        w.retries++;
        w.spPos = (double)w.pos;
        w.spVel = 0;
        w.profActive = true;
        w.holdStart = 0;
        w.integ = 0;
        w.creepState = 0;
        Out.printf("RETRY,%s,%d,%.2f\n", w.name, w.retries, finalErr * 360.0f / CPR);
      } else {
        // Pulsed creep for the last few counts.
        if (w.creepCount >= CREEP_MAX) { finishMove(w, "creepmax"); return; }
        uint32_t tnow = millis();
        if (w.creepState == 0) {
          int dir = (finalErr > 0) ? 1 : -1;
          int md = (dir > 0) ? w.minDutyFwd : w.minDutyRev;
          if (md <= 0) md = 30;
          applyDuty(w, dir * (md + CREEP_BOOST));
          w.creepUntil = tnow + CREEP_MS;
          w.creepState = 1;
        } else if (w.creepState == 1) {
          if (tnow >= w.creepUntil) {
            applyDuty(w, 0);
            w.creepUntil = tnow + CREEP_GAP;
            w.creepState = 2;
            w.creepCount++;
          }
        } else {
          applyDuty(w, 0);
          if (tnow >= w.creepUntil) w.creepState = 0;
        }
        return;
      }
    }

    // Stall guard. Inside the backlash band the motor turns but the encoder
    // does not, so the integrator would wind up and buzz the motor against
    // nothing. If we are commanding real duty and seeing no motion, bleed the
    // integral away instead of pushing harder.
    // Only a mild bleed, and only after a long stall: crossing the backlash
    // band looks exactly like a stall, and decaying hard here was stopping the
    // controller from pushing through it at all.
    int mdNow = (w.duty >= 0) ? w.minDutyFwd : w.minDutyRev;
    if (abs(w.duty) > mdNow && fabsf(w.vel) < 50.0f) {
      if (w.stallStart == 0) w.stallStart = millis();
      else if (millis() - w.stallStart > 700) w.integ *= 0.995f;
    } else {
      w.stallStart = 0;
    }

    w.integ += err * dt;
    w.integ = constrain(w.integ, -2000.0f, 2000.0f);     // anti-windup
    float deriv = (dt > 0) ? (err - w.prevErr) / dt : 0;
    w.prevErr = err;

    // Feedforward from the calibration: viscous term from the duty/speed slope
    // plus the measured stiction offset. With this carrying the motion, the PID
    // only trims small errors, so it never needs a large corrective kick.
    float ff = 0;
    if (fabsf(w.spVel) > 1.0f) {
      ff = (w.slope > 1.0f) ? (w.spVel / w.slope) : 0;      // viscous / back-EMF
      // Stiction help only while we are genuinely behind the setpoint. Adding
      // it whenever spVel is non-zero biases the output forwards and fights
      // the braking during the deceleration ramp.
      if (err * w.spVel > 0) {
        float md = (w.spVel >= 0) ? w.minDutyFwd : w.minDutyRev;
        ff += (w.spVel >= 0 ? md : -md) * 0.70f;
      }
    }

    float u = ff + Kp * err + Ki * w.integ + Kd * deriv
              + Kv * (w.spVel - w.vel);                     // active braking

    // Stiction kick, gated on distance REMAINING rather than tracking error:
    // during a small corrective move the setpoint creeps and tracking error
    // stays tiny, so a tracking-error gate never fires and the move stalls.
    // The direction test means the kick can only ever push toward the target,
    // which is what stops it turning into the old dither.
    if (fabsf(finalErr) > POS_TOL) {
      float md = (u >= 0) ? w.minDutyFwd : w.minDutyRev;
      if (md > 0 && fabsf(u) < md && u * finalErr > 0) u = (u >= 0 ? md : -md);
    }
    applyDuty(w, (int)constrain(u, -255.0f, 255.0f));

  } else if (w.mode == 3) {
    float err = w.velTarget - w.vel;
    w.integ += err * dt;
    w.integ = constrain(w.integ, -20000.0f, 20000.0f);
    float ff = (w.slope > 1) ? w.velTarget / w.slope : 0;   // feedforward
    float u = ff + 0.02f * err + 0.05f * w.integ * 0.01f;
    applyDuty(w, (int)constrain(u, -255.0f, 255.0f));
  }
}

// ---------------------------------------------------------------- calibration
// Everything here is measured, not assumed: the duty at which the wheel
// actually starts turning, and how counts/s scales with duty above it.

// Calibration blocks the main loop, so it polls for an abort key itself --
// otherwise a stop command would be ignored while a motor is spinning.
static bool calAbort = false;

static void calSettle(Wheel &w, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') { calAbort = true; applyDuty(w, 0); return; }
    }
    if (calAbort) { applyDuty(w, 0); return; }
    updateWheel(w, LOOP_US / 1e6f);
    delayMicroseconds(LOOP_US);
  }
}

static float calMeasureVel(Wheel &w, uint32_t ms) {
  int64_t p0 = w.pos;
  uint32_t t0 = millis();
  calSettle(w, ms);
  float dt = (millis() - t0) / 1000.0f;
  return (dt > 0) ? (w.pos - p0) / dt : 0;
}

static int findDeadband(Wheel &w, int sign) {
  for (int d = 6; d <= 255 && !calAbort; d += 2) {
    applyDuty(w, sign * d);
    float v = calMeasureVel(w, 90);
    if (fabsf(v) > 200) {                 // 200 counts/s ~ 3 deg per sample
      applyDuty(w, 0);
      calSettle(w, 250);
      return d;
    }
  }
  applyDuty(w, 0);
  return 0;
}

// Calibration must survive a reboot. Without it minDutyFwd/Rev are zero, the
// stiction kick is disabled, and moves silently undershoot -- which looked
// like a controller fault until the reason codes showed otherwise.
static Preferences prefs;

static void calSave(Wheel &w) {
  prefs.begin("drive", false);
  char k[8];
  snprintf(k, sizeof k, "%sdf", w.name); prefs.putInt(k, w.minDutyFwd);
  snprintf(k, sizeof k, "%sdr", w.name); prefs.putInt(k, w.minDutyRev);
  snprintf(k, sizeof k, "%ssl", w.name); prefs.putFloat(k, w.slope);
  prefs.end();
}

static bool calLoad(Wheel &w) {
  prefs.begin("drive", true);
  char k[8];
  snprintf(k, sizeof k, "%sdf", w.name); int df = prefs.getInt(k, 0);
  snprintf(k, sizeof k, "%sdr", w.name); int dr = prefs.getInt(k, 0);
  snprintf(k, sizeof k, "%ssl", w.name); float sl = prefs.getFloat(k, 0.0f);
  prefs.end();
  if (df > 0 && dr > 0) {
    w.minDutyFwd = df; w.minDutyRev = dr; w.slope = sl;
    return true;
  }
  return false;
}

static void calibrate(Wheel &w) {
  Out.printf("CALSTART,%s\n", w.name);
  calAbort = false;
  stopWheel(w);
  calSettle(w, 200);

  int fwd = findDeadband(w, +1);
  int rev = findDeadband(w, -1);
  w.minDutyFwd = fwd;
  w.minDutyRev = rev;
  Out.printf("CALDEAD,%s,%d,%d\n", w.name, fwd, rev);

  // duty -> speed curve, from just above the deadband to full scale
  int lo = max(fwd, 20);
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  int n = 0;
  for (int i = 0; i < 6 && !calAbort; i++) {
    int d = lo + (255 - lo) * i / 5;
    applyDuty(w, d);
    calSettle(w, 320);                    // let it reach steady state
    float v = calMeasureVel(w, 220);
    Out.printf("CALPT,%s,%d,%.1f\n", w.name, d, v);
    sx += d; sy += v; sxx += (float)d * d; sxy += d * v; n++;
  }
  applyDuty(w, 0);
  stopWheel(w);

  if (calAbort) { Out.printf("CALABORT,%s\n", w.name); calAbort = false; return; }

  float denom = n * sxx - sx * sx;
  w.slope = (n > 1 && fabsf(denom) > 1e-6f) ? (n * sxy - sx * sy) / denom : 0;
  Out.printf("CALSLOPE,%s,%.3f\n", w.name, w.slope);
  calSave(w);
  Out.printf("CALDONE,%s\n", w.name);
}

// ---------------------------------------------------------------- diagnostics
static bool leftOk = false, rightOk = false;

static void testUpstream() {
  Out.println(F("\n[1] UPSTREAM BUS  (mux cleared -- only 0x70 should answer)"));
  muxWrite(0x00);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (ping(a)) {
      Out.printf("      0x%02X responds", a);
      if (a == MUX_ADDR)      Out.print(F("   <- PCA9548A, correct"));
      else if (a == ENC_ADDR) Out.print(F("   <- WRONG: an AS5600 is on the upstream bus"));
      Out.println();
      found++;
    }
  }
  if (!found) Out.println(F("      nothing answered -- SDA/SCL swapped, unpowered, or no pull-ups."));
}

static void testMuxRegister() {
  Out.println(F("\n[2] MUX CONTROL REGISTER  (write a mask, read it back)"));
  const uint8_t masks[] = { 0x00, MASK_LEFT, MASK_RIGHT, 0x00 };
  for (uint8_t i = 0; i < 4; i++) {
    bool w = muxWrite(masks[i]);
    delay(2);
    int rb = muxRead();
    Out.printf("      wrote 0x%02X -> ack %s, read back 0x%02X  %s\n",
                  masks[i], w ? "yes" : "NO ", rb, (w && rb == masks[i]) ? "ok" : "MISMATCH");
  }
}

static void testResetPin() {
  Out.println(F("\n[3] MUX RESET LINE  (GPIO 5 -- pulse low, register must clear)"));
  muxWrite(MASK_LEFT);
  delay(2);
  int before = muxRead();
  digitalWrite(PIN_MUX_RESET, LOW);
  delayMicroseconds(50);
  digitalWrite(PIN_MUX_RESET, HIGH);
  delay(2);
  int after = muxRead();
  Out.printf("      before 0x%02X, after 0x%02X  -> %s\n", before, after,
                (before == MASK_LEFT && after == 0x00)
                  ? "RESET wired and working"
                  : "RESET not effective (software i2cRecover() is the fallback)");
}

static void scanAllChannels() {
  Out.println(F("\n[4] PER-CHANNEL SCAN  (all 8, so a mis-plugged channel shows up)"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    uint8_t mask = 1 << ch;
    if (!muxWrite(mask)) { Out.printf("      ch%d  mux write failed\n", ch); continue; }
    delay(2);
    bool enc = ping(ENC_ADDR);
    const char *tag = "";
    if (mask == MASK_LEFT)  tag = enc ? "  <- LEFT, as designed" : "  <- LEFT expected, MISSING";
    if (mask == MASK_RIGHT) tag = enc ? "  <- RIGHT, as designed" : "  <- RIGHT expected, MISSING";
    if (enc && mask != MASK_LEFT && mask != MASK_RIGHT) tag = "  <- unexpected device";
    Out.printf("      ch%d (mask 0x%02X)  %s%s\n", ch, mask,
                  enc ? "AS5600 @ 0x36 present" : "empty", tag);
    if (mask == MASK_LEFT)  leftOk = enc;
    if (mask == MASK_RIGHT) rightOk = enc;
  }
  muxWrite(0x00);
  delay(2);
  Out.printf("      isolation: mask 0x00 -> 0x36 %s\n",
                ping(ENC_ADDR) ? "STILL VISIBLE (not isolating)" : "gone, correct");
}

static void encoderHealth(const char *name, uint8_t mask) {
  Out.printf("\n      -- %s (mask 0x%02X) --\n", name, mask);
  if (!muxWrite(mask)) { Out.println(F("      mux select failed")); return; }
  delay(2);
  int st = as5600Read8(AS_STATUS);
  if (st < 0) { Out.println(F("      no response from 0x36")); return; }
  bool md = st & 0x20, ml = st & 0x10, mh = st & 0x08;
  Out.printf("      STATUS 0x0B = 0x%02X   MD=%d ML=%d MH=%d\n", st, md, ml, mh);
  if (!md)     Out.println(F("      -> NO MAGNET DETECTED."));
  else if (ml) Out.println(F("      -> magnet too WEAK: move it closer."));
  else if (mh) Out.println(F("      -> magnet too STRONG: move it further away."));
  else         Out.println(F("      -> magnet detected and in range."));
  Out.printf("      AGC 0x1A = %d  (aim for ~64 mid-scale at 3.3 V)\n", as5600Read8(AS_AGC));
  Out.printf("      MAGNITUDE = %d\n", as5600Read12(AS_MAGNITUDE));
  int raw = as5600Read12(AS_RAW_ANGLE);
  Out.printf("      RAW_ANGLE 0x0C = %d  (%.1f deg)\n", raw, raw * 360.0 / CPR);
}

static void testEncoders() {
  Out.println(F("\n[5] ENCODER HEALTH  (magnet placement read from registers)"));
  if (leftOk)  encoderHealth("LEFT  ch6", MASK_LEFT);  else Out.println(F("\n      -- LEFT ch6: absent --"));
  if (rightOk) encoderHealth("RIGHT ch3", MASK_RIGHT); else Out.println(F("\n      -- RIGHT ch3: absent --"));
  muxWrite(0x00);
}

// ---------------------------------------------------------------- mapping
static int readEnc(uint8_t mask) {
  if (!muxWrite(mask)) return -1;
  delayMicroseconds(400);
  return as5600Read12(AS_RAW_ANGLE);
}

static int16_t wrapDelta(int prev, int now) {
  int d = now - prev;
  if (d >  2048) d -= 4096;
  if (d < -2048) d += 4096;
  return (int16_t)d;
}

static void mapPin(uint8_t idx) {
  int lp = readEnc(MASK_LEFT), rp = readEnc(MASK_RIGHT);
  long lacc = 0, racc = 0;

  pwmSet(idx, 154);                       // ~60 %, enough to break stiction
  uint32_t t0 = millis();
  while (millis() - t0 < 1200) {
    int l = readEnc(MASK_LEFT), r = readEnc(MASK_RIGHT);
    if (l >= 0 && lp >= 0) lacc += wrapDelta(lp, l);
    if (r >= 0 && rp >= 0) racc += wrapDelta(rp, r);
    if (l >= 0) lp = l;
    if (r >= 0) rp = r;
  }
  pwmSet(idx, 0);
  delay(500);
  int l = readEnc(MASK_LEFT), r = readEnc(MASK_RIGHT);
  if (l >= 0 && lp >= 0) lacc += wrapDelta(lp, l);
  if (r >= 0 && rp >= 0) racc += wrapDelta(rp, r);

  bool lm = labs(lacc) > 40, rm = labs(racc) > 40;
  const char *v = (lm && rm) ? "BOTH -- shared wire or vibration"
                : lm ? "-> drives the LEFT wheel"
                : rm ? "-> drives the RIGHT wheel" : "-> nothing moved";
  Out.printf("      %-12s  left %+6ld  right %+6ld   %s\n", PWM_NAME[idx], lacc, racc, v);
}

static void mappingTest() {
  Out.println(F("\n[M] PWM MAPPING  (one pin at a time, both encoders watched)"));
  Out.println(F("      60% duty, 1.2 s per pin. WHEELS OFF THE GROUND."));
  for (uint8_t i = 0; i < 4; i++) mapPin(i);
  killDrive();
  Out.println(F("      PASS = each label moves its own wheel, RPWM +ve and LPWM -ve."));
}

static void summary() {
  Out.println(F("\n================ SUMMARY ================"));
  Out.printf("  PCA9548A @ 0x70 ....... %s\n", ping(MUX_ADDR) ? "PRESENT" : "MISSING");
  Out.printf("  LEFT  AS5600 ch6 ...... %s\n", leftOk  ? "PRESENT" : "MISSING");
  Out.printf("  RIGHT AS5600 ch3 ...... %s\n", rightOk ? "PRESENT" : "MISSING");
  Out.println(F("  ready -- see README for the command set"));
  Out.println(F("=========================================\n"));
}

static void runAll() {
  testUpstream();
  testMuxRegister();
  testResetPin();
  scanAllChannels();
  testEncoders();
  summary();
}

// ---------------------------------------------------------------- telemetry
static void sendTelem() {
  Out.printf("D,%lld,%lld,%.0f,%d,%lu,%lld,%lld,%.0f,%d,%lu,%d,%d\n",
                (long long)WL.pos, (long long)WL.target, WL.vel, WL.duty, WL.errCount,
                (long long)WR.pos, (long long)WR.target, WR.vel, WR.duty, WR.errCount,
                WL.mode, WR.mode);
}

static void sendStatus() {
  int la = -1, lst = -1, ra = -1, rst = -1;
  if (muxWrite(MASK_LEFT))  { delayMicroseconds(300); lst = as5600Read8(AS_STATUS); la = as5600Read8(AS_AGC); }
  if (muxWrite(MASK_RIGHT)) { delayMicroseconds(300); rst = as5600Read8(AS_STATUS); ra = as5600Read8(AS_AGC); }
  Out.printf("S,%d,%d,%d,%d,%.3f,%.3f,%.4f,%d,%d,%d,%d,%d,%.2f,%.2f\n",
                la, lst, ra, rst, Kp, Ki, Kd, POS_TOL,
                WL.minDutyFwd, WL.minDutyRev, WR.minDutyFwd, WR.minDutyRev,
                WL.slope, WR.slope);
}

// ---------------------------------------------------------------- commands
static Wheel *pickWheel(const String &s) {
  if (s == "L" || s == "l") return &WL;
  if (s == "R" || s == "r") return &WR;
  return nullptr;
}

static void moveCounts(Wheel &w, int64_t delta) {
  int64_t tgt = w.pos + delta;
  w.finalTarget = tgt;

  if (traceArmed) {                 // record this move at the full loop rate
    traceW = &w;
    traceStart = w.pos;
    traceTargetRel = delta;
    traceCount = 0;
  }

  // Arrive from the same side every time. If this move would approach from the
  // wrong direction, run past the target first and turn around.
  bool wrongWay = (APPROACH_DIR > 0 && delta < 0) || (APPROACH_DIR < 0 && delta > 0);
  if (APPROACH_DIR != 0 && wrongWay) {
    w.approachStage = 1;
    startMove(w, tgt - (int64_t)APPROACH_DIR * TAKEUP);
  } else {
    w.approachStage = 0;
    startMove(w, tgt);
  }
  Out.printf("ACK,move,%s,%.2f\n", w.name, delta * 360.0f / CPR);
}

static void moveBy(Wheel &w, float degrees) {
  moveCounts(w, (int64_t)lroundf(degrees * CPR / 360.0f));
}

// ---- differential drive ----
// Straight line: both wheels the same distance. The measured 5% speed
// difference between them is handled by each wheel's own feedforward slope,
// so equal targets really do produce a straight line.
static void driveMetres(float m) {
  lastDriveM = m;
  int64_t c = (int64_t)lroundf(m * CPM);
  moveCounts(WL, c);
  moveCounts(WR, c);
  Out.printf("ACK,drive,%.4f,%lld\n", m, (long long)c);
}

// Turn in place: the wheels counter-rotate along a circle of diameter TRACK,
// so each travels (TRACK/2) * theta. Positive is counter-clockwise seen from
// above: left wheel back, right wheel forward.
static bool turnDegrees(float deg) {
  if (TRACK_MM <= 0) {
    Out.println(F("ERR,track,set TRACK <mm> first -- a turn cannot be computed without it"));
    return false;
  }
  float arc = (TRACK_MM / 2000.0f) * deg * (float)M_PI / 180.0f;   // metres
  int64_t c = (int64_t)lroundf(arc * CPM);
  moveCounts(WL, -c);
  moveCounts(WR, +c);
  Out.printf("ACK,turn,%.2f,%lld\n", deg, (long long)c);
  return true;
}

static void updatePose() {
  if (TRACK_MM <= 0) { lastPoseL = WL.pos; lastPoseR = WR.pos; return; }
  double dl = (WL.pos - lastPoseL) / (double)CPM;
  double dr = (WR.pos - lastPoseR) / (double)CPM;
  lastPoseL = WL.pos;
  lastPoseR = WR.pos;
  if (dl == 0 && dr == 0) return;
  double dc = (dl + dr) / 2.0;
  double dth = (dr - dl) / (TRACK_MM / 1000.0);
  poseX += dc * cos(poseTh + dth / 2.0);
  poseY += dc * sin(poseTh + dth / 2.0);
  poseTh += dth;
}

// ---------------------------------------------------------------- wifi
// Credentials live in NVS, never in source: this repository is public, and a
// hardcoded SSID and password would be published and indexed. Set them once
// over USB with:  WIFI <ssid> <password>
static bool wifiEnabled = false;
static bool wifiIsAP = false;

static void serverStart() {
  tcpServer.begin();
  tcpServer.setNoDelay(true);      // telemetry is small and frequent
  ArduinoOTA.setHostname("drive-harness");
  ArduinoOTA.onStart([]() {        // never flash while a wheel is turning
    stopAll();
    Out.println(F("OTA,start"));
  });
  ArduinoOTA.onEnd([]() { Out.println(F("OTA,done")); });
  ArduinoOTA.begin();
}

static void wifiStart() {
  prefs.begin("drive", true);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  uint8_t wmode = prefs.getUChar("wmode", 1);    // 1 = join a network, 2 = be one
  prefs.end();

  if (!ssid.length()) {
    Out.println(F("WIFI,none,set with: WIFIAP <ssid> <password>  (or WIFI <ssid> <password> to join a router)"));
    return;
  }

  if (wmode == 2) {
    // Access point: the car carries its own network, so it works anywhere and
    // the address never changes. WPA2 needs 8+ characters; shorter means open.
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    // Deliberately NOT the 192.168.4.x default: that subnet collides with
    // static routes on this laptop's wired network, so traffic for the bot got
    // routed out of the ethernet port instead of over WiFi.
    WiFi.softAPConfig(IPAddress(10, 42, 7, 1), IPAddress(10, 42, 7, 1),
                      IPAddress(255, 255, 255, 0));
    bool ok = (pass.length() >= 8)
                ? WiFi.softAP(ssid.c_str(), pass.c_str())
                : WiFi.softAP(ssid.c_str());
    if (ok) {
      wifiEnabled = true;
      wifiIsAP = true;
      serverStart();
      Out.printf("WIFI,ap,%s,%s,%d,%s\n", ssid.c_str(),
                 WiFi.softAPIP().toString().c_str(), TCP_PORT,
                 pass.length() >= 8 ? "wpa2" : "OPEN");
    } else {
      Out.println(F("WIFI,failed,softAP did not start -- USB serial still works"));
    }
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);            // sleep adds latency spikes to the link
  WiFi.begin(ssid.c_str(), pass.c_str());
  Out.printf("WIFI,connecting,%s\n", ssid.c_str());

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(200);

  if (WiFi.status() == WL_CONNECTED) {
    wifiEnabled = true;
    wifiIsAP = false;
    serverStart();
    Out.printf("WIFI,ok,%s,%d,%d\n", WiFi.localIP().toString().c_str(),
               TCP_PORT, WiFi.RSSI());
  } else {
    Out.println(F("WIFI,failed,continuing on USB serial only"));
  }
}

// Link-loss failsafe. A car that keeps executing its last command after losing
// contact is how hardware gets broken.
static void wifiService() {
  if (!wifiEnabled) return;
  ArduinoOTA.handle();

  if (!tcpClient || !tcpClient.connected()) {
    if (tcpClient) {                       // a client just went away
      tcpClient.stop();
      if (WL.mode != 0 || WR.mode != 0) {
        stopAll();
        Out.println(F("FAULT,link,client lost -- motors stopped"));
      }
    }
    WiFiClient c = tcpServer.available();
    if (c) {
      tcpClient = c;
      tcpClient.setNoDelay(true);
      Out.printf("LINK,up,%s\n", tcpClient.remoteIP().toString().c_str());
    }
  }

  // In AP mode there is no upstream link to lose, so WiFi.status() is not
  // meaningful -- the client-disconnect check above is the failsafe there.
  if (!wifiIsAP && WiFi.status() != WL_CONNECTED && (WL.mode != 0 || WR.mode != 0)) {
    stopAll();
    Out.println(F("FAULT,link,wifi dropped -- motors stopped"));
  }
}

static void geomSave() {
  prefs.begin("drive", false);
  prefs.putFloat("cpm", CPM);
  prefs.putFloat("track", TRACK_MM);
  prefs.end();
}

static void handleLine(String line) {
  line.trim();
  if (!line.length()) return;

  // legacy single-key diagnostics
  if (line.length() == 1) {
    char c = line[0];
    if (c == 't') { telem = !telem; return; }
    if (c == 'r') { stopAll(); runAll(); return; }
    if (c == 'm') { stopAll(); mappingTest(); return; }
    if (c == 'x' || c == 's') { stopAll(); Out.println(F("ACK,stop")); return; }
    if (c >= '1' && c <= '4') { stopAll(); mapPin(c - '1'); return; }
  }

  String tok[5];
  int n = 0, i = 0;
  while (n < 5 && i < (int)line.length()) {
    int sp = line.indexOf(' ', i);
    if (sp < 0) sp = line.length();
    tok[n++] = line.substring(i, sp);
    i = sp + 1;
  }
  String cmd = tok[0];
  cmd.toUpperCase();

  if (cmd == "P" && n >= 3) {                 // P <L|R> <degrees>  relative move
    Wheel *w = pickWheel(tok[1]);
    if (w) moveBy(*w, tok[2].toFloat());
  } else if (cmd == "PB" && n >= 2) {         // PB <degrees>  both wheels
    float d = tok[1].toFloat();
    moveBy(WL, d);
    moveBy(WR, d);
  } else if (cmd == "D" && n >= 3) {          // D <L|R> <-255..255>  open loop
    Wheel *w = pickWheel(tok[1]);
    if (w) { w->mode = 1; applyDuty(*w, tok[2].toInt()); Out.printf("ACK,duty,%s,%d\n", w->name, w->duty); }
  } else if (cmd == "V" && n >= 3) {          // V <L|R> <counts/s>
    Wheel *w = pickWheel(tok[1]);
    if (w) { w->mode = 3; w->integ = 0; w->velTarget = tok[2].toFloat(); Out.printf("ACK,vel,%s,%.0f\n", w->name, w->velTarget); }
  } else if (cmd == "C" && n >= 2) {          // C <L|R>  calibrate
    Wheel *w = pickWheel(tok[1]);
    if (w) calibrate(*w);
  } else if (cmd == "K" && n >= 4) {          // K <kp> <ki> <kd> [kv]
    Kp = tok[1].toFloat(); Ki = tok[2].toFloat(); Kd = tok[3].toFloat();
    // Kv is clamped: the sweep measured 258 deg of overshoot at Kv 0.14, so
    // values above the stable ceiling are not accepted from the console.
    if (n >= 5) Kv = constrain(tok[4].toFloat(), 0.0f, 0.070f);
    Out.printf("ACK,gains,%.3f,%.3f,%.4f,%.4f\n", Kp, Ki, Kd, Kv);
  } else if (cmd == "APPROACH" && n >= 3) {   // APPROACH <0|1|-1> <takeup>
    APPROACH_DIR = tok[1].toInt();
    TAKEUP = tok[2].toInt();
    Out.printf("ACK,approach,%d,%d\n", APPROACH_DIR, TAKEUP);
  } else if (cmd == "TOL" && n >= 2) {
    POS_TOL = tok[1].toInt();
    Out.printf("ACK,tol,%d\n", POS_TOL);
  } else if (cmd == "PROF" && n >= 3) {       // PROF <vmax> <amax>
    VMAX = tok[1].toFloat();
    AMAX = tok[2].toFloat();
    Out.printf("ACK,prof,%.0f,%.0f\n", VMAX, AMAX);
  } else if (cmd == "TRACE") {
    traceArmed = !traceArmed;
    traceCount = 0;
    traceW = nullptr;
    Out.printf("ACK,trace,%d\n", traceArmed ? 1 : 0);
  } else if (cmd == "DUMP") {
    Out.printf("TRC,%u,%lld\n", traceCount, (long long)traceTargetRel);
    for (uint16_t i = 0; i < traceCount; i += 20) {
      Out.printf("TD,%u", i);
      for (uint16_t j = i; j < i + 20 && j < traceCount; j++)
        Out.printf(",%ld", (long)traceBuf[j]);
      Out.println();
    }
    Out.println(F("TRCEND"));
  } else if (cmd == "DRIVE" && n >= 2) {      // DRIVE <metres>
    driveMetres(tok[1].toFloat());
  } else if (cmd == "TURN" && n >= 2) {       // TURN <degrees, +ve = CCW>
    turnDegrees(tok[1].toFloat());
  } else if (cmd == "SPIN" && n >= 2) {
    // In-place turn expressed in WHEEL degrees, not robot heading. Needs no
    // geometry, so it works before TRACK has been measured -- the wheels
    // counter-rotate by a known amount even if the resulting heading change
    // is not yet known.
    float d = tok[1].toFloat();
    int64_t c = (int64_t)lroundf(d * CPR / 360.0f);
    moveCounts(WL, -c);
    moveCounts(WR, +c);
    Out.printf("ACK,spin,%.2f\n", d);
  } else if (cmd == "POSE") {
    Out.printf("POSE,%.4f,%.4f,%.2f,%.4f,%.4f\n", poseX, poseY,
                  poseTh * 180.0 / M_PI, WL.pos / (double)CPM, WR.pos / (double)CPM);
  } else if (cmd == "POSERST") {
    poseX = poseY = poseTh = 0;
    lastPoseL = WL.pos;
    lastPoseR = WR.pos;
    Out.println(F("ACK,posrst"));
  } else if (cmd == "CPM" && n >= 2) {
    CPM = tok[1].toFloat();
    geomSave();
    Out.printf("ACK,cpm,%.1f\n", CPM);
  } else if (cmd == "TRACK" && n >= 2) {
    TRACK_MM = tok[1].toFloat();
    geomSave();
    Out.printf("ACK,track,%.1f\n", TRACK_MM);
  } else if (cmd == "CALDIST" && n >= 2) {    // CALDIST <actual metres travelled>
    float actual = tok[1].toFloat();
    if (lastDriveM != 0 && actual > 0) {
      float old = CPM;
      CPM = CPM * (lastDriveM / actual);      // travelled too far -> fewer counts/m
      geomSave();
      Out.printf("ACK,caldist,%.1f,%.1f,%.2f\n", old, CPM,
                    (CPM - old) / old * 100.0f);
    } else {
      Out.println(F("ERR,caldist,run DRIVE <m> first, then pass the measured distance"));
    }
  } else if ((cmd == "WIFI" || cmd == "WIFIAP") && n >= 3) {
    // Password may contain spaces, so take everything after the SSID verbatim
    // rather than relying on the tokeniser.
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    String ssid = line.substring(sp1 + 1, sp2);
    String pass = line.substring(sp2 + 1);
    uint8_t wmode = (cmd == "WIFIAP") ? 2 : 1;
    prefs.begin("drive", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putUChar("wmode", wmode);
    prefs.end();
    Out.printf("ACK,wifi,%s,%s,%d chars,reboot to apply\n",
               wmode == 2 ? "ap" : "station", ssid.c_str(), pass.length());
    if (wmode == 2 && pass.length() < 8)
      Out.println(F("WARN,ap password under 8 chars -- the network will be OPEN"));
  } else if (cmd == "WIFIQ") {
    Out.printf("WIFI,%s,%s,%s,%d,%s,%d\n",
               wifiEnabled ? (wifiIsAP ? "ap" : "station") : "down",
               wifiEnabled ? (wifiIsAP ? WiFi.softAPIP().toString().c_str()
                                       : WiFi.localIP().toString().c_str()) : "-",
               wifiIsAP ? "-" : String(WiFi.RSSI()).c_str(),
               wifiIsAP ? WiFi.softAPgetStationNum() : 0,
               (tcpClient && tcpClient.connected()) ? "client" : "noclient",
               TCP_PORT);
  } else if (cmd == "WIFICLR") {
    prefs.begin("drive", false);
    prefs.remove("ssid");
    prefs.remove("pass");
    prefs.end();
    Out.println(F("ACK,wificlr,reboot to apply"));
  } else if (cmd == "REBOOT") {
    stopAll();
    Out.println(F("ACK,reboot"));
    delay(120);
    ESP.restart();
  } else if (cmd == "GEOM") {
    Out.printf("GEOM,%.1f,%.1f,%.4f\n", CPM, TRACK_MM, lastDriveM);
  } else if (cmd == "CALQ") {                 // report calibration state on demand
    Out.printf("CALLOAD,L,%d,%d,%d,%.2f\n",
                  (WL.minDutyFwd > 0 && WL.minDutyRev > 0) ? 1 : 0,
                  WL.minDutyFwd, WL.minDutyRev, WL.slope);
    Out.printf("CALLOAD,R,%d,%d,%d,%.2f\n",
                  (WR.minDutyFwd > 0 && WR.minDutyRev > 0) ? 1 : 0,
                  WR.minDutyFwd, WR.minDutyRev, WR.slope);
  } else if (cmd == "CALCLR") {               // forget stored calibration
    prefs.begin("drive", false);
    prefs.clear();
    prefs.end();
    WL.minDutyFwd = WL.minDutyRev = WR.minDutyFwd = WR.minDutyRev = 0;
    WL.slope = WR.slope = 0;
    Out.println(F("ACK,calclr"));
  } else if (cmd == "Z") {                    // zero both positions
    WL.pos = WL.target = 0; WR.pos = WR.target = 0;
    Out.println(F("ACK,zero"));
  } else if (cmd == "X") {
    stopAll();
    Out.println(F("ACK,stop"));
  } else if (cmd == "T") {
    telem = !telem;
  } else {
    Out.printf("ERR,unknown,%s\n", line.c_str());
  }
}

// ---------------------------------------------------------------- main
void setup() {
  for (uint8_t i = 0; i < 4; i++) { pinMode(PWM_PIN[i], OUTPUT); digitalWrite(PWM_PIN[i], LOW); }
  pinMode(PIN_MUX_RESET, OUTPUT);
  digitalWrite(PIN_MUX_RESET, HIGH);
  pwmInit();
  killDrive();

  Serial.begin(115200);
  delay(400);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  Out.println(F("\n\n########## DRIVE HARNESS ##########"));
  Out.println(F("Motor rail live, EN strapped to VCC -- drivers always enabled."));

  prefs.begin("drive", true);
  CPM = prefs.getFloat("cpm", CPM);
  TRACK_MM = prefs.getFloat("track", 0.0f);
  prefs.end();
  Out.printf("GEOM,%.1f,%.1f,0.0\n", CPM, TRACK_MM);
  if (TRACK_MM <= 0)
    Out.println(F("WARNING: TRACK not set -- TURN and pose are unavailable. Use: TRACK <mm>"));

  bool cl = calLoad(WL), cr = calLoad(WR);
  Out.printf("CALLOAD,L,%d,%d,%d,%.2f\n", cl ? 1 : 0, WL.minDutyFwd, WL.minDutyRev, WL.slope);
  Out.printf("CALLOAD,R,%d,%d,%d,%.2f\n", cr ? 1 : 0, WR.minDutyFwd, WR.minDutyRev, WR.slope);
  if (!cl || !cr)
    Out.println(F("WARNING: not calibrated -- run C L and C R, or moves will undershoot."));

  runAll();
  wifiStart();
}

void loop() {
  static String buf;
  static uint32_t lastLoop = 0, lastTelem = 0, lastStat = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleLine(buf); buf = ""; }
    else if (buf.length() < 90) buf += c;
  }

  wifiService();
  static String tbuf;
  if (tcpClient && tcpClient.connected()) {
    while (tcpClient.available()) {
      char c = tcpClient.read();
      if (c == '\n' || c == '\r') { handleLine(tbuf); tbuf = ""; }
      else if (tbuf.length() < 90) tbuf += c;
    }
  }

  uint32_t now = micros();
  if (now - lastLoop >= LOOP_US) {
    float dt = (now - lastLoop) / 1e6f;
    if (dt > 0.2f) dt = LOOP_US / 1e6f;       // first pass or after a blocking call
    lastLoop = now;

    updateWheel(WL, dt);
    updateWheel(WR, dt);
    controlWheel(WL, dt);
    controlWheel(WR, dt);

    updatePose();

    if (traceW && traceCount < TRACE_N)
      traceBuf[traceCount++] = (int32_t)(traceW->pos - traceStart);
  }

  uint32_t ms = millis();
  if (telem && ms - lastTelem >= 50)  { lastTelem = ms; sendTelem(); }
  if (telem && ms - lastStat  >= 700) { lastStat  = ms; sendStatus(); }
}
