// drive-harness-diag -- bring-up, closed-loop control and calibration for the
// ESP32 / BTS7960 / AS5600 / PCA9548A drive harness.
//
// Safety: the motor rail is live and R_EN/L_EN are strapped to VCC, so both
// drivers are permanently enabled. Only the PWM duty being zero keeps the
// motors still. Every path that ends a move drives duty to zero.

#include <Arduino.h>
#include <Wire.h>

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
static bool  telem = false;

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

// PID with a stiction kick: below the measured deadband the motor makes no
// torque at all, so a small proportional term would sit there humming.
static void controlWheel(Wheel &w, float dt) {
  if (w.mode == 2) {
    float err = (float)(w.target - w.pos);
    if (fabsf(err) <= POS_TOL && fabsf(w.vel) < 60) {
      if (!w.settled) {
        if (w.settleStart == 0) w.settleStart = millis();
        else if (millis() - w.settleStart > 150) {
          w.settled = true;
          applyDuty(w, 0);
          Serial.printf("DONE,%s,%lld,%.2f\n", w.name,
                        (long long)(w.target - w.pos),
                        (w.target - w.pos) * 360.0f / CPR);
          w.mode = 0;
          return;
        }
      }
      applyDuty(w, 0);
      return;
    }
    w.settleStart = 0;

    w.integ += err * dt;
    w.integ = constrain(w.integ, -4000.0f, 4000.0f);      // anti-windup
    float deriv = (dt > 0) ? (err - w.prevErr) / dt : 0;
    w.prevErr = err;

    float u = Kp * err + Ki * w.integ + Kd * deriv;
    int md = (u >= 0) ? w.minDutyFwd : w.minDutyRev;
    if (md > 0 && fabsf(u) < md) u = (u >= 0 ? md : -md);  // kick past stiction
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

static void calibrate(Wheel &w) {
  Serial.printf("CALSTART,%s\n", w.name);
  calAbort = false;
  stopWheel(w);
  calSettle(w, 200);

  int fwd = findDeadband(w, +1);
  int rev = findDeadband(w, -1);
  w.minDutyFwd = fwd;
  w.minDutyRev = rev;
  Serial.printf("CALDEAD,%s,%d,%d\n", w.name, fwd, rev);

  // duty -> speed curve, from just above the deadband to full scale
  int lo = max(fwd, 20);
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  int n = 0;
  for (int i = 0; i < 6 && !calAbort; i++) {
    int d = lo + (255 - lo) * i / 5;
    applyDuty(w, d);
    calSettle(w, 320);                    // let it reach steady state
    float v = calMeasureVel(w, 220);
    Serial.printf("CALPT,%s,%d,%.1f\n", w.name, d, v);
    sx += d; sy += v; sxx += (float)d * d; sxy += d * v; n++;
  }
  applyDuty(w, 0);
  stopWheel(w);

  if (calAbort) { Serial.printf("CALABORT,%s\n", w.name); calAbort = false; return; }

  float denom = n * sxx - sx * sx;
  w.slope = (n > 1 && fabsf(denom) > 1e-6f) ? (n * sxy - sx * sy) / denom : 0;
  Serial.printf("CALSLOPE,%s,%.3f\n", w.name, w.slope);
  Serial.printf("CALDONE,%s\n", w.name);
}

// ---------------------------------------------------------------- diagnostics
static bool leftOk = false, rightOk = false;

static void testUpstream() {
  Serial.println(F("\n[1] UPSTREAM BUS  (mux cleared -- only 0x70 should answer)"));
  muxWrite(0x00);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (ping(a)) {
      Serial.printf("      0x%02X responds", a);
      if (a == MUX_ADDR)      Serial.print(F("   <- PCA9548A, correct"));
      else if (a == ENC_ADDR) Serial.print(F("   <- WRONG: an AS5600 is on the upstream bus"));
      Serial.println();
      found++;
    }
  }
  if (!found) Serial.println(F("      nothing answered -- SDA/SCL swapped, unpowered, or no pull-ups."));
}

static void testMuxRegister() {
  Serial.println(F("\n[2] MUX CONTROL REGISTER  (write a mask, read it back)"));
  const uint8_t masks[] = { 0x00, MASK_LEFT, MASK_RIGHT, 0x00 };
  for (uint8_t i = 0; i < 4; i++) {
    bool w = muxWrite(masks[i]);
    delay(2);
    int rb = muxRead();
    Serial.printf("      wrote 0x%02X -> ack %s, read back 0x%02X  %s\n",
                  masks[i], w ? "yes" : "NO ", rb, (w && rb == masks[i]) ? "ok" : "MISMATCH");
  }
}

static void testResetPin() {
  Serial.println(F("\n[3] MUX RESET LINE  (GPIO 5 -- pulse low, register must clear)"));
  muxWrite(MASK_LEFT);
  delay(2);
  int before = muxRead();
  digitalWrite(PIN_MUX_RESET, LOW);
  delayMicroseconds(50);
  digitalWrite(PIN_MUX_RESET, HIGH);
  delay(2);
  int after = muxRead();
  Serial.printf("      before 0x%02X, after 0x%02X  -> %s\n", before, after,
                (before == MASK_LEFT && after == 0x00)
                  ? "RESET wired and working"
                  : "RESET not effective (software i2cRecover() is the fallback)");
}

static void scanAllChannels() {
  Serial.println(F("\n[4] PER-CHANNEL SCAN  (all 8, so a mis-plugged channel shows up)"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    uint8_t mask = 1 << ch;
    if (!muxWrite(mask)) { Serial.printf("      ch%d  mux write failed\n", ch); continue; }
    delay(2);
    bool enc = ping(ENC_ADDR);
    const char *tag = "";
    if (mask == MASK_LEFT)  tag = enc ? "  <- LEFT, as designed" : "  <- LEFT expected, MISSING";
    if (mask == MASK_RIGHT) tag = enc ? "  <- RIGHT, as designed" : "  <- RIGHT expected, MISSING";
    if (enc && mask != MASK_LEFT && mask != MASK_RIGHT) tag = "  <- unexpected device";
    Serial.printf("      ch%d (mask 0x%02X)  %s%s\n", ch, mask,
                  enc ? "AS5600 @ 0x36 present" : "empty", tag);
    if (mask == MASK_LEFT)  leftOk = enc;
    if (mask == MASK_RIGHT) rightOk = enc;
  }
  muxWrite(0x00);
  delay(2);
  Serial.printf("      isolation: mask 0x00 -> 0x36 %s\n",
                ping(ENC_ADDR) ? "STILL VISIBLE (not isolating)" : "gone, correct");
}

static void encoderHealth(const char *name, uint8_t mask) {
  Serial.printf("\n      -- %s (mask 0x%02X) --\n", name, mask);
  if (!muxWrite(mask)) { Serial.println(F("      mux select failed")); return; }
  delay(2);
  int st = as5600Read8(AS_STATUS);
  if (st < 0) { Serial.println(F("      no response from 0x36")); return; }
  bool md = st & 0x20, ml = st & 0x10, mh = st & 0x08;
  Serial.printf("      STATUS 0x0B = 0x%02X   MD=%d ML=%d MH=%d\n", st, md, ml, mh);
  if (!md)     Serial.println(F("      -> NO MAGNET DETECTED."));
  else if (ml) Serial.println(F("      -> magnet too WEAK: move it closer."));
  else if (mh) Serial.println(F("      -> magnet too STRONG: move it further away."));
  else         Serial.println(F("      -> magnet detected and in range."));
  Serial.printf("      AGC 0x1A = %d  (aim for ~64 mid-scale at 3.3 V)\n", as5600Read8(AS_AGC));
  Serial.printf("      MAGNITUDE = %d\n", as5600Read12(AS_MAGNITUDE));
  int raw = as5600Read12(AS_RAW_ANGLE);
  Serial.printf("      RAW_ANGLE 0x0C = %d  (%.1f deg)\n", raw, raw * 360.0 / CPR);
}

static void testEncoders() {
  Serial.println(F("\n[5] ENCODER HEALTH  (magnet placement read from registers)"));
  if (leftOk)  encoderHealth("LEFT  ch6", MASK_LEFT);  else Serial.println(F("\n      -- LEFT ch6: absent --"));
  if (rightOk) encoderHealth("RIGHT ch3", MASK_RIGHT); else Serial.println(F("\n      -- RIGHT ch3: absent --"));
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
  Serial.printf("      %-12s  left %+6ld  right %+6ld   %s\n", PWM_NAME[idx], lacc, racc, v);
}

static void mappingTest() {
  Serial.println(F("\n[M] PWM MAPPING  (one pin at a time, both encoders watched)"));
  Serial.println(F("      60% duty, 1.2 s per pin. WHEELS OFF THE GROUND."));
  for (uint8_t i = 0; i < 4; i++) mapPin(i);
  killDrive();
  Serial.println(F("      PASS = each label moves its own wheel, RPWM +ve and LPWM -ve."));
}

static void summary() {
  Serial.println(F("\n================ SUMMARY ================"));
  Serial.printf("  PCA9548A @ 0x70 ....... %s\n", ping(MUX_ADDR) ? "PRESENT" : "MISSING");
  Serial.printf("  LEFT  AS5600 ch6 ...... %s\n", leftOk  ? "PRESENT" : "MISSING");
  Serial.printf("  RIGHT AS5600 ch3 ...... %s\n", rightOk ? "PRESENT" : "MISSING");
  Serial.println(F("  ready -- see README for the command set"));
  Serial.println(F("=========================================\n"));
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
  Serial.printf("D,%lld,%lld,%.0f,%d,%lu,%lld,%lld,%.0f,%d,%lu,%d,%d\n",
                (long long)WL.pos, (long long)WL.target, WL.vel, WL.duty, WL.errCount,
                (long long)WR.pos, (long long)WR.target, WR.vel, WR.duty, WR.errCount,
                WL.mode, WR.mode);
}

static void sendStatus() {
  int la = -1, lst = -1, ra = -1, rst = -1;
  if (muxWrite(MASK_LEFT))  { delayMicroseconds(300); lst = as5600Read8(AS_STATUS); la = as5600Read8(AS_AGC); }
  if (muxWrite(MASK_RIGHT)) { delayMicroseconds(300); rst = as5600Read8(AS_STATUS); ra = as5600Read8(AS_AGC); }
  Serial.printf("S,%d,%d,%d,%d,%.3f,%.3f,%.4f,%d,%d,%d,%d,%d,%.2f,%.2f\n",
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

static void moveBy(Wheel &w, float degrees) {
  w.target = w.pos + (int64_t)lroundf(degrees * CPR / 360.0f);
  w.integ = 0;
  w.prevErr = 0;
  w.settled = false;
  w.settleStart = 0;
  w.mode = 2;
  Serial.printf("ACK,move,%s,%.2f\n", w.name, degrees);
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
    if (c == 'x' || c == 's') { stopAll(); Serial.println(F("ACK,stop")); return; }
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
    if (w) { w->mode = 1; applyDuty(*w, tok[2].toInt()); Serial.printf("ACK,duty,%s,%d\n", w->name, w->duty); }
  } else if (cmd == "V" && n >= 3) {          // V <L|R> <counts/s>
    Wheel *w = pickWheel(tok[1]);
    if (w) { w->mode = 3; w->integ = 0; w->velTarget = tok[2].toFloat(); Serial.printf("ACK,vel,%s,%.0f\n", w->name, w->velTarget); }
  } else if (cmd == "C" && n >= 2) {          // C <L|R>  calibrate
    Wheel *w = pickWheel(tok[1]);
    if (w) calibrate(*w);
  } else if (cmd == "K" && n >= 4) {          // K <kp> <ki> <kd>
    Kp = tok[1].toFloat(); Ki = tok[2].toFloat(); Kd = tok[3].toFloat();
    Serial.printf("ACK,gains,%.3f,%.3f,%.4f\n", Kp, Ki, Kd);
  } else if (cmd == "TOL" && n >= 2) {
    POS_TOL = tok[1].toInt();
    Serial.printf("ACK,tol,%d\n", POS_TOL);
  } else if (cmd == "Z") {                    // zero both positions
    WL.pos = WL.target = 0; WR.pos = WR.target = 0;
    Serial.println(F("ACK,zero"));
  } else if (cmd == "X") {
    stopAll();
    Serial.println(F("ACK,stop"));
  } else if (cmd == "T") {
    telem = !telem;
  } else {
    Serial.printf("ERR,unknown,%s\n", line.c_str());
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

  Serial.println(F("\n\n########## DRIVE HARNESS ##########"));
  Serial.println(F("Motor rail live, EN strapped to VCC -- drivers always enabled."));
  runAll();
}

void loop() {
  static String buf;
  static uint32_t lastLoop = 0, lastTelem = 0, lastStat = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleLine(buf); buf = ""; }
    else if (buf.length() < 60) buf += c;
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
  }

  uint32_t ms = millis();
  if (telem && ms - lastTelem >= 50)  { lastTelem = ms; sendTelem(); }
  if (telem && ms - lastStat  >= 700) { lastStat  = ms; sendStatus(); }
}
