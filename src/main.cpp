// drive-harness-diag — connection verifier for the ESP32 / PCA9548A / AS5600 / BTS7960 harness.
//
// Safety: the motor rail is live. Every driver pin is forced LOW before anything
// else runs, and nothing in the automatic test sequence ever raises them. Motor
// motion happens only on an explicit serial command.

#include <Arduino.h>
#include <Wire.h>

// ---- harness pin map (matches the wiring reference) ----
#define PIN_SDA        21
#define PIN_SCL        22
#define PIN_MUX_RESET   5

// AS BUILT, verified by the [M] mapping test on 2026-08-13 -- the two driver
// connectors are on the opposite GPIOs from the original wiring reference.
// Do not "correct" these back to 25/26=left without re-running [M].
//   GPIO 32/33 -> LEFT  driver   (encoder ch6 responds)
//   GPIO 25/26 -> RIGHT driver   (encoder ch3 responds)
// RPWM on either side produces increasing encoder counts.
#define PIN_L_RPWM     32
#define PIN_L_LPWM     33
#define PIN_R_RPWM     25
#define PIN_R_LPWM     26
#define PIN_EN         27   // NOT CONNECTED -- R_EN/L_EN are strapped to VCC

#define MUX_ADDR     0x70
#define ENC_ADDR     0x36
#define MASK_LEFT    0x40   // channel 6
#define MASK_RIGHT   0x08   // channel 3

// AS5600 registers
#define AS_STATUS    0x0B
#define AS_RAW_ANGLE 0x0C
#define AS_AGC       0x1A
#define AS_MAGNITUDE 0x1B

static bool leftOk = false, rightOk = false;

static void killDrive() {
  pinMode(PIN_EN, OUTPUT);     digitalWrite(PIN_EN, LOW);
  pinMode(PIN_L_RPWM, OUTPUT); digitalWrite(PIN_L_RPWM, LOW);
  pinMode(PIN_L_LPWM, OUTPUT); digitalWrite(PIN_L_LPWM, LOW);
  pinMode(PIN_R_RPWM, OUTPUT); digitalWrite(PIN_R_RPWM, LOW);
  pinMode(PIN_R_LPWM, OUTPUT); digitalWrite(PIN_R_LPWM, LOW);
}

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

// ---------------------------------------------------------------- test 1
static void testUpstream() {
  Serial.println(F("\n[1] UPSTREAM BUS  (mux cleared -- only 0x70 should answer)"));
  muxWrite(0x00);
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (ping(a)) {
      Serial.printf("      0x%02X responds", a);
      if (a == MUX_ADDR)      Serial.print(F("   <- PCA9548A, correct"));
      else if (a == ENC_ADDR) Serial.print(F("   <- WRONG: an AS5600 is on the upstream bus, not behind the mux"));
      Serial.println();
      found++;
    }
  }
  if (!found) {
    Serial.println(F("      nothing answered."));
    Serial.println(F("      -> SDA/SCL swapped, mux unpowered, or no 4.7k pull-ups to 3V3."));
  }
}

// ---------------------------------------------------------------- test 2
static void testMuxRegister() {
  Serial.println(F("\n[2] MUX CONTROL REGISTER  (write a mask, read it back)"));
  const uint8_t masks[] = {0x00, MASK_LEFT, MASK_RIGHT, 0x00};
  for (uint8_t i = 0; i < 4; i++) {
    bool w = muxWrite(masks[i]);
    delay(2);
    int rb = muxRead();
    Serial.printf("      wrote 0x%02X -> ack %s, read back 0x%02X  %s\n",
                  masks[i], w ? "yes" : "NO ", rb,
                  (w && rb == masks[i]) ? "ok" : "MISMATCH");
  }
}

// ---------------------------------------------------------------- test 3
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
  Serial.printf("      before pulse 0x%02X, after pulse 0x%02X  -> %s\n",
                before, after,
                (before == MASK_LEFT && after == 0x00)
                  ? "RESET wired and working"
                  : "RESET not effective (GPIO 5 not landing on the mux RESET pin)");
}

// ---------------------------------------------------------------- test 4
static void scanAllChannels() {
  Serial.println(F("\n[4] PER-CHANNEL SCAN  (all 8, so a mis-plugged channel shows up)"));
  for (uint8_t ch = 0; ch < 8; ch++) {
    uint8_t mask = 1 << ch;
    if (!muxWrite(mask)) { Serial.printf("      ch%d  mux write failed\n", ch); continue; }
    delay(2);
    bool enc = ping(ENC_ADDR);
    const char *tag = "";
    if (mask == MASK_LEFT)  tag = enc ? "  <- LEFT, as designed" : "  <- LEFT expected here, MISSING";
    if (mask == MASK_RIGHT) tag = enc ? "  <- RIGHT, as designed" : "  <- RIGHT expected here, MISSING";
    if (enc && mask != MASK_LEFT && mask != MASK_RIGHT) tag = "  <- unexpected device on this channel";
    Serial.printf("      ch%d (mask 0x%02X)  %s%s\n", ch, mask,
                  enc ? "AS5600 @ 0x36 present" : "empty", tag);
    if (mask == MASK_LEFT)  leftOk  = enc;
    if (mask == MASK_RIGHT) rightOk = enc;
  }
  muxWrite(0x00);
  delay(2);
  Serial.printf("      isolation check: mask 0x00 -> 0x36 %s\n",
                ping(ENC_ADDR) ? "STILL VISIBLE (mux not isolating)" : "gone, correct");
}

// ---------------------------------------------------------------- test 5
static void encoderHealth(const char *name, uint8_t mask) {
  Serial.printf("\n      -- %s (mask 0x%02X) --\n", name, mask);
  if (!muxWrite(mask)) { Serial.println(F("      mux select failed")); return; }
  delay(2);
  int st = as5600Read8(AS_STATUS);
  if (st < 0) { Serial.println(F("      no response from 0x36")); return; }
  bool md = st & 0x20, ml = st & 0x10, mh = st & 0x08;
  Serial.printf("      STATUS 0x0B = 0x%02X   MD=%d ML=%d MH=%d\n", st, md, ml, mh);
  if (!md)      Serial.println(F("      -> NO MAGNET DETECTED. Magnet missing, too far, or not centred on the shaft."));
  else if (ml)  Serial.println(F("      -> magnet too WEAK: move it closer (target ~1-2 mm)."));
  else if (mh)  Serial.println(F("      -> magnet too STRONG: move it further away."));
  else          Serial.println(F("      -> magnet detected and in range."));

  int agc = as5600Read8(AS_AGC);
  int mag = as5600Read12(AS_MAGNITUDE);
  int raw = as5600Read12(AS_RAW_ANGLE);
  Serial.printf("      AGC 0x1A = %d  (aim for ~64 mid-scale on a 3.3 V supply)\n", agc);
  Serial.printf("      MAGNITUDE = %d\n", mag);
  Serial.printf("      RAW_ANGLE 0x0C = %d  (%.1f deg)\n", raw, raw * 360.0 / 4096.0);
}

static void testEncoders() {
  Serial.println(F("\n[5] ENCODER HEALTH  (magnet placement read from registers, not by eye)"));
  if (leftOk)  encoderHealth("LEFT  ch6", MASK_LEFT);  else Serial.println(F("\n      -- LEFT ch6: skipped, not present --"));
  if (rightOk) encoderHealth("RIGHT ch3", MASK_RIGHT); else Serial.println(F("\n      -- RIGHT ch3: skipped, not present --"));
  muxWrite(0x00);
}

// ---------------------------------------------------------------- test 6
static void testDrivePins() {
  Serial.println(F("\n[6] DRIVER PIN CONTINUITY"));
  Serial.println(F("      SKIPPED BY DESIGN."));
  Serial.println(F("      R_EN/L_EN are hard-wired to VCC on this harness, so both drivers are"));
  Serial.println(F("      permanently enabled. The pull-up probe this test used would raise an"));
  Serial.println(F("      IN pin and actually drive the motor. The only safe proof that the PWM"));
  Serial.println(F("      wiring is correct is the spin test -- command 'l' or 'k'."));
  killDrive();
}

// ---------------------------------------------------------------- summary
static void summary() {
  Serial.println(F("\n================ SUMMARY ================"));
  Serial.printf("  PCA9548A @ 0x70 .......... %s\n", ping(MUX_ADDR) ? "PRESENT" : "MISSING");
  Serial.printf("  LEFT  AS5600 on ch6 ...... %s\n", leftOk  ? "PRESENT" : "MISSING");
  Serial.printf("  RIGHT AS5600 on ch3 ...... %s\n", rightOk ? "PRESENT" : "MISSING");
  Serial.println(F("  BTS7960 signal wiring .... not provable from software, see [6]"));
  Serial.println(F("\n  COMMANDS (type + Enter):"));
  Serial.println(F("    a  - live angle stream, both encoders (turn wheels by hand)"));
  Serial.println(F("    r  - re-run the whole test sequence"));
  Serial.println(F("    l  - spin LEFT motor  25% duty, 1 s   [WHEELS OFF THE GROUND]"));
  Serial.println(F("    k  - spin RIGHT motor 25% duty, 1 s   [WHEELS OFF THE GROUND]"));
  Serial.println(F("    s  - stop everything now"));
  Serial.println(F("=========================================\n"));
}

// ---------------------------------------------------------------- motion
static void spin(const char *name, uint8_t rpwm, uint8_t lpwm, uint8_t mask) {
  Serial.printf("\n  spinning %s at 25%% for 1 s -- watch the wheel\n", name);
  int before = -1, after = -1;
  if (muxWrite(mask)) { delay(2); before = as5600Read12(AS_RAW_ANGLE); }

  digitalWrite(PIN_EN, HIGH);
  analogWrite(rpwm, 64);          // ~25 % duty
  digitalWrite(lpwm, LOW);
  delay(1000);
  analogWrite(rpwm, 0);
  digitalWrite(rpwm, LOW);
  digitalWrite(PIN_EN, LOW);
  delay(300);                     // let it coast to a stop

  if (muxWrite(mask)) { delay(2); after = as5600Read12(AS_RAW_ANGLE); }
  Serial.printf("  RAW_ANGLE before %d, after %d", before, after);
  if (before >= 0 && after >= 0) {
    int d = after - before;
    if (abs(d) < 20) Serial.println(F("  -> encoder did NOT move. Motor did not turn, or magnet is not on this shaft."));
    else             Serial.printf("  -> moved %d counts, encoder and motor agree.\n", d);
  } else Serial.println();
  killDrive();
}

// ---------------------------------------------------------------- telemetry
// Machine-readable stream for the magnet-tuning GUI. One line per sample:
//   T,<Lang>,<Lagc>,<Lmag>,<Lstat>,<Rang>,<Ragc>,<Rmag>,<Rstat>
// A field of -1 means that read failed.

struct EncData { int ang, agc, mag, stat; };

static EncData readAllRegs(uint8_t mask) {
  EncData d = { -1, -1, -1, -1 };
  if (!muxWrite(mask)) return d;
  delayMicroseconds(300);
  d.stat = as5600Read8(AS_STATUS);
  if (d.stat < 0) return d;
  d.agc = as5600Read8(AS_AGC);
  d.mag = as5600Read12(AS_MAGNITUDE);
  d.ang = as5600Read12(AS_RAW_ANGLE);
  return d;
}

static void telemetrySample() {
  EncData l = readAllRegs(MASK_LEFT);
  EncData r = readAllRegs(MASK_RIGHT);
  Serial.printf("T,%d,%d,%d,%d,%d,%d,%d,%d\n",
                l.ang, l.agc, l.mag, l.stat,
                r.ang, r.agc, r.mag, r.stat);
}

// ---------------------------------------------------------------- mapping
// Drive one PWM pin at a time and watch BOTH encoders. This is the only way to
// learn which GPIO actually reaches which driver, independent of the labels.

static int16_t wrapDelta(int prev, int now) {
  int d = now - prev;
  if (d >  2048) d -= 4096;
  if (d < -2048) d += 4096;
  return (int16_t)d;
}

static int readEnc(uint8_t mask) {
  if (!muxWrite(mask)) return -1;
  delayMicroseconds(400);
  return as5600Read12(AS_RAW_ANGLE);
}

#define MAP_CH   4          // LEDC channel used only for this test
#define MAP_DUTY 154        // ~60 % of 8-bit -- enough to break stiction

static void mapPin(const char *name, uint8_t pin) {
  int lp = readEnc(MASK_LEFT), rp = readEnc(MASK_RIGHT);
  long lacc = 0, racc = 0;

  ledcSetup(MAP_CH, 15000, 8);
  ledcAttachPin(pin, MAP_CH);
  ledcWrite(MAP_CH, MAP_DUTY);

  uint32_t t0 = millis();
  while (millis() - t0 < 1200) {
    int l = readEnc(MASK_LEFT);
    int r = readEnc(MASK_RIGHT);
    if (l >= 0 && lp >= 0) lacc += wrapDelta(lp, l);
    if (r >= 0 && rp >= 0) racc += wrapDelta(rp, r);
    if (l >= 0) lp = l;
    if (r >= 0) rp = r;
  }

  ledcWrite(MAP_CH, 0);
  ledcDetachPin(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(500);                               // coast, then catch the last motion
  int l = readEnc(MASK_LEFT), r = readEnc(MASK_RIGHT);
  if (l >= 0 && lp >= 0) lacc += wrapDelta(lp, l);
  if (r >= 0 && rp >= 0) racc += wrapDelta(rp, r);

  const char *verdict;
  bool lm = labs(lacc) > 40, rm = labs(racc) > 40;
  if (lm && rm)      verdict = "BOTH encoders moved -- shared wire or chassis vibration";
  else if (lm)       verdict = "-> drives the LEFT wheel";
  else if (rm)       verdict = "-> drives the RIGHT wheel";
  else               verdict = "-> nothing moved";
  Serial.printf("      %-14s  left %+6ld  right %+6ld   %s\n", name, lacc, racc, verdict);
}

static void mappingTest() {
  Serial.println(F("\n[M] PWM MAPPING  (one pin at a time, both encoders watched)"));
  Serial.println(F("      60% duty, 1.2 s per pin. WHEELS OFF THE GROUND."));
  mapPin("LEFT  RPWM", PIN_L_RPWM);
  mapPin("LEFT  LPWM", PIN_L_LPWM);
  mapPin("RIGHT RPWM", PIN_R_RPWM);
  mapPin("RIGHT LPWM", PIN_R_LPWM);
  killDrive();
  Serial.println(F("      PASS = each label moves its own wheel, RPWM +ve and LPWM -ve."));
}

static void runAll() {
  testUpstream();
  testMuxRegister();
  testResetPin();
  scanAllChannels();
  testEncoders();
  testDrivePins();
  summary();
}

void setup() {
  killDrive();                    // first, before anything else
  pinMode(PIN_MUX_RESET, OUTPUT);
  digitalWrite(PIN_MUX_RESET, HIGH);

  Serial.begin(115200);
  delay(400);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  Serial.println(F("\n\n########## DRIVE HARNESS DIAGNOSTIC ##########"));
  Serial.println(F("Motor rail is live and EN is strapped to VCC -- the drivers are ALWAYS"));
  Serial.println(F("enabled. Only RPWM/LPWM being low keeps the motors still."));
  runAll();
}

static bool streaming = false;
static bool telem = false;

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') { telem = !telem; }
    else if (c == 'a') { streaming = !streaming; Serial.println(streaming ? F("\nstreaming on") : F("\nstreaming off")); }
    else if (c == 'r') { streaming = false; runAll(); }
    else if (c == 'l') { streaming = false; spin("LEFT",  PIN_L_RPWM, PIN_L_LPWM, MASK_LEFT); }
    else if (c == 'k') { streaming = false; spin("RIGHT", PIN_R_RPWM, PIN_R_LPWM, MASK_RIGHT); }
    else if (c == 's') { streaming = false; killDrive(); Serial.println(F("\nstopped")); }
    else if (c == 'm') { streaming = false; mappingTest(); }
    else if (c == '1') { streaming = false; Serial.println(F("\nGPIO25 L_RPWM:")); mapPin("GPIO25 L_RPWM", PIN_L_RPWM); }
    else if (c == '2') { streaming = false; Serial.println(F("\nGPIO26 L_LPWM:")); mapPin("GPIO26 L_LPWM", PIN_L_LPWM); }
    else if (c == '3') { streaming = false; Serial.println(F("\nGPIO32 R_RPWM:")); mapPin("GPIO32 R_RPWM", PIN_R_RPWM); }
    else if (c == '4') { streaming = false; Serial.println(F("\nGPIO33 R_LPWM:")); mapPin("GPIO33 R_LPWM", PIN_R_LPWM); }
  }

  if (telem) {
    telemetrySample();
    delay(40);                    // ~20 Hz, fast enough to feel live in the hand
    return;
  }

  if (streaming) {
    int l = -1, r = -1;
    if (muxWrite(MASK_LEFT))  { delay(1); l = as5600Read12(AS_RAW_ANGLE); }
    if (muxWrite(MASK_RIGHT)) { delay(1); r = as5600Read12(AS_RAW_ANGLE); }
    Serial.printf("  LEFT %5d (%6.1f deg)   RIGHT %5d (%6.1f deg)\n",
                  l, l * 360.0 / 4096.0, r, r * 360.0 / 4096.0);
    delay(150);
  }
}
