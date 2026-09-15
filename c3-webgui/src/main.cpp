// c3-webgui -- closed-loop differential drive for the ESP32-C3 Supermini.
//
// Two BTS7960 half-bridges, two AS5600 magnetic encoders on the MOTOR shafts
// behind a PCA9548A I2C mux. Everything above the raw driver runs in a
// forward-positive frame: +1 command and +1 count both mean "this wheel is
// rolling the bot forwards", on both sides, regardless of the mirrored
// mechanical mounting.
//
// Pinout follows the master schematic:
//
//   LEFT  BTS7960   RPWM -> GPIO 1    LPWM -> GPIO 0
//   RIGHT BTS7960   RPWM -> GPIO 4    LPWM -> GPIO 3
//   I2C             SDA  -> GPIO 6    SCL  -> GPIO 7
//
//   LEFT  AS5600 = mux channel 6 (mask 0x40)
//   RIGHT AS5600 = mux channel 3 (mask 0x08)
//
// R_EN/L_EN strapped to VCC (always enabled) -- zero duty is what keeps the
// motors still.
//
// ---------------------------------------------------------------------------
// ARCHITECTURE (this is the part worth reading)
// ---------------------------------------------------------------------------
//
// The old build was open loop: the joystick's y value became a PWM duty and
// that was the end of it. Encoder sampling shared loop() with WiFi, the async
// web server and 30 lines/s of USB serial, so the sampling interval was
// whatever was left over -- fine for reporting distance, useless as the input
// to a control law.
//
// Now there is a dedicated FreeRTOS control task (CTRL_HZ, below) that owns
// the I2C bus outright and is the only thing that ever talks to it. It runs
// off vTaskDelayUntil, so its period is fixed regardless of what WiFi and the
// web server are doing. loop() has been demoted to command parsing and
// printing; it never touches I2C and never touches PWM while a closed loop is
// running.
//
// The controller is a two-level cascade, per wheel:
//
//   commanded velocity -> integrated into a POSITION REFERENCE (refCounts)
//   position error (refCounts - actual) -> trims the velocity setpoint
//   velocity error -> PI, on top of a measured feedforward, -> PWM duty
//
// Driving straight is a consequence of the structure rather than a special
// case. Both wheels track their own position reference; command both the same
// velocity and both references advance identically, so any difference between
// the wheels is a position error that the loop actively removes. A velocity-
// only loop cannot do this: it nulls speed error but lets the *distance*
// error integrate up forever, which is exactly how a robot drives a slow arc
// while both wheels report the correct speed.
//
// The remaining failure mode is one wheel physically unable to keep up (a
// carpet edge, a stalled gearbox). Left alone the position loop would wind
// the reference away from a wheel that cannot follow it. So the reference
// advance is governed: once either wheel falls POS_ERR_SOFT behind, BOTH
// references slow down together, and past POS_ERR_HARD they stop entirely.
// The pair stays locked to each other even when the base as a whole is being
// held back, and resumes in step once the obstruction clears.
//
// Position hold is the same code path with a zero velocity command: the
// reference stops moving and the loop drives whatever error appears back to
// zero, so a shoved wheel is pushed back where it was.
//
// ---------------------------------------------------------------------------
// CALIBRATION -- one constant for both wheels, deliberately
// ---------------------------------------------------------------------------
//
// calibration.json lists per-wheel gear ratios (LEFT 18.524, RIGHT 18.447).
// Those are NOT a real difference between the two sides: both wheels run the
// same gearbox, so the ratio is one number by construction. The 0.4% spread
// is measurement error (LEFT had 16 hand-turned readings, RIGHT only 5;
// weighting by reading count gives 18.506, within 0.07% of the combined
// 18.518). Programming the per-wheel numbers in would bake that measurement
// artifact into the controller as a permanent 0.4% curve to one side. So the
// combined constant is used for BOTH wheels and there is no per-wheel scale
// factor anywhere in this file. If the car still curves, the cause is
// mechanical or a wheel diameter error, not these numbers.

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <esp_bt.h>
#include <esp_wifi.h>
#include <math.h>

// ======================================================= Ethernet transport
//
// Why this exists: the USB CDC link re-enumerates whenever the motors draw
// current (SETUP.md; it is reproducible and it is a hardware fact, not a
// driver bug). That drops the console mid-run, which is exactly when you
// want it. The W5500 on SPI is a second path to the same console that does
// not care what the motors are doing, and it reaches the laptop over the
// Bullet M2 instead of a tether.
//
// The design goal was to add this WITHOUT touching the ~1200 lines of
// existing Serial.printf() call sites. That is what LinkStream does: it is a
// Stream that fans every write out to BOTH the USB CDC and the TCP client,
// and reads commands from whichever side is talking. The single
// `#define Serial Link` at the end of this block re-points every existing
// call site at it. USB keeps working exactly as before -- it is the recovery
// path if a bad OTA ever lands -- and Ethernet is simply a second head.
//
// Two hazards handled here that are easy to get wrong:
//
//  1. HWCDC::write() BLOCKS for up to 100 ms per call when USB is unplugged.
//     See core 2.0.17 HWCDC.cpp:440, "waits 1ms per sending data attempt, in
//     case CDC is unplugged". This firmware emits ~110 telemetry lines/s, so
//     with the USB cable out that alone would stall loop() into uselessness.
//     Every USB write below is therefore gated on `Serial` being truthy --
//     HWCDC::operator bool() is the real connected state.
//
//  2. Un-batched, each Serial.printf() becomes its own TCP segment: ~110
//     packets/s of ~40 bytes payload each, every one paying a 40-byte TCP/IP
//     header plus a full 802.11 exchange. Over a 2.4 GHz link airtime per
//     PACKET, not per byte, is the scarce resource. So writes accumulate in
//     one MTU-sized buffer that is flushed on a 20 ms timer -- telemetry that
//     used to be ~110 ragged packets/s becomes <=50 full ones, and usually
//     far fewer. 20 ms of buffering is invisible to a human console and is
//     nowhere near any control deadline: control lives in controlTask() and
//     never comes through here.
//
// Thread safety: none needed, and that is load-bearing. controlTask() does
// not print -- every Serial.* call in this firmware is made from loop(). If
// you ever print from the control task, this buffer needs a mutex first.

#include <Ethernet.h>
#include <Update.h>
#include <esp_system.h>
#include <esp_ota_ops.h>

static const uint16_t NET_CMD_PORT  = 3333;   // console + telemetry, line based
static const uint16_t NET_OTA_PORT  = 3232;   // firmware push
static const uint16_t NET_FLUSH_MS  = 20;     // telemetry batching window
static const uint16_t NET_TX_HIGH   = 1200;   // flush early once this full

static EthernetClient netCmd;                 // the one live console client
static bool     netUp      = false;           // link + IP are good
static uint8_t  netTx[1460];                  // one Ethernet MTU
static uint16_t netTxLen   = 0;
static uint32_t netLastFlush = 0;
static uint32_t netTxBytes = 0, netTxPackets = 0;

static void netFlush() {
  netLastFlush = millis();
  if (!netTxLen) return;
  if (netUp && netCmd && netCmd.connected()) {
    netCmd.write(netTx, netTxLen);
    netTxBytes += netTxLen;
    netTxPackets++;
  }
  netTxLen = 0;
}

static void netPush(const uint8_t *b, size_t n) {
  if (!netUp || !netCmd || !netCmd.connected()) return;
  while (n) {
    size_t room = sizeof(netTx) - netTxLen;
    if (room == 0) { netFlush(); room = sizeof(netTx); }
    size_t take = (n < room) ? n : room;
    memcpy(netTx + netTxLen, b, take);
    netTxLen += take;
    b += take;
    n -= take;
  }
  if (netTxLen >= NET_TX_HIGH) netFlush();
}

// The tee. Reads prefer USB so a cable always wins a tie; that makes USB the
// unambiguous recovery console when the network is the thing that is broken.
class LinkStream : public Stream {
public:
  void begin(unsigned long baud) { Serial.begin(baud); }

  size_t write(uint8_t c) override { return write(&c, 1); }

  size_t write(const uint8_t *buf, size_t n) override {
    if (Serial) Serial.write(buf, n);   // gated: unplugged USB blocks 100 ms
    netPush(buf, n);
    return n;
  }

  int available() override {
    int n = Serial.available();
    if (n > 0) return n;
    if (netUp && netCmd && netCmd.connected()) return netCmd.available();
    return 0;
  }

  int read() override {
    if (Serial.available()) return Serial.read();
    if (netUp && netCmd && netCmd.connected()) return netCmd.read();
    return -1;
  }

  int peek() override {
    if (Serial.available()) return Serial.peek();
    if (netUp && netCmd && netCmd.connected()) return netCmd.peek();
    return -1;
  }

  void flush() override {
    netFlush();
    if (Serial) Serial.flush();
  }
};

static LinkStream Link;

// Everything below this line that says Serial means "USB and Ethernet".
#define Serial Link

static void ethBegin();
static void ethPoll();
static void ethStatus();

// ---- radios -------------------------------------------------------------
//
// WiFi and Bluetooth are OFF by default. On a single-core C3 the WiFi task
// runs at priority 18-23, well above the 500 Hz control task at 5, so it
// preempts the control loop whenever the radio needs servicing -- that was
// the entire source of the loop's tick overruns. Off, the control task owns
// the core outright.
//
// Bluetooth is never initialised by this firmware (no BLE library is linked),
// so the controller is already idle; the explicit disable below makes that a
// stated fact rather than an assumption and releases its memory.
//
// The phone joystick needs the radio, so it is not gone -- bring it up at
// runtime with "A 1" over USB serial and drop it with "A 0". No reflash.
static const bool RADIO_ON_AT_BOOT = false;

// WIFI_SSID / WIFI_PASS live in secrets.h, which is gitignored.
// Copy secrets.example.h to secrets.h and fill it in.
#include "secrets.h"

static bool radioUp     = false;
static bool serverBegun = false;

// ---- I2C / encoders ----
static const uint8_t PIN_SDA = 6;
static const uint8_t PIN_SCL = 7;

// The PCA9548A is only rated to 400 kHz, and it is the slowest part on the
// bus, so that is the ceiling regardless of what the AS5600 can do (1 MHz).
static const uint32_t I2C_HZ = 400000;

static const uint8_t MUX_ADDR     = 0x70;   // PCA9548A / HW-617
static const uint8_t AS5600_ADDR  = 0x36;

// Register map. The useful pairs are contiguous, which matters: STATUS,
// ANGLE_HI and ANGLE_LO are 0x0B/0x0C/0x0D, so one 3-byte burst read from
// 0x0B gets the magnet status for free alongside every angle sample. Same
// trick for AGC/MAGNITUDE at 0x1A/0x1B/0x1C.
static const uint8_t AS5600_STATUS    = 0x0B;  // MD 0x20, ML 0x10, MH 0x08
static const uint8_t AS5600_AGC       = 0x1A;  // 8-bit, 0-128 on a 3.3 V rail

// ---- calibration ---------------------------------------------------------
//
// TWO free parameters, and every other distance number in this file is
// DERIVED from them. Nothing is hardcoded twice, so there is no way for the
// counts-per-metre and the counts-per-degree to disagree with each other.
//
//   gearRatio       motor revolutions per WHEEL revolution
//   wheelDiameterM  tyre outside diameter, metres
//
// The same values are used for BOTH wheels, deliberately -- see the header.
//
// On the ratio itself: calibration.json's 18.518 was measured by hand-turning
// the wheel through a known ANGLE and reading the counter, so it does not
// depend on the wheel diameter at all -- it is just counts_per_wheel_rev/4096.
// LEFT (16 readings) gave 18.524 and RIGHT (5 readings) gave 18.447, agreeing
// to 0.42%. A catalogue 18.75:1 gearbox sits 1.25% above that, which is well
// inside the error of eyeballing an angle by hand, so 18.75 is the most
// likely true value. A 20:1 gearbox would be 8% away, which two independent
// wheels cannot both be wrong by in the same direction -- 20:1 is not what
// is on this car.
//
// Settle it with the N command and a ten-revolution test (SOFTWARE.md):
//   G L 3600   -- if the tyre mark comes back exactly, the ratio is right.
//   1.25% wrong shows up as 45 deg of leftover; 8% wrong as 288 deg.
static float gearRatio      = 18.518f;
static float wheelDiameterM = 0.120f;

// Derived. recomputeCal() is the only thing that ever writes these.
static float COUNTS_PER_METRE     = 201196.0f;
static float COUNTS_PER_WHEEL_DEG = 210.69f;
static float COUNTS_PER_WHEEL_REV = 75849.1f;
static float FF_SLOPE             = 209.8f;   // extra duty per m/s

// Sign conventions, resolved into the forward-positive frame.
//
// calibration.json: encoder_sign = -1 (RPWM makes the counters fall), and
// per-wheel direction LEFT = -1, RIGHT = +1 (the motors are mirror-mounted,
// so wiring them identically runs one backwards).
//
//   MOT_DIR[i] : multiply a forward-positive duty by this to get board duty
//   ENC_DIR[i] : multiply raw counts by this to get forward-positive counts
//
// Both are verified by the S spin test, which prints what it expected and
// what it saw -- see the SPIN output and SETUP.md 7.
static const int MOT_DIR[2] = {-1, +1};
// ENC_DIR was {+1, -1} until 2026-08-31, when the two encoder connections were
// physically swapped to fix a crossed motor/encoder pairing. That swap moved
// each encoder onto the other side's mux channel, so the right slot's sign had
// to follow it. Verified with the S spin test: both sides now report `ok`.
static const int ENC_DIR[2] = {+1, +1};

// ---- status LED ----------------------------------------------------------
//
// A blue LED that says what the base is doing without a terminal attached.
//
// PIN CHOICE. GPIO 20 and 21 are UART0 RX and TX. Serial on this board is the
// native USB CDC, so both are otherwise free -- but the HWT905 IMU needs
// GPIO 20 to RECEIVE its stream, and it never needs the ESP32 to talk back
// (it is configured from the WitMotion app). So GPIO 21 is the one that is
// genuinely spare, and that is where the LED belongs. If the LED is wired to
// 20 instead, move it with `L 21` at runtime to check, then move the wire --
// leaving it on 20 costs you the IMU.
//
// The ROM bootloader also prints its boot log on GPIO 21 at reset, so a short
// flicker on power-up is expected and is itself a useful "the chip rebooted"
// tell.
static uint8_t LED_PIN      = 21;
static bool    LED_ACTIVE_LOW = false;   // flip with `L I` for common-anode

// Patterns are a 1 s cycle sampled in 20 slots of 50 ms; bit i is "lit during
// slot i". A bitmask beats a step list here because adding a pattern is one
// readable constant rather than a table entry, and the update is a shift.
static const uint32_t LED_OFF_MASK   = 0x00000;
static const uint32_t LED_SOLID      = 0xFFFFF;  // driving: closed loop, moving
static const uint32_t LED_HEARTBEAT  = 0x00001;  // idle and healthy: one blip
static const uint32_t LED_HOLD       = 0x00005;  // holding position: two blips
static const uint32_t LED_ROTATE     = 0x0000F;  // a G move is running
static const uint32_t LED_FAULT_ENC  = 0x33333;  // encoder/magnet bad: 5 Hz
static const uint32_t LED_FAULT_STALL= 0x00155;  // loop flooring a dead wheel
static const uint32_t LED_IDENTIFY   = 0x55555;  // `L T`: unmistakable 10 Hz

static uint32_t ledMask      = LED_HEARTBEAT;
static uint32_t ledIdentifyUntil = 0;
static const char *ledWhat   = "boot";

// A wheel the loop is flooring while the error refuses to close is the exact
// signature of a motor that is not connected to the encoder being corrected.
// Time it rather than trusting one sample, so a legitimate hard push does not
// trip it.
static const int      STALL_DUTY    = 200;    // duty this high ...
static const float    STALL_ERR_M   = 0.020f; // ... with error this big ...
static const uint32_t STALL_MS      = 700;    // ... for this long
static uint32_t stallSince[2] = {0, 0};
static bool     stallLatched  = false;

static void ledInit() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ACTIVE_LOW ? HIGH : LOW);
}

static void ledSet(bool on) {
  digitalWrite(LED_PIN, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

// Three deliberate flashes at boot: "the firmware started", distinct from the
// bootloader's own flicker because it is slow enough to count.
static void ledBootFlash() {
  for (int i = 0; i < 3; i++) {
    ledSet(true);  delay(90);
    ledSet(false); delay(110);
  }
}

// ---- control loop ----
// 500 Hz. The bus cost per tick is 2 mux writes + 2 three-byte burst reads,
// about 0.4 ms at 400 kHz, so the task uses well under a quarter of its
// period and leaves the rest of the core to WiFi. A geared DC motor's
// mechanical time constant is tens of milliseconds, so 500 Hz is far past
// the point of diminishing returns -- the win here is that the period is
// CONSTANT, which is what the derivative-free PI actually needs.
static const uint32_t CTRL_HZ = 500;
static const float    CTRL_DT = 1.0f / (float)CTRL_HZ;

// Velocity low-pass. alpha = 0.25 at 500 Hz is a ~23 Hz corner: fast enough
// to be invisible to the mechanics, slow enough to kill encoder quantisation.
static const float VEL_ALPHA = 0.25f;

// Feedforward, straight out of calibration.json's duty_to_counts_per_sec fit
// (counts_per_sec = 959*duty - 47316). Inverted and converted to m/s:
//   duty = 49.3 + v_mps * 201196/959
// The 49.3 is the extrapolated deadband, and it matches the measured "no
// motion at or below duty 45, creeps at 50" independently.
static const float FF_DEAD    = 49.3f;   // duty needed before anything moves
static const float FF_CPS_PER_DUTY = 959.0f;  // calibration.json slope
static const float VEL_MAX    = 0.90f;   // m/s; duty 255 works out to ~0.98

// Everything distance-related, rebuilt from the two free parameters. Called
// at boot and by the N command, so changing the ratio at runtime updates the
// odometry, the rotate primitive and the feedforward all at once.
static void recomputeCal() {
  COUNTS_PER_WHEEL_REV = 4096.0f * gearRatio;
  COUNTS_PER_WHEEL_DEG = COUNTS_PER_WHEEL_REV / 360.0f;
  COUNTS_PER_METRE     = COUNTS_PER_WHEEL_REV / (float)(M_PI * wheelDiameterM);
  // The duty-to-speed fit was measured in counts/s, so the m/s slope has to
  // move with counts-per-metre or the feedforward silently goes wrong too.
  FF_SLOPE             = COUNTS_PER_METRE / FF_CPS_PER_DUTY;
}

// Cascade gains. Runtime-settable with the K command.
static float KP_VEL = 250.0f;   // duty per (m/s) of velocity error
static float KI_VEL = 900.0f;   // duty per (m/s) per second
static float KP_POS = 8.0f;     // (m/s) of velocity trim per metre behind

// Reference governor. Past SOFT both references slow down together; past HARD
// they stop. This is what keeps the two wheels locked when one is obstructed.
static float POS_ERR_SOFT = 0.020f;   // m
static float POS_ERR_HARD = 0.060f;   // m

// Track width: needed for yaw and for the W unicycle command, and NOT
// measured on this car (calibration.json has it null). 0.20 m is a stated
// placeholder, not a measurement. Set it with B <metres> once measured; the
// FB telemetry line reports the value in use so nothing downstream has to
// guess. Straight-line distance does not depend on it.
static float TRACK_WIDTH_M = 0.200f;
static bool  TRACK_WIDTH_MEASURED = false;

struct Encoder {
  const char *name;
  uint8_t muxMask;
  bool     seeded;      // have we taken a first reading to unwrap against?
  uint16_t lastRaw;
  int64_t  counts;      // unwrapped, unbounded, RAW board frame
  bool     online;      // last read succeeded

  // Magnet health. status comes free with every angle burst; agc/magnitude
  // are a second read and are polled slowly.
  int16_t  agc;
  int16_t  magnitude;
  int16_t  status;
};

static Encoder encoders[] = {
  {"LEFT",  0x40, false, 0, 0, false, -1, -1, -1},
  {"RIGHT", 0x08, false, 0, 0, false, -1, -1, -1},
};

// ---- motors ----
struct Motor {
  const char *name;
  uint8_t rpwmPin, lpwmPin;
  uint8_t rpwmCh, lpwmCh;
};

static Motor motors[] = {
  {"LEFT",  1, 0, 0, 1},
  {"RIGHT", 4, 3, 2, 3},
};

static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;   // 0-255

// how long without a fresh command before we force-stop (ms)
static const uint32_t CMD_TIMEOUT_MS = 300;

// Position hold is a deliberate "stay put", so it is exempt from the command
// watchdog -- but not forever, or a forgotten hold cooks a stalled motor.
static const uint32_t HOLD_MAX_MS = 120000;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

static volatile uint32_t lastCmdMillis = 0;
static volatile bool driving = false;

// ---------------------------------------------------------------- shared state
//
// The control task and loop() both touch this. loop() only ever holds the
// mutex for a few microseconds (copy a command in, copy a snapshot out), so
// the control task never meaningfully blocks on it.
static SemaphoreHandle_t ctrlMutex = nullptr;
#define LOCK()   xSemaphoreTake(ctrlMutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(ctrlMutex)

enum CtrlMode { CTRL_OFF, CTRL_VEL, CTRL_HOLD };
static volatile CtrlMode ctrlMode = CTRL_OFF;

// Everything below is in the FORWARD-POSITIVE frame and in SI units.
static float   cmdVel[2]   = {0, 0};    // m/s, commanded
static double  refCounts[2] = {0, 0};   // position reference, counts
static float   fwdVel[2]   = {0, 0};    // m/s, filtered measurement
static float   posErrM[2]  = {0, 0};    // m, reference minus actual
static float   velInteg[2] = {0, 0};    // PI integrator state
static int     lastDuty[2] = {0, 0};    // what the loop actually applied
static float   refScale    = 1.0f;      // governor output, 0..1

// Pose, integrated from the wheel counts.
static double poseX = 0, poseY = 0, poseTh = 0;

// Loop health, reported by Q.
// Set when something makes the counters jump discontinuously (a Z zero).
// The control task's differencing state has to be re-primed on the next tick
// or the jump is integrated as real travel: it would inject a metre-sized
// step into the pose and a single enormous spike into the velocity filter.
static volatile bool ctrlResync = true;

// I2C bus scan. Only the control task may touch Wire, so loop() raises a
// request, the task performs the scan in its own context and fills this
// buffer, and loop() prints it. Refused while anything is driving: a full
// sweep is ~1000 probes and stalls encoder sampling for tens of ms.
static volatile bool scanRequest = false;
static volatile bool scanDone    = false;
static char scanBuf[900];

// Split the tick budget into bus time and compute time. Bus time is fixed by
// the I2C hardware and is the same in any language; compute time is the part
// a slower runtime would multiply. Reported by Q.
static volatile uint32_t ctrlI2cUs  = 0;   // rolling max
static volatile uint32_t ctrlMathUs = 0;   // rolling max
static volatile uint64_t ctrlMathSum = 0;  // for a mean
static volatile uint32_t ctrlMathN  = 0;

static volatile uint32_t ctrlTicks = 0;
static volatile uint32_t ctrlOverruns = 0;
static volatile uint32_t ctrlMaxUs = 0;
static volatile uint32_t i2cErrors = 0;

static int64_t fwdCountsOf(int i) { return encoders[i].counts * ENC_DIR[i]; }

// ---------------------------------------------------------------- I2C
//
// Every function in this section runs ONLY on the control task (or in setup(),
// before that task is started). Nothing else may touch Wire.

static uint8_t muxCurrent = 0xFF;   // cached, so we skip redundant writes

static bool muxSelect(uint8_t mask) {
  if (mask == muxCurrent) return true;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(mask);
  if (Wire.endTransmission() != 0) { muxCurrent = 0xFF; i2cErrors++; return false; }
  muxCurrent = mask;
  return true;
}

// One burst read of `n` bytes starting at `reg` from the AS5600 on the
// currently-open channel.
static bool as5600Burst(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { i2cErrors++; return false; }
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, n) != n) { i2cErrors++; return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Hot path: select the channel and pull STATUS+ANGLE in one 3-byte burst,
// then fold the new angle into the unwrapped count.
static void sampleEncoder(Encoder &e) {
  if (!muxSelect(e.muxMask)) { e.online = false; return; }

  uint8_t b[3];
  if (!as5600Burst(AS5600_STATUS, b, 3)) { e.online = false; return; }

  e.status = (int16_t)b[0];
  uint16_t raw = (((uint16_t)b[1] << 8) | b[2]) & 0x0FFF;

  if (!e.seeded) {
    e.lastRaw = raw;
    e.seeded  = true;
    e.online  = true;
    return;
  }

  // Shortest-path delta across the 4096-count wrap point. At 500 Hz this
  // stays correct up to ~250 motor rev/s, far above anything these motors do.
  int32_t d = (int32_t)raw - (int32_t)e.lastRaw;
  if (d >  2048) d -= 4096;
  if (d < -2048) d += 4096;

  e.counts += d;
  e.lastRaw = raw;
  e.online  = true;
}

static void sampleAllEncoders() {
  for (auto &e : encoders) sampleEncoder(e);
}

// AGC and MAGNITUDE, for encoder_tune.py's airgap dashboard. These are a
// bench-tuning readout, never a control input, so they are polled slowly and
// skipped entirely while the wheels are moving -- keeping them out of the
// control path is most of why the loop is now deterministic.
static void sampleAllMagnets() {
  for (auto &e : encoders) {
    if (!muxSelect(e.muxMask)) { e.agc = e.magnitude = -1; continue; }
    uint8_t b[3];
    if (!as5600Burst(AS5600_AGC, b, 3)) { e.agc = e.magnitude = -1; continue; }
    e.agc       = (int16_t)b[0];
    e.magnitude = (int16_t)((((uint16_t)b[1] << 8) | b[2]) & 0x0FFF);
  }
}

// ---------------------------------------------------------------- magnetometer
//
// QMC5883L on the MAIN bus at 0x0D, ahead of the mux, so it is readable
// whatever channel is selected. Note this is NOT the HMC5883L the silkscreen
// claims: boards sold as "GY-271 HMC5883L" are almost always a QMC5883L, a
// different die with a different register map at a different address (0x0D,
// not 0x1E). HMC5883L code will not work on it.
static const uint8_t QMC_ADDR   = 0x0D;
static const uint8_t QMC_DATA   = 0x00;   // X_LSB..Z_MSB, little-endian int16
static const uint8_t QMC_CTRL1  = 0x09;
static const uint8_t QMC_SETRST = 0x0B;

// Continuous, 200 Hz, +-8 gauss, 512x oversampling.
static const uint8_t QMC_CFG    = 0x1D;
static const float   QMC_LSB_PER_GAUSS = 3000.0f;   // at the 8 gauss range

static bool  magPresent = false;
static int16_t magRaw[3] = {0, 0, 0};
static float magHeadingDeg = 0.0f;
static uint32_t magReads = 0, magErrors = 0;

static bool magWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool magInit() {
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission() != 0) return false;
  // The SET/RESET period register must be 0x01 or the part returns garbage.
  if (!magWrite(QMC_SETRST, 0x01)) return false;
  if (!magWrite(QMC_CTRL1, QMC_CFG)) return false;
  return true;
}

static void magSample() {
  if (!magPresent) return;
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(QMC_DATA);
  if (Wire.endTransmission(false) != 0) { magErrors++; return; }
  if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6) != 6) { magErrors++; return; }
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++)
    magRaw[i] = (int16_t)((uint16_t)b[2*i] | ((uint16_t)b[2*i+1] << 8));
  // Flat-mounted heading. No tilt compensation -- that needs an
  // accelerometer, which this part does not have.
  magHeadingDeg = atan2f((float)magRaw[1], (float)magRaw[0]) * 57.29578f;
  if (magHeadingDeg < 0) magHeadingDeg += 360.0f;
  magReads++;
}

// Probe one 7-bit address on whatever channel is currently open.
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Name the parts we expect to meet, so the output is readable without a
// datasheet. The 5883 pair is the one worth calling out: boards sold as
// "GY-271 HMC5883L" are usually a QMC5883L, which is a DIFFERENT chip with a
// different register map, and it answers at 0x0D instead of 0x1E.
static const char *i2cName(uint8_t a) {
  switch (a) {
  case 0x0D: return "QMC5883L magnetometer (the usual GY-271 chip)";
  case 0x1E: return "HMC5883L magnetometer (genuine Honeywell)";
  case 0x36: return "AS5600 encoder";
  case 0x68: return "MPU6050/MPU9250/DS3231";
  case 0x69: return "MPU6050 (AD0 high)";
  case 0x28: case 0x29: return "BNO055 IMU";
  case 0x70: return "PCA9548A mux";
  case 0x76: case 0x77: return "BMP280/BME280";
  default:   return "";
  }
}

static void i2cScan() {
  size_t n = 0;
  int total = 0;
  n += snprintf(scanBuf + n, sizeof(scanBuf) - n,
                "# ---- I2C scan ----\n");

  // Main bus, with every mux channel closed, so only devices wired ahead of
  // the mux can answer.
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  muxCurrent = 0x00;

  n += snprintf(scanBuf + n, sizeof(scanBuf) - n, "# main bus (mux closed):");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78 && n < sizeof(scanBuf) - 90; a++) {
    if (!i2cPresent(a)) continue;
    found++; total++;
    const char *nm = i2cName(a);
    n += snprintf(scanBuf + n, sizeof(scanBuf) - n, "\n#   0x%02X  %s", a, nm);
  }
  if (!found) n += snprintf(scanBuf + n, sizeof(scanBuf) - n, " nothing");

  // Then each mux channel in turn.
  for (uint8_t ch = 0; ch < 8 && n < sizeof(scanBuf) - 120; ch++) {
    uint8_t mask = (uint8_t)(1 << ch);
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(mask);
    if (Wire.endTransmission() != 0) continue;
    muxCurrent = mask;
    found = 0;
    for (uint8_t a = 0x08; a < 0x78 && n < sizeof(scanBuf) - 90; a++) {
      if (a == MUX_ADDR || !i2cPresent(a)) continue;
      if (!found) n += snprintf(scanBuf + n, sizeof(scanBuf) - n,
                                "\n# mux ch%d (mask 0x%02X):", ch, mask);
      found++; total++;
      n += snprintf(scanBuf + n, sizeof(scanBuf) - n,
                    "\n#   0x%02X  %s", a, i2cName(a));
    }
  }

  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
  muxCurrent = 0xFF;

  snprintf(scanBuf + n, sizeof(scanBuf) - n,
           "\n# ---- %d device%s total ----", total, total == 1 ? "" : "s");
  // The bus was hammered and the encoders went unread; do not integrate the
  // gap as motion.
  ctrlResync = true;
  scanDone = true;
}

static float metresOf(const Encoder &e) {
  return (float)e.counts / COUNTS_PER_METRE;
}

// ---------------------------------------------------------------- motors

// Raw board-frame drive, -1..1. Used by the legacy open-loop paths.
void driveMotor(Motor &m, float v) {
  v = constrain(v, -1.0f, 1.0f);
  int duty = (int)(fabs(v) * 255.0f);
  if (v > 0.02f) {
    ledcWrite(m.lpwmCh, 0);
    ledcWrite(m.rpwmCh, duty);
  } else if (v < -0.02f) {
    ledcWrite(m.rpwmCh, 0);
    ledcWrite(m.lpwmCh, duty);
  } else {
    ledcWrite(m.rpwmCh, 0);
    ledcWrite(m.lpwmCh, 0);
  }
}

// Forward-positive drive: +duty always rolls the bot forward, either side.
static void driveMotorFwd(int i, int duty) {
  duty = constrain(duty, -255, 255);
  lastDuty[i] = duty;
  int board = duty * MOT_DIR[i];
  Motor &m = motors[i];
  if (board > 0) { ledcWrite(m.lpwmCh, 0); ledcWrite(m.rpwmCh, board); }
  else if (board < 0) { ledcWrite(m.rpwmCh, 0); ledcWrite(m.lpwmCh, -board); }
  else { ledcWrite(m.rpwmCh, 0); ledcWrite(m.lpwmCh, 0); }
}

void stopAll() {
  for (auto &m : motors) {
    ledcWrite(m.rpwmCh, 0);
    ledcWrite(m.lpwmCh, 0);
  }
  lastDuty[0] = lastDuty[1] = 0;
  driving = false;
}

// Both PWM inputs low leaves the motor coasting. On the BTS7960 the two
// half-bridges are independent, so driving BOTH inputs high is a defined
// state, not shoot-through: it ties both motor terminals to Vcc, shorting the
// winding through the high sides and braking hard.
static void motorBrake(Motor &m) {
  ledcWrite(m.rpwmCh, 255);
  ledcWrite(m.lpwmCh, 255);
}

static void motorCoast(Motor &m) {
  ledcWrite(m.rpwmCh, 0);
  ledcWrite(m.lpwmCh, 0);
}

// Legacy raw drive used by the G rotate: push one motor in whichever
// direction makes the RAW counter rise or fall.
static void motorDrive(Motor &m, bool countUp, int duty) {
  int i = (&m == &motors[0]) ? 0 : 1;
  // raw counts rise when the forward-positive count rises iff ENC_DIR is +1
  bool fwd = countUp ? (ENC_DIR[i] > 0) : (ENC_DIR[i] < 0);
  int board = (fwd ? 1 : -1) * MOT_DIR[i];
  if (board > 0) { ledcWrite(m.lpwmCh, 0); ledcWrite(m.rpwmCh, constrain(duty,0,255)); }
  else { ledcWrite(m.rpwmCh, 0); ledcWrite(m.lpwmCh, constrain(duty,0,255)); }
}

// ------------------------------------------------------------ closed loop API
//
// All of these are called from loop() and just hand a setpoint to the task.

static void ctrlEngage(float vl, float vr, CtrlMode mode) {
  LOCK();
  bool wasOff = (ctrlMode == CTRL_OFF);
  if (wasOff) {
    // Start the reference at the wheels' present position, so engaging the
    // loop never produces a step. Clear the integrators for the same reason.
    for (int i = 0; i < 2; i++) {
      refCounts[i] = (double)fwdCountsOf(i);
      velInteg[i]  = 0.0f;
      posErrM[i]   = 0.0f;
    }
  }
  cmdVel[0] = constrain(vl, -VEL_MAX, VEL_MAX);
  cmdVel[1] = constrain(vr, -VEL_MAX, VEL_MAX);
  ctrlMode  = mode;
  UNLOCK();
  // Only a VELOCITY command arms the 300 ms watchdog. Position hold is a
  // deliberate "stay here" with no controller obliged to keep talking, so
  // marking it as `driving` would let the watchdog cancel the hold 300 ms
  // after it was requested -- which is exactly what it used to do.
  driving = (mode == CTRL_VEL);
  lastCmdMillis = millis();
}

static void ctrlDisengage() {
  LOCK();
  ctrlMode = CTRL_OFF;
  cmdVel[0] = cmdVel[1] = 0.0f;
  velInteg[0] = velInteg[1] = 0.0f;
  UNLOCK();
  stopAll();
}

// ------------------------------------------------------- encoder spin test

// Drive each motor gently in each direction and report what BOTH encoders
// did. Watching the far encoder as well as the near one is how a swapped
// motor/encoder pair shows up. It now also checks the forward-positive sign
// convention: a FWD step must make that wheel's FORWARD count RISE, and if it
// does not, MOT_DIR/ENC_DIR for that side are wrong.
static const int      SPIN_DUTY      = 110;
static const uint32_t SPIN_RUN_MS    = 700;
static const uint32_t SPIN_SETTLE_MS = 400;

static void spinStep(int idx, bool forward, int duty) {
  Motor &m = motors[idx];
  int64_t before[2] = {fwdCountsOf(0), fwdCountsOf(1)};

  driveMotorFwd(idx, forward ? duty : -duty);

  // The control task keeps the counters fresh; we only have to wait.
  uint32_t t0 = millis();
  while (millis() - t0 < SPIN_RUN_MS) delay(2);

  stopAll();

  // Let the wheel coast to rest before reading, or we miss the tail.
  t0 = millis();
  while (millis() - t0 < SPIN_SETTLE_MS) delay(2);

  int64_t dL = fwdCountsOf(0) - before[0];
  int64_t dR = fwdCountsOf(1) - before[1];
  int64_t dNear = idx == 0 ? dL : dR;
  int64_t dFar  = idx == 0 ? dR : dL;

  const char *verdict;
  if (llabs(dNear) < 200)                        verdict = "NO MOTION";
  else if (llabs(dFar) > llabs(dNear) / 4)       verdict = "CROSSTALK - pairing?";
  else if ((dNear > 0) != forward)               verdict = "SIGN WRONG";
  else                                           verdict = "ok";

  Serial.printf("# SPIN %-5s %s duty=%3d -> fwdL %+8lld  fwdR %+8lld  (%+.3f m)  %s\n",
                m.name, forward ? "FWD" : "REV", duty,
                (long long)dL, (long long)dR,
                (double)dNear / COUNTS_PER_METRE, verdict);
}

static void spinTest(int duty) {
  ctrlDisengage();
  Serial.printf("# ---- spin test start (duty %d) ----\n", duty);
  Serial.println("# expect: FWD raises that side's fwd count, REV lowers it,");
  Serial.println("#         and the OTHER side barely moves.");
  for (int i = 0; i < 2; i++) {
    spinStep(i, true, duty);
    delay(300);
    spinStep(i, false, duty);
    delay(300);
  }
  stopAll();
  Serial.println("# ---- spin test done ----");
}

// ----------------------------------------------------- continuous run mode

// Used by wheel_cal.py: hold one motor turning at a fixed duty while a human
// marks each quarter turn of the WHEEL.
static const uint32_t RUN_MAX_MS = 180000;

static int      runSide  = -1;
static uint32_t runUntil = 0;

static void runStop(const char *why) {
  if (runSide < 0) return;
  stopAll();
  runSide = -1;
  Serial.printf("# RUNSTOP %s\n", why);
}

static void runStart(int side, int duty, bool forward) {
  ctrlDisengage();
  if (side < 0 || side > 1) return;
  Motor &m = motors[side];
  ledcWrite(forward ? m.lpwmCh : m.rpwmCh, 0);
  ledcWrite(forward ? m.rpwmCh : m.lpwmCh, duty);
  runSide  = side;
  runUntil = millis() + RUN_MAX_MS;
  Serial.printf("# RUN %s %s duty=%d\n", m.name, forward ? "FWD" : "REV", duty);
}

static void markNow() {
  Serial.printf("# MARK,%lld,%lld,%lu\n",
                (long long)encoders[0].counts,
                (long long)encoders[1].counts,
                (unsigned long)millis());
}

// ------------------------------------------------------ closed-loop rotate
//
// Rotate ONE wheel by a commanded number of WHEEL degrees. This predates the
// velocity cascade and is kept as-is because wheel_cal.py and rot.py depend
// on its exact protocol; it is a single-wheel positioning primitive, not a
// driving mode. It runs inside the control task now, so its brake and settle
// timing is no longer at the mercy of loop() jitter.
static const uint32_t GO_TIMEOUT_MS = 30000;
static const int      GO_CREEP_DUTY = 58;

static uint32_t GO_BRAKE_MS   = 120;
static uint32_t GO_SETTLE_MS  = 200;
static int      GO_TOL_COUNTS = 60;   // ~0.3 deg of wheel
static int      GO_TRIM_DUTY  = 70;
static int      GO_TRIM_MAX   = 10;

enum GoPhase { GO_IDLE, GO_RUN, GO_BRAKE, GO_SETTLE, GO_PULSE };

static int      goSide     = -1;
static GoPhase  goPhase    = GO_IDLE;
static int64_t  goTarget   = 0;
static int64_t  goStart    = 0;
static bool     goDescend  = false;
static int      goDuty     = 0;
static uint32_t goDeadline = 0;
static uint32_t goPhaseEnd = 0;
static int      goTrims    = 0;
static float    goCmdDeg   = 0.0f;

// Where each wheel SHOULD be if every commanded rotate had landed perfectly.
static int64_t goIdeal[2]   = {0, 0};
static bool    goIdealOk[2] = {false, false};

static void goForget(int side) {
  if (side < 0) { goIdealOk[0] = goIdealOk[1] = false; }
  else if (side < 2)  goIdealOk[side] = false;
}

// The control task must not print (it would interleave with loop()'s output
// and shred a line), so a finished move is parked here and loop() prints it.
static volatile bool goResultPending = false;
static char goResultLine[128];

static int64_t goRemaining() {
  int64_t rem = goTarget - encoders[goSide].counts;
  return goDescend ? -rem : rem;
}

static void goFinish(const char *why) {
  if (goSide < 0) return;
  int side = goSide;
  goSide  = -1;
  goPhase = GO_IDLE;
  stopAll();

  int64_t moved = encoders[side].counts - goStart;
  float deg = (float)moved * ENC_DIR[side] * MOT_DIR[side] / COUNTS_PER_WHEEL_DEG;
  float resid = (float)(goTarget - encoders[side].counts)
                * ENC_DIR[side] * MOT_DIR[side] / COUNTS_PER_WHEEL_DEG;

  bool ok = (strcmp(why, "reached") == 0 || strcmp(why, "capped") == 0);
  if (ok) { goIdeal[side] = goTarget; goIdealOk[side] = true; }
  else    { goForget(side); }

  snprintf(goResultLine, sizeof(goResultLine),
           "# GODONE,%s,%.2f,%.2f,%.2f,%s,%d",
           motors[side].name, goCmdDeg, deg, resid, why, goTrims);
  goResultPending = true;
}

static void goStart_(int side, float degrees, int duty) {
  if (side < 0 || side > 1) return;
  ctrlDisengage();
  LOCK();
  goSide    = side;
  goCmdDeg  = degrees;
  goStart   = encoders[side].counts;
  goTrims   = 0;

  int64_t base = encoders[side].counts;
  if (goIdealOk[side] &&
      llabs(goIdeal[side] - encoders[side].counts) <
          (int64_t)(45.0f * COUNTS_PER_WHEEL_DEG)) {
    base = goIdeal[side];
  }

  // A positive commanded angle drives the wheel forwards, in the same
  // forward-positive sense as everything else.
  int64_t delta = (int64_t)llroundf(degrees * COUNTS_PER_WHEEL_DEG)
                  * ENC_DIR[side] * MOT_DIR[side];
  goTarget   = base + delta;
  goDescend  = (goTarget < encoders[side].counts);
  goDuty     = constrain(duty, GO_CREEP_DUTY, 255);
  goDeadline = millis() + GO_TIMEOUT_MS;
  goPhase    = GO_RUN;
  UNLOCK();
  Serial.printf("# GO,%s,%.2f,%d,%lld\n",
                motors[side].name, degrees, goDuty,
                (long long)(goTarget - encoders[side].counts));
}

static void goBrakeSettle(bool trimming) {
  motorBrake(motors[goSide]);
  goPhase    = GO_BRAKE;
  goPhaseEnd = millis() + (trimming ? GO_BRAKE_MS / 2 : GO_BRAKE_MS);
}

static void goService() {
  if (goSide < 0) return;
  if ((int32_t)(millis() - goDeadline) >= 0) { goFinish("timeout"); return; }

  Motor &m = motors[goSide];

  switch (goPhase) {
  case GO_RUN: {
    int64_t left = goRemaining();
    if (left <= 0) { goBrakeSettle(false); return; }
    int64_t total = llabs(goTarget - goStart);
    int duty = goDuty;
    if (total > 0 && left * 5 < total) {
      duty = GO_CREEP_DUTY + (int)((goDuty - GO_CREEP_DUTY) * left * 5 / total);
    }
    motorDrive(m, !goDescend, duty);
    return;
  }
  case GO_BRAKE:
    if ((int32_t)(millis() - goPhaseEnd) < 0) return;
    motorCoast(m);
    goPhase    = GO_SETTLE;
    goPhaseEnd = millis() + (goTrims ? GO_SETTLE_MS / 2 : GO_SETTLE_MS);
    return;
  case GO_SETTLE: {
    if ((int32_t)(millis() - goPhaseEnd) < 0) return;
    int64_t rem = goRemaining();
    if (llabs(rem) <= GO_TOL_COUNTS) { goFinish("reached"); return; }
    if (goTrims >= GO_TRIM_MAX)      { goFinish("capped");  return; }
    goTrims++;
    uint32_t ms = (uint32_t)constrain((int)(llabs(rem) / 12), 12, 60);
    bool countUp = goDescend ? (rem < 0) : (rem > 0);
    motorDrive(m, countUp, GO_TRIM_DUTY);
    goPhase    = GO_PULSE;
    goPhaseEnd = millis() + ms;
    return;
  }
  case GO_PULSE:
    if ((int32_t)(millis() - goPhaseEnd) < 0) return;
    goBrakeSettle(true);
    return;
  default:
    return;
  }
}

// ---------------------------------------------------------------- HWT905 IMU
//
// A WitMotion HWT905-TTL on UART0 RX (GPIO 20). The ESP32 only ever LISTENS:
// the sensor free-runs at 100 Hz and is configured from the WitMotion app, so
// TX is left unattached and GPIO 21 stays free for the status LED.
//
// WHAT IT IS FOR. Wheel odometry cannot measure heading. It infers yaw from
// the difference between the two wheel distances divided by the track width,
// which (a) is scaled by a track width nobody has measured on this car, and
// (b) reads a slipping tyre as a perfect straight line. The gyro measures the
// body's actual rotation and is blind to both problems. Fusing them gives a
// heading that survives slip, and a wheel-vs-gyro disagreement that is itself
// the slip detector.
//
// PACKET FORMAT: fixed 11 bytes -- 0x55, TYPE, four little-endian int16, then
// a checksum that is the low byte of the sum of the first ten.
//   0x51 accel   ax ay az temp   +-16 g
//   0x52 gyro    wx wy wz temp   +-2000 deg/s
//   0x53 euler   roll pitch yaw  +-180 deg
// Only 0x52 feeds the control law. The 0x53 euler yaw is NOT used: it is the
// sensor's own drifting integral, and on this unit roll/pitch carry a fixed
// 1.16 deg offset that the on-chip accel calibration does not remove.
static const int8_t PIN_IMU_RX = 20;
static const int8_t PIN_IMU_TX = -1;      // never talk back; keeps 21 free
static const uint32_t IMU_BAUD = 115200;

// Sign of the sensor's Z axis relative to the car's yaw (CCW positive). It
// depends on which way up the HWT905 ends up bolted, so it is NOT a constant
// to be trusted -- verify it with `U` once the sensor is mounted and flip it
// with `U -1` if a left turn reads negative.
static float IMU_YAW_SIGN = +1.0f;

// Complementary filter weight on the gyro. The measured stationary drift on
// this unit is 0.012-0.126 deg/min, which is two orders of magnitude better
// than a track width that is still a guess, so the gyro carries almost all of
// the heading and the wheels only stop it walking away over minutes.
static const float IMU_ALPHA = 0.98f;

// Health: the stream has to be fresh, or fusion silently falls back to wheels.
static const uint32_t IMU_STALE_MS = 500;

static volatile bool   imuPresent    = false;   // has ever decoded a packet
static volatile uint32_t imuLastMs   = 0;
static volatile uint32_t imuPackets  = 0;
static volatile uint32_t imuBadSum   = 0;
static volatile float  imuGyroZ      = 0.0f;    // deg/s, bias-corrected
static volatile float  imuGyroZRaw   = 0.0f;    // deg/s, as received
static volatile float  imuAccel[3]   = {0, 0, 0};
static volatile float  imuBiasZ      = 0.0f;    // deg/s, estimated at rest
static volatile uint32_t imuBiasN    = 0;       // samples in the bias average

// Two headings, kept side by side on purpose. poseTh is the fused one that
// everything downstream uses; poseThWheel is what the wheels alone claim.
// Their difference is slip plus track-width error, and it is the single most
// useful number on this car for telling a mechanical problem from a maths one.
static double poseThWheel = 0;

static inline bool imuFresh() {
  return imuPresent && (millis() - imuLastMs) < IMU_STALE_MS;
}

static void imuInit() {
  Serial1.begin(IMU_BAUD, SERIAL_8N1, PIN_IMU_RX, PIN_IMU_TX);
}

// Bias nulling (ZUPT). A gyro's yaw is an integral, so a constant bias becomes
// an error that grows forever. The one moment we KNOW the true rate is zero is
// when the car is commanded to stand still and both encoders agree it is, so
// that is when -- and only when -- the bias average is allowed to move.
static void imuAccumulateBias(float raw) {
  if (imuBiasN < 20000) {                 // ~200 s at 100 Hz, then it is set
    imuBiasZ += (raw - imuBiasZ) / (float)(imuBiasN + 1);
    imuBiasN++;
  }
}

// Feed one byte into the frame decoder. Returns true when a packet completed.
static bool imuFeed(uint8_t b, bool carStill) {
  static uint8_t buf[11];
  static uint8_t n = 0;

  if (n == 0 && b != 0x55) return false;  // resync: nothing but a header here
  buf[n++] = b;
  if (n < 11) return false;
  n = 0;

  uint8_t sum = 0;
  for (int i = 0; i < 10; i++) sum += buf[i];
  if (sum != buf[10]) { imuBadSum++; return false; }

  auto i16 = [&](int i) -> int16_t {
    return (int16_t)((uint16_t)buf[2 + 2 * i] | ((uint16_t)buf[3 + 2 * i] << 8));
  };

  switch (buf[1]) {
  case 0x51:
    for (int i = 0; i < 3; i++)
      imuAccel[i] = i16(i) * (16.0f * 9.80665f / 32768.0f);
    break;
  case 0x52: {
    float wz = i16(2) * (2000.0f / 32768.0f);
    imuGyroZRaw = wz;
    if (carStill) imuAccumulateBias(wz);
    imuGyroZ = wz - imuBiasZ;
    break;
  }
  default:
    break;                                // 0x53 euler deliberately ignored
  }

  imuPresent = true;
  imuLastMs  = millis();
  imuPackets++;
  return true;
}

// Drain whatever arrived. Called from the control task so the timing is even;
// a 100 Hz sensor at 11 bytes is 1.1 kB/s, which is nothing at 500 Hz.
static void imuPoll(bool carStill) {
  int guard = 64;                         // never let a noisy line stall a tick
  while (Serial1.available() && guard-- > 0)
    imuFeed((uint8_t)Serial1.read(), carStill);
}

// ------------------------------------------------------------ the controller

// Measured feedforward: the duty this wheel needs to hold `v` m/s with no
// error at all. Getting this roughly right is what lets the PI gains stay
// small, which is what keeps the loop stable.
static float feedForward(float v) {
  if (fabsf(v) < 1e-3f) return 0.0f;
  float s = v > 0 ? 1.0f : -1.0f;
  return s * (FF_DEAD + fabsf(v) * FF_SLOPE);
}

static void controlStep() {
  // --- measure -------------------------------------------------------------
  static int64_t prevCounts[2] = {0, 0};
  static int64_t posePrev[2]   = {0, 0};

  int64_t now[2] = {fwdCountsOf(0), fwdCountsOf(1)};

  if (ctrlResync) {
    ctrlResync = false;
    for (int i = 0; i < 2; i++) {
      prevCounts[i] = now[i];
      posePrev[i]   = now[i];
      fwdVel[i]     = 0.0f;
    }
  }

  for (int i = 0; i < 2; i++) {
    float raw = (float)(now[i] - prevCounts[i]) / COUNTS_PER_METRE / CTRL_DT;
    fwdVel[i] += VEL_ALPHA * (raw - fwdVel[i]);
    prevCounts[i] = now[i];
  }

  // --- pose ----------------------------------------------------------------
  // Standard differential-drive integration. dl/dr are this tick's wheel
  // travel; the midpoint of the two is the body's forward step and their
  // difference over the track width is the yaw step.
  {
    double dl = (double)(now[0] - posePrev[0]) / COUNTS_PER_METRE;
    double dr = (double)(now[1] - posePrev[1]) / COUNTS_PER_METRE;
    posePrev[0] = now[0]; posePrev[1] = now[1];
    double ds = 0.5 * (dl + dr);

    // What the wheels alone claim. Scaled by a track width nobody has
    // measured, and blind to a slipping tyre -- kept anyway, because the gap
    // between this and the fused heading is what exposes both problems.
    double dthWheel = (dr - dl) / TRACK_WIDTH_M;
    poseThWheel += dthWheel;
    if (poseThWheel >  M_PI) poseThWheel -= 2 * M_PI;
    if (poseThWheel < -M_PI) poseThWheel += 2 * M_PI;

    // The car is standing still when nothing is commanded and neither wheel
    // is turning. That is the only safe moment to learn the gyro's bias.
    bool carStill = (ctrlMode == CTRL_OFF) &&
                    (fabs(dl) < 1e-6) && (fabs(dr) < 1e-6);
    imuPoll(carStill);

    // Complementary fusion. The gyro carries the heading; the wheels stop it
    // walking away over minutes. With no IMU this degrades exactly to the
    // wheel-only behaviour it had before, which is why it is safe to ship
    // before the sensor is bolted on.
    double dth;
    if (imuFresh()) {
      double dthGyro = (double)(IMU_YAW_SIGN * imuGyroZ) * (M_PI / 180.0)
                       * CTRL_DT;
      dth = IMU_ALPHA * dthGyro + (1.0 - IMU_ALPHA) * dthWheel;
    } else {
      dth = dthWheel;
    }

    poseX  += ds * cos(poseTh + dth * 0.5);
    poseY  += ds * sin(poseTh + dth * 0.5);
    poseTh += dth;
    if (poseTh >  M_PI) poseTh -= 2 * M_PI;
    if (poseTh < -M_PI) poseTh += 2 * M_PI;
  }

  if (ctrlMode == CTRL_OFF) {
    // Keep the reference glued to the wheels while the loop is idle, so
    // engaging never produces a step, and let the integrators bleed off.
    for (int i = 0; i < 2; i++) {
      refCounts[i] = (double)now[i];
      posErrM[i]   = 0.0f;
      velInteg[i]  = 0.0f;
    }
    refScale = 1.0f;
    return;
  }

  // --- reference governor --------------------------------------------------
  // The whole straight-line guarantee lives in these six lines. When one
  // wheel cannot keep up, BOTH references slow, so the pair never drifts
  // apart -- the base just goes slower than asked and stays straight.
  float lag = fmaxf(fabsf(posErrM[0]), fabsf(posErrM[1]));
  if (lag <= POS_ERR_SOFT)      refScale = 1.0f;
  else if (lag >= POS_ERR_HARD) refScale = 0.0f;
  else refScale = (POS_ERR_HARD - lag) / (POS_ERR_HARD - POS_ERR_SOFT);

  for (int i = 0; i < 2; i++) {
    // --- outer loop: position ---------------------------------------------
    refCounts[i] += (double)cmdVel[i] * refScale * COUNTS_PER_METRE * CTRL_DT;
    posErrM[i] = (float)((refCounts[i] - (double)now[i]) / COUNTS_PER_METRE);

    float velSp = cmdVel[i] * refScale + KP_POS * posErrM[i];
    velSp = constrain(velSp, -VEL_MAX, VEL_MAX);

    // --- inner loop: velocity ---------------------------------------------
    float velErr = velSp - fwdVel[i];
    float ff = feedForward(velSp);
    float unsat = ff + KP_VEL * velErr + KI_VEL * velInteg[i];

    // Integrate, then back-calculate against saturation. Winding up the
    // integrator against a wheel that physically cannot go faster is the
    // classic way a cascade like this turns into a lurch when the load
    // clears, so the integrator only accumulates while there is headroom.
    float sat = constrain(unsat, -255.0f, 255.0f);
    if (fabsf(unsat - sat) < 0.01f || (velErr > 0) != (unsat > 0)) {
      velInteg[i] += velErr * CTRL_DT;
      velInteg[i] = constrain(velInteg[i], -255.0f / KI_VEL, 255.0f / KI_VEL);
    }

    // An offline encoder means the feedback is a lie -- do not drive on it.
    if (!encoders[i].online) { driveMotorFwd(i, 0); continue; }
    driveMotorFwd(i, (int)lroundf(sat));
  }
}

// The control task. This is the only thing in the firmware that touches I2C,
// and while a closed loop is engaged it is the only thing that touches PWM.
static void controlTask(void *) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / CTRL_HZ);
  uint32_t magnetDiv = 0;

  for (;;) {
    uint32_t t0 = micros();

    if (scanRequest) { scanRequest = false; i2cScan(); }

    uint32_t tI2c = micros();
    sampleAllEncoders();

    // Magnetometer at 100 Hz. One 6-byte burst on the main bus, so it costs
    // a single transaction and no mux write.
    static uint32_t magDiv = 0;
    if (magPresent && ++magDiv >= CTRL_HZ / 100) { magDiv = 0; magSample(); }

    // Airgap registers at ~5 Hz, and never while the wheels are turning:
    // they cost a second I2C round trip each and nothing in the control law
    // reads them.
    if (++magnetDiv >= CTRL_HZ / 5) {
      magnetDiv = 0;
      if (!driving && ctrlMode == CTRL_OFF && goSide < 0 && runSide < 0) {
        sampleAllMagnets();
      }
    }

    uint32_t i2cUs = micros() - tI2c;
    if (i2cUs > ctrlI2cUs) ctrlI2cUs = i2cUs;

    uint32_t tMath = micros();
    LOCK();
    controlStep();
    if (ctrlMode == CTRL_OFF) goService();   // legacy rotate, only when idle
    UNLOCK();
    uint32_t mathUs = micros() - tMath;
    if (mathUs > ctrlMathUs) ctrlMathUs = mathUs;
    ctrlMathSum += mathUs;
    ctrlMathN++;

    uint32_t dt = micros() - t0;
    if (dt > ctrlMaxUs) ctrlMaxUs = dt;
    if (dt > (1000000UL / CTRL_HZ)) ctrlOverruns++;
    ctrlTicks++;

    vTaskDelayUntil(&last, period);
  }
}

static void zeroEncoders() {
  LOCK();
  for (auto &e : encoders) {
    e.counts = 0;
    e.seeded = false;   // re-seed on next sample so we don't bank a jump
  }
  refCounts[0] = refCounts[1] = 0;
  poseX = poseY = poseTh = 0;
  ctrlResync = true;      // counters just jumped; do not integrate the jump
  goForget(-1);
  UNLOCK();
}

// x, y in [-1, 1] -- y forward/back, x left/right.
//
// The joystick now commands VELOCITY, not duty. That is the whole point: let
// go of the stick off-centre and the wheels hold the speed you asked for, and
// push the stick straight forward and the car goes straight because the two
// wheels are servoed to the same position reference.
void applyStick(float x, float y) {
  float lin = constrain(y, -1.0f, 1.0f) * VEL_MAX;
  float rot = constrain(x, -1.0f, 1.0f) * VEL_MAX;
  if (fabs(x) < 0.02f && fabs(y) < 0.02f) {
    // Centre stick holds position rather than coasting -- the car stops
    // where it is and stays there.
    ctrlEngage(0.0f, 0.0f, CTRL_HOLD);
  } else {
    ctrlEngage(lin + rot, lin - rot, CTRL_VEL);
  }
  goForget(-1);
}

// ---------------------------------------------------------------- page

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no, viewport-fit=cover">
<title>ESP32 RC</title>
<style>
  html,body { margin:0; height:100%; background:#111; color:#eee;
              font-family:system-ui,sans-serif; overflow:hidden;
              touch-action:none; -webkit-user-select:none; user-select:none; }
  #status { position:fixed; top:8px; left:0; right:0; text-align:center;
            font-size:13px; opacity:.7; }
  #status.ok { color:#4f8; }
  #status.bad { color:#f55; }
  #odo { position:fixed; top:30px; left:0; right:0; display:flex;
         justify-content:center; gap:8px; flex-wrap:wrap; }
  .cell { background:#1a1a1a; border:1px solid #333; border-radius:10px;
          padding:6px 12px; min-width:88px; text-align:center; }
  .cell .lbl { font-size:10px; letter-spacing:.08em; opacity:.55; }
  .cell .val { font-size:22px; font-variant-numeric:tabular-nums;
               font-weight:600; margin-top:2px; }
  .cell .val.off { color:#f55; font-size:14px; }
  .cell.diff .val { font-size:17px; color:#6cf; }
  .cell.diff .val.bad { color:#f55; }
  #zero { position:fixed; bottom:16px; left:16px; padding:14px 22px;
          font-size:15px; color:#eee; background:#333; border:none;
          border-radius:10px; }
  #wrap { position:fixed; inset:0; display:flex; align-items:center; justify-content:center; }
  #base { position:relative; width:min(58vw,58vh); height:min(58vw,58vh);
          border-radius:50%; background:#1c1c1c; border:2px solid #333;
          box-shadow: inset 0 0 30px #000; }
  #knob { position:absolute; width:34%; height:34%; border-radius:50%;
          background:radial-gradient(circle at 35% 35%, #6cf, #159);
          top:33%; left:33%; box-shadow:0 4px 12px #000a; }
  #stop { position:fixed; bottom:16px; left:50%; transform:translateX(-50%);
          padding:14px 40px; font-size:18px; font-weight:bold; color:#fff;
          background:#a11; border:none; border-radius:10px; }
</style>
</head>
<body>
<div id="status" class="bad">connecting...</div>
<div id="odo">
  <div class="cell"><div class="lbl">LEFT</div><div class="val" id="lv">--</div></div>
  <div class="cell"><div class="lbl">RIGHT</div><div class="val" id="rv">--</div></div>
  <div class="cell diff"><div class="lbl">SKEW mm</div><div class="val" id="dv">--</div></div>
  <div class="cell diff"><div class="lbl">m/s L|R</div><div class="val" id="sv">--</div></div>
</div>
<div id="wrap"><div id="base"><div id="knob"></div></div></div>
<button id="zero">ZERO</button>
<button id="stop">STOP</button>
<script>
const statusEl = document.getElementById('status');
const base = document.getElementById('base');
const knob = document.getElementById('knob');
const lv = document.getElementById('lv');
const rv = document.getElementById('rv');
const dv = document.getElementById('dv');
const sv = document.getElementById('sv');
let ws, active = false, cx=0, cy=0, r=0, x=0, y=0;
let sendTimer = null;

function fmtCell(el, m, ok) {
  if (!ok) { el.textContent = 'NO I2C'; el.className = 'val off'; return; }
  el.textContent = m.toFixed(3) + ' m';
  el.className = 'val';
}

function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = () => { statusEl.textContent = 'connected'; statusEl.className = 'ok'; };
  ws.onclose = () => { statusEl.textContent = 'disconnected -- retrying'; statusEl.className = 'bad'; setTimeout(connect, 500); };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    let t;
    try { t = JSON.parse(ev.data); } catch (e) { return; }
    if (t.type !== 'odo') return;
    fmtCell(lv, t.l, t.lok);
    fmtCell(rv, t.r, t.rok);
    if (t.lok && t.rok) {
      const mm = (t.l - t.r) * 1000;
      dv.textContent = mm.toFixed(1);
      dv.className = Math.abs(mm) > 15 ? 'val bad' : 'val';
      sv.textContent = t.vl.toFixed(2) + '|' + t.vr.toFixed(2);
    } else { dv.textContent = '--'; sv.textContent = '--'; }
  };
}
connect();

function send(x, y) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(x.toFixed(2) + ',' + y.toFixed(2));
}

function startSendLoop() {
  if (sendTimer) return;
  sendTimer = setInterval(() => { if (active) send(x, y); }, 50); // 20 Hz
}

function setKnob(px, py) {
  knob.style.left = (50 + px * 33) + '%';
  knob.style.top  = (50 - py * 33) + '%';
}

function pointerDown(clientX, clientY) {
  const rect = base.getBoundingClientRect();
  cx = rect.left + rect.width / 2;
  cy = rect.top + rect.height / 2;
  r = rect.width / 2;
  active = true;
  startSendLoop();
  pointerMove(clientX, clientY);
}

function pointerMove(clientX, clientY) {
  if (!active) return;
  let dx = (clientX - cx) / r;
  let dy = (clientY - cy) / r;
  const mag = Math.hypot(dx, dy);
  if (mag > 1) { dx /= mag; dy /= mag; }
  x = dx; y = -dy;
  setKnob(x, y);
  send(x, y);
}

function pointerUp() {
  active = false;
  x = 0; y = 0;
  setKnob(0, 0);
  send(0, 0);
}

base.addEventListener('pointerdown', e => { base.setPointerCapture(e.pointerId); pointerDown(e.clientX, e.clientY); });
base.addEventListener('pointermove', e => pointerMove(e.clientX, e.clientY));
base.addEventListener('pointerup', pointerUp);
base.addEventListener('pointercancel', pointerUp);

document.getElementById('zero').addEventListener('click', () => {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send('Z');
});
document.getElementById('stop').addEventListener('click', () => { pointerUp(); send(0,0); });
document.addEventListener('visibilitychange', () => { if (document.hidden) { pointerUp(); send(0,0); } });
</script>
</body>
</html>
)HTML";

// ---------------------------------------------------------------- ws

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("ws client #%u connected\n", client->id());
    ctrlDisengage();
    lastCmdMillis = millis();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("ws client #%u disconnected\n", client->id());
    ctrlDisengage();
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      char buf[32];
      size_t n = min(len, sizeof(buf) - 1);
      memcpy(buf, data, n);
      buf[n] = 0;
      if (buf[0] == 'Z') {
        zeroEncoders();
        Serial.println("encoders zeroed");
        return;
      }
      float x = 0, y = 0;
      if (sscanf(buf, "%f,%f", &x, &y) == 2) {
        applyStick(x, y);
      }
    }
  }
}

// ---------------------------------------------------------------- radio

static void radioDown() {
  if (radioUp) {
    ws.closeAll();
    ctrlDisengage();          // never leave a wheel driving on a dead link
  }
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  radioUp = false;
  Serial.println("# radio: WiFi OFF");
}

static bool radioUpNow(uint32_t timeout_ms = 8000) {
  if (radioUp) return true;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeout_ms) delay(200);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("# radio: WiFi connect FAILED, leaving it off");
    radioDown();
    return false;
  }
  if (!serverBegun) {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
      req->send_P(200, "text/html", INDEX_HTML);
    });
    server.begin();
    serverBegun = true;
  }
  if (MDNS.begin("esp32rc")) Serial.println("# mDNS: http://esp32rc.local");
  radioUp = true;
  Serial.printf("# radio: WiFi ON, IP %s\n", WiFi.localIP().toString().c_str());
  return true;
}

// Bluetooth: never started by this build. Disable and release the controller
// memory explicitly so the state is defined rather than merely assumed.
static void btDown() {
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
    esp_bt_controller_disable();
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
    esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
}

// ---------------------------------------------------------------- setup

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nc3-webgui booting (closed-loop build)");

  ctrlMutex = xSemaphoreCreateMutex();
  recomputeCal();

  // LED first, so the three boot flashes happen before anything that can hang.
  ledInit();
  ledBootFlash();

  // The IMU is listen-only, so bringing the UART up costs nothing when no
  // sensor is attached -- it just never decodes a packet and fusion stays off.
  imuInit();

  for (auto &m : motors) {
    ledcSetup(m.rpwmCh, PWM_FREQ, PWM_RES);
    ledcSetup(m.lpwmCh, PWM_FREQ, PWM_RES);
    ledcAttachPin(m.rpwmPin, m.rpwmCh);
    ledcAttachPin(m.lpwmPin, m.lpwmCh);
  }
  stopAll();

  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);

  // Report what actually answered, so a wiring fault is obvious at boot.
  Wire.beginTransmission(MUX_ADDR);
  bool muxOk = (Wire.endTransmission() == 0);
  Serial.printf("PCA9548A @0x%02X: %s\n", MUX_ADDR, muxOk ? "OK" : "NO ACK");
  if (muxOk) {
    for (auto &e : encoders) {
      sampleEncoder(e);
      Serial.printf("AS5600 %-5s (mux ch mask 0x%02X): %s\n",
                    e.name, e.muxMask, e.online ? "OK" : "NO ACK");
    }
  }

  magPresent = magInit();
  Serial.printf("QMC5883L @0x%02X: %s\n", QMC_ADDR,
                magPresent ? "OK (magnetometer, main bus)" : "not found");

  Serial.printf("status LED: GPIO %u, %s\n", LED_PIN,
                LED_ACTIVE_LOW ? "active low" : "active high");
  Serial.printf("HWT905 IMU: UART0 RX GPIO %d @ %lu -- listening (no packet yet)\n",
                PIN_IMU_RX, (unsigned long)IMU_BAUD);

  Serial.printf("calibration: gear %.4f:1, wheel %.1f mm -> %.0f counts/wheel-rev, "
                "%.0f counts/m, %.2f counts/wheel-deg (SAME both wheels)\n",
                gearRatio, wheelDiameterM * 1000.0f, COUNTS_PER_WHEEL_REV,
                COUNTS_PER_METRE, COUNTS_PER_WHEEL_DEG);
  Serial.printf("control: %lu Hz, KP_VEL=%.0f KI_VEL=%.0f KP_POS=%.1f, "
                "vmax=%.2f m/s\n",
                (unsigned long)CTRL_HZ, KP_VEL, KI_VEL, KP_POS, VEL_MAX);
  Serial.printf("track width %.3f m (%s)\n", TRACK_WIDTH_M,
                TRACK_WIDTH_MEASURED ? "measured" : "PLACEHOLDER - set with B");

  // Priority 5 sits above the Arduino loop task (1) and below the WiFi and
  // TCP/IP stacks (18-23), so control preempts application work but never
  // starves the radio. 4 kB is comfortable for this task's tiny frame.
  xTaskCreate(controlTask, "ctrl", 4096, nullptr, 5, nullptr);

  btDown();
  Serial.println("radio: Bluetooth OFF (never started; controller released)");
  if (RADIO_ON_AT_BOOT) {
    radioUpNow();
  } else {
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
    radioUp = false;
    Serial.println("radio: WiFi OFF -- 'A 1' brings it up for the phone GUI");
  }
  ethBegin();

  Serial.printf("free heap %lu\n", (unsigned long)ESP.getFreeHeap());
  Serial.println("ready");
}

// ---------------------------------------------------------------- commands

static uint32_t fbPeriodMs = 20;    // FB telemetry, 50 Hz default

// Defined below, next to loop(), but the Y command needs it here.
static void ethProbe();

static void printHelp() {
  Serial.println("# commands:");
  Serial.println("#  V <l> <r>   closed-loop wheel velocity, m/s, forward-positive");
  Serial.println("#  W <v> <w>   unicycle: linear m/s, angular rad/s");
  Serial.println("#  H           hold position (resists being pushed)");
  Serial.println("#  E           disengage closed loop, motors idle");
  Serial.println("#  D <l> <r>   RAW open-loop duty -255..255 (no feedback)");
  Serial.println("#  G <L|R> <deg> [duty]  single-wheel rotate");
  Serial.println("#  Z           zero counters, reference and pose");
  Serial.println("#  S [duty]    spin test + sign check");
  Serial.println("#  X           stop everything");
  Serial.println("#  L [pin|T|I] status LED: report / move / identify / invert");
  Serial.println("#  U [1|-1]    IMU status, heading fusion, yaw sign");
  Serial.println("#  Y           probe the W5500 over SPI");
  Serial.println("#  C           Ethernet status: link, IP, batching, OTA count");
  Serial.println("#  K [kpv kiv kpp soft hard]  read/set cascade gains");
  Serial.println("#  N [ratio] [wheel_mm]  read/set gear ratio + wheel dia");
  Serial.println("#  B [metres]  read/set track width");
  Serial.println("#  F [hz]      FB telemetry rate, 0 = off");
  Serial.println("#  P [...]     rotate settling params");
  Serial.println("#  A [0|1]    WiFi off/on (off at boot; BT always off)");
  Serial.println("#  I           scan the I2C bus and every mux channel");
  Serial.println("#  Q           control loop health");
  Serial.println("#  M           mark counters now");
  Serial.println("#  R <L|R> <duty> [F|B]  raw continuous run");
}

static void handleCommand(char *cmd) {
  switch (cmd[0]) {

  case 'C': case 'c':
    ethStatus();
    break;

  case 'V': case 'v': {
    float l = 0, r = 0;
    if (sscanf(cmd + 1, " %f %f", &l, &r) == 2) {
      if (goSide >= 0) { LOCK(); goFinish("preempted"); UNLOCK(); }
      runSide = -1;
      ctrlEngage(l, r, CTRL_VEL);
    } else Serial.println("# ERR usage: V <left_mps> <right_mps>");
    break;
  }

  case 'W': case 'w': {
    float v = 0, w = 0;
    if (sscanf(cmd + 1, " %f %f", &v, &w) == 2) {
      if (goSide >= 0) { LOCK(); goFinish("preempted"); UNLOCK(); }
      runSide = -1;
      float half = 0.5f * w * TRACK_WIDTH_M;
      ctrlEngage(v - half, v + half, CTRL_VEL);
      if (!TRACK_WIDTH_MEASURED && fabsf(w) > 1e-3f)
        Serial.println("# WARN track width is a placeholder; yaw is not trustworthy");
    } else Serial.println("# ERR usage: W <linear_mps> <angular_radps>");
    break;
  }

  case 'H': case 'h':
    if (goSide >= 0) { LOCK(); goFinish("preempted"); UNLOCK(); }
    runSide = -1;
    ctrlEngage(0.0f, 0.0f, CTRL_HOLD);
    Serial.println("# holding position");
    break;

  case 'E': case 'e':
    ctrlDisengage();
    Serial.println("# closed loop off");
    break;

  case 'Z': case 'z':
    zeroEncoders();
    Serial.println("# zeroed");
    break;

  case 'S': case 's': {
    int duty = SPIN_DUTY;
    sscanf(cmd + 1, "%d", &duty);
    spinTest(constrain(duty, 0, 255));
    break;
  }

  case 'D': case 'd': {
    int l = 0, r = 0;
    if (sscanf(cmd + 1, " %d %d", &l, &r) == 2) {
      ctrlDisengage();
      if (goSide >= 0) { LOCK(); goFinish("preempted"); UNLOCK(); }
      runSide = -1;
      if (l) goForget(0);
      if (r) goForget(1);
      driveMotor(motors[0], constrain(l, -255, 255) / 255.0f);
      driveMotor(motors[1], constrain(r, -255, 255) / 255.0f);
      driving = (l != 0 || r != 0);
      lastCmdMillis = millis();
    } else Serial.println("# ERR usage: D <left> <right>   (-255..255)");
    break;
  }

  case 'G': case 'g': {
    char side = 0; float deg = 0; int duty = 90;
    if (sscanf(cmd + 1, " %c %f %d", &side, &deg, &duty) >= 2) {
      int idx = (side == 'L' || side == 'l') ? 0
              : (side == 'R' || side == 'r') ? 1 : -1;
      if (idx < 0) Serial.println("# ERR side must be L or R");
      else goStart_(idx, deg, duty);
    } else Serial.println("# ERR usage: G <L|R> <degrees> [duty]");
    break;
  }

  case 'X': case 'x':
    if (goSide >= 0) { LOCK(); goFinish("aborted"); UNLOCK(); }
    runSide = -1;
    ctrlDisengage();
    Serial.println("# stopped");
    break;

  case 'K': case 'k': {
    float kpv = KP_VEL, kiv = KI_VEL, kpp = KP_POS;
    float sft = POS_ERR_SOFT, hrd = POS_ERR_HARD;
    sscanf(cmd + 1, " %f %f %f %f %f", &kpv, &kiv, &kpp, &sft, &hrd);
    LOCK();
    KP_VEL = constrain(kpv, 0.0f, 5000.0f);
    KI_VEL = constrain(kiv, 0.0f, 20000.0f);
    KP_POS = constrain(kpp, 0.0f, 200.0f);
    POS_ERR_SOFT = constrain(sft, 0.001f, 1.0f);
    POS_ERR_HARD = constrain(hrd, POS_ERR_SOFT + 0.001f, 2.0f);
    velInteg[0] = velInteg[1] = 0.0f;
    UNLOCK();
    Serial.printf("# K,kpv=%.1f,kiv=%.1f,kpp=%.2f,soft=%.3f,hard=%.3f\n",
                  KP_VEL, KI_VEL, KP_POS, POS_ERR_SOFT, POS_ERR_HARD);
    break;
  }

  case 'N': case 'n': {
    float g = gearRatio, d = wheelDiameterM * 1000.0f;
    int n = sscanf(cmd + 1, " %f %f", &g, &d);
    if (n >= 1) {
      LOCK();
      gearRatio = constrain(g, 1.0f, 500.0f);
      if (n >= 2) wheelDiameterM = constrain(d / 1000.0f, 0.005f, 1.0f);
      recomputeCal();
      // The reference is stored in counts, which have just changed meaning;
      // re-base it on the wheels so the loop does not jump.
      for (int i = 0; i < 2; i++) { refCounts[i] = (double)fwdCountsOf(i);
                                    velInteg[i] = 0.0f; }
      UNLOCK();
    }
    Serial.printf("# N,gear=%.4f,wheel_mm=%.1f,counts_per_wheel_rev=%.1f,"
                  "counts_per_m=%.1f,counts_per_deg=%.3f,ff_slope=%.1f\n",
                  gearRatio, wheelDiameterM * 1000.0f, COUNTS_PER_WHEEL_REV,
                  COUNTS_PER_METRE, COUNTS_PER_WHEEL_DEG, FF_SLOPE);
    break;
  }

  case 'B': case 'b': {
    float tw = TRACK_WIDTH_M;
    if (sscanf(cmd + 1, " %f", &tw) == 1) {
      TRACK_WIDTH_M = constrain(tw, 0.02f, 2.0f);
      TRACK_WIDTH_MEASURED = true;
    }
    Serial.printf("# B,%.4f,%s\n", TRACK_WIDTH_M,
                  TRACK_WIDTH_MEASURED ? "measured" : "placeholder");
    break;
  }

  case 'F': case 'f': {
    int hz = 0;
    if (sscanf(cmd + 1, " %d", &hz) == 1)
      fbPeriodMs = hz <= 0 ? 0 : (uint32_t)constrain(1000 / hz, 5, 2000);
    Serial.printf("# F,%lu ms\n", (unsigned long)fbPeriodMs);
    break;
  }

  case 'P': case 'p': {
    int tol = GO_TOL_COUNTS, td = GO_TRIM_DUTY;
    int bms = GO_BRAKE_MS, sms = GO_SETTLE_MS, mx = GO_TRIM_MAX;
    sscanf(cmd + 1, " %d %d %d %d %d", &tol, &td, &bms, &sms, &mx);
    GO_TOL_COUNTS = constrain(tol, 4, 4000);
    GO_TRIM_DUTY  = constrain(td, GO_CREEP_DUTY, 255);
    GO_BRAKE_MS   = (uint32_t)constrain(bms, 0, 2000);
    GO_SETTLE_MS  = (uint32_t)constrain(sms, 20, 3000);
    GO_TRIM_MAX   = constrain(mx, 0, 60);
    Serial.printf("# P,tol=%d(%.2fdeg),trimduty=%d,brake=%lu,settle=%lu,maxtrim=%d\n",
                  GO_TOL_COUNTS, (double)GO_TOL_COUNTS / COUNTS_PER_WHEEL_DEG,
                  GO_TRIM_DUTY, (unsigned long)GO_BRAKE_MS,
                  (unsigned long)GO_SETTLE_MS, GO_TRIM_MAX);
    break;
  }

  case 'A': case 'a': {
    int on = -1;
    if (sscanf(cmd + 1, " %d", &on) == 1) {
      if (on) radioUpNow(); else radioDown();
    }
    Serial.printf("# A,wifi=%d,bt=0,heap=%lu\n", radioUp ? 1 : 0,
                  (unsigned long)ESP.getFreeHeap());
    break;
  }

  case 'L': case 'l': {
    char a = 0; int pin = -1;
    if (sscanf(cmd + 1, " %d", &pin) == 1 && pin >= 0 && pin <= 21) {
      // Moving the LED at runtime is the point: you can find out which pin it
      // is actually on without a reflash. It does NOT survive a reboot.
      ledSet(false);
      LED_PIN = (uint8_t)pin;
      ledInit();
      Serial.printf("# LED moved to GPIO %u\n", LED_PIN);
      if (LED_PIN == PIN_IMU_RX)
        Serial.println("# WARN GPIO 20 is the HWT905's RX. The LED works here,"
                       " but the IMU cannot.");
      ledIdentifyUntil = millis() + 3000;
    } else if (sscanf(cmd + 1, " %c", &a) == 1 && (a == 'T' || a == 't')) {
      ledIdentifyUntil = millis() + 3000;
      Serial.printf("# LED identify: 10 Hz on GPIO %u for 3 s\n", LED_PIN);
    } else if (sscanf(cmd + 1, " %c", &a) == 1 && (a == 'I' || a == 'i')) {
      LED_ACTIVE_LOW = !LED_ACTIVE_LOW;
      ledInit();
      Serial.printf("# LED polarity: %s\n",
                    LED_ACTIVE_LOW ? "active LOW" : "active HIGH");
    } else {
      Serial.printf("# LED GPIO %u, %s, showing: %s\n", LED_PIN,
                    LED_ACTIVE_LOW ? "active low" : "active high", ledWhat);
      Serial.println("# patterns: 1 blip/s idle | 2 blips holding | solid driving");
      Serial.println("#           fast 5 Hz encoder fault | 3 blips wheel stalled");
      Serial.println("# usage: L <pin> | L T (identify) | L I (invert)");
    }
    break;
  }

  case 'U': case 'u': {
    float sgn = 0;
    if (sscanf(cmd + 1, " %f", &sgn) == 1 && (sgn == 1.0f || sgn == -1.0f)) {
      IMU_YAW_SIGN = sgn;
      Serial.printf("# IMU yaw sign %+.0f\n", IMU_YAW_SIGN);
      break;
    }
    Serial.printf("# HWT905 on UART0 RX GPIO %d @ %lu\n",
                  PIN_IMU_RX, (unsigned long)IMU_BAUD);
    if (!imuPresent) {
      Serial.println("# IMU: NO PACKETS EVER DECODED -- not wired, wrong baud,");
      Serial.println("#      or TX/RX swapped. Heading is wheels-only.");
    } else {
      Serial.printf("# IMU: %s  packets=%lu badsum=%lu  last %lu ms ago\n",
                    imuFresh() ? "STREAMING" : "STALE",
                    (unsigned long)imuPackets, (unsigned long)imuBadSum,
                    (unsigned long)(millis() - imuLastMs));
      Serial.printf("# gyroZ raw %+.3f deg/s, bias %+.4f (n=%lu), corrected %+.3f\n",
                    imuGyroZRaw, imuBiasZ, (unsigned long)imuBiasN, imuGyroZ);
      Serial.printf("# accel %+.3f %+.3f %+.3f m/s2\n",
                    imuAccel[0], imuAccel[1], imuAccel[2]);
    }
    Serial.printf("# heading: fused %+.2f deg, wheels-only %+.2f deg, gap %+.2f deg\n",
                  poseTh * 180.0 / M_PI, poseThWheel * 180.0 / M_PI,
                  (poseTh - poseThWheel) * 180.0 / M_PI);
    Serial.printf("# fusion %s (alpha %.2f on the gyro), yaw sign %+.0f\n",
                  imuFresh() ? "ACTIVE" : "off -- wheels only",
                  IMU_ALPHA, IMU_YAW_SIGN);
    Serial.println("# usage: U | U 1 | U -1   (flip if a left turn reads negative)");
    break;
  }

  case 'Y': case 'y':
    ethProbe();
    break;

  case 'I': case 'i':
    if (ctrlMode != CTRL_OFF || goSide >= 0 || runSide >= 0 || driving) {
      Serial.println("# ERR stop the motors first (X), the scan stalls sampling");
    } else {
      scanDone = false;
      scanRequest = true;
    }
    break;

  case 'Q': case 'q': {
    static uint32_t lastTicks = 0, lastMs = 0;
    uint32_t t = ctrlTicks, ms = millis();
    float hz = (lastMs && ms > lastMs)
             ? (float)(t - lastTicks) * 1000.0f / (float)(ms - lastMs) : 0.0f;
    lastTicks = t; lastMs = ms;
    float mathMean = ctrlMathN ? (float)ctrlMathSum / (float)ctrlMathN : 0.0f;
    Serial.printf("# Q,ticks=%lu,hz=%.1f,target=%lu,maxus=%lu,overruns=%lu,"
                  "i2cerr=%lu,mode=%d,heap=%lu,i2cmax=%lu,mathmax=%lu,"
                  "mathmean=%.1f\n",
                  (unsigned long)t, hz, (unsigned long)CTRL_HZ,
                  (unsigned long)ctrlMaxUs, (unsigned long)ctrlOverruns,
                  (unsigned long)i2cErrors, (int)ctrlMode,
                  (unsigned long)ESP.getFreeHeap(),
                  (unsigned long)ctrlI2cUs, (unsigned long)ctrlMathUs,
                  mathMean);
    ctrlMaxUs = 0; ctrlI2cUs = 0; ctrlMathUs = 0;
    ctrlMathSum = 0; ctrlMathN = 0;
    break;
  }

  case 'M': case 'm':
    markNow();
    break;

  case 'R': case 'r': {
    char side = 0, dir = 'F';
    int duty = 0;
    if (sscanf(cmd + 1, " %c %d %c", &side, &duty, &dir) >= 2) {
      int idx = (side == 'L' || side == 'l') ? 0
              : (side == 'R' || side == 'r') ? 1 : -1;
      if (idx < 0) Serial.println("# ERR side must be L or R");
      else { goForget(idx);
             runStart(idx, constrain(duty, 0, 255),
                      !(dir == 'B' || dir == 'b')); }
    } else Serial.println("# ERR usage: R <L|R> <duty> [F|B]");
    break;
  }

  case '?':
    printHelp();
    break;

  default:
    break;
  }
}

// ---------------------------------------------------------------- loop
//
// Command parsing and printing only. No I2C, and no PWM while a closed loop
// is engaged.

// ------------------------------------------------------------- W5500 probe
//
// There is no Ethernet stack in this firmware and this is not one. It answers
// exactly one question -- "is a W5500 actually on the SPI pins and alive?" --
// which is the question you have before you spend an afternoon on a stack.
//
// The test is VERSIONR (common block, 0x0039). The W5500 datasheet fixes it
// at 0x04, permanently, for every part. So 0x04 means a real W5500 is wired
// and clocking; 0x00 or 0xFF means MISO is dead, held, or nothing is there.
//
// PINS. GPIO 2 and 8 are ESP32-C3 strapping pins that must read HIGH at reset,
// so both need a 10k pull-up to 3V3 -- on GPIO 8 that pull-up doubles as SPI
// CS's correct idle-high state. This assignment is the one that leaves UART0
// (GPIO 20/21) free for the IMU and the status LED, and keeps MISO off GPIO 9.
static const int8_t PIN_ETH_SCLK = 5;
static const int8_t PIN_ETH_MISO = 10;
static const int8_t PIN_ETH_MOSI = 2;
static const int8_t PIN_ETH_CS   = 8;

static const uint32_t ETH_SPI_HZ = 8000000;   // W5500 is good to 33 MHz+
static bool ethSpiBegun = false;

static uint8_t w5500Read(uint16_t addr, uint8_t bsbRwbOm) {
  digitalWrite(PIN_ETH_CS, LOW);
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t)(addr & 0xFF));
  SPI.transfer(bsbRwbOm);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(PIN_ETH_CS, HIGH);
  return v;
}

static void ethProbe() {
  if (!ethSpiBegun) {
    pinMode(PIN_ETH_CS, OUTPUT);
    digitalWrite(PIN_ETH_CS, HIGH);
    SPI.begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    ethSpiBegun = true;
  }
  SPI.beginTransaction(SPISettings(ETH_SPI_HZ, MSBFIRST, SPI_MODE0));

  // Common register block, read, 1-byte op: BSB=0, RWB=0, OM=01 -> 0x01.
  uint8_t ver    = w5500Read(0x0039, 0x01);
  uint8_t mr     = w5500Read(0x0000, 0x01);
  uint8_t phycfg = w5500Read(0x002E, 0x01);

  // Read VERSIONR a second time. A floating MISO can land on 0x04 once by
  // luck; it does not do it twice in a row alongside a sane MR.
  uint8_t ver2   = w5500Read(0x0039, 0x01);
  SPI.endTransaction();

  Serial.printf("# ---- W5500 probe (SCLK=%d MISO=%d MOSI=%d CS=%d @ %lu Hz) ----\n",
                PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS,
                (unsigned long)ETH_SPI_HZ);
  Serial.printf("# VERSIONR 0x%02X / 0x%02X   MR 0x%02X   PHYCFGR 0x%02X\n",
                ver, ver2, mr, phycfg);

  if (ver == 0x04 && ver2 == 0x04) {
    Serial.println("# W5500: PRESENT (VERSIONR reads its fixed 0x04)");
    Serial.printf("# link: %s, %s, %s duplex\n",
                  (phycfg & 0x01) ? "UP" : "DOWN (no cable, or dead PHY)",
                  (phycfg & 0x02) ? "100 Mbps" : "10 Mbps",
                  (phycfg & 0x04) ? "full" : "half");
  } else if (ver == 0x00 && ver2 == 0x00) {
    Serial.println("# W5500: NOT ANSWERING (all zeroes) -- MISO stuck low, or");
    Serial.println("#        the part is unpowered / held in reset. Check 3V3,");
    Serial.println("#        the RST pull-up to 3V3, and the MISO wire.");
  } else if (ver == 0xFF && ver2 == 0xFF) {
    Serial.println("# W5500: NOT ANSWERING (all ones) -- MISO idle-high with");
    Serial.println("#        nothing driving it. Check CS on GPIO 8 and its");
    Serial.println("#        10k pull-up, and that MOSI/SCLK are not swapped.");
  } else {
    Serial.println("# W5500: GARBAGE -- something is on the bus but it is not");
    Serial.println("#        answering as a W5500. Suspect swapped MISO/MOSI,");
    Serial.println("#        a wrong CS pin, or SPI mode/wiring noise.");
  }
  Serial.println("# ---- W5500 probe done ----");
}

// ---------------------------------------------------- Ethernet bring-up
//
// Addressing: DHCP is tried first because the bench LAN has a server, then
// it falls back to a fixed address so the car is still reachable on a link
// with no DHCP at all -- which is what you get when the Bullet is running as
// a bare wireless bridge with nothing else on the segment. A robot that is
// only reachable when a router happens to be up is not reachable.
//
// The MAC comes from the chip's own eFuse ETH address (ESP_MAC_ETH), not the
// WiFi one. They differ by design; using the WiFi MAC here would collide the
// moment somebody brings the radio up with 'A 1' onto the same segment.

// Arduino's EthernetServer is abstract on ESP32 and will not compile as-is:
// the core's Server base declares `virtual void begin(uint16_t port = 0) = 0`
// while the Ethernet library only ever defined the no-argument begin(). This
// closes that one gap and changes no behaviour -- the port is already fixed by
// the constructor, so the argument is correctly ignored. The default argument
// lives here rather than in a `using` declaration because inheriting both
// overloads would make a bare begin() ambiguous.
class EthServer : public EthernetServer {
public:
  explicit EthServer(uint16_t port) : EthernetServer(port) {}
  void begin(uint16_t port = 0) override { (void)port; EthernetServer::begin(); }
};

static EthServer netCmdServer(NET_CMD_PORT);
static EthServer netOtaServer(NET_OTA_PORT);

static uint8_t   netMac[6];
static bool      netStarted   = false;
static bool      netGotDhcp   = false;
static uint32_t  netOtaCount  = 0;
static IPAddress netFallbackIp (192, 168,  1,  77);
static IPAddress netFallbackGw (192, 168,  1,   1);
static IPAddress netFallbackSn (255, 255, 255,   0);

static void ethBegin() {
  // Share the bus the Y probe already set up, at the same pins.
  if (!ethSpiBegun) {
    pinMode(PIN_ETH_CS, OUTPUT);
    digitalWrite(PIN_ETH_CS, HIGH);
    SPI.begin(PIN_ETH_SCLK, PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_CS);
    ethSpiBegun = true;
  }

  esp_read_mac(netMac, ESP_MAC_ETH);
  Ethernet.init(PIN_ETH_CS);

  Serial.printf("eth: MAC %02X:%02X:%02X:%02X:%02X:%02X, trying DHCP...\n",
                netMac[0], netMac[1], netMac[2],
                netMac[3], netMac[4], netMac[5]);

  // Short timeouts: this runs in setup() and must not hold the car hostage.
  netGotDhcp = (Ethernet.begin(netMac, 6000, 3000) == 1);
  if (!netGotDhcp) {
    Ethernet.begin(netMac, netFallbackIp, netFallbackGw,
                   netFallbackGw, netFallbackSn);
  }

  // hardwareStatus() answering W5500 is also the proof that the SPI bus is
  // sound at the Ethernet library's clock (14 MHz), which is faster than the
  // 8 MHz the Y probe uses. If this says NO HARDWARE but Y says PRESENT, the
  // wiring is marginal at 14 MHz -- shorten the leads or drop the clock.
  EthernetHardwareStatus hw = Ethernet.hardwareStatus();
  if (hw == EthernetNoHardware) {
    Serial.println("# eth: NO HARDWARE at 14 MHz -- run Y; if Y says PRESENT");
    Serial.println("#      the leads are too long/noisy for the library clock");
    netUp = false;
    return;
  }

  netStarted = true;
  netCmdServer.begin();
  netOtaServer.begin();

  IPAddress ip = Ethernet.localIP();
  bool link = (Ethernet.linkStatus() != LinkOFF);
  netUp = link && (uint32_t)ip != 0;

  Serial.printf("eth: W5500 up, %s %d.%d.%d.%d, link %s\n",
                netGotDhcp ? "DHCP" : "STATIC (no DHCP answer)",
                ip[0], ip[1], ip[2], ip[3],
                link ? "UP" : "DOWN (no cable)");
  Serial.printf("eth: console on tcp/%u, firmware push on tcp/%u\n",
                NET_CMD_PORT, NET_OTA_PORT);
}

// ------------------------------------------------------------- OTA over TCP
//
// Protocol, deliberately trivial so the host side is 40 lines of Python and
// needs no libraries:
//
//     -> "OTA <bytes> [md5hex]\n"      then <bytes> raw bytes of firmware
//     <- "OK <bytes>\n"  or  "ERR <reason>\n"
//
// then the board reboots into the new image. There is no authentication: this
// is a private wireless segment, and adding a shared secret here would be
// security theatre while the Bullet itself still ships on factory ubnt/ubnt.
// Fix the radio's credentials -- that is the real door.
//
// Safety: the motors are stopped and the control loop is disengaged BEFORE a
// single byte is written to flash. An update must never be able to leave the
// car driving, and the reboot at the end would otherwise resume from whatever
// state the controller was in.
static void netOtaService() {
  EthernetClient c = netOtaServer.accept();
  if (!c) return;

  char     hdr[96];
  size_t   hl = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 5000) {
    if (!c.connected() && !c.available()) break;
    int ch = c.read();
    if (ch < 0) { delay(1); continue; }
    if (ch == '\n') break;
    if (hl < sizeof(hdr) - 1) hdr[hl++] = (char)ch;
  }
  hdr[hl] = 0;

  unsigned long size = 0;
  char md5[40] = {0};
  if (sscanf(hdr, "OTA %lu %39s", &size, md5) < 1 || size == 0) {
    c.println("ERR bad header, expected: OTA <bytes> [md5hex]");
    c.stop();
    Serial.printf("# OTA rejected, bad header: \"%s\"\n", hdr);
    return;
  }

  Serial.printf("# OTA starting, %lu bytes%s -- stopping motors\n",
                size, md5[0] ? " (md5 checked)" : "");
  stopAll();
  ctrlDisengage();

  if (!Update.begin(size)) {
    c.printf("ERR Update.begin failed: %s\n", Update.errorString());
    c.stop();
    Serial.printf("# OTA failed at begin: %s\n", Update.errorString());
    return;
  }
  if (md5[0]) Update.setMD5(md5);

  uint8_t  buf[1024];
  uint32_t got = 0, lastByte = millis(), lastLog = 0;
  while (got < size) {
    uint32_t want = size - got;
    if (want > sizeof(buf)) want = sizeof(buf);
    int n = c.read(buf, want);
    if (n > 0) {
      if (Update.write(buf, n) != (size_t)n) {
        c.printf("ERR write failed at %lu: %s\n",
                 (unsigned long)got, Update.errorString());
        Update.abort();
        c.stop();
        Serial.printf("# OTA write failed at %lu\n", (unsigned long)got);
        return;
      }
      got += n;
      lastByte = millis();
      if (millis() - lastLog > 500) {
        lastLog = millis();
        Serial.printf("# OTA %lu/%lu (%lu%%)\n", (unsigned long)got, size,
                      (unsigned long)(100UL * got / size));
      }
    } else {
      if (millis() - lastByte > 10000) {
        c.println("ERR timed out mid-transfer");
        Update.abort();
        c.stop();
        Serial.println("# OTA timed out mid-transfer");
        return;
      }
      delay(1);          // also yields to the task watchdog
    }
  }

  if (!Update.end(true)) {
    c.printf("ERR finalise failed: %s\n", Update.errorString());
    c.stop();
    Serial.printf("# OTA finalise failed: %s\n", Update.errorString());
    return;
  }

  netOtaCount++;
  c.printf("OK %lu\n", (unsigned long)got);
  c.flush();
  c.stop();
  Serial.printf("# OTA complete, %lu bytes -- rebooting into new image\n",
                (unsigned long)got);
  netFlush();
  delay(250);
  ESP.restart();
}

// ------------------------------------------------------------- Ethernet poll
//
// Called once per loop(). Everything here is non-blocking except an OTA,
// which deliberately owns the CPU for its couple of seconds with the motors
// already stopped.
static void ethPoll() {
  if (!netStarted) return;

  // Track carrier so a yanked cable is visible rather than mysterious.
  static bool lastLink = true;
  bool link = (Ethernet.linkStatus() != LinkOFF);
  if (link != lastLink) {
    lastLink = link;
    Serial.printf("# eth: link %s\n", link ? "UP" : "DOWN");
  }
  netUp = link;

  // One console client at a time. A new connection displaces a stale one --
  // after a laptop sleeps, the old socket can sit half-open for minutes and
  // you would otherwise be locked out of your own car.
  if (!netCmd || !netCmd.connected()) {
    if (netCmd) {
      netCmd.stop();
      // A dropped console must not leave the car driving. The CMD_TIMEOUT
      // watchdog in loop() would catch it too, but that is up to 400 ms of
      // travel; this is immediate.
      ctrlDisengage();
      Serial.println("# eth: console client gone, control disengaged");
    }
    EthernetClient nc = netCmdServer.accept();
    if (nc) {
      netCmd = nc;
      netTxLen = 0;
      Serial.println("# eth: console client connected");
      printHelp();
    }
  } else {
    EthernetClient extra = netCmdServer.accept();
    if (extra) {
      extra.println("# busy: another console is connected");
      extra.stop();
    }
  }

  netOtaService();

  if (netGotDhcp) Ethernet.maintain();

  if (netTxLen && (millis() - netLastFlush) >= NET_FLUSH_MS) netFlush();
}

static void ethStatus() {
  IPAddress ip = Ethernet.localIP();
  Serial.printf("# eth: %s, %s, ip %d.%d.%d.%d, link %s\n",
                netStarted ? "started" : "NOT started",
                netGotDhcp ? "DHCP" : "static",
                ip[0], ip[1], ip[2], ip[3],
                (Ethernet.linkStatus() != LinkOFF) ? "UP" : "DOWN");
  Serial.printf("# eth: MAC %02X:%02X:%02X:%02X:%02X:%02X, console %s, ports %u/%u\n",
                netMac[0], netMac[1], netMac[2], netMac[3], netMac[4], netMac[5],
                (netCmd && netCmd.connected()) ? "CONNECTED" : "idle",
                NET_CMD_PORT, NET_OTA_PORT);
  Serial.printf("# eth: tx %lu bytes in %lu packets (%.1f B/pkt), %lu OTA(s) served\n",
                (unsigned long)netTxBytes, (unsigned long)netTxPackets,
                netTxPackets ? (float)netTxBytes / netTxPackets : 0.0f,
                (unsigned long)netOtaCount);
  Serial.printf("# eth: running image from partition '%s'\n",
                esp_ota_get_running_partition()->label);
}

// ------------------------------------------------------------- LED updater
//
// Called every loop(). Picks a pattern from the worst thing currently true,
// so the LED always shows the most important state rather than the newest.
static void ledUpdate() {
  uint32_t nowMs = millis();

  // --- work out what the base is doing ------------------------------------
  bool encBad = false;
  for (int i = 0; i < 2; i++) {
    Encoder &e = encoders[i];
    if (!e.online) { encBad = true; break; }
    // MD must be set; ML (too weak) and a pegged AGC both mean the reading is
    // not to be trusted even though the part is answering.
    if (e.status >= 0 && !(e.status & 0x20)) { encBad = true; break; }
    if (e.status >= 0 && (e.status & 0x10))  { encBad = true; break; }
    if (e.agc == 0 || e.agc >= 128)          { encBad = true; break; }
  }

  // A wheel being floored while its error refuses to close. This is what a
  // motor that is not connected to the encoder being corrected looks like.
  for (int i = 0; i < 2; i++) {
    bool bad = (ctrlMode != CTRL_OFF) &&
               (abs(lastDuty[i]) > STALL_DUTY) &&
               (fabsf(posErrM[i]) > STALL_ERR_M);
    if (!bad) { stallSince[i] = 0; continue; }
    if (stallSince[i] == 0) stallSince[i] = nowMs;
    else if (nowMs - stallSince[i] > STALL_MS) stallLatched = true;
  }
  if (ctrlMode == CTRL_OFF) stallLatched = false;   // clears when you let go

  // --- pick the pattern, worst first --------------------------------------
  if (nowMs < ledIdentifyUntil)   { ledMask = LED_IDENTIFY;    ledWhat = "identify"; }
  else if (encBad)                { ledMask = LED_FAULT_ENC;   ledWhat = "encoder fault"; }
  else if (stallLatched)          { ledMask = LED_FAULT_STALL; ledWhat = "wheel stalled"; }
  else if (goSide >= 0)           { ledMask = LED_ROTATE;      ledWhat = "rotating"; }
  else if (ctrlMode == CTRL_HOLD) { ledMask = LED_HOLD;        ledWhat = "holding"; }
  else if (ctrlMode == CTRL_VEL)  { ledMask = LED_SOLID;       ledWhat = "driving"; }
  else                            { ledMask = LED_HEARTBEAT;   ledWhat = "idle"; }

  // --- drive the pin ------------------------------------------------------
  uint8_t slot = (uint8_t)((nowMs % 1000) / 50);    // 20 slots of 50 ms
  ledSet((ledMask >> slot) & 1);
}

void loop() {
  ledUpdate();
  ethPoll();

  if (radioUp) ws.cleanupClients();

  if (runSide >= 0 && (int32_t)(millis() - runUntil) >= 0) runStop("deadline");

  if (goResultPending) { goResultPending = false; Serial.println(goResultLine); }
  if (scanDone) { scanDone = false; Serial.println(scanBuf); }

  // Watchdog. A closed-loop velocity command has to be refreshed or the car
  // stops -- a crashed controller must not be able to drive it away. Position
  // hold is a deliberate "stay put" so it is exempt, but it still expires.
  if (ctrlMode == CTRL_VEL || driving) {
    if (millis() - lastCmdMillis > CMD_TIMEOUT_MS) {
      Serial.println("# command timeout, stopping");
      ctrlDisengage();
    }
  } else if (ctrlMode == CTRL_HOLD &&
             millis() - lastCmdMillis > HOLD_MAX_MS) {
    Serial.println("# hold expired, releasing");
    ctrlDisengage();
  }

  static char cmd[32];
  static uint8_t cmdLen = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmd[cmdLen] = 0;
      if (cmdLen) handleCommand(cmd);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmd) - 1) {
      cmd[cmdLen++] = (char)c;
    }
  }

  // Magnet health for encoder_tune.py, 20 Hz. Emitted from the cached values
  // the control task collects at 5 Hz; the line rate is what makes the
  // dashboard feel live, and the register read is what used to cost latency.
  static uint32_t lastMagnet = 0;
  if (millis() - lastMagnet >= 50) {
    lastMagnet = millis();
    Serial.printf("T,%d,%d,%d,%d,%d,%d,%d,%d\n",
                  encoders[0].online ? (int)encoders[0].lastRaw : -1,
                  encoders[0].agc, encoders[0].magnitude, encoders[0].status,
                  encoders[1].online ? (int)encoders[1].lastRaw : -1,
                  encoders[1].agc, encoders[1].magnitude, encoders[1].status);
  }

  // Legacy line, unchanged, in the RAW board frame. carbot/driver.py,
  // encoder_gui.py and wheel_odom_node.py all parse this.
  // ODO,<leftMetres>,<rightMetres>,<leftOk>,<rightOk>,<leftCounts>,<rightCounts>
  static uint32_t lastOdoLine = 0;
  if (millis() - lastOdoLine >= 100) {
    lastOdoLine = millis();
    Serial.printf("ODO,%.4f,%.4f,%d,%d,%lld,%lld\n",
                  metresOf(encoders[0]), metresOf(encoders[1]),
                  encoders[0].online ? 1 : 0, encoders[1].online ? 1 : 0,
                  (long long)encoders[0].counts, (long long)encoders[1].counts);
  }

  // Feedback line: everything the controller sees, in the forward-positive
  // frame, in SI units. This is the one to use for wheel odometry.
  //
  // FB,<ms>,<Lm>,<Rm>,<Lmps>,<Rmps>,<LspMps>,<RspMps>,<LerrMm>,<RerrMm>,
  //    <Lduty>,<Rduty>,<skewMm>,<x>,<y>,<thDeg>,<refScale>,<mode>,<track>
  static uint32_t lastFb = 0;
  if (fbPeriodMs && millis() - lastFb >= fbPeriodMs) {
    lastFb = millis();
    LOCK();
    float lm = (float)fwdCountsOf(0) / COUNTS_PER_METRE;
    float rm = (float)fwdCountsOf(1) / COUNTS_PER_METRE;
    float vl = fwdVel[0], vr = fwdVel[1];
    float el = posErrM[0] * 1000.0f, er = posErrM[1] * 1000.0f;
    int   dl = lastDuty[0], dr = lastDuty[1];
    float spl = cmdVel[0], spr = cmdVel[1];
    float rs = refScale;
    double px = poseX, py = poseY, pth = poseTh;
    int md = (int)ctrlMode;
    UNLOCK();
    Serial.printf("FB,%lu,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%d,%d,"
                  "%.2f,%.4f,%.4f,%.2f,%.2f,%d,%.3f\n",
                  (unsigned long)millis(), lm, rm, vl, vr, spl, spr, el, er,
                  dl, dr, (lm - rm) * 1000.0f, px, py, pth * 180.0 / M_PI,
                  rs, md, TRACK_WIDTH_M);
  }

  // MAG,<x>,<y>,<z>,<headingDeg>,<field_gauss>,<reads>,<errors>
  // Raw counts plus the flat heading. There is no tilt compensation and no
  // hard/soft-iron calibration here on purpose: this line is for finding out
  // whether the reading is usable on this chassis at all.
  static uint32_t lastMag = 0;
  if (magPresent && millis() - lastMag >= 50) {
    lastMag = millis();
    float mag = sqrtf((float)magRaw[0]*magRaw[0] + (float)magRaw[1]*magRaw[1]
                      + (float)magRaw[2]*magRaw[2]) / QMC_LSB_PER_GAUSS;
    Serial.printf("MAG,%d,%d,%d,%.2f,%.4f,%lu,%lu\n",
                  magRaw[0], magRaw[1], magRaw[2], magHeadingDeg, mag,
                  (unsigned long)magReads, (unsigned long)magErrors);
  }

  // IMU,<fresh>,<gyroZ_dps>,<bias_dps>,<biasN>,<fusedTh_deg>,<wheelTh_deg>,
  //     <gap_deg>,<packets>,<badsum>
  // Emitted whether or not a sensor is attached, so a host can tell "no IMU"
  // apart from "IMU present but silent" without asking. The gap between the
  // fused and wheels-only headings is the slip / track-width error, and it is
  // the number worth watching while driving.
  static uint32_t lastImu = 0;
  if (millis() - lastImu >= 100) {
    lastImu = millis();
    Serial.printf("IMU,%d,%.4f,%.5f,%lu,%.3f,%.3f,%.3f,%lu,%lu\n",
                  imuFresh() ? 1 : 0, imuGyroZ, imuBiasZ,
                  (unsigned long)imuBiasN,
                  poseTh * 180.0 / M_PI, poseThWheel * 180.0 / M_PI,
                  (poseTh - poseThWheel) * 180.0 / M_PI,
                  (unsigned long)imuPackets, (unsigned long)imuBadSum);
  }

  // Push odometry to the phone GUI. This block went missing when the file was
  // rewritten for the closed loop, so the page was connecting and then sitting
  // at "--" forever. It now carries the wheel speeds the page displays too.
  static uint32_t lastPush = 0;
  if (radioUp && millis() - lastPush >= 100) {
    lastPush = millis();
    LOCK();
    float lm = (float)fwdCountsOf(0) / COUNTS_PER_METRE;
    float rm = (float)fwdCountsOf(1) / COUNTS_PER_METRE;
    float vl = fwdVel[0], vr = fwdVel[1];
    UNLOCK();
    char msg[192];
    snprintf(msg, sizeof(msg),
             "{\"type\":\"odo\",\"l\":%.4f,\"r\":%.4f,\"vl\":%.3f,"
             "\"vr\":%.3f,\"lok\":%s,\"rok\":%s}",
             lm, rm, vl, vr,
             encoders[0].online ? "true" : "false",
             encoders[1].online ? "true" : "false");
    ws.textAll(msg);
  }

  static uint32_t lastStationPrint = 0;
  if (radioUp && millis() - lastStationPrint > 5000) {
    lastStationPrint = millis();
    Serial.printf("[%lu] WiFi %s  IP: %s  RSSI: %d dBm  L=%.3fm%s R=%.3fm%s\n",
                  millis() / 1000,
                  WiFi.isConnected() ? "OK" : "DISCONNECTED",
                  WiFi.localIP().toString().c_str(),
                  WiFi.RSSI(),
                  metresOf(encoders[0]), encoders[0].online ? "" : "(off)",
                  metresOf(encoders[1]), encoders[1].online ? "" : "(off)");
  }
}
