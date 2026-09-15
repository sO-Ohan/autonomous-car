// c3-spin-test -- bring-up spin test for two BTS7960 half-bridges on an
// ESP32-C3 Supermini. R_EN/L_EN are strapped to VCC (always enabled), so
// PWM duty at zero is what keeps the motors still.
//
//   BTS #1 RPWM -> GPIO 6      BTS #2 RPWM -> GPIO 4
//   BTS #1 LPWM -> GPIO 7      BTS #2 LPWM -> GPIO 5

#include <Arduino.h>

struct Motor {
  const char *name;
  uint8_t rpwmPin, lpwmPin;
  uint8_t rpwmCh, lpwmCh;
};

static Motor motors[] = {
  {"BTS#1", 6, 7, 0, 1},
  {"BTS#2", 4, 5, 2, 3},
};

static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;      // 0-255
static const int TEST_DUTY = 120;   // ~47%, gentle bring-up level
static const int RUN_MS    = 800;

void stopAll() {
  for (auto &m : motors) {
    ledcWrite(m.rpwmCh, 0);
    ledcWrite(m.lpwmCh, 0);
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nc3-spin-test booting");

  for (auto &m : motors) {
    ledcSetup(m.rpwmCh, PWM_FREQ, PWM_RES);
    ledcSetup(m.lpwmCh, PWM_FREQ, PWM_RES);
    ledcAttachPin(m.rpwmPin, m.rpwmCh);
    ledcAttachPin(m.lpwmPin, m.lpwmCh);
    ledcWrite(m.rpwmCh, 0);
    ledcWrite(m.lpwmCh, 0);
  }
  stopAll();
  Serial.println("ready");
}

void spin(Motor &m, bool forward) {
  uint8_t driveCh = forward ? m.rpwmCh : m.lpwmCh;
  uint8_t idleCh  = forward ? m.lpwmCh : m.rpwmCh;
  Serial.printf("%s %s duty=%d\n", m.name, forward ? "FWD" : "REV", TEST_DUTY);
  ledcWrite(idleCh, 0);
  ledcWrite(driveCh, TEST_DUTY);
  delay(RUN_MS);
  ledcWrite(driveCh, 0);
  delay(300);
}

void loop() {
  static bool done = false;
  if (!done) {
    for (auto &m : motors) {
      spin(m, true);
      spin(m, false);
    }
    stopAll();
    Serial.println("spin test complete, motors stopped");
    done = true;
  }
  delay(1000);
}
