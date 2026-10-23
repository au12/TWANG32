#include "controls.h"
#include <Wire.h>
#include "../../config.h"
#include "../link/twang_imu.h"
#include "radio_rx.h"

namespace controls {

static TwangImu s_imu;
static tw::InputProcessor s_proc;
static Source s_src = SRC_NONE;
static uint32_t s_probeAt = 0;

void begin() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(5);
  if (s_imu.begin(Wire)) Serial.printf("Wired motion sensor: %s at 0x%02X\n", s_imu.name(), s_imu.address());
  else Serial.println("No wired motion sensor (fine if you use the wireless controller)");
}

static bool readWired(tw::ImuSample& s) {
  int16_t a[3], g[3];
  if (!s_imu.present() || !s_imu.read(a, g)) return false;
  for (int i = 0; i < 3; i++) {
    s.accelMg[i] = a[i];
    s.gyroDps[i] = (uint16_t)abs(g[i]);
  }
  return true;
}

tw::Input read(const tw::InputConfig& cfg) {
  tw::ImuSample s;
  bool fresh = false;
  if (radio_rx::take(s)) {
    s_src = SRC_WIRELESS;
    fresh = true;
  } else if (readWired(s)) {
    s_src = SRC_WIRED;
    fresh = true;
  } else {
    s_src = SRC_NONE;
    // sensor unplugged or never fitted: look for one every few seconds (hot-plug)
    uint32_t now = millis();
    if (now - s_probeAt > 3000) {
      s_probeAt = now;
      if (s_imu.begin(Wire)) Serial.printf("Wired motion sensor found: %s\n", s_imu.name());
    }
  }
  if (fresh) s_proc.push(s);
  return s_proc.tick(cfg, fresh);
}

bool calibrate(tw::InputConfig& cfg) {
  tw::Calibrator cal;
  uint32_t until = millis() + 400;
  while ((int32_t)(millis() - until) < 0) {
    tw::ImuSample s;
    if (radio_rx::take(s) || readWired(s)) cal.add(s);
    delay(10);
  }
  return cal.apply(cfg);
}

Source source() { return s_src; }
const char* sourceName() { return s_src == SRC_WIRELESS ? "wireless" : s_src == SRC_WIRED ? "wired" : "none"; }
const char* wiredName() { return s_imu.name(); }
uint8_t wiredAddress() { return s_imu.address(); }
float lastTilt() { return s_proc.lastTilt(); }
uint16_t lastWobble() { return s_proc.lastWobble(); }
const tw::ImuSample& lastSample() { return s_proc.lastSample(); }

}  // namespace controls
