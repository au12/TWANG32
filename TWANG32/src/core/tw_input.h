/*
  TWANG Next - turns raw motion from the spring (wired IMU or wireless controller)
  into game input. Portable, so it is unit tested natively.

  Units
    accel: milli-g. Tilt is reported in the original TWANG's units, about one per
           degree near level (raw / 166 at 16384 counts per g = 98.7 per g).
    gyro:  degrees per second. The original triggered a twang at 30000 counts on an
           MPU6050 left at +-250 deg/s, i.e. about 230 deg/s, which is our default.
*/
#pragma once
#include "tw_base.h"
#include "tw_game.h"

namespace tw {

struct ImuSample {
  int16_t accelMg[3];
  uint16_t gyroDps[3];  // |rate| per axis (wireless: peak over the last 40 ms)
};

struct InputConfig {
  uint8_t axis = 1;           // which accelerometer axis is "tilt"
  bool invert = false;
  uint8_t deadzone = 8;       // tilt units (about degrees)
  uint16_t twangDps = 230;    // gyro rate that counts as a twang
  uint16_t smoothMs = 25;     // tilt smoothing time constant (0 = off)
  bool twangAnyAxis = false;  // false: the original behaviour, only the gyro axis matching `axis`
  int16_t offsetMg[3] = {0, 0, 0};
};

class InputProcessor {
 public:
  static constexpr float TILT_PER_MG = 98.7f / 1000.0f;

  // Feed every sample as it arrives (any rate).
  void push(const ImuSample& s) {
    last_ = s;
    have_ = true;
    for (int i = 0; i < 3; i++) if (s.gyroDps[i] > peak_[i]) peak_[i] = s.gyroDps[i];
  }

  // Once per 60 Hz game tick. `fresh` = the source is alive; when false the
  // player stops (failsafe) instead of continuing with stale input.
  Input tick(const InputConfig& c, bool fresh) {
    Input in;
    int ax = c.axis > 2 ? 1 : c.axis;
    if (!fresh || !have_) {
      smoothed_ = 0;
      for (int i = 0; i < 3; i++) peak_[i] = 0;
      lastTilt_ = 0;
      lastWobble_ = 0;
      return in;
    }
    float raw = (float)(last_.accelMg[ax] - c.offsetMg[ax]) * TILT_PER_MG;
    if (c.smoothMs == 0) {
      smoothed_ = raw;
    } else {
      // one-pole low-pass at the 60 Hz tick: alpha = 1 - e^(-dt/tau)
      float k = (1000.0f / TICK_HZ) / (float)c.smoothMs;
      float alpha = 1.0f - exp2f_(-k * 1.44269504f);
      smoothed_ += (raw - smoothed_) * alpha;
    }
    float t = smoothed_;
    float dz = (float)c.deadzone;
    if (absf(t) < dz) t = 0;
    else t += t > 0 ? -dz : dz;
    // the original moves forward on negative readings unless inverted
    in.tilt = clampf(c.invert ? t : -t, -90.0f, 90.0f);

    uint16_t wob = c.twangAnyAxis ? (uint16_t)maxi(maxi(peak_[0], peak_[1]), peak_[2]) : peak_[ax];
    in.attack = wob >= c.twangDps;
    in.active = in.tilt != 0 || in.attack;
    lastTilt_ = in.tilt;
    lastWobble_ = wob;
    for (int i = 0; i < 3; i++) peak_[i] = 0;
    return in;
  }

  float lastTilt() const { return lastTilt_; }
  uint16_t lastWobble() const { return lastWobble_; }
  const ImuSample& lastSample() const { return last_; }

 private:
  ImuSample last_ = {{0, 0, 0}, {0, 0, 0}};
  bool have_ = false;
  float smoothed_ = 0;
  uint16_t peak_[3] = {0, 0, 0};
  float lastTilt_ = 0;
  uint16_t lastWobble_ = 0;
};

// Calibration: average a few hundred ms of samples taken with the spring at rest.
// The axis carrying gravity is left alone (it isn't a tilt axis).
struct Calibrator {
  int32_t sum[3] = {0, 0, 0};
  int n = 0;
  void add(const ImuSample& s) {
    for (int i = 0; i < 3; i++) sum[i] += s.accelMg[i];
    n++;
  }
  bool apply(InputConfig& c) const {
    if (n < 4) return false;
    for (int i = 0; i < 3; i++) {
      int32_t avg = sum[i] / n;
      c.offsetMg[i] = (int16_t)(absi((int)avg) > 700 ? 0 : avg);
    }
    return true;
  }
};

}  // namespace tw
