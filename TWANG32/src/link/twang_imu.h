/*
  TWANG Next - motion sensor driver for the spring.

  KEEP IN SYNC: copied into controller/TWANGController/. Edit it here, then run
  tools/sync-link.sh.

  Supports the ST LSM6DSOX (Adafruit 4438, recommended) and, as a fallback for
  existing builds, the InvenSense MPU6050 (GY-521). Whichever answers on I2C is used.
  Both are set up the same way: +-2 g accelerometer, +-2000 deg/s gyro, so the
  readings are in real units and a hard flick no longer saturates the gyro.
*/
#pragma once
#include <Arduino.h>
#include <Wire.h>

class TwangImu {
 public:
  enum Kind : uint8_t { NONE = 0, LSM6DSOX = 1, MPU6050 = 2 };

  // Probe the bus. Returns true if a supported sensor was found and configured.
  bool begin(TwoWire& wire) {
    w_ = &wire;
    kind_ = NONE;
    const uint8_t lsmAddrs[2] = {0x6A, 0x6B};
    for (uint8_t a : lsmAddrs) {
      uint8_t who = 0;
      if (readRegs(a, 0x0F, &who, 1) && who == 0x6C) {  // LSM6DSOX (also LSM6DSO)
        addr_ = a;
        kind_ = LSM6DSOX;
        whoami_ = who;
        return configLsm();
      }
    }
    const uint8_t mpuAddrs[2] = {0x68, 0x69};
    for (uint8_t a : mpuAddrs) {
      uint8_t who = 0;
      if (readRegs(a, 0x75, &who, 1) && who != 0x00 && who != 0xFF) {
        addr_ = a;
        kind_ = MPU6050;
        whoami_ = who;
        return configMpu();
      }
    }
    return false;
  }

  // accelMg: milli-g. gyroDps: signed deg/s. Returns false if the sensor stopped answering.
  bool read(int16_t accelMg[3], int16_t gyroDps[3]) {
    uint8_t b[14];
    if (kind_ == LSM6DSOX) {
      if (!readRegs(addr_, 0x22, b, 12)) return fail();
      for (int i = 0; i < 3; i++) {
        int16_t g = (int16_t)(b[i * 2] | (b[i * 2 + 1] << 8));
        int16_t a = (int16_t)(b[6 + i * 2] | (b[6 + i * 2 + 1] << 8));
        gyroDps[i] = (int16_t)(((int32_t)g * 70) / 1000);    // 70 mdps/LSB at +-2000 dps
        accelMg[i] = (int16_t)(((int32_t)a * 61) / 1000);    // 0.061 mg/LSB at +-2 g
      }
    } else if (kind_ == MPU6050) {
      if (!readRegs(addr_, 0x3B, b, 14)) return fail();  // accel, temperature, gyro (big endian)
      for (int i = 0; i < 3; i++) {
        int16_t a = (int16_t)(b[i * 2] << 8 | b[i * 2 + 1]);
        int16_t g = (int16_t)(b[8 + i * 2] << 8 | b[8 + i * 2 + 1]);
        accelMg[i] = (int16_t)(((int32_t)a * 1000) / 16384);  // +-2 g
        gyroDps[i] = (int16_t)(((int32_t)g * 10) / 164);      // 16.4 LSB per deg/s at +-2000
      }
    } else {
      return false;
    }
    failures_ = 0;
    return true;
  }

  // LSM6DSOX only: drop to a ~26 Hz low-power accelerometer and raise INT1 on motion.
  // Used by the controller before deep sleep. thresholdMg is rounded to 31.25 mg steps.
  bool sleepUntilMotion(uint16_t thresholdMg) {
    if (kind_ != LSM6DSOX) return false;
    uint8_t ths = (uint8_t)constrain((thresholdMg + 15) / 31, 1, 63);
    bool ok = writeReg(0x11, 0x00);              // CTRL2_G: gyro off
    ok &= writeReg(0x10, 0x20);                  // CTRL1_XL: 26 Hz, +-2 g
    ok &= writeReg(0x15, 0x10);                  // CTRL6_C: XL_HM_MODE = 1 (low power)
    ok &= writeReg(0x56, 0x01);                  // TAP_CFG0: latch interrupts (LIR)
    ok &= writeReg(0x5C, 0x00);                  // WAKE_UP_DUR
    ok &= writeReg(0x5B, ths);                   // WAKE_UP_THS
    ok &= writeReg(0x58, 0x80);                  // TAP_CFG2: INTERRUPTS_ENABLE
    ok &= writeReg(0x5E, 0x20);                  // MD1_CFG: wake-up on INT1
    delay(80);                                   // let the low-power filter settle
    uint8_t src;
    readRegs(addr_, 0x1B, &src, 1);              // WAKE_UP_SRC: clear any latched event
    return ok;
  }

  Kind kind() const { return kind_; }
  bool present() const { return kind_ != NONE; }
  uint8_t address() const { return addr_; }
  const char* name() const { return kind_ == LSM6DSOX ? "LSM6DSOX" : kind_ == MPU6050 ? "MPU6050" : "none"; }

 private:
  bool configLsm() {
    writeReg(0x12, 0x01);  // CTRL3_C: software reset
    delay(15);
    bool ok = writeReg(0x12, 0x44);  // CTRL3_C: block data update, register auto-increment
    ok &= writeReg(0x5E, 0x00);      // MD1_CFG: no interrupts while awake
    ok &= writeReg(0x58, 0x00);      // TAP_CFG2
    ok &= writeReg(0x15, 0x00);      // CTRL6_C: high-performance accelerometer
    ok &= writeReg(0x10, 0x70);      // CTRL1_XL: 833 Hz, +-2 g
    ok &= writeReg(0x11, 0x7C);      // CTRL2_G: 833 Hz, +-2000 deg/s
    delay(10);
    return ok;
  }
  bool configMpu() {
    bool ok = writeReg(0x6B, 0x00);  // PWR_MGMT_1: wake
    delay(5);
    ok &= writeReg(0x1A, 0x01);      // CONFIG: 184 Hz low-pass
    ok &= writeReg(0x1B, 0x18);      // GYRO_CONFIG: +-2000 deg/s
    ok &= writeReg(0x1C, 0x00);      // ACCEL_CONFIG: +-2 g
    return ok;
  }
  bool fail() {
    if (++failures_ > 30) kind_ = NONE;
    return false;
  }
  bool writeReg(uint8_t reg, uint8_t v) {
    w_->beginTransmission(addr_);
    w_->write(reg);
    w_->write(v);
    return w_->endTransmission(true) == 0;
  }
  bool readRegs(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n) {
    w_->beginTransmission(addr);
    w_->write(reg);
    if (w_->endTransmission(false) != 0) return false;
    if (w_->requestFrom(addr, n) != n) return false;
    for (uint8_t i = 0; i < n; i++) out[i] = (uint8_t)w_->read();
    return true;
  }

  TwoWire* w_ = nullptr;
  Kind kind_ = NONE;
  uint8_t addr_ = 0, whoami_ = 0;
  uint32_t failures_ = 0;
};
