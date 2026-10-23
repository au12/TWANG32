/*
  TWANG Next - wireless controller link (shared by the main unit and the controller).

  KEEP IN SYNC: this file is copied into controller/TWANGController/. Edit it here,
  then run tools/sync-link.sh. tests/run.sh fails if the copies differ.

  Portable C++ (no Arduino), so the protocol logic is unit tested natively.

  Radio: RFM69HCW, GFSK 250 kbps, 125 kHz deviation, fixed frequency in 902-928 MHz
  at -2 dBm (FCC Part 15.249 low-power, no hopping required).

  The controller streams its full state ~400 times a second. Nothing on the uplink
  is acknowledged: a lost packet is simply replaced 2.5 ms later, and the gyro peak
  is held for 40 ms so a twang survives several lost packets. Every 10th packet
  carries LISTEN; the main unit answers it with a downlink (channel changes,
  haptics) while the controller listens for a few milliseconds.
*/
#pragma once
#include <stdint.h>
#include <string.h>

namespace twlink {

constexpr uint8_t VERSION = 1;
constexpr uint8_t TYPE_UP = 0x30 | VERSION;
constexpr uint8_t TYPE_DOWN = 0x50 | VERSION;

constexpr float BITRATE_KBPS = 250.0f;
constexpr float FREQ_DEV_KHZ = 125.0f;
constexpr float RX_BW_KHZ = 500.0f;
constexpr int8_t TX_POWER_DBM = -2;   // RFM69HCW minimum; keeps us inside Part 15.249 limits
constexpr uint8_t PREAMBLE_BITS = 32;

constexpr int NUM_CHANNELS = 3;
// Defaults, in kHz: clear of Z-Wave (908.4/916), Z-Wave Long Range (912/920) and the band edges.
constexpr uint32_t DEFAULT_CHANNELS_KHZ[NUM_CHANNELS] = {925000, 922600, 926600};
constexpr uint32_t MIN_KHZ = 921000;  // stay above Z-Wave LR at 920 MHz
constexpr uint32_t MAX_KHZ = 927400;  // keep the signal inside the 928 MHz band edge

constexpr uint32_t UP_PERIOD_US = 2500;       // 400 packets a second
constexpr uint8_t LISTEN_EVERY = 10;          // every 10th packet asks for a downlink (40 Hz)
constexpr uint32_t LISTEN_WINDOW_US = 3500;   // how long the controller listens
constexpr uint32_t GYRO_HOLD_MS = 40;         // gyro peak held this long across packets
constexpr uint32_t STALE_MS = 100;            // main unit treats the controller as gone after this

// The sync word doubles as the pairing key: a controller only talks to a main
// unit with the same network ID. RFM69 sync bytes must not be 0x00.
inline void syncWord(uint8_t netId, uint8_t out[4]) {
  out[0] = 0x2D;
  out[1] = 0xD4;
  out[2] = 0x54;  // 'T'
  out[3] = netId ? netId : 1;
}

// ------------------------------------------------------------------ packets
enum UpFlags : uint8_t {
  UF_LISTEN = 0x01,    // I will listen for a downlink right after this packet
  UF_LOW_BATT = 0x02,
  UF_JUST_WOKE = 0x04, // first second after power-on or wake from sleep
  UF_IMU_FAULT = 0x08, // sensor not answering: values are zero
};

struct Up {
  uint8_t seq;
  uint8_t flags;
  int16_t accelMg[3];     // median-filtered acceleration, milli-g
  uint16_t gyroPeak[3];   // per-axis |angular rate| peak over the last GYRO_HOLD_MS, deg/s
  uint8_t batt20mV;       // battery voltage / 20 mV (0 = unknown)
  uint8_t dnAck;          // seq of the last downlink received
};
constexpr int UP_LEN = 17;

enum DownFlags : uint8_t {
  DF_SLEEP = 0x01,  // go to sleep now (e.g. the main unit is shutting down)
};

struct Down {
  uint8_t seq;
  uint8_t channel;   // channel index the controller should be on (0..NUM_CHANNELS-1)
  uint8_t haptic4ms; // buzz for this many x 4 ms (0 = none)
  uint8_t flags;
  int8_t rssi;       // how strongly the main unit hears the controller (dBm)
  uint8_t loss;      // uplink packet loss over the last second, percent
};
constexpr int DOWN_LEN = 7;

inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

inline int encodeUp(const Up& u, uint8_t* b) {
  b[0] = TYPE_UP;
  b[1] = u.seq;
  b[2] = u.flags;
  for (int i = 0; i < 3; i++) put16(b + 3 + i * 2, (uint16_t)u.accelMg[i]);
  for (int i = 0; i < 3; i++) put16(b + 9 + i * 2, u.gyroPeak[i]);
  b[15] = u.batt20mV;
  b[16] = u.dnAck;
  return UP_LEN;
}
inline bool decodeUp(const uint8_t* b, int len, Up& u) {
  if (len != UP_LEN || b[0] != TYPE_UP) return false;
  u.seq = b[1];
  u.flags = b[2];
  for (int i = 0; i < 3; i++) u.accelMg[i] = (int16_t)get16(b + 3 + i * 2);
  for (int i = 0; i < 3; i++) u.gyroPeak[i] = get16(b + 9 + i * 2);
  u.batt20mV = b[15];
  u.dnAck = b[16];
  return true;
}
inline int encodeDown(const Down& d, uint8_t* b) {
  b[0] = TYPE_DOWN;
  b[1] = d.seq;
  b[2] = d.channel;
  b[3] = d.haptic4ms;
  b[4] = d.flags;
  b[5] = (uint8_t)d.rssi;
  b[6] = d.loss;
  return DOWN_LEN;
}
inline bool decodeDown(const uint8_t* b, int len, Down& d) {
  if (len != DOWN_LEN || b[0] != TYPE_DOWN) return false;
  d.seq = b[1];
  d.channel = b[2];
  d.haptic4ms = b[3];
  d.flags = b[4];
  d.rssi = (int8_t)b[5];
  d.loss = b[6];
  return d.channel < NUM_CHANNELS;
}

// ------------------------------------------------------------------ main unit: link quality and channel choice
// Counts uplink packets by sequence number and decides when to move channels.
class Receiver {
 public:
  uint8_t channel = 0;

  void reset(uint32_t nowMs) {
    channel = 0;
    lastRx_ = 0;
    haveSeq_ = false;
    winStart_ = nowMs;
    winGot_ = winExp_ = 0;
    loss_ = 0;
    badSince_ = 0;
    dwellStart_ = nowMs;
    everHeard_ = false;
  }

  // Call for every valid uplink packet.
  void onPacket(uint8_t seq, uint32_t nowMs) {
    if (haveSeq_) {
      uint8_t gap = (uint8_t)(seq - lastSeq_);
      // a big jump means the controller rebooted: don't count it as loss
      winExp_ += (gap == 0 || gap > 60) ? 1 : gap;
    } else {
      winExp_ += 1;
    }
    winGot_++;
    lastSeq_ = seq;
    haveSeq_ = true;
    lastRx_ = nowMs;
    everHeard_ = true;
    dwellStart_ = nowMs;
  }

  bool connected(uint32_t nowMs) const { return everHeard_ && nowMs - lastRx_ < STALE_MS; }
  uint8_t lossPercent() const { return loss_; }
  uint32_t lastRx() const { return lastRx_; }

  // Call regularly (every few ms). Returns true when the channel changed.
  // switchNow: set when we want the controller to follow via the next downlink.
  bool poll(uint32_t nowMs) {
    if (nowMs - winStart_ >= 1000) {
      // a silent second while connected is 100% loss; a silent second while disconnected is just silence
      loss_ = winExp_ ? (uint8_t)(100 - (winGot_ * 100) / winExp_) : (connected(nowMs) ? 0 : 100);
      winStart_ = nowMs;
      winGot_ = winExp_ = 0;
      if (loss_ >= BAD_LOSS && everHeard_ && connected(nowMs)) {
        if (!badSince_) badSince_ = nowMs;
      } else {
        badSince_ = 0;
      }
    }
    // a consistently lossy channel: move, and tell the controller in the next downlink
    if (badSince_ && nowMs - badSince_ >= BAD_HOLD_MS) {
      badSince_ = 0;
      next(nowMs);
      return true;
    }
    // heard nothing for a while: the controller may be hunting; dwell on each channel in turn
    if (!connected(nowMs) && nowMs - dwellStart_ >= DWELL_MS) {
      next(nowMs);
      return true;
    }
    return false;
  }

  static constexpr uint8_t BAD_LOSS = 30;      // percent
  static constexpr uint32_t BAD_HOLD_MS = 2000;
  static constexpr uint32_t DWELL_MS = 1500;

 private:
  void next(uint32_t nowMs) {
    channel = (uint8_t)((channel + 1) % NUM_CHANNELS);
    dwellStart_ = nowMs;
  }
  uint32_t lastRx_ = 0, winStart_ = 0, badSince_ = 0, dwellStart_ = 0;
  uint32_t winGot_ = 0, winExp_ = 0;
  uint8_t lastSeq_ = 0, loss_ = 0;
  bool haveSeq_ = false, everHeard_ = false;
};

// ------------------------------------------------------------------ controller: follow the main unit
// Stays on the commanded channel while downlinks arrive; hunts across channels when they stop.
class Hunter {
 public:
  uint8_t channel = 0;

  void reset(uint32_t nowMs) {
    channel = 0;
    lastDown_ = nowMs;
    hopAt_ = 0;
    linked_ = false;
  }
  void onDownlink(const Down& d, uint32_t nowMs) {
    lastDown_ = nowMs;
    hopAt_ = 0;
    linked_ = true;
    channel = d.channel;
  }
  bool linked(uint32_t nowMs) const { return linked_ && nowMs - lastDown_ < LOST_MS; }

  // Call every packet. Returns true when the controller should retune.
  bool poll(uint32_t nowMs) {
    if (nowMs - lastDown_ < LOST_MS) return false;
    if (!hopAt_) hopAt_ = nowMs;
    if (nowMs - hopAt_ >= HOP_MS) {
      channel = (uint8_t)((channel + 1) % NUM_CHANNELS);
      hopAt_ = nowMs;
      return true;
    }
    return false;
  }

  static constexpr uint32_t LOST_MS = 300;  // ~12 missed downlinks
  static constexpr uint32_t HOP_MS = 120;   // ~5 listen windows per channel while hunting

 private:
  uint32_t lastDown_ = 0, hopAt_ = 0;
  bool linked_ = false;
};

inline uint32_t clampChannelKhz(uint32_t khz) {
  return khz < MIN_KHZ ? MIN_KHZ : (khz > MAX_KHZ ? MAX_KHZ : khz);
}

}  // namespace twlink
