// TWANG Next - 915 MHz receiver for the wireless controller (RFM69HCW on the main unit)
#pragma once
#include <Arduino.h>
#include "../core/tw_input.h"

namespace radio_rx {

struct Status {
  bool present;      // radio module found
  bool connected;    // a controller has been heard in the last 100 ms
  uint8_t channel;   // index into the channel list
  uint32_t freqKhz;
  int8_t rssi;       // dBm, last packet
  uint8_t loss;      // percent over the last second
  uint16_t battMv;   // controller battery (0 = unknown)
  bool lowBatt;
  uint16_t packetsPerSec;
  uint32_t lastSeenMs;  // millis() of the last packet (0 = never)
  bool imuFault;
};

// Starts the radio task. Returns false if no RFM69 answers on SPI.
bool begin(uint8_t netId, const uint32_t channelsKhz[3]);
// Apply a new network ID or channel list (takes effect within a few ms).
void configure(uint8_t netId, const uint32_t channelsKhz[3]);
// Latest controller motion: accel from the newest packet, gyro peaks merged since the
// previous call. Returns false when there's nothing fresh (no controller, or stale).
bool take(tw::ImuSample& out);
// Ask the controller to buzz (sent with the next downlink, within ~25 ms).
void haptic(uint16_t ms);
// Tell the controller to go to sleep now.
void sleepController();
Status status();

}  // namespace radio_rx
