// TWANG Next - where player input comes from: the wireless controller or a wired sensor.
#pragma once
#include <Arduino.h>
#include "../core/tw_input.h"

namespace controls {

enum Source : uint8_t { SRC_NONE = 0, SRC_WIRED = 1, SRC_WIRELESS = 2 };

void begin();
// Once per 60 Hz tick. The wireless controller wins whenever it's fresh; otherwise a
// wired sensor on I2C is used; with neither, the player stands still.
tw::Input read(const tw::InputConfig& cfg);
// Capture the current resting position as "level" (blocks ~0.4 s).
bool calibrate(tw::InputConfig& cfg);

Source source();
const char* sourceName();
const char* wiredName();  // "LSM6DSOX", "MPU6050" or "none"
uint8_t wiredAddress();
float lastTilt();
uint16_t lastWobble();
const tw::ImuSample& lastSample();

}  // namespace controls
