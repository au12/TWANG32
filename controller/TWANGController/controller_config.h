/*
  TWANG Next wireless controller - configuration.
  Board: Seeed Studio XIAO ESP32-C3. Wiring: docs/CONTROLLER.md.
*/
#pragma once

// Must match "Radio network" on the main unit's Setup page. 1-250.
#define NET_ID 42

// ------------------------------------------------------------------ pins (GPIO numbers; XIAO label in brackets)
// Only GPIO0-5 can wake the ESP32-C3 from deep sleep, and GPIO2/8/9 decide the boot
// mode, so the sensor's interrupt is on GPIO3 and nothing that holds a pin low at
// power-up sits on GPIO2.
#define PIN_BATT 2       // [D0] battery voltage through a 2 x 220k divider (Seeed's recommended setup)
#define PIN_IMU_INT 3    // [D1] LSM6DSOX INT1 ("I1"): motion wakes the controller
#define PIN_RADIO_G0 4   // [D2] RFM69 G0 (DIO0)
#define PIN_HAPTIC 5     // [D3] vibration motor driver input (optional)
#define PIN_SDA 6        // [D4]
#define PIN_SCL 7        // [D5]
#define PIN_RADIO_CS 21  // [D6]
#define PIN_RADIO_RST 20 // [D7]
#define PIN_SCK 8        // [D8]
#define PIN_MISO 9       // [D9]
#define PIN_MOSI 10      // [D10]

// ------------------------------------------------------------------ behaviour
#define SLEEP_AFTER_S 180        // no movement for this long: deep sleep until the spring moves
#define WAKE_THRESHOLD_MG 90     // how hard a nudge wakes it (31 mg steps)
#define MOTION_DPS 25            // rotation above this counts as "someone is using it"
#define LOW_BATT_MV 3450         // report low battery below this
#define CUTOFF_MV 3250           // below this, sleep and only check back every 10 minutes
#define BATT_DIVIDER 2.0f        // 220k over 220k
#define HAS_BATTERY_SENSE 1      // set to 0 if you leave the divider out

// Lower CPU clock saves power; 80 MHz has plenty of headroom for 400 packets a second.
#define CPU_MHZ 80
