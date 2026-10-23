# Wireless controller

A battery-powered spring that talks to the main unit on 925 MHz. The game, LEDs and sound stay on the main ESP32; the controller only senses the spring and streams it. Latency from spring to game is about 3–4 ms plus the game's normal 60 Hz tick, which beats the original wired build (its 5-frame filter alone lagged about 33 ms).

Why 925 MHz: it stays out of 2.4 GHz entirely (Wi-Fi, Bluetooth, Zigbee) and clear of Z-Wave (908.4 / 916 MHz) and Z-Wave Long Range (912 / 920 MHz).

## Parts

| Part | Notes |
|---|---|
| Seeed Studio XIAO ESP32-C3 | Controller brain, with a LiPo charger and battery pads built in |
| Adafruit LSM6DSOX 6-DoF (product 4438) | At the tip of the spring. Its wake-on-motion lets the controller sleep |
| Adafruit RFM69HCW 915 MHz breakout | One for the controller, **one more for the main unit** |
| 3.7 V LiPo, 1000–2000 mAh, with protection circuit | See "Battery" below |
| Small slide switch | On the battery's + lead |
| 2 × 220 kΩ resistors | Battery voltage sensing (optional but recommended) |
| Coin vibration motor module (3–5 V, with driver transistor) | Optional haptics |
| Solid-core wire, 78 mm × 2 | Quarter-wave antennas for both radios |
| The doorstop spring and a weighted base | Battery and electronics go in the base, not on the spring |

## Wiring: controller

GPIO numbers with the XIAO's printed labels. Power everything from the XIAO's **3V3** pin.

| XIAO pin | GPIO | Goes to |
|---|---|---|
| D0 | 2 | Middle of the 220k / 220k divider between battery + (after the switch) and GND |
| D1 | 3 | LSM6DSOX **I1** (motion interrupt; wakes the controller) |
| D2 | 4 | RFM69 **G0** |
| D3 | 5 | Vibration motor module **IN** (optional) |
| D4 | 6 | LSM6DSOX **SDA** |
| D5 | 7 | LSM6DSOX **SCL** |
| D6 | 21 | RFM69 **CS** |
| D7 | 20 | RFM69 **RST** |
| D8 | 8 | RFM69 **SCK** |
| D9 | 9 | RFM69 **MISO** |
| D10 | 10 | RFM69 **MOSI** |
| 3V3 | | LSM6DSOX **VIN**, RFM69 **VIN**, motor module **VCC** |
| GND | | Everything's GND |

Why these pins: only GPIO 0–5 can wake an ESP32-C3 from deep sleep, so the motion interrupt is on GPIO3. GPIO 2, 8 and 9 decide how the chip boots, so nothing that holds a pin low at power-up sits on GPIO2.

**The spring:** five thin wires run up to the LSM6DSOX: 3V3, GND, SDA, SCL and I1. Solder them; the snap-in cable connectors on the Adafruit board won't survive thousands of twangs. Leave a strain-relief loop at the base. The sensor's power LED draws about a milliamp all the time, so remove it (or cut its jumper, if the board has one) for the best standby life.

**Battery:** solder the LiPo to the BAT+ and BAT− pads on the underside of the XIAO, with the slide switch in the + lead. **Check the polarity before you solder:** JST battery leads aren't standardized between vendors, and reversing them destroys the charger. The divider for D0 goes after the switch so it draws nothing when off.

**Antenna:** solder a 78 mm straight wire to the RFM69's **ANT** pad. Point it up, and keep it away from the steel spring.

## Wiring: main unit

The receiver goes on the main ESP32's spare SPI pins (set in `TWANG32/config.h`). It's detected automatically at boot.

| RFM69 breakout | ESP32 GPIO |
|---|---|
| VIN | 5 V (or 3.3 V) |
| GND | GND |
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| CS | 5 |
| RST | 4 |
| G0 | 32 |

Put the same 78 mm antenna on it, vertical, and keep it out from behind the LED strip's aluminium channel. A wired sensor on the original I2C pins (21/22) still works as a fallback, LSM6DSOX or MPU6050.

## Flashing

1. **Main unit:** install the **RadioLib** library (Library Manager), then build and flash `TWANG32` as usual. The serial log should say `915 MHz radio ready on 925.000 MHz`.
2. **Controller:** in the Arduino IDE pick the board **XIAO_ESP32C3**, open `controller/TWANGController/TWANGController.ino`, and upload over USB-C. `NET_ID` in `controller_config.h` must match **Setup → Wireless controller → Radio network** on the main unit (both default to 42).
3. Open the serial monitor (115200). Every two seconds the controller prints its channel, `linked` or `hunting`, how the main unit hears it, and the battery voltage.

## First run

1. Switch the controller on. It hunts across the three channels and links within a second or two.
2. On your phone: **Setup → Wireless controller** shows "Connected, in control", the signal, packet rate (about 400/s) and battery.
3. **Setup → Spring:** let the spring hang still and press **Calibrate level**. Pick the **Tilt axis** that moves the meter when you tilt the spring, and flip **Reverse direction** if forward is backward. The Adafruit board is oriented differently from the old GY-521, so expect to change the axis once.
4. Flick the spring and watch the wobble meter turn blue. Set **Twang sensitivity** so a deliberate flick twangs and walking past doesn't. The original game's threshold is 230°/s.
5. Press **Buzz the controller** to test haptics.

## Living with it

- **Sleep:** after 3 minutes without movement the controller sleeps (about 50–300 µA, mostly the breakout boards' regulators). A nudge of the spring wakes it in well under a second, with a short buzz. Change `SLEEP_AFTER_S` in `controller_config.h`.
- **Battery:** in play the controller draws roughly 25–35 mA (my estimate; the Setup page shows the real voltage over time), so 2000 mAh is about 60–80 hours of play. The XIAO's built-in charger is slow, at 50–100 mA, so a flat 2000 mAh pack takes most of a day to refill. If that bothers you, a 1000 mAh pack still gives 30+ hours of play and charges overnight.
- **Low battery:** below 3.45 V the Setup page says "charge soon". Below 3.25 V the controller buzzes once and sleeps, checking every 10 minutes, to protect the cell.
- **Interference:** the main unit tracks packet loss. If a channel stays above 30% loss for 2 seconds, it moves both ends to the next of three frequencies (925.0, 922.6, 926.6 MHz). They're fixed in `TWANG32/src/link/twang_link.h`, compiled into both ends so they always agree. If you change them, run `tools/sync-link.sh` and reflash both.
- **Legal note:** the radio runs at −2 dBm on a fixed frequency, inside FCC Part 15.249's low-power limits for 902–928 MHz. Don't raise `TX_POWER_DBM`; at room distance it gains you nothing.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Controller always `hunting` | `NET_ID` mismatch, a missing antenna on either end, or the main unit's radio wasn't found (check its serial log) |
| Serial says `radio NOT FOUND` and it buzzes long | SPI wiring: check SCK/MISO/MOSI/CS, and that the breakout has power |
| XIAO won't run after power-up, but works after pressing reset | MISO (GPIO9) is held low at boot. Add a 10 kΩ pull-up from the RFM69's CS to 3V3 |
| Sleeps right after waking | Battery under 3.25 V, or a bad divider reading: set `HAS_BATTERY_SENSE 0` if you left the divider out |
| Never sleeps | `IMU_FAULT` (check the sensor's wires), or the spring is twitching above `MOTION_DPS` (raise it) |
| Player drifts | Calibrate with the spring at rest, then raise the dead zone by 2 |
