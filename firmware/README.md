# Prebuilt firmware

Built from this branch for the stock TWANG32 board (`config.h` defaults: APA102 on GPIO16/17, MPU6050 on 21/22, DAC audio on GPIO25).

- `twang-next-stock-merged.bin`: first install over USB. Write it at address `0x0` with [esptool-js](https://espressif.github.io/esptool-js/) in Chrome, or `esptool.py --chip esp32 write_flash 0x0 twang-next-stock-merged.bin`.
- `twang-next-stock-ota.bin`: later updates from the web page (Setup → Update firmware).
- `twang-controller-xiao-esp32c3-merged.bin`: the wireless controller, for a Seeed XIAO ESP32-C3 wired as in `docs/CONTROLLER.md` (network ID 42). Write it at `0x0` the same way.

If your hardware differs (WS2812 strip, I2S amplifier, other pins), change `TWANG32/config.h` and build from source instead.
