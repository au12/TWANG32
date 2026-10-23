/*
  TWANG Next - wireless spring controller

  Seeed XIAO ESP32-C3 + Adafruit LSM6DSOX (at the spring tip) + Adafruit RFM69HCW 915 MHz.
  Samples the spring at ~833 Hz and streams its state to the main unit 400 times a
  second on 925 MHz. See docs/CONTROLLER.md for parts, wiring and setup.

  Board: "XIAO_ESP32C3" (ESP32 Arduino core 3.x). Library: RadioLib.
  Shared files twang_link.h / twang_imu.h are copies; edit them in TWANG32/src/link.
*/
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_sleep.h>

#include "controller_config.h"
#include "twang_imu.h"
#include "twang_link.h"

using namespace twlink;

static TwangImu imu;
static SPIClass& spi = SPI;
static RF69 radio = new Module(PIN_RADIO_CS, PIN_RADIO_G0, PIN_RADIO_RST, RADIOLIB_NC, SPI, SPISettings(8000000, MSBFIRST, SPI_MODE0));
static Hunter hunter;

static volatile bool radioIrq = false;
static void IRAM_ATTR onRadio() { radioIrq = true; }

enum RadioState : uint8_t { R_IDLE, R_TX, R_LISTEN };
static RadioState rstate = R_IDLE;
static bool radioOk = false;
static bool lastWasListen = false;
static uint32_t listenUntilUs = 0;
static uint32_t nextTxUs = 0;
static uint8_t seq = 0;
static uint8_t dnAck = 0;

// motion state
static int16_t accelHist[3][5];
static uint8_t accelIdx = 0;
static uint16_t slotPeak[3];      // max |gyro| in the current 2.5 ms slot
static uint16_t ring[3][16];      // last 16 slots = 40 ms of held peaks
static uint8_t ringIdx = 0;
static uint32_t nextSampleUs = 0;
static bool imuOk = false;

// housekeeping
static uint16_t battMv = 0;
static uint32_t battAt = 0;
static uint32_t lastMotionMs = 0;
static uint32_t wokeAt = 0;
static uint32_t hapticUntil = 0;
static uint32_t statsAt = 0, sent = 0, heard = 0;
static int8_t lastRssiAtMain = 0;
static uint8_t lastLossAtMain = 0;

static void tune(uint8_t ch) { radio.setFrequency(clampChannelKhz(DEFAULT_CHANNELS_KHZ[ch]) / 1000.0f); }

static void buzz(uint16_t ms) {
  digitalWrite(PIN_HAPTIC, HIGH);
  uint32_t until = millis() + ms;
  if ((int32_t)(until - hapticUntil) > 0) hapticUntil = until;
}

static uint16_t readBattery() {
#if HAS_BATTERY_SENSE
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(PIN_BATT);
  uint16_t mv = (uint16_t)((sum / 8) * BATT_DIVIDER);
  // a working LiPo is never below 2.5 V: lower means the divider isn't fitted (floating pin)
  return mv < 2500 ? 0 : mv;
#else
  return 0;
#endif
}

static int16_t median5(const int16_t* v) {
  int16_t s[5];
  for (int i = 0; i < 5; i++) s[i] = v[i];
  for (int i = 1; i < 5; i++)
    for (int j = i; j > 0 && s[j - 1] > s[j]; j--) { int16_t t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
  return s[2];
}

static void deepSleep(bool motionWake, uint32_t timerSec) {
  Serial.printf("sleeping (%s)\n", motionWake ? "move the spring to wake" : "battery low");
  Serial.flush();
  digitalWrite(PIN_HAPTIC, LOW);
  if (radioOk) radio.sleep();
  if (motionWake && imu.kind() == TwangImu::LSM6DSOX && imu.sleepUntilMotion(WAKE_THRESHOLD_MG)) {
    esp_deep_sleep_enable_gpio_wakeup(1ULL << PIN_IMU_INT, ESP_GPIO_WAKEUP_GPIO_HIGH);
  } else if (!timerSec) {
    timerSec = 5;  // no wake-on-motion sensor: poll occasionally instead
  }
  if (timerSec) esp_sleep_enable_timer_wakeup((uint64_t)timerSec * 1000000ULL);
  esp_deep_sleep_start();
}

void setup() {
  setCpuFrequencyMhz(CPU_MHZ);
  Serial.begin(115200);
  pinMode(PIN_HAPTIC, OUTPUT);
  digitalWrite(PIN_HAPTIC, LOW);
  pinMode(PIN_IMU_INT, INPUT);
#if HAS_BATTERY_SENSE
  analogReadResolution(12);
#endif

  battMv = readBattery();
  if (battMv && battMv < CUTOFF_MV) deepSleep(false, 600);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(3);
  imuOk = imu.begin(Wire);

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_RADIO_CS);
  int16_t st = radio.begin(clampChannelKhz(DEFAULT_CHANNELS_KHZ[0]) / 1000.0f, BITRATE_KBPS, FREQ_DEV_KHZ, RX_BW_KHZ, 10,
                           PREAMBLE_BITS);
  radioOk = st == RADIOLIB_ERR_NONE;
  if (radioOk) {
    uint8_t sw[4];
    syncWord(NET_ID, sw);
    radio.setOutputPower(TX_POWER_DBM, true);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    radio.setEncoding(RADIOLIB_ENCODING_WHITENING);
    radio.setCrcFiltering(true);
    radio.variablePacketLengthMode(32);
    radio.setSyncWord(sw, 4);
    radio.setDio0Action(onRadio);
  }

  Serial.printf("\nTWANG controller: sensor %s, radio %s (error %d), network %u, battery %u mV\n", imu.name(),
                radioOk ? "RFM69 ok" : "NOT FOUND", st, NET_ID, battMv);
  if (!radioOk) {
    // nothing useful to do without a radio: say so with a long buzz, then sleep
    buzz(600);
    delay(600);
    deepSleep(true, 0);
  }

  hunter.reset(millis());
  lastMotionMs = wokeAt = millis();
  nextSampleUs = nextTxUs = micros();
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) buzz(30);  // "I'm awake"
}

static void sampleImu() {
  int16_t a[3], g[3];
  if (!imuOk) return;
  if (!imu.read(a, g)) {
    if (!imu.present()) imuOk = false;
    return;
  }
  bool moving = false;
  for (int i = 0; i < 3; i++) {
    accelHist[i][accelIdx] = a[i];
    uint16_t r = (uint16_t)abs(g[i]);
    if (r > slotPeak[i]) slotPeak[i] = r;
    if (r > MOTION_DPS) moving = true;
  }
  accelIdx = (accelIdx + 1) % 5;
  if (moving) lastMotionMs = millis();
}

static void sendPacket(uint32_t nowMs) {
  bool linked = hunter.linked(nowMs);
  Up u;
  u.seq = seq++;
  // while hunting for the main unit, every packet asks for an answer
  bool listen = !linked || (u.seq % LISTEN_EVERY) == 0;
  u.flags = (listen ? UF_LISTEN : 0) | (battMv && battMv < LOW_BATT_MV ? UF_LOW_BATT : 0) |
            (nowMs - wokeAt < 1000 ? UF_JUST_WOKE : 0) | (imuOk ? 0 : UF_IMU_FAULT);
  for (int i = 0; i < 3; i++) {
    u.accelMg[i] = imuOk ? median5(accelHist[i]) : 0;
    ring[i][ringIdx] = slotPeak[i];
    slotPeak[i] = 0;
    uint16_t held = 0;
    for (int k = 0; k < 16; k++) if (ring[i][k] > held) held = ring[i][k];
    u.gyroPeak[i] = held;
  }
  ringIdx = (ringIdx + 1) % 16;
  u.batt20mV = (uint8_t)min<uint16_t>(battMv / 20, 255);
  u.dnAck = dnAck;
  uint8_t buf[UP_LEN];
  encodeUp(u, buf);
  radioIrq = false;
  if (radio.startTransmit(buf, UP_LEN) == RADIOLIB_ERR_NONE) {
    rstate = R_TX;
    lastWasListen = listen;
    sent++;
  }
}

static void onDownlink(uint32_t nowMs) {
  uint8_t buf[32];
  size_t len = radio.getPacketLength();
  if (len > sizeof buf || radio.readData(buf, len) != RADIOLIB_ERR_NONE) return;
  Down d;
  if (!decodeDown(buf, (int)len, d)) return;
  heard++;
  dnAck = d.seq;
  lastRssiAtMain = d.rssi;
  lastLossAtMain = d.loss;
  uint8_t before = hunter.channel;
  hunter.onDownlink(d, nowMs);
  if (hunter.channel != before) tune(hunter.channel);
  if (d.haptic4ms) buzz((uint16_t)d.haptic4ms * 4);
  if (d.flags & DF_SLEEP) deepSleep(true, 0);
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  // ---- sample the spring at ~833 Hz
  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    nextSampleUs += 1200;
    if ((int32_t)(nowUs - nextSampleUs) > 5000) nextSampleUs = nowUs + 1200;
    sampleImu();
  }

  // ---- radio state machine
  switch (rstate) {
    case R_TX:
      if (radioIrq) {
        radioIrq = false;
        radio.finishTransmit();
        if (lastWasListen) {
          radio.startReceive();
          listenUntilUs = micros() + LISTEN_WINDOW_US;
          rstate = R_LISTEN;
        } else {
          rstate = R_IDLE;
        }
      }
      break;
    case R_LISTEN:
      if (radioIrq) {
        radioIrq = false;
        onDownlink(nowMs);
        radio.standby();
        rstate = R_IDLE;
      } else if ((int32_t)(nowUs - listenUntilUs) >= 0) {
        radio.standby();
        rstate = R_IDLE;
      }
      break;
    case R_IDLE:
      if ((int32_t)(nowUs - nextTxUs) >= 0) {
        bool linked = hunter.linked(nowMs);
        nextTxUs += linked ? UP_PERIOD_US : 10000;  // 400 Hz when linked, 100 Hz while hunting
        if ((int32_t)(nowUs - nextTxUs) > 20000) nextTxUs = nowUs;
        if (hunter.poll(nowMs)) tune(hunter.channel);
        sendPacket(nowMs);
      }
      break;
  }

  // ---- haptics, battery, sleep
  if (hapticUntil && (int32_t)(nowMs - hapticUntil) >= 0) {
    digitalWrite(PIN_HAPTIC, LOW);
    hapticUntil = 0;
  }
  if (nowMs - battAt > 2000 && rstate == R_IDLE) {
    battAt = nowMs;
    uint16_t v = readBattery();
    if (v) battMv = battMv ? (uint16_t)((battMv * 3 + v) / 4) : v;
    if (battMv && battMv < CUTOFF_MV) {
      buzz(80);
      delay(200);
      deepSleep(false, 600);
    }
  }
  if (imuOk && nowMs - lastMotionMs > (uint32_t)SLEEP_AFTER_S * 1000) deepSleep(true, 0);

  if (Serial && nowMs - statsAt > 2000) {
    statsAt = nowMs;
    Serial.printf("ch %u %s | sent %lu/2s heard %lu | main hears us at %d dBm, %u%% loss | batt %u mV | tilt mg %d %d %d\n",
                  hunter.channel, hunter.linked(nowMs) ? "linked " : "hunting", (unsigned long)sent, (unsigned long)heard,
                  lastRssiAtMain, lastLossAtMain, battMv, median5(accelHist[0]), median5(accelHist[1]), median5(accelHist[2]));
    sent = heard = 0;
  }
}
