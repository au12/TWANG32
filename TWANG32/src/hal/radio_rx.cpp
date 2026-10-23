#include "radio_rx.h"
#include <RadioLib.h>
#include <SPI.h>
#include "../../config.h"
#include "../link/twang_link.h"

namespace radio_rx {

using namespace twlink;

static SPIClass* s_spi = nullptr;
static Module* s_mod = nullptr;
static RF69* s_radio = nullptr;
static TaskHandle_t s_task = nullptr;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_present = false;

// shared with the game loop (guarded by s_mux)
static tw::ImuSample s_sample;
static uint16_t s_peak[3];
static bool s_fresh;
static Status s_status;
static uint16_t s_hapticMs;
static bool s_sleepReq;
static uint8_t s_cfgNet;
static uint32_t s_cfgKhz[NUM_CHANNELS];
static bool s_cfgPending;

static void IRAM_ATTR onDio0() {
  BaseType_t woken = pdFALSE;
  if (s_task) vTaskNotifyGiveFromISR(s_task, &woken);
  portYIELD_FROM_ISR(woken);
}

static void applyConfig(uint8_t net, const uint32_t* khz, uint8_t channel) {
  uint8_t sw[4];
  syncWord(net, sw);
  s_radio->standby();
  s_radio->setSyncWord(sw, 4);
  s_radio->setFrequency(clampChannelKhz(khz[channel]) / 1000.0f);
}

static void task(void*) {
  Receiver rx;
  rx.reset(millis());
  uint8_t net;
  uint32_t khz[NUM_CHANNELS];
  portENTER_CRITICAL(&s_mux);
  net = s_cfgNet;
  memcpy(khz, s_cfgKhz, sizeof khz);
  s_cfgPending = false;
  portEXIT_CRITICAL(&s_mux);
  uint8_t tuned = 0;
  applyConfig(net, khz, tuned);
  s_radio->startReceive();

  bool transmitting = false;
  uint32_t txStart = 0;
  uint8_t downSeq = 0;
  uint16_t pktCount = 0;
  uint32_t pktWindow = millis();
  uint8_t buf[64];

  for (;;) {
    bool irq = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(4)) > 0;
    uint32_t now = millis();

    if (transmitting) {
      if (irq || now - txStart > 20) {  // packet sent (or we missed the interrupt)
        s_radio->finishTransmit();
        transmitting = false;
        if (tuned != rx.channel) {  // we just told the controller to move: follow it
          tuned = rx.channel;
          s_radio->setFrequency(clampChannelKhz(khz[tuned]) / 1000.0f);
        }
        s_radio->startReceive();
      }
    } else if (irq) {
      float rssi = s_radio->getRSSI();  // read before readData() drops to standby
      size_t len = s_radio->getPacketLength();
      int16_t st = len <= sizeof buf ? s_radio->readData(buf, len) : RADIOLIB_ERR_PACKET_TOO_LONG;
      Up up;
      bool ok = st == RADIOLIB_ERR_NONE && decodeUp(buf, (int)len, up);
      if (ok) {
        rx.onPacket(up.seq, now);
        pktCount++;
        portENTER_CRITICAL(&s_mux);
        for (int i = 0; i < 3; i++) {
          s_sample.accelMg[i] = up.accelMg[i];
          if (up.gyroPeak[i] > s_peak[i]) s_peak[i] = up.gyroPeak[i];
        }
        s_fresh = true;
        s_status.rssi = (int8_t)constrain((int)rssi, -127, 0);
        s_status.battMv = (uint16_t)up.batt20mV * 20;
        s_status.lowBatt = up.flags & UF_LOW_BATT;
        s_status.imuFault = up.flags & UF_IMU_FAULT;
        s_status.lastSeenMs = now;
        uint16_t hap = s_hapticMs;
        bool sleepNow = s_sleepReq;
        if (up.flags & UF_LISTEN) { s_hapticMs = 0; s_sleepReq = false; }
        portEXIT_CRITICAL(&s_mux);

        if (up.flags & UF_LISTEN) {
          Down d;
          d.seq = ++downSeq;
          d.channel = rx.channel;
          d.haptic4ms = (uint8_t)min<uint16_t>((hap + 3) / 4, 255);
          d.flags = sleepNow ? DF_SLEEP : 0;
          d.rssi = (int8_t)constrain((int)rssi, -127, 0);
          d.loss = rx.lossPercent();
          int n = encodeDown(d, buf);
          if (s_radio->startTransmit(buf, n) == RADIOLIB_ERR_NONE) {
            transmitting = true;
            txStart = now;
            continue;
          }
        }
      }
      s_radio->startReceive();
    }

    // channel management and config changes (only while receiving)
    if (!transmitting) {
      if (rx.poll(now) && !rx.connected(now) && tuned != rx.channel) {
        tuned = rx.channel;
        s_radio->setFrequency(clampChannelKhz(khz[tuned]) / 1000.0f);
        s_radio->startReceive();
      }
      bool pending;
      portENTER_CRITICAL(&s_mux);
      pending = s_cfgPending;
      if (pending) {
        net = s_cfgNet;
        memcpy(khz, s_cfgKhz, sizeof khz);
        s_cfgPending = false;
      }
      portEXIT_CRITICAL(&s_mux);
      if (pending) {
        applyConfig(net, khz, tuned);
        s_radio->startReceive();
      }
    }

    if (now - pktWindow >= 1000) {
      portENTER_CRITICAL(&s_mux);
      s_status.packetsPerSec = pktCount;
      portEXIT_CRITICAL(&s_mux);
      pktCount = 0;
      pktWindow = now;
    }
    portENTER_CRITICAL(&s_mux);
    s_status.connected = rx.connected(now);
    s_status.loss = rx.lossPercent();
    s_status.channel = tuned;
    s_status.freqKhz = clampChannelKhz(khz[tuned]);
    portEXIT_CRITICAL(&s_mux);
  }
}

bool begin(uint8_t netId, const uint32_t channelsKhz[3]) {
  s_spi = new SPIClass(VSPI);
  s_spi->begin(RADIO_SCK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
  s_mod = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIOLIB_NC, *s_spi, SPISettings(8000000, MSBFIRST, SPI_MODE0));
  s_radio = new RF69(s_mod);
  int16_t st = s_radio->begin(clampChannelKhz(channelsKhz[0]) / 1000.0f, BITRATE_KBPS, FREQ_DEV_KHZ, RX_BW_KHZ, 10, PREAMBLE_BITS);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("No 915 MHz radio found (RadioLib error %d); wireless controller disabled\n", st);
    delete s_radio;
    delete s_mod;
    s_spi->end();
    delete s_spi;
    s_radio = nullptr;
    return false;
  }
  s_radio->setOutputPower(TX_POWER_DBM, true);  // RFM69HCW: high-power PA path, at its minimum setting
  s_radio->setDataShaping(RADIOLIB_SHAPING_0_5);
  s_radio->setEncoding(RADIOLIB_ENCODING_WHITENING);
  s_radio->setCrcFiltering(true);
  s_radio->variablePacketLengthMode(32);
  s_radio->setDio0Action(onDio0);
  s_cfgNet = netId;
  for (int i = 0; i < NUM_CHANNELS; i++) s_cfgKhz[i] = channelsKhz[i];
  memset(&s_status, 0, sizeof s_status);
  s_status.present = true;
  s_status.freqKhz = clampChannelKhz(channelsKhz[0]);
  s_present = true;
  xTaskCreatePinnedToCore(task, "radio", 4096, nullptr, 4, &s_task, 0);
  Serial.printf("915 MHz radio ready on %.3f MHz, network %u\n", s_status.freqKhz / 1000.0f, netId);
  return true;
}

void configure(uint8_t netId, const uint32_t channelsKhz[3]) {
  if (!s_present) return;
  portENTER_CRITICAL(&s_mux);
  s_cfgNet = netId;
  for (int i = 0; i < NUM_CHANNELS; i++) s_cfgKhz[i] = channelsKhz[i];
  s_cfgPending = true;
  portEXIT_CRITICAL(&s_mux);
}

bool take(tw::ImuSample& out) {
  if (!s_present) return false;
  bool fresh;
  portENTER_CRITICAL(&s_mux);
  fresh = s_fresh && s_status.lastSeenMs && (millis() - s_status.lastSeenMs) < STALE_MS;
  out = s_sample;
  for (int i = 0; i < 3; i++) {
    out.gyroDps[i] = s_peak[i];
    s_peak[i] = 0;
  }
  portEXIT_CRITICAL(&s_mux);
  return fresh;
}

void haptic(uint16_t ms) {
  if (!s_present) return;
  portENTER_CRITICAL(&s_mux);
  if (ms > s_hapticMs) s_hapticMs = ms;
  portEXIT_CRITICAL(&s_mux);
}

void sleepController() {
  if (!s_present) return;
  portENTER_CRITICAL(&s_mux);
  s_sleepReq = true;
  portEXIT_CRITICAL(&s_mux);
}

Status status() {
  Status s;
  portENTER_CRITICAL(&s_mux);
  s = s_status;
  s.present = s_present;
  if (s_present && s.lastSeenMs) s.connected = millis() - s.lastSeenMs < STALE_MS;
  portEXIT_CRITICAL(&s_mux);
  return s;
}

}  // namespace radio_rx
