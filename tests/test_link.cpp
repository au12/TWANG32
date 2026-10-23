// TWANG Next - native tests for the wireless link protocol and input processing.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

#include "../TWANG32/src/core/tw_input.h"
#include "../TWANG32/src/link/twang_link.h"

using namespace tw;
using namespace twlink;

static int failures = 0;
#define CHECK(cond, ...)                              \
  do {                                                \
    if (!(cond)) {                                    \
      failures++;                                     \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__);   \
      printf(__VA_ARGS__);                            \
      printf("\n");                                   \
    }                                                 \
  } while (0)

static void testPackets() {
  printf("[packets]\n");
  Up u{200, UF_LISTEN | UF_LOW_BATT, {-981, 12, 32767}, {0, 1999, 65535}, 187, 9};
  uint8_t b[32];
  CHECK(encodeUp(u, b) == UP_LEN, "up length");
  Up v;
  CHECK(decodeUp(b, UP_LEN, v), "up decode");
  CHECK(v.seq == 200 && v.flags == (UF_LISTEN | UF_LOW_BATT) && v.accelMg[0] == -981 && v.accelMg[2] == 32767 &&
            v.gyroPeak[1] == 1999 && v.gyroPeak[2] == 65535 && v.batt20mV == 187 && v.dnAck == 9,
        "up round trip");
  CHECK(!decodeUp(b, UP_LEN - 1, v), "short up accepted");
  b[0] ^= 0xFF;
  CHECK(!decodeUp(b, UP_LEN, v), "wrong type accepted");
  Down d{7, 2, 35, DF_SLEEP, -71, 12};
  CHECK(encodeDown(d, b) == DOWN_LEN, "down length");
  Down e;
  CHECK(decodeDown(b, DOWN_LEN, e) && e.seq == 7 && e.channel == 2 && e.haptic4ms == 35 && e.flags == DF_SLEEP &&
            e.rssi == -71 && e.loss == 12,
        "down round trip");
  b[2] = NUM_CHANNELS;
  CHECK(!decodeDown(b, DOWN_LEN, e), "out of range channel accepted");
  uint8_t sw[4];
  syncWord(0, sw);
  CHECK(sw[0] && sw[1] && sw[2] && sw[3], "sync word contains a zero byte");
  for (int i = 0; i < NUM_CHANNELS; i++)
    CHECK(DEFAULT_CHANNELS_KHZ[i] >= MIN_KHZ && DEFAULT_CHANNELS_KHZ[i] <= MAX_KHZ, "default channel %d out of range", i);
}

static void testInput() {
  printf("[input]\n");
  InputConfig c;
  c.axis = 1;
  c.deadzone = 8;
  c.smoothMs = 0;
  InputProcessor p;
  ImuSample s = {{0, 0, 1000}, {0, 0, 0}};
  // level: no movement
  p.push(s);
  Input in = p.tick(c, true);
  CHECK(in.tilt == 0 && !in.attack, "level should be still");
  // 30 degrees-ish of tilt on Y: 500 mg -> 49 units -> minus deadzone 8 -> 41, and the original sign flip
  s.accelMg[1] = 500;
  p.push(s);
  in = p.tick(c, true);
  CHECK(std::fabs(in.tilt - (-(500 * 0.0987f - 8))) < 0.5f, "tilt mapping, got %.2f", in.tilt);
  c.invert = true;
  p.push(s);
  in = p.tick(c, true);
  CHECK(in.tilt > 40, "invert");
  c.invert = false;
  // smoothing lags but converges
  c.smoothMs = 25;
  InputProcessor q;
  ImuSample z = {{0, 0, 1000}, {0, 0, 0}};
  q.push(z);
  q.tick(c, true);
  ImuSample t = {{0, 600, 800}, {0, 0, 0}};
  float first = 0, later = 0;
  for (int i = 0; i < 12; i++) {
    q.push(t);
    Input r = q.tick(c, true);
    if (i == 0) first = r.tilt;
    later = r.tilt;
  }
  float target = -(600 * 0.0987f - 8);
  CHECK(std::fabs(first) < std::fabs(target) * 0.75f, "smoothing should lag on the first tick (%.1f vs %.1f)", first, target);
  CHECK(std::fabs(later - target) < 1.0f, "smoothing should converge (%.1f vs %.1f)", later, target);
  // twang: peak held between ticks, axis-specific by default
  c.smoothMs = 0;
  InputProcessor w;
  ImuSample g = {{0, 0, 1000}, {0, 300, 0}};
  w.push(g);
  g.gyroDps[1] = 40;
  w.push(g);  // a later, smaller sample must not hide the peak
  in = w.tick(c, true);
  CHECK(in.attack, "twang at 300 dps on the tilt axis");
  ImuSample h = {{0, 0, 1000}, {900, 0, 0}};
  w.push(h);
  in = w.tick(c, true);
  CHECK(!in.attack, "off-axis rotation should not twang by default");
  c.twangAnyAxis = true;
  w.push(h);
  in = w.tick(c, true);
  CHECK(in.attack, "any-axis twang");
  c.twangAnyAxis = false;
  w.push(g);
  in = w.tick(c, true);
  CHECK(!in.attack, "peak must reset after each tick");
  // failsafe
  ImuSample m = {{0, 900, 400}, {0, 0, 0}};
  w.push(m);
  in = w.tick(c, false);
  CHECK(in.tilt == 0 && !in.attack && !in.active, "stale source must stop the player");
  // calibration removes a resting offset but leaves the gravity axis alone
  Calibrator cal;
  for (int i = 0; i < 30; i++) cal.add(ImuSample{{37, -52, 1002}, {0, 0, 0}});
  InputConfig cc = c;
  CHECK(cal.apply(cc) && cc.offsetMg[0] == 37 && cc.offsetMg[1] == -52 && cc.offsetMg[2] == 0, "calibration");
  InputProcessor k;
  k.push(ImuSample{{37, -52, 1002}, {0, 0, 0}});
  in = k.tick(cc, true);
  CHECK(in.tilt == 0, "calibrated rest should read zero (%.2f)", in.tilt);
}

// Simulated radio: per-channel loss, controller streams at 400 Hz with listen windows.
struct Sim {
  std::mt19937 rng{1234};
  double loss[NUM_CHANNELS] = {0, 0, 0};
  bool pass(int ch) { return std::uniform_real_distribution<double>(0, 1)(rng) >= loss[ch]; }
};

struct LinkRun {
  int connectedMs = 0, totalMs = 0, retunes = 0;
  int finalChannel = -1;
};

static LinkRun runLink(Sim& sim, Receiver& rx, Hunter& hunter, int ms, uint32_t& now, uint8_t& seq,
                       int& rxTuned) {
  LinkRun r;
  for (int t = 0; t < ms * 2; t++) {  // 0.5 ms steps
    if (t % 5 == 0) {                  // a packet every 2.5 ms
      now = now + 0;  // clock advances below
      bool listen = (seq % LISTEN_EVERY) == 0;
      int ch = hunter.channel;
      if (ch == rxTuned && sim.pass(ch)) {
        rx.onPacket(seq, now);
        if (listen) {
          // main unit answers on the channel it is tuned to, telling the controller where to be
          Down d{0, rx.channel, 0, 0, -60, rx.lossPercent()};
          if (sim.pass(ch)) hunter.onDownlink(d, now);
          if (rxTuned != rx.channel) { rxTuned = rx.channel; r.retunes++; }
        }
      }
      seq++;
      hunter.poll(now);
    }
    if (t % 2 == 1) {
      now++;
      if (rx.poll(now) && !rx.connected(now)) { rxTuned = rx.channel; r.retunes++; }
      if (rx.connected(now)) r.connectedMs++;
      r.totalMs++;
    }
  }
  r.finalChannel = rxTuned;
  return r;
}

static void testLink() {
  printf("[link]\n");
  // clean channel: connects fast and stays put
  {
    Sim sim;
    Receiver rx;
    Hunter h;
    uint32_t now = 1000;
    rx.reset(now);
    h.reset(now);
    uint8_t seq = 0;
    int tuned = 0;
    LinkRun r = runLink(sim, rx, h, 5000, now, seq, tuned);
    printf("  clean: connected %.1f%% of the time, %d retunes, loss %u%%\n", 100.0 * r.connectedMs / r.totalMs, r.retunes,
           rx.lossPercent());
    CHECK(r.connectedMs > r.totalMs * 0.98, "clean link not connected");
    CHECK(r.retunes == 0, "clean link should not retune");
  }
  // channel 0 jammed outright: both ends must find another channel
  {
    Sim sim;
    sim.loss[0] = 1.0;
    Receiver rx;
    Hunter h;
    uint32_t now = 1000;
    rx.reset(now);
    h.reset(now);
    uint8_t seq = 0;
    int tuned = 0;
    LinkRun r = runLink(sim, rx, h, 8000, now, seq, tuned);
    printf("  channel 0 jammed: connected %.1f%%, ended on channel %d\n", 100.0 * r.connectedMs / r.totalMs, r.finalChannel);
    CHECK(r.finalChannel != 0, "stayed on a dead channel");
    CHECK(r.connectedMs > r.totalMs * 0.6, "took too long to find a working channel");
  }
  // channel 0 lossy (50%): main unit should notice and move the controller
  {
    Sim sim;
    sim.loss[0] = 0.5;
    Receiver rx;
    Hunter h;
    uint32_t now = 1000;
    rx.reset(now);
    h.reset(now);
    uint8_t seq = 0;
    int tuned = 0;
    LinkRun r = runLink(sim, rx, h, 10000, now, seq, tuned);
    printf("  channel 0 at 50%% loss: ended on channel %d, loss now %u%%\n", r.finalChannel, rx.lossPercent());
    CHECK(r.finalChannel != 0, "never left the lossy channel");
    CHECK(rx.lossPercent() < 10, "final loss too high");
  }
  // light interference (10%) is tolerated without hopping
  {
    Sim sim;
    sim.loss[0] = 0.10;
    Receiver rx;
    Hunter h;
    uint32_t now = 1000;
    rx.reset(now);
    h.reset(now);
    uint8_t seq = 0;
    int tuned = 0;
    LinkRun r = runLink(sim, rx, h, 6000, now, seq, tuned);
    printf("  channel 0 at 10%% loss: %d retunes, loss %u%%\n", r.retunes, rx.lossPercent());
    CHECK(r.retunes == 0, "hopped on light loss");
    CHECK(r.connectedMs > r.totalMs * 0.97, "light loss dropped the link");
  }
  // controller reboot (sequence jump) must not count as loss
  {
    Receiver rx;
    rx.reset(0);
    for (int i = 0; i < 200; i++) rx.onPacket((uint8_t)i, 2 + i * 2);
    for (int i = 0; i < 200; i++) rx.onPacket((uint8_t)(i + 130), 402 + i * 2);
    rx.poll(1001);
    CHECK(rx.lossPercent() < 3, "reboot counted as loss: %u%%", rx.lossPercent());
  }
}

int main() {
  testPackets();
  testInput();
  testLink();
  printf(failures ? "\n%d FAILURE(S)\n" : "\nLINK TESTS PASSED\n", failures);
  return failures ? 1 : 0;
}
