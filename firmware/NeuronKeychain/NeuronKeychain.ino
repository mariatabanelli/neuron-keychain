/*
  Neuron keychain, firmware v0.2
  Board : Waveshare ESP32-S3-Touch-LCD-1.28 (GC9A01 round 240x240, CST816S touch, QMI8658 IMU)
  Core  : esp32 by Espressif 3.1.x   Libs: GFX Library for Arduino (moononournation)
  Settings: ESP32S3 Dev Module, USB CDC On Boot Enabled, Flash 16MB, PSRAM QSPI, Partition 16M (3MB APP)

  Screens (swipe left/right to change):
    0 HOME    leaky integrate-and-fire neuron on the ring. Tap = pulse, hold = steady current,
              shake = noise current. Also accepts current over OSC (/neuron/current) and shows the
              sleep stage sent with it (/neuron/stage).
    1 PARAMS  six dials. Tap = next dial, drag up/down = change value.
  OSC out (if Wi-Fi configured in secrets.h): /neuron/v (float, mV) every frame, /neuron/spike (int) on spike.
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "CST816S.h"
#include "QMI8658.h"
#include "secrets.h"

// ---------------- pins ----------------
#define PIN_BL 2
#define PIN_DC 8
#define PIN_CS 9
#define PIN_SCK 10
#define PIN_MOSI 11
#define PIN_MISO 12
#define PIN_RST 14
#define TP_SDA 6
#define TP_SCL 7
#define TP_RST 13
#define TP_IRQ 5
#define PIN_VBAT 1

Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);
Arduino_GFX *panel = new Arduino_GC9A01(bus, PIN_RST, 0, true);
Arduino_Canvas *gfx = new Arduino_Canvas(240, 240, panel);
CST816S touch(TP_SDA, TP_SCL, TP_RST, TP_IRQ);

// ---------------- neuron ----------------
float V_REST = -70.0f, V_THRESH = -55.0f, V_RESET = -75.0f;
float TAU_M = 20.0f, R_M = 10.0f, T_REF = 2.0f;
float I_HOLD = 2.0f, I_TAP = 8.0f, SHAKE_GAIN = 1.0f;
const float DT = 0.1f, TAP_MS = 5.0f;

float v = -70.0f, refUntil = -1, tapUntil = -1, tModel = 0;
uint32_t spikeCount = 0;
float flash = 0;
bool spikePending = false;
bool spikeThisFrame = false;

#define N_TRACE 360
float trace[N_TRACE];
int traceHead = 0;
const float TRACE_MS_PER_SAMPLE = 2000.0f / N_TRACE;
float sinceSample = 0;

// ---------------- parameters screen ----------------
struct Param { const char *name; const char *unit; float *val; float step; float lo, hi; };
Param params[] = {
  { "TAU",   "ms",  &TAU_M,    0.5f,  2.0f,  100.0f },
  { "I",     "nA",  &I_HOLD,   0.05f, 0.0f,  6.0f   },
  { "Vth",   "mV",  &V_THRESH, 0.25f, -65.0f, -40.0f },
  { "Vrst",  "mV",  &V_RESET,  0.25f, -90.0f, -60.0f },
  { "R",     "MO",  &R_M,      0.25f, 2.0f,  30.0f  },
  { "Tref",  "ms",  &T_REF,    0.1f,  0.0f,  10.0f  },
};
const int N_PARAMS = sizeof(params) / sizeof(params[0]);
int selParam = 0;

// ---------------- touch ----------------
int screen = 0;                    // 0 home, 1 params
bool fingerDown = false;
int tx = 0, ty = 0, tx0 = 0, ty0 = 0, tyPrev = 0;
uint32_t tDown = 0, lastTouchPoll = 0, tapShownUntil = 0;
bool swipeDone = false, dragged = false;

bool pollTouch(bool &down, int &x, int &y) {
  Wire.beginTransmission(CST816S_ADDRESS);
  Wire.write(0x02);
  if (Wire.endTransmission(true) != 0) { down = false; return false; }
  if (Wire.requestFrom((uint16_t)CST816S_ADDRESS, (uint32_t)5, true) != 5) { down = false; return false; }
  uint8_t n = Wire.read(), xh = Wire.read(), xl = Wire.read(), yh = Wire.read(), yl = Wire.read();
  down = n > 0;
  x = ((xh & 0x0F) << 8) | xl;
  y = ((yh & 0x0F) << 8) | yl;
  return true;
}
void touchDisableAutoSleep() {
  Wire.beginTransmission(CST816S_ADDRESS); Wire.write(0xFE); Wire.write(0x01); Wire.endTransmission(true);
}

// ---------------- IMU / shake ----------------
bool imuOk = false;
float accPrev[3] = {0, 0, 0};
float shake = 0;                   // smoothed jerk, arbitrary units
void readShake() {
  if (!imuOk) return;
  float acc[3], gyro[3]; unsigned int tc;
  QMI8658_read_xyz(acc, gyro, &tc);
  float j = fabsf(acc[0] - accPrev[0]) + fabsf(acc[1] - accPrev[1]) + fabsf(acc[2] - accPrev[2]);
  for (int i = 0; i < 3; i++) accPrev[i] = acc[i];
  shake = 0.85f * shake + 0.15f * j;
}
float gaussNoise() {               // approx N(0,1): sum of 4 uniforms
  float s = 0; for (int i = 0; i < 4; i++) s += (float)random(0, 10000) / 10000.0f - 0.5f;
  return s * 1.732f;
}

// ---------------- Wi-Fi / OSC ----------------
WiFiUDP udp;
bool wifiWanted = strcmp(WIFI_SSID, "your-wifi-name") != 0;
float extCurrent = 0; uint32_t extCurrentUntil = 0;
char stageLabel[4] = "";
uint8_t oscBuf[256];

void oscSend(const char *addr, float f) {
  if (WiFi.status() != WL_CONNECTED) return;
  int al = strlen(addr) + 1, ap = ((al + 3) / 4) * 4;
  memset(oscBuf, 0, ap + 4 + 4);
  memcpy(oscBuf, addr, al);
  memcpy(oscBuf + ap, ",f\0\0", 4);
  uint32_t u; memcpy(&u, &f, 4);
  uint8_t *p = oscBuf + ap + 4; p[0] = u >> 24; p[1] = u >> 16; p[2] = u >> 8; p[3] = u;
  udp.beginPacket(LAPTOP_IP, OSC_OUT_PORT); udp.write(oscBuf, ap + 8); udp.endPacket();
}
void oscReceive() {
  int n = udp.parsePacket();
  while (n > 0) {
    int len = udp.read(oscBuf, sizeof(oscBuf) - 1); oscBuf[len] = 0;
    const char *addr = (const char *)oscBuf;
    int ap = ((strlen(addr) + 1 + 3) / 4) * 4;
    const char *tags = (const char *)oscBuf + ap;
    int tp = ((strlen(tags) + 1 + 3) / 4) * 4;
    const uint8_t *arg = oscBuf + ap + tp;
    if (!strcmp(addr, "/neuron/current") && tags[1] == 'f') {
      uint32_t u = ((uint32_t)arg[0] << 24) | ((uint32_t)arg[1] << 16) | ((uint32_t)arg[2] << 8) | arg[3];
      memcpy(&extCurrent, &u, 4); extCurrentUntil = millis() + 5000;
    } else if (!strcmp(addr, "/neuron/stage")) {
      if (tags[1] == 's') { strncpy(stageLabel, (const char *)arg, 3); stageLabel[3] = 0; }
      else if (tags[1] == 'i') { int s = arg[3]; const char *names[] = {"W", "1", "2", "3", "R"}; strcpy(stageLabel, s >= 0 && s < 5 ? names[s] : "?"); }
    }
    n = udp.parsePacket();
  }
}

// ---------------- model ----------------
void stepModel(float iIn) {
  tModel += DT;
  if (tModel < refUntil) v = V_RESET;
  else {
    v += ((-(v - V_REST) + R_M * iIn) / TAU_M) * DT;
    if (v >= V_THRESH) { v = 20.0f; spikeCount++; flash = 1.0f; spikePending = true; spikeThisFrame = true; refUntil = tModel + T_REF; }
  }
  sinceSample += DT;
  if (sinceSample >= TRACE_MS_PER_SAMPLE) {
    sinceSample -= TRACE_MS_PER_SAMPLE;
    trace[traceHead] = spikePending ? 20.0f : v; spikePending = false;
    traceHead = (traceHead + 1) % N_TRACE;
  }
  if (v > V_THRESH && tModel >= refUntil) v = V_RESET;
}

// ---------------- drawing ----------------
const int CX = 120, CY = 120;
const float R_MIN = 62.0f, R_MAX = 108.0f;
float vToR(float mv) { float t = (mv - V_RESET) / (20.0f - V_RESET); if (t < 0) t = 0; if (t > 1) t = 1; return R_MIN + t * (R_MAX - R_MIN); }
void drawDashedCircle(float r, uint16_t col) {
  for (int a = 0; a < 360; a += 6) { float a0 = a * DEG_TO_RAD, a1 = (a + 3) * DEG_TO_RAD;
    gfx->drawLine(CX + cosf(a0) * r, CY + sinf(a0) * r, CX + cosf(a1) * r, CY + sinf(a1) * r, col); }
}
void drawStatusDots(float batteryV) {
  char buf[16];
  gfx->setTextSize(1); gfx->setTextColor(gfx->color565(140, 140, 140));
  gfx->setCursor(CX - 14, 26);
  if (batteryV > 2.5f) { snprintf(buf, sizeof(buf), "%.2fV", batteryV); gfx->print(buf); } else gfx->print("USB");
  if (wifiWanted) gfx->fillCircle(CX + 30, 29, 3, WiFi.status() == WL_CONNECTED ? gfx->color565(60, 220, 120) : gfx->color565(120, 60, 60));
  if (imuOk) gfx->fillCircle(CX - 30, 29, 3, gfx->color565(60, 120, 220));
}

void renderHome(float batteryV, bool shaking) {
  gfx->fillScreen(RGB565_BLACK);
  if (flash > 0.01f) { uint8_t g = (uint8_t)(flash * 90); gfx->fillCircle(CX, CY, 118, gfx->color565(g, g / 3, g)); gfx->fillCircle(CX, CY, 112, RGB565_BLACK); }
  gfx->drawCircle(CX, CY, (int)vToR(V_REST), gfx->color565(40, 40, 40));
  drawDashedCircle(vToR(V_THRESH), gfx->color565(110, 110, 110));
  int px = 0, py = 0;
  for (int i = 0; i < N_TRACE; i++) {
    int idx = (traceHead + i) % N_TRACE; float ang = (-90.0f + i) * DEG_TO_RAD; float r = vToR(trace[idx]);
    int x = CX + cosf(ang) * r, y = CY + sinf(ang) * r;
    if (i > 0) { uint8_t b = 60 + (uint8_t)(195.0f * i / N_TRACE);
      gfx->drawLine(px, py, x, y, trace[idx] > V_THRESH ? gfx->color565(255, 120, 255) : gfx->color565(b, b, b)); }
    px = x; py = y;
  }
  char buf[16];
  gfx->setTextColor(RGB565_WHITE); gfx->setTextSize(3);
  snprintf(buf, sizeof(buf), "%4.0f", v > V_THRESH ? V_THRESH : v); gfx->setCursor(CX - 36, CY - 22); gfx->print(buf);
  gfx->setTextSize(1);
  gfx->setCursor(CX - 8, CY + 6); gfx->print("mV");
  gfx->setCursor(CX - 30, CY + 24); gfx->print("spikes "); gfx->print(spikeCount);
  gfx->setCursor(CX - 24, CY + 64);
  if (millis() < tapShownUntil && !fingerDown) gfx->print(" TAP ");
  else if (fingerDown) gfx->print(" HOLD");
  else if (shaking) gfx->print("SHAKE");
  else if (millis() < extCurrentUntil) { snprintf(buf, sizeof(buf), "EEG %.1f", extCurrent); gfx->print(buf); }
  else gfx->print(" rest");
  if (stageLabel[0] && millis() < extCurrentUntil) { gfx->setTextSize(2); gfx->setCursor(CX - 6, CY + 84); gfx->print(stageLabel); }
  drawStatusDots(batteryV);
  gfx->flush();
}

void renderParams(float batteryV) {
  gfx->fillScreen(RGB565_BLACK);
  // six ticks around the ring, selected one bright
  for (int i = 0; i < N_PARAMS; i++) {
    float a = (-90.0f + i * 60.0f) * DEG_TO_RAD;
    int x = CX + cosf(a) * 96, y = CY + sinf(a) * 96;
    bool sel = i == selParam;
    gfx->fillCircle(x, y, sel ? 6 : 3, sel ? gfx->color565(255, 120, 255) : gfx->color565(90, 90, 90));
    gfx->setTextSize(1); gfx->setTextColor(sel ? RGB565_WHITE : gfx->color565(120, 120, 120));
    gfx->setCursor(x - 3 * strlen(params[i].name), y + (sinf(a) > 0.3f ? 10 : (sinf(a) < -0.3f ? -16 : -4)) + (cosf(a) > 0.5f ? 0 : 0));
    gfx->print(params[i].name);
  }
  Param &p = params[selParam];
  char buf[20];
  gfx->setTextColor(RGB565_WHITE); gfx->setTextSize(2);
  gfx->setCursor(CX - 6 * strlen(p.name), CY - 40); gfx->print(p.name);
  gfx->setTextSize(4);
  snprintf(buf, sizeof(buf), *p.val < 10 && *p.val > -10 ? "%.2f" : "%.1f", *p.val);
  gfx->setCursor(CX - 12 * strlen(buf), CY - 12); gfx->print(buf);
  gfx->setTextSize(1); gfx->setTextColor(gfx->color565(140, 140, 140));
  gfx->setCursor(CX - 3 * strlen(p.unit), CY + 26); gfx->print(p.unit);
  gfx->setCursor(CX - 54, CY + 48); gfx->print("drag up/down  tap=next");
  drawStatusDots(batteryV);
  gfx->flush();
}

float readBattery() { uint32_t acc = 0; for (int i = 0; i < 8; i++) acc += analogRead(PIN_VBAT); return (acc / 8.0f) * 3.3f / 4096.0f * 3.0f; }

// ---------------- setup / loop ----------------
uint32_t lastFrame = 0, lastBat = 0; float batteryV = 0;

void setup() {
  pinMode(PIN_BL, OUTPUT); digitalWrite(PIN_BL, HIGH);
  Serial.begin(115200);
  for (int i = 0; i < N_TRACE; i++) trace[i] = V_REST;
  gfx->begin(); gfx->fillScreen(RGB565_BLACK); gfx->flush();
  touch.begin(); touchDisableAutoSleep();
  imuOk = QMI8658_init() != 0;
  if (imuOk) { float a[3], g[3]; unsigned int tc; QMI8658_read_xyz(a, g, &tc); for (int i = 0; i < 3; i++) accPrev[i] = a[i]; }
  analogReadResolution(12);
  if (wifiWanted) { WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS); udp.begin(OSC_IN_PORT); }
  randomSeed(micros());
  lastFrame = millis();
}

void loop() {
  uint32_t now = millis();

  // ---- touch ----
  if (touch.available()) { if (!fingerDown && screen == 0) { tapUntil = tModel + TAP_MS; tapShownUntil = now + 300; } }
  if (now - lastTouchPoll >= 15) {
    lastTouchPoll = now;
    bool down; int x, y;
    pollTouch(down, x, y);
    if (down && !fingerDown) {                       // landed
      tx0 = x; ty0 = y; tyPrev = y; tDown = now; swipeDone = false; dragged = false;
      if (screen == 0) { tapUntil = tModel + TAP_MS; tapShownUntil = now + 300; }
    }
    if (down) {
      tx = x; ty = y;
      int dx = x - tx0, dy = y - ty0;
      if (!swipeDone && abs(dx) > 50 && abs(dx) > abs(dy)) { screen = (screen + (dx > 0 ? 1 : N_PARAMS)) % 2; swipeDone = true; }
      if (screen == 1 && !swipeDone && abs(dy) > 8) {
        dragged = true;
        Param &p = params[selParam];
        *p.val -= (y - tyPrev) * p.step * 0.5f;
        if (*p.val < p.lo) *p.val = p.lo; if (*p.val > p.hi) *p.val = p.hi;
      }
      tyPrev = y;
    }
    if (!down && fingerDown) {                       // lifted
      if (screen == 1 && !dragged && !swipeDone && now - tDown < 400) selParam = (selParam + 1) % N_PARAMS;
    }
    fingerDown = down;
  }

  // ---- sensors / network ----
  readShake();
  bool shaking = shake > 1.5f;              // acc is in m/s^2; rest jitter is well under 1
  if (wifiWanted) oscReceive();
  if (now > extCurrentUntil) extCurrent = 0;

  // ---- model ----
  uint32_t elapsed = now - lastFrame; if (elapsed > 50) elapsed = 50; lastFrame = now;
  int steps = (int)(elapsed / DT);
  float sigma = shaking ? SHAKE_GAIN * shake * 0.3f : 0.0f; if (sigma > 3.0f) sigma = 3.0f;
  spikeThisFrame = false;
  for (int s = 0; s < steps; s++) {
    float iIn = extCurrent;
    if (fingerDown && screen == 0 && !swipeDone) iIn += I_HOLD;
    if (tModel < tapUntil) iIn += I_TAP;
    if (sigma > 0) iIn += sigma * gaussNoise();
    stepModel(iIn);
  }
  flash *= 0.80f;
  if (now - lastBat > 2000) { lastBat = now; batteryV = readBattery(); }
  if (wifiWanted) { oscSend("/neuron/v", v > V_THRESH ? 20.0f : v); if (spikeThisFrame) oscSend("/neuron/spike", 1.0f); }

  if (screen == 0) renderHome(batteryV, shaking); else renderParams(batteryV);
  delay(16);
}
