/*
  Neuron keychain, firmware v0.5 (Hodgkin-Huxley model, switchable)
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
#include "brain_bitmap.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"

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

// ---- palette (1-bit look: paper + ink) ----
#define C_PAPER  (gfx->color565(196, 242, 176))   // pale mint
#define C_INK    (gfx->color565(18, 24, 18))      // near black
#define C_INK_LT (gfx->color565(110, 150, 100))   // dotted lines

// ---------------- neuron ----------------
float V_REST = -70.0f, V_THRESH = -55.0f, V_RESET = -75.0f;
float TAU_M = 20.0f, R_M = 10.0f, T_REF = 2.0f;
float I_HOLD = 2.0f, I_TAP = 8.0f, SHAKE_GAIN = 1.0f;
const float DT = 0.1f, TAP_MS = 5.0f;

// Hodgkin-Huxley (squid axon, 1952). Units: mV, ms, uA/cm^2, mS/cm^2. Same equations as hh_neuron.py.
float MODEL = 0.0f;                 // 0 = leaky integrate-and-fire, 1 = Hodgkin-Huxley
float G_NA = 120.0f, G_K = 36.0f, G_L = 0.3f;
const float E_NA = 50.0f, E_K = -77.0f, E_L = -54.4f, C_M = 1.0f;
const float DT_HH = 0.025f;         // ms; HH needs a fine step (spike is ~1 ms wide)
const float HH_GAIN = 5.0f;         // nA on the touch scale -> uA/cm^2 (hold 2 -> 10, tap 8 -> 40)
float hh_m = 0.053f, hh_h = 0.596f, hh_n = 0.318f;
bool hhAbove = false;
#define N_SCOPE 160                 // high-resolution window of the last 16 ms, for the spike shape
float scope[N_SCOPE]; int scopeHead = 0; float sinceScope = 0;

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
  { "Mode",  "",    &MODEL,    0.05f, 0.0f,  1.0f   },
  { "I",     "nA",  &I_HOLD,   0.05f, 0.0f,  6.0f   },
  { "TAU",   "ms",  &TAU_M,    0.5f,  2.0f,  100.0f },
  { "Vth",   "mV",  &V_THRESH, 0.25f, -65.0f, -40.0f },
  { "gNa",   "mS",  &G_NA,     1.0f,  0.0f,  240.0f },
  { "gK",    "mS",  &G_K,      0.5f,  0.0f,  80.0f  },
  { "gL",    "mS",  &G_L,      0.01f, 0.0f,  1.0f   },
  { "Tref",  "ms",  &T_REF,    0.1f,  0.0f,  10.0f  },
};
bool isHH() { return MODEL >= 0.5f; }
const int N_PARAMS = sizeof(params) / sizeof(params[0]);
int selParam = 0;

// ---------------- sleep ----------------
// Deep sleep after IDLE_MS with no touch, shake or OSC. Wake on touch (CST816S IRQ on GPIO 5, active low).
// Spike count and parameters survive in RTC memory.
const uint32_t IDLE_MS = 30000;
const float V_LOW = 3.50f, V_CUTOFF = 3.30f;
RTC_DATA_ATTR uint32_t rtcSpikes = 0;
RTC_DATA_ATTR float rtcParams[8];
RTC_DATA_ATTR bool rtcValid = false;
uint32_t lastActivity = 0;

void touchSetAutoSleep(bool on) {   // 0xFE: 0 = chip may auto-sleep (wakes on touch, raises IRQ), 1 = always on
  Wire.beginTransmission(CST816S_ADDRESS); Wire.write(0xFE); Wire.write(on ? 0x00 : 0x01); Wire.endTransmission(true);
}

void goToSleep() {
  rtcSpikes = spikeCount;
  for (int i = 0; i < N_PARAMS; i++) rtcParams[i] = *params[i].val;
  rtcValid = true;
  gfx->fillScreen(C_PAPER); gfx->flush();
  panel->displayOff();
  digitalWrite(PIN_BL, LOW);
  gpio_hold_en((gpio_num_t)PIN_BL); gpio_deep_sleep_hold_en();      // keep backlight off while asleep
  touchSetAutoSleep(true);
  delay(50);
  rtc_gpio_pullup_en((gpio_num_t)TP_IRQ); rtc_gpio_pulldown_dis((gpio_num_t)TP_IRQ);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)TP_IRQ, 0);
  esp_deep_sleep_start();
}

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
// incoming current history for the EEG strip (one value per received message)
#define N_EXT 96
float extHist[N_EXT]; int extHead = 0; bool extHistValid = false;
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
      memcpy(&extCurrent, &u, 4); extCurrentUntil = millis() + 5000; lastActivity = millis();
      extHist[extHead] = extCurrent; extHead = (extHead + 1) % N_EXT; extHistValid = true;
    } else if (!strcmp(addr, "/neuron/stage")) {
      if (tags[1] == 's') { strncpy(stageLabel, (const char *)arg, 3); stageLabel[3] = 0; }
      else if (tags[1] == 'i') { int s = arg[3]; const char *names[] = {"W", "1", "2", "3", "R"}; strcpy(stageLabel, s >= 0 && s < 5 ? names[s] : "?"); }
    }
    n = udp.parsePacket();
  }
}

// ---------------- model ----------------
static inline float hh_am(float v) { float d = v + 40.0f; return fabsf(d) < 1e-4f ? 1.0f : 0.1f * d / (1.0f - expf(-d / 10.0f)); }
static inline float hh_bm(float v) { return 4.0f * expf(-(v + 65.0f) / 18.0f); }
static inline float hh_ah(float v) { return 0.07f * expf(-(v + 65.0f) / 20.0f); }
static inline float hh_bh(float v) { return 1.0f / (1.0f + expf(-(v + 35.0f) / 10.0f)); }
static inline float hh_an(float v) { float d = v + 55.0f; return fabsf(d) < 1e-4f ? 0.1f : 0.01f * d / (1.0f - expf(-d / 10.0f)); }
static inline float hh_bn(float v) { return 0.125f * expf(-(v + 65.0f) / 80.0f); }

void recordSamples(float dt, float peak) {
  sinceSample += dt;
  if (sinceSample >= TRACE_MS_PER_SAMPLE) {
    sinceSample -= TRACE_MS_PER_SAMPLE;
    trace[traceHead] = spikePending ? peak : v; spikePending = false;
    traceHead = (traceHead + 1) % N_TRACE;
  }
  sinceScope += dt;
  if (sinceScope >= 0.1f) { sinceScope -= 0.1f; scope[scopeHead] = v; scopeHead = (scopeHead + 1) % N_SCOPE; }
}

void stepHH(float iIn) {           // iIn in nA on the touch scale
  float I = iIn * HH_GAIN;
  tModel += DT_HH;
  float i_na = G_NA * hh_m * hh_m * hh_m * hh_h * (v - E_NA);
  float i_k  = G_K * hh_n * hh_n * hh_n * hh_n * (v - E_K);
  float i_l  = G_L * (v - E_L);
  float dv = (I - i_na - i_k - i_l) / C_M;
  float am = hh_am(v), bm = hh_bm(v), ah = hh_ah(v), bh = hh_bh(v), an = hh_an(v), bn = hh_bn(v);
  hh_m += (am * (1 - hh_m) - bm * hh_m) * DT_HH;
  hh_h += (ah * (1 - hh_h) - bh * hh_h) * DT_HH;
  hh_n += (an * (1 - hh_n) - bn * hh_n) * DT_HH;
  v += dv * DT_HH;
  if (v > 100.0f) v = 100.0f; if (v < -120.0f) v = -120.0f;
  if (v > 0.0f && !hhAbove) { hhAbove = true; spikeCount++; flash = 1.0f; spikePending = true; spikeThisFrame = true; }
  if (v < -20.0f) hhAbove = false;
  recordSamples(DT_HH, 45.0f);
}

void stepModel(float iIn) {
  if (isHH()) { stepHH(iIn); return; }
  tModel += DT;
  if (tModel < refUntil) v = V_RESET;
  else {
    v += ((-(v - V_REST) + R_M * iIn) / TAU_M) * DT;
    if (v >= V_THRESH) { v = 20.0f; spikeCount++; flash = 1.0f; spikePending = true; spikeThisFrame = true; refUntil = tModel + T_REF; }
  }
  recordSamples(DT, 20.0f);
  if (v > V_THRESH && tModel >= refUntil) v = V_RESET;
}

// ---------------- drawing ----------------
const int CX = 120, CY = 120;
const float R_MIN = 62.0f, R_MAX = 108.0f;



// stippled line: draws every other pixel along the segment
void dotLine(float x0, float y0, float x1, float y1, uint16_t col, int phase) {
  float dx = x1 - x0, dy = y1 - y0; int n = (int)fmaxf(fabsf(dx), fabsf(dy)); if (n < 1) n = 1;
  for (int i = 0; i <= n; i++) if (((i + phase) & 1) == 0) gfx->drawPixel((int)(x0 + dx * i / n), (int)(y0 + dy * i / n), col);
}
float vToR(float mv) { float lo = isHH() ? -80.0f : V_RESET, hi = isHH() ? 45.0f : 20.0f; float t = (mv - lo) / (hi - lo); if (t < 0) t = 0; if (t > 1) t = 1; return R_MIN + t * (R_MAX - R_MIN); }
void drawDashedCircle(float r, uint16_t col) {   // dotted: one pixel every 3 degrees
  for (int a = 0; a < 360; a += 3) { float a0 = a * DEG_TO_RAD; gfx->drawPixel(CX + cosf(a0) * r, CY + sinf(a0) * r, col); }
}
void drawStatusDots(float batteryV) {
  char buf[16];
  gfx->setTextSize(1); gfx->setTextColor(C_INK_LT);
  gfx->setCursor(CX - 14, 44);
  if (batteryV > 2.5f) { snprintf(buf, sizeof(buf), batteryV < V_LOW ? "%.2fV low" : "%.2fV", batteryV); gfx->setCursor(batteryV < V_LOW ? CX - 26 : CX - 14, 44); gfx->print(buf); } else gfx->print("USB");
  if (wifiWanted) { if (WiFi.status() == WL_CONNECTED) gfx->fillCircle(CX + 30, 47, 2, C_INK); else gfx->drawCircle(CX + 30, 47, 2, C_INK_LT); }
  if (imuOk) gfx->fillCircle(CX - 30, 47, 2, C_INK_LT);
}

void renderHome(float batteryV, bool shaking) {
  bool eegMode = millis() < extCurrentUntil;
  gfx->fillScreen(C_PAPER);

  // spike bloom: halftone ring at the rim, fades with flash
  if (flash > 0.05f) {
    int rr = 110 + (int)(8 * (1 - flash));
    for (int a = 0; a < 360; a += 2) { float a0 = a * DEG_TO_RAD;
      for (int r = rr; r < 118; r += 2) if (((a / 2 + r) & 1) == 0 && (float)random(0, 100) < flash * 100) gfx->drawPixel(CX + cosf(a0) * r, CY + sinf(a0) * r, C_INK); }
  }

  // rest baseline (faint) and threshold (dotted)
  drawDashedCircle(vToR(isHH() ? -55.0f : V_THRESH), C_INK);
  float rRest = vToR(V_REST);

  // membrane trace as a halftone band: radial stipple from rest radius to the voltage, outline dot at the tip
  float pxp = 0, pyp = 0;
  for (int i = 0; i < N_TRACE; i++) {
    int idx = (traceHead + i) % N_TRACE; float ang = (-90.0f + i) * DEG_TO_RAD;
    float r = vToR(trace[idx]);
    float ca = cosf(ang), sa = sinf(ang);
    float x = CX + ca * r, y = CY + sa * r;
    bool spike = trace[idx] > (isHH() ? -20.0f : V_THRESH);
    int phase = (i + (int)(millis() / 120)) & 1;             // slow shimmer in the halftone
    if (spike) gfx->drawLine(CX + ca * rRest, CY + sa * rRest, x, y, C_INK);   // spikes solid
    else if (i > N_TRACE - 300) dotLine(CX + ca * rRest, CY + sa * rRest, x, y, C_INK, phase);  // newer part dense
    else if ((i & 1) == 0) dotLine(CX + ca * rRest, CY + sa * rRest, x, y, C_INK, phase);        // oldest part sparse
    if (i > 0) gfx->drawLine(pxp, pyp, x, y, C_INK);           // outline
    pxp = x; pyp = y;
  }

  char buf[20];
  gfx->setTextColor(C_INK);
  if (!eegMode) {
    gfx->setTextSize(4);
    snprintf(buf, sizeof(buf), "%d", (int)roundf(isHH() ? v : (v > V_THRESH ? V_THRESH : v)));
    gfx->setCursor(CX - 12 * strlen(buf), CY - 26); gfx->print(buf);
    gfx->setTextSize(1);
    gfx->setCursor(CX - 6, CY + 8); gfx->print("mV");
    snprintf(buf, sizeof(buf), "spikes %lu", (unsigned long)spikeCount);
    gfx->setCursor(CX - 3 * strlen(buf), CY + 24); gfx->print(buf);
    if (isHH()) {   // scope: last 16 ms at full resolution, so the spike's real shape is visible
      int sx0 = CX - 40, sw = 80, sy = CY + 50, sh = 16;
      float py = 0;
      for (int i = 0; i < N_SCOPE; i++) {
        float val = scope[(scopeHead + i) % N_SCOPE]; if (val < -80) val = -80; if (val > 45) val = 45;
        float x = sx0 + (float)sw * i / (N_SCOPE - 1), y = sy - ((val + 80.0f) / 125.0f) * sh;
        if (i > 0) gfx->drawLine(sx0 + (float)sw * (i - 1) / (N_SCOPE - 1), py, x, y, C_INK);
        py = y;
      }
      gfx->setCursor(CX + 44, CY + 40); gfx->print("HH");
    }
    gfx->setCursor(CX - 15, isHH() ? CY + 70 : CY + 60);
    if (millis() < tapShownUntil && !fingerDown) gfx->print(" tap ");
    else if (fingerDown) gfx->print(" hold");
    else if (shaking) gfx->print("shake");
  } else {
    // EEG mode: faint dithered brain behind, the recording's current as a strip, label, stage letter
    gfx->drawBitmap(CX - BRAIN_W / 2, CY - BRAIN_H / 2 - 6, BRAIN_BITS, BRAIN_W, BRAIN_H, C_INK_LT);
    int x0 = CX - 48, w = 96, yMid = CY - 14, h = 22;
    gfx->fillRect(x0 - 4, yMid - h / 2 - 3, w + 8, h + 6, C_PAPER);     // clear a window for the strip
    if (extHistValid) {
      float py = 0;
      for (int i = 0; i < N_EXT; i++) {
        int idx = (extHead + i) % N_EXT; float val = extHist[idx]; if (val < 0) val = 0; if (val > 4) val = 4;
        float x = x0 + (float)w * i / (N_EXT - 1), y = yMid + h / 2 - (val / 4.0f) * h;
        if (i > 0) gfx->drawLine(x0 + (float)w * (i - 1) / (N_EXT - 1), py, x, y, C_INK);
        py = y;
      }
    }
    gfx->setTextSize(1);
    snprintf(buf, sizeof(buf), "EEG %.1f", extCurrent);
    gfx->setCursor(CX - 3 * strlen(buf), CY + 4); gfx->print(buf);
    gfx->setTextSize(4);
    gfx->setCursor(CX - 12, CY + 30); gfx->print(stageLabel[0] ? stageLabel : "?");
    gfx->setTextSize(1);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)spikeCount);
    gfx->setCursor(CX - 3 * strlen(buf), CY + 66); gfx->print(buf);
  }

  drawStatusDots(batteryV);
  gfx->flush();
}

void renderParams(float batteryV) {
  gfx->fillScreen(C_PAPER);
  drawDashedCircle(104, C_INK_LT);
  // six ticks around the ring, selected one bright
  for (int i = 0; i < N_PARAMS; i++) {
    float a = (-90.0f + i * (360.0f / N_PARAMS)) * DEG_TO_RAD;
    int x = CX + cosf(a) * 96, y = CY + sinf(a) * 96;
    bool sel = i == selParam;
    if (sel) gfx->fillCircle(x, y, 5, C_INK); else gfx->drawCircle(x, y, 3, C_INK);
    gfx->setTextSize(1); gfx->setTextColor(sel ? C_INK : C_INK_LT);
    gfx->setCursor(x - 3 * strlen(params[i].name), y + (sinf(a) > 0.3f ? 10 : (sinf(a) < -0.3f ? -16 : -4)) + (cosf(a) > 0.5f ? 0 : 0));
    gfx->print(params[i].name);
  }
  Param &p = params[selParam];
  char buf[20];
  gfx->setTextColor(C_INK); gfx->setTextSize(2);
  gfx->setCursor(CX - 6 * strlen(p.name), CY - 40); gfx->print(p.name);
  gfx->setTextSize(4);
  if (p.val == &MODEL) snprintf(buf, sizeof(buf), isHH() ? "HH" : "LIF");
  else snprintf(buf, sizeof(buf), *p.val < 10 && *p.val > -10 ? "%.2f" : "%.1f", *p.val);
  gfx->setCursor(CX - 12 * strlen(buf), CY - 12); gfx->print(buf);
  gfx->setTextSize(1); gfx->setTextColor(C_INK_LT);
  gfx->setCursor(CX - 3 * strlen(p.unit), CY + 26); gfx->print(p.unit);
  gfx->setCursor(CX - 54, CY + 48); gfx->print("drag=set tap=next hold=off");
  if (wifiWanted) {
    gfx->setCursor(CX - 42, CY + 62);
    if (WiFi.status() == WL_CONNECTED) gfx->print(WiFi.localIP().toString()); else gfx->print("wifi connecting...");
  }
  drawStatusDots(batteryV);
  gfx->flush();
}

float readBattery() { uint32_t acc = 0; for (int i = 0; i < 8; i++) acc += analogRead(PIN_VBAT); return (acc / 8.0f) * 3.3f / 4096.0f * 3.0f; }

// ---------------- setup / loop ----------------
uint32_t lastFrame = 0, lastBat = 0; float batteryV = 0;

void setup() {
  gpio_hold_dis((gpio_num_t)PIN_BL);
  pinMode(PIN_BL, OUTPUT); digitalWrite(PIN_BL, HIGH);
  Serial.begin(115200);
  if (rtcValid) { spikeCount = rtcSpikes; for (int i = 0; i < N_PARAMS; i++) *params[i].val = rtcParams[i]; }
  for (int i = 0; i < N_TRACE; i++) trace[i] = V_REST;
  for (int i = 0; i < N_SCOPE; i++) scope[i] = -65.0f;
  gfx->begin(); gfx->fillScreen(RGB565_BLACK); gfx->flush();
  touch.begin(); touchDisableAutoSleep();
  imuOk = QMI8658_init() != 0;
  if (imuOk) { float a[3], g[3]; unsigned int tc; QMI8658_read_xyz(a, g, &tc); for (int i = 0; i < 3; i++) accPrev[i] = a[i]; }
  analogReadResolution(12);
  if (wifiWanted) { WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS); udp.begin(OSC_IN_PORT); }
  randomSeed(micros());
  lastFrame = millis();
  lastActivity = millis();
}

void loop() {
  uint32_t now = millis();

  // ---- touch ----
  if (touch.available()) { if (!fingerDown && screen == 0) { tapUntil = tModel + TAP_MS; tapShownUntil = now + 300; } }
  if (now - lastTouchPoll >= 15) {
    lastTouchPoll = now;
    bool down; int x, y;
    pollTouch(down, x, y);
    if (down) lastActivity = now;
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
  bool shaking = shake > 150.0f; if (shaking) lastActivity = now;            // measured: rest ~8, vigorous shake ~2000 (raw IMU scale, ~1340 per g)
  if (wifiWanted) {
    static bool announced = false;
    if (WiFi.status() == WL_CONNECTED && !announced) { announced = true; Serial.print("IP "); Serial.println(WiFi.localIP()); }
    oscReceive();
  }
  if (now > extCurrentUntil) extCurrent = 0;

  // ---- model ----
  uint32_t elapsed = now - lastFrame; if (elapsed > 50) elapsed = 50; lastFrame = now;
  int steps = (int)(elapsed / (isHH() ? DT_HH : DT));
  float sigma = shaking ? SHAKE_GAIN * shake / 600.0f : 0.0f; if (sigma > 3.0f) sigma = 3.0f;   // 600 -> 1 nA, 1800+ -> 3 nA
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
  if (screen == 1 && fingerDown && !dragged && !swipeDone && now - tDown > 3000) goToSleep();
  if (now - lastActivity > IDLE_MS) goToSleep();
  if (batteryV > 2.5f && batteryV < V_CUTOFF && now > 10000) goToSleep();
  if (wifiWanted) { oscSend("/neuron/v", isHH() ? v : (v > V_THRESH ? 20.0f : v)); if (spikeThisFrame) oscSend("/neuron/spike", 1.0f); }

  if (screen == 0) renderHome(batteryV, shaking); else renderParams(batteryV);
  delay(16);
}
