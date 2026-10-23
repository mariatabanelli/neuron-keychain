/*
  Neuron keychain, firmware v0.1
  Board : Waveshare ESP32-S3-Touch-LCD-1.28 (GC9A01 round 240x240, CST816S touch)
  Core  : esp32 by Espressif 3.1.x   Libs: GFX Library for Arduino (moononournation)
  Settings: ESP32S3 Dev Module, USB CDC On Boot Enabled, Flash 16MB, PSRAM QSPI, Partition 16M (3MB APP)

  A leaky integrate-and-fire neuron lives on the ring.
    - touch the glass : injects current while your finger is down (hold = steady train)
    - tap             : a short pulse (one spike if it is strong enough)
  The membrane voltage is drawn as a trace running round the ring; the dashed circle is threshold.
  Same equations as lif_neuron.py in the repo.
*/

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "CST816S.h"

// ---------------- pins ----------------
#define PIN_BL     2
#define PIN_DC     8
#define PIN_CS     9
#define PIN_SCK    10
#define PIN_MOSI   11
#define PIN_MISO   12
#define PIN_RST    14
#define TP_SDA     6
#define TP_SCL     7
#define TP_RST     13
#define TP_IRQ     5
#define PIN_VBAT   1     // battery divider -> ADC

// ---------------- display ----------------
Arduino_DataBus *bus = new Arduino_ESP32SPI(PIN_DC, PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);
Arduino_GFX *panel = new Arduino_GC9A01(bus, PIN_RST, 0, true);
Arduino_Canvas *gfx = new Arduino_Canvas(240, 240, panel);   // off-screen buffer in PSRAM, flicker-free

CST816S touch(TP_SDA, TP_SCL, TP_RST, TP_IRQ);

// ---------------- neuron (same numbers as lif_neuron.py) ----------------
float V_REST   = -70.0f;   // mV
float V_THRESH = -55.0f;   // mV
float V_RESET  = -75.0f;   // mV
float TAU_M    = 20.0f;    // ms
float R_M      = 10.0f;    // MOhm
float T_REF    = 2.0f;     // ms
const float DT = 0.1f;     // ms per model step

float I_HOLD   = 2.0f;     // nA injected while finger is down
float I_TAP    = 8.0f;     // nA pulse on a tap
const float TAP_MS = 5.0f;

float v = V_REST;
float refUntil = -1.0f;
float tapUntil = -1.0f;
uint32_t tapShownUntil = 0;   // ms, label only
float tModel = 0.0f;       // model time in ms
uint32_t spikeCount = 0;
float flash = 0.0f;        // spike glow, decays each frame
bool spikePending = false; // make sure the trace records the spike even between samples

// trace ring buffer: one sample per degree around the ring, 2 s of history
#define N_TRACE 360
float trace[N_TRACE];
int traceHead = 0;
const float TRACE_MS_PER_SAMPLE = 2000.0f / N_TRACE;   // 5.56 ms per sample
float sinceSample = 0.0f;

// ---------------- touch state ----------------
bool fingerDown = false;
uint32_t lastTouchPoll = 0;

bool pollFinger() {
  // read the finger-count register directly; if the chip is asleep the read fails = no finger
  Wire.beginTransmission(CST816S_ADDRESS);
  Wire.write(0x02);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom((uint16_t)CST816S_ADDRESS, (uint32_t)1, true) != 1) return false;
  return Wire.read() > 0;
}

void touchDisableAutoSleep() {
  Wire.beginTransmission(CST816S_ADDRESS);
  Wire.write(0xFE); Wire.write(0x01);     // DisAutoSleep
  Wire.endTransmission(true);
}

// ---------------- model ----------------
void stepModel(float iIn) {
  tModel += DT;
  if (tModel < refUntil) {
    v = V_RESET;
  } else {
    float dv = (-(v - V_REST) + R_M * iIn) / TAU_M;
    v += dv * DT;
    if (v >= V_THRESH) {
      v = 20.0f;                    // draw the spike
      spikeCount++;
      flash = 1.0f;
      spikePending = true;
      refUntil = tModel + T_REF;
    }
  }
  sinceSample += DT;
  if (sinceSample >= TRACE_MS_PER_SAMPLE) {
    sinceSample -= TRACE_MS_PER_SAMPLE;
    trace[traceHead] = spikePending ? 20.0f : v;
    spikePending = false;
    traceHead = (traceHead + 1) % N_TRACE;
  }
  if (v > V_THRESH && tModel >= refUntil) v = V_RESET;   // spike drawn, now reset
}

// ---------------- drawing ----------------
const int CX = 120, CY = 120;
const float R_MIN = 62.0f;    // radius for V_RESET
const float R_MAX = 108.0f;   // radius for +20 mV (spike tip)

float vToR(float mv) {
  float t = (mv - V_RESET) / (20.0f - V_RESET);
  if (t < 0) t = 0; if (t > 1) t = 1;
  return R_MIN + t * (R_MAX - R_MIN);
}

void drawDashedCircle(float r, uint16_t col) {
  for (int a = 0; a < 360; a += 6) {
    float a0 = a * DEG_TO_RAD, a1 = (a + 3) * DEG_TO_RAD;
    gfx->drawLine(CX + cosf(a0) * r, CY + sinf(a0) * r, CX + cosf(a1) * r, CY + sinf(a1) * r, col);
  }
}

void render(float batteryV) {
  gfx->fillScreen(RGB565_BLACK);

  // spike glow
  if (flash > 0.01f) {
    uint8_t g = (uint8_t)(flash * 90);
    gfx->fillCircle(CX, CY, 118, gfx->color565(g, g / 3, g));
    gfx->fillCircle(CX, CY, 112, RGB565_BLACK);
  }

  // rest and threshold circles
  gfx->drawCircle(CX, CY, (int)vToR(V_REST), gfx->color565(40, 40, 40));
  drawDashedCircle(vToR(V_THRESH), gfx->color565(110, 110, 110));

  // membrane trace: oldest sample at 12 o'clock going clockwise, newest just before it
  int prevX = 0, prevY = 0;
  for (int i = 0; i < N_TRACE; i++) {
    int idx = (traceHead + i) % N_TRACE;
    float ang = (-90.0f + i) * DEG_TO_RAD;
    float r = vToR(trace[idx]);
    int x = CX + cosf(ang) * r, y = CY + sinf(ang) * r;
    if (i > 0) {
      uint8_t b = 60 + (uint8_t)(195.0f * i / N_TRACE);      // fade in toward newest
      uint16_t col = (trace[idx] > V_THRESH) ? gfx->color565(255, 120, 255) : gfx->color565(b, b, b);
      gfx->drawLine(prevX, prevY, x, y, col);
    }
    prevX = x; prevY = y;
  }

  // centre readouts
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(3);
  char buf[16];
  snprintf(buf, sizeof(buf), "%4.0f", v > V_THRESH ? V_THRESH : v);
  gfx->setCursor(CX - 36, CY - 22); gfx->print(buf);
  gfx->setTextSize(1);
  gfx->setCursor(CX - 8, CY + 6);  gfx->print("mV");
  gfx->setCursor(CX - 30, CY + 24); gfx->print("spikes "); gfx->print(spikeCount);

  // input indicator
  gfx->setCursor(CX - 18, CY + 64);
  if (millis() < tapShownUntil && !fingerDown) gfx->print(" TAP ");
  else if (fingerDown)        gfx->print("HOLD ");
  else                        gfx->print("rest ");

  // battery, small arc of text at top
  gfx->setCursor(CX - 14, 30);
  if (batteryV > 2.5f) { snprintf(buf, sizeof(buf), "%.2fV", batteryV); gfx->print(buf); }
  else                 { gfx->print("USB"); }

  gfx->flush();
}

float readBattery() {
  // Waveshare: 200k/100k divider into GPIO1; 3.3 / 4096 * 3 * raw
  uint32_t acc = 0;
  for (int i = 0; i < 8; i++) acc += analogRead(PIN_VBAT);
  return (acc / 8.0f) * 3.3f / 4096.0f * 3.0f;
}

// ---------------- setup / loop ----------------
uint32_t lastFrame = 0;
float batteryV = 0;

void setup() {
  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  Serial.begin(115200);

  for (int i = 0; i < N_TRACE; i++) trace[i] = V_REST;

  gfx->begin();
  gfx->fillScreen(RGB565_BLACK);
  gfx->flush();

  touch.begin();
  touchDisableAutoSleep();

  analogReadResolution(12);
  lastFrame = millis();
}

void loop() {
  uint32_t now = millis();

  // touch: gesture events for taps, register poll for finger-down
  if (touch.available()) {                    // IRQ fired: finger landed or gesture
    if (!fingerDown) { tapUntil = tModel + TAP_MS; tapShownUntil = now + 300; }
  }
  if (now - lastTouchPoll >= 15) {
    lastTouchPoll = now;
    bool down = pollFinger();
    if (down && !fingerDown) { tapUntil = tModel + TAP_MS; tapShownUntil = now + 300; }   // finger just landed
    fingerDown = down;
  }

  // advance the model in real time: elapsed ms / DT steps
  uint32_t elapsed = now - lastFrame;
  if (elapsed > 50) elapsed = 50;         // cap after a stall
  lastFrame = now;
  int steps = (int)(elapsed / DT);
  for (int s = 0; s < steps; s++) {
    float iIn = 0.0f;
    if (fingerDown) iIn += I_HOLD;
    if (tModel < tapUntil) iIn += I_TAP;
    stepModel(iIn);
  }

  flash *= 0.80f;
  static uint32_t lastBat = 0;
  if (now - lastBat > 2000) { lastBat = now; batteryV = readBattery(); }

  render(batteryV);
  delay(16);   // ~30 fps including draw time
}
