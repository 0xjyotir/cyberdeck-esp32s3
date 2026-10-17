/*
  Integrated Cyberdeck firmware — the deck driven from its own 2.8" TFT
  ================================================================
  ESP32-S3 N16R8 + 2.8" ILI9341 + XPT2046 touch.
  For learning and authorized testing only: audit networks and devices
  you own or have permission to test.

  The deck's Wi-Fi (1.1-1.6) and BLE (2.1-2.6) modules, picked and read
  on the on-deck touchscreen. The module code is lifted verbatim from
  firmware/unified_firmware so it behaves identically, and the locked
  serial protocol is untouched, so the USB Web Serial dashboard can drive
  the same modules at the same time (the screen follows it).

  Tap a subsystem -> tap a module -> START. Results stream onto the screen
  the same way they fill the dashboard table (SSID/ch/RSSI for Wi-Fi,
  name/UUID/RSSI for BLE). Status and error lines ("module active",
  "connect failed", ...) show at the bottom. HALT stops the module.

  nRF24 / RFID / IR: NOT in this firmware yet. Their tiles are shown as
  "Not on deck" and open a short note. To use those modules today, flash
  firmware/unified_firmware (USB dashboard only). Porting them here needs
  their pins moved off the display's pins first (the display is locked).

  UI: dark "industrial" theme (graphite panels, one accent per subsystem,
  FreeSans type from Adafruit GFX's bundled Fonts/ folder).

  ---- Libraries (Arduino Library Manager) ----
    Adafruit ILI9341 (+ Adafruit GFX, Adafruit BusIO), ArduinoJson v7.x.
    (ESP32 BLE + Wi-Fi come with the ESP32 Arduino core.)
  ---- Board ----
    ESP32S3 Dev Module, Flash 16MB, PSRAM "OPI PSRAM", USB CDC On Boot Enabled.

  ---- Wiring (same pins as the working display test) ----
    SCK  12   (shared SPI bus)      LED   7  (or tie to 3V3)
    MOSI 11   (shared SPI bus)      T_CS  4
    MISO 13   (shared SPI bus)      T_IRQ 5
    CS   10   DC 9    RST 8         VCC  3V3
  Touch gets calibrated on first boot (5 crosshairs); saved to flash.
*/

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <BLEAdvertising.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Fonts/FreeSansBold9pt7b.h>   // bundled with Adafruit GFX
#include <Fonts/FreeSans9pt7b.h>

// ===================================================================
// PIN MAP
// ===================================================================
// Display + touch: the exact pins from the working display test.
// Data bus (12/11/13) is shared SPI; the display has its own CS.
// Kept as-is on purpose. No other subsystems are wired to this board.
#define TFT_SCLK   12
#define TFT_MOSI   11
#define TFT_MISO   13
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TFT_BL     7     // backlight; tie to 3V3 and set -1 if you don't wire it
#define TOUCH_CS   4
#define TOUCH_IRQ  5     // set to -1 if T_IRQ is not wired

// ===================================================================
// TOUCH TUNING
// ===================================================================
#define TOUCH_Z_PRESS    250
#define TOUCH_Z_RELEASE  120
#define TOUCH_POLL_MS      5
#define HIT_SLOP           4
#define HOLD_SLOP         14

// ===================================================================
// TYPES (all above the first function, for the Arduino IDE)
// ===================================================================
struct Rect { int16_t x, y, w, h; };

enum Screen : uint8_t { SCR_HOME, SCR_MODULES, SCR_LIVE, SCR_SOON, SCR_SETUP, SCR_CALIB, SCR_CALIB_DONE, SCR_TEST,
                        SCR_SETTINGS };
enum Style  : uint8_t { ST_GRID, ST_HEADER, ST_PILL, ST_ROW, ST_ARROW, ST_BACK, ST_STEP, ST_SLIDER, ST_SEG };
enum Action : uint8_t { ACT_OPEN_SUB, ACT_OPEN_MOD, ACT_OPEN_SETUP, ACT_BACK, ACT_TOGGLE,
                        ACT_CALIBRATE, ACT_TEST, ACT_CLEAR, ACT_CAL_RETRY, ACT_CAL_SAVE,
                        ACT_OPEN_SETTINGS, ACT_SCROLL, ACT_BRIGHT_STEP, ACT_BRIGHT_SET, ACT_THEME };

struct Button {
  Rect r; const char* label; const char* sub;
  uint16_t color; uint8_t style; uint8_t action; int8_t arg;
};

struct ModInfo { const char* id; const char* name; };
// One slot of the home grid. label == nullptr is an empty (reserved) slot.
struct HomeItem { const char* label; uint8_t icon; uint8_t action; int8_t arg; };
struct Subsystem { const char* name; uint16_t color; const ModInfo* mods; uint8_t count; bool live; };

// ---- touch ----
struct TouchRaw  { int16_t x, y; uint16_t z, zEnd, spread; };
struct TouchCal  { float a, b, c, d, e, f; };     // screenX = a*rx+b*ry+c ; screenY = d*rx+e*ry+f
struct TouchCfg  { uint16_t zPress, zRelease, maxSpread; uint8_t confirmSamples, releaseSamples;
                   uint16_t jumpRaw, agreeRaw, settleRaw; uint8_t moveThreshold; int16_t width, height; };
enum  TouchEvType : uint8_t { EV_NONE, EV_DOWN, EV_MOVE, EV_UP };
struct TouchEvent { TouchEvType type; int16_t x, y, rawX, rawY; uint16_t z; bool pressed, sampleOk; };
enum  TouchState  : uint8_t { TS_IDLE, TS_CONFIRM, TS_PRESSED };
struct TouchTracker { TouchCfg cfg; TouchCal cal; TouchState state; uint8_t count, release;
                      int16_t rx[4], ry[4]; uint8_t rn, ri; int16_t x, y, lastEmitX, lastEmitY;
                      bool jumpPending; int16_t jumpX, jumpY; };
struct SavedCal { uint32_t magic; TouchCal cal; float maxErr; };

// ---- on-screen result rows ----
struct Row { char a[26]; char b[26]; uint16_t color; };

// ===================================================================
// THEME: dark industrial (graphite surfaces, one accent per subsystem)
//   Surfaces step up in brightness: BG < PANEL < RAISED. Lines use EDGE.
//   Text has three levels: TEXT (primary), MUTED (labels), DIM (hints).
//   Accents are only for icons, active borders and live data.
// ===================================================================
#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
// Dark is the default; applyTheme() swaps every colour below for the light
// set (Settings > Appearance), so these are variables, not constants.
uint16_t C_BG     = RGB565(14, 16, 20);     // #0E1014 screen
uint16_t C_PANEL  = RGB565(25, 28, 34);     // #191C22 bars, tiles
uint16_t C_RAISED = RGB565(36, 40, 48);     // #242830 pressed tile
uint16_t C_EDGE   = RGB565(46, 51, 60);     // #2E333C hairlines
uint16_t C_CYAN   = RGB565(56, 189, 248);   // #38BDF8 brand accent
uint16_t C_TEXT   = RGB565(230, 232, 236);  // #E6E8EC
uint16_t C_MUTED  = RGB565(139, 146, 156);  // #8B929C
uint16_t C_DIM    = RGB565(90, 97, 107);    // #5A616B
uint16_t C_OK     = RGB565(74, 222, 128);   // #4ADE80
uint16_t C_WARN   = RGB565(245, 165, 36);   // #F5A524
uint16_t C_ERR    = RGB565(240, 82, 82);    // #F05252
uint16_t C_WIFI   = RGB565(56, 189, 248);   // #38BDF8
uint16_t C_BLE    = RGB565(139, 124, 246);  // #8B7CF6
uint16_t C_NRF    = RGB565(245, 158, 11);   // #F59E0B
uint16_t C_RFID   = RGB565(52, 211, 153);   // #34D399
uint16_t C_IR     = RGB565(251, 113, 133);  // #FB7185

const int16_t W = 320, H = 240, HEADER_H = 34, FOOTER_H = 16;

// Home-screen tile icons (first five = subsystem index)
enum Icon : uint8_t { IC_WIFI, IC_BLE, IC_NRF, IC_RFID, IC_IR, IC_SETUP, IC_TOUCH };

// Home grid: 3 columns x 3 visible rows. More items than 9 scroll a row at a
// time with the thin arrows on the right.
const uint8_t GRID_COLS = 3, GRID_ROWS = 3;
const HomeItem HOME_ITEMS[] = {
  {"Wi-Fi",    IC_WIFI,  ACT_OPEN_SUB,      0},
  {"BLE",      IC_BLE,   ACT_OPEN_SUB,      1},
  {"nRF24",    IC_NRF,   ACT_OPEN_SUB,      2},
  {"RFID",     IC_RFID,  ACT_OPEN_SUB,      3},
  {"IR",       IC_IR,    ACT_OPEN_SUB,      4},
  {"Touch",    IC_TOUCH, ACT_OPEN_SETUP,    0},
  {"Settings", IC_SETUP, ACT_OPEN_SETTINGS, 0},
  {nullptr,    0,        0,                 0},   // reserved
  {nullptr,    0,        0,                 0},   // reserved
};
const uint8_t HOME_COUNT = sizeof(HOME_ITEMS) / sizeof(HOME_ITEMS[0]);
const uint8_t BRIGHT_MAX = 10;

// ===================================================================
// MODULE METADATA (names match the dashboard TITLES)
// ===================================================================
const ModInfo WIFI_MODS[] = {
  {"1.1", "Active scan"}, {"1.2", "Wardriving"},   {"1.3", "Sniffer"},
  {"1.4", "Discovery"},   {"1.5", "AP hosting"},   {"1.6", "Frame lab"},
};
const ModInfo BLE_MODS[] = {
  {"2.1", "Scan"},        {"2.2", "GATT enum"},    {"2.3", "Adv sniff"},
  {"2.4", "Beacon"},      {"2.5", "GATT read"},    {"2.6", "Conn test"},
};
const ModInfo NRF_MODS[] = {
  {"4.1", "Spectrum"},    {"4.2", "Sniffer"},      {"4.3", "Addr find"},
  {"4.4", "Protocol"},    {"4.5", "Storage"},      {"4.6", "Lab TX"},
};
const ModInfo RFID_MODS[] = {
  {"5.1", "Fingerprint"}, {"5.2", "Mem dump"},     {"5.3", "Key audit"},
  {"5.4", "Clone"},       {"5.5", "NDEF"},         {"5.6", "Monitor"},
};
const ModInfo IR_MODS[] = {
  {"7.1", "Carrier"},     {"7.2", "Learn"},        {"7.3", "Replay"},
  {"7.4", "Decode"},      {"7.5", "Analyse"},      {"7.6", "LED test"},
};
Subsystem SUBS[] = {   // colours refreshed by applyTheme()
  {"Wi-Fi", C_WIFI, WIFI_MODS, 6, true},
  {"BLE",   C_BLE,  BLE_MODS,  6, true},
  {"nRF24", C_NRF,  NRF_MODS,  6, false},
  {"RFID",  C_RFID, RFID_MODS, 6, false},
  {"IR",    C_IR,   IR_MODS,   6, false},
};
const uint8_t SUB_COUNT = 5;

// Longer names for the live-view header (from the dashboard TITLES).
const char* moduleTitle(const char* id) {
  struct { const char* id; const char* t; } T[] = {
    {"1.1","Active Wi-Fi scan"}, {"1.2","Wardriving logger"}, {"1.3","Promiscuous sniffer"},
    {"1.4","Network discovery"}, {"1.5","AP hosting"}, {"1.6","Frame lab"},
    {"2.1","BLE scan"}, {"2.2","GATT enumeration"}, {"2.3","Advertisement sniffer"},
    {"2.4","Beacon broadcaster"}, {"2.5","GATT read"}, {"2.6","Connection test"},
  };
  for (auto& e : T) if (!strcmp(e.id, id)) return e.t;
  return id;
}

// ===================================================================
// FIRMWARE GLOBALS (from unified_firmware — Wi-Fi + BLE subset)
// ===================================================================
String activeModule = "NONE";
unsigned long moduleActionMillis = 0;
unsigned long moduleStateMillis = 0;
volatile unsigned long packetCount = 0;

BLEScan* pBLEScan = nullptr;
BLEAdvertising* pAdvertising = nullptr;
String bleTargetAddr = "";
bool bleTargetFound = false;
int bleTestAttempts = 0;
static const int BLE_TEST_MAX_ATTEMPTS = 5;

enum WifiScanState { WSCAN_IDLE, WSCAN_RUNNING };
WifiScanState wifiScanState = WSCAN_IDLE;

JsonDocument telemetryDoc;

// ===================================================================
// UI / DISPLAY GLOBALS
// ===================================================================
Adafruit_ILI9341 tft(&SPI, TFT_DC, TFT_CS, TFT_RST);
Preferences prefs;

Screen screen = SCR_HOME;
int8_t curSub = 0, curMod = 0;
bool running = false;
uint8_t homeScroll = 0;            // first visible grid row
uint8_t brightness = 8;            // 1..BRIGHT_MAX, saved in Preferences "cdui"
bool lightMode = false;            // saved in Preferences "cdui"

const uint8_t MAX_BTNS = 16;
Button btns[MAX_BTNS];
uint8_t btnCount = 0;
int8_t pressedBtn = -1;
bool pressedShown = false;
bool ignoreUntilRelease = false;

// live-view result ring
const uint8_t ROWS = 9;
Row rowBuf[ROWS];
uint8_t rowCount = 0, rowHead = 0;
bool listDirty = false;
char statusLine[48] = "";
uint16_t statusColor = C_MUTED;
bool statusDirty = false;
unsigned long lastListDraw = 0;
unsigned long liveStartMs = 0;
unsigned long rowTotal = 0;

// touch
TouchTracker trk;
bool irqGate = false;
unsigned long lastIrqCheckMs = 0;
TouchCal activeCal, candidateCal;
bool haveSavedCal = false;
float savedErr = -1, candidateErr = 0;
Screen calReturn = SCR_HOME;
uint8_t calStep = 0;
float calRawX[5], calRawY[5];
uint16_t calBufX[24], calBufY[24];
uint8_t calCount = 0, calHead = 0;
const int16_t CAL_X[5] = {32, 288, 288, 32, 160};
const int16_t CAL_Y[5] = {32, 32, 208, 208, 120};
const uint32_t CAL_MAGIC = 0xCDA11B03;
const TouchCal DEFAULT_CAL = {320.0f / 3500, 0, -300.0f * 320 / 3500,
                              0, 240.0f / 3500, -300.0f * 240 / 3500};
TouchEvent lastEv;
uint32_t lastPollMs = 0;

// ===================================================================
// TOUCH: filtering, tracking, calibration maths
//   8 samples/axis with outliers dropped + a noise check; pressure read
//   before and after each burst (landing/lift samples discarded);
//   press/release hysteresis; DOWN/MOVE/UP events. Why not the stock
//   library: it is fixed at z>=400 (misses light taps) and averages 3.
// ===================================================================
void sortU16(uint16_t* v, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) { uint16_t k = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; } v[j + 1] = k; }
}
uint16_t trimmedMean(uint16_t* v, uint8_t n, uint16_t* spread) {
  sortU16(v, n);
  uint8_t lo = n / 4, hi = n - n / 4; uint32_t sum = 0;
  for (uint8_t i = lo; i < hi; i++) sum += v[i];
  if (spread) *spread = v[hi - 1] - v[lo];
  return (uint16_t)((sum + (hi - lo) / 2) / (hi - lo));
}
float medianU16(const uint16_t* v, uint8_t n) {
  uint16_t t[24]; if (n > 24) n = 24;
  for (uint8_t i = 0; i < n; i++) t[i] = v[i];
  sortU16(t, n);
  return (n & 1) ? t[n / 2] : (t[n / 2 - 1] + t[n / 2]) / 2.0f;
}
void calMap(const TouchCal& k, float rx, float ry, float& sx, float& sy) {
  sx = k.a * rx + k.b * ry + k.c; sy = k.d * rx + k.e * ry + k.f;
}
double det3(const double m[3][3]) {
  return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
       + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
}
bool solve3(const double m[3][3], const double r[3], double s[3]) {
  double d = det3(m); if (!(fabs(d) > 1e-6)) return false;
  for (int col = 0; col < 3; col++) { double t[3][3];
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[i][j] = (j == col) ? r[i] : m[i][j];
    s[col] = det3(t) / d; }
  return true;
}
bool fitAffine(const float* rx, const float* ry, const float* sx, const float* sy,
               uint8_t n, TouchCal& out, float* maxErr) {
  if (n < 3) return false;
  double mx = 0, my = 0; for (uint8_t i = 0; i < n; i++) { mx += rx[i]; my += ry[i]; } mx /= n; my /= n;
  double m[3][3] = {{0,0,0},{0,0,0},{0,0,0}}, bx[3] = {0,0,0}, by[3] = {0,0,0};
  for (uint8_t i = 0; i < n; i++) { double v[3] = {rx[i]-mx, ry[i]-my, 1.0};
    for (int r = 0; r < 3; r++) { for (int c = 0; c < 3; c++) m[r][c] += v[r]*v[c];
      bx[r] += v[r]*sx[i]; by[r] += v[r]*sy[i]; } }
  double px[3], py[3]; if (!solve3(m, bx, px) || !solve3(m, by, py)) return false;
  out.a = px[0]; out.b = px[1]; out.c = px[2] - px[0]*mx - px[1]*my;
  out.d = py[0]; out.e = py[1]; out.f = py[2] - py[0]*mx - py[1]*my;
  if (maxErr) { float worst = 0;
    for (uint8_t i = 0; i < n; i++) { float X, Y; calMap(out, rx[i], ry[i], X, Y);
      float e = sqrtf((X-sx[i])*(X-sx[i]) + (Y-sy[i])*(Y-sy[i])); if (e > worst) worst = e; }
    *maxErr = worst; }
  return true;
}
void trackerInit(TouchTracker& t, const TouchCal& cal) {
  memset(&t, 0, sizeof(t));
  t.cfg = {TOUCH_Z_PRESS, TOUCH_Z_RELEASE, 60, 2, 2, 400, 150, 60, 2, W, H};
  t.cal = cal; t.state = TS_IDLE;
}
void ringReset(TouchTracker& t) { t.rn = 0; t.ri = 0; }
void ringPush(TouchTracker& t, int16_t x, int16_t y) {
  t.rx[t.ri] = x; t.ry[t.ri] = y; t.ri = (t.ri + 1) % 4; if (t.rn < 4) t.rn++;
}
void ringAvg(const TouchTracker& t, float& ax, float& ay) {
  long sx = 0, sy = 0; for (uint8_t i = 0; i < t.rn; i++) { sx += t.rx[i]; sy += t.ry[i]; }
  ax = (float)sx / t.rn; ay = (float)sy / t.rn;
}
bool ringFar(const TouchTracker& t, int16_t x, int16_t y, uint16_t lim) {
  float ax, ay; ringAvg(t, ax, ay); return fabsf(x - ax) > lim || fabsf(y - ay) > lim;
}
void trackerUpdatePos(TouchTracker& t) {
  float ax, ay, sx, sy; ringAvg(t, ax, ay); calMap(t.cal, ax, ay, sx, sy);
  long X = lroundf(sx), Y = lroundf(sy);
  t.x = (int16_t)(X < 0 ? 0 : (X > t.cfg.width - 1 ? t.cfg.width - 1 : X));
  t.y = (int16_t)(Y < 0 ? 0 : (Y > t.cfg.height - 1 ? t.cfg.height - 1 : Y));
}
TouchEvent trackerUpdate(TouchTracker& t, bool haveSample, const TouchRaw& s) {
  TouchEvent ev; memset(&ev, 0, sizeof(ev)); ev.type = EV_NONE;
  uint16_t thr = (t.state == TS_PRESSED) ? t.cfg.zRelease : t.cfg.zPress;
  bool contact = haveSample && s.z >= thr && s.zEnd >= thr;
  bool ok = contact && s.spread <= t.cfg.maxSpread;
  ev.rawX = s.x; ev.rawY = s.y; ev.z = s.z; ev.sampleOk = ok;
  switch (t.state) {
    case TS_IDLE:
      if (ok) { t.state = TS_CONFIRM; t.count = 1; ringReset(t); ringPush(t, s.x, s.y); }
      break;
    case TS_CONFIRM:
      if (!contact) { t.state = TS_IDLE; break; }
      if (!ok) break;
      if (ringFar(t, s.x, s.y, t.cfg.settleRaw)) { ringReset(t); ringPush(t, s.x, s.y); t.count = 1; break; }
      ringPush(t, s.x, s.y);
      if (++t.count >= t.cfg.confirmSamples) {
        t.state = TS_PRESSED; t.release = 0; t.jumpPending = false;
        trackerUpdatePos(t); t.lastEmitX = t.x; t.lastEmitY = t.y; ev.type = EV_DOWN;
      }
      break;
    case TS_PRESSED:
      if (!contact) { if (++t.release >= t.cfg.releaseSamples) { t.state = TS_IDLE; ev.type = EV_UP; } break; }
      t.release = 0; if (!ok) break;
      if (ringFar(t, s.x, s.y, t.cfg.jumpRaw)) {
        if (t.jumpPending && abs(s.x - t.jumpX) <= t.cfg.agreeRaw && abs(s.y - t.jumpY) <= t.cfg.agreeRaw) {
          ringReset(t); ringPush(t, t.jumpX, t.jumpY); ringPush(t, s.x, s.y); t.jumpPending = false;
        } else { t.jumpPending = true; t.jumpX = s.x; t.jumpY = s.y; break; }
      } else { t.jumpPending = false; ringPush(t, s.x, s.y); }
      trackerUpdatePos(t);
      if (abs(t.x - t.lastEmitX) >= t.cfg.moveThreshold || abs(t.y - t.lastEmitY) >= t.cfg.moveThreshold) {
        t.lastEmitX = t.x; t.lastEmitY = t.y; ev.type = EV_MOVE;
      }
      break;
  }
  ev.x = t.x; ev.y = t.y; ev.pressed = (t.state == TS_PRESSED);
  return ev;
}

// ----- XPT2046 driver on the shared SPI bus -----
void xptStart() { SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); digitalWrite(TOUCH_CS, LOW); }
void xptStop()  { digitalWrite(TOUCH_CS, HIGH); SPI.endTransaction(); }
uint16_t xpt12(uint8_t cmd) { SPI.transfer(cmd); uint16_t v = SPI.transfer16(0); return (v >> 3) & 0x0FFF; }
uint16_t xptPressureRaw() { int z1 = xpt12(0xB1), z2 = xpt12(0xC1); int z = z1 + 4095 - z2; return z < 0 ? 0 : (uint16_t)z; }
uint16_t xptReadPressure() { xptStart(); uint16_t z = xptPressureRaw(); xpt12(0xD0); xptStop(); return z; }
void xptRead(TouchRaw& out) {
  uint16_t xs[8], ys[8];
  xptStart();
  uint16_t zA = xptPressureRaw();
  xpt12(0x91); for (uint8_t i = 0; i < 8; i++) xs[i] = xpt12(0x91);
  xpt12(0xD1); for (uint8_t i = 0; i < 8; i++) ys[i] = xpt12(0xD1);
  uint16_t zB = xptPressureRaw();
  xpt12(0xD0);
  xptStop();
  uint16_t sx, sy;
  out.x = trimmedMean(xs, 8, &sx); out.y = trimmedMean(ys, 8, &sy);
  out.spread = sx > sy ? sx : sy; out.z = zA; out.zEnd = zB;
}
void touchBegin() {
  pinMode(TOUCH_CS, OUTPUT); digitalWrite(TOUCH_CS, HIGH);
  if (TOUCH_IRQ >= 0) pinMode(TOUCH_IRQ, INPUT_PULLUP);
  xptStart(); xpt12(0xD0); xptStop();
  irqGate = (TOUCH_IRQ >= 0);
}
TouchEvent touchPoll(uint32_t nowMs) {
  TouchRaw s; memset(&s, 0, sizeof(s)); bool have = false;
  if (trk.state != TS_IDLE || !irqGate || digitalRead(TOUCH_IRQ) == LOW) {
    xptRead(s); have = true;
  } else if (nowMs - lastIrqCheckMs >= 250) {
    lastIrqCheckMs = nowMs;
    if (xptReadPressure() >= trk.cfg.zPress) { irqGate = false; xptRead(s); have = true; }
  }
  return trackerUpdate(trk, have, s);
}

// ===================================================================
// RESULT ROWS (the on-screen equivalent of a dashboard table row)
// ===================================================================
void rowsClear() { rowCount = 0; rowHead = 0; rowTotal = 0; listDirty = true; }
void pushRow(const char* a, const char* b, uint16_t color) {
  Row& r = rowBuf[rowHead];
  strncpy(r.a, a, sizeof(r.a) - 1); r.a[sizeof(r.a) - 1] = 0;
  strncpy(r.b, b, sizeof(r.b) - 1); r.b[sizeof(r.b) - 1] = 0;
  r.color = color;
  rowHead = (rowHead + 1) % ROWS;
  if (rowCount < ROWS) rowCount++;
  rowTotal++;
  listDirty = true;
}
void setStatus(const char* s, uint16_t color) {
  strncpy(statusLine, s, sizeof(statusLine) - 1); statusLine[sizeof(statusLine) - 1] = 0;
  statusColor = color; statusDirty = true;
}

// Friendly text for the status/error codes the modules emit (matches the dashboard).
const char* friendlyStatus(const char* code) {
  if (!strcmp(code, "connect_failed"))  return "Connect failed - target out of range or busy";
  if (!strcmp(code, "test_complete"))   return "Stress test complete";
  if (!strcmp(code, "malformed_json"))  return "Bad command (malformed JSON)";
  if (!strcmp(code, "unknown_action"))  return "Unknown action";
  return code;
}

// Build a result row from the telemetry just emitted, for the active module.
// Wi-Fi (1.x): ssid / ch / rssi. BLE (2.x): name|addr / uuid|payload / rssi.
void uiOnTelemetry(const char* module, const char* status) {
  if (screen != SCR_LIVE || activeModule != module) return;
  JsonObject d = telemetryDoc["data"];
  bool isErr = !strcmp(status, "error");

  // status/error-only line (skip the active/halted housekeeping)
  if (d["msg"].is<const char*>() || d["msg"].is<String>()) {
    const char* m = d["msg"] | "";
    if (!strcmp(m, "module_active") || !strcmp(m, "module_halted")) return;
    char line[48]; snprintf(line, sizeof(line), "%s: %s", isErr ? "ERR" : "·", friendlyStatus(m));
    setStatus(line, isErr ? C_ERR : C_MUTED);
    return;
  }

  char a[26] = "", b[26] = "";
  if (module[0] == '1') {                         // Wi-Fi
    const char* ssid = d["ssid"] | "<hidden>";
    long ch = d["ch"] | 0; long rssi = d["rssi"] | 0;
    snprintf(a, sizeof(a), "%s", ssid);
    snprintf(b, sizeof(b), "ch%ld   %ld dBm", ch, rssi);
    pushRow(a, b, C_WIFI);
  } else {                                        // BLE
    const char* name = d["name"] | "";
    const char* addr = d["addr"] | "";
    const char* uuid = d["uuid"] | "";
    const char* payload = d["payload"] | "";
    long rssi = d["rssi"] | 0;
    snprintf(a, sizeof(a), "%s", (name[0] && strcmp(name, "Unknown")) ? name : (addr[0] ? addr : "BLE device"));
    if (payload[0])   snprintf(b, sizeof(b), "%ld dBm  %s", rssi, payload);
    else if (uuid[0] && strcmp(uuid, "N/A")) snprintf(b, sizeof(b), "%ld dBm  %s", rssi, uuid);
    else              snprintf(b, sizeof(b), "%ld dBm", rssi);
    pushRow(a, b, isErr ? C_ERR : C_BLE);
  }
}

// Defined further down; referenced before their definitions.
void wifiPacketHandler(void* buf, wifi_promiscuous_pkt_type_t type);
void syncScreenToSerial(const char* module);

// ===================================================================
// TELEMETRY (identical shape to unified_firmware, plus the UI hook)
// ===================================================================
void emitTelemetry(const char* module, const char* status) {
  telemetryDoc["module"] = module;
  telemetryDoc["status"] = status;
  serializeJson(telemetryDoc, Serial);
  Serial.println();
  uiOnTelemetry(module, status);          // feed the screen before clearing
  telemetryDoc.clear();
}
void emitError(const char* module, const char* msg) {
  telemetryDoc["data"]["msg"] = msg;
  emitTelemetry(module, "error");
}

// ===================================================================
// BLE SCAN CALLBACK (verbatim from unified_firmware, 2.x only)
// ===================================================================
class UnifiedBLECallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    if (activeModule == "2.1" || activeModule == "2.3") {
      String name = device.haveName() ? device.getName().c_str() : "Unknown";
      String addr = device.getAddress().toString().c_str();
      int rssi = device.getRSSI();
      String uuid = device.haveServiceUUID() ? device.getServiceUUID().toString().c_str() : "N/A";
      telemetryDoc["data"]["name"] = name;
      telemetryDoc["data"]["addr"] = addr;
      telemetryDoc["data"]["rssi"] = rssi;
      telemetryDoc["data"]["uuid"] = uuid;
      emitTelemetry(activeModule.c_str(), "ok");
    } else if ((activeModule == "2.2" || activeModule == "2.5" || activeModule == "2.6") && !bleTargetFound) {
      if (device.isConnectable()) {
        bleTargetAddr = device.getAddress().toString().c_str();
        bleTargetFound = true;
        if (pBLEScan) pBLEScan->stop();
      }
    }
  }
};

// ===================================================================
// MODULE START / STOP (Wi-Fi + BLE branches, verbatim from firmware)
// ===================================================================
void stopModule();

void startModule(const String& module) {
  if (activeModule != "NONE") stopModule();
  activeModule = module;
  moduleActionMillis = 0; moduleStateMillis = 0;
  bleTargetFound = false; bleTargetAddr = ""; bleTestAttempts = 0;
  wifiScanState = WSCAN_IDLE;

  if (module == "1.3") {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifiPacketHandler);
  } else if (module == "1.5") {
    WiFi.softAP("Cyberdeck_AP", "cyberpass123");
  } else if (module == "2.1" || module == "2.3") {
    pBLEScan->start(0, nullptr, false);
  } else if (module == "2.2" || module == "2.5" || module == "2.6") {
    pBLEScan->start(0, nullptr, false);
  } else if (module == "2.4") {
    if (pAdvertising) pAdvertising->start();
  }

  telemetryDoc["data"]["msg"] = "module_active";
  emitTelemetry(module.c_str(), "ok");
}

void stopModule() {
  if (activeModule == "1.3") {
    esp_wifi_set_promiscuous(false);
    esp_wifi_stop();
  } else if (activeModule == "1.5") {
    WiFi.softAPdisconnect(true);
  } else if (activeModule == "2.1" || activeModule == "2.2" || activeModule == "2.3" ||
             activeModule == "2.5" || activeModule == "2.6") {
    if (pBLEScan) pBLEScan->stop();
  } else if (activeModule == "2.4" && pAdvertising) {
    pAdvertising->stop();
  } else if (activeModule == "1.1" && wifiScanState == WSCAN_RUNNING) {
    WiFi.scanDelete();
    wifiScanState = WSCAN_IDLE;
  }
  telemetryDoc["data"]["msg"] = "module_halted";
  emitTelemetry(activeModule.c_str(), "ok");
  activeModule = "NONE";
}

// ===================================================================
// COMMAND PARSER (so the USB dashboard still drives the board)
// ===================================================================
void handleSerialCommand() {
  if (Serial.available() <= 0) return;
  String incoming = Serial.readStringUntil('\n');
  incoming.trim();
  if (incoming.length() == 0) return;
  JsonDocument cmdDoc;
  DeserializationError err = deserializeJson(cmdDoc, incoming);
  if (err) { emitError("SYS", "malformed_json"); return; }
  const char* action = cmdDoc["action"] | "";
  const char* module = cmdDoc["module"] | "NONE";
  if (strcmp(action, "START") == 0) {
    startModule(String(module));
    syncScreenToSerial(module);        // follow the dashboard on-screen too
  } else if (strcmp(action, "STOP") == 0) {
    stopModule();
  } else {
    emitError("SYS", "unknown_action");
  }
}

// ===================================================================
// MODULE WORK (Wi-Fi 1.x + BLE 2.x loop bodies, verbatim from firmware)
// ===================================================================
void runActiveModule(unsigned long now) {
  if (activeModule == "1.1") {
    if (wifiScanState == WSCAN_IDLE && now - moduleActionMillis >= 2000) {
      WiFi.scanNetworks(true, true);
      wifiScanState = WSCAN_RUNNING;
      moduleActionMillis = now;
    } else if (wifiScanState == WSCAN_RUNNING) {
      int n = WiFi.scanComplete();
      if (n >= 0) {
        for (int i = 0; i < n; ++i) {
          String ssid = WiFi.SSID(i);
          if (ssid == "") ssid = "<Hidden>";
          telemetryDoc["data"]["ssid"] = ssid;
          telemetryDoc["data"]["bssid"] = WiFi.BSSIDstr(i);
          telemetryDoc["data"]["ch"] = WiFi.channel(i);
          telemetryDoc["data"]["rssi"] = WiFi.RSSI(i);
          emitTelemetry("1.1", "ok");
        }
        WiFi.scanDelete();
        wifiScanState = WSCAN_IDLE;
      } else if (n == WIFI_SCAN_FAILED) {
        wifiScanState = WSCAN_IDLE;
      }
    }
  }
  else if (activeModule == "1.2") {
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      uint8_t primaryChannel; wifi_second_chan_t secondChannel;
      esp_wifi_get_channel(&primaryChannel, &secondChannel);
      telemetryDoc["data"]["ssid"] = "Current_Channel_Monitor";
      telemetryDoc["data"]["bssid"] = "ESP32_S3_NODE";
      telemetryDoc["data"]["ch"] = primaryChannel;
      telemetryDoc["data"]["rssi"] = WiFi.RSSI();
      emitTelemetry("1.2", "ok");
    }
  }
  else if (activeModule == "1.3") {
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      telemetryDoc["data"]["ssid"] = "[RAW_PACKET_COUNT]";
      telemetryDoc["data"]["bssid"] = "PROMISC_MODE";
      telemetryDoc["data"]["ch"] = WiFi.channel();
      telemetryDoc["data"]["rssi"] = -(long)(packetCount % 90);
      emitTelemetry("1.3", "ok");
    }
  }
  else if (activeModule == "1.4") {
    if (now - moduleActionMillis >= 2000) {
      moduleActionMillis = now;
      wifi_sta_list_t station;
      esp_wifi_ap_get_sta_list(&station);
      telemetryDoc["data"]["ssid"] = String("Connected_Clients: ") + station.num;
      telemetryDoc["data"]["bssid"] = "AP_CLIENT_SCAN";
      telemetryDoc["data"]["ch"] = 1;
      telemetryDoc["data"]["rssi"] = -50;
      emitTelemetry("1.4", "ok");
    }
  }
  else if (activeModule == "1.5") {
    if (now - moduleActionMillis >= 3000) {
      moduleActionMillis = now;
      IPAddress ip = WiFi.softAPIP();
      telemetryDoc["data"]["ssid"] = String("IP: ") + ip.toString();
      telemetryDoc["data"]["bssid"] = "AP_ACTIVE_HOST";
      telemetryDoc["data"]["ch"] = 1;
      telemetryDoc["data"]["rssi"] = -10;
      emitTelemetry("1.5", "ok");
    }
  }
  else if (activeModule == "1.6") {
    if (now - moduleActionMillis >= 2000) {
      moduleActionMillis = now;
      uint8_t packet[128];
      memset(packet, 0xFF, 32);
      esp_err_t res = esp_wifi_80211_tx(WIFI_IF_STA, packet, sizeof(packet), true);
      telemetryDoc["data"]["ssid"] = String("Frame_Tx_Status: ") + (res == ESP_OK ? "SUCCESS" : "FAIL");
      telemetryDoc["data"]["bssid"] = "INJECTION_LAB";
      telemetryDoc["data"]["ch"] = WiFi.channel();
      telemetryDoc["data"]["rssi"] = -25;
      emitTelemetry("1.6", "ok");
    }
  }
  else if (activeModule == "2.1" || activeModule == "2.3") {
    if (now - moduleActionMillis >= 3000) {
      moduleActionMillis = now;
      pBLEScan->clearResults();
    }
  }
  else if (activeModule == "2.2" || activeModule == "2.5") {
    if (!bleTargetFound) {
      // discovery phase: callback-driven
    } else {
      BLEClient* pClient = BLEDevice::createClient();
      if (pClient->connect(BLEAddress(bleTargetAddr.c_str()))) {
        std::map<std::string, BLERemoteService*>* pServices = pClient->getServices();
        if (pServices != nullptr) {
          for (auto const& [suuid, pService] : *pServices) {
            std::map<std::string, BLERemoteCharacteristic*>* pChars = pService->getCharacteristics();
            if (pChars != nullptr) {
              for (auto const& [cuuid, pChar] : *pChars) {
                telemetryDoc["data"]["name"] = bleTargetAddr;
                telemetryDoc["data"]["uuid"] = pChar->getUUID().toString().c_str();
                telemetryDoc["data"]["rssi"] = -50;
                if (activeModule == "2.2") {
                  telemetryDoc["data"]["payload"] = String("Service: ") + pService->getUUID().toString().c_str();
                } else {
                  String hexVal = "LOCKED";
                  if (pChar->canRead()) {
                    String val = pChar->readValue();
                    hexVal = "";
                    for (unsigned int i = 0; i < val.length(); i++) {
                      char buf[3]; sprintf(buf, "%02X", (uint8_t)val[i]); hexVal += buf;
                    }
                  }
                  telemetryDoc["data"]["payload"] = hexVal;
                }
                emitTelemetry(activeModule.c_str(), "ok");
              }
            }
          }
        }
        pClient->disconnect();
      } else {
        emitError(activeModule.c_str(), "connect_failed");
      }
      bleTargetFound = false;
      pBLEScan->start(0, nullptr, false);
    }
  }
  else if (activeModule == "2.4") {
    if (now - moduleActionMillis >= 1500) {
      moduleActionMillis = now;
      telemetryDoc["data"]["name"] = "Cyberdeck-Beacon";
      telemetryDoc["data"]["uuid"] = "Broadcast Active";
      telemetryDoc["data"]["rssi"] = -10;
      telemetryDoc["data"]["payload"] = "BEACON_FLOOD";
      emitTelemetry("2.4", "ok");
    }
  }
  else if (activeModule == "2.6") {
    if (!bleTargetFound) {
      // discovery phase
    } else if (bleTestAttempts < BLE_TEST_MAX_ATTEMPTS) {
      if (now - moduleActionMillis >= 1000) {
        moduleActionMillis = now;
        bleTestAttempts++;
        BLEClient* client = BLEDevice::createClient();
        unsigned long connStart = millis();
        if (client->connect(BLEAddress(bleTargetAddr.c_str()))) {
          unsigned long latency = millis() - connStart;
          telemetryDoc["data"]["name"] = bleTargetAddr;
          telemetryDoc["data"]["uuid"] = String("Attempt ") + bleTestAttempts;
          telemetryDoc["data"]["rssi"] = -40;
          telemetryDoc["data"]["payload"] = String("SUCCESS: ") + latency + "ms";
          emitTelemetry("2.6", "ok");
          client->disconnect();
        } else {
          telemetryDoc["data"]["name"] = bleTargetAddr;
          telemetryDoc["data"]["uuid"] = String("Attempt ") + bleTestAttempts;
          telemetryDoc["data"]["rssi"] = -99;
          telemetryDoc["data"]["payload"] = "FAILED";
          emitTelemetry("2.6", "error");
        }
      }
    } else {
      telemetryDoc["data"]["msg"] = "test_complete";
      emitTelemetry("2.6", "ok");
      bleTargetFound = false;
      bleTestAttempts = 0;
      activeModule = "NONE";
      running = false;
      setStatus("Stress test complete", C_OK);
    }
  }
}

void wifiPacketHandler(void* buf, wifi_promiscuous_pkt_type_t type) { packetCount++; }

// ===================================================================
// DRAWING
// ===================================================================
// Classic 6x8 font, (x, y) = top-left. Resets any GFX font left set by fontAt().
void printAt(const char* s, int16_t x, int16_t y, uint8_t size, uint16_t color) {
  tft.setFont(nullptr);
  tft.setTextSize(size); tft.setTextColor(color); tft.setCursor(x, y); tft.print(s);
}

// ----- theme (v2) -----
void applyTheme(bool light) {
  lightMode = light;
  if (light) {
    C_BG = RGB565(243, 244, 246);  C_PANEL = RGB565(255, 255, 255); C_RAISED = RGB565(229, 232, 237);
    C_EDGE = RGB565(208, 213, 221); C_TEXT = RGB565(21, 24, 29);    C_MUTED = RGB565(84, 93, 106);
    C_DIM = RGB565(150, 157, 168);  C_CYAN = RGB565(2, 132, 199);   C_OK = RGB565(22, 163, 74);
    C_WARN = RGB565(217, 119, 6);   C_ERR = RGB565(220, 38, 38);
    C_WIFI = RGB565(2, 132, 199);   C_BLE = RGB565(109, 91, 208);   C_NRF = RGB565(217, 119, 6);
    C_RFID = RGB565(5, 150, 105);   C_IR = RGB565(225, 29, 72);
  } else {
    C_BG = RGB565(14, 16, 20);      C_PANEL = RGB565(25, 28, 34);   C_RAISED = RGB565(36, 40, 48);
    C_EDGE = RGB565(46, 51, 60);    C_TEXT = RGB565(230, 232, 236); C_MUTED = RGB565(139, 146, 156);
    C_DIM = RGB565(90, 97, 107);    C_CYAN = RGB565(56, 189, 248);  C_OK = RGB565(74, 222, 128);
    C_WARN = RGB565(245, 165, 36);  C_ERR = RGB565(240, 82, 82);
    C_WIFI = RGB565(56, 189, 248);  C_BLE = RGB565(139, 124, 246);  C_NRF = RGB565(245, 158, 11);
    C_RFID = RGB565(52, 211, 153);  C_IR = RGB565(251, 113, 133);
  }
  const uint16_t acc[SUB_COUNT] = {C_WIFI, C_BLE, C_NRF, C_RFID, C_IR};
  for (uint8_t i = 0; i < SUB_COUNT; i++) SUBS[i].color = acc[i];
  statusColor = C_MUTED;
}
uint16_t iconColor(uint8_t icon) {
  return icon < SUB_COUNT ? SUBS[icon].color : (icon == IC_TOUCH ? C_CYAN : C_MUTED);
}
// Mix two RGB565 colours; t = 0..255 is how much of `a` survives.
uint16_t mix565(uint16_t a, uint16_t b, uint8_t t) {
  uint16_t r = (((a >> 11) & 31) * t + ((b >> 11) & 31) * (255 - t)) / 255;
  uint16_t g = (((a >> 5) & 63) * t + ((b >> 5) & 63) * (255 - t)) / 255;
  uint16_t bl = ((a & 31) * t + (b & 31) * (255 - t)) / 255;
  return (r << 11) | (g << 5) | bl;
}
// FreeSans text. (x, baseline) like GFX fonts; bold picks the weight.
void fontAt(const char* s, int16_t x, int16_t baseline, bool bold, uint16_t color) {
  tft.setFont(bold ? &FreeSansBold9pt7b : &FreeSans9pt7b);
  tft.setTextSize(1); tft.setTextColor(color); tft.setCursor(x, baseline); tft.print(s);
  tft.setFont(nullptr);
}
int16_t fontWidth(const char* s, bool bold) {
  int16_t bx, by; uint16_t bw, bh;
  tft.setFont(bold ? &FreeSansBold9pt7b : &FreeSans9pt7b); tft.setTextSize(1);
  tft.getTextBounds(s, 0, 0, &bx, &by, &bw, &bh);
  tft.setFont(nullptr);
  return bx + bw;
}
// Thick arc around (cx, cy), angles in degrees, 0 = right, 90 = up.
void drawArc(int16_t cx, int16_t cy, int16_t r, int16_t a0, int16_t a1, uint8_t thick, uint16_t c) {
  for (uint8_t t = 0; t < thick; t++)
    for (int16_t a = a0; a <= a1; a += 3) {
      float rad = a * 0.0174533f;
      tft.drawPixel(cx + lroundf(cosf(rad) * (r + t)), cy - lroundf(sinf(rad) * (r + t)), c);
    }
}
void thickLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t c) {
  tft.drawLine(x0, y0, x1, y1, c); tft.drawLine(x0 + 1, y0, x1 + 1, y1, c);
}
// 18x18-ish line icons, centred on (cx, cy); bg = the colour behind them.
void drawIcon(uint8_t icon, int16_t cx, int16_t cy, uint16_t c, uint16_t bg) {
  switch (icon) {
    case IC_WIFI:
      tft.fillCircle(cx, cy + 7, 2, c);
      drawArc(cx, cy + 7, 6, 45, 135, 2, c);
      drawArc(cx, cy + 7, 11, 45, 135, 2, c);
      drawArc(cx, cy + 7, 16, 50, 130, 2, c);
      break;
    case IC_BLE:      // the Bluetooth rune
      thickLine(cx - 1, cy - 9, cx - 1, cy + 9, c);
      thickLine(cx - 1, cy - 9, cx + 4, cy - 4, c);
      thickLine(cx + 4, cy - 4, cx - 6, cy + 5, c);
      thickLine(cx - 1, cy + 9, cx + 4, cy + 4, c);
      thickLine(cx + 4, cy + 4, cx - 6, cy - 5, c);
      break;
    case IC_NRF:      // rising signal bars
      for (uint8_t i = 0; i < 4; i++) {
        int16_t h = 5 + i * 4;
        tft.fillRoundRect(cx - 9 + i * 5, cy + 9 - h, 3, h, 1, c);
      }
      break;
    case IC_RFID:     // card with chip + field waves
      tft.drawRoundRect(cx - 10, cy - 6, 15, 13, 2, c);
      tft.fillRect(cx - 7, cy - 3, 5, 4, c);
      drawArc(cx + 4, cy, 6, -35, 35, 1, c);
      drawArc(cx + 4, cy, 9, -40, 40, 1, c);
      break;
    case IC_IR:       // emitter + beam
      tft.fillRoundRect(cx - 10, cy - 4, 6, 9, 2, c);
      drawArc(cx - 6, cy, 7, -35, 35, 2, c);
      drawArc(cx - 6, cy, 12, -35, 35, 2, c);
      break;
    case IC_TOUCH:    // calibration crosshair
      tft.drawCircle(cx, cy, 7, c); tft.drawCircle(cx, cy, 8, c);
      tft.drawFastHLine(cx - 11, cy, 23, c); tft.drawFastVLine(cx, cy - 11, 23, c);
      tft.fillCircle(cx, cy, 2, c);
      break;
    default:          // IC_SETUP: gear
      for (uint8_t i = 0; i < 8; i++) {
        float a = i * 0.785398f;
        tft.fillCircle(cx + lroundf(cosf(a) * 8), cy + lroundf(sinf(a) * 8), 2, c);
      }
      tft.fillCircle(cx, cy, 7, c);
      tft.fillCircle(cx, cy, 3, bg);
      break;
  }
}
// Rounded status chip with a dot; right-aligned to `right`, vertically centred on cy.
void drawChip(const char* label, int16_t right, int16_t cy, uint16_t c) {
  int16_t w = strlen(label) * 6 + 22, x = right - w;
  tft.fillRoundRect(x, cy - 9, w, 18, 9, mix565(c, C_PANEL, 40));
  tft.drawRoundRect(x, cy - 9, w, 18, 9, mix565(c, C_PANEL, 110));
  tft.fillCircle(x + 9, cy, 3, c);
  printAt(label, x + 16, cy - 3, 1, c);
}
void drawHomeHeader() {
  tft.fillRect(0, 0, W, HEADER_H - 1, C_PANEL);
  tft.drawFastHLine(0, HEADER_H - 1, W, C_EDGE);
  // logo mark: a chip outline with an accent core
  tft.drawRoundRect(10, 8, 18, 18, 4, C_CYAN);
  tft.fillRoundRect(14, 12, 10, 10, 2, C_CYAN);
  fontAt("CYBERDECK", 36, 23, true, C_TEXT);
  int16_t x = 36 + fontWidth("CYBERDECK", true) + 6;
  printAt("MK1", x, 14, 1, C_DIM);
  bool idle = activeModule == "NONE";
  drawChip(idle ? "READY" : activeModule.c_str(), W - 8, HEADER_H / 2, idle ? C_OK : C_WARN);
}
// v2 top bar: Back on the left, title on the right, hairline under it.
void drawTopBar(const char* title, uint16_t accent) {
  tft.fillRect(0, 0, W, HEADER_H - 1, C_PANEL);
  tft.drawFastHLine(0, HEADER_H - 1, W, C_EDGE);
  addButton(0, 0, 84, HEADER_H - 1, "Back", nullptr, accent, ST_BACK, ACT_BACK, 0);
  if (title) fontAt(title, W - 12 - fontWidth(title, true), 22, true, C_TEXT);
}
void drawHomeFooter() {
  int16_t y = H - FOOTER_H;
  tft.fillRect(0, y, W, FOOTER_H, C_PANEL);
  tft.drawFastHLine(0, y, W, C_EDGE);
  tft.fillCircle(10, y + 8, 2, C_OK);
  printAt("Wi-Fi + BLE live", 17, y + 5, 1, C_MUTED);
  printAt("USB 115200  ESP32-S3", W - 8 - 20 * 6, y + 5, 1, C_DIM);
}
void printCentered(const char* s, int16_t cx, int16_t cy, uint8_t size, uint16_t color) {
  printAt(s, cx - (int16_t)strlen(s) * 3 * size, cy - 4 * size, size, color);
}
void drawHeader(const char* title, uint16_t accent, bool left) {
  tft.fillRect(0, 0, W, HEADER_H - 2, C_PANEL);
  tft.fillRect(0, HEADER_H - 2, W, 2, accent);
  if (!title) return;
  int16_t tw = strlen(title) * 12;
  printAt(title, left ? 10 : W - tw - 10, 9, 2, accent);
}
void addButton(int16_t x, int16_t y, int16_t w, int16_t h, const char* label, const char* sub,
               uint16_t color, uint8_t style, uint8_t action, int8_t arg) {
  if (btnCount >= MAX_BTNS) return;
  btns[btnCount++] = {{x, y, w, h}, label, sub, color, style, action, arg};
}
// Home grid geometry: 3 x 3 tiles left of a thin arrow rail, between the
// header and the footer bar. slot = position on screen (0..8).
const int16_t GRID_PAD = 6, GRID_GAP = 5, RAIL_W = 18;
Rect gridRect(uint8_t slot) {
  const int16_t areaW = W - 2 * GRID_PAD - RAIL_W - GRID_GAP;
  const int16_t areaH = H - HEADER_H - FOOTER_H - 2 * GRID_PAD;
  const int16_t tw = (areaW - (GRID_COLS - 1) * GRID_GAP) / GRID_COLS;
  const int16_t th = (areaH - (GRID_ROWS - 1) * GRID_GAP) / GRID_ROWS;
  return {(int16_t)(GRID_PAD + (slot % GRID_COLS) * (tw + GRID_GAP)),
          (int16_t)(HEADER_H + GRID_PAD + (slot / GRID_COLS) * (th + GRID_GAP)), tw, th};
}
Rect railRect(bool down) {
  const int16_t top = HEADER_H + GRID_PAD, h = (H - HEADER_H - FOOTER_H - 2 * GRID_PAD - GRID_GAP) / 2;
  return {(int16_t)(W - GRID_PAD - RAIL_W), (int16_t)(down ? top + h + GRID_GAP : top), RAIL_W, h};
}
uint8_t homeRows() { return (HOME_COUNT + GRID_COLS - 1) / GRID_COLS; }
void drawEmptySlot(const Rect& r) {   // reserved slot: dotted outline
  for (int16_t x = r.x + 6; x < r.x + r.w - 6; x += 4) {
    tft.drawPixel(x, r.y, C_EDGE); tft.drawPixel(x, r.y + r.h - 1, C_EDGE);
  }
  for (int16_t y = r.y + 6; y < r.y + r.h - 6; y += 4) {
    tft.drawPixel(r.x, y, C_EDGE); tft.drawPixel(r.x + r.w - 1, y, C_EDGE);
  }
  tft.drawFastHLine(r.x + r.w / 2 - 4, r.y + r.h / 2, 9, C_EDGE);
  tft.drawFastVLine(r.x + r.w / 2, r.y + r.h / 2 - 4, 9, C_EDGE);
}
void drawChevron(int16_t cx, int16_t cy, int8_t dir, uint16_t c) {   // dir -1 up, +1 down
  for (int8_t t = 0; t < 2; t++) {
    tft.drawLine(cx - 5, cy - 3 * dir + t, cx, cy + 2 * dir + t, c);
    tft.drawLine(cx + 5, cy - 3 * dir + t, cx, cy + 2 * dir + t, c);
  }
}
void drawButton(uint8_t i, bool pressed) {
  const Button& b = btns[i]; uint16_t fg = pressed ? C_BG : b.color; const Rect& r = b.r;
  if (b.style == ST_GRID) {
    // Home tile: icon well on top, name under it, live dot in the corner.
    // Pressed = raised surface + accent outline (no colour flood).
    uint16_t face = pressed ? C_RAISED : C_PANEL;
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, face);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, pressed ? b.color : C_EDGE);
    uint8_t icon = (uint8_t)b.sub[0];          // sub carries the icon id for grid tiles
    bool isSub = b.action == ACT_OPEN_SUB;
    bool live = !isSub || SUBS[b.arg].live;
    const int16_t well = 28, wx = r.x + (r.w - well) / 2, wy = r.y + 5;
    uint16_t wellBg = mix565(b.color, face, live ? 46 : 26);
    tft.fillRoundRect(wx, wy, well, well, 7, wellBg);
    drawIcon(icon, wx + well / 2, wy + well / 2, live ? b.color : mix565(b.color, face, 150), wellBg);
    fontAt(b.label, r.x + (r.w - fontWidth(b.label, true)) / 2, r.y + r.h - 7, true, live ? C_TEXT : C_MUTED);
    if (isSub) tft.fillCircle(r.x + r.w - 8, r.y + 8, 2, live ? C_OK : C_DIM);
  } else if (b.style == ST_HEADER) {
    tft.fillRect(r.x, r.y, r.w, r.h, pressed ? b.color : C_PANEL);
    printCentered(b.label, r.x + r.w / 2, r.y + r.h / 2, 2, fg);
  } else if (b.style == ST_ROW) {
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 5, pressed ? b.color : C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 5, b.color);
    printAt(b.label, r.x + 10, r.y + 6, 1, pressed ? C_BG : b.color);
    if (b.sub) printAt(b.sub, r.x + 44, r.y + 6, 1, pressed ? C_BG : C_TEXT);
  } else if (b.style == ST_ARROW) {   // thin scroll rail half; arg -1 up / +1 down
    bool on = b.color != C_DIM;
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 5, pressed && on ? C_RAISED : C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 5, pressed && on ? C_CYAN : C_EDGE);
    drawChevron(r.x + r.w / 2, r.y + r.h / 2, b.arg, on ? (pressed ? C_CYAN : C_TEXT) : C_EDGE);
  } else if (b.style == ST_BACK) {
    tft.fillRect(r.x, r.y, r.w, r.h, pressed ? C_RAISED : C_PANEL);
    int16_t cy = r.y + r.h / 2;
    for (int8_t t = 0; t < 2; t++) {
      tft.drawLine(14 + t, cy, 19 + t, cy - 5, b.color); tft.drawLine(14 + t, cy, 19 + t, cy + 5, b.color);
    }
    fontAt(b.label, 26, cy + 5, false, b.color);
  } else if (b.style == ST_STEP) {    // square - / + button
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, pressed ? C_RAISED : C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, pressed ? b.color : C_EDGE);
    int16_t cx = r.x + r.w / 2, cy = r.y + r.h / 2;
    tft.fillRect(cx - 6, cy - 1, 13, 3, b.color);
    if (b.arg > 0) tft.fillRect(cx - 1, cy - 6, 3, 13, b.color);
  } else if (b.style == ST_SLIDER) {  // segmented brightness bar
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, pressed ? b.color : C_EDGE);
    const int16_t pad = 6, gap = 3, segW = (r.w - 2 * pad - (BRIGHT_MAX - 1) * gap) / BRIGHT_MAX;
    for (uint8_t k = 0; k < BRIGHT_MAX; k++) {
      int16_t sx = r.x + pad + k * (segW + gap), sh = 8 + k * (r.h - 2 * pad - 8) / (BRIGHT_MAX - 1);
      tft.fillRoundRect(sx, r.y + r.h - pad - sh, segW, sh, 2, k < brightness ? b.color : C_RAISED);
    }
  } else if (b.style == ST_SEG) {     // one half of a segmented control; arg = its value
    bool sel = (b.arg == 1) == lightMode;
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, sel ? b.color : (pressed ? C_RAISED : C_PANEL));
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, sel ? b.color : C_EDGE);
    uint16_t fgc = sel ? (lightMode ? C_PANEL : C_BG) : C_TEXT;
    fontAt(b.label, r.x + (r.w - fontWidth(b.label, true)) / 2, r.y + r.h / 2 + 5, true, fgc);
  } else {  // ST_PILL
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 9, pressed ? b.color : C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 9, b.color);
    tft.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 8, b.color);
    uint8_t size = ((int16_t)strlen(b.label) * 18 <= r.w - 20) ? 3 : 2;
    printCentered(b.label, r.x + r.w / 2, r.y + r.h / 2, size, fg);
  }
}
void drawTarget(uint8_t i, uint8_t mode) {
  int16_t x = CAL_X[i], y = CAL_Y[i];
  tft.fillRect(x - 19, y - 19, 39, 39, C_BG);
  if (mode == 0) return;
  uint16_t c = (mode == 3) ? C_OK : C_CYAN;
  tft.drawCircle(x, y, 12, c);
  tft.drawFastHLine(x - 18, y, 37, c);
  tft.drawFastVLine(x, y - 18, 37, c);
  if (mode >= 2) tft.fillCircle(x, y, 6, c);
}
void calMessage(const char* msg, uint16_t color) {
  tft.fillRect(0, 172, W, 12, C_BG);
  if (msg) printCentered(msg, W / 2, 178, 1, color);
}
void calProgress() {
  char buf[24]; snprintf(buf, sizeof(buf), "Point %u of 5", calStep + 1);
  tft.fillRect(0, 156, W, 12, C_BG);
  printCentered(buf, W / 2, 162, 1, C_TEXT);
}

// ----- the live result view -----
void drawLiveChrome() {
  const Subsystem& s = SUBS[curSub];
  tft.fillScreen(C_BG);
  const char* id = SUBS[curSub].mods[curMod].id;
  const char* title = moduleTitle(id);
  drawHeader(nullptr, s.color, false);
  addButton(0, 0, 90, HEADER_H - 2, "< BACK", nullptr, s.color, ST_HEADER, ACT_BACK, 0);
  // module id + title right of BACK; long titles drop to the small font so
  // they never run into the id or the BACK button
  int16_t tw = strlen(title) * 12;
  if (100 + (int16_t)strlen(id) * 12 + 10 + tw <= W - 10) {
    printAt(id, 100, 9, 2, C_TEXT);
    printAt(title, W - 10 - tw, 9, 2, s.color);
  } else {
    printAt(id, W - 10 - (int16_t)strlen(id) * 6, 6, 1, C_MUTED);
    printAt(title, W - 10 - (int16_t)strlen(title) * 6, 18, 1, s.color);
  }
  // START / HALT pill
  addButton(W - 96, HEADER_H + 4, 88, 30, running ? "HALT" : "START", nullptr,
            running ? C_OK : s.color, ST_PILL, ACT_TOGGLE, 0);
  listDirty = true; statusDirty = true;
}
void drawList() {
  const int16_t x = 6, y0 = HEADER_H + 6, rowH = 18;
  tft.fillRect(x, y0, W - 100, ROWS * rowH, C_BG);
  if (rowCount == 0) {
    printAt(running ? "Scanning..." : "Tap START", x + 4, y0 + 4, 1, C_MUTED);
  } else {
    for (uint8_t i = 0; i < rowCount; i++) {
      // newest first
      uint8_t idx = (rowHead + ROWS - 1 - i) % ROWS;
      int16_t ry = y0 + i * rowH;
      printAt(rowBuf[idx].a, x + 2, ry, 1, rowBuf[idx].color);
      printAt(rowBuf[idx].b, x + 2, ry + 9, 1, C_MUTED);
    }
  }
  // count badge
  char c[20]; snprintf(c, sizeof(c), "%lu seen", rowTotal);
  tft.fillRect(W - 96, HEADER_H + 40, 90, 10, C_BG);
  printAt(c, W - 94, HEADER_H + 40, 1, C_MUTED);
}
void drawStatusLine() {
  tft.fillRect(0, H - 14, W, 14, C_PANEL);
  if (statusLine[0]) printAt(statusLine, 6, H - 12, 1, statusColor);
}

// ===================================================================
// SCREEN BUILDERS
// ===================================================================
void drawScreen() {
  pressedBtn = -1; pressedShown = false; btnCount = 0;

  switch (screen) {
    case SCR_HOME: {
      tft.fillScreen(C_BG);
      drawHomeHeader();
      drawHomeFooter();
      uint8_t rows = homeRows();
      if (homeScroll + GRID_ROWS > rows) homeScroll = rows > GRID_ROWS ? rows - GRID_ROWS : 0;
      for (uint8_t slot = 0; slot < GRID_COLS * GRID_ROWS; slot++) {
        uint8_t i = homeScroll * GRID_COLS + slot;
        if (i >= HOME_COUNT) break;
        Rect r = gridRect(slot);
        const HomeItem& it = HOME_ITEMS[i];
        if (!it.label) { drawEmptySlot(r); continue; }
        // icon id rides in `sub` (first char) so drawButton can draw it
        static char iconTag[HOME_COUNT][2];
        iconTag[i][0] = (char)it.icon; iconTag[i][1] = 0;
        addButton(r.x, r.y, r.w, r.h, it.label, iconTag[i], iconColor(it.icon), ST_GRID, it.action, it.arg);
      }
      bool canUp = homeScroll > 0, canDown = homeScroll + GRID_ROWS < rows;
      Rect u = railRect(false), d = railRect(true);
      addButton(u.x, u.y, u.w, u.h, "", nullptr, canUp ? C_TEXT : C_DIM, ST_ARROW, ACT_SCROLL, -1);
      addButton(d.x, d.y, d.w, d.h, "", nullptr, canDown ? C_TEXT : C_DIM, ST_ARROW, ACT_SCROLL, 1);
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }

    case SCR_MODULES: {
      const Subsystem& s = SUBS[curSub];
      tft.fillScreen(C_BG);
      drawHeader(s.name, s.color, false);
      addButton(0, 0, 90, HEADER_H - 2, "< BACK", nullptr, s.color, ST_HEADER, ACT_BACK, 0);
      const int16_t x = 8, y0 = HEADER_H + 6, rowH = 30, gap = 3;
      for (uint8_t i = 0; i < s.count; i++) {
        int16_t ry = y0 + i * (rowH + gap);
        addButton(x, ry, W - 16, rowH, s.mods[i].id, s.mods[i].name, s.color, ST_ROW, ACT_OPEN_MOD, i);
      }
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }

    case SCR_SOON: {
      const Subsystem& s = SUBS[curSub];
      tft.fillScreen(C_BG);
      drawHeader(s.name, s.color, false);
      addButton(0, 0, 90, HEADER_H - 2, "< BACK", nullptr, s.color, ST_HEADER, ACT_BACK, 0);
      printCentered("Not on the screen yet", W / 2, 96, 2, C_TEXT);
      printCentered("Not in this firmware yet. Flash", W / 2, 128, 1, C_MUTED);
      printCentered("unified_firmware to use it over USB.", W / 2, 142, 1, C_MUTED);
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }

    case SCR_LIVE:
      drawLiveChrome();
      drawList();
      drawStatusLine();
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;

    case SCR_SETUP: {
      tft.fillScreen(C_BG);
      drawHeader("SETUP", C_CYAN, false);
      addButton(0, 0, 90, HEADER_H - 2, "< BACK", nullptr, C_CYAN, ST_HEADER, ACT_BACK, 0);
      addButton(12, 44, W - 24, 48, "RECALIBRATE", nullptr, C_CYAN, ST_PILL, ACT_CALIBRATE, 0);
      addButton(12, 100, W - 24, 48, "TOUCH TEST", nullptr, C_CYAN, ST_PILL, ACT_TEST, 0);
      char line[52];
      if (haveSavedCal) snprintf(line, sizeof(line), "Calibration saved, max error %.1f px", savedErr);
      else snprintf(line, sizeof(line), "Calibration: default (not calibrated)");
      printAt(line, 12, 164, 1, C_TEXT);
      printAt("Wi-Fi + BLE are live on-deck.", 12, 182, 1, C_MUTED);
      printAt("USB Web Serial works at the same time.", 12, 198, 1, C_MUTED);
      printAt("Hold the screen at power-on to recalibrate.", 12, 214, 1, C_MUTED);
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }

    case SCR_SETTINGS: {
      tft.fillScreen(C_BG);
      drawTopBar("Settings", C_CYAN);
      char pct[8]; snprintf(pct, sizeof(pct), "%u%%", brightness * 10);
      fontAt("Brightness", 12, 60, true, C_TEXT);
      printAt(pct, W - 12 - (int16_t)strlen(pct) * 6, 50, 1, C_MUTED);
      addButton(12, 70, 40, 40, "-", nullptr, C_CYAN, ST_STEP, ACT_BRIGHT_STEP, -1);
      addButton(58, 70, W - 116, 40, "", nullptr, C_CYAN, ST_SLIDER, ACT_BRIGHT_SET, 0);
      addButton(W - 52, 70, 40, 40, "+", nullptr, C_CYAN, ST_STEP, ACT_BRIGHT_STEP, 1);
      fontAt("Appearance", 12, 144, true, C_TEXT);
      addButton(12, 154, (W - 30) / 2, 36, "Dark", nullptr, C_CYAN, ST_SEG, ACT_THEME, 0);
      addButton(18 + (W - 30) / 2, 154, (W - 30) / 2, 36, "Light", nullptr, C_CYAN, ST_SEG, ACT_THEME, 1);
      printAt("Saved automatically. More settings coming.", 12, 206, 1, C_DIM);
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }

    case SCR_TEST:
      tft.fillScreen(C_BG);
      drawHeader(nullptr, C_CYAN, false);
      addButton(0, 0, 88, HEADER_H - 2, "< BACK", nullptr, C_CYAN, ST_HEADER, ACT_BACK, 0);
      addButton(W - 88, 0, 88, HEADER_H - 2, "CLEAR", nullptr, C_CYAN, ST_HEADER, ACT_CLEAR, 0);
      printCentered("Touch anywhere - a dot marks where it lands", W / 2, HEADER_H + 10, 1, C_MUTED);
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;

    case SCR_CALIB:
      tft.fillScreen(C_BG);
      printCentered("TOUCH CALIBRATION", W / 2, 66, 2, C_CYAN);
      printCentered("A stylus or fingernail is most accurate", W / 2, 84, 1, C_MUTED);
      printCentered("Press and hold the centre of the cross", W / 2, 148, 1, C_TEXT);
      calProgress();
      drawTarget(calStep, 1);
      return;

    case SCR_CALIB_DONE: {
      tft.fillScreen(C_BG);
      char line[40];
      bool good = candidateErr <= 6, ok = candidateErr <= 12;
      printCentered(good ? "CALIBRATED" : (ok ? "CALIBRATED (OK)" : "CALIBRATION IS OFF"),
                    W / 2, 30, 2, good ? C_OK : (ok ? C_CYAN : C_WARN));
      snprintf(line, sizeof(line), "Max error: %.1f px", candidateErr);
      printCentered(line, W / 2, 60, 2, C_TEXT);
      printCentered(ok ? "Tap SAVE to keep it." : "Retry: press firmly on each centre.", W / 2, 92, 1, C_MUTED);
      addButton(6, 116, 151, 116, "RETRY", nullptr, C_WARN, ST_PILL, ACT_CAL_RETRY, 0);
      addButton(163, 116, 151, 116, "SAVE", nullptr, C_OK, ST_PILL, ACT_CAL_SAVE, 0);
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }
  }
}

void goTo(Screen s) { screen = s; drawScreen(); }

// ===================================================================
// UI SETTINGS (brightness + light/dark), saved in Preferences "cdui"
// ===================================================================
void setBacklight() {
  if (TFT_BL < 0) return;
  // 1..10 -> ~10%..100% PWM; never fully dark so the screen can't be "lost"
  analogWrite(TFT_BL, 25 + (brightness - 1) * 230 / (BRIGHT_MAX - 1));
}
void loadUiPrefs() {
  if (prefs.begin("cdui", true)) {
    brightness = prefs.getUChar("bright", 8);
    lightMode = prefs.getBool("light", false);
    prefs.end();
  }
  if (brightness < 1 || brightness > BRIGHT_MAX) brightness = 8;
}
void saveUiPrefs() {
  if (prefs.begin("cdui", false)) {
    prefs.putUChar("bright", brightness);
    prefs.putBool("light", lightMode);
    prefs.end();
  }
}

// If the USB dashboard starts/stops a module, follow it on the screen.
void syncScreenToSerial(const char* module) {
  for (uint8_t si = 0; si < SUB_COUNT; si++) {
    if (!SUBS[si].live) continue;
    for (uint8_t mi = 0; mi < SUBS[si].count; mi++) {
      if (!strcmp(SUBS[si].mods[mi].id, module)) {
        curSub = si; curMod = mi; running = true; rowsClear();
        setStatus("Started from USB", C_MUTED);
        goTo(SCR_LIVE);
        return;
      }
    }
  }
}

// ===================================================================
// CALIBRATION FLOW
// ===================================================================
void loadCalibration() {
  SavedCal s; size_t n = 0;
  if (prefs.begin("cdtouch", true)) { n = prefs.getBytes("cal", &s, sizeof(s)); prefs.end(); }
  haveSavedCal = (n == sizeof(s) && s.magic == CAL_MAGIC);
  activeCal = haveSavedCal ? s.cal : DEFAULT_CAL;
  savedErr = haveSavedCal ? s.maxErr : -1;
  trk.cal = activeCal;
}
void saveCalibration() {
  SavedCal s; s.magic = CAL_MAGIC; s.cal = candidateCal; s.maxErr = candidateErr;
  if (prefs.begin("cdtouch", false)) { prefs.putBytes("cal", &s, sizeof(s)); prefs.end(); }
  activeCal = candidateCal; savedErr = candidateErr; haveSavedCal = true; trk.cal = activeCal;
}
void startCalibration(Screen returnTo) {
  calReturn = returnTo; calStep = 0; calCount = 0; calHead = 0; goTo(SCR_CALIB);
}
void finishCalibration() {
  float sx[5], sy[5]; for (uint8_t i = 0; i < 5; i++) { sx[i] = CAL_X[i]; sy[i] = CAL_Y[i]; }
  TouchCal k; float err = 0;
  if (!fitAffine(calRawX, calRawY, sx, sy, 5, k, &err)) {
    startCalibration(calReturn); calMessage("Those points didn't work - try again", C_WARN); return;
  }
  candidateCal = k; candidateErr = err; trk.cal = k;
  Serial.printf("Calibration max err %.2f px\n", err);
  goTo(SCR_CALIB_DONE);
}
void calibHandle(const TouchEvent& ev) {
  if (ev.type == EV_DOWN) { calCount = 0; calHead = 0; drawTarget(calStep, 2); }
  if (ev.pressed && ev.sampleOk) {
    calBufX[calHead] = ev.rawX; calBufY[calHead] = ev.rawY;
    calHead = (calHead + 1) % 24; if (calCount < 24) calCount++;
    if (calCount == 10) drawTarget(calStep, 3);
  }
  if (ev.type != EV_UP) return;
  if (calCount < 10) { drawTarget(calStep, 1); calMessage("Hold until the dot turns green", C_WARN); return; }
  float mx = medianU16(calBufX, calCount), my = medianU16(calBufY, calCount);
  for (uint8_t j = 0; j < calStep; j++)
    if (fabsf(mx - calRawX[j]) < 250 && fabsf(my - calRawY[j]) < 250) {
      drawTarget(calStep, 1); calMessage("That matched an earlier point - redo it", C_WARN); return;
    }
  calRawX[calStep] = mx; calRawY[calStep] = my;
  drawTarget(calStep, 0); calStep++;
  if (calStep < 5) { calMessage(nullptr, C_BG); calProgress(); drawTarget(calStep, 1); }
  else finishCalibration();
}

// ===================================================================
// TOUCH -> UI
// ===================================================================
bool inRect(int16_t x, int16_t y, const Rect& r, int16_t slop) {
  return x >= r.x - slop && x < r.x + r.w + slop && y >= r.y - slop && y < r.y + r.h + slop;
}
int8_t hitTest(int16_t x, int16_t y) {
  for (uint8_t i = 0; i < btnCount; i++) if (inRect(x, y, btns[i].r, 0)) return i;
  for (uint8_t i = 0; i < btnCount; i++) if (inRect(x, y, btns[i].r, HIT_SLOP)) return i;
  return -1;
}
void startLiveModule() {
  rowsClear();
  running = true;
  startModule(String(SUBS[curSub].mods[curMod].id));   // the real module
}
void stopLiveModule() {
  if (activeModule != "NONE") stopModule();
  running = false;
}
void doAction(uint8_t i) {
  const Button& b = btns[i];
  switch (b.action) {
    case ACT_OPEN_SUB:
      curSub = b.arg;
      goTo(SUBS[curSub].live ? SCR_MODULES : SCR_SOON);
      break;
    case ACT_OPEN_MOD:
      curMod = b.arg; running = false; rowsClear(); statusLine[0] = 0;
      goTo(SCR_LIVE);
      break;
    case ACT_OPEN_SETUP: goTo(SCR_SETUP); break;
    case ACT_OPEN_SETTINGS: goTo(SCR_SETTINGS); break;
    case ACT_SCROLL: {
      uint8_t rows = homeRows();
      if (b.arg < 0 && homeScroll > 0) { homeScroll--; goTo(SCR_HOME); }
      else if (b.arg > 0 && homeScroll + GRID_ROWS < rows) { homeScroll++; goTo(SCR_HOME); }
      else drawButton(i, false);
      break;
    }
    case ACT_BRIGHT_STEP:
    case ACT_BRIGHT_SET: {
      int16_t v = brightness;
      if (b.action == ACT_BRIGHT_STEP) v += b.arg;
      else v = 1 + (int32_t)(lastEv.x - b.r.x) * BRIGHT_MAX / b.r.w;   // tap position on the bar
      v = constrain(v, 1, BRIGHT_MAX);
      brightness = v; setBacklight(); saveUiPrefs();
      goTo(SCR_SETTINGS);
      break;
    }
    case ACT_THEME:
      if ((b.arg == 1) != lightMode) { applyTheme(b.arg == 1); saveUiPrefs(); }
      goTo(SCR_SETTINGS);
      break;
    case ACT_CALIBRATE:  startCalibration(SCR_SETUP); break;
    case ACT_TEST:       goTo(SCR_TEST); break;
    case ACT_CLEAR:      goTo(SCR_TEST); break;
    case ACT_CAL_RETRY:  startCalibration(calReturn); break;
    case ACT_CAL_SAVE:   saveCalibration(); goTo(calReturn); break;
    case ACT_TOGGLE:
      if (running) { stopLiveModule(); setStatus("Halted", C_MUTED); }
      else         { startLiveModule(); setStatus("Started", C_OK); }
      btns[i].label = running ? "HALT" : "START";
      btns[i].color = running ? C_OK : SUBS[curSub].color;
      drawButton(i, false);
      break;
    case ACT_BACK:
      if (screen == SCR_LIVE)      { stopLiveModule(); goTo(SCR_MODULES); }
      else if (screen == SCR_MODULES || screen == SCR_SOON) goTo(SCR_HOME);
      else if (screen == SCR_TEST) goTo(SCR_SETUP);
      else goTo(SCR_HOME);
      break;
  }
}
void freeTouch(const TouchEvent& ev) {
  if (screen != SCR_TEST) return;
  if (ev.y < HEADER_H + 2) return;
  if (ev.type == EV_DOWN || ev.type == EV_MOVE) tft.fillCircle(ev.x, ev.y, 2, C_CYAN);
  else if (ev.type == EV_UP) tft.drawCircle(ev.x, ev.y, 5, C_OK);
}
void onTouch(const TouchEvent& ev) {
  if (ev.type != EV_NONE) lastEv = ev; else lastEv.pressed = ev.pressed;
  if (ignoreUntilRelease) {
    if (trk.state == TS_IDLE && ev.type == EV_NONE) ignoreUntilRelease = false;
    return;
  }
  if (screen == SCR_CALIB) { calibHandle(ev); return; }

  switch (ev.type) {
    case EV_DOWN:
      pressedBtn = hitTest(ev.x, ev.y);
      if (pressedBtn >= 0) { drawButton(pressedBtn, true); pressedShown = true; }
      else freeTouch(ev);
      break;
    case EV_MOVE:
      if (pressedBtn >= 0) {
        bool inside = inRect(ev.x, ev.y, btns[pressedBtn].r, HOLD_SLOP);
        if (inside != pressedShown) { drawButton(pressedBtn, inside); pressedShown = inside; }
      } else freeTouch(ev);
      break;
    case EV_UP:
      if (pressedBtn >= 0) {
        int8_t b = pressedBtn; pressedBtn = -1;
        if (inRect(ev.x, ev.y, btns[b].r, HOLD_SLOP)) doAction(b);
        else if (pressedShown) drawButton(b, false);
        pressedShown = false;
      } else freeTouch(ev);
      break;
    default: break;
  }
}

// ===================================================================
// SETUP / LOOP
// ===================================================================
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  Serial.setTxTimeoutMs(0);
#endif
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);

  // radios
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  BLEDevice::init("Cyberdeck-S3");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new UnifiedBLECallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pAdvertising = BLEDevice::getAdvertising();
  Serial.println("\n[+] Cyberdeck on-deck (Wi-Fi + BLE) online.");

  // display + touch
  pinMode(TFT_CS, OUTPUT);   digitalWrite(TFT_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT); digitalWrite(TOUCH_CS, HIGH);
  loadUiPrefs();
  applyTheme(lightMode);
  if (TFT_BL >= 0) { pinMode(TFT_BL, OUTPUT); setBacklight(); }
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);
  tft.begin(40000000);
  tft.setRotation(1);
  trackerInit(trk, DEFAULT_CAL);
  touchBegin();
  loadCalibration();

  tft.fillScreen(C_BG);
  printCentered("CYBERDECK", W / 2, 100, 3, C_CYAN);
  printCentered("hold the screen now to recalibrate touch", W / 2, 140, 1, C_MUTED);
  bool held = false;
  uint32_t s0 = millis();
  while (millis() - s0 < 1200) { if (touchPoll(millis()).pressed) held = true; delay(TOUCH_POLL_MS); }

  if (!haveSavedCal || held) { startCalibration(SCR_HOME); ignoreUntilRelease = held; }
  else goTo(SCR_HOME);
}

void loop() {
  unsigned long now = millis();

  handleSerialCommand();    // USB dashboard still drives the board
  runActiveModule(now);     // Wi-Fi / BLE work

  if (now - lastPollMs >= TOUCH_POLL_MS) {
    lastPollMs = now;
    onTouch(touchPoll(now));
  }

  // redraw the live list at most ~8x/sec so a Wi-Fi burst doesn't thrash the TFT
  if (screen == SCR_LIVE) {
    if (listDirty && now - lastListDraw >= 120) { lastListDraw = now; listDirty = false; drawList(); }
    if (statusDirty) { statusDirty = false; drawStatusLine(); }
  }
}
