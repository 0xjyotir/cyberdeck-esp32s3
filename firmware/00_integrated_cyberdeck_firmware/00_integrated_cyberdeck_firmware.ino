/*
  Integrated Cyberdeck firmware — the deck driven from its own 2.8" TFT
  ================================================================
  ESP32-S3 N16R8 + 2.8" ILI9341 + XPT2046 touch.
  For learning and authorized testing only: audit networks and devices
  you own or have permission to test.

  The deck's Wi-Fi (1.1-1.6), BLE (2.1-2.6), nRF24 (4.1-4.6) and infrared
  (7.1-7.6) modules,
  picked and read on the on-deck touchscreen. The module code is lifted
  verbatim from firmware/unified_firmware so it behaves identically, and
  the locked serial protocol is untouched, so the USB Web Serial dashboard
  can drive the same modules at the same time (the screen follows it).

  Tap a subsystem -> tap a module -> START. Results stream onto the screen
  the same way they fill the dashboard table (SSID/ch/RSSI for Wi-Fi,
  name/UUID/RSSI for BLE, channel/address/payload for nRF24, protocol /
  code / pulse timings for IR). Status and
  error lines ("module active", "nRF24 not found", ...) show at the
  bottom. HALT stops the module. nRF24 4.1 opens on a live spectrum graph
  (per-channel bars, peak hold, Wi-Fi 1/6/11 bands, waterfall), the
  on-deck version of the dashboard's 4.1 graph. IR captures (7.2-7.5) can
  be drawn as a waveform on the CHART tab.

  Every module has a "?" reference article on the module list.

  RFID: NOT in this firmware yet. Its tile is shown as "Not on deck" and
  opens a short note. To use it today, flash firmware/unified_firmware
  (USB dashboard only). Porting it here needs its pins moved off the
  display's pins first (the display is locked).

  UI: dark "industrial" theme (graphite panels, one accent per subsystem,
  FreeSans type from Adafruit GFX's bundled Fonts/ folder).

  ---- Libraries (Arduino Library Manager) ----
    Adafruit ILI9341 (+ Adafruit GFX, Adafruit BusIO), ArduinoJson v7.x,
    RF24 (TMRh20) for the nRF24 modules,
    IRremoteESP8266 (David Conran / crankyoldgit) for the IR modules.
    (ESP32 BLE + Wi-Fi come with the ESP32 Arduino core.)
  ---- Board ----
    ESP32S3 Dev Module, Flash 16MB, PSRAM "OPI PSRAM", USB CDC On Boot Enabled.

  ---- Wiring: display (LOCKED, never move these) ----
    SCK  12   (shared SPI bus)      LED   7  (or tie to 3V3)
    MOSI 11   (shared SPI bus)      T_CS  4
    MISO 13   (shared SPI bus)      T_IRQ 5
    CS   10   DC 9    RST 8         VCC  3V3
  ---- Wiring: nRF24L01+ (rewired around the display) ----
    CE   2    CSN 1    IRQ 6 (optional, unused by the code)
    SCK 12 / MOSI 11 / MISO 13 shared with the display, own chip-select.
    VCC 3V3 ONLY + a 10uF cap across VCC/GND at the module.
  ---- Wiring: IR (rewired around the display) ----
    Receiver KY-022 / VS1838B:  S -> GPIO15, middle -> 3V3, - -> GND
    Transmitter KY-005:         S -> GPIO16, middle -> 3V3, - -> GND
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
#include <RF24.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
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
// Kept as-is on purpose (locked): other subsystems are wired around it.
#define TFT_SCLK   12
#define TFT_MOSI   11
#define TFT_MISO   13
#define TFT_CS     10
#define TFT_DC     9
#define TFT_RST    8
#define TFT_BL     7     // backlight; tie to 3V3 and set -1 if you don't wire it
#define TOUCH_CS   4
#define TOUCH_IRQ  5     // set to -1 if T_IRQ is not wired

// nRF24L01+ (modules 4.x). Same SPI bus as the display (SCK 12 / MOSI 11 /
// MISO 13) with its own chip-select, so the two never talk at once: every
// display, touch and radio access is its own SPI transaction with its own
// CS, all from loop(). CSN is driven HIGH at boot, before the display
// starts, so the radio can't answer on MISO while the screen/touch use it.
#define NRF_CE_PIN   2
#define NRF_CSN_PIN  1
#define NRF_IRQ_PIN  6    // optional; wired but unused (the modules poll)
// ---- IR (moved off GPIO4/5, which are the display's touch pins now) ----
static const uint16_t IR_RX_PIN = 15;   // KY-022 / VS1838B receiver S (OUT) -> GPIO15
static const uint16_t IR_TX_PIN = 16;   // KY-005 transmitter S -> GPIO16
// GPIO15/16 are plain GPIOs on the S3: not strapping (0/3/45/46), not USB
// (19/20), not flash/PSRAM (26-37), and not used by the display or nRF24.

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
                        SCR_SETTINGS, SCR_INFO };
enum Style  : uint8_t { ST_GRID, ST_HEADER, ST_PILL, ST_ROW, ST_ARROW, ST_BACK, ST_STEP, ST_SLIDER, ST_SEG,
                        ST_TAB, ST_QMARK, ST_RUN };
enum Action : uint8_t { ACT_OPEN_SUB, ACT_OPEN_MOD, ACT_OPEN_SETUP, ACT_BACK, ACT_TOGGLE,
                        ACT_CALIBRATE, ACT_TEST, ACT_CLEAR, ACT_CAL_RETRY, ACT_CAL_SAVE,
                        ACT_OPEN_SETTINGS, ACT_SCROLL, ACT_BRIGHT_STEP, ACT_BRIGHT_SET, ACT_THEME,
                        ACT_VIEW, ACT_OPEN_INFO, ACT_INFO_SCROLL };

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
// meter: nRF24 level 0-32 drawn as bars (0xFF = none); mhz: nRF24 frequency (0 = none)
// tag: short right-hand value for IR rows (bits, pulses, duty...; "" = none)
struct Row { char a[26]; char b[26]; uint16_t color; int16_t rssi; uint32_t t; uint8_t lvl;
             uint8_t meter; int16_t mhz; char tag[8]; };

// ---- reference-article document model (the "?" reader) ----
struct DocLine { uint8_t kind; const char* text; };   // 0 head,1 body,2 sample,3 cap,10+ figure
struct Doc { const char* title; const char* tag; const DocLine* lines; uint8_t count; };

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
  {"nRF24", C_NRF,  NRF_MODS,  6, true},
  {"RFID",  C_RFID, RFID_MODS, 6, false},
  {"IR",    C_IR,   IR_MODS,   6, true},
};
const uint8_t SUB_COUNT = 5;

// Longer names for the live-view header (from the dashboard TITLES).
const char* moduleTitle(const char* id) {
  struct { const char* id; const char* t; } T[] = {
    {"1.1","Active Wi-Fi scan"}, {"1.2","Wardriving logger"}, {"1.3","Promiscuous sniffer"},
    {"1.4","Network discovery"}, {"1.5","AP hosting"}, {"1.6","Frame lab"},
    {"2.1","BLE scan"}, {"2.2","GATT enumeration"}, {"2.3","Advertisement sniffer"},
    {"2.4","Beacon broadcaster"}, {"2.5","GATT read"}, {"2.6","Connection test"},
    {"4.1","Spectrum scan"}, {"4.2","Packet sniffer"}, {"4.3","Address discovery"},
    {"4.4","Protocol analysis"}, {"4.5","Packet storage"}, {"4.6","Lab transmitter"},
    {"7.1","38kHz carrier"}, {"7.2","Pulse learning"}, {"7.3","Signal replay"},
    {"7.4","Protocol decode"}, {"7.5","Signal analysis"}, {"7.6","LED circuit test"},
  };
  for (auto& e : T) if (!strcmp(e.id, id)) return e.t;
  return id;
}

// ===================================================================
// FIRMWARE GLOBALS (from unified_firmware — Wi-Fi + BLE + nRF24 + IR subset)
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
// GATT connects (2.2 / 2.5 / 2.6). The BLE library's connect() blocks loop(),
// so on the deck it is kept short and spaced out, letting touch + HALT run:
//  - ONE reused client. A fresh createClient() per attempt registers a new
//    GATT app each time; once those slots run out, connect() waits forever
//    and the whole deck freezes.
//  - the advertised address TYPE is passed (phones etc. use random addresses,
//    which never connect as "public"), with a timeout instead of waiting forever.
//  - a device that just failed is skipped for a while so another is tried,
//    and there is a pause between attempts.
BLEClient* pGattClient = nullptr;
decltype(((BLEAdvertisedDevice*)nullptr)->getAddressType()) bleTargetType{};
String bleSkipAddr = "";
unsigned long bleSkipUntil = 0, bleNextTryMs = 0;
static const uint32_t BLE_CONNECT_TIMEOUT_MS = 4000;
static const uint32_t BLE_RETRY_GAP_MS = 1500, BLE_SKIP_MS = 20000;

enum WifiScanState { WSCAN_IDLE, WSCAN_RUNNING };
WifiScanState wifiScanState = WSCAN_IDLE;

// --- 2.4GHz nRF24 (modules 4.x), verbatim from unified_firmware ---
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
bool nrfReady = false;                 // radio.begin() + chip check passed once
uint8_t nrfChannel = 0;                // current channel during a scan / sniff
static const uint8_t  NRF_SNIFF_START_CH = 0;    // 4.2 hop range start
static const uint8_t  NRF_SNIFF_END_CH   = 83;   // 4.2 hop range end (2400-2483 MHz)
static const uint64_t NRF_SNIFF_ADDR = 0xE7E7E7E7E7ULL;  // nRF24 factory-default pipe
static const uint8_t  NRF_PREAMBLE[2] = { 0xAA, 0xAA };  // 4.3 preamble-sniff "address"
static const uint8_t  NRF_STORE_CH = 2;                  // 4.5 fixed capture channel
static const uint8_t  NRF_TEST_CH  = 76;                 // 4.6 lab TX channel
static const uint64_t NRF_TEST_ADDR = 0xCCBBDDEE01ULL;   // 4.6 lab TX pipe
unsigned long nrfHits = 0;                               // 4.3 captures seen
unsigned long nrfSent = 0;                               // 4.6 frames sent
static const uint8_t NRF_RING_SIZE = 32;                 // 4.5 ring-buffer depth
uint8_t nrfRing[NRF_RING_SIZE][32];                      // 4.5 in-RAM capture store
uint8_t nrfRingHead = 0;
unsigned long nrfStored = 0;

// --- Infrared (modules 7.x), verbatim from unified_firmware ---
// 7.1, 7.3 and 7.6 all drive IR_TX_PIN (7.1 through LEDC, 7.3 through
// IRsend, 7.6 through digitalWrite); 7.2, 7.4 and 7.5 share the one IRrecv
// on IR_RX_PIN. Only one module is ever active, and startModule() stops
// the previous one first, so the pin is always released before reuse.
static const uint16_t IR_CAPTURE_BUFFER_SIZE = 1024;
static const uint8_t  IR_TIMEOUT = 15;  // ms of silence that ends a capture
static const uint32_t IR_CARRIER_HZ = 38000;   // 7.1 carrier frequency
static const uint8_t  IR_CARRIER_RES_BITS = 8; // 7.1 LEDC resolution (duty 0-255)
static const uint32_t IR_TEST_PULSE_US = 26;   // 7.6 pulse width (~one 38kHz period)
static const uint32_t IR_TEST_PERIOD_MS = 10;  // 7.6 pulse interval
IRrecv irrecv(IR_RX_PIN, IR_CAPTURE_BUFFER_SIZE, IR_TIMEOUT, true);
IRsend irsend(IR_TX_PIN);
decode_results irResults;
decode_type_t lastIrProtocol = decode_type_t::UNKNOWN;
uint64_t lastIrValue = 0;
uint16_t lastIrBits = 0;
bool lastIrCaptured = false;
// Raw timings (us, mark/space alternating) of the last capture from
// 7.2 / 7.4 / 7.5, so 7.3 can replay codes no decoder recognised.
uint16_t lastIrRaw[IR_CAPTURE_BUFFER_SIZE];
uint16_t lastIrRawLen = 0;
unsigned long irTestPulseCount = 0;   // 7.6 pulses fired since START

const char* lastErrMsg = "";           // last emitError() code, for the screen

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

// ---- live charts + structured log + info reader (UI layer; fed from telemetry) ----
const int16_t LV_BAND = HEADER_H + 2;             // control band (tabs + START)
const int16_t LV_BAND_H = 30;
const int16_t LV_Y0 = LV_BAND + LV_BAND_H + 2;    // content top
bool liveGraph = false;                           // LIVE: false = LOG, true = CHART
const uint8_t GHN = 72;
int8_t  gR[GHN]; uint8_t gRn = 0, gRh = 0; int16_t gLastRssi = 0; bool gHaveRssi = false;
uint16_t gChan[15]; uint16_t gChanMax = 0;        // AP count per 2.4GHz channel
const uint8_t GBK = 8;
uint16_t gDist[GBK]; uint16_t gDistMax = 0;        // RSSI histogram buckets
const uint8_t GRN = 72;
uint16_t gRate[GRN]; uint8_t gRaten = 0, gRateh = 0; uint16_t gRateMax = 1;
uint16_t gRateAccum = 0; unsigned long gRateLastMs = 0, gLastPackets = 0;
bool graphDirty = false; unsigned long lastGraphDraw = 0, lastSeenDraw = 0;
int16_t infoScroll = 0;                            // first visible article line
int8_t  infoSub = 0, infoMod = 0;

// ---- nRF24 graphs (UI layer; fed from the 4.x telemetry) ----
// 4.1 spectrum: latest level per channel, decaying peak hold, waterfall of
// the last NRF_WF_ROWS sweeps. 4.2-4.5: packets per channel. 4.6: ack history.
const uint8_t NRF_CH = 126, NRF_MAXLV = 32, NRF_WF_ROWS = 26;
uint8_t  nrfLvl[NRF_CH];                 // 0..32 RPD hits, last pass per channel
uint16_t nrfPeak16[NRF_CH];              // peak hold, level x16 (fractional decay)
uint8_t  nrfWf[NRF_WF_ROWS][NRF_CH];     // waterfall ring, one row per sweep
uint8_t  nrfWfHead = 0, nrfWfCount = 0;
unsigned long nrfSweeps = 0;
bool     nrfWfDirty = false;
int16_t  nrfDrawnTop[NRF_CH], nrfDrawnPk[NRF_CH];   // what's on screen, to skip unchanged columns
int16_t  nrfCursorCh = -1;               // sweep cursor under the axis
unsigned long nrfPeakMs = 0;
char     nrfStat[24] = "";
uint16_t nrfRampLUT[NRF_MAXLV + 1], nrfWfLUT[NRF_MAXLV + 1];
uint16_t nrfHitCh[NRF_CH]; uint16_t nrfHitMax = 0;  // 4.2-4.5 packets per channel
uint8_t  nrfAck[GHN]; uint8_t nrfAckN = 0, nrfAckH = 0;  // 4.6: 1 ack / 0 no ack
unsigned long nrfAckOk = 0, nrfAckTotal = 0;
uint8_t  nrfShownHop = 0xFF;             // hop channel the chart last showed

// ---- IR graphs (UI layer; fed from the 7.x telemetry + the last capture) ----
unsigned long irLastPulses = 0;          // 7.6: previous pulse count, for pulses / s

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
void graphsReset() {
  gRn = gRh = 0; gHaveRssi = false; gLastRssi = 0;
  memset(gChan, 0, sizeof(gChan)); gChanMax = 0;
  memset(gDist, 0, sizeof(gDist)); gDistMax = 0;
  gRaten = gRateh = 0; gRateMax = 1; gRateAccum = 0;
  gRateLastMs = millis(); gLastPackets = packetCount;
  memset(nrfLvl, 0, sizeof(nrfLvl)); memset(nrfPeak16, 0, sizeof(nrfPeak16));
  memset(nrfWf, 0, sizeof(nrfWf)); nrfWfHead = nrfWfCount = 0; nrfSweeps = 0; nrfWfDirty = true;
  memset(nrfHitCh, 0, sizeof(nrfHitCh)); nrfHitMax = 0;
  nrfAckN = nrfAckH = 0; nrfAckOk = nrfAckTotal = 0; nrfShownHop = 0xFF;
  nrfPeakMs = millis(); nrfStat[0] = 0;
  irLastPulses = 0;
}
void rowsClear() { rowCount = 0; rowHead = 0; rowTotal = 0; listDirty = true; graphsReset(); graphDirty = true; }
void pushRow(const char* a, const char* b, uint16_t color, int16_t rssi, uint8_t lvl) {
  Row& r = rowBuf[rowHead];
  strncpy(r.a, a, sizeof(r.a) - 1); r.a[sizeof(r.a) - 1] = 0;
  strncpy(r.b, b, sizeof(r.b) - 1); r.b[sizeof(r.b) - 1] = 0;
  r.color = color; r.rssi = rssi; r.lvl = lvl; r.t = millis() - liveStartMs;
  r.meter = 0xFF; r.mhz = 0; r.tag[0] = 0;
  rowHead = (rowHead + 1) % ROWS;
  if (rowCount < ROWS) rowCount++;
  rowTotal++; listDirty = true;
}
// nRF24 row: no RSSI; carries a frequency and/or a 0-32 level meter instead.
void pushRowNrf(const char* a, const char* b, uint16_t color, uint8_t lvl, int16_t mhz, uint8_t meter) {
  pushRow(a, b, color, 0, lvl);
  Row& r = rowBuf[(rowHead + ROWS - 1) % ROWS];
  r.mhz = mhz; r.meter = meter;
}
// IR row: no RSSI; a short value (bits / pulses / duty) on the right instead.
void pushRowIr(const char* a, const char* b, uint16_t color, uint8_t lvl, const char* tag) {
  pushRow(a, b, color, 0, lvl);
  Row& r = rowBuf[(rowHead + ROWS - 1) % ROWS];
  strncpy(r.tag, tag, sizeof(r.tag) - 1); r.tag[sizeof(r.tag) - 1] = 0;
}
// graph feeders (derived from the same telemetry the dashboard gets)
void gRpush(int v) { if (v > 0) v = 0; if (v < -120) v = -120; gR[gRh] = (int8_t)v; gRh = (gRh + 1) % GHN; if (gRn < GHN) gRn++; gLastRssi = v; gHaveRssi = true; }
int gRget(uint8_t i) { uint8_t idx = (gRh + GHN - gRn + i) % GHN; return gR[idx]; }
void chanBump(int ch) { if (ch < 1 || ch > 14) return; if (++gChan[ch] > gChanMax) gChanMax = gChan[ch]; }
void distBump(int rssi) { int b = (rssi + 100) / 10; if (b < 0) b = 0; if (b > GBK - 1) b = GBK - 1; if (++gDist[b] > gDistMax) gDistMax = gDist[b]; }
void ratePush(uint16_t v) { gRate[gRateh] = v; gRateh = (gRateh + 1) % GRN; if (gRaten < GRN) gRaten++; if (v > gRateMax) gRateMax = v; }
uint16_t gRateGet(uint8_t i) { uint8_t idx = (gRateh + GRN - gRaten + i) % GRN; return gRate[idx]; }
uint16_t gRateLast() { return gRaten ? gRateGet(gRaten - 1) : 0; }
int peakChannel() { int pk = 0, pv = 0; for (int c = 1; c <= 14; c++) if (gChan[c] > pv) { pv = gChan[c]; pk = c; } return pk; }
uint16_t rssiColor(int r) { if (r >= -55) return C_OK; if (r >= -70) return C_WIFI; if (r >= -82) return C_WARN; return C_ERR; }
uint8_t rssiBars(int r) { if (r >= -55) return 4; if (r >= -67) return 3; if (r >= -78) return 2; if (r >= -90) return 1; return 0; }
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
  if (!strcmp(code, "nrf_not_found"))   return "nRF24 not found: check CE2 CSN1 3V3";
  if (!strcmp(code, "no_code_captured")) return "No code yet: capture with 7.2/7.4/7.5";
  if (!strcmp(code, "ledc_attach_failed")) return "IR carrier failed to start (LEDC)";
  return code;
}

// nRF24 level colour ramp (0..32): cyan -> nRF amber -> red, like the
// dashboard's blue -> amber -> red. Tracks the theme via the C_* colours.
uint16_t nrfRamp(uint8_t lv) {
  if (lv > NRF_MAXLV) lv = NRF_MAXLV;
  if (lv <= NRF_MAXLV / 2) return mix565(C_NRF, C_CYAN, (uint8_t)(lv * 255 / (NRF_MAXLV / 2)));
  return mix565(C_ERR, C_NRF, (uint8_t)((lv - NRF_MAXLV / 2) * 255 / (NRF_MAXLV / 2)));
}
const char* liveId() { return SUBS[curSub].mods[curMod].id; }
// Modules whose live view opens on CHART: the nRF24 spectrum and the IR waveform views.
bool opensOnChart(const char* id) { return !strcmp(id, "4.1") || !strcmp(id, "7.2") || !strcmp(id, "7.5"); }

// A full 126-channel sweep finished: add it to the waterfall and log one
// summary row (busiest channel + how many channels showed a carrier).
void nrfCommitSweep() {
  memcpy(nrfWf[nrfWfHead], nrfLvl, NRF_CH);
  nrfWfHead = (nrfWfHead + 1) % NRF_WF_ROWS;
  if (nrfWfCount < NRF_WF_ROWS) nrfWfCount++;
  nrfSweeps++; nrfWfDirty = true;
  uint8_t pk = 0, busy = 0;
  for (uint8_t c = 0; c < NRF_CH; c++) { if (nrfLvl[c] > nrfLvl[pk]) pk = c; if (nrfLvl[c]) busy++; }
  char a[26], b[26];
  snprintf(a, sizeof(a), "sweep #%lu", nrfSweeps);
  if (nrfLvl[pk]) snprintf(b, sizeof(b), "ch%u  %u/%u  busy %u ch", pk, nrfLvl[pk], NRF_MAXLV, busy);
  else            snprintf(b, sizeof(b), "quiet  busy 0 ch");
  pushRowNrf(a, b, nrfLvl[pk] ? nrfRamp(nrfLvl[pk]) : C_DIM, 0, nrfLvl[pk] ? 2400 + pk : 0, nrfLvl[pk]);
}

// nRF24 telemetry -> screen rows + graph state. Same fields the dashboard's
// NRF_VIEWS table shows; 4.1 feeds the spectrum instead of one row per channel.
void uiOnNrf(const char* m, JsonObject d, bool isErr) {
  char a[26] = "", b[26] = "";
  int ch = d["ch"] | -1;
  if (!strcmp(m, "4.1")) {
    if (ch < 0 || ch >= NRF_CH) return;
    int lv = d["level"] | 0; if (lv < 0) lv = 0; if (lv > NRF_MAXLV) lv = NRF_MAXLV;
    nrfLvl[ch] = (uint8_t)lv;
    if (lv * 16 > nrfPeak16[ch]) nrfPeak16[ch] = lv * 16;
    if (ch == NRF_CH - 1) nrfCommitSweep();
    graphDirty = true;
    return;
  }
  if (!strcmp(m, "4.2")) {
    const char* addr = d["addr"] | "-"; const char* pl = d["payload"] | ""; int len = d["len"] | 0;
    snprintf(a, sizeof(a), "ch %d  %s  %d B", ch, addr, len);
    snprintf(b, sizeof(b), "%.22s%s", pl, strlen(pl) > 22 ? ".." : "");
  } else if (!strcmp(m, "4.3")) {
    const char* addr = d["addr"] | "-"; unsigned long hits = d["hits"] | 0UL;
    snprintf(a, sizeof(a), "%s", addr);
    snprintf(b, sizeof(b), "ch %d  hits %lu", ch, hits);
  } else if (!strcmp(m, "4.4")) {
    const char* addr = d["addr"] | "-"; const char* rate = d["rate"] | "-";
    const char* guess = d["guess"] | "-"; int len = d["len"] | 0;
    snprintf(a, sizeof(a), "ch %d  %s", ch, guess);
    snprintf(b, sizeof(b), "%d B  %s  %s", len, rate, addr);
  } else if (!strcmp(m, "4.5")) {
    unsigned long st = d["stored"] | 0UL; const char* la = d["last_addr"] | "-"; const char* last = d["last"] | "";
    ch = NRF_STORE_CH;
    snprintf(a, sizeof(a), "stored %lu  %s", st, la);
    snprintf(b, sizeof(b), "%.22s%s", last, strlen(last) > 22 ? ".." : "");
  } else if (!strcmp(m, "4.6")) {
    unsigned long sent = d["sent"] | 0UL; bool ack = d["ack"] | false; const char* addr = d["addr"] | "-";
    snprintf(a, sizeof(a), "sent %lu  %s", sent, ack ? "ACK" : "no ack");
    snprintf(b, sizeof(b), "ch %d  %s", ch, addr);
    nrfAck[nrfAckH] = ack ? 1 : 0; nrfAckH = (nrfAckH + 1) % GHN; if (nrfAckN < GHN) nrfAckN++;
    nrfAckTotal++; if (ack) nrfAckOk++;
    pushRowNrf(a, b, ack ? C_OK : C_WARN, ack ? 0 : 2, 2400 + ch, 0xFF);
    gRateAccum++; graphDirty = true;
    return;
  } else return;
  if (ch >= 0 && ch < NRF_CH && ++nrfHitCh[ch] > nrfHitMax) nrfHitMax = nrfHitCh[ch];
  uint16_t col = C_NRF;
  if (!strcmp(m, "4.4") && !strcmp(d["guess"] | "", "control")) col = C_CYAN;
  pushRowNrf(a, b, isErr ? C_ERR : col, isErr ? 3 : 0, ch >= 0 ? 2400 + ch : 0, 0xFF);
  gRateAccum++; graphDirty = true;
}

// IR telemetry -> screen rows. Same fields the dashboard's IR_VIEWS table
// shows; the CHART tab draws the last capture itself (lastIrRaw).
void uiOnIr(const char* m, JsonObject d, bool isErr) {
  char a[26] = "", b[26] = "", t[8] = "";
  uint16_t col = C_IR; uint8_t lvl = 0;
  if (!strcmp(m, "7.1")) {
    unsigned long hz = d["freq_hz"] | 0UL; int duty = d["duty_pct"] | 0; int pin = d["pin"] | 0;
    snprintf(a, sizeof(a), "carrier %lu.%lu kHz", hz / 1000, (hz % 1000) / 100);
    snprintf(b, sizeof(b), "duty %d%%  TX GPIO%d", duty, pin);
    strcpy(t, hz ? "ON" : "OFF");
    if (!hz) { col = C_WARN; lvl = 2; }
  } else if (!strcmp(m, "7.2")) {
    unsigned count = d["count"] | 0U; bool ovf = d["overflow"] | false;
    snprintf(a, sizeof(a), "burst  %u pulses%s", count, ovf ? " OVF" : "");
    uint8_t n = 0;                                   // first timings, as many as fit
    for (uint16_t i = 0; i < lastIrRawLen && n < sizeof(b) - 1; i++) {
      char v[8]; snprintf(v, sizeof(v), i ? " %u" : "%u", lastIrRaw[i]);
      if (n + strlen(v) > sizeof(b) - 3) { strcpy(b + n, ".."); n += 2; break; }
      strcpy(b + n, v); n += strlen(v);
    }
    snprintf(t, sizeof(t), "%u", count);
    if (ovf) { col = C_WARN; lvl = 2; }
  } else if (!strcmp(m, "7.3")) {
    const char* proto = d["protocol"] | "?";
    snprintf(a, sizeof(a), "replayed %s", proto);
    if (!strcmp(proto, "RAW")) snprintf(b, sizeof(b), "%u raw timings @38kHz", (unsigned)(d["count"] | 0U));
    else snprintf(b, sizeof(b), "0x%s  %u bits", d["value"] | "", (unsigned)(d["bits"] | 0U));
    strcpy(t, "SENT");
    char l[48]; snprintf(l, sizeof(l), "Sent %s on GPIO%u", proto, IR_TX_PIN);
    setStatus(l, C_OK);
  } else if (!strcmp(m, "7.4")) {
    const char* proto = d["protocol"] | "?"; unsigned bits = d["bits"] | 0U;
    snprintf(a, sizeof(a), "%s", proto);
    snprintf(b, sizeof(b), "0x%s", d["value"] | "");
    snprintf(t, sizeof(t), "%ub", bits);
    if (!strcmp(proto, "UNKNOWN")) { col = C_WARN; lvl = 2; }
  } else if (!strcmp(m, "7.5")) {
    unsigned pulses = d["pulses"] | 0U;
    unsigned long mk = d["mark_us"] | 0UL, sp = d["space_us"] | 0UL;
    float duty = d["duty_pct"] | 0.0f;
    snprintf(a, sizeof(a), "%u pulses  %s", pulses, d["protocol"] | "?");
    snprintf(b, sizeof(b), "mark %.1f  space %.1f ms", mk / 1000.0f, sp / 1000.0f);
    snprintf(t, sizeof(t), "%d%%", (int)(duty + 0.5f));
  } else if (!strcmp(m, "7.6")) {
    unsigned long pulses = d["pulses"] | 0UL;
    unsigned long dp = pulses >= irLastPulses ? pulses - irLastPulses : pulses;
    irLastPulses = pulses;
    snprintf(a, sizeof(a), "%lu pulses fired", pulses);
    snprintf(b, sizeof(b), "%luus every %lums GPIO%d", (unsigned long)(d["pulse_us"] | 0UL),
             (unsigned long)(d["period_ms"] | 0UL), (int)(d["pin"] | 0));
    snprintf(t, sizeof(t), "+%lu", dp);
    ratePush(dp > 65535 ? 65535 : (uint16_t)dp);     // pulses / s (the loop skips 7.6)
    pushRowIr(a, b, col, lvl, t);
    graphDirty = true;
    return;
  } else return;
  pushRowIr(a, b, isErr ? C_ERR : col, isErr ? 3 : lvl, t);
  gRateAccum++; graphDirty = true;
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
    char line[48]; snprintf(line, sizeof(line), "%s %s", isErr ? "ERR" : "info", friendlyStatus(m));
    setStatus(line, isErr ? C_ERR : C_MUTED);
    return;
  }

  if (module[0] == '4') { uiOnNrf(module, d, isErr); return; }   // nRF24
  if (module[0] == '7') { uiOnIr(module, d, isErr); return; }    // infrared

  char a[26] = "", b[26] = "";
  if (module[0] == '1') {                         // Wi-Fi
    const char* ssid = d["ssid"] | "<hidden>";
    long ch = d["ch"] | 0; long rssi = d["rssi"] | 0;
    snprintf(a, sizeof(a), "%s", ssid);
    snprintf(b, sizeof(b), "ch %ld", ch);
    pushRow(a, b, C_WIFI, (int16_t)rssi, isErr ? 3 : 0);
    gRpush(rssi); chanBump(ch); distBump(rssi);
  } else {                                        // BLE
    const char* name = d["name"] | "";
    const char* addr = d["addr"] | "";
    const char* uuid = d["uuid"] | "";
    const char* payload = d["payload"] | "";
    long rssi = d["rssi"] | 0;
    snprintf(a, sizeof(a), "%s", (name[0] && strcmp(name, "Unknown")) ? name : (addr[0] ? addr : "BLE device"));
    if (payload[0])   snprintf(b, sizeof(b), "%s", payload);
    else if (uuid[0] && strcmp(uuid, "N/A")) snprintf(b, sizeof(b), "%s", uuid);
    else              b[0] = 0;
    pushRow(a, b, isErr ? C_ERR : C_BLE, (int16_t)rssi, isErr ? 3 : 0);
    gRpush(rssi); distBump(rssi);
  }
  gRateAccum++; graphDirty = true;
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
  lastErrMsg = msg;                       // callers pass string literals
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
        String a = device.getAddress().toString().c_str();
        if (a == bleSkipAddr && millis() < bleSkipUntil) return;   // just failed: try another
        bleTargetAddr = a;
        bleTargetType = device.getAddressType();
        bleTargetFound = true;
        if (pBLEScan) pBLEScan->stop();
      }
    }
  }
};

// ===================================================================
// nRF24 INIT — lazy one-time bring-up; reports if the module is missing
//   SPI is already running (setup() starts it for the display), so the
//   radio just joins the bus: radio.begin(&SPI) sets up CE/CSN and talks
//   to the chip in its own SPI transactions, never re-pinning the bus.
// ===================================================================
bool ensureNrf() {
  if (nrfReady) return true;
  if (!radio.begin(&SPI) || !radio.isChipConnected()) return false;
  nrfReady = true;
  return true;
}

// ===================================================================
// IR CAPTURE HELPER — remember the last capture (decoded + raw) for 7.3
// ===================================================================
void rememberIrCapture(const decode_results& r) {
  lastIrProtocol = r.decode_type;
  lastIrValue = r.value;
  lastIrBits = r.bits;
  lastIrRawLen = 0;
  // rawbuf[0] is the gap before the frame; real timings start at 1.
  for (uint16_t i = 1; i < r.rawlen && lastIrRawLen < IR_CAPTURE_BUFFER_SIZE; i++) {
    uint32_t us = (uint32_t)r.rawbuf[i] * kRawTick;
    lastIrRaw[lastIrRawLen++] = us > UINT16_MAX ? UINT16_MAX : (uint16_t)us;
  }
  lastIrCaptured = true;
}

// ===================================================================
// MODULE START / STOP (Wi-Fi + BLE + nRF24 + IR branches, verbatim from firmware)
// ===================================================================
void stopModule();

void startModule(const String& module) {
  if (activeModule != "NONE") stopModule();
  activeModule = module;
  moduleActionMillis = 0; moduleStateMillis = 0;
  bleTargetFound = false; bleTargetAddr = ""; bleTestAttempts = 0;
  bleSkipAddr = ""; bleSkipUntil = 0; bleNextTryMs = 0;
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
  } else if (module == "4.1") {
    if (!ensureNrf()) { emitError("4.1", "nrf_not_found"); activeModule = "NONE"; return; }
    radio.powerUp();
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.setDataRate(RF24_1MBPS);
    radio.setPayloadSize(32);
    radio.stopListening();
    nrfChannel = 0;
  } else if (module == "4.2") {
    if (!ensureNrf()) { emitError("4.2", "nrf_not_found"); activeModule = "NONE"; return; }
    radio.powerUp();
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.setDataRate(RF24_2MBPS);
    radio.setPayloadSize(32);
    radio.setAddressWidth(5);
    nrfChannel = NRF_SNIFF_START_CH;
    radio.setChannel(nrfChannel);
    radio.openReadingPipe(1, NRF_SNIFF_ADDR);
    radio.startListening();
  } else if (module == "4.3") {
    if (!ensureNrf()) { emitError("4.3", "nrf_not_found"); activeModule = "NONE"; return; }
    radio.powerUp();
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.setAddressWidth(2);          // 2-byte width => preamble sniffing
    radio.setDataRate(RF24_2MBPS);
    radio.setPayloadSize(32);
    nrfChannel = NRF_SNIFF_START_CH;
    nrfHits = 0;
    radio.setChannel(nrfChannel);
    radio.openReadingPipe(0, NRF_PREAMBLE);
    radio.startListening();
  } else if (module == "4.4") {
    if (!ensureNrf()) { emitError("4.4", "nrf_not_found"); activeModule = "NONE"; return; }
    radio.powerUp();
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.setAddressWidth(5);
    radio.setDataRate(RF24_2MBPS);
    radio.setPayloadSize(32);
    nrfChannel = NRF_SNIFF_START_CH;
    radio.setChannel(nrfChannel);
    radio.openReadingPipe(1, NRF_SNIFF_ADDR);
    radio.startListening();
  } else if (module == "4.5") {
    if (!ensureNrf()) { emitError("4.5", "nrf_not_found"); activeModule = "NONE"; return; }
    radio.powerUp();
    radio.setAutoAck(false);
    radio.disableCRC();
    radio.setAddressWidth(5);
    radio.setDataRate(RF24_2MBPS);
    radio.setPayloadSize(32);
    radio.setChannel(NRF_STORE_CH);
    radio.openReadingPipe(1, NRF_SNIFF_ADDR);
    nrfStored = 0;
    nrfRingHead = 0;
    radio.startListening();
  } else if (module == "4.6") {
    if (!ensureNrf()) { emitError("4.6", "nrf_not_found"); activeModule = "NONE"; return; }
    radio.powerUp();
    radio.setPALevel(RF24_PA_LOW);     // low power — lab/self-test only
    radio.setAddressWidth(5);
    radio.setDataRate(RF24_2MBPS);
    radio.setPayloadSize(32);
    radio.setRetries(5, 5);            // cap ack wait so loop() stays responsive
    radio.setChannel(NRF_TEST_CH);
    radio.openWritingPipe(NRF_TEST_ADDR);
    radio.stopListening();
    nrfSent = 0;
  } else if (module == "7.1") {
    if (!ledcAttach(IR_TX_PIN, IR_CARRIER_HZ, IR_CARRIER_RES_BITS)) {
      emitError("7.1", "ledc_attach_failed");
      activeModule = "NONE";
      return;
    }
    ledcWrite(IR_TX_PIN, 1 << (IR_CARRIER_RES_BITS - 1));  // 50% duty
  } else if (module == "7.2" || module == "7.4" || module == "7.5") {
    irrecv.enableIRIn();
  } else if (module == "7.3") {
    irsend.begin();
    // one-shot replay happens in the execution section below, then
    // the module auto-returns to NONE.
  } else if (module == "7.6") {
    pinMode(IR_TX_PIN, OUTPUT);
    digitalWrite(IR_TX_PIN, LOW);
    irTestPulseCount = 0;
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
  } else if (activeModule == "4.1" || activeModule == "4.2" || activeModule == "4.3" ||
             activeModule == "4.4" || activeModule == "4.5" || activeModule == "4.6") {
    if (nrfReady) { radio.stopListening(); radio.powerDown(); }
  } else if (activeModule == "7.1") {
    ledcWrite(IR_TX_PIN, 0);
    ledcDetach(IR_TX_PIN);
    pinMode(IR_TX_PIN, OUTPUT);
    digitalWrite(IR_TX_PIN, LOW);   // leave the IR LED off
  } else if (activeModule == "7.2" || activeModule == "7.4" || activeModule == "7.5") {
    irrecv.disableIRIn();
  } else if (activeModule == "7.6") {
    digitalWrite(IR_TX_PIN, LOW);
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
// MODULE WORK (Wi-Fi / BLE / nRF24 / IR loop bodies, verbatim from firmware)
// ===================================================================
BLEClient* gattClient() {
  if (!pGattClient) pGattClient = BLEDevice::createClient();
  return pGattClient;
}
bool gattConnect(BLEClient* c) {
  return c->connect(BLEAddress(bleTargetAddr.c_str()), bleTargetType, BLE_CONNECT_TIMEOUT_MS);
}
// Say what's happening before the (blocking) connect, so the frozen moment is explained.
void gattConnecting() {
  char l[48]; snprintf(l, sizeof(l), "Connecting %s ...", bleTargetAddr.c_str());
  setStatus(l, C_WARN);
  if (screen == SCR_LIVE) { statusDirty = false; drawStatusLine(); }
}

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
    } else if ((long)(now - bleNextTryMs) >= 0) {
      BLEClient* pClient = gattClient();
      gattConnecting();
      if (gattConnect(pClient)) {
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
        bleSkipAddr = bleTargetAddr; bleSkipUntil = millis() + BLE_SKIP_MS;
        emitError(activeModule.c_str(), "connect_failed");
      }
      bleNextTryMs = millis() + BLE_RETRY_GAP_MS;   // let touch / HALT run before the next try
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
        BLEClient* client = gattClient();
        gattConnecting();
        unsigned long connStart = millis();
        if (gattConnect(client)) {
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
        moduleActionMillis = millis();   // 1 s gap after the connect, not before it
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

  // --- 2.4GHZ nRF24 SUBSYSTEM (verbatim from unified_firmware) ---
  // 4.1 spectrum scan: one channel per pass, 32 RPD samples each, cycling 0-125.
  else if (activeModule == "4.1") {
    if (now - moduleActionMillis >= 12) {
      moduleActionMillis = now;
      radio.setChannel(nrfChannel);
      uint8_t hits = 0;
      for (uint8_t i = 0; i < 32; i++) {
        radio.startListening();
        delayMicroseconds(130);
        radio.stopListening();
        if (radio.testRPD()) hits++;
      }
      telemetryDoc["data"]["ch"] = nrfChannel;
      telemetryDoc["data"]["mhz"] = 2400 + nrfChannel;
      telemetryDoc["data"]["level"] = hits;        // 0-32 carrier hits this pass
      emitTelemetry("4.1", "ok");
      nrfChannel = (nrfChannel + 1) % 126;
    }
  }
  // 4.2 packet sniffer: listen on the default pipe, hop channel once a second,
  // dump any received payload as hex. (Pipe-address capture; full promiscuous
  // mode is a later refinement and wants a confirmed nRF24L01+, not a clone.)
  else if (activeModule == "4.2") {
    if (now - moduleStateMillis >= 1000) {
      moduleStateMillis = now;
      nrfChannel++;
      if (nrfChannel > NRF_SNIFF_END_CH) nrfChannel = NRF_SNIFF_START_CH;
      radio.stopListening();
      radio.setChannel(nrfChannel);
      radio.startListening();
    }
    if (radio.available()) {
      uint8_t buf[32];
      radio.read(buf, 32);
      String hex = "";
      for (uint8_t i = 0; i < 32; i++) {
        char b[3];
        sprintf(b, "%02X", buf[i]);
        hex += b;
      }
      telemetryDoc["data"]["ch"] = nrfChannel;
      telemetryDoc["data"]["addr"] = "pipe1";
      telemetryDoc["data"]["len"] = 32;
      telemetryDoc["data"]["payload"] = hex;
      emitTelemetry("4.2", "ok");
    }
  }
  // 4.3 address discovery: preamble sniff, hop channels, report candidate addr.
  else if (activeModule == "4.3") {
    if (now - moduleStateMillis >= 300) {
      moduleStateMillis = now;
      nrfChannel++;
      if (nrfChannel > NRF_SNIFF_END_CH) nrfChannel = NRF_SNIFF_START_CH;
      radio.stopListening();
      radio.setChannel(nrfChannel);
      radio.startListening();
    }
    if (radio.available()) {
      uint8_t buf[32];
      radio.read(buf, 32);
      nrfHits++;
      String addr = "";
      for (uint8_t i = 0; i < 5; i++) {
        char b[3];
        sprintf(b, "%02X", buf[i]);
        addr += b;
      }
      telemetryDoc["data"]["ch"] = nrfChannel;
      telemetryDoc["data"]["addr"] = addr;
      telemetryDoc["data"]["hits"] = nrfHits;
      emitTelemetry("4.3", "ok");
    }
  }
  // 4.4 protocol analysis: per-packet length/rate + heuristic traffic guess.
  else if (activeModule == "4.4") {
    if (now - moduleStateMillis >= 1000) {
      moduleStateMillis = now;
      nrfChannel++;
      if (nrfChannel > NRF_SNIFF_END_CH) nrfChannel = NRF_SNIFF_START_CH;
      radio.stopListening();
      radio.setChannel(nrfChannel);
      radio.startListening();
    }
    if (radio.available()) {
      uint8_t buf[32];
      radio.read(buf, 32);
      uint8_t nonZero = 0;
      for (uint8_t i = 0; i < 32; i++) if (buf[i] != 0) nonZero++;
      telemetryDoc["data"]["ch"] = nrfChannel;
      telemetryDoc["data"]["addr"] = "pipe1";
      telemetryDoc["data"]["len"] = 32;
      telemetryDoc["data"]["rate"] = "2MBPS";
      telemetryDoc["data"]["guess"] = nonZero <= 8 ? "control" : "data";
      emitTelemetry("4.4", "ok");
    }
  }
  // 4.5 packet storage: ring-buffer captures, report running count + last frame.
  else if (activeModule == "4.5") {
    if (radio.available()) {
      radio.read(nrfRing[nrfRingHead], 32);
      String hex = "";
      for (uint8_t i = 0; i < 32; i++) {
        char b[3];
        sprintf(b, "%02X", nrfRing[nrfRingHead][i]);
        hex += b;
      }
      nrfRingHead = (nrfRingHead + 1) % NRF_RING_SIZE;
      nrfStored++;
      telemetryDoc["data"]["stored"] = nrfStored;
      telemetryDoc["data"]["last_addr"] = "pipe1";
      telemetryDoc["data"]["last"] = hex;
      emitTelemetry("4.5", "ok");
    }
  }
  // 4.6 lab transmitter: low-power test TX to your own node, report sent + ack.
  else if (activeModule == "4.6") {
    if (now - moduleActionMillis >= 500) {
      moduleActionMillis = now;
      uint8_t payload[32] = {0};
      payload[0] = 'C'; payload[1] = 'Y'; payload[2] = 'B';
      payload[3] = (uint8_t)(nrfSent & 0xFF);
      bool ack = radio.write(payload, 32);   // bounded by setRetries(5,5)
      nrfSent++;
      telemetryDoc["data"]["ch"] = NRF_TEST_CH;
      telemetryDoc["data"]["addr"] = "CYBER";
      telemetryDoc["data"]["sent"] = nrfSent;
      telemetryDoc["data"]["ack"] = ack;
      emitTelemetry("4.6", "ok");
    }
  }

  // --- INFRARED SUBSYSTEM (verbatim from unified_firmware) ---
  // 7.4 = protocol_libraries (decode via IRremoteESP8266: NEC/Sony/RC5/RC6/...)
  else if (activeModule == "7.4") {
    // irrecv.decode() is non-blocking: it just checks whether the
    // interrupt-driven capture buffer has a finished result.
    if (irrecv.decode(&irResults)) {
      rememberIrCapture(irResults);

      telemetryDoc["data"]["protocol"] = typeToString(irResults.decode_type);
      telemetryDoc["data"]["value"] = uint64ToString(irResults.value, 16);
      telemetryDoc["data"]["bits"] = irResults.bits;
      emitTelemetry("7.4", "ok");

      irrecv.resume();
    }
  }
  // 7.2 = pulse_learning (raw mark/space timings of each capture)
  else if (activeModule == "7.2") {
    if (irrecv.decode(&irResults)) {
      rememberIrCapture(irResults);

      telemetryDoc["data"]["count"] = lastIrRawLen;
      telemetryDoc["data"]["overflow"] = irResults.overflow;
      JsonArray timings = telemetryDoc["data"]["timings_us"].to<JsonArray>();
      for (uint16_t i = 0; i < lastIrRawLen; i++) timings.add(lastIrRaw[i]);
      emitTelemetry("7.2", "ok");

      irrecv.resume();
    }
  }
  // 7.5 = signal_analysis (mark/space totals per capture)
  else if (activeModule == "7.5") {
    if (irrecv.decode(&irResults)) {
      rememberIrCapture(irResults);

      uint32_t markTime = 0;
      uint32_t spaceTime = 0;
      for (uint16_t i = 0; i < lastIrRawLen; i++) {
        // even index = mark (LED on), odd index = space (LED off)
        if (i % 2 == 0) markTime += lastIrRaw[i];
        else            spaceTime += lastIrRaw[i];
      }
      uint32_t total = markTime + spaceTime;

      telemetryDoc["data"]["pulses"] = lastIrRawLen;
      telemetryDoc["data"]["mark_us"] = markTime;
      telemetryDoc["data"]["space_us"] = spaceTime;
      telemetryDoc["data"]["total_us"] = total;
      telemetryDoc["data"]["duty_pct"] = total ? (100.0f * markTime / total) : 0.0f;
      telemetryDoc["data"]["protocol"] = typeToString(irResults.decode_type);
      emitTelemetry("7.5", "ok");

      irrecv.resume();
    }
  }
  // 7.3 = signal_replay (one-shot replay of the last 7.2/7.4/7.5 capture)
  else if (activeModule == "7.3") {
    if (!lastIrCaptured) {
      emitError("7.3", "no_code_captured");
    } else if (lastIrProtocol != decode_type_t::UNKNOWN) {
      irsend.send(lastIrProtocol, lastIrValue, lastIrBits);
      telemetryDoc["data"]["protocol"] = typeToString(lastIrProtocol);
      telemetryDoc["data"]["value"] = uint64ToString(lastIrValue, 16);
      telemetryDoc["data"]["bits"] = lastIrBits;
      telemetryDoc["data"]["action"] = "replayed";
      emitTelemetry("7.3", "ok");
    } else {
      // No decoder matched: resend the raw timings on a 38kHz carrier.
      irsend.sendRaw(lastIrRaw, lastIrRawLen, IR_CARRIER_HZ / 1000);
      telemetryDoc["data"]["protocol"] = "RAW";
      telemetryDoc["data"]["count"] = lastIrRawLen;
      telemetryDoc["data"]["action"] = "replayed";
      emitTelemetry("7.3", "ok");
    }
    activeModule = "NONE";
  }
  // 7.1 = 38khz_modulation (carrier runs in hardware; just report)
  else if (activeModule == "7.1") {
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      telemetryDoc["data"]["freq_hz"] = ledcReadFreq(IR_TX_PIN);
      telemetryDoc["data"]["duty_pct"] = 50;
      telemetryDoc["data"]["pin"] = IR_TX_PIN;
      emitTelemetry("7.1", "ok");
    }
  }
  // 7.6 = led_circuit_tests (26us pulse every 10ms, count reported 1/s)
  else if (activeModule == "7.6") {
    if (now - moduleStateMillis >= IR_TEST_PERIOD_MS) {
      moduleStateMillis = now;
      digitalWrite(IR_TX_PIN, HIGH);
      delayMicroseconds(IR_TEST_PULSE_US);  // bounded 26us busy-wait
      digitalWrite(IR_TX_PIN, LOW);
      irTestPulseCount++;
    }
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      telemetryDoc["data"]["pulses"] = irTestPulseCount;
      telemetryDoc["data"]["pulse_us"] = IR_TEST_PULSE_US;
      telemetryDoc["data"]["period_ms"] = IR_TEST_PERIOD_MS;
      telemetryDoc["data"]["pin"] = IR_TX_PIN;
      emitTelemetry("7.6", "ok");
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
// Clean sub-screen nav bar: Back chevron + right-aligned title, with an
// optional subsystem icon tucked just left of the title. Same family as
// the Settings / Info screens so every inner screen matches the home menu.
void drawNavBar(const char* title, uint16_t accent, int8_t icon) {
  tft.fillRect(0, 0, W, HEADER_H - 1, C_PANEL);
  tft.drawFastHLine(0, HEADER_H - 1, W, C_EDGE);
  addButton(0, 0, 80, HEADER_H - 1, "Back", nullptr, accent, ST_BACK, ACT_BACK, 0);
  int16_t tx = W - 12 - fontWidth(title, true);
  if (icon >= 0) drawIcon((uint8_t)icon, tx - 17, (HEADER_H - 1) / 2, accent, C_PANEL);
  fontAt(title, tx, 22, true, C_TEXT);
}
void drawHomeFooter() {
  int16_t y = H - FOOTER_H;
  tft.fillRect(0, y, W, FOOTER_H, C_PANEL);
  tft.drawFastHLine(0, y, W, C_EDGE);
  tft.fillCircle(10, y + 8, 2, C_OK);
  printAt("Wi-Fi BLE nRF24 IR live", 17, y + 5, 1, C_MUTED);
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
    uint16_t wellBg = mix565(b.color, face, 46);        // uniform well for every tile
    tft.fillRoundRect(wx, wy, well, well, 7, wellBg);
    drawIcon(icon, wx + well / 2, wy + well / 2, b.color, wellBg);   // full-colour icon
    fontAt(b.label, r.x + (r.w - fontWidth(b.label, true)) / 2, r.y + r.h - 7, true, C_TEXT);  // always white
    if (isSub) tft.fillCircle(r.x + r.w - 8, r.y + 8, 2, live ? C_OK : C_DIM);  // dot carries live/USB state
  } else if (b.style == ST_HEADER) {
    tft.fillRect(r.x, r.y, r.w, r.h, pressed ? b.color : C_PANEL);
    printCentered(b.label, r.x + r.w / 2, r.y + r.h / 2, 2, fg);
  } else if (b.style == ST_ROW) {
    // Module card (home-menu aesthetic): mono id chip + FreeSans name + chevron.
    uint16_t face = pressed ? C_RAISED : C_PANEL;
    tft.fillRoundRect(r.x, r.y, r.w, r.h, 6, face);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, 6, pressed ? b.color : C_EDGE);
    const int16_t chw = 34, chh = 18, chx = r.x + 8, chy = r.y + (r.h - chh) / 2;
    tft.fillRoundRect(chx, chy, chw, chh, 5, mix565(b.color, face, 46));
    tft.drawRoundRect(chx, chy, chw, chh, 5, mix565(b.color, face, 120));
    printAt(b.label, chx + (chw - (int16_t)strlen(b.label) * 6) / 2, chy + 6, 1, b.color);
    if (b.sub) fontAt(b.sub, chx + chw + 10, r.y + r.h / 2 + 5, true, C_TEXT);
    int16_t vx = r.x + r.w - 13, vy = r.y + r.h / 2;
    for (int8_t t = 0; t < 2; t++) {
      tft.drawLine(vx + t, vy - 4, vx + 4 + t, vy, C_DIM); tft.drawLine(vx + 4 + t, vy, vx + t, vy + 4, C_DIM);
    }
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
  } else if (b.style == ST_TAB) {       // one segment of the LOG|CHART control (thumb only)
    bool sel = (b.arg == 1) == liveGraph;
    if (sel)          tft.fillRoundRect(r.x, r.y, r.w, r.h, 4, mix565(b.color, C_PANEL, 55));
    else if (pressed) tft.fillRoundRect(r.x, r.y, r.w, r.h, 4, C_RAISED);
    printAt(b.label, r.x + (r.w - (int16_t)strlen(b.label) * 6) / 2, r.y + (r.h - 8) / 2 + 1, 1, sel ? b.color : C_MUTED);
  } else if (b.style == ST_QMARK) {     // round info button on a module row
    tft.fillRoundRect(r.x, r.y, r.w, r.h, r.w / 2, pressed ? b.color : C_PANEL);
    tft.drawRoundRect(r.x, r.y, r.w, r.h, r.w / 2, b.color);
    printAt("?", r.x + r.w / 2 - 3, r.y + r.h / 2 - 3, 1, pressed ? C_BG : b.color);
  } else if (b.style == ST_RUN) {       // START / HALT action pill (filled, with glyph)
    tft.fillRoundRect(r.x, r.y, r.w, r.h, r.h / 2, pressed ? C_RAISED : b.color);
    uint16_t fgc = pressed ? b.color : C_BG;
    int16_t gy = r.y + r.h / 2, gx = r.x + 16;
    if (b.arg == 0) tft.fillTriangle(gx - 3, gy - 4, gx - 3, gy + 4, gx + 4, gy, fgc);  // play
    else            tft.fillRect(gx - 3, gy - 3, 7, 7, fgc);                             // stop
    fontAt(b.label, gx + 12, gy + 5, true, fgc);
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

// ----- live view + charts + log + info reader are defined below -----

// ===================================================================
// ===================================================================
// LIVE CHARTS, STRUCTURED LOG, INFO ARTICLES  (UI layer, v3 additions)
//   All data here is derived from the telemetry the modules already
//   emit; the serial protocol and module logic are untouched.
// ===================================================================

// ----- small chart primitives (themed) -----
void panelFrame(int16_t y, int16_t h, const char* title, uint16_t acc) {
  tft.fillRoundRect(8, y, W - 16, h, 5, C_PANEL);
  tft.drawRoundRect(8, y, W - 16, h, 5, C_EDGE);
  tft.fillRect(11, y + 5, 3, 9, acc);
  printAt(title, 18, y + 5, 1, C_TEXT);
}
void panelStat(int16_t y, const char* s, uint16_t c) {
  printAt(s, W - 14 - (int16_t)strlen(s) * 6, y + 5, 1, c);
}
void chartRSSI(int16_t px, int16_t py, int16_t pw, int16_t ph, uint16_t acc) {
  const int vmin = -100, vmax = -30; const int gl[2] = {-50, -75};
  uint16_t fill = mix565(acc, C_PANEL, 60);
  for (int g = 0; g < 2; g++) {
    int16_t yy = py + (vmax - gl[g]) * (ph - 1) / (vmax - vmin);
    for (int16_t x = px; x < px + pw; x += 6) tft.drawPixel(x, yy, C_EDGE);
    char lb[5]; snprintf(lb, sizeof(lb), "%d", gl[g]); printAt(lb, px + 1, yy - 8, 1, C_DIM);
  }
  if (gRn < 2) { printCentered("waiting for signal", px + pw / 2, py + ph / 2, 1, C_DIM); return; }
  int16_t base = py + ph - 1, ppx = -1, ppy = 0;
  for (int i = 0; i < gRn; i++) {
    int v = gRget(i); int16_t x = px + (gRn == 1 ? 0 : i * (pw - 1) / (gRn - 1));
    int16_t yy = py + (vmax - v) * (ph - 1) / (vmax - vmin);
    if (yy < py) yy = py; if (yy > base) yy = base;
    tft.drawFastVLine(x, yy, base - yy + 1, fill);
    if (ppx >= 0) tft.drawLine(ppx, ppy, x, yy, acc);
    ppx = x; ppy = yy;
  }
  tft.fillCircle(ppx, ppy, 2, acc);
}
void chartChannels(int16_t px, int16_t py, int16_t pw, int16_t ph, uint16_t acc) {
  const int n = 13; int16_t ph2 = ph - 8, mx = gChanMax < 1 ? 1 : gChanMax, bw = pw / n;
  tft.drawFastHLine(px, py + ph2, pw, C_EDGE);
  for (int ch = 1; ch <= n; ch++) {
    int v = gChan[ch]; int16_t bx = px + (ch - 1) * bw + 1, bwi = bw - 2;
    if (v > 0) { int16_t bh = v * (ph2 - 2) / mx; if (bh < 1) bh = 1;
      tft.fillRect(bx, py + ph2 - bh, bwi, bh, v == gChanMax ? acc : mix565(acc, C_PANEL, 120)); }
    else tft.drawFastHLine(bx, py + ph2 - 1, bwi, C_DIM);
    if (ch == 1 || ch == 6 || ch == 11) { char l[3]; snprintf(l, sizeof(l), "%d", ch);
      printAt(l, bx + bwi / 2 - 3, py + ph - 6, 1, C_DIM); }
  }
}
void chartDist(int16_t px, int16_t py, int16_t pw, int16_t ph, uint16_t acc) {
  const int n = GBK; int16_t ph2 = ph - 8, mx = gDistMax < 1 ? 1 : gDistMax, bw = pw / n;
  tft.drawFastHLine(px, py + ph2, pw, C_EDGE);
  for (int i = 0; i < n; i++) {
    int v = gDist[i]; int16_t bx = px + i * bw + 1, bwi = bw - 2;
    if (v > 0) { int16_t bh = v * (ph2 - 2) / mx; if (bh < 1) bh = 1;
      tft.fillRect(bx, py + ph2 - bh, bwi, bh, v == gDistMax ? acc : mix565(acc, C_PANEL, 120)); }
    else tft.drawFastHLine(bx, py + ph2 - 1, bwi, C_DIM);
  }
  const char* lb[3] = {"-100", "-60", "-20"}; int at[3] = {0, 4, 7};
  for (int k = 0; k < 3; k++) printAt(lb[k], px + at[k] * bw, py + ph - 6, 1, C_DIM);
}
void chartRate(int16_t px, int16_t py, int16_t pw, int16_t ph, uint16_t acc) {
  if (gRaten < 2) { printCentered("waiting for data", px + pw / 2, py + ph / 2, 1, C_DIM); return; }
  int16_t mx = gRateMax < 1 ? 1 : gRateMax, base = py + ph - 1;
  char top[12]; snprintf(top, sizeof(top), "max %d", mx); printAt(top, px + 1, py - 2, 1, C_DIM);
  for (int i = 0; i < gRaten; i++) {
    int v = gRateGet(i); int16_t bh = v * (ph - 2) / mx; if (v > 0 && bh < 1) bh = 1;
    int16_t bx = px + i * pw / gRaten, bw = pw / gRaten; if (bw < 1) bw = 1;
    if (bh > 0) tft.fillRect(bx, base - bh, bw > 2 ? bw - 1 : bw, bh, acc);
  }
}
uint8_t chartKind() {
  const char* id = SUBS[curSub].mods[curMod].id;
  if (!strcmp(id, "1.1")) return 0;
  if (!strcmp(id, "2.1") || !strcmp(id, "2.3")) return 1;
  return 2;
}

// ----- LIVE chrome: header + LOG/CHART tabs + START/HALT -----
void drawLiveChrome() {
  const Subsystem& s = SUBS[curSub];
  tft.fillScreen(C_BG);
  const char* id = SUBS[curSub].mods[curMod].id;
  const char* title = moduleTitle(id);
  // --- header: Back + title, with the module id in an accent chip ---
  tft.fillRect(0, 0, W, HEADER_H - 1, C_PANEL);
  tft.drawFastHLine(0, HEADER_H - 1, W, C_EDGE);
  addButton(0, 0, 80, HEADER_H - 1, "Back", nullptr, s.color, ST_BACK, ACT_BACK, 0);
  int16_t tx = W - 12 - fontWidth(title, true);
  fontAt(title, tx, 22, true, C_TEXT);
  const int16_t chw = 34, chh = 18, chx = tx - 8 - chw, chy = (HEADER_H - 1 - chh) / 2;
  tft.fillRoundRect(chx, chy, chw, chh, 5, mix565(s.color, C_PANEL, 46));
  tft.drawRoundRect(chx, chy, chw, chh, 5, mix565(s.color, C_PANEL, 120));
  printAt(id, chx + (chw - (int16_t)strlen(id) * 6) / 2, chy + 6, 1, s.color);
  // --- control band: iOS-style segmented LOG|CHART + START/HALT pill ---
  tft.fillRect(0, LV_BAND, W, LV_BAND_H, C_PANEL);
  tft.drawFastHLine(0, LV_BAND + LV_BAND_H, W, C_EDGE);
  const int16_t contX = 8, contW = 116, contH = 20, contY = LV_BAND + (LV_BAND_H - contH) / 2, segW = (contW - 4) / 2;
  tft.fillRoundRect(contX, contY, contW, contH, 5, C_BG);
  tft.drawRoundRect(contX, contY, contW, contH, 5, C_EDGE);
  addButton(contX + 2,        contY + 2, segW, contH - 4, "LOG",   nullptr, s.color, ST_TAB, ACT_VIEW, 0);
  addButton(contX + 2 + segW, contY + 2, segW, contH - 4, "CHART", nullptr, s.color, ST_TAB, ACT_VIEW, 1);
  addButton(W - 102, LV_BAND + (LV_BAND_H - 22) / 2, 94, 22, running ? "HALT" : "START", nullptr,
            running ? C_ERR : s.color, ST_RUN, ACT_TOGGLE, running ? 1 : 0);
  listDirty = true; statusDirty = true; graphDirty = true;
}
void drawSeen() {
  if (screen != SCR_LIVE) return;
  tft.fillRect(128, LV_BAND + 2, 86, LV_BAND_H - 4, C_PANEL);
  const char* id = liveId();
  const char* what = !strcmp(id, "4.1") ? "sweeps" : (!strcmp(id, "4.6") || !strcmp(id, "7.3")) ? "sent"
                   : (!strcmp(id, "7.1") || !strcmp(id, "7.6")) ? "reports"
                   : id[0] == '7' ? "captures" : "seen";
  char c[18]; snprintf(c, sizeof(c), "%lu %s", rowTotal, what);
  printAt(c, 132, LV_BAND + LV_BAND_H / 2 - 3, 1, C_MUTED);
}
void drawStatusLine() {
  tft.fillRect(0, H - 14, W, 14, C_PANEL);
  if (statusLine[0]) printAt(statusLine, 6, H - 12, 1, statusColor);
}

// ----- the structured log view (time / device / detail / rssi + bars) -----
void drawLog() {
  const int16_t x = 6, y0 = LV_Y0, rowH = 18;
  tft.fillRect(0, y0, W, (H - 14) - y0, C_BG);
  // column header
  tft.fillRect(0, y0, W, 13, C_PANEL);
  bool nrf = curSub == 2, spec = nrf && !strcmp(liveId(), "4.1"), ir = curSub == 4;
  printAt("TIME",   x,     y0 + 3, 1, C_DIM);
  if (ir) {
    const char* id = liveId(); const char* mid = "CAPTURE"; const char* rt = "PULSES";
    if (!strcmp(id, "7.1"))      { mid = "CARRIER";          rt = "STATE"; }
    else if (!strcmp(id, "7.2")) { mid = "BURST / TIMINGS us"; }
    else if (!strcmp(id, "7.3")) { mid = "REPLAY";           rt = "STATE"; }
    else if (!strcmp(id, "7.4")) { mid = "PROTOCOL / CODE";  rt = "BITS"; }
    else if (!strcmp(id, "7.5")) { mid = "CAPTURE";          rt = "DUTY"; }
    else if (!strcmp(id, "7.6")) { mid = "TEST PULSES";      rt = "PER s"; }
    printAt(mid, x + 44, y0 + 3, 1, C_DIM);
    printAt(rt, W - 8 - (int16_t)strlen(rt) * 6, y0 + 3, 1, C_DIM);
  } else {
    printAt(nrf ? (spec ? "SWEEP" : "CAPTURE") : "DEVICE / SSID", x + 44, y0 + 3, 1, C_DIM);
    printAt(nrf ? (spec ? "PEAK" : "MHz") : "RSSI", W - 92, y0 + 3, 1, C_DIM);
    printAt(nrf ? (spec ? "LEVEL" : "") : "SIGNAL", W - 44, y0 + 3, 1, C_DIM);
  }
  tft.drawFastHLine(0, y0 + 13, W, C_EDGE);
  int16_t ry0 = y0 + 15;
  if (rowCount == 0) {
    const char* id = liveId();
    bool irRx = !strcmp(id, "7.2") || !strcmp(id, "7.4") || !strcmp(id, "7.5");
    printAt(running ? (irRx ? "Aim a remote at the receiver, press a key"
                            : "Listening for results...")
                    : (!strcmp(id, "7.3") ? "Tap START to send the last captured code"
                                          : "Tap START to run the module"),
            x + 2, ry0 + 6, 1, C_MUTED);
    return;
  }
  uint8_t show = rowCount > 7 ? 7 : rowCount;
  for (uint8_t i = 0; i < show; i++) {
    uint8_t idx = (rowHead + ROWS - 1 - i) % ROWS; Row& rw = rowBuf[idx];
    int16_t ry = ry0 + i * rowH;
    if (i & 1) tft.fillRect(0, ry, W, rowH, C_PANEL);
    char ts[8]; uint32_t sec = rw.t / 1000; snprintf(ts, sizeof(ts), "%02lu:%02lu", (sec / 60) % 100, sec % 60);
    printAt(ts, x, ry + 2, 1, C_DIM);
    uint16_t lc = rw.lvl == 3 ? C_ERR : (rw.lvl == 2 ? C_WARN : rw.color);
    tft.fillCircle(x + 38, ry + 5, 2, lc);
    char pa[31]; strncpy(pa, rw.a, 30); pa[30] = 0;
    printAt(pa, x + 44, ry + 2, 1, C_TEXT);
    if (rw.b[0]) printAt(rw.b, x + 44, ry + 10, 1, C_MUTED);
    if (rw.rssi) {
      uint16_t rc = rssiColor(rw.rssi);
      char rs[10]; snprintf(rs, sizeof(rs), "%d", (int)rw.rssi);
      printAt(rs, 284 - (int16_t)strlen(rs) * 6, ry + 2, 1, rc);
      uint8_t bars = rssiBars(rw.rssi);
      for (uint8_t k = 0; k < 4; k++) {
        int16_t bh = 3 + k * 3, bx = W - 40 + k * 7, by = ry + 14 - bh;
        tft.fillRect(bx, by, 5, bh, k < bars ? rc : C_EDGE);
      }
    } else if (rw.mhz) {                       // nRF24: frequency instead of RSSI
      char fs[8]; snprintf(fs, sizeof(fs), "%d", (int)rw.mhz);
      printAt(fs, 284 - (int16_t)strlen(fs) * 6, ry + 2, 1, C_MUTED);
    } else if (rw.tag[0]) {                    // IR: bits / pulses / duty
      printAt(rw.tag, W - 8 - (int16_t)strlen(rw.tag) * 6, ry + 2, 1, lc);
    }
    if (rw.meter != 0xFF) {                    // nRF24 4.1: peak level as bars
      uint16_t mc = rw.meter ? nrfRamp(rw.meter) : C_EDGE;
      uint8_t bars = (rw.meter + 7) / 8;
      for (uint8_t k = 0; k < 4; k++) {
        int16_t bh = 3 + k * 3, bx = W - 40 + k * 7, by = ry + 14 - bh;
        tft.fillRect(bx, by, 5, bh, k < bars ? mc : C_EDGE);
      }
    }
  }
}

// ----- the chart view: signal timeline + adaptive second panel -----
void drawCharts() {
  if (curSub == 2) { drawNrfCharts(); return; }   // nRF24 has its own graphs
  if (curSub == 4) { drawIrCharts(); return; }    // so does IR
  tft.fillRect(0, LV_Y0, W, (H - 14) - LV_Y0, C_BG);
  const Subsystem& s = SUBS[curSub]; uint16_t acc = s.color;
  int16_t p1 = LV_Y0 + 2, ph = 72, p2 = p1 + 76;
  panelFrame(p1, ph, "SIGNAL  dBm", acc);
  char s1[12]; if (gHaveRssi) snprintf(s1, sizeof(s1), "%d dBm", gLastRssi); else snprintf(s1, sizeof(s1), "--");
  panelStat(p1, s1, gHaveRssi ? rssiColor(gLastRssi) : C_DIM);
  chartRSSI(14, p1 + 18, W - 28, ph - 24, acc);
  uint8_t k = chartKind();
  const char* t2 = (k == 0) ? "CHANNEL MAP" : (k == 1) ? "RSSI SPREAD"
                 : (!strcmp(SUBS[curSub].mods[curMod].id, "1.3") ? "PACKETS / s" : "EVENT RATE / s");
  panelFrame(p2, ph, t2, acc);
  char s2[16];
  if (k == 0) { int pk = peakChannel(); if (pk) snprintf(s2, sizeof(s2), "peak ch%d", pk); else snprintf(s2, sizeof(s2), "--");
    panelStat(p2, s2, acc); chartChannels(14, p2 + 18, W - 28, ph - 24, acc); }
  else if (k == 1) { snprintf(s2, sizeof(s2), "n=%lu", rowTotal); panelStat(p2, s2, acc);
    chartDist(14, p2 + 18, W - 28, ph - 24, acc); }
  else { snprintf(s2, sizeof(s2), "%u/s", gRateLast()); panelStat(p2, s2, acc);
    chartRate(14, p2 + 18, W - 28, ph - 24, acc); }
}

// ===================================================================
// nRF24 GRAPHS  (UI layer; fed by uiOnNrf from the same 4.x telemetry)
//   4.1      SPECTRUM: per-channel bars (colour by level), peak hold,
//            Wi-Fi 1/6/11 bands, sweep cursor, waterfall of past sweeps.
//            Drawn once in full, then only changed columns are redrawn.
//   4.2-4.4  packets per channel + hop cursor, and packets / s.
//   4.5      the 32-slot capture ring, and stored / s.
//   4.6      ack / no-ack per frame, and frames / s.
// ===================================================================
const int16_t SP_X = 36;                         // plot left; 2 px per channel -> 252 px
const int16_t SP_Y = LV_Y0 + 30, SP_H = 54;      // plot top + height (level 32 = SP_H)
const int16_t SP_B = SP_Y + SP_H;                // axis line
const int16_t WF_Y = SP_B + 16;                  // waterfall top, 2 px per sweep row

bool nrfInBand(uint8_t c) {                      // Wi-Fi ch 1/6/11, 22 MHz wide
  return (c >= 1 && c <= 22) || (c >= 26 && c <= 47) || (c >= 51 && c <= 72);
}
uint16_t nrfColBg(uint8_t c) { return nrfInBand(c) ? mix565(C_WIFI, C_PANEL, lightMode ? 16 : 22) : C_PANEL; }
void nrfBuildLUT() {
  for (uint8_t lv = 0; lv <= NRF_MAXLV; lv++) {
    nrfRampLUT[lv] = nrfRamp(lv);
    // curved alpha so 1-3 hit noise stays dim and real carriers pop
    uint8_t a = lv ? (uint8_t)(255 * powf((float)lv / NRF_MAXLV, 0.75f)) : 0;
    nrfWfLUT[lv] = lv ? mix565(nrfRampLUT[lv], C_BG, a) : C_BG;
  }
}
// One channel column: background (band tint + grid dots), 3-tone bar, peak tick.
void nrfDrawCol(uint8_t c, bool force) {
  int16_t x = SP_X + c * 2;
  uint8_t lv = nrfLvl[c];
  int16_t bh = (lv * SP_H + NRF_MAXLV / 2) / NRF_MAXLV; if (lv && bh < 1) bh = 1;
  int16_t top = SP_B - bh;
  uint8_t pk = (nrfPeak16[c] + 8) / 16; if (pk > NRF_MAXLV) pk = NRF_MAXLV;
  int16_t py = pk ? SP_B - (pk * SP_H + NRF_MAXLV / 2) / NRF_MAXLV : -1;
  if (py < SP_Y && py >= 0) py = SP_Y;
  if (py >= top - 1) py = -1;                    // hidden behind the bar
  if (!force && top == nrfDrawnTop[c] && py == nrfDrawnPk[c]) return;
  nrfDrawnTop[c] = top; nrfDrawnPk[c] = py;
  uint16_t bg = nrfColBg(c);
  if (top > SP_Y) tft.fillRect(x, SP_Y, 2, top - SP_Y, bg);
  if (c % 3 == 0) {                              // dotted grid at levels 32 and 16
    if (SP_Y < top) tft.drawPixel(x, SP_Y, C_EDGE);
    if (SP_Y + SP_H / 2 < top) tft.drawPixel(x, SP_Y + SP_H / 2, C_EDGE);
  }
  if (bh > 0) {                                  // bright cap fading into the panel
    uint16_t col = nrfRampLUT[lv];
    int16_t t1 = (bh + 2) / 3, rest = bh - t1, t2 = rest / 2;
    tft.fillRect(x, top, 2, t1, col);
    if (t2) tft.fillRect(x, top + t1, 2, t2, mix565(col, bg, 170));
    if (rest - t2) tft.fillRect(x, top + t1 + t2, 2, rest - t2, mix565(col, bg, 95));
  }
  if (py >= SP_Y) tft.drawFastHLine(x, py, 2, nrfRampLUT[pk]);
}
bool nrfIsTick(uint8_t c) { return c % 25 == 0; }
// Small marker under the axis on the channel being swept right now.
void nrfDrawCursor(int16_t ch) {
  if (ch == nrfCursorCh) return;
  if (nrfCursorCh >= 0) {
    int16_t ox = SP_X + nrfCursorCh * 2;
    tft.fillRect(ox, SP_B + 1, 2, 3, C_PANEL);
    if (nrfIsTick(nrfCursorCh)) tft.drawFastVLine(ox, SP_B + 1, 3, C_DIM);
  }
  if (ch >= 0) tft.fillRect(SP_X + ch * 2, SP_B + 1, 2, 3, C_TEXT);
  nrfCursorCh = ch;
}
void nrfDrawWaterfall() {
  uint16_t line[NRF_CH * 2];
  for (uint8_t r = 0; r < NRF_WF_ROWS; r++) {
    if (r < nrfWfCount) {
      const uint8_t* row = nrfWf[(nrfWfHead + NRF_WF_ROWS - 1 - r) % NRF_WF_ROWS];   // newest on top
      for (uint8_t c = 0; c < NRF_CH; c++) line[2 * c] = line[2 * c + 1] = nrfWfLUT[row[c]];
    } else {
      for (uint16_t i = 0; i < NRF_CH * 2; i++) line[i] = C_BG;
    }
    tft.drawRGBBitmap(SP_X, WF_Y + r * 2,     line, NRF_CH * 2, 1);
    tft.drawRGBBitmap(SP_X, WF_Y + r * 2 + 1, line, NRF_CH * 2, 1);
  }
}
void nrfDrawStat() {
  char s[24]; uint8_t pk = 0;
  for (uint8_t c = 1; c < NRF_CH; c++) if (nrfLvl[c] > nrfLvl[pk]) pk = c;
  if (!running && !nrfSweeps)  snprintf(s, sizeof(s), "tap START");
  else if (nrfLvl[pk])         snprintf(s, sizeof(s), "peak ch%u %u/%u", pk, nrfLvl[pk], NRF_MAXLV);
  else if (!nrfSweeps)         snprintf(s, sizeof(s), "sweeping...");
  else                         snprintf(s, sizeof(s), "quiet");
  if (!strcmp(s, nrfStat)) return;
  strcpy(nrfStat, s);
  int16_t y = LV_Y0 + 2;
  tft.fillRect(W - 14 - 17 * 6, y + 4, 17 * 6, 10, C_PANEL);
  panelStat(y, s, nrfLvl[pk] ? nrfRampLUT[nrfLvl[pk]] : C_MUTED);
}
void drawNrfSpectrum() {
  nrfBuildLUT();
  const int16_t py = LV_Y0 + 2, ph = (H - 14) - 1 - py;
  panelFrame(py, ph, "SPECTRUM 2400-2525 MHz", C_NRF);
  const uint8_t wc[3] = {12, 37, 62}; const char* wn[3] = {"WiFi 1", "WiFi 6", "WiFi 11"};
  uint16_t wl = mix565(C_WIFI, C_PANEL, 170);
  for (uint8_t i = 0; i < 3; i++)
    printAt(wn[i], SP_X + wc[i] * 2 + 1 - (int16_t)strlen(wn[i]) * 3, SP_Y - 10, 1, wl);
  printAt("32", SP_X - 16, SP_Y - 3, 1, C_DIM);
  printAt("16", SP_X - 16, SP_Y + SP_H / 2 - 3, 1, C_DIM);
  tft.drawFastHLine(SP_X, SP_B, NRF_CH * 2, C_EDGE);
  for (uint8_t c = 0; c < NRF_CH; c += 25) {
    int16_t x = SP_X + c * 2;
    tft.drawFastVLine(x, SP_B + 1, 3, C_DIM);
    char l[6]; snprintf(l, sizeof(l), "%u", 2400 + c);
    printAt(l, x + 1 - 12, SP_B + 5, 1, C_DIM);
  }
  printAt("now", 12, WF_Y, 1, C_DIM);
  printAt("old", 12, WF_Y + NRF_WF_ROWS * 2 - 8, 1, C_DIM);
  for (uint8_t c = 0; c < NRF_CH; c++) nrfDrawCol(c, true);
  nrfCursorCh = -1; nrfDrawCursor(running ? nrfChannel : -1);
  nrfDrawWaterfall(); nrfWfDirty = false;
  nrfStat[0] = 0; nrfDrawStat();
}
// Incremental 4.1 refresh: decay peaks, redraw only columns that changed.
void nrfSpectrumTick() {
  unsigned long now = millis();
  uint32_t dec = (now - nrfPeakMs) * 64 / 1000;   // 4 levels/s, in x16 units
  nrfPeakMs = now;
  for (uint8_t c = 0; c < NRF_CH; c++) {
    uint16_t f = nrfLvl[c] * 16;
    nrfPeak16[c] = (nrfPeak16[c] > f + dec) ? nrfPeak16[c] - dec : f;
    nrfDrawCol(c, false);
  }
  nrfDrawCursor(running ? nrfChannel : -1);
  if (nrfWfDirty) { nrfWfDirty = false; nrfDrawWaterfall(); }
  nrfDrawStat();
}
// The channel the radio is parked on for the running 4.x module.
uint8_t nrfLiveChannel() {
  const char* id = liveId();
  if (!strcmp(id, "4.5")) return NRF_STORE_CH;
  if (!strcmp(id, "4.6")) return NRF_TEST_CH;
  return nrfChannel;
}
// 4.2-4.4: packets seen per channel over the hop range, with a hop cursor.
void chartNrfHits(int16_t py, int16_t ph) {
  const uint8_t n = NRF_SNIFF_END_CH + 1; const int16_t bw = 3, x0 = SP_X;
  const int16_t top = py + 22, bot = py + ph - 16, h = bot - top;
  for (uint8_t c = 0; c < n; c++) {
    int16_t x = x0 + c * bw; uint16_t bg = nrfColBg(c);
    if (bg != C_PANEL) tft.fillRect(x, top, bw, h, bg);
    uint16_t v = nrfHitCh[c];
    if (v) { int16_t bh = v * (h - 2) / (nrfHitMax ? nrfHitMax : 1); if (bh < 2) bh = 2;
      tft.fillRect(x, bot - bh, bw - 1, bh, v == nrfHitMax ? C_NRF : mix565(C_NRF, C_PANEL, 130)); }
  }
  tft.drawFastHLine(x0, bot, n * bw, C_EDGE);
  for (uint8_t c = 0; c < n; c += 20) {
    char l[6]; snprintf(l, sizeof(l), "%u", 2400 + c);
    printAt(l, x0 + c * bw + 1 - 12, bot + 6, 1, C_DIM);
  }
  char mx[8]; snprintf(mx, sizeof(mx), "%u", nrfHitMax);
  printAt(nrfHitMax ? mx : "0", x0 - 6 - (int16_t)strlen(nrfHitMax ? mx : "0") * 6, top - 3, 1, C_DIM);
  if (!nrfHitMax) printCentered(running ? "hopping, no packets yet" : "tap START to listen",
                                x0 + n * bw / 2, top + h / 2, 1, C_DIM);
  if (running) {                                   // hop cursor
    uint8_t c = nrfLiveChannel(); int16_t cx = x0 + c * bw + 1;
    tft.fillTriangle(cx - 3, bot + 5, cx + 3, bot + 5, cx, bot + 1, C_TEXT);
  }
}
// 4.5: the 32-slot RAM ring; filled slots tinted, newest slot bright.
void chartNrfRing(int16_t py, int16_t ph) {
  const int16_t gap = 3, sw = (W - 28 - 15 * gap) / 16, sh = 22, x0 = 14, y0 = py + 22;
  uint8_t filled = nrfStored > NRF_RING_SIZE ? NRF_RING_SIZE : (uint8_t)nrfStored;
  int16_t newest = nrfStored ? (nrfRingHead + NRF_RING_SIZE - 1) % NRF_RING_SIZE : -1;
  for (uint8_t i = 0; i < NRF_RING_SIZE; i++) {
    int16_t x = x0 + (i % 16) * (sw + gap), y = y0 + (i / 16) * (sh + 4);
    bool used = nrfStored >= NRF_RING_SIZE || i < filled;
    if (i == newest)  tft.fillRoundRect(x, y, sw, sh, 3, C_NRF);
    else if (used)    tft.fillRoundRect(x, y, sw, sh, 3, mix565(C_NRF, C_PANEL, 90));
    else              tft.drawRoundRect(x, y, sw, sh, 3, C_EDGE);
  }
  char l[44]; snprintf(l, sizeof(l), "ch%u %u MHz  pipe1 E7E7E7E7E7", NRF_STORE_CH, 2400 + NRF_STORE_CH);
  printAt(l, x0, py + ph - 12, 1, C_MUTED);
}
// 4.6: one bar per frame, tall green = acked, short red = no ack.
void chartNrfAck(int16_t py, int16_t ph) {
  const int16_t x0 = 14, pw = W - 28, top = py + 22, bot = py + ph - 16, h = bot - top;
  const int16_t bw = pw / GHN;
  tft.drawFastHLine(x0, bot, pw, C_EDGE);
  if (!nrfAckN) printCentered(running ? "sending..." : "tap START to send test frames", x0 + pw / 2, top + h / 2, 1, C_DIM);
  for (uint8_t i = 0; i < nrfAckN; i++) {
    uint8_t v = nrfAck[(nrfAckH + GHN - nrfAckN + i) % GHN];
    int16_t bh = v ? h - 2 : h / 3;
    tft.fillRect(x0 + i * bw, bot - bh, bw > 1 ? bw - 1 : 1, bh, v ? C_OK : C_ERR);
  }
  char l[44]; snprintf(l, sizeof(l), "ch%u %u MHz  CYBER  sent %lu  ack %lu",
                       NRF_TEST_CH, 2400 + NRF_TEST_CH, nrfAckTotal, nrfAckOk);
  printAt(l, x0, bot + 5, 1, C_MUTED);
}
void drawNrfCharts() {
  tft.fillRect(0, LV_Y0, W, (H - 14) - LV_Y0, C_BG);
  const char* id = liveId();
  if (!strcmp(id, "4.1")) { drawNrfSpectrum(); return; }
  const int16_t p1 = LV_Y0 + 2, h1 = 88, p2 = p1 + h1 + 4, h2 = (H - 14) - 2 - p2;
  char st[24];
  if (!strcmp(id, "4.6")) {
    panelFrame(p1, h1, "LINK  ACK PER FRAME", C_NRF);
    if (nrfAckTotal) snprintf(st, sizeof(st), "ack %lu%%", nrfAckOk * 100 / nrfAckTotal); else snprintf(st, sizeof(st), "--");
    panelStat(p1, st, nrfAckTotal ? (nrfAckOk * 2 >= nrfAckTotal ? C_OK : C_WARN) : C_DIM);
    chartNrfAck(p1, h1);
  } else if (!strcmp(id, "4.5")) {
    panelFrame(p1, h1, "RING BUFFER  32 SLOTS", C_NRF);
    snprintf(st, sizeof(st), "stored %lu", nrfStored); panelStat(p1, st, C_NRF);
    chartNrfRing(p1, h1);
  } else {
    panelFrame(p1, h1, "PACKETS PER CHANNEL", C_NRF);
    uint8_t c = nrfLiveChannel();
    if (running) snprintf(st, sizeof(st), "on ch%u %u", c, 2400 + c); else snprintf(st, sizeof(st), "--");
    panelStat(p1, st, running ? C_TEXT : C_DIM);
    chartNrfHits(p1, h1);
  }
  const char* t2 = !strcmp(id, "4.3") ? "HITS / s" : !strcmp(id, "4.5") ? "STORED / s"
                 : !strcmp(id, "4.6") ? "FRAMES / s" : "PACKETS / s";
  panelFrame(p2, h2, t2, C_NRF);
  snprintf(st, sizeof(st), "%u/s", gRateLast()); panelStat(p2, st, C_NRF);
  chartRate(14, p2 + 18, W - 28, h2 - 24, C_NRF);
}

// ===================================================================
// IR GRAPHS  (UI layer; fed by uiOnIr and the last capture, lastIrRaw)
//   7.2-7.5  WAVEFORM of the last capture: marks (IR on) high, spaces low.
//   7.2      + pulse-width histogram (marks vs spaces).
//   7.5      + mark / space split (duty cycle).
//   7.3/7.4  + the code: protocol, value, bits (7.3 = what START sends).
//   7.1      38kHz square wave + how to check the LED.
//   7.6      the 26us / 10ms test pulse train + pulses / s.
// ===================================================================
// Mark/space train, scaled to fit pw. v[0] is a mark; marks alternate with spaces.
void irDrawWave(const uint16_t* v, uint16_t n, int16_t px, int16_t py, int16_t pw, int16_t ph, uint16_t c) {
  uint64_t total = 0;
  for (uint16_t i = 0; i < n; i++) total += v[i];
  if (!total) return;
  const int16_t hi = py, lo = py + ph - 1;
  const uint16_t fill = mix565(c, C_PANEL, 70);
  uint64_t t = 0;
  tft.drawFastHLine(px, lo, pw, C_EDGE);
  for (uint16_t i = 0; i < n; i++) {
    uint64_t t1 = t + v[i];
    int16_t x0 = px + (int16_t)(t * (pw - 1) / total), x1 = px + (int16_t)(t1 * (pw - 1) / total);
    if (!(i & 1)) {                                // mark: IR on
      if (x1 > x0) tft.fillRect(x0, hi + 1, x1 - x0, lo - hi - 1, fill);
      tft.drawFastVLine(x0, hi, lo - hi + 1, c);
      tft.drawFastVLine(x1, hi, lo - hi + 1, c);
      tft.drawFastHLine(x0, hi, x1 - x0 + 1, c);
    } else {
      tft.drawFastHLine(x0, lo, x1 - x0 + 1, c);   // space: IR off
    }
    t = t1;
  }
}
uint32_t irTotalUs() { uint32_t t = 0; for (uint16_t i = 0; i < lastIrRawLen; i++) t += lastIrRaw[i]; return t; }

// 7.2: how many marks / spaces fall in each width bucket (ms).
void irDrawHist(int16_t px, int16_t py, int16_t pw, int16_t ph) {
  static const uint16_t edge[7] = {400, 800, 1200, 2000, 3000, 5000, 10000};
  static const char* lb[8] = {"<.4", ".8", "1.2", "2", "3", "5", "10", ">10"};
  uint16_t mk[8] = {0}, sp[8] = {0}, mx = 1;
  for (uint16_t i = 0; i < lastIrRawLen; i++) {
    uint8_t b = 0; while (b < 7 && lastIrRaw[i] >= edge[b]) b++;
    uint16_t& cnt = (i & 1) ? sp[b] : mk[b];
    if (++cnt > mx) mx = cnt;
  }
  const int16_t ph2 = ph - 9, bw = pw / 8, half = (bw - 8) / 2;
  tft.drawFastHLine(px, py + ph2, pw, C_EDGE);
  for (uint8_t b = 0; b < 8; b++) {
    int16_t bx = px + b * bw + 3;
    if (mk[b]) { int16_t h = mk[b] * (ph2 - 1) / mx; if (h < 1) h = 1; tft.fillRect(bx, py + ph2 - h, half, h, C_IR); }
    if (sp[b]) { int16_t h = sp[b] * (ph2 - 1) / mx; if (h < 1) h = 1; tft.fillRect(bx + half + 2, py + ph2 - h, half, h, C_MUTED); }
    printAt(lb[b], px + b * bw + (bw - (int16_t)strlen(lb[b]) * 6) / 2, py + ph - 7, 1, C_DIM);
  }
}

// 7.5: one bar split into mark time and space time.
void irDrawDuty(int16_t p2, int16_t h2) {
  uint32_t mk = 0, sp = 0;
  for (uint16_t i = 0; i < lastIrRawLen; i++) { if (i & 1) sp += lastIrRaw[i]; else mk += lastIrRaw[i]; }
  uint32_t tot = mk + sp; if (!tot) return;
  const int16_t bx = 14, bw = W - 28, by = p2 + 20, bh = 14;
  int16_t mw = (int16_t)((uint64_t)mk * bw / tot);
  tft.fillRoundRect(bx, by, bw, bh, 3, C_EDGE);
  if (mw > 0) tft.fillRoundRect(bx, by, mw, bh, 3, C_IR);
  char l[48]; snprintf(l, sizeof(l), "mark %.1f ms", mk / 1000.0f);
  printAt(l, bx, by + bh + 6, 1, C_IR);
  snprintf(l, sizeof(l), "space %.1f ms", sp / 1000.0f);
  printAt(l, bx + bw - (int16_t)strlen(l) * 6, by + bh + 6, 1, C_MUTED);
}

// 7.3 / 7.4: protocol + value, large.
void irDrawCode(int16_t p2, int16_t h2, bool replay) {
  if (!lastIrCaptured) { printCentered("no code captured yet", W / 2, p2 + h2 / 2 + 4, 1, C_DIM); return; }
  String proto = typeToString(lastIrProtocol);
  bool raw = replay && lastIrProtocol == decode_type_t::UNKNOWN;
  panelStat(p2, raw ? "RAW" : proto.c_str(), C_IR);
  char v[24], sub[32];
  if (raw) { snprintf(v, sizeof(v), "%u timings", lastIrRawLen); snprintf(sub, sizeof(sub), "sent raw at 38kHz"); }
  else { snprintf(v, sizeof(v), "0x%s", uint64ToString(lastIrValue, 16).c_str()); snprintf(sub, sizeof(sub), "%u bits", lastIrBits); }
  uint8_t sz = strlen(v) * 12 <= W - 28 ? 2 : 1;
  printCentered(v, W / 2, p2 + 32, sz, C_TEXT);
  printCentered(sub, W / 2, p2 + h2 - 10, 1, C_MUTED);
}

void drawIrCharts() {
  tft.fillRect(0, LV_Y0, W, (H - 14) - LV_Y0, C_BG);
  const char* id = liveId();
  const int16_t p1 = LV_Y0 + 2, h1 = 88, p2 = p1 + h1 + 4, h2 = (H - 14) - 2 - p2;
  const int16_t cx = 14, cy = p1 + 20, cw = W - 28, ch = h1 - 36, ly = p1 + h1 - 11;
  char st[32];
  if (!strcmp(id, "7.1")) {
    panelFrame(p1, h1, "CARRIER  TX", C_IR);
    snprintf(st, sizeof(st), running ? "38.0 kHz  GPIO%u" : "off  GPIO%u", IR_TX_PIN);
    panelStat(p1, st, running ? C_OK : C_DIM);
    if (running) {
      int16_t per = cw / 8;
      for (uint8_t k = 0; k < 8; k++) {
        int16_t x = cx + k * per;
        tft.fillRect(x, cy + 1, per / 2, ch - 2, mix565(C_IR, C_PANEL, 70));
        tft.drawFastHLine(x, cy, per / 2, C_IR);
        tft.drawFastVLine(x, cy, ch, C_IR); tft.drawFastVLine(x + per / 2, cy, ch, C_IR);
        tft.drawFastHLine(x + per / 2, cy + ch - 1, per - per / 2, C_IR);
      }
    } else {
      tft.drawFastHLine(cx, cy + ch - 1, cw, C_DIM);
      printCentered("tap START to run the carrier", W / 2, cy + ch / 2, 1, C_DIM);
    }
    printAt("26.3 us period, 50% duty", cx, ly, 1, C_DIM);
    panelFrame(p2, h2, "CHECK THE LED", C_IR);
    printAt("Look at the IR LED through a phone", 18, p2 + 24, 1, C_TEXT);
    printAt("camera: it glows faint purple while", 18, p2 + 36, 1, C_TEXT);
    printAt("the carrier runs. Front cameras work best.", 18, p2 + 48, 1, C_MUTED);
    return;
  }
  if (!strcmp(id, "7.6")) {
    panelFrame(p1, h1, "TEST PULSES", C_IR);
    snprintf(st, sizeof(st), "total %lu", irLastPulses); panelStat(p1, st, running ? C_IR : C_DIM);
    tft.drawFastHLine(cx, cy + ch - 1, cw, running ? C_IR : C_DIM);
    if (running) for (uint8_t k = 0; k < 10; k++) {
      int16_t x = cx + 4 + k * (cw - 8) / 10;
      tft.drawFastVLine(x, cy, ch, C_IR); tft.drawFastVLine(x + 1, cy, ch, C_IR);
    }
    else printCentered("tap START to fire test pulses", W / 2, cy + ch / 2, 1, C_DIM);
    snprintf(st, sizeof(st), "%lu us pulse every %lu ms", (unsigned long)IR_TEST_PULSE_US, (unsigned long)IR_TEST_PERIOD_MS);
    printAt(st, cx, ly, 1, C_DIM);
    panelFrame(p2, h2, "PULSES / s", C_IR);
    snprintf(st, sizeof(st), "%u/s", gRateLast()); panelStat(p2, st, C_IR);
    chartRate(14, p2 + 18, W - 28, h2 - 24, C_IR);
    return;
  }
  // 7.2 - 7.5: the last capture
  bool replay = !strcmp(id, "7.3");
  bool have = lastIrCaptured && lastIrRawLen && (replay || rowTotal > 0);
  panelFrame(p1, h1, replay ? "WAVEFORM  CODE TO SEND" : "WAVEFORM  LAST CAPTURE", C_IR);
  if (have) {
    uint32_t tot = irTotalUs();
    snprintf(st, sizeof(st), "%u pulses  %lu.%lu ms", lastIrRawLen, (unsigned long)(tot / 1000), (unsigned long)(tot % 1000 / 100));
    panelStat(p1, st, C_IR);
    irDrawWave(lastIrRaw, lastIrRawLen, cx, cy, cw, ch, C_IR);
    printAt("high = IR on", cx, ly, 1, C_DIM);
    printAt("time ->", cx + cw - 42, ly, 1, C_DIM);
  } else {
    tft.drawFastHLine(cx, cy + ch - 1, cw, C_DIM);
    printCentered(replay ? "capture a code with 7.2, 7.4 or 7.5" : (running ? "aim a remote here and press a key"
                                                                           : "tap START, then press a remote key"),
                  W / 2, cy + ch / 2, 1, C_DIM);
  }
  if (!strcmp(id, "7.2")) {
    panelFrame(p2, h2, "PULSE WIDTHS  ms", C_IR);
    panelStat(p2, "space", C_MUTED);
    printAt("mark", W - 14 - 5 * 6 - 8 - 4 * 6, p2 + 5, 1, C_IR);
    if (have) irDrawHist(14, p2 + 18, W - 28, h2 - 22);
  } else if (!strcmp(id, "7.5")) {
    panelFrame(p2, h2, "MARK vs SPACE", C_IR);
    if (have) {
      uint32_t mk = 0, tot = irTotalUs();
      for (uint16_t i = 0; i < lastIrRawLen; i += 2) mk += lastIrRaw[i];
      snprintf(st, sizeof(st), "duty %lu%%", tot ? (unsigned long)((uint64_t)mk * 100 / tot) : 0UL);
      panelStat(p2, st, C_IR);
      irDrawDuty(p2, h2);
    }
  } else {
    panelFrame(p2, h2, replay ? "CODE TO SEND" : "LAST CODE", C_IR);
    if (have || replay) irDrawCode(p2, h2, replay);
    else printCentered("no code yet", W / 2, p2 + h2 / 2 + 4, 1, C_DIM);
  }
}

// ===================================================================
// MODULE REFERENCE ARTICLES  (the "?" button on each module row)
//   DocLine.kind: 0 head, 1 body, 2 sample(mono), 3 caption, 10/11/12 figure
// ===================================================================
static const DocLine L11[] = {
  {3,"Ask every nearby router to announce itself."},
  {0,"What it is"},
  {1,"An active scan sends probe requests across"},
  {1,"all 2.4GHz channels. Access points reply"},
  {1,"with their SSID, channel and signal, so you"},
  {1,"get a full list of networks in range."},
  {0,"What you can do"},
  {1,"Map which networks and channels are busy,"},
  {1,"spot the strongest AP, or find rogue or"},
  {1,"unexpected SSIDs on a site you own."},
  {10,""},
  {3,"Fig 1: how many APs sit on each channel."},
  {0,"Sample output"},
  {2,"SSID            CH   RSSI"},
  {2,"NETGEAR_5G      36   -42 dBm"},
  {2,"xfinitywifi      6   -68 dBm"},
  {2,"<Hidden>        11   -73 dBm"},
  {0,"Reading it"},
  {1,"RSSI is signal strength in dBm: closer to"},
  {1,"0 is stronger. -42 excellent, -70 fair,"},
  {1,"below -85 barely reachable. Crowded"},
  {1,"channels mean more interference."},
};
static const DocLine L12[] = {
  {3,"Log the channel you are on as you move."},
  {0,"What it is"},
  {1,"Wardriving records the radio environment"},
  {1,"while you travel. This module reports the"},
  {1,"current monitored channel and the deck's"},
  {1,"own RSSI about once per second."},
  {0,"What you can do"},
  {1,"Build a picture of coverage along a route,"},
  {1,"or pair with GPS on the USB dashboard to"},
  {1,"geotag what you find."},
  {0,"Sample output"},
  {2,"Current_Channel_Monitor"},
  {2,"ch 6     -51 dBm"},
  {2,"ch 6     -49 dBm"},
  {0,"Reading it"},
  {1,"A steady channel with changing RSSI shows"},
  {1,"you moving nearer or further from the AP."},
};
static const DocLine L13[] = {
  {3,"Count raw Wi-Fi frames in the air."},
  {0,"What it is"},
  {1,"Promiscuous mode captures every 802.11"},
  {1,"frame on the current channel, not just"},
  {1,"traffic addressed to you. The deck tallies"},
  {1,"frames per second as a live rate."},
  {0,"What you can do"},
  {1,"Gauge how busy a channel is, detect bursts"},
  {1,"of activity, or find the noisiest spot in a"},
  {1,"space. Switch channels to compare."},
  {0,"Sample output"},
  {2,"[RAW_PACKET_COUNT] PROMISC_MODE"},
  {2,"ch 6     420 pkt/s"},
  {2,"ch 6     611 pkt/s"},
  {0,"Reading it"},
  {1,"The CHART tab plots packets/s over time."},
  {1,"Tall bars mean heavy traffic; flat means a"},
  {1,"quiet channel."},
};
static const DocLine L14[] = {
  {3,"List clients joined to the deck's AP."},
  {0,"What it is"},
  {1,"When the deck hosts an access point, this"},
  {1,"counts the stations connected to it."},
  {0,"What you can do"},
  {1,"Confirm a device joined, or watch how many"},
  {1,"clients associate over time during a test."},
  {0,"Sample output"},
  {2,"Connected_Clients: 2"},
  {2,"AP_CLIENT_SCAN   ch 1"},
  {0,"Reading it"},
  {1,"The number is the live client count. Pair"},
  {1,"with AP hosting (1.5) to run the network."},
};
static const DocLine L15[] = {
  {3,"Run a Wi-Fi access point from the deck."},
  {0,"What it is"},
  {1,"The deck broadcasts its own SSID and hands"},
  {1,"out an IP. Other devices can join it like"},
  {1,"any router."},
  {0,"What you can do"},
  {1,"Create a captive test network, stand up a"},
  {1,"quick link between devices, or pair with"},
  {1,"discovery (1.4) to watch who connects."},
  {0,"Sample output"},
  {2,"IP: 192.168.4.1"},
  {2,"AP_ACTIVE_HOST   ch 1"},
  {0,"Reading it"},
  {1,"The IP is the deck's gateway address."},
  {1,"Default SSID is Cyberdeck_AP; change it in"},
  {1,"firmware before any real use."},
};
static const DocLine L16[] = {
  {3,"Send crafted 802.11 frames for testing."},
  {0,"What it is"},
  {1,"A controlled bench for transmitting raw"},
  {1,"management frames and checking whether the"},
  {1,"radio accepted them."},
  {0,"What you can do"},
  {1,"Study how 802.11 framing works and verify"},
  {1,"injection on hardware you own. Use only on"},
  {1,"your own equipment, with permission."},
  {0,"Sample output"},
  {2,"Frame_Tx_Status: SUCCESS"},
  {2,"INJECTION_LAB    ch 6"},
  {0,"Reading it"},
  {1,"SUCCESS means the driver queued the frame;"},
  {1,"it is not proof a device received it."},
};
static const DocLine L21[] = {
  {3,"Discover Bluetooth Low Energy devices."},
  {0,"What it is"},
  {1,"A scan that lists nearby BLE advertisers:"},
  {1,"phones, wearables, beacons and tags, with"},
  {1,"name, address and signal strength."},
  {0,"What you can do"},
  {1,"Inventory devices in a room, find a target"},
  {1,"to inspect, or watch a tag's signal rise"},
  {1,"and fall as it moves."},
  {11,""},
  {3,"Fig 1: one device's RSSI over time."},
  {0,"Sample output"},
  {2,"NAME             RSSI"},
  {2,"Galaxy Buds      -58 dBm"},
  {2,"Mi Band 7        -66 dBm"},
  {2,"AC:23:3F:19:8B   -71 dBm"},
  {0,"Reading it"},
  {1,"Named entries advertise a friendly name;"},
  {1,"others show a MAC address. RSSI closer to"},
  {1,"0 is nearer. UUIDs hint at the service."},
};
static const DocLine L22[] = {
  {3,"Map the services a device exposes."},
  {0,"What it is"},
  {1,"After connecting, GATT enumeration walks"},
  {1,"the device's services and characteristics,"},
  {1,"the menu of what it can do."},
  {0,"What you can do"},
  {1,"Understand a device's capabilities, find"},
  {1,"readable or writable attributes, and learn"},
  {1,"how an app talks to it."},
  {0,"Sample output"},
  {2,"0x180F  Service: Battery"},
  {2,"0x2A19  Characteristic"},
  {2,"0x180A  Service: DeviceInfo"},
  {0,"Reading it"},
  {1,"Each line is one attribute by UUID."},
  {1,"16-bit UUIDs (0x180F) are standard, defined"},
  {1,"by the Bluetooth SIG; 128-bit are custom."},
};
static const DocLine L23[] = {
  {3,"Watch raw BLE advertising packets."},
  {0,"What it is"},
  {1,"BLE devices broadcast small advertising"},
  {1,"packets before any connection. This logs"},
  {1,"them continuously as they arrive."},
  {0,"What you can do"},
  {1,"See how often a device advertises, read its"},
  {1,"payload, and track beacons without pairing."},
  {0,"Sample output"},
  {2,"Tile_Pro         -77 dBm  FD3D"},
  {2,"Unknown          -83 dBm"},
  {2,"iBeacon          -69 dBm"},
  {0,"Reading it"},
  {1,"The trailing field is service data or UUID."},
  {1,"A fast repeat rate drains the device's"},
  {1,"battery faster."},
};
static const DocLine L24[] = {
  {3,"Make the deck advertise as a BLE beacon."},
  {0,"What it is"},
  {1,"The deck sends its own advertising packets"},
  {1,"so other scanners can discover it."},
  {0,"What you can do"},
  {1,"Test a scanner you are building, measure"},
  {1,"range, or provide a known reference signal."},
  {0,"Sample output"},
  {2,"Cyberdeck-Beacon"},
  {2,"Broadcast Active  BEACON_FLOOD"},
  {0,"Reading it"},
  {1,"Any phone BLE scanner app should now see"},
  {1,"Cyberdeck-Beacon in its device list."},
};
static const DocLine L25[] = {
  {3,"Read values from a device's attributes."},
  {0,"What it is"},
  {1,"Connects and reads the raw value of each"},
  {1,"readable characteristic, shown as hex."},
  {0,"What you can do"},
  {1,"Pull a battery level, serial number or"},
  {1,"sensor reading from a device you own."},
  {0,"Sample output"},
  {2,"0x2A19   64"},
  {2,"0x2A29   4C4733"},
  {2,"0x2A25   LOCKED"},
  {0,"Reading it"},
  {1,"Hex is the raw byte value. 64 hex = 100 ="},
  {1,"full battery. LOCKED means read denied."},
};
static const DocLine L26[] = {
  {3,"Measure how fast a device accepts links."},
  {0,"What it is"},
  {1,"Repeatedly connects to a target and times"},
  {1,"how long each connection takes, up to five"},
  {1,"attempts, then reports."},
  {0,"What you can do"},
  {1,"Check link reliability and latency, or"},
  {1,"confirm a device is reachable and stable."},
  {0,"Sample output"},
  {2,"Attempt 1  SUCCESS: 142ms"},
  {2,"Attempt 2  SUCCESS: 138ms"},
  {2,"Attempt 3  FAILED"},
  {0,"Reading it"},
  {1,"Lower ms is faster. Occasional FAILED is"},
  {1,"normal in a noisy area; mostly-failed means"},
  {1,"the target is out of range or busy."},
};
static const DocLine L41[] = {
  {3,"See which 2.4GHz channels are busy."},
  {0,"What it is"},
  {1,"The nRF24 steps through all 126 channels"},
  {1,"(2400-2525 MHz). On each one it listens"},
  {1,"32 times and counts how often a signal"},
  {1,"was stronger than about -64 dBm."},
  {0,"What you can do"},
  {1,"Find quiet channels for your own nRF24 or"},
  {1,"Wi-Fi gear, see where Wi-Fi and Bluetooth"},
  {1,"are active, and spot interference."},
  {13,""},
  {3,"Fig 1: two Wi-Fi humps and one busy ch."},
  {0,"Reading it"},
  {1,"Each bar is one channel, 0-32 hits tall."},
  {1,"The thin mark above it is the peak hold."},
  {1,"Tinted bands are Wi-Fi channels 1, 6, 11."},
  {1,"The waterfall scrolls older sweeps down:"},
  {1,"a bright column means steady traffic."},
  {0,"Sample output"},
  {2,"sweep #12  ch76  18/32  busy 9 ch"},
  {0,"Wiring check"},
  {1,"\"nRF24 not found\" means the radio did"},
  {1,"not answer: check CE 2, CSN 1 and 3.3V."},
};
static const DocLine L42[] = {
  {3,"Catch packets on the default nRF address."},
  {0,"What it is"},
  {1,"Listens on the nRF24 factory-default"},
  {1,"address (E7E7E7E7E7), hopping channels"},
  {1,"0-83 once a second, and shows every"},
  {1,"32-byte packet it receives as hex."},
  {0,"What you can do"},
  {1,"Check that your own nRF24 projects are"},
  {1,"transmitting, and on which channel. Only"},
  {1,"monitor radios you own or may test."},
  {0,"Sample output"},
  {2,"ch 76  pipe1  32 B"},
  {2,"43594201000000000000.."},
  {0,"Reading it"},
  {1,"Only devices left on the default address"},
  {1,"show up. The CHART tab counts packets per"},
  {1,"channel, with a cursor on the channel"},
  {1,"being listened to right now."},
};
static const DocLine L43[] = {
  {3,"Find nRF24 addresses in use nearby."},
  {0,"What it is"},
  {1,"nRF24 links filter packets by address."},
  {1,"This listens with the 2-byte preamble"},
  {1,"AA AA as the address, so it hears the"},
  {1,"start of many packets, and reports the"},
  {1,"5 bytes that follow as a candidate."},
  {0,"What you can do"},
  {1,"Learn which address your own devices use"},
  {1,"before you inspect them with 4.2 or 4.4."},
  {0,"Sample output"},
  {2,"A3C51B9F02"},
  {2,"ch 40  hits 7"},
  {0,"Reading it"},
  {1,"Hits is the running total of captures."},
  {1,"A real address keeps coming back; one-off"},
  {1,"candidates are usually noise."},
};
static const DocLine L44[] = {
  {3,"Guess what kind of traffic a packet is."},
  {0,"What it is"},
  {1,"For each packet caught on the default"},
  {1,"address it reports channel, length and"},
  {1,"data rate, plus a simple guess: 8 or"},
  {1,"fewer non-zero bytes reads as control,"},
  {1,"more reads as data."},
  {0,"What you can do"},
  {1,"Get a feel for how your own nRF24 devices"},
  {1,"talk: short commands versus payloads."},
  {0,"Sample output"},
  {2,"ch 12  control"},
  {2,"32 B  2MBPS  pipe1"},
  {0,"Reading it"},
  {1,"The guess is a rule of thumb, not a full"},
  {1,"decode. Control rows show cyan, data rows"},
  {1,"show amber."},
};
static const DocLine L45[] = {
  {3,"Keep the last 32 packets in memory."},
  {0,"What it is"},
  {1,"Listens on channel 2 (2402 MHz) and keeps"},
  {1,"each packet in a 32-slot ring in RAM."},
  {1,"When it is full, the oldest is replaced."},
  {0,"What you can do"},
  {1,"Collect a burst of traffic from your own"},
  {1,"device; every packet also goes out over"},
  {1,"USB for the dashboard to save."},
  {0,"Sample output"},
  {2,"stored 14  pipe1"},
  {2,"0A1B2C3D4E5F60718293.."},
  {0,"Reading it"},
  {1,"Stored is the total since START. The"},
  {1,"CHART tab shows the 32 slots; the bright"},
  {1,"one holds the newest packet."},
};
static const DocLine L46[] = {
  {3,"Send test packets to a node you own."},
  {0,"What it is"},
  {1,"Every 0.5 s it sends a 32-byte test frame"},
  {1,"at low power on channel 76 (2476 MHz) to"},
  {1,"the address CCBBDDEE01, and reports if"},
  {1,"the receiver acknowledged it."},
  {0,"What you can do"},
  {1,"Range-test a link: set up a second nRF24"},
  {1,"listening on that address, then walk"},
  {1,"away until the acks drop."},
  {0,"Sample output"},
  {2,"sent 42  ACK"},
  {2,"ch 76  CYBER"},
  {0,"Reading it"},
  {1,"ACK means your receiver got the frame."},
  {1,"With no receiver set up, every frame says"},
  {1,"no ack, which is expected."},
};
static const DocLine L71[] = {
  {3,"Run the IR LED at a steady 38kHz."},
  {0,"What it is"},
  {1,"Most remotes flash their IR LED at about"},
  {1,"38kHz. This drives the transmitter on"},
  {1,"GPIO16 with a continuous 38kHz square"},
  {1,"wave at 50% duty, made by the ESP32's PWM"},
  {1,"hardware."},
  {0,"What you can do"},
  {1,"Check the transmitter works: look at the"},
  {1,"LED through a phone camera and it shows"},
  {1,"a faint purple glow."},
  {0,"Sample output"},
  {2,"carrier 38.0 kHz"},
  {2,"duty 50%  TX GPIO16"},
  {0,"Reading it"},
  {1,"The frequency is read back from the PWM"},
  {1,"hardware, so 38.0 kHz confirms it runs."},
  {1,"A plain carrier carries no button code."},
};
static const DocLine L72[] = {
  {3,"Record the raw timing of a key press."},
  {0,"What it is"},
  {1,"The receiver on GPIO15 strips the 38kHz"},
  {1,"carrier and outputs on/off pulses. This"},
  {1,"records each mark (IR on) and space"},
  {1,"(IR off) in microseconds."},
  {0,"What you can do"},
  {1,"Learn a button from your own remote, even"},
  {1,"if no decoder knows its protocol. The"},
  {1,"last capture is kept for replay (7.3)."},
  {14,""},
  {3,"Fig 1: NEC frame, 9ms lead then the bits."},
  {0,"Sample output"},
  {2,"burst  67 pulses"},
  {2,"9024 4496 576 552 576.."},
  {0,"Reading it"},
  {1,"The CHART tab draws the waveform and a"},
  {1,"histogram of pulse widths. OVF means the"},
  {1,"burst was longer than the buffer."},
};
static const DocLine L73[] = {
  {3,"Send the last captured code back out."},
  {0,"What it is"},
  {1,"Sends the last capture from 7.2, 7.4 or"},
  {1,"7.5 once on GPIO16 each time you tap"},
  {1,"START. A known protocol is re-encoded;"},
  {1,"otherwise the raw timings go out at"},
  {1,"38kHz."},
  {0,"What you can do"},
  {1,"Control your own TV or AC from the deck,"},
  {1,"or check that a learned code works."},
  {0,"Sample output"},
  {2,"replayed NEC"},
  {2,"0x20DF10EF  32 bits"},
  {0,"Reading it"},
  {1,"\"No code yet\" means nothing has been"},
  {1,"captured since power-on: run 7.2, 7.4 or"},
  {1,"7.5 and press a remote key first. Use it"},
  {1,"only on devices you own."},
};
static const DocLine L74[] = {
  {3,"Name the protocol and code of a key."},
  {0,"What it is"},
  {1,"Decodes each capture with the IRremote-"},
  {1,"ESP8266 library: NEC, Sony, RC5, RC6,"},
  {1,"Samsung and many more."},
  {0,"What you can do"},
  {1,"Identify a remote's protocol and button"},
  {1,"codes, e.g. to build your own remote."},
  {0,"Sample output"},
  {2,"NEC"},
  {2,"0x20DF10EF                32b"},
  {0,"Reading it"},
  {1,"Value is the button code in hex; bits is"},
  {1,"its length. Holding a key can give short"},
  {1,"repeat codes. UNKNOWN means no decoder"},
  {1,"matched; 7.3 can still replay it raw."},
};
static const DocLine L75[] = {
  {3,"Measure the timing of a captured burst."},
  {0,"What it is"},
  {1,"Adds up how long the IR was on (mark) and"},
  {1,"off (space) in each capture and works out"},
  {1,"the duty cycle."},
  {0,"What you can do"},
  {1,"Compare remotes, spot noisy or partial"},
  {1,"captures, and check a signal before you"},
  {1,"replay it."},
  {0,"Sample output"},
  {2,"67 pulses  NEC                41%"},
  {2,"mark 27.5  space 40.5 ms"},
  {0,"Reading it"},
  {1,"Duty is mark time over total time; many"},
  {1,"remotes land between 30% and 50%. A very"},
  {1,"short total usually means noise."},
};
static const DocLine L76[] = {
  {3,"Fire short test pulses on the IR LED."},
  {0,"What it is"},
  {1,"Pulses GPIO16 high for 26us every 10ms,"},
  {1,"about 100 pulses a second, to check the"},
  {1,"transmitter wiring without a carrier."},
  {0,"What you can do"},
  {1,"Confirm the TX module is connected. A"},
  {1,"scope or logic analyser on GPIO16 is the"},
  {1,"surest check; the LED itself is very dim."},
  {0,"Sample output"},
  {2,"512 pulses fired             +100"},
  {2,"26us every 10ms GPIO16"},
  {0,"Reading it"},
  {1,"The count should climb by about 100 each"},
  {1,"second. If nothing reaches the LED, check"},
  {1,"S goes to GPIO16 and - goes to GND."},
};
#define DOCN(a) (sizeof(a)/sizeof((a)[0]))
static const Doc DOC_11 = {"Active Wi-Fi scan", "WIFI", L11, DOCN(L11)};
static const Doc DOC_12 = {"Wardriving logger", "WIFI", L12, DOCN(L12)};
static const Doc DOC_13 = {"Promiscuous sniffer", "WIFI", L13, DOCN(L13)};
static const Doc DOC_14 = {"Network discovery", "WIFI", L14, DOCN(L14)};
static const Doc DOC_15 = {"AP hosting", "WIFI", L15, DOCN(L15)};
static const Doc DOC_16 = {"Frame lab", "WIFI", L16, DOCN(L16)};
static const Doc DOC_21 = {"BLE scan", "BLE", L21, DOCN(L21)};
static const Doc DOC_22 = {"GATT enumeration", "BLE", L22, DOCN(L22)};
static const Doc DOC_23 = {"Advertisement sniffer", "BLE", L23, DOCN(L23)};
static const Doc DOC_24 = {"Beacon broadcaster", "BLE", L24, DOCN(L24)};
static const Doc DOC_25 = {"GATT read", "BLE", L25, DOCN(L25)};
static const Doc DOC_26 = {"Connection test", "BLE", L26, DOCN(L26)};
static const Doc DOC_41 = {"Spectrum scan", "NRF24", L41, DOCN(L41)};
static const Doc DOC_42 = {"Packet sniffer", "NRF24", L42, DOCN(L42)};
static const Doc DOC_43 = {"Address discovery", "NRF24", L43, DOCN(L43)};
static const Doc DOC_44 = {"Protocol analysis", "NRF24", L44, DOCN(L44)};
static const Doc DOC_45 = {"Packet storage", "NRF24", L45, DOCN(L45)};
static const Doc DOC_46 = {"Lab transmitter", "NRF24", L46, DOCN(L46)};
static const Doc DOC_71 = {"38kHz carrier", "IR", L71, DOCN(L71)};
static const Doc DOC_72 = {"Pulse learning", "IR", L72, DOCN(L72)};
static const Doc DOC_73 = {"Signal replay", "IR", L73, DOCN(L73)};
static const Doc DOC_74 = {"Protocol decode", "IR", L74, DOCN(L74)};
static const Doc DOC_75 = {"Signal analysis", "IR", L75, DOCN(L75)};
static const Doc DOC_76 = {"LED circuit test", "IR", L76, DOCN(L76)};
const Doc* moduleDoc(const char* id) {
  if (!strcmp(id, "1.1")) return &DOC_11; if (!strcmp(id, "1.2")) return &DOC_12;
  if (!strcmp(id, "1.3")) return &DOC_13; if (!strcmp(id, "1.4")) return &DOC_14;
  if (!strcmp(id, "1.5")) return &DOC_15; if (!strcmp(id, "1.6")) return &DOC_16;
  if (!strcmp(id, "2.1")) return &DOC_21; if (!strcmp(id, "2.2")) return &DOC_22;
  if (!strcmp(id, "2.3")) return &DOC_23; if (!strcmp(id, "2.4")) return &DOC_24;
  if (!strcmp(id, "2.5")) return &DOC_25; if (!strcmp(id, "2.6")) return &DOC_26;
  if (!strcmp(id, "4.1")) return &DOC_41; if (!strcmp(id, "4.2")) return &DOC_42;
  if (!strcmp(id, "4.3")) return &DOC_43; if (!strcmp(id, "4.4")) return &DOC_44;
  if (!strcmp(id, "4.5")) return &DOC_45; if (!strcmp(id, "4.6")) return &DOC_46;
  if (!strcmp(id, "7.1")) return &DOC_71; if (!strcmp(id, "7.2")) return &DOC_72;
  if (!strcmp(id, "7.3")) return &DOC_73; if (!strcmp(id, "7.4")) return &DOC_74;
  if (!strcmp(id, "7.5")) return &DOC_75; if (!strcmp(id, "7.6")) return &DOC_76;
  static const DocLine LG[] = { {3,"No reference article for this module yet."} };
  static const Doc DG = {"Module", "", LG, 1};
  return &DG;
}

// ----- article figures (illustrative sample data) -----
void figFrame(int16_t x, int16_t y, int16_t w, int16_t h) {
  tft.fillRoundRect(x, y, w, h, 4, C_PANEL); tft.drawRoundRect(x, y, w, h, 4, C_EDGE);
}
void drawInfoFigure(uint8_t k, int16_t x, int16_t y, int16_t w, int16_t h) {
  figFrame(x, y, w, h);
  int16_t ix = x + 6, iy = y + 6, iw = w - 12, ih = h - 20;
  if (k == 0) {
    const uint8_t s[13] = {3,1,0,2,1,9,2,1,4,1,8,1,2}; uint8_t mx = 9;
    for (int c = 0; c < 13; c++) { int16_t bw = iw / 13, bx = ix + c * bw, bh = s[c] * ih / mx;
      if (bh < 1 && s[c]) bh = 1;
      tft.fillRect(bx + 1, iy + ih - bh, bw - 2, bh, (c == 5 || c == 10) ? C_WIFI : mix565(C_WIFI, C_PANEL, 120)); }
    printAt("ch1", ix, y + h - 11, 1, C_DIM);
    printAt("ch6", ix + iw / 2 - 8, y + h - 11, 1, C_DIM);
    printAt("ch11", ix + iw - 24, y + h - 11, 1, C_DIM);
  } else if (k == 1) {
    const int8_t s[16] = {-80,-74,-76,-70,-66,-68,-60,-58,-62,-55,-59,-64,-61,-57,-60,-58};
    int vmin = -90, vmax = -45; int16_t ppx = -1, ppy = 0;
    for (int i = 0; i < 16; i++) { int16_t px2 = ix + i * (iw - 1) / 15;
      int16_t yy = iy + (vmax - s[i]) * (ih - 1) / (vmax - vmin);
      if (ppx >= 0) tft.drawLine(ppx, ppy, px2, yy, C_BLE); tft.fillCircle(px2, yy, 1, C_BLE);
      ppx = px2; ppy = yy; }
    printAt("near", ix, y + h - 11, 1, C_OK);
    printAt("time ->", ix + iw - 42, y + h - 11, 1, C_DIM);
  } else if (k == 2) {
    const char* lb[4] = {"-55", "-70", "-82", "-95"}; uint16_t cc[4] = {C_OK, C_WIFI, C_WARN, C_ERR};
    for (int i = 0; i < 4; i++) { int16_t cx = ix + i * (iw / 4) + iw / 8;
      for (int bb = 0; bb < 4; bb++) { int16_t bh = 4 + bb * 4;
        tft.fillRect(cx - 14 + bb * 7, iy + ih - bh, 5, bh, bb <= (3 - i) ? cc[i] : C_EDGE); }
      printAt(lb[i], cx - 12, y + h - 11, 1, cc[i]); }
  } else if (k == 3) {                               // nRF24 spectrum (sample)
    for (int c = 0; c < NRF_CH; c++) {
      int d1 = abs(c - 12), d6 = abs(c - 37), lv = (c * 37) % 11 == 0 ? 2 : 0;
      if (d1 <= 11) lv += 9 - d1 * 9 / 12;
      if (d6 <= 11) lv += 15 - d6 * 15 / 12;
      if (c == 76) lv = 26; if (c == 75 || c == 77) lv = 8;
      int16_t bh = lv * ih / NRF_MAXLV;
      if (bh > 0) tft.fillRect(ix + c * 2, iy + ih - bh, 2, bh, nrfRamp(lv));
    }
    tft.drawFastHLine(ix, iy + ih, NRF_CH * 2, C_EDGE);
    printAt("2400", ix, y + h - 11, 1, C_DIM);
    printAt("2462", ix + 62 * 2 - 12, y + h - 11, 1, C_DIM);
    printAt("2525 MHz", ix + NRF_CH * 2 - 48, y + h - 11, 1, C_DIM);
  } else if (k == 4) {                               // IR NEC frame (sample: lead + 8 bits)
    uint16_t v[19] = {9000, 4500}; uint8_t n = 2; const uint8_t code = 0x20;
    for (int bit = 7; bit >= 0; bit--) { v[n++] = 560; v[n++] = (code >> bit) & 1 ? 1690 : 560; }
    v[n++] = 560;
    irDrawWave(v, n, ix, iy, iw, ih, C_IR);
    printAt("lead", ix, y + h - 11, 1, C_DIM);
    printAt("short gap 0, long gap 1", ix + iw - 23 * 6, y + h - 11, 1, C_DIM);
  }
}

// ----- the scrollable article reader -----
uint8_t docLineH(const DocLine& L) { return L.kind == 0 ? 24 : (L.kind >= 10 ? 62 : 12); }
uint8_t infoMaxStart(const Doc* doc) {
  int16_t avail = (H - 6) - (HEADER_H + 6), h = 0; int s = doc->count;
  for (int i = doc->count - 1; i >= 0; i--) { int lh = docLineH(doc->lines[i]);
    if (h + lh > avail) break; h += lh; s = i; }
  return (uint8_t)s;
}
void drawInfoBody(const Doc* doc) {
  const int16_t left = 10, top = HEADER_H + 6, bottom = H - 6, cw = W - left - 24;
  tft.fillRect(0, HEADER_H, W - 20, H - HEADER_H, C_BG);
  int16_t y = top;
  for (uint8_t i = infoScroll; i < doc->count; i++) {
    const DocLine& L = doc->lines[i]; int16_t lh = docLineH(L);
    if (y + lh > bottom) break;
    if (L.kind == 0)       fontAt(L.text, left, y + 16, true, SUBS[infoSub].color);
    else if (L.kind == 1)  printAt(L.text, left, y + 2, 1, C_TEXT);
    else if (L.kind == 3)  printAt(L.text, left, y + 2, 1, C_MUTED);
    else if (L.kind == 2)  { tft.fillRect(left - 3, y - 1, cw + 6, 12, C_PANEL); printAt(L.text, left + 3, y + 2, 1, C_OK); }
    else if (L.kind >= 10) drawInfoFigure(L.kind - 10, left, y + 2, cw, 56);
    y += lh;
  }
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
      drawNavBar(s.name, s.color, (int8_t)curSub);
      const int16_t x = 8, y0 = HEADER_H + 7, rowH = 27, gap = 5, qn = 26;
      for (uint8_t i = 0; i < s.count; i++) {
        int16_t ry = y0 + i * (rowH + gap);
        addButton(x, ry, W - 16 - qn - 6, rowH, s.mods[i].id, s.mods[i].name, s.color, ST_ROW, ACT_OPEN_MOD, i);
        addButton(W - 8 - qn, ry + (rowH - qn) / 2, qn, qn, "?", nullptr, s.color, ST_QMARK, ACT_OPEN_INFO, i);
      }
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }

    case SCR_SOON: {
      const Subsystem& s = SUBS[curSub];
      tft.fillScreen(C_BG);
      drawNavBar(s.name, s.color, (int8_t)curSub);
      const int16_t well = 46, wx = (W - well) / 2, wy = 68;
      tft.fillRoundRect(wx, wy, well, well, 11, mix565(s.color, C_PANEL, 40));
      drawIcon((uint8_t)curSub, wx + well / 2, wy + well / 2, s.color, mix565(s.color, C_PANEL, 40));
      fontAt("Not on deck yet", (W - fontWidth("Not on deck yet", true)) / 2, 144, true, C_TEXT);
      printCentered("This module isn't in this firmware build.", W / 2, 164, 1, C_MUTED);
      printCentered("Flash unified_firmware to use it over USB.", W / 2, 178, 1, C_MUTED);
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;
    }

    case SCR_LIVE:
      drawLiveChrome();
      if (liveGraph) drawCharts(); else drawLog();
      drawStatusLine();
      drawSeen();
      for (uint8_t i = 0; i < btnCount; i++) drawButton(i, false);
      return;

    case SCR_SETUP: {
      tft.fillScreen(C_BG);
      drawNavBar("Setup", C_CYAN, -1);
      addButton(12, 44, W - 24, 48, "RECALIBRATE", nullptr, C_CYAN, ST_PILL, ACT_CALIBRATE, 0);
      addButton(12, 100, W - 24, 48, "TOUCH TEST", nullptr, C_CYAN, ST_PILL, ACT_TEST, 0);
      char line[52];
      if (haveSavedCal) snprintf(line, sizeof(line), "Calibration saved, max error %.1f px", savedErr);
      else snprintf(line, sizeof(line), "Calibration: default (not calibrated)");
      printAt(line, 12, 164, 1, C_TEXT);
      printAt("Wi-Fi, BLE, nRF24 + IR are live on-deck.", 12, 182, 1, C_MUTED);
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

    case SCR_INFO: {
      const Doc* doc = moduleDoc(SUBS[infoSub].mods[infoMod].id);
      uint8_t mxs = infoMaxStart(doc);
      if (infoScroll > mxs) infoScroll = mxs;
      if (infoScroll < 0) infoScroll = 0;
      tft.fillScreen(C_BG);
      drawTopBar(doc->title, SUBS[infoSub].color);
      drawInfoBody(doc);
      int16_t railX = W - 2 - 16, railTop = HEADER_H + 6, railH = (H - HEADER_H - 12 - 4) / 2;
      addButton(railX, railTop, 16, railH, "", nullptr, infoScroll > 0 ? C_TEXT : C_DIM, ST_ARROW, ACT_INFO_SCROLL, -1);
      addButton(railX, railTop + railH + 4, 16, railH, "", nullptr, infoScroll < mxs ? C_TEXT : C_DIM, ST_ARROW, ACT_INFO_SCROLL, 1);
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
        curSub = si; curMod = mi; liveStartMs = millis(); rowsClear();
        running = activeModule != "NONE";          // false if the start failed
        if (opensOnChart(module)) liveGraph = true;     // spectrum / waveform open on the graph
        if (running) setStatus("Started from USB", C_MUTED);
        else { char l[48]; snprintf(l, sizeof(l), "ERR %s", friendlyStatus(lastErrMsg)); setStatus(l, C_ERR); }
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
  liveStartMs = millis();
  rowsClear();
  running = true;
  startModule(String(SUBS[curSub].mods[curMod].id));   // the real module
  if (activeModule == "NONE") running = false;          // start failed (error already on the status line)
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
      if (opensOnChart(liveId())) liveGraph = true;     // spectrum / waveform open on the graph
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
    case ACT_VIEW:
      if (liveGraph != (b.arg == 1)) { liveGraph = (b.arg == 1); goTo(SCR_LIVE); }
      break;
    case ACT_OPEN_INFO:
      infoSub = curSub; infoMod = b.arg; infoScroll = 0; goTo(SCR_INFO);
      break;
    case ACT_INFO_SCROLL: {
      const Doc* doc = moduleDoc(SUBS[infoSub].mods[infoMod].id);
      uint8_t mxs = infoMaxStart(doc);
      int16_t ns = infoScroll + (b.arg < 0 ? -2 : 2);
      if (ns < 0) ns = 0; if (ns > mxs) ns = mxs;
      if (ns != infoScroll) { infoScroll = ns; goTo(SCR_INFO); } else drawButton(i, false);
      break;
    }
    case ACT_CALIBRATE:  startCalibration(SCR_SETUP); break;
    case ACT_TEST:       goTo(SCR_TEST); break;
    case ACT_CLEAR:      goTo(SCR_TEST); break;
    case ACT_CAL_RETRY:  startCalibration(calReturn); break;
    case ACT_CAL_SAVE:   saveCalibration(); goTo(calReturn); break;
    case ACT_TOGGLE:
      if (running) { stopLiveModule(); setStatus("Halted", C_MUTED); }
      else         { startLiveModule(); if (running) setStatus("Started", C_OK); }
      goTo(SCR_LIVE);
      break;
    case ACT_BACK:
      if (screen == SCR_LIVE)      { stopLiveModule(); goTo(SCR_MODULES); }
      else if (screen == SCR_INFO) goTo(SCR_MODULES);
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
  Serial.println("\n[+] Cyberdeck on-deck (Wi-Fi + BLE + nRF24 + IR) online.");

  // display + touch
  pinMode(TFT_CS, OUTPUT);   digitalWrite(TFT_CS, HIGH);
  pinMode(TOUCH_CS, OUTPUT); digitalWrite(TOUCH_CS, HIGH);
  pinMode(NRF_CSN_PIN, OUTPUT); digitalWrite(NRF_CSN_PIN, HIGH);   // radio off the bus
  pinMode(NRF_CE_PIN, OUTPUT);  digitalWrite(NRF_CE_PIN, LOW);
  pinMode(NRF_IRQ_PIN, INPUT_PULLUP);
  pinMode(IR_TX_PIN, OUTPUT);   digitalWrite(IR_TX_PIN, LOW);      // IR LED off until a 7.x module runs
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

  // a module that self-completes (e.g. BLE 2.6) -> refresh the live view
  if (running && activeModule == "NONE") running = false;   // stopped from USB
  static bool prevRun = false;
  if (screen == SCR_LIVE && prevRun && !running) goTo(SCR_LIVE);
  prevRun = running;

  // rate series: real packets/s for the sniffer, else telemetry events/s
  if (screen == SCR_LIVE && running && activeModule != "7.6" && now - gRateLastMs >= 1000) {
    gRateLastMs = now; uint16_t v;
    if (activeModule == "1.3") { unsigned long pc = packetCount, d = pc - gLastPackets; if (d > 65535) d = 65535; v = (uint16_t)d; gLastPackets = pc; }
    else { v = gRateAccum; gRateAccum = 0; }
    ratePush(v); graphDirty = true;
  }

  // throttled LIVE redraws so a Wi-Fi burst can't thrash the TFT
  if (screen == SCR_LIVE) {
    if (!liveGraph) { if (listDirty && now - lastListDraw >= 120) { lastListDraw = now; listDirty = false; drawLog(); } }
    else {
      if (curSub == 2 && running && nrfLiveChannel() != nrfShownHop) { nrfShownHop = nrfLiveChannel(); graphDirty = true; }
      if (graphDirty && now - lastGraphDraw >= 160) {
        lastGraphDraw = now; graphDirty = false;
        if (curSub == 2 && !strcmp(liveId(), "4.1")) nrfSpectrumTick();   // only changed columns
        else drawCharts();
      }
    }
    if (statusDirty) { statusDirty = false; drawStatusLine(); }
    if (now - lastSeenDraw >= 500) { lastSeenDraw = now; drawSeen(); }
  }
}
