/*
 * Cyberdeck ESP32-S3 — Unified Firmware
 * ----------------------------------------------------------------
 * Merges: 01_wifi (1.1-1.6), 02_ble (2.1-2.6), 04_2.4ghz_nrf24 (4.1-4.6),
 *         05_rfid_rc522 (5.1-5.6), 07_infrared (7.1-7.6)
 *
 * RFID module IDs follow firmware/05_rfid_rc522/5.1-5.6 (RC522 13.56 MHz,
 * ISO-14443A). Authorized / educational use only — read/inspect cards you
 * own or are permitted to test.
 *   5.1 card_fingerprint   — UID / ATQA / SAK + derived card type
 *                            (doubles as the RC522 wiring check: rc522_not_found)
 *   5.2 memory_dump        — read-only per-block dump with the default key
 *   5.3 default_key_audit  — dictionary attack of common keys -> A-F grade
 *   5.4 clone_to_magic     — read a SOURCE card you own, write to a magic card
 *                            you own. Writes GATED OFF by default (see below).
 *   5.5 ndef_read_write    — decode the first URI/Text NDEF record; write is
 *                            behind a compile switch, OFF by default.
 *   5.6 reader_monitor     — passive log of every card presented (ring buffer)
 *
 * RFID SAFETY GATES (compile-time, OFF by default — identical to the standalone
 * sketches). See the RFID globals block below:
 *   - RFID_OPERATOR_OWNS_CARDS 0 -> 5.4 refuses every write with
 *     "ownership_not_acknowledged"; set to 1 ONLY for cards you own. Even then
 *     it refuses non-magic targets and never touches sector trailers.
 *   - RFID_NDEF_MODE_WRITE 0 -> 5.5 is read/decode only; set to 1 to provision
 *     RFID_NDEF_URL to a tag you own.
 *
 * RFID pin sharing: the RC522 sits on the SAME SPI bus as the nRF24
 * (SCK 12 / MISO 13 / MOSI 11) with its own chip-select (RC522 SS 10,
 * nRF24 CSN 7). Only one module is ever active, so the shared bus is safe.
 *
 * IR module IDs follow firmware/07_infrared/7.1-7.6:
 *   7.1 38khz_modulation   — continuous 38kHz carrier on TX (LEDC)
 *   7.2 pulse_learning     — raw mark/space timings from RX
 *   7.3 signal_replay      — one-shot replay of the last capture
 *                            (decoded protocol if known, raw otherwise)
 *   7.4 protocol_libraries — protocol decode (NEC/Sony/RC5/RC6/...)
 *   7.5 signal_analysis    — mark/space totals + duty per capture
 *   7.6 led_circuit_tests  — 26us test pulse on TX every 10ms
 *
 * IR pin sharing: 7.1, 7.3 and 7.6 all drive IR_TX_PIN (7.1 through
 * LEDC, 7.3 through IRsend, 7.6 through digitalWrite); 7.2, 7.4 and
 * 7.5 share the one IRrecv on IR_RX_PIN. Only one module is ever
 * active, and startModule() now stops the previous module first, so
 * the LEDC channel is always detached before IRsend / digitalWrite
 * take the pin back.
 *
 * PROTOCOL (locked — see PROTOCOL.md for the full spec):
 *   Command  (host -> S3):  {"module":"1.1","action":"START"}
 *                            {"action":"STOP"}
 *   Telemetry (S3 -> host): {"module":"1.1","status":"ok","data":{...}}
 *
 * Requires: ArduinoJson v7.x  (Library Manager -> "ArduinoJson" by
 *           Benoit Blanchon)
 *           IRremoteESP8266   (Library Manager -> "IRremoteESP8266"
 *           by David Conran / crankyoldgit) — needed for modules
 *           7.3 / 7.4 only.
 *           RF24              (Library Manager -> "RF24" by TMRh20) —
 *           needed for the 2.4GHz nRF24 modules (4.x).
 *           MFRC522           (Library Manager -> "MFRC522" by
 *           miguelbalboa) — needed for the RC522 RFID modules (5.x).
 *
 * NON-BLOCKING NOTES (read this before adding more modules):
 *   - All module polling is gated by millis(), not delay(). The
 *     outer loop() always returns quickly except for two known,
 *     bounded exceptions below.
 *   - BLE scanning (2.1, 2.3, and the discovery phase of 2.2/2.5/2.6)
 *     runs as a CONTINUOUS async scan (duration 0) with results
 *     delivered via the BLEAdvertisedDeviceCallbacks callback. The
 *     loop() body does not block waiting on scan results.
 *   - KNOWN LIMITATION: BLEClient::connect() (used in the GATT
 *     phase of 2.2 / 2.5 / 2.6) is a blocking call inside the
 *     ESP32 Arduino BLE library itself — there is no non-blocking
 *     variant without moving BLE onto its own FreeRTOS task, which
 *     is out of scope for this pass. Each connect attempt can still
 *     block for ~1-3s. What WAS fixed: module 2.6 previously ran
 *     up to 5 connect attempts back-to-back inside a single while()
 *     loop, blocking the whole firmware (incl. Serial command
 *     intake) for up to ~15s straight. It now does ONE attempt per
 *     loop() pass, so a STOP command is checked between attempts.
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
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>
#include <SPI.h>
#include <RF24.h>
#include <MFRC522.h>

// ===================================================================
// PIN MAP (defaults — remap here if your wiring differs)
// ===================================================================
// IR
static const uint16_t IR_RX_PIN = 4;   // IR receiver diode (OUT -> GPIO4)
static const uint16_t IR_TX_PIN = 5;   // IR transmitter LED (via NPN/driver -> GPIO5)
// GPIO4 / GPIO5 are plain GPIOs on the S3: not strapping pins
// (0/3/45/46), not USB (19/20), not flash/PSRAM (26-37).
//
// nRF24L01+ (SPI) — modules 4.x
static const uint8_t NRF_CE_PIN   = 6;
static const uint8_t NRF_CSN_PIN  = 7;
static const uint8_t NRF_SCK_PIN  = 12;
static const uint8_t NRF_MISO_PIN = 13;
static const uint8_t NRF_MOSI_PIN = 11;
// (S3 default FSPI pins. Do NOT use 35/36/37: on octal-PSRAM boards like
//  N8R8/N16R8 those are wired to the PSRAM and are unusable.)
//
// RC522 RFID (SPI) — modules 5.x. Shares the nRF24 bus (SCK 12 / MISO 13 /
// MOSI 11) with its own chip-select; only one module is ever active.
static const uint8_t RC522_SS_PIN   = 10;  // SDA / SS
static const uint8_t RC522_RST_PIN  = 9;   // RST
static const uint8_t RC522_SCK_PIN  = 12;  // shared with nRF24
static const uint8_t RC522_MISO_PIN = 13;  // shared with nRF24
static const uint8_t RC522_MOSI_PIN = 11;  // shared with nRF24
// IRQ unused. VCC 3.3 V ONLY — 5 V kills the RC522.
//
// RESERVED for hardware that has no firmware yet (proposed wiring, untested;
// listed here so new modules don't collide with it):
// 2.8" ILI9341 TFT + XPT2046 touch (own SPI bus):
//   SCK 21, MOSI 18, MISO 47, CS 15, DC 16, RST 17, LED(backlight) 8,
//   T_CS 1, T_IRQ 2 (touch shares the TFT's SCK/MOSI/MISO).

static const uint16_t IR_CAPTURE_BUFFER_SIZE = 1024;
static const uint8_t  IR_TIMEOUT = 15;  // ms of silence that ends a capture
static const uint32_t IR_CARRIER_HZ = 38000;   // 7.1 carrier frequency
static const uint8_t  IR_CARRIER_RES_BITS = 8; // 7.1 LEDC resolution (duty 0-255)
static const uint32_t IR_TEST_PULSE_US = 26;   // 7.6 pulse width (~one 38kHz period)
static const uint32_t IR_TEST_PERIOD_MS = 10;  // 7.6 pulse interval

// ===================================================================
// GLOBAL STATE
// ===================================================================
String activeModule = "NONE";
unsigned long moduleActionMillis = 0;   // last time this module "did its thing"
unsigned long moduleStateMillis = 0;    // generic per-module sub-state timer

volatile unsigned long packetCount = 0;

BLEScan* pBLEScan = nullptr;
BLEAdvertising* pAdvertising = nullptr;
String bleTargetAddr = "";
bool bleTargetFound = false;

// 2.6 connection-testing state (was an inline while() loop — now
// spread across loop() passes so it can't starve Serial/STOP)
int bleTestAttempts = 0;
static const int BLE_TEST_MAX_ATTEMPTS = 5;

// 1.1 async scan sub-state
enum WifiScanState { WSCAN_IDLE, WSCAN_RUNNING };
WifiScanState wifiScanState = WSCAN_IDLE;

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

// --- 2.4GHz nRF24 (modules 4.x) ---
// Educational / authorized-lab RF diagnostics, same scope as the Wi-Fi and
// BLE tools: only audit radios you own or are permitted to test.
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

// --- RC522 RFID (modules 5.x) ---
// 13.56 MHz ISO-14443A reader, educational / authorized-use only: read and
// inspect cards you own or are permitted to test, same scope as the other
// subsystems. SAFETY GATES are compile-time and OFF by default:
//   RFID_OPERATOR_OWNS_CARDS 0 -> 5.4 reads the source card and reports, but
//     refuses every write with "ownership_not_acknowledged". Set to 1 ONLY for
//     cards you own; even then it refuses non-magic targets and never writes a
//     sector trailer (so it can't brick a card's keys).
//   RFID_NDEF_MODE_WRITE 0 -> 5.5 is read/decode only. Set to 1 to provision
//     RFID_NDEF_URL to a writable tag you own.
#define RFID_OPERATOR_OWNS_CARDS 0
#define RFID_NDEF_MODE_WRITE     0
#define RFID_NDEF_URL "https://github.com/RRonium/cyberdeck-esp32s3"

MFRC522 rfid(RC522_SS_PIN, RC522_RST_PIN);
bool rc522Ready = false;                 // PCD_Init + version check passed once
MFRC522::MIFARE_Key rfidDefaultKey;      // factory default FFFFFFFFFFFF
unsigned long rfidLastPoll = 0;          // field-probe cadence (per module)
unsigned long rfidLastSummary = 0;       // 5.6 periodic summary timer
unsigned long rfidStartedAt = 0;         // 5.6 t_ms origin
String rfidLastUid = "";                 // 5.1 debounce of a held card
unsigned long rfidLastSeen = 0;

// 5.3 default-key dictionary (common / factory / transport keys seen in the wild).
const byte RFID_KEY_DICT[][6] = {
  {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},  // blank / factory default
  {0xA0,0xA1,0xA2,0xA3,0xA4,0xA5},  // MAD / NDEF default key A
  {0xD3,0xF7,0xD3,0xF7,0xD3,0xF7},  // NDEF public key B
  {0x00,0x00,0x00,0x00,0x00,0x00},  // all-zero
  {0xB0,0xB1,0xB2,0xB3,0xB4,0xB5},  // common infra transport key
  {0x4D,0x3A,0x99,0xC3,0x51,0xDD},  // common transit key
  {0x1A,0x98,0x2C,0x7E,0x45,0x9A},  // common transit key
  {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF},  // lazy demo key
  {0x12,0x34,0x56,0x78,0x9A,0xBC},  // lazy demo key
};
static const byte RFID_KEY_DICT_LEN = sizeof(RFID_KEY_DICT) / 6;

// 5.4 clone state: source card cached in RAM, then written to a magic target.
static const byte RFID_MAX_BLOCKS = 64;  // Classic 1K = 64 blocks
enum RfidCloneStage { RFID_WAIT_SOURCE, RFID_WAIT_TARGET };
RfidCloneStage rfidCloneStage = RFID_WAIT_SOURCE;
byte rfidSrcBlocks[RFID_MAX_BLOCKS][16];
bool rfidSrcValid[RFID_MAX_BLOCKS];
byte rfidSrcBlockCount = 0;
String rfidSrcUidStr = "";

// 5.6 reader-monitor ring buffer: distinct UIDs with a per-UID seen count.
static const byte RFID_RING_SIZE = 32;
static const unsigned long RFID_REREAD_GUARD_MS = 800;    // ignore a card left in the field
static const unsigned long RFID_SUMMARY_EVERY_MS = 5000;  // periodic distinct/total summary
struct RfidSeen { String uid; String type; uint16_t count; unsigned long lastMs; };
RfidSeen rfidRing[RFID_RING_SIZE];
byte rfidRingCount = 0;
byte rfidRingHead = 0;
uint16_t rfidTotalReads = 0;

// 5.5 NFC Forum URI prefix table (subset covering the common cases).
const char* RFID_URI_PREFIX[] = {
  "", "http://www.", "https://www.", "http://", "https://", "tel:", "mailto:"
};
static const byte RFID_URI_PREFIX_LEN = 7;

JsonDocument telemetryDoc;

// ===================================================================
// TELEMETRY (one shared helper so every module emits the same shape)
//   Caller fills telemetryDoc["data"][...] first, then calls this.
// ===================================================================
void emitTelemetry(const char* module, const char* status) {
  telemetryDoc["module"] = module;
  telemetryDoc["status"] = status;
  serializeJson(telemetryDoc, Serial);
  Serial.println();
  telemetryDoc.clear();
}

void emitError(const char* module, const char* msg) {
  telemetryDoc["data"]["msg"] = msg;
  emitTelemetry(module, "error");
}

// ===================================================================
// nRF24 INIT — lazy one-time bring-up; reports if the module is missing
// ===================================================================
bool ensureNrf() {
  if (nrfReady) return true;
  SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
  if (!radio.begin() || !radio.isChipConnected()) return false;
  nrfReady = true;
  return true;
}

// ===================================================================
// RC522 RFID (modules 5.x) — helpers + per-module workers
//   Lazy one-time bring-up, then per-card operations invoked from loop().
//   Only one 5.x module is ever active; stopModule() halts the PICC and
//   releases Crypto1 before the next module runs.
// ===================================================================
bool ensureRc522() {
  if (rc522Ready) return true;
  SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
  rfid.PCD_Init();
  delay(50);
  // The version register reads back 0x00 / 0xFF when nothing answers on the bus.
  byte v = rfid.PCD_ReadRegister(rfid.VersionReg);
  if (v == 0x00 || v == 0xFF) return false;
  rfid.PCD_AntennaOn();
  for (byte i = 0; i < 6; i++) rfidDefaultKey.keyByte[i] = 0xFF;
  rc522Ready = true;
  return true;
}

String rfidToHex(const byte* buf, byte len) {
  String s = "";
  for (byte i = 0; i < len; i++) { if (buf[i] < 0x10) s += '0'; s += String(buf[i], HEX); }
  s.toUpperCase();
  return s;
}

byte rfidBlocksInSector(byte sector) { return sector < 32 ? 4 : 16; }
byte rfidFirstBlockOf(byte sector) { return sector < 32 ? sector * 4 : 128 + (sector - 32) * 16; }
bool rfidIsTrailer(byte block) {
  if (block < 128) return (block % 4) == 3;
  return ((block - 128) % 16) == 15;
}

// --- 5.1 card_fingerprint: report one freshly-selected card ----------------
void rfidReportCard(const byte* atqa) {
  String uid = rfidToHex(rfid.uid.uidByte, rfid.uid.size);
  if (uid == rfidLastUid && (millis() - rfidLastSeen) < 1200) return;  // debounce held card
  rfidLastUid = uid;
  rfidLastSeen = millis();

  byte sak = rfid.uid.sak;
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(sak);
  telemetryDoc["data"]["uid"]     = uid;
  telemetryDoc["data"]["uid_len"] = rfid.uid.size;
  char atqaHex[5];
  snprintf(atqaHex, sizeof(atqaHex), "%02X%02X", atqa[1], atqa[0]);  // MSB-first for readability
  telemetryDoc["data"]["atqa"]    = atqaHex;
  char sakHex[3];
  snprintf(sakHex, sizeof(sakHex), "%02X", sak);
  telemetryDoc["data"]["sak"]     = sakHex;
  telemetryDoc["data"]["type"]    = String(rfid.PICC_GetTypeName(piccType));
  emitTelemetry("5.1", "ok");

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// --- 5.2 memory_dump: read-only per-block dump with the default key --------
void rfidReportBlock(byte sector, byte block, const byte* data, bool authOk) {
  telemetryDoc["data"]["sector"] = sector;
  telemetryDoc["data"]["block"]  = block;
  telemetryDoc["data"]["auth"]   = authOk ? "ok" : "fail";
  if (authOk) {
    telemetryDoc["data"]["data"]     = rfidToHex(data, 16);
    telemetryDoc["data"]["key_used"] = "FFFFFFFFFFFF";
  }
  emitTelemetry("5.2", "ok");
}

void rfidDumpClassic(byte sectorCount) {
  uint16_t read = 0, locked = 0;
  for (byte s = 0; s < sectorCount; s++) {
    byte trailer = rfidFirstBlockOf(s) + rfidBlocksInSector(s) - 1;
    MFRC522::StatusCode st = rfid.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &rfidDefaultKey, &(rfid.uid));
    bool sectorAuthed = (st == MFRC522::STATUS_OK);
    for (byte b = 0; b < rfidBlocksInSector(s); b++) {
      byte addr = rfidFirstBlockOf(s) + b;
      byte buf[18]; byte sz = sizeof(buf);
      bool ok = false;
      if (sectorAuthed && rfid.MIFARE_Read(addr, buf, &sz) == MFRC522::STATUS_OK) ok = true;
      if (ok) read++; else locked++;
      rfidReportBlock(s, addr, buf, ok);
    }
    rfid.PCD_StopCrypto1();  // release crypto before the next sector's auth
  }
  telemetryDoc["data"]["blocks_read"]   = read;
  telemetryDoc["data"]["blocks_locked"] = locked;
  telemetryDoc["data"]["done"]          = true;
  emitTelemetry("5.2", "ok");
}

void rfidDumpUltralight() {
  // Ultralight / NTAG: 4-byte pages, no auth. MIFARE_Read returns 4 pages (16B)
  // per call; reads past the end start failing, which ends the loop cleanly.
  uint16_t read = 0;
  for (byte page = 0; page < 48; page += 4) {
    byte buf[18]; byte sz = sizeof(buf);
    if (rfid.MIFARE_Read(page, buf, &sz) == MFRC522::STATUS_OK) {
      for (byte p = 0; p < 4; p++) {
        read++;
        byte pageBuf[16] = {0};
        memcpy(pageBuf, buf + p * 4, 4);
        rfidReportBlock(0, page + p, pageBuf, true);
      }
    } else break;
  }
  telemetryDoc["data"]["blocks_read"]   = read;
  telemetryDoc["data"]["blocks_locked"] = 0;
  telemetryDoc["data"]["done"]          = true;
  emitTelemetry("5.2", "ok");
}

void rfidDumpCard() {
  MFRC522::PICC_Type t = rfid.PICC_GetType(rfid.uid.sak);
  switch (t) {
    case MFRC522::PICC_TYPE_MIFARE_1K:   rfidDumpClassic(16); break;
    case MFRC522::PICC_TYPE_MIFARE_4K:   rfidDumpClassic(40); break;
    case MFRC522::PICC_TYPE_MIFARE_MINI: rfidDumpClassic(5);  break;
    case MFRC522::PICC_TYPE_MIFARE_UL:   rfidDumpUltralight(); break;
    default: emitError("5.2", "unsupported_card"); break;  // DESFire / 14443A-4: UID only
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// --- 5.3 default_key_audit: dictionary attack -> per-sector result + grade --
String rfidKeyHex(const byte* k) {
  String s = "";
  for (byte i = 0; i < 6; i++) { if (k[i] < 0x10) s += '0'; s += String(k[i], HEX); }
  s.toUpperCase();
  return s;
}

// Try every dictionary key (A then B) on a sector trailer. Returns the index of
// the key that worked, or -1 if the sector held. A failed Crypto1 auth drops the
// card out of ACTIVE, so we halt + wake + re-select around each failed try.
int rfidCrackSector(byte sector) {
  byte trailer = rfidFirstBlockOf(sector) + rfidBlocksInSector(sector) - 1;
  MFRC522::MIFARE_Key key;
  for (byte k = 0; k < RFID_KEY_DICT_LEN; k++) {
    for (byte i = 0; i < 6; i++) key.keyByte[i] = RFID_KEY_DICT[k][i];
    for (byte which = 0; which < 2; which++) {
      byte authCmd = which == 0 ? MFRC522::PICC_CMD_MF_AUTH_KEY_A : MFRC522::PICC_CMD_MF_AUTH_KEY_B;
      MFRC522::StatusCode st = rfid.PCD_Authenticate(authCmd, trailer, &key, &(rfid.uid));
      rfid.PCD_StopCrypto1();
      if (st == MFRC522::STATUS_OK) return k;
      rfid.PICC_HaltA();
      byte atqa[2]; byte asz = sizeof(atqa);
      rfid.PICC_WakeupA(atqa, &asz);
      rfid.PICC_ReadCardSerial();
    }
  }
  return -1;
}

char rfidGradeFor(byte weak, byte total) {
  if (total == 0) return '?';
  int pct = (int)((long)weak * 100 / total);
  if (weak == 0) return 'A';
  if (pct <= 12) return 'B';
  if (pct <= 37) return 'C';
  if (pct <= 62) return 'D';
  if (pct < 100) return 'E';
  return 'F';
}

void rfidAuditCard() {
  MFRC522::PICC_Type t = rfid.PICC_GetType(rfid.uid.sak);
  byte sectorCount;
  switch (t) {
    case MFRC522::PICC_TYPE_MIFARE_1K:   sectorCount = 16; break;
    case MFRC522::PICC_TYPE_MIFARE_4K:   sectorCount = 40; break;
    case MFRC522::PICC_TYPE_MIFARE_MINI: sectorCount = 5;  break;
    default: emitError("5.3", "unsupported_card"); rfid.PICC_HaltA(); return;
  }
  byte weak = 0;
  for (byte s = 0; s < sectorCount; s++) {
    int k = rfidCrackSector(s);
    bool isWeak = (k >= 0);
    if (isWeak) weak++;
    telemetryDoc["data"]["sector"] = s;
    telemetryDoc["data"]["result"] = isWeak ? "default_key" : "protected";
    if (isWeak) telemetryDoc["data"]["key_found"] = rfidKeyHex(RFID_KEY_DICT[k]);
    emitTelemetry("5.3", "ok");
  }
  char grade = rfidGradeFor(weak, sectorCount);
  telemetryDoc["data"]["weak_sectors"]  = weak;
  telemetryDoc["data"]["total_sectors"] = sectorCount;
  char g[2] = { grade, 0 };
  telemetryDoc["data"]["grade"]         = g;
  telemetryDoc["data"]["done"]          = true;
  emitTelemetry("5.3", "ok");
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// --- 5.4 clone_to_magic: read an owned source, write to an owned magic card --
// Gen1a "UID changeable" magic cards answer two unauthenticated commands
// (0x40 7-bit, then 0x43), each acked with 0x0A. Non-magic cards stay silent.
bool rfidUnlockGen1a() {
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  byte resp = 0; byte respLen = 1; byte validBits = 7;
  byte cmd40 = 0x40;
  MFRC522::StatusCode s = rfid.PCD_TransceiveData(&cmd40, 1, &resp, &respLen, &validBits, 0, false);
  if (s != MFRC522::STATUS_OK || resp != 0x0A) return false;
  byte cmd43 = 0x43; respLen = 1; validBits = 0;
  s = rfid.PCD_TransceiveData(&cmd43, 1, &resp, &respLen, &validBits, 0, false);
  return (s == MFRC522::STATUS_OK && resp == 0x0A);
}

// Raw (unauthenticated) MIFARE write used after a Gen1a unlock:
// {0xA0, block}+CRC -> ACK, then 16 data bytes + CRC -> ACK.
bool rfidMifareRawWrite(byte block, const byte* data16) {
  byte cmd[4]; byte resp = 0; byte respLen = 1; byte validBits = 0;
  cmd[0] = 0xA0; cmd[1] = block;
  if (rfid.PCD_CalculateCRC(cmd, 2, &cmd[2]) != MFRC522::STATUS_OK) return false;
  if (rfid.PCD_TransceiveData(cmd, 4, &resp, &respLen, &validBits, 0, false) != MFRC522::STATUS_OK) return false;
  if (resp != 0x0A) return false;
  byte buf[18];
  memcpy(buf, data16, 16);
  if (rfid.PCD_CalculateCRC(buf, 16, &buf[16]) != MFRC522::STATUS_OK) return false;
  resp = 0; respLen = 1; validBits = 0;
  if (rfid.PCD_TransceiveData(buf, 18, &resp, &respLen, &validBits, 0, false) != MFRC522::STATUS_OK) return false;
  return (resp == 0x0A);
}

void rfidReadSource() {
  MFRC522::PICC_Type t = rfid.PICC_GetType(rfid.uid.sak);
  if (t != MFRC522::PICC_TYPE_MIFARE_1K && t != MFRC522::PICC_TYPE_MIFARE_4K &&
      t != MFRC522::PICC_TYPE_MIFARE_MINI) {
    emitError("5.4", "unsupported_card"); rfid.PICC_HaltA(); return;
  }
  byte sectorCount = (t == MFRC522::PICC_TYPE_MIFARE_4K) ? 40
                     : (t == MFRC522::PICC_TYPE_MIFARE_MINI ? 5 : 16);
  rfidSrcUidStr = rfidToHex(rfid.uid.uidByte, rfid.uid.size);
  for (byte i = 0; i < RFID_MAX_BLOCKS; i++) rfidSrcValid[i] = false;
  rfidSrcBlockCount = 0;
  byte cached = 0;
  for (byte s = 0; s < sectorCount; s++) {
    byte trailer = rfidFirstBlockOf(s) + rfidBlocksInSector(s) - 1;
    if (rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &rfidDefaultKey, &(rfid.uid)) != MFRC522::STATUS_OK) {
      rfid.PCD_StopCrypto1(); continue;  // skip sectors the default key can't open
    }
    for (byte b = 0; b < rfidBlocksInSector(s); b++) {
      byte addr = rfidFirstBlockOf(s) + b;
      if (addr >= RFID_MAX_BLOCKS) continue;
      byte buf[18]; byte sz = sizeof(buf);
      if (rfid.MIFARE_Read(addr, buf, &sz) == MFRC522::STATUS_OK) {
        memcpy(rfidSrcBlocks[addr], buf, 16);
        rfidSrcValid[addr] = true;
        cached++;
        if (addr + 1 > rfidSrcBlockCount) rfidSrcBlockCount = addr + 1;
      }
    }
    rfid.PCD_StopCrypto1();
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  if (cached == 0) { emitError("5.4", "source_locked"); return; }
  telemetryDoc["data"]["stage"]         = "source_read";
  telemetryDoc["data"]["uid"]           = rfidSrcUidStr;
  telemetryDoc["data"]["blocks_cached"] = cached;
  emitTelemetry("5.4", "ok");
  rfidCloneStage = RFID_WAIT_TARGET;
}

void rfidWriteTarget() {
#if RFID_OPERATOR_OWNS_CARDS == 0
  emitError("5.4", "ownership_not_acknowledged");   // writes disabled until the gate is set
  rfid.PICC_HaltA();
  return;
#else
  bool gen1a = rfidUnlockGen1a();
  const char* targetType = gen1a ? "gen1a" : "gen2";

  // Block 0 (UID / manufacturer block): Gen1a raw write, else an authenticated
  // block-0 write (Gen2/CUID). If neither works, it isn't a writable-UID magic card.
  bool block0ok = false;
  if (rfidSrcValid[0]) {
    if (gen1a) {
      block0ok = rfidMifareRawWrite(0, rfidSrcBlocks[0]);
    } else {
      byte atqa[2]; byte asz = sizeof(atqa);
      rfid.PICC_WakeupA(atqa, &asz);
      rfid.PICC_ReadCardSerial();
      if (rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 3, &rfidDefaultKey, &(rfid.uid)) == MFRC522::STATUS_OK) {
        block0ok = (rfid.MIFARE_Write(0, rfidSrcBlocks[0], 16) == MFRC522::STATUS_OK);
      }
      rfid.PCD_StopCrypto1();
    }
  }
  if (!block0ok) {
    emitError("5.4", "not_a_magic_card");
    rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
    rfidCloneStage = RFID_WAIT_SOURCE; return;
  }

  // Data blocks: authenticated writes, never touching block 0 (done) or trailers.
  byte written = 1;  // block 0 counts
  byte sectorCount = rfidSrcBlockCount <= 64 ? 16 : 40;
  for (byte s = 0; s < sectorCount; s++) {
    byte trailer = rfidFirstBlockOf(s) + rfidBlocksInSector(s) - 1;
    byte atqa[2]; byte asz = sizeof(atqa);
    rfid.PICC_HaltA();
    rfid.PICC_WakeupA(atqa, &asz);
    if (!rfid.PICC_ReadCardSerial()) continue;
    if (rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &rfidDefaultKey, &(rfid.uid)) != MFRC522::STATUS_OK) {
      rfid.PCD_StopCrypto1(); continue;
    }
    for (byte b = 0; b < rfidBlocksInSector(s); b++) {
      byte addr = rfidFirstBlockOf(s) + b;
      if (addr == 0 || rfidIsTrailer(addr) || addr >= RFID_MAX_BLOCKS || !rfidSrcValid[addr]) continue;
      if (rfid.MIFARE_Write(addr, rfidSrcBlocks[addr], 16) == MFRC522::STATUS_OK) written++;
    }
    rfid.PCD_StopCrypto1();
  }

  // Verify: read the copy back and diff the cached blocks.
  bool match = true;
  for (byte s = 0; s < sectorCount && match; s++) {
    byte trailer = rfidFirstBlockOf(s) + rfidBlocksInSector(s) - 1;
    byte atqa[2]; byte asz = sizeof(atqa);
    rfid.PICC_HaltA();
    rfid.PICC_WakeupA(atqa, &asz);
    if (!rfid.PICC_ReadCardSerial()) { match = false; break; }
    if (rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &rfidDefaultKey, &(rfid.uid)) != MFRC522::STATUS_OK) {
      rfid.PCD_StopCrypto1(); continue;
    }
    for (byte b = 0; b < rfidBlocksInSector(s); b++) {
      byte addr = rfidFirstBlockOf(s) + b;
      if (addr == 0 || rfidIsTrailer(addr) || addr >= RFID_MAX_BLOCKS || !rfidSrcValid[addr]) continue;
      byte buf[18]; byte sz = sizeof(buf);
      if (rfid.MIFARE_Read(addr, buf, &sz) != MFRC522::STATUS_OK || memcmp(buf, rfidSrcBlocks[addr], 16) != 0) { match = false; break; }
    }
    rfid.PCD_StopCrypto1();
  }

  telemetryDoc["data"]["stage"]          = "clone";
  telemetryDoc["data"]["uid"]            = rfidSrcUidStr;
  telemetryDoc["data"]["blocks_written"] = written;
  telemetryDoc["data"]["verify"]         = match ? "match" : "mismatch";
  telemetryDoc["data"]["target_type"]    = targetType;
  emitTelemetry("5.4", "ok");
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  rfidCloneStage = RFID_WAIT_SOURCE;   // ready for the next clone
#endif
}

// --- 5.5 ndef_read_write: decode (and optionally write) an NDEF record ------
bool rfidIsNfcTag() {
  return (rfid.PICC_GetType(rfid.uid.sak) == MFRC522::PICC_TYPE_MIFARE_UL);  // NTAG reports as UL
}

// Read user memory from page 4 into out[]. Returns bytes actually read.
int rfidReadUserMem(byte* out, int maxLen) {
  int got = 0;
  for (byte page = 4; got < maxLen; page += 4) {
    byte buf[18]; byte sz = sizeof(buf);
    if (rfid.MIFARE_Read(page, buf, &sz) != MFRC522::STATUS_OK) break;
    int n = min(16, maxLen - got);
    memcpy(out + got, buf, n);
    got += n;
    if (page > 200) break;  // hard stop well past NTAG216
  }
  return got;
}

// Decode the first NDEF URI/Text record in a TLV-formatted buffer.
bool rfidDecodeNdef(const byte* mem, int len, String& typeOut, String& payloadOut) {
  int i = 0;
  while (i < len) {
    byte tlvType = mem[i++];
    if (tlvType == 0x00) continue;        // NULL TLV
    if (tlvType == 0xFE) break;           // terminator
    if (i >= len) break;
    int tlvLen = mem[i++];
    if (tlvLen == 0xFF) {                  // 3-byte length form
      if (i + 1 >= len) break;
      tlvLen = (mem[i] << 8) | mem[i + 1];
      i += 2;
    }
    if (tlvType != 0x03) { i += tlvLen; continue; }   // not an NDEF message TLV

    if (i + 3 >= len) break;
    byte hdr = mem[i];
    byte typeLen = mem[i + 1];
    byte payloadLen = mem[i + 2];
    byte tnf = hdr & 0x07;
    int rec = i + 3;
    if (typeLen == 0 || rec + typeLen > len) break;
    byte recType = mem[rec];
    int payloadStart = rec + typeLen;
    if (payloadStart + payloadLen > len) payloadLen = len - payloadStart;

    if (tnf == 0x01 && recType == 'U') {   // well-known URI
      byte prefixIdx = mem[payloadStart];
      String uri = (prefixIdx < RFID_URI_PREFIX_LEN) ? String(RFID_URI_PREFIX[prefixIdx]) : String("");
      for (int p = payloadStart + 1; p < payloadStart + payloadLen; p++) uri += (char)mem[p];
      typeOut = "uri"; payloadOut = uri; return true;
    }
    if (tnf == 0x01 && recType == 'T') {   // well-known Text
      byte statusByte = mem[payloadStart];
      byte langLen = statusByte & 0x3F;
      String txt = "";
      for (int p = payloadStart + 1 + langLen; p < payloadStart + payloadLen; p++) txt += (char)mem[p];
      typeOut = "text"; payloadOut = txt; return true;
    }
    return false;
  }
  return false;
}

void rfidDoRead() {
  if (!rfidIsNfcTag()) { emitError("5.5", "unsupported_card"); rfid.PICC_HaltA(); return; }
  byte mem[256];
  int len = rfidReadUserMem(mem, sizeof(mem));
  String type, payload;
  if (len <= 0 || !rfidDecodeNdef(mem, len, type, payload)) {
    emitError("5.5", "no_ndef"); rfid.PICC_HaltA(); return;
  }
  telemetryDoc["data"]["record_type"] = type;
  telemetryDoc["data"]["payload"]     = payload;
  telemetryDoc["data"]["tag_type"]    = String(rfid.PICC_GetTypeName(rfid.PICC_GetType(rfid.uid.sak)));
  telemetryDoc["data"]["capacity"]    = len;
  emitTelemetry("5.5", "ok");
  rfid.PICC_HaltA();
}

#if RFID_NDEF_MODE_WRITE
// Build a TLV-wrapped single URI record for RFID_NDEF_URL and write it page by page.
void rfidDoWrite() {
  if (!rfidIsNfcTag()) { emitError("5.5", "unsupported_card"); rfid.PICC_HaltA(); return; }
  String url = String(RFID_NDEF_URL);
  byte prefixIdx = 0; String rest = url;
  for (byte p = 1; p < RFID_URI_PREFIX_LEN; p++) {
    String pre = String(RFID_URI_PREFIX[p]);
    if (pre.length() && url.startsWith(pre) && pre.length() > String(RFID_URI_PREFIX[prefixIdx]).length()) {
      prefixIdx = p; rest = url.substring(pre.length());
    }
  }
  byte payloadLen = 1 + rest.length();        // prefix byte + URI tail
  byte ndefLen = 4 + payloadLen;
  byte buf[256]; int n = 0;
  buf[n++] = 0x03; buf[n++] = ndefLen;
  buf[n++] = 0xD1;             // MB|ME|SR, TNF=well-known
  buf[n++] = 0x01;             // type length
  buf[n++] = payloadLen;       // payload length
  buf[n++] = 'U';              // URI record type
  buf[n++] = prefixIdx;        // URI prefix code
  for (uint16_t c = 0; c < rest.length(); c++) buf[n++] = rest[c];
  buf[n++] = 0xFE;             // terminator
  while (n % 4 != 0) buf[n++] = 0x00;  // pad to a whole page

  byte written = 0;
  for (int off = 0; off < n; off += 4) {
    byte page = 4 + off / 4;
    if (rfid.MIFARE_Ultralight_Write(page, &buf[off], 4) != MFRC522::STATUS_OK) {
      emitError("5.5", "write_failed"); rfid.PICC_HaltA(); return;
    }
    written += 4;
  }
  byte mem[256]; int rlen = rfidReadUserMem(mem, n);
  bool match = (rlen >= n) && (memcmp(mem, buf, n) == 0);
  telemetryDoc["data"]["record_type"]  = "uri";
  telemetryDoc["data"]["bytes_written"] = written;
  telemetryDoc["data"]["verify"]        = match ? "match" : "mismatch";
  emitTelemetry("5.5", "ok");
  rfid.PICC_HaltA();
}
#endif

void rfidHandleTag() {
#if RFID_NDEF_MODE_WRITE
  rfidDoWrite();
#else
  rfidDoRead();
#endif
  rfid.PCD_StopCrypto1();
}

// --- 5.6 reader_monitor: passive log of every card presented ---------------
void rfidResetLog() {
  for (byte i = 0; i < RFID_RING_SIZE; i++) { rfidRing[i].uid = ""; rfidRing[i].count = 0; }
  rfidRingCount = 0; rfidRingHead = 0; rfidTotalReads = 0;
  rfidStartedAt = millis();
}

int rfidFindUid(const String& uid) {
  for (byte i = 0; i < rfidRingCount; i++) if (rfidRing[i].uid == uid) return i;
  return -1;
}

void rfidLogCard() {
  String uid = rfidToHex(rfid.uid.uidByte, rfid.uid.size);
  String type = String(rfid.PICC_GetTypeName(rfid.PICC_GetType(rfid.uid.sak)));
  unsigned long now = millis();
  int idx = rfidFindUid(uid);
  if (idx >= 0) {
    if (now - rfidRing[idx].lastMs < RFID_REREAD_GUARD_MS) { rfid.PICC_HaltA(); return; }
    rfidRing[idx].count++;
    rfidRing[idx].lastMs = now;
  } else {
    byte slot;
    if (rfidRingCount < RFID_RING_SIZE) { slot = rfidRingCount++; }
    else { slot = rfidRingHead; rfidRingHead = (rfidRingHead + 1) % RFID_RING_SIZE; }
    rfidRing[slot].uid = uid; rfidRing[slot].type = type; rfidRing[slot].count = 1; rfidRing[slot].lastMs = now;
    idx = slot;
  }
  rfidTotalReads++;
  telemetryDoc["data"]["uid"]        = uid;
  telemetryDoc["data"]["type"]       = type;
  telemetryDoc["data"]["t_ms"]       = now - rfidStartedAt;
  telemetryDoc["data"]["seen_count"] = rfidRing[idx].count;
  emitTelemetry("5.6", "ok");
  rfid.PICC_HaltA();
}

void rfidEmitSummary() {
  telemetryDoc["data"]["distinct"]    = rfidRingCount;
  telemetryDoc["data"]["total_reads"] = rfidTotalReads;
  telemetryDoc["data"]["summary"]     = true;
  emitTelemetry("5.6", "ok");
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
// WIFI PROMISCUOUS CALLBACK (module 1.3)
// ===================================================================
void wifiPacketHandler(void* buf, wifi_promiscuous_pkt_type_t type) {
  packetCount++;
}

// ===================================================================
// BLE SCAN CALLBACK — shared by 2.1 / 2.2 / 2.3 / 2.5 / 2.6
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
// MODULE START / STOP — one-time side effects live here, not in the
// polling loop, so entering a module never repeats init work.
// ===================================================================
void stopModule();

void startModule(const String& module) {
  // Tear down whatever was running so its hardware (WiFi mode, BLE
  // scan, LEDC on the IR TX pin, IRrecv interrupt) is released first.
  if (activeModule != "NONE") stopModule();

  activeModule = module;
  moduleActionMillis = 0;   // 0 forces the first poll to fire immediately
  moduleStateMillis = 0;
  bleTargetFound = false;
  bleTargetAddr = "";
  bleTestAttempts = 0;
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
    pBLEScan->start(0, nullptr, false);  // continuous async scan
  } else if (module == "2.2" || module == "2.5" || module == "2.6") {
    pBLEScan->start(0, nullptr, false);  // continuous async scan for target discovery
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
  } else if (module == "5.1" || module == "5.2" || module == "5.3" ||
             module == "5.4" || module == "5.5" || module == "5.6") {
    // One shared RC522. 5.1 doubles as the wiring check (rc522_not_found).
    if (!ensureRc522()) { emitError(module.c_str(), "rc522_not_found"); activeModule = "NONE"; return; }
    rfidLastPoll = 0;
    rfidLastUid = "";
    rfidLastSeen = 0;
    if (module == "5.4") rfidCloneStage = RFID_WAIT_SOURCE;
    if (module == "5.6") { rfidResetLog(); rfidLastSummary = millis(); }
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
  } else if (activeModule == "7.1") {
    ledcWrite(IR_TX_PIN, 0);
    ledcDetach(IR_TX_PIN);
    pinMode(IR_TX_PIN, OUTPUT);
    digitalWrite(IR_TX_PIN, LOW);   // leave the IR LED off
  } else if (activeModule == "7.2" || activeModule == "7.4" || activeModule == "7.5") {
    irrecv.disableIRIn();
  } else if (activeModule == "7.6") {
    digitalWrite(IR_TX_PIN, LOW);
  } else if (activeModule == "4.1" || activeModule == "4.2" || activeModule == "4.3" ||
             activeModule == "4.4" || activeModule == "4.5" || activeModule == "4.6") {
    if (nrfReady) { radio.stopListening(); radio.powerDown(); }
  } else if (activeModule == "5.1" || activeModule == "5.2" || activeModule == "5.3" ||
             activeModule == "5.4" || activeModule == "5.5" || activeModule == "5.6") {
    if (activeModule == "5.6") rfidEmitSummary();   // final distinct/total tally on stop
    if (rc522Ready) { rfid.PICC_HaltA(); rfid.PCD_StopCrypto1(); }
  } else if (activeModule == "1.1" && wifiScanState == WSCAN_RUNNING) {
    WiFi.scanDelete();
    wifiScanState = WSCAN_IDLE;
  }



  telemetryDoc["data"]["msg"] = "module_halted";
  emitTelemetry(activeModule.c_str(), "ok");
  activeModule = "NONE";
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  BLEDevice::init("Cyberdeck-S3");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new UnifiedBLECallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  pAdvertising = BLEDevice::getAdvertising();

  delay(100);
  Serial.println("\n[+] ESP32-S3 Cyberdeck Unified Firmware Online.");
}

// ===================================================================
// COMMAND PARSER
// ===================================================================
void handleSerialCommand() {
  if (Serial.available() <= 0) return;

  String incoming = Serial.readStringUntil('\n');
  incoming.trim();
  if (incoming.length() == 0) return;

  JsonDocument cmdDoc;
  DeserializationError err = deserializeJson(cmdDoc, incoming);
  if (err) {
    emitError("SYS", "malformed_json");
    return;
  }

  const char* action = cmdDoc["action"] | "";
  const char* module = cmdDoc["module"] | "NONE";

  if (strcmp(action, "START") == 0) {
    startModule(String(module));
  } else if (strcmp(action, "STOP") == 0) {
    stopModule();
  } else {
    emitError("SYS", "unknown_action");
  }
}

// ===================================================================
// LOOP
// ===================================================================
void loop() {
  handleSerialCommand();

  unsigned long now = millis();

  // --- WIFI SUBSYSTEM ---
  if (activeModule == "1.1") {
    // Async scan state machine: kick off -> poll -> report -> cooldown.
    if (wifiScanState == WSCAN_IDLE && now - moduleActionMillis >= 2000) {
      WiFi.scanNetworks(true, true);  // async=true, show_hidden=true
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
      // n == WIFI_SCAN_RUNNING: keep polling next pass, no block.
    }
  }
  else if (activeModule == "1.2") {
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      uint8_t primaryChannel;
      wifi_second_chan_t secondChannel;
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

  // --- BLE SUBSYSTEM ---
  // 2.1 / 2.3: fully callback-driven now (continuous scan started in
  // startModule()). Just housekeeping here: periodically clear the
  // scan result cache so it doesn't grow unbounded.
  else if (activeModule == "2.1" || activeModule == "2.3") {
    if (now - moduleActionMillis >= 3000) {
      moduleActionMillis = now;
      pBLEScan->clearResults();
    }
  }
  else if (activeModule == "2.2" || activeModule == "2.5") {
    if (!bleTargetFound) {
      // discovery phase: callback-driven, nothing to poll
    } else {
      // one connect+read attempt, then return to discovery
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
                } else {  // 2.5 — also attempt a read
                  String hexVal = "LOCKED";
                  if (pChar->canRead()) {
                    String val = pChar->readValue();
                    hexVal = "";
                    for (unsigned int i = 0; i < val.length(); i++) {
                      char buf[3];
                      sprintf(buf, "%02X", (uint8_t)val[i]);
                      hexVal += buf;
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
      pBLEScan->start(0, nullptr, false);  // resume discovery
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
      // discovery phase: callback-driven
    } else if (bleTestAttempts < BLE_TEST_MAX_ATTEMPTS) {
      // ONE attempt per loop() pass — this replaces the old inline
      // while() loop that blocked for up to 5 attempts straight.
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
      // test complete
      telemetryDoc["data"]["msg"] = "test_complete";
      emitTelemetry("2.6", "ok");
      bleTargetFound = false;
      bleTestAttempts = 0;
      activeModule = "NONE";
    }
  }

  // --- 2.4GHZ nRF24 SUBSYSTEM ---
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

  // --- INFRARED SUBSYSTEM ---
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

  // --- RC522 RFID SUBSYSTEM ---
  // One shared reader; only one 5.x module active at a time. Field probing is
  // millis()-gated like every other module. The per-card operations in 5.2
  // (dump), 5.3 (dictionary audit) and 5.4 (clone) run to completion in a
  // single loop() pass once a card is detected — bounded, one-shot work, the
  // same category as the BLE connect exception in the NON-BLOCKING NOTES above.
  else if (activeModule == "5.1") {
    if (now - rfidLastPoll >= 150) {
      rfidLastPoll = now;
      // Do our own RequestA so we capture ATQA, then select the card.
      byte atqa[2]; byte atqaSize = sizeof(atqa);
      MFRC522::StatusCode rc = rfid.PICC_RequestA(atqa, &atqaSize);
      if (rc == MFRC522::STATUS_OK || rc == MFRC522::STATUS_COLLISION) {
        if (rfid.PICC_ReadCardSerial()) rfidReportCard(atqa);
      }
    }
  }
  else if (activeModule == "5.2") {
    if (now - rfidLastPoll >= 200) {
      rfidLastPoll = now;
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) rfidDumpCard();
    }
  }
  else if (activeModule == "5.3") {
    if (now - rfidLastPoll >= 200) {
      rfidLastPoll = now;
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) rfidAuditCard();
    }
  }
  else if (activeModule == "5.4") {
    if (now - rfidLastPoll >= 250) {
      rfidLastPoll = now;
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        if (rfidCloneStage == RFID_WAIT_SOURCE) rfidReadSource();
        else                                    rfidWriteTarget();
      }
    }
  }
  else if (activeModule == "5.5") {
    if (now - rfidLastPoll >= 250) {
      rfidLastPoll = now;
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) rfidHandleTag();
    }
  }
  else if (activeModule == "5.6") {
    if (now - rfidLastPoll >= 120) {
      rfidLastPoll = now;
      if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) rfidLogCard();
    }
    if (now - rfidLastSummary >= RFID_SUMMARY_EVERY_MS) {
      rfidLastSummary = now;
      rfidEmitSummary();
    }
  }

  // No trailing delay(): loop() is now gated entirely by the
  // millis() checks above, so idle passes return immediately.
}
