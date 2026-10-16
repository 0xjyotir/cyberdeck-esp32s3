/*
 * Cyberdeck ESP32-S3 — Module 5.5: PN532 Card Dump Storage (standalone)
 * ---------------------------------------------------------------------
 * Reads a presented tag block-by-block (or page-by-page) into an in-RAM
 * buffer and keeps a running count of how many blocks have been stored
 * this session. This is the capture backend the dashboard shows as a
 * growing "stored" tally; on the real deck the dump would be streamed to
 * the host and written under data_dumps/rfid_dumps/.
 *
 * Educational / authorized-lab NFC diagnostics only — dump cards you own
 * or are explicitly permitted to test. Read-only.
 *
 * Wiring (PN532 in I2C mode):
 *   SDA=GPIO8  SCL=GPIO9  VCC=3.3V  GND=GND  RSTPDN->3.3V
 *
 * Requires: Adafruit PN532 + Adafruit BusIO + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"5.5","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"5.5","status":"ok","data":{"block":4,"data":"00112233445566778899AABBCCDDEEFF","stored":5}}
 *   not present   : {"module":"5.5","status":"error","data":{"msg":"pn532_not_found"}}
 *
 * DEBUG_SERIAL = 1: starts on its own; hold a tag to stream its blocks to
 *   the Serial Monitor with the running stored count. Set DEBUG_SERIAL = 0
 *   for the dashboard (JSON as above).
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1

static const uint8_t PN532_SDA = 8;
static const uint8_t PN532_SCL = 9;
static const int8_t  PN532_IRQ = -1;
static const int8_t  PN532_RST = -1;

static const int      BAUD_RATE    = 115200;
static const uint32_t POLL_EVERY   = 400;
static const uint16_t READ_TIMEOUT = 150;
static const uint8_t  CLASSIC_BLOCKS = 64;
static const uint8_t  UL_PAGES       = 44;
static const uint16_t STORE_CAP      = 256;   // in-RAM ring of 16-byte records

Adafruit_PN532 nfc(PN532_IRQ, PN532_RST, &Wire);
JsonDocument doc;

bool dumpActive = false;
bool pn532Ready = false;
bool busy       = false;
String lastUid  = "";
unsigned long lastPoll = 0;

uint8_t  store[STORE_CAP][16];   // captured blocks (ring)
uint16_t storeHead = 0;          // next write index
uint16_t stored    = 0;          // running count this session (may exceed STORE_CAP)

uint8_t DEFAULT_KEY[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

String uidToHex(const uint8_t* uid, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) { if (uid[i] < 0x10) s += '0'; s += String(uid[i], HEX); }
  s.toUpperCase();
  return s;
}
String bytesToHex(const uint8_t* b, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) { if (b[i] < 0x10) s += '0'; s += String(b[i], HEX); }
  s.toUpperCase();
  return s;
}

void emit(const char* status) {
  doc["module"] = "5.5";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

void storeBlock(int block, const uint8_t* data, uint8_t len) {
  memset(store[storeHead], 0, 16);
  memcpy(store[storeHead], data, min((uint8_t)16, len));
  storeHead = (storeHead + 1) % STORE_CAP;
  stored++;
  String hex = bytesToHex(data, len);
#if DEBUG_SERIAL
  Serial.print("[5.5] stored "); Serial.print(stored);
  Serial.print("  blk "); if (block < 10) Serial.print('0'); Serial.print(block);
  Serial.print(": "); Serial.println(hex);
#else
  doc["data"]["block"]  = block;
  doc["data"]["data"]   = hex;
  doc["data"]["stored"] = stored;
  emit("ok");
#endif
}

bool ensurePn532() {
  if (pn532Ready) return true;
  Wire.begin(PN532_SDA, PN532_SCL);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) return false;
  nfc.SAMConfig();
  pn532Ready = true;
  return true;
}

void dumpClassic(uint8_t* uid, uint8_t uidLen) {
  for (uint8_t block = 0; block < CLASSIC_BLOCKS; block++) {
    if (block % 4 == 0 &&
        !nfc.mifareclassic_AuthenticateBlock(uid, uidLen, block, 0, DEFAULT_KEY)) {
      block += 3;           // sector locked with the default key — skip it
      continue;
    }
    uint8_t data[16];
    if (nfc.mifareclassic_ReadDataBlock(block, data)) storeBlock(block, data, 16);
  }
}

void dumpUltralight() {
  for (uint8_t page = 0; page < UL_PAGES; page++) {
    uint8_t data[4];
    if (!nfc.mifareultralight_ReadPage(page, data)) break;
    storeBlock(page, data, 4);
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] PN532 Module 5.5 Dump Storage ready.");

#if DEBUG_SERIAL
  if (ensurePn532()) {
    Serial.println("[+] DEBUG_SERIAL on: PN532 found. Hold a tag to capture its blocks into RAM.");
    dumpActive = true;
  } else {
    Serial.println("[!] DEBUG_SERIAL on: PN532 NOT found. Check SDA=8 SCL=9, 3.3V, I2C jumpers.");
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.5\"") != -1) {
      if (!ensurePn532()) { doc["data"]["msg"] = "pn532_not_found"; emit("error"); }
      else { dumpActive = true; lastUid = ""; stored = 0; storeHead = 0; }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      dumpActive = false;
    }
  }

  if (dumpActive && pn532Ready && !busy && millis() - lastPoll >= POLL_EVERY) {
    lastPoll = millis();
    uint8_t uid[7] = {0};
    uint8_t uidLen = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, READ_TIMEOUT) && uidLen > 0) {
      String hexUid = uidToHex(uid, uidLen);
      if (hexUid != lastUid) {
        lastUid = hexUid;
        busy = true;
#if DEBUG_SERIAL
        Serial.print("\n[5.5] capturing UID "); Serial.println(hexUid);
#endif
        if (uidLen == 4) dumpClassic(uid, uidLen);
        else             dumpUltralight();
        busy = false;
      }
    } else {
      lastUid = "";
    }
  }
}
