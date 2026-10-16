/*
 * Cyberdeck ESP32-S3 — Module 5.2: PN532 ISO 14443A Block Read (standalone)
 * -------------------------------------------------------------------------
 * Reads the data blocks/pages of a presented ISO 14443A tag. MIFARE
 * Classic blocks are authenticated with the factory default key-A
 * (0xFFFFFFFFFFFF); MIFARE Ultralight / NTAG pages are read directly.
 * Each successfully read block is reported as hex.
 *
 * Educational / authorized-lab NFC diagnostics only — read cards you
 * own or are explicitly permitted to test. Reads are non-destructive;
 * this module never writes.
 *
 * Wiring (PN532 in I2C mode):
 *   SDA=GPIO8  SCL=GPIO9  VCC=3.3V  GND=GND  RSTPDN->3.3V
 *
 * Requires: Adafruit PN532 + Adafruit BusIO + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"5.2","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"5.2","status":"ok","data":{"block":4,"data":"00112233445566778899AABBCCDDEEFF","ok":true}}
 *   locked block  : {"module":"5.2","status":"ok","data":{"block":4,"data":"","ok":false}}
 *   not present   : {"module":"5.2","status":"error","data":{"msg":"pn532_not_found"}}
 *
 * DEBUG_SERIAL = 1: starts on its own and, each time a tag is held to the
 *   antenna, prints a readable block dump to the Serial Monitor:
 *       [5.2] UID 04A2B3C4  MIFARE Classic
 *       blk 04: 00 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF
 *       blk 05: ....  (AUTH FAIL)
 *   One dump per tag presentation; lift and re-present to dump again.
 *   Set DEBUG_SERIAL = 0 for the dashboard (JSON as above).
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
static const uint8_t  CLASSIC_BLOCKS = 64;   // MIFARE Classic 1K = 64 blocks
static const uint8_t  UL_PAGES       = 44;   // Ultralight/NTAG215-ish page count to try

Adafruit_PN532 nfc(PN532_IRQ, PN532_RST, &Wire);
JsonDocument doc;

bool readActive = false;
bool pn532Ready = false;
bool busy       = false;        // dumping one tag right now
String lastUid  = "";
unsigned long lastPoll = 0;

// Factory default MIFARE Classic key-A — the well-known blank-card key.
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
  doc["module"] = "5.2";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

void emitBlock(int block, const String& data, bool ok) {
  doc["data"]["block"] = block;
  doc["data"]["data"]  = data;
  doc["data"]["ok"]    = ok;
  emit("ok");
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

// Dump a 4-byte-UID card as MIFARE Classic (auth each sector's blocks with
// the default key). Returns true if at least one block authenticated.
bool dumpClassic(uint8_t* uid, uint8_t uidLen) {
  bool any = false;
  for (uint8_t block = 0; block < CLASSIC_BLOCKS; block++) {
    // Re-auth at the first block of every 4-block sector.
    if (block % 4 == 0) {
      if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLen, block, 0 /*key A*/, DEFAULT_KEY)) {
        // Whole sector unreadable with the default key — report its 4 blocks locked.
        for (uint8_t b = block; b < block + 4 && b < CLASSIC_BLOCKS; b++) {
#if DEBUG_SERIAL
          Serial.print("blk "); if (b < 10) Serial.print('0'); Serial.print(b); Serial.println(":  (AUTH FAIL)");
#else
          emitBlock(b, "", false);
#endif
        }
        block += 3;
        continue;
      }
    }
    uint8_t data[16];
    if (nfc.mifareclassic_ReadDataBlock(block, data)) {
      any = true;
      String hex = bytesToHex(data, 16);
#if DEBUG_SERIAL
      Serial.print("blk "); if (block < 10) Serial.print('0'); Serial.print(block); Serial.print(": ");
      for (uint8_t i = 0; i < 16; i++) { if (data[i] < 0x10) Serial.print('0'); Serial.print(data[i], HEX); Serial.print(' '); }
      Serial.println();
#else
      emitBlock(block, hex, true);
#endif
    } else {
#if DEBUG_SERIAL
      Serial.print("blk "); if (block < 10) Serial.print('0'); Serial.print(block); Serial.println(":  (READ FAIL)");
#else
      emitBlock(block, "", false);
#endif
    }
  }
  return any;
}

// Dump a 7-byte-UID card as MIFARE Ultralight / NTAG (4 bytes per page).
bool dumpUltralight() {
  bool any = false;
  for (uint8_t page = 0; page < UL_PAGES; page++) {
    uint8_t data[4];
    if (nfc.mifareultralight_ReadPage(page, data)) {
      any = true;
      String hex = bytesToHex(data, 4);
#if DEBUG_SERIAL
      Serial.print("pg  "); if (page < 10) Serial.print('0'); Serial.print(page); Serial.print(": ");
      for (uint8_t i = 0; i < 4; i++) { if (data[i] < 0x10) Serial.print('0'); Serial.print(data[i], HEX); Serial.print(' '); }
      Serial.println();
#else
      emitBlock(page, hex, true);
#endif
    } else {
      break;   // past the end of this tag's memory
    }
  }
  return any;
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] PN532 Module 5.2 ISO 14443A Read ready.");

#if DEBUG_SERIAL
  if (ensurePn532()) {
    Serial.println("[+] DEBUG_SERIAL on: PN532 found. Hold a tag to dump its blocks/pages.");
    readActive = true;
  } else {
    Serial.println("[!] DEBUG_SERIAL on: PN532 NOT found. Check SDA=8 SCL=9, 3.3V, I2C jumpers.");
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.2\"") != -1) {
      if (!ensurePn532()) { doc["data"]["msg"] = "pn532_not_found"; emit("error"); }
      else { readActive = true; lastUid = ""; }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      readActive = false;
    }
  }

  if (readActive && pn532Ready && !busy && millis() - lastPoll >= POLL_EVERY) {
    lastPoll = millis();
    uint8_t uid[7] = {0};
    uint8_t uidLen = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, READ_TIMEOUT) && uidLen > 0) {
      String hexUid = uidToHex(uid, uidLen);
      if (hexUid != lastUid) {       // new tag → dump once
        lastUid = hexUid;
        busy = true;
#if DEBUG_SERIAL
        Serial.print("\n[5.2] UID "); Serial.print(hexUid);
        Serial.println(uidLen == 4 ? "  MIFARE Classic (default key A)" : "  Ultralight / NTAG");
#endif
        if (uidLen == 4) dumpClassic(uid, uidLen);
        else             dumpUltralight();
        busy = false;
      }
    } else {
      lastUid = "";                  // field empty → allow the same tag to be re-dumped later
    }
  }
}
