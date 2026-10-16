/*
 * Cyberdeck ESP32-S3 — Module 5.3: PN532 Card Memory Structure (standalone)
 * -------------------------------------------------------------------------
 * Identifies the card family of a presented ISO 14443A tag and reports
 * its memory layout — sector/block/page counts and total user bytes —
 * without dumping contents. A quick "what am I holding?" probe.
 *
 * Educational / authorized-lab NFC diagnostics only — profile cards you
 * own or are explicitly permitted to test. Read-only.
 *
 * Wiring (PN532 in I2C mode):
 *   SDA=GPIO8  SCL=GPIO9  VCC=3.3V  GND=GND  RSTPDN->3.3V
 *
 * Requires: Adafruit PN532 + Adafruit BusIO + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"5.3","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"5.3","status":"ok","data":{"uid":"04A2B3C4","type":"MIFARE Classic 1K","sectors":16,"blocks":64,"bytes":1024}}
 *   not present   : {"module":"5.3","status":"error","data":{"msg":"pn532_not_found"}}
 *
 * DEBUG_SERIAL = 1: starts on its own and prints one readable summary line
 *   per tag presentation:
 *       [5.3] UID 04A2B3C4  MIFARE Classic 1K  sectors=16 blocks=64 bytes=1024
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

Adafruit_PN532 nfc(PN532_IRQ, PN532_RST, &Wire);
JsonDocument doc;

bool mapActive  = false;
bool pn532Ready = false;
String lastUid  = "";
unsigned long lastPoll = 0;

uint8_t DEFAULT_KEY[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

String uidToHex(const uint8_t* uid, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) { if (uid[i] < 0x10) s += '0'; s += String(uid[i], HEX); }
  s.toUpperCase();
  return s;
}

void emit(const char* status) {
  doc["module"] = "5.3";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
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

// Fill the layout fields from the UID length plus (for Ultralight/NTAG) a
// probe of how many 4-byte pages actually read back. Classic 1K vs 4K is
// told apart by whether sector 16+ authenticates with the default key.
void describe(uint8_t* uid, uint8_t uidLen, String& type, int& sectors, int& blocks, long& bytes) {
  if (uidLen == 4) {
    // Probe block 64 (first block of 4K-only space) to distinguish 1K from 4K.
    bool is4k = nfc.mifareclassic_AuthenticateBlock(uid, uidLen, 64, 0, DEFAULT_KEY);
    if (is4k) { type = "MIFARE Classic 4K"; sectors = 40; blocks = 256; bytes = 4096; }
    else      { type = "MIFARE Classic 1K"; sectors = 16; blocks = 64;  bytes = 1024; }
    return;
  }
  if (uidLen == 7) {
    // Count readable 4-byte pages to size the Ultralight / NTAG part.
    int pages = 0;
    for (uint8_t p = 0; p < 231; p++) {   // NTAG216 tops out ~231 pages
      uint8_t d[4];
      if (!nfc.mifareultralight_ReadPage(p, d)) break;
      pages++;
    }
    blocks = pages; sectors = 0; bytes = (long)pages * 4;
    if      (pages >= 200) type = "NTAG216 (~888 user bytes)";
    else if (pages >= 120) type = "NTAG215 (~504 user bytes)";
    else if (pages >= 40)  type = "NTAG213 (~144 user bytes)";
    else                   type = "Ultralight / NTAG";
    return;
  }
  type = "ISO 14443A tag"; sectors = 0; blocks = 0; bytes = 0;
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] PN532 Module 5.3 Memory Structure ready.");

#if DEBUG_SERIAL
  if (ensurePn532()) {
    Serial.println("[+] DEBUG_SERIAL on: PN532 found. Hold a tag to profile its memory layout.");
    mapActive = true;
  } else {
    Serial.println("[!] DEBUG_SERIAL on: PN532 NOT found. Check SDA=8 SCL=9, 3.3V, I2C jumpers.");
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.3\"") != -1) {
      if (!ensurePn532()) { doc["data"]["msg"] = "pn532_not_found"; emit("error"); }
      else { mapActive = true; lastUid = ""; }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      mapActive = false;
    }
  }

  if (mapActive && pn532Ready && millis() - lastPoll >= POLL_EVERY) {
    lastPoll = millis();
    uint8_t uid[7] = {0};
    uint8_t uidLen = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, READ_TIMEOUT) && uidLen > 0) {
      String hexUid = uidToHex(uid, uidLen);
      if (hexUid != lastUid) {
        lastUid = hexUid;
        String type; int sectors = 0, blocks = 0; long bytes = 0;
        describe(uid, uidLen, type, sectors, blocks, bytes);
#if DEBUG_SERIAL
        Serial.print("[5.3] UID "); Serial.print(hexUid);
        Serial.print("  "); Serial.print(type);
        Serial.print("  sectors="); Serial.print(sectors);
        Serial.print(" blocks="); Serial.print(blocks);
        Serial.print(" bytes="); Serial.println(bytes);
#else
        doc["data"]["uid"]     = hexUid;
        doc["data"]["type"]    = type;
        doc["data"]["sectors"] = sectors;
        doc["data"]["blocks"]  = blocks;
        doc["data"]["bytes"]   = bytes;
        emit("ok");
#endif
      }
    } else {
      lastUid = "";
    }
  }
}
