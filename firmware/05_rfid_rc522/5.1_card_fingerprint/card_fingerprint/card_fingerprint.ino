/*
 * Cyberdeck ESP32-S3 — Module 5.1: RC522 Card Scan & Fingerprint (standalone)
 * ---------------------------------------------------------------------------
 * Detects any ISO-14443A card in the field and reports its UID, ATQA, SAK and
 * a derived human-readable card type (MIFARE Classic 1K/4K, Ultralight, NTAG,
 * DESFire, unknown 14443A-4). This is the foundation every other RFID module
 * builds on, and — like 4.1 does for the nRF24 — it doubles as the "is the
 * RC522 wired up?" check: if the reader never answers, it emits a one-shot
 * "rc522_not_found" error.
 *
 * Teaching point: a card UID is NOT a secret. It is broadcast in the clear to
 * anything that energises the field, which is exactly why UID-only access
 * control is weak. Authorized / educational use only — scan cards you own or
 * are permitted to test.
 *
 * Wiring (RC522 on the shared nRF24 SPI bus, 3.3 V ONLY — 5 V kills it):
 *   SDA(SS) = GPIO10   RST = GPIO9
 *   SCK = GPIO12   MOSI = GPIO11   MISO = GPIO13
 *   IRQ unused     VCC = 3.3V   GND = GND
 *
 * Requires: MFRC522 (miguelbalboa) + ArduinoJson v7.x (Library Manager).
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host  -> board : {"module":"5.1","action":"START"} / {"action":"STOP"}
 *   board -> host  : {"module":"5.1","status":"ok","data":{
 *                       "uid":"DEADBEEF","uid_len":4,"atqa":"0004","sak":"08",
 *                       "type":"MIFARE 1KB"}}
 *   errors         : {"module":"5.1","status":"error","data":{"msg":"rc522_not_found"}}
 *
 * DEBUG_SERIAL (verification mode for the Arduino Serial Monitor):
 *   Set DEBUG_SERIAL to 1, flash, open the Serial Monitor at 115200. Scanning
 *   starts on its own (no START needed) and each card is printed as a readable
 *   line instead of JSON:
 *       [card] UID DE:AD:BE:EF  ATQA 0004  SAK 08  -> MIFARE 1KB
 *   Typing START / STOP JSON in the monitor still works. Set DEBUG_SERIAL back
 *   to 0 for the dashboard — the JSON output is unchanged.
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1   // 1 = human-readable lines in Serial Monitor, 0 = JSON for the dashboard

static const uint8_t RC522_SS_PIN   = 10;
static const uint8_t RC522_RST_PIN  = 9;
static const uint8_t RC522_SCK_PIN  = 12;
static const uint8_t RC522_MISO_PIN = 13;
static const uint8_t RC522_MOSI_PIN = 11;

static const int BAUD_RATE = 115200;
static const unsigned long POLL_INTERVAL_MS = 150;   // how often we probe the field
static const unsigned long SAME_CARD_MS     = 1200;  // debounce: ignore the same UID held in the field

MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);
JsonDocument doc;

bool scanActive = false;
bool readerReady = false;
unsigned long lastPoll = 0;
unsigned long lastSeen = 0;
String lastUid = "";

void emit(const char* status) {
  doc["module"] = "5.1";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

void emitError(const char* msg) {
  doc["data"]["msg"] = msg;
  emit("error");
}

// Uppercase hex string of a byte buffer, no separators (e.g. "DEADBEEF").
String toHex(const byte* buf, byte len) {
  String s = "";
  for (byte i = 0; i < len; i++) {
    if (buf[i] < 0x10) s += '0';
    s += String(buf[i], HEX);
  }
  s.toUpperCase();
  return s;
}

// Lazy one-time bring-up + wiring check. The RC522 version register reads back
// 0x00 or 0xFF when nothing is answering on the bus, so that is our "not found".
bool ensureReader() {
  if (readerReady) return true;
  SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
  mfrc522.PCD_Init();
  delay(50);
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  if (v == 0x00 || v == 0xFF) return false;   // no reader on the bus
  mfrc522.PCD_AntennaOn();
  readerReady = true;
  return true;
}

void startScan() {
  if (!ensureReader()) { emitError("rc522_not_found"); return; }
  scanActive = true;
  lastUid = "";
  lastSeen = 0;
}

void stopScan() {
  scanActive = false;
  if (readerReady) {
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
  }
}

// Report one freshly-selected card. mfrc522.uid is populated; atqa is the
// 2-byte answer-to-request we captured during PICC_RequestA (wire order:
// atqa[0] = low byte, atqa[1] = high byte).
void reportCard(const byte* atqa) {
  String uid = toHex(mfrc522.uid.uidByte, mfrc522.uid.size);

  // Debounce a card left sitting in the field.
  if (uid == lastUid && (millis() - lastSeen) < SAME_CARD_MS) return;
  lastUid = uid;
  lastSeen = millis();

  byte sak = mfrc522.uid.sak;
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(sak);
  String typeName = String(mfrc522.PICC_GetTypeName(piccType));

#if DEBUG_SERIAL
  Serial.print(F("[card] UID "));
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) Serial.print('0');
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    if (i + 1 < mfrc522.uid.size) Serial.print(':');
  }
  char line[48];
  snprintf(line, sizeof(line), "  ATQA %02X%02X  SAK %02X  -> ", atqa[1], atqa[0], sak);
  Serial.print(line);
  Serial.println(typeName);
#else
  doc["data"]["uid"]     = uid;
  doc["data"]["uid_len"] = mfrc522.uid.size;
  char atqaHex[5];
  snprintf(atqaHex, sizeof(atqaHex), "%02X%02X", atqa[1], atqa[0]);  // MSB-first for readability
  doc["data"]["atqa"]    = atqaHex;
  char sakHex[3];
  snprintf(sakHex, sizeof(sakHex), "%02X", sak);
  doc["data"]["sak"]     = sakHex;
  doc["data"]["type"]    = typeName;
  emit("ok");
#endif

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println(F("\n[+] RC522 Module 5.1 Card Scan & Fingerprint ready."));

#if DEBUG_SERIAL
  if (ensureReader()) {
    Serial.println(F("[+] DEBUG_SERIAL on: RC522 found. Present a 13.56 MHz card to the antenna."));
    scanActive = true;
  } else {
    Serial.println(F("[!] DEBUG_SERIAL on: RC522 NOT found. Check SDA=10 RST=9 SCK=12 MOSI=11 MISO=13, 3.3V."));
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.1\"") != -1) {
      startScan();
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      stopScan();
    }
  }

  if (scanActive && readerReady && millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    // Do our own RequestA so we capture ATQA, then select the card.
    byte atqa[2];
    byte atqaSize = sizeof(atqa);
    MFRC522::StatusCode rc = mfrc522.PICC_RequestA(atqa, &atqaSize);
    if (rc == MFRC522::STATUS_OK || rc == MFRC522::STATUS_COLLISION) {
      if (mfrc522.PICC_ReadCardSerial()) reportCard(atqa);
    }
  }
}
