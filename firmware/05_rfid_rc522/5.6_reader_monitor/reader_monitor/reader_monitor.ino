/*
 * Cyberdeck ESP32-S3 — Module 5.6: RC522 Reader Monitor / Tailgating Log (standalone)
 * ----------------------------------------------------------------------------------
 * A continuous, passive scan loop that logs every card presented to the reader —
 * UID, card type, a timestamp (ms since the monitor started) and a per-UID seen
 * count — into a small RAM ring buffer, and counts distinct credentials. It only
 * reads UIDs; it never authenticates, dumps or writes.
 *
 * This is the "what a skimmer / rogue reader harvests" view, and a practical
 * tailgating / occupancy-audit tool: leave it at a door and see how many distinct
 * credentials pass. Authorized / educational use only — run it where you are
 * permitted to observe.
 *
 * Non-blocking and millis()-gated like every other loop module, so STOP is always
 * serviced promptly.
 *
 * Wiring (RC522 on the shared nRF24 SPI bus, 3.3 V ONLY):
 *   SDA(SS) = GPIO10   RST = GPIO9
 *   SCK = GPIO12   MOSI = GPIO11   MISO = GPIO13   IRQ unused   VCC = 3.3V
 *
 * Requires: MFRC522 (miguelbalboa) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host  -> board : {"module":"5.6","action":"START"} / {"action":"STOP"}
 *   per read       : {"module":"5.6","status":"ok","data":{
 *                       "uid":"DEADBEEF","type":"MIFARE 1KB","t_ms":12840,"seen_count":3}}
 *   summary        : {"module":"5.6","status":"ok","data":{
 *                       "distinct":4,"total_reads":11,"summary":true}}
 *   errors         : rc522_not_found
 *
 * DEBUG_SERIAL: set to 1 for a readable log in the Serial Monitor.
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1

static const uint8_t RC522_SS_PIN   = 10;
static const uint8_t RC522_RST_PIN  = 9;
static const uint8_t RC522_SCK_PIN  = 12;
static const uint8_t RC522_MISO_PIN = 13;
static const uint8_t RC522_MOSI_PIN = 11;

static const int BAUD_RATE = 115200;
static const unsigned long POLL_INTERVAL_MS = 120;    // scan cadence
static const unsigned long SUMMARY_EVERY_MS = 5000;   // periodic distinct/total summary
static const unsigned long REREAD_GUARD_MS  = 800;    // ignore the same UID held in the field
static const byte RING_SIZE = 32;                     // distinct UIDs tracked in RAM

MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);
JsonDocument doc;

bool monitorActive = false;
bool readerReady = false;
unsigned long lastPoll = 0;
unsigned long lastSummary = 0;
unsigned long startedAt = 0;

struct Seen {
  String uid;
  String type;
  uint16_t count;
  unsigned long lastMs;
};
Seen ring[RING_SIZE];
byte ringCount = 0;       // distinct UIDs currently tracked (saturates at RING_SIZE)
byte ringHead = 0;        // next slot to overwrite once full
uint16_t totalReads = 0;

void emit(const char* status) {
  doc["module"] = "5.6";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}
void emitError(const char* msg) { doc["data"]["msg"] = msg; emit("error"); }

String toHex(const byte* buf, byte len) {
  String s = "";
  for (byte i = 0; i < len; i++) { if (buf[i] < 0x10) s += '0'; s += String(buf[i], HEX); }
  s.toUpperCase();
  return s;
}

bool ensureReader() {
  if (readerReady) return true;
  SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
  mfrc522.PCD_Init();
  delay(50);
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  if (v == 0x00 || v == 0xFF) return false;
  mfrc522.PCD_AntennaOn();
  readerReady = true;
  return true;
}

void resetLog() {
  for (byte i = 0; i < RING_SIZE; i++) { ring[i].uid = ""; ring[i].count = 0; }
  ringCount = 0; ringHead = 0; totalReads = 0;
  startedAt = millis();
}

// Find an existing entry for a UID, or -1.
int findUid(const String& uid) {
  for (byte i = 0; i < ringCount; i++) if (ring[i].uid == uid) return i;
  return -1;
}

void logCard() {
  String uid = toHex(mfrc522.uid.uidByte, mfrc522.uid.size);
  String type = String(mfrc522.PICC_GetTypeName(mfrc522.PICC_GetType(mfrc522.uid.sak)));
  unsigned long now = millis();

  int idx = findUid(uid);
  if (idx >= 0) {
    // Debounce a card left sitting in the field.
    if (now - ring[idx].lastMs < REREAD_GUARD_MS) { mfrc522.PICC_HaltA(); return; }
    ring[idx].count++;
    ring[idx].lastMs = now;
  } else {
    // New distinct UID -> insert (ring overwrites the oldest once full).
    byte slot;
    if (ringCount < RING_SIZE) { slot = ringCount++; }
    else { slot = ringHead; ringHead = (ringHead + 1) % RING_SIZE; }
    ring[slot].uid = uid; ring[slot].type = type; ring[slot].count = 1; ring[slot].lastMs = now;
    idx = slot;
  }
  totalReads++;

  unsigned long tMs = now - startedAt;
#if DEBUG_SERIAL
  char head[16];
  snprintf(head, sizeof(head), "[%7lu] ", tMs);
  Serial.print(head);
  Serial.print(uid); Serial.print(F("  ")); Serial.print(type);
  Serial.print(F("  x")); Serial.println(ring[idx].count);
#else
  doc["data"]["uid"]        = uid;
  doc["data"]["type"]       = type;
  doc["data"]["t_ms"]       = tMs;
  doc["data"]["seen_count"] = ring[idx].count;
  emit("ok");
#endif
  mfrc522.PICC_HaltA();
}

void emitSummary() {
#if DEBUG_SERIAL
  char line[56];
  snprintf(line, sizeof(line), "[sum] distinct=%u total_reads=%u\n", ringCount, totalReads);
  Serial.print(line);
#else
  doc["data"]["distinct"]    = ringCount;
  doc["data"]["total_reads"] = totalReads;
  doc["data"]["summary"]     = true;
  emit("ok");
#endif
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println(F("\n[+] RC522 Module 5.6 Reader Monitor ready."));

#if DEBUG_SERIAL
  if (ensureReader()) {
    Serial.println(F("[+] DEBUG_SERIAL on: RC522 found. Logging every card presented..."));
    monitorActive = true;
    resetLog();
  } else {
    Serial.println(F("[!] DEBUG_SERIAL on: RC522 NOT found. Check SDA=10 RST=9 SCK=12 MOSI=11 MISO=13, 3.3V."));
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.6\"") != -1) {
      if (!ensureReader()) emitError("rc522_not_found");
      else { monitorActive = true; resetLog(); lastSummary = millis(); }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      if (monitorActive) emitSummary();   // final tally on stop
      monitorActive = false;
      if (readerReady) { mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); }
    }
  }

  if (!monitorActive || !readerReady) return;

  if (millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      logCard();
    }
  }

  if (millis() - lastSummary >= SUMMARY_EVERY_MS) {
    lastSummary = millis();
    emitSummary();
  }
}
