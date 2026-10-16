/*
 * Cyberdeck ESP32-S3 — Module 5.1: PN532 NFC Detection (standalone)
 * -----------------------------------------------------------------
 * Polls for ISO 14443A (MIFARE) tags over I2C and reports each tag's
 * UID and a card-type heuristic as it appears. Also acts as the PN532
 * wiring check: if the reader doesn't answer on I2C, it emits a
 * one-shot "pn532_not_found" error.
 *
 * Educational / authorized-lab NFC diagnostics only — read tags and
 * cards you own or are explicitly permitted to test.
 *
 * Wiring (PN532 breakout in I2C mode — set the onboard DIP/solder
 * jumpers to I2C):
 *   SDA=GPIO8  SCL=GPIO9  VCC=3.3V  GND=GND
 *   RSTPDN -> 3.3V (or a spare GPIO if you want hard resets).
 *   IRQ    -> unused here (polled, not interrupt-driven).
 *
 * Requires: Adafruit PN532 + Adafruit BusIO + ArduinoJson v7.x
 *           (all via Library Manager).
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"5.1","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"5.1","status":"ok","data":{"uid":"04A2B3C4","uid_len":4,"type":"MIFARE 1K / NTAG (4-byte UID)"}}
 *   not present   : {"module":"5.1","status":"error","data":{"msg":"pn532_not_found"}}
 *
 * DEBUG_SERIAL (verification mode for the Arduino Serial Monitor):
 *   Set DEBUG_SERIAL to 1, flash, open the Serial Monitor at 115200.
 *   Detection starts on its own (no START command needed) and prints a
 *   human-readable line each time a tag is seen or removed:
 *
 *       [5.1] found  UID 04 A2 B3 C4         len=4  MIFARE 1K / NTAG (4-byte UID)
 *       [5.1] removed
 *
 *   Hold a tag to the antenna; lift it off to see "removed". Typing STOP
 *   / START JSON in the monitor still works. Set DEBUG_SERIAL back to 0
 *   for the dashboard — the JSON output stays exactly as above.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1   // 1 = human-readable in Serial Monitor, 0 = JSON for the dashboard

static const uint8_t PN532_SDA = 8;
static const uint8_t PN532_SCL = 9;
static const int8_t  PN532_IRQ = -1;    // not used in polled I2C mode
static const int8_t  PN532_RST = -1;    // tie RSTPDN to 3.3V, or set a GPIO here

static const int      BAUD_RATE   = 115200;
static const uint32_t POLL_EVERY  = 300;   // ms between tag polls
static const uint16_t READ_TIMEOUT = 120;  // ms per readPassiveTargetID (bounded, non-blocking enough)

Adafruit_PN532 nfc(PN532_IRQ, PN532_RST, &Wire);
JsonDocument doc;

bool detectActive = false;
bool pn532Ready   = false;
bool tagPresent   = false;         // simple edge tracking so we don't spam duplicates
String lastUid    = "";
unsigned long lastPoll = 0;

// Rough card-type guess from UID length (true ATQA/SAK needs a lower-level
// InListPassiveTarget exchange; length is enough to tell the common families apart).
const char* guessType(uint8_t len) {
  if (len == 4) return "MIFARE 1K/4K or NTAG (4-byte UID)";
  if (len == 7) return "Ultralight/NTAG21x or DESFire (7-byte UID)";
  if (len == 10) return "ISO 14443A (10-byte UID)";
  return "ISO 14443A tag";
}

String uidToHex(const uint8_t* uid, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += '0';
    s += String(uid[i], HEX);
  }
  s.toUpperCase();
  return s;
}

void emit(const char* status) {
  doc["module"] = "5.1";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

// Lazy one-time bring-up; false means the PN532 didn't answer on I2C.
bool ensurePn532() {
  if (pn532Ready) return true;
  Wire.begin(PN532_SDA, PN532_SCL);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) return false;   // 0 = no reader on the bus
  nfc.SAMConfig();                                // normal reader mode
  pn532Ready = true;
  return true;
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] PN532 Module 5.1 NFC Detection ready.");

#if DEBUG_SERIAL
  if (ensurePn532()) {
    Serial.println("[+] DEBUG_SERIAL on: PN532 found, polling for ISO 14443A tags.");
    Serial.println("    Hold a tag to the antenna; lift it off to see 'removed'.");
    detectActive = true;
  } else {
    Serial.println("[!] DEBUG_SERIAL on: PN532 NOT found. Check SDA=8 SCL=9, 3.3V, I2C jumpers, RSTPDN high.");
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.1\"") != -1) {
      if (!ensurePn532()) {
        doc["data"]["msg"] = "pn532_not_found";
        emit("error");
      } else {
        detectActive = true;
        tagPresent = false;
        lastUid = "";
      }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      detectActive = false;
    }
  }

  if (detectActive && pn532Ready && millis() - lastPoll >= POLL_EVERY) {
    lastPoll = millis();
    uint8_t uid[7] = {0};
    uint8_t uidLen = 0;

    // Bounded read: returns within READ_TIMEOUT whether or not a tag is present.
    bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, READ_TIMEOUT);

    if (found && uidLen > 0) {
      String hexUid = uidToHex(uid, uidLen);
      if (!tagPresent || hexUid != lastUid) {    // only report new arrivals
        tagPresent = true;
        lastUid = hexUid;
#if DEBUG_SERIAL
        Serial.print("[5.1] found  UID ");
        for (uint8_t i = 0; i < uidLen; i++) { if (uid[i] < 0x10) Serial.print('0'); Serial.print(uid[i], HEX); Serial.print(' '); }
        Serial.print(" len="); Serial.print(uidLen);
        Serial.print("  "); Serial.println(guessType(uidLen));
#else
        doc["data"]["uid"] = hexUid;
        doc["data"]["uid_len"] = uidLen;
        doc["data"]["type"] = guessType(uidLen);
        emit("ok");
#endif
      }
    } else if (tagPresent) {                     // tag just left the field
      tagPresent = false;
      lastUid = "";
#if DEBUG_SERIAL
      Serial.println("[5.1] removed");
#else
      doc["data"]["uid"] = "";
      doc["data"]["type"] = "removed";
      emit("ok");
#endif
    }
  }
}
