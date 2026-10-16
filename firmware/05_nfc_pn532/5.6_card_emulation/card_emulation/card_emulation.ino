/*
 * Cyberdeck ESP32-S3 — Module 5.6: PN532 Card Emulation (standalone)
 * ------------------------------------------------------------------
 * Puts the PN532 into target (card-emulation) mode so the deck itself
 * presents as an ISO 14443A tag. This is a SELF-TEST of the emulation
 * path: hold a phone (or another reader) to the PN532 antenna and it
 * should detect a tag. It emulates a fixed demo identity only.
 *
 * Educational / authorized-lab use only. Emulate ONLY an identity you
 * own or are explicitly authorized to present — never clone or spoof a
 * credential (access card, transit card, payment card) that is not
 * yours. This module intentionally uses a fixed demo UID and does not
 * clone any scanned card.
 *
 * Wiring (PN532 in I2C mode):
 *   SDA=GPIO8  SCL=GPIO9  VCC=3.3V  GND=GND  RSTPDN->3.3V
 *
 * Requires: Adafruit PN532 + Adafruit BusIO + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"5.6","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"5.6","status":"ok","data":{"uid":"08DEMO01","mode":"ISO14443A target","status":"selected by reader"}}
 *   waiting       : {"module":"5.6","status":"ok","data":{"uid":"08DEMO01","mode":"ISO14443A target","status":"waiting for reader"}}
 *   not present   : {"module":"5.6","status":"error","data":{"msg":"pn532_not_found"}}
 *
 * Note on blocking: the PN532's AsTarget() call waits (bounded by the
 * reader's own timeout, ~1s) for a reader to select the emulated card.
 * Like the BLE connect() case documented in AGENTS.md, this is a single
 * bounded call per loop pass, so STOP is still serviced between attempts.
 *
 * DEBUG_SERIAL = 1: starts emulating on its own; bring a phone near the
 *   antenna to see "selected by reader". Set DEBUG_SERIAL = 0 for the
 *   dashboard (JSON as above).
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

static const int      BAUD_RATE   = 115200;
static const uint32_t POLL_EVERY  = 200;   // ms between emulation attempts
static const char*    DEMO_UID    = "08DEMO01";  // label only — fixed demo identity

Adafruit_PN532 nfc(PN532_IRQ, PN532_RST, &Wire);
JsonDocument doc;

bool emuActive  = false;
bool pn532Ready = false;
bool lastSelected = false;     // edge-track so we don't spam identical lines
unsigned long lastAttempt = 0;

void emit(const char* status) {
  doc["module"] = "5.6";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

void emitState(const char* state) {
  doc["data"]["uid"]    = DEMO_UID;
  doc["data"]["mode"]   = "ISO14443A target";
  doc["data"]["status"] = state;
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

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] PN532 Module 5.6 Card Emulation ready.");
  Serial.println("    Emulates a FIXED demo identity only — never a credential you don't own.");

#if DEBUG_SERIAL
  if (ensurePn532()) {
    Serial.println("[+] DEBUG_SERIAL on: PN532 found. Emulating a tag — bring a phone near the antenna.");
    emuActive = true;
  } else {
    Serial.println("[!] DEBUG_SERIAL on: PN532 NOT found. Check SDA=8 SCL=9, 3.3V, I2C jumpers.");
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.6\"") != -1) {
      if (!ensurePn532()) { doc["data"]["msg"] = "pn532_not_found"; emit("error"); }
      else { emuActive = true; lastSelected = false; }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      emuActive = false;
    }
  }

  if (emuActive && pn532Ready && millis() - lastAttempt >= POLL_EVERY) {
    lastAttempt = millis();

    // One bounded attempt to be selected as a target. Returns 1 when a reader
    // activates the emulated card, 0 on timeout (no reader in range).
    uint8_t selected = nfc.AsTarget();

    if (selected) {
      if (!lastSelected) {
        lastSelected = true;
#if DEBUG_SERIAL
        Serial.println("[5.6] selected by reader  (emulated tag was read)");
#else
        emitState("selected by reader");
#endif
      }
    } else {
      if (lastSelected) {       // transitioned back to idle
        lastSelected = false;
#if DEBUG_SERIAL
        Serial.println("[5.6] waiting for reader");
#else
        emitState("waiting for reader");
#endif
      }
    }
  }
}
