/*
 * Cyberdeck ESP32-S3 — Module 4.4: nRF24 Protocol Analysis (standalone)
 * ----------------------------------------------------------------
 * Captures nRF24 frames on the default pipe and reports a quick
 * breakdown per packet: channel, length, data rate, and a heuristic
 * best-guess at the traffic type (control vs data) from the payload.
 *
 * Educational / authorized-lab RF diagnostics only.
 *
 * Wiring: CE=GPIO6 CSN=GPIO7 SCK=GPIO12 MISO=GPIO13 MOSI=GPIO11
 *         VCC=3.3V GND=GND (10uF cap across VCC/GND recommended).
 * Requires: RF24 (TMRh20) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"4.4","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"4.4","status":"ok","data":{"ch":12,"addr":"pipe1","len":32,"rate":"2MBPS","guess":"control"}}
 *
 * NOTE: the "guess" is a simple heuristic (payload density), not a real
 * protocol decoder — a starting point for classification.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ArduinoJson.h>

static const uint8_t NRF_CE_PIN   = 6;
static const uint8_t NRF_CSN_PIN  = 7;
static const uint8_t NRF_SCK_PIN  = 12;
static const uint8_t NRF_MISO_PIN = 13;
static const uint8_t NRF_MOSI_PIN = 11;

static const uint8_t  SCAN_START_CH = 0;
static const uint8_t  SCAN_END_CH   = 83;
static const uint64_t SNIFF_ADDR = 0xE7E7E7E7E7ULL;

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
JsonDocument doc;

bool analyzeActive = false;
bool nrfReady = false;
uint8_t channel = SCAN_START_CH;
unsigned long lastHop = 0;

void emit(const char* status) {
  doc["module"] = "4.4";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

bool ensureNrf() {
  if (nrfReady) return true;
  SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
  if (!radio.begin() || !radio.isChipConnected()) return false;
  radio.setAutoAck(false);
  radio.disableCRC();
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(32);
  radio.openReadingPipe(1, SNIFF_ADDR);
  nrfReady = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] nRF24 Module 4.4 Protocol Analysis ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"4.4\"") != -1) {
      if (!ensureNrf()) {
        doc["data"]["msg"] = "nrf_not_found";
        emit("error");
      } else {
        channel = SCAN_START_CH;
        radio.setChannel(channel);
        radio.startListening();
        analyzeActive = true;
      }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      analyzeActive = false;
      if (nrfReady) radio.stopListening();
    }
  }

  if (analyzeActive && nrfReady) {
    if (millis() - lastHop >= 1000) {
      lastHop = millis();
      channel++;
      if (channel > SCAN_END_CH) channel = SCAN_START_CH;
      radio.stopListening();
      radio.setChannel(channel);
      radio.startListening();
    }
    if (radio.available()) {
      uint8_t buf[32];
      radio.read(buf, 32);
      uint8_t nonZero = 0;
      for (uint8_t i = 0; i < 32; i++) if (buf[i] != 0) nonZero++;
      const char* guess = nonZero <= 8 ? "control" : "data";
      doc["data"]["ch"] = channel;
      doc["data"]["addr"] = "pipe1";
      doc["data"]["len"] = 32;
      doc["data"]["rate"] = "2MBPS";
      doc["data"]["guess"] = guess;
      emit("ok");
    }
  }
}
