/*
 * Cyberdeck ESP32-S3 — Module 4.1: nRF24 Spectrum Scan (standalone)
 * ----------------------------------------------------------------
 * Sweeps the 126 nRF24 channels (2400-2525 MHz) and reports carrier
 * energy per channel. Also acts as the nRF24 wiring test: if the
 * module doesn't answer, it emits a one-shot "nrf_not_found" error.
 *
 * Educational / authorized-lab RF diagnostics only — scan radios you
 * own or are permitted to test.
 *
 * Wiring (nRF24L01+ on the reserved SPI pins):
 *   CE=GPIO6  CSN=GPIO7  SCK=GPIO12  MISO=GPIO13  MOSI=GPIO11
 *   VCC=3.3V  GND=GND    (add a 10uF cap across VCC/GND — these
 *   modules brown out without one.)
 *
 * Requires: RF24 (TMRh20) + ArduinoJson v7.x (Library Manager).
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"4.1","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"4.1","status":"ok","data":{"ch":0,"mhz":2400,"level":3}}
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

static const int BAUD_RATE = 115200;

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
JsonDocument doc;

bool scanActive = false;
bool nrfReady = false;
uint8_t channel = 0;
unsigned long lastStep = 0;

void emit(const char* status) {
  doc["module"] = "4.1";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

// Lazy one-time bring-up; false means the module didn't answer.
bool ensureNrf() {
  if (nrfReady) return true;
  SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
  if (!radio.begin() || !radio.isChipConnected()) return false;
  radio.setAutoAck(false);
  radio.disableCRC();
  radio.setDataRate(RF24_1MBPS);
  radio.setPayloadSize(32);
  radio.stopListening();
  nrfReady = true;
  return true;
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] nRF24 Module 4.1 Spectrum Scan ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"4.1\"") != -1) {
      if (!ensureNrf()) {
        doc["data"]["msg"] = "nrf_not_found";
        emit("error");
      } else {
        scanActive = true;
        channel = 0;
      }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      scanActive = false;
      if (nrfReady) radio.stopListening();
    }
  }

  // One channel per pass, 32 RPD samples each — bounded, non-blocking.
  if (scanActive && nrfReady && millis() - lastStep >= 12) {
    lastStep = millis();
    radio.setChannel(channel);
    uint8_t hits = 0;
    for (uint8_t i = 0; i < 32; i++) {
      radio.startListening();
      delayMicroseconds(130);
      radio.stopListening();
      if (radio.testRPD()) hits++;
    }
    doc["data"]["ch"] = channel;
    doc["data"]["mhz"] = 2400 + channel;
    doc["data"]["level"] = hits;        // 0-32 carrier hits this pass
    emit("ok");
    channel = (channel + 1) % 126;
  }
}
