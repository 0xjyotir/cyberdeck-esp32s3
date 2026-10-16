/*
 * Cyberdeck ESP32-S3 — Module 4.5: nRF24 Packet Storage (standalone)
 * ----------------------------------------------------------------
 * Captures nRF24 frames into an in-RAM ring buffer and reports the
 * running stored count plus the most recent capture. A groundwork
 * module for SD / pcap logging later; for now it keeps the last
 * RING_SIZE packets in memory.
 *
 * Educational / authorized-lab RF diagnostics only.
 *
 * Wiring: CE=GPIO6 CSN=GPIO7 SCK=GPIO12 MISO=GPIO13 MOSI=GPIO11
 *         VCC=3.3V GND=GND (10uF cap across VCC/GND recommended).
 * Requires: RF24 (TMRh20) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"4.5","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"4.5","status":"ok","data":{"stored":7,"last_addr":"pipe1","last":"AB01.."}}
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

static const uint8_t  SNIFF_CH  = 2;                    // fixed capture channel
static const uint64_t SNIFF_ADDR = 0xE7E7E7E7E7ULL;
static const uint8_t  RING_SIZE = 32;                   // packets kept in RAM

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
JsonDocument doc;

bool storeActive = false;
bool nrfReady = false;
uint8_t ring[RING_SIZE][32];
uint8_t ringHead = 0;
unsigned long stored = 0;

void emit(const char* status) {
  doc["module"] = "4.5";
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
  radio.setChannel(SNIFF_CH);
  radio.openReadingPipe(1, SNIFF_ADDR);
  nrfReady = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] nRF24 Module 4.5 Packet Storage ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"4.5\"") != -1) {
      if (!ensureNrf()) {
        doc["data"]["msg"] = "nrf_not_found";
        emit("error");
      } else {
        stored = 0; ringHead = 0;
        radio.startListening();
        storeActive = true;
      }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      storeActive = false;
      if (nrfReady) radio.stopListening();
    }
  }

  if (storeActive && nrfReady && radio.available()) {
    radio.read(ring[ringHead], 32);
    String hex = "";
    for (uint8_t i = 0; i < 32; i++) {
      char b[3];
      sprintf(b, "%02X", ring[ringHead][i]);
      hex += b;
    }
    ringHead = (ringHead + 1) % RING_SIZE;
    stored++;
    doc["data"]["stored"] = stored;
    doc["data"]["last_addr"] = "pipe1";
    doc["data"]["last"] = hex;
    emit("ok");
  }
}
