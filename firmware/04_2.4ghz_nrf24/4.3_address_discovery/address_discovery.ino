/*
 * Cyberdeck ESP32-S3 — Module 4.3: nRF24 Address Discovery (standalone)
 * ----------------------------------------------------------------
 * Pseudo-promiscuous address discovery: sets a 2-byte address width and
 * listens on a repeating preamble, so any nearby nRF24 traffic slips
 * through; the leading payload bytes are reported as a candidate pipe
 * address. Hops channels to cover the band.
 *
 * Educational / authorized-lab RF diagnostics only — discover addresses
 * of radios you own or are permitted to test.
 *
 * Wiring: CE=GPIO6 CSN=GPIO7 SCK=GPIO12 MISO=GPIO13 MOSI=GPIO11
 *         VCC=3.3V GND=GND (10uF cap across VCC/GND recommended).
 * Requires: RF24 (TMRh20) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"4.3","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"4.3","status":"ok","data":{"ch":12,"addr":"A1B2C3D4E5","hits":4}}
 *
 * NOTE: the preamble-sniff method is chip-dependent and noisy; genuine
 * nRF24L01+ modules work best. Candidate addresses may need a few
 * sightings ("hits") before they're trustworthy.
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

static const uint8_t SCAN_START_CH = 0;
static const uint8_t SCAN_END_CH   = 83;
static const uint8_t PREAMBLE[2]   = { 0xAA, 0xAA };  // repeating-bit preamble

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
JsonDocument doc;

bool discoverActive = false;
bool nrfReady = false;
uint8_t channel = SCAN_START_CH;
unsigned long hits = 0;
unsigned long lastHop = 0;

void emit(const char* status) {
  doc["module"] = "4.3";
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
  radio.setAddressWidth(2);            // 2-byte width => preamble sniffing
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(32);
  radio.openReadingPipe(0, PREAMBLE);
  nrfReady = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] nRF24 Module 4.3 Address Discovery ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"4.3\"") != -1) {
      if (!ensureNrf()) {
        doc["data"]["msg"] = "nrf_not_found";
        emit("error");
      } else {
        channel = SCAN_START_CH; hits = 0;
        radio.setChannel(channel);
        radio.startListening();
        discoverActive = true;
      }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      discoverActive = false;
      if (nrfReady) radio.stopListening();
    }
  }

  if (discoverActive && nrfReady) {
    if (millis() - lastHop >= 300) {          // hop to sweep the band
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
      hits++;
      // leading 5 bytes are the candidate pipe address
      String addr = "";
      for (uint8_t i = 0; i < 5; i++) {
        char b[3];
        sprintf(b, "%02X", buf[i]);
        addr += b;
      }
      doc["data"]["ch"] = channel;
      doc["data"]["addr"] = addr;
      doc["data"]["hits"] = hits;
      emit("ok");
    }
  }
}
