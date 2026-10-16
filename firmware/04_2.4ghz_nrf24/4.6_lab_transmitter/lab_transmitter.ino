/*
 * Cyberdeck ESP32-S3 — Module 4.6: nRF24 Lab Transmitter (standalone)
 * ----------------------------------------------------------------
 * Transmits a known test payload to YOUR OWN listening nRF24 node for
 * link and range testing. Sends an incrementing counter once every
 * 500 ms on a fixed test channel/address, at low PA, and reports how
 * many were sent and whether the receiver acked.
 *
 * Lab / self-test use only: point this at a second nRF24 you control
 * (running 4.2/4.5 on the same channel + address). It is not for
 * transmitting into devices you don't own.
 *
 * Wiring: CE=GPIO6 CSN=GPIO7 SCK=GPIO12 MISO=GPIO13 MOSI=GPIO11
 *         VCC=3.3V GND=GND (10uF cap across VCC/GND recommended).
 * Requires: RF24 (TMRh20) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"4.6","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"4.6","status":"ok","data":{"ch":76,"addr":"CYBER","sent":5,"ack":true}}
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

static const uint8_t  TEST_CH   = 76;                   // fixed lab channel
static const uint64_t TEST_ADDR = 0xCCBBDDEE01ULL;      // test pipe (lab link)

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
JsonDocument doc;

bool txActive = false;
bool nrfReady = false;
unsigned long sent = 0;
unsigned long lastTx = 0;

void emit(const char* status) {
  doc["module"] = "4.6";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

bool ensureNrf() {
  if (nrfReady) return true;
  SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);
  if (!radio.begin() || !radio.isChipConnected()) return false;
  radio.setPALevel(RF24_PA_LOW);        // low power — lab use
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(TEST_CH);
  radio.setPayloadSize(32);
  radio.openWritingPipe(TEST_ADDR);
  radio.stopListening();
  nrfReady = true;
  return true;
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] nRF24 Module 4.6 Lab Transmitter ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"4.6\"") != -1) {
      if (!ensureNrf()) {
        doc["data"]["msg"] = "nrf_not_found";
        emit("error");
      } else {
        sent = 0;
        txActive = true;
      }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      txActive = false;
    }
  }

  if (txActive && nrfReady && millis() - lastTx >= 500) {
    lastTx = millis();
    uint8_t payload[32] = {0};
    payload[0] = 'C'; payload[1] = 'Y'; payload[2] = 'B';
    payload[3] = (uint8_t)(sent & 0xFF);
    bool ack = radio.write(payload, 32);     // true if the node acked
    sent++;
    doc["data"]["ch"] = TEST_CH;
    doc["data"]["addr"] = "CYBER";
    doc["data"]["sent"] = sent;
    doc["data"]["ack"] = ack;
    emit("ok");
  }
}
