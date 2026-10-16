/*
 * Cyberdeck ESP32-S3 — Module 4.2: nRF24 Packet Sniffer (standalone)
 * ----------------------------------------------------------------
 * Listens on the nRF24 default pipe, hops channels 0-83 (2400-2483
 * MHz) once a second, and dumps any received payload as hex.
 *
 * Educational / authorized-lab RF diagnostics only — capture traffic
 * from radios you own or are permitted to test. This captures on the
 * opened pipe address; full promiscuous capture is a later refinement
 * and wants a confirmed nRF24L01+ (not a clone).
 *
 * Wiring (nRF24L01+ on the reserved SPI pins):
 *   CE=GPIO6  CSN=GPIO7  SCK=GPIO12  MISO=GPIO13  MOSI=GPIO11
 *   VCC=3.3V  GND=GND    (10uF cap across VCC/GND recommended.)
 *
 * Requires: RF24 (TMRh20) + ArduinoJson v7.x (Library Manager).
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"4.2","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"4.2","status":"ok","data":{"ch":12,"addr":"pipe1","len":32,"payload":"AB.."}}
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

static const uint8_t  SNIFF_START_CH = 0;
static const uint8_t  SNIFF_END_CH   = 83;             // 2400-2483 MHz
static const uint64_t SNIFF_ADDR = 0xE7E7E7E7E7ULL;    // nRF24 factory-default pipe

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
JsonDocument doc;

bool sniffActive = false;
bool nrfReady = false;
uint8_t channel = SNIFF_START_CH;
unsigned long lastHop = 0;

void emit(const char* status) {
  doc["module"] = "4.2";
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
  Serial.println("\n[+] nRF24 Module 4.2 Packet Sniffer ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"4.2\"") != -1) {
      if (!ensureNrf()) {
        doc["data"]["msg"] = "nrf_not_found";
        emit("error");
      } else {
        channel = SNIFF_START_CH;
        radio.setChannel(channel);
        radio.startListening();
        sniffActive = true;
      }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      sniffActive = false;
      if (nrfReady) radio.stopListening();
    }
  }

  if (sniffActive && nrfReady) {
    if (millis() - lastHop >= 1000) {       // hop channel once a second
      lastHop = millis();
      channel++;
      if (channel > SNIFF_END_CH) channel = SNIFF_START_CH;
      radio.stopListening();
      radio.setChannel(channel);
      radio.startListening();
    }
    if (radio.available()) {
      uint8_t buf[32];
      radio.read(buf, 32);
      String hex = "";
      for (uint8_t i = 0; i < 32; i++) {
        char b[3];
        sprintf(b, "%02X", buf[i]);
        hex += b;
      }
      doc["data"]["ch"] = channel;
      doc["data"]["addr"] = "pipe1";
      doc["data"]["len"] = 32;
      doc["data"]["payload"] = hex;
      emit("ok");
    }
  }
}
