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
 *
 * DEBUG_SERIAL (verification mode for the Arduino Serial Monitor):
 *   Set DEBUG_SERIAL to 1, flash, open the Serial Monitor at 115200.
 *   The scan starts on its own (no START command needed) and, instead
 *   of JSON, prints one "waterfall" row per full 126-channel sweep:
 *
 *       ch   0         1         2   ...        12
 *            0123456789012345678901234...   012345
 *       #001 ----1-------3A71----------...  ------  peak ch17 2417MHz 21/32
 *
 *   Each column is one channel: '-' = no carrier seen, 1-9 / A-F =
 *   rising energy (hit count / 2, capped at F). A Wi-Fi AP on channel
 *   1/6/11 shows up as a ~20-channel-wide band around 2412/2437/2462 MHz;
 *   a phone hotspot or microwave oven next to the board lights it up too.
 *   Typing STOP / START JSON in the monitor still works.
 *   Set DEBUG_SERIAL back to 0 for the dashboard — the JSON stays as-is.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1   // 1 = human-readable spectrum in Serial Monitor, 0 = JSON for the dashboard

static const uint8_t NRF_CE_PIN   = 6;
static const uint8_t NRF_CSN_PIN  = 7;
static const uint8_t NRF_SCK_PIN  = 12;
static const uint8_t NRF_MISO_PIN = 13;
static const uint8_t NRF_MOSI_PIN = 11;

static const int BAUD_RATE = 115200;

static const uint8_t NUM_CHANNELS   = 126;
static const uint8_t SAMPLES_PER_CH = 32;
static const uint8_t HEADER_EVERY   = 20;   // reprint channel ruler every N sweeps

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
JsonDocument doc;

bool scanActive = false;
bool nrfReady = false;
uint8_t channel = 0;
unsigned long lastStep = 0;

#if DEBUG_SERIAL
uint8_t  sweepLevels[NUM_CHANNELS];
uint32_t sweepCount = 0;
#endif

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

#if DEBUG_SERIAL
// Two-line channel ruler: tens digit, then units digit, one column per channel.
void printRuler() {
  Serial.print("\nch   ");
  for (uint8_t c = 0; c < NUM_CHANNELS; c++) Serial.print(c % 10 == 0 ? char('0' + (c / 10) % 10) : ' ');
  Serial.print("\n     ");
  for (uint8_t c = 0; c < NUM_CHANNELS; c++) Serial.print(char('0' + c % 10));
  Serial.println();
}

// One waterfall row for the sweep that just finished, plus the peak channel.
void printSweep() {
  if (sweepCount % HEADER_EVERY == 0) printRuler();
  sweepCount++;

  uint8_t peakCh = 0;
  for (uint8_t c = 0; c < NUM_CHANNELS; c++)
    if (sweepLevels[c] > sweepLevels[peakCh]) peakCh = c;

  char tag[8];
  snprintf(tag, sizeof(tag), "#%03lu ", (unsigned long)(sweepCount % 1000));
  Serial.print(tag);
  for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
    uint8_t h = sweepLevels[c];
    if (h == 0) { Serial.print('-'); continue; }
    uint8_t lvl = min(15, (h + 1) / 2);   // 1-32 hits -> 1-F
    Serial.print(lvl < 10 ? char('0' + lvl) : char('A' + lvl - 10));
  }
  if (sweepLevels[peakCh] == 0) {
    Serial.println("  quiet");
  } else {
    char peak[40];
    snprintf(peak, sizeof(peak), "  peak ch%u %uMHz %u/%u",
             peakCh, 2400 + peakCh, sweepLevels[peakCh], SAMPLES_PER_CH);
    Serial.println(peak);
  }
}
#endif

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] nRF24 Module 4.1 Spectrum Scan ready.");

#if DEBUG_SERIAL
  // Verification mode: start sweeping immediately so the Serial Monitor shows output.
  if (ensureNrf()) {
    Serial.println("[+] DEBUG_SERIAL on: nRF24 found, sweeping 2400-2525 MHz.");
    Serial.println("    '-' = quiet, 1..F = carrier energy per channel.");
    scanActive = true;
    channel = 0;
  } else {
    Serial.println("[!] DEBUG_SERIAL on: nRF24 NOT found. Check CE=6 CSN=7 SCK=12 MISO=13 MOSI=11, 3.3V, 10uF cap.");
  }
#endif
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
    for (uint8_t i = 0; i < SAMPLES_PER_CH; i++) {
      radio.startListening();
      delayMicroseconds(130);
      radio.stopListening();
      if (radio.testRPD()) hits++;
    }

#if DEBUG_SERIAL
    sweepLevels[channel] = hits;
    if (channel == NUM_CHANNELS - 1) printSweep();   // one row per full sweep
#else
    doc["data"]["ch"] = channel;
    doc["data"]["mhz"] = 2400 + channel;
    doc["data"]["level"] = hits;        // 0-32 carrier hits this pass
    emit("ok");
#endif
    channel = (channel + 1) % NUM_CHANNELS;
  }
}
