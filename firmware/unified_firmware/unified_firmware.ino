/*
 * Cyberdeck ESP32-S3 — Unified Firmware
 * ----------------------------------------------------------------
 * Merges: 01_wifi (1.1-1.6), 02_ble (2.1-2.6), 07_infrared (7.1-7.6)
 *
 * IR module IDs follow firmware/07_infrared/7.1-7.6:
 *   7.1 38khz_modulation   — continuous 38kHz carrier on TX (LEDC)
 *   7.2 pulse_learning     — raw mark/space timings from RX
 *   7.3 signal_replay      — one-shot replay of the last capture
 *                            (decoded protocol if known, raw otherwise)
 *   7.4 protocol_libraries — protocol decode (NEC/Sony/RC5/RC6/...)
 *   7.5 signal_analysis    — mark/space totals + duty per capture
 *   7.6 led_circuit_tests  — 26us test pulse on TX every 10ms
 *
 * IR pin sharing: 7.1, 7.3 and 7.6 all drive IR_TX_PIN (7.1 through
 * LEDC, 7.3 through IRsend, 7.6 through digitalWrite); 7.2, 7.4 and
 * 7.5 share the one IRrecv on IR_RX_PIN. Only one module is ever
 * active, and startModule() now stops the previous module first, so
 * the LEDC channel is always detached before IRsend / digitalWrite
 * take the pin back.
 *
 * PROTOCOL (locked — see PROTOCOL.md for the full spec):
 *   Command  (host -> S3):  {"module":"1.1","action":"START"}
 *                            {"action":"STOP"}
 *   Telemetry (S3 -> host): {"module":"1.1","status":"ok","data":{...}}
 *
 * Requires: ArduinoJson v7.x  (Library Manager -> "ArduinoJson" by
 *           Benoit Blanchon)
 *           IRremoteESP8266   (Library Manager -> "IRremoteESP8266"
 *           by David Conran / crankyoldgit) — needed for modules
 *           7.3 / 7.4 only.
 *
 * NON-BLOCKING NOTES (read this before adding more modules):
 *   - All module polling is gated by millis(), not delay(). The
 *     outer loop() always returns quickly except for two known,
 *     bounded exceptions below.
 *   - BLE scanning (2.1, 2.3, and the discovery phase of 2.2/2.5/2.6)
 *     runs as a CONTINUOUS async scan (duration 0) with results
 *     delivered via the BLEAdvertisedDeviceCallbacks callback. The
 *     loop() body does not block waiting on scan results.
 *   - KNOWN LIMITATION: BLEClient::connect() (used in the GATT
 *     phase of 2.2 / 2.5 / 2.6) is a blocking call inside the
 *     ESP32 Arduino BLE library itself — there is no non-blocking
 *     variant without moving BLE onto its own FreeRTOS task, which
 *     is out of scope for this pass. Each connect attempt can still
 *     block for ~1-3s. What WAS fixed: module 2.6 previously ran
 *     up to 5 connect attempts back-to-back inside a single while()
 *     loop, blocking the whole firmware (incl. Serial command
 *     intake) for up to ~15s straight. It now does ONE attempt per
 *     loop() pass, so a STOP command is checked between attempts.
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <BLEAdvertising.h>
#include <ArduinoJson.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>

// ===================================================================
// PIN MAP (defaults — remap here if your wiring differs)
// ===================================================================
// IR
static const uint16_t IR_RX_PIN = 4;   // IR receiver diode (OUT -> GPIO4)
static const uint16_t IR_TX_PIN = 5;   // IR transmitter LED (via NPN/driver -> GPIO5)
// GPIO4 / GPIO5 are plain GPIOs on the S3: not strapping pins
// (0/3/45/46), not USB (19/20), not flash/PSRAM (26-37).
//
// Reserved for future modules (not used yet — documented here so
// 04_2.4ghz_nrf24 / 05_nfc_pn532 don't collide with IR or each other
// when they're merged in next):
//   nRF24L01  (SPI)  CE=GPIO6  CSN=GPIO7  SCK=GPIO12 MISO=GPIO13 MOSI=GPIO11
//                    (S3 default FSPI pins. Do NOT use 35/36/37: on
//                    octal-PSRAM boards like N8R8/N16R8 those are wired
//                    to the PSRAM and are unusable.)
//   PN532     (I2C)  SDA=GPIO8 SCL=GPIO9  (keep off the nRF24 SPI bus)

static const uint16_t IR_CAPTURE_BUFFER_SIZE = 1024;
static const uint8_t  IR_TIMEOUT = 15;  // ms of silence that ends a capture
static const uint32_t IR_CARRIER_HZ = 38000;   // 7.1 carrier frequency
static const uint8_t  IR_CARRIER_RES_BITS = 8; // 7.1 LEDC resolution (duty 0-255)
static const uint32_t IR_TEST_PULSE_US = 26;   // 7.6 pulse width (~one 38kHz period)
static const uint32_t IR_TEST_PERIOD_MS = 10;  // 7.6 pulse interval

// ===================================================================
// GLOBAL STATE
// ===================================================================
String activeModule = "NONE";
unsigned long moduleActionMillis = 0;   // last time this module "did its thing"
unsigned long moduleStateMillis = 0;    // generic per-module sub-state timer

volatile unsigned long packetCount = 0;

BLEScan* pBLEScan = nullptr;
BLEAdvertising* pAdvertising = nullptr;
String bleTargetAddr = "";
bool bleTargetFound = false;

// 2.6 connection-testing state (was an inline while() loop — now
// spread across loop() passes so it can't starve Serial/STOP)
int bleTestAttempts = 0;
static const int BLE_TEST_MAX_ATTEMPTS = 5;

// 1.1 async scan sub-state
enum WifiScanState { WSCAN_IDLE, WSCAN_RUNNING };
WifiScanState wifiScanState = WSCAN_IDLE;

IRrecv irrecv(IR_RX_PIN, IR_CAPTURE_BUFFER_SIZE, IR_TIMEOUT, true);
IRsend irsend(IR_TX_PIN);
decode_results irResults;
decode_type_t lastIrProtocol = decode_type_t::UNKNOWN;
uint64_t lastIrValue = 0;
uint16_t lastIrBits = 0;
bool lastIrCaptured = false;
// Raw timings (us, mark/space alternating) of the last capture from
// 7.2 / 7.4 / 7.5, so 7.3 can replay codes no decoder recognised.
uint16_t lastIrRaw[IR_CAPTURE_BUFFER_SIZE];
uint16_t lastIrRawLen = 0;
unsigned long irTestPulseCount = 0;   // 7.6 pulses fired since START

JsonDocument telemetryDoc;

// ===================================================================
// TELEMETRY (one shared helper so every module emits the same shape)
//   Caller fills telemetryDoc["data"][...] first, then calls this.
// ===================================================================
void emitTelemetry(const char* module, const char* status) {
  telemetryDoc["module"] = module;
  telemetryDoc["status"] = status;
  serializeJson(telemetryDoc, Serial);
  Serial.println();
  telemetryDoc.clear();
}

void emitError(const char* module, const char* msg) {
  telemetryDoc["data"]["msg"] = msg;
  emitTelemetry(module, "error");
}

// ===================================================================
// IR CAPTURE HELPER — remember the last capture (decoded + raw) for 7.3
// ===================================================================
void rememberIrCapture(const decode_results& r) {
  lastIrProtocol = r.decode_type;
  lastIrValue = r.value;
  lastIrBits = r.bits;
  lastIrRawLen = 0;
  // rawbuf[0] is the gap before the frame; real timings start at 1.
  for (uint16_t i = 1; i < r.rawlen && lastIrRawLen < IR_CAPTURE_BUFFER_SIZE; i++) {
    uint32_t us = (uint32_t)r.rawbuf[i] * kRawTick;
    lastIrRaw[lastIrRawLen++] = us > UINT16_MAX ? UINT16_MAX : (uint16_t)us;
  }
  lastIrCaptured = true;
}

// ===================================================================
// WIFI PROMISCUOUS CALLBACK (module 1.3)
// ===================================================================
void wifiPacketHandler(void* buf, wifi_promiscuous_pkt_type_t type) {
  packetCount++;
}

// ===================================================================
// BLE SCAN CALLBACK — shared by 2.1 / 2.2 / 2.3 / 2.5 / 2.6
// ===================================================================
class UnifiedBLECallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        if (activeModule == "2.1" || activeModule == "2.3") {
            String name = device.haveName() ? device.getName().c_str() : "Unknown";
            String addr = device.getAddress().toString().c_str();
            int rssi = device.getRSSI();
            String uuid = device.haveServiceUUID() ? device.getServiceUUID().toString().c_str() : "N/A";

            telemetryDoc["data"]["name"] = name;
            telemetryDoc["data"]["addr"] = addr;
            telemetryDoc["data"]["rssi"] = rssi;
            telemetryDoc["data"]["uuid"] = uuid;
            emitTelemetry(activeModule.c_str(), "ok");
        } else if ((activeModule == "2.2" || activeModule == "2.5" || activeModule == "2.6") && !bleTargetFound) {
            if (device.isConnectable()) {
                bleTargetAddr = device.getAddress().toString().c_str();
                bleTargetFound = true;
                if (pBLEScan) pBLEScan->stop();
            }
        }
    }
};

// ===================================================================
// MODULE START / STOP — one-time side effects live here, not in the
// polling loop, so entering a module never repeats init work.
// ===================================================================
void stopModule();

void startModule(const String& module) {
  // Tear down whatever was running so its hardware (WiFi mode, BLE
  // scan, LEDC on the IR TX pin, IRrecv interrupt) is released first.
  if (activeModule != "NONE") stopModule();

  activeModule = module;
  moduleActionMillis = 0;   // 0 forces the first poll to fire immediately
  moduleStateMillis = 0;
  bleTargetFound = false;
  bleTargetAddr = "";
  bleTestAttempts = 0;
  wifiScanState = WSCAN_IDLE;

  if (module == "1.3") {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifiPacketHandler);
  } else if (module == "1.5") {
    WiFi.softAP("Cyberdeck_AP", "cyberpass123");
  } else if (module == "2.1" || module == "2.3") {
    pBLEScan->start(0, nullptr, false);  // continuous async scan
  } else if (module == "2.2" || module == "2.5" || module == "2.6") {
    pBLEScan->start(0, nullptr, false);  // continuous async scan for target discovery
  } else if (module == "2.4") {
    if (pAdvertising) pAdvertising->start();
  } else if (module == "7.1") {
    if (!ledcAttach(IR_TX_PIN, IR_CARRIER_HZ, IR_CARRIER_RES_BITS)) {
      emitError("7.1", "ledc_attach_failed");
      activeModule = "NONE";
      return;
    }
    ledcWrite(IR_TX_PIN, 1 << (IR_CARRIER_RES_BITS - 1));  // 50% duty
  } else if (module == "7.2" || module == "7.4" || module == "7.5") {
    irrecv.enableIRIn();
  } else if (module == "7.3") {
    irsend.begin();
    // one-shot replay happens in the execution section below, then
    // the module auto-returns to NONE.
  } else if (module == "7.6") {
    pinMode(IR_TX_PIN, OUTPUT);
    digitalWrite(IR_TX_PIN, LOW);
    irTestPulseCount = 0;
  }

  telemetryDoc["data"]["msg"] = "module_active";
  emitTelemetry(module.c_str(), "ok");
}

void stopModule() {
  if (activeModule == "1.3") {
    esp_wifi_set_promiscuous(false);
    esp_wifi_stop();
  } else if (activeModule == "1.5") {
    WiFi.softAPdisconnect(true);
  } else if (activeModule == "2.1" || activeModule == "2.2" || activeModule == "2.3" ||
             activeModule == "2.5" || activeModule == "2.6") {
    if (pBLEScan) pBLEScan->stop();
  } else if (activeModule == "2.4" && pAdvertising) {
    pAdvertising->stop();
  } else if (activeModule == "7.1") {
    ledcWrite(IR_TX_PIN, 0);
    ledcDetach(IR_TX_PIN);
    pinMode(IR_TX_PIN, OUTPUT);
    digitalWrite(IR_TX_PIN, LOW);   // leave the IR LED off
  } else if (activeModule == "7.2" || activeModule == "7.4" || activeModule == "7.5") {
    irrecv.disableIRIn();
  } else if (activeModule == "7.6") {
    digitalWrite(IR_TX_PIN, LOW);
  } else if (activeModule == "1.1" && wifiScanState == WSCAN_RUNNING) {
    WiFi.scanDelete();
    wifiScanState = WSCAN_IDLE;
  }

  telemetryDoc["data"]["msg"] = "module_halted";
  emitTelemetry(activeModule.c_str(), "ok");
  activeModule = "NONE";
}

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);

  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  BLEDevice::init("Cyberdeck-S3");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new UnifiedBLECallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  pAdvertising = BLEDevice::getAdvertising();

  delay(100);
  Serial.println("\n[+] ESP32-S3 Cyberdeck Unified Firmware Online.");
}

// ===================================================================
// COMMAND PARSER
// ===================================================================
void handleSerialCommand() {
  if (Serial.available() <= 0) return;

  String incoming = Serial.readStringUntil('\n');
  incoming.trim();
  if (incoming.length() == 0) return;

  JsonDocument cmdDoc;
  DeserializationError err = deserializeJson(cmdDoc, incoming);
  if (err) {
    emitError("SYS", "malformed_json");
    return;
  }

  const char* action = cmdDoc["action"] | "";
  const char* module = cmdDoc["module"] | "NONE";

  if (strcmp(action, "START") == 0) {
    startModule(String(module));
  } else if (strcmp(action, "STOP") == 0) {
    stopModule();
  } else {
    emitError("SYS", "unknown_action");
  }
}

// ===================================================================
// LOOP
// ===================================================================
void loop() {
  handleSerialCommand();

  unsigned long now = millis();

  // --- WIFI SUBSYSTEM ---
  if (activeModule == "1.1") {
    // Async scan state machine: kick off -> poll -> report -> cooldown.
    if (wifiScanState == WSCAN_IDLE && now - moduleActionMillis >= 2000) {
      WiFi.scanNetworks(true, true);  // async=true, show_hidden=true
      wifiScanState = WSCAN_RUNNING;
      moduleActionMillis = now;
    } else if (wifiScanState == WSCAN_RUNNING) {
      int n = WiFi.scanComplete();
      if (n >= 0) {
        for (int i = 0; i < n; ++i) {
          String ssid = WiFi.SSID(i);
          if (ssid == "") ssid = "<Hidden>";
          telemetryDoc["data"]["ssid"] = ssid;
          telemetryDoc["data"]["bssid"] = WiFi.BSSIDstr(i);
          telemetryDoc["data"]["ch"] = WiFi.channel(i);
          telemetryDoc["data"]["rssi"] = WiFi.RSSI(i);
          emitTelemetry("1.1", "ok");
        }
        WiFi.scanDelete();
        wifiScanState = WSCAN_IDLE;
      } else if (n == WIFI_SCAN_FAILED) {
        wifiScanState = WSCAN_IDLE;
      }
      // n == WIFI_SCAN_RUNNING: keep polling next pass, no block.
    }
  }
  else if (activeModule == "1.2") {
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      uint8_t primaryChannel;
      wifi_second_chan_t secondChannel;
      esp_wifi_get_channel(&primaryChannel, &secondChannel);
      telemetryDoc["data"]["ssid"] = "Current_Channel_Monitor";
      telemetryDoc["data"]["bssid"] = "ESP32_S3_NODE";
      telemetryDoc["data"]["ch"] = primaryChannel;
      telemetryDoc["data"]["rssi"] = WiFi.RSSI();
      emitTelemetry("1.2", "ok");
    }
  }
  else if (activeModule == "1.3") {
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      telemetryDoc["data"]["ssid"] = "[RAW_PACKET_COUNT]";
      telemetryDoc["data"]["bssid"] = "PROMISC_MODE";
      telemetryDoc["data"]["ch"] = WiFi.channel();
      telemetryDoc["data"]["rssi"] = -(long)(packetCount % 90);
      emitTelemetry("1.3", "ok");
    }
  }
  else if (activeModule == "1.4") {
    if (now - moduleActionMillis >= 2000) {
      moduleActionMillis = now;
      wifi_sta_list_t station;
      esp_wifi_ap_get_sta_list(&station);
      telemetryDoc["data"]["ssid"] = String("Connected_Clients: ") + station.num;
      telemetryDoc["data"]["bssid"] = "AP_CLIENT_SCAN";
      telemetryDoc["data"]["ch"] = 1;
      telemetryDoc["data"]["rssi"] = -50;
      emitTelemetry("1.4", "ok");
    }
  }
  else if (activeModule == "1.5") {
    if (now - moduleActionMillis >= 3000) {
      moduleActionMillis = now;
      IPAddress ip = WiFi.softAPIP();
      telemetryDoc["data"]["ssid"] = String("IP: ") + ip.toString();
      telemetryDoc["data"]["bssid"] = "AP_ACTIVE_HOST";
      telemetryDoc["data"]["ch"] = 1;
      telemetryDoc["data"]["rssi"] = -10;
      emitTelemetry("1.5", "ok");
    }
  }
  else if (activeModule == "1.6") {
    if (now - moduleActionMillis >= 2000) {
      moduleActionMillis = now;
      uint8_t packet[128];
      memset(packet, 0xFF, 32);
      esp_err_t res = esp_wifi_80211_tx(WIFI_IF_STA, packet, sizeof(packet), true);
      telemetryDoc["data"]["ssid"] = String("Frame_Tx_Status: ") + (res == ESP_OK ? "SUCCESS" : "FAIL");
      telemetryDoc["data"]["bssid"] = "INJECTION_LAB";
      telemetryDoc["data"]["ch"] = WiFi.channel();
      telemetryDoc["data"]["rssi"] = -25;
      emitTelemetry("1.6", "ok");
    }
  }

  // --- BLE SUBSYSTEM ---
  // 2.1 / 2.3: fully callback-driven now (continuous scan started in
  // startModule()). Just housekeeping here: periodically clear the
  // scan result cache so it doesn't grow unbounded.
  else if (activeModule == "2.1" || activeModule == "2.3") {
    if (now - moduleActionMillis >= 3000) {
      moduleActionMillis = now;
      pBLEScan->clearResults();
    }
  }
  else if (activeModule == "2.2" || activeModule == "2.5") {
    if (!bleTargetFound) {
      // discovery phase: callback-driven, nothing to poll
    } else {
      // one connect+read attempt, then return to discovery
      BLEClient* pClient = BLEDevice::createClient();
      if (pClient->connect(BLEAddress(bleTargetAddr.c_str()))) {
        std::map<std::string, BLERemoteService*>* pServices = pClient->getServices();
        if (pServices != nullptr) {
          for (auto const& [suuid, pService] : *pServices) {
            std::map<std::string, BLERemoteCharacteristic*>* pChars = pService->getCharacteristics();
            if (pChars != nullptr) {
              for (auto const& [cuuid, pChar] : *pChars) {
                telemetryDoc["data"]["name"] = bleTargetAddr;
                telemetryDoc["data"]["uuid"] = pChar->getUUID().toString().c_str();
                telemetryDoc["data"]["rssi"] = -50;
                if (activeModule == "2.2") {
                  telemetryDoc["data"]["payload"] = String("Service: ") + pService->getUUID().toString().c_str();
                } else {  // 2.5 — also attempt a read
                  String hexVal = "LOCKED";
                  if (pChar->canRead()) {
                    String val = pChar->readValue();
                    hexVal = "";
                    for (unsigned int i = 0; i < val.length(); i++) {
                      char buf[3];
                      sprintf(buf, "%02X", (uint8_t)val[i]);
                      hexVal += buf;
                    }
                  }
                  telemetryDoc["data"]["payload"] = hexVal;
                }
                emitTelemetry(activeModule.c_str(), "ok");
              }
            }
          }
        }
        pClient->disconnect();
      } else {
        emitError(activeModule.c_str(), "connect_failed");
      }
      bleTargetFound = false;
      pBLEScan->start(0, nullptr, false);  // resume discovery
    }
  }
  else if (activeModule == "2.4") {
    if (now - moduleActionMillis >= 1500) {
      moduleActionMillis = now;
      telemetryDoc["data"]["name"] = "Cyberdeck-Beacon";
      telemetryDoc["data"]["uuid"] = "Broadcast Active";
      telemetryDoc["data"]["rssi"] = -10;
      telemetryDoc["data"]["payload"] = "BEACON_FLOOD";
      emitTelemetry("2.4", "ok");
    }
  }
  else if (activeModule == "2.6") {
    if (!bleTargetFound) {
      // discovery phase: callback-driven
    } else if (bleTestAttempts < BLE_TEST_MAX_ATTEMPTS) {
      // ONE attempt per loop() pass — this replaces the old inline
      // while() loop that blocked for up to 5 attempts straight.
      if (now - moduleActionMillis >= 1000) {
        moduleActionMillis = now;
        bleTestAttempts++;
        BLEClient* client = BLEDevice::createClient();
        unsigned long connStart = millis();
        if (client->connect(BLEAddress(bleTargetAddr.c_str()))) {
          unsigned long latency = millis() - connStart;
          telemetryDoc["data"]["name"] = bleTargetAddr;
          telemetryDoc["data"]["uuid"] = String("Attempt ") + bleTestAttempts;
          telemetryDoc["data"]["rssi"] = -40;
          telemetryDoc["data"]["payload"] = String("SUCCESS: ") + latency + "ms";
          emitTelemetry("2.6", "ok");
          client->disconnect();
        } else {
          telemetryDoc["data"]["name"] = bleTargetAddr;
          telemetryDoc["data"]["uuid"] = String("Attempt ") + bleTestAttempts;
          telemetryDoc["data"]["rssi"] = -99;
          telemetryDoc["data"]["payload"] = "FAILED";
          emitTelemetry("2.6", "error");
        }
      }
    } else {
      // test complete
      telemetryDoc["data"]["msg"] = "test_complete";
      emitTelemetry("2.6", "ok");
      bleTargetFound = false;
      bleTestAttempts = 0;
      activeModule = "NONE";
    }
  }

  // --- INFRARED SUBSYSTEM ---
  // 7.4 = protocol_libraries (decode via IRremoteESP8266: NEC/Sony/RC5/RC6/...)
  else if (activeModule == "7.4") {
    // irrecv.decode() is non-blocking: it just checks whether the
    // interrupt-driven capture buffer has a finished result.
    if (irrecv.decode(&irResults)) {
      rememberIrCapture(irResults);

      telemetryDoc["data"]["protocol"] = typeToString(irResults.decode_type);
      telemetryDoc["data"]["value"] = uint64ToString(irResults.value, 16);
      telemetryDoc["data"]["bits"] = irResults.bits;
      emitTelemetry("7.4", "ok");

      irrecv.resume();
    }
  }
  // 7.2 = pulse_learning (raw mark/space timings of each capture)
  else if (activeModule == "7.2") {
    if (irrecv.decode(&irResults)) {
      rememberIrCapture(irResults);

      telemetryDoc["data"]["count"] = lastIrRawLen;
      telemetryDoc["data"]["overflow"] = irResults.overflow;
      JsonArray timings = telemetryDoc["data"]["timings_us"].to<JsonArray>();
      for (uint16_t i = 0; i < lastIrRawLen; i++) timings.add(lastIrRaw[i]);
      emitTelemetry("7.2", "ok");

      irrecv.resume();
    }
  }
  // 7.5 = signal_analysis (mark/space totals per capture)
  else if (activeModule == "7.5") {
    if (irrecv.decode(&irResults)) {
      rememberIrCapture(irResults);

      uint32_t markTime = 0;
      uint32_t spaceTime = 0;
      for (uint16_t i = 0; i < lastIrRawLen; i++) {
        // even index = mark (LED on), odd index = space (LED off)
        if (i % 2 == 0) markTime += lastIrRaw[i];
        else            spaceTime += lastIrRaw[i];
      }
      uint32_t total = markTime + spaceTime;

      telemetryDoc["data"]["pulses"] = lastIrRawLen;
      telemetryDoc["data"]["mark_us"] = markTime;
      telemetryDoc["data"]["space_us"] = spaceTime;
      telemetryDoc["data"]["total_us"] = total;
      telemetryDoc["data"]["duty_pct"] = total ? (100.0f * markTime / total) : 0.0f;
      telemetryDoc["data"]["protocol"] = typeToString(irResults.decode_type);
      emitTelemetry("7.5", "ok");

      irrecv.resume();
    }
  }
  // 7.3 = signal_replay (one-shot replay of the last 7.2/7.4/7.5 capture)
  else if (activeModule == "7.3") {
    if (!lastIrCaptured) {
      emitError("7.3", "no_code_captured");
    } else if (lastIrProtocol != decode_type_t::UNKNOWN) {
      irsend.send(lastIrProtocol, lastIrValue, lastIrBits);
      telemetryDoc["data"]["protocol"] = typeToString(lastIrProtocol);
      telemetryDoc["data"]["value"] = uint64ToString(lastIrValue, 16);
      telemetryDoc["data"]["bits"] = lastIrBits;
      telemetryDoc["data"]["action"] = "replayed";
      emitTelemetry("7.3", "ok");
    } else {
      // No decoder matched: resend the raw timings on a 38kHz carrier.
      irsend.sendRaw(lastIrRaw, lastIrRawLen, IR_CARRIER_HZ / 1000);
      telemetryDoc["data"]["protocol"] = "RAW";
      telemetryDoc["data"]["count"] = lastIrRawLen;
      telemetryDoc["data"]["action"] = "replayed";
      emitTelemetry("7.3", "ok");
    }
    activeModule = "NONE";
  }
  // 7.1 = 38khz_modulation (carrier runs in hardware; just report)
  else if (activeModule == "7.1") {
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      telemetryDoc["data"]["freq_hz"] = ledcReadFreq(IR_TX_PIN);
      telemetryDoc["data"]["duty_pct"] = 50;
      telemetryDoc["data"]["pin"] = IR_TX_PIN;
      emitTelemetry("7.1", "ok");
    }
  }
  // 7.6 = led_circuit_tests (26us pulse every 10ms, count reported 1/s)
  else if (activeModule == "7.6") {
    if (now - moduleStateMillis >= IR_TEST_PERIOD_MS) {
      moduleStateMillis = now;
      digitalWrite(IR_TX_PIN, HIGH);
      delayMicroseconds(IR_TEST_PULSE_US);  // bounded 26us busy-wait
      digitalWrite(IR_TX_PIN, LOW);
      irTestPulseCount++;
    }
    if (now - moduleActionMillis >= 1000) {
      moduleActionMillis = now;
      telemetryDoc["data"]["pulses"] = irTestPulseCount;
      telemetryDoc["data"]["pulse_us"] = IR_TEST_PULSE_US;
      telemetryDoc["data"]["period_ms"] = IR_TEST_PERIOD_MS;
      telemetryDoc["data"]["pin"] = IR_TX_PIN;
      emitTelemetry("7.6", "ok");
    }
  }

  // No trailing delay(): loop() is now gated entirely by the
  // millis() checks above, so idle passes return immediately.
}
