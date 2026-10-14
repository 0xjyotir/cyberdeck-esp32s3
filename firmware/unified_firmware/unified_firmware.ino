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

String activeModule = "NONE";
volatile unsigned long packetCount = 0;
BLEScan* pBLEScan = nullptr;
BLEAdvertising* pAdvertising = nullptr;
String bleTargetAddr = "";
bool bleTargetFound = false;

// WiFi Promiscuous Callback[cite: 5]
void wifiPacketHandler(void* buf, wifi_promiscuous_pkt_type_t type) {
  packetCount++;
}

// BLE Scan Callback for 2.1 & 2.2 & 2.5 & 2.6
class UnifiedBLECallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        if (activeModule == "2.1") {
            String name = device.haveName() ? device.getName().c_str() : "Unknown";
            String addr = device.getAddress().toString().c_str();
            int rssi = device.getRSSI();
            String uuid = device.haveServiceUUID() ? device.getServiceUUID().toString().c_str() : "N/A";
            
            Serial.print("{\"name\":\"");
            Serial.print(name);
            Serial.print("\",\"addr\":\"");
            Serial.print(addr);
            Serial.print("\",\"rssi\":");
            Serial.print(rssi);
            Serial.print(",\"uuid\":\"");
            Serial.print(uuid);
            Serial.println("\"}");
        } else if ((activeModule == "2.2" || activeModule == "2.5" || activeModule == "2.6") && !bleTargetFound) {
            if (device.isConnectable()) {
                bleTargetAddr = device.getAddress().toString().c_str();
                bleTargetFound = true;
                if (pBLEScan) pBLEScan->stop();
            }
        }
    }
};

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

void loop() {
  // Handle Serial Commands
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();

    if (incoming.indexOf("\"action\":\"START\"") != -1) {
      // --- WIFI MODULES ---
      if (incoming.indexOf("\"module\":\"1.1\"") != -1) { activeModule = "1.1"; Serial.println("[+] Module 1.1 Active"); }
      else if (incoming.indexOf("\"module\":\"1.2\"") != -1) { activeModule = "1.2"; Serial.println("[+] Module 1.2 Active"); }
      else if (incoming.indexOf("\"module\":\"1.3\"") != -1) { 
        activeModule = "1.3"; 
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_wifi_init(&cfg);
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_rx_cb(&wifiPacketHandler);
        Serial.println("[+] Module 1.3 Promiscuous Sniffer Active"); 
      }
      else if (incoming.indexOf("\"module\":\"1.4\"") != -1) { activeModule = "1.4"; Serial.println("[+] Module 1.4 Active"); }
      else if (incoming.indexOf("\"module\":\"1.5\"") != -1) { 
        activeModule = "1.5"; 
        WiFi.softAP("Cyberdeck_AP", "cyberpass123");
        Serial.println("[+] Module 1.5 AP Hosting Active"); 
      }
      else if (incoming.indexOf("\"module\":\"1.6\"") != -1) { activeModule = "1.6"; Serial.println("[+] Module 1.6 Active"); }
      
      // --- BLE MODULES ---
      else if (incoming.indexOf("\"module\":\"2.1\"") != -1) { activeModule = "2.1"; Serial.println("[+] Module 2.1 BLE Scan Active"); }
      else if (incoming.indexOf("\"module\":\"2.2\"") != -1) { activeModule = "2.2"; bleTargetFound = false; bleTargetAddr = ""; Serial.println("[+] Module 2.2 GATT Services Active"); }
      else if (incoming.indexOf("\"module\":\"2.3\"") != -1) { activeModule = "2.3"; Serial.println("[+] Module 2.3 BLE Sniffer Active"); }
      else if (incoming.indexOf("\"module\":\"2.4\"") != -1) { 
        activeModule = "2.4"; 
        if (pAdvertising) pAdvertising->start();
        Serial.println("[+] Module 2.4 Custom Beacon Active"); 
      }
      else if (incoming.indexOf("\"module\":\"2.5\"") != -1) { activeModule = "2.5"; bleTargetFound = false; bleTargetAddr = ""; Serial.println("[+] Module 2.5 GATT Exploration Active"); }
      else if (incoming.indexOf("\"module\":\"2.6\"") != -1) { activeModule = "2.6"; bleTargetFound = false; bleTargetAddr = ""; Serial.println("[+] Module 2.6 BLE Device Testing Active"); }

    } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
      if (activeModule == "1.3") {
        esp_wifi_set_promiscuous(false);
        esp_wifi_stop();
      } else if (activeModule == "1.5") {
        WiFi.softAPdisconnect(true);
      } else if (activeModule == "2.4" && pAdvertising) {
        pAdvertising->stop();
      }
      activeModule = "NONE";
      Serial.println("[-] Module Halted.");
    }
  }

  // ================= EXECUTION LOOPS =================

  // --- WIFI SUBSYSTEM ---
  if (activeModule == "1.1") {
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; ++i) {
      String ssid = WiFi.SSID(i);
      if (ssid == "") ssid = "<Hidden>";
      Serial.print("{\"ssid\":\"");
      Serial.print(ssid);
      Serial.print("\",\"bssid\":\"");
      Serial.print(WiFi.BSSIDstr(i));
      Serial.print("\",\"ch\":");
      Serial.print(WiFi.channel(i));
      Serial.print(",\"rssi\":");
      Serial.print(WiFi.RSSI(i));
      Serial.println("}");
      delay(20);
    }
    delay(2000);
  } 
  else if (activeModule == "1.2") {
    uint8_t primaryChannel;
    wifi_second_chan_t secondChannel;
    esp_wifi_get_channel(&primaryChannel, &secondChannel);
    
    Serial.print("{\"ssid\":\"Current_Channel_Monitor");
    Serial.print("\",\"bssid\":\"ESP32_S3_NODE\",\"ch\":");
    Serial.print(primaryChannel);
    Serial.print(",\"rssi\":");
    Serial.print(WiFi.RSSI());
    Serial.println("}");
    delay(1000);
  } 
  else if (activeModule == "1.3") {
    Serial.print("{\"ssid\":\"[RAW_PACKET_COUNT]\",\"bssid\":\"PROMISC_MODE\",\"ch\":");
    Serial.print(WiFi.channel());
    Serial.print(",\"rssi\":-");
    Serial.print(packetCount % 90);
    Serial.println("}");
    delay(1000);
  } 
  else if (activeModule == "1.4") {
    wifi_sta_list_t station;
    esp_wifi_ap_get_sta_list(&station);
    
    Serial.print("{\"ssid\":\"Connected_Clients: ");
    Serial.print(station.num);
    Serial.println("\",\"bssid\":\"AP_CLIENT_SCAN\",\"ch\":1,\"rssi\":-50}");
    delay(2000);
  } 
  else if (activeModule == "1.5") {
    IPAddress ip = WiFi.softAPIP();
    Serial.print("{\"ssid\":\"IP: ");
    Serial.print(ip.toString());
    Serial.println("\",\"bssid\":\"AP_ACTIVE_HOST\",\"ch\":1,\"rssi\":-10}");
    delay(3000);
  } 
  else if (activeModule == "1.6") {
    uint8_t packet[128];
    memset(packet, 0xFF, 32); 
    esp_err_t res = esp_wifi_80211_tx(WIFI_IF_STA, packet, sizeof(packet), true);
    
    Serial.print("{\"ssid\":\"Frame_Tx_Status: ");
    Serial.print(res == ESP_OK ? "SUCCESS" : "FAIL");
    Serial.println("\",\"bssid\":\"INJECTION_LAB\",\"ch\":");
    Serial.print(WiFi.channel());
    Serial.println(",\"rssi\":-25}");
    delay(2000);
  }

  // --- BLE SUBSYSTEM ---
  else if (activeModule == "2.1") {
    pBLEScan->start(3, false);
    pBLEScan->clearResults();
    delay(1000);
  }
  else if (activeModule == "2.2" && !bleTargetFound) {
    pBLEScan->start(3, false);
    pBLEScan->clearResults();
    delay(500);
  }
  else if (activeModule == "2.2" && bleTargetFound) {
    BLEClient* pClient = BLEDevice::createClient();
    if (pClient->connect(BLEAddress(bleTargetAddr.c_str()))) {
      std::map<std::string, BLERemoteService*>* pServices = pClient->getServices();
      if (pServices != nullptr) {
        for (auto const& [suuid, pService] : *pServices) {
          std::map<std::string, BLERemoteCharacteristic*>* pChars = pService->getCharacteristics();
          if (pChars != nullptr) {
            for (auto const& [cuuid, pChar] : *pChars) {
              Serial.print("{\"name\":\"");
              Serial.print(bleTargetAddr.c_str());
              Serial.print("\",\"uuid\":\"");
              Serial.print(pChar->getUUID().toString().c_str());
              Serial.print("\",\"rssi\":-50,\"payload\":\"Service: ");
              Serial.print(pService->getUUID().toString().c_str());
              Serial.println("\"}");
            }
          }
        }
      }
      pClient->disconnect();
    }
    bleTargetFound = false;
    delay(2000);
  }
  else if (activeModule == "2.3") {
    pBLEScan->start(2, false);
    pBLEScan->clearResults();
    delay(500);
  }
  else if (activeModule == "2.4") {
    Serial.println("{\"name\":\"Cyberdeck-Beacon\",\"uuid\":\"Broadcast Active\",\"rssi\":-10,\"payload\":\"BEACON_FLOOD\"}");
    delay(1500);
  }
  else if (activeModule == "2.5" && !bleTargetFound) {
    pBLEScan->start(3, false);
    pBLEScan->clearResults();
    delay(500);
  }
  else if (activeModule == "2.5" && bleTargetFound) {
    BLEClient* pClient = BLEDevice::createClient();
    if (pClient->connect(BLEAddress(bleTargetAddr.c_str()))) {
      std::map<std::string, BLERemoteService*>* pServices = pClient->getServices();
      if (pServices != nullptr) {
        for (auto const& [suuid, pService] : *pServices) {
          std::map<std::string, BLERemoteCharacteristic*>* pChars = pService->getCharacteristics();
          if (pChars != nullptr) {
            for (auto const& [cuuid, pChar] : *pChars) {
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
              Serial.print("{\"name\":\"");
              Serial.print(bleTargetAddr);
              Serial.print("\",\"uuid\":\"");
              Serial.print(pChar->getUUID().toString().c_str());
              Serial.print("\",\"rssi\":-50,\"payload\":\"");
              Serial.print(hexVal);
              Serial.println("\"}");
            }
          }
        }
      }
      pClient->disconnect();
    }
    bleTargetFound = false;
    delay(2000);
  }
  else if (activeModule == "2.6" && !bleTargetFound) {
    pBLEScan->start(3, false);
    pBLEScan->clearResults();
    delay(500);
  }
  else if (activeModule == "2.6" && bleTargetFound) {
    int attempts = 0;
    while (activeModule == "2.6" && attempts < 5) {
      attempts++;
      BLEClient* client = BLEDevice::createClient();
      unsigned long connStart = millis();
      if (client->connect(BLEAddress(bleTargetAddr.c_str()))) {
        unsigned long latency = millis() - connStart;
        Serial.print("{\"name\":\"");
        Serial.print(bleTargetAddr);
        Serial.print("\",\"uuid\":\"Attempt ");
        Serial.print(attempts);
        Serial.print("\",\"rssi\":-40,\"payload\":\"SUCCESS: ");
        Serial.print(latency);
        Serial.println("ms\"}");
        client->disconnect();
      } else {
        Serial.print("{\"name\":\"");
        Serial.print(bleTargetAddr);
        Serial.print("\",\"uuid\":\"Attempt ");
        Serial.print(attempts);
        Serial.print("\",\"rssi\":-99,\"payload\":\"FAILED\"}");
      }
      delay(1000);
    }
    bleTargetFound = false;
    activeModule = "NONE";
  }

  delay(100);
}