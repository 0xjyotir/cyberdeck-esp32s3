#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"

String activeModule = "NONE";
volatile unsigned long packetCount = 0;

void wifiPacketHandler(void* buf, wifi_promiscuous_pkt_type_t type) {
  packetCount++;
}

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("\n[+] ESP32-S3 Cyberdeck Hardware Firmware Online.");
}

void loop() {
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();

    if (incoming.indexOf("\"action\":\"START\"") != -1) {
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
    } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
      if (activeModule == "1.3") {
        esp_wifi_set_promiscuous(false);
        esp_wifi_stop();
      } else if (activeModule == "1.5") {
        WiFi.softAPdisconnect(true);
      }
      activeModule = "NONE";
      Serial.println("[-] Module Halted.");
    }
  }

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

  delay(100);
}