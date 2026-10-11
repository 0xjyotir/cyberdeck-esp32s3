#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

const char* ssid = "Cyberdeck_AP";
const char* password = "cyberdeck_secure";

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  Serial.println("\n[+] System Booted. Initializing Advanced AP Station Info Module...");

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_AP);
  
  bool result = WiFi.softAP(ssid, password);
  if (result) {
    Serial.println("[+] Access Point successfully started!");
    Serial.print("[+] SSID: ");
    Serial.println(ssid);
    Serial.print("[+] Gateway IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("[+] Monitoring connected station telemetry...\n");
  } else {
    Serial.println("[-] Failed to start Access Point.");
  }
}

void loop() {
  delay(3000);

  wifi_sta_list_t station_list;
  esp_wifi_ap_get_sta_list(&station_list);

  Serial.println("--------------------------------------------------");
  Serial.print("[*] Active Connected Stations Count: ");
  Serial.println(station_list.num);

  if (station_list.num > 0) {
    Serial.println("--------------------------------------------------");
    Serial.println("  No. | MAC Address       | RSSI");
    Serial.println("--------------------------------------------------");
    
    for (int i = 0; i < station_list.num; i++) {
      wifi_sta_info_t station = station_list.sta[i];
      
      Serial.print("  [");
      Serial.print(i + 1);
      Serial.print("] | ");
      
      for (int j = 0; j < 6; j++) {
        if (station.mac[j] < 16) Serial.print("0");
        Serial.print(station.mac[j], HEX);
        if (j < 5) Serial.print(":");
      }
      
      Serial.print(" | ");
      Serial.print(station.rssi);
      Serial.println(" dBm");
    }
    Serial.println("--------------------------------------------------");
  }
  Serial.println();
}