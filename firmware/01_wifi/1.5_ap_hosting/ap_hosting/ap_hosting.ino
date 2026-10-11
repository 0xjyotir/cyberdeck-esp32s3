#include <Arduino.h>
#include <WiFi.h>

const char* ssid = "Cyberdeck_AP";
const char* password = "cyberdeck_secure";

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  Serial.println("\n[+] System Booted. Initializing AP Hosting Module...");

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_AP);
  
  bool result = WiFi.softAP(ssid, password);
  if (result) {
    Serial.println("[+] Access Point successfully started!");
    Serial.print("[+] SSID: ");
    Serial.println(ssid);
    Serial.print("[+] Gateway IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("[+] Ready for client connections...\n");
  } else {
    Serial.println("[-] Failed to start Access Point.");
  }
}

void loop() {
  delay(3000);
  int stations = WiFi.softAPgetStationNum();
  Serial.print("[*] Active connected stations: ");
  Serial.println(stations);
}