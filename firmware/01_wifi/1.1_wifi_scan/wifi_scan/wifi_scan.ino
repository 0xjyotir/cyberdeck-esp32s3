#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  Serial.println("\n[+] System Booted. Initializing Wi-Fi module...");

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("[+] Wi-Fi subsystem ready. Starting scan loop.");
}

void loop() {
  Serial.println("\n----------------------------------------");
  Serial.println("[-] Scanning ambient Wi-Fi networks...");

  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    Serial.println("[-] No networks found.");
  } else {
    Serial.print("[+] Found ");
    Serial.print(n);
    Serial.println(" networks:");
    Serial.println("----------------------------------------");
    
    for (int i = 0; i < n; ++i) {
      Serial.print("  [");
      if (i + 1 < 10) Serial.print(" ");
      Serial.print(i + 1);
      Serial.print("] RSSI: ");
      
      int rssi = WiFi.RSSI(i);
      if (rssi > -60) Serial.print(" ");
      Serial.print(rssi);
      Serial.print(" dBm | Ch: ");
      
      int ch = WiFi.channel(i);
      if (ch < 10) Serial.print(" ");
      Serial.print(ch);
      Serial.print(" | SSID: ");
      Serial.println(WiFi.SSID(i));
    }
    Serial.println("----------------------------------------");
  }
  
  delay(5000);
}