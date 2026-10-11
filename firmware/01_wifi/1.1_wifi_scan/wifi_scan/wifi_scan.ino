#include <Arduino.h>
#include <WiFi.h>

bool scanActive = false;

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("\n[+] ESP32-S3 Cyberdeck Firmware Online.");
  Serial.println("[+] Ready for Web Serial dashboard commands.");
}

void loop() {
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();

    if (incoming.indexOf("\"action\":\"START\"") != -1) {
      if (incoming.indexOf("\"module\":\"1.1\"") != -1) {
        scanActive = true;
        Serial.println("[+] Module 1.1 Scan started via Web Dashboard.");
      }
    } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
      scanActive = false;
      Serial.println("[-] Module halted via Web Dashboard.");
    }
  }

  if (scanActive) {
    Serial.println("[*] Initiating Wi-Fi network scan...");
    int n = WiFi.scanNetworks(false, true);
    
    if (n == 0) {
      Serial.println("[-] No networks found.");
    } else {
      Serial.print("[+] Scan complete. Found ");
      Serial.print(n);
      Serial.println(" networks.");

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
        
        delay(10);
      }
    }
    
    WiFi.scanDelete();
    delay(2000);
  }
}