#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  Serial.println("\n[+] System Booted. Initializing Cyberdeck Wardriving Module...");

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("[+] Wi-Fi subsystem ready. Running initial sweep...");
  
  Serial.println("\n------------------------------------------------------------------");
  Serial.println("[-] Scanning ambient Wi-Fi networks...");

  int n = WiFi.scanNetworks(false, true);
  
  if (n == 0) {
    Serial.println("[-] No networks found.");
  } else {
    Serial.print("[+] Found ");
    Serial.print(n);
    Serial.print(" APs. Dumping telemetry:\n");
    Serial.println("------------------------------------------------------------------");
    
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
      
      Serial.print(" | Enc: ");
      wifi_auth_mode_t enc = WiFi.encryptionType(i);
      if (enc == WIFI_AUTH_OPEN) {
        Serial.print("OPEN ");
      } else {
        Serial.print("SEC  ");
      }
      
      Serial.print("| BSSID: ");
      Serial.print(WiFi.BSSIDstr(i));
      Serial.print(" | SSID: ");
      Serial.println(WiFi.SSID(i));
    }
    Serial.println("------------------------------------------------------------------");
  }
  
  WiFi.scanDelete();
  Serial.println("\n[+] Wardriving capture complete. Halting execution.");
}

void loop() {
  
}