#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  Serial.println("\n[+] System Booted. Initializing Network Discovery Module...");

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  Serial.println("[+] Wi-Fi subsystem ready. Running detailed AP attribute sweep...");
  
  Serial.println("\n--------------------------------------------------------------------------------");
  Serial.println("[-] Scanning and analyzing local wireless infrastructure...");

  int n = WiFi.scanNetworks(false, true);
  
  if (n == 0) {
    Serial.println("[-] No networks found.");
  } else {
    Serial.print("[+] Found ");
    Serial.print(n);
    Serial.print(" target networks. Compiling attributes:\n");
    Serial.println("--------------------------------------------------------------------------------");
    
    for (int i = 0; i < n; ++i) {
      Serial.print("  [");
      if (i + 1 < 10) Serial.print(" ");
      Serial.print(i + 1);
      Serial.print("] SSID: ");
      
      String ssid = WiFi.SSID(i);
      if (ssid.length() == 0) {
        Serial.print("<Hidden Network>");
      } else {
        Serial.print(ssid);
      }
      
      Serial.print("\n      | BSSID: ");
      Serial.print(WiFi.BSSIDstr(i));
      Serial.print(" | Ch: ");
      int ch = WiFi.channel(i);
      if (ch < 10) Serial.print(" ");
      Serial.print(ch);
      
      Serial.print(" | RSSI: ");
      int rssi = WiFi.RSSI(i);
      if (rssi > -60) Serial.print(" ");
      Serial.print(rssi);
      Serial.print(" dBm");
      
      Serial.print(" | Auth: ");
      wifi_auth_mode_t enc = WiFi.encryptionType(i);
      switch (enc) {
        case WIFI_AUTH_OPEN:         Serial.print("OPEN"); break;
        case WIFI_AUTH_WEP:          Serial.print("WEP"); break;
        case WIFI_AUTH_WPA_PSK:      Serial.print("WPA-PSK"); break;
        case WIFI_AUTH_WPA2_PSK:     Serial.print("WPA2-PSK"); break;
        case WIFI_AUTH_WPA_WPA2_PSK: Serial.print("WPA/WPA2"); break;
        case WIFI_AUTH_WPA3_PSK:     Serial.print("WPA3-PSK"); break;
        default:                     Serial.print("SECURED"); break;
      }
      Serial.println();
    }
    Serial.println("--------------------------------------------------------------------------------");
  }
  
  WiFi.scanDelete();
  Serial.println("\n[+] Network discovery complete. Halting execution.");
}

void loop() {
  
}