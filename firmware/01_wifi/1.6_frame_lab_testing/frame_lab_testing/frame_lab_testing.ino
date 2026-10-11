#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

uint8_t packet[128] = {
  0x80, 0x00, 0x00, 0x00, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0x24, 0x6f, 0x28, 0xde, 0xad, 0xef, 
  0x24, 0x6f, 0x28, 0xde, 0xad, 0xef, 
  0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x64, 0x00, 
  0x01, 0x04, 
  0x00, 0x0c, 'L', 'a', 'b', '_', 'T', 'e', 's', 't', '_', 'N', 'o', 'd',
  0x01, 0x08, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
  0x03, 0x01, 0x01
};

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  Serial.println("\n[+] System Booted. Initializing Frame Lab Testing Module...");

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  Serial.println("[+] Frame injection subsystem ready. Broadcasting custom test frames...\n");
}

void loop() {
  esp_wifi_deauth_sta(0); 
  esp_wifi_80211_tx(WIFI_IF_STA, packet, sizeof(packet), false);
  
  Serial.println("[+] Injected test frame packet broadcasted on channel 1.");
  delay(1000);
}