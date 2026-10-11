#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

void sniffer_callback(void* buf, wifi_promiscuous_pkt_type_t type) {
  const wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t*)buf;
  const uint8_t *payload = pkt->payload;
  uint16_t length = pkt->rx_ctrl.sig_len;

  if (type == WIFI_PKT_MGMT) {
    if (length > 24) {
      uint8_t frame_control = payload[0];
      uint8_t subtype = (frame_control & 0xF0) >> 4;
      
      if (subtype == 4) {
        Serial.print("[!] Probe Req | RSSI: ");
        Serial.print(pkt->rx_ctrl.rssi);
        Serial.print(" dBm | MAC: ");
        for (int i = 10; i <= 15; i++) {
          if (payload[i] < 16) Serial.print("0");
          Serial.print(payload[i], HEX);
          if (i < 15) Serial.print(":");
        }
        Serial.println();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) {
    delay(10);
  }

  Serial.println("\n[+] System Booted. Initializing Wi-Fi Promiscuous Sniffer...");

  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&sniffer_callback);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  Serial.println("[+] Sniffer active. Hopping channels and capturing raw frames...\n");
}

void loop() {
  static unsigned long last_hop = 0;
  if (millis() - last_hop > 500) {
    last_hop = millis();
    static uint8_t channel = 1;
    channel = (channel % 13) + 1;
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  }
}