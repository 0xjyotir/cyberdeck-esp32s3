#include <Arduino.h>
#include <IRsend.h>

const uint8_t IR_TX_PIN = 5;
IRsend irsend(IR_TX_PIN);

void setup() {
  Serial.begin(115200);
  irsend.begin();
  irsend.sendNEC(0x00FF00FF, 32);
}

void loop() {
}