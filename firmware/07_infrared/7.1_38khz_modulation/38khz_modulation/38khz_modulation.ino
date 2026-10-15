#include <Arduino.h>

const uint8_t IR_TX_PIN = 5;

void setup() {
  ledcAttach(IR_TX_PIN, 38000, 8);
  ledcWrite(IR_TX_PIN, 127);
}
void loop() {
}