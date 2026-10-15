#include <Arduino.h>

const uint8_t IR_TX_PIN = 5;

void setup() {
  pinMode(IR_TX_PIN, OUTPUT);
}

void loop() {
  digitalWrite(IR_TX_PIN, HIGH);
  delayMicroseconds(26);
  digitalWrite(IR_TX_PIN, LOW);
  delay(10);
}