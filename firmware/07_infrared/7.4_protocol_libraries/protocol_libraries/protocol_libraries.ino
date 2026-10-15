#include <Arduino.h>
#include <IRrecv.h>
#include <IRutils.h>

const uint16_t IR_RX_PIN = 4;
const uint16_t BUFFER_SIZE = 1024;
const uint8_t TIMEOUT_VAL = 15;

IRrecv irrecv(IR_RX_PIN, BUFFER_SIZE, TIMEOUT_VAL, true);
decode_results results;

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.print("Protocol: ");
    Serial.print(typeToString(results.decode_type));
    Serial.print(" | Value: ");
    Serial.println(uint64ToString(results.value, 16));
    irrecv.resume();
  }
}