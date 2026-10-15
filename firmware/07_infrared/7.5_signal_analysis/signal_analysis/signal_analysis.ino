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
    uint32_t markTime = 0;
    uint32_t spaceTime = 0;
    for (uint16_t i = 1; i < results.rawlen; i++) {
      if (i % 2 != 0) {
        markTime += results.rawbuf[i] * kRawTick;
      } else {
        spaceTime += results.rawbuf[i] * kRawTick;
      }
    }
    Serial.print("Mark: ");
    Serial.print(markTime);
    Serial.print(" us | Space: ");
    Serial.print(spaceTime);
    Serial.println(" us");
    irrecv.resume();
  }
}