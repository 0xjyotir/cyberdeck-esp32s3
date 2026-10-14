#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEAdvertising.h>

bool spammerActive = false;
BLEAdvertising* pAdvertising;

void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 4000)) {
        delay(10);
    }

    BLEDevice::init("Cyberdeck-Spam");
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);

    Serial.println("\n[+] ESP32-S3 Beacon Spammer Online.");
}

void loop() {
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        if (incoming.indexOf("\"action\":\"START\"") != -1 && incoming.indexOf("\"module\":\"2.4\"") != -1) {
            spammerActive = true;
            pAdvertising->start();
            Serial.println("[+] Module 2.4 Beacon Spammer Started.");
        } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
            spammerActive = false;
            pAdvertising->stop();
            Serial.println("[-] Module Halted.");
        }
    }

    delay(50);
}