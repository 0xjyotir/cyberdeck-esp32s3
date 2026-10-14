#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 0;
BLEScan* pBLEScan;
bool snifferActive = false;

class SnifferCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        String addr = device.getAddress().toString().c_str();
        int rssi = device.getRSSI();
        String payload = "";

        if (device.getPayloadLength() > 0) {
            uint8_t* pay = device.getPayload();
            size_t len = device.getPayloadLength();
            for (size_t i = 0; i < len; i++) {
                char buf[3];
                sprintf(buf, "%02X", pay[i]);
                payload += buf;
            }
        } else {
            payload = "None";
        }

        Serial.print("{\"type\":\"sniff\",\"addr\":\"");
        Serial.print(addr);
        Serial.print("\",\"rssi\":");
        Serial.print(rssi);
        Serial.print(",\"payload\":\"");
        Serial.print(payload);
        Serial.println("\"}");
    }
};

void setup() {
    Serial.begin(115200);
    unsigned long start = millis();
    while (!Serial && (millis() - start < 4000)) {
        delay(10);
    }

    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new SnifferCallbacks());
    pBLEScan->setActiveScan(false);
    pBLEScan->setInterval(50);
    pBLEScan->setWindow(25);

    Serial.println("\n[+] ESP32-S3 BLE Advertisement Sniffer Online.");
}

void loop() {
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        if (incoming.indexOf("\"action\":\"START\"") != -1 && incoming.indexOf("\"module\":\"2.3\"") != -1) {
            snifferActive = true;
            Serial.println("[+] Module 2.3 Sniffer Started.");
        } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
            snifferActive = false;
            pBLEScan->stop();
            Serial.println("[-] Module Halted.");
        }
    }

    if (snifferActive) {
        pBLEScan->start(1, false);
        pBLEScan->clearResults();
    }

    delay(20);
}