#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

bool testActive = false;
BLEScan* pBLEScan;
String targetAddr = "";
bool targetFound = false;

class TestCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        if (device.haveServiceUUID() || device.haveName()) {
            targetAddr = device.getAddress().toString().c_str();
            targetFound = true;
            pBLEScan->stop();
        }
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
    pBLEScan->setAdvertisedDeviceCallbacks(new TestCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    Serial.println("\n[+] ESP32-S3 BLE Device Testing Module Online.");
}

void loop() {
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        if (incoming.indexOf("\"action\":\"START\"") != -1 && incoming.indexOf("\"module\":\"2.6\"") != -1) {
            testActive = true;
            targetFound = false;
            Serial.println("[+] Module 2.6 Device Testing Started.");
        } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
            testActive = false;
            targetFound = false;
            Serial.println("[-] Module Halted.");
        }
    }

    if (testActive && !targetFound) {
        pBLEScan->start(3, false);
        pBLEScan->clearResults();
        delay(500);
    } else if (testActive && targetFound) {
        int attempts = 0;
        int successes = 0;

        while (testActive && attempts < 5) {
            attempts++;
            BLEClient* client = BLEDevice::createClient();
            unsigned long connStart = millis();
            
            if (client->connect(BLEAddress(targetAddr.c_str()))) {
                unsigned long latency = millis() - connStart;
                successes++;
                Serial.print("{\"type\":\"test\",\"addr\":\"");
                Serial.print(targetAddr);
                Serial.print("\",\"attempt\":");
                Serial.print(attempts);
                Serial.print(",\"status\":\"SUCCESS\",\"latency_ms\":");
                Serial.print(latency);
                Serial.println("}");
                client->disconnect();
            } else {
                Serial.print("{\"type\":\"test\",\"addr\":\"");
                Serial.print(targetAddr);
                Serial.print("\",\"attempt\":");
                Serial.print(attempts);
                Serial.println(",\"status\":\"FAILED\"}");
            }
            delay(1000);
        }

        Serial.println("[+] BLE Device Testing Sequence Complete.");
        testActive = false;
        targetFound = false;
    }

    delay(50);
}