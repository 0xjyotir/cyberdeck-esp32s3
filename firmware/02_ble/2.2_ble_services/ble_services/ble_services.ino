#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>

bool gattActive = false;
BLEScan* pBLEScan;
String targetDeviceAddress = "";
bool deviceConnected = false;
bool targetFound = false;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.haveServiceUUID() || advertisedDevice.haveName()) {
            targetDeviceAddress = advertisedDevice.getAddress().toString().c_str();
            targetFound = true;
            Serial.print("[+] Target Acquired: ");
            Serial.println(targetDeviceAddress);
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
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    Serial.println("\n[+] ESP32-S3 GATT Enumerator Firmware Online.");
}

void loop() {
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        if (incoming.indexOf("\"action\":\"START\"") != -1 && incoming.indexOf("\"module\":\"2.2\"") != -1) {
            gattActive = true;
            targetFound = false;
            Serial.println("[+] Module 2.2 GATT Enumeration Started.");
        } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
            gattActive = false;
            targetFound = false;
            Serial.println("[-] Module Halted.");
        }
    }

    if (gattActive && !deviceConnected) {
        if (!targetFound) {
            Serial.println("[*] Scanning for target BLE peripheral...");
            pBLEScan->start(3, false);
            pBLEScan->clearResults();
            delay(1000);
        } else {
            Serial.print("[*] Connecting to target: ");
            Serial.println(targetDeviceAddress);
            
            BLEClient* pClient = BLEDevice::createClient();
            if (pClient->connect(BLEAddress(targetDeviceAddress))) {
                deviceConnected = true;
                Serial.println("[+] Connected! Enumerating services...");

                std::map<std::string, BLERemoteService*>* pServices = pClient->getServices();
                if (pServices != nullptr) {
                    for (auto const& [uuid, pService] : *pServices) {
                        Serial.print("{\"service\":\"");
                        Serial.print(pService->getUUID().toString().c_str());
                        Serial.println("\"}");

                        std::map<std::string, BLERemoteCharacteristic*>* pChars = pService->getCharacteristics();
                        if (pChars != nullptr) {
                            for (auto const& [charUuid, pChar] : *pChars) {
                                Serial.print("  {\"char\":\"");
                                Serial.print(pChar->getUUID().toString().c_str());
                                Serial.print("\",\"canRead\":");
                                Serial.print(pChar->canRead() ? "true" : "false");
                                Serial.print(",\"canWrite\":");
                                Serial.print(pChar->canWrite() ? "true" : "false");
                                Serial.println("}");
                            }
                        }
                    }
                }
                pClient->disconnect();
                Serial.println("[+] Enumeration complete. Disconnected.");
            } else {
                Serial.println("[-] Failed to connect to device.");
            }
            targetFound = false;
            delay(3000);
        }
    }

    delay(50);
}