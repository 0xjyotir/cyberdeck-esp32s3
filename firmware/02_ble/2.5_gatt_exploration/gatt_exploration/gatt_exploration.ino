#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>

bool gattExploreActive = false;
BLEScan* pBLEScan;
String targetAddress = "";
bool targetFound = false;

class ExploreCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        // Check if device is connectable and has a valid address
        if (device.isConnectable()) {
            targetAddress = device.getAddress().toString().c_str();
            targetFound = true;
            Serial.print("[+] Target acquired for GATT: ");
            Serial.println(targetAddress);
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
    pBLEScan->setAdvertisedDeviceCallbacks(new ExploreCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    Serial.println("\n[+] ESP32-S3 GATT Explorer Online.");
}

void loop() {
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        if (incoming.indexOf("\"action\":\"START\"") != -1 && incoming.indexOf("\"module\":\"2.5\"") != -1) {
            gattExploreActive = true;
            targetFound = false;
            Serial.println("[+] Module 2.5 GATT Exploration Started.");
        } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
            gattExploreActive = false;
            targetFound = false;
            Serial.println("[-] Module Halted.");
        }
    }

    if (gattExploreActive && !targetFound) {
        pBLEScan->start(3, false);
        pBLEScan->clearResults();
        delay(500);
    } else if (gattExploreActive && targetFound) {
        BLEClient* pClient = BLEDevice::createClient();
        
        // Set connection timeout or check if address is valid
        if (targetAddress.length() > 0) {
            Serial.print("[*] Attempting connection to: ");
            Serial.println(targetAddress);

            if (pClient->connect(BLEAddress(targetAddress.c_str()))) {
                Serial.println("[+] Connected! Enumerating GATT services...");
                
                std::map<std::string, BLERemoteService*>* pServices = pClient->getServices();
                if (pServices != nullptr) {
                    for (auto const& [suuid, pService] : *pServices) {
                        std::map<std::string, BLERemoteCharacteristic*>* pChars = pService->getCharacteristics();
                        if (pChars != nullptr) {
                            for (auto const& [cuuid, pChar] : *pChars) {
                                Serial.print("{\"service\":\"");
                                Serial.print(pService->getUUID().toString().c_str());
                                Serial.print("\",\"char\":\"");
                                Serial.print(pChar->getUUID().toString().c_str());
                                Serial.print("\",\"canRead\":");
                                Serial.print(pChar->canRead() ? "true" : "false");

                                if (pChar->canRead()) {
                                    String val = pChar->readValue();
                                    Serial.print(",\"value_hex\":\"");
                                    for (unsigned int i = 0; i < val.length(); i++) {
                                        char buf[3];
                                        sprintf(buf, "%02X", (uint8_t)val[i]);
                                        Serial.print(buf);
                                    }
                                    Serial.print("\"");
                                }
                                Serial.println("}");
                            }
                        }
                    }
                }
                pClient->disconnect();
                Serial.println("[+] GATT Exploration Complete.");
            } else {
                Serial.println("[-] Connection handshake failed or timed out.");
            }
        }

        gattExploreActive = false;
        targetFound = false;
        targetAddress = "";
        delay(2000);
    }

    delay(50);
}