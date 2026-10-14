#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 5;
BLEScan* pBLEScan;
bool bleScanActive = false;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        String name = advertisedDevice.haveName() ? advertisedDevice.getName().c_str() : "<Unnamed>";
        String addr = advertisedDevice.getAddress().toString().c_str();
        int rssi = advertisedDevice.getRSSI();
        
        Serial.print("{\"type\":\"ble\",\"name\":\"");
        Serial.print(name);
        Serial.print("\",\"addr\":\"");
        Serial.print(addr);
        Serial.print("\",\"rssi\":");
        Serial.print(rssi);
        Serial.println("}");
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

    Serial.println("\n[+] ESP32-S3 BLE Scanner Firmware Online.");
}

void loop() {
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();

        if (incoming.indexOf("\"action\":\"START\"") != -1 && incoming.indexOf("\"module\":\"2.1\"") != -1) {
            bleScanActive = true;
            Serial.println("[+] BLE Module 2.1 Scan Started.");
        } else if (incoming.indexOf("\"action\":\"STOP\"") != -1) {
            bleScanActive = false;
            Serial.println("[-] BLE Module Halted.");
        }
    }

    if (bleScanActive) {
        BLEScanResults* foundDevices = pBLEScan->start(scanTime, false);
        pBLEScan->clearResults();
        delay(1000);
    }

    delay(50);
}