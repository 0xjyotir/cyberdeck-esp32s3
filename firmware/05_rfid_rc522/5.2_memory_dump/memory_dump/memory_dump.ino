/*
 * Cyberdeck ESP32-S3 — Module 5.2: RC522 Memory Dump / Reader (standalone)
 * -----------------------------------------------------------------------
 * Reads out the actual contents of a presented card. For a MIFARE Classic it
 * authenticates each sector with the factory default key (FFFFFFFFFFFF) and
 * dumps every readable block as hex; for Ultralight / NTAG it reads the pages
 * directly (no auth). One pass per card.
 *
 * Teaching point: "the card is just an ID" is a myth. This shows how much real
 * data (names, balances, flags, access bits) lives on a cheap card, and how the
 * sector trailer's access bits gate it. Blocks that refuse the default key show
 * up as locked — a hint that something there is actually protected.
 *
 * This module only READS. It never writes. Authorized / educational use only —
 * dump cards you own or are permitted to test.
 *
 * Wiring (RC522 on the shared nRF24 SPI bus, 3.3 V ONLY):
 *   SDA(SS) = GPIO10   RST = GPIO9
 *   SCK = GPIO12   MOSI = GPIO11   MISO = GPIO13   IRQ unused   VCC = 3.3V
 *
 * Requires: MFRC522 (miguelbalboa) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host  -> board : {"module":"5.2","action":"START"} / {"action":"STOP"}
 *   per block      : {"module":"5.2","status":"ok","data":{
 *                       "sector":1,"block":4,"data":"00112233...","key_used":"FFFFFFFFFFFF",
 *                       "auth":"ok"}}
 *   summary        : {"module":"5.2","status":"ok","data":{
 *                       "blocks_read":48,"blocks_locked":16,"done":true}}
 *   errors         : rc522_not_found / unsupported_card
 *
 * DEBUG_SERIAL: set to 1 to print a readable per-block hex dump in the Serial
 *   Monitor instead of JSON. START/STOP JSON still works.
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1

static const uint8_t RC522_SS_PIN   = 10;
static const uint8_t RC522_RST_PIN  = 9;
static const uint8_t RC522_SCK_PIN  = 12;
static const uint8_t RC522_MISO_PIN = 13;
static const uint8_t RC522_MOSI_PIN = 11;

static const int BAUD_RATE = 115200;
static const unsigned long POLL_INTERVAL_MS = 200;

MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);
JsonDocument doc;

bool dumpActive = false;
bool readerReady = false;
unsigned long lastPoll = 0;

// Factory default MIFARE key.
MFRC522::MIFARE_Key defaultKey;

void emit(const char* status) {
  doc["module"] = "5.2";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

void emitError(const char* msg) {
  doc["data"]["msg"] = msg;
  emit("error");
}

String toHex(const byte* buf, byte len) {
  String s = "";
  for (byte i = 0; i < len; i++) {
    if (buf[i] < 0x10) s += '0';
    s += String(buf[i], HEX);
  }
  s.toUpperCase();
  return s;
}

bool ensureReader() {
  if (readerReady) return true;
  SPI.begin(RC522_SCK_PIN, RC522_MISO_PIN, RC522_MOSI_PIN, RC522_SS_PIN);
  mfrc522.PCD_Init();
  delay(50);
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  if (v == 0x00 || v == 0xFF) return false;
  mfrc522.PCD_AntennaOn();
  for (byte i = 0; i < 6; i++) defaultKey.keyByte[i] = 0xFF;
  readerReady = true;
  return true;
}

// Emit one block's worth of data (or a locked marker).
void reportBlock(byte sector, byte block, const byte* data, bool authOk) {
#if DEBUG_SERIAL
  char head[24];
  snprintf(head, sizeof(head), "  s%02u b%03u ", sector, block);
  Serial.print(head);
  if (authOk) {
    for (byte i = 0; i < 16; i++) { if (data[i] < 0x10) Serial.print('0'); Serial.print(data[i], HEX); Serial.print(' '); }
    Serial.println();
  } else {
    Serial.println(F("<locked / auth failed with default key>"));
  }
#else
  doc["data"]["sector"]   = sector;
  doc["data"]["block"]    = block;
  doc["data"]["auth"]     = authOk ? "ok" : "fail";
  if (authOk) {
    doc["data"]["data"]     = toHex(data, 16);
    doc["data"]["key_used"] = "FFFFFFFFFFFF";
  }
  emit("ok");
#endif
}

// Blocks-per-sector differs on 4K: sectors 0-31 have 4, sectors 32-39 have 16.
byte blocksInSector(byte sector) { return sector < 32 ? 4 : 16; }
// Absolute address of the first block of a sector.
byte firstBlockOf(byte sector) {
  if (sector < 32) return sector * 4;
  return 128 + (sector - 32) * 16;
}

void dumpClassic(byte sectorCount) {
  uint16_t read = 0, locked = 0;
  for (byte s = 0; s < sectorCount; s++) {
    byte trailer = firstBlockOf(s) + blocksInSector(s) - 1;
    MFRC522::StatusCode st = mfrc522.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &defaultKey, &(mfrc522.uid));
    bool sectorAuthed = (st == MFRC522::STATUS_OK);

    for (byte b = 0; b < blocksInSector(s); b++) {
      byte addr = firstBlockOf(s) + b;
      byte buf[18];
      byte sz = sizeof(buf);
      bool ok = false;
      if (sectorAuthed) {
        if (mfrc522.MIFARE_Read(addr, buf, &sz) == MFRC522::STATUS_OK) ok = true;
      }
      if (ok) read++; else locked++;
      reportBlock(s, addr, buf, ok);
    }
    mfrc522.PCD_StopCrypto1();  // release crypto before the next sector's auth
  }

#if DEBUG_SERIAL
  char sum[48];
  snprintf(sum, sizeof(sum), "[dump] blocks_read=%u blocks_locked=%u\n", read, locked);
  Serial.print(sum);
#else
  doc["data"]["blocks_read"]   = read;
  doc["data"]["blocks_locked"] = locked;
  doc["data"]["done"]          = true;
  emit("ok");
#endif
}

void dumpUltralight() {
  // Ultralight / NTAG: 4-byte pages, no auth. MIFARE_Read returns 4 pages (16B)
  // at a time. We read up to 48 pages (covers NTAG216); reads past the end just
  // start failing, which ends the loop cleanly.
  uint16_t read = 0, locked = 0;
  for (byte page = 0; page < 48; page += 4) {
    byte buf[18];
    byte sz = sizeof(buf);
    if (mfrc522.MIFARE_Read(page, buf, &sz) == MFRC522::STATUS_OK) {
      for (byte p = 0; p < 4; p++) {
        read++;
        // Each page is 4 bytes; present it padded to the common 16-wide block view.
        byte pageBuf[16] = {0};
        memcpy(pageBuf, buf + p * 4, 4);
        reportBlock(0, page + p, pageBuf, true);
      }
    } else {
      break;  // end of memory
    }
  }
#if DEBUG_SERIAL
  char sum[48];
  snprintf(sum, sizeof(sum), "[dump] pages_read=%u\n", read);
  Serial.print(sum);
#else
  doc["data"]["blocks_read"]   = read;
  doc["data"]["blocks_locked"] = locked;
  doc["data"]["done"]          = true;
  emit("ok");
#endif
}

void dumpCard() {
  MFRC522::PICC_Type t = mfrc522.PICC_GetType(mfrc522.uid.sak);
  switch (t) {
    case MFRC522::PICC_TYPE_MIFARE_1K:  dumpClassic(16); break;
    case MFRC522::PICC_TYPE_MIFARE_4K:  dumpClassic(40); break;
    case MFRC522::PICC_TYPE_MIFARE_MINI: dumpClassic(5); break;
    case MFRC522::PICC_TYPE_MIFARE_UL:  dumpUltralight(); break;
    default:
      emitError("unsupported_card");  // DESFire / 14443A-4: UID only, no open dump
      break;
  }
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println(F("\n[+] RC522 Module 5.2 Memory Dump ready."));

#if DEBUG_SERIAL
  if (ensureReader()) {
    Serial.println(F("[+] DEBUG_SERIAL on: RC522 found. Present a card to dump its memory."));
    dumpActive = true;
  } else {
    Serial.println(F("[!] DEBUG_SERIAL on: RC522 NOT found. Check SDA=10 RST=9 SCK=12 MOSI=11 MISO=13, 3.3V."));
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.2\"") != -1) {
      if (!ensureReader()) emitError("rc522_not_found");
      else dumpActive = true;
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      dumpActive = false;
      if (readerReady) { mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); }
    }
  }

  if (dumpActive && readerReady && millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      dumpCard();
    }
  }
}
