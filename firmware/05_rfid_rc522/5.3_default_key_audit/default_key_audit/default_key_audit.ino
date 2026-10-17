/*
 * Cyberdeck ESP32-S3 — Module 5.3: RC522 Default-Key Security Audit (standalone)
 * -----------------------------------------------------------------------------
 * The standout RFID module. Runs a presented MIFARE Classic card against a
 * dictionary of common / factory / transport keys and reports, per sector,
 * whether it fell to a default key or held. Produces an overall security grade
 * (A–F) for the card.
 *
 * This is a real pentest deliverable, not a toy: it answers "would this badge
 * survive a 10-second attack?". Most cheap or legacy MIFARE Classic deployments
 * fail here — which is the lesson. It is PURE READ / AUTH: nothing is written,
 * no data is dumped, only which key opened each sector. Authorized / educational
 * use only — audit cards you own or are permitted to test.
 *
 * (Sectors that resist the whole dictionary are reported "protected". Actually
 * recovering those keys needs a Crypto1 nested/darkside attack — out of scope
 * here, noted as mk2 stretch in the roadmap.)
 *
 * Wiring (RC522 on the shared nRF24 SPI bus, 3.3 V ONLY):
 *   SDA(SS) = GPIO10   RST = GPIO9
 *   SCK = GPIO12   MOSI = GPIO11   MISO = GPIO13   IRQ unused   VCC = 3.3V
 *
 * Requires: MFRC522 (miguelbalboa) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host  -> board : {"module":"5.3","action":"START"} / {"action":"STOP"}
 *   per sector     : {"module":"5.3","status":"ok","data":{
 *                       "sector":0,"result":"default_key","key_found":"FFFFFFFFFFFF"}}
 *   summary        : {"module":"5.3","status":"ok","data":{
 *                       "weak_sectors":14,"total_sectors":16,"grade":"F","done":true}}
 *   errors         : rc522_not_found / unsupported_card
 *
 * DEBUG_SERIAL: set to 1 for a readable per-sector report + grade in the Serial
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

// Common / factory / transport keys seen in the wild. Add your own as needed.
const byte KEY_DICT[][6] = {
  {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},  // blank / factory default
  {0xA0,0xA1,0xA2,0xA3,0xA4,0xA5},  // MAD / NDEF default key A
  {0xD3,0xF7,0xD3,0xF7,0xD3,0xF7},  // NDEF public key B
  {0x00,0x00,0x00,0x00,0x00,0x00},  // all-zero
  {0xB0,0xB1,0xB2,0xB3,0xB4,0xB5},  // common infra transport key
  {0x4D,0x3A,0x99,0xC3,0x51,0xDD},  // common transit key
  {0x1A,0x98,0x2C,0x7E,0x45,0x9A},  // common transit key
  {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF},  // lazy demo key
  {0x12,0x34,0x56,0x78,0x9A,0xBC},  // lazy demo key
};
static const byte KEY_DICT_LEN = sizeof(KEY_DICT) / 6;

MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);
JsonDocument doc;

bool auditActive = false;
bool readerReady = false;
unsigned long lastPoll = 0;

void emit(const char* status) {
  doc["module"] = "5.3";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

void emitError(const char* msg) {
  doc["data"]["msg"] = msg;
  emit("error");
}

String keyHex(const byte* k) {
  String s = "";
  for (byte i = 0; i < 6; i++) { if (k[i] < 0x10) s += '0'; s += String(k[i], HEX); }
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
  readerReady = true;
  return true;
}

byte blocksInSector(byte sector) { return sector < 32 ? 4 : 16; }
byte firstBlockOf(byte sector) {
  if (sector < 32) return sector * 4;
  return 128 + (sector - 32) * 16;
}

// Try every dictionary key (A then B) on a sector trailer. Returns the index of
// the key that worked, or -1 if the sector held. The card must be re-selected
// between failed auths, so we halt+wake around each try.
int crackSector(byte sector, byte* uidByte, byte uidSize, byte sak) {
  byte trailer = firstBlockOf(sector) + blocksInSector(sector) - 1;
  MFRC522::MIFARE_Key key;

  for (byte k = 0; k < KEY_DICT_LEN; k++) {
    for (byte i = 0; i < 6; i++) key.keyByte[i] = KEY_DICT[k][i];

    for (byte which = 0; which < 2; which++) {
      byte authCmd = which == 0 ? MFRC522::PICC_CMD_MF_AUTH_KEY_A
                                : MFRC522::PICC_CMD_MF_AUTH_KEY_B;
      MFRC522::StatusCode st = mfrc522.PCD_Authenticate(authCmd, trailer, &key, &(mfrc522.uid));
      mfrc522.PCD_StopCrypto1();
      if (st == MFRC522::STATUS_OK) return k;

      // A failed Crypto1 auth drops the card out of ACTIVE; re-select it.
      mfrc522.PICC_HaltA();
      byte atqa[2]; byte asz = sizeof(atqa);
      mfrc522.PICC_WakeupA(atqa, &asz);
      mfrc522.PICC_ReadCardSerial();
    }
  }
  return -1;
}

char gradeFor(byte weak, byte total) {
  if (total == 0) return '?';
  int pct = (int)((long)weak * 100 / total);
  if (weak == 0) return 'A';
  if (pct <= 12) return 'B';
  if (pct <= 37) return 'C';
  if (pct <= 62) return 'D';
  if (pct < 100) return 'E';
  return 'F';
}

void auditCard() {
  MFRC522::PICC_Type t = mfrc522.PICC_GetType(mfrc522.uid.sak);
  byte sectorCount;
  switch (t) {
    case MFRC522::PICC_TYPE_MIFARE_1K:   sectorCount = 16; break;
    case MFRC522::PICC_TYPE_MIFARE_4K:   sectorCount = 40; break;
    case MFRC522::PICC_TYPE_MIFARE_MINI: sectorCount = 5;  break;
    default:
      emitError("unsupported_card");  // only Classic has Crypto1 sectors to audit
      mfrc522.PICC_HaltA();
      return;
  }

  byte uidByte[10]; byte uidSize = mfrc522.uid.size; byte sak = mfrc522.uid.sak;
  memcpy(uidByte, mfrc522.uid.uidByte, uidSize);

  byte weak = 0;
#if DEBUG_SERIAL
  Serial.println(F("[audit] sector results:"));
#endif
  for (byte s = 0; s < sectorCount; s++) {
    int k = crackSector(s, uidByte, uidSize, sak);
    bool isWeak = (k >= 0);
    if (isWeak) weak++;

#if DEBUG_SERIAL
    char head[20];
    snprintf(head, sizeof(head), "  s%02u  ", s);
    Serial.print(head);
    if (isWeak) { Serial.print(F("DEFAULT KEY  ")); Serial.println(keyHex(KEY_DICT[k])); }
    else        { Serial.println(F("protected")); }
#else
    doc["data"]["sector"]    = s;
    doc["data"]["result"]    = isWeak ? "default_key" : "protected";
    if (isWeak) doc["data"]["key_found"] = keyHex(KEY_DICT[k]);
    emit("ok");
#endif
  }

  char grade = gradeFor(weak, sectorCount);
#if DEBUG_SERIAL
  char sum[64];
  snprintf(sum, sizeof(sum), "[audit] weak=%u/%u  grade=%c\n", weak, sectorCount, grade);
  Serial.print(sum);
#else
  doc["data"]["weak_sectors"]  = weak;
  doc["data"]["total_sectors"] = sectorCount;
  char g[2] = { grade, 0 };
  doc["data"]["grade"]         = g;
  doc["data"]["done"]          = true;
  emit("ok");
#endif

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println(F("\n[+] RC522 Module 5.3 Default-Key Security Audit ready."));

#if DEBUG_SERIAL
  if (ensureReader()) {
    Serial.println(F("[+] DEBUG_SERIAL on: RC522 found. Present a MIFARE Classic card to audit."));
    auditActive = true;
  } else {
    Serial.println(F("[!] DEBUG_SERIAL on: RC522 NOT found. Check SDA=10 RST=9 SCK=12 MOSI=11 MISO=13, 3.3V."));
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.3\"") != -1) {
      if (!ensureReader()) emitError("rc522_not_found");
      else auditActive = true;
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      auditActive = false;
      if (readerReady) { mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); }
    }
  }

  if (auditActive && readerReady && millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      auditCard();
    }
  }
}
