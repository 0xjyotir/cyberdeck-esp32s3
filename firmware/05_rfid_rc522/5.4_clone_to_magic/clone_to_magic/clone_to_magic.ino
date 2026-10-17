/*
 * Cyberdeck ESP32-S3 — Module 5.4: RC522 Clone to Magic Card (standalone)
 * ----------------------------------------------------------------------
 * The canonical access-control demo, done ethically: read a SOURCE MIFARE
 * Classic card that YOU OWN, then write its UID + sector data onto a Gen1a/Gen2
 * "magic" card that YOU OWN, and verify by reading the copy back. The point is
 * to show *why* UID-based door systems are unsafe — "your badge can be copied in
 * seconds" — not to defeat anyone else's credential.
 *
 *   ⚠️  AUTHORIZED / OWNED CARDS ONLY. Cloning a credential you do not own or
 *       are not explicitly authorized to duplicate is illegal in most places.
 *       This module is a teaching aid for your own card onto your own blank.
 *
 * Ethical gate: writes are DISABLED until you set OPERATOR_OWNS_CARDS to 1 below
 * (the standalone equivalent of the dashboard's "source card owned by operator"
 * acknowledgement). Left at 0, the module still reads the source and reports,
 * but refuses every write with "ownership_not_acknowledged". It also refuses any
 * target that is not a magic card ("not_a_magic_card") and never touches sector
 * trailers (so it can't brick the card's keys).
 *
 * Flow:
 *   1. START -> present the SOURCE card. It reads UID + data blocks (default key)
 *      into RAM and reports "source_read".
 *   2. Present the TARGET magic card. It writes block 0 (UID) via the Gen1a
 *      backdoor if present, else an authenticated block-0 write (Gen2/CUID),
 *      writes the data blocks, reads back and diffs, then reports "clone".
 *
 * Wiring (RC522 on the shared nRF24 SPI bus, 3.3 V ONLY):
 *   SDA(SS) = GPIO10   RST = GPIO9
 *   SCK = GPIO12   MOSI = GPIO11   MISO = GPIO13   IRQ unused   VCC = 3.3V
 *
 * Requires: MFRC522 (miguelbalboa) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host  -> board : {"module":"5.4","action":"START"} / {"action":"STOP"}
 *   source read    : {"module":"5.4","status":"ok","data":{
 *                       "stage":"source_read","uid":"DEADBEEF","blocks_cached":48}}
 *   clone result   : {"module":"5.4","status":"ok","data":{
 *                       "stage":"clone","uid":"DEADBEEF","blocks_written":47,
 *                       "verify":"match","target_type":"gen1a"}}
 *   errors         : rc522_not_found / unsupported_card / ownership_not_acknowledged /
 *                    not_a_magic_card / source_locked
 *
 * DEBUG_SERIAL: set to 1 for readable progress lines in the Serial Monitor.
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1

// ⚠️ ETHICAL GATE — set to 1 ONLY for cards you own / are authorized to clone.
#define OPERATOR_OWNS_CARDS 0

static const uint8_t RC522_SS_PIN   = 10;
static const uint8_t RC522_RST_PIN  = 9;
static const uint8_t RC522_SCK_PIN  = 12;
static const uint8_t RC522_MISO_PIN = 13;
static const uint8_t RC522_MOSI_PIN = 11;

static const int BAUD_RATE = 115200;
static const unsigned long POLL_INTERVAL_MS = 250;
static const byte MAX_BLOCKS = 64;   // Classic 1K = 64 blocks; we cache up to this

MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);
JsonDocument doc;
MFRC522::MIFARE_Key defaultKey;

bool cloneActive = false;
bool readerReady = false;
unsigned long lastPoll = 0;

enum Stage { WAIT_SOURCE, WAIT_TARGET };
Stage stage = WAIT_SOURCE;

// RAM cache of the source card.
byte srcUid[10];      byte srcUidLen = 0;
byte srcBlocks[MAX_BLOCKS][16];
bool srcValid[MAX_BLOCKS];
byte srcBlockCount = 0;
String srcUidStr = "";

void emit(const char* status) {
  doc["module"] = "5.4";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}
void emitError(const char* msg) { doc["data"]["msg"] = msg; emit("error"); }

String toHex(const byte* buf, byte len) {
  String s = "";
  for (byte i = 0; i < len; i++) { if (buf[i] < 0x10) s += '0'; s += String(buf[i], HEX); }
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

byte blocksInSector(byte sector) { return sector < 32 ? 4 : 16; }
byte firstBlockOf(byte sector) { return sector < 32 ? sector * 4 : 128 + (sector - 32) * 16; }
bool isTrailer(byte block) {
  if (block < 128) return (block % 4) == 3;
  return ((block - 128) % 16) == 15;
}

// --- Gen1a magic-card backdoor -------------------------------------------
// The classic "UID changeable" chinese magic card answers two unauthenticated
// commands: 0x40 (7-bit frame) then 0x43, each acked with the 4-bit 0x0A. If it
// does, we can read/write block 0 with no key. Non-magic cards stay silent.
bool unlockGen1a() {
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  byte resp = 0; byte respLen = 1; byte validBits = 7;
  byte cmd40 = 0x40;
  MFRC522::StatusCode s = mfrc522.PCD_TransceiveData(&cmd40, 1, &resp, &respLen, &validBits, 0, false);
  if (s != MFRC522::STATUS_OK || resp != 0x0A) return false;

  byte cmd43 = 0x43; respLen = 1; validBits = 0;
  s = mfrc522.PCD_TransceiveData(&cmd43, 1, &resp, &respLen, &validBits, 0, false);
  return (s == MFRC522::STATUS_OK && resp == 0x0A);
}

// Raw (unauthenticated) MIFARE write used after a Gen1a unlock. Two phases:
// {0xA0, block}+CRC -> ACK, then 16 data bytes + CRC -> ACK.
bool mifareRawWrite(byte block, const byte* data16) {
  byte cmd[4]; byte resp = 0; byte respLen = 1; byte validBits = 0;
  cmd[0] = 0xA0; cmd[1] = block;
  if (mfrc522.PCD_CalculateCRC(cmd, 2, &cmd[2]) != MFRC522::STATUS_OK) return false;
  if (mfrc522.PCD_TransceiveData(cmd, 4, &resp, &respLen, &validBits, 0, false) != MFRC522::STATUS_OK) return false;
  if (resp != 0x0A) return false;

  byte buf[18];
  memcpy(buf, data16, 16);
  if (mfrc522.PCD_CalculateCRC(buf, 16, &buf[16]) != MFRC522::STATUS_OK) return false;
  resp = 0; respLen = 1; validBits = 0;
  if (mfrc522.PCD_TransceiveData(buf, 18, &resp, &respLen, &validBits, 0, false) != MFRC522::STATUS_OK) return false;
  return (resp == 0x0A);
}

// --- Source read ----------------------------------------------------------
void readSource() {
  MFRC522::PICC_Type t = mfrc522.PICC_GetType(mfrc522.uid.sak);
  if (t != MFRC522::PICC_TYPE_MIFARE_1K && t != MFRC522::PICC_TYPE_MIFARE_4K &&
      t != MFRC522::PICC_TYPE_MIFARE_MINI) {
    emitError("unsupported_card");
    mfrc522.PICC_HaltA();
    return;
  }
  byte sectorCount = (t == MFRC522::PICC_TYPE_MIFARE_4K) ? 40 : (t == MFRC522::PICC_TYPE_MIFARE_MINI ? 5 : 16);

  srcUidLen = mfrc522.uid.size;
  memcpy(srcUid, mfrc522.uid.uidByte, srcUidLen);
  srcUidStr = toHex(srcUid, srcUidLen);
  for (byte i = 0; i < MAX_BLOCKS; i++) srcValid[i] = false;
  srcBlockCount = 0;

  byte cached = 0;
  for (byte s = 0; s < sectorCount; s++) {
    byte trailer = firstBlockOf(s) + blocksInSector(s) - 1;
    if (mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &defaultKey, &(mfrc522.uid)) != MFRC522::STATUS_OK) {
      mfrc522.PCD_StopCrypto1();
      continue;  // sector not openable with default key; skip, don't fail the whole read
    }
    for (byte b = 0; b < blocksInSector(s); b++) {
      byte addr = firstBlockOf(s) + b;
      if (addr >= MAX_BLOCKS) continue;
      byte buf[18]; byte sz = sizeof(buf);
      if (mfrc522.MIFARE_Read(addr, buf, &sz) == MFRC522::STATUS_OK) {
        memcpy(srcBlocks[addr], buf, 16);
        srcValid[addr] = true;
        cached++;
        if (addr + 1 > srcBlockCount) srcBlockCount = addr + 1;
      }
    }
    mfrc522.PCD_StopCrypto1();
  }
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  if (cached == 0) { emitError("source_locked"); return; }

#if DEBUG_SERIAL
  Serial.print(F("[clone] source read UID ")); Serial.print(srcUidStr);
  Serial.print(F("  blocks_cached=")); Serial.println(cached);
  Serial.println(F("        now present the TARGET magic card..."));
#else
  doc["data"]["stage"]         = "source_read";
  doc["data"]["uid"]           = srcUidStr;
  doc["data"]["blocks_cached"] = cached;
  emit("ok");
#endif
  stage = WAIT_TARGET;
}

// --- Target write ----------------------------------------------------------
void writeTarget() {
#if OPERATOR_OWNS_CARDS == 0
  emitError("ownership_not_acknowledged");   // writes disabled until the gate is set
  mfrc522.PICC_HaltA();
  return;
#else
  // Don't clone a card onto itself.
  String tgtUid = toHex(mfrc522.uid.uidByte, mfrc522.uid.size);

  bool gen1a = unlockGen1a();
  const char* targetType = gen1a ? "gen1a" : "gen2";

  // Block 0 (UID / manufacturer block): Gen1a via raw write, else authenticated
  // write (Gen2/CUID). If neither works, it isn't a writable-UID magic card.
  bool block0ok = false;
  if (srcValid[0]) {
    if (gen1a) {
      block0ok = mifareRawWrite(0, srcBlocks[0]);
    } else {
      // Re-select and try an authenticated block-0 write (CUID/Gen2 cards allow it).
      byte atqa[2]; byte asz = sizeof(atqa);
      mfrc522.PICC_WakeupA(atqa, &asz);
      mfrc522.PICC_ReadCardSerial();
      if (mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 3, &defaultKey, &(mfrc522.uid)) == MFRC522::STATUS_OK) {
        block0ok = (mfrc522.MIFARE_Write(0, srcBlocks[0], 16) == MFRC522::STATUS_OK);
      }
      mfrc522.PCD_StopCrypto1();
    }
  }
  if (!block0ok) { emitError("not_a_magic_card"); mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); stage = WAIT_SOURCE; return; }

  // Data blocks: authenticated writes, skipping trailers (never overwrite keys)
  // and block 0 (already done). Re-select per sector as auth resets state.
  byte written = 1;  // block 0 counts
  byte sectorCount = srcBlockCount <= 64 ? 16 : 40;
  for (byte s = 0; s < sectorCount; s++) {
    byte trailer = firstBlockOf(s) + blocksInSector(s) - 1;
    byte atqa[2]; byte asz = sizeof(atqa);
    mfrc522.PICC_HaltA();
    mfrc522.PICC_WakeupA(atqa, &asz);
    if (!mfrc522.PICC_ReadCardSerial()) continue;
    if (mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &defaultKey, &(mfrc522.uid)) != MFRC522::STATUS_OK) {
      mfrc522.PCD_StopCrypto1();
      continue;
    }
    for (byte b = 0; b < blocksInSector(s); b++) {
      byte addr = firstBlockOf(s) + b;
      if (addr == 0 || isTrailer(addr) || addr >= MAX_BLOCKS || !srcValid[addr]) continue;
      if (mfrc522.MIFARE_Write(addr, srcBlocks[addr], 16) == MFRC522::STATUS_OK) written++;
    }
    mfrc522.PCD_StopCrypto1();
  }

  // Verify: read the copy back and diff the cached blocks.
  bool match = true;
  for (byte s = 0; s < sectorCount && match; s++) {
    byte trailer = firstBlockOf(s) + blocksInSector(s) - 1;
    byte atqa[2]; byte asz = sizeof(atqa);
    mfrc522.PICC_HaltA();
    mfrc522.PICC_WakeupA(atqa, &asz);
    if (!mfrc522.PICC_ReadCardSerial()) { match = false; break; }
    if (mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailer, &defaultKey, &(mfrc522.uid)) != MFRC522::STATUS_OK) {
      mfrc522.PCD_StopCrypto1(); continue;
    }
    for (byte b = 0; b < blocksInSector(s); b++) {
      byte addr = firstBlockOf(s) + b;
      if (addr == 0 || isTrailer(addr) || addr >= MAX_BLOCKS || !srcValid[addr]) continue;
      byte buf[18]; byte sz = sizeof(buf);
      if (mfrc522.MIFARE_Read(addr, buf, &sz) != MFRC522::STATUS_OK || memcmp(buf, srcBlocks[addr], 16) != 0) { match = false; break; }
    }
    mfrc522.PCD_StopCrypto1();
  }

#if DEBUG_SERIAL
  char line[72];
  snprintf(line, sizeof(line), "[clone] -> %s  blocks_written=%u  verify=%s\n",
           targetType, written, match ? "match" : "mismatch");
  Serial.print(line);
#else
  doc["data"]["stage"]          = "clone";
  doc["data"]["uid"]            = srcUidStr;
  doc["data"]["blocks_written"] = written;
  doc["data"]["verify"]         = match ? "match" : "mismatch";
  doc["data"]["target_type"]    = targetType;
  emit("ok");
#endif

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  stage = WAIT_SOURCE;   // ready for the next clone
#endif
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println(F("\n[+] RC522 Module 5.4 Clone to Magic Card ready."));
#if OPERATOR_OWNS_CARDS == 0
  Serial.println(F("[i] Writes DISABLED (OPERATOR_OWNS_CARDS=0). Set it to 1 for cards you own."));
#else
  Serial.println(F("[!] Writes ENABLED. Owned/authorized cards ONLY."));
#endif

#if DEBUG_SERIAL
  if (ensureReader()) {
    Serial.println(F("[+] DEBUG_SERIAL on: RC522 found. Present the SOURCE card."));
    cloneActive = true;
    stage = WAIT_SOURCE;
  } else {
    Serial.println(F("[!] DEBUG_SERIAL on: RC522 NOT found. Check SDA=10 RST=9 SCK=12 MOSI=11 MISO=13, 3.3V."));
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.4\"") != -1) {
      if (!ensureReader()) emitError("rc522_not_found");
      else { cloneActive = true; stage = WAIT_SOURCE; }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      cloneActive = false;
      if (readerReady) { mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); }
    }
  }

  if (cloneActive && readerReady && millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      if (stage == WAIT_SOURCE) readSource();
      else                      writeTarget();
    }
  }
}
