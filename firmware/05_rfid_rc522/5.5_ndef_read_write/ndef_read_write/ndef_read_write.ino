/*
 * Cyberdeck ESP32-S3 — Module 5.5: RC522 NDEF Read / Write (standalone)
 * --------------------------------------------------------------------
 * Reads and (optionally) writes NDEF records on writable NFC tags
 * (NTAG213/215/216 and MIFARE Ultralight). Read decodes the first URI or Text
 * record on the tag; write provisions a tag with a single well-known URI record.
 *
 * Two teaching angles:
 *   - Defensive awareness: a tag that silently opens a URL is a phishing / "don't
 *     tap random NFC tags" demo.
 *   - Practical: provisioning your own tags for the deck.
 * Low-risk and demo-friendly. Authorized / educational use only — write tags you
 * own.
 *
 * Mode is a compile-time switch (writing is off by default):
 *   NDEF_MODE_WRITE 0 -> READ/decode the presented tag (default, read-only)
 *   NDEF_MODE_WRITE 1 -> WRITE the configured NDEF_URL to the presented tag
 * NDEF_URL is the https:// URL written in write mode (prefix-compressed per the
 * NFC Forum URI record type).
 *
 * Wiring (RC522 on the shared nRF24 SPI bus, 3.3 V ONLY):
 *   SDA(SS) = GPIO10   RST = GPIO9
 *   SCK = GPIO12   MOSI = GPIO11   MISO = GPIO13   IRQ unused   VCC = 3.3V
 *
 * Requires: MFRC522 (miguelbalboa) + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host  -> board : {"module":"5.5","action":"START"} / {"action":"STOP"}
 *   read result    : {"module":"5.5","status":"ok","data":{
 *                       "record_type":"uri","payload":"https://example.org",
 *                       "tag_type":"MIFARE Ultralight","capacity":144}}
 *   write result   : {"module":"5.5","status":"ok","data":{
 *                       "record_type":"uri","bytes_written":24,"verify":"match"}}
 *   errors         : rc522_not_found / unsupported_card / no_ndef / write_failed
 *
 * DEBUG_SERIAL: set to 1 for readable lines in the Serial Monitor.
 */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1

#define NDEF_MODE_WRITE 0                 // 0 = read/decode, 1 = write NDEF_URL
#define NDEF_URL "https://github.com/RRonium/cyberdeck-esp32s3"

static const uint8_t RC522_SS_PIN   = 10;
static const uint8_t RC522_RST_PIN  = 9;
static const uint8_t RC522_SCK_PIN  = 12;
static const uint8_t RC522_MISO_PIN = 13;
static const uint8_t RC522_MOSI_PIN = 11;

static const int BAUD_RATE = 115200;
static const unsigned long POLL_INTERVAL_MS = 250;
static const byte USER_START_PAGE = 4;    // NTAG/UL user memory begins at page 4

MFRC522 mfrc522(RC522_SS_PIN, RC522_RST_PIN);
JsonDocument doc;

bool ndefActive = false;
bool readerReady = false;
unsigned long lastPoll = 0;

// NFC Forum URI prefix table (subset covering the common cases).
const char* URI_PREFIX[] = {
  "", "http://www.", "https://www.", "http://", "https://", "tel:", "mailto:"
};
static const byte URI_PREFIX_LEN = 7;

void emit(const char* status) {
  doc["module"] = "5.5";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}
void emitError(const char* msg) { doc["data"]["msg"] = msg; emit("error"); }

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

bool isNfcTag() {
  MFRC522::PICC_Type t = mfrc522.PICC_GetType(mfrc522.uid.sak);
  return (t == MFRC522::PICC_TYPE_MIFARE_UL);   // NTAG reports as Ultralight on the RC522
}

// Read `len` bytes of user memory starting at USER_START_PAGE into out[].
// MIFARE_Read returns 4 pages (16 B) per call. Returns bytes actually read.
int readUserMem(byte* out, int maxLen) {
  int got = 0;
  for (byte page = USER_START_PAGE; got < maxLen; page += 4) {
    byte buf[18]; byte sz = sizeof(buf);
    if (mfrc522.MIFARE_Read(page, buf, &sz) != MFRC522::STATUS_OK) break;
    int n = min(16, maxLen - got);
    memcpy(out + got, buf, n);
    got += n;
    if (page > 200) break;  // hard stop well past NTAG216
  }
  return got;
}

// Decode the first NDEF URI/Text record found in a TLV-formatted buffer.
bool decodeNdef(const byte* mem, int len, String& typeOut, String& payloadOut) {
  int i = 0;
  while (i < len) {
    byte tlvType = mem[i++];
    if (tlvType == 0x00) continue;        // NULL TLV, skip
    if (tlvType == 0xFE) break;           // terminator
    if (i >= len) break;
    int tlvLen = mem[i++];
    if (tlvLen == 0xFF) {                  // 3-byte length form
      if (i + 1 >= len) break;
      tlvLen = (mem[i] << 8) | mem[i + 1];
      i += 2;
    }
    if (tlvType != 0x03) { i += tlvLen; continue; }   // not an NDEF message TLV

    // NDEF message: parse first record header.
    if (i + 3 >= len) break;
    byte hdr = mem[i];
    byte typeLen = mem[i + 1];
    byte payloadLen = mem[i + 2];
    byte tnf = hdr & 0x07;
    int rec = i + 3;
    if (typeLen == 0 || rec + typeLen > len) break;
    byte recType = mem[rec];
    int payloadStart = rec + typeLen;
    if (payloadStart + payloadLen > len) payloadLen = len - payloadStart;

    if (tnf == 0x01 && recType == 'U') {   // well-known URI
      byte prefixIdx = mem[payloadStart];
      String uri = (prefixIdx < URI_PREFIX_LEN) ? String(URI_PREFIX[prefixIdx]) : String("");
      for (int p = payloadStart + 1; p < payloadStart + payloadLen; p++) uri += (char)mem[p];
      typeOut = "uri"; payloadOut = uri; return true;
    }
    if (tnf == 0x01 && recType == 'T') {   // well-known Text
      byte statusByte = mem[payloadStart];
      byte langLen = statusByte & 0x3F;
      String txt = "";
      for (int p = payloadStart + 1 + langLen; p < payloadStart + payloadLen; p++) txt += (char)mem[p];
      typeOut = "text"; payloadOut = txt; return true;
    }
    return false;   // some other record type
  }
  return false;
}

void doRead() {
  if (!isNfcTag()) { emitError("unsupported_card"); mfrc522.PICC_HaltA(); return; }
  byte mem[256];
  int len = readUserMem(mem, sizeof(mem));
  String type, payload;
  if (len <= 0 || !decodeNdef(mem, len, type, payload)) {
    emitError("no_ndef"); mfrc522.PICC_HaltA(); return;
  }
#if DEBUG_SERIAL
  Serial.print(F("[ndef] ")); Serial.print(type); Serial.print(F(" = ")); Serial.println(payload);
#else
  doc["data"]["record_type"] = type;
  doc["data"]["payload"]     = payload;
  doc["data"]["tag_type"]    = String(mfrc522.PICC_GetTypeName(mfrc522.PICC_GetType(mfrc522.uid.sak)));
  doc["data"]["capacity"]    = len;
  emit("ok");
#endif
  mfrc522.PICC_HaltA();
}

#if NDEF_MODE_WRITE
// Build a TLV-wrapped single URI record for NDEF_URL and write it page by page.
void doWrite() {
  if (!isNfcTag()) { emitError("unsupported_card"); mfrc522.PICC_HaltA(); return; }

  // Choose the longest matching URI prefix to compress the stored string.
  String url = String(NDEF_URL);
  byte prefixIdx = 0; String rest = url;
  for (byte p = 1; p < URI_PREFIX_LEN; p++) {
    String pre = String(URI_PREFIX[p]);
    if (pre.length() && url.startsWith(pre) && pre.length() > String(URI_PREFIX[prefixIdx]).length()) {
      prefixIdx = p; rest = url.substring(pre.length());
    }
  }

  byte payloadLen = 1 + rest.length();        // prefix byte + URI tail
  byte ndefLen = 4 + payloadLen;              // record hdr(1)+typeLen(1)+payloadLen(1)+type(1)+payload
  // TLV: 0x03 <ndefLen> <ndef...> 0xFE
  byte buf[256]; int n = 0;
  buf[n++] = 0x03; buf[n++] = ndefLen;
  buf[n++] = 0xD1;             // MB|ME|SR, TNF=well-known
  buf[n++] = 0x01;             // type length
  buf[n++] = payloadLen;       // payload length
  buf[n++] = 'U';              // URI record type
  buf[n++] = prefixIdx;        // URI prefix code
  for (uint16_t c = 0; c < rest.length(); c++) buf[n++] = rest[c];
  buf[n++] = 0xFE;             // terminator
  while (n % 4 != 0) buf[n++] = 0x00;  // pad to a whole page

  // Write 4 bytes per page from USER_START_PAGE.
  byte written = 0;
  for (int off = 0; off < n; off += 4) {
    byte page = USER_START_PAGE + off / 4;
    if (mfrc522.MIFARE_Ultralight_Write(page, &buf[off], 4) != MFRC522::STATUS_OK) {
      emitError("write_failed"); mfrc522.PICC_HaltA(); return;
    }
    written += 4;
  }

  // Verify by reading the record back.
  byte mem[256]; int rlen = readUserMem(mem, n);
  bool match = (rlen >= n) && (memcmp(mem, buf, n) == 0);

#if DEBUG_SERIAL
  Serial.print(F("[ndef] wrote uri ")); Serial.print(url);
  Serial.print(F("  bytes=")); Serial.print(written);
  Serial.print(F("  verify=")); Serial.println(match ? F("match") : F("mismatch"));
#else
  doc["data"]["record_type"]  = "uri";
  doc["data"]["bytes_written"] = written;
  doc["data"]["verify"]        = match ? "match" : "mismatch";
  emit("ok");
#endif
  mfrc522.PICC_HaltA();
}
#endif

void handleTag() {
#if NDEF_MODE_WRITE
  doWrite();
#else
  doRead();
#endif
  mfrc522.PCD_StopCrypto1();
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println(F("\n[+] RC522 Module 5.5 NDEF Read/Write ready."));
#if NDEF_MODE_WRITE
  Serial.print(F("[i] WRITE mode. Will provision: ")); Serial.println(F(NDEF_URL));
#else
  Serial.println(F("[i] READ mode (default). Present an NTAG/Ultralight tag to decode."));
#endif

#if DEBUG_SERIAL
  if (ensureReader()) {
    Serial.println(F("[+] DEBUG_SERIAL on: RC522 found."));
    ndefActive = true;
  } else {
    Serial.println(F("[!] DEBUG_SERIAL on: RC522 NOT found. Check SDA=10 RST=9 SCK=12 MOSI=11 MISO=13, 3.3V."));
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.5\"") != -1) {
      if (!ensureReader()) emitError("rc522_not_found");
      else ndefActive = true;
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      ndefActive = false;
      if (readerReady) { mfrc522.PICC_HaltA(); mfrc522.PCD_StopCrypto1(); }
    }
  }

  if (ndefActive && readerReady && millis() - lastPoll >= POLL_INTERVAL_MS) {
    lastPoll = millis();
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      handleTag();
    }
  }
}
