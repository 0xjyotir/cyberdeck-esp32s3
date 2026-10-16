/*
 * Cyberdeck ESP32-S3 — Module 5.4: PN532 NDEF Read / Write (standalone)
 * ---------------------------------------------------------------------
 * Reads the NDEF message off an NTAG / MIFARE Ultralight tag and parses
 * the first record (Text and URI well-known types are decoded to a
 * readable string). Optionally — behind a deliberate compile flag —
 * writes one sample NDEF URI record to a tag you own, so you can verify
 * the write path without any risk of an accidental overwrite.
 *
 * Educational / authorized-lab NFC diagnostics only. WRITING MODIFIES
 * THE TAG: only ever enable NDEF_WRITE for a blank/spare tag that you
 * own. Reading is non-destructive and is the default.
 *
 * Wiring (PN532 in I2C mode):
 *   SDA=GPIO8  SCL=GPIO9  VCC=3.3V  GND=GND  RSTPDN->3.3V
 *
 * Requires: Adafruit PN532 + Adafruit BusIO + ArduinoJson v7.x.
 *
 * Serial protocol (115200 baud, newline-delimited JSON):
 *   host -> board : {"module":"5.4","action":"START"} / {"action":"STOP"}
 *   board -> host : {"module":"5.4","status":"ok","data":{"action":"read","record":0,"tnf":1,"type":"U","payload":"https://example.com"}}
 *   no NDEF       : {"module":"5.4","status":"ok","data":{"action":"read","record":-1,"type":"","payload":"no NDEF message"}}
 *   not present   : {"module":"5.4","status":"error","data":{"msg":"pn532_not_found"}}
 *
 * Read vs write is a COMPILE-TIME choice (NDEF_WRITE) so the locked
 * {module,action} protocol stays intact — START never writes unless you
 * built the sketch in write mode on purpose.
 *
 * DEBUG_SERIAL = 1: starts on its own; hold an NTAG to read its NDEF
 *   (or, with NDEF_WRITE = 1, to write the sample URI once). Set
 *   DEBUG_SERIAL = 0 for the dashboard (JSON as above).
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <ArduinoJson.h>

#define DEBUG_SERIAL 1
#define NDEF_WRITE   0     // 1 = write the sample URI to an OWNED blank tag; 0 = read only (safe default)

static const char* SAMPLE_URI = "cyberdeck-esp32s3.test";   // written only when NDEF_WRITE = 1

static const uint8_t PN532_SDA = 8;
static const uint8_t PN532_SCL = 9;
static const int8_t  PN532_IRQ = -1;
static const int8_t  PN532_RST = -1;

static const int      BAUD_RATE    = 115200;
static const uint32_t POLL_EVERY   = 400;
static const uint16_t READ_TIMEOUT = 150;
static const uint8_t  UL_MAX_PAGES = 135;   // enough to cover the common NTAG21x user area

Adafruit_PN532 nfc(PN532_IRQ, PN532_RST, &Wire);
JsonDocument doc;

bool ndefActive = false;
bool pn532Ready = false;
String lastUid  = "";
unsigned long lastPoll = 0;

// URI prefix table (NFC Forum URI record abbreviations, index 0x00..).
const char* URI_PREFIX[] = {
  "", "http://www.", "https://www.", "http://", "https://", "tel:", "mailto:",
  "ftp://anonymous:anonymous@", "ftp://ftp.", "ftps://", "sftp://", "smb://",
  "nfs://", "ftp://", "dav://", "news:", "telnet://", "imap:", "rtsp://",
  "urn:", "pop:", "sip:", "sips:", "tftp:", "btspp://"
};
const uint8_t URI_PREFIX_N = sizeof(URI_PREFIX) / sizeof(URI_PREFIX[0]);

String uidToHex(const uint8_t* uid, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) { if (uid[i] < 0x10) s += '0'; s += String(uid[i], HEX); }
  s.toUpperCase();
  return s;
}

void emit(const char* status) {
  doc["module"] = "5.4";
  doc["status"] = status;
  serializeJson(doc, Serial);
  Serial.println();
  doc.clear();
}

void emitRead(int record, int tnf, const String& type, const String& payload) {
  doc["data"]["action"]  = "read";
  doc["data"]["record"]  = record;
  if (tnf >= 0) doc["data"]["tnf"] = tnf;
  doc["data"]["type"]    = type;
  doc["data"]["payload"] = payload;
  emit("ok");
#if DEBUG_SERIAL
  Serial.print("[5.4] read  rec="); Serial.print(record);
  Serial.print(" tnf="); Serial.print(tnf);
  Serial.print(" type="); Serial.print(type);
  Serial.print(" payload="); Serial.println(payload);
#endif
}

bool ensurePn532() {
  if (pn532Ready) return true;
  Wire.begin(PN532_SDA, PN532_SCL);
  nfc.begin();
  if (!nfc.getFirmwareVersion()) return false;
  nfc.SAMConfig();
  pn532Ready = true;
  return true;
}

// Read the user memory of an NTAG/Ultralight into buf; returns bytes read.
uint16_t readUserMem(uint8_t* buf, uint16_t cap) {
  uint16_t n = 0;
  for (uint8_t page = 4; page < UL_MAX_PAGES && n + 4 <= cap; page++) {  // page 4 = start of user area
    uint8_t d[4];
    if (!nfc.mifareultralight_ReadPage(page, d)) break;
    memcpy(buf + n, d, 4);
    n += 4;
  }
  return n;
}

// Find TLV 0x03 (NDEF message) and decode its first record into type/payload.
// Returns the record index (0) on success, -1 if no NDEF message was found.
int parseNdef(const uint8_t* mem, uint16_t n, int& tnf, String& type, String& payload) {
  uint16_t i = 0;
  while (i < n) {
    uint8_t t = mem[i++];
    if (t == 0x00) continue;          // NULL TLV — skip
    if (t == 0xFE) break;             // Terminator TLV
    if (i >= n) break;
    uint16_t len = mem[i++];
    if (len == 0xFF) {                // 3-byte length form
      if (i + 1 >= n) break;
      len = (mem[i] << 8) | mem[i + 1];
      i += 2;
    }
    if (t == 0x03) {                  // NDEF message TLV — parse its first record
      if (i >= n) break;
      uint8_t hdr = mem[i++];
      tnf = hdr & 0x07;
      bool sr = hdr & 0x10;           // short record
      if (i >= n) break;
      uint8_t typeLen = mem[i++];
      uint32_t payLen = 0;
      if (sr) { payLen = mem[i++]; }
      else    { payLen = ((uint32_t)mem[i] << 24) | ((uint32_t)mem[i+1] << 16) | (mem[i+2] << 8) | mem[i+3]; i += 4; }
      if (hdr & 0x08) i++;            // IL flag → skip ID length byte
      String rtype;
      for (uint8_t k = 0; k < typeLen && i < n; k++) rtype += char(mem[i++]);
      // Decode well-known Text ('T') and URI ('U'); otherwise emit raw hex.
      String out;
      if (rtype == "U" && payLen > 0 && i < n) {
        uint8_t code = mem[i++]; payLen--;
        if (code < URI_PREFIX_N) out += URI_PREFIX[code];
        for (uint32_t k = 0; k < payLen && i < n; k++) out += char(mem[i++]);
      } else if (rtype == "T" && payLen > 0 && i < n) {
        uint8_t status = mem[i++]; payLen--;
        uint8_t langLen = status & 0x3F;
        for (uint8_t k = 0; k < langLen && i < n && payLen; k++) { i++; payLen--; }
        for (uint32_t k = 0; k < payLen && i < n; k++) out += char(mem[i++]);
      } else {
        for (uint32_t k = 0; k < payLen && i < n; k++) { if (mem[i] < 0x10) out += '0'; out += String(mem[i++], HEX); }
        out.toUpperCase();
      }
      type = rtype; payload = out;
      return 0;
    }
    i += len;                         // not NDEF — skip this TLV's value
  }
  return -1;
}

void setup() {
  Serial.begin(BAUD_RATE);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 4000)) delay(10);
  Serial.println("\n[+] PN532 Module 5.4 NDEF Read/Write ready.");
#if NDEF_WRITE
  Serial.println("[!] NDEF_WRITE = 1 — this build WRITES to the tag. Use a blank/spare tag you own.");
#endif

#if DEBUG_SERIAL
  if (ensurePn532()) {
    Serial.println("[+] DEBUG_SERIAL on: PN532 found. Hold an NTAG/Ultralight tag to the antenna.");
    ndefActive = true;
  } else {
    Serial.println("[!] DEBUG_SERIAL on: PN532 NOT found. Check SDA=8 SCL=9, 3.3V, I2C jumpers.");
  }
#endif
}

void loop() {
  if (Serial.available() > 0) {
    String in = Serial.readStringUntil('\n');
    in.trim();
    if (in.indexOf("\"action\":\"START\"") != -1 && in.indexOf("\"module\":\"5.4\"") != -1) {
      if (!ensurePn532()) { doc["data"]["msg"] = "pn532_not_found"; emit("error"); }
      else { ndefActive = true; lastUid = ""; }
    } else if (in.indexOf("\"action\":\"STOP\"") != -1) {
      ndefActive = false;
    }
  }

  if (ndefActive && pn532Ready && millis() - lastPoll >= POLL_EVERY) {
    lastPoll = millis();
    uint8_t uid[7] = {0};
    uint8_t uidLen = 0;
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, READ_TIMEOUT) && uidLen > 0) {
      String hexUid = uidToHex(uid, uidLen);
      if (hexUid != lastUid) {
        lastUid = hexUid;

        if (uidLen != 7) {            // NDEF demo targets NTAG/Ultralight
          emitRead(-1, -1, "", "present a 7-byte-UID NTAG/Ultralight for the NDEF demo");
          return;
        }

#if NDEF_WRITE
        // One-shot write of the sample URI to this owned tag, then report it.
        bool ok = nfc.ntag2xx_WriteNDEFURI(NDEF_URIPREFIX_HTTPS, (char*)SAMPLE_URI);
        doc["data"]["action"]  = "write";
        doc["data"]["record"]  = 0;
        doc["data"]["type"]    = "U";
        doc["data"]["payload"] = ok ? (String("https://") + SAMPLE_URI) : String("write failed");
        emit(ok ? "ok" : "error");
  #if DEBUG_SERIAL
        Serial.print("[5.4] write "); Serial.println(ok ? "ok" : "FAILED");
  #endif
#else
        // Read + parse the NDEF message.
        static uint8_t mem[UL_MAX_PAGES * 4];
        uint16_t n = readUserMem(mem, sizeof(mem));
        int tnf = -1; String type, payload;
        int rec = parseNdef(mem, n, tnf, type, payload);
        if (rec < 0) emitRead(-1, -1, "", "no NDEF message");
        else         emitRead(rec, tnf, type, payload);
#endif
      }
    } else {
      lastUid = "";
    }
  }
}
