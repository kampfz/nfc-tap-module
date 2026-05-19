/**
 * PN532 Read Test — Arduino Nano ESP32 (ABX00083)
 * ------------------------------------------------
 * Minimal sketch to verify the Elechouse PN532 module is wired
 * correctly and can read NTAG215 (or any ISO14443A) tags.
 *
 * On boot it prints the PN532 firmware version. Then it polls
 * for cards and prints the UID (hex) plus the type/length of
 * each tag it sees.
 *
 * Wiring (PN532 → ESP32):
 *   SDA → A4
 *   SCL → A5
 *   VCC → 3V3   (NOT 5V)
 *   GND → GND
 *
 * PN532 mode switches: SW1 = ON, SW2 = OFF  (I2C mode, address 0x24)
 *
 * Library: Adafruit PN532 (Library Manager)
 * Board:   Arduino ESP32 Boards → "Arduino Nano Nora ESP32"
 */

#include <Wire.h>
#include <Adafruit_PN532.h>

// Use the A4/A5 board constants (NOT bare 11/12). On the Nano ESP32, A4 and
// A5 always resolve to the correct silk pads (GPIO 11 / 12) regardless of
// the IDE's "Pin Numbering" mode. Bare 11/12 in default mode would point at
// D11/D12 (MOSI/MISO pads) instead.
#define PN532_SDA    A4
#define PN532_SCL    A5

// IMPORTANT: the Adafruit_PN532 two-arg constructor is (irq, reset) — NOT
// (sda, scl). 255 = "not connected, skip setup".
#define PN532_IRQ    255
#define PN532_RESET  255

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

// Quick I2C bus scan — prints any device addresses that ACK.
// PN532 in I2C mode should ACK at 0x24.
void scanI2C() {
  Serial.println(F("[i2c]  Scanning bus..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("[i2c]  Device at 0x"));
      if (addr < 0x10) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == 0x24) Serial.print(F("  <-- PN532"));
      Serial.println();
      found++;
    }
  }
  if (!found) {
    Serial.println(F("[i2c]  No devices found on the bus."));
    Serial.println(F("       Check power (3V3, GND), SDA/SCL pins, mode switches."));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { /* wait briefly for USB CDC */ }
  delay(200);

  Serial.println();
  Serial.println(F("[boot] PN532 read test"));

  Wire.begin(PN532_SDA, PN532_SCL);   // A4 = SDA, A5 = SCL
  Wire.setTimeOut(50);                // ms — fail fast if a slave hangs the bus
  delay(50);
  scanI2C();

  nfc.begin();

  uint32_t version = nfc.getFirmwareVersion();
  if (!version) {
    Serial.println(F("[err]  PN532 not responding to firmware-version query."));
    Serial.println(F("       If the I2C scan above showed 0x24, the chip is on the"));
    Serial.println(F("       bus but the library handshake failed — try power cycle."));
    Serial.println(F("       If the scan was empty: check SDA on A4, SCL on A5,"));
    Serial.println(F("       VCC=3V3, GND, and mode switches SW1=ON SW2=OFF."));
    while (1) { delay(1000); }
  }

  Serial.print(F("[ok]   PN532 chip 0x"));
  Serial.println((version >> 24) & 0xFF, HEX);
  Serial.print(F("[ok]   Firmware v"));
  Serial.print((version >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((version >> 8) & 0xFF, DEC);

  // Configure the Secure Access Module — required before reading cards.
  // We deliberately do NOT call setPassiveActivationRetries(0xFF): that
  // tells the chip to retry forever, which conflicts with the per-call
  // host timeout in readPassiveTargetID and desyncs the I2C handshake.
  // The chip default (a small finite retry count) is what we want here.
  nfc.SAMConfig();

  Serial.println(F("[ok]   Ready — tap a tag on the reader"));
}

// ── UID → Wave-ID-style decimal string ──────────────────────────────────────
// RFIdeas Wave ID readers (configured for "full forward") pack the entire
// UID as a big-endian integer and "type" it as a decimal string. NTAG215
// UIDs are 7 bytes (max 56 bits) — well within a uint64_t.
String uidToWaveId(uint8_t* uid, uint8_t uidLen) {
  uint64_t v = 0;
  for (uint8_t i = 0; i < uidLen; i++) v = (v << 8) | uid[i];

  // Arduino's String/Serial don't print uint64_t directly — build it manually.
  if (v == 0) return String("0");
  char buf[21]; uint8_t i = sizeof(buf) - 1; buf[i] = 0;
  while (v && i > 0) { buf[--i] = '0' + (v % 10); v /= 10; }
  return String(&buf[i]);
}

// ── NDEF parser ──────────────────────────────────────────────────────────────
// Parses a TLV-encoded NDEF stream and extracts the first Well-Known Text
// record. The byte layout is identical for NTAG2xx and MIFARE Classic — only
// the way you *fetch* the bytes differs (page reads vs sector auth + block
// reads), so this function is shared between both card types.
//
// TLV stream:   [03][len][NDEF message][FE]
// NDEF record:  [header][typeLen=1][payloadLen]['T'][status][lang...][text...]
// status byte:  bit7 = encoding (0=UTF-8, 1=UTF-16), bits5-0 = lang code length
bool parseNdefText(uint8_t* data, uint16_t dataLen, String& out) {
  // Walk the TLV stream until we find an NDEF Message TLV (tag=0x03) or end.
  uint16_t pos = 0;
  while (pos < dataLen) {
    uint8_t tag = data[pos];
    if (tag == 0x03) break;             // NDEF Message — found it
    if (tag == 0xFE) {                  // Terminator — no NDEF on tag
      Serial.println(F("[ndef] Tag is blank (no NDEF message). Write one with NFC Tools."));
      return false;
    }
    if (tag == 0x00) { pos++; continue; }   // Null TLV is single-byte
    // Other TLVs (0x01 lock control, 0x02 memory control) carry a length byte
    if (pos + 1 >= dataLen) return false;
    pos += 2 + data[pos + 1];
  }
  if (pos >= dataLen) {
    Serial.println(F("[ndef] No NDEF TLV found in user memory"));
    return false;
  }

  pos++;                                // skip 0x03
  uint16_t ndefLen = data[pos++];       // 1-byte length, or 0xFF + 2-byte length
  if (ndefLen == 0xFF) {
    ndefLen = ((uint16_t)data[pos] << 8) | data[pos + 1];
    pos += 2;
  }
  if (pos + ndefLen > dataLen) {
    Serial.println(F("[ndef] NDEF message extends past read buffer — increase read size"));
    return false;
  }

  // Parse the first record header
  uint8_t header = data[pos++];
  uint8_t typeLen = data[pos++];
  uint32_t payloadLen;
  if (header & 0x10) {                  // SR (short record) — 1-byte length
    payloadLen = data[pos++];
  } else {                              // 4-byte big-endian length
    payloadLen = ((uint32_t)data[pos] << 24) | ((uint32_t)data[pos+1] << 16) |
                 ((uint32_t)data[pos+2] << 8) | data[pos+3];
    pos += 4;
  }
  uint8_t idLen = (header & 0x08) ? data[pos++] : 0;

  if (typeLen != 1 || data[pos] != 'T') {
    Serial.print(F("[ndef] Not a Text record (type='"));
    for (uint8_t i = 0; i < typeLen && pos + i < dataLen; i++) {
      Serial.print((char)data[pos + i]);
    }
    Serial.println(F("') — only Text records (T) are decoded by this sketch"));
    return false;
  }
  pos += typeLen + idLen;

  // Text record payload: [status][lang][text]
  uint8_t status = data[pos++];
  uint8_t langLen = status & 0x3F;
  bool utf16 = status & 0x80;
  pos += langLen;                       // skip language code (e.g. "en")

  uint32_t textLen = (payloadLen > (uint32_t)(1 + langLen))
                       ? payloadLen - 1 - langLen : 0;

  if (utf16) {
    Serial.println(F("[ndef] Text record is UTF-16 — only UTF-8 is decoded here"));
    return false;
  }

  out = "";
  for (uint32_t i = 0; i < textLen && pos + i < dataLen; i++) {
    out += (char)data[pos + i];
  }
  return true;
}

// ── NTAG2xx: read user memory starting at page 4 ─────────────────────────────
// NTAG215 has 504 bytes of user memory; we read the first 64 bytes which is
// plenty for a typical short text ID.
bool readNdefNtag(String& out) {
  const uint8_t NUM_PAGES = 16;
  uint8_t data[NUM_PAGES * 4] = { 0 };

  for (uint8_t i = 0; i < NUM_PAGES; i++) {
    if (!nfc.ntag2xx_ReadPage(4 + i, &data[i * 4])) {
      Serial.print(F("[ndef] NTAG page read failed at page "));
      Serial.println(4 + i);
      return false;
    }
  }
  return parseNdefText(data, sizeof(data), out);
}

// ── MIFARE Classic: authenticate & read sectors 1..3, skipping trailers ──────
// Standard NFC Forum mapping uses key D3F7D3F7D3F7 for NDEF sectors. Blank
// factory cards still have all-FF keys, so we try both. We MUST re-authenticate
// at every sector boundary — auth state doesn't carry across.
//
// Layout: each sector = 4 blocks of 16 bytes. Block 3 of each sector is the
// trailer (keys/access bits) and is skipped — only blocks 0..2 are user data.
// 3 sectors × 3 user blocks × 16 bytes = 144 bytes, plenty for short text.
bool readNdefMifareClassic(uint8_t* uid, uint8_t uidLen, String& out) {
  uint8_t ndefKey[6]    = { 0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7 };
  uint8_t defaultKey[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  // Probe sector 1 with NDEF key first, fall back to default key
  uint8_t* key = ndefKey;
  if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLen, 4, 0, key)) {
    key = defaultKey;
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLen, 4, 0, key)) {
      Serial.println(F("[ndef] MIFARE Classic auth failed (tried D3F7D3F7D3F7 and FFFFFFFFFFFF)"));
      Serial.println(F("       Card may use non-default keys — can't read NDEF without them."));
      return false;
    }
    Serial.println(F("[ndef] Note: card uses default key, not standard NDEF key"));
  }

  uint8_t data[144] = { 0 };
  uint16_t dataLen = 0;

  for (uint8_t sector = 1; sector <= 3; sector++) {
    uint8_t firstBlock = sector * 4;

    // Re-authenticate at every new sector (sector 1 already done above)
    if (sector > 1) {
      if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLen, firstBlock, 0, key)) {
        break;   // give up; we likely have enough data already
      }
    }
    for (uint8_t b = 0; b < 3; b++) {   // user blocks only (skip trailer at +3)
      if (!nfc.mifareclassic_ReadDataBlock(firstBlock + b, &data[dataLen])) {
        Serial.print(F("[ndef] Block read failed at "));
        Serial.println(firstBlock + b);
        return false;
      }
      dataLen += 16;
    }
  }
  return parseNdefText(data, dataLen, out);
}

void loop() {
  uint8_t uid[7] = { 0 };
  uint8_t uidLen = 0;

  // 500 ms read window — gives a tap more time to land inside one poll.
  bool found = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 500);

  if (found) {
    // Build the protocol-format ID: continuous uppercase hex, no spaces.
    // Matches what nfc_activation.ino sends to TouchDesigner.
    char uidStr[15] = { 0 };   // up to 7 bytes × 2 hex chars + null
    for (uint8_t i = 0; i < uidLen; i++) {
      sprintf(&uidStr[i * 2], "%02X", uid[i]);
    }

    Serial.print(F("[tag]  UID hex: "));
    Serial.print(uidStr);
    Serial.print(F("   ("));
    Serial.print(uidLen);
    Serial.print(F(" bytes, "));
    if (uidLen == 4)      Serial.print(F("MIFARE Classic / Ultralight"));
    else if (uidLen == 7) Serial.print(F("NTAG / Ultralight / DESFire"));
    else                  Serial.print(F("unknown"));
    Serial.println(')');

    Serial.print(F("[id]   "));
    Serial.println(uidToWaveId(uid, uidLen));

    // Try to read an NDEF text record from the user memory.
    // 7-byte UID → NTAG2xx (page reads, no auth)
    // 4-byte UID → MIFARE Classic (sector auth + block reads)
    String text;
    bool ok = false;
    if (uidLen == 7) {
      ok = readNdefNtag(text);
    } else if (uidLen == 4) {
      ok = readNdefMifareClassic(uid, uidLen, text);
    } else {
      Serial.println(F("[ndef] NDEF read skipped — unrecognized UID length"));
    }
    if (ok) {
      Serial.print(F("[id]   "));
      Serial.println(text);
    }

    // Small debounce so a single tap doesn't spam dozens of reads.
    delay(750);
  }
}
