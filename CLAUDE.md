# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash

This is an Arduino project — there is no build script. Use Arduino IDE 2.x or `arduino-cli`.

**Board:** Arduino Nano ESP32 (ABX00083 / ESP32-S3). Select `Arduino Nano ESP32` in the board manager (`arduino-esp32` package). USB CDC On Boot must be **Enabled**.

**Required libraries** (install via Library Manager):
- `Adafruit PN532` by Adafruit
- `FastLED` by Daniel Garcia — **pin to ≤ 3.10.3**. FastLED 3.10.4 fails to build against ESP32 core 2.0.18 (`io_pin_remap.h` macro clashes + ambiguous `memcpy`).
- `ArduinoJson` by Benoit Blanchon

**Flash:**
```
# Arduino CLI
arduino-cli compile --fqbn arduino:esp32:nano_nora firmware/nfc_activation
arduino-cli upload  --fqbn arduino:esp32:nano_nora -p /dev/cu.usbmodem* firmware/nfc_activation
```

Serial monitor: 115200 baud.

## Hardware Context

| Component | Notes |
|---|---|
| PN532 NFC module | I2C mode: SW1=ON, SW2=OFF. Address 0x24. SDA→A4, SCL→A5, VCC→3V3 (not 5V). |
| WS2812B LED ring | 40 LEDs, 4". Data→GPIO5, 5V from VBUS. |
| NFC cards | NTAG215, 504 bytes, 13.56 MHz. |

PN532 V3 modules from Amazon are clones with inconsistent RF performance; Elechouse V4 is the production recommendation.

## Firmware Architecture

**Two sketches:**
- `firmware/nfc_activation/` — production firmware
- `firmware/pn532_test/` — standalone I2C/PN532 diagnostic (flash this first on a new board)

**`nfc_activation` is dual-core:**
- **Core 0 — `nfcTask`**: all I2C/PN532 blocking operations. Polls for cards, reads NDEF pages, writes to shared volatile globals.
- **Core 1 — `loop()`**: LED animation, serial command processing, state machine transitions. Never touches I2C.

**State machine:** `LedState` structs stored in `states[]` (up to `MAX_STATES=16`). States have animation presets, gradient colors, duration/returnTo, and transition type. States persist to ESP32 NVS flash via `Preferences`. The five built-in states (`idle`, `active`, `reading`, `success`, `failure`) are `isDefault=true` and cannot be deleted.

**NFC tap flow:**
1. `nfcTask` detects card rising edge via `readPassiveTargetID` → sets `nfcCardDetected=true`, `readPending=true`
2. `loop()` sees `nfcCardDetected` → triggers `reading` state
3. `nfcTask` retries `readNdefGuestData()` (pages 4–29) on every poll while `readPending` and within `READ_RETRY_MS=2800ms`
4. On successful page read: sets `nfcNewScan=true`; `loop()` picks it up, validates badge format, emits `[scan]` JSON over serial, queues `nfcResultPending`
5. After `readingMinMs` (default 600ms), triggers `success` or `failure`

**Badge format validation (`isOurBadgeFormat`):** NDEF payload must start with 16 decimal digits + 4 alphanumeric chars (20-char badge ID), then caret-delimited `^firstName^lastName^company^`. Foreign cards (phones, hotel keys) are rejected — emit `[warn]`, flash failure, never emit `[scan]`.

**Reset cards:** UIDs enrolled via `ADDRESET` bypass `acceptTaps` and all state guards. On tap: hard-reset to `idle`, clear in-flight scan state, re-enable `acceptTaps`, emit `[reset]` (never `[scan]`).

**Serial protocol:** Board emits `[scan]`, `[hb]`, `[state]`, `[cfg]`, `[warn]`, `[reset]`, `[ok]`, `[err]`. Host sends `OK`/`FAIL`/`TRIGGER <name>`/`SET ...`/`SAVE`/`RESET`/`SETTAPS`/`SETDELAY`/`ADDRESET`/etc. See README or the sketch header for the full command table.

## Key Timing Constants

| Constant | Value | Purpose |
|---|---|---|
| `READ_SETTLE_MS` | 40ms | Delay after card select before reading pages |
| `READ_RETRY_MS` | 2800ms | How long to keep retrying NDEF read (must be < `reading.duration` 3000ms) |
| `TAP_COOLDOWN_MS` | 1500ms | Minimum gap between accepted taps |
| `CARD_RELEASE_MS` | 400ms | Field must read empty this long before re-arming |
| `Wire.setTimeOut` | 200ms | I2C transaction timeout |

## Dev Tool

`tools/dev-tool.html` — browser-based LED state editor and serial debug console. Requires Chrome or Edge (Web Serial API). Open the file directly from disk (no server needed). Use **Simulate Tap** to test LED animations without hardware, **Import**/**Flash States to Board** to load `tools/led-states.json` onto a fresh board.
