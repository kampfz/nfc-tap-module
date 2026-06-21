# NFC Tap Module

Tabletop NFC tap module: guest taps an NTAG215 card on the reader, a 40-LED ring provides ambient idle animation and tap feedback (success/failure breathing flash), and the card's UID + NDEF guest data are emitted over USB Serial to a host computer.

This repo contains everything needed to build, flash, and bench-test a module:

- `firmware/nfc_activation/` — main production firmware (Arduino sketch)
- `firmware/pn532_test/` — diagnostic sketch for verifying PN532 wiring
- `tools/dev-tool.html` — browser-based LED state editor and serial debug tool
- `tools/led-states.json` — default LED state config
- `docs/wiring-diagram.html` — interactive wiring reference

---

## Hardware

| Component | Part | Notes |
|---|---|---|
| Microcontroller | Arduino Nano ESP32 — ABX00083 (ESP32-S3) | |
| NFC Reader | Elechouse PN532 v3 or v4 (I2C mode) | V4 recommended for production (CE/FCC certified) |
| LED Ring | WS2812B 40-LED Ring (4" / 106mm), 5V | |
| NFC Cards | NTAG215 PVC cards, 504 bytes, 13.56MHz | |

> **PN532 sourcing:** All V3 modules on Amazon are third-party clones — functional but not officially certified. The genuine V4 is available direct from elechouse.com, is a drop-in replacement with better read performance, and is recommended for production.

---

## Wiring

| From | Pin | To | Pin | Notes |
|---|---|---|---|---|
| PN532 | SDA | ESP32 | A4 | I2C data |
| PN532 | SCL | ESP32 | A5 | I2C clock |
| PN532 | VCC | ESP32 | 3V3 | 3.3V only — **not 5V** |
| PN532 | GND | ESP32 | GND | Common ground |
| LED Ring | DI | ESP32 | GPIO5 | Data in (direct connection) |
| LED Ring | 5V | ESP32 | VBUS | USB 5V power |
| LED Ring | GND | ESP32 | GND | Common ground |
| LED Ring | DO | — | — | Unused (for daisy-chaining) |

**PN532 I2C mode switch:** SW1=ON, SW2=OFF. Must be set before powering up. I2C address: `0x24`.

**Power:** The LED ring is powered directly from the ESP32's VBUS pin (USB 5V rail). No external supply needed. 40 LEDs at full white draw ~2.5A, but with brightness capped (~50%) a standard USB port is sufficient. For full brightness, use a USB-C power adapter rated 2A+.

See `docs/wiring-diagram.html` for an interactive view.

---

## Firmware

**File:** `firmware/nfc_activation/nfc_activation.ino`

### Build environment

- **Arduino IDE 2.x** (or arduino-cli)
- **Board package:** Arduino ESP32 boards — select `Arduino Nano ESP32`
- **Libraries** (install via Library Manager):
  - `Adafruit PN532` by Adafruit
  - `FastLED` by Daniel Garcia — **use ≤ 3.10.3.** FastLED **3.10.4** fails to build against ESP32 core 2.0.18 (`io_pin_remap.h` macro clashes on `pinMode`/`digitalWrite` + an ambiguous `memcpy` in `executeTransition`). If Library Manager auto-updates you to 3.10.4 and the build breaks, downgrade: `arduino-cli lib install FastLED@3.10.3`.
  - `ArduinoJson` by Benoit Blanchon

### Flashing

1. Open `firmware/nfc_activation/nfc_activation.ino` in the Arduino IDE
2. Select board: **Arduino Nano ESP32**, USB CDC On Boot: **Enabled**
3. Select the COM port for the board
4. Click **Upload**

On first flash, the firmware writes its default LED states to ESP32 flash (NVS) and they persist across power cycles.

### What the firmware does

- Polls the PN532 over I2C for NFC taps
- On tap, reads the card UID and any NDEF text record (badge ID, name, company)
- Emits scan data + heartbeat + state changes over USB Serial (115200 baud)
- Runs a dynamic state machine driving the LED ring (idle / active / reading / success / failure, plus any custom states added via the dev tool)
- Accepts serial commands to trigger states, modify state parameters, and import/export configs
- **Validates the badge format** before flashing success — foreign cards are rejected, not forwarded (see [Badge format check](#badge-format-check))
- Supports **reset cards** — designated UIDs that, when tapped, hard-reset the ring to `idle` and re-enable taps even if the device has wedged (see [Reset cards](#reset-cards))

### Default LED states

| State | Preset | Color | Behavior |
|-------|--------|-------|----------|
| idle | wave | white | Indefinite ambient (default boot state) |
| active | noise | blue | Indefinite post-scan ambient |
| reading | chase | amber | Card detected, ~3s timeout to failure |
| success | breathe | green | 1.5s, returns to active |
| failure | breathe | red | 1.5s, returns to idle |

Defaults can be modified via the dev tool but not deleted. Additional custom states can be added at runtime.

---

## Dev Tool / LED State Manager

**File:** `tools/dev-tool.html`

Browser-based editor and serial debug tool. **Requires Chrome or Edge** (Web Serial API — Firefox/Safari are not supported).

### Usage

1. Open `tools/dev-tool.html` in Chrome or Edge (double-click the file — it runs locally, no server needed)
2. Click **Connect** and pick the ESP32's serial port from the browser dialog
3. The current board state appears in the UI:
   - Left panel: state list + add/delete controls
   - Right panel: selected state's animation params, gradient colors, transitions, duration
   - Bottom: global settings (Accept Taps gate, reading delay) + action buttons

### Bench-testing a board

1. **Connect** to the ESP32
2. Click **Simulate Tap** — triggers the `reading` state, then randomly fires success (80%) or failure (20%). Use this to confirm LEDs animate correctly without an NFC card.
3. Hold a real NTAG215 card on the reader for ~1 second — `[scan]` line appears in the debug log; LED ring goes through reading → success.
4. Click **OK** / **FAIL** buttons to manually trigger success/failure states.

### Loading the default config onto a fresh board

1. Click **Import** → select `tools/led-states.json`
2. Click **Flash States to Board** — sends all states via serial and persists to NVS in one step
3. Power cycle to confirm states survived the reboot

### Live tuning

All edits in the dev tool send commands to the board in real time. To make changes permanent, click **Save to Board** (writes to NVS). **Reset** wipes the board back to compiled defaults.

### Reset cards

The **Reset Cards** panel (below Global Settings) manages cards that recover the activation when tapped — handy if the ring gets stuck or taps are paused. The enrolled list syncs live from the board on connect.

1. Tap the card you want to dedicate — its UID appears on the **Enroll last tapped card** button
2. Click that button (or type a UID and **+ Add**)
3. Click **Save to Board** to persist the enrollment to NVS

Remove a card with the `✕` next to it. See [Reset cards](#reset-cards-1) under the protocol for what happens on a reset tap.

---

## PN532 Diagnostic Sketch

**File:** `firmware/pn532_test/pn532_test.ino`

Standalone sketch for verifying the PN532 module independently of the LED ring. On boot it scans the I2C bus, prints the PN532 firmware version, then polls for tags and prints UIDs. Useful for confirming wiring on a freshly assembled board before flashing the full activation firmware.

**Wiring note:** the `Adafruit_PN532(a, b)` two-argument constructor takes `(IRQ, RESET)` pins — **not** `(SDA, SCL)`. Pass `255, 255` (unused) and configure the I2C bus via `Wire.begin(A4, A5)` separately.

---

## Serial Protocol (115200 baud)

Reference for testing without the dev tool — any serial monitor will work.

### Board → Host

| Message | Description |
|---------|-------------|
| `[scan] {"uid":"...","badgeId":"...","name":"...","company":"..."}` | NFC scan (JSON). **Emitted only for a card that passes the format check** — a non-badge produces no `[scan]` (see [Badge format check](#badge-format-check)). |
| `[hb] <millis>` | Heartbeat, every 5s |
| `[state] <name>` | Emitted on every state change |
| `[warn] unrecognized badge format — rejected (uid <uid>)` | Card read but not one of our badges — **no `[scan]` is emitted**, flashes failure. UID is logged so a foreign card can still be enrolled as a reset card. |
| `[cfg] acceptTaps=<0\|1>` | Echoed on boot, after each `SETTAPS`, and when a reset card re-enables taps |
| `[reset] state cleared by card <uid>` | A reset card was tapped (see [Reset cards](#reset-cards-1)) |
| `[reset] <uid>,<uid>,...` | `LISTRESET` reply — enrolled reset UIDs (empty if none) |
| `[ok] ...` / `[err] ...` | Command acknowledgements |

### Host → Board

| Command | Description |
|---------|-------------|
| `OK` | Trigger success state |
| `FAIL` | Trigger failure state |
| `TRIGGER <name>` | Trigger any state by name |
| `LIST` | List all state names |
| `DUMP` | Export all states as JSON |
| `SAVE` | Persist current states to flash (NVS) |
| `RESET` | Wipe NVS, restore compiled defaults |
| `SETTAPS 0\|1` | Global tap gate: 1 = poll NFC, 0 = pause polling |
| `SETDELAY <ms>` | Minimum reading-state display time (0–10000ms, default 600) |
| `ADDSTATE <name>` | Create new state |
| `DELSTATE <name>` | Delete state (not defaults) |
| `SET <state> <param> <val>` | Modify a state parameter |
| `ADDCOLOR <state> <r> <g> <b> [<pos>]` | Add gradient color stop |
| `SETCOLOR <state> <idx> <r> <g> <b> [<pos>]` | Modify a color stop |
| `DELCOLOR <state> <idx>` | Remove a color stop |
| `SETPOS <state> <idx> <pos>` | Reposition a gradient stop (0.0–1.0) |
| `IMPORT` | Begin JSON state import (then send JSON + `ENDIMPORT`) |
| `ADDRESET <uid>` | Enroll a UID as a reset card (no arg = last scanned) |
| `DELRESET <uid>` | Remove a reset card |
| `LISTRESET` | List enrolled reset UIDs (replies `[reset] <uid>,...`) |

Cards must be held on the reader for ~1 second to read the NDEF record; quick taps only capture the UID.

The firmware enforces a `TAP_COOLDOWN_MS` of 1500ms between scans to prevent double-fires.

### Badge format check

Before flashing **success** or forwarding guest data, the firmware checks that the scanned
card is one of our badges. The NDEF text record must begin with a 20-character badge ID —
**16 decimal digits + a 4-character alphanumeric activation code** — ahead of the
caret-delimited guest fields (`{badgeId}{code}^{firstName}^{lastName}^{company}^`).

A card that fails (foreign NTAG, hotel key, transit pass, a **phone**, …) is **rejected**: the
firmware emits `[warn] unrecognized badge format — rejected (uid <uid>)` and flashes **failure**.
Crucially it does **not** emit a `[scan]` — so the host never sees the tap and can't drive the
ring to success (e.g. a TouchDesigner state flashing green off the scan). The UID is logged so a
foreign card can still be enrolled as a reset card. Only a passing card emits `[scan]`, flashes
success, and forwards full guest data.

### Reset cards

A reset card is any NFC card whose UID is enrolled in the reset list (max 8, persisted to NVS on `SAVE`). When tapped, the firmware **hard-resets to `idle`, re-enables `acceptTaps`, and clears any in-flight scan** — deliberately bypassing the tap gate and the success/failure/reading guards, so it recovers the activation even when the ring is stuck mid-scan. A reset tap emits `[reset] state cleared by card <uid>` rather than `[scan] ...`, so a host never processes it as a guest tap. Because a reset re-enables taps, it then also emits `[cfg] acceptTaps=1` so the host/dev-tool re-syncs its tap-gate state (the dev tool's **Accept Taps** toggle flips back to On automatically). The PN532 is polled even while taps are gated off, so reset cards work regardless of the `acceptTaps` state.

Enroll cards from the dev tool's **Reset Cards** panel or via `ADDRESET`/`DELRESET`/`LISTRESET`, then `SAVE`.

---

## Quick Start — Bench Bringup of a New Module

1. Wire per the table above (or `docs/wiring-diagram.html`)
2. Set PN532 DIP switches to I2C mode (SW1=ON, SW2=OFF)
3. Install the three Arduino libraries listed under **Firmware → Build environment**
4. Flash `firmware/pn532_test/pn532_test.ino` and open the serial monitor at 115200 — confirm the PN532 firmware version prints and tag UIDs appear when you tap a card
5. Flash `firmware/nfc_activation/nfc_activation.ino` — LED ring should light up in the idle animation
6. Open `tools/dev-tool.html` in Chrome/Edge, **Connect**, and **Simulate Tap** — confirm reading → success/failure animations play
7. Tap a real NTAG215 card — confirm `[scan]` line appears with the UID

If you're cloning a known-good config to a new board: **Import** `tools/led-states.json`, then **Flash States to Board**.

---

## Known Issues

- **LED flickering on some rings** — ESP32 3.3V logic may not cleanly drive WS2812B data lines. Workarounds:
  - Add a 330Ω resistor in series on the data line
  - Use a 3.3V→5V level shifter for production (recommended)
  - Power the ring from 3.3V temporarily (dimmer but stable for testing)
