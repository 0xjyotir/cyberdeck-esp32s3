# AGENTS.md — Cyberdeck ESP32-S3

Guide for any AI coding agent (Claude, Cursor, Copilot, etc.) picking up this
project cold. It explains what the project is, how the pieces fit together, the
conventions the owner expects, and exactly where the work currently stands so
you can continue without re-discovering everything.

> This file is for agents. End users read `README.md`. If you change how the
> project works, update **both** this file and `README.md`.

---

## 1. What this project is

An open-source, pocket-size **ESP32-S3 cyberdeck** for wireless and physical
security auditing — strictly for learning and **authorized** testing. One
ESP32-S3 runs a single "unified firmware" that exposes several auditing modules
(Wi-Fi, BLE, nRF24 and infrared). A **browser dashboard**
talks to the board over **USB Web Serial** and drives those modules live.

The hardware vision is a single self-contained deck: the ESP32-S3 plus a
**2.8" SPI TFT touchscreen** (assumed ILI9341 240×320 + XPT2046 touch) for
picking modules and reading their output without a laptop, and an **RC522
13.56 MHz RFID reader**. The earlier plan for an M5StickC Plus2 command terminal
is **dropped**. The RC522 RFID modules (5.1–5.6) are now in the unified firmware
and dashboard.

The project has entered the **on-deck stage**: the 2.8" TFT is working on
hardware, and the **integrated cyberdeck firmware**
(`firmware/00_integrated_cyberdeck_firmware/`) drives the deck from its own
touchscreen. Going forward that is the main ("unified") firmware the owner
flashes. **It is not at parity yet:** today it carries Wi-Fi (1.x) and BLE
(2.x) on-deck; nRF24, RFID and IR are still only in the USB-only
`firmware/unified_firmware/`, which stays in the repo until they are ported.

### Ethical scope (do not ignore)
This is defensive / educational security tooling. Only ever add capabilities
for auditing networks and devices the operator **owns or is authorized to
test**. Keep that framing in code comments, the README, and any UI copy. Do not
add features whose only purpose is covert attack, mass targeting, or evading
detection.

---

## 2. The two pieces, and how they talk

```
┌────────────────────────────┐   USB serial, JSON @ 115200    ┌─────────────────────────┐
│  ESP32-S3                   │  host → {module, action}       │  Browser dashboard      │
│  firmware/unified_firmware  │ ─────────────────────────────▶ │  dashboard/ (Web Serial)│
│  one JSON command loop      │  board → {module,status,data}  │  Chrome / Edge only     │
└────────────────────────────┘ ◀───────────────────────────── └─────────────────────────┘
```

### Serial protocol (LOCKED — do not change the shape)
- **Transport:** USB CDC serial, **115200 baud**, newline-delimited JSON, one
  JSON object per line.
- **Host → board (command):**
  ```json
  {"module":"7.4","action":"START"}
  {"action":"STOP"}
  ```
  `action` is `"START"` or `"STOP"`. `module` is a dotted id like `"1.1"`,
  `"2.6"`, `"7.4"`. `STOP` needs no module (it halts whatever is active).
- **Board → host (telemetry):**
  ```json
  {"module":"7.4","status":"ok","data":{"protocol":"NEC","value":"20DF10EF","bits":32}}
  ```
  `status` is `"ok"` or `"error"`. `data` is a per-module object. Errors use
  `{"module":"7.3","status":"error","data":{"msg":"no_code_captured"}}`.
  System-level parse errors come back as module `"SYS"`
  (e.g. `malformed_json`, `unknown_action`).

Every module emits the same envelope via one `emitTelemetry()` helper in the
firmware, and the dashboard parses every line with that same shape. If you add a
module, keep the envelope identical and only vary the `data` fields.

---

## 3. Repository layout

```text
cyberdeck-esp32s3/
├── AGENTS.md                   ← this file
├── README.md                   End-user facing overview + live dashboard link
├── dashboard/                  Browser UI, auto-deployed to GitHub Pages
│   ├── index.html              Welcome / landing page (animated board hero)
│   ├── app.html                The dashboard itself (sidebar, telemetry table, serial console)
│   └── assets/
│       ├── css/                base.css (design tokens) · welcome.css · dashboard.css
│       ├── js/                 board.js (hero animation) · dashboard.js (Web Serial + modules) · resize.js (draggable panes) · spectrum.js (4.1 graph)
│       └── vendor/             anime.min.js (vendored; do not fetch from a CDN)
├── firmware/
│   ├── 00_integrated_cyberdeck_firmware/  ⭐ The on-deck firmware: TFT touch UI + Wi-Fi/BLE + same JSON loop
│   │   └── 00_integrated_cyberdeck_firmware.ino
│   ├── unified_firmware/       USB-only firmware with ALL 5 subsystems (kept until nRF24/RFID/IR are ported on-deck)
│   │   └── unified_firmware.ino
│   ├── 01_wifi/                Standalone per-module sketches 1.1–1.6 (reference)
│   ├── 02_ble/                 Standalone per-module sketches 2.1–2.6 (reference)
│   ├── 04_2.4ghz_nrf24/        Standalone nRF24 sketches 4.1–4.6 (reference)
│   ├── 05_rfid_rc522/          Standalone RC522 sketches 5.1–5.6 (reference)
│   └── 07_infrared/            Standalone IR sketches 7.1–7.6 (reference)
├── hardware/                   3d-models/ · schematics/ (placeholders)
├── docs/                       Documentation + README images (banner.svg)
├── data_dumps/                 pcap_captures/ (capture output)
└── .github/workflows/
    └── pages.yml               Deploys dashboard/ to GitHub Pages
```

The per-module folders under `firmware/01_*`, `02_*`, `07_*` are the **original
standalone sketches**, kept as reference. Two merged firmwares sit on top of them:

- `firmware/00_integrated_cyberdeck_firmware/` — **the main firmware from now on.**
  The deck is navigated and used from the 2.8" TFT; the USB dashboard still
  works at the same time over the same protocol. See §4b.
- `firmware/unified_firmware/` — the USB-only build with every module (§4). It
  remains the way to run nRF24 / RFID / IR until those are ported on-deck.

When you add a capability, put it **into the integrated firmware** (and keep the
unified firmware working until the integrated one reaches parity); the
standalone sketch is just where the logic was first proven.

---

## 4. Firmware deep dive (`unified_firmware.ino`)

Single-file Arduino sketch. Structure, top to bottom:

- **Pin map** (edit here if wiring differs):
  - IR receiver `IR_RX_PIN = GPIO4`, IR transmitter `IR_TX_PIN = GPIO5`.
    GPIO4/5 are plain GPIOs on the S3 (not strapping, not USB, not flash/PSRAM).
  - nRF24 SPI `CE=6 CSN=7 SCK=12 MISO=13 MOSI=11`.
  - RC522 RFID (subsystem 5) on the **shared nRF24 SPI bus** (`SCK=12 MISO=13
    MOSI=11`) with its own chip-select: `SS=10 RST=9`, 3.3 V only, IRQ unused.
    Only one module is ever active, so the shared bus is safe.
  - The 2.8" TFT is **not** in this firmware; its locked pins (§4b) collide
    with the RC522/nRF24/IR pins above. When those subsystems join the
    integrated firmware, **they** move; the display never does.
  - ⚠️ **Never use GPIO35/36/37** — on octal-PSRAM boards (N8R8 / N16R8) those
    pins are wired to the PSRAM and are unusable.
- **Global state** — `activeModule` (a `String` like `"1.1"`, or `"NONE"`),
  millis-based timers, BLE scan/advertising handles, IR capture buffers.
- **`emitTelemetry()` / `emitError()`** — the one place telemetry is serialized.
- **`startModule()` / `stopModule()`** — all one-time hardware setup/teardown
  lives here, never in the loop. `startModule()` stops the previous module
  first so shared hardware (Wi-Fi mode, BLE scan, the IR TX pin's LEDC channel,
  the IRrecv interrupt) is released before the next module grabs it.
- **`handleSerialCommand()`** — reads one line, parses JSON, dispatches
  START/STOP.
- **`loop()`** — a big `if (activeModule == …)` chain. Each module does its
  periodic work gated by `millis()`, never `delay()`.

### Non-blocking discipline (important)
`loop()` must stay fast so serial commands (including STOP) are always serviced.
All polling is `millis()`-gated. Two known, bounded exceptions, already handled:
- BLE scanning runs as a **continuous async scan** (duration 0) with results
  delivered via callback — the loop doesn't block on scan results.
- `BLEClient::connect()` (GATT phases of 2.2 / 2.5 / 2.6) is **blocking inside
  the ESP32 BLE library itself** (~1–3 s per attempt); there's no non-blocking
  variant without moving BLE to its own FreeRTOS task (out of scope). Module 2.6
  was fixed to do **one** connect attempt per `loop()` pass instead of five
  back-to-back, so STOP is checked between attempts.

If you add modules, keep this rule: no unbounded `delay()` / `while()` in
`loop()`. One-shot work (like IR replay) sets `activeModule = "NONE"` when done.

### Arduino libraries required
- **ArduinoJson** v7.x (Benoit Blanchon)
- **IRremoteESP8266** (David Conran / crankyoldgit) — only for IR decode/replay
  (7.3 / 7.4), but the sketch includes its headers, so it must be installed to
  compile.
- **RF24** (TMRh20) — for the 2.4 GHz nRF24 modules (4.x). The sketch includes
  its headers, so it must be installed to compile.
- **MFRC522** (miguelbalboa) — for the RC522 RFID modules (5.x). The sketch
  includes its headers, so it must be installed to compile.
- ESP32 BLE + Wi-Fi come with the ESP32 Arduino core.

Target board: **ESP32-S3** (dev notes assume N16R8 / N8R8 octal-PSRAM).

---

## 4b. Integrated cyberdeck firmware (`00_integrated_cyberdeck_firmware.ino`)

Single self-contained `.ino` (the owner wants no extra headers or zips). The
Wi-Fi/BLE module code (`startModule` / `stopModule` / `handleSerialCommand` /
`runActiveModule` / BLE callbacks) is copied **verbatim** from
`unified_firmware`, and `emitTelemetry()` calls a `uiOnTelemetry()` hook that
turns each telemetry line into an on-screen row, so the screen and the USB
dashboard always agree. A module started from the dashboard makes the screen
jump to its live view (`syncScreenToSerial()`).

- **Hardware:** 2.8" ILI9341 (320×240, rotation 1) + XPT2046 touch.
- **Display pins — LOCKED by the owner, never change them:** `CS=10 DC=9 RST=8
  LED/BL=7 T_CS=4 T_IRQ=5`, shared SPI `SCK=12 MOSI=11 MISO=13`, VCC 3.3 V.
  If other subsystems are added to this board, rewire **them** around the display.
- **Libraries:** Adafruit ILI9341 + Adafruit GFX + Adafruit BusIO, ArduinoJson
  v7. Fonts come from Adafruit GFX's bundled `Fonts/` folder (FreeSans9pt7b,
  FreeSansBold9pt7b) — don't add external font files. **Not** TFT_eSPI (white
  screen on core 3.3.x) and **not** the XPT2046_Touchscreen library.
- **Board settings:** ESP32S3 Dev Module, Flash 16MB, PSRAM "OPI PSRAM", USB CDC
  On Boot Enabled.
- **Touch:** custom XPT2046 driver inside the sketch (8-sample trimmed mean,
  pressure hysteresis, DOWN/MOVE/UP events polled every 5 ms) and a 5-point
  affine calibration saved in `Preferences` (runs on first boot, or hold the
  screen at power-on, or Setup → Recalibrate). Buttons highlight on DOWN and
  act on UP.
- **Screens:** home (subsystem tiles) → module list → live view (START/HALT,
  scrolling result rows, status line), plus Setup, touch test and calibration.
  nRF24 / RFID / IR tiles say "Not on deck" and open a note pointing at
  `unified_firmware`.
- **Theme:** dark "industrial" palette (`C_BG`/`C_PANEL`/`C_RAISED`/`C_EDGE`
  surfaces, `C_TEXT`/`C_MUTED`/`C_DIM` text, one accent per subsystem). Use
  `printAt()` for the classic 6×8 font (it resets the GFX font) and `fontAt()`
  for FreeSans.
- **Non-blocking:** redraws are `millis()`-gated (live list at most ~8×/s). The
  same BLE caveat applies: GATT modules 2.2/2.5/2.6 freeze the UI ~1–3 s per
  connect attempt.
- **Checking layout without hardware:** compile the drawing code against the
  real Adafruit GFX library with a framebuffer subclass and dump PNGs (the
  owner's project files keep a small render harness for this).

---

## 5. The modules (implemented set)

Everything below is wired in **both** the unified firmware **and** the
dashboard. `module` id → what it does → key `data` fields it emits.

### Subsystem 1 · Wi-Fi (1.1–1.6) — ✅ firmware + dashboard
| id | name | emits (`data`) |
|----|------|----------------|
| 1.1 | Active Wi-Fi scan (async) | `ssid, bssid, ch, rssi` per network |
| 1.2 | Wardriving / channel monitor | `ssid, bssid, ch, rssi` |
| 1.3 | Promiscuous sniffer (raw 802.11 count) | `ssid, bssid, ch, rssi` |
| 1.4 | Network / client discovery | `ssid, bssid, ch, rssi` |
| 1.5 | AP hosting (`Cyberdeck_AP`) | softAP IP in `ssid` |
| 1.6 | Frame lab / injection test | tx status in `ssid` |

### Subsystem 2 · BLE (2.1–2.6) — ✅ firmware + dashboard
| id | name | emits (`data`) |
|----|------|----------------|
| 2.1 | BLE scan & fingerprint | `name, addr, rssi, uuid` |
| 2.2 | GATT service enumeration | `name, uuid, rssi, payload` |
| 2.3 | Advertisement sniffer | `name, addr, rssi, uuid` |
| 2.4 | Custom beacon broadcaster | `name, uuid, rssi, payload` |
| 2.5 | GATT exploration + value read | `name, uuid, payload` (hex) |
| 2.6 | Device connection stress test | `name, uuid, rssi, payload` (latency) |

### Subsystem 7 · Infrared (7.1–7.6) — 🧪 firmware + dashboard, HARDWARE UNTESTED
IR RX on GPIO4, TX on GPIO5. Receiver wiring: **S → GPIO4, middle → 3.3 V,
minus → GND**.

| id | name | what it does | emits (`data`) |
|----|------|--------------|----------------|
| 7.1 | 38 kHz carrier | continuous 38 kHz / 50 % duty on TX (LEDC). Verify with a phone camera. | `freq_hz, duty_pct, pin` |
| 7.2 | Pulse learning | captures raw mark/space timings of each IR burst | `count, overflow, timings_us[]` |
| 7.3 | Signal replay | one-shot resend of the last 7.2/7.4/7.5 capture (raw @ 38 kHz if protocol unknown), then idles | `protocol, value, bits`/`count, action` |
| 7.4 | Protocol decode | decodes NEC / Sony / RC5 / RC6 / … | `protocol, value, bits` |
| 7.5 | Signal analysis | mark/space totals + duty cycle per capture | `pulses, mark_us, space_us, total_us, duty_pct, protocol` |
| 7.6 | LED circuit test | fires a 26 µs pulse every 10 ms to check the TX driver | `pulses, pulse_us, period_ms, pin` |

**Pin sharing:** 7.1/7.3/7.6 all drive `IR_TX_PIN`; 7.2/7.4/7.5 share the one
`IRrecv` on `IR_RX_PIN`. Only one module is ever active, and `startModule()`
tears down the previous one first, so the LEDC channel is always detached before
`IRsend`/`digitalWrite` take the TX pin back.

### Subsystem 4 · nRF24 (2.4 GHz) — 🧪 firmware + dashboard, HARDWARE UNTESTED
nRF24L01+ on SPI: `CE=6 CSN=7 SCK=12 MISO=13 MOSI=11`. Lazy init via
`ensureNrf()`, which runs `radio.begin()` + `radio.isChipConnected()` and emits
`{"status":"error","data":{"msg":"nrf_not_found"}}` if the module is missing —
so 4.1 doubles as the wiring check.

| id | name | what it does | emits (`data`) |
|----|------|--------------|----------------|
| 4.1 | Spectrum scan | sweeps channels 0–125 (2400–2525 MHz), 32 RPD samples/channel/pass | `ch, mhz, level` (0–32) |
| 4.2 | Packet sniffer | listens on the default pipe, hops ch 0–83, dumps payloads as hex | `ch, addr, len, payload` |
| 4.3 | Address discovery | 2-byte preamble sniff, hops the band, reports candidate pipe addresses | `ch, addr, hits` |
| 4.4 | Protocol analysis | per-packet length + data rate + heuristic control/data guess | `ch, addr, len, rate, guess` |
| 4.5 | Packet storage | in-RAM ring buffer (32 frames) of captures, running count | `stored, last_addr, last` |
| 4.6 | Lab transmitter | low-power test TX to your own node (link/range test) | `ch, addr, sent, ack` |

Only one module active at a time; `stopModule()` powers the radio down. 4.6 is a
self-test transmitter for a node you own — not for injecting into other devices.
Each standalone sketch also lives under `firmware/04_2.4ghz_nrf24/4.x_*/`.

### Subsystem 5 · RFID (RC522, 5.1–5.6) — ✅ firmware + dashboard
RC522 13.56 MHz (ISO-14443A) reader on the SPI bus **shared with the nRF24**
(`SCK=12 MISO=13 MOSI=11`) with its own chip-select (`SS=10 RST=9`, 3.3 V only,
IRQ unused). Lazy init via `ensureRc522()`, which runs `PCD_Init()` + a version-
register check and emits `{"status":"error","data":{"msg":"rc522_not_found"}}`
if the reader is missing — so 5.1 doubles as the wiring check. Authorized /
educational use only — read/inspect cards you own or are permitted to test.

| id | name | what it does | emits (`data`) |
|----|------|--------------|----------------|
| 5.1 | Card fingerprint | UID / ATQA / SAK + derived card type; the RC522 wiring check | `uid, uid_len, atqa, sak, type` |
| 5.2 | Memory dump | read-only per-block dump with the default key (Classic) / pages (UL/NTAG) | `sector, block, data, key_used, auth` + `blocks_read, blocks_locked, done` |
| 5.3 | Default-key audit | dictionary of common keys per sector → A–F grade (pure read/auth) | `sector, result, key_found` + `weak_sectors, total_sectors, grade, done` |
| 5.4 | Clone to magic card | read an owned source, write to an owned magic card, verify | `stage, uid, blocks_cached` / `blocks_written, verify, target_type` |
| 5.5 | NDEF read / write | decode first URI/Text record (write behind a compile switch) | `record_type, payload, tag_type, capacity` / `bytes_written, verify` |
| 5.6 | Reader monitor | passive log of every card presented, ring buffer + distinct count | `uid, type, t_ms, seen_count` + `distinct, total_reads, summary` |

**Safety gates (compile-time, OFF by default — identical to the standalone
sketches):** `RFID_OPERATOR_OWNS_CARDS 0` makes 5.4 read the source and report
but refuse every write with `ownership_not_acknowledged`; even set to 1 it
refuses non-magic targets and never writes a sector trailer. `RFID_NDEF_MODE_WRITE 0`
keeps 5.5 read-only. Only one module active at a time; `stopModule()` halts the
PICC and releases Crypto1 (5.6 emits a final distinct/total summary on stop). The
per-card operations in 5.2/5.3/5.4 run to completion in one `loop()` pass once a
card is detected — bounded, one-shot work, the same category as the BLE connect
exception. Each standalone sketch also lives under
`firmware/05_rfid_rc522/5.x_*/`.

### Not implemented yet (folders exist, logic does not)
- **nRF24 / RFID / IR on-deck:** not yet in the integrated firmware (§4b). Needs
  their pins moved off the display's locked pins first, then the module code
  ported and a screen view per module. Must keep `loop()` non-blocking.
- The **M5StickC Plus2** terminal is dropped; don't build for it.
- **CC1101 sub-GHz**, **RDM6300 125 kHz RFID** and **PN532 NFC** are **out of
  scope** for this hardware revision — do not re-add them. (PN532 was removed
  after the test board turned out faulty.)

---

## 6. Dashboard deep dive (`dashboard/`)

Static site, no build step. Plain HTML/CSS/JS. **Chrome or Edge only** (Web
Serial API; unavailable in Firefox/Safari). It talks only to the board on the
user's own machine — nothing is sent to any server.

- `index.html` — welcome/landing page. Hero is an animated schematic ESP32-S3
  board (`assets/js/board.js`, using vendored anime.js) with six pulsing signal
  lines: Wi-Fi, 2.8" TFT, BLE, NRF24, IR, RC522 RFID. Six subsystem cards
  (Wi-Fi, BLE, IR, nRF24, RFID, on-deck screen) sit in an even 3×2 grid, with
  "The point" as a full-width strip (`.card-wide`) beneath them; keep the count
  at six so the grid stays even. Below the subsystem cards is
  **the field manual** (`#guide`): a per-function reference for all 30 implemented
  functions (including the six RFID 5.x functions), grouped by subsystem as native
  `<details>` accordions. Each entry
  explains what the function does, defines every telemetry `data` field, shows an
  example telemetry line, and links to that function's own standalone `.ino` on
  GitHub (not the unified firmware). Pure HTML/CSS, no new JS. When you add or
  change a module, add or update its accordion here too.
- `app.html` — the dashboard: collapsible subsystem sidebar, a live telemetry
  table, and a raw serial console. "Connect USB serial" → pick a module → "Start
  module" / "Halt".
- `assets/js/resize.js` — standalone pane resizing. Two `.resizer` handles in
  `app.html` (sidebar↔main, telemetry↔serial) set `.sidebar` width and
  `.panel-serial` height inline. A pane being enlarged is capped at **75%** of
  its container along that axis; re-clamped on window resize; sizes persist in
  `localStorage` (`cyberdeck.panes`); double-click / Enter resets; arrow keys
  nudge. The column handle is hidden ≤820px (sidebar stacks). Doesn't touch
  `dashboard.js`.
- `assets/js/spectrum.js` — canvas spectrum view for nRF24 **4.1** (Bruce-style
  per-channel bars, colour by level, peak hold, Wi-Fi 1/6/11 bands, waterfall
  of the last 60 sweeps, hover tooltip). `window.Spectrum.{push,reset,show,note}`;
  `dashboard.js` feeds it `{ch, level}` and hides the table while 4.1 is
  active. Status/error lines (e.g. `nrf_not_found`) show as a note above it.
  Raw JSON still prints in the serial console.
- **Clear buttons** — telemetry header (`clearTelemetry()`, also resets the
  spectrum) and serial header (`clearConsole()`). View-only; nothing is sent
  to the board.
- `assets/js/dashboard.js` — the Web Serial bridge and all module metadata. Key
  objects to edit when adding a module:
  - `TITLES` / `DESCS` — the module title + description text.
  - `IR_VIEWS` / `NRF_VIEWS` / `RFID_VIEWS` — per-module table headers + row
    renderer (IR, nRF24 and RFID have custom views; Wi-Fi and BLE use generic
    renderers keyed off the `1.`/`2.` prefix). `updateTelemetry()` also maps a
    few RFID status codes (`rc522_not_found`, `ownership_not_acknowledged`, …) to
    friendly lines.
  - `GROUP` / `ACCENT` — maps a module id prefix to a subsystem accent colour
    (`--wifi`/`--ble`/`--ir`/`--nrf`/`--rfid`).
  - To add a module button, add a `<button class="mod-btn" data-mod="X.Y"
    onclick="switchModule('X.Y', this)">` in the right `<div class="sub-list">`
    in `app.html`.

### Design system (owner's locked choices — keep them)
"Blueprint" style: **navy grid paper + cyan** palette (tokens live in
`assets/css/base.css`). Hero headline font **Syne**; **all other text**
`Anthropic Sans` with **Hanken Grotesk** fallback (Anthropic Sans isn't publicly
licensed so it can't be bundled — Hanken Grotesk is what actually loads, via
Google Fonts). Serial console stays **monospace** (JetBrains Mono, `--code`).
Hero board = dashed schematic with six pulsing lines, **no antenna rings/
squiggles**. The dashboard **layout** must stay as-is — restyle only, don't
re-architect it.

---

## 7. Hosting / deploy

Live site: **https://rronium.github.io/cyberdeck-esp32s3/** (welcome page; "Try
it" opens the dashboard).

`.github/workflows/pages.yml` deploys `dashboard/` to GitHub Pages on every push
to `main` that touches `dashboard/**` (or the workflow file), plus manual
`workflow_dispatch`. One-time setup the repo owner must do: **Settings → Pages →
Source: GitHub Actions**.

> Agents in cloud sandboxes usually **can't reach `github.io`** to verify the
> live site. Check a deploy via the **Actions run log** for the "Deploy
> dashboard to GitHub Pages" workflow instead of trying to curl the page.

---

## 8. Build / test / run

- **Firmware:** opened and flashed from the **Arduino IDE** (ESP32-S3 board
  selected, ArduinoJson + IRremoteESP8266 installed). The **Arduino toolchain
  cannot be downloaded in a cloud sandbox**, so an agent working remotely
  **cannot compile the firmware** — hand compile/flash back to the repo owner
  and describe exactly what to check. Review `.ino` changes by reading, not
  building.
- **Dashboard:** no build. Open `dashboard/index.html` in Chrome/Edge, or use
  the live Pages URL. Testing the serial path needs a physical ESP32-S3 on USB.
- **No automated test suite / linter** in the repo today. "Verification" is
  manual: flash, connect the dashboard, exercise the module, read telemetry.

---

## 9. Working conventions (the repo owner's rules)

These are firm — follow them unless the owner says otherwise:

1. **Never commit directly to `main`.** Always work on a branch and **open a
   pull request**.
2. **Show a preview and wait for the owner's explicit "yes" before pushing** any
   change. Draft first, get sign-off, then push.
3. **Every module's full functionality** belongs in **both** the unified
   firmware and the dashboard — not a subset. Firmware and dashboard ship
   together.
4. **Don't break the serial protocol** (§2). The envelope is locked; only add
   `data` fields.
5. **Keep firmware `loop()` non-blocking** (§4).
6. Keep the dashboard **design tokens and layout** intact (§6) — restyle, don't
   redesign, unless asked.
7. Update **both** `README.md` and this `AGENTS.md` when behaviour changes.
8. PR descriptions: open with a plain-language "Before:" / "After:" pair, then a
   one-line "what it does", then a short "How".

---

## 10. Current state & what's next (as of the integrated on-deck firmware stage)

**Done:** Wi-Fi (1.1–1.6) and BLE (2.1–2.6) implemented and tested; IR (7.1–7.6)
implemented in firmware + dashboard but **not yet tested on hardware**; dashboard
redesigned (Blueprint) and deployed to GitHub Pages; README refreshed. 2.8" TFT +
touch working on hardware; integrated cyberdeck firmware (§4b) runs Wi-Fi + BLE
from the touchscreen with a dark industrial UI (uncompiled in the cloud; the
owner flashes and tests).

**Immediate goal:** get the IR modules **tested on real hardware** end-to-end
(flash unified firmware → open dashboard → exercise 7.1–7.6 with an IR
receiver/transmitter and a remote). Wiring: RX S→GPIO4, middle→3.3 V, minus→GND;
TX LED on GPIO5.

**Next up:**
- nRF24 (subsystem 4): all six modules (4.1–4.6) are implemented in the unified
  firmware + dashboard and as standalone sketches (hardware-untested, SPI `CE=6
  CSN=7 SCK=12 MISO=13 MOSI=11`). Needs flashing + an nRF24L01+ to verify.
- RC522 RFID (subsystem 5): all six modules (5.1–5.6) are now implemented in the
  unified firmware + dashboard and as standalone sketches. Ron has verified the
  RC522 on hardware (5.2 confirmed working); the unified-firmware port is
  uncompiled in the cloud (no Arduino toolchain) and needs a flash + a card to
  exercise 5.1–5.6 end-to-end. Clone (5.4) writes and NDEF (5.5) writes stay
  gated off by default.
- Integrated firmware UI: 3×3 home grid with scroll arrows, a Settings tile
  (brightness, light/dark mode), and an "i" info article for every function.
- Port nRF24 / RFID / IR into the integrated firmware (after re-pinning them
  around the locked display pins), then retire `unified_firmware`.

For a longer, narrative history of decisions and milestones, see the project
documentation files kept with the project (context/architecture + progress/
milestones write-ups).
