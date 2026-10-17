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
(Wi-Fi, BLE, infrared, with nRF24 and NFC planned). A **browser dashboard**
talks to the board over **USB Web Serial** and drives those modules live.

The long-term hardware vision is a dual-node design: the ESP32-S3 core engine
plus an **M5StickC Plus2** handheld command terminal (its own screen, buttons,
battery, IR LED) talking to the core over UART / ESP-NOW. The M5Stick terminal
firmware is **not built yet** — today everything is driven from the browser
dashboard over USB.

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
│   ├── unified_firmware/       ⭐ THE firmware to flash — every module in one JSON loop
│   │   └── unified_firmware.ino
│   ├── 01_wifi/                Standalone per-module sketches 1.1–1.6 (reference)
│   ├── 02_ble/                 Standalone per-module sketches 2.1–2.6 (reference)
│   ├── 04_2.4ghz_nrf24/        nRF24 modules 4.1–4.6 — folders only, NOT implemented
│   ├── 05_nfc_pn532/           PN532 modules 5.1–5.6 — folders only, NOT implemented
│   ├── 07_infrared/            Standalone IR sketches 7.1–7.6 (reference)
│   └── 00_integrated_cyberdeck_os/
├── hardware/                   3d-models/ · schematics/ (placeholders)
├── docs/                       Documentation + README images (banner.svg)
├── data_dumps/                 pcap_captures/ · rfid_dumps/ (capture output)
└── .github/workflows/
    └── pages.yml               Deploys dashboard/ to GitHub Pages
```

The per-module folders under `firmware/01_*`, `02_*`, `07_*` are the **original
standalone sketches**, kept as reference. The firmware that actually runs on the
board is `firmware/unified_firmware/unified_firmware.ino` — it merges those
modules behind one command loop. When you add a capability, port it **into the
unified firmware**; the standalone sketch is just where the logic was first
proven.

---

## 4. Firmware deep dive (`unified_firmware.ino`)

Single-file Arduino sketch. Structure, top to bottom:

- **Pin map** (edit here if wiring differs):
  - IR receiver `IR_RX_PIN = GPIO4`, IR transmitter `IR_TX_PIN = GPIO5`.
    GPIO4/5 are plain GPIOs on the S3 (not strapping, not USB, not flash/PSRAM).
  - **Reserved for future modules** (documented in-code so they don't collide):
    nRF24 SPI `CE=6 CSN=7 SCK=12 MISO=13 MOSI=11`; PN532 I2C `SDA=8 SCL=9`.
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
- **Adafruit PN532** + **Adafruit BusIO** — for the PN532 NFC modules (5.x).
  The sketch includes `<Adafruit_PN532.h>`, so both must be installed to compile.
- **RF24** (TMRh20) — for the 2.4 GHz nRF24 modules (4.x). The sketch includes
  its headers, so it must be installed to compile.
- ESP32 BLE + Wi-Fi come with the ESP32 Arduino core.

Target board: **ESP32-S3** (dev notes assume N16R8 / N8R8 octal-PSRAM).

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

### Subsystem 5 · PN532 NFC — 🧪 firmware + dashboard, HARDWARE UNTESTED
PN532 on I2C: `SDA=8 SCL=9` (wire the breakout in I2C mode; RSTPDN→3.3 V). Lazy
init via `ensurePn532()` (`Wire.begin(8,9)` + `nfc.begin()` +
`getFirmwareVersion()`), which emits `{"status":"error","data":{"msg":"pn532_not_found"}}`
if the reader is missing — so 5.1 doubles as the wiring check. Full-card reads
(5.2/5.5) walk **one block/page per `loop()` pass** to stay non-blocking, and
re-select the card after a MIFARE Classic auth failure.

| id | name | what it does | emits (`data`) |
|----|------|--------------|----------------|
| 5.1 | Tag detection | polls for ISO 14443A tags, reports UID on arrival/removal | `uid, uid_len, type` |
| 5.2 | Block read | walks the card block-by-block (Classic default key A / UL pages) | `block, data, ok` |
| 5.3 | Memory structure | identifies card family + layout | `uid, type, sectors, blocks, bytes` |
| 5.4 | NDEF read | reads + decodes the first NDEF record (Text/URI); read-only | `action, record, tnf, type, payload` |
| 5.5 | Dump storage | block-by-block dump with a running captured count | `stored, block, data` |
| 5.6 | Card emulation | PN532 target mode, fixed demo identity (never a cloned card) | `uid, mode, status` |

5.4 is read-only in the unified firmware; NDEF **writing** stays a compile-time
option (`NDEF_WRITE`) only in the standalone sketch, so a START command over USB
can never overwrite a tag. 5.6 presents a fixed demo UID as a self-test. Each
standalone sketch also lives under `firmware/05_nfc_pn532/5.x_*/`.

### Not implemented yet (folders exist, logic does not)
- **M5StickC Plus2 command terminal** firmware (screen UI, buttons, IR blaster,
  ESP-NOW telemetry render).
- **CC1101 sub-GHz** and **RDM6300 125 kHz RFID** are **out of scope** for this
  hardware revision — do not re-add them.

---

## 6. Dashboard deep dive (`dashboard/`)

Static site, no build step. Plain HTML/CSS/JS. **Chrome or Edge only** (Web
Serial API; unavailable in Firefox/Safari). It talks only to the board on the
user's own machine — nothing is sent to any server.

- `index.html` — welcome/landing page. Hero is an animated schematic ESP32-S3
  board (`assets/js/board.js`, using vendored anime.js) with six pulsing signal
  lines: Wi-Fi, M5StickC Plus2, BLE, NFC, NRF24, IR.
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
  - `IR_VIEWS` — per-module table headers + row renderer (IR has custom views;
    Wi-Fi and BLE use generic renderers keyed off the `1.`/`2.` prefix).
  - `GROUP` / `ACCENT` — maps a module id prefix to a subsystem accent colour
    (`--wifi`/`--ble`/`--ir`/`--nrf`/`--nfc`).
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

## 10. Current state & what's next (as of the IR-testing phase)

**Done:** Wi-Fi (1.1–1.6) and BLE (2.1–2.6) implemented and tested; IR (7.1–7.6)
implemented in firmware + dashboard but **not yet tested on hardware**; dashboard
redesigned (Blueprint) and deployed to GitHub Pages; README refreshed.

**Immediate goal:** get the IR modules **tested on real hardware** end-to-end
(flash unified firmware → open dashboard → exercise 7.1–7.6 with an IR
receiver/transmitter and a remote). Wiring: RX S→GPIO4, middle→3.3 V, minus→GND;
TX LED on GPIO5.

**Next up:**
- nRF24 (subsystem 4): all six modules (4.1–4.6) are implemented in the unified
  firmware + dashboard and as standalone sketches (hardware-untested, SPI `CE=6
  CSN=7 SCK=12 MISO=13 MOSI=11`). Needs flashing + an nRF24L01+ to verify.
- PN532 NFC (subsystem 5): all six modules (5.1–5.6) are implemented in the
  unified firmware + dashboard and as standalone sketches (hardware-untested,
  I2C `SDA=8 SCL=9`). Needs flashing + a PN532 (plus the Adafruit PN532 / BusIO
  libraries) and a tag to verify.
- Build the M5StickC Plus2 command-terminal firmware (screen UI + ESP-NOW/UART
  link to the core).

For a longer, narrative history of decisions and milestones, see the project
documentation files kept with the project (context/architecture + progress/
milestones write-ups).
