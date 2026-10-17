/* Cyberdeck dashboard logic — Web Serial bridge + module switching + telemetry
   rendering. Ported unchanged from the original single-file dashboard; only the
   DOM wiring (class names, accent colour per subsystem) was updated to match the
   restyled markup. Serial protocol is identical: JSON at 115200, host sends
   {module, action:"START"|"STOP"}, board replies {module, status, data:{...}}. */
(function () {
  let port, reader, inputDone;
  let currentModule = "1.1";

  const $ = (id) => document.getElementById(id);

  /* ---- subsystem accent, used for the module panel + active sidebar item ---- */
  const GROUP = (m) => m.startsWith("1.") ? "wifi" : m.startsWith("2.") ? "ble"
    : m.startsWith("4.") ? "nrf" : m.startsWith("5.") ? "rfid" : m.startsWith("7.") ? "ir" : "wifi";
  const ACCENT = { wifi: "--wifi", ble: "--ble", ir: "--ir", nrf: "--nrf", rfid: "--rfid" };

  /* ---- collapsible subsystems ---- */
  window.toggleSub = function (btn) {
    const list = document.getElementById(btn.getAttribute("aria-controls"));
    const open = btn.getAttribute("aria-expanded") === "true";
    btn.setAttribute("aria-expanded", String(!open));
    list.hidden = open;
  };

  /* ---- serial connection ---- */
  window.toggleSerial = async function () { port ? await disconnect() : await connect(); };

  async function connect() {
    if (!("serial" in navigator)) {
      appendLog("[-] Web Serial is not available in this browser. Use Chrome or Edge over HTTPS/localhost.", "err");
      return;
    }
    try {
      port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });
      setConnected(true);
      appendLog("USB serial port opened at 115200 baud.", "sys");
      readLoop();
    } catch (err) {
      appendLog("[-] Failed to open serial port: " + err, "err");
      port = null;
    }
  }

  async function disconnect() {
    if (reader) { await reader.cancel(); await inputDone.catch(() => {}); }
    if (port) { await port.close(); port = null; }
    setConnected(false);
    appendLog("Serial port closed.", "sys");
  }

  function setConnected(on) {
    const pill = $("conn-status"), btn = $("serial-btn"), dot = $("live-dot");
    if (on) {
      pill.textContent = "CONNECTED · 115200"; pill.classList.add("ok");
      btn.textContent = "Disconnect"; dot.classList.add("on");
    } else {
      pill.textContent = "DISCONNECTED"; pill.classList.remove("ok");
      btn.textContent = "Connect USB serial"; dot.classList.remove("on");
    }
  }

  async function readLoop() {
    const dec = new TextDecoderStream();
    inputDone = port.readable.pipeTo(dec.writable);
    reader = dec.readable.getReader();
    try {
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        if (value) handleIncoming(value);
      }
    } catch (e) { appendLog("[-] Stream read error: " + e, "err"); }
  }

  let lineBuffer = "";
  function handleIncoming(text) {
    lineBuffer += text;
    const lines = lineBuffer.split("\n");
    lineBuffer = lines.pop();
    for (let line of lines) {
      line = line.trim();
      if (!line) continue;
      appendLog(line);
      if (line.startsWith("{") && line.endsWith("}")) {
        try { updateTelemetry(JSON.parse(line)); } catch (e) {}
      }
    }
  }

  window.sendCommand = async function (action) {
    if (!port || !port.writable) {
      appendLog("[-] Connect your ESP32 over USB serial first.", "err");
      return;
    }
    const cmd = JSON.stringify({ module: currentModule, action }) + "\n";
    const writer = port.writable.getWriter();
    await writer.write(new TextEncoder().encode(cmd));
    writer.releaseLock();
    appendLog(cmd.trim(), "sent");
  };

  function appendLog(message, cls) {
    const box = $("console");
    const div = document.createElement("div");
    if (cls) div.className = cls;
    div.textContent = message;
    box.appendChild(div);
    box.scrollTop = box.scrollHeight;
  }

  /* ---- Clear buttons: view-only, the board keeps streaming ---- */
  const EMPTY_ROW = `<tr><td colspan="4" class="empty">Awaiting hardware telemetry stream…</td></tr>`;
  window.clearTelemetry = function () {
    $("telemetry-body").innerHTML = EMPTY_ROW;
    if (window.Spectrum) Spectrum.reset();
  };
  window.clearConsole = function () {
    $("console").innerHTML = "";
    appendLog("[system] Console cleared.", "sys");
  };

  /* ---- infrared telemetry views (unchanged) ---- */
  const dash = (v) => (v !== undefined && v !== null) ? v : "-";
  const hex = (v) => v ? "0x" + String(v).toUpperCase() : "-";
  const IR_VIEWS = {
    "7.1": { live: true, headers: ["Carrier", "Frequency", "Duty", "Pin"],
      row: d => ["ON", dash(d.freq_hz) + " Hz", dash(d.duty_pct) + " %", "GPIO" + dash(d.pin)] },
    "7.2": { headers: ["Pulses", "Overflow", "Total", "Timings (µs)"],
      row: d => {
        const t = d.timings_us || [];
        const total = t.reduce((a, b) => a + b, 0);
        const preview = t.slice(0, 16).join(", ") + (t.length > 16 ? ", …" : "");
        return [dash(d.count), d.overflow ? "YES" : "no", total + " µs", `<span title="${t.join(", ")}">${preview}</span>`];
      } },
    "7.3": { headers: ["Protocol", "Value", "Bits / Pulses", "Status"],
      row: d => [dash(d.protocol), hex(d.value), d.bits !== undefined ? d.bits + " bits" : dash(d.count) + " pulses", dash(d.action)] },
    "7.4": { headers: ["Protocol", "Value", "Bits", "Status"],
      row: d => [dash(d.protocol), hex(d.value), dash(d.bits), "captured"] },
    "7.5": { headers: ["Protocol", "Mark / Space", "Total", "Duty"],
      row: d => [dash(d.protocol), `${dash(d.mark_us)} / ${dash(d.space_us)} µs`, dash(d.total_us) + " µs",
        d.duty_pct !== undefined ? Number(d.duty_pct).toFixed(1) + " %" : "-"] },
    "7.6": { live: true, headers: ["Pulses Sent", "Pulse Width", "Period", "Pin"],
      row: d => [dash(d.pulses), dash(d.pulse_us) + " µs", dash(d.period_ms) + " ms", "GPIO" + dash(d.pin)] }
  };

  /* ---- nRF24 (2.4 GHz) telemetry views ---- */
  const NRF_VIEWS = {
    "4.1": { live: false, headers: ["Channel", "Frequency", "Level", "Signal"],
      row: d => {
        const lvl = Number(d.level) || 0;
        const pct = Math.min(100, Math.round(lvl / 32 * 100));
        return ["CH " + dash(d.ch), dash(d.mhz) + " MHz", lvl + " / 32",
          `<span style="display:inline-block;width:64px;height:6px;border-radius:3px;background:rgba(255,255,255,.12);vertical-align:middle"><i style="display:block;height:100%;border-radius:3px;width:${pct}%;background:var(--nrf)"></i></span>`];
      } },
    "4.2": { headers: ["Channel", "Address", "Length", "Payload"],
      row: d => ["CH " + dash(d.ch), dash(d.addr), dash(d.len) + " B", dash(d.payload)] },
    "4.3": { headers: ["Channel", "Candidate Address", "Hits"],
      row: d => ["CH " + dash(d.ch), dash(d.addr), dash(d.hits)] },
    "4.4": { headers: ["Channel", "Address", "Length", "Rate", "Guess"],
      row: d => ["CH " + dash(d.ch), dash(d.addr), dash(d.len) + " B", dash(d.rate), dash(d.guess)] },
    "4.5": { live: true, headers: ["Stored", "Last Address", "Last Payload"],
      row: d => [dash(d.stored), dash(d.last_addr), dash(d.last)] },
    "4.6": { live: true, headers: ["Channel", "Address", "Sent", "Ack"],
      row: d => ["CH " + dash(d.ch), dash(d.addr), dash(d.sent), d.ack ? "ACK" : "no ack"] }
  };

  /* ---- RC522 RFID (13.56 MHz) telemetry views ---- */
  const RFID_VIEWS = {
    "5.1": { headers: ["UID", "Card Type", "ATQA", "SAK"],
      row: d => [dash(d.uid), dash(d.type), dash(d.atqa), dash(d.sak)] },
    "5.2": { headers: ["Sector", "Block", "Data (hex)", "Auth"],
      row: d => d.done
        ? [`${dash(d.blocks_read)} read`, `${dash(d.blocks_locked)} locked`, "—", "DUMP DONE"]
        : ["S" + dash(d.sector), dash(d.block), d.auth === "ok" ? dash(d.data) : "—",
           d.auth === "ok" ? "default key" : "locked"] },
    "5.3": { headers: ["Sector", "Result", "Key Found"],
      row: d => d.done
        ? [`GRADE ${dash(d.grade)}`, `${dash(d.weak_sectors)}/${dash(d.total_sectors)} weak`, "AUDIT DONE"]
        : ["S" + dash(d.sector), d.result === "default_key" ? "default key" : "protected",
           dash(d.key_found)] },
    "5.4": { headers: ["Stage", "UID", "Blocks", "Verify / Type"],
      row: d => d.stage === "clone"
        ? ["clone", dash(d.uid), `${dash(d.blocks_written)} written`, `${dash(d.verify)} · ${dash(d.target_type)}`]
        : ["source read", dash(d.uid), `${dash(d.blocks_cached)} cached`, "present target"] },
    "5.5": { headers: ["Record", "Payload / Bytes", "Tag", "Verify"],
      row: d => d.bytes_written !== undefined
        ? [dash(d.record_type), `${dash(d.bytes_written)} B written`, "—", dash(d.verify)]
        : [dash(d.record_type), dash(d.payload), dash(d.tag_type), dash(d.capacity) + " B"] },
    "5.6": { headers: ["UID", "Card Type", "Time", "Seen"],
      row: d => d.summary
        ? [`${dash(d.distinct)} distinct`, `${dash(d.total_reads)} total`, "—", "SUMMARY"]
        : [dash(d.uid), dash(d.type),
           d.t_ms !== undefined ? (Number(d.t_ms) / 1000).toFixed(1) + " s" : "-",
           "×" + dash(d.seen_count)] }
  };

  function updateTelemetry(data) {
    const tbody = $("telemetry-body");
    const msg = data.data || data;
    const mod = data.module || currentModule;
    if (mod !== currentModule && mod !== "SYS") return;

    // 4.1 renders as the spectrum graph instead of table rows; the raw JSON
    // line has already been printed to the serial console by handleIncoming().
    if (currentModule === "4.1" && window.Spectrum) {
      if (msg.ch !== undefined) { Spectrum.push(msg.ch, msg.level); return; }
      if (msg.msg !== undefined && msg.msg !== "module_active" && msg.msg !== "module_halted") {
        const isError = data.status === "error";
        const label = msg.msg === "nrf_not_found"
          ? "nRF24 not found: check CE=6 CSN=7 SCK=12 MISO=13 MOSI=11, 3.3 V and the 10 µF cap."
          : msg.msg;
        Spectrum.note(`[${mod}] ${isError ? "ERROR" : "STATUS"}: ${label}`, isError);
      }
      return;
    }

    if (tbody.querySelector(".empty")) tbody.innerHTML = "";

    const tr = document.createElement("tr");
    const isStatusOnly = msg.msg !== undefined && Object.keys(msg).length === 1;

    if (isStatusOnly) {
      if (msg.msg === "module_active" || msg.msg === "module_halted") return;
      const isError = data.status === "error";
      const label = msg.msg === "no_code_captured"
        ? "No IR code captured yet: run 7.2, 7.4 or 7.5 and point a remote at the receiver first."
        : msg.msg === "ledc_attach_failed"
        ? "Could not start the 38kHz carrier on the IR LED pin."
        : msg.msg === "rc522_not_found"
        ? "RC522 not found: check SDA=10 RST=9 SCK=12 MOSI=11 MISO=13, 3.3 V only (5 V kills it)."
        : msg.msg === "ownership_not_acknowledged"
        ? "Clone writes are disabled. They stay off until the firmware's RFID_OPERATOR_OWNS_CARDS gate is set to 1 for cards you own."
        : msg.msg === "not_a_magic_card"
        ? "Target is not a writable-UID magic card (Gen1a/Gen2). Clone needs a magic card you own."
        : msg.msg === "unsupported_card"
        ? "This card type isn't supported by that module (e.g. DESFire / 14443A-4 has no open dump)."
        : msg.msg === "no_ndef"
        ? "No NDEF URI/Text record found on this tag."
        : msg.msg === "source_locked"
        ? "Source card didn't open with the default key, so nothing was cached."
        : msg.msg === "write_failed"
        ? "NDEF write failed: the tag may be read-only or not an NTAG/Ultralight."
        : msg.msg;
      tr.innerHTML = `<td colspan="4" style="color:${isError ? "var(--err)" : "var(--mute)"}">[${mod}] ${isError ? "ERROR" : "STATUS"}: ${label}</td>`;
    } else if (IR_VIEWS[currentModule]) {
      const cells = IR_VIEWS[currentModule].row(msg);
      const styles = ["color:var(--ir);font-weight:700", "color:var(--text)", "color:var(--text)", "color:var(--wifi);font-size:12px"];
      tr.innerHTML = cells.map((c, i) => `<td style="${styles[i]}">${c}</td>`).join("");
      if (IR_VIEWS[currentModule].live) tbody.innerHTML = "";
    } else if (NRF_VIEWS[currentModule]) {
      const cells = NRF_VIEWS[currentModule].row(msg);
      const styles = ["color:var(--nrf);font-weight:700", "color:var(--text)", "color:var(--text)", "color:var(--mute);font-size:12px"];
      tr.innerHTML = cells.map((c, i) => `<td style="${styles[i] || "color:var(--text)"}">${c}</td>`).join("");
      if (NRF_VIEWS[currentModule].live) tbody.innerHTML = "";
    } else if (RFID_VIEWS[currentModule]) {
      const cells = RFID_VIEWS[currentModule].row(msg);
      const styles = ["color:var(--rfid);font-weight:700", "color:var(--text)", "color:var(--text)", "color:var(--mute);font-size:12px"];
      tr.innerHTML = cells.map((c, i) => `<td style="${styles[i] || "color:var(--text)"}">${c}</td>`).join("");
      if (RFID_VIEWS[currentModule].live) tbody.innerHTML = "";
    } else if (currentModule.startsWith("1.")) {
      const d = msg;
      tr.innerHTML = `
        <td style="color:var(--wifi);font-weight:700">${d.ssid || "Hidden"}</td>
        <td style="color:var(--mute)">${d.bssid || "N/A"}</td>
        <td style="color:var(--text)">${d.ch || "-"}</td>
        <td style="color:var(--ir)">${d.rssi || "-"} dBm</td>`;
    } else {
      const d = msg;
      tr.innerHTML = `
        <td style="color:var(--ble);font-weight:700">${d.name || d.addr || "BLE Device"}</td>
        <td style="color:var(--mute);font-size:12px">${d.uuid || d.service || "N/A"}</td>
        <td style="color:var(--text)">${d.rssi ? d.rssi + " dBm" : (d.canRead !== undefined ? (d.canRead ? "READ" : "LOCKED") : "-")}</td>
        <td style="color:var(--wifi);font-size:12px">${d.value_hex || d.payload || d.status || "Active"}</td>`;
    }
    tbody.insertBefore(tr, tbody.firstChild);
  }

  /* ---- module metadata (unchanged) ---- */
  const TITLES = {
    "1.1": "MODULE 1.1: ACTIVE WIFI SCANNER", "1.2": "MODULE 1.2: WARDRIVING LOGGER",
    "1.3": "MODULE 1.3: PROMISCUOUS SNIFFER", "1.4": "MODULE 1.4: NETWORK DISCOVERY",
    "1.5": "MODULE 1.5: AP HOSTING", "1.6": "MODULE 1.6: FRAME LAB TESTING",
    "2.1": "MODULE 2.1: BLE SCAN & FINGERPRINTING", "2.2": "MODULE 2.2: GATT SERVICE ENUMERATION",
    "2.3": "MODULE 2.3: ADVERTISEMENT SNIFFER", "2.4": "MODULE 2.4: CUSTOM BEACON SPAMMER",
    "2.5": "MODULE 2.5: GATT EXPLORATION & VALUE READER", "2.6": "MODULE 2.6: BLE DEVICE STRESS TESTING",
    "4.1": "MODULE 4.1: 2.4GHZ SPECTRUM SCAN", "4.2": "MODULE 4.2: nRF24 PACKET SNIFFER",
    "4.3": "MODULE 4.3: nRF24 ADDRESS DISCOVERY", "4.4": "MODULE 4.4: nRF24 PROTOCOL ANALYSIS",
    "4.5": "MODULE 4.5: nRF24 PACKET STORAGE", "4.6": "MODULE 4.6: nRF24 LAB TRANSMITTER",
    "5.1": "MODULE 5.1: RFID CARD FINGERPRINT", "5.2": "MODULE 5.2: RFID MEMORY DUMP",
    "5.3": "MODULE 5.3: RFID DEFAULT-KEY AUDIT", "5.4": "MODULE 5.4: RFID CLONE TO MAGIC CARD",
    "5.5": "MODULE 5.5: RFID NDEF READ / WRITE", "5.6": "MODULE 5.6: RFID READER MONITOR",
    "7.1": "MODULE 7.1: 38KHZ CARRIER MODULATION", "7.2": "MODULE 7.2: IR PULSE LEARNING",
    "7.3": "MODULE 7.3: IR SIGNAL REPLAY", "7.4": "MODULE 7.4: IR PROTOCOL DECODE",
    "7.5": "MODULE 7.5: IR SIGNAL ANALYSIS", "7.6": "MODULE 7.6: IR LED CIRCUIT TEST"
  };
  const DESCS = {
    "1.1": "Scans local airspace for active BSSIDs, SSIDs, channels, and RSSI metrics.",
    "1.2": "Logs mobile access points and geolocation telemetry data.",
    "1.3": "Captures raw 802.11 management and data frames in promiscuous mode.",
    "1.4": "Discovers connected clients and active hosts on the target network.",
    "1.5": "Hosts an administrative Access Point for localized web control.",
    "1.6": "Injects custom test frames and verifies injection success rates.",
    "2.1": "Discovers nearby BLE peripherals, MAC addresses, RSSI, and mfg data.",
    "2.2": "Enumerates GATT services, characteristic UUIDs, and permission flags.",
    "2.3": "Intercepts raw low-level advertisement payloads and hex packets.",
    "2.4": "Broadcasts custom beacon frames for range and scanner verification.",
    "2.5": "Connects to devices, inspects characteristics, and dumps live hex values.",
    "2.6": "Measures connection handshake latencies and stress-tests targets.",
    "4.1": "Sweeps all 126 nRF24 channels (2400–2525 MHz) and reports carrier energy per channel.",
    "4.2": "Listens on a hopping channel for nRF24 payloads and dumps raw packet bytes as hex.",
    "4.3": "Preamble-sniffs the band and reports candidate nRF24 pipe addresses.",
    "4.4": "Captures packets and reports length, data rate and a traffic-type guess.",
    "4.5": "Buffers captured packets in RAM and tracks a running stored count.",
    "4.6": "Low-power test transmitter — sends to your own nRF24 node for link/range testing.",
    "5.1": "Reads any 13.56 MHz ISO-14443A card's UID, ATQA, SAK and derived card type. Also the RC522 wiring check. A UID is broadcast in the clear — authorized / educational use only.",
    "5.2": "Read-only dump of a presented card: authenticates each MIFARE Classic sector with the default key and dumps every readable block; reads Ultralight/NTAG pages directly. Never writes.",
    "5.3": "Runs a MIFARE Classic card against a dictionary of common / factory / transport keys and grades it A–F by how many sectors fall. Pure read/auth — nothing is written or dumped.",
    "5.4": "Teaching demo: read a SOURCE card you own, then write it to a magic card you own, to show why UID door systems are unsafe. Writes are GATED OFF in firmware by default; it refuses non-magic targets and never touches trailers.",
    "5.5": "Decodes the first URI/Text NDEF record on an NTAG/Ultralight tag — the 'don't tap random NFC tags' demo. Writing is behind a firmware compile switch, off by default.",
    "5.6": "Passive log of every card presented to the reader (UID, type, time, per-UID count) with a running distinct/total summary. UID-only; never authenticates or writes.",
    "7.1": "Drives a continuous 38kHz, 50% duty carrier on the IR LED. Check it with a phone camera or scope.",
    "7.2": "Records the raw mark/space timings of every IR burst the receiver sees.",
    "7.3": "Re-transmits the last capture from 7.2, 7.4 or 7.5 once (raw at 38kHz if the protocol is unknown), then returns to idle.",
    "7.4": "Listens on the IR receiver and decodes NEC, Sony, RC5, RC6 and other remote protocols.",
    "7.5": "Sums mark and space time for each capture and reports the signal duty cycle.",
    "7.6": "Fires a 26µs pulse on the IR LED every 10ms to verify the driver circuit."
  };

  window.switchModule = function (modId, btn) {
    currentModule = modId;
    document.querySelectorAll(".mod-btn").forEach(b => b.classList.remove("active"));
    if (btn) btn.classList.add("active");

    const group = GROUP(modId);
    $("module-bar").style.setProperty("--accent", `var(${ACCENT[group]})`);

    const headerRow = $("table-headers");
    if (modId.startsWith("1.")) {
      headerRow.innerHTML = `<th>SSID / Target</th><th>MAC / BSSID</th><th>CH</th><th>RSSI</th>`;
    } else if (IR_VIEWS[modId]) {
      headerRow.innerHTML = IR_VIEWS[modId].headers.map(h => `<th>${h}</th>`).join("");
    } else if (NRF_VIEWS[modId]) {
      headerRow.innerHTML = NRF_VIEWS[modId].headers.map(h => `<th>${h}</th>`).join("");
    } else if (RFID_VIEWS[modId]) {
      headerRow.innerHTML = RFID_VIEWS[modId].headers.map(h => `<th>${h}</th>`).join("");
    } else {
      headerRow.innerHTML = `<th>Target / Name</th><th>UUID / Addr</th><th>State / RSSI</th><th>Payload / Status</th>`;
    }

    $("module-title").textContent = TITLES[modId];
    $("module-desc").textContent = DESCS[modId];
    $("telemetry-body").innerHTML = EMPTY_ROW;

    // 4.1 swaps the table for the spectrum graph; every other module uses the table.
    const spectrumOn = modId === "4.1" && !!window.Spectrum;
    document.querySelector(".table-scroll").hidden = spectrumOn;
    if (window.Spectrum) { Spectrum.show(spectrumOn); if (spectrumOn) Spectrum.reset(); }

    appendLog(`Switched active context to Module ${modId}`, "sys");
  };

  /* boot: select the first module */
  document.addEventListener("DOMContentLoaded", () => {
    const first = document.querySelector('.mod-btn[data-mod="1.1"]');
    window.switchModule("1.1", first);
  });
})();
