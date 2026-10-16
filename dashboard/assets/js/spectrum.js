/* nRF24 spectrum view for module 4.1, styled after Bruce firmware's per-channel
   bar display. dashboard.js feeds it one {ch, level} reading per serial line;
   this file keeps the 126-channel state and draws it on a canvas once per
   animation frame (data arrives ~80×/s, so drawing per line would waste work).
   View only: it never touches the serial port or the console. */
(function () {
  "use strict";

  var CH = 126;            // nRF24 channels 0–125 → 2400–2525 MHz
  var MAX = 32;            // RPD samples per channel per pass (level 0–32)
  var WF_ROWS = 60;        // waterfall history, one row per full sweep
  var PEAK_DECAY = 4;      // peak-hold fall rate, levels per second
  var WIFI = [{ ch: 12, n: 1 }, { ch: 37, n: 6 }, { ch: 62, n: 11 }]; // 2412/2437/2462 MHz
  var TICKS = [0, 12, 37, 62, 80, 100, 125];

  var root, canvas, ctx, tip, elSweep, elPeak, elNote;
  var levels = new Uint8Array(CH);
  var peaks = new Float32Array(CH);
  var lastCh = -1, sweeps = 0;
  var wf, wfCtx;           // offscreen waterfall, 1px per channel per sweep
  var running = false, raf = 0, lastT = 0, hoverCh = -1;
  var colors = { lo: [122, 160, 255], mid: [255, 210, 122], hi: [255, 107, 107], grid: "rgba(255,255,255,.06)", txt: "#6e6e73", txt2: "#a1a1a6" };

  function hexToRgb(h) {
    h = (h || "").trim().replace("#", "");
    if (h.length === 3) h = h.replace(/./g, "$&$&");
    var n = parseInt(h, 16);
    return isNaN(n) ? null : [(n >> 16) & 255, (n >> 8) & 255, n & 255];
  }

  // Read the subsystem hues from the design tokens so the graph tracks the theme.
  function readTokens() {
    var cs = getComputedStyle(document.body);
    colors.lo = hexToRgb(cs.getPropertyValue("--nrf")) || colors.lo;
    colors.mid = hexToRgb(cs.getPropertyValue("--ir")) || colors.mid;
    colors.hi = hexToRgb(cs.getPropertyValue("--err")) || colors.hi;
    colors.txt = cs.getPropertyValue("--txt3").trim() || colors.txt;
    colors.txt2 = cs.getPropertyValue("--txt2").trim() || colors.txt2;
  }

  // Level 0–32 → colour ramp nRF blue → IR amber → error red.
  function ramp(level) {
    var f = Math.min(1, level / MAX), a, b, t;
    if (f < 0.5) { a = colors.lo; b = colors.mid; t = f / 0.5; }
    else { a = colors.mid; b = colors.hi; t = (f - 0.5) / 0.5; }
    return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
  }
  function rgba(c, a) { return "rgba(" + (c[0] | 0) + "," + (c[1] | 0) + "," + (c[2] | 0) + "," + a + ")"; }

  function geometry() {
    var w = canvas.clientWidth, h = canvas.clientHeight;
    var left = 30, right = 8, top = 18, axis = 20;
    var wfH = Math.max(24, Math.round(h * 0.26));
    var plotH = Math.max(40, h - top - axis - wfH - 10);
    return {
      w: w, h: h, x: left, y: top, pw: Math.max(10, w - left - right), ph: plotH,
      wfY: top + plotH + axis + 6, wfH: wfH
    };
  }

  function draw(t) {
    if (!running) return;
    var dt = lastT ? Math.min(0.1, (t - lastT) / 1000) : 0;
    lastT = t;
    for (var i = 0; i < CH; i++) peaks[i] = Math.max(levels[i], peaks[i] - PEAK_DECAY * dt);

    var g = geometry(), dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, g.w, g.h);
    var bw = g.pw / CH;
    var xOf = function (c) { return g.x + c * bw; };
    var yOf = function (lv) { return g.y + g.ph - (lv / MAX) * g.ph; };

    // Wi-Fi channel 1/6/11 bands (22 MHz wide) behind everything.
    ctx.font = "10px 'JetBrains Mono', ui-monospace, monospace";
    ctx.textAlign = "center";
    WIFI.forEach(function (b) {
      ctx.fillStyle = "rgba(88,198,255,.05)";
      ctx.fillRect(xOf(b.ch - 11), g.y, bw * 22, g.ph);
      ctx.fillStyle = "rgba(88,198,255,.45)";
      ctx.fillText("Wi-Fi " + b.n, xOf(b.ch) + bw / 2, g.y - 6);
    });

    // Horizontal grid + y labels.
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (var lv = 0; lv <= MAX; lv += 8) {
      var gy = Math.round(yOf(lv)) + 0.5;
      ctx.strokeStyle = colors.grid;
      ctx.beginPath(); ctx.moveTo(g.x, gy); ctx.lineTo(g.x + g.pw, gy); ctx.stroke();
      if (lv && lv % 16 === 0) { ctx.fillStyle = colors.txt; ctx.fillText(String(lv), g.x - 6, gy); }
    }

    // Live bars, then peak-hold ticks.
    var gap = bw > 3 ? 1 : 0;
    for (var c = 0; c < CH; c++) {
      var x = xOf(c);
      if (levels[c] > 0) {
        var col = ramp(levels[c]), top = yOf(levels[c]);
        var grad = ctx.createLinearGradient(0, top, 0, g.y + g.ph);
        grad.addColorStop(0, rgba(col, 0.95));
        grad.addColorStop(1, rgba(col, 0.25));
        ctx.fillStyle = grad;
        ctx.fillRect(x, top, bw - gap, g.y + g.ph - top);
      }
      if (peaks[c] >= 1) {
        ctx.fillStyle = rgba(ramp(peaks[c]), 0.9);
        ctx.fillRect(x, Math.round(yOf(peaks[c])), bw - gap, 2);
      }
      if (c === hoverCh) {
        ctx.fillStyle = "rgba(255,255,255,.08)";
        ctx.fillRect(x, g.y, bw, g.ph);
      }
    }

    // X axis: MHz ticks.
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    ctx.fillStyle = colors.txt;
    ctx.strokeStyle = "rgba(255,255,255,.14)";
    ctx.beginPath(); ctx.moveTo(g.x, g.y + g.ph + 0.5); ctx.lineTo(g.x + g.pw, g.y + g.ph + 0.5); ctx.stroke();
    TICKS.forEach(function (tc) {
      var tx = xOf(tc) + bw / 2;
      ctx.beginPath(); ctx.moveTo(tx, g.y + g.ph); ctx.lineTo(tx, g.y + g.ph + 4); ctx.stroke();
      ctx.fillText(String(2400 + tc), tx, g.y + g.ph + 6);
    });

    // Waterfall: offscreen history scaled to fit, newest sweep on top.
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = "rgba(0,0,0,.25)";
    ctx.fillRect(g.x, g.wfY, g.pw, g.wfH);
    ctx.drawImage(wf, 0, 0, CH, WF_ROWS, g.x, g.wfY, g.pw, g.wfH);

    raf = requestAnimationFrame(draw);
  }

  // Shift the waterfall down one row and paint the sweep that just finished on top.
  function commitSweep() {
    // "copy" replaces pixels instead of blending, so faint rows don't stack up alpha as they scroll.
    wfCtx.globalCompositeOperation = "copy";
    wfCtx.drawImage(wf, 0, 0, CH, WF_ROWS - 1, 0, 1, CH, WF_ROWS - 1);
    wfCtx.globalCompositeOperation = "source-over";
    var row = wfCtx.createImageData(CH, 1);
    for (var c = 0; c < CH; c++) {
      // Curve the alpha so 1–3-hit background noise stays dim and real carriers pop.
      var col = ramp(levels[c]), a = levels[c] ? Math.round(255 * Math.pow(levels[c] / MAX, 0.75)) : 0;
      row.data[c * 4] = col[0]; row.data[c * 4 + 1] = col[1]; row.data[c * 4 + 2] = col[2]; row.data[c * 4 + 3] = a;
    }
    wfCtx.putImageData(row, 0, 0);
    sweeps++;
    updateMeta();
  }

  function updateMeta() {
    var pc = 0;
    for (var c = 1; c < CH; c++) if (levels[c] > levels[pc]) pc = c;
    elSweep.textContent = "sweep #" + sweeps;
    elPeak.textContent = levels[pc] ? "peak ch" + pc + " · " + (2400 + pc) + " MHz · " + levels[pc] + "/" + MAX : "peak · quiet";
  }

  function onMove(e) {
    var g = geometry(), r = canvas.getBoundingClientRect();
    var mx = e.clientX - r.left, my = e.clientY - r.top;
    var c = Math.floor((mx - g.x) / (g.pw / CH));
    if (c < 0 || c >= CH || my < g.y || my > g.wfY + g.wfH) { onLeave(); return; }
    hoverCh = c;
    tip.hidden = false;
    tip.textContent = "ch " + c + " · " + (2400 + c) + " MHz · " + levels[c] + "/" + MAX + " · peak " + Math.round(peaks[c]);
    var tw = tip.offsetWidth;
    tip.style.left = Math.min(Math.max(4, mx - tw / 2), r.width - tw - 4) + "px";
    tip.style.top = Math.max(0, my - 34) + "px";
  }
  function onLeave() { hoverCh = -1; if (tip) tip.hidden = true; }

  function fit() {
    if (!canvas) return;
    var dpr = window.devicePixelRatio || 1;
    var w = Math.max(1, Math.round(canvas.clientWidth * dpr)), h = Math.max(1, Math.round(canvas.clientHeight * dpr));
    if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  }

  function init() {
    root = document.getElementById("spectrum");
    if (!root) return false;
    canvas = root.querySelector("canvas");
    ctx = canvas.getContext("2d");
    tip = root.querySelector(".spectrum-tip");
    elSweep = document.getElementById("spec-sweep");
    elPeak = document.getElementById("spec-peak");
    elNote = document.getElementById("spec-note");
    wf = document.createElement("canvas");
    wf.width = CH; wf.height = WF_ROWS;
    wfCtx = wf.getContext("2d");
    canvas.addEventListener("mousemove", onMove);
    canvas.addEventListener("mouseleave", onLeave);
    if ("ResizeObserver" in window) new ResizeObserver(fit).observe(canvas);
    window.addEventListener("resize", fit);
    return true;
  }

  window.Spectrum = {
    // One serial reading. A channel lower than the previous one means the sweep wrapped.
    push: function (ch, level) {
      if (!root && !init()) return;
      ch = Number(ch); level = Math.max(0, Math.min(MAX, Number(level) || 0));
      if (!(ch >= 0 && ch < CH)) return;
      if (ch < lastCh) commitSweep();
      lastCh = ch;
      levels[ch] = level;
      if (level > peaks[ch]) peaks[ch] = level;
      if (elNote.textContent) note("");
    },
    reset: function () {
      if (!root && !init()) return;
      levels.fill(0); peaks.fill(0); lastCh = -1; sweeps = 0;
      wfCtx.clearRect(0, 0, CH, WF_ROWS);
      note("");
      updateMeta();
    },
    note: note,
    show: function (on) {
      if (!root && !init()) return;
      root.hidden = !on;
      if (on && !running) {
        readTokens();
        running = true; lastT = 0;
        requestAnimationFrame(function () { fit(); updateMeta(); raf = requestAnimationFrame(draw); });
      } else if (!on && running) {
        running = false;
        cancelAnimationFrame(raf);
        onLeave();
      }
    }
  };

  // Status / error line shown above the graph (e.g. nrf_not_found).
  function note(text, isError) {
    if (!elNote) return;
    elNote.textContent = text || "";
    elNote.classList.toggle("err", !!isError);
  }
})();
