/* Resizable panes for the dashboard. Standalone: it only sets the sidebar width
   and the serial console height, and never touches the Web Serial logic in
   dashboard.js. A pane being enlarged is capped at 75% of the space it shares
   along that axis. Sizes persist per browser in localStorage (best effort). */
(function () {
  "use strict";

  var MAX_FRAC = 0.75;
  var STEP = 16;            // keyboard arrow step, px
  var STORE_KEY = "cyberdeck.panes";

  var body = document.querySelector(".app-body");
  var sidebar = document.querySelector(".sidebar");
  var panels = document.querySelector(".panels");
  var serial = document.querySelector(".panel-serial");
  var colHandle = document.querySelector(".resizer-col");
  var rowHandle = document.querySelector(".resizer-row");
  if (!body || !sidebar || !panels || !serial || !colHandle || !rowHandle) return;

  function load() {
    try { return JSON.parse(localStorage.getItem(STORE_KEY)) || {}; } catch (e) { return {}; }
  }
  function save(state) {
    try { localStorage.setItem(STORE_KEY, JSON.stringify(state)); } catch (e) {}
  }
  var state = load();

  /* Usable length of a flex container along one axis, minus the handle's own
     footprint (it cancels the gap with negative margins, so this is ~the gap). */
  function avail(el, axis) {
    var cs = getComputedStyle(el);
    var inner = axis === "x"
      ? el.clientWidth - parseFloat(cs.paddingLeft) - parseFloat(cs.paddingRight)
      : el.clientHeight - parseFloat(cs.paddingTop) - parseFloat(cs.paddingBottom);
    return Math.max(0, inner - 12);
  }

  /* Each pane spec: which element, which CSS property, and its clamp range.
     Sidebar: grows up to 75% of the body width.
     Serial:  grows up to 75% of the panels height, and shrinks no further than
              25% so the telemetry pane above it never passes 75% either. */
  var panes = {
    col: {
      el: sidebar, prop: "width", axis: "x", def: 244,
      range: function () { var a = avail(body, "x"); return [Math.min(180, a * MAX_FRAC), a * MAX_FRAC]; }
    },
    row: {
      el: serial, prop: "height", axis: "y", def: 184,
      range: function () { var a = avail(panels, "y"); return [a * (1 - MAX_FRAC), a * MAX_FRAC]; }
    }
  };

  function clamp(v, r) { return Math.min(r[1], Math.max(r[0], v)); }

  function apply(key, px, persist) {
    var p = panes[key];
    var v = Math.round(clamp(px, p.range()));
    p.el.style[p.prop] = v + "px";
    if (persist) { state[key] = v; save(state); }
    return v;
  }

  function current(key) {
    var r = panes[key].el.getBoundingClientRect();
    return panes[key].axis === "x" ? r.width : r.height;
  }

  function reset(key) {
    panes[key].el.style[panes[key].prop] = "";
    delete state[key];
    save(state);
  }

  function isActive(handle) { return handle.offsetParent !== null; } // hidden on narrow screens

  function wire(key, handle) {
    var p = panes[key];
    var startPos = 0, startSize = 0;

    handle.addEventListener("pointerdown", function (e) {
      if (e.button !== 0) return;
      e.preventDefault();
      handle.setPointerCapture(e.pointerId);
      startPos = p.axis === "x" ? e.clientX : e.clientY;
      startSize = current(key);
      handle.classList.add("dragging");
      document.body.classList.add("is-resizing", p.axis === "x" ? "resizing-col" : "resizing-row");
    });

    handle.addEventListener("pointermove", function (e) {
      if (!handle.hasPointerCapture(e.pointerId)) return;
      var delta = (p.axis === "x" ? e.clientX : e.clientY) - startPos;
      // The serial console sits below its handle, so dragging down shrinks it.
      apply(key, key === "row" ? startSize - delta : startSize + delta, false);
    });

    function end(e) {
      if (!handle.hasPointerCapture(e.pointerId)) return;
      handle.releasePointerCapture(e.pointerId);
      handle.classList.remove("dragging");
      document.body.classList.remove("is-resizing", "resizing-col", "resizing-row");
      state[key] = Math.round(current(key));
      save(state);
    }
    handle.addEventListener("pointerup", end);
    handle.addEventListener("pointercancel", end);

    handle.addEventListener("dblclick", function () { reset(key); });

    handle.addEventListener("keydown", function (e) {
      var grow = { col: ["ArrowRight"], row: ["ArrowUp"] }[key];
      var shrink = { col: ["ArrowLeft"], row: ["ArrowDown"] }[key];
      if (grow.indexOf(e.key) >= 0) { apply(key, current(key) + STEP, true); e.preventDefault(); }
      else if (shrink.indexOf(e.key) >= 0) { apply(key, current(key) - STEP, true); e.preventDefault(); }
      else if (e.key === "Home" || e.key === "Enter") { reset(key); e.preventDefault(); }
    });
  }

  wire("col", colHandle);
  wire("row", rowHandle);

  // Restore saved sizes, and re-clamp on window resize so no pane ends up
  // above 75% after the browser shrinks.
  function reclamp() {
    Object.keys(panes).forEach(function (key) {
      var handle = key === "col" ? colHandle : rowHandle;
      if (state[key] == null || !isActive(handle)) return;
      apply(key, state[key], false);
    });
  }
  var raf = 0;
  window.addEventListener("resize", function () {
    cancelAnimationFrame(raf);
    raf = requestAnimationFrame(reclamp);
  });
  reclamp();
})();
