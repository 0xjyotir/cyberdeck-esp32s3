/* Animated ESP32-S3 board for the welcome hero. The board itself is static SVG
   (#deck-board), drawn as a dashed schematic; this script lights it up: a bright
   dot runs outward along each subsystem trace in that subsystem's hue, and the
   power LED breathes. Pointer movement spawns an extra pulse so the board
   feels alive under the cursor. Honours prefers-reduced-motion. */
(function () {
  const rm = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const svg = document.getElementById("deck-board");
  if (!svg || !window.anime) return;

  const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

  // Each trace has a dim base path (already in the SVG) and a bright pulse path.
  const TRACES = [
    { id: "wifi", color: "--wifi", dur: 1700 },
    { id: "m5",   color: "--m5",   dur: 1750 },
    { id: "ble",  color: "--ble",  dur: 1500 },
    { id: "ir",   color: "--ir",   dur: 1900 },
    { id: "nrf",  color: "--nrf",  dur: 1600 },
    { id: "nfc",  color: "--nfc",  dur: 1800 }
  ];

  function pulse(t, delay) {
    const el = document.getElementById("pulse-" + t.id);
    if (!el) return;
    el.style.stroke = css(t.color);
    anime({
      targets: el,
      strokeDashoffset: [2400, -40],
      easing: "easeInOutSine",
      duration: t.dur,
      delay: delay || 0,
      complete: () => pulse(t, 2200 + Math.random() * 1800)
    });
  }

  function led() {
    anime({
      targets: "#power-led",
      opacity: [0.25, 1],
      easing: "easeInOutQuad",
      duration: 900,
      direction: "alternate",
      loop: true
    });
  }

  // intro: fade the schematic in piece by piece, then start the loops
  function start() {
    anime({ targets: "#deck-board .trace-base, #deck-board .chip, #deck-board .pin, #deck-board .pad, #deck-board .usb",
      opacity: [0, 1], easing: "easeOutQuad", duration: 900, delay: anime.stagger(20, { start: 200 }) });

    TRACES.forEach((t, i) => pulse(t, 1400 + i * 260));
    led();
  }

  if (rm) {
    // static: everything visible, no motion
    svg.querySelectorAll(".chip, .pin, .pad, .usb, .trace-base").forEach((el) => (el.style.opacity = 1));
    return;
  }
  start();

  // pointer interaction: nearest trace gets an immediate extra pulse
  const hero = svg.closest(".hero");
  let throttle;
  hero.addEventListener("pointermove", () => {
    if (throttle) return;
    throttle = setTimeout(() => (throttle = null), 500);
    const t = TRACES[Math.floor(Math.random() * TRACES.length)];
    const el = document.getElementById("pulse-" + t.id);
    if (!el) return;
    el.style.stroke = css(t.color);
    anime({ targets: el, strokeDashoffset: [2400, -40], easing: "easeOutSine", duration: 900 });
  });
})();
