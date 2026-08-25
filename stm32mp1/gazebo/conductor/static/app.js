// Cheetah Conductor - vanilla JS, no build step. Polls /api/state, renders
// slots/status/log, and posts a FROZEN config to /api/launch. There is
// deliberately no "edit while running" path: fields disable the moment the
// fleet leaves "idle", because a code/param change mid-run means the run has
// to be redone, not patched.

const DEFAULT_MISSIONS = ["star:10.514:5", "oval:40:5.0", "atom:9.0:6"];
const MISSION_LABEL = { star: "Star (5pt, r=10.5)", oval: "Oval (40m + R5)", atom: "Atom (6-lobe, R=9)", dash: "Dash (100m straight)" };
const GAITS = ["trotRunning", "trotting", "walking", "walking2", "pacing"];

// Same hue list as trail_daemon.py's DOG_HUES, so a dog looks the same colour
// whether the fleet is watched here or (previously) in the native GUI.
const HUES = [
  { dim: "rgba(255,140,26,0.55)", bright: "rgba(255,190,64,1)", fur: "#e08a2e", furDark: "#a8641b" },
  { dim: "rgba(51,140,255,0.55)", bright: "rgba(89,204,255,1)", fur: "#3a8fd6", furDark: "#215f96" },
  { dim: "rgba(77,230,89,0.55)", bright: "rgba(128,255,140,1)", fur: "#4fae55", furDark: "#2f7a35" },
];

// ---------------------------------------------------------------------------
// 8-BIT GO1 MARKER. Chuck's craft-marker is a drone icon pointed at heading;
// the quadruped equivalent is a small pixel-art Go1 (boxy camera head, four
// jointed legs, no background - the earlier shiba placeholder had a product
// photo's white backdrop baked in, which read as a stray white box on the
// canvas), tinted per dog so the fleet stays colour-coded. Drawn once per hue
// onto an offscreen canvas at load and reused every frame - only the
// position/rotation changes, not the pixels. Top-down/plan silhouette (nose
// up), matching how it gets rotated to yaw - not the side-profile pose of a
// product photo, which would not read sensibly spinning in a top-down view.
//   1 = chassis (hue.fur)   5 = chassis shade / legs (hue.furDark)
//   3 = near-black (camera lenses, foot pads)   . = transparent
const DOGE_GRID = [
  "....1111....",
  "....1331....",
  "....1111....",
  "...5.11.5...",
  "..5..11..5..",
  "....111111..",
  "...11111111.",
  "...11111111.",
  "....111111..",
  "..5..11..5..",
  "...5.11.5...",
  "..3......3..",
  "...3....3...",
];
const DOGE_PX = 2.4;   // canvas px per sprite block
const SPRITE_CACHE = {};

function dogeSprite(hue) {
  if (SPRITE_CACHE[hue.fur]) return SPRITE_CACHE[hue.fur];
  const w = DOGE_GRID[0].length, h = DOGE_GRID.length;
  const cv = document.createElement("canvas");
  cv.width = w * DOGE_PX; cv.height = h * DOGE_PX;
  const g = cv.getContext("2d");
  const colors = { "1": hue.fur, "5": hue.furDark, "3": "#1a1410" };
  for (let r = 0; r < h; r++) {
    for (let c = 0; c < w; c++) {
      const ch = DOGE_GRID[r][c];
      if (ch === "." || ch === " ") continue;
      g.fillStyle = colors[ch] || "#fff";
      g.fillRect(c * DOGE_PX, r * DOGE_PX, DOGE_PX, DOGE_PX);
    }
  }
  SPRITE_CACHE[hue.fur] = { canvas: cv, w: cv.width, h: cv.height };
  return SPRITE_CACHE[hue.fur];
}

// Big red X for a camera tile when its dog has fallen - an SVG overlay is
// simplest here since these tiles are plain DOM, not canvas.
const FALL_X_SVG = `<div class="cam-fell"><svg viewBox="0 0 100 100">
  <line x1="10" y1="10" x2="90" y2="90" stroke="#ff3b30" stroke-width="10" stroke-linecap="round"/>
  <line x1="90" y1="10" x2="10" y2="90" stroke="#ff3b30" stroke-width="10" stroke-linecap="round"/>
</svg></div>`;

function drawFallX(ctx, cx, cy, r) {
  ctx.strokeStyle = "#ff3b30"; ctx.lineWidth = 3.5; ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(cx - r, cy - r); ctx.lineTo(cx + r, cy + r);
  ctx.moveTo(cx + r, cy - r); ctx.lineTo(cx - r, cy + r);
  ctx.stroke();
}

let state = { phase: "idle", recipes: {}, hard_cap: 3.9 };
let slots = DEFAULT_MISSIONS.map((m, i) => ({
  mission: m, gait: i === 2 ? "trotting" : "trotRunning", speed: i === 2 ? 2.1 : 3.5, dash: 100,
  cam_front: true, cam_nadir: true, cam_chase: true,
  chase_distance: 3.0, chase_height: 1.2, chase_degree: 90,
}));

function kindOf(spec) { return spec.split(":")[0]; }

function renderSlots() {
  const el = document.getElementById("slots");
  // Only an ACTIVE fleet locks the draft - "done" used to leave every input
  // disabled until a manual Stop, which read as the whole UI being broken.
  const locked = state.phase === "launching" || state.phase === "running";
  el.innerHTML = slots.map((s, i) => {
    const recipe = state.recipes[kindOf(s.mission)] || {};
    return `<div class="play-card active">
      <h3>Dog ${i} <small>${(recipe.note || "").split(" - ")[1] || ""}</small></h3>
      <p>${recipe.note || ""}</p>
      <div class="slot-row">
        <label>Mission
          <select data-i="${i}" data-f="mission" ${locked ? "disabled" : ""}>
            ${Object.keys(MISSION_LABEL).map(k =>
              `<option value="${k === "dash" ? "outback:100" : k + (k === "star" ? ":10.514:5" : k === "oval" ? ":40:5.0" : ":9.0:6")}"
                ${kindOf(s.mission) === k ? "selected" : ""}>${MISSION_LABEL[k]}</option>`).join("")}
          </select>
        </label>
        <label>Gait
          <select data-i="${i}" data-f="gait" ${locked ? "disabled" : ""}>
            ${GAITS.map(g => `<option ${s.gait === g ? "selected" : ""}>${g}</option>`).join("")}
          </select>
        </label>
        <label>Speed cmd (m/s)
          <input type="number" step="0.1" min="0.3" max="3.9" value="${s.speed}"
                 data-i="${i}" data-f="speed" ${locked ? "disabled" : ""}>
        </label>
        <label>Dash finish (m)
          <input type="number" step="10" min="0" max="200" value="${s.dash ?? 100}"
                 title="Straight sprint appended after the loop closes - stop, lie down, stand back up, then dash. 0 = no dash, mission ends at the loop."
                 data-i="${i}" data-f="dash" ${locked ? "disabled" : ""}>
        </label>
        <label style="justify-content:flex-end">
          <button class="slot-remove" data-remove="${i}" ${locked || slots.length <= 1 ? "disabled" : ""}>&times; remove</button>
        </label>
      </div>
      <div class="slot-row cam-config-row">
        <label class="cam-check"><input type="checkbox" data-i="${i}" data-f="dash_toggle"
               ${(s.dash ?? 0) > 0 ? "checked" : ""} ${locked ? "disabled" : ""}
               title="Quick toggle for the field above - checked sets a 100m dash finish, unchecked sets 0 (mission ends at the loop)."> 100m dash when done</label>
      </div>
      <div class="slot-row cam-config-row">
        ${["air", "pro", "edu"].map(m => `
        <label class="cam-check" title="Hard per-model speed ceiling, enforced server-side on top of the fleet cap - the speed command can never exceed it. Edu's 4.7 is Unitree's peak-sprint figure (optimised test conditions), not a sustained rating.">
          <input type="radio" name="model-${i}" data-i="${i}" data-f="model" value="${m}"
                 ${(s.model || "edu") === m ? "checked" : ""} ${locked ? "disabled" : ""}>
          Go1 ${m[0].toUpperCase() + m.slice(1)} (&le;${(state.model_max_speed || { air: 2.5, pro: 3.5, edu: 4.7 })[m]} m/s)</label>`).join("")}
      </div>
      <div class="slot-row cam-config-row">
        <label class="cam-check" title="Live: unchecking mid-run mutes the stream instantly, re-checking resumes it. A camera unchecked at LAUNCH is never spawned in the world and cannot come back mid-run."><input type="checkbox" data-i="${i}" data-f="cam_front"
               ${s.cam_front !== false ? "checked" : ""}> Front cam</label>
        <label class="cam-check" title="Live: unchecking mid-run mutes the stream instantly, re-checking resumes it."><input type="checkbox" data-i="${i}" data-f="cam_nadir"
               ${s.cam_nadir !== false ? "checked" : ""}> Nadir cam</label>
        <label class="cam-check" title="Live: unchecking mid-run mutes the stream instantly, re-checking resumes it."><input type="checkbox" data-i="${i}" data-f="cam_chase"
               ${s.cam_chase !== false ? "checked" : ""}> Chase cam</label>
      </div>
      <div class="slot-row cam-config-row">
        <label>Chase dist (m)
          <input type="number" step="0.5" min="0.5" max="10" value="${s.chase_distance ?? 3.0}"
                 data-i="${i}" data-f="chase_distance" ${locked ? "disabled" : ""}>
        </label>
        <label>Chase height (m)
          <input type="number" step="0.1" min="0.1" max="5" value="${s.chase_height ?? 1.2}"
                 data-i="${i}" data-f="chase_height" ${locked ? "disabled" : ""}>
        </label>
        <label>Chase angle (deg)
          <input type="number" step="15" min="-360" max="360" value="${s.chase_degree ?? 90}"
                 title="0 = directly behind, 90 = left side, -90 = right side"
                 data-i="${i}" data-f="chase_degree" ${locked ? "disabled" : ""}>
        </label>
      </div>
    </div>`;
  }).join("");

  // Every field edit updates the LOCAL array immediately (instant feedback)
  // and mirrors it to the server's draft over REST - same endpoint an
  // external automation script would hit, so a click in the browser and a
  // curl are literally the same code path underneath.
  const NUMERIC_FIELDS = new Set(["speed", "dash", "chase_distance", "chase_height", "chase_degree"]);
  el.querySelectorAll("select,input").forEach(elm => {
    elm.addEventListener("change", e => {
      const i = +e.target.dataset.i;
      // dash_toggle is a UI-only quick-set for the "dash" field - not a real
      // slot key, so it maps onto the same "dash" the numeric input owns
      // rather than getting its own field on the server.
      const f = e.target.dataset.f === "dash_toggle" ? "dash" : e.target.dataset.f;
      slots[i][f] = e.target.dataset.f === "dash_toggle" ? (e.target.checked ? 100 : 0)
        : e.target.type === "checkbox" ? e.target.checked
        : NUMERIC_FIELDS.has(f) ? +e.target.value : e.target.value;
      // Mirror the server's model ceiling locally for instant feedback -
      // the server clamps authoritatively either way.
      if (f === "model" || f === "speed") {
        const cap = (state.model_max_speed || { air: 2.5, pro: 3.5, edu: 4.7 })[slots[i].model || "edu"] ?? 3.9;
        if (+slots[i].speed > cap) slots[i].speed = cap;
      }
      fetch("/api/slots/" + i, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ [f]: slots[i][f] }),
      });
      renderSlots();
    });
  });
  el.querySelectorAll("[data-remove]").forEach(b => {
    b.addEventListener("click", e => {
      const i = +e.target.dataset.remove;
      slots.splice(i, 1);
      fetch("/api/slots/" + i, { method: "DELETE" });
      renderSlots();
    });
  });
  document.getElementById("addSlot").disabled = locked || slots.length >= 3;
}

function renderScene() {
  const canvas = document.getElementById("scene");
  const wrap = canvas.parentElement;
  const dpr = window.devicePixelRatio || 1;
  const w = wrap.clientWidth, h = wrap.clientHeight;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const planned = state.planned || {}, positions = state.positions || {};
  const allPts = [];
  Object.values(planned).forEach(pts => pts.forEach(p => allPts.push(p)));
  Object.values(positions).forEach(p => { allPts.push([p.x, p.y]);
    (p.trail || []).forEach(t => allPts.push(t)); });

  if (!allPts.length) {
    ctx.fillStyle = "#3a4038"; ctx.font = "12px monospace";
    ctx.fillText("no fleet launched - nothing to draw yet", 14, 22);
    updateLegend();
    return;
  }
  // world is ENU (x=east, y=north). Fit all points with padding, flip Y so
  // north is up on screen.
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  allPts.forEach(([x, y]) => { minX = Math.min(minX, x); maxX = Math.max(maxX, x);
                                minY = Math.min(minY, y); maxY = Math.max(maxY, y); });
  const pad = 6;
  minX -= pad; maxX += pad; minY -= pad; maxY += pad;
  const spanX = Math.max(maxX - minX, 1), spanY = Math.max(maxY - minY, 1);
  const scale = Math.min(w / spanX, h / spanY);
  const ox = (w - spanX * scale) / 2 - minX * scale;
  const oy = h - ((h - spanY * scale) / 2 - minY * scale); // flip: screen-y grows down
  const X = x => ox + x * scale, Y = y => oy - y * scale;

  // grid, subtle
  ctx.strokeStyle = "#1a2020"; ctx.lineWidth = 1;
  for (let gx = Math.ceil(minX / 10) * 10; gx < maxX; gx += 10) {
    ctx.beginPath(); ctx.moveTo(X(gx), 0); ctx.lineTo(X(gx), h); ctx.stroke();
  }
  for (let gy = Math.ceil(minY / 10) * 10; gy < maxY; gy += 10) {
    ctx.beginPath(); ctx.moveTo(0, Y(gy)); ctx.lineTo(w, Y(gy)); ctx.stroke();
  }

  (state.slots || []).forEach(s => {
    const i = s.index, hue = HUES[i % HUES.length];
    const p = planned[i];
    if (p && p.length > 1) {
      ctx.strokeStyle = hue.dim; ctx.lineWidth = 2; ctx.beginPath();
      p.forEach(([x, y], k) => k ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)));
      ctx.closePath(); ctx.stroke();
    }
    const live = positions[i];
    if (live && live.trail && live.trail.length > 1) {
      ctx.strokeStyle = hue.bright; ctx.lineWidth = 2.5; ctx.beginPath();
      live.trail.forEach(([x, y], k) => k ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)));
      ctx.stroke();
    }
    if (live) {
      const cx = X(live.x), cy = Y(live.y);
      const sprite = dogeSprite(hue);
      const size = 22;  // fixed screen size - a UI glyph, not to-scale
      // world yaw is CCW-from-east; canvas Y points down and the sprite's
      // own "forward" is drawn facing up (-Y in its local grid), so rotate
      // by -(yaw) then correct for the axis flip, same convention as the
      // heading tick this replaces.
      const screenAngle = -live.yaw + Math.PI / 2;
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(screenAngle);
      ctx.drawImage(sprite.canvas, -size / 2, -size / 2, size, size);
      ctx.restore();

      // COMPASS NEEDLE - a separate, explicit heading indicator alongside the
      // sprite's own rotation. At 22px the sprite's asymmetry (which end is
      // the camera head) is hard to read at a glance, especially mid-turn;
      // a needle sticking out past the body reads unambiguously regardless.
      const nLen = size * 0.95, nTailLen = size * 0.32, headLen = 5.5;
      const nx = Math.cos(screenAngle - Math.PI / 2), ny = Math.sin(screenAngle - Math.PI / 2);
      const tipX = cx + nx * nLen, tipY = cy + ny * nLen;
      ctx.save();
      ctx.strokeStyle = "rgba(230,235,230,0.9)"; ctx.lineWidth = 1.4; ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(cx - nx * nTailLen, cy - ny * nTailLen);
      ctx.lineTo(tipX, tipY);
      ctx.stroke();
      // arrowhead
      const perpX = -ny, perpY = nx;
      ctx.fillStyle = "rgba(230,235,230,0.9)";
      ctx.beginPath();
      ctx.moveTo(tipX, tipY);
      ctx.lineTo(tipX - nx * headLen + perpX * headLen * 0.55, tipY - ny * headLen + perpY * headLen * 0.55);
      ctx.lineTo(tipX - nx * headLen - perpX * headLen * 0.55, tipY - ny * headLen - perpY * headLen * 0.55);
      ctx.closePath(); ctx.fill();
      ctx.restore();

      const fell = (state.status || []).find(r => r.index === i && r.phase === "fell");
      if (fell) drawFallX(ctx, cx, cy, size * 0.8);

      ctx.fillStyle = "#e6ebe6"; ctx.font = "10px monospace";
      ctx.fillText("#" + i, cx + size / 2 + 3, cy - size / 2);

      // LIVE SPEED / HEIGHT READOUT, stacked above the icon.
      if (live.speed != null || live.z != null) {
        ctx.font = "10px monospace"; ctx.textAlign = "center";
        if (live.speed != null) {
          ctx.fillStyle = hue.bright;
          ctx.fillText(live.speed.toFixed(1) + " m/s", cx, cy - size / 2 - 16);
        }
        if (live.z != null) {
          ctx.fillStyle = "rgba(230,235,230,0.85)";
          ctx.fillText("z=" + live.z.toFixed(2) + "m", cx, cy - size / 2 - 5);
        }
        ctx.textAlign = "left";
      }
    }
  });
  updateLegend();
}

function updateLegend() {
  const el = document.getElementById("sceneLegend");
  const rows = (state.slots || []).map(s => {
    const hue = HUES[s.index % HUES.length];
    return `<span class="sw" style="background:${hue.bright}"></span>#${s.index} ${s.mission || ""}`;
  });
  el.innerHTML = rows.length
    ? rows.join("<br>") + "<br>dim = planned, bright = flown"
    : "no dogs placed";
}

function renderFleet() {
  const cap = +document.getElementById("capSlider").value;
  document.getElementById("capVal").textContent = cap.toFixed(1);

  const cards = document.getElementById("fleetCards");
  const rows = state.status || [];
  if (!rows.length) {
    cards.innerHTML = `<div class="fleet-card"><div class="fleet-meta">No fleet launched yet. Configure slots on the right, set the speed cap, and Launch.</div></div>`;
  } else {
    cards.innerHTML = rows.map(r => {
      const s = (state.slots || []).find(x => x.index === r.index) || {};
      const cls = r.phase === "complete" ? "complete" : r.phase === "fell" ? "fell" : "";
      const dot = r.phase === "complete" ? "up" : r.phase === "fell" ? "down" : "mid";
      const cam = (state.cameras || {})[r.index] || {};
      const tile = (name, label) => {
        const url = cam[name];
        return `<div class="cam-tile${url ? " live" : ""}">
          ${url ? `<img src="${url}">` : ""}
          <span class="cam-label">${label}</span>
          ${r.phase === "fell" ? FALL_X_SVG : ""}
        </div>`;
      };
      return `<div class="fleet-card ${cls}">
        <div class="fleet-head"><span class="dot ${dot}"></span>
          <span class="fleet-idx">#${r.index}</span>
          <span class="fleet-name">${s.mission || ""}</span>
          <span class="fleet-phase ${r.phase}">${r.phase}</span>
        </div>
        <div class="fleet-meta">gait=${s.gait_name || s.gait || "-"} cmd=${s.speed ?? "-"} m/s
          ${r.waypoints ? "&middot; wp " + r.waypoints : ""}
          ${r.text ? "&middot; " + r.text : ""}
          ${r.t ? "&middot; t=" + r.t : ""}
        </div>
        ${(() => {
          // Tile visibility follows the DRAFT checkbox live (the server
          // gates the stream on the same flag), so unchecking a camera
          // mid-run hides its tile immediately instead of freezing it on
          // the last frame. The locked flag still gates what was spawned.
          const d = (state.draft_slots || [])[r.index] || {};
          const on = k => s[k] !== false && d[k] !== false;
          return `<div class="cam-row">${on("cam_front") ? tile("front_cam", "FWD") : ""}${
            on("cam_nadir") ? tile("nadir_cam", "DOWN") : ""}${
            on("cam_chase") ? tile("chase_cam", "CHASE") : ""}</div>`;
        })()}
      </div>`;
    }).join("");
  }

  const rid = state.run_id ? ("RUN " + state.run_id) : "";
  const hdr = document.querySelector(".topbar .subtitle, .topbar h2, .topbar span");
  if (hdr && rid && !hdr.dataset.norun) hdr.textContent = hdr.textContent.replace(/ · RUN \d+$/, "") + " · " + rid;
  document.getElementById("logBox").textContent = (state.log || []).join("\n");
  document.getElementById("logBox").scrollTop = 1e9;

  const running = state.phase === "launching" || state.phase === "running";
  document.getElementById("launchBtn").disabled = running;
  document.getElementById("launchBtn").textContent =
    running ? "Fleet running..." : (state.phase === "done" ? "Re-launch fleet" : "Launch fleet");
  document.getElementById("stopBtn").disabled = !running;
  document.getElementById("caption").textContent =
    "STM32MP1 -> Go1 SITL fleet control - phase: " + state.phase;

  renderLoadWidget();
}

function renderLoadWidget() {
  const load = state.host_load || {};
  const setBar = (fillId, valId, pct) => {
    const fill = document.getElementById(fillId), val = document.getElementById(valId);
    if (pct == null) { val.textContent = "--"; fill.style.width = "0%"; return; }
    val.textContent = pct.toFixed(0) + "%";
    fill.style.width = Math.min(100, pct) + "%";
    fill.classList.toggle("warn", pct >= 60 && pct < 90);
    fill.classList.toggle("hot", pct >= 90);
  };
  setBar("cpuFill", "cpuVal", load.cpu_pct);
  setBar("gpuFill", "gpuVal", load.gpu_pct);
}

let _synced = false;
async function poll() {
  try {
    const r = await fetch("/api/state");
    state = await r.json();
    // One-time: adopt the SERVER's draft (which an external REST call may
    // already have edited before this page ever opened) instead of the
    // browser's hardcoded defaults. After this, local edits are the source
    // of truth for the page and are themselves what pushes to the server.
    if (!_synced && state.draft_slots) {
      slots = state.draft_slots;
      document.getElementById("capSlider").value = state.draft_cap ?? 3.5;
      if (state.draft_terrain) document.getElementById("terrainSelect").value = state.draft_terrain;
      renderSlots();
      _synced = true;
    }
  } catch (e) { /* server restarting - ignore this tick */ }
  renderFleet();
  renderScene();
  setTimeout(poll, 400);  // faster than the old 1s poll - the scene wants to feel live
}
window.addEventListener("resize", renderScene);

document.getElementById("capSlider").addEventListener("input", renderFleet);
document.getElementById("capSlider").addEventListener("change", e => {
  fetch("/api/speed_cap", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ value: +e.target.value }),
  });
});
document.getElementById("terrainSelect").addEventListener("change", e => {
  fetch("/api/terrain", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ value: e.target.value }),
  });
});
document.getElementById("addSlot").addEventListener("click", async () => {
  if (slots.length >= 3) return;
  const r = await fetch("/api/slots/add", { method: "POST" });
  const j = await r.json();
  if (j.ok) { slots = j.slots; renderSlots(); } else { alert(j.message); }
});
document.getElementById("launchBtn").addEventListener("click", async () => {
  const cap = +document.getElementById("capSlider").value;
  const terrainVal = document.getElementById("terrainSelect").value;
  const r = await fetch("/api/launch", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ slots, speed_cap: cap, terrain: terrainVal }),
  });
  const j = await r.json();
  if (!j.ok) alert(j.message || j.error);
});
document.getElementById("stopBtn").addEventListener("click", async () => {
  await fetch("/api/stop", { method: "POST" });
});

renderSlots();
poll();
