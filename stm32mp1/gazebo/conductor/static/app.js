// Cheetah Conductor - vanilla JS, no build step. Polls /api/state, renders
// slots/status/log, and posts a FROZEN config to /api/launch. There is
// deliberately no "edit while running" path: fields disable the moment the
// fleet leaves "idle", because a code/param change mid-run means the run has
// to be redone, not patched.

const DEFAULT_MISSIONS = ["star:10.514:5", "oval:40:5.0", "atom:9.0:6"];
// One entry per DROPDOWN OPTION, not per mission kind - the three Lissajous
// ratios all share kindOf()==="lissajous" (same recipe, same note) but are
// three distinct, separately selectable missions, so this is a flat list
// rather than the kind-keyed dict it used to be.
const MISSION_OPTIONS = [
  { value: "star:10.514:5", label: "Star (5pt, r=10.5)" },
  { value: "oval:40:5.0", label: "Oval (40m + R5)" },
  { value: "atom:9.0:6", label: "Atom (6-lobe, R=9)" },
  { value: "dash:100", label: "Dash (100m straight)" },
  // circle:R:N is an N-gon - N=8 is a regular OCTAGON (45 deg per vertex),
  // and labelling it "Circle" hid exactly the property (discrete sharp
  // direction changes) that the cornering-envelope work showed matters
  // most. N=36 (~10 deg/vertex) is functionally smooth at this port's
  // corridor/lookahead scale and is the real circle. Spec strings stay
  // stable (generator, history, recipes all key on "circle"); only the
  // labels tell the truth now.
  { value: "circle:9:8", label: "Octagon search (8\u00D745\u00B0, SAR)" },
  { value: "circle:9:36", label: "Circle (smooth, 36-gon)" },
  { value: "sector:15:3", label: "Sector search (SAR)" },
  { value: "parallel:30:5:8", label: "Parallel track search (SAR)" },
  { value: "expsquare:5:12", label: "Expanding square search (SAR)" },
  { value: "lissajous:15:1:2", label: "Lissajous 1:2" },
  { value: "lissajous:15:5:7", label: "Lissajous 5:7" },
  { value: "lissajous:15:11:9", label: "Lissajous 11:9" },
  { value: "spiro:9.0:8", label: "Spirograph (8-lobe rosette)" },
  // The per-angle cornering probe: ONE isolated corner with a real
  // approach and exit. Listed at 45 deg as a starting point - the angle
  // is the second parameter and is meant to be swept (corner:25:90,
  // corner:25:135, ...) via the mission field or the API.
  { value: "corner:25:45", label: "Corner probe (25m, 45\u00B0)" },
];
// DERIVED FROM THE SERVER, not hardcoded. This was a literal 5-name list
// while server.py's GAITS has had EIGHT for a long time - bounding(1),
// pronking(2) and galloping(22) were simply missing, so three of this
// project's gaits could not be selected from the panel at all even though
// /api/launch accepts them and CLAUDE.md is full of measured results for
// them. Operator-visible consequence: the flight-gait dash investigation
// could be run by script but not reproduced by hand from the UI.
//
// Same duplicated-source-of-truth defect as the draft slots carrying their
// own gait/speed literals beside RECIPES (which drifted the oval onto a
// configuration already measured broken) - so it gets the same treatment:
// one source, read live. The fallback list is only for the first paint,
// before the first poll lands.
let GAITS = ["trotRunning", "trotting", "walking", "walking2", "pacing"];
function gaitChoices() {
  const g = state && state.gaits;
  if (!g) return GAITS;
  // Sorted by the numeric SIM_GAIT id so the order is stable and matches
  // how they are indexed everywhere else, rather than hash order.
  return Object.keys(g).sort((a, b) => g[a] - g[b]);
}

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
// True while a slot removal is in flight. Every slot input (including the
// OTHER remove buttons) gets disabled during it - a removal changes every
// later slot's index, and any second click/edit captured against the
// pre-removal index would act on the wrong slot or DELETE an already-
// shifted one out from under a different slot. See the removal handler.
let _removing = false;
let _uiRev = null;   // the ui_rev this page booted with - see poll()

function kindOf(spec) { return spec.split(":")[0]; }

// state.recipes stores gait as the numeric index server.py/GAITS use
// internally; the draft slots (and this UI) store it as the name string
// the <select> shows. One place to convert between them so the mission-
// change handler and the mismatch warning below can never disagree.
function gaitNameForIndex(idx) {
  return Object.keys(state.gaits || {}).find(name => state.gaits[name] === idx);
}

function renderSlots() {
  const el = document.getElementById("slots");
  // Only an ACTIVE fleet locks the draft - "done" used to leave every input
  // disabled until a manual Stop, which read as the whole UI being broken.
  const locked = state.phase === "launching" || state.phase === "running" || _removing;
  // While a fleet is ACTIVE, show the RUNNING fleet's own slots - exactly
  // as many dogs as are actually running, with their actual launched
  // config - not the draft (operator, 2026-08-28: "each run slots should
  // be clear, and only the slots for the current run should be shown,
  // never stale or unused dogs"). A body-launch (mission_runner) never
  // touches the draft, so during automated runs the draft was showing
  // three leftover default dogs while one ran. The draft comes back,
  // untouched, the moment the fleet ends. Index alignment is preserved by
  // construction (locked slot i IS dog i IS draft slot i for the live
  // camera controls, which read/write the draft server-side).
  const running = locked && !_removing && (state.slots || []).length > 0;
  const view = running
    ? state.slots.map(s => ({ ...s, gait: s.gait_name || s.gait }))
    : slots;
  const tk = state.terrain;
  const tnote = (state.terrain_types && state.terrain_types[tk] || {}).note || "";
  const banner = running
    ? `<div class="cap-hint" style="margin-bottom:0.4rem">Showing the RUNNING fleet's ${view.length} slot(s); the draft returns when it ends.<br><b>Terrain: ${tk || "?"}</b>${tnote ? " - " + tnote : ""}</div>`
    : "";
  el.innerHTML = banner + view.map((s, i) => {
    const recipe = state.recipes[kindOf(s.mission)] || {};
    // Flag any drift from this course's own measured-good combo - a gait
    // swap that leaves a stale gait/speed behind (the bug that spun the
    // atom out running trotRunning@3.5, star/oval's profile, under a note
    // that still read "trotting @ 2.1") used to fail silently. The mission
    // change now snaps both together, but the gait/speed fields stay
    // freely editable afterward (some A/B work wants that), so this warns
    // rather than blocks whenever the ACTIVE combo has drifted from the
    // recipe, from either side.
    const recipeGait = recipe.gait !== undefined ? gaitNameForIndex(recipe.gait) : null;
    const gaitOff = recipeGait && s.gait !== recipeGait;
    // Compare against the recipe speed CLAMPED BY THIS SLOT'S MODEL CEILING,
    // not the raw recipe number. A Go1 Air (cap 2.5) selecting the oval
    // (recipe 3.5) snaps to 2.5 - the best that model can legally run - and
    // the raw comparison then flagged it "not the validated combo" forever,
    // an unclearable warning about a limit the operator cannot lift from
    // here (the cap is enforced server-side on purpose). Audited across
    // every mission option x model: this was the only fresh-selection path
    // that warned. The model radio already shows the ceiling.
    const modelCap = (state.model_max_speed || { air: 2.5, pro: 3.5, edu: 4.7 })[s.model || "edu"] ?? 3.9;
    const recipeSpeedHere = typeof recipe.speed === "number" ? Math.min(recipe.speed, modelCap) : null;
    const speedOff = recipeSpeedHere !== null && Math.abs(s.speed - recipeSpeedHere) > 0.05;
    const offRecipe = gaitOff || speedOff;
    return `<div class="play-card active">
      <h3>Dog ${i}</h3>
      ${(() => {
        // Recipe notes grew from one-liners into documentation paragraphs;
        // dumped raw they read as corrupted debug text on the card
        // (operator-reported). Show the FIRST SENTENCE only, full note on
        // hover.
        const note = recipe.note || "";
        const first = note.split(". ")[0];
        const brief = first.length > 110 ? first.slice(0, 107) + "..." : first;
        return note ? `<p title="${note.replace(/"/g, "&quot;")}">${brief}${note.length > brief.length ? " …" : ""}</p>` : "";
      })()}
      ${offRecipe ? (running
        // A LOCKED, already-running card cannot be corrected, so the alarm
        // styling is wrong there - scripted probes (terrain matrix, corner
        // sweep) run off-recipe ON PURPOSE. Neutral information instead.
        ? `<p class="cap-hint">off-recipe probe (course recipe: ${recipeGait || "?"} @ ${recipe.speed} m/s)</p>`
        : `<p class="gait-warning">&#9888; not this course's validated combo - ` +
          `recipe: ${recipeGait || "?"} @ ${recipe.speed} m/s` +
          `${recipe.extra ? " (" + recipe.extra + ")" : ""}</p>`) : ""}
      <div class="slot-row">
        <label>Mission
          <select data-i="${i}" data-f="mission" ${locked ? "disabled" : ""}>
            ${(() => {
              // Exact match first (needed to tell the three Lissajous
              // ratios apart - they share a kind). If the slot carries a
              // customized mission string (non-default radius/points,
              // e.g. set via the REST API rather than this dropdown),
              // fall back to the first option of the same KIND so the
              // dropdown still shows something sensible rather than
              // nothing selected.
              const exact = MISSION_OPTIONS.some(o => o.value === s.mission);
              return MISSION_OPTIONS.map(opt => {
                const sel = exact ? s.mission === opt.value
                                   : kindOf(s.mission) === kindOf(opt.value) &&
                                     opt.value === MISSION_OPTIONS.find(
                                       o => kindOf(o.value) === kindOf(s.mission))?.value;
                return `<option value="${opt.value}" ${sel ? "selected" : ""}>${opt.label}</option>`;
              }).join("");
            })()}
          </select>
        </label>
        <label>Gait${gaitOff ? ' <span class="gait-warning-dot" title="not the recipe gait">&#9888;</span>' : ""}
          <select data-i="${i}" data-f="gait" ${locked ? "disabled" : ""}>
            ${gaitChoices().map(g => `<option ${s.gait === g ? "selected" : ""}>${g}</option>`).join("")}
          </select>
        </label>
        <label>Speed cmd (m/s)${speedOff ? ' <span class="gait-warning-dot" title="not the recipe speed">&#9888;</span>' : ""}
          <input type="number" step="0.1" min="0.3" max="3.9" value="${s.speed}"
                 data-i="${i}" data-f="speed" ${locked ? "disabled" : ""}>
        </label>
        <label>Dash finish (m)
          <input type="number" step="10" min="0" max="200" value="${s.dash ?? 100}"
                 title="Straight sprint appended after the loop closes - stop, lie down, stand back up, then dash. 0 = no dash, mission ends at the loop."
                 data-i="${i}" data-f="dash" ${locked ? "disabled" : ""}>
        </label>
        <label style="justify-content:flex-end">
          <button class="slot-remove" data-remove="${i}" ${locked ? "disabled" : ""}>&times; remove</button>
        </label>
      </div>
      <div class="slot-row cam-config-row">
        <label class="cam-check"><input type="checkbox" data-i="${i}" data-f="dash_toggle"
               ${(s.dash ?? 0) > 0 ? "checked" : ""} ${locked ? "disabled" : ""}
               title="Quick toggle for the field above - checked sets a 100m dash finish, unchecked sets 0 (mission ends at the loop)."> 100m dash when done</label>
        <label class="cam-check"><input type="checkbox" data-i="${i}" data-f="close_leg"
               ${s.close_leg !== false ? "checked" : ""} ${locked ? "disabled" : ""}
               title="Walk the last leg back to the start point. Some courses end where they began (lissajous/spiro/atom/oval land within ~1m); others just stop where their shape ran out - circle 6.9m, sector 15m, expsquare 18m, parallel 46m - leaving a visible unclosed leg on the overlay. No effect on a dash (one waypoint) or an already-closed curve."> Close final leg</label>
      </div>
      <div class="slot-row cam-config-row">
        ${["air", "pro", "edu"].map(m => `
        <label class="cam-check" title="Hard per-model speed ceiling, enforced server-side on top of the fleet cap - the speed command can never exceed it. Edu's 4.7 is Unitree's peak-sprint figure (optimised test conditions), not a sustained rating.">
          <input type="radio" name="model-${i}" data-i="${i}" data-f="model" value="${m}"
                 ${(s.model || "edu") === m ? "checked" : ""} ${locked ? "disabled" : ""}>
          Go1 ${m[0].toUpperCase() + m.slice(1)} (&le;${(state.model_max_speed || { air: 2.5, pro: 3.5, edu: 4.7 })[m]} m/s)</label>`).join("")}
      </div>
      <div class="slot-row cam-config-row">
        ${(() => {
          // Camera checkboxes are LIVE while a camera is streaming (the
          // server gates each frame on the draft flag), but a camera that
          // was OFF at launch was never spawned in the world, so mid-run
          // the box can do nothing - which read as "dead checkbox"
          // (operator-reported, 2026-08-28: suite runs launch camera-dark
          // via mission_runner, leaving every cam box inert during them).
          // Disable + explain in exactly that case; everywhere else the
          // box behaves as before.
          const ls = (state.slots || []).find(x => x.index === i);
          const fleetUp = state.phase === "launching" || state.phase === "running";
          const inert = k => fleetUp && ls && ls[k] === false;
          const camBox = (k, label) => `
        <label class="cam-check" title="${inert(k)
            ? "OFF at launch for the RUNNING fleet - the sensor was never spawned, so it cannot come on mid-run. The box takes effect at the next launch."
            : "Live: unchecking mid-run mutes the stream instantly, re-checking resumes it. A camera unchecked at LAUNCH is never spawned in the world and cannot come back mid-run."}"><input type="checkbox" data-i="${i}" data-f="${k}"
               ${(running ? ((slots[i] || s)[k] !== false) : (s[k] !== false)) ? "checked" : ""} ${inert(k) || _removing ? "disabled" : ""}> ${label}</label>`;
          const anyInert = ["cam_front", "cam_nadir", "cam_chase"].some(inert);
          // VISIBLE text, not just a hover tooltip - the operator clicked a
          // greyed box twice and "got nothing" before this existed.
          const why = anyInert
            ? `<div class="cap-hint" style="flex-basis:100%" title="A camera unchecked at launch is never spawned in the world; only a relaunch (panel checkbox, or mission_runner --chase) enables it.">cams off at launch never spawn - relaunch to enable</div>`
            : "";
          return camBox("cam_front", "Front cam")
               + camBox("cam_nadir", "Nadir cam")
               + camBox("cam_chase", "Chase cam")
               + why;
        })()}
      </div>
      ${(() => {
        // These three are deliberately NOT disabled while a fleet runs:
        // _follow_chase_cams reads them from the DRAFT every tick precisely
        // so a mid-run adjustment lands within ~100ms. The old blanket
        // `locked ? disabled` here was blocking the exact live feature the
        // server implements (found chasing the 2026-08-28 camera report).
        // They DO disable when the running fleet never spawned this dog's
        // chase cam (nothing to move) and during a slot removal.
        const ls = (state.slots || []).find(x => x.index === i);
        const fleetUp = state.phase === "launching" || state.phase === "running";
        const chaseInert = (fleetUp && ls && ls.cam_chase === false) || _removing;
        const dis = chaseInert ? "disabled" : "";
        // In the running view s is the LOCKED slot (frozen at launch); the
        // sliders are LIVE via the draft, so their VALUES must render from
        // the draft too or every drag snaps back on the next poll - the
        // exact bug the chase checkbox had (operator: "the checkbox staid
        // lit").
        const live = running ? (slots[i] || s) : s;
        return `<div class="slot-row cam-config-row">
        <label>Chase dist (m)
          <input type="number" step="0.5" min="0.5" max="10" value="${live.chase_distance ?? 3.0}"
                 data-i="${i}" data-f="chase_distance" ${dis}>
        </label>
        <label>Chase height (m)
          <input type="number" step="0.1" min="0.1" max="5" value="${live.chase_height ?? 1.2}"
                 data-i="${i}" data-f="chase_height" ${dis}>
        </label>
        <label>Chase angle (deg)
          <input type="number" step="15" min="-360" max="360" value="${live.chase_degree ?? 90}"
                 title="0 = directly behind, 90 = left side, -90 = right side"
                 data-i="${i}" data-f="chase_degree" ${dis}>
        </label>
      </div>`;
      })()}
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
      const body = { [f]: slots[i][f] };
      // Switching the MISSION KIND must also snap gait/speed/extra to that
      // course's own tuned recipe - each course's stable envelope was
      // measured against a specific gait+speed+extra combination (e.g. the
      // atom needs trotting@2.1 with WP_ALON=0.4; trotRunning@3.5 is star/
      // oval's profile and spins the atom out on its own curvature). Without
      // this, picking a new mission left the PREVIOUS mission's gait/speed
      // in place - the note text updated (it is looked up fresh from
      // state.recipes every render) but the actual command did not, so the
      // panel silently ran an untested, wrong-gait combination.
      if (f === "mission") {
        // END-OF-MISSION DEFAULTS, mirroring server.py's kind_slot_defaults()
        // so the checkboxes flip the moment the dropdown changes instead of
        // waiting a poll tick. A standalone dash wants NEITHER a sprint
        // finish (it already is one) nor a closing leg (that would make it
        // an out-and-back); every loop course wants both. The server applies
        // the same rule authoritatively on the same POST - this is purely
        // for instant feedback, and the values are sent explicitly so the
        // two can never disagree about what was intended.
        const nk = kindOf(slots[i].mission);
        const isDash = (nk === "dash" || nk === "outback");
        slots[i].dash = isDash ? 0 : 100;
        slots[i].close_leg = !isDash;
        body.dash = slots[i].dash;
        body.close_leg = slots[i].close_leg;
        const recipe = state.recipes[kindOf(slots[i].mission)];
        if (recipe) {
          const gaitName = gaitNameForIndex(recipe.gait);
          if (gaitName) { slots[i].gait = gaitName; body.gait = gaitName; }
          const cap = (state.model_max_speed || { air: 2.5, pro: 3.5, edu: 4.7 })[slots[i].model || "edu"] ?? 3.9;
          slots[i].speed = Math.min(recipe.speed, cap);
          body.speed = slots[i].speed;
          slots[i].extra = recipe.extra || "";
          body.extra = slots[i].extra;
        }
      }
      fetch("/api/slots/" + i, {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      renderSlots();
    });
  });
  el.querySelectorAll("[data-remove]").forEach(b => {
    b.addEventListener("click", async e => {
      const i = +e.target.dataset.remove;
      // Speculatively splicing the LOCAL array here (as this used to do)
      // raced with itself: a second remove clicked before this one's
      // response came back captured an index against the PRE-removal
      // layout, so it could delete the wrong slot server-side, or send an
      // out-of-range DELETE that silently no-ops while the panel shows
      // something the server no longer agrees with. Instead: lock every
      // slot control immediately (via _removing, see renderSlots), wait
      // for the server's answer, and adopt ITS slot list - the same
      // server-truth pattern addSlot() already uses - so there is no
      // window for a second click to act on a stale index.
      _removing = true;
      renderSlots();
      try {
        const r = await fetch("/api/slots/" + i, { method: "DELETE" });
        const j = await r.json();
        if (j.ok) slots = j.slots;
        else alert(j.message || "could not remove slot");
      } catch (err) {
        alert("remove failed: " + err);
      } finally {
        _removing = false;
        renderSlots();
      }
    });
  });
  document.getElementById("addSlot").disabled = locked || slots.length >= 3;
  const clearBtn = document.getElementById("clearSlots");
  if (clearBtn) clearBtn.disabled = locked || slots.length === 0;
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
      const cls = r.phase === "complete" ? "complete" : (r.phase === "fell" || r.phase === "invalid") ? "fell" : "";
      const dot = r.phase === "complete" ? "up" : (r.phase === "fell" || r.phase === "invalid") ? "down" : "mid";
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
          <span class="fleet-name">${(MISSION_OPTIONS.find(o => o.value === s.mission) || {}).label || s.mission || ""}</span>
          <small style="opacity:.6">${s.mission || ""}</small>
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
  // Pin to the bottom ONLY when the operator is already there - and decide
  // that BEFORE the re-render grows scrollHeight, or a busy tick reads a
  // just-appended chunk as "scrolled up" and stops following for everyone.
  // An unconditional scroll made reading history impossible: every poll
  // yanked the view back down mid-read (operator: "if I scroll up ...
  // I'm trying to see where it tells me the terrain sdf it selected").
  const lb = document.getElementById("logBox");
  const atBottom = lb.scrollHeight - lb.scrollTop - lb.clientHeight < 40;
  lb.textContent = (state.log || []).join("\n");
  if (atBottom) lb.scrollTop = 1e9;

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
    // Adopt the SERVER's draft whenever it differs from what this page is
    // showing - not just once on load. The one-time-only version left the
    // page permanently stale the moment ANYTHING else touched the draft
    // after initial sync (mission_runner.py, curl against /api/slots/{i},
    // another browser tab) - "recipe: trotRunning @ 3 m/s" mismatch
    // warnings from an external change never appeared without a manual
    // page refresh, which is exactly what re-runs this one-time branch.
    // Guarded two ways so it cannot fight the user's own typing: skip
    // entirely while focus is inside a slot control (mid-edit), and skip
    // the render when the incoming draft is byte-identical to what is
    // already showing (the common case, every 400ms, must stay a no-op).
    // Stale-tab killer: if the panel's own JS/HTML changed on disk since
    // this page loaded, reload to pick it up (guarded so it never eats an
    // in-progress edit). Without this, a UI fix only reaches a tab that
    // happens to get manually refreshed - which is exactly how a verified
    // fix looked like no fix at all to the operator (2026-08-28).
    const editingASlot = document.activeElement &&
      document.activeElement.closest && document.activeElement.closest(".play-card");
    if (state.ui_rev) {
      if (_uiRev === null) _uiRev = state.ui_rev;
      else if (state.ui_rev !== _uiRev && !editingASlot) { location.reload(); return; }
    }
    if (state.draft_slots && !editingASlot) {
      const incoming = JSON.stringify(state.draft_slots);
      if (!_synced || incoming !== JSON.stringify(slots)) {
        slots = state.draft_slots;
        document.getElementById("capSlider").value = state.draft_cap ?? 3.5;
        syncTerrainOptions(state);
        if (state.draft_terrain) document.getElementById("terrainSelect").value = state.draft_terrain;
        renderSlots();
      }
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
function syncTerrainOptions(state) {
  // Populate the terrain dropdown from the SERVER's terrain_types instead
  // of a hardcoded list - the old markup shipped flat/rolling/rough only,
  // so the surface kinds (concrete/grass/sand/mud/ice/... - and even the
  // pre-existing "ramp") were selectable over REST but invisible in the
  // panel. Each option's tooltip carries the kind's own note (mu etc.).
  const kinds = state.terrain_types;
  if (!kinds) return;
  const sel = document.getElementById("terrainSelect");
  const want = Object.keys(kinds);
  const have = [...sel.options].map(o => o.value);
  if (want.length === have.length && want.every((k, i) => k === have[i])) return;
  const cur = sel.value;
  sel.innerHTML = want.map(k =>
    `<option value="${k}" title="${(kinds[k].note || "").replace(/"/g, "&quot;")}">` +
    `${k}${kinds[k].note ? " - " + kinds[k].note : ""}</option>`).join("");
  if (want.includes(cur)) sel.value = cur;
}

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
document.getElementById("clearSlots").addEventListener("click", async () => {
  if (!slots.length) return;
  // Same _removing guard the per-slot remove uses: disable every slot
  // control synchronously so nothing can act on a stale index while the
  // DELETE is in flight, and adopt the SERVER's returned list as truth
  // rather than mutating locally and hoping they agree.
  _removing = true;
  renderSlots();
  try {
    const r = await fetch("/api/slots", { method: "DELETE" });
    const j = await r.json();
    if (j.ok) slots = j.slots;
    else alert(j.message || "could not clear slots");
  } catch (err) {
    alert("clear failed: " + err);
  } finally {
    _removing = false;
    renderSlots();
  }
});
document.getElementById("launchBtn").addEventListener("click", async e => {
  // Disable and relabel INSTANTLY, before the fetch - renderFleet() only
  // disables this button once a poll tick (every 400ms) observes
  // phase leave "done"/"idle", which left a window where a click had
  // visibly done nothing yet. A user re-clicking into that window fired a
  // SECOND /api/launch, which the server correctly refuses ("a fleet is
  // already active") - but that refusal surfaced as alert(), a BLOCKING
  // modal that freezes all page JS (including the poll loop) until
  // dismissed. Stacking several of those from a few impatient clicks is
  // exactly what "needs clicked several times to react" looks like from
  // outside: the first click had already launched the fleet, but the page
  // looked frozen behind a pile of dialogs earned by the clicks after it.
  const btn = e.currentTarget;
  btn.disabled = true;
  const prevText = btn.textContent;
  btn.textContent = "Launching...";
  const cap = +document.getElementById("capSlider").value;
  const terrainVal = document.getElementById("terrainSelect").value;
  try {
    const r = await fetch("/api/launch", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ slots, speed_cap: cap, terrain: terrainVal }),
    });
    const j = await r.json();
    if (!j.ok) {
      // Refused (not actually launching) - restore the button ourselves;
      // renderFleet()'s poll-driven disable only fires once phase is
      // really "launching"/"running", which never happened here.
      btn.disabled = false;
      btn.textContent = prevText;
      alert(j.message || j.error);
    }
    // On success, leave it disabled with "Launching..." - the next poll
    // tick's renderFleet() takes over from here as phase actually moves.
  } catch (err) {
    btn.disabled = false;
    btn.textContent = prevText;
    alert("launch failed: " + err);
  }
});
document.getElementById("stopBtn").addEventListener("click", async () => {
  await fetch("/api/stop", { method: "POST" });
});

renderSlots();
poll();
