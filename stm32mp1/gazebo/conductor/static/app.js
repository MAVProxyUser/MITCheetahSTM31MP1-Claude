// Cheetah Conductor - vanilla JS, no build step. Polls /api/state, renders
// slots/status/log, and posts a FROZEN config to /api/launch. There is
// deliberately no "edit while running" path: fields disable the moment the
// fleet leaves "idle", because a code/param change mid-run means the run has
// to be redone, not patched.

const DEFAULT_MISSIONS = ["star:10.514:5", "oval:40:5.0", "atom:9.0:6"];
const MISSION_LABEL = { star: "Star (5pt, r=10.5)", oval: "Oval (40m + R5)", atom: "Atom (6-lobe, R=9)", dash: "Dash (100m straight)" };
const GAITS = ["trotRunning", "trotting", "walking", "walking2", "pacing"];

let state = { phase: "idle", recipes: {}, hard_cap: 3.9 };
let slots = DEFAULT_MISSIONS.map((m, i) => ({
  mission: m, gait: i === 2 ? "trotting" : "trotRunning", speed: i === 2 ? 2.1 : 3.5,
}));

function kindOf(spec) { return spec.split(":")[0]; }

function renderSlots() {
  const el = document.getElementById("slots");
  const locked = state.phase !== "idle";
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
        <label style="justify-content:flex-end">
          <button class="slot-remove" data-remove="${i}" ${locked || slots.length <= 1 ? "disabled" : ""}>&times; remove</button>
        </label>
      </div>
    </div>`;
  }).join("");

  el.querySelectorAll("select,input").forEach(elm => {
    elm.addEventListener("change", e => {
      const i = +e.target.dataset.i, f = e.target.dataset.f;
      slots[i][f] = f === "speed" ? +e.target.value : e.target.value;
      renderSlots();
    });
  });
  el.querySelectorAll("[data-remove]").forEach(b => {
    b.addEventListener("click", e => {
      slots.splice(+e.target.dataset.remove, 1);
      renderSlots();
    });
  });
  document.getElementById("addSlot").disabled = locked || slots.length >= 3;
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
      return `<div class="fleet-card ${cls}">
        <div class="fleet-head"><span class="dot ${dot}"></span>
          <span class="fleet-idx">#${r.index}</span>
          <span class="fleet-name">${s.mission || ""}</span>
          <span class="fleet-phase ${r.phase}">${r.phase}</span>
        </div>
        <div class="fleet-meta">gait=${s.gait || "-"} cmd=${s.speed ?? "-"} m/s
          ${r.waypoints ? "&middot; wp " + r.waypoints : ""}
          ${r.text ? "&middot; " + r.text : ""}
          ${r.t ? "&middot; t=" + r.t : ""}
        </div>
      </div>`;
    }).join("");
  }

  document.getElementById("logBox").textContent = (state.log || []).join("\n");
  document.getElementById("logBox").scrollTop = 1e9;

  const running = state.phase === "launching" || state.phase === "running";
  document.getElementById("launchBtn").disabled = running;
  document.getElementById("launchBtn").textContent =
    running ? "Fleet running..." : (state.phase === "done" ? "Re-launch fleet" : "Launch fleet");
  document.getElementById("stopBtn").disabled = !running;
  document.getElementById("caption").textContent =
    "STM32MP1 -> Go1 SITL fleet control - phase: " + state.phase;
}

async function poll() {
  try {
    const r = await fetch("/api/state");
    state = await r.json();
  } catch (e) { /* server restarting - ignore this tick */ }
  renderFleet();
  setTimeout(poll, 1000);
}

document.getElementById("capSlider").addEventListener("input", renderFleet);
document.getElementById("addSlot").addEventListener("click", () => {
  if (slots.length < 3) {
    slots.push({ mission: "star:10.514:5", gait: "trotRunning", speed: 3.5 });
    renderSlots();
  }
});
document.getElementById("launchBtn").addEventListener("click", async () => {
  const cap = +document.getElementById("capSlider").value;
  const r = await fetch("/api/launch", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ slots, speed_cap: cap }),
  });
  const j = await r.json();
  if (!j.ok) alert(j.message || j.error);
});
document.getElementById("stopBtn").addEventListener("click", async () => {
  await fetch("/api/stop", { method: "POST" });
});

renderSlots();
poll();
