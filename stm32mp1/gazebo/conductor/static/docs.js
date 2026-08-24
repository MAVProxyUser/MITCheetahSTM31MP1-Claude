// API docs page - vanilla JS, no build step, matching the rest of the
// Conductor. Each entry below is the single source of truth for one route:
// rendering, the curl command, and the actual "Play" fetch all come from it.

const BASE = window.location.origin;
document.getElementById("baseUrl").textContent = BASE;

const ENDPOINTS = [
  {
    method: "GET", path: "/api/state", stateChanging: false,
    desc: "Full snapshot of everything the panel shows: phase, locked slots " +
          "(once launched), live per-dog status, the last 60 log lines, the " +
          "draft fleet config, planned/flown paths, live positions " +
          "(x/y/z/yaw/speed), camera frames, host CPU/GPU load, and the " +
          "terrain/gait/recipe tables. Polled every 400ms by the browser - " +
          "this one route is the entire state of the app.",
  },
  {
    method: "POST", path: "/api/launch", stateChanging: true,
    desc: "Freeze a config and launch the fleet. Refuses a second call " +
          "while one is already <code>launching</code> or <code>running</code> " +
          "- stop it first. Every field is optional: omit all of them (send " +
          "<code>{}</code>) to launch exactly what the draft panel is " +
          "currently showing, which is what clicking \"Launch fleet\" in the " +
          "browser does under the hood.",
    body: {
      slots: [{ mission: "star:10.514:5", gait: "trotRunning", speed: 3.5, dash: 100 }],
      speed_cap: 3.9,
      terrain: "flat",
    },
  },
  {
    method: "POST", path: "/api/stop", stateChanging: true,
    desc: "Terminate every process this run started (gz sim, bridges, " +
          "controllers) and return the fleet to <code>idle</code>. No body.",
  },
  {
    method: "POST", path: "/api/slots/add", stateChanging: true,
    desc: "Append a slot to the DRAFT fleet (max 3), auto-picking a mission " +
          "kind not already in use. No body. Refused once at 3 slots.",
  },
  {
    method: "POST", path: "/api/slots/{i}", stateChanging: true, hasIndex: true,
    desc: "Edit one field or several on draft slot <code>{i}</code> (0-based). " +
          "Any subset - send only what you're changing. <code>dash</code> is " +
          "metres of straight finishing sprint appended after the loop closes " +
          "(0 = no dash). <code>cam_front</code>/<code>cam_nadir</code>/" +
          "<code>cam_chase</code> enable or disable that dog's three feeds - a " +
          "disabled camera is not even spawned in the world (real GPU/CPU " +
          "saved, not just hidden in the UI). <code>chase_distance</code> (m), " +
          "<code>chase_height</code> (m) and <code>chase_degree</code> " +
          "(0 = directly behind, 90 = left side, -90 = right side) position the " +
          "chase cam, body-mounted so it rides with the dog for free. Speed is " +
          "clamped to the 3.9 m/s hard cap server-side regardless of what's sent.",
    body: { mission: "star:10.514:5", gait: "trotRunning", speed: 3.5, dash: 100,
            cam_front: true, cam_nadir: true, cam_chase: true,
            chase_distance: 3.0, chase_height: 1.2, chase_degree: 90 },
  },
  {
    method: "DELETE", path: "/api/slots/{i}", stateChanging: true, hasIndex: true,
    desc: "Remove draft slot <code>{i}</code>. Refused if it's the last " +
          "remaining slot - at least one is required.",
  },
  {
    method: "POST", path: "/api/speed_cap", stateChanging: true,
    desc: "Set the draft fleet-wide speed ceiling, m/s. Clamped to " +
          "[0.3, 3.9] - the 3.9 hard cap is enforced here, not just in the " +
          "browser's <code>&lt;input max&gt;</code>, so it survives someone " +
          "editing devtools.",
    body: { value: 3.5 },
  },
  {
    method: "POST", path: "/api/terrain", stateChanging: true,
    desc: "Set the draft ground type. <code>flat</code> is the exact " +
          "ground_plane every campaign result in CLAUDE.md was measured on; " +
          "<code>rolling</code>/<code>rough</code>/<code>ramp</code> are " +
          "procedural and unvalidated. Unknown values are rejected.",
    body: { value: "flat" },
  },
];

function badge(m) { return `<span class="ep-method ${m}">${m}</span>`; }

function pathHtml(ep) {
  return ep.path.replace("{i}", '<span class="param">{i}</span>');
}

function buildCurl(ep, index, bodyText) {
  const url = BASE + ep.path.replace("{i}", index);
  let cmd = `curl -s`;
  if (ep.method !== "GET") cmd += ` -X ${ep.method}`;
  cmd += ` ${url}`;
  if (ep.body !== undefined) {
    cmd += ` -H "Content-Type: application/json" -d '${bodyText.trim()}'`;
  }
  return cmd;
}

function render() {
  const root = document.getElementById("endpoints");
  root.innerHTML = ENDPOINTS.map((ep, n) => `
    <div class="ep-card${ep.stateChanging ? " state-changing" : ""}" id="ep-${n}">
      <div class="ep-head" data-toggle="${n}">
        ${badge(ep.method)}
        <span class="ep-path">${pathHtml(ep)}</span>
        ${ep.stateChanging ? '<span class="ep-badge">state-changing</span>' : ""}
        <span class="ep-chevron">&#9656;</span>
      </div>
      <div class="ep-body">
        <div class="ep-desc">${ep.desc}</div>
        ${ep.hasIndex ? `
        <div class="ep-field">
          <label>Slot index (i)</label>
          <input type="number" min="0" max="2" value="0" data-index="${n}">
        </div>` : ""}
        ${ep.body !== undefined ? `
        <div class="ep-field">
          <label>Request body (JSON, editable)</label>
          <textarea class="ep-body-editor" data-body="${n}">${JSON.stringify(ep.body, null, 2)}</textarea>
        </div>` : ""}
        <div class="ep-actions">
          <button class="ep-btn play" data-play="${n}"><span class="glyph">&#9654;</span> Play</button>
          <button class="ep-btn" data-copy="${n}">Copy curl</button>
          <span class="ep-copied" data-copied="${n}">copied</span>
        </div>
        <pre class="ep-curl" data-curl="${n}"></pre>
        <div class="ep-response" data-response="${n}" style="display:none">
          <div class="ep-response-head">
            <span>Response</span>
            <span class="ep-status" data-status="${n}"></span>
          </div>
          <pre class="ep-response-body" data-response-body="${n}"></pre>
        </div>
      </div>
    </div>
  `).join("");

  root.querySelectorAll("[data-toggle]").forEach(el => {
    el.addEventListener("click", () => {
      document.getElementById("ep-" + el.dataset.toggle).classList.toggle("open");
      updateCurl(+el.dataset.toggle);
    });
  });
  root.querySelectorAll(".ep-body-editor, input[data-index]").forEach(el => {
    el.addEventListener("input", () => {
      const n = el.dataset.body ?? el.dataset.index;
      updateCurl(+n);
    });
  });
  root.querySelectorAll("[data-copy]").forEach(el => {
    el.addEventListener("click", () => copyCurl(+el.dataset.copy));
  });
  root.querySelectorAll("[data-play]").forEach(el => {
    el.addEventListener("click", () => play(+el.dataset.play));
  });

  ENDPOINTS.forEach((_, n) => updateCurl(n));
}

function getIndex(n) {
  const el = document.querySelector(`input[data-index="${n}"]`);
  return el ? el.value : "0";
}
function getBodyText(n) {
  const el = document.querySelector(`[data-body="${n}"]`);
  return el ? el.value : "";
}

function updateCurl(n) {
  const ep = ENDPOINTS[n];
  const el = document.querySelector(`[data-curl="${n}"]`);
  if (!el) return;
  el.textContent = buildCurl(ep, getIndex(n), getBodyText(n));
}

function copyCurl(n) {
  const text = document.querySelector(`[data-curl="${n}"]`).textContent;
  navigator.clipboard.writeText(text).then(() => {
    const badgeEl = document.querySelector(`[data-copied="${n}"]`);
    badgeEl.classList.add("show");
    setTimeout(() => badgeEl.classList.remove("show"), 1400);
  });
}

async function play(n) {
  const ep = ENDPOINTS[n];
  const btn = document.querySelector(`[data-play="${n}"]`);
  const respWrap = document.querySelector(`[data-response="${n}"]`);
  const respBody = document.querySelector(`[data-response-body="${n}"]`);
  const statusEl = document.querySelector(`[data-status="${n}"]`);

  const url = BASE + ep.path.replace("{i}", getIndex(n));
  const opts = { method: ep.method };
  if (ep.body !== undefined) {
    opts.headers = { "Content-Type": "application/json" };
    opts.body = getBodyText(n);
    try { JSON.parse(opts.body); }
    catch (e) {
      respWrap.style.display = "block";
      statusEl.textContent = "invalid JSON"; statusEl.className = "ep-status err";
      respBody.textContent = String(e);
      return;
    }
  }

  btn.disabled = true;
  const prevGlyph = btn.querySelector(".glyph").innerHTML;
  btn.querySelector(".glyph").innerHTML = "&#8987;";
  try {
    const r = await fetch(url, opts);
    const text = await r.text();
    let pretty = text;
    try { pretty = JSON.stringify(JSON.parse(text), null, 2); } catch (e) { /* not JSON, show raw */ }
    respWrap.style.display = "block";
    statusEl.textContent = r.status + " " + r.statusText;
    statusEl.className = "ep-status " + (r.ok ? "ok" : "err");
    respBody.textContent = pretty;
  } catch (e) {
    respWrap.style.display = "block";
    statusEl.textContent = "network error"; statusEl.className = "ep-status err";
    respBody.textContent = String(e);
  } finally {
    btn.disabled = false;
    btn.querySelector(".glyph").innerHTML = prevGlyph;
  }
}

render();
