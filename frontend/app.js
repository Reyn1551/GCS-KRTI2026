/* KRTI Ground Control Station - frontend logic
 *
 * All tunables live in CONFIG below:
 *  - camera stream URLs (MJPEG served by pi/camera_server.py on the Pi)
 *  - servo channel map: MAVLink servo number -> PCA9685 channel
 *    (must match SERVO_MAP in pi/pca_servo.py)
 *  - default open/close PWM values (placeholders until servo specs arrive)
 */

const CONFIG = {
  piHost: "192.168.10.179",
  camDownPort: 8080,   // Logitech webcam (down-facing)
  camFrontPort: 8081,  // RealSense D435i RGB (forward)
  defaultAlt: 10,
  map: {
    center: [-6.9147, 107.6098], // fallback view before first GPS fix
    zoom: 17,
    // online tiles - if the field WiFi has no internet, pre-cache tiles or
    // point this at a local tile server (e.g. http://<laptop>:8080/tile/{z}/{x}/{y}.png)
    tileUrl: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attribution: "Esri World Imagery",
  },
  // the two PCA9685 channels (0 and 1) are one group - the GCS sends a single
  // DO_SET_SERVO on FC output 10 and the Pi mirrors it onto both channels
  servos: [
    { name: "PAYLOAD (PCA ch0+1)", servo: 10, openPwm: 1900, closePwm: 1100 },
  ],
  trailLength: 200,
};

/* ============================== state ============================== */

let telemetry = {};
let mission = [];        // {id, type:'waypoint'|'servo', lat, lon, alt, servo, pwm}
let nextId = 1;
let addMode = true;
let homeSet = false;

const markers = new Map();   // mission item id -> L.marker (waypoints only)
let droneMarker = null;
let droneTrail = null;
let missionLine = null;

/* ============================== helpers ============================== */

const $ = (id) => document.getElementById(id);

function toast(msg, isError = false) {
  const el = $("toast");
  el.textContent = msg;
  el.classList.toggle("toast-error", isError);
  el.hidden = false;
  clearTimeout(el._t);
  el._t = setTimeout(() => (el.hidden = true), 3500);
}

async function api(path, body) {
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: body === undefined ? "{}" : JSON.stringify(body),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      toast(data.detail || `${path} failed (${res.status})`, true);
      return null;
    }
    return data;
  } catch (err) {
    toast(`backend unreachable: ${err.message}`, true);
    return null;
  }
}

const fmt = (v, digits = 1) =>
  v === null || v === undefined || Number.isNaN(v) ? "--" : Number(v).toFixed(digits);

/* ============================== map ============================== */

const map = L.map("map", { zoomControl: true }).setView(CONFIG.map.center, CONFIG.map.zoom);
L.tileLayer(CONFIG.map.tileUrl, {
  maxZoom: 19,
  attribution: CONFIG.map.attribution,
}).addTo(map);

missionLine = L.polyline([], { color: "#39c6a5", weight: 2, dashArray: "6 6" }).addTo(map);
droneTrail = L.polyline([], { color: "#e8a33d", weight: 2, opacity: 0.7 }).addTo(map);

function droneIcon(heading = 0) {
  return L.divIcon({
    className: "",
    html: `<div class="drone-marker"><svg width="26" height="26" viewBox="0 0 26 26"
        style="transform: rotate(${heading}deg)">
        <path d="M13 2 L20 22 L13 17 L6 22 Z" fill="#e8a33d" stroke="#0c0f13" stroke-width="1.5"/>
      </svg></div>`,
    iconSize: [0, 0],
  });
}

function wpIcon(seq) {
  return L.divIcon({
    className: "",
    html: `<div class="wp-marker">${seq}</div>`,
    iconSize: [0, 0],
  });
}

map.on("click", (e) => {
  if (!addMode) return;
  const alt = Number($("inp-alt").value) || CONFIG.defaultAlt;
  addWaypoint(e.latlng.lat, e.latlng.lng, alt);
});

/* ============================== mission model ============================== */

function addWaypoint(lat, lon, alt) {
  mission.push({ id: nextId++, type: "waypoint", lat, lon, alt });
  renderMission();
}

function addServoItem(servoDef, pwm) {
  mission.push({
    id: nextId++,
    type: "servo",
    servo: servoDef.servo,
    pwm,
    label: servoDef.name,
  });
  renderMission();
}

function moveItem(id, dir) {
  const i = mission.findIndex((m) => m.id === id);
  const j = i + dir;
  if (i < 0 || j < 0 || j >= mission.length) return;
  [mission[i], mission[j]] = [mission[j], mission[i]];
  renderMission();
}

function deleteItem(id) {
  mission = mission.filter((m) => m.id !== id);
  renderMission();
}

/* ============================== mission rendering ============================== */

function renderMission() {
  // markers
  for (const [id, marker] of markers) {
    if (!mission.find((m) => m.id === id)) {
      map.removeLayer(marker);
      markers.delete(id);
    }
  }

  const linePts = [];
  let wpSeq = 0;
  mission.forEach((item) => {
    if (item.type !== "waypoint") return;
    wpSeq += 1;
    let marker = markers.get(item.id);
    if (!marker) {
      marker = L.marker([item.lat, item.lon], {
        icon: wpIcon(wpSeq),
        draggable: true,
      }).addTo(map);
      marker.on("dragend", () => {
        const p = marker.getLatLng();
        item.lat = p.lat;
        item.lon = p.lng;
        renderMission();
      });
      markers.set(item.id, marker);
    } else {
      marker.setLatLng([item.lat, item.lon]);
      marker.setIcon(wpIcon(wpSeq));
    }
    linePts.push([item.lat, item.lon]);
  });
  missionLine.setLatLngs(linePts);

  // list
  const list = $("mission-list");
  list.innerHTML = "";
  const curSeq = telemetry.mission_seq; // vehicle-reported current mission item (0 = home)

  if (mission.length === 0) {
    const empty = document.createElement("div");
    empty.className = "mission-empty";
    empty.textContent = "empty - click the map to add waypoints";
    list.appendChild(empty);
  }

  mission.forEach((item, idx) => {
    const li = document.createElement("li");
    li.className = "mission-item" + (item.type === "servo" ? " mi-servo" : "");
    // seq N on the vehicle == mission[N-1] here (seq 0 is home)
    if (curSeq !== null && curSeq !== undefined && curSeq === idx + 1) {
      li.classList.add("mi-current");
    }

    const seq = document.createElement("span");
    seq.className = "mi-seq";
    seq.textContent = String(idx + 1).padStart(2, "0");
    li.appendChild(seq);

    const body = document.createElement("span");
    body.className = "mi-body";
    if (item.type === "waypoint") {
      body.textContent = `WP ${item.lat.toFixed(6)}, ${item.lon.toFixed(6)}`;
      body.title = body.textContent;
    } else {
      body.textContent = `SERVO ${item.servo} -> ${item.pwm}us`;
    }
    li.appendChild(body);

    if (item.type === "waypoint") {
      const alt = document.createElement("input");
      alt.className = "mi-alt";
      alt.type = "number";
      alt.min = 1;
      alt.max = 200;
      alt.value = item.alt;
      alt.title = "Altitude (m)";
      alt.onchange = () => {
        item.alt = Number(alt.value) || item.alt;
      };
      li.appendChild(alt);
    }

    const up = document.createElement("button");
    up.className = "mi-btn";
    up.textContent = "\u25B2";
    up.title = "Move up";
    up.onclick = () => moveItem(item.id, -1);
    li.appendChild(up);

    const down = document.createElement("button");
    down.className = "mi-btn";
    down.textContent = "\u25BC";
    down.title = "Move down";
    down.onclick = () => moveItem(item.id, 1);
    li.appendChild(down);

    const del = document.createElement("button");
    del.className = "mi-btn mi-del";
    del.textContent = "X";
    del.title = "Delete";
    del.onclick = () => deleteItem(item.id);
    li.appendChild(del);

    list.appendChild(li);
  });

  $("mission-count").textContent = `${mission.length} item${mission.length === 1 ? "" : "s"}`;
}

/* ============================== telemetry HUD ============================== */

const GPS_FIX_TEXT = {
  0: "NO GPS", 1: "NO FIX", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK FLT", 6: "RTK FIX",
};

function setPill(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.className = "pill" + (cls ? " " + cls : "");
}

function updateHUD() {
  const t = telemetry;

  setPill("pill-link", t.connected ? "LINK OK" : "NO LINK",
    t.connected ? "pill-ok" : "pill-danger");
  setPill("pill-mode", t.mode || "---", t.connected ? "pill-ok" : "");
  setPill("pill-arm", t.armed ? "ARMED" : "DISARMED", t.armed ? "pill-armed" : "");

  const fixText = GPS_FIX_TEXT[t.gps_fix] || "GPS --";
  setPill("pill-gps", `${fixText} ${t.satellites ?? "--"}`, t.gps_fix >= 3 ? "pill-ok" : "");

  const battV = fmt(t.battery_voltage, 1);
  const battP = t.battery_remaining;
  setPill("pill-batt", battP !== null && battP !== undefined ? `${battV}V ${battP}%` : `${battV} V`,
    battP !== null && battP !== undefined && battP < 25 ? "pill-danger" : "");

  $("t-alt").textContent = fmt(t.alt_rel, 1);
  $("t-gs").textContent = fmt(t.groundspeed, 1);
  $("t-hdg").textContent = fmt(t.heading, 0);
  $("t-vs").textContent = fmt(t.climb, 1);
  $("t-sats").textContent = t.satellites ?? "--";
  $("t-thr").textContent = t.throttle ?? "--";
  $("t-roll").textContent = fmt(t.roll, 0);
  $("t-pitch").textContent = fmt(t.pitch, 0);
  $("t-pos").textContent =
    t.lat !== null && t.lat !== undefined
      ? `${t.lat.toFixed(7)}, ${t.lon.toFixed(7)}  |  MSL ${fmt(t.alt_msl, 1)} m`
      : "no position";

  // drone marker + trail
  if (t.lat !== null && t.lat !== undefined && t.lat !== 0) {
    const pos = [t.lat, t.lon];
    if (!droneMarker) {
      droneMarker = L.marker(pos, { icon: droneIcon(t.heading || 0) }).addTo(map);
    } else {
      droneMarker.setLatLng(pos);
      droneMarker.setIcon(droneIcon(t.heading || 0));
    }
    const trail = droneTrail.getLatLngs();
    trail.push(pos);
    if (trail.length > CONFIG.trailLength) trail.shift();
    droneTrail.setLatLngs(trail);

    if (!homeSet) {
      map.setView(pos, CONFIG.map.zoom);
      homeSet = true;
    }
  }
}

/* ============================== websocket ============================== */

function connectTelemetry() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`);
  ws.onmessage = (ev) => {
    telemetry = JSON.parse(ev.data);
    updateHUD();
  };
  ws.onclose = () => setTimeout(connectTelemetry, 2000);
  ws.onerror = () => ws.close();
}

/* ============================== controls ============================== */

function wireControls() {
  // map toolbar
  $("btn-addmode").onclick = () => {
    addMode = !addMode;
    $("btn-addmode").textContent = `ADD WP: ${addMode ? "ON" : "OFF"}`;
    $("btn-addmode").classList.toggle("tb-active", addMode);
  };
  $("btn-add-current").onclick = () => {
    if (telemetry.lat === null || telemetry.lat === undefined) {
      toast("no GPS position yet", true);
      return;
    }
    const alt = Number($("inp-alt").value) || CONFIG.defaultAlt;
    addWaypoint(telemetry.lat, telemetry.lon, alt);
    toast("current position added");
  };
  $("btn-clear-mission").onclick = () => {
    mission = [];
    renderMission();
  };

  // mission
  $("btn-upload").onclick = async () => {
    if (mission.length === 0) {
      toast("mission is empty", true);
      return;
    }
    const items = mission.map((m) =>
      m.type === "waypoint"
        ? { type: "waypoint", lat: m.lat, lon: m.lon, alt: m.alt }
        : { type: "servo", servo: m.servo, pwm: m.pwm }
    );
    const res = await api("/api/mission/upload", { items });
    if (res) toast(`mission uploaded (${res.uploaded} items incl. home)`);
  };
  $("btn-start").onclick = async () => {
    const res = await api("/api/mission/start");
    if (res) toast("mission started (AUTO)");
  };
  $("btn-pause").onclick = async () => {
    const res = await api("/api/mission/pause");
    if (res) toast("holding position (LOITER)");
  };

  // flight
  $("btn-arm").onclick = async () => {
    if (!confirm("Arm the motors?")) return;
    const res = await api("/api/arm");
    if (res) toast("armed");
  };
  $("btn-disarm").onclick = async () => {
    if (!confirm("Disarm the motors?")) return;
    const res = await api("/api/disarm");
    if (res) toast("disarmed");
  };
  $("btn-takeoff").onclick = async () => {
    const alt = Number($("inp-takeoff-alt").value) || 5;
    if (!confirm(`Arm and take off to ${alt} m?`)) return;
    const res = await api("/api/takeoff", { altitude: alt });
    if (res) toast(`takeoff to ${alt} m commanded`);
  };
  $("btn-land").onclick = async () => {
    const res = await api("/api/land");
    if (res) toast("landing");
  };
  $("btn-rtl").onclick = async () => {
    const res = await api("/api/rtl");
    if (res) toast("returning to launch");
  };

  // servo mission item form
  const selServo = $("sel-servo");
  CONFIG.servos.forEach((s) => {
    const opt = document.createElement("option");
    opt.value = s.servo;
    opt.textContent = s.name;
    selServo.appendChild(opt);
  });
  const selAction = $("sel-servo-action");
  ["OPEN", "CLOSE", "CUSTOM"].forEach((a) => {
    const opt = document.createElement("option");
    opt.value = a;
    opt.textContent = a;
    selAction.appendChild(opt);
  });
  selAction.onchange = () => {
    $("inp-servo-pwm").hidden = selAction.value !== "CUSTOM";
  };
  $("btn-add-servo").onclick = () => {
    const def = CONFIG.servos.find((s) => s.servo === Number(selServo.value));
    let pwm;
    if (selAction.value === "OPEN") pwm = def.openPwm;
    else if (selAction.value === "CLOSE") pwm = def.closePwm;
    else pwm = Number($("inp-servo-pwm").value) || 1500;
    addServoItem(def, pwm);
  };

  // quick servo open/close buttons
  const wrap = $("servo-controls");
  CONFIG.servos.forEach((s) => {
    const row = document.createElement("div");
    row.className = "servo-row";

    const name = document.createElement("span");
    name.className = "srv-name";
    name.textContent = s.name;
    row.appendChild(name);

    const open = document.createElement("button");
    open.className = "btn btn-primary";
    open.textContent = "OPEN";
    open.onclick = async () => {
      const res = await api("/api/servo", { servo: s.servo, pwm: s.openPwm });
      if (res) toast(`${s.name} open (${s.openPwm}us)`);
    };
    row.appendChild(open);

    const close = document.createElement("button");
    close.className = "btn btn-warn";
    close.textContent = "CLOSE";
    close.onclick = async () => {
      const res = await api("/api/servo", { servo: s.servo, pwm: s.closePwm });
      if (res) toast(`${s.name} close (${s.closePwm}us)`);
    };
    row.appendChild(close);

    wrap.appendChild(row);
  });

  // cameras
  $("cam-down").src = `http://${CONFIG.piHost}:${CONFIG.camDownPort}/stream`;
  $("cam-front").src = `http://${CONFIG.piHost}:${CONFIG.camFrontPort}/stream`;
}

/* ============================== boot ============================== */

wireControls();
renderMission();
connectTelemetry();
