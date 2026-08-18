/* KRTI Ground Control Station - Frontend Logic & Controls
 *
 * Config, telemetry handling, interactive map, resizable window splitters,
 * heading projection line, vehicle re-centering, mission planning & servo control.
 */

const CONFIG = {
  piHost: window.location.hostname || "localhost",
  camDownPort: 8080,   // Logitech webcam (down-facing)
  camFrontPort: 8081,  // RealSense D435i RGB (forward)
  defaultAlt: 1,
  map: {
    center: [-6.9147, 107.6098], // fallback view before first GPS fix
    zoom: 17,
    maxZoom: 22,      // allow very deep manual zoom-in
    tileMaxNative: 19,
    tileUrl: "/api/tiles/{z}/{x}/{y}",
    attribution: "Esri World Imagery",
  },
  servos: [
    { name: "PAYLOAD (PCA ch0+1)", servo: 10, openPwm: 1900, closePwm: 1100 },
  ],
  trailLength: 200,
  headingLineLength: 120, // meters projected forward
};

/* ============================== state ============================== */

let telemetry = {};
let mission = [];        // {id, type:'waypoint'|'servo'|'delay', lat, lon, alt, delay, servo, pwm}
let nextId = 1;
let selectedWaypointId = null;
let addMode = true;
let homeSet = false;
let followMode = false;

const markers = new Map();   // mission item id -> L.marker (waypoints only)
let droneMarker = null;
let droneTrail = null;
let missionLine = null;
let headingLine = null;

/* ============================== helpers ============================== */

const $ = (id) => document.getElementById(id);

function toast(msg, isError = false) {
  const el = $("toast");
  if (!el) return;
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

const map = L.map("map", { zoomControl: false, maxZoom: CONFIG.map.maxZoom }).setView(CONFIG.map.center, CONFIG.map.zoom);
L.control.zoom({ position: "bottomright" }).addTo(map);
L.tileLayer(CONFIG.map.tileUrl, {
  maxZoom: CONFIG.map.maxZoom,
  maxNativeZoom: CONFIG.map.tileMaxNative,
  attribution: CONFIG.map.attribution,
}).addTo(map);
L.control.scale({ imperial: false, position: "bottomleft" }).addTo(map);

const homeMarker = L.marker(CONFIG.map.center, { icon: homeIcon(), interactive: false }).addTo(map);
homeMarker.hideTimeout = null;

missionLine = L.polyline([], { color: "#39c6a5", weight: 2.5, dashArray: "6 6" }).addTo(map);
droneTrail = L.polyline([], { color: "#e8a33d", weight: 2, opacity: 0.75 }).addTo(map);
headingLine = L.polyline([], { color: "#00f2fe", weight: 3, dashArray: "6 4", opacity: 0.95, lineCap: "round" }).addTo(map);

function homeIcon() {
  return L.divIcon({
    className: "",
    html: `<div class="home-marker">
        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.4">
          <path d="M3 10.5 12 3l9 7.5"/>
          <path d="M5 9.5V21h14V9.5"/>
          <path d="M10 21v-6h4v6"/>
        </svg>
      </div>`,
    iconSize: [0, 0],
  });
}

function droneIcon(heading = 0) {
  return L.divIcon({
    className: "",
    html: `<div class="drone-marker"><svg width="28" height="28" viewBox="0 0 26 26"
        style="transform: rotate(${heading}deg)" fill="none" stroke="currentColor" stroke-width="1.6">
        <path d="M13 2 L20 22 L13 17 L6 22 Z" class="drone-body"/>
      </svg></div>`,
    iconSize: [0, 0],
  });
}

function wpIcon(seq, isSelected = false) {
  return L.divIcon({
    className: "",
    html: `<div class="wp-marker ${isSelected ? "wp-selected" : ""}">${seq}</div>`,
    iconSize: [0, 0],
  });
}

map.on("click", (e) => {
  if (!addMode) return;
  const alt = Number($("inp-alt").value) || CONFIG.defaultAlt;
  addWaypoint(e.latlng.lat, e.latlng.lng, alt);
});

// Auto-pause follow mode if user manually pans the map
map.on("dragstart", () => {
  if (followMode) {
    followMode = false;
    const btn = $("btn-toggle-follow");
    if (btn) {
      btn.textContent = "📍 FOLLOW: OFF";
      btn.classList.remove("tb-follow-active");
    }
    toast("Follow mode paused (manual pan)");
  }
});

/* ============================== heading projection ============================== */

function getHeadingEndpoint(lat, lon, headingDeg, distanceMeters = 120) {
  if (lat === null || lon === null || headingDeg === null || headingDeg === undefined) return null;
  const headingRad = (headingDeg * Math.PI) / 180;
  const dLat = (distanceMeters * Math.cos(headingRad)) / 111320;
  const dLon = (distanceMeters * Math.sin(headingRad)) / (111320 * Math.cos((lat * Math.PI) / 180));
  return [lat + dLat, lon + dLon];
}

/* ============================== vehicle centering ============================== */

function centerDrone(smooth = true) {
  if (telemetry.lat !== null && telemetry.lat !== undefined && telemetry.lat !== 0) {
    if (smooth) {
      map.panTo([telemetry.lat, telemetry.lon]);
    } else {
      map.setView([telemetry.lat, telemetry.lon], map.getZoom());
    }
    toast("Centered vehicle on map");
  } else {
    toast("No valid GPS position yet", true);
  }
}

function toggleFollow() {
  followMode = !followMode;
  const btn = $("btn-toggle-follow");
  if (btn) {
    btn.textContent = `📍 FOLLOW: ${followMode ? "ON" : "OFF"}`;
    btn.classList.toggle("tb-follow-active", followMode);
  }
  if (followMode) {
    centerDrone(true);
  }
}

/* ============================== mission model ============================== */

function addWaypoint(lat, lon, alt, delay = 0, radius = 2.0) {
  const item = { id: nextId++, type: "waypoint", lat, lon, alt, delay, radius };
  mission.push(item);
  selectedWaypointId = item.id;
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

function addDelayItem(sec) {
  mission.push({ id: nextId++, type: "delay", delay: sec });
  renderMission();
}

function updateWaypointToCurrentPos(id) {
  if (telemetry.lat === null || telemetry.lat === undefined || telemetry.lat === 0) {
    toast("No GPS position yet", true);
    return;
  }
  const item = mission.find((m) => m.id === id);
  if (!item || item.type !== "waypoint") return;

  item.lat = telemetry.lat;
  item.lon = telemetry.lon;
  renderMission();

  let wpSeq = 0;
  for (const m of mission) {
    if (m.type === "waypoint") wpSeq++;
    if (m.id === id) break;
  }
  toast(`WP ${String(wpSeq).padStart(2, "0")} updated to current position`);
}

function moveItem(id, dir) {
  const i = mission.findIndex((m) => m.id === id);
  const j = i + dir;
  if (i < 0 || j < 0 || j >= mission.length) return;
  [mission[i], mission[j]] = [mission[j], mission[i]];
  renderMission();
}

function deleteItem(id) {
  if (selectedWaypointId === id) selectedWaypointId = null;
  mission = mission.filter((m) => m.id !== id);
  renderMission();
}

/* ============================== mission persistence (localStorage) ============================== */

const MISSION_STORAGE_KEY = "krti_mission";

function saveMissionToStorage() {
  try {
    const data = { mission, nextId };
    localStorage.setItem(MISSION_STORAGE_KEY, JSON.stringify(data));
  } catch (e) {
    /* storage full or blocked — no big deal */
  }
}

function loadMissionFromStorage() {
  try {
    const raw = localStorage.getItem(MISSION_STORAGE_KEY);
    if (!raw) return false;
    const data = JSON.parse(raw);
    if (data && Array.isArray(data.mission) && data.mission.length > 0) {
      mission = data.mission;
      const maxId = mission.reduce((m, it) => Math.max(m, it.id || 0), 0);
      nextId = Math.max(maxId + 1, data.nextId || 1);
      return true;
    }
  } catch (e) {
    /* corrupt data, ignore */
  }
  return false;
}

/* ============================== mission rendering ============================== */

function renderMission() {
  for (const [id, marker] of markers) {
    if (!mission.find((m) => m.id === id)) {
      map.removeLayer(marker);
      markers.delete(id);
    }
  }

function saveMissionFile() {
  try {
    if (!mission || mission.length === 0) {
      toast("Cannot save: Mission list is empty! Add waypoints first.", true);
      return;
    }

    // Generate QGC WPL 110 ArduPilot standard format
    let lines = ["QGC WPL 110"];
    let seq = 0;

    // Home waypoint (seq 0)
    const homeLat = (telemetry.lat && telemetry.lat !== 0) ? telemetry.lat : (mission[0]?.lat || CONFIG.map.center[0]);
    const homeLon = (telemetry.lon && telemetry.lon !== 0) ? telemetry.lon : (mission[0]?.lon || CONFIG.map.center[1]);
    lines.push(`${seq++}\t1\t0\t16\t0\t0\t0\t0\t${homeLat.toFixed(7)}\t${homeLon.toFixed(7)}\t0.000000\t1`);

    for (const item of mission) {
      if (item.type === "waypoint") {
        lines.push(`${seq++}\t0\t3\t16\t${item.delay || 0}\t0\t0\t0\t${item.lat.toFixed(7)}\t${item.lon.toFixed(7)}\t${item.alt.toFixed(6)}\t1`);
      } else if (item.type === "servo") {
        const itemSeq = seq++;
        lines.push(`${itemSeq}\t0\t0\t183\t${item.servo}\t${item.pwm}\t0\t0\t0\t0\t0\t1`);
        if (item.autoClose) {
          lines.push(`# META_SERVO_AUTOCLOSE seq=${itemSeq} autoClose=1 closeDelay=${item.closeDelay || 2} closePwm=${item.closePwm || 1100}`);
        }
      } else if (item.type === "delay") {
        lines.push(`${seq++}\t0\t0\t93\t${item.delay}\t-1\t-1\t-1\t0\t0\t0\t1`);
      } else if (item.type === "arm") {
        lines.push(`${seq++}\t0\t0\t400\t1\t0\t0\t0\t0\t0\t0\t1`);
      } else if (item.type === "disarm") {
        lines.push(`${seq++}\t0\t0\t400\t0\t0\t0\t0\t0\t0\t0\t1`);
      } else if (item.type === "takeoff") {
        lines.push(`${seq++}\t0\t3\t22\t0\t0\t0\t0\t0\t0\t${item.alt.toFixed(6)}\t1`);
      } else if (item.type === "land") {
        lines.push(`${seq++}\t0\t3\t21\t0\t0\t0\t0\t0\t0\t0\t1`);
      }
    }

    const content = lines.join("\n");
    const blob = new Blob([content], { type: "text/plain;charset=utf-8" });
    const url = URL.createObjectURL(blob);
    const filename = `krti_mission_${new Date().toISOString().slice(0, 10)}.waypoints`;

    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    a.style.display = "none";
    document.body.appendChild(a);
    a.click();
    setTimeout(() => {
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    }, 100);

    toast(`Misi berhasil disimpan! (${mission.length} item tersimpan ke file ${filename})`);
  } catch (err) {
    toast(`Gagal menyimpan file misi: ${err.message}`, true);
  }
}
window.saveMissionFile = saveMissionFile;
window.loadMissionFile = loadMissionFile;

function loadMissionFile(file) {
  if (!file) {
    toast("Failed to load: File not found or invalid format", true);
    return;
  }

  const reader = new FileReader();
  reader.onerror = () => {
    toast(`Gagal membaca file ${file.name}: Terjadi kesalahan pada sistem file`, true);
  };

  reader.onload = (e) => {
    try {
      const text = e.target.result || "";

      // Try JSON format first
      if (file.name.endsWith(".json")) {
        const data = JSON.parse(text);
        if (Array.isArray(data) && data.length > 0) {
          mission = data.map((m, idx) => ({ ...m, id: m.id || (Date.now() + idx) }));
          selectedWaypointId = null;
          renderMission();
          toast(`Misi berhasil dimuat! (${mission.length} item dimuat dari file ${file.name})`);
          return;
        }
      }

      // Parse ArduPilot QGC WPL 110 or space/tab text
      const lines = text.split(/\r?\n/);
      const newMission = [];

      for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim();
        if (!line || line.startsWith("//") || line.startsWith("QGC WPL")) continue;

        if (line.startsWith("# META_SERVO_AUTOCLOSE")) {
          const mClose = line.match(/autoClose=1\s+closeDelay=([\d.]+)\s+closePwm=(\d+)/);
          if (mClose && newMission.length > 0) {
            const lastItem = newMission[newMission.length - 1];
            if (lastItem && lastItem.type === "servo") {
              lastItem.autoClose = true;
              lastItem.closeDelay = parseFloat(mClose[1]) || 2;
              lastItem.closePwm = parseInt(mClose[2], 10) || 1100;
            }
          }
          continue;
        }

        if (line.startsWith("#")) continue;

        const parts = line.split(/[\s\t]+/);
        if (parts.length >= 11) {
          const seq = Number(parts[0]);
          const cmd = Number(parts[3]);
          const p1 = Number(parts[4]);
          const p2 = Number(parts[5]);
          const lat = Number(parts[8]);
          const lon = Number(parts[9]);
          const alt = Number(parts[10]);

          if (seq === 0 && cmd === 16 && (lat === 0 || alt === 0)) continue;

          if (cmd === 16) {
            if (lat !== 0 || lon !== 0) {
              newMission.push({
                id: Date.now() + i,
                type: "waypoint",
                lat: lat,
                lon: lon,
                alt: alt > 0 ? alt : 1,
                delay: p1 > 0 ? p1 : 0
              });
            }
          } else if (cmd === 183) {
            newMission.push({
              id: Date.now() + i,
              type: "servo",
              servo: p1,
              pwm: p2,
              label: `Servo CH${p1}`
            });
          } else if (cmd === 93) {
            newMission.push({
              id: Date.now() + i,
              type: "delay",
              delay: p1
            });
          } else if (cmd === 400) {
            newMission.push({
              id: Date.now() + i,
              type: p1 === 1 ? "arm" : "disarm"
            });
          } else if (cmd === 22) {
            newMission.push({
              id: Date.now() + i,
              type: "takeoff",
              alt: alt > 0 ? alt : 1
            });
          } else if (cmd === 21) {
            newMission.push({
              id: Date.now() + i,
              type: "land"
            });
          }
        }
      }

      if (newMission.length > 0) {
        mission = newMission;
        selectedWaypointId = null;
        renderMission();
        toast(`Misi berhasil dimuat! (${mission.length} item dimuat dari file ${file.name})`);
      } else {
        toast(`Gagal memuat misi: Tidak ada item misi valid ditemukan di file ${file.name}`, true);
      }
    } catch (err) {
      toast(`Gagal memuat file misi ${file.name}: ${err.message}`, true);
    }
  };

  reader.readAsText(file);
}

  const linePts = [];
  let wpSeq = 0;
  mission.forEach((item) => {
    if (item.type !== "waypoint") return;
    wpSeq += 1;
    const isSelected = item.id === selectedWaypointId;
    let marker = markers.get(item.id);
    if (!marker) {
      marker = L.marker([item.lat, item.lon], {
        icon: wpIcon(wpSeq, isSelected),
        draggable: true,
      }).addTo(map);
      marker.on("dragend", () => {
        const p = marker.getLatLng();
        item.lat = p.lat;
        item.lon = p.lng;
        selectedWaypointId = item.id;
        renderMission();
      });
      marker.on("click", () => {
        selectedWaypointId = item.id;
        renderMission();
      });
      markers.set(item.id, marker);
    } else {
      marker.setLatLng([item.lat, item.lon]);
      marker.setIcon(wpIcon(wpSeq, isSelected));
    }
    linePts.push([item.lat, item.lon]);
  });
  missionLine.setLatLngs(linePts);

  const list = $("mission-list");
  if (!list) return;
  list.innerHTML = "";
  const curSeq = telemetry.mission_seq;

  if (mission.length === 0) {
    const empty = document.createElement("div");
    empty.className = "mission-empty";
    empty.innerHTML = `
      <div class="empty-title">Mission List Empty</div>
      <div class="empty-desc">Click anywhere on the map or use the <b>CURRENT DRONE POS</b> button to add waypoints.</div>
    `;
    list.appendChild(empty);
  }

  let selWpSeq = null;

  let draggedIndex = null;

  mission.forEach((item, idx) => {
    const li = document.createElement("li");
    li.className = "mission-item";
    li.draggable = true;
    li.dataset.index = String(idx);
    if (item.type === "servo") li.classList.add("mi-servo");
    if (item.type === "delay") li.classList.add("mi-delay-item");
    if (item.id === selectedWaypointId) {
      li.classList.add("mi-selected");
    }
    if (curSeq !== null && curSeq !== undefined && curSeq === idx + 1) {
      li.classList.add("mi-current");
    }

    // Drag handle icon (⋮⋮)
    const handle = document.createElement("span");
    handle.className = "mi-drag-handle";
    handle.innerHTML = "&#8942;&#8942;";
    handle.title = "Click & hold to drag and reorder mission sequence";
    li.appendChild(handle);

    // HTML5 Drag and Drop events
    li.ondragstart = (e) => {
      draggedIndex = idx;
      li.classList.add("mi-dragging");
      e.dataTransfer.effectAllowed = "move";
      e.dataTransfer.setData("text/plain", String(idx));
    };

    li.ondragover = (e) => {
      e.preventDefault();
      e.dataTransfer.dropEffect = "move";
      li.classList.add("mi-drag-over");

      // Auto-scroll list container when dragging near top/bottom
      const container = $("mission-list");
      if (container) {
        const rect = container.getBoundingClientRect();
        if (e.clientY < rect.top + 40) {
          container.scrollTop -= 10;
        } else if (e.clientY > rect.bottom - 40) {
          container.scrollTop += 10;
        }
      }
    };

    li.ondragleave = () => {
      li.classList.remove("mi-drag-over");
    };

    li.ondrop = (e) => {
      e.preventDefault();
      li.classList.remove("mi-drag-over");
      const fromIdx = draggedIndex !== null ? draggedIndex : Number(e.dataTransfer.getData("text/plain"));
      const toIdx = idx;
      if (fromIdx !== null && !isNaN(fromIdx) && fromIdx !== toIdx && fromIdx >= 0 && fromIdx < mission.length) {
        const [moved] = mission.splice(fromIdx, 1);
        mission.splice(toIdx, 0, moved);
        renderMission();
        toast(`Item moved to position #${String(toIdx + 1).padStart(2, "0")}`);
      }
    };

    li.ondragend = () => {
      draggedIndex = null;
      document.querySelectorAll(".mission-item").forEach((el) => {
        el.classList.remove("mi-dragging", "mi-drag-over");
      });
    };

    li.onclick = (e) => {
      if (e.target.tagName !== "INPUT" && e.target.tagName !== "BUTTON" && !e.target.closest("button") && !e.target.classList.contains("mi-drag-handle")) {
        selectedWaypointId = selectedWaypointId === item.id ? null : item.id;
        renderMission();
      }
    };

    const seq = document.createElement("span");
    seq.className = "mi-seq";
    seq.textContent = String(idx + 1).padStart(2, "0");
    li.appendChild(seq);

    const contentWrap = document.createElement("div");
    contentWrap.className = "mi-content-wrap";

    if (item.type === "waypoint") {
      let currentWpSeq = 0;
      for (const m of mission) {
        if (m.type === "waypoint") currentWpSeq++;
        if (m.id === item.id) break;
      }
      if (item.id === selectedWaypointId) {
        selWpSeq = currentWpSeq;
      }

      const header = document.createElement("div");
      header.className = "mi-header";
      header.innerHTML = `<span class="mi-badge mi-badge-wp">WAYPOINT ${currentWpSeq}</span> <span class="mi-coords">${item.lat.toFixed(6)}, ${item.lon.toFixed(6)}</span>`;
      contentWrap.appendChild(header);

      const fieldsRow = document.createElement("div");
      fieldsRow.className = "mi-fields-row";

      // Altitude field
      const altField = document.createElement("label");
      altField.className = "mi-field";
      altField.title = "Target relative altitude (meters) for this waypoint";
      altField.innerHTML = `<span>Altitude:</span>`;
      const altInp = document.createElement("input");
      altInp.className = "mi-alt";
      altInp.type = "number";
      altInp.min = 1;
      altInp.max = 200;
      altInp.value = item.alt;
      altInp.onchange = () => {
        item.alt = Number(altInp.value) || item.alt;
      };
      altField.appendChild(altInp);
      altField.appendChild(document.createTextNode("m"));
      fieldsRow.appendChild(altField);

      // Delay field
      const delayField = document.createElement("label");
      delayField.className = "mi-field";
      delayField.title = "Hold delay (seconds) at this waypoint before proceeding";
      delayField.innerHTML = `<span>Delay:</span>`;
      const delayInp = document.createElement("input");
      delayInp.className = "mi-delay";
      delayInp.type = "number";
      delayInp.min = 0;
      delayInp.max = 3600;
      delayInp.step = 1;
      delayInp.value = item.delay ?? 0;
      delayInp.onchange = () => {
        item.delay = Math.max(0, Number(delayInp.value) || 0);
      };
      delayField.appendChild(delayInp);
      delayField.appendChild(document.createTextNode("s"));
      fieldsRow.appendChild(delayField);

      // Radius field (Acceptance Radius)
      const radField = document.createElement("label");
      radField.className = "mi-field";
      radField.title = "Acceptance hit radius (meters). Set smaller e.g. 0.3m for precision dropping.";
      radField.innerHTML = `<span>Radius:</span>`;
      const radInp = document.createElement("input");
      radInp.className = "mi-radius";
      radInp.type = "number";
      radInp.min = 0.1;
      radInp.max = 50;
      radInp.step = 0.1;
      radInp.value = item.radius ?? 2.0;
      radInp.onchange = () => {
        item.radius = Math.max(0.1, Number(radInp.value) || 2.0);
      };
      radField.appendChild(radInp);
      radField.appendChild(document.createTextNode("m"));
      fieldsRow.appendChild(radField);

      contentWrap.appendChild(fieldsRow);
    } else if (item.type === "servo") {
      const header = document.createElement("div");
      header.className = "mi-header";
      const actionLabel = item.actionText || (item.pwm >= 1500 ? "OPEN" : "CLOSE");
      const isBuka = actionLabel === "OPEN" || actionLabel === "BUKA";
      const hasAutoClose = isBuka && !!item.autoClose;
      const autoBadgeText = hasAutoClose ? ` &rarr; AUTO CLOSE (${item.closeDelay || 2}s)` : "";

      const badgeStyle = isBuka
        ? 'background: rgba(63, 185, 80, 0.2); color: var(--teal); border: 1px solid rgba(63, 185, 80, 0.3);'
        : 'background: rgba(210, 153, 34, 0.2); color: var(--amber); border: 1px solid rgba(210, 153, 34, 0.3);';
      header.innerHTML = `<span class="mi-badge" style="${badgeStyle}">SERVO [${actionLabel}${autoBadgeText}]</span> <span class="mi-servo-info">${item.label || 'Servo CH' + item.servo} &rarr; <b>${actionLabel}</b> (PWM ${item.pwm}µs)</span>`;
      contentWrap.appendChild(header);

      if (isBuka) {
        const fieldsRow = document.createElement("div");
        fieldsRow.className = "mi-fields-row margin-top-xs";

        const chkLabel = document.createElement("label");
        chkLabel.className = "mi-field-chk";
        chkLabel.title = "Automatically close servo after delay without stopping the drone in flight";
        chkLabel.innerHTML = `<input type="checkbox" ${item.autoClose ? "checked" : ""} /> <span>Auto-Close In-Flight</span>`;
        const chkInp = chkLabel.querySelector("input");

        const delayField = document.createElement("label");
        delayField.className = "mi-field";
        delayField.title = "Wait time (seconds) from open to auto-close while flying";
        delayField.style.display = item.autoClose ? "inline-flex" : "none";
        delayField.innerHTML = `<span>Delay:</span>`;
        const delayInp = document.createElement("input");
        delayInp.className = "mi-delay";
        delayInp.type = "number";
        delayInp.min = 0.5;
        delayInp.max = 60;
        delayInp.step = 0.5;
        delayInp.value = item.closeDelay ?? 2;

        chkInp.onchange = (e) => {
          e.stopPropagation();
          item.autoClose = chkInp.checked;
          if (item.closeDelay === undefined) item.closeDelay = 2;
          renderMission();
        };

        delayInp.onchange = (e) => {
          e.stopPropagation();
          item.closeDelay = Math.max(0.1, Number(delayInp.value) || 2);
          renderMission();
        };

        delayField.appendChild(delayInp);
        delayField.appendChild(document.createTextNode("s"));

        fieldsRow.appendChild(chkLabel);
        fieldsRow.appendChild(delayField);
        contentWrap.appendChild(fieldsRow);
      }
    } else if (item.type === "takeoff") {
      const header = document.createElement("div");
      header.className = "mi-header";
      header.innerHTML = `<span class="mi-badge mi-badge-delay">TAKEOFF</span> <span class="mi-delay-info">Auto Takeoff Climb &rarr; ${item.alt}m</span>`;
      contentWrap.appendChild(header);
    } else if (item.type === "delay") {
      const header = document.createElement("div");
      header.className = "mi-header";
      header.innerHTML = `<span class="mi-badge mi-badge-delay">HOVER DELAY</span> <span class="mi-delay-info">Hover / Hold Position For ${item.delay} Seconds</span>`;
      contentWrap.appendChild(header);

      const fieldsRow = document.createElement("div");
      fieldsRow.className = "mi-fields-row";

      const delayField = document.createElement("label");
      delayField.className = "mi-field";
      delayField.title = "Hover delay duration (seconds)";
      delayField.innerHTML = `<span>Duration:</span>`;
      const delayInp = document.createElement("input");
      delayInp.className = "mi-delay";
      delayInp.type = "number";
      delayInp.min = 1;
      delayInp.max = 3600;
      delayInp.step = 1;
      delayInp.value = item.delay ?? 5;
      delayInp.onchange = () => {
        item.delay = Math.max(1, Number(delayInp.value) || 5);
        renderMission();
      };
      delayField.appendChild(delayInp);
      delayField.appendChild(document.createTextNode("s"));
      fieldsRow.appendChild(delayField);

      contentWrap.appendChild(fieldsRow);
    } else if (item.type === "land") {
      const header = document.createElement("div");
      header.className = "mi-header";
      header.innerHTML = `<span class="mi-badge" style="background: rgba(245, 158, 11, 0.2); color: var(--amber); border: 1px solid rgba(245, 158, 11, 0.3);">LANDING</span> <span class="mi-delay-info">Auto Landing Command</span>`;
      contentWrap.appendChild(header);
    } else if (item.type === "arm") {
      const header = document.createElement("div");
      header.className = "mi-header";
      header.innerHTML = `<span class="mi-badge" style="background: rgba(248, 81, 73, 0.2); color: var(--red); border: 1px solid rgba(248, 81, 73, 0.3);">ARM MOTORS</span> <span class="mi-delay-info">Arm Drone Motors Command</span>`;
      contentWrap.appendChild(header);
    } else if (item.type === "disarm") {
      const header = document.createElement("div");
      header.className = "mi-header";
      header.innerHTML = `<span class="mi-badge" style="background: rgba(248, 81, 73, 0.2); color: var(--red); border: 1px solid rgba(248, 81, 73, 0.3);">DISARM MOTORS</span> <span class="mi-delay-info">Disarm Drone Motors Command</span>`;
      contentWrap.appendChild(header);
    }

    li.appendChild(contentWrap);

    // Actions button group
    const actions = document.createElement("div");
    actions.className = "mi-actions";

    if (item.type === "waypoint") {
      const posBtn = document.createElement("button");
      posBtn.className = "mi-btn mi-pos";
      posBtn.textContent = "POS";
      posBtn.title = "Update koordinat waypoint ini ke lokasi GPS drone saat ini";
      posBtn.onclick = (e) => {
        e.stopPropagation();
        selectedWaypointId = item.id;
        updateWaypointToCurrentPos(item.id);
      };
      actions.appendChild(posBtn);
    }

    const up = document.createElement("button");
    up.className = "mi-btn";
    up.textContent = "UP";
    up.title = "Pindahkan urutan ke atas";
    up.onclick = (e) => { e.stopPropagation(); moveItem(item.id, -1); };
    actions.appendChild(up);

    const down = document.createElement("button");
    down.className = "mi-btn";
    down.textContent = "DN";
    down.title = "Pindahkan urutan ke bawah";
    down.onclick = (e) => { e.stopPropagation(); moveItem(item.id, 1); };
    actions.appendChild(down);

    const del = document.createElement("button");
    del.className = "mi-btn mi-del";
    del.innerHTML = "&times;";
    del.title = "Delete this command from mission";
    del.onclick = (e) => { e.stopPropagation(); deleteItem(item.id); };
    actions.appendChild(del);

    li.appendChild(actions);
    list.appendChild(li);
  });

  const btnAddCurrent = $("btn-add-current");
  if (btnAddCurrent) {
    if (selWpSeq !== null) {
      btnAddCurrent.textContent = `UPDATE WP ${String(selWpSeq).padStart(2, "0")} POS`;
      btnAddCurrent.title = `Update Waypoint ${selWpSeq} coordinates to current drone GPS position`;
      btnAddCurrent.classList.add("tb-active");
    } else {
      btnAddCurrent.textContent = "CURRENT DRONE POS";
      btnAddCurrent.title = "Add current drone GPS position as a new waypoint";
      btnAddCurrent.classList.remove("tb-active");
    }
  }

  if ($("mission-count")) {
    $("mission-count").textContent = `${mission.length} item${mission.length === 1 ? '' : 's'} registered`;
  }

  saveMissionToStorage();
}

/* ============================== parameter configuration ============================== */

let paramDefs = [];        // [{group, name, title, desc, unit, min, max, step, default}]
let paramValues = {};      // name -> value as read from the vehicle
let paramDirty = {};       // name -> true when edited but not yet sent

function switchTab(tab) {
  const isConfig = tab === "config";
  $("tab-mission").classList.toggle("tab-active", !isConfig);
  $("tab-config").classList.toggle("tab-active", isConfig);
  $("panel-mission").hidden = isConfig;
  $("panel-config").hidden = !isConfig;
  if (isConfig) {
    map.invalidateSize();
    refreshParams(true);
  }
}

async function loadParamDefs() {
  try {
    const res = await fetch("/api/params/defs");
    const data = await res.json();
    if (res.ok && Array.isArray(data.params)) {
      paramDefs = data.params;
      renderParamGroups();
    }
  } catch (err) {
    toast(`Gagal memuat katalog parameter: ${err.message}`, true);
  }
}

function renderParamGroups(filterQuery = "") {
  const wrap = $("param-groups");
  if (!wrap) return;
  wrap.innerHTML = "";

  const query = (filterQuery || "").trim().toLowerCase();

  const groups = new Map();
  for (const def of paramDefs) {
    if (query) {
      const matchName = def.name.toLowerCase().includes(query);
      const matchTitle = (def.title || "").toLowerCase().includes(query);
      const matchDesc = (def.desc || "").toLowerCase().includes(query);
      const matchGroup = (def.group || "").toLowerCase().includes(query);
      if (!matchName && !matchTitle && !matchDesc && !matchGroup) continue;
    }
    if (!groups.has(def.group)) groups.set(def.group, []);
    groups.get(def.group).push(def);
  }

  if (groups.size === 0) {
    const empty = document.createElement("div");
    empty.className = "param-empty";
    empty.innerHTML = `
      <div class="empty-title">Parameter Tidak Ditemukan</div>
      <div class="empty-desc">Tidak ada parameter yang cocok dengan kata kunci "<b>${filterQuery}</b>".</div>
    `;
    wrap.appendChild(empty);
    return;
  }

  for (const [group, defs] of groups) {
    const sec = document.createElement("section");
    sec.className = "card param-group";

    const h = document.createElement("h2");
    h.innerHTML = `<span>${group}</span> <span class="h2-count">${defs.length} parameter</span>`;
    sec.appendChild(h);

    const table = document.createElement("table");
    table.className = "param-table";

    const thead = document.createElement("thead");
    thead.innerHTML = `
      <tr>
        <th style="width: 38%">Parameter & Penjelasan</th>
        <th style="width: 24%">Nilai Input</th>
        <th style="width: 20%">Rentang</th>
        <th style="width: 18%; text-align: right">Aksi & Status</th>
      </tr>
    `;
    table.appendChild(thead);

    const tbody = document.createElement("tbody");

    for (const def of defs) {
      const tr = document.createElement("tr");
      tr.className = "param-tr";
      tr.id = `param-card-${def.name}`;
      if (paramDirty[def.name]) tr.classList.add("param-dirty");

      // Column 1: Title, MAVLink key & desc
      const tdInfo = document.createElement("td");
      tdInfo.className = "ptd-info";
      tdInfo.innerHTML = `
        <div class="ptd-head">
          <span class="ptd-title">${def.title || def.name}</span>
          <span class="ptd-key">${def.name}</span>
        </div>
        <div class="ptd-desc">${def.desc}</div>
      `;
      tr.appendChild(tdInfo);

      // Column 2: Input box with unit
      const tdInput = document.createElement("td");
      tdInput.className = "ptd-input";

      const inputWrap = document.createElement("div");
      inputWrap.className = "param-input-wrap";

      const input = document.createElement("input");
      input.className = "param-inp";
      input.type = "number";
      input.min = def.min;
      input.max = def.max;
      input.step = def.step;
      input.value = paramValues[def.name] !== undefined ? String(paramValues[def.name]) : "";
      input.placeholder = "--";
      input.disabled = paramValues[def.name] === undefined;
      input.id = `param-inp-${def.name}`;

      const statusTag = document.createElement("span");
      statusTag.className = "param-status-badge";
      statusTag.id = `param-status-${def.name}`;
      if (paramDirty[def.name]) {
        statusTag.textContent = "Diubah";
        statusTag.className = "param-status-badge badge-dirty";
      } else if (paramValues[def.name] !== undefined) {
        statusTag.textContent = "Terbaca";
        statusTag.className = "param-status-badge badge-ok";
      } else {
        statusTag.textContent = "Belum Ada";
        statusTag.className = "param-status-badge";
      }

      input.onchange = () => {
        const v = Number(input.value);
        if (Number.isNaN(v)) return;
        const original = paramValues[def.name];
        const changed = original === undefined || Math.abs(v - original) > 1e-9;
        paramDirty[def.name] = changed;
        tr.classList.toggle("param-dirty", changed);
        if (changed) {
          tr.classList.remove("param-sent");
          statusTag.textContent = "Diubah";
          statusTag.className = "param-status-badge badge-dirty";
        }
      };
      inputWrap.appendChild(input);

      if (def.unit) {
        const unitTag = document.createElement("span");
        unitTag.className = "param-unit-tag";
        unitTag.textContent = def.unit;
        inputWrap.appendChild(unitTag);
      }
      tdInput.appendChild(inputWrap);
      tr.appendChild(tdInput);

      // Column 3: Range & Recommended value
      const tdRange = document.createElement("td");
      tdRange.className = "ptd-range";
      let rangeText = `[${def.min} .. ${def.max}] ${def.unit || ''}`;
      if (def.default !== undefined) rangeText += ` (Rec: ${def.default})`;
      tdRange.textContent = rangeText;
      tr.appendChild(tdRange);

      // Column 4: Send button & status tag
      const tdAction = document.createElement("td");
      tdAction.className = "ptd-action";

      const btn = document.createElement("button");
      btn.className = "btn btn-primary param-set";
      btn.textContent = "KIRIM";
      btn.onclick = async (e) => {
        e.stopPropagation();
        await sendParam(def, input, btn, statusTag);
      };
      tdAction.appendChild(btn);
      tdAction.appendChild(statusTag);
      tr.appendChild(tdAction);

      tbody.appendChild(tr);
    }

    table.appendChild(tbody);
    sec.appendChild(table);
    wrap.appendChild(sec);
  }
}

async function sendAllDirtyParams() {
  if (!telemetry.connected) {
    toast("Drone tidak terhubung MAVLink", true);
    return;
  }
  const dirtyNames = Object.keys(paramDirty).filter((n) => paramDirty[n]);
  if (dirtyNames.length === 0) {
    toast("Tidak ada draf parameter yang diubah");
    return;
  }
  toast(`Mengirim ${dirtyNames.length} draf parameter ke RAM...`);
  let successCount = 0;
  for (const name of dirtyNames) {
    const def = paramDefs.find((d) => d.name === name);
    const input = $(`param-inp-${name}`);
    const statusTag = $(`param-status-${name}`);
    if (def && input) {
      const tr = input.closest(".param-tr");
      const btn = tr ? tr.querySelector(".param-set") : null;
      const res = await sendParam(def, input, btn, statusTag);
      if (res) successCount++;
    }
  }
  toast(`${successCount} dari ${dirtyNames.length} parameter berhasil dikirim ke RAM`);
}

function resetParamDrafts() {
  const dirtyNames = Object.keys(paramDirty).filter((n) => paramDirty[n]);
  if (dirtyNames.length === 0) {
    toast("No parameter drafts to reset");
    return;
  }
  for (const name of dirtyNames) {
    delete paramDirty[name];
    const input = $(`param-inp-${name}`);
    const tr = $(`param-card-${name}`);
    const statusTag = $(`param-status-${name}`);
    if (input) {
      input.value = paramValues[name] !== undefined ? String(paramValues[name]) : "";
    }
    if (tr) tr.classList.remove("param-dirty", "param-sent");
    if (statusTag) {
      statusTag.textContent = "Terbaca";
      statusTag.className = "param-status-badge badge-ok";
    }
  }
  toast(`Draf ${dirtyNames.length} parameter telah dikembalikan ke nilai awal`);
}

function exportParamFile() {
  const keys = Object.keys(paramValues);
  if (keys.length === 0) {
    toast("No parameters loaded to export", true);
    return;
  }
  let lines = [
    "# ArduPilot Parameter File Export",
    "# Generated by KRTI 2026 Ground Control Station",
    `# Date: ${new Date().toISOString()}`,
    "",
  ];
  for (const name of keys) {
    const val = paramValues[name];
    if (val !== undefined && val !== null) {
      lines.push(`${name}\t${val}`);
    }
  }
  const content = lines.join("\n");
  const blob = new Blob([content], { type: "text/plain;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `krti_ardupilot_params_${new Date().toISOString().slice(0, 10)}.param`;
  a.click();
  URL.revokeObjectURL(url);
  toast(`Berhasil mengekspor ${keys.length} parameter ke file .param`);
}

function importParamFile(file) {
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (e) => {
    const text = e.target.result || "";
    const lines = text.split(/\r?\n/);
    let importedCount = 0;

    for (const rawLine of lines) {
      const line = rawLine.trim();
      if (!line || line.startsWith("#") || line.startsWith("//")) continue;

      const parts = line.split(/[\s,=\t]+/);
      if (parts.length >= 2) {
        const name = parts[0].toUpperCase().trim();
        const val = Number(parts[1]);
        if (!Number.isNaN(val)) {
          const def = paramDefs.find((d) => d.name === name);
          if (def) {
            paramDirty[name] = true;
            paramValues[name] = val;
            const input = $(`param-inp-${name}`);
            const tr = $(`param-card-${name}`);
            const statusTag = $(`param-status-${name}`);
            if (input) {
              input.value = String(val);
              input.disabled = false;
            }
            if (tr) tr.classList.add("param-dirty");
            if (statusTag) {
              statusTag.textContent = "Draf Impor";
              statusTag.className = "param-status-badge badge-dirty";
            }
            importedCount++;
          }
        }
      }
    }

    if (importedCount > 0) {
      toast(`Successfully imported ${importedCount} parameters! Click 'SEND ALL' to apply to drone RAM.`);
    } else {
      toast("No matching parameters found in file", true);
    }
  };
  reader.readAsText(file);
}

async function refreshParams(quiet = false) {
  if (paramDefs.length === 0) return;
  if (!telemetry.connected) {
    if ($("param-status")) $("param-status").textContent = "DISCONNECTED";
    toast("Drone disconnected - cannot read parameters", true);
    return;
  }
  if (!quiet) toast("Fetching live parameters from drone via MAVLink...");
  const names = paramDefs.map((d) => d.name);
  const res = await api("/api/params/read", { names });
  if (!res || !res.values) {
    if ($("param-status")) $("param-status").textContent = "Read Failed";
    return;
  }
  paramValues = {};
  paramDirty = {};
  for (const def of paramDefs) {
    const v = res.values[def.name];
    if (v !== undefined && v !== null) paramValues[def.name] = v;
    const input = $(`param-inp-${def.name}`);
    const card = $(`param-card-${def.name}`);
    const statusTag = $(`param-status-${def.name}`);
    
    if (input) {
      input.value = v === null || v === undefined ? "" : String(v);
      input.disabled = !(v !== null && v !== undefined);
    }
    if (card) card.classList.remove("param-dirty", "param-sent");
    if (statusTag) {
      if (v !== null && v !== undefined) {
        statusTag.textContent = "Read from Drone";
        statusTag.className = "param-status-badge badge-ok";
      } else {
        statusTag.textContent = "No Response";
        statusTag.className = "param-status-badge badge-err";
      }
    }
  }
  if ($("param-status")) $("param-status").textContent = "Read from ArduPilot";
  if (!quiet) toast("Parameters successfully updated from drone");
}

async function sendParam(def, input, btn, statusTag) {
  if (!telemetry.connected) {
    toast("Drone not connected via MAVLink", true);
    return;
  }
  const v = Number(input.value);
  if (Number.isNaN(v)) {
    toast(`Invalid value for ${def.name}`, true);
    return;
  }
  if (v < def.min || v > def.max) {
    toast(`${def.name}: Value out of range [${def.min} to ${def.max}]`, true);
    return;
  }
  btn.disabled = true;
  const res = await api("/api/params/set", { name: def.name, value: v });
  btn.disabled = false;
  
  if (res) {
    paramValues[def.name] = res.value ?? v;
    delete paramDirty[def.name];
    input.value = String(res.value ?? v);
    const card = input.closest(".param-card");
    if (card) {
      card.classList.remove("param-dirty");
      card.classList.add("param-sent");
    }
    if (statusTag) {
      statusTag.textContent = "Saved to RAM";
      statusTag.className = "param-status-badge badge-sent";
    }
    toast(`${def.name} (${def.title || ''}) = ${input.value} ${def.unit || ''} successfully sent to RAM`);
  } else {
    const card = input.closest(".param-card");
    if (card) card.classList.add("param-dirty");
    input.value = paramValues[def.name] !== undefined ? String(paramValues[def.name]) : "";
    if (statusTag) {
      statusTag.textContent = "Send Failed";
      statusTag.className = "param-status-badge badge-err";
    }
  }
}

async function saveParamsToEeprom() {
  if (!telemetry.connected) {
    toast("Drone not connected via MAVLink", true);
    return;
  }
  if (!confirm("Save all RAM parameter changes to non-volatile EEPROM permanently?")) return;
  const res = await api("/api/params/save");
  if (res) toast("All parameters successfully saved to EEPROM permanently");
}

/* ============================== Timer System (KRTI 2026) ============================== */

// 1. Competition Countdown Timer (10 Minutes = 600 Seconds)
let compTimerSec = 600;
let compTimerInterval = null;
let compTimerRunning = false;
let compTimerStarted = false;

// 2. Flight Duration Timer & Best Time
let flightTimerSec = 0;
let flightTimerInterval = null;
let flightTimerRunning = false;
let bestFlightSec = localStorage.getItem("krti_best_flight_sec")
  ? Number(localStorage.getItem("krti_best_flight_sec"))
  : null;
let lastTelemetryArmed = false;

function fmtMMSS(sec) {
  const m = Math.floor(sec / 60);
  const s = sec % 60;
  return `${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`;
}

// --- Competition 10m Countdown Timer ---

function updateCompTimerUI() {
  const valEl = $("comp-timer-val");
  const tagEl = $("comp-timer-tag");
  const boxEl = $("comp-timer-box");

  if (valEl) valEl.textContent = fmtMMSS(compTimerSec);

  if (boxEl) {
    if (compTimerSec < 60 && compTimerRunning) {
      boxEl.classList.add("timer-warning");
    } else {
      boxEl.classList.remove("timer-warning");
    }
  }

  if (tagEl) {
    if (compTimerSec === 0) {
      tagEl.textContent = "EXPIRED";
      tagEl.className = "timer-tag tag-ended";
    } else if (compTimerRunning) {
      tagEl.textContent = "RUNNING";
      tagEl.className = "timer-tag tag-running";
    } else if (compTimerStarted) {
      tagEl.textContent = "PAUSED";
      tagEl.className = "timer-tag tag-paused";
    } else {
      tagEl.textContent = "READY";
      tagEl.className = "timer-tag tag-ready";
    }
  }
}

function startCompTimer() {
  if (compTimerRunning || compTimerSec <= 0) return;
  compTimerRunning = true;
  compTimerStarted = true;
  updateCompTimerUI();

  if (compTimerInterval) clearInterval(compTimerInterval);
  compTimerInterval = setInterval(() => {
    if (compTimerSec > 0) {
      compTimerSec--;
      updateCompTimerUI();
      if (compTimerSec === 0) {
        pauseCompTimer();
        toast("⚠️ 10-MINUTE COMPETITION TIME EXPIRED!", true);
      }
    }
  }, 1000);
}

function pauseCompTimer() {
  if (compTimerInterval) {
    clearInterval(compTimerInterval);
    compTimerInterval = null;
  }
  compTimerRunning = false;
  updateCompTimerUI();
}

function toggleCompTimer() {
  if (compTimerRunning) {
    pauseCompTimer();
    toast("10m Timer paused");
  } else {
    startCompTimer();
    toast("10m Timer running");
  }
}

function resetCompTimer() {
  pauseCompTimer();
  compTimerSec = 600;
  compTimerStarted = false;
  updateCompTimerUI();
  toast("10m Timer reset to 10:00");
}

// --- Flight Duration & Best Time ---

function updateFlightTimerUI() {
  const valEl = $("flight-timer-val");
  const tagEl = $("flight-timer-tag");
  const bestEl = $("flight-best-val");

  if (valEl) valEl.textContent = fmtMMSS(flightTimerSec);

  if (tagEl) {
    if (flightTimerRunning) {
      tagEl.textContent = "IN-FLIGHT";
      tagEl.className = "timer-tag tag-running";
    } else {
      tagEl.textContent = "STANDBY";
      tagEl.className = "timer-tag tag-idle";
    }
  }

  if (bestEl) {
    bestEl.textContent = bestFlightSec !== null ? fmtMMSS(bestFlightSec) : "--:--";
  }
}

function startFlightTimer() {
  if (flightTimerRunning) return;
  flightTimerRunning = true;
  flightTimerSec = 0;
  updateFlightTimerUI();

  if (flightTimerInterval) clearInterval(flightTimerInterval);
  flightTimerInterval = setInterval(() => {
    flightTimerSec++;
    updateFlightTimerUI();
  }, 1000);
}

function stopFlightTimer() {
  if (!flightTimerRunning) return;
  flightTimerRunning = false;
  if (flightTimerInterval) {
    clearInterval(flightTimerInterval);
    flightTimerInterval = null;
  }
  updateFlightTimerUI();

  if (flightTimerSec > 3) {
    if (bestFlightSec === null || flightTimerSec < bestFlightSec) {
      bestFlightSec = flightTimerSec;
      localStorage.setItem("krti_best_flight_sec", String(bestFlightSec));
      updateFlightTimerUI();
      toast(`🏆 NEW BEST FLIGHT TIME RECORD: ${fmtMMSS(bestFlightSec)}!`);
    } else {
      toast(`Flight completed: ${fmtMMSS(flightTimerSec)} (Best: ${fmtMMSS(bestFlightSec)})`);
    }
  }
}

function resetBestTime() {
  bestFlightSec = null;
  localStorage.removeItem("krti_best_flight_sec");
  updateFlightTimerUI();
  toast("Best flight time cleared");
}

// Triggered when user starts a mission flight attempt
function onFlightStartTrigger() {
  // 1. Competition 10m countdown: Auto-start on first mission start, does NOT reset on retries
  if (!compTimerStarted && compTimerSec > 0) {
    startCompTimer();
  }
  // 2. Flight duration timer: Reset to 0 and start measuring this flight attempt
  startFlightTimer();
}

function checkTelemetryTimerHooks() {
  const t = telemetry;
  if (!t) return;

  if (t.armed && !lastTelemetryArmed) {
    if (t.mode === "AUTO" || t.mode === "GUIDED") {
      onFlightStartTrigger();
    }
  } else if (!t.armed && lastTelemetryArmed) {
    stopFlightTimer();
  }

  if (flightTimerRunning && (t.mode === "LAND" || t.mode === "RTL") && t.alt_rel !== null && t.alt_rel < 0.3) {
    stopFlightTimer();
  }

  lastTelemetryArmed = !!t.armed;
}

/* ============================== telemetry HUD ============================== */

const GPS_FIX_TEXT = {
  0: "NO GPS", 1: "NO FIX", 2: "2D FIX", 3: "3D FIX", 4: "DGPS", 5: "RTK FLOAT", 6: "RTK FIX",
};

function setPill(id, text, cls) {
  const el = $(id);
  if (!el) return;
  el.innerHTML = text;
  el.className = "pill" + (cls ? " " + cls : "");
}

function updateHUD() {
  const t = telemetry;
  checkTelemetryTimerHooks();

  setPill("pill-link", t.connected ? `<span class="pill-dot"></span> LINK OK` : `<span class="pill-dot"></span> NO LINK`,
    t.connected ? "pill-ok" : "pill-danger");
  setPill("pill-mode", `MODE: ${t.mode || "---"}`, t.connected ? "pill-ok" : "");
  setPill("pill-arm", t.armed ? "ARMED" : "DISARMED", t.armed ? "pill-armed" : "");

  const fixText = GPS_FIX_TEXT[t.gps_fix] || "GPS: NO FIX";
  setPill("pill-gps", `${fixText} (${t.satellites ?? "--"} Sats)`, t.gps_fix >= 3 ? "pill-ok" : "");

  const battV = fmt(t.battery_voltage, 1);
  const battP = t.battery_remaining;
  setPill("pill-batt", battP !== null && battP !== undefined ? `${battV}V (${battP}%)` : `${battV} V`,
    battP !== null && battP !== undefined && battP < 25 ? "pill-danger" : "");
  if ($("pill-status")) {
    const s = t.last_status;
    $("pill-status").textContent = s ? `STATUS: ${s}` : "STATUS: ---";
    $("pill-status").title = s || "";
    $("pill-status").className = "pill" + (s ? " pill-warn" : "");
  }

  if ($("t-alt")) $("t-alt").textContent = fmt(t.alt_rel, 1);
  if ($("t-alt-msl")) $("t-alt-msl").textContent = fmt(t.alt_msl, 1);
  if ($("t-gs")) $("t-gs").textContent = fmt(t.groundspeed, 1);
  if ($("t-hdg")) $("t-hdg").textContent = fmt(t.heading, 0);
  if ($("t-vs")) $("t-vs").textContent = fmt(t.climb, 1);
  if ($("t-sats")) $("t-sats").textContent = t.satellites ?? "--";
  if ($("t-thr")) $("t-thr").textContent = t.throttle ?? "--";
  if ($("t-roll")) $("t-roll").textContent = fmt(t.roll, 0);
  if ($("t-pitch")) $("t-pitch").textContent = fmt(t.pitch, 0);
  if ($("t-pos")) {
    $("t-pos").textContent =
      t.lat !== null && t.lat !== undefined
        ? `GPS: ${t.lat.toFixed(7)}, ${t.lon.toFixed(7)} | Altitude MSL: ${fmt(t.alt_msl, 1)} m`
        : "GPS Location: No Fix";
  }

  // drone marker + trail + heading line
  if (t.lat !== null && t.lat !== undefined && t.lat !== 0) {
    const pos = [t.lat, t.lon];
    const heading = t.heading || 0;

    if (!droneMarker) {
      droneMarker = L.marker(pos, { icon: droneIcon(heading) }).addTo(map);
    } else {
      droneMarker.setLatLng(pos);
      droneMarker.setIcon(droneIcon(heading));
    }

    // Forward Heading Line
    const forwardPos = getHeadingEndpoint(t.lat, t.lon, heading, CONFIG.headingLineLength);
    if (forwardPos) {
      headingLine.setLatLngs([pos, forwardPos]);
    }

    // Drone Trail
    const trail = droneTrail.getLatLngs();
    trail.push(pos);
    if (trail.length > CONFIG.trailLength) trail.shift();
    droneTrail.setLatLngs(trail);

    if (!homeSet) {
      map.setView(pos, CONFIG.map.zoom);
      homeSet = true;
    }

    // Auto Follow
    if (followMode) {
      map.panTo(pos);
    }
  } else {
    headingLine.setLatLngs([]);
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

/* ============================== resizers (window splitters) ============================== */

function initResizers() {
  const resizerV = $("resizer-v");
  const resizerH = $("resizer-h");
  const sidebar = $("sidebar");
  const camerasPanel = $("cameras-panel");
  const mapCol = $("map-col");

  // Restore saved layout preferences
  const savedSidebarWidth = localStorage.getItem("gcs_sidebar_width");
  if (savedSidebarWidth && sidebar) {
    sidebar.style.flexBasis = `${savedSidebarWidth}px`;
  }
  const savedCamerasHeight = localStorage.getItem("gcs_cameras_height");
  if (savedCamerasHeight && camerasPanel) {
    camerasPanel.style.flexBasis = `${savedCamerasHeight}px`;
  }
  map.invalidateSize();

  let isDraggingV = false;
  let isDraggingH = false;

  if (resizerV && sidebar) {
    resizerV.addEventListener("mousedown", (e) => {
      isDraggingV = true;
      document.body.classList.add("is-dragging-v");
      resizerV.classList.add("dragging");
      e.preventDefault();
    });
  }

  if (resizerH && camerasPanel && mapCol) {
    resizerH.addEventListener("mousedown", (e) => {
      isDraggingH = true;
      document.body.classList.add("is-dragging-h");
      resizerH.classList.add("dragging");
      e.preventDefault();
    });
  }

  document.addEventListener("mousemove", (e) => {
    if (isDraggingV && sidebar) {
      const layoutWidth = document.body.clientWidth;
      let newWidth = layoutWidth - e.clientX;
      if (newWidth < 260) newWidth = 260;
      if (newWidth > 600) newWidth = 600;
      sidebar.style.flexBasis = `${newWidth}px`;
      localStorage.setItem("gcs_sidebar_width", newWidth);
      requestAnimationFrame(() => map.invalidateSize());
    }

    if (isDraggingH && camerasPanel && mapCol) {
      const mapColHeight = mapCol.clientHeight;
      const mapColTop = mapCol.getBoundingClientRect().top;
      let newHeight = mapColTop + mapColHeight - e.clientY;
      if (newHeight < 80) newHeight = 80;
      if (newHeight > mapColHeight - 120) newHeight = mapColHeight - 120;
      camerasPanel.style.flexBasis = `${newHeight}px`;
      localStorage.setItem("gcs_cameras_height", newHeight);
      requestAnimationFrame(() => map.invalidateSize());
    }
  });

  document.addEventListener("mouseup", () => {
    if (isDraggingV || isDraggingH) {
      isDraggingV = false;
      isDraggingH = false;
      document.body.classList.remove("is-dragging-v", "is-dragging-h");
      if (resizerV) resizerV.classList.remove("dragging");
      if (resizerH) resizerH.classList.remove("dragging");
      map.invalidateSize();
    }
  });

  window.addEventListener("resize", () => {
    map.invalidateSize();
  });
}

/* ============================== controls ============================== */

function onServoStateChange() {
  const state = $("sel-item-servo-state")?.value || "open";
  const boxAuto = $("box-servo-autoclose");
  const boxDelay = $("box-servo-closedelay");
  const isBuka = state === "open";
  if (boxAuto) boxAuto.hidden = !isBuka;
  if (boxDelay) {
    const chk = $("chk-item-servo-autoclose")?.checked;
    boxDelay.hidden = !isBuka || !chk;
  }
}
window.onServoStateChange = onServoStateChange;

function onServoAutoCloseToggle() {
  const chk = $("chk-item-servo-autoclose")?.checked;
  const boxDelay = $("box-servo-closedelay");
  if (boxDelay) boxDelay.hidden = !chk;
}
window.onServoAutoCloseToggle = onServoAutoCloseToggle;

function onMissionItemTypeChange() {
  const type = $("sel-item-type")?.value || "waypoint";
  const boxWp = $("box-item-waypoint");
  const boxTk = $("box-item-takeoff");
  const boxSv = $("box-item-servo");
  const boxDl = $("box-item-delay");

  if (boxWp) { boxWp.hidden = (type !== "waypoint"); boxWp.style.display = (type === "waypoint" ? "grid" : "none"); }
  if (boxTk) { boxTk.hidden = (type !== "takeoff"); boxTk.style.display = (type === "takeoff" ? "flex" : "none"); }
  if (boxSv) { boxSv.hidden = (type !== "servo"); boxSv.style.display = (type === "servo" ? "grid" : "none"); }
  if (boxDl) { boxDl.hidden = (type !== "delay"); boxDl.style.display = (type === "delay" ? "flex" : "none"); }
  onServoStateChange();
}
window.onMissionItemTypeChange = onMissionItemTypeChange;

function addSelectedMissionItem() {
  const type = $("sel-item-type")?.value || "waypoint";
  if (type === "waypoint") {
    const alt = Number($("inp-item-alt")?.value) || 1;
    const radius = Number($("inp-item-wp-radius")?.value) || 2.0;
    const delay = Number($("inp-item-wp-delay")?.value) || 0;
    const lat = telemetry.lat && telemetry.lat !== 0 ? telemetry.lat : CONFIG.map.center[0];
    const lon = telemetry.lon && telemetry.lon !== 0 ? telemetry.lon : CONFIG.map.center[1];
    addWaypoint(lat, lon, alt, delay, radius);
    toast(`New Waypoint (${alt}m, rad ${radius}m, delay ${delay}s) added to mission`);
  } else if (type === "takeoff") {
    const alt = Number($("inp-item-takeoff-alt")?.value) || 1;
    mission.push({
      id: Date.now(),
      type: "takeoff",
      alt: alt
    });
    renderMission();
    toast(`Auto Takeoff command (${alt}m) added to mission`);
  } else if (type === "servo") {
    const servoVal = Number($("sel-item-servo-ch")?.value) || 10;
    const state = $("sel-item-servo-state")?.value || "open";
    const sConf = CONFIG.servos.find((s) => s.servo === servoVal) || { name: `Servo CH${servoVal}`, openPwm: 1900, closePwm: 1100 };
    const isBuka = state === "open";
    const pwm = isBuka ? (sConf.openPwm || 1900) : (sConf.closePwm || 1100);
    const actionText = isBuka ? "OPEN" : "CLOSE";
    const autoClose = isBuka && !!($("chk-item-servo-autoclose")?.checked);
    const closeDelay = autoClose ? Math.max(0.1, Number($("inp-item-servo-closedelay")?.value) || 2) : 0;
    const closePwm = sConf.closePwm || 1100;

    mission.push({
      id: Date.now(),
      type: "servo",
      servo: servoVal,
      pwm: pwm,
      actionText: actionText,
      autoClose: autoClose,
      closeDelay: closeDelay,
      closePwm: closePwm,
      label: `${sConf.name}`
    });
    renderMission();
    const autoInfo = autoClose ? ` [Auto-Close: ${closeDelay}s]` : "";
    toast(`Servo command ${sConf.name} (${actionText}${autoInfo}) added to mission`);
  } else if (type === "delay") {
    const sec = Number($("inp-item-delay-sec")?.value) || 5;
    addDelayItem(sec);
  } else if (type === "land") {
    mission.push({
      id: Date.now(),
      type: "land"
    });
    renderMission();
    toast("Auto Landing command added to mission");
  } else if (type === "arm") {
    mission.push({
      id: Date.now(),
      type: "arm"
    });
    renderMission();
    toast("Arm Motors command added to mission");
  } else if (type === "disarm") {
    mission.push({
      id: Date.now(),
      type: "disarm"
    });
    renderMission();
    toast("Disarm Motors command added to mission");
  }
}
window.addSelectedMissionItem = addSelectedMissionItem;

function wireControls() {
  const selItemServoCh = $("sel-item-servo-ch");
  if (selItemServoCh) {
    selItemServoCh.innerHTML = "";
    CONFIG.servos.forEach((s) => {
      const opt = document.createElement("option");
      opt.value = s.servo;
      opt.textContent = s.name;
      selItemServoCh.appendChild(opt);
    });
  }
  onMissionItemTypeChange();
  // sidebar tabs
  if ($("tab-mission")) $("tab-mission").onclick = () => switchTab("mission");
  if ($("tab-config")) $("tab-config").onclick = () => switchTab("config");

  // parameter configuration
  if ($("btn-param-refresh")) $("btn-param-refresh").onclick = () => refreshParams(false);
  if ($("btn-param-send-all")) $("btn-param-send-all").onclick = () => sendAllDirtyParams();
  if ($("btn-param-reset")) $("btn-param-reset").onclick = () => resetParamDrafts();
  if ($("btn-param-save")) $("btn-param-save").onclick = () => saveParamsToEeprom();
  if ($("btn-param-export")) $("btn-param-export").onclick = () => exportParamFile();
  if ($("btn-param-import") && $("inp-param-file")) {
    $("btn-param-import").onclick = () => $("inp-param-file").click();
    $("inp-param-file").onchange = (e) => {
      if (e.target.files && e.target.files[0]) {
        importParamFile(e.target.files[0]);
        e.target.value = "";
      }
    };
  }

  const inpParamSearch = $("inp-param-search");
  const btnParamClear = $("btn-param-clear-search");
  if (inpParamSearch) {
    inpParamSearch.oninput = () => {
      const q = inpParamSearch.value;
      if (btnParamClear) btnParamClear.hidden = !q;
      renderParamGroups(q);
    };
  }
  if (btnParamClear) {
    btnParamClear.onclick = () => {
      if (inpParamSearch) inpParamSearch.value = "";
      btnParamClear.hidden = true;
      renderParamGroups("");
    };
  }

  // Map centering & follow buttons
  if ($("btn-center-drone")) {
    $("btn-center-drone").onclick = () => centerDrone(true);
  }
  if ($("btn-toggle-follow")) {
    $("btn-toggle-follow").onclick = () => toggleFollow();
  }

  // map toolbar
  if ($("btn-addmode")) {
    $("btn-addmode").onclick = () => {
      addMode = !addMode;
      $("btn-addmode").textContent = `MAP CLICK: ${addMode ? "ON" : "OFF"}`;
      $("btn-addmode").classList.toggle("tb-active", addMode);
    };
  }
  if ($("btn-add-current")) {
    $("btn-add-current").onclick = () => {
      if (telemetry.lat === null || telemetry.lat === undefined) {
        toast("GPS fix not available, cannot acquire drone location", true);
        return;
      }
      const selectedItem = mission.find((m) => m.id === selectedWaypointId);
      if (selectedItem && selectedItem.type === "waypoint") {
        updateWaypointToCurrentPos(selectedWaypointId);
      } else {
        const alt = Number($("inp-alt").value) || CONFIG.defaultAlt;
        addWaypoint(telemetry.lat, telemetry.lon, alt);
        toast("Current drone GPS location added to mission");
      }
    };
  }
  if ($("btn-clear-mission")) {
    $("btn-clear-mission").onclick = () => {
      if (mission.length === 0) return;
      if (!confirm("Clear all mission waypoints?")) return;
      mission = [];
      selectedWaypointId = null;
      renderMission();
      toast("Mission list cleared");
    };
  }

  // mission save & load
  if ($("btn-save-mission")) $("btn-save-mission").onclick = () => saveMissionFile();
  if ($("btn-load-mission") && $("inp-mission-file")) {
    $("btn-load-mission").onclick = () => $("inp-mission-file").click();
    $("inp-mission-file").onchange = (e) => {
      if (e.target.files && e.target.files[0]) {
        loadMissionFile(e.target.files[0]);
        e.target.value = "";
      }
    };
  }

  // mission
  if ($("btn-upload")) {
    $("btn-upload").onclick = async () => {
      if (mission.length === 0) {
        toast("Daftar misi masih kosong! Tambahkan waypoint terlebih dahulu", true);
        return;
      }
      const items = mission.map((m) => {
        if (m.type === "waypoint") {
          return { type: "waypoint", lat: m.lat, lon: m.lon, alt: m.alt, delay: m.delay ?? 0, radius: m.radius ?? 2.0 };
        }
        if (m.type === "servo") {
          return {
            type: "servo",
            servo: m.servo,
            pwm: m.pwm,
            auto_close: !!m.autoClose,
            close_delay: Number(m.closeDelay || 2.0),
            close_pwm: Number(m.closePwm || 1100),
          };
        }
        if (m.type === "delay") {
          return { type: "delay", delay: m.delay };
        }
        return { type: m.type, alt: m.alt };
      });
      const res = await api("/api/mission/upload", { items });
      if (res) {
        const extra = res.auto_takeoff ? " — Auto TAKEOFF item inserted at start" : "";
        toast(`Mission uploaded successfully to drone (${res.uploaded} items including Home)${extra}`);
      }
    };
  }
  // Timers controls
  if ($("btn-comp-timer-toggle")) $("btn-comp-timer-toggle").onclick = toggleCompTimer;
  if ($("btn-comp-timer-reset")) $("btn-comp-timer-reset").onclick = resetCompTimer;
  if ($("btn-reset-best")) $("btn-reset-best").onclick = resetBestTime;

  if ($("btn-start")) {
    $("btn-start").onclick = async () => {
      if (!confirm("Start automated mission flight execution (AUTO mode)?")) return;
      const res = await api("/api/mission/start");
      if (res) {
        toast("Automated mission started (AUTO mode)");
        onFlightStartTrigger();
      }
    };
  }
  if ($("btn-pause")) {
    $("btn-pause").onclick = async () => {
      const res = await api("/api/mission/pause");
      if (res) toast("Vehicle holding position (LOITER mode)");
    };
  }

  // flight controls
  if ($("btn-arm")) {
    $("btn-arm").onclick = async () => {
      if (!confirm("Arm motors (ARM MOTORS)? Ensure surrounding area is clear!")) return;
      const res = await api("/api/arm");
      if (res) toast("Motors Armed successfully");
    };
  }
  if ($("btn-disarm")) {
    $("btn-disarm").onclick = async () => {
      if (!confirm("Disarm motors immediately (DISARM)? Motors will stop instantly!")) return;
      const res = await api("/api/disarm");
      if (res) {
        toast("Motors Disarmed successfully");
        stopFlightTimer();
      }
    };
  }
  if ($("btn-takeoff")) {
    $("btn-takeoff").onclick = async () => {
      const alt = Number($("inp-takeoff-alt").value) || 1;
      if (!confirm(`Arm motors & execute Auto Takeoff climb to ${alt} meters (GUIDED mode)?`)) return;
      const res = await api("/api/takeoff", { altitude: alt });
      if (res) {
        toast(`Takeoff command to ${alt}m altitude sent`);
        onFlightStartTrigger();
      }
    };
  }
  if ($("btn-land")) {
    $("btn-land").onclick = async () => {
      if (!confirm("Command drone to land automatically at current position (LAND mode)?")) return;
      const res = await api("/api/land");
      if (res) {
        toast("Auto landing command sent (LAND mode)");
        stopFlightTimer();
      }
    };
  }
  if ($("btn-rtl")) {
    $("btn-rtl").onclick = async () => {
      if (!confirm("Command drone to return home to takeoff point (RTL mode)?")) return;
      const res = await api("/api/rtl");
      if (res) toast("Return Home command sent (RTL mode)");
    };
  }

  // servo mission item form
  const selServo = $("sel-servo");
  if (selServo) {
    CONFIG.servos.forEach((s) => {
      const opt = document.createElement("option");
      opt.value = s.servo;
      opt.textContent = s.name;
      selServo.appendChild(opt);
    });
  }
  const selAction = $("sel-servo-action");
  if (selAction) {
    [
      { label: "OPEN PAYLOAD", val: "OPEN" },
      { label: "CLOSE PAYLOAD", val: "CLOSE" },
      { label: "CUSTOM PWM (500-2500 µs)", val: "CUSTOM" },
    ].forEach((a) => {
      const opt = document.createElement("option");
      opt.value = a.val;
      opt.textContent = a.label;
      selAction.appendChild(opt);
    });
    selAction.onchange = () => {
      if ($("box-servo-pwm")) $("box-servo-pwm").hidden = selAction.value !== "CUSTOM";
    };
  }
  if ($("btn-add-servo")) {
    $("btn-add-servo").onclick = () => {
      const def = CONFIG.servos.find((s) => s.servo === Number(selServo.value));
      let pwm;
      if (selAction.value === "OPEN") pwm = def.openPwm;
      else if (selAction.value === "CLOSE") pwm = def.closePwm;
      else pwm = Number($("inp-servo-pwm").value) || 1500;
      addServoItem(def, pwm);
      toast(`Servo command ${def.name} (${pwm}µs) added to mission`);
    };
  }

  if ($("btn-add-delay")) {
    $("btn-add-delay").onclick = () => {
      const sec = Math.max(1, Number($("inp-delay-sec").value) || 5);
      addDelayItem(sec);
      toast(`Hover delay item (${sec}s) added to mission`);
    };
  }

  // quick servo open/close buttons
  const wrap = $("servo-controls");
  if (wrap) {
    CONFIG.servos.forEach((s) => {
      const row = document.createElement("div");
      row.className = "servo-row";

      const name = document.createElement("span");
      name.className = "srv-name";
      name.textContent = s.name;
      row.appendChild(name);

      const open = document.createElement("button");
      open.className = "btn btn-primary";
      open.textContent = "OPEN SERVO";
      open.onclick = async () => {
        const res = await api("/api/servo", { servo: s.servo, pwm: s.openPwm });
        if (res) toast(`Servo ${s.name} OPEN (${s.openPwm}µs)`);
      };
      row.appendChild(open);

      const close = document.createElement("button");
      close.className = "btn btn-warn";
      close.textContent = "CLOSE SERVO";
      close.onclick = async () => {
        const res = await api("/api/servo", { servo: s.servo, pwm: s.closePwm });
        if (res) toast(`Servo ${s.name} CLOSE (${s.closePwm}µs)`);
      };
      row.appendChild(close);

      wrap.appendChild(row);
    });
  }

  // camera stream sources
  const setupCamStream = (imgId, port) => {
    const img = $(imgId);
    if (!img) return;
    const url = `http://${CONFIG.piHost}:${port}/stream`;
    img.src = url;
    img.onerror = () => {
      setTimeout(() => {
        img.src = url + "?t=" + Date.now();
      }, 3000);
    };
  };

  setupCamStream("cam-down", CONFIG.camDownPort);
  // RealSense front camera disabled for now
  // setupCamStream("cam-front", CONFIG.camFrontPort);

  // exposure control (down camera)
  const setupExposure = () => {
    const ctl = $("exposure-ctl");
    const slider = $("exposure-slider");
    const val = $("exposure-val");
    if (!ctl || !slider || !val) return;

    const url = `http://${CONFIG.piHost}:${CONFIG.camDownPort}/exposure`;
    let timer = null;

    const setValue = (v) => { val.textContent = v; };
    const send = (value) => {
      fetch(`${url}?value=${value}`)
        .then((r) => r.json())
        .then((j) => {
          if (j.ok) {
            slider.value = j.value;
            setValue(j.value);
          }
        })
        .catch(() => {});
    };

    fetch(url)
      .then((r) => r.json())
      .then((j) => {
        if (!j.ok) throw new Error(j.error);
        slider.min = j.min;
        slider.max = j.max;
        slider.step = Math.max(1, Math.round((j.max - j.min) / 200));
        slider.value = j.value;
        slider.disabled = false;
        setValue(j.value);
      })
      .catch((e) => {
        ctl.title = "Exposure tidak tersedia: " + (e.message || "offline");
        ctl.style.opacity = 0.5;
      });

    slider.addEventListener("input", () => {
      setValue(slider.value);
      clearTimeout(timer);
      timer = setTimeout(() => send(slider.value), 120);
    });
    slider.addEventListener("change", () => {
      clearTimeout(timer);
      send(slider.value);
    });
  };

  setupExposure();
}

/* ============================== boot ============================== */

wireControls();
initResizers();
updateCompTimerUI();
updateFlightTimerUI();
if (loadMissionFromStorage()) {
  toast("Last mission restored from browser storage");
}
renderMission();
connectTelemetry();
loadParamDefs();
