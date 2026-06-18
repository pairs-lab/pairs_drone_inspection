/* ============================================================
   Automated Cycle Count — Operator Console  (live dashboard)
   WS 8765: live_sim frames / telemetry / inspection
   WS /ws:  backend inspection events (post-process)
   REST:    /api/bins, /api/history, /api/scene, /api/sap, /api/inspect
   ============================================================ */
'use strict';

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const COLUMNS = ['A', 'B', 'C'];
const LEVELS  = [1, 2, 3, 4, 5, 6];
const ALL_BINS = COLUMNS.flatMap(c => LEVELS.map(l => `${c}${l}`));

// Derive the Isaac live_sim host from the page URL so the dashboard works whether
// it's opened on the server (localhost) or from a remote machine (e.g. a browser
// with a real GPU, since a VNC desktop only has software GL and can't run WebGL).
// Isaac live_sim's WebSocket listens on 0.0.0.0:8765, so it's reachable either way.
const ISAAC_WS_URL   = (() => {
  const p = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${p}://${location.hostname || 'localhost'}:8765`;
})();
const BACKEND_WS_URL = (() => {
  const p = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${p}://${location.host}/ws`;
})();

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
const cellState = {};
ALL_BINS.forEach(id => { cellState[id] = { state: 'idle', part: null, qty: null }; });
let currentRole = 'user';
let inspectAllRunning = false;

// ---------------------------------------------------------------------------
// DOM helpers
// ---------------------------------------------------------------------------
const $  = s => document.querySelector(s);
const $$ = s => document.querySelectorAll(s);

const beDot        = $('#be-dot');
const beLabel      = $('#be-label');
const isaacDot     = $('#isaac-dot');
const isaacLabel   = $('#isaac-label');
const roleSelect   = $('#role-select');
const binInput     = $('#bin-input');
const btnInspect   = $('#btn-inspect');
const btnInspectAll = $('#btn-inspect-all');
const btnHome      = $('#btn-home');
const alertCont    = $('#alert-container');
const binGrid      = $('#bin-grid');
const historyTbody = $('#history-tbody');
const sapSection   = $('#sap-form-section');
const sapBin       = $('#sap-bin');
const sapPart      = $('#sap-part');
const sapQty       = $('#sap-qty');
const btnSapUpdate = $('#btn-sap-update');
const sapMsg       = $('#sap-msg');

// camera
const cameraImg         = $('#camera-img');
const cameraPlaceholder = $('#camera-placeholder');
const placeholderText   = $('#placeholder-text');
const liveDot           = $('#live-dot');
const liveText          = $('#live-text');
const teleHud           = $('#tele-hud');
const detStrip          = $('#det-strip');

// map
const mapCanvas    = $('#map-canvas');
const mapStatus    = $('#map-status');
const lidarHud     = $('#lidar-hud');
const lidarShowChk = $('#lidar-show');

// ---------------------------------------------------------------------------
// Escape HTML
// ---------------------------------------------------------------------------
function esc(s) {
  if (s == null) return '';
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

// ============================================================================
//  LIVE CAMERA  (ws://localhost:8765)
// ============================================================================
let isaacWs = null;
let isaacRetryDelay = 2000;
let isaacRetryTimer = null;
let firstFrameSeen = false;

function connectIsaac() {
  setIsaacStatus('pend', 'Connecting to Isaac…');
  isaacWs = new WebSocket(ISAAC_WS_URL);

  isaacWs.onopen = () => {
    setIsaacStatus('ok', 'Isaac connected');
    isaacRetryDelay = 2000;
    placeholderText.textContent = 'Waiting for first frame…';
  };

  isaacWs.onmessage = (evt) => {
    let msg;
    try { msg = JSON.parse(evt.data); } catch { return; }
    if (msg.type === 'frame')      handleFrame(msg);
    else if (msg.type === 'telemetry') handleTelemetry(msg);
    else if (msg.type === 'inspection') handleInspectionIsaac(msg);
    else if (msg.type === 'lidar') handleLidar(msg);
  };

  isaacWs.onclose = () => {
    setIsaacStatus('err', 'Isaac disconnected — retrying…');
    firstFrameSeen = false;
    liveDot.classList.remove('on');
    liveText.textContent = 'Isaac disconnected';
    isaacRetryTimer = setTimeout(() => {
      isaacRetryDelay = Math.min(isaacRetryDelay * 1.5, 30000);
      connectIsaac();
    }, isaacRetryDelay);
  };

  isaacWs.onerror = () => { isaacWs.close(); };
}

function setIsaacStatus(state, label) {
  isaacDot.className = 'conn-dot ' + (state === 'ok' ? 'ok' : state === 'err' ? 'err' : '');
  isaacLabel.textContent = label;
}

function handleFrame(msg) {
  if (!firstFrameSeen) {
    firstFrameSeen = true;
    cameraPlaceholder.style.display = 'none';
    cameraImg.style.display = 'block';
    liveDot.classList.add('on');
    liveText.textContent = 'LIVE';
  }
  cameraImg.src = 'data:image/jpeg;base64,' + msg.jpeg;
}

function handleTelemetry(msg) {
  const pos   = (msg.pos   || [0,0,0]).map(v => Number(v).toFixed(2));
  const state = msg.state  || 'IDLE';
  const tgt   = msg.target || '—';
  const clr   = msg.clearance != null ? Number(msg.clearance).toFixed(2) : '—';
  const lmin  = msg.lidar_min != null ? Number(msg.lidar_min).toFixed(2) : null;

  // HUD line
  let hudStr = `state: ${state}  |  pos: (${pos.join(', ')})  |  target: ${tgt}  |  clearance: ${clr}m`;
  if (lmin !== null) hudStr += `  |  lidar_min: ${lmin}m`;
  teleHud.textContent = hudStr;

  // Detections strip
  const dets = msg.detections || [];
  if (dets.length) {
    detStrip.textContent = dets.map(d =>
      `${esc(d.part_no ?? '?')} qty=${d.qty ?? '?'}`).join('  ·  ');
  } else {
    detStrip.textContent = '';
  }

  // Mark the targeted bin as scanning in the grid (visual only from telemetry)
  if (tgt && tgt !== '—' && ALL_BINS.includes(tgt)) {
    if (cellState[tgt].state === 'idle') {
      updateCell(tgt, 'scanning', cellState[tgt].part, cellState[tgt].qty);
    }
  }

  // Update 3D drone marker position
  if (msg.pos) moveDroneMarker(msg.pos);
}

function handleInspectionIsaac(msg) {
  // Comes from live_sim WS (the real drone completed scanning)
  const bid = (msg.bin_id || '').toUpperCase();
  const state = msg.status === 'discrepancy' ? 'discrepancy' : 'completed';
  updateCell(bid, state, msg.scanned_part ?? null, msg.scanned_qty ?? null);

  // Colour bin in 3D map
  setBinColor3D(bid, state);

  // History row
  prependHistoryRow({
    bin_id:       msg.bin_id,
    timestamp:    new Date(msg.ts * 1000).toISOString(),
    scanned_part: msg.scanned_part,
    scanned_qty:  msg.scanned_qty,
    system_part:  msg.system_part,
    system_qty:   msg.system_qty,
    status:       msg.status,
    latency_s:    null,
  });

  // Alert on discrepancy
  if (msg.status === 'discrepancy') {
    showAlert(
      `Discrepancy at ${bid.toUpperCase()}: ` +
      `scanned ${esc(msg.scanned_part ?? '?')} / qty ${msg.scanned_qty ?? '?'} ` +
      `vs system ${esc(msg.system_part ?? '?')} / qty ${msg.system_qty ?? '?'}`,
      bid
    );
  }
}

// ---------------------------------------------------------------------------
//  LiDAR point cloud handler
// ---------------------------------------------------------------------------
function handleLidar(msg) {
  const pts = msg.points;
  if (!pts || pts.length === 0) return;

  _lastLidarN = msg.n || pts.length;

  // Compute min distance from drone origin for the HUD
  let minDist = Infinity;
  for (const p of pts) {
    const d = Math.sqrt(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
    if (d > 0.05 && d < minDist) minDist = d;
  }
  _lastLidarMin = isFinite(minDist) ? minDist : null;

  // Update LiDAR HUD
  if (lidarHud) {
    const minStr = _lastLidarMin != null ? _lastLidarMin.toFixed(2) + ' m' : '— m';
    lidarHud.textContent = `LiDAR: ${_lastLidarN} pts, min ${minStr}`;
  }

  // Add new scan points to rolling buffer (flat Float32 triples)
  for (const p of pts) {
    lidarBuffer.push(p[0], p[1], p[2]);
  }
  // Trim to cap (remove oldest points)
  const maxFloats = MAX_LIDAR_PTS * 3;
  if (lidarBuffer.length > maxFloats) {
    lidarBuffer.splice(0, lidarBuffer.length - maxFloats);
  }

  // Update Three.js BufferGeometry
  if (lidarGeo) {
    const arr = lidarGeo.attributes.position.array;
    const count = Math.min(lidarBuffer.length / 3, MAX_LIDAR_PTS);
    const offset = lidarBuffer.length - count * 3;
    for (let i = 0; i < count * 3; i++) {
      arr[i] = lidarBuffer[offset + i];
    }
    lidarGeo.attributes.position.needsUpdate = true;
    lidarGeo.setDrawRange(0, count);
    if (lidarPoints) lidarPoints.visible = lidarShowEnabled;
  }
}

// Send command to Isaac via WS 8765
function sendIsaacCmd(cmd) {
  if (!isaacWs || isaacWs.readyState !== WebSocket.OPEN) {
    showTempMsg('Isaac WS not connected — start live_sim first.');
    return false;
  }
  isaacWs.send(JSON.stringify(cmd));
  return true;
}

// ============================================================================
//  BACKEND WebSocket (/ws) — receives inspection broadcast from REST pipeline
// ============================================================================
let beWs = null;
let beRetryDelay = 2000;

function connectBackend() {
  setBeStatus('pend', 'Backend…');
  beWs = new WebSocket(BACKEND_WS_URL);
  beWs.onopen = () => {
    setBeStatus('ok', 'Backend OK');
    beRetryDelay = 2000;
  };
  beWs.onmessage = (evt) => {
    let msg; try { msg = JSON.parse(evt.data); } catch { return; }
    if (msg.event === 'inspection' && msg.data) applyResult(msg.data);
    else if (msg.event === 'alert') {
      const d = msg.data || {};
      showAlert(
        `Discrepancy at ${esc((msg.bin_id||'').toUpperCase())}: ` +
        `scanned ${esc(d.scanned_part??'?')} / qty ${d.scanned_qty??'?'} ` +
        `vs SAP ${esc(d.system_part??'?')} / qty ${d.system_qty??'?'}`,
        msg.bin_id
      );
    }
  };
  beWs.onclose = () => {
    setBeStatus('err', 'Backend disconnected');
    setTimeout(() => { beRetryDelay = Math.min(beRetryDelay*1.5, 30000); connectBackend(); }, beRetryDelay);
  };
  beWs.onerror = () => { beWs.close(); };
}

function setBeStatus(state, label) {
  beDot.className   = 'conn-dot ' + (state==='ok'?'ok': state==='err'?'err':'');
  beLabel.textContent = label;
}

// ============================================================================
//  BIN GRID
// ============================================================================
function buildGrid() {
  binGrid.innerHTML = '';
  COLUMNS.forEach((col, ci) => {
    LEVELS.forEach((lvl, ri) => {
      const id = `${col}${lvl}`;
      const cell = document.createElement('div');
      cell.className = 'bin-cell';
      cell.id = `cell-${id}`;
      cell.dataset.binId = id;
      cell.style.gridColumn = ci + 1;
      cell.style.gridRow    = ri + 1;
      cell.innerHTML = `
        <span class="bin-id">${id}</span>
        <span class="bin-part" id="part-${id}">—</span>
        <span class="bin-qty"  id="qty-${id}"></span>
      `;
      cell.addEventListener('click', () => onCellClick(id));
      binGrid.appendChild(cell);
    });
  });
}

function updateCell(binId, state, part, qty) {
  const cell = $(`#cell-${binId}`);
  if (!cell) return;
  cell.className = `bin-cell ${state}`;
  const pe = $(`#part-${binId}`);
  const qe = $(`#qty-${binId}`);
  if (pe) pe.textContent = part || '—';
  if (qe) qe.textContent = qty != null ? `Qty: ${qty}` : '';
  cellState[binId] = { state, part, qty };
}

function onCellClick(binId) {
  if (currentRole === 'viewer') return;
  if (cellState[binId]?.state === 'scanning') return;
  binInput.value = binId;
  // Send inspect command to Isaac live drone
  updateCell(binId, 'scanning', cellState[binId].part, cellState[binId].qty);
  setBinColor3D(binId, 'scanning');
  sendIsaacCmd({ type: 'cmd', action: 'inspect', bin_id: binId });
}

async function loadBins() {
  try {
    const r = await fetch('/api/bins');
    if (!r.ok) return;
    const d = await r.json();
    (d.bins || []).forEach(b => {
      if (cellState[b.bin_id]?.state === 'idle')
        updateCell(b.bin_id, 'idle', b.part_no, b.qty);
    });
  } catch(_) {}
}

// ---------------------------------------------------------------------------
// Inspect via REST (also used for "Inspect All" — drives backend pipeline)
// ---------------------------------------------------------------------------
async function inspectBinRest(binId) {
  binId = binId.toUpperCase().trim();
  if (!ALL_BINS.includes(binId)) { showTempMsg(`Unknown bin: ${binId}`); return null; }
  updateCell(binId, 'scanning', cellState[binId].part, cellState[binId].qty);
  try {
    const res = await fetch(`/api/inspect/${binId}`, { method: 'POST' });
    const data = await res.json();
    if (!res.ok) {
      updateCell(binId, 'idle', null, null);
      showTempMsg(data.detail || `Error inspecting ${binId}`);
      return null;
    }
    applyResult(data);
    return data;
  } catch (err) {
    updateCell(binId, 'idle', null, null);
    console.error('inspectBinRest error:', err);
    return null;
  }
}

function applyResult(data) {
  const binId = (data.bin_id || '').toUpperCase();
  const state = data.status === 'discrepancy' ? 'discrepancy' : 'completed';
  updateCell(binId, state, data.scanned_part ?? data.system_part ?? null,
             data.scanned_qty ?? data.system_qty ?? null);
  setBinColor3D(binId, state);
  prependHistoryRow(data);
}

async function inspectAll() {
  if (inspectAllRunning) return;
  inspectAllRunning = true;
  btnInspectAll.disabled = true;
  btnInspect.disabled    = true;
  for (const id of ALL_BINS) {
    // For live demo: send Isaac command, then short wait
    if (isaacWs && isaacWs.readyState === WebSocket.OPEN) {
      sendIsaacCmd({ type: 'cmd', action: 'inspect', bin_id: id });
      await new Promise(r => setTimeout(r, 4000)); // wait for drone
    } else {
      await inspectBinRest(id);
      await new Promise(r => setTimeout(r, 300));
    }
  }
  inspectAllRunning = false;
  applyRolePermissions();
}

// ---------------------------------------------------------------------------
// Controls
// ---------------------------------------------------------------------------
btnInspect.addEventListener('click', () => {
  const id = binInput.value.trim();
  if (!id) return;
  const bid = id.toUpperCase();
  if (!ALL_BINS.includes(bid)) { showTempMsg(`Unknown bin: ${id}`); return; }
  updateCell(bid, 'scanning', cellState[bid].part, cellState[bid].qty);
  setBinColor3D(bid, 'scanning');
  if (!sendIsaacCmd({ type: 'cmd', action: 'inspect', bin_id: bid })) {
    // fallback to REST if Isaac not connected
    inspectBinRest(bid);
  }
});
binInput.addEventListener('keydown', e => { if (e.key === 'Enter') btnInspect.click(); });
btnInspectAll.addEventListener('click', inspectAll);
btnHome.addEventListener('click', () => {
  sendIsaacCmd({ type: 'cmd', action: 'home' });
});

// ---------------------------------------------------------------------------
// RBAC
// ---------------------------------------------------------------------------
function applyRolePermissions() {
  const can = currentRole !== 'viewer' && !inspectAllRunning;
  btnInspect.disabled    = !can;
  btnInspectAll.disabled = !can;
  btnHome.disabled       = currentRole === 'viewer';
  binInput.disabled      = !can;
  sapSection.classList.toggle('visible', currentRole === 'admin');
  $$('.bin-cell').forEach(c => { c.style.cursor = currentRole === 'viewer' ? 'default' : 'pointer'; });
}

roleSelect.addEventListener('change', () => {
  currentRole = roleSelect.value;
  applyRolePermissions();
});

// ---------------------------------------------------------------------------
// SAP update (Admin)
// ---------------------------------------------------------------------------
btnSapUpdate.addEventListener('click', async () => {
  const binId = (sapBin.value || '').toUpperCase().trim();
  const part  = sapPart.value.trim();
  const qty   = parseInt(sapQty.value, 10);
  if (!binId || !part || isNaN(qty)) { sapMsg.textContent='Fill all fields.'; sapMsg.style.color='#f87171'; return; }
  try {
    const r = await fetch(`/api/sap/${binId}`, {
      method: 'PUT', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({part_no:part, qty}),
    });
    const d = await r.json();
    if (r.ok) { sapMsg.textContent=`SAP updated: ${binId} → ${part} / ${qty}`; sapMsg.style.color='#34d399'; }
    else       { sapMsg.textContent=d.detail||'Update failed.'; sapMsg.style.color='#f87171'; }
  } catch { sapMsg.textContent='Network error.'; sapMsg.style.color='#f87171'; }
  setTimeout(() => { sapMsg.textContent=''; }, 4000);
});

// Prefill SAP form on cell click in admin mode
document.addEventListener('click', e => {
  const cell = e.target.closest('.bin-cell');
  if (cell && currentRole === 'admin') {
    const bid = (cell.dataset.binId || '').toUpperCase();
    sapBin.value = bid;
    if (bid) fetch(`/api/sap/${bid}`).then(r=>r.ok?r.json():null).then(d=>{
      if (d) { sapPart.value=d.part_no||''; sapQty.value=d.qty??''; }
    }).catch(()=>{});
  }
});

// ============================================================================
//  ALERTS
// ============================================================================
function showAlert(message, _binId) {
  const div = document.createElement('div');
  div.className = 'alert-banner';
  div.innerHTML = `<span class="alert-msg">&#x26A0; ${esc(message)}</span><button title="Dismiss">&#x2715;</button>`;
  div.querySelector('button').onclick = () => div.remove();
  alertCont.prepend(div);
  setTimeout(() => { if (div.parentNode) div.remove(); }, 30000);
}

function showTempMsg(msg) {
  const div = document.createElement('div');
  div.className = 'alert-banner';
  div.style.cssText = 'border-color:#f59e0b;background:#451a03';
  div.innerHTML = `<span class="alert-msg">${esc(msg)}</span><button>&#x2715;</button>`;
  div.querySelector('button').onclick = () => div.remove();
  alertCont.prepend(div);
  setTimeout(() => { if (div.parentNode) div.remove(); }, 8000);
}

// ============================================================================
//  HISTORY
// ============================================================================
function prependHistoryRow(data) {
  const row = makeHistoryRow(data);
  historyTbody.firstChild
    ? historyTbody.insertBefore(row, historyTbody.firstChild)
    : historyTbody.appendChild(row);
  while (historyTbody.children.length > 200) historyTbody.removeChild(historyTbody.lastChild);
}

function makeHistoryRow(d) {
  const tr = document.createElement('tr');
  const ts = d.timestamp ? new Date(d.timestamp).toLocaleTimeString() : '—';
  const scanned = `${esc(d.scanned_part??'?')} / ${d.scanned_qty??'?'}`;
  const system  = `${esc(d.system_part ??'?')} / ${d.system_qty ??'?'}`;
  const status  = d.status ?? '';
  tr.innerHTML = `
    <td>${esc((d.bin_id||'').toUpperCase())}</td>
    <td style="font-family:system-ui;font-size:.65rem">${ts}</td>
    <td>${scanned}</td>
    <td>${system}</td>
    <td><span class="badge ${esc(status)}">${esc(status||'—')}</span></td>
  `;
  return tr;
}

async function loadHistory() {
  try {
    const r = await fetch('/api/history');
    if (!r.ok) return;
    const d = await r.json();
    historyTbody.innerHTML = '';
    (d.history || []).forEach(row => historyTbody.appendChild(makeHistoryRow(row)));
  } catch(_) {}
}

// ============================================================================
//  3D MAP  (Three.js)
// ============================================================================
let threeScene, threeRenderer, threeCamera, threeControls;
let droneMesh = null;
const binMeshes = {};  // bin_id -> mesh
const trailPoints = [];
const MAX_TRAIL = 120;
let trailLine = null;
let trailGeo  = null;
let sceneLoaded = false;

// LiDAR point cloud — rolling buffer of recent scans
let lidarPoints = null;        // THREE.Points object (added to scene)
let lidarGeo    = null;        // THREE.BufferGeometry
const MAX_LIDAR_PTS = 8000;    // rolling cap across scans
const lidarBuffer = [];        // accumulated [x,y,z] triples (flat Float32)
let lidarShowEnabled = true;
let _lastLidarN = 0;
let _lastLidarMin = null;

function initThree() {
  const W = mapCanvas.clientWidth  || 600;
  const H = mapCanvas.clientHeight || 300;

  // --- Build the scene graph FIRST, independent of the WebGL renderer. ---
  // The renderer (line: new THREE.WebGLRenderer) can throw if the GPU briefly
  // can't hand out a WebGL context — common on this 8 GB card while Isaac is
  // hammering the RTX. Previously that left threeScene undefined and made
  // buildScene3D() crash ("Cannot read properties of undefined (reading 'add')")
  // with no recovery. Creating the scene up front means objects always get
  // added safely; the renderer is created lazily and retried until it succeeds.
  threeScene = new THREE.Scene();

  threeCamera = new THREE.PerspectiveCamera(50, W/H, 0.01, 200);
  threeCamera.position.set(6, -8, 7);
  threeCamera.up.set(0, 0, 1);
  threeCamera.lookAt(1.2, -0.9, 2.4);

  // Lights
  const amb = new THREE.AmbientLight(0xffffff, 0.45);
  threeScene.add(amb);
  const dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(4, -6, 8);
  threeScene.add(dir);

  // Floor grid (XY plane at z=0, Isaac Z-up)
  const grid = new THREE.GridHelper(20, 40, 0x222840, 0x1a2035);
  grid.rotation.x = Math.PI / 2;  // GridHelper is XZ; rotate so it lies in XY
  grid.position.set(1.2, -2, 0);
  threeScene.add(grid);

  // LiDAR point cloud object (initially empty, updated by handleLidar)
  lidarGeo = new THREE.BufferGeometry();
  const lidarPosAttr = new THREE.BufferAttribute(new Float32Array(MAX_LIDAR_PTS * 3), 3);
  lidarGeo.setAttribute('position', lidarPosAttr);
  lidarGeo.setDrawRange(0, 0);
  const lidarMat = new THREE.PointsMaterial({ color: 0x22d3ee, size: 0.06, sizeAttenuation: true });
  lidarPoints = new THREE.Points(lidarGeo, lidarMat);
  threeScene.add(lidarPoints);

  // Toggle show/hide
  if (lidarShowChk) {
    lidarShowChk.addEventListener('change', () => {
      lidarShowEnabled = lidarShowChk.checked;
      if (lidarPoints) lidarPoints.visible = lidarShowEnabled;
    });
  }

  // If the GPU drops the WebGL context (driver reset / Isaac contention),
  // recreate it instead of leaving the map permanently blank.
  mapCanvas.addEventListener('webglcontextlost', (e) => {
    e.preventDefault();
    console.warn('WebGL context lost — will recreate.');
    dropRenderer();
    if (mapStatus) mapStatus.textContent = 'map: GPU context lost, recovering…';
  }, false);
  mapCanvas.addEventListener('webglcontextrestored', () => { ensureRenderer(); }, false);

  // Try once now; the animate loop keeps (throttled-)retrying until it succeeds.
  ensureRenderer();

  // Animate — renders only when a renderer exists, otherwise keeps retrying.
  function animate() {
    requestAnimationFrame(animate);
    if (!threeRenderer) ensureRenderer();
    if (!threeRenderer) return;
    if (threeControls) threeControls.update();
    try {
      threeRenderer.render(threeScene, threeCamera);
    } catch (e) {
      console.warn('WebGL render error, dropping renderer:', e.message);
      dropRenderer();  // release context; recreated on a later frame by ensureRenderer()
    }
  }
  animate();

  // Resize observer
  const ro = new ResizeObserver(() => {
    if (!threeRenderer || !threeCamera) return;
    const w = mapCanvas.clientWidth, h = mapCanvas.clientHeight;
    if (!w || !h) return;
    threeRenderer.setSize(w, h, false);
    threeCamera.aspect = w / h;
    threeCamera.updateProjectionMatrix();
  });
  ro.observe(mapCanvas);
}

// Drop the renderer AND release its WebGL context. Important: just nulling the
// reference leaks the GL context, and browsers cap live contexts per page — leak
// enough (e.g. across context-loss events) and new contexts get refused, which
// is exactly the "stuck waiting for GPU" failure mode.
function dropRenderer() {
  if (threeRenderer) {
    try { threeRenderer.forceContextLoss(); } catch (_) {}
    try { threeRenderer.dispose(); } catch (_) {}
  }
  threeRenderer = null;
}

// Quick reason string if the browser can't hand out a WebGL context at all.
function webglUnavailableReason() {
  if (typeof window.WebGLRenderingContext === 'undefined') return 'this browser has no WebGL';
  let gl = null;
  try { gl = mapCanvas.getContext('webgl2') || mapCanvas.getContext('webgl'); } catch (_) {}
  if (!gl) return 'the browser refused a WebGL context (GPU process down, too many WebGL tabs/contexts, or hardware acceleration off)';
  return '';  // WebGL is actually available
}

// Lazily (re)create the WebGL renderer with retry + fallback so a transient
// GPU/VRAM hiccup (common when Isaac is rendering) doesn't permanently break the
// 3D map. Self-heals; after repeated failures it backs off and shows actionable
// help instead of an endless "waiting". Returns true once a renderer exists.
let _rendererRetryAt = 0;
let _rendererFails = 0;
function ensureRenderer() {
  if (threeRenderer) return true;
  if (typeof THREE === 'undefined') return false;
  const now = (typeof performance !== 'undefined' ? performance.now() : Date.now());
  if (now < _rendererRetryAt) return false;
  _rendererRetryAt = now + (_rendererFails < 4 ? 1000 : 5000);  // back off after a few fails
  for (const opts of [
    { antialias: true,  powerPreference: 'high-performance', failIfMajorPerformanceCaveat: false },
    { antialias: false, powerPreference: 'default',          failIfMajorPerformanceCaveat: false },
  ]) {
    try {
      const r = new THREE.WebGLRenderer(Object.assign({ canvas: mapCanvas }, opts));
      r.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
      r.setSize(mapCanvas.clientWidth || 600, mapCanvas.clientHeight || 300, false);
      r.setClearColor(0x080b12);
      threeRenderer = r;
      _rendererFails = 0;
      // OrbitControls needs the renderer's DOM element; attach once.
      if (!threeControls && typeof THREE.OrbitControls === 'function') {
        threeControls = new THREE.OrbitControls(threeCamera, threeRenderer.domElement);
        threeControls.enableDamping = true;
        threeControls.dampingFactor = 0.08;
        threeControls.target.set(1.2, -0.9, 2.4);
      }
      if (mapStatus && /GPU|context|waiting|WebGL/.test(mapStatus.textContent || '')) {
        mapStatus.textContent = sceneLoaded ? `${Object.keys(binMeshes).length} bins loaded` : 'loading…';
      }
      return true;
    } catch (e) {
      console.warn('WebGLRenderer init attempt failed:', e && e.message);
    }
  }
  _rendererFails++;
  if (mapStatus) {
    if (_rendererFails >= 4) {
      const why = webglUnavailableReason() || 'WebGL renderer could not be created';
      mapStatus.textContent = `3D map off — WebGL unavailable: ${why}. Fix: close other tabs & fully restart the browser, or enable hardware acceleration (chrome://gpu).`;
      if (_rendererFails === 4) {
        console.error('[3D map] WebGL renderer cannot be created —', why,
          '\nThe live camera is unaffected (plain <img>). Fixes: fully restart the browser to free WebGL contexts, verify chrome://gpu shows "Hardware accelerated", or try another browser.');
      }
    } else {
      mapStatus.textContent = 'map: waiting for GPU…';
    }
  }
  return false;
}

async function loadScene() {
  try {
    mapStatus.textContent = 'fetching /api/scene…';
    const r = await fetch('/api/scene');
    if (!r.ok) throw new Error(`scene HTTP ${r.status}`);
    const s = await r.json();
    buildScene3D(s);
    mapStatus.textContent = `${(s.bins||[]).length} bins loaded`;
    sceneLoaded = true;
  } catch(e) {
    mapStatus.textContent = `scene error: ${e.message}`;
    console.warn('3D scene load failed:', e);
  }
}

// Isaac is Z-up, Three.js is Y-up by default — we keep Three.js native
// with camera.up=(0,0,1) so we can treat world coords directly as (x,y,z)
// where Z = up. No axis remapping needed; just pass pos[] as (x, y, z)=>(x, y, z).
function isaacPos(p) {
  // Isaac X,Y,Z → Three.js X,Y,Z (Z is up in our scene, camera.up=(0,0,1))
  return new THREE.Vector3(p[0], p[1], p[2]);
}

function buildScene3D(s) {
  if (!threeScene) { console.warn('buildScene3D: scene not initialised yet'); return; }
  // ---- Racks ----
  function addRack(rk, color) {
    const cx = (rk.x_min + rk.x_max) / 2;
    const cy = (rk.y_min + rk.y_max) / 2;
    const cz = (rk.z_min + rk.z_max) / 2;
    const sw = rk.x_max - rk.x_min;
    const sd = rk.y_max - rk.y_min;
    const sh = rk.z_max - rk.z_min;
    const geo  = new THREE.BoxGeometry(sw, sd, sh);
    const mat  = new THREE.MeshLambertMaterial({ color, transparent:true, opacity:0.12, depthWrite:false });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(cx, cy, cz);
    threeScene.add(mesh);
    // wire frame
    const edges = new THREE.EdgesGeometry(geo);
    const wire  = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({color, opacity:0.45, transparent:true}));
    wire.position.copy(mesh.position);
    threeScene.add(wire);
  }

  if (s.primary_rack) addRack(s.primary_rack, 0x3b82f6);
  if (s.second_rack)  addRack(s.second_rack,  0x818cf8);

  // ---- Obstacle ----
  if (s.obstacle) {
    const ob = s.obstacle;
    const cx = (ob.x_min+ob.x_max)/2, cy = (ob.y_min+ob.y_max)/2, cz = (ob.z_min+ob.z_max)/2;
    const geo = new THREE.BoxGeometry(ob.x_max-ob.x_min, ob.y_max-ob.y_min, ob.z_max-ob.z_min);
    const mat = new THREE.MeshLambertMaterial({color:0xf59e0b, transparent:true, opacity:0.45});
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(cx, cy, cz);
    threeScene.add(mesh);
  }

  // ---- Bins ----
  (s.bins || []).forEach(b => {
    const [bx, by, bz] = b.world_pos;
    const [bw, bd, bh] = b.box_size || [0.7, 0.5, 0.5];
    const geo  = new THREE.BoxGeometry(bw, bd, bh);
    const mat  = new THREE.MeshLambertMaterial({ color: 0x2a2d3e });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(bx, by, bz);
    mesh.userData.binId = b.bin_id;
    threeScene.add(mesh);
    binMeshes[b.bin_id] = mesh;

    // bin label sprite (simple canvas texture)
    const sprite = makeLabelSprite(b.bin_id);
    sprite.position.set(bx, by - bd/2 - 0.05, bz);
    sprite.scale.set(0.35, 0.18, 1);
    threeScene.add(sprite);
  });

  // ---- Second-rack boxes (the mirror rack across the aisle — visual only,
  //      not inspected, so static cardboard at the same columns/levels) ----
  if (s.second_rack && (s.bins || []).length) {
    const ry = s.second_rack.center ? s.second_rack.center[1]
             : (s.second_rack.y_min + s.second_rack.y_max) / 2;
    const cardboard = new THREE.MeshLambertMaterial({ color: 0x8a6a43 });
    s.bins.forEach(b => {
      const [bx, , bz] = b.world_pos;
      const [bw, bd, bh] = b.box_size || [0.7, 0.5, 0.5];
      const box = new THREE.Mesh(new THREE.BoxGeometry(bw, bd, bh), cardboard);
      box.position.set(bx, ry, bz);
      threeScene.add(box);
    });
  }

  // ---- Drone marker: a small quadrotor (body + X-frame + 4 rotor discs) ----
  droneMesh = new THREE.Group();
  const _bodyMat  = new THREE.MeshLambertMaterial({ color: 0x2b303b });
  const _armMat   = new THREE.MeshLambertMaterial({ color: 0x566072 });
  const _rotorMat = new THREE.MeshLambertMaterial({ color: 0x22c55e });
  const _REACH = 0.20;
  // crossed arms (X-frame) reaching the 4 rotors
  for (const _sgn of [1, -1]) {
    const bar = new THREE.Mesh(new THREE.BoxGeometry(2 * _REACH * Math.SQRT2, 0.03, 0.02), _armMat);
    bar.rotation.z = _sgn * Math.PI / 4;
    droneMesh.add(bar);
  }
  // central body + a forward camera/gimbal nub (points +Y = forward)
  droneMesh.add(new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.18, 0.06), _bodyMat));
  const _nub = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.08, 0.05),
                              new THREE.MeshLambertMaterial({ color: 0x9aa7b8 }));
  _nub.position.set(0, 0.11, -0.015);
  droneMesh.add(_nub);
  // 4 rotor discs at the arm tips (scene is Z-up, so the discs lie in the XY plane)
  for (const [_sx, _sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.085, 0.012, 18), _rotorMat);
    rotor.rotation.x = Math.PI / 2;
    rotor.position.set(_sx * _REACH, _sy * _REACH, 0.04);
    droneMesh.add(rotor);
  }
  droneMesh.position.set(-2, -2, 1); // home
  threeScene.add(droneMesh);

  // Trail
  trailGeo = new THREE.BufferGeometry();
  const positions = new Float32Array(MAX_TRAIL * 3);
  trailGeo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  trailGeo.setDrawRange(0, 0);
  trailLine = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({color:0x22c55e, opacity:0.4, transparent:true}));
  threeScene.add(trailLine);
}

function makeLabelSprite(text) {
  const c = document.createElement('canvas');
  c.width = 64; c.height = 32;
  const ctx = c.getContext('2d');
  ctx.fillStyle = 'rgba(0,0,0,0)';
  ctx.fillRect(0,0,64,32);
  ctx.fillStyle = '#94a3b8';
  ctx.font = 'bold 20px monospace';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText(text, 32, 16);
  const tex = new THREE.CanvasTexture(c);
  const mat = new THREE.SpriteMaterial({ map: tex, transparent: true });
  return new THREE.Sprite(mat);
}

const BIN_COLORS = {
  idle:        0x2a2d3e,
  scanning:    0x92400e,
  completed:   0x14532d,
  discrepancy: 0x7f1d1d,
};

function setBinColor3D(binId, state) {
  const mesh = binMeshes[binId];
  if (!mesh) return;
  mesh.material.color.setHex(BIN_COLORS[state] ?? BIN_COLORS.idle);
}

function moveDroneMarker(pos) {
  if (!droneMesh) return;
  droneMesh.position.set(pos[0], pos[1], pos[2]);

  // Trail
  trailPoints.push(new THREE.Vector3(pos[0], pos[1], pos[2]));
  if (trailPoints.length > MAX_TRAIL) trailPoints.shift();

  const positions = trailGeo.attributes.position.array;
  for (let i = 0; i < trailPoints.length; i++) {
    positions[i*3]   = trailPoints[i].x;
    positions[i*3+1] = trailPoints[i].y;
    positions[i*3+2] = trailPoints[i].z;
  }
  trailGeo.attributes.position.needsUpdate = true;
  trailGeo.setDrawRange(0, trailPoints.length);
}

// ============================================================================
//  INIT
// ============================================================================
document.addEventListener('DOMContentLoaded', async () => {
  buildGrid();
  applyRolePermissions();
  // 3D map must never block the live camera/LiDAR WS connection.
  try { initThree(); } catch (e) { console.error('initThree failed (map disabled):', e); }
  try { await Promise.all([loadBins(), loadHistory(), loadScene()]); }
  catch (e) { console.error('scene/data load failed:', e); }
  connectBackend();
  connectIsaac();
});
