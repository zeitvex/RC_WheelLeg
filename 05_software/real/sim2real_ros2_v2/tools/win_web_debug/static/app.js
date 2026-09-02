'use strict';

const JOINT_NAMES = [
  'FL_H_ABD', 'FL_H_PIT', 'FL_KNEE',
  'FR_H_ABD', 'FR_H_PIT', 'FR_KNEE',
  'RL_H_ABD', 'RL_H_PIT', 'RL_KNEE',
  'RR_H_ABD', 'RR_H_PIT', 'RR_KNEE',
  'FL_WHEEL', 'FR_WHEEL', 'RL_WHEEL', 'RR_WHEEL',
];

const $ = id => document.getElementById(id);
const cmd = { vx: 0, vy: 0, yaw: 0 };
let currentMode = 'UNKNOWN';
let dragging = false;
let mapModel = null;
let latestPose = null;
let lastGoal = null;
let pendingModePromise = null;

const mapCanvas = $('map-canvas');
const mapCtx = mapCanvas ? mapCanvas.getContext('2d') : null;

async function post(payload) {
  try {
    await fetch('/api/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
  } catch (e) {
    appendEvent('API_ERROR', e.message, 'bad');
  }
}

function sendCmd() {
  post({
    type: 'cmd_vel',
    linear: { x: cmd.vx, y: cmd.vy, z: 0 },
    angular: { x: 0, y: 0, z: cmd.yaw },
  });
  $('cmd-display').textContent =
    `vx=${cmd.vx.toFixed(2)}  vy=${cmd.vy.toFixed(2)}  yaw=${cmd.yaw.toFixed(2)}`;
}

function zeroAll() {
  cmd.vx = 0;
  cmd.vy = 0;
  cmd.yaw = 0;
  $('cmd-vx').value = 0;
  $('cmd-vy').value = 0;
  $('cmd-yaw').value = 0;
  $('cmd-vx-v').textContent = '0.00';
  $('cmd-vy-v').textContent = '0.00';
  $('cmd-yaw-v').textContent = '0.00';
  $('cmd-display').textContent = 'vx=0.00  vy=0.00  yaw=0.00';
  $('stick').style.transform = 'translate(-50%, -50%)';
  post({ type: 'zero' });
}

function highlightMode(mode) {
  for (const m of ['DISABLED', 'REMOTE', 'WEB', 'NAV']) {
    const btn = $('btn-' + m.toLowerCase());
    if (btn) btn.classList.toggle('active-mode', m === mode);
  }
  const el = $('stage');
  if (el) {
    el.textContent = mode;
    el.className = 'stage ' + mode;
  }
  currentMode = mode;
}

async function setMode(mode) {
  if (mode === 'WEB') {
    const ok = confirm('Confirm switch to WEB control?\nMake sure the robot is safe and velocity is zero.');
    if (!ok) return false;
  }
  await post({ type: 'mode', mode });
  highlightMode(mode);
  appendEvent('MODE_SET', `-> ${mode}`, 'ok');
  return true;
}

async function ensureMode(mode) {
  if (currentMode === mode) return true;
  if (pendingModePromise) {
    const activeMode = await pendingModePromise;
    if (activeMode === mode) return true;
  }
  pendingModePromise = (async () => {
    const ok = await setMode(mode);
    return ok ? mode : currentMode;
  })();
  const resolvedMode = await pendingModePromise;
  pendingModePromise = null;
  return resolvedMode === mode;
}

function worldToCanvas(x, y) {
  if (!mapModel) return null;
  const { minX, minY, scale, pad, drawH } = mapModel;
  return {
    x: pad + (x - minX) * scale,
    y: pad + drawH - (y - minY) * scale,
  };
}

function canvasToWorld(px, py) {
  if (!mapModel) return null;
  const { minX, minY, scale, pad, drawH } = mapModel;
  return {
    x: minX + (px - pad) / scale,
    y: minY + (drawH - (py - pad)) / scale,
  };
}

function buildMapModel(points) {
  if (!points || !points.length || !mapCanvas) return null;
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const [x, y] of points) {
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
  }
  const pad = 20;
  const usableW = mapCanvas.width - pad * 2;
  const usableH = mapCanvas.height - pad * 2;
  const spanX = Math.max(maxX - minX, 1e-6);
  const spanY = Math.max(maxY - minY, 1e-6);
  const scale = Math.min(usableW / spanX, usableH / spanY);
  return { minX, minY, maxX, maxY, scale, pad, drawH: usableH };
}

function drawMap() {
  if (!mapCtx || !mapCanvas) return;
  mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  mapCtx.fillStyle = '#0b1020';
  mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);

  if (!mapModel) {
    mapCtx.fillStyle = '#9ca3af';
    mapCtx.font = '16px sans-serif';
    mapCtx.fillText('Map not loaded', 24, 32);
    return;
  }

  mapCtx.fillStyle = 'rgba(255,255,255,0.28)';
  for (const [x, y] of mapModel.points) {
    const p = worldToCanvas(x, y);
    if (!p) continue;
    mapCtx.fillRect(p.x, p.y, 1.5, 1.5);
  }

  if (latestPose) {
    const p = worldToCanvas(latestPose.x, latestPose.y);
    if (p) {
      mapCtx.fillStyle = '#3b82f6';
      mapCtx.beginPath();
      mapCtx.arc(p.x, p.y, 6, 0, Math.PI * 2);
      mapCtx.fill();
      mapCtx.strokeStyle = '#60a5fa';
      mapCtx.lineWidth = 2;
      mapCtx.beginPath();
      mapCtx.moveTo(p.x, p.y);
      mapCtx.lineTo(p.x + Math.cos(latestPose.yaw) * 18, p.y - Math.sin(latestPose.yaw) * 18);
      mapCtx.stroke();
    }
  }

  if (lastGoal) {
    const p = worldToCanvas(lastGoal.x, lastGoal.y);
    if (p) {
      mapCtx.strokeStyle = '#ef4444';
      mapCtx.lineWidth = 2;
      mapCtx.beginPath();
      mapCtx.moveTo(p.x - 7, p.y - 7);
      mapCtx.lineTo(p.x + 7, p.y + 7);
      mapCtx.moveTo(p.x + 7, p.y - 7);
      mapCtx.lineTo(p.x - 7, p.y + 7);
      mapCtx.stroke();
    }
  }
}

async function fetchMap() {
  try {
    const res = await fetch('/api/map');
    const data = await res.json();
    const points = data.points || [];
    mapModel = buildMapModel(points);
    if (mapModel) mapModel.points = points;
    if (data.pose) latestPose = data.pose;
    drawMap();
    appendEvent('MAP', `loaded ${points.length} filtered points`, 'ok');
  } catch (e) {
    appendEvent('MAP_ERROR', e.message, 'bad');
  }
}

function onMapClick(event) {
  if (!mapCanvas || !mapModel) return;
  const rect = mapCanvas.getBoundingClientRect();
  const px = (event.clientX - rect.left) * (mapCanvas.width / rect.width);
  const py = (event.clientY - rect.top) * (mapCanvas.height / rect.height);
  const world = canvasToWorld(px, py);
  if (!world) return;
  lastGoal = world;
  drawMap();
  ensureMode('NAV').then(ok => {
    if (!ok) return;
    post({ type: 'go_to', x: world.x, y: world.y });
    appendEvent('NAV_GO', `x=${world.x.toFixed(2)} y=${world.y.toFixed(2)}`, 'ok');
  });
}

function setText(id, text, cls) {
  const el = $(id);
  if (!el) return;
  el.textContent = text;
  if (cls !== undefined) el.className = 'diag-value ' + cls;
}

function initJointsGrid() {
  const grid = $('joints-grid');
  if (!grid) return;
  grid.innerHTML = JOINT_NAMES.map((name, i) => `
    <div class="motor-row" id="mi-${i}">
      <span class="stale" id="ms-${i}" style="color:#ef4444">●</span>
      <span class="name">${name}</span>
      <span class="val pos" id="mp-${i}">0.00</span>
      <span class="val vel" id="mv-${i}">0.00</span>
      <span class="val tau" id="mt-${i}">0.00</span>
    </div>`).join('');
}

function updateJointsGrid(robot) {
  if (!robot) return;
  const pos = robot.joint_pos || [];
  const vel = robot.joint_vel || [];
  const tau = robot.joint_torque || [];
  const upd = robot.update_counts || [];
  for (let i = 0; i < 16; i++) {
    const dot = $('ms-' + i);
    const cnt = upd[i] ?? 0;
    if (dot) dot.style.color = cnt > 0 ? '#30d158' : '#ef4444';
    const p = $('mp-' + i);
    if (p) p.textContent = (pos[i] || 0).toFixed(2);
    const v = $('mv-' + i);
    if (v) v.textContent = (vel[i] || 0).toFixed(2);
    const t = $('mt-' + i);
    if (t) {
      t.textContent = (tau[i] || 0).toFixed(2);
      t.style.color = Math.abs(tau[i] || 0) > 16 ? '#ff453a' : '#ff9f0a';
    }
  }
}

function applyState(data) {
  const connected = data.connected && (!data.local_receive_time || Date.now() / 1000 - data.local_receive_time < 2.5);
  if (!connected) {
    $('stage').textContent = 'DISCONNECTED';
    $('stage').className = 'stage DISCONNECTED';
    return;
  }

  const mode = data.mode || 'UNKNOWN';
  if (mode !== currentMode) highlightMode(mode);

  const rt = data.runtime || {};
  const src = rt.target_source || '--';
  const srcCls = src === 'safety_brake' ? 'bad' : src === 'runtime_policy' ? 'ok' : src === 'runtime_zero_hold' ? 'active' : '';
  setText('d-source', src, srcCls);
  setText('d-released', String(rt.runtime_released ?? '--'), rt.runtime_released ? 'ok' : '');
  setText('d-alpha', (rt.release_alpha ?? '--') !== '--' ? Number(rt.release_alpha).toFixed(2) : '--');
  setText('d-zero', String(rt.zero_command ?? '--'));
  setText('d-estop', String(data.estop ?? '--'), data.estop ? 'bad' : 'ok');
  setText('d-mux', data.mux_status || '--');

  const robot = data.robot || {};
  const imuAge = robot.imu_age_ms ?? null;
  setText('d-imu-fresh', String(robot.imu_fresh ?? '--'), robot.imu_fresh ? 'ok' : 'bad');
  setText('d-imu-age', imuAge !== null ? imuAge.toFixed(1) : '--', imuAge !== null ? (imuAge > 200 ? 'bad' : imuAge > 60 ? 'warn' : 'ok') : '');
  const grav = robot.projected_gravity;
  setText('d-gravity', grav ? grav.map(v => Number(v).toFixed(2)).join(', ') : '--', grav && grav[2] < -0.5 ? 'ok' : 'warn');
  setText('d-holdover', String(robot.holdover_count ?? '--'), (robot.holdover_count || 0) > 10 ? 'warn' : '');
  const odomAge = robot.odom_age_ms ?? null;
  setText('d-odom-age', odomAge !== null ? odomAge.toFixed(1) : '--', odomAge !== null ? (odomAge > 500 ? 'bad' : odomAge > 200 ? 'warn' : 'ok') : '');
  const lp = robot.odom_local_pos;
  setText('d-odom-pos', lp ? `x=${Number(lp[0]).toFixed(2)} y=${Number(lp[1]).toFixed(2)}` : '--');
  setText('d-nav-status', data.nav_status || '--');

  const nav = data.nav || {};
  if (nav.pose) {
    latestPose = nav.pose;
    setText('d-nav-pose', `x=${nav.pose.x.toFixed(2)} y=${nav.pose.y.toFixed(2)} yaw=${(nav.pose.yaw * 57.2958).toFixed(1)}deg`);
    drawMap();
  }

  const cv = data.cmd_vel || {};
  const lin = cv.linear || {};
  const ang = cv.angular || {};
  setText('cv-vx', (lin.x ?? 0).toFixed(3));
  setText('cv-vy', (lin.y ?? 0).toFixed(3));
  setText('cv-yaw', (ang.z ?? 0).toFixed(3));
  updateJointsGrid(robot);
}

async function poll() {
  try {
    const res = await fetch('/api/state');
    const data = await res.json();
    applyState(data);
  } catch (_) {}
}

function appendEvent(kind, detail, cls) {
  const el = $('events-log');
  if (!el) return;
  const div = document.createElement('div');
  const t = new Date().toLocaleTimeString();
  div.innerHTML = `<span class="ev-t">${t}</span> <span class="ev-${cls || 'ok'}">${kind}</span> <span style="color:#8e8e93">${detail || ''}</span>`;
  el.appendChild(div);
  while (el.children.length > 200) el.removeChild(el.firstChild);
  el.scrollTop = el.scrollHeight;
}

$('btn-disabled').onclick = () => {
  zeroAll();
  setMode('DISABLED');
};
$('btn-remote').onclick = () => setMode('REMOTE');
$('btn-web').onclick = () => setMode('WEB');
$('btn-nav').onclick = () => setMode('NAV');
$('btn-zero').onclick = zeroAll;
$('btn-estop').onclick = () => {
  if (confirm('Confirm soft e-stop?')) {
    post({ type: 'estop', data: true });
    zeroAll();
    appendEvent('ESTOP', 'soft e-stop triggered', 'bad');
  }
};
$('btn-refresh-map').onclick = fetchMap;
$('btn-record').onclick = () => ensureMode('NAV').then(ok => {
  if (ok) post({ type: 'nav_cmd', command: 'record web_p1' });
});
$('btn-stop-nav').onclick = () => ensureMode('NAV').then(ok => {
  if (ok) post({ type: 'nav_cmd', command: 'stop' });
});
$('btn-go-rel').onclick = () => ensureMode('NAV').then(ok => {
  if (ok) post({ type: 'go_rel', dx: 0.3, dy: 0.0 });
});

for (const [id, key] of [['cmd-vx', 'vx'], ['cmd-vy', 'vy'], ['cmd-yaw', 'yaw']]) {
  $(id).addEventListener('input', e => {
    cmd[key] = parseFloat(e.target.value);
    $(id + '-v').textContent = cmd[key].toFixed(2);
    ensureMode('WEB').then(ok => {
      if (ok) sendCmd();
    });
  });
}

const joystick = $('joystick');
const stick = $('stick');

function updateJoystick(clientX, clientY) {
  const rect = joystick.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const maxR = rect.width * 0.42;
  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.hypot(dx, dy);
  if (dist > maxR) {
    dx = dx / dist * maxR;
    dy = dy / dist * maxR;
  }
  stick.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
  cmd.vx = parseFloat((-(dy / maxR) * 0.8).toFixed(3));
  cmd.vy = parseFloat(((dx / maxR) * 0.3).toFixed(3));
  $('cmd-vx').value = cmd.vx;
  $('cmd-vy').value = cmd.vy;
  $('cmd-vx-v').textContent = cmd.vx.toFixed(2);
  $('cmd-vy-v').textContent = cmd.vy.toFixed(2);
  if (currentMode === 'WEB') sendCmd();
}

joystick.addEventListener('pointerdown', e => {
  ensureMode('WEB').then(ok => {
    if (!ok) {
      dragging = false;
      zeroAll();
      return;
    }
    dragging = true;
    joystick.setPointerCapture(e.pointerId);
    updateJoystick(e.clientX, e.clientY);
  });
});
joystick.addEventListener('pointermove', e => {
  if (dragging) updateJoystick(e.clientX, e.clientY);
});
joystick.addEventListener('pointerup', () => {
  dragging = false;
  zeroAll();
});
joystick.addEventListener('pointercancel', () => {
  dragging = false;
  zeroAll();
});

if (mapCanvas) mapCanvas.addEventListener('click', onMapClick);

setInterval(() => {
  if (currentMode === 'WEB' && !dragging) sendCmd();
}, 50);

initJointsGrid();
setInterval(poll, 100);
fetchMap();
drawMap();
appendEvent('READY', 'page loaded, waiting for bridge state', 'ok');
