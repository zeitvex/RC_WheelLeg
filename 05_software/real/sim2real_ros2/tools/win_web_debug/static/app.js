'use strict';

const JOINT_NAMES = [
  'FL_H_ABD','FL_H_PIT','FL_KNEE',
  'FR_H_ABD','FR_H_PIT','FR_KNEE',
  'RL_H_ABD','RL_H_PIT','RL_KNEE',
  'RR_H_ABD','RR_H_PIT','RR_KNEE',
  'FL_WHEEL','FR_WHEEL','RL_WHEEL','RR_WHEEL',
];

const $ = id => document.getElementById(id);
const cmd = { vx: 0, vy: 0, yaw: 0 };
let cmdSendTimer = null;
let currentMode = 'UNKNOWN';

// ── API ──────────────────────────────────────────────────────────────────────
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
  cmd.vx = 0; cmd.vy = 0; cmd.yaw = 0;
  $('cmd-vx').value  = 0;
  $('cmd-vy').value  = 0;
  $('cmd-yaw').value = 0;
  $('cmd-vx-v').textContent  = '0.00';
  $('cmd-vy-v').textContent  = '0.00';
  $('cmd-yaw-v').textContent = '0.00';
  $('cmd-display').textContent = 'vx=0.00  vy=0.00  yaw=0.00';
  $('stick').style.transform = 'translate(-50%, -50%)';
  post({ type: 'zero' });
}

// ── Buttons ──────────────────────────────────────────────────────────────────
function setMode(mode) {
  if (mode === 'WEB' && !confirm('确认切换到 WEB 控制？\n请确认机器人安全且速度为 0。')) return;
  post({ type: 'mode', mode });
  appendEvent('MODE_SET', `→ ${mode}`, 'ok');
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

$('btn-disabled').onclick = () => { zeroAll(); setMode('DISABLED'); };
$('btn-remote').onclick   = () => setMode('REMOTE');
$('btn-web').onclick      = () => setMode('WEB');
$('btn-nav').onclick      = () => setMode('NAV');
$('btn-zero').onclick     = zeroAll;
$('btn-estop').onclick    = () => {
  if (confirm('确认触发软急停？')) {
    post({ type: 'estop', data: true });
    zeroAll();
    appendEvent('ESTOP', '软急停已触发', 'bad');
  }
};

// ── Sliders ──────────────────────────────────────────────────────────────────
for (const [id, key] of [['cmd-vx','vx'],['cmd-vy','vy'],['cmd-yaw','yaw']]) {
  $(id).addEventListener('input', e => {
    cmd[key] = parseFloat(e.target.value);
    $(id + '-v').textContent = cmd[key].toFixed(2);
    if (currentMode === 'WEB') sendCmd();
  });
}

// ── Joystick ─────────────────────────────────────────────────────────────────
const joystick = $('joystick');
const stick    = $('stick');
let dragging   = false;

function updateJoystick(clientX, clientY) {
  const rect = joystick.getBoundingClientRect();
  const cx = rect.left + rect.width  / 2;
  const cy = rect.top  + rect.height / 2;
  const maxR = rect.width * 0.42;
  let dx = clientX - cx;
  let dy = clientY - cy;
  const dist = Math.hypot(dx, dy);
  if (dist > maxR) { dx = dx / dist * maxR; dy = dy / dist * maxR; }
  stick.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
  cmd.vx = parseFloat((-(dy / maxR) * 0.8).toFixed(3));
  cmd.vy = parseFloat((  (dx / maxR) * 0.3).toFixed(3));
  $('cmd-vx').value = cmd.vx;
  $('cmd-vy').value = cmd.vy;
  $('cmd-vx-v').textContent = cmd.vx.toFixed(2);
  $('cmd-vy-v').textContent = cmd.vy.toFixed(2);
  if (currentMode === 'WEB') sendCmd();
}

joystick.addEventListener('pointerdown', e => {
  dragging = true;
  joystick.setPointerCapture(e.pointerId);
  updateJoystick(e.clientX, e.clientY);
});
joystick.addEventListener('pointermove', e => { if (dragging) updateJoystick(e.clientX, e.clientY); });
joystick.addEventListener('pointerup',     () => { dragging = false; zeroAll(); });
joystick.addEventListener('pointercancel', () => { dragging = false; zeroAll(); });

// ── Joints grid init ─────────────────────────────────────────────────────────
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
  const pos  = robot.joint_pos    || [];
  const vel  = robot.joint_vel    || [];
  const tau  = robot.joint_torque || [];
  const upd  = robot.update_counts || [];
  for (let i = 0; i < 16; i++) {
    const dot = $('ms-' + i);
    const cnt = upd[i] ?? 0;
    if (dot) dot.style.color = cnt > 0 ? '#30d158' : '#ef4444';
    const p = $('mp-' + i); if (p) p.textContent = (pos[i] || 0).toFixed(2);
    const v = $('mv-' + i); if (v) v.textContent = (vel[i] || 0).toFixed(2);
    const t = $('mt-' + i);
    if (t) {
      t.textContent = (tau[i] || 0).toFixed(2);
      t.style.color = Math.abs(tau[i] || 0) > 16 ? '#ff453a' : '#ff9f0a';
    }
  }
}

// ── State polling ─────────────────────────────────────────────────────────────
function setText(id, text, cls) {
  const el = $(id);
  if (!el) return;
  el.textContent = text;
  if (cls !== undefined) el.className = 'diag-value ' + cls;
}

function applyState(data) {
  const connected = data.connected &&
    (!data.local_receive_time || Date.now() / 1000 - data.local_receive_time < 2.5);

  const stage = $('stage');
  if (stage) {
    if (!connected) {
      stage.textContent = 'DISCONNECTED';
      stage.className   = 'stage DISCONNECTED';
      return;
    }
  }

  const mode = data.mode || 'UNKNOWN';
  if (mode !== currentMode) highlightMode(mode);

  const rt = data.runtime || {};
  const src = rt.target_source || '--';
  const srcCls = src === 'safety_brake' ? 'bad'
               : src === 'runtime_policy' ? 'ok'
               : src === 'runtime_zero_hold' ? 'active' : '';
  setText('d-source',   src,                      srcCls);
  setText('d-released', String(rt.runtime_released ?? '--'),
    rt.runtime_released ? 'ok' : '');
  setText('d-alpha',    (rt.release_alpha ?? '--') !== '--'
    ? Number(rt.release_alpha).toFixed(2) : '--');
  setText('d-zero',     String(rt.zero_command ?? '--'));
  setText('d-estop',    String(data.estop ?? '--'),
    data.estop ? 'bad' : 'ok');
  setText('d-mux',      data.mux_status || '--');

  const robot = data.robot || {};
  const imuAge = robot.imu_age_ms ?? null;
  setText('d-imu-fresh', String(robot.imu_fresh ?? '--'),
    robot.imu_fresh ? 'ok' : 'bad');
  setText('d-imu-age',   imuAge !== null ? imuAge.toFixed(1) : '--',
    imuAge !== null ? (imuAge > 200 ? 'bad' : imuAge > 60 ? 'warn' : 'ok') : '');

  const grav = robot.projected_gravity;
  setText('d-gravity', grav ? grav.map(v => Number(v).toFixed(2)).join(', ') : '--',
    grav && grav[2] < -0.5 ? 'ok' : 'warn');

  setText('d-holdover',  String(robot.holdover_count ?? '--'),
    (robot.holdover_count || 0) > 10 ? 'warn' : '');

  const odomAge = robot.odom_age_ms ?? null;
  setText('d-odom-age', odomAge !== null ? odomAge.toFixed(1) : '--',
    odomAge !== null ? (odomAge > 500 ? 'bad' : odomAge > 200 ? 'warn' : 'ok') : '');

  const lp = robot.odom_local_pos;
  setText('d-odom-pos', lp ? `x=${Number(lp[0]).toFixed(2)} y=${Number(lp[1]).toFixed(2)}` : '--');

  const cv = data.cmd_vel || {};
  const lin = cv.linear  || {};
  const ang = cv.angular || {};
  setText('cv-vx',  (lin.x ?? 0).toFixed(3));
  setText('cv-vy',  (lin.y ?? 0).toFixed(3));
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

// ── WEB mode heartbeat ────────────────────────────────────────────────────────
setInterval(() => {
  if (currentMode === 'WEB' && !dragging) sendCmd();
}, 50);

// ── Event log ─────────────────────────────────────────────────────────────────
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

// ── Init ──────────────────────────────────────────────────────────────────────
initJointsGrid();
setInterval(poll, 100);
appendEvent('READY', '页面已加载，等待 Nano 连接', 'ok');
