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
let latestConnected = false;
let pendingModePromise = null;
let currentGoalName = '';
let currentGoalIndex = -1;
let currentGoalTotal = 0;
let latestModelState = { current_model: '--', switch_state: '--', backend: '--', switching: false };
let goalPointsByName = {};
let defaultMissionName = '';
let defaultMissionGoals = [];
let routeAlignmentInfo = {};
let odomFallbackInfo = { active: false };
let activeNavPath = { goal_name: '', stage: 'idle', path_index: 0, points: [] };

const mapCanvas = $('map-canvas');
const mapCtx = mapCanvas ? mapCanvas.getContext('2d') : null;

async function post(payload) {
  try {
    const res = await fetch('/api/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) {
      throw new Error(data.error || `HTTP ${res.status}`);
    }
    return { ok: true, data };
  } catch (e) {
    appendEvent('API_ERROR', e.message, 'bad');
    return { ok: false, error: e.message };
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
  updateTaskButtonState();
}

function updateTaskButtonState() {
  const btn = $('btn-run-task');
  const odomBtn = $('btn-odom-task');
  const odomStopBtn = $('btn-odom-stop');
  const hasMission = Boolean(defaultMissionName);
  const runEnabled = latestConnected && currentMode === 'NAV' && hasMission;
  const odomEnabled = latestConnected && hasMission;

  if (btn) {
    btn.disabled = !runEnabled;
    btn.textContent = hasMission ? `Run ${defaultMissionName}` : 'Run Mission';
    if (!hasMission) {
      btn.title = 'No default mission loaded from backend';
    } else {
      btn.title = runEnabled
        ? `Run default mission ${defaultMissionName}`
        : 'Available only in NAV mode while connected';
    }
  }

  if (odomBtn) {
    odomBtn.disabled = !odomEnabled;
    odomBtn.title = hasMission
      ? 'Initialize map->odom from waypoint id1, switch to NAV, then run the current route'
      : 'No default mission loaded from backend';
  }

  if (odomStopBtn) {
    odomStopBtn.disabled = !(latestConnected && odomFallbackInfo.active);
    odomStopBtn.title = odomFallbackInfo.active
      ? 'Stop pure odom fallback and stop simple_nav'
      : 'Odom fallback is not active';
  }
}

function setText(id, text, cls) {
  const el = $(id);
  if (!el) return;
  el.textContent = text;
  if (cls !== undefined) el.className = 'diag-value ' + cls;
}

function formatNumber(value, digits = 2) {
  return Number.isFinite(value) ? Number(value).toFixed(digits) : '--';
}

function setLastGoalByName(goalName) {
  const goal = goalPointsByName[goalName];
  if (!goal) return false;
  lastGoal = { x: goal.x, y: goal.y };
  currentGoalName = goalName;
  currentGoalIndex = defaultMissionGoals.findIndex(goalItem => goalItem.name === goalName);
  setText('d-active-goal', goalName, 'active');
  renderTaskGoals();
  return true;
}

function syncGoalMarkerFromNavStatus(statusText) {
  if (!statusText) return;
  const missionStatusMatch = statusText.match(/waypoint\s+(\d+)\/(\d+)\s*->\s*([A-Za-z0-9_()=.,-]+)\b/i);
  if (missionStatusMatch) {
    currentGoalIndex = Math.max(0, Number.parseInt(missionStatusMatch[1], 10) - 1);
    currentGoalTotal = Math.max(0, Number.parseInt(missionStatusMatch[2], 10));
    setLastGoalByName(missionStatusMatch[3]);
    return;
  }
  const nextGoalMatch = statusText.match(/->\s*([A-Za-z0-9_()=.,-]+)\b/);
  if (nextGoalMatch && setLastGoalByName(nextGoalMatch[1])) {
    return;
  }
  const reachedMatch = statusText.match(/reached\s+([A-Za-z0-9_()=.,-]+)\b/i);
  if (reachedMatch && setLastGoalByName(reachedMatch[1])) {
    return;
  }
  if (/navigation stopped:/i.test(statusText)) {
    currentGoalName = '';
    currentGoalIndex = -1;
    currentGoalTotal = 0;
    setText('d-active-goal', '--');
    renderTaskGoals();
  }
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
  if (!mapCanvas) return null;
  const boundsPoints = [...(points || [])];
  for (const goal of defaultMissionGoals) boundsPoints.push([goal.x, goal.y]);
  if (!boundsPoints.length) return null;

  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const [x, y] of boundsPoints) {
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
  return { minX, minY, maxX, maxY, scale, pad, drawH: usableH, points: points || [] };
}

function goalColor(goal) {
  return goal.policy === 'crawl' ? '#30d158' : '#ffd60a';
}

function drawMissionPath() {
  if (!mapCtx || defaultMissionGoals.length < 2) return;
  mapCtx.save();
  mapCtx.lineWidth = 2;
  mapCtx.strokeStyle = 'rgba(34, 197, 94, 0.95)';
  mapCtx.beginPath();
  for (let i = 0; i < defaultMissionGoals.length; i += 1) {
    const goal = defaultMissionGoals[i];
    const p = worldToCanvas(goal.x, goal.y);
    if (!p) continue;
    if (i === 0) mapCtx.moveTo(p.x, p.y);
    else mapCtx.lineTo(p.x, p.y);
  }
  mapCtx.stroke();

  if (currentGoalIndex > 0) {
    mapCtx.strokeStyle = 'rgba(10, 132, 255, 0.95)';
    mapCtx.lineWidth = 3;
    mapCtx.beginPath();
    let started = false;
    for (let i = 0; i <= Math.min(currentGoalIndex, defaultMissionGoals.length - 1); i += 1) {
      const goal = defaultMissionGoals[i];
      const p = worldToCanvas(goal.x, goal.y);
      if (!p) continue;
      if (!started) {
        mapCtx.moveTo(p.x, p.y);
        started = true;
      } else {
        mapCtx.lineTo(p.x, p.y);
      }
    }
    if (started) mapCtx.stroke();
  }
  mapCtx.restore();
}

function drawActiveNavPath() {
  if (!mapCtx || !activeNavPath || !Array.isArray(activeNavPath.points) || activeNavPath.points.length < 2) return;
  mapCtx.save();
  mapCtx.lineWidth = 2.5;
  mapCtx.strokeStyle = 'rgba(168, 85, 247, 0.95)';
  mapCtx.beginPath();
  let started = false;
  for (const point of activeNavPath.points) {
    if (!Array.isArray(point) || point.length < 2) continue;
    const p = worldToCanvas(point[0], point[1]);
    if (!p) continue;
    if (!started) {
      mapCtx.moveTo(p.x, p.y);
      started = true;
    } else {
      mapCtx.lineTo(p.x, p.y);
    }
  }
  if (started) mapCtx.stroke();

  const highlightIndex = Math.min(
    Math.max(0, Number(activeNavPath.path_index || 0)),
    activeNavPath.points.length - 1,
  );
  const highlightPoint = activeNavPath.points[highlightIndex];
  if (Array.isArray(highlightPoint) && highlightPoint.length >= 2) {
    const p = worldToCanvas(highlightPoint[0], highlightPoint[1]);
    if (p) {
      mapCtx.fillStyle = '#c084fc';
      mapCtx.beginPath();
      mapCtx.arc(p.x, p.y, 5, 0, Math.PI * 2);
      mapCtx.fill();
    }
  }
  mapCtx.restore();
}

function drawGoal(goal, index) {
  const p = worldToCanvas(goal.x, goal.y);
  if (!p || !mapCtx) return;
  const active = goal.name === currentGoalName;
  const completed = currentGoalIndex > index;
  mapCtx.fillStyle = active ? '#ff9f0a' : completed ? '#67e8f9' : goalColor(goal);
  mapCtx.beginPath();
  mapCtx.arc(p.x, p.y, active ? 7 : 5, 0, Math.PI * 2);
  mapCtx.fill();
  mapCtx.strokeStyle = '#111827';
  mapCtx.lineWidth = 1.5;
  mapCtx.stroke();

  mapCtx.fillStyle = active ? '#ffcf66' : '#ffe680';
  mapCtx.font = '12px sans-serif';
  mapCtx.fillText(`${index + 1}.${goal.name}`, p.x + 8, p.y - 8);
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

  drawMissionPath();
  drawActiveNavPath();

  for (const [index, goal] of defaultMissionGoals.entries()) {
    drawGoal(goal, index);
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

function formatRouteAlignment(info) {
  if (!info || Object.keys(info).length === 0) return '--';
  const applied = Number.isFinite(info.applied_deg) ? `${Number(info.applied_deg).toFixed(2)}deg` : '--';
  const hits = Number.isFinite(info.hits) && Number.isFinite(info.total) ? `${info.hits}/${info.total}` : '--';
  const reason = info.reason || (info.enabled ? 'auto align' : 'fixed');
  return `${applied} hits=${hits} ${reason}`;
}

function formatOdomFallback(info) {
  if (!info || !info.active) return 'inactive';
  const anchor = info.anchor || 'id1';
  const pose = info.map_pose || {};
  const x = Number.isFinite(pose.x) ? Number(pose.x).toFixed(2) : '--';
  const y = Number.isFinite(pose.y) ? Number(pose.y).toFixed(2) : '--';
  const yaw = Number.isFinite(pose.yaw_deg) ? Number(pose.yaw_deg).toFixed(1) : '--';
  const handoff = info.handoff_pending ? ' handoff pending' : '';
  return `${anchor} x=${x} y=${y} yaw=${yaw}deg${handoff}`;
}

function renderTaskGoals() {
  const list = $('task-goals-list');
  if (!list) return;
  if (!defaultMissionGoals.length) {
    list.innerHTML = '<div class="task-goal-item empty">No mission goals loaded</div>';
    return;
  }
  list.innerHTML = defaultMissionGoals.map((goal, index) => {
    const yawText = Number.isFinite(goal.yaw_deg) ? `${Number(goal.yaw_deg).toFixed(1)}deg` : '--';
    const yawTolText = Number.isFinite(goal.yaw_tolerance_deg) ? `${Number(goal.yaw_tolerance_deg).toFixed(1)}deg` : '--';
    const tolText = Number.isFinite(goal.tolerance) ? Number(goal.tolerance).toFixed(2) : '--';
    const policyText = goal.policy || 'rough';
    const requireYawText = goal.require_yaw ? ' strict_yaw' : '';
    const preDockText = Number.isFinite(goal.pre_dock_distance) ? ` pre_dock=${Number(goal.pre_dock_distance).toFixed(2)}` : '';
    const activeClass = goal.name === currentGoalName ? ' active' : '';
    return `
      <div class="task-goal-item${activeClass}">
        <div class="task-goal-head">
          <span class="task-goal-index">${index + 1}</span>
          <span class="task-goal-name">${goal.name}</span>
          <span class="task-goal-policy ${policyText}">${policyText}</span>
        </div>
        <div class="task-goal-meta">x=${formatNumber(goal.x)} y=${formatNumber(goal.y)} yaw=${yawText} yaw_tol=${yawTolText} tol=${tolText}${requireYawText}${preDockText}</div>
      </div>`;
  }).join('');
}

function applyMapMetadata(data) {
  const goalSpecs = Array.isArray(data.goal_specs) ? data.goal_specs : [];
  goalPointsByName = {};
  for (const goal of goalSpecs) {
    if (!goal || !goal.name) continue;
    goalPointsByName[goal.name] = goal;
  }
  defaultMissionName = data.default_mission_name || '';
  defaultMissionGoals = Array.isArray(data.default_mission_goals) ? data.default_mission_goals : [];
  routeAlignmentInfo = data.route_alignment || {};
  odomFallbackInfo = data.odom_fallback || odomFallbackInfo;
  activeNavPath = data.nav_path || activeNavPath;
  currentGoalIndex = currentGoalName
    ? defaultMissionGoals.findIndex(goal => goal.name === currentGoalName)
    : -1;
  currentGoalTotal = defaultMissionGoals.length;

  setText('d-task-mission', defaultMissionName || '--', defaultMissionName ? 'ok' : '');
  setText('d-task-goals', String(defaultMissionGoals.length || 0), defaultMissionGoals.length ? 'ok' : '');
  setText('d-route-align', formatRouteAlignment(routeAlignmentInfo), routeAlignmentInfo.enabled ? 'ok' : 'warn');
  setText(
    'd-odom-fallback',
    formatOdomFallback(odomFallbackInfo),
    odomFallbackInfo.handoff_pending ? 'warn' : odomFallbackInfo.active ? 'active' : '',
  );
  updateTaskButtonState();
  renderTaskGoals();
}

async function fetchMap() {
  try {
    const res = await fetch('/api/map');
    const data = await res.json();
    applyMapMetadata(data);
    const points = Array.isArray(data.points) ? data.points : [];
    mapModel = buildMapModel(points);
    if (data.pose) latestPose = data.pose;
    drawMap();
    appendEvent('MAP', `loaded ${points.length} filtered points, mission=${defaultMissionName || 'none'}`, 'ok');
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
  currentGoalName = '';
  setText('d-active-goal', 'direct goal', 'active');
  renderTaskGoals();
  drawMap();
  ensureMode('NAV').then(async ok => {
    if (!ok) return;
    const result = await post({ type: 'go_to', x: world.x, y: world.y });
    if (result.ok) {
      appendEvent('NAV_GO', `x=${world.x.toFixed(2)} y=${world.y.toFixed(2)}`, 'ok');
    }
  });
}

function initJointsGrid() {
  const grid = $('joints-grid');
  if (!grid) return;
  grid.innerHTML = JOINT_NAMES.map((name, i) => `
    <div class="motor-row" id="mi-${i}">
      <span class="stale" id="ms-${i}" style="color:#ef4444">*</span>
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
  latestConnected = connected;
  if (!connected) {
    $('stage').textContent = 'DISCONNECTED';
    $('stage').className = 'stage DISCONNECTED';
    updateTaskButtonState();
    return;
  }

  const mode = data.mode || 'UNKNOWN';
  if (mode !== currentMode) highlightMode(mode);
  updateTaskButtonState();

  const rt = data.runtime || {};
  const src = rt.target_source || '--';
  const srcCls = src === 'safety_brake' ? 'bad' : src === 'runtime_policy' ? 'ok' : src === 'runtime_zero_hold' ? 'active' : '';
  setText('d-source', src, srcCls);
  setText('d-released', String(rt.runtime_released ?? '--'), rt.runtime_released ? 'ok' : '');
  setText('d-alpha', (rt.release_alpha ?? '--') !== '--' ? Number(rt.release_alpha).toFixed(2) : '--');
  setText('d-zero', String(rt.zero_command ?? '--'));
  setText('d-estop', String(data.estop ?? '--'), data.estop ? 'bad' : 'ok');
  setText('d-mux', data.mux_status || '--');

  const model = data.model || {};
  latestModelState = {
    current_model: model.current_model || '--',
    switch_state: model.switch_state || '--',
    backend: model.backend || '--',
    switching: Boolean(model.switching),
  };
  setText('d-model', latestModelState.current_model, latestModelState.current_model === 'crawl' ? 'active' : 'ok');
  setText('d-model-switch', latestModelState.switch_state, latestModelState.switching ? 'warn' : 'ok');
  setText('d-model-backend', latestModelState.backend || '--');
  setText('d-model-switching', String(latestModelState.switching), latestModelState.switching ? 'warn' : 'ok');

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
  syncGoalMarkerFromNavStatus(data.nav_status || '');

  const nav = data.nav || {};
  if (nav.odom_fallback) {
    odomFallbackInfo = nav.odom_fallback;
    setText(
      'd-odom-fallback',
      formatOdomFallback(odomFallbackInfo),
      odomFallbackInfo.handoff_pending ? 'warn' : odomFallbackInfo.active ? 'active' : '',
    );
  }
  if (nav.pose) {
    latestPose = nav.pose;
    setText('d-nav-pose', `x=${nav.pose.x.toFixed(2)} y=${nav.pose.y.toFixed(2)} yaw=${(nav.pose.yaw * 57.2958).toFixed(1)}deg`);
  }
  if (nav.path) activeNavPath = nav.path;
  drawMap();

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

async function setMode(mode) {
  if (mode === 'WEB') {
    const ok = confirm('Confirm switch to WEB control?\nMake sure the robot is safe and velocity is zero.');
    if (!ok) return false;
  }
  const result = await post({ type: 'mode', mode });
  if (!result.ok) return false;
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

$('btn-disabled').onclick = async () => {
  zeroAll();
  await setMode('DISABLED');
};
$('btn-remote').onclick = () => setMode('REMOTE');
$('btn-web').onclick = () => setMode('WEB');
$('btn-nav').onclick = () => setMode('NAV');
$('btn-zero').onclick = zeroAll;
$('btn-estop').onclick = async () => {
  if (confirm('Confirm soft e-stop?')) {
    const result = await post({ type: 'estop', data: true });
    zeroAll();
    if (result.ok) appendEvent('ESTOP', 'soft e-stop triggered', 'bad');
  }
};
$('btn-refresh-map').onclick = fetchMap;
$('btn-odom-task').onclick = async () => {
  if (!latestConnected) {
    appendEvent('ODOM', 'bridge not connected', 'warn');
    updateTaskButtonState();
    return;
  }
  if (!defaultMissionName || !defaultMissionGoals.length) {
    appendEvent('ODOM', 'no route mission loaded', 'warn');
    return;
  }
  const start = defaultMissionGoals[0];
  const ok = confirm(
    `Run odom fallback mission ${defaultMissionName}?\n\n` +
    `Place the robot at waypoint id1 before confirming:\n` +
    `x=${Number(start.x).toFixed(3)} y=${Number(start.y).toFixed(3)} yaw=${Number(start.yaw_deg || 0).toFixed(1)}deg`
  );
  if (!ok) return;
  setLastGoalByName(start.name);
  drawMap();
  const result = await post({ type: 'odom_task' });
  if (result.ok) {
    highlightMode('NAV');
    appendEvent('ODOM', `init at ${start.name}, run ${defaultMissionName}`, 'ok');
    setTimeout(fetchMap, 400);
  }
};
$('btn-odom-stop').onclick = async () => {
  if (!latestConnected) {
    appendEvent('ODOM', 'bridge not connected', 'warn');
    updateTaskButtonState();
    return;
  }
  const result = await post({ type: 'odom_stop' });
  if (result.ok) {
    odomFallbackInfo = { active: false };
    setText('d-odom-fallback', formatOdomFallback(odomFallbackInfo), '');
    updateTaskButtonState();
    appendEvent('ODOM', 'fallback stopped', 'ok');
    setTimeout(fetchMap, 400);
  }
};
$('btn-run-task').onclick = async () => {
  if (!latestConnected) {
    appendEvent('TASK', 'bridge not connected', 'warn');
    updateTaskButtonState();
    return;
  }
  if (currentMode !== 'NAV') {
    appendEvent('TASK', 'only available in NAV mode', 'warn');
    updateTaskButtonState();
    return;
  }
  if (!defaultMissionName) {
    appendEvent('TASK', 'no default mission loaded', 'warn');
    return;
  }
  if (defaultMissionGoals.length) {
    setLastGoalByName(defaultMissionGoals[0].name);
    drawMap();
  }
  const result = await post({ type: 'nav_cmd', command: `run ${defaultMissionName}` });
  if (result.ok) {
    appendEvent('TASK', `run mission ${defaultMissionName}`, 'ok');
  }
};
$('btn-record').onclick = () => ensureMode('NAV').then(async ok => {
  if (!ok) return;
  const result = await post({ type: 'nav_cmd', command: 'record web_p1' });
  if (result.ok) appendEvent('NAV_RECORD', 'record web_p1', 'ok');
});
$('btn-stop-nav').onclick = () => ensureMode('NAV').then(async ok => {
  if (!ok) return;
  const result = await post({ type: 'nav_cmd', command: 'stop' });
  if (result.ok) appendEvent('NAV_STOP', 'simple_nav stop', 'ok');
});
$('btn-go-rel').onclick = () => ensureMode('NAV').then(async ok => {
  if (!ok) return;
  const result = await post({ type: 'go_rel', dx: 0.3, dy: 0.0 });
  if (result.ok) appendEvent('NAV_REL', 'forward 0.3m', 'ok');
});
$('btn-model-switch').onclick = async () => {
  if (latestModelState.switching) {
    appendEvent('MODEL', 'switch already in progress', 'warn');
    return;
  }
  const result = await post({ type: 'model_toggle' });
  if (result.ok) appendEvent('MODEL', `toggle requested from ${latestModelState.current_model}`, 'ok');
};

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
