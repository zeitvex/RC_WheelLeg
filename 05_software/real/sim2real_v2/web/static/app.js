const SIM_JOINT_ORDER = [
  ["fl", "hip_abduction"], ["fl", "hip_pitch"], ["fl", "knee"],
  ["fr", "hip_abduction"], ["fr", "hip_pitch"], ["fr", "knee"],
  ["rl", "hip_abduction"], ["rl", "hip_pitch"], ["rl", "knee"],
  ["rr", "hip_abduction"], ["rr", "hip_pitch"], ["rr", "knee"],
  ["fl", "wheel"], ["fr", "wheel"], ["rl", "wheel"], ["rr", "wheel"],
];

const $ = (id) => document.getElementById(id);
const PLOTS = {};
let CURRENT_STATUS = null;
let SSE_CONN = null;
let SSE_RECONNECT_TIMER = null;
let LAST_RENDER_TS = 0;

async function api(path, body = null) {
  const options = { method: body ? "POST" : "GET" };
  if (body) {
    options.headers = { "Content-Type": "application/json" };
    options.body = JSON.stringify(body);
  }
  const response = await fetch(path, options);
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(payload.error || `HTTP ${response.status}`);
  }
  return payload;
}

function safeText(value, fallback = "--") {
  return value === undefined || value === null || Number.isNaN(value) ? fallback : value;
}

function appendEvent(ev) {
  const el = $("events-log");
  if (!el) return;
  const item = document.createElement("div");
  let cls = "ev-name";
  if (/ERROR|STOP|NAN/.test(ev.kind || "")) cls = "ev-stop";
  else if (/FAULT|BRAKE/.test(ev.kind || "")) cls = "ev-fault";
  else if (/DONE|CONNECTED|ENABLED|PRIMED/.test(ev.kind || "")) cls = "ev-ok";
  const t = ev.t ? new Date(ev.t * 1000).toLocaleTimeString() : new Date().toLocaleTimeString();
  const detail = Object.entries(ev)
    .filter(([k]) => !["t", "kind"].includes(k))
    .slice(0, 6)
    .map(([k, v]) => `${k}=${typeof v === "number" ? v.toFixed(3) : JSON.stringify(v).slice(0, 80)}`)
    .join(" ");
  item.innerHTML = `<span class="ev-t">${t}</span> <span class="${cls}">${ev.kind}</span> <span style="color:#8e8e93">${detail}</span>`;
  el.appendChild(item);
  while (el.children.length > 300) el.removeChild(el.firstChild);
  el.scrollTop = el.scrollHeight;
}

function setStage(stage, detail) {
  const el = $("stage");
  if (!el) return;
  el.textContent = stage + (detail ? ` · ${detail}` : "");
  el.className = "stage " + stage;
}

function setButtonEnabled(id, enabled) {
  const el = $(id);
  if (!el) return;
  el.disabled = !enabled;
}

function updateButtons(status) {
  if (!status) return;
  const stage = status.stage || "DISCONNECTED";
  const busy = !!status.busy;
  const runtime = stage === "RUNTIME";
  const connected = stage !== "DISCONNECTED" && stage !== "CONNECTING";
  const enabled = ["ENABLED", "STARTING_UP", "STAND_HOLD", "RUNTIME"].includes(stage);
  const canStartup = stage === "ENABLED";
  const canRuntimeStart = stage === "STAND_HOLD";
  const canRuntimeStop = runtime;
  const remoteAllowed = !!status.remote_takeover_allowed;
  const remoteActive = !!status.remote_takeover_active;
  const runtimeUiBusy = busy && !runtime;

  setButtonEnabled("btn-connect", !runtimeUiBusy && stage === "DISCONNECTED");
  setButtonEnabled("btn-disconnect", !runtimeUiBusy && connected);
  setButtonEnabled("btn-enable", !runtimeUiBusy && ["CONNECTED", "FAULTED"].includes(stage));
  setButtonEnabled("btn-disable", !runtimeUiBusy && enabled);
  setButtonEnabled("btn-startup", !runtimeUiBusy && canStartup);
  setButtonEnabled("btn-runtime-start", !runtimeUiBusy && canRuntimeStart);
  setButtonEnabled("btn-runtime-stop", canRuntimeStop);
  setButtonEnabled("btn-reset-estop", !runtimeUiBusy && stage === "ESTOPPED");
  setButtonEnabled("btn-estop", connected);
  setButtonEnabled("btn-remote-release", runtime && remoteAllowed && !remoteActive);
  setButtonEnabled("btn-remote-reclaim", runtime && remoteActive);
}

function renderState(state) {
  const el = $("state-summary");
  if (!el) return;
  if (!state) {
    el.innerHTML = '<div class="state-item"><span class="k">STATUS</span><span class="v">NO DATA</span></div>';
    return;
  }
  const metric = (k, v, cls = "") =>
    `<div class="state-item"><span class="k">${k}</span><span class="v ${cls}">${v}</span></div>`;
  const safetyText = ["NORMAL", "CLIP", "BRAKE", "ESTOP"][state.safety_level || 0];
  const guardText = ["NORMAL", "WARN", "STOP"][state.guard_level || 0] || "NORMAL";
  const imuCls = (state.imu_age_ms || 0) > 60 ? "bad" : (state.imu_age_ms || 0) > 30 ? "warn" : "";
  const dtCls = (state.loop_dt_ms || 0) > 25 ? "bad" : (state.loop_dt_ms || 0) > 22 ? "warn" : "";
  const gravityZ = state.proj_gravity?.[2] ?? -1;
  const gravityCls = gravityZ > -0.5 ? "warn" : "";
  const rawMax = Math.max(...(state.raw || [0]).map((x) => Math.abs(x || 0)));
  const odom = state.odom || null;
  const odomLocal = odom?.local_pos || odom?.pos || [0, 0, 0];
  const odomText = odom ? `${odom.type || "ODOM"} ${(odom.age_ms || 0).toFixed(0)}ms ${odomLocal[0].toFixed(2)},${odomLocal[1].toFixed(2)}` : "no odom";
  const cmd = state.cmd || [0, 0, 0];
  const rawCmd = state.raw_cmd || cmd;
  const stand = state.stand_balance || CURRENT_STATUS?.diagnostics?.stand_balance || {};
  const targetInfo = state.latest_target || CURRENT_STATUS?.diagnostics?.latest_target || {};
  const trackingErr = Math.max(
    ...(state.joint_pos || []).slice(0, 12).map((pos, i) => Math.abs(pos - ((state.target || [])[i] || 0))),
    0,
  );
  const inputModeEl = $("input-mode");
  if (inputModeEl) inputModeEl.textContent = CURRENT_STATUS?.input_mode || "WEB";
  const remoteAllowedEl = $("remote-allowed");
  if (remoteAllowedEl) remoteAllowedEl.textContent = CURRENT_STATUS?.remote_takeover_allowed ? "YES" : "NO";
  const remoteEstopEl = $("remote-estop");
  if (remoteEstopEl) remoteEstopEl.textContent = CURRENT_STATUS?.remote_soft_estop ? "ON" : "OFF";
  const remotePortEl = $("remote-port");
  if (remotePortEl) remotePortEl.textContent = CURRENT_STATUS?.remote_status?.port || "--";
  el.innerHTML = [
    metric("phase", safeText(state.phase, "?")),
    metric("cmd", `${cmd.map((x) => (x || 0).toFixed(2)).join(",")}`),
    metric("raw cmd", `${rawCmd.map((x) => (x || 0).toFixed(2)).join(",")}`),
    metric("imu_age", `${(state.imu_age_ms || 0).toFixed(1)} ms`, imuCls),
    metric("loop_dt", `${(state.loop_dt_ms || 0).toFixed(1)} ms`, dtCls),
    metric("safety", safetyText, state.safety_level >= 2 ? "bad" : state.safety_level === 1 ? "warn" : ""),
    metric("guard", guardText, state.guard_level >= 2 ? "bad" : state.guard_level === 1 ? "warn" : ""),
    metric("holdover", String(state.holdover_total || 0)),
    metric("raw max", rawMax.toFixed(2)),
    metric("grav_z", gravityZ.toFixed(3), gravityCls),
    metric("odom", odomText, odom && (odom.age_ms || 0) < 200 && !odom.jump_detected ? "" : "warn"),
    metric("track_err", trackingErr.toFixed(3), trackingErr > 0.5 ? "bad" : trackingErr > 0.2 ? "warn" : ""),
    metric("stand", stand.enabled ? `p ${((stand.pitch_deg) || 0).toFixed(1)} r ${((stand.roll_deg) || 0).toFixed(1)}` : "off", stand.enabled && !stand.pitch_compensation_enabled ? "warn" : ""),
    metric("pitch_corr", `${((stand.pitch_corr) || 0).toFixed(3)} ${stand.pitch_compensation_enabled ? "on" : "off"}`),
    metric("target_src", targetInfo.source ? `${targetInfo.source} ${(targetInfo.age_ms || 0).toFixed(0)}ms` : "--", targetInfo.age_ms > 100 ? "warn" : ""),
  ].join("");
}

function renderDiagnostics(diag, state) {
  if (!diag) return;
  const setValue = (id, text, cls = "") => {
    const el = $(id);
    if (!el) return;
    el.textContent = text;
    el.className = "diag-value " + cls;
  };
  setValue("diag-norm", "Aligned", "success");
  setValue("diag-latency", `${(state?.loop_dt_ms || 0).toFixed(1)} ms`, (state?.loop_dt_ms || 0) > 25 ? "danger" : (state?.loop_dt_ms || 0) > 22 ? "warning" : "success");
  const trackErr = Math.max(
    ...(state?.joint_pos || []).slice(0, 12).map((pos, i) => Math.abs(pos - ((state?.target || [])[i] || 0))),
    0,
  );
  setValue("diag-track-err", `${trackErr.toFixed(3)} rad`, trackErr > 0.5 ? "danger" : trackErr > 0.2 ? "warning" : "success");
  setValue("diag-runtime", diag.runtime_active ? "ACTIVE" : "IDLE", diag.runtime_active ? "success" : "warning");
  setValue("diag-runtime-age", diag.last_runtime_age_s == null ? "--" : `${diag.last_runtime_age_s.toFixed(2)} s`, diag.last_runtime_age_s != null && diag.last_runtime_age_s > 1.0 ? "danger" : "success");
  setValue("diag-poll-age", diag.last_poll_age_s == null ? "--" : `${diag.last_poll_age_s.toFixed(2)} s`, diag.last_poll_age_s != null && diag.last_poll_age_s > 1.0 ? "warning" : "success");
  setValue("diag-cmd-age", diag.last_command_age_s == null ? "--" : `${diag.last_command_age_s.toFixed(2)} s`);
  setValue("diag-poll-errors", String(diag.poll_error_count || 0), (diag.poll_error_count || 0) > 0 ? "danger" : "success");
  setValue("diag-api-errors", String(diag.api_error_count || 0), (diag.api_error_count || 0) > 0 ? "warning" : "success");
  setValue("diag-overruns", `${diag.runtime_overrun_count || 0} / max ${(diag.runtime_overrun_max_ms || 0).toFixed(1)} ms`, (diag.runtime_overrun_count || 0) > 0 ? "warning" : "success");
  setValue("diag-policy-stale", `${diag.runtime_policy_stale_count || 0} / max ${(diag.runtime_policy_stale_max_ms || 0).toFixed(1)} ms`, (diag.runtime_policy_stale_count || 0) > 0 ? "warning" : "success");
  const profile = state?.loop_profile || diag.last_loop_profile || {};
  const slowest = Object.entries(profile)
    .filter(([k]) => k !== "total_ms")
    .sort((a, b) => (b[1] || 0) - (a[1] || 0))[0];
  const totalMs = profile.total_ms ?? state?.loop_dt_ms ?? 0;
  setValue("diag-loop-profile", slowest ? `${totalMs.toFixed(1)} ms, slow ${slowest[0]}=${(slowest[1] || 0).toFixed(1)}` : "--");
  const targetInfo = state?.latest_target || diag.latest_target || {};
  if (targetInfo.source) {
    setValue("diag-target", `${targetInfo.source} age ${(targetInfo.age_ms || 0).toFixed(0)}ms d ${(targetInfo.delta_max || 0).toFixed(3)}`, (targetInfo.age_ms || 0) > 100 ? "warning" : "success");
  } else {
    setValue("diag-target", "--");
  }
  const stand = state?.stand_balance || diag.stand_balance || {};
  if (stand.enabled) {
    setValue(
      "diag-stand",
      `pitch ${((stand.pitch_deg) || 0).toFixed(2)}deg corr ${((stand.pitch_corr) || 0).toFixed(3)} ${stand.pitch_compensation_enabled ? "pitch-on" : "pitch-off"}`,
      stand.pitch_compensation_enabled ? "warning" : "success",
    );
  } else {
    setValue("diag-stand", "disabled", "warning");
  }
  const obsAbs = state?.obs_abs_max;
  const rawAbs = state?.raw_abs_max;
  const scaledAbs = state?.scaled_abs_max;
  const signalText = obsAbs == null ? "--" : `obs ${obsAbs.toFixed(2)} raw ${(rawAbs || 0).toFixed(2)} scaled ${(scaledAbs || 0).toFixed(2)}`;
  setValue("diag-signal", signalText, obsAbs > 80 || rawAbs > 9.5 ? "warning" : "success");
  const freshCount = state?.motor_fresh_count;
  const byCount = state?.motor_fresh_by_update_count;
  const byValue = state?.motor_fresh_by_value_change;
  setValue("diag-motor-fresh", freshCount == null ? "--" : `${freshCount}/16 (cnt ${byCount || 0}, val ${byValue || 0})`, freshCount === 16 ? "success" : freshCount >= 12 ? "warning" : "danger");
  const odom = state?.odom;
  if (odom) {
    const p = odom.local_pos || odom.pos || [0, 0, 0];
    const yaw = odom.local_yaw == null ? 0 : odom.local_yaw;
    const jump = odom.jump_detected ? " JUMP" : "";
    setValue("diag-odom", `${odom.type || "ODOM"} ${((odom.age_ms || 0)).toFixed(0)}ms x=${p[0].toFixed(2)} y=${p[1].toFixed(2)} yaw=${yaw.toFixed(2)}${jump}`, (odom.age_ms || 0) > 500 || odom.jump_detected ? "warning" : "success");
  } else {
    setValue("diag-odom", "not available", "warning");
  }
  setValue("diag-suppression", String(diag.zero_cmd_suppression), diag.zero_cmd_suppression ? "warning" : "success");
  const pathEl = $("diag-policy");
  if (pathEl) pathEl.textContent = diag.policy_path || "--";
}

function renderFault(status) {
  const faultBox = $("fault-box");
  const faultText = $("fault-text");
  const traceText = $("traceback-text");
  if (!faultBox || !faultText || !traceText) return;
  if (!status.fault_reason && !status.last_error) {
    faultBox.classList.add("hidden");
    faultText.textContent = "";
    traceText.textContent = "";
    return;
  }
  faultBox.classList.remove("hidden");
  faultText.textContent = status.fault_reason || status.last_error || "";
  traceText.textContent = status.last_traceback || "";
}

function applyStatus(status) {
  if (!status) return;
  CURRENT_STATUS = { ...(CURRENT_STATUS || {}), ...status };
  const merged = CURRENT_STATUS;
  if (merged.stage) setStage(merged.stage, merged.detail || "");
  if (merged.busy !== undefined && $("busy")) $("busy").textContent = merged.busy ? " [BUSY]" : "";
  if (merged.log_dir && $("logdir")) $("logdir").textContent = merged.log_dir;
  updateButtons(merged);
  renderFault(merged);
  if (merged.last_state !== undefined) {
    const now = performance.now();
    if (now - LAST_RENDER_TS > 80) {
      renderState(merged.last_state);
      renderDiagnostics(merged.diagnostics || {}, merged.last_state);
      if (window.viewer3d && window.viewer3d._isLoaded && merged.last_state.joint_pos) {
        window.viewer3d.updateJoints(merged.last_state.joint_pos);
      }
      updateMotorsGrid(merged.last_state);
      addPlotData(merged.last_state);
      LAST_RENDER_TS = now;
    }
  }
}

async function refreshDebug() {
  try {
    const debug = await api("/api/debug");
    if (debug.status) {
      applyStatus(debug.status);
    }
    renderDiagnostics(debug.status?.diagnostics || {}, debug.status?.last_state || null);
    renderFault(debug.status || {});
    const diagJson = $("debug-json");
    if (diagJson) diagJson.textContent = JSON.stringify(debug.status?.diagnostics || {}, null, 2);
  } catch (err) {
    appendEvent({ kind: "DEBUG_FETCH_ERROR", error: err.message });
  }
}

function connectSSE() {
  if (SSE_CONN) {
    SSE_CONN.close();
    SSE_CONN = null;
  }
  if (SSE_RECONNECT_TIMER) {
    clearTimeout(SSE_RECONNECT_TIMER);
    SSE_RECONNECT_TIMER = null;
  }
  const es = new EventSource("/events");
  SSE_CONN = es;
  es.onmessage = (event) => {
    const ev = JSON.parse(event.data);
    if (ev.kind === "STATUS_FULL" || ev.kind === "PULSE" || ev.kind === "STATUS") {
      applyStatus(ev);
      if (ev.fault_reason) appendEvent({ t: ev.t, kind: "FAULT_REASON", reason: ev.fault_reason });
    } else {
      appendEvent(ev);
    }
  };
  es.onerror = () => {
    if (SSE_CONN) {
      SSE_CONN.close();
      SSE_CONN = null;
    }
    if (!SSE_RECONNECT_TIMER) {
      SSE_RECONNECT_TIMER = setTimeout(() => {
        SSE_RECONNECT_TIMER = null;
        connectSSE();
      }, 1500);
    }
  };
}

window.jog = async (leg, joint, dir) => {
  const delta = parseFloat($("jt-delta").value) * dir;
  const kp = parseFloat($("jt-kp").value);
  const kd = parseFloat($("jt-kd").value);
  const duration = parseFloat($("jt-dur").value);
  try {
    await api("/api/test_motor", { leg, joint, delta_rad: delta, kp, kd, duration_s: duration });
    appendEvent({ kind: "JOG_SENT", leg, joint, delta });
  } catch (err) {
    appendEvent({ kind: "JOG_ERROR", error: err.message, leg, joint });
  }
};

function initMotorsGrid() {
  const grid = $("motors-grid");
  if (!grid) return;
  const abbr = { hip_abduction: "H_ABD", hip_pitch: "H_PIT", knee: "KNEE", wheel: "WHEEL" };
  grid.innerHTML = SIM_JOINT_ORDER.map(([leg, joint], i) => `
    <div class="motor-row" id="mi-${i}">
      <span class="m-status" id="ms-${i}" title="offline">●</span>
      <span class="name" title="${leg}_${joint}">${leg.toUpperCase()}_${abbr[joint]}</span>
      <span class="val pos">0.00</span>
      <span class="val vel">0.00</span>
      <span class="val tau">0.00</span>
      <span class="val temp">0°C</span>
      <span class="val fault">OK</span>
      <div class="m-jog">
        <button class="btn-jog" onclick="window.jog('${leg}','${joint}',-1)">-</button>
        <button class="btn-jog" onclick="window.jog('${leg}','${joint}',1)">+</button>
      </div>
    </div>
  `).join("");
}

function updateMotorsGrid(state) {
  if (!state || !state.joint_pos) return;
  const positions = state.joint_pos;
  const velocities = state.joint_vel || [];
  const torques = state.joint_torque || [];
  const stale = state.per_motor_stale || [];
  const temps = state.motor_temperatures || [];
  const faults = state.motor_fault_codes || [];
  for (let i = 0; i < 16; i += 1) {
    const row = $("mi-" + i);
    if (!row) continue;
    const dot = $("ms-" + i);
    if (dot) {
      const count = stale[i] ?? 99;
      if (count <= 0) {
        dot.style.color = "#4ade80";
        dot.title = "online";
      } else if (count < 5) {
        dot.style.color = "#facc15";
        dot.title = `stale(${count})`;
      } else {
        dot.style.color = "#ef4444";
        dot.title = `offline(${count})`;
      }
    }
    row.children[2].textContent = (positions[i] || 0).toFixed(2);
    row.children[3].textContent = (velocities[i] || 0).toFixed(2);
    const tau = torques[i] || 0;
    row.children[4].textContent = tau.toFixed(2);
    row.children[4].style.color = Math.abs(tau) > 16.0 ? "var(--color-danger)" : "";
    row.children[4].style.fontWeight = Math.abs(tau) > 16.0 ? "bold" : "";

    const temp = temps[i] ?? 0.0;
    row.children[5].textContent = temp.toFixed(0) + "°C";
    row.children[5].style.color = temp > 60.0 ? "#ff453a" : temp > 45.0 ? "#ffd60a" : "";
    row.children[5].style.fontWeight = temp > 45.0 ? "bold" : "";

    const fault = faults[i] ?? 0;
    if (fault === 0) {
      row.children[6].textContent = "OK";
      row.children[6].style.color = "#30d158";
      row.children[6].style.fontWeight = "";
    } else {
      row.children[6].textContent = "E" + fault.toString(16).toUpperCase();
      row.children[6].style.color = "#ff453a";
      row.children[6].style.fontWeight = "bold";
    }
  }
}

function initPlots() {
  const colors12 = ["#ff453a", "#ff9f0a", "#ffd60a", "#32ade6", "#0a84ff", "#5e5ce6", "#ff375f", "#bf5af2", "#30d158", "#66d4cf", "#8e8e93", "#c7c7cc"];
  const specs = [
    { id: "plot-pos", title: "Leg Pos (12)", nCh: 12, colors: colors12 },
    { id: "plot-vel", title: "Wheel Vel (4)", nCh: 4, colors: ["#ff453a", "#32ade6", "#30d158", "#ffd60a"] },
    { id: "plot-imu", title: "IMU (gyro+gz)", nCh: 4, colors: ["#ff453a", "#30d158", "#0a84ff", "#ffd60a"] },
    { id: "plot-diag", title: "Diag (dt+age)", nCh: 2, colors: ["#ff453a", "#30d158"] },
  ];
  const maxPts = 150;
  specs.forEach((spec) => {
    const canvas = $(spec.id);
    if (!canvas) return;
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = 80;
    PLOTS[spec.id] = {
      ctx: canvas.getContext("2d"),
      title: spec.title,
      nCh: spec.nCh,
      colors: spec.colors,
      data: Array.from({ length: spec.nCh }, () => new Array(maxPts).fill(0)),
      yMin: Array(spec.nCh).fill(Infinity),
      yMax: Array(spec.nCh).fill(-Infinity),
      maxPts,
    };
  });
}

function addPlotData(state) {
  if (!state) return;
  const channels = [
    ["plot-pos", (state.joint_pos || []).slice(0, 12)],
    ["plot-vel", (state.joint_vel || []).slice(12, 16)],
    ["plot-imu", [...(state.gyro || [0, 0, 0]), (state.proj_gravity || [0, 0, -1])[2]]],
    ["plot-diag", [state.loop_dt_ms || 0, state.imu_age_ms || 0]],
  ];
  channels.forEach(([id, values]) => {
    const plot = PLOTS[id];
    if (!plot) return;
    for (let i = 0; i < plot.nCh && i < values.length; i += 1) {
      const data = plot.data[i];
      data.push(values[i]);
      if (data.length > plot.maxPts) data.shift();
      if (values[i] < plot.yMin[i]) plot.yMin[i] = values[i];
      if (values[i] > plot.yMax[i]) plot.yMax[i] = values[i];
    }
    drawPlot(plot);
  });
}

function drawPlot(plot) {
  const { ctx, data, colors, yMin, yMax, title, maxPts } = plot;
  const canvas = ctx.canvas;
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "rgba(255,255,255,0.5)";
  ctx.font = "10px monospace";
  ctx.fillText(title, 4, 12);
  const margin = { l: 30, r: 4, t: 16, b: 4 };
  const plotW = width - margin.l - margin.r;
  const plotH = height - margin.t - margin.b;
  if (plotW <= 0 || plotH <= 0) return;
  for (let i = 0; i < data.length; i += 1) {
    if (yMin[i] === Infinity) {
      yMin[i] = -1;
      yMax[i] = 1;
    }
    const curMin = Math.min(...data[i]);
    const curMax = Math.max(...data[i]);
    yMin[i] = yMin[i] * 0.99 + curMin * 0.01;
    yMax[i] = yMax[i] * 0.99 + curMax * 0.01;
  }
  const globalMin = Math.min(...yMin);
  const globalMax = Math.max(...yMax);
  const range = globalMax - globalMin || 1;
  data.forEach((series, i) => {
    if (series.length < 2) return;
    ctx.strokeStyle = colors[i] || "#8e8e93";
    ctx.lineWidth = 1.0;
    ctx.beginPath();
    series.forEach((value, j) => {
      const x = margin.l + (j / maxPts) * plotW;
      const y = margin.t + plotH - ((value - globalMin) / range) * plotH;
      if (j === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
  ctx.fillStyle = "rgba(255,255,255,0.4)";
  ctx.font = "9px monospace";
  ctx.fillText(globalMax.toFixed(1), 2, margin.t + 8);
  ctx.fillText(globalMin.toFixed(1), 2, margin.t + plotH - 2);
}

async function refreshLogs() {
  try {
    const result = await api("/api/logs");
    const tbody = document.querySelector("#logs-table tbody");
    if (!tbody) return;
    tbody.innerHTML = result.sessions.map((s) => `
      <tr>
        <td style="font-family:monospace">${s.id.slice(-8)}</td>
        <td>${s.state_csv ? `<a href="/api/logs/${s.id}/state.csv" download>CSV</a>` : "—"}</td>
        <td>${s.events_jsonl ? `<a href="/api/logs/${s.id}/events.jsonl" download>JSONL</a>` : "—"}</td>
        <td>${s.size_kb} KB</td>
      </tr>
    `).join("");
  } catch (err) {
    appendEvent({ kind: "LOG_REFRESH_ERROR", error: err.message });
  }
}

let cmdTimer = null;
function sendCmd() {
  if (cmdTimer) return;
  cmdTimer = setTimeout(() => {
    cmdTimer = null;
    api("/api/cmd", {
      vx: parseFloat($("cmd-vx").value),
      vy: parseFloat($("cmd-vy").value),
      yaw: parseFloat($("cmd-yaw").value),
    }).catch((err) => appendEvent({ kind: "CMD_ERROR", error: err.message }));
  }, 50);
}

function bind() {
  $("btn-connect").onclick = () => api("/api/connect", { dry_run: $("dry-run").checked }).catch((err) => appendEvent({ kind: "CONNECT_ERROR", error: err.message }));
  $("btn-disconnect").onclick = () => api("/api/disconnect", {}).catch((err) => appendEvent({ kind: "DISCONNECT_ERROR", error: err.message }));
  $("btn-enable").onclick = () => api("/api/enable", {}).catch((err) => appendEvent({ kind: "ENABLE_ERROR", error: err.message }));
  $("btn-disable").onclick = () => api("/api/disable", {}).catch((err) => appendEvent({ kind: "DISABLE_ERROR", error: err.message }));
  $("btn-startup").onclick = () => api("/api/startup", {}).catch((err) => appendEvent({ kind: "STARTUP_ERROR", error: err.message }));
  $("btn-runtime-start").onclick = () => api("/api/runtime/start", { policy_path: $("policy-path").value || null }).catch((err) => appendEvent({ kind: "RUNTIME_START_ERROR", error: err.message }));
  $("btn-runtime-stop").onclick = () => api("/api/runtime/stop", {}).catch((err) => appendEvent({ kind: "RUNTIME_STOP_ERROR", error: err.message }));
  $("btn-remote-release").onclick = () => api("/api/remote_takeover", { enabled: true }).catch((err) => appendEvent({ kind: "REMOTE_TAKEOVER_ENABLE_ERROR", error: err.message }));
  $("btn-remote-reclaim").onclick = () => api("/api/remote_takeover", { enabled: false }).catch((err) => appendEvent({ kind: "REMOTE_TAKEOVER_DISABLE_ERROR", error: err.message }));
  $("btn-estop").onclick = () => api("/api/estop", {}).catch((err) => appendEvent({ kind: "ESTOP_ERROR", error: err.message }));
  $("btn-reset-estop").onclick = () => api("/api/reset_estop", {}).catch((err) => appendEvent({ kind: "RESET_ESTOP_ERROR", error: err.message }));
  $("btn-refresh-debug").onclick = () => refreshDebug();

  ["vx", "vy", "yaw"].forEach((key) => {
    const el = $("cmd-" + key);
    el.oninput = () => {
      $("cmd-" + key + "-v").textContent = parseFloat(el.value).toFixed(2);
      sendCmd();
    };
  });
  $("btn-cmd-zero").onclick = () => {
    ["vx", "vy", "yaw"].forEach((key) => {
      const el = $("cmd-" + key);
      el.value = 0;
      $("cmd-" + key + "-v").textContent = "0.00";
    });
    sendCmd();
  };

  const jtSlider = $("jt-delta");
  jtSlider.oninput = () => { $("jt-delta-v").textContent = parseFloat(jtSlider.value).toFixed(2); };

  $("btn-show-logs").onclick = () => {
    refreshLogs();
    $("logs-modal").classList.remove("hidden");
  };
  $("btn-close-logs").onclick = () => $("logs-modal").classList.add("hidden");
}

window.addEventListener("DOMContentLoaded", () => {
  initMotorsGrid();
  bind();
  initPlots();
  connectSSE();
  refreshLogs();
  refreshDebug();
  updateButtons({ stage: "DISCONNECTED", busy: false });
  setInterval(refreshLogs, 10000);
  setInterval(refreshDebug, 5000);
});

window.addEventListener("resize", () => {
  Object.values(PLOTS).forEach((plot) => {
    plot.ctx.canvas.width = plot.ctx.canvas.parentElement.clientWidth;
  });
});
