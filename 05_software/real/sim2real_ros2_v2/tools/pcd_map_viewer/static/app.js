import * as THREE from "/vendor/three.module.js";
import { OrbitControls } from "/vendor/jsm/controls/OrbitControls.js";

const mapSelect = document.getElementById("mapSelect");
const layerSelect = document.getElementById("layerSelect");
const viewSelect = document.getElementById("viewSelect");
const cloudModeSelect = document.getElementById("cloudModeSelect");
const serverState = document.getElementById("serverState");
const cursorInfo = document.getElementById("cursorInfo");
const mainPanel = document.querySelector(".main");
const cloudZMinInput = document.getElementById("cloudZMin");
const cloudZMaxInput = document.getElementById("cloudZMax");
const cloudMaxPointsInput = document.getElementById("cloudMaxPoints");
const voxelSizeInput = document.getElementById("voxelSize");
const voxelMinPointsInput = document.getElementById("voxelMinPoints");
const voxelMinClusterInput = document.getElementById("voxelMinCluster");
const mapCanvas = document.getElementById("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");
const cloudCanvas = document.getElementById("cloudCanvas");

const routeName = document.getElementById("routeName");
const wpX = document.getElementById("wpX");
const wpY = document.getElementById("wpY");
const wpYaw = document.getElementById("wpYaw");
const wpObstacle = document.getElementById("wpObstacle");
const wpSpeed = document.getElementById("wpSpeed");
const wpPolicy = document.getElementById("wpPolicy");
const wpTol = document.getElementById("wpTol");
const wpYawTol = document.getElementById("wpYawTol");
const wpRequireYaw = document.getElementById("wpRequireYaw");
const wpPreDockDist = document.getElementById("wpPreDockDist");
const wpPreDockTol = document.getElementById("wpPreDockTol");
const waypointList = document.getElementById("waypointList");
const obstacleSelect = document.getElementById("obstacleSelect");
const obstacleName = document.getElementById("obstacleName");
const obstacleType = document.getElementById("obstacleType");
const obstacleX = document.getElementById("obstacleX");
const obstacleY = document.getElementById("obstacleY");
const obstacleYaw = document.getElementById("obstacleYaw");
const obstacleLength = document.getElementById("obstacleLength");
const obstacleWidth = document.getElementById("obstacleWidth");
const obstaclePolicy = document.getElementById("obstaclePolicy");
const obstacleList = document.getElementById("obstacleList");
const UI_VERSION = "obstacle-terrain-2";

serverState.textContent = UI_VERSION;

let maps = [];
let currentMap = null;
let currentLayer = null;
let currentInfo = { width: 1, height: 1, resolution: 1, origin: [0, 0, 0] };
let mapImage = new Image();
let scale2d = 1;
let offsetX = 0;
let offsetY = 0;
let dragging2d = false;
let dragLast2d = null;
let moved2d = false;
let selected = -1;
let waypoints = [];
let obstacles = [];
let activeObstacle = -1;
let addingObstacle = false;
let obstacleDraft = null;
let routeSegmentsMeta = [];
let routeSettings = {
  yawToleranceDegDefault: null,
  requireYawDefault: null,
  preDockDistanceDefault: null,
  preDockToleranceDefault: null,
};
let viewMode = "3d";
let cloudZMin = -2.0;
let cloudZMax = 1.0;
let cloudMaxPoints = 120000;
let cloudMode = "points";
let voxelSize = 0.20;
let voxelMinPoints = 1;
let voxelMinCluster = 1;
let cloudStats = null;
let pointerDown3d = null;
let moved3d = false;

const cloud = {
  mode: "points",
  count: 0,
  positions: new Float32Array(),
  bounds: null,
  center: [0, 0, 0],
  voxelSize: 0.20,
};

function defaultPolicyForObstacle(obstacle) {
  return obstacle === "low_bar" ? "crawl" : "rough";
}

function makeDefaultObstacle(index = 0, x = 0, y = 0, length = 1.0, width = 1.0) {
  return {
    name: `obstacle_${index + 1}`,
    obstacle: "rough_pit",
    x: Number(x.toFixed(4)),
    y: Number(y.toFixed(4)),
    yawDeg: 0,
    length: Math.max(0.05, Number(length.toFixed(4))),
    width: Math.max(0.05, Number(width.toFixed(4))),
    policy: "rough",
  };
}

function ensureObstacles() {
  if (!Array.isArray(obstacles)) obstacles = [];
  activeObstacle = obstacles.length
    ? Math.max(0, Math.min(activeObstacle, obstacles.length - 1))
    : -1;
  for (let i = 0; i < obstacles.length; i += 1) {
    const item = obstacles[i];
    if (!item.name) item.name = `obstacle_${i + 1}`;
    if (!item.obstacle) item.obstacle = "rough_pit";
    item.x = Number(item.x || 0);
    item.y = Number(item.y || 0);
    item.yawDeg = Number(item.yawDeg || item.yaw_deg || 0);
    item.length = Math.max(0.05, Number(item.length || 1.0));
    item.width = Math.max(0.05, Number(item.width || 1.0));
    item.policy = item.policy || defaultPolicyForObstacle(item.obstacle);
  }
}

function ensureWaypoints() {
  if (!Array.isArray(waypoints)) waypoints = [];
  for (let i = 0; i < waypoints.length; i += 1) {
    const item = waypoints[i];
    item.yawToleranceDeg = Number(
      item.yawToleranceDeg ?? item.yaw_tolerance_deg ?? 30,
    );
    item.requireYaw = Boolean(item.requireYaw ?? item.require_yaw ?? false);
    const preDockDistance = item.preDockDistance ?? item.pre_dock_distance;
    item.preDockDistance = preDockDistance == null ? null : Number(preDockDistance);
    const preDockTolerance = item.preDockTolerance ?? item.pre_dock_tolerance;
    item.preDockTolerance = preDockTolerance == null ? null : Number(preDockTolerance);
  }
}

function normalizeRouteWaypoints(route) {
  if (Array.isArray(route?.segments) && route.segments.length) {
    const points = [];
    route.segments.forEach((segment) => {
      if (!segment || !Array.isArray(segment.waypoints)) return;
      segment.waypoints.forEach((wp) => {
        if (wp && typeof wp === "object") points.push(wp);
      });
    });
    return points;
  }
  return Array.isArray(route?.waypoints) ? route.waypoints : [];
}

function normalizeRouteSegments(route) {
  if (Array.isArray(route?.segments) && route.segments.length) {
    return route.segments.map((segment, index) => ({
      name: segment?.name || `segment_${index + 1}`,
      obstacle: segment?.obstacle || "flat",
      waypointCount: Array.isArray(segment?.waypoints) ? segment.waypoints.length : 0,
    }));
  }
  const points = normalizeRouteWaypoints(route);
  return points.length ? [{
    name: "segment_1",
    obstacle: "flat",
    waypointCount: points.length,
  }] : [];
}

function applyLoadedRoute(route) {
  routeName.value = route?.name || routeName.value || "test_route";
  waypoints = normalizeRouteWaypoints(route);
  obstacles = route && Array.isArray(route.obstacles) ? route.obstacles : [];
  routeSegmentsMeta = normalizeRouteSegments(route);
  routeSettings = {
    yawToleranceDegDefault: route?.yawToleranceDegDefault ?? route?.yaw_tolerance_deg_default ?? null,
    requireYawDefault: route?.requireYawDefault ?? route?.require_yaw_default ?? null,
    preDockDistanceDefault: route?.preDockDistanceDefault ?? route?.pre_dock_distance_default ?? null,
    preDockToleranceDefault: route?.preDockToleranceDefault ?? route?.pre_dock_tolerance_default ?? null,
  };
  activeObstacle = obstacles.length ? 0 : -1;
  ensureObstacles();
  ensureWaypoints();
  refreshWaypointObstacles(false);
}

function allWaypointRefs() {
  ensureWaypoints();
  return waypoints.map((wp, pointIndex) => ({ pointIndex, wp }));
}

function selectedRef() {
  const refs = allWaypointRefs();
  return selected >= 0 && selected < refs.length ? refs[selected] : null;
}

function obstacleCorners(item) {
  const yaw = (Number(item.yawDeg || 0) * Math.PI) / 180;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const hx = Math.max(0.025, Number(item.length || 1.0) * 0.5);
  const hy = Math.max(0.025, Number(item.width || 1.0) * 0.5);
  return [
    [-hx, -hy],
    [hx, -hy],
    [hx, hy],
    [-hx, hy],
  ].map(([lx, ly]) => ({
    x: item.x + lx * c - ly * s,
    y: item.y + lx * s + ly * c,
  }));
}

function pointInObstacle(x, y, item) {
  const yaw = (-Number(item.yawDeg || 0) * Math.PI) / 180;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const dx = x - Number(item.x || 0);
  const dy = y - Number(item.y || 0);
  const lx = dx * c - dy * s;
  const ly = dx * s + dy * c;
  return Math.abs(lx) <= Number(item.length || 1.0) * 0.5
    && Math.abs(ly) <= Number(item.width || 1.0) * 0.5;
}

function waypointObstacleInfo(x, y) {
  ensureObstacles();
  for (let i = obstacles.length - 1; i >= 0; i -= 1) {
    const item = obstacles[i];
    if (pointInObstacle(x, y, item)) {
      return {
        obstacle: item.obstacle || "flat",
        obstacleName: item.name || item.obstacle || "obstacle",
        policy: item.policy || defaultPolicyForObstacle(item.obstacle),
      };
    }
  }
  return { obstacle: "flat", obstacleName: "", policy: "rough" };
}

function applyObstacleToWaypoint(wp, updatePolicy = false) {
  const info = waypointObstacleInfo(Number(wp.x || 0), Number(wp.y || 0));
  wp.obstacle = info.obstacle;
  wp.obstacleName = info.obstacleName;
  if (updatePolicy) wp.policy = info.policy;
  return wp;
}

function refreshWaypointObstacles(updatePolicy = false) {
  ensureWaypoints();
  waypoints.forEach((wp) => applyObstacleToWaypoint(wp, updatePolicy));
}

function activeObstacleItem() {
  ensureObstacles();
  return activeObstacle >= 0 && activeObstacle < obstacles.length ? obstacles[activeObstacle] : null;
}

const renderer = new THREE.WebGLRenderer({
  canvas: cloudCanvas,
  antialias: true,
  preserveDrawingBuffer: true,
});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setClearColor(0x091116, 1);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x091116);

const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 4000);
camera.up.set(0, 0, 1);
camera.position.set(12, -12, 10);

const controls = new OrbitControls(camera, cloudCanvas);
controls.enableDamping = true;
controls.enablePan = true;
controls.target.set(0, 0, 0);

scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dirLight = new THREE.DirectionalLight(0xffffff, 1.1);
dirLight.position.set(12, -8, 18);
scene.add(dirLight);

const gridHelper = new THREE.GridHelper(30, 30, 0x324a56, 0x20303a);
gridHelper.rotation.x = Math.PI * 0.5;
scene.add(gridHelper);

let pointsObject = null;
let voxelObject = null;
let voxelPickMesh = null;

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const zPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
const tmpVec = new THREE.Vector3();
const tmpBox = new THREE.Box3();
const tmpCenter = new THREE.Vector3();
const tmpSize = new THREE.Vector3();
const tmpMatrix = new THREE.Matrix4();
const tmpPosition = new THREE.Vector3();

function makeAxisArrow(direction, color, length, shaftRadius, headLength, headRadius) {
  const group = new THREE.Group();
  const shaftLength = Math.max(0.01, length - headLength);
  const shaftMaterial = new THREE.MeshStandardMaterial({
    color,
    roughness: 0.35,
    metalness: 0.08,
  });
  const headMaterial = new THREE.MeshStandardMaterial({
    color,
    emissive: color,
    emissiveIntensity: 0.14,
    roughness: 0.25,
    metalness: 0.1,
  });

  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 16),
    shaftMaterial,
  );
  shaft.position.y = shaftLength * 0.5;

  const head = new THREE.Mesh(
    new THREE.ConeGeometry(headRadius, headLength, 20),
    headMaterial,
  );
  head.position.y = shaftLength + headLength * 0.5;

  group.add(shaft);
  group.add(head);
  group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.clone().normalize());
  return group;
}

function makeAxes(length = 2.0) {
  const group = new THREE.Group();
  group.add(makeAxisArrow(new THREE.Vector3(1, 0, 0), 0xff6b6b, length, 0.035, 0.22, 0.09));
  group.add(makeAxisArrow(new THREE.Vector3(0, 1, 0), 0x4ade80, length, 0.035, 0.22, 0.09));
  group.add(makeAxisArrow(new THREE.Vector3(0, 0, 1), 0x38bdf8, length, 0.035, 0.22, 0.09));
  return group;
}

function resize() {
  const rect = mapCanvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  mapCanvas.width = Math.max(1, Math.round(rect.width * dpr));
  mapCanvas.height = Math.max(1, Math.round(rect.height * dpr));
  mapCanvas.style.width = `${rect.width}px`;
  mapCanvas.style.height = `${rect.height}px`;
  mapCtx.setTransform(dpr, 0, 0, dpr, 0, 0);

  renderer.setSize(rect.width, rect.height, false);
  camera.aspect = Math.max(1e-3, rect.width / Math.max(1, rect.height));
  camera.updateProjectionMatrix();
  draw();
}

function screenSize() {
  const dpr = window.devicePixelRatio || 1;
  return { w: mapCanvas.width / dpr, h: mapCanvas.height / dpr };
}

function setView(mode) {
  viewMode = mode;
  mainPanel.classList.toggle("view-3d", mode === "3d");
  mainPanel.classList.toggle("view-2d", mode === "2d");
  cloudCanvas.classList.toggle("hidden", mode !== "3d");
  mapCanvas.classList.toggle("hidden", false);
  controls.enabled = mode === "3d";
  draw();
}

function fit2d() {
  if (!mapImage.width || !mapImage.height) return;
  const { w, h } = screenSize();
  scale2d = Math.min(w / mapImage.width, h / mapImage.height) * 0.94;
  offsetX = (w - mapImage.width * scale2d) / 2;
  offsetY = (h - mapImage.height * scale2d) / 2;
  draw();
}

function updateGridForBounds(bounds) {
  if (!bounds) return;
  const dx = bounds.max[0] - bounds.min[0];
  const dy = bounds.max[1] - bounds.min[1];
  const span = Math.max(6, dx, dy);
  gridHelper.scale.setScalar(Math.max(1, span / 30));
  gridHelper.position.set(
    (bounds.min[0] + bounds.max[0]) * 0.5,
    (bounds.min[1] + bounds.max[1]) * 0.5,
    Math.min(0, bounds.min[2]),
  );
}

function fit3d() {
  if (!cloud.bounds) return;
  const min = new THREE.Vector3(...cloud.bounds.min);
  const max = new THREE.Vector3(...cloud.bounds.max);
  tmpBox.set(min, max);
  tmpBox.getCenter(tmpCenter);
  tmpBox.getSize(tmpSize);
  cloud.center = [tmpCenter.x, tmpCenter.y, tmpCenter.z];
  controls.target.copy(tmpCenter);

  const maxDim = Math.max(tmpSize.x, tmpSize.y, tmpSize.z, cloud.voxelSize * 8, 1.0);
  const fovRad = THREE.MathUtils.degToRad(camera.fov);
  const distance = Math.max(maxDim * 1.45, (maxDim * 0.5) / Math.tan(fovRad * 0.5));
  const direction = new THREE.Vector3(1.0, -1.15, 0.82).normalize();
  camera.position.copy(tmpCenter.clone().add(direction.multiplyScalar(distance)));
  controls.minDistance = Math.max(cloud.voxelSize * 2.0, distance * 0.02);
  controls.maxDistance = Math.max(distance * 8.0, maxDim * 8.0);
  camera.near = Math.max(0.02, distance / 4000.0);
  camera.far = Math.max(1000.0, distance * 12.0, maxDim * 20.0);
  camera.updateProjectionMatrix();

  updateGridForBounds(cloud.bounds);
  draw();
}

function imageToWorld(px, py) {
  const res = currentInfo.resolution;
  const origin = currentInfo.origin;
  return {
    x: origin[0] + (px + 0.5) * res,
    y: origin[1] + (currentInfo.height - py - 0.5) * res,
  };
}

function worldToImage(x, y) {
  const res = currentInfo.resolution;
  const origin = currentInfo.origin;
  return {
    x: (x - origin[0]) / res,
    y: currentInfo.height - 1 - Math.floor((y - origin[1]) / res),
  };
}

function eventPoint2d(event) {
  const rect = mapCanvas.getBoundingClientRect();
  const sx = event.clientX - rect.left;
  const sy = event.clientY - rect.top;
  return {
    x: (sx - offsetX) / scale2d,
    y: (sy - offsetY) / scale2d,
    sx,
    sy,
  };
}

function currentProjectionContext() {
  return { rect: cloudCanvas.getBoundingClientRect() };
}

function project3d(x, y, z, ctx = null) {
  const c = ctx || currentProjectionContext();
  tmpVec.set(x, y, z).project(camera);
  if (tmpVec.z < -1 || tmpVec.z > 1) return null;
  return {
    sx: (tmpVec.x * 0.5 + 0.5) * c.rect.width,
    sy: (-tmpVec.y * 0.5 + 0.5) * c.rect.height,
    depth: tmpVec.z,
  };
}

function disposeMaterial(material) {
  if (Array.isArray(material)) {
    material.forEach((item) => item.dispose());
  } else if (material) {
    material.dispose();
  }
}

function disposeObject3d(object) {
  if (!object) return;
  object.traverse((node) => {
    if (node.geometry) node.geometry.dispose();
    if (node.material) disposeMaterial(node.material);
  });
}

function clearCloudObjects() {
  if (pointsObject) {
    scene.remove(pointsObject);
    disposeObject3d(pointsObject);
    pointsObject = null;
  }
  if (voxelObject) {
    scene.remove(voxelObject);
    disposeObject3d(voxelObject);
    voxelObject = null;
  }
  voxelPickMesh = null;
}

function makePointCloudObject(positions, bounds) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));

  const colorArray = new Float32Array(positions.length);
  const minZ = bounds.min[2];
  const maxZ = bounds.max[2];
  const span = Math.max(1e-6, maxZ - minZ);
  for (let i = 0; i < positions.length; i += 3) {
    const t = Math.max(0, Math.min(1, (positions[i + 2] - minZ) / span));
    const color = new THREE.Color().setRGB(
      0.15 + 0.75 * t,
      0.78 - 0.45 * t,
      0.95 - 0.75 * t,
    );
    colorArray[i] = color.r;
    colorArray[i + 1] = color.g;
    colorArray[i + 2] = color.b;
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colorArray, 3));

  const material = new THREE.PointsMaterial({
    size: 0.035,
    vertexColors: true,
    sizeAttenuation: true,
  });
  return new THREE.Points(geometry, material);
}

function makeVoxelGroup(positions, size) {
  const geometry = new THREE.BoxGeometry(size, size, size);
  const fillMaterial = new THREE.MeshStandardMaterial({
    color: 0xf59e0b,
    roughness: 0.42,
    metalness: 0.04,
  });
  const edgeMaterial = new THREE.MeshBasicMaterial({
    color: 0x5b3a06,
    wireframe: true,
    transparent: true,
    opacity: 0.30,
  });

  const count = positions.length / 3;
  const fillMesh = new THREE.InstancedMesh(geometry, fillMaterial, count);
  const edgeMesh = new THREE.InstancedMesh(geometry, edgeMaterial, count);
  fillMesh.instanceMatrix.setUsage(THREE.StaticDrawUsage);
  edgeMesh.instanceMatrix.setUsage(THREE.StaticDrawUsage);

  for (let i = 0; i < count; i += 1) {
    const j = i * 3;
    tmpMatrix.makeTranslation(positions[j], positions[j + 1], positions[j + 2]);
    fillMesh.setMatrixAt(i, tmpMatrix);
    edgeMesh.setMatrixAt(i, tmpMatrix);
  }
  fillMesh.instanceMatrix.needsUpdate = true;
  edgeMesh.instanceMatrix.needsUpdate = true;

  const group = new THREE.Group();
  group.add(fillMesh);
  group.add(edgeMesh);
  return { group, pickMesh: fillMesh };
}

function setCloudData(data) {
  clearCloudObjects();
  const positions = new Float32Array(data.positions);
  pointsObject = makePointCloudObject(positions, data.bounds);
  scene.add(pointsObject);

  cloud.mode = "points";
  cloud.positions = positions;
  cloud.count = positions.length / 3;
  cloud.bounds = data.bounds;
  cloud.voxelSize = voxelSize;
  cloudStats = data;

  cloudZMin = data.z_min;
  cloudZMax = data.z_max;
  if (Number.isFinite(data.z_min) && data.z_min > -1000000) {
    cloudZMinInput.value = data.z_min.toFixed(1);
  }
  cloudZMaxInput.value = data.z_max.toFixed(1);
  cloudMaxPointsInput.value = String(cloudMaxPoints);
  serverState.textContent =
    `${data.sampled_count} / ${data.filtered_point_count} pts (raw ${data.point_count}, z ${formatZRange(data.z_min, data.z_max)}, max=${cloudMaxPoints})`;
  fit3d();
}

function setVoxelData(data) {
  clearCloudObjects();
  const positions = new Float32Array(data.positions);
  const voxelGroup = makeVoxelGroup(positions, data.voxel_size);
  voxelObject = voxelGroup.group;
  voxelPickMesh = voxelGroup.pickMesh;
  scene.add(voxelObject);

  cloud.mode = "voxels";
  cloud.positions = positions;
  cloud.count = positions.length / 3;
  cloud.bounds = data.bounds;
  cloud.voxelSize = data.voxel_size;
  cloudStats = data;

  cloudZMin = data.z_min;
  cloudZMax = data.z_max;
  voxelSize = data.voxel_size;
  voxelMinPoints = data.min_points_per_voxel;
  voxelMinCluster = data.min_cluster_voxels;
  cloudZMinInput.value = data.z_min.toFixed(1);
  cloudZMaxInput.value = data.z_max.toFixed(1);
  cloudMaxPointsInput.value = String(cloudMaxPoints);
  voxelSizeInput.value = data.voxel_size.toFixed(2);
  voxelMinPointsInput.value = String(data.min_points_per_voxel);
  voxelMinClusterInput.value = String(data.min_cluster_voxels);
  serverState.textContent =
    `${data.sampled_voxel_count} / ${data.occupied_voxel_count} voxels (size=${data.voxel_size.toFixed(2)}m, z ${formatZRange(data.z_min, data.z_max)}, minPts=${data.min_points_per_voxel}, cluster=${data.min_cluster_voxels})`;
  fit3d();
}

function formatZRange(zMin, zMax) {
  const lo = Number.isFinite(zMin) && zMin > -1000000 ? zMin.toFixed(1) : "-inf";
  return `${lo}..${zMax.toFixed(1)}m`;
}

function draw3dWaypointsOverlay() {
  const rect = cloudCanvas.getBoundingClientRect();
  mapCanvas.classList.remove("hidden");
  mapCtx.clearRect(0, 0, rect.width, rect.height);
  const ctx = currentProjectionContext();
  draw3dGridOverlay(mapCtx, ctx);
  draw3dObstacles(mapCtx, ctx);
  const refs = allWaypointRefs();

  for (let i = 1; i < refs.length; i += 1) {
    const a = project3d(refs[i - 1].wp.x, refs[i - 1].wp.y, waypointViewZ(refs[i - 1].wp), ctx);
    const b = project3d(refs[i].wp.x, refs[i].wp.y, waypointViewZ(refs[i].wp), ctx);
    if (!a || !b) continue;
    mapCtx.strokeStyle = "#d48bff";
    mapCtx.lineWidth = 3;
    mapCtx.beginPath();
    mapCtx.moveTo(a.sx, a.sy);
    mapCtx.lineTo(b.sx, b.sy);
    mapCtx.stroke();
  }

  for (let i = 0; i < refs.length; i += 1) {
    const wp = refs[i].wp;
    const viewZ = waypointViewZ(wp);
    const p = project3d(wp.x, wp.y, viewZ, ctx);
    if (!p) continue;
    mapCtx.fillStyle = i === selected ? "#67e8f9" : "#facc15";
    mapCtx.beginPath();
    mapCtx.arc(p.sx, p.sy, i === selected ? 8 : 6, 0, Math.PI * 2);
    mapCtx.fill();

    const yaw = (wp.yawDeg || 0) * Math.PI / 180;
    const end = project3d(
      wp.x + Math.cos(yaw) * Math.max(cloud.voxelSize * 3.0, 0.4),
      wp.y + Math.sin(yaw) * Math.max(cloud.voxelSize * 3.0, 0.4),
      viewZ,
      ctx,
    );
    if (end) {
      mapCtx.strokeStyle = mapCtx.fillStyle;
      mapCtx.lineWidth = 2;
      mapCtx.beginPath();
      mapCtx.moveTo(p.sx, p.sy);
      mapCtx.lineTo(end.sx, end.sy);
      mapCtx.stroke();
    }

    mapCtx.fillStyle = "#e5e7eb";
    mapCtx.font = "12px Segoe UI";
    mapCtx.fillText(`${i + 1}:${wp.obstacle || "flat"}`, p.sx + 10, p.sy - 8);
  }
}

function draw3dObstacles(overlay, ctx) {
  overlay.save();
  const items = obstacleDraft ? [...obstacles, obstacleDraft] : obstacles;
  for (let i = 0; i < items.length; i += 1) {
    const item = items[i];
    const corners = obstacleCorners(item)
      .map((p) => project3d(p.x, p.y, 0, ctx))
      .filter(Boolean);
    if (corners.length !== 4) continue;
    const active = item === activeObstacleItem();
    overlay.beginPath();
    overlay.moveTo(corners[0].sx, corners[0].sy);
    for (let j = 1; j < corners.length; j += 1) overlay.lineTo(corners[j].sx, corners[j].sy);
    overlay.closePath();
    overlay.fillStyle = active ? "rgba(103, 232, 249, 0.16)" : "rgba(248, 113, 113, 0.12)";
    overlay.strokeStyle = active ? "#67e8f9" : "#fb7185";
    overlay.lineWidth = active ? 3 : 2;
    overlay.fill();
    overlay.stroke();
    const center = project3d(item.x, item.y, 0, ctx);
    if (center) {
      overlay.fillStyle = "#fee2e2";
      overlay.font = "12px Segoe UI";
      overlay.fillText(`${item.name || "obstacle"}:${item.obstacle || "flat"}`, center.sx + 6, center.sy - 6);
    }
  }
  overlay.restore();
}

function waypointViewZ(wp) {
  if (Number.isFinite(wp._viewZ)) return wp._viewZ;
  if (Number.isFinite(wp.z)) return wp.z;
  return 0;
}

function niceGridStep(span) {
  if (!Number.isFinite(span) || span <= 0) return 1;
  const raw = span / 12;
  const pow = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / pow;
  if (n < 1.5) return pow;
  if (n < 3.5) return 2 * pow;
  if (n < 7.5) return 5 * pow;
  return 10 * pow;
}

function draw3dGridOverlay(overlay, ctx) {
  if (!cloud.bounds) return;
  const min = cloud.bounds.min;
  const max = cloud.bounds.max;
  const span = Math.max(max[0] - min[0], max[1] - min[1]);
  const step = niceGridStep(span);
  const z = Math.min(0, min[2]);
  const startX = Math.floor(min[0] / step) * step;
  const endX = Math.ceil(max[0] / step) * step;
  const startY = Math.floor(min[1] / step) * step;
  const endY = Math.ceil(max[1] / step) * step;

  overlay.save();
  overlay.lineWidth = 1;
  overlay.strokeStyle = "rgba(148, 163, 184, 0.20)";
  for (let x = startX; x <= endX + 1e-6; x += step) {
    const a = project3d(x, startY, z, ctx);
    const b = project3d(x, endY, z, ctx);
    if (!a || !b) continue;
    overlay.beginPath();
    overlay.moveTo(a.sx, a.sy);
    overlay.lineTo(b.sx, b.sy);
    overlay.stroke();
  }
  for (let y = startY; y <= endY + 1e-6; y += step) {
    const a = project3d(startX, y, z, ctx);
    const b = project3d(endX, y, z, ctx);
    if (!a || !b) continue;
    overlay.beginPath();
    overlay.moveTo(a.sx, a.sy);
    overlay.lineTo(b.sx, b.sy);
    overlay.stroke();
  }

  const axisLen = Math.max(step * 2, span * 0.12);
  let axisBase = [0, 0, z];
  let origin = project3d(axisBase[0], axisBase[1], axisBase[2], ctx);
  if (!origin) {
    axisBase = [cloud.center[0], cloud.center[1], z];
    origin = project3d(axisBase[0], axisBase[1], axisBase[2], ctx);
  }
  if (origin) {
    const xAxis = project3d(axisBase[0] + axisLen, axisBase[1], z, ctx);
    const yAxis = project3d(axisBase[0], axisBase[1] + axisLen, z, ctx);
    drawAxis(overlay, origin, xAxis, "#f87171", "X");
    drawAxis(overlay, origin, yAxis, "#4ade80", "Y");
  }
  overlay.restore();
}

function drawAxis(overlay, a, b, color, label) {
  if (!a || !b) return;
  overlay.strokeStyle = color;
  overlay.fillStyle = color;
  overlay.lineWidth = 2;
  overlay.beginPath();
  overlay.moveTo(a.sx, a.sy);
  overlay.lineTo(b.sx, b.sy);
  overlay.stroke();
  overlay.font = "12px Segoe UI";
  overlay.fillText(label, b.sx + 4, b.sy + 4);
}

function draw2d() {
  const { w, h } = screenSize();
  mapCtx.clearRect(0, 0, w, h);
  mapCtx.fillStyle = "#0b0f17";
  mapCtx.fillRect(0, 0, w, h);
  if (mapImage.complete && mapImage.width) {
    mapCtx.imageSmoothingEnabled = false;
    mapCtx.drawImage(mapImage, offsetX, offsetY, mapImage.width * scale2d, mapImage.height * scale2d);
  }
  draw2dObstacles();
  draw2dWaypoints();
}

function worldToScreen2d(x, y) {
  const p = worldToImage(x, y);
  return { sx: offsetX + p.x * scale2d, sy: offsetY + p.y * scale2d };
}

function draw2dObstacles() {
  mapCtx.save();
  const items = obstacleDraft ? [...obstacles, obstacleDraft] : obstacles;
  for (let i = 0; i < items.length; i += 1) {
    const item = items[i];
    const corners = obstacleCorners(item).map((p) => worldToScreen2d(p.x, p.y));
    const active = item === activeObstacleItem();
    mapCtx.beginPath();
    mapCtx.moveTo(corners[0].sx, corners[0].sy);
    for (let j = 1; j < corners.length; j += 1) mapCtx.lineTo(corners[j].sx, corners[j].sy);
    mapCtx.closePath();
    mapCtx.fillStyle = active ? "rgba(103, 232, 249, 0.18)" : "rgba(248, 113, 113, 0.14)";
    mapCtx.strokeStyle = active ? "#67e8f9" : "#fb7185";
    mapCtx.lineWidth = active ? 3 : 2;
    mapCtx.fill();
    mapCtx.stroke();
    const center = worldToScreen2d(item.x, item.y);
    mapCtx.fillStyle = "#fee2e2";
    mapCtx.font = "12px Segoe UI";
    mapCtx.fillText(`${item.name || "obstacle"}:${item.obstacle || "flat"}`, center.sx + 6, center.sy - 6);
  }
  mapCtx.restore();
}

function draw2dWaypoints() {
  mapCtx.save();
  mapCtx.lineWidth = 2;
  const refs = allWaypointRefs();
  if (refs.length > 1) {
    mapCtx.beginPath();
    for (let i = 0; i < refs.length; i += 1) {
      const p = worldToImage(refs[i].wp.x, refs[i].wp.y);
      const sx = offsetX + p.x * scale2d;
      const sy = offsetY + p.y * scale2d;
      if (i === 0) mapCtx.moveTo(sx, sy);
      else mapCtx.lineTo(sx, sy);
    }
    mapCtx.strokeStyle = "#22c55e";
    mapCtx.stroke();
  }
  for (let i = 0; i < refs.length; i += 1) {
    const wp = refs[i].wp;
    const p = worldToImage(wp.x, wp.y);
    const sx = offsetX + p.x * scale2d;
    const sy = offsetY + p.y * scale2d;
    mapCtx.fillStyle = i === selected ? "#67e8f9" : "#facc15";
    mapCtx.beginPath();
    mapCtx.arc(sx, sy, i === selected ? 8 : 6, 0, Math.PI * 2);
    mapCtx.fill();
    const yaw = (wp.yawDeg || 0) * Math.PI / 180;
    mapCtx.beginPath();
    mapCtx.moveTo(sx, sy);
    mapCtx.lineTo(sx + Math.cos(yaw) * 24, sy - Math.sin(yaw) * 24);
    mapCtx.strokeStyle = mapCtx.fillStyle;
    mapCtx.stroke();
    mapCtx.fillStyle = "#e5e7eb";
    mapCtx.font = "12px Segoe UI";
    mapCtx.fillText(`${i + 1}:${wp.obstacle || "flat"}`, sx + 10, sy - 8);
  }
  mapCtx.restore();
}

function draw() {
  if (viewMode === "3d") {
    renderer.render(scene, camera);
    draw3dWaypointsOverlay();
  } else {
    draw2d();
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (viewMode === "3d") {
    controls.update();
  }
  draw();
}

async function loadMaps() {
  const res = await fetch("/api/maps");
  const data = await res.json();
  maps = collapseMapsBySource((data.maps || []).filter((m) => !m.is_debug));
  mapSelect.innerHTML = "";
  for (const m of maps) {
    const opt = document.createElement("option");
    opt.value = m.name;
    opt.textContent = mapDisplayName(m);
    mapSelect.appendChild(opt);
  }
  serverState.textContent = `${UI_VERSION}: ${maps.length} maps`;
  if (maps.length) {
    const preferred = maps.find((m) => m.name === "map1_height") || maps.find((m) => m.name === "map1") || maps[0];
    mapSelect.value = preferred.name;
    await selectMap(preferred.name);
  }
}

function collapseMapsBySource(items) {
  const groups = new Map();
  for (const item of items) {
    const key = item.meta?.source_pcd || item.name;
    item.displayName = sourceDisplayName(item);
    const existing = groups.get(key);
    if (!existing || mapPreferenceScore(item) > mapPreferenceScore(existing)) {
      groups.set(key, item);
    }
  }
  return [...groups.values()].sort((a, b) => mapDisplayName(a).localeCompare(mapDisplayName(b)));
}

function sourceDisplayName(item) {
  const source = item.meta?.source_pcd || "";
  const normalized = String(source).replaceAll("\\", "/");
  const file = normalized.split("/").pop() || item.name;
  return file.replace(/\.pcd$/i, "") || item.name;
}

function mapDisplayName(item) {
  return item.displayName || sourceDisplayName(item);
}

function mapPreferenceScore(item) {
  let score = 0;
  if (item.name.endsWith("_height")) score += 30;
  if (item.name === "map1_height") score += 20;
  if (item.layers?.includes("pcd_z1_topdown.png")) score += 10;
  if (item.routes?.length) score += 5;
  return score;
}

async function selectMap(name) {
  currentMap = maps.find((m) => m.name === name);
  if (!currentMap) return;
  layerSelect.innerHTML = "";
  const layers = currentMap.layers.includes("pcd_z1_topdown.png")
    ? ["pcd_z1_topdown.png"]
    : [currentMap.layers[0]];
  for (const layer of layers) {
    const opt = document.createElement("option");
    opt.value = layer;
    opt.textContent = layer;
    layerSelect.appendChild(opt);
  }
  const loadedStored = loadStoredRoute(name);
  if (!loadedStored) {
    await loadRouteFromDisk(currentMap);
  }
  selected = allWaypointRefs().length ? 0 : -1;
  renderObstacleControls();
  renderWaypointList();
  await Promise.all([selectLayer(layers[0]), loadPointCloud()]);
}

function loadStoredRoute(name) {
  const raw = localStorage.getItem(`route:${name}`);
  if (!raw) {
    waypoints = [];
    obstacles = [];
    activeObstacle = -1;
    routeSegmentsMeta = [];
    return false;
  }
  try {
    const stored = JSON.parse(raw);
    applyLoadedRoute(stored);
  } catch (_) {
    waypoints = [];
    obstacles = [];
    routeSegmentsMeta = [];
    activeObstacle = -1;
    return false;
  }
  return true;
}

async function loadRouteFromDisk(mapItem) {
  if (!mapItem?.routes?.length) {
    waypoints = [];
    obstacles = [];
    routeSegmentsMeta = [];
    activeObstacle = -1;
    return false;
  }
  const preferred = mapItem.routes.find((name) => name === `${routeName.value.trim() || "test_route"}.json`)
    || mapItem.routes[0];
  const res = await fetch(`/api/route?map=${encodeURIComponent(mapItem.name)}&name=${encodeURIComponent(preferred)}`);
  const route = await res.json();
  applyLoadedRoute(route);
  persist();
  return true;
}

async function selectLayer(layer) {
  currentLayer = layer;
  const infoRes = await fetch(`/api/layer_info?map=${encodeURIComponent(currentMap.name)}&layer=${encodeURIComponent(layer)}`);
  currentInfo = await infoRes.json();
  mapImage = new Image();
  mapImage.onload = () => {
    if (viewMode === "2d") fit2d();
    else draw();
  };
  mapImage.src = `/api/layer?map=${encodeURIComponent(currentMap.name)}&layer=${encodeURIComponent(layer)}&t=${Date.now()}`;
}

async function loadPointCloud() {
  serverState.textContent = "loading pcd";
  const endpoint = cloudMode === "voxels"
    ? `/api/voxels?map=${encodeURIComponent(currentMap.name)}&max_voxels=${cloudMaxPoints}&z_min=${cloudZMin}&z_max=${cloudZMax}&voxel_size=${voxelSize}&min_points_per_voxel=${voxelMinPoints}&min_cluster_voxels=${voxelMinCluster}`
    : `/api/pointcloud?map=${encodeURIComponent(currentMap.name)}&max_points=${cloudMaxPoints}&z_min=${cloudZMin}&z_max=${cloudZMax}`;
  const res = await fetch(endpoint);
  const data = await res.json();
  if (data.ok === false) throw new Error(data.error);
  if (cloudMode === "voxels") setVoxelData(data);
  else setCloudData(data);
}

function updatePointer(event) {
  const rect = cloudCanvas.getBoundingClientRect();
  pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
}

function screenToGround(event) {
  updatePointer(event);
  const hit = new THREE.Vector3();
  const ok = raycaster.ray.intersectPlane(zPlane, hit);
  if (!ok) return null;
  return { x: hit.x, y: hit.y, z: 0 };
}

function pick3dPoint(event) {
  updatePointer(event);
  if (cloud.mode === "voxels" && voxelPickMesh) {
    const hits = raycaster.intersectObject(voxelPickMesh, false);
    if (hits.length > 0) {
      const hit = hits[0];
      if (Number.isInteger(hit.instanceId)) {
        voxelPickMesh.getMatrixAt(hit.instanceId, tmpMatrix);
        tmpPosition.setFromMatrixPosition(tmpMatrix);
        return { x: tmpPosition.x, y: tmpPosition.y, z: tmpPosition.z };
      }
      return { x: hit.point.x, y: hit.point.y, z: hit.point.z };
    }
  }

  if (cloud.count > 0) {
    let best = null;
    let bestD = 16 * 16;
    const stride = Math.max(1, Math.floor(cloud.count / 45000));
    const ctx = currentProjectionContext();
    for (let i = 0; i < cloud.count; i += stride) {
      const j = i * 3;
      const p = project3d(cloud.positions[j], cloud.positions[j + 1], cloud.positions[j + 2], ctx);
      if (!p) continue;
      const dx = p.sx - (event.clientX - ctx.rect.left);
      const dy = p.sy - (event.clientY - ctx.rect.top);
      const d = dx * dx + dy * dy;
      if (d < bestD) {
        bestD = d;
        best = { x: cloud.positions[j], y: cloud.positions[j + 1], z: cloud.positions[j + 2] };
      }
    }
    if (best) return best;
  }

  return screenToGround(event);
}

function nearestWaypointScreen(sx, sy) {
  let best = -1;
  let bestD = 18 * 18;
  const ctx = viewMode === "3d" ? currentProjectionContext() : null;
  const refs = allWaypointRefs();
  for (let i = 0; i < refs.length; i += 1) {
    const wp = refs[i].wp;
    let p;
    if (viewMode === "3d") {
      p = project3d(wp.x, wp.y, waypointViewZ(wp), ctx);
    } else {
      const img = worldToImage(wp.x, wp.y);
      p = { sx: offsetX + img.x * scale2d, sy: offsetY + img.y * scale2d };
    }
    if (!p) continue;
    const dx = p.sx - sx;
    const dy = p.sy - sy;
    const d = dx * dx + dy * dy;
    if (d < bestD) {
      bestD = d;
      best = i;
    }
  }
  return best;
}

function setSelected(index) {
  selected = index;
  const ref = selectedRef();
  if (ref) {
    const wp = ref.wp;
    wpX.value = wp.x.toFixed(3);
    wpY.value = wp.y.toFixed(3);
    wpYaw.value = (wp.yawDeg || 0).toFixed(1);
    wpObstacle.value = wp.obstacle || "flat";
    wpSpeed.value = (wp.speed || 0.35).toFixed(2);
    wpPolicy.value = wp.policy || "rough";
    wpTol.value = (wp.tolerance || 0.15).toFixed(2);
    wpYawTol.value = Number(wp.yawToleranceDeg ?? 30).toFixed(1);
    wpRequireYaw.checked = Boolean(wp.requireYaw);
    wpPreDockDist.value = wp.preDockDistance == null ? "" : Number(wp.preDockDistance).toFixed(2);
    wpPreDockTol.value = wp.preDockTolerance == null ? "" : Number(wp.preDockTolerance).toFixed(2);
  }
  renderWaypointList();
  draw();
}

function addWaypoint(world) {
  if (!world) return;
  const refs = allWaypointRefs();
  const previous = refs.length ? refs[refs.length - 1].wp : null;
  let yawDeg = 0;
  if (previous) yawDeg = Math.atan2(world.y - previous.y, world.x - previous.x) * 180 / Math.PI;
  const info = waypointObstacleInfo(world.x, world.y);
  waypoints.push({
    x: Number(world.x.toFixed(4)),
    y: Number(world.y.toFixed(4)),
    _viewZ: Number((world.z || 0).toFixed(4)),
    yawDeg: Number(yawDeg.toFixed(1)),
    yawToleranceDeg: Number(wpYawTol.value || 30),
    speed: Number(wpSpeed.value || 0.35),
    policy: info.policy || wpPolicy.value,
    tolerance: Number(wpTol.value || 0.15),
    requireYaw: Boolean(wpRequireYaw.checked),
    preDockDistance: wpPreDockDist.value === "" ? null : Number(wpPreDockDist.value),
    preDockTolerance: wpPreDockTol.value === "" ? null : Number(wpPreDockTol.value),
    obstacle: info.obstacle,
    obstacleName: info.obstacleName,
  });
  persist();
  setSelected(allWaypointRefs().length - 1);
}

function persist() {
  if (!currentMap) return;
  const route = currentRoute();
  localStorage.setItem(`route:${currentMap.name}`, JSON.stringify({
    name: route.name,
    waypoints: route.waypoints,
    segments: route.segments,
    obstacles: route.obstacles,
  }));
}

function renderWaypointList() {
  ensureWaypoints();
  renderObstacleControls();
  waypointList.innerHTML = "";
  const refs = allWaypointRefs();
  refs.forEach((ref, i) => {
    const wp = ref.wp;
    const li = document.createElement("li");
    li.className = i === selected ? "active" : "";
    const strictYawText = wp.requireYaw ? " strict_yaw" : "";
    const preDockText = wp.preDockDistance == null ? "" : ` pre_dock=${Number(wp.preDockDistance).toFixed(2)}`;
    li.textContent = `${i + 1}. [${wp.obstacle || "flat"}] x=${wp.x.toFixed(2)} y=${wp.y.toFixed(2)} yaw=${(wp.yawDeg || 0).toFixed(0)} ${wp.policy || "rough"}${strictYawText}${preDockText}`;
    li.onclick = () => setSelected(i);
    waypointList.appendChild(li);
  });
}

function renderObstacleControls() {
  ensureObstacles();
  obstacleSelect.innerHTML = "";
  for (let i = 0; i < obstacles.length; i += 1) {
    const opt = document.createElement("option");
    opt.value = String(i);
    opt.textContent = `${i + 1}. ${obstacles[i].name} (${obstacles[i].obstacle})`;
    obstacleSelect.appendChild(opt);
  }
  const item = activeObstacleItem();
  obstacleSelect.value = activeObstacle >= 0 ? String(activeObstacle) : "";
  if (!item) {
    obstacleName.value = `obstacle_${obstacles.length + 1}`;
    obstacleType.value = "rough_pit";
    obstacleX.value = "";
    obstacleY.value = "";
    obstacleYaw.value = "0";
    obstacleLength.value = "1.00";
    obstacleWidth.value = "1.00";
    obstaclePolicy.value = "rough";
    renderObstacleList();
    return;
  }
  obstacleName.value = item.name;
  obstacleType.value = item.obstacle;
  obstacleX.value = item.x.toFixed(3);
  obstacleY.value = item.y.toFixed(3);
  obstacleYaw.value = (item.yawDeg || 0).toFixed(1);
  obstacleLength.value = (item.length || 1).toFixed(2);
  obstacleWidth.value = (item.width || 1).toFixed(2);
  obstaclePolicy.value = item.policy || defaultPolicyForObstacle(item.obstacle);
  renderObstacleList();
}

function renderObstacleList() {
  obstacleList.innerHTML = "";
  ensureObstacles();
  obstacles.forEach((item, i) => {
    const li = document.createElement("li");
    li.className = i === activeObstacle ? "active" : "";
    li.textContent = `${i + 1}. [${item.obstacle}] ${item.name} x=${item.x.toFixed(2)} y=${item.y.toFixed(2)} yaw=${(item.yawDeg || 0).toFixed(0)}`;
    li.onclick = () => {
      activeObstacle = i;
      renderObstacleControls();
      draw();
    };
    obstacleList.appendChild(li);
  });
}

function currentRoute() {
  refreshWaypointObstacles(false);
  const cleanedWaypoints = cleanWaypoints(waypoints);
  const segmentMeta = routeSegmentsMeta.length
    ? routeSegmentsMeta.map((segment) => ({ ...segment }))
    : [{ name: "segment_1", obstacle: "flat", waypointCount: cleanedWaypoints.length }];
  const segments = [];
  let cursor = 0;
  for (const meta of segmentMeta) {
    const count = Math.max(0, Number(meta.waypointCount || 0));
    const items = cleanedWaypoints.slice(cursor, cursor + count);
    cursor += count;
    if (!items.length) continue;
    segments.push({
      name: meta.name || `segment_${segments.length + 1}`,
      obstacle: meta.obstacle || "flat",
      waypoints: items,
    });
  }
  if (cursor < cleanedWaypoints.length) {
    const remaining = cleanedWaypoints.slice(cursor);
    if (segments.length) {
      segments[segments.length - 1].waypoints.push(...remaining);
    } else {
      segments.push({
        name: "segment_1",
        obstacle: "flat",
        waypoints: remaining,
      });
    }
  }
  if (!segments.length && cleanedWaypoints.length) {
    segments.push({
      name: "segment_1",
      obstacle: "flat",
      waypoints: cleanedWaypoints,
    });
  }
  return {
    name: routeName.value.trim() || "test_route",
    map: mapDisplayName(currentMap || {}) || currentMap?.name || "",
    frame_id: "map",
    createdAt: new Date().toISOString(),
    yawToleranceDegDefault: routeSettings.yawToleranceDegDefault,
    requireYawDefault: routeSettings.requireYawDefault,
    preDockDistanceDefault: routeSettings.preDockDistanceDefault,
    preDockToleranceDefault: routeSettings.preDockToleranceDefault,
    obstacles: cleanObstacles(obstacles),
    segments,
    waypoints: cleanedWaypoints,
  };
}

function cleanWaypoints(items) {
  refreshWaypointObstacles(false);
  return items.map((wp) => ({
    x: Number(wp.x),
    y: Number(wp.y),
    yawDeg: Number(wp.yawDeg || 0),
    yawToleranceDeg: Number(wp.yawToleranceDeg ?? 30),
    speed: Number(wp.speed || 0.35),
    policy: wp.policy || "rough",
    tolerance: Number(wp.tolerance || 0.15),
    requireYaw: Boolean(wp.requireYaw),
    preDockDistance: wp.preDockDistance == null ? null : Number(wp.preDockDistance),
    preDockTolerance: wp.preDockTolerance == null ? null : Number(wp.preDockTolerance),
    obstacle: wp.obstacle || "flat",
    obstacleName: wp.obstacleName || "",
  }));
}

function cleanObstacles(items) {
  ensureObstacles();
  return items.map((item, index) => ({
    name: item.name || `obstacle_${index + 1}`,
    obstacle: item.obstacle || "rough_pit",
    x: Number(item.x || 0),
    y: Number(item.y || 0),
    yawDeg: Number(item.yawDeg || 0),
    length: Math.max(0.05, Number(item.length || 1.0)),
    width: Math.max(0.05, Number(item.width || 1.0)),
    policy: item.policy || defaultPolicyForObstacle(item.obstacle),
  }));
}

function download(filename, text) {
  const blob = new Blob([text], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

function handleDown2d(event) {
  if (addingObstacle) {
    const p = eventPoint2d(event);
    const world = imageToWorld(p.x, p.y);
    obstacleDraft = makeDefaultObstacle(obstacles.length, world.x, world.y, 0.2, 0.2);
    obstacleDraft._start = world;
    moved2d = false;
    dragging2d = true;
    dragLast2d = { x: event.clientX, y: event.clientY };
    draw();
    return;
  }
  dragging2d = true;
  moved2d = false;
  dragLast2d = { x: event.clientX, y: event.clientY };
}

function updateCursor3d(event) {
  const p = pick3dPoint(event);
  if (p) {
    const info = waypointObstacleInfo(p.x, p.y);
    const detail = cloud.mode === "voxels"
      ? `voxels: ${cloud.count}\nvoxel: ${cloud.voxelSize.toFixed(2)} m`
      : `points: ${cloud.count}`;
    cursorInfo.textContent = `x: ${p.x.toFixed(3)}
y: ${p.y.toFixed(3)}
z: ${p.z.toFixed(3)}
terrain: ${info.obstacle}
${detail}
z min: ${cloudZMin.toFixed(1)} m
z max: ${cloudZMax.toFixed(1)} m
max pts: ${cloudMaxPoints}
mode: ${cloudMode}`;
  } else {
    cursorInfo.textContent = `3D map loading...
mode: ${cloudMode}`;
  }
}

function handleMove2d(event) {
  const p = eventPoint2d(event);
  const world = imageToWorld(p.x, p.y);
  const info = waypointObstacleInfo(world.x, world.y);
  cursorInfo.textContent = `x: ${world.x.toFixed(3)}
y: ${world.y.toFixed(3)}
terrain: ${info.obstacle}
px: ${p.x.toFixed(1)}, py: ${p.y.toFixed(1)}
layer: ${currentLayer}`;

  if (!dragging2d || !dragLast2d) return;
  const dx = event.clientX - dragLast2d.x;
  const dy = event.clientY - dragLast2d.y;
  if (Math.abs(dx) + Math.abs(dy) > 2) moved2d = true;

  if (addingObstacle && obstacleDraft?._start) {
    const start = obstacleDraft._start;
    obstacleDraft.x = Number(((start.x + world.x) * 0.5).toFixed(4));
    obstacleDraft.y = Number(((start.y + world.y) * 0.5).toFixed(4));
    obstacleDraft.length = Math.max(0.05, Number(Math.abs(world.x - start.x).toFixed(4)));
    obstacleDraft.width = Math.max(0.05, Number(Math.abs(world.y - start.y).toFixed(4)));
    draw();
    return;
  }

  if (event.shiftKey && selected >= 0) {
    const ref = selectedRef();
    if (!ref) return;
    const wp = ref.wp;
    wp.yawDeg = Math.atan2(world.y - wp.y, world.x - wp.x) * 180 / Math.PI;
    persist();
    renderWaypointList();
  } else {
    offsetX += dx;
    offsetY += dy;
  }
  dragLast2d = { x: event.clientX, y: event.clientY };
  draw();
}

async function handleUp2d(event) {
  dragging2d = false;
  if (addingObstacle && obstacleDraft) {
    const item = { ...obstacleDraft };
    delete item._start;
    item.name = obstacleName.value.trim() || `obstacle_${obstacles.length + 1}`;
    item.obstacle = obstacleType.value || "rough_pit";
    item.policy = obstaclePolicy.value || defaultPolicyForObstacle(item.obstacle);
    obstacles.push(item);
    activeObstacle = obstacles.length - 1;
    addingObstacle = false;
    obstacleDraft = null;
    refreshWaypointObstacles(true);
    persist();
    renderObstacleControls();
    renderWaypointList();
    draw();
    return;
  }
  const rect = mapCanvas.getBoundingClientRect();
  const sx = event.clientX - rect.left;
  const sy = event.clientY - rect.top;
  if (moved2d) return;
  const near = nearestWaypointScreen(sx, sy);
  if (near >= 0) {
    setSelected(near);
    return;
  }
  const p = eventPoint2d(event);
  const world = imageToWorld(p.x, p.y);
  addWaypoint(world);
  try {
    const res = await fetch(`/api/cell?map=${encodeURIComponent(currentMap.name)}&x=${world.x}&y=${world.y}`);
    const data = await res.json();
    cursorInfo.textContent += `\ncell: ${JSON.stringify(data.values || {}, null, 2)}`;
  } catch (_) {}
}

function handleWheel2d(event) {
  event.preventDefault();
  const rect = mapCanvas.getBoundingClientRect();
  const sx = event.clientX - rect.left;
  const sy = event.clientY - rect.top;
  const before = { x: (sx - offsetX) / scale2d, y: (sy - offsetY) / scale2d };
  const factor = event.deltaY < 0 ? 1.12 : 0.89;
  scale2d = Math.max(0.03, Math.min(80, scale2d * factor));
  offsetX = sx - before.x * scale2d;
  offsetY = sy - before.y * scale2d;
  draw();
}

mapCanvas.addEventListener("mousedown", (event) => {
  if (viewMode === "2d") handleDown2d(event);
});
mapCanvas.addEventListener("mousemove", (event) => {
  if (viewMode === "2d") handleMove2d(event);
});
mapCanvas.addEventListener("mouseup", (event) => {
  if (viewMode === "2d") handleUp2d(event);
});
mapCanvas.addEventListener("wheel", (event) => {
  if (viewMode === "2d") handleWheel2d(event);
}, { passive: false });

cloudCanvas.addEventListener("mousemove", (event) => {
  if (viewMode === "3d") updateCursor3d(event);
  if (pointerDown3d) {
    const dx = event.clientX - pointerDown3d.x;
    const dy = event.clientY - pointerDown3d.y;
    if (Math.abs(dx) + Math.abs(dy) > 4) moved3d = true;
  }
});

cloudCanvas.addEventListener("pointerdown", (event) => {
  if (viewMode !== "3d" || event.button !== 0) return;
  pointerDown3d = { x: event.clientX, y: event.clientY };
  moved3d = false;
});

cloudCanvas.addEventListener("pointerup", () => {
  pointerDown3d = null;
});

cloudCanvas.addEventListener("pointercancel", () => {
  pointerDown3d = null;
  moved3d = false;
});

cloudCanvas.addEventListener("click", async (event) => {
  if (viewMode !== "3d") return;
  const rect = cloudCanvas.getBoundingClientRect();
  const sx = event.clientX - rect.left;
  const sy = event.clientY - rect.top;
  if (addingObstacle) {
    if (moved3d) {
      moved3d = false;
      return;
    }
    const world = pick3dPoint(event);
    if (!world) return;
    const item = makeDefaultObstacle(obstacles.length, world.x, world.y, Number(obstacleLength.value || 1.0), Number(obstacleWidth.value || 1.0));
    item.name = obstacleName.value.trim() || item.name;
    item.obstacle = obstacleType.value || "rough_pit";
    item.yawDeg = Number(obstacleYaw.value || 0);
    item.policy = obstaclePolicy.value || defaultPolicyForObstacle(item.obstacle);
    obstacles.push(item);
    activeObstacle = obstacles.length - 1;
    addingObstacle = false;
    refreshWaypointObstacles(true);
    persist();
    renderObstacleControls();
    renderWaypointList();
    draw();
    return;
  }
  const near = nearestWaypointScreen(sx, sy);
  if (near >= 0) {
    setSelected(near);
    return;
  }
  if (moved3d) {
    moved3d = false;
    return;
  }
  const world = pick3dPoint(event);
  if (!world) return;
  addWaypoint(world);
  try {
    const res = await fetch(`/api/cell?map=${encodeURIComponent(currentMap.name)}&x=${world.x}&y=${world.y}`);
    const data = await res.json();
    cursorInfo.textContent += `\ncell: ${JSON.stringify(data.values || {}, null, 2)}`;
  } catch (_) {}
});

document.getElementById("fitBtn").onclick = () => (viewMode === "3d" ? fit3d() : fit2d());
document.getElementById("reloadBtn").onclick = loadMaps;
mapSelect.onchange = () => selectMap(mapSelect.value);
layerSelect.onchange = () => selectLayer(layerSelect.value);
viewSelect.onchange = () => setView(viewSelect.value);

cloudModeSelect.onchange = () => {
  cloudMode = cloudModeSelect.value;
  if (currentMap) {
    loadPointCloud().catch((err) => {
      serverState.textContent = "error";
      cursorInfo.textContent = String(err);
    });
  }
};

cloudZMinInput.onchange = () => {
  const next = Number(cloudZMinInput.value);
  if (!Number.isFinite(next)) {
    cloudZMinInput.value = cloudZMin.toFixed(1);
    return;
  }
  cloudZMin = next;
  if (currentMap) {
    loadPointCloud().catch((err) => {
      serverState.textContent = "error";
      cursorInfo.textContent = String(err);
    });
  }
};

cloudZMaxInput.onchange = () => {
  const next = Number(cloudZMaxInput.value);
  if (!Number.isFinite(next)) {
    cloudZMaxInput.value = cloudZMax.toFixed(1);
    return;
  }
  cloudZMax = next;
  if (currentMap) {
    loadPointCloud().catch((err) => {
      serverState.textContent = "error";
      cursorInfo.textContent = String(err);
    });
  }
};

cloudMaxPointsInput.onchange = () => {
  const next = Number(cloudMaxPointsInput.value);
  if (!Number.isFinite(next)) {
    cloudMaxPointsInput.value = String(cloudMaxPoints);
    return;
  }
  cloudMaxPoints = Math.max(1000, Math.min(300000, Math.round(next)));
  cloudMaxPointsInput.value = String(cloudMaxPoints);
  if (currentMap) {
    loadPointCloud().catch((err) => {
      serverState.textContent = "error";
      cursorInfo.textContent = String(err);
    });
  }
};

voxelSizeInput.onchange = () => {
  const next = Number(voxelSizeInput.value);
  if (!Number.isFinite(next)) {
    voxelSizeInput.value = voxelSize.toFixed(2);
    return;
  }
  voxelSize = Math.max(0.01, Math.min(2.0, next));
  voxelSizeInput.value = voxelSize.toFixed(2);
  if (currentMap && cloudMode === "voxels") {
    loadPointCloud().catch((err) => {
      serverState.textContent = "error";
      cursorInfo.textContent = String(err);
    });
  }
};

voxelMinPointsInput.onchange = () => {
  const next = Number(voxelMinPointsInput.value);
  if (!Number.isFinite(next)) {
    voxelMinPointsInput.value = String(voxelMinPoints);
    return;
  }
  voxelMinPoints = Math.max(1, Math.min(100, Math.round(next)));
  voxelMinPointsInput.value = String(voxelMinPoints);
  if (currentMap && cloudMode === "voxels") {
    loadPointCloud().catch((err) => {
      serverState.textContent = "error";
      cursorInfo.textContent = String(err);
    });
  }
};

voxelMinClusterInput.onchange = () => {
  const next = Number(voxelMinClusterInput.value);
  if (!Number.isFinite(next)) {
    voxelMinClusterInput.value = String(voxelMinCluster);
    return;
  }
  voxelMinCluster = Math.max(1, Math.min(10000, Math.round(next)));
  voxelMinClusterInput.value = String(voxelMinCluster);
  if (currentMap && cloudMode === "voxels") {
    loadPointCloud().catch((err) => {
      serverState.textContent = "error";
      cursorInfo.textContent = String(err);
    });
  }
};

obstacleSelect.onchange = () => {
  activeObstacle = Number(obstacleSelect.value);
  renderObstacleControls();
  draw();
};

obstacleType.onchange = () => {
  obstaclePolicy.value = defaultPolicyForObstacle(obstacleType.value);
};

document.getElementById("drawObstacleBtn").onclick = () => {
  addingObstacle = true;
  obstacleDraft = null;
  serverState.textContent = viewMode === "2d" ? "drag obstacle rect" : "click obstacle center";
};

document.getElementById("updateObstacleBtn").onclick = () => {
  const item = activeObstacleItem();
  if (!item) return;
  item.name = obstacleName.value.trim() || `obstacle_${activeObstacle + 1}`;
  item.obstacle = obstacleType.value || "rough_pit";
  item.x = Number(obstacleX.value || 0);
  item.y = Number(obstacleY.value || 0);
  item.yawDeg = Number(obstacleYaw.value || 0);
  item.length = Math.max(0.05, Number(obstacleLength.value || 1.0));
  item.width = Math.max(0.05, Number(obstacleWidth.value || 1.0));
  item.policy = obstaclePolicy.value || defaultPolicyForObstacle(item.obstacle);
  refreshWaypointObstacles(true);
  persist();
  renderObstacleControls();
  renderWaypointList();
  draw();
};

document.getElementById("deleteObstacleBtn").onclick = () => {
  if (activeObstacle < 0 || activeObstacle >= obstacles.length) return;
  obstacles.splice(activeObstacle, 1);
  activeObstacle = Math.min(activeObstacle, obstacles.length - 1);
  refreshWaypointObstacles(true);
  persist();
  renderObstacleControls();
  renderWaypointList();
  draw();
};

document.getElementById("updateWpBtn").onclick = () => {
  const ref = selectedRef();
  if (!ref) return;
  const next = applyObstacleToWaypoint({
    x: Number(wpX.value),
    y: Number(wpY.value),
    _viewZ: waypointViewZ(ref.wp),
    yawDeg: Number(wpYaw.value),
    yawToleranceDeg: Number(wpYawTol.value || 30),
    speed: Number(wpSpeed.value),
    policy: wpPolicy.value,
    tolerance: Number(wpTol.value),
    requireYaw: Boolean(wpRequireYaw.checked),
    preDockDistance: wpPreDockDist.value === "" ? null : Number(wpPreDockDist.value),
    preDockTolerance: wpPreDockTol.value === "" ? null : Number(wpPreDockTol.value),
  }, false);
  waypoints[ref.pointIndex] = next;
  wpObstacle.value = next.obstacle || "flat";
  persist();
  renderWaypointList();
  draw();
};

document.getElementById("deleteWpBtn").onclick = () => {
  const ref = selectedRef();
  if (!ref) return;
  waypoints.splice(ref.pointIndex, 1);
  selected = Math.min(selected, allWaypointRefs().length - 1);
  persist();
  renderWaypointList();
  draw();
};

document.getElementById("clearRouteBtn").onclick = () => {
  waypoints = [];
  selected = -1;
  routeSegmentsMeta = [];
  persist();
  renderWaypointList();
  draw();
};

document.getElementById("exportJsonBtn").onclick = () => {
  const route = currentRoute();
  download(`${route.name}.json`, JSON.stringify(route, null, 2));
};

document.getElementById("saveRouteBtn").onclick = async () => {
  const route = currentRoute();
  const res = await fetch("/api/route", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ map: currentMap.name, route }),
  });
  const data = await res.json();
  serverState.textContent = data.ok ? "route saved" : data.error;
};

window.addEventListener("resize", resize);

setView("3d");
resize();
animate();
loadMaps().catch((err) => {
  serverState.textContent = "error";
  cursorInfo.textContent = String(err);
});
