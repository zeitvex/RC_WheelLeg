/**
 * RobotViewer3D — sim2real 3D 可视化（基于 robot_viewer 的 MJCFAdapter + Three.js）
 *
 * 加载 wheelleg.xml → MJCFAdapter.parse → Three.js 场景树
 * 建立 jointName → THREE.Object3D 映射，通过 updateJoints(pos16) 实时更新。
 * 支持 OrbitControls 旋转/缩放/平移。
 *
 * 用法：
 *   const viewer = new RobotViewer3D(canvasElement);
 *   await viewer.load('/mjcf/wheelleg.xml');
 *   viewer.updateJoints(jointPositions16);
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { MJCFAdapter } from './MJCFAdapter.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

// 16 关节的标准顺序（与 motor_mapping.py:SIM_JOINT_ORDER 对齐）
const JOINT_ORDER = [
    'fl_hip_abduction_joint', 'fl_hip_pitch_joint', 'fl_knee_joint',
    'fr_hip_abduction_joint', 'fr_hip_pitch_joint', 'fr_knee_joint',
    'rl_hip_abduction_joint', 'rl_hip_pitch_joint', 'rl_knee_joint',
    'rr_hip_abduction_joint', 'rr_hip_pitch_joint', 'rr_knee_joint',
    'fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint',
];

// MJCF → Three.js 坐标轴转换：让 MJCF 的 Z 轴(向上) 映射到 Three.js 的 Y 轴(向上)
const MJCF_TO_THREE = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
// 或直接用 euler: (0, PI, 0)

export class RobotViewer3D {
    /**
     * @param {HTMLCanvasElement} canvas
     * @param {object} [opts]
     * @param {string} [opts.meshBaseUrl='/meshes/']  STL mesh 文件的 HTTP 路径前缀
     * @param {string} [opts.mjcfUrl='/mjcf/wheelleg.xml']
     * @param {string} [opts.backgroundColor='#1a1d24']
     */
    constructor(canvas, opts = {}) {
        this.canvas = canvas;
        this.meshBaseUrl = opts.meshBaseUrl || '/meshes/';
        this.mjcfUrl = opts.mjcfUrl || '/mjcf/wheelleg.xml';

        // Three.js 核心
        const w = canvas.clientWidth, h = canvas.clientHeight;
        this.scene = new THREE.Scene();
        // 移除背景色，使用透明背景，由 CSS 控制
        // this.scene.background = new THREE.Color(opts.backgroundColor || '#1a1d24');

        this.camera = new THREE.PerspectiveCamera(55, w / h, 0.05, 50);
        this.camera.position.set(0.5, 0.35, 0.65);
        this.camera.lookAt(0.2, 0, 0);

        this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
        this.renderer.setSize(w, h);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;

        // OrbitControls
        this.controls = new OrbitControls(this.camera, canvas);
        this.controls.target.set(0.15, 0.08, 0.0);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.12;
        this.controls.update();

        // 灯光
        this._setupLights();

        // 地面
        const grid = new THREE.GridHelper(2, 20, 0x444444, 0x222222);
        grid.position.y = -0.35;
        this.scene.add(grid);

        // 状态
        this.model = null;
        this.rootGroup = null;
        this.jointMap = new Map();           // jointName → { joint, group }
        this._isLoaded = false;
        this._rafId = null;
        this._stlCache = new Map();          // filename → BufferGeometry
    }

    _setupLights() {
        const ambient = new THREE.AmbientLight(0x606060, 1.5);
        this.scene.add(ambient);

        const dir1 = new THREE.DirectionalLight(0xffffff, 2.5);
        dir1.position.set(2, 3, 2);
        this.scene.add(dir1);

        const dir2 = new THREE.DirectionalLight(0x8899cc, 1.0);
        dir2.position.set(-1, 1, -1);
        this.scene.add(dir2);

        const hemi = new THREE.HemisphereLight(0x8899cc, 0x334455, 1.2);
        this.scene.add(hemi);
    }

    // ---- 加载模型 ----
    async load(mjcfUrlOverride) {
        const url = mjcfUrlOverride || this.mjcfUrl;
        console.log('[RobotViewer3D] loading MJCF:', url);
        const resp = await fetch(url);
        if (!resp.ok) throw new Error(`MJCF 404: ${url}`);
        const xmlText = await resp.text();

        // 用 MJCFAdapter 解析 → UnifiedRobotModel
        // fileMap 为空时不传；MeshLoader 会自动 fallback 到 URL 加载
        const model = await MJCFAdapter.parse(xmlText, null);
        this.model = model;
        console.log('[RobotViewer3D] parsed:', model.links.size, 'links,', model.joints.size, 'joints');

        // 取 rootGroup（MJCFAdapter.createThreeObject 已构建完整 hierarchy）
        this.rootGroup = model.threeObject;
        // 坐标轴转换：MJCF → Three.js
        this.rootGroup.applyMatrix4(MJCF_TO_THREE);
        this.scene.add(this.rootGroup);

        // 遍历 joints，建立索引
        this.jointMap.clear();
        for (const [jointName, joint] of model.joints) {
            if (joint.threeObject) {
                this.jointMap.set(jointName, joint);
            }
        }
        // 已建立映射的关节列表
        const mapped = Array.from(this.jointMap.keys()).sort();
        console.log('[RobotViewer3D] joint map:', mapped.length, 'joints');

        this._isLoaded = true;
        this._startRenderLoop();
    }

    // ---- 渲染循环（按需 + 持续） ----
    _startRenderLoop() {
        if (this._rafId) return;
        const loop = () => {
            this.controls.update();
            this.renderer.render(this.scene, this.camera);
            this._rafId = requestAnimationFrame(loop);
        };
        loop();
    }

    // ---- 实时更新关节角度 ----
    /**
     * @param {Float64Array|number[]} pos16 — 16 关节角度 (rad)，顺序同 SIM_JOINT_ORDER
     *   索引 0-11: 腿关节 (fl_abd,fl_pitch,fl_knee,fr...,rl...,rr...)
     *   索引 12-15: 轮子关节 (fl_wheel,fr_wheel,rl_wheel,rr_wheel)
     */
    updateJoints(pos16) {
        if (!this._isLoaded) return;
        for (let i = 0; i < JOINT_ORDER.length && i < pos16.length; i++) {
            const name = JOINT_ORDER[i];
            const joint = this.jointMap.get(name);
            if (joint) {
                MJCFAdapter.setJointAngle(joint, pos16[i]);
            }
        }
    }

    // ---- 重置相机 ----
    resetCamera() {
        this.camera.position.set(0.5, 0.35, 0.65);
        this.controls.target.set(0.15, 0.08, 0.0);
        this.controls.update();
    }

    // ---- 调整大小 ----
    resize() {
        const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
    }

    dispose() {
        if (this._rafId) cancelAnimationFrame(this._rafId);
        this.renderer.dispose();
    }
}
