/**
 * Adapted MeshLoader for sim2real web console.
 * Supports both fileMap-based loading (original robot_viewer API) and URL-based
 * fetching from the sim2real HTTP server at /meshes/<name>.STL.
 *
 * Uses importmap-resolved Three.js via CDN (no bundler).
 */
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

const _stlLoader = new STLLoader();

let loadersCache = null;
async function getLoaders() {
    if (!loadersCache) {
        loadersCache = { STLLoader: _stlLoader };
    }
    return loadersCache;
}

function normalizePath(path) {
    if (!path) return '';
    return path.replace(/\\/g, '/').replace(/^\/+/, '').replace(/\/+/g, '/');
}

/**
 * Load mesh from URL (sim2real server) or fileMap (robot_viewer compatibility).
 * @param {string} meshPath - e.g. "fl_hip_abduction_Link.STL"
 * @param {Map|null} fileMap - optional File map (compat with MJCFAdapter)
 * @param {string|null} meshBaseUrl - e.g. "/meshes/" for URL-based loading
 * @returns {Promise<THREE.BufferGeometry|THREE.Group|null>}
 */
export async function loadMeshFile(meshPath, fileMap = null, meshBaseUrl = null) {
    const fileName = normalizePath(meshPath).split('/').pop();

    // Strategy 1: try fileMap (robot_viewer compatibility)
    if (fileMap) {
        for (const [key, file] of fileMap.entries()) {
            if (typeof key === 'string' && key.toLowerCase().endsWith(fileName.toLowerCase())) {
                try {
                    const url = URL.createObjectURL(file);
                    const geom = await new Promise((resolve, reject) => {
                        _stlLoader.load(url, resolve, undefined, reject);
                    });
                    URL.revokeObjectURL(url);
                    console.log('[MeshLoader] loaded from fileMap:', fileName);
                    return geom;
                } catch (e) {
                    URL.revokeObjectURL(url);
                    console.warn('[MeshLoader] fileMap load failed:', fileName, e);
                }
            }
        }
    }

    // Strategy 2: try URL-based loading from sim2real server
    const baseUrl = meshBaseUrl || '/meshes/';
    const url = baseUrl + fileName;
    try {
        console.log('[MeshLoader] fetching:', url);
        const resp = await fetch(url);
        if (!resp.ok) {
            console.warn('[MeshLoader] 404:', url);
            return null;
        }
        const arrayBuf = await resp.arrayBuffer();
        const blobUrl = URL.createObjectURL(new Blob([arrayBuf]));
        const geom = await new Promise((resolve, reject) => {
            _stlLoader.load(blobUrl, resolve, undefined, reject);
        });
        URL.revokeObjectURL(blobUrl);
        console.log('[MeshLoader] loaded from URL:', fileName);
        return geom;
    } catch (e) {
        console.warn('[MeshLoader] URL load failed:', url, e);
    }
    return null;
}

export function ensureMeshHasPhongMaterial(meshObject) {
    meshObject.traverse((child) => {
        if (child.isMesh && child.material) {
            const materials = Array.isArray(child.material) ? child.material : [child.material];
            materials.forEach((mat, i) => {
                if (!mat) return;
                if (mat.type === 'MeshBasicMaterial' || mat.type === 'MeshLambertMaterial') {
                    const nm = new THREE.MeshPhongMaterial({
                        color: mat.color, map: mat.map,
                        transparent: mat.transparent, opacity: mat.opacity, side: mat.side,
                        shininess: 50, specular: new THREE.Color(0.3, 0.3, 0.3),
                    });
                    if (nm.map) nm.map.colorSpace = THREE.SRGBColorSpace;
                    materials[i] = nm;
                } else if (mat.isMeshPhongMaterial || mat.isMeshStandardMaterial) {
                    if (mat.shininess === undefined || mat.shininess < 50) mat.shininess = 50;
                    if (!mat.specular) mat.specular = new THREE.Color(0.3, 0.3, 0.3);
                    mat.needsUpdate = true;
                }
            });
            if (Array.isArray(child.material)) child.material = materials;
            else if (materials.length === 1) child.material = materials[0];
        }
    });
}

export { getLoaders };
