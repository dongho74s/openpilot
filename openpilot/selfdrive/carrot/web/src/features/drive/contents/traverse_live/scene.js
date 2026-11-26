// Ported from the Traverse Live phone app (~/Documents/traverse-viz-app/web/src/scene.js, 2026-11-05); keep in sync.
// three.js renderer reproducing the drive_viz v7 look: PIL-style road (fades into darkness), soft shadows and glows,
// Kenney (CC0) vehicles converted to real proportions (models_kenney.json), primitive pedestrians / two-wheelers.
// World frame: x forward, y left, z up  ->  three.js (X right, Y up, Z backward) via v3(x, y, z) = (-y, z, -x).
import * as THREE from 'three';
import { ribbon, dashes } from './lanes.js';

const srgb = (r, g, b) => new THREE.Color().setRGB(r / 255, g / 255, b / 255, THREE.SRGBColorSpace);
const lin = (r, g, b) => new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace);
export const v3 = (x, y, z) => new THREE.Vector3(-y, z, -x);

const ROAD_NEAR = [44, 48, 53], ROAD_FAR = [14, 16, 19];
const LINE = [226, 230, 234], BLUE = [48, 136, 255];
const COL = {
  body: lin(0.74, 0.76, 0.78), truck: lin(0.66, 0.69, 0.72), cargo: lin(0.78, 0.80, 0.82), ego: lin(0.84, 0.85, 0.87),
  lead: lin(0.56, 0.68, 0.90), radar: lin(0.28, 0.30, 0.33),
};

// ---------- Carrot web 3D view: car paint and the camera rig ----------
// Common car colours (sRGB 0..1) with rough on-road weights; a track keeps its colour by id.
const PAINTS = [
  [[0.86, 0.87, 0.88], 3], [[0.62, 0.64, 0.67], 2], [[0.34, 0.35, 0.37], 2], [[0.06, 0.065, 0.07], 3],
  [[0.12, 0.17, 0.30], 1], [[0.55, 0.08, 0.07], 0.5], [[0.62, 0.57, 0.48], 0.5], [[0.12, 0.20, 0.16], 0.3],
];
const PAINT_TOTAL = PAINTS.reduce((sum, [, w]) => sum + w, 0);
const paintCache = new Map();
export function paintFor(uid) {
  const n = Number.isFinite(uid) ? Math.abs(Math.trunc(uid)) : 0;
  let x = ((n * 2654435761) >>> 0) / 4294967296 * PAINT_TOTAL;
  let rgb = PAINTS[0][0];
  for (const [c, w] of PAINTS) { if (x < w) { rgb = c; break; } x -= w; }
  const key = rgb.join(',');
  if (!paintCache.has(key)) paintCache.set(key, lin(...rgb));
  return paintCache.get(key);
}
// Mostly diffuse paint under a clear coat: the scene is lit from ahead and the cars are seen from behind, so a metallic
// body would mirror the dark environment and read as black.
function paintMaterial(color) {
  return new THREE.MeshPhysicalMaterial({ color, metalness: 0.08, roughness: 0.42, clearcoat: 0.7, clearcoatRoughness: 0.18 });
}

// Orbit around a point on the original view ray (camera (-12.5, 0, 6.4) looking at (24, 0, 0), world x forward,
// y left, z up), so the default view is exactly the app's and zoom is a dolly along that ray.
export const RIG_PIVOT = [6, 0, 6.4 * (1 - 18.5 / 36.5)];
const RIG_R0 = Math.hypot(18.5, 6.4 * 18.5 / 36.5);
export const RIG_DEFAULT = Object.freeze({ zoom: 1, yaw: 0, pitch: Math.atan2(6.4 * 18.5 / 36.5, 18.5) * 180 / Math.PI });
// Closest: just behind and above the own car (its rear is 3.6 m behind the camera origin); furthest: an overview.
export const RIG_LIMITS = Object.freeze({ zoom: [0.7, 2.2], yaw: [-80, 80], pitch: [2, 60] });
export function clampRig(key, value) {
  const [lo, hi] = RIG_LIMITS[key];
  return Math.min(hi, Math.max(lo, value));
}
export function createCameraRig() {
  return { ...RIG_DEFAULT, target: { ...RIG_DEFAULT } };
}
// Eye position for a rig state; zooming out also tilts the view down a little for an overview.
export function rigEye(r) {
  const dist = RIG_R0 * r.zoom;
  const pitch = Math.min(70, Math.max(2, r.pitch + Math.max(0, r.zoom - 1) * 7)) * Math.PI / 180;
  const yaw = r.yaw * Math.PI / 180;
  return [RIG_PIVOT[0] - dist * Math.cos(pitch) * Math.cos(yaw), RIG_PIVOT[1] + dist * Math.cos(pitch) * Math.sin(yaw),
    RIG_PIVOT[2] + dist * Math.sin(pitch)];
}

// ---------- shaders: ground decals use the v3 screen-row gradient / fade ----------
const DECAL_VS = `varying vec3 vW;
void main(){ vec4 w = modelMatrix * vec4(position, 1.0); vW = w.xyz; gl_Position = projectionMatrix * viewMatrix * w; }`;
const DECAL_FS = `
uniform vec3 uColor; uniform float uOpacity; uniform float uCy; uniform float uH; uniform int uMode; uniform float uDist;
varying vec3 vW;
float hash(vec2 p){ return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453); }
void main(){
  float rowTop = uH - gl_FragCoord.y;
  float rows = pow(clamp((rowTop - uCy) / (uH - uCy), 0.0, 1.0), 0.75);
  if (uMode == 0) {
    // Carrot web 3D view: asphalt grain fixed to the ground (x driven distance), so the road visibly moves under the car.
    vec2 g = vec2(vW.x, -vW.z + uDist);
    float grain = 0.55 * hash(floor(g * vec2(7.0, 3.5))) + 0.45 * hash(floor(g * vec2(1.6, 0.8)));
    vec3 base = mix(vec3(${ROAD_FAR.map((c) => (c / 255).toFixed(4)).join(',')}), vec3(${ROAD_NEAR.map((c) => (c / 255).toFixed(4)).join(',')}), rows);
    gl_FragColor = vec4(base * (0.93 + 0.12 * grain * rows), 1.0);
  }
  else { gl_FragColor = vec4(uColor, uOpacity * clamp(rows * 1.6, 0.0, 1.0)); }
}`;
const SOFT_VS = `varying vec2 vUv; void main(){ vUv = uv; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
const SOFT_RIBBON_FS = `uniform vec3 uColor; uniform float uOpacity; varying vec2 vUv;
void main(){ float d = abs(vUv.y - 0.5) * 2.0; float a = 1.0 - smoothstep(0.35, 1.0, d); gl_FragColor = vec4(uColor, uOpacity * a * a); }`;

function softRectTexture() {
  // soft-edged rectangle computed per pixel (canvas blur filters are not available in every WebView)
  const N = 128, img = new Uint8Array(N * N * 4), inner0 = 34, inner1 = N - 34, R = 30;
  const fall = (d) => { const u = Math.min(1, Math.max(0, d / R)); return 1 - u * u * (3 - 2 * u); };
  for (let y = 0; y < N; y++) for (let x = 0; x < N; x++) {
    const dx = x < inner0 ? inner0 - x : (x > inner1 ? x - inner1 : 0), dy = y < inner0 ? inner0 - y : (y > inner1 ? y - inner1 : 0);
    const a = Math.round(255 * fall(dx) * fall(dy)), o = 4 * (y * N + x);
    img[o] = img[o + 1] = img[o + 2] = a; img[o + 3] = 255;
  }
  const t = new THREE.DataTexture(img, N, N, THREE.RGBAFormat); t.needsUpdate = true; t.magFilter = THREE.LinearFilter;
  t.minFilter = THREE.LinearMipmapLinearFilter; t.generateMipmaps = true; return t;
}
function radialTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 64;
  const g = c.getContext('2d'); const gr = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  gr.addColorStop(0, 'rgba(255,255,255,1)'); gr.addColorStop(0.35, 'rgba(255,255,255,0.55)'); gr.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = gr; g.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}
function envTexture(renderer) {
  const c = document.createElement('canvas'); c.width = 512; c.height = 256;
  const g = c.getContext('2d'); const gr = g.createLinearGradient(0, 0, 0, 256);   // top -> bottom (SceneKit gradient, flipped)
  const css = (r, gg, b) => `rgb(${Math.round(r * 255)},${Math.round(gg * 255)},${Math.round(b * 255)})`;
  gr.addColorStop(0.0, css(0.25, 0.27, 0.30)); gr.addColorStop(0.38, css(0.12, 0.13, 0.14)); gr.addColorStop(0.48, css(0.55, 0.58, 0.62));
  gr.addColorStop(0.55, css(0.10, 0.11, 0.12)); gr.addColorStop(1.0, css(0.02, 0.02, 0.03));
  g.fillStyle = gr; g.fillRect(0, 0, 512, 256);
  const t = new THREE.CanvasTexture(c); t.mapping = THREE.EquirectangularReflectionMapping; t.colorSpace = THREE.SRGBColorSpace;
  const pm = new THREE.PMREMGenerator(renderer); const env = pm.fromEquirectangular(t).texture; t.dispose(); pm.dispose();
  return env;
}

// dynamic triangle strip between two polylines (world xy, constant height)
class Strip {
  constructor(material, maxPts = 260, order = 0) {
    this.max = maxPts;
    this.pos = new Float32Array(maxPts * 2 * 3); this.uv = new Float32Array(maxPts * 2 * 2);
    const idx = [];
    for (let i = 0; i < maxPts - 1; i++) { const k = 2 * i; idx.push(k, k + 1, k + 3, k, k + 3, k + 2); }
    this.geo = new THREE.BufferGeometry();
    this.geo.setAttribute('position', new THREE.BufferAttribute(this.pos, 3).setUsage(THREE.DynamicDrawUsage));
    this.geo.setAttribute('uv', new THREE.BufferAttribute(this.uv, 2).setUsage(THREE.DynamicDrawUsage));
    this.geo.setIndex(idx);
    this.mesh = new THREE.Mesh(this.geo, material); this.mesh.frustumCulled = false; this.mesh.renderOrder = order;
    this.mesh.visible = false;
  }
  set(A, B, z) {
    const n = Math.min(A.length, B.length, this.max);
    if (n < 2) { this.mesh.visible = false; return; }
    for (let i = 0; i < n; i++) {
      const o = 6 * i;
      this.pos[o] = -A[i][1]; this.pos[o + 1] = z; this.pos[o + 2] = -A[i][0];
      this.pos[o + 3] = -B[i][1]; this.pos[o + 4] = z; this.pos[o + 5] = -B[i][0];
      const u = i / (n - 1); this.uv[4 * i] = u; this.uv[4 * i + 1] = 0; this.uv[4 * i + 2] = u; this.uv[4 * i + 3] = 1;
    }
    this.geo.attributes.position.needsUpdate = true; this.geo.attributes.uv.needsUpdate = true;
    this.geo.setDrawRange(0, (n - 1) * 6); this.mesh.visible = true;
  }
}

export class DriveScene {
  constructor(canvas, models) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.NoToneMapping;
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(srgb(12, 14, 17), 45, 165);
    this.scene.environment = envTexture(this.renderer);
    this.scene.environmentIntensity = 1.0;
    const sun = new THREE.DirectionalLight(0xffffff, 2.6); sun.position.copy(v3(5, 12, 40)); sun.target.position.copy(v3(18, 0, 0));
    this.scene.add(sun, sun.target);
    const amb = new THREE.AmbientLight(lin(0.75, 0.8, 0.9), 0.55); this.scene.add(amb);
    this.camera = new THREE.PerspectiveCamera(46, 16 / 9, 0.5, 400);
    this.camera.position.copy(v3(-12.5, 0, 6.4)); this.camera.lookAt(v3(24, 0, 0));
    this.uniformsCommon = { uCy: { value: 0 }, uH: { value: 1 }, uDist: { value: 0 } };
    // Carrot web 3D view: orbit rig around a point on the default view ray (pinch / wheel zoom, drag to look around).
    this.rig = createCameraRig();
    this.applyCamera();

    this.softTex = softRectTexture(); this.radTex = radialTexture();
    this.decal = (rgb, opacity, order, mode = 1) => {
      const m = new THREE.ShaderMaterial({
        vertexShader: DECAL_VS, fragmentShader: DECAL_FS, transparent: mode !== 0, depthWrite: mode === 0, side: THREE.DoubleSide,
        uniforms: { uColor: { value: new THREE.Vector3(...rgb.map((c) => c / 255)) }, uOpacity: { value: opacity }, uMode: { value: mode }, ...this.uniformsCommon },
      });
      return m;
    };
    this.road = new Strip(this.decal([0, 0, 0], 1, 0, 0), 260, 0); this.scene.add(this.road.mesh);
    this.pool = { lines: [], edges: [], soft: [], shadows: [], sprites: [] };
    this.used = {};
    this.pathMat = this.decal(BLUE, 120 / 255, 4);
    this.path = new Strip(this.pathMat, 260, 4); this.scene.add(this.path.mesh);
    this.pathGlow = new Strip(new THREE.ShaderMaterial({ vertexShader: SOFT_VS, fragmentShader: SOFT_RIBBON_FS, transparent: true, depthWrite: false,
      side: THREE.DoubleSide, uniforms: { uColor: { value: new THREE.Vector3(...BLUE.map((c) => c / 255)) }, uOpacity: { value: 0.66 } } }), 260, 5);
    this.scene.add(this.pathGlow.mesh);

    // vehicle templates from models_kenney.json
    this.templates = {}; this.lengths = {}; this.widths = {}; this.lamps = {};
    for (const [name, m] of Object.entries(models)) {
      const groups = {};
      for (const [g, d] of Object.entries(m.groups)) {
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(d.v, 3));
        geo.setAttribute('normal', new THREE.Float32BufferAttribute(d.n, 3));
        geo.setIndex(d.i);
        groups[g] = geo;
      }
      this.templates[name] = groups; this.lengths[name] = m.L; this.widths[name] = m.W; this.lamps[name] = m.lamps || {};
    }
    // Carrot web 3D view: the own car is a generic SUV (models_kenney.json has no separate "ego" model).
    if (!this.templates.ego && this.templates.suv) {
      this.templates.ego = this.templates.suv; this.lengths.ego = this.lengths.suv; this.widths.ego = this.widths.suv; this.lamps.ego = this.lamps.suv;
    }
    this.lengths.moto = 2.0; this.widths.moto = 0.7; this.lengths.ped = 0.5; this.widths.ped = 0.5;
    this.std = (c, metal, rough, extra = {}) => new THREE.MeshStandardMaterial({ color: c, metalness: metal, roughness: rough, ...extra });
    this.M = {
      glass: () => this.std(lin(0.09, 0.105, 0.125), 0.55, 0.12), trim: () => this.std(lin(0.07, 0.075, 0.08), 0.1, 0.7),
      lower: () => this.std(lin(0.13, 0.135, 0.145), 0.1, 0.55), chrome: () => this.std(lin(0.62, 0.64, 0.67), 0.85, 0.22),
      tire: () => this.std(lin(0.03, 0.03, 0.035), 0.0, 0.9), rim: () => this.std(lin(0.55, 0.57, 0.60), 0.9, 0.25),
      cargo: () => this.std(COL.cargo, 0.05, 0.6),
      lampOff: () => this.std(lin(0.30, 0.03, 0.03), 0.3, 0.2), headOff: () => this.std(lin(0.6, 0.62, 0.64), 0.8, 0.1),
      person: () => this.std(lin(0.86, 0.84, 0.80), 0.0, 0.55), personD: () => this.std(lin(0.40, 0.42, 0.46), 0.0, 0.6),
    };
    this.LAMP = { brake: lin(1.0, 0.08, 0.05), turn: lin(1.0, 0.55, 0.0), head: lin(1.0, 0.97, 0.88) };
    this.objs = new Map();
    this.ego = this.makeVehicle('ego'); this.ego.root.position.copy(v3(-3.6, 0, 0)); this.scene.add(this.ego.root);
    this.egoShadow = this.softQuad([0, 0, 0], 0.5, 1); this.scene.add(this.egoShadow);
    this.egoContact = this.softQuad([0, 0, 0], 0.42, 1); this.scene.add(this.egoContact);
  }

  resize(w, h, dpr) {
    this.renderer.setPixelRatio(dpr); this.renderer.setSize(w, h, false);
    const landscape = w >= h;
    const f = landscape ? Math.min(0.66 * w, 1.1733 * h) : 0.82 * w;
    const cx = w / 2, cy = (landscape ? 0.30 : 0.34) * h, n = this.camera.near, far = this.camera.far;
    this.camera.projectionMatrix.makePerspective(-cx * n / f, (w - cx) * n / f, cy * n / f, -(h - cy) * n / f, n, far);
    this.camera.projectionMatrixInverse.copy(this.camera.projectionMatrix).invert();
    this.uniformsCommon.uCy.value = cy * dpr; this.uniformsCommon.uH.value = h * dpr;
    this.size = { w, h, f, cx, cy };
  }

  softQuad(rgb, opacity, order, additive = false) {
    const m = new THREE.MeshBasicMaterial({ alphaMap: this.softTex, color: srgb(...rgb), transparent: true, opacity, depthWrite: false,
      blending: additive ? THREE.AdditiveBlending : THREE.NormalBlending, fog: false });
    const q = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), m);
    q.rotation.x = -Math.PI / 2; q.renderOrder = order; q.frustumCulled = false;
    return q;
  }
  // footprint rectangle [x0, x0+L] x [y0 +- W/2] widened like v3's blurred footprints
  placeSoft(q, x0, y0, L, W, z) {
    const k = 128 / (128 - 2 * 34 + 30);                       // quad size per unit of the soft rect's half-power width
    q.scale.set((W + 0.7) * k, (L + 0.8) * k, 1);
    q.position.copy(v3(x0 + L / 2, y0, z));
  }

  makeVehicle(name) {
    const root = new THREE.Group(); const mats = []; const nodes = {};
    if (name === 'ped' || name === 'moto') return this.makeVru(name);
    for (const [g, geo] of Object.entries(this.templates[name])) {
      let m;
      if (g === 'body') m = paintMaterial(name === 'ego' ? COL.ego : (name === 'truck' ? COL.truck : COL.body));
      else if (g.startsWith('lamp') || g === 'chmsl') m = this.M.lampOff();
      else if (g.startsWith('head')) m = this.M.headOff();
      else m = (this.M[g] || this.M.trim)();
      const mesh = new THREE.Mesh(geo, m); mesh.name = g; root.add(mesh); mats.push(m); nodes[g] = mesh;
    }
    return { root, mats, nodes, name, glows: [] };
  }
  makeVru(name) {
    const root = new THREE.Group(); const mats = []; const nodes = {};
    const cap = (r, h, mat) => { mats.push(mat); return new THREE.Mesh(new THREE.CapsuleGeometry(r, h, 4, 10), mat); };
    const person = (seated) => {
      const p = new THREE.Group(); const b = seated ? 0.62 : 0;
      if (!seated) for (const [nm, yy] of [['legL', 0.11], ['legR', -0.11]]) {
        const hip = new THREE.Group(); hip.position.copy(v3(0, yy, 0.90));
        const leg = cap(0.085, 0.92, this.M.personD()); leg.position.set(0, -0.45, 0); hip.add(leg); p.add(hip); nodes[nm] = hip;
      }
      const torso = cap(0.17, 0.66, this.M.person()); torso.position.copy(v3(0, 0, b + 1.17)); p.add(torso);
      for (const [nm, yy] of [['armL', 0.25], ['armR', -0.25]]) {
        const sh = new THREE.Group(); sh.position.copy(v3(0, yy, b + 1.40));
        const arm = cap(0.055, 0.62, this.M.person()); arm.position.set(0, -0.29, 0); sh.add(arm); p.add(sh); nodes[nm] = sh;
      }
      const hm = this.M.person(); mats.push(hm);
      const head = new THREE.Mesh(new THREE.SphereGeometry(0.125, 16, 12), hm); head.position.copy(v3(0, 0, b + 1.62)); p.add(head);
      return p;
    };
    if (name === 'ped') root.add(person(false));
    else {
      const bm = this.std(lin(0.12, 0.13, 0.15), 0.5, 0.3); mats.push(bm);
      const body = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.42, 1.25), bm); body.position.copy(v3(1.0, 0, 0.62)); root.add(body);
      for (const xx of [0.35, 1.65]) {
        const tm = this.M.tire(); mats.push(tm);
        const w = new THREE.Mesh(new THREE.TorusGeometry(0.26, 0.075, 10, 24), tm); w.rotation.y = Math.PI / 2; w.position.copy(v3(xx, 0, 0.33)); root.add(w);
      }
      const rider = person(true); rider.position.copy(v3(0.95, 0, 0)); root.add(rider);
    }
    return { root, mats, nodes, name, glows: [] };
  }

  glowSprite() {
    const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.radTex, color: 0xffffff, transparent: true, depthWrite: false, depthTest: false,
      blending: THREE.AdditiveBlending, fog: false }));
    s.renderOrder = 20; this.scene.add(s); return s;
  }

  setLamp(v, nm, kind) {
    const mesh = v.nodes[nm]; if (!mesh) return;
    const m = mesh.material;
    if (kind) { m.emissive = this.LAMP[kind]; m.emissiveIntensity = 3.0; m.color.setRGB(0, 0, 0); }
    else { m.emissiveIntensity = 0; m.color.copy(nm.startsWith('head') ? lin(0.6, 0.62, 0.64) : lin(0.30, 0.03, 0.03)); }
  }

  // world-space lamp glow sprite (ellipse like v3's blurred 2D glows)
  lampGlow(i, p, kind, alpha, gs) {
    while (this.pool.sprites.length <= i) this.pool.sprites.push(this.glowSprite());
    const s = this.pool.sprites[i]; s.visible = true;
    s.material.color.copy(kind === 'head' ? srgb(255, 248, 226) : kind === 'turn' ? srgb(255, 166, 0) : srgb(255, 44, 40));
    s.material.opacity = 0.85 * alpha;
    s.position.copy(v3(p[0], p[1], p[2])); s.scale.set(1.5 * gs, 0.9 * gs, 1);
  }

  pooledSoft(kind, i, rgb, opacity, order, additive) {
    const arr = this.pool[kind];
    while (arr.length <= i) { const q = this.softQuad(rgb, opacity, order, additive); this.scene.add(q); arr.push(q); }
    const q = arr[i]; q.visible = true; q.material.color.copy(srgb(...rgb)); q.material.opacity = opacity; return q;
  }
  pooledStrip(kind, i, order) {
    const arr = this.pool[kind];
    while (arr.length <= i) { const s = new Strip(this.decal([255, 255, 255], 1, order), 160, order); this.scene.add(s.mesh); arr.push(s); }
    return arr[i];
  }

  // frame: { road:{l,r}, lines:[{pts,typ,prob}], edges:[pts], path:{pts,on}, bsm:[l,r], objs:[...], ego:{br,tl,tr}, dist }
  update(fr) {
    const counts = { lines: 0, edges: 0, soft: 0, shadows: 0, sprites: 0 };
    if (fr.road) this.road.set(fr.road.l, fr.road.r, 0); else this.road.mesh.visible = false;
    // lane markings
    const addLine = (pts, w, rgb, a) => {
      const r = ribbon(pts, w); if (!r) return;
      const s = this.pooledStrip('lines', counts.lines++, 3);
      s.mesh.material.uniforms.uColor.value.set(rgb[0] / 255, rgb[1] / 255, rgb[2] / 255); s.mesh.material.uniforms.uOpacity.value = a;
      s.set(r[0], r[1], 0.015);
    };
    for (const ln of fr.lines) {
      if (!ln.pts || ln.prob < 0.2) continue;
      const a = Math.min(1, 0.25 + ln.prob);
      if (ln.typ === 6) addLine(ln.pts, 0.30, [120, 126, 132], a * 0.7);
      else if (ln.typ === 2) for (const seg of dashes(ln.pts, fr.dist)) addLine(seg, 0.14, LINE, a);
      else if (ln.typ === 1) addLine(ln.pts, 0.14, LINE, a);
      else addLine(ln.pts, 0.08, [150, 156, 162], a * 0.5);
    }
    for (const e of fr.edges) addLine(e, 0.20, [110, 116, 122], 110 / 255);
    // path
    if (fr.path && fr.path.pts) {
      const pts = fr.path.pts.filter((p) => p[0] < 60);
      const r = ribbon(pts, 1.8);
      if (r) {
        this.path.set(r[0], r[1], 0.02);
        this.pathMat.uniforms.uColor.value.set(...(fr.path.on ? BLUE : [150, 160, 170]).map((c) => c / 255));
        this.pathMat.uniforms.uOpacity.value = fr.path.on ? 120 / 255 : 26 / 255;
        const g = ribbon(pts, 3.6);
        if (fr.path.on && g) this.pathGlow.set(g[0], g[1], 0.022); else this.pathGlow.mesh.visible = false;
      } else { this.path.mesh.visible = false; this.pathGlow.mesh.visible = false; }
    } else { this.path.mesh.visible = false; this.pathGlow.mesh.visible = false; }
    // BSM zones
    (fr.bsm || []).forEach((on, i) => {
      if (!on) return;
      const q = this.pooledSoft('soft', counts.soft++, [255, 40, 40], 0.6, 6, false);
      this.placeSoft(q, -9, i === 0 ? 3.5 : -3.5, 13, 2.8, 0.025);
    });
    // objects
    const seen = new Set(); let si = 0;
    for (const o of fr.objs) {
      seen.add(o.k);
      if (o.m !== 'ped' && o.m !== 'moto' && !this.templates[o.m]) o.m = this.templates.sedan ? 'sedan' : Object.keys(this.templates)[0];
      let v = this.objs.get(o.k);
      if (!v || v.name !== o.m) { if (v) this.disposeVehicle(v); v = this.makeVehicle(o.m); this.scene.add(v.root); this.objs.set(o.k, v); }
      const L = this.lengths[o.m] ?? 4.8, W = this.widths[o.m] ?? 1.9;
      if (o.f) { v.root.position.copy(v3(o.x + L, o.y, 0)); v.root.rotation.y = Math.PI; }
      else { v.root.position.copy(v3(o.x, o.y, 0)); v.root.rotation.y = 0; }
      for (const m of v.mats) { m.transparent = o.a < 0.999; m.opacity = o.a; }
      const sh = this.pooledSoft('shadows', counts.shadows++, [0, 0, 0], (o.m === 'ped' ? 0.45 : 0.5) * o.a, 1, false);
      if (o.m === 'ped') this.placeSoft(sh, o.x - 0.25, o.y, 0.5, 0.5, 0.005);
      else {
        this.placeSoft(sh, o.x, o.y, L, W, 0.005);
        const cs = this.pooledSoft('shadows', counts.shadows++, [0, 0, 0], 0.42 * o.a, 1, false);
        cs.scale.set(W * 0.9, L * 0.82, 1); cs.position.copy(v3(o.x + L / 2, o.y, 0.006));
      }
      if (o.lead) { const q = this.pooledSoft('soft', counts.soft++, BLUE, 0.6 * o.a, 6, false); this.placeSoft(q, o.x - 0.6, o.y, L + 1.2, W + 0.5, 0.025); }
      if (o.m === 'ped') {
        const ph = o.ph || 0, n = v.nodes;
        if (n.legL) n.legL.rotation.x = 0.45 * Math.sin(ph); if (n.legR) n.legR.rotation.x = -0.45 * Math.sin(ph);
        if (n.armL) n.armL.rotation.x = -0.35 * Math.sin(ph); if (n.armR) n.armR.rotation.x = 0.35 * Math.sin(ph);
        continue;
      }
      if (o.m === 'moto') continue;
      v.nodes.body.material.color.copy(o.lead ? COL.lead : (o.r ? COL.radar : (o.m === 'truck' ? COL.truck : paintFor(o.uid))));
      const lm = this.lamps[o.m] || {};
      const at = (nm) => { const [lx, ly, lz] = lm[nm]; return o.f ? [o.x + L - lx, o.y - ly, lz] : [o.x + lx, o.y + ly, lz]; };
      const kindL = o.tl ? 'turn' : (o.br ? 'brake' : null), kindR = o.tr ? 'turn' : (o.br ? 'brake' : null);
      this.setLamp(v, 'lampL', kindL); this.setLamp(v, 'lampR', kindR);
      this.setLamp(v, 'headL', o.f ? 'head' : null); this.setLamp(v, 'headR', o.f ? 'head' : null);
      if (o.f && lm.headL) { this.lampGlow(si++, at('headL'), 'head', o.a, 0.7); this.lampGlow(si++, at('headR'), 'head', o.a, 0.7); }
      else {
        if (kindL && lm.lampL) this.lampGlow(si++, at('lampL'), kindL, o.a, 1.0);
        if (kindR && lm.lampR) this.lampGlow(si++, at('lampR'), kindR, o.a, 1.0);
      }
    }
    for (const [k, v] of this.objs) if (!seen.has(k)) { this.disposeVehicle(v); this.objs.delete(k); }
    // ego
    const e = fr.ego || {}, lm = this.lamps.ego || {};
    const ekL = e.tl ? 'turn' : (e.br ? 'brake' : null), ekR = e.tr ? 'turn' : (e.br ? 'brake' : null);
    this.setLamp(this.ego, 'lampL', ekL); this.setLamp(this.ego, 'lampR', ekR);
    if (ekL && lm.lampL) this.lampGlow(si++, [-3.6 + lm.lampL[0], lm.lampL[1], lm.lampL[2]], ekL, 1, 0.45);
    if (ekR && lm.lampR) this.lampGlow(si++, [-3.6 + lm.lampR[0], lm.lampR[1], lm.lampR[2]], ekR, 1, 0.45);
    this.egoShadow.material.opacity = 0.56; this.placeSoft(this.egoShadow, -3.6, 0, this.lengths.ego, this.widths.ego, 0.005);
    this.egoContact.material.opacity = 0.42;
    this.egoContact.scale.set(this.widths.ego * 0.9, this.lengths.ego * 0.82, 1);
    this.egoContact.position.copy(v3(-3.6 + this.lengths.ego / 2, 0, 0.006));
    this.uniformsCommon.uDist.value = fr.dist || 0;
    this.stepCamera();
    counts.sprites = si;
    for (const kind of ['lines', 'soft', 'shadows']) for (let i = counts[kind]; i < this.pool[kind].length; i++) {
      const x = this.pool[kind][i]; (x.mesh || x).visible = false;
    }
    for (let i = si; i < this.pool.sprites.length; i++) this.pool.sprites[i].visible = false;
    this.renderer.render(this.scene, this.camera);
  }

  disposeVehicle(v) {
    this.scene.remove(v.root);
    for (const m of v.mats) m.dispose();
  }

  // ---- Carrot web 3D view camera: zoom (pinch / wheel), look around (drag), reset (double tap) ----
  applyCamera() {
    const r = this.rig;
    const eye = rigEye(r);
    this.camera.position.copy(v3(eye[0], eye[1], eye[2]));
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(v3(RIG_PIVOT[0], RIG_PIVOT[1], RIG_PIVOT[2]));
    this.camera.updateMatrixWorld();
    // Zoomed out, keep the far road out of the fog.
    const k = Math.max(1, r.zoom);
    this.scene.fog.near = 45 * k; this.scene.fog.far = 165 * k;
  }
  // Ease towards the target view once per frame.
  stepCamera() {
    const r = this.rig;
    const before = [r.zoom, r.yaw, r.pitch];
    r.zoom += (r.target.zoom - r.zoom) * 0.35;
    r.yaw += (r.target.yaw - r.yaw) * 0.35;
    r.pitch += (r.target.pitch - r.pitch) * 0.35;
    if (before.some((x, i) => Math.abs(x - [r.zoom, r.yaw, r.pitch][i]) > 1e-5)) this.applyCamera();
  }
  zoomBy(factor) { this.rig.target.zoom = clampRig('zoom', this.rig.target.zoom / factor); }
  orbitBy(dYawDeg, dPitchDeg) {
    this.rig.target.yaw = clampRig('yaw', this.rig.target.yaw + dYawDeg);
    this.rig.target.pitch = clampRig('pitch', this.rig.target.pitch + dPitchDeg);
  }
  resetView() { this.rig.target = { ...RIG_DEFAULT }; }
  isDefaultView() {
    const t = this.rig.target;
    return Math.abs(t.zoom - RIG_DEFAULT.zoom) < 1e-3 && Math.abs(t.yaw - RIG_DEFAULT.yaw) < 0.05 && Math.abs(t.pitch - RIG_DEFAULT.pitch) < 0.05;
  }
  // World point (x forward, y left, z up) -> CSS px in the canvas, or null behind the camera / off screen.
  projectToScreen(x, y, z) {
    if (!this.size) return null;
    const p = v3(x, y, z).project(this.camera);
    if (!(p.z > -1 && p.z < 1)) return null;
    const sx = (p.x + 1) / 2 * this.size.w, sy = (1 - p.y) / 2 * this.size.h;
    if (sx < 0 || sx > this.size.w || sy < 0 || sy > this.size.h) return null;
    return { x: sx, y: sy };
  }

  // Carrot web: free the GPU context when the drive tab stops this view (not in the app, which never stops).
  dispose() {
    this.scene.traverse((node) => {
      node.geometry?.dispose?.();
      const mats = Array.isArray(node.material) ? node.material : (node.material ? [node.material] : []);
      for (const m of mats) { m.map?.dispose?.(); m.alphaMap?.dispose?.(); m.dispose?.(); }
    });
    this.scene.environment?.dispose?.();
    this.softTex?.dispose?.(); this.radTex?.dispose?.();
    this.renderer.dispose();
    this.renderer.forceContextLoss?.();
  }
}
