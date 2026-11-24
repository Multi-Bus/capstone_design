// 편대 제어 화면: 서버 상태(/ws)를 3D로 그리고 버튼과 단축키 명령을 보낸다.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { CSS2DRenderer, CSS2DObject } from "three/addons/renderers/CSS2DRenderer.js";

const css = getComputedStyle(document.documentElement);
const color = (name) => new THREE.Color(css.getPropertyValue(name).trim());
const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)").matches;
const FORMATION = { line: "일렬 편대", triangle: "삼각 편대", circle: "원 편대" };
const PHASE = { ground: "지상", takeoff: "이륙", hover: "호버", landing: "착륙" };

// 장면
const stage = document.getElementById("stage");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
stage.appendChild(renderer.domElement);
const labels = new CSS2DRenderer();
labels.domElement.style.cssText = "position:absolute;inset:0;pointer-events:none";
stage.appendChild(labels.domElement);

const scene = new THREE.Scene();
scene.background = color("--bg");

// 카메라: 오른쪽 앞에서 비스듬히 내려다본다
const camera = new THREE.PerspectiveCamera(38, 1, 0.05, 200);
const VIEW = { azimuth: THREE.MathUtils.degToRad(28), elevation: THREE.MathUtils.degToRad(36) };
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.minDistance = 3;
controls.maxDistance = 26;
controls.maxPolarAngle = THREE.MathUtils.degToRad(84);
controls.autoRotateSpeed = 0.35;
controls.target.set(0, 0.6, 0);
function placeCamera(dist) {
  const { azimuth: a, elevation: e } = VIEW;
  camera.position.set(Math.sin(a) * Math.cos(e), Math.sin(e), Math.cos(a) * Math.cos(e))
    .multiplyScalar(dist).add(controls.target);
}
placeCamera(7);
let lastTouch = -Infinity; // 사용자가 마지막으로 카메라를 만진 시각
let down = null; // 포인터를 누른 위치
// 드래그나 휠로 카메라를 만졌을 때만 자동 구도를 6초 멈춘다. 바닥 클릭은 해당 없음.
function touched() { lastTouch = performance.now(); controls.autoRotate = false; }
renderer.domElement.addEventListener("pointermove", (e) => { if (e.buttons && down && Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) touched(); });
renderer.domElement.addEventListener("wheel", touched, { passive: true });

scene.add(new THREE.HemisphereLight(0xdbe6ff, 0x050b1f, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(4, 9, 6);
scene.add(sun);

// 그리드 무대 = 실제 방(15 m). 1 m 굵은 칸 + 0.25 m 잔칸, 바닥과 뒷벽
const ROOM = 15;
function grid(divisions, opacity) {
  const g = new THREE.GridHelper(ROOM, divisions, color("--grid"), color("--grid"));
  g.material.transparent = true;
  g.material.opacity = opacity;
  g.material.depthWrite = false;
  return g;
}
const floor = new THREE.Group();
floor.add(grid(60, 0.14), grid(15, 0.55));
scene.add(floor);
const wall = new THREE.Group();
wall.add(grid(60, 0.08), grid(15, 0.32));
wall.rotation.x = Math.PI / 2;
wall.position.set(0, ROOM / 2, -ROOM / 2);
scene.add(wall);
{
  const h = ROOM / 2;
  const edge = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(
    [[-h, -h], [h, -h], [h, h], [-h, h]].map(([x, z]) => new THREE.Vector3(x, 0.003, z))),
    new THREE.LineBasicMaterial({ color: color("--accent") }));
  scene.add(edge);
}

// 쿼드콥터
const M = {
  body: new THREE.MeshStandardMaterial({ color: 0xdfe8fa, metalness: 0.25, roughness: 0.45 }),
  frame: new THREE.MeshStandardMaterial({ color: 0x3a4a6e, metalness: 0.4, roughness: 0.5 }),
  motor: new THREE.MeshStandardMaterial({ color: 0x1b2440, metalness: 0.6, roughness: 0.35 }),
  disk: new THREE.MeshBasicMaterial({ color: color("--grid"), transparent: true, opacity: 0.22, depthWrite: false }),
  blade: new THREE.MeshStandardMaterial({ color: 0xc7d6f5, roughness: 0.6 }),
  front: new THREE.MeshBasicMaterial({ color: color("--accent") }),
  ring: new THREE.MeshBasicMaterial({ color: color("--grid"), transparent: true, opacity: 0.7, side: THREE.DoubleSide }),
  tether: new THREE.LineBasicMaterial({ color: color("--grid"), transparent: true, opacity: 0.35 }),
};
const G = {
  body: new THREE.CylinderGeometry(0.07, 0.085, 0.045, 8),
  canopy: new THREE.SphereGeometry(0.06, 20, 10, 0, Math.PI * 2, 0, Math.PI / 2),
  arm: new THREE.CylinderGeometry(0.011, 0.011, 0.44, 10),
  motor: new THREE.CylinderGeometry(0.022, 0.022, 0.03, 16),
  disk: new THREE.CylinderGeometry(0.085, 0.085, 0.002, 32),
  blade: new THREE.BoxGeometry(0.17, 0.004, 0.016),
  led: new THREE.SphereGeometry(0.012, 10, 8),
  ring: new THREE.RingGeometry(0.13, 0.155, 40),
  tether: new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, -1, 0)]),
};

function makeDrone(id) {
  const g = new THREE.Group();
  g.add(new THREE.Mesh(G.body, M.body));
  const canopy = new THREE.Mesh(G.canopy, M.body);
  canopy.position.y = 0.02;
  canopy.scale.set(1, 0.55, 1);
  g.add(canopy);
  for (const a of [Math.PI / 4, -Math.PI / 4]) {
    const arm = new THREE.Mesh(G.arm, M.frame);
    arm.rotation.set(0, a, Math.PI / 2);
    g.add(arm);
  }
  g.userData.rotors = [];
  for (const [x, z] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
    const motor = new THREE.Mesh(G.motor, M.motor);
    motor.position.set(x * 0.155, 0.01, z * 0.155);
    g.add(motor);
    const r = new THREE.Group();
    r.position.set(x * 0.155, 0.03, z * 0.155);
    r.add(new THREE.Mesh(G.disk, M.disk), new THREE.Mesh(G.blade, M.blade));
    r.userData.dir = x * z;
    g.add(r);
    g.userData.rotors.push(r);
  }
  const led = new THREE.Mesh(G.led, M.front); // 앞쪽 표시
  led.position.set(0, 0, -0.09);
  g.add(led);
  const tether = new THREE.Line(G.tether, M.tether); // 바닥까지 수직선: 높이를 보여준다
  g.add(tether);

  const tag = document.createElement("div");
  tag.className = "tag";
  tag.textContent = String(id).padStart(2, "0");
  const label = new CSS2DObject(tag);
  label.position.set(0, 0.2, 0);
  g.add(label);

  const ring = new THREE.Mesh(G.ring, M.ring);
  ring.rotation.x = -Math.PI / 2;
  Object.assign(g.userData, { target: new THREE.Vector3(), flying: false, tether, ring, spin: 0 });
  scene.add(g, ring);
  return g;
}

// 기둥, A* 경로, 목표 표식
const PILLAR_H = 1.5;
const pillarMat = {
  fixed: new THREE.MeshStandardMaterial({ color: 0x1a3478, metalness: 0.2, roughness: 0.6 }),
  placed: new THREE.MeshStandardMaterial({ color: 0x3f74e0, metalness: 0.2, roughness: 0.5 }),
};
const pillarCap = new THREE.LineBasicMaterial({ color: color("--grid") });
const pillars = new Map();
function syncPillars(list) {
  const keep = new Set();
  for (const o of list) {
    const key = `${o.x},${o.y}`;
    keep.add(key);
    if (pillars.has(key)) continue;
    const g = new THREE.Group();
    const body = new THREE.Mesh(new THREE.CylinderGeometry(o.r, o.r, PILLAR_H, 40), o.fixed ? pillarMat.fixed : pillarMat.placed);
    body.position.y = PILLAR_H / 2;
    const rim = new THREE.LineLoop(new THREE.EdgesGeometry(new THREE.CircleGeometry(o.r, 40)), pillarCap);
    rim.rotation.x = -Math.PI / 2;
    rim.position.y = PILLAR_H + 0.002;
    g.add(body, rim);
    g.position.set(o.x, 0, -o.y);
    scene.add(g);
    pillars.set(key, g);
  }
  for (const [key, g] of pillars) if (!keep.has(key)) { scene.remove(g); pillars.delete(key); }
}

const pathLine = new THREE.Line(new THREE.BufferGeometry(),
  new THREE.LineDashedMaterial({ color: color("--accent"), dashSize: 0.25, gapSize: 0.15 }));
pathLine.position.y = 0.01;
pathLine.visible = false;
scene.add(pathLine);
const goalMark = new THREE.Group();
{
  const m = new THREE.MeshBasicMaterial({ color: color("--accent"), side: THREE.DoubleSide });
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.28, 0.33, 48), m);
  const dot = new THREE.Mesh(new THREE.CircleGeometry(0.07, 24), m);
  for (const x of [ring, dot]) { x.rotation.x = -Math.PI / 2; goalMark.add(x); }
  goalMark.position.y = 0.012;
  goalMark.visible = false;
  scene.add(goalMark);
}
let goalAt = null; // 카메라가 목표까지 담도록 기억
function syncPath(state) {
  goalAt = state.goal ? new THREE.Vector3(state.goal[0], 0, -state.goal[1]) : null;
  goalMark.visible = !!state.goal;
  if (state.goal) goalMark.position.set(state.goal[0], 0.012, -state.goal[1]);
  const pts = state.path ?? [];  // 첫 점은 지금 편대 중심, 나머지는 남은 웨이포인트
  pathLine.visible = pts.length > 1;
  if (pts.length > 1) {
    pathLine.geometry.dispose();
    pathLine.geometry = new THREE.BufferGeometry().setFromPoints(pts.map(([x, y]) => new THREE.Vector3(x, 0, -y)));
    pathLine.computeLineDistances();
  }
}

// 상태 반영. 시뮬 (x, y, z) -> three (x, z, -y)
const drones = new Map();
const LIFT = 0.03;
const ui = {
  formation: document.getElementById("formation"),
  summary: document.getElementById("summary"),
  fleet: document.getElementById("fleet-list"),
  link: document.getElementById("link"),
  linkText: document.getElementById("link-text"),
  lost: document.getElementById("lost"),
  buttons: document.querySelectorAll("#dock button"),
};

function summarize(list) {
  const n = {};
  for (const d of list) n[d.phase] = (n[d.phase] ?? 0) + 1;
  if (n.ground === list.length) return `${list.length}대 지상 대기`;
  if (n.hover === list.length) return `${list.length}대 호버`;
  return ["takeoff", "hover", "landing", "ground"].filter((p) => n[p]).map((p) => `${PHASE[p]} ${n[p]}`).join(", ");
}

function apply(state) {
  for (const d of state.drones) {
    let g = drones.get(d.id);
    const fresh = !g;
    if (fresh) drones.set(d.id, (g = makeDrone(d.id)));
    g.userData.target.set(d.x, d.z + LIFT, -d.y);
    if (fresh) g.position.copy(g.userData.target);
    g.userData.flying = d.phase !== "ground";
  }
  ui.formation.textContent = FORMATION[state.formation] ?? (state.drones.every((d) => d.phase === "ground") ? "대기" : "기본 대형");
  ui.summary.textContent = summarize(state.drones) + (state.goal ? ", A* 경로로 이동 중" : "");
  syncPillars(state.obstacles ?? []);
  syncPath(state);
  for (const b of document.querySelectorAll("[data-formation]")) {
    b.setAttribute("aria-pressed", String(b.dataset.formation === state.formation));
  }
  ui.fleet.replaceChildren(...state.drones.map((d) => {
    const li = document.createElement("li");
    li.dataset.phase = d.phase;
    li.innerHTML = `<span class="id">${String(d.id).padStart(2, "0")}</span><span class="phase">${PHASE[d.phase]}</span><span class="alt">${d.z.toFixed(2)} m</span>`;
    return li;
  }));
}

// 서버 연결
let ws = null;
function setLink(ok) {
  ui.link.classList.toggle("ok", ok);
  ui.linkText.textContent = ok ? "서버 연결됨" : "연결 끊김";
  for (const b of ui.buttons) b.disabled = !ok;
}
setLink(false);
ui.linkText.textContent = "연결 중";
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { setLink(true); ui.lost.hidden = true; };
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.drones) apply(m);
    else if (m.error) toast(m.error);
  };
  ws.onclose = () => {
    setLink(false);
    ui.lost.hidden = false;
    setTimeout(connect, 1000);
  };
}
connect();

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(msg));
}
for (const b of document.querySelectorAll("[data-formation]")) {
  b.onclick = () => send({ cmd: "formation", name: b.dataset.formation });
}
for (const b of document.querySelectorAll("[data-cmd]")) {
  b.onclick = () => send({ cmd: b.dataset.cmd });
}
// 알림
const toastEl = document.getElementById("toast");
let toastTimer = 0;
function toast(text) {
  toastEl.textContent = text;
  toastEl.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toastEl.hidden = true; }, 2600);
}

// 경로 지도 도구: 켠 상태에서 바닥을 클릭하면 목표 또는 장애물. 드래그는 카메라 회전.
let tool = null;
const toolButtons = document.querySelectorAll("[data-tool]");
function setTool(t) {
  tool = tool === t ? null : t;
  for (const b of toolButtons) b.setAttribute("aria-pressed", String(b.dataset.tool === tool));
  stage.classList.toggle("picking", !!tool);
}
for (const b of toolButtons) b.onclick = () => setTool(b.dataset.tool);
const ray = new THREE.Raycaster();
const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const hit = new THREE.Vector3();
renderer.domElement.addEventListener("pointerdown", (e) => { down = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener("pointerup", (e) => {
  if (!tool || !down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return;
  const r = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
  if (!ray.ray.intersectPlane(ground, hit)) return;
  const x = Math.round(hit.x * 10) / 10, y = Math.round(-hit.z * 10) / 10;
  send({ cmd: tool, x, y });
});

// 발표용 단축키: 1 2 3 편대, S 시작, X 중지, H 복귀, G 목표, O 장애물, C 장애물 지우기, Esc 도구 끄기
const KEYS = { 1: { cmd: "formation", name: "line" }, 2: { cmd: "formation", name: "triangle" },
  3: { cmd: "formation", name: "circle" }, s: { cmd: "start" }, x: { cmd: "stop" }, h: { cmd: "home" },
  c: { cmd: "clear_obstacles" } };
window.addEventListener("keydown", (e) => {
  if (e.ctrlKey || e.metaKey || e.altKey || e.repeat) return;
  const k = e.key.toLowerCase();
  if (k === "g") setTool("goal");
  else if (k === "o") setTool("obstacle");
  else if (k === "escape" && tool) setTool(tool);
  else if (KEYS[k]) send(KEYS[k]);
});

// 크기 맞춤
function resize() {
  const w = stage.clientWidth, h = stage.clientHeight;
  renderer.setSize(w, h);
  labels.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener("resize", resize);
resize();

// 그리기: 위치 보간, 프로펠러, 카메라가 편대 중심과 크기를 따라간다
const clock = new THREE.Clock();
const center = new THREE.Vector3();
const offset = new THREE.Vector3();
renderer.setAnimationLoop(() => {
  const dt = Math.min(clock.getDelta(), 0.1);
  const k = 1 - Math.exp(-dt * 12);
  if (drones.size) {
    center.set(0, 0, 0);
    let extent = 0;
    for (const g of drones.values()) {
      const u = g.userData;
      g.position.lerp(u.target, k);
      const h = Math.max(g.position.y, 0.001);
      u.tether.scale.y = h;
      u.tether.visible = h > 0.08;
      u.ring.position.set(g.position.x, 0.002, g.position.z);
      u.spin += ((u.flying ? 1 : 0) - u.spin) * k * 0.5; // 프로펠러가 서서히 돌고 서서히 멈춘다
      for (const r of u.rotors) r.rotation.y += r.userData.dir * dt * 45 * u.spin;
      center.add(g.position);
    }
    center.divideScalar(drones.size);
    for (const g of drones.values()) extent = Math.max(extent, g.position.distanceTo(center));
    if (goalAt) {  // 이동 중에는 편대와 목표 사이를 함께 담는다
      const span = Math.hypot(goalAt.x - center.x, goalAt.z - center.z);
      center.lerp(new THREE.Vector3(goalAt.x, center.y, goalAt.z), 0.5);
      extent = Math.max(extent, span / 2 + 1);
    }

    const idle = performance.now() - lastTouch > 6000;
    const delta = offset.copy(center).sub(controls.target).multiplyScalar(1 - Math.exp(-dt * 2));
    controls.target.add(delta);
    camera.position.add(delta);
    if (idle) {
      controls.autoRotate = !reduceMotion;
      // 편대 크기에 맞춰 거리를 천천히 맞춘다
      const want = THREE.MathUtils.clamp(4.2 + extent * 1.8, 5.5, 18);
      offset.copy(camera.position).sub(controls.target);
      offset.setLength(THREE.MathUtils.lerp(offset.length(), want, 1 - Math.exp(-dt * 0.8)));
      camera.position.copy(controls.target).add(offset);
    }
  }
  controls.update(dt);
  renderer.render(scene, camera);
  labels.render(scene, camera);
});
