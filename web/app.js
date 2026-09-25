// UMC-500 twin viewer: poses the Haas model from jog sliders, a simulated program, or live data.
// World frame = machine frame (Z up, mm). Meshes are stored at the CAD pose, so every link group's
// matrix is just its joint motion relative to that pose -- the same math as umc_twin/kinematics.py.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const JOINTS = ['X', 'Y', 'Z', 'B', 'C'];
const GUARDING = new Set(['enclosure', 'tc_enclosure', 'auto_door', 'auto_window', 'control_panel']);
const $ = (id) => document.getElementById(id);

const S = {
  cfg: null, parts: null, q0: [0, 0, 0, 0, 0], q: [0, 0, 0, 0, 0], options: {},
  result: null, time: 0, playing: false, speed: 1, source: 'manual', server: false,
  live: null, listingStart: -1, lastLine: -1, currentTool: null, highlight: new Set(),
};

// ---------------------------------------------------------------- scene
const canvas = $('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(35, 1, 5, 60000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x6b7280, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(2000, -3000, 5000);
scene.add(sun);
const fill = new THREE.DirectionalLight(0xffffff, 0.6);
fill.position.set(-3000, 2000, 1500);
scene.add(fill);

const links = { base: new THREE.Group() };
scene.add(links.base);
const partObjs = {};      // id -> Object3D
const partMats = {};      // id -> [materials]
let floor;

const CAMS = {
  iso: [[3300, -4300, 2600], [-150, 250, -50]],
  front: [[-150, -5200, 300], [-150, 250, -50]],
  right: [[5200, 250, 300], [-150, 250, -50]],
  top: [[-150, 250, 6500], [-150, 250, 0]],
  table: [[900, -1150, 650], [0, 0, 0]],
};
function setCam(name) {
  const [p, t] = CAMS[name];
  camera.position.set(...p);
  controls.target.set(...t);
  controls.update();
}

function resize() {
  const r = canvas.parentElement.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(r.height, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(canvas.parentElement);

// ---------------------------------------------------------------- kinematics
function jointMatrix(name, value, q0) {
  const j = S.cfg.joints[name];
  const d = value - q0;
  const m = new THREE.Matrix4();
  const axis = new THREE.Vector3(...j.axis).normalize();
  if (j.type === 'rotary') {
    const o = new THREE.Vector3(...(j.origin || [0, 0, 0]));
    m.makeTranslation(o.x, o.y, o.z)
      .multiply(new THREE.Matrix4().makeRotationAxis(axis, THREE.MathUtils.degToRad(d)))
      .multiply(new THREE.Matrix4().makeTranslation(-o.x, -o.y, -o.z));
  } else {
    m.makeTranslation(axis.x * d, axis.y * d, axis.z * d);
  }
  return m;
}

function applyPose(q) {
  S.q = q.slice();
  JOINTS.forEach((n, i) => {
    const g = links[n];
    g.matrix.copy(jointMatrix(n, q[i], S.q0[i]));
    g.matrixWorldNeedsUpdate = true;
  });
  updateDRO();
}

function toolTipInTable(len) {
  // tip in world -> table (C link) frame
  scene.updateMatrixWorld(true);
  const g = S.cfg.spindle.gauge_point;
  const d = S.cfg.spindle.direction;
  const tipLocal = new THREE.Vector3(g[0] + d[0] * len, g[1] + d[1] * len, g[2] + d[2] * len);
  const world = tipLocal.applyMatrix4(links[S.cfg.spindle.link].matrixWorld);
  return world.applyMatrix4(links[S.cfg.table.link].matrixWorld.clone().invert());
}

// ---------------------------------------------------------------- machine model
async function loadMachine() {
  const manifest = await (await fetch('assets/machine.json')).json();
  S.cfg = manifest.config;
  S.parts = manifest.parts;
  S.q0 = JOINTS.map((n) => S.cfg.cad.pose[n]);
  S.q = S.q0.slice();
  for (const [k, v] of Object.entries(S.cfg.options)) S.options[k] = v.default;

  for (const n of JOINTS) {
    links[n] = new THREE.Group();
    links[n].matrixAutoUpdate = false;
  }
  for (const n of JOINTS) links[S.cfg.joints[n].parent].add(links[n]);

  const loader = new GLTFLoader();
  const todo = S.cfg.parts.filter((p) => S.parts[p.id]);
  let done = 0;
  $('load-count').textContent = `0 / ${todo.length}`;
  await Promise.all(todo.map(async (p) => {
    const gltf = await loader.loadAsync(`assets/${S.parts[p.id].file}`);
    const opacity = p.opacity ?? 1;
    const mat = new THREE.MeshStandardMaterial({
      color: p.color, metalness: 0.35, roughness: 0.55,
      transparent: opacity < 1, opacity, depthWrite: opacity >= 1,
      side: opacity < 1 ? THREE.DoubleSide : THREE.FrontSide,
    });
    partMats[p.id] = [mat];
    gltf.scene.traverse((o) => { if (o.isMesh) { o.material = mat; o.renderOrder = opacity < 1 ? 2 : 0; } });
    gltf.scene.userData.part = p;
    partObjs[p.id] = gltf.scene;
    links[p.link].add(gltf.scene);
    $('load-count').textContent = `${++done} / ${todo.length}`;
  }));
  $('loading').classList.add('hidden');

  floor = new THREE.GridHelper(6000, 30, 0x8a949e, 0xb8c0c8);
  floor.rotation.x = Math.PI / 2;
  floor.position.z = Math.min(...Object.values(S.parts).map((p) => p.bbox_mm[0][2])) - 1;
  floor.material.transparent = true;
  floor.material.opacity = 0.5;
  scene.add(floor);

  buildTool({ length: Number($('jog-tool').value), diameter: 10, holder_diameter: 50, holder_length: 45 });
  buildOptionsUI();
  refreshVisibility();
  buildJogUI();
  applyPose(S.q);
}

function refreshVisibility() {
  const guarding = $('chk-guarding').checked;
  for (const p of S.cfg.parts) {
    const o = partObjs[p.id];
    if (!o) continue;
    const optOk = Object.entries(p.option || {}).every(([k, v]) => S.options[k] === v);
    o.visible = optOk && p.visible !== false && (guarding || !GUARDING.has(p.id));
  }
  if (floor) floor.visible = $('chk-floor').checked;
  if (pathObj) pathObj.visible = $('chk-path').checked;
}

// ---------------------------------------------------------------- tool, stock, toolpath
const toolGroup = new THREE.Group();
const toolMat = new THREE.MeshStandardMaterial({ color: 0xd4a017, metalness: 0.6, roughness: 0.35 });
const holderMat = new THREE.MeshStandardMaterial({ color: 0x8d949b, metalness: 0.5, roughness: 0.4 });
function cylinderAlong(radius, from, to, mat) {
  const g = new THREE.CylinderGeometry(radius, radius, to - from, 28);
  g.rotateX(Math.PI / 2);           // cylinder axis -> Z
  const m = new THREE.Mesh(g, mat);
  m.position.z = -(from + to) / 2;  // hangs down from the gauge line
  return m;
}
function buildTool(tool) {
  toolGroup.clear();
  if (!S.cfg) return;
  const [gx, gy, gz] = S.cfg.spindle.gauge_point;
  toolGroup.position.set(gx, gy, gz);
  const L = Math.max(tool.length || 0, 0);
  toolGroup.userData.length = L;
  if (!toolGroup.parent) links[S.cfg.spindle.link].add(toolGroup);
  if (L <= 0) return;
  const hl = Math.min(tool.holder_length ?? 45, L - 1);
  if (hl > 0) toolGroup.add(cylinderAlong((tool.holder_diameter ?? 50) / 2, 0, hl, holderMat));
  toolGroup.add(cylinderAlong((tool.diameter ?? 10) / 2, Math.max(hl, 0), L, toolMat));
}

const jobGroup = new THREE.Group();   // stock, fixtures, toolpath -- rides on the table
let pathObj = null;
const tipMarker = new THREE.Mesh(new THREE.SphereGeometry(2.5, 16, 12), new THREE.MeshBasicMaterial({ color: 0xe11d48 }));
function solidMesh(s, color, opacity) {
  const [a, b, c] = s.size;
  const g = s.type === 'cylinder' ? new THREE.CylinderGeometry(a / 2, a / 2, b, 48).rotateX(Math.PI / 2) : new THREE.BoxGeometry(a, b, c);
  const h = s.type === 'cylinder' ? b : c;
  const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color, transparent: opacity < 1, opacity, roughness: 0.7, metalness: 0.1 }));
  m.position.set(s.position[0], s.position[1], s.position[2] + h / 2);
  return m;
}
function buildJob(result) {
  jobGroup.clear();
  if (!jobGroup.parent) links[S.cfg.table.link].add(jobGroup);
  const setup = result.setup || {};
  for (const f of setup.fixtures || []) jobGroup.add(solidMesh(f, 0x6b7280, 1));
  if (setup.stock) jobGroup.add(solidMesh(setup.stock, 0x9ecae1, 0.45));

  // toolpath in the table frame: feed = teal, rapid = orange; skip tool-change legs
  const pos = [], col = [];
  const cFeed = new THREE.Color(0x0f9d8a), cRapid = new THREE.Color(0xf59e0b);
  for (let k = 1; k < result.t.length; k++) {
    const m = result.motion[k];
    if (m === 4 || m === 3 || result.tool[k] !== result.tool[k - 1]) continue;
    const a = result.tip[k - 1], b = result.tip[k];
    if (a[0] === b[0] && a[1] === b[1] && a[2] === b[2]) continue;
    pos.push(...a, ...b);
    const c = (m === 0 || m === 5) ? cRapid : cFeed;
    col.push(c.r, c.g, c.b, c.r, c.g, c.b);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  pathObj = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true }));
  jobGroup.add(pathObj);
  jobGroup.add(tipMarker);
  refreshVisibility();
}

// ---------------------------------------------------------------- UI: DRO + jog
function fmt(v, rot) { return (v >= 0 ? ' ' : '') + v.toFixed(rot ? 3 : 3); }
function updateDRO() {
  if (!S.cfg) return;
  const rows = JOINTS.map((n, i) => {
    const j = S.cfg.joints[n];
    const lim = j.limits;
    const over = lim && (S.q[i] < lim[0] - 1e-6 || S.q[i] > lim[1] + 1e-6);
    const unit = j.type === 'rotary' ? '°' : 'mm';
    return `<tr class="${over ? 'over' : ''}"><td>${n}</td><td>${fmt(S.q[i], j.type === 'rotary')}</td>` +
      `<td>${lim ? `${lim[0]} … ${lim[1]} ${unit}` : 'continuous'}</td></tr>`;
  });
  $('dro-body').innerHTML = rows.join('');
  const L = toolGroup.userData.length || 0;
  const tip = toolTipInTable(L);
  $('tip-readout').textContent = `Tool tip, table frame: X ${tip.x.toFixed(2)}  Y ${tip.y.toFixed(2)}  Z ${tip.z.toFixed(2)}`;
  tipMarker.position.copy(tip);
  document.querySelectorAll('.jog').forEach((row, i) => {
    const r = row.querySelector('input[type=range]'), num = row.querySelector('input[type=number]');
    if (document.activeElement !== r) r.value = S.q[i];
    if (document.activeElement !== num) num.value = S.q[i].toFixed(3);
  });
}

function buildJogUI() {
  const box = $('jog-sliders');
  box.innerHTML = '';
  JOINTS.forEach((n, i) => {
    const j = S.cfg.joints[n];
    const [lo, hi] = j.limits || [-360, 360];
    const row = document.createElement('div');
    row.className = 'jog';
    row.innerHTML = `<b>${n}</b><input type="range" min="${lo}" max="${hi}" step="${j.type === 'rotary' ? 0.5 : 0.5}" aria-label="${n}">` +
      `<input type="number" step="${j.type === 'rotary' ? 1 : 1}" aria-label="${n} value">`;
    const set = (v) => { const q = S.q.slice(); q[i] = Number(v); manual(); applyPose(q); };
    row.querySelector('input[type=range]').addEventListener('input', (e) => set(e.target.value));
    row.querySelector('input[type=number]').addEventListener('change', (e) => set(e.target.value));
    box.appendChild(row);
  });
}

function manual() {
  S.playing = false;
  $('btn-play').textContent = '▶';
  if (S.live) disconnectLive();
  setMode('MANUAL');
}
function setMode(text, cls = '') {
  const el = $('hud-mode');
  el.textContent = text;
  el.className = cls;
}

// ---------------------------------------------------------------- UI: options
function buildOptionsUI() {
  const box = $('options');
  box.innerHTML = '';
  for (const [k, v] of Object.entries(S.cfg.options)) {
    const lab = document.createElement('label');
    lab.className = 'row';
    lab.innerHTML = `<span style="min-width:110px">${k.replace('_', ' ')}</span>`;
    const sel = document.createElement('select');
    for (const c of v.choices) sel.add(new Option(c, c, false, c === S.options[k]));
    sel.addEventListener('change', () => { S.options[k] = sel.value; refreshVisibility(); });
    lab.appendChild(sel);
    box.appendChild(lab);
  }
}

// ---------------------------------------------------------------- program / playback
function loadResult(result) {
  if (result.error) { $('sim-status').textContent = `Error: ${result.error}`; return; }
  S.result = result;
  S.time = 0;
  S.lastLine = -1;
  S.currentTool = null;
  if (result.options) {
    Object.assign(S.options, result.options);
    buildOptionsUI();
  }
  if (result.program) $('gcode').value = result.program.join('\n');
  if (result.setup_yaml) $('setup').value = result.setup_yaml;
  buildJob(result);
  $('player').classList.remove('hidden');
  const s = result.summary;
  $('summary').textContent = `Cycle ${fmtTime(s.duration_s)} · cutting ${fmtTime(s.cutting_s)} · rapid ${fmtTime(s.rapid_s)} · ` +
    `${s.tool_changes} tool change${s.tool_changes === 1 ? '' : 's'}`;
  renderIssues(result);
  S.listingStart = -1;
  seek(0);
  selectTab('program');
  setMode('PROGRAM');
}

function renderIssues(r) {
  const ul = $('issues');
  const items = [
    ...r.collisions.map((c) => ({ cls: 'collision', t: c.t, line: c.line, text: c.kind === 'rapid_into_stock' ? `rapid into stock (${c.a})` : `${c.a} × ${c.b}`, parts: [c.a, c.b] })),
    ...r.limits.map((l) => ({ cls: 'limit', t: l.t, line: l.line, text: l.message })),
    ...r.warnings.map((w) => ({ cls: 'warn', t: null, line: w.line, text: w.message })),
  ].sort((a, b) => (a.t ?? 1e9) - (b.t ?? 1e9));
  ul.innerHTML = '';
  if (!items.length) {
    ul.innerHTML = `<li class="none">No collisions, over-travel or warnings${r.collision_checked === false ? ' (collision check off)' : ''}</li>`;
    return;
  }
  for (const it of items) {
    const li = document.createElement('li');
    li.className = it.cls;
    li.textContent = `L${it.line}${it.t != null ? ` · ${fmtTime(it.t)}` : ''} · ${it.text}`;
    li.addEventListener('click', () => {
      if (it.t != null) { S.playing = false; $('btn-play').textContent = '▶'; seek(it.t); }
      highlight(it.parts || []);
    });
    ul.appendChild(li);
  }
}

function highlight(ids) {
  for (const id of S.highlight) for (const m of partMats[id] || []) m.emissive?.setHex(0x000000);
  S.highlight = new Set(ids.filter((id) => partMats[id]));
  for (const id of S.highlight) for (const m of partMats[id]) m.emissive.setHex(0xaa0000);
  if (ids.length) setMode('COLLISION', 'crash');
}

function fmtTime(s) {
  const m = Math.floor(s / 60), r = s - m * 60;
  return m ? `${m}:${r.toFixed(1).padStart(4, '0')}` : `${r.toFixed(1)} s`;
}

function sampleAt(t) {
  const T = S.result.t;
  let lo = 0, hi = T.length - 1;
  if (t <= T[0]) return { k: 0, f: 0 };
  if (t >= T[hi]) return { k: hi, f: 0 };
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (T[mid] <= t) lo = mid; else hi = mid; }
  const span = T[hi] - T[lo];
  return { k: hi, f: span > 0 ? (t - T[lo]) / span : 1 };
}

function seek(t) {
  const r = S.result;
  if (!r) return;
  S.time = Math.min(Math.max(t, 0), r.summary.duration_s);
  const { k, f } = sampleAt(S.time);
  const a = r.q[Math.max(k - 1, 0)], b = r.q[k];
  const q = a.map((v, i) => v + (b[i] - v) * f);
  const toolNo = r.tool[k];
  if (toolNo !== S.currentTool) {
    S.currentTool = toolNo;
    const tools = r.setup?.tools || {};
    const tool = tools[toolNo] || tools[String(toolNo)] || r.setup?.default_tool || { length: 100 };
    buildTool(toolNo ? tool : { length: 0 });
    $('tool-readout').textContent = toolNo ? `T${toolNo} ${tool.name || ''} · L ${tool.length} · Ø ${tool.diameter}` : 'No tool';
  }
  applyPose(q);
  showLine(r.line[k]);
  $('time').textContent = `${fmtTime(S.time)} / ${fmtTime(r.summary.duration_s)}`;
  if (document.activeElement !== $('scrub')) $('scrub').value = Math.round((S.time / Math.max(r.summary.duration_s, 1e-9)) * 1000);
}

function showLine(line) {
  if (line === S.lastLine || !S.result?.program) return;
  S.lastLine = line;
  const prog = S.result.program;
  $('hud-line').textContent = line ? `N${line}  ${prog[line - 1] ?? ''}` : '';
  // render a window of the listing around the current line (programs can be long)
  const W = 150;
  if (S.listingStart < 0 || line < S.listingStart + 20 || line > S.listingStart + 2 * W - 20) {
    S.listingStart = Math.max(1, line - W);
    const end = Math.min(prog.length, S.listingStart + 2 * W);
    const esc = (s) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
    let html = '';
    for (let i = S.listingStart; i <= end; i++) html += `<div data-l="${i}"><span class="n">${i}</span>${esc(prog[i - 1] ?? '')}</div>`;
    $('listing').innerHTML = html;
  }
  $('listing').querySelector('.cur')?.classList.remove('cur');
  const el = $('listing').querySelector(`[data-l="${line}"]`);
  if (el) {
    el.classList.add('cur');
    const box = $('listing');
    const top = el.offsetTop;  // #listing is position:relative
    if (top < box.scrollTop || top > box.scrollTop + box.clientHeight - 20) box.scrollTop = top - box.clientHeight / 2;
  }
}

async function simulate() {
  const gcode = $('gcode').value;
  if (!gcode.trim()) { $('sim-status').textContent = 'Paste or open a program first.'; return; }
  if (!S.server) {
    $('sim-status').textContent = 'Simulating needs the local server: python -m umc_twin serve. You can still open a result .json from the CLI.';
    return;
  }
  $('sim-status').textContent = 'Simulating…';
  const t0 = performance.now();
  const res = await fetch('api/simulate', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ gcode, setup: $('setup').value, options: S.options, collisions: $('chk-collide').checked }),
  });
  const result = await res.json();
  if (result.error) { $('sim-status').textContent = `Error: ${result.error}`; return; }
  result.setup_yaml = $('setup').value;
  $('sim-status').textContent = `Simulated in ${((performance.now() - t0) / 1000).toFixed(1)} s`;
  loadResult(result);
}

async function loadExample(name) {
  $('sim-status').textContent = 'Loading…';
  const result = await (await fetch(`assets/examples/${name}.json`)).json();
  $('sim-status').textContent = `Loaded ${name} (pre-computed; edit and Simulate to re-run)`;
  loadResult(result);
}

// ---------------------------------------------------------------- live
function connectLive() {
  if (!S.server) { $('live-status').textContent = 'needs the local server (python -m umc_twin serve --replay …)'; return; }
  S.playing = false;
  const es = new EventSource('api/live');
  S.live = es;
  $('btn-live').textContent = 'Disconnect';
  $('live-status').textContent = 'connecting…';
  es.onmessage = (ev) => {
    const st = JSON.parse(ev.data);
    applyPose(st.q);
    $('live-status').textContent = `${st.source} · ${st.mode || ''} · line ${st.line ?? '–'} · T${st.tool ?? '–'} · S${Math.round(st.spindle_rpm ?? 0)}`;
    setMode('LIVE · ' + st.source.toUpperCase(), 'live');
    if (S.result?.program && st.line) showLine(st.line);
  };
  es.onerror = () => { $('live-status').textContent = 'no live source (start the server with --replay)'; disconnectLive(); };
}
function disconnectLive() {
  S.live?.close();
  S.live = null;
  $('btn-live').textContent = 'Connect';
}

// ---------------------------------------------------------------- calibration
function calInputs() {
  const num = (id) => { const v = $(id).value.trim(); return v === '' ? null : Number(v); };
  const inputs = { units: $('cal-units').value };
  if (num('cal-nose') != null) inputs.nose_to_platter = num('cal-nose');
  const m = ['cal-mrzp-x', 'cal-mrzp-y', 'cal-mrzp-z'].map(num);
  if (m.every((v) => v != null)) inputs.mrzp = m;
  if ($('cal-b').value) inputs.b_direction = $('cal-b').value;
  if ($('cal-c').value) inputs.c_direction = $('cal-c').value;
  const rapid = {};
  if (num('cal-brapid')) rapid.B = num('cal-brapid');
  if (num('cal-crapid')) rapid.C = num('cal-crapid');
  if (Object.keys(rapid).length) inputs.rotary_rapid = rapid;
  if (num('cal-tc') != null) inputs.tool_change_time = num('cal-tc');
  return inputs;
}
function cliFor(inputs) {
  const a = [`python -m umc_twin calibrate --units ${inputs.units}`];
  if (inputs.nose_to_platter != null) a.push(`--nose-to-platter ${inputs.nose_to_platter}`);
  if (inputs.mrzp) a.push(`--mrzp ${inputs.mrzp.join(' ')}`);
  if (inputs.b_direction) a.push(`--b-dir ${inputs.b_direction}`);
  if (inputs.c_direction) a.push(`--c-dir ${inputs.c_direction}`);
  if (inputs.rotary_rapid?.B) a.push(`--b-rapid ${inputs.rotary_rapid.B}`);
  if (inputs.rotary_rapid?.C) a.push(`--c-rapid ${inputs.rotary_rapid.C}`);
  if (inputs.tool_change_time != null) a.push(`--tc-time ${inputs.tool_change_time}`);
  return a.join(' ');
}
async function runCalibration(save) {
  const inputs = calInputs();
  if (!S.server) {
    $('cal-out').textContent = `Saving needs the local server. Or run:\n\n${cliFor(inputs)}`;
    return;
  }
  const res = await (await fetch('api/calibrate', {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ ...inputs, save }),
  })).json();
  if (res.error) { $('cal-out').textContent = `Error: ${res.error}`; return; }
  const s = res.summary;
  $('cal-out').textContent = [...res.notes,
    `nose → platter at home: ${s.nose_to_platter_at_home_mm.toFixed(2)} mm (${(s.nose_to_platter_at_home_mm / 25.4).toFixed(3)} in)`,
    `MRZP: ${s.mrzp_in.map((v) => v.toFixed(4)).join(' / ')} in`,
    save ? 'Saved. Reloading the model…' : 'Preview only. Save to apply.'].join('\n');
  if (save) setTimeout(() => location.reload(), 1200);
}
async function loadCalibrationInfo() {
  if (!S.server) return;
  const info = await (await fetch('api/calibration')).json();
  const est = info.cad_estimate.mrzp_in.map((v) => v.toFixed(4)).join(' / ');
  $('cal-mrzp-est').textContent = `The CAD predicts about ${est} in.`;
  if (info.summary.calibrated) $('cal-out').textContent = `Calibrated: nose → platter ${info.summary.nose_to_platter_at_home_mm.toFixed(2)} mm, MRZP ${info.summary.mrzp_in.map((v) => v.toFixed(4)).join(' / ')} in`;
}
async function saveOptions() {
  if (!S.server) { $('options-status').textContent = 'needs the local server'; return; }
  const res = await (await fetch('api/calibrate', {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ options: S.options, save: true }),
  })).json();
  $('options-status').textContent = res.error ? `Error: ${res.error}` : 'saved to config/calibration.yaml';
}

// ---------------------------------------------------------------- wiring
function selectTab(name) {
  document.querySelectorAll('.tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('hidden', t.id !== `tab-${name}`));
}
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => selectTab(b.dataset.tab)));
document.querySelectorAll('#cams button').forEach((b) => b.addEventListener('click', () => setCam(b.dataset.cam)));
$('btn-home').addEventListener('click', () => { manual(); applyPose([0, 0, 0, 0, 0]); });
$('btn-center').addEventListener('click', () => {
  manual();
  const g = S.cfg.spindle.gauge_point, L = toolGroup.userData.length || 0;
  applyPose([-g[0], -g[1], S.cfg.table.top_z + 50 - g[2] + L, 0, 0].map((v, i) => i === 2 ? Math.min(v, 0) : v));
});
$('jog-tool').addEventListener('change', (e) => { buildTool({ length: Number(e.target.value) }); applyPose(S.q); });
['chk-guarding', 'chk-path', 'chk-floor'].forEach((id) => $(id).addEventListener('change', refreshVisibility));
$('btn-sim').addEventListener('click', simulate);
$('btn-demo').addEventListener('click', () => loadExample('demo_5axis'));
$('btn-crash').addEventListener('click', () => loadExample('crash_demo'));
$('file-nc').addEventListener('change', async (e) => {
  const f = e.target.files[0];
  if (!f) return;
  $('gcode').value = await f.text();
  $('gcode').closest('details').open = true;
  $('sim-status').textContent = `${f.name} loaded — press Simulate`;
});
$('file-json').addEventListener('change', async (e) => {
  const f = e.target.files[0];
  if (f) loadResult(JSON.parse(await f.text()));
});
$('btn-play').addEventListener('click', () => {
  if (!S.result) return;
  if (S.live) disconnectLive();
  if (S.time >= S.result.summary.duration_s) S.time = 0;
  S.playing = !S.playing;
  $('btn-play').textContent = S.playing ? '❚❚' : '▶';
  highlight([]);
  setMode('PROGRAM');
});
$('speed').addEventListener('change', (e) => { S.speed = Number(e.target.value); });
$('scrub').addEventListener('input', (e) => { if (S.result) seek((Number(e.target.value) / 1000) * S.result.summary.duration_s); });
$('btn-live').addEventListener('click', () => (S.live ? disconnectLive() : connectLive()));
$('btn-cal-preview').addEventListener('click', () => runCalibration(false));
$('btn-cal-save').addEventListener('click', () => runCalibration(true));
$('btn-save-options').addEventListener('click', saveOptions);

let last = performance.now();
function frame(now) {
  const dt = (now - last) / 1000;
  last = now;
  if (S.playing && S.result) {
    seek(S.time + dt * S.speed);
    if (S.time >= S.result.summary.duration_s) { S.playing = false; $('btn-play').textContent = '▶'; }
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

(async function main() {
  resize();
  setCam('iso');
  requestAnimationFrame(frame);
  try {
    const st = await fetch('api/status');
    S.server = st.ok && (await st.json()).live_source !== undefined;
  } catch { S.server = false; }
  $('server-status').textContent = S.server ? 'server connected' : 'static';
  $('server-status').classList.toggle('ok', S.server);
  await loadMachine();
  loadCalibrationInfo();
  const params = new URLSearchParams(location.search);
  if (params.get('example')) loadExample(params.get('example'));
  window.__twin = { S, applyPose, seek, setCam, loadExample, selectTab };  // handy from the console
})();
