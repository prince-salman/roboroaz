/**
 * Interactive Code Injection & Script Execution Engine
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop
 * Compiles and executes user JavaScript navigation scripts in real-time.
 */

class CodeEngine {
  constructor() {
    this.compiledFunction = null;
    this.memory = {}; // Preserved state between ticks
    this.isRunning = false;
    this.hasError = false;
    this.lastErrorMessage = null;
    this.logCallbacks = [];
    this.lastLogTime = 0;

    // Default Preset Scripts
    this.presets = {
      whiteboard_waypoints: `const M = memory;
const W = 5.0;
const H = 0.165;

if (!M.initialized) {
  M.initialized = true;
  if (sensors.pose.x < 1.5 && sensors.pose.y < 1.5 && robot.resetPose) {
    robot.resetPose(2.5, 0.6, 0);
  }
  M.originX = sensors.pose.x;
  M.originY = sensors.pose.y;
  M.HEAD = Math.round(sensors.imu.headingRad / (Math.PI / 2)) * (Math.PI / 2);
  M.targets = [
    { id: 1, x: 0.0, y: 2.0 },
    { id: 2, x: 2.0, y: 3.0 },
    { id: 3, x: -2.0, y: 4.0 }
  ];
  M.idx = 0;
  M.state = "NAVIGATING";
  M.holdTimer = 0;
  M.HOLD = 5.0;
  M.TOL = 0.08;
  M.RES = 0.05;
  M.N = Math.round(W / M.RES);
  M.SAFE = 0.06;
  M.INFL = 0.4;
  M.VMAX = 0.35;
  M.path = [];
  M.pi = 0;
  M.replanTimer = 999;
  M.vgx = 0;
  M.vgy = 0;
  M.pts = [];
  M.progT = 0;
  M.progX = sensors.pose.x;
  M.progY = sensors.pose.y;
  M.relax = 0;
  M.lastSec = -1;
  M.done = false;
  robot.setShape("O");
  robot.log("=== 1. INITIALIZE ===");
  robot.log("Origin (0,0) at (" + M.originX.toFixed(2) + ", " + M.originY.toFixed(2) + ") | LiDAR = eyes, IMU = ears");
  robot.log("Targets: (0,2) -> (2,3) -> (-2,4) | hold 5s each");
  robot.log("A* + Costmap Inflation | Holonomic DWA | heading locked by IMU");
}

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

const gapBox = (x, y, o) => {
  const gx = Math.max(o.x - (x + H), (x - H) - (o.x + o.w));
  const gy = Math.max(o.y - (y + H), (y - H) - (o.y + o.h));
  return Math.max(gx, gy);
};

const X = sensors.pose.x;
const Y = sensors.pose.y;
const TH = sensors.imu.headingRad;
const ranges = sensors.lidar.ranges;

const scanPts = [];
const nearPts = [];
for (let a = 0; a < 360; a += 3) {
  const r = ranges[a];
  if (r > 0.05 && r < 2.5) {
    const ang = TH + (a * Math.PI) / 180;
    const px = X + r * Math.cos(ang);
    const py = Y + r * Math.sin(ang);
    scanPts.push(px, py);
    if (r < 0.7) nearPts.push(px, py);
  }
}

const clearAt = (x, y) => {
  let d = Math.min(x - H - 0.02, W - 0.02 - x - H, y - H - 0.02, W - 0.02 - y - H);
  const obs = sensors.obstacles || [];
  for (let k = 0; k < obs.length; k++) {
    const g = gapBox(x, y, obs[k]);
    if (g < d) d = g;
  }
  for (let k = 0; k < nearPts.length; k += 2) {
    const g = Math.max(Math.abs(nearPts[k] - x), Math.abs(nearPts[k + 1] - y)) - H;
    if (g < d) d = g;
  }
  return d;
};

const buildCostmap = () => {
  const N = M.N;
  const R = M.RES;
  const CAP = M.INFL + 0.2;
  const dist = new Float32Array(N * N);
  for (let j = 0; j < N; j++) {
    const cy = (j + 0.5) * R;
    const wy = Math.min(cy - H - 0.02, W - 0.02 - cy - H);
    for (let i = 0; i < N; i++) {
      const cx = (i + 0.5) * R;
      dist[j * N + i] = Math.min(CAP, wy, cx - H - 0.02, W - 0.02 - cx - H);
    }
  }
  const reach = H + CAP;
  const stamp = (x0, y0, x1, y1) => {
    const i0 = Math.max(0, Math.floor((x0 - reach) / R));
    const i1 = Math.min(N - 1, Math.floor((x1 + reach) / R));
    const j0 = Math.max(0, Math.floor((y0 - reach) / R));
    const j1 = Math.min(N - 1, Math.floor((y1 + reach) / R));
    for (let j = j0; j <= j1; j++) {
      const cy = (j + 0.5) * R;
      const gy = Math.max(y0 - cy, cy - y1);
      for (let i = i0; i <= i1; i++) {
        const cx = (i + 0.5) * R;
        const g = Math.max(Math.max(x0 - cx, cx - x1), gy) - H;
        const k = j * N + i;
        if (g < dist[k]) dist[k] = g;
      }
    }
  };
  const obs = sensors.obstacles || [];
  for (let k = 0; k < obs.length; k++) stamp(obs[k].x, obs[k].y, obs[k].x + obs[k].w, obs[k].y + obs[k].h);
  for (let k = 0; k < scanPts.length; k += 2) stamp(scanPts[k], scanPts[k + 1], scanPts[k], scanPts[k + 1]);
  M.dist = dist;
};

const lethalAt = (k, safe) => M.dist[k] < safe;
const cellCost = (k) => {
  const d = M.dist[k];
  if (d >= M.INFL) return 0;
  const f = (M.INFL - d) / M.INFL;
  return 25 * f * f;
};

const astar = (sx, sy, gx, gy, safe) => {
  const N = M.N;
  const toI = (v) => Math.max(0, Math.min(N - 1, Math.floor(v / M.RES)));
  const si = toI(sx), sj = toI(sy);
  let gi = toI(gx), gj = toI(gy);
  const startK = sj * N + si;
  const safeEff = Math.min(safe, M.dist[startK] - 0.005);
  if (lethalAt(gj * N + gi, safeEff)) {
    let best = -1, bestD = 1e9;
    for (let dj = -14; dj <= 14; dj++) {
      for (let di = -14; di <= 14; di++) {
        const ni = gi + di, nj = gj + dj;
        if (ni < 0 || nj < 0 || ni >= N || nj >= N) continue;
        const k = nj * N + ni;
        if (lethalAt(k, safe)) continue;
        const dd = di * di + dj * dj;
        if (dd < bestD) { bestD = dd; best = k; }
      }
    }
    if (best < 0) return null;
    gi = best % N;
    gj = Math.floor(best / N);
  }
  const goalK = gj * N + gi;
  const g = new Float32Array(N * N).fill(1e9);
  const par = new Int32Array(N * N).fill(-1);
  const closed = new Uint8Array(N * N);
  const CAPH = N * N * 8;
  const hf = new Float32Array(CAPH);
  const hk = new Int32Array(CAPH);
  let hn = 0;
  const push = (k, f) => {
    if (hn >= CAPH) return;
    let c = hn++;
    while (c > 0) {
      const p = (c - 1) >> 1;
      if (hf[p] <= f) break;
      hf[c] = hf[p];
      hk[c] = hk[p];
      c = p;
    }
    hf[c] = f;
    hk[c] = k;
  };
  const pop = () => {
    const top = hk[0];
    hn--;
    if (hn > 0) {
      const f = hf[hn], k = hk[hn];
      let c = 0;
      for (;;) {
        const l = 2 * c + 1;
        if (l >= hn) break;
        const r = l + 1;
        const m = r < hn && hf[r] < hf[l] ? r : l;
        if (hf[m] >= f) break;
        hf[c] = hf[m];
        hk[c] = hk[m];
        c = m;
      }
      hf[c] = f;
      hk[c] = k;
    }
    return top;
  };
  const hfun = (i, j) => {
    const dx = Math.abs(i - gi), dy = Math.abs(j - gj);
    return Math.max(dx, dy) + 0.4142 * Math.min(dx, dy);
  };
  const free = (k) => k === startK || !lethalAt(k, safeEff);
  g[startK] = 0;
  push(startK, hfun(si, sj));
  const D = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, 1.4142], [-1, 1, 1.4142], [1, -1, 1.4142], [-1, -1, 1.4142]];
  let found = false;
  while (hn > 0) {
    const k = pop();
    if (closed[k]) continue;
    closed[k] = 1;
    if (k === goalK) { found = true; break; }
    const ci = k % N, cj = Math.floor(k / N);
    for (let q = 0; q < 8; q++) {
      const ni = ci + D[q][0], nj = cj + D[q][1];
      if (ni < 0 || nj < 0 || ni >= N || nj >= N) continue;
      const nk = nj * N + ni;
      if (closed[nk] || !free(nk)) continue;
      if (D[q][2] > 1 && (!free(cj * N + ni) || !free(nj * N + ci))) continue;
      const ng = g[k] + D[q][2] * (1 + cellCost(nk));
      if (ng < g[nk]) {
        g[nk] = ng;
        par[nk] = k;
        push(nk, ng + hfun(ni, nj));
      }
    }
  }
  if (!found) return null;
  const out = [];
  let c = goalK;
  while (c !== -1) {
    out.unshift({ x: ((c % N) + 0.5) * M.RES, y: (Math.floor(c / N) + 0.5) * M.RES });
    c = par[c];
  }
  out[0] = { x: sx, y: sy };
  return out;
};

const segmentClear = (ax, ay, bx, by, need) => {
  const L = Math.hypot(bx - ax, by - ay);
  const n = Math.max(1, Math.ceil(L / 0.04));
  for (let s = 1; s <= n; s++) {
    const t = s / n;
    if (clearAt(ax + (bx - ax) * t, ay + (by - ay) * t) < need) return false;
  }
  return true;
};

const drive = (gvx, gvy) => {
  M.vgx = gvx;
  M.vgy = gvy;
  const c = Math.cos(TH), s = Math.sin(TH);
  const omega = Math.max(-1.5, Math.min(1.5, -3.0 * wrap(TH - M.HEAD)));
  robot.setVelocity(gvx * c + gvy * s, -gvx * s + gvy * c, omega);
};

if (M.idx >= M.targets.length) {
  drive(0, 0);
  if (!M.done) {
    M.done = true;
    M.state = "FINISHED";
    if (robot.setPlannedPath) robot.setPlannedPath([]);
    robot.log(">>> MISSION COMPLETE: (0,2), (2,3), (-2,4) reached, 5s hold each <<<");
  }
  return;
}

const target = M.targets[M.idx];
const TGX = Math.max(H + 0.05, Math.min(W - H - 0.05, target.x + M.originX));
const TGY = Math.max(H + 0.05, Math.min(W - H - 0.05, target.y + M.originY));
if (robot.setGoalMarker) robot.setGoalMarker(TGX, TGY);

if (M.state === "HOLD") {
  drive(0, 0);
  M.holdTimer += dt;
  const sec = Math.floor(M.holdTimer);
  if (sec !== M.lastSec) {
    M.lastSec = sec;
    robot.log("Hold at (" + target.x + ", " + target.y + "): " + Math.min(5, sec) + "s / 5s");
  }
  if (M.holdTimer >= M.HOLD) {
    robot.log(">>> 5s interval done at point " + target.id + " -> next target");
    M.idx++;
    M.state = "NAVIGATING";
    M.holdTimer = 0;
    M.path = [];
    M.replanTimer = 999;
    M.relax = 0;
  }
  return;
}

M.replanTimer += dt;
let carrotBlocked = false;
if (M.path.length > 0 && M.pi < M.path.length) {
  const p = M.path[Math.min(M.path.length - 1, M.pi + 1)];
  carrotBlocked = !segmentClear(X, Y, p.x, p.y, 0.0);
}

if (M.path.length === 0 || M.replanTimer > 1.0 || carrotBlocked) {
  M.pts = scanPts.slice();
  buildCostmap();
  const levels = [M.SAFE, 0.035, 0.015, 0.003];
  let path = null;
  for (let L = Math.min(M.relax, levels.length - 1); L < levels.length && !path; L++) {
    path = astar(X, Y, TGX, TGY, levels[L]);
  }
  if (path) {
    M.path = path;
    M.pi = 0;
    if (robot.setPlannedPath) robot.setPlannedPath(path);
    const e = path[path.length - 1];
    const off = Math.hypot(e.x - TGX, e.y - TGY);
    if (off > 0.05 && M.adjLogged !== M.idx) {
      M.adjLogged = M.idx;
      robot.log("Target (" + target.x + ", " + target.y + ") blocked by obstacle -> nearest reachable point (" + (e.x - M.originX).toFixed(2) + ", " + (e.y - M.originY).toFixed(2) + ")");
    }
  }
  M.replanTimer = 0;
}

const goal = M.path.length > 0 ? M.path[M.path.length - 1] : { x: TGX, y: TGY };
const distGoal = Math.hypot(goal.x - X, goal.y - Y);
if (distGoal < M.TOL) {
  M.state = "HOLD";
  M.holdTimer = 0;
  M.lastSec = -1;
  drive(0, 0);
  robot.log(">>> REACHED point " + target.id + " (" + target.x + ", " + target.y + ") | local pose (" + (X - M.originX).toFixed(2) + ", " + (Y - M.originY).toFixed(2) + ")");
  return;
}

M.progT += dt;
if (M.progT > 3.0) {
  if (Math.hypot(X - M.progX, Y - M.progY) < 0.05) {
    M.relax = Math.min(3, M.relax + 1);
    M.replanTimer = 999;
    robot.log("Low progress -> replan with tighter clearance level " + M.relax);
  }
  M.progT = 0;
  M.progX = X;
  M.progY = Y;
}

let cx = goal.x, cy = goal.y;
if (M.path.length > 1) {
  let best = M.pi, bd = 1e9;
  for (let q = M.pi; q < Math.min(M.path.length, M.pi + 40); q++) {
    const d = Math.hypot(M.path[q].x - X, M.path[q].y - Y);
    if (d < bd) { bd = d; best = q; }
  }
  M.pi = best;
  const curClear = clearAt(X, Y);
  const need = Math.min(0.03, Math.max(0, curClear - 0.005));
  let li = Math.min(M.path.length - 1, M.pi + 1);
  for (let q = M.pi + 1; q < M.path.length; q++) {
    if (Math.hypot(M.path[q].x - X, M.path[q].y - Y) > 0.45) break;
    if (!segmentClear(X, Y, M.path[q].x, M.path[q].y, need)) break;
    li = q;
  }
  cx = M.path[li].x;
  cy = M.path[li].y;
}

let dx = cx - X, dy = cy - Y;
const dn = Math.hypot(dx, dy) || 1e-6;
dx /= dn;
dy /= dn;
const vcap = Math.max(0.06, Math.min(M.VMAX, 1.2 * distGoal));

M.dwaT = (M.dwaT || 0) + dt;
if (M.dwaT < 0.05 && M.hasCmd) {
  drive(M.vgx, M.vgy);
  return;
}
M.dwaT = 0;
M.hasCmd = true;

const startClear = clearAt(X, Y);
const HOR = 0.6;
const STEPS = 5;
let best = null;
let bestScore = -1e9;
const cands = [];
for (let a = 0; a < 16; a++) {
  const ang = (a * Math.PI) / 8;
  for (const f of [1.0, 0.45]) cands.push([Math.cos(ang) * vcap * f, Math.sin(ang) * vcap * f]);
}
cands.push([dx * vcap, dy * vcap], [dx * vcap * 0.5, dy * vcap * 0.5], [dx * vcap * 0.2, dy * vcap * 0.2], [0, 0]);
for (let c = 0; c < cands.length; c++) {
  const vx = cands[c][0], vy = cands[c][1];
  let minC = 9;
  for (let s = 1; s <= STEPS; s++) {
    const t = (HOR * s) / STEPS;
    const cl = clearAt(X + vx * t, Y + vy * t);
    if (cl < minC) minC = cl;
  }
  const floor = Math.min(0.02, startClear - 0.002);
  if (minC < floor) continue;
  const prog = vx * dx + vy * dy;
  const sp = Math.hypot(vx, vy);
  const off = sp > 1e-6 ? 1 - prog / sp : 0;
  const score = 4.0 * prog - 0.15 * off + 0.8 * Math.min(minC, 0.12) - 0.3 * Math.hypot(vx - M.vgx, vy - M.vgy);
  if (score > bestScore) {
    bestScore = score;
    best = [vx, vy];
  }
}

if (!best) {
  let ex = 0, ey = 0;
  for (let k = 0; k < scanPts.length; k += 2) {
    const qx = X - scanPts[k], qy = Y - scanPts[k + 1];
    const d = Math.hypot(qx, qy);
    if (d < 0.5 && d > 1e-3) { ex += qx / (d * d); ey += qy / (d * d); }
  }
  const en = Math.hypot(ex, ey);
  best = en > 1e-6 ? [(0.08 * ex) / en, (0.08 * ey) / en] : [0, 0];
  M.replanTimer = 999;
}

drive(best[0], best[1]);
`,

      default_avoidance: `const M = memory;
const W = 5.0;
const H = 0.165;
const RES = 0.05;
const N = Math.round(W / RES);

if (!M.initialized) {
  M.initialized = true;
  M.path = [];
  M.pi = 0;
  M.replanTimer = 999;
  M.lastDelivered = 0;
  M.state = "NAVIGATE";
  M.backTimer = 0;
  robot.log("=== Autonomous 3-Cube Cargo Pusher ===");
  robot.log("Misi: Mendorong seluruh 3 kubus ke tanda kuning target");
  const initTarget = (sensors.cargo && sensors.cargo.activeTarget) || (sensors.cargo && sensors.cargo.cubes && sensors.cargo.cubes[0]);
  if (initTarget) {
    robot.log("Menerima koordinat target awal #0" + initTarget.id + " (" + initTarget.name + "): X=" + initTarget.x.toFixed(2) + ", Y=" + initTarget.y.toFixed(2));
  }
}

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const X = sensors.pose.x;
const Y = sensors.pose.y;
const TH = sensors.imu.headingRad;
const GX = sensors.target.x;
const GY = sensors.target.y;

if (robot.setGoalMarker) robot.setGoalMarker(GX, GY);

if (sensors.mission && sensors.mission.completed) {
  if (!M.completionLogged) {
    M.completionLogged = true;
    robot.log("Misi Selesai: Seluruh 3 kubus berhasil didorong ke tanda kuning target!");
  }
  robot.setVelocity(0, 0, 0);
  return;
}

if (M.state === "BACKING_UP") {
  M.backTimer -= dt;
  if (M.backTimer <= 0) {
    M.state = "NAVIGATE";
    M.path = [];
    M.replanTimer = 999;
  } else {
    robot.setVelocity(-0.25, 0, 0);
    return;
  }
}

const cubes = sensors.cargo.cubes || [];
const undelivered = cubes.filter(c => Math.hypot(c.x - GX, c.y - GY) >= 0.48 && c.state !== "DELIVERED_AT_GOAL");
const deliveredCount = 3 - undelivered.length;

if (deliveredCount !== M.lastDelivered) {
  M.lastDelivered = deliveredCount;
  robot.log("Kubus #" + deliveredCount + " berhasil didorong ke tanda kuning! Terkirim: " + deliveredCount + "/3");
  if (deliveredCount < 3) {
    const nextTarget = (sensors.cargo && sensors.cargo.activeTarget) || (sensors.cargo && sensors.cargo.cubes && sensors.cargo.cubes.find(c => c.id === deliveredCount + 1));
    if (nextTarget) {
      robot.log("Menerima koordinat target berikutnya #0" + nextTarget.id + " (" + nextTarget.name + "): X=" + nextTarget.x.toFixed(2) + ", Y=" + nextTarget.y.toFixed(2));
    }
    M.state = "BACKING_UP";
    M.backTimer = 0.8;
    robot.setVelocity(-0.25, 0, 0);
    return;
  } else {
    robot.log("Misi Selesai: Seluruh 3 kubus berhasil dikirim ke tanda kuning!");
  }
}

const buildCostmap = () => {
  const dist = new Float32Array(N * N);
  const CAP = 0.6;
  for (let j = 0; j < N; j++) {
    const cy = (j + 0.5) * RES;
    const wy = Math.min(cy - H - 0.02, W - 0.02 - cy - H);
    for (let i = 0; i < N; i++) {
      const cx = (i + 0.5) * RES;
      dist[j * N + i] = Math.min(CAP, wy, cx - H - 0.02, W - 0.02 - cx - H);
    }
  }
  const reach = H + CAP;
  const obs = sensors.obstacles || [];
  for (let k = 0; k < obs.length; k++) {
    const o = obs[k];
    const i0 = Math.max(0, Math.floor((o.x - reach) / RES));
    const i1 = Math.min(N - 1, Math.floor((o.x + o.w + reach) / RES));
    const j0 = Math.max(0, Math.floor((o.y - reach) / RES));
    const j1 = Math.min(N - 1, Math.floor((o.y + o.h + reach) / RES));
    for (let j = j0; j <= j1; j++) {
      const cy = (j + 0.5) * RES;
      const gy = Math.max(o.y - cy, cy - (o.y + o.h));
      for (let i = i0; i <= i1; i++) {
        const cx = (i + 0.5) * RES;
        const g = Math.max(Math.max(o.x - cx, cx - (o.x + o.w)), gy) - H;
        const idx = j * N + i;
        if (g < dist[idx]) dist[idx] = g;
      }
    }
  }
  return dist;
};

const astar = (costmap, sx, sy, gx, gy, safe) => {
  const toI = (v) => Math.max(0, Math.min(N - 1, Math.floor(v / RES)));
  const si = toI(sx), sj = toI(sy);
  let gi = toI(gx), gj = toI(gy);
  const startK = sj * N + si;
  const safeEff = Math.min(safe, costmap[startK] - 0.005);
  if (costmap[gj * N + gi] < safeEff) {
    let best = -1, bestD = 1e9;
    for (let dj = -14; dj <= 14; dj++) {
      for (let di = -14; di <= 14; di++) {
        const ni = gi + di, nj = gj + dj;
        if (ni < 0 || ni >= N || nj < 0 || nj >= N) continue;
        const nk = nj * N + ni;
        if (costmap[nk] >= safeEff) {
          const d = di * di + dj * dj;
          if (d < bestD) { bestD = d; best = nk; }
        }
      }
    }
    if (best !== -1) { gi = best % N; gj = Math.floor(best / N); }
  }
  const targetK = gj * N + gi;
  const gScore = new Float32Array(N * N).fill(1e9);
  const cameFrom = new Int32Array(N * N).fill(-1);
  const inQ = new Uint8Array(N * N);
  const qK = [startK];
  const qF = [0];
  gScore[startK] = 0;
  inQ[startK] = 1;
  const dX = [1, -1, 0, 0, 1, -1, 1, -1];
  const dY = [0, 0, 1, -1, 1, 1, -1, -1];
  const dC = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414];
  let endK = -1;
  while (qK.length > 0) {
    let bi = 0;
    for (let i = 1; i < qK.length; i++) { if (qF[i] < qF[bi]) bi = i; }
    const curK = qK.splice(bi, 1)[0];
    qF.splice(bi, 1);
    inQ[curK] = 0;
    if (curK === targetK) { endK = curK; break; }
    const ci = curK % N, cj = Math.floor(curK / N);
    const curG = gScore[curK];
    for (let dir = 0; dir < 8; dir++) {
      const ni = ci + dX[dir], nj = cj + dY[dir];
      if (ni < 0 || ni >= N || nj < 0 || nj >= N) continue;
      const nk = nj * N + ni;
      if (costmap[nk] < safeEff) continue;
      const d = costmap[nk];
      const penalty = d >= 0.4 ? 0 : 25 * Math.pow((0.4 - d) / 0.4, 2);
      const tentG = curG + dC[dir] * RES * (1 + penalty);
      if (tentG < gScore[nk]) {
        cameFrom[nk] = curK;
        gScore[nk] = tentG;
        const h = Math.hypot(ni - gi, nj - gj) * RES;
        const f = tentG + h;
        if (!inQ[nk]) {
          qK.push(nk);
          qF.push(f);
          inQ[nk] = 1;
        }
      }
    }
  }
  if (endK === -1) return null;
  const raw = [];
  let curr = endK;
  while (curr !== -1) {
    raw.push({ x: (curr % N + 0.5) * RES, y: (Math.floor(curr / N) + 0.5) * RES });
    curr = cameFrom[curr];
  }
  raw.reverse();
  return raw;
};

let targetGoalX = GX;
let targetGoalY = GY;
let isPushingCube = false;

if (undelivered.length === 0) {
  targetGoalX = GX;
  targetGoalY = GY;
  if (Math.hypot(GX - X, GY - Y) < 0.65) {
    robot.setVelocity(0, 0, 0);
    return;
  }
} else {
  undelivered.sort((a, b) => a.id - b.id);
  const targetId = (sensors.cargo && sensors.cargo.activeTarget) ? sensors.cargo.activeTarget.id : (deliveredCount + 1);
  const tc = undelivered.find(c => c.id === targetId) || undelivered[0];
  const distToCube = Math.hypot(tc.x - X, tc.y - Y);
  const cosT = Math.cos(TH);
  const sinT = Math.sin(TH);
  const lx = (tc.x - X) * cosT + (tc.y - Y) * sinT;
  const ly = -(tc.x - X) * sinT + (tc.y - Y) * cosT;
  const inScoopContact = (lx >= 0.08 && lx <= 0.35 && Math.abs(ly) <= 0.14);

  if (inScoopContact) {
    isPushingCube = true;
    targetGoalX = GX;
    targetGoalY = GY;
  } else {
    const toGx = GX - tc.x;
    const toGy = GY - tc.y;
    const gDist = Math.hypot(toGx, toGy) || 1;
    const behindDist = 0.22;
    targetGoalX = tc.x - (toGx / gDist) * behindDist;
    targetGoalY = tc.y - (toGy / gDist) * behindDist;
    if (distToCube < 0.38) {
      const angToCube = Math.atan2(tc.y - Y, tc.x - X);
      const diff = wrap(angToCube - TH);
      if (Math.abs(diff) > 0.12) {
        robot.setVelocity(0.06, 0, Math.max(-2.2, Math.min(2.2, 3.2 * diff)));
      } else {
        robot.setVelocity(0.25, 0, Math.max(-1.2, Math.min(1.2, 1.5 * diff)));
      }
      return;
    }
  }
}

M.replanTimer = (M.replanTimer || 0) + dt;
if (M.path.length === 0 || M.replanTimer > 0.8) {
  const costmap = buildCostmap();
  const p = astar(costmap, X, Y, targetGoalX, targetGoalY, 0.04) || astar(costmap, X, Y, targetGoalX, targetGoalY, 0.015);
  if (p) {
    M.path = p;
    M.pi = 0;
    if (robot.setPlannedPath) robot.setPlannedPath(p);
  }
  M.replanTimer = 0;
}

if (M.path.length > 0) {
  while (M.pi < M.path.length - 1 && Math.hypot(M.path[M.pi].x - X, M.path[M.pi].y - Y) < 0.20) {
    M.pi++;
  }
  const pt = M.path[M.pi];
  const targetAng = Math.atan2(pt.y - Y, pt.x - X);
  const angDiff = wrap(targetAng - TH);
  const distToGoal = Math.hypot(targetGoalX - X, targetGoalY - Y);
  const maxSpd = isPushingCube ? 0.28 : 0.35;
  const spd = Math.max(0.14, Math.min(maxSpd, distToGoal));
  if (Math.abs(angDiff) > 0.40) {
    robot.setVelocity(0.06, 0, Math.max(-2.5, Math.min(2.5, 3.5 * angDiff)));
  } else {
    robot.setVelocity(spd * Math.cos(angDiff), 0, Math.max(-2.0, Math.min(2.0, 2.5 * angDiff)));
  }
} else {
  const targetAng = Math.atan2(targetGoalY - Y, targetGoalX - X);
  const angDiff = wrap(targetAng - TH);
  robot.setVelocity(0.12, 0, Math.max(-1.8, Math.min(1.8, 2.0 * angDiff)));
}
`,

      // 2. Goal Seeking with Artificial Potential Field
      goal_seeker: `/**
 * ROBO-ROARZ POTENTIAL FIELD GOAL SEEKER
 * Platform: Single-Block Smorphi
 */

if (!memory.init) {
  memory.init = true;
  robot.log("Target Seeking Navigator Started!");
}

const target = sensors.target;
const frontDist = sensors.lidar.getFront(35);
const leftDist = sensors.lidar.getLeft(45);
const rightDist = sensors.lidar.getRight(45);

if (target.reached) {
  robot.setVelocity(0, 0, 0);
  robot.log("Zona Target Tercapai!");
  return;
}

// 1. Attractive force towards target
let targetAngle = target.angle;
let attractiveVx = Math.cos(targetAngle) * 0.32;
let attractiveVy = Math.sin(targetAngle) * 0.32;

// 2. Repulsive force from obstacles
let repulseVx = 0;
let repulseVy = 0;

if (frontDist < 0.65) {
  const urgency = (0.65 - frontDist) / 0.65;
  repulseVx -= urgency * 0.45;
  if (leftDist > rightDist) {
    repulseVy += urgency * 0.35;
  } else {
    repulseVy -= urgency * 0.35;
  }
}

let cmdVx = attractiveVx + repulseVx;
let cmdVy = attractiveVy + repulseVy;
let cmdOmega = targetAngle * 0.8;

robot.setVelocity(cmdVx, cmdVy, cmdOmega);
`,

      // 3. Mecanum Holonomic Orbit (No-Turn)
      holonomic_drift: `/**
 * MECANUM HOLONOMIC ORBIT DEMO
 * Demonstrates 3-DOF lateral strafing without altering robot heading.
 */

if (!memory.timer) memory.timer = 0;
memory.timer += dt;

const phase = (memory.timer % 4.0) / 4.0;
const angle = phase * Math.PI * 2;

// Move along a circle in local frame without rotating
const vx = Math.cos(angle) * 0.30;
const vy = Math.sin(angle) * 0.30;

robot.setVelocity(vx, vy, 0.0);
`,

      // 4. PID Wall Follower
      wall_follower: `/**
 * PID WALL FOLLOWER (Single-Block Smorphi)
 * Maintains constant 0.35m distance from the right wall.
 */

const targetDist = 0.35;
const currentDist = sensors.lidar.getRight(30);

if (!memory.prevError) memory.prevError = 0;

const error = targetDist - currentDist;
const derivative = (error - memory.prevError) / dt;
memory.prevError = error;

const Kp = 1.8;
const Kd = 0.4;
const steer = Kp * error + Kd * derivative;

robot.setVelocity(0.28, 0.0, -steer);
`,
    };
    this.presets.cargo_retrieval = this.presets.default_avoidance;
  }

  /**
   * Compile user script text into an executable function
   * @param {string} codeText
   */
  compileScript(codeText) {
    this.hasError = false;
    this.lastErrorMessage = null;

    try {
      this.compiledFunction = new Function("sensors", "robot", "memory", "dt", codeText);
      this.memory = {};
      this.log("Code compiled successfully.");
      return { success: true };
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Syntax Error: ${err.message}`;
      this.log(`[SYNTAX ERROR] ${err.message}`, "error");
      return { success: false, error: err.message };
    }
  }

  /**
   * Execute one simulation tick of user code
   * @param {Object} sensorsInput - Read-only sensor snapshot
   * @param {SmorphiRobot} robot - Robot model instance
   * @param {number} dt - Delta time
   */
  executeTick(sensorsInput, robot, dt) {
    if (!this.isRunning || !this.compiledFunction || this.hasError) {
      return;
    }

    const robotAPI = {
      setVelocity: (vx, vy, omega) => {
        robot.setVelocity(vx, vy, omega);
      },
      setShape: (shape) => {
        return true;
      },
      getLoadedMass: () => {
        return robot.getLoadedMass();
      },
      stop: () => {
        robot.stop();
      },
      resetPose: (x, y, theta) => {
        robot.resetPose(x, y, theta);
      },
      setPlannedPath: (points) => {
        robot.debugPath = Array.isArray(points) ? points : [];
      },
      setGoalMarker: (x, y) => {
        robot.debugGoal = { x, y };
      },
      log: (msg) => {
        this.log(String(msg), "user");
      },
    };

    try {
      this.compiledFunction(sensorsInput, robotAPI, this.memory, dt);
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Runtime Error: ${err.message}`;
      this.log(`[RUNTIME ERROR] ${err.message}`, "error");
      robot.stop();
    }
  }

  /**
   * Logging facility for simulator console
   */
  log(message, type = "info") {
    const now = performance.now();
    if (type === "user") {
      if (!this._logWindowStart || now - this._logWindowStart > 100) {
        this._logWindowStart = now;
        this._logCountWindow = 0;
      }
      this._logCountWindow++;
      if (this._logCountWindow > 10) return;
    }
    this.lastLogTime = now;

    const entry = {
      time: new Date().toLocaleTimeString(),
      type,
      text: message,
    };

    for (const cb of this.logCallbacks) {
      cb(entry);
    }
  }

  onLog(callback) {
    this.logCallbacks.push(callback);
  }

  start() {
    this.isRunning = true;
    this.hasError = false;
  }

  stop() {
    this.isRunning = false;
  }

  resetMemory() {
    this.memory = {};
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = CodeEngine;
}
