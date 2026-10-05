function planGridPath(startX, startY, goalX, goalY, obstacles, lidarRanges, robotPose) {
  const N = 50;
  const cellW = 5.0 / N;
  const cellH = 5.0 / N;
  const grid = new Uint8Array(N * N);
  const margin = 0.16;

  for (let i = 0; i < N; i++) {
    grid[0 * N + i] = 1;
    grid[1 * N + i] = 1;
    grid[(N - 1) * N + i] = 1;
    grid[(N - 2) * N + i] = 1;
    grid[i * N + 0] = 1;
    grid[i * N + 1] = 1;
    grid[i * N + (N - 1)] = 1;
    grid[i * N + (N - 2)] = 1;
  }

  if (obstacles) {
    for (let k = 0; k < obstacles.length; k++) {
      const obs = obstacles[k];
      const minI = Math.max(0, Math.floor((obs.x - margin) / cellW));
      const maxI = Math.min(N - 1, Math.floor((obs.x + obs.w + margin) / cellW));
      const minJ = Math.max(0, Math.floor((obs.y - margin) / cellH));
      const maxJ = Math.min(N - 1, Math.floor((obs.y + obs.h + margin) / cellH));
      for (let j = minJ; j <= maxJ; j++) {
        for (let i = minI; i <= maxI; i++) {
          grid[j * N + i] = 1;
        }
      }
    }
  }

  if (lidarRanges && robotPose) {
    const rx = robotPose.x;
    const ry = robotPose.y;
    const rth = robotPose.theta;
    const inflCells = Math.ceil(margin / cellW);
    for (let deg = 0; deg < 360; deg += 6) {
      const dist = lidarRanges[deg];
      if (dist > 0.10 && dist < 3.5) {
        const rad = rth + (deg * Math.PI) / 180;
        const ox = rx + dist * Math.cos(rad);
        const oy = ry + dist * Math.sin(rad);
        const oi = Math.floor(ox / cellW);
        const oj = Math.floor(oy / cellH);
        for (let dj = -inflCells; dj <= inflCells; dj++) {
          for (let di = -inflCells; di <= inflCells; di++) {
            if (di * di + dj * dj <= inflCells * inflCells) {
              const ni = oi + di;
              const nj = oj + dj;
              if (ni >= 0 && ni < N && nj >= 0 && nj < N) {
                grid[nj * N + ni] = 1;
              }
            }
          }
        }
      }
    }
  }

  let startI = Math.max(2, Math.min(N - 3, Math.floor(startX / cellW)));
  let startJ = Math.max(2, Math.min(N - 3, Math.floor(startY / cellH)));
  let goalI = Math.max(2, Math.min(N - 3, Math.floor(goalX / cellW)));
  let goalJ = Math.max(2, Math.min(N - 3, Math.floor(goalY / cellH)));

  if (grid[startJ * N + startI] === 1) {
    for (let r = 1; r <= 8; r++) {
      let found = false;
      for (let dj = -r; dj <= r && !found; dj++) {
        for (let di = -r; di <= r && !found; di++) {
          const ni = startI + di, nj = startJ + dj;
          if (ni >= 2 && ni < N - 2 && nj >= 2 && nj < N - 2 && grid[nj * N + ni] === 0) {
            startI = ni; startJ = nj; found = true;
          }
        }
      }
      if (found) break;
    }
  }

  if (grid[goalJ * N + goalI] === 1) {
    for (let r = 1; r <= 10; r++) {
      let found = false;
      for (let dj = -r; dj <= r && !found; dj++) {
        for (let di = -r; di <= r && !found; di++) {
          const ni = goalI + di, nj = goalJ + dj;
          if (ni >= 2 && ni < N - 2 && nj >= 2 && nj < N - 2 && grid[nj * N + ni] === 0) {
            goalI = ni; goalJ = nj; found = true;
          }
        }
      }
      if (found) break;
    }
  }

  const queue = [[startI, startJ]];
  const visited = new Uint8Array(N * N);
  const parent = new Int32Array(N * N).fill(-1);
  visited[startJ * N + startI] = 1;

  let foundPath = false;
  const dirs = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [-1, 1], [1, -1], [-1, -1]];

  while (queue.length > 0) {
    const [ci, cj] = queue.shift();
    if (ci === goalI && cj === goalJ) {
      foundPath = true;
      break;
    }
    for (let d = 0; d < dirs.length; d++) {
      const ni = ci + dirs[d][0], nj = cj + dirs[d][1];
      if (ni >= 0 && ni < N && nj >= 0 && nj < N) {
        const idx = nj * N + ni;
        if (!visited[idx] && grid[idx] === 0) {
          visited[idx] = 1;
          parent[idx] = cj * N + ci;
          queue.push([ni, nj]);
        }
      }
    }
  }

  if (!foundPath) return [];

  const path = [];
  let curr = goalJ * N + goalI;
  while (curr !== -1) {
    const pj = Math.floor(curr / N);
    const pi = curr % N;
    path.unshift({ x: (pi + 0.5) * cellW, y: (pj + 0.5) * cellH });
    curr = parent[curr];
  }
  return path;
}

if (!memory.initialized) {
  memory.initialized = true;
  if (sensors.pose.x < 1.5 && sensors.pose.y < 1.5 && robot.resetPose) {
    robot.resetPose(2.5, 0.6, Math.PI / 2);
  }
  memory.originX = sensors.pose.x;
  memory.originY = sensors.pose.y;
  memory.waypoints = [
    { id: 1, x: 0.0, y: 2.0 },
    { id: 2, x: 2.0, y: 3.0 },
    { id: 3, x: -2.0, y: 4.0 }
  ];
  memory.currentWptIndex = 0;
  memory.state = "NAVIGATING";
  memory.holdTimer = 0.0;
  memory.HOLD_DURATION = 5.0;
  memory.TOLERANCE = 0.32;
  memory.lastLogSec = -1;
  memory.path = [];
  memory.pathIdx = 0;
  memory.replanCooldown = 0;
  robot.setShape("O");
  robot.log("Origin (0,0) set at pose (" + memory.originX.toFixed(2) + ", " + memory.originY.toFixed(2) + ")");
  robot.log("Sensors: 2D LiDAR (eyes) & IMU (ears) initialized");
  robot.log("Targets: 1:(0, 2) -> 2:(2, 3) -> 3:(-2, 4)");
}

const headingRad = sensors.imu.headingRad;
const rx = sensors.pose.x;
const ry = sensors.pose.y;
const localX = rx - memory.originX;
const localY = ry - memory.originY;

if (memory.currentWptIndex >= memory.waypoints.length) {
  memory.state = "FINISHED";
  robot.setVelocity(0, 0, 0);
  if (!memory.finishedLogged) {
    memory.finishedLogged = true;
    robot.log(">>> MISSION SUCCESS: All 3 Points Visited with 5-Second Intervals! <<<");
  }
  return;
}

const target = memory.waypoints[memory.currentWptIndex];
const errorX = target.x - localX;
const errorY = target.y - localY;
const distToTarget = Math.hypot(errorX, errorY);

const targetGlobalX = Math.max(0.3, Math.min(4.7, target.x + memory.originX));
const targetGlobalY = Math.max(0.3, Math.min(4.7, target.y + memory.originY));

if (robot.setGoalMarker) {
  robot.setGoalMarker(targetGlobalX, targetGlobalY);
}

if (memory.state === "NAVIGATING" && distToTarget < memory.TOLERANCE) {
  memory.state = "HOLD_AT_POINT";
  memory.holdTimer = 0.0;
  memory.lastLogSec = -1;
  robot.setVelocity(0, 0, 0);
  robot.log(">>> REACHED Point #" + target.id + " at (" + target.x + ", " + target.y + ")! Starting 5-second interval hold...");
}

if (memory.state === "HOLD_AT_POINT") {
  memory.holdTimer += dt;
  const currentSec = Math.floor(memory.holdTimer);
  if (currentSec !== memory.lastLogSec && currentSec <= 5) {
    robot.log("Point #" + target.id + " (" + target.x + ", " + target.y + ") Hold: " + memory.holdTimer.toFixed(1) + "s / 5.0s");
    memory.lastLogSec = currentSec;
  }
  robot.setVelocity(0, 0, 0);
  if (memory.holdTimer >= memory.HOLD_DURATION) {
    robot.log(">>> 5.0s Hold Complete for Point #" + target.id + "! Moving to next target...");
    memory.currentWptIndex++;
    memory.state = "NAVIGATING";
    memory.holdTimer = 0.0;
    memory.path = [];
    memory.pathIdx = 0;
  }
  return;
}

memory.replanCooldown += dt;
if (!memory.path || memory.path.length === 0 || (memory.replanCooldown > 1.2 && memory.pathIdx >= memory.path.length - 2)) {
  const lidarArr = sensors.lidar && sensors.lidar.ranges ? sensors.lidar.ranges : null;
  memory.path = planGridPath(rx, ry, targetGlobalX, targetGlobalY, sensors.obstacles, lidarArr, sensors.pose);
  memory.pathIdx = 0;
  memory.replanCooldown = 0;
  if (robot.setPlannedPath) {
    robot.setPlannedPath(memory.path);
  }
}

let carrotX = targetGlobalX;
let carrotY = targetGlobalY;

if (memory.path && memory.path.length > 0) {
  while (memory.pathIdx < memory.path.length - 1) {
    const pt = memory.path[memory.pathIdx];
    if (Math.hypot(pt.x - rx, pt.y - ry) < 0.28) {
      memory.pathIdx++;
    } else {
      break;
    }
  }
  const carrotIdx = Math.min(memory.path.length - 1, memory.pathIdx + 1);
  carrotX = memory.path[carrotIdx].x;
  carrotY = memory.path[carrotIdx].y;
}

const cdx = carrotX - rx;
const cdy = carrotY - ry;
const cdist = Math.hypot(cdx, cdy) || 0.001;

const speed = Math.min(0.38, Math.max(0.18, cdist * 1.5));
let globalVx = (cdx / cdist) * speed;
let globalVy = (cdy / cdist) * speed;

const cosH = Math.cos(headingRad);
const sinH = Math.sin(headingRad);
let localVx = globalVx * cosH + globalVy * sinH;
let localVy = -globalVx * sinH + globalVy * cosH;

const frontDist = sensors.lidar && sensors.lidar.getFront ? sensors.lidar.getFront(35) : 2.0;
const leftDist = sensors.lidar && sensors.lidar.getLeft ? sensors.lidar.getLeft(45) : 2.0;
const rightDist = sensors.lidar && sensors.lidar.getRight ? sensors.lidar.getRight(45) : 2.0;

if (frontDist < 0.32) {
  const urgency = (0.32 - frontDist) / 0.32;
  localVx -= urgency * 0.30;
  const dodge = leftDist > rightDist ? 1 : -1;
  localVy += dodge * urgency * 0.35;
}

const travelAngle = Math.atan2(cdy, cdx);
let headingError = travelAngle - headingRad;
while (headingError > Math.PI) headingError -= 2 * Math.PI;
while (headingError < -Math.PI) headingError += 2 * Math.PI;
const omega = Math.max(-1.8, Math.min(1.8, headingError * 1.5));

robot.setVelocity(localVx, localVy, omega);
