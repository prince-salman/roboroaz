/**
 * Interactive Code Injection & Script Execution Engine
 * Compiles and executes user JavaScript navigation scripts in real-time,
 * provides sandboxed error isolation, persistent memory across ticks, and preset algorithms.
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
      // 0. Whiteboard Sense-Think-Act Waypoints Navigator (5s Interval)
      whiteboard_waypoints: `function planGridPath(startX, startY, goalX, goalY, obstacles, lidarRanges, robotPose) {
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
`,

      // 1. User-Requested Default Autonomous Obstacle Avoidance Template
      default_avoidance: `/**
 * ROBO-ROARZ AUTONOMOUS OBSTACLE AVOIDANCE & MORPHING
 * ----------------------------------------------------
 * Inputs:
 *   - sensors.lidar: 360-deg laser array (ranges, .getFront(), .getLeft(), .getRight())
 *   - sensors.imu:   { heading, yaw_rate }
 *   - sensors.pose:  { x, y, theta }
 *   - sensors.shape: Active shape ("O", "I", "L", "T", "Z", "S")
 *   - sensors.target:{ distance, angle, reached }
 *
 * Outputs:
 *   - robot.setVelocity(vx, vy, omega): Set holonomic speed (m/s, rad/s)
 *   - robot.setShape(shape): Morph into "I" | "O" | "L" | "T" | "Z" | "S"
 *   - robot.log(message): Output text to simulator console
 */

// Initialize state machine
if (!memory.initialized) {
  memory.state = "CRUISE";
  memory.dodgeDirection = 1; // 1 = Left, -1 = Right
  memory.stuckTimer = 0;
  memory.initialized = true;
  robot.setShape("O"); // Start with standard stable 2x2 shape
  robot.log("RoboRoarZ Autonomous Script Initialized.");
}

// 1. Read LiDAR Distance Sectors
const frontDist = sensors.lidar.getFront(30);  // Min dist in [-30°, +30°]
const leftDist  = sensors.lidar.getLeft(40);   // Min dist on left flank
const rightDist = sensors.lidar.getRight(40);  // Min dist on right flank
const backDist  = sensors.lidar.getBack(30);

// 2. Narrow Corridor Detection Logic
// If both left and right walls are close (< 0.38m), we are entering a narrow passage!
const isNarrowCorridor = (leftDist < 0.38 && rightDist < 0.38);

if (isNarrowCorridor) {
  if (sensors.shape !== "I") {
    robot.log(">>> Narrow corridor detected! Morphing to streamlined 'I' shape...");
    robot.setShape("I");
  }
  // Drive forward slowly and smoothly through the corridor
  const lateralCorrection = (leftDist - rightDist) * 0.4;
  robot.setVelocity(0.20, lateralCorrection, 0.0);
  return;
}

// 3. Front Obstacle Avoidance Logic (< 0.6 m threshold)
if (frontDist < 0.60) {
  memory.stuckTimer += dt;

  // Decide bypass direction based on open space
  if (leftDist > rightDist) {
    memory.dodgeDirection = 1; // Crab/turn left
  } else {
    memory.dodgeDirection = -1; // Crab/turn right
  }

  // Use Mecanum Holonomic capability:
  // Combine lateral strafing (crabbing) with slight reverse & rotation
  const strafeSpeed = 0.25 * memory.dodgeDirection;
  const turnSpeed = 1.2 * memory.dodgeDirection;
  const reverseSpeed = frontDist < 0.30 ? -0.10 : 0.0;

  robot.setVelocity(reverseSpeed, strafeSpeed, turnSpeed);

  if (Math.random() < 0.02) {
    robot.log(\`Obstacle at \${frontDist.toFixed(2)}m -> Crabbing \${memory.dodgeDirection > 0 ? 'LEFT' : 'RIGHT'}\`);
  }
} else {
  // Clear path ahead: Cruise forward at nominal speed
  memory.stuckTimer = 0;

  // If in open space with shape "I", return to "O" for optimal turning stability
  if (sensors.shape === "I" && leftDist > 0.65 && rightDist > 0.65) {
    robot.log("Open area reached. Restoring 'O' shape.");
    robot.setShape("O");
  }

  robot.setVelocity(0.35, 0.0, 0.0);
}
`,

      // 2. Goal Seeking with Artificial Potential Field & Dynamic Morphing
      goal_seeker: `/**
 * ROBO-ROARZ GOAL-SEEKING & RECONFIGURATION NAVIGATOR
 * Combines attractive goal vector with LiDAR repulsive obstacle forces.
 */

if (!memory.init) {
  memory.init = true;
  robot.setShape("O");
  robot.log("Target Seeking Navigator Started!");
}

const target = sensors.target;
const frontDist = sensors.lidar.getFront(35);
const leftDist = sensors.lidar.getLeft(45);
const rightDist = sensors.lidar.getRight(45);

// Check if Goal Reached!
if (target.reached) {
  robot.setVelocity(0, 0, 0);
  robot.log("MISSION ACCOMPLISHED: Target Objective Reached!");
  return;
}

// Check for tight choke points on the way to goal
if (leftDist < 0.35 && rightDist < 0.35) {
  robot.setShape("I"); // Morph to squeeze through
} else if (sensors.shape === "I" && leftDist > 0.6 && rightDist > 0.6) {
  robot.setShape("O");
}

// 1. Attractive force towards target
let targetAngle = target.angle; // radians relative to heading
let attractiveVx = Math.cos(targetAngle) * 0.32;
let attractiveVy = Math.sin(targetAngle) * 0.32;

// 2. Repulsive force from obstacles
let repulseVx = 0;
let repulseVy = 0;

if (frontDist < 0.65) {
  const urgency = (0.65 - frontDist) / 0.65;
  repulseVx -= urgency * 0.45;
  // Push toward clearer side
  if (leftDist > rightDist) {
    repulseVy += urgency * 0.35;
  } else {
    repulseVy -= urgency * 0.35;
  }
}

// Combine forces for Mecanum holonomic locomotion
let vx = attractiveVx + repulseVx;
let vy = attractiveVy + repulseVy;
let omega = targetAngle * 1.5; // Rotate to face target

// Keep rotation smooth
omega = Math.max(-2.0, Math.min(2.0, omega));

robot.setVelocity(vx, vy, omega);
`,

      // 3. Mecanum Holonomic Omnidirectional Strafe Demo
      holonomic_drift: `/**
 * MECANUM HOLONOMIC DRIFT & ORBIT DEMO
 * Demonstrates 3-DOF crabbing (lateral motion) without turning heading!
 */

if (!memory.t) {
  memory.t = 0;
  robot.setShape("O");
  robot.log("Holonomic Mecanum Strafe Demo Initialized.");
}

memory.t += dt;

// Circular drift trajectory:
// Moves sideways and forward while keeping heading fixed at 0 rad!
const speed = 0.28;
const vx = Math.cos(memory.t * 0.8) * speed;
const vy = Math.sin(memory.t * 0.8) * speed;

// Check front LiDAR
if (sensors.lidar.getFront(25) < 0.4) {
  robot.setVelocity(-0.15, vy, 0);
} else {
  robot.setVelocity(vx, vy, 0.0); // Zero rotation! Pure holonomic translation!
}
`,

      // 4. Wall Follower (PID)
      wall_follower: `/**
 * PID RIGHT-WALL FOLLOWER
 * Maintains constant 0.38m distance to right wall using LiDAR
 */

if (!memory.pid) {
  memory.targetDist = 0.38;
  memory.prevError = 0;
  memory.integral = 0;
  memory.pid = true;
  robot.setShape("O");
  robot.log("Right Wall Follower Initialized.");
}

const frontDist = sensors.lidar.getFront(35);
const rightDist = sensors.lidar.getRight(40);

if (frontDist < 0.5) {
  // Obstacle ahead: Turn left immediately
  robot.setVelocity(0.05, 0.0, 1.5);
  return;
}

// PD Controller on wall distance
const error = rightDist - memory.targetDist;
const derivative = (error - memory.prevError) / dt;
memory.prevError = error;

const Kp = 1.8;
const Kd = 0.4;
const steer = Kp * error + Kd * derivative;

robot.setVelocity(0.28, 0.0, -steer);
`,
    };
  }

  /**
   * Compile user script text into an executable function
   * @param {string} codeText
   */
  compileScript(codeText) {
    this.hasError = false;
    this.lastErrorMessage = null;

    try {
      // Sandboxed function wrapping
      // Arguments: sensors, robot, memory, dt
      this.compiledFunction = new Function("sensors", "robot", "memory", "dt", codeText);
      this.memory = {}; // Reset persistent memory on fresh compile
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

    // Safe robot controller proxy
    const robotAPI = {
      setVelocity: (vx, vy, omega) => {
        robot.setVelocity(vx, vy, omega);
      },
      setShape: (shape) => {
        return robot.setShape(shape);
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
      robot.stop(); // Fail-safe stop
    }
  }

  /**
   * Logging facility for simulator console
   */
  log(message, type = "info") {
    const now = performance.now();
    // Throttle duplicate rapid logs
    if (type === "user" && now - this.lastLogTime < 50) return;
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
