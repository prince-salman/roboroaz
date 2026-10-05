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
      whiteboard_waypoints: `/**
 * WHITEBOARD SENSE-THINK-ACT AUTONOMOUS WAYPOINT NAVIGATOR
 * =========================================================
 * Implementation of Whiteboard Specification:
 * - Rules:
 *   1. Coordinate system origin must be (0, 0)
 *   2. Holonomic / omnidirectional wheel kinematics (Mecanum 3-DOF)
 *   3. 2D LiDAR = Eyes, IMU = Ears
 *   4. Interval 5 sec / point (Hold 5s at each waypoint)
 *
 * - Waypoints: (0, 2) -> (2, 3) -> (-2, 4)
 *
 * - Sense-Think-Act Architecture:
 *   1. Initialize: Origin (0,0), data streams, waypoints array
 *   2. Sense: Read IMU heading (ears), LiDAR ranges & odometry (eyes)
 *   3. Think: Coordinate error calculation, waypoint arrival check,
 *             5s timer countdown, LiDAR obstacle & corridor detection
 *   4. Act: Holonomic velocity transformation (Vx, Vy, Omega),
 *           morphing ("I" vs "O"), state transitions
 */

// ==========================================
// 1. INITIALIZE (Step 1 from Whiteboard)
// ==========================================
if (!memory.initialized) {
  memory.initialized = true;

  // Reposition robot to optimal arena center-bottom if starting at default corner
  // This ensures local coordinates (0, 2), (2, 3), (-2, 4) fit cleanly in the 5x5m arena
  if (sensors.pose.x < 1.5 && sensors.pose.y < 1.5 && robot.resetPose) {
    robot.resetPose(2.5, 0.6, Math.PI / 2);
  }

  // Rule 1: Set coordinate (0, 0) as local reference origin
  memory.originX = sensors.pose.x;
  memory.originY = sensors.pose.y;
  memory.originTheta = sensors.imu.headingRad;

  // Waypoints loaded from instruction (Whiteboard)
  memory.waypoints = [
    { id: 1, x: 0.0, y: 2.0 },
    { id: 2, x: 2.0, y: 3.0 },
    { id: 3, x: -2.0, y: 4.0 },
  ];

  memory.currentWptIndex = 0;
  memory.state = "NAVIGATING"; // "NAVIGATING" | "HOLD_AT_POINT" | "FINISHED"
  memory.holdTimer = 0.0;
  memory.HOLD_DURATION = 5.0; // Rule 4: Interval 5 sec / point
  memory.TOLERANCE = 0.28;    // Waypoint reached tolerance (meters)
  memory.lastLogSec = -1;

  robot.setShape("O"); // Initial stable 2x2 shape
  robot.log("================================================");
  robot.log("Whiteboard Sense-Think-Act Navigator Initialized!");
  robot.log(\`Rule 1: Origin (0,0) set at global (\${memory.originX.toFixed(2)}, \${memory.originY.toFixed(2)})\`);
  robot.log("Rule 2: Holonomic Mecanum 3-DOF Kinematics Active");
  robot.log("Rule 3: Eyes (2D LiDAR) & Ears (6-DOF IMU) Online");
  robot.log("Rule 4: 5.0s Interval Hold per Waypoint Activated");
  robot.log("Waypoints: 1:(0, 2) -> 2:(2, 3) -> 3:(-2, 4)");
  robot.log("================================================");
}

// ==========================================
// 2. SENSE (Step 2 from Whiteboard)
// ==========================================
// Ears: IMU heading and orientation
const headingRad = sensors.imu.headingRad; // Current yaw heading (-PI to PI)
const headingDeg = sensors.imu.heading;    // Current heading (0 to 360 deg)

// Eyes: 2D LiDAR Raycasting sectors
const frontDist = sensors.lidar.getFront(35); // Front cone [-35°, +35°]
const leftDist  = sensors.lidar.getLeft(45);  // Left flank
const rightDist = sensors.lidar.getRight(45); // Right flank
const backDist  = sensors.lidar.getBack(30);  // Rear cone

// Position tracking relative to origin (0, 0)
const localX = sensors.pose.x - memory.originX;
const localY = sensors.pose.y - memory.originY;

// ==========================================
// 3. THINK (Step 3 from Whiteboard)
// ==========================================

// Check if all waypoints have been visited
if (memory.currentWptIndex >= memory.waypoints.length) {
  memory.state = "FINISHED";
  robot.setVelocity(0, 0, 0);
  if (Math.random() < 0.02) {
    robot.log(">>> MISSION ACCOMPLISHED: All Waypoints Visited! <<<");
  }
  return;
}

const target = memory.waypoints[memory.currentWptIndex];
const errorX = target.x - localX;
const errorY = target.y - localY;
const distToTarget = Math.hypot(errorX, errorY);

// State A: Check if arrival condition met
if (memory.state === "NAVIGATING" && distToTarget < memory.TOLERANCE) {
  memory.state = "HOLD_AT_POINT";
  memory.holdTimer = 0.0;
  memory.lastLogSec = -1;
  robot.setVelocity(0, 0, 0);
  robot.log(\`>>> REACHED Waypoint #\${target.id} at (\${target.x}, \${target.y})! Starting 5-second interval hold... <<<\`);
}

// State B: Holding for 5 seconds (Rule 4)
if (memory.state === "HOLD_AT_POINT") {
  memory.holdTimer += dt;
  const currentSec = Math.floor(memory.holdTimer);

  if (currentSec !== memory.lastLogSec && currentSec <= 5) {
    const remaining = (memory.HOLD_DURATION - memory.holdTimer).toFixed(1);
    robot.log(\`Waypoint #\${target.id} Hold: \${memory.holdTimer.toFixed(1)}s / 5.0s (Remaining: \${remaining}s)\`);
    memory.lastLogSec = currentSec;
  }

  // Act: Hold position
  robot.setVelocity(0, 0, 0);

  // Transition after 5 seconds elapsed
  if (memory.holdTimer >= memory.HOLD_DURATION) {
    robot.log(\`>>> 5.0s Interval Complete for Waypoint #\${target.id}! Transitioning to next target... <<<\`);
    memory.currentWptIndex++;
    memory.state = "NAVIGATING";
    memory.holdTimer = 0.0;
  }
  return;
}

// State C: Navigating to active waypoint with Holonomic DWA & LiDAR Obstacle Avoidance
// Narrow corridor detection & shape morphing
if (leftDist < 0.36 && rightDist < 0.36) {
  if (sensors.shape !== "I") robot.setShape("I");
} else if (sensors.shape === "I" && leftDist > 0.60 && rightDist > 0.60) {
  robot.setShape("O");
}

const targetGlobalAngle = Math.atan2(errorY, errorX);
let targetAngleRel = targetGlobalAngle - headingRad;
while (targetAngleRel > Math.PI) targetAngleRel -= 2 * Math.PI;
while (targetAngleRel < -Math.PI) targetAngleRel += 2 * Math.PI;

let vx = Math.cos(targetAngleRel) * 0.32;
let vy = Math.sin(targetAngleRel) * 0.32;

// 2D LiDAR Obstacle Repulsion (Eyes) & Holonomic Crabbing
if (frontDist < 0.70) {
  const urgency = (0.70 - frontDist) / 0.70;
  vx -= urgency * 0.45;
  const dodgeDir = leftDist > rightDist ? 1 : -1;
  vy += dodgeDir * urgency * 0.50;
}
if (leftDist < 0.45) {
  vy -= ((0.45 - leftDist) / 0.45) * 0.35;
}
if (rightDist < 0.45) {
  vy += ((0.45 - rightDist) / 0.45) * 0.35;
}

const spd = Math.hypot(vx, vy);
if (spd > 0.40) {
  vx = (vx / spd) * 0.40;
  vy = (vy / spd) * 0.40;
}

// Heading alignment via IMU (Ears)
let headingError = targetGlobalAngle - headingRad;
while (headingError > Math.PI) headingError -= 2 * Math.PI;
while (headingError < -Math.PI) headingError += 2 * Math.PI;
const omega = Math.max(-1.5, Math.min(1.5, headingError * 1.5));

// ==========================================
// 4. ACT (Step 4 from Whiteboard)
// ==========================================
// Command Holonomic Wheels (vx: forward, vy: strafe crab, omega: rotation)
robot.setVelocity(vx, vy, omega);
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
