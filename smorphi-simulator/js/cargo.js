/**
 * Cargo Cubes Management & Rigid-Body Dynamics
 * Handles 3 physical Cargo Cubes: dimensions (90x90x90 mm, 350g),
 * floor friction drag (static/kinetic), scoop mesh containment,
 * Center of Mass (CoM) coupling, and inertial shifting under centrifugal force.
 */

class CargoCube {
  constructor(id, name, hex, css, label, x, y) {
    this.id = id;
    this.name = name;
    this.hex = hex;
    this.css = css;
    this.label = label;

    // Position and orientation
    this.x = x;
    this.y = y;
    this.theta = 0;

    // Velocities
    this.vx = 0.0;
    this.vy = 0.0;
    this.omega = 0.0;

    // Physical properties
    this.size = CONFIG.CARGO.SIZE; // 0.090 m
    this.halfSize = this.size / 2; // 0.045 m
    this.mass = CONFIG.CARGO.MASS; // 0.35 kg

    // Interaction State: "UNTOUCHED" | "PUSHING_ON_GROUND" | "CAPTURED_INSIDE_MESH"
    this.state = "UNTOUCHED";
    this.captureSlot = -1; // 0, 1, 2
    this.slotOffsetX = 0.145; // default center of scoop along robot heading
    this.slotOffsetY = 0.0;

    // Inertial shifting inside mesh cavity due to centrifugal / tangential force
    this.shakeOffsetX = 0.0;
    this.shakeOffsetY = 0.0;
    this.shakeVelY = 0.0;

    // Last known safe coordinates for recovery
    this.lastSafeX = x;
    this.lastSafeY = y;
  }

  /**
   * Reset cube to specified coordinates
   */
  reset(x, y) {
    this.x = x;
    this.y = y;
    this.theta = 0;
    this.vx = 0;
    this.vy = 0;
    this.omega = 0;
    this.state = "UNTOUCHED";
    this.captureSlot = -1;
    this.shakeOffsetX = 0;
    this.shakeOffsetY = 0;
    this.shakeVelY = 0;
    this.lastSafeX = x;
    this.lastSafeY = y;
  }

  /**
   * Get Oriented Bounding Box (OBB) for SAT collision detection
   */
  getOBB() {
    const half = this.halfSize;
    const cosT = Math.cos(this.theta);
    const sinT = Math.sin(this.theta);

    const corners = [
      { x: this.x + (-half) * cosT - (-half) * sinT, y: this.y + (-half) * sinT + (-half) * cosT },
      { x: this.x + ( half) * cosT - (-half) * sinT, y: this.y + ( half) * sinT + (-half) * cosT },
      { x: this.x + ( half) * cosT - ( half) * sinT, y: this.y + ( half) * sinT + ( half) * cosT },
      { x: this.x + (-half) * cosT - ( half) * sinT, y: this.y + (-half) * sinT + ( half) * cosT },
    ];

    const axes = [
      { x: cosT, y: sinT },
      { x: -sinT, y: cosT },
    ];

    return {
      cx: this.x,
      cy: this.y,
      halfW: half,
      halfH: half,
      corners,
      axes,
    };
  }

  /**
   * Update cube state, floor friction drag on ground
   * @param {number} dt - Delta time
   * @param {SmorphiRobot} robot
   */
  update(dt, robot) {
    // 1. Guard against NaN or infinite coordinates - restore to last safe position
    if (!Number.isFinite(this.x) || !Number.isFinite(this.y)) {
      this.x = Number.isFinite(this.lastSafeX) ? this.lastSafeX : 2.5;
      this.y = Number.isFinite(this.lastSafeY) ? this.lastSafeY : 2.5;
      this.vx = 0;
      this.vy = 0;
    } else {
      this.lastSafeX = this.x;
      this.lastSafeY = this.y;
    }

    if (!Number.isFinite(this.vx)) this.vx = 0;
    if (!Number.isFinite(this.vy)) this.vy = 0;
    if (!Number.isFinite(this.theta)) this.theta = 0;
    if (!Number.isFinite(this.omega)) this.omega = 0;

    // 2. Velocity magnitude limit (prevent explosive impulses)
    const maxSpd = 1.0;
    const speed = Math.hypot(this.vx, this.vy);
    if (speed > maxSpd) {
      const scale = maxSpd / speed;
      this.vx *= scale;
      this.vy *= scale;
    }

    // 3. UNTOUCHED, PUSHING_ON_GROUND, or DELIVERED_AT_GOAL: Simulating floor friction on ground
    const currentSpeed = Math.hypot(this.vx, this.vy);
    if (currentSpeed > 1e-4) {
      const g = 9.81;
      const muK = CONFIG.CARGO.FLOOR_FRICTION_KINETIC || 0.28;
      const frictionAccel = muK * g;
      const speedDelta = frictionAccel * dt;

      if (currentSpeed <= speedDelta) {
        this.vx = 0;
        this.vy = 0;
      } else {
        const factor = (currentSpeed - speedDelta) / currentSpeed;
        this.vx *= factor;
        this.vy *= factor;
      }

      this.x += this.vx * dt;
      this.y += this.vy * dt;
    }

    // 4. Rotational damping on floor
    if (Math.abs(this.omega) > 1e-4) {
      const rotDamping = 8.0 * dt;
      if (Math.abs(this.omega) <= rotDamping) {
        this.omega = 0;
      } else {
        this.omega -= Math.sign(this.omega) * rotDamping;
      }
      this.theta += this.omega * dt;
    }

    // 5. Hard arena boundary clamping (Half size is 0.045m, safety inset 0.010m -> [0.055, 4.945])
    this.x = Math.max(0.055, Math.min(4.945, this.x));
    this.y = Math.max(0.055, Math.min(4.945, this.y));
  }
}

class CargoManager {
  constructor() {
    this.cubes = [];
    this.initCubes();
  }

  /**
   * Instantiate 3 Cargo Cubes with configured theme colors and labels
   */
  initCubes() {
    const cargoConfigs = CONFIG.CARGO.COLORS;
    this.cubes = [
      new CargoCube(1, cargoConfigs[0].name, cargoConfigs[0].hex, cargoConfigs[0].css, cargoConfigs[0].label, 1.8, 1.2),
      new CargoCube(2, cargoConfigs[1].name, cargoConfigs[1].hex, cargoConfigs[1].css, cargoConfigs[1].label, 2.5, 3.2),
      new CargoCube(3, cargoConfigs[2].name, cargoConfigs[2].hex, cargoConfigs[2].css, cargoConfigs[2].label, 3.6, 2.2),
    ];
  }

  /**
   * Spawns cubes scattered across distinct regions of the arena
   * so the robot must navigate through obstacles to find and deliver each one.
   * @param {ArenaMap} map
   */
  spawnCubes(map) {
    if (!map) return;

    const W = map.width || 5.0;
    const H = map.height || 5.0;
    const RES = 0.10;
    const N = Math.round(W / RES);
    const robotR = 0.165;
    const clearanceReq = 0.04;

    const distMap = new Float32Array(N * N);
    for (let j = 0; j < N; j++) {
      const cy = (j + 0.5) * RES;
      const wy = Math.min(cy - robotR, H - cy - robotR);
      for (let i = 0; i < N; i++) {
        const cx = (i + 0.5) * RES;
        let d = Math.min(wy, cx - robotR, W - cx - robotR);
        if (map.obstacles) {
          for (const obs of map.obstacles) {
            const nx = Math.max(obs.x, Math.min(cx, obs.x + obs.w));
            const ny = Math.max(obs.y, Math.min(cy, obs.y + obs.h));
            const od = Math.hypot(cx - nx, cy - ny) - robotR;
            if (od < d) d = od;
          }
        }
        distMap[j * N + i] = d;
      }
    }

    const regions = [
      { xMin: 0.8, xMax: 2.2, yMin: 1.0, yMax: 3.2 },
      { xMin: 2.8, xMax: 4.2, yMin: 1.0, yMax: 3.0 },
      { xMin: 0.8, xMax: 3.0, yMin: 3.2, yMax: 4.3 },
    ];

    const chosenPoints = [];
    const spawnPt = map.spawn || { x: 2.5, y: 0.6 };
    const goalPt = map.goal || { x: 4.2, y: 4.2 };

    for (let r = 0; r < regions.length; r++) {
      const reg = regions[r];
      const candidates = [];
      const i0 = Math.max(0, Math.floor(reg.xMin / RES));
      const i1 = Math.min(N - 1, Math.floor(reg.xMax / RES));
      const j0 = Math.max(0, Math.floor(reg.yMin / RES));
      const j1 = Math.min(N - 1, Math.floor(reg.yMax / RES));

      for (let j = j0; j <= j1; j++) {
        const cy = (j + 0.5) * RES;
        for (let i = i0; i <= i1; i++) {
          const cx = (i + 0.5) * RES;
          const d = distMap[j * N + i];
          if (d < clearanceReq) continue;
          if (Math.hypot(cx - spawnPt.x, cy - spawnPt.y) < 0.70) continue;
          if (Math.hypot(cx - goalPt.x, cy - goalPt.y) < 0.90) continue;

          let tooClose = false;
          for (const p of chosenPoints) {
            if (Math.hypot(cx - p.x, cy - p.y) < 1.1) {
              tooClose = true;
              break;
            }
          }
          if (tooClose) continue;

          candidates.push({ x: cx, y: cy, clearance: d });
        }
      }

      if (candidates.length > 0) {
        candidates.sort((a, b) => b.clearance - a.clearance);
        const topN = Math.max(1, Math.min(10, Math.floor(candidates.length * 0.25)));
        const picked = candidates[Math.floor(Math.random() * topN)];
        chosenPoints.push(picked);
      } else {
        const path = map.reachablePath || [];
        const fallbackIdx = Math.floor(path.length * (0.25 + r * 0.25));
        if (path[fallbackIdx]) {
          chosenPoints.push({ x: path[fallbackIdx].x, y: path[fallbackIdx].y, clearance: 0.1 });
        } else {
          chosenPoints.push({ x: 1.5 + r * 1.0, y: 2.0, clearance: 0.1 });
        }
      }
    }

    for (let i = 0; i < 3; i++) {
      const pt = chosenPoints[i] || { x: 1.5 + i * 1.0, y: 2.0 };
      this.cubes[i].reset(pt.x, pt.y);
    }
  }

  /**
   * Update physics of all cubes and resolve scoop ground-pushing interactions
   */
  update(dt, robot, map) {
    let pushingCount = 0;
    let deliveredCount = 0;
    let hasScoopContact = false;
    let minProximity = 5.0;

    const cosT = Math.cos(robot.theta);
    const sinT = Math.sin(robot.theta);

    for (const cube of this.cubes) {
      // Check if cube is delivered to the yellow goal marker
      const goalRadius = (CONFIG.ARENA && CONFIG.ARENA.GOAL_RADIUS) || 0.45;
      const distToGoal = (map && map.goal) ? Math.hypot(cube.x - map.goal.x, cube.y - map.goal.y) : 999;
      if (distToGoal <= goalRadius + 0.03 || (cube.state === "DELIVERED_AT_GOAL" && distToGoal <= goalRadius + 0.10)) {
        cube.state = "DELIVERED_AT_GOAL";
        deliveredCount++;
      }

      // Relative vector from robot center to cube center
      const dx = cube.x - robot.x;
      const dy = cube.y - robot.y;

      // Project into robot body frame (lx = forward, ly = left)
      const lx = dx * cosT + dy * sinT;
      const ly = -dx * sinT + dy * cosT;
      const dist = Math.hypot(dx, dy);

      if (lx > 0 && Math.abs(ly) < 0.25 && dist < minProximity) {
        minProximity = dist;
      }

      // Scoop physical push interaction:
      // Scoop extends from mount x=0.085 to x=0.225. Inner half-width is 0.075m.
      // Cube half size is 0.045m.
      // Minimum lx for cube center is scoop backplate (0.085) + cube.halfSize (0.045) = 0.130m.
      const inFrontOfScoop = (lx >= 0.08 && lx <= 0.28 && Math.abs(ly) <= 0.11);

      if (inFrontOfScoop) {
        hasScoopContact = true;
        if (cube.state !== "DELIVERED_AT_GOAL") {
          cube.state = "PUSHING_ON_GROUND";
          pushingCount++;
        }

        // Scoop backplate physical non-penetration constraint (pushing forward)
        if (lx < 0.135) {
          const pushForward = Math.min(0.035, 0.135 - lx);
          cube.x += pushForward * cosT;
          cube.y += pushForward * sinT;
        }

        // Side prongs centering guide (keeps cube centered in scoop during pushing)
        if (Math.abs(ly) > 0.045) {
          const centerPushY = Math.max(-0.025, Math.min(0.025, -Math.sign(ly) * (Math.abs(ly) - 0.045) * 0.5));
          cube.x += -centerPushY * sinT;
          cube.y += centerPushY * cosT;
        }

        // Transfer forward movement: cube slides across ground with robot
        if (robot.vx > 0.01) {
          cube.vx = robot.globalVx;
          cube.vy = robot.globalVy;
        }
      } else {
        if (cube.state === "PUSHING_ON_GROUND") {
          const spd = Math.hypot(cube.vx, cube.vy);
          if (spd < 0.01 && dist > 0.35) {
            cube.state = (distToGoal <= goalRadius + 0.03) ? "DELIVERED_AT_GOAL" : "UNTOUCHED";
          }
        }
      }

      cube.update(dt, robot);
    }

    // Update robot loaded mass, CoM offset, and traction slip from pushing
    robot.updateCargoState(pushingCount, hasScoopContact, minProximity);
  }

  /**
   * Get formatted telemetry summary for CodeEngine and TelemetryDashboard
   */
  getTelemetrySummary(robot) {
    const cosT = Math.cos(robot.theta);
    const sinT = Math.sin(robot.theta);

    // Sequential waypoint dispatch: Find next undelivered cube by ID order (1 -> 2 -> 3)
    const activeCube = this.cubes.find(c => c.state !== "DELIVERED_AT_GOAL") || null;
    let activeTarget = null;
    if (activeCube) {
      const dx = activeCube.x - robot.x;
      const dy = activeCube.y - robot.y;
      const dist = Math.hypot(dx, dy);
      const targetAngle = Math.atan2(dy, dx);
      let relAngle = targetAngle - robot.theta;
      while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
      while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

      activeTarget = {
        id: activeCube.id,
        name: activeCube.name,
        color: activeCube.name,
        hex: activeCube.hex,
        css: activeCube.css,
        label: activeCube.label,
        state: activeCube.state,
        x: activeCube.x,
        y: activeCube.y,
        dist: dist,
        relAngle: relAngle,
        relAngleDeg: (relAngle * 180) / Math.PI,
      };
    }

    const cubeDetails = this.cubes.map(cube => {
      const dx = cube.x - robot.x;
      const dy = cube.y - robot.y;
      const dist = Math.hypot(dx, dy);
      const targetAngle = Math.atan2(dy, dx);
      let relAngle = targetAngle - robot.theta;
      while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
      while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

      const isTarget = activeCube && (cube.id === activeCube.id);
      const isUnlocked = activeCube ? (cube.id <= activeCube.id) : true;

      return {
        id: cube.id,
        name: cube.name,
        color: cube.name,
        hex: cube.hex,
        css: cube.css,
        label: cube.label,
        state: cube.state,
        isDelivered: cube.state === "DELIVERED_AT_GOAL",
        isTarget: isTarget,
        isUnlocked: isUnlocked,
        dist: dist,
        relAngle: relAngle,
        relAngleDeg: (relAngle * 180) / Math.PI,
        x: cube.x,
        y: cube.y,
      };
    });

    const deliveredCount = this.cubes.filter(c => c.state === "DELIVERED_AT_GOAL").length;
    const pushingCount = this.cubes.filter(c => c.state === "PUSHING_ON_GROUND").length;

    return {
      count: deliveredCount,
      deliveredCount: deliveredCount,
      pushingCount: pushingCount,
      activeTarget: activeTarget,
      targetCube: activeTarget,
      nextTargetId: activeTarget ? activeTarget.id : null,
      totalMassKg: deliveredCount * CONFIG.CARGO.MASS,
      loadPercent: (deliveredCount / CONFIG.CARGO.COUNT) * 100,
      isFull: deliveredCount >= CONFIG.CARGO.COUNT,
      hasContact: robot.hasScoopContact,
      frontClearance: robot.scoopProximity,
      comOffsetX: robot.comOffsetX,
      cubes: cubeDetails,
    };
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { CargoCube, CargoManager };
}
