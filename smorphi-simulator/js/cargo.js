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
    // UNTOUCHED, PUSHING_ON_GROUND, or DELIVERED_AT_GOAL: Simulating floor friction on ground
    const speed = Math.hypot(this.vx, this.vy);
    if (speed > 1e-4) {
      const g = 9.81;
      const muK = CONFIG.CARGO.FLOOR_FRICTION_KINETIC || 0.28;
      const frictionAccel = muK * g;
      const speedDelta = frictionAccel * dt;

      if (speed <= speedDelta) {
        this.vx = 0;
        this.vy = 0;
      } else {
        const factor = (speed - speedDelta) / speed;
        this.vx *= factor;
        this.vy *= factor;
      }

      this.x += this.vx * dt;
      this.y += this.vy * dt;
    }

    // Rotational damping on floor
    if (Math.abs(this.omega) > 1e-4) {
      const rotDamping = 8.0 * dt;
      if (Math.abs(this.omega) <= rotDamping) {
        this.omega = 0;
      } else {
        this.omega -= Math.sign(this.omega) * rotDamping;
      }
      this.theta += this.omega * dt;
    }
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
   * Spawns cubes deterministically at safe open waypoints along the verified A* path
   * without altering the maze layout or obstacle generator.
   * @param {ArenaMap} map
   */
  spawnCubes(map) {
    const path = (map && map.reachablePath && map.reachablePath.length > 5) ? map.reachablePath : null;

    if (path) {
      const len = path.length;
      // Waypoint indices along path
      const idx1 = Math.floor(len * 0.25);
      const idx2 = Math.floor(len * 0.55);
      const idx3 = Math.floor(len * 0.80);

      const p1 = path[idx1];
      const p2 = path[idx2];
      const p3 = path[idx3];

      this.cubes[0].reset(p1.x, p1.y);
      this.cubes[1].reset(p2.x, p2.y);
      this.cubes[2].reset(p3.x, p3.y);
    } else {
      // Fallback default safe points
      const fallbacks = [
        { x: 1.8, y: 1.4 },
        { x: 2.5, y: 3.2 },
        { x: 3.8, y: 2.2 },
      ];

      for (let i = 0; i < 3; i++) {
        let pt = fallbacks[i];
        // Ensure within bounds
        pt.x = Math.max(0.5, Math.min(map.width - 0.5, pt.x));
        pt.y = Math.max(0.5, Math.min(map.height - 0.5, pt.y));
        this.cubes[i].reset(pt.x, pt.y);
      }
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
      const distToGoal = (map && map.goal) ? Math.hypot(cube.x - map.goal.x, cube.y - map.goal.y) : 999;
      if (distToGoal < 0.65 || (cube.state === "DELIVERED_AT_GOAL" && distToGoal < 0.90)) {
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
          const pushForward = 0.135 - lx;
          cube.x += pushForward * cosT;
          cube.y += pushForward * sinT;
        }

        // Side prongs centering guide (keeps cube centered in scoop during pushing)
        if (Math.abs(ly) > 0.045) {
          const centerPushY = -Math.sign(ly) * (Math.abs(ly) - 0.045) * 0.5;
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
            cube.state = (distToGoal < 0.65) ? "DELIVERED_AT_GOAL" : "UNTOUCHED";
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

    const cubeDetails = this.cubes.map(cube => {
      const dx = cube.x - robot.x;
      const dy = cube.y - robot.y;
      const dist = Math.hypot(dx, dy);
      const targetAngle = Math.atan2(dy, dx);
      let relAngle = targetAngle - robot.theta;
      while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
      while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

      return {
        id: cube.id,
        name: cube.name,
        color: cube.name,
        hex: cube.hex,
        css: cube.css,
        label: cube.label,
        state: cube.state,
        isDelivered: cube.state === "DELIVERED_AT_GOAL",
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
