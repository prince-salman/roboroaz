/**
 * Physics Engine & Multi-Body Collision Detection
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop
 * Implements Separating Axis Theorem (SAT) for robot chassis, scoop prongs,
 * and 3 physical Cargo Cubes. Continuous contact resolution, wall sliding,
 * and realistic cargo containment physics.
 */

class PhysicsEngine {
  constructor(arenaMap) {
    this.map = arenaMap;
  }

  /**
   * Run physics simulation step with sub-stepping for anti-tunneling
   * @param {SmorphiRobot} robot
   * @param {number} dt - Full frame delta time
   * @param {CargoManager} [cargoManager]
   */
  step(robot, dt, cargoManager = null) {
    const numSubsteps = CONFIG.SIM.MAX_SUBSTEPS || 4;
    const subDt = dt / numSubsteps;

    robot.inCollision = false;

    for (let step = 0; step < numSubsteps; step++) {
      // 1. Integrate robot kinematics & dynamics
      robot.update(subDt);

      // 2. Resolve robot vs arena boundaries & obstacles
      this.resolveRobotCollisions(robot);

      // 3. Update & resolve Cargo Cubes dynamics
      if (cargoManager) {
        cargoManager.update(subDt, robot, this.map);
        this.resolveCargoCollisions(robot, cargoManager);
      }
    }
  }

  /**
   * Detect and resolve collisions between robot and arena obstacles/perimeter
   * @param {SmorphiRobot} robot
   */
  resolveRobotCollisions(robot) {
    const robotBoxes = robot.getModuleBoxes();

    // 1. Perimeter Boundary Wall Collisions
    for (const box of robotBoxes) {
      for (const corner of box.corners) {
        if (corner.x < 0.02) {
          robot.x += (0.02 - corner.x);
          robot.globalVx = Math.max(0, robot.globalVx);
          robot.inCollision = true;
        } else if (corner.x > this.map.width - 0.02) {
          robot.x -= (corner.x - (this.map.width - 0.02));
          robot.globalVx = Math.min(0, robot.globalVx);
          robot.inCollision = true;
        }

        if (corner.y < 0.02) {
          robot.y += (0.02 - corner.y);
          robot.globalVy = Math.max(0, robot.globalVy);
          robot.inCollision = true;
        } else if (corner.y > this.map.height - 0.02) {
          robot.y -= (corner.y - (this.map.height - 0.02));
          robot.globalVy = Math.min(0, robot.globalVy);
          robot.inCollision = true;
        }
      }
    }

    // 2. Arena Obstacles Collisions using SAT
    const rBound = robot.getBoundingRadius();

    for (const obs of this.map.obstacles) {
      const obsCenterX = obs.x + obs.w / 2;
      const obsCenterY = obs.y + obs.h / 2;
      const obsRadius = Math.hypot(obs.w, obs.h) / 2;

      // Broad-phase distance rejection
      if (Math.hypot(robot.x - obsCenterX, robot.y - obsCenterY) > (rBound + obsRadius)) {
        continue;
      }

      // Narrow-phase SAT test for each of the robot's OBBs
      const obsBox = {
        cx: obsCenterX,
        cy: obsCenterY,
        halfW: obs.w / 2,
        halfH: obs.h / 2,
        corners: [
          { x: obs.x, y: obs.y },
          { x: obs.x + obs.w, y: obs.y },
          { x: obs.x + obs.w, y: obs.y + obs.h },
          { x: obs.x, y: obs.y + obs.h },
        ],
        axes: [
          { x: 1, y: 0 },
          { x: 0, y: 1 },
        ],
      };

      for (const box of robotBoxes) {
        const collision = this.testOBBCollision(box, obsBox);
        if (collision.intersects) {
          robot.inCollision = true;
          robot.collisionCount++;

          // Apply Minimum Translation Vector (MTV) separation
          const pushDistance = collision.depth * 1.05;
          robot.x += collision.normal.x * pushDistance;
          robot.y += collision.normal.y * pushDistance;

          // Inelastic impulse / wall-sliding response
          const dot = robot.globalVx * collision.normal.x + robot.globalVy * collision.normal.y;
          if (dot < 0) {
            robot.globalVx -= dot * collision.normal.x * 1.1;
            robot.globalVy -= dot * collision.normal.y * 1.1;

            // Re-project into robot body frame
            const cosT = Math.cos(robot.theta);
            const sinT = Math.sin(robot.theta);
            robot.vx = robot.globalVx * cosT + robot.globalVy * sinT;
            robot.vy = -robot.globalVx * sinT + robot.globalVy * cosT;
          }
        }
      }
    }
  }

  /**
   * Detect and resolve collisions for Cargo Cubes against walls, obstacles, and robot scoop
   * @param {SmorphiRobot} robot
   * @param {CargoManager} cargoManager
   */
  resolveCargoCollisions(robot, cargoManager) {
    const cubes = cargoManager.cubes;
    const halfSize = CONFIG.CARGO.SIZE / 2; // 0.045m
    const boundMargin = halfSize + 0.010; // 0.055m

    const cosT = Math.cos(robot.theta);
    const sinT = Math.sin(robot.theta);

    for (const cube of cubes) {
      if (cube.state === "CAPTURED_INSIDE_MESH") {
        continue;
      }

      // 1. Cube vs Perimeter Walls (Clean AABB boundary clamping)
      if (cube.x < boundMargin) {
        cube.x = boundMargin;
        cube.vx = Math.max(0, cube.vx);
      } else if (cube.x > this.map.width - boundMargin) {
        cube.x = this.map.width - boundMargin;
        cube.vx = Math.min(0, cube.vx);
      }

      if (cube.y < boundMargin) {
        cube.y = boundMargin;
        cube.vy = Math.max(0, cube.vy);
      } else if (cube.y > this.map.height - boundMargin) {
        cube.y = this.map.height - boundMargin;
        cube.vy = Math.min(0, cube.vy);
      }

      // 2. Cube vs Obstacles (Continuous projection & non-penetration)
      let cubeHitObstacle = false;
      if (this.map.obstacles) {
        for (const obs of this.map.obstacles) {
          const obsCenterX = obs.x + obs.w / 2;
          const obsCenterY = obs.y + obs.h / 2;
          const obsRadius = Math.hypot(obs.w, obs.h) / 2;

          if (Math.hypot(cube.x - obsCenterX, cube.y - obsCenterY) > (cube.size + obsRadius)) {
            continue;
          }

          // Closest point on obstacle AABB
          const nx = Math.max(obs.x, Math.min(cube.x, obs.x + obs.w));
          const ny = Math.max(obs.y, Math.min(cube.y, obs.y + obs.h));
          const distX = cube.x - nx;
          const distY = cube.y - ny;
          const distSq = distX * distX + distY * distY;

          // If cube center is inside obstacle box (deep penetration), project to nearest external face
          if (cube.x >= obs.x && cube.x <= obs.x + obs.w && cube.y >= obs.y && cube.y <= obs.y + obs.h) {
            cubeHitObstacle = true;
            const dLeft = cube.x - obs.x;
            const dRight = (obs.x + obs.w) - cube.x;
            const dBottom = cube.y - obs.y;
            const dTop = (obs.y + obs.h) - cube.y;
            const minD = Math.min(dLeft, dRight, dBottom, dTop);

            if (minD === dLeft) {
              cube.x = obs.x - (halfSize + 0.005);
              cube.vx = Math.min(0, cube.vx);
            } else if (minD === dRight) {
              cube.x = obs.x + obs.w + (halfSize + 0.005);
              cube.vx = Math.max(0, cube.vx);
            } else if (minD === dBottom) {
              cube.y = obs.y - (halfSize + 0.005);
              cube.vy = Math.min(0, cube.vy);
            } else {
              cube.y = obs.y + obs.h + (halfSize + 0.005);
              cube.vy = Math.max(0, cube.vy);
            }
          } else if (distSq < (halfSize + 0.002) ** 2) {
            cubeHitObstacle = true;
            const dLen = Math.sqrt(distSq);
            const pen = (halfSize + 0.002) - dLen;
            if (dLen > 1e-4) {
              const pushX = (distX / dLen) * pen;
              const pushY = (distY / dLen) * pen;
              cube.x += pushX;
              cube.y += pushY;

              // Damp velocity along collision normal
              const dotV = cube.vx * (distX / dLen) + cube.vy * (distY / dLen);
              if (dotV < 0) {
                cube.vx -= dotV * (distX / dLen);
                cube.vy -= dotV * (distY / dLen);
              }
            }
          }
        }
      }

      // 3. Bidirectional Robot vs Cube Scoop Contact
      const dx = cube.x - robot.x;
      const dy = cube.y - robot.y;
      const lx = dx * cosT + dy * sinT;
      const ly = -dx * sinT + dy * cosT;
      const inScoopCavity = (lx >= 0.06 && lx <= 0.28 && Math.abs(ly) <= 0.12);

      if (inScoopCavity) {
        robot.hasScoopContact = true;
        // Scoop backplate physical non-penetration (lx = 0.135)
        if (lx < 0.135) {
          const penetration = 0.135 - lx;
          if (cubeHitObstacle) {
            // Cube is blocked against obstacle! Block robot from crushing cube!
            robot.x -= penetration * cosT;
            robot.y -= penetration * sinT;
            if (robot.vx > 0) robot.vx = 0;
            const dotRob = robot.globalVx * cosT + robot.globalVy * sinT;
            if (dotRob > 0) {
              robot.globalVx -= dotRob * cosT;
              robot.globalVy -= dotRob * sinT;
            }
            robot.inCollision = true;
          } else {
            // Free floor pushing: push cube forward
            const pushDist = Math.min(0.035, penetration);
            cube.x += pushDist * cosT;
            cube.y += pushDist * sinT;
          }
        }
      } else {
        // External Contact: Only test chassis/outer bounds if cube is outside scoop
        const cubeOBB = cube.getOBB();
        const robotBoxes = robot.getModuleBoxes();
        for (const rBox of robotBoxes) {
          const col = this.testOBBCollision(cubeOBB, rBox);
          if (col && col.intersects) {
            const pushDist = Math.min(0.035, (col.depth || 0) * 1.05);
            if (col.normal && Number.isFinite(col.normal.x) && Number.isFinite(col.normal.y)) {
              cube.x += col.normal.x * pushDist;
              cube.y += col.normal.y * pushDist;
            }
            if (robot.vx > 0.01) {
              cube.vx = robot.globalVx * 0.90;
              cube.vy = robot.globalVy * 0.90;
            }
            robot.hasScoopContact = true;
            break;
          }
        }
      }
    }
  }

  /**
   * Separating Axis Theorem (SAT) between two OBBs
   */
  testOBBCollision(boxA, boxB) {
    if (typeof RobotGeometry !== "undefined") {
      return RobotGeometry.testOBBCollision(boxA, boxB);
    }

    const axes = [...boxA.axes, ...boxB.axes];
    let minOverlap = Infinity;
    let bestAxis = { x: 0, y: 0 };

    for (const axis of axes) {
      const len = Math.hypot(axis.x, axis.y);
      if (len === 0) continue;
      const nx = axis.x / len;
      const ny = axis.y / len;

      let minA = Infinity, maxA = -Infinity;
      for (const p of boxA.corners) {
        const proj = p.x * nx + p.y * ny;
        if (proj < minA) minA = proj;
        if (proj > maxA) maxA = proj;
      }

      let minB = Infinity, maxB = -Infinity;
      for (const p of boxB.corners) {
        const proj = p.x * nx + p.y * ny;
        if (proj < minB) minB = proj;
        if (proj > maxB) maxB = proj;
      }

      const overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
      if (overlap <= 0) return { intersects: false };

      if (overlap < minOverlap) {
        minOverlap = overlap;
        bestAxis = { x: nx, y: ny };
        const dirX = boxA.cx - boxB.cx;
        const dirY = boxA.cy - boxB.cy;
        if (dirX * bestAxis.x + dirY * bestAxis.y < 0) {
          bestAxis.x = -bestAxis.x;
          bestAxis.y = -bestAxis.y;
        }
      }
    }

    return { intersects: true, depth: minOverlap, normal: bestAxis };
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = PhysicsEngine;
}
