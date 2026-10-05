/**
 * TUGAS PAPAN TULIS: SENSE - THINK - ACT
 * =====================================
 * Rules:
 *   1. coordinate must be (0,0)
 *   2. Holonomic / omni directional wheel
 *   3. 2D Lidar = eyes
 *      IMU = ears
 *   4. Interval 5 sec / point
 *
 * Target Coordinates:
 *   (0, 2) -> (2, 3) -> (-2, 4)
 */

// ==========================================
// 1. INITIALIZE
// ==========================================
if (!memory.initialized) {
  memory.initialized = true;

  // - Starting position: set titik awal sebagai origin (0, 0)
  memory.startX = sensors.pose.x;
  memory.startY = sensors.pose.y;

  // - Init data stream Lidar and IMU: sudah aktif melalui sensors.lidar dan sensors.imu

  // - Load target coordinate from instruction
  memory.targetCoordinates = [
    { x: 0.0, y: 2.0 },
    { x: 2.0, y: 3.0 },
    { x: -2.0, y: 4.0 },
  ];

  memory.currentIndex = 0;
  memory.timer5s = 0.0;
  memory.isWaiting = false;

  robot.log("=== 1. INITIALIZE ===");
  robot.log("Starting position: Origin (0, 0)");
  robot.log("Data stream LiDAR & IMU: Active");
  robot.log("Target coordinates: (0, 2), (2, 3), (-2, 4)");
}

// ==========================================
// 2. SENSE
// ==========================================
// - read IMU for robot heading
const robotHeading = sensors.imu.headingRad;

// - read LIDAR for position (posisi koordinat relatif terhadap origin (0,0))
const currentX = sensors.pose.x - memory.startX;
const currentY = sensors.pose.y - memory.startY;
const frontDist = sensors.lidar.getFront(35);
const leftDist = sensors.lidar.getLeft(45);
const rightDist = sensors.lidar.getRight(45);

// ==========================================
// 3. THINK
// ==========================================
// Cek jika seluruh koordinat telah tercapai
if (memory.currentIndex >= memory.targetCoordinates.length) {
  robot.setVelocity(0, 0, 0);
  robot.log("SELESAI: Semua titik koordinat telah tercapai!");
  return;
}

const currentTarget = memory.targetCoordinates[memory.currentIndex];
const deltaX = currentTarget.x - currentX;
const deltaY = currentTarget.y - currentY;
const distance = Math.hypot(deltaX, deltaY);

// - use IMU to recognize coordinate value (menghitung arah & sudut bearing ke target)
const targetHeading = Math.atan2(deltaY, deltaX);
let headingError = targetHeading - robotHeading;
while (headingError > Math.PI) headingError -= 2 * Math.PI;
while (headingError < -Math.PI) headingError += 2 * Math.PI;

// - Hitung jarak & cek apakah sudah sampai di titik (Rule 4: Interval 5 sec / point)
if (distance < 0.28) {
  memory.isWaiting = true;
}

if (memory.isWaiting) {
  memory.timer5s += dt;
  robot.log(`Point (${currentTarget.x}, ${currentTarget.y}) tercapai. Interval tunggu: ${memory.timer5s.toFixed(1)}s / 5s`);

  if (memory.timer5s >= 5.0) {
    robot.log(`>>> Interval 5 detik selesai di (${currentTarget.x}, ${currentTarget.y}). Lanjut ke titik berikutnya! <<<`);
    memory.currentIndex++;
    memory.timer5s = 0.0;
    memory.isWaiting = false;
  }
}

// ==========================================
// 4. ACT
// ==========================================
if (memory.isWaiting) {
  // - Diam selama interval 5 detik pada titik target
  robot.setVelocity(0, 0, 0);
} else {
  // - Deteksi lorong sempit dan morphing bentuk "I"
  if (leftDist < 0.36 && rightDist < 0.36) {
    if (sensors.shape !== "I") robot.setShape("I");
  } else if (sensors.shape === "I" && leftDist > 0.60 && rightDist > 0.60) {
    robot.setShape("O");
  }

  // - Target vector in robot local frame
  let targetAngleRel = targetHeading - robotHeading;
  while (targetAngleRel > Math.PI) targetAngleRel -= 2 * Math.PI;
  while (targetAngleRel < -Math.PI) targetAngleRel += 2 * Math.PI;

  let vxLocal = Math.cos(targetAngleRel) * 0.32;
  let vyLocal = Math.sin(targetAngleRel) * 0.32;

  // - 2D LiDAR Obstacle Repulsion (Eyes) & Holonomic Crabbing
  if (frontDist < 0.70) {
    const urgency = (0.70 - frontDist) / 0.70;
    vxLocal -= urgency * 0.45;
    const dodgeDir = leftDist > rightDist ? 1 : -1;
    vyLocal += dodgeDir * urgency * 0.50;
  }
  if (leftDist < 0.45) {
    vyLocal -= ((0.45 - leftDist) / 0.45) * 0.35;
  }
  if (rightDist < 0.45) {
    vyLocal += ((0.45 - rightDist) / 0.45) * 0.35;
  }

  const spd = Math.hypot(vxLocal, vyLocal);
  if (spd > 0.40) {
    vxLocal = (vxLocal / spd) * 0.40;
    vyLocal = (vyLocal / spd) * 0.40;
  }

  const omega = Math.max(-1.5, Math.min(1.5, headingError * 1.5));
  robot.setVelocity(vxLocal, vyLocal, omega);
}
