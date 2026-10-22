/**
 * Three.js 3D Viewport & Visualizer
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop & 3 Cargo Cubes
 * Features 4-tier multi-stack chassis (Drive, Power, Mini PC, Planar LiDAR),
 * 4 spinning Mecanum wheels, detailed wire-mesh cargo scoop, 3 physical Cargo Cubes,
 * 360° LiDAR laser beams/point cloud, shadows, and multi-camera modes.
 */

class Viewport3D {
  constructor(containerId, arenaMap) {
    this.container = document.getElementById(containerId);
    this.map = arenaMap;

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;

    // Camera modes: "perspective" | "topdown" | "follow"
    this.cameraMode = "perspective";

    // Visual elements
    this.robotGroup = null;
    this.wheelMeshes = [];
    this.lidarScannerMesh = null;
    this.lidarLaserLines = null;
    this.lidarPointGeometry = null;
    this.lidarPointsMesh = null;
    this.obstacleMeshes = [];
    this.trajectoryLine = null;
    this.goalGroup = null;
    this.cargoMeshes = [];

    // Visualization toggles
    this.showLidarBeams = true;
    this.showTrajectory = true;
    this.showBoundingBoxes = false;

    this.init();
  }

  init() {
    if (!this.container) return;

    const width = this.container.clientWidth || 800;
    const height = this.container.clientHeight || 600;

    // 1. Scene setup
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x090d16);
    this.scene.fog = new THREE.FogExp2(0x090d16, 0.035);

    // 2. Camera setup
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.05, 50);
    this.camera.position.set(2.5, 4.8, 6.2);
    this.camera.lookAt(2.5, 0, 2.5);

    // 3. WebGL Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    // 4. OrbitControls
    if (typeof THREE.OrbitControls !== "undefined") {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.target.set(2.5, 0, 2.5);
      this.controls.maxPolarAngle = Math.PI / 2 - 0.05;
      this.controls.minDistance = 1.0;
      this.controls.maxDistance = 12.0;
    }

    // 5. Lighting
    this.setupLighting();

    // 6. Arena Environment
    this.buildArenaEnvironment();

    // 7. Single-Block Smorphi Robot & Front Scoop
    this.buildRobotModel();

    // 8. 3 Cargo Cubes
    this.buildCargoCubes();

    // 9. LiDAR Ray Visualization
    this.buildLidarVisualizer();

    // 10. Trajectory Trail
    this.buildTrajectoryVisualizer();

    // 11. Passive Goal Marker
    this.buildGoalMarker();

    // 12. Obstacles
    this.rebuildObstacleMeshes();

    // 13. Resize listener
    window.addEventListener("resize", () => this.onWindowResize());
  }

  setupLighting() {
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    this.scene.add(ambientLight);

    const sunLight = new THREE.DirectionalLight(0xffffff, 0.95);
    sunLight.position.set(4, 9, 3);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 0.5;
    sunLight.shadow.camera.far = 18;
    const d = 4.0;
    sunLight.shadow.camera.left = -d;
    sunLight.shadow.camera.right = d;
    sunLight.shadow.camera.top = d;
    sunLight.shadow.camera.bottom = -d;
    sunLight.shadow.bias = -0.0005;
    this.scene.add(sunLight);

    const fillLight = new THREE.DirectionalLight(0x06b6d4, 0.35);
    fillLight.position.set(-2, 4, -2);
    this.scene.add(fillLight);
  }

  buildArenaEnvironment() {
    const W = this.map.width;
    const H = this.map.height;

    // Floor Mesh
    const floorGeo = new THREE.PlaneGeometry(W, H);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.85,
      metalness: 0.2,
    });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.set(W / 2, 0, H / 2);
    floor.receiveShadow = true;
    this.scene.add(floor);

    // Metric Grid (every 0.5m)
    const gridHelper = new THREE.GridHelper(W, 10, 0x38bdf8, 0x1e293b);
    gridHelper.position.set(W / 2, 0.002, H / 2);
    this.scene.add(gridHelper);

    // Perimeter Border Walls (Competition White Modular Panels)
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0xf8fafc,
      roughness: 0.35,
      metalness: 0.15,
    });
    const wallThick = CONFIG.ARENA.WALL_THICKNESS;
    const wallH = CONFIG.ARENA.WALL_HEIGHT;

    const wallsData = [
      { w: W + wallThick * 2, h: wallH, d: wallThick, x: W / 2, y: wallH / 2, z: -wallThick / 2 },
      { w: W + wallThick * 2, h: wallH, d: wallThick, x: W / 2, y: wallH / 2, z: H + wallThick / 2 },
      { w: wallThick, h: wallH, d: H, x: -wallThick / 2, y: wallH / 2, z: H / 2 },
      { w: wallThick, h: wallH, d: H, x: W + wallThick / 2, y: wallH / 2, z: H / 2 },
    ];

    for (const w of wallsData) {
      const geo = new THREE.BoxGeometry(w.w, w.h, w.d);
      const mesh = new THREE.Mesh(geo, wallMat);
      mesh.position.set(w.x, w.y, w.z);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);

      // Warning neon stripe on top of perimeter
      const stripeGeo = new THREE.BoxGeometry(w.w, 0.02, w.d);
      const stripeMat = new THREE.MeshBasicMaterial({ color: 0x0284c7 });
      const stripe = new THREE.Mesh(stripeGeo, stripeMat);
      stripe.position.set(w.x, wallH + 0.01, w.z);
      this.scene.add(stripe);
    }
  }

  /**
   * Build 3D Single-Block Smorphi Base Unit with 4 Stacked White Laser-Cut Acrylic Decks,
   * Blue TT Motors, FPV Camera, Battery Pack, and Competition Yellow Front Scoop with Dual Blue IR Sensors.
   */
  buildRobotModel() {
    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);

    const W = CONFIG.ROBOT.WIDTH;  // 0.170m
    const L = CONFIG.ROBOT.LENGTH; // 0.170m

    // Materials for physical Smorphi construction (matching Tutorial Videos Part 1, 4, 5, 6)
    const whiteAcrylicMat = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      roughness: 0.18,
      metalness: 0.08,
      transparent: true,
      opacity: 0.95,
    });

    const blackBasePlateMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.45,
      metalness: 0.6,
    });

    const standoffMat = new THREE.MeshStandardMaterial({
      color: 0x111827,
      roughness: 0.3,
      metalness: 0.85,
    });

    const ttMotorMat = new THREE.MeshStandardMaterial({
      color: 0x0284c7, // Distinctive blue TT motor gearbox housing
      roughness: 0.4,
      metalness: 0.3,
    });

    const shaftYellowMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      roughness: 0.3,
      metalness: 0.2,
    });

    // 1. Four Laser-Cut White Acrylic Decks
    const deckHeights = [0.038, 0.098, 0.168, 0.238];
    const deckSizes = [
      { l: L * 0.98, w: W * 0.98 },
      { l: L * 0.94, w: W * 0.94 },
      { l: L * 0.92, w: W * 0.92 },
      { l: L * 0.92, w: W * 0.92 },
    ];

    for (let d = 0; d < 4; d++) {
      const dGeo = new THREE.BoxGeometry(deckSizes[d].l, 0.005, deckSizes[d].w);
      const dMesh = new THREE.Mesh(dGeo, d === 0 ? blackBasePlateMat : whiteAcrylicMat);
      dMesh.position.y = deckHeights[d];
      dMesh.castShadow = true;
      dMesh.receiveShadow = true;
      this.robotGroup.add(dMesh);

      // White acrylic edge bevel glow for realistic laser-cut acrylic sheen
      if (d > 0) {
        const edgeGeo = new THREE.BoxGeometry(deckSizes[d].l + 0.001, 0.002, deckSizes[d].w + 0.001);
        const edgeMat = new THREE.MeshBasicMaterial({ color: 0xe2e8f0 });
        const edgeMesh = new THREE.Mesh(edgeGeo, edgeMat);
        edgeMesh.position.y = deckHeights[d];
        this.robotGroup.add(edgeMesh);
      }
    }

    // 2. Black Aluminum Standoff Pillars at the 4 Corners
    const standoffCorners = [
      { x: 0.066, z: 0.066 },
      { x: 0.066, z: -0.066 },
      { x: -0.066, z: 0.066 },
      { x: -0.066, z: -0.066 },
    ];
    const totalStandoffH = deckHeights[3] - deckHeights[0];
    const standoffGeo = new THREE.CylinderGeometry(0.004, 0.004, totalStandoffH, 12);

    for (const sc of standoffCorners) {
      const standoff = new THREE.Mesh(standoffGeo, standoffMat);
      standoff.position.set(sc.x, deckHeights[0] + totalStandoffH / 2, sc.z);
      standoff.castShadow = true;
      this.robotGroup.add(standoff);

      // Brass/silver screw tops on upper deck
      const screwGeo = new THREE.CylinderGeometry(0.003, 0.003, 0.003, 8);
      const screwMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.9, roughness: 0.2 });
      const screw = new THREE.Mesh(screwGeo, screwMat);
      screw.position.set(sc.x, deckHeights[3] + 0.003, sc.z);
      this.robotGroup.add(screw);
    }

    // 3. Four Blue TT Gear Motors with yellow output shafts
    const motorOffsets = [
      { x: 0.045, z: 0.045 },
      { x: 0.045, z: -0.045 },
      { x: -0.045, z: 0.045 },
      { x: -0.045, z: -0.045 },
    ];
    for (const mo of motorOffsets) {
      const mGeo = new THREE.BoxGeometry(0.042, 0.022, 0.022);
      const mMesh = new THREE.Mesh(mGeo, ttMotorMat);
      mMesh.position.set(mo.x, 0.024, mo.z);
      mMesh.castShadow = true;
      this.robotGroup.add(mMesh);

      // Yellow output axle boss
      const bossGeo = new THREE.CylinderGeometry(0.006, 0.006, 0.008, 10);
      const boss = new THREE.Mesh(bossGeo, shaftYellowMat);
      boss.rotation.x = Math.PI / 2;
      boss.position.set(mo.x, 0.024, mo.z + (mo.z > 0 ? 0.013 : -0.013));
      this.robotGroup.add(boss);
    }

    // 4. Mecanum Wheels at chassis corners
    const wheelR = CONFIG.ROBOT.WHEEL_RADIUS; // 0.030m
    const wheelW = CONFIG.ROBOT.WHEEL_WIDTH;  // 0.025m
    const wheelOffsets = CONFIG.ROBOT.WHEEL_OFFSETS; // [FL, FR, RL, RR]

    this.wheelMeshes = [];

    for (let w = 0; w < 4; w++) {
      const off = wheelOffsets[w];
      const wheelGroup = new THREE.Group();

      const wheelGeo = new THREE.CylinderGeometry(wheelR, wheelR, wheelW, 16);
      const wheelMat = new THREE.MeshStandardMaterial({
        color: 0x1e293b,
        roughness: 0.4,
        metalness: 0.8,
      });
      const wheelMesh = new THREE.Mesh(wheelGeo, wheelMat);
      wheelMesh.rotation.x = Math.PI / 2; // Orient along lateral axis
      wheelMesh.castShadow = true;
      wheelGroup.add(wheelMesh);

      // Angled Mecanum Rollers accent ring
      const rollerRingGeo = new THREE.TorusGeometry(wheelR * 0.82, 0.0035, 6, 16);
      const rollerMat = new THREE.MeshBasicMaterial({ color: 0x94a3b8 });
      const rollerRing = new THREE.Mesh(rollerRingGeo, rollerMat);
      wheelGroup.add(rollerRing);

      // In Three.js: Robot +X is local X, Robot +Y (Left) is local -Z!
      wheelGroup.position.set(off.lx, wheelR, -off.ly);
      this.robotGroup.add(wheelGroup);
      this.wheelMeshes.push(wheelMesh);
    }

    // 5. Tier 2: Motor Driver Deck (y = 0.098 to 0.168)
    const driverBoardGeo = new THREE.BoxGeometry(0.065, 0.006, 0.075);
    const driverBoardMat = new THREE.MeshStandardMaterial({ color: 0x991b1b, roughness: 0.4 }); // Red PCB
    const driverBoard = new THREE.Mesh(driverBoardGeo, driverBoardMat);
    driverBoard.position.set(0, 0.103, 0);
    this.robotGroup.add(driverBoard);

    // Motor driver aluminum heatsink
    const heatsinkGeo = new THREE.BoxGeometry(0.025, 0.025, 0.025);
    const heatsinkMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.9, roughness: 0.2 });
    const heatsink = new THREE.Mesh(heatsinkGeo, heatsinkMat);
    heatsink.position.set(0, 0.118, 0);
    this.robotGroup.add(heatsink);

    // 6. Tier 3: Main Controller / ESP32 Deck (y = 0.168 to 0.238)
    const espBoardGeo = new THREE.BoxGeometry(0.080, 0.006, 0.080);
    const espBoardMat = new THREE.MeshStandardMaterial({ color: 0x1e3a8a, roughness: 0.35 }); // Blue PCB
    const espBoard = new THREE.Mesh(espBoardGeo, espBoardMat);
    espBoard.position.set(0, 0.173, 0);
    this.robotGroup.add(espBoard);

    // Status LED (Emerald Green)
    const ledGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.006, 12);
    const ledMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const led = new THREE.Mesh(ledGeo, ledMat);
    led.position.set(-0.045, 0.176, 0.035);
    this.robotGroup.add(led);

    // Comms Heartbeat LED (Cyan)
    const comLedMat = new THREE.MeshBasicMaterial({ color: 0x06b6d4 });
    const comLed = new THREE.Mesh(ledGeo, comLedMat);
    comLed.position.set(-0.045, 0.176, -0.035);
    this.robotGroup.add(comLed);

    // 7. Tier 4: Top Deck with FPV Camera, Battery Pack & 2D LiDAR Turret
    // FPV Camera Unit on Front Edge of Deck 4 (Tutorial Part 1 & 5)
    const camGroup = new THREE.Group();
    const camBodyGeo = new THREE.BoxGeometry(0.022, 0.022, 0.024);
    const camBodyMat = new THREE.MeshStandardMaterial({ color: 0x111827, roughness: 0.4 });
    const camBody = new THREE.Mesh(camBodyGeo, camBodyMat);
    camGroup.add(camBody);

    const camLensGeo = new THREE.CylinderGeometry(0.007, 0.007, 0.008, 16);
    const camLensMat = new THREE.MeshStandardMaterial({ color: 0x0284c7, metalness: 0.9, roughness: 0.1 });
    const camLens = new THREE.Mesh(camLensGeo, camLensMat);
    camLens.rotation.z = Math.PI / 2;
    camLens.position.set(0.012, 0, 0);
    camGroup.add(camLens);

    camGroup.position.set(0.072, 0.252, 0);
    this.robotGroup.add(camGroup);

    // LiPo Battery Pack with yellow tape strap
    const batGeo = new THREE.BoxGeometry(0.055, 0.022, 0.075);
    const batMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.6 });
    const battery = new THREE.Mesh(batGeo, batMat);
    battery.position.set(-0.035, 0.252, 0);
    this.robotGroup.add(battery);

    const strapGeo = new THREE.BoxGeometry(0.012, 0.024, 0.077);
    const strapMat = new THREE.MeshBasicMaterial({ color: 0xfacc15 }); // Yellow safety tape strap
    const strap = new THREE.Mesh(strapGeo, strapMat);
    strap.position.set(-0.035, 0.252, 0);
    this.robotGroup.add(strap);

    // 2D Planar LiDAR Turret
    const lidarPedestalGeo = new THREE.CylinderGeometry(0.035, 0.042, 0.024, 16);
    const lidarPedestalMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8, roughness: 0.3 });
    const lidarPedestal = new THREE.Mesh(lidarPedestalGeo, lidarPedestalMat);
    lidarPedestal.position.set(0.015, 0.252, 0);
    this.robotGroup.add(lidarPedestal);

    const lidarHeadGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.024, 16);
    const lidarHeadMat = new THREE.MeshStandardMaterial({ color: 0x090d16, metalness: 0.9, roughness: 0.2 });
    this.lidarScannerMesh = new THREE.Mesh(lidarHeadGeo, lidarHeadMat);
    this.lidarScannerMesh.position.set(0.015, 0.276, 0);

    // Laser diode indicator
    const diodeGeo = new THREE.BoxGeometry(0.012, 0.006, 0.006);
    const diodeMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const diode = new THREE.Mesh(diodeGeo, diodeMat);
    diode.position.set(0.03, 0, 0);
    this.lidarScannerMesh.add(diode);

    this.robotGroup.add(this.lidarScannerMesh);

    // ==========================================
    // FRONT CARGO YELLOW SCOOP WITH DUAL IR SENSORS
    // ==========================================
    this.buildCargoScoop();
  }

  /**
   * Build the Front Competition Yellow 3D-Printed U-Frame Pusher Bumper with Dual Blue IR Sensors
   * Strictly Ground-Pushing Only (No bottom plate / 0% lifting - per RoboRoarZ rules)
   */
  buildCargoScoop() {
    const bumperGroup = new THREE.Group();

    const mountX = CONFIG.SCOOP ? CONFIG.SCOOP.MOUNT_X : 0.085;
    const scoopLen = CONFIG.SCOOP ? CONFIG.SCOOP.LENGTH : 0.140;
    const outerW = CONFIG.SCOOP ? CONFIG.SCOOP.OUTER_WIDTH : 0.190;
    const scoopH = CONFIG.SCOOP ? CONFIG.SCOOP.HEIGHT : 0.080;
    const wallThick = CONFIG.SCOOP ? CONFIG.SCOOP.WALL_THICKNESS : 0.014;

    const centerX = mountX + scoopLen / 2;
    const halfOuterW = outerW / 2;
    const groundClearance = 0.008; // 8mm clearance above floor: strictly NO bottom floor / NO lifting

    // Competition Bright Yellow 3D-Printed PLA Material (from Videos Part 4, 5, 6)
    const bumperYellowMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      roughness: 0.38,
      metalness: 0.12,
    });

    const bumperAccentMat = new THREE.MeshStandardMaterial({
      color: 0xeab308,
      roughness: 0.3,
      metalness: 0.2,
    });

    // 1. Left Pusher Arm Prong (In Three.js Left is -Z) - Completely open bottom
    const sideArmGeo = new THREE.BoxGeometry(scoopLen, scoopH, wallThick);
    const leftArm = new THREE.Mesh(sideArmGeo, bumperYellowMat);
    leftArm.position.set(centerX, groundClearance + scoopH / 2, -(halfOuterW - wallThick / 2));
    leftArm.castShadow = true;
    bumperGroup.add(leftArm);

    const leftTopRim = new THREE.Mesh(new THREE.BoxGeometry(scoopLen, 0.005, wallThick + 0.002), bumperAccentMat);
    leftTopRim.position.set(centerX, groundClearance + scoopH, -(halfOuterW - wallThick / 2));
    bumperGroup.add(leftTopRim);

    // 2. Right Pusher Arm Prong (+Z) - Completely open bottom
    const rightArm = new THREE.Mesh(sideArmGeo, bumperYellowMat);
    rightArm.position.set(centerX, groundClearance + scoopH / 2, (halfOuterW - wallThick / 2));
    rightArm.castShadow = true;
    bumperGroup.add(rightArm);

    const rightTopRim = new THREE.Mesh(new THREE.BoxGeometry(scoopLen, 0.005, wallThick + 0.002), bumperAccentMat);
    rightTopRim.position.set(centerX, groundClearance + scoopH, (halfOuterW - wallThick / 2));
    bumperGroup.add(rightTopRim);

    // 3. Back Pushing Plate against robot front chassis (Pushes cube forward on ground)
    const backPlateGeo = new THREE.BoxGeometry(wallThick, scoopH, outerW);
    const backPlate = new THREE.Mesh(backPlateGeo, bumperYellowMat);
    backPlate.position.set(mountX + wallThick / 2, groundClearance + scoopH / 2, 0);
    backPlate.castShadow = true;
    bumperGroup.add(backPlate);

    // Rubberized contact push pad on inner face of backplate
    const pushPadGeo = new THREE.BoxGeometry(0.005, scoopH * 0.75, outerW * 0.85);
    const pushPadMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, roughness: 0.9 });
    const pushPad = new THREE.Mesh(pushPadGeo, pushPadMat);
    pushPad.position.set(mountX + wallThick + 0.0025, groundClearance + scoopH / 2, 0);
    bumperGroup.add(pushPad);

    // Tactile contact sensor plate for contact feedback
    const sensorPlateGeo = new THREE.BoxGeometry(0.003, 0.020, 0.06);
    const sensorPlateMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const sensorPlate = new THREE.Mesh(sensorPlateGeo, sensorPlateMat);
    sensorPlate.position.set(mountX + wallThick + 0.005, groundClearance + scoopH / 2, 0);
    bumperGroup.add(sensorPlate);

    // 4. Angled Front Funnel Wings (Guides cube into center while pushing on ground)
    const wingLen = 0.025;
    const wingGeo = new THREE.BoxGeometry(wingLen, scoopH, wallThick);
    const leftWing = new THREE.Mesh(wingGeo, bumperYellowMat);
    leftWing.position.set(mountX + scoopLen + wingLen * 0.35, groundClearance + scoopH / 2, -(halfOuterW + 0.008));
    leftWing.rotation.y = -Math.PI / 6;
    bumperGroup.add(leftWing);

    const rightWing = new THREE.Mesh(wingGeo, bumperYellowMat);
    rightWing.position.set(mountX + scoopLen + wingLen * 0.35, groundClearance + scoopH / 2, +(halfOuterW + 0.008));
    rightWing.rotation.y = Math.PI / 6;
    bumperGroup.add(rightWing);

    // 5. Dual Blue IR Sensor Modules Mounted on Prong Tips (Tutorial Part 4 & 5)
    const irPcbMat = new THREE.MeshStandardMaterial({ color: 0x1d4ed8, roughness: 0.3 }); // Distinctive blue PCB
    const irLedMat = new THREE.MeshBasicMaterial({ color: 0x22c55e }); // Glowing green status LED
    const irOpticMat = new THREE.MeshStandardMaterial({ color: 0x090d16, roughness: 0.2 }); // Black optical detector
    const irTransMat = new THREE.MeshStandardMaterial({ color: 0x93c5fd, transparent: true, opacity: 0.8 }); // Clear transmitter

    const irProngOffsets = [
      { z: -(halfOuterW + 0.003), label: "IR_LEFT" },  // module1
      { z: +(halfOuterW + 0.003), label: "IR_RIGHT" }, // module4
    ];

    for (const ir of irProngOffsets) {
      const irGroup = new THREE.Group();

      // Blue PCB board
      const pcbGeo = new THREE.BoxGeometry(0.028, 0.004, 0.014);
      const pcb = new THREE.Mesh(pcbGeo, irPcbMat);
      irGroup.add(pcb);

      // Transmitter tube (front-facing)
      const txGeo = new THREE.CylinderGeometry(0.003, 0.003, 0.008, 12);
      const tx = new THREE.Mesh(txGeo, irTransMat);
      tx.rotation.z = Math.PI / 2;
      tx.position.set(0.014, 0, -0.0035);
      irGroup.add(tx);

      // Receiver photodiode (front-facing)
      const rx = new THREE.Mesh(txGeo, irOpticMat);
      rx.rotation.z = Math.PI / 2;
      rx.position.set(0.014, 0, 0.0035);
      irGroup.add(rx);

      // Power / Signal green LED
      const ledG = new THREE.BoxGeometry(0.003, 0.002, 0.003);
      const ledM = new THREE.Mesh(ledG, irLedMat);
      ledM.position.set(-0.006, 0.003, 0);
      irGroup.add(ledM);

      // Potentiometer trimmer (blue cube with silver screw)
      const potGeo = new THREE.BoxGeometry(0.005, 0.004, 0.005);
      const potMat = new THREE.MeshStandardMaterial({ color: 0x2563eb, metalness: 0.5 });
      const pot = new THREE.Mesh(potGeo, potMat);
      pot.position.set(0.002, 0.004, 0);
      irGroup.add(pot);

      irGroup.position.set(mountX + scoopLen - 0.016, 0.020, ir.z);
      bumperGroup.add(irGroup);
    }

    this.robotGroup.add(bumperGroup);
  }

  /**
   * Build 3 Physical Cargo Cubes with industrial metallic bevel look and neon ID badges
   */
  buildCargoCubes() {
    this.cargoMeshes = [];
    const size = CONFIG.CARGO.SIZE; // 0.090m
    const colors = CONFIG.CARGO.COLORS; // [Amber, Cyan, Emerald]

    for (let i = 0; i < 3; i++) {
      const cubeGroup = new THREE.Group();

      // Main Beveled Cube Body
      const bodyGeo = new THREE.BoxGeometry(size * 0.94, size * 0.94, size * 0.94);
      const bodyMat = new THREE.MeshStandardMaterial({
        color: colors[i].hex,
        roughness: 0.35,
        metalness: 0.7,
      });
      const body = new THREE.Mesh(bodyGeo, bodyMat);
      body.castShadow = true;
      body.receiveShadow = true;
      cubeGroup.add(body);

      // Metallic corner reinforcement trims
      const trimGeo = new THREE.BoxGeometry(size, 0.008, 0.008);
      const trimMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.9, roughness: 0.2 });
      const topTrimF = new THREE.Mesh(trimGeo, trimMat);
      topTrimF.position.set(0, size * 0.46, size * 0.46);
      cubeGroup.add(topTrimF);

      const topTrimB = new THREE.Mesh(trimGeo, trimMat);
      topTrimB.position.set(0, size * 0.46, -size * 0.46);
      cubeGroup.add(topTrimB);

      // Top glowing badge with label
      const badgeGeo = new THREE.BoxGeometry(size * 0.5, 0.004, size * 0.5);
      const badgeMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
      const badge = new THREE.Mesh(badgeGeo, badgeMat);
      badge.position.y = size * 0.47 + 0.002;
      cubeGroup.add(badge);

      // Floating glowing target beacon indicator (inverted pyramid pointer)
      const beaconGeo = new THREE.ConeGeometry(0.022, 0.045, 4);
      const beaconMat = new THREE.MeshBasicMaterial({ color: 0xfacc15 });
      const beacon = new THREE.Mesh(beaconGeo, beaconMat);
      beacon.rotation.x = Math.PI;
      beacon.position.y = size + 0.035;
      beacon.visible = false;
      cubeGroup.add(beacon);
      cubeGroup.targetBeacon = beacon;

      cubeGroup.position.set(2.5, size / 2, 2.5);
      this.scene.add(cubeGroup);
      this.cargoMeshes.push(cubeGroup);
    }
  }

  buildLidarVisualizer() {
    const numBeams = CONFIG.LIDAR.NUM_BEAMS;

    // Point cloud
    this.lidarPointGeometry = new THREE.BufferGeometry();
    const positions = new Float32Array(numBeams * 3);
    const colors = new Float32Array(numBeams * 3);

    this.lidarPointGeometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    this.lidarPointGeometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));

    const pMat = new THREE.PointsMaterial({
      size: 0.06,
      vertexColors: true,
      transparent: true,
      opacity: 0.9,
    });
    this.lidarPointsMesh = new THREE.Points(this.lidarPointGeometry, pMat);
    this.scene.add(this.lidarPointsMesh);

    // Laser Lines Fan
    const lineGeo = new THREE.BufferGeometry();
    const linePositions = new Float32Array(numBeams * 2 * 3);
    lineGeo.setAttribute("position", new THREE.BufferAttribute(linePositions, 3));

    const lineMat = new THREE.LineBasicMaterial({
      color: 0x06b6d4,
      transparent: true,
      opacity: 0.18,
    });
    this.lidarLaserLines = new THREE.LineSegments(lineGeo, lineMat);
    this.scene.add(this.lidarLaserLines);
  }

  buildTrajectoryVisualizer() {
    const maxPoints = 300;
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(maxPoints * 3);
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const mat = new THREE.LineBasicMaterial({
      color: 0x22c55e, // Fluorescent lime-green trajectory trail (matches Wefaa simulator)
      linewidth: 2,
      transparent: true,
      opacity: 0.85,
    });

    this.trajectoryLine = new THREE.Line(geom, mat);
    this.scene.add(this.trajectoryLine);

    const pathGeom = new THREE.BufferGeometry();
    pathGeom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(2000 * 3), 3));
    pathGeom.setDrawRange(0, 0);
    const pathMat = new THREE.LineBasicMaterial({ color: 0xe879f9, transparent: true, opacity: 0.95 });
    this.plannedPathLine = new THREE.Line(pathGeom, pathMat);
    this.scene.add(this.plannedPathLine);
  }

  buildGoalMarker() {
    this.goalGroup = new THREE.Group();

    // Floating passive beacon
    const beaconGeo = new THREE.OctahedronGeometry(0.16, 0);
    const beaconMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      emissive: 0xeab308,
      emissiveIntensity: 0.6,
      metalness: 0.8,
      roughness: 0.2,
    });
    this.goalBeacon = new THREE.Mesh(beaconGeo, beaconMat);
    this.goalBeacon.position.y = 0.42;
    this.goalGroup.add(this.goalBeacon);

    // Vertical hologram light cylinder
    const cylGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.85, 16, 1, true);
    const cylMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      transparent: true,
      opacity: 0.2,
      side: THREE.DoubleSide,
    });
    const cyl = new THREE.Mesh(cylGeo, cylMat);
    cyl.position.y = 0.42;
    this.goalGroup.add(cyl);

    // Floor Delivery Pad (Yellow landing zone disc)
    const padGeo = new THREE.CircleGeometry(0.45, 32);
    const padMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      transparent: true,
      opacity: 0.16,
      side: THREE.DoubleSide,
    });
    const pad = new THREE.Mesh(padGeo, padMat);
    pad.rotation.x = -Math.PI / 2;
    pad.position.y = 0.003;
    this.goalGroup.add(pad);

    // Outer delivery boundary ring on floor
    const outerRingGeo = new THREE.RingGeometry(0.44, 0.48, 32);
    const outerRingMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      side: THREE.DoubleSide,
    });
    const outerRing = new THREE.Mesh(outerRingGeo, outerRingMat);
    outerRing.rotation.x = -Math.PI / 2;
    outerRing.position.y = 0.005;
    this.goalGroup.add(outerRing);

    // Inner concentric target ring
    const innerRingGeo = new THREE.RingGeometry(0.18, 0.22, 32);
    const innerRingMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      side: THREE.DoubleSide,
    });
    const innerRing = new THREE.Mesh(innerRingGeo, innerRingMat);
    innerRing.rotation.x = -Math.PI / 2;
    innerRing.position.y = 0.006;
    this.goalGroup.add(innerRing);

    this.scene.add(this.goalGroup);
  }

  rebuildObstacleMeshes() {
    for (const mesh of this.obstacleMeshes) {
      this.scene.remove(mesh);
    }
    this.obstacleMeshes = [];

    const crateMat = new THREE.MeshStandardMaterial({
      color: 0xf1f5f9, // Competition modular white partition panel (Tutorial Part 1)
      roughness: 0.35,
      metalness: 0.15,
    });

    const corridorMat = new THREE.MeshStandardMaterial({
      color: 0xe2e8f0, // Modular white corridor partition panel
      roughness: 0.35,
      metalness: 0.15,
    });

    const obsH = 0.35; // 35 cm obstacle height

    for (const obs of this.map.obstacles) {
      let mesh;
      const isCorridor = obs.type === "corridor";

      if (obs.type === "pillar") {
        const radius = Math.min(obs.w, obs.h) / 2;
        const geo = new THREE.CylinderGeometry(radius, radius, obsH, 16);
        mesh = new THREE.Mesh(geo, crateMat);
        mesh.position.set(obs.x + obs.w / 2, obsH / 2, obs.y + obs.h / 2);
      } else {
        const geo = new THREE.BoxGeometry(obs.w, obsH, obs.h);
        mesh = new THREE.Mesh(geo, isCorridor ? corridorMat : crateMat);
        mesh.position.set(obs.x + obs.w / 2, obsH / 2, obs.y + obs.h / 2);

        if (isCorridor) {
          const edgeGeo = new THREE.BoxGeometry(obs.w, 0.02, obs.h);
          const edgeMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b });
          const edge = new THREE.Mesh(edgeGeo, edgeMat);
          edge.position.y = obsH / 2 + 0.01;
          mesh.add(edge);
        }
      }

      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);
      this.obstacleMeshes.push(mesh);
    }
  }

  /**
   * Update 3D elements on each frame
   * @param {SmorphiRobot} robot
   * @param {SensorSuite} sensorSuite
   * @param {number} dt
   * @param {CargoManager} [cargoManager]
   * @param {boolean} [isGoalReached]
   */
  update(robot, sensorSuite, dt, cargoManager = null, isGoalReached = false) {
    // 1. Update Robot Position & Heading
    this.robotGroup.position.set(robot.x, 0, robot.y);
    this.robotGroup.rotation.y = -robot.theta;

    // 2. Spin 4 Mecanum Wheels
    for (let w = 0; w < 4; w++) {
      if (this.wheelMeshes[w]) {
        this.wheelMeshes[w].rotation.x = robot.wheelAngles[w];
      }
    }

    // 3. Spin LiDAR Turret (10 Hz)
    if (this.lidarScannerMesh) {
      this.lidarScannerMesh.rotation.y += Math.PI * 2 * 10 * dt;
    }

    // 4. Update 3 Cargo Cubes
    if (cargoManager && cargoManager.cubes) {
      const activeTargetId = (sensorSuite && sensorSuite.cargo && sensorSuite.cargo.activeTarget) ? sensorSuite.cargo.activeTarget.id : null;
      for (let i = 0; i < 3; i++) {
        const cube = cargoManager.cubes[i];
        if (this.cargoMeshes[i]) {
          const safeX = Number.isFinite(cube.x) ? Math.max(0.05, Math.min(4.95, cube.x)) : 2.5;
          const safeY = Number.isFinite(cube.y) ? Math.max(0.05, Math.min(4.95, cube.y)) : 2.5;
          const safeTheta = Number.isFinite(cube.theta) ? cube.theta : 0;
          this.cargoMeshes[i].position.set(safeX, (cube.size || 0.09) / 2, safeY);
          this.cargoMeshes[i].rotation.y = -safeTheta;
          this.cargoMeshes[i].visible = true;

          if (this.cargoMeshes[i].targetBeacon) {
            const isTarget = (cube.id === activeTargetId) && (cube.state !== "DELIVERED_AT_GOAL");
            this.cargoMeshes[i].targetBeacon.visible = isTarget;
            if (isTarget) {
              this.cargoMeshes[i].targetBeacon.rotation.y += 3.5 * dt;
              this.cargoMeshes[i].targetBeacon.position.y = (cube.size || 0.09) + 0.035 + Math.sin(Date.now() * 0.008) * 0.012;
            }
          }
        }
      }
    }

    // 5. Update LiDAR Rays & Points
    this.updateLidarVisualization(sensorSuite.lidarPoints, robot);

    // 6. Update Trajectory Trail
    this.updateTrajectoryVisualization(robot.trajectory);

    // 7. Update Goal Beacon (Tanda Kuning / Target Finish Point)
    if (this.goalGroup && this.map && this.map.goal) {
      this.goalGroup.position.set(this.map.goal.x, 0, this.map.goal.y);
      if (isGoalReached) {
        this.goalBeacon.rotation.y += 4.5 * dt;
        this.goalBeacon.rotation.x += 2.0 * dt;
        this.goalBeacon.position.y = 0.42 + Math.sin(Date.now() * 0.008) * 0.08;
        if (this.goalBeacon.material) {
          this.goalBeacon.material.color.setHex(0x10b981);
          this.goalBeacon.material.emissive.setHex(0x059669);
          this.goalBeacon.material.emissiveIntensity = 0.9 + Math.sin(Date.now() * 0.01) * 0.4;
        }
      } else {
        this.goalBeacon.rotation.y += 1.8 * dt;
        this.goalBeacon.rotation.x += 0.8 * dt;
        this.goalBeacon.position.y = 0.42;
        if (this.goalBeacon.material) {
          this.goalBeacon.material.color.setHex(0xfacc15);
          this.goalBeacon.material.emissive.setHex(0xeab308);
          this.goalBeacon.material.emissiveIntensity = 0.6;
        }
      }
    }

    if (this.plannedPathLine) {
      const pts = robot.debugPath || [];
      const n = Math.min(pts.length, 2000);
      const attr = this.plannedPathLine.geometry.attributes.position;
      for (let i = 0; i < n; i++) attr.setXYZ(i, pts[i].x, 0.03, pts[i].y);
      this.plannedPathLine.geometry.setDrawRange(0, n);
      attr.needsUpdate = true;
    }

    // 8. Camera
    this.updateCamera(robot);

    // 9. Render WebGL Scene
    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }

  updateLidarVisualization(points, robot) {
    if (!this.lidarPointsMesh || !points || points.length === 0) return;

    this.lidarPointsMesh.visible = this.showLidarBeams;
    this.lidarLaserLines.visible = this.showLidarBeams;

    if (!this.showLidarBeams) return;

    const posAttr = this.lidarPointGeometry.attributes.position;
    const colAttr = this.lidarPointGeometry.attributes.color;
    const linePosAttr = this.lidarLaserLines.geometry.attributes.position;

    const sensorY = CONFIG.LIDAR.ELEVATION_Y || 0.285;

    for (let i = 0; i < points.length; i++) {
      const pt = points[i];
      const hitX = pt.x;
      const hitY = sensorY;
      const hitZ = pt.y;

      posAttr.setXYZ(i, hitX, hitY, hitZ);

      // Color coding: Red < 0.6m, Amber < 1.2m, Cyan >= 1.2m
      if (pt.distance < 0.6) {
        colAttr.setXYZ(i, 0.94, 0.27, 0.27);
      } else if (pt.distance < 1.2) {
        colAttr.setXYZ(i, 0.96, 0.62, 0.07);
      } else {
        colAttr.setXYZ(i, 0.02, 0.71, 0.83);
      }

      // Laser lines from sensor center to hit point
      const lineIdx = i * 2;
      linePosAttr.setXYZ(lineIdx, robot.x, sensorY, robot.y);
      linePosAttr.setXYZ(lineIdx + 1, hitX, hitY, hitZ);
    }

    this.lidarPointGeometry.attributes.position.needsUpdate = true;
    this.lidarPointGeometry.attributes.color.needsUpdate = true;
    this.lidarLaserLines.geometry.attributes.position.needsUpdate = true;
  }

  updateTrajectoryVisualization(trajectory) {
    if (!this.trajectoryLine) return;
    this.trajectoryLine.visible = this.showTrajectory;
    if (!this.showTrajectory || !trajectory || trajectory.length === 0) {
      this.trajectoryLine.geometry.setDrawRange(0, 0);
      return;
    }

    const positions = this.trajectoryLine.geometry.attributes.position;
    const count = Math.min(trajectory.length, 300);

    for (let i = 0; i < count; i++) {
      positions.setXYZ(i, trajectory[i].x, 0.008, trajectory[i].y);
    }

    this.trajectoryLine.geometry.setDrawRange(0, count);
    positions.needsUpdate = true;
  }

  updateCamera(robot) {
    if (this.cameraMode === "topdown") {
      this.camera.position.set(this.map.width / 2, 7.5, this.map.height / 2);
      this.camera.lookAt(this.map.width / 2, 0, this.map.height / 2);
      if (this.controls) this.controls.enabled = false;
    } else if (this.cameraMode === "follow") {
      if (this.controls) this.controls.enabled = false;
      const dist = 1.6;
      const height = 1.2;
      const cosT = Math.cos(robot.theta);
      const sinT = Math.sin(robot.theta);

      const targetCamX = robot.x - cosT * dist;
      const targetCamZ = robot.y - sinT * dist;

      this.camera.position.x += (targetCamX - this.camera.position.x) * 0.12;
      this.camera.position.y += (height - this.camera.position.y) * 0.12;
      this.camera.position.z += (targetCamZ - this.camera.position.z) * 0.12;

      this.camera.lookAt(robot.x + cosT * 0.5, 0.15, robot.y + sinT * 0.5);
    } else {
      if (this.controls) {
        this.controls.enabled = true;
        this.controls.update();
      }
    }
  }

  setCameraMode(mode) {
    this.cameraMode = mode;
    if (mode === "perspective" && this.controls) {
      this.controls.enabled = true;
      this.camera.position.set(2.5, 4.8, 6.2);
      this.controls.target.set(2.5, 0, 2.5);
    }
  }

  onWindowResize() {
    if (!this.container || !this.renderer || !this.camera) return;
    const width = this.container.clientWidth || 800;
    const height = this.container.clientHeight || 600;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = Viewport3D;
}
