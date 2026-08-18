(function () {
  'use strict';

  const ROSBRIDGE_PORT = 9090;
  const TELEOP_RATE_HZ = 10;
  const LINEAR_SPEED = 0.4;
  const ANGULAR_SPEED = 1.0;


  const statusEl = document.getElementById('connection-status');
  const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:${ROSBRIDGE_PORT}` });

  ros.on('connection', () => {
    statusEl.textContent = 'connected';
    statusEl.className = 'status status--connected';
  });
  ros.on('close', () => {
    statusEl.textContent = 'disconnected';
    statusEl.className = 'status status--disconnected';
  });
  ros.on('error', () => {
    statusEl.textContent = 'error';
    statusEl.className = 'status status--disconnected';
  });

  const odomTopic = new ROSLIB.Topic({
    ros, name: '/odom', messageType: 'nav_msgs/msg/Odometry',
  });
  odomTopic.subscribe((msg) => updateRobotPose(msg.pose.pose));

  const cameraTopic = new ROSLIB.Topic({
    ros, name: '/detection/image_raw/compressed', messageType: 'sensor_msgs/msg/CompressedImage',
  });
  const cameraFeed = document.getElementById('camera-feed');
  cameraTopic.subscribe((msg) => {
    cameraFeed.src = `data:image/jpeg;base64,${msg.data}`;
  });

  const positionTopic = new ROSLIB.Topic({
    ros, name: '/person/position', messageType: 'geometry_msgs/msg/PointStamped',
  });
  const trackingStateEl = document.getElementById('tracking-state');
  const distanceEl = document.getElementById('distance-value');
  const bearingEl = document.getElementById('bearing-value');
  let lastPositionAt = 0;
  positionTopic.subscribe((msg) => {
    lastPositionAt = Date.now();
    const { x, y } = msg.point;
    const distance = Math.hypot(x, y);
    const bearingDeg = (Math.atan2(y, x) * 180) / Math.PI;
    trackingStateEl.textContent = 'tracking';
    distanceEl.textContent = `${distance.toFixed(2)} m`;
    bearingEl.textContent = `${bearingDeg.toFixed(1)} deg`;
  });
  setInterval(() => {
    if (Date.now() - lastPositionAt > 1500) {
      trackingStateEl.textContent = 'searching...';
    }
  }, 500);

  const overrideTopic = new ROSLIB.Topic({
    ros, name: '/manual_override', messageType: 'std_msgs/msg/Bool',
  });
  const teleopTopic = new ROSLIB.Topic({
    ros, name: '/cmd_vel_teleop', messageType: 'geometry_msgs/msg/Twist',
  });

  const overrideToggle = document.getElementById('override-toggle');
  overrideToggle.addEventListener('change', () => {
    overrideTopic.publish(new ROSLIB.Message({ data: overrideToggle.checked }));
  });


  let drive = { linear: 0, angular: 0 };

  function publishTeleop() {
    teleopTopic.publish(new ROSLIB.Message({
      linear: { x: drive.linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: drive.angular },
    }));
  }
  setInterval(() => {
    if (overrideToggle.checked) publishTeleop();
  }, 1000 / TELEOP_RATE_HZ);

  const driveActions = {
    forward: () => (drive = { linear: LINEAR_SPEED, angular: 0 }),
    back: () => (drive = { linear: -LINEAR_SPEED, angular: 0 }),
    left: () => (drive = { linear: 0, angular: ANGULAR_SPEED }),
    right: () => (drive = { linear: 0, angular: -ANGULAR_SPEED }),
    stop: () => (drive = { linear: 0, angular: 0 }),
  };

  const buttonMap = {
    'btn-forward': 'forward',
    'btn-back': 'back',
    'btn-left': 'left',
    'btn-right': 'right',
    'btn-stop': 'stop',
  };
  Object.entries(buttonMap).forEach(([id, action]) => {
    const el = document.getElementById(id);
    const start = () => driveActions[action]();
    const stop = () => driveActions.stop();
    el.addEventListener('mousedown', start);
    el.addEventListener('touchstart', (e) => { e.preventDefault(); start(); });
    ['mouseup', 'mouseleave', 'touchend'].forEach((ev) => el.addEventListener(ev, stop));
  });

  const keyMap = {
    KeyW: 'forward', ArrowUp: 'forward',
    KeyS: 'back', ArrowDown: 'back',
    KeyA: 'left', ArrowLeft: 'left',
    KeyD: 'right', ArrowRight: 'right',
  };
  const activeKeys = new Set();
  window.addEventListener('keydown', (e) => {
    if (!keyMap[e.code] || !overrideToggle.checked) return;
    activeKeys.add(e.code);
    driveActions[keyMap[e.code]]();
  });
  window.addEventListener('keyup', (e) => {
    if (!keyMap[e.code]) return;
    activeKeys.delete(e.code);
    if (activeKeys.size === 0) driveActions.stop();
  });


  const container = document.getElementById('scene-container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f1115);

  const camera = new THREE.PerspectiveCamera(
    50, container.clientWidth / container.clientHeight, 0.05, 100);
  camera.position.set(3, 3, 3);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  container.appendChild(renderer.domElement);

  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
  dirLight.position.set(4, 6, 4);
  scene.add(dirLight);

  const grid = new THREE.GridHelper(20, 40, 0x2a2f3a, 0x1c2029);
  scene.add(grid);

  const robotGroup = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.4, 0.15, 0.3),
    new THREE.MeshStandardMaterial({ color: 0x4fb3ff }),
  );
  body.position.y = 0.1;
  robotGroup.add(body);
  const heading = new THREE.Mesh(
    new THREE.ConeGeometry(0.08, 0.2, 12),
    new THREE.MeshStandardMaterial({ color: 0xff5d5d }),
  );
  heading.rotation.z = -Math.PI / 2;
  heading.position.set(0.3, 0.1, 0);
  robotGroup.add(heading);
  scene.add(robotGroup);

  function updateRobotPose(pose) {
    robotGroup.position.set(pose.position.x, 0, -pose.position.y);
    const q = pose.orientation;
    const yaw = Math.atan2(
      2 * (q.w * q.z + q.x * q.y),
      1 - 2 * (q.y * q.y + q.z * q.z),
    );
    robotGroup.rotation.set(0, -yaw, 0);
  }

  function resizeRenderer() {
    const { clientWidth, clientHeight } = container;
    camera.aspect = clientWidth / clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(clientWidth, clientHeight);
  }
  window.addEventListener('resize', resizeRenderer);

  function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
  }
  animate();
})();
