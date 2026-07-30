import * as THREE from "three";
import { GLTFLoader } from "/js/vendor/GLTFLoader.js";

const $ = (id) => document.getElementById(id);
const radians = Math.PI / 180;
const telemetry = {
  latest: null,
  receivedAt: 0,
  messages: 0,
  messagesWindow: 0,
  messageHz: 0,
  messageWindowStarted: performance.now(),
  lastAttitudeSample: -1,
  lastImuSample: -1,
  lastAttitudeAt: 0,
  attitudeIntervals: [],
  attitudeJitterMs: 0,
};

const finite = (value) => Number.isFinite(value);
const formatted = (value, suffix = "", digits = 1) =>
  finite(value) ? `${Number(value).toFixed(digits)}${suffix}` : "—";
const plain = (value) => value === null || value === undefined || value === "" ? "—" : value;
const wrapDegrees = (value) => ((value + 180) % 360 + 360) % 360 - 180;

function connectionAgeMs(connection) {
  if (finite(connection?.last_frame_at_ms)) {
    return Math.max(0, Date.now() - connection.last_frame_at_ms);
  }
  return connection?.last_frame_age_ms;
}

function buildSignals(id) {
  $(id).innerHTML = ["X", "Y", "Z"]
    .map((axis) => `<div class="signal">${axis}<b data-axis="${axis}">—</b></div>`)
    .join("");
}

function buildMotors(count = 4) {
  $("motors").innerHTML = Array.from({ length: count }, (_, index) => `
    <div class="motor">
      <div class="motor-fill" data-motor-fill="${index}"></div>
      <div class="motor-label">M${index + 1}<b data-motor="${index}">—</b></div>
    </div>
  `).join("");
}

buildSignals("gyro");
buildSignals("accel");
buildMotors();

class FlightView {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(34, 1, 0.05, 100);
    this.camera.position.set(4.4, 2.8, 5.0);
    this.camera.lookAt(0, 0.1, 0);

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      powerPreference: "high-performance",
    });
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.08;
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(container.clientWidth, container.clientHeight, false);
    container.appendChild(this.renderer.domElement);

    this.scene.add(new THREE.HemisphereLight(0xc7e7ff, 0x131820, 2.4));
    const key = new THREE.DirectionalLight(0xffffff, 4.0);
    key.position.set(3.5, 5.0, 4.0);
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0x58d6ff, 2.0);
    rim.position.set(-4.0, 1.0, -3.0);
    this.scene.add(rim);

    const grid = new THREE.GridHelper(10, 20, 0x30424e, 0x172129);
    grid.position.y = -1.15;
    grid.material.transparent = true;
    grid.material.opacity = 0.52;
    this.scene.add(grid);

    const ring = new THREE.Mesh(
      new THREE.TorusGeometry(1.75, 0.008, 6, 160),
      new THREE.MeshBasicMaterial({ color: 0x3c5868, transparent: true, opacity: 0.8 }),
    );
    ring.rotation.x = Math.PI / 2;
    ring.position.y = -0.06;
    this.scene.add(ring);

    this.droneRoot = new THREE.Group();
    this.droneBasis = new THREE.Group();
    this.droneRoot.add(this.droneBasis);
    this.scene.add(this.droneRoot);
    this.lastFrame = performance.now();
    this.frames = 0;
    this.frameWindowStarted = this.lastFrame;
    this.renderFps = 0;
    this.rotors = [];
    this.enabled = true;

    this.resize = this.resize.bind(this);
    this.animate = this.animate.bind(this);
    new ResizeObserver(this.resize).observe(container);
    this.loadModel();
    this.renderer.setAnimationLoop(this.animate);
  }

  async loadModel() {
    const gltf = await new GLTFLoader().loadAsync("/data/CesiumDrone.glb");
    const model = gltf.scene;
    const box = new THREE.Box3().setFromObject(model);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const scale = 3.2 / Math.max(size.x, size.y, size.z);
    model.position.sub(center);
    model.scale.setScalar(scale);
    model.rotation.y = Math.PI;
    model.traverse((node) => {
      if (!node.isMesh) return;
      node.castShadow = false;
      node.receiveShadow = false;
      if (node.material) {
        node.material.envMapIntensity = 0.8;
        node.material.needsUpdate = true;
      }
      if (/prop|rotor|blade/i.test(node.name)) this.rotors.push(node);
    });
    this.droneBasis.add(model);
  }

  setAttitude(values, reference) {
    if (!values || !values.every(finite)) return;
    const origin = reference || [0, 0, 0];
    const roll = wrapDegrees(values[0] - origin[0]);
    const pitch = wrapDegrees(values[1] - origin[1]);
    const heading = wrapDegrees(values[2] - origin[2]);

    // Betaflight Configurator SensorsTab.vue renderModel convention:
    // model X = -pitch, wrapper Y = -heading, model Z = -roll.
    // The lock only subtracts the displayed origin. It does not alter body axes.
    this.droneRoot.rotation.set(0, -heading * radians, 0);
    this.droneBasis.rotation.set(-pitch * radians, 0, -roll * radians);
  }

  setEnabled(enabled) {
    if (this.enabled === enabled) return;
    this.enabled = enabled;
    if (enabled) {
      this.lastFrame = performance.now();
      this.frameWindowStarted = this.lastFrame;
      this.frames = 0;
      this.renderer.setAnimationLoop(this.animate);
    } else {
      this.renderer.setAnimationLoop(null);
      this.renderFps = 0;
      $("renderFps").textContent = "OFF";
    }
  }

  resize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    if (!width || !height) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  animate(now) {
    const delta = Math.min(0.05, Math.max(0.001, (now - this.lastFrame) / 1000));
    this.lastFrame = now;

    const motorValues = telemetry.latest?.signals?.motors || [];
    const averageMotor = motorValues.length
      ? motorValues.reduce((sum, value) => sum + value, 0) / motorValues.length
      : 1000;
    const rotorSpeed = Math.max(0, (averageMotor - 950) / 1000) * 42;
    for (let index = 0; index < this.rotors.length; index += 1) {
      this.rotors[index].rotation.y += rotorSpeed * delta * (index % 2 ? -1 : 1);
    }

    this.renderer.render(this.scene, this.camera);
    this.frames += 1;
    const frameElapsed = now - this.frameWindowStarted;
    if (frameElapsed >= 500) {
      this.renderFps = Math.round(this.frames * 1000 / frameElapsed);
      this.frames = 0;
      this.frameWindowStarted = now;
      $("renderFps").textContent = `${this.renderFps} Hz`;
    }
  }
}

const flightView = new FlightView($("viewport"));

const GroundMode = Object.freeze({
  IDLE: "idle",
  COLLECTING: "collecting",
  LOCKED: "locked",
  REJECTED: "rejected",
});

class GroundReference {
  constructor(statusElement) {
    this.statusElement = statusElement;
    this.mode = GroundMode.IDLE;
    this.reference = null;
    this.gyroBias = null;
    this.quality = null;
    this.lastRejection = "";
    this.samples = [];
    this.targetSamples = 32;
    this.started = false;
    this.updateLabel("GROUND · RAW");
  }

  updateLabel(text) {
    this.statusElement.dataset.state = this.mode;
    this.statusElement.textContent = text;
  }

  progressLabel(prefix) {
    const lastReason = this.lastRejection && this.mode === GroundMode.COLLECTING
      ? ` · LAST: ${this.lastRejection}`
      : "";
    this.updateLabel(`${prefix} · ${this.samples.length}/${this.targetSamples}${lastReason}`);
  }

  start(data) {
    if (data?.health?.armed !== false) {
      this.reject("DISARM FIRST");
      return;
    }
    this.mode = GroundMode.COLLECTING;
    this.reference = null;
    this.gyroBias = null;
    this.quality = null;
    this.lastRejection = "";
    this.samples.length = 0;
    this.started = true;
    this.progressLabel("GROUND · COLLECTING");
  }

  reject(message) {
    this.mode = GroundMode.REJECTED;
    this.reference = null;
    this.gyroBias = null;
    this.quality = null;
    this.lastRejection = message;
    this.samples.length = 0;
    this.progressLabel(`GROUND · REJECTED: ${message}`);
  }

  waitForStill(message) {
    this.lastRejection = message;
    this.samples.length = 0;
    this.progressLabel(`GROUND · REJECTED: ${message}`);
  }

  observe(data) {
    if (!this.started && data?.connection?.state === "live" && data?.health?.armed === false) {
      this.start(data);
    }
    if (this.mode !== GroundMode.COLLECTING) return;
    if (data?.health?.armed !== false) {
      this.reject("DISARM FIRST");
      return;
    }

    const attitude = data?.signals?.attitude_deg;
    const gyro = data?.signals?.gyro_msp;
    const accel = data?.signals?.accel_msp;
    if (![attitude, gyro, accel].every((values) => values?.length === 3 && values.every(finite))) {
      this.progressLabel("GROUND · WAITING FOR IMU");
      return;
    }

    const gyroPeak = Math.max(...gyro.map(Math.abs));
    if (gyroPeak > 8) {
      this.waitForStill(`MOTION ${gyroPeak.toFixed(0)} dps`);
      return;
    }

    this.samples.push({
      roll: attitude[0],
      pitch: attitude[1],
      heading: attitude[2],
      gyro: [...gyro],
      accelNorm: Math.hypot(...accel),
    });
    this.progressLabel("GROUND · ACCEPTED");
    if (this.samples.length === this.targetSamples) this.finish();
  }

  finish() {
    const meanAngle = (field) => {
      const sine = this.samples.reduce((sumValue, sample) => sumValue + Math.sin(sample[field] * radians), 0);
      const cosine = this.samples.reduce((sumValue, sample) => sumValue + Math.cos(sample[field] * radians), 0);
      return Math.atan2(sine, cosine) / radians;
    };
    const rollMean = meanAngle("roll");
    const pitchMean = meanAngle("pitch");
    const tiltSpread = this.samples.reduce((largest, sample) => Math.max(
      largest,
      Math.abs(wrapDegrees(sample.roll - rollMean)),
      Math.abs(wrapDegrees(sample.pitch - pitchMean)),
    ), 0);
    const accelMean = this.samples.reduce((sum, sample) => sum + sample.accelNorm, 0) / this.samples.length;
    const accelSpread = this.samples.reduce(
      (largest, sample) => Math.max(largest, Math.abs(sample.accelNorm - accelMean)),
      0,
    );

    if (tiltSpread > 1.0) {
      this.waitForStill(`TILT ${tiltSpread.toFixed(1)}°`);
      return;
    }
    if (accelMean < 100) {
      this.waitForStill("INVALID ACCEL");
      return;
    }
    if (accelSpread > accelMean * 0.06) {
      const variation = accelSpread * 100 / accelMean;
      this.waitForStill(`ACCEL VARIATION ${variation.toFixed(1)}%`);
      return;
    }

    const gyroBias = [0, 1, 2].map((axis) =>
      this.samples.reduce((sumValue, sample) => sumValue + sample.gyro[axis], 0) / this.samples.length);
    const gyroPeak = this.samples.reduce((largest, sample) => Math.max(
      largest,
      ...sample.gyro.map(Math.abs),
    ), 0);
    const attitudeScore = Math.max(0, 1 - tiltSpread / 1.0);
    const accelScore = Math.max(0, 1 - accelSpread / (accelMean * 0.06));
    const gyroScore = Math.max(0, 1 - gyroPeak / 8);

    this.reference = [rollMean, pitchMean, meanAngle("heading")];
    this.gyroBias = Object.freeze(gyroBias);
    const validLockScore = (
      0.5 * attitudeScore
      + 0.3 * accelScore
      + 0.2 * gyroScore
    );
    this.quality = Math.round(70 + 30 * validLockScore);
    this.mode = GroundMode.LOCKED;
    this.lastRejection = "";
    this.updateLabel(`GROUND · LOCKED ${this.quality}`);
  }
}

class ImuScope {
  constructor(canvas) {
    this.canvas = canvas;
    this.context = canvas.getContext("2d", { alpha: false });
    this.capacity = 360;
    this.cursor = 0;
    this.count = 0;
    this.gyro = Array.from({ length: 3 }, () => new Float32Array(this.capacity));
    this.accel = Array.from({ length: 3 }, () => new Float32Array(this.capacity));
    this.enabled = false;
    this.frame = null;
    this.colors = ["#ff626f", "#5cf29a", "#5db4ff"];
    this.draw = this.draw.bind(this);
    this.resize = this.resize.bind(this);
    new ResizeObserver(this.resize).observe(canvas.parentElement);
    this.resize();
  }

  resize() {
    const bounds = this.canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    this.canvas.width = Math.max(1, Math.round(bounds.width * ratio));
    this.canvas.height = Math.max(1, Math.round(bounds.height * ratio));
    this.context.setTransform(ratio, 0, 0, ratio, 0, 0);
  }

  push(gyro, accel) {
    if (![gyro, accel].every((values) => values?.length === 3 && values.every(finite))) return;
    for (let axis = 0; axis < 3; axis += 1) {
      this.gyro[axis][this.cursor] = gyro[axis];
      this.accel[axis][this.cursor] = accel[axis];
    }
    this.cursor = (this.cursor + 1) % this.capacity;
    this.count = Math.min(this.count + 1, this.capacity);
    if (this.enabled && this.frame === null) {
      this.frame = requestAnimationFrame(this.draw);
    }
  }

  setEnabled(enabled) {
    if (this.enabled === enabled) return;
    this.enabled = enabled;
    if (enabled) {
      this.frame = requestAnimationFrame(this.draw);
    } else if (this.frame !== null) {
      cancelAnimationFrame(this.frame);
      this.frame = null;
    }
  }

  plot(values, top, height, scale) {
    const context = this.context;
    const width = this.canvas.clientWidth;
    const center = top + height / 2;
    const samples = Math.max(2, this.count);
    for (let axis = 0; axis < 3; axis += 1) {
      context.beginPath();
      context.strokeStyle = this.colors[axis];
      context.lineWidth = 1.35;
      for (let sample = 0; sample < this.count; sample += 1) {
        const index = (this.cursor - this.count + sample + this.capacity) % this.capacity;
        const x = 18 + sample * (width - 36) / (samples - 1);
        const y = center - Math.max(-1, Math.min(1, values[axis][index] / scale)) * (height * 0.42);
        if (sample === 0) context.moveTo(x, y);
        else context.lineTo(x, y);
      }
      context.stroke();
    }
  }

  draw() {
    this.frame = null;
    if (!this.enabled) return;
    const context = this.context;
    const width = this.canvas.clientWidth;
    const height = this.canvas.clientHeight;
    context.fillStyle = "#07090b";
    context.fillRect(0, 0, width, height);

    const panelHeight = (height - 96) / 2;
    const panels = [
      { label: "GYROSCOPE · MSP RAW", top: 58, values: this.gyro, minimum: 16 },
      { label: "ACCELEROMETER · MSP RAW", top: 74 + panelHeight, values: this.accel, minimum: 2300 },
    ];
    context.font = "10px ui-monospace, SFMono-Regular, Menlo, monospace";
    for (const panel of panels) {
      let scale = panel.minimum;
      for (let axis = 0; axis < 3; axis += 1) {
        for (let sample = 0; sample < this.count; sample += 1) {
          scale = Math.max(scale, Math.abs(panel.values[axis][sample]));
        }
      }
      scale *= 1.08;
      context.strokeStyle = "#202830";
      context.lineWidth = 1;
      context.strokeRect(18, panel.top, width - 36, panelHeight);
      context.beginPath();
      context.moveTo(18, panel.top + panelHeight / 2);
      context.lineTo(width - 18, panel.top + panelHeight / 2);
      context.stroke();
      context.fillStyle = "#7e8a94";
      context.fillText(`${panel.label}   ±${Math.round(scale)}`, 18, panel.top - 9);
      this.plot(panel.values, panel.top, panelHeight, scale);
    }
    context.fillStyle = this.colors[0];
    context.fillText("X", width - 78, 36);
    context.fillStyle = this.colors[1];
    context.fillText("Y", width - 56, 36);
    context.fillStyle = this.colors[2];
    context.fillText("Z", width - 34, 36);
  }
}

const groundReference = new GroundReference($("groundState"));
const imuScope = new ImuScope($("imuScope"));
let scopeMode = false;

function setVector(id, values, digits = 0) {
  const labels = ["X", "Y", "Z"];
  labels.forEach((axis, index) => {
    const element = document.querySelector(`#${id} [data-axis="${axis}"]`);
    if (element) element.textContent = formatted(values?.[index], "", digits);
  });
}

function updateMotors(motors) {
  const shown = motors?.length ? motors : [null, null, null, null];
  if ($("motors").children.length !== shown.length) buildMotors(shown.length);
  shown.forEach((motor, index) => {
    const label = document.querySelector(`[data-motor="${index}"]`);
    const fill = document.querySelector(`[data-motor-fill="${index}"]`);
    if (label) label.textContent = plain(motor);
    if (fill) {
      const normalized = finite(motor) ? Math.max(0, Math.min(1, (motor - 1000) / 1000)) : 0;
      fill.style.height = `${normalized * 100}%`;
    }
  });
}

function renderTelemetry(data) {
  const connection = data.connection || {};
  const controller = data.controller || {};
  const health = data.health || {};
  const signals = data.signals || {};
  const traffic = data.traffic || {};
  const state = connection.state || "waiting_usb";
  const displayState = state === "live"
    ? "live"
    : state === "opening" || state === "handshake"
      ? "connecting"
      : "offline";
  const port = connection.port || connection.last_port;

  $("linkState").dataset.state = displayState;
  $("linkText").textContent = displayState === "live"
    ? `LIVE · ${port?.split("/").pop() || "MSP"}`
    : displayState === "connecting"
      ? `CONNECTING · ${port?.split("/").pop() || "USB"}`
      : `OFFLINE · ${connection.message || "Waiting for USB"}`;
  $("stageStatus").textContent = displayState === "live"
    ? (scopeMode ? "Raw IMU scope" : "Vehicle attitude")
    : displayState === "connecting" ? "Connecting to flight controller" : "Last flight-controller sample";

  $("controller").textContent = plain(controller.variant);
  $("controllerMeta").textContent = [controller.version, controller.board, `API ${controller.api_version || "—"}`]
    .filter(Boolean).join(" · ");

  const armed = health.armed;
  $("armed").textContent = armed === true ? "ARMED" : armed === false ? "DISARMED" : "—";
  $("armed").className = armed === true ? "armed" : "armed disarmed";
  $("loop").textContent = finite(health.loop_hz) ? `${health.loop_hz} Hz` : "—";
  $("cpu").textContent = formatted(health.cpu_load_percent, "%");
  $("i2c").textContent = plain(health.i2c_errors);
  $("sensors").textContent = health.sensors?.length ? health.sensors.join(" · ") : "—";

  $("voltage").textContent = formatted(signals.battery_v, " V");
  $("current").textContent = formatted(signals.current_a, " A", 2);
  $("mah").textContent = finite(signals.mah) ? `${signals.mah} mAh` : "—";
  $("rssi").textContent = formatted(signals.rssi_percent, "%");
  $("altitude").textContent = formatted(signals.altitude_m, " m", 2);
  $("vario").textContent = formatted(signals.vario_mps, " m/s", 2);

  const attitude = signals.attitude_deg || [];
  $("roll").textContent = formatted(attitude[0], "°");
  $("pitch").textContent = formatted(attitude[1], "°");
  $("yaw").textContent = formatted(attitude[2], "°");
  setVector("gyro", signals.gyro_msp);
  setVector("accel", signals.accel_msp);
  updateMotors(signals.motors);

  $("stateHz").textContent = `${telemetry.messageHz || 0} Hz`;
  $("mspFps").textContent = `${formatted(traffic.frames_per_second, "", 0)} Hz`;
  $("attitudeHz").textContent = `${formatted(traffic.attitude_per_second, "", 0)} Hz`;
  $("imuHz").textContent = `${formatted(traffic.imu_per_second, "", 0)} Hz`;
  $("jitter").textContent = formatted(telemetry.attitudeJitterMs, " ms", 1);
  $("age").textContent = formatted(connectionAgeMs(connection), " ms", 0);
}

function acceptState(data) {
  telemetry.latest = data;
  telemetry.receivedAt = performance.now();
  telemetry.messages += 1;
  telemetry.messagesWindow += 1;
  const traffic = data.traffic || {};

  if (traffic.attitude_samples_total !== telemetry.lastAttitudeSample) {
    const attitudeNow = performance.now();
    const interval = attitudeNow - telemetry.lastAttitudeAt;
    if (telemetry.lastAttitudeAt > 0 && interval < 100) {
      telemetry.attitudeIntervals.push(interval);
      if (telemetry.attitudeIntervals.length > 64) telemetry.attitudeIntervals.shift();
      const mean = telemetry.attitudeIntervals.reduce((sum, value) => sum + value, 0)
        / telemetry.attitudeIntervals.length;
      telemetry.attitudeJitterMs = Math.sqrt(
        telemetry.attitudeIntervals.reduce(
          (sum, value) => sum + (value - mean) ** 2,
          0,
        ) / telemetry.attitudeIntervals.length,
      );
    } else {
      telemetry.attitudeIntervals.length = 0;
      telemetry.attitudeJitterMs = 0;
    }
    telemetry.lastAttitudeAt = attitudeNow;
    telemetry.lastAttitudeSample = traffic.attitude_samples_total;
    groundReference.observe(data);
    flightView.setAttitude(
      data.signals?.attitude_deg,
      groundReference.reference,
    );
  }
  if (traffic.imu_samples_total !== telemetry.lastImuSample) {
    telemetry.lastImuSample = traffic.imu_samples_total;
    imuScope.push(data.signals?.gyro_msp, data.signals?.accel_msp);
  }
}

$("groundButton").addEventListener("click", () => {
  groundReference.start(telemetry.latest);
});

$("scopeButton").addEventListener("click", () => {
  scopeMode = !scopeMode;
  document.querySelector(".stage").classList.toggle("scope-mode", scopeMode);
  $("scopeButton").setAttribute("aria-pressed", String(scopeMode));
  $("scopeButton").textContent = scopeMode ? "3D VIEW" : "IMU SCOPE";
  flightView.setEnabled(!scopeMode);
  if (scopeMode) imuScope.resize();
  imuScope.setEnabled(scopeMode);
  if (telemetry.latest) renderTelemetry(telemetry.latest);
});

const events = new EventSource("/events");
events.addEventListener("state", (event) => {
  acceptState(JSON.parse(event.data));
});
events.onerror = () => {
  $("linkState").dataset.state = "connecting";
  $("linkText").textContent = "CONNECTING · LOCALHOST";
};

setInterval(() => {
  const now = performance.now();
  const elapsed = now - telemetry.messageWindowStarted;
  telemetry.messageHz = Math.round(telemetry.messagesWindow * 1000 / elapsed);
  telemetry.messagesWindow = 0;
  telemetry.messageWindowStarted = now;
}, 500);

setInterval(() => {
  if (telemetry.latest) renderTelemetry(telemetry.latest);
}, 50);
