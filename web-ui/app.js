const TOPICS = {
  battery: { topic: "/power", type: "sensor_msgs/msg/BatteryState" },
  charger: { topic: "/power/charger_present", type: "std_msgs/msg/Bool" },
  chargeVoltage: { topic: "/power/charge_voltage", type: "std_msgs/msg/Float32" },
  emergency: { topic: "/hardware/emergency", type: "std_msgs/msg/Bool" },
  rain: { topic: "/hardware/rain", type: "std_msgs/msg/Bool" },
  uiEvent: { topic: "/hardware/ui_event", type: "open_mower_next/msg/UiButtonEvent" },
  gpsFix: { topic: "/gps/fix", type: "sensor_msgs/msg/NavSatFix" },
  gpsOdom: { topic: "/gps/odom", type: "nav_msgs/msg/Odometry" },
  map: {
    topic: "/mowing_map",
    type: "open_mower_next/msg/Map",
    qos: { durability: "transient_local", reliability: "reliable", history: "keep_last", depth: 1 },
  },
  localizedOdom: { topic: "/odometry/filtered/map", type: "nav_msgs/msg/Odometry" },
  rpiTemperature: { topic: "/hardware/raspi/cpu_temperature", type: "sensor_msgs/msg/Temperature" },
  vescLeft: { topic: "/hardware/vesc/left/status", type: "vesc_msgs/msg/VescStateStamped" },
  vescRight: { topic: "/hardware/vesc/right/status", type: "vesc_msgs/msg/VescStateStamped" },
  vescMower: { topic: "/hardware/vesc/mower/status", type: "vesc_msgs/msg/VescStateStamped" },
};

const BUTTONS = {
  home: { id: 2, label: "HOME" },
  start: { id: 3, label: "START" },
  s1: { id: 4, label: "S1" },
  s2: { id: 5, label: "S2" },
};

const PRESS_SINGLE = 0;

const state = {
  connected: false,
  lastMessageAt: null,
  batteryAt: null,
  gpsAt: null,
  mapAt: null,
  poseAt: null,
  hardwareAt: null,
  rpiTemperatureAt: null,
  vescAt: { left: null, right: null, mower: null },
  battery: null,
  gpsFix: null,
  gpsOdom: null,
  map: null,
  localizedOdom: null,
  charger: null,
  chargeVoltage: null,
  emergency: null,
  rain: null,
  uiEvent: null,
  rpiTemperature: null,
  vescStatus: { left: null, right: null, mower: null },
};

const els = {};
let ros = null;

document.addEventListener("DOMContentLoaded", () => {
  bindElements();
  els.rosbridgeUrl.value = defaultRosbridgeUrl();
  els.connectionForm.addEventListener("submit", (event) => {
    event.preventDefault();
    connect(els.rosbridgeUrl.value.trim());
  });
  els.clearLog.addEventListener("click", () => {
    els.logList.replaceChildren();
  });
  document.querySelectorAll("[data-service]").forEach((button) => {
    button.addEventListener("click", () => handleServiceButton(button.dataset.service));
  });
  document.querySelectorAll("[data-button]").forEach((button) => {
    button.addEventListener("click", () => publishButton(button.dataset.button));
  });
  setControlsEnabled(false);
  render();
  setInterval(render, 1000);
});

function bindElements() {
  for (const id of [
    "connection-form",
    "rosbridge-url",
    "connect-button",
    "last-update",
    "connection-status",
    "emergency-status",
    "gps-status",
    "charger-status",
    "map-age",
    "pose-label",
    "map-view",
    "map-empty",
    "battery-age",
    "battery-percent",
    "battery-fill",
    "battery-voltage",
    "battery-current",
    "battery-status",
    "charge-voltage",
    "gps-age",
    "gps-fix-label",
    "gps-latitude",
    "gps-longitude",
    "gps-altitude",
    "gps-accuracy",
    "gps-speed",
    "gps-heading",
    "hardware-age",
    "emergency-value",
    "rain-value",
    "charger-value",
    "button-value",
    "rpi-temperature",
    "left-vesc-temperature",
    "right-vesc-temperature",
    "mower-vesc-temperature",
    "command-feedback",
    "clear-log",
    "log-list",
  ]) {
    els[toCamel(id)] = document.getElementById(id);
  }
}

function toCamel(id) {
  return id.replace(/-([a-z])/g, (_, letter) => letter.toUpperCase());
}

function defaultRosbridgeUrl() {
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const hostname = window.location.hostname || "127.0.0.1";
  return `${protocol}//${hostname}:9090`;
}

function connect(url) {
  if (!url) {
    log("Enter a rosbridge WebSocket URL.");
    return;
  }
  if (ros) {
    ros.close();
  }
  ros = new RosbridgeClient(url);
  setConnectionState("warn", "Connecting", "ROS bridge");
  els.connectButton.disabled = true;
  log(`Connecting to ${url}`);

  ros.onOpen = () => {
    state.connected = true;
    els.connectButton.disabled = false;
    setControlsEnabled(true);
    setConnectionState("ok", "Connected", "ROS bridge");
    subscribeAll();
    advertiseUiEvents();
    log("Connected to rosbridge.");
    render();
  };
  ros.onClose = () => {
    state.connected = false;
    els.connectButton.disabled = false;
    setControlsEnabled(false);
    setConnectionState("bad", "Disconnected", "ROS bridge");
    log("Disconnected from rosbridge.");
    render();
  };
  ros.onError = () => {
    state.connected = false;
    els.connectButton.disabled = false;
    setControlsEnabled(false);
    setConnectionState("bad", "Error", "ROS bridge");
    log("Connection error.");
    render();
  };
  ros.onMessage = (message) => handleRosMessage(message);
  ros.connect();
}

function subscribeAll() {
  Object.values(TOPICS).forEach(({ topic, type, qos }) => {
    ros.subscribe(topic, type, 500, qos);
  });
}

function advertiseUiEvents() {
  ros.send({
    op: "advertise",
    topic: TOPICS.uiEvent.topic,
    type: TOPICS.uiEvent.type,
  });
}

function handleRosMessage(message) {
  if (message.op !== "publish") {
    if (message.op === "service_response") {
      const outcome = message.result ? "succeeded" : "failed";
      setFeedback(`Service ${message.service || message.id} ${outcome}.`);
    } else if (message.op === "status" && message.level === "error") {
      log(message.msg || "rosbridge reported an error.");
    }
    return;
  }

  state.lastMessageAt = Date.now();
  switch (message.topic) {
    case TOPICS.battery.topic:
      state.battery = message.msg;
      state.batteryAt = Date.now();
      break;
    case TOPICS.charger.topic:
      state.charger = Boolean(message.msg.data);
      state.hardwareAt = Date.now();
      break;
    case TOPICS.chargeVoltage.topic:
      state.chargeVoltage = Number(message.msg.data);
      state.hardwareAt = Date.now();
      break;
    case TOPICS.emergency.topic:
      state.emergency = Boolean(message.msg.data);
      state.hardwareAt = Date.now();
      break;
    case TOPICS.rain.topic:
      state.rain = Boolean(message.msg.data);
      state.hardwareAt = Date.now();
      break;
    case TOPICS.uiEvent.topic:
      state.uiEvent = message.msg;
      state.hardwareAt = Date.now();
      break;
    case TOPICS.gpsFix.topic:
      state.gpsFix = message.msg;
      state.gpsAt = Date.now();
      break;
    case TOPICS.gpsOdom.topic:
      state.gpsOdom = message.msg;
      state.gpsAt = Date.now();
      break;
    case TOPICS.map.topic:
      state.map = message.msg;
      state.mapAt = Date.now();
      break;
    case TOPICS.localizedOdom.topic:
      state.localizedOdom = message.msg;
      state.poseAt = Date.now();
      break;
    case TOPICS.rpiTemperature.topic:
      state.rpiTemperature = message.msg;
      state.rpiTemperatureAt = Date.now();
      state.hardwareAt = Date.now();
      break;
    case TOPICS.vescLeft.topic:
      state.vescStatus.left = message.msg;
      state.vescAt.left = Date.now();
      state.hardwareAt = Date.now();
      break;
    case TOPICS.vescRight.topic:
      state.vescStatus.right = message.msg;
      state.vescAt.right = Date.now();
      state.hardwareAt = Date.now();
      break;
    case TOPICS.vescMower.topic:
      state.vescStatus.mower = message.msg;
      state.vescAt.mower = Date.now();
      state.hardwareAt = Date.now();
      break;
    default:
      return;
  }
  render();
}

function handleServiceButton(name) {
  if (!ensureConnected()) {
    return;
  }
  if (name === "set-emergency") {
    ros.callService("/hardware/set_emergency", "std_srvs/srv/SetBool", { data: true });
    setFeedback("Requested emergency stop.");
    log("Called /hardware/set_emergency.");
  } else if (name === "clear-emergency") {
    ros.callService("/hardware/clear_emergency", "std_srvs/srv/Trigger", {});
    setFeedback("Requested emergency clear.");
    log("Called /hardware/clear_emergency.");
  }
}

function publishButton(name) {
  if (!ensureConnected()) {
    return;
  }
  const button = BUTTONS[name];
  if (!button) {
    return;
  }
  ros.publish(TOPICS.uiEvent.topic, {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "" },
    button_id: button.id,
    press_duration: PRESS_SINGLE,
  });
  setFeedback(`Published ${button.label} button event.`);
  log(`Published ${button.label} on /hardware/ui_event.`);
}

function ensureConnected() {
  if (!ros || !state.connected) {
    setFeedback("Connect to rosbridge first.");
    return false;
  }
  return true;
}

function render() {
  renderConnection();
  renderBattery();
  renderGps();
  renderMap();
  renderHardware();
}

function renderConnection() {
  const lastText = state.lastMessageAt ? `Last ROS message ${ageText(state.lastMessageAt)} ago` : "Waiting for ROS data";
  els.lastUpdate.textContent = state.connected ? lastText : "Disconnected";
}

function renderBattery() {
  const battery = state.battery;
  if (!battery) {
    els.batteryAge.textContent = "No message yet";
    return;
  }
  const percent = normalizeBatteryPercent(battery.percentage);
  els.batteryAge.textContent = `Updated ${ageText(state.batteryAt)} ago`;
  els.batteryPercent.textContent = Number.isFinite(percent) ? `${Math.round(percent)}%` : "--%";
  els.batteryFill.style.width = Number.isFinite(percent) ? `${clamp(percent, 0, 100)}%` : "0%";
  els.batteryVoltage.textContent = formatUnit(battery.voltage, "V", 2);
  els.batteryCurrent.textContent = formatUnit(battery.current, "A", 2);
  els.batteryStatus.textContent = batteryStatusLabel(battery.power_supply_status);
  els.chargeVoltage.textContent = formatUnit(state.chargeVoltage, "V", 2);
}

function renderGps() {
  const fix = state.gpsFix;
  if (!fix) {
    setStatusPill(els.gpsStatus, "idle", "No data", "GPS fix");
    els.gpsAge.textContent = "No message yet";
    return;
  }

  const status = Number(fix.status?.status);
  const fixLabel = gpsFixLabel(status);
  const fixClass = status >= 0 ? "ok" : "bad";
  const accuracy = gpsAccuracy(fix);
  els.gpsAge.textContent = `Updated ${ageText(state.gpsAt)} ago`;
  els.gpsFixLabel.textContent = fixLabel;
  els.gpsLatitude.textContent = formatCoordinate(fix.latitude);
  els.gpsLongitude.textContent = formatCoordinate(fix.longitude);
  els.gpsAltitude.textContent = formatUnit(fix.altitude, "m", 1);
  els.gpsAccuracy.textContent = formatUnit(accuracy, "m", 2);
  els.gpsSpeed.textContent = formatUnit(gpsSpeed(state.gpsOdom), "m/s", 2);
  els.gpsHeading.textContent = formatUnit(gpsHeading(state.gpsOdom), "deg", 0);
  setStatusPill(els.gpsStatus, fixClass, fixLabel, accuracy ? `${accuracy.toFixed(2)} m` : "GPS fix");
}

function renderMap() {
  const canvas = els.mapView;
  const wrapper = canvas.parentElement;
  const rect = wrapper.getBoundingClientRect();
  const pixelRatio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(rect.width));
  const height = Math.max(1, Math.floor(rect.height));
  if (canvas.width !== Math.floor(width * pixelRatio) || canvas.height !== Math.floor(height * pixelRatio)) {
    canvas.width = Math.floor(width * pixelRatio);
    canvas.height = Math.floor(height * pixelRatio);
    canvas.style.width = `${width}px`;
    canvas.style.height = `${height}px`;
  }

  const ctx = canvas.getContext("2d");
  ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  ctx.clearRect(0, 0, width, height);
  drawMapBackground(ctx, width, height);

  const map = state.map;
  const pose = state.localizedOdom?.pose?.pose;
  const bounds = mapBounds(map, pose);
  els.mapEmpty.hidden = Boolean(bounds);

  if (!bounds) {
    els.mapAge.textContent = "Waiting for map and pose";
    els.poseLabel.textContent = "--";
    return;
  }

  const transform = mapTransform(bounds, width, height);
  drawMapGrid(ctx, bounds, transform, width, height);
  drawAreas(ctx, map, transform);
  drawDockingStations(ctx, map, transform);
  if (pose) {
    drawMower(ctx, pose, transform);
  }

  const mapText = state.mapAt ? `Map ${ageText(state.mapAt)} ago` : "Waiting for map";
  const poseText = state.poseAt ? `pose ${ageText(state.poseAt)} ago` : "waiting for pose";
  els.mapAge.textContent = `${mapText}, ${poseText}`;
  els.poseLabel.textContent = pose ? formatPose(pose) : "--";
}

function drawMapBackground(ctx, width, height) {
  ctx.fillStyle = "#f7f6f0";
  ctx.fillRect(0, 0, width, height);
}

function drawMapGrid(ctx, bounds, transform, width, height) {
  const span = Math.max(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY);
  const step = niceGridStep(span / 6);
  const startX = Math.floor(bounds.minX / step) * step;
  const startY = Math.floor(bounds.minY / step) * step;
  ctx.save();
  ctx.strokeStyle = "#ded8cb";
  ctx.lineWidth = 1;
  for (let x = startX; x <= bounds.maxX; x += step) {
    const point = transform({ x, y: bounds.minY });
    ctx.beginPath();
    ctx.moveTo(point.x, 0);
    ctx.lineTo(point.x, height);
    ctx.stroke();
  }
  for (let y = startY; y <= bounds.maxY; y += step) {
    const point = transform({ x: bounds.minX, y });
    ctx.beginPath();
    ctx.moveTo(0, point.y);
    ctx.lineTo(width, point.y);
    ctx.stroke();
  }
  ctx.restore();
}

function drawAreas(ctx, map, transform) {
  const areas = map?.areas || [];
  for (const area of areas) {
    const points = area?.area?.polygon?.points || [];
    if (points.length < 2) {
      continue;
    }
    const style = areaStyle(Number(area.type));
    drawPolygon(ctx, points, transform, style.fill, style.stroke);
  }
}

function drawPolygon(ctx, points, transform, fill, stroke) {
  ctx.save();
  ctx.beginPath();
  points.forEach((point, index) => {
    const screen = transform(point);
    if (index === 0) {
      ctx.moveTo(screen.x, screen.y);
    } else {
      ctx.lineTo(screen.x, screen.y);
    }
  });
  ctx.closePath();
  ctx.fillStyle = fill;
  ctx.strokeStyle = stroke;
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawDockingStations(ctx, map, transform) {
  const stations = map?.docking_stations || [];
  for (const station of stations) {
    const pose = station?.pose?.pose;
    const approach = station?.approach_pose?.pose;
    if (!pose?.position) {
      continue;
    }
    const dock = transform(pose.position);
    ctx.save();
    if (approach?.position) {
      const start = transform(approach.position);
      ctx.strokeStyle = "#7a4f16";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(start.x, start.y);
      ctx.lineTo(dock.x, dock.y);
      ctx.stroke();
    }
    ctx.fillStyle = "#5b3715";
    ctx.strokeStyle = "#fffdf8";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(dock.x - 6, dock.y - 6, 12, 12);
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
}

function drawMower(ctx, pose, transform) {
  const position = pose.position;
  const screen = transform(position);
  const yaw = quaternionYaw(pose.orientation);
  ctx.save();
  ctx.translate(screen.x, screen.y);
  ctx.rotate(-yaw);
  ctx.fillStyle = "#1f6f95";
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(-10, -8);
  ctx.lineTo(-7, 0);
  ctx.lineTo(-10, 8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function mapBounds(map, pose) {
  const points = [];
  for (const area of map?.areas || []) {
    points.push(...(area?.area?.polygon?.points || []));
  }
  for (const station of map?.docking_stations || []) {
    if (station?.pose?.pose?.position) {
      points.push(station.pose.pose.position);
    }
    if (station?.approach_pose?.pose?.position) {
      points.push(station.approach_pose.pose.position);
    }
  }
  if (pose?.position) {
    points.push(pose.position);
  }
  const finitePoints = points.filter((point) => Number.isFinite(Number(point.x)) && Number.isFinite(Number(point.y)));
  if (!finitePoints.length) {
    return null;
  }
  const xs = finitePoints.map((point) => Number(point.x));
  const ys = finitePoints.map((point) => Number(point.y));
  let minX = Math.min(...xs);
  let maxX = Math.max(...xs);
  let minY = Math.min(...ys);
  let maxY = Math.max(...ys);
  const padding = Math.max(1, Math.max(maxX - minX, maxY - minY) * 0.08);
  minX -= padding;
  maxX += padding;
  minY -= padding;
  maxY += padding;
  return { minX, maxX, minY, maxY };
}

function mapTransform(bounds, width, height) {
  const padding = 24;
  const spanX = Math.max(1, bounds.maxX - bounds.minX);
  const spanY = Math.max(1, bounds.maxY - bounds.minY);
  const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
  const offsetX = (width - spanX * scale) / 2;
  const offsetY = (height - spanY * scale) / 2;
  return (point) => ({
    x: offsetX + (Number(point.x) - bounds.minX) * scale,
    y: height - offsetY - (Number(point.y) - bounds.minY) * scale,
  });
}

function areaStyle(type) {
  if (type === 0) {
    return { fill: "rgba(180, 59, 66, 0.20)", stroke: "#b43b42" };
  }
  if (type === 1) {
    return { fill: "rgba(35, 108, 143, 0.16)", stroke: "#236c8f" };
  }
  return { fill: "rgba(47, 125, 88, 0.20)", stroke: "#2f7d58" };
}

function niceGridStep(value) {
  const power = Math.pow(10, Math.floor(Math.log10(Math.max(value, 1))));
  const normalized = value / power;
  if (normalized <= 1) {
    return power;
  }
  if (normalized <= 2) {
    return power * 2;
  }
  if (normalized <= 5) {
    return power * 5;
  }
  return power * 10;
}

function renderHardware() {
  els.hardwareAge.textContent = state.hardwareAt ? `Updated ${ageText(state.hardwareAt)} ago` : "Waiting for status topics";
  els.emergencyValue.textContent = boolLabel(state.emergency, "Active", "Clear");
  els.rainValue.textContent = boolLabel(state.rain, "Detected", "Clear");
  els.chargerValue.textContent = boolLabel(state.charger, "Present", "Not present");
  els.buttonValue.textContent = uiButtonLabel(state.uiEvent);
  els.rpiTemperature.textContent = temperatureLabel(state.rpiTemperature, state.rpiTemperatureAt);
  els.leftVescTemperature.textContent = vescTemperatureLabel(state.vescStatus.left, state.vescAt.left);
  els.rightVescTemperature.textContent = vescTemperatureLabel(state.vescStatus.right, state.vescAt.right);
  els.mowerVescTemperature.textContent = vescTemperatureLabel(state.vescStatus.mower, state.vescAt.mower);

  if (state.emergency === true) {
    setStatusPill(els.emergencyStatus, "bad", "Active", "Emergency");
  } else if (state.emergency === false) {
    setStatusPill(els.emergencyStatus, "ok", "Clear", "Emergency");
  } else {
    setStatusPill(els.emergencyStatus, "idle", "Unknown", "Emergency");
  }

  if (state.charger === true) {
    setStatusPill(els.chargerStatus, "ok", "Present", "Charger");
  } else if (state.charger === false) {
    setStatusPill(els.chargerStatus, "warn", "Away", "Charger");
  } else {
    setStatusPill(els.chargerStatus, "idle", "Unknown", "Charger");
  }
}

function setConnectionState(kind, label, meta) {
  setStatusPill(els.connectionStatus, kind, label, meta);
}

function setStatusPill(element, kind, label, meta) {
  element.classList.remove("state-ok", "state-warn", "state-bad", "state-idle");
  element.classList.add(`state-${kind}`);
  element.querySelector("strong").textContent = label;
  element.querySelector("small").textContent = meta;
}

function setControlsEnabled(enabled) {
  document.querySelectorAll("[data-service], [data-button]").forEach((button) => {
    button.disabled = !enabled;
  });
}

function setFeedback(text) {
  els.commandFeedback.textContent = text;
}

function log(text) {
  const item = document.createElement("li");
  const time = document.createElement("time");
  const message = document.createElement("span");
  time.textContent = new Date().toLocaleTimeString();
  message.textContent = text;
  item.append(time, message);
  els.logList.prepend(item);
  while (els.logList.children.length > 30) {
    els.logList.lastElementChild.remove();
  }
}

function normalizeBatteryPercent(value) {
  const number = Number(value);
  if (!Number.isFinite(number) || number < 0) {
    return NaN;
  }
  return number <= 1 ? number * 100 : number;
}

function batteryStatusLabel(value) {
  const labels = {
    0: "Unknown",
    1: "Charging",
    2: "Discharging",
    3: "Not charging",
    4: "Full",
  };
  return labels[Number(value)] || "--";
}

function gpsFixLabel(value) {
  const labels = {
    "-1": "No fix",
    0: "Fix",
    1: "SBAS",
    2: "GBAS",
  };
  return labels[value] || "Unknown";
}

function gpsAccuracy(fix) {
  const covariance = fix.position_covariance || [];
  const variance = Number(covariance[0]);
  return variance >= 0 ? Math.sqrt(variance) : NaN;
}

function gpsSpeed(odom) {
  const linear = odom?.twist?.twist?.linear;
  if (!linear) {
    return NaN;
  }
  const x = Number(linear.x) || 0;
  const y = Number(linear.y) || 0;
  const z = Number(linear.z) || 0;
  return Math.sqrt(x * x + y * y + z * z);
}

function gpsHeading(odom) {
  const orientation = odom?.pose?.pose?.orientation;
  if (!orientation) {
    return NaN;
  }
  const yaw = quaternionYaw(orientation);
  return (yaw * 180) / Math.PI;
}

function quaternionYaw(orientation = {}) {
  const { x = 0, y = 0, z = 0, w = 1 } = orientation;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

function formatPose(pose) {
  const x = Number(pose.position?.x);
  const y = Number(pose.position?.y);
  const yaw = (quaternionYaw(pose.orientation) * 180) / Math.PI;
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    return "--";
  }
  return `${x.toFixed(1)}, ${y.toFixed(1)} m / ${yaw.toFixed(0)} deg`;
}

function uiButtonLabel(event) {
  if (!event) {
    return "--";
  }
  const button = Object.values(BUTTONS).find((entry) => entry.id === Number(event.button_id));
  const duration = ["single", "long", "very long"][Number(event.press_duration)] || "unknown";
  return `${button?.label || `Button ${event.button_id}`} (${duration})`;
}

function temperatureLabel(message, at) {
  if (!message) {
    return "--";
  }
  const value = formatUnit(message.temperature, "C", 1);
  return at ? `${value} / ${ageText(at)} ago` : value;
}

function vescTemperatureLabel(message, at) {
  if (!message) {
    return "--";
  }
  const status = message.state || {};
  const motor = formatUnit(status.temperature_motor, "C", 1);
  const pcb = formatUnit(status.temperature_pcb, "C", 1);
  const age = at ? ` / ${ageText(at)} ago` : "";
  return `Motor ${motor} / PCB ${pcb}${age}`;
}

function boolLabel(value, trueLabel, falseLabel) {
  if (value === true) {
    return trueLabel;
  }
  if (value === false) {
    return falseLabel;
  }
  return "--";
}

function formatUnit(value, unit, digits) {
  const number = Number(value);
  if (!Number.isFinite(number)) {
    return "--";
  }
  return `${number.toFixed(digits)} ${unit}`;
}

function formatCoordinate(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) {
    return "--";
  }
  return number.toFixed(7);
}

function ageText(timestamp) {
  if (!timestamp) {
    return "--";
  }
  const seconds = Math.max(0, Math.round((Date.now() - timestamp) / 1000));
  if (seconds < 60) {
    return `${seconds}s`;
  }
  return `${Math.floor(seconds / 60)}m ${seconds % 60}s`;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

class RosbridgeClient {
  constructor(url) {
    this.url = url;
    this.socket = null;
    this.nextId = 1;
    this.onOpen = () => {};
    this.onClose = () => {};
    this.onError = () => {};
    this.onMessage = () => {};
  }

  connect() {
    this.socket = new WebSocket(this.url);
    this.socket.addEventListener("open", this.onOpen);
    this.socket.addEventListener("close", this.onClose);
    this.socket.addEventListener("error", this.onError);
    this.socket.addEventListener("message", (event) => {
      try {
        this.onMessage(JSON.parse(event.data));
      } catch (error) {
        log(`Could not parse rosbridge message: ${error.message}`);
      }
    });
  }

  close() {
    if (this.socket) {
      this.socket.close();
    }
  }

  send(payload) {
    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return false;
    }
    this.socket.send(JSON.stringify(payload));
    return true;
  }

  subscribe(topic, type, throttleRate = 0, qos = null) {
    const payload = {
      op: "subscribe",
      id: this.id("subscribe"),
      topic,
      type,
      throttle_rate: throttleRate,
    };
    if (qos) {
      payload.qos = qos;
    }
    this.send(payload);
  }

  publish(topic, msg) {
    this.send({
      op: "publish",
      id: this.id("publish"),
      topic,
      msg,
    });
  }

  callService(service, type, args) {
    this.send({
      op: "call_service",
      id: this.id("service"),
      service,
      type,
      args,
    });
  }

  id(prefix) {
    const id = `${prefix}:${this.nextId}`;
    this.nextId += 1;
    return id;
  }
}
