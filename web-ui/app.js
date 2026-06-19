const TOPICS = {
  battery: { topic: "/power", type: "sensor_msgs/msg/BatteryState" },
  charger: { topic: "/power/charger_present", type: "std_msgs/msg/Bool" },
  chargeVoltage: { topic: "/power/charge_voltage", type: "std_msgs/msg/Float32" },
  emergency: { topic: "/hardware/emergency", type: "std_msgs/msg/Bool" },
  rain: { topic: "/hardware/rain", type: "std_msgs/msg/Bool" },
  uiEvent: { topic: "/hardware/ui_event", type: "open_mower_next/msg/UiButtonEvent" },
  gpsFix: { topic: "/gps/fix", type: "sensor_msgs/msg/NavSatFix" },
  gpsOdom: { topic: "/gps/odom", type: "nav_msgs/msg/Odometry" },
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
  hardwareAt: null,
  battery: null,
  gpsFix: null,
  gpsOdom: null,
  charger: null,
  chargeVoltage: null,
  emergency: null,
  rain: null,
  uiEvent: null,
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
  Object.values(TOPICS).forEach(({ topic, type }) => {
    ros.subscribe(topic, type, 500);
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

function renderHardware() {
  els.hardwareAge.textContent = state.hardwareAt ? `Updated ${ageText(state.hardwareAt)} ago` : "Waiting for status topics";
  els.emergencyValue.textContent = boolLabel(state.emergency, "Active", "Clear");
  els.rainValue.textContent = boolLabel(state.rain, "Detected", "Clear");
  els.chargerValue.textContent = boolLabel(state.charger, "Present", "Not present");
  els.buttonValue.textContent = uiButtonLabel(state.uiEvent);

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
  const { x = 0, y = 0, z = 0, w = 1 } = orientation;
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return (yaw * 180) / Math.PI;
}

function uiButtonLabel(event) {
  if (!event) {
    return "--";
  }
  const button = Object.values(BUTTONS).find((entry) => entry.id === Number(event.button_id));
  const duration = ["single", "long", "very long"][Number(event.press_duration)] || "unknown";
  return `${button?.label || `Button ${event.button_id}`} (${duration})`;
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

  subscribe(topic, type, throttleRate = 0) {
    this.send({
      op: "subscribe",
      id: this.id("subscribe"),
      topic,
      type,
      throttle_rate: throttleRate,
    });
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
