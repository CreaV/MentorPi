/* eslint-env browser */
/* global ROSLIB, nipplejs */

// ---- config ----
const ROSBRIDGE_PORT = 9090;
const VIDEO_PORT = 8081;
const VIDEO_TOPIC = '/camera/color/image_raw';
const TWIST_TOPIC = '/cmd_vel';
const STATUS_TOPIC = '/mode/status';
const SET_MODE_SRV = '/mode/set';
const LIST_MAPS_SRV = '/mode/list_maps';

const MAX_LIN = 0.4;   // m/s
const MAX_ANG = 1.5;   // rad/s
const PUB_HZ = 20;
const DEADMAN_MS = 300;

// ---- state ----
const host = window.location.hostname || '127.0.0.1';
const wsUrl = `ws://${host}:${ROSBRIDGE_PORT}`;
let ros = null;
let twistPub = null;
let lastStickInputAt = 0;
let stick = { lx: 0, ly: 0, az: 0 };
let currentMode = 'unknown';

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));

// ---- toast ----
let toastTimer = null;
function toast(msg, isError = false) {
  const el = $('#toast');
  el.textContent = msg;
  el.classList.toggle('error', isError);
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 2500);
}

// ---- connection status ----
function setConn(state, label) {
  const el = $('#conn-pill');
  el.dataset.state = state;
  el.textContent = label || state;
}

// ---- video ----
function startVideoStream() {
  const url = `http://${host}:${VIDEO_PORT}/stream?topic=${encodeURIComponent(VIDEO_TOPIC)}&type=mjpeg`;
  const img = $('#cam');
  img.onload = () => $('#video').classList.add('has-signal');
  img.onerror = () => {
    $('#video').classList.remove('has-signal');
    // back off + retry
    setTimeout(() => { img.src = url + '&t=' + Date.now(); }, 1500);
  };
  img.src = url;
}

// ---- ros connect / reconnect ----
function connectRos() {
  setConn('connecting', 'connecting');
  ros = new ROSLIB.Ros({ url: wsUrl });

  ros.on('connection', () => {
    setConn('ok', 'connected');
    setupTopicsAndServices();
  });
  ros.on('error', (e) => {
    console.error('ros error', e);
    setConn('error', 'ws error');
  });
  ros.on('close', () => {
    setConn('error', 'disconnected');
    setTimeout(connectRos, 2000);
  });
}

function setupTopicsAndServices() {
  twistPub = new ROSLIB.Topic({
    ros, name: TWIST_TOPIC, messageType: 'geometry_msgs/msg/Twist',
    queue_size: 1, throttle_rate: 0,
  });

  const status = new ROSLIB.Topic({
    ros, name: STATUS_TOPIC, messageType: 'std_msgs/msg/String',
  });
  status.subscribe((msg) => {
    currentMode = msg.data;
    $('#mode-pill').dataset.mode = currentMode;
    $('#mode-pill').textContent = labelOf(currentMode);
    $$('.mode-btn').forEach((b) =>
      b.classList.toggle('active', b.dataset.mode === currentMode));
  });
}

function labelOf(mode) {
  return ({
    idle: 'IDLE',
    slam_2d: '2D SLAM',
    slam_3d: '3D SLAM',
    loc_2d: 'LOC 2D',
    loc_3d: 'LOC 3D',
  })[mode] || mode || '…';
}

// ---- twist publishing loop ----
setInterval(() => {
  if (!twistPub) return;
  const idle = (Date.now() - lastStickInputAt) > DEADMAN_MS;
  const lx = idle ? 0 : stick.lx;
  const ly = idle ? 0 : stick.ly;
  const az = idle ? 0 : stick.az;
  const msg = new ROSLIB.Message({
    linear:  { x: lx, y: ly, z: 0 },
    angular: { x: 0,  y: 0,  z: az },
  });
  twistPub.publish(msg);
}, 1000 / PUB_HZ);

// ---- joysticks ----
function setupSticks() {
  const left = nipplejs.create({
    zone: $('#stick-left'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#4ea1ff',
    size: 140,
    threshold: 0.05,
  });
  const right = nipplejs.create({
    zone: $('#stick-right'),
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: '#4ea1ff',
    size: 140,
    threshold: 0.05,
  });

  // left stick: linear motion. angle 0=right, 90=up.
  // forward (up)  -> linear.x = +
  // strafe right  -> linear.y = - (ROS REP-103: y is left-positive)
  left.on('move', (_, data) => {
    const f = clamp(data.distance / 70, 0, 1); // 70 = half size
    const ang = (data.angle?.radian ?? 0);
    stick.lx = +(Math.sin(ang) * f * MAX_LIN).toFixed(3); // sin: up=+1
    stick.ly = +(Math.cos(ang) * f * MAX_LIN * -1).toFixed(3); // cos: right=+1, flip to ROS y
    lastStickInputAt = Date.now();
  });
  left.on('end', () => { stick.lx = 0; stick.ly = 0; });

  // right stick: rotation. only horizontal axis used.
  right.on('move', (_, data) => {
    const f = clamp(data.distance / 70, 0, 1);
    const ang = (data.angle?.radian ?? 0);
    stick.az = +(Math.cos(ang) * f * MAX_ANG * -1).toFixed(3); // right tilt -> negative yaw (CW)
    lastStickInputAt = Date.now();
  });
  right.on('end', () => { stick.az = 0; });
}

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

// ---- mode switching ----
function callSetMode(mode, mapFile = '', databasePath = '') {
  if (!ros) { toast('not connected', true); return; }
  const srv = new ROSLIB.Service({
    ros, name: SET_MODE_SRV, serviceType: 'mentorpi_msgs/srv/SetMode',
  });
  const req = new ROSLIB.ServiceRequest({
    mode, map_file: mapFile, database_path: databasePath,
  });
  $$('.mode-btn').forEach((b) => b.disabled = true);
  toast(`switching to ${mode}…`);
  srv.callService(req, (res) => {
    $$('.mode-btn').forEach((b) => b.disabled = false);
    toast(res.message || (res.success ? 'ok' : 'failed'), !res.success);
  }, (err) => {
    $$('.mode-btn').forEach((b) => b.disabled = false);
    toast('service error: ' + err, true);
  });
}

function setupModeButtons() {
  $$('.mode-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      const m = btn.dataset.mode;
      if (m === 'loc_2d' || m === 'loc_3d') openMapSheet(m);
      else callSetMode(m);
    });
  });
}

// ---- map sheet ----
let sheetMode = 'loc_2d'; // which localization mode the sheet is picking for
function openMapSheet(mode) {
  sheetMode = mode;
  $('#map-sheet-title').textContent = mode === 'loc_3d' ? 'Choose 3D map (.db)' : 'Choose 2D map';
  $('#map-sheet').hidden = false;
  refreshMaps();
}
function closeMapSheet() {
  $('#map-sheet').hidden = true;
}
function refreshMaps() {
  if (!ros) { renderMaps([]); return; }
  const list = $('#map-list');
  list.innerHTML = '<li class="empty">loading…</li>';
  const srv = new ROSLIB.Service({
    ros, name: LIST_MAPS_SRV, serviceType: 'mentorpi_msgs/srv/ListMaps',
  });
  srv.callService(new ROSLIB.ServiceRequest({ mode: sheetMode }), (res) => {
    renderMaps(res.maps || []);
  }, (err) => {
    list.innerHTML = `<li class="empty">error: ${err}</li>`;
  });
}
function renderMaps(maps) {
  const list = $('#map-list');
  if (!maps.length) {
    list.innerHTML = sheetMode === 'loc_3d'
      ? '<li class="empty">no .db in ~/rtabmap_maps/ — build one with 3D SLAM first</li>'
      : '<li class="empty">no maps in ~/maps/ — build one with 2D SLAM first</li>';
    return;
  }
  list.innerHTML = '';
  maps.forEach((m) => {
    const li = document.createElement('li');
    li.textContent = m;
    li.addEventListener('click', () => {
      closeMapSheet();
      // loc_2d takes map_file, loc_3d takes database_path
      if (sheetMode === 'loc_3d') callSetMode('loc_3d', '', m);
      else callSetMode('loc_2d', m);
    });
    list.appendChild(li);
  });
}

// ---- boot ----
document.addEventListener('DOMContentLoaded', () => {
  startVideoStream();
  setupSticks();
  setupModeButtons();
  $('#map-close').addEventListener('click', closeMapSheet);
  $('#map-refresh').addEventListener('click', refreshMaps);
  $('#map-sheet').addEventListener('click', (e) => {
    if (e.target.id === 'map-sheet') closeMapSheet();
  });
  // prevent double-tap zoom on iOS
  document.addEventListener('gesturestart', (e) => e.preventDefault());
  connectRos();
});
