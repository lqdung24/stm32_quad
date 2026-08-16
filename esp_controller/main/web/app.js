const MAGIC = 0xA55A, VERSION = 1, CONTROL = 1, STATUS = 2;
const ARM = 1, ESTOP = 2, ACRO_MODE = 1 << 3, FAILSAFE_ACTIVE = 1 << 9;
const ESP_ACK_TIMEOUT_MS = 700, STM_TIMEOUT_MS = 300;
const THROTTLE_MAX = 500, THROTTLE_KEY_STEP = 5, AXIS_MAX = 1000;
const stateNames = ['BOOT', 'DISARMED', 'ARMED', 'FAILSAFE', 'ERROR'];
const $ = id => document.getElementById(id);

let socket = null, session = 0, sequence = 0;
let armRequested = false, emergency = true, deadman = false, requested = 0;
let rollCommand = 0, pitchCommand = 0, yawCommand = 0;
let controlMode = 'test', motorMode = 'all', selectedMotor = 1;
let lastEspAckAt = 0, lastStatusAt = 0, lastStatus = null;
let armTimer = null, reconnectTimer = null, controllerBusy = false;

const fieldNodes = name => document.querySelectorAll(`[data-status="${name}"]`);
function setStatus(name, text, tone = '') {
  fieldNodes(name).forEach(node => {
    node.innerHTML = (name === 'ws' || name === 'stm') ? `<i class="dot"></i>${text}` : text;
    node.className = `status-value ${tone}`;
  });
}
function setCommand(name, value) {
  document.querySelectorAll(`[data-command="${name}"]`).forEach(node => node.textContent = value);
}

async function enterControlMode() { $('launch').hidden = true; }
async function enterFullscreen() {
  try {
    if (!document.fullscreenElement && document.documentElement.requestFullscreen)
      await document.documentElement.requestFullscreen({ navigationUI:'hide' });
  } catch (_) {}
}
$('enter').addEventListener('click', enterControlMode);
$('fullscreen').addEventListener('click', enterFullscreen);

function crc16(data, length) {
  let crc = 0xffff;
  for (let i = 0; i < length; i++) {
    crc ^= data[i] << 8;
    for (let bit = 0; bit < 8; bit++)
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
  }
  return crc;
}

function createControl() {
  const data = new Uint8Array(30), view = new DataView(data.buffer);
  const flags = emergency ? ESTOP : ((armRequested ? ARM : 0) | (controlMode === 'pilot' ? ACRO_MODE : 0));
  const throttleEnabled = controlMode === 'pilot' || deadman;
  const throttle = (!emergency && armRequested && throttleEnabled) ? requested : 0;
  const roll = controlMode === 'pilot' && armRequested ? rollCommand : 0;
  const pitch = controlMode === 'pilot' && armRequested ? pitchCommand : 0;
  const yaw = controlMode === 'pilot' && armRequested ? yawCommand : 0;
  const motorSelection = controlMode === 'test' ? (motorMode === 'all' ? 0 : selectedMotor) : 0;

  view.setUint16(0, MAGIC, true);
  view.setUint8(2, VERSION); view.setUint8(3, CONTROL);
  view.setUint16(4, sequence++ & 0xffff, true);
  view.setUint16(6, session, true);
  view.setUint16(8, flags, true);
  view.setUint8(10, 12); view.setUint8(11, 0);
  view.setUint32(12, Math.floor(performance.now()) >>> 0, true);
  view.setUint16(16, throttle, true);
  view.setInt16(18, roll, true);
  view.setInt16(20, pitch, true);
  view.setInt16(22, yaw, true);
  view.setUint16(24, motorSelection, true);
  view.setUint16(26, 0, true);
  view.setUint16(28, crc16(data, 28), true);
  return data;
}

function sendNow() {
  if (socket?.readyState !== WebSocket.OPEN) return;
  socket.send(createControl());
}

function updateCommandUi() {
  setCommand('throttle', String(requested));
  setCommand('requested', `${requested} / ${THROTTLE_MAX}`);
  setCommand('roll', String(rollCommand));
  setCommand('pitch', String(pitchCommand));
  setCommand('yaw', String(yawCommand));
  $('requested').value = `${requested} / ${THROTTLE_MAX}`;
}

function setKnob(knob, x, y) {
  knob.style.transform = `translate(calc(-50% + ${x}px),calc(-50% + ${y}px))`;
}

function drawStickPositions() {
  const leftRadius = Math.max(0, $('left-stick').clientWidth / 2 - $('left-knob').offsetWidth / 2 - 5);
  const rightRadius = Math.max(0, $('right-stick').clientWidth / 2 - $('right-knob').offsetWidth / 2 - 5);
  const leftY = leftRadius * (1 - 2 * requested / THROTTLE_MAX);
  setKnob($('left-knob'), leftRadius * yawCommand / AXIS_MAX, leftY);
  setKnob($('right-knob'), rightRadius * rollCommand / AXIS_MAX, -rightRadius * pitchCommand / AXIS_MAX);
}

function resetInputs() {
  deadman = false; requested = 0;
  rollCommand = 0; pitchCommand = 0; yawCommand = 0;
  $('slider').value = 0;
  updateCommandUi();
  requestAnimationFrame(drawStickPositions);
}

function setMotorModeLocked(locked) {
  $('mode-all').disabled = locked;
  $('mode-single').disabled = locked;
  $('motor-select').disabled = locked;
}

function updateControlAvailability() {
  $('slider').disabled = !armRequested || controlMode !== 'test';
  ['left-stick','right-stick'].forEach(id => {
    $(id).classList.toggle('locked', !armRequested || controlMode !== 'pilot');
    $(id).setAttribute('aria-disabled', String(!armRequested || controlMode !== 'pilot'));
  });
}

function updateMotorModeUi() {
  $('mode-all-label').classList.toggle('selected', motorMode === 'all');
  $('mode-single-label').classList.toggle('selected', motorMode === 'single');
  $('motor-picker').hidden = motorMode !== 'single';
  $('throttle-label').textContent = motorMode === 'all' ? 'Throttle chung 4 motor' : `Throttle riêng M${selectedMotor}`;
  document.querySelectorAll('.motor').forEach(card => {
    const motor = Number(card.dataset.motor);
    card.classList.toggle('selected', motorMode === 'all' || motor === selectedMotor);
  });
}

function renderControlMode() {
  const pilot = controlMode === 'pilot';
  $('content').className = pilot ? 'pilot' : 'test';
  $('left-stick-panel').hidden = !pilot;
  $('right-stick-panel').hidden = !pilot;
  $('test-panel').hidden = pilot;
  $('test-options').hidden = pilot;
  $('mode-badge').textContent = pilot ? 'JOYSTICK · RATE' : 'TEST MOTOR';
  $('mode-toggle').textContent = pilot ? 'MỞ TEST MOTOR' : 'MỞ JOYSTICK';
  updateControlAvailability();
  requestAnimationFrame(drawStickPositions);
}

function forceLocalSafe(message, useEmergency) {
  armRequested = false;
  emergency = useEmergency;
  resetInputs();
  updateControlAvailability();
  setMotorModeLocked(false);
  document.querySelectorAll('.motor-pwm').forEach(item => item.textContent = '1000 µs');
  if (message) $('message').textContent = message;
}

$('mode-toggle').addEventListener('click', () => {
  const nextMode = controlMode === 'test' ? 'pilot' : 'test';
  const keepEmergency = emergency;
  forceLocalSafe('Đang đổi chế độ; đã DISARM và đưa mọi lệnh về 0.', keepEmergency);
  sendNow();
  controlMode = nextMode;
  renderControlMode();
  sendNow();
  $('message').textContent = nextMode === 'pilot'
    ? 'Đã mở joystick. Throttle bắt đầu ở 0; giữ ARM 1 giây trước khi điều khiển.'
    : 'Đã về chế độ test motor và DISARM toàn bộ motor.';
});

function newSession() {
  const random = new Uint16Array(1);
  crypto.getRandomValues(random);
  session = random[0] || 1; sequence = 0;
  lastStatus = null; lastStatusAt = 0; emergency = false;
  forceLocalSafe('', false);
}

function scheduleReconnect() {
  if (controllerBusy || reconnectTimer !== null) return;
  reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, 1000);
}

function connect() {
  if (socket && (socket.readyState === WebSocket.CONNECTING || socket.readyState === WebSocket.OPEN)) return;
  const currentSocket = new WebSocket(`ws://${location.host}/ws`);
  socket = currentSocket; currentSocket.binaryType = 'arraybuffer';
  setStatus('ws','CONNECTING','warn');

  currentSocket.onopen = () => {
    if (socket !== currentSocket) return;
    lastEspAckAt = performance.now(); newSession();
    setStatus('ws','WAIT ACK','warn');
    $('message').textContent = 'Đã mở WebSocket, đang chờ ESP32 phản hồi…';
    sendNow();
  };
  currentSocket.onmessage = event => {
    if (socket !== currentSocket) return;
    if (typeof event.data === 'string') {
      if (event.data === 'ESP_ALIVE') {
        lastEspAckAt = performance.now(); setStatus('ws','CONNECTED','good'); return;
      }
      if (event.data === 'CONTROLLER_BUSY') {
        controllerBusy = true;
        forceLocalSafe('ESP32 đang được điều khiển bởi thiết bị hoặc tab khác.', true);
        setStatus('ws','BUSY','warn'); currentSocket.close(4001,'Controller busy');
      }
      return;
    }
    decodeStatus(new Uint8Array(event.data));
  };
  currentSocket.onclose = () => {
    if (socket !== currentSocket) return;
    forceLocalSafe('Mất kết nối ESP32; watchdog sẽ dừng motor.', true);
    socket = null;
    if (!controllerBusy) { setStatus('ws','DISCONNECTED','bad'); scheduleReconnect(); }
  };
  currentSocket.onerror = () => currentSocket.close();
}

function decodeStatus(data) {
  if (data.length !== 38) return;
  const view = new DataView(data.buffer,data.byteOffset,data.byteLength);
  if (view.getUint16(0,true) !== MAGIC || view.getUint8(2) !== VERSION ||
      view.getUint8(3) !== STATUS || view.getUint8(10) !== 20 ||
      view.getUint16(36,true) !== crc16(data,36)) return;

  const state = view.getUint8(30), errors = view.getUint16(31,true);
  const applied = view.getUint16(20,true);
  const pulseValues = [22,24,26,28].map(offset => view.getUint16(offset,true));
  lastStatusAt = performance.now(); lastStatus = { state,errors };
  setStatus('state',stateNames[state] || 'INVALID',state === 2 ? 'good' : state >= 3 ? 'bad' : '');
  setStatus('applied',`${applied} / ${THROTTLE_MAX}`);
  setStatus('rate',`${view.getUint16(33,true)} pkt/s`);
  document.querySelectorAll('.motor-pwm').forEach((item,index) => item.textContent = `${pulseValues[index]} µs`);
  const failed = state === 3 || (errors & FAILSAFE_ACTIVE);
  setStatus('failsafe',failed ? 'ACTIVE' : 'CLEAR',failed ? 'bad' : 'good');
  if ((state === 3 || state === 4) && armRequested) {
    forceLocalSafe('STM32 vào FAILSAFE/ERROR; đã khóa điều khiển và gửi e-stop.', true);
    sendNow();
  }
}

document.querySelectorAll('input[name="motor-mode"]').forEach(input => {
  input.addEventListener('change', () => {
    resetInputs(); sendNow(); motorMode = input.value; updateMotorModeUi(); sendNow();
    if (armRequested) $('message').textContent = 'Đã đưa throttle về 0 trước khi đổi đầu ra motor.';
  });
});
$('motor-select').addEventListener('change', () => {
  resetInputs(); sendNow(); selectedMotor = Number($('motor-select').value);
  updateMotorModeUi(); sendNow();
});

$('slider').addEventListener('pointerdown', event => {
  if (!$('slider').disabled) { deadman = true; $('slider').setPointerCapture(event.pointerId); }
});
$('slider').addEventListener('input', () => {
  requested = Number($('slider').value); updateCommandUi();
});
function releaseDeadman() {
  if (!deadman && requested === 0) return;
  resetInputs(); sendNow();
}
['pointerup','pointercancel','lostpointercapture'].forEach(name => $('slider').addEventListener(name,releaseDeadman));

function stickGeometry(zone) {
  const rect = zone.getBoundingClientRect();
  const knob = zone.querySelector('.knob');
  return { rect, radius:Math.max(1,Math.min(rect.width,rect.height) / 2 - knob.offsetWidth / 2 - 5) };
}
function clampUnitVector(x,y) {
  const length = Math.hypot(x,y);
  return length > 1 ? { x:x/length,y:y/length } : { x,y };
}
function applyAxisDeadzone(value,deadzone = .06) {
  const magnitude = Math.abs(value);
  if (magnitude <= deadzone) return 0;
  return Math.sign(value) * (magnitude-deadzone) / (1-deadzone);
}
function pointerVector(event,zone) {
  const { rect,radius } = stickGeometry(zone);
  return clampUnitVector((event.clientX - rect.left - rect.width/2) / radius,
                         (event.clientY - rect.top - rect.height/2) / radius);
}
function updateLeftStick(event) {
  const vector = pointerVector(event,$('left-stick'));
  yawCommand = Math.round(applyAxisDeadzone(vector.x) * AXIS_MAX);
  requested = Math.round((1 - vector.y) * THROTTLE_MAX / 2);
  requested = Math.max(0,Math.min(THROTTLE_MAX,requested));
  updateCommandUi(); drawStickPositions();
}
function updateRightStick(event) {
  const vector = pointerVector(event,$('right-stick'));
  rollCommand = Math.round(applyAxisDeadzone(vector.x) * AXIS_MAX);
  pitchCommand = Math.round(applyAxisDeadzone(-vector.y) * AXIS_MAX);
  updateCommandUi(); drawStickPositions();
}
function startStick(event,side) {
  if (!armRequested || controlMode !== 'pilot') {
    $('message').textContent = 'Joystick đang khóa. Cần kết nối và giữ ARM 1 giây.';
    return;
  }
  const zone = side === 'left' ? $('left-stick') : $('right-stick');
  if (side === 'left') {
    const { rect,radius } = stickGeometry(zone);
    const currentX = rect.left + rect.width/2 + radius * yawCommand/AXIS_MAX;
    const currentY = rect.top + rect.height/2 + radius * (1 - 2*requested/THROTTLE_MAX);
    const allowedDistance = $('left-knob').offsetWidth * .9;
    if (Math.hypot(event.clientX-currentX,event.clientY-currentY) > allowedDistance) {
      $('message').textContent = 'Chạm vào núm cần trái rồi kéo để tránh throttle nhảy đột ngột.';
      return;
    }
  }
  zone.setPointerCapture(event.pointerId);
  zone.dataset.pointer = String(event.pointerId);
  side === 'left' ? updateLeftStick(event) : updateRightStick(event);
}
function moveStick(event,side) {
  const zone = side === 'left' ? $('left-stick') : $('right-stick');
  if (zone.dataset.pointer !== String(event.pointerId)) return;
  side === 'left' ? updateLeftStick(event) : updateRightStick(event);
}
function releaseStick(event,side) {
  const zone = side === 'left' ? $('left-stick') : $('right-stick');
  if (zone.dataset.pointer !== String(event.pointerId)) return;
  delete zone.dataset.pointer;
  if (side === 'left') yawCommand = 0;
  else { rollCommand = 0; pitchCommand = 0; }
  updateCommandUi(); drawStickPositions(); sendNow();
}
[['left-stick','left'],['right-stick','right']].forEach(([id,side]) => {
  const zone = $(id);
  zone.addEventListener('pointerdown',event => startStick(event,side));
  zone.addEventListener('pointermove',event => moveStick(event,side));
  ['pointerup','pointercancel','lostpointercapture'].forEach(name =>
    zone.addEventListener(name,event => releaseStick(event,side)));
});
window.addEventListener('resize',() => requestAnimationFrame(drawStickPositions));

function espConnected() {
  return socket?.readyState === WebSocket.OPEN && performance.now()-lastEspAckAt < ESP_ACK_TIMEOUT_MS;
}
function stmConnected() {
  return lastStatus !== null && performance.now()-lastStatusAt < STM_TIMEOUT_MS;
}
function canArm() {
  return espConnected() && stmConnected() && lastStatus.state === 1 &&
    requested === 0 && rollCommand === 0 && pitchCommand === 0 && yawCommand === 0 && !emergency;
}

function keyboardThrottleEnabled() {
  return controlMode === 'pilot' && armRequested && !emergency &&
    espConnected() && stmConnected() && lastStatus?.state === 2;
}
function isTextEntryTarget(target) {
  return target instanceof Element && target.closest('input, textarea, select, [contenteditable="true"]');
}
window.addEventListener('keydown', event => {
  if ((event.key !== 'ArrowUp' && event.key !== 'ArrowDown') ||
      isTextEntryTarget(event.target) || !keyboardThrottleEnabled()) return;

  event.preventDefault();
  const delta = event.key === 'ArrowUp' ? THROTTLE_KEY_STEP : -THROTTLE_KEY_STEP;
  requested = Math.max(0, Math.min(THROTTLE_MAX, requested + delta));
  updateCommandUi();
  drawStickPositions();
  sendNow();
});

document.querySelectorAll('[data-action="arm"]').forEach(button => {
  button.addEventListener('pointerdown', () => {
    if (!canArm()) {
      $('message').textContent = 'Không thể ARM: cần ESP CONNECTED, STM32 DISARMED và mọi lệnh bằng 0.';
      return;
    }
    $('message').textContent = 'Tiếp tục giữ để ARM…';
    armTimer = setTimeout(() => {
      armRequested = true; setMotorModeLocked(false); updateControlAvailability(); sendNow();
      $('message').textContent = controlMode === 'pilot'
        ? 'Đã ARM. Kéo cần trái từ vị trí thấp nhất; thả tay giữ throttle, yaw tự về giữa.'
        : 'Đã ARM; giữ slider để tăng PWM.';
      armTimer = null;
    },1000);
  });
  ['pointerup','pointerleave','pointercancel'].forEach(name => button.addEventListener(name,() => {
    if (armTimer) { clearTimeout(armTimer); armTimer = null; $('message').textContent = 'Đã hủy ARM.'; }
  }));
});
document.querySelectorAll('[data-action="disarm"]').forEach(button => button.addEventListener('click',() => {
  forceLocalSafe('Đã DISARM toàn bộ motor.',false); sendNow();
}));
document.querySelectorAll('[data-action="stop"]').forEach(button => button.addEventListener('click',() => {
  forceLocalSafe('EMERGENCY STOP đã gửi.',true); sendNow();
}));

window.addEventListener('pagehide',() => {
  emergency = true; armRequested = false; resetInputs(); sendNow();
});
document.addEventListener('visibilitychange',() => {
  if (document.hidden && armRequested) {
    emergency = true; armRequested = false; resetInputs(); updateControlAvailability(); sendNow();
  }
});

setInterval(sendNow,25);
setInterval(() => {
  const now = performance.now(), espOnline = espConnected(), stmOnline = stmConnected();
  setStatus('stm',stmOnline ? 'ONLINE' : 'OFFLINE',stmOnline ? 'good' : 'bad');
  if (!stmOnline && armRequested) {
    forceLocalSafe('Mất status STM32 khi đang ARM; đã khóa điều khiển và gửi e-stop.',true); sendNow();
  }
  if (socket?.readyState === WebSocket.OPEN && now-lastEspAckAt >= ESP_ACK_TIMEOUT_MS) {
    const staleSocket = socket;
    forceLocalSafe('ESP32 không phản hồi; đóng kết nối và chờ watchdog dừng motor.',true);
    setStatus('ws','NO RESPONSE','bad'); staleSocket.close(4000,'ESP heartbeat timeout');
  } else if (espOnline) setStatus('ws','CONNECTED','good');
},100);

updateCommandUi(); updateMotorModeUi(); renderControlMode(); connect();
