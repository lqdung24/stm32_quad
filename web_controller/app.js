const MAGIC = 0xA55A, VERSION = 1, CONTROL = 1, STATUS = 2;
const ARM = 1, ESTOP = 2, ACRO_MODE = 1 << 3, FAILSAFE_ACTIVE = 1 << 9;
const UART_LINK_LOST = 1 << 1;
const STM_TIMEOUT_MS = 300;
const MAX_ACK_LAG_PACKETS = 16;
const THROTTLE_MAX = 500, THROTTLE_KEY_STEP = 5, AXIS_MAX = 1000;
const stateNames = ['BOOT', 'DISARMED', 'ARMED', 'FAILSAFE', 'ERROR'];
const $ = id => document.getElementById(id);

let port = null, writer = null, session = 0, sequence = 0;
let lastControlSequence = null, lastAcknowledgedAt = 0;
let armRequested = false, emergency = true, deadman = false, requested = 0;
let rollCommand = 0, pitchCommand = 0, yawCommand = 0;
let controlMode = 'test', motorMode = 'all', selectedMotor = 1;
let lastStatusAt = 0, lastStatus = null;
let armTimer = null, serialReaderActive = false, cobsBuffer = [];

const fieldNodes = name => document.querySelectorAll(`[data-status="${name}"]`);
function setStatus(name, text, tone = '') {
  fieldNodes(name).forEach(node => {
    node.innerHTML = (name === 'ws' || name === 'esp' || name === 'stm') ? `<i class="dot"></i>${text}` : text;
    node.className = `status-value ${tone}`;
  });
}
function setCommand(name, value) {
  document.querySelectorAll(`[data-command="${name}"]`).forEach(node => node.textContent = value);
}

async function enterControlMode() {
  $('launch').hidden = true;
  await connect();
}
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
  lastControlSequence = sequence++ & 0xffff;
  view.setUint16(4, lastControlSequence, true);
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
  if (!writer) return;
  writer.write(cobsEncode(createControl())).catch(serialDisconnected);
}

function cobsEncode(data) {
  const output = [0, 0];
  let codeIndex = 1, code = 1;
  for (const byte of data) {
    if (byte === 0) {
      output[codeIndex] = code; code = 1; codeIndex = output.length; output.push(0);
    } else {
      output.push(byte); code++;
      if (code === 0xff) { output[codeIndex] = code; code = 1; codeIndex = output.length; output.push(0); }
    }
  }
  output[codeIndex] = code;
  output.push(0);
  return new Uint8Array(output);
}

function cobsDecode(encoded) {
  const output = [];
  for (let i = 0; i < encoded.length;) {
    const code = encoded[i++];
    if (!code || i + code - 1 > encoded.length) return null;
    for (let count = 1; count < code; count++) output.push(encoded[i++]);
    if (code !== 0xff && i < encoded.length) output.push(0);
  }
  return new Uint8Array(output);
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
  lastStatus = null; lastStatusAt = 0; lastControlSequence = null; lastAcknowledgedAt = 0; emergency = false;
  forceLocalSafe('', false);
}

async function connect() {
  if (writer) return;
  if (!('serial' in navigator)) {
    $('message').textContent = 'Trình duyệt này không hỗ trợ Web Serial; hãy dùng Chrome hoặc Edge trên laptop.';
    return;
  }
  try {
    setStatus('ws','SELECT PORT','warn');
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    writer = port.writable.getWriter();
    newSession();
    setStatus('ws','CONNECTED','good');
    $('message').textContent = 'Đã kết nối USB. Đang chờ status từ drone qua ESP-NOW…';
    serialReaderActive = true;
    readSerial();
    sendNow();
  } catch (err) {
    console.error('Web Serial connect error:', err);
    $('message').textContent =
        `${err.name}: ${err.message}`;
    serialDisconnected();
  }
}

async function readSerial() {
  const reader = port.readable.getReader();
  try {
    while (serialReaderActive) {
      const { value, done } = await reader.read();
      if (done) break;
      for (const byte of value) {
        if (byte === 0) {
          const packet = cobsDecode(cobsBuffer);
          cobsBuffer = [];
          if (packet) decodeStatus(packet);
        } else if (cobsBuffer.length <= 51) cobsBuffer.push(byte);
        else cobsBuffer = [];
      }
    }
  } catch (_) {
    // Physical USB removal reaches this path.
  } finally {
    reader.releaseLock();
    serialDisconnected();
  }
}

function serialDisconnected() {
  if (!port && !writer) return;
  serialReaderActive = false;
  if (writer) { writer.releaseLock(); writer = null; }
  port = null; cobsBuffer = [];
  forceLocalSafe('Mất kết nối USB ESP32; STM32 sẽ failsafe khi hết lệnh điều khiển.', true);
  setStatus('ws','DISCONNECTED','bad');
}

function decodeStatus(data) {
  if (data.length !== 38) return;
  const view = new DataView(data.buffer,data.byteOffset,data.byteLength);
  if (view.getUint16(0,true) !== MAGIC || view.getUint8(2) !== VERSION ||
      view.getUint8(3) !== STATUS || view.getUint8(10) !== 20 ||
      view.getUint16(36,true) !== crc16(data,36)) return;

  const statusSession = view.getUint16(6,true), acknowledgedSequence = view.getUint16(16,true);
  const state = view.getUint8(30), errors = view.getUint16(31,true);
  const applied = view.getUint16(20,true);
  const pulseValues = [22,24,26,28].map(offset => view.getUint16(offset,true));
  lastStatusAt = performance.now();
  if (statusSession === session && lastControlSequence !== null) {
    const acknowledgementLag = (lastControlSequence - acknowledgedSequence) & 0xffff;
    if (acknowledgementLag <= MAX_ACK_LAG_PACKETS) lastAcknowledgedAt = lastStatusAt;
  }
  lastStatus = { state,errors };
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
  return writer !== null && lastAcknowledgedAt !== 0 &&
    performance.now()-lastAcknowledgedAt < STM_TIMEOUT_MS;
}
function stmConnected() {
  return espConnected() && lastStatus !== null &&
    performance.now()-lastStatusAt < STM_TIMEOUT_MS &&
    (lastStatus.errors & UART_LINK_LOST) === 0;
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
  const now = performance.now(), usbOnline = writer !== null;
  const espOnline = espConnected(), stmOnline = stmConnected();
  setStatus('esp',espOnline ? 'ONLINE' : 'OFFLINE',espOnline ? 'good' : 'bad');
  setStatus('stm',stmOnline ? 'ONLINE' : 'OFFLINE',stmOnline ? 'good' : 'bad');
  if (!espOnline && armRequested) {
    forceLocalSafe('Mất link ESP-NOW khi đang ARM; đã khóa điều khiển và chờ STM32 failsafe.',true); sendNow();
  } else if (!stmOnline && armRequested) {
    forceLocalSafe('Mất status STM32 khi đang ARM; đã khóa điều khiển và gửi e-stop.',true); sendNow();
  }
  if (usbOnline) setStatus('ws','CONNECTED','good');
},100);

updateCommandUi(); updateMotorModeUi(); renderControlMode();
