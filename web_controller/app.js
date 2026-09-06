const MAGIC = 0xA55A, VERSION = 1, CONTROL = 1, STATUS = 2, FLIGHT_TELEMETRY = 7;
const ARM = 1, ESTOP = 2, ACRO_MODE = 1 << 3, FAILSAFE_ACTIVE = 1 << 9;
const TELEMETRY_SIZE = 50, TELEMETRY_PAYLOAD_SIZE = 32;
const TELEMETRY_STATE_MASK = 0x0007, TELEMETRY_ACTUATORS_ACTIVE = 1 << 3;
const TELEMETRY_ATTITUDE_VALID = 1 << 4, TELEMETRY_ALLOWED_FLAGS = 0x001f;
const TELEMETRY_HISTORY_MS = 60000, MAX_TELEMETRY_SAMPLES = 4000;
const MAX_RECORDED_SAMPLES = 180000;
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
let telemetryHistory = [], recordedTelemetry = [], telemetryArrivalTimes = [];
let telemetryPacketCount = 0, telemetryInvalidCount = 0, telemetryDropCount = 0;
let lastTelemetrySequence = null, lastTelemetrySession = null, telemetryRecording = false;

const telemetryColumns = [
  'host_time_iso','host_time_ms','stm_time_ms','sequence','session_id','state','state_name',
  'actuators_active','attitude_valid','roll_deg','pitch_deg','yaw_deg',
  'gyro_roll_rad_s','gyro_pitch_rad_s','gyro_yaw_rad_s',
  'setpoint_roll_rad_s','setpoint_pitch_rad_s','setpoint_yaw_rad_s',
  'error_roll_rad_s','error_pitch_rad_s','error_yaw_rad_s',
  'pid_roll','pid_pitch','pid_yaw','motor1_us','motor2_us','motor3_us','motor4_us'
];

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
  lastTelemetrySequence = null; lastTelemetrySession = null;
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
          if (packet) decodePacket(packet);
        } else if (cobsBuffer.length < 64) cobsBuffer.push(byte);
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

function decodePacket(data) {
  if (data.length < 4) return;
  const view = new DataView(data.buffer,data.byteOffset,data.byteLength);
  if (view.getUint16(0,true) !== MAGIC || view.getUint8(2) !== VERSION) return;
  const type = view.getUint8(3);
  if (type === STATUS) decodeStatus(data);
  else if (type === FLIGHT_TELEMETRY) decodeFlightTelemetry(data);
}

function decodeStatus(data) {
  if (data.length !== 38) return;
  const view = new DataView(data.buffer,data.byteOffset,data.byteLength);
  if (view.getUint16(0,true) !== MAGIC || view.getUint8(2) !== VERSION ||
      view.getUint8(3) !== STATUS || view.getUint8(10) !== 20 ||
      view.getUint8(11) !== 0 ||
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

function decodeFlightTelemetry(data) {
  const reject = () => {
    telemetryInvalidCount++;
    updateTelemetryStats();
    return false;
  };
  if (data.length !== TELEMETRY_SIZE) return reject();

  const view = new DataView(data.buffer,data.byteOffset,data.byteLength);
  const flags = view.getUint16(8,true);
  const state = flags & TELEMETRY_STATE_MASK;
  if (view.getUint16(0,true) !== MAGIC || view.getUint8(2) !== VERSION ||
      view.getUint8(3) !== FLIGHT_TELEMETRY ||
      view.getUint8(10) !== TELEMETRY_PAYLOAD_SIZE || view.getUint8(11) !== 0 ||
      (flags & ~TELEMETRY_ALLOWED_FLAGS) !== 0 || state >= stateNames.length ||
      view.getUint16(48,true) !== crc16(data,48)) return reject();

  const motorPwmUs = [40,42,44,46].map(offset => view.getUint16(offset,true));
  if (motorPwmUs.some(pulse => pulse < 1000 || pulse > 2000)) return reject();

  const receivedAtMs = performance.now();
  const wallTimeMs = Date.now();
  const sequenceValue = view.getUint16(4,true);
  const sessionId = view.getUint16(6,true);
  const attitudeDeg = [16,18,20].map(offset => view.getInt16(offset,true) / 100);
  const gyroRadS = [22,24,26].map(offset => view.getInt16(offset,true) / 1000);
  const setpointRadS = [28,30,32].map(offset => view.getInt16(offset,true) / 1000);
  const pidOutput = [34,36,38].map(offset => view.getInt16(offset,true) / 100);
  const sample = {
    receivedAtMs, wallTimeMs, stmTimeMs:view.getUint32(12,true),
    sequence:sequenceValue, sessionId, state,
    actuatorsActive:(flags & TELEMETRY_ACTUATORS_ACTIVE) !== 0,
    attitudeValid:(flags & TELEMETRY_ATTITUDE_VALID) !== 0,
    attitudeDeg, gyroRadS, setpointRadS,
    rateErrorRadS:setpointRadS.map((target,index) => target - gyroRadS[index]),
    pidOutput, motorPwmUs
  };

  if (lastTelemetrySession === sessionId && lastTelemetrySequence !== null) {
    const delta = (sequenceValue - lastTelemetrySequence) & 0xffff;
    if (delta > 1 && delta < 0x8000) telemetryDropCount += delta - 1;
  }
  lastTelemetrySession = sessionId;
  lastTelemetrySequence = sequenceValue;
  telemetryPacketCount++;
  telemetryArrivalTimes.push(receivedAtMs);
  telemetryHistory.push(sample);
  while (telemetryArrivalTimes.length && telemetryArrivalTimes[0] < receivedAtMs - 1000)
    telemetryArrivalTimes.shift();
  while (telemetryHistory.length > MAX_TELEMETRY_SAMPLES ||
         (telemetryHistory.length && telemetryHistory[0].receivedAtMs < receivedAtMs - TELEMETRY_HISTORY_MS))
    telemetryHistory.shift();

  if (telemetryRecording) {
    if (recordedTelemetry.length < MAX_RECORDED_SAMPLES) recordedTelemetry.push(sample);
    else {
      telemetryRecording = false;
      $('record-telemetry').classList.remove('recording');
      $('record-telemetry').textContent = 'BẮT ĐẦU GHI';
      $('message').textContent = 'Đã đạt giới hạn 180.000 mẫu; dừng ghi để bảo vệ bộ nhớ trình duyệt.';
    }
  }
  updateTelemetryStats();
  return true;
}

function updateTelemetryStats() {
  const latest = telemetryHistory[telemetryHistory.length - 1];
  const now = performance.now();
  while (telemetryArrivalTimes.length && telemetryArrivalTimes[0] < now - 1000)
    telemetryArrivalTimes.shift();
  $('telemetry-count').textContent = String(telemetryPacketCount);
  $('telemetry-rate').textContent = `${telemetryArrivalTimes.length} Hz`;
  $('telemetry-drops').textContent = String(telemetryDropCount);
  $('telemetry-invalid').textContent = String(telemetryInvalidCount);
  $('telemetry-recorded').textContent = String(recordedTelemetry.length);
  $('save-telemetry-csv').disabled = recordedTelemetry.length === 0;
  $('save-telemetry-txt').disabled = recordedTelemetry.length === 0;
  $('telemetry-summary').textContent = latest
    ? `${stateNames[latest.state]} · ${telemetryArrivalTimes.length} Hz · seq ${latest.sequence}`
    : 'Chưa có dữ liệu';
}

function telemetrySeries() {
  const mode = $('plot-mode').value;
  const axis = Number($('plot-axis').value);
  const axisNames = ['Roll','Pitch','Yaw'];
  if (mode === 'tuning') return [
    {
      label:`${axisNames[axis]} rate (rad/s)`,
      series:[
        { label:'Setpoint', color:'#ffc96b', value:sample => sample.setpointRadS[axis] },
        { label:'Gyro', color:'#45b8ff', value:sample => sample.gyroRadS[axis] },
        { label:'Error', color:'#4ee0a0', value:sample => sample.rateErrorRadS[axis] }
      ]
    },
    {
      label:`${axisNames[axis]} PID tổng`,
      series:[{ label:'PID output', color:'#ff7bd5', value:sample => sample.pidOutput[axis] }]
    }
  ];
  if (mode === 'attitude') return [{
    label:'Attitude (deg)',
    series:[
      { label:'Roll', color:'#45b8ff', value:sample => sample.attitudeDeg[0] },
      { label:'Pitch', color:'#ffc96b', value:sample => sample.attitudeDeg[1] },
      { label:'Yaw', color:'#ff7bd5', value:sample => sample.attitudeDeg[2] }
    ]
  }];
  return [{
    label:'Motor PWM (µs)',
    series:[
      { label:'M1 FL', color:'#45b8ff', value:sample => sample.motorPwmUs[0] },
      { label:'M2 RL', color:'#ffc96b', value:sample => sample.motorPwmUs[1] },
      { label:'M3 FR', color:'#4ee0a0', value:sample => sample.motorPwmUs[2] },
      { label:'M4 RR', color:'#ff7bd5', value:sample => sample.motorPwmUs[3] }
    ]
  }];
}

function renderTelemetryLegend(panels) {
  const unique = new Map();
  panels.forEach(panel => panel.series.forEach(series => unique.set(series.label,series.color)));
  $('telemetry-legend').replaceChildren(...[...unique].map(([label,color]) => {
    const item = document.createElement('span');
    item.textContent = label;
    item.style.setProperty('--series-color',color);
    return item;
  }));
}

function drawTelemetryPlot() {
  if (!$('telemetry-panel').open) return;
  const canvas = $('telemetry-chart');
  const width = Math.max(320,Math.floor(canvas.clientWidth));
  const height = Math.max(220,Math.floor(canvas.clientHeight));
  const pixelRatio = Math.max(1,window.devicePixelRatio || 1);
  if (canvas.width !== Math.floor(width * pixelRatio) || canvas.height !== Math.floor(height * pixelRatio)) {
    canvas.width = Math.floor(width * pixelRatio);
    canvas.height = Math.floor(height * pixelRatio);
  }
  const context = canvas.getContext('2d');
  context.setTransform(pixelRatio,0,0,pixelRatio,0,0);
  context.clearRect(0,0,width,height);
  context.fillStyle = '#08111a'; context.fillRect(0,0,width,height);

  const windowMs = Number($('plot-window').value) * 1000;
  const newestTime = telemetryHistory.length
    ? telemetryHistory[telemetryHistory.length - 1].receivedAtMs
    : performance.now();
  const samples = telemetryHistory.filter(sample => sample.receivedAtMs >= newestTime - windowMs);
  const panels = telemetrySeries();
  renderTelemetryLegend(panels);
  if (!samples.length) {
    context.fillStyle = '#91a5b8'; context.font = '14px system-ui';
    context.textAlign = 'center'; context.fillText('Đang chờ FLIGHT_TELEMETRY từ STM32…',width/2,height/2);
    return;
  }

  const left = 58, right = 12, top = 16, bottom = 24, gap = panels.length > 1 ? 25 : 0;
  const panelHeight = (height - top - bottom - gap) / panels.length;
  const plotWidth = width - left - right;
  const xFor = sample => left + ((sample.receivedAtMs - (newestTime - windowMs)) / windowMs) * plotWidth;

  panels.forEach((panel,panelIndex) => {
    const panelTop = top + panelIndex * (panelHeight + gap);
    const values = panel.series.flatMap(series => samples.map(series.value)).filter(Number.isFinite);
    let minimum = Math.min(0,...values), maximum = Math.max(0,...values);
    if (maximum - minimum < 1e-6) { minimum -= 1; maximum += 1; }
    const padding = (maximum - minimum) * .08;
    minimum -= padding; maximum += padding;
    const yFor = value => panelTop + panelHeight - ((value - minimum) / (maximum - minimum)) * panelHeight;

    context.strokeStyle = '#26394b'; context.lineWidth = 1;
    context.fillStyle = '#91a5b8'; context.font = '10px system-ui'; context.textAlign = 'right';
    for (let line = 0; line <= 4; line++) {
      const y = panelTop + panelHeight * line / 4;
      const value = maximum - (maximum - minimum) * line / 4;
      context.beginPath(); context.moveTo(left,y); context.lineTo(width-right,y); context.stroke();
      context.fillText(value.toFixed(Math.abs(value) >= 100 ? 0 : 2),left-5,y+3);
    }
    context.fillStyle = '#bde7ff'; context.textAlign = 'left'; context.fillText(panel.label,left,panelTop-5);

    panel.series.forEach(series => {
      context.strokeStyle = series.color; context.lineWidth = 1.6; context.beginPath();
      let started = false;
      samples.forEach(sample => {
        const value = series.value(sample);
        if (!Number.isFinite(value)) return;
        const x = xFor(sample), y = yFor(value);
        if (!started) { context.moveTo(x,y); started = true; } else context.lineTo(x,y);
      });
      context.stroke();
    });
  });

  context.fillStyle = '#91a5b8'; context.font = '10px system-ui'; context.textAlign = 'center';
  context.fillText(`-${Math.round(windowMs/1000)} s`,left,height-7);
  context.fillText('now',width-right,height-7);
}

function telemetryRow(sample) {
  return {
    host_time_iso:new Date(sample.wallTimeMs).toISOString(), host_time_ms:sample.wallTimeMs,
    stm_time_ms:sample.stmTimeMs, sequence:sample.sequence, session_id:sample.sessionId,
    state:sample.state, state_name:stateNames[sample.state],
    actuators_active:sample.actuatorsActive, attitude_valid:sample.attitudeValid,
    roll_deg:sample.attitudeDeg[0], pitch_deg:sample.attitudeDeg[1], yaw_deg:sample.attitudeDeg[2],
    gyro_roll_rad_s:sample.gyroRadS[0], gyro_pitch_rad_s:sample.gyroRadS[1], gyro_yaw_rad_s:sample.gyroRadS[2],
    setpoint_roll_rad_s:sample.setpointRadS[0], setpoint_pitch_rad_s:sample.setpointRadS[1], setpoint_yaw_rad_s:sample.setpointRadS[2],
    error_roll_rad_s:sample.rateErrorRadS[0], error_pitch_rad_s:sample.rateErrorRadS[1], error_yaw_rad_s:sample.rateErrorRadS[2],
    pid_roll:sample.pidOutput[0], pid_pitch:sample.pidOutput[1], pid_yaw:sample.pidOutput[2],
    motor1_us:sample.motorPwmUs[0], motor2_us:sample.motorPwmUs[1], motor3_us:sample.motorPwmUs[2], motor4_us:sample.motorPwmUs[3]
  };
}

function csvValue(value) {
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"','""')}"` : text;
}

function downloadTelemetry(format) {
  if (!recordedTelemetry.length) return;
  const rows = recordedTelemetry.map(telemetryRow);
  const separator = format === 'csv' ? ',' : '\t';
  const body = [telemetryColumns.join(separator),...rows.map(row =>
    telemetryColumns.map(column => format === 'csv' ? csvValue(row[column]) : String(row[column])).join(separator)
  )].join('\n');
  const prefix = format === 'txt'
    ? '# Drone flight telemetry; tab-separated; PID fields are combined controller outputs.\n'
    : '';
  const blob = new Blob([prefix,body,'\n'],{type:format === 'csv' ? 'text/csv;charset=utf-8' : 'text/plain;charset=utf-8'});
  const link = document.createElement('a');
  const stamp = new Date().toISOString().replace(/[:.]/g,'-');
  link.href = URL.createObjectURL(blob);
  link.download = `drone-telemetry-${stamp}.${format}`;
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href),0);
}

function toggleTelemetryRecording() {
  telemetryRecording = !telemetryRecording;
  const button = $('record-telemetry');
  button.classList.toggle('recording',telemetryRecording);
  button.textContent = telemetryRecording ? 'DỪNG GHI' : 'BẮT ĐẦU GHI';
  $('message').textContent = telemetryRecording
    ? 'Đang ghi FLIGHT_TELEMETRY vào bộ nhớ trình duyệt; nhấn DỪNG GHI rồi tải CSV/TXT.'
    : `Đã dừng ghi ở ${recordedTelemetry.length} mẫu; có thể tải CSV hoặc TXT.`;
}

function clearTelemetryData() {
  telemetryRecording = false;
  telemetryHistory = []; recordedTelemetry = []; telemetryArrivalTimes = [];
  telemetryPacketCount = 0; telemetryInvalidCount = 0; telemetryDropCount = 0;
  lastTelemetrySequence = null; lastTelemetrySession = null;
  $('record-telemetry').classList.remove('recording');
  $('record-telemetry').textContent = 'BẮT ĐẦU GHI';
  updateTelemetryStats(); drawTelemetryPlot();
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

$('telemetry-panel').addEventListener('toggle',drawTelemetryPlot);
$('plot-mode').addEventListener('change',() => {
  $('plot-axis').disabled = $('plot-mode').value !== 'tuning';
  $('plot-axis-label').classList.toggle('disabled',$('plot-mode').value !== 'tuning');
  drawTelemetryPlot();
});
$('plot-axis').addEventListener('change',drawTelemetryPlot);
$('plot-window').addEventListener('change',drawTelemetryPlot);
$('record-telemetry').addEventListener('click',toggleTelemetryRecording);
$('clear-telemetry').addEventListener('click',clearTelemetryData);
$('save-telemetry-csv').addEventListener('click',() => downloadTelemetry('csv'));
$('save-telemetry-txt').addEventListener('click',() => downloadTelemetry('txt'));
window.addEventListener('resize',drawTelemetryPlot);

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
  updateTelemetryStats();
  drawTelemetryPlot();
},100);

updateCommandUi(); updateMotorModeUi(); renderControlMode(); updateTelemetryStats();
