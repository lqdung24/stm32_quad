This file is a merged representation of the entire codebase, combined into a single document by Repomix.
The content has been processed where content has been compressed (code blocks are separated by ⋮---- delimiter).

# File Summary

## Purpose

This file contains a packed representation of the entire repository's contents.
It is designed to be easily consumable by AI systems for analysis, code review,
or other automated processes.

## File Format

The content is organized as follows:

1. This summary section
2. Repository information
3. Directory structure
4. Repository files (if enabled)
5. Multiple file entries, each consisting of:
   a. A header with the file path (## File: path/to/file)
   b. The full contents of the file in a code block

## Usage Guidelines

- This file should be treated as read-only. Any changes should be made to the
  original repository files, not this packed version.
- When processing this file, use the file path to distinguish
  between different files in the repository.
- Be aware that this file may contain sensitive information. Handle it with
  the same level of security as you would the original repository.

## Notes

- Some files may have been excluded based on .gitignore rules and Repomix's configuration
- Binary files are not included in this packed representation. Please refer to the Repository Structure section for a complete list of file paths, including binary files
- Files matching patterns in .gitignore are excluded
- Files matching default ignore patterns are excluded
- Content has been compressed - code blocks are separated by ⋮---- delimiter
- Files are sorted by Git change count (files with more changes are at the bottom)

# Directory Structure

```
app.js
index.html
style.css
```

# Files

## File: app.js

```javascript
const $ = id
⋮----
const fieldNodes = name => document.querySelectorAll(`[data-status="$
function setStatus(name, text, tone = '')
function setCommand(name, value)
⋮----
async function enterControlMode()
async function enterFullscreen()
⋮----
function crc16(data, length)
⋮----
function createControl()
⋮----
function sendNow()
⋮----
function cobsEncode(data)
⋮----
function cobsDecode(encoded)
⋮----
function updateCommandUi()
⋮----
function setKnob(knob, x, y)
⋮----
function drawStickPositions()
⋮----
function resetInputs()
⋮----
function setMotorModeLocked(locked)
⋮----
function updateControlAvailability()
⋮----
function updateMotorModeUi()
⋮----
function renderControlMode()
⋮----
function forceLocalSafe(message, useEmergency)
⋮----
function newSession()
⋮----
async function connect()
⋮----
async function readSerial()
⋮----
// Physical USB removal reaches this path.
⋮----
function serialDisconnected()
⋮----
function decodeStatus(data)
⋮----
function releaseDeadman()
⋮----
function stickGeometry(zone)
function clampUnitVector(x,y)
function applyAxisDeadzone(value,deadzone = .06)
function pointerVector(event,zone)
function updateLeftStick(event)
function updateRightStick(event)
function startStick(event,side)
function moveStick(event,side)
function releaseStick(event,side)
⋮----
function espConnected()
function stmConnected()
function canArm()
⋮----
function keyboardThrottleEnabled()
function isTextEntryTarget(target)
```

## File: index.html

```html
<!doctype html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no,viewport-fit=cover">
  <meta name="mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <title>Drone Controller</title>
  <link rel="stylesheet" href="style.css">
</head>
<body>
<div id="launch" class="launch">
  <div class="launch-card">
    <h2>Drone Controller</h2>
    <p>Tháo cánh quạt khi thử nghiệm. Chế độ joystick được tối ưu cho màn hình ngang.</p>
    <button id="enter">VÀO ĐIỀU KHIỂN</button>
  </div>
</div>

<main>
  <header class="topbar">
    <div class="brand">
      <h1>Drone Controller</h1>
      <p class="sub">Rate control · Quad-X · USB Serial + ESP-NOW</p>
    </div>
    <div class="top-actions">
      <button id="mode-toggle">MỞ JOYSTICK</button>
      <button id="fullscreen" aria-label="Toàn màn hình">⛶</button>
    </div>
  </header>

  <p class="rotate-note">Xoay ngang màn hình để có vùng điều khiển lớn hơn.</p>

  <div id="content" class="test">
    <section id="left-stick-panel" class="panel stick-panel" hidden>
      <h2 class="stick-title">Cần trái</h2>
      <div class="stick-layout">
        <span class="axis-label axis-up">↑<br>Throttle</span>
        <span class="axis-label axis-left">← Yaw</span>
        <div id="left-stick" class="joystick locked" role="slider" aria-label="Throttle và yaw" aria-disabled="true">
          <div id="left-knob" class="knob"></div>
        </div>
        <span class="axis-label axis-right">Yaw →</span>
        <span class="axis-label axis-down">↓<br>Throttle</span>
      </div>
      <div class="stick-readout">Throttle <strong data-command="throttle">0</strong> · Yaw <strong data-command="yaw">0</strong><br><small>Laptop: ↑ / ↓ tăng hoặc giảm ga</small></div>
    </section>

    <section id="status-panel" class="panel">
      <div id="mode-badge" class="mode-badge">TEST MOTOR</div>
      <div class="status-list">
        <div class="status-row"><span>USB → Ground ESP</span><span data-status="ws" class="status-value bad"><i class="dot"></i>DISCONNECTED</span></div>
        <div class="status-row"><span>ESP-NOW Ground ↔ Air</span><span data-status="esp" class="status-value bad"><i class="dot"></i>OFFLINE</span></div>
        <div class="status-row"><span>STM32 link</span><span data-status="stm" class="status-value bad"><i class="dot"></i>OFFLINE</span></div>
        <div class="status-row"><span>Trạng thái drone</span><span data-status="state" class="status-value">UNKNOWN</span></div>
        <div class="status-row"><span>Failsafe</span><span data-status="failsafe" class="status-value">UNKNOWN</span></div>
        <div class="status-row"><span>UART</span><span data-status="rate" class="status-value">0 pkt/s</span></div>
      </div>
      <div class="command-mini">
        <span>Yêu cầu <strong data-command="requested">0 / 500</strong></span>
        <span>Đã áp dụng <strong data-status="applied">0 / 500</strong></span>
        <span>Roll <strong data-command="roll">0</strong></span>
        <span>Pitch <strong data-command="pitch">0</strong></span>
      </div>

      <div id="test-options" class="test-options">
        <p>Chế độ kiểm tra</p>
        <div class="mode-grid">
          <label id="mode-all-label" class="selected"><input id="mode-all" name="motor-mode" type="radio" value="all" checked>Cả 4 motor</label>
          <label id="mode-single-label"><input id="mode-single" name="motor-mode" type="radio" value="single">Từng motor</label>
        </div>
        <div id="motor-picker" class="motor-picker" hidden>
          <select id="motor-select" aria-label="Chọn motor">
            <option value="1">M1 · Front-left · PA6</option>
            <option value="2">M2 · Rear-left · PA7</option>
            <option value="3">M3 · Front-right · PB0</option>
            <option value="4">M4 · Rear-right · PB1</option>
          </select>
        </div>
      </div>

      <div class="actions">
        <button class="arm" data-action="arm">ARM</button>
        <button class="disarm" data-action="disarm">DISARM</button>
        <button class="stop" data-action="stop">DỪNG KHẨN CẤP</button>
      </div>
    </section>

    <section id="right-stick-panel" class="panel stick-panel" hidden>
      <h2 class="stick-title">Cần phải</h2>
      <div class="stick-layout">
        <span class="axis-label axis-up">↑<br>Pitch forward</span>
        <span class="axis-label axis-left">← Roll</span>
        <div id="right-stick" class="joystick locked" role="slider" aria-label="Pitch và roll" aria-disabled="true">
          <div id="right-knob" class="knob"></div>
        </div>
        <span class="axis-label axis-right">Roll →</span>
        <span class="axis-label axis-down">↓<br>Pitch backward</span>
      </div>
      <div class="stick-readout">Roll <strong data-command="roll">0</strong> · Pitch <strong data-command="pitch">0</strong></div>
    </section>

    <section id="test-panel" class="panel">
      <span id="throttle-label">Throttle chung 4 motor</span>
      <output id="requested">0 / 500</output>
      <input id="slider" type="range" min="0" max="500" step="5" value="0" disabled>
      <p class="hint">Dead-man: phải giữ tay trên slider; thả tay sẽ về 0 ngay.</p>
      <div class="motors">
        <div class="motor selected" data-motor="1">M1 · FL<strong class="motor-pwm">1000 µs</strong></div>
        <div class="motor selected" data-motor="2">M2 · RL<strong class="motor-pwm">1000 µs</strong></div>
        <div class="motor selected" data-motor="3">M3 · FR<strong class="motor-pwm">1000 µs</strong></div>
        <div class="motor selected" data-motor="4">M4 · RR<strong class="motor-pwm">1000 µs</strong></div>
      </div>
    </section>
  </div>

  <p id="message">Nhấn VÀO ĐIỀU KHIỂN rồi chọn cổng USB của ESP32.</p>
</main>

<script src="app.js"></script>
</body>
</html>
```

## File: style.css

```css
:root {
* { box-sizing:border-box; }
[hidden] { display:none !important; }
html,body { width:100%; min-height:100%; }
body {
main {
button,select,input { font:inherit; }
button {
button:disabled,select:disabled { opacity:.42; }
.topbar { display:flex; align-items:center; justify-content:space-between; gap:8px; }
.brand { min-width:0; }
h1 { margin:0; font-size:clamp(1rem,2.8vw,1.35rem); line-height:1.1; }
.sub { margin:2px 0 0; color:var(--muted); font-size:.73rem; }
.top-actions { display:flex; gap:7px; }
#mode-toggle { min-width:142px; background:#176a9a; }
#fullscreen { min-width:43px; background:#2b3d50; font-size:1.15rem; }
#content { flex:1; min-height:0; display:grid; gap:9px; align-items:stretch; }
#content.pilot { grid-template-columns:minmax(210px,1fr) minmax(190px,.7fr) minmax(210px,1fr); }
#content.test { grid-template-columns:minmax(245px,.78fr) minmax(380px,1.42fr); }
.panel {
⋮----
/* Compact information and flight actions in the middle column. */
#status-panel { padding:10px 12px; display:flex; flex-direction:column; gap:7px; }
.mode-badge {
.status-list { display:grid; gap:3px; }
.status-row {
.status-row:last-child { border-bottom:0; }
.status-value { font-weight:850; text-align:right; }
.good { color:var(--green); } .bad { color:var(--red); } .warn { color:var(--yellow); }
.dot { display:inline-block; width:8px; height:8px; margin-right:5px; border-radius:50%; background:currentColor; }
.command-mini {
.command-mini strong { color:var(--text); float:right; }
.actions { display:grid; grid-template-columns:1fr 1fr; gap:6px; margin-top:auto; }
.arm { background:#19794f; }
.disarm { background:#3c4d60; }
.stop { grid-column:1/-1; background:#b92834; }
#message {
⋮----
/* Landscape dual-stick controller. */
.stick-panel { padding:7px 9px 9px; display:flex; flex-direction:column; align-items:center; justify-content:center; }
.stick-title { margin:0 0 3px; font-size:.85rem; }
.stick-layout {
.axis-label { color:#aec0d1; font-size:clamp(.62rem,1.25vw,.76rem); font-weight:750; text-align:center; line-height:1.12; }
.axis-up { grid-column:2; grid-row:1; }
.axis-left { grid-column:1; grid-row:2; }
.axis-right { grid-column:3; grid-row:2; }
.axis-down { grid-column:2; grid-row:3; }
.joystick {
.joystick::after {
.joystick.locked { opacity:.45; }
.knob {
.stick-readout { margin-top:1px; color:var(--muted); font-size:.7rem; }
.stick-readout strong { color:var(--blue); }
⋮----
/* Existing no-prop motor test remains available as the other mode. */
#test-panel { padding:12px 14px; text-align:center; display:flex; flex-direction:column; }
#test-panel output { display:block; margin:6px 0; font-size:clamp(2.3rem,9vw,4.4rem); line-height:1; font-weight:950; }
input[type=range] { width:100%; height:66px; accent-color:var(--blue); touch-action:none; }
.hint { margin:0 0 7px; color:var(--muted); font-size:.76rem; }
.mode-grid { display:grid; grid-template-columns:1fr 1fr; gap:6px; margin:6px 0; }
.mode-grid label {
.mode-grid label.selected { border-color:var(--blue); background:#15364d; }
.mode-grid input { accent-color:var(--blue); }
.motor-picker select { width:100%; min-height:39px; padding:6px 9px; color:var(--text); border:1px solid #3b4d60; border-radius:9px; background:#0e1823; }
.motors { display:grid; grid-template-columns:repeat(4,1fr); gap:6px; margin-top:auto; }
.motor { padding:7px 4px; background:#0e1823; border:1px solid transparent; border-radius:9px; font-size:.7rem; }
.motor.selected { border-color:var(--blue); background:#15364d; }
.motor strong { display:block; margin-top:2px; color:var(--green); font-size:.78rem; }
.test-options { margin-top:5px; padding-top:5px; border-top:1px solid var(--line); }
.test-options > p { margin:0; font-size:.75rem; font-weight:800; }
.launch {
.launch-card { max-width:530px; }
.launch h2 { margin:0 0 7px; }
.launch p { color:#a9bbcb; }
#enter { width:100%; background:#19794f; }
.rotate-note { display:none; }
⋮----
#content.pilot { grid-template-columns:1fr 1fr; }
#content.pilot #status-panel { grid-column:1/-1; grid-row:1; }
#content.pilot .stick-panel { grid-row:2; }
#content.test { grid-template-columns:1fr; }
.joystick { width:min(39vw,230px); height:min(39vw,230px); }
.stick-layout { grid-template-columns:46px 1fr 46px; min-height:180px; }
.rotate-note { display:block; margin:0; color:var(--yellow); font-size:.72rem; text-align:center; }
⋮----
.sub { display:none; }
#mode-toggle { min-width:115px; font-size:.72rem; }
#content.pilot { grid-template-columns:1fr; }
#content.pilot .stick-panel { grid-row:auto; }
#content.pilot #status-panel { grid-column:auto; grid-row:auto; }
.motors { grid-template-columns:1fr 1fr; }
⋮----
main { gap:5px; padding-top:5px; padding-bottom:5px; }
.sub,.stick-readout { display:none; }
#status-panel { padding:7px 8px; gap:4px; }
.status-row { min-height:21px; font-size:.7rem; }
.command-mini { padding:5px; }
button { min-height:36px; font-size:.72rem; }
.stick-panel { padding:4px 6px; }
.stick-layout { min-height:170px; grid-template-rows:27px minmax(145px,1fr) 27px; }
.joystick { width:min(29vw,38vh,220px); height:min(29vw,38vh,220px); min-width:145px; min-height:145px; }
#message { min-height:25px; padding:4px 8px; }
#test-panel { padding:8px 11px; }
input[type=range] { height:48px; }
```
