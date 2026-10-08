'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

// Exercise the real packet writer and UI event handlers without a serial device.
const nodes = new Map();
function node(id) {
  if (!nodes.has(id)) nodes.set(id, {
    value:'', dataset:{}, style:{}, clientWidth:200, offsetWidth:40,
    classList:{ toggle() {}, remove() {}, add() {} },
    handlers:{}, addEventListener(name, fn) { this.handlers[name] = fn; },
    setAttribute() {}, setPointerCapture() {}, querySelector() { return node('knob'); }, getBoundingClientRect() { return {left:0,top:0,width:200,height:200}; }
  });
  return nodes.get(id);
}
const timers = new Map();
const windowHandlers = new Map();
class Element {
  constructor(tag) { this.tag = tag; }
  closest() { return this.tag ? this : null; }
}
let timerId = 0;
const context = vm.createContext({
  document:{getElementById:node, addEventListener() {},
    querySelectorAll(selector) {
      if (selector === '[data-action="arm"]') return [node('arm')];
      if (selector === '[data-action="stop"]') return [node('stop')];
      return [];
    }},
  window:{addEventListener(name, fn, options) { windowHandlers.set(name,{fn,options}); }},
  Element, performance:{now:() => 1000},
  requestAnimationFrame() {}, setInterval() {}, console,
  setTimeout(fn) { timers.set(++timerId,fn); return timerId; },
  clearTimeout(id) { timers.delete(id); },
});
vm.runInContext(fs.readFileSync(path.join(__dirname,'../app.js'),'utf8'),context);
const run = source => vm.runInContext(source,context);
function readPacket() {
  run('var createControlForTest = createControl()');
  const bytes = run('createControlForTest');
  const view = new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  assert.equal(bytes.length,30);
  assert.equal(view.getUint16(28,true),run('crc16(createControlForTest,28)'));
  return view;
}
run("controlMode='pilot'; emergency=false; armRequested=true; requested=200; rollCommand=400; pitchCommand=-500; yawCommand=600");
let data = readPacket();
assert.equal(data.getUint16(8,true),5); // ARM | ANGLE
assert.equal(data.getUint16(16,true),200);
assert.equal(data.getInt16(20,true),-500);
run("flightMode='acro'");
assert.equal(readPacket().getUint16(8,true),9);
run("controlMode='test'; deadman=true");
assert.equal(readPacket().getUint16(8,true),33);
run("motorMode='single'; selectedMotor=3");
assert.equal(readPacket().getUint16(24,true),3);
run("controlMode='test'; emergency=true");
data = readPacket();
assert.equal(data.getUint16(8,true),2);
assert.equal(data.getUint16(24,true),0);
run("controlMode='pilot'; emergency=true");
data = readPacket();
assert.equal(data.getUint16(8,true),2);
for (const offset of [16,18,20,22]) assert.equal(data.getInt16(offset,true),0);

// Switching mode sends a zero-throttle DISARM on both sides of the change.
run("emergency=false; var sent=[]; writer={write(frame){sent.push(cobsDecode(frame.slice(1,-1)));return Promise.resolve();}}; armTimer=setTimeout(()=>{},1000)");
node('flight-mode').value = 'angle';
node('flight-mode').handlers.change();
assert.equal(run('sent.length'),2);
assert.equal(run('armRequested'),false);
assert.equal(run('requested'),0);
assert.equal(run('rollCommand'),0);
assert.equal(run('armTimer'),null);
assert.equal(timers.size,0);
const sent = run('sent');
for (const [i,bytes] of Array.from(sent).entries()) {
  const view = new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  assert.equal(view.getUint16(8,true),i === 0 ? 8 : 4);
  for (const offset of [16,18,20,22]) assert.equal(view.getInt16(offset,true),0);
}

// Joystick forward is negative FRD pitch (nose-down); right is positive roll.
run('updateRightStick({clientX:100,clientY:25})');
assert.ok(run('pitchCommand') < 0);
run('updateRightStick({clientX:175,clientY:100})');
assert.ok(run('rollCommand') > 0);

// A pending ARM hold cannot outlive a mode switch or stale link.
run('resetInputs(); lastAcknowledgedAt=1000; lastStatusAt=1000; lastStatus={state:1,errors:0}; emergency=false');
node('arm').handlers.pointerdown();
assert.equal(timers.size,1);
node('flight-mode').value = 'acro';
node('flight-mode').handlers.change();
assert.equal(timers.size,0);
node('arm').handlers.pointerdown();
assert.equal(timers.size,1);
run('lastAcknowledgedAt=1');
const callback = Array.from(timers.values())[0];
timers.clear();
callback();
assert.equal(run('armRequested'),false);

// Pilot input stays locked until the STM32 reports ARMED after zero frames.
run('armRequested=true; lastStatus={state:1,errors:0}; rollCommand=0');
node('right-stick').handlers.pointerdown({pointerId:7,clientX:175,clientY:100});
assert.equal(node('right-stick').dataset.pointer,undefined);
assert.equal(run('rollCommand'),0);
run('lastStatus={state:2,errors:0}');
node('right-stick').handlers.pointerdown({pointerId:7,clientX:175,clientY:100});
assert.equal(node('right-stick').dataset.pointer,'7');
assert.ok(run('rollCommand') > 0);

// Esc uses the real safety handler and serial packet writer in either mode,
// even when a form control owns focus or the STM32 link status is stale.
const keydown = windowHandlers.get('keydown');
assert.equal(keydown.options.capture,true);
function pressEscape(target, repeat = false) {
  let prevented = false;
  keydown.fn({key:'Escape',target,repeat,preventDefault() { prevented = true; }});
  assert.equal(prevented,true);
}
function assertEmergencyPacket(bytes) {
  assert.equal(bytes.length,30);
  const view = new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  assert.equal(view.getUint16(8,true),2); // E-STOP only; ARM/mode flags cleared.
  for (const offset of [16,18,20,22,24,26]) assert.equal(view.getInt16(offset,true),0);
  assert.equal(view.getUint16(28,true),run('crc16(sent[sent.length-1],28)'));
}
for (const mode of ['pilot','test']) {
  for (const tag of ['', 'input', 'textarea', 'select', 'contenteditable']) {
    run(`controlMode=${JSON.stringify(mode)}; emergency=false; armRequested=true;
      deadman=true; requested=350; rollCommand=400; pitchCommand=-500; yawCommand=600;
      lastAcknowledgedAt=1; lastStatusAt=1; sent=[]; armTimer=setTimeout(()=>{},1000)`);
    pressEscape(new Element(tag));
    assert.equal(run('emergency'),true);
    assert.equal(run('armRequested'),false);
    assert.equal(run('deadman'),false);
    assert.equal(run('armTimer'),null);
    assert.equal(timers.size,0);
    assert.equal(run('sent.length'),1); // Sent immediately without the 25 ms timer.
    for (const command of ['requested','rollCommand','pitchCommand','yawCommand']) {
      assert.equal(run(command),0);
    }
    assertEmergencyPacket(run('sent[0]'));
    pressEscape(new Element(tag),true);
    assert.equal(run('sent.length'),2);
    assertEmergencyPacket(run('sent[1]'));
    assert.equal(run('canArm()'),false);
  }
}

// The on-screen STOP button shares precisely the same safety behavior.
run('emergency=false; armRequested=true; requested=300; sent=[]');
node('stop').handlers.click();
assert.equal(run('emergency'),true);
assert.equal(run('armRequested'),false);
assert.equal(run('requested'),0);
assert.equal(run('sent.length'),1);
assertEmergencyPacket(run('sent[0]'));

// With no serial writer, Esc still cancels an ARM hold and zeros local inputs.
run('writer=null; emergency=false; armRequested=false; requested=300; sent=[]; armTimer=setTimeout(()=>{},1000)');
pressEscape(new Element('input'));
assert.equal(run('emergency'),true);
assert.equal(run('requested'),0);
assert.equal(run('armTimer'),null);
assert.equal(timers.size,0);
assert.equal(run('sent.length'),0);
// Telemetry supports the old layout and the synchronized snapshot extension.
function telemetryPacket({legacy=false, sampleTime=0xffffffff, sampleId=0,
                          commitTime=0, flags=0x3a} = {}) {
  const bytes = new Uint8Array(legacy ? 50 : 58);
  const view = new DataView(bytes.buffer);
  view.setUint16(0,0xa55a,true);
  view.setUint8(2,1); view.setUint8(3,legacy ? 7 : 8);
  view.setUint16(4,42,true); view.setUint16(6,9,true);
  view.setUint16(8,legacy ? flags & 0x1f : flags,true);
  view.setUint8(10,legacy ? 32 : 40);
  view.setUint32(12,sampleTime,true);
  view.setInt16(16,-123,true); view.setInt16(22,1000,true);
  view.setInt16(28,2000,true); view.setInt16(34,-456,true);
  [40,42,44,46].forEach(offset => view.setUint16(offset,1500,true));
  if (!legacy) {
    view.setUint32(48,sampleId,true); view.setUint32(52,commitTime,true);
  }
  context.fixture = bytes;
  view.setUint16(bytes.length-2,run('crc16(fixture,fixture.length-2)'),true);
  return bytes;
}
function decodeTelemetry(bytes) {
  context.fixture = bytes;
  return run('decodeFlightTelemetry(fixture)');
}
run('clearTelemetryData(); telemetryRecording=true');
assert.equal(decodeTelemetry(telemetryPacket()),true);
assert.equal(run('telemetryHistory[0].sampleId'),0); // Zero is valid after wrap.
assert.equal(run('telemetryHistory[0].motorCommitTimeMs'),0);
assert.equal(run('telemetryHistory[0].motorCommitDelayMs'),1);
assert.equal(run('telemetryHistory[0].outputSampleMatched'),true);
assert.equal(run('telemetryHistory[0].gyroRadS[0]'),1);
assert.equal(run('telemetryHistory[0].pidOutput[0]'),-4.56);
assert.equal(run('telemetryRow(telemetryHistory[0]).sample_id'),0);
assert.equal(run('telemetryRow(telemetryHistory[0]).sample_time_ms'),0xffffffff);
assert.equal(run('telemetryRow(telemetryHistory[0]).motor_commit_time_ms'),0);
assert.equal(run('recordedTelemetry.length'),1);
assert.match(node('telemetry-summary').textContent,/sample 0.*commit \+1 ms/);

assert.equal(decodeTelemetry(telemetryPacket({flags:0x11,sampleId:17})),true);
assert.equal(run('telemetryHistory[1].outputSampleMatched'),false);
assert.equal(run('telemetryHistory[1].motorCommitDelayMs'),null);
assert.equal(run('telemetryRow(telemetryHistory[1]).motor_commit_delay_ms'),'');
assert.equal(decodeTelemetry(telemetryPacket({legacy:true})),true);
assert.equal(run('telemetryHistory[2].sampleId'),null);
assert.equal(run('telemetryHistory[2].motorCommitTimeMs'),null);
assert.equal(run('telemetryRow(telemetryHistory[2]).sample_id'),'');
assert.match(node('telemetry-summary').textContent,/legacy/);

// A checksum-valid packet must also have the correct type/size/flag contract.
function mutateTelemetry(bytes, offset, value) {
  bytes[offset] = value;
  context.fixture = bytes;
  new DataView(bytes.buffer).setUint16(bytes.length-2,run('crc16(fixture,fixture.length-2)'),true);
  return bytes;
}
assert.equal(decodeTelemetry(mutateTelemetry(telemetryPacket(),3,7)),false);
assert.equal(decodeTelemetry(mutateTelemetry(telemetryPacket({legacy:true}),8,0x3a)),false);
assert.equal(decodeTelemetry(mutateTelemetry(telemetryPacket(),10,32)),false);
const damaged = telemetryPacket(); damaged[52] ^= 1;
assert.equal(decodeTelemetry(damaged),false);
assert.equal(run('telemetryInvalidCount'),4);
assert.equal(run('telemetryHistory.length'),3);
console.log('web control mode and telemetry tests: PASS');
