// The default choices must give back the shipped firmware files, byte for byte: a change to
// one of them fails here until the configurator follows.
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const M = require('../model.js');
const G = require('../generate.js');

const repo = path.join(__dirname, '..', '..');
// the pre-commit hook rewrites @date, so it is left out of the comparison
const shipped = (file) => fs.readFileSync(path.join(repo, file), 'utf8').replace(/^( \* @date ).*$/m, '$1');
const generated = (text) => text.replace(/^( \* @date ).*$/m, '$1');

test('defaults give back config.h', () => {
  assert.strictEqual(generated(G.configH(M.defaults())), shipped('Mk2_1phase_RFdatalog_temp/config.h'));
});

test('defaults give back config_system.h', () => {
  assert.strictEqual(generated(G.configSystemH(M.defaults())), shipped('Mk2_1phase_RFdatalog_temp/config_system.h'));
});

test('defaults give back config_rf.h', () => {
  assert.strictEqual(generated(G.configRfH(M.defaults())), shipped('Mk2_1phase_RFdatalog_temp/config_rf.h'));
});

// the shipped receiver is unit 1, with 2 loads on D4 and D3
function oneRemoteUnit() {
  const m = M.defaults();
  m.loads.push({ type: 'remote', unit: 1, led: null }, { type: 'remote', unit: 1, led: null });
  m.priorities = m.loads.map((_, i) => i);
  return m;
}

test('defaults give back RemoteLoadReceiver/config.h', () => {
  assert.strictEqual(generated(G.receiverConfigH(oneRemoteUnit(), 1)), shipped('RemoteLoadReceiver/config.h'));
});

test('defaults give back RemoteLoadReceiver/config_rf.h', () => {
  assert.strictEqual(generated(G.receiverConfigRfH(oneRemoteUnit(), 1)), shipped('RemoteLoadReceiver/config_rf.h'));
});

test('trailing comments are aligned within runs of consecutive lines', () => {
  const text = G.render([['int a;', '/**< a */'], ['long bb;', '// b'], '', ['c;', '/**< c */']]);
  assert.strictEqual(text, 'int a;    /**< a */\nlong bb;  // b\n\nc; /**< c */\n');
});

test('files: RF and receiver files only when needed, YAML only with mk2Wifi', () => {
  const names = (m) => G.files(m, '2026-01-01').map((f) => f.path);
  assert.deepStrictEqual(names(M.defaults()), ['Mk2_1phase_RFdatalog_temp/config.h', 'Mk2_1phase_RFdatalog_temp/config_system.h']);
  const m = oneRemoteUnit();
  m.mk2wifi.enabled = true;
  assert.deepStrictEqual(names(m), [
    'Mk2_1phase_RFdatalog_temp/config.h',
    'Mk2_1phase_RFdatalog_temp/config_system.h',
    'Mk2_1phase_RFdatalog_temp/config_rf.h',
    'RemoteLoadReceiver-unit1/config.h',
    'RemoteLoadReceiver-unit1/config_rf.h',
    'mk2pvrouter.yaml',
  ]);
});

test('relay count: new relays get the README thresholds, a single relay fits on one line', () => {
  const m = M.defaults();
  m.relays.enabled = true;
  M.resizeRelays(m, 4);
  assert.deepStrictEqual(m.relays.list[3], { pin: null, surplus: 1000, import: 200, minOn: 5, minOff: 5 });
  M.resizeRelays(m, 1);
  m.relays.list[0].pin = 8;
  assert.match(G.configH(m), /^ {37}\{ \{ 8, 100, 200, 1, 1 \} \} \}; \/\*\*< config for relay diversion/m);
});

test('motherboard: the new board selects PcbVersion::NEW', () => {
  const m = M.defaults();
  m.pcbVersion = 'NEW';
  assert.match(G.configH(m), /^inline constexpr PcbVersion PCB_VERSION\{ PcbVersion::NEW \};$/m);
});

test('override pins: a comment says what each one forces, numbered as on the page', () => {
  const m = M.defaults();
  m.loads = [
    { type: 'remote', unit: 1, led: null },
    { type: 'local', pin: 6 },
  ];
  m.priorities = [0, 1];
  m.relays.enabled = true;
  M.resizeRelays(m, 1);
  m.relays.list[0].pin = 8;
  m.overrides.list = [
    { pin: 5, wifi: false, targets: [{ load: 0 }] },
    { pin: 9, wifi: false, targets: [{ load: 1 }, { relay: 0 }] },
    { pin: 14, wifi: false, targets: 'ALL_REMOTE_LOADS' },
  ];
  const text = G.configH(m);
  assert.match(text, /^\/\/ {3}D5: load 1 \(remote unit 1\)\n\/\/ {3}D9: load 2 \(D6\), relay 1 \(D8\)\n\/\/ {3}A0: all remote loads\ninline constexpr OverridePins/m);
  assert.doesNotMatch(G.configH(M.defaults()), /What each override pin forces/);
});

// One boost of the default model replaced: its line in config.h
function boostsLine(boosts) {
  const m = M.defaults();
  m.dualTariff.boosts = boosts;
  return G.configH(m)
    .split('\n')
    .find((l) => l.startsWith('inline constexpr ScheduledBoosts'));
}

test('scheduled boosts: each kind of window gives its form', () => {
  const one = (b) => boostsLine([{ targets: [{ load: 0 }], ...b }]);
  assert.match(one({ from: 'start', at: 0, for: 120 }), /scheduledBoosts\{ \{ \{ FIRST\(HOURS\(2\)\), \{ LOAD\(0\) \} \} \} \};/);
  assert.match(one({ from: 'end', at: 90, for: null }), /scheduledBoosts\{ \{ \{ LAST\(MINUTES\(90\)\), \{ LOAD\(0\) \} \} \} \};/);
  assert.match(one({ from: 'start', at: 60, for: null }), /scheduledBoosts\{ \{ \{ FROM_START\(HOURS\(1\)\), \{ LOAD\(0\) \} \} \} \};/);
  assert.match(one({ from: 'end', at: 180, for: 90 }), /scheduledBoosts\{ \{ \{ BEFORE_END\(HOURS\(3\)\), FOR\(MINUTES\(90\)\), \{ LOAD\(0\) \} \} \} \};/);
  assert.match(one({ from: 'start', at: 30, for: 60 }), /scheduledBoosts\{ \{ \{ FROM_START\(MINUTES\(30\)\), FOR\(HOURS\(1\)\), \{ LOAD\(0\) \} \} \} \};/);
});

test('scheduled boosts: an empty list, a group of targets', () => {
  assert.match(boostsLine([]), /^inline constexpr ScheduledBoosts scheduledBoosts\{\};/);
  assert.match(boostsLine([{ from: 'start', at: 0, for: 120, targets: 'ALL_RELAYS' }]), /\{ \{ \{ FIRST\(HOURS\(2\)\), ALL_RELAYS\(\) \} \} \}; \/\*\*< all relays, the first 2 hours \*\/$/);
});

test('scheduled boosts: several, one per line, a comment says what each one forces', () => {
  const m = M.defaults();
  m.dualTariff.boosts = [
    { from: 'end', at: 180, for: 120, targets: [{ load: 0 }] },
    { from: 'start', at: 0, for: 60, targets: 'ALL_LOADS' },
  ];
  assert.match(
    G.configH(m),
    /^\/\/ What each scheduled boost forces .*\n\/\/ {3}boost 1: load 1 \(D5\)\n\/\/ {3}boost 2: all loads\ninline constexpr ScheduledBoosts scheduledBoosts\{ \{ \{ BEFORE_END\(HOURS\(3\)\), FOR\(HOURS\(2\)\), \{ LOAD\(0\) \} \},\n {52}\{ FIRST\(HOURS\(1\)\), ALL_LOADS\(\) \} \} \};/m
  );
});

test('normalize: the former force list is dropped, the default boosts take its place', () => {
  const m = M.normalize({ dualTariff: { enabled: true, pin: 3, force: [{ start: -3, duration: 2 }] } });
  assert.ok(!('force' in m.dualTariff));
  assert.deepStrictEqual(m.dualTariff.boosts, M.defaults().dualTariff.boosts);
  assert.deepStrictEqual(M.defaults().dualTariff.boosts, [{ from: 'end', at: 180, for: 120, targets: [{ load: 0 }] }]);
});

const calibrationFile = () => fs.readFileSync(path.join(repo, 'Mk2_1phase_RFdatalog_temp/calibration.h'), 'utf8');
// The text with one value replaced: the tests follow the values of calibration.h
function withValue(text, name, value) {
  const t = M.calibrationToken(text, name);
  return text.slice(0, t.start) + value + text.slice(t.end);
}
// Calibration mode on the given board, the shipped calibration.h loaded
function loaded(pcbVersion, change = () => {}) {
  const m = M.defaults();
  m.pcbVersion = pcbVersion;
  m.calibrationMode = true;
  const text = calibrationFile();
  Object.assign(m.calibration, { file: text, fileName: 'calibration.h', ...M.parseCalibration(text) });
  change(m);
  return m;
}
// The numbers of the lines that differ between two texts
function changedLines(a, b) {
  const x = a.split('\n');
  const y = b.split('\n');
  assert.strictEqual(x.length, y.length);
  return x.flatMap((l, i) => (l === y[i] ? [] : [i]));
}

test('calibration: the values of both boards, read from calibration.h', () => {
  const text = calibrationFile();
  assert.deepStrictEqual(M.parseCalibration(text), {
    powerCalOldBoard: 0.05,
    powerCalDivertedOldBoard: 0.05,
    voltageCalOldBoard: 0.805653,
    ctRating: 50,
    ctRatingDiverted: 50,
    powerCalPerAmp: 0.00286405,
    voltageCalNewBoard: 0.888065256,
  });
  assert.strictEqual(M.calibrationValue(text, 'f_ctRating'), 50, 'f_ctRating, not f_ctRatingDiverted');
  assert.strictEqual(M.calibrationValue(text, 'f_powerCal'), null, 'only named in the comments');
  assert.strictEqual(M.calibrationValue('inline constexpr float f_ctRating{ 50.0F, 50.0F };', 'f_ctRating'), null, 'a list');
});

test('calibration: a commented-out line is not the one read or rewritten', () => {
  const comment = '// inline constexpr float f_powerCalPerAmp{ 1.0F };\n';
  const text = comment + calibrationFile();
  assert.strictEqual(M.calibrationValue(text, 'f_powerCalPerAmp'), 0.00286405);
  const m = loaded('NEW', (m) => {
    m.calibration.file = text;
    m.calibration.power = { router: 500, meter: 1000 };
  });
  assert.ok(G.calibrationH(m).startsWith(comment));
  assert.strictEqual(G.calibrationH(m), withValue(text, 'f_powerCalPerAmp', '0.00572810F'));
  assert.strictEqual(M.parseCalibration('int a;'), null);
  // the calibration.h of the 3-phase router, with per-phase arrays, is not this one
  assert.strictEqual(M.parseCalibration(calibrationFile().replace('f_powerCalPerAmp{ 0.00286405F }', 'f_powerCalPerAmp[NO_OF_PHASES]{ 0.00286405F }')), null);
});

test('calibration, old boards: the grid and the diverted power correct their own values', () => {
  const m = loaded('OLD_OLED', (m) => {
    m.calibration.power = { router: 1000, meter: 1100 };
    m.calibration.diverted = { router: 500, meter: 450 };
    m.calibration.voltage = { router: 230, meter: 232 };
  });
  const r = M.calibrationResult(m);
  assert.strictEqual(r.power.toFixed(5), '0.05500');
  assert.strictEqual(r.diverted.toFixed(5), '0.04500');
  assert.strictEqual(r.voltage.toFixed(6), '0.812659');
  assert.strictEqual(r.divertedError, null, 'old boards: corrected, not checked');
  assert.deepStrictEqual(G.calibrationLines(m), [
    'inline constexpr float f_powerCalOldBoard{ 0.05500F };',
    'inline constexpr float f_powerCalDivertedOldBoard{ 0.04500F };',
    'inline constexpr float f_voltageCalOldBoard{ 0.812659F };',
  ]);
  const text = calibrationFile();
  const out = G.calibrationH(m);
  let expected = withValue(text, 'f_powerCalOldBoard', '0.05500F');
  expected = withValue(expected, 'f_powerCalDivertedOldBoard', '0.04500F');
  expected = withValue(expected, 'f_voltageCalOldBoard', '0.812659F');
  assert.strictEqual(out, expected);
  assert.strictEqual(changedLines(text, out).length, 3, 'every other byte untouched');
});

test('calibration, new board: f_powerCalPerAmp corrected, the CT ratings as entered', () => {
  const m = loaded('NEW', (m) => (m.calibration.power = { router: 1000, meter: 1100 }));
  const value = '0.00315046'; // 0.00286405 x 1.1 = 0.003150455
  assert.strictEqual(M.calibrationResult(m).power.toFixed(8), value);
  assert.deepStrictEqual(G.calibrationLines(m), [
    'inline constexpr float f_ctRating{ 50.0F };',
    'inline constexpr float f_ctRatingDiverted{ 50.0F };',
    `inline constexpr float f_powerCalPerAmp{ ${value}F };`,
    'inline constexpr float f_voltageCalNewBoard{ 0.888065256F };',
  ]);
  const text = calibrationFile();
  assert.strictEqual(G.calibrationH(m), withValue(text, 'f_powerCalPerAmp', `${value}F`));
  assert.strictEqual(changedLines(text, G.calibrationH(m)).length, 1, 'every other byte untouched');

  m.calibration.power = { router: null, meter: null };
  m.calibration.ctRating = 30.3;
  assert.strictEqual(G.calibrationH(m), withValue(text, 'f_ctRating', '30.3F'), 'a new CT rating');
  m.pcbVersion = 'OLD_SEG';
  assert.strictEqual(G.calibrationH(m), null, 'no CT rating on the old boards');
});

test('calibration, new board: the diverted power checks CT2, no CT2 line changes', () => {
  const m = loaded('NEW', (m) => (m.calibration.diverted = { router: 500, meter: 450 }));
  const r = M.calibrationResult(m);
  assert.strictEqual(r.diverted, null, 'CT2 has no factor of its own');
  assert.strictEqual((r.divertedError * 100).toFixed(1), '-10.0');
  assert.strictEqual(G.calibrationH(m), null, 'nothing to write');
  assert.ok(G.calibrationLines(m).includes('inline constexpr float f_ctRatingDiverted{ 50.0F };'));
  // the error left after the grid correction, which CT2 shares (f_powerCalPerAmp)
  m.calibration.power = { router: 1000, meter: 900 };
  assert.strictEqual((M.calibrationResult(m).divertedError * 100).toFixed(1), '0.0');
});

test('calibration without CT2: no diverted reading, no CT2 line', () => {
  for (const board of ['OLD_SEG', 'NEW']) {
    const m = loaded(board, (m) => {
      m.divertedCtPresent = false;
      m.calibration.diverted = { router: 500, meter: 450 };
    });
    const r = M.calibrationResult(m);
    assert.strictEqual(r.diverted, null, board);
    assert.strictEqual(r.divertedError, null, board);
    assert.ok(!G.calibrationLines(m).some((l) => /Diverted/.test(l)), board);
    assert.strictEqual(G.calibrationH(m), null, board);
  }
});

test('calibration: no lines while a current value is missing', () => {
  const m = M.defaults();
  assert.strictEqual(G.calibrationLines(m), null);
  Object.assign(m.calibration, { ctRating: 50, ctRatingDiverted: 50, powerCalPerAmp: 0.0029, voltageCalNewBoard: 0.89 });
  assert.ok(G.calibrationLines(m));
  m.pcbVersion = 'OLD_SEG';
  assert.strictEqual(G.calibrationLines(m), null, 'no values of the old boards yet');
});

test('calibration mode: human-readable output and no YAML, the usual choice kept', () => {
  const m = M.defaults();
  m.serialOutput = 'IoT';
  m.mk2wifi.enabled = true;
  m.calibrationMode = true;
  assert.match(G.configH(m), /SERIAL_OUTPUT_TYPE = SerialOutputType::HumanReadable;/);
  assert.ok(!G.files(m).some((f) => f.path.endsWith('.yaml')));
  assert.strictEqual(m.serialOutput, 'IoT');
  m.calibrationMode = false;
  assert.match(G.configH(m), /SERIAL_OUTPUT_TYPE = SerialOutputType::IoT;/);
  assert.ok(G.files(m).some((f) => f.path.endsWith('.yaml')));
});

test('calibration mode: config.h and the new calibration.h, nothing else', () => {
  const m = M.resizeLoads(M.defaults(), 2);
  m.loads[1] = { type: 'remote', unit: 1, led: null };
  m.calibrationMode = true;
  assert.deepStrictEqual(
    G.files(m).map((f) => f.path),
    ['Mk2_1phase_RFdatalog_temp/config.h']
  );
  const text = calibrationFile();
  Object.assign(m.calibration, { file: text, ...M.parseCalibration(text) });
  assert.strictEqual(G.calibrationH(m), null, 'nothing changes without readings');
  assert.deepStrictEqual(
    G.files(m).map((f) => f.path),
    ['Mk2_1phase_RFdatalog_temp/config.h']
  );
  m.calibration.power = { router: 1000, meter: 1020 };
  assert.deepStrictEqual(
    G.files(m).map((f) => f.path),
    ['Mk2_1phase_RFdatalog_temp/config.h', 'Mk2_1phase_RFdatalog_temp/calibration.h']
  );
  assert.strictEqual(G.files(m)[1].text, G.calibrationH(m));
  m.calibrationMode = false;
  assert.ok(!G.files(m).some((f) => f.path.endsWith('calibration.h')), 'only in calibration mode');
});

test('normalize: the calibration values are single numbers, per-phase arrays are not taken', () => {
  const m = M.normalize({
    calibration: { ctRating: [50, 50, 50], powerCalPerAmp: 0.003, power: [{ router: 1000, meter: 1100 }], voltage: { router: 230, meter: 232 } },
  });
  assert.strictEqual(m.calibration.ctRating, null);
  assert.strictEqual(m.calibration.powerCalPerAmp, 0.003);
  assert.deepStrictEqual(m.calibration.power, { router: null, meter: null });
  assert.deepStrictEqual(m.calibration.voltage, { router: 230, meter: 232 });
  assert.deepStrictEqual(m.calibration.diverted, { router: null, meter: null });
});

test('temperature threshold: the sensor it is checked on', () => {
  const m = M.defaults();
  m.dualTariff.tempSensor = 1;
  assert.match(G.configH(m), /^inline constexpr uint8_t BOOST_TEMP_SENSOR\{ 1 \};/m);
});

test('removing a sensor: the boosts keep the same sensor, or fall back to the first', () => {
  const three = () => {
    const m = M.defaults();
    m.temperature.sensors = [{ address: 'A', name: '' }, { address: 'B', name: '' }, { address: 'C', name: '' }];
    return m;
  };
  let m = three();
  m.dualTariff.tempSensor = 2;
  M.removeSensor(m, 0); // one before it: same sensor, one place lower
  assert.deepStrictEqual([m.temperature.sensors.length, m.dualTariff.tempSensor], [2, 1]);
  m = three();
  m.dualTariff.tempSensor = 1;
  M.removeSensor(m, 1); // the chosen one: the first
  assert.strictEqual(m.dualTariff.tempSensor, 0);
  m = three();
  m.dualTariff.tempSensor = 0;
  M.removeSensor(m, 2); // one after it: unchanged
  assert.strictEqual(m.dualTariff.tempSensor, 0);
});

test('temperature: D3 on the new board, wired there; the chosen pin on the old one', () => {
  const m = M.normalize({ pcbVersion: 'NEW', temperature: { mode: 'router', pin: 4 } });
  assert.match(G.configH(m), /^inline constexpr TemperatureSensing temperatureSensing\{ 3,$/m);
  m.pcbVersion = 'OLD_SEG';
  assert.match(G.configH(m), /^inline constexpr TemperatureSensing temperatureSensing\{ 4,$/m);
});

test('no probe read by the router: an unreadable threshold is written as 100, the firmware ignores it', () => {
  const m = M.defaults();
  m.dualTariff.tempThreshold = null;
  assert.match(G.configH(m), /^inline constexpr int16_t iTemperatureThreshold\{ 100 \};/m);
  m.dualTariff.tempThreshold = 60;
  assert.match(G.configH(m), /^inline constexpr int16_t iTemperatureThreshold\{ 60 \};/m);
});

test('fewer loads or relays: the boosts and override pins drop the targets that no longer exist', () => {
  const m = M.defaults(); // 3 loads
  M.resizeLoads(m, 4);
  M.resizeRelays(m, 2);
  m.dualTariff.boosts = [{ from: 'start', at: 0, for: 60, targets: [{ load: 0 }, { load: 3 }, { relay: 1 }] }];
  m.overrides.list = [{ pin: 4, wifi: false, targets: [{ load: 3 }, { relay: 0 }] }, { pin: 8, wifi: false, targets: 'ALL_LOADS' }];
  M.resizeLoads(m, 3);
  M.resizeRelays(m, 1);
  assert.deepStrictEqual(m.dualTariff.boosts[0].targets, [{ load: 0 }]);
  assert.deepStrictEqual(m.overrides.list[0].targets, [{ relay: 0 }]);
  assert.strictEqual(m.overrides.list[1].targets, 'ALL_LOADS');
});

test('a new load avoids the probes pin', () => {
  const m = M.defaults(); // new board, loads on 5, 6, 7
  m.temperature.mode = 'router'; // probes on D3
  M.resizeLoads(m, 4);
  assert.strictEqual(m.loads[3].pin, 4);
});

const rotationLine = (m) => G.configH(m).split('\n').find((l) => l.includes('RotationGroups rotationGroups'));

test('rotation groups: none without rotation, even with groups in the model', () => {
  const m = M.defaults();
  m.rotation.groups = [[0, 1]];
  assert.match(rotationLine(m), /rotationGroups\{\};/);
});

test('rotation groups: every local load is ALL_LOCAL_LOADS(), whatever the order', () => {
  const m = M.resizeLoads(M.defaults(), 3);
  Object.assign(m.rotation, { mode: 'AUTO', groups: [[2, 0, 1]] });
  assert.match(rotationLine(m), /rotationGroups\{ \{ ALL_LOCAL_LOADS\(\) \} \};/);
});

test('rotation groups: two groups, the loads listed', () => {
  const m = M.defaults();
  M.resizeLoads(m, 4);
  Object.assign(m.rotation, { mode: 'AUTO', groups: [[0, 1], [2, 3]] });
  assert.match(rotationLine(m), /rotationGroups\{ \{ LOAD\(0\), LOAD\(1\) \}, \{ LOAD\(2\), LOAD\(3\) \} \};/);
});

test('rotation delay: a hidden, invalid delay is written as the 8-hour default', () => {
  const delayLine = (m) => G.configH(m).split('\n').find((l) => l.includes('ROTATION_AFTER_SECONDS{'));
  const m = M.defaults();
  Object.assign(m.rotation, { mode: 'PIN', pin: 4, afterSeconds: 0 });
  assert.match(delayLine(m), /ROTATION_AFTER_SECONDS\{ 8UL \* 3600UL \}/);
  Object.assign(m.rotation, { mode: 'AUTO', afterSeconds: 2 * 3600 });
  assert.match(delayLine(m), /ROTATION_AFTER_SECONDS\{ 2UL \* 3600UL \}/);
});

test('boards and display: OLD_SEG with SEG_HW', () => {
  const m = M.defaults();
  Object.assign(m, { pcbVersion: 'OLD_SEG', displayType: 'SEG_HW' });
  m.loads[0].pin = 3;
  const h = G.configH(m);
  assert.match(h, /^inline constexpr PcbVersion PCB_VERSION\{ PcbVersion::OLD_SEG \};$/m);
  assert.match(h, /^inline constexpr DisplayType TYPE_OF_DISPLAY\{ DisplayType::SEG_HW \};/m);
});

test('the shipped single-load notes only for one local load on pin 5', () => {
  const shippedNotes = (h) => [h.includes('Shipped with one local load'), h.includes('by default, the load\n//   uses D5')];
  assert.deepStrictEqual(shippedNotes(G.configH(M.defaults())), [true, true]);
  const remote = M.defaults();
  remote.loads = [{ type: 'remote', unit: 1, led: null }];
  assert.deepStrictEqual(shippedNotes(G.configH(remote)), [false, false]);
  const pin6 = M.defaults();
  pin6.loads[0].pin = 6;
  assert.deepStrictEqual(shippedNotes(G.configH(pin6)), [false, false]);
  assert.match(G.configH(remote), /by default, the loads\n\/\/ {3}use D5, D6 and D7\./);
});

test('CT2 absent and no anti-creep limit: the two constants change, the section stays', () => {
  const m = M.defaults();
  m.divertedCtPresent = false;
  m.antiCreepLimit = 0;
  const lines = (t) => t.split('\n').filter((l) => /DIVERTED_CT_PRESENT|ANTI_CREEP_LIMIT|DISPLAY_SHUTDOWN_IN_HOURS/.test(l));
  const was = lines(G.configH(M.defaults()));
  const now = lines(G.configH(m));
  assert.strictEqual(now.length, was.length);
  assert.ok(now[0].includes('DIVERTED_CT_PRESENT{ false }'));
  assert.ok(now[1].includes('ANTI_CREEP_LIMIT{ 0 }'));
  // the rest is the same, but for the alignment of the comments next to the two changed values
  const squash = (t) => t.replace(/ +(?=\/\*\*<)/g, ' ').replace('DIVERTED_CT_PRESENT{ false }', 'DIVERTED_CT_PRESENT{ true }').replace('ANTI_CREEP_LIMIT{ 0 }', 'ANTI_CREEP_LIMIT{ 5 }');
  assert.strictEqual(squash(G.configH(m)), squash(G.configH(M.defaults())));
});
