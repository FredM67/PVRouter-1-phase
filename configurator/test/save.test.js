// SPDX-License-Identifier: GPL-3.0-or-later
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const M = require('../model.js');
const G = require('../generate.js');
const I = require('../i18n.js');

test('saveRouter: a save without the field is a 3-phase save', () => {
  assert.strictEqual(M.defaults().router, 1);
  assert.strictEqual(M.saveRouter({ formatVersion: 1 }), 3);
  assert.strictEqual(M.saveRouter({ router: 1 }), 1);
  assert.strictEqual(M.saveRouter({ router: 3 }), 3);
});

test('normalize: refuses the saves of the other router', () => {
  assert.throws(() => M.normalize({ formatVersion: 1, pcbVersion: 'NEW' }), /router/);
  assert.throws(() => M.normalize({ router: 3 }), /router/);
  assert.strictEqual(M.normalize({ router: 1 }).router, 1);
  assert.strictEqual(M.normalize(M.defaults()).router, 1);
});

test('normalize: a missing setting gets its default', () => {
  assert.strictEqual(M.normalize({ ...M.defaults(), antiCreepLimit: undefined }).antiCreepLimit, 5);
});

test('files: never a YAML file, in any mode', () => {
  const m = M.defaults();
  m.mk2wifi.enabled = true;
  assert.ok(!G.files(m).some((f) => f.path.endsWith('.yaml')));
  m.calibrationMode = true;
  assert.ok(!G.files(m).some((f) => f.path.endsWith('.yaml')));
});

test('threePhaseUrl: relative on the docs site, absolute elsewhere', () => {
  const abs = 'https://fredm67.github.io/Mk2PVRouter/configurateur/';
  assert.strictEqual(M.threePhaseUrl({ protocol: 'https:', pathname: '/Mk2PVRouter/configurateur-monophase/' }), '../configurateur/');
  assert.strictEqual(M.threePhaseUrl({ protocol: 'https:', pathname: '/Mk2PVRouter/dev/configurateur-monophase/index.html' }), '../configurateur/');
  assert.strictEqual(M.threePhaseUrl({ protocol: 'file:', pathname: '/D:/Git/Mk2PVRouter/configurator/index.html' }), abs);
  assert.strictEqual(M.threePhaseUrl({ protocol: 'https:', pathname: '/PVRouter-1-phase/configurator/' }), abs);
  assert.strictEqual(M.threePhaseUrl({ protocol: 'file:', pathname: '/x/Mk2PVRouter/configurator/index.html' }), abs);
});

test('texts: single-phase page, YAML to come, calibration needs a load running by other means', () => {
  for (const lang of ['en', 'fr']) {
    assert.ok(I.t(lang, 'where.yamlNext'));
    assert.ok(I.t(lang, 'router.refused', { link: 'U' }).includes('U'));
    assert.ok(!/3-phase|triphas/i.test(I.t(lang, 'title')));
    assert.match(I.t(lang, 'cal.intro'), lang === 'en' ? /never switches/ : /ne commute jamais/);
  }
  for (const lang of ['en', 'fr']) {
    assert.match(I.t(lang, 'cal.divertedNewHelp'), /f_ctRatingDiverted/);
    assert.ok(!/only a check|seulement une vérification/.test(I.t(lang, 'cal.intro')), lang);
  }
  assert.ok(!('cal.divertedCheckHelp' in I.STRINGS.en) && !('warn.calCt2' in I.STRINGS.en));
});
