# Configurator for the single-phase router

Status: design, for approval. Repo: PVRouter-1-phase, new `configurator/` (plus `RemoteLoadReceiver/`).
Small companion changes: PVRouter-3-phase (`configurator/`, the router switch) and Mk2PVRouter
(publishing).

## Intent

The 3-phase repo has a web configurator (`configurator/`). It writes `config.h`, `config_system.h`,
`config_rf.h`, the RemoteLoadReceiver files and the mk2Wifi ESPHome YAML. Its CI checks two things:
its default choices give back the shipped files byte for byte, and what it writes builds. The
single-phase router (`Mk2_1phase_RFdatalog_temp`) now has the same features and nearly the same
settings files, but no configurator: its users edit `config.h` by hand.

From the user's point of view there is **one** configurator, on the documentation site, and it
starts with the router: single-phase or 3-phase. Internally, each configurator must stay in sync with
its own firmware, and the two firmwares may drift apart. So each repo has its own configurator,
tested only against its own code, with no dependency between the repos.

## Goals

- `configurator/` in PVRouter-1-phase, started as a copy of the 3-phase one and adapted:
  - its defaults give back the single-phase shipped `config.h`, `config_system.h` and `config_rf.h`
    byte for byte (`@date` aside);
  - its presets build against `Mk2_1phase_RFdatalog_temp`;
  - the repo's CI checks both.
- `RemoteLoadReceiver/` copied into PVRouter-1-phase. The single-phase router sends the same frames,
  so the configurator writes and builds receivers against the repo's own copy, as on the 3-phase
  side.
- The single-phase settings the 3-phase router does not have, with the checks of the single-phase
  `validation.h`:
  - the three boards: OLD_SEG, OLD_OLED, NEW;
  - CT2 (diverted energy): present, anti-creep limit, display shutdown delay;
  - the display: NONE, SEG, SEG_HW, OLED.
- The calibration helper on the single-phase `calibration.h`: scalars per board, plus CT2.
- One configurator for the user. Each page has a "Router: single-phase / 3-phase" switch at the top,
  a link to the other page, and the documentation site publishes both.

## Non-goals

- The single-phase ESPHome YAML. It is the next step, with its own design: the single-phase IoT frame
  is not the 3-phase one. Until then, the page writes the firmware files and no YAML (see mk2Wifi
  below).
- No firmware change in either repo.
- No shared code, submodule or cross-repo CI between the two configurators. A fix made on one side is
  carried to the other by hand when it applies.

## Anti-goals

- A configurator that no longer matches its firmware without its CI failing.
- A 3-phase user who sees single-phase settings, or the reverse.
- A save that silently loads into the wrong router's page.

## Constraints

- As on the 3-phase side:
  - plain scripts, no build step, no dependency; they load in the browser and in Node;
  - English and French texts, French typography (U+00A0 before `: ; ! ?` and inside « »).
- Generated files are clang-format clean and build.
- The 3-phase configurator keeps giving back its shipped files. Its only change is the switch link
  and the save check.

## Approaches

1. One codebase in PVRouter-3-phase with a router switch; CI checks out PVRouter-1-phase.
2. A copy in PVRouter-1-phase, adapted and tested against its own firmware (**chosen by the user**).
3. A configurator in its own repo, or in Mk2PVRouter, checking both firmwares as submodules.

The user chose 2: each configurator must stay in sync with its own firmware, and 1 ties the two repos
together. 3 is out because a configurator must live with the firmware it writes for. The cost of 2 is
accepted: two copies that may drift, since the two firmwares may drift too.

For the receiver, the user chose to copy `RemoteLoadReceiver/` into PVRouter-1-phase. The
alternatives were a test snapshot of its settings files, or no receiver output with a link to the
3-phase page.

## Design

### Repository layout (PVRouter-1-phase)

- `RemoteLoadReceiver/`: the 3-phase repo's folder at its `dev` head, unchanged (sketch, settings,
  platformio.ini, README).
  - Where needed, the README's references to the 3-phase router are reworded to fit both routers.
  - The single-phase READMEs link to the 3-phase repo's receiver today; they now link to the local
    folder, and the CHANGELOG says so.
  - CI builds the receiver: the 3-phase job that does it is copied.
- `configurator/`: the 3-phase `configurator/` at its `dev` head, then adapted.
  - Same files and roles: `model.js`, `validate.js`, `timeline.js`, `generate.js`, `yaml.js`,
    `i18n.js`, `zip.js`, `folder.js`, `app.js`, `index.html`, `style.css`, `test/`.
  - `yaml.js` and its tests stay, unused by the page in this step, ready for the YAML step.
  - `check-yaml.sh` is not wired into CI yet.

### Settings (`model.js`)

The single-phase defaults are the shipped files:

- one local load on D5, priorities `[0]`;
- `NO_OF_PHASES` 1 and `WORKING_ZONE_IN_JOULES` 360: constants written by `generate.js`, not
  settings;
- `pcbVersion`: `OLD_SEG`, `OLD_OLED`, `NEW` (default `NEW`);
- new settings:
  - `divertedCtPresent: true`;
  - `antiCreepLimit: 5` (W, 0..255);
  - `displayShutdownInHours: 8` (1..255);
  - `displayType: 'NONE'` (`NONE`, `SEG`, `SEG_HW`, `OLED`).

Pins taken by the display, from the single-phase `utils_display.h`. They are removed from the pins
offered, as the RF pins are.

- SEG: D2, D5 to D13, A0, A2;
- SEG_HW: D5 to D9, A0 to A2;
- OLED: A4, A5.

Temperature pin per board: taken from the single-phase `config.h` and `validation.h`. Expected: NEW
keeps D3 with its pull-up; the old boards take a free pin.

**mk2Wifi.** The module plugs into the universal board (NEW), which is the same board in both
routers: same jumpers, same `MK2WIFI_GPIO` map.

- Its settings are offered with NEW only, and hidden and refused with OLD_SEG and OLD_OLED: the
  `wifi` control inputs, temperature mode `esp`, the jumper list.
- Until the YAML step, a line next to them says the YAML comes next. The firmware files are complete
  without it.
- Both steps are meant to reach `dev` before a release, so the published page (from `main`) never
  shows the gap.

**Calibration helper.** It uses the single-phase `calibration.h` names, and asks for grid power,
voltage, and diverted power with CT2.

| Board | Values |
| --- | --- |
| OLD_SEG, OLD_OLED | `f_powerCalOldBoard`, `f_powerCalDivertedOldBoard`, `f_voltageCalOldBoard` |
| NEW | `f_ctRating`, `f_ctRatingDiverted`, `f_powerCalPerAmp`, `f_voltageCalNewBoard` |

- Old boards: the diverted reading corrects `f_powerCalDivertedOldBoard`.
- NEW: the diverted reading corrects `f_ctRatingDiverted` (a later decision of the user, as step 4
  of the single-phase Readme's calibration says): new = old × kD / kGrid, with kD = meterD / routerD
  and kGrid = meterP / routerP. CT2 has no factor of its own there
  (`f_powerCalDiverted = f_powerCalPerAmp * f_ctRatingDiverted`, in `config_derived.h`), so
  dividing by kGrid compensates the grid correction already applied to `f_powerCalPerAmp`. The
  page also shows the error left, and above a few percent it suggests checking
  `f_ctRatingDiverted`. Without a diverted reading the rating stays as it is.

### Checks (`validate.js`)

The 3-phase checks that also exist in the single-phase `validation.h` stay. The single-phase ones are
added:

- display pins against every other pin;
- SEG and SEG_HW only on OLD_SEG; OLED not on OLD_SEG;
- SEG with RF (D2, D10 to D13);
- `DISPLAY_SHUTDOWN_IN_HOURS` > 0;
- the CT2 ranges.

The plan diffs the two `validation.h`: checks the single-phase one lacks are dropped, and the
reverse. Each error is tested against its `static_assert` message, as on the 3-phase side.

### Files (`generate.js`)

- `config.h`, `config_system.h`, `config_rf.h` as the single-phase shipped files, under
  `Mk2_1phase_RFdatalog_temp/`.
- The receiver files as today, against the local `RemoteLoadReceiver/`.
- `folder.js` looks for the single-phase sketch folder.

### Page and the switch

- The title and texts say single-phase.
- At the top, "Router: **single-phase** / 3-phase": the other entry links to the 3-phase page. The
  3-phase page gets the mirror link, in a small PVRouter-3-phase PR.
- The links are absolute, to the documentation site, so they also work offline and from each repo's
  GitHub Pages preview:
  - 3-phase: `https://fredm67.github.io/Mk2PVRouter/configurateur/`;
  - single-phase: `…/configurateur-monophase/`.
- Saves get a `router` field: `1` here, `3` on the 3-phase side; a save without it is a 3-phase one.
  A save from the other router is refused, with a link to its page. Each side keeps its own
  `formatVersion`.

### Publishing

- Mk2PVRouter `content/conf.py`: `copy_configurator` copies `3-phase/configurator` to
  `/configurateur/` today, and that address stays. It also copies `1-phase/configurator` to
  `/configurateur-monophase/`.
- A single-phase configurator page in the docs, next to `configurateur-triphase.rst`.
- PVRouter-1-phase GitHub Pages: the configurator as a preview, as the 3-phase pages do.

### CI and tests (PVRouter-1-phase)

- A `Configurator` job, copied from the 3-phase one without the ESPHome steps:
  - `node --test configurator/test/*.test.js`;
  - clang-format;
  - `compile-presets.sh` against `Mk2_1phase_RFdatalog_temp` and `RemoteLoadReceiver`.
- A receiver build job.
- Presets:
  - defaults;
  - OLD_SEG with SEG_HW and two loads;
  - OLD_OLED with OLED and temperature;
  - NEW with remote units, relays and dual tariff;
  - NEW with mk2Wifi inputs;
  - 60 Hz.

### Order of the work

1. PVRouter-1-phase: `RemoteLoadReceiver/` and its CI, then `configurator/` with its tests, presets,
   CI, README (both languages) and CHANGELOG.
2. PVRouter-3-phase: the switch link and the `router` save check.
3. Mk2PVRouter: publishing and the docs page, once both are on `main`.

## Left to the builder (the plan)

- The exact edits to the copied `generate.js`, `app.js` and `i18n.js`. The shipped files are the
  reference: a failing byte-for-byte test points at the line.
- The temperature-pin rule per board, confirmed against the single-phase code.
- Which lines of the receiver README need rewording.
