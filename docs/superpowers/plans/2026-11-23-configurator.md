# Single-phase configurator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** a web configurator in PVRouter-1-phase that writes the single-phase router's config files
(and the remote units'). It is checked byte for byte against the shipped files and built in CI. A
switch link joins it to the 3-phase page, and the documentation site publishes both.

**Architecture:** the 3-phase `configurator/` and `RemoteLoadReceiver/` are copied from
PVRouter-3-phase `origin/dev` and adapted to `Mk2_1phase_RFdatalog_temp`. They stay plain scripts with
no build step, and are tested only against this repo's files. Two small companion PRs follow, in
PVRouter-3-phase and in Mk2PVRouter.

**Tech Stack:** plain JavaScript (browser and Node, `node --test`), Bash, PlatformIO, clang-format
18.1.8, GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-11-23-configurator-design.md`

## Global Constraints

- **Byte for byte:** `node configurator/test/generate-preset.js` with the defaults gives back the shipped
  `Mk2_1phase_RFdatalog_temp/config.h`, `config_system.h` and `config_rf.h`, and
  `RemoteLoadReceiver/config.h` and `config_rf.h`, byte for byte, `@date` aside.
- **No dependency between the repos:** nothing in this repo's code or CI reads PVRouter-3-phase. The
  copy is taken once, by hand, from `origin/dev` of PVRouter-3-phase (at or after `20341e2c`).
- **The code itself:** plain scripts, no build step, no npm dependency. Every script loads as a classic
  `<script>` and as a Node module (the existing UMD wrapper).
- **The generated files:** clang-format clean (18.1.8, the repo's `.clang-format`) and building.
- **Texts:** in English and in French (`i18n.js`). French typography: U+00A0 before `: ; ! ?` and
  inside « ».
- **Single-phase settings and defaults:**
  - boards `OLD_SEG`, `OLD_OLED`, `NEW` (default `NEW`);
  - `divertedCtPresent: true`, `antiCreepLimit: 5` (0..255), `displayShutdownInHours: 8` (1..255);
  - `displayType: 'NONE'` (`NONE`, `SEG`, `SEG_HW`, `OLED`);
  - written as constants: `NO_OF_PHASES` 1 and `WORKING_ZONE_IN_JOULES` 360.
- **Display pins:**
  - SEG: D2, D5 to D13, A0, A2;
  - SEG_HW: D5 to D9, A0 to A2;
  - OLED: A4, A5.
- **mk2Wifi:** its settings are offered with `NEW` only.
- **No YAML yet:** no YAML file is written or offered in this step. `yaml.js` and its tests stay in the
  repo.
- **Switch links:**
  - 3-phase: `https://fredm67.github.io/Mk2PVRouter/configurateur/`;
  - single-phase: `https://fredm67.github.io/Mk2PVRouter/configurateur-monophase/`.
- **Saves:** a `router` field, `1` on this page and `3` on the 3-phase page. A save without it is a
  3-phase save.
- **Commits:**
  - end with `Co-Authored-By: Claude <noreply@anthropic.com>`;
  - never commit `*/version.h` or `*/.vscode/extensions.json`;
  - new `.sh` files get `git update-index --chmod=+x`.
- **Pre-commit hooks:** on "Permission denied" or "resource busy", wait about 20 s, re-stage and retry.

## Review Focus

1. **A save from an older page or from the other router.**
   - A save without `router`, or with `router: 3`: refused with a link to the 3-phase page, never
     half-loaded.
   - A single-phase save from this version, missing a newer field: loads with the defaults filled in
     (`normalize`).
   - Owner: Task 6.
2. **Changing the board after choosing a display or mk2Wifi inputs.**
   - SEG on OLD_SEG, then NEW: an error points at the display, not a crash or silent output.
   - Inputs on mk2Wifi pins, then an old board: the wifi flags are refused.
   - Owner: Task 3.
3. **The display taking pins that loads or relays already use.**
   - The SEG display with a load on D5: an error on the conflicting pin, matching
     `validation.h`'s "Duplicate pin" refusal.
   - Owner: Task 3.
4. **Calibration with CT2 absent.**
   - No diverted reading is asked.
   - No `f_powerCalDiverted…` line is written.
   - Owner: Task 5.
5. **The "save into the firmware folder" pick of the wrong folder.**
   - The 3-phase sketch folder (`Mk2_3phase_RFdatalog_temp`) is not recognized as the router folder.
   - Owner: Task 2.

---

### Task 1: RemoteLoadReceiver in this repo

**Files:**
- Create: `RemoteLoadReceiver/`, a copy of PVRouter-3-phase `origin/dev:RemoteLoadReceiver/`
  (`.gitignore`, `.vscode/` without `extensions.json` if it is in `.gitignore` here, `README.md`,
  `RemoteLoadReceiver.ino`, `config.h`, `config_rf.h`, `platformio.ini`, `receiver.cpp`,
  `utils_bits.h`, `utils_pins.h`)
- Modify:
  - `.github/workflows/build.yml`: a `remote-load-receiver` job, copied from the 3-phase one, with
    `Mk2_1phase_RFdatalog_temp` in its "copies match" step;
  - `Mk2_1phase_RFdatalog_temp/Readme.md` and `Readme.en.md`: the receiver links point to
    `../RemoteLoadReceiver/`;
  - `CHANGELOG.md`.

**Interfaces:**
- Produces: `RemoteLoadReceiver/config.h` and `config_rf.h`. Task 2 compares the generated receiver
  files with them.

- [ ] **Step 1: Copy the folder.** Use `git -C D:/Git/IoT/PVRouter-3-phase archive origin/dev
  RemoteLoadReceiver | tar -x -C <worktree>`.
- [ ] **Step 2: Compare the shared helpers.** Run
  `diff <(grep -v '@date' Mk2_1phase_RFdatalog_temp/utils_pins.h) <(grep -v '@date' RemoteLoadReceiver/utils_pins.h)`,
  and the same for `utils_bits.h`. Expected: no difference.
  - If a file differs, or is missing in `Mk2_1phase_RFdatalog_temp`, the receiver keeps its copy.
    Drop that file from the CI check and record a ruling.
- [ ] **Step 3: Build.** Run `pio run -d RemoteLoadReceiver`. Expected: SUCCESS.
- [ ] **Step 4: Reword the README.**
  - In `RemoteLoadReceiver/README.md`, reword the phrases that assume the 3-phase router so they fit
    both routers.
  - Change the links in the two single-phase READMEs (the lines that cite
    `PVRouter-3-phase/tree/main/RemoteLoadReceiver`).
  - Keep U+00A0 in the French text.
- [ ] **Step 5: Add the CI job and the CHANGELOG line.**
  - Add the job to `build.yml`, then parse it with `python -c "import yaml,sys;yaml.safe_load(open(sys.argv[1]))" .github/workflows/build.yml`.
  - CHANGELOG `[Unreleased]`: "RemoteLoadReceiver is now in this repository".
- [ ] **Step 6: Commit** `feat: RemoteLoadReceiver in this repository, built in CI`.

### Task 2: The configurator copy gives back the single-phase files

**Files:**
- Create: `configurator/`, a copy of PVRouter-3-phase `origin/dev:configurator/` (all files, `test/`
  and `test/presets/` included).
- Modify, in the copy:
  - `model.js`, `generate.js`, `folder.js`, `app.js`;
  - `test/generate.test.js`, `test/folder.test.js`, `test/compile-presets.sh`, `README.md`.

**Interfaces:**
- Produces:
  - `M.defaults()`, the single-phase defaults, with the fields of the Global Constraints;
  - `M.PCB_VERSIONS = ['OLD_SEG', 'OLD_OLED', 'NEW']`, `M.DISPLAY_TYPES = ['NONE', 'SEG', 'SEG_HW', 'OLED']`;
  - `G.configH(m, date)`, `G.configSystemH(m, date)`, `G.configRfH(m, date)` and `G.files(m, date)`,
    with paths under `Mk2_1phase_RFdatalog_temp/`;
  - folder.js `ROUTER = 'Mk2_1phase_RFdatalog_temp'`.

- [ ] **Step 1: Copy and point the tests at this repo.**
  - Copy the folder (`git archive`, as in Task 1).
  - Replace every `Mk2_3phase_RFdatalog_temp` with `Mk2_1phase_RFdatalog_temp` in `test/` and
    `compile-presets.sh`.
  - Delete the presets that need the 3-phase firmware or the YAML: keep only `default.json`.
    Task 7 adds the single-phase ones.
  - Leave `test/yaml.test.js` and `check-yaml.sh` as they are.
- [ ] **Step 2: Run the tests and check they fail.** Run `node --test configurator/test/*.test.js`.
  Expected: FAIL in `defaults give back config.h` and `defaults give back config_system.h`. Those diffs
  are the to-do list.
- [ ] **Step 3: Make `model.js` and `generate.js` match.**
  - In `model.js`:
    - the defaults: one local load on D5, priorities `[0]`;
    - the new fields;
    - `PCB_VERSIONS` and `DISPLAY_TYPES`;
    - drop `PHASES` and use 1 where it was used;
    - the header comment names `Mk2_1phase_RFdatalog_temp`.
  - In `generate.js`, write exactly the shipped text:
    - the board comment block;
    - the load-map comment;
    - the CT2 section and the display section (with `DisplayType::${m.displayType}`);
    - the mk2Wifi pin comment;
    - the `dualTariffPin` comment;
    - `NO_OF_PHASES{ 1 }` and `WORKING_ZONE_IN_JOULES{ 360UL }` with their comments.
  - Where the load-map comment depends on the number of loads, follow the shipped wording for the
    default, and the 3-phase wording otherwise.
- [ ] **Step 4: Fix the paths.** In `generate.js` `files()`, `folder.js` (`ROUTER`) and `app.js`
  (the "where" list and the zip names), the paths become `Mk2_1phase_RFdatalog_temp`.
  - Add a folder test: a picked folder that holds `Mk2_3phase_RFdatalog_temp/` and no
    `Mk2_1phase_RFdatalog_temp/` is not taken as the router folder. Assert what `folder.js` returns
    for an unrecognized folder today.
- [ ] **Step 5: Run the tests and check they pass.** Run `node --test configurator/test/*.test.js`.
  Expected: all pass, the receiver byte-for-byte tests included (against Task 1's copy).
  - If an existing test asserts 3-phase content (`P1..P3`, three loads), change it to the
    single-phase equivalent.
  - Record any test removed, with a ruling.
- [ ] **Step 6: Build the default preset.** Run `bash configurator/test/compile-presets.sh`. Expected:
  `default` passes, with clang-format on PATH.
- [ ] **Step 7: Commit** `feat: configurator, copied from PVRouter-3-phase; its defaults give back
  the single-phase config files`.

### Task 3: Boards, display, temperature pin and mk2Wifi

**Files:**
- Modify: `configurator/model.js`, `validate.js`, `generate.js`, `app.js`, `i18n.js`.
- Test: `configurator/test/validate.test.js`, `generate.test.js`.

**Interfaces:**
- Consumes: Task 2's fields.
- Produces:
  - `M.displayPins(m) -> number[]`, the pins taken by the display, with A0..A5 as 14..19;
  - `M.mk2wifiAvailable(m) -> boolean`, `m.pcbVersion === 'NEW'`;
  - `M.tempPin(m)`: `TEMP_PIN` (3) on `NEW`, `m.temperature.pin` on the old boards (confirm against
    the single-phase `config.h` pinout notes);
  - error keys `err.displayBoard`, `err.oledBoard`, `err.displayRf`, `err.displayPin`,
    `err.mk2wifiBoard`.

- [ ] **Step 1: Write the failing tests** in `validate.test.js`, each asserting the error key and the
  field it points at:
  - SEG on NEW → `err.displayBoard`;
  - SEG_HW on OLD_OLED → `err.displayBoard`;
  - OLED on OLD_SEG → `err.oledBoard`;
  - SEG on OLD_SEG with RF logging → `err.displayRf`;
  - SEG_HW on OLD_SEG with RF → no `err.displayRf`;
  - SEG on OLD_SEG with the default load on D5 → `err.displayPin` on that load;
  - SEG_HW with a relay on D9 → `err.displayPin`;
  - SEG_HW with a load on D4 → valid;
  - mk2Wifi enabled on OLD_OLED → `err.mk2wifiBoard`;
  - OLD_SEG, temperature by the router on pin 4 → valid;
  - OLD_SEG, temperature on pin 5 with SEG → `err.displayPin` (or the existing temperature pin error,
    whichever matches `validation.h`'s message).

  In `generate.test.js`: OLD_SEG and SEG_HW write `PcbVersion::OLD_SEG` and `DisplayType::SEG_HW`.
- [ ] **Step 2: Run the tests and check they fail.** Run `node --test configurator/test/validate.test.js configurator/test/generate.test.js`.
  Expected: the new tests FAIL.
- [ ] **Step 3: Implement the model and the checks.**
  - The model helpers.
  - The checks, mirroring the single-phase `validation.h` static_asserts on display and board, and the
    `displayPinMask` part of `check_pins()`.
  - Remove the display pins from the pins the page offers, as `RF_PINS` are.
  - Diff the 3-phase and the single-phase `validation.h`: drop the copied checks the single-phase one
    lacks (the CtFilter `lpf_gain`/`alpha` check is one), and record each drop as a ruling.
- [ ] **Step 4: Update the page.**
  - The display select next to the board, offering only the types valid for the chosen board.
  - The mk2Wifi section shown with `NEW` only.
  - The board's help text lists the three boards.
  - EN and FR texts for every new key.
- [ ] **Step 5: Run the tests and check they pass.** Run `node --test configurator/test/*.test.js`.
  Expected: all pass.
- [ ] **Step 6: Commit** `feat(configurator): the three single-phase boards, the display and their
  pins`.

### Task 4: CT2, the diverted energy

**Files:**
- Modify: `configurator/validate.js`, `generate.js`, `app.js`, `i18n.js`.
- Test: `configurator/test/validate.test.js`, `generate.test.js`.

**Interfaces:**
- Produces: error keys `err.antiCreep` (not an integer 0..255) and `err.displayShutdown` (not an
  integer 1..255).

- [ ] **Step 1: Write the failing tests.**
  - `antiCreepLimit` 256 → `err.antiCreep`; −1 → `err.antiCreep`; 0 → valid.
  - `displayShutdownInHours` 0 → `err.displayShutdown`; 255 → valid.
  - Generate: `divertedCtPresent: false` and `antiCreepLimit: 0` write `DIVERTED_CT_PRESENT{ false }`
    and `ANTI_CREEP_LIMIT{ 0 }`, the rest of the section unchanged.
- [ ] **Step 2: Run the tests and check they fail.** Run `node --test configurator/test/*.test.js`.
  Expected: the new tests FAIL.
- [ ] **Step 3: Implement the checks and the CT2 section of the page.**
  - Place the section after the feature toggles, as in `config.h`.
  - Without CT2, the anti-creep field is hidden. The display shutdown field stays: the display shows
    temperatures too.
  - EN and FR texts.
- [ ] **Step 4: Run the tests and check they pass.** Expected: all pass.
- [ ] **Step 5: Commit** `feat(configurator): CT2, the diverted energy`.

### Task 5: Calibration helper on the single-phase calibration.h

**Files:**
- Modify: `configurator/model.js` (`CAL_VALUES`, `CAL_NAMES`, `CT_RATING`, `calibrationArray` →
  `calibrationValue`, `parseCalibration`, `calibrationResult`, `ctRatingUsed`), `generate.js`
  (`calibrationLines`, `calibrationH`), `app.js`, `i18n.js`.
- Test: the calibration tests in `configurator/test/generate.test.js` (and `validate.test.js` if they
  live there).

**Interfaces:**
- Produces:
  - `M.calibrationValue(text, name) -> number | null`: the active `inline constexpr float name{ v }`
    of `calibration.h`;
  - readings `m.calibration.power`, `voltage`, `diverted`, each `{ router, meter }`.

- [ ] **Step 1: Write the failing tests.** Use the shipped single-phase `calibration.h` (read from
  `Mk2_1phase_RFdatalog_temp/calibration.h`):
  - `parseCalibration` returns `f_powerCalOldBoard` 0.05, `f_powerCalDivertedOldBoard` 0.05,
    `f_voltageCalOldBoard` 0.805653, `f_ctRating` 50, `f_ctRatingDiverted` 50, `f_powerCalPerAmp`
    0.00286405, `f_voltageCalNewBoard` 0.888065256.
  - OLD_OLED, power router 1000 / meter 1100: the new `f_powerCalOldBoard` is 0.05500.
  - OLD_OLED, diverted router 500 / meter 450: `f_powerCalDivertedOldBoard` 0.04500.
  - NEW, power router 1000 / meter 1100: `f_powerCalPerAmp` × 1.1.
  - NEW, diverted 500 / 450: no line changes for CT2, and `calibrationResult` reports a CT2 check
    error of −10 %.
  - `calibrationH` writes the corrected values into the loaded file and leaves every other byte
    untouched.
  - With `divertedCtPresent: false`: no diverted reading, no CT2 line.
- [ ] **Step 2: Run the tests and check they fail.** Expected: FAIL.
- [ ] **Step 3: Implement.**
  - Scalars instead of `[NO_OF_PHASES]` arrays.
  - The per-board sets above.
  - On NEW, the CT2 check message from 5 % error on: suggest checking `f_ctRatingDiverted`.
  - The page asks for P, V and, with CT2, the diverted power, on the human-readable output
    (`P:`, `V:`, `diverted <W> W`).
- [ ] **Step 4: Run the tests and check they pass.** Expected: all pass.
- [ ] **Step 5: Commit** `feat(configurator): calibration helper for the single-phase boards and CT2`.

### Task 6: The page says single-phase, the switch link, saves and no YAML

**Files:**
- Modify: `configurator/model.js` (`normalize`, `defaults`), `app.js`, `index.html`, `i18n.js`.
- Test: `configurator/test/validate.test.js` or a new `configurator/test/save.test.js`.

**Interfaces:**
- Produces:
  - `M.defaults().router === 1`;
  - `M.saveRouter(data) -> 1 | 3`, which is `data.router ?? 3`;
  - `M.normalize(data)` throws `Error('router')` when `saveRouter(data) !== 1`.

- [ ] **Step 1: Write the failing tests.**
  - `saveRouter({ formatVersion: 1 })` is 3, and `saveRouter({ router: 1 })` is 1.
  - `normalize` of a save without `router` throws `router`.
  - `normalize` of `{ ...defaults(), antiCreepLimit: undefined }` fills the default 5.
  - `G.files(defaults())` has no path ending in `.yaml`, even with `mk2wifi.enabled`.
- [ ] **Step 2: Run the tests and check they fail.** Expected: FAIL.
- [ ] **Step 3: Implement.**
  - On loading a save, `Error('router')` shows a message with a link to the 3-phase page instead of
    loading.
  - The same for `localStorage`: a stored 3-phase state is dropped for the defaults.
  - The page title and texts say single-phase.
  - At the top, "Router: **single-phase** / 3-phase", where "3-phase" is a link to the 3-phase URL.
  - The mk2Wifi section shows the "the YAML comes next" line, and no YAML download or file.
  - EN and FR texts.
- [ ] **Step 4: Run the tests and check they pass.** Expected: all pass.
- [ ] **Step 5: Look at the page.** Open `configurator/index.html` in a headless browser (Chrome
  `--headless --screenshot`, as for the 3-phase page). Check:
  - the switch;
  - the board and display;
  - CT2;
  - no YAML.
- [ ] **Step 6: Commit** `feat(configurator): single-phase page, switch to the 3-phase one, saves of
  the other router refused`.

### Task 7: Presets, CI, Pages preview and documentation

**Files:**
- Create: `configurator/test/presets/`:
  - `old-seg-hw-two-loads.json`;
  - `old-oled-display-temp.json`;
  - `new-remote-relays-dualtariff.json`;
  - `new-mk2wifi-inputs.json`;
  - `sixty-hz.json`.
- Modify:
  - `.github/workflows/build.yml`:
    - a `configurator` job, copied from the 3-phase one without the ESPHome steps;
    - its `pages` job copies `configurator/` as a preview, as the 3-phase `pages` job does;
  - `configurator/README.md`;
  - `Mk2_1phase_RFdatalog_temp/Readme.md` and `Readme.en.md`, with a link to the configurator;
  - `pages/index.html`, with a link to the preview;
  - `CHANGELOG.md`.

- [ ] **Step 1: Make the presets.** Make each preset as a save of the page (or `M.defaults()` edited
  in Node and written with `JSON.stringify(m, null, 2)`). Each one must pass `validate`.
- [ ] **Step 2: Build them.** Run `bash configurator/test/compile-presets.sh`. Expected: every preset
  builds and is clang-format clean.
  - A preset the compiler refuses while the configurator accepts it is a bug in Task 3–5's checks: fix
    the check, not the preset.
- [ ] **Step 3: Add the CI.**
  - The `configurator` job: unit tests, clang-format 18.1.8, `compile-presets.sh`.
  - The pages preview: copy the 3-phase `pages` job's configurator step.
  - Parse the YAML as in Task 1.
- [ ] **Step 4: Write the documentation.**
  - `configurator/README.md`:
    - single-phase;
    - kept in sync with `Mk2_1phase_RFdatalog_temp`;
    - copied from PVRouter-3-phase: fixes are carried by hand when they apply;
    - no YAML yet.
  - The single-phase READMEs (FR with U+00A0, EN) and the landing page link to the configurator.
  - CHANGELOG `[Unreleased]`: the configurator.
- [ ] **Step 5: Run the full checks.** Run `node --test configurator/test/*.test.js`, then
  `bash configurator/test/compile-presets.sh`. Expected: all pass.
- [ ] **Step 6: Commit** `feat(configurator): single-phase presets, CI and Pages preview`.

### Task 8 (PVRouter-3-phase, separate PR after this one): the mirror switch and the router check

**Files:**
- Modify: PVRouter-3-phase `configurator/model.js`, `app.js`, `i18n.js`, `test/`.

- [ ] **Step 1: Write the failing tests.**
  - `M.defaults().router === 3`.
  - `normalize({ ...defaults(), router: 1 })` throws `router`.
  - A save without `router` loads.
  - The shipped byte-for-byte tests still pass.
- [ ] **Step 2: Implement.**
  - "Router: single-phase / **3-phase**", where "single-phase" is a link to the single-phase URL.
  - A refused single-phase save links to it.
  - EN and FR texts.
- [ ] **Step 3: Run the checks.** Run `node --test configurator/test/*.test.js` and
  `bash configurator/test/compile-presets.sh`. Expected: pass.
- [ ] **Step 4: Commit** `feat(configurator): switch to the single-phase configurator`, in a worktree
  of PVRouter-3-phase from `origin/dev`.

### Task 9 (Mk2PVRouter, after both are on `main`): publishing

**Files:**
- Modify: `content/conf.py` (`copy_configurator`).
- Create: `content/firmware/configurateur-monophase.rst`, next to `configurateur-triphase.rst`, and
  add it to the toctree.

- [ ] **Step 1: Copy both pages.** `copy_configurator` copies `1-phase/configurator` to
  `configurateur-monophase/` as it copies `3-phase/configurator` to `configurateur/`. If a submodule
  has no `configurator/index.html`, it warns as today.
- [ ] **Step 2: Build and check.** Run `sphinx-build -b html content _build/html`. Expected: both
  `_build/html/configurateur/index.html` and `_build/html/configurateur-monophase/index.html` exist.
- [ ] **Step 3: Commit** `docs: the single-phase configurator, at /configurateur-monophase/`.
