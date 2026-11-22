# Single-phase firmware at parity with the 3-phase router: design

Date: 2026-11-22. Repository: PVRouter-1-phase. Reference: PVRouter-3-phase, firmware
`Mk2_3phase_RFdatalog_temp` on `dev` once PR #241 (rotation groups) is merged.

## Intent

The universal mainboard (PVRouter-KiCad, `mainboard/`, rev. 6.0) serves single-phase and 3-phase
sites. A single-phase user should get everything a 3-phase user gets: the same settings, the same
features, the same checks at compile time, the same tests. On top of that, single phase can do
what the 3-phase router cannot: measure the diverted energy with a second CT. The 3-phase router
uses all six ADC inputs for its three voltages and three currents; single phase needs three
(voltage, grid current, diverted current).

The single-phase firmware stays a firmware of its own (not one firmware for both): mixing single
and 3-phase, old and new boards in one code base would make it too complex. Sharing code between
the two firmwares is deferred (see Non-goals).

## Goals

1. **Feature parity** with the 3-phase firmware, as it is on `dev` after #241:
   - the settings-only `config.h` (`config_types.h`, `config_derived.h`), with target codes
     (`LOAD()`, `RELAY()`, `ALL_...()`, `REMOTE_LOAD()`) and the load map;
   - dual tariff with scheduled boosts (`scheduledBoosts`) and the boost temperature threshold;
   - priority rotation (OFF / AUTO / PIN) with rotation groups, the pin working with dual tariff;
   - relay diversion (relay engine, EWMA filter, minimum ON/OFF times, battery mode);
   - override pins with targets, routing stop (diversion pin), router OFF pin;
   - remote loads over RF (RFM69), with remote units running `RemoteLoadReceiver`;
   - DS18B20 temperature sensors;
   - watchdog LED;
   - serial output: HumanReadable, IoT (mk2Wifi / ESPHome), JSON; the startup summary;
   - every compile-time check of `validation.h` that applies to one phase.
2. **Diverted energy** measured on CT2: diverted power, energy in Wh with Robin's anti-creep limit,
   reported on every serial output and on the display.
3. **Both boards:**
   - **old single-phase Mk2 board** (`PcbVersion::OLD`): voltage on A3, CT1 (grid) on A5, CT2
     (diverted) on A4, ADC reference AVcc, 4-digit 7-segment display, with or without the
     pin-saving logic chips;
   - **universal board** (`PcbVersion::NEW`) in single-phase mode (JP1/JP2 on I2C): voltage on A0
     (VT1), CT1 (grid) on A1, CT2 (diverted) on A3 (the L2 current input), internal 1.1 V
     reference, OLED on I2C (A4/A5).
4. **Robin's single-phase control core**, the one of `Mk2_fasterControl_twoLoads_5` (July 2023),
   whose multi-load logic comes from `Mk2_multiLoad_wired_7a` (see "Control core").
5. **The CT1 filter** (`lpf_gain`, `alpha`) set from measurements, per CT type, for both firmwares,
   with a recording sketch (see "CT recording tool").
6. **Tests** as for the 3-phase firmware: native tests, compile-fail cases, grid simulator
   scenarios with expected results, CI. Real-board tests by testers with an old board and a
   universal board.

## Non-goals

- **Sharing code between the two firmwares.** Set aside for now (options discussed: one repository
  with shared files copied by a script, submodule, hand copies). Nothing in this design may make it
  harder later: the features keep the 3-phase file names and interfaces.
- **The configurator and the ESPHome packages for single phase:** a second stage, after the
  firmware works.
- **Comparing the new firmware with Robin's sketch in the simulator.** The new firmware gets its
  own scenarios with expected results, as the 3-phase one; real boards validate it.
- **Split-phase.**
- **The older sketches** (`Mk2_fasterControl_twoLoads_temp_1`, `Mk2_fasterControl_threeLoads_temp_1`,
  `Mk2_fasterControl_Full`) are not changed. They stay as they are; the README points to the new
  firmware. Note: the two single-file sketches still carry Robin's version 4 two-load logic that he
  archived as problematic; the README says so.

## Anti-goals

- The control behaviour regresses against `twoLoads_5`: the version 4 logic (thresholds moved on
  the measured energy, decisions taken on the predicted one) must not come back.
- A setting with the same name as in the 3-phase firmware but another meaning. Same names, same
  meaning, same syntax, so a user moving between the two, and later the configurator, find the
  same settings.
- A firmware that does not fit the ATmega328P (32 KB flash, 2 KB RAM) with every feature on.
- Anything that makes the diversion depend on the main loop: the time-critical code stays in the
  ADC interrupt, as in both current firmwares.

## Constraints

- ATmega328P at 16 MHz, avr-gcc 7.3, C++17, Arduino IDE and PlatformIO; the sketch folder holds
  every file the Arduino IDE compiles.
- Free-running ADC (auto-trigger), as the 3-phase firmware: an `ADMUX` written in the interrupt
  applies to the conversion after the one already running.
- RF: RFM69 with the LowPowerLab library only (the RFM12B / JeeLib of the old sketches is not
  supported). On the old board, the RF module needs D2 and D10–D13, which the display without
  pin-saving chips also uses: the pin checks of `validation.h` refuse such a combination.
- Licence and repository files as in the 3-phase repository: the whole repository moves to
  GPL-3.0-or-later, the older sketches and `dev/` tools included (NOTICE with the origins of the
  code, SPDX tags, "your settings files are yours"). The user said: set everything like in the
  3-phase.

## Approaches

**A. Start from today's 3-phase firmware and make it single-phase (chosen).** Copy the 3-phase
firmware and its tooling, then replace the sampling and the control core with the single-phase
ones, add CT2 and the displays. Everything to match comes over exactly as it is; the work is in a
small, well-understood part.

**B. Start from `Mk2_fasterControl_Full` and port each 3-phase change.** The core is already right,
but two years of 3-phase changes would be ported one by one (config split, scheduled boosts,
target codes, rotation groups, remote loads, relay fixes, override targets, test tools), each a
chance to drift. Rejected.

**C. Write it fresh.** Rejected: throws away tested code on both sides.

## Design

### Repository layout

- A new firmware folder in PVRouter-1-phase, `Mk2_1phase_RFdatalog_temp/` (my call: the name
  mirrors `Mk2_3phase_RFdatalog_temp`, so the scripts and docs translate one to one), copied from
  the 3-phase firmware at the `dev` commit that holds #241.
- Copied and adapted from the 3-phase repository: `test/native` (as the firmware's `test/`),
  `scripts/ci/compile-fail*`, `sim/` (grid simulator), `.pre-commit-config.yaml`, the CI
  workflows, `NOTICE`, the licence and the SPDX headers. The CHANGELOG starts with this firmware.
- `RemoteLoadReceiver` is not copied: single-phase routers use the 3-phase repository's receiver,
  which the README links to (the RF frame is the same).

### Boards and sampling

- `PCB_VERSION` (`PcbVersion::OLD` / `NEW`), as in the 3-phase firmware, selects the analog pins
  and the ADC reference:

  | | Voltage | CT1, grid | CT2, diverted | Reference |
  |---|---|---|---|---|
  | `OLD` (Mk2 single-phase board) | A3 | A5 | A4 | AVcc |
  | `NEW` (universal board, JP1/JP2 on I2C) | A0 | A1 | A3 | internal 1.1 V |

- Three channels in a fixed chain, sampled continuously: each channel about every 312 µs (3 ×
  ~104 µs). The chain takes the `ADMUX` pipeline into account so that the real order is the
  intended one; a simulator check pins it, as #239 does for the 3-phase firmware (`EXPECT_ADMUX`).
- CT2 is always sampled, so the timing never depends on the settings. A setting says whether a CT2
  is fitted (my call: `DIVERTED_CT_PRESENT`, default `true` on both boards); without it, nothing
  about diverted energy is reported or displayed.
- On the universal board, the CT2 input is complete in single-phase mode: every SMD part is
  soldered at the factory, and JP1/JP2 only cut A4/A5 off the L3 sensing lines for the OLED
  (confirmed by the hardware owner). The KiCad readme's "only CT1 is connected" is to be corrected
  in PVRouter-KiCad.

### Control core

Robin's `twoLoads_5` logic, already in `Mk2_fasterControl_Full/processing.cpp`:

- the energy bucket of 0.1 Wh (`WORKING_RANGE_IN_JOULES 360`);
- at the first sample of the negative half cycle, the predicted energy at the end of the cycle
  (energy at its start plus the average power of the first half);
- at the 3rd sample of the negative half cycle, the decision: above the middle of the bucket a
  load may be added, below it one may be removed, the opposite threshold reset to its default;
  after a switch, only the load just switched may change during the post-transition period, and
  the threshold follows the prediction;
- loads switched at the positive zero crossing;
- the logical-to-physical mapping goes through the load priorities, so rotation groups apply as in
  the 3-phase firmware (`Rotation::Table`);
- relays decide on the EWMA of the grid power, as in the 3-phase firmware; remote loads receive
  their state by RF as in the 3-phase firmware, once per decision.

### CT1 filter

- `lpf_gain` and `alpha` in `calibration.h`, per board, as in Robin's sketches (an IIR low-pass
  filter of the grid current added back with a gain, to offset the CT acting as a high-pass
  filter).
- Their defaults come from the CT recording tool. Until a CT type is measured (my call): on the
  old board with the SCT-013-000, the values of `Mk2_fasterControl_Full/calibration.h`
  (`lpf_gain` 9, `alpha` 0.0011, set in Dec. 2024 on real hardware at the same ~312 µs spacing);
  off (`lpf_gain` 0) on the universal board, whose voltage-output CTs behave differently. The
  README says which values are measured.

### Diverted energy

- Power on CT2 per mains cycle, energy accumulated in Wh with Robin's anti-creep limit
  (`ANTI_CREEP_LIMIT`, joules per mains cycle), reset after `DISPLAY_SHUTDOWN_IN_HOURS` without
  diversion, as in Robin's sketches.
- Calibration: on the old board, `powerCal_diverted` as `f_powerCal` today; on the universal board,
  the CT rating of CT2 times the per-amp value, as the 3-phase new-board calibration.
- What counts: whatever CT2 measures (my call). The README tells to pass the wires of every local
  output, TRIAC and relay, through CT2. Remote loads are never counted, nor estimated. The
  simulator model has CT2 measure every local load and relay.
- Reported: HumanReadable (datalog line), JSON, IoT. In the IoT frames, two new labels (my call):
  `PD`, diverted power in W, and `ED`, diverted energy in Wh. The ESPHome `mk2pvrouter` component
  (esphome_components repo) publishes only the labels that have a sensor and ignores the others,
  with a 1048-byte frame buffer, so existing mk2Wifi installs are not affected; their sensors come
  with the second stage.

### Displays

- Old board: the 4-digit 7-segment display, with or without the pin-saving chips, from
  `Mk2_fasterControl_Full/utils_display.h`.
- Universal board: the OLED on I2C, from `Mk2_fasterControl_Full/utils_oled.h`.
- Content as `Full`: diverted energy and temperature, in turn.
- `TYPE_OF_DISPLAY` selects none, 7-segment (with or without pin-saving chips) or OLED; the pins
  the display takes enter the pin checks.

### Settings

- The 3-phase `config.h`, settings and comments, minus what only 3 phases need, plus:
  `TYPE_OF_DISPLAY`, `DIVERTED_CT_PRESENT`, the CT2 calibration, the filter values.
- Per-phase arrays become single values where the setting is per phase (calibration, phase
  calibration); the names stay those of the 3-phase firmware.

### Tests

- Native tests of the 3-phase firmware that apply to one phase, kept and adapted; new ones for
  the single-phase core (prediction, decision, post-transition) and the diverted energy.
- Compile-fail cases adapted; new ones for the display pins and `DIVERTED_CT_PRESENT`.
- Grid simulator: a model of each board (pins, reference, CT2 measuring the diverted load), the
  ADC pipeline check, scenarios with expected results (surplus steps, clouds, no surplus, relays,
  boosts, rotation, remote loads).
- CI as in the 3-phase repository.
- Real boards: testers with an old board and a universal board run the firmware before release.

### CT recording tool (both firmwares)

- A sketch modelled on Robin's `RST_375us_dev` (archive: `IoT_old_laptop/Robin`), in `dev/` of
  each repository, one version per firmware.
- It samples exactly as the firmware: free-running ADC, the same channel chain and reference
  (single phase: ~312 µs per CT; 3-phase: ~624 µs per CT, six channels).
- It switches a load on for a set time (one half cycle, then several cycles), records raw samples
  of the voltage and of each CT around the step, and prints them as CSV.
- A script fits the CT's time constant from the recording and gives `alpha` and `lpf_gain` for
  that CT type at that sample spacing.

## Build order

1. Firmware: copy, sampling and boards, control core, diverted energy, displays, settings,
   validation, native and compile-fail tests, CI. Simulator board models and scenarios.
2. CT recording tool, both firmwares; filter defaults from the measurements. Open: which CT
   types are measured (at least the SCT-013-000 and the universal board's default
   voltage-output CT) and who records them; it sets when this step is done, not stage 1.
3. Real-board tests by the testers.
4. Second stage: configurator single-phase mode, ESPHome packages, docs.

## Left to the builder

- The split into files where the 3-phase layout does not fit one phase.
- The order of the porting steps within stage 1, and the plan's task boundaries.
- Test names, the exact simulator model, the CSV format of the recording tool and the fitting
  script.
- The flash and RAM budget work if everything does not fit (for instance the U8g2 page mode or
  U8x8 text mode for the OLED); the anti-goal stands.
