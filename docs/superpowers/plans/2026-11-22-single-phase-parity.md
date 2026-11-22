# Single-phase firmware at parity, step 1: firmware and tests. Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A new single-phase firmware, `Mk2_1phase_RFdatalog_temp/`, with every feature of the 3-phase firmware, Robin's faster single-phase control, diverted energy on CT2 and the three single-phase boards, tested natively, by compile-fail cases and in the grid simulator.

**Architecture:** Copy the 3-phase firmware and its tooling from PVRouter-3-phase `origin/dev` at `087af072` (approach A of the spec), then replace what one phase changes: the ADC chain (3 channels instead of 6), the decision point (predicted energy at sample set index 3 (counted from 0, i.e. the 4th) of the negative half cycle, through the unchanged `LoadPriorities::Switching`), CT2 diverted energy, the boards and the displays. Internally the code keeps `NO_OF_PHASES` (= 1) and its arrays, so it stays close to the 3-phase source; the user-facing calibration settings become single values.

**Tech Stack:** ATmega328P, avr-gcc 7.3, C++17, PlatformIO (`basic`, `native` environments), Unity, simavr-based `sim/grid_sim`, bash scripts, GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-11-22-single-phase-parity-design.md`

## Global Constraints

- Firmware folder `Mk2_1phase_RFdatalog_temp/` in the PVRouter-1-phase repository; the older sketches (`Mk2_fasterControl_Full`, `Mk2_fasterControl_twoLoads_temp_1`, `Mk2_fasterControl_threeLoads_temp_1`, `Robin/`, `dev/`) are not changed, except their SPDX/licence headers (Task 9).
- Settings keep the 3-phase names, meaning and syntax ("same names, same meaning, same syntax").
- `PCB_VERSION`: `PcbVersion::OLD_SEG` (V A3, CT1 A5, CT2 A4, AVcc), `PcbVersion::OLD_OLED` (V A0, CT1 A1, CT2 A3, AVcc), `PcbVersion::NEW` (V A0, CT1 A1, CT2 A3, internal 1.1 V).
- `TYPE_OF_DISPLAY`: `DisplayType::NONE`, `SEG`, `SEG_HW` (with the pin-saving chips), `OLED`. 7-segment only on `OLD_SEG`; OLED only on `OLD_OLED` and `NEW`.
- `DIVERTED_CT_PRESENT` (`bool`, default `true`): without it nothing about diverted energy is reported or displayed; CT2 is sampled either way.
- Diverted energy = the surplus the router diverts: the mains cycles during which any local load is forced or boosted are not counted (CT2 cannot tell the loads apart); remote loads never counted.
- IoT labels: `D` diverted power in W, `E` diverted energy in Wh (the single-phase tags already documented in `teleinfo.h`; `D1`…`Dn` stay the per-load ON-time %).
- CT1 filter interim values: `lpf_gain` 9, `alpha` 0.0011 on `OLD_SEG` and `OLD_OLED`; `lpf_gain` 0 on `NEW`.
- Bucket: `WORKING_ZONE_IN_JOULES` as in the 3-phase firmware's `constants.h`/`config.h` (Robin's 360 J = 0.1 Wh range).
- Must build for the ATmega328P with every feature on (32 KB flash, 2 KB RAM).
- Time-critical code stays in `ISR(ADC_vect)`.
- RF: RFM69 / LowPowerLab only.
- Licence: the whole repository GPL-3.0-or-later, NOTICE and SPDX tags as in PVRouter-3-phase.
- Commits end with `Co-Authored-By: Claude <noreply@anthropic.com>`; never commit `version.h` changes made by builds.

Commands (from Git Bash, tools in WSL):
- Build: `MSYS_NO_PATHCONV=1 wsl bash -lc 'cd /mnt/d/Git/IoT/pvr-wt/1phase-parity/Mk2_1phase_RFdatalog_temp && ~/.platformio/penv/bin/pio run -e basic'`
- Native tests: same with `pio test -e native`
- Compile-fail: `MSYS_NO_PATHCONV=1 wsl bash -lc 'cd /mnt/d/Git/IoT/pvr-wt/1phase-parity && PATH=$HOME/.platformio/penv/bin:$PATH bash scripts/ci/compile-fail.sh'` (write it to a script file if `$HOME` vanishes, see the repository memory notes)
- Simulator: `make -C sim check` (and the per-board targets of Task 8), from WSL, after building the firmware as `sim/README.md` says.

## Review Focus

1. **The real ADC order on each board.** With the free-running ADC, an `ADMUX` write applies to the conversion after next; a chain written in the "obvious" order files samples under the wrong channel. Expected: V → CT1 → CT2 on all three boards. Pinned by `test_adc_chain` (Task 2) and the simulator's `misfiled_samples 0` / `admux_delay_min 128` expectations (Task 8).
2. **Prediction and thresholds on the same value.** Robin's version 4 moved the thresholds on the measured energy and decided on the predicted one. Expected: one call per cycle, `onNewCycle(priorities, prediction, bucketLevels)`, nothing else touching the thresholds. Pinned by `test_decision_uses_prediction_only` (Task 3).
3. **A half cycle with no or few samples** (start-up, a missing or distorted mains wave): the prediction divides by the sample count. Expected: no division by zero, a prediction equal to the bucket level when there are no samples. Pinned by `test_prediction_without_samples` (Task 3).
4. **Board, display and RF pin conflicts.** The 7-segment display without pin-saving chips uses D2 and D10–D13, which the RFM69 also needs; the OLED needs A4/A5, which `OLD_SEG` uses as analog inputs. Expected: refused at compile time with a clear message. Pinned by compile-fail cases 70–73 (Task 7).
5. **No CT2 fitted.** An unconnected CT2 input reads noise. Expected: with `DIVERTED_CT_PRESENT` false, no `D`/`E` labels, no diverted line in the datalog, nothing on the display; with it true, noise below the anti-creep limit adds nothing. Pinned by `test_anti_creep_drops_noise` (Task 4) and `test_no_diverted_labels_without_ct2` (Task 5).

---

## File Structure

New in PVRouter-1-phase (copied from PVRouter-3-phase `087af072`, then changed):

- `Mk2_1phase_RFdatalog_temp/` – the firmware. Renamed: `Mk2_3phase_RFdatalog_temp.ino` → `Mk2_1phase_RFdatalog_temp.ino`.
  - New `adc_chain.h` – Arduino-free: the analog pins and reference per board, the three-entry chain in the order the pipeline needs.
  - New `fast_control.h` – Arduino-free: the end-of-cycle prediction.
  - New `diverted_energy.h` – Arduino-free: CT2 energy accounting (anti-creep, Wh, reset after inactivity).
  - New `utils_display.h`, `utils_oled.h` – ported from `Mk2_fasterControl_Full/`.
  - Changed: `config_system.h`, `config_types.h`, `config.h`, `config_derived.h`, `calibration.h`, `processing.cpp`, `processing.h`, `shared_var.h`, `main.cpp`, `utils.h`, `teleinfo.h`, `validation.h`, `platformio.ini`, `Readme.md`, `Readme.en.md`, `CLAUDE.md`, `docs/`.
  - `test/native/` – copied tests; new `test_adc_chain`, `test_fast_control`, `test_diverted_energy`.
- `scripts/ci/` – `compile-fail.sh` and cases, pointed at the new folder.
- `sim/` – grid simulator with a channel map per board.
- `.github/workflows/`, `.github/actions/setup-platformio/`, `.pre-commit-config.yaml`, `scripts/git-hooks`, `scripts/hooks`, `scripts/install-hooks.*` – as in PVRouter-3-phase, pointed at the new folder.
- `NOTICE`, `LICENSE`, `CHANGELOG.md`, `README.md` (repository root).

Not copied: `RemoteLoadReceiver/` (the README links to PVRouter-3-phase's), `configurator/` (step 4), `Mk2_3phase_RFdatalog_temp/scripts/` (cloud tuning, 3-phase specific).

---

### Task 1: Import the 3-phase firmware and its tooling, unchanged in behaviour

**Files:**
- Create: `Mk2_1phase_RFdatalog_temp/**` (from `PVRouter-3-phase/Mk2_3phase_RFdatalog_temp/` at `087af072`, without `.pio`, `.vscode`, `scripts/`)
- Create: `scripts/ci/**`, `sim/**` (without build products), `.github/actions/setup-platformio/action.yml`, `.github/workflows/build.yml`, `.github/workflows/relay-sim.yml`, `.pre-commit-config.yaml`, `.clang-tidy`, `scripts/git-hooks/pre-commit`, `scripts/hooks/update-metadata.sh`, `scripts/install-hooks.sh`, `scripts/install-hooks.bat`
- Modify: every path `Mk2_3phase_RFdatalog_temp` in the copied scripts, Makefile, workflows and `platformio.ini` → `Mk2_1phase_RFdatalog_temp`

**Interfaces:**
- Produces: a building copy (still 3-phase logic) that every later task changes; the test commands of Global Constraints work.

- [ ] **Step 1: Copy** with `git -C D:/Git/IoT/PVRouter-3-phase archive 087af072 Mk2_3phase_RFdatalog_temp scripts/ci sim .github/actions .github/workflows/build.yml .github/workflows/relay-sim.yml .pre-commit-config.yaml .clang-tidy scripts/git-hooks scripts/hooks scripts/install-hooks.sh scripts/install-hooks.bat | tar -x -C D:/Git/IoT/pvr-wt/1phase-parity`, then `git mv`-equivalent renames of the folder and the `.ino`, and replace the folder name in every copied file (`grep -rl Mk2_3phase_RFdatalog_temp`). Merge `.github/workflows/build.yml` with the repository's existing one: the existing jobs for the older sketches stay.
- [ ] **Step 2: Build.** Expected: `SUCCESS`, the same flash and RAM as PVRouter-3-phase's `basic` build at `087af072`.
- [ ] **Step 3: Native tests.** Expected: the same count as PVRouter-3-phase (388), all pass.
- [ ] **Step 4: Compile-fail.** Expected: every case `ok`, exit 0.
- [ ] **Step 5: Commit** `chore: import the 3-phase firmware and its tooling as the base of the single-phase firmware`.

### Task 2: Boards and the three-channel ADC chain

**Files:**
- Create: `Mk2_1phase_RFdatalog_temp/adc_chain.h`
- Create: `Mk2_1phase_RFdatalog_temp/test/native/test_adc_chain/test_main.cpp`, `README.md`
- Modify: `config_system.h` (`NO_OF_PHASES{ 1 }`), `config_types.h` (`enum class PcbVersion`), `config.h` (`PCB_VERSION` comment and default), `processing.cpp` (sensor pins, `_ADMUX`, `_channels`, `ISR(ADC_vect)`, `SAMPLE_SET_PERIOD_US`), `calibration.h` (single values per board), `config_derived.h`
- Modify: `test/native/test_config_h/test_main.cpp`

**Interfaces:**
- Produces: `enum class PcbVersion : uint8_t { OLD_SEG, OLD_OLED, NEW };` in `config_types.h`.
- Produces, in `adc_chain.h`, namespace `AdcChain`:
  - `enum class Channel : uint8_t { V, I_GRID, I_DIVERTED };`
  - `constexpr uint8_t pinOf(PcbVersion board, Channel ch)` – the analog input (A0 = 0): OLD_SEG V 3, I_GRID 5, I_DIVERTED 4; OLD_OLED and NEW V 0, I_GRID 1, I_DIVERTED 3.
  - `constexpr uint8_t refBits(PcbVersion board)` – `bit(REFS0)` (AVcc) for the old boards, `bit(REFS1) | bit(REFS0)` for NEW. Use the AVR bit numbers as plain constants (REFS1 = 7, REFS0 = 6) so the header stays Arduino-free.
  - `constexpr uint8_t admuxWrittenAfter(PcbVersion board, Channel converted)` – the `ADMUX` value the ISR writes after the conversion of `converted`, so that the conversion after next is the channel after next in V → I_GRID → I_DIVERTED.
  - `constexpr Channel channelConverted(uint8_t round)` – which channel round `round` (0, 1, 2, …) converts once the pipeline is full.
- Produces: `SAMPLE_SET_PERIOD_US` = 3 conversions (≈312 µs at 16 MHz, prescaler 128), `SAMPLE_SETS_PER_CYCLE` 64 at 50 Hz, 53 at 60 Hz.

- [ ] **Step 1: Write the failing test** `test_adc_chain`:
  - `test_pins_per_board`: the nine `pinOf` values above.
  - `test_reference_per_board`: AVcc for OLD_SEG and OLD_OLED, 1.1 V for NEW.
  - `test_pipeline_order`: simulate the pipeline (the conversion running when the ISR for round k fires was set up by the write of round k-2) over 30 rounds from a chain primed with V then I_GRID; every converted channel equals `channelConverted(round)`, cycling V, I_GRID, I_DIVERTED, for each board.
- [ ] **Step 2: Run** native tests. Expected: FAIL, `adc_chain.h` not found.
- [ ] **Step 3: Implement** `adc_chain.h`; then in `processing.cpp` build `_channels[3]` from `AdcChain::admuxWrittenAfter`, keep `ISR(ADC_vect)` writing `ADMUX` last, dispatch V to `processVoltageRawSample(0, raw)`, I_GRID to `processCurrentRawSample(0, raw)`, I_DIVERTED to a new `processDivertedCurrentRawSample(raw)` (empty body until Task 4). Calibration becomes single values: `f_powerCalOldBoard`, `f_voltageCalOldBoard`, `f_powerCalDivertedOldBoard` (both old boards), `f_ctRating`, `f_ctRatingDiverted`, `f_powerCalPerAmp`, `f_voltageCalNewBoard` (NEW), `f_phaseCal`; `config_derived.h` or `processing.cpp` turns them into the internal `[NO_OF_PHASES]` arrays. Remove the code that only exists for phases 2 and 3 (per-phase power and voltage datalog fields beyond phase 0, `power_L`/`Vrms_L` loops: keep one phase).
- [ ] **Step 4: Run** native tests and build. Expected: PASS (all tests, the copied ones adapted where they assumed 3 phases); build `SUCCESS` for each `PCB_VERSION` (build three times, editing `config.h`).
- [ ] **Step 5: Commit** `feat: three single-phase boards, a three-channel ADC chain`.

### Task 3: Faster control: decide on the predicted energy

**Files:**
- Create: `Mk2_1phase_RFdatalog_temp/fast_control.h`
- Create: `Mk2_1phase_RFdatalog_temp/test/native/test_fast_control/test_main.cpp`, `README.md`
- Modify: `processing.cpp` (`processMinusHalfCycle`, `processRawSamples`, `processStartNewCycle`)

**Interfaces:**
- Consumes: `LoadPriorities::Switching<N>::onNewCycle(uint8_t (&)[N], const int32_t& energy, const Bucket&)`, `LoadPriorities::Bucket` (unchanged, `load_priorities.h`).
- Produces, in `fast_control.h`, namespace `FastControl`:
  - `inline constexpr uint8_t DECISION_SAMPLE{ 3 };` – the sample of the negative half cycle at which the decision is taken (Robin's `sampleSetsDuringNegativeHalfOfMainsCycle == 3`).
  - `constexpr int32_t predictEndOfCycle(int32_t bucket, int32_t energyFirstHalf, uint8_t samplesFirstHalf, uint8_t samplesPerCycle)` – the bucket level at the start of the cycle plus the energy of the first half scaled to a whole cycle (`energyFirstHalf × samplesPerCycle / samplesFirstHalf`, i.e. the average power as one cycle's energy, as Robin's `energyInBucket_long + averagePower`); returns `bucket` when `samplesFirstHalf` is 0.

The decision moves from "2nd sample of the positive half, measured bucket" (3-phase) to "sample set index 3 (the 4th) of the negative half, predicted bucket" (Robin): at the first sample of the negative half, `processMinusHalfCycle` computes the prediction from the cycle's `l_sumP[0]` and sample count so far (through the same per-sample calibration as `processLatestContribution`); at sample set index 3 (counted from 0, as Robin's `== 3`) of the negative half, `processRawSamples` calls `processStartNewCycle()`, whose `onNewCycle` gets the prediction. The loads change at the next positive zero crossing (the zero-crossing triac drivers latch there). The bucket's clamp to [0, capacity] stays after the contribution at the positive half cycle, as in the 3-phase firmware.

- [ ] **Step 1: Write the failing tests** `test_fast_control`:
  - `test_prediction_adds_the_first_half_as_a_whole_cycle`: `predictEndOfCycle(1000, 300, 32, 64) == 1600`.
  - `test_prediction_without_samples`: `predictEndOfCycle(1000, 300, 0, 64) == 1000`.
  - `test_prediction_negative_power`: `predictEndOfCycle(1000, -200, 32, 64) == 600`.
  - `test_decision_uses_prediction_only`: a `Switching<2>` and a `Bucket` with capacity 1000, mid 500, defaults 500/500; with the measured bucket at 400 (below the middle) and a prediction of 900, one `onNewCycle` with the prediction switches load 0 ON; the test then checks that `upperThreshold`/`lowerThreshold` hold the values the prediction implies (upper reset path untouched, lower reset to 500), i.e. no code path feeds the measured value.
- [ ] **Step 2: Run** native tests. Expected: FAIL, `fast_control.h` not found.
- [ ] **Step 3: Implement** `fast_control.h` and the processing changes above.
- [ ] **Step 4: Run** native tests and build. Expected: PASS, `SUCCESS`.
- [ ] **Step 5: Commit** `feat: decide on the predicted energy at mid-cycle, as Robin's fasterControl`.

### Task 4: Diverted energy on CT2

**Files:**
- Create: `Mk2_1phase_RFdatalog_temp/diverted_energy.h`
- Create: `Mk2_1phase_RFdatalog_temp/test/native/test_diverted_energy/test_main.cpp`, `README.md`
- Modify: `config.h` (`DIVERTED_CT_PRESENT`, `ANTI_CREEP_LIMIT`, `DISPLAY_SHUTDOWN_IN_HOURS` if not present), `processing.cpp` (`processDivertedCurrentRawSample`, per-cycle accounting at the positive half cycle, datalog copy), `shared_var.h` (`copyOf_divertedEnergyTotal_Wh`, `copyOf_divertedPower_W` and an `EDD_isActive` flag, as in `Mk2_fasterControl_Full/shared_var.h`)

**Interfaces:**
- Produces, in `diverted_energy.h`: `template< int32_t IEU_PER_WH, int32_t ANTI_CREEP_IEU, uint32_t SHUTDOWN_CYCLES > class DivertedEnergy` with
  - `void addCycle(int32_t energyIEU, bool anyLoadOn, bool anyLoadForced)` – ignores the cycle's energy when `anyLoadForced`; otherwise adds it if it is at least `ANTI_CREEP_IEU`, carries whole Wh into the total, counts cycles without any load ON and resets everything after `SHUTDOWN_CYCLES` of them;
  - `uint16_t totalWh() const`, `bool isActive() const` (Robin's `EDD_isActive`: some energy since the last reset).
- Produces: `Shared::copyOf_divertedEnergyTotal_Wh` (uint16_t), `Shared::copyOf_divertedPower_W` (int16_t, average over the datalog period), `Shared::EDD_isActive` (bool).
- `anyLoadOn` = any local load or relay physically ON. `anyLoadForced` = any local load or relay ON because of an override (override pin, scheduled boost, forced output: the 3-phase `Shared::b_overrideLoadOn[]`), as `Full` skips CT2 while its load is overridden.

- [ ] **Step 1: Write the failing tests** `test_diverted_energy`:
  - `test_whole_wh_carried`: with `IEU_PER_WH` 100, adding 60 twice gives `totalWh() == 1` and the remainder kept (a third 60 gives 1, a fourth gives 2).
  - `test_anti_creep_drops_noise`: with `ANTI_CREEP_IEU` 5, 1000 cycles of 4 leave `totalWh() == 0` and `isActive() == false`.
  - `test_reset_after_inactivity`: with `SHUTDOWN_CYCLES` 10, energy then 11 cycles with `anyLoadOn` false reset the total to 0 and `isActive()` to false; 9 cycles do not.
  - `test_total_saturates`: the total stops at 65535, never wraps.
  - `test_forced_cycle_not_counted`: 10 cycles of 60 with `anyLoadForced` true leave `totalWh() == 0`; the same 10 cycles with it false give 6 Wh.
- [ ] **Step 2: Run** native tests. Expected: FAIL, `diverted_energy.h` not found.
- [ ] **Step 3: Implement** `diverted_energy.h`; in `processing.cpp`, `processDivertedCurrentRawSample` accumulates V × I for CT2 like `processCurrentRawSample` (same DC removal, no CT filter), and the positive half cycle turns the cycle's sum into energy with the CT2 calibration and calls `addCycle`, only when `DIVERTED_CT_PRESENT`. The calibration: `f_powerCalDivertedOldBoard` on the old boards, `f_powerCalPerAmp × f_ctRatingDiverted` on NEW.
- [ ] **Step 4: Run** native tests and build. Expected: PASS, `SUCCESS`; with `DIVERTED_CT_PRESENT` false, the build is smaller (the accounting compiles out).
- [ ] **Step 5: Commit** `feat: diverted energy measured on CT2`.

### Task 5: Report the diverted energy

**Files:**
- Modify: `utils.h` (HumanReadable datalog line, JSON, IoT frame, startup summary), `teleinfo.h` (if it needs the label set), `test/embedded/test_teleinfo` or a new native test of the IoT frame builder if it is Arduino-free

**Interfaces:**
- Consumes: `Shared::copyOf_divertedEnergyTotal_Wh`, `Shared::copyOf_divertedPower_W` (Task 4).
- Produces: IoT labels `D` (W) and `E` (Wh), sent after `P`; JSON keys `"D"` and `"E"` (short, as the other JSON keys); HumanReadable: `, diverted <W> W, <Wh> Wh` appended to the datalog line; startup summary: `Diverted energy (CT2) is present` / `is NOT present` through `infolnPresence`.

- [ ] **Step 1: Write the failing test** `test_no_diverted_labels_without_ct2` and `test_diverted_labels_with_ct2` in the teleinfo test (whichever environment runs it today): the frame for a datalog with `DIVERTED_CT_PRESENT` true contains `D` and `E` with the given values (and not confused with `D1`…`Dn`); false contains neither.
- [ ] **Step 2: Run** it. Expected: FAIL.
- [ ] **Step 3: Implement** the four outputs.
- [ ] **Step 4: Run** native tests, the teleinfo test and the build. Expected: PASS, `SUCCESS`.
- [ ] **Step 5: Commit** `feat: diverted power and energy on every output`.

### Task 6: Displays

**Files:**
- Create: `Mk2_1phase_RFdatalog_temp/utils_display.h` (from `Mk2_fasterControl_Full/utils_display.h`), `utils_oled.h` (from `Mk2_fasterControl_Full/utils_oled.h`)
- Modify: `config_types.h` (`enum class DisplayType : uint8_t { NONE, OLED, SEG, SEG_HW };`), `config.h` (`TYPE_OF_DISPLAY`, default `DisplayType::NONE`), `main.cpp` (set-up and periodic update, as `Mk2_fasterControl_Full/main.cpp` lines 400–405 and 500–535), `processing.cpp` or `main.cpp` (the 7-segment refresh, wherever `Full` calls `refreshDisplay`), `platformio.ini` (`olikraus/U8g2` in `lib_deps`)

**Interfaces:**
- Consumes: `Shared::copyOf_divertedEnergyTotal_Wh`, `Shared::EDD_isActive` (Task 4), the temperatures (3-phase `utils_temp.h`).
- Produces: `DisplayType` (used by Task 7's checks). Display content as `Full`: diverted energy and temperature in turn; with `DIVERTED_CT_PRESENT` false, temperature only (or nothing without sensors).

- [ ] **Step 1: Port** both headers; the 7-segment pins stay those of `Full` (`SEG_HW`: D5–D9, D14–D16; `SEG`: `digitSelectorPin` D16, D10, D13, D11 and `segmentDrivePin` D2, D5, D12, D6, D7, D9, D8, D14). Use U8g2 in page mode (`_1_` constructor) for the OLED, to keep RAM.
- [ ] **Step 2: Build** each combination: `OLD_SEG` + `SEG`, `OLD_SEG` + `SEG_HW`, `OLD_OLED` + `OLED`, `NEW` + `OLED`, and `NEW` + `OLED` with relays, RF remote loads, temperature, dual tariff and rotation all on. Expected: `SUCCESS`; the last one under 32256 B flash and with at least 300 B of RAM left for the stack (the `basic` RAM figure ≤ 1748 B).
- [ ] **Step 3: Commit** `feat: 7-segment display on the first old board, OLED on the others`.

### Task 7: Compile-time checks and compile-fail cases

**Files:**
- Modify: `validation.h`
- Modify: `scripts/ci/compile-fail/*.sh` (cases that rely on 3-phase settings, `00`–`69`)
- Create: `scripts/ci/compile-fail/70-seg-display-on-oled-board.sh`, `71-oled-on-seg-board.sh`, `72-seg-without-chips-and-rf.sh`, `73-display-pin-used-by-a-load.sh`, `74-every-feature-builds.sh`, `75-no-diverted-ct-builds.sh`

**Interfaces:**
- Consumes: `PcbVersion` (Task 2), `DisplayType` (Task 6), `DIVERTED_CT_PRESENT` (Task 4).
- Produces: these `static_assert` messages, in the file's `"******** … ! Please check … in your config.h ! ********"` form:
  - `"The 7-segment display only exists on PcbVersion::OLD_SEG"`
  - `"The OLED needs A4/A5, which PcbVersion::OLD_SEG uses as analog inputs"`
  - display pins enter the existing duplicate-pin check (the RF pins already do), so cases 72 and 73 fail on its message.

- [ ] **Step 1: Write the cases 70–75** in the harness's `replace`/`expect` form: 70 sets `NEW` + `SEG_HW` (expect the first message); 71 `OLD_SEG` + `OLED` (the second); 72 `OLD_SEG` + `SEG` + remote loads (the duplicate-pin message); 73 `OLD_SEG` + `SEG_HW` with a load on D5 (the duplicate-pin message); 74 the every-feature `NEW` + `OLED` configuration of Task 6 (expect empty: builds); 75 `DIVERTED_CT_PRESENT` false (builds).
- [ ] **Step 2: Run** compile-fail. Expected: 70 and 71 FAIL as "built but should be refused" (their checks do not exist yet), the others `ok`.
- [ ] **Step 3: Implement** the checks; adapt the older cases whose `replace` lines no longer match the single-phase `config.h`.
- [ ] **Step 4: Run** compile-fail. Expected: every case `ok`, exit 0.
- [ ] **Step 5: Commit** `feat: board and display checks at compile time`.

### Task 8: Grid simulator for the three boards

**Files:**
- Modify: `sim/grid_sim.c`, `sim/board_scn.sh`, `sim/configs/sim_config.sh`, `sim/configs/relays.patch`, `sim/configs/remote_loads.patch`, `sim/Makefile`, `sim/ci_targets.sh`, `sim/README.md`, `sim/scenarios/*.scn`, `sim/scenarios/relays/*.scn`, `sim/scenarios/rf/*.scn`

**Interfaces:**
- Consumes: the firmware of Tasks 2–6.
- Produces:
  - `grid_sim` with one phase and a channel map: a scenario or board setting `channels V <pin> I1 <pin> I2 <pin>` (the board files write it), the ADC check following V → I1 → I2; `AVCC_MV` set per board (`avcc <mV>` setting, 3300 for both old boards); CT2's current = the sum of the currents of every local load and relay that is ON.
  - New expectation `expect diverted_wh <min> <max>` (the energy the firmware reports, read from its datalog line) and `expect diverted_w_avg <min> <max>`.
  - `board_scn.sh OLD_SEG|OLD_OLED|NEW` and `sim_config.sh OLD_SEG|OLD_OLED|NEW [CT rating]`.
  - Make targets `check` (OLD_SEG), `check-old-oled`, `check-new`, `check-relays`, `check-rf`, each scenario its own target as today.

- [ ] **Step 1: Write the expectations first**: in `scenarios/common.scn`, `expect misfiled_samples 0` and `expect admux_delay_min 128`; in `surplus_step.scn`, `expect diverted_wh` around the energy the scenario's surplus gives over its duration (the scenario's surplus × time, ±5 %); in `no_surplus.scn`, `expect diverted_wh 0 0`.
- [ ] **Step 2: Run** `make -C sim check`. Expected: FAIL (one phase, unknown settings).
- [ ] **Step 3: Implement** the simulator changes above; adapt the relay and RF patches and scenarios to the single-phase `config.h` (their pins on the board in use).
- [ ] **Step 4: Run** `make -C sim check check-old-oled check-new check-relays check-rf`. Expected: every scenario PASS.
- [ ] **Step 5: Commit** `test(sim): the three single-phase boards, CT2 and the diverted energy`.

### Task 9: CI, licence and documentation

**Files:**
- Modify: `.github/workflows/build.yml`, `.github/workflows/relay-sim.yml` (jobs for the new firmware: build per board, native, compile-fail, simulator), `README.md` (repository: the new firmware first; the older sketches as frozen, with the note that the two single-file sketches carry Robin's version 4 two-load logic he archived as problematic)
- Create: `NOTICE` (from PVRouter-3-phase's, with the single-phase origins: Robin's `Mk2_fasterControl_twoLoads_5`, `Mk2_multiLoad_wired_7a`, `Mk2_fasterControl_RFdatalog_1`), `CHANGELOG.md`
- Modify: `LICENSE` (GPL-3.0-or-later text, as PVRouter-3-phase), SPDX tags in every source file of the repository (the older sketches and `dev/` included), `Mk2_1phase_RFdatalog_temp/Readme.md`, `Readme.en.md`, `CLAUDE.md`, `docs/*.md` (single phase: boards, CT2 wiring through every local output, displays, the IoT labels, the filter values marked as interim)

- [ ] **Step 1: Write** the workflows and run them locally where possible (`act` is not required: check the YAML with the repository's `check yaml` pre-commit hook and run each job's commands by hand).
- [ ] **Step 2: Write** the licence files and docs. French: U+00A0 before `:`, `;`, `?`, `!` in `Readme.md`.
- [ ] **Step 3: Run** the whole suite: build per board, native tests, compile-fail, every simulator target. Expected: all pass.
- [ ] **Step 4: Commit** `docs: the single-phase firmware, its licence and CI`.
