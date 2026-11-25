---
name: isr-change
description: Use when changing anything the ADC interrupt runs in the single-phase firmware (ISR(ADC_vect) in processing.cpp, the CycleTask work after the zero crossings, the 7-segment refresh on the CT1 call, adc_chain.h, ADMUX handling, 1-Wire scheduling in temp_schedule.h), or when proposing a speed-up of the sampling path, before writing the change.
---

# Changing the ADC interrupt (single-phase)

## What a change must keep

- 1664 cycles per call. ADMUX is written for the conversion after next, at least one ADC clock
  (128 cycles) after its conversion started, and before the next conversion starts.
- The simulator enforces margins, not just the deadline (`sim/scenarios/common.scn`):
  `isr_max 560`, `admux_delay_min 128`, `admux_delay_max 1620`; `rf/` and `relays/` 560 and 620;
  the board files 1600 for `check-seg-temp` and 1480 for `check-new-temp`. **Never raise a margin
  to make a change pass**: they sit just above the worst measured so that a change eating the
  headroom fails before samples are lost. A change that needs a higher margin is a finding to
  report, not a number to edit.
- The once-per-cycle work is spread over the CT2 calls after the zero crossings (`CycleTask`) to
  keep each call short; the 1-Wire transactions start right after the positive crossing's work
  (`temp_schedule.h`). Moving work between calls moves it against the 1-Wire slots: measure the
  temperature builds.
- The tightest build is `check-seg-temp` (old board, display refreshed on the CT1 call, CT1
  filter, a call entered up to ~1200 cycles late after a 1-Wire slot).
- The simulated ADC locks the channel at once: an ADMUX write that is too early shows only in the
  `ADMUX delay` figure, never as misfiled samples. The ADC reference cancels out in the
  simulator: a wrong reference is caught by the native tests of `adc_chain.h`, not by the sim.
- The bar: a change must not add flash, lengthen the **slowest** call (`ADC ISR max`), or bring
  the ADMUX write closer to 128 or to its margins. Gains on the average call alone are not
  wanted: `loop()` is not short of CPU.
- Measured on the 3-phase ISR and turned down there (2026-11): a `switch` with a constant
  channel per case (+912 B flash, ADMUX only 27 cycles above 128) and the rare paths `noinline`
  (slowest call +40 cycles, +64 B). Do not propose them here without a real need.

## Measure: dev first, same procedure for the change

Keep dev and the change in two trees. Run in WSL (Git Bash has no simavr). Put the steps in a
script file and run it with `MSYS_NO_PATHCONV=1 wsl bash <file>`: Git Bash expands `$VARS` first.
The build commands of every target are in `sim/README.md` ("The other boards..."): follow them.

- `export PVR_SIM_BUILD=1` (the banner's length shifts the run against the scenario).
- `sh sim/configs/sim_config.sh <BOARD> ...` before **every** build, then `restore` the config
  files (`git checkout config.h calibration.h config_system.h version.h`). It also sets three
  loads on D5-D7: the scenarios assume them. Without it, scenarios fail in ways that look like a
  regression.
- Patches with `sed -n '/^diff --git/,$p' <patch> | patch -p1` (works in a WSL git worktree,
  where git does not); one `PLATFORMIO_BUILD_DIR` per build.

| Targets | Why |
|---|---|
| `check`, `check-old-oled`, `check-new`, `check-new20`, `check-60hz` | every board, 20 A CTs, 60 Hz |
| `check-seg-temp`, `check-new-temp` | thinnest ADMUX margin, after a 1-Wire slot |
| `check-rf`, `check-relays` | their own `common.scn` limits |
| `compare-boards`, `compare-old-oled`, `compare-boards-20`, `compare-ct` | same result on every board |

Run dev twice: simavr is deterministic, two runs of the same firmware give the same figures, or
nothing can be compared.

## Report and decide

Table per target, dev against the change: ISR average and max (with the channel and angle of the
longest call), ADMUX delay min and max, misfiled samples, overruns, flash and RAM (`avr-size`),
scenario results. Drop the change when a margin fails, a scenario that passed fails, flash grows,
the slowest call gets longer, only the average improves, or a warning appears. Then also `pio test -e native` and `pio test -e uno_sim`
(WSL) and clang-format 18.1.8. Update `Mk2_1phase_RFdatalog_temp/docs/performance.md` and the
figures in `CLAUDE.md` / `sim/README.md` if they moved. Port to PVRouter-3-phase only after
measuring there, with its own `isr-change` skill: its ISR is different.
