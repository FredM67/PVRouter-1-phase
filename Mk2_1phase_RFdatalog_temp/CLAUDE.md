# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Mk2_1phase_RFdatalog_temp (repository PVRouter-1-phase) is a real-time embedded system for single-phase solar power routing on Arduino Uno (ATmega328P). It diverts excess photovoltaic energy to loads (water heaters, etc.) via TRIAC/relay control, and measures the diverted energy with a second CT. This is **safety-critical power electronics code** with strict real-time constraints.

It derives from the 3-phase firmware (PVRouter-3-phase, `Mk2_3phase_RFdatalog_temp`): same settings, same names, same meaning, same file names and interfaces (sharing code later must stay possible). What differs:
- **Control core**: Robin Emley's fasterControl (`Mk2_fasterControl_twoLoads_5`), in `fast_control.h` and `processing.cpp`: prediction at the first sample set of the negative half cycle, decision at its index 3 (`DECISION_SAMPLE`), loads switched at the next positive zero crossing. Never bring back the version 4 logic (thresholds moved on the measured energy, decisions on the predicted one).
- **Boards** (`PCB_VERSION`): `OLD_SEG` (V A3, CT1 A5, CT2 A4, AVcc), `OLD_OLED` (V A0, CT1 A1, CT2 A3, AVcc), `NEW` (V A0, CT1 A1, CT2 A3, internal 1.1 V); `adc_chain.h`. Calibration values are single values, not per-phase arrays (`f_powerCalOldBoard`, `f_powerCalDivertedOldBoard`, `f_ctRating`, `f_ctRatingDiverted`...).
- **Diverted energy** on CT2 (`diverted_energy.h`), **displays** (`TYPE_OF_DISPLAY`: `display_core.h`, `utils_display.h`, `utils_oled.h`), **CT1 filter** of the old boards (`ct_filter.h`, fixed point, interim values).
- **The work of the mains cycle spread over the CT2 calls** (`CycleTask`, `runNextCycleTask()` in `processing.cpp`): the V call at a zero crossing only takes the snapshots that must be taken on that sample, then each CT2 call runs one task (contribution, CT2 energy, datalog sum, prediction, decision, load states, ports), so that every call writes ADMUX in time, even after waiting for a 1-Wire time slot. The 3-phase firmware keeps that work in the call of the crossing and starts its 1-Wire transactions right after it instead: not ported.
- `RemoteLoadReceiver` is not in this repository: the single-phase router drives the 3-phase repository's receivers.

## Build Commands

```bash
# Build (default configuration)
pio run

# Build specific environment
pio run -e basic           # Production build
pio run -e basic_debug     # Debug build with symbols

# Run tests
pio test -e native         # Cross-platform unit tests
pio test -e uno_sim        # Embedded tests in simavr (no hardware)
bash ../scripts/ci/compile-fail.sh   # settings validation.h must refuse (from the repo root: scripts/ci/compile-fail.sh)

# Upload to Arduino
pio run -t upload

# Code formatting check (clang-format 18, as the pre-commit hook and CI pin it)
clang-format --dry-run --Werror *.cpp *.h

# Grid simulator (Linux/WSL, from the repo root): see ../sim/README.md
make -C ../sim check

# Static analysis
pio check
```

## Architecture

### Layer Structure

1. **Application Layer** (`main.cpp`) - Main loop, load priority management, dual tariff forcing
2. **Processing Engine** (`processing.cpp`) - 9.6 kHz ADC ISR over three channels (V, CT1, CT2), real power calculation (V × I × cos φ), energy bucket, fasterControl prediction and decision, diverted energy
3. **Hardware Abstraction** (`utils_*.h`) - Pin control, relay timing, OneWire temperature sensors
4. **Configuration Layer** (`config.h`, `validation.h`) - Compile-time configuration with `static_assert` validation

### Key Files

- `config.h` - User configuration (features, pins, loads) - **primary file users edit**. Settings only: it includes `config_types.h` and nothing else
- `config_types.h` - What config.h is written with: settings types, `LOAD()`/`RELAY()`/`ALL_...()` target codes, the boost windows. No setting, no Arduino
- `config_derived.h` - What the code includes (never config.h): derived constants, `remoteLoads`, and the objects built from the settings (`relayDiverter`, `overrideTable`, `boostSchedule`, `tempSensors`)
- `calibration.h` - Power calibration values - **users edit for their hardware**
- `processing.cpp/.h` - Core real-time processing engine
- `validation.h` - ~50+ compile-time safety assertions
- `utils_relay.h` - Relay control: pins and printing; the decisions, with EWMA filtering, are in `relay_engine.h` (Arduino-free, native tests)
- `utils_temp.h` - DS18B20 temperature sensor management; `temp_schedule.h` - which 1-Wire transaction runs on which mains cycle (Arduino-free, native tests)
- `utils_display.h` / `utils_oled.h` - The display (`TYPE_OF_DISPLAY`): the 7-segment display, multiplexed from the ADC ISR, and the OLED (U8x8), updated once per second from the loop; the pins, port images and content are in `display_core.h` (Arduino-free, native tests)

### Real-Time Constraints

- ADC ISR runs at **9.6 kHz** (every 104μs, 1,664 CPU cycles); each channel every ~312μs, 64 sample sets per mains cycle at 50 Hz
- Free-running ADC: the ADMUX written in the ISR applies to the conversion after next (`adc_chain.h`)
- Every ISR call must end within one ADC conversion; the grid simulator enforces it (`sim/scenarios/common.scn`): longest call `isr_max` 560 cycles, latest ADMUX write `admux_delay_max` 1620 cycles (1480 for `check-new-temp`, 1600 for `check-seg-temp`: the temperature builds, after a 1-Wire slot). The once-per-cycle work is spread over the CT2 calls that follow the zero crossings (`CycleTask` in processing.cpp) to keep each call short; the 1-Wire transactions start right after the positive crossing's work (`temp_schedule.h`), one per mains cycle, so that their slots miss those calls
- No blocking calls, no dynamic allocation in ISR
- Volatile shared variables for ISR-main loop communication

## Critical Development Rules

### Memory Constraints (Arduino Uno: 32KB Flash, 2KB RAM)

- **No malloc/dynamic allocation** - all memory statically allocated
- Use `constexpr` for compile-time constants

### Code Style

- C++17 with GNU extensions (`-std=gnu++17`)
- `constexpr` over `#define` for constants
- `static_cast<>` over C-style casts
- `enum class` for type-safe enumerations
- No magic numbers - use named constants
- camelCase for functions/variables, UPPER_CASE for constants, PascalCase for types

### Safety Requirements

- All load control must include safety validation
- Never modify power routing logic without understanding the energy bucket algorithm
- Measure every ISR change in the grid simulator against dev, as the `isr-change` skill (`.claude/skills/isr-change/`) describes; test timing on real hardware with oscilloscope when the change is risky
- Compile-time validation via `static_assert` is preferred over runtime checks

## Git Workflow

- Two-branch model: `main` (stable) + `dev` (development)
- PRs target `dev` branch
- Conventional commits: `fix:`, `feat:`, `docs:`, `chore:`
- Formatting: a fresh clone has no pre-commit hook until `scripts/install-hooks.sh` (or `pre-commit install`) runs; without it, run `clang-format -i` (18.1.8, as CI) on the changed files before committing, or the formatting check fails
- A setting users see also needs its page in the documentation repository (FredM67/Mk2PVRouter, French docs), in a PR there or an issue that the PR links to
- **Port every change to the 3-phase firmware** (PVRouter-3-phase, `Mk2_3phase_RFdatalog_temp`), when it applies there: the two firmwares share their settings, names, files and interfaces. A fix, feature or refactor here gets a matching PR there, unless it touches what only this firmware has (see "What differs" above: the fasterControl core, CT2's diverted energy, the displays, the old boards and their CT1 filter). Each PR says whether it was ported, with a link to the other PR, or why not.

## Test Structure

```
test/
├── embedded/     # AVR tests, run in simavr (uno_sim), benchmarks included
│   ├── test_teleinfo/
│   ├── test_utils_relay/
│   └── ...
└── native/       # Cross-platform tests (run on host)
    ├── test_fast_control/
    ├── test_diverted_energy/
    ├── test_ct_filter/
    ├── test_adc_chain/
    ├── test_display/
    └── ...
```

Also: `../scripts/ci/compile-fail/` (one case per file, the static_assert message expected) and `../sim/` (grid simulator, scenarios with expected results on every board).

## Dependencies

- **ArduinoJson 6.x** (NOT v7): only for the embedded test `test_serial_output`, the firmware writes its JSON itself
- **OneWire 2.3.8+** (for DS18B20 sensors)
- **RFM69** by LowPowerLab (RF datalog and remote loads; RFM12B / JeeLib not supported)
- **U8g2** (its U8x8 part, for the OLED)
