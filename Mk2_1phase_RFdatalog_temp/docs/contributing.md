# Contributing Guide

How to work on the router firmware. The workflow of the repository (branches, pre-commit hooks)
is also in [CONTRIBUTING.md](../../CONTRIBUTING.md); the rules below come from
[CLAUDE.md](../../CLAUDE.md) and [the firmware's CLAUDE.md](../CLAUDE.md), which the maintainers
follow too.

## Setup

### PlatformIO (recommended)

```bash
git clone https://github.com/FredM67/PVRouter-1-phase.git
cd PVRouter-1-phase/Mk2_1phase_RFdatalog_temp
pio run -e basic          # build
pio test -e native        # native unit tests (needs gcc)
```

PlatformIO fetches the libraries itself (`platformio.ini`). On Windows, the native tests, the
simavr tests, the compile-fail checks and the grid simulator run in WSL: Git Bash has no gcc and
no simavr.

### Arduino IDE

Install **OneWire**, **RFM69** (LowPowerLab) and **U8g2** with the Library Manager, then open
`Mk2_1phase_RFdatalog_temp.ino`: see
[Required libraries for Arduino IDE](../Readme.en.md#required-libraries-for-arduino-ide).

## Code rules

- C++17, `constexpr`, `enum class`, `static_cast`; camelCase for variables and functions,
  UPPER_CASE for constants, PascalCase for types.
- Include `config_derived.h`, never `config.h`: `config.h` holds the settings only
  (see [configuration.md](configuration.md)).
- No dynamic allocation, and no float arithmetic at run time: the float calibration values are
  turned into fixed-point constants at compile time; the energy bucket, the prediction, the
  datalog and the CT1 filter are integer code.
- Keep the narrow types (`uint8_t`, ...). The build must stay warning-free; when a loop count can
  be 0, write the loop with `!=` rather than casting to `int`.
- No feature on pins 14 and above (A0-A5 are the analog inputs, the display or the I2C bus).
- Formatting: clang-format 18 (`.clang-format`), applied by the pre-commit hook and checked by CI.
- A new file starts with the header the others have (`@file`, `@author`, `@brief`, `@version`,
  `@date`, `@copyright` and `SPDX-License-Identifier: GPL-3.0-or-later`); the pre-commit hook
  updates `@date` and `@copyright`.
- Do not bring back the version 4 control logic (thresholds moved on the measured energy,
  decisions on the predicted one): the loads are decided on the prediction (`fast_control.h`).

### The ADC interrupt

`ISR(ADC_vect)` (`processing.cpp`) has 1664 CPU cycles per call, one ADC conversion (104 µs).

- ADMUX is written in the ISR for the conversion after next (`adc_chain.h`).
- No busy-wait, no allocation, no serial output in the ISR.
- The work of the mains cycle stays spread over the CT2 calls (`CycleTask`).
- Any ISR change is measured in the grid simulator (`ADC ISR max`, `ADMUX delay`,
  `misfiled samples`; [sim/README.md](../../sim/README.md)), against `dev`, with the procedure of
  [.claude/skills/isr-change](../../.claude/skills/isr-change/SKILL.md). It must not add flash,
  lengthen the slowest call or bring the ADMUX write closer to its limits. The current figures
  are in [performance.md](performance.md).

### Settings

Every user setting is checked at compile time in `validation.h`, with a `static_assert` and a
message that says what to fix. A new check gets a case in `scripts/ci/compile-fail/` (see
[testing.md](testing.md)). A setting users see also needs its page in the documentation
repository (FredM67/Mk2PVRouter).

## Before a pull request

Run the tests your change touches ([testing.md](testing.md)):

```bash
pio test -e native                  # from Mk2_1phase_RFdatalog_temp/
pio test -e uno_sim                 # embedded tests in simavr (Linux/WSL)
bash scripts/ci/compile-fail.sh     # from the repository root (Linux/WSL)
make -C sim check                   # grid simulator, see sim/README.md
```

Never commit the local changes to `version.h` (rewritten by every build) or
`.vscode/extensions.json` (rewritten by VS Code).

## Branches, commits, pull requests

- `main` (releases) and `dev` (integration) are protected. Work on a topic branch: `feat/...`,
  `fix/...`, `docs/...`, `ci/...`; personal branches under `users/`.
- Conventional commits: `feat:`, `fix:`, `docs:`, `test:`, `ci:`, `refactor:`, `chore:`; `!`
  marks a change that breaks existing settings.
- Pull requests target `dev`, never `main`, with the
  [pull request template](../../.github/pull_request_template.md).
- The firmware shares its settings, names and files with the 3-phase firmware
  ([PVRouter-3-phase](https://github.com/FredM67/PVRouter-3-phase)). A change that applies there
  gets a matching pull request; each pull request says whether it was ported, or why not.

CI (`.github/workflows/build.yml`) builds the firmware in several configurations (each board with
its display, the serial outputs, remote loads) and runs the native tests, the embedded tests in
simavr, the compile-fail checks, the grid simulator scenarios, the configurator tests and the
build of the remote load receiver, and posts a performance report (flash, RAM and benchmark
cycles against the target branch) on the pull request. Other workflows run the relay scenarios
of the grid simulator, check the formatting and the spelling, and run CodeQL.
