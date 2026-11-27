# Testing Guide

The firmware is checked in five ways. All of them run in CI (`.github/workflows/build.yml`, and
`relay-sim.yml` for the relay scenarios), and locally on Linux or WSL: on Windows, Git Bash has
no gcc and no simavr.

| Kind | What it checks | Where |
|---|---|---|
| Native tests | The Arduino-free headers, on the PC (Unity) | `test/native/` |
| Embedded tests | Code that must run on the AVR: assembly, serial output, pins; benchmarks | `test/embedded/` |
| Compile-fail checks | Settings that `validation.h` must refuse, each with its message | `scripts/ci/compile-fail/` |
| Grid simulator | The unmodified firmware on a simulated single-phase installation, on every board: regulation, loads, CT2, relays, RF, temperature sensors, ISR timing | `sim/` |
| Benchmarks | Flash, RAM and CPU cycles, compared on every pull request | `test/embedded/*_perf`, `scripts/ci/` |

## Native tests

```bash
cd Mk2_1phase_RFdatalog_temp
pio test -e native                          # every suite (needs gcc)
pio test -e native -f "*test_fast_control*" # one suite
```

One directory per suite, `test/native/test_*/`. The suites and what each covers:
[test/README.md](../test/README.md).

## Embedded tests

```bash
pio test -e uno_sim     # in the simavr simulator, no hardware (Linux/WSL)
pio test -e uno         # on a connected Arduino Uno
```

The `uno_sim` environment (`platformio.ini`) runs each suite of `test/embedded/` in simavr, a
cycle-exact ATmega328P at 16 MHz. Every embedded suite calls `haltAfterTests()`
(`test/embedded/halt_after_tests.h`) right after `UNITY_END()`, or simavr never exits.

## Compile-fail checks

```bash
bash scripts/ci/compile-fail.sh                               # from the repository root, every case
bash scripts/ci/compile-fail.sh scripts/ci/compile-fail/40-duplicate-pin.sh   # one case
```

Each case edits the shipped `config*.h` and `calibration.h` and sets `expect` to part of the
`static_assert` message the build must stop on; an empty `expect` is a build that must succeed. A
new check in `validation.h` gets a case here. Details in the header of
`scripts/ci/compile-fail.sh`.

## Grid simulator

```bash
PVR_SIM_BUILD=1 sh sim/configs/sim_config.sh   # before every firmware build for the simulator
(cd Mk2_1phase_RFdatalog_temp && pio run -e basic)
make -C sim check                              # OLD_SEG; see sim/README.md for the other targets
```

`sim_config.sh` sets the board (`OLD_SEG` by default, `OLD_OLED` or `NEW`), the human-readable
output and three local loads. `grid_sim` runs `firmware.elf` in simavr against scenarios (surplus
over time, loads, CT2, relays, remote units, DS18B20 sensors) and fails on any unmet expectation.
The targets are `check`, `check-old-oled`, `check-new`, `check-new20`, `check-60hz`,
`check-seg-temp`, `check-new-temp`, `check-relays` and `check-rf`. The simulator also measures the
ADC interrupt: `ADC ISR max`, `ADMUX delay`, `misfiled samples`; any change to the interrupt is
measured there. The builds each target needs and the scenario format:
[sim/README.md](../../sim/README.md).

## Benchmarks

The `*_perf` suites and `test_datalog_math_avr` print `BENCH|name|value|unit` lines with cycle
counts from simavr. CI collects them with the flash and RAM of every build and posts a
performance report on each pull request (a value more than 2 % worse is flagged). See
[Benchmark history](../test/README.md#benchmark-history).

## Other tests

- Configurator: `node --test configurator/test/*.test.js` (see
  [configurator/README.md](../../configurator/README.md)).
