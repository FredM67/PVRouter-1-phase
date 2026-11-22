# Changelog

All notable changes to the single-phase router firmware (`Mk2_1phase_RFdatalog_temp`) and its
tooling. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

Changes on `dev`, not yet on `main`.

> **Not yet tested on a real board:** `Mk2_1phase_RFdatalog_temp` passes the native tests, the compile-time checks and the grid simulator, but has not run on real hardware yet. Keep `Mk2_fasterControl_Full` on an installation in service until it has.

### Upgrading from `Mk2_fasterControl_Full`

`Mk2_1phase_RFdatalog_temp` is a new firmware, next to the older sketches, which stay as they
are. Its settings are those of the 3-phase firmware ([PVRouter-3-phase]), with the same names,
meaning and syntax: the `config.h` of `Mk2_fasterControl_Full` does not fit. Start from the
shipped `config.h`:

- **Motherboard:** set `PCB_VERSION` to `PcbVersion::OLD_SEG` (Robin's Mk2 board, 7-segment
  display), `PcbVersion::OLD_OLED` (the second old board, OLED) or `PcbVersion::NEW` (the
  universal board). The shipped `config.h` targets `NEW` with the mk2Wifi module
  (`SerialOutputType::IoT`).
- **Calibration:** copy `powerCal_grid`, `powerCal_diverted` and `f_voltageCal` of `Full` into
  `f_powerCalOldBoard`, `f_powerCalDivertedOldBoard` and `f_voltageCalOldBoard`; `lpf_gain` and
  `alpha` become `lpf_gainOldBoard` and `alphaOldBoard`. On the new board, enter the CT ratings in
  `f_ctRating` and `f_ctRatingDiverted`.
- **Display:** `TYPE_OF_DISPLAY` is `NONE`, `SEG`, `SEG_HW` or `OLED`.
- **CT2:** pass the wires of every local output, TRIACs and relays, through CT2.
- **mk2Wifi:** the frames carry two new lines, `D` (diverted power, W) and `E` (diverted energy,
  Wh); the ESPHome configuration ignores them until its single-phase sensors come.

### Added

- **The single-phase firmware, `Mk2_1phase_RFdatalog_temp`:** derived from the 3-phase firmware
  `Mk2_3phase_RFdatalog_temp` (once its rotation groups were in), with every feature that applies
  to one phase: the settings-only `config.h` with its target codes and load map, dual tariff
  with scheduled boosts and their temperature threshold, priority rotation with rotation groups,
  relay diversion (EWMA filter, minimum times, battery mode), override pins, routing stop and
  router OFF pins, remote loads over RFM69 (driven by the `RemoteLoadReceiver` of PVRouter-3-phase),
  DS18B20 sensors, watchdog LED, calibration mode, the HumanReadable, IoT and JSON outputs and
  the compile-time checks of `validation.h`.
- **Robin Emley's fasterControl core**, that of `Mk2_fasterControl_twoLoads_5`: at the first
  sample set of the negative half cycle, the energy at the end of the cycle is predicted; at its
  4th sample set, the loads are decided on that prediction; they switch at the next positive zero
  crossing. The prediction needs no division in the ADC interrupt.
- **Three boards** (`PCB_VERSION`): `OLD_SEG` (V A3, CT1 A5, CT2 A4, AVcc), `OLD_OLED` (V A0,
  CT1 A1, CT2 A3, AVcc) and `NEW` (V A0, CT1 A1, CT2 A3, internal 1.1 V). The three channels are
  sampled in a fixed chain, each about every 312 µs, ADMUX written for the conversion after next.
  A0 to A5 cannot be used for any feature.
- **Diverted energy on CT2** (`DIVERTED_CT_PRESENT`, `ANTI_CREEP_LIMIT` in W,
  `DISPLAY_SHUTDOWN_IN_HOURS`): diverted power and energy in every serial output (text line
  `, diverted <W> W, <Wh> Wh`, JSON keys `D` and `E`, IoT lines `D` and `E`) and on the display.
  The cycles during which a load is forced are not counted; the total is reset after a period
  without diversion.
- **Displays** (`TYPE_OF_DISPLAY`): the 4-digit 7-segment display of `OLD_SEG`, with (`SEG_HW`) or
  without (`SEG`) its pin-saving chips, multiplexed from the ADC interrupt, and the OLED of
  `OLD_OLED` and `NEW` (U8x8). They show the diverted energy and each temperature in turn, every
  5 s, `FORC` and `OFF` in the energy's turn. The display's pins enter the pin checks.
- **CT1 filter** of the old boards (`lpf_gainOldBoard`, `alphaOldBoard`), Robin's, in fixed point;
  off on the new board. The values shipped (9 and 0.0011) are interim, to be measured per CT type.
- **Datalog period** up to 40 s (30 s with relays): the sample set counter widens to 32 bits
  beyond 19 s.
- **Tests:** native tests (prediction, decision, diverted energy, CT1 filter, ADC chain,
  display, and those of the 3-phase firmware that apply), embedded tests in simavr, compile-fail
  cases (displays, CT2, analog pins), and the grid simulator with a model of each board (CT2
  measuring every local load and relay, the 7-segment display's pin remapping), its scenarios
  run on every board, at 60 Hz, with 20 A CTs and with temperature sensing.
- **CI:** builds of each board with its display, native, embedded and compile-fail tests, the grid
  and relay simulations, the size and speed report of the pull requests, the formatting check
  (clang-format 18).
- **GitHub Pages** (<https://fredm67.github.io/PVRouter-1-phase/>), as PVRouter-3-phase: a landing
  page, the flash and RAM of every build and the embedded benchmarks at each change of `dev`
  (`bench/`, from the `pages` job of the CI), and the Doxygen documentation of
  `Mk2_1phase_RFdatalog_temp` (`doc/`). Flash and RAM badges in the READMEs.
- **NOTICE:** where the code comes from, the third-party code and the libraries used; the files
  that hold your settings are yours.

### Changed

- **Doxygen and CodeQL:** both follow `Mk2_1phase_RFdatalog_temp` instead of `Mk2_fasterControl_Full`.
  The documentation is deployed to `doc/` of `gh-pages`, no longer at its root: the old
  `Mk2_fasterControl_Full` site is removed by the first update of the pages after that.
- **License:** Robin Emley's sketches in `Robin/` and `dev/` stay as he published them, without
  a licence tag. The rest of the repository, `Mk2_fasterControl_Full` and the single-file
  sketches included, moves from LGPL-2.1 to GPL-3.0-or-later, as PVRouter-3-phase; its source
  files carry an SPDX tag.
- **README:** the new firmware first; the older sketches are frozen. The single-file sketches
  `Mk2_fasterControl_twoLoads_temp_1` and `Mk2_fasterControl_threeLoads_temp_1` carry the
  two-load logic of Robin's version 4, which he archived as problematic.

[PVRouter-3-phase]: https://github.com/FredM67/PVRouter-3-phase
