<div align = center>

[![CI](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/build.yml?branch=dev&label=CI&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/build.yml)
[![relay simulation](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/relay-sim.yml?branch=dev&label=relay%20simulation&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/relay-sim.yml)
[![CodeQL](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/codeql.yml?branch=dev&label=CodeQL&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/codeql.yml)
[![doc](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/doxygen-gh-pages.yml?branch=dev&label=doc&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/doxygen-gh-pages.yml)
<br/>
[![Arduino Uno](https://img.shields.io/badge/Arduino%20Uno-ATmega328P-00878F?logo=arduino&logoColor=white)](https://docs.arduino.cc/hardware/uno-rev3/)
[![flash](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Ffredm67.github.io%2FPVRouter-1-phase%2Fbench%2Fhistory.json&query=%24.entries%5B-1%3A%5D.sizes.basic.flash&label=flash&suffix=%20%2F%2032256%20B&color=blue)](https://fredm67.github.io/PVRouter-1-phase/bench/)
[![RAM](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Ffredm67.github.io%2FPVRouter-1-phase%2Fbench%2Fhistory.json&query=%24.entries%5B-1%3A%5D.sizes.basic.ram&label=RAM&suffix=%20%2F%202048%20B&color=blue)](https://fredm67.github.io/PVRouter-1-phase/bench/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/17)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-ready-F5822A?logo=platformio&logoColor=white)](https://platformio.org/)
<br/>
[![License](https://img.shields.io/github/license/FredM67/PVRouter-1-phase)](https://github.com/FredM67/PVRouter-1-phase/blob/main/LICENSE)
[![Last commit](https://img.shields.io/github/last-commit/FredM67/PVRouter-1-phase/dev?label=last%20commit%20%28dev%29)](https://github.com/FredM67/PVRouter-1-phase/commits/dev)
[![Issues](https://img.shields.io/github/issues/FredM67/PVRouter-1-phase)](https://github.com/FredM67/PVRouter-1-phase/issues)
[![Stars](https://img.shields.io/github/stars/FredM67/PVRouter-1-phase?style=flat)](https://github.com/FredM67/PVRouter-1-phase/stargazers)
[![Forks](https://img.shields.io/github/forks/FredM67/PVRouter-1-phase?style=flat)](https://github.com/FredM67/PVRouter-1-phase/network)
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)
<br/>
<br/>
  [![en](https://img.shields.io/badge/lang-en-red.svg)](Readme.en.md)
  [![fr](https://img.shields.io/badge/lang-fr-blue.svg)](Readme.md)
</div>

# PVRouter (single-phase version)

Firmware of a single-phase photovoltaic diverter for the Arduino Uno (ATmega328P). It measures the power exchanged with the grid, and sends the surplus of the solar production to resistive loads, typically the heating element of a water heater, instead of exporting it.

It derives from Robin Emley's single-phase [Mk2PVRouter](https://web.archive.org/web/20231115123017/https://mk2pvrouter.co.uk/) (archived): it has the control core of his `Mk2_fasterControl_twoLoads_5`, and the features and settings of the 3-phase firmware, `Mk2_3phase_RFdatalog_temp`. All the time-critical processing runs in the ADC interrupt, with integer arithmetic only, and the whole configuration is checked at compile time.

> [!NOTE]
> Building or installing a router? The **[user documentation](https://fredm67.github.io/Mk2PVRouter/)** (in French) covers the hardware, the installation, the calibration and the troubleshooting.
> For a 3-phase installation, see [PVRouter-3-phase](https://github.com/FredM67/PVRouter-3-phase).

> [!WARNING]
> **Not yet tested on a real board.** This firmware passes its native tests, its compile-time checks and the grid simulator on the three boards, but it has not run on real hardware yet. Until it has, use `Mk2_fasterControl_Full` (in `obsolete/`) on an installation in service.

- [PVRouter (single-phase version)](#pvrouter-single-phase-version)
  - [Features](#features)
  - [Getting started](#getting-started)
  - [Repository layout](#repository-layout)
  - [Developer documentation](#developer-documentation)
  - [Zero-export installations (Enphase)](#zero-export-installations-enphase)
  - [The older sketches, frozen](#the-older-sketches-frozen)
  - [Coming soon](#coming-soon)
  - [Roadmap](#roadmap)
- [Support](#support)
- [Contributing](#contributing)
- [Authors and acknowledgment](#authors-and-acknowledgment)

## Features

What the router does for you:

- **Heats your water with your solar surplus.** As soon as your panels produce more than the house consumes, the router sends the excess to the water heater instead of giving it away to the grid. It matches the surplus closely: 300 W of surplus means about 300 W in the water heater, without buying power from the grid ([output settings](Mk2_1phase_RFdatalog_temp/Readme.en.md#triac-output-configuration)).
- **Several heating elements, in the order you choose.** The router feeds the first one; once it runs at full power, it moves on to the next. So that they wear evenly, the order can also change every day ([priority rotation](Mk2_1phase_RFdatalog_temp/Readme.en.md#priority-rotation)).
- **Runs a heat pump, air conditioning, a pool pump... on the surplus.** These appliances cannot be dimmed: they are either on or off. The router switches them on through a relay when the surplus is large enough, and off when the house starts buying power again, without reacting to every passing cloud. Minimum ON and OFF times spare their compressor or motor. Works with a home battery too ([relays](Mk2_1phase_RFdatalog_temp/Readme.en.md#onoff-relay-output-configuration)).
- **Controls a distant water heater, wirelessly.** If the water heater is far from the electrical panel, a small unit next to it receives the router's orders by radio ([remote loads](Mk2_1phase_RFdatalog_temp/Readme.en.md#rf-module-and-remote-loads-configuration)).
- **Hot water even without sun.** If the day was not enough, the router heats during off-peak hours, at the time and for the duration you choose, and stops as soon as the water is hot enough ([off-peak hours](Mk2_1phase_RFdatalog_temp/Readme.en.md#off-peak-hours-management-and-scheduled-boost-dual-tariff)). A simple switch can also force full power at any time ([manual boost](Mk2_1phase_RFdatalog_temp/Readme.en.md#manual-boost-pin-triggered)).
- **Measures what it diverted**, with a second current sensor (CT2), reported on every serial output and on the display ([diverted energy](Mk2_1phase_RFdatalog_temp/Readme.en.md#diverted-energy-ct2), [display](Mk2_1phase_RFdatalog_temp/Readme.en.md#display)).
- **Knows the water temperature**, with one or more sensors ([temperature sensors](Mk2_1phase_RFdatalog_temp/Readme.en.md#temperature-sensors-configuration)).
- **Pauses while you are away**: either stop diverting (the surplus goes back to the grid, off-peak heating still works), or switch everything off, off-peak heating included ([routing stop](Mk2_1phase_RFdatalog_temp/Readme.en.md#routing-stop), [router OFF](Mk2_1phase_RFdatalog_temp/Readme.en.md#router-off)).
- **Everything in Home Assistant**: power, voltage, temperatures, state of the loads, through the mk2Wifi Wi-Fi module ([mk2Wifi](Mk2_1phase_RFdatalog_temp/Readme.en.md#configuration-with-the-mk2wifi-module)), or on a computer over USB ([serial output](Mk2_1phase_RFdatalog_temp/Readme.en.md#serial-output-type)).
- **Easy calibration**, with a dedicated mode in the firmware ([calibration](Mk2_1phase_RFdatalog_temp/Readme.en.md#router-calibration)).
- **Three boards**: Robin's Mk2 board with its 7-segment display, the second old single-phase board with an OLED, and the universal board ([motherboard](Mk2_1phase_RFdatalog_temp/Readme.en.md#motherboard)).
- **Works with zero-export installations too**, such as Enphase micro-inverters set to export nothing ([below](#zero-export-installations-enphase)).

## Getting started

1. **Hardware and wiring**: the [user documentation](https://fredm67.github.io/Mk2PVRouter/), the [installation guide](docs/HowToInstall.pdf), and the wiring diagrams in [docs](docs/).
2. **Configuration**: the settings are in `config.h`, `config_system.h` and `config_rf.h`, the calibration in `calibration.h`. The [configurator](configurator/) writes the settings from a web page: open `configurator/index.html` in a browser.
3. **Build and upload**: with [Visual Studio Code and PlatformIO](Mk2_1phase_RFdatalog_temp/Readme.en.md#use-with-visual-studio-code-recommended) (recommended) or the [Arduino IDE](Mk2_1phase_RFdatalog_temp/Readme.en.md#use-with-arduino-ide).
4. **Calibration**: see [router calibration](Mk2_1phase_RFdatalog_temp/Readme.en.md#router-calibration).

## Repository layout

| Folder                                                     | Contents                                                                                |
| ---------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| [`Mk2_1phase_RFdatalog_temp`](Mk2_1phase_RFdatalog_temp/)  | the router firmware, its configuration, its tests and its technical documentation       |
| [`RemoteLoadReceiver`](RemoteLoadReceiver/)                | the firmware of the remote load units (RFM69)                                           |
| [`configurator`](configurator/)                            | the web page that writes the configuration files                                        |
| [`sim`](sim/)                                              | the grid simulator: the firmware runs on a simulated single-phase installation (simavr) |
| [`dev`](dev/)                                              | test and calibration sketches: raw samples, transformer checker, display checks, ...    |
| [`obsolete`](obsolete/)                                    | the older sketches, frozen ([below](#the-older-sketches-frozen))                        |
| [`docs`](docs/)                                            | installation guide, wiring diagrams                                                     |

## Developer documentation

- [Code documentation](https://fredm67.github.io/PVRouter-1-phase/doc/) (Doxygen) and [firmware size and speed history](https://fredm67.github.io/PVRouter-1-phase/bench/), on the [project site](https://fredm67.github.io/PVRouter-1-phase/).
- [Changelog](CHANGELOG.md) of the changes on `dev`, not yet on `main`.
- [Technical documentation](Mk2_1phase_RFdatalog_temp/docs/README.md): architecture, interrupts, power calculation, calibration values, performance.
- [Tests](Mk2_1phase_RFdatalog_temp/test/README.md): native unit tests, and embedded tests and benchmarks in simavr.
- [Grid simulator](sim/README.md): regulation, ISR timing, switching instants, radio and relays, checked in CI.
- [Analysis tools](Mk2_1phase_RFdatalog_temp/Readme.en.md#analysis-documentation-and-tools): filtering of the relay outputs, tuning for cloudy climates and battery systems.

## Zero-export installations (Enphase)

With a zero-export profile, the PV system curtails its production as soon as it exceeds the consumption of the house: nothing is ever exported. The router therefore never sees any surplus, and has nothing to divert.

The answer is a negative `REQUIRED_EXPORT_IN_WATTS` (in `config_system.h`, see [its description](Mk2_1phase_RFdatalog_temp/Readme.en.md#required_export_in_watts-parameter)): the router then behaves as a small PV generator. With *-20*, it adds -20 W to every measurement:

- when the house **imports** energy, the measurement stays positive: the router does nothing;
- when the measurement is **around zero**, the zero-export limitation is active. With -20 W added, the router sees a surplus, and starts diverting to the water heater. The Envoy sees more consumption and raises the production; the router sees zero again, adds -20 W again, and diverts more. This goes on until the production reaches its maximum: the measurement then stays just above zero, and the system is stable.

Tested on a real installation with the 3-phase firmware, which handles this setting the same way. Depending on the installation, *-20* may need to be raised or lowered.

## The older sketches, frozen

These sketches, in [`obsolete/`](obsolete/), are kept as they are, for reference; they are no longer developed:

- ***Mk2_fasterControl_Full*** is a refactored version of the original sketch ***Mk2_fasterControl_twoLoads_x*** from Robin Emley: time-critical work in the ISR, serial data logging, temperature sensor, relays, telemetry for Home Assistant, universal PCB, delayed diversion start, and a display of the diverted energy (the temperature on the display is new in `Mk2_1phase_RFdatalog_temp`).
- ***Mk2_fasterControl_twoLoads_temp_1*** and ***Mk2_fasterControl_threeLoads_temp_1*** are single-file sketches. They carry the two-load logic of Robin's version 4, which he archived as problematic: prefer `Mk2_1phase_RFdatalog_temp`, which has the logic of his version 5.
- ***Robin/*** holds two of Robin Emley's sketches. His test and calibration tools stay in ***dev/***.

## Coming soon

- wiring diagram for mechanical thermostat
- wiring diagram for electronic thermostat (ACI)

## Roadmap

- the ESPHome YAML of the mk2Wifi module, written by the configurator;
- the CT1 filter values measured for each CT type (the shipped ones are interim);
- tests on real boards.

# Support

This project is maintained by [@FredM67](https://github.com/FredM67). Individual support by email is not possible; help shared publicly is also much more valuable, since everyone can benefit from it.

| Type                                   | Where                                                                         |
| -------------------------------------- | ----------------------------------------------------------------------------- |
| 🚨 **Bug reports**                     | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 📚 **Documentation issues**            | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 🎁 **Feature requests**                | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 🛡 **Report a security vulnerability** | See [SECURITY.md](SECURITY.md)                                                |
| 💬 **General questions**               | [GitHub Discussions](https://github.com/FredM67/PVRouter-1-phase/discussions) |

# Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for the code of conduct and the process for submitting pull requests.

# Authors and acknowledgment

- **Robin Emley** - the Mk2PVRouter, its boards and sketches - [mk2pvrouter.co.uk](https://web.archive.org/web/20231115123017/https://mk2pvrouter.co.uk/) (archived)
- **Frédéric Metrich** - this version - [FredM67](https://github.com/FredM67)

See also the list of [contributors](https://github.com/FredM67/PVRouter-1-phase/graphs/contributors) who participated in this project.

# License

GPL-3.0-or-later, see [LICENSE](LICENSE). [NOTICE](NOTICE) details where the code comes from: the firmware derives from the 3-phase firmware and from Robin Emley's Mk2PVRouter sketches, which he published without a license statement; the changes are under the GPL.

The files that hold your settings (`config.h`, `config_system.h`, `config_rf.h`, `calibration.h`) and every file the configurator writes are yours: use, change and share them on any terms you like.
