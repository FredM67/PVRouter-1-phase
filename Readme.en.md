<div align = center>

[![CI](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/build.yml?branch=dev&label=CI&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/build.yml)
[![relay simulation](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/relay-sim.yml?branch=dev&label=relay%20simulation&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/relay-sim.yml)
[![CodeQL](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/codeql.yml?branch=dev&label=CodeQL&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/codeql.yml)
[![doc](https://img.shields.io/github/actions/workflow/status/FredM67/PVRouter-1-phase/doxygen-gh-pages.yml?branch=main&label=doc&logo=github)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/doxygen-gh-pages.yml)
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

My version of the single-phase Mk2PVRouter firmware (see <http://www.mk2pvrouter.co.uk>): it diverts the surplus of a photovoltaic installation to resistive loads, such as water heaters, instead of exporting it to the grid.

---
**_NOTE:_**

For a three-phase version, please see [PVRouter-3-phase](https://github.com/FredM67/PVRouter-3-phase).

---

## Mk2_1phase_RFdatalog_temp, the firmware

***[Mk2_1phase_RFdatalog_temp](Mk2_1phase_RFdatalog_temp/)*** is the single-phase firmware, the one developed further. It has the features and the settings of the 3-phase firmware, `Mk2_3phase_RFdatalog_temp`, which it derives from, with the control core of Robin Emley's `Mk2_fasterControl_twoLoads_5`:

- three boards: Robin's Mk2 board with its 7-segment display (`OLD_SEG`), the second old single-phase board with an OLED (`OLD_OLED`), and the universal board (`NEW`);
- the diverted energy measured with a second CT (CT2), reported on every serial output and on the display;
- TRIAC outputs, relay outputs with their filter and minimum times (battery mode included), remote loads over RF (RFM69);
- dual tariff with scheduled boosts, priority rotation with rotation groups, override pins, routing stop and router OFF pins;
- DS18B20 temperature sensors, watchdog LED, 7-segment or OLED display;
- serial output for the serial monitor, for Home Assistant through the mk2Wifi module (ESPHome), or in JSON;
- every setting checked at compile time; native tests, tests in a simulated ATmega328P, and a grid simulator that runs the firmware on a simulated single-phase installation, all run by CI.

> [!WARNING]
> **Not yet tested on a real board.** This firmware passes its native tests, its compile-time checks and the grid simulator on the three boards, but it has not run on real hardware yet. Until it has, use `Mk2_fasterControl_Full` on an installation in service.

Documentation: [English](Mk2_1phase_RFdatalog_temp/Readme.en.md), [français](Mk2_1phase_RFdatalog_temp/Readme.md). Changes: [CHANGELOG.md](CHANGELOG.md). Code documentation (Doxygen) and firmware size and speed history: [project site](https://fredm67.github.io/PVRouter-1-phase/).

## The older sketches, frozen

These sketches are kept as they are, for reference; they are no longer developed:

- ***Mk2_fasterControl_Full*** is a refactored version of the original sketch ***Mk2_fasterControl_twoLoads_x*** from Robin Emley: time-critical work in the ISR, serial data logging, temperature sensor, relays, telemetry for Home Assistant, universal PCB, delayed diversion start, and a display of the diverted energy (the temperature on the display is new in `Mk2_1phase_RFdatalog_temp`).
- ***Mk2_fasterControl_twoLoads_temp_1*** and ***Mk2_fasterControl_threeLoads_temp_1*** are single-file sketches. They carry the two-load logic of Robin's version 4, which he archived as problematic: prefer `Mk2_1phase_RFdatalog_temp`, which has the logic of his version 5.
- ***Robin/*** holds two of Robin Emley's sketches, ***dev/*** his test and calibration tools.

## Coming soon
- wiring diagram for mechanical thermostat
- wiring diagram for electronic thermostat (ACI)

## Support

This project is maintained by [@FredM67](https://github.com/FredM67). Please understand that we won't be able to provide individual support via email. We also believe that help is much more valuable if it's shared publicly, so that more people can benefit from it.

| Type                                  | Platforms                                                                     |
| ------------------------------------- | ----------------------------------------------------------------------------- |
| 🚨 **Bug Reports**                     | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 📚 **Docs Issue**                      | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 🎁 **Feature Requests**                | [GitHub Issue Tracker](https://github.com/FredM67/PVRouter-1-phase/issues)    |
| 🛡 **Report a security vulnerability** | See [SECURITY.md](SECURITY.md)                                                |
| 💬 **General Questions**               | [GitHub Discussions](https://github.com/FredM67/PVRouter-1-phase/discussions) |

## Roadmap

- the single-phase mode of the configurator, and the ESPHome packages of the mk2Wifi module;
- the CT1 filter values measured for each CT type (the shipped ones are interim);
- tests on real boards.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors and acknowledgment

- **Robin Emley** - the Mk2PVRouter, its boards and sketches - [www.mk2pvrouter.co.uk](http://www.mk2pvrouter.co.uk)
- **Frédéric Metrich** - this version - [FredM67](https://github.com/FredM67)

See also the list of [contributors](https://github.com/FredM67/PVRouter-1-phase/graphs/contributors) who participated in this project.

## License

GPL-3.0-or-later, see [LICENSE](LICENSE). [NOTICE](NOTICE) details where the code comes from: the firmware derives from the 3-phase firmware and from Robin Emley's Mk2PVRouter sketches, which he published without a license statement; the changes are under the GPL.

The files that hold your settings (`config.h`, `config_system.h`, `config_rf.h`, `calibration.h`) are yours: use, change and share them on any terms you like.
