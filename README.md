<div align = center>

[![GitHub issues](https://img.shields.io/github/issues/FredM67/PVRouter-1-phase)](https://github.com/FredM67/PVRouter-1-phase/issues)
[![GitHub forks](https://img.shields.io/github/forks/FredM67/PVRouter-1-phase)](https://github.com/FredM67/PVRouter-1-phase/network)
[![GitHub stars](https://img.shields.io/github/stars/FredM67/PVRouter-1-phase)](https://github.com/FredM67/PVRouter-1-phase/stargazers)
[![CodeQL](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/codeql.yml/badge.svg)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/codeql.yml)
[![Doxygen](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/doxygen-gh-pages.yml/badge.svg)](https://github.com/FredM67/PVRouter-1-phase/actions/workflows/doxygen-gh-pages.yml)
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)
<br/>
<br/>
  [![en](https://img.shields.io/badge/lang-en-red.svg)](Readme.en.md)
  [![fr](https://img.shields.io/badge/lang-fr-blue.svg)](Readme.md)
</div>

# PVRouter (single-phase version)

My version of the single-phase Mk2PVRouter firmware (see <http://www.mk2pvrouter.co.uk>)

Robin Emley already proposes a single phase PV-router.
It supports one or two resistive output loads, which are completely independent.

---
**_NOTE:_**

For a three-phase version, please see [PVRouter-3-phase](https://github.com/FredM67/PVRouter-3-phase).

---

***Mk2_fasterControl_Full*** is a refactored version of the original sketch ***Mk2_fasterControl_twoLoads_x*** from Robin Emley.

Main changes in this version:

- Time-critical work is now in the ISR.
- Serial data logging has been added
- Temperature sensor has been added.
- Support for relays
- Code documentation
- Telemetry for Home Assistant
- Support of universal PCB
- Delayed diversion start

On this version, the display will alternatively shows the diverted energy or the temperature of the water heater (Dallas temperature sensor needed).

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

No changes are currently planned.

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors and acknowledgment

- **Frédéric Metrich** - _Initial work_ - [FredM67](https://github.com/FredM67)

See also the list of [contributors](https://github.com/FredM67/PVRouter-1-phase/graphs/contributors) who participated in this project.
