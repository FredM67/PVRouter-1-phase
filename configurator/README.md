# Configurator

A static web page that writes the firmware configuration of the **single-phase** router
(`Mk2_1phase_RFdatalog_temp`): `config.h`, `config_system.h` and `config_rf.h` for the router, and `config.h` and
`config_rf.h` for each remote unit (`RemoteLoadReceiver`). It does not write the ESPHome YAML for the mk2Wifi
module yet: that is the next step (`yaml.js` and its tests are in the repository, not used).

It is kept in sync with `Mk2_1phase_RFdatalog_temp` by a byte-for-byte test (see below). It started as a copy
of the configurator of PVRouter-3-phase and was adapted to this firmware; each configurator is tested against its
own firmware only, and the fixes of the 3-phase one are carried over by hand when they apply. It works offline:
open `index.html` in a browser.

The Mk2PVRouter documentation site publishes it at
<https://fredm67.github.io/Mk2PVRouter/configurateur-monophase/>. A link on the page switches to the
3-phase configurator (<https://fredm67.github.io/Mk2PVRouter/configurateur/>), and the other way round; each
refuses the saves of the other router (a save carries `router`: `1` here, `3` there).

No build step, no dependency: plain scripts that also load in Node for the tests.

| File          | Role                                                                     |
| ------------- | ------------------------------------------------------------------------ |
| `model.js`    | the settings, their defaults (= the shipped files) and derived values    |
| `validate.js` | the checks of `validation.h`, plus what the compiler cannot see          |
| `timeline.js` | the geometry of the boost timeline: windows, 15-minute snap, drag        |
| `generate.js` | model → the firmware files                                               |
| `yaml.js`     | model → the ESPHome YAML (fail-safe controls, sensors of what is sent)   |
| `i18n.js`     | English and French texts of the page                                     |
| `zip.js`      | "download all"                                                           |
| `folder.js`   | "save into the firmware folder" with a backup, and its undo (Chrome, Edge) |
| `app.js`      | the page                                                                 |

## Keeping it in sync with the firmware

The default choices must give back the shipped files byte for byte (`@date` aside). After a
change to `Mk2_1phase_RFdatalog_temp/config.h`, `config_system.h`, `config_rf.h` or to the
receiver's config files, update `generate.js` (and `model.js` for a new setting) until the
tests pass again.

```sh
node --test configurator/test/*.test.js           # unit tests
bash configurator/test/compile-presets.sh         # builds every preset (pio, clang-format)
bash configurator/test/check-yaml.sh              # 'esphome config' on the mk2Wifi presets (not used yet)
```

The presets (`test/presets/*.json`) are saves of the page: a new one is picked up by both
scripts.

## License

The configurator is GPL-3.0-or-later, as the rest of the project. The files it writes are yours: use, change and
share them on any terms you like (see [NOTICE](../NOTICE)).
