# PVRouter (single-phase version)

My version of the single-phase Mk2PVRouter firmware (see <http://www.mk2pvrouter.co.uk>)

Robin Emley already proposes a single phase PV-router.
It supports one or two resistive output loads, which are completely independent.

---
**_NOTE:_**

For a three-phase version, please see [PVRouter-3-phase](https://github.com/FredM67/PVRouter-3-phase).

---

***Mk2_fasterControl_twoLoads_temp_1*** is a refactored version of the original sketch ***Mk2_fasterControl_twoLoads_2*** from Robin Emley.

Main changes in this version:

- Time-critical work is now in the ISR.
- Serial data logging has been added
- Temperature sensor has been added.

On this version, the display will alternatively shows the diverted energy or the temperature of the water heater (Dallas temperature sensor needed).

## Coming soon
- wiring diagram for mechanical thermostat
- wiring diagram for electronic thermostat (ACI)
- code documentation