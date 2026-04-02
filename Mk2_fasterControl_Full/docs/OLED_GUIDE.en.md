# OLED Display & Rotary Encoder — User Guide

When `DisplayType::OLED` is selected, the system uses a **128x64 SSD1306 I2C OLED display** (on A4/A5) paired with a **rotary encoder with push button** for navigating pages and editing settings at runtime.

## Page Carousel

The display is organized as a **circular carousel of pages**, built dynamically at startup based on compile-time configuration. Rotate the encoder to scroll through pages.

| Page | Condition | Content |
|------|-----------|---------|
| **ENERGY** | always | Diverted energy in kWh (large font) |
| **GRID** | always | Voltage, instantaneous grid power, relay average power, diverted power |
| **TEMP** | `TEMP_SENSOR_PRESENT` | Up to 6 temperature sensor readings |
| **ROUTING** | always | Per-group diversion ON/OFF toggles |
| **BOOST** | one per visible boost entry | Per-channel boost ON/OFF toggle |
| **RELAY CFG** | one per relay, if `RELAY_DIVERSION` | Edit surplus/import thresholds and min ON/OFF timers |
| **SYSTEM CFG** | `OLED_ENABLE_RUNTIME_SETTINGS` | Edit `REQUIRED_EXPORT_IN_WATTS` and `DIVERSION_START_THRESHOLD_WATTS` |
| **RESTART** | `OLED_ENABLE_RESTART_PAGE` | Software restart via watchdog |

Only pages relevant to the current configuration are shown. For example, TEMP pages only appear if temperature sensors are configured, RELAY CFG pages only appear if relay diversion is enabled, and BOOST pages only appear for entries with `visibleOnOLED: true`.

## Interaction Model

The UI has three modes, controlled by the encoder rotation and button presses:

### 1. VIEW mode (default)
- **Rotate**: scroll through pages (wraps around)
- **Short press**: enter the page's action (depends on page type)
- **Long press (3 s)**: return to home page (ENERGY)

### 2. LIST_NAV mode (ROUTING, RELAY CFG, SYSTEM CFG pages)
- **Rotate**: move cursor (`>`) between items in the list
- **Short press on ROUTING page**: toggle the selected diversion group ON/OFF
- **Short press on RELAY CFG / SYSTEM CFG page**:
  - On a value item: enter VALUE_EDIT mode
  - On "SAVE": save all values to EEPROM and return to VIEW
- **Long press (3 s)**: cancel any unsaved edit and return home

### 3. VALUE_EDIT mode (RELAY CFG, SYSTEM CFG pages)
- **Rotate**: adjust the selected value
  - Thresholds in watts: step = **10 W** per detent
  - Timers in minutes: step = **1 min** per detent
- **Short press**: confirm the edit, return to LIST_NAV
- **Long press (3 s)**: **cancel** the edit (restore previous value) and return home

### Visual Indicators

- `>` prefix: cursor points to this item (LIST_NAV mode)
- `*` prefix: this item is currently being edited (VALUE_EDIT mode)

```
ROUTING page example          RELAY CFG page example

   ╔════════════════╗            ╔════════════════╗
   ║    ROUTAGE     ║            ║   RELAIS 1     ║
   ║                ║            ║  > SM  : 1000W ║
   ║  >Div 1  ON   ║            ║    SA  :  200W ║
   ║   Div 2  OFF  ║            ║    Ton :  10m  ║
   ║   Div 3  ON   ║            ║    Toff:  10m  ║
   ║                ║            ║    VALIDER     ║
   ║  Act. / 3s bk ║            ║  Act. / 3s bk  ║
   ╚════════════════╝            ╚════════════════╝
```

## Boost Pages

Each boost entry with `visibleOnOLED: true` gets its own BOOST page. The page shows:
- The boost channel number
- The target output (TRIAC or relay)
- The current state (ON/OFF)

A **short press** toggles the boost ON/OFF immediately. The boost state is OR'ed with the physical pin state: if either the OLED toggle or the hardware pin is active, the boost is ON.

## Routing Page

Each diversion group with `visibleOnOLED: true` appears as a line on the ROUTING page. A short press enters list navigation, then a second press toggles the selected group. The diversion state is AND'ed with the physical pin state: both the OLED toggle and the hardware pin must be active for routing to be authorized.

## EEPROM Persistence

When `OLED_ENABLE_RUNTIME_SETTINGS` is `true`, editable values (relay thresholds/timers, system watt settings) are saved to EEPROM when the user confirms via the "SAVE" item. The EEPROM data includes a signature, version byte, and checksum. On boot, saved values are loaded if valid; otherwise, compile-time defaults from `config.h` are used.

> [!NOTE]
> Boost and diversion toggle states are **not** saved to EEPROM. They reset to their defaults on each reboot (all boosts OFF, all diversions authorized).
