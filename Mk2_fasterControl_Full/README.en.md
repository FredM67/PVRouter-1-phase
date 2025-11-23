[![en](https://img.shields.io/badge/lang-en-red.svg)](README.en.md)
[![fr](https://img.shields.io/badge/lang-fr-blue.svg)](README.md)

This program is designed to be used with the Arduino IDE and/or other development IDEs like VSCode + PlatformIO.

- [Use with Arduino IDE](#use-with-arduino-ide)
  - [Required Libraries](#required-libraries)
- [Use with Visual Studio Code](#use-with-visual-studio-code)
- [Quick Overview of the Files](#quick-overview-of-the-files)
- [Development Documentation](#development-documentation)
- [Router Calibration](#router-calibration)
- [Program Configuration](#program-configuration)
  - [PCB Version Configuration](#pcb-version-configuration)
  - [Serial Output Type](#serial-output-type)
  - [Display Configuration](#display-configuration)
  - [TRIAC Output Configuration](#triac-output-configuration)
  - [On-Off Relay Output Configuration](#on-off-relay-output-configuration)
    - [Operating Principle](#operating-principle)
  - [Watchdog Configuration](#watchdog-configuration)
  - [Temperature Sensor Configuration](#temperature-sensor-configuration)
    - [Enabling the Feature](#enabling-the-feature)
      - [With Arduino IDE](#with-arduino-ide)
      - [With Visual Studio Code and PlatformIO](#with-visual-studio-code-and-platformio)
    - [Sensor Configuration (common to both cases)](#sensor-configuration-common-to-both-cases)
  - [Dual Tariff Configuration (Off-Peak Hours Management)](#dual-tariff-configuration-off-peak-hours-management)
    - [Hardware Configuration](#hardware-configuration)
    - [Software Configuration](#software-configuration)
  - [Priority Rotation](#priority-rotation)
  - [Forced Operation Configuration](#forced-operation-configuration)
  - [Diversion Shutdown](#diversion-shutdown)
- [Advanced Program Configuration](#advanced-program-configuration)
  - [Parameter `DIVERSION_START_THRESHOLD_WATTS`](#parameter-diversion_start_threshold_watts)
  - [Parameter `REQUIRED_EXPORT_IN_WATTS`](#parameter-required_export_in_watts)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

# Use with Arduino IDE

To use this program with the Arduino IDE, you need to download and install the latest version of the Arduino IDE. Choose the "standard" version, NOT the Microsoft Store version. Opt for the "Win 10 and newer, 64 bits" or "MSI installer" version.

Since the code is optimized with one of the latest C++ standards, you need to modify a configuration file to enable C++17. You'll find the '**platform.txt**' file in the Arduino IDE installation path.

For **Windows**, you'll typically find the file in '**C:\Program Files (x86)\Arduino\hardware\arduino\avr**' and/or in '**%LOCALAPPDATA%\Arduino15\packages\arduino\hardware\avr\x.y.z**' where '**x.y.z**' is the version of the Arduino AVR Boards package.

You can also run this command in Powershell: `Get-Childitem –Path C:\ -Include platform.txt -Recurse -ErrorAction SilentlyContinue`. It may take a few seconds/minutes until the file is found.

For **Linux**, if using the AppImage package, you'll find this file in '~/.arduino15/packages/arduino/hardware/avr/1.8.6'. You can run `find / -name platform.txt 2>/dev/null` in case the location has changed.

For **MacOSX**, this file is located in '/Users/[user]/Library/Arduino15/packages/arduino/hardware/avr/1.8.6'.

Open the file in any text editor (you'll need administrator rights) and replace the parameter '**-std=gnu++11**' with '**-std=gnu++17**'. That's it!

If your Arduino IDE was open, please close all instances and reopen it.

## Required Libraries

To use the project, and depending on the program configuration, you will need the following libraries:
- ArduinoJson: you should stay with version 6.*. Version 7 is not suitable for the Atmega328P (Arduino Uno).
- U8g2
- OneWire

# Use with Visual Studio Code

You'll need to install additional extensions. The most popular and used extensions for this job are '*Arduino*' and '*Platform IO*'.
The entire project has been designed to be used optimally with *Platform IO*.

# Quick Overview of the Files

- **Mk2_fasterControl_Full.ino** : This file is needed for Arduino IDE
- **calibration.h** : contains the calibration parameters
- **config.h** : the user's preferences are stored here (pin assignments, features, ...)
- **config_system.h** : rarely modified system constants
- **constants.h** : some constants — *do not edit*
- **debug.h** : some macros for serial output and debugging
- **dualtariff.h** : dual tariff function definitions
- **ewma_avg.h** : modified EWMA average calculation functions
- **main.cpp** : main source code
- **movingAvg.h** : source code for moving average
- **processing.cpp** : source code for the processing engine
- **processing.h** : functions prototypes for the processing engine
- **README.md** : this file
- **teleinfo.h**: source code for *IoT Telemetry* feature
- **types.h** : type definitions ...
- **type_traits.h** : some STL stuff not yet available in the avr package
- **type_traits** : contains missing STL templates
- **utils_display.h** : source code for *7-segment display* feature
- **utils_dualtariff.h** : source code for *Off-Peak Hours management* feature
- **utils_oled.h** : source code for *OLED I2C display* feature
- **utils_pins.h** : some functions for direct access to microcontroller inputs/outputs
- **utils_relay.h** : source code for *relay diversion* feature
- **utils_temp.h** : source code for *Temperature* feature
- **utils.h** : helper functions and miscellaneous stuff
- **validation.h** : parameter validation, this code is only executed at compile time!
- **platformio.ini** : PlatformIO settings
- **inject_sketch_name.py** : helper script for PlatformIO
- **Doxyfile** : parameter for Doxygen (code documentation)

The end-user should ONLY edit the **calibration.h** and **config.h** files.

# Development Documentation

You can start reading the documentation here [1-phase router](https://fredm67.github.io/PVRouter-1-phase/) (in English).

# Router Calibration
The calibration values are found in the **calibration.h** file.
These are the lines:
```cpp
inline constexpr float powerCal_grid{ 0.0435f };
inline constexpr float powerCal_diverted{ 0.0435f };
```

These default values do not affect the router's operation.

However, they must be determined precisely if you want to have a display consistent with reality.

# Program Configuration

Configuring a feature generally follows two steps:
- Enabling the feature
- Configuring the feature's parameters

Configuration consistency is checked during compilation. For example, if a *pin* is allocated twice by mistake, the compiler will generate an error.

## PCB Version Configuration

The router exists in two PCB (printed circuit board) versions, which use different analog pins for voltage and current sensors. It is important to configure this option correctly according to the PCB version you have.

To enable the old PCB (default):
```cpp
inline constexpr bool OLD_PCB{ true };
```

To use the new PCB:
```cpp
inline constexpr bool OLD_PCB{ false };
```

This configuration automatically modifies the analog pin assignments:
- **Old PCB** : voltage sensor on A3, CT1 (grid) on A5, CT2 (diversion) on A4
- **New PCB** : voltage sensor on A0, CT1 (grid) on A1, CT2 (diversion) on A3

---
> [!WARNING]
> Incorrect configuration of this parameter will prevent the router from functioning correctly, as the sensors will not be read on the correct analog pins.
---

## Serial Output Type

The serial output type can be configured to suit different needs. Three options are available:

- **HumanReadable** : Human-readable output, ideal for debugging or commissioning.
- **IoT** : Output formatted for IoT platforms like HomeAssistant.
- **JSON** : Output formatted for platforms like EmonCMS (JSON).

To configure the serial output type, modify the following constant in the **config.h** file:
```cpp
inline constexpr SerialOutputType SERIAL_OUTPUT_TYPE = SerialOutputType::HumanReadable;
```
Replace `HumanReadable` with `IoT` or `JSON` according to your needs.

## Display Configuration

Configure the display type in `config.h`:
```cpp
inline constexpr DisplayType TYPE_OF_DISPLAY{ DisplayType::NONE };
```

Possible options are:
- **DisplayType::NONE** : No display is used.
- **DisplayType::OLED** : Uses an OLED screen to display information.
- **DisplayType::SEG** : Uses a segment display to display information.
- **DisplayType::SEG_HW** : Uses a segment display with specific hardware interface to display information (presence of **IC3** and **IC4** circuits).

---
> [!NOTE]
> The OLED display is not yet available. It requires a new PCB version that will be available soon.
---

## TRIAC Output Configuration

The first step is to define the number of TRIAC outputs:

```cpp
inline constexpr uint8_t NO_OF_DUMPLOADS{ 2 };
```

Then, you need to assign the corresponding *pins* as well as the priority order at startup.
```cpp
inline constexpr uint8_t physicalLoadPin[NO_OF_DUMPLOADS]{ 5, 7 };
inline constexpr uint8_t loadPrioritiesAtStartup[NO_OF_DUMPLOADS]{ 0, 1 };
```

## On-Off Relay Output Configuration
On-off relay outputs allow powering devices that contain electronics (heat pump, etc.).

You need to enable the feature like this:
```cpp
inline constexpr bool RELAY_DIVERSION{ true };
```

Each relay requires the definition of five parameters:
- the **pin** number to which the relay is connected
- the **surplus threshold** before starting (default **1000 W**)
- the **import threshold** before stopping (default **200 W**)
- the **minimum operating duration** in minutes (default **5 min**)
- the **minimum stop duration** in minutes (default **5 min**).

Example relay configuration:
```cpp
inline constexpr RelayEngine relays{ { { 4, 1000, 200, 10, 10 } } };
```
In this example, the relay is connected to *pin* **4**, it will trigger from **1000 W** surplus, stop from **200 W** import, and has a minimum operating and stop duration of **10 min**.

To configure multiple relays, simply list each relay's configuration:
```cpp
inline constexpr RelayEngine relays{ { { 4, 1000, 200, 10, 10 },
                                       { 3, 1500, 250, 5, 15 } } };
```
Relays are activated in list order and deactivated in reverse order.
In all cases, minimum operating and stop durations are always respected.

### Operating Principle
Surplus and import thresholds are calculated using an exponentially weighted moving average (EWMA), in our specific case, it's a modification of a triple exponentially weighted moving average (TEMA).
By default, this average is calculated over a window of about **10 min**. You can adjust this duration to suit your needs.
It's possible to lengthen it but also to shorten it.
For Arduino performance reasons, the chosen duration will be rounded to a nearby duration that will allow calculations without impacting router performance.

If the user prefers a 15 min window, just write:
```cpp
inline constexpr RelayEngine relays{ 15_i, { { 3, 1000, 200, 1, 1 } } };
```
___
> [!NOTE]
> Pay attention to the '**_i**' suffix after the number *15*!
___

The relays configured in the system are managed by a system similar to a state machine.
Every second, the system increases the duration of the current state of each relay and proceeds with all relays based on the current average power:
- if the current average power is above the import threshold, it tries to turn off some relays.
- if the current average power is above the surplus threshold, it tries to turn on more relays.

Relays are processed in ascending order for surplus and in descending order for import.

For each relay, the transition or state change is managed as follows:
- if the relay is *OFF* and the current average power is below the surplus threshold, the relay tries to switch to *ON* state. This transition is subject to the condition that the relay has been *OFF* for at least the *minOFF* duration.
- if the relay is *ON* and the current average power is above the import threshold, the relay tries to switch to *OFF* state. This transition is subject to the condition that the relay has been *ON* for at least the *minON* duration.

## Watchdog Configuration
A watchdog is an electronic circuit or software used in digital electronics to ensure that an automaton or computer does not remain stuck at a particular stage of the processing it is performing.

This is accomplished using an LED that blinks at a frequency of 1 Hz, or every second.
Thus, the user knows on one hand if their router is powered on, and if this LED stops blinking, it means the Arduino has frozen (a case never encountered yet!).
A simple press of the *Reset* button will restart the system without unplugging anything.

You need to enable the feature like this:
```cpp
inline constexpr bool WATCHDOG_PIN_PRESENT{ true };
```
and define the *pin* used, in the example *9*:
```cpp
inline constexpr uint8_t watchDogPin{ 9 };
```

## Temperature Sensor Configuration
It's possible to connect one or more Dallas DS18B20 temperature sensors.
These sensors can be used for informational purposes or to control forced operation mode.

To enable this feature, the procedure differs depending on whether you use the Arduino IDE or Visual Studio Code with the PlatformIO extension.

### Enabling the Feature

To enable this feature, the procedure differs depending on whether you use the Arduino IDE or Visual Studio Code with the PlatformIO extension.

#### With Arduino IDE
Enable the following line by removing the comment:
```cpp
#define TEMP_ENABLED
```

If the *OneWire* library is not installed, install it via the **Tools** => **Manage Libraries...** menu.
Search for "Onewire" and install "**OneWire** by Jim Studt, ..." version **2.3.7** or newer.

#### With Visual Studio Code and PlatformIO
Select the "**env:temperature (Mk2_3phase_RFdatalog_temp)**" configuration.

### Sensor Configuration (common to both cases)
To configure the sensors, you need to enter their addresses.
Use a program to scan the connected sensors.
You can find such programs on the Internet or among the examples provided with the Arduino IDE.
It's recommended to stick a label with each sensor's address on its cable.

Enter the addresses as follows:
```cpp
inline constexpr TemperatureSensing temperatureSensing{ 4,
                                                        { { 0x28, 0xBE, 0x41, 0x6B, 0x09, 0x00, 0x00, 0xA4 },
                                                          { 0x28, 0x1B, 0xD7, 0x6A, 0x09, 0x00, 0x00, 0xB7 } } };
```
The number *4* as the first parameter is the *pin* that the user will have chosen for the *OneWire* bus.

___
> [!NOTE]
> Multiple sensors can be connected on the same cable.
> On the Internet you'll find all the details regarding the topology usable with this type of sensor.
___

## Dual Tariff Configuration (Off-Peak Hours Management)
It's possible to entrust off-peak hours management to the router.
This allows, for example, limiting forced heating to avoid heating water too much with the goal of using surplus the next morning.
This limit can be in duration or temperature (requires using a Dallas DS18B20 temperature sensor).

### Hardware Configuration
Disconnect the Day/Night contactor control, which is no longer necessary.
Connect directly a chosen *pin* to the meter's dry contact (*C1* and *C2* terminals).
___
> [!WARNING]
> You must connect **directly**, a *pin/ground* pair with the meter's *C1/C2* terminals.
> There must NOT be 230 V on this circuit!
___

### Software Configuration
Enable the feature as follows:
```cpp
inline constexpr bool DUAL_TARIFF{ true };
```
Configure the *pin* to which the meter is connected:
```cpp
inline constexpr uint8_t dualTariffPin{ 3 };
```

Configure the duration in *hours* of the off-peak period (for now, only one period per day is supported):
```cpp
inline constexpr uint8_t ul_OFF_PEAK_DURATION{ 8 };
```

Finally, define the operating modalities during the off-peak period:
```cpp
inline constexpr pairForceLoad rg_ForceLoad[NO_OF_DUMPLOADS]{ { -3, 2 } };
```
It's possible to define a configuration for each load independently of the others.
The first parameter of *rg_ForceLoad* determines the start delay relative to the beginning or end of off-peak hours:
- if the number is positive and less than 24, it's the number of hours,
- if the number is negative greater than −24, it's the number of hours relative to the end of off-peak hours
- if the number is positive and greater than 24, it's the number of minutes,
- if the number is negative less than −24, it's the number of minutes relative to the end of off-peak hours

The second parameter determines the forced operation duration:
- if the number is less than 24, it's the number of hours,
- if the number is greater than 24, it's the number of minutes.

Examples for better understanding (with off-peak start at 23:00, until 7:00 i.e. 8 h duration):
- ```{ -3, 2 }``` : start **3 hours BEFORE** the end of period (at 4 am), for a duration of 2 h.
- ```{ 3, 2 }``` : start **3 hours AFTER** the beginning of period (at 2 am), for a duration of 2 h.
- ```{ -150, 2 }``` : start **150 minutes BEFORE** the end of period (at 4:30), for a duration of 2 h.
- ```{ 3, 180 }``` : start **3 hours AFTER** the beginning of period (at 2 am), for a duration of 180 min.

For *infinite* duration (therefore until the end of the off-peak period), use ```UINT16_MAX``` as the second parameter:
- ```{ -3, UINT16_MAX }``` : start **3 hours BEFORE** the end of period (at 4 am) with forced operation until the end of off-peak period.

If your system consists of 2 outputs (```NO_OF_DUMPLOADS``` will then have a value of 2), and you only want forced operation on the 2nd output, write:
```cpp
inline constexpr pairForceLoad rg_ForceLoad[NO_OF_DUMPLOADS]{ { 0, 0 },
                                                              { -3, 2 } };
```

## Priority Rotation
Priority rotation is useful when powering a three-phase water heater.
It allows balancing the operating duration of different heating elements over an extended period.

But it can also be interesting if you want to swap the priorities of two devices each day (two water heaters, etc.).

Once again, enabling this function has 2 modes:
- **automatic**, specify:
```cpp
inline constexpr RotationModes PRIORITY_ROTATION{ RotationModes::AUTO };
```
- **manual**, then write:
```cpp
inline constexpr RotationModes PRIORITY_ROTATION{ RotationModes::PIN };
```
In **automatic** mode, rotation happens automatically every 24 hours.
In **manual** mode, you must also define the *pin* that will trigger rotation:
```cpp
inline constexpr uint8_t rotationPin{ 10 };
```

## Forced Operation Configuration
It's possible to trigger forced operation (some routers call this function *Boost*) via a *pin*.
You can connect a micro-switch, a timer (WARNING, NO 230 V on this line), or any other dry contact.

To enable this feature, use the following code:
```cpp
inline constexpr bool OVERRIDE_PIN_PRESENT{ true };
```
You must also specify the *pin* to which the dry contact is connected:
```cpp
inline constexpr uint8_t forcePin{ 11 };
```

## Diversion Shutdown
It can be convenient to disable routing during a prolonged absence.
This feature is particularly useful if the control *pin* is connected to a dry contact that can be remotely controlled, for example via an Alexa routine or similar.
Thus, you can disable routing during your absence and reactivate it one or two days before your return, to have hot water (free) available upon arrival.

To enable this feature, use the following code:
```cpp
inline constexpr bool DIVERSION_PIN_PRESENT{ true };
```
You must also specify the *pin* to which the dry contact is connected:
```cpp
inline constexpr uint8_t diversionPin{ 12 };
```

# Advanced Program Configuration

These parameters are found in the `config_system.h` file.

## Parameter `DIVERSION_START_THRESHOLD_WATTS`
The `DIVERSION_START_THRESHOLD_WATTS` parameter defines a surplus threshold before any routing to the loads configured on the router. It's primarily intended for installations with storage batteries.
By default, this value is set to 0 W.
By setting this parameter to 50 W for example, the router will only start routing when 50 W surplus is available. Once routing has started, all surplus will be routed.
This feature allows establishing a clear hierarchy in the use of produced energy, favoring energy storage over immediate consumption. You can adjust this value according to battery charging system responsiveness and your energy use priorities.

> [!IMPORTANT]
> This parameter only concerns the routing start condition.
> Once the threshold is reached and routing has started, **all** surplus becomes available for loads.

## Parameter `REQUIRED_EXPORT_IN_WATTS`
The `REQUIRED_EXPORT_IN_WATTS` parameter determines the minimum amount of energy the system must reserve for export or import to the electrical grid before diverting surplus to controlled loads.
By default set to 0 W, this parameter can be used to guarantee constant export to the grid, for example to comply with electricity resale agreements.
A negative value will force the router to consume this power from the grid. This can be useful or even necessary for installations configured in *zero injection* to prime solar production.

> [!IMPORTANT]
> Unlike the first parameter, this one represents a permanent offset that is continuously subtracted from available surplus.
> If set to 20 W for example, the system will **always** reserve 20 W for export, regardless of other conditions.

# Troubleshooting
- Ensure all required libraries are installed.
- Verify correct configuration of pins and parameters.
- Check the serial output for error messages.

# Contributing
Contributions are welcome! Please submit issues, feature requests, and pull requests via GitHub.

*unfinished doc*
