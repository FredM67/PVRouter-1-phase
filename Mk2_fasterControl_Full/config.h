/**
 * @file config.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Main end-user configuration file
 * @version 0.3
 * @date 2026-03-25
 *
 * @copyright Copyright (c) 2025
 *
 * --------------------------------------------------------------------------------------
 * QUICK GUIDE
 * --------------------------------------------------------------------------------------
 * 1) Start with the "QUICK USER SETTINGS" section.
 * 2) Then review the "OUTPUTS / INPUTS / OLED UI" sections.
 * 3) The two base routing thresholds are still defined in config_system.h:
 *      - REQUIRED_EXPORT_IN_WATTS
 *      - DIVERSION_START_THRESHOLD_WATTS
 *    Those values can also be edited later from the OLED when enabled.
 *
 * IMPORTANT
 * --------------------------------------------------------------------------------------
 * - Keep the same constant names used by the firmware.
 * - Use 0xff or unused_pin when a pin or option is not used.
 */

#ifndef CONFIG_H
#define CONFIG_H

//======================================================================================
// Optional modules / serial outputs
//======================================================================================
//#define RF_PRESENT     /**< Uncomment only if an RFM12B module is installed */
//#define EMONESP        /**< Uncomment only if an ESP WiFi module is installed */

#define ENABLE_DEBUG    /**< Enable debug print statements */
#define SERIALPRINT     /**< Human readable serial commissioning output */
#define SERIALOUT       /**< Wired serial output enabled */

#include "config_system.h"
#include "config_types.h"
#include "debug.h"
#include "types.h"
#include "utils_dualtariff.h"
#include "utils_relay.h"

//======================================================================================
// QUICK USER SETTINGS
//======================================================================================
inline constexpr SerialOutputType SERIAL_OUTPUT_TYPE = SerialOutputType::HumanReadable;

inline constexpr uint8_t NO_OF_DUMPLOADS{ 1 };           /**< Number of TRIAC dump-load outputs */
inline constexpr bool EMONESP_CONTROL{ false };
inline constexpr bool OLD_PCB{ false };

inline constexpr bool WATCHDOG_PIN_PRESENT{ true };
inline constexpr bool RELAY_DIVERSION{ true };
inline constexpr bool DUAL_TARIFF{ false };
inline constexpr bool TEMP_SENSOR_PRESENT{ false };

inline constexpr RotationModes PRIORITY_ROTATION{ RotationModes::OFF };
inline constexpr DisplayType TYPE_OF_DISPLAY{ DisplayType::OLED };

// Backward compatibility flag kept enabled automatically when related tables are used.
inline constexpr bool OVERRIDE_PIN_PRESENT{ true };

#include "utils_temp.h"

//======================================================================================
// DISPLAY REMINDER
//======================================================================================
// OLED uses I2C on A4/A5 on Arduino UNO.
//
// 7-segment displays consume many digital pins. If you switch away from OLED,
// re-check the pins below carefully.

//======================================================================================
// OUTPUTS - TRIAC / dump-load outputs
//======================================================================================
inline constexpr uint8_t physicalLoadPin[NO_OF_DUMPLOADS]{ 2 };
inline constexpr uint8_t loadPrioritiesAtStartup[NO_OF_DUMPLOADS]{ 0 };

//======================================================================================
// OUTPUTS - RELAY diversion outputs
// -------------------------------------------------------------------------------------
// Each entry = { pin, surplusThreshold, importThreshold, minON, minOFF }
//
//   pin              : digital output pin driving the relay
//   surplusThreshold : surplus power (W) to turn ON
//   importThreshold  : import power (W) to turn OFF
//   minON            : minimum ON time in minutes
//   minOFF           : minimum OFF time in minutes
//
// Example:
//   inline constexpr RelayEngine relays{
//     {
//       { 3, 1000, 200, 5, 5 },
//       { 4, 500, 100, 10, 10 }
//     }
//   };
//======================================================================================
inline constexpr RelayEngine relays{
  {
    { 3, 1000, 200, 1, 1 },
    { 4, 1000, 200, 1, 1 }
  }
};

//======================================================================================
// INPUTS / SINGLE PINS
// Set to 0xff when not used
//======================================================================================
inline constexpr uint8_t dualTariffPin{ 0xff };
inline constexpr uint8_t rotationPin{ 0xff };
inline constexpr uint8_t forcePin{ 0xff };       /**< Legacy single boost input kept unused */
inline constexpr uint8_t watchDogPin{ 5 };

//======================================================================================
// DUAL TARIFF
//======================================================================================
inline constexpr uint8_t ul_OFF_PEAK_DURATION{ 8 };
inline constexpr pairForceLoad rg_ForceLoad[NO_OF_DUMPLOADS]{ { -3, 2 } };

//======================================================================================
// TEMPERATURE SENSORS
//======================================================================================
inline constexpr int16_t iTemperatureThreshold{ 100 };
inline constexpr TemperatureSensing temperatureSensing{
  0xff,
  { { 0x28, 0x1B, 0xD7, 0x6A, 0x09, 0x00, 0x00, 0xB7 } }
};

//======================================================================================
// LOAD PRIORITY ROTATION
//======================================================================================
inline constexpr uint16_t ROTATION_AFTER_SECONDS{ 8u * 3600u };

//======================================================================================
// OLED USER INTERFACE
//======================================================================================
// Rotary encoder wiring:
//   CLK -> D11
//   DT  -> D12
//   SW  -> D13
//
// OLED_ENABLE_RUNTIME_SETTINGS:
//   true  => relay/system thresholds editable from OLED + EEPROM backup
//   false => display only
//
// OLED_ENABLE_RESTART_PAGE:
//   true  => adds a dedicated OLED page to reboot the router
//   false => page hidden
//======================================================================================
inline constexpr OledEncoderConfig oledEncoder{ 11, 12, 13 };
inline constexpr bool OLED_ENABLE_RUNTIME_SETTINGS{ true };
inline constexpr bool OLED_ENABLE_RESTART_PAGE{ true };

//======================================================================================
// OUTPUT INDEXING HELPERS — LOAD(), RELAY(), ALL_LOADS_AND_RELAYS()
// -------------------------------------------------------------------------------------
// TRIAC outputs : 0 .. NO_OF_DUMPLOADS - 1
// Relay outputs : NO_OF_DUMPLOADS .. NO_OF_DUMPLOADS + relay_count - 1
//
// Example with 1 TRIAC + 2 relays:
//   LOAD(0)  => output index 0
//   RELAY(0)  => output index 1
//   RELAY(1)  => output index 2
//======================================================================================
#include "config_helpers.h"

//======================================================================================
// BOOST COMMANDS (manual force ON per output)
// -------------------------------------------------------------------------------------
// Each entry = { inputPin, outputIndex, visibleOnOLED }
//
//   inputPin      : physical pin (INPUT_PULLUP) or unused_pin for OLED-only
//   outputIndex   : target output — use LOAD(n) or RELAY(n)
//   visibleOnOLED : true => dedicated BOOST page on OLED
//                   false => physical input only
//
// Example:
//   inline constexpr BoostControlConfig boostControls{
//     { { 7, LOAD(0), true },
//       { unused_pin, RELAY(1), true },
//       { 8, LOAD(0), false } }
//   };
//======================================================================================
inline constexpr BoostControlConfig boostControls{
  { { unused_pin, LOAD(0), true },
    { unused_pin, RELAY(0), true },
    { unused_pin, RELAY(1), true } }
};

//======================================================================================
// DIVERSION AUTHORIZATION GROUPS
// -------------------------------------------------------------------------------------
// Each entry = { inputPin, outputMask, visibleOnOLED }
//
// Semantic: ON = routing authorized, OFF = routing blocked.
// All groups start ON at boot.
//
//   inputPin      : physical pin (INPUT_PULLUP, LOW = block) or unused_pin for OLED-only
//   outputMask    : affected outputs — use LOAD(n), RELAY(n), ALL_LOADS(),
//                   ALL_RELAYS(), ALL_LOADS_AND_RELAYS(), or combine with |
//   visibleOnOLED : true => toggle from OLED routing page
//                   false => physical input only
//
// Example:
//   inline constexpr DiversionGroupConfig diversionGroups{
//     { { 10, ALL_LOADS_AND_RELAYS(), true },
//       { unused_pin, LOAD(0), true },
//       { unused_pin, RELAY(0) | RELAY(1), true },
//       { 9, ALL_LOADS(), false } }
//   };
//======================================================================================
inline constexpr DiversionGroupConfig diversionGroups{
  { { 10, ALL_LOADS_AND_RELAYS(), true },
    { unused_pin, LOAD(0), true },
    { unused_pin, RELAY(0), true },
    { unused_pin, RELAY(1), true } }
};

#endif /* CONFIG_H */
