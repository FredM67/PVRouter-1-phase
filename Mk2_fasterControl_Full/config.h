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
 * - The bottom part of this file contains helper structures derived from your choices.
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

// Backward compatibility flags kept enabled automatically when related tables are used.
inline constexpr bool DIVERSION_PIN_PRESENT{ true };
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
// Format per relay: { pin, SM, SA, Ton, Toff }
//   SM   = Seuil Marche   (surplus threshold, W)
//   SA   = Seuil Arrêt    (import threshold, W)
//   Ton  = minimum ON time in minutes
//   Toff = minimum OFF time in minutes
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
inline constexpr uint8_t diversionPin{ 0xff };   /**< Legacy single diversion input kept unused */
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
// ADVANCED OUTPUT INDEXING (used by BOOST and DIVERSION tables below)
// -------------------------------------------------------------------------------------
// TRIAC outputs : 0 .. NO_OF_DUMPLOADS - 1
// Relay outputs : NO_OF_DUMPLOADS .. NO_OF_DUMPLOADS + relay_count - 1
//
// Example with 1 TRIAC + 2 relays:
//   TRIAC(0)  => output index 0
//   RELAY(0)  => output index 1
//   RELAY(1)  => output index 2
//======================================================================================
inline constexpr uint8_t TOTAL_ROUTER_OUTPUTS{
  static_cast< uint8_t >(NO_OF_DUMPLOADS + (RELAY_DIVERSION ? relays.get_size() : 0))
};

constexpr OutputId TRIAC(const uint8_t idx)
{
  return { idx };
}

constexpr OutputId RELAY(const uint8_t idx)
{
  return { static_cast< uint8_t >(NO_OF_DUMPLOADS + idx) };
}

constexpr uint16_t ALL_OUTPUTS()
{
  return TOTAL_ROUTER_OUTPUTS >= 16u ? 0xFFFFu : static_cast< uint16_t >((1u << TOTAL_ROUTER_OUTPUTS) - 1u);
}

//======================================================================================
// BOOST COMMANDS (manual force ON per output)
// -------------------------------------------------------------------------------------
// One entry = one independent boost source.
//
// inputPin:
//   - physical input pin using INPUT_PULLUP logic
//   - unused_pin if no physical input is used
//
// outputIndex:
//   - target output (TRIAC or relay)
//
// visibleOnOLED:
//   - true  => create a dedicated BOOST page on OLED
//   - false => command exists only from physical input
//
// Current configuration requested:
//   BOOST 1 => D7 => TRIAC 1
//   BOOST 2 => D8 => RELAY 1
//   BOOST 3 => D9 => RELAY 2
//======================================================================================
inline constexpr uint8_t BOOST_CONTROL_COUNT{ 3 };
inline constexpr BoostControlConfig boostControls[BOOST_CONTROL_COUNT]{
  { unused_pin, TRIAC(0), true },
  { unused_pin, RELAY(0), true },
  { unused_pin, RELAY(1), true }
};

//======================================================================================
// DIVERSION AUTHORIZATION GROUPS
// -------------------------------------------------------------------------------------
// IMPORTANT SEMANTIC:
//   ON  = routing authorized
//   OFF = routing blocked for the configured outputs
// Startup is ON by default.
//
// inputPin:
//   - physical input pin using INPUT_PULLUP logic
//   - pulling the pin LOW blocks the group
//   - unused_pin if the group is OLED-only
//
// outputMask:
//   - affected outputs
//   - use ALL_OUTPUTS() for all active outputs
//
// visibleOnOLED:
//   - true  => appears on OLED routing page
//   - false => physical-only group
//
// Current configuration requested:
//   Diversion 1 => D10        => ALL outputs => OLED visible
//   Diversion 2 => OLED only  => output 1    => OLED visible
//   Diversion 3 => OLED only  => outputs 2,3 => OLED visible
//======================================================================================
inline constexpr uint8_t DIVERSION_GROUP_COUNT{ 4 };
inline constexpr DiversionGroupConfig diversionGroups[DIVERSION_GROUP_COUNT]{
  { 10, ALL_OUTPUTS(), true },
  { unused_pin, TRIAC(0), true },
  { unused_pin, RELAY(0), true },
  { unused_pin, RELAY(1), true }
};

#endif /* CONFIG_H */
