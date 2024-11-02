/**
 * @file config.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Configuration values to be set by the end-user
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CONFIG_H
#define CONFIG_H

//--------------------------------------------------------------------------------------------------
//#define TEMP_ENABLED  /**< this line must be commented out if the temperature sensor is not present */
//#define RF_PRESENT  /**< this line must be commented out if the RFM12B module is not present */

// Output messages
//#define EMONESP  /**< Uncomment if an ESP WiFi module is used

#define ENABLE_DEBUG /**< enable this line to include debugging print statements */
#define SERIALPRINT  /**< include 'human-friendly' print statement for commissioning - comment this line to exclude. */
//#define SERIALOUT /**< Uncomment if a wired serial connection is used */
//--------------------------------------------------------------------------------------------------

#include "config_system.h"
#include "debug.h"
#include "types.h"

#include "utils_dualtariff.h"
#include "utils_relay.h"
#include "utils_temp.h"

inline constexpr uint8_t NO_OF_DUMPLOADS{ 2 }; /**< number of dump loads connected to the diverter */

#ifdef EMONESP
inline constexpr bool EMONESP_CONTROL{ true };
inline constexpr bool DIVERSION_PIN_PRESENT{ true };                    /**< managed through EmonESP */
inline constexpr RotationModes PRIORITY_ROTATION{ RotationModes::PIN }; /**< managed through EmonESP */
inline constexpr bool OVERRIDE_PIN_PRESENT{ true };                     /**< managed through EmonESP */
#else
inline constexpr bool EMONESP_CONTROL{ false };
inline constexpr bool DIVERSION_PIN_PRESENT{ false };                   /**< set it to 'true' if you want to control diversion ON/OFF */
inline constexpr RotationModes PRIORITY_ROTATION{ RotationModes::OFF }; /**< set it to 'OFF/AUTO/PIN' if you want manual/automatic rotation of priorities */
inline constexpr bool OVERRIDE_PIN_PRESENT{ false };                    /**< set it to 'true' if there's a override pin */
#endif

inline constexpr bool WATCHDOG_PIN_PRESENT{ false }; /**< set it to 'true' if there's a watch led */
inline constexpr bool RELAY_DIVERSION{ false };      /**< set it to 'true' if a relay is used for diversion */
inline constexpr bool DUAL_TARIFF{ false };          /**< set it to 'true' if there's a dual tariff each day AND the router is connected to the billing meter */

//  The two versions of the hardware require different logic.  The following line should
//  be included if the additional logic chips are present, or excluded if they are
//  absent (in which case some wire links need to be fitted)
//
#define PIN_SAVING_HARDWARE

// allocation of digital pins which are not dependent on the display type that is in use
// *************************************************************************************
inline constexpr uint8_t physicalLoadPin[NO_OF_DUMPLOADS]{ 4, 3 };         /**< for 1-phase PCB */
inline constexpr uint8_t loadPrioritiesAtStartup[NO_OF_DUMPLOADS]{ 0, 1 }; /**< load priorities and states at startup */

// Set the value to 0xff when the pin is not needed (feature deactivated)
inline constexpr uint8_t dualTariffPin{ 0xff }; /**< for 3-phase PCB, off-peak trigger */
inline constexpr uint8_t diversionPin{ 0xff };  /**< if LOW, set diversion on standby */
inline constexpr uint8_t rotationPin{ 0xff };   /**< if LOW, trigger a load priority rotation */
inline constexpr uint8_t forcePin{ 0xff };      /**< for 3-phase PCB, force pin */
inline constexpr uint8_t watchDogPin{ 0xff };   /**< watch dog LED */

inline constexpr RelayEngine relays{ { { 0xff, 1000, 200, 1, 1 } } }; /**< config for relay diversion, see class definition for defaults and advanced options */

inline constexpr uint8_t ul_OFF_PEAK_DURATION{ 8 };                        /**< Duration of the off-peak period in hours */
inline constexpr pairForceLoad rg_ForceLoad[NO_OF_DUMPLOADS]{ { -3, 2 } }; /**< force config for load #1 ONLY for dual tariff */

inline constexpr int16_t iTemperatureThreshold{ 100 }; /**< the temperature threshold to stop overriding in °C */

inline constexpr TemperatureSensing temperatureSensing{ 0xff,
                                                        { { 0x28, 0x1B, 0xD7, 0x6A, 0x09, 0x00, 0x00, 0xB7 } } }; /**< list of temperature sensor Addresses */

inline constexpr uint32_t ROTATION_AFTER_CYCLES{ 8UL * 3600UL * SUPPLY_FREQUENCY }; /**< rotates load priorities after this period of inactivity */

#endif  // CONFIG_H
