/**
 * @file config_system.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Basic configuration values to be set by the end-user
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CONFIG_SYSTEM_H
#define CONFIG_SYSTEM_H

#include <Arduino.h>

#include "type_traits.hpp"

inline constexpr uint8_t NO_OF_PHASES{ 1 }; /**< number of phases of the main supply. */

// Physical constants, please do not change!
inline constexpr uint16_t SECONDS_PER_MINUTE{ 60 };
inline constexpr uint16_t MINUTES_PER_HOUR{ 60 };
inline constexpr uint16_t JOULES_PER_WATT_HOUR{ 3600 };  /**< (0.001 kWh = 3600 Joules) */

// Change these values to suit the local mains frequency and supply meter
inline constexpr uint8_t SUPPLY_FREQUENCY{ 50 };
inline constexpr uint16_t WORKING_ZONE_IN_JOULES{ 360 };  /**< 0.1 Wh, reduced for faster start-up */
inline constexpr int16_t REQUIRED_EXPORT_IN_WATTS{ 0 };   /**< when set to a negative value, this acts as a PV generator */

// to prevent the diverted energy total from 'creeping'
inline constexpr uint8_t ANTI_CREEP_LIMIT{ 5 };  /**< in Joules per mains cycle (has no effect when set to 0) */

inline constexpr uint32_t mainsCyclesPerHour{ SUPPLY_FREQUENCY * SECONDS_PER_MINUTE * MINUTES_PER_HOUR };

inline constexpr uint8_t DATALOG_PERIOD_IN_SECONDS{ 5 }; /**< Period of datalogging in seconds */

inline constexpr typename conditional< DATALOG_PERIOD_IN_SECONDS * SUPPLY_FREQUENCY >= UINT8_MAX, uint16_t, uint8_t >::type
  DATALOG_PERIOD_IN_MAINS_CYCLES{ DATALOG_PERIOD_IN_SECONDS * SUPPLY_FREQUENCY }; /**< Period of datalogging in cycles */

// Computes inverse value at compile time to use '*' instead of '/'
inline constexpr float invSUPPLY_FREQUENCY{ 1.0F / SUPPLY_FREQUENCY };
inline constexpr float invDATALOG_PERIOD_IN_MAINS_CYCLES{ 1.0F / DATALOG_PERIOD_IN_MAINS_CYCLES };

inline constexpr uint16_t delayBeforeSerialStarts{ 1000 };  /**< in milli-seconds, to allow Serial window to be opened */
inline constexpr uint16_t startUpPeriod{ 3000 };            /**< in milli-seconds, to allow LP filter to settle */

//--------------------------------------------------------------------------------------------------
#ifdef EMONESP
#undef SERIALPRINT  /**< Must not corrupt serial output to emonHub with 'human-friendly' printout */
#undef SERIALOUT
#undef DEBUGGING
#include <ArduinoJson.h>
#endif

#ifdef SERIALOUT
#undef EMONESP
#undef SERIALPRINT  /**< Must not corrupt serial output to emonHub with 'human-friendly' printout */
#undef DEBUGGING
#endif
//--------------------------------------------------------------------------------------------------

#endif /* CONFIG_SYSTEM_H */
