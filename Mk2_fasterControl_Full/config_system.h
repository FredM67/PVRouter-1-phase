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

#ifndef __CONFIG_SYSTEM_H__
#define __CONFIG_SYSTEM_H__

#include <Arduino.h>

#include "type_traits.hpp"

inline constexpr uint8_t ADC_TIMER_PERIOD{ 125 };  // uS (determines the sampling rate / amount of idle time)

// Physical constants, please do not change!
inline constexpr uint16_t SECONDS_PER_MINUTE{ 60 };
inline constexpr uint16_t MINUTES_PER_HOUR{ 60 };
inline constexpr uint16_t JOULES_PER_WATT_HOUR{ 3600 };  //  (0.001 kWh = 3600 Joules)

// Change these values to suit the local mains frequency and supply meter
inline constexpr uint8_t CYCLES_PER_SECOND{ 50 };
inline constexpr uint16_t WORKING_RANGE_IN_JOULES{ 360 };  // 0.1 Wh, reduced for faster start-up
inline constexpr int16_t REQUIRED_EXPORT_IN_WATTS{ 0 };    // when set to a negative value, this acts as a PV generator

// to prevent the diverted energy total from 'creeping'
inline constexpr uint8_t ANTI_CREEP_LIMIT{ 5 };  // in Joules per mains cycle (has no effect when set to 0)

#endif  // __CONFIG_SYSTEM_H__
