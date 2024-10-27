/**
 * @file processing.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Public functions/variables of processing engine
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _PROCESSING_H
#define _PROCESSING_H

#include "config.h"

// allocation of analogue pins which are not dependent on the display type that is in use
// **************************************************************************************
inline constexpr uint8_t voltageSensor{ 3 };           // A3 is for the voltage sensor
inline constexpr uint8_t currentSensor_diverted{ 4 };  // A4 is for CT2 which measures diverted current
inline constexpr uint8_t currentSensor_grid{ 5 };      // A5 is for CT1 which measures grid current

inline constexpr uint16_t delayBeforeSerialStarts{ 1000 };  // in milli-seconds, to allow Serial window to be opened
inline constexpr uint16_t startUpPeriod{ 3000 };            // in milli-seconds, to allow LP filter to settle

#endif  // _PROCESSING_H
