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

#ifndef _CONFIG_H
#define _CONFIG_H

#include "config_system.h"
#include "types.h"

inline constexpr uint8_t NO_OF_DUMPLOADS{ 2 };  // The logic expects a minimum of 2 dumploads,
                                                // for local & remote loads, but neither has to
                                                // be physically present.

//  The two versions of the hardware require different logic.  The following line should
//  be included if the additional logic chips are present, or excluded if they are
//  absent (in which case some wire links need to be fitted)
//
#define PIN_SAVING_HARDWARE

// allocation of digital pins which are not dependent on the display type that is in use
// *************************************************************************************
inline constexpr uint8_t physicalLoadPin[NO_OF_DUMPLOADS]{ 4, 3 }; /**< for 1-phase PCB */

#endif  // _CONFIG_H
