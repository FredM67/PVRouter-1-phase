/**
 * @file config_derived.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief The configuration as the firmware uses it: the settings, and what is built from them
 * @version 1.0
 * @date 2026-11-21
 *
 * @copyright Copyright (c) 2026-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * @details The code includes this file, never config.h directly. config.h only holds the
 *          settings; here come the values derived from them and the objects built from them:
 *          - the calibration of the board (calibration.h): f_powerCal, f_voltageCal and the CT1
 *            filter of PCB_VERSION, in the per-phase arrays the code uses (one phase);
 *          - the number of remote loads and units, from the load map;
 *          - remoteLoads, the remote-load controller, which holds the transmission state;
 *          - relayDiverter, the relay engine, from relays;
 *          - overrideTable and boostSchedule, the bitmasks that overridePins and scheduledBoosts
 *            force, once LOAD(), RELAY() and ALL_...() are turned into pins;
 *          - tempSensors, which reads the sensors of temperatureSensing.
 */

#ifndef CONFIG_DERIVED_H
#define CONFIG_DERIVED_H

#include "config_system.h"
#include "config.h"

// A config.h of the former layout defines CONFIG_H (debug.h stops on it too, with the reason)
#if defined(CONFIG_H)
#error "This config.h has the former layout, with #include lines and derived values. config.h now only holds the settings: create it again with the configurator, or see 'config.h holds the settings only' in the CHANGELOG to update it by hand."
#endif

#include "types.h"
#include "debug.h"

//--------------------------------------------------------------------------------------------------
// From calibration.h: the values of the board, as the firmware uses them
//
#include "calibration.h"

static_assert(NO_OF_PHASES == 1, "f_powerCal and f_voltageCal below list 1 phase");

/** Whether the router runs with the set of the new board */
inline constexpr bool NEW_BOARD_CALIBRATION{ PCB_VERSION == PcbVersion::NEW };

/** f_powerCal of CT1, in the per-phase array the router uses */
inline constexpr float f_powerCal[NO_OF_PHASES]{ NEW_BOARD_CALIBRATION ? f_powerCalPerAmp * f_ctRating : f_powerCalOldBoard };

/** f_voltageCal, in the per-phase array the router uses */
inline constexpr float f_voltageCal[NO_OF_PHASES]{ NEW_BOARD_CALIBRATION ? f_voltageCalNewBoard : f_voltageCalOldBoard };

/** f_powerCal of CT2, for the diverted energy */
inline constexpr float f_powerCalDiverted{ NEW_BOARD_CALIBRATION ? f_powerCalPerAmp * f_ctRatingDiverted : f_powerCalDivertedOldBoard };

/** the CT1 filter of the board */
inline constexpr float lpf_gain{ NEW_BOARD_CALIBRATION ? lpf_gainNewBoard : lpf_gainOldBoard };
inline constexpr float alpha{ NEW_BOARD_CALIBRATION ? alphaNewBoard : alphaOldBoard };

//--------------------------------------------------------------------------------------------------
// From the load map
//
inline constexpr uint8_t NO_OF_REMOTE_LOADS{ Load::countRemoteLoads(physicalLoadPin) }; /**< number of loads controlled via RF */
inline constexpr uint8_t NO_OF_REMOTE_UNITS{ Load::countUnits(physicalLoadPin) };       /**< number of remote units to talk to */
inline constexpr bool REMOTE_LOADS_PRESENT{ NO_OF_REMOTE_LOADS != 0 };                  /**< true if remote loads are configured */

#include "remote_loads_core.h"

/**
 * @brief The one and only remote-load controller.
 *
 * @details Declared non-const on purpose: it holds the per-unit transmission state.
 *          @c RemoteLoadCore has a @c constexpr default constructor and brace-or-equal
 *          member initialisers, so this object is constant-initialized. It therefore
 *          lands in @c .bss with no entry in @c .init_array, and @c --gc-sections drops
 *          it outright when no remote unit is configured.
 */
inline RemoteLoadCore< NO_OF_REMOTE_UNITS > remoteLoads{};

//--------------------------------------------------------------------------------------------------
// The objects built from the settings
//
#include "rotation_groups.h"
#include "utils_dualtariff.h"
#include "utils_override.h"
#include "utils_relay.h"
#include "utils_temp.h"

/** @brief What LOAD(), RELAY() and ALL_...() stand for, in overridePins and scheduledBoosts. */
inline constexpr TargetMap targetMap{ physicalLoadPin, relays, RELAY_DIVERSION };

inline constexpr RelayDiverter relayDiverter{ relays };                     /**< the relay engine */
inline constexpr OverrideTable overrideTable{ overridePins, targetMap };    /**< what each override pin forces */
inline constexpr BoostSchedule boostSchedule{ scheduledBoosts, targetMap }; /**< what each scheduled boost forces */
inline constexpr TempSensors tempSensors{ temperatureSensing };             /**< the temperature sensors */
/** @brief The priority places each rotation group rotates in. */
inline constexpr Rotation::Table rotationTable{ rotationGroups, physicalLoadPin, loadPrioritiesAtStartup };

#include "remote_loads.h"  // the RF shell; needs remoteLoads and the RF feature flags

#endif /* CONFIG_DERIVED_H */
