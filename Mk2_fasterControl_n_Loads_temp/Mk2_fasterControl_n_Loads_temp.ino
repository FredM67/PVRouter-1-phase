/**
 * @file Mk2_fasterControl_twoLoads_temp_1.ino
 * @author Robin Emley (www.Mk2PVrouter.co.uk)
 * @author Frederic Metrich (frederic.metrich@live.fr)
 * @brief Mk2_fasterControl_twoLoads_temp_1.ino - A photovoltaïc energy diverter.
 * @date 2020-04-08
 *
 * @mainpage A 3-phase photovoltaïc router/diverter
 *
 * @section description Description
 * Mk2_fasterControl_twoLoads_temp_1.ino - Arduino program that maximizes the use of home photovoltaïc production
 * by monitoring energy consumption and diverting power to one or more resistive charge(s) when needed.
 * In the absence of such a system, surplus energy flows away to the grid and is of no benefit to the PV-owner.
 *
 * @section history History
 * __Initially released as Mk2_bothDisplays_1 in March 2014.__
 *
 * This sketch is for diverting suplus PV power to a dump load using a triac or
 * Solid State Relay. It is based on the Mk2i PV Router code that I have posted in on
 * the OpenEnergyMonitor forum. The original version, and other related material,
 * can be found on my Summary Page at www.openenergymonitor.org/emon/node/1757
 *
 * In this latest version, the pin-allocations have been changed to suit my
 * PCB-based hardware for the Mk2 PV Router. The integral voltage sensor is
 * fed from one of the secondary coils of the transformer. Current is measured
 * via Current Transformers at the CT1 and CT1 ports.
 *
 * CT1 is for 'grid' current, to be measured at the grid supply point.
 * CT2 is for the load current, so that diverted energy can be recorded
 *
 * A persistence-based 4-digit display is supported. This can be driven in two
 * different ways, one with an extra pair of logic chips, and one without. The
 * appropriate version of the sketch must be selected by including or commenting
 * out the "#define PIN_SAVING_HARDWARE" statement near the top of the code.
 *
 * __September 2014: renamed as Mk2_bothDisplays_2, with these changes:__
 * - cycleCount removed (was not actually used in this sketch, but could have overflowed);
 * - removal of unhelpful comments in the IO pin section;
 * - tidier initialisation of display logic in setup();
 * - addition of REQUIRED_EXPORT_IN_WATTS logic (useful as a built-in PV simulation facility);
 *
 * __December 2014: renamed as Mk2_bothDisplays_3, with these changes:__
 * - persistence check added for zero-crossing detection (polarityConfirmed)
 * - lowestNoOfSampleSetsPerMainsCycle added, to check for any disturbances
 *
 * __December 2014: renamed as Mk2_bothDisplays_3a, with these changes:__
 * - some typographical errors fixed.
 *
 * __January 2016: renamed as Mk2_bothDisplays_3b, with these changes:__
 * - a minor change in the ISR to remove a timing uncertainty.
 *
 * __January 2016: updated to Mk2_bothDisplays_3c:__
 * -  The variables to store the ADC results are now declared as "volatile" to remove
 *      any possibility of incorrect operation due to optimisation by the compiler.
 *
 * __February 2016: updated to Mk2_bothDisplays_4, with these changes:__
 * - improvements to the start-up logic. The start of normal operation is now
 *     synchronised with the start of a new mains cycle.
 * - reduce the amount of feedback in the Low Pass Filter for removing the DC content
 *     from the Vsample stream. This resolves an anomaly which has been present since
 *     the start of this project. Although the amount of feedback has previously been
 *     excessive, this anomaly has had minimal effect on the system's overall behaviour.
 * - removal of the unhelpful "triggerNeedsToBeArmed" mechanism
 * - tidying of the "confirmPolarity" logic to make its behaviour more clear
 * - SWEETZONE_IN_JOULES changed to WORKING_RANGE_IN_JOULES
 * - change "triac" to "load" wherever appropriate
 *
 * __November 2019: updated to Mk2_fasterControl_1 with these changes:__
 * - Half way through each mains cycle, a prediction is made of the likely energy level at the
 *     end of the cycle. That predicted value allows the triac to be switched at the +ve going
 *     zero-crossing point rather than waiting for a further 10 ms. These changes allow for
 *     faster switching of the load.
 * - The range of the energy bucket has been reduced to one tenth of its former value. This
 *     allows the unit's operation to commence more rapidly whenever surplus power is available.
 * - controlMode is no longer selectable, the unit's operation being effectively hard-coded
 *     as "Normal" rather than Anti-flicker.
 * - Port D3 now supports an indicator which shows when the level in the energy bucket
 *     reaches either end of its range. While the unit is actively diverting surplus power,
 *     it is vital that the level in the reduced capacity energy bucket remains within its
 *     permitted range, hence the addition of this indicator.
 *
 * __February 2020: updated to Mk2_fasterControl_twoLoads_1 with these changes:__
 * - the energy overflow indicator has been disabled to free up port D3
 * - port D3 now supports a second load
 *
 * __February 2020: updated to Mk2_fasterControl_twoLoads_2 with these changes:__
 * - improved multi-load control logic to prevent the primary load from being disturbed by
 *     the lower priority one. This logic now mirrors that in the Mk2_multiLoad_wired_n line.
 *
 * __March 2021: updated to Mk2_fasterControl_twoLoads_3 with these changes:__
 * - extra filtering added to offset the HPF effect of CT1.  This allows the energy state in
 *     10 ms time to be predicted with more confidence.  Specifically, it is no longer necessary
 *     to include a 30% boost factor after each change of load state.
 *
 * __June 2021: updated to Mk2_fasterControl_3 with these changes:__
 * - to reflect the performance of recently manufactured YHDC SCT_013_000 CTs,
 *     the value of the parameter lpf_gain has been reduced from 12 to 8.
 *
 *      Robin Emley
 *      www.Mk2PVrouter.co.uk
 *
 * __April 2020: renamed as Mk2_fasterControl_twoLoads__temp_1 with these changes:__
 * - This sketch has been restructured in order to make better use of the ISR.
 * - This sketch has been again re-engineered. All 'defines' have been removed except
 *     the ones for compile-time optional functionalities.
 * - All constants have been replaced with constexpr initialized at compile-time
 * - all number-types have been replaced with fixed width number types
 * - old fashion enums replaced by scoped enums with fixed types
 * - All of the time-critical code is now contained within the ISR and its helper functions.
 * - Values for datalogging are transferred to the main code using a flag-based handshake mechanism.
 * - The diversion of surplus power can no longer be affected by slower
 *     activities which may be running in the main code such as Serial statements and RF.
 * - Temperature sensing is supported. A pullup resistor (4K7 or similar) is required for the Dallas sensor.
 * - Display shows diverted energy and temperature in a cycling way (5 seconds cycle)
 *
 * __October 2020: updated Mk2_fasterControl_twoLoads_2 with these changes:__
 * - fix a couple of small compiler errors depending on the defines set/unset (thx Gaetan)
 *
 *   Fred Metrich
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <Arduino.h>

static_assert(__cplusplus >= 201703L, "**** Please define 'gnu++17' in 'platform.txt' ! ****");
static_assert(__cplusplus >= 201703L, "See also : https://github.com/FredM67/PVRouter-Single/blob/main/Mk2_fasterControl_n_Loads_temp/Readme.md");

// The active code can be found in the other cpp/h files
