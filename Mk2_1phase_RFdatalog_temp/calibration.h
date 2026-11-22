/**
 * @file calibration.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Calibration values definition
 * @version 0.2
 * @date 2026-11-22
 *
 * @copyright Copyright (c) 2023-2026
 *
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "config.h"

//
// Calibration values
//-------------------
// Each board has its own set below: the router uses the one of PCB_VERSION (config.h).
// f_powerCal converts the products of voltage and current samples into watts: CT1 (grid) for the
// power the router diverts, CT2 (diverted) for the diverted energy it reports. f_voltageCal
// converts the voltage samples into volts, for the voltage shown in the datalog only. To
// calibrate, compare with a reference meter:
//   new value = old value x reference reading / router reading
//
// ---- Old boards (PcbVersion::OLD_SEG and PcbVersion::OLD_OLED) ----
//
// When calculating real power, which is what this code does, the individual
// conversion rates for voltage and current are not of importance. It is
// only the conversion rate for POWER which is important. This is the
// product of the individual conversion rates for voltage and current. It
// therefore has the units of ADC-steps squared per Watt. Most systems will
// have a power conversion rate of around 20 (ADC-steps squared per Watt).
//
// powerCal is the RECIPROCAL of the power conversion rate. A good value
// to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)
//
// Values measured on a Mk2 board (as in Mk2_fasterControl_Full): a starting point for yours.
inline constexpr float f_powerCalOldBoard{ 0.0436F };          /**< CT1, grid */
inline constexpr float f_powerCalDivertedOldBoard{ 0.04446F }; /**< CT2, diverted */
//
// For datalogging purposes, f_voltageCal has been added too. Because the range of ADC values is
// similar to the actual range of volts, the optimal value for this cal factor is likely to be
// close to unity.
inline constexpr float f_voltageCalOldBoard{ 0.805653F }; /**< compared with Sentron PAC 4200 */
//
// CT1 filter: an IIR low-pass filter of the grid current, added back with a gain, offsets the CT
// acting as a high-pass filter. Interim values for the SCT-013-000 (Dec. 2024, on real hardware).
// Turned into fixed point at compile time (ct_filter.h): alpha from 0.00005 to 0.0625, gain below 64.
inline constexpr float lpf_gainOldBoard{ 9 }; /**< setting this to 0 disables this extra processing */
inline constexpr float alphaOldBoard{ 0.0011 };
//
// ---- New board (PcbVersion::NEW, rev. 6.0+) ----
//
// f_powerCal is given per amp of CT rating: f_powerCal = f_powerCalPerAmp x f_ctRating, so
// changing a CT only means entering its rating. The default values suit 333 mV voltage-output CTs;
// they come from the design of the board, corrected by the calibration of a first board (see
// docs/calibration.md). With a current-output CT and a burden resistor R (YHDC SCT-013-000,
// 2000:1), f_ctRating = 666 / R: the current giving 333 mV across the burden.
inline constexpr float f_ctRating{ 50.0F };                  /**< rated current of CT1 (grid), in A */
inline constexpr float f_ctRatingDiverted{ 50.0F };          /**< rated current of CT2 (diverted), in A */
inline constexpr float f_powerCalPerAmp{ 0.00286405F };      /**< f_powerCal per A of CT rating */
inline constexpr float f_voltageCalNewBoard{ 0.888065256F }; /**< volts per step */
//
// CT1 filter: off, the voltage-output CTs behave differently.
inline constexpr float lpf_gainNewBoard{ 0 }; /**< setting this to 0 disables this extra processing */
inline constexpr float alphaNewBoard{ 0.002 };
//
// ---- All boards ----
//
// f_phaseCal is used to alter the phase of the voltage waveform relative to the current waveform.
// The algorithm interpolates between the most recent pair of voltage samples according to the value of f_phaseCal.
//
//    With f_phaseCal = 1, the most recent sample is used.
//    With f_phaseCal = 0, the previous sample is used
//    With f_phaseCal = 0.5, the mid-point (average) value in used
//
// NB. Any tool which determines the optimal value of f_phaseCal must have a similar
// scheme for taking sample values as does this sketch.
//
inline constexpr float f_phaseCal{ 1 }; /**< Nominal values only */

#endif  // CALIBRATION_H
