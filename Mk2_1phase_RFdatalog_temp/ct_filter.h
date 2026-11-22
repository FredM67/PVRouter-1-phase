/**
 * @file ct_filter.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief The CT1 filter: a low-pass of the grid current added back with a gain
 * @version 1.0
 * @date 2026-11-22
 *
 * @copyright Copyright (c) 2026-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * @details A current transformer acts as a high-pass filter. Robin's sketches offset it with
 *          an IIR low-pass filter of the current, added back with a gain (lpf_gain, alpha in
 *          calibration.h). This is his filter, Mk2_fasterControl_Full/processing.cpp, at his
 *          resolution: an int32 working value at x256 scale (one ADC count = 256), so that
 *          the state moves as soon as alpha x the difference reaches one unit (3.6 ADC counts
 *          with alpha 0.0011, against 14 counts at the x64 scale of the firmware).
 *
 *          The firmware's sample is DC-removed and left-aligned (x64 scale, int16): it is
 *          scaled up to x256, filtered, scaled back to x64 and saturated to the int16 range,
 *          so that a peak beyond full scale cannot wrap and flip the sign of the power.
 *
 *          Pure C++, no Arduino: the native tests include it.
 */

#ifndef CT_FILTER_H
#define CT_FILTER_H

#include <stdint.h>

namespace CtFilter
{
/** @brief The state of the filter of one CT, and the filter itself. */
struct LowPassBoost
{
  int32_t lpf{}; /**< low-pass of the current, x256 scale (Robin's lpf_long) */

  /**
   * @brief Filters one sample.
   *
   * @param sampleMinusDC The DC-removed current sample, x64 scale (left-aligned ADC).
   * @param gain The gain of the low-pass added back (lpf_gain); 0 returns the sample.
   * @param alpha The coefficient of the low-pass.
   * @return The filtered sample, x64 scale, saturated to the int16 range.
   */
  int16_t apply(const int16_t sampleMinusDC, const float gain, const float alpha)
  {
    int32_t sample{ static_cast< int32_t >(sampleMinusDC) * 4 };  // x256, as Robin's

    // as Robin's: computed in float, truncated back to int32
    lpf = static_cast< int32_t >(static_cast< float >(lpf) + alpha * static_cast< float >(sample - lpf));
    sample = static_cast< int32_t >(static_cast< float >(sample) + gain * static_cast< float >(lpf));

    sample >>= 2;  // back to x64

    if (sample > INT16_MAX)
    {
      return INT16_MAX;
    }
    if (sample < INT16_MIN)
    {
      return INT16_MIN;
    }
    return static_cast< int16_t >(sample);
  }
};
}  // namespace CtFilter

#endif  // CT_FILTER_H
