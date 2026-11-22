/**
 * @file fast_control.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Faster control: the load decision on the predicted energy, at mid-cycle
 * @version 0.1
 * @date 2026-11-22
 *
 * @copyright Copyright (c) 2026-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * @details Robin Emley's fasterControl. At the first sample set of the negative half cycle,
 *          the average power of the first half is known: the energy at the end of the cycle is
 *          predicted from it. A few sample sets later (DECISION_SAMPLE), the loads are decided on
 *          that prediction. The zero-crossing triac drivers latch the new states at the next
 *          positive zero crossing: the loads change at the start of the next cycle, half a cycle
 *          sooner than with a decision taken after that crossing, on the measured bucket.
 *
 *          The ISR predicts through the per-sample calibration table, without a division
 *          (wholeCycleIndexOf()); the division form is kept as the reference of the tests.
 *
 * @note This header has no Arduino dependency, so it can be exercised by the native
 *       test suite.
 *
 * @ingroup TimeCritical
 */

#ifndef FAST_CONTROL_H
#define FAST_CONTROL_H

#include <stdint.h>

#include "energy_bucket.h"

namespace FastControl
{
/**
 * @brief The sample set of the negative half cycle at which the loads are decided.
 *
 * @details Counted from 0 at the first sample set of the negative half, as Robin's
 *          sampleSetsDuringNegativeHalfOfMainsCycle == POST_ZERO_CROSSING_MAX_COUNT (3): the
 *          voltage is then far enough past the zero crossing for the triac drivers to be armed
 *          reliably.
 */
inline constexpr uint8_t DECISION_SAMPLE{ 3 };

/**
 * @brief The energy in the bucket predicted at the end of the mains cycle.
 *
 * @details The bucket at the start of the cycle plus the energy of the first half scaled to a
 *          whole cycle: the average power as one cycle's energy, as Robin's
 *          `energyInBucket_long + averagePower`.
 *
 * @param bucket The bucket level at the start of the cycle, in bucket units.
 * @param energyFirstHalf The energy of the sample sets so far, in bucket units, converted as a
 *                        whole cycle's contribution of samplesPerCycle sample sets.
 * @param samplesFirstHalf The number of sample sets so far.
 * @param samplesPerCycle The number of sample sets of a whole cycle that energyFirstHalf was
 *                        converted for.
 * @return bucket + energyFirstHalf x samplesPerCycle / samplesFirstHalf (integer division,
 *         truncated towards 0), or bucket when samplesFirstHalf is 0.
 *
 * @pre |energyFirstHalf| x samplesPerCycle fits in an int32_t (the caller checks its range at
 *      compile time).
 */
constexpr int32_t predictEndOfCycle(const int32_t bucket, const int32_t energyFirstHalf, const uint8_t samplesFirstHalf, const uint8_t samplesPerCycle)
{
  if (0 == samplesFirstHalf)
  {
    return bucket;
  }
  return bucket + energyFirstHalf * samplesPerCycle / samplesFirstHalf;
}

/** @brief No entry of the per-sample calibration table for this half cycle (wholeCycleIndexOf()). */
inline constexpr uint8_t NO_INDEX{ UINT8_MAX };

/**
 * @brief The entry of the per-sample calibration table that turns the first half into a whole
 *        cycle's energy without a division.
 *
 * @details The entry for n sample sets holds cal / n (Energy::toFixedPerSample()): the entry
 *          for 2n, applied to 2 x sumP, gives sumP x cal / n, the first half's average power as
 *          one cycle's energy, which predictEndOfCycle() above computes with a division.
 *
 * @param samplesFirstHalf The number of sample sets so far, n.
 * @param nMin The smallest number of sample sets of the table (N_MIN).
 * @param nMax The largest number of sample sets of the table (N_MAX).
 * @return 2n - nMin when nMin <= 2n <= nMax, NO_INDEX otherwise (no proper half cycle).
 */
constexpr uint8_t wholeCycleIndexOf(const uint8_t samplesFirstHalf, const uint8_t nMin, const uint8_t nMax)
{
  const uint16_t twice{ static_cast< uint16_t >(2U * samplesFirstHalf) };
  return ((twice < nMin) || (twice > nMax)) ? NO_INDEX : static_cast< uint8_t >(twice - nMin);
}

/**
 * @brief The energy in the bucket predicted at the end of the mains cycle, without a division.
 *
 * @details As predictEndOfCycle() above: bucket + sumP x cal / n, through the table entry of
 *          wholeCycleIndexOf(), with the same rounding as the bucket's own contributions.
 *
 * @param bucket The bucket level at the start of the cycle, in bucket units.
 * @param sumFirstHalf The sum of the sample sets so far (V_ADC x I_ADC), at most nMax / 2 x 2^18.
 * @param calOfTwice The table entry wholeCycleIndexOf() gives.
 * @param sumShift The shift of the sums the table was built for (SumShift of toFixedPerSample()).
 * @param calShift The shift of the table.
 * @return The predicted bucket level.
 */
inline int32_t predictEndOfCycle(const int32_t bucket, const int32_t sumFirstHalf, const uint16_t calOfTwice, const uint8_t sumShift, const uint8_t calShift)
{
  // 2 x sumP of at most nMax / 2 sample sets: within the 24 bits of contribution() once shifted
  return bucket + Energy::contribution((sumFirstHalf * 2) >> sumShift, calOfTwice, calShift);
}
}  // namespace FastControl

#endif  // FAST_CONTROL_H
