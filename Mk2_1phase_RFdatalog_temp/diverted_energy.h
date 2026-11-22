/**
 * @file diverted_energy.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief The diverted energy measured on CT2: Wh total, anti-creep limit, reset after inactivity
 * @version 1.0
 * @date 2026-11-22
 *
 * @copyright Copyright (c) 2026-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * @details Robin's diverted energy of Mk2_fasterControl_Full (processLatestContribution()):
 *          once per mains cycle, the energy measured on CT2 is added to an accumulator if it
 *          reaches the anti-creep limit, and whole Wh are carried into a 16-bit total. After a
 *          long period without diversion (DISPLAY_SHUTDOWN_IN_HOURS), the total is reset, for
 *          the next day's data.
 *
 *          What differs from Full:
 *          - the cycles during which a local load or relay is forced (override pin, scheduled
 *            boost) are not counted at all: CT2 measures every local output, it cannot tell
 *            the forced load from the others (Full skips CT2 while its single load is forced);
 *          - "active" (Robin's EDD_isActive) means some energy counted since the last reset;
 *            the energy is counted from the first cycle, there is no gate;
 *          - the reset counts the cycles without diversion: without any local load or relay ON,
 *            or with one forced. Full counts on the diversion state of its first load, which an
 *            override does not change: a boost at night must not keep the day's total alive.
 *
 *          The energy unit (IEU) is the caller's: the router passes the cycle's average power
 *          in W x 2^Energy::FRACTION_BITS, the unit of its energy bucket (energy_bucket.h).
 *
 *          Pure C++, no Arduino: the native tests include it.
 */

#ifndef DIVERTED_ENERGY_H
#define DIVERTED_ENERGY_H

#include <stdint.h>

#include "mult_asm.h"

/**
 * @brief The diverted energy: a Wh total and the "active" flag of Robin's display.
 *
 * @tparam IEU_PER_WH Energy units in one Wh.
 * @tparam ANTI_CREEP_IEU Smallest energy of a cycle that counts, in energy units (0 or more).
 * @tparam SHUTDOWN_CYCLES Beyond that many cycles without diversion, everything is reset.
 */
template< int32_t IEU_PER_WH, int32_t ANTI_CREEP_IEU, uint32_t SHUTDOWN_CYCLES >
class DivertedEnergy
{
  static_assert(IEU_PER_WH > 0, "IEU_PER_WH must be positive");
  static_assert(ANTI_CREEP_IEU >= 0, "ANTI_CREEP_IEU must not be negative");

public:
  /**
   * @brief The energy of a cycle that counts: 0 when forced, or below the anti-creep limit.
   *
   * @param energyIEU The cycle's energy measured on CT2.
   * @param anyLoadForced Whether a local load or relay is ON because of an override.
   * @return The energy counted, 0 or more (at least ANTI_CREEP_IEU when not 0).
   */
  static constexpr int32_t countedEnergy(const int32_t energyIEU, const bool anyLoadForced)
  {
    return (anyLoadForced || (energyIEU < ANTI_CREEP_IEU)) ? 0 : energyIEU;
  }

  /**
   * @brief Adds one mains cycle.
   *
   * @param energyIEU The cycle's energy measured on CT2, below IEU_PER_WH.
   * @param anyLoadOn Whether any local load or relay is ON.
   * @param anyLoadForced Whether any local load or relay is ON because of an override.
   */
  void addCycle(const int32_t energyIEU, const bool anyLoadOn, const bool anyLoadForced)
  {
    const int32_t counted{ countedEnergy(energyIEU, anyLoadForced) };

    if (counted > 0)
    {
      active = true;
      recentIEU += counted;

      // whole Wh into the total: one at most, a cycle's energy being below one Wh
      if (recentIEU >= IEU_PER_WH)
      {
        recentIEU -= IEU_PER_WH;
        if (total != UINT16_MAX)
        {
          ++total;
        }
      }
    }

    if (anyLoadOn && !anyLoadForced)
    {
      cyclesWithoutDiversion = 0;
    }
    else if (++cyclesWithoutDiversion > SHUTDOWN_CYCLES)
    {
      reset();
    }
  }

  /** @brief The diverted energy since the last reset, in Wh, saturated to 65535. */
  uint16_t totalWh() const
  {
    return total;
  }

  /** @brief Whether some energy was counted since the last reset (Robin's EDD_isActive). */
  bool isActive() const
  {
    return active;
  }

private:
  /** @brief Clears everything, the count of cycles without diversion included. */
  void reset()
  {
    recentIEU = 0;
    total = 0;
    active = false;
    cyclesWithoutDiversion = 0;
  }

  int32_t recentIEU{ 0 };              /**< energy below one Wh (Robin's divertedEnergyRecent_IEU) */
  uint32_t cyclesWithoutDiversion{ 0 }; /**< Robin's absenceOfDivertedEnergyCountInMC */
  uint16_t total{ 0 };                 /**< whole Wh (Robin's divertedEnergyTotal_Wh) */
  bool active{ false };                /**< Robin's EDD_isActive */
};

namespace DivertedPower
{
/**
 * @brief Division by the number of cycles of the datalog period, as a fixed-point product.
 *
 * @details watts() computes ((sum >> preShift) x factor) >> (8 + postShift): a 24 x 16 bit
 *          multiplication (multU24x16_to32_hi8(), ~30 cycles on AVR) instead of a 32-bit
 *          division (~600 cycles), in the ISR.
 */
struct Average
{
  uint8_t preShift;  /**< the sum, shifted down to 24 bits */
  uint16_t factor;   /**< round(2^(preShift + 8 + postShift) / divisor) */
  uint8_t postShift; /**< shift after the multiplication */
};

/**
 * @brief The fixed-point division for an average power over a datalog period.
 *
 * @param cycles The mains cycles of the period.
 * @param maxSum The largest sum of the period, in energy units.
 * @param fractionBits The fractional bits of the energy unit: one cycle of 1 W adds 2^fractionBits.
 */
constexpr Average averageOver(const uint32_t cycles, const uint32_t maxSum, const uint8_t fractionBits)
{
  const uint64_t divisor{ static_cast< uint64_t >(cycles) << fractionBits };

  uint8_t pre{ 0 };
  while ((maxSum >> pre) >= (1UL << 24))
  {
    ++pre;
  }

  uint8_t post{ 0 };
  while ((pre + 8 + post + 1 < 63) && (((1ULL << (pre + 8 + post + 1)) + divisor / 2) / divisor <= UINT16_MAX))
  {
    ++post;
  }

  return Average{ pre, static_cast< uint16_t >(((1ULL << (pre + 8 + post)) + divisor / 2) / divisor), post };
}

/**
 * @brief The average power over the period, in W.
 *
 * @param sum The energy of the period, in energy units (at most the maxSum of averageOver()).
 * @param average From averageOver().
 * @return sum / (cycles x 2^fractionBits), rounded, saturated to 32767 W.
 */
inline int16_t watts(const uint32_t sum, const Average& average)
{
  uint32_t product;  // ((sum >> preShift) x factor) >> 8
  multU24x16_to32_hi8(product, static_cast< uint32_t >(sum >> average.preShift), average.factor);

  if (average.postShift != 0)
  {
    product = (product + (1UL << (average.postShift - 1))) >> average.postShift;
  }

  return product > INT16_MAX ? INT16_MAX : static_cast< int16_t >(product);
}
}  // namespace DivertedPower

#endif  // DIVERTED_ENERGY_H
