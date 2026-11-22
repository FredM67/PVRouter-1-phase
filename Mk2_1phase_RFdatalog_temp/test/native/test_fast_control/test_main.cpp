// SPDX-License-Identifier: GPL-3.0-or-later
/**
 * @file test_main.cpp
 * @brief Native unit tests for fast_control.h: the decision on the predicted energy
 *
 * @details At the first sample set of the negative half cycle, the energy at the end of the cycle
 *          is predicted: the bucket at the start of the cycle plus the energy of the first half,
 *          scaled to a whole cycle (Robin's fasterControl). The decision, a few sample sets later,
 *          is taken on that prediction only. No config.h, no stubs: the real code.
 */

#include <unity.h>

#include "energy_bucket.h"    // Real header - pure C++
#include "fast_control.h"     // Real header - pure C++
#include "load_priorities.h"  // Real header - pure C++

void setUp(void) {}
void tearDown(void) {}

// ============================================================================
// Prediction
// ============================================================================

void test_prediction_adds_the_first_half_as_a_whole_cycle()
{
  // 300 units over 32 of 64 sample sets: 600 units over the whole cycle
  TEST_ASSERT_EQUAL_INT32(1600, FastControl::predictEndOfCycle(1000, 300, 32, 64));
}

void test_prediction_without_samples()
{
  // no sample set yet: no power known, the bucket as it is
  TEST_ASSERT_EQUAL_INT32(1000, FastControl::predictEndOfCycle(1000, 300, 0, 64));
}

void test_prediction_negative_power()
{
  // import: the prediction falls
  TEST_ASSERT_EQUAL_INT32(600, FastControl::predictEndOfCycle(1000, -200, 32, 64));
}

// computed at compile time
static_assert(FastControl::predictEndOfCycle(1000, 300, 32, 64) == 1600, "constexpr prediction");
static_assert(FastControl::DECISION_SAMPLE == 3, "Robin's POST_ZERO_CROSSING_MAX_COUNT");

// ============================================================================
// Prediction without a division, through the per-sample calibration table
// ============================================================================

/**
 * @brief Every in-range n gives the division form, within the rounding of the table.
 *
 * @details The ISR's former prediction: the first half through the table entry for a whole cycle
 *          (samplesPerCycle sample sets), then x samplesPerCycle / n. The division-free form:
 *          2 x the first half through the entry for 2n sample sets, 2 x sumP x cal / 2n. Both
 *          round: the entries to 16 bits, contribution() to the unit, the division by truncating.
 */
template< uint8_t NMin, uint8_t NMax, uint8_t SumShift >
static void checkDivisionFree(const float cal, const uint8_t samplesPerCycle)
{
  const float cals[1]{ cal };
  const auto table{ Energy::toFixedPerSample< NMin, NMax, SumShift >(cals) };

  static constexpr int32_t sums[]{ 0, 1, -1, 1000, -1000, 123456, -123456, 2000000, -2000000, (34L << 18) - 1, -((34L << 18) - 1) };

  uint8_t checked{ 0 };
  for (uint8_t n = 1; n != NMax; ++n)
  {
    const uint8_t index{ FastControl::wholeCycleIndexOf(n, NMin, NMax) };
    if ((2 * n < NMin) || (2 * n > NMax))
    {
      TEST_ASSERT_EQUAL_UINT8(FastControl::NO_INDEX, index);
      continue;
    }
    TEST_ASSERT_EQUAL_UINT8(2 * n - NMin, index);
    ++checked;

    for (const int32_t sum : sums)
    {
      if ((sum > (static_cast< int32_t >(n) << 18)) || (sum < -(static_cast< int32_t >(n) << 18)))
      {
        continue;  // beyond full scale for n sample sets
      }
      const int32_t wholeCycle{ Energy::contribution(sum >> SumShift, table.value[0][samplesPerCycle - NMin], table.shift) };
      const int32_t expected{ FastControl::predictEndOfCycle(1000, wholeCycle, n, samplesPerCycle) };
      const int32_t actual{ FastControl::predictEndOfCycle(1000, sum, table.value[0][index], SumShift, table.shift) };

      const int32_t magnitude{ expected > 1000 ? expected - 1000 : 1000 - expected };
      TEST_ASSERT_INT32_WITHIN(2 + magnitude / 8192, expected, actual);
    }
  }
  TEST_ASSERT_TRUE(checked >= 4);
}

void test_division_free_prediction_50Hz()
{
  // 64 sample sets per cycle, 60..68 expected: sumP is halved (SUM_P_SHIFT 1)
  checkDivisionFree< 60, 68, 1 >(0.05F, 64);
  checkDivisionFree< 60, 68, 1 >(0.00286405F * 50.0F, 64);
}

void test_division_free_prediction_60Hz()
{
  // 53 sample sets per cycle, 49..57 expected: no shift
  checkDivisionFree< 49, 57, 0 >(0.05F, 53);
  checkDivisionFree< 49, 57, 0 >(0.00286405F * 20.0F, 53);
}

void test_division_free_index_out_of_range()
{
  // no proper half cycle: no table entry, the caller keeps the bucket
  TEST_ASSERT_EQUAL_UINT8(FastControl::NO_INDEX, FastControl::wholeCycleIndexOf(0, 60, 68));
  TEST_ASSERT_EQUAL_UINT8(FastControl::NO_INDEX, FastControl::wholeCycleIndexOf(29, 60, 68));
  TEST_ASSERT_EQUAL_UINT8(0, FastControl::wholeCycleIndexOf(30, 60, 68));
  TEST_ASSERT_EQUAL_UINT8(8, FastControl::wholeCycleIndexOf(34, 60, 68));
  TEST_ASSERT_EQUAL_UINT8(FastControl::NO_INDEX, FastControl::wholeCycleIndexOf(35, 60, 68));
  TEST_ASSERT_EQUAL_UINT8(FastControl::NO_INDEX, FastControl::wholeCycleIndexOf(200, 60, 68));
}

// ============================================================================
// Decision
// ============================================================================

void test_decision_uses_prediction_only()
{
  constexpr LoadPriorities::Bucket bucket{ 1000, 500, 500, 500 };
  LoadPriorities::Switching< 2 > switching;
  uint8_t priorities[2]{ 0, 1 };

  const int32_t measured{ 400 };  // the bucket at the start of the cycle: below the middle
  // the first half brings 250 units over 32 of 64 sample sets: 500 over the cycle
  const int32_t prediction{ FastControl::predictEndOfCycle(measured, 250, 32, 64) };
  TEST_ASSERT_EQUAL_INT32(900, prediction);

  switching.onNewCycle(priorities, prediction, bucket);

  // Upper half: load 0 ON, the lower threshold reset to its default, the upper one untouched
  // (still its initial 0). Fed the measured 400, load 0 would stay OFF, the upper threshold would
  // be reset to 500 and the lower one would stay 0.
  TEST_ASSERT_TRUE((priorities[0] & loadStateOnBit) != 0);
  TEST_ASSERT_TRUE((priorities[1] & loadStateOnBit) == 0);
  TEST_ASSERT_EQUAL_INT32(500, switching.lowerThreshold);
  TEST_ASSERT_EQUAL_INT32(0, switching.upperThreshold);
}

int main(int argc, char** argv)
{
  (void)argc;
  (void)argv;

  UNITY_BEGIN();

  RUN_TEST(test_prediction_adds_the_first_half_as_a_whole_cycle);
  RUN_TEST(test_prediction_without_samples);
  RUN_TEST(test_prediction_negative_power);
  RUN_TEST(test_division_free_prediction_50Hz);
  RUN_TEST(test_division_free_prediction_60Hz);
  RUN_TEST(test_division_free_index_out_of_range);
  RUN_TEST(test_decision_uses_prediction_only);

  return UNITY_END();
}
