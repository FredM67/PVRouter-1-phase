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
  RUN_TEST(test_decision_uses_prediction_only);

  return UNITY_END();
}
