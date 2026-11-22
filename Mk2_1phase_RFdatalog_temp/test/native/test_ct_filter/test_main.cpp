/**
 * @file test_main.cpp
 * @brief Native tests for ct_filter.h: the CT1 filter, at the resolution of Robin's sketches
 *
 * @details The current sample reaches the filter DC-removed, left-aligned (x64 scale, int16).
 *          The filter works on an int32 at x256, as Mk2_fasterControl_Full does, then returns
 *          to x64 and saturates to the int16 range instead of wrapping.
 */

#include <unity.h>

#include <stdint.h>

#include "ct_filter.h"  // Real header - pure C++

void setUp(void) {}
void tearDown(void) {}

inline constexpr float GAIN{ 9 };        /**< interim value of the old boards */
inline constexpr float ALPHA{ 0.0011F }; /**< interim value of the old boards */

/** @brief Robin's filter, verbatim from Mk2_fasterControl_Full/processing.cpp (x256 scale). */
struct Robin
{
  int32_t lpf_long{};

  int32_t apply(const int16_t countsMinusDC, const float lpf_gain, const float alpha)
  {
    int32_t sampleIminusDC_grid = ((int32_t)countsMinusDC) << 8;
    const int32_t last_lpf_long = lpf_long;
    lpf_long += alpha * (sampleIminusDC_grid - last_lpf_long);
    sampleIminusDC_grid += (lpf_gain * lpf_long);
    return sampleIminusDC_grid;
  }
};

/** @brief A DC-removed sample in ADC counts, at the x64 scale of the firmware. */
static int16_t x64(const int16_t counts)
{
  return static_cast< int16_t >(counts * 64);
}

void test_gain_zero_is_identity(void)
{
  CtFilter::LowPassBoost filter;
  static constexpr int16_t samples[]{ 0, 1, -1, 63, -64, 1000, -1000, 32704, -32768, 32767, 12345, -23456 };

  for (uint8_t round = 0; round != 50; ++round)
  {
    for (const int16_t sample : samples)
    {
      TEST_ASSERT_EQUAL_INT16(sample, filter.apply(sample, 0.0F, ALPHA));
    }
  }
}

void test_full_scale_with_a_dc_residual_saturates(void)
{
  // a DC residual of 6 counts builds up the low-pass state...
  CtFilter::LowPassBoost positive;
  CtFilter::LowPassBoost negative;
  for (uint16_t i = 0; i != 20000; ++i)
  {
    positive.apply(x64(6), GAIN, ALPHA);
    negative.apply(x64(-6), GAIN, ALPHA);
  }
  TEST_ASSERT_TRUE(positive.lpf > 0);
  TEST_ASSERT_TRUE(negative.lpf < 0);

  // ... then a current peak at full scale: beyond int16, it saturates with its sign kept
  TEST_ASSERT_EQUAL_INT16(INT16_MAX, positive.apply(x64(511), GAIN, ALPHA));
  TEST_ASSERT_EQUAL_INT16(INT16_MIN, negative.apply(x64(-512), GAIN, ALPHA));
}

void test_a_small_steady_input_moves_the_state(void)
{
  // 4 ADC counts: alpha x 4 x 256 = 1.13 > 1, the state moves (at x64 it would not: 0.28)
  CtFilter::LowPassBoost filter;
  filter.apply(x64(4), GAIN, ALPHA);
  TEST_ASSERT_EQUAL_INT32(1, filter.lpf);

  CtFilter::LowPassBoost negativeFilter;
  negativeFilter.apply(x64(-4), GAIN, ALPHA);
  TEST_ASSERT_EQUAL_INT32(-1, negativeFilter.lpf);
}

void test_matches_robins_formula(void)
{
  // a 50 Hz current of 200 counts peak with a DC residual of 5 counts, 64 samples per cycle
  static constexpr int16_t sine64[16]{ 0, 20, 39, 56, 71, 83, 92, 98, 100, 98, 92, 83, 71, 56, 39, 20 };

  CtFilter::LowPassBoost filter;
  Robin robin;

  for (uint16_t i = 0; i != 64U * 50U; ++i)  // one second
  {
    const uint8_t k{ static_cast< uint8_t >(i % 64) };
    const int16_t quarter{ sine64[k % 16] };
    const int16_t counts{ static_cast< int16_t >(5 + 2 * ((k < 32) ? quarter : -quarter)) };

    const int32_t expected{ robin.apply(counts, GAIN, ALPHA) >> 2 };  // back to x64
    const int16_t actual{ filter.apply(x64(counts), GAIN, ALPHA) };

    TEST_ASSERT_EQUAL_INT32(robin.lpf_long, filter.lpf);
    TEST_ASSERT_EQUAL_INT32(expected, actual);
  }
}

int main(int /* argc */, char** /* argv */)
{
  UNITY_BEGIN();

  RUN_TEST(test_gain_zero_is_identity);
  RUN_TEST(test_full_scale_with_a_dc_residual_saturates);
  RUN_TEST(test_a_small_steady_input_moves_the_state);
  RUN_TEST(test_matches_robins_formula);

  return UNITY_END();
}
