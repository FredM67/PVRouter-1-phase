// SPDX-License-Identifier: GPL-3.0-or-later
/**
 * @file test_main.cpp
 * @brief Native tests for ct_filter.h: the CT1 filter, in fixed point
 *
 * @details The current sample reaches the filter DC-removed, left-aligned (x64 scale, int16).
 *          The filter is Robin's (Mk2_fasterControl_Full), with the float maths replaced by
 *          fixed point: these tests hold it to his time constant, his output and his dead zone.
 */

#include <unity.h>

#include <math.h>
#include <stdint.h>

#include "ct_filter.h"  // Real header - pure C++

void setUp(void) {}
void tearDown(void) {}

inline constexpr float GAIN{ 9 };        /**< interim value of the old boards */
inline constexpr float ALPHA{ 0.0011F }; /**< interim value of the old boards */

inline constexpr CtFilter::Coefficients COEFFS{ CtFilter::coefficientsOf(GAIN, ALPHA) };
inline constexpr CtFilter::Coefficients NO_GAIN{ CtFilter::coefficientsOf(0.0F, ALPHA) };

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

/** @brief The low-pass state of the filter, in ADC counts. */
static float lowPassCounts(const CtFilter::LowPassBoost &filter)
{
  return static_cast< float >(static_cast< double >(filter.lowPass()) / (64.0 * (1L << CtFilter::LPF_FRACTION_BITS)));
}

void test_gain_zero_is_identity(void)
{
  CtFilter::LowPassBoost filter;
  static constexpr int16_t samples[]{ 0, 1, -1, 63, -64, 1000, -1000, 32704, -32768, 32767, 12345, -23456 };

  for (uint8_t round = 0; round != 50; ++round)
  {
    for (const int16_t sample : samples)
    {
      TEST_ASSERT_EQUAL_INT16(sample, filter.apply(sample, NO_GAIN));
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
    positive.apply(x64(6), COEFFS);
    negative.apply(x64(-6), COEFFS);
  }
  TEST_ASSERT_TRUE(positive.lowPass() > 0);
  TEST_ASSERT_TRUE(negative.lowPass() < 0);

  // ... then a current peak at full scale: beyond int16, it saturates with its sign kept
  TEST_ASSERT_EQUAL_INT16(INT16_MAX, positive.apply(x64(511), COEFFS));
  TEST_ASSERT_EQUAL_INT16(INT16_MIN, negative.apply(x64(-512), COEFFS));
}

void test_a_small_steady_input_moves_the_state(void)
{
  // 4 ADC counts: alpha x 4 x 256 = 1.13 > 1, the state moves (at x64 it would not: 0.28)
  CtFilter::LowPassBoost filter;
  filter.apply(x64(4), COEFFS);
  TEST_ASSERT_EQUAL_INT32(1, filter.lowPass());

  CtFilter::LowPassBoost negativeFilter;
  negativeFilter.apply(x64(-4), COEFFS);
  TEST_ASSERT_EQUAL_INT32(-1, negativeFilter.lowPass());
}

void test_time_constant_within_2_percent_of_robins(void)
{
  // Robin's low-pass, in float: lpf += alpha x (x - lpf). After a step, it reaches 1 - 1/e of
  // the step in -1 / ln(1 - alpha) samples (908.6 with alpha 0.0011).
  const float expected{ static_cast< float >(-1.0 / log(1.0 - static_cast< double >(ALPHA))) };

  static constexpr int16_t STEP{ 200 };  // counts
  CtFilter::LowPassBoost filter;
  uint16_t samples{ 0 };
  while (lowPassCounts(filter) < static_cast< float >(STEP * (1.0 - exp(-1.0))))
  {
    filter.apply(x64(STEP), COEFFS);
    ++samples;
    TEST_ASSERT_TRUE(samples < 10000);
  }

  TEST_ASSERT_FLOAT_WITHIN(expected * 0.02F, expected, static_cast< float >(samples));
}

void test_steady_state_within_one_count_of_robins(void)
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
    const int16_t actual{ filter.apply(x64(counts), COEFFS) };

    TEST_ASSERT_INT32_WITHIN(64, expected, actual);  // one ADC count
  }

  // and the state has settled on the DC residual as Robin's has
  TEST_ASSERT_FLOAT_WITHIN(1.0F, static_cast< float >(robin.lpf_long) / 256.0F, lowPassCounts(filter));
}

int main(int /* argc */, char ** /* argv */)
{
  UNITY_BEGIN();

  RUN_TEST(test_gain_zero_is_identity);
  RUN_TEST(test_full_scale_with_a_dc_residual_saturates);
  RUN_TEST(test_a_small_steady_input_moves_the_state);
  RUN_TEST(test_time_constant_within_2_percent_of_robins);
  RUN_TEST(test_steady_state_within_one_count_of_robins);

  return UNITY_END();
}
