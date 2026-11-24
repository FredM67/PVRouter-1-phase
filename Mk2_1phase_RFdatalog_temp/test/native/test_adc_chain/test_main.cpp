// SPDX-License-Identifier: GPL-3.0-or-later
/**
 * @file test_main.cpp
 * @brief Native tests for adc_chain.h: the analog inputs, the reference and the ADMUX pipeline
 *
 * @details Three channels, sampled in a fixed chain V -> I_GRID -> I_DIVERTED. The ADC runs
 *          free: when the ISR of a finished conversion runs, the next conversion has already
 *          started with the previous ADMUX, so the ADMUX written now applies to the conversion
 *          after next. The pipeline test plays this out and checks that the channel really
 *          converted is the one the ISR believes it handles, on each board.
 */

#include <unity.h>

#include "adc_chain.h"  // Real header - pure C++

void setUp(void) {}
void tearDown(void) {}

using AdcChain::Channel;

inline constexpr PcbVersion boards[]{ PcbVersion::OLD_SEG, PcbVersion::OLD_OLED, PcbVersion::NEW };
inline constexpr Channel chain[]{ Channel::V, Channel::I_GRID, Channel::I_DIVERTED };

inline constexpr uint8_t MUX_MASK{ 0x0F };                      /**< MUX3..0: the analog input */
inline constexpr uint8_t REF_MASK{ (1U << 7) | (1U << 6) };     /**< REFS1, REFS0 */
inline constexpr uint8_t AVCC{ 1U << 6 };                       /**< REFS0 alone: AVcc */
inline constexpr uint8_t INTERNAL_1V1{ (1U << 7) | (1U << 6) }; /**< REFS1 and REFS0: internal 1.1 V */
inline constexpr uint8_t ADLAR_BIT{ 1U << 5 };                  /**< left-adjusted result */

/** @brief The channel an ADMUX value selects on a board, from its analog input. */
static Channel channelOfAdmux(const PcbVersion board, const uint8_t admux)
{
  const uint8_t pin{ static_cast< uint8_t >(admux & MUX_MASK) };
  for (const Channel ch : chain)
  {
    if (AdcChain::pinOf(board, ch) == pin)
    {
      return ch;
    }
  }
  TEST_FAIL_MESSAGE("ADMUX selects an input that is no channel of the board");
  return Channel::V;
}

void test_order_of_the_chain(void)
{
  static_assert(AdcChain::NO_OF_CONVERSIONS == 3, "V, CT1 and CT2");

  TEST_ASSERT_EQUAL_UINT8(0, static_cast< uint8_t >(Channel::V));
  TEST_ASSERT_EQUAL_UINT8(1, static_cast< uint8_t >(Channel::I_GRID));
  TEST_ASSERT_EQUAL_UINT8(2, static_cast< uint8_t >(Channel::I_DIVERTED));
}

void test_pins_per_board(void)
{
  TEST_ASSERT_EQUAL_UINT8(3, AdcChain::pinOf(PcbVersion::OLD_SEG, Channel::V));
  TEST_ASSERT_EQUAL_UINT8(5, AdcChain::pinOf(PcbVersion::OLD_SEG, Channel::I_GRID));
  TEST_ASSERT_EQUAL_UINT8(4, AdcChain::pinOf(PcbVersion::OLD_SEG, Channel::I_DIVERTED));

  TEST_ASSERT_EQUAL_UINT8(0, AdcChain::pinOf(PcbVersion::OLD_OLED, Channel::V));
  TEST_ASSERT_EQUAL_UINT8(1, AdcChain::pinOf(PcbVersion::OLD_OLED, Channel::I_GRID));
  TEST_ASSERT_EQUAL_UINT8(3, AdcChain::pinOf(PcbVersion::OLD_OLED, Channel::I_DIVERTED));

  TEST_ASSERT_EQUAL_UINT8(0, AdcChain::pinOf(PcbVersion::NEW, Channel::V));
  TEST_ASSERT_EQUAL_UINT8(1, AdcChain::pinOf(PcbVersion::NEW, Channel::I_GRID));
  TEST_ASSERT_EQUAL_UINT8(3, AdcChain::pinOf(PcbVersion::NEW, Channel::I_DIVERTED));

  static_assert(AdcChain::pinOf(PcbVersion::OLD_SEG, Channel::I_GRID) == 5, "pinOf() must be constexpr");
}

void test_reference_per_board(void)
{
  TEST_ASSERT_EQUAL_HEX8(AVCC, AdcChain::refBits(PcbVersion::OLD_SEG));
  TEST_ASSERT_EQUAL_HEX8(AVCC, AdcChain::refBits(PcbVersion::OLD_OLED));
  TEST_ASSERT_EQUAL_HEX8(INTERNAL_1V1, AdcChain::refBits(PcbVersion::NEW));

  static_assert(AdcChain::refBits(PcbVersion::NEW) == INTERNAL_1V1, "refBits() must be constexpr");
}

void test_admux_of_is_reference_adlar_and_pin(void)
{
  for (const PcbVersion board : boards)
  {
    for (const Channel ch : chain)
    {
      TEST_ASSERT_EQUAL_HEX8(AdcChain::refBits(board) | ADLAR_BIT | AdcChain::pinOf(board, ch), AdcChain::admuxOf(board, ch));
    }
  }
  TEST_ASSERT_EQUAL_HEX8(0x63, AdcChain::admuxOf(PcbVersion::OLD_SEG, Channel::V));
  TEST_ASSERT_EQUAL_HEX8(0xE3, AdcChain::admuxOf(PcbVersion::NEW, Channel::I_DIVERTED));
}

void test_after_next_of_every_position(void)
{
  // the ADMUX written after a conversion applies to the conversion after next, wrap included
  TEST_ASSERT_EQUAL_UINT8(static_cast< uint8_t >(Channel::I_DIVERTED), static_cast< uint8_t >(AdcChain::channelAfterNext(Channel::V)));
  TEST_ASSERT_EQUAL_UINT8(static_cast< uint8_t >(Channel::V), static_cast< uint8_t >(AdcChain::channelAfterNext(Channel::I_GRID)));           // wrap
  TEST_ASSERT_EQUAL_UINT8(static_cast< uint8_t >(Channel::I_GRID), static_cast< uint8_t >(AdcChain::channelAfterNext(Channel::I_DIVERTED)));  // wrap

  static_assert(AdcChain::channelAfterNext(Channel::I_DIVERTED) == Channel::I_GRID, "channelAfterNext() must be constexpr");
}

void test_admux_written_after_selects_the_channel_after_next(void)
{
  for (const PcbVersion board : boards)
  {
    for (const Channel ch : chain)
    {
      TEST_ASSERT_EQUAL_HEX8(AdcChain::admuxOf(board, AdcChain::channelAfterNext(ch)), AdcChain::admuxWrittenAfter(board, ch));
    }
  }
}

void test_channel_converted_by_round(void)
{
  for (uint8_t round = 0; round != 30; ++round)
  {
    TEST_ASSERT_EQUAL_UINT8(round % 3, static_cast< uint8_t >(AdcChain::channelConverted(round)));
  }
  static_assert(AdcChain::channelConverted(3) == Channel::V, "channelConverted() must be constexpr");
}

void test_pipeline_order(void)
{
  constexpr uint8_t ROUNDS{ 30 };

  for (const PcbVersion board : boards)
  {
    // admuxOf[k]: the ADMUX the conversion of round k runs with
    uint8_t admuxOf[ROUNDS + 2]{};

    // the chain primed with V, then I_GRID
    admuxOf[0] = AdcChain::admuxOf(board, Channel::V);
    admuxOf[1] = AdcChain::admuxOf(board, Channel::I_GRID);

    for (uint8_t round = 0; round != ROUNDS; ++round)
    {
      // the ISR of round k believes it handles channelConverted(k)...
      const Channel believed{ AdcChain::channelConverted(round) };

      // ... and the ADC really converted the channel its ADMUX selected
      TEST_ASSERT_EQUAL_UINT8(static_cast< uint8_t >(believed), static_cast< uint8_t >(channelOfAdmux(board, admuxOf[round])));
      TEST_ASSERT_EQUAL_HEX8(AdcChain::refBits(board), admuxOf[round] & REF_MASK);
      TEST_ASSERT_EQUAL_HEX8(ADLAR_BIT, admuxOf[round] & ADLAR_BIT);

      // the conversion of round k+1 has already started: the write of round k applies to round k+2
      admuxOf[round + 2] = AdcChain::admuxWrittenAfter(board, believed);
    }
  }
}

int main(int /* argc */, char** /* argv */)
{
  UNITY_BEGIN();

  RUN_TEST(test_order_of_the_chain);
  RUN_TEST(test_pins_per_board);
  RUN_TEST(test_reference_per_board);
  RUN_TEST(test_admux_of_is_reference_adlar_and_pin);
  RUN_TEST(test_after_next_of_every_position);
  RUN_TEST(test_admux_written_after_selects_the_channel_after_next);
  RUN_TEST(test_channel_converted_by_round);
  RUN_TEST(test_pipeline_order);

  return UNITY_END();
}
