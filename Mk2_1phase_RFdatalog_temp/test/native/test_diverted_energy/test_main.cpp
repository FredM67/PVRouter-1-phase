/**
 * @file test_main.cpp
 * @brief Native tests for diverted_energy.h: the diverted energy measured on CT2
 *
 * @details One call of addCycle() per mains cycle, with the cycle's energy in integer energy
 *          units (IEU), whether a local load or relay is ON, and whether one is forced. The
 *          energy above the anti-creep limit is summed, whole Wh carried into a 16-bit total;
 *          a forced cycle is not counted; after SHUTDOWN_CYCLES cycles without diversion,
 *          everything is reset, as Robin's display shutdown. No config.h, no stubs.
 */

#include <unity.h>

#include <stdint.h>

#include "diverted_energy.h"  // Real header - pure C++

void setUp(void) {}
void tearDown(void) {}

/** @brief Adds 'count' cycles of the same energy and states. */
template< typename E >
static void addCycles(E& energy, const uint32_t count, const int32_t energyIEU, const bool anyLoadOn, const bool anyLoadForced)
{
  for (uint32_t i = 0; i != count; ++i)
  {
    energy.addCycle(energyIEU, anyLoadOn, anyLoadForced);
  }
}

void test_whole_wh_carried(void)
{
  DivertedEnergy< 100, 5, 1000 > energy;

  energy.addCycle(60, true, false);
  TEST_ASSERT_EQUAL_UINT16(0, energy.totalWh());
  energy.addCycle(60, true, false);
  TEST_ASSERT_EQUAL_UINT16(1, energy.totalWh());
  energy.addCycle(60, true, false);  // the remainder (20) was kept: 80
  TEST_ASSERT_EQUAL_UINT16(1, energy.totalWh());
  energy.addCycle(60, true, false);  // 140: one more Wh
  TEST_ASSERT_EQUAL_UINT16(2, energy.totalWh());
  TEST_ASSERT_TRUE(energy.isActive());
}

void test_anti_creep_drops_noise(void)
{
  DivertedEnergy< 100, 5, 100000 > energy;

  addCycles(energy, 1000, 4, true, false);

  TEST_ASSERT_EQUAL_UINT16(0, energy.totalWh());
  TEST_ASSERT_FALSE(energy.isActive());

  energy.addCycle(5, true, false);  // at the limit: counted
  TEST_ASSERT_TRUE(energy.isActive());
}

void test_negative_energy_is_not_counted(void)
{
  DivertedEnergy< 100, 0, 1000 > energy;  // no anti-creep limit

  addCycles(energy, 10, 60, true, false);
  addCycles(energy, 10, -60, true, false);  // CT2 the wrong way round, or noise: not subtracted

  TEST_ASSERT_EQUAL_UINT16(6, energy.totalWh());
}

void test_reset_after_inactivity(void)
{
  DivertedEnergy< 100, 5, 10 > energy;

  addCycles(energy, 4, 60, true, false);
  TEST_ASSERT_EQUAL_UINT16(2, energy.totalWh());

  addCycles(energy, 9, 0, false, false);  // 9 cycles without any load ON: kept
  TEST_ASSERT_EQUAL_UINT16(2, energy.totalWh());
  TEST_ASSERT_TRUE(energy.isActive());

  energy.addCycle(0, true, false);         // a load ON again: the count starts over
  addCycles(energy, 10, 0, false, false);  // 10: kept
  TEST_ASSERT_EQUAL_UINT16(2, energy.totalWh());

  energy.addCycle(0, true, false);
  addCycles(energy, 11, 0, false, false);  // 11: reset
  TEST_ASSERT_EQUAL_UINT16(0, energy.totalWh());
  TEST_ASSERT_FALSE(energy.isActive());

  energy.addCycle(60, true, false);  // the remainder was reset too: 60 only
  energy.addCycle(30, true, false);
  TEST_ASSERT_EQUAL_UINT16(0, energy.totalWh());
}

void test_total_saturates(void)
{
  DivertedEnergy< 10, 1, 1000 > energy;

  addCycles(energy, 65535UL, 10, true, false);
  TEST_ASSERT_EQUAL_UINT16(65535U, energy.totalWh());

  addCycles(energy, 1000, 10, true, false);
  TEST_ASSERT_EQUAL_UINT16(65535U, energy.totalWh());  // stops, never wraps
}

void test_forced_cycle_not_counted(void)
{
  DivertedEnergy< 100, 5, 1000 > forced;
  addCycles(forced, 10, 60, true, true);
  TEST_ASSERT_EQUAL_UINT16(0, forced.totalWh());
  TEST_ASSERT_FALSE(forced.isActive());

  DivertedEnergy< 100, 5, 1000 > notForced;
  addCycles(notForced, 10, 60, true, false);
  TEST_ASSERT_EQUAL_UINT16(6, notForced.totalWh());
}

void test_forced_cycles_count_towards_reset(void)
{
  // A boost at night keeps a load ON, but diverts no surplus: the total of the day must still
  // be reset after SHUTDOWN_CYCLES, as in Robin's sketch, whose count follows the diversion only.
  DivertedEnergy< 100, 5, 10 > energy;

  addCycles(energy, 4, 60, true, false);
  addCycles(energy, 5, 0, false, false);
  addCycles(energy, 6, 60, true, true);  // forced: 11 cycles without diversion
  TEST_ASSERT_EQUAL_UINT16(0, energy.totalWh());
  TEST_ASSERT_FALSE(energy.isActive());
}

void test_counted_energy(void)
{
  using E = DivertedEnergy< 100, 5, 10 >;

  TEST_ASSERT_EQUAL_INT32(0, E::countedEnergy(4, false));
  TEST_ASSERT_EQUAL_INT32(5, E::countedEnergy(5, false));
  TEST_ASSERT_EQUAL_INT32(0, E::countedEnergy(-500, false));
  TEST_ASSERT_EQUAL_INT32(0, E::countedEnergy(500, true));
}

void test_average_watts(void)
{
  // 250 cycles (5 s at 50 Hz), 16 IEU per W per cycle, at most 600000 IEU per cycle
  constexpr DivertedPower::Average average{ DivertedPower::averageOver(250, 250UL * 600000UL, 4) };

  TEST_ASSERT_EQUAL_INT16(0, DivertedPower::watts(0, average));
  TEST_ASSERT_EQUAL_INT16(1000, DivertedPower::watts(250UL * 16000UL, average));    // 1000 W all along
  TEST_ASSERT_EQUAL_INT16(500, DivertedPower::watts(125UL * 16000UL, average));     // half the cycles forced: 0 W
  TEST_ASSERT_EQUAL_INT16(32767, DivertedPower::watts(250UL * 600000UL, average));  // 37500 W: saturated
  TEST_ASSERT_EQUAL_INT16(2345, DivertedPower::watts(250UL * 16UL * 2345UL, average));

  // 40 s at 60 Hz, the longest period
  constexpr DivertedPower::Average longest{ DivertedPower::averageOver(2400, 2400UL * 600000UL, 4) };
  TEST_ASSERT_EQUAL_INT16(1234, DivertedPower::watts(2400UL * 16UL * 1234UL, longest));
  TEST_ASSERT_EQUAL_INT16(7, DivertedPower::watts(2400UL * 16UL * 7UL, longest));
}

int main(int /* argc */, char** /* argv */)
{
  UNITY_BEGIN();

  RUN_TEST(test_whole_wh_carried);
  RUN_TEST(test_anti_creep_drops_noise);
  RUN_TEST(test_negative_energy_is_not_counted);
  RUN_TEST(test_reset_after_inactivity);
  RUN_TEST(test_total_saturates);
  RUN_TEST(test_forced_cycle_not_counted);
  RUN_TEST(test_forced_cycles_count_towards_reset);
  RUN_TEST(test_counted_energy);
  RUN_TEST(test_average_watts);

  return UNITY_END();
}
