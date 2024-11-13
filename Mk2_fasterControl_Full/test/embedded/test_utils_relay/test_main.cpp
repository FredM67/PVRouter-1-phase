#include <Arduino.h>
#include <U8g2lib.h>

#include <unity.h>

#include "utils_pins.h"
#include "utils_relay.h"

constexpr RelayEngine relays{ { { 2, 1000, 200, 1, 1 },
                                { 3, 100, 20, 2, 3 } } };

void setUp(void)
{
  relays.initializePins();
}

void tearDown(void)
{
  // clean stuff up here
}

void test_get_pin(void)
{
  TEST_ASSERT_EQUAL(2, relays.get_relay(0).get_pin());
  TEST_ASSERT_EQUAL(3, relays.get_relay(1).get_pin());
}

void test_get_surplusThreshold(void)
{
  TEST_ASSERT_EQUAL(1000, relays.get_relay(0).get_surplusThreshold());
}

void test_get_importThreshold(void)
{
  TEST_ASSERT_EQUAL(20, relays.get_relay(1).get_importThreshold());
}

void test_get_minON(void)
{
  TEST_ASSERT_EQUAL(1 * 60, relays.get_relay(0).get_minON());
}

void test_get_minOFF(void)
{
  TEST_ASSERT_EQUAL(3 * 60, relays.get_relay(1).get_minOFF());
}

void test_isRelayON(void)
{
  TEST_ASSERT_FALSE(relays.get_relay(1).isRelayON());
}

void test_relay_turnON(void)
{
  const auto& my_relay{ relays.get_relay(1) };

  TEST_ASSERT_FALSE(my_relay.isRelayON());

  /* The relay is OFF, test the "TurnON" case */
  const auto surplus{ -my_relay.get_surplusThreshold() - 1 };

  TEST_ASSERT_FALSE(my_relay.proceed_relay(surplus));
  delay(100);
  TEST_ASSERT_FALSE(my_relay.isRelayON());

  for (uint8_t timer = 0; timer < my_relay.get_minOFF() - 1; ++timer)
  {
    my_relay.inc_duration();
  }
  TEST_ASSERT_FALSE(my_relay.proceed_relay(surplus));
  delay(100);

  my_relay.inc_duration();

  TEST_ASSERT_TRUE(my_relay.proceed_relay(surplus));
  TEST_ASSERT_TRUE(my_relay.isRelayON());
}

void test_relay_turnOFF(void)
{
  const auto& my_relay{ relays.get_relay(1) };

  TEST_ASSERT_TRUE(my_relay.isRelayON());

  /* The relay is ON, test the "TurnOFF" case */
  const auto consum{ my_relay.get_importThreshold() + 1 };

  TEST_ASSERT_FALSE(my_relay.proceed_relay(consum));
  delay(100);
  TEST_ASSERT_TRUE(my_relay.isRelayON());

  for (uint8_t timer = 0; timer < my_relay.get_minON() - 1; ++timer)
  {
    my_relay.inc_duration();
  }
  TEST_ASSERT_FALSE(my_relay.proceed_relay(consum));
  delay(100);

  my_relay.inc_duration();

  TEST_ASSERT_TRUE(my_relay.proceed_relay(consum));
  TEST_ASSERT_FALSE(my_relay.isRelayON());
}

void test_proceed_relay(void)
{
  RUN_TEST(test_relay_turnON);
  delay(100);
  RUN_TEST(test_relay_turnOFF);
}

void test_get_size(void)
{
  TEST_ASSERT_EQUAL(2, relays.get_size());
}

void setup()
{
  delay(1000);

  UNITY_BEGIN();  // IMPORTANT LINE!
}

uint8_t i = 0;
uint8_t max_blinks = 1;

void loop()
{
  RUN_TEST(test_get_pin);

  delay(100);
  RUN_TEST(test_get_surplusThreshold);

  delay(100);
  RUN_TEST(test_get_importThreshold);

  delay(100);
  RUN_TEST(test_get_minON);

  delay(100);
  RUN_TEST(test_get_minOFF);

  delay(100);
  RUN_TEST(test_isRelayON);

  delay(100);
  RUN_TEST(test_proceed_relay);

  delay(100);
  RUN_TEST(test_get_size);

  UNITY_END();  // stop unit testing
}