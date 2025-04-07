#include <Arduino.h>
#include <unity.h>

#include "FastDivision.h"  // Include the header file for the functions to test

void setUp(void)
{
  // Set up code here (if needed)
}

void tearDown(void)
{
  // Clean up code here (if needed)
}

// Test for divu10
void test_divu10(void)
{
  // Basic cases
  TEST_ASSERT_EQUAL(1, divu10(10));  // 10 / 10 = 1
  TEST_ASSERT_EQUAL(2, divu10(20));  // 20 / 10 = 2
  TEST_ASSERT_EQUAL(0, divu10(0));   // 0 / 10 = 0
  TEST_ASSERT_EQUAL(9, divu10(99));  // 99 / 10 = 9

  // Edge cases
  TEST_ASSERT_EQUAL(0, divu10(1));     // 1 / 10 = 0
  TEST_ASSERT_EQUAL(0, divu10(9));     // 9 / 10 = 0
  TEST_ASSERT_EQUAL(10, divu10(100));  // 100 / 10 = 10
  TEST_ASSERT_EQUAL(11, divu10(110));  // 110 / 10 = 11

  // Large values
  TEST_ASSERT_EQUAL(100, divu10(1000));    // 1000 / 10 = 100
  TEST_ASSERT_EQUAL(6553, divu10(65535));  // 65535 / 10 = 6553 (max 16-bit unsigned value)

  // Random values
  TEST_ASSERT_EQUAL(25, divu10(250));    // 250 / 10 = 25
  TEST_ASSERT_EQUAL(123, divu10(1234));  // 1234 / 10 = 123
  TEST_ASSERT_EQUAL(42, divu10(425));    // 425 / 10 = 42
}

// Test for divmod10
void test_divmod10(void)
{
  uint32_t quotient;
  uint8_t remainder;

  // Basic cases
  divmod10(10, quotient, remainder);
  TEST_ASSERT_EQUAL(1, quotient);   // 10 / 10 = 1
  TEST_ASSERT_EQUAL(0, remainder);  // 10 % 10 = 0

  divmod10(20, quotient, remainder);
  TEST_ASSERT_EQUAL(2, quotient);   // 20 / 10 = 2
  TEST_ASSERT_EQUAL(0, remainder);  // 20 % 10 = 0

  divmod10(0, quotient, remainder);
  TEST_ASSERT_EQUAL(0, quotient);   // 0 / 10 = 0
  TEST_ASSERT_EQUAL(0, remainder);  // 0 % 10 = 0

  divmod10(99, quotient, remainder);
  TEST_ASSERT_EQUAL(9, quotient);   // 99 / 10 = 9
  TEST_ASSERT_EQUAL(9, remainder);  // 99 % 10 = 9

  // Edge cases
  divmod10(1, quotient, remainder);
  TEST_ASSERT_EQUAL(0, quotient);   // 1 / 10 = 0
  TEST_ASSERT_EQUAL(1, remainder);  // 1 % 10 = 1

  divmod10(9, quotient, remainder);
  TEST_ASSERT_EQUAL(0, quotient);   // 9 / 10 = 0
  TEST_ASSERT_EQUAL(9, remainder);  // 9 % 10 = 9

  divmod10(100, quotient, remainder);
  TEST_ASSERT_EQUAL(10, quotient);  // 100 / 10 = 10
  TEST_ASSERT_EQUAL(0, remainder);  // 100 % 10 = 0

  divmod10(101, quotient, remainder);
  TEST_ASSERT_EQUAL(10, quotient);  // 101 / 10 = 10
  TEST_ASSERT_EQUAL(1, remainder);  // 101 % 10 = 1

  // Large values
  divmod10(65535, quotient, remainder);
  TEST_ASSERT_EQUAL(6553, quotient);  // 65535 / 10 = 6553
  TEST_ASSERT_EQUAL(5, remainder);    // 65535 % 10 = 5

  // Random values
  divmod10(250, quotient, remainder);
  TEST_ASSERT_EQUAL(25, quotient);  // 250 / 10 = 25
  TEST_ASSERT_EQUAL(0, remainder);  // 250 % 10 = 0

  divmod10(1234, quotient, remainder);
  TEST_ASSERT_EQUAL(123, quotient);  // 1234 / 10 = 123
  TEST_ASSERT_EQUAL(4, remainder);   // 1234 % 10 = 4

  divmod10(425, quotient, remainder);
  TEST_ASSERT_EQUAL(42, quotient);  // 425 / 10 = 42
  TEST_ASSERT_EQUAL(5, remainder);  // 425 % 10 = 5

  uint32_t tmpVal;
  uint8_t digit;

  divmod10(2345, tmpVal, digit);
  TEST_ASSERT_EQUAL(5, digit);

  divmod10(tmpVal, tmpVal, digit);
  TEST_ASSERT_EQUAL(4, digit);

  divmod10(tmpVal, tmpVal, digit);
  TEST_ASSERT_EQUAL(3, digit);
  TEST_ASSERT_EQUAL(2, tmpVal);
}

void setup()
{
  delay(1000);    // Wait for Serial to initialize
  UNITY_BEGIN();  // Start Unity test framework
}

uint8_t i = 0;
uint8_t max_blinks = 1;

void loop()
{
  if (i < max_blinks)
  {
    RUN_TEST(test_divu10);
    delay(100);
    RUN_TEST(test_divmod10);
    delay(100);
    ++i;
  }
  else if (i == max_blinks)
  {
    UNITY_END();  // End Unity test framework
  }
}