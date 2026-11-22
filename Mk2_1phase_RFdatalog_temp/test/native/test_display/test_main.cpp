/**
 * @file test_main.cpp
 * @brief Native tests for display_core.h: the pins, port images and content of the display
 *
 * @details The pins are those of Mk2_fasterControl_Full. The port images are checked against
 *          the ATmega328P ports (D0-D7 on PORTD, D8-D13 on PORTB, D14-D19 on PORTC). No
 *          config.h, no stubs.
 */

#include <unity.h>

#include <initializer_list>
#include <stdint.h>
#include <string.h>

#include "display_core.h"  // Real header - pure C++

using namespace Display;

void setUp(void) {}
void tearDown(void) {}

namespace
{
uint32_t maskOf(std::initializer_list< uint8_t > pins)
{
  uint32_t mask{ 0 };
  for (const auto pin : pins)
  {
    mask |= 1UL << pin;
  }
  return mask;
}

void assertImage(const PortImage expected, const PortImage actual, const char* message)
{
  TEST_ASSERT_EQUAL_HEX8_MESSAGE(expected.b, actual.b, message);
  TEST_ASSERT_EQUAL_HEX8_MESSAGE(expected.c, actual.c, message);
  TEST_ASSERT_EQUAL_HEX8_MESSAGE(expected.d, actual.d, message);
}

void assertChars(const uint8_t a, const uint8_t b, const uint8_t c, const uint8_t d, const uint8_t (&chars)[NO_OF_DIGITS], const char* message)
{
  const uint8_t expected[NO_OF_DIGITS]{ a, b, c, d };
  TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE(expected, chars, NO_OF_DIGITS, message);
}
}  // namespace

/** The pins of each display type, as Full: the list for the checks, and the mask. */
void test_pins_of_each_type()
{
  TEST_ASSERT_EQUAL_UINT8(0, pinsOf(DisplayType::NONE).count);
  TEST_ASSERT_EQUAL_HEX32(0, pinMaskOf(DisplayType::NONE));

  TEST_ASSERT_EQUAL_UINT8(12, pinsOf(DisplayType::SEG).count);
  TEST_ASSERT_EQUAL_HEX32(maskOf({ 2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16 }), pinMaskOf(DisplayType::SEG));

  TEST_ASSERT_EQUAL_UINT8(8, pinsOf(DisplayType::SEG_HW).count);
  TEST_ASSERT_EQUAL_HEX32(maskOf({ 5, 6, 7, 8, 9, 14, 15, 16 }), pinMaskOf(DisplayType::SEG_HW));

  TEST_ASSERT_EQUAL_UINT8(2, pinsOf(DisplayType::OLED).count);
  TEST_ASSERT_EQUAL_HEX32(maskOf({ 18, 19 }), pinMaskOf(DisplayType::OLED));

  // no pin twice in a list: as many bits as pins
  for (const auto type : { DisplayType::SEG, DisplayType::SEG_HW, DisplayType::OLED })
  {
    TEST_ASSERT_EQUAL_UINT8(pinsOf(type).count, __builtin_popcountl(pinMaskOf(type)));
  }
}

/** A pin's bit in its port. */
void test_image_of_a_pin()
{
  assertImage({ 0, 0, 0x04 }, imageOf(static_cast< uint8_t >(2)), "D2: PD2");
  assertImage({ 0x02, 0, 0 }, imageOf(static_cast< uint8_t >(9)), "D9: PB1");
  assertImage({ 0x20, 0, 0 }, imageOf(static_cast< uint8_t >(13)), "D13: PB5");
  assertImage({ 0, 0x01, 0 }, imageOf(static_cast< uint8_t >(14)), "D14 (A0): PC0");
  assertImage({ 0, 0x04, 0 }, imageOf(static_cast< uint8_t >(16)), "D16 (A2): PC2");
}

/** SEG: segments a to g on D2, D5, D12, D6, D7, D9, D8, the DP on D14, digits on D16, D10, D13, D11. */
void test_seg_images()
{
  assertImage({ 0x3F, 0x05, 0xE4 }, pinsImageOf(DisplayType::SEG), "every pin");

  // '8': a to g = D2, D5, D12, D6, D7, D9, D8
  assertImage({ 0x13, 0, 0xE4 }, charImageOf(DisplayType::SEG, 8), "'8'");
  // '1': b, c = D5, D12
  assertImage({ 0x10, 0, 0x20 }, charImageOf(DisplayType::SEG, 1), "'1'");
  // '-': g = D8
  assertImage({ 0x01, 0, 0 }, charImageOf(DisplayType::SEG, CHAR_MINUS), "'-'");
  assertImage({}, charImageOf(DisplayType::SEG, CHAR_BLANK), "' '");
  assertImage({}, charImageOf(DisplayType::SEG, 99), "out of range: blank");

  assertImage({ 0, 0x01, 0 }, dpImageOf(DisplayType::SEG), "DP: D14");

  // digit lines: HIGH (off) but the one on
  assertImage({ 0x2C, 0, 0 }, digitImageOf(DisplayType::SEG, 0), "digit 0 on: D16 LOW");
  assertImage({ 0x28, 0x04, 0 }, digitImageOf(DisplayType::SEG, 1), "digit 1 on: D10 LOW");
  assertImage({ 0x0C, 0x04, 0 }, digitImageOf(DisplayType::SEG, 2), "digit 2 on: D13 LOW");
  assertImage({ 0x24, 0x04, 0 }, digitImageOf(DisplayType::SEG, 3), "digit 3 on: D11 LOW");

  assertImage({ 0x2C, 0x04, 0 }, blankHighOf(DisplayType::SEG), "blank: every digit off");
  assertImage({}, blankLowOf(DisplayType::SEG), "blank: nothing LOW");

  // '1' with its DP at digit 2
  assertImage({ 0x1C, 0x05, 0x20 }, imageOf(DisplayType::SEG, static_cast< uint8_t >(1 | DP), 2), "'1.' at digit 2");
}

/** SEG_HW: BCD on D7, D9, D8, D6, the DP on D14, the 74HC138 on D16, D15, the enable on D5. */
void test_seg_hw_images()
{
  assertImage({ 0x03, 0x07, 0xE0 }, pinsImageOf(DisplayType::SEG_HW), "every pin");

  assertImage({}, charImageOf(DisplayType::SEG_HW, 0), "'0': 0000");
  assertImage({ 0, 0, 0xC0 }, charImageOf(DisplayType::SEG_HW, 9), "'9': 1001 = D7, D6");
  assertImage({ 0x03, 0, 0xC0 }, charImageOf(DisplayType::SEG_HW, CHAR_BLANK), "' ': 1111");
  assertImage({ 0x03, 0, 0xC0 }, charImageOf(DisplayType::SEG_HW, CHAR_MINUS), "'-': a blank");

  assertImage({ 0, 0x01, 0 }, dpImageOf(DisplayType::SEG_HW), "DP: D14");

  assertImage({}, digitImageOf(DisplayType::SEG_HW, 0), "digit 0: 00, enable LOW");
  assertImage({ 0, 0x02, 0 }, digitImageOf(DisplayType::SEG_HW, 1), "digit 1: D15");
  assertImage({ 0, 0x04, 0 }, digitImageOf(DisplayType::SEG_HW, 2), "digit 2: D16");
  assertImage({ 0, 0x06, 0 }, digitImageOf(DisplayType::SEG_HW, 3), "digit 3: D16, D15");

  assertImage({ 0, 0, 0x20 }, blankHighOf(DisplayType::SEG_HW), "blank: the 74HC4543 off");
  assertImage({ 0, 0x01, 0 }, blankLowOf(DisplayType::SEG_HW), "blank: the DP off");
}

/** No image sets a bit outside the pins of its display: the interrupt's masked writes keep the other pins. */
void test_images_stay_within_the_pins()
{
  for (const auto type : { DisplayType::SEG, DisplayType::SEG_HW })
  {
    const PortImage pins{ pinsImageOf(type) };
    for (uint8_t code = 0; code != NO_OF_CHARS; ++code)
    {
      for (uint8_t digit = 0; digit != NO_OF_DIGITS; ++digit)
      {
        const PortImage image{ imageOf(type, static_cast< uint8_t >(code | DP), digit) };
        TEST_ASSERT_EQUAL_HEX8(0, image.b & ~pins.b);
        TEST_ASSERT_EQUAL_HEX8(0, image.c & ~pins.c);
        TEST_ASSERT_EQUAL_HEX8(0, image.d & ~pins.d);
      }
    }
    const PortImage high{ blankHighOf(type) };
    const PortImage low{ blankLowOf(type) };
    TEST_ASSERT_EQUAL_HEX8(0, (high.b | low.b) & ~pins.b);
    TEST_ASSERT_EQUAL_HEX8(0, (high.c | low.c) & ~pins.c);
    TEST_ASSERT_EQUAL_HEX8(0, (high.d | low.d) & ~pins.d);
  }
}

/** The energy slot: FORC first, then OFF, then the energy or the walking dots; nothing without CT2. */
void test_energy_slot()
{
  Content content{ Content::TEMPERATURE };

  TEST_ASSERT_TRUE(energySlot(true, false, true, true, true, content));
  TEST_ASSERT_EQUAL_UINT8(Content::FORCED, content);

  TEST_ASSERT_TRUE(energySlot(false, false, true, true, true, content));
  TEST_ASSERT_EQUAL_UINT8(Content::OFF, content);

  TEST_ASSERT_TRUE(energySlot(false, true, true, true, true, content));
  TEST_ASSERT_EQUAL_UINT8(Content::ENERGY, content);

  TEST_ASSERT_TRUE(energySlot(false, true, true, false, true, content));
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(Content::WAITING, content, "7-segment: walking dots before any diversion");

  TEST_ASSERT_TRUE(energySlot(false, true, true, false, false, content));
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(Content::ENERGY, content, "OLED: 0.000 kWh before any diversion");

  content = Content::TEMPERATURE;
  TEST_ASSERT_FALSE_MESSAGE(energySlot(false, true, false, false, true, content), "no CT2: no energy slot");
  TEST_ASSERT_EQUAL_UINT8(Content::TEMPERATURE, content);

  TEST_ASSERT_TRUE_MESSAGE(energySlot(true, true, false, false, true, content), "no CT2: FORC still shows");
  TEST_ASSERT_EQUAL_UINT8(Content::FORCED, content);
  TEST_ASSERT_TRUE_MESSAGE(energySlot(false, false, false, false, true, content), "no CT2: OFF still shows");
  TEST_ASSERT_EQUAL_UINT8(Content::OFF, content);
}

/** The turn: the energy slot, then each sensor; temperatures only without slot; WAITING with nothing. */
void test_turn()
{
  // slot and 2 sensors
  Item item{ itemAt(0, true, Content::ENERGY, 2) };
  TEST_ASSERT_EQUAL_UINT8(Content::ENERGY, item.content);
  item = itemAt(1, true, Content::ENERGY, 2);
  TEST_ASSERT_EQUAL_UINT8(Content::TEMPERATURE, item.content);
  TEST_ASSERT_EQUAL_UINT8(0, item.sensor);
  item = itemAt(2, true, Content::ENERGY, 2);
  TEST_ASSERT_EQUAL_UINT8(Content::TEMPERATURE, item.content);
  TEST_ASSERT_EQUAL_UINT8(1, item.sensor);
  item = itemAt(3, true, Content::ENERGY, 2);
  TEST_ASSERT_EQUAL_UINT8_MESSAGE(Content::ENERGY, item.content, "back to the start");

  // no sensor: the slot all the time
  for (uint8_t step = 0; step != 5; ++step)
  {
    TEST_ASSERT_EQUAL_UINT8(Content::FORCED, itemAt(step, true, Content::FORCED, 0).content);
  }

  // no slot (no CT2): the temperatures only
  item = itemAt(0, false, Content::WAITING, 2);
  TEST_ASSERT_EQUAL_UINT8(Content::TEMPERATURE, item.content);
  TEST_ASSERT_EQUAL_UINT8(0, item.sensor);
  item = itemAt(1, false, Content::WAITING, 2);
  TEST_ASSERT_EQUAL_UINT8(1, item.sensor);

  // nothing to show
  TEST_ASSERT_EQUAL_UINT8(Content::WAITING, itemAt(7, false, Content::WAITING, 0).content);

  // a step beyond the items (the slot just went away) still gives an item
  item = itemAt(2, false, Content::WAITING, 1);
  TEST_ASSERT_EQUAL_UINT8(Content::TEMPERATURE, item.content);
  TEST_ASSERT_EQUAL_UINT8(0, item.sensor);
}

/** The energy on 4 digits, as Full: "1.234" up to 9999 Wh, "12.34" above. */
void test_seg_energy()
{
  uint8_t chars[NO_OF_DIGITS]{};

  segEnergy(0, chars);
  assertChars(0 | DP, 0, 0, 0, chars, "0 Wh");
  segEnergy(1234, chars);
  assertChars(1 | DP, 2, 3, 4, chars, "1234 Wh");
  segEnergy(9999, chars);
  assertChars(9 | DP, 9, 9, 9, chars, "9999 Wh");
  segEnergy(10000, chars);
  assertChars(1, 0 | DP, 0, 0, chars, "10000 Wh");
  segEnergy(65535, chars);
  assertChars(6, 5 | DP, 5, 3, chars, "65535 Wh");
}

/** FORC, OFF and the walking dots, as Full. */
void test_seg_texts()
{
  uint8_t chars[NO_OF_DIGITS]{};

  segForced(chars);
  assertChars(CHAR_F, 0, CHAR_R, CHAR_C, chars, "FORC");
  segOff(chars);
  assertChars(CHAR_BLANK, 0, CHAR_F, CHAR_F, chars, " OFF");
  segWalkingDots(2, chars);
  assertChars(CHAR_BLANK, CHAR_BLANK, CHAR_BLANK | DP, CHAR_BLANK, chars, "dot on digit 2");
  segWalkingDots(4, chars);
  assertChars(CHAR_BLANK | DP, CHAR_BLANK, CHAR_BLANK, CHAR_BLANK, chars, "position 4 is 0");
}

/** The temperature on 4 digits, rounded to 0.1 °C; 'C' and '-' blank on SEG_HW. */
void test_seg_temperature()
{
  uint8_t chars[NO_OF_DIGITS]{};
  constexpr uint8_t B{ CHAR_BLANK };

  segTemperature(2350, true, chars);
  assertChars(2, 3 | DP, 5, CHAR_C, chars, "23.50");
  segTemperature(2349, true, chars);
  assertChars(2, 3 | DP, 5, CHAR_C, chars, "23.49 rounds to 23.5");
  segTemperature(520, true, chars);
  assertChars(B, 5 | DP, 2, CHAR_C, chars, "5.20");
  segTemperature(4, true, chars);
  assertChars(B, 0 | DP, 0, CHAR_C, chars, "0.04");
  segTemperature(-4, true, chars);
  assertChars(B, 0 | DP, 0, CHAR_C, chars, "-0.04: no -0.0");
  segTemperature(-520, true, chars);
  assertChars(CHAR_MINUS, 5 | DP, 2, CHAR_C, chars, "-5.20");
  segTemperature(-1250, true, chars);
  assertChars(CHAR_MINUS, 1, 3, CHAR_C, chars, "-12.50: whole degrees");
  segTemperature(9994, true, chars);
  assertChars(9, 9 | DP, 9, CHAR_C, chars, "99.94");
  segTemperature(9995, true, chars);
  assertChars(1, 0, 0, CHAR_C, chars, "99.95 rounds to 100");
  segTemperature(12500, true, chars);
  assertChars(1, 2, 5, CHAR_C, chars, "125.00: whole degrees");

  segTemperature(DEVICE_DISCONNECTED_RAW, true, chars);
  assertChars(CHAR_MINUS, CHAR_MINUS, CHAR_MINUS, CHAR_MINUS, chars, "invalid");
  segTemperature(OUTOFRANGE_TEMPERATURE, true, chars);
  assertChars(CHAR_MINUS, CHAR_MINUS, CHAR_MINUS, CHAR_MINUS, chars, "out of range");

  segTemperature(2350, false, chars);
  assertChars(2, 3 | DP, 5, B, chars, "SEG_HW: no 'C'");
  segTemperature(-520, false, chars);
  assertChars(B, 5 | DP, 2, B, chars, "SEG_HW: no '-'");
  segTemperature(DEVICE_DISCONNECTED_RAW, false, chars);
  assertChars(B, B, B, B, chars, "SEG_HW: invalid is blank");
}

/** The large digits of the OLED: 5 characters, the width of the screen. */
void test_oled_texts()
{
  char text[OLED_TEXT_SIZE]{};

  oledEnergy(0, text);
  TEST_ASSERT_EQUAL_STRING("0.000", text);
  oledEnergy(1234, text);
  TEST_ASSERT_EQUAL_STRING("1.234", text);
  oledEnergy(12345, text);
  TEST_ASSERT_EQUAL_STRING("12.34", text);
  oledEnergy(65535, text);
  TEST_ASSERT_EQUAL_STRING("65.53", text);

  oledTemperature(2350, text);
  TEST_ASSERT_EQUAL_STRING(" 23.5", text);
  oledTemperature(520, text);
  TEST_ASSERT_EQUAL_STRING("  5.2", text);
  oledTemperature(-520, text);
  TEST_ASSERT_EQUAL_STRING(" -5.2", text);
  oledTemperature(-4, text);
  TEST_ASSERT_EQUAL_STRING("  0.0", text);
  oledTemperature(-1250, text);
  TEST_ASSERT_EQUAL_STRING("-12.5", text);
  oledTemperature(-5500, text);
  TEST_ASSERT_EQUAL_STRING("-55.0", text);
  oledTemperature(10000, text);
  TEST_ASSERT_EQUAL_STRING("100.0", text);
  oledTemperature(12500, text);
  TEST_ASSERT_EQUAL_STRING("125.0", text);
  oledTemperature(DEVICE_DISCONNECTED_RAW, text);
  TEST_ASSERT_EQUAL_STRING(" --.-", text);
  oledTemperature(OUTOFRANGE_TEMPERATURE, text);
  TEST_ASSERT_EQUAL_STRING(" --.-", text);
  oledTemperature(INT16_MAX, text);
  TEST_ASSERT_EQUAL_UINT(5, strlen(text));
}

int main(int /* argc */, char** /* argv */)
{
  UNITY_BEGIN();

  RUN_TEST(test_pins_of_each_type);
  RUN_TEST(test_image_of_a_pin);
  RUN_TEST(test_seg_images);
  RUN_TEST(test_seg_hw_images);
  RUN_TEST(test_images_stay_within_the_pins);
  RUN_TEST(test_energy_slot);
  RUN_TEST(test_turn);
  RUN_TEST(test_seg_energy);
  RUN_TEST(test_seg_texts);
  RUN_TEST(test_seg_temperature);
  RUN_TEST(test_oled_texts);

  return UNITY_END();
}
