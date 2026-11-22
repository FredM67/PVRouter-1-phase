/**
 * @file display_core.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief The display, without Arduino: its pins, what it shows when, and the text it shows
 * @version 1.0
 * @date 2026-11-22
 *
 * @copyright Copyright (c) 2024-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * @details From Mk2_fasterControl_Full (utils_display.h, utils_oled.h). Nothing here depends on
 *          a setting or on Arduino, so the native tests build it on the PC. utils_display.h
 *          drives the display with it.
 *          - The pins of each display type, as a list and as a mask, for the pin checks.
 *          - The port image of each character of the 7-segment display: the loop turns the
 *            4 characters into 4 images, and the ADC interrupt only writes one of them to the
 *            ports every few ms (no table look-up, no per-pin write in the interrupt).
 *          - What is shown in turn: the energy slot (FORC, OFF, the diverted energy, or the
 *            walking dots while nothing was diverted since the last reset), then each
 *            temperature.
 *          - The characters of the 7-segment display and the text of the OLED for a value.
 *
 * @ingroup 7SegDisplay
 */

#ifndef DISPLAY_CORE_H
#define DISPLAY_CORE_H

#include <stdint.h>

#include "config_types.h"  // DisplayType
#include "ds18b20.h"       // isValidTemperature()

namespace Display
{
//--------------------------------------------------------------------------------------------------
// Pins
//
inline constexpr uint8_t NO_OF_DIGITS{ 4 }; /**< digits of the 7-segment display */

// SEG_HW: the 7 segments through a 74HC4543 (BCD to 7 segments), the digit through a 74HC138
inline constexpr uint8_t enableDisableLine{ 5 };              /**< 74HC4543 enable, LOW: on (not the DP) */
inline constexpr uint8_t decimalPointLine{ 14 };              /**< the DP, driven on its own */
inline constexpr uint8_t digitLocationLine[2]{ 16, 15 };      /**< 74HC138: which digit */
inline constexpr uint8_t digitSelectionLine[4]{ 7, 9, 8, 6 }; /**< 74HC4543: BCD of the character */

// SEG: every segment and every digit on its own pin
inline constexpr uint8_t digitSelectorPin[NO_OF_DIGITS]{ 16, 10, 13, 11 }; /**< LOW: digit on */
inline constexpr uint8_t segmentDrivePin[8]{ 2, 5, 12, 6, 7, 9, 8, 14 };   /**< a to g, then the DP; HIGH: on */

// OLED: the I2C pins of the ATmega328P
inline constexpr uint8_t OLED_SDA_PIN{ 18 }; /**< A4 */
inline constexpr uint8_t OLED_SCL_PIN{ 19 }; /**< A5 */

inline constexpr uint8_t MAX_PINS{ 12 }; /**< the most pins a display takes (SEG) */

/** @brief The pins a display takes, D0 = 0 ... A5 = 19. */
struct PinList
{
  uint8_t count{ 0 };      /**< how many */
  uint8_t pin[MAX_PINS]{}; /**< the first 'count' are the pins */

  /** @brief Adds the pins of @p list. */
  template< uint8_t N >
  constexpr void add(const uint8_t (&list)[N])
  {
    for (uint8_t i = 0; i != N; ++i)
    {
      pin[count++] = list[i];
    }
  }
};

/**
 * @brief The pins display type @p type takes, for the pin checks.
 */
constexpr PinList pinsOf(const DisplayType type)
{
  PinList pins{};
  if (type == DisplayType::SEG)
  {
    pins.add(digitSelectorPin);
    pins.add(segmentDrivePin);
  }
  else if (type == DisplayType::SEG_HW)
  {
    pins.add(digitLocationLine);
    pins.add(digitSelectionLine);
    const uint8_t others[]{ enableDisableLine, decimalPointLine };
    pins.add(others);
  }
  else if (type == DisplayType::OLED)
  {
    const uint8_t i2c[]{ OLED_SDA_PIN, OLED_SCL_PIN };
    pins.add(i2c);
  }
  return pins;
}

/**
 * @brief The pins display type @p type takes as a mask: bit n is pin n (D0 = bit 0 ... A5 = bit 19).
 * @note 32 bits: the 7-segment display uses D16 (A2) and the OLED A4/A5, beyond a 16-bit mask.
 */
constexpr uint32_t pinMaskOf(const DisplayType type)
{
  const PinList pins{ pinsOf(type) };
  uint32_t mask{ 0 };
  for (uint8_t i = 0; i != pins.count; ++i)
  {
    mask |= 1UL << pins.pin[i];
  }
  return mask;
}

//--------------------------------------------------------------------------------------------------
// Characters of the 7-segment display
//
inline constexpr uint8_t CHAR_BLANK{ 10 }; /**< ' ' */
inline constexpr uint8_t CHAR_F{ 11 };     /**< 'F' (SEG only) */
inline constexpr uint8_t CHAR_R{ 12 };     /**< 'r' (SEG only) */
inline constexpr uint8_t CHAR_C{ 13 };     /**< 'C' (SEG only) */
inline constexpr uint8_t CHAR_MINUS{ 14 }; /**< '-' (SEG only; blank on SEG_HW) */
inline constexpr uint8_t NO_OF_CHARS{ 15 };
inline constexpr uint8_t DP{ 0x80 }; /**< OR-ed to a character: its decimal point on */

/**
 * @brief SEG: the segments a to g of each character, 1 = on (Full's segMap, plus '-').
 */
inline constexpr uint8_t segMap[NO_OF_CHARS][7]{
  { 1, 1, 1, 1, 1, 1, 0 },  // '0'
  { 0, 1, 1, 0, 0, 0, 0 },  // '1'
  { 1, 1, 0, 1, 1, 0, 1 },  // '2'
  { 1, 1, 1, 1, 0, 0, 1 },  // '3'
  { 0, 1, 1, 0, 0, 1, 1 },  // '4'
  { 1, 0, 1, 1, 0, 1, 1 },  // '5'
  { 1, 0, 1, 1, 1, 1, 1 },  // '6'
  { 1, 1, 1, 0, 0, 0, 0 },  // '7'
  { 1, 1, 1, 1, 1, 1, 1 },  // '8'
  { 1, 1, 1, 1, 0, 1, 1 },  // '9'
  { 0, 0, 0, 0, 0, 0, 0 },  // ' '
  { 1, 0, 0, 0, 1, 1, 1 },  // 'F'
  { 0, 0, 0, 0, 1, 0, 1 },  // 'r'
  { 1, 0, 0, 1, 1, 1, 0 },  // 'C'
  { 0, 0, 0, 0, 0, 0, 1 },  // '-'
};

/**
 * @brief SEG_HW: the BCD lines of the 74HC4543 for each character, 1 = HIGH (Full's
 *        digitValueMap, plus '-'). The chip shows 0 to 9 and a blank (10 to 15) only: Full's
 *        'F', 'r' and 'C' show as 7, blank and 2, and '-' is a blank.
 */
inline constexpr uint8_t digitValueMap[NO_OF_CHARS][4]{
  { 0, 0, 0, 0 },  // '0'
  { 0, 0, 0, 1 },  // '1'
  { 0, 0, 1, 0 },  // '2'
  { 0, 0, 1, 1 },  // '3'
  { 0, 1, 0, 0 },  // '4'
  { 0, 1, 0, 1 },  // '5'
  { 0, 1, 1, 0 },  // '6'
  { 0, 1, 1, 1 },  // '7'
  { 1, 0, 0, 0 },  // '8'
  { 1, 0, 0, 1 },  // '9'
  { 1, 1, 1, 1 },  // ' '
  { 0, 1, 1, 1 },  // 'F'
  { 1, 0, 1, 0 },  // 'r'
  { 0, 0, 1, 0 },  // 'C'
  { 1, 1, 1, 1 },  // '-'
};

/** SEG_HW: the 74HC138 lines of each digit, 1 = HIGH */
inline constexpr uint8_t digitLocationMap[NO_OF_DIGITS][2]{ { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 } };

/** @brief The bits of pins in the ports B, C and D of the ATmega328P. */
struct PortImage
{
  uint8_t b{ 0 }; /**< PORTB: D8 to D13 */
  uint8_t c{ 0 }; /**< PORTC: D14 (A0) to D19 (A5) */
  uint8_t d{ 0 }; /**< PORTD: D0 to D7 */

  constexpr PortImage& operator|=(const PortImage& other)
  {
    b |= other.b;
    c |= other.c;
    d |= other.d;
    return *this;
  }
};

/** @brief The bit of pin @p pin in its port. */
constexpr PortImage imageOf(const uint8_t pin)
{
  PortImage image{};
  if (pin < 8)
  {
    image.d = static_cast< uint8_t >(1U << pin);
  }
  else if (pin < 14)
  {
    image.b = static_cast< uint8_t >(1U << (pin - 8));
  }
  else
  {
    image.c = static_cast< uint8_t >(1U << (pin - 14));
  }
  return image;
}

/** @brief The bits of every pin of display type @p type: what the interrupt writes. */
constexpr PortImage pinsImageOf(const DisplayType type)
{
  const PinList pins{ pinsOf(type) };
  PortImage image{};
  for (uint8_t i = 0; i != pins.count; ++i)
  {
    image |= imageOf(pins.pin[i]);
  }
  return image;
}

/**
 * @brief The pins to set HIGH before a digit changes, so that nothing shows while the ports
 *        change: SEG, every digit off; SEG_HW, the 74HC4543 off.
 */
constexpr PortImage blankHighOf(const DisplayType type)
{
  PortImage image{};
  if (type == DisplayType::SEG)
  {
    for (const auto pin : digitSelectorPin)
    {
      image |= imageOf(pin);
    }
  }
  else if (type == DisplayType::SEG_HW)
  {
    image = imageOf(enableDisableLine);
  }
  return image;
}

/**
 * @brief The pins to set LOW before a digit changes: SEG_HW, the DP (the 74HC4543 does not
 *        drive it). Nothing with SEG: the digit lines cut the DP too.
 */
constexpr PortImage blankLowOf(const DisplayType type)
{
  return type == DisplayType::SEG_HW ? imageOf(decimalPointLine) : PortImage{};
}

/**
 * @brief The image of character @p character alone (no DP, no digit line): SEG, its
 *        segments; SEG_HW, its BCD lines.
 */
constexpr PortImage charImageOf(const DisplayType type, const uint8_t character)
{
  PortImage image{};
  const uint8_t c{ character < NO_OF_CHARS ? character : CHAR_BLANK };

  if (type == DisplayType::SEG)
  {
    for (uint8_t segment = 0; segment != 7; ++segment)
    {
      if (segMap[c][segment])
      {
        image |= imageOf(segmentDrivePin[segment]);
      }
    }
  }
  else if (type == DisplayType::SEG_HW)
  {
    for (uint8_t line = 0; line != 4; ++line)
    {
      if (digitValueMap[c][line])
      {
        image |= imageOf(digitSelectionLine[line]);
      }
    }
  }
  return image;
}

/** @brief The image of the DP on: SEG, its segment; SEG_HW, its line. */
constexpr PortImage dpImageOf(const DisplayType type)
{
  if (type == DisplayType::SEG)
  {
    return imageOf(segmentDrivePin[7]);
  }
  return type == DisplayType::SEG_HW ? imageOf(decimalPointLine) : PortImage{};
}

/**
 * @brief The image of the digit lines with digit @p digit on: SEG, every digit line HIGH but
 *        that of @p digit; SEG_HW, the 74HC138 lines of @p digit, and the enable line LOW
 *        (the 74HC4543 on).
 */
constexpr PortImage digitImageOf(const DisplayType type, const uint8_t digit)
{
  PortImage image{};
  if (type == DisplayType::SEG)
  {
    for (uint8_t i = 0; i != NO_OF_DIGITS; ++i)
    {
      if (i != digit)
      {
        image |= imageOf(digitSelectorPin[i]);
      }
    }
  }
  else if (type == DisplayType::SEG_HW)
  {
    for (uint8_t line = 0; line != 2; ++line)
    {
      if (digitLocationMap[digit & 3][line])
      {
        image |= imageOf(digitLocationLine[line]);
      }
    }
  }
  return image;
}

/**
 * @brief The image of character @p code (with its DP bit) at digit @p digit, the digit on:
 *        what the interrupt writes over pinsImageOf().
 */
constexpr PortImage imageOf(const DisplayType type, const uint8_t code, const uint8_t digit)
{
  PortImage image{ charImageOf(type, static_cast< uint8_t >(code & 0x7F)) };
  if (code & DP)
  {
    image |= dpImageOf(type);
  }
  image |= digitImageOf(type, digit);
  return image;
}

//--------------------------------------------------------------------------------------------------
// What is shown
//
/** @brief What the display shows. */
enum class Content : uint8_t
{
  WAITING,     /**< nothing to show: the walking dots (7-segment), a blank (OLED) */
  FORCED,      /**< a load or relay is forced: "FORC" */
  OFF,         /**< router or diversion OFF: " OFF" */
  ENERGY,      /**< the diverted energy */
  TEMPERATURE, /**< a temperature */
};

/** @brief One step of the turn: what, and which sensor for a temperature. */
struct Item
{
  Content content; /**< what */
  uint8_t sensor;  /**< the sensor, for TEMPERATURE */
};

/**
 * @brief The energy slot: what comes first in the turn, or WAITING for no slot.
 *
 * @param forced a load or relay is forced
 * @param diversionEnabled the router and the diversion are ON
 * @param divertedCt a CT2 is fitted (DIVERTED_CT_PRESENT)
 * @param energyActive some energy was diverted since the last reset (EDD_isActive)
 * @param walkingDots WAITING as an item (7-segment: the dots while nothing was diverted)
 *
 * @details As Full: FORC first, then OFF, then the energy, or the walking dots before any
 *          diversion. Without CT2 there is no energy slot, but FORC and OFF still show.
 */
constexpr bool energySlot(const bool forced, const bool diversionEnabled, const bool divertedCt, const bool energyActive,
                          const bool walkingDots, Content& content)
{
  if (forced)
  {
    content = Content::FORCED;
  }
  else if (!diversionEnabled)
  {
    content = Content::OFF;
  }
  else if (!divertedCt)
  {
    return false;
  }
  else
  {
    content = (energyActive || !walkingDots) ? Content::ENERGY : Content::WAITING;
  }
  return true;
}

/**
 * @brief What step @p step of the turn shows: the energy slot, if any, then each sensor.
 *
 * @param step counts the turns; any value (taken modulo the number of items)
 * @param hasSlot whether there is an energy slot (energySlot())
 * @param slot what the energy slot shows
 * @param noOfSensors the temperature sensors
 * @return WAITING if there is nothing to show.
 */
constexpr Item itemAt(const uint8_t step, const bool hasSlot, const Content slot, const uint8_t noOfSensors)
{
  const uint8_t noOfItems{ static_cast< uint8_t >(noOfSensors + (hasSlot ? 1 : 0)) };
  if (noOfItems == 0)
  {
    return { Content::WAITING, 0 };
  }
  uint8_t index{ static_cast< uint8_t >(step % noOfItems) };
  if (hasSlot)
  {
    if (index == 0)
    {
      return { slot, 0 };
    }
    --index;
  }
  return { Content::TEMPERATURE, index };
}

//--------------------------------------------------------------------------------------------------
// 7-segment display: the 4 characters (left to right) for a value
//
/** @brief Writes "FORC". */
constexpr void segForced(uint8_t (&chars)[NO_OF_DIGITS])
{
  chars[0] = CHAR_F;
  chars[1] = 0;  // 'O' as '0'
  chars[2] = CHAR_R;
  chars[3] = CHAR_C;
}

/** @brief Writes " OFF". */
constexpr void segOff(uint8_t (&chars)[NO_OF_DIGITS])
{
  chars[0] = CHAR_BLANK;
  chars[1] = 0;  // 'O' as '0'
  chars[2] = CHAR_F;
  chars[3] = CHAR_F;
}

/** @brief Writes the walking dots: 4 blanks, the DP of digit @p position (0 to 3) on. */
constexpr void segWalkingDots(const uint8_t position, uint8_t (&chars)[NO_OF_DIGITS])
{
  for (auto& c : chars)
  {
    c = CHAR_BLANK;
  }
  chars[position & 3] |= DP;
}

/**
 * @brief Writes the energy @p wh in kWh: "1.234" up to 9999 Wh, "12.34" above (as Full).
 */
constexpr void segEnergy(const uint16_t wh, uint8_t (&chars)[NO_OF_DIGITS])
{
  const bool above10kWh{ wh > 9999U };
  uint16_t value{ above10kWh ? static_cast< uint16_t >(wh / 10U) : wh };

  for (uint8_t i = NO_OF_DIGITS; i != 0; --i)
  {
    chars[i - 1] = static_cast< uint8_t >(value % 10U);
    value /= 10U;
  }
  chars[above10kWh ? 1 : 0] |= DP;
}

/**
 * @brief Writes the temperature @p temperature_x100, rounded to 0.1 °C.
 *
 * @param letters SEG: 'C' and '-' can be shown; SEG_HW: they cannot (the 74HC4543 shows 0 to 9
 *                and a blank only)
 *
 * @details "23.5C", "-5.2C" (the DP after the units); from 100 °C and below -9.9 °C, whole
 *          degrees: "100C", "-13C". An invalid reading shows "----". On SEG_HW, no 'C', and a
 *          negative temperature lights the DP of digit 0 instead of the '-' (that DP is free in
 *          every negative layout); an invalid reading is 4 blanks.
 */
constexpr void segTemperature(const int16_t temperature_x100, const bool letters, uint8_t (&chars)[NO_OF_DIGITS])
{
  const uint8_t unit{ letters ? CHAR_C : CHAR_BLANK };
  const uint8_t minus{ letters ? CHAR_MINUS : static_cast< uint8_t >(CHAR_BLANK | DP) };

  if (!isValidTemperature(temperature_x100))
  {
    for (auto& c : chars)
    {
      c = letters ? CHAR_MINUS : CHAR_BLANK;
    }
    return;
  }

  const uint16_t magnitude{ static_cast< uint16_t >(temperature_x100 < 0 ? -static_cast< int32_t >(temperature_x100) : temperature_x100) };
  const uint16_t tenths{ static_cast< uint16_t >((magnitude + 5U) / 10U) };  // 0.1 °C, rounded
  const bool negative{ (temperature_x100 < 0) && (tenths != 0) };            // no "-0.0"

  chars[3] = unit;
  if (tenths < 100U)
  {
    // -9.9 to 9.9 °C: [sign or blank][units.][tenths]
    chars[2] = static_cast< uint8_t >(tenths % 10U);
    chars[1] = static_cast< uint8_t >(((tenths / 10U) % 10U) | DP);
    chars[0] = negative ? minus : CHAR_BLANK;
  }
  else if (!negative)
  {
    // [tens][units.][tenths] up to 99.9 °C, then [hundreds][tens][units]
    if (tenths < 1000U)
    {
      chars[2] = static_cast< uint8_t >(tenths % 10U);
      chars[1] = static_cast< uint8_t >(((tenths / 10U) % 10U) | DP);
      chars[0] = static_cast< uint8_t >(tenths / 100U);
    }
    else
    {
      const uint16_t degrees{ static_cast< uint16_t >((tenths + 5U) / 10U) };
      chars[2] = static_cast< uint8_t >(degrees % 10U);
      chars[1] = static_cast< uint8_t >((degrees / 10U) % 10U);
      chars[0] = static_cast< uint8_t >((degrees / 100U) % 10U);
    }
  }
  else
  {
    // below -9.9 °C: [-][tens][units]
    const uint16_t degrees{ static_cast< uint16_t >((tenths + 5U) / 10U) };
    chars[2] = static_cast< uint8_t >(degrees % 10U);
    chars[1] = static_cast< uint8_t >((degrees / 10U) % 10U);
    chars[0] = minus;
  }
}

//--------------------------------------------------------------------------------------------------
// OLED: the text of a value, for the large digits (5 characters, the width of the screen)
//
inline constexpr uint8_t OLED_TEXT_SIZE{ 6 }; /**< 5 characters and the terminating 0 */

/** @brief Writes the energy @p wh in kWh: "1.234" up to 9999 Wh, "12.34" above. */
constexpr void oledEnergy(const uint16_t wh, char (&text)[OLED_TEXT_SIZE])
{
  uint8_t chars[NO_OF_DIGITS]{};
  segEnergy(wh, chars);

  uint8_t pos{ 0 };
  for (const auto c : chars)
  {
    text[pos++] = static_cast< char >('0' + (c & 0x7F));
    if (c & DP)
    {
      text[pos++] = '.';
    }
  }
  text[pos] = '\0';
}

/**
 * @brief Writes the temperature @p temperature_x100, rounded to 0.1 °C, right-aligned on
 *        5 characters: " 23.5", " -5.2", "100.0", "-12.5"; " --.-" for an invalid reading.
 */
constexpr void oledTemperature(const int16_t temperature_x100, char (&text)[OLED_TEXT_SIZE])
{
  for (auto& c : text)
  {
    c = ' ';
  }
  text[OLED_TEXT_SIZE - 1] = '\0';

  if (!isValidTemperature(temperature_x100))
  {
    text[1] = '-';
    text[2] = '-';
    text[3] = '.';
    text[4] = '-';
    return;
  }

  const uint16_t magnitude{ static_cast< uint16_t >(temperature_x100 < 0 ? -static_cast< int32_t >(temperature_x100) : temperature_x100) };
  uint16_t tenths{ static_cast< uint16_t >((magnitude + 5U) / 10U) };  // 0.1 °C, rounded
  const bool negative{ (temperature_x100 < 0) && (tenths != 0) };      // no "-0.0"
  if (tenths > 9999U)
  {
    tenths = 9999U;  // the readings stay far below (DS18B20: 125 °C)
  }

  int8_t pos{ OLED_TEXT_SIZE - 2 };
  text[pos--] = static_cast< char >('0' + tenths % 10U);
  text[pos--] = '.';
  tenths /= 10U;
  do
  {
    text[pos--] = static_cast< char >('0' + tenths % 10U);
    tenths /= 10U;
  } while ((tenths != 0) && (pos >= 0));

  if (negative && (pos >= 0))
  {
    text[pos] = '-';
  }
}
}  // namespace Display

#endif /* DISPLAY_CORE_H */
