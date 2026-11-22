/**
 * @file utils_display.h
 * @author Frederic Metrich (frederic.metrich@live.fr)
 * @brief 7-segment display functions
 * @version 1.0
 * @date 2026-11-22
 *
 * @copyright Copyright (c) 2024-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * @details From Mk2_fasterControl_Full/utils_display.h. The display is multiplexed from the ADC
 *          interrupt, as in Full: one digit after another, every MAX_DISPLAY_TIME_COUNT sample
 *          sets (10 x 312 us, so each digit for about 3 ms and the whole display 80 times a
 *          second). Unlike Full, the interrupt does not look up the characters nor write the
 *          pins one at a time: the loop turns the 4 characters into the port images of the
 *          4 digits (display_core.h), and the interrupt writes one image with a few masked
 *          port writes.
 *
 *          The ports are written with read-modify-write in the interrupt, which nothing
 *          interrupts. The loop never writes these ports that way (setPinState() and
 *          togglePin() go through PINx, digitalWrite() disables the interrupts), so no
 *          change is lost either way.
 *
 * @ingroup 7SegDisplay
 */

#ifndef UTILS_DISPLAY_H
#define UTILS_DISPLAY_H

#include <Arduino.h>
#include <util/atomic.h>

#include "config_derived.h"
#include "display_core.h"

inline constexpr bool SEVEN_SEGMENT_DISPLAY{ TYPE_OF_DISPLAY == DisplayType::SEG || TYPE_OF_DISPLAY == DisplayType::SEG_HW }; /**< a 7-segment display */

inline constexpr Display::PinList displayPins{ Display::pinsOf(TYPE_OF_DISPLAY) }; /**< the pins the display takes, for the pin checks */
inline constexpr uint32_t displayPinMask{ Display::pinMaskOf(TYPE_OF_DISPLAY) };   /**< the same as a mask, bit n = pin n (D16 = A2 ... A5 = bit 19) */

inline constexpr uint8_t MAX_DISPLAY_TIME_COUNT{ 10 }; /**< sample sets (of ~312 us) each digit is on */
inline constexpr uint8_t DISPLAY_TURN_SECONDS{ 5 };    /**< each item of the turn (energy, each temperature) shows this long */

namespace SevenSegment
{
inline constexpr Display::PortImage PINS{ Display::pinsImageOf(TYPE_OF_DISPLAY) };       /**< every pin of the display */
inline constexpr Display::PortImage BLANK_HIGH{ Display::blankHighOf(TYPE_OF_DISPLAY) }; /**< set HIGH before a digit changes */
inline constexpr Display::PortImage BLANK_LOW{ Display::blankLowOf(TYPE_OF_DISPLAY) };   /**< set LOW before a digit changes */

/** the image of each digit, written by the loop (interrupts off), read by the ADC interrupt */
inline Display::PortImage digitImages[Display::NO_OF_DIGITS]{};

/** @brief The image of each character, without the DP and the digit lines, in flash. */
struct CharImages
{
  Display::PortImage character[Display::NO_OF_CHARS]; /**< by character code */
  Display::PortImage digit[Display::NO_OF_DIGITS];    /**< the digit lines of each digit */
  Display::PortImage dp;                              /**< the DP */
};

/** @brief Builds CharImages from display_core.h, at compile time. */
constexpr CharImages makeCharImages()
{
  CharImages images{};
  for (uint8_t c = 0; c != Display::NO_OF_CHARS; ++c)
  {
    images.character[c] = Display::charImageOf(TYPE_OF_DISPLAY, c);
  }
  for (uint8_t i = 0; i != Display::NO_OF_DIGITS; ++i)
  {
    images.digit[i] = Display::digitImageOf(TYPE_OF_DISPLAY, i);
  }
  images.dp = Display::dpImageOf(TYPE_OF_DISPLAY);
  return images;
}
}  // namespace SevenSegment

/**
 * @brief Shows these 4 characters (left to right, DP bit included) from the next digit on.
 *
 * @details From the loop: the 4 images are built first, then copied with the interrupts off
 *          (a few us), so the interrupt never shows half an update.
 *
 * @ingroup 7SegDisplay
 */
inline void setDisplayCharacters(const uint8_t (&chars)[Display::NO_OF_DIGITS])
{
  if constexpr (SEVEN_SEGMENT_DISPLAY)
  {
    static constexpr SevenSegment::CharImages table PROGMEM{ SevenSegment::makeCharImages() };

    Display::PortImage images[Display::NO_OF_DIGITS];
    for (uint8_t i = 0; i != Display::NO_OF_DIGITS; ++i)
    {
      const uint8_t c{ static_cast< uint8_t >(chars[i] & 0x7F) };
      Display::PortImage part;
      memcpy_P(&images[i], &table.digit[i], sizeof(Display::PortImage));
      memcpy_P(&part, &table.character[c < Display::NO_OF_CHARS ? c : Display::CHAR_BLANK], sizeof(Display::PortImage));
      images[i] |= part;
      if (chars[i] & Display::DP)
      {
        memcpy_P(&part, &table.dp, sizeof(Display::PortImage));
        images[i] |= part;
      }
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      memcpy(SevenSegment::digitImages, images, sizeof(images));
    }
  }
}

/**
 * @brief Sets the pins of the 7-segment display as outputs, nothing shown.
 *
 * @ingroup 7SegDisplay
 */
inline void initializeDisplay()
{
  if constexpr (SEVEN_SEGMENT_DISPLAY)
  {
    using namespace SevenSegment;

    // every pin LOW but every digit off (SEG) or the 74HC4543 off (SEG_HW), then as outputs
    // (before the ADC interrupt starts: nothing else writes these ports yet)
    PORTB = static_cast< uint8_t >((PORTB & static_cast< uint8_t >(~PINS.b)) | BLANK_HIGH.b);
    PORTC = static_cast< uint8_t >((PORTC & static_cast< uint8_t >(~PINS.c)) | BLANK_HIGH.c);
    PORTD = static_cast< uint8_t >((PORTD & static_cast< uint8_t >(~PINS.d)) | BLANK_HIGH.d);
    DDRB |= PINS.b;
    DDRC |= PINS.c;
    DDRD |= PINS.d;

    const uint8_t blank[Display::NO_OF_DIGITS]{ Display::CHAR_BLANK, Display::CHAR_BLANK, Display::CHAR_BLANK, Display::CHAR_BLANK };
    setDisplayCharacters(blank);
  }
}

/**
 * @brief Shows item @p item of the turn on the 7-segment display, once per second.
 *
 * @param item what to show (Display::itemAt())
 * @param energyWh the diverted energy, in Wh
 * @param temperature_x100 the temperature of the item, for TEMPERATURE
 *
 * @details The walking dots move each second, as in Full.
 *
 * @ingroup 7SegDisplay
 */
inline void showOn7SegmentDisplay(const Display::Item& item, const uint16_t energyWh, const int16_t temperature_x100)
{
  if constexpr (SEVEN_SEGMENT_DISPLAY)
  {
    static uint8_t locationOfDot{ 0 };
    uint8_t chars[Display::NO_OF_DIGITS]{};

    switch (item.content)
    {
      case Display::Content::FORCED:
        Display::segForced(chars);
        break;
      case Display::Content::OFF:
        Display::segOff(chars);
        break;
      case Display::Content::ENERGY:
        Display::segEnergy(energyWh, chars);
        break;
      case Display::Content::TEMPERATURE:
        Display::segTemperature(temperature_x100, TYPE_OF_DISPLAY == DisplayType::SEG, chars);
        break;
      default:
        Display::segWalkingDots(locationOfDot, chars);
        locationOfDot = (locationOfDot + 1) & 3;
        break;
    }
    setDisplayCharacters(chars);
  }
}

#if !defined(__DOXYGEN__)
inline void refresh7SegDisplay() __attribute__((always_inline));
#endif

/**
 * @brief Multiplexes the 7-segment display: the next digit, every MAX_DISPLAY_TIME_COUNT calls.
 *
 * @details Called by the ADC interrupt once per sample set (with the CT1 sample), as in
 *          Full. Most calls only count. Every MAX_DISPLAY_TIME_COUNT-th call:
 *          1. nothing shows: every digit off (SEG), or the 74HC4543 off and the DP off (SEG_HW);
 *          2. the image of the next digit is written to ports B, C, then D. With SEG_HW, the
 *             enable line is on port D: the chip comes on once the BCD and digit lines are set.
 *             With SEG, the digit line comes on with its port, a few cycles before the segments
 *             of the ports written after it: about 0.2 us in 3 ms, nothing visible.
 *
 * @ingroup 7SegDisplay
 */
inline void refresh7SegDisplay()
{
  if constexpr (SEVEN_SEGMENT_DISPLAY)
  {
    using namespace SevenSegment;

    static uint8_t displayTime_count{ 0 };
    static uint8_t digitLocationThatIsActive{ 0 };

    if (++displayTime_count != MAX_DISPLAY_TIME_COUNT)
    {
      return;
    }
    displayTime_count = 0;

    digitLocationThatIsActive = (digitLocationThatIsActive + 1) & (Display::NO_OF_DIGITS - 1);
    const Display::PortImage& image{ digitImages[digitLocationThatIsActive] };

    // 1. nothing shows while the ports change
    if constexpr (BLANK_HIGH.b != 0)
    {
      PORTB |= BLANK_HIGH.b;
    }
    if constexpr (BLANK_HIGH.c != 0)
    {
      PORTC |= BLANK_HIGH.c;
    }
    if constexpr (BLANK_HIGH.d != 0)
    {
      PORTD |= BLANK_HIGH.d;
    }
    if constexpr (BLANK_LOW.b != 0)
    {
      PORTB &= static_cast< uint8_t >(~BLANK_LOW.b);
    }
    if constexpr (BLANK_LOW.c != 0)
    {
      PORTC &= static_cast< uint8_t >(~BLANK_LOW.c);
    }
    if constexpr (BLANK_LOW.d != 0)
    {
      PORTD &= static_cast< uint8_t >(~BLANK_LOW.d);
    }

    // 2. the next digit (SEG_HW: port D last, it holds the enable line)
    if constexpr (PINS.b != 0)
    {
      PORTB = static_cast< uint8_t >((PORTB & static_cast< uint8_t >(~PINS.b)) | image.b);
    }
    if constexpr (PINS.c != 0)
    {
      PORTC = static_cast< uint8_t >((PORTC & static_cast< uint8_t >(~PINS.c)) | image.c);
    }
    if constexpr (PINS.d != 0)
    {
      PORTD = static_cast< uint8_t >((PORTD & static_cast< uint8_t >(~PINS.d)) | image.d);
    }
  }
}

#endif /* UTILS_DISPLAY_H */
