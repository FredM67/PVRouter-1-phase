/**
 * @file config_types.h
 * @brief Struct definitions used by config.h for boost, diversion, and OLED configuration.
 *
 * These types are separated from config.h so the user configuration file
 * only contains values, not type definitions.
 */

#ifndef CONFIG_TYPES_H
#define CONFIG_TYPES_H

#include <Arduino.h>

struct OutputId
{
  uint8_t index;

  constexpr operator uint8_t() const { return index; }
  constexpr operator uint16_t() const { return static_cast< uint16_t >(1u << index); }
};

constexpr uint16_t operator|(const OutputId a, const OutputId b)
{
  return static_cast< uint16_t >(a) | static_cast< uint16_t >(b);
}

constexpr uint16_t operator|(const uint16_t a, const OutputId b)
{
  return a | static_cast< uint16_t >(b);
}

struct OledEncoderConfig
{
  uint8_t pinCLK;
  uint8_t pinDT;
  uint8_t pinSW;
};

struct BoostControlConfig
{
  uint8_t inputPin;
  uint8_t outputIndex;
  bool visibleOnOLED;
};

struct DiversionGroupConfig
{
  uint8_t inputPin;
  uint16_t outputMask;
  bool visibleOnOLED;
};

#endif /* CONFIG_TYPES_H */
