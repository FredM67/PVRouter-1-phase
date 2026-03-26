/**
 * @file router_settings.h
 * @brief Helpers for runtime configuration and dynamic routing commands.
 */

#ifndef ROUTER_SETTINGS_H
#define ROUTER_SETTINGS_H

#include <Arduino.h>

#include "config.h"
#include "eeprom_settings.h"
#include "router_runtime.h"
#include "shared_var.h"
#include "utils_pins.h"

inline constexpr int16_t SYSTEM_SETTING_MIN_WATTS{ -5000 };
inline constexpr int16_t SYSTEM_SETTING_MAX_WATTS{ 5000 };
inline constexpr int16_t RELAY_THRESHOLD_MAX_WATTS{ 5000 };
inline constexpr uint16_t RELAY_TIMER_MAX_MINUTES{ 1440 };

inline constexpr uint8_t getRelayCount()
{
  return RELAY_DIVERSION ? relays.get_size() : 0;
}

inline constexpr bool isTriacOutputIndex(const uint8_t outputIndex)
{
  return outputIndex < NO_OF_DUMPLOADS;
}

inline constexpr uint8_t relayArrayIndexFromOutput(const uint8_t outputIndex)
{
  return static_cast< uint8_t >(outputIndex - NO_OF_DUMPLOADS);
}

inline uint16_t makeRelayOutputMask()
{
  uint16_t mask{ 0 };

  if constexpr (RELAY_DIVERSION)
  {
    for (uint8_t idx = 0; idx < relays.get_size(); ++idx)
    {
      bit_set(mask, idx);
    }
  }

  return mask;
}

inline uint16_t makeTriacOutputMask()
{
  uint16_t mask{ 0 };

  for (uint8_t idx = 0; idx < NO_OF_DUMPLOADS; ++idx)
  {
    bit_set(mask, idx);
  }

  return mask;
}

inline bool triacIsForced(const uint8_t triacIndex)
{
  return Shared::b_overrideLoadOn[triacIndex] || bit_read(RouterRuntime::triacBoostMask, triacIndex);
}

inline void applyBoostOverridesToTriacs()
{
  // Intentionally left empty.
  // TRIAC boost is evaluated on demand via triacIsForced() so that switching BOOST OFF
  // immediately removes the forced contribution without corrupting the legacy override flags.
}

inline bool relayIsForcedByBoost(const uint8_t relayIndex)
{
  return bit_read(RouterRuntime::relayBoostMask, relayIndex);
}

inline bool relayIsStoppedByDiversion(const uint8_t relayIndex)
{
  return bit_read(RouterRuntime::relayDiversionMask, relayIndex);
}

// Functions implemented in router_settings.cpp
uint16_t getBoostMaskFromInputsAndOLED();
uint16_t getDiversionMaskFromInputsAndOLED();
void refreshRoutingMasks();
void initializeRuntimeRoutingCommands();
bool isAnyBoostActiveForOutput(uint8_t outputIndex);
bool isAnyDiversionActiveForOutput(uint8_t outputIndex);

#endif /* ROUTER_SETTINGS_H */
