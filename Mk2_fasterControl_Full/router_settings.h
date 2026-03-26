/**
 * @file router_settings.h
 * @brief Helpers for runtime configuration, EEPROM persistence and dynamic routing commands.
 */

#ifndef ROUTER_SETTINGS_H
#define ROUTER_SETTINGS_H

#include <Arduino.h>
#include <EEPROM.h>

#include "calibration.h"
#include "config.h"
#include "router_runtime.h"
#include "shared_var.h"
#include "utils_pins.h"

struct PersistentRelaySettings
{
  int16_t surplusThreshold;
  int16_t importThreshold;
  uint16_t minON_minutes;
  uint16_t minOFF_minutes;
};

struct PersistentRouterSettings
{
  uint16_t signature;
  uint8_t version;
  int16_t requiredExportInWatts;
  int16_t diversionStartThresholdWatts;
  PersistentRelaySettings relays[MAX_RELAY_OUTPUTS];
  uint16_t checksum;
};

inline constexpr uint16_t ROUTER_SETTINGS_SIGNATURE{ 0x524Du };
inline constexpr uint8_t ROUTER_SETTINGS_VERSION{ 1u };
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

inline uint16_t computeSettingsChecksum(const PersistentRouterSettings &settings)
{
  const auto *raw{ reinterpret_cast< const uint8_t * >(&settings) };
  uint16_t checksum{ 0 };

  for (uint16_t idx = 0; idx < sizeof(PersistentRouterSettings) - sizeof(settings.checksum); ++idx)
  {
    checksum = static_cast< uint16_t >(checksum + raw[idx]);
  }

  return checksum;
}

inline void refreshRuntimeThresholds()
{
  RouterRuntime::requiredExportPerMainsCycle_inIEU = static_cast< int32_t >(RouterRuntime::requiredExportInWatts * (1.0f / powerCal_grid));
  RouterRuntime::diversionStartThreshold_inIEU = static_cast< int32_t >(RouterRuntime::diversionStartThresholdWatts * (1.0f / powerCal_grid));
}

inline void loadRelayDefaultsToRuntime()
{
  for (uint8_t idx = 0; idx < MAX_RELAY_OUTPUTS; ++idx)
  {
    RouterRuntime::relaySettings[idx] = {};
  }

  if constexpr (RELAY_DIVERSION)
  {
    for (uint8_t idx = 0; idx < relays.get_size(); ++idx)
    {
      RouterRuntime::relaySettings[idx].surplusThreshold = relays.get_relay(idx).get_surplusThreshold();
      RouterRuntime::relaySettings[idx].importThreshold = relays.get_relay(idx).get_importThreshold();
      RouterRuntime::relaySettings[idx].minON_minutes = static_cast< uint16_t >(relays.get_relay(idx).get_minON() / 60u);
      RouterRuntime::relaySettings[idx].minOFF_minutes = static_cast< uint16_t >(relays.get_relay(idx).get_minOFF() / 60u);
    }
  }
}

inline void loadRuntimeSettingsDefaults()
{
  RouterRuntime::requiredExportInWatts = REQUIRED_EXPORT_IN_WATTS;
  RouterRuntime::diversionStartThresholdWatts = DIVERSION_START_THRESHOLD_WATTS;

  loadRelayDefaultsToRuntime();
  refreshRuntimeThresholds();
}

inline bool arePersistentSettingsValid(const PersistentRouterSettings &settings)
{
  if ((settings.signature != ROUTER_SETTINGS_SIGNATURE) || (settings.version != ROUTER_SETTINGS_VERSION))
  {
    return false;
  }

  return settings.checksum == computeSettingsChecksum(settings);
}

inline void loadRuntimeSettingsFromEEPROM()
{
  PersistentRouterSettings settings{};
  EEPROM.get(0, settings);

  if (!arePersistentSettingsValid(settings))
  {
    loadRuntimeSettingsDefaults();
    return;
  }

  RouterRuntime::requiredExportInWatts = settings.requiredExportInWatts;
  RouterRuntime::diversionStartThresholdWatts = settings.diversionStartThresholdWatts;

  loadRelayDefaultsToRuntime();

  for (uint8_t idx = 0; idx < getRelayCount(); ++idx)
  {
    RouterRuntime::relaySettings[idx].surplusThreshold = settings.relays[idx].surplusThreshold;
    RouterRuntime::relaySettings[idx].importThreshold = settings.relays[idx].importThreshold;
    RouterRuntime::relaySettings[idx].minON_minutes = settings.relays[idx].minON_minutes;
    RouterRuntime::relaySettings[idx].minOFF_minutes = settings.relays[idx].minOFF_minutes;
  }

  refreshRuntimeThresholds();
}

inline void saveRuntimeSettingsToEEPROM()
{
  PersistentRouterSettings settings{};
  settings.signature = ROUTER_SETTINGS_SIGNATURE;
  settings.version = ROUTER_SETTINGS_VERSION;
  settings.requiredExportInWatts = RouterRuntime::requiredExportInWatts;
  settings.diversionStartThresholdWatts = RouterRuntime::diversionStartThresholdWatts;

  for (uint8_t idx = 0; idx < getRelayCount(); ++idx)
  {
    settings.relays[idx].surplusThreshold = RouterRuntime::relaySettings[idx].surplusThreshold;
    settings.relays[idx].importThreshold = RouterRuntime::relaySettings[idx].importThreshold;
    settings.relays[idx].minON_minutes = RouterRuntime::relaySettings[idx].minON_minutes;
    settings.relays[idx].minOFF_minutes = RouterRuntime::relaySettings[idx].minOFF_minutes;
  }

  settings.checksum = computeSettingsChecksum(settings);
  EEPROM.put(0, settings);
}

inline uint16_t getBoostMaskFromInputsAndOLED()
{
  uint16_t mask{ 0 };

  for (uint8_t idx = 0; idx < BOOST_CONTROL_COUNT; ++idx)
  {
    bool active{ RouterRuntime::boostStateFromOLED[idx] };

    if (boostControls[idx].inputPin != unused_pin)
    {
      active |= !getPinState(boostControls[idx].inputPin);
    }

    if (active)
    {
      bit_set(mask, boostControls[idx].outputIndex);
    }
  }

  return mask;
}

inline uint16_t getDiversionMaskFromInputsAndOLED()
{
  uint16_t mask{ 0 };

  for (uint8_t idx = 0; idx < DIVERSION_GROUP_COUNT; ++idx)
  {
    // Diversion groups are now handled as authorizations:
    // - true  => routing allowed (displayed as ON)
    // - false => routing blocked for the configured outputs (displayed as OFF)
    bool authorized{ RouterRuntime::diversionAuthorizedFromOLED[idx] };

    if (diversionGroups[idx].inputPin != unused_pin)
    {
      // Physical input remains active LOW. Pulling the input LOW disables the group.
      authorized &= getPinState(diversionGroups[idx].inputPin);
    }

    if (!authorized)
    {
      mask |= diversionGroups[idx].outputMask;
    }
  }

  return static_cast< uint16_t >(mask & allOutputsMask());
}

inline void refreshRoutingMasks();

inline void initializeRuntimeRoutingCommands()
{
  for (uint8_t idx = 0; idx < MAX_BOOST_CHANNELS; ++idx)
  {
    RouterRuntime::boostStateFromOLED[idx] = false;
  }

  for (uint8_t idx = 0; idx < MAX_DIVERSION_GROUPS; ++idx)
  {
    // Startup requirement: routing is authorized by default.
    RouterRuntime::diversionAuthorizedFromOLED[idx] = true;
  }

  refreshRoutingMasks();
}

inline void refreshRoutingMasks()
{
  const uint16_t flatBoostMask{ getBoostMaskFromInputsAndOLED() };
  const uint16_t flatDiversionMask{ getDiversionMaskFromInputsAndOLED() };

  RouterRuntime::triacBoostMask = static_cast< uint16_t >(flatBoostMask & makeTriacOutputMask());
  RouterRuntime::relayBoostMask = static_cast< uint16_t >((flatBoostMask >> NO_OF_DUMPLOADS) & makeRelayOutputMask());

  RouterRuntime::triacDiversionMask = static_cast< uint16_t >(flatDiversionMask & makeTriacOutputMask());
  RouterRuntime::relayDiversionMask = static_cast< uint16_t >((flatDiversionMask >> NO_OF_DUMPLOADS) & makeRelayOutputMask());

  Shared::b_diversionEnabled = RouterRuntime::triacDiversionMask != makeTriacOutputMask();
}

inline bool isAnyBoostActiveForOutput(const uint8_t outputIndex)
{
  const uint16_t flatBoostMask{ getBoostMaskFromInputsAndOLED() };
  return bit_read(flatBoostMask, outputIndex);
}

inline bool isAnyDiversionActiveForOutput(const uint8_t outputIndex)
{
  const uint16_t flatDiversionMask{ getDiversionMaskFromInputsAndOLED() };
  return bit_read(flatDiversionMask, outputIndex);
}

inline bool triacIsForced(const uint8_t triacIndex)
{
  // Keep the legacy override state untouched and add TRIAC boost as a separate runtime source.
  // This avoids latching the override ON when a BOOST page/input is switched back to OFF.
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

#endif /* ROUTER_SETTINGS_H */
