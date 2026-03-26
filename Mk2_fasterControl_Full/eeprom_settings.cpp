/**
 * @file eeprom_settings.cpp
 * @brief EEPROM persistence for runtime router settings.
 */

#include "eeprom_settings.h"

#include <EEPROM.h>

#include "calibration.h"
#include "config.h"

uint16_t computeSettingsChecksum(const PersistentRouterSettings &settings)
{
  const auto *raw{ reinterpret_cast< const uint8_t * >(&settings) };
  uint16_t checksum{ 0 };

  for (uint16_t idx = 0; idx < sizeof(PersistentRouterSettings) - sizeof(settings.checksum); ++idx)
  {
    checksum = static_cast< uint16_t >(checksum + raw[idx]);
  }

  return checksum;
}

bool arePersistentSettingsValid(const PersistentRouterSettings &settings)
{
  if ((settings.signature != ROUTER_SETTINGS_SIGNATURE) || (settings.version != ROUTER_SETTINGS_VERSION))
  {
    return false;
  }

  return settings.checksum == computeSettingsChecksum(settings);
}

void refreshRuntimeThresholds()
{
  RouterRuntime::requiredExportPerMainsCycle_inIEU = static_cast< int32_t >(RouterRuntime::requiredExportInWatts * (1.0f / powerCal_grid));
  RouterRuntime::diversionStartThreshold_inIEU = static_cast< int32_t >(RouterRuntime::diversionStartThresholdWatts * (1.0f / powerCal_grid));
}

void loadRelayDefaultsToRuntime()
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

void loadRuntimeSettingsDefaults()
{
  RouterRuntime::requiredExportInWatts = REQUIRED_EXPORT_IN_WATTS;
  RouterRuntime::diversionStartThresholdWatts = DIVERSION_START_THRESHOLD_WATTS;

  loadRelayDefaultsToRuntime();
  refreshRuntimeThresholds();
}

void loadRuntimeSettingsFromEEPROM()
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

  const uint8_t relayCount{ RELAY_DIVERSION ? relays.get_size() : static_cast< uint8_t >(0) };
  for (uint8_t idx = 0; idx < relayCount; ++idx)
  {
    RouterRuntime::relaySettings[idx].surplusThreshold = settings.relays[idx].surplusThreshold;
    RouterRuntime::relaySettings[idx].importThreshold = settings.relays[idx].importThreshold;
    RouterRuntime::relaySettings[idx].minON_minutes = settings.relays[idx].minON_minutes;
    RouterRuntime::relaySettings[idx].minOFF_minutes = settings.relays[idx].minOFF_minutes;
  }

  refreshRuntimeThresholds();
}

void saveRuntimeSettingsToEEPROM()
{
  PersistentRouterSettings settings{};
  settings.signature = ROUTER_SETTINGS_SIGNATURE;
  settings.version = ROUTER_SETTINGS_VERSION;
  settings.requiredExportInWatts = RouterRuntime::requiredExportInWatts;
  settings.diversionStartThresholdWatts = RouterRuntime::diversionStartThresholdWatts;

  const uint8_t relayCount{ RELAY_DIVERSION ? relays.get_size() : static_cast< uint8_t >(0) };
  for (uint8_t idx = 0; idx < relayCount; ++idx)
  {
    settings.relays[idx].surplusThreshold = RouterRuntime::relaySettings[idx].surplusThreshold;
    settings.relays[idx].importThreshold = RouterRuntime::relaySettings[idx].importThreshold;
    settings.relays[idx].minON_minutes = RouterRuntime::relaySettings[idx].minON_minutes;
    settings.relays[idx].minOFF_minutes = RouterRuntime::relaySettings[idx].minOFF_minutes;
  }

  settings.checksum = computeSettingsChecksum(settings);
  EEPROM.put(0, settings);
}
