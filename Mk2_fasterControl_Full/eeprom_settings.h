/**
 * @file eeprom_settings.h
 * @brief EEPROM persistence for runtime router settings.
 */

#ifndef EEPROM_SETTINGS_H
#define EEPROM_SETTINGS_H

#include <Arduino.h>

#include "router_runtime.h"

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

// Functions implemented in eeprom_settings.cpp
uint16_t computeSettingsChecksum(const PersistentRouterSettings &settings);
bool arePersistentSettingsValid(const PersistentRouterSettings &settings);
void refreshRuntimeThresholds();
void loadRelayDefaultsToRuntime();
void loadRuntimeSettingsDefaults();
void loadRuntimeSettingsFromEEPROM();
void saveRuntimeSettingsToEEPROM();

#endif /* EEPROM_SETTINGS_H */
