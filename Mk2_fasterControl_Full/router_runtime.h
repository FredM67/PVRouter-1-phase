/**
 * @file router_runtime.h
 * @brief Runtime state shared by optional OLED/UI, boost and diversion features.
 */

#ifndef ROUTER_RUNTIME_H
#define ROUTER_RUNTIME_H

#include <Arduino.h>

inline constexpr uint8_t MAX_RELAY_OUTPUTS{ 6 };
inline constexpr uint8_t MAX_BOOST_CHANNELS{ 8 };
inline constexpr uint8_t MAX_DIVERSION_GROUPS{ 4 };

struct RelayRuntimeSettings
{
  int16_t surplusThreshold{ 1000 };
  int16_t importThreshold{ 200 };
  uint16_t minON_minutes{ 5 };
  uint16_t minOFF_minutes{ 5 };
};

namespace RouterRuntime
{
inline RelayRuntimeSettings relaySettings[MAX_RELAY_OUTPUTS]{};

inline uint16_t triacBoostMask{ 0 };
inline uint16_t triacDiversionMask{ 0 };
inline uint16_t relayBoostMask{ 0 };
inline uint16_t relayDiversionMask{ 0 };

inline bool boostStateFromOLED[MAX_BOOST_CHANNELS]{};
inline bool diversionAuthorizedFromOLED[MAX_DIVERSION_GROUPS]{}; /**< true = routing allowed for the group, false = group disabled */

inline int16_t requiredExportInWatts{ 0 };
inline int16_t diversionStartThresholdWatts{ 0 };
inline int32_t requiredExportPerMainsCycle_inIEU{ 0 };
inline int32_t diversionStartThreshold_inIEU{ 0 };

inline bool oledRefreshRequested{ true };
}

#endif /* ROUTER_RUNTIME_H */
