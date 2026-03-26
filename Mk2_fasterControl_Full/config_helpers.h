/**
 * @file config_helpers.h
 * @brief Output indexing helpers derived from user configuration values.
 *
 * Included by config.h after the user values (NO_OF_DUMPLOADS, relays, etc.)
 * are defined. Provides TRIAC(), RELAY(), ALL_OUTPUTS() for use in the
 * boost/diversion tables.
 */

#ifndef CONFIG_HELPERS_H
#define CONFIG_HELPERS_H

#include "config_types.h"

inline constexpr uint8_t TOTAL_ROUTER_OUTPUTS{
  static_cast< uint8_t >(NO_OF_DUMPLOADS + (RELAY_DIVERSION ? relays.get_size() : 0))
};

constexpr OutputId TRIAC(const uint8_t idx)
{
  return { idx };
}

constexpr OutputId RELAY(const uint8_t idx)
{
  return { static_cast< uint8_t >(NO_OF_DUMPLOADS + idx) };
}

constexpr uint16_t ALL_OUTPUTS()
{
  return TOTAL_ROUTER_OUTPUTS >= 16u ? 0xFFFFu : static_cast< uint16_t >((1u << TOTAL_ROUTER_OUTPUTS) - 1u);
}

#endif /* CONFIG_HELPERS_H */
