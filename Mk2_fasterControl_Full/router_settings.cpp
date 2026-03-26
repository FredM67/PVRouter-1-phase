/**
 * @file router_settings.cpp
 * @brief Implementation of dynamic routing commands and mask computation.
 */

#include "router_settings.h"

uint16_t getBoostMaskFromInputsAndOLED()
{
  uint16_t mask{ 0 };

  for (uint8_t idx = 0; idx < boostControls.size(); ++idx)
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

uint16_t getDiversionMaskFromInputsAndOLED()
{
  uint16_t mask{ 0 };

  for (uint8_t idx = 0; idx < diversionGroups.size(); ++idx)
  {
    bool authorized{ RouterRuntime::diversionAuthorizedFromOLED[idx] };

    if (diversionGroups[idx].inputPin != unused_pin)
    {
      authorized &= getPinState(diversionGroups[idx].inputPin);
    }

    if (!authorized)
    {
      mask |= diversionGroups[idx].outputMask;
    }
  }

  return static_cast< uint16_t >(mask & ALL_LOADS_AND_RELAYS());
}

void refreshRoutingMasks()
{
  const uint16_t flatBoostMask{ getBoostMaskFromInputsAndOLED() };
  const uint16_t flatDiversionMask{ getDiversionMaskFromInputsAndOLED() };

  RouterRuntime::loadBoostMask = static_cast< uint16_t >(flatBoostMask & makeLoadOutputMask());
  RouterRuntime::relayBoostMask = static_cast< uint16_t >((flatBoostMask >> NO_OF_DUMPLOADS) & makeRelayOutputMask());

  RouterRuntime::loadDiversionMask = static_cast< uint16_t >(flatDiversionMask & makeLoadOutputMask());
  RouterRuntime::relayDiversionMask = static_cast< uint16_t >((flatDiversionMask >> NO_OF_DUMPLOADS) & makeRelayOutputMask());

  Shared::b_diversionEnabled = RouterRuntime::loadDiversionMask != makeLoadOutputMask();
}

void initializeRuntimeRoutingCommands()
{
  for (uint8_t idx = 0; idx < MAX_BOOST_CHANNELS; ++idx)
  {
    RouterRuntime::boostStateFromOLED[idx] = false;
  }

  for (uint8_t idx = 0; idx < MAX_DIVERSION_GROUPS; ++idx)
  {
    RouterRuntime::diversionAuthorizedFromOLED[idx] = true;
  }

  refreshRoutingMasks();
}

bool isAnyBoostActiveForOutput(const uint8_t outputIndex)
{
  const uint16_t flatBoostMask{ getBoostMaskFromInputsAndOLED() };
  return bit_read(flatBoostMask, outputIndex);
}

bool isAnyDiversionActiveForOutput(const uint8_t outputIndex)
{
  const uint16_t flatDiversionMask{ getDiversionMaskFromInputsAndOLED() };
  return bit_read(flatDiversionMask, outputIndex);
}
