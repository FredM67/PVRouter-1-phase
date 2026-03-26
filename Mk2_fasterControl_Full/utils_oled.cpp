/**
 * @file utils_oled.cpp
 * @author Frederic Metrich (frederic.metrich@live.fr)
 * @brief Implementation of OLED display rendering, navigation, and setup.
 * @version 0.2
 * @date 2026-03-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "utils_oled.h"

#include <stdarg.h>
#if defined(__AVR__)
#include <avr/wdt.h>
#endif

// Definition of the OLED display object
conditional< TYPE_OF_DISPLAY == DisplayType::OLED, U8X8_SSD1306_128X64_NONAME_HW_I2C, u8x8_fake >::type u8x8(/* reset=*/U8X8_PIN_NONE);

// Definition of the XBM logo stored in PROGMEM
const unsigned char logo_xbm[] U8X8_PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xef, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xef, 0xbf, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0x9f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xfb, 0xcf, 0x9f, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xf3,
  0xcf, 0x9f, 0x7f, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xdf, 0xdf, 0x3f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xdf, 0xdf, 0x3f, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xcf, 0xdf, 0xdf, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf,
  0x9f, 0xcf, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0xdf, 0xcf, 0xcf,
  0xff, 0xff, 0xff, 0xff, 0xf3, 0xbf, 0x9f, 0xcf, 0xef, 0x7f, 0xfe, 0xff,
  0xff, 0xe7, 0x3f, 0xbf, 0xef, 0xe7, 0x3f, 0xff, 0xff, 0xff, 0xcf, 0x7f,
  0xbf, 0xef, 0xf7, 0x9f, 0xff, 0xff, 0xff, 0x1f, 0x7f, 0xbe, 0xef, 0xf3,
  0xc7, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xbe, 0xef, 0xfb, 0xe7, 0xff, 0xff,
  0xff, 0xff, 0xfc, 0x3d, 0xe7, 0xf9, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xf9,
  0x79, 0xf7, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xf3, 0xfb, 0xff, 0x7e,
  0xfe, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff,
  0x1f, 0xff, 0xcf, 0xff, 0xff, 0x9f, 0xff, 0xc7, 0xff, 0x3f, 0xf8, 0x9f,
  0xff, 0xff, 0xef, 0xff, 0xe0, 0xff, 0xff, 0xe1, 0xff, 0xff, 0xff, 0xef,
  0x3f, 0xfc, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0x87, 0xff, 0xff,
  0xff, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xe1,
  0xff, 0xff, 0x3f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03, 0x00, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0x43, 0x0b, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xc7, 0x3f, 0xfe, 0xff, 0xff, 0xff, 0x07, 0x00, 0xf0, 0xc7, 0x7f, 0x7e,
  0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xff, 0xfe, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xc7, 0xff, 0xa3, 0x45, 0xce, 0xe2, 0xff, 0xff, 0xff, 0xc7, 0xff, 0x3b,
  0x32, 0xb4, 0xfa, 0xff, 0xff, 0xff, 0xc7, 0xbf, 0x23, 0x62, 0x92, 0xe2,
  0xff, 0xff, 0xff, 0xc7, 0x9f, 0xab, 0x14, 0xa4, 0xfa, 0xff, 0xff, 0xff,
  0x07, 0x80, 0xf7, 0xeb, 0xcd, 0xe6, 0x07, 0x00, 0xe0, 0xc7, 0x8f, 0xff,
  0xff, 0xff, 0xff, 0xe7, 0x23, 0xfc, 0xc7, 0x9f, 0xe4, 0xff, 0xff, 0xff,
  0xcf, 0x33, 0xfc, 0xc7, 0x1f, 0x00, 0xfc, 0xff, 0xff, 0x8f, 0x99, 0xf9,
  0xc7, 0x3f, 0x3c, 0xf8, 0xff, 0xff, 0x9f, 0x99, 0xf1, 0xc7, 0x3f, 0xfc,
  0xf0, 0xff, 0xff, 0x3f, 0xcc, 0xf3, 0xc7, 0x3f, 0xfc, 0xf0, 0xff, 0xff,
  0x3f, 0xc4, 0xe7, 0xc7, 0x3f, 0xfc, 0xe1, 0xff, 0xff, 0x07, 0x00, 0xe0,
  0xc7, 0x3f, 0xfc, 0xe1, 0xff, 0xff, 0x07, 0x00, 0xe8, 0xc7, 0x3f, 0xfc,
  0xf1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0x3f, 0xfc, 0xf1, 0xff, 0xff,
  0xff, 0xff, 0xff, 0x03, 0x00, 0xfc, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x3f, 0x3c, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x00,
  0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x1c, 0xfe, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0x3f, 0x7c, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x3f, 0x7c, 0xf8, 0xff, 0xff, 0xff, 0x18, 0x2b, 0x22, 0x3e, 0xfc,
  0xf8, 0xff, 0xff, 0xff, 0x6a, 0x6a, 0x73, 0x3e, 0xfc, 0xf0, 0xff, 0xff,
  0xff, 0xc8, 0x7a, 0x23, 0x3e, 0xfc, 0xf0, 0xff, 0xff, 0xff, 0x58, 0x6a,
  0x2b, 0x3e, 0xfc, 0xf1, 0xff, 0xff, 0xff, 0x97, 0x77, 0xeb, 0x3d, 0xfc,
  0xe1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xfc, 0xe3, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0x3f, 0xfc, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0x1f, 0xf8, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0xfc,
  0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

// XBM tile helpers

uint8_t *get_tile_from_xbm(uint8_t tx, uint8_t ty, uint8_t xbm_byte_width, const unsigned char *xbm)
{
  static uint8_t d[8];
  const auto *p{ xbm + tx + xbm_byte_width * (ty << 3) };

  memset(d, 0, sizeof(d));

  for (uint8_t i = 0; i < 8; ++i)
  {
    uint8_t b = u8x8_pgm_read(p);
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      if (b & (1 << bit))
      {
        d[bit] |= (1 << i);
      }
    }
    p += xbm_byte_width;
  }
  return d;
}

void u8x8_draw_xbm(uint8_t _tx, uint8_t _ty, uint8_t xbm_width, uint8_t xbm_height, const unsigned char *xbm)
{
  for (uint8_t y = 0; y < (xbm_height >> 3); ++y)
  {
    for (uint8_t x = 0; x < (xbm_width >> 3); ++x)
    {
      const auto tile{ get_tile_from_xbm(x, y, xbm_width >> 3, xbm) };
      u8x8.drawTile(_tx + x, _ty + y, 1, tile);
    }
  }
}

// Drawing helpers

void drawLine(const uint8_t row, const char *text)
{
  snprintf(OledUI::lineBuffer, sizeof(OledUI::lineBuffer), "%-16.16s", text);
  u8x8.drawString(0, row, OledUI::lineBuffer);
}

void drawLineFmt(const uint8_t row, const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  vsnprintf(OledUI::lineBuffer, sizeof(OledUI::lineBuffer), fmt, args);
  va_end(args);
  drawLine(row, OledUI::lineBuffer);
}

void drawCenteredLine(const uint8_t row, const char *text)
{
  char buffer[OLED_COLS + 1]{};
  snprintf(buffer, sizeof(buffer), "%s", text);

  const size_t len{ strlen(buffer) > OLED_COLS ? OLED_COLS : strlen(buffer) };
  const uint8_t leftPad{ static_cast< uint8_t >((OLED_COLS - len) / 2u) };

  memset(OledUI::lineBuffer, ' ', OLED_COLS);
  memcpy(&OledUI::lineBuffer[leftPad], buffer, len);
  OledUI::lineBuffer[OLED_COLS] = '\0';
  u8x8.drawString(0, row, OledUI::lineBuffer);
}

void drawCenteredLineFmt(const uint8_t row, const char *fmt, ...)
{
  char buffer[OLED_COLS + 1]{};
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  drawCenteredLine(row, buffer);
}

void drawCenteredTitle(const char *title)
{
  setTitleFont();
  drawCenteredLine(OLED_TITLE_ROW, title);
  setBodyFont();
}

void drawCenteredTitleFmt(const char *fmt, ...)
{
  char buffer[OLED_COLS + 1]{};
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  drawCenteredTitle(buffer);
}

void drawCentered2x2(const uint8_t row, const char *text)
{
  char textBuffer[9]{};
  snprintf(textBuffer, sizeof(textBuffer), "%s", text);

  const size_t len{ strlen(textBuffer) > 8 ? 8 : strlen(textBuffer) };
  const uint8_t leftPad{ static_cast< uint8_t >((8u - len) / 2u) };

  char paddedBuffer[9];
  memset(paddedBuffer, ' ', 8);
  memcpy(&paddedBuffer[leftPad], textBuffer, len);
  paddedBuffer[8] = '\0';

  u8x8.draw2x2String(0, row, paddedBuffer);
}

// Index mapping helpers

uint8_t visibleBoostPageCount()
{
  uint8_t count{ 0 };
  for (uint8_t idx = 0; idx < boostControls.size(); ++idx)
  {
    count += boostControls[idx].visibleOnOLED ? 1u : 0u;
  }
  return count;
}

uint8_t visibleDiversionCount()
{
  uint8_t count{ 0 };
  for (uint8_t idx = 0; idx < diversionGroups.size(); ++idx)
  {
    count += diversionGroups[idx].visibleOnOLED ? 1u : 0u;
  }
  return count;
}

uint8_t currentConfigItemCount()
{
  const auto &page{ OledUI::pages[OledUI::currentPage] };

  switch (page.kind)
  {
    case OledPageKind::ROUTING:
      return visibleDiversionCount() == 0 ? 1u : visibleDiversionCount();
    case OledPageKind::RELAY_CFG:
      return relayConfigItemCount();
    case OledPageKind::SYSTEM_CFG:
      return systemConfigItemCount();
    default:
      return 1;
  }
}

uint8_t boostPageConfigIndexFromVisibleIndex(uint8_t visibleIndex)
{
  for (uint8_t idx = 0; idx < boostControls.size(); ++idx)
  {
    if (!boostControls[idx].visibleOnOLED)
    {
      continue;
    }

    if (visibleIndex == 0)
    {
      return idx;
    }

    --visibleIndex;
  }

  return 0;
}

uint8_t diversionConfigIndexFromVisibleIndex(uint8_t visibleIndex)
{
  for (uint8_t idx = 0; idx < diversionGroups.size(); ++idx)
  {
    if (!diversionGroups[idx].visibleOnOLED)
    {
      continue;
    }

    if (visibleIndex == 0)
    {
      return idx;
    }

    --visibleIndex;
  }

  return 0;
}

// Page rebuild

void rebuildPages()
{
  uint8_t idx{ 0 };

  OledUI::pages[idx++] = { OledPageKind::ENERGY, 0 };
  OledUI::pages[idx++] = { OledPageKind::GRID, 0 };

  if constexpr (TEMP_SENSOR_PRESENT)
  {
    OledUI::pages[idx++] = { OledPageKind::TEMP, 0 };
  }

  OledUI::pages[idx++] = { OledPageKind::ROUTING, 0 };

  for (uint8_t boostIdx = 0; boostIdx < visibleBoostPageCount(); ++boostIdx)
  {
    OledUI::pages[idx++] = { OledPageKind::BOOST, boostIdx };
  }

  if constexpr (RELAY_DIVERSION)
  {
    for (uint8_t relayIdx = 0; relayIdx < getRelayCount(); ++relayIdx)
    {
      OledUI::pages[idx++] = { OledPageKind::RELAY_CFG, relayIdx };
    }
  }

  if constexpr (OLED_ENABLE_RUNTIME_SETTINGS)
  {
    OledUI::pages[idx++] = { OledPageKind::SYSTEM_CFG, 0 };
  }

  if constexpr (OLED_ENABLE_RESTART_PAGE)
  {
    OledUI::pages[idx++] = { OledPageKind::RESTART, 0 };
  }

  OledUI::pageCount = idx;

  if (OledUI::currentPage >= OledUI::pageCount)
  {
    OledUI::currentPage = 0;
  }
}

// Edit value management

int16_t *editableInt16ValueForCurrentSelection()
{
  const auto &page{ OledUI::pages[OledUI::currentPage] };

  if (page.kind == OledPageKind::SYSTEM_CFG)
  {
    if (OledUI::selectedItem == 0)
    {
      return &RouterRuntime::requiredExportInWatts;
    }
    if (OledUI::selectedItem == 1)
    {
      return &RouterRuntime::diversionStartThresholdWatts;
    }
  }
  else if (page.kind == OledPageKind::RELAY_CFG)
  {
    auto &relayCfg{ RouterRuntime::relaySettings[page.index] };

    switch (OledUI::selectedItem)
    {
      case 0:
        return &relayCfg.surplusThreshold;
      case 1:
        return &relayCfg.importThreshold;
      default:
        break;
    }
  }

  return nullptr;
}

uint16_t *editableUInt16ValueForCurrentSelection()
{
  const auto &page{ OledUI::pages[OledUI::currentPage] };

  if (page.kind == OledPageKind::RELAY_CFG)
  {
    auto &relayCfg{ RouterRuntime::relaySettings[page.index] };

    switch (OledUI::selectedItem)
    {
      case 2:
        return &relayCfg.minON_minutes;
      case 3:
        return &relayCfg.minOFF_minutes;
      default:
        break;
    }
  }

  return nullptr;
}

void captureEditBackup()
{
  OledUI::editBackupValid = false;

  if (auto *value = editableInt16ValueForCurrentSelection())
  {
    OledUI::editBackupValid = true;
    OledUI::editBackupIsSigned = true;
    OledUI::editBackupSigned = *value;
    return;
  }

  if (auto *value = editableUInt16ValueForCurrentSelection())
  {
    OledUI::editBackupValid = true;
    OledUI::editBackupIsSigned = false;
    OledUI::editBackupUnsigned = *value;
  }
}

void restoreEditBackup()
{
  if (!OledUI::editBackupValid)
  {
    return;
  }

  if (OledUI::editBackupIsSigned)
  {
    if (auto *value = editableInt16ValueForCurrentSelection())
    {
      *value = OledUI::editBackupSigned;
    }
  }
  else
  {
    if (auto *value = editableUInt16ValueForCurrentSelection())
    {
      *value = OledUI::editBackupUnsigned;
    }
  }

  OledUI::editBackupValid = false;
}

void clampEditedValue()
{
  if (auto *value = editableInt16ValueForCurrentSelection())
  {
    const auto &page{ OledUI::pages[OledUI::currentPage] };

    if (page.kind == OledPageKind::SYSTEM_CFG)
    {
      *value = constrain(*value, SYSTEM_SETTING_MIN_WATTS, SYSTEM_SETTING_MAX_WATTS);
    }
    else
    {
      *value = constrain(*value, 0, RELAY_THRESHOLD_MAX_WATTS);
    }
  }

  if (auto *value = editableUInt16ValueForCurrentSelection())
  {
    *value = constrain(*value, static_cast< uint16_t >(0), RELAY_TIMER_MAX_MINUTES);
  }
}

// Action helpers

void saveCurrentConfigPage()
{
  refreshRuntimeThresholds();
  saveRuntimeSettingsToEEPROM();
  requestOLEDRefresh();
}

void goHome()
{
  clearEditBackup();
  OledUI::currentPage = 0;
  OledUI::mode = OledInteractionMode::VIEW;
  OledUI::selectedItem = 0;
  requestOLEDRefresh();
}

void requestSoftwareRestart()
{
  clearDisplay();
  drawLine(2, "Redemarrage...");
  drawLine(4, "Patientez");
#if defined(__AVR__)
  wdt_enable(WDTO_15MS);
  while (true)
  {
  }
#else
  void (*resetFunc)(void) = nullptr;
  resetFunc();
#endif
}

// Navigation

void handleRotation(const int8_t step)
{
  if (step == 0)
  {
    return;
  }

  if (OledUI::mode == OledInteractionMode::VIEW)
  {
    int16_t nextPage{ static_cast< int16_t >(OledUI::currentPage) + step };
    if (nextPage < 0)
    {
      nextPage = static_cast< int16_t >(OledUI::pageCount - 1);
    }
    else if (nextPage >= OledUI::pageCount)
    {
      nextPage = 0;
    }
    OledUI::currentPage = static_cast< uint8_t >(nextPage);
    requestOLEDRefresh();
    return;
  }

  if (OledUI::mode == OledInteractionMode::LIST_NAV)
  {
    const uint8_t itemCount{ currentConfigItemCount() };
    int16_t nextItem{ static_cast< int16_t >(OledUI::selectedItem) + step };
    if (nextItem < 0)
    {
      nextItem = static_cast< int16_t >(itemCount - 1);
    }
    else if (nextItem >= itemCount)
    {
      nextItem = 0;
    }
    OledUI::selectedItem = static_cast< uint8_t >(nextItem);
    requestOLEDRefresh();
    return;
  }

  if (OledUI::mode == OledInteractionMode::VALUE_EDIT)
  {
    if (auto *value = editableInt16ValueForCurrentSelection())
    {
      *value = static_cast< int16_t >(*value + (step * 10));
      clampEditedValue();
      requestOLEDRefresh();
      return;
    }

    if (auto *value = editableUInt16ValueForCurrentSelection())
    {
      *value = static_cast< uint16_t >(*value + step);
      clampEditedValue();
      requestOLEDRefresh();
    }
  }
}

void handleButton(ButtonEvent event)
{
  if (event == ButtonEvent::NONE)
  {
    return;
  }

  const auto &page{ OledUI::pages[OledUI::currentPage] };

  if (event == ButtonEvent::LONG_3S)
  {
    if (OledUI::mode == OledInteractionMode::VALUE_EDIT)
    {
      restoreEditBackup();
      refreshRuntimeThresholds();
    }

    goHome();
    return;
  }

  if ((page.kind == OledPageKind::BOOST) && (event == ButtonEvent::SHORT_PRESS))
  {
    const uint8_t boostIndex{ boostPageConfigIndexFromVisibleIndex(page.index) };
    if (boostIndex >= boostControls.size()) { return; }
    RouterRuntime::boostStateFromOLED[boostIndex] ^= true;
    refreshRoutingMasks();
    requestOLEDRefresh();
    return;
  }

  if ((page.kind == OledPageKind::ROUTING) && (event == ButtonEvent::SHORT_PRESS))
  {
    if (visibleDiversionCount() == 0)
    {
      return;
    }

    if (OledUI::mode == OledInteractionMode::VIEW)
    {
      OledUI::mode = OledInteractionMode::LIST_NAV;
      OledUI::selectedItem = 0;
    }
    else
    {
      const uint8_t diversionIndex{ diversionConfigIndexFromVisibleIndex(OledUI::selectedItem) };
      if (diversionIndex >= diversionGroups.size()) { return; }
      RouterRuntime::diversionAuthorizedFromOLED[diversionIndex] ^= true;
      OledUI::mode = OledInteractionMode::VIEW;
      refreshRoutingMasks();
    }

    requestOLEDRefresh();
    return;
  }

  if (((page.kind == OledPageKind::RELAY_CFG) || (page.kind == OledPageKind::SYSTEM_CFG)) && (event == ButtonEvent::SHORT_PRESS))
  {
    if (OledUI::mode == OledInteractionMode::VIEW)
    {
      OledUI::mode = OledInteractionMode::LIST_NAV;
      OledUI::selectedItem = 0;
    }
    else if (OledUI::mode == OledInteractionMode::LIST_NAV)
    {
      const uint8_t itemCount{ currentConfigItemCount() };
      if (OledUI::selectedItem + 1 == itemCount)
      {
        clearEditBackup();
        saveCurrentConfigPage();
        OledUI::mode = OledInteractionMode::VIEW;
      }
      else
      {
        captureEditBackup();
        OledUI::mode = OledInteractionMode::VALUE_EDIT;
      }
    }
    else if (OledUI::mode == OledInteractionMode::VALUE_EDIT)
    {
      clampEditedValue();
      clearEditBackup();
      OledUI::mode = OledInteractionMode::LIST_NAV;
      refreshRuntimeThresholds();
    }

    requestOLEDRefresh();
    return;
  }

  if ((page.kind == OledPageKind::RESTART) && (event == ButtonEvent::SHORT_PRESS))
  {
    requestSoftwareRestart();
    return;
  }
}

// Page renderers

void renderEnergyPage()
{
  drawCenteredTitle("ENERGIE");

  char energyBuffer[9]{};
  snprintf(energyBuffer, sizeof(energyBuffer), "%u.%03u", OledUI::latestEnergyWh / 1000u, OledUI::latestEnergyWh % 1000u);
  drawCentered2x2(3, energyBuffer);
  drawCenteredLine(6, "kWh");
}

void renderGridPage()
{
  drawCenteredTitle("INFO RESEAU");
  drawCenteredLineFmt(2, "U  %3d.%02u V", tx_data.Vrms_L_x100 / 100, static_cast<unsigned>(abs(tx_data.Vrms_L_x100 % 100)));
  drawCenteredLineFmt(3, "Pinst %5dW", tx_data.powerGrid);

  if constexpr (RELAY_DIVERSION)
  {
    drawCenteredLineFmt(5, "PmRel %5ldW", static_cast<long>(relays.get_average()));
  }
  else
  {
    drawCenteredLine(5, "PmRel   n/a");
  }

  drawCenteredLineFmt(6, "Pdiv  %5dW", tx_data.powerDiverted);
}

void renderTempPage()
{
  drawCenteredTitle("TEMPERATURES");

  if constexpr (TEMP_SENSOR_PRESENT)
  {
    const uint8_t tempCount{ temperatureSensing.get_size() < 6u ? temperatureSensing.get_size() : 6u };
    for (uint8_t idx = 0; idx < tempCount; ++idx)
    {
      if ((OUTOFRANGE_TEMPERATURE == tx_data.temperature_x100[idx]) || (DEVICE_DISCONNECTED_RAW == tx_data.temperature_x100[idx]))
      {
        drawCenteredLineFmt(idx + OLED_CONTENT_START_ROW, "T%u : NC", idx + 1);
      }
      else
      {
        drawCenteredLineFmt(idx + OLED_CONTENT_START_ROW, "T%u : %3d.%02u C", idx + 1, tx_data.temperature_x100[idx] / 100,
                            static_cast<unsigned>(abs(tx_data.temperature_x100[idx] % 100)));
      }
    }
  }
}

void renderRoutingPage()
{
  drawCenteredTitle("ROUTAGE");

  if (visibleDiversionCount() == 0)
  {
    drawCenteredLine(4, "Aucune option");
    return;
  }

  for (uint8_t line = 0; line < visibleDiversionCount(); ++line)
  {
    const uint8_t diversionIndex{ diversionConfigIndexFromVisibleIndex(line) };
    if (diversionIndex >= diversionGroups.size()) { continue; }
    const char prefix{ (OledUI::mode == OledInteractionMode::LIST_NAV && OledUI::selectedItem == line) ? '>' : ' ' };
    bool authorized{ RouterRuntime::diversionAuthorizedFromOLED[diversionIndex] };
    if (diversionGroups[diversionIndex].inputPin != unused_pin)
    {
      authorized &= getPinState(diversionGroups[diversionIndex].inputPin);
    }
    drawCenteredLineFmt(line + OLED_CONTENT_START_ROW, "%cDiv %u  %s", prefix, diversionIndex + 1, authorized ? "ON" : "OFF");
  }

  drawCenteredLine(7, OledUI::mode == OledInteractionMode::LIST_NAV ? "Act. / 3s back" : "Clic=action");
}

void renderBoostPage(const uint8_t visibleBoostIndex)
{
  const uint8_t boostIndex{ boostPageConfigIndexFromVisibleIndex(visibleBoostIndex) };
  if (boostIndex >= boostControls.size()) { return; }
  const auto &cfg{ boostControls[boostIndex] };
  const bool active{ RouterRuntime::boostStateFromOLED[boostIndex] || (cfg.inputPin != unused_pin && !getPinState(cfg.inputPin)) };

  drawCenteredTitleFmt("BOOST %u", boostIndex + 1);

  if (isTriacOutputIndex(cfg.outputIndex))
  {
    drawCenteredLineFmt(3, "Sortie TRIAC %u", cfg.outputIndex + 1);
  }
  else
  {
    drawCenteredLineFmt(3, "Sortie REL %u", relayArrayIndexFromOutput(cfg.outputIndex) + 1);
  }

  drawCenteredLineFmt(5, "Etat : %s", active ? "ON" : "OFF");
  drawCenteredLine(7, "Clic=action");
}

void renderRelayConfigPage(const uint8_t relayIndex)
{
  const auto &cfg{ RouterRuntime::relaySettings[relayIndex] };
  drawCenteredTitleFmt("RELAIS %u", relayIndex + 1);
  drawCenteredLineFmt(2, "%c SM  : %5dW", OledUI::selectedItem == 0 ? (OledUI::mode == OledInteractionMode::VALUE_EDIT ? '*' : '>') : ' ', cfg.surplusThreshold);
  drawCenteredLineFmt(3, "%c SA  : %5dW", OledUI::selectedItem == 1 ? (OledUI::mode == OledInteractionMode::VALUE_EDIT ? '*' : '>') : ' ', cfg.importThreshold);
  drawCenteredLineFmt(4, "%c Ton : %4um", OledUI::selectedItem == 2 ? (OledUI::mode == OledInteractionMode::VALUE_EDIT ? '*' : '>') : ' ', cfg.minON_minutes);
  drawCenteredLineFmt(5, "%c Toff: %3um", OledUI::selectedItem == 3 ? (OledUI::mode == OledInteractionMode::VALUE_EDIT ? '*' : '>') : ' ', cfg.minOFF_minutes);
  drawCenteredLineFmt(6, "%c VALIDER", OledUI::selectedItem == 4 ? '>' : ' ');
  drawCenteredLine(7, "Act. / 3s back");
}

void renderSystemConfigPage()
{
  drawCenteredTitle("REQ / DIV");
  drawCenteredLineFmt(3, "%c REQ : %5dW", OledUI::selectedItem == 0 ? (OledUI::mode == OledInteractionMode::VALUE_EDIT ? '*' : '>') : ' ', RouterRuntime::requiredExportInWatts);
  drawCenteredLineFmt(4, "%c DIV : %5dW", OledUI::selectedItem == 1 ? (OledUI::mode == OledInteractionMode::VALUE_EDIT ? '*' : '>') : ' ', RouterRuntime::diversionStartThresholdWatts);
  drawCenteredLineFmt(6, "%c VALIDER", OledUI::selectedItem == 2 ? '>' : ' ');
  drawCenteredLine(7, "Act. / 3s back");
}

void renderRestartPage()
{
  drawCenteredTitle("RESTART");
  drawCenteredLine(3, "Clic = action");
  drawCenteredLine(4, "3s = back");
  drawCenteredLine(6, "Restart routeur");
}

void renderCurrentPage()
{
  if constexpr (TYPE_OF_DISPLAY != DisplayType::OLED)
  {
    return;
  }

  const bool pageChanged{ OledUI::lastRenderedPage != static_cast< int16_t >(OledUI::currentPage) };
  if (pageChanged)
  {
    clearDisplay();
    OledUI::lastRenderedPage = OledUI::currentPage;
  }

  const auto &page{ OledUI::pages[OledUI::currentPage] };

  switch (page.kind)
  {
    case OledPageKind::ENERGY:
      renderEnergyPage();
      break;
    case OledPageKind::GRID:
      renderGridPage();
      break;
    case OledPageKind::TEMP:
      renderTempPage();
      break;
    case OledPageKind::ROUTING:
      renderRoutingPage();
      break;
    case OledPageKind::BOOST:
      renderBoostPage(page.index);
      break;
    case OledPageKind::RELAY_CFG:
      renderRelayConfigPage(page.index);
      break;
    case OledPageKind::SYSTEM_CFG:
      renderSystemConfigPage();
      break;
    case OledPageKind::RESTART:
      renderRestartPage();
      break;
  }
}

// Public API

void setupOLED()
{
  if constexpr (TYPE_OF_DISPLAY == DisplayType::OLED)
  {
    if (!u8x8.begin())
    {
      Serial.println(F("u8x8 allocation failed"));
    }

    u8x8.clearDisplay();
    u8x8.noInverse();
    setBodyFont();
    u8x8_draw_xbm((u8x8.getCols() - (LOGO_WIDTH >> 3)) >> 1, (u8x8.getRows() - (LOGO_HEIGHT >> 3)) >> 1, LOGO_WIDTH, LOGO_HEIGHT, logo_xbm);
    delay(3000);

    rebuildPages();
    const uint8_t a{ static_cast< uint8_t >(!getPinState(oledEncoder.pinCLK)) };
    const uint8_t b{ static_cast< uint8_t >(!getPinState(oledEncoder.pinDT)) };
    OledUI::encoderPrevAB = static_cast< uint8_t >((a << 1) | b);
    requestOLEDRefresh();
  }
}

void handleOLED()
{
  if constexpr (TYPE_OF_DISPLAY != DisplayType::OLED)
  {
    return;
  }

  const int8_t step{ readEncoderStep() };
  const ButtonEvent buttonEvent{ pollButtonEvent() };

  handleRotation(step);
  handleButton(buttonEvent);

  if (RouterRuntime::oledRefreshRequested || (millis() - OledUI::lastRenderMs > 1000u))
  {
    renderCurrentPage();
    RouterRuntime::oledRefreshRequested = false;
    OledUI::lastRenderMs = millis();
  }
}
