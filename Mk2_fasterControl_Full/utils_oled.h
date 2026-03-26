/**
 * @file utils_oled.h
 * @author Frederic Metrich (frederic.metrich@live.fr)
 * @brief Some utility functions for OLED display
 * @version 0.2
 * @date 2026-03-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef UTILS_OLED_H
#define UTILS_OLED_H

#include <Arduino.h>
#include <U8g2lib.h>

#include "router_settings.h"
#include "type_traits.hpp"
#include "types.h"
#include "utils.h"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
#define OLED_RESET -1       /**< Reset pin # (or -1 if sharing Arduino reset pin) */
#define SCREEN_ADDRESS 0x3C /**< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32 */

class u8x8_fake
{
public:
  u8x8_fake(uint8_t) {}

  bool begin()
  {
    return true;
  }
  void clearDisplay() {}
  void noInverse() {}
  uint8_t getCols()
  {
    return 16;
  }
  uint8_t getRows()
  {
    return 8;
  }

  void setFont(const uint8_t *) {}
  void drawString(uint8_t, uint8_t, const char *) {}
  void draw2x2String(uint8_t, uint8_t, const char *) {}
  void drawGlyph(uint8_t, uint8_t, uint8_t) {}

  void drawTile(uint8_t, uint8_t, uint8_t, uint8_t *) {}
};

extern conditional< TYPE_OF_DISPLAY == DisplayType::OLED, U8X8_SSD1306_128X64_NONAME_HW_I2C, u8x8_fake >::type u8x8;

inline constexpr uint8_t LOGO_WIDTH{ 72 };
inline constexpr uint8_t LOGO_HEIGHT{ 64 };
inline constexpr uint8_t OLED_ROWS{ 8 };
inline constexpr uint8_t OLED_COLS{ 16 };
inline constexpr uint8_t OLED_MAX_PAGES{ 20 };
inline constexpr uint8_t OLED_TITLE_ROW{ 0 };
inline constexpr uint8_t OLED_CONTENT_START_ROW{ 2 };

extern const unsigned char logo_xbm[] U8X8_PROGMEM;

enum class OledPageKind : uint8_t
{
  ENERGY,
  GRID,
  TEMP,
  ROUTING,
  BOOST,
  RELAY_CFG,
  SYSTEM_CFG,
  RESTART
};

enum class OledInteractionMode : uint8_t
{
  VIEW,
  LIST_NAV,
  VALUE_EDIT
};

struct OledPageDescriptor
{
  OledPageKind kind;
  uint8_t index;
};

namespace OledUI
{
inline OledPageDescriptor pages[OLED_MAX_PAGES]{};
inline uint8_t pageCount{ 0 };
inline uint8_t currentPage{ 0 };
inline OledInteractionMode mode{ OledInteractionMode::VIEW };
inline uint8_t selectedItem{ 0 };
inline uint16_t latestEnergyWh{ 0 };
inline uint32_t lastRenderMs{ 0 };
inline int16_t lastRenderedPage{ -1 };
inline uint32_t buttonPressMs{ 0 };
inline bool buttonWasPressed{ false };
inline uint8_t encoderPrevAB{ 0 };
inline int8_t encoderDelta{ 0 };
inline uint32_t lastInputMs{ 0 };
inline uint8_t activeRelayPageIndex{ 0 };
inline char lineBuffer[OLED_COLS + 1]{};
inline bool editBackupValid{ false };
inline bool editBackupIsSigned{ false };
inline int16_t editBackupSigned{ 0 };
inline uint16_t editBackupUnsigned{ 0 };
}

enum class ButtonEvent : uint8_t
{
  NONE,
  SHORT_PRESS,
  LONG_3S
};

// Small inline functions kept in header for performance (hot loop path)

inline void requestOLEDRefresh()
{
  RouterRuntime::oledRefreshRequested = true;
}

inline void clearDisplay()
{
  u8x8.clearDisplay();
}

inline void updateWatchdog()
{
  // Intentionally disabled on OLED because the user reported display glitches.
}

inline void setBodyFont()
{
  if constexpr (TYPE_OF_DISPLAY == DisplayType::OLED)
  {
    u8x8.setFont(u8x8_font_chroma48medium8_r);
  }
}

inline void setTitleFont()
{
  if constexpr (TYPE_OF_DISPLAY == DisplayType::OLED)
  {
    u8x8.setFont(u8x8_font_7x14B_1x2_r);
  }
}

inline int8_t readEncoderStep()
{
  if constexpr (TYPE_OF_DISPLAY != DisplayType::OLED)
  {
    return 0;
  }

  const uint8_t a{ static_cast< uint8_t >(!getPinState(oledEncoder.pinCLK)) };
  const uint8_t b{ static_cast< uint8_t >(!getPinState(oledEncoder.pinDT)) };
  const uint8_t ab{ static_cast< uint8_t >((a << 1) | b) };

  static const int8_t table[16]{ 0, -1, 1, 0,
                                 1, 0, 0, -1,
                                 -1, 0, 0, 1,
                                 0, 1, -1, 0 };

  const uint8_t transition{ static_cast< uint8_t >((OledUI::encoderPrevAB << 2) | ab) };
  OledUI::encoderPrevAB = ab;
  OledUI::encoderDelta = static_cast< int8_t >(OledUI::encoderDelta + table[transition]);

  if (OledUI::encoderDelta >= 4)
  {
    OledUI::encoderDelta = 0;
    return 1;
  }

  if (OledUI::encoderDelta <= -4)
  {
    OledUI::encoderDelta = 0;
    return -1;
  }

  return 0;
}

inline bool buttonPressed()
{
  if constexpr (TYPE_OF_DISPLAY != DisplayType::OLED)
  {
    return false;
  }

  return !getPinState(oledEncoder.pinSW);
}

inline ButtonEvent pollButtonEvent()
{
  const bool pressed{ buttonPressed() };

  if (pressed && !OledUI::buttonWasPressed)
  {
    OledUI::buttonWasPressed = true;
    OledUI::buttonPressMs = millis();
  }
  else if (!pressed && OledUI::buttonWasPressed)
  {
    OledUI::buttonWasPressed = false;
    const uint32_t heldMs{ millis() - OledUI::buttonPressMs };

    if (heldMs >= 3000u)
    {
      return ButtonEvent::LONG_3S;
    }
    return ButtonEvent::SHORT_PRESS;
  }

  return ButtonEvent::NONE;
}

inline uint8_t relayConfigItemCount()
{
  return 5;
}

inline uint8_t systemConfigItemCount()
{
  return 3;
}

uint8_t currentConfigItemCount();

inline void updateOLED(uint16_t value)
{
  OledUI::latestEnergyWh = value;
  requestOLEDRefresh();
}

inline void clearEditBackup()
{
  OledUI::editBackupValid = false;
}

// Functions implemented in utils_oled.cpp
uint8_t *get_tile_from_xbm(uint8_t tx, uint8_t ty, uint8_t xbm_byte_width, const unsigned char *xbm);
void u8x8_draw_xbm(uint8_t _tx, uint8_t _ty, uint8_t xbm_width, uint8_t xbm_height, const unsigned char *xbm);
void drawLine(uint8_t row, const char *text);
void drawLineFmt(uint8_t row, const char *fmt, ...);
void drawCenteredLine(uint8_t row, const char *text);
void drawCenteredLineFmt(uint8_t row, const char *fmt, ...);
void drawCenteredTitle(const char *title);
void drawCenteredTitleFmt(const char *fmt, ...);
void drawCentered2x2(uint8_t row, const char *text);
uint8_t visibleBoostPageCount();
uint8_t visibleDiversionCount();
uint8_t boostPageConfigIndexFromVisibleIndex(uint8_t visibleIndex);
uint8_t diversionConfigIndexFromVisibleIndex(uint8_t visibleIndex);
void rebuildPages();
int16_t *editableInt16ValueForCurrentSelection();
uint16_t *editableUInt16ValueForCurrentSelection();
void captureEditBackup();
void restoreEditBackup();
void clampEditedValue();
void saveCurrentConfigPage();
void goHome();
void requestSoftwareRestart();
void handleRotation(int8_t step);
void handleButton(ButtonEvent event);
void renderEnergyPage();
void renderGridPage();
void renderTempPage();
void renderRoutingPage();
void renderBoostPage(uint8_t visibleBoostIndex);
void renderRelayConfigPage(uint8_t relayIndex);
void renderSystemConfigPage();
void renderRestartPage();
void renderCurrentPage();
void setupOLED();
void handleOLED();

#endif /* UTILS_OLED_H */
