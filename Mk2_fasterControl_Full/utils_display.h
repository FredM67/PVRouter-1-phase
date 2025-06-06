/**
 * @file utils_display.h
 * @author Frederic Metrich (frederic.metrich@live.fr)
 * @brief 7-segments display functions
 * @version 0.1
 * @date 2024-10-29
 * 
 * @copyright Copyright (c) 2025
 * 
 * @ingroup 7SegDisplay
 */

#ifndef UTILS_DISPLAY_H
#define UTILS_DISPLAY_H

#include "config_system.h"
#include "config.h"
#include "FastDivision.h"

////////////////////////////////////////////////////////////////////////////////////////
// General Configuration (Shared by SEG_HW and SEG)
////////////////////////////////////////////////////////////////////////////////////////

inline constexpr uint8_t noOfDigitLocations{ 4 };
inline constexpr uint8_t noOfPossibleCharacters{ 14 };
inline constexpr uint8_t UPDATE_PERIOD_FOR_DISPLAYED_DATA{ 50 };  // mains cycles
inline constexpr uint8_t DISPLAY_SHUTDOWN_IN_HOURS{ 8 };          // auto-reset after this period of inactivity

inline constexpr uint32_t displayShutdown_inMainsCycles{ DISPLAY_SHUTDOWN_IN_HOURS * mainsCyclesPerHour };
inline constexpr uint8_t MAX_DISPLAY_TIME_COUNT{ 10 };  // no of processing loops between display updates

inline uint8_t charsForDisplay[noOfDigitLocations]{ 20, 20, 20, 20 };  // all blank

////////////////////////////////////////////////////////////////////////////////////////
// The 7-segment display can be driven in two ways:
// 1. By a set of logic chips (74HC4543 7-segment display driver and 74HC138 2->4 line demultiplexer)
// 2. By direct control of the segment lines and digit selection lines
//
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
// Hardware-Driven Display (SEG_HW)
////////////////////////////////////////////////////////////////////////////////////////

inline constexpr uint8_t DRIVER_CHIP_DISABLED{ HIGH };
inline constexpr uint8_t DRIVER_CHIP_ENABLED{ LOW };

// the primary segments are controlled by a pair of logic chips
inline constexpr uint8_t noOfDigitSelectionLines{ 4 };  // <- for the 74HC4543 7-segment display driver
inline constexpr uint8_t noOfDigitLocationLines{ 2 };   // <- for the 74HC138 2->4 line demultiplexer

inline constexpr uint8_t enableDisableLine{ 5 };  // <- affects the primary 7 segments only (not the DP)
inline constexpr uint8_t decimalPointLine{ 14 };  // <- this line has to be individually controlled.

inline constexpr uint8_t digitLocationLine[noOfDigitLocationLines]{ 16, 15 };
inline constexpr uint8_t digitSelectionLine[noOfDigitSelectionLines]{ 7, 9, 8, 6 };

////////////////////////////////////////////////////////////////////////////////////////
// The final column of digitValueMap[] is for the decimal point status.  In this version,
// the decimal point has to be treated differently than the other seven segments.
//
inline constexpr uint8_t digitValueMap[noOfPossibleCharacters][noOfDigitSelectionLines]{
  LOW, LOW, LOW, LOW,      // '0' <- element 0
  LOW, LOW, LOW, HIGH,     // '1' <- element 1
  LOW, LOW, HIGH, LOW,     // '2' <- element 2
  LOW, LOW, HIGH, HIGH,    // '3' <- element 3
  LOW, HIGH, LOW, LOW,     // '4' <- element 4
  LOW, HIGH, LOW, HIGH,    // '5' <- element 5
  LOW, HIGH, HIGH, LOW,    // '6' <- element 6
  LOW, HIGH, HIGH, HIGH,   // '7' <- element 7
  HIGH, LOW, LOW, LOW,     // '8' <- element 8
  HIGH, LOW, LOW, HIGH,    // '9' <- element 9
  HIGH, HIGH, HIGH, HIGH,  // ' ' <- element 10
  LOW, HIGH, HIGH, HIGH,   // 'F' <- element 11
  HIGH, LOW, HIGH, LOW,    // 'r' <- element 12
  LOW, LOW, HIGH, LOW,     // 'C' <- element 13
};

// a tidy means of identifying the DP status data when accessing the above table
inline constexpr uint8_t DPstatus_columnID{ noOfDigitSelectionLines };

inline constexpr uint8_t digitLocationMap[noOfDigitLocations][noOfDigitLocationLines]{
  LOW, LOW,    // Digit 1
  LOW, HIGH,   // Digit 2
  HIGH, LOW,   // Digit 3
  HIGH, HIGH,  // Digit 4
};

/**
 * @brief Initializes the display for hardware-driven 7-segment displays.
 * 
 * @details This function configures the necessary pin modes and initial states for hardware-driven 
 *          7-segment displays. It sets up the decimal point line, control lines for the 74HC4543 
 *          7-segment display driver, the enable line, and the control lines for the 74HC138 
 *          2-to-4 demultiplexer.
 * 
 * Key operations include:
 * - Configuring the IO drivers for the 4-digit display.
 * - Setting the pin mode for the decimal point line.
 * - Setting up the control lines for the 74HC4543 7-segment display driver.
 * - Setting up the enable line for the 74HC4543 7-segment display driver.
 * - Setting up the control lines for the 74HC138 2-to-4 demultiplexer.
 * 
 * @ingroup 7SegDisplay
 */
inline void initializeDisplayHW()
{
  // Configure the IO drivers for the 4-digit display
  pinMode(decimalPointLine, OUTPUT);  // The 'decimal point' line

  // Set up the control lines for the 74HC4543 7-segment display driver
  for (uint8_t i = 0; i < noOfDigitSelectionLines; ++i)
  {
    pinMode(digitSelectionLine[i], OUTPUT);
  }

  // Set up the enable line for the 74HC4543 7-segment display driver
  pinMode(enableDisableLine, OUTPUT);
  setPinState(enableDisableLine, DRIVER_CHIP_DISABLED);

  // Set up the control lines for the 74HC138 2->4 demultiplexer
  for (uint8_t i = 0; i < noOfDigitLocationLines; ++i)
  {
    pinMode(digitLocationLine[i], OUTPUT);
  }
}

/**
 * @brief Updates the 7-segment display for the next digit (hardware-driven).
 * 
 * @details This function handles the process of updating the 7-segment display by:
 *          - Disabling the driver chip and decimal point line.
 *          - Determining the next digit location to activate.
 *          - Setting up the digit location and character to display.
 *          - Enabling the driver chip for the new digit.
 * 
 * @note This function assumes the use of hardware-driven 7-segment displays (DisplayType::SEG_HW).
 * 
 * @ingroup 7SegDisplay
 */
inline void update7SegmentHWDisplay()
{
  static uint8_t digitLocationThatIsActive = 0;

  // 1. Disable the Decimal Point driver line
  setPinState(decimalPointLine, LOW);

  // 2. Disable the driver chip while changes are taking place
  setPinState(enableDisableLine, DRIVER_CHIP_DISABLED);

  // 3. Determine the next digit location to be active
  if (++digitLocationThatIsActive == noOfDigitLocations)
  {
    digitLocationThatIsActive = 0;
  }

  // 4. Set up the digit location drivers for the new active location
  for (uint8_t line = 0; line < noOfDigitLocationLines; ++line)
  {
    const auto& lineState{ digitLocationMap[digitLocationThatIsActive][line] };
    setPinState(digitLocationLine[line], lineState);
  }

  // 5. Determine the character to be displayed at this new location
  const auto& digitVal{ charsForDisplay[digitLocationThatIsActive] };

  // 6. Configure the 7-segment driver for the character to be displayed
  for (uint8_t line = 0; line < noOfDigitSelectionLines; ++line)
  {
    const auto& lineState{ digitValueMap[digitVal & 0x7F][line] };
    setPinState(digitSelectionLine[line], lineState);
  }

  // 7. Set up the Decimal Point driver line
  setPinState(decimalPointLine, digitVal & 0x80);  // DP


  // 8. Enable the 7-segment driver chip
  setPinState(enableDisableLine, DRIVER_CHIP_ENABLED);
}

// End of config for the version with the extra logic chips
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
// Software-Driven Display (SEG)
////////////////////////////////////////////////////////////////////////////////////////

inline constexpr uint8_t ON{ HIGH };
inline constexpr uint8_t OFF{ LOW };

inline constexpr uint8_t noOfSegmentsPerDigit{ 8 };  // includes one for the decimal point

enum class DigitEnableStates : uint8_t
{
  DIGIT_ENABLED,
  DIGIT_DISABLED
};

inline constexpr uint8_t digitSelectorPin[noOfDigitLocations]{ 16, 10, 13, 11 };
inline constexpr uint8_t segmentDrivePin[noOfSegmentsPerDigit]{ 2, 5, 12, 6, 7, 9, 8, 14 };

////////////////////////////////////////////////////////////////////////////////////////
// The final column of segMap[] is for the decimal point status.  In this version,
// the decimal point is treated separately from the other seven segments.
//
inline constexpr uint8_t segMap[noOfPossibleCharacters][noOfSegmentsPerDigit - 1]{
  ON, ON, ON, ON, ON, ON, OFF,        // '0' <- element 0
  OFF, ON, ON, OFF, OFF, OFF, OFF,    // '1' <- element 1
  ON, ON, OFF, ON, ON, OFF, ON,       // '2' <- element 2
  ON, ON, ON, ON, OFF, OFF, ON,       // '3' <- element 3
  OFF, ON, ON, OFF, OFF, ON, ON,      // '4' <- element 4
  ON, OFF, ON, ON, OFF, ON, ON,       // '5' <- element 5
  ON, OFF, ON, ON, ON, ON, ON,        // '6' <- element 6
  ON, ON, ON, OFF, OFF, OFF, OFF,     // '7' <- element 7
  ON, ON, ON, ON, ON, ON, ON,         // '8' <- element 8
  ON, ON, ON, ON, OFF, ON, ON,        // '9' <- element 9
  OFF, OFF, OFF, OFF, OFF, OFF, OFF,  // ' ' <- element 10
  ON, OFF, OFF, OFF, ON, ON, ON,      // 'F' <- element 11
  OFF, OFF, OFF, OFF, ON, OFF, ON,    // 'r' <- element 12
  ON, OFF, OFF, ON, ON, ON, OFF,      // 'C' <- element 13
};

/**
 * @brief Initializes the display for software-driven 7-segment displays.
 * 
 * @details Configures the necessary pin modes and initial states for software-driven 
 *          7-segment displays. This includes setting up the segment drive pins and 
 *          digit selector pins, disabling all digit selector pins, and turning off 
 *          all segment drive pins initially.
 * 
 * Key operations include:
 * - Configuring the pin modes for segment drive pins.
 * - Configuring the pin modes for digit selector pins.
 * - Disabling all digit selector pins.
 * - Turning off all segment drive pins.
 * 
 * @ingroup 7SegDisplay
 */
inline void initializeDisplaySW()
{
  // Set the pin mode for each segment drive pin
  for (uint8_t i = 0; i < noOfSegmentsPerDigit; ++i)
  {
    pinMode(segmentDrivePin[i], OUTPUT);
  }

  // Set the pin mode for each digit selector pin
  for (uint8_t i = 0; i < noOfDigitLocations; ++i)
  {
    pinMode(digitSelectorPin[i], OUTPUT);
  }

  // Disable all digit selector pins initially
  for (uint8_t i = 0; i < noOfDigitLocations; ++i)
  {
    setPinState(digitSelectorPin[i], (uint8_t)DigitEnableStates::DIGIT_DISABLED);
  }

  // Turn off all segment drive pins initially
  for (uint8_t i = 0; i < noOfSegmentsPerDigit; ++i)
  {
    setPinState(segmentDrivePin[i], OFF);
  }
}

/**
 * @brief Updates the 7-segment display for the next digit (software-driven).
 * 
 * @details This function handles the process of updating the 7-segment display for configurations 
 *          without additional driver chips. It deactivates the current digit, determines the next 
 *          digit to activate, sets up the segment drivers for the new digit, and activates the 
 *          corresponding digit-enable line.
 * 
 * Key operations include:
 * - Deactivating the currently active digit.
 * - Determining the next digit location to activate.
 * - Setting up the segment drivers for the character to be displayed.
 * - Activating the digit-enable line for the new active location.
 * 
 * @note This function assumes the use of software-driven 7-segment displays (DisplayType::SEG).
 * 
 * @ingroup 7SegDisplay
 */
inline void update7SegmentSWDisplay()
{
  static uint8_t digitLocationThatIsActive{ 0 };

  // 1. Deactivate the location which is currently being displayed
  setPinState(digitSelectorPin[digitLocationThatIsActive], (uint8_t)DigitEnableStates::DIGIT_DISABLED);

  // 2. Determine the next digit location to be displayed
  if (++digitLocationThatIsActive == noOfDigitLocations)
  {
    digitLocationThatIsActive = 0;
  }

  // 3. Determine the relevant character for the new active location
  const auto& digitVal{ charsForDisplay[digitLocationThatIsActive] };

  // 4. Set up the segment drivers for the character to be displayed (includes the DP)
  for (uint8_t segment = 0; segment < noOfSegmentsPerDigit - 1; ++segment)
  {
    const auto& segmentState{ segMap[digitVal & 0x7F][segment] };
    setPinState(segmentDrivePin[segment], segmentState);
  }
  setPinState(segmentDrivePin[noOfSegmentsPerDigit - 1], digitVal & 0x80);  // DP


  // 5. Activate the digit-enable line for the new active location
  setPinState(digitSelectorPin[digitLocationThatIsActive], (uint8_t)DigitEnableStates::DIGIT_ENABLED);
}

// End of config for the version without the extra logic chips
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
// Shared Functions
////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Initializes the display for hardware-driven 7-segment displays.
 * 
 * @details This function determines the type of display (hardware-driven or software-driven) 
 *          and calls the appropriate initialization function.
 * 
 * @ingroup 7SegDisplay
 */
inline void initializeDisplay()
{
  if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG_HW)
  {
    initializeDisplayHW();
  }
  else if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG)
  {
    initializeDisplaySW();
  }
}

/**
 * @brief Displays "OFF" on the 7-segment display.
 * 
 * @details This function configures the display to show "OFF" when diversion is disabled.
 *          It displays the text right-aligned on the 4-digit display.
 *          Uses the existing '0' character definition for 'O' to save memory.
 * 
 * @ingroup 7SegDisplay
 */
inline void displayOff()
{
  if constexpr (!(TYPE_OF_DISPLAY == DisplayType::SEG || TYPE_OF_DISPLAY == DisplayType::SEG_HW))
  {
    return;
  }

  // Set display to " OFF" (right-aligned)
  charsForDisplay[0] = 10;  // Blank
  charsForDisplay[1] = 0;   // 'O' (reusing '0' character)
  charsForDisplay[2] = 11;  // 'F'
  charsForDisplay[3] = 11;  // 'F'
}

/**
 * @brief Displays "FORC" on the 7-segment display for forced load override.
 * 
 * @details This function configures the display to show "FORC" when a load is overridden.
 *          Uses the existing '0' character definition for 'O' to save memory.
 * 
 * @ingroup 7SegDisplay
 */
inline void displayForced()
{
  if constexpr (!(TYPE_OF_DISPLAY == DisplayType::SEG || TYPE_OF_DISPLAY == DisplayType::SEG_HW))
  {
    return;
  }

  // Set display to "FORC"
  charsForDisplay[0] = 11;  // 'F'
  charsForDisplay[1] = 0;   // 'O' (reusing '0' character)
  charsForDisplay[2] = 12;  // 'r'
  charsForDisplay[3] = 13;  // 'C'
}

/**
 * @brief Configures the value for display on a 7-segment display.
 *
 * @details This function controls what is shown on the 7-segment display based on 
 *          the system state. It handles multiple display modes:
 *          - "FORC" when a load is in forced override mode
 *          - "OFF" when diversion is disabled
 *          - "Walking dots" when energy display is not active
 *          - Energy values with appropriate decimal point placement
 *
 * @param _EDD_isActive A boolean indicating whether the energy display is active.
 * @param _ValueToDisplay The energy value to be displayed (16-bit unsigned integer).
 * @param _diversionEnabled A boolean indicating if diversion is enabled (default=true).
 * @param _loadForced A boolean indicating if a load is in forced override mode (default=false).
 *
 * Display precedence order:
 * 1. Forced load status (shows "FOrC")
 * 2. Diversion enabled status (shows "OFF")
 * 3. Energy display inactive (shows walking dots)
 * 4. Energy value display
 *
 * For energy value display:
 * - Values up to 9999 show with decimal point after first digit (e.g., 1.234)
 * - Values above 9999 are divided by 10 and shown with decimal point after second digit (e.g., 12.34)
 * 
 * @ingroup 7SegDisplay
 */
inline void configureValueForDisplay(const bool _EDD_isActive, const uint16_t _ValueToDisplay, const bool _diversionEnabled = false,
                                     const bool _loadForced = false)
{
  if constexpr (!(TYPE_OF_DISPLAY == DisplayType::SEG || TYPE_OF_DISPLAY == DisplayType::SEG_HW))
  {
    return;
  }

  // Check for forced load first
  if (_loadForced)
  {
    displayForced();
    return;
  }

  // If diversion is disabled, show "OFF"
  if (!_diversionEnabled)
  {
    displayOff();
    return;
  }

  if (!_EDD_isActive)
  {
    static uint8_t locationOfDot{ 0 };

    // "walking dots" display
    charsForDisplay[0] = 10;
    charsForDisplay[1] = 10;
    charsForDisplay[2] = 10;
    charsForDisplay[3] = 10;

    if (++locationOfDot == noOfDigitLocations)
    {
      locationOfDot = 0;
    }

    charsForDisplay[locationOfDot] |= 0x80;  // dot

    return;
  }

  const auto energyValueExceeds10kWh{ _ValueToDisplay > 9999u };

  uint32_t tmpVal{ energyValueExceeds10kWh ? divu10(_ValueToDisplay) : _ValueToDisplay };

  divmod10(tmpVal, tmpVal, charsForDisplay[3]);  // Extract units place
  divmod10(tmpVal, tmpVal, charsForDisplay[2]);  // Extract tens place
  divmod10(tmpVal, tmpVal, charsForDisplay[1]);  // Extract hundreds place
  charsForDisplay[0] = tmpVal;                   // Remaining value is the thousands place

  // assign the decimal point location
  if (energyValueExceeds10kWh)
  {
    charsForDisplay[1] |= 0x80;
  }  // dec point after 2nd digit
  else
  {
    charsForDisplay[0] |= 0x80;
  }  // dec point after 1st digit
}

/**
 * @brief Refreshes the display by updating the active digit and its segments.
 *
 * @details This function manages the display of digits on a 7-segment display. It determines 
 *          which digit is currently active and updates the display to show the next digit 
 *          when the current digit's display time has expired. The logic differs based on 
 *          the type of display hardware.
 *
 *          For DisplayType::SEG_HW:
 *          - Updates the hardware-driven 7-segment display by disabling the driver chip, 
 *            setting up the next digit, and enabling the driver chip.
 *
 *          For DisplayType::SEG:
 *          - Updates the software-driven 7-segment display by deactivating the current digit, 
 *            setting up the next digit, and activating the corresponding digit-enable line.
 *
 * @ingroup 7SegDisplay
 */
inline void refresh7SegDisplay()
{
  // This routine keeps track of which digit is being displayed and checks when its
  // display time has expired.  It then makes the necessary adjustments for displaying
  // the next digit.
  //   The two versions of the hardware require different logic.

  if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG_HW || TYPE_OF_DISPLAY == DisplayType::SEG)
  {
    static uint8_t displayTime_count{ 0 };
    if (++displayTime_count != MAX_DISPLAY_TIME_COUNT)
    {
      return;
    }
    displayTime_count = 0;
  }

  if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG_HW)
  {
    update7SegmentHWDisplay();
  }
  else if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG)
  {
    update7SegmentSWDisplay();
  }
}

#endif /* UTILS_DISPLAY_H */
