/**
 * @file utils_display.h
 * @author Frederic Metrich (frederic.metrich@live.fr)
 * @brief 7-segments display functions
 * @version 0.1
 * @date 2024-10-29
 * 
 * @copyright Copyright (c) 2024
 * 
 * @ingroup 7SegDisplay
 */

#ifndef UTILS_DISPLAY_H
#define UTILS_DISPLAY_H

#include "config_system.h"

#include "FastDivision.h"

////////////////////////////////////////////////////////////////////////////////////////
// Various settings for the 4-digit display, which needs to be refreshed every few mS
inline constexpr uint8_t noOfDigitLocations{ 4 };
inline constexpr uint8_t noOfPossibleCharacters{ 22 };
inline constexpr uint8_t MAX_DISPLAY_TIME_COUNT{ 10 };            // no of processing loops between display updates
inline constexpr uint8_t UPDATE_PERIOD_FOR_DISPLAYED_DATA{ 50 };  // mains cycles
inline constexpr uint8_t DISPLAY_SHUTDOWN_IN_HOURS{ 8 };          // auto-reset after this period of inactivity

inline constexpr uint32_t displayShutdown_inMainsCycles{ DISPLAY_SHUTDOWN_IN_HOURS * mainsCyclesPerHour };

////////////////////////////////////////////////////////////////////////////////////////
// The 7-segment display can be driven in two ways:
// 1. By a set of logic chips (74HC4543 7-segment display driver and 74HC138 2->4 line demultiplexer)
// 2. By direct control of the segment lines and digit selection lines
//
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
// This is for a config with the extra logic chips
//
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
// the decimal point has to be treated differently than the other seven segments, so
// a convenient means of accessing this column is provided.
//
inline constexpr uint8_t digitValueMap[noOfPossibleCharacters][noOfDigitSelectionLines + 1]{
  LOW, LOW, LOW, LOW, LOW,      // '0' <- element 0
  LOW, LOW, LOW, HIGH, LOW,     // '1' <- element 1
  LOW, LOW, HIGH, LOW, LOW,     // '2' <- element 2
  LOW, LOW, HIGH, HIGH, LOW,    // '3' <- element 3
  LOW, HIGH, LOW, LOW, LOW,     // '4' <- element 4
  LOW, HIGH, LOW, HIGH, LOW,    // '5' <- element 5
  LOW, HIGH, HIGH, LOW, LOW,    // '6' <- element 6
  LOW, HIGH, HIGH, HIGH, LOW,   // '7' <- element 7
  HIGH, LOW, LOW, LOW, LOW,     // '8' <- element 8
  HIGH, LOW, LOW, HIGH, LOW,    // '9' <- element 9
  LOW, LOW, LOW, LOW, HIGH,     // '0.' <- element 10
  LOW, LOW, LOW, HIGH, HIGH,    // '1.' <- element 11
  LOW, LOW, HIGH, LOW, HIGH,    // '2.' <- element 12
  LOW, LOW, HIGH, HIGH, HIGH,   // '3.' <- element 13
  LOW, HIGH, LOW, LOW, HIGH,    // '4.' <- element 14
  LOW, HIGH, LOW, HIGH, HIGH,   // '5.' <- element 15
  LOW, HIGH, HIGH, LOW, HIGH,   // '6.' <- element 16
  LOW, HIGH, HIGH, HIGH, HIGH,  // '7.' <- element 17
  HIGH, LOW, LOW, LOW, HIGH,    // '8.' <- element 18
  HIGH, LOW, LOW, HIGH, HIGH,   // '9.' <- element 19
  HIGH, HIGH, HIGH, HIGH, LOW,  // ' '  <- element 20
  HIGH, HIGH, HIGH, HIGH, HIGH  // '.'  <- element 21
};

// a tidy means of identifying the DP status data when accessing the above table
inline constexpr uint8_t DPstatus_columnID = noOfDigitSelectionLines;

inline constexpr uint8_t digitLocationMap[noOfDigitLocations][noOfDigitLocationLines]{
  LOW, LOW,    // Digit 1
  LOW, HIGH,   // Digit 2
  HIGH, LOW,   // Digit 3
  HIGH, HIGH,  // Digit 4
};

// End of config for the version with the extra logic chips
////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
// This is for a config without the extra logic chips
//
inline constexpr uint8_t ON{ HIGH };
inline constexpr uint8_t OFF{ LOW };

inline constexpr uint8_t noOfSegmentsPerDigit{ 8 };  // includes one for the decimal point

inline constexpr bool DIGIT_ENABLED{ false };
inline constexpr bool DIGIT_DISABLED{ true };

inline constexpr uint8_t digitSelectorPin[noOfDigitLocations]{ 16, 10, 13, 11 };
inline constexpr uint8_t segmentDrivePin[noOfSegmentsPerDigit]{ 2, 5, 12, 6, 7, 9, 8, 14 };

////////////////////////////////////////////////////////////////////////////////////////
// The final column of segMap[] is for the decimal point status.  In this version,
// the decimal point is treated just like all the other segments, so there is
// no need to access this column specifically.
//
inline constexpr uint8_t segMap[noOfPossibleCharacters][noOfSegmentsPerDigit]{
  ON, ON, ON, ON, ON, ON, OFF, OFF,        // '0' <- element 0
  OFF, ON, ON, OFF, OFF, OFF, OFF, OFF,    // '1' <- element 1
  ON, ON, OFF, ON, ON, OFF, ON, OFF,       // '2' <- element 2
  ON, ON, ON, ON, OFF, OFF, ON, OFF,       // '3' <- element 3
  OFF, ON, ON, OFF, OFF, ON, ON, OFF,      // '4' <- element 4
  ON, OFF, ON, ON, OFF, ON, ON, OFF,       // '5' <- element 5
  ON, OFF, ON, ON, ON, ON, ON, OFF,        // '6' <- element 6
  ON, ON, ON, OFF, OFF, OFF, OFF, OFF,     // '7' <- element 7
  ON, ON, ON, ON, ON, ON, ON, OFF,         // '8' <- element 8
  ON, ON, ON, ON, OFF, ON, ON, OFF,        // '9' <- element 9
  ON, ON, ON, ON, ON, ON, OFF, ON,         // '0.' <- element 10
  OFF, ON, ON, OFF, OFF, OFF, OFF, ON,     // '1.' <- element 11
  ON, ON, OFF, ON, ON, OFF, ON, ON,        // '2.' <- element 12
  ON, ON, ON, ON, OFF, OFF, ON, ON,        // '3.' <- element 13
  OFF, ON, ON, OFF, OFF, ON, ON, ON,       // '4.' <- element 14
  ON, OFF, ON, ON, OFF, ON, ON, ON,        // '5.' <- element 15
  ON, OFF, ON, ON, ON, ON, ON, ON,         // '6.' <- element 16
  ON, ON, ON, OFF, OFF, OFF, OFF, ON,      // '7.' <- element 17
  ON, ON, ON, ON, ON, ON, ON, ON,          // '8.' <- element 18
  ON, ON, ON, ON, OFF, ON, ON, ON,         // '9.' <- element 19
  OFF, OFF, OFF, OFF, OFF, OFF, OFF, OFF,  // ' ' <- element 20
  OFF, OFF, OFF, OFF, OFF, OFF, OFF, ON    // '.' <- element 11
};

// End of config for the version without the extra logic chips
////////////////////////////////////////////////////////////////////////////////////////

uint8_t charsForDisplay[noOfDigitLocations]{ 20, 20, 20, 20 };  // all blank

/**
 * @brief Initializes the display based on the type of display defined by TYPE_OF_DISPLAY.
 * 
 * This function sets up the necessary pin modes and initial states for the display.
 * It supports two types of displays: SEG_HW and SEG.
 * 
 * - For DisplayType::SEG_HW (hardware-driven 7-segment display):
 *   - Configures the IO drivers for the 4-digit display.
 *   - Sets the pin mode for the decimal point line.
 *   - Sets up the control lines for the 74HC4543 7-seg display driver.
 *   - Sets up the enable line for the 74HC4543 7-seg display driver.
 *   - Sets up the control lines for the 74HC138 2->4 demux.
 * 
 * - For DisplayType::SEG (software-driven 7-segment display):
 *   - Sets the pin mode for each segment drive pin.
 *   - Sets the pin mode for each digit selector pin.
 *   - Disables all digit selector pins initially.
 *   - Turns off all segment drive pins initially.
 * 
 * @ingroup 7SegDisplay
 */
void initializeDisplay()
{
  if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG_HW)
  {
    // configure the IO drivers for the 4-digit display
    //
    // the Decimal Point line is driven directly from the processor
    pinMode(decimalPointLine, OUTPUT);  // the 'decimal point' line

    // set up the control lines for the 74HC4543 7-seg display driver
    for (int16_t i = 0; i < noOfDigitSelectionLines; ++i)
    {
      pinMode(digitSelectionLine[i], OUTPUT);
    }

    // an enable line is required for the 74HC4543 7-seg display driver
    pinMode(enableDisableLine, OUTPUT);  // for the 74HC4543 7-seg display driver
    setPinState(enableDisableLine, DRIVER_CHIP_DISABLED);

    // set up the control lines for the 74HC138 2->4 demux
    for (int16_t i = 0; i < noOfDigitLocationLines; ++i)
    {
      pinMode(digitLocationLine[i], OUTPUT);
    }
  }
  else if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG)
  {
    for (int16_t i = 0; i < noOfSegmentsPerDigit; ++i)
    {
      pinMode(segmentDrivePin[i], OUTPUT);
    }

    for (int16_t i = 0; i < noOfDigitLocations; ++i)
    {
      pinMode(digitSelectorPin[i], OUTPUT);
    }

    for (int16_t i = 0; i < noOfDigitLocations; ++i)
    {
      setPinState(digitSelectorPin[i], DIGIT_DISABLED);
    }

    for (int16_t i = 0; i < noOfSegmentsPerDigit; ++i)
    {
      setPinState(segmentDrivePin[i], OFF);
    }
  }
}

/**
 * @brief Configures the value for display on a 7-segment display.
 *
 * This function configures the value to be displayed on a 7-segment display.
 * It handles both active energy display and a "walking dots" display when the
 * energy display is not active.
 *
 * @param _EDD_isActive A boolean indicating whether the energy display is active.
 * @param _ValueToDisplay The value to be displayed, represented as a 16-bit unsigned integer.
 *
 * When the energy display is active, the function scales the value appropriately
 * and assigns digits to the display characters. If the value exceeds 10,000, it
 * is rescaled to fit within the display's constraints. The decimal point is placed
 * after the first or second digit based on the value.
 *
 * When the energy display is not active, the function displays a "walking dots"
 * pattern by cycling a dot through the display positions.
 * 
 * @ingroup 7SegDisplay
 */
void configureValueForDisplay(const bool _EDD_isActive, const uint16_t _ValueToDisplay)
{
  if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG || TYPE_OF_DISPLAY == DisplayType::SEG_HW)
  {
    static uint8_t locationOfDot = 0;

    if (_EDD_isActive)
    {
      uint16_t val = _ValueToDisplay;
      bool energyValueExceeds10kWh;

      if (val < 10000)
      {
        // no need to re-scale (display to 3 DPs)
        energyValueExceeds10kWh = false;
      }
      else
      {
        // re-scale is needed (display to 2 DPs)
        energyValueExceeds10kWh = true;
        val = divu10(val);
      }

      uint8_t thisDigit = divu10(divu10(divu10(val)));
      charsForDisplay[0] = thisDigit;
      val -= 1000 * thisDigit;

      thisDigit = divu10(divu10(val));
      charsForDisplay[1] = thisDigit;
      val -= 100 * thisDigit;

      thisDigit = divu10(val);
      charsForDisplay[2] = thisDigit;
      val -= 10 * thisDigit;

      charsForDisplay[3] = val;

      // assign the decimal point location
      if (energyValueExceeds10kWh)
      {
        charsForDisplay[1] += 10;
      }  // dec point after 2nd digit
      else
      {
        charsForDisplay[0] += 10;
      }  // dec point after 1st digit
    }
    else
    {
      // "walking dots" display
      charsForDisplay[locationOfDot] = 20;  // blank

      ++locationOfDot;
      if (locationOfDot >= noOfDigitLocations)
      {
        locationOfDot = 0;
      }

      charsForDisplay[locationOfDot] = 21;  // dot
    }
  }
}


/**
 * @brief Refreshes the display by updating the active digit and its segments.
 *
 * This routine manages the display of digits on a 7-segment display. It keeps track of 
 * which digit is currently being displayed and updates the display when the current 
 * digit's display time has expired. The logic differs based on the type of display hardware.
 *
 * For DisplayType::SEG_HW:
 * 1. Sets the decimal point line to 'off'.
 * 2. Disables the 7-segment driver chip.
 * 3. Determines the next digit location to be active.
 * 4. Sets up the location lines for the new active location.
 * 5. Determines the relevant character for the new active location.
 * 6. Configures the driver chip for the new character to be displayed.
 * 7. Sets up the decimal point line for the new active location.
 * 8. Enables the 7-segment driver chip.
 *
 * For DisplayType::SEG:
 * 1. Deactivates the digit-enable line that was previously active.
 * 2. Determines the next digit location to be active.
 * 3. Determines the relevant character for the new active location.
 * 4. Sets up the segment drivers for the character to be displayed (includes the DP).
 * 5. Activates the digit-enable line for the new active location.
 *
 * The function uses static variables to keep track of the display time count and the 
 * currently active digit location. When the display time count exceeds a predefined 
 * maximum value, the function updates the display to show the next digit.
 * 
 * @ingroup 7SegDisplay
 */
void refreshDisplay()
{
  // This routine keeps track of which digit is being displayed and checks when its
  // display time has expired.  It then makes the necessary adjustments for displaying
  // the next digit.
  //   The two versions of the hardware require different logic.

  if constexpr (TYPE_OF_DISPLAY == DisplayType::SEG_HW)
  {
    // With this version of the hardware, care must be taken that all transitory states
    // are masked out.  Note that the enableDisableLine only masks the seven primary
    // segments, not the Decimal Point line which must therefore be treated separately.
    // The sequence is:
    //
    // 1. set the decimal point line to 'off'
    // 2. disable the 7-segment driver chip
    // 3. determine the next location which is to be active
    // 4. set up the location lines for the new active location
    // 5. determine the relevant character for the new active location
    // 6. configure the driver chip for the new character to be displayed
    // 7. set up decimal point line for the new active location
    // 8. enable the 7-segment driver chip

    static uint8_t displayTime_count = 0;
    static uint8_t digitLocationThatIsActive = 0;

    ++displayTime_count;

    if (displayTime_count > MAX_DISPLAY_TIME_COUNT)
    {
      displayTime_count = 0;

      // 1. disable the Decimal Point driver line;
      setPinState(decimalPointLine, LOW);

      // 2. disable the driver chip while changes are taking place
      setPinState(enableDisableLine, DRIVER_CHIP_DISABLED);

      // 3. determine the next digit location to be active
      ++digitLocationThatIsActive;
      if (digitLocationThatIsActive >= noOfDigitLocations)
      {
        digitLocationThatIsActive = 0;
      }

      // 4. set up the digit location drivers for the new active location
      for (uint8_t line = 0; line < noOfDigitLocationLines; ++line)
      {
        const auto lineState{ digitLocationMap[digitLocationThatIsActive][line] };
        setPinState(digitLocationLine[line], lineState);
      }

      // 5. determine the character to be displayed at this new location
      // (which includes the decimal point information)
      const auto digitVal{ charsForDisplay[digitLocationThatIsActive] };

      // 6. configure the 7-segment driver for the character to be displayed
      for (uint8_t line = 0; line < noOfDigitSelectionLines; ++line)
      {
        const auto lineState{ digitValueMap[digitVal][line] };
        setPinState(digitSelectionLine[line], lineState);
      }

      // 7. set up the Decimal Point driver line;
      setPinState(decimalPointLine, digitValueMap[digitVal][DPstatus_columnID]);

      // 8. enable the 7-segment driver chip
      setPinState(enableDisableLine, DRIVER_CHIP_ENABLED);
    }
  }
  else if (TYPE_OF_DISPLAY == DisplayType::SEG)
  {
    // This version is more straightforward because the digit-enable lines can be
    // used to mask out all of the transitory states, including the Decimal Point.
    // The sequence is:
    //
    // 1. de-activate the digit-enable line that was previously active
    // 2. determine the next location which is to be active
    // 3. determine the relevant character for the new active location
    // 4. set up the segment drivers for the character to be displayed (includes the DP)
    // 5. activate the digit-enable line for the new active location

    static uint8_t displayTime_count{ 0 };
    static uint8_t digitLocationThatIsActive{ 0 };

    ++displayTime_count;

    if (displayTime_count > MAX_DISPLAY_TIME_COUNT)
    {
      displayTime_count = 0;

      // 1. de-activate the location which is currently being displayed
      setPinState(digitSelectorPin[digitLocationThatIsActive], DIGIT_DISABLED);

      // 2. determine the next digit location which is to be displayed
      ++digitLocationThatIsActive;
      if (digitLocationThatIsActive >= noOfDigitLocations)
      {
        digitLocationThatIsActive = 0;
      }

      // 3. determine the relevant character for the new active location
      const auto digitVal{ charsForDisplay[digitLocationThatIsActive] };

      // 4. set up the segment drivers for the character to be displayed (includes the DP)
      for (uint8_t segment = 0; segment < noOfSegmentsPerDigit; ++segment)
      {
        uint8_t segmentState = segMap[digitVal][segment];
        setPinState(segmentDrivePin[segment], segmentState);
      }

      // 5. activate the digit-enable line for the new active location
      setPinState(digitSelectorPin[digitLocationThatIsActive], DIGIT_ENABLED);
    }
  }
}  // end of refreshDisplay()

#endif /* UTILS_DISPLAY_H */
