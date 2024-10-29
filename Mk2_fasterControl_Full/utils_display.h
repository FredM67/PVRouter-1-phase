/**
 * @file utils_display.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef UTILS_DISPLAY
#define UTILS_DISPLAY

#include "config_system.h"

// Various settings for the 4-digit display, which needs to be refreshed every few mS
constexpr uint8_t noOfDigitLocations{ 4 };
constexpr uint8_t noOfPossibleCharacters{ 22 };
constexpr uint8_t MAX_DISPLAY_TIME_COUNT{ 10 };            // no of processing loops between display updates
constexpr uint8_t UPDATE_PERIOD_FOR_DISPLAYED_DATA{ 50 };  // mains cycles
constexpr uint8_t DISPLAY_SHUTDOWN_IN_HOURS{ 8 };          // auto-reset after this period of inactivity
// #define DISPLAY_SHUTDOWN_IN_HOURS 0.01 // for testing that the display clears after 36 seconds

constexpr uint32_t displayShutdown_inMainsCycles{ DISPLAY_SHUTDOWN_IN_HOURS * mainsCyclesPerHour };

//  The two versions of the hardware require different logic.
#ifdef PIN_SAVING_HARDWARE

#define DRIVER_CHIP_DISABLED HIGH
#define DRIVER_CHIP_ENABLED LOW

// the primary segments are controlled by a pair of logic chips
const uint8_t noOfDigitSelectionLines = 4;  // <- for the 74HC4543 7-segment display driver
const uint8_t noOfDigitLocationLines = 2;   // <- for the 74HC138 2->4 line demultiplexer

uint8_t enableDisableLine = 5;  // <- affects the primary 7 segments only (not the DP)
uint8_t decimalPointLine = 14;  // <- this line has to be individually controlled.

uint8_t digitLocationLine[noOfDigitLocationLines] = { 16, 15 };
uint8_t digitSelectionLine[noOfDigitSelectionLines] = { 7, 9, 8, 6 };

// The final column of digitValueMap[] is for the decimal point status.  In this version,
// the decimal point has to be treated differently than the other seven segments, so
// a convenient means of accessing this column is provided.
//
uint8_t digitValueMap[noOfPossibleCharacters][noOfDigitSelectionLines + 1] = {
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
const uint8_t DPstatus_columnID = noOfDigitSelectionLines;

uint8_t digitLocationMap[noOfDigitLocations][noOfDigitLocationLines] = {
  LOW, LOW,    // Digit 1
  LOW, HIGH,   // Digit 2
  HIGH, LOW,   // Digit 3
  HIGH, HIGH,  // Digit 4
};

#else  // PIN_SAVING_HARDWARE

#define ON HIGH
#define OFF LOW

const uint8_t noOfSegmentsPerDigit = 8;  // includes one for the decimal point
enum digitEnableStates
{
  DIGIT_ENABLED,
  DIGIT_DISABLED
};

uint8_t digitSelectorPin[noOfDigitLocations] = { 16, 10, 13, 11 };
uint8_t segmentDrivePin[noOfSegmentsPerDigit] = { 2, 5, 12, 6, 7, 9, 8, 14 };

// The final column of segMap[] is for the decimal point status.  In this version,
// the decimal point is treated just like all the other segments, so there is
// no need to access this column specifically.
//
uint8_t segMap[noOfPossibleCharacters][noOfSegmentsPerDigit] = {
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
#endif  // PIN_SAVING_HARDWARE

uint8_t charsForDisplay[noOfDigitLocations] = { 20, 20, 20, 20 };  // all blank

void initializeDisplay()
{
#ifdef PIN_SAVING_HARDWARE
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
  digitalWrite(enableDisableLine, DRIVER_CHIP_DISABLED);

  // set up the control lines for the 74HC138 2->4 demux
  for (int16_t i = 0; i < noOfDigitLocationLines; ++i)
  {
    pinMode(digitLocationLine[i], OUTPUT);
  }
#else
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
    digitalWrite(digitSelectorPin[i], DIGIT_DISABLED);
  }

  for (int16_t i = 0; i < noOfSegmentsPerDigit; ++i)
  {
    digitalWrite(segmentDrivePin[i], OFF);
  }
#endif
}

// called infrequently, to update the characters to be displayed
void configureValueForDisplay(const bool _EDD_isActive, const uint16_t _divertedEnergyTotal_Wh)
{
  static uint8_t locationOfDot = 0;

  //  Serial.println(divertedEnergyTotal_Wh);

  if (_EDD_isActive)
  {
    uint16_t val = _divertedEnergyTotal_Wh;
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
      val = val / 10;
    }

    uint8_t thisDigit = val / 1000;
    charsForDisplay[0] = thisDigit;
    val -= 1000 * thisDigit;

    thisDigit = val / 100;
    charsForDisplay[1] = thisDigit;
    val -= 100 * thisDigit;

    thisDigit = val / 10;
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

void refreshDisplay()
{
  // This routine keeps track of which digit is being displayed and checks when its
  // display time has expired.  It then makes the necessary adjustments for displaying
  // the next digit.
  //   The two versions of the hardware require different logic.

#ifdef PIN_SAVING_HARDWARE
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
    uint8_t lineState;

    displayTime_count = 0;

    // 1. disable the Decimal Point driver line;
    digitalWrite(decimalPointLine, LOW);

    // 2. disable the driver chip while changes are taking place
    digitalWrite(enableDisableLine, DRIVER_CHIP_DISABLED);

    // 3. determine the next digit location to be active
    ++digitLocationThatIsActive;
    if (digitLocationThatIsActive >= noOfDigitLocations)
    {
      digitLocationThatIsActive = 0;
    }

    // 4. set up the digit location drivers for the new active location
    for (uint8_t line = 0; line < noOfDigitLocationLines; ++line)
    {
      lineState = digitLocationMap[digitLocationThatIsActive][line];
      digitalWrite(digitLocationLine[line], lineState);
    }

    // 5. determine the character to be displayed at this new location
    // (which includes the decimal point information)
    uint8_t digitVal = charsForDisplay[digitLocationThatIsActive];

    // 6. configure the 7-segment driver for the character to be displayed
    for (uint8_t line = 0; line < noOfDigitSelectionLines; ++line)
    {
      lineState = digitValueMap[digitVal][line];
      digitalWrite(digitSelectionLine[line], lineState);
    }

    // 7. set up the Decimal Point driver line;
    digitalWrite(decimalPointLine, digitValueMap[digitVal][DPstatus_columnID]);

    // 8. enable the 7-segment driver chip
    digitalWrite(enableDisableLine, DRIVER_CHIP_ENABLED);
  }

#else   // PIN_SAVING_HARDWARE

  // This version is more straightforward because the digit-enable lines can be
  // used to mask out all of the transitory states, including the Decimal Point.
  // The sequence is:
  //
  // 1. de-activate the digit-enable line that was previously active
  // 2. determine the next location which is to be active
  // 3. determine the relevant character for the new active location
  // 4. set up the segment drivers for the character to be displayed (includes the DP)
  // 5. activate the digit-enable line for the new active location

  static uint8_t displayTime_count = 0;
  static uint8_t digitLocationThatIsActive = 0;

  ++displayTime_count;

  if (displayTime_count > MAX_DISPLAY_TIME_COUNT)
  {
    displayTime_count = 0;

    // 1. de-activate the location which is currently being displayed
    digitalWrite(digitSelectorPin[digitLocationThatIsActive], DIGIT_DISABLED);

    // 2. determine the next digit location which is to be displayed
    ++digitLocationThatIsActive;
    if (digitLocationThatIsActive >= noOfDigitLocations)
    {
      digitLocationThatIsActive = 0;
    }

    // 3. determine the relevant character for the new active location
    uint8_t digitVal = charsForDisplay[digitLocationThatIsActive];

    // 4. set up the segment drivers for the character to be displayed (includes the DP)
    for (uint8_t segment = 0; segment < noOfSegmentsPerDigit; ++segment)
    {
      uint8_t segmentState = segMap[digitVal][segment];
      digitalWrite(segmentDrivePin[segment], segmentState);
    }

    // 5. activate the digit-enable line for the new active location
    digitalWrite(digitSelectorPin[digitLocationThatIsActive], DIGIT_ENABLED);
  }
#endif  // PIN_SAVING_HARDWARE

}  // end of refreshDisplay()

#endif /* UTILS_DISPLAY */
