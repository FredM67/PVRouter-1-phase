/**
 * @file main.cpp
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Main code
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */
static_assert(__cplusplus >= 201703L, "**** Please define 'gnu++17' in 'platform.txt' ! ****");
static_assert(__cplusplus >= 201703L, "See also : https://github.com/FredM67/PVRouter-3-phase/blob/main/Mk2_3phase_RFdatalog_temp/Readme.md");

#include <Arduino.h>  // may not be needed, but it's probably a good idea to include this

#include "config.h"
#include "calibration.h"
#include "processing.h"
#include "types.h"
#include "utils.h"
#include "utils_relay.h"
#include "utils_display.h"
//#include "validation.h"

// --------------  general global variables -----------------
//
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, some variables need to be 'int32_t'
//

/**
 * @brief Interrupt Service Routine - Interrupt-Driven Analog Conversion.
 * 
 * @details An Interrupt Service Routine is now defined which instructs the ADC to perform a conversion
 *          for each of the voltage and current sensors in turn.
 *
 *          This Interrupt Service Routine is for use when the ADC is in the free-running mode.
 *          It is executed whenever an ADC conversion has finished, approx every 104 µs. In
 *          free-running mode, the ADC has already started its next conversion by the time that
 *          the ISR is executed. The ISR therefore needs to "look ahead".
 *
 *          At the end of conversion Type N, conversion Type N+1 will start automatically. The ISR
 *          which runs at this point therefore needs to capture the results of conversion Type N,
 *          and set up the conditions for conversion Type N+2, and so on.
 *
 *          By means of various helper functions, all of the time-critical activities are processed
 *          within the ISR.
 *
 *          The main code is notified by means of a flag when fresh copies of loggable data are available.
 *
 *          Keep in mind, when writing an Interrupt Service Routine (ISR):
 *            - Keep it short
 *            - Don't use delay()
 *            - Don't do serial prints
 *            - Make variables shared with the main code volatile
 *            - Variables shared with main code may need to be protected by "critical sections"
 *            - Don't try to turn interrupts off or on
 *
 * @ingroup TimeCritical
 */
ISR(ADC_vect)
{
  static uint8_t sample_index{ 0 };
  int16_t rawSample;

  switch (sample_index)
  {
    case 0:
      rawSample = ADC;  // store the ADC value (this one is for Voltage L1)
      //sampleV = ADC;                                // store the ADC value (this one is for Voltage)
      ADMUX = bit(REFS0) + currentSensor_diverted;  // set up the next conversion, which is for Diverted Current
      ++sample_index;                               // increment the control flag
      //
      processVoltageRawSample(rawSample);
      break;
    case 1:
      rawSample = ADC;  // store the ADC value (this one is for Voltage L1)
      //sampleI_diverted_raw = ADC;               // store the ADC value (this one is for Diverted Current)
      ADMUX = bit(REFS0) + currentSensor_grid;  // set up the next conversion, which is for Grid Current
      ++sample_index;                           // increment the control flag
      //
      processDivertedCurrentRawSample(rawSample);
      break;
    case 2:
      rawSample = ADC;  // store the ADC value (this one is for Voltage L1)
      //sampleI_grid_raw = ADC;              // store the ADC value (this one is for Grid Current)
      ADMUX = bit(REFS0) + voltageSensor;  // set up the next conversion, which is for Voltage
      sample_index = 0;                    // reset the control flag
      //
      processGridCurrentRawSample(rawSample);
      break;
    default:
      sample_index = 0;  // to prevent lockup (should never get here)
  }
}

/**
 * @brief Called once during startup.
 * @details This function initializes a couple of variables we cannot init at compile time and
 *          sets a couple of parameters for runtime.
 *
 */
void setup()
{
  delay(delayBeforeSerialStarts);  // allow time to open Serial monitor

  DEBUG_PORT.begin(9600);
  Serial.begin(9600);  // initialize Serial interface, Do NOT set greater than 9600

  // On start, always display config info in the serial monitor
  printConfiguration();

  // initializes all loads to OFF at startup
  initializeProcessing();

  initializeOptionalPins();

  logLoadPriorities();

  initializeDisplay();

  DBUG(F(">>free RAM = "));
  DBUGLN(freeRam());  // a useful value to keep an eye on
  DBUGLN(F("----"));
}

/**
 * @brief Main processor.
 * @details None of the workload in loop() is time-critical.
 *          All the processing of ADC data is done within the ISR.
 *
 */
void loop()
{
  static uint8_t perSecondTimer{ 0 };
  static uint8_t timerForDisplayUpdate{ 0 };

  if (b_newCycle)  // flag is set after every pair of ADC conversions
  {
    b_newCycle = false;  // reset the flag
    ++perSecondTimer;
    ++timerForDisplayUpdate;

    if (timerForDisplayUpdate >= UPDATE_PERIOD_FOR_DISPLAYED_DATA)
    {  // the 4-digit display needs to be refreshed every few mS. For convenience,
      // this action is performed every N times around this processing loop.
      timerForDisplayUpdate = 0;

      // After a pre-defined period of inactivity, the 4-digit display needs to
      // close down in readiness for the next's day's data.
      //
      if (absenceOfDivertedEnergyCount > displayShutdown_inMainsCycles)
      {
        // clear the accumulators for diverted energy
        divertedEnergyTotal_Wh = 0;
        divertedEnergyRecent_IEU = 0;
        EDD_isActive = false;  // energy diversion detector is now inactive
      }

      configureValueForDisplay(EDD_isActive, divertedEnergyTotal_Wh);
      //          Serial.println(energyInBucket_prediction);
    }

    if (perSecondTimer >= SUPPLY_FREQUENCY)
    {
      perSecondTimer = 0;

      if constexpr (WATCHDOG_PIN_PRESENT)
      {
        togglePin(watchDogPin);
      }

      // checkDiversionOnOff();

      // if (!forceFullPower())
      // {
      //   bOffPeak = proceedLoadPrioritiesAndOverriding(iTemperature_x100);  // called every second
      // }

      if constexpr (RELAY_DIVERSION)
      {
        relays.inc_duration();
        relays.proceed_relays();
      }

      refreshDisplay();
    }
  }

  if (b_datalogEventPending)
  {
    b_datalogEventPending = false;
    // logData();

    if constexpr (RELAY_DIVERSION)
    {
      //relays.update_average(tx_data.power);
    }
  }
}  // end of loop()