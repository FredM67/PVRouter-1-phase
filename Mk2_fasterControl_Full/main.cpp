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

  static int16_t sampleI_grid_raw;
  static int16_t sampleI_diverted_raw;

  switch (sample_index)
  {
    case 0:
      rawSample = ADC;  // store the ADC value (this one is for Voltage L1)
      //sampleV = ADC;                                // store the ADC value (this one is for Voltage)
      ADMUX = bit(REFS0) + currentSensor_diverted;  // set up the next conversion, which is for Diverted Current
      ++sample_index;                               // increment the control flag
      sampleI_diverted = sampleI_diverted_raw;
      sampleI_grid = sampleI_grid_raw;
      dataReady = true;  // all three ADC values can now be processed
      break;
    case 1:
      rawSample = ADC;  // store the ADC value (this one is for Voltage L1)
      //sampleI_diverted_raw = ADC;               // store the ADC value (this one is for Diverted Current)
      ADMUX = bit(REFS0) + currentSensor_grid;  // set up the next conversion, which is for Grid Current
      ++sample_index;                           // increment the control flag
      break;
    case 2:
      rawSample = ADC;  // store the ADC value (this one is for Voltage L1)
      //sampleI_grid_raw = ADC;              // store the ADC value (this one is for Grid Current)
      ADMUX = bit(REFS0) + voltageSensor;  // set up the next conversion, which is for Voltage
      sample_index = 0;                    // reset the control flag
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

  for (int16_t i = 0; i < NO_OF_DUMPLOADS; ++i)
  {
    logicalLoadState[i] = LoadStates::LOAD_OFF;
    pinMode(physicalLoadPin[i], OUTPUT);  // driver pin for Load #n
    physicalLoadState[i] = LoadStates::LOAD_OFF;
  }

  updatePortsStates();

  initializeDisplay();

  // First stop the ADC
  bit_clear(ADCSRA, ADEN);

  // Activate free-running mode
  ADCSRB = 0x00;

  // Set up the ADC to be free-running
  bit_set(ADCSRA, ADPS0);  // Set the ADC's clock to system clock / 128
  bit_set(ADCSRA, ADPS1);
  bit_set(ADCSRA, ADPS2);

  bit_set(ADCSRA, ADATE);  // set the Auto Trigger Enable bit in the ADCSRA register. Because
  // bits ADTS0-2 have not been set (i.e. they are all zero), the
  // ADC's trigger source is set to "free running mode".

  bit_set(ADCSRA, ADIE);  // set the ADC interrupt enable bit. When this bit is written
  // to one and the I-bit in SREG is set, the
  // ADC Conversion Complete Interrupt is activated.

  bit_set(ADCSRA, ADEN);  // Enable the ADC

  bit_set(ADCSRA, ADSC);  // start ADC manually first time

  sei();  // Enable Global Interrupts

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
  if (dataReady)  // flag is set after every set of ADC conversions
  {
    dataReady = false;       // reset the flag
    allGeneralProcessing();  // executed once for each set of V&I samples
  }
}  // end of loop()