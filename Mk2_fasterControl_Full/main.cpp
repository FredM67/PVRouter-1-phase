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
static_assert(__cplusplus >= 201703L, "See also : https://github.com/FredM67/PVRouter-3-phase/blob/main/Mk2_fasterControl_Full/Readme.md");

#include <Arduino.h>  // may not be needed, but it's probably a good idea to include this

#include "config.h"
#include "calibration.h"
#include "processing.h"
#include "types.h"
#include "utils.h"
#include "utils_relay.h"
#include "utils_display.h"
#include "utils_oled.h"
#include "utils_temp.h"
#include "validation.h"

// --------------  general global variables -----------------
//
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, some variables need to be 'int32_t'
//

/**
 * @brief Forces all loads to full power.
 * 
 * @details This function overrides the normal load control logic and forces all loads to operate 
 *          at full power. It checks the state of the override pin and updates the override flags 
 *          for all loads accordingly. If debugging is enabled, it logs the state transitions 
 *          (e.g., "Trigger override!" or "End override!") to the debug output.
 * 
 *          Key operations include:
 *          - Checking the state of the override pin.
 *          - Updating the override flags for all loads.
 *          - Logging state transitions if debugging is enabled.
 * 
 * @return true if the override is active and all loads are forced to full power.
 * @return false if the override is not active.
 * 
 * @note This function is only executed if the `OVERRIDE_PIN_PRESENT` compile-time constant is defined.
 * 
 * @see getPinState
 * @see ENABLE_DEBUG
 */
bool forceFullPower()
{
  if constexpr (OVERRIDE_PIN_PRESENT)
  {
    const auto pinState{ getPinState(forcePin) };

#ifdef ENABLE_DEBUG
    static uint8_t previousState{ HIGH };
    if (previousState != pinState)
    {
      DBUGLN(!pinState ? F("Trigger override!") : F("End override!"));
    }

    previousState = pinState;
#endif

    for (auto &bOverrideLoad : b_overrideLoadOn)
    {
      bOverrideLoad = !pinState;
    }

    return !pinState;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Rotates the load priorities.
 * 
 * @details This function triggers a reordering of load priorities. It sets a flag to indicate 
 *          that the priorities need to be rotated and waits until the rotation is completed 
 *          within the Interrupt Service Routine (ISR). Once the rotation is done, it logs the 
 *          updated load priorities for debugging or monitoring purposes.
 * 
 *          Key operations include:
 *          - Setting the `b_reOrderLoads` flag to initiate priority rotation.
 *          - Waiting for the ISR to complete the rotation.
 *          - Logging the updated load priorities.
 * 
 * @note This function uses a delay to wait for the ISR to complete the rotation. Ensure that 
 *       the delay duration is appropriate for the system's timing requirements.
 * 
 * @see logLoadPriorities
 */
void proceedRotation()
{
  b_reOrderLoads = true;

  // waits till the priorities have been rotated from inside the ISR
  do
  {
    delay(10);
  } while (b_reOrderLoads);

  // prints the (new) load priorities
  logLoadPriorities();
}

/**
 * @brief Handles load priority in combination with dual tariff.
 * 
 * @details This function manages load priorities and overrides during dual tariff operation. 
 *          It detects transitions between off-peak and peak periods using the state of the 
 *          dual tariff pin. During the off-peak period, it rotates load priorities (if enabled) 
 *          and manages load overrides based on the elapsed time and temperature thresholds. 
 *          At the end of the off-peak period, it resets the state and logs the transition.
 * 
 *          Key operations include:
 *          - Detecting the start and end of off-peak periods.
 *          - Rotating load priorities automatically during off-peak periods.
 *          - Managing load overrides based on elapsed time and temperature thresholds.
 * 
 * @param currentTemperature_x100 The current temperature multiplied by 100 (e.g., 25.00°C is represented as 2500).
 *                                Used for temperature-based load override logic.
 * 
 * @return true if the high tariff (on-peak period) is active.
 * @return false if the low tariff (off-peak period) is active.
 * 
 * @note This function relies on several compile-time constants and global variables, such as 
 *       `PRIORITY_ROTATION`, `NO_OF_DUMPLOADS`, and `rg_OffsetForce`.
 * 
 * @see proceedRotation
 * @see getPinState
 */
bool proceedLoadPrioritiesAndOverridingDualTariff(const int16_t currentTemperature_x100)
{
  constexpr int16_t iTemperatureThreshold_x100{ iTemperatureThreshold * 100 };
  static bool pinOffPeakState{ HIGH };
  const auto pinNewState{ getPinState(dualTariffPin) };

  if (pinOffPeakState && !pinNewState)
  {
    // we start off-peak period
    DBUGLN(F("Change to off-peak period!"));

    ul_TimeOffPeak = millis();

    if constexpr (PRIORITY_ROTATION == RotationModes::AUTO)
    {
      proceedRotation();
    }
  }
  else
  {
    const auto ulElapsedTime{ static_cast< uint32_t >(millis() - ul_TimeOffPeak) };
    const auto pinState{ getPinState(forcePin) };

    for (uint8_t i = 0; i < NO_OF_DUMPLOADS; ++i)
    {
      // for each load, if we're inside off-peak period and within the 'force period', trigger the ISR to turn the load ON
      if (!pinOffPeakState && !pinNewState && (ulElapsedTime >= rg_OffsetForce[i][0]) && (ulElapsedTime < rg_OffsetForce[i][1]))
      {
        b_overrideLoadOn[i] = !pinState || (currentTemperature_x100 <= iTemperatureThreshold_x100);
      }
      else
      {
        b_overrideLoadOn[i] = !pinState;
      }
    }
  }
  // end of off-peak period
  if (!pinOffPeakState && pinNewState)
  {
    DBUGLN(F("Change to peak period!"));
  }

  pinOffPeakState = pinNewState;

  return (LOW == pinOffPeakState);
}

/**
 * @brief Handles load priority rotation and overriding logic.
 * 
 * @details This function manages the load priorities and overrides based on the current system state. 
 *          Since the system does not have access to a clock, it detects the start of the off-peak 
 *          period using the main energy meter. When the off-peak period starts, the function rotates 
 *          the load priorities for the next day. It also supports dual tariff systems, rotation after 
 *          a specified time, and manual overrides using a force pin. The function detects transitions 
 *          between off-peak and peak periods and triggers load priority rotations accordingly.
 * 
 *          Key operations include:
 *          - Detecting off-peak and peak period transitions.
 *          - Rotating load priorities automatically or based on user input.
 *          - Managing load overrides using a force pin.
 * 
 * @param currentTemperature_x100 The current temperature multiplied by 100 (e.g., 25.00°C is represented as 2500).
 *                                Used for temperature-based load override logic.
 * 
 * @return true if the off-peak tariff is active.
 * @return false if the on-peak tariff is active.
 * 
 * @note This function relies on several compile-time constants and global variables, such as 
 *       `DUAL_TARIFF`, `EMONESP_CONTROL`, `PRIORITY_ROTATION`, and `OVERRIDE_PIN_PRESENT`.
 * 
 * @see proceedLoadPrioritiesAndOverridingDualTariff
 * @see proceedRotation
 */
bool proceedLoadPrioritiesAndOverriding(const int16_t currentTemperature_x100)
{
  if constexpr (DUAL_TARIFF)
  {
    return proceedLoadPrioritiesAndOverridingDualTariff(currentTemperature_x100);
  }

  if constexpr (EMONESP_CONTROL)
  {
    static uint8_t pinRotationState{ HIGH };
    const auto pinNewState{ getPinState(rotationPin) };

    if (pinRotationState && !pinNewState)
    {
      DBUGLN(F("Trigger rotation!"));

      proceedRotation();
    }
    pinRotationState = pinNewState;
  }
  else if (ROTATION_AFTER_SECONDS < absenceOfDivertedEnergyCount)
  {
    if constexpr (PRIORITY_ROTATION == RotationModes::AUTO)
    {
      proceedRotation();
    }
    absenceOfDivertedEnergyCount = 0;
  }
  if constexpr (OVERRIDE_PIN_PRESENT)
  {
    const auto pinState{ getPinState(forcePin) };

    for (auto &bOverrideLoad : b_overrideLoadOn)
    {
      bOverrideLoad = !pinState;
    }
  }

  return false;
}

/**
 * @brief Checks and updates the diversion state.
 * 
 * @details This function monitors the state of the diversion pin to determine whether the diversion 
 *          is active or not. If the diversion pin state changes, it updates the `b_diversionOff` 
 *          flag accordingly. Additionally, if debugging is enabled, it logs the state transitions 
 *          (e.g., "Trigger diversion OFF!" or "End diversion OFF!") to the debug output.
 * 
 * @note This function is only executed if the `DIVERSION_PIN_PRESENT` compile-time constant is defined.
 * 
 * @see getPinState
 * @see ENABLE_DEBUG
 */
void checkDiversionOnOff()
{
  if constexpr (DIVERSION_PIN_PRESENT)
  {
    const auto pinState{ getPinState(diversionPin) };

#ifdef ENABLE_DEBUG
    static auto previousState{ HIGH };
    if (previousState != pinState)
    {
      DBUGLN(!pinState ? F("Trigger diversion OFF!") : F("End diversion OFF!"));
    }

    previousState = pinState;
#endif

    b_diversionOff = !pinState;
  }
}

/**
 * @brief Updates the temperature readings and sends a new request for the next cycle.
 * 
 * @details This function reads the temperature values from the sensors, validates them, 
 *          and updates the global temperature data. If a temperature reading is invalid 
 *          (e.g., disconnected sensor or out-of-range value), it is replaced with a 
 *          default value (`DEVICE_DISCONNECTED_RAW`). After processing the current 
 *          readings, the function sends a new request to the sensors to prepare for 
 *          the next cycle.
 * 
 *          Key operations include:
 *          - Reading temperature values from all sensors.
 *          - Validating the temperature readings.
 *          - Updating the global temperature data structure.
 *          - Sending a new request to the sensors for the next cycle.
 * 
 * @note This function is only executed if the `TEMP_SENSOR_PRESENT` compile-time constant is defined.
 * 
 * @see temperatureSensing
 * @see DEVICE_DISCONNECTED_RAW
 */
void updateTemperature()
{
  if constexpr (TEMP_SENSOR_PRESENT)
  {
    uint8_t idx{ temperatureSensing.get_size() };
    do
    {
      auto tmp = temperatureSensing.readTemperature(--idx);

      // if read temperature is 85 and the delta with previous is greater than 5, skip the value
      if (8500 == tmp && (abs(tmp - tx_data.temperature_x100[idx]) > 500))
      {
        tmp = DEVICE_DISCONNECTED_RAW;
      }

      tx_data.temperature_x100[idx] = tmp;
    } while (idx);

    temperatureSensing.requestTemperatures();  // for use next time around
  }
}

/**
 * @brief Performs calculations on data for logging purposes.
 * 
 * @details This function processes accumulated data over a logging period to calculate key metrics 
 *          such as power grid usage, diverted power, and voltage. These calculations are based on 
 *          the number of sample sets collected during the logging period and calibration constants.
 * 
 *          The function also adjusts the calculated voltage based on the logging period duration 
 *          (e.g., for periods longer than 10 seconds, a scaling factor is applied).
 * 
 * @note This function relies on global variables such as `copyOf_sumP_grid_overDL_Period`, 
 *       `copyOf_sampleSetsDuringThisDatalogPeriod`, and `f_voltageCal`.
 * 
 * @see sendResults
 * @see updateTemperature
 */
void processCalculationsForLogging()
{
  tx_data.powerGrid = copyOf_sumP_grid_overDL_Period / copyOf_sampleSetsDuringThisDatalogPeriod * powerCal_grid;
  tx_data.powerGrid *= -1;

  tx_data.powerDiverted = copyOf_sumP_diverted_overDL_Period / copyOf_sampleSetsDuringThisDatalogPeriod * powerCal_diverted;

  if constexpr (DATALOG_PERIOD_IN_SECONDS > 10)
  {
    tx_data.Vrms_L_x100 = static_cast< int32_t >((100 << 2) * f_voltageCal * sqrt(copyOf_sum_Vsquared / copyOf_sampleSetsDuringThisDatalogPeriod));
  }
  else
  {
    tx_data.Vrms_L_x100 = static_cast< int32_t >(100 * f_voltageCal * sqrt(copyOf_sum_Vsquared / copyOf_sampleSetsDuringThisDatalogPeriod));
  }
}

/**
 * @brief Called once during startup.
 * 
 * @details This function initializes various components and settings required for the program to run. 
 *          It sets up the serial communication, initializes the display, configures optional pins, 
 *          and ensures all loads are turned off at startup. Additionally, it logs the load priorities 
 *          and initializes temperature sensors if present.
 * 
 *          Key operations include:
 *          - Delaying to allow time for the Serial monitor to open.
 *          - Initializing the Serial interface and debug port.
 *          - Setting up the OLED display and initializing it.
 *          - Configuring optional pins and logging load priorities.
 *          - Initializing temperature sensors if the feature is enabled.
 *          - Printing the available free RAM for debugging purposes.
 * 
 * @note This function is executed only once at the beginning of the program.
 * 
 * @see printConfiguration
 * @see setupOLED
 * @see initializeDisplay
 * @see initializeProcessing
 * @see logLoadPriorities
 */
void setup()
{
  delay(delayBeforeSerialStarts);  // allow time to open Serial monitor

  DEBUG_PORT.begin(9600);
  Serial.begin(9600);  // initialize Serial interface, Do NOT set greater than 9600

  pinMode(4, OUTPUT);

  // On start, always display config info in the serial monitor
  printConfiguration();

  setupOLED();

  initializeDisplay();

  // initializes all loads to OFF at startup
  initializeProcessing();

  logLoadPriorities();

  if constexpr (TEMP_SENSOR_PRESENT)
  {
    temperatureSensing.initTemperatureSensors();
  }

  DBUG(F(">>free RAM = "));
  DBUGLN(freeRam());  // a useful value to keep an eye on
  DBUGLN(F("----"));
}

/**
 * @brief Handles tasks that need to be executed every second.
 * 
 * @details This function performs a series of operations that are executed once per second. 
 *          These include updating the absence of diverted energy count, toggling the watchdog pin 
 *          (if present), updating the watchdog, checking the diversion state, managing load priorities 
 *          and overrides, and refreshing the display. It also handles relay operations if relay diversion 
 *          is enabled.
 * 
 * @param bOffPeak A reference to a boolean indicating whether the off-peak tariff is active.
 *                 This value may be updated based on load priorities and overriding logic.
 * @param iTemperature_x100 The current temperature multiplied by 100 (e.g., 25.00°C is represented as 2500).
 *                          Used for temperature-based logic, such as load overrides.
 * 
 * @note This function relies on several compile-time constants and global variables, such as 
 *       `WATCHDOG_PIN_PRESENT`, `RELAY_DIVERSION`, and `EDD_isIdle`.
 * 
 * @see proceedLoadPrioritiesAndOverriding
 * @see forceFullPower
 * @see checkDiversionOnOff
 * @see refreshDisplay
 */
void handlePerSecondTasks(bool &bOffPeak, int16_t iTemperature_x100)
{
  if (EDD_isIdle)
  {
    ++absenceOfDivertedEnergyCount;
  }

  if constexpr (WATCHDOG_PIN_PRESENT)
  {
    togglePin(watchDogPin);
  }

  updateWatchdog();
  checkDiversionOnOff();

  if (!forceFullPower())
  {
    bOffPeak = proceedLoadPrioritiesAndOverriding(iTemperature_x100);
  }

  if constexpr (RELAY_DIVERSION)
  {
    relays.inc_duration();
    relays.proceed_relays();
  }
}

/**
 * @brief Main processor loop.
 * 
 * @details This function is the main execution loop of the program. It handles periodic tasks, 
 *          including refreshing the display, managing load priorities, processing data for logging, 
 *          and sending results. The loop is designed to handle non-time-critical tasks, as all 
 *          time-sensitive operations are performed within the Interrupt Service Routine (ISR).
 * 
 *          Key operations include:
 *          - Incrementing timers for periodic tasks.
 *          - Refreshing the display at a defined interval.
 *          - Managing energy diversion and load priorities.
 *          - Processing accumulated data for logging purposes.
 *          - Sending results to external systems.
 * 
 * @note The loop relies on several global variables and flags, such as `b_newCycle`, 
 *       `b_datalogEventPending`, and `absenceOfDivertedEnergyCount`. It also uses compile-time 
 *       constants like `SUPPLY_FREQUENCY` and `UPDATE_PERIOD_FOR_DISPLAYED_DATA`.
 * 
 * @see handlePerSecondTasks
 * @see processCalculationsForLogging
 * @see updateTemperature
 * @see sendResults
 */
void loop()
{
  static bool initLoop{ true };
  static uint8_t perSecondTimer{ 0 };
  static bool bOffPeak{ false };
  static uint8_t timerForDisplayUpdate{ 0 };
  static int16_t iTemperature_x100{ 0 };

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
      if (absenceOfDivertedEnergyCount > displayShutdown_inSeconds)
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
      handlePerSecondTasks(bOffPeak, iTemperature_x100);
    }
  }

  if (b_datalogEventPending)
  {
    if (initLoop)
    {
      initLoop = false;
      clearDisplay();
    }

    b_datalogEventPending = false;

    processCalculationsForLogging();

    if constexpr (RELAY_DIVERSION)
    {
      relays.update_average(tx_data.powerGrid);
    }

    updateTemperature();

    updateOLED(divertedEnergyTotal_Wh);

    sendResults(bOffPeak);
  }
}  // end of loop()