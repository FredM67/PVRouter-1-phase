
#include <Arduino.h>

#include "calibration.h"
#include "dualtariff.h"
#include "processing.h"
#include "utils_pins.h"
#include "utils_display.h"

// allocation of analogue pins which are not dependent on the display type that is in use
// **************************************************************************************
inline constexpr uint8_t voltageSensor{ OLD_PCB ? 3 : 0 };          /**< A0 is for the voltage sensor (A3 for the old PCB) */
inline constexpr uint8_t currentSensor_grid{ OLD_PCB ? 5 : 1 };     /**< A1 is for CT1 which measures grid current (A5 for the old PCB) */
inline constexpr uint8_t currentSensor_diverted{ OLD_PCB ? 4 : 3 }; /**< A3 is for CT2 which measures diverted current (A4 for the old PCB) */
// ------------------------------------------

uint8_t loadPrioritiesAndState[NO_OF_DUMPLOADS]; /**< load priorities */

// For an enhanced polarity detection mechanism, which includes a persistence check
inline constexpr uint8_t PERSISTENCE_FOR_POLARITY_CHANGE{ 1 }; /**< allows polarity changes to be confirmed */

inline constexpr uint8_t POST_ZERO_CROSSING_MAX_COUNT{ 3 }; /**< allows trigger device to be reliably armed */

// Define operating limits for the LP filters which identify DC offset in the voltage
// sample streams. By limiting the output range, these filters always should start up
// correctly.
int32_t DCoffset_V_long{ 512L * 256 };                  /**< <--- for LPF */
constexpr int32_t DCoffset_V_min{ (512L - 100) * 256 }; /**< mid-point of ADC minus a working margin */
constexpr int32_t DCoffset_V_max{ (512L + 100) * 256 }; /**< mid-point of ADC plus a working margin */

constexpr int16_t DCoffset_I{ 512 }; /**< nominal mid-point value of ADC @ x1 scale */

constexpr int32_t capacityOfEnergyBucket_long{ static_cast< int32_t >(WORKING_ZONE_IN_JOULES * SUPPLY_FREQUENCY * (1.0F / powerCal_grid)) }; /**< main energy bucket for single-phase use, with units of Joules * SUPPLY_FREQUENCY */

constexpr int32_t midPointOfEnergyBucket_long{ capacityOfEnergyBucket_long >> 1 }; /**< for resetting flexible thresholds */

constexpr int32_t lowerThreshold_default{ midPointOfEnergyBucket_long }; /**< default lower threshold for the energy bucket (50% of capacity) */
constexpr int32_t upperThreshold_default{ midPointOfEnergyBucket_long }; /**< default upper threshold for the energy bucket (50% of capacity) */

constexpr int32_t antiCreepLimit_inIEUperMainsCycle{ static_cast< int32_t >(ANTI_CREEP_LIMIT * (1.0F / powerCal_diverted)) };     /**< threshold value in Integer Energy Units (IEU) that prevents small measurement noise from being incorrectly registered as diverted energy */
constexpr int32_t requiredExportPerMainsCycle_inIEU{ static_cast< int32_t >(REQUIRED_EXPORT_IN_WATTS * (1.0F / powerCal_grid)) }; /**< target amount of energy to be exported to the grid during each mains cycle, expressed in Integer Energy Units (IEU) */
// When using integer maths, calibration values that have supplied in floating point
// form need to be rescaled.

// When using integer maths, the SIZE of the ENERGY BUCKET is altered to match the
// scaling of the energy detection mechanism that is in use.  This avoids the need
// to re-scale every energy contribution, thus saving processing time.  This process
// is described in more detail in the function, allGeneralProcessing(), just before
// the energy bucket is updated at the start of each new cycle of the mains.
//
// An electricity meter has a small range over which energy can ebb and flow without
// penalty.  This has been termed its "sweet-zone".  For optimal performance, the energy
// bucket of a PV Router should match this value.  The sweet-zone value is therefore
// included in the calculation below.
//
int32_t energyInBucket_long{ 0 };  /**< in Integer Energy Units */
int32_t lowerEnergyThreshold{ 0 }; /**< dynamic lower threshold */
int32_t upperEnergyThreshold{ 0 }; /**< dynamic upper threshold */

int32_t divertedEnergyRecent_IEU{ 0 }; /**< Hi-res accumulator of limited range */
uint16_t divertedEnergyTotal_Wh{ 0 };  /**< WattHour register of 63K range */

// For recording the accumulated amount of diverted energy data (using CT2), a similar
// calibration mechanism is required.  Rather than a bucket with a fixed capacity, the
// accumulator for diverted energy just needs to be scaled correctly.  As soon as its
// value exceeds 1 Wh, an associated WattHour register is incremented, and the
// accumulator's value is decremented accordingly. The calculation below is to determine
// the scaling for this accumulator.

constexpr int32_t IEU_per_Wh_diverted{ static_cast< int32_t >(JOULES_PER_WATT_HOUR * SUPPLY_FREQUENCY * (1.0F / powerCal_diverted)) };  // depends on powerCal, frequency & the 'sweetzone' size.

bool recentTransition{ false };                   /**< a load state has been recently toggled */
uint8_t postTransitionCount{ 0 };                 /**< counts the number of cycle since last transition */
constexpr uint8_t POST_TRANSITION_MAX_COUNT{ 3 }; /**< allows each transition to take effect */
uint8_t activeLoad{ NO_OF_DUMPLOADS };            /**< current active load */

int32_t sumP_grid{ 0 };                   /**< for per-cycle summation of 'real power' */
int32_t sumP_grid_overDL_Period{ 0 };     /**< for per-cycle summation of 'real power' during datalog period */
int32_t sumP_diverted{ 0 };               /**< for per-cycle summation of 'real power' */
int32_t sumP_diverted_overDL_Period{ 0 }; /**< for per-cycle summation of 'real power' during datalog period */
int32_t cumVdeltasThisCycle_long{ 0 };    /**< for the LPF which determines DC offset (voltage) */
int32_t l_sum_Vsquared{ 0 };              /**< for summation of V^2 values during datalog period */

int32_t realEnergy_grid{ 0 };           /**< stores the calculated real energy from the grid connection point (CT1) for the current mains cycle */
int32_t realEnergy_diverted{ 0 };       /**< stores the calculated real energy diverted to controlled loads (CT2) for the current mains cycle */
int32_t energyInBucket_prediction{ 0 }; /**< predicted energy level at the end of the current mains cycle */
int32_t sampleVminusDC_long{ 0 };       /**< voltage sample with DC offset removed */

Polarities polarityOfMostRecentVsample;    /**< for zero-crossing detection */
Polarities polarityConfirmed;              /**< for zero-crossing detection */
Polarities polarityConfirmedOfLastSampleV; /**< for zero-crossing detection */

// For a mechanism to check the continuity of the sampling sequence
uint8_t sampleSetsDuringThisMainsCycle{ 0 };     /**< number of sample sets during each mains cycle */
uint16_t sampleSetsDuringThisDatalogPeriod{ 0 }; /**< number of sample sets during each datalogging period */

uint8_t lowestNoOfSampleSetsPerMainsCycle{ 0 }; /**< For a mechanism to check the integrity of this code structure */

uint16_t sampleSetsDuringNegativeHalfOfMainsCycle{ 0 }; /**< for arming the triac/trigger */

LoadStates physicalLoadState[NO_OF_DUMPLOADS]; /**< Physical state of the loads */
uint16_t countLoadON[NO_OF_DUMPLOADS]{};       /**< Number of cycle the load was ON (over 1 datalog period) */

uint32_t absenceOfDivertedEnergyCountInMC{ 0 }; /**< number of main cycles without diverted energy */

remove_cv< remove_reference< decltype(DATALOG_PERIOD_IN_MAINS_CYCLES) >::type >::type n_cycleCountForDatalogging{ 0 }; /**< for counting how often datalog is updated */

uint8_t perSecondCounter{ 0 }; /**< for counting  every second inside the ISR */

bool beyondStartUpPeriod{ false }; /**< start-up delay, allows things to settle */

/**
 * @brief Retrieves the output pins configuration.
 *
 * This function determines which pins are configured as output pins
 * based on the current hardware setup. It ensures that no pin is
 * configured multiple times and handles special cases like the watchdog pin.
 *
 * @return A 16-bit value representing the configured output pins.
 *         Returns 0 if an invalid configuration is detected.
 *
 * @note This function is marked as `constexpr` and can be evaluated at compile time.
 *
 * @ingroup Initialization
 */
constexpr uint16_t getOutputPins()
{
  uint16_t output_pins{ 0 };

  for (const auto &loadPin : physicalLoadPin)
  {
    if (bit_read(output_pins, loadPin))
      return 0;

    bit_set(output_pins, loadPin);
  }

  if constexpr (WATCHDOG_PIN_PRESENT)
  {
    if (bit_read(output_pins, watchDogPin))
      return 0;

    bit_set(output_pins, watchDogPin);
  }

  if constexpr (RELAY_DIVERSION)
  {
    for (uint8_t idx = 0; idx < relays.get_size(); ++idx)
    {
      const auto relayPin = relays.get_relay(idx).get_pin();

      if (bit_read(output_pins, relayPin))
        return 0;

      bit_set(output_pins, relayPin);
    }
  }

  return output_pins;
}

/**
 * @brief Retrieves the input pins configuration.
 *
 * This function determines which pins are configured as input pins
 * based on the current hardware setup. It ensures that no pin is
 * configured multiple times and handles special cases like the dual tariff,
 * diversion, rotation, and force pins.
 *
 * @return A 16-bit value representing the configured input pins.
 *         Returns 0 if an invalid configuration is detected.
 *
 * @note This function is marked as `constexpr` and can be evaluated at compile time.
 *
 * @ingroup Initialization
 */
constexpr uint16_t getInputPins()
{
  uint16_t input_pins{ 0 };

  if constexpr (DUAL_TARIFF)
  {
    if (bit_read(input_pins, dualTariffPin))
      return 0;

    bit_set(input_pins, dualTariffPin);
  }

  if constexpr (DIVERSION_PIN_PRESENT)
  {
    if (bit_read(input_pins, diversionPin))
      return 0;

    bit_set(input_pins, diversionPin);
  }

  if constexpr (PRIORITY_ROTATION == RotationModes::PIN)
  {
    if (bit_read(input_pins, rotationPin))
      return 0;

    bit_set(input_pins, rotationPin);
  }

  if constexpr (OVERRIDE_PIN_PRESENT)
  {
    if (bit_read(input_pins, forcePin))
      return 0;

    bit_set(input_pins, forcePin);
  }

  return input_pins;
}

/**
 * @brief Initializes the processing engine, including ports, load states, and ADC setup.
 *
 * This function performs the following tasks:
 * - Initializes the DC offset array for voltage samples.
 * - Configures the input and output pins based on the hardware setup.
 * - Sets up the ADC in free-running mode with interrupts enabled.
 * - Prepares the system for processing energy and load states.
 *
 * @note This function must be called during system initialization to ensure proper operation.
 *
 * @ingroup Initialization
 */
void initializeProcessing()
{
  if constexpr (OLD_PCB)
    initializeOldPCBPins();
  else
  {
    setPinsAsOutput(getOutputPins());      // set the output pins as OUTPUT
    setPinsAsInputPullup(getInputPins());  // set the input pins as INPUT_PULLUP

    for (uint8_t i = 0; i < NO_OF_DUMPLOADS; ++i)
    {
      loadPrioritiesAndState[i] = loadPrioritiesAtStartup[i];
      loadPrioritiesAndState[i] &= loadStateMask;
    }
  }

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
}

/**
 * @brief Updates the control ports for each of the physical loads.
 *
 * This function determines the ON/OFF state of each physical load and updates
 * the corresponding control ports. It ensures that the correct pins are set
 * to their respective states based on the load's current status.
 *
 * @details
 * - If a load is OFF, its corresponding pin is added to the `pinsOFF` mask.
 * - If a load is ON, its corresponding pin is added to the `pinsON` mask.
 * - Finally, the pins are updated using `setPinsOFF` and `setPinsON` functions.
 *
 * @ingroup TimeCritical
 */
void updatePortsStates()
{
  uint16_t pinsON{ 0 };
  uint16_t pinsOFF{ 0 };

  uint8_t i{ NO_OF_DUMPLOADS };

  // On this particular PCB, the trigger has been soldered active high.  This means that the
  // trigger line must be set to LOW to turn the load ON.
  do
  {
    --i;
    // update the local load's state.
    if (LoadStates::LOAD_ON == physicalLoadState[i])
    {
      // setPinOFF(physicalLoadPin[i]);
      pinsOFF |= bit(physicalLoadPin[i]);
    }
    else
    {
      ++countLoadON[i];
      // setPinON(physicalLoadPin[i]);
      pinsON |= bit(physicalLoadPin[i]);
    }
  } while (i);

  setPinsOFF(pinsOFF);
  setPinsON(pinsON);
}

/**
 * @brief This function provides the link between the logical and physical loads.
 *
 * This function maps the logical load states to the physical load states. It ensures
 * that the physical loads are updated according to the logical priorities and states.
 * The function also handles priority rotation if enabled.
 *
 * @details The array, logicalLoadState[], contains the on/off state of all logical loads, with
 *          element 0 being for the one with the highest priority. The array,
 *          physicalLoadState[], contains the on/off state of all physical loads.
 *
 *          The lowest 7 bits of element is the load number as defined in 'physicalLoadState'.
 *          The highest bit of each 'loadPrioritiesAndState' determines if the load is ON or OFF.
 *          The order of each element in 'loadPrioritiesAndState' determines the load priority.
 *          'loadPrioritiesAndState[i] & loadStateMask' will extract the load number at position 'i'
 *          'loadPrioritiesAndState[i] & loadStateOnBit' will extract the load state at position 'i'
 *
 *          Any other mapping relationships could be configured here.
 *
 * @ingroup TimeCritical
 */
void updatePhysicalLoadStates()
{
  if constexpr (PRIORITY_ROTATION != RotationModes::OFF)
  {
    if (Shared::b_reOrderLoads)
    {
      uint8_t i{ NO_OF_DUMPLOADS - 1 };
      const auto temp{ loadPrioritiesAndState[i] };
      do
      {
        loadPrioritiesAndState[i] = loadPrioritiesAndState[i - 1];
        --i;
      } while (i);
      loadPrioritiesAndState[0] = temp;

      Shared::b_reOrderLoads = false;
    }
  }

  const bool bDiversionOff{ Shared::b_diversionOff };
  uint8_t idx{ NO_OF_DUMPLOADS };
  do
  {
    --idx;
    const auto iLoad{ loadPrioritiesAndState[idx] & loadStateMask };
    physicalLoadState[iLoad] = !bDiversionOff && (Shared::b_overrideLoadOn[iLoad] || (loadPrioritiesAndState[idx] & loadStateOnBit)) ? LoadStates::LOAD_ON : LoadStates::LOAD_OFF;
  } while (idx);
}

/**
 * @brief Processes the polarity of the current voltage sample.
 *
 * This function determines the polarity of the voltage sample by removing the DC offset
 * and comparing the adjusted value to zero. The polarity is then stored for use in
 * zero-crossing detection and other processing steps.
 *
 * @details
 * - Removes the DC offset from the raw voltage sample using a low-pass filter (LPF).
 * - Determines the polarity of the adjusted voltage sample (positive or negative).
 * - Updates the `polarityOfMostRecentVsample` variable with the determined polarity.
 *
 * @param rawSample The raw voltage sample from the ADC.
 *
 * @ingroup TimeCritical
 */
void processPolarity(const int16_t rawSample)
{
  // remove DC offset from the raw voltage sample by subtracting the accurate value
  // as determined by a LP filter.
  sampleVminusDC_long = ((long)rawSample << 8) - DCoffset_V_long;
  // determine the polarity of the latest voltage sample
  polarityOfMostRecentVsample = (sampleVminusDC_long > 0) ? Polarities::POSITIVE : Polarities::NEGATIVE;
}

/**
 * @brief Processes the raw current sample for the grid connection point.
 *
 * This function calculates the real power at the grid connection point using the raw
 * current sample from the ADC. It applies a low-pass filter (LPF) to offset the high-pass
 * filter (HPF) effect of the current transformer (CT). The filtered current sample is
 * then used to compute the instantaneous power, which is accumulated for further processing.
 *
 * @details
 * - Removes the DC offset from the raw current sample.
 * - Applies an LPF to counteract the HPF effect of the CT.
 * - Calculates the instantaneous power using the filtered voltage and current samples.
 * - Accumulates the power for the current mains cycle and the datalogging period.
 *
 * @param rawSample The raw current sample from the ADC.
 *
 * @ingroup TimeCritical
 */
void processGridCurrentRawSample(const int16_t rawSample)
{
  // extra items for an LPF to improve the processing of data samples from CT1
  static int32_t lpf_long{};  // new LPF, for offsetting the behaviour of CTx as a HPF

  // First, deal with the power at the grid connection point (as measured via CT1)
  // remove most of the DC offset from the current sample (the precise value does not matter)
  int32_t sampleIminusDC_grid = ((int32_t)(rawSample - DCoffset_I)) << 8;
  //
  // extra filtering to offset the HPF effect of CT1
  const int32_t last_lpf_long = lpf_long;
  lpf_long += alpha * (sampleIminusDC_grid - last_lpf_long);
  sampleIminusDC_grid += (lpf_gain * lpf_long);

  // calculate the "real power" in this sample pair and add to the accumulated sum
  const int32_t filtV_div4 = sampleVminusDC_long >> 2;  // reduce to 16-bits (now x64, or 2^6)
  const int32_t filtI_div4 = sampleIminusDC_grid >> 2;  // reduce to 16-bits (now x64, or 2^6)
  int32_t instP = filtV_div4 * filtI_div4;              // 32-bits (now x4096, or 2^12)
  instP >>= 12;                                         // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_grid += instP;                                   // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  sumP_grid_overDL_Period += instP;
}

/**
 * @brief Processes the raw current sample for the diverted connection point.
 *
 * This function calculates the real power at the diverted connection point using the raw
 * current sample from the ADC. It applies a low-pass filter (LPF) to offset the high-pass
 * filter (HPF) effect of the current transformer (CT). The filtered current sample is
 * then used to compute the instantaneous power, which is accumulated for further processing.
 *
 * @details
 * - Removes the DC offset from the raw current sample.
 * - Calculates the instantaneous power using the filtered voltage and current samples.
 * - Accumulates the power for the current mains cycle and the datalogging period.
 * - Skips processing if the load is overridden.
 *
 * @param rawSample The raw current sample from the ADC.
 *
 * @ingroup TimeCritical
 */
void processDivertedCurrentRawSample(const int16_t rawSample)
{
  if (Shared::b_diversionOff || Shared::b_overrideLoadOn[0])
  {
    return;  // no diverted power when the load is overridden
  }

  // Now deal with the diverted power (as measured via CT2)
  // remove most of the DC offset from the current sample (the precise value does not matter)
  int32_t sampleIminusDC_diverted = ((int32_t)(rawSample - DCoffset_I)) << 8;

  // calculate the "real power" in this sample pair and add to the accumulated sum
  const int32_t filtV_div4 = sampleVminusDC_long >> 2;      // reduce to 16-bits (now x64, or 2^6)
  const int32_t filtI_div4 = sampleIminusDC_diverted >> 2;  // reduce to 16-bits (now x64, or 2^6)
  int32_t instP = filtV_div4 * filtI_div4;                  // 32-bits (now x4096, or 2^12)
  instP >>= 12;                                             // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_diverted += instP;                                   // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  sumP_diverted_overDL_Period += instP;
}

/**
 * @brief Confirms the polarity of the current voltage sample.
 *
 * This function ensures that a zero-crossing point is not declared until a specified
 * number of consecutive samples in the opposite half of the waveform have been encountered.
 * It helps to stabilize polarity detection and avoid false zero-crossing detections.
 *
 * @details
 * - Compares the polarity of the most recent voltage sample with the last confirmed polarity.
 * - If the polarities match, the counter is reset.
 * - If the polarities differ, the counter is incremented.
 * - Once the counter exceeds the persistence threshold (`PERSISTENCE_FOR_POLARITY_CHANGE`),
 *   the polarity is confirmed and updated.
 *
 * @ingroup TimeCritical
 */
void confirmPolarity()
{
  /* This routine prevents a zero-crossing point from being declared until 
   * a certain number of consecutive samples in the 'other' half of the 
   * waveform have been encountered.  
   */
  static uint8_t count{ 0 };
  if (polarityOfMostRecentVsample == polarityConfirmedOfLastSampleV)
  {
    count = 0;
    return;
  }

  if (++count > PERSISTENCE_FOR_POLARITY_CHANGE)
  {
    count = 0;
    polarityConfirmed = polarityOfMostRecentVsample;
  }
}

/**
 * @brief This routine is called by the ISR when a pair of V & I samples becomes available.
 *
 * This function processes raw voltage and current samples to determine the energy state
 * and manage load control. It handles zero-crossing detection, energy bucket updates,
 * and load state adjustments based on the energy level.
 *
 * @details
 * - Detects the start of a new positive or negative half-cycle based on polarity.
 * - Processes energy contributions for each half-cycle.
 * - Updates the energy bucket and predicts the energy state at the end of the cycle.
 * - Adjusts logical and physical load states to maintain energy levels within thresholds.
 * - Updates the control ports for physical loads.
 * - Handles energy diversion display updates and ensures stability during startup.
 *
 * @ingroup TimeCritical
 */
void processRawSamples()
{
  if (polarityConfirmed == Polarities::POSITIVE)
  {
    if (polarityConfirmedOfLastSampleV != Polarities::POSITIVE)
    {
      // This is the start of a new +ve half cycle (just after the zero-crossing point)
      if (beyondStartUpPeriod)
      {
        processPlusHalfCycle();

        processStartNewCycle();
      }
      else
      {
        processStartUp();
      }
    }  // end of processing that is specific to the first Vsample in each +ve half cycle

    // still processing samples where the voltage is Polarities::POSITIVE ...
    // (in this go-faster code, the action from here has moved to the negative half of the cycle)

  }  // end of processing that is specific to samples where the voltage is positive
  else  // the polarity of this sample is negative
  {
    if (polarityConfirmedOfLastSampleV != Polarities::NEGATIVE)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)
      processMinusHalfCycle();
    }

    // check to see whether the trigger device can now be reliably armed
    if (sampleSetsDuringNegativeHalfOfMainsCycle == POST_ZERO_CROSSING_MAX_COUNT)
    {
      if (beyondStartUpPeriod)
      {
        /* Determining whether any of the loads need to be changed is is a 3-stage process:
         * - change the LOGICAL load states as necessary to maintain the energy level
         * - update the PHYSICAL load states according to the logical -> physical mapping 
         * - update the driver lines for each of the loads.
         */

        // Restrictions apply for the period immediately after a load has been switched.
        // Here the recentTransition flag is checked and updated as necessary.
        if (recentTransition)
        {
          if (++postTransitionCount == POST_TRANSITION_MAX_COUNT)
          {
            recentTransition = false;
          }
        }

        if (energyInBucket_prediction > midPointOfEnergyBucket_long)
        {
          // the energy state is in the upper half of the working range
          lowerEnergyThreshold = lowerThreshold_default;  // reset the "opposite" threshold
          if (energyInBucket_prediction > upperEnergyThreshold)
          {
            // Because the energy level is high, some action may be required
            proceedHighEnergyLevel();
          }
        }
        else
        {                                                 // the energy state is in the lower half of the working range
          upperEnergyThreshold = upperThreshold_default;  // reset the "opposite" threshold
          if (energyInBucket_prediction < lowerEnergyThreshold)
          {
            // Because the energy level is low, some action may be required
            proceedLowEnergyLevel();
          }
        }

        updatePhysicalLoadStates();  // allows the logical-to-physical mapping to be changed

        // update each of the physical loads
        updatePortsStates();

        // update the Energy Diversion Detector
        if (loadPrioritiesAndState[0] & loadStateOnBit)
        {
          absenceOfDivertedEnergyCountInMC = 0;
          Shared::EDD_isActive = true;
        }
        else
        {
          ++absenceOfDivertedEnergyCountInMC;
        }

        // Now that the energy-related decisions have been taken, min and max limits can now
        // be applied  to the level of the energy bucket.  This is to ensure correct operation
        // when conditions change, i.e. when import changes to export, and vice versa.
        //
        if (energyInBucket_long > capacityOfEnergyBucket_long)
        {
          energyInBucket_long = capacityOfEnergyBucket_long;
        }
        else if (energyInBucket_long < 0)
        {
          energyInBucket_long = 0;
        }
      }
    }

    ++sampleSetsDuringNegativeHalfOfMainsCycle;
  }  // end of processing that is specific to samples where the voltage is negative
  refresh7SegDisplay();
}

/**
 * @brief Process the calculation for the current voltage sample.
 *
 * This function calculates the squared voltage value for the current sample
 * and accumulates it for further processing. It also updates the DC offset
 * and polarity information for the next cycle.
 *
 * @details
 * - The voltage squared (V²) is calculated and accumulated for RMS calculations.
 * - The low-pass filter is updated to remove the DC offset from the voltage signal.
 * - The polarity of the last confirmed sample is stored for zero-crossing detection.
 * - The number of samples during the current mains cycle is incremented.
 *
 * @ingroup TimeCritical
 */
void processVoltage()
{
  long filtV_div4 = sampleVminusDC_long >> 2;        // reduce to 16-bits (now x64, or 2^6)
  int32_t inst_Vsquared{ filtV_div4 * filtV_div4 };  // 32-bits (now x4096, or 2^12)

  inst_Vsquared >>= 12;  // scaling is now x1 (V_ADC x I_ADC)

  l_sum_Vsquared += inst_Vsquared;  // cumulative V^2 (V_ADC x I_ADC)

  // store items for use during next loop
  cumVdeltasThisCycle_long += sampleVminusDC_long;     // for use with LP filter
  polarityConfirmedOfLastSampleV = polarityConfirmed;  // for identification of half cycle boundaries
}

/**
 * @brief Process the current voltage raw sample.
 *
 * This function processes the raw voltage sample by performing the following steps:
 *
 * @details
 * - Determines the polarity of the raw voltage sample using `processPolarity()`.
 * - Confirms the polarity using `confirmPolarity()` to ensure stability in zero-crossing detection.
 * - Processes raw samples for energy state and load control using `processRawSamples()`.
 * - Calculates the squared voltage value and accumulates it for RMS calculations using `processVoltage()`.
 * - Increments the sample set counter for the current datalogging period.
 *
 * @param rawSample The raw voltage sample from the ADC.
 *
 * @ingroup TimeCritical
 */
void processVoltageRawSample(const int16_t rawSample)
{
  processPolarity(rawSample);
  confirmPolarity();

  processRawSamples();  // deals with aspects that only occur at particular stages of each mains cycle

  // processing for EVERY set of samples
  //
  processVoltage();

  ++sampleSetsDuringThisMainsCycle;
  ++sampleSetsDuringThisDatalogPeriod;
}

/**
 * @brief Process the startup period for the router.
 *
 * This function ensures that the system remains in a stable state during the startup period.
 * It waits for the DC-blocking filters to settle and initializes key variables for energy
 * and load management. Once the startup period is complete, the system transitions to normal
 * operation.
 *
 * @details
 * - Waits until the startup delay has elapsed, allowing filters to stabilize.
 * - Resets energy and power accumulators for grid and diverted power.
 * - Initializes counters for sample sets and load states.
 * - Marks the end of the startup period by setting `beyondStartUpPeriod` to true.
 *
 * @ingroup TimeCritical
 */
void processStartUp()
{
  // wait until the DC-blocking filters have had time to settle
  if (millis() <= (delayBeforeSerialStarts + startUpPeriod))
  {
    return;
  }

  beyondStartUpPeriod = true;
  sumP_grid = 0;
  sumP_grid_overDL_Period = 0;
  sumP_diverted = 0;
  sumP_diverted_overDL_Period = 0;
  sampleSetsDuringThisMainsCycle = 0;  // not yet dealt with for this cycle
  lowestNoOfSampleSetsPerMainsCycle = UINT8_MAX;
  // can't say "Go!" here 'cos we're in an ISR!
}

/**
 * @brief This code is executed once per 20ms, shortly after the start of each new
 *        mains cycle on phase 0.
 *
 * This function manages the energy bucket and ensures that its level remains within
 * the defined limits. It is responsible for maintaining system stability when conditions
 * change, such as when energy import switches to export or vice versa.
 *
 * @details
 * - Applies maximum and minimum limits to the energy bucket's level.
 * - Ensures correct operation during transitions between energy import and export states.
 *
 * @ingroup TimeCritical
 */
void processStartNewCycle()
{
  // Apply max and min limits to bucket's level.  This is to ensure correct operation
  // when conditions change, i.e. when import changes to export, and vice versa.
  //
  if (energyInBucket_long > capacityOfEnergyBucket_long)
  {
    energyInBucket_long = capacityOfEnergyBucket_long;
  }
  else if (energyInBucket_long < 0)
  {
    energyInBucket_long = 0;
  }

  // clear the per-cycle accumulators for use in this new mains cycle.
  sampleSetsDuringThisMainsCycle = 0;
  sumP_grid = 0;
  sumP_diverted = 0;
  sampleSetsDuringNegativeHalfOfMainsCycle = 0;
}

/**
 * @brief Process the start of a new +ve half cycle, just after the zero-crossing point.
 *
 * This function handles the processing required at the start of a new positive half-cycle
 * of the mains waveform. It updates performance metrics, processes the latest energy
 * contributions, and resets accumulators for the new cycle.
 *
 * @details
 * - Updates the lowest number of sample sets per mains cycle for performance monitoring.
 * - Processes the latest energy contributions using `processLatestContribution()`.
 * - Handles data logging for the current cycle using `processDataLogging()`.
 * - Resets per-cycle accumulators for energy and sample sets.
 *
 * @ingroup TimeCritical
 */
void processPlusHalfCycle()
{
  // a simple routine for checking the performance of this new ISR structure
  if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle)
  {
    lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;
  }

  processLatestContribution();

  processDataLogging();
}

/**
 * @brief Process the start of a new -ve half cycle, just after the zero-crossing point.
 *
 * This function handles the processing required at the start of a new negative half-cycle
 * of the mains waveform. It updates the low-pass filter (LPF) for DC offset removal,
 * ensures the LPF output remains within valid limits, and predicts the energy state
 * at the end of the mains cycle.
 *
 * @details
 * - Updates the low-pass filter (LPF) for DC offset removal using the cumulative voltage deltas.
 * - Ensures the LPF output remains within the valid range to avoid signal drift.
 * - Calculates the average power during the first half of the mains cycle.
 * - Predicts the energy state at the end of the mains cycle based on the average power.
 *
 * @ingroup TimeCritical
 */
void processMinusHalfCycle()
{
  // This is the start of a new -ve half cycle (just after the zero-crossing point)
  // which is a convenient point to update the Low Pass Filter for DC-offset removal
  //  The portion which is fed back into the integrator is approximately one percent
  // of the average offset of all the Vsamples in the previous mains cycle.
  //
  const auto previousOffset{ DCoffset_V_long };
  DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long >> 12);
  cumVdeltasThisCycle_long = 0;

  // To ensure that the LPF will always start up correctly when 240V AC is available, its
  // output value needs to be prevented from drifting beyond the likely range of the
  // voltage signal.  This avoids the need to use a HPF as was done for initial Mk2 builds.
  //
  if (DCoffset_V_long < DCoffset_V_min)
  {
    DCoffset_V_long = DCoffset_V_min;
  }
  else if (DCoffset_V_long > DCoffset_V_max)
  {
    DCoffset_V_long = DCoffset_V_max;
  }

  // The average power that has been measured during the first half of this mains cycle can now be used
  // to predict the energy state at the end of this mains cycle.  That prediction will be used to alter
  // the state of the load as necessary. The arming signal for the triac can't be set yet - that must
  // wait until the voltage has advanced further beyond the -ve going zero-crossing point.
  //
  long averagePower = sumP_grid / sampleSetsDuringThisMainsCycle;  // for 1st half of this mains cycle only
  //
  // To avoid repetitive and unnecessary calculations, the increase in energy during each mains cycle is
  // deemed to be numerically equal to the average power.  The predicted value for the energy state at the
  // end of this mains cycle will therefore be the known energy state at its start plus the average power
  // as measured. Although the average power has been determined over only half a mains cycle, the correct
  // number of contributing sample sets has been used so the result can be expected to be a true measurement
  // of average power, not half of it.
  //
  energyInBucket_prediction = energyInBucket_long + averagePower;  // at end of this mains cycle
}

/**
 * @brief Process the latest contribution after each new cycle.
 *
 * This function calculates the real power and energy during the last whole mains cycle.
 * It updates the energy bucket and prepares the system for the next cycle.
 *
 * @details
 * - Calculates the average real power for the grid and diverted connections.
 * - Adjusts the grid power to account for required export (useful for PV simulation).
 * - Converts power to energy and updates the energy bucket.
 * - Marks the start of a new cycle for the main code.
 *
 * @ingroup TimeCritical
 */
void processLatestContribution()
{
  // Calculate the real power and energy during the last whole mains cycle.
  //
  // sumP contains the sum of many individual calculations of instantaneous power.  In
  // order to obtain the average power during the relevant period, sumP must first be
  // divided by the number of samples that have contributed to its value.
  //
  // The next stage would normally be to apply a calibration factor so that real power
  // can be expressed in Watts.  That's fine for floating point maths, but it's not such
  // a good idea when integer maths is being used.  To keep the numbers large, and also
  // to save time, calibration of power is omitted at this stage.  Real Power (stored as
  // a 'long') is therefore (1/powerCal) times larger than the actual power in Watts.
  //
  int32_t realPower_grid = sumP_grid / sampleSetsDuringThisMainsCycle;          // proportional to Watts
  int32_t realPower_diverted = sumP_diverted / sampleSetsDuringThisMainsCycle;  // proportional to Watts

  realPower_grid -= requiredExportPerMainsCycle_inIEU;  // <- useful for PV simulation

  // Next, the energy content of this power rating needs to be determined.  Energy is
  // power multiplied by time, so the next step would normally be to multiply the measured
  // value of power by the time over which it was measured.
  //   Instantaneous power is calculated once every mains cycle. When integer maths is
  // being used, a repetitive power-to-energy conversion seems an unnecessary workload.
  // As all sampling periods are of similar duration, it is more efficient to just
  // add all of the power samples together, and note that their sum is actually
  // SUPPLY_FREQUENCY greater than it would otherwise be.
  //   Although the numerical value itself does not change, I thought that a new name
  // may be helpful so as to minimise confusion.
  //   The 'energy' variable below is SUPPLY_FREQUENCY * (1/powerCal) times larger than
  // the actual energy in Joules.
  //
  realEnergy_grid = realPower_grid;
  realEnergy_diverted = realPower_diverted;

  // Energy contributions from the grid connection point (CT1) are summed in an
  // accumulator which is known as the energy bucket.  The purpose of the energy bucket
  // is to mimic the operation of the supply meter.  The range over which energy can
  // pass to and fro without loss or charge to the user is known as its 'sweet-zone'.
  // The capacity of the energy bucket is set to this same value within setup().
  //
  // The latest contribution can now be added to this energy bucket
  energyInBucket_long += realEnergy_grid;

  if (Shared::EDD_isActive)  // Energy Diversion Display
  {
    // For diverted energy, the latest contribution needs to be added to an
    // accumulator which operates with maximum precision.

    if (realEnergy_diverted < antiCreepLimit_inIEUperMainsCycle)
    {
      realEnergy_diverted = 0;
    }
    divertedEnergyRecent_IEU += realEnergy_diverted;

    // Whole kWh are then recorded separately
    if (divertedEnergyRecent_IEU > IEU_per_Wh_diverted)
    {
      divertedEnergyRecent_IEU -= IEU_per_Wh_diverted;
      ++divertedEnergyTotal_Wh;
    }
  }

  // After a pre-defined period of inactivity, the 4-digit display needs to
  // close down in readiness for the next's day's data.
  //
  if (absenceOfDivertedEnergyCountInMC > displayShutdown_inMainsCycles)
  {
    // clear the accumulators for diverted energy
    divertedEnergyTotal_Wh = 0;
    divertedEnergyRecent_IEU = 0;
    Shared::EDD_isActive = false;  // energy diversion detector is now inactive
  }

  if (++perSecondCounter == SUPPLY_FREQUENCY)
  {
    perSecondCounter = 0;

    if (absenceOfDivertedEnergyCountInMC > SUPPLY_FREQUENCY)
      ++Shared::absenceOfDivertedEnergyCountInSeconds;
    else
      Shared::absenceOfDivertedEnergyCountInSeconds = 0;

    // The diverted energy total is copied to a variable before it is used.
    // This is done to avoid the possibility of a race-condition whereby the
    // diverted energy total is updated while the display is being updated.
    Shared::copyOf_divertedEnergyTotal_Wh = divertedEnergyTotal_Wh;
  }

  Shared::b_newCycle = true;  //  a 50 Hz 'tick' for use by the main code
}

/**
 * @brief Process the case of high energy level, some action may be required.
 *
 * This function determines whether additional loads can be switched ON when the energy
 * level in the bucket exceeds the upper threshold. It ensures that the energy thresholds
 * remain within valid limits and handles load activation during the post-transition period.
 *
 * @details
 * - Identifies the next logical load to be added.
 * - Updates the upper energy threshold during the post-transition period.
 * - Ensures only the active load can be switched during the post-transition period.
 * - Activates the identified load and updates the transition state.
 *
 * @ingroup TimeCritical
 */
void proceedHighEnergyLevel()
{
  bool bOK_toAddLoad{ true };
  const auto tempLoad{ nextLogicalLoadToBeAdded() };

  if (tempLoad == NO_OF_DUMPLOADS)
  {
    return;
  }

  // a load which is now OFF has been identified for potentially being switched ON
  if (recentTransition)
  {
    // During the post-transition period, any increase in the energy level is noted.
    upperEnergyThreshold = energyInBucket_prediction;

    // the energy thresholds must remain within range
    if (upperEnergyThreshold > capacityOfEnergyBucket_long)
    {
      upperEnergyThreshold = capacityOfEnergyBucket_long;
    }

    // Only the active load may be switched during this period.  All other loads must
    // wait until the recent transition has had sufficient opportunity to take effect.
    bOK_toAddLoad = (tempLoad == activeLoad);
  }

  if (bOK_toAddLoad)
  {
    loadPrioritiesAndState[tempLoad] |= loadStateOnBit;
    activeLoad = tempLoad;
    postTransitionCount = 0;
    recentTransition = true;
  }
}

/**
 * @brief Process the case of low energy level, some action may be required.
 *
 * This function determines whether loads can be switched OFF when the energy
 * level in the bucket falls below the lower threshold. It ensures that the energy
 * thresholds remain within valid limits and handles load deactivation during the
 * post-transition period.
 *
 * @details
 * - Identifies the next logical load to be removed.
 * - Updates the lower energy threshold during the post-transition period.
 * - Ensures only the active load can be switched during the post-transition period.
 * - Deactivates the identified load and updates the transition state.
 *
 * @ingroup TimeCritical
 */
void proceedLowEnergyLevel()
{
  bool bOK_toRemoveLoad{ true };
  const auto tempLoad{ nextLogicalLoadToBeRemoved() };

  if (tempLoad >= NO_OF_DUMPLOADS)
  {
    return;
  }

  // a load which is now ON has been identified for potentially being switched OFF
  if (recentTransition)
  {
    // During the post-transition period, any decrease in the energy level is noted.
    lowerEnergyThreshold = energyInBucket_prediction;

    // the energy thresholds must remain within range
    if (lowerEnergyThreshold < 0)
    {
      lowerEnergyThreshold = 0;
    }

    // Only the active load may be switched during this period.  All other loads must
    // wait until the recent transition has had sufficient opportunity to take effect.
    bOK_toRemoveLoad = (tempLoad == activeLoad);
  }

  if (bOK_toRemoveLoad)
  {
    loadPrioritiesAndState[tempLoad] &= loadStateMask;
    activeLoad = tempLoad;
    postTransitionCount = 0;
    recentTransition = true;
  }
}

/**
 * @brief Retrieve the next load that could be removed (be aware of the reverse-order).
 *
 * This function identifies the next logical load that can be switched OFF based on
 * the reverse order of load priorities. It ensures that the highest-priority load
 * remains active while deactivating lower-priority loads as needed.
 *
 * @return The load number if successful, NO_OF_DUMPLOADS in case of failure.
 *
 * @details
 * - Iterates through the load priorities in reverse order.
 * - Checks if a load is currently ON and eligible for deactivation.
 * - Returns the load number if a suitable load is found.
 * - Returns NO_OF_DUMPLOADS if no load can be deactivated.
 *
 * @ingroup TimeCritical
 */
uint8_t nextLogicalLoadToBeAdded()
{
  for (uint8_t index = 0; index < NO_OF_DUMPLOADS; ++index)
  {
    if (!(loadPrioritiesAndState[index] & loadStateOnBit))
    {
      return (index);
    }
  }

  return (NO_OF_DUMPLOADS);
}

/**
 * @brief Process with data logging.
 *
 * At the end of each datalogging period, this function copies the relevant variables
 * for use by the main code. These variables are then reset for the next datalogging period.
 *
 * @details
 * - Checks if the datalogging period has elapsed.
 * - Copies accumulated power, energy, and sample data for the current period.
 * - Resets the accumulators for the next datalogging period.
 * - Signals the main processor that new logging data are available.
 *
 * @ingroup TimeCritical
 */
uint8_t nextLogicalLoadToBeRemoved()
{
  uint8_t index{ NO_OF_DUMPLOADS };

  do
  {
    if (loadPrioritiesAndState[--index] & loadStateOnBit)
    {
      return (index);
    }
  } while (index);

  return (NO_OF_DUMPLOADS);
}

/**
 * @brief Process with data logging.
 *
 * At the end of each datalogging period, this function copies the relevant variables
 * for use by the main code. These variables are then reset for the next datalogging period.
 *
 * @details
 * - Checks if the datalogging period has elapsed.
 * - Copies accumulated power, energy, and sample data for the current period.
 * - Resets the accumulators for the next datalogging period.
 * - Signals the main processor that new logging data are available.
 *
 * @ingroup TimeCritical
 */
void processDataLogging()
{
  if (++n_cycleCountForDatalogging < DATALOG_PERIOD_IN_MAINS_CYCLES)
  {
    return;  // data logging period not yet reached
  }

  n_cycleCountForDatalogging = 0;

  Shared::copyOf_sumP_grid_overDL_Period = sumP_grid_overDL_Period;
  sumP_grid_overDL_Period = 0;

  Shared::copyOf_sumP_diverted_overDL_Period = sumP_diverted_overDL_Period;
  sumP_diverted_overDL_Period = 0;

  Shared::copyOf_divertedEnergyTotal_Wh_forDL = divertedEnergyTotal_Wh;

  Shared::copyOf_sum_Vsquared = l_sum_Vsquared;
  l_sum_Vsquared = 0;

  uint8_t i{ NO_OF_DUMPLOADS };
  do
  {
    --i;
    Shared::copyOf_countLoadON[i] = countLoadON[i];
    countLoadON[i] = 0;
  } while (i);

  Shared::copyOf_sampleSetsDuringThisDatalogPeriod = sampleSetsDuringThisDatalogPeriod;  // (for diags only)
  Shared::copyOf_lowestNoOfSampleSetsPerMainsCycle = lowestNoOfSampleSetsPerMainsCycle;  // (for diags only)
  Shared::copyOf_energyInBucket_long = energyInBucket_long;                              // (for diags only)

  lowestNoOfSampleSetsPerMainsCycle = UINT8_MAX;
  sampleSetsDuringThisDatalogPeriod = 0;

  // signal the main processor that logging data are available
  // we skip the period from start to running stable
  Shared::b_datalogEventPending = beyondStartUpPeriod;
}

/**
 * @brief Print the settings used for the selected output mode.
 *
 * This function outputs the configuration parameters related to the selected output mode.
 * It is primarily used for debugging and verifying the system's configuration.
 *
 * @details
 * - Prints the zero-crossing persistence value in sample sets.
 * - Prints the capacity of the energy bucket in integer energy units.
 *
 * @ingroup Debugging
 */
void printParamsForSelectedOutputMode()
{
  DBUG("\tzero-crossing persistence (sample sets) = ");
  DBUGLN(PERSISTENCE_FOR_POLARITY_CHANGE);

  DBUG("\tcapacityOfEnergyBucket_long = ");
  DBUGLN(capacityOfEnergyBucket_long);
}

/**
 * @brief Interrupt Service Routine - Interrupt-Driven Analog Conversion.
 *
 * This ISR is triggered whenever an ADC conversion is completed, approximately every 104 µs
 * in free-running mode. It processes the results of the completed conversion and sets up
 * the next conversion in sequence. The ISR handles voltage and current samples for the grid
 * and diverted connections, ensuring real-time processing of energy data.
 *
 * @details 
 * An Interrupt Service Routine is now defined which instructs the ADC to perform a conversion
 * for each of the voltage and current sensors in turn.
 *
 * This Interrupt Service Routine is for use when the ADC is in the free-running mode.
 * It is executed whenever an ADC conversion has finished, approx every 104 µs. In
 * free-running mode, the ADC has already started its next conversion by the time that
 * the ISR is executed. The ISR therefore needs to "look ahead".
 *
 * At the end of conversion Type N, conversion Type N+1 will start automatically. The ISR
 * which runs at this point therefore needs to capture the results of conversion Type N,
 * and set up the conditions for conversion Type N+2, and so on.
 *
 * By means of various helper functions, all of the time-critical activities are processed
 * within the ISR.
 *
 * The main code is notified by means of a flag when fresh copies of loggable data are available.
 *
 * @note
 * Guidelines for writing an ISR:
 * - Keep it short and efficient.
 * - Avoid using delay() or serial prints.
 * - Use `volatile` for shared variables.
 * - Protect shared variables with critical sections if necessary.
 * - Avoid enabling or disabling interrupts within the ISR.
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
      rawSample = ADC;  // store the ADC value (this one is for Voltage)
      //sampleV = ADC;                                // store the ADC value (this one is for Voltage)
      ADMUX = bit(REFS0) + currentSensor_diverted;  // set up the next conversion, which is for Diverted Current
      ++sample_index;                               // increment the control flag
      //
      processVoltageRawSample(rawSample);
      break;
    case 1:
      rawSample = ADC;  // store the ADC value (this one is for current at CT1)
      //sampleI_diverted_raw = ADC;               // store the ADC value (this one is for Diverted Current)
      ADMUX = bit(REFS0) + voltageSensor;  // set up the next conversion, which is for Grid Current
      ++sample_index;                      // increment the control flag
      //
      processGridCurrentRawSample(rawSample);
      break;
    case 2:
      rawSample = ADC;  // store the ADC value (this one is for current at CT2)
      //sampleI_grid_raw = ADC;              // store the ADC value (this one is for Grid Current)
      ADMUX = bit(REFS0) + currentSensor_grid;  // set up the next conversion, which is for Voltage
      sample_index = 0;                         // reset the control flag
      //
      processDivertedCurrentRawSample(rawSample);
      break;
    default:
      sample_index = 0;  // to prevent lockup (should never get here)
  }
}

/**
 * @brief Initializes optional pins for the old PCB configuration.
 *
 * This function configures various optional pins based on the hardware setup and
 * feature flags. It ensures that the pins are properly initialized for their respective
 * purposes, such as dual tariff, override, priority rotation, diversion, relay diversion,
 * and watchdog functionality.
 *
 * @warning Ensure that the pin assignments are valid for the selected PCB configuration.
 *
 * @details
 * - Configures the dual tariff pin as an input with an internal pull-up resistor.
 * - Configures the override pin as an input with an internal pull-up resistor.
 * - Configures the priority rotation pin as an input with an internal pull-up resistor.
 * - Configures the diversion pin as an input with an internal pull-up resistor.
 * - Initializes relay diversion pins if enabled.
 * - Configures the watchdog pin as an output and sets it to OFF.
 *
 * @ingroup Initialization
 */
void initializeOldPCBPins()
{
  for (int16_t i = 0; i < NO_OF_DUMPLOADS; ++i)
  {
    loadPrioritiesAndState[i] = loadPrioritiesAtStartup[i];
    pinMode(physicalLoadPin[i], OUTPUT);  // driver pin for Load #n
    loadPrioritiesAndState[i] &= loadStateMask;
  }

  updatePhysicalLoadStates();  // allows the logical-to-physical mapping to be changed

  updatePortsStates();  // updates output pin states

  if constexpr (DUAL_TARIFF)
  {
    pinMode(dualTariffPin, INPUT_PULLUP);  // set as input & enable the internal pullup resistor
    delay(100);                            // allow time to settle
  }

  if constexpr (OVERRIDE_PIN_PRESENT)
  {
    pinMode(forcePin, INPUT_PULLUP);  // set as input & enable the internal pullup resistor
    delay(100);                       // allow time to settle
  }

  if constexpr (PRIORITY_ROTATION == RotationModes::PIN)
  {
    pinMode(rotationPin, INPUT_PULLUP);  // set as input & enable the internal pullup resistor
    delay(100);                          // allow time to settle
  }

  if constexpr (DIVERSION_PIN_PRESENT)
  {
    pinMode(diversionPin, INPUT_PULLUP);  // set as input & enable the internal pullup resistor
    delay(100);                           // allow time to settle
  }

  if constexpr (RELAY_DIVERSION)
  {
    for (uint8_t idx = 0; idx < relays.get_size(); ++idx)
    {
      const auto relayPin = relays.get_relay(idx).get_pin();

      pinMode(relayPin, OUTPUT);
      setPinOFF(relayPin);  // set to off
    }
  }

  if constexpr (WATCHDOG_PIN_PRESENT)
  {
    pinMode(watchDogPin, OUTPUT);  // set as output
    setPinOFF(watchDogPin);        // set to off
  }
}

/**
 * @brief Prints the load priorities to the Serial output.
 *
 * This function logs the current load priorities and states to the Serial output
 * for debugging purposes. It provides a detailed view of the load configuration
 * and their respective priorities.
 *
 * @details
 * - Each load's priority and state are printed in a human-readable format.
 * - This function is only active when debugging is enabled (`ENABLE_DEBUG`).
 *
 * @ingroup GeneralProcessing
 */
void logLoadPriorities()
{
#ifdef ENABLE_DEBUG

  DBUGLN(F("Load Priorities: "));
  for (const auto &loadPrioAndState : loadPrioritiesAndState)
  {
    DBUG(F("\tload "));
    DBUGLN(loadPrioAndState);
  }

#endif
}

static_assert(IEU_per_Wh_diverted > 4000000, "IEU_per_Wh_diverted calculation is incorrect");
