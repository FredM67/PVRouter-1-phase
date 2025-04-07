
#include <Arduino.h>

#include "calibration.h"
#include "dualtariff.h"
#include "processing.h"
#include "utils_pins.h"

// Define operating limits for the LP filters which identify DC offset in the voltage
// sample streams. By limiting the output range, these filters always should start up
// correctly.
constexpr int32_t DCoffset_V_min{ (512L - 100) * 256 }; /**< mid-point of ADC minus a working margin */
constexpr int32_t DCoffset_V_max{ (512L + 100) * 256 }; /**< mid-point of ADC plus a working margin */
constexpr int16_t DCoffset_I{ 512 };                    /**< nominal mid-point value of ADC @ x1 scale */

int32_t DCoffset_V_long{ 512L * 256 }; /**< <--- for LPF */

/**< main energy bucket for single-phase use, with units of Joules * SUPPLY_FREQUENCY */
constexpr int32_t capacityOfEnergyBucket_long{ static_cast< int32_t >(WORKING_ZONE_IN_JOULES * SUPPLY_FREQUENCY * (1 / powerCal_grid)) };  // depends on powerCal, frequency & the 'sweetzone' size.
/**< for resetting flexible thresholds */
constexpr int32_t midPointOfEnergyBucket_long{ capacityOfEnergyBucket_long >> 1 };  // used for 'normal' and single-threshold 'AF' logic

constexpr int32_t lowerThreshold_default{ capacityOfEnergyBucket_long >> 1 };
constexpr int32_t upperThreshold_default{ capacityOfEnergyBucket_long >> 1 };

// to avoid the diverted energy accumulator 'creeping' when the load is not active
constexpr int32_t antiCreepLimit_inIEUperMainsCycle{ static_cast< int32_t >(ANTI_CREEP_LIMIT * (1 / powerCal_grid)) };

constexpr int32_t requiredExportPerMainsCycle_inIEU{ static_cast< int32_t >(REQUIRED_EXPORT_IN_WATTS * (1 / powerCal_grid)) };

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
int32_t energyInBucket_long{ 0 }; /**< in Integer Energy Units */
int32_t lowerEnergyThreshold;     /**< dynamic lower threshold */
int32_t upperEnergyThreshold;     /**< dynamic upper threshold */

// For recording the accumulated amount of diverted energy data (using CT2), a similar
// calibration mechanism is required.  Rather than a bucket with a fixed capacity, the
// accumulator for diverted energy just needs to be scaled correctly.  As soon as its
// value exceeds 1 Wh, an associated WattHour register is incremented, and the
// accumulator's value is decremented accordingly. The calculation below is to determine
// the scaling for this accumulator.

constexpr int32_t IEU_per_Wh{ static_cast< int32_t >(JOULES_PER_WATT_HOUR * SUPPLY_FREQUENCY * (1 / powerCal_diverted)) };  // depends on powerCal, frequency & the 'sweetzone' size.

bool recentTransition{ false };                   /**< a load state has been recently toggled */
uint8_t postTransitionCount;                      /**< counts the number of cycle since last transition */
constexpr uint8_t POST_TRANSITION_MAX_COUNT{ 3 }; /**< allows each transition to take effect */
uint8_t activeLoad{ 0 };                          /**< current active load */

int32_t sumP_grid;                   /**< for per-cycle summation of 'real power' */
int32_t sumP_grid_overDL_Period;     /**< for per-cycle summation of 'real power' during datalog period */
int32_t sumP_diverted;               /**< for per-cycle summation of 'real power' */
int32_t sumP_diverted_overDL_Period; /**< for per-cycle summation of 'real power' during datalog period */
int32_t cumVdeltasThisCycle_long;    /**< for the LPF which determines DC offset (voltage) */
int32_t l_sum_Vsquared;              /**< for summation of V^2 values during datalog period */

int32_t realEnergy_grid{ 0 };
int32_t realEnergy_diverted{ 0 };
int32_t energyInBucket_prediction{ 0 };
int32_t sampleVminusDC_long{ 0 };

Polarities polarityOfMostRecentVsample;    /**< for zero-crossing detection */
Polarities polarityConfirmed;              /**< for zero-crossing detection */
Polarities polarityConfirmedOfLastSampleV; /**< for zero-crossing detection */

// For a mechanism to check the continuity of the sampling sequence
uint8_t sampleSetsDuringThisMainsCycle;     /**< number of sample sets during each mains cycle */
uint16_t sampleSetsDuringThisDatalogPeriod; /**< number of sample sets during each datalogging period */

uint8_t lowestNoOfSampleSetsPerMainsCycle; /**< For a mechanism to check the integrity of this code structure */

uint16_t sampleSetsDuringNegativeHalfOfMainsCycle{ 0 }; /**< for arming the triac/trigger */

LoadStates physicalLoadState[NO_OF_DUMPLOADS]; /**< Physical state of the loads */
uint16_t countLoadON[NO_OF_DUMPLOADS];         /**< Number of cycle the load was ON (over 1 datalog period) */

remove_cv< remove_reference< decltype(DATALOG_PERIOD_IN_MAINS_CYCLES) >::type >::type n_cycleCountForDatalogging{ 0 }; /**< for counting how often datalog is updated */

bool beyondStartUpPeriod{ false }; /**< start-up delay, allows things to settle */

/**
 * @brief Initializes the ports and load states for processing
 *
 */
void initializeProcessing()
{
  for (int16_t i = 0; i < NO_OF_DUMPLOADS; ++i)
  {
    loadPrioritiesAndState[i] = loadPrioritiesAtStartup[i];
    pinMode(physicalLoadPin[i], OUTPUT);  // driver pin for Load #n
    loadPrioritiesAndState[i] &= loadStateMask;
  }

  updatePhysicalLoadStates();  // allows the logical-to-physical mapping to be changed

  updatePortsStates();  // updates output pin states

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
 * @brief Initializes the optional pins
 *
 */
void initializeOptionalPins()
{
  if constexpr (DUAL_TARIFF)
  {
    pinMode(dualTariffPin, INPUT_PULLUP);  // set as input & enable the internal pullup resistor
    delay(100);                            // allow time to settle

    //ul_TimeOffPeak = millis();
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
    relays.initializePins();
  }

  if constexpr (WATCHDOG_PIN_PRESENT)
  {
    pinMode(watchDogPin, OUTPUT);  // set as output
    setPinOFF(watchDogPin);        // set to off
  }
}

#if !defined(__DOXYGEN__)
void updatePortsStates() __attribute__((optimize("-O3")));
#endif
/**
 * @brief update the control ports for each of the physical loads
 *
 */
void updatePortsStates()
{
  uint16_t pinsON{ 0 };
  uint16_t pinsOFF{ 0 };

  uint8_t i{ NO_OF_DUMPLOADS };

  do
  {
    --i;
    // update the local load's state.
    if (LoadStates::LOAD_OFF == physicalLoadState[i])
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
    if (b_reOrderLoads)
    {
      uint8_t i{ NO_OF_DUMPLOADS - 1 };
      const auto temp{ loadPrioritiesAndState[i] };
      do
      {
        loadPrioritiesAndState[i] = loadPrioritiesAndState[i - 1];
        --i;
      } while (i);
      loadPrioritiesAndState[0] = temp;

      b_reOrderLoads = false;
    }
  }

  const bool bDiversionOff{ b_diversionOff };
  uint8_t idx{ NO_OF_DUMPLOADS };
  do
  {
    --idx;
    const auto iLoad{ loadPrioritiesAndState[idx] & loadStateMask };
    physicalLoadState[iLoad] = !bDiversionOff && (b_overrideLoadOn[iLoad] || (loadPrioritiesAndState[idx] & loadStateOnBit)) ? LoadStates::LOAD_ON : LoadStates::LOAD_OFF;
  } while (idx);
}

/**
 * @brief Process with the polarity for the actual voltage sample
 *
 * @param rawSample the current sample
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
 * @brief Process the calculation for the actual current raw sample for the grid
 *
 * @param rawSample the current sample
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
  instP = instP >> 12;                                  // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_grid += instP;                                   // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  sumP_grid_overDL_Period += instP;
}

/**
 * @brief Process the calculation for the actual current raw sample for the diverted power
 *
 * @param rawSample the current sample
 *
 * @ingroup TimeCritical
 */
void processDivertedCurrentRawSample(const int16_t rawSample)
{
  if(b_overrideLoadOn[0])
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
  instP = instP >> 12;                                      // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_diverted += instP;                                   // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
  sumP_diverted_overDL_Period += instP;
}

/**
 * @brief This routine prevents a zero-crossing point from being declared until a certain number
 *        of consecutive samples in the 'other' half of the waveform have been encountered.
 *
 * @ingroup TimeCritical
 */
void confirmPolarity()
{
  /* This routine prevents a zero-crossing point from being declared until 
   * a certain number of consecutive samples in the 'other' half of the 
   * waveform have been encountered.  
   */
  static uint8_t count = 0;
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
 * @brief This routine is called by the ISR when a pair of V & I sample becomes available.
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

        if (EDD_isActive)  // Energy Diversion Display
        {
          // For diverted energy, the latest contribution needs to be added to an
          // accumulator which operates with maximum precision.

          if (realEnergy_diverted < antiCreepLimit_inIEUperMainsCycle)
          {
            realEnergy_diverted = 0;
          }
          divertedEnergyRecent_IEU += realEnergy_diverted;

          // Whole kWh are then recorded separately
          if (divertedEnergyRecent_IEU > IEU_per_Wh)
          {
            divertedEnergyRecent_IEU -= IEU_per_Wh;
            if (!b_overrideLoadOn[0])
            {
              ++divertedEnergyTotal_Wh;
            }
          }
        }
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
    if (sampleSetsDuringNegativeHalfOfMainsCycle == 3)
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
          ++postTransitionCount;
          if (postTransitionCount >= POST_TRANSITION_MAX_COUNT)
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
          absenceOfDivertedEnergyCount = 0;
          EDD_isIdle = false;
          EDD_isActive = true;
        }
        else
        {
          EDD_isIdle = true;
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
}

/**
 * @brief Process the calculation for the current voltage sample
 *
 * @ingroup TimeCritical
 */
void processVoltage()
{
  long filtV_div4 = sampleVminusDC_long >> 2;        // reduce to 16-bits (now x64, or 2^6)
  int32_t inst_Vsquared{ filtV_div4 * filtV_div4 };  // 32-bits (now x4096, or 2^12)

  if constexpr (DATALOG_PERIOD_IN_SECONDS > 10)
  {
    inst_Vsquared >>= 16;  // scaling is now x1/16 (V_ADC x I_ADC)
  }
  else
  {
    inst_Vsquared >>= 12;  // scaling is now x1 (V_ADC x I_ADC)
  }

  l_sum_Vsquared += inst_Vsquared;  // cumulative V^2 (V_ADC x I_ADC)

  // store items for use during next loop
  cumVdeltasThisCycle_long += sampleVminusDC_long;     // for use with LP filter
  polarityConfirmedOfLastSampleV = polarityConfirmed;  // for identification of half cycle boundaries

  ++sampleSetsDuringThisMainsCycle;
}

/**
 * @brief Process the current voltage raw sample
 *
 * @param rawSample the current sample
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

  ++sampleSetsDuringThisDatalogPeriod;
}

/**
 * @brief Process the startup period for the router.
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
 * @brief This code is executed once per 20mS, shortly after the start of each new
 *        mains cycle on phase 0.
 * @details Changing the state of the loads  is a 3-part process:
 *          - change the LOGICAL load states as necessary to maintain the energy level
 *          - update the PHYSICAL load states according to the logical -> physical mapping
 *          - update the driver lines for each of the loads.
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
}

/**
 * @brief Process the start of a new +ve half cycle, just after the zero-crossing point.
 *
 * @ingroup TimeCritical
 */

void processPlusHalfCycle()
{
  processLatestContribution();

  // a simple routine for checking the performance of this new ISR structure
  if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle)
  {
    lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;
  }

  processDataLogging();

  // clear the per-cycle accumulators for use in this new mains cycle.
  sampleSetsDuringThisMainsCycle = 0;
  sumP_grid = 0;
  sumP_diverted = 0;
  sampleSetsDuringNegativeHalfOfMainsCycle = 0;
}

/**
 * @brief Process the start of a new -ve half cycle, just after the zero-crossing point.
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
  long previousOffset = DCoffset_V_long;
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
 * @brief Process the latest contribution after each new cycle additional
 *        processing is performed after each main cycle based on phase 0.
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

  b_newCycle = true;  //  a 50 Hz 'tick' for use by the main code
}

/**
 * @brief Process the case of high energy level, some action may be required.
 *
 * @ingroup TimeCritical
 */
void proceedHighEnergyLevel()
{
  bool bOK_toAddLoad{ true };
  const auto tempLoad{ nextLogicalLoadToBeAdded() };

  if (tempLoad >= NO_OF_DUMPLOADS)
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

#if !defined(__DOXYGEN__)
uint8_t nextLogicalLoadToBeAdded() __attribute__((optimize("-O3")));
#endif
/**
 * @brief Retrieve the next load that could be added (be aware of the order)
 *
 * @return The load number if successful, NO_OF_DUMPLOADS in case of failure
 *
 * @ingroup TimeCritical
 */
uint8_t nextLogicalLoadToBeAdded()
{
  for (uint8_t index = 0; index < NO_OF_DUMPLOADS; ++index)
  {
    if (0x00 == (loadPrioritiesAndState[index] & loadStateOnBit))
    {
      return (index);
    }
  }

  return (NO_OF_DUMPLOADS);
}

#if !defined(__DOXYGEN__)
uint8_t nextLogicalLoadToBeRemoved() __attribute__((optimize("-O3")));
#endif
/**
 * @brief Retrieve the next load that could be removed (be aware of the reverse-order)
 *
 * @return The load number if successful, NO_OF_DUMPLOADS in case of failure
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

#if !defined(__DOXYGEN__)
void processDataLogging() __attribute__((optimize("-O3")));
#endif
/**
 * @brief Process with data logging.
 * @details At the end of each datalogging period, copies are made of the relevant variables
 *          for use by the main code. These variable are then reset for use during the next
 *          datalogging period.
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

  copyOf_sumP_grid_overDL_Period = sumP_grid_overDL_Period;
  sumP_grid_overDL_Period = 0;

  copyOf_sumP_diverted_overDL_Period = sumP_diverted_overDL_Period;
  sumP_diverted_overDL_Period = 0;

  copyOf_sum_Vsquared = l_sum_Vsquared;
  l_sum_Vsquared = 0;

  uint8_t i{ NO_OF_DUMPLOADS };
  do
  {
    --i;
    copyOf_countLoadON[i] = countLoadON[i];
    countLoadON[i] = 0;
  } while (i);

  copyOf_sampleSetsDuringThisDatalogPeriod = sampleSetsDuringThisDatalogPeriod;  // (for diags only)
  copyOf_lowestNoOfSampleSetsPerMainsCycle = lowestNoOfSampleSetsPerMainsCycle;  // (for diags only)
  copyOf_energyInBucket_long = energyInBucket_long;                              // (for diags only)

  lowestNoOfSampleSetsPerMainsCycle = UINT8_MAX;
  sampleSetsDuringThisDatalogPeriod = 0;

  // signal the main processor that logging data are available
  // we skip the period from start to running stable
  b_datalogEventPending = beyondStartUpPeriod;
}

/**
 * @brief Print the settings used for the selected output mode.
 *
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