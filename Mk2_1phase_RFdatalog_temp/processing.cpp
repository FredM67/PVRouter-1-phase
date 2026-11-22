/**
 * @file processing.cpp
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Implements the processing engine
 * @version 0.1
 * @date 2026-11-21
 *
 * @copyright Copyright (c) 2021-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <Arduino.h>

#include "config_derived.h"
#include "dualtariff.h"
#include "processing.h"
#include "utils_pins.h"
#include "shared_var.h"
#include "mult_asm.h"
#include "energy_bucket.h"
#include "load_priorities.h"
#include "fast_control.h"
#include "adc_chain.h"
#include "ct_filter.h"
#include "datalog_math.h"
#include "diverted_energy.h"

// analogue input pins and ADC reference of the board: adc_chain.h
// AVCC (= VCC, 5 V or 3.3 V) on the old boards, the internal 1.1 V on the new one (buffered from AREF).
// ADLAR=1 enables left-aligned ADC for efficient fixed-point math.
static_assert(AdcChain::BIT_REFS1 == REFS1 && AdcChain::BIT_REFS0 == REFS0 && AdcChain::BIT_ADLAR == ADLAR, "adc_chain.h: ADMUX bits differ from avr/io.h");
// ------------------------------------------

// Define ideal bias, ADC mid-range, left aligned.
constexpr uint16_t i_DCoffset_V_nom{ 511U << 6 }; /**< nominal mid-point value of ADC @ x64 scale */
uint16_t i_DCoffset_V[NO_OF_PHASES]{};            /**< <--- for LPF */

/**< main energy bucket for 3-phase use, with units of Joules * SUPPLY_FREQUENCY * 2^Energy::FRACTION_BITS */
constexpr int32_t l_capacityOfEnergyBucket_main{ Energy::fromWatts(WORKING_ZONE_IN_JOULES * SUPPLY_FREQUENCY) };
/**< for resetting flexible thresholds */
constexpr int32_t l_midPointOfEnergyBucket_main{ l_capacityOfEnergyBucket_main / 2 };
/**< threshold in anti-flicker mode - must not exceed 0.4 */
constexpr float f_offsetOfEnergyThresholdsInAFmode{ 0.1F };

// The sums of a datalog period cannot wrap, whatever DATALOG_PERIOD_IN_SECONDS (40 s at most,
// validation.h): at most N_MAX sample sets per mains cycle (config_derived.h), each adding at
// most 2^18 (|V_ADC x I_ADC| or V_ADC^2, x1 scale).
constexpr uint64_t FULL_SCALE_PER_SAMPLE{ 1UL << 18 };
// the sample sets, the divisor of the datalog power and voltage: 16 or 32 bits, as needed
static_assert(static_cast< SampleSetCounter >(MAX_SAMPLE_SETS_PER_DATALOG) == MAX_SAMPLE_SETS_PER_DATALOG, "******** The counter of the sample sets of a datalog period is too narrow ! ********");
// the power: 64 bits, summed once per mains cycle
static_assert(MAX_SAMPLE_SETS_PER_DATALOG * FULL_SCALE_PER_SAMPLE <= static_cast< uint64_t >(INT64_MAX), "the datalog power would overflow");
// V^2: 32 bits, scaled down by 16 when the period needs it
static_assert(MAX_SAMPLE_SETS_PER_DATALOG * (FULL_SCALE_PER_SAMPLE >> (Datalog::isVSquaredScaled(DATALOG_PERIOD_IN_SECONDS) ? 4 : 0)) <= UINT32_MAX,
              "******** The datalog V^2 sum would overflow: check Datalog::isVSquaredScaled() ! ********");

/**< beyond 64 sample sets per cycle, sumP may exceed the 24 bits of Energy::contribution(): halved */
constexpr uint8_t SUM_P_SHIFT{ N_MAX > 64 ? 1 : 0 };

/**< power calibration over n, in fixed point, for every expected n: no division in the ISR */
const Energy::PerSampleCalibration< N_MIN, N_MAX, NO_OF_PHASES > powerCalPerSample PROGMEM{ Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCal) };
constexpr uint8_t powerCalPerSampleShift{ Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCal).shift };

// The prediction at mid-cycle (fast_control.h): the sum of the sample sets so far goes through the
// table entry for a whole cycle of SAMPLE_SETS_PER_CYCLE sample sets, then is scaled by
// SAMPLE_SETS_PER_CYCLE / (sample sets so far). Its largest magnitude, with N_MAX sample sets at full
// scale, x SAMPLE_SETS_PER_CYCLE must fit in the int32_t of FastControl::predictEndOfCycle().
constexpr uint8_t PREDICTION_CAL_INDEX{ SAMPLE_SETS_PER_CYCLE - N_MIN };
constexpr uint64_t MAX_PREDICTION_ENERGY{ ((((static_cast< uint64_t >(N_MAX) << 18) >> SUM_P_SHIFT) * Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCal).value[0][PREDICTION_CAL_INDEX])
                                           >> (powerCalPerSampleShift - Energy::FRACTION_BITS))
                                          + 1 };
static_assert(MAX_PREDICTION_ENERGY * SAMPLE_SETS_PER_CYCLE <= static_cast< uint64_t >(INT32_MAX), "the predicted energy would overflow: check f_powerCal");

// ---- Diverted energy (CT2) ----
// The cycle's CT2 sum goes through the same conversion as the grid's in processLatestContribution()
// (same SUM_P_SHIFT, a table of the same shape), with the CT2 calibration: the energy of a cycle
// is its average power in W x 2^Energy::FRACTION_BITS, the unit of the energy bucket. One Wh is
// 3600 J, i.e. 3600 x SUPPLY_FREQUENCY cycles of 1 W: Energy::fromWatts(3600 x SUPPLY_FREQUENCY).
// The anti-creep limit compares the cycle's average power with ANTI_CREEP_LIMIT W, as Robin's
// (his IEU per cycle and his limit are both 1 / powerCal_diverted per W).
constexpr float f_powerCalDivertedTable[NO_OF_PHASES]{ f_powerCalDiverted }; /**< for the per-sample table */
/**< CT2 calibration over n, in fixed point: no division in the ISR */
const Energy::PerSampleCalibration< N_MIN, N_MAX, NO_OF_PHASES > divertedCalPerSample PROGMEM{ Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCalDivertedTable) };
constexpr uint8_t divertedCalPerSampleShift{ Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCalDivertedTable).shift };

constexpr int32_t DIVERTED_IEU_PER_WH{ Energy::fromWatts(3600L * SUPPLY_FREQUENCY) };                                /**< 2 880 000 at 50 Hz */
constexpr int32_t DIVERTED_ANTI_CREEP_IEU{ Energy::fromWatts(ANTI_CREEP_LIMIT) };                                    /**< 80 with 5 W */
constexpr uint32_t DIVERTED_SHUTDOWN_CYCLES{ static_cast< uint32_t >(DISPLAY_SHUTDOWN_IN_HOURS) * 3600UL * SUPPLY_FREQUENCY }; /**< 1 440 000 with 8 h at 50 Hz */

// The largest energy of a cycle: N_MAX sample sets at full scale, through the largest table entry.
constexpr uint32_t MAX_DIVERTED_CYCLE_ENERGY{ static_cast< uint32_t >(((((static_cast< uint64_t >(N_MAX) << 18) >> SUM_P_SHIFT) * Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCalDivertedTable).value[0][0])
                                                                       >> (divertedCalPerSampleShift - Energy::FRACTION_BITS))
                                                                      + 1) };
// a cycle carries one Wh at most (DivertedEnergy::addCycle())
static_assert(MAX_DIVERTED_CYCLE_ENERGY < static_cast< uint32_t >(DIVERTED_IEU_PER_WH), "a mains cycle would divert more than 1 Wh: check f_powerCalDiverted");
// the energy counted over a datalog period, for its average power: 32 bits
constexpr uint32_t MAX_DIVERTED_DATALOG_ENERGY{ MAX_DIVERTED_CYCLE_ENERGY * DATALOG_PERIOD_IN_MAINS_CYCLES };
static_assert(static_cast< uint64_t >(MAX_DIVERTED_CYCLE_ENERGY) * DATALOG_PERIOD_IN_MAINS_CYCLES <= UINT32_MAX, "******** The diverted energy of a datalog period would overflow: shorten DATALOG_PERIOD_IN_SECONDS or check f_powerCalDiverted ! ********");
/**< the average power over the period: sum / (cycles x 2^FRACTION_BITS), without a division */
constexpr DivertedPower::Average divertedPowerAverage{ DivertedPower::averageOver(DATALOG_PERIOD_IN_MAINS_CYCLES, MAX_DIVERTED_DATALOG_ENERGY, Energy::FRACTION_BITS) };

/**
 * @brief The pins of the local outputs that CT2 measures: the local loads' TRIACs and the relays.
 * @details Neither the remote loads' status LEDs, nor the watchdog LED.
 */
constexpr uint16_t getDivertedOutputPins()
{
  uint16_t pins{ 0 };

  for (const auto &loadEntry : physicalLoadPin)
  {
    if (Load::isLocal(loadEntry))
    {
      bit_set(pins, Load::pinOf(loadEntry));
    }
  }

  if constexpr (RELAY_DIVERSION)
  {
    for (uint8_t idx = 0; idx < relayDiverter.size(); ++idx)
    {
      bit_set(pins, relayDiverter.get_relay(idx).get_pin());
    }
  }

  return pins;
}
constexpr uint16_t divertedOutputPins{ getDivertedOutputPins() }; /**< pins 0..13 (validation.h): PORTD, then PORTB */

DivertedEnergy< DIVERTED_IEU_PER_WH, DIVERTED_ANTI_CREEP_IEU, DIVERTED_SHUTDOWN_CYCLES > divertedEnergy; /**< Wh total and EDD_isActive */

/** override bit of each load: the ISR tests a mask instead of shifting by the pin */
const Load::OverrideMasks< NO_OF_DUMPLOADS > overrideMasks PROGMEM{ Load::overrideMasks(physicalLoadPin) };

constexpr OutputModes outputMode{ OutputModes::NORMAL }; /**< Output mode to be used */

bool b_diversionStarted{ false }; /**< Tracks whether diversion has started */

/**
 * @brief set default threshold at compile time so the variable can be read-only
 *
 * @param lower True to set the lower threshold, false for higher
 * @return the corresponding threshold
 */
constexpr int32_t initThreshold(const bool lower)
{
  return static_cast< int32_t >(lower
                                  ? l_capacityOfEnergyBucket_main * (0.5F - ((OutputModes::ANTI_FLICKER == outputMode) ? f_offsetOfEnergyThresholdsInAFmode : 0.0F))
                                  : l_capacityOfEnergyBucket_main * (0.5F + ((OutputModes::ANTI_FLICKER == outputMode) ? f_offsetOfEnergyThresholdsInAFmode : 0.0F)));
}

constexpr int32_t l_lowerThreshold_default{ initThreshold(true) };  /**< lower default threshold set accordingly to the output mode */
constexpr int32_t l_upperThreshold_default{ initThreshold(false) }; /**< upper default threshold set accordingly to the output mode */

int32_t l_energyInBucket_main{ 0 };       /**< main energy bucket (over all phases) */
int32_t l_energyInBucket_prediction{ 0 }; /**< the bucket predicted at the end of the current mains cycle (fast_control.h) */

/** the fixed levels of the main energy bucket */
constexpr LoadPriorities::Bucket bucketLevels{ l_capacityOfEnergyBucket_main, l_midPointOfEnergyBucket_main,
                                               l_lowerThreshold_default, l_upperThreshold_default };

LoadPriorities::Switching< NO_OF_DUMPLOADS > loadSwitching; /**< for improved control of multiple loads */

int32_t l_sumP[NO_OF_PHASES]{};           /**< cumulative power per phase */
int16_t i_sampleVminusDC[NO_OF_PHASES]{}; /**< current raw voltage sample filtered (left-aligned ADC) */
uint32_t l_filterDC_V[NO_OF_PHASES]{};    /**< for the LPF which determines DC offset (voltage) */

int64_t l_sumP_atSupplyPoint[NO_OF_PHASES]{}; /**< for summation of 'real power' values during datalog period, once per mains cycle */
uint32_t l_sum_Vsquared[NO_OF_PHASES]{};      /**< for summation of V^2 values during datalog period */

int32_t l_sumP_diverted{ 0 };            /**< cumulative power on CT2 during the mains cycle (V_ADC x I_ADC) */
uint32_t l_divertedEnergy_overDL{ 0 };   /**< diverted energy counted during the datalog period, in Energy:: units */

uint8_t n_samplesDuringThisMainsCycle[NO_OF_PHASES]{};     /**< number of sample sets for each phase during each mains cycle */
uint8_t n_samplesDuringNegativeHalf{ 0 };                  /**< sample sets of the negative half cycle so far (phase 0), up to DECISION_SAMPLE + 1 */
SampleSetCounter i_sampleSetsDuringThisDatalogPeriod{ 0 }; /**< number of sample sets during each datalogging period */

remove_cv< remove_reference< decltype(DATALOG_PERIOD_IN_MAINS_CYCLES) >::type >::type n_cycleCountForDatalogging{ 0 }; /**< for counting how often datalog is updated */

uint8_t n_lowestNoOfSampleSetsPerMainsCycle{ 0 }; /**< For a mechanism to check the integrity of this code structure */

// For an enhanced polarity detection mechanism, which includes a persistence check
Polarities polarityOfMostRecentSampleV[NO_OF_PHASES]{};    /**< for zero-crossing detection */
Polarities polarityConfirmed[NO_OF_PHASES]{};              /**< for zero-crossing detection */
Polarities polarityConfirmedOfLastSampleV[NO_OF_PHASES]{}; /**< for zero-crossing detection */

LoadStates physicalLoadState[NO_OF_DUMPLOADS]{}; /**< Physical state of the loads */
uint16_t countLoadON[NO_OF_DUMPLOADS]{};         /**< Number of cycle the load was ON (over 1 datalog period) */

uint32_t absenceOfDivertedEnergyCountInMC{ 0 }; /**< number of main cycles without diverted energy */

uint8_t perSecondCounter{ 0 }; /**< for counting  every second inside the ISR */

bool beyondStartUpPeriod{ false }; /**< start-up delay, allows things to settle */

/**
 * @brief Initializes all elements of a given array to a specified value.
 *
 * This template function provides a type-safe and efficient way to initialize
 * arrays of any type and size with a specific value. Being a `constexpr` function,
 * it can be evaluated at compile time when all inputs are compile-time constants,
 * or at runtime when needed.
 *
 * This utility is particularly useful in embedded systems where:
 * - Predictable and uniform initialization is required
 * - Type safety is important to prevent initialization errors
 * - The array size is automatically deduced, avoiding size mismatch errors
 *
 * @tparam T The type of the array elements (automatically deduced from the array)
 * @tparam N The number of elements in the array (automatically deduced)
 *
 * @param[in,out] array Reference to the array to be initialized. All elements will be
 *                      overwritten with the specified value.
 * @param[in] value The value to assign to each element of the array. The type must be
 *                  compatible with the array element type.
 *
 * @note This function is marked `constexpr`, allowing compile-time evaluation when
 *       both the array and value are compile-time constants. When used with runtime
 *       values, it executes as a regular function.
 *
 * @par Example Usage:
 * @code
 * uint32_t offsets[NO_OF_PHASES];
 * initializeArray(offsets, static_cast<uint32_t>(0x8000UL << 16));
 * @endcode
 *
 * @ingroup Initialization
 */
template< typename T, size_t N >
constexpr void initializeArray(T (&array)[N], T value)
{
  for (size_t i = 0; i < N; ++i)
  {
    array[i] = value;
  }
}

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

  // Local TRIAC pins, plus the optional status LED of each remote load
  for (const auto &loadEntry : physicalLoadPin)
  {
    const uint8_t pin{ Load::pinOf(loadEntry) };

    if (!Load::isLocal(loadEntry) && (pin == 0))
      continue;  // remote load without a status LED

    if (bit_read(output_pins, pin))
      return 0;

    bit_set(output_pins, pin);
  }

  if constexpr (WATCHDOG_PIN_PRESENT)
  {
    if (bit_read(output_pins, watchDogPin))
      return 0;

    bit_set(output_pins, watchDogPin);
  }

  if constexpr (RELAY_DIVERSION)
  {
    for (uint8_t idx = 0; idx < relayDiverter.size(); ++idx)
    {
      const auto relayPin = relayDiverter.get_relay(idx).get_pin();

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

  if constexpr (ROUTER_OFF_PIN_PRESENT)
  {
    if (bit_read(input_pins, routerOffPin))
      return 0;

    bit_set(input_pins, routerOffPin);
  }

  if constexpr (PRIORITY_ROTATION == RotationModes::PIN)
  {
    if (bit_read(input_pins, rotationPin))
      return 0;

    bit_set(input_pins, rotationPin);
  }

  if constexpr (OVERRIDE_PIN_PRESENT)
  {
    // Add all configured override pins
    for (uint8_t i = 0; i < overrideTable.size(); ++i)
    {
      const uint8_t pin = overrideTable.getPin(i);
      if (bit_read(input_pins, pin))
        return 0;

      bit_set(input_pins, pin);
    }
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
  // Initialize DC offset in Q16.16 format: (0x8000 << 16) for left-aligned ADC
  // 0x8000 is the mid-point of a left-aligned 10-bit ADC result
  initializeArray(i_DCoffset_V, i_DCoffset_V_nom);  // nominal mid-point value of ADC left-aligned, x64 scale

  constexpr uint32_t filterDC_V_nom = static_cast< uint32_t >(i_DCoffset_V_nom) << 15;
  initializeArray(l_filterDC_V, filterDC_V_nom);  // nominal mid-point value of ADC left-aligned

  setPinsAsOutput(getOutputPins());      // set the output pins as OUTPUT
  setPinsAsInputPullup(getInputPins());  // set the input pins as INPUT_PULLUP

  uint8_t i{ NO_OF_DUMPLOADS };
  do
  {
    --i;
    loadPrioritiesAndState[i] = loadPrioritiesAtStartup[i];
    loadPrioritiesAndState[i] &= loadStateMask;
  } while (i);

  if constexpr (RF_CHIP_PRESENT)
  {
    // Initialize shared RF module
    if (initialize_rf())
    {
      infoln(F("RF module initialized"));
    }
    else
    {
      infoln(F("RF module initialization FAILED"));
    }
  }

  if constexpr (REMOTE_LOADS_PRESENT)
  {
    remoteLoads.reset();
    infoln(F("Remote loads initialized"));
  }

  // First stop the ADC
  bit_clear(ADCSRA, ADEN);

  // Activate free-running mode
  ADCSRB = 0x00;

  // Reference and left-aligned result for the first conversion too, on the first channel (V)
  ADMUX = AdcChain::admuxOf(PCB_VERSION, AdcChain::Channel::V);

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
 * the corresponding control ports.
 *
 * @details
 * - If a load is OFF, its corresponding pin is added to the `pinsOFF` mask.
 * - If a load is ON, its corresponding pin is added to the `pinsON` mask.
 * - Finally, the pins are updated using `setPinsOFF` and `setPinsON` functions.
 * - Overrides are already part of the physical load states, and the relays are driven
 *   by the relay engine only: no other pin is ever written here.
 *
 * @ingroup TimeCritical
 */
void updatePortsStates()
{
  uint16_t pinsON{ 0 };
  uint16_t pinsOFF{ 0 };

  // One pass over the load map. A local load drives its TRIAC pin; a remote load is
  // driven over RF and only drives its optional status LED here, if it has one.
  uint8_t i{ NO_OF_DUMPLOADS };
  do
  {
    --i;

    const uint8_t loadEntry{ physicalLoadPin[i] };
    // with an all-local map both of these fold away at compile time, leaving the original code
    const uint8_t pin{ REMOTE_LOADS_PRESENT ? Load::pinOf(loadEntry) : loadEntry };
    const bool bDrivesAPin{ !REMOTE_LOADS_PRESENT || Load::isLocal(loadEntry) || (pin != 0) };

    if (LoadStates::LOAD_OFF == physicalLoadState[i])
    {
      if (bDrivesAPin)
      {
        pinsOFF |= bit(pin);
      }
    }
    else
    {
      // count ON-cycles on the load's *state*, so remote loads are datalogged too
      ++countLoadON[i];

      if (bDrivesAPin)
      {
        pinsON |= bit(pin);
      }
    }
  } while (i);

  setPinsOFF(pinsOFF);
  setPinsON(pinsON);
}

/**
 * @brief Updates the physical load states based on logical load priorities and states.
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
 * @note This function is called during each cycle to ensure the physical load states
 *       are synchronized with the logical load states.
 *
 * @ingroup TimeCritical
 */
void updatePhysicalLoadStates()
{
  if constexpr (NO_OF_DUMPLOADS == 0)
  {
    return;
  }

  if constexpr (PRIORITY_ROTATION != RotationModes::OFF)
  {
    if (Shared::b_reOrderLoads)
    {
      rotationTable.rotate(loadPrioritiesAndState);  // each rotation group, among its own places

      Shared::b_reOrderLoads = false;
    }
  }

  // Diversion OFF stops the surplus diversion only: a forced load stays ON. With the router
  // OFF, the main loop clears the overrides and disables diversion: every load is OFF.
  const bool bDiversionEnabled{ Shared::b_diversionEnabled };
  // the override bitmasks are read once: the same snapshot for every load of this cycle
  const uint16_t overrideBitmask{ Shared::overrideBitmask };
  const uint8_t remoteOverrideBitmask{ REMOTE_LOADS_PRESENT ? Shared::remoteOverrideBitmask : uint8_t{ 0 } };

  LoadPriorities::toPhysical< CALIBRATION_MODE >(
    loadPrioritiesAndState, physicalLoadState, bDiversionEnabled,
    [overrideBitmask, remoteOverrideBitmask](const uint8_t iLoad) __attribute__((always_inline)) {
      // A local load has its pin's bit in the local override bitmask, a remote load its
      // ordinal's bit in the remote one; the other mask is 0. With an all-local map, the
      // remote test folds away.
      bool bOverrideActive{ (overrideBitmask & pgm_read_word(&overrideMasks.local[iLoad])) != 0 };
      if constexpr (REMOTE_LOADS_PRESENT)
      {
        bOverrideActive = bOverrideActive || ((remoteOverrideBitmask & pgm_read_byte(&overrideMasks.remote[iLoad])) != 0);
      }
      return bOverrideActive;
    });
}

/**
 * @brief Processes the polarity of the current voltage sample for a specific phase.
 *
 * This function removes the DC offset from the raw voltage sample and determines
 * the polarity (positive or negative) of the sample. The polarity is stored for
 * use in zero-crossing detection and other processing tasks.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 * @param rawSample The current raw voltage sample for the specified phase.
 *
 * @details
 * - The DC offset is subtracted from the raw sample using a low-pass filter (LPF).
 * - The polarity is determined based on whether the filtered sample is greater
 *   than or less than zero.
 *
 * @ingroup TimeCritical
 */
void processPolarity(const uint8_t phase, const uint16_t rawSample)
{
  // With left-aligned ADC and Q16.16 bias, simply subtract the integer part
  // rawSample is already left-aligned (16-bit), extract bias from Q16.16 (upper 16 bits)
  // Add 0.5 for the rounding, +32 when 10-bits left aligned `1<<(15-10)`
  i_sampleVminusDC[phase] = (rawSample | 32U) - i_DCoffset_V[phase];  // +0.5 for the rounding

  polarityOfMostRecentSampleV[phase] = (i_sampleVminusDC[phase] > 0) ? Polarities::POSITIVE : Polarities::NEGATIVE;
}

/**
 * @brief Processes the current raw sample for the specified phase.
 *
 * This function processes the raw current sample for a specific phase by applying
 * filtering to remove DC offset, compensating for the high-pass filter effect of
 * current transformers (CTs), and calculating the instantaneous power.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 * @param rawSample The current raw sample for the specified phase.
 *
 * @details
 * - The DC offset is removed from the raw sample.
 * - Additional filtering is applied to offset the high-pass filter effect of CTs.
 * - The instantaneous power is calculated and accumulated for the phase.
 *
 * @ingroup TimeCritical
 */
void processCurrentRawSample(const uint8_t phase, const uint16_t rawSample)
{
  // extra items for an LPF to improve the processing of data samples from CT1
  static CtFilter::LowPassBoost ctFilter[NO_OF_PHASES];  // for offsetting the behaviour of CTx as a HPF

  // remove most of the DC offset from the current sample.
  // Use the voltage bias, it's most likely more accurate than the default mid-range.
  // Add 0.5 for the rounding, +32 when 10-bits left aligned `1<<(15-10)`
  int16_t sampleIminusDC = (rawSample | 32U) - i_DCoffset_V[phase];

  // extra filtering to offset the HPF effect of CTx
  // Using if constexpr ensures zero overhead when CT filtering is disabled
  if constexpr (lpf_gain != 0.0F && alpha != 0.0F)
  {
    sampleIminusDC = ctFilter[phase].apply(sampleIminusDC, lpf_gain, alpha);  // Robin's filter, at x256, saturated
  }

  // calculate the "real power" in this sample pair and add to the accumulated sum
  // Reduce to 14-bits (x16 scale) before multiplication for efficiency on 8-bit MCU
  const int16_t filtV_div4{ i_sampleVminusDC[phase] >> 2 };  // reduce to 14-bits (now x16, or 2^4)
  const int16_t filtI_div4{ sampleIminusDC >> 2 };           // reduce to 14-bits (now x16, or 2^4)

  // Using optimized assembly multiplication for ~3x speedup in ISR
  int32_t instP;
  multS16x16_to32(instP, filtV_div4, filtI_div4);  // 32-bits (now x256, or 2^8)
  instP >>= 8;                                     // scaling is now x1, as for Mk2 (V_ADC x I_ADC)

  l_sumP[phase] += instP;  // cumulative power, scaling as for Mk2 (V_ADC x I_ADC); into the datalog sum once per cycle
}

/**
 * @brief Processes the raw sample of CT2, the current diverted to the loads.
 *
 * @details CT2 is sampled in every round of the chain, so that the timing never depends on the
 *          settings. As processCurrentRawSample(), without the CT filter: the same DC removal
 *          (the voltage bias), V x I at x1 scale (V_ADC x I_ADC) into the cycle's sum, which
 *          processDivertedEnergy() turns into energy at the positive zero crossing. Without
 *          DIVERTED_CT_PRESENT, the sample is dropped.
 *
 *          The voltage sample is the latest one, two conversions (~208 us) before this CT2
 *          sample, against one (~104 us) for CT1.
 *
 * @param rawSample The raw sample of CT2 (left-aligned ADC).
 *
 * @ingroup TimeCritical
 */
void processDivertedCurrentRawSample(const uint16_t rawSample)
{
  if constexpr (!DIVERTED_CT_PRESENT)
  {
    (void)rawSample;
    return;
  }

  // Add 0.5 for the rounding, +32 when 10-bits left aligned `1<<(15-10)`
  const int16_t sampleIminusDC = (rawSample | 32U) - i_DCoffset_V[0];

  const int16_t filtV_div4{ i_sampleVminusDC[0] >> 2 };  // reduce to 14-bits (now x16, or 2^4)
  const int16_t filtI_div4{ sampleIminusDC >> 2 };       // reduce to 14-bits (now x16, or 2^4)

  int32_t instP;
  multS16x16_to32(instP, filtV_div4, filtI_div4);  // 32-bits (now x256, or 2^8)
  instP >>= 8;                                     // scaling is now x1 (V_ADC x I_ADC)

  l_sumP_diverted += instP;  // at most N_MAX x 2^18 in a cycle
}

/**
 * @brief Confirms the polarity of the current voltage sample for a specific phase.
 *
 * This routine prevents a zero-crossing point from being declared until a certain number
 * of consecutive samples in the 'other' half of the waveform have been encountered.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - If the polarity of the most recent sample matches the last confirmed polarity,
 *   the persistence counter is reset.
 * - If the polarity differs, the persistence counter is incremented.
 * - Once the persistence counter exceeds a predefined threshold, the polarity is confirmed.
 *
 * @ingroup TimeCritical
 */
void confirmPolarity(const uint8_t phase)
{
  static uint8_t count[NO_OF_PHASES]{};

  if (polarityOfMostRecentSampleV[phase] == polarityConfirmedOfLastSampleV[phase])
  {
    count[phase] = 0;
    return;
  }

  if (++count[phase] > PERSISTENCE_FOR_POLARITY_CHANGE)
  {
    count[phase] = 0;
    polarityConfirmed[phase] = polarityOfMostRecentSampleV[phase];
  }
}

/**
 * @brief Processes the current voltage sample for the specified phase.
 *
 * This function processes the voltage sample for a specific phase by calculating
 * the cumulative voltage squared (V²) for RMS calculations, updating the low-pass
 * filter for DC offset removal, and preparing for zero-crossing detection.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - The voltage squared (V²) is calculated and accumulated for RMS calculations.
 * - The low-pass filter is updated to remove the DC offset from the voltage signal.
 * - The polarity of the last confirmed sample is stored for zero-crossing detection.
 * - The number of samples during the current mains cycle is incremented.
 *
 * @ingroup TimeCritical
 */
void processVoltage(const uint8_t phase)
{
  // for the Vrms calculation (for datalogging only)
  // Reduce to 14-bits (x16 scale) before multiplication for efficiency on 8-bit MCU
  const int16_t filtV_div4{ i_sampleVminusDC[phase] >> 2 };  // reduce to 14-bits (now x16, or 2^4)

  // Using optimized assembly multiplication for ~3x speedup in ISR
  int32_t inst_Vsquared_signed;
  multS16x16_to32(inst_Vsquared_signed, filtV_div4, filtV_div4);
  uint32_t inst_Vsquared = static_cast< uint32_t >(inst_Vsquared_signed);

  // The sum cannot overflow: checked at compile time with the sample count (top of this file).
  // Datalog::vrmsFactor() compensates the x1/16 scaling, on the same condition.
  if constexpr (Datalog::isVSquaredScaled(DATALOG_PERIOD_IN_SECONDS))
  {
    inst_Vsquared >>= 12;  // scaling is now x1/16 (V_ADC x V_ADC)
  }
  else
  {
    inst_Vsquared >>= 8;  // scaling is now x1 (V_ADC x V_ADC)
  }

  l_sum_Vsquared[phase] += inst_Vsquared;  // cumulative V^2 (V_ADC x V_ADC)
  //
  // store items for use during next loop
  l_filterDC_V[phase] += i_sampleVminusDC[phase];                    // for use with LP filter
  polarityConfirmedOfLastSampleV[phase] = polarityConfirmed[phase];  // for identification of half cycle boundaries
  ++n_samplesDuringThisMainsCycle[phase];                            // for real power calculations
}

/**
 * @brief Processes the startup period for the router.
 *
 * This function handles the initial startup period, allowing the DC-blocking filters
 * to settle before normal operation begins. It ensures that the system is stable
 * before processing energy and load states.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - During the startup period, the function waits until the filters have settled.
 * - Once the startup period is over, it resets key variables and flags to prepare
 *   for normal operation.
 *
 * @ingroup TimeCritical
 */
void processStartUp(const uint8_t phase)
{
  // wait until the DC-blocking filters have had time to settle
  if (millis() <= (initialDelay + startUpPeriod))
  {
    return;  // still settling, do nothing
  }

  // the DC-blocking filters have had time to settle
  beyondStartUpPeriod = true;
  l_sumP[phase] = 0;
  l_sumP_atSupplyPoint[phase] = 0;
  l_sumP_diverted = 0;
  n_samplesDuringThisMainsCycle[phase] = 0;
  i_sampleSetsDuringThisDatalogPeriod = 0;

  n_lowestNoOfSampleSetsPerMainsCycle = UINT8_MAX;
  // can't say "Go!" here 'cos we're in an ISR!
}

/**
 * @brief Decides the loads of the next mains cycle, on phase 0.
 *
 * This function is executed once per 20ms (for 50Hz), at the sample set DECISION_SAMPLE of
 * the negative half cycle of phase 0 (fast_control.h). The zero-crossing triac drivers latch
 * the new states at the next positive zero crossing.
 *
 * @details
 * - Decides on the energy predicted at the end of the cycle, not on the measured bucket.
 * - Handles recent transitions and updates the post-transition counter.
 * - Adjusts energy thresholds and determines whether to add or remove loads.
 * - Updates the physical load states and control ports.
 *
 * @ingroup TimeCritical
 */
void processStartNewCycle()
{
  // may switch one load ON or OFF (load_priorities.h), on the prediction
  loadSwitching.onNewCycle(loadPrioritiesAndState, l_energyInBucket_prediction, bucketLevels);

  updatePhysicalLoadStates();  // allows the logical-to-physical mapping to be changed

  updatePortsStates();  // update the control ports for each of the physical loads

  if constexpr (REMOTE_LOADS_PRESENT)
  {
    // Rebuild the RF payloads AFTER local physical ports are updated.
    // The actual transmission happens in loop(), never here.
    remoteLoads.updateLoads< physicalLoadPin >(physicalLoadState);
  }

  if (loadPrioritiesAndState[0] & loadStateOnBit)
  {
    absenceOfDivertedEnergyCountInMC = 0;
  }
  else
  {
    ++absenceOfDivertedEnergyCountInMC;
  }
}

/**
 * @brief Processes the start of a new negative half cycle for the specified phase.
 *
 * This function is called just after the zero-crossing point of a negative half cycle.
 * It refreshes the DC offset used to remove the DC component from the voltage signal.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - @c l_filterDC_V is a Q17.15 integrator: @c processVoltage() adds every
 *   @c i_sampleVminusDC to it, so each sample feeds back with a gain of 1/32768.
 * - Here the integrator is simply rescaled to the x64 (left-aligned ADC) domain to
 *   give the offset applied to the following cycle.
 * - The integrator is free-running: it is never reset, and is not clamped. Its time
 *   constant is long enough that a plausible offset drift is tracked smoothly, while
 *   a stuck or disconnected VT settles to that input instead of to a fixed limit.
 *   See @c test/native/test_dc_offset_filter for the tracking and step-response cases.
 * - On phase 0, beyond the start-up period, the energy at the end of the cycle is predicted
 *   from the first half (fast_control.h), for the decision a few sample sets later.
 *
 * @ingroup TimeCritical
 */
void processMinusHalfCycle(const uint8_t phase)
{
  // This is a convenient point to refresh the DC offset for the phase being processed.
  // Bits 15..30 of the integrator, as '>> 15' into a uint16_t, but without its 15-turn
  // shift loop (~105 cycles, 4 here): '<< 1' only drops bit 31, which the uint16_t drops
  // too. Identical for all 2^32 values (checked exhaustively).
  i_DCoffset_V[phase] = static_cast< uint16_t >((l_filterDC_V[phase] << 1) >> 16);

  if ((0 != phase) || !beyondStartUpPeriod)
  {
    return;
  }

  // The prediction, as Robin's 'energyInBucket_long + averagePower'. The sum of the first half
  // goes through the same conversion as the whole cycle in processLatestContribution() (same
  // shift, same table), with the entry for SAMPLE_SETS_PER_CYCLE sample sets; predictEndOfCycle()
  // then scales it by SAMPLE_SETS_PER_CYCLE / n: (sumP / n) x cal, the first half's average power
  // as one cycle's energy, in bucket units. Beyond N_MAX sample sets (no proper half cycle), the
  // sum may exceed the 24 bits of Energy::contribution(): no prediction, the bucket as it is.
  const uint8_t n{ n_samplesDuringThisMainsCycle[0] };
  int32_t energyFirstHalf{ 0 };
  if (n <= N_MAX)
  {
    energyFirstHalf = Energy::contribution(l_sumP[0] >> SUM_P_SHIFT, pgm_read_word(&powerCalPerSample.value[0][PREDICTION_CAL_INDEX]), powerCalPerSampleShift);
  }
  l_energyInBucket_prediction = FastControl::predictEndOfCycle(l_energyInBucket_main, energyFirstHalf, n, SAMPLE_SETS_PER_CYCLE);
}

/**
 * @brief Process the latest contribution after each phase-specific new cycle.
 *
 * This function calculates and updates the energy contribution for the specified phase
 * after each new cycle. It ensures that the energy bucket is updated with the latest
 * power measurements and applies necessary adjustments.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - Adds the latest energy contribution to the main energy bucket.
 * - Applies adjustments for required export energy on phase 0.
 * - Signals a new mains cycle for phase 0.
 *
 * @ingroup TimeCritical
 */
void processLatestContribution(const uint8_t phase)
{
  // for efficiency, the energy scale is Joules * SUPPLY_FREQUENCY, in integer fixed point
  // add the latest energy contribution to the main energy accumulator
  // (sumP / n) x cal == sumP x (cal / n): cal / n comes from the table, so there is no division
  const uint8_t n{ n_samplesDuringThisMainsCycle[phase] };
  int32_t sumP{ l_sumP[phase] };
  uint8_t index{ static_cast< uint8_t >(n - N_MIN) };

  if ((n < N_MIN) || (n > N_MAX))
  {
    // unexpected cycle length (start-up, missing phase): rescale the sum to N_MIN samples
    sumP = (sumP / n) * N_MIN;
    index = 0;
  }

  l_energyInBucket_main += Energy::contribution(sumP >> SUM_P_SHIFT, pgm_read_word(&powerCalPerSample.value[phase][index]), powerCalPerSampleShift);

  // apply any adjustment that is required.
  if (0 == phase)
  {
    // If diversion hasn't started yet, use start threshold, otherwise use regular offset
    if (!b_diversionStarted)
    {
      l_energyInBucket_main -= Energy::fromWatts(DIVERSION_START_THRESHOLD_WATTS);

      // Check if we've exceeded the threshold to start diversion
      if (l_energyInBucket_main > l_upperThreshold_default)
      {
        b_diversionStarted = true;
        // Once started, we divert all surplus according to the configured fixed offset
      }
    }
    else
    {
      // When diversion is already started, apply normal export offset if configured
      // Comment or remove this if you want to divert ALL surplus once started
      l_energyInBucket_main -= Energy::fromWatts(REQUIRED_EXPORT_IN_WATTS);
    }

    if (++perSecondCounter == SUPPLY_FREQUENCY)
    {
      perSecondCounter = 0;

      if (absenceOfDivertedEnergyCountInMC > SUPPLY_FREQUENCY)
      {
        ++Shared::absenceOfDivertedEnergyCountInSeconds;
        // Reset diversion state if we've had no diversion for a full second
        b_diversionStarted = false;
      }
      else
        Shared::absenceOfDivertedEnergyCountInSeconds = 0;
    }

    Shared::b_newMainsCycle = true;  //  a 50 Hz 'tick' for use by the main code
  }
  // max and min limits to the main accumulator's level: processPlusHalfCycle()
}

/**
 * @brief Turns the cycle's CT2 sum into energy and counts it, at the positive zero crossing.
 *
 * @details
 * - The sum goes through the CT2 table, as the grid's in processLatestContribution(): the
 *   cycle's average power in W x 2^Energy::FRACTION_BITS. A cycle of an unexpected length
 *   (start-up, missing mains) counts as no energy: no division in the ISR.
 * - The local outputs are read on their port registers: any of them ON, and any of them ON
 *   and forced (override pin, scheduled boost: the local override bitmask). A cycle with a
 *   forced output is not counted (DivertedEnergy, diverted_energy.h). The port holds the states
 *   just decided for the cycle that starts now, so the states may lag one cycle behind the
 *   energy; the overrides change once per second.
 * - The energy counted goes into the datalog sum too, for the average diverted power.
 *
 * @ingroup TimeCritical
 */
void processDivertedEnergy()
{
  const uint8_t n{ n_samplesDuringThisMainsCycle[0] };
  int32_t energy{ 0 };
  if ((n >= N_MIN) && (n <= N_MAX))
  {
    energy = Energy::contribution(l_sumP_diverted >> SUM_P_SHIFT, pgm_read_word(&divertedCalPerSample.value[0][n - N_MIN]), divertedCalPerSampleShift);
  }
  l_sumP_diverted = 0;

  const uint16_t outputsON{ static_cast< uint16_t >(((static_cast< uint16_t >(PORTB) << 8) | PORTD) & divertedOutputPins) };
  const bool anyLoadOn{ outputsON != 0 };
  const bool anyLoadForced{ (Shared::overrideBitmask & outputsON) != 0 };

  const int32_t counted{ divertedEnergy.countedEnergy(energy, anyLoadForced) };
  divertedEnergy.addCycle(counted, anyLoadOn, anyLoadForced);
  l_divertedEnergy_overDL += static_cast< uint32_t >(counted);  // 0 or more

  Shared::EDD_isActive = divertedEnergy.isActive();
}

/**
 * @brief Process data logging at the end of each logging period.
 *
 * This function handles the data logging process by copying relevant variables
 * for use by the main code and resetting them for the next logging period.
 *
 * @details
 * - Copies cumulative power and voltage squared values for each phase.
 * - Copies load ON counts and other diagnostic variables.
 * - Resets variables for the next data logging period.
 * - Signals the main processor that logging data is available after the startup period.
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

  uint8_t phase{ NO_OF_PHASES };
  do
  {
    --phase;
    Shared::copyOf_sumP_atSupplyPoint[phase] = l_sumP_atSupplyPoint[phase];
    l_sumP_atSupplyPoint[phase] = 0;

    Shared::copyOf_sum_Vsquared[phase] = l_sum_Vsquared[phase];
    l_sum_Vsquared[phase] = 0;
  } while (phase);

  uint8_t i{ NO_OF_DUMPLOADS };
  do
  {
    --i;
    Shared::copyOf_countLoadON[i] = countLoadON[i];
    countLoadON[i] = 0;
  } while (i);

  Shared::copyOf_sampleSetsDuringThisDatalogPeriod = i_sampleSetsDuringThisDatalogPeriod;  // (for diags only)
  Shared::copyOf_lowestNoOfSampleSetsPerMainsCycle = n_lowestNoOfSampleSetsPerMainsCycle;  // (for diags only)
  Shared::copyOf_energyInBucket_main = l_energyInBucket_main;                              // (for diags only)

  if constexpr (DIVERTED_CT_PRESENT)
  {
    // the average over the whole period: a forced cycle counts as 0 W
    Shared::copyOf_divertedPower_W = DivertedPower::watts(l_divertedEnergy_overDL, divertedPowerAverage);
    Shared::copyOf_divertedEnergyTotal_Wh = divertedEnergy.totalWh();
    l_divertedEnergy_overDL = 0;
  }

  n_lowestNoOfSampleSetsPerMainsCycle = UINT8_MAX;
  i_sampleSetsDuringThisDatalogPeriod = 0;

  // signal the main processor that logging data are available
  // we skip the period from start to running stable
  Shared::b_datalogEventPending = beyondStartUpPeriod;
}

/**
 * @brief Process the start of a new positive half cycle for the specified phase.
 *
 * This function is called just after the zero-crossing point of a positive half cycle.
 * It processes the latest energy contribution, updates performance metrics, and handles
 * data logging for phase 0.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - Processes the latest energy contribution for the specified phase.
 * - Updates the minimum number of ADC sample sets per mains cycle for phase 0.
 * - Handles data logging at the end of the logging period for phase 0.
 * - Resets cumulative power and sample count for the phase.
 *
 * @ingroup TimeCritical
 */
void processPlusHalfCycle(const uint8_t phase)
{
  processLatestContribution(phase);  // runs at 6.6 ms intervals

  l_sumP_atSupplyPoint[phase] += l_sumP[phase];  // the cycle's power into the datalog sum, 64 bits

  // A performance check to monitor and display the minimum number of sets of
  // ADC samples per mains cycle, the expected number being 20ms / (104us * 6) = 32.05
  //
  if (0 == phase)
  {
    // Min and max limits to the level of the energy bucket, once the cycle's contribution is in
    // (the decision of this cycle comes later, at mid-cycle, on the prediction). This is to
    // ensure correct operation when conditions change, i.e. when import changes to export, and
    // vice versa.
    if (l_energyInBucket_main > l_capacityOfEnergyBucket_main)
    {
      l_energyInBucket_main = l_capacityOfEnergyBucket_main;
    }
    else if (l_energyInBucket_main < 0)
    {
      l_energyInBucket_main = 0;
    }

    if (n_samplesDuringThisMainsCycle[phase] < n_lowestNoOfSampleSetsPerMainsCycle)
    {
      n_lowestNoOfSampleSetsPerMainsCycle = n_samplesDuringThisMainsCycle[phase];
    }

    if constexpr (DIVERTED_CT_PRESENT)
    {
      processDivertedEnergy();  // before the datalog, so that the period ends with this cycle
    }

    processDataLogging();
  }

  l_sumP[phase] = 0;
  n_samplesDuringThisMainsCycle[phase] = 0;
}

/**
 * @brief Processes raw voltage and current samples for the specified phase.
 *
 * This routine is called by the ISR when a pair of voltage and current samples
 * becomes available. It handles the processing of raw samples, including polarity
 * detection, zero-crossing handling, and half-cycle processing.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - Determines the polarity of the current sample and handles transitions between
 *   positive and negative half cycles.
 * - Processes the start of new positive and negative half cycles.
 * - For phase 0, it triggers the start of a new mains cycle and handles startup logic.
 *
 * @ingroup TimeCritical
 */
void processRawSamples(const uint8_t phase)
{
  // The raw V and I samples are processed in "phase pairs"
  const auto &lastPolarity{ polarityConfirmedOfLastSampleV[phase] };

  if (Polarities::POSITIVE == polarityConfirmed[phase])
  {
    // the polarity of this sample is positive
    if (Polarities::POSITIVE != lastPolarity)
    {
      // This is the start of a new +ve half cycle, for this phase, just after the zero-crossing point.
      if (beyondStartUpPeriod)
      {
        processPlusHalfCycle(phase);
      }
      else
      {
        processStartUp(phase);
      }
    }
  }
  else
  {
    // the polarity of this sample is negative
    if (Polarities::NEGATIVE != lastPolarity)
    {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)
      processMinusHalfCycle(phase);
      if (0 == phase)
      {
        n_samplesDuringNegativeHalf = 0;
      }
    }

    // The loads are decided at mid-cycle, on the prediction (fast_control.h): the voltage is now
    // far enough past the zero crossing for the trigger devices to be reliably armed, and they
    // latch the new states at the next positive zero crossing. The count stops past the decision,
    // so that a negative voltage that lasts never wraps it into a second decision.
    if ((0 == phase) && (n_samplesDuringNegativeHalf <= FastControl::DECISION_SAMPLE))
    {
      if (beyondStartUpPeriod && (FastControl::DECISION_SAMPLE == n_samplesDuringNegativeHalf))
      {
        // This code is executed once per 20mS, at mid-cycle on phase 0.
        processStartNewCycle();
      }
      ++n_samplesDuringNegativeHalf;
    }
  }
}

/**
 * @brief Processes the current voltage raw sample for the specified phase.
 *
 * This function processes the raw voltage sample for a specific phase by handling
 * polarity detection, zero-crossing confirmation, and voltage processing. It ensures
 * that the voltage sample is properly filtered and analyzed for further processing.
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 * @param rawSample The current raw voltage sample for the specified phase.
 *
 * @details
 * - Determines and confirms the polarity of the raw voltage sample.
 * - Handles zero-crossing detection and processes raw samples for the phase.
 * - Updates voltage-related metrics and increments the sample set count for phase 0.
 *
 * @ingroup TimeCritical
 */
void processVoltageRawSample(const uint8_t phase, const uint16_t rawSample)
{
  processPolarity(phase, rawSample);
  confirmPolarity(phase);

  processRawSamples(phase);  // deals with aspects that only occur at particular stages of each mains cycle

  processVoltage(phase);

  if (phase == 0)
  {
    ++i_sampleSetsDuringThisDatalogPeriod;
  }
}

/**
 * @brief Print the settings used for the selected output mode.
 *
 * This function displays the relevant configuration parameters for the currently
 * selected output mode. It provides details about the energy bucket capacity,
 * thresholds, and mode-specific settings.
 *
 * @details
 * - For the "normal" mode, it displays the energy bucket capacity and thresholds.
 * - For the "anti-flicker" mode, it also displays the offset of energy thresholds.
 *
 * @ingroup Initialization
 */
void printParamsForSelectedOutputMode()
{
  // display relevant settings for selected output mode
  info(F("Output mode:    "));
  if (OutputModes::NORMAL == outputMode)
  {
    infoln(F("normal"));
  }
  else
  {
    infoln(F("anti-flicker"));
    constexpr int32_t offsetOfEnergyThresholds_x100{ SerialOutput::toDecimals(f_offsetOfEnergyThresholdsInAFmode, 2) };
    info(F("\toffsetOfEnergyThresholds  = "));
    infolnDecimals(offsetOfEnergyThresholds_x100, 2);
  }
  info(F("\tl_capacityOfEnergyBucket_main = "));
  infoln(l_capacityOfEnergyBucket_main);
  info(F("\tl_lowerEnergyThreshold   = "));
  infoln(l_lowerThreshold_default);
  info(F("\tl_upperEnergyThreshold   = "));
  infoln(l_upperThreshold_default);
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

// ADC optimization: circular linked list for channels (private implementation)

/**
 * @brief ADC channel context for circular linked list optimization
 *
 * This structure replaces the switch-case logic in the ADC ISR with a more
 * efficient circular linked list approach. Minimal memory footprint.
 *
 * Based on florentbr's optimization suggestion #1 for reducing ISR overhead.
 *
 * @ingroup TimeCritical
 */
struct adc_ctx_t
{
  struct adc_ctx_t *next;  /**< pointer to next context in circular list */
  AdcChain::Channel index; /**< channel just converted: V, I_GRID, I_DIVERTED */
  uint8_t admux;           /**< ADMUX register value written after this channel: the channel after next */
};

// ADC optimization: circular linked list for channel management
// The ADMUX written after a conversion applies to the conversion after next (adc_chain.h)
static adc_ctx_t _channels[AdcChain::NO_OF_CHANNELS] = {
  { .next = &_channels[1], .index = AdcChain::Channel::V, .admux = AdcChain::admuxWrittenAfter(PCB_VERSION, AdcChain::Channel::V) },                   // V -> setup for I_DIVERTED
  { .next = &_channels[2], .index = AdcChain::Channel::I_GRID, .admux = AdcChain::admuxWrittenAfter(PCB_VERSION, AdcChain::Channel::I_GRID) },         // I_GRID -> setup for V
  { .next = &_channels[0], .index = AdcChain::Channel::I_DIVERTED, .admux = AdcChain::admuxWrittenAfter(PCB_VERSION, AdcChain::Channel::I_DIVERTED) }  // I_DIVERTED -> setup for I_GRID
};

static adc_ctx_t *_ctx = &_channels[0];

ISR(ADC_vect)
{
  uint16_t adc_raw = ADC;

  switch (_ctx->index)
  {
    case AdcChain::Channel::V:
      processVoltageRawSample(0, adc_raw);
      break;
    case AdcChain::Channel::I_GRID:
      processCurrentRawSample(0, adc_raw);
      break;
    default:
      processDivertedCurrentRawSample(adc_raw);
      break;
  }

  // Set ADMUX at the end of the interrupt to ensure at least 128 CPU cycles
  // have passed since the trigger event (ATmega328p datasheet requirement)
  ADMUX = _ctx->admux;

  _ctx = _ctx->next;
}  // end of ISR
