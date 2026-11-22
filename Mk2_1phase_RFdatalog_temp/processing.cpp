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
#include "utils_display.h"

// analogue input pins and ADC reference of the board: adc_chain.h
// AVCC (= VCC, 5 V or 3.3 V) on the old boards, the internal 1.1 V on the new one (buffered from AREF).
// ADLAR=1 enables left-aligned ADC for efficient fixed-point math.
static_assert(AdcChain::BIT_REFS1 == REFS1 && AdcChain::BIT_REFS0 == REFS0 && AdcChain::BIT_ADLAR == ADLAR, "adc_chain.h: ADMUX bits differ from avr/io.h");
// ------------------------------------------

// Define ideal bias, ADC mid-range, left aligned.
constexpr uint16_t i_DCoffset_V_nom{ 511U << 6 }; /**< nominal mid-point value of ADC @ x64 scale */
uint16_t i_DCoffset_V[NO_OF_PHASES]{};            /**< <--- for LPF */

/**< main energy bucket, with units of Joules * SUPPLY_FREQUENCY * 2^Energy::FRACTION_BITS */
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

// The prediction at mid-cycle (fast_control.h): 2 x the sum of the n sample sets so far goes through
// the table entry for 2n sample sets, sumP x cal / n, without a division. With 2n <= N_MAX, 2 x sumP
// is at most N_MAX x 2^18, within the 24 bits of Energy::contribution() once shifted by SUM_P_SHIFT
// (as a whole cycle's sum), and the energy at most a whole cycle's at full scale.

// ---- Diverted energy (CT2) ----
// The cycle's CT2 sum goes through the same conversion as the grid's in processLatestContribution()
// (same SUM_P_SHIFT, a table of the same shape), with the CT2 calibration: the energy of a cycle
// is its average power in W x 2^Energy::FRACTION_BITS, the unit of the energy bucket. One Wh is
// 3600 J, i.e. 3600 x SUPPLY_FREQUENCY cycles of 1 W: Energy::fromWatts(3600 x SUPPLY_FREQUENCY).
// The anti-creep limit compares the cycle's average power with ANTI_CREEP_LIMIT W, as Robin's
// (his IEU per cycle and his limit are both 1 / powerCal_diverted per W).
constexpr float f_powerCalDivertedTable[NO_OF_PHASES]{ DIVERTED_CT_PRESENT ? f_powerCalDiverted : 1.0F }; /**< for the per-sample table (1 as a stand-in without CT2: its rating is not used) */
/**< CT2 calibration over n, in fixed point: no division in the ISR */
const Energy::PerSampleCalibration< N_MIN, N_MAX, NO_OF_PHASES > divertedCalPerSample PROGMEM{ Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCalDivertedTable) };
constexpr uint8_t divertedCalPerSampleShift{ Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCalDivertedTable).shift };

constexpr int32_t DIVERTED_IEU_PER_WH{ Energy::fromWatts(3600L * SUPPLY_FREQUENCY) };                                          /**< 2 880 000 at 50 Hz */
constexpr int32_t DIVERTED_ANTI_CREEP_IEU{ Energy::fromWatts(ANTI_CREEP_LIMIT) };                                              /**< 80 with 5 W */
constexpr uint32_t DIVERTED_SHUTDOWN_CYCLES{ static_cast< uint32_t >(DISPLAY_SHUTDOWN_IN_HOURS) * 3600UL * SUPPLY_FREQUENCY }; /**< 1 440 000 with 8 h at 50 Hz */

// The largest energy of a cycle: N_MAX sample sets at full scale, through the largest table entry.
constexpr uint32_t MAX_DIVERTED_CYCLE_ENERGY{ static_cast< uint32_t >(((((static_cast< uint64_t >(N_MAX) << 18) >> SUM_P_SHIFT) * Energy::toFixedPerSample< N_MIN, N_MAX, SUM_P_SHIFT >(f_powerCalDivertedTable).value[0][0])
                                                                       >> (divertedCalPerSampleShift - Energy::FRACTION_BITS))
                                                                      + 1) };
// a cycle carries one Wh at most (DivertedEnergy::addCycle())
static_assert(!DIVERTED_CT_PRESENT || MAX_DIVERTED_CYCLE_ENERGY < static_cast< uint32_t >(DIVERTED_IEU_PER_WH), "a mains cycle would divert more than 1 Wh: check f_powerCalDiverted");
// the energy counted over a datalog period, for its average power: 32 bits
constexpr uint32_t MAX_DIVERTED_DATALOG_ENERGY{ MAX_DIVERTED_CYCLE_ENERGY * DATALOG_PERIOD_IN_MAINS_CYCLES };
static_assert(!DIVERTED_CT_PRESENT || static_cast< uint64_t >(MAX_DIVERTED_CYCLE_ENERGY) * DATALOG_PERIOD_IN_MAINS_CYCLES <= UINT32_MAX, "******** The diverted energy of a datalog period would overflow: shorten DATALOG_PERIOD_IN_SECONDS or check f_powerCalDiverted ! ********");
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

int32_t l_sumP_diverted{ 0 };          /**< cumulative power on CT2 during the mains cycle (V_ADC x I_ADC) */
uint32_t l_divertedEnergy_overDL{ 0 }; /**< diverted energy counted during the datalog period, in Energy:: units */

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
    static_assert(CtFilter::isAccurate(lpf_gain, alpha), "******** CT1 filter: alpha must be within 0.00005..0.0625 and lpf_gain below 64 (calibration.h) ! ********");
    constexpr CtFilter::Coefficients ctFilterCoefficients{ CtFilter::coefficientsOf(lpf_gain, alpha) };

    sampleIminusDC = ctFilter[phase].apply(sampleIminusDC, ctFilterCoefficients);  // Robin's filter, at x256, in fixed point, saturated
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
  l_sum_Vsquared[phase] = 0;  // like the sample sets: the first datalog period holds its own samples only
  if constexpr (DIVERTED_CT_PRESENT)
  {
    l_sumP_diverted = 0;
  }
  n_samplesDuringThisMainsCycle[phase] = 0;
  i_sampleSetsDuringThisDatalogPeriod = 0;

  n_lowestNoOfSampleSetsPerMainsCycle = UINT8_MAX;
  // can't say "Go!" here 'cos we're in an ISR!
}

// ---- The work of the mains cycle, spread over the CT2 calls that follow ----
//
// The ADC interrupt writes ADMUX last, and that write must land within one conversion (1664
// cycles) of the interrupt, even when the interrupt waited for the end of a 1-Wire time slot
// (interrupts off up to ~1150 cycles, temperature sensing): every call must reach its ADMUX
// write within about 450 cycles. The V call at a zero crossing only takes the snapshots that
// must be taken on that very sample, and posts the rest of the work as tasks; each CT2 call,
// the lightest, then runs one task, in the order of the bits below. The loads still latch at
// the positive zero crossing; the decision is taken on the same prediction as before, and the
// energies and datalog values are the same, only computed a few sample sets later.
static_assert(NO_OF_PHASES == 1, "the cycle tasks handle one phase");

namespace CycleTask
{
inline constexpr uint16_t CONTRIBUTION{ 1U << 0 };     /**< the cycle's energy into the bucket (positive crossing) */
inline constexpr uint16_t DIVERTED{ 1U << 1 };         /**< the cycle's CT2 energy (positive crossing) */
inline constexpr uint16_t DIVERTED_COUNT{ 1U << 2 };   /**< the CT2 energy counted (positive crossing) */
inline constexpr uint16_t DATALOG_SUM{ 1U << 3 };      /**< the datalog sum, the counters, the main code's tick (positive crossing) */
inline constexpr uint16_t DATALOG_COPY{ 1U << 4 };     /**< end of a datalog period: the copies for the main code */
inline constexpr uint16_t DATALOG_COPY_CT2{ 1U << 5 }; /**< end of a datalog period: the CT2 copies, then the event */
inline constexpr uint16_t PREDICTION{ 1U << 6 };       /**< the prediction (negative crossing) */
inline constexpr uint16_t DECISION{ 1U << 7 };         /**< the decision on the prediction (DECISION_SAMPLE) */
inline constexpr uint16_t LOAD_STATES{ 1U << 8 };      /**< the logical states to the physical loads (after the decision) */
inline constexpr uint16_t PORTS{ 1U << 9 };            /**< the physical states to the ports and the RF payloads (after the decision) */
}  // namespace CycleTask

uint16_t pendingCycleTasks{ 0 }; /**< CycleTask bits still to run */

// snapshots taken at the zero crossings, for the tasks
int32_t l_sumP_lastCycle{ 0 };             /**< l_sumP of the cycle that just ended */
int32_t l_sumP_diverted_lastCycle{ 0 };    /**< l_sumP_diverted of the cycle that just ended */
int32_t l_divertedEnergy_lastCycle{ 0 };   /**< its energy, from the CT2 table */
uint16_t divertedOutputsON_lastCycle{ 0 }; /**< the local outputs ON when it was turned into energy */
uint8_t n_samplesLastCycle{ 0 };           /**< sample sets of the cycle that just ended */
int32_t l_sumP_firstHalf{ 0 };             /**< l_sumP of the first half, at the negative crossing */
uint8_t n_samplesFirstHalf{ 0 };           /**< sample sets of the first half */
// the sums of a datalog period, taken on its last sample
uint32_t l_sum_Vsquared_lastPeriod{ 0 };
SampleSetCounter i_sampleSetsLastPeriod{ 0 };
bool b_datalogPeriodEndsNext{ DATALOG_PERIOD_IN_MAINS_CYCLES == 1 }; /**< the next positive crossing ends the datalog period (addToDatalogSum()) */

/**
 * @brief Decides the loads of the next mains cycle, on the prediction.
 *
 * This task runs once per mains cycle, on the CT2 call of the sample set DECISION_SAMPLE of the
 * negative half cycle (fast_control.h). The physical states and the ports follow on the next two
 * CT2 calls (updatePhysicalLoadStates(), applyDecision()); the zero-crossing triac drivers latch
 * the new states at the next positive zero crossing.
 *
 * @details
 * - Decides on the energy predicted at the end of the cycle, not on the measured bucket.
 * - Handles recent transitions and updates the post-transition counter.
 * - Adjusts energy thresholds and determines whether to add or remove loads.
 *
 * @ingroup TimeCritical
 */
void processStartNewCycle()
{
  // may switch one load ON or OFF (load_priorities.h), on the prediction
  loadSwitching.onNewCycle(loadPrioritiesAndState, l_energyInBucket_prediction, bucketLevels);
}

/**
 * @brief Applies the decision: the ports of the local loads, the RF payloads, the absence count.
 *
 * @details The task after updatePhysicalLoadStates(), on the next CT2 call.
 *
 * @ingroup TimeCritical
 */
void applyDecision()
{
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
 * - On phase 0, beyond the start-up period, the sum of the first half is taken for the
 *   prediction (fast_control.h), which the next CT2 call computes (predictEndOfCycle()), for
 *   the decision a few sample sets later.
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

  if (pendingCycleTasks)
  {
    finishCycleTasks();  // a half cycle shorter than its tasks: never on a proper mains
  }

  // the first half, as it is on this sample
  l_sumP_firstHalf = l_sumP[0];
  n_samplesFirstHalf = n_samplesDuringThisMainsCycle[0];
  pendingCycleTasks = CycleTask::PREDICTION;
}

/**
 * @brief Predicts the bucket at the end of the cycle, from the first half (fast_control.h).
 *
 * @details The task posted at the negative crossing. As Robin's "energyInBucket_long +
 *          averagePower": the first half's average power as one cycle's energy, in bucket
 *          units, through the table entry for 2n sample sets (no division). Outside N_MIN..N_MAX
 *          for 2n (no proper half cycle), no prediction: the bucket as it is.
 *
 * @ingroup TimeCritical
 */
void predictEndOfCycle()
{
  const uint8_t index{ FastControl::wholeCycleIndexOf(n_samplesFirstHalf, N_MIN, N_MAX) };
  if (FastControl::NO_INDEX == index)
  {
    l_energyInBucket_prediction = l_energyInBucket_main;
    return;
  }
  l_energyInBucket_prediction = FastControl::predictEndOfCycle(l_energyInBucket_main, l_sumP_firstHalf, pgm_read_word(&powerCalPerSample.value[0][index]), SUM_P_SHIFT, powerCalPerSampleShift);
}

/**
 * @brief Rescales the sum of a cycle of an unexpected length to N_MIN sample sets.
 *
 * @details A call of its own: its division keeps its registers out of the contribution task,
 *          which runs it on an unexpected cycle length only (start-up, missing mains).
 *
 * @param sumP The sum of the cycle.
 * @param n Its sample sets, not 0.
 * @return (sumP / n) x N_MIN
 */
int32_t rescaleToNMin(const int32_t sumP, const uint8_t n)
{
  return (sumP / n) * N_MIN;
}

/**
 * @brief Process the latest contribution, the task after each positive crossing.
 *
 * This function calculates and updates the energy contribution of the cycle that just ended.
 * It ensures that the energy bucket is updated with the latest power measurements and applies
 * necessary adjustments.
 *
 * @details
 * - Adds the latest energy contribution to the main energy bucket.
 * - Applies adjustments for required export energy.
 * - Limits the bucket to its range, once the cycle's contribution is in.
 *
 * @ingroup TimeCritical
 */
void processLatestContribution()
{
  // for efficiency, the energy scale is Joules * SUPPLY_FREQUENCY, in integer fixed point
  // add the latest energy contribution to the main energy accumulator
  // (sumP / n) x cal == sumP x (cal / n): cal / n comes from the table, so there is no division
  const uint8_t n{ n_samplesLastCycle };
  int32_t sumP{ l_sumP_lastCycle };
  uint8_t index{ static_cast< uint8_t >(n - N_MIN) };

  // the new cycle's samples so far stay in the sum (processPlusHalfCycle())
  l_sumP[0] -= sumP;

  if ((n < N_MIN) || (n > N_MAX))
  {
    // unexpected cycle length (start-up, missing phase): rescale the sum to N_MIN samples
    sumP = rescaleToNMin(sumP, n);
    index = 0;
  }

  l_energyInBucket_main += Energy::contribution(sumP >> SUM_P_SHIFT, pgm_read_word(&powerCalPerSample.value[0][index]), powerCalPerSampleShift);

  // apply any adjustment that is required.
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
}

/**
 * @brief Turns the cycle's CT2 sum into energy, the task after the contribution.
 *
 * @details
 * - The sum goes through the CT2 table, as the grid's in processLatestContribution(): the
 *   cycle's average power in W x 2^Energy::FRACTION_BITS. A cycle of an unexpected length
 *   (start-up, missing mains) counts as no energy: no division in the ISR.
 * - The sum was taken, not cleared, at the crossing: it is taken off here.
 * - The local outputs are read on their port registers. The port holds the states decided for
 *   the cycle that starts now, so the states may lag one cycle behind the energy.
 * - countDivertedEnergy() counts it on the next CT2 call.
 *
 * @ingroup TimeCritical
 */
void processDivertedEnergy()
{
  const int32_t sumP{ l_sumP_diverted_lastCycle };
  l_sumP_diverted -= sumP;  // the new cycle's samples so far stay in the sum (processPlusHalfCycle())

  const uint8_t n{ n_samplesLastCycle };
  int32_t energy{ 0 };
  if ((n >= N_MIN) && (n <= N_MAX))
  {
    energy = Energy::contribution(sumP >> SUM_P_SHIFT, pgm_read_word(&divertedCalPerSample.value[0][n - N_MIN]), divertedCalPerSampleShift);
  }
  l_divertedEnergy_lastCycle = energy;

  divertedOutputsON_lastCycle = static_cast< uint16_t >(((static_cast< uint16_t >(PORTB) << 8) | PORTD) & divertedOutputPins);
}

/**
 * @brief Counts the cycle's CT2 energy, the task after processDivertedEnergy().
 *
 * @details
 * - Any of the local outputs ON, and any of them ON and forced (override pin, scheduled boost:
 *   the local override bitmask). A cycle with a forced output is not counted (DivertedEnergy,
 *   diverted_energy.h). The overrides change once per second.
 * - The energy counted goes into the datalog sum too, for the average diverted power.
 *
 * @ingroup TimeCritical
 */
void countDivertedEnergy()
{
  const int32_t energy{ l_divertedEnergy_lastCycle };
  const uint16_t outputsON{ divertedOutputsON_lastCycle };
  const bool anyLoadOn{ outputsON != 0 };
  const bool anyLoadForced{ (Shared::overrideBitmask & outputsON) != 0 };

  const int32_t counted{ divertedEnergy.countedEnergy(energy, anyLoadForced) };
  divertedEnergy.addCycle(counted, anyLoadOn, anyLoadForced);
  l_divertedEnergy_overDL += static_cast< uint32_t >(counted);  // 0 or more

  Shared::EDD_isActive = divertedEnergy.isActive();
}

/**
 * @brief Copies the sums of the datalog period that just ended for the main code.
 *
 * @details The task after the cycle's power is in the datalog sum: the power, V^2 and the
 *          sample sets (taken on the last sample of the period), the ON counts of the loads and
 *          the diagnostics. Without CT2, it signals the main code that logging data are available.
 *
 * @ingroup TimeCritical
 */
void processDataLogging()
{
  Shared::copyOf_sumP_atSupplyPoint[0] = l_sumP_atSupplyPoint[0];
  l_sumP_atSupplyPoint[0] = 0;

  Shared::copyOf_sum_Vsquared[0] = l_sum_Vsquared_lastPeriod;
  l_sum_Vsquared[0] -= l_sum_Vsquared_lastPeriod;                 // the next period's samples so far stay
  i_sampleSetsDuringThisDatalogPeriod -= i_sampleSetsLastPeriod;  // the same

  uint8_t i{ NO_OF_DUMPLOADS };
  do
  {
    --i;
    Shared::copyOf_countLoadON[i] = countLoadON[i];
    countLoadON[i] = 0;
  } while (i);

  Shared::copyOf_sampleSetsDuringThisDatalogPeriod = i_sampleSetsLastPeriod;               // (for diags only)
  Shared::copyOf_lowestNoOfSampleSetsPerMainsCycle = n_lowestNoOfSampleSetsPerMainsCycle;  // (for diags only)
  n_lowestNoOfSampleSetsPerMainsCycle = UINT8_MAX;
  Shared::copyOf_energyInBucket_main = l_energyInBucket_main;  // (for diags only)

  if constexpr (!DIVERTED_CT_PRESENT)
  {
    // signal the main processor that logging data are available (beyond the start-up period)
    Shared::b_datalogEventPending = true;
  }
}

/**
 * @brief Copies the CT2 figures of the datalog period that just ended, then signals the main code.
 *
 * @ingroup TimeCritical
 */
void processDivertedDataLogging()
{
  // the average over the whole period: a forced cycle counts as 0 W
  Shared::copyOf_divertedPower_W = DivertedPower::watts(l_divertedEnergy_overDL, divertedPowerAverage);
  Shared::copyOf_divertedEnergyTotal_Wh = divertedEnergy.totalWh();
  l_divertedEnergy_overDL = 0;

  // signal the main processor that logging data are available (beyond the start-up period)
  Shared::b_datalogEventPending = true;
}

/**
 * @brief The rest of the cycle's bookkeeping, the task after the CT2 energy.
 *
 * @details
 * - Adds the power of the cycle that just ended to the datalog sum.
 * - Updates the minimum number of ADC sample sets per mains cycle.
 * - Counts the cycles of the datalog period, and tells the next positive crossing whether it
 *   ends the period.
 * - Counts the seconds without diversion, and signals a new mains cycle to the main code.
 *
 * @ingroup TimeCritical
 */
void addToDatalogSum()
{
  l_sumP_atSupplyPoint[0] += l_sumP_lastCycle;  // the cycle's power into the datalog sum, 64 bits

  // A performance check to monitor and display the minimum number of sets of
  // ADC samples per mains cycle, the expected number being 20ms / (104us * 3) = 64
  if (n_samplesLastCycle < n_lowestNoOfSampleSetsPerMainsCycle)
  {
    n_lowestNoOfSampleSetsPerMainsCycle = n_samplesLastCycle;
  }

  if (++n_cycleCountForDatalogging >= DATALOG_PERIOD_IN_MAINS_CYCLES)
  {
    n_cycleCountForDatalogging = 0;
  }
  b_datalogPeriodEndsNext = (n_cycleCountForDatalogging == DATALOG_PERIOD_IN_MAINS_CYCLES - 1);

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

/**
 * @brief Runs the first pending cycle task (CycleTask), on a CT2 call.
 *
 * @details The dispatch only: each task is a call of its own (processing.h), which saves only the
 *          registers it uses.
 *
 * @ingroup TimeCritical
 */
void runNextCycleTask()
{
  const uint16_t tasks{ pendingCycleTasks };

  if (tasks & CycleTask::CONTRIBUTION)
  {
    pendingCycleTasks = tasks & ~CycleTask::CONTRIBUTION;
    processLatestContribution();
  }
  else if (tasks & CycleTask::DIVERTED)
  {
    pendingCycleTasks = tasks & ~CycleTask::DIVERTED;
    processDivertedEnergy();
  }
  else if (tasks & CycleTask::DIVERTED_COUNT)
  {
    pendingCycleTasks = tasks & ~CycleTask::DIVERTED_COUNT;
    countDivertedEnergy();
  }
  else if (tasks & CycleTask::DATALOG_SUM)
  {
    pendingCycleTasks = tasks & ~CycleTask::DATALOG_SUM;
    addToDatalogSum();
  }
  else if (tasks & CycleTask::DATALOG_COPY)
  {
    pendingCycleTasks = tasks & ~CycleTask::DATALOG_COPY;
    processDataLogging();
  }
  else if (tasks & CycleTask::DATALOG_COPY_CT2)
  {
    pendingCycleTasks = tasks & ~CycleTask::DATALOG_COPY_CT2;
    processDivertedDataLogging();
  }
  else if (tasks & CycleTask::PREDICTION)
  {
    pendingCycleTasks = tasks & ~CycleTask::PREDICTION;
    predictEndOfCycle();
  }
  else if (tasks & CycleTask::DECISION)
  {
    pendingCycleTasks = (tasks & ~CycleTask::DECISION) | CycleTask::LOAD_STATES;
    processStartNewCycle();
  }
  else if (tasks & CycleTask::LOAD_STATES)
  {
    pendingCycleTasks = (tasks & ~CycleTask::LOAD_STATES) | CycleTask::PORTS;
    updatePhysicalLoadStates();  // allows the logical-to-physical mapping to be changed
  }
  else
  {
    pendingCycleTasks = tasks & ~CycleTask::PORTS;
    applyDecision();
  }
}

/**
 * @brief Runs every pending cycle task at once, before new snapshots replace theirs.
 *
 * @details Only with a half cycle shorter than its tasks (a few sample sets), which a proper
 *          mains never gives.
 *
 * @ingroup TimeCritical
 */
void finishCycleTasks()
{
  while (pendingCycleTasks)
  {
    runNextCycleTask();
  }
}

/**
 * @brief Process the start of a new positive half cycle for the specified phase.
 *
 * This function is called just after the zero-crossing point of a positive half cycle.
 * It takes what must be taken on this very sample, and posts the rest of the work of the
 * cycle that just ended to the following CT2 calls (CycleTask).
 *
 * @param phase The phase number [0..NO_OF_PHASES[.
 *
 * @details
 * - The sums and the sample count of the cycle that just ended are taken and restarted: this
 *   sample's power and count belong to the new cycle.
 * - At the end of a datalog period, takes and restarts its V^2 sum and its sample count: this
 *   sample belongs to the next period.
 * - Posts the contribution, the CT2 energy, the rest of the bookkeeping and, at the end of a
 *   period, the copies.
 *
 * @ingroup TimeCritical
 */
void processPlusHalfCycle(const uint8_t phase)
{
  (void)phase;  // one phase (static_assert above)

  if (pendingCycleTasks)
  {
    finishCycleTasks();  // a half cycle shorter than its tasks: never on a proper mains
  }

  // The sums are taken, not cleared: the contribution task takes them off the sums, which by
  // then also hold the first samples of the new cycle (fewer cycles on this sample).
  n_samplesLastCycle = n_samplesDuringThisMainsCycle[0];
  n_samplesDuringThisMainsCycle[0] = 0;
  l_sumP_lastCycle = l_sumP[0];
  if constexpr (DIVERTED_CT_PRESENT)
  {
    l_sumP_diverted_lastCycle = l_sumP_diverted;
  }

  uint16_t tasks{ static_cast< uint16_t >(CycleTask::CONTRIBUTION | CycleTask::DATALOG_SUM | (DIVERTED_CT_PRESENT ? (CycleTask::DIVERTED | CycleTask::DIVERTED_COUNT) : 0)) };

  if (b_datalogPeriodEndsNext)
  {
    b_datalogPeriodEndsNext = false;

    // the last sample of the period: this sample belongs to the next one (taken here, taken off
    // the sums by processDataLogging())
    l_sum_Vsquared_lastPeriod = l_sum_Vsquared[0];
    i_sampleSetsLastPeriod = i_sampleSetsDuringThisDatalogPeriod;

    tasks |= CycleTask::DATALOG_COPY | (DIVERTED_CT_PRESENT ? CycleTask::DATALOG_COPY_CT2 : 0);
  }

  pendingCycleTasks = tasks;
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
        // Once per mains cycle, at mid-cycle on phase 0: the decision and the ports, on the
        // next two CT2 calls (CycleTask)
        pendingCycleTasks |= CycleTask::DECISION;
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
      refresh7SegDisplay();  // the 7-segment display, multiplexed once per sample set (as Full); nothing otherwise
      break;
    default:
      processDivertedCurrentRawSample(adc_raw);
      if (pendingCycleTasks)
      {
        runNextCycleTask();  // the work of the mains cycle, one task per CT2 call
      }
      break;
  }

  // Set ADMUX at the end of the interrupt to ensure at least 128 CPU cycles
  // have passed since the trigger event (ATmega328p datasheet requirement)
  ADMUX = _ctx->admux;

  _ctx = _ctx->next;
}  // end of ISR
