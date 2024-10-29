/* Mk2_fasterControl_twoLoads_5.ino
 *
 *  (initially released as Mk2_bothDisplays_1 in March 2014)
 * This sketch is for diverting surplus PV power to a dump load using a triac or  
 * Solid State Relay. It is based on the Mk2i PV Router code that I have posted in on  
 * the OpenEnergyMonitor forum.  The original version, and other related material, 
 * can be found on my Summary Page at www.openenergymonitor.org/emon/node/1757
 *
 * In this latest version, the pin-allocations have been changed to suit my 
 * PCB-based hardware for the Mk2 PV Router.  The integral voltage sensor is 
 * fed from one of the secondary coils of the transformer.  Current is measured 
 * via Current Transformers at the CT1 and CT1 ports.  
 * 
 * CT1 is for 'grid' current, to be measured at the grid supply point.
 * CT2 is for the load current, so that diverted energy can be recorded
 *
 * A persistence-based 4-digit display is supported. This can be driven in two
 * different ways, one with an extra pair of logic chips, and one without.  The 
 * appropriate version of the sketch must be selected by including or commenting 
 * out the "#define PIN_SAVING_HARDWARE" statement near the top of the code.
 *
 * September 2014: renamed as Mk2_bothDisplays_2, with these changes:
 * - cycleCount removed (was not actually used in this sketch, but could have overflowed);
 * - removal of unhelpful comments in the IO pin section;
 * - tidier initialisation of display logic in setup();
 * - addition of REQUIRED_EXPORT_IN_WATTS logic (useful as a built-in PV simulation facility);
 *
 * December 2014: renamed as Mk2_bothDisplays_3, with these changes:
 * - persistence check added for zero-crossing detection (polarityConfirmed)
 * - lowestNoOfSampleSetsPerMainsCycle added, to check for any disturbances
 *
 * December 2014: renamed as Mk2_bothDisplays_3a, with some typographical errors fixed.
 *
 * January 2016: renamed as Mk2_bothDisplays_3b, with a minor change in the ISR to 
 *   remove a timing uncertainty.
 *
 * January 2016: updated to Mk2_bothDisplays_3c:
 *   The variables to store the ADC results are now declared as "volatile" to remove 
 *   any possibility of incorrect operation due to optimisation by the compiler.
 *
 * February 2016: updated to Mk2_bothDisplays_4, with these changes:
 * - improvements to the start-up logic.  The start of normal operation is now 
 *    synchronized with the start of a new mains cycle.
 * - reduce the amount of feedback in the Low Pass Filter for removing the DC content
 *     from the Vsample stream. This resolves an anomaly which has been present since 
 *     the start of this project.  Although the amount of feedback has previously been 
 *     excessive, this anomaly has had minimal effect on the system's overall behaviour.
 * - removal of the unhelpful "triggerNeedsToBeArmed" mechanism
 * - tidying of the "confirmPolarity" logic to make its behaviour more clear
 * - SWEETZONE_IN_JOULES changed to WORKING_ZONE_IN_JOULES 
 * - change "triac" to "load" wherever appropriate
 *
 * November 2019: updated to Mk2_fasterControl_1 with these changes:
 * - Half way through each mains cycle, a prediction is made of the likely energy level at the
 *   end of the cycle.  That predicted value allows the triac to be switched at the +ve going 
 *   zero-crossing point rather than waiting for a further 10 ms.  These changes allow for 
 *   faster switching of the load.
 * - The range of the energy bucket has been reduced to one tenth of its former value. This
 *   allows the unit's operation to commence more rapidly whenever surplus power is available.
 * - controlMode is no longer selectable, the unit's operation being effectively hard-coded 
 *   as "Normal" rather than Anti-flicker. 
 * - Port D3 now supports an indicator which shows when the level in the energy bucket
 *   reaches either end of its range.  While the unit is actively diverting surplus power,
 *   it is vital that the level in the reduced capacity energy bucket remains within its 
 *   permitted range, hence the addition of this indicator.
 *   
 * February 2020: updated to Mk2_fasterControl_1a with these changes:
 * - removal of some redundant code in the logic for determining the next load state.
 * 
 * March 2021: updated to Mk2_fasterControl_2 with these changes:
 * - extra filtering added to offset the HPF effect of CT1.  This allows the energy state in
 *   10 ms time to be predicted with more confidence.  Specifically, it is no longer necessary 
 *   to include a 30% boost factor after each change of load state.
 * 
 * June 2021: updated to Mk2_fasterControl_3 with these changes:
 * - to reflect the performance of recently manufactured YHDC SCT_013_000 CTs, 
 *   the value of the parameter lpf_gain has been reduced from 12 to 8.
 *   
 * July 2023: updated to Mk2_fasterControl_twoLoads_5 with these changes:
 * - the ability to control two loads has been transferred from the latest version of my
 *   standard multiLoad sketch, Mk2_multiLoad_wired_7a.  The faster control algorithm  
 *   has been retained.  
 *      The previous 2-load "faster control" sketch (version 4) has been archived as its 
 *   behaviour was found to be problematic.
 *   
 *      Robin Emley
 *      www.Mk2PVrouter.co.uk
 * 
 * __October 2024, renamed as Mk2_fasterControl_Full with these changes:__
 * - heavy refactoring to make the code more readable and maintainable.
 * - added comments to explain the code.
 * - added assertions to ensure the code is compiled with the correct C++ version.
 * - added a link to the Readme.md file in the comments.
 * - refactoring of the ISR to enhance the performance.
 */

static_assert(__cplusplus >= 201703L, "**** Please define 'gnu++17' in 'platform.txt' ! ****");
static_assert(__cplusplus >= 201703L, "See also : https://github.com/FredM67/PVRouter-3-phase/blob/main/Mk2_3phase_RFdatalog_temp/Readme.md");

// The active code can be found in the other cpp/h files

#include <Arduino.h>

#include "calibration.h"
#include "config.h"
#include "processing.h"
#include "utils_pins.h"
#include "utils_display.h"

LoadStates logicalLoadState[NO_OF_DUMPLOADS];
LoadStates physicalLoadState[NO_OF_DUMPLOADS];

// For this go-faster version, the unit's operation will effectively always be "Normal";
// there is no "Anti-flicker" option. The controlMode variable has been removed.

constexpr int16_t DCoffset_I{ 512 };  // nominal mid-point value of ADC @ x1 scale

// General global variables that are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'
//
bool beyondStartUpPhase = false;  // start-up delay, allows things to settle

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
int32_t energyInBucket_long{ 0 };                                                                                                          // in Integer Energy Units
constexpr int32_t capacityOfEnergyBucket_long{ static_cast< int32_t >(WORKING_ZONE_IN_JOULES * SUPPLY_FREQUENCY * (1 / powerCal_grid)) };  // depends on powerCal, frequency & the 'sweetzone' size.
constexpr int32_t midPointOfEnergyBucket_long{ capacityOfEnergyBucket_long >> 1 };                                                         // used for 'normal' and single-threshold 'AF' logic

constexpr int32_t lowerThreshold_default{ capacityOfEnergyBucket_long >> 1 };
constexpr int32_t upperThreshold_default{ capacityOfEnergyBucket_long >> 1 };

// Define operating limits for the LP filter which identifies DC offset in the voltage
// sample stream.  By limiting the output range, the filter always should start up
// correctly.
int32_t DCoffset_V_long{ 512L * 256 };                   // nominal mid-point value of ADC @ x256 scale
constexpr int32_t DCoffset_V_min{ (512L - 100) * 256 };  // mid-point of ADC minus a working margin
constexpr int32_t DCoffset_V_max{ (512L + 100) * 256 };  // mid-point of ADC plus a working margin

// For recording the accumulated amount of diverted energy data (using CT2), a similar
// calibration mechanism is required.  Rather than a bucket with a fixed capacity, the
// accumulator for diverted energy just needs to be scaled correctly.  As soon as its
// value exceeds 1 Wh, an associated WattHour register is incremented, and the
// accumulator's value is decremented accordingly. The calculation below is to determine
// the scaling for this accumulator.

// to avoid the diverted energy accumulator 'creeping' when the load is not active
constexpr int32_t antiCreepLimit_inIEUperMainsCycle{ static_cast< int32_t >(ANTI_CREEP_LIMIT * (1 / powerCal_grid)) };

constexpr int32_t requiredExportPerMainsCycle_inIEU{ static_cast< int32_t >(REQUIRED_EXPORT_IN_WATTS * (1 / powerCal_grid)) };

int32_t divertedEnergyRecent_IEU{ 0 };                                                                                      // Hi-res accumulator of limited range
uint16_t divertedEnergyTotal_Wh{ 0 };                                                                                       // WattHour register of 63K range
constexpr int32_t IEU_per_Wh{ static_cast< int32_t >(JOULES_PER_WATT_HOUR * SUPPLY_FREQUENCY * (1 / powerCal_diverted)) };  // depends on powerCal, frequency & the 'sweetzone' size.

uint32_t absenceOfDivertedEnergyCount{ 0 };

int32_t lowerEnergyThreshold;
int32_t upperEnergyThreshold;

bool recentTransition = false;
uint8_t postTransitionCount;
constexpr uint8_t POST_TRANSITION_MAX_COUNT{ 3 }; /**< allows each transition to take effect */
uint8_t activeLoad{ 0 };

// for interaction between the main processor and the ISRs
volatile bool dataReady{ false };
volatile int16_t sampleI_grid;
volatile int16_t sampleI_diverted;
volatile int16_t sampleV;

// For an enhanced polarity detection mechanism, which includes a persistence check
inline constexpr uint8_t PERSISTENCE_FOR_POLARITY_CHANGE{ 1 }; /**< allows polarity changes to be confirmed */

Polarities polarityOfMostRecentVsample;
Polarities polarityConfirmed;
Polarities polarityConfirmedOfLastSampleV;

// For a mechanism to check the continuity of the sampling sequence
inline constexpr uint16_t CONTINUITY_CHECK_MAXCOUNT{ 250 }; /**< mains cycles */
uint16_t sampleCount_forContinuityChecker;
uint16_t sampleSetsDuringThisMainsCycle;    /**< number of sample sets during each mains cycle */
uint16_t lowestNoOfSampleSetsPerMainsCycle; /**< For a mechanism to check the integrity of this code structure */

// for this go-faster sketch, the phaseCal logic has been removed.  If required, it can be
// found in most of the standard Mk2_bothDisplay_n versions

bool EDD_isActive = false;  // energy diversion detection

int32_t sumP_grid;                 // for per-cycle summation of 'real power'
int32_t sumP_diverted;             // for per-cycle summation of 'real power'
int32_t cumVdeltasThisCycle_long;  // for the LPF which determines DC offset (voltage)

int32_t realEnergy_grid{ 0 };
int32_t realEnergy_diverted{ 0 };
int32_t energyInBucket_prediction{ 0 };
int32_t sampleVminusDC_long{ 0 };

// extra items for an LPF to improve the processing of data samples from CT1
int32_t lpf_long;  // new LPF, for offsetting the behaviour of CT1 as a HPF

uint16_t sampleSetsDuringNegativeHalfOfMainsCycle{ 0 };  // for arming the triac/trigger

void processPolarity(const int16_t rawSample)
{
  // remove DC offset from the raw voltage sample by subtracting the accurate value
  // as determined by a LP filter.
  sampleVminusDC_long = ((long)rawSample << 8) - DCoffset_V_long;
  // determine the polarity of the latest voltage sample
  polarityOfMostRecentVsample = (sampleVminusDC_long > 0) ? Polarities::POSITIVE : Polarities::NEGATIVE;
}

void processPlusHalfCycle()
{
  processLatestContribution();

  // a simple routine for checking the performance of this new ISR structure
  if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle)
  {
    lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;
  }

  // clear the per-cycle accumulators for use in this new mains cycle.
  sampleSetsDuringThisMainsCycle = 0;
  sumP_grid = 0;
  sumP_diverted = 0;
  sampleSetsDuringNegativeHalfOfMainsCycle = 0;
}

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

void processStartUp()
{
  // wait until the DC-blocking filters have had time to settle
  if (millis() <= (delayBeforeSerialStarts + startUpPeriod))
  {
    return;
  }

  beyondStartUpPhase = true;
  sumP_grid = 0;
  sumP_diverted = 0;
  sampleSetsDuringThisMainsCycle = 0;    // not yet dealt with for this cycle
  sampleCount_forContinuityChecker = 1;  // opportunity has been missed for this cycle
  lowestNoOfSampleSetsPerMainsCycle = 999;
  // can't say "Go!" here 'cos we're in an ISR!
}

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
  long realPower_grid = sumP_grid / sampleSetsDuringThisMainsCycle;          // proportional to Watts
  long realPower_diverted = sumP_diverted / sampleSetsDuringThisMainsCycle;  // proportional to Watts

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
}

void processVoltage()
{
  long filtV_div4 = sampleVminusDC_long >> 2;  // reduce to 16-bits (now x64, or 2^6)

  // store items for use during next loop
  cumVdeltasThisCycle_long += sampleVminusDC_long;     // for use with LP filter
  polarityConfirmedOfLastSampleV = polarityConfirmed;  // for identification of half cycle boundaries

  ++sampleSetsDuringThisMainsCycle;
}

void processGridCurrentRawSample()
{
  // First, deal with the power at the grid connection point (as measured via CT1)
  // remove most of the DC offset from the current sample (the precise value does not matter)
  int32_t sampleIminusDC_grid = ((int32_t)(sampleI_grid - DCoffset_I)) << 8;
  //
  // extra filtering to offset the HPF effect of CT1
  int32_t last_lpf_long = lpf_long;
  lpf_long = last_lpf_long + alpha * (sampleIminusDC_grid - last_lpf_long);
  sampleIminusDC_grid += (lpf_gain * lpf_long);

  // calculate the "real power" in this sample pair and add to the accumulated sum
  const int32_t filtV_div4 = sampleVminusDC_long >> 2;  // reduce to 16-bits (now x64, or 2^6)
  const int32_t filtI_div4 = sampleIminusDC_grid >> 2;  // reduce to 16-bits (now x64, or 2^6)
  int32_t instP = filtV_div4 * filtI_div4;              // 32-bits (now x4096, or 2^12)
  instP = instP >> 12;                                  // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_grid += instP;                                   // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
}

void processDivertedCurrentRawSample()
{
  // Now deal with the diverted power (as measured via CT2)
  // remove most of the DC offset from the current sample (the precise value does not matter)
  int32_t sampleIminusDC_diverted = ((int32_t)(sampleI_diverted - DCoffset_I)) << 8;

  // calculate the "real power" in this sample pair and add to the accumulated sum
  const int32_t filtV_div4 = sampleVminusDC_long >> 2;      // reduce to 16-bits (now x64, or 2^6)
  const int32_t filtI_div4 = sampleIminusDC_diverted >> 2;  // reduce to 16-bits (now x64, or 2^6)
  int32_t instP = filtV_div4 * filtI_div4;                  // 32-bits (now x4096, or 2^12)
  instP = instP >> 12;                                      // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_diverted += instP;                                   // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)
}

void proceedHighEnergyLevel()
{
  bool OK_toAddLoad = true;
  uint8_t tempLoad = nextLogicalLoadToBeAdded();
  if (tempLoad < NO_OF_DUMPLOADS)
  {
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
      if (tempLoad != activeLoad)
      {
        OK_toAddLoad = false;
      }
    }

    if (OK_toAddLoad)
    {
      logicalLoadState[tempLoad] = LoadStates::LOAD_ON;
      activeLoad = tempLoad;
      postTransitionCount = 0;
      recentTransition = true;
    }
  }
}

void proceedLowEnergyLevel()
{
  bool OK_toRemoveLoad = true;
  uint8_t tempLoad = nextLogicalLoadToBeRemoved();
  if (tempLoad < NO_OF_DUMPLOADS)
  {
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
      if (tempLoad != activeLoad)
      {
        OK_toRemoveLoad = false;
      }
    }

    if (OK_toRemoveLoad)
    {
      logicalLoadState[tempLoad] = LoadStates::LOAD_OFF;
      activeLoad = tempLoad;
      postTransitionCount = 0;
      recentTransition = true;
    }
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

// This routine is called to process each set of V & I samples. The main processor and
// the ADC work autonomously, their operation being only linked via the dataReady flag.
// As soon as a new set of data is made available by the ADC, the main processor can
// start to work on it immediately.
//
void allGeneralProcessing()
{
  static uint8_t timerForDisplayUpdate = 0;
  static enum LoadStates nextStateOfLoad = LoadStates::LOAD_OFF;

  processPolarity(sampleV);
  confirmPolarity();

  if (polarityConfirmed == Polarities::POSITIVE)
  {
    if (polarityConfirmedOfLastSampleV != Polarities::POSITIVE)
    {
      // This is the start of a new +ve half cycle (just after the zero-crossing point)
      if (beyondStartUpPhase)
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
            ++divertedEnergyTotal_Wh;
          }
        }

        if (timerForDisplayUpdate > UPDATE_PERIOD_FOR_DISPLAYED_DATA)
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
        else
        {
          ++timerForDisplayUpdate;
        }

        // continuity checker
        ++sampleCount_forContinuityChecker;
        if (sampleCount_forContinuityChecker >= CONTINUITY_CHECK_MAXCOUNT)
        {
          sampleCount_forContinuityChecker = 0;
          Serial.println(lowestNoOfSampleSetsPerMainsCycle);
          lowestNoOfSampleSetsPerMainsCycle = 999;
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
      if (beyondStartUpPhase)
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
        if (physicalLoadState[0] == LoadStates::LOAD_ON)
        {
          absenceOfDivertedEnergyCount = 0;
          EDD_isActive = true;
        }
        else
        {
          ++absenceOfDivertedEnergyCount;
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

  // processing for EVERY set of samples
  //
  processVoltage();

  processGridCurrentRawSample();

  refreshDisplay();
}
//  ----- end of main Mk2i code -----

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

void processVoltageRawSample(const int16_t rawSample)
{
  processPolarity(rawSample);
  confirmPolarity();

  processRawSamples();

  processVoltage();
}


#if !defined(__DOXYGEN__)
uint8_t nextLogicalLoadToBeAdded() __attribute__((optimize("-O3")));
#endif

uint8_t nextLogicalLoadToBeAdded()
{
  for (uint8_t index = 0; index < NO_OF_DUMPLOADS; ++index)
  {
    if (logicalLoadState[index] == LoadStates::LOAD_OFF)
    {
      return (index);
    }
  }

  return (NO_OF_DUMPLOADS);
}

#if !defined(__DOXYGEN__)
uint8_t nextLogicalLoadToBeRemoved() __attribute__((optimize("-O3")));
#endif

uint8_t nextLogicalLoadToBeRemoved()
{
  uint8_t index{ NO_OF_DUMPLOADS };

  do
  {
    if (logicalLoadState[--index] == LoadStates::LOAD_ON)
    {
      return (index);
    }
  } while (index);

  return (NO_OF_DUMPLOADS);
}


void updatePhysicalLoadStates()
/*
 * This function provides the link between the logical and physical loads.  The 
 * array, logicalLoadState[], contains the on/off state of all logical loads, with 
 * element 0 being for the one with the highest priority.  The array, 
 * physicalLoadState[], contains the on/off state of all physical loads. 
 * 
 * The association between the physical and logical loads is 1:1.  By default, numerical
 * equivalence is maintained, so logical(N) maps to physical(N).  If physical load 1 is set 
 * to have priority, rather than physical load 0, the logical-to-physical association for 
 * loads 0 and 1 are swapped.
 *
 * Any other mapping relationships could be configured here.
 */
{
  for (int16_t i = 0; i < NO_OF_DUMPLOADS; ++i)
  {
    physicalLoadState[i] = logicalLoadState[i];
  }

  /*
  if (loadPriorityMode == LOAD_1_HAS_PRIORITY)
  {
    // swap physical loads 0 & 1 if remote load has priority 
    physicalLoadState[0] = logicalLoadState[1];
    physicalLoadState[1] = logicalLoadState[0];
  } 
*/
}
