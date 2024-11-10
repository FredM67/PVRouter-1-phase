/**
 * @file processing.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Public functions/variables of processing engine
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PROCESSING_H
#define PROCESSING_H

#include "config.h"

// allocation of analogue pins which are not dependent on the display type that is in use
// **************************************************************************************
inline constexpr uint8_t voltageSensor{ 0 };           // A0 is for the voltage sensor
inline constexpr uint8_t currentSensor_grid{ 1 };      // A1 is for CT1 which measures grid current
inline constexpr uint8_t currentSensor_diverted{ 3 };  // A3 is for CT2 which measures diverted current
// ------------------------------------------

inline uint8_t loadPrioritiesAndState[NO_OF_DUMPLOADS]; /**< load priorities */

// For an enhanced polarity detection mechanism, which includes a persistence check
inline constexpr uint8_t PERSISTENCE_FOR_POLARITY_CHANGE{ 1 }; /**< allows polarity changes to be confirmed */

inline constexpr uint16_t delayBeforeSerialStarts{ 1000 };  // in milli-seconds, to allow Serial window to be opened
inline constexpr uint16_t startUpPeriod{ 3000 };            // in milli-seconds, to allow LP filter to settle

// for interaction between the main processor and the ISR
inline volatile uint32_t absenceOfDivertedEnergyCount{ 0 }; /**< number of main cycles without diverted energy */
inline volatile bool b_datalogEventPending{ false };        /**< async trigger to signal datalog is available */
inline volatile bool b_newCycle{ false };                   /**< async trigger to signal start of new main cycle based on first phase */
inline volatile bool b_overrideLoadOn[NO_OF_DUMPLOADS];     /**< async trigger to force specific load(s) to ON */
inline volatile bool b_reOrderLoads{ false };               /**< async trigger for loads re-ordering */
inline volatile bool b_diversionOff{ false };               /**< async trigger to stop diversion */
inline volatile bool EDD_isActive{ false };                 /**< energy diversion detection */

inline volatile int32_t divertedEnergyRecent_IEU{ 0 };  // Hi-res accumulator of limited range
inline volatile uint16_t divertedEnergyTotal_Wh{ 0 };   // WattHour register of 63K range

// since there's no real locking feature for shared variables, a couple of data
// generated from inside the ISR are copied from time to time to be passed to the
// main processor. When the data are available, the ISR signals it to the main processor.
inline volatile int32_t copyOf_sumP_grid_overDL_Period;            /**< copy of cumulative grid power */
inline volatile int32_t copyOf_sumP_diverted_overDL_Period;        /**< copy of cumulative diverted power */
inline volatile int32_t copyOf_sum_Vsquared;                       /**< copy of for summation of V^2 values during datalog period */
inline volatile int32_t copyOf_energyInBucket_long;                /**< copy of main energy bucket (over all phases) */
inline volatile uint8_t copyOf_lowestNoOfSampleSetsPerMainsCycle;  /**<  */
inline volatile uint16_t copyOf_sampleSetsDuringThisDatalogPeriod; /**< copy of for counting the sample sets during each datalogging period */
inline volatile uint16_t copyOf_countLoadON[NO_OF_DUMPLOADS];      /**< copy of number of cycle the load was ON (over 1 datalog period) */

#ifdef TEMP_ENABLED
inline PayloadTx_struct< temperatureSensing.get_size() > tx_data; /**< logging data */
#else
inline PayloadTx_struct<> tx_data; /**< logging data */
#endif

void initializeProcessing();
void initializeOptionalPins();
void updatePhysicalLoadStates();
void updatePortsStates();
void printParamsForSelectedOutputMode();

void processGridCurrentRawSample(int16_t rawSample);
void processDivertedCurrentRawSample(int16_t rawSample);
void processVoltageRawSample(int16_t rawSample);
void processRawSamples();

void processVoltage();

#if defined(__DOXYGEN__)
inline void processStartUp();
inline void processStartNewCycle();
inline void processPlusHalfCycle();
inline void processMinusHalfCycle();
inline void processVoltage();
inline void processPolarity(int16_t rawSample);
inline void confirmPolarity();
inline void proceedLowEnergyLevel();
inline void proceedHighEnergyLevel();
inline uint8_t nextLogicalLoadToBeAdded();
inline uint8_t nextLogicalLoadToBeRemoved();
inline void processLatestContribution();
#else
inline void processStartUp() __attribute__((always_inline));
inline void processStartNewCycle() __attribute__((always_inline));
inline void processPlusHalfCycle() __attribute__((always_inline));
inline void processMinusHalfCycle() __attribute__((always_inline));
inline void processVoltage() __attribute__((always_inline));
inline void processPolarity(int16_t rawSample) __attribute__((always_inline));
inline void confirmPolarity() __attribute__((always_inline));
inline void proceedLowEnergyLevel() __attribute__((always_inline));
inline void proceedHighEnergyLevel() __attribute__((always_inline));
inline uint8_t nextLogicalLoadToBeAdded() __attribute__((always_inline));
inline uint8_t nextLogicalLoadToBeRemoved() __attribute__((always_inline));
inline void processLatestContribution() __attribute__((always_inline));
#endif

void processDataLogging();

#endif  // PROCESSING_H
