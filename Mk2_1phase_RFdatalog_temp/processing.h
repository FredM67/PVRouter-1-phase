/**
 * @file processing.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Public functions/variables of processing engine
 * @version 0.1
 * @date 2026-11-21
 *
 * @copyright Copyright (c) 2021-2026
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifndef PROCESSING_H
#define PROCESSING_H

#include "config_derived.h"

inline uint8_t loadPrioritiesAndState[NO_OF_DUMPLOADS]; /**< load priorities */

inline constexpr uint8_t PERSISTENCE_FOR_POLARITY_CHANGE{ 1 }; /**< allows polarity changes to be confirmed */

inline constexpr uint16_t initialDelay{ 3000 };  /**< in milli-seconds, to allow time to open the Serial monitor */
inline constexpr uint16_t startUpPeriod{ 3000 }; /**< in milli-seconds, to allow LP filter to settle */

// one temperature per sensor, only with temperature sensing (TEMP_SENSOR_PRESENT is a constexpr, not a macro)
inline PayloadTx_struct< NO_OF_PHASES, TEMP_SENSOR_PRESENT ? tempSensors.size() : 0 > tx_data; /**< logging data */

void printParamsForSelectedOutputMode();

#if defined(__DOXYGEN__)
void initializeProcessing();
inline void processStartUp(uint8_t phase);
inline void processVoltageRawSample(const uint8_t phase, const uint16_t rawSample);
inline void processCurrentRawSample(const uint8_t phase, const uint16_t rawSample);
inline void processDivertedCurrentRawSample(const uint16_t rawSample);
inline void processPlusHalfCycle(uint8_t phase);
inline void processMinusHalfCycle(uint8_t phase);
inline void processRawSamples(const uint8_t phase);
inline void processVoltage(uint8_t phase);
inline void processPolarity(uint8_t phase, uint16_t rawSample);
inline void confirmPolarity(uint8_t phase);
void runNextCycleTask();
void finishCycleTasks();
void processLatestContribution();
int32_t rescaleToNMin(int32_t sumP, uint8_t n);
void processDivertedEnergy();
void countDivertedEnergy();
void addToDatalogSum();
void processDataLogging();
void processDivertedDataLogging();
void predictEndOfCycle();
void processStartNewCycle();
void updatePhysicalLoadStates();
void applyDecision();
inline void updatePortsStates();
#else
// -O3 on a function that is a call of its own: here 50 bytes smaller than with -Os
void initializeProcessing() __attribute__((optimize("-O3")));
inline void processStartUp(uint8_t phase) __attribute__((always_inline));
inline void processVoltageRawSample(const uint8_t phase, const uint16_t rawSample) __attribute__((always_inline));
inline void processCurrentRawSample(const uint8_t phase, const uint16_t rawSample) __attribute__((always_inline));
inline void processDivertedCurrentRawSample(const uint16_t rawSample) __attribute__((always_inline));
inline void processPlusHalfCycle(uint8_t phase) __attribute__((always_inline));
inline void processMinusHalfCycle(uint8_t phase) __attribute__((always_inline));
inline void processRawSamples(const uint8_t phase) __attribute__((always_inline));
inline void processVoltage(uint8_t phase) __attribute__((always_inline));
inline void processPolarity(uint8_t phase, uint16_t rawSample) __attribute__((always_inline));
inline void confirmPolarity(uint8_t phase) __attribute__((always_inline));
// The work of the mains cycle (CycleTask in processing.cpp), one task per CT2 call: the dispatch
// and each task are calls of their own, that save only the registers they use. The dispatch has
// two callers (the ISR and finishCycleTasks()): inlined, it would be there twice.
void runNextCycleTask() __attribute__((noinline));
void finishCycleTasks() __attribute__((noinline));  // the tasks at once: a half cycle shorter than its tasks only
void processLatestContribution() __attribute__((noinline));
int32_t rescaleToNMin(int32_t sumP, uint8_t n) __attribute__((noinline));  // its division apart: unexpected cycle lengths only
void processDivertedEnergy() __attribute__((noinline));
void countDivertedEnergy() __attribute__((noinline));
void addToDatalogSum() __attribute__((noinline));
void processDataLogging() __attribute__((noinline));
void processDivertedDataLogging() __attribute__((noinline));
void predictEndOfCycle() __attribute__((noinline));
void processStartNewCycle() __attribute__((noinline));
void updatePhysicalLoadStates() __attribute__((noinline));
void applyDecision() __attribute__((noinline));
// An inlined function is compiled with the options of the function it lands in: an optimize()
// attribute on it changes nothing. updatePortsStates() is a call: -O3 unrolls its load loop into
// constant masks (grid simulator: 38 cycles less on average, 88 less at most).
inline void updatePortsStates() __attribute__((optimize("-O3")));
#endif

#endif /* PROCESSING_H */
