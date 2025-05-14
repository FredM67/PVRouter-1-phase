/**
 * @file processing.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Public functions/variables of processing engine
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef PROCESSING_H
#define PROCESSING_H

#include "shared_var.h"

void initializeProcessing();
void updatePhysicalLoadStates();
void updatePortsStates();
void printParamsForSelectedOutputMode();
void initializeOldPCBPins();
void logLoadPriorities();

#if defined(__DOXYGEN__)
inline void processStartUp();
inline void processStartNewCycle();
inline void processPlusHalfCycle();
inline void processMinusHalfCycle();
inline void processVoltageRawSample(int16_t rawSample);
inline void processRawSamples();
inline void processVoltage();
inline void processPolarity(int16_t rawSample);
inline void confirmPolarity();
inline void processGridCurrentRawSample(int16_t rawSample);
inline void processDivertedCurrentRawSample(int16_t rawSample);
inline void proceedLowEnergyLevel();
inline void proceedHighEnergyLevel();
inline uint8_t nextLogicalLoadToBeAdded();
inline uint8_t nextLogicalLoadToBeRemoved();
inline void processLatestContribution();
inline void processDataLogging();
#else
inline void processStartUp() __attribute__((always_inline));
inline void processStartNewCycle() __attribute__((always_inline));
inline void processPlusHalfCycle() __attribute__((always_inline));
inline void processMinusHalfCycle() __attribute__((always_inline));
inline void processVoltageRawSample(int16_t rawSample) __attribute__((always_inline));
inline void processRawSamples() __attribute__((always_inline));
inline void processVoltage() __attribute__((always_inline));
inline void processPolarity(int16_t rawSample) __attribute__((always_inline));
inline void confirmPolarity() __attribute__((always_inline));
inline void processGridCurrentRawSample(int16_t rawSample) __attribute__((always_inline));
inline void processDivertedCurrentRawSample(int16_t rawSample) __attribute__((always_inline));
inline void proceedLowEnergyLevel() __attribute__((always_inline));
inline void proceedHighEnergyLevel() __attribute__((always_inline));
inline uint8_t nextLogicalLoadToBeAdded() __attribute__((always_inline, optimize("-O3")));
inline uint8_t nextLogicalLoadToBeRemoved() __attribute__((always_inline, optimize("-O3")));
inline void processLatestContribution() __attribute__((always_inline));
inline void processDataLogging() __attribute__((always_inline, optimize("-O3")));

void handlePerSecondTasks(bool& bOffPeak, int16_t iTemperature_x100) __attribute__((always_inline));
#endif

#endif /* PROCESSING_H */
