/**
 * @file main.cpp
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief 
 * @version 0.1
 * @date 2023-05-13
 * 
 * @copyright Copyright (c) 2023
 * 
 */

static_assert(__cplusplus >= 201703L, "**** Please define 'gnu++17' in 'platform.txt' ! ****");
static_assert(__cplusplus >= 201703L, "See also : https://github.com/FredM67/PVRouter-3-phase/blob/main/Mk2_3phase_RFdatalog_temp/Readme.md");

#include <Arduino.h>  // may not be needed, but it's probably a good idea to include this

#include "config.h"

/**
 * @brief Called once during startup.
 * @details This function initializes a couple of variables we cannot init at compile time and
 *          sets a couple of parameters for runtime.
 *
 */
void setup() {
}

/**
 * @brief Main processor.
 * @details None of the workload in loop() is time-critical.
 *          All the processing of ADC data is done within the ISR.
 *
 */
void loop() {
  static uint8_t perSecondTimer{ 0 };
  static int16_t iTemperature_x100{ 0 };
}