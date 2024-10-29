#ifndef _UTILS_H
#define _UTILS_H

#include "calibration.h"
#include "constants.h"
#include "processing.h"

/**
 * @brief Print the configuration during start
 *
 */
inline void printConfiguration()
{
#ifndef PROJECT_PATH
#define PROJECT_PATH (__FILE__)
#endif

#ifndef BRANCH_NAME
#define BRANCH_NAME ("N/A")
#endif
#ifndef COMMIT_HASH
#define COMMIT_HASH ("N/A")
#endif

  DBUGLN();
  DBUGLN();
  DBUGLN(F("----------------------------------"));
  DBUG(F("Sketch ID: "));
  DBUGLN(F(PROJECT_PATH));

  DBUG(F("From branch '"));
  DBUG(F(BRANCH_NAME));
  DBUG(F("', commit "));
  DBUGLN(F(COMMIT_HASH));

  DBUG(F("Build on "));
#ifdef CURRENT_TIME
  DBUGLN(F(CURRENT_TIME));
#else
  DBUG(F(__DATE__));
  DBUG(F(" "));
  DBUGLN(F(__TIME__));
#endif
  DBUGLN(F("ADC mode:       free-running"));

  DBUGLN(F("Electrical settings"));

  DBUG(F("\tf_powerCal for Grid"));
  DBUG(F(" =    "));
  DBUGLN(powerCal_grid, 6);
  DBUG(F("\tf_powerCal for Diversion"));
  DBUG(F(" =    "));
  DBUGLN(powerCal_diverted, 6);

  DBUG("\tAnti-creep limit (Joules / mains cycle) = ");
  DBUGLN(ANTI_CREEP_LIMIT);
  DBUG("\tExport rate (Watts) = ");
  DBUGLN(REQUIRED_EXPORT_IN_WATTS);

  DBUG("\tzero-crossing persistence (sample sets) = ");
  DBUGLN(PERSISTENCE_FOR_POLARITY_CHANGE);
  DBUG("\tcontinuity sampling display rate (mains cycles) = ");
  DBUGLN(CONTINUITY_CHECK_MAXCOUNT);

  DBUG("\tcapacityOfEnergyBucket_long = ");
  DBUGLN(capacityOfEnergyBucket_long);
}

/**
 * @brief Get the available RAM during setup
 *
 * @return int The amount of free RAM
 */
inline int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

#endif /* _UTILS_H */
