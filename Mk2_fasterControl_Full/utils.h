/**
 * @file utils.h
 * @author Fredéric Metrich (frederic.metrich@live.fr)
 * @brief Some utility functions
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include "calibration.h"
#include "constants.h"
#include "dualtariff.h"
#include "processing.h"
#include "teleinfo.h"

#include "FastDivision.h"

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

  printParamsForSelectedOutputMode();
}

/**
 * @brief Prints the load priorities to the Serial output.
 *
 */
inline void logLoadPriorities()
{
#ifdef ENABLE_DEBUG

  DBUGLN(F("Load Priorities: "));
  for (const auto& loadPrioAndState : loadPrioritiesAndState)
  {
    DBUG(F("\tload "));
    DBUGLN(loadPrioAndState);
  }

#endif
}

/**
 * @brief Prints data logs to the Serial output in text format
 *
 */
inline void printForSerialText()
{
  Serial.print(copyOf_energyInBucket_long * invSUPPLY_FREQUENCY);
  Serial.print(F(", P:"));
  Serial.print(tx_data.powerGrid);

  if constexpr (RELAY_DIVERSION)
  {
    Serial.print(F("/"));
    Serial.print(relays.get_average());
  }

  Serial.print(F(", D:"));
  Serial.print(tx_data.powerDiverted);

  Serial.print(F(", E:"));
  Serial.print(divertedEnergyTotal_Wh);

  Serial.print(F(", V"));
  Serial.print(F(":"));
  Serial.print((float)tx_data.Vrms_L_x100 * 0.01F);

  if constexpr (TEMP_SENSOR_PRESENT)
  {
    for (uint8_t idx = 0; idx < temperatureSensing.get_size(); ++idx)
    {
      if ((OUTOFRANGE_TEMPERATURE == tx_data.temperature_x100[idx])
          || (DEVICE_DISCONNECTED_RAW == tx_data.temperature_x100[idx]))
      {
        continue;
      }

      Serial.print(F(", T"));
      Serial.print(idx + 1);
      Serial.print(F(":"));
      Serial.print((float)tx_data.temperature_x100[idx] * 0.01F);
    }
  }

  Serial.print(F(", (minSampleSets/MC "));
  Serial.print(copyOf_lowestNoOfSampleSetsPerMainsCycle);
  Serial.print(F(", #ofSampleSets "));
  Serial.print(copyOf_sampleSetsDuringThisDatalogPeriod);

#ifndef DUAL_TARIFF
  if constexpr (PRIORITY_ROTATION != RotationModes::OFF)
  {
    Serial.print(F(", NoED "));
    Serial.print(absenceOfDivertedEnergyCount);
  }
#endif  // DUAL_TARIFF

  Serial.println(F(")"));
}

inline void printForSerialJson()
{
  ArduinoJson::StaticJsonDocument< 256 > doc;

  doc["P"] = tx_data.powerGrid;

  if constexpr (RELAY_DIVERSION)
  {
    doc["R"] = relays.get_average();
  }

  doc["D"] = tx_data.powerDiverted;
  doc["E"] = divertedEnergyTotal_Wh;
  doc["V"] = (float)tx_data.Vrms_L_x100 * 0.01F;

  if constexpr (TEMP_SENSOR_PRESENT)
  {
    for (uint8_t idx = 0; idx < temperatureSensing.get_size(); ++idx)
    {
      if ((OUTOFRANGE_TEMPERATURE == tx_data.temperature_x100[idx])
          || (DEVICE_DISCONNECTED_RAW == tx_data.temperature_x100[idx]))
      {
        continue;
      }
      doc[String("T") + (idx + 1)] = (float)tx_data.temperature_x100[idx] * 0.01F;
    }
  }

  if constexpr (SUPPLY_FREQUENCY == 50)
  {
    //doc["NoED"] = divu50(absenceOfDivertedEnergyCount);
  }
  else if constexpr (SUPPLY_FREQUENCY == 60)
  {
    doc["NoED"] = divu60(absenceOfDivertedEnergyCount);
  }
  else
    static_assert(SUPPLY_FREQUENCY == 50 || SUPPLY_FREQUENCY == 60, "SUPPLY_FREQUENCY must be either 50 or 60");

  serializeJson(doc, Serial);
  Serial.println();
}

void sendTelemetryData()
{
  TeleInfo teleInfo;

    teleInfo.startFrame(); // Start a new telemetry frame

    teleInfo.send("P", tx_data.powerGrid); // Send power grid data

    if constexpr (RELAY_DIVERSION)
    {
        teleInfo.send("R", static_cast<int16_t>(relays.get_average())); // Send relay average if diversion is enabled
    }

    teleInfo.send("D", tx_data.powerDiverted); // Send power diverted
    teleInfo.send("E", static_cast<int16_t>(divertedEnergyTotal_Wh)); // Send diverted energy in Wh
    teleInfo.send("V", tx_data.Vrms_L_x100); // Send voltage in volts

    if constexpr (TEMP_SENSOR_PRESENT)
    {
        for (uint8_t idx = 0; idx < temperatureSensing.get_size(); ++idx)
        {
            if ((OUTOFRANGE_TEMPERATURE == tx_data.temperature_x100[idx])
                || (DEVICE_DISCONNECTED_RAW == tx_data.temperature_x100[idx]))
            {
                continue; // Skip invalid temperature readings
            }
            teleInfo.send("T", tx_data.temperature_x100[idx], idx + 1); // Send temperature
        }
    }

    if constexpr (SUPPLY_FREQUENCY == 50)
    {
        teleInfo.send("NoED", static_cast<int16_t>(divu50(absenceOfDivertedEnergyCount))); // Send absence of diverted energy count for 50Hz
    }
    else if constexpr (SUPPLY_FREQUENCY == 60)
    {
        teleInfo.send("NoED", static_cast<int16_t>(divu60(absenceOfDivertedEnergyCount))); // Send absence of diverted energy count for 60Hz
    }
    else
    {
        static_assert(SUPPLY_FREQUENCY == 50 || SUPPLY_FREQUENCY == 60, "SUPPLY_FREQUENCY must be either 50 or 60");
    }

    teleInfo.endFrame(); // Finalize and send the telemetry frame
}

/**
 * @brief Prints data logs to the Serial output in text or json format
 *
 * @param bOffPeak true if off-peak tariff is active
 */
inline void sendResults(bool bOffPeak)
{
  static bool startup{ true };

  if (startup)
  {
    startup = false;
    return;  // reject the first datalogging which is incomplete !
  }

#ifdef RF_PRESENT
  send_rf_data();  // *SEND RF DATA*
#endif

#if defined SERIALOUT
  printForSerialJson();
#endif  // if defined SERIALOUT

  if constexpr (EMONESP_CONTROL)
  {
    //printForEmonESP(bOffPeak);
  }

#if defined SERIALPRINT && !defined EMONESP
  printForSerialText();
#endif  // if defined SERIALPRINT && !defined EMONESP
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

#endif  // UTILS_H
