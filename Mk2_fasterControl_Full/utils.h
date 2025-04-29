/**
 * @file utils.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @author Frédéric Metrich (frederic.metrich@live.fr)
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

#include "FastDivision.h"

#include "calibration.h"
#include "constants.h"
#include "dualtariff.h"
#include "processing.h"
#include "teleinfo.h"

#include "utils_temp.h"

#include "version.h"

#include "utils_temp.h"

#include "version.h"

#if TEMP_SENSOR_PRESENT
inline PayloadTx_struct< temperatureSensing.get_size() > tx_data; /**< logging data */
#else
inline PayloadTx_struct<> tx_data; /**< logging data */
#endif

/**
 * @brief Print the configuration during startup.
 *
 * This function outputs the system configuration to the Serial output during startup.
 * It includes details about the sketch, build information, electrical settings, and
 * enabled features.
 *
 * @details
 * - Prints the sketch ID, branch name, commit hash, and build date/time.
 * - Outputs electrical settings such as power calibration, voltage calibration, and phase calibration.
 * - Displays enabled features like temperature sensing, dual tariff, load rotation, relay diversion, and RF communication.
 * - Logs the selected datalogging format (Human-readable, IoT, or JSON).
 *
 * @ingroup Initialization
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

  DBUG(F("Datalogging capability "));
  if constexpr (SERIAL_OUTPUT_TYPE == SerialOutputType::HumanReadable)
  {
    DBUGLN(F("in Human-readable format"));
  }
  else if constexpr (SERIAL_OUTPUT_TYPE == SerialOutputType::IoT)
  {
    DBUGLN(F("in IoT format"));
  }
  else if constexpr (SERIAL_OUTPUT_TYPE == SerialOutputType::JSON)
  {
    DBUGLN(F("in JSON format"));
  }
  else
  {
    DBUGLN(F("is NOT present"));
  }
}

/**
 * @brief Prints data logs to the Serial output in text format.
 *
 * This function outputs telemetry data in a human-readable text format to the Serial output.
 * It includes information about power, voltage, temperature, and system performance metrics.
 *
 * @details
 * - Prints total power, phase-specific power, and RMS voltage for each phase.
 * - Includes temperature data if temperature sensing is enabled.
 * - Outputs additional system metrics like the number of sample sets and absence of diverted energy count.
 *
 * @ingroup Telemetry
 */
inline void printForSerialText()
{
  Serial.print(Shared::copyOf_energyInBucket_long * invSUPPLY_FREQUENCY);
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
  Serial.print(Shared::copyOf_divertedEnergyTotal_Wh_forDL);

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
  Serial.print(Shared::copyOf_lowestNoOfSampleSetsPerMainsCycle);
  Serial.print(F(", #ofSampleSets "));
  Serial.print(Shared::copyOf_sampleSetsDuringThisDatalogPeriod);

#ifndef DUAL_TARIFF
  if constexpr (PRIORITY_ROTATION != RotationModes::OFF)
  {
    Serial.print(F(", NoED "));
    Serial.print(Shared::absenceOfDivertedEnergyCountInSeconds);
  }
#endif  // DUAL_TARIFF

  Serial.println(F(")"));
}

/**
 * @brief Write telemetry data to Serial in JSON format.
 *
 * This function outputs telemetry data in a format compatible with JSON, including
 * power, voltage, load states, temperature, and tariff information.
 *
 * @param bOffPeak Indicates whether the system is in an off-peak tariff period.
 *
 * @details
 * - Outputs total power and phase-specific power.
 * - Includes load ON percentages for each load.
 * - Outputs temperature data if temperature sensing is enabled.
 * - Includes tariff information if dual tariff is enabled.
 *
 * @ingroup Telemetry
 */
inline void printForJSON(const bool bOffPeak)
{
  ArduinoJson::StaticJsonDocument< 256 > doc;

  doc["P"] = tx_data.powerGrid;

  if constexpr (RELAY_DIVERSION)
  {
    doc["R"] = relays.get_average();
  }

  doc["D"] = tx_data.powerDiverted;
  doc["E"] = Shared::copyOf_divertedEnergyTotal_Wh_forDL;
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

  doc["NoED"] = Shared::absenceOfDivertedEnergyCountInSeconds;

  serializeJson(doc, Serial);
  Serial.println();
}

/**
 * @brief Sends telemetry data using the TeleInfo class.
 *
 * This function collects various telemetry data (e.g., power, voltage, temperature, etc.)
 * and sends it in a structured format using the `TeleInfo` class. The data is sent as a
 * telemetry frame, which starts with a frame initialization, includes multiple data points,
 * and ends with a frame finalization.
 *
 * The function supports conditional features such as relay diversion, temperature sensing,
 * and different supply frequencies (50 Hz or 60 Hz).
 *
 * @details
 * - **Power Data**: Sends the total power grid data.
 * - **Relay Data**: If relay diversion is enabled (`RELAY_DIVERSION`), sends the average relay data.
 * - **Voltage Data**: Sends the voltage data for each phase.
 * - **Temperature Data**: If temperature sensing is enabled (`TEMP_SENSOR_PRESENT`), sends valid temperature readings.
 * - **Absence of Diverted Energy Count**: The amount of seconds without diverting energy.
 *
 * @note The function uses compile-time constants (`constexpr`) to include or exclude specific features.
 *       Invalid temperature readings (e.g., `OUTOFRANGE_TEMPERATURE` or `DEVICE_DISCONNECTED_RAW`) are skipped.
 *
 * @throws static_assert If `SUPPLY_FREQUENCY` is not 50 or 60 Hz.
 */
void sendTelemetryData()
{
  static TeleInfo teleInfo;

  teleInfo.startFrame();  // Start a new telemetry frame

  teleInfo.send("P", tx_data.powerGrid);  // Send power grid data

  if constexpr (RELAY_DIVERSION)
  {
    teleInfo.send("R", static_cast< int16_t >(relays.get_average()));  // Send relay average if diversion is enabled

    uint8_t idx = 0;
    do
    {
      teleInfo.send("R", relays.get_relay(idx).isRelayON());  // Send diverted energy count for each relay
    } while (++idx < relays.get_size());
  }

  teleInfo.send("V", tx_data.Vrms_L_x100);                                    // Send voltage in volts
  teleInfo.send("S", Shared::copyOf_sampleSetsDuringThisDatalogPeriod);
  teleInfo.send("S_MC", Shared::copyOf_lowestNoOfSampleSetsPerMainsCycle);

  teleInfo.send("D", tx_data.powerDiverted);                                  // Send power diverted
  teleInfo.send("E", static_cast< int16_t >(Shared::copyOf_divertedEnergyTotal_Wh_forDL));  // Send diverted energy in Wh

  if constexpr (TEMP_SENSOR_PRESENT)
  {
    for (uint8_t idx = 0; idx < temperatureSensing.get_size(); ++idx)
    {
      if ((OUTOFRANGE_TEMPERATURE == tx_data.temperature_x100[idx])
          || (DEVICE_DISCONNECTED_RAW == tx_data.temperature_x100[idx]))
      {
        continue;  // Skip invalid temperature readings
      }
      teleInfo.send("T", tx_data.temperature_x100[idx], idx + 1);  // Send temperature
    }
  }

  teleInfo.send("N", static_cast< int16_t >(Shared::absenceOfDivertedEnergyCountInSeconds));  // Send absence of diverted energy count for 50Hz

  teleInfo.endFrame();  // Finalize and send the telemetry frame
}

/**
 * @brief Prints or sends telemetry data logs based on the selected output format.
 *
 * This function handles the transmission of telemetry data in various formats, such as
 * human-readable text, IoT telemetry, or JSON format. It also ensures that the first
 * incomplete datalogging event is skipped during startup.
 *
 * @param bOffPeak Indicates whether the system is in an off-peak tariff period.
 *
 * @details
 * - If RF communication is enabled, it sends RF data.
 * - Depending on the `SERIAL_OUTPUT_TYPE`, it prints data in text format, sends telemetry
 *   data, or outputs data in JSON format.
 * - Skips the first datalogging event during startup to avoid incomplete data.
 *
 * @ingroup GeneralProcessing
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

  if constexpr (SERIAL_OUTPUT_TYPE == SerialOutputType::HumanReadable)
  {
    printForSerialText();
  }
  else if constexpr (SERIAL_OUTPUT_TYPE == SerialOutputType::IoT)
  {
    sendTelemetryData();
  }
  else if constexpr (SERIAL_OUTPUT_TYPE == SerialOutputType::JSON)
  {
    printForJSON(bOffPeak);
  }
}

/**
 * @brief Get the available RAM during setup.
 *
 * This function calculates the amount of free RAM available in the system.
 * It is useful for debugging and ensuring that the system has sufficient memory
 * for proper operation.
 *
 * @return int The amount of free RAM in bytes.
 *
 * @ingroup Debugging
 */
inline int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

#endif  // UTILS_H
