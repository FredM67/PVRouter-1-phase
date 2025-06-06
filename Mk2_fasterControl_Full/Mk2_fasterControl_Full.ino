/**
 * @file Mk2_fasterControl_Full.ino
 * @author Frederic Metrich (frederic.metrich@live.fr)
 * @brief A photovoltaic energy diverter.
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2025
 * 
 * @mainpage A 1-phase photovoltaic router/diverter
 *
 * @section description Description
 * Mk2_fasterControl_Full.ino - Arduino program that maximizes the use of home photovoltaic production
 * by monitoring energy consumption and diverting power to one or more resistive charge(s) when needed.
 * In the absence of such a system, surplus energy flows away to the grid and is of no benefit to the PV-owner.
 *
 * @section history History
 *
 * __May 2025, updated Mk2_fasterControl_Full with these changes:__
 * - heavy refactoring to make the code more readable and maintainable.
 * - added even more comments to explain the code.
 * - added telemetry for use with Home Assistant (with additional ESPHome module) or similar.
 * - disable diverted energy counting when load is overridden.
 * - better Doxygen documentation.
 * - added support for new PCB-based hardware. Old hardware is still supported.
 * - added support for new OLED display. Old display is still supported.
 * - delayed start of diversion
 *
 * __November 2024, renamed as Mk2_fasterControl_Full with these changes:__
 * - heavy refactoring to make the code more readable and maintainable.
 * - added comments to explain the code.
 * - added assertions to ensure the code is compiled with the correct C++ version.
 * - added a link to the README.md file in the comments.
 * - refactoring of the ISR to enhance the performance.
 *
 * __July 2023: updated to Mk2_fasterControl_twoLoads_5 with these changes:__
 * - the ability to control two loads has been transferred from the latest version of my
 *   standard multiLoad sketch, Mk2_multiLoad_wired_7a.  The faster control algorithm  
 *   has been retained.  
 *      The previous 2-load "faster control" sketch (version 4) has been archived as its 
 *   behaviour was found to be problematic.
 *   
 * @author Fred Metrich
 * @copyright Copyright (c) 2025
 * 
 * __June 2021: updated to Mk2_fasterControl_3 with these changes:__
 * - to reflect the performance of recently manufactured YHDC SCT_013_000 CTs, 
 *   the value of the parameter lpf_gain has been reduced from 12 to 8.
 *
 * __March 2021: updated to Mk2_fasterControl_2 with these changes:__
 * - extra filtering added to offset the HPF effect of CT1.  This allows the energy state in
 *   10 ms time to be predicted with more confidence.  Specifically, it is no longer necessary 
 *   to include a 30% boost factor after each change of load state.
 *
 * __February 2020: updated to Mk2_fasterControl_1a with these changes:__
 * - removal of some redundant code in the logic for determining the next load state.
 *
 * __November 2019: updated to Mk2_fasterControl_1 with these changes:__
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
 * __February 2016: updated to Mk2_bothDisplays_4, with these changes:__
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
 * __January 2016: updated to Mk2_bothDisplays_3c:__
 *   The variables to store the ADC results are now declared as "volatile" to remove 
 *   any possibility of incorrect operation due to optimisation by the compiler.
 *
 * __January 2016: renamed as Mk2_bothDisplays_3b, with this change:__
 * - a minor change in the ISR to remove a timing uncertainty.
 *
 * __December 2014: renamed as Mk2_bothDisplays_3a, with some typographical errors fixed.__
 *
 * __December 2014: renamed as Mk2_bothDisplays_3, with these changes:__
 * - persistence check added for zero-crossing detection (polarityConfirmed)
 * - lowestNoOfSampleSetsPerMainsCycle added, to check for any disturbances
 *
 * __September 2014: renamed as Mk2_bothDisplays_2, with these changes:__
 * - cycleCount removed (was not actually used in this sketch, but could have overflowed);
 * - removal of unhelpful comments in the IO pin section;
 * - tidier initialisation of display logic in setup();
 * - addition of REQUIRED_EXPORT_IN_WATTS logic (useful as a built-in PV simulation facility);
 *
 * __Issue 1 was released as Mk2_bothDisplays_1 in March 2014.__
 *
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
 * @author Robin Emley
 *         www.Mk2PVrouter.co.uk
 *
 */

/*!
 *  @defgroup TimeCritical Time-critical functions
 *  @brief Functions used by the ISR.
 *  
 *  @details This group contains functions that are executed within the Interrupt Service Routine (ISR) 
 *           or are closely tied to time-sensitive operations. These functions are optimized for speed 
 *           and efficiency to ensure the system operates without delays or interruptions.
 *  
 *  @note These functions handle tasks such as ADC sampling, zero-crossing detection, and real-time data processing.
 */

/*!
 *  @defgroup RelayDiversion Relay diversion feature
 *  @brief Functions used for managing relay-based energy diversion.
 *  
 *  @details This group includes functions that control the diversion of surplus photovoltaic energy 
 *           to resistive loads using relays. It supports multiple loads, load prioritization, and 
 *           features like forced full-power operation and load rotation.
 *  
 *  @note These functions ensure efficient energy usage by dynamically adjusting relay states based on 
 *        surplus energy availability and user-defined priorities.
 */

/*!
 *  @defgroup TemperatureSensing Temperature sensing feature
 *  @brief Functions used for temperature monitoring and processing.
 *  
 *  @details This group contains functions that handle temperature sensing using DS18B20 sensors. 
 *           It includes reading temperature data, filtering invalid readings, and updating telemetry 
 *           with valid temperature values. The temperature data can also influence load diversion decisions.
 *  
 *  @note The system supports multiple sensors and stores temperature values as integers multiplied by 100 for precision.
 */

/*!
 *  @defgroup DualTariff Dual tariff feature
 *  @brief Functions used for managing dual tariff periods.
 *  
 *  @details This group includes functions that detect and handle changes between off-peak and on-peak tariff periods. 
 *           It adjusts load priorities and diversion strategies based on the current tariff state. 
 *           Features like forced load operation during off-peak periods are also supported.
 *  
 *  @note The dual tariff feature helps optimize energy usage and cost savings by prioritizing energy diversion 
 *        during off-peak periods.
 */

/*!
 *  @defgroup RF RF feature
 *  @brief Functions used for RF communication.
 *  
 *  @details This group contains functions that enable wireless communication using RF modules (e.g., RFM12B or RF69). 
 *           It supports telemetry transmission, remote control commands, and runtime configuration of RF settings. 
 *           The system can operate with or without an RF module.
 *  
 *  @note RF communication is optional and can be disabled at compile time if not required.
 */

/*!
 *  @defgroup Telemetry Telemetry feature
 *  @brief Functions used for data logging and external system communication.
 *  
 *  @details This group includes functions that manage telemetry data collection and transmission. 
 *           It supports integration with external systems like HomeAssistant, providing real-time data 
 *           on power, voltage, temperature, and system status. Telemetry also includes diagnostic information.
 *  
 *  @note The telemetry feature ensures that the system's performance and status can be monitored remotely.
 */

/**
 * @defgroup GeneralProcessing General Processing
 * @brief Functions and routines that handle general system processing tasks.
 *
 * This group includes functions that are not time-critical but are essential
 * for the overall operation of the system. These tasks include monitoring
 * system states, managing load priorities, and handling diversion logic.
 *
 * @details
 * - Functions in this group are typically called from the main loop or other
 *   non-interrupt contexts.
 * - They ensure the system operates correctly and adapts to changing conditions.
 */

/**
 * @defgroup Initialization Initialization
 * @brief Functions and classes responsible for system setup and configuration.
 *
 * This group includes functions and classes that handle the initialization of
 * hardware components, system parameters, and other configurations required
 * for the proper operation of the system.
 *
 * @details
 * - Ensures that all components are properly configured before the system starts.
 * - Includes setup routines for relays, pins, ADCs, and other peripherals.
 * - Provides default configurations and validations to ensure system stability.
 */

/*!
 *  @defgroup OLEDDisplay OLED Display Feature
 *  @brief Functions for managing the OLED display.
 *  
 *  @details
 *  This group handles the initialization and updates of the OLED display. It is used to show
 *  real-time system data such as power, temperature, and system status.
 *  
 *  ### Features:
 *  - Display real-time metrics (e.g., power, temperature).
 *  - Show system status and error messages.
 *  - Support for graphical and textual elements.
 *  
 *  @note The OLED display uses the U8g2 library for rendering.
 */

/*!
 *  @defgroup 7SegDisplay 7-Segment Display Feature
 *  @brief Functions for managing the 7-segment display.
 *  
 *  @details
 *  This group handles the initialization and updates of the 7-segment display. It is used to show
 *  numeric data such as power levels, energy consumption, or system status in a simple and clear format.
 *  
 *  ### Features:
 *  - Display numeric metrics (e.g., power, energy).
 *  - Support for multiple digits with persistence-based display logic.
 *  - Configurable for different hardware setups (e.g., pin-saving or standard configurations).
 *  
 *  @note The 7-segment display is an alternative to the OLED display for simpler visual feedback.
 */

static_assert(__cplusplus >= 201703L, "**** Please define 'gnu++17' in 'platform.txt' ! ****");
static_assert(__cplusplus >= 201703L, "See also : https://github.com/FredM67/PVRouter-3-phase/blob/main/Mk2_fasterControl_Full/README.md");

// The active code can be found in the other cpp/h files
