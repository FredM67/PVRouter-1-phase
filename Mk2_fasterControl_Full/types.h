/**
 * @file types.h
 * @author Frédéric Metrich (frederic.metrich@live.fr)
 * @brief Some basics classes/types
 * @version 0.1
 * @date 2024-10-03
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _TYPES_H
#define _TYPES_H

#include <Arduino.h>

// -------------------------------
// definitions of enumerated types

/** Polarities */
enum class polarities : uint8_t
{
  NEGATIVE, /**< polarity is negative */
  POSITIVE  /**< polarity is positive */
};

/** Load state (for use if loads are active high (Rev 2 PCB)) */
enum class loadStates : uint8_t
{
  LOAD_OFF, /**< load is OFF */
  LOAD_ON   /**< load is ON */
};

#endif  // _TYPES_H
