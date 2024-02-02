// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JKBMSTYPESTEST_H
#define JKBMSTYPESTEST_H

#include <gtest/gtest.h>
#include <devices/jkbms/JkBmsTypes.hpp>

/**
 * @file
 * This header provides common definitions for unit tests related to the JkBmsTypes.
 *
*/

namespace jkbms
{
namespace test
{
  static constexpr uint16_t JKBMS_WARN_MSG_LOW_CAP_ALARM              = {types::bf::bitIdxToValue<uint16_t>( 0)}; // Bit 0:  Low capacity alarm
  static constexpr uint16_t JKBMS_WARN_MSG_MOS_TUBE_OVERTEMP_ALARM    = {types::bf::bitIdxToValue<uint16_t>( 1)}; // Bit 1:  MOS tube over temperature alarm                 -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_CHG_OVERVOLTAGE_ALARM      = {types::bf::bitIdxToValue<uint16_t>( 2)}; // Bit 2:  Charge over voltage alarm                       -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_CELL_OVERVOLTAGE           = {types::bf::bitIdxToValue<uint16_t>( 3)}; // Bit 3:  cell over voltage                               -> x
  static constexpr uint16_t JKBMS_WARN_MSG_CELL_UNDERVOLTAGE          = {types::bf::bitIdxToValue<uint16_t>( 4)}; // Bit 4:  cell under voltage                              -> x
  static constexpr uint16_t JKBMS_WARN_MSG_CHG_OVERTEMP               = {types::bf::bitIdxToValue<uint16_t>( 5)}; // Bit 5:  Charge over temperature                         -> x
  static constexpr uint16_t JKBMS_WARN_MSG_DCHG_OVERCURRENT_ALARM     = {types::bf::bitIdxToValue<uint16_t>( 6)}; // Bit 6:  discharge over current alarm                    -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_DCHG_OVERCURRENT           = {types::bf::bitIdxToValue<uint16_t>( 7)}; // Bit 7:  discharge overcurent                            -> x
  static constexpr uint16_t JKBMS_WARN_MSG_BATTERY_BOX_OVERTEMP_ALARM = {types::bf::bitIdxToValue<uint16_t>( 8)}; // Bit 8:  over temperature alarm in the battery box       -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_BATTERY_LOW_TEMPERATURE    = {types::bf::bitIdxToValue<uint16_t>( 9)}; // Bit 9:  Battery low temperature                         -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_CHG_UNDER_TEMPERATURE      = {types::bf::bitIdxToValue<uint16_t>(10)}; // Bit 10: Charge under temperature                        -> x
  static constexpr uint16_t JKBMS_WARN_MSG_UNKNOWN_BIT_11             = {types::bf::bitIdxToValue<uint16_t>(11)}; // Bit 11:                                                 -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_PROTECTION_309A            = {types::bf::bitIdxToValue<uint16_t>(12)}; // Bit 12: 309_A protection                                -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_PROTECTION_309B            = {types::bf::bitIdxToValue<uint16_t>(13)}; // Bit 13: 309_B protection                                -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_RESERVED_BIT_14            = {types::bf::bitIdxToValue<uint16_t>(14)}; // Bit 14:                                                 -> ?
  static constexpr uint16_t JKBMS_WARN_MSG_RESERVED_BIT_15            = {types::bf::bitIdxToValue<uint16_t>(15)}; // Bit 15:                                                 -> ?
} // namespace test
} // namespace jkbms

#endif // JKBMSTYPESTEST_H