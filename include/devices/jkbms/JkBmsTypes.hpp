// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JKBMSTYPES_H
#define JKBMSTYPES_H

#include <type_safe/flag_set.hpp>

/**
 * @file
 * This header provides type definitions for the JkBms device.
 *
*/

namespace jkbms
{

/** Bit definition for the BatWarnMsgBits bitfields
 *  This enum class defines the bits used in the JkBmsWarnMsg bitfield
*/
enum class BatWarnMsgBits
{
  LOW_CAP_ALARM,              //!< Bit 0:  Low capacity alarm
  MOS_TUBE_OVERTEMP_ALARM,    //!< Bit 1:  MOS tube over temperature alarm                 -> ?
  CHG_OVERVOLTAGE_ALARM,      //!< Bit 2:  Charge over voltage alarm                       -> ?
  CELL_OVERVOLTAGE,           //!< Bit 3:  cell over voltage                               -> x
  CELL_UNDERVOLTAGE,          //!< Bit 4:  cell under voltage                              -> x
  CHG_OVERTEMP,               //!< Bit 5:  Charge over temperature                         -> x
  DCHG_OVERCURRENT_ALARM,     //!< Bit 6:  discharge over current alarm                    -> ?
  DCHG_OVERCURRENT,           //!< Bit 7:  discharge overcurent                            -> x
  BATTERY_BOX_OVERTEMP_ALARM, //!< Bit 8:  over temperature alarm in the battery box       -> ?
  BATTERY_LOW_TEMPERATURE,    //!< Bit 9:  Battery low temperature                         -> ?
  CHG_UNDER_TEMPERATURE,      //!< Bit 10: Charge under temperature                        -> x
  UNKNOWN_BIT_11,             //!< Bit 11:                                                 -> ?
  PROTECTION_309A,            //!< Bit 12: 309_A protection                                -> ?
  PROTECTION_309B,            //!< Bit 13: 309_B protection                                -> ?
  RESERVED_BIT_14,            //!< Bit 14:                                                 -> ?
  RESERVED_BIT_15,            //!< Bit 15:                                                 -> ?
  _flag_set_size,             //!< //!< Number of bits - Required by type_safe::flag_set
};

using JkBmsWarnMsg = type_safe::flag_set<BatWarnMsgBits>;

} // namespace jkbms

#endif // JKBMSTYPES_H