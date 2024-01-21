// Copyright (c) 2023 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JKBMSTYPES_H
#define JKBMSTYPES_H

#include <types/Bitfields.hpp>

namespace jkbms
{

enum class BatWarnMsgBits
{
  LOW_CAP_ALARM,              // Bit 0:  Low capacity alarm
  MOS_TUBE_OVERTEMP_ALARM,    // Bit 1:  MOS tube over temperature alarm                 -> ?
  CHG_OVERVOLTAGE_ALARM,      // Bit 2:  Charge over voltage alarm                       -> ?
  CELL_OVERVOLTAGE,           // Bit 3:  cell over voltage                               -> x
  CELL_UNDERVOLTAGE,          // Bit 4:  cell under voltage                              -> x
  CHG_OVERTEMP,               // Bit 5:  Charge over temperature                         -> x
  DCHG_OVERCURRENT_ALARM,     // Bit 6:  discharge over current alarm                    -> ?
  DCHG_OVERCURRENT,           // Bit 7:  discharge overcurent                            -> x
  BATTERY_BOX_OVERTEMP_ALARM, // Bit 8:  over temperature alarm in the battery box       -> ?
  BATTERY_LOW_TEMPERATURE,    // Bit 9:  Battery low temperature                         -> ?
  CHG_UNDER_TEMPERATURE,      // Bit 10: Charge under temperature                        -> x
  UNKNOWN_BIT_11,             // Bit 11:                                                 -> ?
  PROTECTION_309A,            // Bit 12: 309_A protection                                -> ?
  PROTECTION_309B,            // Bit 13: 309_B protection                                -> ?
  RESERVED_BIT_14,            // Bit 14:                                                 -> ?
  RESERVED_BIT_15             // Bit 15:                                                 -> ?
};

using JkBmsWarnMsg  = types::bf::Bitfields<uint16_t,                                                        // The underlying type
                                           types::bf::Field<BatWarnMsgBits::RESERVED_BIT_15,            1>, // Field ID (or name) and its size
                                           types::bf::Field<BatWarnMsgBits::RESERVED_BIT_14,            1>,
                                           types::bf::Field<BatWarnMsgBits::PROTECTION_309B,            1>,
                                           types::bf::Field<BatWarnMsgBits::PROTECTION_309A,            1>,
                                           types::bf::Field<BatWarnMsgBits::UNKNOWN_BIT_11,             1>,
                                           types::bf::Field<BatWarnMsgBits::CHG_UNDER_TEMPERATURE,      1>,
                                           types::bf::Field<BatWarnMsgBits::BATTERY_LOW_TEMPERATURE,    1>,
                                           types::bf::Field<BatWarnMsgBits::BATTERY_BOX_OVERTEMP_ALARM, 1>,
                                           types::bf::Field<BatWarnMsgBits::DCHG_OVERCURRENT,           1>,
                                           types::bf::Field<BatWarnMsgBits::DCHG_OVERCURRENT_ALARM,     1>,
                                           types::bf::Field<BatWarnMsgBits::CHG_OVERTEMP,               1>,
                                           types::bf::Field<BatWarnMsgBits::CELL_UNDERVOLTAGE,          1>,
                                           types::bf::Field<BatWarnMsgBits::CELL_OVERVOLTAGE,           1>,
                                           types::bf::Field<BatWarnMsgBits::CHG_OVERVOLTAGE_ALARM,      1>,
                                           types::bf::Field<BatWarnMsgBits::MOS_TUBE_OVERTEMP_ALARM,    1>,
                                           types::bf::Field<BatWarnMsgBits::LOW_CAP_ALARM,              1>>;

} // namespace jkbms

#endif // JKBMSTYPES_H