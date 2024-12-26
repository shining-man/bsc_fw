// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef BMSDATATYPES_H
#define BMSDATATYPES_H

#include <cstddef> // std::size_t
#include <cstdint> // uint16_t, ...
#include <utils/TypeConversions.hpp>
#include <devices/jkbms/JkBmsTypes.hpp>
#include <type_safe/flag_set.hpp>

/**
 * @file
 * This header provides some type definitions for the BmsData module.
 * Actually this is mainly the bitfield definition of the BmsErrorStatus bits
 * and the conversion from the device status to the BmsErrorStatus
 *
*/

/** Bit definition for the BmsErrorStatus bitfields
 *  This enum class defines the bits used in the BmsErrorStatus bitfield
*/
enum class BmsErrorBits
{
  CELL_OVP,        //!< Bit  0: single cell overvoltage protection status flag
  CELL_UVP,        //!< Bit  1: single cell undervoltage protection status flag
  BATTERY_OVP,     //!< Bit  2: whole pack overvoltage protection status flag
  BATTERY_UVP,     //!< Bit  3: whole pack undervoltage protection status flag
  CHG_OTP,         //!< Bit  4: charging over temperature protection status flag
  CHG_UTP,         //!< Bit  5: charging low temperature protection status flag
  DCHG_OTP,        //!< Bit  6: Discharge over temperature protection status flag
  DCHG_UTP,        //!< Bit  7: discharge low temperature protection status flag
  CHG_OCP,         //!< Bit  8: charging overcurrent protection status flag
  DCHG_OCP,        //!< Bit  9: Discharge overcurrent protection status flag
  SHORT_CIRCUIT,   //!< Bit 10: short circuit protection status flag
  AFE_ERROR,       //!< Bit 11: Front-end detection IC error status flag
  SOFT_LOCK,       //!< Bit 12: software lock MOS status flag
  RESERVED_BIT_13, //!< Bit 13: Reserved status flag
  RESERVED_BIT_14, //!< Bit 14: Reserved status flag
  RESERVED_BIT_15, //!< Bit 15: Reserved status flag
  _flag_set_size,  //!< Number of bits - Required by type_safe::flag_set
};

/** Type safe definition for the BmsErrorStatus status flags */
using BmsErrorStatus = type_safe::flag_set<BmsErrorBits>;

// Note:
// Following constexpr variables are defined for legacy code.
// Shall be removed after conversion of the whole source code to the type safe bitfield above.

// Status bit index definitions
constexpr std::size_t BMS_ERR_STATUS_CELL_OVP_BIT_IDX      { 0}; // Bit index of single cell overvoltage protection status flag
constexpr std::size_t BMS_ERR_STATUS_CELL_UVP_BIT_IDX      { 1}; // Bit index of single cell undervoltage protection status flag
constexpr std::size_t BMS_ERR_STATUS_BATTERY_OVP_BIT_IDX   { 2}; // Bit index of whole pack overvoltage protection status flag
constexpr std::size_t BMS_ERR_STATUS_BATTERY_UVP_BIT_IDX   { 3}; // Bit index of whole pack undervoltage protection status flag
constexpr std::size_t BMS_ERR_STATUS_CHG_OTP_BIT_IDX       { 4}; // Bit index of charging over temperature protection status flag
constexpr std::size_t BMS_ERR_STATUS_CHG_UTP_BIT_IDX       { 5}; // Bit index of charging low temperature protection status flag
constexpr std::size_t BMS_ERR_STATUS_DSG_OTP_BIT_IDX       { 6}; // Bit index of Discharge over temperature protection status flag
constexpr std::size_t BMS_ERR_STATUS_DSG_UTP_BIT_IDX       { 7}; // Bit index of discharge low temperature protection status flag
constexpr std::size_t BMS_ERR_STATUS_CHG_OCP_BIT_IDX       { 8}; // Bit index of charging overcurrent protection status flag
constexpr std::size_t BMS_ERR_STATUS_DSG_OCP_BIT_IDX       { 9}; // Bit index of Discharge overcurrent protection status flag
constexpr std::size_t BMS_ERR_STATUS_SHORT_CIRCUIT_BIT_IDX {10}; // Bit index of short circuit protection status flag
constexpr std::size_t BMS_ERR_STATUS_AFE_ERROR_BIT_IDX     {11}; // Bit index of Front-end detection IC error status flag
constexpr std::size_t BMS_ERR_STATUS_SOFT_LOCK_BIT_IDX     {12}; // Bit index of software lock MOS status flag
constexpr std::size_t BMS_ERR_STATUS_RESERVED1_BIT_IDX     {13}; // Bit index of Reserved status flag
constexpr std::size_t BMS_ERR_STATUS_RESERVED2_BIT_IDX     {14}; // Bit index of Reserved status flag
constexpr std::size_t BMS_ERR_STATUS_RESERVED3_BIT_IDX     {15}; // Bit index of Reserved status flag

// Status bit value definitions
constexpr uint16_t BMS_ERR_STATUS_OK            {0};
constexpr uint16_t BMS_ERR_STATUS_CELL_OVP      {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CELL_OVP_BIT_IDX)};      //      1 - bit0 single cell overvoltage protection
constexpr uint16_t BMS_ERR_STATUS_CELL_UVP      {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CELL_UVP_BIT_IDX)};      //      2 - bit1 single cell undervoltage protection
constexpr uint16_t BMS_ERR_STATUS_BATTERY_OVP   {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_BATTERY_OVP_BIT_IDX)};   //      4 - bit2  whole pack overvoltage protection
constexpr uint16_t BMS_ERR_STATUS_BATTERY_UVP   {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_BATTERY_UVP_BIT_IDX)};   //      8 - bit3  Whole pack undervoltage protection
constexpr uint16_t BMS_ERR_STATUS_CHG_OTP       {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CHG_OTP_BIT_IDX)};       //     16 - bit4  charging over temperature protection
constexpr uint16_t BMS_ERR_STATUS_CHG_UTP       {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CHG_UTP_BIT_IDX)};       //     32 - bit5  charging low temperature protection
constexpr uint16_t BMS_ERR_STATUS_DSG_OTP       {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_DSG_OTP_BIT_IDX)};       //     64 - bit6  Discharge over temperature protection
constexpr uint16_t BMS_ERR_STATUS_DSG_UTP       {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_DSG_UTP_BIT_IDX)};       //    128 - bit7  discharge low temperature protection
constexpr uint16_t BMS_ERR_STATUS_CHG_OCP       {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CHG_OCP_BIT_IDX)};       //    256 - bit8  charging overcurrent protection
constexpr uint16_t BMS_ERR_STATUS_DSG_OCP       {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_DSG_OCP_BIT_IDX)};       //    512 - bit9  Discharge overcurrent protection
constexpr uint16_t BMS_ERR_STATUS_SHORT_CIRCUIT {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_SHORT_CIRCUIT_BIT_IDX)}; //   1024 - bit10 short circuit protection
constexpr uint16_t BMS_ERR_STATUS_AFE_ERROR     {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_AFE_ERROR_BIT_IDX)};     //   2048 - bit11 Front-end detection IC error
constexpr uint16_t BMS_ERR_STATUS_SOFT_LOCK     {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_SOFT_LOCK_BIT_IDX)};     //   4096 - bit12 software lock MOS
constexpr uint16_t BMS_ERR_STATUS_RESERVED1     {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_RESERVED1_BIT_IDX)};     //   8192 - bit13 Reserved
constexpr uint16_t BMS_ERR_STATUS_RESERVED2     {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_RESERVED2_BIT_IDX)};     //  16384 - bit14 Reserved
constexpr uint16_t BMS_ERR_STATUS_RESERVED3     {utils::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_RESERVED3_BIT_IDX)};     //  32768 - bit15 Reserved

/**
 * @brief This method convert the JkBmsWarnMsg into the BmsErrorStatus.
 * */
inline BmsErrorStatus bmsErrorFromMessage(const jkbms::JkBmsWarnMsg& bmsMsg)
{
  using namespace jkbms;
  BmsErrorStatus bmsStat = BmsErrorStatus::from_int(BMS_ERR_STATUS_OK);
                                                                                                          // Bit  0: Low capacity alarm
  if(bmsMsg.is_set(BatWarnMsgBits::MOS_TUBE_OVERTEMP_ALARM))    { bmsStat |= BmsErrorBits::CHG_OTP; }     // Bit  1: MOS tube over temperature alarm            -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::CHG_OVERVOLTAGE_ALARM))      { bmsStat |= BmsErrorBits::BATTERY_OVP; } // Bit  2: Charge over voltage alarm                  -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::CELL_OVERVOLTAGE))           { bmsStat |= BmsErrorBits::CELL_OVP; }    // Bit  3: cell over voltage                          -> x
  if(bmsMsg.is_set(BatWarnMsgBits::CELL_UNDERVOLTAGE))          { bmsStat |= BmsErrorBits::CELL_UVP; }    // Bit  4: cell under voltage                         -> x
  if(bmsMsg.is_set(BatWarnMsgBits::CHG_OVERCURRENT))            { bmsStat |= BmsErrorBits::CHG_OCP; }     // Bit  5: Over current                               -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::DCHG_OVERCURRENT_ALARM))     { bmsStat |= BmsErrorBits::DCHG_OCP; }    // Bit  6: discharge over current alarm               -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::DCHG_OVERCURRENT))           { bmsStat |= BmsErrorBits::DCHG_OCP; }    // Bit  7: discharge overcurent                       -> x
  if(bmsMsg.is_set(BatWarnMsgBits::BATTERY_BOX_OVERTEMP_ALARM)) { bmsStat |= BmsErrorBits::CHG_OTP; }     // Bit  8: over temperature alarm in the battery box  -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::BATTERY_LOW_TEMPERATURE))    { bmsStat |= BmsErrorBits::CHG_UTP; }     // Bit  9: Battery low temperature                    -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::CHG_UNDER_TEMPERATURE))      { bmsStat |= BmsErrorBits::CHG_UTP; }     // Bit 10: Charge under temperature                   -> x
  if(bmsMsg.is_set(BatWarnMsgBits::UNKNOWN_BIT_11))             { bmsStat |= BmsErrorBits::SOFT_LOCK; }   // Bit 11:                                            -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::PROTECTION_309A))            { bmsStat |= BmsErrorBits::SOFT_LOCK; }   // Bit 12: 309_A protection                           -> ?
  if(bmsMsg.is_set(BatWarnMsgBits::PROTECTION_309B))            { bmsStat |= BmsErrorBits::SOFT_LOCK; }   // Bit 13: 309_B protection                           -> ?

  return bmsStat;
}

#endif // BMSDATATYPES_H