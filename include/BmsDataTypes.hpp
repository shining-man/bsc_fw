// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef BMSDATATYPES_H
#define BMSDATATYPES_H

#include <cstddef> // std::size_t
#include <cstdint> // uint16_t, ...
#include <types/Bitfields.hpp>
#include <devices/jkbms/JkBmsTypes.hpp>

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
  CELL_OVP,        //!< single cell overvoltage protection status flag
  CELL_UVP,        //!< single cell undervoltage protection status flag
  BATTERY_OVP,     //!< whole pack overvoltage protection status flag
  BATTERY_UVP,     //!< whole pack undervoltage protection status flag
  CHG_OTP,         //!< charging over temperature protection status flag
  CHG_UTP,         //!< charging low temperature protection status flag
  DCHG_OTP,        //!< Discharge over temperature protection status flag
  DCHG_UTP,        //!< discharge low temperature protection status flag
  CHG_OCP,         //!< charging overcurrent protection status flag
  DCHG_OCP,        //!< Discharge overcurrent protection status flag
  SHORT_CIRCUIT,   //!< short circuit protection status flag
  AFE_ERROR,       //!< Front-end detection IC error status flag
  SOFT_LOCK,       //!< software lock MOS status flag
  RESERVED_BIT_13, //!< Reserved status flag
  RESERVED_BIT_14, //!< Reserved status flag
  RESERVED_BIT_15, //!< Reserved status flag
};

/** Type safe definition for the BmsErrorStatus bitfield */
using BmsErrorStatus  = types::bf::Bitfields<uint16_t,                                           // The underlying type
                                             types::bf::Field<BmsErrorBits::RESERVED_BIT_15, 1>, // Field ID and its size (1bit)
                                             types::bf::Field<BmsErrorBits::RESERVED_BIT_14, 1>,
                                             types::bf::Field<BmsErrorBits::RESERVED_BIT_13, 1>,
                                             types::bf::Field<BmsErrorBits::SOFT_LOCK,       1>,
                                             types::bf::Field<BmsErrorBits::AFE_ERROR,       1>,
                                             types::bf::Field<BmsErrorBits::SHORT_CIRCUIT,   1>,
                                             types::bf::Field<BmsErrorBits::DCHG_OCP,        1>,
                                             types::bf::Field<BmsErrorBits::CHG_OCP,         1>,
                                             types::bf::Field<BmsErrorBits::DCHG_UTP,        1>,
                                             types::bf::Field<BmsErrorBits::DCHG_OTP,        1>,
                                             types::bf::Field<BmsErrorBits::CHG_UTP,         1>,
                                             types::bf::Field<BmsErrorBits::CHG_OTP,         1>,
                                             types::bf::Field<BmsErrorBits::BATTERY_UVP,     1>,
                                             types::bf::Field<BmsErrorBits::BATTERY_OVP,     1>,
                                             types::bf::Field<BmsErrorBits::CELL_UVP,        1>,
                                             types::bf::Field<BmsErrorBits::CELL_OVP,        1>>;

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
constexpr uint16_t BMS_ERR_STATUS_CELL_OVP      {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CELL_OVP_BIT_IDX)};      //      1 - bit0 single cell overvoltage protection
constexpr uint16_t BMS_ERR_STATUS_CELL_UVP      {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CELL_UVP_BIT_IDX)};      //      2 - bit1 single cell undervoltage protection
constexpr uint16_t BMS_ERR_STATUS_BATTERY_OVP   {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_BATTERY_OVP_BIT_IDX)};   //      4 - bit2  whole pack overvoltage protection
constexpr uint16_t BMS_ERR_STATUS_BATTERY_UVP   {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_BATTERY_UVP_BIT_IDX)};   //      8 - bit3  Whole pack undervoltage protection
constexpr uint16_t BMS_ERR_STATUS_CHG_OTP       {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CHG_OTP_BIT_IDX)};       //     16 - bit4  charging over temperature protection
constexpr uint16_t BMS_ERR_STATUS_CHG_UTP       {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CHG_UTP_BIT_IDX)};       //     32 - bit5  charging low temperature protection
constexpr uint16_t BMS_ERR_STATUS_DSG_OTP       {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_DSG_OTP_BIT_IDX)};       //     64 - bit6  Discharge over temperature protection
constexpr uint16_t BMS_ERR_STATUS_DSG_UTP       {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_DSG_UTP_BIT_IDX)};       //    128 - bit7  discharge low temperature protection
constexpr uint16_t BMS_ERR_STATUS_CHG_OCP       {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_CHG_OCP_BIT_IDX)};       //    256 - bit8  charging overcurrent protection
constexpr uint16_t BMS_ERR_STATUS_DSG_OCP       {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_DSG_OCP_BIT_IDX)};       //    512 - bit9  Discharge overcurrent protection
constexpr uint16_t BMS_ERR_STATUS_SHORT_CIRCUIT {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_SHORT_CIRCUIT_BIT_IDX)}; //   1024 - bit10 short circuit protection
constexpr uint16_t BMS_ERR_STATUS_AFE_ERROR     {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_AFE_ERROR_BIT_IDX)};     //   2048 - bit11 Front-end detection IC error
constexpr uint16_t BMS_ERR_STATUS_SOFT_LOCK     {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_SOFT_LOCK_BIT_IDX)};     //   4096 - bit12 software lock MOS
constexpr uint16_t BMS_ERR_STATUS_RESERVED1     {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_RESERVED1_BIT_IDX)};     //   8192 - bit13 Reserved
constexpr uint16_t BMS_ERR_STATUS_RESERVED2     {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_RESERVED2_BIT_IDX)};     //  16384 - bit14 Reserved
constexpr uint16_t BMS_ERR_STATUS_RESERVED3     {types::bf::bitIdxToValue<uint16_t>(BMS_ERR_STATUS_RESERVED3_BIT_IDX)};     //  32768 - bit15 Reserved

/**
 * @brief Template method to convert a device specific error/warning message to the BmsErrorStatus.
 * */
template <typename DEVICE>
inline BmsErrorStatus bmsErrorFromMessage(const DEVICE& bmsMsg);

//
/**
 * @brief Specialization to convert the JkBmsWarnMsg into the BmsErrorStatus.
 * */
template <>
inline BmsErrorStatus bmsErrorFromMessage(const jkbms::JkBmsWarnMsg& bmsMsg)
{
  BmsErrorStatus bmsErrors {BMS_ERR_STATUS_OK};
                                                                                                               // Bit  0: Low capacity alarm
  bmsErrors.at<BmsErrorBits::CHG_OTP>()     |= bmsMsg.at<jkbms::BatWarnMsgBits::MOS_TUBE_OVERTEMP_ALARM>();    // Bit  1: MOS tube over temperature alarm                 -> ?
  bmsErrors.at<BmsErrorBits::BATTERY_OVP>() |= bmsMsg.at<jkbms::BatWarnMsgBits::CHG_OVERVOLTAGE_ALARM>();      // Bit  2: Charge over voltage alarm                       -> ?
  bmsErrors.at<BmsErrorBits::CELL_OVP>()    |= bmsMsg.at<jkbms::BatWarnMsgBits::CELL_OVERVOLTAGE>();           // Bit  3: cell over voltage                               -> x
  bmsErrors.at<BmsErrorBits::CELL_UVP>()    |= bmsMsg.at<jkbms::BatWarnMsgBits::CELL_UNDERVOLTAGE>();          // Bit  4: cell under voltage                              -> x
  bmsErrors.at<BmsErrorBits::CHG_OTP>()     |= bmsMsg.at<jkbms::BatWarnMsgBits::CHG_OVERTEMP>();               // Bit  5: Charge over temperature                         -> x
  bmsErrors.at<BmsErrorBits::DCHG_OCP>()    |= bmsMsg.at<jkbms::BatWarnMsgBits::DCHG_OVERCURRENT_ALARM>();     // Bit  6: discharge over current alarm                    -> ?
  bmsErrors.at<BmsErrorBits::DCHG_OCP>()    |= bmsMsg.at<jkbms::BatWarnMsgBits::DCHG_OVERCURRENT>();           // Bit  7: discharge overcurent                            -> x
  bmsErrors.at<BmsErrorBits::CHG_OTP>()     |= bmsMsg.at<jkbms::BatWarnMsgBits::BATTERY_BOX_OVERTEMP_ALARM>(); // Bit  8: over temperature alarm in the battery box       -> ?
  // TODO : Is it correct, that BATTERY_LOW_TEMPERATURE is mapped to CHG_OCP (was Bit 9:  Battery low temperature to BMS_ERR_STATUS_CHG_OCP)
  bmsErrors.at<BmsErrorBits::CHG_OCP>()     |= bmsMsg.at<jkbms::BatWarnMsgBits::BATTERY_LOW_TEMPERATURE>();    // Bit  9: Battery low temperature                         -> ?
  bmsErrors.at<BmsErrorBits::CHG_UTP>()     |= bmsMsg.at<jkbms::BatWarnMsgBits::CHG_UNDER_TEMPERATURE>();      // Bit 10: Charge under temperature                        -> x
  bmsErrors.at<BmsErrorBits::SOFT_LOCK>()   |= bmsMsg.at<jkbms::BatWarnMsgBits::UNKNOWN_BIT_11>();             // Bit 11:                                                 -> ?
  bmsErrors.at<BmsErrorBits::SOFT_LOCK>()   |= bmsMsg.at<jkbms::BatWarnMsgBits::PROTECTION_309A>();            // Bit 12: 309_A protection                                -> ?
  bmsErrors.at<BmsErrorBits::SOFT_LOCK>()   |= bmsMsg.at<jkbms::BatWarnMsgBits::PROTECTION_309B>();            // Bit 13: 309_B protection                                -> ?

  return bmsErrors;
}

#endif // BMSDATATYPES_H