// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef TYPES_CONVERSIONS_H
#define TYPES_CONVERSIONS_H

#include <type_traits>

/**
 * @brief This module provides some helper functions to calculate the decimal representation of a single bit (2^n)
 * with range check depending on the used data type
 *
 *  NOTE: The template methods require C++17 and above!
*/

namespace utils
{

/**
 * @brief This method returns true, if \a bitIdx is a valid bit in index for the type given by the template parameter.
 * @tparam DATA_TYPE The data type to be used to verify if bitIdx is in range.
 * @param bitIdx The index to be check for validity.
 *
 * @note This method does actually only support unsigned values, which is verified by the second template parameter at compile time!
*/
template<typename DATA_TYPE, typename = std::enable_if_t<std::is_unsigned<DATA_TYPE>::value, DATA_TYPE>>
constexpr bool isBitIdxInRange(std::size_t bitIdx)
{
  return (bitIdx < std::numeric_limits<DATA_TYPE>::digits);
}

/**
 * @brief This method just returns the result of the expression 2^n, with n = bitIdx.
 *        In case the bitIdx is not in the range of the underlying data type:
 *         - calls the assert() method if asserts are enabled (normaly only enabled in debug build)
 *         - returns 0, if asserts are disabled.
 * @tparam DATA_TYPE The data type to be used to verify if bitIdx is in range.
 * @param bitIdx The index to be check for validity.
 *
 * @note This method does actually only support unsigned values, which is verified by the second template parameter at compile time!
 * @note If supported by the compiler, we can check the range at compile time if the method is constexpr evaluated using std::is_constant_evaluated().
*/
template<typename DATA_TYPE,  typename = std::enable_if_t<std::is_unsigned<DATA_TYPE>::value, DATA_TYPE>>
constexpr DATA_TYPE bitIdxToValue(std::size_t bitIdx)
{
  const bool isIdxInRange = isBitIdxInRange<DATA_TYPE>(bitIdx);
  assert(isIdxInRange && "bitIdx is NOT in range");
  return isIdxInRange ? (static_cast<DATA_TYPE>(1) << bitIdx) : static_cast<DATA_TYPE>(0);
}

} // namespace utils

#endif // TYPES_CONVERSIONS_H