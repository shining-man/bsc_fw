// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef TYPES_BITFIELDS_H
#define TYPES_BITFIELDS_H

#include <jungles/bitfields.hpp>

namespace types
{

namespace bf
{

using namespace jungles;

template<class DATA_TYPE> //, typename std::enable_if<std::is_unsigned<DATA_TYPE>::value>::type>
constexpr bool isBitIdxInRange(std::size_t bitIdx)
{
  return (bitIdx < std::numeric_limits<DATA_TYPE>::digits);
}

template<class DATA_TYPE> //, typename std::enable_if<std::is_unsigned<DATA_TYPE>::value>::type>
constexpr auto bitIdxToValue(std::size_t bitIdx)
{
  static_assert(std::is_unsigned_v<DATA_TYPE>, "signed values are not supported yet");

  if (isBitIdxInRange<DATA_TYPE>(bitIdx))
  {
      return (static_cast<DATA_TYPE>(1) << bitIdx);
  }
  else
  {
      assert(isBitIdxInRange<DATA_TYPE>(bitIdx) && "bitIdx is NOT in range");
      return 0;
  }
}

} // namespace bf
} // namespace types

#endif // TYPES_BITFIELDS_H