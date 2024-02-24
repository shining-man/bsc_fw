// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>
#include <typeinfo>
#include <utils/TypeConversions.hpp>

namespace types
{
namespace test
{

class BitfieldTest :
  public ::testing::Test
{
  protected:
  BitfieldTest() {}
  virtual ~BitfieldTest() {}

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  //
  /**
   * @brief Code here will be called immediately after the constructor (right before each test).
   */
  virtual void SetUp() {}

  /**
   * @brief Code here will be called immediately after each test (right before the destructor).
   */
  virtual void TearDown() {}

  // Objects declared here can be used by all tests in the test case for Foo.

  template<typename T> char const* getTypeName() { return __PRETTY_FUNCTION__; }

  template<typename T>
  void verifyMethodIsBitIdxInRange(bool verifyInRange)
  {
    using DataType = T;
    const std::size_t valueToVerify = (verifyInRange) ? std::numeric_limits<DataType>::digits - 1 :
                                                        std::numeric_limits<DataType>::digits;

    ASSERT_EQ(true, utils::isBitIdxInRange<T>(0)); // Zero must be alway in range for idx
    ASSERT_EQ(verifyInRange, utils::isBitIdxInRange<T>(valueToVerify)) << "Failed index of "
                                                                                  << ((verifyInRange) ? "in range test: " : "out of range test: ")
                                                                                  << valueToVerify
                                                                                  << ", Type: " << getTypeName<DataType>();
  };

  template<typename T>
  void verifyMethodBitIdxToValue()
  {
    using DataType = T;
    const std::size_t numberOfBits = std::numeric_limits<DataType>::digits;

    for (std::size_t i = 0; i < numberOfBits; ++i)
    {
      const DataType compareValue = (static_cast<DataType>(1) << i);
      ASSERT_EQ(compareValue, utils::bitIdxToValue<DataType>(i)) << "Failed index: " << i << ", Type: " << getTypeName<DataType>();
    }
  };
};

TEST_F(BitfieldTest, isBitIdxInRange_VerifyInRangeValues)
{
  constexpr bool IN_RANGE_TEST {true};

  verifyMethodIsBitIdxInRange<uint8_t>(IN_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uint16_t>(IN_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uint32_t>(IN_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uint64_t>(IN_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uintmax_t>(IN_RANGE_TEST);
}

TEST_F(BitfieldTest, isBitIdxInRange_VerifyOutOfRangeValues)
{
  constexpr bool OUT_OF_RANGE_TEST {false};

  verifyMethodIsBitIdxInRange<uint8_t>(OUT_OF_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uint16_t>(OUT_OF_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uint32_t>(OUT_OF_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uint64_t>(OUT_OF_RANGE_TEST);
  verifyMethodIsBitIdxInRange<uintmax_t>(OUT_OF_RANGE_TEST);

}

TEST_F(BitfieldTest, bitIdxToValue_CheckConversion)
{
  verifyMethodBitIdxToValue<uint8_t>();
  verifyMethodBitIdxToValue<uint16_t>();
  verifyMethodBitIdxToValue<uint32_t>();
  verifyMethodBitIdxToValue<uint64_t>();
  verifyMethodBitIdxToValue<uintmax_t>();
}

#if GTEST_HAS_DEATH_TEST // Only available on native target
TEST_F(BitfieldTest, bitIdxToValue_AssertsOnIdxOutOfRange)
{
  using DataType = uint8_t;
  DataType value {5}; // Just a value != 0

  ASSERT_DEBUG_DEATH(value = utils::bitIdxToValue<DataType>(std::numeric_limits<DataType>::digits), "bitIdx is NOT in range");

   // If this is not a DEBUG build we can expect, that the ASSERT_DEBUG_DEATH above executed the statement without assert.
   // bitIdxToValue must return 0 in that case. Let´s just verify this again.
  if (0 == value)
    ASSERT_EQ(0, utils::bitIdxToValue<DataType>(std::numeric_limits<DataType>::digits));
}
#endif

} //namespace test

} //namespace types

// Note: This is just a workaround, to prevent duplicate code for test application startup.
//       If I have found a way to use multiple sources for unit tests within platformio, this include can be removed.
#include <common/main-test.cpp>
