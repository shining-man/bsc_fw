// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>
#include <typeinfo>
#include <types/Bitfields.hpp>

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

    enum class BitFields
  {
    BIT_0,
    BIT_1,
    BIT_2,
    BIT_3,
    BIT_4,
    BIT_5,
    BIT_6,
    BIT_7
  };

  using BitFieldType = types::bf::Bitfields<uint8_t,
                                          types::bf::Field<BitFields::BIT_7, 1>,
                                          types::bf::Field<BitFields::BIT_6, 1>,
                                          types::bf::Field<BitFields::BIT_5, 1>,
                                          types::bf::Field<BitFields::BIT_4, 1>,
                                          types::bf::Field<BitFields::BIT_3, 1>,
                                          types::bf::Field<BitFields::BIT_2, 1>,
                                          types::bf::Field<BitFields::BIT_1, 1>,
                                          types::bf::Field<BitFields::BIT_0, 1>>;

  // Helper struct to hold the type
  template <typename T>
  struct TypeHelper
  {
    using Type = T;
  };

  template<typename DataType> char const* getTypeName() { return __PRETTY_FUNCTION__; }

  template<class T>
  void verifyMethodBitIdxToValue()
  {
    using DataType = T::Type;
    const std::size_t numberOfBits = std::numeric_limits<DataType>::digits;

    for (std::size_t i = 0; i < numberOfBits; ++i)
    {
      const DataType compareValue = (static_cast<DataType>(1) << i);
      ASSERT_EQ(compareValue, types::bf::bitIdxToValue<DataType>(i)) << "Failed index: " << i << ", Type: " << getTypeName<DataType>();
    }
  };
};

// Just a simple test to verify initialization by ctor
TEST_F(BitfieldTest, BitField_CheckInitializedValues)
{
  {
    const BitFieldType field1(0xAA);
    ASSERT_EQ(0xAA, field1.serialize());
    ASSERT_EQ(0b0, field1.at<BitFields::BIT_0>());
    ASSERT_EQ(0b1, field1.at<BitFields::BIT_1>());
    ASSERT_EQ(0b0, field1.at<BitFields::BIT_2>());
    ASSERT_EQ(0b1, field1.at<BitFields::BIT_3>());
    ASSERT_EQ(0b0, field1.at<BitFields::BIT_4>());
    ASSERT_EQ(0b1, field1.at<BitFields::BIT_5>());
    ASSERT_EQ(0b0, field1.at<BitFields::BIT_6>());
    ASSERT_EQ(0b1, field1.at<BitFields::BIT_7>());
  }
}

// Verify, that the copy of one bitfield to another does work.
// There is actually a bug in the main repository, which does not allow this copy.
// Run this test to verify it is fixed in the version we use.
TEST_F(BitfieldTest, BitField_CheckBitFieldCopyFromConst)
{
  {
    const BitFieldType field1(0xAA);
    BitFieldType field2;
    ASSERT_EQ(0xAA, field1.serialize());
    ASSERT_EQ(0x0,  field2.serialize());

    field2.at<BitFields::BIT_3>() = field1.at<BitFields::BIT_3>();

    ASSERT_EQ(0xAA, field1.serialize());
    ASSERT_EQ(0x08, field2.serialize());
  }
}

TEST_F(BitfieldTest, bitIdxToValue_CheckConversion)
{
  verifyMethodBitIdxToValue<TypeHelper<uint8_t>>();
  verifyMethodBitIdxToValue<TypeHelper<uint16_t>>();
  verifyMethodBitIdxToValue<TypeHelper<uint32_t>>();
  verifyMethodBitIdxToValue<TypeHelper<uint64_t>>();
  verifyMethodBitIdxToValue<TypeHelper<uintmax_t>>();
}

TEST_F(BitfieldTest, bitIdxToValue_AssertsOnIdxOutOfRange)
{
  using DataType = uint8_t;
  DataType value {5}; // Just a value != 0

  ASSERT_DEBUG_DEATH(value = types::bf::bitIdxToValue<DataType>(std::numeric_limits<DataType>::digits), "bitIdx is NOT in range");

   // If this is not a DEBUG build we can expect, that the ASSERT_DEBUG_DEATH above executed the statement without assert.
   // bitIdxToValue must return 0 in that case. Let´s just verify this again.
  if (0 == value)
    ASSERT_EQ(0, types::bf::bitIdxToValue<DataType>(std::numeric_limits<DataType>::digits));

}

} //namespace test

} //namespace types

#if defined(ARDUINO)
#include <Arduino.h>

void setup()
{
    // should be the same value as for the `test_speed` option in "platformio.ini"
    // default value is test_speed=115200
    Serial.begin(115200);

    ::testing::InitGoogleTest();
}

void loop()
{
	// Run tests
	if (RUN_ALL_TESTS())
	;

	// sleep 1 sec
	delay(1000);
}

#else
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
	if (RUN_ALL_TESTS())
	;
	// Always return zero-code and allow PlatformIO to parse results
	return 0;
}
#endif