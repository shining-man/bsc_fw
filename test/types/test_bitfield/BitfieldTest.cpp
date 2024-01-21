// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>
#include <types/Bitfields.hpp>
#include <jungles/bitfields.hpp>

namespace types
{
namespace test
{

enum class BitFields
{
  BIT_0,
  BIT_1,
	BIT_2,
	BIT_3,
	BIT_4,
	BIT_5,
	BIT_6,
	BIT_7,

};

using BitFieldType = jungles::Bitfields<uint8_t,
                                        jungles::Field<BitFields::BIT_0, 1>,
                                        jungles::Field<BitFields::BIT_1, 1>,
																				jungles::Field<BitFields::BIT_2, 1>,
																				jungles::Field<BitFields::BIT_3, 1>,
																				jungles::Field<BitFields::BIT_4, 1>,
																				jungles::Field<BitFields::BIT_5, 1>,
																				jungles::Field<BitFields::BIT_6, 1>,
																				jungles::Field<BitFields::BIT_7, 1>>;


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
};

TEST_F(BitfieldTest, DefaultConstructor_CheckInitializedValues)
{
	const BitFieldType field1(0b10);
	BitFieldType field2;

	field2.at<BitFields::BIT_0>() = field1.at<BitFields::BIT_1>();
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