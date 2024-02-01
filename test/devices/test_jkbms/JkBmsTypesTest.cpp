// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>
#include <devices/jkbms/JkBmsTypes.hpp>

namespace jkbms
{
namespace test
{

class JkBmsTypesTest :
	public ::testing::Test
{
	protected:
	JkBmsTypesTest() {}
	virtual ~JkBmsTypesTest() {}

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


	// Objects declared here can be used by all tests in the test case for Foo.
};

TEST_F(JkBmsTypesTest, VerifyLowCapacityAlarmFlag)
{
	JkBmsWarnMsg batWarnMsg2(0);
	const JkBmsWarnMsg batWarnMsg3(JKBMS_WARN_MSG_LOW_CAP_ALARM);
	batWarnMsg2.at<BatWarnMsgBits::LOW_CAP_ALARM>() = batWarnMsg3.at<BatWarnMsgBits::LOW_CAP_ALARM>();

	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_LOW_CAP_ALARM);

	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::LOW_CAP_ALARM>());
}

TEST_F(JkBmsTypesTest, VerifyMOSTubeOverTemperatureAlarmFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_MOS_TUBE_OVERTEMP_ALARM);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::MOS_TUBE_OVERTEMP_ALARM>());
}

TEST_F(JkBmsTypesTest, VerifyChargeOvervoltageAlarmFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_CHG_OVERVOLTAGE_ALARM);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::CHG_OVERVOLTAGE_ALARM>());
}

TEST_F(JkBmsTypesTest, VerifyCellOvervoltageFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_CELL_OVERVOLTAGE);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::CELL_OVERVOLTAGE>());
}

TEST_F(JkBmsTypesTest, VerifyCellUndervoltageFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_CELL_UNDERVOLTAGE);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::CELL_UNDERVOLTAGE>());
}

TEST_F(JkBmsTypesTest, VerifyChargeOvertempFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_CHG_OVERTEMP);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::CHG_OVERTEMP>());
}

TEST_F(JkBmsTypesTest, VerifyDischargeOvercurrentAlarmFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_DCHG_OVERCURRENT_ALARM);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::DCHG_OVERCURRENT_ALARM>());
}

TEST_F(JkBmsTypesTest, VerifyDischargeOvercurrentFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_DCHG_OVERCURRENT);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::DCHG_OVERCURRENT>());
}

TEST_F(JkBmsTypesTest, VerifyBatteryBoxOvertempAlarmFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_BATTERY_BOX_OVERTEMP_ALARM);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::BATTERY_BOX_OVERTEMP_ALARM>());
}

TEST_F(JkBmsTypesTest, VerifyBatteryBoxLowTemperatureFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_BATTERY_LOW_TEMPERATURE);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::BATTERY_LOW_TEMPERATURE>());
}

TEST_F(JkBmsTypesTest, VerifyChargeUnderTemperatureFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_CHG_UNDER_TEMPERATURE);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::CHG_UNDER_TEMPERATURE>());
}

TEST_F(JkBmsTypesTest, VerifyUnknownBit11Flag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_UNKNOWN_BIT_11);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::UNKNOWN_BIT_11>());
}

TEST_F(JkBmsTypesTest, VerifyProtection309AFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_PROTECTION_309A);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::PROTECTION_309A>());
}

TEST_F(JkBmsTypesTest, VerifyProtection309BFlag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_PROTECTION_309B);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::PROTECTION_309B>());
}

TEST_F(JkBmsTypesTest, VerifyReservedBit14Flag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_RESERVED_BIT_14);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::RESERVED_BIT_14>());
}

TEST_F(JkBmsTypesTest, VerifyReservedBit15Flag)
{
	JkBmsWarnMsg batWarnMsg(JKBMS_WARN_MSG_RESERVED_BIT_15);
	ASSERT_EQ(0b1, batWarnMsg.at<BatWarnMsgBits::RESERVED_BIT_15>());
}

} //namespace test
} //namespace types

// Note: This is just a workaround, to prevent duplicate code for test application startup.
//       If I have found a way to use multiple sources for unit tests within platformio, this include can be removed.
#include <common/main-test.cpp>
