// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>
#include <devices/test_jkbms/JkBmsTypesTest.hpp>
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

	// Objects declared here can be used by all tests in the test case for Foo.
};

TEST_F(JkBmsTypesTest, VerifySizeOf_JkBmsWarnMsg)
{
  ASSERT_EQ(sizeof(uint16_t), sizeof(JkBmsWarnMsg));
}

TEST_F(JkBmsTypesTest, VerifyLowCapacityAlarmFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_LOW_CAP_ALARM);

  ASSERT_EQ(BatWarnMsgBits::LOW_CAP_ALARM, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyMOSTubeOverTemperatureAlarmFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_MOS_TUBE_OVERTEMP_ALARM);
  ASSERT_EQ(BatWarnMsgBits::MOS_TUBE_OVERTEMP_ALARM, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyChargeOvervoltageAlarmFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_CHG_OVERVOLTAGE_ALARM);
  ASSERT_EQ(BatWarnMsgBits::CHG_OVERVOLTAGE_ALARM, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyCellOvervoltageFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_CELL_OVERVOLTAGE);
  ASSERT_EQ(BatWarnMsgBits::CELL_OVERVOLTAGE, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyCellUndervoltageFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_CELL_UNDERVOLTAGE);
  ASSERT_EQ(BatWarnMsgBits::CELL_UNDERVOLTAGE, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyChargeOvertempFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_CHG_OVERTEMP);
  ASSERT_EQ(BatWarnMsgBits::CHG_OVERTEMP, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyDischargeOvercurrentAlarmFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_DCHG_OVERCURRENT_ALARM);
  ASSERT_EQ(BatWarnMsgBits::DCHG_OVERCURRENT_ALARM, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyDischargeOvercurrentFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_DCHG_OVERCURRENT);
  ASSERT_EQ(BatWarnMsgBits::DCHG_OVERCURRENT, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyBatteryBoxOvertempAlarmFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_BATTERY_BOX_OVERTEMP_ALARM);
  ASSERT_EQ(BatWarnMsgBits::BATTERY_BOX_OVERTEMP_ALARM, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyBatteryBoxLowTemperatureFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_BATTERY_LOW_TEMPERATURE);
  ASSERT_EQ(BatWarnMsgBits::BATTERY_LOW_TEMPERATURE, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyChargeUnderTemperatureFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_CHG_UNDER_TEMPERATURE);
  ASSERT_EQ(BatWarnMsgBits::CHG_UNDER_TEMPERATURE, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyUnknownBit11Flag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_UNKNOWN_BIT_11);
  ASSERT_EQ(BatWarnMsgBits::UNKNOWN_BIT_11, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyProtection309AFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_PROTECTION_309A);
  ASSERT_EQ(BatWarnMsgBits::PROTECTION_309A, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyProtection309BFlag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_PROTECTION_309B);
  ASSERT_EQ(BatWarnMsgBits::PROTECTION_309B, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyReservedBit14Flag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_RESERVED_BIT_14);
  ASSERT_EQ(BatWarnMsgBits::RESERVED_BIT_14, batWarnMsg);
}

TEST_F(JkBmsTypesTest, VerifyReservedBit15Flag)
{
  JkBmsWarnMsg batWarnMsg = JkBmsWarnMsg::from_int(JKBMS_WARN_MSG_RESERVED_BIT_15);
  ASSERT_EQ(BatWarnMsgBits::RESERVED_BIT_15, batWarnMsg);
}

} //namespace test
} //namespace jkbms

// Note: This is just a workaround, to prevent duplicate code for test application startup.
//       If I have found a way to use multiple sources for unit tests within platformio, this include can be removed.
#include <common/main-test.cpp>
