// Copyright (c) 2024 Meik Jäckle
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <gtest/gtest.h>
#include <devices/test_jkbms/JkBmsTypesTest.hpp>
#include <BmsDataTypes.hpp>

class BmsDataTypesTest :
	public ::testing::Test
{
  protected:
  BmsDataTypesTest() {}
  virtual ~BmsDataTypesTest() {}

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

TEST_F(BmsDataTypesTest, VerifyConstValues_BMS_ERROR_STATUS)
{
  ASSERT_EQ(    0, BMS_ERR_STATUS_OK);
  ASSERT_EQ(    1, BMS_ERR_STATUS_CELL_OVP);
  ASSERT_EQ(    2, BMS_ERR_STATUS_CELL_UVP);
  ASSERT_EQ(    4, BMS_ERR_STATUS_BATTERY_OVP);
  ASSERT_EQ(    8, BMS_ERR_STATUS_BATTERY_UVP);
  ASSERT_EQ(   16, BMS_ERR_STATUS_CHG_OTP);
  ASSERT_EQ(   32, BMS_ERR_STATUS_CHG_UTP);
  ASSERT_EQ(   64, BMS_ERR_STATUS_DSG_OTP);
  ASSERT_EQ(  128, BMS_ERR_STATUS_DSG_UTP);
  ASSERT_EQ(  256, BMS_ERR_STATUS_CHG_OCP);
  ASSERT_EQ(  512, BMS_ERR_STATUS_DSG_OCP);
  ASSERT_EQ( 1024, BMS_ERR_STATUS_SHORT_CIRCUIT);
  ASSERT_EQ( 2048, BMS_ERR_STATUS_AFE_ERROR);
  ASSERT_EQ( 4096, BMS_ERR_STATUS_SOFT_LOCK);
  ASSERT_EQ( 8192, BMS_ERR_STATUS_RESERVED1);
  ASSERT_EQ(16384, BMS_ERR_STATUS_RESERVED2);
  ASSERT_EQ(32768, BMS_ERR_STATUS_RESERVED3);
}


TEST_F(BmsDataTypesTest, VerifySizeOf_BmsErrorStatus)
{
  ASSERT_EQ(sizeof(uint16_t), sizeof(BmsErrorStatus));
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_CELL_OVP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_CELL_OVP);
  ASSERT_EQ(BmsErrorBits::CELL_OVP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_CELL_OVP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_CELL_UVP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_CELL_UVP);
  ASSERT_EQ(BmsErrorBits::CELL_UVP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_CELL_UVP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_BATTERY_OVP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_BATTERY_OVP);
  ASSERT_EQ(BmsErrorBits::BATTERY_OVP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_BATTERY_OVP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_BATTERY_UVP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_BATTERY_UVP);
  ASSERT_EQ(BmsErrorBits::BATTERY_UVP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_BATTERY_UVP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_CHG_OTP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_CHG_OTP);
  ASSERT_EQ(BmsErrorBits::CHG_OTP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_CHG_OTP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_CHG_UTP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_CHG_UTP);
  ASSERT_EQ(BmsErrorBits::CHG_UTP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_CHG_UTP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_DCHG_OTP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_DSG_OTP);
  ASSERT_EQ(BmsErrorBits::DCHG_OTP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_DSG_OTP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_DCHG_UTP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_DSG_UTP);
  ASSERT_EQ(BmsErrorBits::DCHG_UTP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_DSG_UTP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_CHG_OCP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_CHG_OCP);
  ASSERT_EQ(BmsErrorBits::CHG_OCP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_CHG_OCP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_DCHG_OCP)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_DSG_OCP);
  ASSERT_EQ(BmsErrorBits::DCHG_OCP, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_DSG_OCP, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_SHORT_CIRCUIT)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_SHORT_CIRCUIT);
  ASSERT_EQ(BmsErrorBits::SHORT_CIRCUIT, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_SHORT_CIRCUIT, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_AFE_ERROR)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_AFE_ERROR);
  ASSERT_EQ(BmsErrorBits::AFE_ERROR, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_AFE_ERROR, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_SOFT_LOCK)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_SOFT_LOCK);
  ASSERT_EQ(BmsErrorBits::SOFT_LOCK, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_SOFT_LOCK, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_RESERVED_BIT_13)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_RESERVED1);
  ASSERT_EQ(BmsErrorBits::RESERVED_BIT_13, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_RESERVED1, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_RESERVED_BIT_14)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_RESERVED2);
  ASSERT_EQ(BmsErrorBits::RESERVED_BIT_14, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_RESERVED2, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyErrorFlag_RESERVED_BIT_15)
{
  BmsErrorStatus bmsErrors = BmsErrorStatus::from_int(BMS_ERR_STATUS_RESERVED3);
  ASSERT_EQ(BmsErrorBits::RESERVED_BIT_15, bmsErrors);
  ASSERT_EQ(BMS_ERR_STATUS_RESERVED3, bmsErrors.to_int<uint16_t>());
}

TEST_F(BmsDataTypesTest, VerifyMethod_bmsErrorFromMessage_JkBmsWarnMsg)
{
  { // LOW_CAP_ALARM is actually not set by bmsErrorFromMessage, let's verify it
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_LOW_CAP_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(0, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg  = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_MOS_TUBE_OVERTEMP_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_OTP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_CHG_OVERVOLTAGE_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_BATTERY_OVP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_CELL_OVERVOLTAGE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CELL_OVP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_CELL_UNDERVOLTAGE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CELL_UVP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_CHG_OVERCURRENT);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_OCP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_DCHG_OVERCURRENT_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_DSG_OCP, bmsErrors.to_int<uint16_t>());
  }

  { // Note: same bit set in BmsErrorStatus as on overcurrent alarm!
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_DCHG_OVERCURRENT);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_DSG_OCP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_BATTERY_BOX_OVERTEMP_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_OTP, bmsErrors.to_int<uint16_t>());
  }

  { 
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_BATTERY_LOW_TEMPERATURE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_UTP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_CHG_UNDER_TEMPERATURE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_UTP, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_UNKNOWN_BIT_11);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_SOFT_LOCK, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_PROTECTION_309A);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_SOFT_LOCK, bmsErrors.to_int<uint16_t>());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_PROTECTION_309B);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_SOFT_LOCK, bmsErrors.to_int<uint16_t>());
  }

  { // Reserved bit 14 is actually not set by bmsErrorFromMessage, let's verify it
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_RESERVED_BIT_14);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(0, bmsErrors.to_int<uint16_t>());
  }

  { // Reserved bit 15 is actually not set by bmsErrorFromMessage, let's verify it
    const jkbms::JkBmsWarnMsg jkBmsMsg = jkbms::JkBmsWarnMsg::from_int(jkbms::test::JKBMS_WARN_MSG_RESERVED_BIT_15);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(0, bmsErrors.to_int<uint16_t>());
  }
}

// Note: This is just a workaround, to prevent duplicate code for test application startup.
//       If I have found a way to use multiple sources for unit tests within platformio, this include can be removed.
#include <common/main-test.cpp>
