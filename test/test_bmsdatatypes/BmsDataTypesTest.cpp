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

TEST_F(BmsDataTypesTest, VerifyBitFields_CELL_OVP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_CELL_OVP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::CELL_OVP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_CELL_UVP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_CELL_UVP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::CELL_UVP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_BATTERY_OVP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_BATTERY_OVP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::BATTERY_OVP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_BATTERY_UVP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_BATTERY_UVP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::BATTERY_UVP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_CHG_OTP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_CHG_OTP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::CHG_OTP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_CHG_UTP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_CHG_UTP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::CHG_UTP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_DCHG_OTP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_DSG_OTP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::DCHG_OTP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_DCHG_UTP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_DSG_UTP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::DCHG_UTP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_CHG_OCP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_CHG_OCP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::CHG_OCP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_DCHG_OCP)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_DSG_OCP);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::DCHG_OCP>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_SHORT_CIRCUIT)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_SHORT_CIRCUIT);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::SHORT_CIRCUIT>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_AFE_ERROR)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_AFE_ERROR);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::AFE_ERROR>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_SOFT_LOCK)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_SOFT_LOCK);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::SOFT_LOCK>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_RESERVED_BIT_13)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_RESERVED1);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::RESERVED_BIT_13>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_RESERVED_BIT_14)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_RESERVED2);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::RESERVED_BIT_14>());
}

TEST_F(BmsDataTypesTest, VerifyBitFields_RESERVED_BIT_15)
{
  BmsErrorStatus bmsErrors(BMS_ERR_STATUS_RESERVED3);

  ASSERT_EQ(0b1, bmsErrors.at<BmsErrorBits::RESERVED_BIT_15>());
}

TEST_F(BmsDataTypesTest, VerifyMethod_bmsErrorFromMessage_JkBmsWarnMsg)
{
  { // LOW_CAP_ALARM is actually not set by bmsErrorFromMessage, let's verify it
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_LOW_CAP_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(0, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_MOS_TUBE_OVERTEMP_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_OTP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_CHG_OVERVOLTAGE_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_BATTERY_OVP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_CELL_OVERVOLTAGE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CELL_OVP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_CELL_UNDERVOLTAGE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CELL_UVP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_CHG_OVERTEMP);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_OTP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_DCHG_OVERCURRENT_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_DSG_OCP, bmsErrors.serialize());
  }

  { // Note: same bit set in BmsErrorStatus as on overcurrent alarm!
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_DCHG_OVERCURRENT);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_DSG_OCP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_BATTERY_BOX_OVERTEMP_ALARM);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_OTP, bmsErrors.serialize());
  }

  { // TODO: Same question as in BmsDataTypes.hpp:
    //       Is it correct, that BATTERY_LOW_TEMPERATURE is mapped to CHG_OCP (was Bit 9:  Battery low temperature to BMS_ERR_STATUS_CHG_OCP)
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_BATTERY_LOW_TEMPERATURE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_OCP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_CHG_UNDER_TEMPERATURE);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_CHG_UTP, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_UNKNOWN_BIT_11);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_SOFT_LOCK, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_PROTECTION_309A);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_SOFT_LOCK, bmsErrors.serialize());
  }

  {
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_PROTECTION_309B);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(BMS_ERR_STATUS_SOFT_LOCK, bmsErrors.serialize());
  }

  { // Reserved bit 14 is actually not set by bmsErrorFromMessage, let's verify it
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_RESERVED_BIT_14);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(0, bmsErrors.serialize());
  }

  { // Reserved bit 15 is actually not set by bmsErrorFromMessage, let's verify it
    const jkbms::JkBmsWarnMsg jkBmsMsg(jkbms::test::JKBMS_WARN_MSG_RESERVED_BIT_15);
    BmsErrorStatus bmsErrors = bmsErrorFromMessage(jkBmsMsg);
    ASSERT_EQ(0, bmsErrors.serialize());
  }
}

// Note: This is just a workaround, to prevent duplicate code for test application startup.
//       If I have found a way to use multiple sources for unit tests within platformio, this include can be removed.
#include <common/main-test.cpp>
