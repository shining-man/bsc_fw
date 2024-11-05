// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "extension/ExtDisplay.h"
#include "dio.h"

static const char *TAG = "EXT_DISPLAY";

ExtDisplay::ExtDisplay(uint8_t address) : ExtDeviceI2C(address) { }

ExtDisplay::~ExtDisplay() {}


void ExtDisplay::initialize()
{
  uint8_t u8_lErr = isDeviceAvailable(deviceAddress);
  if (u8_lErr == 0)
  {
    setEnabled(true);
    BSC_LOGI(TAG,"Display found");
  }
  else
  {
    setEnabled(true);
    BSC_LOGI(TAG,"Display not found (%i)",u8_lErr);
  }
}


void ExtDisplay::sendDataStr(Inverter &inverter, uint8_t data1, uint8_t data2, std::string data, uint8_t dataLen)
{
  i2cSendData(inverter, deviceAddress, data1, data2, 0, data, dataLen);
}

void ExtDisplay::sendData(Inverter &inverter)
{
  struct  bmsData_s *p_lBmsData;
  p_lBmsData = getBmsData();

  for(uint8_t i = 0; i < BT_DEVICES_COUNT - 2; i++) //ToDo: Erweitern auf 7 Devices. Dazu muss aber auch das Display angepasst werden
  {
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_CELL_VOLTAGE, i, &p_lBmsData->bmsCellVoltage[i], 48);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_TOTAL_VOLTAGE, i, &p_lBmsData->bmsTotalVoltage[i], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MAX_CELL_DIFFERENCE_VOLTAGE, i, &p_lBmsData->bmsMaxCellDifferenceVoltage[i], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_AVG_VOLTAGE, i, &p_lBmsData->bmsAvgVoltage[i], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_TOTAL_CURRENT, i, &p_lBmsData->bmsTotalCurrent[i], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MAX_CELL_VOLTAGE, i, &p_lBmsData->bmsMaxCellVoltage[i], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MIN_CELL_VOLTAGE, i, &p_lBmsData->bmsMinCellVoltage[i], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MAX_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMaxVoltageCellNumber[i], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MIN_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMinVoltageCellNumber[i], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_IS_BALANCING_ACTIVE, i, &p_lBmsData->bmsIsBalancingActive[i], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_BALANCING_CURRENT, i, &p_lBmsData->bmsBalancingCurrent[i], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_TEMPERATURE, i, &p_lBmsData->bmsTempature[i], 6);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_CHARGE_PERCENT, i, &p_lBmsData->bmsChargePercentage[i], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_ERRORS, i, &p_lBmsData->bmsErrors[i], 4);
  }

  uint i = 5;
  for(uint8_t n = BT_DEVICES_COUNT; n < (BT_DEVICES_COUNT + 3); n++)
  {
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_CELL_VOLTAGE, i, &p_lBmsData->bmsCellVoltage[n], 48);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_TOTAL_VOLTAGE, i, &p_lBmsData->bmsTotalVoltage[n], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MAX_CELL_DIFFERENCE_VOLTAGE, i, &p_lBmsData->bmsMaxCellDifferenceVoltage[n], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_AVG_VOLTAGE, i, &p_lBmsData->bmsAvgVoltage[n], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_TOTAL_CURRENT, i, &p_lBmsData->bmsTotalCurrent[n], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MAX_CELL_VOLTAGE, i, &p_lBmsData->bmsMaxCellVoltage[n], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MIN_CELL_VOLTAGE, i, &p_lBmsData->bmsMinCellVoltage[n], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MAX_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMaxVoltageCellNumber[n], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_MIN_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMinVoltageCellNumber[n], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_IS_BALANCING_ACTIVE, i, &p_lBmsData->bmsIsBalancingActive[n], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_BALANCING_CURRENT, i, &p_lBmsData->bmsBalancingCurrent[n], 2);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_TEMPERATURE, i, &p_lBmsData->bmsTempature[n], 6);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_CHARGE_PERCENT, i, &p_lBmsData->bmsChargePercentage[n], 1);
    i2cSendData(inverter, deviceAddress, BMS_DATA, BMS_ERRORS, i, &p_lBmsData->bmsErrors[n], 4);
    i++;
  }

  struct Inverter::inverterData_s *p_lInverterData;
  p_lInverterData = inverter.getInverterData();
  i2cSendData(inverter, deviceAddress, INVERTER_DATA, INVERTER_VOLTAGE, 0, &p_lInverterData->batteryVoltage, 2);
  i2cSendData(inverter, deviceAddress, INVERTER_DATA, INVERTER_CURRENT, 0, &p_lInverterData->batteryCurrent, 2);
  i2cSendData(inverter, deviceAddress, INVERTER_DATA, INVERTER_SOC, 0, &p_lInverterData->inverterSoc, 2);
  i2cSendData(inverter, deviceAddress, INVERTER_DATA, INVERTER_CHARGE_CURRENT, 0, p_lInverterData->inverterChargeCurrent/10);
  i2cSendData(inverter, deviceAddress, INVERTER_DATA, INVERTER_DISCHARG_CURRENT, 0, p_lInverterData->inverterDischargeCurrent/10);

  uint16_t u16_lBscAlarms = getAlarm();
  i2cSendData(inverter, deviceAddress, BSC_DATA, BSC_ALARMS, 0, &u16_lBscAlarms, 2);

  //Relaistate
  uint8_t ioData = getDoData();
  i2cSendData(inverter, deviceAddress, BSC_DATA, BSC_RELAIS, 0, &ioData, 1);

  //Display Timeout
  uint8_t dispTimeout = WebSettings::getInt(ID_PARAM_DISPLAY_TIMEOUT,0,DT_ID_PARAM_DISPLAY_TIMEOUT);
  i2cSendData(inverter, deviceAddress, BSC_DATA, BSC_DISPLAY_TIMEOUT, 0, &dispTimeout, 1);
}

