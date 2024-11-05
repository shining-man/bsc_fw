// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "extension/ExtBluetooth.h"

static const char *TAG = "EXT_BT";

ExtBluetooth::ExtBluetooth(uint8_t address) : ExtDeviceI2C(address) { }

ExtBluetooth::~ExtBluetooth() {}


void ExtBluetooth::initialize()
{
  uint8_t u8_lErr = isDeviceAvailable(deviceAddress);
  if (u8_lErr == 0)
  {
    setEnabled(true);
    BSC_LOGI(TAG,"BT Ext. found");
  }
  else
  {
    setEnabled(false);
    BSC_LOGI(TAG,"BT Ext. not found (%i)",u8_lErr);
  }
}


void ExtBluetooth::readBtData()
{
  uint8_t bytesReceived = 0;

  struct  bmsData_s *p_lBmsData;
  p_lBmsData = getBmsData();

  for(uint8_t bmsNr = 0; bmsNr < 5; bmsNr++)
  {
    for(uint8_t bmsDataTyp = (uint8_t)BMS_CELL_VOLTAGE; bmsDataTyp < (uint8_t)BMS_LAST_DATA_MILLIS + 1; bmsDataTyp++)
    {
      //vTaskDelay(pdMS_TO_TICKS(10));
      uint8_t data[] = {BSC_GET_BT_EXTENSION_BMSDATA, bmsDataTyp, bmsNr, 0};
      i2cWriteBytes(deviceAddress, data, sizeof(data) / sizeof(data[0]));

      I2cRxSemaphoreTake();
      bytesReceived = i2cRequest(deviceAddress, 50); //len: getBmsDataBytes(bmsDataTyp)+2
      if(bytesReceived > 0)
      {
        uint8_t u8_lBmsNrNew, u8_BmsDataTypRet;
        uint8_t rxBuf[bytesReceived];
        i2cReadBytes(rxBuf, bytesReceived);
        I2cRxSemaphoreGive();

        u8_BmsDataTypRet = rxBuf[1];

        u8_lBmsNrNew = bmsNr; // zum Test

        switch(u8_BmsDataTypRet)
        {
          case BMS_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsCellVoltage[u8_lBmsNrNew], &rxBuf[2], 48);
            break;
          case BMS_TOTAL_VOLTAGE:
            memcpy(&p_lBmsData->bmsTotalVoltage[u8_lBmsNrNew], &rxBuf[2], 4);
            break;
          case BMS_MAX_CELL_DIFFERENCE_VOLTAGE:
            memcpy(&p_lBmsData->bmsMaxCellDifferenceVoltage[u8_lBmsNrNew], &rxBuf[2], 2);
            break;
          case BMS_AVG_VOLTAGE:
            memcpy(&p_lBmsData->bmsAvgVoltage[u8_lBmsNrNew], &rxBuf[2], 2);
            break;
          case BMS_TOTAL_CURRENT:
            memcpy(&p_lBmsData->bmsTotalCurrent[u8_lBmsNrNew], &rxBuf[2], 2);
            break;
          case BMS_MAX_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsMaxCellVoltage[u8_lBmsNrNew], &rxBuf[2], 2);
            break;
          case BMS_MIN_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsMinCellVoltage[u8_lBmsNrNew], &rxBuf[2], 2);
            break;
          case BMS_MAX_VOLTAGE_CELL_NUMBER:
            memcpy(&p_lBmsData->bmsMaxVoltageCellNumber[u8_lBmsNrNew], &rxBuf[2], 1);
            break;
          case BMS_MIN_VOLTAGE_CELL_NUMBER:
            memcpy(&p_lBmsData->bmsMinVoltageCellNumber[u8_lBmsNrNew], &rxBuf[2], 1);
            break;
          case BMS_IS_BALANCING_ACTIVE:
            memcpy(&p_lBmsData->bmsIsBalancingActive[u8_lBmsNrNew], &rxBuf[2], 1);
            break;
          case BMS_BALANCING_CURRENT:
            memcpy(&p_lBmsData->bmsBalancingCurrent[u8_lBmsNrNew], &rxBuf[2], 4);
            break;
          case BMS_TEMPERATURE:
            memcpy(&p_lBmsData->bmsTempature[u8_lBmsNrNew], &rxBuf[2], 12);
            break;
          case BMS_CHARGE_PERCENT:
            memcpy(&p_lBmsData->bmsChargePercentage[u8_lBmsNrNew], &rxBuf[2], 1);
            break;
          case BMS_ERRORS:
            memcpy(&p_lBmsData->bmsErrors[u8_lBmsNrNew], &rxBuf[2], 4);
            break;
          case BMS_LAST_DATA_MILLIS:
            if(rxBuf[2] == true) p_lBmsData->bmsLastDataMillis[u8_lBmsNrNew] = millis();
            //if(rxBuf[2] == true) BSC_LOGI(TAG,"New data from %i", u8_lBmsNrNew);
            break;
          default:
            break;
        }
      }
      else I2cRxSemaphoreGive();
    }
  }
}