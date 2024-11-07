// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "extension/ExtBluetooth.h"
#include "WebSettings.h"
#include "Utility.h"

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

    sendBtDeviceMACs();
  }
  else
  {
    setEnabled(false);
    BSC_LOGI(TAG,"BT Ext. not found (%i)",u8_lErr);
  }
}


void ExtBluetooth::readBtData()
{
  uint8_t dataDeviceNr;
  uint8_t bytesReceived = 0;
  bool dataDevFound = false;

  struct  bmsData_s *p_lBmsData;
  p_lBmsData = getBmsData();

  for(uint8_t bmsNr = 0; bmsNr < BT_EXT_DEVICES_COUNT; bmsNr++)
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
        uint8_t rxBuf[bytesReceived];
        i2cReadBytes(rxBuf, bytesReceived);
        I2cRxSemaphoreGive();


        //dataDeviceNr = bmsNr; // zum Test
        // Suche Data-mapping Device
        dataDevFound = false;
        for(uint8_t i = 0; i < MUBER_OF_DATA_DEVICES; i++)
        {
          uint8_t dataDeviceSchnittstelle = (uint8_t)WebSettings::getInt(ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE,i,DT_ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE);
          if(dataDeviceSchnittstelle == bmsNr) 
          {
            dataDevFound = true;
            dataDeviceNr = i;
          }
        }
        if(!dataDevFound) continue;

        switch(rxBuf[1])
        {
          case BMS_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsCellVoltage[dataDeviceNr], &rxBuf[2], 48);
            break;
          case BMS_TOTAL_VOLTAGE:
            memcpy(&p_lBmsData->bmsTotalVoltage[dataDeviceNr], &rxBuf[2], 4);
            break;
          case BMS_MAX_CELL_DIFFERENCE_VOLTAGE:
            memcpy(&p_lBmsData->bmsMaxCellDifferenceVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_AVG_VOLTAGE:
            memcpy(&p_lBmsData->bmsAvgVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_TOTAL_CURRENT:
            memcpy(&p_lBmsData->bmsTotalCurrent[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_MAX_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsMaxCellVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_MIN_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsMinCellVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_MAX_VOLTAGE_CELL_NUMBER:
            memcpy(&p_lBmsData->bmsMaxVoltageCellNumber[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_MIN_VOLTAGE_CELL_NUMBER:
            memcpy(&p_lBmsData->bmsMinVoltageCellNumber[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_IS_BALANCING_ACTIVE:
            memcpy(&p_lBmsData->bmsIsBalancingActive[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_BALANCING_CURRENT:
            memcpy(&p_lBmsData->bmsBalancingCurrent[dataDeviceNr], &rxBuf[2], 4);
            break;
          case BMS_TEMPERATURE:
            memcpy(&p_lBmsData->bmsTempature[dataDeviceNr], &rxBuf[2], 12);
            break;
          case BMS_CHARGE_PERCENT:
            memcpy(&p_lBmsData->bmsChargePercentage[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_ERRORS:
            memcpy(&p_lBmsData->bmsErrors[dataDeviceNr], &rxBuf[2], 4);
            break;
          case BMS_LAST_DATA_MILLIS:
            if(rxBuf[2] == true) p_lBmsData->bmsLastDataMillis[dataDeviceNr] = millis();
            break;
          default:
            break;
        }
      }
      else I2cRxSemaphoreGive();
    }
  }
}


void ExtBluetooth::sendBtDeviceMACs()
{
  if(!isEnabled()) return;

  struct  bmsData_s *p_lBmsData;
  p_lBmsData = getBmsData();

  for(uint8_t bmsNr = 0; bmsNr < BT_EXT_DEVICES_COUNT; bmsNr++)
  {
    uint8_t data[11];
    data[0] = BSC_SET_BT_EXTENSION_DATA;
    data[1] = BSC_BT_CONNECT_MACS;
    data[2] = bmsNr;
    data[3] = 0;

    data[4] = WebSettings::getInt(ID_PARAM_SS_BTDEV, bmsNr, DT_ID_PARAM_SS_BTDEV); //Device Typ

    //data[5-9] = MAC
    if(!parseMacAddress(WebSettings::getString(ID_PARAM_SS_BTDEVMAC,bmsNr).c_str(), &data[5]))
    {
      // Fehler beim Parsen der MAC
      BSC_LOGE(TAG, "Fehler beim Parsen der MAC");
    }

    i2cWriteBytes(deviceAddress, data, sizeof(data) / sizeof(data[0]));
  }
}



void ExtBluetooth::sendDataAfterParameterChange()
{
  sendBtDeviceMACs();
}




