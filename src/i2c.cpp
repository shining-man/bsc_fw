// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "i2c.h"
#include "Wire.h"
#include "defines.h"
#include "log.h"
#include "WebSettings.h"
#include "Canbus.h"
#include "Alarmrules.h"
#include "BmsData.h"

static const char *TAG = "I2C";

TwoWire I2C = TwoWire(0);

struct bmsData_s *p_lBmsData;
struct inverterData_s *p_lInverterData;
bool   bo_mDisplayEnabled;

void isDisplayConn();
void displaySendData_bms();  
void i2cSendData(uint8_t data1, uint8_t data2, uint8_t data3, const void *dataAdr, uint8_t dataLen);

void i2cInit()
{
  p_lBmsData = getBmsData();
  p_lInverterData = getInverterData();
  bool i2cBegin = I2C.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
  ESP_LOGI(TAG,"I2C begin=%d",i2cBegin);

  isDisplayConn();
}

void isDisplayConn()
{
    uint8_t u8_lErr;

    Wire.beginTransmission(I2C_DEV_ADDR_DISPLAY);
    u8_lErr = Wire.endTransmission();
    if (u8_lErr == 0)
    {
      bo_mDisplayEnabled = true;
      ESP_LOGI(TAG,"Display found");
    }
    else
    {
      bo_mDisplayEnabled = false;
      ESP_LOGI(TAG,"Display not found");
    }    
}

void i2cCyclicRun()
{
  if(bo_mDisplayEnabled)
  {
    displaySendData_bms();
  }
}

void i2cSendData(uint8_t data1, uint8_t data2, uint8_t data3, const void *dataAdr, uint8_t dataLen)
{
  if(!bo_mDisplayEnabled) return;

  uint8_t   txBuf[104];

  txBuf[0]=data1;
  txBuf[1]=data2;
  txBuf[2]=data3; //z.B. Nr. des BMS
  txBuf[3]=0x00; //Reserve (evtl. CRC8)

  if(data1==BMS_DATA){bmsDataSemaphoreTake();}
  else if(data1==INVERTER_DATA){inverterDataSemaphoreTake();}

  memcpy(&txBuf[TXBUFF_OFFSET], dataAdr, dataLen);

  if(data1==BMS_DATA){bmsDataSemaphoreGive();}
  else if(data1==INVERTER_DATA){inverterDataSemaphoreGive();}

  I2C.beginTransmission(I2C_DEV_ADDR_DISPLAY);
  uint8_t quant = I2C.write(txBuf,TXBUFF_OFFSET+dataLen);
  uint8_t error = I2C.endTransmission(true);

  //vTaskDelay(pdMS_TO_TICKS(10));
}

void i2cSendData(uint8_t data1, uint8_t data2, uint8_t data3, String data, uint8_t dataLen)
{
  i2cSendData(data1, data2, data3, data.c_str(), dataLen);
}


void displaySendData_bms()
{
  for(uint8_t i=0;i<BT_DEVICES_COUNT-2;i++) //ToDo: Erweitern auf 7 Devices. Dazu muss aber auch das Display angepasst werden
  {
    i2cSendData(BMS_DATA, BMS_CELL_VOLTAGE, i, &p_lBmsData->bmsCellVoltage[i], 48);
    i2cSendData(BMS_DATA, BMS_TOTAL_VOLTAGE, i, &p_lBmsData->bmsTotalVoltage[i], 4);
    i2cSendData(BMS_DATA, BMS_MAX_CELL_DIFFERENCE_VOLTAGE, i, &p_lBmsData->bmsMaxCellDifferenceVoltage[i], 2);
    i2cSendData(BMS_DATA, BMS_AVG_VOLTAGE, i, &p_lBmsData->bmsAvgVoltage[i], 2);
    i2cSendData(BMS_DATA, BMS_TOTAL_CURRENT, i, &p_lBmsData->bmsTotalCurrent[i], 4);
    i2cSendData(BMS_DATA, BMS_MAX_CELL_VOLTAGE, i, &p_lBmsData->bmsMaxCellVoltage[i], 2);
    i2cSendData(BMS_DATA, BMS_MIN_CELL_VOLTAGE, i, &p_lBmsData->bmsMinCellVoltage[i], 2);
    i2cSendData(BMS_DATA, BMS_MAX_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMaxVoltageCellNumber[i], 1);
    i2cSendData(BMS_DATA, BMS_MIN_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMinVoltageCellNumber[i], 1);
    i2cSendData(BMS_DATA, BMS_IS_BALANCING_ACTIVE, i, &p_lBmsData->bmsIsBalancingActive[i], 1);
    i2cSendData(BMS_DATA, BMS_BALANCING_CURRENT, i, &p_lBmsData->bmsBalancingCurrent[i], 4);
    i2cSendData(BMS_DATA, BMS_TEMPERATURE, i, &p_lBmsData->bmsTempature[i], 12);
    i2cSendData(BMS_DATA, BMS_CHARGE_PERCENT, i, &p_lBmsData->bmsChargePercentage[i], 1);
    i2cSendData(BMS_DATA, BMS_ERRORS, i, &p_lBmsData->bmsErrors[i], 4);
  }
  
  uint i=5;
  for(uint8_t n=BT_DEVICES_COUNT;n<(BT_DEVICES_COUNT+3);n++)
  {
    i2cSendData(BMS_DATA, BMS_CELL_VOLTAGE, i, &p_lBmsData->bmsCellVoltage[n], 48);
    i2cSendData(BMS_DATA, BMS_TOTAL_VOLTAGE, i, &p_lBmsData->bmsTotalVoltage[n], 4);
    i2cSendData(BMS_DATA, BMS_MAX_CELL_DIFFERENCE_VOLTAGE, i, &p_lBmsData->bmsMaxCellDifferenceVoltage[n], 2);
    i2cSendData(BMS_DATA, BMS_AVG_VOLTAGE, i, &p_lBmsData->bmsAvgVoltage[n], 2);
    i2cSendData(BMS_DATA, BMS_TOTAL_CURRENT, i, &p_lBmsData->bmsTotalCurrent[n], 4);
    i2cSendData(BMS_DATA, BMS_MAX_CELL_VOLTAGE, i, &p_lBmsData->bmsMaxCellVoltage[n], 2);
    i2cSendData(BMS_DATA, BMS_MIN_CELL_VOLTAGE, i, &p_lBmsData->bmsMinCellVoltage[n], 2);
    i2cSendData(BMS_DATA, BMS_MAX_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMaxVoltageCellNumber[n], 1);
    i2cSendData(BMS_DATA, BMS_MIN_VOLTAGE_CELL_NUMBER, i, &p_lBmsData->bmsMinVoltageCellNumber[n], 1);
    i2cSendData(BMS_DATA, BMS_IS_BALANCING_ACTIVE, i, &p_lBmsData->bmsIsBalancingActive[n], 1);
    i2cSendData(BMS_DATA, BMS_BALANCING_CURRENT, i, &p_lBmsData->bmsBalancingCurrent[n], 4);
    i2cSendData(BMS_DATA, BMS_TEMPERATURE, i, &p_lBmsData->bmsTempature[n], 12);
    i2cSendData(BMS_DATA, BMS_CHARGE_PERCENT, i, &p_lBmsData->bmsChargePercentage[n], 1);
    i2cSendData(BMS_DATA, BMS_ERRORS, i, &p_lBmsData->bmsErrors[n], 4);
    i++;
  }

  i2cSendData(INVERTER_DATA, INVERTER_VOLTAGE, 0, &p_lInverterData->inverterVoltage, 2);
  i2cSendData(INVERTER_DATA, INVERTER_CURRENT, 0, &p_lInverterData->inverterCurrent, 2);
  i2cSendData(INVERTER_DATA, INVERTER_SOC, 0, &p_lInverterData->inverterSoc, 2);
  i2cSendData(INVERTER_DATA, INVERTER_CHARGE_CURRENT, 0, &p_lInverterData->inverterChargeCurrent, 2);
  i2cSendData(INVERTER_DATA, INVERTER_DISCHARG_CURRENT, 0, &p_lInverterData->inverterDischargeCurrent, 2);

  uint16_t u16_lBscAlarms = getAlarm();
  i2cSendData(BSC_DATA, BSC_ALARMS, 0, &u16_lBscAlarms, 2);
}