// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BmsData.h"
#include "defines.h"

SemaphoreHandle_t mBmsDataMutex = NULL;


//                                                                                 // NEEY 4A | JbdBms | JK-BMS | 
uint32_t   bmsCellVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][24];          //    x    |   x    |   x    |
float      bmsCellResistance[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][24];       //    x    |        |        |
float      bmsTotalVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];             //    x    |   x    |   x    |
uint32_t   bmsMaxCellDifferenceVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT]; //    x    |   x    |   x    |
uint32_t   bmsAvgVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];               //    x    |   x    |   x    |
float      bmsTotalCurrent[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];             //         |   x    |   x    |
uint32_t   bmsMaxCellVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];           //    x    |   x    |   x    |
uint8_t    bmsMaxVoltageCellNumber[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];     //    x    |        |        |
uint8_t    bmsMinVoltageCellNumber[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];     //    x    |        |        |
uint8_t    bmsIsBalancingActive[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];        //    x    |        |        |
float      bmsBalancingCurrent[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];         //    x    |        |        |
float      bmsTempature[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][3];             //    2    |   3    |   3    |
uint8_t    bmsChargePercentage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];         //         |   x    |   x    |
uint32_t   bmsErrors[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];                   //    *    |   x    |   x    |
unsigned long bmsLastDataMillis[SERIAL_BMS_DEVICES_COUNT];                         //    x    |   x    |        |
//                                                                                 // *=Teilweise

/*
Errors:
STATUS_OK                0
STATUS_CELL_OVP          1   //bit0 single cell overvoltage protection 
STATUS_CELL_UVP          2   //bit1 single cell undervoltage protection    
STATUS_PACK_OVP          4   //bit2 whole pack overvoltage protection 
STATUS_PACK_UVP          8   //bit3 Whole pack undervoltage protection     
STATUS_CHG_OTP          16   //bit4 charging over-temperature protection 
STATUS_CHG_UTP          32   //bit5 charging low temperature protection 
STATUS_DSG_OTP          64   //bit6 Discharge over temperature protection  
STATUS_DSG_UTP         128   //bit7 discharge low temperature protection   
STATUS_CHG_OCP         256   //bit8 charging overcurrent protection 
STATUS_DSG_OCP         512   //bit9 Discharge overcurrent protection       
STATUS_SHORT_CIRCUIT  1024   //bit10 short circuit protection              
STATUS_AFE_ERROR      2048   //bit11 Front-end detection IC error 
STATUS_SOFT_LOCK      4096   //bit12 software lock MOS 
*/


void bmsDataInit()
{
  mBmsDataMutex = xSemaphoreCreateMutex();
}

void bmsDataSemaphoreTake()
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
}
void bmsDataSemaphoreGive()
{
  xSemaphoreGive(mBmsDataMutex);
}



uint32_t getBmsCellVoltage(uint8_t devNr, uint8_t cellNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint32_t ret = bmsCellVoltage[devNr][cellNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsCellVoltage(uint8_t devNr, uint8_t cellNr, uint32_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsCellVoltage[devNr][cellNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsCellResistance(uint8_t devNr, uint8_t cellNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsCellResistance[devNr][cellNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsCellResistance(uint8_t devNr, uint8_t cellNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsCellResistance[devNr][cellNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsTotalVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsTotalVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTotalVoltage(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsTotalVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint32_t getBmsMaxCellDifferenceVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint32_t ret = bmsMaxCellDifferenceVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMaxCellDifferenceVoltage(uint8_t devNr, uint32_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsMaxCellDifferenceVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint32_t getBmsAvgVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint32_t ret = bmsAvgVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsAvgVoltage(uint8_t devNr, uint32_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsAvgVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsTotalCurrent(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsTotalCurrent[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTotalCurrent(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsTotalCurrent[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint32_t getBmsMaxCellVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint32_t ret = bmsMaxCellVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMaxCellVoltage(uint8_t devNr, uint32_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsMaxCellVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsMaxVoltageCellNumber(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsMaxVoltageCellNumber[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMaxVoltageCellNumber(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsMaxVoltageCellNumber[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsMinVoltageCellNumber(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsMinVoltageCellNumber[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMinVoltageCellNumber(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsMinVoltageCellNumber[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsIsBalancingActive(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsIsBalancingActive[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsIsBalancingActive(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsIsBalancingActive[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsBalancingCurrent(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsBalancingCurrent[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsBalancingCurrent(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsBalancingCurrent[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsTempature(uint8_t devNr, uint8_t sensorNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsTempature[devNr][sensorNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTempature(uint8_t devNr, uint8_t sensorNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsTempature[devNr][sensorNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsChargePercentage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsChargePercentage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsChargePercentage(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsChargePercentage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint32_t getBmsErrors(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint32_t ret = bmsErrors[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsErrors(uint8_t devNr, uint32_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsErrors[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


unsigned long getBmsLastDataMillis(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  unsigned long ret = bmsLastDataMillis[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsLastDataMillis(uint8_t devNr, unsigned long value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsLastDataMillis[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}