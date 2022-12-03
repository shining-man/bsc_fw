// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BmsData.h"

static SemaphoreHandle_t mBmsDataMutex = NULL;

static struct bmsData_s bmsData;


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


struct bmsData_s * getBmsData()
{
  return &bmsData;
}

uint16_t getBmsCellVoltage(uint8_t devNr, uint8_t cellNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint16_t ret = bmsData.bmsCellVoltage[devNr][cellNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsCellVoltage(uint8_t devNr, uint8_t cellNr, uint16_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsCellVoltage[devNr][cellNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


/*float getBmsCellResistance(uint8_t devNr, uint8_t cellNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsData.bmsCellResistance[devNr][cellNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsCellResistance(uint8_t devNr, uint8_t cellNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsCellResistance[devNr][cellNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}*/


float getBmsTotalVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsData.bmsTotalVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTotalVoltage(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsTotalVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint16_t getBmsMaxCellDifferenceVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint16_t ret = bmsData.bmsMaxCellDifferenceVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMaxCellDifferenceVoltage(uint8_t devNr, uint16_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsMaxCellDifferenceVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint16_t getBmsAvgVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint16_t ret = bmsData.bmsAvgVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsAvgVoltage(uint8_t devNr, uint16_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsAvgVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsTotalCurrent(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsData.bmsTotalCurrent[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTotalCurrent(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsTotalCurrent[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint16_t getBmsMaxCellVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint16_t ret = bmsData.bmsMaxCellVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMaxCellVoltage(uint8_t devNr, uint16_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsMaxCellVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint16_t getBmsMinCellVoltage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint16_t ret = bmsData.bmsMinCellVoltage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMinCellVoltage(uint8_t devNr, uint16_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsMinCellVoltage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsMaxVoltageCellNumber(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsData.bmsMaxVoltageCellNumber[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMaxVoltageCellNumber(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsMaxVoltageCellNumber[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsMinVoltageCellNumber(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsData.bmsMinVoltageCellNumber[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsMinVoltageCellNumber(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsMinVoltageCellNumber[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsIsBalancingActive(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsData.bmsIsBalancingActive[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsIsBalancingActive(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsIsBalancingActive[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsBalancingCurrent(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsData.bmsBalancingCurrent[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsBalancingCurrent(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsBalancingCurrent[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsTempature(uint8_t devNr, uint8_t sensorNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = bmsData.bmsTempature[devNr][sensorNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTempature(uint8_t devNr, uint8_t sensorNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsTempature[devNr][sensorNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint8_t getBmsChargePercentage(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsData.bmsChargePercentage[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsChargePercentage(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsChargePercentage[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


uint32_t getBmsErrors(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint32_t ret = bmsData.bmsErrors[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsErrors(uint8_t devNr, uint32_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsErrors[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}


unsigned long getBmsLastDataMillis(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  unsigned long ret = bmsData.bmsLastDataMillis[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsLastDataMillis(uint8_t devNr, unsigned long value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsLastDataMillis[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}