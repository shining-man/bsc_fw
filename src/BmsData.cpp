// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BmsData.h"
#include "WebSettings.h"

static const char * TAG = "BMSDATA";

static SemaphoreHandle_t mBmsDataMutex = NULL;

static struct bmsData_s bmsData;
struct bmsFilterData_s bmsFilterData;

static uint8_t bmsSettingsReadback[BT_DEVICES_COUNT][32];

static bool bo_SOC100CellvolHasBeenReached[BMSDATA_NUMBER_ALLDEVICES];

uint8_t u8_mBmsFilterErrorCounter[BMSDATA_NUMBER_ALLDEVICES];

uint8_t u8_readWriteDataSerialBms[SERIAL_BMS_DEVICES_COUNT];



void bmsDataInit()
{
  mBmsDataMutex = xSemaphoreCreateMutex();

  for(uint8_t i=0;i<BMSDATA_NUMBER_ALLDEVICES;i++)
  {
    for(uint8_t n=0;n<24;n++)
    {
      setBmsCellVoltage(i,n,0xFFFF);
    }
    setBmsLastDataMillis(i,0);
    u8_mBmsFilterErrorCounter[i]=0;
  }
  
  bmsFilterData.u8_mFilterBmsCellVoltagePercent=0;
  //bmsFilterData.u8_mFilterBmsCellVoltageMaxCount=0;
}


struct bmsFilterData_s* getBmsFilterData()
{
  return &bmsFilterData;
}

uint8_t* getBmsFilterErrorCounter(uint8_t bmsNr)
{
  return &u8_mBmsFilterErrorCounter[bmsNr];
}


void bmsDataSemaphoreTake()
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
}
void bmsDataSemaphoreGive()
{
  xSemaphoreGive(mBmsDataMutex);
}


struct bmsData_s* getBmsData()
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
bool setBmsCellVoltage(uint8_t devNr, uint8_t cellNr, uint16_t value)
{
  bool ret = true;
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);

  if(bmsFilterData.u8_mFilterBmsCellVoltagePercent>0 && bmsData.bmsCellVoltage[devNr][cellNr]!=0xFFFF) //Wenn größer 0, dann ist der Filter aktiv
  {
    if(value<(bmsData.bmsCellVoltage[devNr][cellNr]+(float)(bmsData.bmsCellVoltage[devNr][cellNr]/100.0*bmsFilterData.u8_mFilterBmsCellVoltagePercent)))
    {
      bmsData.bmsCellVoltage[devNr][cellNr] = value;
    }
    else
    {
      //Wert zu groß
      u8_mBmsFilterErrorCounter[devNr]|=(1<<7); //Fehler-Bit setzen (bit 7)
    }
  }
  else
  {
    bmsData.bmsCellVoltage[devNr][cellNr] = value;
  }

  xSemaphoreGive(mBmsDataMutex);
  return ret;
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
  float ret = (float)(bmsData.bmsTotalVoltage[devNr]/100.0);
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTotalVoltage(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsTotalVoltage[devNr] = (int16_t)(value*100);
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
  float ret = (float)(bmsData.bmsTotalCurrent[devNr]/100.0);
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTotalCurrent(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsTotalCurrent[devNr] = (int16_t)(value*100);
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
  float ret = (float)(bmsData.bmsBalancingCurrent[devNr]/100.0);
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsBalancingCurrent(uint8_t devNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsBalancingCurrent[devNr] = (int16_t)(value*100);
  xSemaphoreGive(mBmsDataMutex);
}


float getBmsTempature(uint8_t devNr, uint8_t sensorNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  float ret = (float)(bmsData.bmsTempature[devNr][sensorNr]/100.0);
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsTempature(uint8_t devNr, uint8_t sensorNr, float value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsTempature[devNr][sensorNr] = (int16_t)(value*100);
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
  if(devNr>=BT_DEVICES_COUNT)
  {
    if(value<100) bo_SOC100CellvolHasBeenReached[devNr]=false;
    
    uint16_t u16_CellvoltSoc100 = WebSettings::getInt(ID_PARAM_BMS_BALUE_ADJUSTMENTS_SOC100_CELL_VOLTAGE,devNr-BT_DEVICES_COUNT,DT_ID_PARAM_BMS_BALUE_ADJUSTMENTS_SOC100_CELL_VOLTAGE);
    if(u16_CellvoltSoc100>0 && ( bmsData.bmsMaxCellVoltage[devNr]>=u16_CellvoltSoc100  or bo_SOC100CellvolHasBeenReached[devNr]) )
    {
      bo_SOC100CellvolHasBeenReached[devNr]=true;
      value=100;
    }
    else
    {
      if(value==100) value=99;
    }
  }

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


uint8_t getBmsStateFETs(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  uint8_t ret = bmsData.bmsStateFETs[devNr];
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsStateFETs(uint8_t devNr, uint8_t value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  bmsData.bmsStateFETs[devNr] = value;
  xSemaphoreGive(mBmsDataMutex);
}

boolean getBmsStateFETsCharge(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  boolean ret = ((bmsData.bmsStateFETs[devNr]&0x01)==0x01) ? true : false;
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsStateFETsCharge(uint8_t devNr, boolean value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  if(value) bmsData.bmsStateFETs[devNr] |= 0x01;
  else bmsData.bmsStateFETs[devNr] &= ~(0x01);
  xSemaphoreGive(mBmsDataMutex);
}

boolean getBmsStateFETsDischarge(uint8_t devNr)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  boolean ret = ((bmsData.bmsStateFETs[devNr]&0x02)==0x02) ? true : false;
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setBmsStateFETsDischarge(uint8_t devNr, boolean value)
{
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  if(value) bmsData.bmsStateFETs[devNr] |= 0x02;
  else bmsData.bmsStateFETs[devNr] &= ~(0x02);
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


bool getSerialBmsWriteData(uint8_t devNr)
{
  bool ret=false;
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  if(u8_readWriteDataSerialBms[devNr]==1)ret=true;
  xSemaphoreGive(mBmsDataMutex);
  return ret;
}
void setSerialBmsWriteData(uint8_t devNr, bool value)
{
  uint8_t val=0;
  if(value)val=1;
  xSemaphoreTake(mBmsDataMutex, portMAX_DELAY);
  u8_readWriteDataSerialBms[devNr] = val;
  xSemaphoreGive(mBmsDataMutex);
}




uint8_t getBmsDataBytes(uint8_t dataType)
{
  switch(dataType)
  {
    case BMS_CELL_VOLTAGE:
      return 48;
      break;
    case BMS_TOTAL_VOLTAGE:
      return 4;
      break;
    case BMS_MAX_CELL_DIFFERENCE_VOLTAGE:
      return 2;
      break;
    case BMS_AVG_VOLTAGE:
      return 2;
      break;
    case BMS_TOTAL_CURRENT:
     return 2;
      break;
    case BMS_MAX_CELL_VOLTAGE:
      return 2;
      break;
    case BMS_MIN_CELL_VOLTAGE:
      return 2;
      break;
    case BMS_MAX_VOLTAGE_CELL_NUMBER:
      return 1;
      break;
    case BMS_MIN_VOLTAGE_CELL_NUMBER:
      return 1;
      break;
    case BMS_IS_BALANCING_ACTIVE:
      return 1;
      break;
    case BMS_BALANCING_CURRENT:
      return 4;
      break;
    case BMS_TEMPERATURE:
      return 12;
      break;
    case BMS_CHARGE_PERCENT:
      return 1;
      break;
    case BMS_ERRORS:
      return 4;
      break;
    case BMS_LAST_DATA_MILLIS:
      return 4;
      break;
    default:
      return 0;
      break;
  }
}


uint8_t * getBmsSettingsReadback(uint8_t bmsNr)
{
  return &bmsSettingsReadback[bmsNr][0];
}


#ifdef LOG_BMS_DATA
void logBmsData(uint8_t bmsNr)
{
  uint8_t i=0;
  BSC_LOGI(TAG,"----------------------");
  BSC_LOGI(TAG,"BMS: %i",bmsNr);
  for(i=0;i<24;i++) BSC_LOGI(TAG,"Cellvoltage #%i: %i",i,getBmsCellVoltage(bmsNr,i)); //Cellvoltage
  BSC_LOGI(TAG,"Totalvoltage: %f",getBmsTotalVoltage(bmsNr));
  BSC_LOGI(TAG,"Totalcurrent: %f",getBmsTotalCurrent(bmsNr));
  BSC_LOGI(TAG,"Max. cellvotage: %i",getBmsMaxCellVoltage(bmsNr));
  BSC_LOGI(TAG,"Max. voltage Cellnr: %i",getBmsMaxVoltageCellNumber(bmsNr));
  BSC_LOGI(TAG,"Min. cellvoltage: %i",getBmsMinCellVoltage(bmsNr));
  BSC_LOGI(TAG,"Min. voltage Cellnr: %i",getBmsMinVoltageCellNumber(bmsNr));
  BSC_LOGI(TAG,"AvgVoltage: %i",getBmsAvgVoltage(bmsNr));
  BSC_LOGI(TAG,"MaxCellDif. voltage: %i",getBmsMaxCellDifferenceVoltage(bmsNr));
  
  BSC_LOGI(TAG,"Charge percentage: %i",getBmsChargePercentage(bmsNr));
  BSC_LOGI(TAG,"Is balancing active: %i",getBmsIsBalancingActive(bmsNr));
  BSC_LOGI(TAG,"Balancing current: %f",getBmsBalancingCurrent(bmsNr));
  BSC_LOGI(TAG,"State FETs: %i",getBmsStateFETs(bmsNr));
  BSC_LOGI(TAG,"Errors: %i",getBmsErrors(bmsNr));
  
  for(i=0;i<3;i++) BSC_LOGI(TAG,"Temp. #%i: %f",i,getBmsTempature(bmsNr,i)); 
  BSC_LOGI(TAG,"LastDataMillis: %i",getBmsLastDataMillis(bmsNr));
}
#endif
