// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "inverter/BmsDataUtils.hpp"
#include "defines.h"
#include "BmsData.h"

BmsDataUtils::BmsDataUtils()
{
    // Konstruktor-Code
}

BmsDataUtils::~BmsDataUtils()
{
    // Dekonstruktor-Code
}

uint8_t BmsDataUtils::getNumberOfBatteryModules(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd)
{
  uint8_t u8_lModules=1;

  if(u16_mBmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u16_mBmsDatasourceAdd>>i)&0x01)
      {
        u8_lModules++;
      }
    }
  }
  return u8_lModules;
}

uint8_t BmsDataUtils::getNumberOfBatteryModulesCharge(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd)
{
  uint8_t u8_lModules=0;
  if(getBmsStateFETsCharge(u8_mBmsDatasource)) u8_lModules++;

  if(u16_mBmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u16_mBmsDatasourceAdd>>i)&0x01)
      {
        if(getBmsStateFETsCharge(BMSDATA_FIRST_DEV_SERIAL+i))
        {
          u8_lModules++;
        }
      }
    }
  }
  return u8_lModules;
}

uint8_t BmsDataUtils::getNumberOfBatteryModulesDischarge(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd)
{
  uint8_t u8_lModules=0;
  if(getBmsStateFETsDischarge(u8_mBmsDatasource)) u8_lModules++;

  if(u16_mBmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u16_mBmsDatasourceAdd>>i)&0x01)
      {
        if(getBmsStateFETsDischarge(BMSDATA_FIRST_DEV_SERIAL+i))
        {
          u8_lModules++;
        }
      }
    }
  }
  return u8_lModules;
}


//Maximale Zellspannung von allen aktiven BMSen ermitteln
uint16_t BmsDataUtils::getMaxCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd)
{
  uint8_t BmsNr, CellNr;
  return getMaxCellSpannungFromBms(u8_mBmsDatasource, u16_mBmsDatasourceAdd, BmsNr, CellNr);
}

uint16_t BmsDataUtils::getMaxCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd, uint8_t &BmsNr,uint8_t &CellNr)
{
  uint8_t u8_lBmsNr=0;
  uint8_t u8_lCellNr=0;


  uint16_t u16_lCellSpg = getBmsMaxCellVoltage(u8_mBmsDatasource);
  u8_lBmsNr=u8_mBmsDatasource;
  u8_lCellNr=getBmsMaxVoltageCellNumber(u8_mBmsDatasource);

  if(u16_mBmsDatasourceAdd>0)
  {
    uint16_t u16_lMaxCellSpg=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u16_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<CAN_BMS_COMMUNICATION_TIMEOUT) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          u16_lMaxCellSpg=getBmsMaxCellVoltage(BMSDATA_FIRST_DEV_SERIAL+i);
          if(u16_lMaxCellSpg>u16_lCellSpg)
          {
            u16_lCellSpg=u16_lMaxCellSpg;
            u8_lBmsNr=BMSDATA_FIRST_DEV_SERIAL+i;
            u8_lCellNr=getBmsMaxVoltageCellNumber(BMSDATA_FIRST_DEV_SERIAL+i);
          }
        }
      }
    }
    #ifdef CAN_DEBUG
    BSC_LOGD(TAG,"getMaxCellSpannung: MaxSpg=%i, bms=%i, cell=%i",u16_lCellSpg, u8_lBmsNr, u8_lCellNr); //nur zum Debug
    #endif
  }
  #ifdef CAN_DEBUG
  else
  {
    BSC_LOGD(TAG,"getMaxCellSpannung: MaxSpg=%i",u16_lCellSpg); //nur zum Debug
  }
  #endif

  BmsNr = u8_lBmsNr;
  CellNr = u8_lCellNr;
  return u16_lCellSpg;
}


//Minimale Zellspannung von allen aktiven BMSen ermitteln
uint16_t BmsDataUtils::getMinCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd)
{
  uint8_t BmsNr, CellNr;
  return getMinCellSpannungFromBms(u8_mBmsDatasource, u16_mBmsDatasourceAdd, BmsNr, CellNr);
}

uint16_t BmsDataUtils::getMinCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd, uint8_t &BmsNr,uint8_t &CellNr)
{
  uint8_t u8_lBmsNr=0;
  uint8_t u8_lCellNr=0;
  uint16_t u16_lCellSpg=0xFFFF;

  if((millis()-getBmsLastDataMillis(u8_mBmsDatasource))<CAN_BMS_COMMUNICATION_TIMEOUT)
  {
    u16_lCellSpg = getBmsMinCellVoltage(u8_mBmsDatasource);
    u8_lBmsNr=u8_mBmsDatasource;
    u8_lCellNr=getBmsMinVoltageCellNumber(u8_mBmsDatasource);
  }

  if(u16_mBmsDatasourceAdd>0)
  {
    uint16_t u16_lMinCellSpg=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u16_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<CAN_BMS_COMMUNICATION_TIMEOUT) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          u16_lMinCellSpg=getBmsMinCellVoltage(BMSDATA_FIRST_DEV_SERIAL+i);
          if(u16_lMinCellSpg<u16_lCellSpg)
          {
            u16_lCellSpg=u16_lMinCellSpg;
            u8_lBmsNr=BMSDATA_FIRST_DEV_SERIAL+i;
            u8_lCellNr=getBmsMinVoltageCellNumber(BMSDATA_FIRST_DEV_SERIAL+i);
          }
        }
      }
    }
    #ifdef CAN_DEBUG
    BSC_LOGD(TAG,"getMinCellSpannung: MinSpg=%i, bms=%i, cell=%i",u16_lCellSpg, u8_lBmsNr, u8_lCellNr); //nur zum Debug
    #endif
  }
  #ifdef CAN_DEBUG
  else
  {
    BSC_LOGD(TAG,"getMinCellSpannung: MinSpg=%i",u16_lCellSpg); //nur zum Debug
  }
  #endif

  BmsNr = u8_lBmsNr;
  CellNr = u8_lCellNr;
  return u16_lCellSpg;
}


//Maximale Cell-Difference von allen aktiven BMSen ermitteln
uint16_t BmsDataUtils::getMaxCellDifferenceFromBms(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd)
{
  uint8_t u8_lBmsNr=0; //nur zum Debug
  uint8_t u8_lCellNr=0;  //nur zum Debug

  uint16_t u16_lCellDiff = getBmsMaxCellDifferenceVoltage(u8_mBmsDatasource);

  if(u16_mBmsDatasourceAdd>0)
  {
    uint16_t u16_lMaxCellDiff=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u16_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<CAN_BMS_COMMUNICATION_TIMEOUT) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          u16_lMaxCellDiff=getBmsMaxCellDifferenceVoltage(BMSDATA_FIRST_DEV_SERIAL+i);
          if(u16_lMaxCellDiff>u16_lCellDiff)
          {
            u16_lCellDiff=u16_lMaxCellDiff;
            u8_lBmsNr=BMSDATA_FIRST_DEV_SERIAL+i;  //nur zum Debug
            u8_lCellNr=getBmsMaxVoltageCellNumber(BMSDATA_FIRST_DEV_SERIAL+i); //nur zum Debug
          }
        }
      }
    }
    #ifdef CAN_DEBUG
    BSC_LOGD(TAG,"getMaxCellDifference: MaxDiff=%i, bms=%i, cell=%i",u16_lCellDiff, u8_lBmsNr, u8_lCellNr); //nur zum Debug
    #endif
  }
  #ifdef CAN_DEBUG
  else
  {
    BSC_LOGD(TAG,"getMaxCellDifference: MaxDiff=%i",u16_lCellDiff); //nur zum Debug
  }
  #endif

  return u16_lCellDiff;
}


void BmsDataUtils::buildBatteryCellText(char *buffer, uint8_t batteryNr, uint8_t cellNr)
{
  memset(buffer, 0, 8); // Clear
  if(batteryNr<BMSDATA_FIRST_DEV_SERIAL) snprintf(buffer, 8, "B%d C%d", batteryNr, cellNr);
  else snprintf(buffer, 8, "S%d C%d", batteryNr-BMSDATA_FIRST_DEV_SERIAL, cellNr);
}


// Ermitteln des niedrigsten Ladestroms der BMSen
float BmsDataUtils::getMinCurrentFromBms(uint8_t u8_mBmsDatasource, uint16_t u16_mBmsDatasourceAdd)
{
  float u16_lMinCurrent=0xFFFF;

  if((millis()-getBmsLastDataMillis(u8_mBmsDatasource))<CAN_BMS_COMMUNICATION_TIMEOUT)
  {
    u16_lMinCurrent = getBmsTotalCurrent(u8_mBmsDatasource);
  }

  if(u16_mBmsDatasourceAdd>0)
  {
    float u16_lMinCurrentTmp=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u16_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<CAN_BMS_COMMUNICATION_TIMEOUT) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          u16_lMinCurrentTmp=getBmsTotalCurrent(BMSDATA_FIRST_DEV_SERIAL+i);
          if(u16_lMinCurrentTmp<u16_lMinCurrent) u16_lMinCurrent=u16_lMinCurrentTmp;
        }
      }
    }
  }

  return u16_lMinCurrent;
}