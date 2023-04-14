// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef BMSDATA_H
#define BMSDATA_H

#include <Arduino.h>
#include "defines.h"


struct bmsData_s
{
  /*                                                                 // N | J | J | S | J | D |
                                                                     // E | b | K | e | K | A |
                                                                     // E | d |   | p |   | L |
                                                                     // Y |   |   | l | B | Y |
                                                                     // 4 |   |   | o | T |   |
                                                                     // A |   |   | s |   |   |
                                                                     //---|---|---|---|---|---|*/
  uint16_t   bmsCellVoltage[BMSDATA_NUMBER_ALLDEVICES][24];          // x | x | x | x |   | x |
  //float    bmsCellResistance[BMSDATA_NUMBER_ALLDEVICES][24];       // x |   |   |   |   |   |
  float      bmsTotalVoltage[BMSDATA_NUMBER_ALLDEVICES];             // x | x | x | x |   | x |
  uint16_t   bmsMaxCellDifferenceVoltage[BMSDATA_NUMBER_ALLDEVICES]; // x | x | x | x |   | x |
  uint16_t   bmsAvgVoltage[BMSDATA_NUMBER_ALLDEVICES];               // x | x | x | x |   | x |
  float      bmsTotalCurrent[BMSDATA_NUMBER_ALLDEVICES];             // - | x | x | x |   | x |
  uint16_t   bmsMaxCellVoltage[BMSDATA_NUMBER_ALLDEVICES];           // x | x | x | x |   | x |
  uint16_t   bmsMinCellVoltage[BMSDATA_NUMBER_ALLDEVICES];           // x | x | x | x |   | x |
  uint8_t    bmsMaxVoltageCellNumber[BMSDATA_NUMBER_ALLDEVICES];     // x |   |   |   |   | x |
  uint8_t    bmsMinVoltageCellNumber[BMSDATA_NUMBER_ALLDEVICES];     // x |   |   |   |   | X |
  uint8_t    bmsIsBalancingActive[BMSDATA_NUMBER_ALLDEVICES];        // x |   |   |   |   | x |
  int16_t    bmsBalancingCurrent[BMSDATA_NUMBER_ALLDEVICES];         // x |   |   |   |   |   |
  float      bmsTempature[BMSDATA_NUMBER_ALLDEVICES][3];             // 2 | 3 | 3 | 3 |   | 3 |
  uint8_t    bmsChargePercentage[BMSDATA_NUMBER_ALLDEVICES];         // - | x | x | x |   | x |
  uint32_t   bmsErrors[BMSDATA_NUMBER_ALLDEVICES];                   // * | x | x |   |   |   |
  uint8_t    bmsStateFETs[BMSDATA_NUMBER_ALLDEVICES];                // - | x | x | x | x | x | bit 0=FET charge, bit 1=FET discharge
  unsigned long bmsLastDataMillis[BMSDATA_NUMBER_ALLDEVICES];        // x | x | x | x | x |   |
  //                                                                 // *=Teilweise; -=Nicht verfügbar
};



//bmsErrors
#define BMS_ERR_STATUS_OK                0
#define BMS_ERR_STATUS_CELL_OVP          1   //bit0  single cell overvoltage protection 
#define BMS_ERR_STATUS_CELL_UVP          2   //bit1  single cell undervoltage protection    
#define BMS_ERR_STATUS_BATTERY_OVP       4   //bit2  whole pack overvoltage protection 
#define BMS_ERR_STATUS_BATTERY_UVP       8   //bit3  Whole pack undervoltage protection     
#define BMS_ERR_STATUS_CHG_OTP          16   //bit4  charging over temperature protection 
#define BMS_ERR_STATUS_CHG_UTP          32   //bit5  charging low temperature protection 
#define BMS_ERR_STATUS_DSG_OTP          64   //bit6  Discharge over temperature protection  
#define BMS_ERR_STATUS_DSG_UTP         128   //bit7  discharge low temperature protection   
#define BMS_ERR_STATUS_CHG_OCP         256   //bit8  charging overcurrent protection 
#define BMS_ERR_STATUS_DSG_OCP         512   //bit9  Discharge overcurrent protection       
#define BMS_ERR_STATUS_SHORT_CIRCUIT  1024   //bit10 short circuit protection              
#define BMS_ERR_STATUS_AFE_ERROR      2048   //bit11 Front-end detection IC error 
#define BMS_ERR_STATUS_SOFT_LOCK      4096   //bit12 software lock MOS 
#define BMS_ERR_STATUS_RESERVED1      8192   //bit13 Reserved 
#define BMS_ERR_STATUS_RESERVED2     16384   //bit14 Reserved
#define BMS_ERR_STATUS_RESERVED3     32768   //bit15 Reserved 


//Filter
struct bmsFilterData_s
{
  uint8_t u8_mFilterBmsCellVoltagePercent;
};


void bmsDataInit();
void bmsDataSemaphoreTake();
void bmsDataSemaphoreGive();

struct bmsData_s* getBmsData();
struct bmsFilterData_s* getBmsFilterData();
uint8_t* getBmsFilterErrorCounter(uint8_t);

uint16_t getBmsCellVoltage(uint8_t devNr, uint8_t cellNr);
bool setBmsCellVoltage(uint8_t devNr, uint8_t cellNr, uint16_t value);

//float getBmsCellResistance(uint8_t devNr, uint8_t cellNr);
//void  setBmsCellResistance(uint8_t devNr, uint8_t cellNr, float value);

float getBmsTotalVoltage(uint8_t devNr);
void setBmsTotalVoltage(uint8_t devNr, float value);

uint16_t getBmsMaxCellDifferenceVoltage(uint8_t devNr);
void setBmsMaxCellDifferenceVoltage(uint8_t devNr, uint16_t value);

uint16_t getBmsAvgVoltage(uint8_t devNr);
void setBmsAvgVoltage(uint8_t devNr, uint16_t value);

float getBmsTotalCurrent(uint8_t devNr);
void setBmsTotalCurrent(uint8_t devNr, float value);

uint8_t getBmsMaxVoltageCellNumber(uint8_t devNr);
void setBmsMaxVoltageCellNumber(uint8_t devNr, uint8_t value);

uint8_t getBmsMinVoltageCellNumber(uint8_t devNr);
void setBmsMinVoltageCellNumber(uint8_t devNr, uint8_t value);

uint8_t getBmsIsBalancingActive(uint8_t devNr);
void setBmsIsBalancingActive(uint8_t devNr, uint8_t value);

float getBmsBalancingCurrent(uint8_t devNr);
void setBmsBalancingCurrent(uint8_t devNr, float value);

float getBmsTempature(uint8_t devNr, uint8_t sensorNr);
void setBmsTempature(uint8_t devNr, uint8_t sensorNr, float value);

uint8_t getBmsChargePercentage(uint8_t devNr);
void setBmsChargePercentage(uint8_t devNr, uint8_t value);

uint16_t getBmsMaxCellVoltage(uint8_t devNr);
void setBmsMaxCellVoltage(uint8_t devNr, uint16_t value);

uint16_t getBmsMinCellVoltage(uint8_t devNr);
void setBmsMinCellVoltage(uint8_t devNr, uint16_t value);

uint32_t getBmsErrors(uint8_t devNr);
void setBmsErrors(uint8_t devNr, uint32_t value);

uint8_t getBmsStateFETs(uint8_t devNr);
void    setBmsStateFETs(uint8_t devNr, uint8_t value);
boolean getBmsStateFETsCharge(uint8_t devNr);
void    setBmsStateFETsCharge(uint8_t devNr, boolean value);
boolean getBmsStateFETsDischarge(uint8_t devNr);
void    setBmsStateFETsDischarge(uint8_t devNr, boolean value);

unsigned long getBmsLastDataMillis(uint8_t devNr);
void setBmsLastDataMillis(uint8_t devNr, unsigned long value);

uint8_t getBmsDataBytes(uint8_t dataType);

uint8_t * getBmsSettingsReadback(uint8_t bmsNr);

#endif