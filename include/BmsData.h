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
  //                                                                                 // NEEY 4A | JbdBms | JK-BMS | 
  uint16_t   bmsCellVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][24];          //    x    |   x    |   x    |
  //float    bmsCellResistance[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][24];       //    x    |        |        |
  float      bmsTotalVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];             //    x    |   x    |   x    |
  uint16_t   bmsMaxCellDifferenceVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT]; //    x    |   x    |   x    |
  uint16_t   bmsAvgVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];               //    x    |   x    |   x    |
  float      bmsTotalCurrent[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];             //         |   x    |   x    |
  uint16_t   bmsMaxCellVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];           //    x    |   x    |   x    |
  uint16_t   bmsMinCellVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];           //    x    |   x    |   x    |
  uint8_t    bmsMaxVoltageCellNumber[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];     //    x    |        |        |
  uint8_t    bmsMinVoltageCellNumber[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];     //    x    |        |        |
  uint8_t    bmsIsBalancingActive[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];        //    x    |        |        |
  float      bmsBalancingCurrent[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];         //    x    |        |        |
  float      bmsTempature[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][3];             //    2    |   3    |   3    |
  uint8_t    bmsChargePercentage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];         //         |   x    |   x    |
  uint32_t   bmsErrors[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];                   //    *    |   x    |   x    |
  unsigned long bmsLastDataMillis[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];        //    x    |   x    |   x    |
  //                                                                                 // *=Teilweise
};

/*bmsErrors
bit0  single cell overvoltage protection 
bit1  single cell undervoltage protection    
bit2  whole pack overvoltage protection 
bit3  Whole pack undervoltage protection     
bit4  charging over-temperature protection 
bit5  charging low temperature protection 
bit6  Discharge over temperature protection  
bit7  discharge low temperature protection   
bit8  charging overcurrent protection 
bit9  Discharge overcurrent protection       
bit10 short circuit protection              
bit11 Front-end detection IC error 
bit12 software lock MOS 
*/

void bmsDataInit();
void bmsDataSemaphoreTake();
void bmsDataSemaphoreGive();

struct bmsData_s * getBmsData();

uint16_t getBmsCellVoltage(uint8_t devNr, uint8_t cellNr);
void setBmsCellVoltage(uint8_t devNr, uint8_t cellNr, uint16_t value);

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

unsigned long getBmsLastDataMillis(uint8_t devNr);
void setBmsLastDataMillis(uint8_t devNr, unsigned long value);



#endif