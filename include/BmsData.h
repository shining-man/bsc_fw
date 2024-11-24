// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef BMSDATA_H
#define BMSDATA_H

#include <Arduino.h>
#include "defines.h"
#include "BmsDataTypes.hpp"

struct bmsData_s
{
                                                                               // N | J | J | S | J | D | S | J | G | G | S | B | S | J |
                                                                               // E | b | K | e | K | A | Y | K | O | O | M | P | e | K |
                                                                               // E | d |   | p |   | L | L |   | B | B | A | N | p |   |
                                                                               // Y |   |   | l | B | Y | C | V | E | E | R |   | l | I |
                                                                               // 4 |   |   | o | T |   | I | 1 | L | L | T |   | o | N |
                                                                               // A |   |   | s |   |   | N | 3 |   |   | S |   | s | V |
                                                                               //   |   |   |   |   |   |   |   | R | P | H |   |   | . |
                                                                               //   |   |   |   |   |   |   |   | N | C | U |   | v |   |
                                                                               //   |   |   |   |   |   |   |   | 1 | 2 | N |   | 3 |   |
                                                                               //   |   |   |   |   |   |   |   | 5 | 0 | T |   |   |   |
                                                                               //   |   |   |   |   |   |   |   | 0 | 0 |   |   |   |   |
                                                                               //---|---|---|---|---|---|---|---|---|---|---|---|---|---|
  uint16_t   bmsCellVoltage[MUBER_OF_DATA_DEVICES][24];                    // x | x | x | x |   | x | x | x | x | x |   |   | x | x |
  //float    bmsCellResistance[MUBER_OF_DATA_DEVICES][24];                 // x |   |   |   |   |   |   |   |   |   |   |   |   |   |
  int16_t    bmsTotalVoltage[MUBER_OF_DATA_DEVICES];                       // x | x | x | x |   | x | x | x | x | x | x |   | x | x |
  uint16_t   bmsMaxCellDifferenceVoltage[MUBER_OF_DATA_DEVICES];           // x | x | x | x |   | x | x | x | x | x |   |   | x | x |
  uint16_t   bmsAvgVoltage[MUBER_OF_DATA_DEVICES];                         // x | x | x | x |   | x | x | x | x | x |   |   | x | x |
  int16_t    bmsTotalCurrent[MUBER_OF_DATA_DEVICES];                       // - | x | x | x |   | x | x | - | x | x | x |   | x | x |
  uint16_t   bmsMaxCellVoltage[MUBER_OF_DATA_DEVICES];                     // x | x | x | x |   | x | x | c | x | x |   |   | x | x |
  uint16_t   bmsMinCellVoltage[MUBER_OF_DATA_DEVICES];                     // x | x | x | x |   | x | x | c | x | x |   |   | x | x |
  uint8_t    bmsMaxVoltageCellNumber[MUBER_OF_DATA_DEVICES];               // x |   |   |   |   | x |   | c | x | x |   |   | x | x |
  uint8_t    bmsMinVoltageCellNumber[MUBER_OF_DATA_DEVICES];               // x |   |   |   |   | x |   | c | x | x |   |   | x | x |
  uint8_t    bmsIsBalancingActive[MUBER_OF_DATA_DEVICES];                  // x |   |   |   |   | x |   | x |   |   |   |   | x | x |
  int16_t    bmsBalancingCurrent[MUBER_OF_DATA_DEVICES];                   // x |   |   |   |   |   |   | x |   |   |   |   |   | x |
  int16_t    bmsTempature[MUBER_OF_DATA_DEVICES][NR_OF_BMS_TEMP_SENSORS];  // 2 | 3 | 3 | 6 |   | 3 | 3 | x | 3 | 3 |   |   | 6 | 5 |
  uint8_t    bmsChargePercentage[MUBER_OF_DATA_DEVICES];                   // - | x | x | x |   | x | x | - | x | x | x |   | x | x |
  uint32_t   bmsErrors[MUBER_OF_DATA_DEVICES];                             // * | x | x |   |   |   | x | * | x | x |   |   | x |   |
  uint32_t   bmsWarnings[MUBER_OF_DATA_DEVICES];                           //   |   |   | x |   |   |   |   |   |   |   |   |   |   |
  uint8_t    bmsStateFETs[MUBER_OF_DATA_DEVICES];                          // - | x | x | x | x | x | x | - | - | - |   |   | x |   | bit 0=FET charge, bit 1=FET discharge
  unsigned long bmsLastDataMillis[MUBER_OF_DATA_DEVICES];                  // x | x | x | x | x |   | x | x | x | x |   |   | x |   |
  uint16_t   bmsCellVoltageCrc[MUBER_OF_DATA_DEVICES];                     // Wird nach dem Holen Daten vom BMS berechnet
  uint8_t    bmsLastChangeCellVoltageCrc[MUBER_OF_DATA_DEVICES];           // Wird nach dem Holen Daten vom BMS berechnet
  //                                                                           // *=Teilweise; -=Nicht verfügbar; c=wird berechnet
};


//Filter
struct bmsFilterData_s
{
  uint8_t u8_mFilterBmsCellVoltagePercent;
};

struct btDevData_s
{
  uint8_t macAdresse[6];
  char    devName[10];
  uint8_t settingsReadback[32];
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
void setBmsTotalVoltage_int(uint8_t devNr, int16_t value);

uint16_t getBmsMaxCellDifferenceVoltage(uint8_t devNr);
void setBmsMaxCellDifferenceVoltage(uint8_t devNr, uint16_t value);

uint16_t getBmsAvgVoltage(uint8_t devNr);
void setBmsAvgVoltage(uint8_t devNr, uint16_t value);

float getBmsTotalCurrent(uint8_t devNr);
void setBmsTotalCurrent(uint8_t devNr, float value);
void setBmsTotalCurrent_int(uint8_t devNr, int16_t value);

uint8_t getBmsMaxVoltageCellNumber(uint8_t devNr);
void setBmsMaxVoltageCellNumber(uint8_t devNr, uint8_t value);

uint8_t getBmsMinVoltageCellNumber(uint8_t devNr);
void setBmsMinVoltageCellNumber(uint8_t devNr, uint8_t value);

uint8_t getBmsIsBalancingActive(uint8_t devNr);
void setBmsIsBalancingActive(uint8_t devNr, uint8_t value);

float getBmsBalancingCurrent(uint8_t devNr);
void setBmsBalancingCurrent(uint8_t devNr, float value);
void setBmsBalancingCurrentI16(uint8_t devNr, int16_t value);

float getBmsTempature(uint8_t devNr, uint8_t sensorNr);
int16_t getBmsTempatureI16(uint8_t devNr, uint8_t sensorNr);
void setBmsTempature(uint8_t devNr, uint8_t sensorNr, float value);
void setBmsTempatureI16(uint8_t devNr, uint8_t sensorNr, int16_t value);

uint8_t getBmsChargePercentage(uint8_t devNr);
void setBmsChargePercentage(uint8_t devNr, uint8_t value);

uint16_t getBmsMaxCellVoltage(uint8_t devNr);
void setBmsMaxCellVoltage(uint8_t devNr, uint16_t value);

uint16_t getBmsMinCellVoltage(uint8_t devNr);
void setBmsMinCellVoltage(uint8_t devNr, uint16_t value);

uint32_t getBmsErrors(uint8_t devNr);
void setBmsErrors(uint8_t devNr, uint32_t value);
void setBmsErrors(uint8_t devNr, const BmsErrorStatus& status);
void ignoreTemporarilyUndervoltageAlarm();

uint32_t getBmsWarnings(uint8_t devNr);
void setBmsWarnings(uint8_t devNr, uint32_t value);

uint8_t getBmsStateFETs(uint8_t devNr);
void    setBmsStateFETs(uint8_t devNr, uint8_t value);
bool    getBmsStateFETsCharge(uint8_t devNr);
void    setBmsStateFETsCharge(uint8_t devNr, boolean value);
bool    getBmsStateFETsDischarge(uint8_t devNr);
void    setBmsStateFETsDischarge(uint8_t devNr, boolean value);

uint16_t getBmsCellVoltageCrc(uint8_t devNr);
void setBmsCellVoltageCrc(uint8_t devNr, uint16_t value);
uint8_t getBmsLastChangeCellVoltageCrc(uint8_t devNr);
void setBmsLastChangeCellVoltageCrc(uint8_t devNr, uint8_t value);

unsigned long getBmsLastDataMillis(uint8_t devNr);
void setBmsLastDataMillis(uint8_t devNr, unsigned long value);

//write serial data
uint8_t *getSerialBmsWriteData(uint8_t devNr, serialDataRwTyp_e *dataTyp, uint8_t *rwDataLen);
void setSerialBmsWriteData(serialDataRwTyp_e dataTyp, uint8_t *data, uint8_t dataLen);
void addSerialBmsWriteDevNr(uint8_t devNr);
void clearSerialBmsWriteData(uint8_t devNr);

//read serial data
void setSerialBmsReadData(uint8_t devNr, serialDataRwTyp_e dataTyp, uint8_t *data, uint8_t dataLen);
uint8_t *getSerialBmsReadeData(uint8_t devNr, serialDataRwTyp_e *dataTyp, uint8_t *rwDataLen);
void clearSerialBmsReadData(uint8_t devNr);

uint8_t getBmsDataBytes(uint8_t dataType);

uint8_t * getBmsSettingsReadback(uint8_t bmsNr);
struct btDevData_s *getBmsDeviceData(uint8_t bmsNr);

bool isMultiple485bms(uint8_t bms);
bool haveAllBmsFirstData();

#ifdef LOG_BMS_DATA
void logBmsData(uint8_t bmsNr);
#endif

#endif
