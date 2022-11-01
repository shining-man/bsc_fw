// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef BMSDATA_H
#define BMSDATA_H

#include <Arduino.h>

void bmsDataInit();
void bmsDataSemaphoreTake();
void bmsDataSemaphoreGive();

uint32_t getBmsCellVoltage(uint8_t devNr, uint8_t cellNr);
void  setBmsCellVoltage(uint8_t devNr, uint8_t cellNr, uint32_t value);

float getBmsCellResistance(uint8_t devNr, uint8_t cellNr);
void  setBmsCellResistance(uint8_t devNr, uint8_t cellNr, float value);

float getBmsTotalVoltage(uint8_t devNr);
void setBmsTotalVoltage(uint8_t devNr, float value);

uint32_t getBmsMaxCellDifferenceVoltage(uint8_t devNr);
void setBmsMaxCellDifferenceVoltage(uint8_t devNr, uint32_t value);

uint32_t getBmsAvgVoltage(uint8_t devNr);
void setBmsAvgVoltage(uint8_t devNr, uint32_t value);

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

uint32_t getBmsMaxCellVoltage(uint8_t devNr);
void setBmsMaxCellVoltage(uint8_t devNr, uint32_t value);

uint32_t getBmsMinCellVoltage(uint8_t devNr);
void setBmsMinCellVoltage(uint8_t devNr, uint32_t value);

uint32_t getBmsErrors(uint8_t devNr);
void setBmsErrors(uint8_t devNr, uint32_t value);

unsigned long getBmsLastDataMillis(uint8_t devNr);
void setBmsLastDataMillis(uint8_t devNr, unsigned long value);



#endif