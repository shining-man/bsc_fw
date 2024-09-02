// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef I2C_H
#define I2C_H

#include "Arduino.h"
#include "defines.h"

class Inverter; //Forward-Deklaration

void i2cInit();
void i2cTakeSemaphore();
void i2cCyclicRun(Inverter &inverter);
void i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, const void *dataAdr, uint8_t dataLen);
void i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, String data, uint8_t dataLen);
void i2cExtSerialSetEnable(uint8_t u8_serialDevNr, serialRxTxEn_e serialRxTxEn, bool takeSemaphore);
bool isSerialExtEnabled();

#endif