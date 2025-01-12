// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef EXTSERIAL_H
#define EXTSERIAL_H

#include "extension/ExtInterface_I2C.h"
#include "defines.h"


class ExtSerial : public ExtInterface_I2C {
  public:
    ExtSerial(uint8_t address, SemaphoreHandle_t &lMutexI2cRx);
    ~ExtSerial();

    void initialize() override;

    void extSerialSetEnable(uint8_t u8_serialDevNr, serialRxTxEn_e serialRxTxEn);

};

#endif