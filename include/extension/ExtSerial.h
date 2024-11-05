// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef EXTSERIAL_H
#define EXTSERIAL_H

#include "extension/ExtDeviceI2C.h"
#include "defines.h"


class ExtSerial : public ExtDeviceI2C {
  public:
    ExtSerial(uint8_t address);
    ~ExtSerial();

    void initialize() override;

    void extSerialSetEnable(uint8_t u8_serialDevNr, serialRxTxEn_e serialRxTxEn);

};

#endif