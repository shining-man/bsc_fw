// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef EXTDISPLAY_H
#define EXTDISPLAY_H

#include "extension/ExtInterface_I2C.h"
#include "defines.h"


class ExtDisplay : public ExtInterface_I2C {
  public:
    ExtDisplay(uint8_t address, SemaphoreHandle_t &lMutexI2cRx);
    ~ExtDisplay();

    void initialize() override;

    void sendDataStr(Inverter &inverter, uint8_t data1, uint8_t data2, std::string data, uint8_t dataLen);
    void sendData(Inverter &inverter);
};

#endif