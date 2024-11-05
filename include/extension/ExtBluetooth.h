// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef EXTBLUETOOTH_H
#define EXTBLUETOOTH_H

#include "extension/ExtDeviceI2C.h"
#include "defines.h"


class ExtBluetooth : public ExtDeviceI2C {
public:
  ExtBluetooth(uint8_t address);
  ~ExtBluetooth();

  void initialize() override;

  void readBtData();

};

#endif