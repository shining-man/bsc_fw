// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef EXTBLUETOOTH_H
#define EXTBLUETOOTH_H

#include "extension/ExtInterface_I2C.h"
#include "defines.h"


class ExtBluetooth : public ExtInterface_I2C {
public:
  ExtBluetooth(uint8_t address);
  ~ExtBluetooth();

  void initialize() override;

  void getBtBmsData();
  void getNeeySettings();
  void sendNeeySettings();

  void sendDataAfterParameterChange();

private:
  struct NeeySettings_s {
      uint8_t balancerOn;
      uint8_t cellCount;
      uint8_t batteryType;
      uint16_t batteryCapacity;
      float startVoltage;
      float maxBalanceCurrent;
      float sleepVoltage;
      float equalizationVoltage;
  } __attribute__((packed));

  void sendBtDeviceMACs();

};

#endif