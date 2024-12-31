// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"


namespace nsInverterBattery
{
  class InverterBattery
  {
  public:
    InverterBattery();
    ~InverterBattery();

    void getBatteryVoltage(Inverter &inverter, Inverter::inverterData_s &inverterData);
    void getBatteryCurrent(Inverter &inverter, Inverter::inverterData_s &inverterData);
    int16_t getBatteryTemp(Inverter::inverterData_s &inverterData);
    void getBatteryCapacity(uint8_t mBmsDatasource, uint16_t mBmsDatasourceAdd, 
      uint16_t &onlineCapacity, uint16_t &totalCapacity);
        
  private:
      bool isBatteryCapacityAvailable(uint8_t batteryNumber);

  };
}