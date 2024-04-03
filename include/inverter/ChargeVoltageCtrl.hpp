// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"


namespace nsChargeVoltageCtrl
{
  class ChargeVoltageCtrl
  {
  public:
    ChargeVoltageCtrl();
    ~ChargeVoltageCtrl();

    void calcChargVoltage(Inverter &inverter, Inverter::inverterData_s &inverterData);

    enum e_stateAutobalance {STATE_AUTOBAL_OFF, STATE_AUTOBAL_WAIT, STATE_AUTOBAL_WAIT_START_VOLTAGE, STATE_AUTOBAL_RUNING};

  private:
    uint16_t calcDynamicReduzeChargeVolltage(Inverter::inverterData_s &inverterData, uint16_t u16_lChargeVoltage);
    void setAutobalanceVoltage(Inverter::inverterData_s &inverterData, uint16_t &u16_lChargeVoltage);
    void calcDynamicChargeVoltageOffset(Inverter::inverterData_s &inverterData, uint16_t &u16_lChargeVoltage);
  };
}