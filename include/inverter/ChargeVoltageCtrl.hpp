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

    private:
        uint16_t calcDynamicReduzeChargeVolltage(Inverter::inverterData_s &inverterData, uint16_t u16_lChargeVoltage);
    };
}