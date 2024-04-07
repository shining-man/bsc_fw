// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"


namespace nsDisChargeCurrentCtrl
{
    class DisChargeCurrentCtrl
    {
    public:
        DisChargeCurrentCtrl();
        ~DisChargeCurrentCtrl();

        void calcDisChargCurrent(Inverter &inverter, Inverter::inverterData_s &inverterData, bool alarmSetDischargeCurrentToZero);

    private:
        int16_t calcMaxDischargeCurrentProPack(Inverter::inverterData_s &inverterData);
        int16_t calcEntladestromZellspanung(Inverter::inverterData_s &inverterData, int16_t i16_pMaxDischargeCurrent);
    };
}