// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"


namespace nsSocCtrl {
    class SocCtrl {
    public:
        SocCtrl();
        ~SocCtrl();

        enum SM_SocZellspgStates {STATE_MINCELLSPG_SOC_WAIT_OF_MIN=0, STATE_MINCELLSPG_SOC_BELOW_MIN, STATE_MINCELLSPG_SOC_LOCKTIMER};

        uint16_t calcSoc(Inverter &inverter, Inverter::inverterData_s &inverterData, bool alarmSetSocToFull);

    private:
        uint8_t getNewSocByMinCellVoltage(Inverter::inverterData_s &inverterData, uint8_t u8_lSoc);
    };
}