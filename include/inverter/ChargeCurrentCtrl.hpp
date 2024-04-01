// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"


namespace nsChargeCurrentCtrl {
    class ChargeCurrentCtrl {
    public:
        ChargeCurrentCtrl();
        ~ChargeCurrentCtrl();

        void calcChargCurrent(Inverter &inverter, Inverter::inverterData_s &inverterData, bool alarmSetChargeCurrentToZero);

        #ifdef PIO_UNIT_TESTING
        int16_t calcMaximalenLadestromSprung(int16_t i16_pNewChargeCurrent, int16_t i16_lastChargeCurrent);
        int16_t calcChargeCurrentCutOff(Inverter::inverterData_s &inverterData, int16_t u16_lChargeCurrent);

        int16_t calcLadestromZellspanung(Inverter::inverterData_s &inverterData, int16_t i16_pMaxChargeCurrent);
        int16_t calcLadestromBeiZelldrift(Inverter::inverterData_s &inverterData, int16_t i16_pMaxChargeCurrent);
        int16_t calcLadestromSocAbhaengig(int16_t i16_lMaxChargeCurrent, uint8_t u8_lSoc);
        int16_t calcChargecurrent_MaxCurrentPerPackToHigh(int16_t i16_pMaxChargeCurrent);
        #endif

    private:
        #ifndef PIO_UNIT_TESTING
        int16_t calcMaximalenLadestromSprung(int16_t i16_pNewChargeCurrent, int16_t i16_lastChargeCurrent);
        int16_t calcChargeCurrentCutOff(Inverter::inverterData_s &inverterData, int16_t u16_lChargeCurrent);

        int16_t calcLadestromZellspanung(Inverter::inverterData_s &inverterData, int16_t i16_pMaxChargeCurrent);
        int16_t calcLadestromBeiZelldrift(Inverter::inverterData_s &inverterData, int16_t i16_pMaxChargeCurrent);
        int16_t calcLadestromSocAbhaengig(int16_t i16_lMaxChargeCurrent, uint8_t u8_lSoc);
        int16_t calcChargecurrent_MaxCurrentPerPackToHigh(int16_t i16_pMaxChargeCurrent);
        #endif
    };
}