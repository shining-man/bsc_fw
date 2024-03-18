// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "inverter/ChargeVoltageCtrl.hpp"
#include "inverter/BmsDataUtils.hpp"
#include "defines.h"
#include "WebSettings.h"

namespace nsChargeVoltageCtrl
{
  ChargeVoltageCtrl::ChargeVoltageCtrl()
  {
    ;
  }

  ChargeVoltageCtrl::~ChargeVoltageCtrl()
  {
    ;
  }


  void ChargeVoltageCtrl::calcChargVoltage(Inverter &inverter, Inverter::inverterData_s &inverterData)
  {
    uint16_t u16_lChargeVoltage = (uint16_t)WebSettings::getFloat(ID_PARAM_BMS_MAX_CHARGE_SPG,0);
    u16_lChargeVoltage = calcDynamicReduzeChargeVolltage(inverterData, u16_lChargeVoltage);
  }


  /* */
  uint16_t ChargeVoltageCtrl::calcDynamicReduzeChargeVolltage(Inverter::inverterData_s &inverterData, uint16_t u16_lChargeVoltage)
  {
    static uint16_t u16_lDynamicChargeVoltage = u16_lChargeVoltage;

    if(WebSettings::getBool(ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_EN,0)==true) //wenn enabled
    {
      uint16_t u16_lStartZellVoltage = WebSettings::getInt(ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_ZELLSPG,0,DT_ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_ZELLSPG);
      uint16_t u16_lDeltaCellVoltage= WebSettings::getInt(ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_DELTA,0,DT_ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_DELTA);

      if(BmsDataUtils::getMaxCellSpannungFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd)>u16_lStartZellVoltage)
      {
        uint16_t u16_lMaxCellDiffVoltage = BmsDataUtils::getMaxCellDifferenceFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd);
        if(u16_lMaxCellDiffVoltage>u16_lDeltaCellVoltage)
        {
          u16_lDynamicChargeVoltage-=1; //1=100mV
          if(u16_lDynamicChargeVoltage<0)u16_lDynamicChargeVoltage=0;
          return u16_lDynamicChargeVoltage;
        }
        else if(u16_lMaxCellDiffVoltage<u16_lDeltaCellVoltage)
        {
          u16_lDynamicChargeVoltage+=1; //1=100mV
          if(u16_lDynamicChargeVoltage>u16_lChargeVoltage)u16_lDynamicChargeVoltage=u16_lChargeVoltage;
          return u16_lDynamicChargeVoltage;
        }
      }
    }
    return u16_lChargeVoltage;
  }
}