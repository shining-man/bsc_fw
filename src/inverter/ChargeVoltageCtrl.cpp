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
    uint16_t u16_lChargeVoltage = (uint16_t)WebSettings::getInt(ID_PARAM_BMS_MAX_CHARGE_SPG,0,DT_ID_PARAM_BMS_MAX_CHARGE_SPG);

    //calcDynamicChargeVoltageOffset(inverterData, u16_lChargeVoltage); // Hier den dynamischen Offset addieren
    setAutobalanceVoltage(inverterData, u16_lChargeVoltage); //Ggf. die Ladespannung anheben auf die Autobalancespannung
    u16_lChargeVoltage = calcDynamicReduzeChargeVolltage(inverterData, u16_lChargeVoltage);

    inverter.inverterDataSemaphoreTake();
    inverterData.inverterChargeVoltage = u16_lChargeVoltage; //ToDo semaphore
    inverter.inverterDataSemaphoreGive();
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


  /*
   * Autobalancing
   */
  void ChargeVoltageCtrl::setAutobalanceVoltage(Inverter::inverterData_s &inverterData, uint16_t &u16_lChargeVoltage)
  {
    // Wenn deaktiviert
    if(WebSettings::getBool(ID_PARAM_INVERTER_AUTOBALANCE_ENABLE, 0) == false)
    {
      inverterData.mStateAutobalance=STATE_AUTOBAL_OFF;
      return;
    }

    if(inverterData.mStateAutobalance==STATE_AUTOBAL_OFF)
    {
      inverterData.lastAutobalanceRun=millis();
      inverterData.mStateAutobalance=STATE_AUTOBAL_WAIT;
    }

    // Warte auf den Start (Intervall)
    if(inverterData.mStateAutobalance==STATE_AUTOBAL_WAIT)
    {
      const uint8_t u16_lStartInterval = WebSettings::getInt(ID_PARAM_INVERTER_AUTOBALANCE_START_INTERVAL,0,DT_ID_PARAM_INVERTER_AUTOBALANCE_START_INTERVAL);

      #ifdef UTEST_RESTAPI
      if(millis()-inverterData.lastAutobalanceRun > ((uint32_t)u16_lStartInterval*1000))
      #else
      if(millis()-inverterData.lastAutobalanceRun > ((uint32_t)u16_lStartInterval*1440000)) // Tage: 24*60*1000
      #endif
      {
        inverterData.mStateAutobalance=STATE_AUTOBAL_WAIT_START_VOLTAGE;
      }
    }

    // Wenn Intervall erreicht, dann auf Start-Cellvoltage warten
    else if(inverterData.mStateAutobalance==STATE_AUTOBAL_WAIT_START_VOLTAGE)
    {
      const uint16_t u16_lStartCellVoltage = WebSettings::getInt(ID_PARAM_INVERTER_AUTOBALANCE_START_CELLVOLTAGE,0,DT_ID_PARAM_INVERTER_AUTOBALANCE_START_CELLVOLTAGE);
      if(BmsDataUtils::getMaxCellSpannungFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd) >= u16_lStartCellVoltage)
      {
        inverterData.mStateAutobalance=STATE_AUTOBAL_RUNING;
        inverterData.autobalanceStartTime=millis();
      }
    }

    // Autobalancing läuft
    else if(inverterData.mStateAutobalance==STATE_AUTOBAL_RUNING)
    {
      // Timeout
      const uint16_t u16_lTimeout = (uint16_t)WebSettings::getInt(ID_PARAM_INVERTER_AUTOBALANCE_TIMEOUT,0,DT_ID_PARAM_INVERTER_AUTOBALANCE_TIMEOUT);
      if(millis()-inverterData.autobalanceStartTime > (u16_lTimeout*60*1000)) // if timeout
      {
        // ToDo: MQTT Message, Trigger?
        inverterData.lastAutobalanceRun=millis();
        inverterData.mStateAutobalance=STATE_AUTOBAL_WAIT;
        return;
      }

      // Finish
      const uint8_t u8_lCelldifFinish = WebSettings::getInt(ID_PARAM_INVERTER_AUTOBALANCE_CELLDIF_FINISH,0,DT_ID_PARAM_INVERTER_AUTOBALANCE_CELLDIF_FINISH);
      if(BmsDataUtils::getMaxCellDifferenceFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd) <= u8_lCelldifFinish)
      {
        inverterData.lastAutobalanceRun=millis();
        inverterData.mStateAutobalance=STATE_AUTOBAL_WAIT;
        return;
      }

      // Ladespannung einstellen
      u16_lChargeVoltage = WebSettings::getInt(ID_PARAM_INVERTER_AUTOBALANCE_CHARGE_VOLTAGE,0,DT_ID_PARAM_INVERTER_AUTOBALANCE_CHARGE_VOLTAGE);
    }

    // Error
    /*else
    {
    }*/
  }


    /*
    * Berechnen des dynamischen Offsets
    */
    void ChargeVoltageCtrl::calcDynamicChargeVoltageOffset(Inverter::inverterData_s &inverterData, uint16_t &u16_lChargeVoltage)
    {
      uint16_t u16_lCurrent = WebSettings::getInt(ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_CURRENT,0,DT_ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_CURRENT);
      if(u16_lCurrent>0)
      {
        uint16_t u16_lOffsetMin = WebSettings::getInt(ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_OFFSET_MIN,0,DT_ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_OFFSET_MIN);
        uint16_t u16_lOffsetMax = WebSettings::getInt(ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_OFFSET_MAX,0,DT_ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_OFFSET_MAX);
        int16_t u16_lMinBmsCurrent = (int16_t)BmsDataUtils::getMinCurrentFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd);

        if(u16_lMinBmsCurrent>=u16_lCurrent) u16_lChargeVoltage+=u16_lOffsetMax;
        else if(u16_lMinBmsCurrent>0) u16_lChargeVoltage+=(u16_lOffsetMin+((u16_lOffsetMax-u16_lOffsetMin)/u16_lCurrent*u16_lMinBmsCurrent));
      }
    }



}