// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "inverter/DisChargeCurrentCtrl.hpp"
#include "inverter/BmsDataUtils.hpp"
#include "defines.h"
#include "WebSettings.h"

namespace nsDisChargeCurrentCtrl
{
  DisChargeCurrentCtrl::DisChargeCurrentCtrl()
  {
    ;
  }

  DisChargeCurrentCtrl::~DisChargeCurrentCtrl()
  {
    ;
  }

  void DisChargeCurrentCtrl::calcDisChargCurrent(Inverter &inverter, Inverter::inverterData_s &inverterData, bool alarmSetDischargeCurrentToZero)
  {
    int16_t i16_lMaxDischargeCurrentList[1] = {0};
    int16_t i16_mNewDisChargeCurrent = 0;

    if(alarmSetDischargeCurrentToZero)
    {
      i16_mNewDisChargeCurrent =0;
    }
    else
    {
      int16_t i16_lMaxDischargeCurrent = (int16_t)WebSettings::getInt(ID_PARAM_BMS_MAX_DISCHARGE_CURRENT,0,DT_ID_PARAM_BMS_MAX_DISCHARGE_CURRENT);
      //u8_mModulesCntDischarge=1;

      //Maximalen Entladestrom aus den einzelnen Packs errechnen
      if(inverterData.u16_bmsDatasourceAdd>0)
      {
        int16_t i16_lMaxCurrent=0;
        for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
        {
          if((inverterData.u8_bmsDatasource-BMSDATA_FIRST_DEV_SERIAL)==i || (inverterData.u16_bmsDatasourceAdd>>i)&0x01)
          {
            #ifdef CAN_DEBUG
            BSC_LOGD(TAG,"MaxDischargeCurrent Pack: i=%i, bmsErr=%i, FETdisState=%i, time=%i",
              i,getBmsErrors(BMSDATA_FIRST_DEV_SERIAL+i),getBmsStateFETsDischarge(BMSDATA_FIRST_DEV_SERIAL+i),millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i));
            #endif

            if(getBmsErrors(BMSDATA_FIRST_DEV_SERIAL+i)==0 && (millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i)<CAN_BMS_COMMUNICATION_TIMEOUT) &&
              getBmsStateFETsDischarge(BMSDATA_FIRST_DEV_SERIAL+i))
            {
              i16_lMaxCurrent+=WebSettings::getInt(ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT,i,DT_ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT);
              //u8_mModulesCntDischarge++;
            }
          }
        }
        if(i16_lMaxCurrent<i16_lMaxDischargeCurrent) i16_lMaxDischargeCurrent=i16_lMaxCurrent;
      }
      #ifdef CAN_DEBUG
      BSC_LOGI(TAG,"dischargeCurrent Pack:%i",i16_lMaxDischargeCurrent);
      #endif


      i16_lMaxDischargeCurrentList[0] = calcEntladestromZellspanung(inverterData, i16_lMaxDischargeCurrent);

      //Bestimmt kleinsten Entladestrom aller Optionen
      for(uint8_t i=0;i<sizeof(i16_lMaxDischargeCurrentList)/sizeof(i16_lMaxDischargeCurrentList[0]);i++)
      {
        if(i16_lMaxDischargeCurrentList[i] < i16_lMaxDischargeCurrent)
        {
          i16_lMaxDischargeCurrent = i16_lMaxDischargeCurrentList[i];
        }
      }

      i16_mNewDisChargeCurrent = i16_lMaxDischargeCurrent;
    }

    inverter.inverterDataSemaphoreTake();
    inverterData.inverterDischargeCurrent = i16_mNewDisChargeCurrent;
    inverterData.calcDischargeCurrentCellVoltage = i16_lMaxDischargeCurrentList[0];
    inverter.inverterDataSemaphoreGive();
  }



  /*******************************************************************************************************
   * Berechnet der Maximalzulässigen Entladestrom anhand der eigestellten Zellspannungsparameter
   *******************************************************************************************************/
  int16_t DisChargeCurrentCtrl::calcEntladestromZellspanung(Inverter::inverterData_s &inverterData, int16_t i16_pMaxDischargeCurrent)
  {
    uint16_t u16_lStartSpg = WebSettings::getInt(ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_STARTSPG,0,DT_ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_STARTSPG);

    if(u16_lStartSpg>0) //wenn enabled
    {
      //Kleinste Zellspannung von den aktiven BMSen ermitteln
      uint16_t u16_lAktuelleMinZellspg = BmsDataUtils::getMinCellSpannungFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd);

      if(u16_lStartSpg>=u16_lAktuelleMinZellspg)
      {
        uint16_t u16_lEndSpg = WebSettings::getInt(ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_ENDSPG,0,DT_ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_ENDSPG);
        int16_t i16_lMindestChargeCurrent = (WebSettings::getInt(ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM,0,DT_ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM));

        if(u16_lStartSpg<=u16_lEndSpg) return i16_pMaxDischargeCurrent; //Startspannung <= Endspannung => Fehler
        if(i16_pMaxDischargeCurrent<=i16_lMindestChargeCurrent) return i16_pMaxDischargeCurrent; //Maximaler Entladestrom < Mindest-Entladestrom => Fehler

        if(u16_lAktuelleMinZellspg<u16_lEndSpg)
        {
          //Wenn die aktuelle Zellspannung bereits kleiner als die Endzellspannung ist,
          //dann Ladestrom auf Mindest-Entladestrom einstellen
          return i16_lMindestChargeCurrent;
        }
        else
        {
          uint32_t u32_lAenderungProMv = ((u16_lStartSpg-u16_lEndSpg)*100)/(i16_pMaxDischargeCurrent-i16_lMindestChargeCurrent); //Änderung pro mV
          uint32_t u32_lStromAenderung = ((u16_lStartSpg-u16_lAktuelleMinZellspg)*100)/u32_lAenderungProMv; //Ladestrom um den theoretisch reduziert werden muss
          if(u32_lStromAenderung>(i16_pMaxDischargeCurrent-i16_lMindestChargeCurrent))
          {
            return i16_lMindestChargeCurrent;
          }
          else
          {
            uint16_t u16_lNewChargeCurrent = i16_pMaxDischargeCurrent-u32_lStromAenderung; //neuer Entladestrom
            return u16_lNewChargeCurrent;
          }

        }
      }
    }
    return i16_pMaxDischargeCurrent;
  }

}