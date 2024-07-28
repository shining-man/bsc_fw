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
      i16_mNewDisChargeCurrent = 0;
    }
    else
    {
      int16_t i16_lMaxDischargeCurrent = calcMaxDischargeCurrentProPack(inverterData);


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


  /*
   * Berechnet den maximal zulässigen Ladestrom aus allen vorhanden Packs
   */
  int16_t DisChargeCurrentCtrl::calcMaxDischargeCurrentProPack(Inverter::inverterData_s &inverterData)
  {
    int16_t i16_lMaxDischargeCurrent = (int16_t)WebSettings::getInt(ID_PARAM_BMS_MAX_DISCHARGE_CURRENT,0,DT_ID_PARAM_BMS_MAX_DISCHARGE_CURRENT);
    //u8_mModulesCntDischarge=1;

    //Maximalen Entladestrom aus den einzelnen Packs errechnen
    if(inverterData.bmsDatasourceAdd>0)
    {
      int16_t i16_lMaxCurrent=0;
      for(uint8_t i=0;i<MUBER_OF_DATA_DEVICES;i++)
      {
        if((inverterData.bmsDatasource)==i || (inverterData.bmsDatasourceAdd>>i)&0x01)
        {
          #ifdef CAN_DEBUG
          BSC_LOGD(TAG,"MaxDischargeCurrent Pack: i=%i, bmsErr=%i, FETdisState=%i, time=%i",
            i,getBmsErrors(i),getBmsStateFETsDischarge(i),millis()-getBmsLastDataMillis(i));
          #endif

          if(getBmsErrors(i)==0 && (millis()-getBmsLastDataMillis(i)<CAN_BMS_COMMUNICATION_TIMEOUT) &&
            getBmsStateFETsDischarge(i))
          {
            i16_lMaxCurrent+=WebSettings::getInt(ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT,i,DT_ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT);
            //u8_mModulesCntDischarge++;
          }
        }
      }
      if(i16_lMaxCurrent < i16_lMaxDischargeCurrent) i16_lMaxDischargeCurrent = i16_lMaxCurrent;
    }
    #ifdef CAN_DEBUG
    BSC_LOGI(TAG,"dischargeCurrent Pack:%i",i16_lMaxDischargeCurrent);
    #endif

    return i16_lMaxDischargeCurrent * 10;
  }


  /*******************************************************************************************************
   * Berechnet der Maximalzulässigen Entladestrom anhand der eigestellten Zellspannungsparameter
   *******************************************************************************************************/
  int16_t DisChargeCurrentCtrl::calcEntladestromZellspanung(Inverter::inverterData_s &inverterData, int16_t i16_pMaxDischargeCurrent)
  {
    if(WebSettings::getBool(ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_EN, 0))
    {
      uint16_t u16_lStartSpg = WebSettings::getInt(ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_STARTSPG,0,DT_ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_STARTSPG);

      //Kleinste Zellspannung von den aktiven BMSen ermitteln
      uint16_t u16_lAktuelleMinZellspg = BmsDataUtils::getMinCellSpannungFromBms(inverterData.bmsDatasource, inverterData.bmsDatasourceAdd);

      if(u16_lStartSpg>=u16_lAktuelleMinZellspg)
      {
        uint16_t u16_lEndSpg = WebSettings::getInt(ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_ENDSPG,0,DT_ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_ENDSPG);
        int16_t i16_lMindestDischargeCurrent = (WebSettings::getInt(ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM,0,DT_ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM));
        i16_lMindestDischargeCurrent *= 10;

        if(u16_lStartSpg<=u16_lEndSpg) return i16_pMaxDischargeCurrent; //Startspannung <= Endspannung => Fehler
        if(i16_pMaxDischargeCurrent<=i16_lMindestDischargeCurrent) return i16_pMaxDischargeCurrent; //Maximaler Entladestrom < Mindest-Entladestrom => Fehler

        if(u16_lAktuelleMinZellspg<u16_lEndSpg)
        {
          //Wenn die aktuelle Zellspannung bereits kleiner als die Endzellspannung ist,
          //dann Ladestrom auf Mindest-Entladestrom einstellen
          return i16_lMindestDischargeCurrent;
        }
        else
        {
          uint32_t u32_lAenderungProMv = ((u16_lStartSpg-u16_lEndSpg) * 100) / (i16_pMaxDischargeCurrent - i16_lMindestDischargeCurrent); //Änderung pro mV
          if(u32_lAenderungProMv == 0) return i16_pMaxDischargeCurrent;
          uint32_t u32_lStromAenderung = ((u16_lStartSpg - u16_lAktuelleMinZellspg) * 100) / u32_lAenderungProMv; //Ladestrom um den theoretisch reduziert werden muss

          if(u32_lStromAenderung > (i16_pMaxDischargeCurrent - i16_lMindestDischargeCurrent))
          {
            return i16_lMindestDischargeCurrent;
          }
          else
          {
            return i16_pMaxDischargeCurrent - u32_lStromAenderung; //neuer Entladestrom
          }

        }
      }
    }
    return i16_pMaxDischargeCurrent;
  }

}