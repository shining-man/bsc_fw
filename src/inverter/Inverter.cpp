// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "inverter/Inverter.hpp"
#include "inverter/BmsDataUtils.hpp"
#include "inverter/ChargeCurrentCtrl.hpp"
#include "inverter/DisChargeCurrentCtrl.hpp"
#include "inverter/ChargeVoltageCtrl.hpp"
#include "inverter/SocCtrl.hpp"
#include "inverter/InverterBattery.hpp"
#include "inverter/Canbus.hpp"

#include "WebSettings.h"
#include "defines.h"
#include "BmsData.h"
#include "Ow.h"
#include "mqtt_t.h"
#include "log.h"
#include "AlarmRules.h"

const char *Inverter::TAG = "Inverter";

nsCanbus::Canbus canbus;

Inverter::Inverter() :
u8_mMqttTxTimer(0),
alarmSetChargeCurrentToZero(false),
alarmSetDischargeCurrentToZero(false),
alarmSetSocToFull(false)
{

}

Inverter::~Inverter() {

}

void Inverter::inverterInit()
{
  mInverterDataMutex = xSemaphoreCreateMutex();

  //u8_mBmsDatasource = 0;
  alarmSetChargeCurrentToZero = false;
  alarmSetDischargeCurrentToZero = false;
  alarmSetSocToFull = false;

  inverterData.u16_mSocZellspannungSperrzeitTimer = 0;
  inverterData.u8_mSocZellspannungState = nsSocCtrl::SocCtrl::SM_SocZellspgStates::STATE_MINCELLSPG_SOC_WAIT_OF_MIN;

  // Autoblance
  inverterData.mStateAutobalance = nsChargeVoltageCtrl::ChargeVoltageCtrl::e_stateAutobalance::STATE_AUTOBAL_WAIT;
  inverterData.lastAutobalanceRun = millis();
  inverterData.autobalanceStartTime = 0;

  // Cut-Off
  inverterData.mChargeCurrentCutOffMittelwert = 0;
  inverterData.mChargeCurrentCutOffMittelwertCounter = 0;

  inverterData.floatState = e_stateFloat::ABSORPTION_VOLTAGE;

  loadIverterSettings();
  canbus.init();
}


void Inverter::inverterDataSemaphoreTake()
{
  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
}


void Inverter::inverterDataSemaphoreGive()
{
  xSemaphoreGive(mInverterDataMutex);
}


Inverter::inverterData_s * Inverter::getInverterData()
{
  return &inverterData;
}


void Inverter::loadIverterSettings()
{
  uint8_t u8_bmsDatasource;
  uint16_t u16_bmsDatasourceAdd;

  inverterData.noBatteryPackOnline = true;
  u8_bmsDatasource = WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE,0,DT_ID_PARAM_BMS_CAN_DATASOURCE);
  uint8_t u8_lNumberOfSerial2BMSs = WebSettings::getInt(ID_PARAM_SERIAL2_CONNECT_TO_ID,0,DT_ID_PARAM_SERIAL2_CONNECT_TO_ID);

  uint32_t bmsConnectFilter=0;
  /*for(uint8_t i;i<BT_DEVICES_COUNT;i++)
  {
    if(WebSettings::getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV)!=0)
    {
      bmsConnectFilter |= (1<<i);
    }
  }*/
  for(uint8_t i;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    if(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,i,DT_ID_PARAM_SERIAL_CONNECT_DEVICE)!=0)
    {
      bmsConnectFilter |= (1<<i);
    }
    else if(i>=3 && u8_lNumberOfSerial2BMSs-i+2>0) bmsConnectFilter |= (1<<i); //Seplos BMS berücksichtigen
  }

  u16_bmsDatasourceAdd=(((uint32_t)WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE_SS1,0,DT_ID_PARAM_BMS_CAN_DATASOURCE_SS1))&bmsConnectFilter);

  // In den zusätzlichen Datenquellen die Masterquelle entfernen
  if(u8_bmsDatasource>=BT_DEVICES_COUNT) bitClear(u16_bmsDatasourceAdd,u8_bmsDatasource-BT_DEVICES_COUNT);

  inverterData.u8_bmsDatasource = u8_bmsDatasource;
  inverterData.u16_bmsDatasourceAdd = u16_bmsDatasourceAdd;

  BSC_LOGI(TAG,"Load inverter settings(): dataSrc=%i, dataSrcAdd=%i",inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd);
  //BSC_LOGI(TAG,"loadIverterSettings(): dataSrcAdd=%i, u8_mBmsDatasource=%i, bmsConnectFilter=%i, u8_mBmsDatasourceAdd=%i",WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE_SS1,0,DT_ID_PARAM_BMS_CAN_DATASOURCE_SS1),u8_bmsDatasource,bmsConnectFilter, u16_bmsDatasourceAdd);
}


//Ladeleistung auf 0 einstellen
void Inverter::setChargeCurrentToZero(bool val)
{
  alarmSetChargeCurrentToZero = val;
}


//Entladeleistung auf 0 einstellen
void Inverter::setDischargeCurrentToZero(bool val)
{
  alarmSetDischargeCurrentToZero = val;
}


//SOC auf 100 einstellen
void Inverter::setSocToFull(bool val)
{
  alarmSetSocToFull = val;
}


//Wird vom Task aus der main.c zyklisch aufgerufen
void Inverter::cyclicRun()
{
  if(WebSettings::getBool(ID_PARAM_BMS_CAN_ENABLE,0))
  {
    u8_mMqttTxTimer++;
    canbus.readCanMessages(inverterData);

    getInverterValues();
    canbus.sendBmsCanMessages(inverterData);

    sendMqttMsg();
  }
  else inverterData.noBatteryPackOnline = true;
}


void Inverter::getInverterValues()
{
  // Ladespannung
  nsChargeVoltageCtrl::ChargeVoltageCtrl chargeVoltageCtrl = nsChargeVoltageCtrl::ChargeVoltageCtrl();
  chargeVoltageCtrl.calcChargVoltage(*this, inverterData);

  // Ladestrom
  nsChargeCurrentCtrl::ChargeCurrentCtrl chargeCurrentCtl = nsChargeCurrentCtrl::ChargeCurrentCtrl();
  chargeCurrentCtl.calcChargCurrent(*this, inverterData, alarmSetChargeCurrentToZero);

  // Entladestrom
  nsDisChargeCurrentCtrl::DisChargeCurrentCtrl disChargeCurrentCtl = nsDisChargeCurrentCtrl::DisChargeCurrentCtrl();
  disChargeCurrentCtl.calcDisChargCurrent(*this, inverterData, alarmSetDischargeCurrentToZero);

  // SoC
  nsSocCtrl::SocCtrl socCtrl = nsSocCtrl::SocCtrl();
  socCtrl.calcSoc(*this, inverterData, alarmSetSocToFull);

  // Batteriespannung
  nsInverterBattery::InverterBattery inverterBattery = nsInverterBattery::InverterBattery();
  inverterBattery.getBatteryVoltage(*this, inverterData);

  // Batteriestrom
  inverterBattery.getBatteryCurrent(*this, inverterData);
}


void Inverter::sendMqttMsg()
{
  if(u8_mMqttTxTimer == 15)
  {
    u8_mMqttTxTimer=0;

    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_INVERTER_CHARGE_VOLTAGE, -1, (float)(inverterData.inverterChargeVoltage/10.0f));
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_CURRENT_SOLL, -1, inverterData.inverterChargeCurrent/10);
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_DISCHARGE_CURRENT_SOLL, -1, inverterData.inverterDischargeCurrent/10);
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_PERCENT, -1, inverterData.inverterSoc);

    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, (float)(inverterData.batteryVoltage/100.0f));
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TOTAL_CURRENT, -1, (float)(inverterData.batteryCurrent/10.0f));


    nsInverterBattery::InverterBattery inverterBattery = nsInverterBattery::InverterBattery();
    int16_t i16_lBattTemp = inverterBattery.getBatteryTemp(inverterData);
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TEMPERATURE, -1, (float)(i16_lBattTemp));
  }
}
