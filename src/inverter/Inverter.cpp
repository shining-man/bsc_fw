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

#include "WebSettings.h"
#include "defines.h"
#include "BmsData.h"
#include "Ow.h"
#include "mqtt_t.h"
#include <ESP32TWAISingleton.hpp>
#include "log.h"
#include "AlarmRules.h"

const char *Inverter::TAG = "CAN";



Inverter::Inverter() :
u8_mMqttTxTimer(0),
//u8_mBmsDatasource(0),
//u8_mBmsDatasourceAdd(0),
u8_mSelCanInverter(0),
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

  //u8_mBmsDatasource=0;
  alarmSetChargeCurrentToZero=false;
  alarmSetDischargeCurrentToZero=false;
  alarmSetSocToFull=false;

  inverterData.u16_mSocZellspannungSperrzeitTimer=0;
  inverterData.u8_mSocZellspannungState=nsSocCtrl::SocCtrl::SM_SocZellspgStates::STATE_MINCELLSPG_SOC_WAIT_OF_MIN;

  loadIverterSettings();

  constexpr bool CAN_ENABLE_ALERTS {true};
  constexpr std::size_t CAN_RX_QUEUE_LENGTH {10};
  constexpr std::size_t CAN_TX_QUEUE_LENGTH {10};
  const can::Baudrate baudrate = (u8_mSelCanInverter==ID_CAN_DEVICE_VICTRON_250K) ? can::Baudrate::BAUD_250KBPS :
                                                                                    can::Baudrate::BAUD_500KBPS;
  const esp_err_t err = CAN.begin(GPIO_NUM_5, // Rx pin
                                  GPIO_NUM_4, // Tx pin
                                  baudrate,
                                  CAN_ENABLE_ALERTS,
                                  CAN_RX_QUEUE_LENGTH,
                                  CAN_TX_QUEUE_LENGTH);
  BSC_LOGI(TAG, "%s", CAN.getErrorText(err).c_str());
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
  u8_mSelCanInverter = WebSettings::getInt(ID_PARAM_SS_CAN,0,DT_ID_PARAM_SS_CAN);
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

  BSC_LOGI(TAG,"loadIverterSettings(): dataSrcAdd=%i, u8_mBmsDatasource=%i, bmsConnectFilter=%i, u8_mBmsDatasourceAdd=%i",WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE_SS1,0,DT_ID_PARAM_BMS_CAN_DATASOURCE_SS1),u8_bmsDatasource,bmsConnectFilter, u16_bmsDatasourceAdd);
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
    readCanMessages();
    sendBmsCanMessages();
    if(u8_mMqttTxTimer>=15)u8_mMqttTxTimer=0;
  }
  else inverterData.noBatteryPackOnline = true;
}


void Inverter::readCanMessages()
{
  twai_message_t canMessage;
  twai_status_info_t canStatus;

  canStatus = CAN.getStatus();

  // JK-BMS CAN
  bool isJkCanBms=false;
  uint8_t u8_jkCanBms;
  for(uint8_t i=0; i<3; i++)
  {
    if(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,i,DT_ID_PARAM_SERIAL_CONNECT_DEVICE)==ID_SERIAL_DEVICE_JKBMS_CAN)
    {
      isJkCanBms=true;
      u8_jkCanBms=BMSDATA_FIRST_DEV_SERIAL+i;
      //BSC_LOGI(TAG,"JK: dev=%i",u8_jkCanBms);

      setBmsLastDataMillis(u8_jkCanBms,millis()); //Nur zum Test
      break;
    }
  }

  for(uint8_t i=0;i<10;i++)
  {
    canStatus = CAN.getStatus();
    if(canStatus.msgs_to_rx==0) break;
    CAN.read(&canMessage);

    #ifdef CAN_DEBUG
    BSC_LOGI(TAG,"RX ID: %i",canMessage.identifier);
    #endif

    /*uint16_t canId11bit=(canMessage.identifier&0x7FF);
    BSC_LOGI(TAG,"RX Extd=%i, ID=%i, 11bit=%i, len=%i",canMessage.extd,canMessage.identifier,canId11bit,canMessage.data_length_code);
    if(canMessage.data_length_code>0)
    {
      String data="";
      for(uint8_t i=0; i<canMessage.data_length_code;i++)
      {
        data+=canMessage.data[i];
        data+=" ";
      }
      BSC_LOGI(TAG,"RX data=%s",data.c_str());
    }*/

    if(isJkCanBms)
    {
      if(canMessage.identifier==0x02F4) //756; Battery status informatio
      {
        int16_t can_batVolt=0;
        int16_t can_batCurr=0;
        uint8_t can_soc=0;

        can_batVolt = ((int16_t)canMessage.data[1]<<8 | canMessage.data[0])*10; //Strom mit 10 multiplizieren; für BmsData

        can_batCurr = ((int16_t)canMessage.data[3]<<8 | canMessage.data[2])-4000;
        can_batCurr*=10; //Strom mit 10 multiplizieren; für BmsData

        can_soc = canMessage.data[4];

        setBmsTotalVoltage_int(u8_jkCanBms,can_batVolt);
        setBmsTotalCurrent_int(u8_jkCanBms,can_batCurr);
        setBmsChargePercentage(u8_jkCanBms,can_soc);
        setBmsLastDataMillis(u8_jkCanBms,millis());

        //BSC_LOGI(TAG,"JK: volt=%i, curr=%i, soc=%i",can_batVolt,can_batCurr,can_soc);
      }
      else if(canMessage.identifier==0x04F4) //1268; Cell voltage
      {
        setBmsMaxCellVoltage(u8_jkCanBms,((uint16_t)canMessage.data[1]<<8 | canMessage.data[0]));
        setBmsMaxVoltageCellNumber(u8_jkCanBms,canMessage.data[2]);
        setBmsMinCellVoltage(u8_jkCanBms,((uint16_t)canMessage.data[4]<<8 | canMessage.data[3]));
        setBmsMinVoltageCellNumber(u8_jkCanBms,canMessage.data[5]);
        setBmsLastDataMillis(u8_jkCanBms,millis());
      }
      else if(canMessage.identifier==0x05F4) //1524; Cell temperature
      {
        setBmsLastDataMillis(u8_jkCanBms,millis());
      }
      else if(canMessage.identifier==0x07F4) //2036; Warning message
      {
        setBmsLastDataMillis(u8_jkCanBms,millis());
      }
    }

  }
}



void Inverter::getInverterValues()
{

   /*******************************
     * Ladespannung
     *******************************/
    nsChargeVoltageCtrl::ChargeVoltageCtrl chargeVoltageCtrl = nsChargeVoltageCtrl::ChargeVoltageCtrl();

    //Soll-Ladespannung in die Ausgangs-Msg. schreiben
    chargeVoltageCtrl.calcChargVoltage(*this, inverterData)*10;

    if(u8_mMqttTxTimer==15)
    {
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_INVERTER_CHARGE_VOLTAGE, -1, (float)(msgData.chargevoltagelimit/10.0));
    }

    /*******************************
     * Ladestrom
     *******************************/
    nsChargeCurrentCtrl::ChargeCurrentCtrl chargeCurrentCtl = nsChargeCurrentCtrl::ChargeCurrentCtrl();

    //Soll-Ladestrom in die Ausgangs-Msg. schreiben
    chargeCurrentCtl.calcChargCurrent(*this, inverterData, alarmSetChargeCurrentToZero)*10;

    // Wert per mqqt senden
    if(u8_mMqttTxTimer==15)
    {
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_CURRENT_SOLL, -1, inverterData.inverterChargeCurrent);
    }

    /*******************************
     * Entladestrom
     *******************************/
    nsDisChargeCurrentCtrl::DisChargeCurrentCtrl disChargeCurrentCtl = nsDisChargeCurrentCtrl::DisChargeCurrentCtrl();

    // Soll-Entladestrom in die Ausgangs-Msg. schreiben
    chargeCurrentCtl.calcChargCurrent(*this, inverterData, alarmSetDischargeCurrentToZero)*10;

    // Wert per mqqt senden
    if(u8_mMqttTxTimer==15)
    {
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_DISCHARGE_CURRENT_SOLL, -1, inverterData.inverterDischargeCurrent);
    }



}










void Inverter::sendCanMsg(uint32_t identifier, uint8_t *buffer, uint8_t length)
{
  esp_err_t err = CAN.write(can::FrameType::STD_FRAME,identifier,length,buffer);
  if(err!=ESP_OK) BSC_LOGI(TAG, "%s", CAN.getErrorText(err).c_str());

  #ifdef CAN_DEBUG_STATUS
  twai_status_info_t canStatus = CAN.getStatus();

  if(err!=ESP_OK || canStatus.state!=1 || canStatus.tx_error_counter!=0 || canStatus.tx_failed_count!=0 || canStatus.arb_lost_count!=0 || canStatus.bus_error_count!=0)
  {
    BSC_LOGE(TAG, "state=%i, msgs_to_tx=%i, msgs_to_rx=%i, tx_error=%i, rx_error=%i, tx_failed=%i, rx_missed=%i, rx_overrun=%i, arb_lost=%i, bus_error=%i", \
    canStatus.state, canStatus.msgs_to_tx, canStatus.msgs_to_rx, canStatus.tx_error_counter, canStatus.rx_error_counter, \
    canStatus.tx_failed_count, canStatus.rx_missed_count, canStatus.rx_overrun_count, canStatus.arb_lost_count, canStatus.bus_error_count);
  }

  /*
    twai_state_t state;             //< Current state of TWAI controller (Stopped/Running/Bus-Off/Recovery)
    uint32_t msgs_to_tx;            //< Number of messages queued for transmission or awaiting transmission completion
    uint32_t msgs_to_rx;            //< Number of messages in RX queue waiting to be read
    uint32_t tx_error_counter;      //< Current value of Transmit Error Counter
    uint32_t rx_error_counter;      //< Current value of Receive Error Counter
    uint32_t tx_failed_count;       //< Number of messages that failed transmissions
    uint32_t rx_missed_count;       //< Number of messages that were lost due to a full RX queue (or errata workaround if enabled)
    uint32_t rx_overrun_count;      //< Number of messages that were lost due to a RX FIFO overrun
    uint32_t arb_lost_count;        //< Number of instances arbitration was lost
    uint32_t bus_error_count;       //< Number of instances a bus error has occurred
  */
  BSC_LOGI(TAG,"Alert=%i", CAN.getAlert());
  #endif

  vTaskDelay(pdMS_TO_TICKS(5));
}

void Inverter::sendBmsCanMessages()
{
  switch (u8_mSelCanInverter)
  {
    case ID_CAN_DEVICE_DEYE:
    case ID_CAN_DEVICE_SOLISRHI:
      sendCanMsg_ChgVoltCur_DisChgCur_351();
      sendCanMsg_soc_soh_355();
      sendCanMsg_356();
      vTaskDelay(pdMS_TO_TICKS(50));
      sendCanMsg_hostname_35e_370_371();
      sendCanMsg_359(); //Alarms
      break;

    case ID_CAN_DEVICE_VICTRON:
    case ID_CAN_DEVICE_VICTRON_250K:
      // CAN-IDs for core functionality: 0x351, 0x355, 0x356 and 0x35A.
      sendCanMsg_ChgVoltCur_DisChgCur_351();
      sendCanMsg_hostname_35e_370_371();
      sendCanMsg_35a(); //Alarms

      sendCanMsg_372();
      sendCanMsg_35f();

      sendCanMsg_soc_soh_355();
      sendCanMsg_356();
      sendCanMsg_373_376_377();

      //Send extended data
      if(WebSettings::getBool(ID_PARAM_BMS_CAN_EXTENDED_DATA_ENABLE,0)==true)
      {
        sendCanMsgTemp();
        sendCanMsgBmsData();
      }
      //374, 359
      break;

    default:
      break;
  }

  //Solis: ID:305 Heartbeat Inverter
}




// Transmit hostname
void Inverter::sendCanMsg_hostname_35e_370_371()
{
  char hostname_general[16] = {'B','S','C',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
  char hostname_pylon[16] = {'P','Y','L','O','N',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

    switch (u8_mSelCanInverter)
  {
    case ID_CAN_DEVICE_DEYE:
    case ID_CAN_DEVICE_SOLISRHI:
      sendCanMsg(0x35e, (uint8_t *)&hostname_pylon, 6);
      break;
    case ID_CAN_DEVICE_VICTRON:
      sendCanMsg(0x370, (uint8_t *)&hostname_general, 8);
      sendCanMsg(0x371, (uint8_t *)&hostname_general[8], 8);
      sendCanMsg(0x35e, (uint8_t *)&hostname_general, 6);
      break;
  }
}

/*
 * Data 0 + 1:
 * CVL: Battery Charge Voltage (data type : 16bit unsigned int, byte order : little endian, scale factor : 0.1, unit : V)
 * Data 2 + 3:
 * CCL: DC Charge Current Limitation (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : A)
 * Data 4 + 5:
 * DCL: DC Discharge Current Limitation (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : A)
*/
void Inverter::sendCanMsg_ChgVoltCur_DisChgCur_351()
{
  data351 msgData;
  uint8_t errors = 0;

  /* Hier evtl. weitere Fehler feststellen
   * Aktuell nicht notwendig, da die Fehler in den Unterfunktionen festgestellt werden */
  if (errors!=0) //wenn Fehler
  {
    msgData.chargevoltagelimit  = (uint16_t)(WebSettings::getFloat(ID_PARAM_BMS_MAX_CHARGE_SPG,0)*10.0);
    msgData.maxchargecurrent    = 0;
    msgData.maxDischargeCurrent = 0;
    msgData.dischargevoltage    = 0; //not use
  }
  else
  {
    msgData.dischargevoltage    = 0; //not use

    /*******************************
     * Ladespannung
     *******************************/
    nsChargeVoltageCtrl::ChargeVoltageCtrl chargeVoltageCtrl = nsChargeVoltageCtrl::ChargeVoltageCtrl();

    //Soll-Ladespannung in die Ausgangs-Msg. schreiben
    msgData.chargevoltagelimit = chargeVoltageCtrl.calcChargVoltage(*this, inverterData)*10;

    if(u8_mMqttTxTimer==15)
    {
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_INVERTER_CHARGE_VOLTAGE, -1, (float)(msgData.chargevoltagelimit/10.0));
    }

    /*******************************
     * Ladestrom
     *******************************/
    nsChargeCurrentCtrl::ChargeCurrentCtrl chargeCurrentCtl = nsChargeCurrentCtrl::ChargeCurrentCtrl();

    //Soll-Ladestrom in die Ausgangs-Msg. schreiben
    msgData.maxchargecurrent = chargeCurrentCtl.calcChargCurrent(*this, inverterData, alarmSetChargeCurrentToZero)*10;

    // Wert per mqqt senden
    if(u8_mMqttTxTimer==15)
    {
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_CURRENT_SOLL, -1, inverterData.inverterChargeCurrent);
    }

    /*******************************
     * Entladestrom
     *******************************/
    nsDisChargeCurrentCtrl::DisChargeCurrentCtrl disChargeCurrentCtl = nsDisChargeCurrentCtrl::DisChargeCurrentCtrl();

    // Soll-Entladestrom in die Ausgangs-Msg. schreiben
    msgData.maxDischargeCurrent = chargeCurrentCtl.calcChargCurrent(*this, inverterData, alarmSetDischargeCurrentToZero)*10;

    // Wert per mqqt senden
    if(u8_mMqttTxTimer==15)
    {
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_DISCHARGE_CURRENT_SOLL, -1, inverterData.inverterDischargeCurrent);
    }
  }

  sendCanMsg(0x351, (uint8_t *)&msgData, sizeof(msgData));
}


/* SOC
 * Data 0 + 1:
 * SOC Value (data type : 16bit unsigned int, byte order : little endian, scale factor : 1, unit : %)
 * Data 2 + 3:
 * SOH Value (data type : 16bit unsigned int, byte order : little endian, scale factor : 1, unit : %)
 */
void Inverter::sendCanMsg_soc_soh_355()
{
  data355 msgData;

  nsSocCtrl::SocCtrl socCtrl = nsSocCtrl::SocCtrl();

  // Soll-Entladestrom in die Ausgangs-Msg. schreiben
  msgData.soc = socCtrl.calcSoc(*this, inverterData, alarmSetSocToFull);

  // Wert per mqqt senden
  if(u8_mMqttTxTimer==15)
  {
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_PERCENT, -1, msgData.soc);
  }

  msgData.soh = 100; // SOH, uint16 1 %
  sendCanMsg(0x355, (uint8_t *)&msgData, sizeof(data355));
}


/* Battery voltage
 * Data 0 + 1:
 * Battery Voltage (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.01, unit : V)
 * Data 2 + 3:
 * Battery Current (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : A)
 * Data 4 + 5:
 * Battery Temperature (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : degC)
 */
void Inverter::sendCanMsg_356()
{
  data356 msgData;
  bool isOneBatteryPackOnline=false;

  //Batteriespannung
  msgData.voltage = 0;

  if((millis()-getBmsLastDataMillis(inverterData.u8_bmsDatasource))<CAN_BMS_COMMUNICATION_TIMEOUT)
  {
    msgData.voltage = (int16_t)(getBmsTotalVoltage(inverterData.u8_bmsDatasource)*100);
  }
  else //Wenn Masterquelle offline, dann nächstes BMS nehmen das online ist
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      //Wenn BMS ausgewählt und die letzten 5000ms Daten kamen
      if(((inverterData.u16_bmsDatasourceAdd>>i)&0x01) && ((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<CAN_BMS_COMMUNICATION_TIMEOUT))
      {
          msgData.voltage = (int16_t)(getBmsTotalVoltage(BT_DEVICES_COUNT+i)*100);
          break;
      }
    }
  }

  //Batteriestrom
  msgData.current = (int16_t)(getBmsTotalCurrent(inverterData.u8_bmsDatasource)*10);
  if((millis()-getBmsLastDataMillis(inverterData.u8_bmsDatasource))<CAN_BMS_COMMUNICATION_TIMEOUT) isOneBatteryPackOnline=true;
  #ifdef CAN_DEBUG
  BSC_LOGI(TAG,"Battery current: u8_mBmsDatasource=%i, cur=%i, u8_mBmsDatasourceAdd=%i",u8_mBmsDatasource, msgData.current, u8_mBmsDatasourceAdd);
  #endif

  //Wenn zusätzliche Datenquellen angegeben sind:
  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    #ifdef CAN_DEBUG
    long lTime = getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i);
    #endif
    //Wenn BMS ausgewählt und die letzten 5000ms Daten kamen
    if(((inverterData.u16_bmsDatasourceAdd>>i)&0x01) && ((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<CAN_BMS_COMMUNICATION_TIMEOUT))
    {
      isOneBatteryPackOnline=true;
      msgData.current += (int16_t)(getBmsTotalCurrent(BT_DEVICES_COUNT+i)*10);
      #ifdef CAN_DEBUG
      BSC_LOGI(TAG,"Battery current (T): dev=%i, time=%i, cur=%i",i,millis()-lTime, msgData.current);
      #endif
    }
    #ifdef CAN_DEBUG
    else
    {
      BSC_LOGI(TAG,"Battery current (F): dev=%i, time1=%i, time2=%i, cur=%i",i,millis()-lTime,lTime,msgData.current);
    }
    #endif
  }

  //Temperatur
  uint8_t u8_lBmsTempQuelle=WebSettings::getInt(ID_PARAM_INVERTER_BATT_TEMP_QUELLE,0,DT_ID_PARAM_INVERTER_BATT_TEMP_QUELLE);
  uint8_t u8_lBmsTempSensorNr=WebSettings::getInt(ID_PARAM_INVERTER_BATT_TEMP_SENSOR,0,DT_ID_PARAM_INVERTER_BATT_TEMP_SENSOR);
  if(u8_lBmsTempQuelle==1)
  {
    if(u8_lBmsTempSensorNr<3)
    {
      msgData.temperature = (int16_t)(getBmsTempature(inverterData.u8_bmsDatasource,u8_lBmsTempSensorNr)*10);
    }
    else
    {
      msgData.temperature = (int16_t)(getBmsTempature(inverterData.u8_bmsDatasource,0)*10); //Im Fehlerfall immer Sensor 0 des BMS nehmen
    }
  }
  else if(u8_lBmsTempQuelle==2)
  {
    if(u8_lBmsTempSensorNr<MAX_ANZAHL_OW_SENSOREN)
    {
      msgData.temperature = (int16_t)(owGetTemp(u8_lBmsTempSensorNr)*10);
    }
    else
    {
      msgData.temperature = (int16_t)(getBmsTempature(inverterData.u8_bmsDatasource,0)*10); //Im Fehlerfall immer Sensor 0 des BMS nehmen
    }
  }
  else
  {
    msgData.temperature = (int16_t)(getBmsTempature(inverterData.u8_bmsDatasource,0)*10);  //Im Fehlerfall immer Sensor 0 des BMS nehmen
  }


  #ifdef CAN_DEBUG
  BSC_LOGD(TAG, "CAN: current=%i temperature=%i voltage=%i", msgData.current, msgData.temperature, msgData.voltage);
  #endif

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  if(isOneBatteryPackOnline) inverterData.noBatteryPackOnline=false;
  else inverterData.noBatteryPackOnline=true;
  inverterData.inverterVoltage = msgData.voltage;
  inverterData.inverterCurrent = msgData.current;
  xSemaphoreGive(mInverterDataMutex);

  if(u8_mMqttTxTimer==15)
  {
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, (float)(msgData.voltage/100));
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TOTAL_CURRENT, -1, (float)(msgData.current/10));
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TEMPERATURE, -1, (float)(msgData.temperature/10));
  }

  sendCanMsg(0x356, (uint8_t *)&msgData, sizeof(data356));
}


// Send alarm details
void Inverter::sendCanMsg_359()
{
  data35a msgData;
  uint8_t u8_lValue=0;

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x //bit0  single cell overvoltage protection
  #define BMS_ERR_STATUS_CELL_UVP          2  x //bit1  single cell undervoltage protection
  #define BMS_ERR_STATUS_BATTERY_OVP       4  x //bit2  whole pack overvoltage protection
  #define BMS_ERR_STATUS_BATTERY_UVP       8  x //bit3  Whole pack undervoltage protection
  #define BMS_ERR_STATUS_CHG_OTP          16  x //bit4  charging over temperature protection
  #define BMS_ERR_STATUS_CHG_UTP          32  x //bit5  charging low temperature protection
  #define BMS_ERR_STATUS_DSG_OTP          64  x //bit6  Discharge over temperature protection
  #define BMS_ERR_STATUS_DSG_UTP         128  x //bit7  discharge low temperature protection
  #define BMS_ERR_STATUS_CHG_OCP         256  x //bit8  charging overcurrent protection
  #define BMS_ERR_STATUS_DSG_OCP         512  x //bit9  Discharge overcurrent protection
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  x //bit10 short circuit protection
  #define BMS_ERR_STATUS_AFE_ERROR      2048  x //bit11 Front-end detection IC error
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  x //bit12 software lock MOS
  #define BMS_ERR_STATUS_RESERVED1      8192  - //bit13 Reserved
  #define BMS_ERR_STATUS_RESERVED2     16384  - //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  - //bit15 Reserved */


  /*
  Pylontech V1.2

  Data 0 (Alarm)
  0: -
  1: Battery high voltage
  2: Battery low voltage alarm
  3: Battery high temp
  4: Battery low temp
  5: -
  6: -
  7: Discharge over current

  Data 1 (Alarm)
  0: Charge over current
  1: -
  2: -
  3: System error
  4: -
  5: -
  6: -
  7: -

  Data 2 (Warning)
  0: -
  1: Battery high voltage
  2: Battery low voltage alarm
  3: Battery high temp
  4: Battery low temp
  5: -
  6: -
  7: Discharg high current

  Data 3 (Warning)
  0: Charge high current
  1: -
  2: -
  3: System error
  4: -
  5: -
  6: -
  7: -

  Data 4: Pack number (data type : 8bit unsigned char)
  Data 5: 0x50
  Data 6: 0x4E
  Data 6: -
  */

  uint32_t u32_bmsErrors = getBmsErrors(inverterData.u8_bmsDatasource);

  if(inverterData.u16_bmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((inverterData.u16_bmsDatasourceAdd>>i)&0x01)
      {
        u32_bmsErrors |= getBmsErrors(BMSDATA_FIRST_DEV_SERIAL+i);
      }
    }
  }

  msgData.u8_b0=0;
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_OVP)==BMS_ERR_STATUS_CELL_OVP) msgData.u8_b0 |= B00000010;    //1: Battery high voltage
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_UVP)==BMS_ERR_STATUS_CELL_UVP) msgData.u8_b0 |= B00000100;    //2: Battery low voltage alarm
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_OVP)==BMS_ERR_STATUS_BATTERY_OVP) msgData.u8_b0 |= B00000010; //1: Battery high voltage
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_UVP)==BMS_ERR_STATUS_BATTERY_UVP) msgData.u8_b0 |= B00000100; //2: Battery low voltage alarm

  if((u32_bmsErrors&BMS_ERR_STATUS_CHG_OTP)==BMS_ERR_STATUS_CHG_OTP) msgData.u8_b0 |= B00001000;      //3: Battery high temp
  if((u32_bmsErrors&BMS_ERR_STATUS_CHG_UTP)==BMS_ERR_STATUS_CHG_UTP) msgData.u8_b0 |= B00010000;      //4: Battery low temp
  if((u32_bmsErrors&BMS_ERR_STATUS_DSG_OTP)==BMS_ERR_STATUS_DSG_OTP) msgData.u8_b0 |= B00001000;      //3: Battery high temp
  if((u32_bmsErrors&BMS_ERR_STATUS_DSG_UTP)==BMS_ERR_STATUS_DSG_UTP) msgData.u8_b0 |= B00010000;      //4: Battery low temp

  if((u32_bmsErrors&BMS_ERR_STATUS_DSG_OCP)==BMS_ERR_STATUS_DSG_OCP) msgData.u8_b0 |= B10000000;      //7: Discharge over current

  msgData.u8_b1=0;
  if((u32_bmsErrors&BMS_ERR_STATUS_CHG_OCP)==BMS_ERR_STATUS_CHG_OCP) msgData.u8_b1 |= B00000001;              //0: Charge high current
  if((u32_bmsErrors&BMS_ERR_STATUS_SHORT_CIRCUIT)==BMS_ERR_STATUS_SHORT_CIRCUIT) msgData.u8_b1 |= B00001000;  //3: System error
  if((u32_bmsErrors&BMS_ERR_STATUS_AFE_ERROR)==BMS_ERR_STATUS_AFE_ERROR) msgData.u8_b1 |= B00001000;          //3: System error
  if((u32_bmsErrors&BMS_ERR_STATUS_SOFT_LOCK)==BMS_ERR_STATUS_SHORT_CIRCUIT) msgData.u8_b1 |= B00001000;      //3: System error


  //Alarme über Trigger einbinden
  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE)) msgData.u8_b0 |= B00000010;
  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE)) msgData.u8_b0 |= B00000100;
  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE)) msgData.u8_b0 |= B00001000;
  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOWTEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_LOWTEMPERATURE)) msgData.u8_b0 |= B00010000;


  msgData.u8_b2=0;
  msgData.u8_b3=0;
  msgData.u8_b4=0x01; //Pack number (data type : 8bit unsigned char)
  msgData.u8_b5=0x50;
  msgData.u8_b6=0x4E;
  msgData.u8_b7=0;

  sendCanMsg(0x359, (uint8_t *)&msgData, sizeof(data35a));
}


// Send alarm details
void Inverter::sendCanMsg_35a()
{
  const uint8_t BB0_ALARM = B00000001;
  const uint8_t BB1_ALARM = B00000100;
  const uint8_t BB2_ALARM = B00010000;
  const uint8_t BB3_ALARM = B01000000;

  const uint8_t BB0_OK = B00000010;
  const uint8_t BB1_OK = B00001000;
  const uint8_t BB2_OK = B00100000;
  const uint8_t BB3_OK = B10000000;

  data35a msgData;
  msgData.u8_b0=0;
  msgData.u8_b1=0;
  msgData.u8_b2=0;
  msgData.u8_b3=0;
  msgData.u8_b4=0;
  msgData.u8_b5=0;
  msgData.u8_b6=0;
  msgData.u8_b7=0;

  uint8_t u8_lValue=0;

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x //bit0  single cell overvoltage protection
  #define BMS_ERR_STATUS_CELL_UVP          2  x //bit1  single cell undervoltage protection
  #define BMS_ERR_STATUS_BATTERY_OVP       4  x //bit2  whole pack overvoltage protection
  #define BMS_ERR_STATUS_BATTERY_UVP       8  x //bit3  Whole pack undervoltage protection
  #define BMS_ERR_STATUS_CHG_OTP          16  x //bit4  charging over temperature protection
  #define BMS_ERR_STATUS_CHG_UTP          32  x //bit5  charging low temperature protection
  #define BMS_ERR_STATUS_DSG_OTP          64  x //bit6  Discharge over temperature protection
  #define BMS_ERR_STATUS_DSG_UTP         128  x //bit7  discharge low temperature protection
  #define BMS_ERR_STATUS_CHG_OCP         256  x //bit8  charging overcurrent protection
  #define BMS_ERR_STATUS_DSG_OCP         512  x //bit9  Discharge overcurrent protection
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  x //bit10 short circuit protection
  #define BMS_ERR_STATUS_AFE_ERROR      2048  x //bit11 Front-end detection IC error
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  x //bit12 software lock MOS
  #define BMS_ERR_STATUS_RESERVED1      8192  - //bit13 Reserved
  #define BMS_ERR_STATUS_RESERVED2     16384  - //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  - //bit15 Reserved */


  //msgData.u8_b0 |= BB0_ALARM; //n.b.
  //msgData.u8_b0 |= BB1_ALARM; //High battery voltage
  //msgData.u8_b0 |= BB2_ALARM; //Low battery voltage
  //msgData.u8_b0 |= BB3_ALARM; //High Temperature

  //msgData.u8_b1 |= BB0_ALARM; //Low Temperature
  //msgData.u8_b1 |= BB1_ALARM; //High charge Temperature
  //msgData.u8_b1 |= BB2_ALARM; //Low charge Temperature
  //msgData.u8_b1 |= BB3_ALARM; //High discharge current

  //msgData.u8_b2 |= BB0_ALARM; //High charge current
  //msgData.u8_b2 |= BB1_ALARM; //n.b.
  //msgData.u8_b2 |= BB2_ALARM; //n.b.
  //msgData.u8_b2 |= BB3_ALARM; //Internal failure

  //msgData.u8_b3 |= BB0_ALARM; // Cell imbalance
  //msgData.u8_b3 |= BB1_ALARM; //n.b.
  //msgData.u8_b3 |= BB2_ALARM; //n.b.
  //msgData.u8_b3 |= BB3_ALARM; //n.b.

  uint32_t u32_bmsErrors = getBmsErrors(inverterData.u8_bmsDatasource);
  //BSC_LOGI(TAG,"u8_mBmsDatasource=%i, u32_bmsErrors=%i",u8_mBmsDatasource,u32_bmsErrors);

  if(inverterData.u16_bmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((inverterData.u16_bmsDatasourceAdd>>i)&0x01)
      {
        u32_bmsErrors |= getBmsErrors(BMSDATA_FIRST_DEV_SERIAL+i);
      }
    }
  }

  // 0 (bit 0+1) n.b.
  msgData.u8_b0 |= BB0_OK;

  // 0 (bit 2+3) Battery high voltage alarm
  msgData.u8_b0 |= (((u32_bmsErrors&BMS_ERR_STATUS_BATTERY_OVP)==BMS_ERR_STATUS_BATTERY_OVP) ||
    ((u32_bmsErrors&BMS_ERR_STATUS_CELL_OVP)==BMS_ERR_STATUS_CELL_OVP))? BB1_ALARM : BB1_OK;

  // 0 (bit 4+5) Battery low voltage alarm
  msgData.u8_b0 |= (((u32_bmsErrors&BMS_ERR_STATUS_BATTERY_UVP)==BMS_ERR_STATUS_BATTERY_UVP) ||
    ((u32_bmsErrors&BMS_ERR_STATUS_CELL_UVP)==BMS_ERR_STATUS_CELL_UVP)) ? BB2_ALARM : BB2_OK;

  // 0 (bit 6+7) Battery high temperature alarm
  msgData.u8_b0 |= ((u32_bmsErrors&BMS_ERR_STATUS_DSG_OTP)==BMS_ERR_STATUS_DSG_OTP) ? BB3_ALARM : BB3_OK;

  // 1 (bit 0+1) Battery low temperature alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_DSG_UTP)==BMS_ERR_STATUS_DSG_UTP) ? BB0_ALARM : BB0_OK;

  // 1 (bit 2+3) Battery high temperature charge alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_CHG_OTP)==BMS_ERR_STATUS_CHG_OTP) ? BB1_ALARM : BB1_OK;

  // 1 (bit 4+5) Battery low temperature charge alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_CHG_UTP)==BMS_ERR_STATUS_CHG_UTP) ? BB2_ALARM : BB2_OK;

  // 1 (bit 6+7) Battery high discharge current alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_DSG_OCP)==BMS_ERR_STATUS_DSG_OCP) ? BB3_ALARM : BB3_OK;

  // 2 (bit 0+1) Battery high charge current alarm
  msgData.u8_b2 |= ((u32_bmsErrors&BMS_ERR_STATUS_CHG_OCP)==BMS_ERR_STATUS_CHG_OCP) ? BB0_ALARM : BB0_OK;

  // 2 (bit 2+3) Contactor Alarm (not implemented)
  msgData.u8_b2 |= BB1_OK;

  // 2 (bit 4+5) Short circuit Alarm (not implemented)
  msgData.u8_b2 |= BB2_OK;

  // 2 (bit 6+7) BMS internal alarm
  msgData.u8_b2 |= (((u32_bmsErrors&BMS_ERR_STATUS_AFE_ERROR)==BMS_ERR_STATUS_AFE_ERROR) ||
    ((u32_bmsErrors&BMS_ERR_STATUS_SHORT_CIRCUIT)==BMS_ERR_STATUS_SHORT_CIRCUIT) ||
    ((u32_bmsErrors&BMS_ERR_STATUS_SOFT_LOCK)==BMS_ERR_STATUS_SOFT_LOCK)) ? BB3_ALARM : BB3_OK;

  // 3 (bit 0+1) Cell imbalance alarm
  // 3 (bit 2+3) n.b.
  // 3 (bit 4+5) n.b.
  // 3 (bit 6+7) n.b.


  //Warnings
  // 4 (bit 0+1) n.b.
  // 4 (bit 2+3) Battery high voltage warning
  // 4 (bit 4+5) Battery low voltage warning
  // 4 (bit 6+7) Battery high temperature warning

  // 5 (bit 0+1) Battery low temperature warning
  // 5 (bit 2+3) Battery high temperature charge warning
  // 5 (bit 4+5) Battery low temperature charge warning
  // 5 (bit 6+7) Battery high current warning

  // 6 (bit 0+1) Battery high charge current warning
  // 6 (bit 2+3) Contactor warning (not implemented)
  // 6 (bit 4+5) Short circuit warning (not implemented)
  // 6 (bit 6+7) BMS internal warning

  // 7 (bit 0+1) Cell imbalance warning
  // 7 (bit 2+3) System status (online/offline)
  // 7 (bit 4+5) n.b.
  // 7 (bit 6+7) n.b.


  //Alarme über Trigger einbinden
  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE))
  {
    msgData.u8_b0 &= ~(BB1_OK);
    msgData.u8_b0 |= BB1_ALARM;
  }

  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE))
  {
    msgData.u8_b0 &= ~(BB2_OK);
    msgData.u8_b0 |= BB2_ALARM;
  }

  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE))
  {
    msgData.u8_b0 &= ~(BB3_OK);
    msgData.u8_b0 |= BB3_ALARM;
  }

  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOWTEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_LOWTEMPERATURE))
  {
    msgData.u8_b1 &= ~(BB0_OK);
    msgData.u8_b1 |= BB0_ALARM;
  }


  //BSC_LOGI(TAG,"0x35a=%i,%i,%i,%i,%i,%i,%i,%i",msgData.u8_b0,msgData.u8_b1,msgData.u8_b2,msgData.u8_b3,msgData.u8_b4,msgData.u8_b5,msgData.u8_b6,msgData.u8_b7);
  sendCanMsg(0x35a, (uint8_t *)&msgData, sizeof(data35a));
}


void Inverter::sendCanMsg_35f()
{
  struct data35f
  {
    uint16_t BatteryModel;
    uint16_t Firmwareversion;
    uint16_t Onlinecapacity;
  };

  data35f msgData;

  msgData.BatteryModel = 0;
  msgData.Firmwareversion = 3;
  msgData.Onlinecapacity = 0;

  sendCanMsg(0x35f, (uint8_t *)&msgData, sizeof(data35f));
}


void Inverter::sendCanMsg_372()
{
  struct data372
  {
    uint16_t numberofmodulesok;
    uint16_t numberofmodulesblockingcharge;
    uint16_t numberofmodulesblockingdischarge;
    // uint16_t numberofmodulesoffline;
  };
  data372 msgData;

  msgData.numberofmodulesok = BmsDataUtils::getNumberOfBatteryModules(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd);
  msgData.numberofmodulesblockingcharge = BmsDataUtils::getNumberOfBatteryModules(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd)-BmsDataUtils::getNumberOfBatteryModulesCharge(u8_bmsDatasource, u16_bmsDatasourceAdd);
  msgData.numberofmodulesblockingdischarge = BmsDataUtils::getNumberOfBatteryModules(u8_bmsDatasource, u16_bmsDatasourceAdd)-BmsDataUtils::getNumberOfBatteryModulesDischarge(u8_bmsDatasource, u16_bmsDatasourceAdd);
  //msgData.numberofmodulesoffline = 0;

  sendCanMsg(0x372, (uint8_t *)&msgData, sizeof(data372));
}


void Inverter::sendCanMsg_373_376_377()
{
  data373 msgData;

  msgData.maxCellVoltage = BmsDataUtils::getMaxCellSpannungFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd);
  msgData.minCellColtage = BmsDataUtils::getMinCellSpannungFromBms(inverterData.u8_bmsDatasource, inverterData.u16_bmsDatasourceAdd);

  if(getBmsTempature(inverterData.u8_bmsDatasource,1)>getBmsTempature(inverterData.u8_bmsDatasource,2))
  {
    msgData.minCellTemp = 273 + getBmsTempature(inverterData.u8_bmsDatasource,2);
    msgData.maxCellTemp = 273 + getBmsTempature(inverterData.u8_bmsDatasource,1);
  }
  else
  {
    msgData.minCellTemp = 273 + getBmsTempature(inverterData.u8_bmsDatasource,1);
    msgData.maxCellTemp = 273 + getBmsTempature(inverterData.u8_bmsDatasource,2);
  }

  sendCanMsg(0x373, (uint8_t *)&msgData, sizeof(data373));

  //sendCanMsg(0x376, (uint8_t *)&msgData, 8); //lowestExternalTemp
  //sendCanMsg(0x377, (uint8_t *)&msgData, 8); //highestExternalTemp
}



void Inverter::sendCanMsgTemp()
{
  uint32_t u16_lCanId = 0x380;

  struct dataTemp
  {
    uint16_t temperature[4];
  };
  dataTemp msgData;

  for(uint8_t i=0;i<64;i+=4)
  {
    for(uint8_t n=0;n<4;n++) msgData.temperature[n] = (uint16_t)(owGetTemp(i+n)*100);
    sendCanMsg(u16_lCanId, (uint8_t *)&msgData, sizeof(dataTemp));
    u16_lCanId++;
  }
}

void Inverter::sendCanMsgBmsData()
{
  uint32_t u16_lBaseCanId = 0x400;
  uint32_t u16_lCanId=u16_lBaseCanId;
  static uint8_t u8_mCanSendDataBmsNumber=0;

  struct dataCellVoltage
  {
    uint16_t cellVoltage[4];
  };
  dataCellVoltage msgData;

  struct dataBms1
  {
    bool stateFETCharge;
    bool stateFETDischarge;
    bool stateBalancingActive;
  };
  dataBms1 msgDataBms1;

  struct dataBms2
  {
    int16_t value1;
  };
  dataBms2 msgDataBms2;


  u16_lCanId=u16_lBaseCanId+(0x32*u8_mCanSendDataBmsNumber);

  //Cellvoltage
  for(uint8_t n=0;n<16;n+=4)
  {
    for(uint8_t x=0;x<4;x++) msgData.cellVoltage[x] = getBmsCellVoltage(u8_mCanSendDataBmsNumber,n+x);
    sendCanMsg(u16_lCanId, (uint8_t *)&msgData, sizeof(dataCellVoltage));
    u16_lCanId++;
  }

  msgDataBms1.stateFETCharge = getBmsStateFETsCharge(u8_mCanSendDataBmsNumber);
  msgDataBms1.stateFETDischarge = getBmsStateFETsDischarge(u8_mCanSendDataBmsNumber);
  msgDataBms1.stateBalancingActive = getBmsIsBalancingActive(u8_mCanSendDataBmsNumber);
  sendCanMsg(u16_lCanId, (uint8_t *)&msgDataBms1, sizeof(dataBms1));
  u16_lCanId++;

  msgDataBms2.value1 = (int16_t)(getBmsBalancingCurrent(u8_mCanSendDataBmsNumber)*100);
  sendCanMsg(u16_lCanId, (uint8_t *)&msgDataBms2, sizeof(dataBms2));
  u16_lCanId++;

  u8_mCanSendDataBmsNumber++;
  if(u8_mCanSendDataBmsNumber==BMSDATA_NUMBER_ALLDEVICES)u8_mCanSendDataBmsNumber=0;
}

