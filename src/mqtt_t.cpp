// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "mqtt_t.h"

#include <deque>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include "WebSettings.h"
#include "defines.h"
#include "BmsData.h"
#include "Ow.h"
#include "log.h"
#include "BleHandler.h"
#include "AlarmRules.h"


static const char* TAG = "MQTT";

static SemaphoreHandle_t mMqttMutex = NULL;

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
IPAddress    mqttIpAdr;

static String str_mMqttDeviceName;
static String str_mMqttTopicName;

uint32_t u32_mMqttPublishLoopTimmer=0;

uint8_t  mMqttEnable = 0;
uint8_t  u8_mWaitConnectCounter;

uint32_t sendeTimerBmsMsg;
uint32_t sendeDelayTimer10ms;
uint32_t sendeDelayTimer500ms;
uint8_t  sendBmsData_mqtt_sendeCounter=0;
bool     bmsDataSendFinsh=true;
uint8_t  sendOwTemperatur_mqtt_sendeCounter=0;
bool     owDataSendFinsh=true;

//bool     bo_mSendPrioMessages=false;

struct mqttEntry_s {
  int8_t t1;
  int8_t t2;
  int8_t t3;
  int8_t t4;
  std::string value;
};
std::deque<mqttEntry_s> txBuffer;

enum enum_smMqttEnableState {MQTT_ENABLE_STATE_OFF, MQTT_ENABLE_STATE_EN, MQTT_ENABLE_STATE_READY};
enum enum_smMqttConnectState {SM_MQTT_WAIT_CONNECTION, SM_MQTT_CONNECTED, SM_MQTT_DISCONNECTED};
enum_smMqttConnectState smMqttConnectState;
enum_smMqttConnectState smMqttConnectStateOld;

bool mqttPublishLoopFromTxBuffer();
void mqttDataToTxBuffer();
void mqttPublishBmsData(uint8_t);
void mqttPublishOwTemperatur(uint8_t);
void mqttPublishTrigger();
void mqttCallback(char* topic, uint8_t* payload, unsigned int length);


void initMqtt()
{
  mMqttMutex = xSemaphoreCreateMutex();

  smMqttConnectState=SM_MQTT_DISCONNECTED;
  smMqttConnectStateOld=SM_MQTT_DISCONNECTED;
  u8_mWaitConnectCounter=0;

  if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0)) mMqttEnable = MQTT_ENABLE_STATE_EN;
  if(mMqttEnable == MQTT_ENABLE_STATE_OFF) return;

  if(!WebSettings::getString(ID_PARAM_MQTT_SERVER_IP,0).equals(""))
  {
    wifiClient.setTimeout(1);
    wifiClient.setClientSelectTimeout(900000, 1);

    mqttClient.setKeepAlive(30);
    mqttClient.setSocketTimeout(1);

    mqttIpAdr.fromString(WebSettings::getString(ID_PARAM_MQTT_SERVER_IP,0).c_str());
    mqttClient.setServer(mqttIpAdr, (uint16_t)WebSettings::getInt(ID_PARAM_MQTT_SERVER_PORT,0,DT_ID_PARAM_MQTT_SERVER_PORT));
    BSC_LOGI(TAG,"MQTT: ip=%s, port=%i", mqttIpAdr.toString().c_str(), WebSettings::getInt(ID_PARAM_MQTT_SERVER_PORT,0,DT_ID_PARAM_MQTT_SERVER_PORT));
    mqttClient.setCallback(mqttCallback);
  }
}

#ifdef MQTT_DEBUG
bool bo_mBTisScanRuningOld=false;
#endif
bool mqttLoop()
{
  //Is MQTT Enabled?
  if(mMqttEnable == MQTT_ENABLE_STATE_OFF)
  {
    /*#ifdef MQTT_DEBUG
    BSC_LOGD(TAG,"mqttLoop(): mMqttEnable=0, ret=1");
    #endif*/
    return true;
  }
  else if(mMqttEnable == MQTT_ENABLE_STATE_EN)
  {
    if(haveAllBmsFirstData())
    {
      // Sendebuffer leeren da schon falsche Werte enthalten sein könnten
      xSemaphoreTake(mMqttMutex, portMAX_DELAY);
      txBuffer.clear();
      xSemaphoreGive(mMqttMutex);
      mMqttEnable = MQTT_ENABLE_STATE_READY;
      BSC_LOGI(TAG,"Daten von allen BMS's vorhanden. Das Senden kann starten.");
    }
    else return true;
  }

  bool ret=false;

  //Running Bluetooth scan?
  bool bo_lBTisScanRuning=BleHandler::isNotAllDeviceConnectedOrScanRunning();
  #ifdef MQTT_DEBUG
  if(bo_lBTisScanRuning!=bo_mBTisScanRuningOld)
  {
    BSC_LOGD(TAG,"mqttLoop(): BTscanRuning=%i", bo_lBTisScanRuning);
    bo_mBTisScanRuningOld=bo_lBTisScanRuning;
  }
  #endif
  if(bo_lBTisScanRuning) return true;

  //Mqtt connect SM
  switch(smMqttConnectState)
  {
    case SM_MQTT_WAIT_CONNECTION:
      // Warte auf Verbindung zum Broker
      break;

    case SM_MQTT_CONNECTED:
      if(!mqttClient.connected())
      {
        smMqttConnectState=SM_MQTT_DISCONNECTED;
        #ifdef MQTT_DEBUG
        BSC_LOGD(TAG,"mqttLoop(): SM_MQTT_CONNECTED: !mqttClient.connected(), ret=0");
        #endif
        ret=false;
        break;
      }

      if(BleHandler::isNotAllDeviceConnectedOrScanRunning())
      {
        /*if(bo_mSendPrioMessages)
        {
          mqttClient.loop();
          mqttPublishLoopFromTxBuffer();  //MQTT Messages zyklisch publishen
        }*/
        #ifdef MQTT_DEBUG
        BSC_LOGD(TAG,"mqttLoop(): isNotAllDeviceConnectedOrScanRunning, ret=1");
        #endif
        ret=true;
        break;
      }

      mqttClient.loop();

      //MQTT Messages zyklisch publishen
      mqttPublishLoopFromTxBuffer();

      //Sende Diverse MQTT Daten
      mqttDataToTxBuffer();

      //#ifdef MQTT_DEBUG
      //BSC_LOGD(TAG,"mqttLoop(): SM_MQTT_CONNECTED, ret=1");
      //#endif
      ret=true;
      break;

    case SM_MQTT_DISCONNECTED:
      smMqttConnectState=SM_MQTT_WAIT_CONNECTION;

      #ifdef MQTT_DEBUG
      BSC_LOGD(TAG,"mqttLoop(): SM_MQTT_DISCONNECTED -> SM_MQTT_WAIT_CONNECTION, ret=0");
      #endif
      ret=false;
      break;

    default:
      smMqttConnectState=SM_MQTT_DISCONNECTED;

      #ifdef MQTT_DEBUG
      BSC_LOGD(TAG,"mqttLoop(): default -> SM_MQTT_DISCONNECTED, ret=0");
      #endif
      ret=false;
      break;
  }

  //Log MQTT SM-state (smMqttConnectStateOld only for log)
  if(smMqttConnectState!=smMqttConnectStateOld)
  {
    BSC_LOGD(TAG,"smMqttConnectState=%i",smMqttConnectState);
    smMqttConnectStateOld=smMqttConnectState;
  }

  //#ifdef MQTT_DEBUG
  //BSC_LOGD(TAG,"mqttLoop(): END: ret=%i",ret);
  //#endif
  return ret;
}


bool mqttConnect()
{
  bool ret=false;
  uint8_t bo_lBreak=0;
  if(mMqttEnable == MQTT_ENABLE_STATE_OFF) return true;

  #ifdef MQTT_DEBUG
  BSC_LOGD(TAG,"mqttConnect() u8_mWaitConnectCounter=%i",u8_mWaitConnectCounter);
  #endif

  smMqttConnectState=SM_MQTT_WAIT_CONNECTION;

  //Nur wenn WLAN-Verbindung besteht
  if(WiFi.status() != WL_CONNECTED) bo_lBreak+=1;
  if(BleHandler::isNotAllDeviceConnectedOrScanRunning()) bo_lBreak+=2;

  if(WebSettings::getString(ID_PARAM_MQTT_SERVER_IP,0).equals("")) bo_lBreak+=4;
  if(WebSettings::getInt(ID_PARAM_MQTT_SERVER_PORT,0,DT_ID_PARAM_MQTT_SERVER_PORT)<=0) bo_lBreak+=8;

  if(bo_lBreak!=0)
  {
    #ifdef MQTT_DEBUG
    BSC_LOGD(TAG,"mqttConnect() break (%i)",bo_lBreak);
    #endif
    return false;
  }

  str_mMqttDeviceName = WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0);
  str_mMqttTopicName = WebSettings::getString(ID_PARAM_MQTT_TOPIC_NAME,0);
  String mqttUser = WebSettings::getString(ID_PARAM_MQTT_USERNAME,0);
  String mqttPwd = WebSettings::getString(ID_PARAM_MQTT_PWD,0);

  if(!mqttClient.connected())
  {
    if(u8_mWaitConnectCounter==5)
    {
      #ifdef MQTT_DEBUG
      BSC_LOGD(TAG,"mqttConnect() u8_mWaitConnectCounter==5 => disconnect");
      #endif
      u8_mWaitConnectCounter=0;
      mqttClient.disconnect();
      smMqttConnectState=SM_MQTT_DISCONNECTED;
      return false;
    }

    u8_mWaitConnectCounter++;

    #ifdef MQTT_DEBUG
    BSC_LOGD(TAG,"Connecting to MQTT Broker...");
    #endif
    if(mqttUser.equals("") || mqttPwd.equals("")) //Wenn kein User oder Pwd eingetragen, dann ohne verbinden
    {
      ret=mqttClient.connect(str_mMqttDeviceName.c_str());
    }
    else
    {
      ret=mqttClient.connect(str_mMqttDeviceName.c_str(), mqttUser.c_str(), mqttPwd.c_str());
    }
  }
  else
  {
    u8_mWaitConnectCounter=0;
    smMqttConnectState=SM_MQTT_CONNECTED;
    ret=true;

    // Subscribe
    String str_lSubTopic = WebSettings::getString(ID_PARAM_MQTT_TOPIC_NAME,0);
    str_lSubTopic+="/input/#";
    mqttClient.subscribe(str_lSubTopic.c_str());

    BSC_LOGI(TAG,"MQTT Broker connected");
  }

  return ret;
}


void mqttDisconnect()
{
  smMqttConnectState=SM_MQTT_DISCONNECTED;

  if(mqttClient.connected())
  {
    BSC_LOGI(TAG,"MQTT Broker disconnected");
    mqttClient.disconnect();
  }

  txBuffer.clear();
}


bool mqttConnected()
{
  if(smMqttConnectState==SM_MQTT_CONNECTED) return true;
  else return false;
}

uint16_t getTxBufferSize()
{
  return txBuffer.size();
}

bool mqttPublishLoopFromTxBuffer()
{
  if(millis()>(u32_mMqttPublishLoopTimmer+15))
  {
    if(smMqttConnectState==SM_MQTT_DISCONNECTED) return false;
    xSemaphoreTake(mMqttMutex, portMAX_DELAY);

    if(txBuffer.size()>0)
    {
      struct mqttEntry_s mqttEntry = txBuffer.at(0);

      String topic = str_mMqttTopicName + "/" + mqttTopics[mqttEntry.t1];
      if(mqttEntry.t2!=-1){topic+="/"; topic+=String(mqttEntry.t2);}
      if(mqttEntry.t3!=-1){topic+="/"; topic+=mqttTopics[mqttEntry.t3];}
      if(mqttEntry.t4!=-1){topic+="/"; topic+=String(mqttEntry.t4);}

      //uint32_t pubTime = millis();
      if(mqttClient.publish(topic.c_str(), mqttEntry.value.c_str()) == false)
      {
        // Wenn die Nachricht nicht mehr versendet werden kann
        BSC_LOGI(TAG, "MQTT Broker nicht erreichbar (Timeout)");
        //BSC_LOGI(TAG, "MQTT Broker nicht erreichbar (Timeout: %i ms)", millis() - pubTime);
        mqttDisconnect();
      }
      txBuffer.pop_front();
    }

    //if(txBuffer.size()==0) bo_mSendPrioMessages=false;

    xSemaphoreGive(mMqttMutex);
    u32_mMqttPublishLoopTimmer=millis();
  }
  return true;
}


void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, std::string value)
{
  if(mMqttEnable <= MQTT_ENABLE_STATE_EN) return;
  if(smMqttConnectState==SM_MQTT_DISCONNECTED) return; //Wenn nicht verbunden, dann Nachricht nicht annehmen
  if(WiFi.status()!=WL_CONNECTED) return; //Wenn Wifi nicht verbunden
  if(BleHandler::isNotAllDeviceConnectedOrScanRunning()) //Wenn nicht alle BT-Devices verbunden sind
  {
     /*if(t1==MQTT_TOPIC_ALARM)
     {
      //Trigger Meldungen mit Priorität abarbeiten
      xSemaphoreTake(mMqttMutex, portMAX_DELAY);
      txBuffer.clear();
      xSemaphoreGive(mMqttMutex);
      mqttPublishTrigger();
      bo_mSendPrioMessages=true;
     }*/
     return;
  }
  if(txBuffer.size()>300)return; //Wenn zu viele Nachrichten im Sendebuffer sind, neue Nachrichten ablehnen

  //Wenn BMS msg, dann msg anpassen
  if(t1==MQTT_TOPIC_BMS_BT)
  {
    if(t2>=BT_DEVICES_COUNT)
    {
      t1=MQTT_TOPIC_BMS_SERIAL;
      t2=t2-BT_DEVICES_COUNT;
    }
  }

  struct mqttEntry_s mqttEntry;
  mqttEntry.t1=t1;
  mqttEntry.t2=t2;
  mqttEntry.t3=t3;
  mqttEntry.t4=t4;
  mqttEntry.value=value;

  xSemaphoreTake(mMqttMutex, portMAX_DELAY);
  txBuffer.push_back(mqttEntry);
  xSemaphoreGive(mMqttMutex);
}


void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, uint32_t value)
{
  mqttPublish(t1, t2, t3, t4, std::to_string(value));
}

void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int32_t value)
{
  mqttPublish(t1, t2, t3, t4, std::to_string(value));
}

void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, float value)
{
  char buffer[20];
  sprintf(buffer, "%.3f", value);
  std::string tmpStr(buffer);

  mqttPublish(t1, t2, t3, t4, tmpStr);
}

void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, bool value)
{
  mqttPublish(t1, t2, t3, t4, std::to_string(value));
}


//Nicht alle mqtt Nachrichten auf einmal senden um RAM zu sparen
void mqttDataToTxBuffer()
{
  if(smMqttConnectState==SM_MQTT_DISCONNECTED) return; //Wenn nicht verbunden, dann zurück

  //Sende Daten via mqtt, wenn aktiv
  if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0))
  {
    uint32_t u8_lMqttSendDelay = (uint32_t)WebSettings::getInt(ID_PARAM_MQTT_SEND_DELAY,0,DT_ID_PARAM_MQTT_SEND_DELAY)*1000;
    if(millis()-sendeTimerBmsMsg>=u8_lMqttSendDelay)
    {
      sendeTimerBmsMsg = millis();
      sendeDelayTimer10ms = millis();
      sendeDelayTimer500ms = millis();
      bmsDataSendFinsh=false;
      sendBmsData_mqtt_sendeCounter=0;
      owDataSendFinsh=false;
      sendOwTemperatur_mqtt_sendeCounter=0;

      mqttPublishTrigger();
    }

    if(millis()-sendeDelayTimer500ms>=500) //Sende alle 500ms eine Nachricht
    {
      sendeDelayTimer500ms = millis();

      if(!bmsDataSendFinsh)
      {
        if((getBmsLastDataMillis(sendBmsData_mqtt_sendeCounter)+5000)>millis()) //Nur senden wenn die Daten nicht älter als 5 sec. sind
        {
          mqttPublishBmsData(sendBmsData_mqtt_sendeCounter);
        }
        else
        {
          mqttPublish(MQTT_TOPIC_BMS_BT, sendBmsData_mqtt_sendeCounter, MQTT_TOPIC2_BMS_DATA_VALID, -1, 0); //invalid
        }
        sendBmsData_mqtt_sendeCounter++;
        if(sendBmsData_mqtt_sendeCounter==BMSDATA_NUMBER_ALLDEVICES)bmsDataSendFinsh=true;
      }
    }

    if(millis()-sendeDelayTimer10ms>=10) //Sende alle 10ms eine Nachricht
    {
      sendeDelayTimer10ms = millis();

      if(!owDataSendFinsh)
      {
        mqttPublishOwTemperatur(sendOwTemperatur_mqtt_sendeCounter);
        sendOwTemperatur_mqtt_sendeCounter++;
        if(sendOwTemperatur_mqtt_sendeCounter==MAX_ANZAHL_OW_SENSOREN)owDataSendFinsh=true;
      }
    }
  }
}


void mqttPublishBmsData(uint8_t i)
{
  if(smMqttConnectState==SM_MQTT_DISCONNECTED) return; //Wenn nicht verbunden, dann zurück

  //CellVoltage
  for(uint8_t n=0;n<24;n++)
  {
    uint16_t u16_lCellVoltage = getBmsCellVoltage(i,n);
    //Zellvoltage nur senden wenn nicht 0xFFFF
    if(u16_lCellVoltage!=0xFFFF) mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE, n, u16_lCellVoltage);
  }

  //Max. Cell Voltage
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE_MAX, -1, getBmsMaxCellVoltage(i));
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE_MAX_NR, -1, getBmsMaxVoltageCellNumber(i));

  //Min. Cell Voltage
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE_MIN, -1, getBmsMinCellVoltage(i));
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE_MIN_NR, -1, getBmsMinVoltageCellNumber(i));

  //bmsTotalVoltage
  //Hier werden nur die Daten von den BT-Devices gesendet
  if(i<=6) mqttPublish(MQTT_TOPIC_BMS_BT, i, (uint8_t)MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(i));

  //maxCellDifferenceVoltage
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_MAXCELL_DIFFERENCE_VOLTAGE, -1, getBmsMaxCellDifferenceVoltage(i));

  //totalCurrent
  //Hier werden nur die Daten von den BT-Devices gesendet
  if(i<=6) mqttPublish(MQTT_TOPIC_BMS_BT, i, (uint8_t)MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(i));

  //balancingActive
  if(i<=4) mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_BALANCING_ACTIVE, -1, getBmsIsBalancingActive(i));

  //balancingCurrent
  if(i<=4) mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_BALANCING_CURRENT, -1, getBmsBalancingCurrent(i));

  //tempature
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_TEMPERATURE, 0, getBmsTempature(i,0));
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_TEMPERATURE, 1, getBmsTempature(i,1));
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_TEMPERATURE, 2, getBmsTempature(i,2));

  //chargePercent
  /*if(i>4)*/ mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CHARGE_PERCENT, -1, getBmsChargePercentage(i));

  //Errors
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_ERRORS, -1, getBmsErrors(i));

  //Warnings
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_WARNINGS, -1, getBmsWarnings(i));

  //
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_FET_STATE_CHARGE, -1, getBmsStateFETsCharge(i));
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_FET_STATE_DISCHARGE, -1, getBmsStateFETsDischarge(i));

  //valid
  mqttPublish(MQTT_TOPIC_BMS_BT, sendBmsData_mqtt_sendeCounter, MQTT_TOPIC2_BMS_DATA_VALID, -1, 1); //invalid
}


void mqttPublishOwTemperatur(uint8_t i)
{
  if(smMqttConnectState==SM_MQTT_DISCONNECTED) return; //Wenn nicht verbunden, dann zurück

  float f_lOwTemp;

  f_lOwTemp=owGetTemp(i);
  if(WebSettings::getStringFlash(ID_PARAM_ONEWIRE_ADR,i).equals("")==false)
  {
    if(f_lOwTemp!=TEMP_IF_SENSOR_READ_ERROR /*&& f_lOwTemp!=0*/)
    {
      mqttPublish(MQTT_TOPIC_TEMPERATUR, i, -1, -1, owGetTemp(i));
    }
  }
  else
  {
    //Evtl. eine "Senor nicht verbunden" Meldung senden
  }
}


void mqttPublishTrigger()
{
  if(smMqttConnectState==SM_MQTT_DISCONNECTED) return; //Wenn nicht verbunden, dann zurück

  for(uint8_t i=0;i<CNT_ALARMS;i++)
  {
    mqttPublish(MQTT_TOPIC_ALARM, i+1, -1, -1, getAlarm(i));
  }
}





// Callback function
void mqttCallback(char* topic, uint8_t* payload, unsigned int payLen)
{
  if(payLen==0) return;

  String str_topicName = WebSettings::getString(ID_PARAM_MQTT_TOPIC_NAME,0);
  String topicStr(topic);
  uint8_t idxCnt = topicStr.indexOf('/');

  if(idxCnt>2)
  {
    String str_lSubTopic=str_topicName+F("/input/vtrigger/");
    if(topicStr.startsWith(str_lSubTopic.c_str()))
    {
      topicStr.remove(0, str_lSubTopic.length());
      uint8_t vTriggerNr = atoi(topicStr.c_str());
      if(vTriggerNr>0 && vTriggerNr<=10 && payLen==1)
      {
        uint8_t vTriggerValue = payload[0];
        if(vTriggerValue=='0')
        {
          //BSC_LOGI(TAG,"vTrigger nr=%i: LOW",vTriggerNr);
          setVirtualTrigger(vTriggerNr, false);
        }
        else if(vTriggerValue=='1')
        {
          //BSC_LOGI(TAG,"vTrigger nr=%i: HIGH",vTriggerNr);
          setVirtualTrigger(vTriggerNr, true);
        }
      }
    }
  }

  //char lPayload[payLen];
  //memcpy(lPayload,payload,payLen);
  //BSC_LOGI(TAG,"CB: topic=%s, payload=%s, len=%i",topic,lPayload,payLen);
}
