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


static const char* TAG = "MQTT";

TimerHandle_t mqttReconnectTimer;

WiFiClient  wifiClient;
PubSubClient mqttClient(wifiClient);

IPAddress   mqttIpAdr;

uint32_t u32_mMqttPublishLoopTimmer=0;
struct mqttEntry_s {
    int8_t t1;
    int8_t t2;
    int8_t t3;
    int8_t t4;
    String value;
};
std::deque<mqttEntry_s> txBuffer;

static SemaphoreHandle_t mMqttMutex = NULL;

bool bo_mIsConnected=false; 

bool mqttPublishLoopFromTxBuffer();
void mqttDataToTxBuffer();
void mqttPublishBmsData(uint8_t);
void mqttPublishOwTemperatur(uint8_t);

void initMqtt()
{
  mMqttMutex = xSemaphoreCreateMutex();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(5000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(mqttConnect));

  if(!WebSettings::getString(ID_PARAM_MQTT_SERVER_IP,0,0,0).equals(""))
  {
    mqttIpAdr.fromString(WebSettings::getString(ID_PARAM_MQTT_SERVER_IP,0,0,0).c_str());
    mqttClient.setServer(mqttIpAdr, WebSettings::getInt(ID_PARAM_MQTT_SERVER_PORT,0,0,0));
  }
}


void mqttLoop()
{
  if(!mqttClient.connected())
  {
    if(bo_mIsConnected) mqttConnect();
  }
  mqttClient.loop();

  if(bo_mIsConnected)
  {
    //MQTT Messages zyklisch publishen
    mqttPublishLoopFromTxBuffer(); 

    //Sende Diverse MQTT Daten
    mqttDataToTxBuffer();
  }
}


void mqttConnect()
{
  //Nur wenn WLAN-Verbindung besteht
  if(WiFi.status() != WL_CONNECTED) return;

  if(WebSettings::getString(ID_PARAM_MQTT_SERVER_IP,0,0,0).equals("")) return;
  if(WebSettings::getString(ID_PARAM_MQTT_SERVER_PORT,0,0,0).equals("")) return;
  
  str_mMqttDeviceName = WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0,0,0);
  String mqttUser = WebSettings::getString(ID_PARAM_MQTT_USERNAME,0,0,0);
  String mqttPwd = WebSettings::getString(ID_PARAM_MQTT_PWD,0,0,0);

  if(!mqttClient.connected())
  {
    bo_mIsConnected=false;
    
    ESP_LOGD(TAG,"Connecting to MQTT");
    xTimerStart(mqttReconnectTimer, 0);
    if(mqttUser.equals("") || mqttPwd.equals("")) //Wenn kein User oder Pwd eingetragen, dann ohne verbinden
    {
      mqttClient.connect(str_mMqttDeviceName.c_str()); 
    }
    else
    {
      mqttClient.connect(str_mMqttDeviceName.c_str(), mqttUser.c_str(), mqttPwd.c_str());
    }
  }
  else
  {
    bo_mIsConnected=true;
    ESP_LOGI(TAG,"MQTT connected");
  }
}


void mqttDisconnect()
{
  bo_mIsConnected=false;
  
  xTimerStop(mqttReconnectTimer, 0); 

  if(mqttClient.connected())
  {
    ESP_LOGI(TAG,"Disconnecting to MQTT");
    mqttClient.disconnect();
  }
}


bool mqttConnected()
{
  return bo_mIsConnected;
}


bool mqttPublishLoopFromTxBuffer()
{
  if(millis()>(u32_mMqttPublishLoopTimmer+10))
  {
    if(bo_mIsConnected==false) return false;
    xSemaphoreTake(mMqttMutex, portMAX_DELAY);
    
    if(txBuffer.size()>0)
    {
      if(mqttClient.connected())
      {
        struct mqttEntry_s mqttEntry = txBuffer.at(0);

        String topic = str_mMqttDeviceName + "/" + mqttTopics[mqttEntry.t1];
        if(mqttEntry.t2!=-1){topic+="/"; topic+=String(mqttEntry.t2);}
        if(mqttEntry.t3!=-1){topic+="/"; topic+=mqttTopics[mqttEntry.t3];}
        if(mqttEntry.t4!=-1){topic+="/"; topic+=String(mqttEntry.t4);}
        //debugPrintln(topic);

        mqttClient.publish(topic.c_str(), mqttEntry.value.c_str());
        txBuffer.pop_front();
      }
      else
      {
        ESP_LOGD(TAG,"mqttPublish: MQTT not connected");
        u32_mMqttPublishLoopTimmer=millis();
        bo_mIsConnected=false;
        mqttConnect();
        return false;
      }
    }

    xSemaphoreGive(mMqttMutex);
    u32_mMqttPublishLoopTimmer=millis();
  }
  return true;
}


void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, String value)
{
  if(bo_mIsConnected==false) return; //Wenn nicht verbunden, dann Nachricht nicht annehmen

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
  mqttPublish(t1, t2, t3, t4, String(value));
}

void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int32_t value)
{
  mqttPublish(t1, t2, t3, t4, String(value));
}

void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, float value)
{
  mqttPublish(t1, t2, t3, t4, String(value));
}

void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, bool value)
{
  mqttPublish(t1, t2, t3, t4, String(value));
}


uint32_t sendeTimerBmsMsg;
uint32_t sendeDelayTimer10ms;
uint32_t sendeDelayTimer100ms;
uint8_t sendBmsData_mqtt_sendeCounter=0;
bool bmsDataSendFinsh=false;
uint8_t sendOwTemperatur_mqtt_sendeCounter=0;
bool owDataSendFinsh=false;

//Nicht alle mqtt Nachrichten auf einmal senden um RAM zu sparen
void mqttDataToTxBuffer()
{ 
  if(bo_mIsConnected==false) return; //Wenn nicht verbunden, dann zurück

  //Sende Daten via mqtt, wenn aktiv
  if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
  {
    if(millis()-sendeTimerBmsMsg>=10000)
    {
      sendeTimerBmsMsg = millis();
      sendeDelayTimer10ms = millis();
      sendeDelayTimer100ms = millis();
      bmsDataSendFinsh=false;
      sendBmsData_mqtt_sendeCounter=0;
      owDataSendFinsh=false;
      sendOwTemperatur_mqtt_sendeCounter=0;
    }

    if(millis()-sendeDelayTimer100ms>=500) //Sende alle 100ms eine Nachricht
    {
      sendeDelayTimer100ms = millis();

      if(!bmsDataSendFinsh)
      {
        mqttPublishBmsData(sendBmsData_mqtt_sendeCounter);
        sendBmsData_mqtt_sendeCounter++;
        if(sendBmsData_mqtt_sendeCounter==BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT)bmsDataSendFinsh=true;
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
  if(bo_mIsConnected==false) return; //Wenn nicht verbunden, dann zurück
  


  //CellVoltage
  for(uint8_t n=0;n<24;n++)
  {
    mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE, n, getBmsCellVoltage(i,n));
  }

  //Max. Cell Voltage
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE_MAX, -1, getBmsMaxCellVoltage(i));

  //Min. Cell Voltage
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CELL_VOLTAGE_MIN, -1, getBmsMinCellVoltage(i));
  
  //bmsTotalVoltage
  //Hier werden nur die Daten von den BT-Devices gesendet
  if(i<=4) mqttPublish(MQTT_TOPIC_BMS_BT, i, (uint8_t)MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(i));

  //maxCellDifferenceVoltage
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_MAXCELL_DIFFERENCE_VOLTAGE, -1, getBmsMaxCellDifferenceVoltage(i));

  //totalCurrent
  //if(i>4) mqttPublish("bms/"+String(i)+"/totalCurrent", getBmsTotalCurrent(i));

  //balancingActive
  if(i<=4) mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_BALANCING_ACTIVE, -1, getBmsIsBalancingActive(i));

  //balancingCurrent
  if(i<=4) mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_BALANCING_CURRENT, -1, getBmsBalancingCurrent(i));

  //tempature
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_TEMPERATURE, 0, getBmsTempature(i,0));
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_TEMPERATURE, 1, getBmsTempature(i,1));
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_TEMPERATURE, 2, getBmsTempature(i,2));

  //chargePercent
  if(i>4) mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_CHARGE_PERCENT, -1, getBmsChargePercentage(i));

  //Errors
  mqttPublish(MQTT_TOPIC_BMS_BT, i, MQTT_TOPIC2_ERRORS, -1, getBmsErrors(i));
}


void mqttPublishOwTemperatur(uint8_t i)
{
  if(bo_mIsConnected==false) return; //Wenn nicht verbunden, dann zurück

  float f_lOwTemp;

  f_lOwTemp=owGetTemp(i);
  if(WebSettings::getString(ID_PARAM_ONEWIRE_ADR,0,0,i).equals("")==false)
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