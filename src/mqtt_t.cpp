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


static const char* TAG = "mqqt_t";

TimerHandle_t mqttReconnectTimer;

WiFiClient  wifiClient;
PubSubClient mqttClient(wifiClient);

IPAddress   mqttIpAdr;

uint32_t mqttPublishLoopTimmer=0;
std::deque<std::array<String, 2>> txBuffer;

SemaphoreHandle_t mMqttMutex = NULL;

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
  
  mqttDeviceName = WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0,0,0);

  if(!mqttClient.connected())
  {
    bo_mIsConnected=false;
    
    ESP_LOGD(TAG,"Connecting to MQTT");
    xTimerStart(mqttReconnectTimer, 0);
    mqttClient.connect(mqttDeviceName.c_str()); 
  }
  else
  {
    bo_mIsConnected=true;
    ESP_LOGD(TAG,"MQTT connected");
  }
}


void mqttDisconnect()
{
  bo_mIsConnected=false;
  
  xTimerStop(mqttReconnectTimer, 0); 

  if(mqttClient.connected())
  {
    Serial.println("Disconnecting to MQTT...");
    mqttClient.disconnect();
  }
}


bool mqttConnected()
{
  return bo_mIsConnected;
}


bool mqttPublishLoopFromTxBuffer()
{
  if(millis()>(mqttPublishLoopTimmer+10))
  {
    if(bo_mIsConnected==false) return false;
    xSemaphoreTake(mMqttMutex, portMAX_DELAY);
    
    if(txBuffer.size()>0)
    {
      if(mqttClient.connected())
      {
        String topic = mqttDeviceName + "/" + txBuffer.at(0)[0].c_str();
        mqttClient.publish(topic.c_str(), txBuffer.at(0)[1].c_str());
        txBuffer.pop_front();
      }
      else
      {
        ESP_LOGD(TAG,"mqttPublish: MQTT not connected");
        mqttPublishLoopTimmer=millis();
        bo_mIsConnected=false;
        mqttConnect();
        return false;
      }
    }

    xSemaphoreGive(mMqttMutex);
    mqttPublishLoopTimmer=millis();
  }
  return true;
}


void mqttPublish(String topic, String value)
{
  if(bo_mIsConnected==false) return; //Wenn nicht verbunden, dann Nachricht nicht annehmen

  xSemaphoreTake(mMqttMutex, portMAX_DELAY);
  txBuffer.push_back({{topic,value}});
  xSemaphoreGive(mMqttMutex);
}

void mqttPublish(String topic, uint32_t value)
{
  mqttPublish(topic, String(value));
}

void mqttPublish(String topic, int32_t value)
{
  mqttPublish(topic, String(value));
}

void mqttPublish(String topic, uint8_t value)
{
  mqttPublish(topic, String(value));
}

void mqttPublish(String topic, int8_t value)
{
  mqttPublish(topic, String(value));
}

void mqttPublish(String topic, float value)
{
  mqttPublish(topic, String(value));
}

void mqttPublish(String topic, bool value)
{
  mqttPublish(topic, String(value));
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
    mqttPublish("bms/"+String(i)+"/cellVoltage/"+String(n), getBmsCellVoltage(i,n));
  }

  //Max. Cell Voltage
  mqttPublish("bms/"+String(i)+"/cellVoltageMax", getBmsMaxCellVoltage(i));

  //Min. Cell Voltage
  mqttPublish("bms/"+String(i)+"/cellVoltageMin", getBmsMinCellVoltage(i));
  
  //bmsTotalVoltage
  //Hier werden nur die Daten von den BT-Devices gesendet
  if(i<=4) mqttPublish("bms/"+String(i)+"/totalVoltage", getBmsTotalVoltage(i));

  //maxCellDifferenceVoltage
  mqttPublish("bms/"+String(i)+"/maxCellDifferenceVoltage", getBmsMaxCellDifferenceVoltage(i));

  //totalCurrent
  //if(i>4) mqttPublish("bms/"+String(i)+"/totalCurrent", getBmsTotalCurrent(i));

  //balancingActive
  if(i<=4) mqttPublish("bms/"+String(i)+"/balancingActive", getBmsIsBalancingActive(i));

  //balancingCurrent
  if(i<=4) mqttPublish("bms/"+String(i)+"/balancingCurrent", getBmsBalancingCurrent(i));

  //tempature
  /*if(i<=4)*/ mqttPublish("bms/"+String(i)+"/tempature/0", getBmsTempature(i,0));
  /*if(i<=4)*/ mqttPublish("bms/"+String(i)+"/tempature/1", getBmsTempature(i,1));

  //chargePercent
  if(i>4) mqttPublish("bms/"+String(i)+"/chargePercent", getBmsChargePercentage(i));

  //Errors
  mqttPublish("bms/"+String(i)+"/errors", getBmsErrors(i));
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
      mqttPublish("temperatur/"+String(i), owGetTemp(i));
    }   
  }
  else
  {
    //Evtl. eine "Senor nicht verbunden" Meldung senden
  } 
}