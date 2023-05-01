// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


/*
// Core dump
#define CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH 1
// #define CONFIG_ESP_COREDUMP_ENABLE_TO_UART is not set
// #define CONFIG_ESP_COREDUMP_ENABLE_TO_NONE is not set
// #define CONFIG_ESP_COREDUMP_DATA_FORMAT_BIN is not set
#define CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF 1
#define CONFIG_ESP_COREDUMP_CHECKSUM_CRC32 1
// #define CONFIG_ESP_COREDUMP_CHECKSUM_SHA256 is not set
#define CONFIG_ESP_COREDUMP_CHECK_BOOT 1
#define CONFIG_ESP_COREDUMP_ENABLE 1
#define CONFIG_ESP_COREDUMP_MAX_TASKS_NUM 64
#define CONFIG_ESP_COREDUMP_STACK_SIZE 0
// end of Core dump

#define CONFIG_BTDM_CTRL_MODEM_SLEEP 0              //1->0  shiningman
#define CONFIG_BTDM_CTRL_MODEM_SLEEP_MODE_ORIG 0    //1->0  shiningman
*/


#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebOTA.h>
#include <SPIFFS.h>

#include "defines.h"
#include "WebSettings.h"
#include "BleHandler.h"
#include "params.h"
#include "webpages.h"
#include "AlarmRules.h"
#include "dio.h"
#include "Ow.h"
#include "Canbus.h"
#include "BscSerial.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include "i2c.h"
#include "webUtility.h"
#include "bscTime.h"
#include "restapi.h"
#include "devices/NeeyBalancer.h"
/*extern "C"
{
  #include "esp_core_dump.h"
}*/

static const char *TAG = "MAIN";

WebServer server;
BleHandler* bleHandler;
BscSerial bscSerial;   // Serial

//Websettings
WebSettings webSettingsSystem;
WebSettings webSettingsBluetooth;
WebSettings webSettingsSerial;
WebSettings webSettingsAlarmBt;
WebSettings webSettingsAlarmTemp;
WebSettings webSettingsDitialOut;
WebSettings webSettingsDitialIn;
WebSettings webSettingsOnewire;
WebSettings webSettingsOnewire2;
WebSettings webSettingsBmsToInverter;
WebSettings webSettingsDeviceNeeyBalancer;
WebSettings webSettingsDeviceJbdBms;

//Tasks
TaskHandle_t task_handle_ble = NULL;
TaskHandle_t task_handle_alarmrules = NULL;
TaskHandle_t task_handle_onewire = NULL;
TaskHandle_t task_handle_canbusTx = NULL;
TaskHandle_t task_handle_bscSerial = NULL;
TaskHandle_t task_handle_i2c = NULL;
TaskHandle_t task_handle_wifiConn = NULL;

//Task semaphore
SemaphoreHandle_t mutexTaskRunTime_ble = NULL;
SemaphoreHandle_t mutexTaskRunTime_alarmrules = NULL;
SemaphoreHandle_t mutexTaskRunTime_ow = NULL;
SemaphoreHandle_t mutexTaskRunTime_can = NULL;
SemaphoreHandle_t mutexTaskRunTime_serial = NULL;
SemaphoreHandle_t mutexTaskRunTime_i2c = NULL;
SemaphoreHandle_t mutexTaskRunTime_wifiConn = NULL;

//millis des letzten runs
uint32_t lastTaskRun_ble = 0;
uint32_t lastTaskRun_alarmrules = 0;
uint32_t lastTaskRun_onewire = 0;
uint32_t lastTaskRuncanbusTx = 0;
uint32_t lastTaskRun_bscSerial = 0;
uint32_t lastTaskRun_i2c = 0;
uint32_t lastTaskRun_wifiConn = 0;

RTC_DATA_ATTR static uint8_t bootCounter = 0;


uint8_t    u8_mTaskRunSate=0;           //Status ob alle Tasks laufen
bool       isBoot=true;
bool       doConnectWiFi=false;         //true, wenn gerade versucht wird eine Verbindung aufzubauen
bool       firstWlanModeSTA=false;      //true, wenn der erste WLAN-Mode nach einem Neustart STA ist
WiFiMode_t WlanStaApOk=WIFI_OFF;        //WIFI_OFF, wenn Wlan verbunden oder AP erstellt
bool       wlanEventStaDisonnect=false;
bool       wlanEventStaConnect=false;

bool       changeWlanDataForI2C=false;  //true, wenn sich die WLAN Verbindung geändert hat

// Variablen für connectWifi()
static boolean bo_mWifiConnected;
static String str_lWlanSsid;
static String str_lWlanPwd;
static uint16_t u16_lWlanConnTimeout;
static boolean bo_lWlanNeverAp;
static unsigned long wlanConnectTimer;
static const char *hostname = {"bsc"};

//
void task_ble(void *param);
boolean connectWiFi();
//void readCoreDump();
//esp_err_t esp_core_dump_image_erase();

void free_dump()
{
  BSC_LOGI(TAG, "Free Heap: %i", ESP.getFreeHeap());
}

/*
void readCoreDump()
{
  size_t size = 0;
  size_t address = 0;
  if (esp_core_dump_image_get(&address, &size) == ESP_OK)
  {
    const esp_partition_t *pt = NULL;
    pt = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, "coredump");

    if (pt != NULL)
    {
      uint8_t bf[256];
      char str_dst[640];
      int16_t toRead;

      for (int16_t i = 0; i < (size / 256) + 1; i++)
      {
        strcpy(str_dst, "");
        toRead = (size - i * 256) > 256 ? 256 : (size - i * 256);

        esp_err_t er = esp_partition_read(pt, i * 256, bf, toRead);
        if (er != ESP_OK)
        {
          Serial.printf("FAIL [%x]\n",er);
          //ESP_LOGE("ESP32", "FAIL [%x]", er);
          break;
        }

        for (int16_t j = 0; j < 256; j++)
        {
          char str_tmp[3];

          sprintf(str_tmp, "%02x", bf[j]);
          strcat(str_dst, str_tmp);
        }

        printf("%s", str_dst);
      }
    }
    else
    {
      Serial.println("Partition NULL");
      //ESP_LOGE("ESP32", "Partition NULL");
    }
    esp_core_dump_image_erase();
  }
  else
  {
    Serial.println("esp_core_dump_image_get() FAIL");
    //ESP_LOGI("ESP32", "esp_core_dump_image_get() FAIL");
  }
}
*/


void onWiFiEvent(WiFiEvent_t event)
{
  BSC_LOGI(TAG, "[WiFi] event: %d", event);
  switch(event)
  {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wlanEventStaDisonnect=false;
      wlanEventStaConnect=true;
      break;
        
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      wlanEventStaConnect=false;
      if(WlanStaApOk!=WIFI_AP)
        wlanEventStaDisonnect=true;
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      WlanStaApOk=WIFI_AP;
      break;
  }
}


boolean connectWiFi()
{
  doConnectWiFi=true;

  bo_mWifiConnected=false;
  str_lWlanSsid = webSettingsSystem.getString(ID_PARAM_WLAN_SSID,0);
  str_lWlanPwd  = webSettingsSystem.getString(ID_PARAM_WLAN_PWD,0);
  u16_lWlanConnTimeout  = webSettingsSystem.getInt(ID_PARAM_WLAN_CONNECT_TIMEOUT,0,DT_ID_PARAM_WLAN_CONNECT_TIMEOUT);
  bo_lWlanNeverAp=false;

  if(u16_lWlanConnTimeout==0)
  {
    if(firstWlanModeSTA) bo_lWlanNeverAp=true;
    u16_lWlanConnTimeout=30;
  }
  
  #ifdef WLAN_DEBUG
  BSC_LOGI(TAG, "[WiFi] status (a): %i", WiFi.status());
  #endif

  if(!str_lWlanSsid.equals("") && !str_lWlanPwd.equals(""))
  {
    BSC_LOGI(TAG, "Verbindung zu %s",str_lWlanSsid.c_str());
    WiFi.scanDelete();
    WiFi.setHostname(hostname);
    WiFi.mode(WIFI_STA);
    WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
    WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
    WiFi.begin(str_lWlanSsid.c_str(), str_lWlanPwd.c_str());

    wlanConnectTimer=millis()+(u16_lWlanConnTimeout*1000);
    #ifdef WLAN_DEBUG
    BSC_LOGI(TAG, "wlanConnectTimer=%i",wlanConnectTimer);
    #endif
    uint8_t cnt=0;
    while ((WiFi.status() != WL_CONNECTED))
    {
      #ifdef WLAN_DEBUG
      BSC_LOGI(TAG, "wlanConnectTimer=%i",wlanConnectTimer);
      #endif
      if(!bo_lWlanNeverAp && millis()>wlanConnectTimer) break;

      if(cnt>15)
      {
        cnt=0;
        WiFi.begin(str_lWlanSsid.c_str(), str_lWlanPwd.c_str());
      }
      cnt++;

      vTaskDelay(pdMS_TO_TICKS(1000));

      //Lebenszeichen des Tasks
      if(xSemaphoreTake(mutexTaskRunTime_wifiConn, 100))
      {
        lastTaskRun_wifiConn=millis();
        xSemaphoreGive(mutexTaskRunTime_wifiConn);
      }
    }
    
    if (WiFi.status() == WL_CONNECTED)
    {
      BSC_LOGI(TAG, "IP-Adresse = %s",WiFi.localIP().toString().c_str());
      bo_mWifiConnected=true;
      firstWlanModeSTA=true;
      changeWlanDataForI2C=true;
    }
  }
  
  if (!bo_mWifiConnected && (!firstWlanModeSTA || !bo_lWlanNeverAp))
  {
    BSC_LOGI(TAG, "Wifi AP");
    WlanStaApOk=WIFI_AP;
    WiFi.mode(WIFI_AP);
    String str_lHostname = hostname;
    str_lHostname += "_" + String((uint32_t)ESP.getEfuseMac());
    WiFi.softAP(str_lHostname.c_str(),"",1);  
    changeWlanDataForI2C=true;
  }
  
  #ifdef WLAN_DEBUG
  BSC_LOGI(TAG, "[WiFi] status (b): %i", WiFi.status());
  #endif
  doConnectWiFi=false;
  return bo_mWifiConnected;
}


/*
  Handle Tasks
*/
void task_ConnectWiFi(void *param)
{
  enum connectStateEnums {ConnState_wifiDisconnected, ConnState_noWifiConnection, ConnState_connectWifi, ConnState_wifiConnected, 
    ConnState_wlanApMode, ConnState_connectMQTTstart, ConnState_connectMQTT, ConnState_connectBT, ConnState_connectBT2, ConnState_idle};

  connectStateEnums mConnectStateEnums=ConnState_noWifiConnection;
  connectStateEnums mConnectStateEnumsOld=ConnState_noWifiConnection;
  uint8_t u8_mWaitConnCounter=0;
  unsigned long connectMqttTimer; //Timeout MQTT Verbindungsaufbau
  bool bo_lFirstRun=true;

  #ifdef WLAN_DEBUG
  unsigned long tConnWifiHelpTimer=0;
  tConnWifiHelpTimer=millis();
  #endif

  BSC_LOGD(TAG, "-> 'task_ConnectWiFi' runs on core %d", xPortGetCoreID());
  BSC_LOGD(TAG, "mConnectState=%i", mConnectStateEnums);

  for(;;)
  {
    switch(mConnectStateEnums)
    {
      case ConnState_wifiDisconnected:
        WlanStaApOk=WIFI_OFF;
        mqttDisconnect();
        server.stop(); //Webserver beenden
        bleHandler->stop();
        mConnectStateEnums=ConnState_noWifiConnection;
        break;

      case ConnState_noWifiConnection: //kein Wlan
        connectWiFi();
        if(WlanStaApOk==WIFI_AP) mConnectStateEnums=ConnState_wlanApMode;
        else mConnectStateEnums=ConnState_connectWifi;
        break;

      case ConnState_connectWifi: //Wlan Verbindug aufbauen
        if(WiFi.status()==WL_CONNECTED && wlanEventStaConnect)
        {
          u8_mWaitConnCounter=0;
          wlanEventStaConnect=false;
          mConnectStateEnums=ConnState_wifiConnected;
        }
        else
        {
          if(u8_mWaitConnCounter>5)
          {
            u8_mWaitConnCounter=0;
            mConnectStateEnums=ConnState_connectBT;
          }
          u8_mWaitConnCounter++;
        }
        break;

      case ConnState_wifiConnected: //Wlan verbunden
        WlanStaApOk=WIFI_STA;
        server.begin(WEBSERVER_PORT);  //Webserver starten
        if(bo_lFirstRun)
        {
          bo_lFirstRun=false;
          initTime();
        }
        timeRunCyclic(true); //Hole 1x die Zeit
        BSC_LOGI(TAG,"Time: %s",getBscDateTime().c_str());
        
        mConnectStateEnums=ConnState_connectMQTTstart;
        break;

      case ConnState_wlanApMode:
        server.begin(WEBSERVER_PORT);  //Webserver starten
        mConnectStateEnums=ConnState_connectBT;
        break;


      case ConnState_connectMQTTstart: //MQTT verbinden
        connectMqttTimer=millis();
        mConnectStateEnums=ConnState_connectMQTT;
        break;

      case ConnState_connectMQTT: //MQTT verbinden
        //Wenn 30s keine Verbindung möglich, dann Verbindungsversuch abbrechen
        if(millis()-connectMqttTimer>=30000)
        {
          BSC_LOGI(TAG,"No Mqtt connection!");
          connectMqttTimer=millis();
          mConnectStateEnums=ConnState_connectBT;
          break;
        }

        if(wlanEventStaDisonnect==true && WlanStaApOk!=WIFI_AP)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }

        if(webSettingsSystem.getBool(ID_PARAM_MQTT_SERVER_ENABLE,0))
        {
          if(mqttConnect()) mConnectStateEnums=ConnState_connectBT;
        }
        else
        {
          mConnectStateEnums=ConnState_connectBT;
        }
        break;

      case ConnState_connectBT: //BT verbinden
        if(wlanEventStaDisonnect==true && WlanStaApOk!=WIFI_AP)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }

        bleHandler->start();
        mConnectStateEnums=ConnState_connectBT2;
        break;

      case ConnState_connectBT2: //BT verbinden
        //ToDo: Timeout einbauen

        if(wlanEventStaDisonnect==true && WlanStaApOk!=WIFI_AP)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }

        if(bleHandler->isNotAllDeviceConnectedOrScanRunning()==false)mConnectStateEnums=ConnState_idle;
        break;

      case ConnState_idle: //Idle; Wenn alle Verbindungen stehen
        if(wlanEventStaDisonnect==true && WlanStaApOk!=WIFI_AP)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }
        //Wenn MQTT Disconnected -> mConnectStateEnums=ConnState_connectMQTT
        else if(WlanStaApOk==WIFI_STA)
        {
          if(!mqttLoop())
          {
            mConnectStateEnums=ConnState_connectMQTTstart; //ConnState_connectMQTT;
            break;
          }
        }
        break;

      default:
        BSC_LOGE(TAG, "Error: mConnectState=%i", mConnectStateEnums);
        mConnectStateEnums=ConnState_wifiDisconnected;
    }

    if(mConnectStateEnums!=mConnectStateEnumsOld)
    {
      BSC_LOGD(TAG, "mConnectState=%i", mConnectStateEnums);
      mConnectStateEnumsOld=mConnectStateEnums;
    }
    
    
    if((millis()-tConnWifiHelpTimer)>1000)
    {
      tConnWifiHelpTimer=millis();
      #ifdef WLAN_DEBUG2
      BSC_LOGD(TAG, "FreeHeap=%i, MinFreeHeap=%i, mqttTxBuffSize=%i, WlanStaApOk=%i", xPortGetFreeHeapSize(),xPortGetMinimumEverFreeHeapSize(),getTxBufferSize(),WlanStaApOk);
      #endif

      if(xSemaphoreTake(mutexTaskRunTime_wifiConn, 100))
      {
        lastTaskRun_wifiConn=millis();
        xSemaphoreGive(mutexTaskRunTime_wifiConn);
      }

      if(mConnectStateEnums==ConnState_idle)
      {
        timeRunCyclic(false);
      }
    }

    //Wenn nicht im Idle, dann 1000ms warten
    if(mConnectStateEnums!=ConnState_idle) vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task_ble(void *param)
{
  BSC_LOGD(TAG, "-> 'task_ble' runs on core %d", xPortGetCoreID());

  //init Bluetooth
  BSC_LOGI(TAG, "Init BLE...");
  bleHandler = new BleHandler();
  bleHandler->init();
  BSC_LOGI(TAG, "Init BLE...ok");

  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(2000));
    if(doConnectWiFi==false) break;
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    if(bleHandler!=nullptr) if(WlanStaApOk!=WIFI_OFF) bleHandler->run();
    
    xSemaphoreTake(mutexTaskRunTime_ble, portMAX_DELAY);
    lastTaskRun_ble=millis();
    xSemaphoreGive(mutexTaskRunTime_ble);
  }
}

void task_alarmRules(void *param)
{
  BSC_LOGD(TAG, "-> 'task_alarmRules' runs on core %d", xPortGetCoreID());

  initAlarmRules();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    runAlarmRules();
    xSemaphoreTake(mutexTaskRunTime_alarmrules, portMAX_DELAY);
    lastTaskRun_alarmrules=millis();
    xSemaphoreGive(mutexTaskRunTime_alarmrules);
  }
}

void task_onewire(void *param)
{
  BSC_LOGD(TAG, "-> 'task_onewire' runs on core %d", xPortGetCoreID());

  //init Onewire
  owSetup();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    owCyclicRun();
    xSemaphoreTake(mutexTaskRunTime_ow, portMAX_DELAY);
    lastTaskRun_onewire=millis();
    xSemaphoreGive(mutexTaskRunTime_ow);
  }
}

void task_canbusTx(void *param)
{
  BSC_LOGD(TAG, "-> 'task_canbusTx' runs on core %d", xPortGetCoreID());

  canSetup();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    canTxCyclicRun();
    if(xSemaphoreTake(mutexTaskRunTime_can, 100))
    {
      lastTaskRuncanbusTx=millis();
      xSemaphoreGive(mutexTaskRunTime_can);
    }
  }
}

void task_bscSerial(void *param)
{
  BSC_LOGD(TAG, "-> 'task_bscSerial' runs on core %d", xPortGetCoreID());

  //init Serial
  bscSerial.initSerial();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    bscSerial.cyclicRun();
    xSemaphoreTake(mutexTaskRunTime_serial, portMAX_DELAY);
    lastTaskRun_bscSerial=millis();
    xSemaphoreGive(mutexTaskRunTime_serial);
  }
}

void task_i2c(void *param)
{
  BSC_LOGD(TAG, "-> 'task_i2c' runs on core %d", xPortGetCoreID());

  i2cInit();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(2000));
    i2cCyclicRun(); //Sende Daten zum Display

    if(changeWlanDataForI2C)
    {
      changeWlanDataForI2C=false;
      String ipAddr;
      if(WiFi.getMode()==WIFI_MODE_AP) ipAddr="192.168.4.1";
      else ipAddr = WiFi.localIP().toString();
      i2cSendData(I2C_DEV_ADDR_DISPLAY, BSC_DATA, BSC_IP_ADDR, 0, ipAddr, 16);
    }
      
    xSemaphoreTake(mutexTaskRunTime_i2c, portMAX_DELAY);
    lastTaskRun_i2c=millis();
    xSemaphoreGive(mutexTaskRunTime_i2c);
  }
}


/*
  Handle WebPages
*/
void handlePage_root(){server.send(200, "text/html", htmlPageRoot);}
void handlePage_settings(){server.send(200, "text/html", htmlPageSettings);}
void handlePage_alarm(){server.send(200, "text/html", htmlPageAlarm);}
void handlePage_schnittstellen(){server.send(200, "text/html", htmlPageSchnittstellen);}
void handle_htmlPageBmsSpg(){server.send(200, "text/html", htmlPageBmsSpg);}
void handlePage_status(){server.send(200, "text/html", htmlPageStatus);}
void handlePage_htmlPageOwTempLive(){server.send(200, "text/html", htmlPageOwTempLive);}


/*
  Handle WebSettings
*/
void handle_paramAlarmBt()
{
  webSettingsAlarmBt.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    changeAlarmSettings();
  }
}

void handle_paramAlarmTemp()
{
  webSettingsAlarmTemp.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    changeAlarmSettings();
  }
}

void handle_paramDigitalOut()
{
  webSettingsDitialOut.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    changeAlarmSettings();
  }
}

void handle_paramDigitalIn()
{
  webSettingsDitialIn.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    changeAlarmSettings();
  }
}

void handle_paramBluetooth()
{
  webSettingsBluetooth.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    changeAlarmSettings();
  }
}

void handle_paramOnewire2(){webSettingsOnewire2.handleHtmlFormRequest(&server);}

void handle_paramBmsToInverter()
{
  webSettingsBmsToInverter.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    loadCanSettings();
  }
}

void handle_paramSerial()
{
  webSettingsSerial.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      bscSerial.setReadBmsFunktion(i, WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,i,DT_ID_PARAM_SERIAL_CONNECT_DEVICE));
    }
    changeAlarmSettings();

    bmsFilterData_s* bmsFilterData = getBmsFilterData();
    bmsFilterData->u8_mFilterBmsCellVoltagePercent = WebSettings::getIntFlash(ID_PARAM_BMS_FILTER_CELL_VOLTAGE_PERCENT,0,DT_ID_PARAM_BMS_FILTER_CELL_VOLTAGE_PERCENT);
  }
}

void handle_paramOnewireAdr()
{
  webSettingsOnewire.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    takeOwSensorAddress();
  }
}

void handle_paramSystem()
{
  webSettingsSystem.handleHtmlFormRequest(&server);
  if (server.hasArg("SAVE"))
  {
    initMqtt();

    if(webSettingsSystem.getBool(ID_PARAM_MQTT_SERVER_ENABLE,0))
    {
      mqttConnect();
    }
    else
    {
      mqttDisconnect();
    }
  }
}

void handle_paramDevicesNeeyBalancer(){webSettingsDeviceNeeyBalancer.handleHtmlFormRequest(&server);}
void handle_getNeeySettingsReadback()
{
  String value;
  NeeyBalancer::getNeeyReadbackDataAsString(value);
  server.send(200, "text/html", value);
  value="";
}        

void handle_paramDeviceJbdBms(){webSettingsDeviceJbdBms.handleHtmlFormRequest(&server);}

void handle_getData()
{
  server.send(200, "text/html", "Testdata");
}

void handle_getDashboardData()
{
  //1. Free Heap
  String tmp = String(xPortGetFreeHeapSize());
  tmp += "<br>";
  tmp += String(xPortGetMinimumEverFreeHeapSize());

  //2. Alarme
  tmp += "|";
  for(uint8_t i=0;i<CNT_ALARMS;i++)
  {
    tmp += getAlarm(i);
    if(i<CNT_ALARMS) tmp += "&nbsp;";
    if(i==4) tmp += "|";
  }

  //3. mqtt state
  tmp += "|";
  tmp += mqttConnected();

  //4. Task status
  tmp += "|";
  tmp += String(u8_mTaskRunSate);
  
  //5. + 6. BT-Devices
  tmp += "|";
  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleHandler!=nullptr)
    {
      uint8_t u8_lBtDevConnState=bleHandler->bmsIsConnect(i);
      if(u8_lBtDevConnState==0) tmp += "&ndash;";
      else if(u8_lBtDevConnState==1) tmp += "n";
      else if(u8_lBtDevConnState==2) tmp += "c";
    } 
    if(i<BT_DEVICES_COUNT) tmp += "&nbsp;";
    if(i==4) tmp += "|";
  }

  //7. WLAN RSSI
  tmp += "|";
  tmp += String(WiFi.RSSI()+100); //0...100 => bad...good

  server.send(200, "text/html", tmp.c_str());
}

void handle_getBmsSpgData()
{
  String sendData = "";
  float onePer = (4-0.5)/100;
  for(uint8_t i=0;i<16;i++)
  {
    int8_t spg = (getBmsCellVoltage(1,i)-0.5)/onePer;
    if(spg<0){spg=0;}
    sendData += String(spg);
    sendData += ";";
    sendData += String(getBmsCellVoltage(1,i));
    sendData += ";";
  }
  server.send(200, "text/html", sendData.c_str());
}

void handle_getOwTempData()
{
  String sendData = "";
  for(uint8_t i=0;i<MAX_ANZAHL_OW_SENSOREN;i++)
  {
    sendData += String(owGetTemp(i));
    sendData += ";";
  }
  server.send(200, "text/html", sendData.c_str());
}

void handle_getBtDevices()
{
  if(bleHandler!=nullptr)
  {
  bleHandler->startScan(); //BT Scan starten
  server.send(200, "text/html", bleHandler->getBtScanResult().c_str());
  }
}


void handle_getOnewireDeviceAdr()
{
  server.send(200, "text/html", getSensorAdr().c_str());
}


void btnSystemDeleteLog()
{
  SPIFFS.remove("/log.txt");
  SPIFFS.remove("/log1.txt");
  if(SPIFFS.exists("/log.txt")) //Wenn Log-Datei immer noch vorhanden
  {
    SPIFFS.format();
    delay(100);
    webSettingsSystem.writeConfig();
  }
  BSC_LOGI(TAG, "Logfiles deleted");
}


void btnWriteNeeyData()
{
  bleHandler->sendDataToNeey();
}


void btnWriteJbdBmsData()
{
  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    setSerialBmsWriteData(i,true);
  }
}


uint8_t checkTaskRun()
{
  uint8_t ret = 0;
  if(xSemaphoreTake( mutexTaskRunTime_ble,(TickType_t)10)==pdTRUE)
  {
    if(millis()-lastTaskRun_ble>10000) ret+=1;
    xSemaphoreGive(mutexTaskRunTime_ble);
  }

  if(xSemaphoreTake( mutexTaskRunTime_alarmrules,(TickType_t)10)==pdTRUE)
  {
    if(millis()-lastTaskRun_alarmrules>3000) ret+=2;
    xSemaphoreGive(mutexTaskRunTime_alarmrules);
  }
  
  if(xSemaphoreTake( mutexTaskRunTime_ow,(TickType_t)10)==pdTRUE)
  {
    if(millis()-lastTaskRun_onewire>2000) ret+=4;
    xSemaphoreGive(mutexTaskRunTime_ow);
  }
  
  if(xSemaphoreTake( mutexTaskRunTime_can,(TickType_t)10)==pdTRUE)
  {
    if(millis()-lastTaskRuncanbusTx>2000) ret+=8;
    xSemaphoreGive(mutexTaskRunTime_can);
  }
  
  if(xSemaphoreTake( mutexTaskRunTime_serial,(TickType_t)10)==pdTRUE)
  {
    if(millis()-lastTaskRun_bscSerial>3000) ret+=16;
    xSemaphoreGive(mutexTaskRunTime_serial);
  }
  
  if(xSemaphoreTake( mutexTaskRunTime_i2c,(TickType_t)10)==pdTRUE)
  {
    if(millis()-lastTaskRun_i2c>4000) ret+=32;
    xSemaphoreGive(mutexTaskRunTime_i2c);
  }

  if(xSemaphoreTake( mutexTaskRunTime_wifiConn,(TickType_t)10)==pdTRUE)
  {
    if(millis()-lastTaskRun_wifiConn>4000) ret+=64;
    xSemaphoreGive(mutexTaskRunTime_wifiConn);
  }

  return ret;
}


void setup()
{
  //Register setzen
  *((volatile uint32_t *) (RTC_CNTL_SDIO_CONF_REG)) |= RTC_CNTL_SDIO_TIEH; 

  mutexTaskRunTime_ble = xSemaphoreCreateMutex();
  mutexTaskRunTime_alarmrules = xSemaphoreCreateMutex();
  mutexTaskRunTime_ow = xSemaphoreCreateMutex();
  mutexTaskRunTime_can = xSemaphoreCreateMutex();
  mutexTaskRunTime_serial = xSemaphoreCreateMutex();
  mutexTaskRunTime_i2c = xSemaphoreCreateMutex();
  mutexTaskRunTime_wifiConn = xSemaphoreCreateMutex();

  if(bootCounter!=0xFF) bootCounter++;
  isBoot = true;

  //Serielle Debugausgabe
  debugInit();

  BSC_LOGI(TAG, "BSC %s", BSC_SW_VERSION);
  BSC_LOGI(TAG, "bootCounter=%i", bootCounter);
  //readCoreDump();

  //init DIO 
  if(bootCounter==1) initDio(false);
  else initDio(true);
  BSC_LOGI(TAG, "HW: %i", getHwVersion());

  bmsDataInit();

  //init WebSettings
  free_dump();  
  webSettingsSystem.initWebSettings(paramSystem, "System", "/WebSettings.conf");
  webSettingsBluetooth.initWebSettings(paramBluetooth, "Bluetooth", "/WebSettings.conf");
  webSettingsBluetooth.setTimerHandlerName("getBtDevices");
  webSettingsSerial.initWebSettings(paramSerial, "Serial", "/WebSettings.conf");
  webSettingsAlarmBt.initWebSettings(paramAlarmBms, "Alarm BMS (BT + Serial)", "/WebSettings.conf");
  webSettingsAlarmTemp.initWebSettings(paramAlarmTemp, "Alarm Temperatur", "/WebSettings.conf");
  webSettingsDitialOut.initWebSettings(paramDigitalOut, "Digitalausg&auml;nge", "/WebSettings.conf");
  webSettingsDitialIn.initWebSettings(paramDigitalIn, "Digitaleing&auml;nge", "/WebSettings.conf");
  webSettingsOnewire.initWebSettings(paramOnewireAdr, "Onewire", "/WebSettings.conf");
  webSettingsOnewire.setTimerHandlerName("getOwDevices",2000);
  webSettingsOnewire2.initWebSettings(paramOnewire2, "Onewire II", "/WebSettings.conf");
  webSettingsBmsToInverter.initWebSettings(paramBmsToInverter, "Wechselrichter & Laderegelung", "/WebSettings.conf");
  webSettingsDeviceNeeyBalancer.initWebSettings(paramDeviceNeeyBalancer, "NEEY Balancer", "/WebSettings.conf");
  webSettingsDeviceNeeyBalancer.setTimerHandlerName("getNeeySettingsReadback",2000);
  webSettingsDeviceJbdBms.initWebSettings(paramDeviceJbdBms, "JBD BMS", "/WebSettings.conf");

  //Buttons
  webSettingsSystem.setButtons(BUTTON_1,"Delete Log");
  webSettingsSystem.registerOnButton1(&btnSystemDeleteLog);

  webSettingsDeviceNeeyBalancer.setButtons(BUTTON_1,"Write Data to NEEY");
  webSettingsDeviceNeeyBalancer.registerOnButton1(&btnWriteNeeyData);

  webSettingsDeviceJbdBms.setButtons(BUTTON_1,"Write Data to JBD");
  webSettingsDeviceJbdBms.registerOnButton1(&btnWriteJbdBmsData);
  free_dump();  

  //mqtt
  initMqtt();

  //init WLAN
  BSC_LOGI(TAG, "Init WLAN...");
  WiFi.onEvent(onWiFiEvent);
  WiFi.setAutoReconnect(false);
  xTaskCreatePinnedToCore(task_ble, "ble", 2500, nullptr, 5, &task_handle_ble, CONFIG_BT_NIMBLE_PINNED_TO_CORE);
  xTaskCreatePinnedToCore(task_ConnectWiFi, "wlanConn", 2500, nullptr, 1, &task_handle_wifiConn, 1);
  //connectWiFi();
  
  if (MDNS.begin(hostname))
  {
    BSC_LOGI(TAG, "MDNS responder gestartet");
  }

  //ini WebPages
  server.on("/",handlePage_root);
  server.on("/htmlPageStatus/",handlePage_status);
  server.on("/settings/",handlePage_settings);
  server.on("/settings/alarm/",handlePage_alarm);
  server.on("/settings/schnittstellen/",handlePage_schnittstellen);
  server.on("/bmsSpg/",handle_htmlPageBmsSpg);
  server.on("/settings/devices/", HTTP_GET, []() {server.send(200, "text/html", htmlPageDevices);});
  server.on("/restapi", HTTP_GET, []() {buildJsonRest(&server);});

  server.on("/settings/system/",handle_paramSystem);
  server.on("/settings/bms_can/",handle_paramBmsToInverter);
  server.on("/settings/alarm/alarmTemp/",handle_paramAlarmTemp);
  server.on("/settings/alarm/alarmBt/",handle_paramAlarmBt);
  server.on("/settings/schnittstellen/dout/",handle_paramDigitalOut);
  server.on("/settings/schnittstellen/din/",handle_paramDigitalIn);
  server.on("/settings/schnittstellen/bt/",handle_paramBluetooth);
  server.on("/settings/schnittstellen/serial/",handle_paramSerial);
  server.on("/settings/schnittstellen/ow/",handle_paramOnewireAdr);
  server.on("/settings/schnittstellen/ow2/",handle_paramOnewire2);
    
  server.on("/settings/devices/neeyBalancer/",handle_paramDevicesNeeyBalancer);
  server.on("/settings/devices/neeyBalancer/getNeeySettingsReadback",handle_getNeeySettingsReadback);

  server.on("/settings/devices/jbdBms/",handle_paramDeviceJbdBms);

  server.on("/getData",handle_getData);
  server.on("/settings/schnittstellen/bt/getBtDevices",handle_getBtDevices);
  server.on("/getDashboardData",handle_getDashboardData);
  server.on("/bmsSpg/getBmsSpgData",handle_getBmsSpgData);
  server.on("/owTempLive",handlePage_htmlPageOwTempLive);
  server.on("/getOwTempData",handle_getOwTempData);
  server.on("/settings/schnittstellen/ow/getOwDevices",handle_getOnewireDeviceAdr);

  server.on("/log", HTTP_GET, []() {if(!handleFileRead(&server, "/log.txt")){server.send(404, "text/plain", "FileNotFound");}});
  server.on("/log1", HTTP_GET, []() {if(!handleFileRead(&server, "/log1.txt")){server.send(404, "text/plain", "FileNotFound");}});
  server.on("/param", HTTP_GET, []() {if(!handleFileRead(&server, "/WebSettings.conf")){server.send(404, "text/plain", "FileNotFound");}});

  //server.on("/websettings.js", HTTP_GET, []() {server.send(200, "text/javascript", webSettingsJs);});
  //server.on("/websettings.css", HTTP_GET, []() {server.send(200, "text/css", webSettingsCss);});

  webota.init(&server, "/settings/webota/"); //webota

  server.on("/restart/", []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", "<HTML><BODY>Reboot ok<br><a href='../'>Home</a></BODY></HTML>");
    sleep(2);
    ESP.restart();
  });

  //Erstelle Tasks
  xTaskCreatePinnedToCore(task_onewire, "ow", 2500, nullptr, 5, &task_handle_onewire, 1);
  xTaskCreatePinnedToCore(task_bscSerial, "serial", 2500, nullptr, 5, &task_handle_bscSerial, 1);
  xTaskCreatePinnedToCore(task_alarmRules, "alarmrules", 2500, nullptr, configMAX_PRIORITIES - 5, &task_handle_alarmrules, 1);
  xTaskCreatePinnedToCore(task_canbusTx, "can", 2700, nullptr, 5, &task_handle_canbusTx, 1);
  xTaskCreatePinnedToCore(task_i2c, "i2c", 2500, nullptr, 5, &task_handle_i2c, 1);

  free_dump();  


  /*delay(1000);
  Serial.println("RESTART ::");
  int a = 0;
  int b = 4;
  Serial.printf("%d\n", b / a);*/
}


uint8_t u8_lTaskRunSate;
void loop()
{
  #ifdef DEBUG_ON_FS
  writeLogToFS();
  #endif

  if(WlanStaApOk!=WIFI_OFF)
  {
    server.handleClient();
  }

  //Taskrun state
  u8_lTaskRunSate=checkTaskRun();
  if(u8_mTaskRunSate!=u8_lTaskRunSate)
  {
    BSC_LOGI(TAG,"TaskRunSate=%i",u8_lTaskRunSate);
    u8_mTaskRunSate=u8_lTaskRunSate;
  }
}
