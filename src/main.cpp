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

//#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <OTAupdater.h>
#include <FS.h>
#ifdef USE_LittleFS
  #define SPIFFS LittleFS
  #include <LittleFS.h>
#else
  #include <SPIFFS.h>
#endif

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
#ifdef BPN
#include "devices/bpnWebHandler.h"
#endif

#ifdef INSIDER_V1
#include "webapp2.hpp"
#endif

extern "C"
{
  #include "esp_core_dump.h"
}

static const char *TAG = "MAIN";

WebServer server;
BleHandler bleHandler;
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
uint32_t   u32_getTimeTimer;            // Timer um regelmäig die Zeit zu holen

bool       changeWlanDataForI2C=false;  //true, wenn sich die WLAN Verbindung geändert hat

// Variablen für connectWifi()
static boolean bo_mWifiConnected;
static String str_lWlanSsid;
static String str_lWlanPwd;
static uint16_t u16_lWlanConnTimeout;
static boolean bo_lWlanNeverAp;
static unsigned long wlanConnectTimer;

// Zeitstempel vom letzten Boot
static boolean bo_BootTimeStamp = false;
static String str_BootTimeStamp;

#ifdef LOG_BMS_DATA
long debugLogTimer;
#endif

//Für Tests
//uint32_t loopDuration=0;


//
void task_ble(void *param);
boolean connectWiFi();

void free_dump()
{
  BSC_LOGI(TAG, "Free Heap: %i", ESP.getFreeHeap());
}

/*void bscCoreDumpExport()
{
  if (esp_core_dump_image_check() == ESP_OK)
  {
    esp_core_dump_summary_t *summary = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
    if (summary)
    {
      if (esp_core_dump_get_summary(summary) == ESP_OK)
      {
        char outputString[16];

        //core["exc_task"] = summary->exc_task;
        BSC_LOGI(TAG, "exc_task %s", summary->exc_task);
        //core["app_elf_sha256"] = (char *)summary->app_elf_sha256;
        BSC_LOGI(TAG, "app_elf_sha256 %s", (char *)summary->app_elf_sha256);
        //core["dumpver"] = summary->core_dump_version;
        BSC_LOGI(TAG, "dumpver %i", summary->core_dump_version);

        ultoa(summary->exc_pc, outputString, 16);
        //core["exc_pc"] = outputString;
        BSC_LOGI(TAG, "exc_pc %s", outputString);

        ultoa(summary->exc_tcb, outputString, 16);
        //core["exc_tcb"] = outputString;
        BSC_LOGI(TAG, "exc_tcb %s", outputString);

        //core["bt_corrupted"] = summary->exc_bt_info.corrupted;
        BSC_LOGI(TAG, "bt_corrupted %d", summary->exc_bt_info.corrupted);
        //core["bt_depth"] = summary->exc_bt_info.depth;
        BSC_LOGI(TAG, "bt_depth %i", summary->exc_bt_info.depth);
        //auto backtrace = core.createNestedArray("backtrace");
        for (auto value : summary->exc_bt_info.bt)
        {
          ultoa(value, outputString, 16);
          //backtrace.add(outputString);
          BSC_LOGI(TAG, "backtrace %s", outputString);
        }

        ltoa(summary->ex_info.epcx_reg_bits, outputString, 2);
        //core["epcx_reg_bits"] = outputString;
        BSC_LOGI(TAG, "epcx_reg_bits %s", outputString);
        ltoa(summary->ex_info.exc_cause, outputString, 16);
        //core["exc_cause"] = outputString;
        BSC_LOGI(TAG, "exc_cause %s", outputString);
        ultoa(summary->ex_info.exc_vaddr, outputString, 16);
        //core["exc_vaddr"] = outputString;
        BSC_LOGI(TAG, "exc_vaddr %s", outputString);

        //auto exc_a = core.createNestedArray("exc_a");
        for (auto value : summary->ex_info.exc_a)
        {
          ultoa(value, outputString, 16);
          //exc_a.add(outputString);
          BSC_LOGI(TAG, "exc_a %s", outputString);
        }
        //auto epcx = core.createNestedArray("epcx");
        for (auto value : summary->ex_info.epcx)
        {
          ultoa(value, outputString, 16);
          //epcx.add(outputString);
          BSC_LOGI(TAG, "epcx %s", outputString);
        }

        esp_core_dump_image_erase();
      }
    }
    free(summary);
  }
}*/

/*void readCoreDump()
{
  size_t size = 0;
  size_t address = 0;
  esp_err_t ret = esp_core_dump_image_get(&address, &size);
  BSC_LOGI("ESP32", "esp_core_dump_image_get() ret=%i, %s, size=%i", ret, esp_err_to_name(ret), size);
  if (ret == ESP_OK)
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
          //Serial.printf("FAIL [%x]\n",er);
          BSC_LOGE("ESP32", "FAIL [%x]", er);
          break;
        }

        for (int16_t j = 0; j < 256; j++)
        {
          char str_tmp[3];

          sprintf(str_tmp, "%02x", bf[j]);
          strcat(str_dst, str_tmp);
        }

        //printf("%s", str_dst);
        BSC_LOGE(TAG, "%s", str_dst);
      }
    }
    else
    {
      //Serial.println("Partition NULL");
      BSC_LOGE("ESP32", "Partition NULL");
    }
    esp_core_dump_image_erase();
  }
  else
  {
    //Serial.println("esp_core_dump_image_get() FAIL");
    BSC_LOGE("ESP32", "esp_core_dump_image_get() FAIL errno=%i, %s, size=%i", ret, esp_err_to_name(ret), size);
  }
}*/


void onWiFiEvent(WiFiEvent_t event)
{
  switch(event)
  {
    case 	ARDUINO_EVENT_WIFI_READY:
      BSC_LOGI(TAG, "WIFI ready (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_SCAN_DONE:
      BSC_LOGI(TAG, "WIFI scan done (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_START:
      BSC_LOGI(TAG, "WIFI STA start (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_STOP:
      BSC_LOGI(TAG, "WIFI STA stop (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_CONNECTED:
      BSC_LOGI(TAG, "WIFI STA connected (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      BSC_LOGI(TAG, "WIFI STA disconnected (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      BSC_LOGI(TAG, "WIFI STA authmode change (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_GOT_IP:
      BSC_LOGI(TAG, "WIFI STA got IP (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_GOT_IP6:
      BSC_LOGI(TAG, "WIFI STA got IP6 (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_STA_LOST_IP:
      BSC_LOGI(TAG, "WIFI STA lost IP(%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_AP_START:
      BSC_LOGI(TAG, "WIFI AP start (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_AP_STOP:
      BSC_LOGI(TAG, "WIFI AP stop (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
      BSC_LOGI(TAG, "WIFI AP PROBEREQRECVED (%i)",event);
      break;
	  case 	ARDUINO_EVENT_WIFI_AP_GOT_IP6:
      BSC_LOGI(TAG, "WIFI AP got IP6 (%i)",event);
      break;
    default:
      BSC_LOGI(TAG, "WIFI event: %d", event);
  }

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
  bo_lWlanNeverAp=false;
  str_lWlanSsid = webSettingsSystem.getString(ID_PARAM_WLAN_SSID,0);
  str_lWlanPwd  = webSettingsSystem.getString(ID_PARAM_WLAN_PWD,0);
  u16_lWlanConnTimeout  = webSettingsSystem.getInt(ID_PARAM_WLAN_CONNECT_TIMEOUT,0,DT_ID_PARAM_WLAN_CONNECT_TIMEOUT);

  // Wenn eine statische IP festgelegt wurde
  IPAddress lWlanIpAdresse;
  if(lWlanIpAdresse.fromString(webSettingsSystem.getStringFlash(ID_PARAM_WLAN_IP_ADRESSE,0)))
  {
    IPAddress lWlanGateway;
    IPAddress lWlanSubnet;
    IPAddress lWlanDns;
    if(lWlanGateway.fromString(webSettingsSystem.getStringFlash(ID_PARAM_WLAN_GATEWAY,0)))
    {
      if(lWlanSubnet.fromString(webSettingsSystem.getStringFlash(ID_PARAM_WLAN_SUBNET,0)))
      {
        if(lWlanDns.fromString(webSettingsSystem.getStringFlash(ID_PARAM_WLAN_DNS,0)))
          WiFi.config(lWlanIpAdresse,lWlanGateway,lWlanSubnet,lWlanDns);
        else WiFi.config(lWlanIpAdresse,lWlanGateway,lWlanSubnet);
        BSC_LOGI(TAG, "Static IP: %s, Gateway: %s, Subnet: %s",lWlanIpAdresse.toString().c_str(),lWlanGateway.toString().c_str(),lWlanSubnet.toString().c_str());
      }
    }
  }

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
    WiFi.setHostname(WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0).c_str());
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
    BSC_LOGI(TAG, "Open Wifi AP");
    WlanStaApOk=WIFI_AP;
    WiFi.mode(WIFI_AP);
    String str_lHostname = WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0);
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

  unsigned long tConnWifiHelpTimer=0;
  tConnWifiHelpTimer=millis();

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
        bleHandler.stop();
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

        // Zeitstempel der ersten Zeit nach dem Booten ermitteln
        if(!bo_BootTimeStamp)
        {
          str_BootTimeStamp=getBscDateTime().c_str();
          bo_BootTimeStamp=true;
          //BSC_LOGI(TAG,"Boottime: %s",str_BootTimeStamp.c_str());
        }

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
          #ifdef MAIN_DEBUG
          BSC_LOGI(TAG,"No Mqtt connection!");
          #endif
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

        bleHandler.start();
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

        if(bleHandler.isNotAllDeviceConnectedOrScanRunning()==false)mConnectStateEnums=ConnState_idle;
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

          if(millis()-u32_getTimeTimer>3600000)
          {
            u32_getTimeTimer=millis();
            timeRunCyclic(true);
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
  bleHandler.init();
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

    if(WlanStaApOk!=WIFI_OFF) bleHandler.run();

    xSemaphoreTake(mutexTaskRunTime_ble, portMAX_DELAY);
    lastTaskRun_ble=millis();
    xSemaphoreGive(mutexTaskRunTime_ble);
  }
}

void task_alarmRules(void *param)
{
  BSC_LOGD(TAG, "-> 'task_alarmRules' runs on core %d", xPortGetCoreID());

  vTaskDelay(pdMS_TO_TICKS(15000));
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
    vTaskDelay(pdMS_TO_TICKS(1200));
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


#ifdef UTEST_FS
void task_fsTest1(void *param)
{
  BSC_LOGD(TAG, "-> 'task_fsTest1' runs on core %d", xPortGetCoreID());
  uint32_t fsTestCounter = 0;
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    fsTestCounter++;
    BSC_LOGI(TAG,"Task FSTEST1 cnt=%i",fsTestCounter);
  }
}

void task_fsTest2(void *param)
{
  BSC_LOGD(TAG, "-> 'task_fsTest2' runs on core %d", xPortGetCoreID());
  uint32_t fsTestCounter = 0;
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    fsTestCounter++;
    BSC_LOGI(TAG,"Task FSTEST2 cnt=%i",fsTestCounter);
  }
}

void task_fsTest3(void *param)
{
  BSC_LOGD(TAG, "-> 'task_fsTest3' runs on core %d", xPortGetCoreID());
  uint32_t fsTestCounter = 0;
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(100));
    logTrigger(fsTestCounter,0,true);
    fsTestCounter++;
    if(fsTestCounter==10)fsTestCounter=0;
  }
}
#endif


/*
  Handle WebPages
*/
void handlePage_root(){server.send(200, "text/html", htmlPageRoot);}
void handlePage_settings(){server.send(200, "text/html", htmlPageSettings);}
void handlePage_alarm(){server.send(200, "text/html", htmlPageAlarm);}
void handlePage_schnittstellen(){server.send(200, "text/html", htmlPageSchnittstellen);}
void handle_htmlPageBmsSpg(){server.send(200, "text/html", htmlPageBmsSpg);}
void handlePage_status(){server.send(200, "text/html", htmlPageStatus);}
void handlePage_htmlPageMenuLivedata(){server.send(200, "text/html", htmlPageMenuLivedata);}
void handlePage_htmlPageOwTempLive(){server.send(200, "text/html", htmlPageOwTempLive);}
void handlePage_htmlPageBscDataLive(){server.send(200, "text/html", htmlPageBscDataLive);}

/*#ifdef BPN
void handlePage_htmlPageBpnSettings()
{
  server.send(200, "text/html", htmlPageBpnSettings);
  bpnHandleHtmlFormRequest(&server,true);
}
#endif*/

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
    bmsFilterData->u8_mFilterBmsCellVoltagePercent = (uint8_t)WebSettings::getIntFlash(ID_PARAM_BMS_FILTER_CELL_VOLTAGE_PERCENT,0,DT_ID_PARAM_BMS_FILTER_CELL_VOLTAGE_PERCENT);
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
  /*if (server.hasArg("SAVE"))
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
  }*/
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
  tmp += " / ";
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
    uint8_t u8_lBtDevConnState=bleHandler.bmsIsConnect(i);
    if(u8_lBtDevConnState==0) tmp += "&ndash;";
    else if(u8_lBtDevConnState==1) tmp += "n";
    else if(u8_lBtDevConnState==2) tmp += "c";

    if(i<BT_DEVICES_COUNT) tmp += "&nbsp;";
    if(i==4) tmp += "|";
  }

  //7. WLAN RSSI
  tmp += "|";
  tmp += String(WiFi.RSSI()+100); //0...100 => bad...good

  //8. Boot Time
  tmp += "|";
  tmp += str_BootTimeStamp.c_str();

  //9-11. BMS data (serial 0-2)
  for(uint8_t i=0;i<3;i++)
  {
    tmp += "|";
    if(millis()-getBmsLastDataMillis(BT_DEVICES_COUNT+i)<5000)
    {
      float fl_lBmsTotalVolage = getBmsTotalVoltage(BT_DEVICES_COUNT+i);
      float fl_lBmsTotalCurrent = getBmsTotalCurrent(BT_DEVICES_COUNT+i);
      uint8_t u8_lBmsSoc = getBmsChargePercentage(BT_DEVICES_COUNT+i);
      tmp += String(fl_lBmsTotalVolage) + ";" + String(fl_lBmsTotalCurrent) + ";" + String(u8_lBmsSoc);
    }
    else
    {
      tmp += "--;--;--";
    }
  }

  //12. Hostname
  tmp += "|";
  tmp += WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0);

  //13. BSC SN
  tmp += "|S";
  tmp += getSN();
  tmp += "Z";

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

void handle_getBscLiveData()
{
  buildJsonRest(&server);
}


void handle_getBtDevices()
{
  bleHandler.startScan(); //BT Scan starten
  server.send(200, "text/html", bleHandler.getBtScanResult().c_str());
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
  bleHandler.sendDataToNeey();
}

void btnReadNeeyData()
{
  BSC_LOGI(TAG,"StartReadDataFromNeey");
  bleHandler.readDataFromNeey();
}


void btnWriteJbdBmsData()
{
  /*for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    setSerialBmsWriteData(i,true);
  }*/
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


void sendResponceUpdateBpnFw()
{

  BSC_LOGI(TAG,"Upload ok");
  uint8_t data=0;
  setSerialBmsWriteData(BPN_START_FWUPDATE, &data, 1);
  addSerialBmsWriteDevNr(2);

  server.send(200, "text/plain", "Upload ok");
}


void setup()
{
  //Register setzen
  *((volatile uint32_t *) (RTC_CNTL_SDIO_CONF_REG)) |= RTC_CNTL_SDIO_TIEH;

  //esp_core_dump_init();

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
  //bscCoreDumpExport();

  //init DIO
  if(bootCounter==1) initDio(false);
  else initDio(true);
  BSC_LOGI(TAG, "HW: %i", getHwVersion());

  //esp_task_wdt_init(60,false);
  //esp_task_wdt_add(NULL);

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

  webSettingsDeviceNeeyBalancer.setButtons(BUTTON_1,"Read data from NEEY");
  webSettingsDeviceNeeyBalancer.registerOnButton1(&btnReadNeeyData);
  webSettingsDeviceNeeyBalancer.setButtons(BUTTON_2,"Write data to NEEY");
  webSettingsDeviceNeeyBalancer.registerOnButton2(&btnWriteNeeyData);

  webSettingsDeviceJbdBms.setButtons(BUTTON_1,"Write data to JBD");
  webSettingsDeviceJbdBms.registerOnButton1(&btnWriteJbdBmsData);
  free_dump();

  BSC_LOGI(TAG,"Hostname: %s", WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0).c_str()); //Der Hostname kann erst nach dem lesen der Parameter genutzt werden

  //mqtt
  initMqtt();

  //init WLAN
  BSC_LOGI(TAG, "Init WLAN...");
  WiFi.onEvent(onWiFiEvent);
  WiFi.setAutoReconnect(false);
  xTaskCreatePinnedToCore(task_ble, "ble", 2500, nullptr, 5, &task_handle_ble, CONFIG_BT_NIMBLE_PINNED_TO_CORE);
  xTaskCreatePinnedToCore(task_ConnectWiFi, "wlanConn", 2500, nullptr, 1, &task_handle_wifiConn, 1);


  if (MDNS.begin(WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0).c_str()))
  {
    BSC_LOGI(TAG, "MDNS responder gestartet");
  }

  //
  bool bo_isSupporter = false;
  #ifdef INSIDER_V1
  bo_isSupporter=initWebApp2(&server, &webSettingsSystem, &bleHandler, &bscSerial);
  #endif
  String bscSn = getSN();
  ESP_LOGI(TAG,"BSC SN: %s, %i",bscSn.c_str());

  //ini WebPages
  #ifdef INSIDER_V1
  if(bo_isSupporter)
  {
    server.on("/old",handlePage_root);
    server.on("/support/", HTTP_GET, []() {server.send(200, "text/html", "<HEAD><meta http-equiv=\"refresh\" content=\"0;url=/#/support\"></HEAD>");});

    server.on("/p_system", HTTP_GET, []() {server.send_P(200, "application/json", paramSystem);});
    server.on("/p_bt", HTTP_GET, []() {server.send_P(200, "application/json", paramBluetooth);});
    server.on("/p_serial", HTTP_GET, []() {server.send_P(200, "application/json", paramSerial);});
    server.on("/p_alarmbms", HTTP_GET, []() {server.send_P(200, "application/json", paramAlarmBms);});
    server.on("/p_alarmtemp", HTTP_GET, []() {server.send_P(200, "application/json", paramAlarmTemp);});
    server.on("/p_do", HTTP_GET, []() {server.send_P(200, "application/json", paramDigitalOut);});
    server.on("/p_di", HTTP_GET, []() {server.send_P(200, "application/json", paramDigitalIn);});
    server.on("/p_ow", HTTP_GET, []() {server.send_P(200, "application/json", paramOnewire2);});
    server.on("/p_owadr", HTTP_GET, []() {server.send_P(200, "application/json", paramOnewireAdr);});
    server.on("/p_inverter", HTTP_GET, []() {server.send_P(200, "application/json", paramBmsToInverter);});
    server.on("/p_neey", HTTP_GET, []() {server.send_P(200, "application/json", paramDeviceNeeyBalancer);});
    server.on("/p_jbd", HTTP_GET, []() {server.send_P(200, "application/json", paramDeviceJbdBms);});
    server.on("/p_bpn", HTTP_GET, []() {server.send_P(200, "application/json", paramDeviceBpn);});
  }
  else
  {
    server.on("/",handlePage_root);
    server.on("/support/", HTTP_GET, []() {server.send(200, "text/html", htmlPageSupport);});
  }
  #else
  server.on("/",handlePage_root);
  server.on("/support/", HTTP_GET, []() {server.send(200, "text/html", htmlPageSupport);});
  #endif
  server.on("/favicon.svg", HTTP_GET, []() {server.send(200, "image/svg+xml", htmlFavicon);});

  server.on("/htmlPageStatus/",handlePage_status);
  server.on("/settings/",handlePage_settings);
  server.on("/settings/alarm/",handlePage_alarm);
  server.on("/settings/schnittstellen/",handlePage_schnittstellen);
  server.on("/bmsSpg/",handle_htmlPageBmsSpg);
  server.on("/settings/devices/", HTTP_GET, []() {server.send(200, "text/html", htmlPageDevices);});
  server.on("/restapi", HTTP_GET, []() {buildJsonRest(&server);});
  //server.on("/setParameter", HTTP_POST, []() {handle_setParameter(&server);});

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
  server.on("/livedata/",handlePage_htmlPageMenuLivedata);
  server.on("/livedata/owTempLive/",handlePage_htmlPageOwTempLive);
  server.on("/livedata/owTempLive/getOwTempData",handle_getOwTempData);
  server.on("/livedata/bscDataLive/",handlePage_htmlPageBscDataLive);
  server.on("/livedata/bscDataLive/getBscLiveData",handle_getBscLiveData);
  server.on("/settings/schnittstellen/ow/getOwDevices",handle_getOnewireDeviceAdr);

  //BPN
  #ifdef BPN
  //server.on("/bpn",handlePage_htmlPageBpnSettings);
  //server.on("/getBpnData",[]() {handle_getBpnData(&server);});
  server.on("/upload", HTTP_GET, []() {server.send(200, "text/html", htmlPageUpload);});
  server.on("/uploadPbnFw", HTTP_POST, sendResponceUpdateBpnFw, [](){handleFileUpload(&server, true, "bpnFw.bin");});
  #endif

  server.on("/log", HTTP_GET, []() {if(!handleFileRead(&server, true, "/log.txt")){server.send(404, "text/plain", "FileNotFound");}});
  server.on("/log1", HTTP_GET, []() {if(!handleFileRead(&server, true, "/log1.txt")){server.send(404, "text/plain", "FileNotFound");}});
  server.on("/trigger", HTTP_GET, []() {if(!handleFileRead(&server, true, "/trigger.txt")){server.send(404, "text/plain", "FileNotFound");}});
  server.on("/valueslog", HTTP_GET, []() {if(!handleFileRead(&server, true, "/values")){server.send(404, "text/plain", "FileNotFound");}});
  //server.on("/param", HTTP_GET, []() {if(!handleFileRead(&server, false, "/WebSettings.conf")){server.send(404, "text/plain", "FileNotFound");}});

  otaUpdater.init(&server, "/settings/webota/", true);

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
  #ifdef UTEST_FS
  xTaskCreatePinnedToCore(task_fsTest1, "fstest1", 2500, nullptr, 5, &task_handle_i2c, 1);
  xTaskCreatePinnedToCore(task_fsTest2, "fstest2", 2500, nullptr, 5, &task_handle_i2c, 1);
  xTaskCreatePinnedToCore(task_fsTest3, "fstest3", 2500, nullptr, 5, &task_handle_i2c, 1);
  #endif

  free_dump();

  #ifdef LOG_BMS_DATA
  debugLogTimer=millis();
  #endif

  /*delay(1000);
  ESP_LOGI(TAG, "RESTART");
  delay(1000);
  int a = 0;
  int b = 4;
  Serial.printf("%d\n", b / a);*/
}


uint8_t u8_lTaskRunSate;
//uint32_t loopRunTime=0;
void loop()
{
  //loopDuration=millis()-loopRunTime;
  //loopRunTime=millis();
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
    #ifdef MAIN_DEBUG
    BSC_LOGI(TAG,"TaskRunSate=%i",u8_lTaskRunSate);
    #endif
    u8_mTaskRunSate=u8_lTaskRunSate;
  }

  logValues();

  #ifdef LOG_BMS_DATA
  if(millis()-debugLogTimer>=10000)
  {
    debugLogTimer = millis();
    logBmsData(10); //0-6 BT; 7-9 serial 0-2; 10-17 serial ext.
  }
  #endif
}
