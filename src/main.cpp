// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


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

static const char *TAG = "MAIN";

WebServer server;
BleHandler* bleHanlder;

//Serial
BscSerial bscSerial1(0,1,SERIAL1_PIN_RX,SERIAL1_PIN_TX,SERIAL1_PIN_TX_EN);   // Hw Serial 1
BscSerial bscSerial2(1,2,SERIAL2_PIN_RX,SERIAL2_PIN_TX,SERIAL2_PIN_TX_EN);   // Hw Serial 2
#ifndef DEBUG_ON_HW_SERIAL
BscSerial bscSerial3(2,0,SERIAL3_PIN_RX,SERIAL3_PIN_TX,SERIAL3_PIN_TX_EN);   // Hw Serial 0
#else
BscSerial bscSerial3(2,SERIAL3_PIN_RX,SERIAL3_PIN_TX,SERIAL3_PIN_TX_EN);   // Sw Serial
#endif

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

//Timer
TimerHandle_t wifiReconnectTimer;

//Tasks
TaskHandle_t task_handle_ble = NULL;
TaskHandle_t task_handle_alarmrules = NULL;
TaskHandle_t task_handle_onewire = NULL;
TaskHandle_t task_handle_canbusTx = NULL;
TaskHandle_t task_handle_bscSerial = NULL;
TaskHandle_t task_handle_i2c = NULL;
TaskHandle_t task_handle_wlanConn = NULL;

//Task semaphore
static SemaphoreHandle_t mutexTaskRunTime = NULL;

//millis des letzten runs
uint32_t lastTaskRun_ble = 0;
uint32_t lastTaskRun_alarmrules = 0;
uint32_t lastTaskRun_onewire = 0;
uint32_t lastTaskRuncanbusTx = 0;
uint32_t lastTaskRun_bscSerial = 0;
uint32_t lastTaskRun_i2c = 0;

RTC_DATA_ATTR static uint8_t bootCounter = 0;

unsigned long currentMillis;
unsigned long previousMillis10000;

uint8_t u8_mTaskRunSate=0;   //Status ob alle Tasks laufen
bool    isBoot=true;
bool    doConnectWiFi=false;     //true, wenn gerade versucht wird eine Verbindung aufzubauen
bool    firstWlanModeSTA=false;  //true, wenn der erste WLAN-Mode nach einem Neustart STA ist
bool    WlanStaApOk=false;       //true, wenn Wlan verbunden oder AP erstellt

bool    changeWlanDataForI2C=false; //true, wenn sich die WLAN Verbindung geändert hat

//
void task_ble(void *param);
boolean connectWiFi();


void free_dump()
{
  ESP_LOGI(TAG, "Free Heap: %i", ESP.getFreeHeap());
}



static bool wlanEventStaDisonnect=false;
static bool wlanEventStaConnect=false;

void onWiFiEvent(WiFiEvent_t event)
{
    ESP_LOGI(TAG, "[WiFi] event: %d", event);
    switch(event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wlanEventStaDisonnect=false;
      wlanEventStaConnect=true;
      break;
        
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      wlanEventStaConnect=false;
      wlanEventStaDisonnect=true;
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      WlanStaApOk=true;
      bleHanlder->init();
      break;
    }
}


boolean connectWiFi()
{
  doConnectWiFi=true;
  boolean connected = false;
  static String ssid = webSettingsSystem.getString(ID_PARAM_WLAN_SSID,0,0,0);
  static String pwd  = webSettingsSystem.getString(ID_PARAM_WLAN_PWD,0,0,0);
  
  #ifdef WLAN_DEBUG
  ESP_LOGI(TAG, "[WiFi] status (a): %i", WiFi.status());
  #endif

  if(!ssid.equals("") && !pwd.equals(""))
  {
    ESP_LOGI(TAG, "Verbindung zu %s",ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pwd.c_str());
    uint8_t cnt = 0;
    while ((WiFi.status() != WL_CONNECTED) && (cnt<30))
    {
      vTaskDelay(pdMS_TO_TICKS(1000));
      cnt++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      ESP_LOGI(TAG, "IP-Adresse = %s",WiFi.localIP().toString().c_str());
      connected=true;
      firstWlanModeSTA=true;
      changeWlanDataForI2C=true;
    }
  }
  
  if (!connected && !firstWlanModeSTA) {
    //wifiApMode = true;
    ESP_LOGI(TAG, "Wifi AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("BSC","",1);  
    changeWlanDataForI2C=true;
  }
  
  #ifdef WLAN_DEBUG
  ESP_LOGI(TAG, "[WiFi] status (b): %i", WiFi.status());
  #endif
  doConnectWiFi=false;
  return connected;
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

  unsigned long tConnWifiHelpTimer=0;

  ESP_LOGD(TAG, "-> 'task_ConnectWiFi' runs on core %d", xPortGetCoreID());
  ESP_LOGD(TAG, "mConnectState=%i", mConnectStateEnums);

  tConnWifiHelpTimer=millis();

  for(;;)
  {
    switch(mConnectStateEnums)
    {
      case ConnState_wifiDisconnected:
        WlanStaApOk=false;
        mqttDisconnect();
        server.stop(); //Webserver beenden
        bleHanlder->stop();
        mConnectStateEnums=ConnState_noWifiConnection;
        break;

      case ConnState_noWifiConnection: //kein Wlan
        //if(!wifiApMode)
        connectWiFi();
        if(WiFi.getMode()==WIFI_MODE_AP) mConnectStateEnums=ConnState_wlanApMode;
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
            mConnectStateEnums=ConnState_noWifiConnection;
          }
          u8_mWaitConnCounter++;
        }
        break;

      case ConnState_wifiConnected: //Wlan verbunden
        WlanStaApOk=true;
        server.begin(WEBSERVER_PORT);  //Webserver starten
        ////initTime();
        
        mConnectStateEnums=ConnState_connectMQTTstart;
        break;

      case ConnState_wlanApMode:
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
          ESP_LOGI(TAG,"No Mqtt connection!");
          connectMqttTimer=millis();
          mConnectStateEnums=ConnState_connectBT;
          break;
        }

        if(wlanEventStaDisonnect==true)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }

        if(webSettingsSystem.getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
        {
          if(mqttConnect())
          {
            mConnectStateEnums=ConnState_connectBT;
          }
        }
        else
        {
          mConnectStateEnums=ConnState_connectBT;
        }
        break;

      case ConnState_connectBT: //BT verbinden
        if(wlanEventStaDisonnect==true)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }

        bleHanlder->start();
        mConnectStateEnums=ConnState_connectBT2;
        break;

      case ConnState_connectBT2: //BT verbinden
        //ToDo: Timeout einbauen

        if(wlanEventStaDisonnect==true)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }

        if(bleHanlder->isNotAllDeviceConnectedOrScanRunning()==false)mConnectStateEnums=ConnState_idle;
        break;

      case ConnState_idle: //Idle; Wenn alle Verbindungen stehen
        if(wlanEventStaDisonnect==true)
        {
          wlanEventStaDisonnect=false;
          mConnectStateEnums=ConnState_wifiDisconnected;
          break;
        }
        //Wenn MQTT Disconnected -> mConnectStateEnums=ConnState_connectMQTT
        else if(!mqttLoop())
        {
          mConnectStateEnums=ConnState_connectMQTT;
          break;
        }

        break;

      default:
        ESP_LOGE(TAG, "Error: mConnectState=%i", mConnectStateEnums);
        mConnectStateEnums=ConnState_wifiDisconnected;
    }

    if(mConnectStateEnums!=mConnectStateEnumsOld)
    {
      ESP_LOGD(TAG, "mConnectState=%i", mConnectStateEnums);
      mConnectStateEnumsOld=mConnectStateEnums;
    }
    
    if(millis()>tConnWifiHelpTimer+1000)
    {
      tConnWifiHelpTimer=millis();
      ESP_LOGD(TAG, "FreeHeap=%i, MinFreeHeap=%i, mqttTxBuffSize=%i", xPortGetFreeHeapSize(),xPortGetMinimumEverFreeHeapSize(),getTxBufferSize());
    }

    //Wenn nicht im Idle, dann 1000ms warten
    if(mConnectStateEnums!=ConnState_idle)vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void task_ble(void *param)
{
  ESP_LOGD(TAG, "-> 'task_ble' runs on core %d", xPortGetCoreID());

  //init Bluetooth
  ESP_LOGI(TAG, "Init BLE...");
  bleHanlder = new BleHandler();
  bleHanlder->init();
  ESP_LOGI(TAG, "Init BLE...ok");

  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(2000));
    if(doConnectWiFi==false) break;
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    if(bleHanlder!=nullptr) if(WlanStaApOk) bleHanlder->run();
    
    xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
    lastTaskRun_ble=millis();
    xSemaphoreGive(mutexTaskRunTime);
  }
}

void task_alarmRules(void *param)
{
  ESP_LOGD(TAG, "-> 'task_alarmRules' runs on core %d", xPortGetCoreID());

  initAlarmRules();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    runAlarmRules();
    xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
    lastTaskRun_alarmrules=millis();
    xSemaphoreGive(mutexTaskRunTime);
  }
}

void task_onewire(void *param)
{
  ESP_LOGD(TAG, "-> 'task_onewire' runs on core %d", xPortGetCoreID());

  //init Onewire
  owSetup();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    owCyclicRun();
    xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
    lastTaskRun_onewire=millis();
    xSemaphoreGive(mutexTaskRunTime);
  }
}

void task_canbusTx(void *param)
{
  ESP_LOGD(TAG, "-> 'task_canbusTx' runs on core %d", xPortGetCoreID());

  canSetup();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    canTxCyclicRun();
    xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
    lastTaskRuncanbusTx=millis();
    xSemaphoreGive(mutexTaskRunTime);
  }
}

void task_bscSerial(void *param)
{
  ESP_LOGD(TAG, "-> 'task_bscSerial' runs on core %d", xPortGetCoreID());

  //init Serial
  bscSerial1.initSerial();
  bscSerial2.initSerial();
  bscSerial3.initSerial();

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    bscSerial1.cyclicRun();
    bscSerial2.cyclicRun();
    bscSerial3.cyclicRun();
    xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
    lastTaskRun_bscSerial=millis();
    xSemaphoreGive(mutexTaskRunTime);
  }
}

void task_i2c(void *param)
{
  ESP_LOGD(TAG, "-> 'task_i2c' runs on core %d", xPortGetCoreID());

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
      i2cSendData(BSC_DATA, BSC_IP_ADDR, 0, ipAddr, 16);
    }
      
    xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
    lastTaskRun_i2c=millis();
    xSemaphoreGive(mutexTaskRunTime);
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
    bscSerial1.setReadBmsFunktion(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,0,0));
    bscSerial2.setReadBmsFunktion(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,1,0));
    bscSerial3.setReadBmsFunktion(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,2,0));
    changeAlarmSettings();
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

    if(webSettingsSystem.getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
    {
      mqttConnect();
    }
    else
    {
      mqttDisconnect();
    }
  }
}

        

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
  
  //5. BT-Devices
  tmp += "|";
  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleHanlder!=nullptr) tmp += bleHanlder->bmsIsConnect(i);
    if(i<BT_DEVICES_COUNT) tmp += "&nbsp;";
    if(i==4) tmp += "|";
  }

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
  if(bleHanlder!=nullptr)
  {
  bleHanlder->startScan(); //BT Scan starten
  server.send(200, "text/html", bleHanlder->getBtScanResult().c_str());
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
  ESP_LOGI(TAG, "Logfiles deleted");
}


uint8_t checkTaskRun()
{
  uint8_t ret = 0;
  xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
  if(millis()-lastTaskRun_ble>10000) ret+=1;
  if(millis()-lastTaskRun_alarmrules>3000) ret+=2;
  if(millis()-lastTaskRun_onewire>2000) ret+=4;
  if(millis()-lastTaskRuncanbusTx>2000) ret+=8;
  if(millis()-lastTaskRun_bscSerial>3000) ret+=16;
  if(millis()-lastTaskRun_i2c>4000) ret+=32;
  xSemaphoreGive(mutexTaskRunTime);
  return ret;
}


void setup()
{
  mutexTaskRunTime = xSemaphoreCreateMutex();
  if(bootCounter!=0xFF) bootCounter++;
  isBoot = true;

  //Serielle Debugausgabe
  debugInit();

  ESP_LOGI(TAG, "BSC %s", BSC_SW_VERSION);
  ESP_LOGI(TAG, "bootCounter=%i", bootCounter);

  //init DIO 
  if(bootCounter==1) initDio(false);
  else initDio(true);
  ESP_LOGI(TAG, "HW: %i", getHwVersion());

  bmsDataInit();

  //init WebSettings
  webSettingsSystem.initWebSettings(paramSystem, "System", "/WebSettings.conf",0);
  webSettingsBluetooth.initWebSettings(paramBluetooth, "Bluetooth", "/WebSettings.conf",0);
  webSettingsBluetooth.setTimerHandlerName("getBtDevices");
  webSettingsSerial.initWebSettings(paramSerial, "Serial", "/WebSettings.conf",0);
  webSettingsAlarmBt.initWebSettings(paramAlarmBms, "Alarm BMS (BT + Serial)", "/WebSettings.conf",0);
  webSettingsAlarmTemp.initWebSettings(paramAlarmTemp, "Alarm Temperatur", "/WebSettings.conf",0);
  webSettingsDitialOut.initWebSettings(paramDigitalOut, "Digitalausg&auml;nge", "/WebSettings.conf",0);
  webSettingsDitialIn.initWebSettings(paramDigitalIn, "Digitaleing&auml;nge", "/WebSettings.conf",0);
  webSettingsOnewire.initWebSettings(paramOnewireAdr, "Onewire", "/WebSettings.conf",0);
  webSettingsOnewire.setTimerHandlerName("getOwDevices",2000);
  webSettingsOnewire2.initWebSettings(paramOnewire2, "Onewire II", "/WebSettings.conf",0);
  webSettingsBmsToInverter.initWebSettings(paramBmsToInverter, "Wechselrichter & Laderegelung", "/WebSettings.conf",0);

  webSettingsSystem.setButtons(BUTTON_1,"Delete Log");
  webSettingsSystem.registerOnButton1(&btnSystemDeleteLog);

  //mqtt
  initMqtt();

  //init WLAN
  ESP_LOGI(TAG, "Init WLAN...");
  WiFi.onEvent(onWiFiEvent);
  WiFi.setAutoReconnect(false);
  connectWiFi();
  ESP_LOGI(TAG, "Init WLAN...ok");
  
  char dns[30] = {'b','s','c'};
  if (MDNS.begin(dns))
  {
    ESP_LOGI(TAG, "MDNS responder gestartet");
  }

  //ini WebPages
  server.on("/",handlePage_root);
  server.on("/htmlPageStatus/",handlePage_status);
  server.on("/settings/",handlePage_settings);
  server.on("/settings/alarm/",handlePage_alarm);
  server.on("/settings/schnittstellen/",handlePage_schnittstellen);
  server.on("/bmsSpg/",handle_htmlPageBmsSpg);

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

  webota.init(&server, "/settings/webota/"); //webota

  server.on("/restart/", []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", "<HTML><BODY>Reboot ok<br><a href='../'>Home</a></BODY></HTML>");
    sleep(2);
    ESP.restart();
  });

  //starte webserver
  ESP_LOGI(TAG, "Starte Webserver...");
  server.begin(WEBSERVER_PORT);
  ESP_LOGI(TAG, "Starte Webserver...ok");

  //Erstelle Tasks
  xTaskCreatePinnedToCore(task_ble, "ble", 3000, nullptr, 5, &task_handle_ble, CONFIG_BT_NIMBLE_PINNED_TO_CORE);
  xTaskCreatePinnedToCore(task_onewire, "ow", 3100, nullptr, 5, &task_handle_onewire, 1);
  xTaskCreatePinnedToCore(task_bscSerial, "serial", 3000, nullptr, 5, &task_handle_bscSerial, 1);
  xTaskCreatePinnedToCore(task_alarmRules, "alarmrules", 2700, nullptr, configMAX_PRIORITIES - 5, &task_handle_alarmrules, 1);
  xTaskCreatePinnedToCore(task_canbusTx, "can", 2700, nullptr, 5, &task_handle_canbusTx, 1);
  xTaskCreatePinnedToCore(task_i2c, "i2c", 2500, nullptr, 5, &task_handle_i2c, 1);
  xTaskCreatePinnedToCore(task_ConnectWiFi, "wlanConn", 2500, nullptr, 1, &task_handle_wlanConn, 1);

 
  //uint32_t chipid = (uint32_t)ESP.getEfuseMac();


  previousMillis10000=millis();
  free_dump();  

  initTime();
  timeRunCyclic(); //Hole 1x die Zeit
  ESP_LOGI(TAG,"Time: %s",getBscDateTime().c_str());
}

unsigned long testtimer;
uint8_t u8_lTaskRunSate;
void loop()
{
  if(WlanStaApOk)
  {
    server.handleClient();
    //timeRunCyclic();
  }

  //Taskrun state
  u8_lTaskRunSate=checkTaskRun();
  if(u8_mTaskRunSate!=u8_lTaskRunSate)
  {
    ESP_LOGI(TAG,"TaskRunSate=%i",u8_lTaskRunSate);
    u8_mTaskRunSate=u8_lTaskRunSate;
  }

  //10s Intervall
  currentMillis = millis();
  if(currentMillis-previousMillis10000>=10000)
  {    
    //Sende Daten via mqqtt, wenn aktiv
    if(WlanStaApOk && WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
    {      
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_ESP32_TEMP, -1, temperatureRead());
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_CURRENT_SOLL, -1, getAktualChargeCurrentSoll());
      
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_FREE_HEAP, -1, xPortGetFreeHeapSize());
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_MIN_FREE_HEAP, -1, xPortGetMinimumEverFreeHeapSize());
      
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_HIGHWATER_TASK_BLE, -1, uxTaskGetStackHighWaterMark(task_handle_ble));
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_HIGHWATER_TASK_ALARMRULES, -1, uxTaskGetStackHighWaterMark(task_handle_alarmrules));
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_HIGHWATER_TASK_OW, -1, uxTaskGetStackHighWaterMark(task_handle_onewire));
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_HIGHWATER_TASK_CAN, -1, uxTaskGetStackHighWaterMark(task_handle_canbusTx));
      mqttPublish(MQTT_TOPIC_SYS, -1, MQTT_TOPIC2_HIGHWATER_TASK_SERIAL, -1, uxTaskGetStackHighWaterMark(task_handle_bscSerial));
    }

    previousMillis10000 = currentMillis;
  }


  /*if(millis()-testtimer>=60000)
  {
    ESP_LOGI(TAG,"Reconnect!");
    testtimer=millis();
    mConnectStateEnums=ConnState_wlanDisconnecting;
  }*/
  
}