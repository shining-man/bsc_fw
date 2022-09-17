// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WebOTA.h>

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


WebServer server;
BleHandler bleHanlder;
BscSerial bscSerial1(0,1,16,17,18);  // Hw Serial 1
BscSerial bscSerial2(1,2,35,33,32);  // Hw Serial 2
BscSerial bscSerial3(2,23,25);    // Sw Serial

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

//Task semaphore
SemaphoreHandle_t mutexTaskRunTime = NULL;

//millis des letzten runs
uint32_t lastTaskRun_ble = 0;
uint32_t lastTaskRun_alarmrules = 0;
uint32_t lastTaskRun_onewire = 0;
uint32_t lastTaskRuncanbusTx = 0;
uint32_t lastTaskRun_bscSerial = 0;

RTC_DATA_ATTR static uint8_t bootCounter = 0;

unsigned long currentMillis;
unsigned long previousMillis10000;

uint8_t u8_mTaskRunSate=false;     //Status ob alle Tasks laufen
bool    isBoot=true;
bool    wifiApMode=false;        //true, wenn AP Mode
bool    doConnectWiFi=false;     //true, wenn gerade versucht wird eine Verbindung aufzubauen
bool    firstWlanModeSTA=false;  //true, wenn der erste WLAN-Mode nach einem Neustart STA ist


void free_dump() {
  Serial.print(F("Heap free: "));
  Serial.println(ESP.getFreeHeap());
}


void onWiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      //Webserver neu starten
        server.begin(WEBSERVER_PORT); 

        //MQTT verbinden
        if(WiFi.status() == WL_CONNECTED && webSettingsSystem.getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
        {
          mqttConnect();
        }
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        mqttDisconnect();
        server.stop(); //Webserver beenden

        if(!wifiApMode)
        {
          if(!doConnectWiFi) xTimerStart(wifiReconnectTimer, 0);
        }
        else
        {
          xTimerStop(wifiReconnectTimer, 0);
        }
        break;
    }
}


boolean connectWiFi()
{
  doConnectWiFi=true;
  boolean connected = false;
  String ssid = webSettingsSystem.getString(ID_PARAM_WLAN_SSID,0,0,0);
  String pwd  = webSettingsSystem.getString(ID_PARAM_WLAN_PWD,0,0,0);

  if(!ssid.equals("") && !pwd.equals(""))
  {
    Serial.print("Verbindung zu ");
    Serial.print(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pwd.c_str());
    uint8_t cnt = 0;
    while ((WiFi.status() != WL_CONNECTED) && (cnt<20)){
      delay(500);
      Serial.print(":");
      cnt++;
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("IP-Adresse = ");
      Serial.println(WiFi.localIP());
      connected=true;
      firstWlanModeSTA=true;
    }
  }
  
  if (!connected && !firstWlanModeSTA) {
    wifiApMode = true;
    Serial.println("Wifi AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("BSC","",1);  
  }
  
  doConnectWiFi=false;
  return connected;
}


/*
  Handle Tasks
*/
void task_ble(void *param)
{
  for(;;)
  {
  vTaskDelay(pdMS_TO_TICKS(2000));
  if(doConnectWiFi==false) break;
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    bleHanlder.run();
    xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
    lastTaskRun_ble=millis();
    xSemaphoreGive(mutexTaskRunTime);
  }
}

void task_alarmRules(void *param)
{
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


/*
  Handle WebPages
*/
void handlePage_root(){server.send(200, "text/html", htmlPageRoot);}
void handlePage_settings(){server.send(200, "text/html", htmlPageSettings);}
void handlePage_alarm(){server.send(200, "text/html", htmlPageAlarm);}
void handlePage_schnittstellen(){server.send(200, "text/html", htmlPageSchnittstellen);}
void handle_htmlPageBmsSpg(){server.send(200, "text/html", htmlPageBmsSpg);}
void handlePage_status(){server.send(200, "text/html", htmlPageStatus);}


/*
  Handle WebSettings
*/
void handle_paramAlarmBt(){webSettingsAlarmBt.handleHtmlFormRequest(&server);}
void handle_paramAlarmTemp(){webSettingsAlarmTemp.handleHtmlFormRequest(&server);}
void handle_paramDigitalOut(){webSettingsDitialOut.handleHtmlFormRequest(&server);}
void handle_paramDigitalIn(){webSettingsDitialIn.handleHtmlFormRequest(&server);}
void handle_paramBluetooth(){webSettingsBluetooth.handleHtmlFormRequest(&server);}
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
  
  server.send(200, "text/html", tmp.c_str());
}

void handle_getBmsSpgData()
{
  Serial.println("handle_getBmsSpgData");
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
  //Serial.println(sendData);
  server.send(200, "text/html", sendData.c_str());
}

void handle_getBtDevices()
{
  bleHanlder.startScan(); //BT Scan starten
  server.send(200, "text/html", bleHanlder.getBtScanResult().c_str());
}


void handle_getOnewireDeviceAdr() {
  Serial.println("handle_getOnewireDeviceAdr()");
  server.send(200, "text/html", getSensorAdr().c_str());
}


/*void btnTest1()
{
  Serial.println("TestBtn 1");
}

void btnTest2()
{
  Serial.println("TestBtn 2");
}

void btnTest3()
{
  Serial.println("TestBtn 3");
}*/


uint8_t checkTaskRun()
{
  uint8_t ret = 0;
  xSemaphoreTake(mutexTaskRunTime, portMAX_DELAY);
  if(millis()-lastTaskRun_ble>3000) ret+=1;
  if(millis()-lastTaskRun_alarmrules>2000) ret+=2;
  if(millis()-lastTaskRun_onewire>2000) ret+=4;
  if(millis()-lastTaskRuncanbusTx>2000) ret+=8;
  if(millis()-lastTaskRun_bscSerial>2000) ret+=16;
  xSemaphoreGive(mutexTaskRunTime);
  return ret;
}


void setup() {  
  if(bootCounter!=0xFF) bootCounter++;

  //esp_log_level_set("*", ESP_LOG_VERBOSE);   

  mutexTaskRunTime = xSemaphoreCreateMutex();

  isBoot = true;
  Serial.begin(115200);
  Serial.println("BSC");
  Serial.printf("bootCounter=%i\n", bootCounter);

  //init DIO 
  if(bootCounter==1) initDio(false);
  else initDio(true);

  bmsDataInit();

  //init WebSettings
  webSettingsSystem.initWebSettings(&paramSystem, "System", "/WebSettings.conf",0);

  webSettingsBluetooth.initWebSettings(&paramBluetooth, "Bluetooth", "/WebSettings.conf",0);
  webSettingsBluetooth.setTimerHandlerName("getBtDevices");
  webSettingsSerial.initWebSettings(&paramSerial, "Serial", "/WebSettings.conf",0);
  webSettingsAlarmBt.initWebSettings(&paramAlarmBms, "Alarm Bluetooth Ger&auml;te", "/WebSettings.conf",0);
  webSettingsAlarmTemp.initWebSettings(&paramAlarmTemp, "Alarm Temperatur", "/WebSettings.conf",0);
  webSettingsDitialOut.initWebSettings(&paramDigitalOut, "Digitalausg&auml;nge", "/WebSettings.conf",0);
  webSettingsDitialIn.initWebSettings(&paramDigitalIn, "Digitaleing&auml;nge", "/WebSettings.conf",0);
  webSettingsOnewire.initWebSettings(&paramOnewireAdr, "Onewire", "/WebSettings.conf",0);
  webSettingsOnewire.setTimerHandlerName("getOwDevices",2000);
  webSettingsOnewire2.initWebSettings(&paramOnewire2, "Onewire II", "/WebSettings.conf",0);

  webSettingsBmsToInverter.initWebSettings(&paramBmsToInverter, "BMS-Daten Wechselrichter", "/WebSettings.conf",0);
  /*webSettingsBmsToInverter.setButtons(BUTTON_1,"test1");
  webSettingsBmsToInverter.setButtons(BUTTON_2,"test2");
  webSettingsBmsToInverter.setButtons(BUTTON_3,"test3");
  webSettingsBmsToInverter.registerOnButton1(&btnTest1);
  webSettingsBmsToInverter.registerOnButton2(&btnTest2);
  webSettingsBmsToInverter.registerOnButton3(&btnTest3);*/



  //Erstelle Timer
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectWiFi));
   
  //mqtt
  initMqtt();

  //init WLAN
  Serial.println(F("Init WLAN..."));
  WiFi.onEvent(onWiFiEvent);
  connectWiFi();
  Serial.println(F("ok"));
  
  char dns[30] = {'b','s','c'};
  if (MDNS.begin(dns))
  {
    Serial.println("MDNS responder gestartet");
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
  server.on("/settings/schnittstellen/ow/getOwDevices",handle_getOnewireDeviceAdr);
  
  webota.init(&server, "/webota"); //webota

  server.on("/restart/", []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", "<HTML><BODY>Reboot ok<br><a href='../'>Home</a></BODY></HTML>");
    sleep(2);
    ESP.restart();
  });

  //starte webserver
  Serial.print(F("Starte Webserver..."));
  server.begin(WEBSERVER_PORT);
  Serial.println(F("ok"));

  //init Bluetooth
  Serial.print(F("Init BLE..."));
  bleHanlder.init();
  Serial.println(F("ok"));

  //init Onewire
  owSetup();

  //Serial
  bscSerial1.initSerial();
  bscSerial2.initSerial();
  bscSerial3.initSerial();

  //init Alarmrules
  initAlarmRules();

  //Erstelle Tasks
  xTaskCreate(task_ble, "ble", 3000, nullptr, 5, &task_handle_ble);
  xTaskCreate(task_alarmRules, "alarmrules", 2700, nullptr, configMAX_PRIORITIES - 5, &task_handle_alarmrules);
  xTaskCreate(task_onewire, "ow", 3100, nullptr, 5, &task_handle_onewire);
  xTaskCreate(task_canbusTx, "can", 2700, nullptr, 5, &task_handle_canbusTx);
  xTaskCreate(task_bscSerial, "serial", 3000, nullptr, 5, &task_handle_bscSerial);

  previousMillis10000=millis();
  free_dump();  
}


void loop() {
  server.handleClient();
  
  mqttLoop();

  currentMillis = millis();

  if(currentMillis - previousMillis10000 >=10000)
  {
    u8_mTaskRunSate = checkTaskRun();

    //Sende Daten via mqqtt, wenn aktiv
    if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
    {
      mqttPublish("sys/esp32Temp", temperatureRead()); 
      mqttPublish("inverter/chargeCurrentSoll", getAktualChargeCurrentSoll());

      mqttPublish("sys/free_heap", xPortGetFreeHeapSize());
      mqttPublish("sys/min_free_heap", xPortGetMinimumEverFreeHeapSize());
      
      mqttPublish("sys/highWater_task_ble", uxTaskGetStackHighWaterMark(task_handle_ble));
      mqttPublish("sys/highWater_task_alarmrules", uxTaskGetStackHighWaterMark(task_handle_alarmrules));
      mqttPublish("sys/highWater_task_ow", uxTaskGetStackHighWaterMark(task_handle_onewire));
      mqttPublish("sys/highWater_task_can", uxTaskGetStackHighWaterMark(task_handle_canbusTx));
      mqttPublish("sys/highWater_task_serial", uxTaskGetStackHighWaterMark(task_handle_bscSerial));
    }

    previousMillis10000 = currentMillis;
  }


}


