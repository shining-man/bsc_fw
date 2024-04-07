// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "log.h"
#include "defines.h"
#include "SoftwareSerial.h"
#include <FS.h>
#ifdef USE_LittleFS
  #define SPIFFS LittleFS
  #include <LittleFS.h>
#else
  #include <SPIFFS.h>
#endif
#include <FS.h>
#include "bscTime.h"
#include "BmsData.h"
#include "WebSettings.h"
#include "inverter/Inverter.hpp"

static const char *TAG = "LOG";

static SemaphoreHandle_t logMutex = NULL;
static SemaphoreHandle_t deleteLogMutex = NULL;
static SemaphoreHandle_t fsMutex = NULL;

static File spiffsTriggerLogFile;
static File spiffsValueLogFile;

#ifdef DEBUG_ON_FS
static File spiffsLogFile;
static char log_print_buffer[512];
int vprintf_into_spiffs(const char* szFormat, va_list args);
#endif


#define VALUE_LOG_DATASET_SIZE 30


//fsMutex
void fsLock()
{
  xSemaphoreTake(fsMutex, portMAX_DELAY);
}

void fsUnlock()
{
  xSemaphoreGive(fsMutex);
}

bool isFsLock()
{
  if(xSemaphoreTake(fsMutex, (TickType_t)5) == pdTRUE) return true;
  else return false;
}


void debugInit()
{
  logMutex = xSemaphoreCreateMutex();
  deleteLogMutex = xSemaphoreCreateMutex();
  fsMutex = xSemaphoreCreateMutex();
  #ifdef DEBUG_ON_HW_SERIAL
  Serial.end();
  Serial.setPins(18,1); // Als RX Pin den U1RXTX_En nehmen, da dieser hier nicht gebraucht wird
  Serial.begin(115200);
  #else
  //Serial beenden um auf die Pins 3+1 die Softserial zu mappen
  Serial.end();
  Serial.setPins(SERIAL3_PIN_RX,SERIAL3_PIN_TX);
  #endif


  if(!SPIFFS.begin())
  {
    BSC_LOGE(TAG,"LITTLEFS Mount Failed");
    SPIFFS.format();
  }

  if(SPIFFS.begin())
  {
    if(SPIFFS.exists("/trigger.txt")) spiffsTriggerLogFile=SPIFFS.open("/trigger.txt", FILE_APPEND);
    else spiffsTriggerLogFile=SPIFFS.open("/trigger.txt", FILE_WRITE);
  }

  if(SPIFFS.begin())
  {
    if(SPIFFS.exists("/values"))
    {
      spiffsValueLogFile=SPIFFS.open("/values", "r+");
    }
    else
    {
      spiffsValueLogFile=SPIFFS.open("/values", FILE_WRITE);
      for(uint32_t i=0;i<(1440*VALUE_LOG_DATASET_SIZE);i++) spiffsValueLogFile.write(0x0);
    }
  }

  #ifdef DEBUG_ON_FS
  if (SPIFFS.begin())
  {
    BSC_LOGE(TAG,"LITTLEFS Mount Failed (2)");
    esp_log_set_vprintf(&vprintf_into_spiffs);

    if(SPIFFS.exists("/log.txt")) spiffsLogFile=SPIFFS.open("/log.txt", FILE_APPEND);
    else spiffsLogFile=SPIFFS.open("/log.txt", FILE_WRITE);
  }
  #endif


  //esp_log_level_set("*", ESP_LOG_VERBOSE); //Log ALL
  esp_log_level_set("*", ESP_LOG_INFO); //Log INFO

  //esp_log_level_set("MAIN", ESP_LOG_INFO);
  //esp_log_level_set("BLE_HANDLER", ESP_LOG_INFO);
  //esp_log_level_set("MQTT", ESP_LOG_INFO);
  //esp_log_level_set("ALARM", ESP_LOG_INFO);
  //esp_log_level_set("OW", ESP_LOG_INFO); //onewire
  //esp_log_level_set("CAN", ESP_LOG_INFO);
  //esp_log_level_set("JBD_BMS", ESP_LOG_VERBOSE);

  //esp_log_level_set("ALARM", ESP_LOG_DEBUG);
  //esp_log_level_set("I2C", ESP_LOG_DEBUG);
  //esp_log_level_set("MAIN", ESP_LOG_DEBUG);


  #ifdef NEEY_DEBUG
  esp_log_level_set("NEEY", ESP_LOG_DEBUG);
  #endif
  #ifdef NEEY_WRITE_DATA_DEBUG
  esp_log_level_set("NEEY", ESP_LOG_DEBUG);
  esp_log_level_set("BLE_HANDLER", ESP_LOG_DEBUG);
  #endif
  #ifdef JK_DEBUG
  esp_log_level_set("JK_BMS", ESP_LOG_DEBUG);
  #endif
  #ifdef JK_BT_DEBUG
  esp_log_level_set("JKBT", ESP_LOG_DEBUG);
  #endif
  #ifdef SEPLOS_DEBUG
  esp_log_level_set("SEPLOS_BMS", ESP_LOG_DEBUG);
  #endif
  #ifdef DALY_DEBUG
  esp_log_level_set("DALY_BMS", ESP_LOG_DEBUG);
  #endif
  #ifdef BT_DEBUG        //Bluetooth
  esp_log_level_set("BLE_HANDLER", ESP_LOG_DEBUG);
  #endif
  #ifdef MQTT_DEBUG
  esp_log_level_set("MQTT", ESP_LOG_DEBUG);
  #endif
  #ifdef CAN_DEBUG
  esp_log_level_set("CAN", ESP_LOG_DEBUG);
  #endif
  #ifdef WEBSET_DEBUG
  esp_log_level_set("WEB_SETTINGS", ESP_LOG_DEBUG);
  #endif

  #ifdef GOBEL_DEBUG
  esp_log_level_set("GOBEL_BMS", ESP_LOG_DEBUG);
  #endif
  #ifdef GOBELPC200_DEBUG
  esp_log_level_set("GOBEL_BMS_PC200", ESP_LOG_DEBUG);
  #endif
  #ifdef WLAN_DEBUG
  #endif
  #ifdef WLAN_DEBUG2
  #endif


  #ifdef DEBUG_ON_FS
  if(!SPIFFS.exists("/log.txt"))
  {
    BSC_LOGE(TAG, "Error with the SPIFFS!");
  }
  BSC_LOGI(TAG, "Free Space total=%i, used=%i, logSize=%i",SPIFFS.totalBytes(),SPIFFS.usedBytes(),spiffsLogFile.size());
  #endif
}


#ifdef DEBUG_ON_FS
bool logEn=true;
static String str_mPrintBuffer0="";
static String str_mPrintBuffer1="";
static uint8_t u8_mAktivPrintBuffer=0;
static bool bo_mNewDataInBuffer=false;
int vprintf_into_spiffs(const char* szFormat, va_list args)
{
  int ret=0;
  if(!logEn) return 0;


    //write evaluated format string into buffer
    ret = vsnprintf (log_print_buffer, sizeof(log_print_buffer), szFormat, args);

    if(ret >= 0)
    {
      #ifdef LOG_TO_SERIAL
      #ifdef DEBUG_ON_HW_SERIAL
      Serial.print(log_print_buffer);
      //#else
      //debugPort.print(log_print_buffer);
      #endif
      #endif

      xSemaphoreTake(logMutex, portMAX_DELAY);

      if(u8_mAktivPrintBuffer==0)str_mPrintBuffer0+=log_print_buffer;
      else str_mPrintBuffer1+=log_print_buffer;
      bo_mNewDataInBuffer=true;

      xSemaphoreGive(logMutex);
  }
	return ret;
}
#endif


#ifdef DEBUG_ON_FS
unsigned long writeLogToFsTimer=0;
void writeLogToFS()
{
  if((millis()-writeLogToFsTimer)>50) writeLogToFsTimer=millis();
  else return;

  if(isFsLock()==false) return; //fsLock
  if(spiffsLogFile.size()>100000)
  {
    xSemaphoreTake(deleteLogMutex, portMAX_DELAY);
    spiffsLogFile.close();
    SPIFFS.remove("/log1.txt");
    SPIFFS.rename("/log.txt","/log1.txt");
    spiffsLogFile = SPIFFS.open("/log.txt", FILE_WRITE);
    xSemaphoreGive(deleteLogMutex);
  }

  xSemaphoreTake(logMutex, portMAX_DELAY);
  if(bo_mNewDataInBuffer)
  {
    if(u8_mAktivPrintBuffer==0)u8_mAktivPrintBuffer=1;
    else u8_mAktivPrintBuffer=0;
    bo_mNewDataInBuffer=false;
    xSemaphoreGive(logMutex);
  }
  else
  {
    xSemaphoreGive(logMutex);
    fsUnlock();
    return;
  }

  xSemaphoreTake(deleteLogMutex, portMAX_DELAY);
  if(u8_mAktivPrintBuffer==0)
  {
    spiffsLogFile.print(str_mPrintBuffer1);
    str_mPrintBuffer1="";
  }
  else
  {
    spiffsLogFile.print(str_mPrintBuffer0);
    str_mPrintBuffer0="";
  }

  spiffsLogFile.flush();
  xSemaphoreGive(deleteLogMutex);
  fsUnlock();
}

void deleteLogfile()
{
  fsLock();
  xSemaphoreTake(deleteLogMutex, portMAX_DELAY);
  spiffsLogFile.close();
  SPIFFS.remove("/log.txt");
  SPIFFS.remove("/log1.txt");
  if(SPIFFS.exists("/log.txt")) //Wenn Log-Datei immer noch vorhanden
  {
    BSC_LOGI(TAG, "Fehler beim löschen der Logfiles");
  }
  else
  {
    spiffsLogFile = SPIFFS.open("/log.txt", FILE_WRITE);
    BSC_LOGI(TAG, "Logfiles gelöscht");
  }
  xSemaphoreGive(deleteLogMutex);
  fsUnlock();
}
#endif

void logTrigger(uint8_t triggerNr, uint8_t cause, bool trigger)
{
  fsLock();
  if(spiffsTriggerLogFile.size()>10000)
  {
    spiffsTriggerLogFile.close();
    SPIFFS.remove("/trigger1.txt");
    SPIFFS.rename("/trigger.txt","/trigger1.txt");
    spiffsTriggerLogFile = SPIFFS.open("/trigger.txt", FILE_WRITE);
  }

  //spiffsTriggerLogFile.printf("%s%02x%02x%02x\r\n",getBscDateTimeCc2(),triggerNr,cause,trigger);
  spiffsTriggerLogFile.printf("%08x%02x%02x%02x%02x",getEpoch(),triggerNr,cause,trigger,0xAA);
  spiffsTriggerLogFile.flush();
  fsUnlock();
}


uint8_t u8_lGetMinutesOld;
void logValues(Inverter &inverter)
{
  uint8_t u8_lGetMinutes = getMinutes();
  if(u8_lGetMinutes==u8_lGetMinutesOld) return;
  u8_lGetMinutesOld=u8_lGetMinutes;

  if(WebSettings::getInt(ID_PARAM_SYSTEM_RECORD_VALUES_PERIODE,0,DT_ID_PARAM_SYSTEM_RECORD_VALUES_PERIODE)==0) return;

  uint32_t timeMinutes = getDayMinutes();
  //BSC_LOGI(TAG,"logValues: New Entry, time=%i, u8_lGetMinutes=%i, getMinutesOld=%i",timeMinutes, u8_lGetMinutes, u8_lGetMinutesOld);

  if(timeMinutes==0)
  {
    spiffsValueLogFile.close();
    SPIFFS.remove("/values1");
    SPIFFS.rename("/values","/values1");
    spiffsValueLogFile = SPIFFS.open("/values", FILE_WRITE);
    for(uint32_t i=0;i<(1440*VALUE_LOG_DATASET_SIZE);i++) spiffsValueLogFile.write(0x0);
  }


  inverter.inverterDataSemaphoreTake();
  Inverter::inverterData_s *inverterData = inverter.getInverterData();
  int16_t inverterCurrent = inverterData->batteryCurrent;
  int16_t inverterVoltage = inverterData->batteryVoltage;
  uint16_t inverterSoc = inverterData->inverterSoc;
  int16_t inverterChargeCurrent = inverterData->inverterChargeCurrent/10;
  int16_t inverterDischargeCurrent = inverterData->inverterDischargeCurrent/10;

  int16_t calcChargeCurrentCellVoltage = inverterData->calcChargeCurrentCellVoltage;
  int16_t calcChargeCurrentSoc = inverterData->calcChargeCurrentSoc;
  int16_t calcChargeCurrentCelldrift = inverterData->calcChargeCurrentCelldrift;
  int16_t calcChargeCurrentCutOff = inverterData->calcChargeCurrentCutOff;
  inverter.inverterDataSemaphoreGive();

  /*bmsDataSemaphoreTake();
  bmsData_s *bmsData = getBmsData();
  uint16_t bmsMinCellVoltage = bmsData->bmsMaxCellVoltage;
  uint16_t bmsMinCellVoltage = bmsData->bmsMinCellVoltage;
  bmsDataSemaphoreGive();*/

  fsLock();
  spiffsValueLogFile.seek(timeMinutes*VALUE_LOG_DATASET_SIZE,SeekSet);
  spiffsValueLogFile.write((uint8_t*)&inverterCurrent,2);
  spiffsValueLogFile.write((uint8_t*)&inverterVoltage,2);
  spiffsValueLogFile.write((uint8_t*)&inverterSoc,2);
  spiffsValueLogFile.write((uint8_t*)&inverterChargeCurrent,2);
  spiffsValueLogFile.write((uint8_t*)&inverterDischargeCurrent,2);
  spiffsValueLogFile.write((uint8_t*)&calcChargeCurrentCellVoltage,2);
  spiffsValueLogFile.write((uint8_t*)&calcChargeCurrentSoc,2);
  spiffsValueLogFile.write((uint8_t*)&calcChargeCurrentCelldrift,2);
  spiffsValueLogFile.write((uint8_t*)&calcChargeCurrentCutOff,2);
  spiffsValueLogFile.flush();
  fsUnlock();
}
