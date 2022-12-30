// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "log.h"
#include "defines.h"
#include "SoftwareSerial.h"
#include "SPIFFS.h"
#include <FS.h>
#include "bscTime.h"

static const char *TAG = "LOG";

static SemaphoreHandle_t logMutex = NULL;

#ifndef DEBUG_ON_HW_SERIAL
SoftwareSerial debugPort;
#endif

#ifdef DEBUG_ON_FS
static File spiffsLogFile;
static char log_print_buffer[512];
int vprintf_into_spiffs(const char* szFormat, va_list args);
#endif


void debugInit()
{
  logMutex = xSemaphoreCreateMutex();

  #ifdef DEBUG_ON_HW_SERIAL
  Serial.begin(115200);
  #else
  //Serial beenden um auf die Pins 3+1 die Softserial zu mappen
  Serial.end();
  Serial.setPins(SERIAL3_PIN_RX,SERIAL3_PIN_TX);

  debugPort.enableIntTx(false);
  debugPort.enableRx(false);
  debugPort.begin(DEBUG_SW_BAUDRATE, SWSERIAL_8N1, 3, 1, false, 64, 11);
  #endif

  #ifdef DEBUG_ON_FS
  if(!SPIFFS.begin())
  {
    SPIFFS.format();
  }

  if (SPIFFS.begin())
  {
    esp_log_set_vprintf(&vprintf_into_spiffs);

    if(SPIFFS.exists("/log.txt")) spiffsLogFile=SPIFFS.open("/log.txt", FILE_APPEND);
    else spiffsLogFile=SPIFFS.open("/log.txt", FILE_WRITE);
  }
  #endif


  #ifdef DEBUG_ON_HW_SERIAL
  //esp_log_level_set("*", ESP_LOG_VERBOSE); //LOG all
  esp_log_level_set("*", ESP_LOG_INFO); 
  esp_log_level_set("MAIN", ESP_LOG_VERBOSE); 
  esp_log_level_set("JBD_BMS", ESP_LOG_VERBOSE); 
  esp_log_level_set("JK_BMS", ESP_LOG_VERBOSE); 
  esp_log_level_set("ALARM", ESP_LOG_INFO); 
  esp_log_level_set("BLE_HANDLER", ESP_LOG_DEBUG); 
  esp_log_level_set("SEPLOS_BMS", ESP_LOG_DEBUG); 
  
  esp_log_level_set("OW", ESP_LOG_INFO); //onewire    ESP_LOG_DEBUG
  #else
  esp_log_level_set("*", ESP_LOG_INFO); 
  esp_log_level_set("MAIN", ESP_LOG_VERBOSE); 
  esp_log_level_set("JBD_BMS", ESP_LOG_VERBOSE); 
  esp_log_level_set("JK_BMS", ESP_LOG_VERBOSE); 
  esp_log_level_set("ALARM", ESP_LOG_INFO); 
  esp_log_level_set("BLE_HANDLER", ESP_LOG_DEBUG); 
  esp_log_level_set("SEPLOS_BMS", ESP_LOG_DEBUG); 
  
  esp_log_level_set("OW", ESP_LOG_INFO); //onewire    ESP_LOG_DEBUG
  #endif


  #ifdef DEBUG_ON_FS
  if(!SPIFFS.exists("/log.txt")) 
  {
    ESP_LOGE(TAG, "Error with the SPIFFS!");
    /*ESP_LOGE(TAG, "Format SPIFFS!");
    ESP_LOGI(TAG, "Free Space total=%i, used=%i, logSize=%i",SPIFFS.totalBytes(),SPIFFS.usedBytes(),spiffsLogFile.size());
    SPIFFS.format();
    spiffsLogFile = SPIFFS.open("/log.txt", FILE_WRITE);*/
  }
  ESP_LOGI(TAG, "Free Space total=%i, used=%i, logSize=%i",SPIFFS.totalBytes(),SPIFFS.usedBytes(),spiffsLogFile.size());
  #endif
}

#ifdef DEBUG_ON_FS
bool logEn=true;
int vprintf_into_spiffs(const char* szFormat, va_list args)
{
  if(!logEn) return 0;
	
  xSemaphoreTake(logMutex, portMAX_DELAY);

  //write evaluated format string into buffer
	int ret = vsnprintf (log_print_buffer, sizeof(log_print_buffer), szFormat, args);

	if(ret >= 0)
  {
    #ifdef LOG_TO_SERIAL
    #ifdef DEBUG_ON_HW_SERIAL
    Serial.print(log_print_buffer);
    #else
    debugPort.print(log_print_buffer);
    #endif
    #endif

    if(spiffsLogFile)
    {
      if(spiffsLogFile.size()>100000)
      {
        spiffsLogFile.close();
        SPIFFS.remove("/log1.txt");
        SPIFFS.rename("/log.txt","/log1.txt");
        spiffsLogFile = SPIFFS.open("/log.txt", FILE_WRITE);
      } 

      spiffsLogFile.write((uint8_t*) log_print_buffer, (size_t) ret);
      spiffsLogFile.flush();            
    }
	}
  xSemaphoreGive(logMutex);
	return ret;
}
#endif
