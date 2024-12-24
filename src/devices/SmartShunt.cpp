// Copyright (c) 2023 shiningman
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/SmartShunt.h"
#include "devices/DeviceUtils.hpp"
#include "BscSerial.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include "WebSettings.h"
#include "Utility.h"

static const char *TAG = "SMARTSHUNT";

static Stream *mPort;
static uint8_t u8_mDevNr;

static void      getDataFromBms(deviceUtils::DeviceUtils &devUtils, BscSerial *bscSerial, uint16_t ID_Get);
static bool      recvAnswer(deviceUtils::DeviceUtils &devUtils, uint8_t * t_outMessage, uint16_t ID_Get);
static void      parseMessage(deviceUtils::DeviceUtils &devUtils, uint8_t * t_message);
static bool      hexIsValid(deviceUtils::DeviceUtils &devUtils, const uint8_t* buffer, int size);
static bool      getBMSTelegramm(deviceUtils::DeviceUtils &devUtils, BscSerial *bscSerial, uint16_t ID_Get);

enum SM_readData {SEARCH_START, SEARCH_END};

static serialDevData_s *mDevData;

static uint8_t u8_DataPointer;
static uint8_t mDataMappingNr;

//https://www.victronenergy.com/live/vedirect_protocol:faq


bool SmartShunt_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData)
{
  bool ret = true;
  mDevData = devData;
  mPort = port;
  u8_mDevNr = devNr;
  uint8_t response[SMARTSHUNT_MAX_ANSWER_LEN];
  deviceUtils::DeviceUtils devUtils;

  mDataMappingNr = devData->dataMappingNr;

// Nur die erste 4 Daten sind Relevant für die Ausgabe eines Fehlers
  if (!getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_SOC)) ret = false; 
  if (!getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_MAIN_VOLTAGE)) ret = false; 
  if (!getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_CURRENT)) ret = false; 
  if (!getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_POWER)) ret = false; 
 
  if(mDevData->bo_sendMqttMsg)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_TIME_TO_GO);
    u8_DataPointer=1;
  }
    else if (u8_DataPointer==1)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_CYCLE);
    u8_DataPointer=2;
  }
    else if (u8_DataPointer==2)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_TOTAL_VOLT_MIN);
    u8_DataPointer=3;
  }
    else if (u8_DataPointer==3)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_TOTAL_VOLT_MAX);
    u8_DataPointer=4;
  }
    else if (u8_DataPointer==4)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_TIME_SINCE_FULL);
    u8_DataPointer=5;
  }
    else if (u8_DataPointer==5)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_VOLT_MIN_COUNT);
    u8_DataPointer=6;
  }
    else if (u8_DataPointer==6)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_TOTAL_VOLT_MAX_COUNT);
    u8_DataPointer=7;
  }
    else if (u8_DataPointer==7)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_AMOUNT_DCH_ENERGY);
    u8_DataPointer=8;
  }
    else if (u8_DataPointer==8)
  {
    getBMSTelegramm(devUtils, bscSerial, SMARTSHUNT_ID_AMOUNT_CH_ENERGY);
    u8_DataPointer=9;
  }

  if(u8_DataPointer>8)u8_DataPointer=0;

  return ret;
}

bool getBMSTelegramm(deviceUtils::DeviceUtils &devUtils, BscSerial *bscSerial, uint16_t ID_Get)
{
    uint8_t response[SMARTSHUNT_MAX_ANSWER_LEN];
    uint8_t anzahl_wiederholungen;
    bool ret = true;

    // Wenn beim ersten Telegramm (SOC) ein Fehler auftaucht, wird probiert dieses zu wiederholen. 
    // Grund: Der Smartshunt geht nach ca. 1s wieder in den Standartkommunikationsmodus zurück und
    // mit diesem Telegramm kann / wird Kollision entstehen, deshalb warten wir nach einem Fehler 100ms.
    // Danach ist der Shunt mit dem Telegram in jedem Fall fertig und ist wieder bereit für das
    // HEX Protokoll.

    if (ID_Get == SMARTSHUNT_ID_SOC)
    { anzahl_wiederholungen = 2;}
    else
    { anzahl_wiederholungen = 1;}

    for(uint8_t i=0;i<anzahl_wiederholungen;i++)
    {
      getDataFromBms(devUtils, bscSerial, ID_Get);
      if(recvAnswer(devUtils, response, ID_Get))
      {
        parseMessage(devUtils, response);
        break;
      }
      else
      {
        #ifdef SMARTSHUNT_DEBUG
          BSC_LOGE(TAG,"Antwort nicht OK - ID Get %u - Versuch Nr. :%i",ID_Get,i);
        #endif
        if(i>=anzahl_wiederholungen)
        {
          #ifdef SMARTSHUNT_DEBUG
            BSC_LOGE(TAG,"Antwort nicht OK - ID Get %u",ID_Get);
          #endif
          ret = false;
        }
      }
    }
    return ret;
}

static void getDataFromBms(deviceUtils::DeviceUtils &devUtils, BscSerial *bscSerial, uint16_t ID_Get)
{
  uint8_t u8_lData[11];
  uint8_t u8_lSendData[20];
  uint8_t chksum;

  // Sendedaten vorbereiten
  u8_lSendData[0]=0x3A;             // Start der Nachricht mit ":"
  u8_lSendData[1]=0x37;             // Command 7 = Get (in ASCII)

  u8_lData[2]=(ID_Get >> 0)&0xFF; // ID der abzufragenden Daten - Low Byte
  u8_lData[3]=(ID_Get >> 8)&0xFF; // ID der abzufragenden Daten - High Byte
  u8_lData[4]=0x00;              // Flag 0x00

  chksum = 0x55-0x07; // Checksum (0x55 - 0x07(Get) - Bytes bis hier)
  for(uint8_t i=2;i<5;i++) chksum -= u8_lData[i];

  devUtils.ByteToAsciiHex(&u8_lSendData[2],&u8_lData[2], 3);

  u8_lData[5]=chksum;            // Checksum
  devUtils.ByteToAsciiHex(&u8_lSendData[8],&u8_lData[5], 1);

  u8_lSendData[10]=0x0A;             // Ende Befehl /n (LF)

  #ifdef SMARTSHUNT_DEBUG
    String sendBytes="";
    uint8_t u8_logByteCount=0;
    for(uint8_t z=0;z<11;z++)
    {
      u8_logByteCount++;
      sendBytes+="0x";
      sendBytes+= String(u8_lSendData[z],16);
      sendBytes+=" ";
    }
    BSC_LOGI(TAG,"ENDE SendBytes=%i: %s",u8_logByteCount, sendBytes.c_str());
  #endif

  //TX
  bscSerial->sendSerialData(mPort, u8_mDevNr, u8_lSendData, 11);

}

/// @brief
/// @param p_lRecvBytes
/// @return
static bool recvAnswer(deviceUtils::DeviceUtils &devUtils, uint8_t *p_lRecvBytes, uint16_t ID_Get)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvDataLen;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START;
  u8_lRecvBytesCnt=0;
  u8_lRecvDataLen=0xFF;
  bool bo_lDataComplete=false;

  for(;;)
  {
    //Timeout
    // wenn innerhalb von 50ms das Telegram noch nicht begonnen hat, dann Timeout
    if( (((millis()-u32_lStartTime)>100) && ID_Get==SMARTSHUNT_ID_SOC) || (((millis()-u32_lStartTime)>50) && !(ID_Get==SMARTSHUNT_ID_SOC)) )
    {
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
      #endif
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch (SMrecvState)  {
        case SEARCH_START:
          if (u8_lRecvByte == 0x3A)  //":"
          {
            SMrecvState=SEARCH_END;
          }
          break;

        case SEARCH_END:
          p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
          if(u8_lRecvByte == 0x0A)  // "Linefeed"
          {
            bo_lDataComplete=true;
            break;
          }

          u8_lRecvBytesCnt++;
          break;

        default:
          break;
        }
    }
    else vTaskDelay(pdMS_TO_TICKS(10));

    if(bo_lDataComplete) break; //Recv Pakage complete
    if(u8_lRecvBytesCnt>=SMARTSHUNT_MAX_ANSWER_LEN) 
     {
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"recvAnswer - Antwort zu lang");
      #endif
      return false; //Answer too long!
     }
  }

  #ifdef SMARTSHUNT_DEBUG
  buffer2Log(p_lRecvBytes, u8_lRecvBytesCnt);
  #endif

  //Überprüfe Cheksum
  if(!hexIsValid(devUtils, p_lRecvBytes,u8_lRecvBytesCnt)) 
    {
      #ifdef SMARTSHUNT_DEBUG
      BSC_LOGI(TAG,"recvAnswer - Checksum falsch"); 
      #endif
      return false;
    }
  
  return true;
}

static void parseMessage(deviceUtils::DeviceUtils &devUtils, uint8_t * t_message)
{
  uint16_t Smartshunt_ID;
  uint16_t Smartshunt_Flag;


  // Ist die Nachricht die Antwort auf ein GET (7)?
  if(t_message[0] == 0x37)
  {
    Smartshunt_ID = devUtils.AsciiHexToU16(t_message,1);
    Smartshunt_Flag = devUtils.AsciiHexToByte(t_message[5], t_message[6]);
    #ifdef SMARTSHUNT_DEBUG
      BSC_LOGI(TAG,"Flag =%u",Smartshunt_Flag);
    #endif

    if(Smartshunt_ID==SMARTSHUNT_ID_MAIN_VOLTAGE)
    {
      float floatValue = (float)(static_cast<int16_t>(devUtils.AsciiHexToU16(t_message,7))) / 100; // 0,01V
      setBmsTotalVoltage(mDataMappingNr, floatValue);
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, floatValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Voltage =%f",floatValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_CURRENT)
    {
      float floatValue = (float)(static_cast<int16_t>(devUtils.AsciiHexToU16(t_message,7))) / 10; // 0,1A
      setBmsTotalCurrent(mDataMappingNr, floatValue);
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, floatValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Current =%f",floatValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_POWER)
    {
      int16_t i16_tValue = (int16_t)devUtils.AsciiHexToU16(t_message,7); // W
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_POWER, -1, i16_tValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Power =%i",i16_tValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_SOC)
    {
      float floatValue = (float)devUtils.AsciiHexToU16(t_message,7) / 100; // 0,01%
      setBmsChargePercentage(mDataMappingNr, (uint8_t)floatValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"SOC =%u",(uint8_t)floatValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_TIME_TO_GO)
    {
      uint16_t u16_tValue = devUtils.AsciiHexToU16(t_message,7); // 1 Minute
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TIME_TO_GO, -1, u16_tValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"TTG =%u",u16_tValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_CYCLE)
    {
      uint32_t u32_tValue = devUtils.AsciiHexToU32(t_message,7); // 1 Cycle
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_CYCLE, -1, u32_tValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Cycle =%u",u32_tValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_TOTAL_VOLT_MIN)
    {
      float floatValue = (float)devUtils.AsciiHexToU32(t_message,7) / 100; // 0,01V
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TOTAL_VOLT_MIN, -1, floatValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Total Volt Minimum =%f",floatValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_TOTAL_VOLT_MAX)
    {
      float floatValue = (float)devUtils.AsciiHexToU32(t_message,7) / 100; // 0,01V
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TOTAL_VOLT_MAX, -1, floatValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Total Volt Maximum =%f",floatValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_TIME_SINCE_FULL)
    {
      uint32_t u32_tValue = devUtils.AsciiHexToU32(t_message,7); // 1 Seconds
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TIME_SINCE_FULL, -1, u32_tValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Second since last full =%u",u32_tValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_SOC_SYNC_COUNT)
    {
      uint32_t u32_tValue = devUtils.AsciiHexToU32(t_message,7); // 1 Seconds
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_SOC_SYNC_COUNT, -1, u32_tValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Number of automatic synchronizations =%u",u32_tValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_VOLT_MIN_COUNT)
    {
      uint32_t u32_tValue = devUtils.AsciiHexToU32(t_message,7); // 1 Seconds
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TOTAL_VOLT_MIN_COUNT, -1, u32_tValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Number of Low Voltage Alarms =%u",u32_tValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_TOTAL_VOLT_MAX_COUNT)
    {
      uint32_t u32_tValue = devUtils.AsciiHexToU32(t_message,7); // 1 Seconds
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_TOTAL_VOLT_MAX_COUNT, -1, u32_tValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Number of High Voltage Alarms =%u",u32_tValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_AMOUNT_DCH_ENERGY)
    {
      float floatValue = (float)devUtils.AsciiHexToU32(t_message,7) / 100; // 1 0,01kWh
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_AMOUNT_DCH_ENERGY, -1, floatValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Amount of discharged energy =%f",floatValue);
      #endif
    }
    else if(Smartshunt_ID==SMARTSHUNT_ID_AMOUNT_CH_ENERGY)
    {
      float floatValue = (float)devUtils.AsciiHexToU32(t_message,7) / 100; // 1 0,01kWh
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, mDataMappingNr, MQTT_TOPIC2_AMOUNT_CH_ENERGY, -1, floatValue);
      #ifdef SMARTSHUNT_DEBUG
        BSC_LOGI(TAG,"Amount of discharged energy =%f",floatValue);
      #endif
    }
  }
}


/*
 *	hexIsValid
 *  This function compute checksum and validate hex frame
 */

static bool hexIsValid(deviceUtils::DeviceUtils &devUtils, const uint8_t* buffer, int size)
{
  uint8_t checksum=0x55-devUtils.AsciiHexToNibble(buffer[0]);
  for (int i=1; i<size; i+=2)
  {
  checksum -= devUtils.AsciiHexToByte(buffer[i],buffer[i+1]);
  }
  #ifdef SMARTSHUNT_DEBUG 
    if(!(checksum==0)) BSC_LOGE(TAG,"Checksum =%u",checksum);
  #endif
  return (checksum==0);
}