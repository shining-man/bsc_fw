// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/NeeySerial.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

static const char *TAG = "NEEY";

static Stream *mPort;
static uint8_t u8_mTxEnRS485pin, u8_mDevNr;



//
static bool recvAnswer(uint8_t *p_lRecvBytes);
static void message2Log(uint8_t * t_message, uint8_t address, uint8_t len);


static serialDevData_s *mDevData;

/*
0x2B 0x43 0x4F 0x4E 0x4E 0x45 0x43 0x54 0x45 0x44 0x0D 0x0A

0xAA 0x55 0x11 0x01 0x01 0x00 0x14 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xFA 0xFF
0xAA 0x55 0x11 0x01 0x04 0x00 0x14 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xFF 0xFF
0xAA 0x55 0x11 0x02 0x00 0x00 0x14 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xF9 0xFF

0x2B 0x44 0x49 0x53 0x43 0x4F 0x4E 0x4E 0x45 0x43 0x54 0x45 0x44 0x0D 0x0A
*/
uint8_t neeyCmdConnect[12] PROGMEM = {0x2B, 0x43, 0x4F, 0x4E, 0x4E, 0x45, 0x43, 0x54, 0x45, 0x44, 0x0D, 0x0A};
uint8_t neeyCmdDisconnect[15] PROGMEM = {0x2B, 0x44, 0x49, 0x53, 0x43, 0x4F, 0x4E, 0x4E, 0x45, 0x43, 0x54, 0x45, 0x44, 0x0D, 0x0A};
uint8_t neeyCmd1[20] PROGMEM = {0xAA, 0x55, 0x11, 0x01, 0x01, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0xFF};
uint8_t neeyCmd2[20] PROGMEM = {0xAA, 0x55, 0x11, 0x01, 0x04, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF};
uint8_t neeyCmd3[20] PROGMEM = {0xAA, 0x55, 0x11, 0x02, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF9, 0xFF};

uint8_t neeyCmd4[20] PROGMEM = {0xAA, 0x55, 0x11, 0x01, 0x02, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0xFF};

bool NeeySerial_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData)
{
  bool ret = true;
  mDevData = devData;
  mPort = port;
  u8_mDevNr = devNr;
  


  uint8_t response[300];


  /*sendDataToBms(neeyCmdConnect,12);
  vTaskDelay(pdMS_TO_TICKS(50));

  sendDataToBms(neeyCmd1,20);
  vTaskDelay(pdMS_TO_TICKS(50));

  sendDataToBms(neeyCmd2,20);
  vTaskDelay(pdMS_TO_TICKS(50));*/

   //Buffer leeren
  for (unsigned long clearRxBufTime = millis(); millis()-clearRxBufTime<100;)
  {
    if(port->available()) port->read();
    else break;
  }

  /*sendDataToBms(neeyCmd3,20);
  vTaskDelay(pdMS_TO_TICKS(50));*/

  bscSerial->sendSerialData(mPort, u8_mDevNr, neeyCmd4, 20);
  vTaskDelay(pdMS_TO_TICKS(50));

  //Auf antwort warten
  if(recvAnswer(response))
  {
    message2Log(response,0,100);
  }

  //sendDataToBms(neeyCmdDisconnect,15);

  return ret;
}


static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint32_t u32_lStartTime=millis();
  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>500)
    {

      #ifndef UTEST_RESTAPI
      BSC_LOGE(TAG,"Timeout: available=%i", mPort->available());
      #endif
      #ifdef SEPLOS_DEBUG
      String recvBytes="";
      for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
      {
        recvBytes+="0x";
        recvBytes+=String(p_lRecvBytes[x],16);
        recvBytes+=" ";
      }
      BSC_LOGD(TAG,"Timeout: RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
      #endif
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() >= 300)
    {
      for(uint16_t i=0;i<300;i++)
      {
        p_lRecvBytes[i]=mPort->read();
      }
      return true;
    }
    else vTaskDelay(pdMS_TO_TICKS(25));



  }


  return false;
}


static void message2Log(uint8_t * t_message, uint8_t address, uint8_t len)
{
  String recvBytes="";
  uint8_t u8_logByteCount=0;
  BSC_LOGI(TAG,"Dev=%i, RecvBytes=%i",address, len);
  for(uint8_t x=0;x<len;x++)
  {
    u8_logByteCount++;
    recvBytes+="0x";
    recvBytes+=String(t_message[x],16);
    recvBytes+=" ";
    if(u8_logByteCount==20)
    {
      BSC_LOGI(TAG,"%s",recvBytes.c_str());
      recvBytes="";
      u8_logByteCount=0;
    }
  }
  BSC_LOGI(TAG,"%s",recvBytes.c_str());
  //log_print_buf(p_lRecvBytes, u8_lRecvBytesCnt);
}
