// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/JkBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include <devices/jkbms/JkBmsTypes.hpp>

static const char *TAG = "JK_BMS";

static Stream *mPort;
static uint8_t u8_mDevNr;
static uint16_t u16_mLastRecvBytesCnt;

enum SM_readData {SEARCH_START_BYTE1, SEARCH_START_BYTE2, LEN1, LEN2, SEARCH_END};

static uint8_t getDataMsg[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};

//
static void sendMessage(uint8_t *sendMsg);
static bool recvAnswer(uint8_t * t_outMessage);
static void parseData(uint8_t * t_message);

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;
static serialDevData_s *mDevData;


bool JkBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool bo_lRet=true;
  mDevData=devData;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn=callback;
  uint8_t response[JKBMS_MAX_ANSWER_LEN];

  #ifdef JK_DEBUG
  BSC_LOGD(TAG,"Serial %i send",u8_mDevNr);
  #endif
  sendMessage(getDataMsg);
  if(recvAnswer(response))
  {
    parseData(response);

    //mqtt
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr));
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr));
  }
  else bo_lRet=false;

  if(devNr>=2) callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxTxDisable);
  return bo_lRet;
}


static void sendMessage(uint8_t *sendMsg)
{
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
  usleep(20);
  mPort->write(sendMsg, 21);
  mPort->flush();
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn);
}


static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_CyclesWithoutData;
  uint16_t u16_lRecvDataLen;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START_BYTE1;
  u16_mLastRecvBytesCnt=0;
  u16_lRecvDataLen=0xFFFF;
  u8_CyclesWithoutData=0;
  uint16_t crc=0;

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>200)
    {
      BSC_LOGI(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i",u8_mDevNr, u16_lRecvDataLen, u16_mLastRecvBytesCnt);
      /*for(uint16_t x=0;x<u16_mLastRecvBytesCnt;x++)
      {
        BSC_LOGD(TAG,"Byte=%i: %i",x, String(p_lRecvBytes[x]));
      }*/
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();
      if(u16_mLastRecvBytesCnt<u16_lRecvDataLen-4){crc += u8_lRecvByte;}

      switch (SMrecvState)  {
        case SEARCH_START_BYTE1:
          if (u8_lRecvByte == 0x4E){SMrecvState=SEARCH_START_BYTE2;}
          break;

        case SEARCH_START_BYTE2:
          if (u8_lRecvByte == 0x57){SMrecvState=LEN1;}
          break;

        case LEN1:
          p_lRecvBytes[u16_mLastRecvBytesCnt]=u8_lRecvByte;
          u16_mLastRecvBytesCnt++;
          u16_lRecvDataLen=(u8_lRecvByte<<8);
          SMrecvState=LEN2;
          break;

        case LEN2:
          p_lRecvBytes[u16_mLastRecvBytesCnt]=u8_lRecvByte;
          u16_mLastRecvBytesCnt++;
          u16_lRecvDataLen|=u8_lRecvByte;
          SMrecvState=SEARCH_END;
          break;

        case SEARCH_END:
          p_lRecvBytes[u16_mLastRecvBytesCnt]=u8_lRecvByte;
          u16_mLastRecvBytesCnt++;
          break;

        default:
          break;
        }
      u8_CyclesWithoutData=0;
    }
    else if (u16_mLastRecvBytesCnt==0) vTaskDelay(pdMS_TO_TICKS(10)); // Wenn noch keine Daten empfangen wurden, dann setze den Task 10ms aus
    else if (u16_mLastRecvBytesCnt>0 && u8_CyclesWithoutData>10) vTaskDelay(pdMS_TO_TICKS(10)); // Wenn trotz empfangenen Daten 10ms wieder nichts empfangen wurde, dann setze den Task 10ms aus
    else // Wenn in diesem Zyklus keine Daten Empfangen wurden, dann setze den Task 1ms aus
    {
      u8_CyclesWithoutData++;
    vTaskDelay(pdMS_TO_TICKS(1));
    }

    if(u16_mLastRecvBytesCnt==u16_lRecvDataLen) break; //Recv Pakage complete
    if(u16_mLastRecvBytesCnt>=JKBMS_MAX_ANSWER_LEN) return false; //Answer too long!
  }


  #ifdef JK_DEBUG
  if(u16_mLastRecvBytesCnt>5)
  {
    BSC_LOGD(TAG,"RecvBytes=%i, %i, %i, %i, %i, %i", u16_mLastRecvBytesCnt, p_lRecvBytes[u16_mLastRecvBytesCnt-5], p_lRecvBytes[u16_mLastRecvBytesCnt-4],
      p_lRecvBytes[u16_mLastRecvBytesCnt-3], p_lRecvBytes[u16_mLastRecvBytesCnt-2], p_lRecvBytes[u16_mLastRecvBytesCnt-1]);
  }
  #endif

  if(p_lRecvBytes[u16_mLastRecvBytesCnt-5]!=0x68) return false; //letztes Byte vor der crc muss 0x68 sein

  //Überprüfe Cheksum
	uint8_t crcB3 = (crc >> 8) & 0xFF;  // Byte 3
  uint8_t crcB4 = crc & 0xFF;         // Byte 4

  #ifdef JK_DEBUG
  BSC_LOGD(TAG,"crc=%i %i", crcB3, crcB4);
  #endif
  if(p_lRecvBytes[u16_mLastRecvBytesCnt-2]!=crcB3 && p_lRecvBytes[u16_mLastRecvBytesCnt-1]!=crcB4) return false;

  return true;
}


void parseData(uint8_t * t_message)
{
  int16_t  i16_lTmpValue;
  uint16_t u16_lTmpValue;
  uint32_t u32_lTmpValue;
  uint32_t u32_lCycleCapacity=0;
  uint16_t u16_lCycle=0;

  // Variables for 0x79
  uint8_t  u8_lNumOfCells = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint16_t u16_lZellDifferenceVoltage = 0;
  uint8_t  u8_lZellNumberMinVoltage = 0;
  uint8_t  u8_lZellNumberMaxVoltage = 0;
  uint16_t u16_lCellSum = 0;
  uint16_t u16_lZellVoltage = 0;
  uint16_t u16_lCellLow = 0xFFFF;
  uint16_t u16_lCellHigh = 0x0;

  for(uint16_t i=9; i<u16_mLastRecvBytesCnt-5;)
  {
    switch (t_message[i])  {
      case 0x79: //Cell voltage

        u8_lNumOfCells = t_message[i+1]/3;
        #ifdef JK_DEBUG
        BSC_LOGD(TAG,"NumOfCells=%i", u8_lNumOfCells);
        #endif
        i+=2;
        for(uint8_t n=0;n<u8_lNumOfCells;n++)
        {
          if(t_message[i]>u8_lNumOfCells)
          {
            #ifdef JK_DEBUG
            BSC_LOGD(TAG,"n>NOC: %i, %i, %i, %i", i, n, u8_lNumOfCells, t_message[i]);
            #endif
            break;
          }
          u16_lZellVoltage = ((t_message[i+1]<<8) | t_message[i+2]);
          setBmsCellVoltage(BT_DEVICES_COUNT+u8_mDevNr,n, u16_lZellVoltage);
          i+=3;

          u16_lCellSum += u16_lZellVoltage;

          if (u16_lZellVoltage > u16_lCellHigh)
          {
            u16_lCellHigh = u16_lZellVoltage;
            u8_lZellNumberMaxVoltage=n;
          }
          if (u16_lZellVoltage < u16_lCellLow)
          {
            u16_lCellLow = u16_lZellVoltage;
            u8_lZellNumberMinVoltage=n;
          }

          u16_lZellMinVoltage = u16_lCellLow;
          u16_lZellMaxVoltage = u16_lCellHigh;
          u16_lZellDifferenceVoltage = u16_lCellHigh - u16_lCellLow;

          #ifdef JK_DEBUG
          BSC_LOGD(TAG,"V%i=%i",n, u16_lZellVoltage);
          #endif
        }

        setBmsMaxCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellHigh);
        setBmsMinCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellLow);
        setBmsMaxVoltageCellNumber(BT_DEVICES_COUNT+u8_mDevNr, u8_lZellNumberMaxVoltage);
        setBmsMinVoltageCellNumber(BT_DEVICES_COUNT+u8_mDevNr, u8_lZellNumberMinVoltage);
        setBmsAvgVoltage(BT_DEVICES_COUNT+u8_mDevNr, (float)(u16_lCellSum/u8_lNumOfCells));
        setBmsMaxCellDifferenceVoltage(BT_DEVICES_COUNT+u8_mDevNr,(float)(u16_lZellDifferenceVoltage));

        break;

      case 0x80: // Read tube temp.
        setBmsTempature(BT_DEVICES_COUNT+u8_mDevNr, 0, ((uint16_t)t_message[i+1] << 8 | t_message[i+2]) );
        #ifdef JK_DEBUG
        BSC_LOGD(TAG,"0x80=%i",((uint16_t)t_message[i+1] << 8 | t_message[i+2]));
        #endif
        i+=3;
        break;

      case 0x81: // Battery inside temp
        setBmsTempature(BT_DEVICES_COUNT+u8_mDevNr, 1, ((uint16_t)t_message[i+1] << 8 | t_message[i+2]) );
        #ifdef JK_DEBUG
        BSC_LOGD(TAG,"0x81=%i",((uint16_t)t_message[i+1] << 8 | t_message[i+2]));
        #endif
        i+=3;
        break;

      case 0x82: // Battery temp
        setBmsTempature(BT_DEVICES_COUNT+u8_mDevNr, 2, ((uint16_t)t_message[i+1] << 8 | t_message[i+2]) );
        #ifdef JK_DEBUG
        BSC_LOGD(TAG,"0x82=%i",((uint16_t)t_message[i+1] << 8 | t_message[i+2]));
        #endif
        i+=3;
        break;

      case 0x83: // Total Batery Voltage
        setBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr, (float)(((uint16_t)t_message[i+1] << 8 | t_message[i+2])*0.01));
        #ifdef JK_DEBUG
        BSC_LOGD(TAG,"0x83=%f",(float)(((uint16_t)t_message[i+1] << 8 | t_message[i+2])*0.01));
        #endif
        i+=3;
        break;

      case 0x84: // Current
      	i16_lTmpValue = (uint16_t)(t_message[i+1]<<8)|t_message[i+2];
        if (i16_lTmpValue & 0x8000){i16_lTmpValue = (i16_lTmpValue & 0x7fff);}
        else {i16_lTmpValue *= -1;} // Wenn negativ

        setBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr, (float)i16_lTmpValue*0.01);
        #ifdef JK_DEBUG
        BSC_LOGD(TAG,"0x84:%i, %i, %f",t_message[i+1], t_message[i+2], (float)i16_lTmpValue*0.01);
        #endif
        i+=3;
        break;

      case 0x85: // Remaining Battery Capazity
        setBmsChargePercentage(BT_DEVICES_COUNT+u8_mDevNr, t_message[i+1]); //in %
        i+=2;
        break;

      case 0x87: // Cycle
        u16_lCycle = ((uint16_t)t_message[i+1] << 8 | t_message[i+2]);
        i+=3;
        break;

      case 0x89: // Total Battery cycle Capacity
        u32_lCycleCapacity = (((uint16_t)t_message[i+1] << 24 | t_message[i+2] << 16 | t_message[i+3] << 8 | t_message[i+4]));
        i+=5;
        break;

      case 0x8b: // Battery_Warning_Massage
      {
        const jkbms::JkBmsWarnMsg bmsWarnMsg = jkbms::JkBmsWarnMsg::from_int<uint16_t>(((static_cast<uint16_t>(t_message[i+1]) << 8) | t_message[i+2]));
        const BmsErrorStatus bmsErrorStat = bmsErrorFromMessage(bmsWarnMsg);
        //BSC_LOGI(TAG,"0x8b=%i, bmsErrorsBsc=%i",bmsWarnMsg.to_int<uint16_t>(), bmsErrorStat.to_int<uint32_t>());
        setBmsErrors(BT_DEVICES_COUNT+u8_mDevNr, bmsErrorStat);
        i+=3;
      } break;

      case 0x8C:  // Battery status information
        u16_lTmpValue = ((uint16_t)t_message[i+1] << 8 | t_message[i+2]);
        if((u16_lTmpValue>>3)&0x01)
        {
          setBmsStateFETsCharge(BT_DEVICES_COUNT+u8_mDevNr,false);
          setBmsStateFETsDischarge(BT_DEVICES_COUNT+u8_mDevNr,false);
        }
        else
        {
          //Bit 0: charging on
          if(u16_lTmpValue&0x01) setBmsStateFETsCharge(BT_DEVICES_COUNT+u8_mDevNr,true);
          else setBmsStateFETsCharge(BT_DEVICES_COUNT+u8_mDevNr,false);

          //Bit 1: discharge on
          if((u16_lTmpValue>>1)&0x01) setBmsStateFETsDischarge(BT_DEVICES_COUNT+u8_mDevNr,true);
          else setBmsStateFETsDischarge(BT_DEVICES_COUNT+u8_mDevNr,false);

          //Bit 2: equilization on
          if((u16_lTmpValue>>2)&0x01) setBmsIsBalancingActive(BT_DEVICES_COUNT+u8_mDevNr,true);
          else setBmsIsBalancingActive(BT_DEVICES_COUNT+u8_mDevNr,false);
        }
        i+=3;
        break;

      case 0x86:
      case 0x9D:
      case 0xA9:
      case 0xAB:
      case 0xAC:
      case 0xAE:
      case 0xAF:
      case 0xB1:
      case 0xB3:
      case 0xB8:
        i+=2;
        break;

      case 0x8A:
      case 0x8E:
      case 0x8F:
      case 0x90:
      case 0x91:
      case 0x92:
      case 0x93:
      case 0x94:
      case 0x95:
      case 0x96:
      case 0x97:
      case 0x98:
      case 0x99:
      case 0x9A:
      case 0x9B:
      case 0x9c:
      case 0x9E:
      case 0x9F:
      case 0xA0:
      case 0xA1:
      case 0xA2:
      case 0xA3:
      case 0xA4:
      case 0xA5:
      case 0xA6:
      case 0xA7:
      case 0xA8:
      case 0xAD:
      case 0xB0:
        i+=3;
        break;

      case 0xAA:
      case 0xB5:
      case 0xB6:
      case 0xB9:
        i+=5;
        break;

      case 0xC0:
        i+=6;
        break;

      case 0xB4:
        i+=9;
        break;

      case 0xB2:
        i+=11;
        break;

      case 0xB7:
        i+=16;
        break;

      case 0xBA:
        i+=25;
        break;

      default:
        i++;
        break;

    }
  }


  if(mDevData->bo_sendMqttMsg)
  {
    //Nachrichten senden
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CYCLE_CAPACITY, -1, u32_lCycleCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);
  }

}

