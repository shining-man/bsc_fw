// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/JkBms.h"
#include "BmsData.h"
#include "mqtt_t.h"

Stream *mPortJk;
uint8_t u8_mDevNrJk, u8_mTxEnRS485pinJk;
uint16_t u16_mLastRecvBytesCntJk;

enum SM_readData {SEARCH_START_BYTE1, SEARCH_START_BYTE2, LEN1, LEN2, RECV_DATA};

uint8_t getDataMsg[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};


void      JkBms_sendMessage(uint8_t *sendMsg);
bool      JkBms_recvAnswer(uint8_t * t_outMessage);
void      JkBms_parseData(uint8_t * t_message);



bool JkBms_readBmsData(Stream *port, uint8_t devNr, uint8_t txEnRS485pin)
{
  bool bo_lRet=true;
  mPortJk = port;
  u8_mDevNrJk = devNr-1;
  u8_mTxEnRS485pinJk = txEnRS485pin;
  uint8_t response[JKBMS_MAX_ANSWER_LEN];

  JkBms_sendMessage(getDataMsg);
  if(JkBms_recvAnswer(response))
  {
    JkBms_parseData(response);

    //mqtt
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNrJk)+"/totalVoltage", getBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNrJk));
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNrJk)+"/totalCurrent", getBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNrJk));
  }
  else
  {
    Serial.printf("bmsData checksum wrong\n");
    bo_lRet=false;
  }


  if(bo_lRet==false) return bo_lRet;
  
  setBmsLastDataMillis(BT_DEVICES_COUNT+u8_mDevNrJk,millis());

  return bo_lRet;  
}


void JkBms_sendMessage(uint8_t *sendMsg)
{
  if(u8_mTxEnRS485pinJk>0) digitalWrite(u8_mTxEnRS485pinJk, HIGH); 
  usleep(20);
  mPortJk->write(sendMsg, 7);
  mPortJk->flush();  
  //usleep(50);
  if(u8_mTxEnRS485pinJk>0) digitalWrite(u8_mTxEnRS485pinJk, LOW); 
}


bool JkBms_recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte;
  uint16_t u16_lRecvDataLen;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START_BYTE1;
  u16_mLastRecvBytesCntJk=0;
  u16_lRecvDataLen=0xFFFF;
  uint16_t crc=0;

  for(;;)
  {
    //Timeout
    if(millis()-u32_lStartTime > 200) 
    {
      Serial.printf("Timeout: u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i\n",u16_lRecvDataLen, u16_mLastRecvBytesCntJk);
      for(uint16_t x=0;x<u16_mLastRecvBytesCntJk;x++)
      {
        Serial.print(String(p_lRecvBytes[x]));
        Serial.print(" ");
      }
      Serial.println("");
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPortJk->available() > 0)
    {
      u8_lRecvByte = mPortJk->read();
      if(u16_mLastRecvBytesCntJk<u16_lRecvDataLen-4){crc += u8_lRecvByte;}

      switch (SMrecvState)  {
        case SEARCH_START_BYTE1:
          if (u8_lRecvByte == 0x4E){SMrecvState=SEARCH_START_BYTE2;}
          break;

        case SEARCH_START_BYTE2:
          if (u8_lRecvByte == 0x57){SMrecvState=LEN1;}
          break;

        case LEN1:
          p_lRecvBytes[u16_mLastRecvBytesCntJk]=u8_lRecvByte;
          u16_mLastRecvBytesCntJk++;
          u16_lRecvDataLen=(u8_lRecvByte<<8);
          SMrecvState=LEN2;
          break;

        case LEN2:
          p_lRecvBytes[u16_mLastRecvBytesCntJk]=u8_lRecvByte;
          u16_mLastRecvBytesCntJk++;
          u16_lRecvDataLen|=u8_lRecvByte;
          SMrecvState=RECV_DATA;
          break;

        case RECV_DATA:
          p_lRecvBytes[u16_mLastRecvBytesCntJk]=u8_lRecvByte;
          u16_mLastRecvBytesCntJk++;
          break;
      
        default:
          break;
        }
    }

    if(u16_mLastRecvBytesCntJk==u16_lRecvDataLen) break; //Recv Pakage complete
  }

  if(p_lRecvBytes[u16_mLastRecvBytesCntJk-5]!=0x68) return false; //letztes Byte vor der crc muss 0x68 sein

  //Überprüfe Cheksum
	uint8_t crcB3 = (crc >> 8) & 0xFF;  // Byte 3
  uint8_t crcB4 = crc & 0xFF;         // Byte 4
  if(p_lRecvBytes[u16_mLastRecvBytesCntJk-2]!=crcB3 && p_lRecvBytes[u16_mLastRecvBytesCntJk-1]!=crcB4) return false; 

  return true;
}




uint32_t mqttSendeTimer_jk=0;
void JkBms_parseData(uint8_t * t_message)
{
  uint32_t u32_lBalanceCapacity=0;
  uint16_t u16_lCycle=0;

  // Variables for 0x79
  uint32_t u32_lZellVoltage=0;
  uint8_t u8_lNumOfCells = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint16_t u16_lZellDifferenceVoltage = 0;
  uint16_t u16_lCellSum = 0;
  uint16_t u16_lZellVoltage = 0;
  uint16_t u16_lCellLow = 0xFFFF; 
  uint16_t u16_lCellHigh = 0x0;

  for(uint16_t i=9; i<u16_mLastRecvBytesCntJk-5;)
  {
    switch (t_message[i])  {
      case 0x79: //Cell voltage

        u8_lNumOfCells = t_message[i+1]/3;
        i+=2;
        for(uint8_t n=0;n<u8_lNumOfCells;n++)
        {
          if(t_message[i]>u8_lNumOfCells) break;
          u32_lZellVoltage = ((t_message[i+1]<<8) | t_message[i+2]);
          setBmsCellVoltage(BT_DEVICES_COUNT+u8_mDevNrJk,n, u32_lZellVoltage);
          i+=3;

          u16_lCellSum += u16_lZellVoltage;

          if (u16_lZellVoltage > u16_lCellHigh){u16_lCellHigh = u16_lZellVoltage;}
          if (u16_lZellVoltage < u16_lCellLow){u16_lCellLow = u16_lZellVoltage;}

          u16_lZellMinVoltage = u16_lCellLow;
          u16_lZellMaxVoltage = u16_lCellHigh;
          u16_lZellDifferenceVoltage = u16_lCellHigh - u16_lCellLow; 
        }
        
        setBmsMaxCellVoltage(BT_DEVICES_COUNT+u8_mDevNrJk, u16_lCellHigh);
        setBmsMinCellVoltage(BT_DEVICES_COUNT+u8_mDevNrJk, u16_lCellLow);
        setBmsAvgVoltage(BT_DEVICES_COUNT+u8_mDevNrJk, (float)(u16_lCellSum/u8_lNumOfCells));
        setBmsMaxCellDifferenceVoltage(BT_DEVICES_COUNT+u8_mDevNrJk,(float)(u16_lZellDifferenceVoltage));

        break;

      case 0x80: // Read tube temp. 
        setBmsTempature(BT_DEVICES_COUNT+u8_mDevNrJk, 0, ((uint16_t)t_message[i+1] << 8 | t_message[i+2]));
        i+=3;
        break; 

      case 0x81: // Battery inside temp
        setBmsTempature(BT_DEVICES_COUNT+u8_mDevNrJk, 1, ((uint16_t)t_message[i+1] << 8 | t_message[i+2]));
        i+=3;
        break; 

      case 0x82: // Battery temp
        setBmsTempature(BT_DEVICES_COUNT+u8_mDevNrJk, 2, ((uint16_t)t_message[i+1] << 8 | t_message[i+2]));
        i+=3;
        break;   

      case 0x83: // Total Batery Voltage
        setBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNrJk, (float)(((uint16_t)t_message[i+1] << 8 | t_message[i+2])*0.01));
        i+=3;
        break; 

      case 0x84: // Current
        setBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNrJk, (float)(((uint16_t)t_message[i+1] << 8 | t_message[i+2])));
        i+=3;
        break; 

      case 0x85: // Remaining Battery Capazity 
        setBmsChargePercentage(BT_DEVICES_COUNT+u8_mDevNrJk, t_message[i+1]); //in %
        i+=2;
        break;

      case 0x87: // Cycle
        u16_lCycle = ((uint16_t)t_message[i+1] << 8 | t_message[i+2]);
        i+=3;
        break;   

      case 0x89: // Total Battery cycle Capacity
        u32_lBalanceCapacity = (((uint16_t)t_message[i+1] << 24 | t_message[i+2] << 16 | t_message[i+3] << 8 | t_message[i+4])); 
        i+=5;
        break; 

      case 0x8b: // Battery_Warning_Massage
        setBmsErrors(BT_DEVICES_COUNT+u8_mDevNrJk, (((uint16_t)t_message[i+1] << 8 | t_message[i+2])));
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
      case 0x8C:
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


  if(millis()>(mqttSendeTimer_jk+10000))
  {
    //Nachrichten senden
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNrJk)+"/BalanceCapacity", u32_lBalanceCapacity);
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNrJk)+"/Cycle", u16_lCycle);

    mqttSendeTimer_jk=millis();
  }

}

