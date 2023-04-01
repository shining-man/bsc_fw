// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/JbdBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

static const char *TAG = "JBD_BMS";

static Stream *mPort;
static uint8_t u8_mDevNr, u8_mTxEnRS485pin;

enum SM_readData {SEARCH_START, SEARCH_END};

static uint8_t basicMsg[] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };
static uint8_t cellMsg[]  = { 0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77 };

//
static float     f_mTotalVoltageOld=0xFFFF;
static uint16_t  u16_mBalanceCapacityOld=0xFFFF;
static uint32_t  u32_mChargeMAh=0;
static uint32_t  u32_mDischargeMAh=0;
static uint32_t  mqttSendeTimer=0;

//
static void      sendMessage(uint8_t *sendMsg);
static bool      recvAnswer(uint8_t * t_outMessage);
static void      parseBasicMessage(uint8_t * t_message);
static void      parseCellVoltageMessage(uint8_t * t_message);

static uint16_t  convertToUint16(int highbyte, int lowbyte);
static int16_t   convertToInt16(int highbyte, int lowbyte);

static bool      checkCrc(uint8_t *recvMsg);
static uint16_t  calcCrc(uint8_t *recvMsg);


bool JbdBms_readBmsData(Stream *port, uint8_t devNr, uint8_t txEnRS485pin, uint8_t u8_addData)
{
  bool bo_lRet=true;
  mPort = port;
  u8_mDevNr = devNr;
  u8_mTxEnRS485pin = txEnRS485pin;
  uint8_t response[JBDBMS_MAX_ANSWER_LEN];

  sendMessage(basicMsg);
  if(recvAnswer(response))
  {
    parseBasicMessage(response);

    //mqtt
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr));
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr));
  }
  else
  {
    ESP_LOGI(TAG,"sendReqBasicMessage checksum wrong");
    bo_lRet=false;
  }
 
  sendMessage(cellMsg);
  if(recvAnswer(response))
  {
    parseCellVoltageMessage(response);
  }
  else
  {
    ESP_LOGI(TAG,"sendCellMessage checksum wrong");
    bo_lRet=false;
  }

  if(bo_lRet==false) return bo_lRet;

  return bo_lRet;  
}


static void sendMessage(uint8_t *sendMsg)
{
  if(u8_mTxEnRS485pin>0) digitalWrite(u8_mTxEnRS485pin, HIGH); 
  usleep(20);
  mPort->write(sendMsg, 7);
  mPort->flush();  
  //usleep(50);
  if(u8_mTxEnRS485pin>0) digitalWrite(u8_mTxEnRS485pin, LOW); 
}


static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvDataLen;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START;
  u8_lRecvBytesCnt=0;
  u8_lRecvDataLen=0xFF;

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>200) 
    {
      ESP_LOGI(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
      /*for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
      {
        ESP_LOGD(TAG,"Byte=%i: %i",x, String(p_lRecvBytes[x]));
      }*/
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch (SMrecvState)  {
        case SEARCH_START:
          if (u8_lRecvByte == 0xDD)
          {
            p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
            u8_lRecvBytesCnt++;
            SMrecvState=SEARCH_END;
          }
          break;

        case SEARCH_END:
          p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
          if(u8_lRecvBytesCnt==3) u8_lRecvDataLen=p_lRecvBytes[u8_lRecvBytesCnt]; //Länge des zu empfangenden Pakets
          u8_lRecvBytesCnt++;
          break;
      
        default:
          break;
        }
    }

    if(u8_lRecvBytesCnt==4+u8_lRecvDataLen+3) break; //Recv Pakage complete
  }

  if(p_lRecvBytes[2]!=0x0) return false; //0x0 ok; 0x80 Fehler
  if(p_lRecvBytes[u8_lRecvBytesCnt-1]!=0x77) return false; //letztes Byte muss 0x77 sein

  //Überprüfe Cheksum
  if(checkCrc(p_lRecvBytes)==false) return false;

  return true;
}


static void parseBasicMessage(uint8_t * t_message)
{
  float f_lTotalVoltage = (float)convertToUint16(t_message[JBD_BYTE_TOTAL_VOLTAGE], t_message[JBD_BYTE_TOTAL_VOLTAGE+1])/100;
  if(f_lTotalVoltage<(f_mTotalVoltageOld*0.9))
  {
    //Batteriespannung mehr als 10% gegenüber der letzten Messung gesunken
    f_mTotalVoltageOld=f_lTotalVoltage;
    return;
  }
  f_mTotalVoltageOld=f_lTotalVoltage;

  setBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr, f_lTotalVoltage);
  setBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr, ((float)convertToInt16(t_message[JBD_BYTE_CURRENT], t_message[JBD_BYTE_CURRENT+1])/100));
  setBmsChargePercentage(BT_DEVICES_COUNT+u8_mDevNr, t_message[JBD_BYTE_RSOC]);
  setBmsErrors(BT_DEVICES_COUNT+u8_mDevNr, convertToUint16(t_message[JBD_BYTE_CURRENT_ERRORS], t_message[JBD_BYTE_CURRENT_ERRORS+1]));
  for(uint8_t n=0;n<t_message[JBD_BYTE_NTC_NUMBER];n++)
  {
    if(n>=3) break; //Abbrechen da das Array in den BmsDaten nicht größer ist
    setBmsTempature(BT_DEVICES_COUNT+u8_mDevNr,n, (((float)convertToUint16(t_message[JBD_BYTE_NTCn+n*2], t_message[JBD_BYTE_NTCn+n*2+1])) - 2731) / 10.00f);
  }

  uint16_t u16_lBalanceCapacity = convertToUint16(t_message[JBD_BYTE_BALANCE_CAPACITY], t_message[JBD_BYTE_BALANCE_CAPACITY+1]); //10mAH

  if(u16_mBalanceCapacityOld==0xFFFF)u16_mBalanceCapacityOld=u16_lBalanceCapacity;
  //Zähler zurücksetzen bevor sie ueberlaufen
  if((u32_mChargeMAh+(u16_lBalanceCapacity-u16_mBalanceCapacityOld)>ULONG_MAX) ||
    (u32_mDischargeMAh+(u16_mBalanceCapacityOld-u16_lBalanceCapacity)>ULONG_MAX))
  {
    u32_mChargeMAh=0;
    u32_mDischargeMAh=0;
  }
  if(u16_lBalanceCapacity>u16_mBalanceCapacityOld) //charge
  {
    u32_mChargeMAh = u32_mChargeMAh+(u16_lBalanceCapacity-u16_mBalanceCapacityOld);  
  }
  else if(u16_lBalanceCapacity<u16_mBalanceCapacityOld) //discharge
  {
    u32_mDischargeMAh = u32_mDischargeMAh+(u16_mBalanceCapacityOld-u16_lBalanceCapacity);  
  }
  u16_mBalanceCapacityOld=u16_lBalanceCapacity;
  

  if((millis()-mqttSendeTimer)>10000)
  {
    uint16_t u16_lFullCapacity, u16_lCycle, u16_lBalanceStatus, u16_lFetStatus;
    u16_lFullCapacity = convertToUint16(t_message[JBD_BYTE_FULL_CAPACITY], t_message[JBD_BYTE_FULL_CAPACITY+1]); //10mAH
    u16_lCycle = convertToUint16(t_message[JBD_BYTE_CYCLE], t_message[JBD_BYTE_CYCLE+1]); //
    u16_lBalanceStatus = convertToUint16(t_message[JBD_BYTE_BALANCE_STATUS], t_message[JBD_BYTE_BALANCE_STATUS+1]); 
    //uint16_t u16_lBalanceStatus2 = convertToUint16(t_message[JBD_BYTE_BALANCE_STATUS_2], t_message[JBD_BYTE_BALANCE_STATUS_2+1]); 

    u16_lFetStatus = t_message[JBD_BYTE_FET_STATUS]; 
    //Bit 0: charging
    if(u16_lFetStatus&0x01) setBmsStateFETsCharge(BT_DEVICES_COUNT+u8_mDevNr,true);
    else setBmsStateFETsCharge(BT_DEVICES_COUNT+u8_mDevNr,false);
    //Bit 1: discharge
    if((u16_lFetStatus>>1)&0x01) setBmsStateFETsDischarge(BT_DEVICES_COUNT+u8_mDevNr,true);
    else setBmsStateFETsDischarge(BT_DEVICES_COUNT+u8_mDevNr,false);

    //uint16_t u16_lBatterySeries = t_message[JBD_BYTE_BATTERY_SERIES]; 
    //JBD_BYTE_PRODUCTION_DATE
    //JBD_BYTE_SOFTWARE_VERSION

    //Nachrichten senden
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_BALANCE_STATUS, -1, u16_lBalanceStatus);

    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CHARGED_ENERGY, -1, u32_mChargeMAh);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_DISCHARGED_ENERGY, -1, u32_mDischargeMAh);

    mqttSendeTimer=millis();
  }

}


static void parseCellVoltageMessage(uint8_t * t_message) 
{
  uint8_t u8_lNumOfCells = 0;
  uint16_t u16_lZellVoltage = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint16_t u16_lZellDifferenceVoltage = 0;

  uint16_t u16_lCellSum = 0;

  uint16_t u16_lCellLow = 0xFFFF; 
  uint16_t u16_lCellHigh = 0x0;

  u8_lNumOfCells = t_message[3] / 2;  //Data length / 2 => number of cells


  byte offset = 4; 
  for (byte i=0; i<u8_lNumOfCells; i++) 
  {
    u16_lZellVoltage = (uint16_t)convertToUint16(t_message[i*2+offset], t_message[i*2+1+offset]);
    if(u16_lZellVoltage<=500 || u16_lZellVoltage>5000) continue;
    setBmsCellVoltage(BT_DEVICES_COUNT+u8_mDevNr,i, (float)(u16_lZellVoltage));

    u16_lCellSum += u16_lZellVoltage;

    if (u16_lZellVoltage > u16_lCellHigh){u16_lCellHigh = u16_lZellVoltage;}
    if (u16_lZellVoltage < u16_lCellLow){u16_lCellLow = u16_lZellVoltage;}

    u16_lZellMinVoltage = u16_lCellLow;
    u16_lZellMaxVoltage = u16_lCellHigh;
    u16_lZellDifferenceVoltage = u16_lCellHigh - u16_lCellLow; 
  }
  
  setBmsMaxCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellHigh);
  setBmsMinCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellLow);
  setBmsAvgVoltage(BT_DEVICES_COUNT+u8_mDevNr, (float)(u16_lCellSum/u8_lNumOfCells));
  setBmsMaxCellDifferenceVoltage(BT_DEVICES_COUNT+u8_mDevNr,(float)(u16_lZellDifferenceVoltage));
}


static uint16_t convertToUint16(int highbyte, int lowbyte)
{
  return ((highbyte<<8) | lowbyte);
}


static int16_t convertToInt16(int highbyte, int lowbyte)
{
  int16_t value = convertToUint16(highbyte, lowbyte);
  if (value & 0x8000){value=((~value)*-1);}// Wenn negativ
  return value;
}


static bool checkCrc(uint8_t *recvMsg)
{
  uint8_t u8_lDataLen = recvMsg[3];
  calcCrc(recvMsg);
  uint16_t checkSumRecv = (recvMsg[4+u8_lDataLen]<<8) | (recvMsg[4+u8_lDataLen+1]);

  if (checkSumRecv != calcCrc(recvMsg)) return false;
  return true;
}


static uint16_t calcCrc(uint8_t *recvMsg)
{
	uint8_t u8_lDataLen = recvMsg[3];
	uint16_t u8_lSum = 0;

	for (int i=4; i<u8_lDataLen+4; i++)
	{
		u8_lSum = u8_lSum + recvMsg[i];
	}
	return (u8_lSum+u8_lDataLen-1) ^ 0xFFFF;
}











