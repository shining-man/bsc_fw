// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/JbdBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "debug.h"

Stream *mPort;
uint8_t u8_mDevNr, u8_mTxEnRS485pin;

enum SM_readData {SEARCH_START, RECV_DATA};

uint8_t basicMsg[] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };
uint8_t cellMsg[]  = { 0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77 };


void      JbdBms_sendMessage(uint8_t *sendMsg);
bool      JbdBms_recvAnswer(uint8_t * t_outMessage);
void      JbdBms_parseBasicMessage(uint8_t * t_message);
void      JbdBms_parseCellVoltageMessage(uint8_t * t_message);

uint16_t  JbdBms_convertToUint16(int highbyte, int lowbyte);
int16_t   JbdBms_convertToInt16(int highbyte, int lowbyte);

bool      JbdBms_checkCrc(uint8_t *recvMsg);
uint16_t  JbdBms_calcCrc(uint8_t *recvMsg);


bool JbdBms_readBmsData(Stream *port, uint8_t devNr, uint8_t txEnRS485pin)
{
  bool bo_lRet=true;
  mPort = port;
  u8_mDevNr = devNr;
  u8_mTxEnRS485pin = txEnRS485pin;
  uint8_t response[JBDBMS_MAX_ANSWER_LEN];

  JbdBms_sendMessage(basicMsg);
  if(JbdBms_recvAnswer(response))
  {
    JbdBms_parseBasicMessage(response);

    //mqtt
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/totalVoltage", getBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr));
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/totalCurrent", getBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr));
  }
  else
  {
    debugPrintf("sendReqBasicMessage checksum wrong\n");
    bo_lRet=false;
  }
 
  JbdBms_sendMessage(cellMsg);
  if(JbdBms_recvAnswer(response))
  {
    JbdBms_parseCellVoltageMessage(response);
  }
  else
  {
    debugPrintf("sendCellMessage checksum wrong\n");
    bo_lRet=false;
  }

  if(bo_lRet==false) return bo_lRet;
  
  setBmsLastDataMillis(BT_DEVICES_COUNT+u8_mDevNr,millis());

  return bo_lRet;  
}


void JbdBms_sendMessage(uint8_t *sendMsg)
{
  if(u8_mTxEnRS485pin>0) digitalWrite(u8_mTxEnRS485pin, HIGH); 
  usleep(20);
  mPort->write(sendMsg, 7);
  mPort->flush();  
  //usleep(50);
  if(u8_mTxEnRS485pin>0) digitalWrite(u8_mTxEnRS485pin, LOW); 
}


bool JbdBms_recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvDataLen;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START;
  u8_lRecvBytesCnt=0;
  u8_lRecvDataLen=0xFF;

  for(;;)
  {
    //Timeout
    if(millis()-u32_lStartTime > 200) 
    {
      debugPrintf("Timeout: u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i\n",u8_lRecvDataLen, u8_lRecvBytesCnt);
      //p_lRecvBytes[u8_lRecvBytesCnt]=0;
      //debugPrintf("buffer:%s",p_lRecvBytes);
      for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
      {
        debugPrint(String(p_lRecvBytes[x]));
        debugPrint(" ");
      }
      debugPrintln("");
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
            SMrecvState=RECV_DATA;
          }
          break;

        case RECV_DATA:
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
  if(JbdBms_checkCrc(p_lRecvBytes)==false) return false;

  return true;
}

float    f_mTotalVoltageOld=0xFFFF;
uint16_t u16_mBalanceCapacityOld=0xFFFF;
uint32_t u32_mChargeMAh=0;
uint32_t u32_mDischargeMAh=0;
uint32_t mqttSendeTimer=0;
void JbdBms_parseBasicMessage(uint8_t * t_message)
{
  float f_lTotalVoltage = (float)JbdBms_convertToUint16(t_message[JBD_BYTE_TOTAL_VOLTAGE], t_message[JBD_BYTE_TOTAL_VOLTAGE+1])/100;
  if(f_lTotalVoltage<(f_mTotalVoltageOld*0.9))
  {
    //Batteriespannung mehr als 10% gegenüber der letzten Messung gesunken
    f_mTotalVoltageOld=f_lTotalVoltage;
    return;
  }
  f_mTotalVoltageOld=f_lTotalVoltage;

  setBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr, f_lTotalVoltage);
  setBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr, ((float)JbdBms_convertToInt16(t_message[JBD_BYTE_CURRENT], t_message[JBD_BYTE_CURRENT+1])/100));
  setBmsChargePercentage(BT_DEVICES_COUNT+u8_mDevNr, t_message[JBD_BYTE_RSOC]);
  setBmsErrors(BT_DEVICES_COUNT+u8_mDevNr, JbdBms_convertToUint16(t_message[JBD_BYTE_CURRENT_ERRORS], t_message[JBD_BYTE_CURRENT_ERRORS+1]));
  for(uint8_t n=0;n<t_message[JBD_BYTE_NTC_NUMBER];n++)
  {
    if(n>=3) break; //Abbrechen da das Array in den BmsDaten nicht größer ist
    setBmsTempature(BT_DEVICES_COUNT+u8_mDevNr,n, (((float)JbdBms_convertToUint16(t_message[JBD_BYTE_NTCn+n*2], t_message[JBD_BYTE_NTCn+n*2+1])) - 2731) / 10.00f);
  }

  uint16_t u16_lBalanceCapacity = JbdBms_convertToUint16(t_message[JBD_BYTE_BALANCE_CAPACITY], t_message[JBD_BYTE_BALANCE_CAPACITY+1]); //10mAH

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
  

  if(millis()>(mqttSendeTimer+10000))
  {
    uint16_t /*u16_lBalanceCapacity,*/ u16_lFullCapacity, u16_lCycle, u16_lBalanceStatus, u16_lFetStatus;
    u16_lFullCapacity = JbdBms_convertToUint16(t_message[JBD_BYTE_FULL_CAPACITY], t_message[JBD_BYTE_FULL_CAPACITY+1]); //10mAH
    u16_lCycle = JbdBms_convertToUint16(t_message[JBD_BYTE_CYCLE], t_message[JBD_BYTE_CYCLE+1]); //
    u16_lBalanceStatus = JbdBms_convertToUint16(t_message[JBD_BYTE_BALANCE_STATUS], t_message[JBD_BYTE_BALANCE_STATUS+1]); 
    //uint16_t u16_lBalanceStatus2 = JbdBms_convertToUint16(t_message[JBD_BYTE_BALANCE_STATUS_2], t_message[JBD_BYTE_BALANCE_STATUS_2+1]); 
    u16_lFetStatus = t_message[JBD_BYTE_FET_STATUS]; 
    //uint16_t u16_lBatterySeries = t_message[JBD_BYTE_BATTERY_SERIES]; 
    //JBD_BYTE_PRODUCTION_DATE
    //JBD_BYTE_SOFTWARE_VERSION

    //Nachrichten senden
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/BalanceCapacity", u16_lBalanceCapacity);
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/FullCapacity", u16_lFullCapacity);
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/Cycle", u16_lCycle);
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/BalanceStatus", u16_lBalanceStatus);
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/FetStatus", u16_lFetStatus);

    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/ChargedEnergy", u32_mChargeMAh);
    mqttPublish("bms/"+String(BT_DEVICES_COUNT+u8_mDevNr)+"/DischargedEnergy", u32_mDischargeMAh);

    mqttSendeTimer=millis();
  }

}


void JbdBms_parseCellVoltageMessage(uint8_t * t_message) 
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
    u16_lZellVoltage = (uint16_t)JbdBms_convertToUint16(t_message[i*2+offset], t_message[i*2+1+offset]);
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


uint16_t JbdBms_convertToUint16(int highbyte, int lowbyte)
{
  return ((highbyte<<8) | lowbyte);
}


int16_t JbdBms_convertToInt16(int highbyte, int lowbyte)
{
  int16_t value = JbdBms_convertToUint16(highbyte, lowbyte);
  if (value & 0x8000){value=((~value)*-1);}// Wenn negativ
  return value;
}


bool JbdBms_checkCrc(uint8_t *recvMsg)
{
  uint8_t u8_lDataLen = recvMsg[3];
  JbdBms_calcCrc(recvMsg);
  uint16_t checkSumRecv = (recvMsg[4+u8_lDataLen]<<8) | (recvMsg[4+u8_lDataLen+1]);

  if (checkSumRecv != JbdBms_calcCrc(recvMsg)) return false;
  return true;
}


uint16_t JbdBms_calcCrc(uint8_t *recvMsg)
{
	uint8_t u8_lDataLen = recvMsg[3];
	uint16_t u8_lSum = 0;

	for (int i=4; i<u8_lDataLen+4; i++)
	{
		u8_lSum = u8_lSum + recvMsg[i];
	}
	return (u8_lSum+u8_lDataLen-1) ^ 0xFFFF;
}











