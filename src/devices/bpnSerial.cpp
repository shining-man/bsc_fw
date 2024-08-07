// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/JbdBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include "WebSettings.h"

static const char *TAG = "BPN";

static Stream *mPort;
static uint8_t u8_mDevNr;

typedef enum {
  WAITING_FOR_START_B1,
  WAITING_FOR_START_B2,
  RECEIVING_DATA,
  WAITING_FOR_END_B1,
  WAITING_FOR_END_B2,
  PROCESSING_DATA
} SM_readData;

struct sBpnRecvData {
  //uint8_t  startB1;
  //uint8_t  startB2;
  uint8_t nodeNr;
  uint8_t protocolVersion;
  uint8_t dataLen;
  uint8_t command;
  uint8_t data[0xFF];
  uint16_t crc;
};

//static uint8_t basicMsg[] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };
//static uint8_t cellMsg[]  = { 0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77 };

//
//static float     f_mTotalVoltageOld=0xFFFF;
//static uint16_t  u16_mBalanceCapacityOld=0xFFFF;
//static uint32_t  u32_mChargeMAh=0;
//static uint32_t  u32_mDischargeMAh=0;
//static uint32_t  mqttSendeTimer=0;

//
static void      buildSendMessage(serialDevData_s *devData, uint8_t cmd);
//static void      sendMessage(uint8_t *sendMsg, uint8_t len);
static bool      recvAnswer(sBpnRecvData *p_lRecvBytes);
//static void      parseBasicMessage(uint8_t * t_message);
//static void      parseCellVoltageMessage(uint8_t * t_message);

bool updateBpnFw();
bool bpnFwUpdateRecvAnswer();

//static uint16_t  convertToUint16(int highbyte, int lowbyte);
//static int16_t   convertToInt16(int highbyte, int lowbyte);

//static bool      checkCrc(uint8_t *recvMsg);
//static uint16_t  calcCrc(uint8_t *recvMsg);

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;
static serialDevData_s *mDevData;

bool bpn_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool bo_retFailure=false;
  mDevData=devData;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn=callback;

  if(devData->bo_writeData)
  {
    if(devData->rwDataTyp==BPN_WRITE_READ_SETTINGS || devData->rwDataTyp==BPN_READ_SETTINGS)
    {
      if(devData->rwDataTyp==BPN_WRITE_READ_SETTINGS) buildSendMessage(devData, 0xC);
      else if(devData->rwDataTyp==BPN_READ_SETTINGS) buildSendMessage(devData, 0xD);
      vTaskDelay(pdMS_TO_TICKS(100));

      struct sBpnRecvData bpnRecvData;
      if(!recvAnswer(&bpnRecvData))bo_retFailure=true;
      setSerialBmsReadData(u8_mDevNr, BPN_READ_SETTINGS, bpnRecvData.data, bpnRecvData.dataLen);

      BSC_LOGI(TAG, "Recv Test: cmd=%i, len=%i",bpnRecvData.command, bpnRecvData.dataLen);  
    }
    else if(devData->rwDataTyp==BPN_START_FWUPDATE)
    {
      BSC_LOGI(TAG, "Start FW update");
      buildSendMessage(devData, 0x10);
      vTaskDelay(pdMS_TO_TICKS(100));
      updateBpnFw();
      BSC_LOGI(TAG, "FW update fertig");
    }
  }
  else
  {



    struct sCellData {
      uint32_t   errors;

      uint16_t cellVoltage[18];
      uint16_t cellVoltage_withoutBallanceCells[18];

      int16_t  totalVoltage;
      int16_t  totalCurrent;

      uint16_t highestCellVoltage;
      uint16_t lowestCellVoltage;
      uint16_t maxCellDifferenceVoltage;
      uint16_t avgCellVoltage;

      uint8_t  highestCellNr;
      uint8_t  highestCellNrBallance;
      uint8_t  lowestCellNr;
      uint8_t  lowestCellNrBallance;

      uint8_t  isBalancingActive;
      uint8_t  soc;
      uint8_t  stateFETs;

      uint8_t reserve[1];
    };
    struct sCellData cellData;

    buildSendMessage(devData, 0x20);
    vTaskDelay(pdMS_TO_TICKS(100));

    struct sBpnRecvData bpnRecvData;
    if(recvAnswer(&bpnRecvData))
    {
      memcpy(&cellData,&bpnRecvData.data,sizeof(cellData));
      for(uint8_t i=0;i<18;i++) setBmsCellVoltage(7+u8_mDevNr,i,ROUND(cellData.cellVoltage[i],10));
      setBmsTotalVoltage_int(7+u8_mDevNr,cellData.totalVoltage);
      setBmsTotalCurrent_int(7+u8_mDevNr,cellData.totalCurrent);
      //BSC_LOGI(TAG,"u8_mDevNr=%i, cellVoltage0=%i",u8_mDevNr,cellData.cellVoltage[0]);
    }
    else bo_retFailure=true;

    bo_retFailure=false;
  }






  //BSC_LOGI(TAG,"bpn_readBmsData B");

  //BSC_LOGI(TAG,"serial write=%i, dataTyp=%i, rwData_0=%i,rwData_10=%i",devData->bo_writeData,devData->rwDataTyp,devData->rwData[0],devData->rwData[10]);

  //bool bo_lRet=true;
  /*uint8_t response[JBDBMS_MAX_ANSWER_LEN];

  sendMessage(basicMsg,7);
  if(recvAnswer(response))
  {
    parseBasicMessage(response);

    //mqtt
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr));
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr));
  }
  else bo_lRet=false;
   
  sendMessage(cellMsg,7);
  if(recvAnswer(response)) parseCellVoltageMessage(response);
  else bo_lRet=false;
  
  if(devData->bo_writeData)
  {
    //response als buffer nehmen um zusätzlichen Speicher zu sparen
    buildMessage(response,true,JBDBMS_REG_CELLVOLTAGE_100,WebSettings::getIntFlash(ID_PARAM_JBD_CELL_VOLTAGE_100,u8_mDevNr+BMSDATA_FIRST_DEV_SERIAL,DT_ID_PARAM_JBD_CELL_VOLTAGE_100));
    sendMessage(response,9);
  }

  if(devNr>=2) callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxTxDisable);
  return bo_lRet;  */
  if(bo_retFailure)return false;
  else return true;
}


static void buildSendMessage(serialDevData_s *devData, uint8_t cmd)
{
  uint8_t u8_lCmd=cmd;
  uint8_t frameStart[6];
  uint8_t frameEnd[4];

  //Payload
  //if(devData->bo_writeData==true) //write
  //{
    //BSC_LOGI(TAG,"buildSendMessage write");

    frameStart[0] = 0x55; // Startbyte 1
    frameStart[1] = 0xAA; // Startbyte 2
    frameStart[2] = 0x1;  //
    frameStart[3] = 0x1;  //
    frameStart[4] = devData->rwDataLen;      // Data len
    frameStart[5] = u8_lCmd; // cmd

    frameEnd[0] = 200;  //
    frameEnd[1] = 205;  //
    frameEnd[2] = 0xDE; // Endbyte 1
    frameEnd[3] = 0xF0; // Endbyte 2

    callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
    usleep(20);
    mPort->write(frameStart, 6);
    mPort->write(devData->rwData, devData->rwDataLen);
    mPort->write(frameEnd, 4);
    mPort->flush();  
    callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn);
    
  /*}
  else //read
  {

  }*/
}


static bool recvAnswer(sBpnRecvData *p_lRecvBytes)
{
  //uint8_t u8_lRecvDataLen, ;
  uint8_t u8_lRecvByte, u8_CyclesWithoutData, u8_lRecvBytesCnt;
  uint32_t u32_lStartTime=millis();
  uint8_t currentRxState=WAITING_FOR_START_B1;
  uint8_t *p1;

  u8_lRecvBytesCnt=0;
  //u8_lRecvDataLen=0xFF;
  u8_CyclesWithoutData=0;

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>200) 
    {
      BSC_LOGI(TAG,"Timeout: Serial=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvBytesCnt);
      /*for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
      {
        BSC_LOGD(TAG,"Byte=%i: %i",x, String(p_lRecvBytes[x]));
      }*/
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch(currentRxState)
      {
        case WAITING_FOR_START_B1:
          u8_lRecvBytesCnt=0;
          if(u8_lRecvByte==0x55) currentRxState=WAITING_FOR_START_B2;
          break;

        case WAITING_FOR_START_B2:
          if(u8_lRecvByte==0xAA) currentRxState=RECEIVING_DATA;
          else currentRxState=WAITING_FOR_START_B1;
          break;

        case RECEIVING_DATA:
          p1 = (uint8_t *)p_lRecvBytes+u8_lRecvBytesCnt;
          *p1 = u8_lRecvByte;

          if(u8_lRecvBytesCnt>=4)
          {
            if(u8_lRecvBytesCnt==p_lRecvBytes->dataLen+5)
            {
              currentRxState=WAITING_FOR_END_B1;
              break;
            }
          }
          u8_lRecvBytesCnt++;
          break;

        case WAITING_FOR_END_B1:
          if(u8_lRecvByte==0xDE) currentRxState=WAITING_FOR_END_B2;
          else currentRxState=WAITING_FOR_START_B1;
          break;

        case WAITING_FOR_END_B2:
          if(u8_lRecvByte==0xF0)
          {
            currentRxState=PROCESSING_DATA;
            return true;
          }
          else currentRxState=WAITING_FOR_START_B1;
          break;

        case PROCESSING_DATA:
          break;


        default:
          //Fehler
          break;
      }
      u8_CyclesWithoutData=0;
    }
    else if (u8_lRecvBytesCnt==0) vTaskDelay(pdMS_TO_TICKS(10)); // Wenn noch keine Daten empfangen wurden, dann setze den Task 10ms aus
    else if (u8_lRecvBytesCnt>0 && u8_CyclesWithoutData>10) vTaskDelay(pdMS_TO_TICKS(10)); // Wenn trotz empfangenen Daten 10ms wieder nichts empfangen wurde, dann setze den Task 10ms aus
    else // Wenn in diesem Zyklus keine Daten Empfangen wurde, dann setze den Task 1ms aus
    {
      u8_CyclesWithoutData++;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    //if(u8_lRecvBytesCnt==4+u8_lRecvDataLen+3) break; //Recv Pakage complete
    //if(u8_lRecvBytesCnt>=JBDBMS_MAX_ANSWER_LEN) return false; //Answer too long!
  }

  //if(p_lRecvBytes[2]!=0x0) return false; //0x0 ok; 0x80 Fehler
  //if(p_lRecvBytes[u8_lRecvBytesCnt-1]!=0x77) return false; //letztes Byte muss 0x77 sein

  //Überprüfe Cheksum
  //if(checkCrc(p_lRecvBytes)==false) return false;

  return false;
}


/*static void parseBasicMessage(uint8_t * t_message)
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
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_BALANCE_STATUS, -1, u16_lBalanceStatus);

    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CHARGED_ENERGY, -1, u32_mChargeMAh);
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_DISCHARGED_ENERGY, -1, u32_mDischargeMAh);

    mqttSendeTimer=millis();
  }

}*/


/*static void parseCellVoltageMessage(uint8_t * t_message) 
{
  uint8_t u8_lNumOfCells = 0;
  uint16_t u16_lZellVoltage = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint16_t u16_lZellDifferenceVoltage = 0;
  uint8_t  u8_lZellNumberMinVoltage = 0;
  uint8_t  u8_lZellNumberMaxVoltage = 0;

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

    if (u16_lZellVoltage > u16_lCellHigh)
    {
      u16_lCellHigh = u16_lZellVoltage;
      u8_lZellNumberMaxVoltage=i;
      
    }
    if (u16_lZellVoltage < u16_lCellLow)
    {
      u16_lCellLow = u16_lZellVoltage;
      u8_lZellNumberMinVoltage=i;
    }

    u16_lZellMinVoltage = u16_lCellLow;
    u16_lZellMaxVoltage = u16_lCellHigh;
    u16_lZellDifferenceVoltage = u16_lCellHigh - u16_lCellLow; 
  }
  
  setBmsMaxCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellHigh);
  setBmsMinCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellLow);
  setBmsMaxVoltageCellNumber(BT_DEVICES_COUNT+u8_mDevNr, u8_lZellNumberMaxVoltage);
  setBmsMinVoltageCellNumber(BT_DEVICES_COUNT+u8_mDevNr, u8_lZellNumberMinVoltage);
  setBmsAvgVoltage(BT_DEVICES_COUNT+u8_mDevNr, (float)(u16_lCellSum/u8_lNumOfCells));
  setBmsMaxCellDifferenceVoltage(BT_DEVICES_COUNT+u8_mDevNr,(float)(u16_lZellDifferenceVoltage));
}*/


/*static uint16_t convertToUint16(int highbyte, int lowbyte)
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
}*/







bool updateBpnFw();
bool bpnFwUpdateRecvAnswer();


#include "FS.h"
#include <FS.h>
#ifdef USE_LittleFS
  #define SPIFFS LittleFS
  #include <LittleFS.h> 
#else
  #include <SPIFFS.h>
#endif 
#include "crc.h"

#define COMMAND_BL_MEM_WRITE     0x57

#define COMMAND_BL_MEM_WRITE_LEN 0xB
#define BL_DATA_LEN              0x80

#define BASE_MEM_ADRESS 0x08008000


bool updateBpnFw()
{
	//FILE* file = fopen("/littlefs/bpnFw.bin", "rb");
	FILE* file = fopen("/spiffs/bpnFw.bin", "rb");		
	if (file)
	{
		uint8_t  dataBuf[BL_DATA_LEN +COMMAND_BL_MEM_WRITE_LEN] = { 0 };
		uint32_t aktMemAddress = BASE_MEM_ADRESS;
		uint32_t bytesSent = 0; //Gesendete Bytes
		uint8_t  lenToRead = BL_DATA_LEN;
    bool     bo_firstDataSend=true;


		while (lenToRead>=BL_DATA_LEN)
		{
			dataBuf[1] = COMMAND_BL_MEM_WRITE;
			dataBuf[2] = (aktMemAddress & 0xFF);
			dataBuf[3] = ((aktMemAddress >> 8) & 0xFF);
			dataBuf[4] = ((aktMemAddress >> 16) & 0xFF);
			dataBuf[5] = ((aktMemAddress >> 24) & 0xFF);

			for (uint8_t i = 0; i < BL_DATA_LEN; i++)
			{
				dataBuf[i + 7] = fgetc(file);
				if (feof(file))
				{
					lenToRead = i;
					break;
				}
			}

			dataBuf[6] = lenToRead;

			/* COMMAND_BL_MEM_WRITE_LEN: 1 byte len, 1 byte command code, 4 byte mem base address,
			 * 1 byte payload len, lenToRead is amount of bytes read from file, 4 byte CRC */
			uint8_t memWriteCmdTotalLen = COMMAND_BL_MEM_WRITE_LEN + lenToRead;
			dataBuf[0] = memWriteCmdTotalLen - 1;

			uint32_t crc32 = calcCrc32(dataBuf, memWriteCmdTotalLen - 4);
			dataBuf[lenToRead + 7] = (crc32 & 0xFF);
			dataBuf[lenToRead + 8] = ((crc32 >> 8) & 0xFF);
			dataBuf[lenToRead + 9] = ((crc32 >> 16) & 0xFF);
			dataBuf[lenToRead + 10] = ((crc32 >> 24) & 0xFF);


			//Debugausgabe
			/*printf("Address:      0x%X\n", aktMemAddress);
			printf("Bytes packet: 0x%X\n", lenToRead);
			printf("Bytes sum:    0x%X / %i\n", bytesSent+lenToRead, bytesSent+lenToRead);
			printf("CRC:          0x%X\n", crc32);
			uint8_t n=0;
			for (uint8_t i = 0; i < lenToRead+COMMAND_BL_MEM_WRITE_LEN; i++)
			{
				n++;
				if (n == 9) {std::cout << "  "; }
				if (n > 16) { n = 1; std::cout << "\n";}
				printf("0x%.2X ", (int)dataBuf[i]);
			}
			std::cout << "\n\n";*/

      //Send data
      callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
      usleep(20);
      mPort->write(dataBuf, memWriteCmdTotalLen);
      mPort->flush();  
      callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn);

      //Nach dem senden der ersten Daten muss auf das löschen der Flashes gewartet werden
      if(bo_firstDataSend)
      { 
        bo_firstDataSend=false;
        vTaskDelay(pdMS_TO_TICKS(2000));
      }

      //wait for answer
      if(!bpnFwUpdateRecvAnswer())
      {
        BSC_LOGI(TAG,"BPN FW update error, wrong answer recv.");
        break;
      }

			aktMemAddress += lenToRead;
			bytesSent += lenToRead;
		}
		fclose(file);
	}
  else
  {
    BSC_LOGI(TAG,"Update BPN FW: Can not open FW-file");
    return false;
  }

  //Update Finish
  uint8_t  dataBuf[] = {0x1, 0x60};
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
  usleep(20);
  mPort->write(&dataBuf[0], 1);
  mPort->flush();  
  vTaskDelay(pdMS_TO_TICKS(100));
  mPort->write(&dataBuf[1], 1);
  mPort->flush();  
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn);

  return true;
}


bool bpnFwUpdateRecvAnswer()
{
  enum SM_FwUpadteRecv {BPN_FWUPDATE_SEARCH_STATUS, BPN_FWUPDATE_SEARCH_LEN, BPN_FWUPDATE_RECV_DATA};
  uint32_t u32_lStartTime=millis();
  uint8_t SMrecvState=BPN_FWUPDATE_SEARCH_STATUS;
  uint8_t u8_lRecvBytes[20];
  uint8_t u8_lRecvByte=0;
  uint8_t u8_lRecvDataLenSoll=0xFF;
  uint8_t u8_lRecvBytesCnt=0;

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>2000) 
    {
      BSC_LOGI(TAG,"Timeout: Serial=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvBytesCnt);
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch (SMrecvState)
      {
        case BPN_FWUPDATE_SEARCH_STATUS:
          if (u8_lRecvByte == 0xA5) SMrecvState=BPN_FWUPDATE_SEARCH_LEN; //OK
          else if (u8_lRecvByte == 0x7F) return false; //FAIL
          else return false;
          break;

        case BPN_FWUPDATE_SEARCH_LEN:
          u8_lRecvDataLenSoll=u8_lRecvByte;
          SMrecvState=BPN_FWUPDATE_RECV_DATA;
          break;

        case BPN_FWUPDATE_RECV_DATA:
          u8_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
          u8_lRecvBytesCnt++;
          break;
      
        default:
          break;
      }
    }

    if(u8_lRecvBytesCnt==u8_lRecvDataLenSoll) break; //Recv Pakage complete
    //if(u8_lRecvBytesCnt>=JBDBMS_MAX_ANSWER_LEN) return false; //Answer too long!
  }

	/* Flash_HAL_OK==0x00
   * FLASH_HAL_ERROR==0x01
   * FLASH_HAL_BUSY==0x02
   * FLASH_HAL_TIMEOUT==0x03
   * Flash_HAL_INV_ADDR==0x04 */
  if(u8_lRecvBytes[0]!=0x0) return false; //0x0 ok, alles andere ist ein Fehler

  return true;
}


