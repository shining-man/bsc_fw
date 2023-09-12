// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/SeplosBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

static const char *TAG = "SEPLOS_BMS";

static Stream *mPort;
static uint8_t u8_mTxEnRS485pin, u8_mCountOfPacks, u8_mDevNr; 

enum SM_readData {SEARCH_START, SEARCH_END};

//
static float    f_mTotalVoltageOld=0xFFFF;
static uint16_t u16_mBalanceCapacityOld=0xFFFF;
static uint32_t u32_mChargeMAh=0;
static uint32_t u32_mDischargeMAh=0;
static uint32_t mqttSendeTimer=0;
static uint16_t u16_mRecvBytesLastMsg=0; //for debug

//
static void      getDataFromBms(uint8_t address, uint8_t function);
static bool      recvAnswer(uint8_t * t_outMessage);
static void      parseMessage(uint8_t * t_message, uint8_t address);
static void      parseMessage_Alarms(uint8_t * t_message, uint8_t address);

uint8_t         convertAsciiHexToByte(char a, char b);
static char     convertByteToAsciiHex(uint8_t v);
void            convertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length);
uint16_t        lCrc(const uint16_t len);
static bool     checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt);
static uint16_t calcCrc(uint8_t *data, const uint16_t i16_lLen);

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;


bool SeplosBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool ret = true;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn=callback;
  u8_mCountOfPacks = devData->u8_NumberOfDevices;
  uint8_t response[SEPLOSBMS_MAX_ANSWER_LEN];

  uint8_t u8_lSeplosAdr=devData->u8_deviceNr;
  uint8_t u8_lSeplosAdrBmsData=devData->u8_BmsDataAdr;
  if(u8_mCountOfPacks>1)
  {
    u8_lSeplosAdr+=1;
  }

  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG,"SeplosBms_readBmsData() devNr=%i, readFromAdr=%i, BmsDataAdr=%i, CountOfPacks=%i",u8_mDevNr,u8_lSeplosAdr,u8_lSeplosAdrBmsData,u8_mCountOfPacks);
  #endif

  getDataFromBms(u8_lSeplosAdr, 0x42);
  if(recvAnswer(response))
  {
    parseMessage(response, u8_lSeplosAdrBmsData);

    //mqtt
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_lSeplosAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(BT_DEVICES_COUNT+u8_lSeplosAdrBmsData));
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_lSeplosAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(BT_DEVICES_COUNT+u8_lSeplosAdrBmsData));
  }
  else
  {
    ret=false; 
  }

  if(ret==true)
  {
    getDataFromBms(u8_lSeplosAdr, 0x44); //Alarms
    if(recvAnswer(response))
    {
      parseMessage_Alarms(response, u8_lSeplosAdrBmsData);
    }
    else
    {
      ret=false;
    }
  }
  
  if(u8_mDevNr>=2) callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxTxDisable);
  vTaskDelay(pdMS_TO_TICKS(25));
  return ret;  
}

static void getDataFromBms(uint8_t address, uint8_t function)
{
  /* Beispieldaten
   * ->: 7E 32 30 30 30 34 36 34 32 45 30 30 32 30 30 46 44 33 37 0D 
   * <-: 7E 32 30 30 30 34 36 30 30 31 30 39 36 30 30 30 31 31 30 30 43 43 30 30 43 43 33 30 43 43 32 30 43 42 46 30 43 43 33 30 43 43 30 30 43 43 30 30 43 43 31 30 43 43 31 30 43 43 30 30 43 43 32 30 43 43 33 30 43 43 37 30 43 43 35 30 43 43 35 30 43 43 36 30 36 30 42 36 46 30 42 37 32 30 42 37 32 30 42 37 31 30 42 39 36 30 42 37 43 46 44 37 46 31 34 36 41 32 38 33 45 30 41 34 45 32 30 30 32 30 33 34 45 32 30 30 30 31 35 30 33 45 38 31 34 36 43 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 44 44 30 31 0D
   */

  uint16_t lenid = lCrc(2);

  const uint16_t frame_len = 7;
  uint8_t u8_lData[9];
  uint8_t u8_lSendData[20];

  u8_lData[0]=0x20;           // VER
  u8_lData[1]=address;        // ADDR
  u8_lData[2]=0x46;           // CID1
  u8_lData[3]=function;       // CID2 (0x42)
  u8_lData[4]=(lenid >> 8);   // LCHKSUM (0xE0)
  u8_lData[5]=(lenid >> 0);   // LENGTH (0x02)
  u8_lData[6]=address;        // VALUE (0x00)

  convertByteToAsciiHex(&u8_lSendData[1], &u8_lData[0], frame_len);

  uint16_t crc = calcCrc(&u8_lSendData[1], frame_len*2);
  #ifdef SEPLOS_DEBUG
  BSC_LOGD(TAG,"crc=%i", crc);
  #endif
  u8_lData[7]=(crc >> 8);  // CHKSUM (0xFD)
  u8_lData[8]=(crc >> 0);  // CHKSUM (0x37)
  convertByteToAsciiHex(&u8_lSendData[15], &u8_lData[7], 2); 

  u8_lSendData[0]=0x7E;   // SOF (0x7E)
  u8_lSendData[19]=0x0D;  // EOF (0x0D)

  #ifdef SEPLOS_DEBUG
  String recvBytes="";
  for(uint8_t x=0;x<9;x++)
  {
    recvBytes+="0x";
    recvBytes+=String(u8_lData[x],16);
    recvBytes+=" ";
  }
  BSC_LOGD(TAG,"sendBytes: %s", recvBytes.c_str());
  //log_print_buf(u8_lSendData, 20);
  #endif


  //TX
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
  usleep(20);
  mPort->write(u8_lSendData, 20);
  mPort->flush();  
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn); 
}


static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvDataLen, u8_CyclesWithoutData;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START;
  u8_lRecvBytesCnt=0;
  u8_lRecvDataLen=0xFF;
  u8_CyclesWithoutData=0;
  bool bo_lDataComplete=false;

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>200) 
    {
      BSC_LOGE(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
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
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch (SMrecvState)  {
        case SEARCH_START:
          if (u8_lRecvByte == 0x7E)
          {
            SMrecvState=SEARCH_END;
          }
          break;

        case SEARCH_END:
          p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
          if(u8_lRecvByte == 0x0D)
          {
            bo_lDataComplete=true;
            break;
          }

          u8_lRecvBytesCnt++;
          break;
      
        default:
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

    if(bo_lDataComplete) break; //Recv Pakage complete   
  }
  u16_mRecvBytesLastMsg=u8_lRecvBytesCnt; //for debug

  #ifdef SEPLOS_DEBUG
  String recvBytes="";
  uint8_t u8_logByteCount=0;
  for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
  {
    u8_logByteCount++;
    recvBytes+="0x";
    recvBytes+=String(p_lRecvBytes[x],16);
    recvBytes+=" ";
    if(u8_logByteCount==20)
    {
      BSC_LOGD(TAG,"RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
      recvBytes="";
      u8_logByteCount=0;
    }
  }
  BSC_LOGD(TAG,"RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
  //log_print_buf(p_lRecvBytes, u8_lRecvBytesCnt);
  #endif

  //Überprüfe Cheksum
  if(!checkCrc(p_lRecvBytes,u8_lRecvBytesCnt)) return false;

  return true;
}


void message2Log(uint8_t * t_message, uint8_t address)
{
  //#ifdef SEPLOS_DEBUG
  String recvBytes="";
  uint8_t u8_logByteCount=0;
  BSC_LOGI(TAG,"Dev=%i, RecvBytes=%i",address, u16_mRecvBytesLastMsg);
  for(uint8_t x=0;x<u16_mRecvBytesLastMsg;x++)
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
  //#endif
}


static void parseMessage(uint8_t * t_message, uint8_t address)
{
  //lambda get16bitFromMsg(i)
	auto get16bitFromMsg = [&](size_t i) -> uint16_t {
		return (uint16_t(convertAsciiHexToByte(t_message[i * 2], t_message[(i * 2) + 1])) << 8) | 
           (uint16_t(convertAsciiHexToByte(t_message[(i * 2)+2], t_message[(i * 2) + 3])) << 0);
	};

  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG, "parseMessage: serialDev=%i",address);
  #endif

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

  uint8_t u8_lMsgoffset=0;

  // 0x20 00 46 00 10 96 00 01 10 0C D7 0C E9 0C F4 0C D6 0C EF0CE50CE10CDC0CE90CF00CE80CEF0CEA0CDA0CDE0CD8060BA60BA00B970BA60BA50BA2FD5C14A0344E0A426803134650004603E8149F0000000000000000
  //
  // Byte   Address Content: Description                      Decoded content               Coeff./Unit
  //   0    0x20             Protocol version      VER        2.0
  //   1    0x00             Device address        ADR
  //   2    0x46             Device type           CID1       Lithium iron phosphate battery BMS
  //   3    0x00             Function code         CID2       0x00: Normal, 0x01 VER error, 0x02 Chksum error, ...
  //   4    0x10             Data length checksum  LCHKSUM
  //   5    0x96             Data length           LENID      150 / 2 = 75
  //   6    0x00             Data flag
  //   7    0x00             Command group (Nr. Batterygroup)
  //   8    0x10             Number of cells                  16  
  //   9      0x0C 0xD7      Cell voltage 1                   3287 * 0.001f = 3.287         V
  //   11     0x0C 0xE9      Cell voltage 2                   3305 * 0.001f = 3.305         V
  //   ...    ...            ...
  //   39     0x0C 0xD8      Cell voltage 16         

	u8_lNumOfCells = convertAsciiHexToByte(t_message[8], t_message[8+1]);  //Number of cells
  #ifdef SEPLOS_DEBUG
  BSC_LOGD(TAG, "Number of cells: %d", u8_lNumOfCells);
  #endif

  for (uint8_t i=0; i<u8_lNumOfCells; i++) 
  {
    u16_lZellVoltage = get16bitFromMsg(9+(i*2));
    setBmsCellVoltage(BT_DEVICES_COUNT+address,i, (float)(u16_lZellVoltage));

    u16_lCellSum+=u16_lZellVoltage;

    if(u16_lZellVoltage>u16_lCellHigh)
    {
      u16_lCellHigh=u16_lZellVoltage;
      u8_lZellNumberMaxVoltage=i;
    }
    if(u16_lZellVoltage<u16_lCellLow)
    {
      u16_lCellLow=u16_lZellVoltage;
      u8_lZellNumberMinVoltage=i;
    }

    u16_lZellMinVoltage=u16_lCellLow;
    u16_lZellMaxVoltage=u16_lCellHigh;
    u16_lZellDifferenceVoltage=u16_lCellHigh-u16_lCellLow; 
  }
  
  setBmsMaxCellVoltage(BT_DEVICES_COUNT+address, u16_lCellHigh);
  setBmsMinCellVoltage(BT_DEVICES_COUNT+address, u16_lCellLow);
  setBmsMaxVoltageCellNumber(BT_DEVICES_COUNT+address, u8_lZellNumberMaxVoltage);
  setBmsMinVoltageCellNumber(BT_DEVICES_COUNT+address, u8_lZellNumberMinVoltage);
  setBmsAvgVoltage(BT_DEVICES_COUNT+address, (float)(u16_lCellSum/u8_lNumOfCells));
  setBmsMaxCellDifferenceVoltage(BT_DEVICES_COUNT+address,(float)(u16_lZellDifferenceVoltage));
  

  u8_lMsgoffset = 9+(u8_lNumOfCells*2);

  //   41     0x06           Number of temperatures           6                                    V
  uint8_t u8_lCntTempSensors = convertAsciiHexToByte(t_message[u8_lMsgoffset*2], t_message[u8_lMsgoffset*2+1]);
  #ifdef SEPLOS_DEBUG
  BSC_LOGD(TAG, "Number of temperature sensors: %d", u8_lCntTempSensors);
  #endif

  //   42     0x0B 0xA6      Temperature sensor 1             (2982 - 2731) * 0.1f = 25.1          °C
  //   44     0x0B 0xA0      Temperature sensor 2             (2976 - 2731) * 0.1f = 24.5          °C
  //   46     0x0B 0x97      Temperature sensor 3             (2967 - 2731) * 0.1f = 23.6          °C
  //   48     0x0B 0xA6      Temperature sensor 4             (2982 - 2731) * 0.1f = 25.1          °C
  //   50     0x0B 0xA5      Environment temperature          (2981 - 2731) * 0.1f = 25.0          °C
  //   52     0x0B 0xA2      Mosfet temperature               (2978 - 2731) * 0.1f = 24.7          °C
  float fl_lBmsTemps[3]; //Hier werden die ketzten drei Temperaturen zwischengespeichert
  for (uint8_t i=0; i<u8_lCntTempSensors; i++)
  {
    fl_lBmsTemps[2] = (float)(get16bitFromMsg(u8_lMsgoffset+1+(i*2))-0xAAB)*0.1;
    if(i<3) setBmsTempature(BT_DEVICES_COUNT+address,i,fl_lBmsTemps[2]);
    else if(i>=3 && i<5)fl_lBmsTemps[i-3]=fl_lBmsTemps[2];
  }

  u8_lMsgoffset=u8_lMsgoffset+1+(u8_lCntTempSensors*2);

  //   54     0xFD 0x5C      Charge/discharge current         signed int?                   A
  float f_lTotalCurrent = (float)((int16_t)get16bitFromMsg(u8_lMsgoffset))*0.01f;
  setBmsTotalCurrent(BT_DEVICES_COUNT+address,f_lTotalCurrent);

  //   56     0x14 0xA0      Total battery voltage            5280 * 0.01f = 52.80          V
  float f_lTotalVoltage = (float)get16bitFromMsg(u8_lMsgoffset+2)*0.01f;
  setBmsTotalVoltage(BT_DEVICES_COUNT+address, f_lTotalVoltage);

  //   58     0x34 0x4E      Restkapazität                   13390 * 0.01f = 133.90         Ah
  uint16_t u16_lBalanceCapacity=get16bitFromMsg(u8_lMsgoffset+4)/100;

  //   60     0x0A           Custom number                    10

  //   61     0x42 0x68      Battery capacity                 17000 * 0.01f = 170.00        Ah
  uint16_t u16_lFullCapacity=get16bitFromMsg(u8_lMsgoffset+7)/100;

  //   63     0x03 0x13      Stage of charge                  787 * 0.1f = 78.7             %
  #ifdef SEPLOS_DEBUG
  uint8_t u8_lSoc = get16bitFromMsg(u8_lMsgoffset+9)/10;
  uint8_t u8_lSocOld = getBmsChargePercentage(BT_DEVICES_COUNT+address);
  uint8_t u8_lSocOld5Percent = u8_lSocOld/100*5;
  if(u8_lSoc<u8_lSocOld-u8_lSocOld5Percent || u8_lSoc>u8_lSocOld+u8_lSocOld5Percent)
  {
    digitalWrite(GPIO_LED1_HW1, !digitalRead(GPIO_LED1_HW1));
    BSC_LOGI(TAG, "SoC Abweichung > 5%; SoC_alt=%i, SoC_neu=%i",u8_lSocOld,u8_lSoc);
    message2Log(t_message, address);
  }
  #endif
  setBmsChargePercentage(BT_DEVICES_COUNT+address, get16bitFromMsg(u8_lMsgoffset+9)/10);

  //   65     0x46 0x50      Rated capacity                   18000 * 0.01f = 180.00        Ah
  //(float) get16bitFromMsg(u8_lMsgoffset + 11) * 0.01f);

  //   67     0x00 0x46      Number of cycles                 70
  uint16_t u16_lCycle=get16bitFromMsg(u8_lMsgoffset + 13);

  //   69     0x03 0xE8      State of health                  1000 * 0.1f = 100.0           %
  //(float) get16bitFromMsg(u8_lMsgoffset + 15) * 0.1f);

  //   71     0x14 0x9F      Port voltage                     5279 * 0.01f = 52.79          V
  //(float) get16bitFromMsg(u8_lMsgoffset + 17) * 0.01f);

  //   73     0x00 0x00      Reserved
  //   75     0x00 0x00      Reserved
  //   77     0x00 0x00      Reserved
  //   79     0x00 0x00      Reserved


  if((millis()-mqttSendeTimer)>10000)
  {
    //Nachrichten senden
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_TEMPERATURE, 3, fl_lBmsTemps[0]);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_TEMPERATURE, 4, fl_lBmsTemps[1]);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_TEMPERATURE, 5, fl_lBmsTemps[2]);

    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);

    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_BALANCE_STATUS, -1, u16_lBalanceStatus);
    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_FET_STATUS, -1, u16_lFetStatus);

    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_CHARGED_ENERGY, -1, u32_mChargeMAh);
    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+address, MQTT_TOPIC2_DISCHARGED_ENERGY, -1, u32_mDischargeMAh);

    mqttSendeTimer=millis();
  }
}


static void parseMessage_Alarms(uint8_t * t_message, uint8_t address)
{
  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG, "parseMessage: serialDev=%i",address);
  #endif

  // Byte   Address Content: Description                      Decoded content               Coeff./Unit
  //   0    0x20             Protocol version      VER        2.0
  //   1    0x00             Device address        ADR
  //   2    0x46             Device type           CID1       Lithium iron phosphate battery BMS
  //   3    0x00             Function code         CID2       0x00: Normal, 0x01 VER error, 0x02 Chksum error, ...
  //   4    0x10             Data length checksum  LCHKSUM
  //   5    0x96             Data length           LENID      150 / 2 = 75
  //   6    0x00             Data flag
  //   7    0x01             Command group

  // Byte   Description
  // The following are 24 byte alarms 
  //   8    Number of cells M=16
  //   9    Cell 1 alarm
  //  10    Cell 2 alarm
  //  11    Cell 3 alarm
  //  12    Cell 4 alarm
  //  13    Cell 5 alarm
  //  14    Cell 6 alarm
  //  15    Cell 7 alarm
  //  16    Cell 8 alarm
  //  17    Cell 9 alarm
  //  18    Cell 10 alarm
  //  19    Cell 11 alarm
  //  20    Cell 12 alarm
  //  21    Cell 13 alarm
  //  22    Cell 14 alarm
  //  23    Cell 15 alarm
  //  24    Cell 16 alarm
  //  25    Number of temperatures N=6
  //  26    Cell temperature alarm 1 
  //  27    Cell temperature alarm 2
  //  28    Cell temperature alarm 3
  //  29    Cell temperature alarm 4
  //  30    Environment temperature alarm 
  //  31    Power temperature alarm 1 
  //  32    Charge/discharge current alarm
  //  33    Total battery voltage alarm
  // The following are 20 bit alarms (Vmtl. sind nicht bit sondern byte gemeint) 
  //  34    Number of custom alarms P=20
  //  35    Alarm event 1
  //  36    Alarm event 2
  //  37    Alarm event 3
  //  38    Alarm event 4
  //  39    Alarm event 5
  //  40    Alarm event 6
  //  41    On-off state 
  //  42    Equilibrium state 1 
  //  43    Equilibrium state 2
  //  44    System state
  //  45    Disconnection state 1
  //  46    Disconnection state 2
  //  47    Alarm event 7
  //  48    Alarm event 8
  //  49    Reservation extension 
  //  50    Reservation extension 
  //  51    Reservation extension 
  //  52    Reservation extension 
  //  53    Reservation extension 
  //  54    Reservation extension  
 

  //  Comments on byte alarms 
  //  S/N  Value  Meaning
  //  1    0x00   Normal, no alarm
  //  2    0x01   Alarm that analog quantity reaches the lower limit
  //  3    0x02   Alarm that analog quantity reaches the upper limit
  //  4    0xF0   Other alarms 
  //  
  //  Alarm event 1 - Flag bit information (1: trigger, 0: normal)
  //  0 Voltage sensor fault
  //  1 Temperature sensor fault
  //  2 Current sensor fault
  //  3 Key switch fault
  //  4 Cell voltage dropout fault
  //  5 Charge switch fault
  //  6 Discharge switch fault
  //  7 Current limit switch fault
  //  
  //  Alarm event 2 - Flag bit information (1: trigger, 0: normal)
  //  0 Monomer high voltage alarm
  //  1 Monomer overvoltage protection
  //  2 Monomer low voltage alarm
  //  3 Monomer under voltage protection
  //  4 High voltage alarm for total voltage
  //  5 Overvoltage protection for total voltage
  //  6 Low voltage alarm for total voltage
  //  7 Under voltage protection for total voltage
  //  
  //  Alarm event 3 - Flag bit information (1: trigger, 0: normal)
  //  0 Charge high temperature alarm (Cell temperature)
  //  1 Charge over temperature protection (Cell temperature)
  //  2 Charge low temperature alarm (Cell temperature)
  //  3 Charge under temperature protection (Cell temperature)
  //  4 Discharge high temperature alarm (Cell temperature)
  //  5 Discharge over temperature protection (Cell temperature)
  //  6 Discharge low temperature alarm (Cell temperature)
  //  7 Discharge under temperature protection (Cell temperature)
  //  
  //  Alarm event 4 - Flag bit information (1: trigger, 0: normal)
  //  0 Environment high temperature alarm Environment
  //  1 Environment over temperature protection temperature
  //  2 Environment low temperature alarm
  //  3 Environment under temperature protection
  //  4 Power over temperature protection Power
  //  5 Power high temperature alarm temperature
  //  6 Cell low temperature heating Cell temperature
  //  7 Reservation bit
  //  
  //  Alarm event 5 - Flag bit information (1: trigger, 0: normal)
  //  0 Charge over current alarm
  //  1 Charge over current protection
  //  2 Discharge over current alarm
  //  3 Discharge over current protection
  //  4 Transient over current protection
  //  5 Output short circuit protection
  //  6 Transient over current lockout
  //  7 Output short circuit lockout
  //  
  //  Alarm event 6 - Flag bit information (1: trigger, 0: normal)
  //  0 Charge high voltage protection
  //  1 Intermittent recharge waiting
  //  2 Residual capacity alarm
  //  3 Residual capacity protection
  //  4 Cell low voltage charging prohibition
  //  5 Output reverse polarity protection
  //  6 Output connection fault
  //  7 Inside bit
  //  
  //  On-off state - Flag bit information (1: on, 0: off)
  //  0 Discharge switch state
  //  1 Charge switch state
  //  2 Current limit switch state
  //  3 Heating switch state
  //  4-7 Reservation bit
  //  
  //  Equilibrium state 1 - Flag bit information (1: on, 0: off)
  //  0 Cell 01 equilibrium
  //  1 Cell 02 equilibrium
  //  2 Cell 03 equilibrium
  //  3 Cell 04 equilibrium
  //  4 Cell 05 equilibrium
  //  5 Cell 06 equilibrium
  //  6 Cell 07 equilibrium
  //  7 Cell 08 equilibrium
  //  
  //  Equilibrium state 2 - Flag bit information (1: on, 0: off)
  //  0 Cell 09 equilibrium
  //  1 Cell 10 equilibrium
  //  2 Cell 11 equilibrium
  //  3 Cell 12 equilibrium
  //  4 Cell 13 equilibrium
  //  5 Cell 14 equilibrium
  //  6 Cell 15 equilibrium
  //  7 Cell 16 equilibrium
  //  
  //  System state Flag bit information (1: access, 0: exit)
  //  0 Discharge
  //  1 Charge
  //  2 Floating charge
  //  3 Reservation bit
  //  4 Standby
  //  5 Shutdown
  //  6 Reservation bit
  //  7 Reservation bit
  //  
  //  Disconnection state 1 - Flag bit information (1: trigger, 0: normal)
  //  0 Cell 01 disconnection
  //  1 Cell 02 disconnection
  //  2 Cell 03 disconnection
  //  3 Cell 04 disconnection
  //  4 Cell 05 disconnection
  //  5 Cell 06 disconnection
  //  6 Cell 07 disconnection
  //  7 Cell 08 disconnection
  //  
  //  Disconnection state 2 - Flag bit information (1: trigger, 0: normal)
  //  0 Cell 09 disconnection
  //  1 Cell 10 disconnection
  //  2 Cell 11 disconnection
  //  3 Cell 12 disconnection
  //  4 Cell 13 disconnection
  //  5 Cell 14 disconnection
  //  6 Cell 15 disconnection
  //  7 Cell 16 disconnection
  //  
  //  Alarm event 7 - Flag bit information (1: trigger, 0: normal)
  //  0 Inside bit
  //  1 Inside bit
  //  2 Inside bit
  //  3 Inside bit
  //  4 Automatic charging waiting
  //  5 Manual charging waiting
  //  6 Inside bit
  //  7 Inside bit
  //  
  //  Alarm event 8 - Flag bit information (1: trigger, 0: normal)
  //  0 EEP storage fault
  //  1 RTC error
  //  2 Voltage calibration not performed
  //  3 Current calibration not performed
  //  4 Zero calibration not performed
  //  5 Inside bit
  //  6 Inside bit
  //  7 Inside bit


  // Beispieldaten:
  // ->: 7E 32 30 30 30 34 36 34 34 45 30 30 32 30 30 46 44 33 35 0D
  // <-: 7E 32 30 30 30 34 36 30 30 38 30 36 32 30 30 30 31 31 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 36 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 31 34 30 30 30 30 30 30 30 30 30 30 30 30 30 33 30 30 30 30 30 31 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 31 45 42 33 32 0D Response Ok


  uint8_t  u8_dataLen = convertAsciiHexToByte(t_message[10], t_message[11]); //5
  uint32_t u32_alarm = 0;
  boolean  bo_lValue=false;

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x //bit0  single cell overvoltage protection
  #define BMS_ERR_STATUS_CELL_UVP          2  x //bit1  single cell undervoltage protection
  #define BMS_ERR_STATUS_BATTERY_OVP       4  x //bit2  whole pack overvoltage protection
  #define BMS_ERR_STATUS_BATTERY_UVP       8  x //bit3  Whole pack undervoltage protection
  #define BMS_ERR_STATUS_CHG_OTP          16  x //bit4  charging over temperature protection
  #define BMS_ERR_STATUS_CHG_UTP          32  x //bit5  charging low temperature protection
  #define BMS_ERR_STATUS_DSG_OTP          64  x //bit6  Discharge over temperature protection
  #define BMS_ERR_STATUS_DSG_UTP         128  x //bit7  discharge low temperature protection
  #define BMS_ERR_STATUS_CHG_OCP         256  x //bit8  charging overcurrent protection
  #define BMS_ERR_STATUS_DSG_OCP         512  x //bit9  Discharge overcurrent protection
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  - //bit10 short circuit protection
  #define BMS_ERR_STATUS_AFE_ERROR      2048  x //bit11 Front-end detection IC error
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  - //bit12 software lock MOS
  #define BMS_ERR_STATUS_RESERVED1      8192  - //bit13 Reserved
  #define BMS_ERR_STATUS_RESERVED2     16384  - //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  - //bit15 Reserved */

  for (uint8_t i = 70; i < u8_dataLen+14; i+=2) //14=offset
  {
    uint8_t u8_lByte = convertAsciiHexToByte(t_message[i], t_message[i+1]);

    switch (i)
    {
      //  35    Alarm event 1
      case 70: //35*2
        if (u8_lByte > 0) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR; //?
        break;

      //  36    Alarm event 2
      case 72:
        if ((u8_lByte & 0x1) == 0x1) u32_alarm |= BMS_ERR_STATUS_CELL_OVP; //?
        if ((u8_lByte & 0x2) == 0x2) u32_alarm |= BMS_ERR_STATUS_CELL_OVP; //?
        if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_CELL_UVP; //?
        if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_CELL_UVP; //?
        if ((u8_lByte & 0x10) == 0x10) u32_alarm |= BMS_ERR_STATUS_BATTERY_OVP;
        if ((u8_lByte & 0x20) == 0x20) u32_alarm |= BMS_ERR_STATUS_BATTERY_OVP; //?
        if ((u8_lByte & 0x40) == 0x40) u32_alarm |= BMS_ERR_STATUS_BATTERY_UVP;
        if ((u8_lByte & 0x80) == 0x80) u32_alarm |= BMS_ERR_STATUS_BATTERY_UVP; //?
        break;

      //  37    Alarm event 3
      case 74:
        if ((u8_lByte & 0x1) == 0x1) u32_alarm |= BMS_ERR_STATUS_CHG_OTP;
        if ((u8_lByte & 0x2) == 0x2) u32_alarm |= BMS_ERR_STATUS_CHG_OTP; //?
        if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_CHG_UTP;
        if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_CHG_UTP; //?
        if ((u8_lByte & 0x10) == 0x10) u32_alarm |= BMS_ERR_STATUS_DSG_OTP;
        if ((u8_lByte & 0x20) == 0x20) u32_alarm |= BMS_ERR_STATUS_DSG_OTP; //?
        if ((u8_lByte & 0x40) == 0x40) u32_alarm |= BMS_ERR_STATUS_DSG_UTP;
        if ((u8_lByte & 0x80) == 0x80) u32_alarm |= BMS_ERR_STATUS_DSG_UTP; //?
        break;

      //  38    Alarm event 4
      case 76:
        if (u8_lByte > 0) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR; //?
        break;

      //  39    Alarm event 5
      case 78:
        if ((u8_lByte & 0x1) == 0x1) u32_alarm |= BMS_ERR_STATUS_CHG_OCP;
        if ((u8_lByte & 0x2) == 0x2) u32_alarm |= BMS_ERR_STATUS_CHG_OCP; //?
        if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_DSG_OCP;
        if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_DSG_OCP; //?
        if ((u8_lByte & 0x10) == 0x10) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR; //?
        if ((u8_lByte & 0x20) == 0x20) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR; //?
        if ((u8_lByte & 0x40) == 0x40) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR; //?
        if ((u8_lByte & 0x80) == 0x80) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR; //?
        break;

      //  40    Alarm event 6
      case 80:
        if (u8_lByte > 0) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR; //?
        break;

      //  41    On-off state - Flag bit information (1: on, 0: off)
      case 82: 
        // 0 Discharge switch state
        bo_lValue=false;
        if ((u8_lByte & 0x1) == 0x1) bo_lValue=true;
        setBmsStateFETsDischarge(BT_DEVICES_COUNT+address,bo_lValue);

        // 1 Charge switch state
        bo_lValue=false;
        if ((u8_lByte & 0x2) == 0x2) bo_lValue=true;
        setBmsStateFETsCharge(BT_DEVICES_COUNT+address,bo_lValue);
        break;
    }
  }

  setBmsErrors(BT_DEVICES_COUNT+address, u32_alarm);
}


uint8_t convertAsciiHexToByte(char a, char b)
{
  a = (a<='9') ? a-'0' : (a&0x7)+9;
  b = (b<='9') ? b-'0' : (b&0x7)+9;
  return (a<<4)+b;
}


static char convertByteToAsciiHex(uint8_t v)
{
  return v>=10 ? 'A'+(v-10) : '0'+v;
}


void convertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length)
{
  if(length==0) return;

  for(size_t i=0; i<length; i++)
  {
    dest[2*i] = convertByteToAsciiHex((data[i] & 0xF0) >> 4);
    dest[2*i+1] = convertByteToAsciiHex(data[i] & 0x0F);
  }
}


uint16_t lCrc(const uint16_t len)
{
  uint16_t u16_lLcrc = 0x0000;

  if (len == 0) return 0x0000;

  u16_lLcrc = (len&0xf) + ((len>>4)&0xf) + ((len>>8)&0xf);
  u16_lLcrc = ~(u16_lLcrc % 16) + 1;

  return (u16_lLcrc<<12) + len;  // 4 byte checksum + 12 bytes length
}


static bool checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt)
{
  uint16_t u16_lCrc = calcCrc(recvMsg, u8_lRecvBytesCnt-4);
 	uint16_t u16_lRemoteCrc = (uint16_t)convertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt-2], recvMsg[u8_lRecvBytesCnt-1]) |	
    (uint16_t)(convertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt-4], recvMsg[u8_lRecvBytesCnt-3]))<<8;

  if (u16_lCrc != u16_lRemoteCrc)
  {
    BSC_LOGE(TAG, "CRC failed: %04X != %04X", u16_lCrc, u16_lRemoteCrc);
    return false;
  }

  return true;
}


static uint16_t calcCrc(uint8_t *data, const uint16_t i16_lLen)
{
  uint16_t u16_lSum = 0x00;
  for (uint16_t i=0; i<i16_lLen; i++)
  {
    u16_lSum=u16_lSum+data[i];
  }
  u16_lSum=~u16_lSum;
  u16_lSum++;
  return u16_lSum;
}





