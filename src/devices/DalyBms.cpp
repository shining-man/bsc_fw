// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/DalyBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

static const char *TAG = "DALY_BMS";

#define DALAY_SEND_DELAY 20

static Stream *mPort;
static uint8_t u8_mDevNr;

enum SM_readData {SEARCH_START, SEARCH_END};

//
static uint8_t u8_mNumberOfCells=0;
static uint8_t u8_mNumOfTempSensors=0;

static float    f_mTotalVoltageOld=0xFFFF;
//static uint32_t mqttSendeTimer=0;

//
static void      getDataFromBms(BscSerial *bscSerial, uint8_t address, uint8_t function);
static bool      recvAnswer(uint8_t * t_outMessage, uint8_t packets);
static void      parseMessage(uint8_t * t_message, uint8_t dataMappingNr);


static serialDevData_s *mDevData;

bool DalyBms_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData)
{
  bool bo_ret=true;
  mDevData=devData;
  mPort = port;
  u8_mDevNr = devNr;
    uint8_t response[DALY_FRAME_SIZE*16];

  uint8_t dataMappingNr = devData->dataMappingNr;

  #ifdef DALY_DEBUG
  BSC_LOGI(TAG,"DalyBms_readBmsData()");
  #endif

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_BATTERY_SOC);
  if(recvAnswer(response, 1))
  {
    parseMessage(response, dataMappingNr);

    mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(dataMappingNr));
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(dataMappingNr));
  }
  else bo_ret=false;
  vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_MIN_MAX_VOLTAGE);
  if(recvAnswer(response,1)) parseMessage(response, dataMappingNr);
  else bo_ret=false;
  vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  //getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_MIN_MAX_TEMPERATURE); //not use
  //if(recvAnswer(response,1)) parseMessage(response);
  //else bo_ret=false;
  //vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_MOS);
  if(recvAnswer(response,1)) parseMessage(response, dataMappingNr);
  else bo_ret=false;
  vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_STATUS); //Cellnumbers
  if(recvAnswer(response,1)) parseMessage(response, dataMappingNr);
  else bo_ret=false;
  vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_CELL_VOLTAGE);
  if(recvAnswer(response,u8_mNumberOfCells/3+u8_mNumberOfCells%3)) parseMessage(response, dataMappingNr);
  else bo_ret=false;
  vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_TEMPERATURE);
  if(recvAnswer(response,1)) parseMessage(response, dataMappingNr);
  else bo_ret=false;
  vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_BALLANCER);
  if(recvAnswer(response,1)) parseMessage(response, dataMappingNr);
  else bo_ret=false;
  vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  getDataFromBms(bscSerial, DALAY_BMS_ADRESS, DALY_REQUEST_FAILURE);
  if(recvAnswer(response,1)) parseMessage(response, dataMappingNr);
  else bo_ret=false;
  //vTaskDelay(pdMS_TO_TICKS(DALAY_SEND_DELAY));

  return bo_ret;
}

static void getDataFromBms(BscSerial *bscSerial, uint8_t address, uint8_t function)
{
  uint8_t u8_lData[DALY_FRAME_SIZE];
  u8_lData[0] = DALAY_START_BYTE;   // Start
  u8_lData[1] = address;            // ID
  u8_lData[2] = function;           // Data
  u8_lData[3] = 0x08;               // Data Length (fix)
  u8_lData[4] = 0x00;               // Empty Data
  u8_lData[5] = 0x00;
  u8_lData[6] = 0x00;
  u8_lData[7] = 0x00;
  u8_lData[8] = 0x00;
  u8_lData[9] = 0x00;
  u8_lData[10] = 0x00;
  u8_lData[11] = 0x00;
  u8_lData[12] = (uint8_t) (u8_lData[0] + u8_lData[1] + u8_lData[2] + u8_lData[3]);  // Checksum (Lower byte of the other bytes sum)

  #ifdef DALY_DEBUG
  String recvBytes="";
  for(uint8_t x=0;x<DALY_FRAME_SIZE;x++)
  {
    recvBytes+=String(u8_lData[x],16);
    recvBytes+=" ";
  }
  BSC_LOGD(TAG,"sendBytes: %s", recvBytes.c_str());
  #endif

  //Empfangsbuffer leeren wenn da noch etwas drin sein sollte
  for(uint8_t i=0;i<200;i++)
  {
    if (mPort->available() == 0) break;
  }

  //TX
  bscSerial->sendSerialData(mPort, u8_mDevNr, u8_lData, DALY_FRAME_SIZE);
}


static bool recvAnswer(uint8_t *p_lRecvBytes, uint8_t packets)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvBytesCntPacket, u8_checlSum, u8_CyclesWithoutData;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START;
  u8_lRecvBytesCnt=0;
  u8_lRecvBytesCntPacket=0;
  u8_checlSum=0;
  u8_CyclesWithoutData=0;

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>200)
    {
      BSC_LOGE(TAG,"Timeout: Serial=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvBytesCnt);
      #ifdef DALY_DEBUG
      String recvBytes="";
      for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
      {
        recvBytes+=String(p_lRecvBytes[x],16);
        recvBytes+=" ";
      }
      BSC_LOGD(TAG,"Timeout: RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
      #endif
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if(mPort->available() > 0)
    {
      u32_lStartTime = millis(); // Timer zurücksetzen

      u8_lRecvByte = mPort->read();

      switch (SMrecvState)
      {
        case SEARCH_START:
          if (u8_lRecvByte == DALAY_START_BYTE)
          {
            u32_lStartTime=millis();
            p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
            u8_lRecvBytesCnt++;
            u8_checlSum=u8_lRecvByte;
            u8_lRecvBytesCntPacket=1;
            SMrecvState=SEARCH_END;
          }
          break;

        case SEARCH_END:
          p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
          u8_lRecvBytesCnt++;
          u8_lRecvBytesCntPacket++;

          if(u8_lRecvBytesCntPacket==DALY_FRAME_SIZE)
          {
            //BSC_LOGI(TAG,"Last byte; cnt=%i, cntPacket=%i, recvByte=%i, chkSum=%i",u8_lRecvBytesCnt, u8_lRecvBytesCntPacket, u8_lRecvByte, u8_checlSum);
            SMrecvState=SEARCH_START;

            //Überprüfe Cheksum
            if(u8_checlSum!=p_lRecvBytes[u8_lRecvBytesCnt-1]) return false;
          }
          else u8_checlSum+=u8_lRecvByte;
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

    if(u8_lRecvBytesCnt==(DALY_FRAME_SIZE*packets)) break; //Recv Pakage complete
    if(u8_lRecvBytesCnt>=(DALY_FRAME_SIZE*16)) return false; //Answer too long!
  }

  /*#ifdef DALY_DEBUG
  String recvBytes="";
  uint8_t logBytCnt=0;
  for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
  {
    recvBytes+=String(p_lRecvBytes[x]);
    recvBytes+=" ";
    logBytCnt++;
    if(logBytCnt>=13)
    {
      BSC_LOGI(TAG,"RX: %s", recvBytes.c_str());
      recvBytes="";
      logBytCnt=0;
    }
  }
  BSC_LOGI(TAG,"RX: %s", recvBytes.c_str());
  #endif*/

  return true;
}


static void parseMessage(uint8_t * t_message, uint8_t dataMappingNr)
{
  bool     bo_value=false;
  uint8_t  u8_lValue = 0;
  uint16_t u16_lValue1, u16_lValue2;
  uint32_t u32_lValue;

  switch (t_message[2])
  {
    case DALY_REQUEST_BATTERY_SOC:
      setBmsTotalVoltage(dataMappingNr, ((float)((t_message[4]<<8) | t_message[5]) / 10.0f));
      setBmsTotalCurrent(dataMappingNr, ((float)(((t_message[8]<<8) | t_message[9]) - DALY_CURRENT_OFFSET) / 10.0f));
      setBmsChargePercentage(dataMappingNr, ROUND( ((t_message[10]<<8) | t_message[11]), 10 ) );
      break;

    case DALY_REQUEST_MIN_MAX_VOLTAGE:
      u16_lValue1 = ((t_message[4]<<8) | t_message[5]);
      u16_lValue2 = ((t_message[7]<<8) | t_message[8]);
      setBmsMaxCellVoltage(dataMappingNr, u16_lValue1);
      setBmsMaxVoltageCellNumber(dataMappingNr, t_message[6]);
      setBmsMinCellVoltage(dataMappingNr, u16_lValue2);
      setBmsMinVoltageCellNumber(dataMappingNr, t_message[9]);
      setBmsMaxCellDifferenceVoltage(dataMappingNr, u16_lValue1 - u16_lValue2);
      break;

    case DALY_REQUEST_MIN_MAX_TEMPERATURE:
      // (t_message[4] - DALY_TEMPERATURE_OFFSET); // temp max
      // (t_message[6] - DALY_TEMPERATURE_OFFSET); // temp min
      // (get.tempMax + get.tempMin) / 2;          // temp average
      break;

    case DALY_REQUEST_MOS:
      //Charge FET
      if(t_message[5]==1) setBmsStateFETsCharge(dataMappingNr,true);
      else setBmsStateFETsCharge(dataMappingNr,false);

      //Discharge FET
      if(t_message[6]==1) setBmsStateFETsDischarge(dataMappingNr,true);
      else setBmsStateFETsDischarge(dataMappingNr,false);

      //Remain capacity (mAH)
      //ToDo: only MQTT
      // ((uint32_t)t_message[8]<<0x18) | ((uint32_t)t_message[9]<<0x10) | ((uint32_t)t_message[10]<<0x08) | (uint32_t)t_message[11];
      break;

    case DALY_REQUEST_STATUS:
      u8_mNumberOfCells = t_message[4];
      u8_mNumOfTempSensors = t_message[5];
      #ifdef DALY_DEBUG
      BSC_LOGI(TAG,"NumberOfCells=%i, NumOfTempSensor=%i",u8_mNumberOfCells,u8_mNumOfTempSensors);
      #endif
      break;

    case DALY_REQUEST_CELL_VOLTAGE:
      u8_lValue=0;
      u16_lValue1=0;
      u32_lValue=0;
      for (size_t n = 0; n <= (u8_mNumberOfCells/3+u8_mNumberOfCells%3); n++)
      {
        for (size_t i = 0; i < 3; i++)
        {
          u16_lValue1 = (t_message[(n*DALY_FRAME_SIZE)+5+(i*2)]<<8) | t_message[(n*DALY_FRAME_SIZE)+6+(i*2)];
          u32_lValue+=u16_lValue1;
          setBmsCellVoltage(dataMappingNr, u8_lValue, u16_lValue1);
          u8_lValue++;
          if (u8_lValue == u8_mNumberOfCells)
          {
            setBmsAvgVoltage(dataMappingNr, u32_lValue/u8_mNumberOfCells);
            return;
          }
        }
      }
      break;

    case DALY_REQUEST_TEMPERATURE:
      u8_lValue = 0;
      for (uint8_t i=0; i<7; i++)
      {
        setBmsTempature(dataMappingNr, u8_lValue, (t_message[5+i]-40));
        u8_lValue++;
        if (u8_lValue == u8_mNumOfTempSensors) break;
      }
      break;

    case DALY_REQUEST_BALLANCER:
      for (uint8_t i=0; i<6; i++)
      {
        for (uint8_t n=0; n<8; n++)
        {
          if (bitRead(t_message[i+4], n))
          {
            bo_value=true;
            break;
          }
        }
      }
      setBmsIsBalancingActive(dataMappingNr, bo_value);
      break;


    case DALY_REQUEST_FAILURE:
      /*bmsErrors
      #define BMS_ERR_STATUS_OK                0
      #define BMS_ERR_STATUS_CELL_OVP          1   //bit0  single cell overvoltage protection
      #define BMS_ERR_STATUS_CELL_UVP          2   //bit1  single cell undervoltage protection
      #define BMS_ERR_STATUS_BATTERY_OVP       4   //bit2  whole pack overvoltage protection
      #define BMS_ERR_STATUS_BATTERY_UVP       8   //bit3  Whole pack undervoltage protection
      #define BMS_ERR_STATUS_CHG_OTP          16   //bit4  charging over temperature protection
      #define BMS_ERR_STATUS_CHG_UTP          32   //bit5  charging low temperature protection
      #define BMS_ERR_STATUS_DSG_OTP          64   //bit6  Discharge over temperature protection
      #define BMS_ERR_STATUS_DSG_UTP         128   //bit7  discharge low temperature protection
      #define BMS_ERR_STATUS_CHG_OCP         256   //bit8  charging overcurrent protection
      #define BMS_ERR_STATUS_DSG_OCP         512   //bit9  Discharge overcurrent protection
      #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024   //bit10 short circuit protection
      #define BMS_ERR_STATUS_AFE_ERROR      2048   //bit11 Front-end detection IC error
      #define BMS_ERR_STATUS_SOFT_LOCK      4096   //bit12 software lock MOS */

      u16_lValue1 = t_message[4];
      if(u16_lValue1&0x1) u16_lValue2|=1;         //Bit 0: Cell volt high level 1
      if((u16_lValue1>>1)&0x1) u16_lValue2|=1;    //Bit 1: Cell volt high level 2
      if((u16_lValue1>>2)&0x1) u16_lValue2|=2;    //Bit 2: Cell volt low level 1
      if((u16_lValue1>>3)&0x1) u16_lValue2|=2;    //Bit 3: Cell volt low level 2
      if((u16_lValue1>>4)&0x1) u16_lValue2|=4;    //Bit 4: Sum volt high level 1
      if((u16_lValue1>>5)&0x1) u16_lValue2|=4;    //Bit 5: Sum volt high level 2
      if((u16_lValue1>>6)&0x1) u16_lValue2|=8;    //Bit 6: Sum volt low level 1
      if((u16_lValue1>>7)&0x1) u16_lValue2|=8;    //Bit 7: Sum volt low level 2

      u16_lValue1 = t_message[5];
      if(u16_lValue1&0x1) u16_lValue2|=16;         //Bit 0: Chg temp high level 1
      if((u16_lValue1>>1)&0x1) u16_lValue2|=16;    //Bit 1: Chg temp high level 2
      if((u16_lValue1>>2)&0x1) u16_lValue2|=32;    //Bit 2: Chg temp low level 1
      if((u16_lValue1>>3)&0x1) u16_lValue2|=32;    //Bit 3: Chg temp low level 2
      if((u16_lValue1>>4)&0x1) u16_lValue2|=64;    //Bit 4: Dischg temp high level 1
      if((u16_lValue1>>5)&0x1) u16_lValue2|=64;    //Bit 5: Dischg temp high level 2
      if((u16_lValue1>>6)&0x1) u16_lValue2|=128;   //Bit 6: Dischg temp low level 1
      if((u16_lValue1>>7)&0x1) u16_lValue2|=128;   //Bit 7: Dischg temp low level 2

      u16_lValue1 = t_message[6];
      if(u16_lValue1&0x1) u16_lValue2|=256;        //Bit 0: Chg overcurrent level 1
      if((u16_lValue1>>1)&0x1) u16_lValue2|=256;   //Bit 1: Chg overcurrent level 2
      if((u16_lValue1>>2)&0x1) u16_lValue2|=512;   //Bit 2: Dischg overcurrent level 1
      if((u16_lValue1>>3)&0x1) u16_lValue2|=512;   //Bit 3: Dischg overcurrent level 2
      if((u16_lValue1>>4)&0x1) u16_lValue2|=4096;  //Bit 4: SOC high level 1
      if((u16_lValue1>>5)&0x1) u16_lValue2|=4096;  //Bit 5: SOC high level 2
      if((u16_lValue1>>6)&0x1) u16_lValue2|=4096;  //Bit 6: SOC Low level 1
      if((u16_lValue1>>7)&0x1) u16_lValue2|=4096;  //Bit 7: SOC Low level 2

      u16_lValue1 = t_message[7];
      if(u16_lValue1&0x1) u16_lValue2|=4096;       //Bit 0: Diff volt level 1
      if((u16_lValue1>>1)&0x1) u16_lValue2|=4096;  //Bit 1: Diff volt level 2
      if((u16_lValue1>>2)&0x1) u16_lValue2|=4096;  //Bit 2: Diff temp level 1
      if((u16_lValue1>>3)&0x1) u16_lValue2|=4096;  //Bit 3: Diff temp level 2
      //Bit 4~Bit7:Reserved

      u16_lValue1 = t_message[8];
      if(u16_lValue1&0x1) u16_lValue2|=4096;       //Bit 0: Chg MOS temp high alarm
      if((u16_lValue1>>1)&0x1) u16_lValue2|=4096;  //Bit 1: Dischg MOS temp high alarm
      if((u16_lValue1>>2)&0x1) u16_lValue2|=4096;  //Bit 2: Chg MOS temp sensor err
      if((u16_lValue1>>3)&0x1) u16_lValue2|=4096;  //Bit 3: Dischg MOS temp sensor err
      if((u16_lValue1>>4)&0x1) u16_lValue2|=4096;  //Bit 4: Chg MOS adhesion err
      if((u16_lValue1>>5)&0x1) u16_lValue2|=4096;  //Bit 5: Dischg MOS adhesion err
      if((u16_lValue1>>6)&0x1) u16_lValue2|=4096;  //Bit 6: Chg MOS open circuit err
      if((u16_lValue1>>7)&0x1) u16_lValue2|=4096;  //Bit 7: Discrg MOS open circuit err

      u16_lValue1 = t_message[9];
      if(u16_lValue1&0x1) u16_lValue2|=4096;       //Bit 0: AFE collect chip err
      if((u16_lValue1>>1)&0x1) u16_lValue2|=4096;  //Bit 1: Voltage collect dropped
      if((u16_lValue1>>2)&0x1) u16_lValue2|=4096;  //Bit 2: Cell temp sensor err
      if((u16_lValue1>>3)&0x1) u16_lValue2|=4096;  //Bit 3: EEPROM err
      if((u16_lValue1>>4)&0x1) u16_lValue2|=4096;  //Bit 4: RTC err
      if((u16_lValue1>>5)&0x1) u16_lValue2|=4096;  //Bit 5: Precharge failure
      if((u16_lValue1>>6)&0x1) u16_lValue2|=4096;  //Bit 6: Communication failure
      if((u16_lValue1>>7)&0x1) u16_lValue2|=4096;  //Bit 7: Internal communication failure

      u16_lValue1 = t_message[10];
      if(u16_lValue1&0x1) u16_lValue2|=4096;       //Bit 0: Current module fault
      if((u16_lValue1>>1)&0x1) u16_lValue2|=4096;  //Bit 1: Sum voltage detect fault
      if((u16_lValue1>>2)&0x1) u16_lValue2|=1024;  //Bit 2: Short circuit protect fault
      if((u16_lValue1>>3)&0x1) u16_lValue2|=4096;  //Bit 3: Low volt forbidden chg fault
      //Bit4-Bit7：Reserved

      //Byte7: Fault code

      setBmsErrors(dataMappingNr,u16_lValue2);
      break;


    default:
      break;
  }

}
