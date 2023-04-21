// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT
#if 1
#include "devices/DalyBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

static const char *TAG = "DALY_BMS";

static Stream *mPort;
static uint8_t u8_mDevNr, u8_mConnToId;

enum SM_readData {SEARCH_START, SEARCH_END};

//
static uint8_t u8_mNumberOfCells=0;
static uint8_t u8_mNumOfTempSensors=0;

static float    f_mTotalVoltageOld=0xFFFF;
//static uint32_t mqttSendeTimer=0;

//
static void      getDataFromBms(uint8_t address, uint8_t function);
static bool      recvAnswer(uint8_t * t_outMessage, uint8_t packets);
static void      parseMessage(uint8_t * t_message);

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;

bool DalyBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), uint8_t u8_addData)
{
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn=callback;
  u8_mConnToId = u8_addData;
  uint8_t response[DALY_FRAME_SIZE*16];

  #ifdef DALY_DEBUG
  BSC_LOGI(TAG,"DalyBms_readBmsData()");
  #endif

  //for(uint8_t adr=0; adr<u8_mConnToId+1;adr++)
  //{
    getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_BATTERY_SOC);
    if(recvAnswer(response, 1))
    {
      parseMessage(response);

      mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr));
      mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr));
    }
    else return false;
    
    getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_MIN_MAX_VOLTAGE); 
    if(recvAnswer(response,1)) parseMessage(response);
    else return false;

    //getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_MIN_MAX_TEMPERATURE); 
    //if(recvAnswer(response,1)) parseMessage(response);
    //else return false;

    getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_MOS); 
    if(recvAnswer(response,1)) parseMessage(response);
    else return false;

    getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_STATUS); 
    if(recvAnswer(response,1)) parseMessage(response);
    else return false;

    getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_CELL_VOLTAGE); 
    if(recvAnswer(response,u8_mNumberOfCells/3)) parseMessage(response);
    else return false;

    getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_TEMPERATURE); 
    if(recvAnswer(response,1)) parseMessage(response);
    else return false;

    getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_BALLANCER); 
    if(recvAnswer(response,1)) parseMessage(response);
    else return false;

    //getDataFromBms(DALAY_BMS_ADRESS, DALY_REQUEST_FAILURE); 
    //if(recvAnswer(response,1)) parseMessage(response);
    //else return false;
    

  //}

  if(devNr>=2) callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxTxDisable);
  return true;  
}

static void getDataFromBms(uint8_t address, uint8_t function)
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
    recvBytes+=String(u8_lData[x]);
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
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
  usleep(20);
  mPort->write(u8_lData, DALY_FRAME_SIZE);
  mPort->flush();  
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn);
}


static bool recvAnswer(uint8_t *p_lRecvBytes, uint8_t packets)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvBytesCntPacket, u8_checlSum;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START;
  u8_lRecvBytesCnt=0;
  u8_lRecvBytesCntPacket=0;
  u8_checlSum=0;

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
        recvBytes+=String(p_lRecvBytes[x]);
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
          if (u8_lRecvByte == DALAY_START_BYTE)
          {
            p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
            u8_lRecvBytesCnt++;
            u8_checlSum=u8_lRecvByte;
            u8_lRecvBytesCntPacket=1;
            SMrecvState=SEARCH_END;
          }
          break;

        case SEARCH_END:
          p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
          u8_checlSum+=u8_lRecvByte;
          u8_lRecvBytesCnt++;
          u8_lRecvBytesCntPacket++;
          SMrecvState=SEARCH_START;
          break;
      
        default:
          break;
        }
    }

    if(u8_lRecvBytesCnt==u8_lRecvBytesCntPacket)
    {
      //Überprüfe Cheksum
      if(u8_checlSum!=p_lRecvBytes[u8_lRecvBytesCnt-1]) return false; 
    } 

    if(u8_lRecvBytesCnt==(DALY_FRAME_SIZE*packets)) break; //Recv Pakage complete   
  }

  #ifdef DALY_DEBUG
  String recvBytes="";
  for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
  {
    recvBytes+=String(p_lRecvBytes[x]);
    recvBytes+=" ";
  }
  BSC_LOGD(TAG,"RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
  #endif

  return true;
}


static void parseMessage(uint8_t * t_message)
{
  bool     bo_value=false;
  uint8_t  u8_lValue = 0;
  uint16_t u16_lMaxCellmV, u16_lMinCellmV;

  switch (t_message[2]) 
  {
    case DALY_REQUEST_BATTERY_SOC:
      setBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr, ((float)((t_message[4]<<8) | t_message[5]) / 10.0f));
      setBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr, ((float)(((t_message[8]<<8) | t_message[9]) - DALY_CURRENT_OFFSET) / 10.0f));
      setBmsChargePercentage(BT_DEVICES_COUNT+u8_mDevNr, ((uint8_t)((t_message[10]<<8) | t_message[11]) / 10));
      break;

    case DALY_REQUEST_MIN_MAX_VOLTAGE:
      u16_lMaxCellmV = ((t_message[4]<<8) | t_message[5]);
      u16_lMinCellmV = ((t_message[7]<<8) | t_message[8]);
      setBmsMaxCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lMaxCellmV);
      setBmsMaxVoltageCellNumber(BT_DEVICES_COUNT+u8_mDevNr, t_message[6]);
      setBmsMinCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lMinCellmV);
      setBmsMinVoltageCellNumber(BT_DEVICES_COUNT+u8_mDevNr, t_message[9]);
      setBmsAvgVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lMaxCellmV - u16_lMinCellmV);
      break;

    case DALY_REQUEST_MIN_MAX_TEMPERATURE:
      // (t_message[4] - DALY_TEMPERATURE_OFFSET); // temp max
      // (t_message[6] - DALY_TEMPERATURE_OFFSET); // temp min
      // (get.tempMax + get.tempMin) / 2;          // temp average
      break;

    case DALY_REQUEST_MOS:
      //Charge FET
      if(t_message[5]==1) setBmsStateFETsCharge(BT_DEVICES_COUNT+u8_mDevNr,true);
      else setBmsStateFETsCharge(BT_DEVICES_COUNT+u8_mDevNr,false);

      //Discharge FET
      if(t_message[6]==1) setBmsStateFETsDischarge(BT_DEVICES_COUNT+u8_mDevNr,true);
      else setBmsStateFETsDischarge(BT_DEVICES_COUNT+u8_mDevNr,false);

      //only MQTT
      // ((uint32_t)t_message[8]<<0x18) | ((uint32_t)t_message[9]<<0x10) | ((uint32_t)t_message[10]<<0x08) | (uint32_t)t_message[11]; //Remain capacity (mAH)
      break;

    case DALY_REQUEST_STATUS:
      u8_mNumberOfCells = t_message[4];
      u8_mNumOfTempSensors = t_message[5];
      break;

    case DALY_REQUEST_CELL_VOLTAGE:
      for (size_t n = 0; n <= (u8_mNumberOfCells/3); n++)
      {
        for (size_t i = 0; i < 3; i++)
        {
          setBmsCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u8_lValue, (t_message[(n*DALY_FRAME_SIZE)+5+(i*2)]<<8) | t_message[(n*DALY_FRAME_SIZE)+6+(i*2)]);
          u8_lValue++;
          if (u8_lValue == 24) break;
        }
      }
      break;

    case DALY_REQUEST_TEMPERATURE:
      u8_lValue = 0;
  
      for (uint8_t i=0; i<7; i++)
      {
        setBmsTempature(BT_DEVICES_COUNT+u8_mDevNr, u8_lValue, (t_message[5+i]-40));
        u8_lValue++;
        if (u8_lValue+1 == 3) break;
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
      setBmsIsBalancingActive(BT_DEVICES_COUNT+u8_mDevNr, bo_value);
      break;

    
    case DALY_REQUEST_FAILURE:
      break;


    default:
      break;
  }

}




#endif