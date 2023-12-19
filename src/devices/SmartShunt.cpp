// Copyright (c) 2023 shiningman
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/SmartShunt.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include "WebSettings.h"

static const char *TAG = "SMARTSHUNT";

static Stream *mPort;
static uint8_t u8_mDevNr;

enum States {
  IDLE,
  RECORD_BEGIN,
  RECORD_NAME,
  RECORD_VALUE,
  CHECKSUM,
  RECORD_HEX
};

int mState;                                 // current state
char * mTextPointer;                        // pointer to the private buffer we're writing to, name or value
char mName[9];                              // buffer for the field name
char mValue[33];                            // buffer for the field value

static constexpr char checksumTagName[] = "CHECKSUM";

uint8_t u8_tSoc;   // %
int16_t i16_tVolt; // mV
int16_t i16_tCurr; // mA

uint8_t rxValues=0;

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;
static serialDevData_s *mDevData;

//https://www.victronenergy.com/live/vedirect_protocol:faq


void newLabelRecv(char * mName, char * mValue)
{
  int32_t val;
  //BSC_LOGI(TAG,"New value: name=%s, val=%s",mName,mValue);

  if(strcmp_P(mName, "SOC") == 0)
  {
    sscanf(mValue, "%ld", &val);
    u8_tSoc=(uint8_t)(val/10);
    rxValues|=RX_VAL_SOC;
    //BSC_LOGI(TAG,"SoC=%i",u8_tSoc);
  }
  else if(strcmp_P(mName, "V") == 0)
  {
    sscanf(mValue, "%ld", &val); // mV
    i16_tVolt=val/10;
    rxValues|=RX_VAL_U;
    //BSC_LOGI(TAG,"U=%i",i16_tVolt);
  }
  else if(strcmp_P(mName, "I") == 0)
  {
    sscanf(mValue, "%ld", &val); // mA
    i16_tCurr=val/10;
    rxValues|=RX_VAL_I;
    //BSC_LOGI(TAG,"I=%i",i16_tCurr);
  }
  /*else if(strcmp_P(mName, "H4") == 0) // Number of charge cycles
  {
    sscanf(mValue, "%ld", &val);
  }*/
  BSC_LOGI(TAG,"newLabelRecv rxValues=%i",rxValues);
}


void frameEnd()
{
  BSC_LOGI(TAG,"U=%i",i16_tVolt);

  setBmsChargePercentage(u8_mDevNr, u8_tSoc);
  setBmsTotalVoltage_int(u8_mDevNr, i16_tVolt);
  setBmsTotalCurrent_int(u8_mDevNr, i16_tCurr);

  //ToDo: insert MQTT
  //CE, TTG, AR, H4, H7, H8, H11, H12, H17, H18
}


bool hexRx(uint8_t inbyte)
{
	return true;
}


bool SmartShunt_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool bo_ret=true;
  bool bo_break=false;
  mDevData=devData;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn=callback;

  uint8_t inbyte=0;
  uint8_t inbyteOrg=0;
  uint16_t	mChecksum; 
  uint32_t u32_lStartTime=millis();
  rxValues=0;

  callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn);

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>300) 
    {
      BSC_LOGI(TAG,"Timeout: Serial=%i", u8_mDevNr);
      bo_ret = false;
      bo_break = true;
    }
    
    if(port->available())
    {
			inbyteOrg = inbyte = port->read();

      if((inbyte == ':') && (mState != CHECKSUM)) mState = RECORD_HEX;
      if(mState != RECORD_HEX) mChecksum += inbyte;
      inbyte = toupper(inbyte);

      switch(mState)
      {
        case IDLE:
          /* wait for \n of the start of an record */
          switch(inbyte)
          {
            case '\n': //0xa
              mState = RECORD_BEGIN;
              break;
            case '\r': //0xd
              BSC_LOGI(TAG,"Frame start");
              mChecksum = inbyteOrg;
            default:
              break;
          }
          break;
        case RECORD_BEGIN:
          mTextPointer = mName;
          *mTextPointer++ = inbyte;
          mState = RECORD_NAME;
          break;
        case RECORD_NAME:
          // The record name is being received, terminated by a \t
          switch(inbyte)
          {
            case '\t': //0x9
              // the Checksum record indicates a EOR
              if(mTextPointer < (mName + sizeof(mName)))
              {
                *mTextPointer = 0;
                if (strcmp(mName, checksumTagName) == 0) {
                  mState = CHECKSUM;
                  break;
                }
              }
              mTextPointer = mValue; /* Reset value pointer */
              mState = RECORD_VALUE;
              break;
            default:
              // add byte to name, but do no overflow
              if(mTextPointer < (mName + sizeof(mName))) *mTextPointer++ = inbyte;
              break;
          }
          break;
        case RECORD_VALUE:
          // The record value is being received.  The \r indicates a new record.
          switch(inbyte)
          {
            case '\n':
              // forward record, only if it could be stored completely
              if(mTextPointer < (mValue + sizeof(mValue)))
              {
                *mTextPointer = 0; // make zero ended
                newLabelRecv(mName, mValue);
              }
              mState = RECORD_BEGIN;
              break;
            case '\r': /* Skip */
              break;
            default:
              // add byte to value, but do no overflow
              if(mTextPointer < (mValue + sizeof(mValue))) *mTextPointer++ = inbyte;
              break;
          }
          break;
        case CHECKSUM:
        {
          if(mChecksum==0) 
          {
            frameEnd();
            if(rxValues==RX_VAL_OK) bo_break=true;
          }
          else
          {
            BSC_LOGE(TAG,"Invalid frame (%i)",mChecksum);
            bo_ret=false;
            bo_break=true;
          }
          mChecksum = 0;
          mState = IDLE;
          //bo_break=true;
          break;
        }
        case RECORD_HEX:
          if(hexRx(inbyte))
          {
            mChecksum = 0;
            mState = IDLE;
          }
          break;
      }
    }
    else
    {
      //When not a new char available
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    if(bo_break) break;
  }

  if(devNr>=2) callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxTxDisable);
  BSC_LOGI(TAG,"SmartShunt_readBmsData bo_ret=%d, rxValues=%i",bo_ret, rxValues);
  return bo_ret; 
}
