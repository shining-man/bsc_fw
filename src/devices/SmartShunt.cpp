// Copyright (c) 2023 shiningman
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/SmartShunt.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include "WebSettings.h"

#define smartshunt_id_main_voltage    0xED8D
#define smartshunt_id_current         0xED8F
#define smartshunt_id_power           0xED8E
#define smartshunt_id_Consumed_Ah     0xEEFF
#define smartshunt_id_SOC             0x0FFF

static const char *TAG = "SMARTSHUNT";

static Stream *mPort;
static uint8_t u8_mDevNr;
static void      getDataFromBms(uint16_t ID_Get);
static bool      recvAnswer(uint8_t * t_outMessage);
static void      parseMessage(uint8_t * t_message, uint8_t address);
uint8_t          SmartShuntconvertAsciiHexToByte(char a, char b);
static char      SmartShuntconvertByteToAsciiHex(uint8_t v);
void             SmartShuntconvertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length);
static bool      hexIsValid(const uint8_t* buffer, int size);

enum States {
  IDLE,
  RECORD_BEGIN,
  RECORD_NAME,
  RECORD_VALUE,
  CHECKSUM,
  RECORD_HEX
};

enum SM_readData {SEARCH_START, SEARCH_END};

int mState;                                 // current state
char * mTextPointer;                        // pointer to the private buffer we're writing to, name or value
char mName[9];                              // buffer for the field name
char mValue[33];                            // buffer for the field value


static constexpr char checksumTagName[] = "CHECKSUM";

uint8_t u8_tSoc;   // %
int16_t i16_tVolt; // mV
int16_t i16_tCurr; // mA

uint8_t rxValues=0;

// Test
static uint16_t errCntSmartShunt=0;

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;
static serialDevData_s *mDevData;

//https://www.victronenergy.com/live/vedirect_protocol:faq


bool SmartShunt_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool ret = true;
  mDevData = devData;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn=callback;
    uint8_t response[SMARTSHUNT_MAX_ANSWER_LEN];


getDataFromBms(smartshunt_id_main_voltage);

if(recvAnswer(response))
{
  ;
}
else
{
  BSC_LOGE(TAG,"Checksum nicht OK - Main Voltage");
}

getDataFromBms(smartshunt_id_current);
if(recvAnswer(response))
{
  ;
}
else
{
  BSC_LOGE(TAG,"Checksum nicht OK - current");
}

getDataFromBms(smartshunt_id_power);
if(recvAnswer(response))
{
  ;
}
else
{
  BSC_LOGE(TAG,"Checksum nicht OK - power");
}

getDataFromBms(smartshunt_id_Consumed_Ah);
if(recvAnswer(response))
{
  ;
}
else
{
  BSC_LOGE(TAG,"Checksum nicht OK - Consumed_Ah");
}

getDataFromBms(smartshunt_id_SOC);
if(recvAnswer(response))
{
  ;
}
else
{
  BSC_LOGE(TAG,"Checksum nicht OK - SOC");
}




/*   bool bo_ret=true;
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

  uint16_t byteReadCntGes=0;
  uint16_t byteReadCnt=0;
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
      byteReadCntGes++;
			inbyteOrg = inbyte = port->read();

      if((inbyte == ':') && (mState != CHECKSUM)) mState = RECORD_HEX;
      if(mState != RECORD_HEX) mChecksum = ((mChecksum+inbyte)&255);
      if(mState != RECORD_HEX) byteReadCnt++;
      inbyte = toupper(inbyte);

      switch(mState)
      {
        case IDLE:
          // wait for \n of the start of an record
          switch(inbyte)
          {
            case '\n': //0xa
              mState = RECORD_BEGIN;
              break;
            case '\r': //0xd //Frame start
              mChecksum = inbyteOrg;
              byteReadCnt=1;
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
              mTextPointer = mValue; // Reset value pointer
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
                //newLabelRecv(mName, mValue);
              }
              mState = RECORD_BEGIN;
              break;
            case '\r': // Skip
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
            if(rxValues==RX_VAL_OK)
            {
              frameEnd();
              bo_break=true;
            }
          }
          else
          {
            errCntSmartShunt++;
            BSC_LOGE(TAG,"Invalid frame (%i)",mChecksum);
            bo_ret=false;
            bo_break=true;
          }
          mChecksum = 0;
          mState = IDLE;
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

  //Buffer leeren
  uint16_t byteDelCnt=0;
  for (unsigned long clearRxBufTime = millis(); millis()-clearRxBufTime<100;)
  {
    if(port->available())
    {
      port->read();
      byteDelCnt++;
    }
    else break;
  }


  if(devNr>=2) callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxTxDisable);
  //BSC_LOGI(TAG,"ret=%d, rxVal=%i, delCnt=%i, readCntGes=%i, errCnt=%i",bo_ret, rxValues, byteDelCnt, byteReadCntGes, errCntSmartShunt);
  return bo_ret;

*/

return false;

}


static void getDataFromBms(uint16_t ID_Get)
{
  uint8_t u8_lData[11];
  uint8_t u8_lSendData[20];
  uint8_t chksum;

  // Sendedaten vorbereiten
  u8_lSendData[0]=0x3A;             // Start der Nachricht mit ":"
  u8_lSendData[1]=0x37;             // Command 7 = Get (in ASCII)

  u8_lData[2]=(ID_Get >> 0)&0xFF; // ID der abzufragenden Daten - Low Byte
  u8_lData[3]=(ID_Get >> 8)&0xFF; // ID der abzufragenden Daten - High Byte
  u8_lData[4]=0x00;              // Flag 0x00

  chksum = 0x55-0x07; // Checksum (0x55 - 0x07(Get) - Bytes bis hier)
  for(uint8_t i=2;i<5;i++)
  {
    chksum -= u8_lData[i];

  }

  SmartShuntconvertByteToAsciiHex(&u8_lSendData[2],&u8_lData[2], 3);


  u8_lData[5]=chksum;            // Checksum
  SmartShuntconvertByteToAsciiHex(&u8_lSendData[8],&u8_lData[5], 1);

  u8_lSendData[10]=0x0A;             // Ende Befehl /n (LF)

  #ifdef SMARTSHUNT_DEBUG
    String sendBytes="";
    uint8_t u8_logByteCount=0;
    for(uint8_t z=0;z<11;z++)
    {
      u8_logByteCount++;
      sendBytes+="0x";
      sendBytes+= String(u8_lSendData[z],16);
      sendBytes+=" ";
    }
    BSC_LOGI(TAG,"ENDE SendBytes=%i: %s",u8_logByteCount, sendBytes.c_str());
  #endif

  //TX
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
  usleep(20);
  mPort->write(u8_lSendData, 11);
  mPort->flush();
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxEn);

}

/// @brief
/// @param p_lRecvBytes
/// @return
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
    // wenn innerhalb von 500ms das Telegram noch nicht begonnen hat, dann Timeout
    if( ((millis()-u32_lStartTime)>500) )
    {
        BSC_LOGE(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch (SMrecvState)  {
        case SEARCH_START:
          if (u8_lRecvByte == 0x3A)  //":"
          {
            SMrecvState=SEARCH_END;
          }
          break;

        case SEARCH_END:
          p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
          if(u8_lRecvByte == 0x0A)  // "Linefeed"
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
    if(u8_lRecvBytesCnt>=SMARTSHUNT_MAX_ANSWER_LEN) return false; //Answer too long!
  }

  #ifdef SMARTSHUNT_DEBUG
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
      BSC_LOGI(TAG,"RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
      recvBytes="";
      u8_logByteCount=0;
    }
  }
  BSC_LOGI(TAG,"RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
  #endif


  //Überprüfe Cheksum
  if(!hexIsValid(p_lRecvBytes,u8_lRecvBytesCnt)) return false;

  return true;
}

static void parseMessage(uint8_t * t_message, uint8_t address)
{
;
}

/*
void newLabelRecv(char * mName, char * mValue)
{
  int32_t val;
  //BSC_LOGI(TAG,"New value: name=%s, val=%s",mName,mValue);

  if(strcmp_P(mName, "SOC") == 0)
  {
    sscanf(mValue, "%ld", &val);
    u8_tSoc=(uint8_t)(val/10);
    rxValues|=RX_VAL_SOC;
    //BSC_LOGI(TAG,"SoC=%i, rxValues=%i",u8_tSoc,rxValues);
  }
  else if(strcmp_P(mName, "V") == 0)
  {
    sscanf(mValue, "%ld", &val); // mV
    i16_tVolt=val/10;
    rxValues|=RX_VAL_U;

    if(mDevData->bo_sendMqttMsg) mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, i16_tVolt);
    //BSC_LOGI(TAG,"U=%i, rxValues=%i",i16_tVolt,rxValues);
  }
  else if(strcmp_P(mName, "I") == 0)
  {
    sscanf(mValue, "%ld", &val); // mA
    i16_tCurr=val/10;
    rxValues|=RX_VAL_I;

    if(mDevData->bo_sendMqttMsg) mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, i16_tCurr);
    //(TAG,"I=%i, rxValues=%i",i16_tCurr,rxValues);
  }

  else if(strcmp_P(mName, "P") == 0) // Power
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val); // W
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_POWER, -1, val);
  }
  else if(strcmp_P(mName, "TTG") == 0) // Time to Go/Restlaufzeit
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val); // Minuten
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TIME_TO_GO, -1, val);;
  }
  else if(strcmp_P(mName, "H4") == 0) // Anzahl der Ladezyklen
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val); // Cycle
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CYCLE, -1, val);
  }
  else if(strcmp_P(mName, "H7") == 0) // Minimum Batteriespannung
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLT_MIN, -1, val);
  }
  else if(strcmp_P(mName, "H8") == 0) // Maximum Batteriespannung
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLT_MAX, -1, val);
  }
  else if(strcmp_P(mName, "H9") == 0) // Zeit seit Letztenmal Batterie Voll
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TIME_SINCE_FULL, -1, val);
  }
  else if(strcmp_P(mName, "H10") == 0) // Anzahl der Automatischen Synchros
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_SOC_SYNC_COUNT, -1, val);
  }
  else if(strcmp_P(mName, "H11") == 0) // Anzahl Batterie Unterspannungen Alarme
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLT_MIN_COUNT, -1, val);
  }
  else if(strcmp_P(mName, "H12") == 0) // Anzahl Batterie Überspannungen Alarme
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLT_MAX_COUNT, -1, val);
  }
  else if(strcmp_P(mName, "H17") == 0) //Menge Entladende Energie in kwh
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_AMOUNT_DCH_ENERGY, -1, val/100);
  }
  else if(strcmp_P(mName, "H18") == 0) // Menge Geladene Energie in kwH
  {
    //if(!mDevData->bo_sendMqttMsg) return;
    sscanf(mValue, "%ld", &val);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_AMOUNT_CH_ENERGY, -1, val/100);
  }

  //BSC_LOGI(TAG,"newLabelRecv rxValues=%i",rxValues);
}

*/

void frameEnd()
{
  //BSC_LOGI(TAG,"save values");

  setBmsChargePercentage(BT_DEVICES_COUNT+u8_mDevNr, u8_tSoc);
  setBmsTotalVoltage_int(BT_DEVICES_COUNT+u8_mDevNr, i16_tVolt);
  setBmsTotalCurrent_int(BT_DEVICES_COUNT+u8_mDevNr, i16_tCurr);

}


uint8_t SmartShuntconvertAsciiHexToByte(char a, char b)
{
  a = (a<='9') ? a-'0' : (a&0x7)+9;
  b = (b<='9') ? b-'0' : (b&0x7)+9;
  return (a<<4)+b;
}


static char SmartShuntconvertByteToAsciiHex(uint8_t v)
{
  return v>=10 ? 'A'+(v-10) : '0'+v;
}


void SmartShuntconvertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length)
{
  if(length==0) return;

  for(size_t i=0; i<length; i++)
  {
    dest[2*i] = SmartShuntconvertByteToAsciiHex((data[i] & 0xF0) >> 4);
    dest[2*i+1] = SmartShuntconvertByteToAsciiHex(data[i] & 0x0F);
  }
}

/*
 *	hexIsValid
 *  This function compute checksum and validate hex frame
 */
#define ascii2hex(v) (v-48-(v>='A'?7:0))
#define hex2byte(b) (ascii2hex(*(b)))*16+((ascii2hex(*(b+1))))

static bool hexIsValid(const uint8_t* buffer, int size) {
  uint8_t checksum=0x55-ascii2hex(buffer[1]);
  for (int i=2; i<size; i+=2) checksum -= hex2byte(buffer+i);
  return (checksum==0);
}
