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
static uint8_t u8_mDevNr, u8_mTxEnRS485pin;

enum SM_readData {SEARCH_START, SEARCH_END};

//
static float    f_mTotalVoltageOld=0xFFFF;
static uint16_t u16_mBalanceCapacityOld=0xFFFF;
static uint32_t u32_mChargeMAh=0;
static uint32_t u32_mDischargeMAh=0;
static uint32_t mqttSendeTimer=0;

//
static void      getDataFromBms(uint8_t address, uint8_t function);
static bool      recvAnswer(uint8_t * t_outMessage);
static void      parseMessage(uint8_t * t_message);

uint8_t convertAsciiHexToByte(char a, char b);
static char convertByteToAsciiHex(uint8_t v);
void convertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length);
uint16_t lCrc(const uint16_t len);
static bool checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt);
static uint16_t calcCrc(uint8_t *data, const uint16_t i16_lLen);


bool SeplosBms_readBmsData(Stream *port, uint8_t devNr, uint8_t txEnRS485pin)
{
  bool bo_lRet=true;
  mPort = port;
  u8_mDevNr = devNr;
  u8_mTxEnRS485pin = txEnRS485pin;
  uint8_t response[SEPLOSBMS_MAX_ANSWER_LEN];

  ESP_LOGI(TAG,"SeplosBms_readBmsData()");

  getDataFromBms(0, 0x42);
  if(recvAnswer(response))
  {
    parseMessage(response);

    //mqtt
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr));
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr));
  }
  else
  {
    ESP_LOGI(TAG,"Checksum wrong");
    bo_lRet=false;
  }
 

  if(bo_lRet==false) return bo_lRet;
  
  setBmsLastDataMillis(BT_DEVICES_COUNT+u8_mDevNr,millis());

  return bo_lRet;  
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
  u8_lData[6]=0x0;            // VALUE (0x00)

  convertByteToAsciiHex(&u8_lSendData[1], &u8_lData[0], frame_len);

  uint16_t crc = calcCrc(&u8_lSendData[1], frame_len*2);
  ESP_LOGD(TAG,"crc=%i", crc);
  u8_lData[7]=(crc >> 8);  // CHKSUM (0xFD)
  u8_lData[8]=(crc >> 0);  // CHKSUM (0x37)
  convertByteToAsciiHex(&u8_lSendData[15], &u8_lData[7], 2); 

  u8_lSendData[0]=0x7E;   // SOF (0x7E)
  u8_lSendData[19]=0x0D;  // EOF (0x0D)

  #ifdef SEPLOS_DEBUG
  String recvBytes="";
  for(uint8_t x=0;x<9;x++)
  {
    recvBytes+=String(u8_lData[x]);
    recvBytes+=" ";
  }
  ESP_LOGD(TAG,"sendBytes: %s", recvBytes.c_str());
  
  /*recvBytes="";
  for(uint8_t x=0;x<20;x++)
  {
    recvBytes+=String(u8_lSendData[x]);
    recvBytes+=" ";
  }
  ESP_LOGD(TAG,"sendBytes2: %s", recvBytes.c_str());*/
  #endif

  
  //TX
  if(u8_mTxEnRS485pin>0) digitalWrite(u8_mTxEnRS485pin, HIGH); 
  usleep(20);
  mPort->write(u8_lSendData, 20);
  mPort->flush();  
  if(u8_mTxEnRS485pin>0) digitalWrite(u8_mTxEnRS485pin, LOW); 
}


static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvDataLen;
  uint32_t u32_lStartTime=millis();
  SMrecvState=SEARCH_START;
  u8_lRecvBytesCnt=0;
  u8_lRecvDataLen=0xFF;
  bool bo_lDataComplete=false;

  for(;;)
  {
    //Timeout
    if(millis()-u32_lStartTime > 200) 
    {
      ESP_LOGI(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
      #ifdef SEPLOS_DEBUG
      String recvBytes="";
      for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
      {
        recvBytes+=String(p_lRecvBytes[x]);
        recvBytes+=" ";
      }
      ESP_LOGD(TAG,"Timeout: RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
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
            p_lRecvBytes[u8_lRecvBytesCnt]=u8_lRecvByte;
            u8_lRecvBytesCnt++;
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
    }

    if(bo_lDataComplete) break; //Recv Pakage complete   
  }

  #ifdef SEPLOS_DEBUG
  String recvBytes="";
  for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
  {
    recvBytes+=String(p_lRecvBytes[x]);
    recvBytes+=" ";
  }
  ESP_LOGD(TAG,"RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
  #endif

  //Überprüfe Cheksum
  if(!checkCrc(p_lRecvBytes,u8_lRecvBytesCnt)) return false;


  for (uint16_t i=1; i<(u8_lRecvBytesCnt/2); i++)
  {
    p_lRecvBytes[i]=convertAsciiHexToByte(p_lRecvBytes[i*2], p_lRecvBytes[i*2+1]);
  }

  return true;
}

/*
static void checkMsgLen(uint8_t * t_message)
{
  // num_of_cells   frame_size   data_len
  // 8              65           118 (0x76)   guessed
  // 14             77           142 (0x8E)
  // 15             79           146 (0x92)
  // 16             81           150 (0x96)
  // 24             97           182 (0xB6)   guessed
  if (t_message.size() >= 65 && data[8] >= 8 && data[8] <= 24) {
    this->on_telemetry_data_(data);
    return;
  }

  ESP_LOGW(TAG, "Unhandled data received: %s", format_hex_pretty(&data.front(), data.size()).c_str());
}
*/

static void parseMessage(uint8_t * t_message)
{
  //lambda get16bitFromMsg(i)
  auto get16bitFromMsg = [&](size_t i) -> uint16_t {
    return (uint16_t(t_message[i + 0]) << 8) | (uint16_t(t_message[i + 1]) << 0);
  };

  ESP_LOGI(TAG, "parseMessage()");

  uint8_t u8_lNumOfCells = 0;
  uint16_t u16_lZellVoltage = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint16_t u16_lZellDifferenceVoltage = 0;

  uint16_t u16_lCellSum = 0;

  uint16_t u16_lCellLow = 0xFFFF; 
  uint16_t u16_lCellHigh = 0x0;

  uint8_t u8_lMsgoffset=0;

  // ->
  // 0x20 00 46 00 10 96 00 01 10 0C D70CE90CF40CD60CEF0CE50CE10CDC0CE90CF00CE80CEF0CEA0CDA0CDE0CD8060BA60BA00B970BA60BA50BA2FD5C14A0344E0A426803134650004603E8149F0000000000000000
  //
  // *Data*
  //
  // Byte   Address Content: Description                      Decoded content               Coeff./Unit
  //   0    0x20             Protocol version      VER        2.0
  //   1    0x00             Device address        ADR
  //   2    0x46             Device type           CID1       Lithium iron phosphate battery BMS
  //   3    0x00             Function code         CID2       0x00: Normal, 0x01 VER error, 0x02 Chksum error, ...
  //   4    0x10             Data length checksum  LCHKSUM
  //   5    0x96             Data length           LENID      150 / 2 = 75
  //   6    0x00             Data flag
  //   7    0x01             Command group
  //   8    0x10             Number of cells                  16

  uint8_t u8_lCellCnt = t_message[8];
  ESP_LOGI(TAG, "Number of cells: %d", u8_lCellCnt);
  
  //   9      0x0C 0xD7      Cell voltage 1                   3287 * 0.001f = 3.287         V
  //   11     0x0C 0xE9      Cell voltage 2                   3305 * 0.001f = 3.305         V
  //   ...    ...            ...
  //   39     0x0C 0xD8      Cell voltage 16         
  u8_lNumOfCells = t_message[8];  //Number of cells

  for (byte i=0; i<u8_lNumOfCells; i++) 
  {
    u16_lZellVoltage = get16bitFromMsg(9+(i*2));
    setBmsCellVoltage(BT_DEVICES_COUNT+u8_mDevNr,i, (float)(u16_lZellVoltage));

    u16_lCellSum+=u16_lZellVoltage;

    if(u16_lZellVoltage>u16_lCellHigh){u16_lCellHigh=u16_lZellVoltage;}
    if(u16_lZellVoltage<u16_lCellLow){u16_lCellLow=u16_lZellVoltage;}

    u16_lZellMinVoltage=u16_lCellLow;
    u16_lZellMaxVoltage=u16_lCellHigh;
    u16_lZellDifferenceVoltage=u16_lCellHigh-u16_lCellLow; 
  }
  
  setBmsMaxCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellHigh);
  setBmsMinCellVoltage(BT_DEVICES_COUNT+u8_mDevNr, u16_lCellLow);
  setBmsAvgVoltage(BT_DEVICES_COUNT+u8_mDevNr, (float)(u16_lCellSum/u8_lNumOfCells));
  setBmsMaxCellDifferenceVoltage(BT_DEVICES_COUNT+u8_mDevNr,(float)(u16_lZellDifferenceVoltage));
  

  u8_lMsgoffset=9+(u8_lNumOfCells*2);

  //   41     0x06           Number of temperatures           6                             V
  uint8_t u8_lCntTempSensors = t_message[u8_lMsgoffset];
  ESP_LOGI(TAG, "Number of temperature sensors: %d", u8_lCntTempSensors);

  //   42     0x0B 0xA6      Temperature sensor 1             (2982 - 2731) * 0.1f = 25.1          °C
  //   44     0x0B 0xA0      Temperature sensor 2             (2976 - 2731) * 0.1f = 24.5          °C
  //   46     0x0B 0x97      Temperature sensor 3             (2967 - 2731) * 0.1f = 23.6          °C
  //   48     0x0B 0xA6      Temperature sensor 4             (2982 - 2731) * 0.1f = 25.1          °C
  //   50     0x0B 0xA5      Environment temperature          (2981 - 2731) * 0.1f = 25.0          °C
  //   52     0x0B 0xA2      Mosfet temperature               (2978 - 2731) * 0.1f = 24.7          °C
  for (uint8_t i=0; i<3; i++)
  {
    float f_lTemp = (float)get16bitFromMsg(u8_lMsgoffset+1+(i*2)*0.1);
    setBmsTempature(BT_DEVICES_COUNT+u8_mDevNr,i,f_lTemp);
  }

  u8_lMsgoffset=u8_lMsgoffset+1+(u8_lCntTempSensors*2);

  //   54     0xFD 0x5C      Charge/discharge current         signed int?                   A
  float f_lTotalCurrent = (float)((int16_t)get16bitFromMsg(u8_lMsgoffset))*0.01f;
  setBmsTotalCurrent(BT_DEVICES_COUNT+u8_mDevNr,f_lTotalCurrent);

  //   56     0x14 0xA0      Total battery voltage            5280 * 0.01f = 52.80          V
  float f_lTotalVoltage = (float)get16bitFromMsg(u8_lMsgoffset+2)*0.01f;
  setBmsTotalVoltage(BT_DEVICES_COUNT+u8_mDevNr, f_lTotalVoltage);

  //   58     0x34 0x4E      Restkapazität                   13390 * 0.01f = 133.90        Ah
  uint16_t u16_lBalanceCapacity=get16bitFromMsg(u8_lMsgoffset+4)/100;

  //   60     0x0A           Custom number                    10

  //   61     0x42 0x68      Battery capacity                 17000 * 0.01f = 170.00        Ah
  uint16_t u16_lFullCapacity=get16bitFromMsg(u8_lMsgoffset+7)/100;

  //   63     0x03 0x13      Stage of charge                  787 * 0.1f = 78.7             %
  setBmsChargePercentage(BT_DEVICES_COUNT+u8_mDevNr, get16bitFromMsg(u8_lMsgoffset+9)/10);

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


  if(millis()>(mqttSendeTimer+10000))
  {
    //Nachrichten senden
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);
    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_BALANCE_STATUS, -1, u16_lBalanceStatus);
    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_FET_STATUS, -1, u16_lFetStatus);

    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_CHARGED_ENERGY, -1, u32_mChargeMAh);
    //mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT+u8_mDevNr, MQTT_TOPIC2_DISCHARGED_ENERGY, -1, u32_mDischargeMAh);

    mqttSendeTimer=millis();
  }
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

  u16_lLcrc = (len & 0xf) + ((len >> 4) & 0xf) + ((len >> 8) & 0xf);
  u16_lLcrc = ~(u16_lLcrc % 16) + 1;

  return (u16_lLcrc << 12) + len;  // 4 byte checksum + 12 bytes length
}


static bool checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt)
{
  uint16_t u16_lCrc = calcCrc(recvMsg + 1, u8_lRecvBytesCnt-5);
  uint16_t u16_lRemoteCrc = convertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt-2], recvMsg[u8_lRecvBytesCnt-1] |
                           (convertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt-4], recvMsg[u8_lRecvBytesCnt-3]))<<8);

  if (u16_lCrc != u16_lRemoteCrc)
  {
    ESP_LOGW(TAG, "CRC failed: %04X != %04X", u16_lCrc, u16_lRemoteCrc);
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





