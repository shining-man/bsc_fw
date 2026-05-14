// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/Pylontech.h"
#include "BscSerial.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

static const char *TAG = "PYLONTECH";

static Stream *mPort;
static uint8_t u8_mTxEnRS485pin, u8_mCountOfPacks, u8_mDevNr;

enum SM_readData {SEARCH_START, SEARCH_END};

//
static float    f_mTotalVoltageOld=0xFFFF;
static uint16_t u16_mBalanceCapacityOld=0xFFFF;
static uint32_t u32_mChargeMAh=0;
static uint32_t u32_mDischargeMAh=0;
static uint16_t u16_mRecvBytesLastMsg=0; //for debug

//
static void      getDataFromBms(BscSerial *bscSerial, uint8_t address, uint8_t function);
static bool      recvAnswer(uint8_t * t_outMessage);
static bool      parseMessage(uint8_t * t_message, uint8_t address);
static void      parseMessage_Alarms(uint8_t * t_message, uint8_t address);

static uint8_t  convertAsciiHexToByte(char a, char b);
static char     convertByteToAsciiHex(uint8_t v);
static void     convertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length);
static uint16_t lCrc(const uint16_t len);
static bool     checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt);
static uint16_t calcCrc(uint8_t *data, const uint16_t i16_lLen);

static serialDevData_s *mDevData;


// devNr=serialPortNumber
bool Pylontech_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData)
{
  bool ret = true;
  mDevData = devData;
  mPort = port;
  u8_mDevNr = devNr;
  uint8_t response[PYLONTECH_MAX_ANSWER_LEN];

  uint8_t u8_lSeplosAdr = devData->bmsAdresse;
  uint8_t u8_lSeplosAdrBmsData = devData->dataMappingNr;

  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG,"Pylontech_readBmsData() devNr=%i, readFromAdr=%i, BmsDataAdr=%i, CountOfPacks=%i",u8_mDevNr,u8_lSeplosAdr,u8_lSeplosAdrBmsData,u8_mCountOfPacks);
  #endif

  getDataFromBms(bscSerial, u8_lSeplosAdr, 0x42);
  if(recvAnswer(response))
  {
    ret = parseMessage(response, u8_lSeplosAdrBmsData);

    //mqtt
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, u8_lSeplosAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(u8_lSeplosAdrBmsData));
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, u8_lSeplosAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(u8_lSeplosAdrBmsData));
  }
  else
  {
    ret=false;
  }

  if(ret==true)
  {
    getDataFromBms(bscSerial, u8_lSeplosAdr, 0x44); //Alarms
    if(recvAnswer(response))
    {
      parseMessage_Alarms(response, u8_lSeplosAdrBmsData);
    }
    else
    {
      ret=false;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(25));
  return ret;
}

static void getDataFromBms(BscSerial *bscSerial, uint8_t address, uint8_t function)
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
  BSC_LOGI(TAG,"crc=%i", crc);
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
  BSC_LOGI(TAG,"sendBytes: %s", recvBytes.c_str());
  //log_print_buf(u8_lSendData, 20);
  #endif


  //TX
  bscSerial->sendSerialData(mPort, u8_mDevNr, u8_lSendData, 20);
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

      #ifndef UTEST_RESTAPI
      BSC_LOGE(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
      #endif
      #ifdef SEPLOS_DEBUG
      String recvBytes="";
      for(uint8_t x=0;x<u8_lRecvBytesCnt;x++)
      {
        recvBytes+="0x";
        recvBytes+=String(p_lRecvBytes[x],16);
        recvBytes+=" ";
      }
      BSC_LOGI(TAG,"Timeout: RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
      #endif
      return false;
    }

    //Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch (SMrecvState)
      {
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
    if(u8_lRecvBytesCnt>=PYLONTECH_MAX_ANSWER_LEN) return false; //Answer too long!
  }
  u16_mRecvBytesLastMsg=u8_lRecvBytesCnt; //for debug

  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG,"RecvBytes=%i",u8_lRecvBytesCnt);
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
      BSC_LOGI(TAG,"%s",recvBytes.c_str());
      recvBytes="";
      u8_logByteCount=0;
    }
  }
  BSC_LOGI(TAG,"%s",recvBytes.c_str());
  //log_print_buf(p_lRecvBytes, u8_lRecvBytesCnt);
  #endif

  //Überprüfe Cheksum
  if(!checkCrc(p_lRecvBytes,u8_lRecvBytesCnt))return false;

  return true;
}


static bool parseMessage(uint8_t * t_message, uint8_t address)
{
  auto getByteFromMsg = [&](size_t i) -> uint8_t {
    return convertAsciiHexToByte(t_message[i * 2], t_message[(i * 2) + 1]);
  };
  auto getU16FromMsg = [&](size_t i) -> uint16_t {
    return (uint16_t(getByteFromMsg(i)) << 8) | uint16_t(getByteFromMsg(i + 1));
  };
  auto getS16FromMsg = [&](size_t i) -> int16_t {
    return static_cast<int16_t>(getU16FromMsg(i));
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

  u8_lNumOfCells = getByteFromMsg(8);  //Number of cells
  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG, "Number of cells: %d", u8_lNumOfCells);
  #endif

  if(u8_lNumOfCells == 0)
  {
    BSC_LOGE(TAG, "Number of cells: %d", u8_lNumOfCells);
    return false;
  }

  for (uint8_t i=0; i<u8_lNumOfCells; i++)
  {
    u16_lZellVoltage = getU16FromMsg(9 + (i * 2));
    setBmsCellVoltage(address,i, (float)(u16_lZellVoltage));

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

  setBmsMaxCellVoltage(address, u16_lCellHigh);
  setBmsMinCellVoltage(address, u16_lCellLow);
  setBmsMaxVoltageCellNumber(address, u8_lZellNumberMaxVoltage);
  setBmsMinVoltageCellNumber(address, u8_lZellNumberMinVoltage);
  setBmsAvgVoltage(address, (float)(u16_lCellSum/u8_lNumOfCells));
  setBmsMaxCellDifferenceVoltage(address,(float)(u16_lZellDifferenceVoltage));


  u8_lMsgoffset = 9+(u8_lNumOfCells*2);

  //   40     Number of temperatures
  uint8_t u8_lCntTempSensors = getByteFromMsg(u8_lMsgoffset);
  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG, "Number of temperature sensors: %d", u8_lCntTempSensors);
  #endif
  if(u8_lCntTempSensors == 0)
  {
    BSC_LOGE(TAG, "Number of temperature sensors: %d", u8_lCntTempSensors);
    return false;
  }

  //   41     0x0B 0xA6      Temperature sensor 1             (2982 - 2731) * 0.1f = 25.1          °C
  //   43     0x0B 0xA0      Temperature sensor 2             (2976 - 2731) * 0.1f = 24.5          °C
  //   45     0x0B 0x97      Temperature sensor 3             (2967 - 2731) * 0.1f = 23.6          °C
  //   47     0x0B 0xA6      Temperature sensor 4             (2982 - 2731) * 0.1f = 25.1          °C
  //   49     0x0B 0xA5      Environment temperature          (2981 - 2731) * 0.1f = 25.0          °C
  for (uint8_t i = 0; i < u8_lCntTempSensors; i++)
  {
    if(i >= NR_OF_BMS_TEMP_SENSORS) break;
    setBmsTempature(address, i, (float)(getU16FromMsg(u8_lMsgoffset + 1 + (i * 2)) - 0xAAB) * 0.1f);
  }

  u8_lMsgoffset=u8_lMsgoffset+1+(u8_lCntTempSensors*2);

  //   51     0xFD 0x5C      Charge/discharge current         signed int?                   A
  setBmsTotalCurrent_int(address, getS16FromMsg(u8_lMsgoffset) * 10);

  //   53     0x14 0xA0      Total battery voltage 
  setBmsTotalVoltage_int(address, (int16_t)(getU16FromMsg(u8_lMsgoffset + 2) / 10));

  //   57     0x0A           Custom number                    10
  // Custom number = 2: Battery capacity <= 65Ah      z.B. US2000
  // Custom number = 4: Battery capacity > 65Ah       z.B. US5000
  uint8_t customNumber = getByteFromMsg(u8_lMsgoffset + 6);

  uint32_t remainCapacity, totalCapacity;
  if(customNumber == 2) {
    //   55     0x34 0x4E      Restkapazität                   13390 * 0.01f = 133.90         Ah
    remainCapacity = (uint32_t)getU16FromMsg(u8_lMsgoffset + 4);

    //   58     0x42 0x68      Battery capacity                 17000 * 0.01f = 170.00        Ah
    totalCapacity = (uint32_t)getU16FromMsg(u8_lMsgoffset + 7);
  } 
  else {
    // Restkapazität
    remainCapacity =
      ((uint32_t)getByteFromMsg(u8_lMsgoffset + 11) << 16) |
      ((uint32_t)getByteFromMsg(u8_lMsgoffset + 12) << 8) |
      (uint32_t)getByteFromMsg(u8_lMsgoffset + 13);

    // Battery capacity
    totalCapacity =
      ((uint32_t)getByteFromMsg(u8_lMsgoffset + 14) << 16) |
      ((uint32_t)getByteFromMsg(u8_lMsgoffset + 15) << 8) |
      (uint32_t)getByteFromMsg(u8_lMsgoffset + 16);
  }

  //   Stage of charge  
  uint32_t socX100 = 0U;
  if (totalCapacity > 0U) {
    remainCapacity = (remainCapacity > totalCapacity) ? totalCapacity : remainCapacity;
    socX100 = (remainCapacity * 10000U + (totalCapacity / 2U)) / totalCapacity;
    if (socX100 > 10000U) socX100 = 10000U;

    // Das "+ total_capacity / 2" sorgt für mathematische Rundung
   
    // Mit einer Nachkommastelle
    /*(remain_capacity * 1000 + total_capacity / 2) / total_capacity; // 238 => 23,8%
    ganzteil = soc_x10 / 10; 
    nachkomma = soc_x10 % 10;*/
  }
  setBmsChargePercentage(address, ROUND(socX100, 100));

  //   67     0x00 0x46      Number of cycles                 70
  uint16_t u16_lCycle = getU16FromMsg(u8_lMsgoffset + 9);


  if(mDevData->bo_sendMqttMsg)
  {
    //Nachrichten senden
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, address, MQTT_TOPIC2_BALANCE_CAPACITY, -1, ROUND(remainCapacity, 100));
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, address, MQTT_TOPIC2_FULL_CAPACITY, -1, ROUND(totalCapacity, 100));
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, address, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);
  }

  return true;
}


static void parseMessage_Alarms(uint8_t * t_message, uint8_t address)
{
  auto getByteFromMsg = [&](size_t i) -> uint8_t {
    return convertAsciiHexToByte(t_message[i * 2], t_message[(i * 2) + 1]);
  };

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

  uint8_t byte;
  uint32_t u32_alarm = 0;
  //uint32_t u32_warnings = 0;

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x  //bit0  single cell overvoltage protection
  #define BMS_ERR_STATUS_CELL_UVP          2  x  //bit1  single cell undervoltage protection
  #define BMS_ERR_STATUS_BATTERY_OVP       4  x  //bit2  whole pack overvoltage protection
  #define BMS_ERR_STATUS_BATTERY_UVP       8  x  //bit3  Whole pack undervoltage protection
  #define BMS_ERR_STATUS_CHG_OTP          16  x  //bit4  charging over temperature protection
  #define BMS_ERR_STATUS_CHG_UTP          32  -  //bit5  charging low temperature protection
  #define BMS_ERR_STATUS_DSG_OTP          64  x  //bit6  Discharge over temperature protection
  #define BMS_ERR_STATUS_DSG_UTP         128  -  //bit7  discharge low temperature protection
  #define BMS_ERR_STATUS_CHG_OCP         256  x  //bit8  charging overcurrent protection
  #define BMS_ERR_STATUS_DSG_OCP         512  x  //bit9  Discharge overcurrent protection
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  -  //bit10 short circuit protection
  #define BMS_ERR_STATUS_AFE_ERROR      2048  -  //bit11 Front-end detection IC error
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  -  //bit12 software lock MOS
  #define BMS_ERR_STATUS_RESERVED1      8192  -  //bit13 Reserved
  #define BMS_ERR_STATUS_RESERVED2     16384  -  //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  -  //bit15 Reserved */


  uint8_t msgOffset = 8; // Nr of Cells
  uint8_t nrOfCells = getByteFromMsg(msgOffset); // Cells
  msgOffset += 1; //Number of cells

  // Cell OVP/UVP
  for (uint8_t i = msgOffset; i < msgOffset + nrOfCells; i++)
  {
    byte = getByteFromMsg(i);
    if(byte == 0x1) u32_alarm |= BMS_ERR_STATUS_CELL_UVP;
    else if(byte == 0x2) u32_alarm |= BMS_ERR_STATUS_CELL_OVP;
  }
  msgOffset += nrOfCells;

  msgOffset += getByteFromMsg(msgOffset) + 1; // Temperatures + count byte

  msgOffset += 3; // Charge current, Module voltage, Discharge current


  for (uint8_t i = msgOffset; i < msgOffset + 5; i++) 
  {
    byte = getByteFromMsg(i);

    // Status 1
    if(i == msgOffset) {
      if(isBitSet(byte, 7)) u32_alarm |= BMS_ERR_STATUS_BATTERY_UVP;
      if(isBitSet(byte, 6)) u32_alarm |= BMS_ERR_STATUS_CHG_OTP;
      if(isBitSet(byte, 5)) u32_alarm |= BMS_ERR_STATUS_DSG_OTP;
      if(isBitSet(byte, 4)) u32_alarm |= BMS_ERR_STATUS_DSG_OCP;
      // Bit 3: n.b.
      if(isBitSet(byte, 2)) u32_alarm |= BMS_ERR_STATUS_CHG_OCP;
      //if(isBitSet(byte, 1)) u32_alarm |= BMS_ERR_STATUS_CELL_UVP;
      if(isBitSet(byte, 0)) u32_alarm |= BMS_ERR_STATUS_BATTERY_OVP;
    }
    // Status 2
    else if(i == msgOffset + 1) {
      if(isBitSet(byte, 2)) setBmsStateFETsDischarge(address, true);
      else setBmsStateFETsDischarge(address, false);

      if(isBitSet(byte, 1)) setBmsStateFETsCharge(address, true);
      else setBmsStateFETsCharge(address, false);
    }

    // Status 3: msgOffset + 4:

    // Status 4: msgOffset + 6:

    // Status 5: msgOffset + 8:
  }

  setBmsErrors(address, u32_alarm);
  //setBmsWarnings(address, u32_warnings);
}

static uint8_t convertAsciiHexToByte(char a, char b)
{
  a = (a<='9') ? a-'0' : (a&0x7)+9;
  b = (b<='9') ? b-'0' : (b&0x7)+9;
  return (a<<4)+b;
}

static char convertByteToAsciiHex(uint8_t v)
{
  return v>=10 ? 'A'+(v-10) : '0'+v;
}

static void convertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length)
{
  if(length==0) return;

  for(size_t i=0; i<length; i++)
  {
    dest[2*i] = convertByteToAsciiHex((data[i] & 0xF0) >> 4);
    dest[2*i+1] = convertByteToAsciiHex(data[i] & 0x0F);
  }
}

static uint16_t lCrc(const uint16_t len)
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
