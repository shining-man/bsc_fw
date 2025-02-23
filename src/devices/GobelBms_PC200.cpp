// Copyright (c) 2023 dirk
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/GobelBms_PC200.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

/*
Getting pack analog information:
7E 32 35 30 30 34 36 34 32 45 30 30 32 30 31 46 44 33 31 0D （COMMAND = 01H）
Getting pack warn information:
7E 32 35 30 30 34 36 34 34 45 30 30 32 30 31 46 44 32 46 0D （COMMAND = 01H）

7E（SOI）
32 35（VER，that is version 25H，V2.5）
30 30（ADR，the battery address is 0）
34 36（CID1，46H）
34 32（CID2，42H）
45 30 30 32（LENGTH，E002，LENID is 002H，DATAINFO length is 2 ，LCHKSUM is EH）
46 46  INFO FFH
46 44 30 36（CHKSUM，FD06）
0D（EOI）

Response command information explanation:

Command information: 7E 32 35 30 30 34 36 34 32 45 30 30 32 46 46 46 44 30 36 0D
Response information:
7E 32 35 30 30 34 36 30 30 46 30 37 41 30 30 30 31 31 30 30 44 34 32 30 44 31 34 30 44 31 33 30 44 31
33 30 44 31 33 30 44 31 33 30 44 31 33 30 44 31 33 30 44 31 31 30 44 31 32 30 44 31 33 30 44 31 31 30
44 31 31 30 44 31 32 30 44 31 30 30 44 31 33 30 36 30 42 42 37 30 42 42 37 30 42 42 38 30 42 42 36 30
42 42 33 30 42 42 44 30 30 30 30 44 31 35 35 31 32 38 45 30 33 31 33 38 38 30 30 30 30 31 33 38 38 45
33 41 43 0D

Response information explanation:
7E（SOI）
32 35（VER，that is veision 25H，V2.5）
30 30（ADR，the battery address is 0）
34 36（CID1，46H）
30 30（RTN，00H）
46 30 37 41（LENGTH，F07A，LENID is 07AH，DATAINFO length is 122 ，LCHKSUM is FH）
30 30（INFOFLAG is 00H。other information is DATAI）
30 31（PACK number，01H）
31 30（battery cell number M，is 10H，that has 16 cell）
30 44 34 32（first cell voltage: 0D42H，that’s 3394mV）
30 44 31 34（second cell voltage: 0D14H，that’s 3348mV）
30 44 31 33（third cell voltage: 0D13H，that’s 3347mV）
30 44 31 33（forth cell voltage: 0D13H，that’s 3347mV）
30 44 31 33（fifth cell voltage: 0D13H，that’s 3347mV）
30 44 31 33（sixth cell voltage: 0D13H，that’s 3347mV）
30 44 31 33（seventh cell voltage: 0D13H，that’s 3347mV）
30 44 31 33（eighth cell voltage: 0D13H，that’s 3347mV）
30 44 31 31（ninth cell voltage : 0D11H，that’s 3345mV）
30 44 31 32（tenth cell voltage: 0D12H，that’s 3346mV）
30 44 31 33（eleventh cell voltage: 0D13H，that’s 3347mV）
30 44 31 31（twelfth cell voltage: 0D11H，that’s 3345mV）
30 44 31 31（thirteenth cell voltage: 0D11H，that’s 3345mV）
30 44 31 32（fourteenth cell voltage: 0D12H，that’s 3346mV）
30 44 31 30（fifteenth cell voltage: 0D10H，that’s 3344mV）
30 44 31 33（sixteenth cell voltage: 0D13H，that’s 3347mV）
30 36（temperature number N，06H，has 6 temperatures）
30 42 42 37（first temperature: 0BB7H，that’s 2999，26.9℃）
30 42 42 37（second temperature: 0BB7H，that’s 2999，26.9℃）
30 42 42 38（third temperature: 0BB8H，that’s 3000，27.0℃）
30 42 42 36（forth temperature: 0BB6H，that’s 2998，26.8℃）
30 42 42 33（fifth temperature （MOS）: 0BB3H，that’s 2995，26.5℃）
30 42 42 44（sixth temperature（environment）: 0BBDH，that’s 2994，27.5℃）
30 30 30 30（PACK current，0000H，unit:10mA，range: -327.68A-+327.67A）
44 31 35 35（PACK total voltage，D155H , that’s 53.589V）
31 32 38 45（PACK remain capacity，128EH, that’s 47.50AH）
30 33（user define number P，03H）
31 33 38 38（PACK full capacity ，1388H , that’s 50.00AH）
30 30 30 30（cycle times，0000H）
31 33 38 38（PACK design capacity，1388H , that’s 50.00AH）
45 33 41 43（CHKSUM，E3ACH）
0D（EOI）
*/

enum SM_readData
{
  SEARCH_START,
  SEARCH_END
};

static const char *TAG = "GOBEL_BMS_PC200";
static Stream *mPort;

//
static uint8_t u8_mDevNr, u8_mTxEnRS485pin, u8_mCountOfPacks;

//
static void getDataFromBms(BscSerial *bscSerial, uint8_t address, uint8_t function);
static bool recvAnswer(uint8_t *t_outMessage);
static void parseMessage(uint8_t *t_message, uint8_t address);
static void parseMessage_Alarms(uint8_t *t_message, uint8_t address);

static uint8_t convertAsciiHexToByte(char a, char b);
static uint8_t convertAsciiHexToByte(uint8_t *p, size_t offset);
static char convertByteToAsciiHex(uint8_t v);
static void convertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length);
static uint16_t lCrc(const uint16_t len);
static bool checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt);
static uint16_t calcCrc(uint8_t *data, const uint16_t i16_lLen);


static serialDevData_s *mDevData;

static void BSC_Dump_MSGD(const char * tag, const char * txt, const uint8_t* ptr, const size_t len)
{
  String recvBytes = "";
  uint8_t u8_logByteCount = 0;
  for (size_t x = 0; x < len; x++)
  {
    u8_logByteCount++;
    recvBytes += "0x";
    recvBytes += String(ptr[x], 16);
    recvBytes += " ";
    if (u8_logByteCount == 20)
    {
      BSC_LOGD(tag, "%s%i: %s", txt, len, recvBytes.c_str());
      recvBytes = "";
      u8_logByteCount = 0;
    }
  }
  BSC_LOGD(tag, "%s%i: %s", txt, len, recvBytes.c_str());
}

bool GobelBmsPC200_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData)
{
  bool ret = true;
  mDevData=devData;
  mPort = port;
  u8_mDevNr = devNr;
  uint8_t response[GOBELBMS_MAX_ANSWER_LEN];

  uint8_t u8_lGobelAdr = devData->bmsAdresse;
  uint8_t u8_lGobelAdrBmsData = devData->dataMappingNr;


  #ifdef GOBELPC200_DEBUG
  BSC_LOGI(TAG, "GobelBms_readBmsData() devNr=%i, firstAdr=%i, CountOfPacks=%i", devNr, u8_lGobelAdr, u8_mCountOfPacks);
  #endif

  #ifdef GOBELPC200_DEBUG
  BSC_LOGI(TAG, "read data from pack %i", u8_lGobelAdr);
  #endif
  getDataFromBms(bscSerial, u8_lGobelAdr, 0x42);
  if (recvAnswer(response))
  {
    parseMessage(response, u8_lGobelAdrBmsData);

    // mqtt
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, u8_lGobelAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(u8_lGobelAdrBmsData));
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, u8_lGobelAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(u8_lGobelAdrBmsData));
  }
  else ret = false;
  
  if (ret == true)
  {
    vTaskDelay(pdMS_TO_TICKS(25));
    getDataFromBms(bscSerial, u8_lGobelAdr, 0x44); // Alarms
    if (recvAnswer(response))
    {
      parseMessage_Alarms(response, u8_lGobelAdrBmsData);
    }
    else ret = false;
  }

  vTaskDelay(pdMS_TO_TICKS(25));
  return ret;
}


static void getDataFromBms(BscSerial *bscSerial, uint8_t address, uint8_t function)
{
  uint16_t lenid = lCrc(2);

  const uint16_t frame_len = 7;
  uint8_t u8_lData[9];
  uint8_t u8_lSendData[20];

  u8_lData[0] = 0x25;         // VER
  u8_lData[1] = address;      // ADDR
  u8_lData[2] = 0x46;         // CID1
  u8_lData[3] = function;     // CID2 (0x42)
  u8_lData[4] = (lenid >> 8); // LCHKSUM (0xE0)
  u8_lData[5] = (lenid >> 0); // LENGTH (0x02)
  u8_lData[6] = 0xff;         // INFO (address ?)

  convertByteToAsciiHex(&u8_lSendData[1], &u8_lData[0], frame_len);

  uint16_t crc = calcCrc(&u8_lSendData[1], frame_len * 2);
#ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "adr=%i, crc=%i", address, crc);
#endif
  u8_lData[7] = (crc >> 8); // CHKSUM (0xFD)
  u8_lData[8] = (crc >> 0); // CHKSUM (0x37)
  convertByteToAsciiHex(&u8_lSendData[15], &u8_lData[7], 2);

  u8_lSendData[0] = 0x7E;  // SOF (0x7E)
  u8_lSendData[19] = 0x0D; // EOF (0x0D)

#ifdef GOBELPC200_DEBUG
  BSC_Dump_MSGD(TAG, "SendBMSData=", u8_lData, 9);
  BSC_Dump_MSGD(TAG, "SendBMS=", u8_lSendData, 20);
#endif

  // TX
  bscSerial->sendSerialData(mPort, u8_mDevNr, u8_lSendData, 20);
}


static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvDataLen;
  uint32_t u32_lStartTime = millis();
  SMrecvState = SEARCH_START;
  u8_lRecvBytesCnt = 0;
  u8_lRecvDataLen = 0xFF;
  bool bo_lDataComplete = false;

  for (;;)
  {
    // Timeout
    if ((millis() - u32_lStartTime) > 500)
    {
      BSC_LOGE2(TAG, "Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
      #ifdef GOBELPC200_DEBUG
        BSC_Dump_MSGD(TAG, "Timeout: RecvBytes=", p_lRecvBytes, u8_lRecvBytesCnt);
      #endif
      return false;
    }

    // Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();

      switch (SMrecvState)
      {
      case SEARCH_START:
        if (u8_lRecvByte == 0x7E)
        {
          SMrecvState = SEARCH_END;
        }
        break;

      case SEARCH_END:
        p_lRecvBytes[u8_lRecvBytesCnt] = u8_lRecvByte;
        if (u8_lRecvByte == 0x0D)
        {
          bo_lDataComplete = true;
          break;
        }

        u8_lRecvBytesCnt++;
        break;

      default:
        break;
      }
    }
    else vTaskDelay(pdMS_TO_TICKS(10));
    

    if (bo_lDataComplete)
      break; // Recv Pakage complete
  }

#ifdef GOBELPC200_DEBUG
  BSC_Dump_MSGD(TAG, "RecvBytes=", p_lRecvBytes, u8_lRecvBytesCnt);
#endif

  // Überprüfe Checksum
  if (!checkCrc(p_lRecvBytes, u8_lRecvBytesCnt))
    return false;

  return true;
}


static void parseMessage(uint8_t *t_message, uint8_t address)
{
  // lambda get16bitFromMsg(i)
  auto get16bitFromMsg = [&](size_t i) -> uint16_t
  {
    return (uint16_t(convertAsciiHexToByte(t_message, i) << 8) |
           (uint16_t(convertAsciiHexToByte(t_message, i + 1) << 0)));
  };

  #ifdef GOBELPC200_DEBUG
  BSC_LOGI(TAG, "parseMessage: serialDev=%i", address);
  #endif

  uint8_t u8_lNumOfCells = 0;
  uint16_t u16_lZellVoltage = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint16_t u16_lZellDifferenceVoltage = 0;
  uint8_t u8_lZellNumberMinVoltage = 0;
  uint8_t u8_lZellNumberMaxVoltage = 0;

  uint16_t u16_lCellSum = 0;

  uint16_t u16_lCellLow = 0xFFFF;
  uint16_t u16_lCellHigh = 0x0;

  uint8_t u8_lMsgoffset = 0;

  //  08 | 31 30（battery cell number M，is 10H，that has 16 cell）
  u8_lNumOfCells = convertAsciiHexToByte(t_message, 8); // Number of cells
  if(u8_lNumOfCells == 0) {
    BSC_LOGE(TAG, "Number of cells is 0!");
    return;
  }
  #ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Number of cells: %d", u8_lNumOfCells);
  #endif

  u8_lMsgoffset = 9;
  for (uint8_t i = 0; i < u8_lNumOfCells; i++)
  {
    u16_lZellVoltage = get16bitFromMsg(u8_lMsgoffset);
    setBmsCellVoltage(address, i, (float)(u16_lZellVoltage));
    u8_lMsgoffset += 2;

    u16_lCellSum += u16_lZellVoltage;

    if (u16_lZellVoltage > u16_lCellHigh)
    {
      u16_lCellHigh = u16_lZellVoltage;
      u8_lZellNumberMaxVoltage = i;
    }
    if (u16_lZellVoltage < u16_lCellLow)
    {
      u16_lCellLow = u16_lZellVoltage;
      u8_lZellNumberMinVoltage = i;
    }

    u16_lZellMinVoltage = u16_lCellLow;
    u16_lZellMaxVoltage = u16_lCellHigh;
    u16_lZellDifferenceVoltage = u16_lCellHigh - u16_lCellLow;
  }

  setBmsMaxCellVoltage(address, u16_lCellHigh);
  setBmsMinCellVoltage(address, u16_lCellLow);
  setBmsMaxVoltageCellNumber(address, u8_lZellNumberMaxVoltage);
  setBmsMinVoltageCellNumber(address, u8_lZellNumberMinVoltage);
  setBmsAvgVoltage(address, (float)(u16_lCellSum / u8_lNumOfCells));
  setBmsMaxCellDifferenceVoltage(address, (float)(u16_lZellDifferenceVoltage));

  // 41 | 30 36（temperature number N，06H，has 6 temperatures
  uint8_t u8_lCntTempSensors = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
#ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Number of temperature sensors: %d", u8_lCntTempSensors);
#endif

  // 42 | 30 42 42 37（first temperature: 0BB7H，that’s 2999，26.9℃）
  // 44 | 30 42 42 37（second temperature: 0BB7H，that’s 2999，26.9℃）
  // 46 | 30 42 42 38（third temperature: 0BB8H，that’s 3000，27.0℃）
  // 48 | 30 42 42 36（forth temperature: 0BB6H，that’s 2998，26.8℃）
  // 50 | 30 42 42 33（fifth temperature （MOS）: 0BB3H，that’s 2995，26.5℃）
  // 52 | 30 42 42 44（sixth temperature（environment）: 0BBDH，that’s 2994，27.5℃）
  
  for (uint8_t i = 0; i < u8_lCntTempSensors; i++)
  {
    if(i < 6) setBmsTempatureI16(address, i, (get16bitFromMsg(u8_lMsgoffset) - 0xAAB) * 10);
    u8_lMsgoffset += 2;
  }

  // 54 | 30 30 30 30（PACK current，0000H，unit:10mA，range: -327.68A-+327.67A）
  setBmsTotalCurrent_int(address, (int16_t)get16bitFromMsg(u8_lMsgoffset));
  u8_lMsgoffset += 2;

  // 56 | 44 31 35 35（PACK total voltage，D155H , that’s 53.589V）
  float f_lTotalVoltage = (float)get16bitFromMsg(u8_lMsgoffset) * 0.001f;
  setBmsTotalVoltage(address, f_lTotalVoltage);
  u8_lMsgoffset += 2;

  // 58 | 31 32 38 45（PACK remain capacity，128EH, that’s 47.50AH）
  uint16_t u16_lBalanceCapacity = get16bitFromMsg(u8_lMsgoffset);
  u8_lMsgoffset += 3;

  // 60 | 30 33（user define number P，03H）
  // 61 | 31 33 38 38（PACK full capacity ，1388H , that’s 50.00AH）
  uint16_t u16_lFullCapacity = get16bitFromMsg(u8_lMsgoffset);
  u8_lMsgoffset += 2;

  setBmsChargePercentage(address, (float)((float)u16_lBalanceCapacity / (float)u16_lFullCapacity * 100.0f));
  if(u16_lFullCapacity == 0) {
    BSC_LOGE(TAG, "Full Capacity is 0!");
    return;
  }
  #ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Capacity: %i %i, soc=%i", u16_lBalanceCapacity, u16_lFullCapacity, getBmsChargePercentage(address));
  #endif
  u16_lBalanceCapacity /= 100;
  u16_lFullCapacity /= 100;

  // 63 | 30 30 30 30（cycle times，0000H）
  uint16_t u16_lCycle = get16bitFromMsg(u8_lMsgoffset);
  u8_lMsgoffset += 2;

  // 65 | 31 33 38 38（PACK design capacity，1388H , that’s 50.00AH）

  // Nachrichten senden
  if (mDevData->bo_sendMqttMsg)
  {
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, address, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, address, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, address, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);
  }
}


static void parseMessage_Alarms(uint8_t *t_message, uint8_t address)
{
  #ifdef GOBELPC200_DEBUG
  BSC_LOGI(TAG, "parseMessage: serialDev=%i", u8_mDevNr + address);
  #endif

  uint8_t warnstate = 0;
  uint8_t u8_lMsgoffset = 0;
  uint32_t u32_alarm = 0;
  uint32_t u32_warnings = 0;
  boolean bo_lValue = false;


  uint8_t u8_lNumOfCells = convertAsciiHexToByte(t_message, 8); // Number of cells
  #ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Number of cells: %d", u8_lNumOfCells);
  #endif

  u8_lMsgoffset = 9;
  for (uint8_t i = 0; i < u8_lNumOfCells; i++)
  {
    warnstate = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
    if (warnstate == 0x01)
      u32_warnings |= BMS_ERR_STATUS_CELL_UVP;
    else if (warnstate == 0x02)
      u32_warnings |= BMS_ERR_STATUS_CELL_OVP;
  }


  /*uint8_t u8_lCntTempSensors = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
  #ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Number of temp sensors: %d", u8_lCntTempSensors);
  #endif
  for (uint8_t i = 0; i < u8_lNumOfCells; i++)
  {
    //warnstate = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
    // TODO
  }*/


  /*7 undefine 
    6 Short circuit 
    5 Discharge current protect 
    4 charge current protect 
    3 Lower total voltage protect 
    2 Above total voltage protect 
    1 Lower cell voltage protect 
    0 Above cell voltage protect*/
  warnstate = convertAsciiHexToByte(t_message, 35);
  if (isBitSet(warnstate, 0))
      u32_alarm |= BMS_ERR_STATUS_CELL_OVP;

  if (isBitSet(warnstate, 1))
      u32_alarm |= BMS_ERR_STATUS_CELL_UVP;

  if (isBitSet(warnstate, 2))
      u32_alarm |= BMS_ERR_STATUS_BATTERY_OVP;

  if (isBitSet(warnstate, 3))
      u32_alarm |= BMS_ERR_STATUS_BATTERY_UVP;

  if (isBitSet(warnstate, 4))
      u32_alarm |= BMS_ERR_STATUS_CHG_OCP;

  if (isBitSet(warnstate, 5))
      u32_alarm |= BMS_ERR_STATUS_DSG_OCP;

  if (isBitSet(warnstate, 6))
      u32_alarm |= BMS_ERR_STATUS_SHORT_CIRCUIT;


  /*7 Fully 
    6 Lower Env temperature protect 
    5 above Env temperature protect 
    4 Above MOS temperature protect 
    3 Lower discharge temperature protect
    2 Lower charge temperature protect 
    1 above discharge temperature protect
    0 above charge temperature protect */
  warnstate = convertAsciiHexToByte(t_message, 36);
  if (isBitSet(warnstate, 0))
      u32_alarm |= BMS_ERR_STATUS_CHG_OTP;

  if (isBitSet(warnstate, 1))
      u32_alarm |= BMS_ERR_STATUS_DSG_OTP;

  if (isBitSet(warnstate, 2))
      u32_alarm |= BMS_ERR_STATUS_CHG_UTP;

  if (isBitSet(warnstate, 3))
      u32_alarm |= BMS_ERR_STATUS_DSG_UTP;


  /*7 Heart indicate 
    6 undefine 
    5 ACin 
    4 Reverse indicate 
    3 Pack indicate 
    2 DFET indicate 
    1 CFET indicate 
    0 Current limit indicate */
  warnstate = convertAsciiHexToByte(t_message, 37);
  if (isBitSet(warnstate, 1)) setBmsStateFETsCharge(address, true);
  else setBmsStateFETsCharge(address, false);
      
  if (isBitSet(warnstate, 1)) setBmsStateFETsDischarge(address, true);
  else setBmsStateFETsDischarge(address, false);
      

/*1 Discharge MOS fault
  0 Charge MOS fault */
  warnstate = convertAsciiHexToByte(t_message, 39);
  if (isBitSet(warnstate, 0))
      u32_alarm |= BMS_ERR_STATUS_SHORT_CIRCUIT;

  if (isBitSet(warnstate, 1))
      u32_alarm |= BMS_ERR_STATUS_SHORT_CIRCUIT;


  setBmsWarnings(address, u32_warnings);
  setBmsErrors(address, u32_alarm);
}


uint8_t convertAsciiHexToByte(char a, char b)
{
  a = (a <= '9') ? a - '0' : (a & 0x7) + 9;
  b = (b <= '9') ? b - '0' : (b & 0x7) + 9;
  return (a << 4) + b;
}

uint8_t convertAsciiHexToByte(uint8_t *p, size_t offset)
{
  char a = p[offset * 2];
  char b = p[1 + offset * 2];

  a = (a <= '9') ? a - '0' : (a & 0x7) + 9;
  b = (b <= '9') ? b - '0' : (b & 0x7) + 9;
  return (a << 4) + b;
}

static char convertByteToAsciiHex(uint8_t v)
{
  return v >= 10 ? 'A' + (v - 10) : '0' + v;
}

void convertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length)
{
  if (length == 0)
    return;

  for (size_t i = 0; i < length; i++)
  {
    dest[2 * i] = convertByteToAsciiHex((data[i] & 0xF0) >> 4);
    dest[2 * i + 1] = convertByteToAsciiHex(data[i] & 0x0F);
  }
}

uint16_t lCrc(const uint16_t len)
{
  uint16_t u16_lLcrc = 0x0000;

  if (len == 0)
    return 0x0000;

  u16_lLcrc = (len & 0xf) + ((len >> 4) & 0xf) + ((len >> 8) & 0xf);
  u16_lLcrc = ~(u16_lLcrc % 16) + 1;

  return (u16_lLcrc << 12) + len; // 4 byte checksum + 12 bytes length
}

static bool checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt)
{
  uint16_t u16_lCrc = calcCrc(recvMsg, u8_lRecvBytesCnt - 4);
  uint16_t u16_lRemoteCrc = (uint16_t)convertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt - 2], recvMsg[u8_lRecvBytesCnt - 1]) |
                            (uint16_t)(convertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt - 4], recvMsg[u8_lRecvBytesCnt - 3])) << 8;

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
  for (uint16_t i = 0; i < i16_lLen; i++)
  {
    u16_lSum = u16_lSum + data[i];
  }
  u16_lSum = ~u16_lSum;
  u16_lSum++;
  return u16_lSum;
}