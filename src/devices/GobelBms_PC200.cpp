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
static void getDataFromBms(uint8_t address, uint8_t function);
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

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;
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

bool GobelBmsPC200_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool ret = true;
  mDevData=devData;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn = callback;
  uint8_t response[GOBELBMS_MAX_ANSWER_LEN];

  uint8_t u8_lGobelAdr = devData->bmsAdresse;
  uint8_t u8_lGobelAdrBmsData = devData->dataMappingNr;


  #ifdef GOBELPC200_DEBUG
  BSC_LOGI(TAG, "GobelBms_readBmsData() devNr=%i, firstAdr=%i, CountOfPacks=%i", devNr, u8_lGobelAdr, u8_mCountOfPacks);
  #endif

  #ifdef GOBELPC200_DEBUG
  BSC_LOGI(TAG, "read data from pack %i", u8_lGobelAdr);
  #endif
  getDataFromBms(u8_lGobelAdr, 0x42);
  if (recvAnswer(response))
  {
    parseMessage(response, u8_lGobelAdrBmsData);

    // mqtt
    mqttPublish(MQTT_TOPIC_BMS_BT, u8_lGobelAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(u8_lGobelAdrBmsData));
    mqttPublish(MQTT_TOPIC_BMS_BT, u8_lGobelAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(u8_lGobelAdrBmsData));
  }
  else ret = false;
  
  if (ret == true)
  {
    vTaskDelay(pdMS_TO_TICKS(25));
    getDataFromBms(u8_lGobelAdr, 0x44); // Alarms
    if (recvAnswer(response))
    {
      //parseMessage_Alarms(response, u8_lGobelAdrBmsData);
    }
    else ret = false;
  }

  if(devNr >= 2) callbackSetTxRxEn(u8_mDevNr, serialRxTx_RxTxDisable);
  vTaskDelay(pdMS_TO_TICKS(25));
  return ret;
}


static void getDataFromBms(uint8_t address, uint8_t function)
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
  u8_lData[6] = 0xFF;         // INFO

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
  callbackSetTxRxEn(u8_mDevNr, serialRxTx_TxEn);
  usleep(20);
  mPort->write(u8_lSendData, 20);
  mPort->flush();
  callbackSetTxRxEn(u8_mDevNr, serialRxTx_RxEn);
}


static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_lRecvBytesCnt, u8_lRecvDataLen, u8_CyclesWithoutData;
  uint32_t u32_lStartTime = millis();
  SMrecvState = SEARCH_START;
  u8_lRecvBytesCnt = 0;
  u8_lRecvDataLen = 0xFF;
  u8_CyclesWithoutData = 0;
  bool bo_lDataComplete = false;

  for (;;)
  {
    // Timeout
    if ((millis() - u32_lStartTime) > 500)
    {
      BSC_LOGE(TAG, "Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
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
      u8_CyclesWithoutData = 0;
    }
    else if (u8_lRecvBytesCnt == 0)
      vTaskDelay(pdMS_TO_TICKS(10)); // Wenn noch keine Daten empfangen wurden, dann setze den Task 10ms aus
    else if (u8_lRecvBytesCnt > 0 && u8_CyclesWithoutData > 10)
      vTaskDelay(pdMS_TO_TICKS(10)); // Wenn trotz empfangenen Daten 10ms wieder nichts empfangen wurde, dann setze den Task 10ms aus
    else                             // Wenn in diesem Zyklus keine Daten Empfangen wurde, dann setze den Task 1ms aus
    {
      u8_CyclesWithoutData++;
      vTaskDelay(pdMS_TO_TICKS(1));
    }

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

/*
Byte | Data
00 | 32 35（VER，that is version 25H，V2.5）
01 | 30 30（ADR，the battery address is 0）
02 | 34 36（CID1，46H）
03 | 30 30（RTN，00H）
04 | 46 30 37 41（LENGTH，F07A，LENID is 07AH，DATAINFO length is 122 ，LCHKSUM is FH）
06 | 30 30（INFOFLAG is 00H。other information is DATAI）
07 | 30 31（PACK number，01H）
08 | 31 30（battery cell number M，is 10H，that has 16 cell）
09 | 30 44 34 32（first cell voltage: 0D42H，that’s 3394mV）
11 | 30 44 31 34（second cell voltage: 0D14H，that’s 3348mV）
13 | 30 44 31 33（third cell voltage: 0D13H，that’s 3347mV）
15 | 30 44 31 33（forth cell voltage: 0D13H，that’s 3347mV）
17 | 30 44 31 33（fifth cell voltage: 0D13H，that’s 3347mV）
19 | 30 44 31 33（sixth cell voltage: 0D13H，that’s 3347mV）
21 | 30 44 31 33（seventh cell voltage: 0D13H，that’s 3347mV）
23 | 30 44 31 33（eighth cell voltage: 0D13H，that’s 3347mV）
25 | 30 44 31 31（ninth cell voltage : 0D11H，that’s 3345mV）
27 | 30 44 31 32（tenth cell voltage: 0D12H，that’s 3346mV）
29 | 30 44 31 33（eleventh cell voltage: 0D13H，that’s 3347mV）
31 | 30 44 31 31（twelfth cell voltage: 0D11H，that’s 3345mV）
33 | 30 44 31 31（thirteenth cell voltage: 0D11H，that’s 3345mV）
35 | 30 44 31 32（fourteenth cell voltage: 0D12H，that’s 3346mV）
37 | 30 44 31 30（fifteenth cell voltage: 0D10H，that’s 3344mV）
39 | 30 44 31 33（sixteenth cell voltage: 0D13H，that’s 3347mV）
41 | 30 36（temperature number N，06H，has 6 temperatures）
42 | 30 42 42 37（first temperature: 0BB7H，that’s 2999，26.9℃）
44 | 30 42 42 37（second temperature: 0BB7H，that’s 2999，26.9℃）
46 | 30 42 42 38（third temperature: 0BB8H，that’s 3000，27.0℃）
48 | 30 42 42 36（forth temperature: 0BB6H，that’s 2998，26.8℃）
50 | 30 42 42 33（fifth temperature （MOS）: 0BB3H，that’s 2995，26.5℃）
52 | 30 42 42 44（sixth temperature（environment）: 0BBDH，that’s 2994，27.5℃）
54 | 30 30 30 30（PACK current，0000H，unit:10mA，range: -327.68A-+327.67A）
56 | 44 31 35 35（PACK total voltage，D155H , that’s 53.589V）
58 | 31 32 38 45（PACK remain capacity，128EH, that’s 47.50AH）
60 | 30 33（user define number P，03H）
61 | 31 33 38 38（PACK full capacity ，1388H , that’s 50.00AH）
63 | 30 30 30 30（cycle times，0000H）
65 | 31 33 38 38（PACK design capacity，1388H , that’s 50.00AH）
67 | 45 33 41 43（CHKSUM，E3ACH）
*/

  //  08 | 31 30（battery cell number M，is 10H，that has 16 cell）
  u8_lNumOfCells = convertAsciiHexToByte(t_message, 8); // Number of cells
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
  float fl_lBmsTemps[3]; // Hier werden die letzten drei Temperaturen zwischengespeichert
  for (uint8_t i = 0; i < u8_lCntTempSensors; i++)
  {
    fl_lBmsTemps[2] = (float)(get16bitFromMsg(u8_lMsgoffset) - 0xAAB) * 0.1;
    u8_lMsgoffset += 2;
    if (i < 3)
      setBmsTempature(address, i, fl_lBmsTemps[2]);
    else if (i >= 3 && i < 5)
      fl_lBmsTemps[i - 3] = fl_lBmsTemps[2];
  }

  // 54 | 30 30 30 30（PACK current，0000H，unit:10mA，range: -327.68A-+327.67A）
  setBmsTotalCurrent_int(address, ((int16_t)get16bitFromMsg(u8_lMsgoffset)*10));
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
  #ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Capacity: %f, %f, soc=%i", u16_lBalanceCapacity, u16_lFullCapacity, getBmsChargePercentage(address));
  #endif
  u16_lBalanceCapacity /= 100;
  u16_lFullCapacity /= 100;

  // 63 | 30 30 30 30（cycle times，0000H）
  uint16_t u16_lCycle = get16bitFromMsg(u8_lMsgoffset);
  u8_lMsgoffset += 2;

  // 65 | 31 33 38 38（PACK design capacity，1388H , that’s 50.00AH）

  if (mDevData->bo_sendMqttMsg)
  {
    // Nachrichten senden
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_TEMPERATURE, 3, fl_lBmsTemps[0]);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_TEMPERATURE, 4, fl_lBmsTemps[1]);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_TEMPERATURE, 5, fl_lBmsTemps[2]);

    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);
  }
}

static void parseMessage_Alarms(uint8_t *t_message, uint8_t address)
{
#ifdef GOBELPC200_DEBUG
  BSC_LOGI(TAG, "parseMessage_Alarms: serialDev=%i", address);
#endif
/*

item content DATAI bytes Remark
1 * PACKnumber M / COMMAND value 1
2 PACK 1warn information See at chart A.18
3 …… ……
M + 1 PACK Mwarn information See at chart A.18

Char A.18 One pack warn information and transmittion order
1 Cell number M 1
2 Cell voltage 1 warn 1
3 Cell voltage 2 warn 1
4 …… ……
M + 1 Cell voltage M warn 1
M + 2 Temperature number N 1
M + 3 Temperature 1 warn 1
M + 4 Temperature 2 warn 1
M + 5 …… ……
M + N + 2 Temperature N warn 1
M + N + 3 PACK charge current warn 1
M + N + 4 PACK total voltage warn 1
M + N + 5 PACK discharge current warn 1
M + N + 6 Protect state 1 1 See at chart A.19
M + N + 7 Protect state 2 1 See at chart A.20
M + N + 8 Instructions state 1 See at chart A.21
M + N + 9 Control state 1 See at chart A.22
M + N + 10 Fault state 1 See at chart A.23
M + N + 11 Balance state1 1 Balance state for 1-8
M + N + 12 Balance state2 1 Balance state for 9-16
M + N + 13 Warn state1 1 See at chart A.24
M + N + 14 Warn state2 1 See at chart A.25
description:
10
—— 00H: normal
—— 01H: below lower limit
—— 02H: above upper limit
—— 80H~EFH: user define
—— F0H: other fault。

Char A.19 Protect state1 explanation
|BIT| content                             | Remark
----------------------------------------------------------------------------------
| 7 | undefine                            |
| 6 | Short circuit                       | 1: Short circuit protect 0: normal
| 5 | Discharge current protect           | 1: Discharge current protect 0: normal
| 4 | charge current protect              | 1: charge current protect 0: normal
| 3 | Lower total voltage protect         | 1: Lower total voltage protect 0: normal
| 2 | Above total voltage protect         | 1: Above total voltage protect 0: normal
| 1 | Lower cell voltage protect          | 1: Above total voltage protect 0: normal
| 0 | Above cell voltage protect          | 1: Above cell voltage protect 0: normal

Char A.20 Protect state 2 explanation
|BIT| content                             | Remark
----------------------------------------------------------------------------------
| 7 | Fully                               | 1: Fully 0: normal
| 6 | Lower Env temperature protect       | 1: Lower Env temperature protect 0:  normal
| 5 | above Env temperature protect       | 1: above Env temperature protect 0:  normal
| 4 | Above MOS temperature protect       | 1: Above MOS temperature protect 0:  normal
| 3 | Lower discharge temperature protect | 1: Lower discharge temperature protect 0: normal
| 2 | Lower charge temperature protect    | 1: Lower charge temperature protect 0:  normal
| 1 | above discharge temperature protect | 1: above discharge temperature protect 0: normal
| 0 | above charge temperature protect    | 1: above charge temperature protect 0:  normal

Char A.21 Instructions state explanation
|BIT| content                             | Remark
----------------------------------------------------------------------------------
| 7 | Heart indicate                      | 1: ON 0: OFF
| 6 | undefine                            |
| 5 | ACin                                | 1: ON 0: normal
| 4 | Reverse indicate                    | 1: ON 0: normal
| 3 | Pack indicate                       | 1: Pack indicate 0: unuse
| 2 | DFET indicate                       | 1: ON 0: OFF
| 1 | * CFET indicate                     | 1: ON 0: OFF
| 0 | Current limit indicate              | 1: ON 0: OFF

Char A.22 Control state explanation
|BIT| content                             | Remark
----------------------------------------------------------------------------------
| 7 | Undefined                           |
| 6 | Undefined                           |
| 5 | LED warn functiuon                  | 1: unenable 0: enable
| 4 | Current limit function              | 1: unenable 0: enable
| 3 | Current limit gear                  | 1: low gear 0: high gear
| 2 | undefine                            |
| 1 | undefine                            |
| 0 | Buzzer warn function                | 1: enable 0: unenable

Char A.23 Fault state explanation
|BIT| content                             | Remark
----------------------------------------------------------------------------------
| 7 | undefine                            |
| 6 | undefine                            |
| 5 | Sample fault                        | 1: fault 0: normal
| 4 | Cell fault                          | 1: fault 0: normal
| 3 | Undefined
| 2 | NTC fault (NTC)                     | 1: fault 0: normal
| 1 | Discharge MOS fault                 | 1: fault 0: normal
| 0 | Charge MOS fault                    | 1: fault 0: normal

Char A.24 Warn state1 explanation
|BIT| content                             | Remark
----------------------------------------------------------------------------------
| 7 | undefine                            |
| 6 | undefine                            |
| 5 | Discharge current warn              | 1: warn 0: normal
| 4 | charge current warn                 | 1: warn 0: normal
| 3 | Lower tatal voltage warn            | 1: warn 0: normal
| 2 | above tatal voltage warn            | 1: warn 0: normal
| 1 | Lower cell voltage warn             | 1: warn 0: normal
| 0 | above cell voltage warn             | 1: warn 0: normal

Char A.25 Warn state2 explanation
|BIT| content                             | Remark
----------------------------------------------------------------------------------
| 7 | Low power warn                      | 1: warn 0: normal
| 6 | High MOS temperature warn           | 1: warn 0: normal
| 5 | low env temperature warn            | 1: warn 0: normal
| 4 | high env temperature warn           | 1: warn 0: normal
| 3 | low discharge temperature warn      | 1: warn 0: normal
| 2 | low charge temperature warn         | 1: warn 0: normal
| 1 | above discharge temperature warn    | 1: warn 0: normal
| 0 | above charge temperature warn       | 1: warn 0: normal
*/

  uint8_t u8_lMsgoffset = 0;
  uint32_t u32_alarm = 0;
  boolean bo_lValue = false;

  uint8_t u8_lNumOfCells = convertAsciiHexToByte(t_message, 8); // Number of cells
#ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Number of cells: %d", u8_lNumOfCells);
#endif

  u8_lMsgoffset = 9;
  for (uint8_t i = 0; i < u8_lNumOfCells; i++)
  {
    uint8_t cellwarn = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
    if (cellwarn == 0x01)
      u32_alarm |= BMS_ERR_STATUS_CELL_UVP;
    else if (cellwarn == 0x02)
      u32_alarm |= BMS_ERR_STATUS_CELL_OVP;
  }

  uint8_t u8_lCntTempSensors = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
#ifdef GOBELPC200_DEBUG
  BSC_LOGD(TAG, "Number of temp sensors: %d", u8_lCntTempSensors);
#endif
  for (uint8_t i = 0; i < u8_lNumOfCells; i++)
  {
    uint8_t tempwarn = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
    // TODO
  }

  u8_lMsgoffset+=10;
  // M + N + 13 Warn state1 1 See at chart A.24
  uint8_t warnstate1 = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
  /*Char A.24 Warn state1 explanation
  |BIT| content                             | Remark
  ----------------------------------------------------------------------------------
  | 7 | undefine                            |
  | 6 | undefine                            |
  | 5 | Discharge current warn              | 1: warn 0: normal
  | 4 | charge current warn                 | 1: warn 0: normal
  | 3 | Lower tatal voltage warn            | 1: warn 0: normal
  | 2 | above tatal voltage warn            | 1: warn 0: normal
  | 1 | Lower cell voltage warn             | 1: warn 0: normal
  | 0 | above cell voltage warn             | 1: warn 0: normal
  */
  if (warnstate1 & (1<<0))
      u32_alarm |= BMS_ERR_STATUS_CELL_OVP;

  if (warnstate1 & (1<<1))
      u32_alarm |= BMS_ERR_STATUS_CELL_UVP;

  if (warnstate1 & (1<<2))
      u32_alarm |= BMS_ERR_STATUS_BATTERY_OVP;

  if (warnstate1 & (1<<3))
      u32_alarm |= BMS_ERR_STATUS_BATTERY_UVP;

  if (warnstate1 & (1<<4))
      u32_alarm |= BMS_ERR_STATUS_CHG_OCP;

  if (warnstate1 & (1<<5))
      u32_alarm |= BMS_ERR_STATUS_DSG_OCP;

  // M + N + 14 Warn state2 1 See at chart A.25
  uint8_t warnstate2 = convertAsciiHexToByte(t_message, u8_lMsgoffset++);
  /*
  Char A.25 Warn state2 explanation
  |BIT| content                             | Remark
  ----------------------------------------------------------------------------------
  | 7 | Low power warn                      | 1: warn 0: normal
  | 6 | High MOS temperature warn           | 1: warn 0: normal
  | 5 | low env temperature warn            | 1: warn 0: normal
  | 4 | high env temperature warn           | 1: warn 0: normal
  | 3 | low discharge temperature warn      | 1: warn 0: normal
  | 2 | low charge temperature warn         | 1: warn 0: normal
  | 1 | above discharge temperature warn    | 1: warn 0: normal
  | 0 | above charge temperature warn       | 1: warn 0: normal
  */
  if (warnstate2 & (1<<0))
      u32_alarm |= BMS_ERR_STATUS_CHG_OTP;

  if (warnstate2 & (1<<1))
      u32_alarm |= BMS_ERR_STATUS_DSG_OTP;

  if (warnstate2 & (1<<2))
      u32_alarm |= BMS_ERR_STATUS_CHG_UTP;

  if (warnstate2 & (1<<3))
      u32_alarm |= BMS_ERR_STATUS_DSG_UTP;

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