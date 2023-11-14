// Copyright (c) 2023 dirk
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <exception>
#include "devices/GobelBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

static const char *TAG = "GOBEL_BMS";
#define GOBEL_DEBUG

Stream *mPort;
/*
Protocol: https://github.com/fancyui/Gobel-Power-RN-BMS-RS485-ModBus

Response Analog
Example: 37 45 11 01 46 00 F0 4D B0 00 C5 5C 01 01 FA 61 00 00 C8 91 35 5B 05 71 48 6D 60 00 03 2F 64 01 00 10 0C 8D 0C 8D 0C 8B 0C 8C 0C 8B 0C 8B 0C 8E 0C 8E 0C 8F 0C 88 0C 89 0C 8D 0C 8E 0C 8D 0C 8D 0C 85 04 0B B0 0B AD 0B AE 0B AD 01 0B B9 01 0B C1 48 D4 F2 A5 E6 78 0D
----------------------------------------------------------------------------------------------
Data            Name                    Unit            Result                      Type
----------------------------------------------------------------------------------------------
37 45           head                    /               /
11              ver                     /               1.1
01              adr                     /               1
46              cid1                    /               /
00              RTN                     /               check RTN table
F0 4D           length                  /               /
B0              cid2                    /               check CID table
00              cid3                    /               check CID table
C5 5C           /                       /               /
01              total pack number       /               1
------------------------------------
01              #1 pack adr             /               1
FA 61           current                 10mA            -14390                      signed
00 00 C8 91     voltage                 1mV             51345
35 5B           remain capacity         10mAh           136590
05              /
71 48           total capacity          10mAh           290000
6D 60           design capacity         10mAh           280000
00 03           cycle number            /               3
2F              SOC                     %               47%
64              SOH                     %               100%
01              parallel number         /               1 (if the pack is slave: 0)
00              slave adr               /               0 (if the pack is master: 0)
10              Cell number             /               16
0C 8D           Cell1 voltage           1mV             3213
0C 8D           Cell2 voltage           1mV             3213
0C 8B           Cell3 voltage           1mV             3211
0C 8C           Cell4 voltage           1mV             3212
0C 8B           Cell5 voltage           1mV             3211
0C 8B           Cell6 voltage           1mV             3211
0C 8E           Cell7 voltage           1mV             3214
0C 8E           Cell8 voltage           1mV             3214
0C 8F           Cell9 voltage           1mV             3215
0C 88           Cell10 voltage          1mV             3208
0C 89           Cell11 voltage          1mV             3209
0C 8D           Cell12 voltage          1mV             3213
0C 8E           Cell13 voltage          1mV             3214
0C 8D           Cell14 voltage          1mV             3213
0C 8D           Cell15 voltage          1mV             3213
0C 85           Cell16 voltage          1mV             3205
04              Cell NTC number         /               4
0B B0           Cell NTC1               0.1K            26.05
0B AD           Cell NTC2               0.1K            25.75
0B AE           Cell NTC3               0.1K            25.85
0B AD           Cell NTC4               0.1K            25.75
01              MOS NTC number          /               1
0B B9           MOS NTC1                0.1K            26.95
01              Ambient NTC number      /               1
0B C1           Ambient NTC             0.1K            27.75
48 D4 F2 A5     crc32                   /               /
-----------------------------------
{#2 pack data}
-----------------------------------
E6 78           chksum                  /               /
0D              end                     /               /

*/

class parser
{
public:
  parser(uint8_t *ptr, uint16_t len) : m_ptr(ptr), m_length(len), m_read(0){};
  void seek(uint16_t n)
  {
    m_read += n;
  }

  uint8_t getuint8()
  {
    uint8_t ret;
    if (m_read >= m_length)
      throw std::out_of_range("no more data");
    ret = m_ptr[m_read++];
    return ret;
  }

  uint16_t getuint16()
  {
    uint16_t ret;
    if ((m_read + 1) >= m_length)
      throw std::out_of_range("no more data");
    ret = (m_ptr[m_read++] << 8) + m_ptr[m_read++];
    return ret;
  }

  uint16_t getuint32()
  {
    uint32_t ret;
    if ((m_read + 3) >= m_length)
      throw std::out_of_range("no more data");
    ret = (m_ptr[m_read++] << 24) + (m_ptr[m_read++] << 16) + (m_ptr[m_read++] << 8) + m_ptr[m_read++];
    return ret;
  }

  int16_t getint16()
  {
    int16_t ret;
    if ((m_read + 1) >= m_length)
      throw std::out_of_range("no more data");
    ret = (m_ptr[m_read++] << 8) + m_ptr[m_read++];
    return ret;
  }

  float gettemp()
  {
    float temp = (float(getuint16() - 2731)) * 0.1;
    return temp;
  }

protected:
  uint8_t *m_ptr;
  uint16_t m_length;
  uint16_t m_read;
};

static uint8_t u8_mDevNr, u8_mConnToId;
;
static uint16_t u16_mLastRecvBytesCnt;

enum SM_readData
{
  SEARCH_START_BYTE1,
  SEARCH_START_BYTE2,
  DATA,
  SEARCH_END
};

static uint8_t getDataMsg[] = {0x37, 0x45, 0x11, 0x00, 0x46, 0xB0, 0x00, 0x00, 0xFE, 0xF9, 0x0D};
static void sendMessage(uint8_t *sendMsg, size_t len);
static bool recvAnswer(uint8_t *t_outMessage);
static void parseData(uint8_t *t_message);

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;

bool GobelBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool bo_lRet = true;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn = callback;
  uint8_t response[GOBELBMS_MAX_ANSWER_LEN];

#ifdef GOBEL_DEBUG
  BSC_LOGD(TAG, "Serial %i send", u8_mDevNr);
#endif
  sendMessage(getDataMsg, ARRAY_SIZE(getDataMsg));
  if (recvAnswer(response))
  {
    parseData(response);

    // mqtt
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(BT_DEVICES_COUNT + u8_mDevNr));
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(BT_DEVICES_COUNT + u8_mDevNr));
  }
  else
  {
    BSC_LOGI(TAG, "bmsData checksum wrong; Serial(%i)", u8_mDevNr);
    bo_lRet = false;
  }

  if (bo_lRet == false)
    return bo_lRet;

  return bo_lRet;
}

static void sendMessage(uint8_t *sendMsg, size_t len)
{
  callbackSetTxRxEn(u8_mDevNr, serialRxTx_TxEn);
  usleep(20);
  mPort->write(sendMsg, len);
  mPort->flush();
  callbackSetTxRxEn(u8_mDevNr, serialRxTx_RxEn);
}

static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte;
  uint16_t u16_lRecvDataLen;
  uint32_t u32_lStartTime = millis();
  SMrecvState = SEARCH_START_BYTE1;
  u16_mLastRecvBytesCnt = 0;
  u16_lRecvDataLen = 0xFFFF;
  uint16_t cksum = 0;

  for (;;)
  {
    // Timeout
    if (millis() - u32_lStartTime > 200)
    {
      BSC_LOGI(TAG, "Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u16_lRecvDataLen, u16_mLastRecvBytesCnt);
      return false;
    }

    // Überprüfen ob Zeichen verfügbar
    if (mPort->available() > 0)
    {
      u8_lRecvByte = mPort->read();
      switch (SMrecvState)
      {
      case SEARCH_START_BYTE1:
        if (u8_lRecvByte == 0x37)
        {
          SMrecvState = SEARCH_START_BYTE2;
        }
        break;

      case SEARCH_START_BYTE2:
        if (u8_lRecvByte == 0x45)
        {
          SMrecvState = DATA;
        }
        break;

      case DATA:
        cksum += u8_lRecvByte;
        p_lRecvBytes[u16_mLastRecvBytesCnt] = u8_lRecvByte;
        u16_mLastRecvBytesCnt++;
        if (u16_mLastRecvBytesCnt == 6)
        {
          SMrecvState = SEARCH_END;
          // TODO: Check length
          u16_lRecvDataLen = 6 + ((p_lRecvBytes[4] & 0x0f) << 8) + p_lRecvBytes[5] + 3;
        }
        break;

      case SEARCH_END:
        if (u16_mLastRecvBytesCnt < u16_lRecvDataLen - 3)
        {
          cksum += u8_lRecvByte;
        }
        p_lRecvBytes[u16_mLastRecvBytesCnt] = u8_lRecvByte;
        u16_mLastRecvBytesCnt++;
        break;

      default:
        break;
      }
    }

    if (u16_mLastRecvBytesCnt == u16_lRecvDataLen) break; // Recv Pakage complete
    if(u16_mLastRecvBytesCnt>=GOBELBMS_MAX_ANSWER_LEN) return false; //Answer too long!
  }

#ifdef GOBEL_DEBUG
  if (u16_mLastRecvBytesCnt > 5)
  {
    BSC_LOGD(TAG, "RecvBytes=%x, %x, %x, %x, %x, %x", u16_mLastRecvBytesCnt, p_lRecvBytes[u16_mLastRecvBytesCnt - 5], p_lRecvBytes[u16_mLastRecvBytesCnt - 4],
             p_lRecvBytes[u16_mLastRecvBytesCnt - 3], p_lRecvBytes[u16_mLastRecvBytesCnt - 2], p_lRecvBytes[u16_mLastRecvBytesCnt - 1]);
  }
#endif

  if (p_lRecvBytes[u16_mLastRecvBytesCnt - 1] != 0x0d)
    return false; // letztes Byte vor der crc muss 0x68 sein

  cksum = 1 + ~cksum;
  // Überprüfe Cheksum
  uint8_t cksumH = (cksum >> 8) & 0xFF;
  uint8_t cksumL = cksum & 0xFF;

#ifdef GOBEL_DEBUG
  BSC_LOGD(TAG, "ckhsum=%2.2x %2.2x", cksumH, cksumL);
#endif
  if (p_lRecvBytes[u16_mLastRecvBytesCnt - 3] != cksumH && p_lRecvBytes[u16_mLastRecvBytesCnt - 2] != cksumL)
    return false;

  return true;
}

uint32_t mqttSendeTimer_gobel = 0;
void parseData(uint8_t *t_message)
{
  uint16_t u16_lBalanceCapacity = 0;
  uint16_t u16_lFullCapacity = 0;
  uint16_t u16_lCycle = 0;

  uint8_t u8_lNumOfCells = 0;
  uint8_t u8_lNumOfPacks = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint16_t u16_lZellDifferenceVoltage = 0;
  uint16_t u16_lCellSum = 0;
  uint16_t u16_lZellVoltage = 0;
  uint16_t u16_lCellLow = 0xFFFF;
  uint16_t u16_lCellHigh = 0x0;
  uint8_t u8_lCntTempSensors = 0;
  float fl_lBmsTemps[3];

  parser p(t_message, u16_mLastRecvBytesCnt);

  try
  {
    p.seek(10);
    u8_lNumOfPacks = p.getuint8();

    for (uint8_t pack = 0; pack < u8_lNumOfPacks; pack++)
    {

      if (pack == 1) // TODO: Handle more packs
        break;
      p.getuint8(); // Pack addr
      setBmsTotalCurrent(BT_DEVICES_COUNT + u8_mDevNr, (float)p.getint16() * 0.01);
      setBmsTotalVoltage(BT_DEVICES_COUNT + u8_mDevNr, (float)p.getuint32() * 0.001);

      u16_lBalanceCapacity = p.getuint16();                               // Remain capacity
      p.getuint8();                                                       // 05
      u16_lFullCapacity = p.getuint16();                                  // total capacity
      p.getuint16();                                                      // design capacity
      u16_lCycle = p.getuint16();                                         // cycle number
      setBmsChargePercentage(BT_DEVICES_COUNT + u8_mDevNr, p.getuint8()); // SOC in %
      p.getuint8();                                                       // SOH
      p.getuint8();                                                       // parallel number
      p.getuint8();                                                       // slave addr
      u8_lNumOfCells = p.getuint8();
#ifdef GOBEL_DEBUG
      BSC_LOGD(TAG, "n>NOC:  %i", u8_lNumOfCells);
#endif
      for (uint8_t n = 0; n < u8_lNumOfCells; n++)
      {
        u16_lZellVoltage = (p.getuint16());
        setBmsCellVoltage(BT_DEVICES_COUNT + u8_mDevNr, n, u16_lZellVoltage);
        u16_lCellSum += u16_lZellVoltage;

        if (u16_lZellVoltage > u16_lCellHigh)
        {
          u16_lCellHigh = u16_lZellVoltage;
        }
        if (u16_lZellVoltage < u16_lCellLow)
        {
          u16_lCellLow = u16_lZellVoltage;
        }

        u16_lZellMinVoltage = u16_lCellLow;
        u16_lZellMaxVoltage = u16_lCellHigh;
        u16_lZellDifferenceVoltage = u16_lCellHigh - u16_lCellLow;

#ifdef GOBEL_DEBUG
        BSC_LOGD(TAG, "V%i=%i", n, u16_lZellVoltage);
#endif
      }

      setBmsMaxCellVoltage(BT_DEVICES_COUNT + u8_mDevNr, u16_lCellHigh);
      setBmsMinCellVoltage(BT_DEVICES_COUNT + u8_mDevNr, u16_lCellLow);
      setBmsAvgVoltage(BT_DEVICES_COUNT + u8_mDevNr, (float)(u16_lCellSum / u8_lNumOfCells));
      setBmsMaxCellDifferenceVoltage(BT_DEVICES_COUNT + u8_mDevNr, (u16_lZellDifferenceVoltage));

      u8_lCntTempSensors = p.getuint8(); // cell NTC
      for (uint8_t i = 0; i < u8_lCntTempSensors; i++)
      {
        float temp = p.gettemp();
        if (i < 3)
          setBmsTempature(BT_DEVICES_COUNT + u8_mDevNr, i, temp);
      }

      u8_lCntTempSensors = p.getuint8(); // MOS NTC
      for (uint8_t i = 0; i < u8_lCntTempSensors; i++)
      {
        float temp = p.gettemp();
        if (i < 1)
          fl_lBmsTemps[0] = temp;
      }

      u8_lCntTempSensors = p.getuint8(); // ambient NTC
      for (uint8_t i = 0; i < u8_lCntTempSensors; i++)
      {
        float temp = p.gettemp();
        if (i < 1)
          fl_lBmsTemps[1] = temp;
      }
    }

    p.getuint32(); // CRC32, TODO: clarify polynom and used input data
  }
  catch (const std::exception &e)
  {
    BSC_LOGI(TAG, "Parser Error: %s Rx%d", e.what(), u16_mLastRecvBytesCnt);
  }

  /*bmsErrors
  bit0  single cell overvoltage protection
  bit1  single cell undervoltage protection
  bit2  whole pack overvoltage protection
  bit3  Whole pack undervoltage protection
  bit4  charging over-temperature protection
  bit5  charging low temperature protection
  bit6  Discharge over temperature protection
  bit7  discharge low temperature protection
  bit8  charging overcurrent protection
  bit9  Discharge overcurrent protection
  bit10 short circuit protection
  bit11 Front-end detection IC error
  bit12 software lock MOS
  */
  //        setBmsErrors(BT_DEVICES_COUNT+u8_mDevNr, uint16_t); TODO: Handle errors

  if (millis() > (mqttSendeTimer_gobel + 10000))
  {
    // Nachrichten senden
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);

    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_TEMPERATURE, 3, fl_lBmsTemps[0]);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_TEMPERATURE, 4, fl_lBmsTemps[1]);
    mqttPublish(MQTT_TOPIC_BMS_BT, BT_DEVICES_COUNT + u8_mDevNr, MQTT_TOPIC2_TEMPERATURE, 5, fl_lBmsTemps[2]);

    mqttSendeTimer_gobel = millis();
  }
}
