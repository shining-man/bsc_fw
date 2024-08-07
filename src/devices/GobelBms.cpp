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

static Stream *mPort;
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

static uint8_t u8_mDevNr, u8_mConnToId, u8_mCountOfPacks;
static uint16_t u16_mLastRecvBytesCnt;

enum SM_readData
{
  SEARCH_START_BYTE1,
  SEARCH_START_BYTE2,
  DATA,
  SEARCH_END
};

static uint8_t getDataMsg[] = {0x11, 0x01, 0x46, 0xB0, 0x00, 0x00};
static uint8_t getWarnMsg[] = {0x11, 0x01, 0x46, 0xB1, 0x00, 0x00};
static void sendMessage(uint8_t *sendMsg, size_t len);
static bool recvAnswer(uint8_t *t_outMessage);
static void parseData(uint8_t *t_message, uint8_t dataMappingNr);
static void parseWarnData(uint8_t *t_message, uint8_t dataMappingNr);

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;
static serialDevData_s *mDevData;

bool GobelBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool bo_lRet = true;
  mDevData=devData;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn = callback;
  uint8_t u8_addr = 0;
  uint8_t i;
  uint8_t response[GOBELBMS_MAX_ANSWER_LEN];

  uint8_t u8_packAdr = devData->bmsAdresse;
  uint8_t dataMappingNr = devData->dataMappingNr;


  #ifdef GOBEL_DEBUG
  BSC_LOGD(TAG, "Serial %i send, addr: %i", u8_mDevNr, u8_packAdr);
  #endif

  getDataMsg[1] = u8_packAdr;
  sendMessage(getDataMsg, ARRAY_SIZE(getDataMsg));
  if (recvAnswer(response))
  {
    parseData(response, dataMappingNr);

    // mqtt
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(dataMappingNr));
    mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(dataMappingNr));

    getWarnMsg[1] = u8_packAdr;
    sendMessage(getWarnMsg, ARRAY_SIZE(getWarnMsg));
    if (recvAnswer(response))
    {
      parseWarnData(response, dataMappingNr);
    }

    setBmsLastDataMillis(dataMappingNr, millis());
  }
  else
  {
    BSC_LOGD(TAG, "bmsData checksum wrong; Serial(%i)", u8_mDevNr);
    bo_lRet = false;
  }

  if (bo_lRet == false) return bo_lRet;

  vTaskDelay(pdMS_TO_TICKS(25));
  

  if (devNr >= 2) callbackSetTxRxEn(u8_mDevNr, serialRxTx_RxTxDisable);
  return bo_lRet;
}

static void sendMessage(uint8_t *sendMsg, size_t len)
{
  uint16_t chksum = 0;
  size_t i = 0;
  callbackSetTxRxEn(u8_mDevNr, serialRxTx_TxEn);
  usleep(20);
  mPort->write(0x37);
  mPort->write(0x45);
  mPort->write(sendMsg, len);
  for (i = 0; i < len; i++)
    chksum += sendMsg[i];

  chksum = 1 + ~chksum;
  mPort->write(chksum >> 8);
  mPort->write(chksum & 0xff);

  mPort->write(0x0d);
  mPort->flush();
  callbackSetTxRxEn(u8_mDevNr, serialRxTx_RxEn);
}

static bool recvAnswer(uint8_t *p_lRecvBytes)
{
  uint8_t SMrecvState, u8_lRecvByte, u8_CyclesWithoutData;
  uint16_t u16_lRecvDataLen;
  uint32_t u32_lStartTime = millis();
  SMrecvState = SEARCH_START_BYTE1;
  u16_mLastRecvBytesCnt = 0;
  u16_lRecvDataLen = 0xFFFF;
  uint16_t cksum = 0;
  u8_CyclesWithoutData = 0;

  for (;;)
  {
    // Timeout
    //  wenn innerhalb von 500ms das Telegram noch nicht begonnen hat, dann Timeout
    //  oder wenn es begonnen hat, dann 700ms
    if (((millis() - u32_lStartTime) > 500 && u16_mLastRecvBytesCnt == 0) || ((millis() - u32_lStartTime) > 700 && u16_mLastRecvBytesCnt > 0))
    {
      BSC_LOGD(TAG, "Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u16_lRecvDataLen, u16_mLastRecvBytesCnt);
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
      u8_CyclesWithoutData = 0;
    }
    else if (u16_mLastRecvBytesCnt == 0)
      vTaskDelay(pdMS_TO_TICKS(10)); // Wenn noch keine Daten empfangen wurden, dann setze den Task 10ms aus
    else if (u16_mLastRecvBytesCnt > 0 && u8_CyclesWithoutData > 10)
      vTaskDelay(pdMS_TO_TICKS(10)); // Wenn trotz empfangenen Daten 10ms wieder nichts empfangen wurde, dann setze den Task 10ms aus
    else                             // Wenn in diesem Zyklus keine Daten Empfangen wurde, dann setze den Task 1ms aus
    {
      u8_CyclesWithoutData++;
      vTaskDelay(pdMS_TO_TICKS(1));
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

void parseData(uint8_t *t_message, uint8_t dataMappingNr)
{
  uint16_t u16_lBalanceCapacity = 0;
  uint16_t u16_lFullCapacity = 0;
  uint16_t u16_lCycle = 0;

  uint8_t u8_lNumOfCells = 0;
  uint8_t u8_lNumOfPacks = 0;
  uint16_t u16_lZellMinVoltage = 0;
  uint16_t u16_lZellMaxVoltage = 0;
  uint8_t  u8_lZellNumberMinVoltage = 0;
  uint8_t  u8_lZellNumberMaxVoltage = 0;
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

    if (u8_lNumOfPacks != 0x01)
      return;
    for (uint8_t pack = 0; pack < u8_lNumOfPacks; pack++)
    {

      if (pack == 1) // TODO: Handle more packs
        break;
      p.getuint8(); // Pack addr
      setBmsTotalCurrent(dataMappingNr, (float)p.getint16() * 0.01);
      setBmsTotalVoltage(dataMappingNr, (float)p.getuint32() * 0.001);

      u16_lBalanceCapacity = p.getuint16();                                         // Remain capacity
      p.getuint8();                                                                 // 05
      u16_lFullCapacity = p.getuint16();                                            // total capacity
      p.getuint16();                                                                // design capacity
      u16_lCycle = p.getuint16();                                                   // cycle number
      setBmsChargePercentage(dataMappingNr, p.getuint8()); // SOC in %
      p.getuint8();                                                                 // SOH
      p.getuint8();                                                                 // parallel number
      p.getuint8();                                                                 // slave addr
      u8_lNumOfCells = p.getuint8();
#ifdef GOBEL_DEBUG
      BSC_LOGD(TAG, "n>NOC:  %i", u8_lNumOfCells);
#endif
      for (uint8_t n = 0; n < u8_lNumOfCells; n++)
      {
        u16_lZellVoltage = (p.getuint16());
        setBmsCellVoltage(dataMappingNr, n, u16_lZellVoltage);
        u16_lCellSum += u16_lZellVoltage;

        if (u16_lZellVoltage > u16_lCellHigh)
        {
          u16_lCellHigh = u16_lZellVoltage;
          u8_lZellNumberMaxVoltage=n;
        }
        if (u16_lZellVoltage < u16_lCellLow)
        {
          u16_lCellLow = u16_lZellVoltage;
          u8_lZellNumberMinVoltage=n;
        }

        u16_lZellMinVoltage = u16_lCellLow;
        u16_lZellMaxVoltage = u16_lCellHigh;
        u16_lZellDifferenceVoltage = u16_lCellHigh - u16_lCellLow;

#ifdef GOBEL_DEBUG
        BSC_LOGD(TAG, "V%i=%i", n, u16_lZellVoltage);
#endif
      }

      setBmsMaxCellVoltage(dataMappingNr, u16_lCellHigh);
      setBmsMinCellVoltage(dataMappingNr, u16_lCellLow);
      setBmsMaxVoltageCellNumber(dataMappingNr, u8_lZellNumberMaxVoltage);
      setBmsMinVoltageCellNumber(dataMappingNr, u8_lZellNumberMinVoltage);
      setBmsAvgVoltage(dataMappingNr, (float)(u16_lCellSum / u8_lNumOfCells));
      setBmsMaxCellDifferenceVoltage(dataMappingNr, (u16_lZellDifferenceVoltage));

      u8_lCntTempSensors = p.getuint8(); // cell NTC
      for (uint8_t i = 0; i < u8_lCntTempSensors; i++)
      {
        float temp = p.gettemp();
        if (i < 3)
          setBmsTempature(dataMappingNr, i, temp);
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

    p.getuint32(); // CRC32, according to Gobel there is a problem. Ignore for now.

    if (mDevData->bo_sendMqttMsg)
    {
      // Nachrichten senden
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);

      mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TEMPERATURE, 3, fl_lBmsTemps[0]);
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TEMPERATURE, 4, fl_lBmsTemps[1]);
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TEMPERATURE, 5, fl_lBmsTemps[2]);
    }
  }
  catch (const std::exception &e)
  {
    BSC_LOGI(TAG, "Parser Error: %s Rx%d", e.what(), u16_mLastRecvBytesCnt);
  }
}

void parseWarnData(uint8_t *t_message, uint8_t dataMappingNr)
{

  uint8_t u8_lNumOfCells = 0;
  uint8_t u8_lNumOfPacks = 0;
  uint32_t u32_alarm = 0;
  parser p(t_message, u16_mLastRecvBytesCnt);

  try
  {
    uint8_t val;
    p.seek(10);
    u8_lNumOfPacks = p.getuint8();

    if (u8_lNumOfPacks != 0x01)
      return;

    for (uint8_t pack = 0; pack < u8_lNumOfPacks; pack++)
    {

      if (pack == 1) // TODO: Handle more packs
        break;
      p.getuint8(); // Pack addr

      u8_lNumOfCells = p.getuint8();
#ifdef GOBEL_DEBUG
      BSC_LOGD(TAG, "n>NOC:  %i", u8_lNumOfCells);
#endif
      for (uint8_t n = 0; n < u8_lNumOfCells; n++)
      {
        uint8_t cellwarning = p.getuint8();
        if (cellwarning == 0x01)
          u32_alarm |= BMS_ERR_STATUS_CELL_UVP;
        else if (cellwarning == 0x02)
          u32_alarm |= BMS_ERR_STATUS_CELL_OVP;
      }

      uint8_t u8_lNumOfNTC = p.getuint8();
      for (uint8_t n = 0; n < u8_lNumOfNTC; n++)
      {
        uint8_t ntcwarning = p.getuint8();
        if (ntcwarning == 0x01)
          u32_alarm |= (BMS_ERR_STATUS_CHG_UTP | BMS_ERR_STATUS_DSG_UTP);
        else if (ntcwarning == 0x02)
          u32_alarm |= (BMS_ERR_STATUS_CHG_OTP | BMS_ERR_STATUS_DSG_OTP);
      }

      uint8_t u8_lNumOfAmbietnNTC = p.getuint8();
      for (uint8_t n = 0; n < u8_lNumOfAmbietnNTC; n++)
      {
        uint8_t ntcwarning = p.getuint8();
        if (ntcwarning == 0x01)
          u32_alarm |= (BMS_ERR_STATUS_CHG_UTP | BMS_ERR_STATUS_DSG_UTP);
        else if (ntcwarning == 0x02)
          u32_alarm |= (BMS_ERR_STATUS_CHG_OTP | BMS_ERR_STATUS_DSG_OTP);
      }

      uint8_t u8_lNumOfMOSNTC = p.getuint8();
      for (uint8_t n = 0; n < u8_lNumOfMOSNTC; n++)
      {
        uint8_t ntcwarning = p.getuint8();
        if (ntcwarning == 0x01)
          u32_alarm |= (BMS_ERR_STATUS_CHG_UTP | BMS_ERR_STATUS_DSG_UTP);
        else if (ntcwarning == 0x02)
          u32_alarm |= (BMS_ERR_STATUS_CHG_OTP | BMS_ERR_STATUS_DSG_OTP);
      }

      val = p.getuint8(); // Charge current warning
      if (val == 0x02)
        u32_alarm |= BMS_ERR_STATUS_CHG_OCP;

      val = p.getuint8(); // Pack voltage warning
      if (val == 0x01)
        u32_alarm |= BMS_ERR_STATUS_BATTERY_UVP;
      else if (val == 0x02)
        u32_alarm |= BMS_ERR_STATUS_BATTERY_OVP;

      val = p.getuint8(); // Discharge current warning
      if (val == 0x02)
        u32_alarm |= BMS_ERR_STATUS_DSG_OTP;

      p.getuint32(); // Protection state code
      p.getuint32(); // Function control code
      p.getuint32(); // Working state code
      p.getuint32(); // Fault state code
      p.getuint32(); // Warning state code
      p.getuint32(); // Cells balance state code
      p.getuint32(); // Cells balance state code
      p.getuint16(); // BMS&Inverter State code
      p.getuint16(); // Max Charge current
      p.getuint16(); // Max Discharge current
      p.getuint32(); // CRC32, according to Gobel there is a problem. Ignore for now.

      setBmsErrors(dataMappingNr, u32_alarm);
    }
  }
  catch (const std::exception &e)
  {
    BSC_LOGI(TAG, "Parser Error: %s Rx%d", e.what(), u16_mLastRecvBytesCnt);
    //    ESP_LOG_BUFFER_HEXDUMP(TAG, t_message, u16_mLastRecvBytesCnt, BSC_LOGI);
  }
}
