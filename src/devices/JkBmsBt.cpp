// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "devices/JkBmsBt.h"
#include "BmsData.h"
#include "mqtt_t.h"

static const char *TAG = "JKBT";

static boolean bo_mStartSeyOk[BT_DEVICES_COUNT];
static uint8_t u8_mRecvFrameNr[BT_DEVICES_COUNT];
static uint8_t u8_mRecvData[BT_DEVICES_COUNT][4];

void jkBmsBtDevInit(uint8_t devNr)
{
  u8_mRecvFrameNr[devNr]=0;
  bo_mStartSeyOk[devNr]=false;
}

void jkBmsBtCopyData(uint8_t devNr, uint8_t frameVersion, uint8_t* pData, size_t length)
{
  if(length>=20)
  {
    //Start seq Cellinfo: 0x55 0xAA 0xEB 0x90 0x02
    if(pData[0]==0x55 && pData[1]==0xAA && pData[2]==0xEB && pData[3]==0x90 && pData[4]==0x02)
    {
      u8_mRecvFrameNr[devNr]=1;
      bo_mStartSeyOk[devNr]=true;
    }
    else
    {
      u8_mRecvFrameNr[devNr]++;
    }

    //Fehlerauswertung Framelänge  (128,22,128,22,20?)
    if(u8_mRecvFrameNr[devNr]==1 && length!=128){u8_mRecvFrameNr[devNr]=0; return;}
    else if(u8_mRecvFrameNr[devNr]==2 && length!=22){u8_mRecvFrameNr[devNr]=0; return;}
    else if(u8_mRecvFrameNr[devNr]==3 && length!=128){u8_mRecvFrameNr[devNr]=0; return;}
    else if(u8_mRecvFrameNr[devNr]==4 && length!=22){u8_mRecvFrameNr[devNr]=0; return;}
    else if(u8_mRecvFrameNr[devNr]==5 && length!=20){u8_mRecvFrameNr[devNr]=0; return;}

    #if 0
    //Beispiel-Daten
    //JK02
    uint8_t frameData1[128] = {0x55, 0xAA, 0xEB, 0x90, 0x02, 0x8C, 0xFF, 0x0C, 0x01, 0x0D, 0x01, 0x0D, 0xFF, 0x0C, 0x01, 0x0D, 0x01, 0x0D, 0xFF, 0x0C, 0x01, 0x0D, 0x01, 0x0D, 0x01, 0x0D, 0x01, 0x0D, 0xFF, 0x0C, 0x01, 0x0D, 0x01, 0x0D, 0x01, 0x0D, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x01, 0x96, 0x01, 0x8C, 0x01, 0x87, 0x01, 0x84, 0x01, 0x84, 0x01, 0x83, 0x01, 0x84, 0x01, 0x85, 0x01, 0x81, 0x01, 0x83, 0x01, 0x86, 0x01, 0x82, 0x01, 0x82, 0x01, 0x83, 0x01, 0x85, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xD0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t frameData2[22] = {0x00, 0x00, 0xBE, 0x00, 0xBF, 0x00, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x8E, 0x0B, 0x01, 0x00, 0x68, 0x3C, 0x01, 0x00};
    uint8_t frameData3[128] = {0x00, 0x00, 0x00, 0x00, 0x3D, 0x04, 0x00, 0x00, 0x64, 0x00, 0x79, 0x04, 0xCA, 0x03, 0x10, 0x00, 0x01, 0x01, 0xAA, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD5, 0x02, 0x00, 0x00, 0x00, 0x00, 0xAE, 0xD6, 0x3B, 0x40, 0x00, 0x00, 0x00, 0x00, 0x58, 0xAA, 0xFD, 0xFF, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0xEC, 0xE6, 0x4F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t frameData4[22] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCD};

    //JK02_32S
    uint8_t frameData1_32s[128] = {0x55, 0xAA, 0xEB, 0x90, 0x02, 0x2F, 0x8F, 0x0C, 0xD4, 0x0C, 0x9D, 0x0C, 0xE0, 0x0C, 0xE0, 0x0C, 0x8D, 0x0C, 0x8D, 0x0C, 0xDF, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xB7, 0x0C, 0x53, 0x00, 0x03, 0x05, 0x34, 0x00, 0x33, 0x00, 0x33, 0x00, 0x33, 0x00, 0x32, 0x00, 0x32, 0x00, 0x32, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t frameData2_32s[22] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x01, 0x00, 0x00, 0x00, 0x00};
    uint8_t frameData3_32s[128] = {0xBB, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x01, 0x26, 0x01, 0x10, 0x00, 0x00, 0x00, 0xFC, 0xF7, 0x02, 0x63, 0x5F, 0xEA, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0xA3, 0x08, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0xCA, 0x03, 0x00, 0x00, 0x00, 0x00, 0xA7, 0xB3, 0x40, 0x40, 0x84, 0x00, 0x00, 0x00, 0x2C, 0x0A, 0xDC, 0xE1, 0x00, 0x01, 0x00, 0x01, 0x00, 0x05, 0x00, 0x00, 0x12, 0xB2, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t frameData4_32s[22] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x7F, 0xDC, 0x1F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F};

    uint8_t* pDataBsp;

    /*if(u8_mRecvFrameNr[devNr]==1) pDataBsp=frameData1;
    else if(u8_mRecvFrameNr[devNr]==2) pDataBsp=frameData2;
    else if(u8_mRecvFrameNr[devNr]==3) pDataBsp=frameData3;
    else if(u8_mRecvFrameNr[devNr]==4) pDataBsp=frameData4;*/

    if(u8_mRecvFrameNr[devNr]==1) pDataBsp=frameData1_32s;
    else if(u8_mRecvFrameNr[devNr]==2) pDataBsp=frameData2_32s;
    else if(u8_mRecvFrameNr[devNr]==3) pDataBsp=frameData3_32s;
    else if(u8_mRecvFrameNr[devNr]==4) pDataBsp=frameData4_32s;

    #ifdef JK_BT_DEBUG
    BSC_LOGI(TAG,"DevNr=%i, Frame Nr=%i",devNr,u8_mRecvFrameNr[devNr]);

    BSC_LOGI(TAG,"Data: %i,%i,%i,%i,%i",pDataBsp[0],pDataBsp[1],pDataBsp[2],pDataBsp[3],pDataBsp[4]); //Test
    #endif

    if(u8_mRecvFrameNr[devNr]>0 && u8_mRecvFrameNr[devNr]<=5) jkBmsBtDecodeCellInfo_jk02(devNr, pDataBsp, u8_mRecvFrameNr[devNr], FRAME_VERSION_JK02_32S); //Testdaten

    #else
    if(bo_mStartSeyOk[devNr])
    {
      if(u8_mRecvFrameNr[devNr]>0 && u8_mRecvFrameNr[devNr]<=5)
      {
        jkBmsBtDecodeCellInfo_jk02(devNr, pData, u8_mRecvFrameNr[devNr], frameVersion);
      }
      if(u8_mRecvFrameNr[devNr]==4 || u8_mRecvFrameNr[devNr]==5)
      {
        setBmsLastDataMillis(devNr,millis());
        bo_mStartSeyOk[devNr]=false;
      }
    }
    #endif
  }
}


uint8_t jkBmsBtCrc(uint8_t *data, uint16_t len)
{
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    crc = crc + data[i];
  }
  return crc;
}


void jkBmsBtBuildSendFrame(uint8_t *frame, uint8_t address, uint32_t value, uint8_t length)
{
  frame[0] = 0xAA;     // start sequence
  frame[1] = 0x55;     // start sequence
  frame[2] = 0x90;     // start sequence
  frame[3] = 0xEB;     // start sequence
  frame[4] = address;
  frame[5] = length;
  frame[6] = value >> 0;
  frame[7] = value >> 8;
  frame[8] = value >> 16;
  frame[9] = value >> 24;
  frame[10] = 0x00;
  frame[11] = 0x00;
  frame[12] = 0x00;
  frame[13] = 0x00;
  frame[14] = 0x00;
  frame[15] = 0x00;
  frame[16] = 0x00;
  frame[17] = 0x00;
  frame[18] = 0x00;
  frame[19] = jkBmsBtCrc(frame, 20-1);

  //BSC_LOGD(TAG, "Write register: %s", format_hex_pretty(frame, sizeof(frame)).c_str());
}


void jkBmsSetErrors(uint8_t devNr, uint16_t err)
{
  uint32_t u32_lRawErrors=0;

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x //bit0  single cell overvoltage protection
  #define BMS_ERR_STATUS_CELL_UVP          2  x //bit1  single cell undervoltage protection
  #define BMS_ERR_STATUS_BATTERY_OVP       4  - //bit2  whole pack overvoltage protection
  #define BMS_ERR_STATUS_BATTERY_UVP       8  - //bit3  Whole pack undervoltage protection
  #define BMS_ERR_STATUS_CHG_OTP          16  x //bit4  charging over temperature protection
  #define BMS_ERR_STATUS_CHG_UTP          32  x //bit5  charging low temperature protection
  #define BMS_ERR_STATUS_DSG_OTP          64  - //bit6  Discharge over temperature protection
  #define BMS_ERR_STATUS_DSG_UTP         128  - //bit7  discharge low temperature protection
  #define BMS_ERR_STATUS_CHG_OCP         256  x //bit8  charging overcurrent protection
  #define BMS_ERR_STATUS_DSG_OCP         512  - //bit9  Discharge overcurrent protection
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  - //bit10 short circuit protection
  #define BMS_ERR_STATUS_AFE_ERROR      2048  - //bit11 Front-end detection IC error
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  - //bit12 software lock MOS
  #define BMS_ERR_STATUS_RESERVED1      8192  - //bit13 Reserved
  #define BMS_ERR_STATUS_RESERVED2     16384  - //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  - //bit15 Reserved */

  // 136   2   0x00 0x00              System alarms
  //           0x00 0x01                Charge over temperature              0000 0000 0000 0001
  if((err&0x1)==0x1) u32_lRawErrors |= BMS_ERR_STATUS_CHG_OTP;

  //           0x00 0x02                Charge under temperature             0000 0000 0000 0010
  if((err&0x2)==0x2) u32_lRawErrors |= BMS_ERR_STATUS_CHG_UTP;

  //           0x00 0x04                                                     0000 0000 0000 0100

  //           0x00 0x08                Cell Undervoltage                    0000 0000 0000 1000
  if((err&0x8)==0x8) u32_lRawErrors |= BMS_ERR_STATUS_CELL_UVP;

  //           0x00 0x10                                                     0000 0000 0001 0000
  //           0x00 0x20                                                     0000 0000 0010 0000
  //           0x00 0x40                                                     0000 0000 0100 0000
  //           0x00 0x80                                                     0000 0000 1000 0000
  //           0x01 0x00                                                     0000 0001 0000 0000
  //           0x02 0x00                                                     0000 0010 0000 0000
  //           0x04 0x00                Cell count is not equal to settings  0000 0100 0000 0000
  //           0x08 0x00                Current sensor anomaly               0000 1000 0000 0000

  //           0x10 0x00                Cell Over Voltage                    0001 0000 0000 0000
  if((err&0x1000)==0x1000) u32_lRawErrors |= BMS_ERR_STATUS_CELL_OVP;

  //           0x20 0x00                                                     0010 0000 0000 0000
  //           0x40 0x00                Charge overc urrent protection        0100 0000 0000 0000
  if((err&0x4000)==0x4000) u32_lRawErrors |= BMS_ERR_STATUS_CHG_OCP;

  //           0x80 0x00                                                     1000 0000 0000 0000
  //
  //           0x14 0x00                Cell Over Voltage +                  0001 0100 0000 0000
  //                                    Cell count is not equal to settings
  //           0x04 0x08                Cell Undervoltage +                  0000 0100 0000 1000
  //                                    Cell count is not equal to settings


  //BSC_LOGI(TAG,"devNr=%i,err=%i,u32_lRawErrors=%i",devNr,err,u32_lRawErrors);
  setBmsErrors(devNr,u32_lRawErrors);
}

void jkBmsBtDecodeCellInfo_jk02(uint8_t devNr, uint8_t* pData, uint8_t frameNr, uint8_t u8_frameVersion)
{
  auto getJk16bit = [&](size_t i) -> uint16_t { return (uint16_t(pData[i + 1]) << 8) | (uint16_t(pData[i + 0]) << 0); };
  auto getJk32bit = [&](size_t i) -> uint32_t { return (uint32_t(getJk16bit(i + 2)) << 16) | (uint32_t(getJk16bit(i + 0)) << 0); };

  uint16_t u16_lPos=0;
  uint8_t  u8_lOffset=0;
  uint8_t  u8_lFrameOffset=0;

  // Assumption: The value of data[189] (JK02) or data[189+32] (JK02_32S) is 0x01, 0x02 or 0x03
  //uint8_t u8_frameVersion = (data[189] == 0x00 && data[189 + 32] > 0) ? FRAME_VERSION_JK02_32S : FRAME_VERSION_JK02;
  if(u8_frameVersion==FRAME_VERSION_JK02_32S){u8_lOffset=16;}


  if(frameNr==1)
  {
    // Byte Len  Payload                Content              Coeff.      Unit        Example value
    // 0     2   0x55 0xAA 0xEB 0x90    Header
    // 4     1   0x02                   Record type
    // 5     1   0x8C                   Frame counter
    // 6     2   0xFF 0x0C              Voltage cell 01       0.001        V
    // 8     2   0x01 0x0D              Voltage cell 02       0.001        V
    // 10    2   0x01 0x0D              Voltage cell 03       0.001        V
    // ...
    uint8_t  u8_lCells = 24;//+(u8_lOffset/2);
    uint16_t u16_lMinCellVoltage = 0xFFFF;
    uint16_t u16_lMaxCellVoltage = 0;
    uint8_t  u8_lZellNumberMinVoltage = 0;
    uint8_t  u8_lZellNumberMaxVoltage = 0;
    for (uint8_t i = 0; i < u8_lCells; i++)
    {
      uint16_t u16_lCellVoltage = getJk16bit(i*2+6);
      if (u16_lCellVoltage > 0 && u16_lCellVoltage < u16_lMinCellVoltage)
      {
        u16_lMinCellVoltage = u16_lCellVoltage;
        u8_lZellNumberMinVoltage=i;
      }
      if (u16_lCellVoltage > u16_lMaxCellVoltage)
      {
        u16_lMaxCellVoltage = u16_lCellVoltage;
        u8_lZellNumberMaxVoltage=i;
      }
      setBmsCellVoltage(devNr,i,u16_lCellVoltage);

      float fl_lCellResistance = (float)getJk16bit(i*2+64+u8_lOffset)*0.001f;
      mqttPublish(MQTT_TOPIC_DATA_DEVICE, devNr, MQTT_TOPIC2_CELL_RESISTANCE, i, String(fl_lCellResistance,3));
    }
    setBmsMinCellVoltage(devNr,u16_lMinCellVoltage);
    setBmsMaxCellVoltage(devNr,u16_lMaxCellVoltage);
    setBmsMaxVoltageCellNumber(devNr, u8_lZellNumberMaxVoltage);
    setBmsMinVoltageCellNumber(devNr, u8_lZellNumberMinVoltage);

    #ifdef JK_BT_DEBUG
    BSC_LOGI(TAG,"DevNr=%i, CellVoltage[0]=%i",devNr,getBmsCellVoltage(devNr,0));
    #endif

    // 54    4   0xFF 0xFF 0x00 0x00    Enabled cells bitmask
    //           0x0F 0x00 0x00 0x00    4 cells enabled
    //           0xFF 0x00 0x00 0x00    8 cells enabled
    //           0xFF 0x0F 0x00 0x00    12 cells enabled
    //           0xFF 0x1F 0x00 0x00    13 cells enabled
    //           0xFF 0xFF 0x00 0x00    16 cells enabled
    //           0xFF 0xFF 0xFF 0x00    24 cells enabled
    //           0xFF 0xFF 0xFF 0xFF    32 cells enabled
    //data[54 + u8_lOffset], data[55 + u8_lOffset], data[56 + u8_lOffset], data[57 + u8_lOffset]);

    // 58    2   0x00 0x0D              Average Cell Voltage  0.001        V
    u16_lPos=58+u8_lOffset;
    setBmsAvgVoltage(devNr,getJk16bit(u16_lPos));

    // 60    2   0x00 0x00              Delta Cell Voltage    0.001        V
    u16_lPos=60+u8_lOffset;
    setBmsMaxCellDifferenceVoltage(devNr,getJk16bit(u16_lPos));

    // 62    1   0x00                   Max voltage cell      1
    // 63    1   0x00                   Min voltage cell      1

    // 64    2   0x9D 0x01              Resistance Cell 01    0.001        Ohm
    // 66    2   0x96 0x01              Resistance Cell 02    0.001        Ohm
    // 68    2   0x8C 0x01              Resistance Cell 03    0.001        Ohm
    // ...
    // 110   2   0x00 0x00              Resistance Cell 24    0.001        Ohm


  }

  u8_lOffset=u8_lOffset*2; //32

  if((frameNr==1 && u8_frameVersion==FRAME_VERSION_JK02) || (frameNr==2 && u8_frameVersion==FRAME_VERSION_JK02_32S))
  {
    u8_lFrameOffset=0;
    if(u8_frameVersion==FRAME_VERSION_JK02_32S)u8_lFrameOffset=128;

    // 112   2   0x00 0x00              Unknown112    (32S:112+32=144)
    u16_lPos=112+u8_lOffset-u8_lFrameOffset;
    if (u8_frameVersion == FRAME_VERSION_JK02_32S)
    {
      setBmsTempature(devNr,0,(float)(((int16_t)getJk16bit(u16_lPos))*0.1f));
      #ifdef JK_BT_DEBUG
      BSC_LOGI(TAG,"MOS Temp: %f°C",(float)((int16_t) getJk16bit(u16_lPos))*0.1f);
      #endif
    }

    // 114   4   0x00 0x00 0x00 0x00    Wire resistance warning bitmask (each bit indicates a warning per cell / wire)
    /*BSC_LOGD(TAG, "Wire resistance warning bitmask: %02X %02X %02X %02X", data[114 + u8_lOffset], data[115 + u8_lOffset],
            data[116 + u8_lOffset], data[117 + u8_lOffset]);*/
  }
  if((frameNr==1 && u8_frameVersion==FRAME_VERSION_JK02) || (frameNr==3 && u8_frameVersion==FRAME_VERSION_JK02_32S))
  {
    u8_lFrameOffset=0;
    if(u8_frameVersion==FRAME_VERSION_JK02_32S)u8_lFrameOffset=118+u8_lOffset;

    // 118   4   0x03 0xD0 0x00 0x00    Battery voltage       0.001        V    (32S: Byte 150)
    u16_lPos=118+u8_lOffset-u8_lFrameOffset;
    float fl_lTotalVoltage = (float)getJk32bit(u16_lPos)*0.001f;
    setBmsTotalVoltage(devNr,fl_lTotalVoltage);

    // 122   4   0x00 0x00 0x00 0x00    Battery power         0.001        W

    if(u8_frameVersion==FRAME_VERSION_JK02)
    {
      u8_mRecvData[devNr][0] = pData[126];
      u8_mRecvData[devNr][1] = pData[127];
    }
  }
  if(frameNr==2 && u8_frameVersion==FRAME_VERSION_JK02 || (frameNr==3 && u8_frameVersion==FRAME_VERSION_JK02_32S))
  {
    u8_lFrameOffset=128;
    if(u8_frameVersion==FRAME_VERSION_JK02_32S)u8_lFrameOffset=150;

    // 126   4   0x00 0x00 0x00 0x00    Charge current        0.001        A    (32S: Byte 158)
    u16_lPos=126+u8_lOffset-u8_lFrameOffset;
    float fl_lCurrent;
    if(u8_frameVersion==FRAME_VERSION_JK02)
    {
      fl_lCurrent = (((int)pData[1] << 24 | pData[0] << 16 | u8_mRecvData[devNr][1] << 8 | u8_mRecvData[devNr][0]) * 0.001);
      //BSC_LOGI(TAG,"C:%i,%i,%i,%i",u8_mRecvData[devNr][0],u8_mRecvData[devNr][1],pData[0],pData[1]);
    }
    else if(u8_frameVersion==FRAME_VERSION_JK02_32S)
    {
      fl_lCurrent = (float)((int32_t) getJk32bit(u16_lPos)) * 0.001f;
      //BSC_LOGI(TAG,"C32:%i,%i,%i,%i",pData[0],pData[1],pData[2],pData[3]);
    }
    setBmsTotalCurrent(devNr,fl_lCurrent);

    // Don't use byte 122 because it's unsigned
    // float power = (float) ((int32_t) getJk32bit(122 + u8_lOffset)) * 0.001f;
    //float power = fl_lTotalVoltage * current;
    //this->publish_state_(this->power_sensor_, power);
    //this->publish_state_(this->charging_power_sensor_, std::max(0.0f, power));               // 500W vs 0W -> 500W
    //this->publish_state_(this->discharging_power_sensor_, std::abs(std::min(0.0f, power)));  // -500W vs 0W -> 500W

    // 130   2   0xBE 0x00              Temperature Sensor 1  0.1          °C
    u16_lPos=130+u8_lOffset-u8_lFrameOffset;
    setBmsTempature(devNr,1,(float)(((int16_t)getJk16bit(u16_lPos))*0.1f));

    // 132   2   0xBF 0x00              Temperature Sensor 2  0.1          °C
    u16_lPos=132+u8_lOffset-u8_lFrameOffset;
    setBmsTempature(devNr,2,(float)(((int16_t)getJk16bit(u16_lPos))*0.1f));

    // 134   2   0xD2 0x00              MOS Temperature       0.1          °C
    u16_lPos=134+u8_lOffset-u8_lFrameOffset;
    if (u8_frameVersion == FRAME_VERSION_JK02_32S)
    {
      uint32_t u32_lRawErrorsBitmask = (uint32_t)((uint16_t(pData[u16_lPos]) << 8) | (uint16_t(pData[u16_lPos+1]) << 0));
      jkBmsSetErrors(devNr, u32_lRawErrorsBitmask);
    }
    else
    {
      setBmsTempature(devNr,0,(float)(((int16_t)getJk16bit(u16_lPos))*0.1f));
      #ifdef JK_BT_DEBUG
      BSC_LOGI(TAG,"MOS Temp: %f°C",(float)((int16_t) getJk16bit(u16_lPos))*0.1f);
      #endif
    }

    // 136   2   0x00 0x00              System alarms
    //           0x00 0x01                Charge overtemperature               0000 0000 0000 0001
    //           0x00 0x02                Charge undertemperature              0000 0000 0000 0010
    //           0x00 0x04                                                     0000 0000 0000 0100
    //           0x00 0x08                Cell Undervoltage                    0000 0000 0000 1000
    //           0x00 0x10                                                     0000 0000 0001 0000
    //           0x00 0x20                                                     0000 0000 0010 0000
    //           0x00 0x40                                                     0000 0000 0100 0000
    //           0x00 0x80                                                     0000 0000 1000 0000
    //           0x01 0x00                                                     0000 0001 0000 0000
    //           0x02 0x00                                                     0000 0010 0000 0000
    //           0x04 0x00                Cell count is not equal to settings  0000 0100 0000 0000
    //           0x08 0x00                Current sensor anomaly               0000 1000 0000 0000
    //           0x10 0x00                Cell Over Voltage                    0001 0000 0000 0000
    //           0x20 0x00                                                     0010 0000 0000 0000
    //           0x40 0x00                Charge overcurrent protection        0100 0000 0000 0000
    //           0x80 0x00                                                     1000 0000 0000 0000
    //
    //           0x14 0x00                Cell Over Voltage +                  0001 0100 0000 0000
    //                                    Cell count is not equal to settings
    //           0x04 0x08                Cell Undervoltage +                  0000 0100 0000 1000
    //                                    Cell count is not equal to settings

    u16_lPos=136+u8_lOffset-u8_lFrameOffset;
    if (u8_frameVersion != FRAME_VERSION_JK02_32S)
    {
      uint32_t u32_lRawErrorsBitmask = (uint16_t(pData[u16_lPos]) << 8) | (uint16_t(pData[u16_lPos+1]) << 0);
      jkBmsSetErrors(devNr,u32_lRawErrorsBitmask);
    }

    // 138   2   0x00 0x00              Balance current      0.001         A
    u16_lPos=138+u8_lOffset-u8_lFrameOffset;
    setBmsBalancingCurrent(devNr,(float)((int16_t) getJk16bit(u16_lPos)) * 0.001f);

    // 140   1   0x00                   Balancing action                   0x00: Off
    //                                                                     0x01: Charging balancer
    //                                                                     0x02: Discharging balancer
    u16_lPos=140+u8_lOffset-u8_lFrameOffset;
    boolean bo_lBalancingAction = (bool)(pData[u16_lPos] != 0x00);
    setBmsIsBalancingActive(devNr,(uint8_t)bo_lBalancingAction);

    // 141   1   0x54                   State of charge in   1.0           %
    u16_lPos=141+u8_lOffset-u8_lFrameOffset;
    setBmsChargePercentage(devNr,pData[u16_lPos]);

    // 142   4   0x8E 0x0B 0x01 0x00    Capacity_Remain      0.001         Ah
    u16_lPos=142+u8_lOffset-u8_lFrameOffset;
    //(float) getJk32bit(u16_lPos) * 0.001f;
    //ToDo: only mqtt

    // 146   4   0x68 0x3C 0x01 0x00    Nominal_Capacity     0.001         Ah
    u16_lPos=146+u8_lOffset-u8_lFrameOffset;
    //(float) getJk32bit(u16_lPos) * 0.001f;
    //ToDo: only mqtt

  }
  if(frameNr==3 && u8_frameVersion==FRAME_VERSION_JK02 || (frameNr==3 && u8_frameVersion==FRAME_VERSION_JK02_32S))  //278
  {
    if(u8_frameVersion==FRAME_VERSION_JK02)u8_lFrameOffset=150;
    //if(u8_frameVersion==FRAME_VERSION_JK02_32S)u8_lFrameOffset=150;

    // 150   4   0x00 0x00 0x00 0x00    Cycle_Count          1.0
    u16_lPos=150+u8_lOffset-u8_lFrameOffset;
    //(float) getJk32bit(u16_lPos);
    //ToDo: only mqtt


    // 154   4   0x3D 0x04 0x00 0x00    Cycle_Capacity       0.001         Ah
    u16_lPos=154+u8_lOffset-u8_lFrameOffset;
    //(float) getJk32bit(u16_lPos) * 0.001f;
    //ToDo: only mqtt

    // 158   2   0x64 0x00              Unknown158
    // 160   2   0x79 0x04              Unknown160 (Cycle capacity?)

    // 162   4   0xCA 0x03 0x10 0x00    Total runtime in seconds           s
    u16_lPos=162+u8_lOffset-u8_lFrameOffset;
    //(float) getJk32bit(u16_lPos);
    //format_total_runtime_(getJk32bit(u16_lPos));
    //ToDo: only mqtt

    // 166   1   0x01                   Charging switch enabled                      0x00: off, 0x01: on
    u16_lPos=166+u8_lOffset-u8_lFrameOffset;
    setBmsStateFETsCharge(devNr,(bool)pData[u16_lPos]);

    // 167   1   0x01                   Discharging switch enabled                   0x00: off, 0x01: on
    u16_lPos=167+u8_lOffset-u8_lFrameOffset;
    setBmsStateFETsDischarge(devNr,(bool)pData[u16_lPos]);

  }

  // 168   1   0xAA                   Unknown168
  // 169   2   0x06 0x00              Unknown169
  // 171   2   0x00 0x00              Unknown171
  // 173   2   0x00 0x00              Unknown173
  // 175   2   0x00 0x00              Unknown175
  // 177   2   0x00 0x00              Unknown177
  // 179   2   0x00 0x00              Unknown179
  // 181   2   0x00 0x07              Unknown181
  // 183   2   0x00 0x01              Unknown183
  // 185   2   0x00 0x00              Unknown185
  // 187   2   0x00 0xD5              Unknown187
  // 189   2   0x02 0x00              Unknown189
  // 190   1   0x00                   Unknown190
  // 191   1   0x00                   Balancer status (working: 0x01, idle: 0x00)
  // 192   1   0x00                   Unknown192
  // 193   2   0x00 0xAE              Unknown193
  // 195   2   0xD6 0x3B              Unknown195
  // 197   10  0x40 0x00 0x00 0x00 0x00 0x58 0xAA 0xFD 0xFF 0x00
  // 207   7   0x00 0x00 0x01 0x00 0x02 0x00 0x00
  // 214   4   0xEC 0xE6 0x4F 0x00    Uptime 100ms
  //
  // 218   81  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00
  // 299   1   0xCD                   CRC

}
