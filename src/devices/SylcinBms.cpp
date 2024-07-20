// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/SylcinBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"

static const char *TAG = "SYLCIN_BMS";

static Stream *mPort;
static uint8_t u8_mTxEnRS485pin, u8_mCountOfPacks, u8_mDevNr;

enum SM_readData {SEARCH_START, SEARCH_END};

//
static float    f_mTotalVoltageOld=0xFFFF;
static uint16_t u16_mBalanceCapacityOld=0xFFFF;
static uint32_t u32_mChargeMAh=0;
static uint32_t u32_mDischargeMAh=0;

//
static void      getDataFromBms(uint8_t address, uint8_t function);
static bool      recvAnswer(uint8_t * t_outMessage);
static bool      parseMessage(uint8_t * t_message, uint8_t address);
static bool      parseMessage_Alarms(uint8_t * t_message, uint8_t address);

uint8_t         sylcinconvertAsciiHexToByte(char a, char b);
static char     sylcinconvertByteToAsciiHex(uint8_t v);
void            sylcinconvertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length);
uint16_t        sylcinlCrc(const uint16_t len);
static bool     checkCrc(uint8_t *recvMsg, uint8_t u8_lRecvBytesCnt);
static uint16_t calcCrc(uint8_t *data, const uint16_t i16_lLen);

static void (*callbackSetTxRxEn)(uint8_t, uint8_t) = NULL;
static serialDevData_s *mDevData;


bool SylcinBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  bool ret = true;
  mDevData = devData;
  mPort = port;
  u8_mDevNr = devNr;
  callbackSetTxRxEn=callback;
  uint8_t response[SYLCINBMS_MAX_ANSWER_LEN];

  // Hinweis: Beim Sylcin BMS fängt die erste Adresse immer mit 1 an!
  uint8_t u8_lSylcinAdr = devData->bmsAdresse;
  uint8_t u8_lSylcinAdrBmsData = devData->dataMappingNr;

  #ifdef SYLCIN_DEBUG
  BSC_LOGI(TAG,"SylcinBms_readBmsData() devNr=%i, readFromAdr=%i, BmsDataAdr=%i, CountOfPacks=%i, Packs=%i",u8_mDevNr,u8_lSylcinAdr,u8_lSylcinAdrBmsData,u8_mCountOfPacks,devData->u8_addData);
  #endif

  // Kleine Pause zwischen den Packs
  if(u8_lSylcinAdr>1)vTaskDelay(pdMS_TO_TICKS(50));

  getDataFromBms(u8_lSylcinAdr, 0x42);
  if(recvAnswer(response))
  {
    if(parseMessage(response, u8_lSylcinAdrBmsData))
    {
      //mqtt
      mqttPublish(MQTT_TOPIC_BMS_BT, u8_lSylcinAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(u8_lSylcinAdrBmsData));
      mqttPublish(MQTT_TOPIC_BMS_BT, u8_lSylcinAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(u8_lSylcinAdrBmsData));
    }
    else ret = false;
  }
  else ret = false;

  if(ret == true)
  {
    getDataFromBms(u8_lSylcinAdr, 0x44); //Alarms
    if(recvAnswer(response))
    {
      if(!parseMessage_Alarms(response, u8_lSylcinAdrBmsData)) ret = false;
    }
    else ret = false;
  }

  if(u8_mDevNr>=2) callbackSetTxRxEn(u8_mDevNr,serialRxTx_RxTxDisable);
  return ret;
}

static void getDataFromBms(uint8_t address, uint8_t function)
{
  /* Beispieldaten
   * ->: 7E 35 32 30 31 34 36 34 32 45 30 30 32 30 31 46 44 33 30 0D
   * <-: 7E 32 30 30 30 34 36 30 30 31 30 39 36 30 30 30 31 31 30 30 43 43 30 30 43 43 33 30 43 43 32 30 43 42 46 30 43 43 33 30 43 43 30 30 43 43 30 30 43 43 31 30 43 43 31 30 43 43 30 30 43 43 32 30 43 43 33 30 43 43 37 30 43 43 35 30 43 43 35 30 43 43 36 30 36 30 42 36 46 30 42 37 32 30 42 37 32 30 42 37 31 30 42 39 36 30 42 37 43 46 44 37 46 31 34 36 41 32 38 33 45 30 41 34 45 32 30 30 32 30 33 34 45 32 30 30 30 31 35 30 33 45 38 31 34 36 43 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 44 44 30 31 0D
   */

  uint16_t lenid = sylcinlCrc(2);

  const uint16_t frame_len = 7;
  uint8_t u8_lData[9];
  uint8_t u8_lSendData[20];

  u8_lData[0]=0x52;           // VER (0x52)
  u8_lData[1]=address;        // ADDR (0x01)
  u8_lData[2]=0x46;           // CID1 (0x46)
  u8_lData[3]=function;       // CID2 (0x42)
  u8_lData[4]=(lenid >> 8);   // LCHKSUM (0xE0)
  u8_lData[5]=(lenid >> 0);   // LENGTH (0x02)
  u8_lData[6]=address;        // VALUE (0x01)

  sylcinconvertByteToAsciiHex(&u8_lSendData[1], &u8_lData[0], frame_len);

  uint16_t crc = calcCrc(&u8_lSendData[1], frame_len*2);
  #ifdef SYLCIN_DEBUG
  BSC_LOGD(TAG,"crc=%i", crc);
  #endif
  u8_lData[7]=(crc >> 8);  // CHKSUM (0xFD)
  u8_lData[8]=(crc >> 0);  // CHKSUM (0x30)
  sylcinconvertByteToAsciiHex(&u8_lSendData[15], &u8_lData[7], 2);

  u8_lSendData[0]=0x7E;   // SOF (0x7E)
  u8_lSendData[19]=0x0D;  // EOF (0x0D)

  #ifdef SYLCIN_DEBUG
  String recvBytes="";
  for(uint8_t x=0;x<9;x++)
  {
    recvBytes+="0x";
    recvBytes+=String(u8_lData[x],16);
    recvBytes+=" ";
  }
  BSC_LOGD(TAG,"sendBytes: %s", recvBytes.c_str());
  //log_print_buf(u8_lSendData, 20);
  #endif


  //TX
  callbackSetTxRxEn(u8_mDevNr,serialRxTx_TxEn);
  usleep(20);
  mPort->write(u8_lSendData, 20);
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
    // oder wenn es begonnen hat, dann 700ms
    if( ((millis()-u32_lStartTime)>500 && u8_lRecvBytesCnt==0) || ((millis()-u32_lStartTime)>700 && u8_lRecvBytesCnt>0))
    {
        BSC_LOGE(TAG,"Timeout: Serial=%i, u8_lRecvDataLen=%i, u8_lRecvBytesCnt=%i", u8_mDevNr, u8_lRecvDataLen, u8_lRecvBytesCnt);
        #ifdef SYLCIN_DEBUG
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
            BSC_LOGE(TAG,"Timeout: RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
            recvBytes="";
            u8_logByteCount=0;
          }
        }
        BSC_LOGE(TAG,"Timeout: RecvBytes=%i: %s",u8_lRecvBytesCnt, recvBytes.c_str());
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
    if(u8_lRecvBytesCnt>=SYLCINBMS_MAX_ANSWER_LEN) return false; //Answer too long!
  }

  #ifdef SYLCIN_DEBUG
  String recvBytes="";
  uint8_t u8_logByteCount=0;
  if(u8_lRecvBytesCnt==108)
  {
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
    //log_print_buf(p_lRecvBytes, u8_lRecvBytesCnt);
  }
  #endif


  //Überprüfe Cheksum
  if(!checkCrc(p_lRecvBytes,u8_lRecvBytesCnt)) return false;

  return true;
}


static bool parseMessage(uint8_t * t_message, uint8_t address)
{
  //lambda get16bitFromMsg(i)
	auto get16bitFromMsg = [&](size_t i) -> uint16_t {
		return (uint16_t(sylcinconvertAsciiHexToByte(t_message[i * 2], t_message[(i * 2) + 1])) << 8) |
           (uint16_t(sylcinconvertAsciiHexToByte(t_message[(i * 2)+2], t_message[(i * 2) + 3])) << 0);
	};

  #ifdef SYLCIN_DEBUG
  BSC_LOGI(TAG, "parseMessage: serialDev=%i",address);
  #endif

  uint8_t u8_lNumOfCells = 0;
  uint8_t u8_lDatalength = 0;
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

  // 35 32 30 31 34 36 30 30 34 30 38 34 30 31 31	30 30 44 31 41 30 44 31 44 30 44 31 38 30 44 31 38 30 44 31 38 30 44 31 45 30 44 31 41 30 44 31 41 30 44 31 37 30 44 31 41 30 44 31 38 30 44 31 44 30 44 32 37 30 44 31 41 30 44 31 41 30 44 31 39 30 36 30 30 33 46 30 30 34 30 30 30 34 30 30 30 34 30 30 30 33 45 30 30 33 45 30 41 42 33 31 34 46 37 33 35 39 39 30 36 34 45 32 30 30 30 31 32 34 34 36 34 30 30 30 30 30 30 30 30 30 30 30 30 45 31 42 45
  //
  // Nibble   Address Content: Description                      Decoded content               Coeff./Unit
  //   0,1    0x52             Protocol version      VER      5.2
  //   2,3    0x01             Device address        ADR      Adresse 1
  //   4,5    0x46             Device type           CID1     Lithium iron phosphate battery BMS
  //   6,7    0x00             Function code         CID2     0x00: Normal, 0x01 VER error, 0x02 Chksum error, ...
  //   8      0x4              Data length checksum  LCHKSUM
  //   9-11   0x084            Data length           LENID     132 / 2 = 66
  //   12,13  0x01             Batterie Nummer                Adresse 1
  //   14,15  0x10             Number of cells                16
  //   16,17,18,19  0x0D1A     Cell voltage 1                 3354 * 0.001f = 3.354         V
  //   20,21,22,23  0x0D1D     Cell voltage 2                 3357 * 0.001f = 3.357         V
  //   ...    ...            ...
  //   76,77,78,79  0x0D19     Cell voltage 16                3353 * 0.001f = 3.353         V


  // Kontrolle ob Function Code OK
  if (sylcinconvertAsciiHexToByte(t_message[6], t_message[7])!=0x00)
  {
    BSC_LOGE(TAG, "Function Code nicht OK: 0x%02x", sylcinconvertAsciiHexToByte(t_message[6], t_message[7]));
    return false;
  }

  // Kontrolle Datenpaketlänge
  u8_lDatalength = sylcinconvertAsciiHexToByte(t_message[10], t_message[11]);
  if(u8_lDatalength != 132)
  {
    BSC_LOGE(TAG, "Falsche Datenpaketlaenge (132 erwartet): %d byte lang", sylcinconvertAsciiHexToByte(t_message[10], t_message[11]));
    return false;
  }



	u8_lNumOfCells = sylcinconvertAsciiHexToByte(t_message[14], t_message[14+1]);  //Number of cells 0-16
  #ifdef SYLCIN_DEBUG
  BSC_LOGD(TAG, "Number of cells: %d", u8_lNumOfCells);
  #endif

  for (uint8_t i=0; i<u8_lNumOfCells; i++)
  {
    u16_lZellVoltage = get16bitFromMsg(8+(i*2));
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


  u8_lMsgoffset = 8+(u8_lNumOfCells*2);

  //   80+81     0x06           Number of temperatures           6                                    V
  uint8_t u8_lCntTempSensors = sylcinconvertAsciiHexToByte(t_message[u8_lMsgoffset*2], t_message[u8_lMsgoffset*2+1]);
  #ifdef SYLCIN_DEBUG
  BSC_LOGD(TAG, "Number of temperature sensors: %d", u8_lCntTempSensors);
  #endif

  //   82,83,84,85     0x00 0x3F      Temperature sensor 1             63 - 40 = 23          °C
  //   86,87,88,89     0x00 0x40      Temperature sensor 2             64 - 40 = 24          °C
  //   90,91,92,93     0x00 0x40      Temperature sensor 3             64 - 40 = 24          °C
  //   94,95,96,97     0x00 0x40      Temperature sensor 4             64 - 40 = 24          °C
  //   98,99,100,101   0x00 0x3E      Temperature sensor 5             62 - 40 = 22          °C
  //   102,103,104,105 0x00 0x3E      Temperature sensor 6             62 - 40 = 22          °C

  float fl_lBmsTemps[3]; //Hier werden die ketzten drei Temperaturen zwischengespeichert
  for (uint8_t i=0; i<u8_lCntTempSensors; i++)
  {
    fl_lBmsTemps[2] = (float)(get16bitFromMsg(u8_lMsgoffset+1+(i*2)))-40.0;
    if(i<3) setBmsTempature(address,i,fl_lBmsTemps[2]);
    else if(i>=3 && i<5)fl_lBmsTemps[i-3]=fl_lBmsTemps[2];
  }

  u8_lMsgoffset=u8_lMsgoffset+1+(u8_lCntTempSensors*2);

  //   106,107,108,109     0x0A 0xB3      Charge/discharge current         signed int?                   A
  float f_lTotalCurrent = (float)((int16_t)get16bitFromMsg(u8_lMsgoffset))*0.01f;
  setBmsTotalCurrent(address,f_lTotalCurrent);

  //   110,111,112,113     0x14 0xF7      Total battery voltage            5367 * 0.01f = 53.67          V
  float f_lTotalVoltage = (float)get16bitFromMsg(u8_lMsgoffset+2)*0.01f;
  setBmsTotalVoltage(address, f_lTotalVoltage);

  //   114,115,116,117     0x35 0x99      Restkapazität                   13721 * 0.01f = 137.21         Ah
  uint16_t u16_lBalanceCapacity=get16bitFromMsg(u8_lMsgoffset+4)/100;

  //   118,119             0x06           Custom number                    06

  //   120,121,122,123     0x4E 0x20      Battery capacity                 20000 * 0.01f = 200.00        Ah
  uint16_t u16_lFullCapacity=get16bitFromMsg(u8_lMsgoffset+7)/100;

  //   124,125,126,127     0x00 0x12      Anzahl Zyklen                   0x12 -> 18
  uint16_t u16_lCycle=get16bitFromMsg(u8_lMsgoffset+9);

  //   128,129             0x44           Stage of charge                 0x44 -> 68                     %
  setBmsChargePercentage(address, sylcinconvertAsciiHexToByte(t_message[(u8_lMsgoffset+11)*2], t_message[(u8_lMsgoffset+11)*2+1]));

  //   130,131             0x64      State of health                      0x64 -> 100                    %

  //   132,133,134,135     0x00 0x00      Reserved
  //   136,137,138,139     0x00 0x00      Reserved
  //   140,141,142,143     0x00 0x00      Reserved


  if(mDevData->bo_sendMqttMsg)
  {
    //Nachrichten senden
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_TEMPERATURE, 3, fl_lBmsTemps[0]);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_TEMPERATURE, 4, fl_lBmsTemps[1]);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_TEMPERATURE, 5, fl_lBmsTemps[2]);

    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_BALANCE_CAPACITY, -1, u16_lBalanceCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_FULL_CAPACITY, -1, u16_lFullCapacity);
    mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_CYCLE, -1, u16_lCycle);

    //mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_BALANCE_STATUS, -1, u16_lBalanceStatus);
    //mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_FET_STATUS, -1, u16_lFetStatus);

    //mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_CHARGED_ENERGY, -1, u32_mChargeMAh);
    //mqttPublish(MQTT_TOPIC_BMS_BT, address, MQTT_TOPIC2_DISCHARGED_ENERGY, -1, u32_mDischargeMAh);
  }

  return true;
}


static bool parseMessage_Alarms(uint8_t * t_message, uint8_t address)
{
  #ifdef SYLCIN_DEBUG
  BSC_LOGI(TAG, "parseMessage: serialDev=%i",address);
  #endif

  // Ermittlung der Daten aus BMSTool 2.B

  // Byte   Address Content: Description                      Decoded content               Coeff./Unit
  //   0    0x52             Protocol version      VER        5.2
  //   1    0x01             Device address        ADR
  //   2    0x46             Device type           CID1       Lithium iron phosphate battery BMS
  //   3    0x00             Function code         CID2       0x00: Normal, 0x01 VER error, 0x02 Chksum error, ...
  //   4    0xF0             Data length checksum  LCHKSUM
  //   5    0x5C             Data length           LENID      92 Byte
  //   6    0x01             Command group

  // Byte   Description
  // The following are 24 byte alarms
  //   7    Number of cells M=16
  //   8    Cell 1 alarm
  //   9    Cell 2 alarm
  //  10    Cell 3 alarm
  //  11    Cell 4 alarm
  //  12    Cell 5 alarm
  //  13    Cell 6 alarm
  //  14    Cell 7 alarm
  //  15    Cell 8 alarm
  //  16    Cell 9 alarm
  //  17    Cell 10 alarm
  //  18    Cell 11 alarm
  //  19    Cell 12 alarm
  //  20    Cell 13 alarm
  //  21    Cell 14 alarm
  //  22    Cell 15 alarm
  //  23    Cell 16 alarm
  //  24    Number of temperatures N=6
  //  25    Cell temperature alarm 1
  //  26    Cell temperature alarm 2
  //  27    Cell temperature alarm 3
  //  28    Cell temperature alarm 4
  //  29    Environment temperature alarm
  //  30    Power temperature alarm 1
  //  31    Charge/discharge current alarm
  //  32    Total battery voltage alarm
  //  33    Short State
  //  34    Short Times
  //  35    Anzahl OEM Bytes M=16
  //  36..  OEM Bytes

  // OEM Bytes muss ich noch entschlüsseln

  // OEM Byte 0
  // Bit 0 - Warning Cell Voltage Above
  // Bit 1 - Warning Cell Voltage Under
  // Bit 2 - Warning Total Voltage Above
  // Bit 3 - Warning Total Voltage Under
  // Bit 4 - Warning Charge Overcurrent
  // Bit 5 - Warning Discharge Overcurrent
  // Bit 6 - Warning Voltage differnce
  // Bit 7 - Warning Voltage differnce

  // OEM Byte 1
  // Bit 0 - Protect Cell Voltage Above
  // Bit 1 - Protect Cell Voltage Under
  // Bit 2 - Protect Total Voltage Above
  // Bit 3 - Protect Total Voltage Under
  // Bit 4 - Protect Charge Overcurrent
  // Bit 5 - Protect Discharge Overcurrent
  // Bit 6 - Protect Discharge Overcurrent2
  // Bit 7 - Protect Short

  // OEM Byte 2
  // Bit 0 - Warning Cell over Temperature in Charge
  // Bit 1 - Warning Cell under Temperature in Charge
  // Bit 2 - Warning Cell over Temperature in Discharge
  // Bit 3 - Warning Cell under Temperature in Discharge
  // Bit 4 - Warning Environment over Temperature
  // Bit 5 - Warning Environment under Temperature
  // Bit 6 - Warning FETs over Temperature
  // Bit 7 - Warning FETs under Temperature

  // OEM Byte 3
  // Bit 0 - Protect Cell over Temperature in Charge
  // Bit 1 - Protect Cell under Temperature in Charge
  // Bit 2 - Protect Cell over Temperature in Discharge
  // Bit 3 - Protect Cell under Temperature in Discharge
  // Bit 4 - Protect Environment over Temperature
  // Bit 5 - Protect Environment under Temperature
  // Bit 6 - Protect FETs over Temperature
  // Bit 7 - Protect FETs under Temperature

  // OEM Byte 4
  // Bit 0 - Protect Inverse Charge
  // Bit 1 - Warning SOC low
  // Bit 2 - Error Charge MOS
  // Bit 3 - Error Discharge MOS
  // Bit 4 - Warning Cell over Temperature
  // Bit 5 - Warning Cell under Temperature
  // Bit 6 - Status Charge
  // Bit 7 - Status Discharge

  // OEM Byte 5
  // Bit 0 - Status Charge FET
  // Bit 1 - Status Discharge FET
  // Bit 2 - Status Precharge FET
  // Bit 3 - -
  // Bit 4 - Status Full Charge
  // Bit 5 - -
  // Bit 6 - Status CurrentLimited
  // Bit 7 - -

  // OEM Byte 6
  // Bit 0 - -
  // Bit 1 - Status Heating
  // Bit 2 - -
  // Bit 3 - -
  // Bit 4 - -
  // Bit 5 - -
  // Bit 6 - -
  // Bit 7 - -

  // OEM Byte 13
  // Bit 0 - Error Discharge MOS
  // Bit 1 - Error Charge MOS
  // Bit 2 - Error AFE
  // Bit 3 - Error Realtime clock
  // Bit 4 - Error Flash
  // Bit 5 - Error EEprom
  // Bit 6 - -
  // Bit 7 - -

  // OEM Byte 14
  // Bit 0 - -
  // Bit 1 - -
  // Bit 2 - Error Cell
  // Bit 3 - Error Tempsensor
  // Bit 4 - Error gSensor
  // Bit 5 - -
  // Bit 6 - -
  // Bit 7 - -


  //  Comments on byte alarms
  //  S/N  Value  Meaning
  //  1    0x00   Normal, no alarm
  //  2    0x01   Alarm that analog quantity reaches the lower limit
  //  3    0x02   Alarm that analog quantity reaches the upper limit
  //  4    0xF0   Other alarms
  //


  // Beispieldaten:
  // ->:  7E 35 32 30 31 34 36 34 34 45 30 30 32 30 31 46 44 32 45 0D
  // <-: 7E 35 32 30 31 34 36 30 30 46 30 35 43 30 31 31 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 36 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 46 30 30 30 30 30 30 30 30 30 30 41 33 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 30 45 43 30 45 0D

  uint8_t  u8_countCells = sylcinconvertAsciiHexToByte(t_message[14], t_message[15]);
  uint8_t  u8_countTemps = sylcinconvertAsciiHexToByte(t_message[u8_countCells * 2 + 16], t_message[u8_countCells * 2 + 17]);

  uint8_t  u8_startOEMData = u8_countCells * 2 + 16 + 2 + u8_countTemps *2 + 10;
  uint32_t u32_alarm = 0;
  boolean  bo_lValue=false;

  // Kontrolle ob Function Code OK
  if (sylcinconvertAsciiHexToByte(t_message[6], t_message[7])!=0x00)
  {
    BSC_LOGE(TAG, "Function Code Alarme nicht OK: 0x%02x", sylcinconvertAsciiHexToByte(t_message[6], t_message[7]));
    return false;
  }

  // Kontrolle Datenpaketlänge
  uint8_t u8_lDatalength = sylcinconvertAsciiHexToByte(t_message[10], t_message[11]);
  if(u8_lDatalength != 92)
  {
    BSC_LOGE(TAG, "Falsche Datenpaketlaenge Alarme (92 erwartet): %d byte lang", sylcinconvertAsciiHexToByte(t_message[10], t_message[11]));
    return false;
  }

  #ifdef SYLCIN_DEBUG
  BSC_LOGD(TAG, "Start Alarmdaten=%i",u8_startOEMData);
  #endif

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x //bit0  single cell overvoltage protection
  #define BMS_ERR_STATUS_CELL_UVP          2  x //bit1  single cell undervoltage protection
  #define BMS_ERR_STATUS_BATTERY_OVP       4  x //bit2  whole pack overvoltage protection
  #define BMS_ERR_STATUS_BATTERY_UVP       8  x //bit3  Whole pack undervoltage protection
  #define BMS_ERR_STATUS_CHG_OTP          16  x //bit4  charging over temperature protection
  #define BMS_ERR_STATUS_CHG_UTP          32  x //bit5  charging low temperature protection
  #define BMS_ERR_STATUS_DSG_OTP          64  x //bit6  Discharge over temperature protection
  #define BMS_ERR_STATUS_DSG_UTP         128  x //bit7  discharge low temperature protection
  #define BMS_ERR_STATUS_CHG_OCP         256  x //bit8  charging overcurrent protection
  #define BMS_ERR_STATUS_DSG_OCP         512  x //bit9  Discharge overcurrent protection
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  x //bit10 short circuit protection
  #define BMS_ERR_STATUS_AFE_ERROR      2048  x //bit11 Front-end detection IC error
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  - //bit12 software lock MOS
  #define BMS_ERR_STATUS_RESERVED1      8192  - //bit13 Reserved
  #define BMS_ERR_STATUS_RESERVED2     16384  - //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  - //bit15 Reserved */

  //  OEM Byte 0
  // Bit 0 - Warning Cell Voltage Above
  // Bit 1 - Warning Cell Voltage Under
  // Bit 2 - Warning Total Voltage Above
  // Bit 3 - Warning Total Voltage Under
  // Bit 4 - Warning Charge Overcurrent
  // Bit 5 - Warning Discharge Overcurrent
  // Bit 6 - Warning Voltage differnce
  // Bit 7 - Warning Voltage differnce
  uint8_t u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData], t_message[u8_startOEMData+1]);
  // nur Warnungen -> deswegen keine Daten


  // OEM Byte 1
  // Bit 0 - Protect Cell Voltage Above
  // Bit 1 - Protect Cell Voltage Under
  // Bit 2 - Protect Total Voltage Above
  // Bit 3 - Protect Total Voltage Under
  // Bit 4 - Protect Charge Overcurrent
  // Bit 5 - Protect Discharge Overcurrent
  // Bit 6 - Protect Discharge Overcurrent2
  // Bit 7 - Protect Short
  u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData+2], t_message[u8_startOEMData+3]);
  if ((u8_lByte & 0x1) == 0x1) u32_alarm |= BMS_ERR_STATUS_CELL_OVP;
  if ((u8_lByte & 0x2) == 0x2) u32_alarm |= BMS_ERR_STATUS_CELL_UVP;
  if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_BATTERY_OVP;
  if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_BATTERY_UVP;
  if ((u8_lByte & 0x10) == 0x10) u32_alarm |= BMS_ERR_STATUS_CHG_OCP;
  if ((u8_lByte & 0x20) == 0x20) u32_alarm |= BMS_ERR_STATUS_DSG_OCP;
  if ((u8_lByte & 0x40) == 0x40) u32_alarm |= BMS_ERR_STATUS_DSG_OCP;
  if ((u8_lByte & 0x80) == 0x80) u32_alarm |= BMS_ERR_STATUS_SHORT_CIRCUIT;

  // OEM Byte 2
  // Bit 0 - Warning Cell over Temperature in Charge
  // Bit 1 - Warning Cell under Temperature in Charge
  // Bit 2 - Warning Cell over Temperature in Discharge
  // Bit 3 - Warning Cell under Temperature in Discharge
  // Bit 4 - Warning Environment over Temperature
  // Bit 5 - Warning Environment under Temperature
  // Bit 6 - Warning FETs over Temperature
  // Bit 7 - Warning FETs under Temperature
  u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData+4], t_message[u8_startOEMData+5]);
  // nur Warnungen -> deswegen keine Daten

  // OEM Byte 3
  // Bit 0 - Protect Cell over Temperature in Charge
  // Bit 1 - Protect Cell under Temperature in Charge
  // Bit 2 - Protect Cell over Temperature in Discharge
  // Bit 3 - Protect Cell under Temperature in Discharge
  // Bit 4 - Protect Environment over Temperature
  // Bit 5 - Protect Environment under Temperature
  // Bit 6 - Protect FETs over Temperature
  // Bit 7 - Protect FETs under Temperature
    u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData+6], t_message[u8_startOEMData+7]);
  if ((u8_lByte & 0x1) == 0x1) u32_alarm |= BMS_ERR_STATUS_CHG_OTP;
  if ((u8_lByte & 0x2) == 0x2) u32_alarm |= BMS_ERR_STATUS_CHG_UTP;
  if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_DSG_OTP;
  if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_DSG_UTP;
  if ((u8_lByte & 0x10) == 0x10) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x20) == 0x20) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x40) == 0x40) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x80) == 0x80) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;

  // OEM Byte 4
  // Bit 0 - Protect Inverse Charge
  // Bit 1 - Warning SOC low
  // Bit 2 - Error Charge MOS
  // Bit 3 - Error Discharge MOS
  // Bit 4 - Warning Cell over Temperature
  // Bit 5 - Warning Cell under Temperature
  // Bit 6 - Status Charge
  // Bit 7 - Status Discharge
  u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData+8], t_message[u8_startOEMData+9]);
  if ((u8_lByte & 0x1) == 0x1) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;

  // OEM Byte 5
  // Bit 0 - Status Discharge FET
  // Bit 1 - Status Charge FET
  // Bit 2 - Status Precharge FET
  // Bit 3 - -
  // Bit 4 - Status Full Charge
  // Bit 5 - -
  // Bit 6 - Status CurrentLimited
  // Bit 7 - -
  u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData+10], t_message[u8_startOEMData+11]);
  // Bit 0  Discharge switch state
  bo_lValue=false;
  if ((u8_lByte & 0x1) == 0x1) bo_lValue=true;
  setBmsStateFETsDischarge(address,bo_lValue);

  // Bit 1 Charge switch state
  bo_lValue=false;
  if ((u8_lByte & 0x2) == 0x2) bo_lValue=true;
  setBmsStateFETsCharge(address,bo_lValue);

  // OEM Byte 6
  // Bit 0 - -
  // Bit 1 - Status Heating
  // Bit 2 - -
  // Bit 3 - -
  // Bit 4 - -
  // Bit 5 - -
  // Bit 6 - -
  // Bit 7 - -

  // OEM Byte 13
  // Bit 0 - Error Discharge MOS
  // Bit 1 - Error Charge MOS
  // Bit 2 - Error AFE
  // Bit 3 - Error Realtime clock
  // Bit 4 - Error Flash
  // Bit 5 - Error EEprom
  // Bit 6 - -
  // Bit 7 - -
  u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData+26], t_message[u8_startOEMData+27]);
  if ((u8_lByte & 0x1) == 0x1) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x2) == 0x2) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x10) == 0x10) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x20) == 0x20) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;

  // OEM Byte 14
  // Bit 0 - -
  // Bit 1 - -
  // Bit 2 - Error Cell
  // Bit 3 - Error Tempsensor
  // Bit 4 - Error gSensor
  // Bit 5 - -
  // Bit 6 - -
  // Bit 7 - -
  u8_lByte = sylcinconvertAsciiHexToByte(t_message[u8_startOEMData+28], t_message[u8_startOEMData+29]);
  if ((u8_lByte & 0x4) == 0x4) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x8) == 0x8) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;
  if ((u8_lByte & 0x10) == 0x10) u32_alarm |= BMS_ERR_STATUS_AFE_ERROR;



  setBmsErrors(address, u32_alarm);

  return true;

}


uint8_t sylcinconvertAsciiHexToByte(char a, char b)
{
  a = (a<='9') ? a-'0' : (a&0x7)+9;
  b = (b<='9') ? b-'0' : (b&0x7)+9;
  return (a<<4)+b;
}


static char sylcinconvertByteToAsciiHex(uint8_t v)
{
  return v>=10 ? 'A'+(v-10) : '0'+v;
}


void sylcinconvertByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length)
{
  if(length==0) return;

  for(size_t i=0; i<length; i++)
  {
    dest[2*i] = sylcinconvertByteToAsciiHex((data[i] & 0xF0) >> 4);
    dest[2*i+1] = sylcinconvertByteToAsciiHex(data[i] & 0x0F);
  }
}


uint16_t sylcinlCrc(const uint16_t len)
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
 	uint16_t u16_lRemoteCrc = (uint16_t)sylcinconvertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt-2], recvMsg[u8_lRecvBytesCnt-1]) |
    (uint16_t)(sylcinconvertAsciiHexToByte(recvMsg[u8_lRecvBytesCnt-4], recvMsg[u8_lRecvBytesCnt-3]))<<8;

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





