// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "devices/NeeyBalancer.h"
#include "BmsData.h"
#include "WebSettings.h"

static const char* TAG = "NEEY";

byte NeeyBalancer_cmdBalanceOn[20] PROGMEM =  {0xaa, 0x55, 0x11, 0x00, 0x05, 0x0d, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf3, 0xff};
byte NeeyBalancer_cmdBalanceOff[20] PROGMEM = {0xaa, 0x55, 0x11, 0x00, 0x05, 0x0d, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf2, 0xff};


static uint8_t u8_neeySendStep=0;

// Die Variablen sind für das empfangen bei den NEEYs die alles in 20 Byte Paketen senden
static uint8_t  neeyPacketNumber[7] = {0,0,0,0,0,0,0}; 
static uint32_t neeyLastRxBytes[7] = {0,0,0,0,0,0,0};
static uint32_t neeyRxDataType[7] = {0,0,0,0,0,0,0};


NeeyBalancer::NeeyBalancer()
{
};

void NeeyBalancer::init()
{
}

void NeeyBalancer::searchDataStart(uint8_t* pData)
{
  for(uint8_t i=0;i<19;i++)
  {
    if(pData[i]==0x55 && pData[i+1]==0xAA)
    {
      BSC_LOGI(TAG,"Start gefunden (%i): %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
        i, pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], pData[6], pData[7], pData[8], pData[9], pData[10],
        pData[11], pData[12], pData[13], pData[14], pData[15], pData[16], pData[17], pData[18], pData[19]);
    }
  }
}

void NeeyBalancer::neeyBalancerCopyData(uint8_t devNr, uint8_t* pData, size_t length)
{
  //BSC_LOGI(TAG,"RX dev=%i, len=%i, d=%i, %i, %i, %i, %i, %i",
  //  devNr, length, pData[0], pData[1], pData[2], pData[3], pData[4], pData[5]);

  #ifdef NEEY_DEBUG
  BSC_LOGI(TAG,"RX devNr=%i, len=%i",devNr,length);
  //log_print_buf(pData, length);

  String log="";
  uint8_t logLenCnt=0;
  for(uint8_t i=0;i<length;i++)
  {
    logLenCnt++;
    log+=String(pData[i], HEX);
    log+=" ";
    if(logLenCnt==20)
    {
      logLenCnt=0;
      BSC_LOGI(TAG,"RX: %s",log.c_str());
      log="";
    }
  }
  BSC_LOGI(TAG,"RX: %s",log.c_str());
  log="";
  #endif


  if(length == 241)
  {
    // 0     2   0x55 0xAA              Header
    // 2     1   0x11                   Device address
    // 3     1   0x01                   Function (read)
    // 4     2   0x02 0x00              Command (cell info)
    if(!(pData[0]==0x55 && pData[1]==0xAA && pData[2]==0x11 && pData[3]==0x01 && pData[4]==0x02 && pData[5]==0x00)) return;
    if((millis()-getBmsLastDataMillis(devNr))<500) return; //Nur alle 500ms eine neue Nachricht akzeptieren

    float    f_lTmpValue;
    uint32_t u32_lTmpValue;
    //Cellspannung
    for(uint8_t i=0;i<24;i++)
    {
      memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_CELVOLTAGE+(i*4), 4);
      setBmsCellVoltage(devNr,i,(uint16_t)(f_lTmpValue*1000));
    }

    //Avg. Voltage
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_AVERAGEVOLTAGE, 4);
    setBmsAvgVoltage(devNr,(uint16_t)(f_lTmpValue*1000));

    //Delta Cell Voltage
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_DELTACELLVOLTAGE, 4);
    setBmsMaxCellDifferenceVoltage(devNr,(uint16_t)(f_lTmpValue*1000));

    /*
    Errors:
    STATUS_OK                0
    STATUS_CELL_OVP          1   //bit0 single cell overvoltage protection
    STATUS_CELL_UVP          2   //bit1 single cell undervoltage protection
    STATUS_PACK_OVP          4   //bit2 whole pack overvoltage protection
    STATUS_PACK_UVP          8   //bit3 Whole pack undervoltage protection
    STATUS_CHG_OTP          16   //bit4 charging over-temperature protection
    STATUS_CHG_UTP          32   //bit5 charging low temperature protection
    STATUS_DSG_OTP          64   //bit6 Discharge over temperature protection
    STATUS_DSG_UTP         128   //bit7 discharge low temperature protection
    STATUS_CHG_OCP         256   //bit8 charging overcurrent protection
    STATUS_DSG_OCP         512   //bit9 Discharge overcurrent protection
    STATUS_SHORT_CIRCUIT  1024   //bit10 short circuit protection
    STATUS_AFE_ERROR      2048   //bit11 Front-end detection IC error
    STATUS_SOFT_LOCK      4096   //bit12 software lock MOS
    */
    memcpy(&u32_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_ERRCELLOV, 4);
    setBmsErrors(devNr, u32_lTmpValue);

    struct bmsData_s *p_lBmsData = getBmsData();
    bmsDataSemaphoreTake();
    //memcpy(&p_lBmsData->bmsCellResistance[devNr][0], pData+OFFSET_NEEYBAL4A_DATA0x2_CELLRESISTANCE, 4*24);
    memcpy(&p_lBmsData->bmsMaxVoltageCellNumber[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_MAXVOLTCELLNR, 1);
    memcpy(&p_lBmsData->bmsMinVoltageCellNumber[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_MINVOLTCELLNR, 1);
    memcpy(&p_lBmsData->bmsIsBalancingActive[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_BALANCING, 1);
    bmsDataSemaphoreGive();

    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_TOTALVOLTAGE, 4);
    setBmsTotalVoltage(devNr,f_lTmpValue);
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_BALANCINGCUR, 4);
    setBmsBalancingCurrent(devNr,f_lTmpValue);
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_TEMPERATUR, 4);
    setBmsTempature(devNr,0,f_lTmpValue);
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_TEMPERATUR+4, 4);
    setBmsTempature(devNr,1,f_lTmpValue);


    uint16_t f_lMaxZellVoltage = getBmsCellVoltage(devNr,getBmsMaxVoltageCellNumber(devNr));
    uint16_t f_lMinZellVoltage = getBmsCellVoltage(devNr,getBmsMinVoltageCellNumber(devNr));
    //setBmsMaxCellDifferenceVoltage(devNr, f_lMaxZellVoltage-f_lMinZellVoltage);

    setBmsMaxCellVoltage(devNr, f_lMaxZellVoltage);
    setBmsMinCellVoltage(devNr, f_lMinZellVoltage);

    //BSC_LOGI(TAG,"setBmsLastDataMillis");
    setBmsLastDataMillis(devNr,millis());
  }
  else if(length <= 20)
  {
    /*Packet	von	bis
     *  0      0	 19
     *  1     20	 39
     *  2     40	 59
     *  3     60	 79
     *  4	    80	 99
     *  5	   100	119
     *  6	   120	139
     *  7	   140	159
     *  8	   160	179
     *  9	   180	199
     *  10	 200	219
     *  11	 220	239
     *  12	 240	259
     *  13	 260	279
     *  14	 280	299
     *  15	 300	319
     */

    #ifdef NEEY_DEBUG
    BSC_LOGI(TAG,"dev=%i, p=%i", devNr, neeyPacketNumber[devNr]);
    #endif

    //NeeyBalancer::searchDataStart(pData);

    // Settings
    if((pData[0]==0x55 && pData[1]==0xAA && pData[2]==0x11 && pData[3]==0x01 && pData[4]==0x04 && pData[5]==0x00 && pData[6]==0x64))
    {
      #ifdef NEEY_DEBUG
      BSC_LOGI(TAG,"Settings: dev=%i", devNr);
      #endif

      neeyRxDataType[devNr] = 1;
      neeyPacketNumber[devNr] = 0;
      return;
    }

    // Batt. Data
    else if(pData[0]==0x55 && pData[1]==0xAA && pData[2]==0x11 && pData[3]==0x01 && pData[4]==0x02 && pData[5]==0x00)
    {
      #ifdef NEEY_DEBUG
      BSC_LOGI(TAG,"Battdata: dev=%i, len=%i, pkgNr=%i", devNr, length, neeyPacketNumber[devNr]);
      #endif

      //BSC_LOGI(TAG,"%i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
      //  pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], pData[6], pData[7], pData[8], pData[9], pData[10],
      //  pData[11], pData[12], pData[13], pData[14], pData[15], pData[16], pData[17], pData[18], pData[19]);

      neeyRxDataType[devNr] = 2;
      neeyPacketNumber[devNr] = 0;
    }



    // Settings
    if(neeyRxDataType[devNr] == 1)
    { 
      BSC_LOGI(TAG,"S; dev=%i, p=%i", devNr, neeyPacketNumber[devNr]);

      if(neeyPacketNumber[devNr] == 0)
      { 
        neeyPacketNumber[devNr] = 1;

        bmsDataSemaphoreTake();
        memcpy(getBmsSettingsReadback(devNr), &pData[8], 11);
        bmsDataSemaphoreGive();
      } 
      else if(neeyPacketNumber[devNr] == 1)
      { 
        neeyPacketNumber[devNr] = 2;

        bmsDataSemaphoreTake();
        memcpy(getBmsSettingsReadback(devNr)+11, &pData[0], 20);
        bmsDataSemaphoreGive();
      } 
      else if(neeyPacketNumber[devNr] == 2)
      { 
        neeyPacketNumber[devNr] = 3;

        bmsDataSemaphoreTake();
        memcpy(getBmsSettingsReadback(devNr)+31, &pData[0], 1);
        bmsDataSemaphoreGive();

        setBmsLastDataMillis(devNr,millis());
      } 
    } 
    
    // NEEY Batt. Data
    else if(neeyRxDataType[devNr] == 2)
    { 
      //BSC_LOGI(TAG,"B; dev=%i, p=%i", devNr, neeyPacketNumber[devNr]);

      // Byte 0-19
      if(neeyPacketNumber[devNr] == 0)
      {
        float    f_lTmpValue;
        memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_CELVOLTAGE, 4);
        setBmsCellVoltage(devNr,0,(uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"cv0=%f", f_lTmpValue);

        memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_CELVOLTAGE+4, 4);
        setBmsCellVoltage(devNr,1,(uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"cv1=%f", f_lTmpValue);

        memcpy(((uint8_t*)&neeyLastRxBytes[devNr]), pData+OFFSET_NEEYBAL4A_DATA0x2_CELVOLTAGE+8, 3);  // Die ersten drei Byte der nächsten Zelle
      }

      // Byte 20-39, 40-59, 60-79, 80-99
      else if(neeyPacketNumber[devNr] >= 1 && neeyPacketNumber[devNr] <= 4)
      {
        float    f_lTmpValue;

        uint8_t ii = (neeyPacketNumber[devNr] - 1) * 5; 

        memcpy(((uint8_t*)&f_lTmpValue), ((uint8_t*)&neeyLastRxBytes[devNr]), 3);
        memcpy(((uint8_t*)&f_lTmpValue)+3, pData, 1);
        setBmsCellVoltage(devNr, ii+2, (uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"cv%i=%f", ii+2, f_lTmpValue);

        memcpy(&f_lTmpValue, pData+1, 4);
        setBmsCellVoltage(devNr, ii+3, (uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"cv%i=%f", ii+3, f_lTmpValue);

        memcpy(&f_lTmpValue, pData+5, 4);
        setBmsCellVoltage(devNr, ii+4, (uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"cv%i=%f", ii+4, f_lTmpValue);

        memcpy(&f_lTmpValue, pData+9, 4);
        setBmsCellVoltage(devNr, ii+5, (uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"cv%i=%f", ii+5, f_lTmpValue);

        memcpy(&f_lTmpValue, pData+13, 4);
        setBmsCellVoltage(devNr, ii+6, (uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"cv%i=%f", ii+6, f_lTmpValue);

        memcpy(((uint8_t*)&neeyLastRxBytes[devNr]), pData+17, 3);  // Die ersten drei Byte der nächsten Zelle
      } 

      // Byte 200-219
      else if(neeyPacketNumber[devNr] == 10)
      {
        // 201   4   0xDE 0x40 0x51 0x42              Total voltage
        // 205   4   0xDE 0x40 0x51 0x40              Average cell voltage
        // 209   4   0x00 0x17 0x08 0x3C              Delta Cell Voltage
        // 213   1   0x0A                             Max voltage cell
        // 214   1   0x00                             Min voltage cell
        // 215   1   0x0F                             Single number (not exposed at the android app)
        // 217   4   0x19 0xA1 0x82 0xC0              Balancing current

        float    f_lTmpValue;

        memcpy(&f_lTmpValue, pData+1, 4);
        setBmsTotalVoltage(devNr,f_lTmpValue);
        //if(pData[1] == 0) { 
        //  BSC_LOGI(TAG,"RX dev=%i, len=%i, d=%i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i, %i",
        //    devNr, length, pData[0], pData[1], pData[2], pData[3], pData[4], pData[5], pData[6], pData[7], pData[8], pData[9], pData[10],
        //    pData[11], pData[12], pData[13], pData[14], pData[15], pData[16], pData[17], pData[18], pData[19]);
        //} 
        //BSC_LOGI(TAG,"tv=%f", f_lTmpValue);


        memcpy(&f_lTmpValue, pData+5, 4);
        setBmsAvgVoltage(devNr,(uint16_t)(f_lTmpValue*1000));

        //Delta Cell Voltage
        memcpy(&f_lTmpValue, pData+9, 4);
        setBmsMaxCellDifferenceVoltage(devNr,(uint16_t)(f_lTmpValue*1000));
        //BSC_LOGI(TAG,"diff=%f", f_lTmpValue);

        // Max Cellvoltage
        setBmsMaxVoltageCellNumber(devNr,pData[13]);
        uint16_t tCellVoltage = getBmsCellVoltage(devNr, pData[13]);
        setBmsMaxCellVoltage(devNr, tCellVoltage);
        //BSC_LOGI(TAG,"max=%i, %i", pData[13], tCellVoltage);

        // Min Cellvoltage
        setBmsMinVoltageCellNumber(devNr,pData[14]);
        tCellVoltage = getBmsCellVoltage(devNr, pData[14]);
        setBmsMinCellVoltage(devNr, tCellVoltage);
        //BSC_LOGI(TAG,"min=%i, %i", pData[14], tCellVoltage);

        memcpy(((uint8_t*)&neeyLastRxBytes[devNr]), pData+17, 3);  // Die ersten drei Byte vom nächste Wert
      } 

      // Byte 220-239
      else if(neeyPacketNumber[devNr] == 11)
      {
        // 217   4   0x19 0xA1 0x82 0xC0              Balancing current
        // 221   4   0xC3 0xF5 0x48 0x42              Temperature 1
        // 225   4   0xC3 0xF5 0x48 0x42              Temperature 2

        float    f_lTmpValue;

        memcpy(((uint8_t*)&f_lTmpValue), ((uint8_t*)&neeyLastRxBytes[devNr]), 3);
        memcpy(((uint8_t*)&f_lTmpValue)+3, pData, 1);
        setBmsBalancingCurrent(devNr,f_lTmpValue);
    
        memcpy(&f_lTmpValue, pData+1, 4);
        setBmsTempature(devNr,0,f_lTmpValue);
        //BSC_LOGI(TAG,"t0=%f", f_lTmpValue);

        memcpy(&f_lTmpValue, pData+5, 4);
        setBmsTempature(devNr,1,f_lTmpValue);
        //BSC_LOGI(TAG,"t1=%f", f_lTmpValue);

        // 
        setBmsLastDataMillis(devNr,millis());
      } 

      if(neeyPacketNumber[devNr] <= 15)
      {
        neeyPacketNumber[devNr]++;
        //neeyRxDataType[devNr] = 0;
      } 
      else if(neeyPacketNumber[devNr] >= 16)
      {
        //BSC_LOGI(TAG,"dev=%i: Paket %i", devNr, neeyPacketNumber[devNr]);
        neeyPacketNumber[devNr]++;
      } 
    }

    

  } 
  else //Alles was keine 241 oder 20 Byte ist
  {  
    //String log="";
    //for(uint8_t i=0;i<length;i++){log+=String(pData[i], HEX); log+=" ";}
    //BSC_LOGI(TAG,"RX: %s",log.c_str()); log="";

    //BSC_LOGI(TAG,"RX");
    //log_print_buf(pData, length);

    if((pData[0]==0x55 && pData[1]==0xAA && pData[2]==0x11 && pData[3]==0x01 && pData[4]==0x04 && pData[5]==0x00 && pData[6]==0x64))
    {
      //BSC_LOGI(TAG,"RX settings");

      /*BSC_LOGI(TAG,"RX dev=%i, len=%i",devNr,length);
      String log="";
      uint8_t logLenCnt=0;
      for(uint8_t i=0;i<length;i++)
      {
        logLenCnt++;
        log+=String(pData[i], HEX);
        log+=" ";
        if(logLenCnt==20)
        {
          logLenCnt=0;
          BSC_LOGI(TAG,"RX: %s",log.c_str());
          log="";
        }
      }
      BSC_LOGI(TAG,"RX: %s",log.c_str());
      log="";*/

      bmsDataSemaphoreTake();
      memcpy(getBmsSettingsReadback(devNr), &pData[8], 32);
      bmsDataSemaphoreGive();

      return;
    }
  }

}



uint8_t NeeyBalancer::neeyBtCrc(uint8_t devTyp, uint8_t* data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        if(devTyp==ID_BT_DEVICE_NEEY_GW_24S4EB) crc = crc ^ data[i];
        else crc = crc + data[i];
    }
    return crc;
}


void NeeyBalancer::neeyBtBuildSendData(uint8_t devTyp, uint8_t* frame, uint8_t byte3, uint8_t cmd, uint8_t func, uint32_t value)
{
    frame[0] = 0xAA;     // start sequence
    frame[1] = 0x55;     // start sequence
    frame[2] = 0x11;     // start sequence
    frame[3] = byte3;    // start sequence
    frame[4] = cmd;      //
    frame[5] = func;     //
    frame[6] = 0x14;     // length
    frame[7] = 0x00;
    frame[8] = value >> 0;
    frame[9] = value >> 8;
    frame[10] = value >> 16;
    frame[11] = value >> 24;
    frame[12] = 0x00;
    frame[13] = 0x00;
    frame[14] = 0x00;
    frame[15] = 0x00;
    frame[16] = 0x00;
    frame[17] = 0x00;
    frame[18] = neeyBtCrc(devTyp, frame, 18);
    frame[19] = 0xFF;

    #ifdef NEEY_WRITE_DATA_DEBUG
    String log="";
    for(uint8_t i=0;i<20;i++)
    {
      log+=String(frame[i], HEX);
      log+=" ";
    }
    BSC_LOGI(TAG,"TX Frame: %s",log.c_str());
    log="";

    //BSC_LOGI(TAG,"TX");
    //log_print_buf(frame, 20);
    #endif
}

void NeeyBalancer::neeyBtBuildSendData(uint8_t devTyp, uint8_t* frame, uint8_t cmd, uint8_t func, uint32_t value)
{
    neeyBtBuildSendData(devTyp, frame, 0x0, cmd, func, value);
}

#if 0
void NeeyBalancer::neeyWriteMsg1(NimBLERemoteCharacteristic* pChr)
{
  //{0xaa, 0x55, 0x11, 0x01, 0x01, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfa, 0xff};
  uint8_t  frame[20];
  neeyBtBuildSendData(frame, 0x01, 0x01, 0x0, (uint32_t)0x0); //msg1
  pChr->writeValue(frame, 20, true);
}
#endif

void NeeyBalancer::neeyWriteMsg2(uint8_t devTyp, NimBLERemoteCharacteristic* pChr)
{
  uint8_t  frame[20];
  neeyBtBuildSendData(devTyp, frame, 0x01, 0x04, 0x0, (uint32_t)0x0); //msg2
  pChr->writeValue(frame, 20, true);
}

void NeeyBalancer::neeyBtBuildSendData(uint8_t devTyp, uint8_t* frame, uint8_t cmd, uint8_t func, float value)
{
  //{0xaa, 0x55, 0x11, 0x01, 0x04, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff};
  uint32_t ui32_lValue;
  memcpy(&ui32_lValue, &value, 4);

  neeyBtBuildSendData(devTyp, frame, 0x0, cmd, func, ui32_lValue);
}

void NeeyBalancer::sendNeeyConnectMsg(uint8_t devtyp, NimBLERemoteCharacteristic* pChr)
{
  uint8_t  frame[20];
  if(devtyp==ID_BT_DEVICE_NEEY_EK_24S4EB)
  {
    //byte NeeyBalancer_8A_getInfo1[20] PROGMEM = {0xaa, 0x55, 0x11, 0x01, 0x01, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xff}; // 0x26
    //byte NeeyBalancer_8A_getInfo2[20] PROGMEM = {0xaa, 0x55, 0x11, 0x01, 0x04, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0xff}; // 0x29
    //byte NeeyBalancer_8A_getInfo3[20] PROGMEM = {0xaa, 0x55, 0x11, 0x01, 0x02, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27, 0xff}; // 0x27

    neeyBtBuildSendData(devtyp, frame, 0x01, 0x01, 0x0, (uint32_t)0x0);
    frame[18]=0x26;
    pChr->writeValue(frame, 20, true);
    delay(200);

    neeyBtBuildSendData(devtyp, frame, 0x01, 0x04, 0x0, (uint32_t)0x0);
    frame[18]=0x29;
    pChr->writeValue(frame, 20, true);
    delay(200);

    neeyBtBuildSendData(devtyp, frame, 0x01, 0x02, 0x0, (uint32_t)0x0);
    frame[18]=0x27;
    pChr->writeValue(frame, 20, true);
  }
}


void NeeyBalancer::neeySetBalancerOnOff(NimBLERemoteCharacteristic* pChr, boolean u8_state)
{
  if(u8_state) pChr->writeValue(NeeyBalancer_cmdBalanceOn, 20);
  else pChr->writeValue(NeeyBalancer_cmdBalanceOff, 20);
}

bool NeeyBalancer::neeyWriteData(uint8_t devtyp, uint8_t btDevNr, NimBLERemoteCharacteristic* pChr)
{
  bool ret=false;

  uint8_t  frame[20];
  uint8_t  u8_lValue;
  uint16_t u16_lValue;
  float    f_lValue;

  //u8_neeySendStep++;

  #ifdef NEEY_WRITE_DATA_DEBUG
  BSC_LOGI(TAG,"neeyWriteData: dev=%i",btDevNr);
  #endif

  switch(u8_neeySendStep)
  {
    case 1:
      neeyBtBuildSendData(devtyp, frame, 0x04, 0x0, (uint32_t)0x0);
      pChr->writeValue(frame, 20, true);
      break;

    case 2:
      u8_lValue=(uint8_t)WebSettings::getIntFlash(ID_PARAM_NEEY_BALANCER_ON,btDevNr,DT_ID_PARAM_NEEY_BALANCER_ON);
      if(u8_lValue==0) neeySetBalancerOnOff(pChr, false);
      else if(u8_lValue==110) neeySetBalancerOnOff(pChr, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: BALANCER_ON val=%i",u8_lValue);
      #endif
      break;

    case 3:
      u8_lValue=(uint8_t)WebSettings::getIntFlash(ID_PARAM_NEEY_BAT_TYPE,btDevNr,DT_ID_PARAM_NEEY_BAT_TYPE);
      neeyBtBuildSendData(devtyp, frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_BAT_TYPE, (uint32_t)u8_lValue);
      pChr->writeValue(frame, 20, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: BAT_TYPE val=%i",u8_lValue);
      #endif
      break;

    case 4:
      u8_lValue=(uint8_t)WebSettings::getIntFlash(ID_PARAM_NEEY_CELLS,btDevNr,DT_ID_PARAM_NEEY_CELLS);
      neeyBtBuildSendData(devtyp, frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_CELLS, (uint32_t)u8_lValue);
      pChr->writeValue(frame, 20, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: CELLS val=%i",u8_lValue);
      #endif
      break;

    case 5:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_START_VOLTAGE,btDevNr);
      neeyBtBuildSendData(devtyp, frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_START_VOL, f_lValue);
      pChr->writeValue(frame, 20, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: START_VOL val=%f",f_lValue);
      #endif
      break;

    case 6:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_MAX_BALANCE_CURRENT,btDevNr);
      neeyBtBuildSendData(devtyp, frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT, f_lValue);
      pChr->writeValue(frame, 20, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: BAL_CUR val=%f",f_lValue);
      #endif
      break;

    case 7:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_EQUALIZATION_VOLTAGE,btDevNr);
      neeyBtBuildSendData(devtyp, frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE, f_lValue);
      pChr->writeValue(frame, 20, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: EQL_VOL val=%f",f_lValue);
      #endif
      break;

    case 8:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_SLEEP_VOLTAGE,btDevNr);
      neeyBtBuildSendData(devtyp, frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE, f_lValue);
      pChr->writeValue(frame, 20, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: SLEEP_VOL val=%f",f_lValue);
      #endif
      break;

    case 9:
      u16_lValue=(uint16_t)WebSettings::getIntFlash(ID_PARAM_NEEY_BAT_CAPACITY,btDevNr,DT_ID_PARAM_NEEY_BAT_CAPACITY);
      neeyBtBuildSendData(devtyp, frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_BAT_CAP, (uint32_t)u16_lValue);
      pChr->writeValue(frame, 20, true);
      #ifdef NEEY_WRITE_DATA_DEBUG
      BSC_LOGI(TAG,"neeyWriteData: BAT_CAP val=%f",f_lValue);
      #endif
      break;

    case 10:
      neeyBtBuildSendData(devtyp, frame, 0x01, 0x04, 0x0, (uint32_t)0x0); //msg2
      pChr->writeValue(frame, 20, true);
      break;

    case 11:
      neeyBtBuildSendData(devtyp, frame, 0x04, 0x0, (uint32_t)0x0);
      pChr->writeValue(frame, 20, true);

      //u8_neeySendStep=0;
      ret=true;
      break;
  }

  //NeeyBalancer::neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_BUZZER_MODE, f_lTmpValue);

  return ret;
}

/*
Default startStep = 1
*/
void NeeyBalancer::neeyWriteData_GotoStartStep(uint8_t startStep)
{
  if(startStep==0) startStep=1;

  u8_neeySendStep=startStep; //1
  #ifdef NEEY_WRITE_DATA_DEBUG
  BSC_LOGI(TAG,"neeyWriteData_GotoStartStep");
  #endif
}


bool NeeyBalancer::neeyWriteData_GotoNextStep()
{
  if(u8_neeySendStep==11)
  {
    u8_neeySendStep=0;
    return true;
  }
  else
  {
    u8_neeySendStep++;
    return false;
  }
}


float NeeyBalancer::neeyGetReadbackDataFloat(uint8_t devNr, uint8_t dataType)
{
  float retValue=0;
  bmsDataSemaphoreTake();

  switch(dataType)
  {
    case NEEYBAL4A_FUNC_SETTING_START_VOL:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[1], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[5], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[9], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[20], 4);
      break;
  }

  bmsDataSemaphoreGive();
  return retValue;
}


uint32_t NeeyBalancer::neeyGetReadbackDataInt(uint8_t devNr, uint8_t dataType)
{
  uint32_t retValue=0;
  bmsDataSemaphoreTake();

  switch(dataType)
  {
    case NEEYBAL4A_FUNC_SETTING_CELLS:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[0], 1);
      break;
    case NEEYBAL4A_FUNC_SETTING_BAT_TYPE:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[15], 1);
      break;
    case NEEYBAL4A_FUNC_SETTING_BAT_CAP:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[16], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_BUZZER_MODE:
      break;
    case NEEYBAL4A_FUNC_SETTING_BALLANCER_ON_OFF:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[13], 1);
      break;
  }

  bmsDataSemaphoreGive();
  return retValue;
}


void NeeyBalancer::getNeeyReadbackDataAsString(String &value)
{
  String  str_lText="";
  uint8_t u8_lValue=0;
  uint8_t devTyp;

  //ID_PARAM_NEEY_BUZZER //not use
  //ID_PARAM_NEEY_BALANCER_ON

  if(u8_neeySendStep>0) //Write Data, please wait
  {
    value += F("display;flex|");  //spinner visible on
    value += F("btn1;0");        //disable
    //Hier kein "|" anhängen, da dies im nächsten Schritte bei den BMS Daten erfolgt
  }
  else
  {
    value += F("display;none|");  //spinner visible off
    value += F("btn1;1");        //enable
    //Hier kein "|" anhängen, da dies im nächsten Schritte bei den BMS Daten erfolgt
  }

  for(uint8_t i=0;i<BT_INTERNAL_DEVICES_COUNT;i++)
  {
    devTyp = WebSettings::getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV);
    if(devTyp==ID_BT_DEVICE_NEEY_GW_24S4EB || devTyp==ID_BT_DEVICE_NEEY_EK_24S4EB)
    {
      value += "|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_START_VOLTAGE, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_START_VOL),3) + " V|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_MAX_BALANCE_CURRENT, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT),3) + " A|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_SLEEP_VOLTAGE, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE),3) + " V|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_EQUALIZATION_VOLTAGE, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE),3) + " V|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_CELLS, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_CELLS)) + "|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_BAT_TYPE, i)) + ";";
      u8_lValue = NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BAT_TYPE);
      str_lText="";
      if(u8_lValue==1) str_lText=F("NCM");
      else if(u8_lValue==2) str_lText=F("LFP");
      else if(u8_lValue==3) str_lText=F("LTO");
      else if(u8_lValue==4) str_lText=F("PbAc");
      else String(u8_lValue);
      value += str_lText + "|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_BAT_CAPACITY, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BAT_CAP)) + " Ah|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_BALANCER_ON, i)) + ";";
      u8_lValue = NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BALLANCER_ON_OFF);
      str_lText="";
      if(u8_lValue==0) str_lText=F("AUS");
      else if(u8_lValue==1) str_lText=F("EIN");
      value += str_lText;

    }
  }
}
