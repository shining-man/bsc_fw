// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BscSerial.h"
#include "defines.h"
#include "WebSettings.h"
#include "BmsData.h"
#include "log.h"

//include Devices
#include "devices/JbdBms.h"
#include "devices/JkBms.h"
#include "devices/SeplosBms.h"
#include "devices/DalyBms.h"

static const char *TAG = "BSC_SERIAL";

static uint8_t u8_mFilterBmsCellVoltageMaxCount;

#ifdef UTEST_BMS_FILTER
bool readBmsTestData(uint8_t devNr);
#endif


BscSerial::BscSerial()
{
  for(uint8_t i=0;i<8;i++)
  {
    readBmsExt[i]=0;
  }
};

BscSerial::BscSerial(uint8_t u8_lSerialNr, uint8_t hwUartNr, uint8_t rx, uint8_t tx, uint8_t txEnRS485pin)
{
  mSerialMutex = xSemaphoreCreateMutex();
  isSoftSerial=false;
  u8_mSerialNr = u8_lSerialNr;
  u8_mHwUartNr = hwUartNr;
  u8_mTxEnRS485pin = txEnRS485pin;
  u8_mRx=rx;
  u8_mTx=tx;

  if(u8_mHwUartNr==0) Serial.end();
}

BscSerial::BscSerial(uint8_t u8_lSerialNr, uint8_t rx, uint8_t tx, uint8_t txEnRS485pin)
{
  mSerialMutex = xSemaphoreCreateMutex();
  isSoftSerial=true;
  u8_mSerialNr = u8_lSerialNr;
  u8_mHwUartNr = 0;
  u8_mTxEnRS485pin = txEnRS485pin;
  u8_mRx=rx;
  u8_mTx=tx;
}


void BscSerial::initSerial()
{
  u8_mFilterBmsCellVoltageMaxCount=0;
  u8_mAddData=0;

  if(isSoftSerial==true)
  {
    setSoftSerial(9600);
  }
  else
  {
    setHwSerial(9600);
  }

  uint8_t funktionsTyp = WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,u8_mSerialNr,0);
  ESP_LOGI(TAG, "initSerial u8_mSerialNr=%i, funktionsTyp=%i",u8_mSerialNr,funktionsTyp);
  setReadBmsFunktion(funktionsTyp);
}


void BscSerial::setHwSerial(uint32_t baudrate)
{
  u8_mAddData=0;
  if(u8_mTxEnRS485pin!=0) pinMode(u8_mTxEnRS485pin, OUTPUT);

  if(u8_mHwUartNr==0)
  {
    Serial.end();
    pinMode(u8_mTx, OUTPUT | PULLUP);
    Serial.begin(baudrate,SERIAL_8N1,u8_mRx,u8_mTx);
    stream_mPort=&Serial;
  }
  else if(u8_mHwUartNr==1)
  {
    Serial1.end();
    pinMode(u8_mTx, OUTPUT);
    Serial1.begin(baudrate,SERIAL_8N1,u8_mRx,u8_mTx);
    stream_mPort=&Serial1;
  }
  else if(u8_mHwUartNr==2)
  {
    Serial2.end();
    pinMode(u8_mTx, OUTPUT);
    Serial2.begin(baudrate,SERIAL_8N1,u8_mRx,u8_mTx);
    stream_mPort=&Serial2;
  }
}


void BscSerial::setSoftSerial(uint32_t baudrate)
{
  if(u8_mTxEnRS485pin!=0) pinMode(u8_mTxEnRS485pin, OUTPUT);

  static SoftwareSerial mySwSerial(u8_mRx,u8_mTx,false);
  stream_mPort = &mySwSerial;
  static_cast<SoftwareSerial*>(stream_mPort)->begin(baudrate);
}


void BscSerial::setSerialBaudrate(uint32_t baudrate)
{
  if(u8_mSerialNr==0)
  {
    Serial1.end();
    setHwSerial(baudrate);
  }
  else if(u8_mSerialNr==1)
  {
    Serial2.end();
    setHwSerial(baudrate);
  }
  else if(u8_mSerialNr==2)
  {
    #ifndef DEBUG_ON_HW_SERIAL
    Serial.end();
    setHwSerial(baudrate);
    #else
    static_cast<SoftwareSerial*>(stream_mPort)->end();
    setSoftSerial(baudrate);
    #endif
  }
}


void BscSerial::setReadBmsFunktion(uint8_t funktionsTyp)
{
  u8_mFilterBmsCellVoltageMaxCount = WebSettings::getIntFlash(ID_PARAM_BMS_FILTER_RX_ERROR_COUNT,0,0,0,PARAM_DT_U8);

  xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  u8_mAddData=0;

  switch (funktionsTyp)
  {
    case ID_SERIAL_DEVICE_NB:
      readBms = 0;
      break;

    case ID_SERIAL_DEVICE_JBDBMS:
      ESP_LOGI(TAG,"setReadBmsFunktion JBD-BMS");
      setSerialBaudrate(9600);
      readBms = &JbdBms_readBmsData;
      break;

    case ID_SERIAL_DEVICE_JKBMS:
      ESP_LOGI(TAG,"setReadBmsFunktion JK-BMS");
      setSerialBaudrate(115200);
      readBms = &JkBms_readBmsData;
      break;
      
    case ID_SERIAL_DEVICE_SEPLOSBMS:
      ESP_LOGI(TAG,"setReadBmsFunktion SEPLOS");
      setSerialBaudrate(19200);
      readBms = &SeplosBms_readBmsData;
      break;
      
    case ID_SERIAL_DEVICE_DALYBMS:
      ESP_LOGI(TAG,"setReadBmsFunktion DALY");
      setSerialBaudrate(9600);
      readBms = &DalyBms_readBmsData;
      break;
   
    default:
      readBms = 0;
  }

  if(u8_mSerialNr==2)
  {
    if(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,u8_mSerialNr,0)==ID_SERIAL_DEVICE_SEPLOSBMS)
    {
      u8_mAddData=WebSettings::getInt(ID_PARAM_SERIAL_SEPLOS_CONNECT_TO_ID,0,0,0);
    }
  }
  xSemaphoreGive(mSerialMutex);
}


void BscSerial::cyclicRun()
{
  bool bmsReadOk=false;
  if(readBms==0){return;}    //Wenn nicht Initialisiert

  //Aktuell (später entfernen)
  uint8_t *u8_pBmsFilterErrorCounter = getBmsFilterErrorCounter(BT_DEVICES_COUNT+u8_mSerialNr);

  xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  *u8_pBmsFilterErrorCounter &= ~(0x80); //Fehlermerker des aktuellen Durchgangs löschen (bit 0)
  #ifndef UTEST_BMS_FILTER
  bmsReadOk=readBms(stream_mPort, u8_mSerialNr, u8_mTxEnRS485pin, u8_mAddData); //Wenn kein Fehler beim Holen der Daten vom BMS  
  #else
  bmsReadOk=readBmsTestData(BT_DEVICES_COUNT+u8_mSerialNr);
  ESP_LOGI(TAG,"Filter: RX serial Data; errCnt=%i",*u8_pBmsFilterErrorCounter);
  #endif
  if((*u8_pBmsFilterErrorCounter>>7)) //Wenn beim Empfang Fehler wahren
  {
    //ESP_LOGI(TAG,"Filter: RX Error; errCnt=%i",*u8_pBmsFilterErrorCounter);
    if((*u8_pBmsFilterErrorCounter&0x7F)>=u8_mFilterBmsCellVoltageMaxCount) //Wenn Fehler
    {
      //Zu häufig Fehler
      bmsReadOk=false;
      //ESP_LOGI(TAG,"Filter: Zu viele Errors ist=%i, max=%i", (*u8_pBmsFilterErrorCounter&0x7F), u8_mFilterBmsCellVoltageMaxCount);
    }
    else
    {
      if((*u8_pBmsFilterErrorCounter&0x7F)<125) *u8_pBmsFilterErrorCounter=*u8_pBmsFilterErrorCounter+1;
      //ESP_LOGI(TAG,"Filter: ErrCount ist=%i, max=%i", (*u8_pBmsFilterErrorCounter&0x7F), u8_mFilterBmsCellVoltageMaxCount);
    }
  }
  else
  {
    if(*u8_pBmsFilterErrorCounter>0) ESP_LOGI(TAG,"Filter: Reset RX Error");
    *u8_pBmsFilterErrorCounter = 0; //Fehler Counter zurücksetzen
  }
  xSemaphoreGive(mSerialMutex);
  if(bmsReadOk)
  {
    setBmsLastDataMillis(BT_DEVICES_COUNT+u8_mSerialNr,millis());
  }
  else ESP_LOGE(TAG,"Checksum wrong");
}





  #ifdef UTEST_BMS_FILTER
  static uint16_t u16_filterTestErrCnt=0;
  bool readBmsTestData(uint8_t devNr)
  {
    ESP_LOGI(TAG,"readBmsTestData: u16_filterTestErrCnt=%i", u16_filterTestErrCnt);

    for(uint8_t i=0;i<24;i++)
    {
      setBmsCellVoltage(devNr,i,3450);
    }

    if(u16_filterTestErrCnt>5 && u16_filterTestErrCnt<30)
    {
      setBmsCellVoltage(devNr,3,6900);
    }


    u16_filterTestErrCnt++;
    return true;
  }
  #endif