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

static const char *TAG = "BSC_SERIAL";

BscSerial::BscSerial()
{
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
  if(u8_mTxEnRS485pin!=0) pinMode(u8_mTxEnRS485pin, OUTPUT);

  if(u8_mHwUartNr==0)
  {
    Serial.begin(baudrate,SERIAL_8N1,u8_mRx,u8_mTx);
    stream_mPort=&Serial;
  }
  else if(u8_mHwUartNr==1)
  {
    Serial1.begin(baudrate,SERIAL_8N1,u8_mRx,u8_mTx);
    stream_mPort=&Serial1;
  }
  else if(u8_mHwUartNr==2)
  {
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
  xSemaphoreTake(mSerialMutex, portMAX_DELAY);
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
   
    default:
      readBms = 0;
  }
  xSemaphoreGive(mSerialMutex);
}


void BscSerial::cyclicRun()
{
  bool bmsReadOk=false;
  if(readBms==0){return;}    //Wenn nicht Initialisiert

  xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  if(readBms(stream_mPort, u8_mSerialNr, u8_mTxEnRS485pin)) //Wenn kein Fehler beim Holen der Daten vom BMS
  {
    bmsReadOk=true;
  }
  xSemaphoreGive(mSerialMutex);
  if(bmsReadOk)
  {
    setBmsLastDataMillis(BT_DEVICES_COUNT+u8_mSerialNr,millis());
  }
}