// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BscSerial.h"
#include "defines.h"
#include "devices/JbdBms.h"
#include "WebSettings.h"
#include "BmsData.h"


BscSerial::BscSerial()
{
};

BscSerial::BscSerial(uint8_t u8_lSerialNr, uint8_t hwUartNr, uint8_t rx, uint8_t tx, uint8_t txEnRS485pin)
{
  mSerialMutex = xSemaphoreCreateMutex();
  u8_mHwUartNr = hwUartNr;
  u8_mSerialNr = u8_lSerialNr;
  u8_mTxEnRS485pin = txEnRS485pin;

  pinMode(txEnRS485pin, OUTPUT);

  if(hwUartNr==0)
  {
    Serial.begin(9600,SERIAL_8N1,rx,tx);
    stream_mPort=&Serial;
  }
  else if(hwUartNr==1)
  {
    Serial1.begin(9600,SERIAL_8N1,rx,tx);
    stream_mPort=&Serial1;
  }
  else if(hwUartNr==2)
  {
    Serial2.begin(9600,SERIAL_8N1,rx,tx);
    stream_mPort=&Serial2;
  }
}

BscSerial::BscSerial(uint8_t u8_lSerialNr, uint8_t rx, uint8_t tx)
{
  mSerialMutex = xSemaphoreCreateMutex();
  u8_mSerialNr = u8_lSerialNr;
  u8_mTxEnRS485pin = 0;

  SoftwareSerial mySwSerial(rx, tx);
  stream_mPort = &mySwSerial;
  static_cast<SoftwareSerial*>(stream_mPort)->begin(9600);
}


void BscSerial::initSerial()
{
  uint8_t funktionsTyp = WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,u8_mSerialNr,0);
  Serial.printf("initSerial u8_mSerialNr=%i, funktionsTyp=%i\n",u8_mSerialNr,funktionsTyp);
  setReadBmsFunktion(funktionsTyp);
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
      Serial.println("setReadBmsFunktion JBD");
      readBms = &JbdBms_readBmsData;
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
  if(readBms(stream_mPort, u8_mHwUartNr, u8_mTxEnRS485pin)) //Wenn kein Fehler beim Holen der Daten vom BMS
  {
    bmsReadOk=true;
  }
  xSemaphoreGive(mSerialMutex);
  if(bmsReadOk)
  {
    setBmsLastDataMillis(u8_mHwUartNr,millis());
  }
}