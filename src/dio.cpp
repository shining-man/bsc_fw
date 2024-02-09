// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "dio.h"
#include "defines.h"


RTC_DATA_ATTR uint8_t doOutData;
static uint8_t u8_mHwVersion;

#ifndef DEBUG_JTAG
SPIClass * hspi = NULL;

void initDio(bool restore)
{
  u8_mHwVersion=0;

  pinMode(IO_DO_PL, OUTPUT);
  pinMode(H_CLK, OUTPUT);

  digitalWrite(IO_DO_PL, LOW);
  digitalWrite(H_CLK, HIGH);

  if(restore==false) doOutData = 0;

  hspi = new SPIClass(HSPI);
  hspi->begin(H_CLK, H_MISO, H_MOSI);

  dioRwInOut();

  u8_mHwVersion = ((dioRwInOut()&0xE0)>>5);

  if(u8_mHwVersion>0)
  {
    pinMode(GPIO_LED1_HW1, OUTPUT);
  }
}

void setDoData(uint8_t data)
{
    doOutData = data;
}

uint8_t getDoData()
{
  return doOutData;
}

//Eingänge lesen u. Ausgänge schreiben
uint8_t dioRwInOut()
{
  uint8_t receivedVal = 0;

  //Eingänge ins Eingangsregister des HC165 laden (High-Flanke)
  digitalWrite(IO_DO_PL, LOW);
  delayMicroseconds(5);
  digitalWrite(IO_DO_PL, HIGH);
  delayMicroseconds(5);

  hspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  receivedVal = hspi->transfer(doOutData);
  hspi->endTransaction();

  //Ausgänge setzen HC595 (High-Flanke)
  digitalWrite(IO_DO_PL, LOW);
  delayMicroseconds(5);
  digitalWrite(IO_DO_PL, HIGH);

  return receivedVal;
}

uint8_t getHwVersion()
{
  return u8_mHwVersion;
}

#else

void initDio(bool restore)
{
  ;
}

void setDoData(uint8_t data)
{
    doOutData = data;
}

uint8_t getDoData()
{
  return doOutData;
}

//Eingänge lesen u. Ausgänge schreiben
uint8_t dioRwInOut()
{
  return 0;
}

uint8_t getHwVersion()
{
  return 0xff;
}
#endif