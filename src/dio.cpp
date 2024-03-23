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
  hspi->setDataMode(SPI_MODE3);
  hspi->setBitOrder(MSBFIRST);
  hspi->setFrequency(1000000);

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

  // PL Pin auf LOW
  digitalWrite(IO_DO_PL, LOW);

  //Ausgaänge ins HC595 schreiben
  hspi->transfer(doOutData);

  // SPI Mode wechseln
  hspi->setDataMode(SPI_MODE2);

  // Ausgänge von HC595 übernehmen und HC165 einlesen (HIGH-Flanke)
  digitalWrite(IO_DO_PL, HIGH);

  // Eingänge des HC165 lesen
  receivedVal = hspi->transfer(0);

  // Spi Mode wieder zurück wechseln
  hspi->setDataMode(SPI_MODE3);

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