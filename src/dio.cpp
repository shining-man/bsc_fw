// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "dio.h"


/*
HC595:
STore Clock (IO_DO_PL): Der Wechsel von Low auf High kopiert den Inhalt des Shift-Registers 
in das Ausgaberegister bzw. Ausgangsregister oder auch Speicherregister
*/

SPIClass * hspi = NULL;
RTC_DATA_ATTR uint8_t doOutData;
static uint8_t u8_mHwVersion;

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
}

void setDoData(uint8_t data)
{
    doOutData = data;
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