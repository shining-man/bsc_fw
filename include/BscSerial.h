// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef BSCSERIAL_H
#define BSCSERIAL_H

#include "Arduino.h"
#include <SoftwareSerial.h>

class BscSerial {
public:
  BscSerial();
  BscSerial(uint8_t u8_lSerialNr, uint8_t uart_nr, uint8_t rx, uint8_t tx, uint8_t txEnRS485pin);
  BscSerial(uint8_t u8_lSerialNr, uint8_t rx, uint8_t tx, uint8_t txEnRS485pin);

  void setHwSerial(uint8_t u8_lSerialNr, uint32_t baudrate, uint8_t hwUartNr, uint8_t rx, uint8_t tx, uint8_t txEnRS485pin);
  void setSoftSerial(uint8_t u8_lSerialNr, uint32_t baudrate, uint8_t rx, uint8_t tx, uint8_t txEnRS485pin);

  void initSerial();

  void cyclicRun();
  void setReadBmsFunktion(uint8_t funktionsTyp);
  
  
private:
  SemaphoreHandle_t mSerialMutex = NULL;

  bool (*readBms)(Stream*, uint8_t, uint8_t) = 0; // Funktionszeiger anlegen, Initialisierung mit 0
  Stream * stream_mPort;
  uint8_t u8_mHwUartNr;
  uint8_t u8_mSerialNr;
  uint8_t u8_mTxEnRS485pin;
  uint8_t u8_mRx;
  uint8_t u8_mTx;

  void setSerialBaudrate(uint32_t baudrate);

};


#endif
