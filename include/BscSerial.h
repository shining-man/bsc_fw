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

  void initSerial();
  void stopCyclicRun(bool state);
  void setHwSerial(uint8_t u8_devNr, uint32_t baudrate);
  void setSoftSerial(uint8_t u8_devNr, uint32_t baudrate);
  void setSerialBaudrate(uint8_t u8_devNr);
  void setSerialRxBufferSize(uint8_t u8_devNr, uint16_t rxBufSize);
  
  void cyclicRun();

  void setReadBmsFunktion(uint8_t u8_devNr, uint8_t funktionsTyp);
  
  
private:
  SemaphoreHandle_t mSerialMutex = NULL;

  void setSerialBaudrate(uint8_t u8_devNr, uint32_t baudrate);

};


#endif
