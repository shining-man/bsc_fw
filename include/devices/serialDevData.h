// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SERIALDEVDATA_H
#define SERIALDEVDATA_H

#include <Arduino.h>
//#include "defines.h"

struct serialDevData_s
{
  uint8_t  u8_deviceNr;
  uint8_t  u8_NumberOfDevices;
  uint8_t  u8_BmsDataAdr;
  bool     bo_sendMqttMsg;

  bool               bo_writeData;    
  serialDataRwTyp_e  rwDataTyp;
  uint8_t            rwDataLen;
  uint8_t            *rwData;
};

#endif 
