// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef OW_H
#define OW_H

#include <Arduino.h>
  
uint8_t getSensorAdrFromParams();
void    takeOwSensorAddress();
void    owSetup();

void    owCyclicRun();
float   owGetTemp(uint8_t sensNr);
String  getSensorAdr();

#endif