// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef BSCTIME_H
#define BSCTIME_H

#include "Arduino.h"

void   initTime();
bool   timeRunCyclic(bool bo_resetTimeout);
String getBscTime();
String getBscDateTime();
const char* getBscDateTimeCc();
const char* getBscDateTimeCc2();
uint32_t getEpoch();
uint32_t getDayMinutes();
uint8_t getMinutes();
void   setTimeFromNTP();

#endif