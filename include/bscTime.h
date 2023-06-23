// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef BSCTIME_H
#define BSCTIME_H

#include "Arduino.h"

void   initTime();
void   timeRunCyclic(bool bo_resetTimeout);
String getBscTime();
String getBscDateTime();
const char* getBscDateTimeCc();
void   setTimeFromNTP();

#endif