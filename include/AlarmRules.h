// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef AlarmRules_h
#define AlarmRules_h


#include "Arduino.h"
#include "WebSettings.h"
#include "BleHandler.h"
#include "defines.h"

#define CNT_ALARMS        10
#define CNT_DIGITALOUT     7
#define CNT_DIGITALIN      4


void initAlarmRules();
void runAlarmRules();

bool getAlarm(uint8_t alarmNr);

#endif