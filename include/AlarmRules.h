// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef ALARMRULES_H
#define ALARMRULES_H

#include "Arduino.h"
#include "WebSettings.h"
#include "BleHandler.h"
#include "defines.h"


void initAlarmRules();
void runAlarmRules();
void changeAlarmSettings();

bool getAlarm(uint8_t alarmNr);
uint16_t getAlarm();
bool isTriggerActive(uint16_t paramId, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);

#endif