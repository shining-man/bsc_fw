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

#define ALARM_CAUSE_DI 0
#define ALARM_CAUSE_BMS_NO_DATA 1
#define ALARM_CAUSE_BMS_CELL_VOLTAGE 2
#define ALARM_CAUSE_BMS_TOTAL_VOLTAGE_MIN 3
#define ALARM_CAUSE_BMS_TOTAL_VOLTAGE_MAX 4
#define ALARM_CAUSE_TEMPERATUR 5
#define ALARM_CAUSE_OW_SENSOR_ERR 6
#define ALARM_CAUSE_CELL_VOLTAGE_PLAUSIBILITY 7
#define ALARM_CAUSE_SOC 8
#define ALARM_CAUSE_FAN 9

void initAlarmRules();
void runAlarmRules();
void changeAlarmSettings();

bool getAlarm(uint8_t alarmNr);
uint16_t getAlarm();
bool isTriggerActive(uint16_t paramId, uint8_t groupNr, uint8_t dataType);

#endif