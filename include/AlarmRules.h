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

class Inverter;

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
#define ALARM_VIRTUAL_TRIGGER 10

static const char* ALARM_CAUSE_TEXT[] = {
  "DigitalInput",               //  0
  "BMS_No_Data",                //  1
  "CellVoltage",                //  2
  "TotalVoltage_Min",           //  3
  "TotalVoltage_Max",           //  4
  "Temperature",                //  5
  "OneWire_Sense_Err",          //  6
  "CellVoltage_Plausibility",   //  7
  "SoC",                        //  8
  "Fan",                        //  9
  "vTrigger"                    // 10
};

void initAlarmRules(Inverter &inverter);
void runAlarmRules(Inverter &inverter);
void changeAlarmSettings();

bool getAlarm(uint8_t alarmNr);
uint16_t getAlarm();
bool isTriggerActive(uint16_t paramId, uint8_t groupNr, uint8_t dataType);

bool setVirtualTrigger(uint8_t triggerNr, bool val);

#endif