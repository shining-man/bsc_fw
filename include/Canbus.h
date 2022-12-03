// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>

struct inverterData_s
{
  int16_t    inverterVoltage;
  int16_t    inverterCurrent;
  uint16_t   inverterSoc;
  int16_t    inverterChargeCurrent;
  int16_t    inverterDischargeCurrent;
};


void canSetup();
void loadCanSettings();
void canTxCyclicRun();
void canSetChargeCurrentToZero(bool);
void canSetDischargeCurrentToZero(bool);
void canSetSocToFull(bool);
uint16_t getAktualChargeCurrentSoll();

struct inverterData_s * getInverterData();

void inverterDataSemaphoreTake();
void inverterDataSemaphoreGive();

#endif