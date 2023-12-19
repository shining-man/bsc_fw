// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>

#define CAN_BMS_COMMUNICATION_TIMEOUT 5000

struct inverterData_s
{
  bool       noBatteryPackOnline;
  int16_t    inverterVoltage;
  int16_t    inverterCurrent;
  uint16_t   inverterSoc;
  int16_t    inverterChargeCurrent;
  int16_t    inverterDischargeCurrent;

  //Ströme von der Ladestromregelung
  int16_t calcChargeCurrentCellVoltage;
  int16_t calcChargeCurrentSoc;
  int16_t calcChargeCurrentCelldrift;
  int16_t calcChargeCurrentCutOff;
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