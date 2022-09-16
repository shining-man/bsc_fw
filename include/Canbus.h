// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef CANBUS_H
#define CANBUS_H

#include <Arduino.h>

void canSetup();
void loadCanSettings();
void canTxCyclicRun();
void canSetChargeCurrentToZero(bool);
void canSetDischargeCurrentToZero(bool);
void canSetSocToFull(bool);
uint16_t getAktualChargeCurrentSoll();

#endif