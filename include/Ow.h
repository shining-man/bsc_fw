// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef OW_H
#define OW_H

#include <Arduino.h>

#define DEVICE_DISCONNECTED_C_U16 DEVICE_DISCONNECTED_C*100

uint8_t getSensorAdrFromParams();
void    takeOwSensorAddress();
void    owSetup();

void    owCyclicRun();
float   owGetTemp(uint8_t sensNr);
uint8_t owGetAllSensorError();
String  getSensorAdr();

#endif