// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef NEEY_SERIAL_H
#define NEEY_SERIAL_H

#include "Arduino.h"
#include "defines.h"
#include "devices/serialDevData.h"

bool NeeySerial_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData);

#endif
