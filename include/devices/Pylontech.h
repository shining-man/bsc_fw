// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef PYLONTECH_H
#define PYLONTECH_H

#include "Arduino.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define PYLONTECH_MAX_ANSWER_LEN   0xff

bool Pylontech_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData);

#endif
