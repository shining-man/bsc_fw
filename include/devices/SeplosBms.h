// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SEPLOSBMS_H
#define SEPLOSBMS_H

#include "Arduino.h"
#include "defines.h"

#define SEPLOSBMS_MAX_ANSWER_LEN   0xff

bool SeplosBms_readBmsData(Stream *port, uint8_t devNrv, uint8_t txEnRS485pin);

#endif 
