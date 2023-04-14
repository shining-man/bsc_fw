// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SEPLOSBMS_H
#define SEPLOSBMS_H

#include "Arduino.h"
#include "defines.h"

#define SEPLOSBMS_MAX_ANSWER_LEN   0xff

bool SeplosBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), uint8_t u8_addData);

#endif 
