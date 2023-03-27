// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JKBMS_H
#define JKBMS_H

#include "Arduino.h"
#include "defines.h"

#define JKBMS_MAX_ANSWER_LEN   0x200


bool JkBms_readBmsData(Stream *port, uint8_t devNrv, uint8_t txEnRS485pin, uint8_t u8_addData);


#endif 
