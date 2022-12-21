// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JKBMSV13_H
#define JKBMSV13_H

#include "Arduino.h"
#include "defines.h"

#define JKBMSV13_MAX_ANSWER_LEN   70


bool JkBmsV13_readBmsData(Stream *port, uint8_t devNrv, uint8_t txEnRS485pin);


#endif 
