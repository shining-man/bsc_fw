// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JKBMSV13_H
#define JKBMSV13_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define JKBMSV13_MAX_ANSWER_LEN   70


bool JkBmsV13_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);


#endif
