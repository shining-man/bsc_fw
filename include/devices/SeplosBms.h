// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SEPLOSBMS_H
#define SEPLOSBMS_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define SEPLOSBMS_MAX_ANSWER_LEN   0xff

bool SeplosBms_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
