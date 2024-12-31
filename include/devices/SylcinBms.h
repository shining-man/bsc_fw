// Copyright (c) 2023 nsc2001
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SYLCINBMS_H
#define SYLCINBMS_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define SYLCINBMS_MAX_ANSWER_LEN   0xff

bool SylcinBms_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
