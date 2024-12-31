// Copyright (c) 2023 dirk
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef GOBELBMS_PC200_H
#define GOBELBMS_PC200_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define GOBELBMS_MAX_ANSWER_LEN   0x200

bool GobelBmsPC200_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
