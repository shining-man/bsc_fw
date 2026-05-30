// Copyright (c) 2026
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef FELICITY_LUXY_BMS_H
#define FELICITY_LUXY_BMS_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define FELICITY_LUXY_BMS_MAX_ANSWER_LEN   0xC0

bool FelicityLuxYBms_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
