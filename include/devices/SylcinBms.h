// Copyright (c) 2023 nsc2001
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SYLCINBMS_H
#define SYLCINBMS_H

#include "Arduino.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define SYLCINBMS_MAX_ANSWER_LEN   0xff

bool SylcinBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData);

#endif 
