// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SEPLOSBMSV3_H
#define SEPLOSBMSV3_H

#include "Arduino.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define SEPLOSBMS_MAX_ANSWER_LEN   0xff


#define TOTAL_VOLTAGE    0x1000
#define TOTAL_CURRENT    0x1001
#define SEPLOSV3_SOC     0x1005
#define AVG_CELL_VOLTAGE 0x1008
#define MAX_CELL_VOLTAGE 0x100A
#define MIN_CELL_VOLTAGE 0x100B

#define CELLVOLTAGE_1    0x1100
#define TEMPERATURE_1    0x1110

#define FET_STATE        0x1278

bool SeplosBmsV3_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData);

#endif
