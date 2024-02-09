// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DALYBMS_H
#define DALYBMS_H

#include "Arduino.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define DALY_FRAME_SIZE                     13
#define DALAY_START_BYTE                  0xA5
#define DALAY_BMS_ADRESS                  0x40

#define DALY_REQUEST_BATTERY_SOC          0x90
#define DALY_REQUEST_MIN_MAX_VOLTAGE      0x91
#define DALY_REQUEST_MIN_MAX_TEMPERATURE  0x92
#define DALY_REQUEST_MOS                  0x93
#define DALY_REQUEST_STATUS               0x94
#define DALY_REQUEST_CELL_VOLTAGE         0x95
#define DALY_REQUEST_TEMPERATURE          0x96
#define DALY_REQUEST_BALLANCER            0x97
#define DALY_REQUEST_FAILURE              0x98

#define DALY_TEMPERATURE_OFFSET             40
#define DALY_CURRENT_OFFSET              30000


bool DalyBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData);


#endif
