// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JK_INVERTER_BMS_H
#define JK_INVERTER_BMS_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define JK_INVERTER_BMS_MAX_ANSWER_LEN   280


bool JkInverterBms_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
