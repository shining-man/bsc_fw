// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SMARTSHUNT_H
#define SMARTSHUNT_H

#include "Arduino.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define RX_VAL_SOC   1
#define RX_VAL_U     2
#define RX_VAL_I     4
#define RX_VAL_OK    7   // Summe aller RX_VAL_*





bool SmartShunt_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData);

#endif
