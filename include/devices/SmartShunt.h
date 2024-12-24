// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SMARTSHUNT_H
#define SMARTSHUNT_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define SMARTSHUNT_ID_MAIN_VOLTAGE          0xED8D
#define SMARTSHUNT_ID_CURRENT               0xED8F
#define SMARTSHUNT_ID_POWER                 0xED8E
#define SMARTSHUNT_ID_CONSUMED_AH           0xEEFF
#define SMARTSHUNT_ID_SOC                   0x0FFF
#define SMARTSHUNT_ID_TIME_TO_GO            0x0FFE
#define SMARTSHUNT_ID_CYCLE                 0x0303
#define SMARTSHUNT_ID_TOTAL_VOLT_MIN        0x0306
#define SMARTSHUNT_ID_TOTAL_VOLT_MAX        0x0307
#define SMARTSHUNT_ID_TIME_SINCE_FULL       0x0308
#define SMARTSHUNT_ID_SOC_SYNC_COUNT        0x0309
#define SMARTSHUNT_ID_VOLT_MIN_COUNT        0x030A
#define SMARTSHUNT_ID_TOTAL_VOLT_MAX_COUNT  0x030B
#define SMARTSHUNT_ID_AMOUNT_DCH_ENERGY     0x0310
#define SMARTSHUNT_ID_AMOUNT_CH_ENERGY      0x0311

#define SMARTSHUNT_MAX_ANSWER_LEN   0x20

bool SmartShunt_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
