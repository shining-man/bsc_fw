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

#define smartshunt_id_main_voltage          0xED8D
#define smartshunt_id_current               0xED8F
#define smartshunt_id_power                 0xED8E
#define smartshunt_id_Consumed_Ah           0xEEFF
#define smartshunt_id_SOC                   0x0FFF
#define smartshunt_id_TIME_TO_GO            0x0FFE
#define smartshunt_id_CYCLE                 0x0303
#define smartshunt_id_TOTAL_VOLT_MIN        0x0306
#define smartshunt_id_TOTAL_VOLT_MAX        0x0307
#define smartshunt_id_TIME_SINCE_FULL       0x0308
#define smartshunt_id_SOC_SYNC_COUNT        0x0309
#define smartshunt_id_VOLT_MIN_COUNT        0x030A
#define smartshunt_id_TOTAL_VOLT_MAX_COUNT  0x030B
#define smartshunt_id_AMOUNT_DCH_ENERGY     0x0310
#define smartshunt_id_AMOUNT_CH_ENERGY      0x0311

#define SMARTSHUNT_MAX_ANSWER_LEN   0x20

bool SmartShunt_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
