// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef SEPLOSBMSV3_H
#define SEPLOSBMSV3_H

#include "Arduino.h"
#include "BscSerial.h"
#include "defines.h"
#include "devices/serialDevData.h"

#define SEPLOSBMS_MAX_ANSWER_LEN   0xff


#define SEPLOS3_TOTAL_VOLTAGE         0x1000
#define SEPLOS3_TOTAL_CURRENT         0x1001
#define SEPLOS3_SEPLOSV3_SOC          0x1005
#define SEPLOS3_AVG_CELL_VOLTAGE      0x1008
#define SEPLOS3_MAX_CELL_VOLTAGE      0x100A
#define SEPLOS3_MIN_CELL_VOLTAGE      0x100B

#define SEPLOS3_CELLVOLTAGE_1         0x1100
#define SEPLOS3_TEMPERATURE_1         0x1110
#define SEPLOS3_TEMPERATURE_5         0x1118


#define SEPLOS3_PIC_BASE_ADR          0x1200
#define SEPLOS3_EQUALIZATION_08_01    ((0x1230 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x1206 // 0x1230
#define SEPLOS3_EQUALIZATION_16_09    ((0x1238 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x1207 // 0x1238
#define SEPLOS3_VOLATGE_EVENT         ((0x1248 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x1209 // 0x1248 // Voltage event code (TB02)
#define SEPLOS3_CELLS_TEMPERATURE     ((0x1250 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x120A // 0x1250 // Cells Temperature event code (TB03)
#define SEPLOS3_ENVIRONMENT_TEMP      ((0x1258 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x120B // 0x1258 // Environment and power Temperature event code (TB04)
#define SEPLOS3_CURRENT_EVENT_1       ((0x1260 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x120B // 0x1260 // Current event code1 (TB05)
#define SEPLOS3_CURRENT_EVENT_2       ((0x1268 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x120D // 0x1268 // Current event code2 (TB16)
#define SEPLOS3_RESIDUAL_CAPACITY     ((0x1270 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x120E // 0x1270 // The residual capacity code (TB06)
#define SEPLOS3_FET_EVENT             ((0x1278 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x120F // 0x1278 // The FET event code (TB07)
//#define SEPLOS3_BATTERY_EQUALIZATION // 0x1280 // Battery equalization state code (TB08)
//#define SEPLOS3_SYSTEM_STATE         // 0x1240 // System state code (TB09)
#define SEPLOS3_FARD_FAULT            ((0x1288 - SEPLOS3_PIC_BASE_ADR) / 0x8) + SEPLOS3_PIC_BASE_ADR //0x1211 // 0x1288 // Hard fault event code (TB15)


bool SeplosBmsV3_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData);

#endif
