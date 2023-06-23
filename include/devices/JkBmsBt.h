// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef JKBMSBT_H
#define JKBMSBT_H

#include "Arduino.h"
#include "defines.h"



#define JKBMS_BT_COMMAND_CELL_INFO    0x96
#define JKBMS_BT_COMMAND_DEVICE_INFO  0x97


#define FRAME_VERSION_JK02      1
#define FRAME_VERSION_JK02_32S  2
#define FRAME_VERSION_JK04      3



void    jkBmsBtDevInit(uint8_t devNr);
void    jkBmsBtCopyData(uint8_t devNr, uint8_t frameVersion, uint8_t* pData, size_t length);
uint8_t jkBmsBtCrc(uint8_t *data, uint16_t len);
void    jkBmsBtBuildSendFrame(uint8_t *frame, uint8_t address, uint32_t value, uint8_t length);

void jkBmsBtDecodeCellInfo_jk02(uint8_t devNr, uint8_t* pData, uint8_t frameNr, uint8_t u8_frameVersion);


#endif
