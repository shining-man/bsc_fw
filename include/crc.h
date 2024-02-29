// Copyright (c) 2023 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef INC_CRC_H_
#define INC_CRC_H_

#include <stdint.h>

uint16_t crc16 (uint8_t *nData, uint16_t wLength);
uint16_t crc16 (uint16_t crcIn, uint8_t *nData, uint16_t wLength);

uint32_t calcCrc32(uint8_t* pData, uint32_t DataLength);
uint32_t calcCrc32(uint32_t crcIn, uint8_t* pData, uint32_t DataLength);

#endif
