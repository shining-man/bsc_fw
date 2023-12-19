// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DIO_H
#define DIO_H

#include <Arduino.h>
#include <SPI.h>


void    initDio(bool restore);
void    setDoData(uint8_t data);
uint8_t getDoData();
uint8_t dioRwInOut();
uint8_t getHwVersion();

#endif