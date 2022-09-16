// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DIO_H
#define DIO_H

#include <Arduino.h>
#include <SPI.h>

#define H_CLK       14
#define H_MOSI      13
#define H_MISO      12
#define IO_DO_PL    26
#define IO_DI_PL    26 //27




  
void    initDio(bool restore);
void    setDoData(uint8_t data);
uint8_t dioRwInOut();
  






#endif