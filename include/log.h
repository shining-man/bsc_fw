// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef LOG_H
#define LOG_H

#include <Arduino.h>
  
void debugInit();

#ifdef DEBUG_ON_FS
void writeLogToFS();
#endif

#endif