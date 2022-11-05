// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
  
void debugInit();
void debugPrintTest();

//print
void debugPrint(const char str[]);
size_t debugPrint(const String &s);
void debugPrint(int n);

//println
void debugPrintln(const char str[]);
void debugPrintln(const String &s);
void debugPrintln(int n);

//printf
size_t debugPrintf(const char *format, ...);
  
#endif