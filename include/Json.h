// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef Json_h
#define Json_h

#include <Arduino.h>

class Json {
public:
  Json();
  
  uint16_t getArraySize(const String *json, long startPos);
  bool getValue(const String *json, int idx, String name, uint32_t searchStartPos, String& retValue, uint32_t& arrayStart);

private:
  bool jsonIndexPos_Array(const String *json, int idx, long &startPos, long& endPos);

};


#endif