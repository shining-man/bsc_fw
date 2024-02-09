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

  uint16_t getArraySize(const char *json, long startPos);
  bool getValue(const char *json, int idx, String name, uint32_t searchStartPos, String& retValue, uint32_t& arrayStart);

private:
  bool jsonIndexPos_Array(const char *json, int idx, long &startPos, long& endPos);

};


#endif