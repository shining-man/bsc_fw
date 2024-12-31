// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef BleHandler_h
#define BleHandler_h

#include "Arduino.h"
#include "WebSettings.h"
#include "defines.h"
#include "BmsData.h"


class BleHandler {
public:
  BleHandler();
  ~BleHandler();

  static void setBalancerState(uint8_t devNr, boolean Alarm);

private:

};

#endif
