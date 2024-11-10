// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef NEEYBALANCER_H
#define NEEYBALANCER_H

#include "defines.h"

//
#define NEEYBAL4A_CMD_WRITE                          0x05

#define NEEYBAL4A_FUNC_SETTING_CELLS                 0x01
#define NEEYBAL4A_FUNC_SETTING_START_VOL             0x02
#define NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT       0x03
#define NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE         0x04
#define NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE  0x17
#define NEEYBAL4A_FUNC_SETTING_BAT_CAP               0x16
#define NEEYBAL4A_FUNC_SETTING_BAT_TYPE              0x15
#define NEEYBAL4A_FUNC_SETTING_BUZZER_MODE           0x14
#define NEEYBAL4A_FUNC_SETTING_BALLANCER_ON_OFF      0x0D


class NeeyBalancer {
public:
  NeeyBalancer();

  static float    neeyGetReadbackDataFloat(uint8_t devNr, uint8_t dataType);
  static uint32_t neeyGetReadbackDataInt(uint8_t devNr, uint8_t dataType);
  static void     getNeeyReadbackDataAsString(std::string &value);
};

#endif
