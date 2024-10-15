// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"

namespace nsCanbus
{
  class Canbus
  {
  public:
    Canbus(); // Konstruktor
    ~Canbus(); // Dekonstruktor

    void init();
    void readCanMessages(Inverter::inverterData_s &inverterData);
    void sendBmsCanMessages(Inverter::inverterData_s &inverterData);

  private:
    static const char *TAG;

    struct data351
    {
      uint16_t chargevoltagelimit;   // CVL
      int16_t  maxchargecurrent;     // CCL
      int16_t  maxDischargeCurrent;  // DCL
      uint16_t dischargevoltage;     // DVL
    };

    struct data355
    {
      uint16_t soc; //state of charge
      uint16_t soh; //state of health
      uint16_t notUsed0;
      uint16_t notUsed1;
    };

    struct data356
    {
      int16_t voltage;
      int16_t current;
      int16_t temperature;
      uint16_t notUsed0;
    };

    struct data35a
    {
      uint8_t u8_b0;
      uint8_t u8_b1;
      uint8_t u8_b2;
      uint8_t u8_b3;
      uint8_t u8_b4;
      uint8_t u8_b5;
      uint8_t u8_b6;
      uint8_t u8_b7;
    };

    struct data373
    {
      uint16_t minCellColtage;
      uint16_t maxCellVoltage;
      uint16_t minCellTemp;
      uint16_t maxCellTemp;
    };

    void sendCanMsg_ChgVoltCur_DisChgCur_351(Inverter::inverterData_s &inverterData, uint8_t canDevice);
    void sendCanMsg_soc_soh_355(Inverter::inverterData_s &inverterData, uint8_t canDevice);
    void sendCanMsg_Battery_Voltage_Current_Temp_356(Inverter::inverterData_s &inverterData, uint8_t canDevice);
    void sendCanMsg_Alarm_359(Inverter::inverterData_s &inverterData);
    void sendCanMsg_Alarm_35a(Inverter::inverterData_s &inverterData);
    void sendCanMsg_hostname_35e_370_371();
    void sendCanMsg_version_35f(Inverter::inverterData_s &inverterData, uint8_t canDevice);
    void sendCanMsg_battery_modules_372(Inverter::inverterData_s &inverterData);
    void sendCanMsg_min_max_values_373_376_377(Inverter::inverterData_s &inverterData);
    void sendCanMsg_minCellVoltage_text_374(Inverter::inverterData_s &inverterData);
    void sendCanMsg_maxCellVoltage_text_375(Inverter::inverterData_s &inverterData);
    void sendCanMsg_InstalledCapacity_379(Inverter::inverterData_s &inverterData);

    void sendCanMsg(uint32_t identifier, uint8_t *buffer, uint8_t length);
    void sendExtendedCanMsgTemp();
    void sendExtendedCanMsgBmsData();

    void onCanReceive(int packetSize);
    void sendCanMsg_productinfo_382();//Anpassung SolarEdgeRWS
  };
}