// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"

class BmsDataUtils
{
public:
    BmsDataUtils(); // Konstruktor
    ~BmsDataUtils(); // Dekonstruktor

    static uint8_t getNumberOfBatteryModules(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd);
    static void    getNumberOfBatteryModulesOnline(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd, 
      uint16_t &moduleOnline, uint16_t &moduleOffline);
    static uint8_t getNumberOfBatteryModulesCharge(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd);
    static uint8_t getNumberOfBatteryModulesDischarge(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd);
    static uint16_t getMaxCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd);
    static uint16_t getMaxCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd, uint8_t &BmsNr,uint8_t &CellNr);
    static uint16_t getMinCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd);
    static uint16_t getMinCellSpannungFromBms(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd, uint8_t &BmsNr,uint8_t &CellNr);
    static uint16_t getMaxCellDifferenceFromBms(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd);
    static float getMinCurrentFromBms(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd);
    static void  getMinMaxBatteryTemperature(uint8_t u8_mBmsDatasource, uint32_t mBmsDatasourceAdd, 
      int16_t &tempHigh, int16_t &tempLow, uint8_t &tempLowSensor, uint8_t &tempLowPack, uint8_t &tempHighSensor, uint8_t &tempHighPack);
    static bool isDataDeviceAvailable(Inverter::inverterData_s &inverterData, uint8_t deviceNr);

    static void buildBatteryCellText(char (&buffer)[8], uint8_t batteryNr, uint8_t cellNr);
    static void buildBatteryTempText(char (&buffer)[8], uint8_t batteryNr, uint8_t cellNr);

private:

};
