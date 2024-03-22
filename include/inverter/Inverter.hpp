// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include <cstdint>
#ifndef PIO_UNIT_TESTING
#include "WebSettings.h"
#endif
#include "defines.h"
#include "BmsData.h"
#ifndef PIO_UNIT_TESTING
#include "Ow.h"
#include "mqtt_t.h"
#include "log.h"
#include "AlarmRules.h"
#endif


class Inverter {
public:
    Inverter();
    ~Inverter();

    struct inverterData_s
    {
        // Ausgewählte BMS Quellen
        uint8_t u8_bmsDatasource;
        uint16_t u16_bmsDatasourceAdd;

        // Wechselrichterdaten
        bool       noBatteryPackOnline;
        int16_t    batteryVoltage;
        int16_t    batteryCurrent;
        int16_t    batteryTemperatur;
        uint16_t   inverterChargeVoltage;
        uint16_t   inverterSoc;
        int16_t    inverterChargeCurrent;
        int16_t    inverterDischargeCurrent;

        // Ströme von der Ladestromregelung
        int16_t calcChargeCurrentCellVoltage;
        int16_t calcChargeCurrentSoc;
        int16_t calcChargeCurrentCelldrift;
        int16_t calcChargeCurrentCutOff;

        // Entladeströme von der Regelung
        int16_t calcDischargeCurrentCellVoltage;

        // Charge current cut off
        uint16_t u16_mChargeCurrentCutOfTimer;

        // Wenn Zellspannung kleiner x mV wird SoC auf x% setzen
        uint8_t  u8_mSocZellspannungState;
        uint16_t u16_mSocZellspannungSperrzeitTimer;
    };

    void inverterInit();
    void loadIverterSettings();
    void cyclicRun();

    void inverterDataSemaphoreTake();
    void inverterDataSemaphoreGive();
    struct inverterData_s *getInverterData();

    void setChargeCurrentToZero(bool val);
    void setDischargeCurrentToZero(bool val);
    void setSocToFull(bool val);

    uint16_t getAktualChargeCurrentSoll();


private:
    static const char *TAG;

    SemaphoreHandle_t mInverterDataMutex;
    struct inverterData_s inverterData;

    uint8_t u8_mMqttTxTimer=0;
    uint8_t u8_mSelCanInverter;

    bool alarmSetChargeCurrentToZero;
    bool alarmSetDischargeCurrentToZero;
    bool alarmSetSocToFull;


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
    };

    struct data356
    {
    int16_t voltage;
    int16_t current;
    int16_t temperature;
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


    void readCanMessages();
    void sendBmsCanMessages();

    void getInverterValues();
    void sendMqttMsg();


    void sendCanMsg_ChgVoltCur_DisChgCur_351();
    void sendCanMsg_soc_soh_355();
    void sendCanMsg_Battery_Voltage_Current_Temp_356();
    void sendCanMsg_Alarm_359();
    void sendCanMsg_Alarm_35a();
    void sendCanMsg_hostname_35e_370_371();
    void sendCanMsg_version_35f();
    void sendCanMsg_battery_modules_372();
    void sendCanMsg_min_max_values_373_376_377();
    void sendCanMsg_minCellVoltage_text_374();
    void sendCanMsg_maxCellVoltage_text_375();

    void sendCanMsg(uint32_t identifier, uint8_t *buffer, uint8_t length);
    void sendExtendedCanMsgTemp();
    void sendExtendedCanMsgBmsData();

    void onCanReceive(int packetSize);


};
