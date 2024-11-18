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

    enum e_stateFloat {FLOAT_VOLTAGE, ABSORPTION_VOLTAGE, ABSORPTION_VOLTAGE_AUTOBALANCER};

    struct inverterData_s
    {
        // Ausgewählte BMS Quellen
        uint8_t  bmsDatasource;
        uint32_t bmsDatasourceAdd;

        // Wechselrichterdaten
        bool     noBatteryPackOnline;
        int16_t  batteryVoltage;           // Faktor: 100
        int16_t  batteryCurrent;           // Faktor: 10
        int16_t  batteryTemperatur;
        uint16_t inverterChargeVoltage;    // Faktor: 10
        uint16_t inverterSoc;
        int16_t  inverterChargeCurrent;    // Faktor: 10
        int16_t  inverterDischargeCurrent; // Faktor: 10

        // Ströme von der Ladestromregelung
        int16_t  calcChargeCurrentCellVoltage;
        int16_t  calcChargeCurrentSoc;
        int16_t  calcChargeCurrentCelldrift;
        int16_t  calcChargeCurrentCutOff;

        // Entladeströme von der Regelung
        int16_t  calcDischargeCurrentCellVoltage;

        // Charge current cut off
        uint16_t mChargeCurrentCutOfTimer;
        int16_t  mChargeCurrentCutOffMittelwert;
        uint16_t mChargeCurrentCutOffMittelwertCounter;

        // Wenn Zellspannung kleiner x mV wird SoC auf x% setzen
        uint8_t  u8_mSocZellspannungState;
        uint16_t u16_mSocZellspannungSperrzeitTimer;

        // Autobalance
        uint8_t  mStateAutobalance;
        uint32_t lastAutobalanceRun;
        uint32_t autobalanceStartTime;
        uint32_t autobalanceVoltageErreichtTime;

        //
        e_stateFloat floatState;
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

    uint8_t getAutobalState();
    e_stateFloat getChargeVoltageState(); // Absorbtion, Floate

private:
    static const char *TAG;



    SemaphoreHandle_t mInverterDataMutex;
    struct inverterData_s inverterData;

    uint8_t u8_mMqttTxTimer=0;

    bool alarmSetChargeCurrentToZero;
    bool alarmSetDischargeCurrentToZero;
    bool alarmSetSocToFull;

    void getInverterValues();
    void sendMqttMsg();
};
