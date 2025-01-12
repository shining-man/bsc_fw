// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef EXTDEVICE_I2C_H
#define EXTDEVICE_I2C_H

#include "extension/ExtPeripheral.h"
#include <stdint.h>

#include "inverter/Inverter.hpp"

class ExtInterface_I2C : public ExtPeripheral {
  private:
    static ExtInterface_I2C* instance;
    SemaphoreHandle_t &mutexI2cRx;

    uint8_t mI2cRxBuf[256];

    static void onReceiveWrapper(int len);
    static void onRequestWrapper();
    void onReceiveCb(int len);
    void onRequestCb();

    void i2cWriteBytesUnsecure(uint8_t devAdr, uint8_t *data, uint8_t dataLen);

  protected:
    uint8_t deviceAddress;

    void i2cCyclicRun(Inverter &inverter);

    void I2cRxSemaphoreTake();
    void I2cRxSemaphoreGive();

    void i2cWriteRegister(uint8_t u8_i2cDevAdr, uint8_t u8_reg, uint8_t u8_data);
    void i2cWriteBytes(uint8_t devAdr, uint8_t *data, uint8_t dataLen);
    void i2cReadBytes(uint8_t *data, uint8_t dataLen);
    uint8_t i2cRequest(uint8_t devAdr, uint8_t dataLen);
    bool getDataFromExtension(uint8_t i2cAdr, uint8_t data0, uint8_t data1, uint8_t bmsNr, uint8_t *rxData, uint8_t rxLen);
    bool getDataFromExtensionDelay(uint8_t i2cAdr, uint8_t data0, uint8_t data1, uint8_t bmsNr, uint8_t *rxData, uint8_t rxLen, uint8_t delay);

    void i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, const void *dataAdr, uint8_t dataLen);
    void i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, std::string data, uint8_t dataLen);
    void i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, int16_t data);

    uint8_t isDeviceAvailable(uint8_t devAdresse);

  public:
    ExtInterface_I2C(uint8_t address, SemaphoreHandle_t & lMutexI2cRx);


  protected:
    //void writeByte(uint8_t reg, uint8_t data);
    //uint8_t readByte(uint8_t reg);
};

#endif