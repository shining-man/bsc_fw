// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef INC_MDOBUSRTU_H
#define INC_MDOBUSRTU_H

#include "Arduino.h"
#include "BscSerial.h"

namespace modbusrtu
{
    class ModbusRTU
    {
    public:
        enum class fCode : uint8_t
        {
            READ_COIL_01       = 0x01,
            //WRITE_COIL_0F      = 0x0F,
            READ_CMD_04        = 0x04,
            //WRITE_CMD_10       = 0x10

            READ_CMD_JK        = 0x03
        };

        // Konstruktor
        ModbusRTU(Stream *port, uint8_t devNr);

        // Destruktor
        ~ModbusRTU();

        bool readData(BscSerial *bscSerial, uint8_t addr, fCode cmd, uint16_t startRegister, uint16_t len, uint8_t *retData);

        bool     getBitValue(uint16_t address, uint8_t b);
        uint8_t  getU8Value(uint16_t address);
        uint16_t getU16Value(uint16_t address);
        int16_t  getI16Value(uint16_t address);

        uint8_t  getU8ValueByOffset(uint16_t offset);
        int8_t   getI8ValueByOffset(uint16_t offset);
        uint16_t getU16ValueByOffset(uint16_t offset);
        int16_t  getI16ValueByOffset(uint16_t offset);
        uint32_t getU32ValueByOffset(uint16_t offset);
        int32_t  getI32ValueByOffset(uint16_t offset);
        bool     getBitValueByOffset(uint16_t offset, uint8_t b);


    private:
        Stream *mPort;
        uint8_t mSerialPortNr;

        uint16_t mStartRegAdr;
        uint8_t *mRetData;
        uint8_t retDataLen;

        enum class modbusRxState : uint8_t
        {
            WAIT_START     = 0x0,
            RECV_DATA      = 0x1
        };

        void buildSendMsg(BscSerial *bscSerial, uint8_t addr, fCode cmd, uint16_t startRegister, uint16_t len);
        bool readSerialData();
        void errorNoData(uint16_t offset);
    };

} // namespace modbusrtu

#endif