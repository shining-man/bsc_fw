// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef INC_MDOBUSRTU_H
#define INC_MDOBUSRTU_H

#include "Arduino.h"

namespace modbusrtu
{
    class ModbusRTU
    {
    public:
        enum class fCode : uint8_t
        {
            READ_COIL_01       = 0x01,
            //WRITE_COIL_0F      = 0x0F,
            READ_CMD_04        = 0x04
            //WRITE_CMD_10       = 0x10
        };

        // Konstruktor
        ModbusRTU(Stream *port, void (*callback)(uint8_t, uint8_t), uint8_t devNr);

        // Destruktor
        ~ModbusRTU();

        bool readData(uint8_t addr, fCode cmd, uint16_t startRegister, uint16_t len, uint8_t *retData);

        bool     getBitValue(uint16_t address, uint8_t b);
        uint8_t  getU8Value(uint16_t address);
        uint16_t getU16Value(uint16_t address);
        int16_t  getI16Value(uint16_t address);

    private:
        Stream *mPort;
        void (*mCallback)(uint8_t, uint8_t);
        uint8_t mDevNr;

        uint16_t mStartRegAdr;
        uint8_t *mRetData;
        uint8_t retDataLen;

        enum class modbusRxState : uint8_t
        {
            WAIT_START     = 0x0,
            RECV_DATA      = 0x1
        };

        void buildSendMsg(uint8_t addr, fCode cmd, uint16_t startRegister, uint16_t len);
        bool readSerialData();
    };

} // namespace modbusrtu

#endif