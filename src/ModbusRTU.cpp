// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "ModbusRTU.hpp"
#include <Arduino.h>
#include "defines.h"
#include "crc.h"

namespace modbusrtu
{

static const char *TAG = "MODBUS";

ModbusRTU::ModbusRTU(Stream *port, uint8_t serialPortNr)
{
  mStartRegAdr=0;
  retDataLen=0;

  mPort = port;
  mSerialPortNr = serialPortNr;
}

ModbusRTU::~ModbusRTU()
{
  ;
}

/*static void message2Log(uint8_t * t_message, uint8_t len, uint8_t address)
{
  String recvBytes="";
  uint8_t u8_logByteCount=0;
  BSC_LOGI(TAG,"Dev=%i, RecvBytes=%i",address, len);
  for(uint8_t x=0;x<len;x++)
  {
    u8_logByteCount++;
    recvBytes+="0x";
    recvBytes+=String(t_message[x],16);
    recvBytes+=" ";
    if(u8_logByteCount==20)
    {
      BSC_LOGI(TAG,"%s",recvBytes.c_str());
      recvBytes="";
      u8_logByteCount=0;
    }
  }
  BSC_LOGI(TAG,"%s",recvBytes.c_str());
  //log_print_buf(p_lRecvBytes, u8_lRecvBytesCnt);
}*/

/*
 * Anfrage:
 * ---------------------------
 * Address               ADDR  (1 Byte)
 * Command               CMD   (1 Byte)
 * Beginning register address  (2 Byte)
 * Register number n           (2 Byte)
 * CRC                         (2 Byte)
*/
bool ModbusRTU::readData(BscSerial *bscSerial, uint8_t addr, fCode cmd, uint16_t startRegister, uint16_t len, uint8_t *retData)
{
  mStartRegAdr=startRegister;
  mRetData=retData;

  // send msg
  buildSendMsg(bscSerial, addr, cmd, startRegister, len);

  // wait
  vTaskDelay(25/portTICK_PERIOD_MS);

  // read data
  return readSerialData();
}


void ModbusRTU::buildSendMsg(BscSerial *bscSerial, uint8_t addr, fCode cmd, uint16_t startRegister, uint16_t len)
{
  uint8_t lSendData[8];

  lSendData[0]=addr;                        // ADDR
  lSendData[1]=(uint8_t)cmd;                // CMD
  lSendData[2]=((startRegister>>8)&0xff);   //
  lSendData[3]=(startRegister&0xff);        //
  lSendData[4]=((len >> 8)&0xff);           //
  lSendData[5]=(len&0xff);                  //

  // calc CRC
  uint16_t u16_lCrc = crc16(lSendData, 6);
  lSendData[6]=(u16_lCrc&0xff);        // CRC
  lSendData[7]=((u16_lCrc>>8)&0xff);   // CRC

  //message2Log(lSendData, 8, addr);

  // RX-Buffer leeren
  for(unsigned long clearRxBufTime = millis(); millis()-clearRxBufTime<100;)
  {
    if(mPort->available()) mPort->read();
    else break;
  }

  // send msg
  bscSerial->sendSerialData(mPort, mSerialPortNr, lSendData, 8);
}


bool ModbusRTU::readSerialData()
{
  modbusRxState rxState = modbusRxState::WAIT_START;
  uint8_t address,functionCode;
  uint16_t lCrc;
  uint32_t u32_lStartTime = millis();

  retDataLen=0;

  for(;;)
  {
    //Timeout
    if((millis()-u32_lStartTime)>200)
    {
      BSC_LOGE(TAG,"Timeout: Serial=%i, dataLen=%i, available=%i", mSerialPortNr, retDataLen, mPort->available());
      return false;
    }

    if(rxState==modbusRxState::WAIT_START)
    {
      // Mindestens 4 Bytes müssen verfügbar sein (Adresse, Funktionscode, Längenbytes, ..., CRC)
      if (mPort->available() >= 4)
      {
        // Lesen der Daten vom Modbus-Gerät
        address = mPort->read();
        lCrc = crc16(&address, 1);

        // Function code
        functionCode = mPort->read();
        lCrc = crc16(lCrc, &functionCode, 1);

        // Länge des Datenfelds
        retDataLen = mPort->read();
        lCrc = crc16(lCrc, &retDataLen, 1);

        //BSC_LOGI(TAG,"adr=%i, fCode=%i, len=%i",address,functionCode,retDataLen);
        rxState=modbusRxState::RECV_DATA;
      }
      else vTaskDelay(pdMS_TO_TICKS(25));
    }
    else if(rxState==modbusRxState::RECV_DATA)
    {
      // Überprüfe, ob alle Daten empfangen wurden
      if(mPort->available() >= retDataLen+2)
      {

        for(int i=0; i < retDataLen; i++) mRetData[i]=mPort->read();

        // CRC (2 Bytes)
        uint16_t u16_crcSoll = mPort->read() | (mPort->read()<<8) ;
        lCrc = crc16(lCrc, mRetData, retDataLen);

        if(u16_crcSoll==lCrc) return true;
        else
        {
          BSC_LOGE(TAG,"CRC wrong: soll=%i, ist=%i",u16_crcSoll,lCrc);
          return false;
        }
      }
      else vTaskDelay(pdMS_TO_TICKS(25));
    }
  }

  return false;
}


uint8_t ModbusRTU::getU8Value(uint16_t address)
{
  if(mStartRegAdr>address) return 0;

  uint16_t sb = address-mStartRegAdr;
  if(sb>retDataLen) return 0;

  return mRetData[sb];
}

bool ModbusRTU::getBitValue(uint16_t address, uint8_t b)
{
  if(mStartRegAdr>address) return 0;

  return isBitSet(getU8Value(address),b);
}

uint16_t ModbusRTU::getU16Value(uint16_t address)
{
  if(mStartRegAdr>address) return 0;

  uint16_t sb = (address-mStartRegAdr)*2;
  if(sb>retDataLen) return 0;

  return (mRetData[sb]<<8) | mRetData[sb+1];
}

int16_t ModbusRTU::getI16Value(uint16_t address)
{
  if(mStartRegAdr>address) return 0;

  uint16_t sb = (address-mStartRegAdr)*2;
  if(sb>retDataLen) return 0;

  return (int16_t)((mRetData[sb]<<8) | mRetData[sb+1]);
}



uint8_t ModbusRTU::getU8ValueByOffset(uint16_t offset)
{
  if(offset > retDataLen)
  {
    errorNoData(offset);
    return 0;
  }
  return mRetData[offset];
}

int8_t ModbusRTU::getI8ValueByOffset(uint16_t offset)
{
  return (int8_t)getU8ValueByOffset(offset);
}

uint16_t ModbusRTU::getU16ValueByOffset(uint16_t offset)
{
  if(offset+1 > retDataLen)
  {
    errorNoData(offset+1);
    return 0;
  }
  return (mRetData[offset]<<8) | mRetData[offset+1];
}

int16_t ModbusRTU::getI16ValueByOffset(uint16_t offset)
{
  return (int16_t)getU16ValueByOffset(offset);
}

uint32_t ModbusRTU::getU32ValueByOffset(uint16_t offset)
{
  if(offset+3 > retDataLen)
  {
    errorNoData(offset+3);
    return 0;
  }
  return (mRetData[offset]<<24) | (mRetData[offset+1]<<16) | (mRetData[offset+2]<<8) | mRetData[offset+3];
}

int32_t ModbusRTU::getI32ValueByOffset(uint16_t offset)
{
  return (int32_t)getU32ValueByOffset(offset);
}

bool ModbusRTU::getBitValueByOffset(uint16_t offset, uint8_t b)
{
  if(offset > retDataLen)
  {
    errorNoData(offset);
    return 0;
  }

  return isBitSet(mRetData[offset],b);
}

void ModbusRTU::errorNoData(uint16_t offset)
{
  BSC_LOGE(TAG,"Keine Daten! Lese Byte %i von %i",offset, retDataLen);
}

} // namespace modbusrtu