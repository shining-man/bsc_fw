// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "extension/devices/ExtSerial.h"
#include "mcp23017.h"

static const char *TAG = "EXT_SERIAL";

ExtSerial::ExtSerial(uint8_t address) : ExtInterface_I2C(address) { }

ExtSerial::~ExtSerial() {}


void ExtSerial::initialize()
{
  uint8_t u8_lErr = isDeviceAvailable(deviceAddress);
  if (u8_lErr == 0)
  {
    setEnabled(true);
    BSC_LOGI(TAG,"Serial Ext. found");
    
    /*
    A0	A1	A2	I2C-Adresse
    0	  0	  0	  0x20 (32)
    1	  0	  0	  0x21 (33)
    0	  1	  0	  0x22 (34)
    1	  1	  0	  0x23 (35)
    0	  0	  1	  0x24 (36)
    1	  0	  1	  0x25 (37)
    0	  1	  1	  0x26 (38)
    1	  1	  1	  0x27 (39)
    */
    i2cWriteRegister(deviceAddress, MCP23017_IODIRA, 0x0);
    i2cWriteRegister(deviceAddress, MCP23017_IODIRB, 0x0);

    i2cWriteRegister(deviceAddress, MCP23017_GPIOA, 0xAA);
    i2cWriteRegister(deviceAddress, MCP23017_GPIOB, 0xAA);
  }
  else
  {
    setEnabled(false);
    BSC_LOGI(TAG,"Serial Ext. not found (%i)",u8_lErr);
  }
}


void ExtSerial::extSerialSetEnable(uint8_t u8_serialDevNr, serialRxTxEn_e serialRxTxEn)
{
  uint8_t valueA=0;
  uint8_t valueB=0;
  const char TX_EN = 0x00;
  const char RX_EN = 0x03;
  const char TXRX_DIS = 0x02;
  static bool semaphoreState = false;

  for(uint8_t i=0;i<8;i++)
  {
    if(i==u8_serialDevNr)
    {
      if(u8_serialDevNr<4)
      {
        if(serialRxTxEn == serialRxTx_TxEn) valueA |= (RX_EN << (u8_serialDevNr*2));
        else if(serialRxTxEn == serialRxTx_RxEn) valueA |= (TX_EN << (u8_serialDevNr*2));
        else if(serialRxTxEn == serialRxTx_RxTxDisable) valueA |= (TXRX_DIS << (u8_serialDevNr*2));
      }
      else
      {
        if(serialRxTxEn == serialRxTx_TxEn) valueB |= (RX_EN << ((u8_serialDevNr-4)*2));
        else if(serialRxTxEn == serialRxTx_RxEn) valueB |= (TX_EN << ((u8_serialDevNr-4)*2));
        else if(serialRxTxEn == serialRxTx_RxTxDisable) valueB |= (TXRX_DIS << ((u8_serialDevNr-4)*2));
      }
    }
    else
    {
      if(i < 4) valueA |= (TXRX_DIS << (i*2));
      else valueB |= (TXRX_DIS << ((i-4)*2));
    }
  }


  // Mögliche States
  // enum serialRxTxEn_e {serialRxTx_RxTxDisable, serialRxTx_TxEn, serialRxTx_RxEn};

  if(semaphoreState == false)
  {
    I2cRxSemaphoreTake();
    semaphoreState = true;
  }


  if(serialRxTxEn == serialRxTx_TxEn)
  {
    i2cWriteRegister(deviceAddress, MCP23017_IODIRA, 0x0);
    i2cWriteRegister(deviceAddress, MCP23017_IODIRB, 0x0);
  }

  if(u8_serialDevNr < 4) i2cWriteRegister(deviceAddress, MCP23017_GPIOA, valueA);
  else i2cWriteRegister(deviceAddress, MCP23017_GPIOB, valueB);

  if(serialRxTxEn != serialRxTx_TxEn)
  {
    I2cRxSemaphoreGive();
    semaphoreState = false;
  }
}