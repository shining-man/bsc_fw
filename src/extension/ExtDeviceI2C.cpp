// Copyright (c) 2024 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "extension/ExtDeviceI2C.h"
#include "Wire.h"

static const char *TAG = "EXT_DEV_I2C";

ExtDeviceI2C* ExtDeviceI2C::instance = nullptr;


ExtDeviceI2C::ExtDeviceI2C(uint8_t address) 
  : deviceAddress(address) 
  {  
    mutexI2cRx = xSemaphoreCreateMutex();
    instance = this;

    Wire.onReceive(onReceiveWrapper);
    Wire.onRequest(onRequestWrapper);

    bool i2cBegin = Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
    BSC_LOGD(TAG,"Init I2C - Status (%d)",i2cBegin);
  }


void ExtDeviceI2C::onReceiveWrapper(int len) {
    if (instance) {
        instance->onReceiveCb(len); 
    }
}

void ExtDeviceI2C::onRequestWrapper() {
    if (instance) {
        instance->onRequestCb(); 
    }
}

void ExtDeviceI2C::onRequestCb()
{
  //i2cSendDataToMaster();
}

void ExtDeviceI2C::onReceiveCb(int len)
{
  //BSC_LOGI(TAG, "onReceive");

  uint8_t rxCnt = 0;
  if(len <= 4)
  {
    while(Wire.available())
    {
      mI2cRxBuf[rxCnt] = Wire.read();
      rxCnt++;
    }
  }
}

void ExtDeviceI2C::I2cRxSemaphoreTake()
{
  xSemaphoreTake(mutexI2cRx, portMAX_DELAY);
}

void ExtDeviceI2C::I2cRxSemaphoreGive()
{
  xSemaphoreGive(mutexI2cRx);
}

uint8_t ExtDeviceI2C::isDeviceAvailable(uint8_t devAdresse)
{
  uint8_t txBuf[1];
  txBuf[0] = 0;

  Wire.beginTransmission(devAdresse);
  Wire.write(txBuf,1);
  return Wire.endTransmission();
}




/*
 * Beim Nutzen der Funktion das mutexI2cRx nicht vergessen!
 */
void ExtDeviceI2C::i2cWriteRegister(uint8_t u8_i2cDevAdr, uint8_t u8_reg, uint8_t u8_data)
{
  //xSemaphoreTake(mutexI2cRx, portMAX_DELAY);
  Wire.beginTransmission(u8_i2cDevAdr);
  Wire.write(u8_reg);
  Wire.write(u8_data);
  Wire.endTransmission();
}


void ExtDeviceI2C::i2cWriteBytes(uint8_t devAdr, uint8_t *data, uint8_t dataLen)
{
  xSemaphoreTake(mutexI2cRx, portMAX_DELAY);
  Wire.beginTransmission(devAdr);
  Wire.flush();
  Wire.write(data, dataLen);
  Wire.endTransmission();
  xSemaphoreGive(mutexI2cRx);
}


void ExtDeviceI2C::i2cReadBytes(uint8_t *data, uint8_t dataLen)
{
  Wire.readBytes(data, dataLen);
}


uint8_t ExtDeviceI2C::i2cRequest(uint8_t devAdr, uint8_t dataLen)
{
  return Wire.requestFrom(devAdr, dataLen);
}


void ExtDeviceI2C::i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, const void *dataAdr, uint8_t dataLen)
{
  uint8_t   txBuf[104];
  txBuf[0]=data1;
  txBuf[1]=data2;
  txBuf[2]=data3; //z.B. Nr. des BMS
  txBuf[3]=0x00;  //Reserve (evtl. CRC8)

  if(data1==BMS_DATA){bmsDataSemaphoreTake();}
  else if(data1==INVERTER_DATA){inverter.inverterDataSemaphoreTake();}

  if(dataLen>0) memcpy(&txBuf[TXBUFF_OFFSET], dataAdr, dataLen);

  if(data1==BMS_DATA){bmsDataSemaphoreGive();}
  else if(data1==INVERTER_DATA){inverter.inverterDataSemaphoreGive();}

  xSemaphoreTake(mutexI2cRx, portMAX_DELAY);
  Wire.beginTransmission(i2cAdr);
  uint8_t quant = Wire.write(txBuf,TXBUFF_OFFSET+dataLen);
  uint8_t error = Wire.endTransmission();
  xSemaphoreGive(mutexI2cRx);

  //vTaskDelay(pdMS_TO_TICKS(10));
}


void ExtDeviceI2C::i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, std::string data, uint8_t dataLen)
{
  i2cSendData(inverter, i2cAdr, data1, data2, data3, data.c_str(), dataLen);
}


void ExtDeviceI2C::i2cSendData(Inverter &inverter, uint8_t i2cAdr, uint8_t data1, uint8_t data2, uint8_t data3, int16_t data)
{
  i2cSendData(inverter, i2cAdr, data1, data2, data3, &data, 2);
}


/*void ExtDevice::writeByte(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t ExtDevice::readByte(uint8_t reg) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(deviceAddress, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}*/