// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/SeplosBmsV3.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include "ModbusRTU.hpp"

static const char *TAG = "SEPLOS_V3";

static void parsePackInfoA(modbusrtu::ModbusRTU *modbus, uint8_t devNr);
static void parsePackInfoB(modbusrtu::ModbusRTU *modbus, uint8_t devNr);
static void parsePackInfoC(modbusrtu::ModbusRTU *modbus, uint8_t devNr);

//static void message2Log(uint8_t * t_message, uint8_t len, uint8_t address);

static serialDevData_s *mDevData;

bool SeplosBmsV3_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  mDevData = devData;
  uint8_t u8_mCountOfPacks = devData->u8_NumberOfDevices;
  uint8_t response[SEPLOSBMS_MAX_ANSWER_LEN];

  uint8_t u8_lSeplosAdr=devData->u8_deviceNr;
  uint8_t u8_lSeplosAdrBmsData=devData->u8_BmsDataAdr+BT_DEVICES_COUNT;
  if(u8_mCountOfPacks>1)
  {
    u8_lSeplosAdr+=1;
  }

  #ifdef SEPLOS_DEBUG
  BSC_LOGI(TAG,"SeplosBms_readBmsData() devNr=%i, readFromAdr=%i, BmsDataAdr=%i, CountOfPacks=%i",u8_mDevNr,u8_lSeplosAdr,u8_lSeplosAdrBmsData,u8_mCountOfPacks);
  #endif

  modbusrtu::ModbusRTU modbus(port,callback,devNr);


  if(modbus.readData(u8_lSeplosAdr,modbusrtu::ModbusRTU::fCode::READ_CMD_04,0x1000,18,response))
  {
    //message2Log(response, 36, 0);
    parsePackInfoA(&modbus, u8_lSeplosAdrBmsData);
    vTaskDelay(pdMS_TO_TICKS(25));
  }
  else return false;

  if(modbus.readData(u8_lSeplosAdr,modbusrtu::ModbusRTU::fCode::READ_CMD_04,0x1100,26,response))
  {
    parsePackInfoB(&modbus, u8_lSeplosAdrBmsData);
    vTaskDelay(pdMS_TO_TICKS(25));
  }
  else return false;

  if(modbus.readData(u8_lSeplosAdr,modbusrtu::ModbusRTU::fCode::READ_COIL_01,0x1200,0x90,response))
  {
    parsePackInfoC(&modbus, u8_lSeplosAdrBmsData);
  }
  else return false;

  return true;
}


static void parsePackInfoA(modbusrtu::ModbusRTU *modbus, uint8_t devNr)
{
  setBmsTotalVoltage_int(devNr,modbus->getU16Value(TOTAL_VOLTAGE)); // 10mV
  setBmsTotalCurrent_int(devNr,modbus->getU16Value(TOTAL_CURRENT)); // 10mA
  setBmsAvgVoltage(devNr,modbus->getU16Value(AVG_CELL_VOLTAGE)); // 1mV
  setBmsChargePercentage(devNr, ROUND(modbus->getU16Value(SEPLOSV3_SOC), 10)); // 0.1%

  uint16_t maxCellVoltage = modbus->getU16Value(MAX_CELL_VOLTAGE); // 1mV
  uint16_t minCellVoltage = modbus->getU16Value(MIN_CELL_VOLTAGE); // 1mV
  setBmsMaxCellVoltage(devNr,maxCellVoltage);
  setBmsMinCellVoltage(devNr,minCellVoltage);
  setBmsMaxCellDifferenceVoltage(devNr,maxCellVoltage-minCellVoltage);
}


static void parsePackInfoB(modbusrtu::ModbusRTU *modbus, uint8_t devNr)
{
  // Cellvoltage
  uint16_t cellVoltage;
  uint8_t maxCellVoltageNr,minCellVoltageNr;
  uint16_t maxCellVoltage=0;
  uint16_t minCellVoltage=0xFFFF;

  for(uint8_t i=0;i<16;i++)
  {
    cellVoltage = modbus->getU16Value(CELLVOLTAGE_1+i);
    //BSC_LOGI(TAG,"%i, cellVoltage=%i",i,cellVoltage);
    setBmsCellVoltage(devNr,i,cellVoltage);

    if(cellVoltage>maxCellVoltage)
    {
      maxCellVoltage=cellVoltage;
      maxCellVoltageNr=i;
    }
    if(cellVoltage<minCellVoltage)
    {
      minCellVoltage=cellVoltage;
      minCellVoltageNr=i;
    }
  }
  setBmsMaxVoltageCellNumber(devNr,maxCellVoltageNr);
  setBmsMinVoltageCellNumber(devNr,minCellVoltageNr);

  // Temperature
  for(uint8_t i=0;i<3;i++) setBmsTempatureI16(devNr, i, (int16_t)modbus->getU16Value(TEMPERATURE_1+i));
}


static void parsePackInfoC(modbusrtu::ModbusRTU *modbus, uint8_t devNr)
{

  /* ToDo:
  * bmsIsBalancingActive
  * bmsErrors
  */

  setBmsStateFETsDischarge(devNr,modbus->getBitValue(FET_STATE,0));
  setBmsStateFETsCharge(devNr,modbus->getBitValue(FET_STATE,1));

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
