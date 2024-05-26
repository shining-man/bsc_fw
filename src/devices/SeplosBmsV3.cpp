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

    // MQTT
    mqttPublish(MQTT_TOPIC_BMS_BT, u8_lSeplosAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(u8_lSeplosAdrBmsData));
    mqttPublish(MQTT_TOPIC_BMS_BT, u8_lSeplosAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(u8_lSeplosAdrBmsData));

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
  setBmsTotalVoltage_int(devNr,modbus->getU16Value(SEPLOS3_TOTAL_VOLTAGE)); // 10mV
  setBmsTotalCurrent_int(devNr,modbus->getU16Value(SEPLOS3_TOTAL_CURRENT)); // 10mA
  setBmsAvgVoltage(devNr,modbus->getU16Value(SEPLOS3_AVG_CELL_VOLTAGE)); // 1mV
  setBmsChargePercentage(devNr, ROUND(modbus->getU16Value(SEPLOS3_SEPLOSV3_SOC), 10)); // 0.1%

  uint16_t maxCellVoltage = modbus->getU16Value(SEPLOS3_MAX_CELL_VOLTAGE); // 1mV
  uint16_t minCellVoltage = modbus->getU16Value(SEPLOS3_MIN_CELL_VOLTAGE); // 1mV
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
    cellVoltage = modbus->getU16Value(SEPLOS3_CELLVOLTAGE_1+i);
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
  for(uint8_t i=0;i<3;i++) setBmsTempatureI16(devNr, i, (int16_t)modbus->getU16Value(SEPLOS3_TEMPERATURE_1+i));
}

#include "BmsDataTypes.hpp"
static void parsePackInfoC(modbusrtu::ModbusRTU *modbus, uint8_t devNr)
{
  uint32_t errors = 0;

  /* ToDo:
  * bmsIsBalancingActive
  * bmsErrors
  */



/*
BMS_ERR_STATUS_CELL_OVP       // x     1 - bit0 single cell overvoltage protection
BMS_ERR_STATUS_CELL_UVP       // x     2 - bit1 single cell undervoltage protection
BMS_ERR_STATUS_BATTERY_OVP    // x     4 - bit2  whole pack overvoltage protection
BMS_ERR_STATUS_BATTERY_UVP    // x     8 - bit3  Whole pack undervoltage protection
BMS_ERR_STATUS_CHG_OTP        // x    16 - bit4  charging over temperature protection
BMS_ERR_STATUS_CHG_UTP        // x    32 - bit5  charging low temperature protection
BMS_ERR_STATUS_DSG_OTP        // x    64 - bit6  Discharge over temperature protection
BMS_ERR_STATUS_DSG_UTP        // x   128 - bit7  discharge low temperature protection
BMS_ERR_STATUS_CHG_OCP        // x   256 - bit8  charging overcurrent protection
BMS_ERR_STATUS_DSG_OCP        // x   512 - bit9  Discharge overcurrent protection
BMS_ERR_STATUS_SHORT_CIRCUIT  // x  1024 - bit10 short circuit protection
BMS_ERR_STATUS_AFE_ERROR      //   2048 - bit11 Front-end detection IC error
BMS_ERR_STATUS_SOFT_LOCK      //   4096 - bit12 software lock MOS
*/

 /*
 #define SEPLOS3_EQUALIZATION_08_01    0x1230
 #define SEPLOS3_EQUALIZATION_16_09    0x1238

 0x1230 Cell 08-01 equalization event code 08-01
 0x1238 Cell 16-09 equalization event code 16-09 */
if(modbus->getU8Value(SEPLOS3_EQUALIZATION_08_01) > 0 
  || modbus->getU8Value(SEPLOS3_EQUALIZATION_16_09) > 0) setBmsIsBalancingActive(devNr, 1); 
else setBmsIsBalancingActive(devNr, 0);




  /* 0x1248 Voltage event code (SEPLOS3_VOLATGE_EVENT, TB02)
  INDEX Definition
  Bit0  Cell high voltage alarm
  Bit1  Cell over voltage protection
  Bit2  Cell low voltage alarm
  Bit3  Cell under voltage protection
  Bit4  Pack high voltage alarm
  Bit5  Pack over voltage protection
  Bit6  Pack low voltage alarm
  Bit7  Pack under voltage protection*/
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,0)) errors |= BMS_ERR_STATUS_CELL_OVP;
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,1)) errors |= BMS_ERR_STATUS_CELL_OVP;
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,2)) errors |= BMS_ERR_STATUS_CELL_UVP;
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,3)) errors |= BMS_ERR_STATUS_CELL_UVP;
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,4)) errors |= BMS_ERR_STATUS_BATTERY_OVP;
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,5)) errors |= BMS_ERR_STATUS_BATTERY_OVP;
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,6)) errors |= BMS_ERR_STATUS_BATTERY_UVP;
  if(modbus->getBitValue(SEPLOS3_VOLATGE_EVENT,7)) errors |= BMS_ERR_STATUS_BATTERY_UVP;

  
  /* 0x1250 Cells Temperature event code (SEPLOS3_CELLS_TEMPERATURE, TB03)
  INDEX Definition
  Bit0  Charge high temperature alarm
  Bit1  Charge over temperature protection
  Bit2  Charge low temperature alarm
  Bit3  Charge under temperature protection
  Bit4  Discharge high temperature alarm
  Bit5  Discharge over temperature protection
  Bit6  Discharge low temperature alarm
  Bit7  Discharge under temperature protection*/
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,0)) errors |= BMS_ERR_STATUS_CHG_OTP;
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,1)) errors |= BMS_ERR_STATUS_CHG_OTP;
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,2)) errors |= BMS_ERR_STATUS_CHG_UTP;
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,3)) errors |= BMS_ERR_STATUS_CHG_UTP;
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,4)) errors |= BMS_ERR_STATUS_DSG_OTP;
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,5)) errors |= BMS_ERR_STATUS_DSG_OTP;
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,6)) errors |= BMS_ERR_STATUS_DSG_UTP;
  if(modbus->getBitValue(SEPLOS3_CELLS_TEMPERATURE,7)) errors |= BMS_ERR_STATUS_DSG_UTP;


  /* 0x1258 Environment and power Temperature event code (SEPLOS3_ENVIRONMENT_TEMP, TB04)
  INDEX Definition
  Bit0  High environment temperature alarm
  Bit1  Over environment temperature protection
  Bit2  Low environment temperature alarm
  Bit3  Under environment temperature protection
  Bit4  High Power temperature alarm
  Bit5  Over Power temperature protection
  Bit6  Cell temperature low heating
  Bit7  Reservation*/


  /* 0x1260 Current event code1 (SEPLOS3_CURRENT_EVENT_1, TB05)
  INDEX Definition
  Bit0  Charge current alarm
  Bit1  Charge over current protection
  Bit2  Charge second level current protection
  Bit3  Discharge current alarm
  Bit4  Discharge over current protection
  Bit5  Discharge second level over current protection
  Bit6  Output short circuit protection
  Bit7  Reservation*/
  if(modbus->getBitValue(SEPLOS3_CURRENT_EVENT_1,0)) errors |= BMS_ERR_STATUS_CHG_OCP;
  if(modbus->getBitValue(SEPLOS3_CURRENT_EVENT_1,1)) errors |= BMS_ERR_STATUS_CHG_OCP;
  if(modbus->getBitValue(SEPLOS3_CURRENT_EVENT_1,2)) errors |= BMS_ERR_STATUS_CHG_OCP;
  if(modbus->getBitValue(SEPLOS3_CURRENT_EVENT_1,3)) errors |= BMS_ERR_STATUS_DSG_OCP;
  if(modbus->getBitValue(SEPLOS3_CURRENT_EVENT_1,4)) errors |= BMS_ERR_STATUS_DSG_OCP;
  if(modbus->getBitValue(SEPLOS3_CURRENT_EVENT_1,5)) errors |= BMS_ERR_STATUS_DSG_OCP;
  if(modbus->getBitValue(SEPLOS3_CURRENT_EVENT_1,6)) errors |= BMS_ERR_STATUS_SHORT_CIRCUIT;


  /* 0x1268 Current event code2 (SEPLOS3_CURRENT_EVENT_2, TB16)
  INDEX Definition
  Bit0  Output short latch up
  Bit1  Reservation
  Bit2  Second Charge latch up
  Bit3  Second Discharge latch up
  Bit4  Reservation
  Bit5  Reservation
  Bit6  Reservation
  Bit7  Reservation*/


  /* 0x1270 The residual capacity code (SEPLOS3_RESIDUAL_CAPACITY, TB06)
  INDEX Definition
  Bit0  Reservation
  Bit1  Reservation
  Bit2  Soc alarm
  Bit3  Soc protection
  Bit4  Cell Diff alarm
  Bit5  Reservation
  Bit6  Reservation
  Bit7  Reservation*/


  /* 0x1278 The FET event code (SEPLOS3_FET_EVENT, TB07)
  INDEX Definition
  Bit0  Discharge FET on
  Bit1  Charge FET on
  Bit2  Current limiting FET on
  Bit3  Heating on
  Bit4  Reservation
  Bit5  Reservation
  Bit6  Reservation
  Bit7  Reservation*/
  setBmsStateFETsDischarge(devNr,modbus->getBitValue(SEPLOS3_FET_EVENT,0));
  setBmsStateFETsCharge(devNr,modbus->getBitValue(SEPLOS3_FET_EVENT,1));


  /* 0x1280 battery equalization state code (SEPLOS3_BATTERY_EQUALIZATION, TB08)
  INDEX Definition
  Bit0  low Soc alarm
  Bit1  Intermittent charge
  Bit2  External switch control
  Bit3  Static standby and sleep mode
  Bit4  History data recording
  Bit5  Under Soc protect
  Bit6  Acktive-Limited Current
  Bit7  Passive-Limited Current*/
  

  /* 0x1240 System state code (SEPLOS3_SYSTEM_STATE, TB09)
  INDEX Definition
  Bit0  Discharge
  Bit1  Charge
  Bit2  Floating charge
  Bit3  Full charge
  Bit4  Standby mode
  Bit5  Turn off
  Bit6  Reservation
  Bit7  Reservation */
  

  /* 0x1288 Hard fault event code (SEPLOS3_FARD_FAULT, TB15)
  Bit   Definition                Note
  Bit0  NTC Fault                 Wire break or short
  Bit1  AFE Fault                 AFE Comm. Error
  Bit2  Charge Mosfets Fault      Mosfets short
  Bit3  Discharge Mosfets Fault   Mosfets short
  Bit4  Cell Fault                Large Voltage different
  Bit5  Break Line Fault
  Bit6  Key Fault
  Bit7  Aerosol Alarm*/
  for(uint8_t i = 0; i<8;i++) 
    if(modbus->getBitValue(SEPLOS3_FARD_FAULT,i)) errors |= BMS_ERR_STATUS_AFE_ERROR;


  setBmsErrors(devNr , errors);
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
