// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/JkInverterBms.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include "log.h"
#include "ModbusRTU.hpp"

static const char *TAG = "JK_INV_BMS";

static void parsePackInfoA(modbusrtu::ModbusRTU *modbus, uint8_t dataMappingNr);
static void parsePackInfoB(modbusrtu::ModbusRTU *modbus, uint8_t dataMappingNr);
static void parsePackInfoC(modbusrtu::ModbusRTU *modbus, uint8_t dataMappingNr);

//static void message2Log(uint8_t * t_message, uint8_t len, uint8_t address);

static serialDevData_s *mDevData;

bool JkInverterBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  mDevData = devData;
  uint8_t response[JK_INVERTER_BMS_MAX_ANSWER_LEN];

  uint8_t jkInvBmsAdr = devData->bmsAdresse;
  uint8_t jkInvBmsAdrBmsData = devData->dataMappingNr;

  
  #ifdef JK_INV_DEBUG
  BSC_LOGI(TAG,"JkInverterBms_readBmsData() devNr=%i, readFromAdr=%i, BmsDataAdr=%i",u8_mDevNr,jkInvBmsAdr,jkInvBmsAdrBmsData);
  #endif

  modbusrtu::ModbusRTU modbus(port,callback,devNr);

  /* Es können nicht alle 268 Bytes gleichzeitig gelesen werden.
   * Es kann z.B. in folgenden Teilen gelsensen werden.
   * 0x1000: 123 Register
   * 0x1200: 123 Register
   * 0x127C:  84 Register
   * 0x12DA:  42 Register
   * 0x12FA:  21 Register
   */

  if(modbus.readData(jkInvBmsAdr, modbusrtu::ModbusRTU::fCode::READ_CMD_JK, 0x1200, 37, response))
  {
    //message2Log(response, 36, 0);
    parsePackInfoA(&modbus, jkInvBmsAdrBmsData);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  else 
  {
    #ifdef JK_INV_DEBUG
    BSC_LOGE(TAG,"Fehler beim lesen von 0x1200");
    #endif
    return false;
  }


  if(modbus.readData(jkInvBmsAdr, modbusrtu::ModbusRTU::fCode::READ_CMD_JK, 0x1290, 25, response))
  {
    //message2Log(response, 36, 0);
    parsePackInfoB(&modbus, jkInvBmsAdrBmsData);

    // MQTT
    mqttPublish(MQTT_TOPIC_BMS_BT, jkInvBmsAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(jkInvBmsAdrBmsData));
    mqttPublish(MQTT_TOPIC_BMS_BT, jkInvBmsAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(jkInvBmsAdrBmsData));

    vTaskDelay(pdMS_TO_TICKS(25));
  }
  else 
  {
    #ifdef JK_INV_DEBUG
    BSC_LOGE(TAG,"Fehler beim lesen von 0x1290");
    #endif
    return false;
  }


  if(modbus.readData(jkInvBmsAdr, modbusrtu::ModbusRTU::fCode::READ_CMD_JK, 0x12F8, 3, response))
  {
    //message2Log(response, 36, 0);
    parsePackInfoC(&modbus, jkInvBmsAdrBmsData);

    vTaskDelay(pdMS_TO_TICKS(25));
  }
  else 
  {
    #ifdef JK_INV_DEBUG
    BSC_LOGE(TAG,"Fehler beim lesen von 0x1290");
    #endif
    return false;
  }


  return true;
}


static void parsePackInfoA(modbusrtu::ModbusRTU *modbus, uint8_t dataMappingNr)
{
  // Cellvoltage
  for(uint8_t i;i<24;i++) setBmsCellVoltage(dataMappingNr, i, modbus->getU16ValueByOffset(i*2));

  // Avg. Voltage
  setBmsAvgVoltage(dataMappingNr, modbus->getU16ValueByOffset(68));

  // Max. Cell Dif.
  setBmsMaxCellDifferenceVoltage(dataMappingNr, modbus->getU16ValueByOffset(70));

  // Max. Cellvoltage 
  setBmsMaxVoltageCellNumber(dataMappingNr, modbus->getU8ValueByOffset(72));
  setBmsMaxCellVoltage(dataMappingNr, getBmsCellVoltage(dataMappingNr, modbus->getU8ValueByOffset(72)));

  // Min. Cellvoltage 
  setBmsMinVoltageCellNumber(dataMappingNr, modbus->getU8ValueByOffset(73));
  setBmsMinCellVoltage(dataMappingNr, getBmsCellVoltage(dataMappingNr, modbus->getU8ValueByOffset(73)));
}


static void parsePackInfoB(modbusrtu::ModbusRTU *modbus, uint8_t dataMappingNr)
{
  uint8_t byteOffset = 0x90;

  // Total Voltage
  setBmsTotalVoltage_int(dataMappingNr, modbus->getU32ValueByOffset(144 - byteOffset)/10);

  // Total Current
  setBmsTotalCurrent_int(dataMappingNr, modbus->getI32ValueByOffset(152 - byteOffset)/10);

  // Temperature
  setBmsTempatureI16(dataMappingNr, 0, modbus->getI16ValueByOffset(156 - byteOffset) * 10);
  setBmsTempatureI16(dataMappingNr, 1, modbus->getI16ValueByOffset(158 - byteOffset) * 10);

  //int16_t t0 = modbus->getI16ValueByOffset(156 - byteOffset) * 10;
  //int16_t t1 = modbus->getI16ValueByOffset(158 - byteOffset) * 10;
  //BSC_LOGI(TAG, "t0=%i, t1=%i", t0, t1);

  // Alarm
  uint32_t alarms = 0;
  /*
  if(modbus->getBitValueByOffset(162 - byteOffset, 0)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 16  AlarmChargeMOS
  if(modbus->getBitValueByOffset(162 - byteOffset, 1)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 17  AlarmDischargeMOS
  if(modbus->getBitValueByOffset(162 - byteOffset, 2)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 18  GPSDisconneted
  //                                                                                            Bit 19  Modify PWD. in time
  if(modbus->getBitValueByOffset(162 - byteOffset, 4)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 20  Discharge On Failed
  if(modbus->getBitValueByOffset(162 - byteOffset, 5)) alarms |= BMS_ERR_STATUS_CHG_OTP;     // Bit 21  Battery Over Temp Alarm*/


  if(modbus->getBitValueByOffset(162 - byteOffset, 0)) alarms |= BMS_ERR_STATUS_CHG_OTP;     // Bit  8  AlarmChOTP; Charge Overtemp
  if(modbus->getBitValueByOffset(162 - byteOffset, 1)) alarms |= BMS_ERR_STATUS_CHG_UTP;     // Bit  9  AlarmChUTP; Under Temperature
  if(modbus->getBitValueByOffset(162 - byteOffset, 2)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 10  AlarmCPUAuxCommuErr; ?
  if(modbus->getBitValueByOffset(162 - byteOffset, 3)) alarms |= BMS_ERR_STATUS_CELL_UVP;    // Bit 11  AlarmCellUVP; Cell Untervoltage Protection
  if(modbus->getBitValueByOffset(162 - byteOffset, 4)) alarms |= BMS_ERR_STATUS_BATTERY_UVP; // Bit 12  AlarmBatUVP; ?
  if(modbus->getBitValueByOffset(162 - byteOffset, 5)) alarms |= BMS_ERR_STATUS_DSG_OCP;     // Bit 13  AlarmDchOCP; Discharge overcurrent protection
  if(modbus->getBitValueByOffset(162 - byteOffset, 6)) alarms |= BMS_ERR_STATUS_DSG_OCP;     // Bit 14  AlarmDchSCP; ?
  if(modbus->getBitValueByOffset(162 - byteOffset, 7)) alarms |= BMS_ERR_STATUS_DSG_OTP;     // Bit 15  AlarmDchOTP; Discharge Overtemp


  if(modbus->getBitValueByOffset(163 - byteOffset, 0)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  0  AlarmWireRes; ?
  if(modbus->getBitValueByOffset(163 - byteOffset, 1)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  1  AlarmMosOTP; ?
  if(modbus->getBitValueByOffset(163 - byteOffset, 2)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  2  AlarmCellQuantity; ?
  if(modbus->getBitValueByOffset(163 - byteOffset, 3)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  3  AlarmCurSensorErr; ?
  if(modbus->getBitValueByOffset(163 - byteOffset, 4)) alarms |= BMS_ERR_STATUS_CELL_OVP;    // Bit  4  AlarmCellOVP; Cell OVP 
  if(modbus->getBitValueByOffset(163 - byteOffset, 5)) alarms |= BMS_ERR_STATUS_BATTERY_OVP; // Bit  5  AlarmBatOVP; ?
  if(modbus->getBitValueByOffset(163 - byteOffset, 6)) alarms |= BMS_ERR_STATUS_CHG_OCP;     // Bit  6  AlarmChOCP; Protection chare overcurrent
  if(modbus->getBitValueByOffset(163 - byteOffset, 7)) alarms |= BMS_ERR_STATUS_CHG_OCP;     // Bit  7  AlarmChSCP (ShortCurrentProtection); ?


  setBmsErrors(byteOffset, alarms);

  #ifdef JK_INV_DEBUG
  BSC_LOGI(TAG, "JK Alarms: %i, %i, %i, %i, %i, %i, Errors=%i", modbus->getU8ValueByOffset(160 - byteOffset), modbus->getU8ValueByOffset(161 - byteOffset), 
    modbus->getU8ValueByOffset(162 - byteOffset), modbus->getU8ValueByOffset(163 - byteOffset), modbus->getU8ValueByOffset(164 - byteOffset), modbus->getU8ValueByOffset(165 - byteOffset), alarms );
  #endif

  // Bal. Current
  setBmsBalancingCurrentI16(byteOffset, modbus->getI16ValueByOffset(164 - byteOffset) / 10);

  // Bal. State
  setBmsIsBalancingActive(byteOffset, modbus->getU8ValueByOffset(166 - byteOffset));

  // SoC
  setBmsChargePercentage(byteOffset, modbus->getU8ValueByOffset(167 - byteOffset));

  // FET state charge
  if(modbus->getU8ValueByOffset(192 - byteOffset) > 0) setBmsStateFETsCharge(byteOffset, true);
  else setBmsStateFETsCharge(byteOffset, false);

  // FET state discharge
  if(modbus->getU8ValueByOffset(193 - byteOffset) > 0) setBmsStateFETsDischarge(byteOffset, true);
  else setBmsStateFETsDischarge(byteOffset, false);
}


static void parsePackInfoC(modbusrtu::ModbusRTU *modbus, uint8_t dataMappingNr)
{
  uint8_t byteOffset = 0xF8;

  /* Temperature
   * BSC -> BMS
   * t0  -> T1
   * t1  -> T2
   * t2  -> MOS TEMP
   * t3  -> T4
   * t4  -> T5
  */

  //int16_t t2 = modbus->getI16ValueByOffset(248 - byteOffset) * 10; //MOS
  int16_t t3 = modbus->getI16ValueByOffset(250 - byteOffset) * 10;
  int16_t t4 = modbus->getI16ValueByOffset(252 - byteOffset) * 10;

  if(t3 > t4) setBmsTempatureI16(dataMappingNr, 2, t3);
  else setBmsTempatureI16(dataMappingNr, 2, t4);

  #ifdef JK_INV_DEBUG
  BSC_LOGI(TAG, "t2=%i, t3=%i, t4=%i", t2, t3, t4);
  #endif
}
