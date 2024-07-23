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

static void parsePackInfoA(modbusrtu::ModbusRTU *modbus, uint8_t devNr);
static void parsePackInfoB(modbusrtu::ModbusRTU *modbus, uint8_t devNr);

//static void message2Log(uint8_t * t_message, uint8_t len, uint8_t address);

static serialDevData_s *mDevData;

bool JkInverterBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  mDevData = devData;
  uint8_t u8_mCountOfPacks = devData->u8_NumberOfDevices;
  uint8_t response[JK_INVERTER_BMS_MAX_ANSWER_LEN];

  uint8_t jkInvBmsAdr = devData->u8_deviceNr;
  uint8_t jkInvBmsAdrBmsData = devData->u8_BmsDataAdr + BT_DEVICES_COUNT;

  jkInvBmsAdr += 1;
  
  #ifdef JK_INV_DEBUG
  BSC_LOGI(TAG,"JkInverterBms_readBmsData() devNr=%i, readFromAdr=%i, BmsDataAdr=%i, CountOfPacks=%i",u8_mDevNr,jkInvBmsAdr,jkInvBmsAdrBmsData,u8_mCountOfPacks);
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
    BSC_LOGE(TAG,"Fehler beim lesen von 0x1200");
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
    BSC_LOGE(TAG,"Fehler beim lesen von 0x1290");
    return false;
  }


  return true;
}


static void parsePackInfoA(modbusrtu::ModbusRTU *modbus, uint8_t devNr)
{
  // Cellvoltage
  for(uint8_t i;i<24;i++) setBmsCellVoltage(devNr, i, modbus->getU16ValueByOffset(i*2));

  // Avg. Voltage
  setBmsAvgVoltage(devNr, modbus->getU16ValueByOffset(68));

  // Max. Cell Dif.
  setBmsMaxCellDifferenceVoltage(devNr, modbus->getU16ValueByOffset(70));

  // Max. Cellvoltage 
  setBmsMaxVoltageCellNumber(devNr, modbus->getU8ValueByOffset(72));
  setBmsMaxCellVoltage(devNr, getBmsCellVoltage(devNr, modbus->getU8ValueByOffset(72)));

  // Min. Cellvoltage 
  setBmsMinVoltageCellNumber(devNr, modbus->getU8ValueByOffset(73));
  setBmsMinCellVoltage(devNr, getBmsCellVoltage(devNr, modbus->getU8ValueByOffset(73)));
}


static void parsePackInfoB(modbusrtu::ModbusRTU *modbus, uint8_t devNr)
{
  uint8_t byteOffset = 0x90;

  // Total Voltage
  setBmsTotalVoltage_int(devNr, modbus->getU32ValueByOffset(144 - byteOffset) / 10);

  // Total Current
  setBmsTotalCurrent_int(devNr, modbus->getI32ValueByOffset(152 - byteOffset) / 10);

  // Temperature
  setBmsTempatureI16(devNr, 0, modbus->getI16ValueByOffset(156 - byteOffset) * 10);
  setBmsTempatureI16(devNr, 1, modbus->getI16ValueByOffset(158 - byteOffset) * 10);

  // Alarm
  uint32_t alarms = 0;
  if(modbus->getBitValueByOffset(160 - byteOffset, 0)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  0  AlarmWireRes
  if(modbus->getBitValueByOffset(160 - byteOffset, 1)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  1  AlarmMosOTP
  if(modbus->getBitValueByOffset(160 - byteOffset, 2)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  2  AlarmCellQuantity
  if(modbus->getBitValueByOffset(160 - byteOffset, 3)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit  3  AlarmCurSensorErr
  if(modbus->getBitValueByOffset(160 - byteOffset, 4)) alarms |= BMS_ERR_STATUS_CELL_OVP;    // Bit  4  AlarmCellOVP
  if(modbus->getBitValueByOffset(160 - byteOffset, 5)) alarms |= BMS_ERR_STATUS_BATTERY_OVP; // Bit  5  AlarmBatOVP
  if(modbus->getBitValueByOffset(160 - byteOffset, 6)) alarms |= BMS_ERR_STATUS_CHG_OCP;     // Bit  6  AlarmChOCP
  if(modbus->getBitValueByOffset(160 - byteOffset, 7)) alarms |= BMS_ERR_STATUS_CHG_OCP;     // Bit  7  AlarmChSCP (ShortCurrentProtection)

  if(modbus->getBitValueByOffset(161 - byteOffset, 0)) alarms |= BMS_ERR_STATUS_CHG_OTP;     // Bit  8  AlarmChOTP
  if(modbus->getBitValueByOffset(161 - byteOffset, 1)) alarms |= BMS_ERR_STATUS_CHG_UTP;     // Bit  9  AlarmChUTP
  if(modbus->getBitValueByOffset(161 - byteOffset, 2)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 10  AlarmCPUAuxCommuErr
  if(modbus->getBitValueByOffset(161 - byteOffset, 3)) alarms |= BMS_ERR_STATUS_CELL_UVP;    // Bit 11  AlarmCellUVP
  if(modbus->getBitValueByOffset(161 - byteOffset, 4)) alarms |= BMS_ERR_STATUS_BATTERY_UVP; // Bit 12  AlarmBatUVP
  if(modbus->getBitValueByOffset(161 - byteOffset, 5)) alarms |= BMS_ERR_STATUS_DSG_OCP;     // Bit 13  AlarmDchOCP
  if(modbus->getBitValueByOffset(161 - byteOffset, 6)) alarms |= BMS_ERR_STATUS_DSG_OCP;     // Bit 14  AlarmDchSCP
  if(modbus->getBitValueByOffset(161 - byteOffset, 7)) alarms |= BMS_ERR_STATUS_DSG_OTP;     // Bit 15  AlarmDchOTP

  if(modbus->getBitValueByOffset(162 - byteOffset, 0)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 16  AlarmChargeMOS
  if(modbus->getBitValueByOffset(162 - byteOffset, 1)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 17  AlarmDischargeMOS
  if(modbus->getBitValueByOffset(162 - byteOffset, 2)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 18  GPSDisconneted
  //                                                                                            Bit 19  Modify PWD. in time
  if(modbus->getBitValueByOffset(162 - byteOffset, 4)) alarms |= BMS_ERR_STATUS_AFE_ERROR;   // Bit 20  Discharge On Failed
  if(modbus->getBitValueByOffset(162 - byteOffset, 5)) alarms |= BMS_ERR_STATUS_CHG_OTP;     // Bit 21  Battery Over Temp Alarm

  setBmsErrors(devNr, alarms);


  // Bal. Current
  setBmsBalancingCurrentI16(devNr, modbus->getI16ValueByOffset(164 - byteOffset) / 10);

  // Bal. State
  setBmsIsBalancingActive(devNr, modbus->getU8ValueByOffset(166 - byteOffset));

  // SoC
  setBmsChargePercentage(devNr, modbus->getU8ValueByOffset(167 - byteOffset));

  // FET state charge
  if(modbus->getU8ValueByOffset(192 - byteOffset) > 0) setBmsStateFETsCharge(devNr, true);
  else setBmsStateFETsCharge(devNr, false);

  // FET state discharge
  if(modbus->getU8ValueByOffset(193 - byteOffset) > 0) setBmsStateFETsDischarge(devNr, true);
  else setBmsStateFETsDischarge(devNr, false);
}
