// Copyright (c) 2026
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/FelicityLuxYBms.h"
#include "BmsData.h"
#include "BmsDataTypes.hpp"
#include "ModbusRTU.hpp"
#include "log.h"
#include "mqtt_t.h"

static const char *TAG = "FELICITY_LUXY";

namespace
{
  constexpr uint16_t REG_START               = 0x1302; // 4866
  constexpr uint16_t REG_END                 = 0x133D; // 4925
  constexpr uint16_t REG_STATUS_WORD         = 0x1302; // Charge/Discharge enable bits
  constexpr uint16_t REG_FAULT_STATUS        = 0x1304; // BmsFaultFlag
  constexpr uint16_t REG_PACK_VOLTAGE        = 0x1306; // BattVolt, 0.01V
  constexpr uint16_t REG_PACK_CURRENT        = 0x1307; // BattCurr, 0.1A, int16
  constexpr uint16_t REG_AMBIENT_TEMP        = 0x130A; // AmbTemp, 1C, int16
  constexpr uint16_t REG_SOC                 = 0x130B; // BmsSoc, 1%
  constexpr uint16_t REG_CYCLE_COUNT         = 0x130D; // BmsCycleCnt
  constexpr uint16_t REG_CELL_VOLTAGE_1      = 0x132A; // CellVolt01, mV
  constexpr uint16_t REG_CELL_VOLTAGE_16     = 0x1339; // CellVolt16, mV
  constexpr uint16_t REG_CELL_TEMPERATURE_1  = 0x133A; // CellTemp01, 1C, int16
  constexpr uint16_t REG_CELL_TEMPERATURE_4  = 0x133D; // CellTemp04, 1C, int16
  constexpr uint16_t STATUS_WORD_BIT_CHARGE_ENABLE    = 1U << 0;
  constexpr uint16_t STATUS_WORD_BIT_DISCHARGE_ENABLE = 1U << 2;

  constexpr uint16_t STATUS_BIT_CELL_OVP     = 1U << 2;
  constexpr uint16_t STATUS_BIT_CELL_UVP     = 1U << 3;
  constexpr uint16_t STATUS_BIT_CHG_OCP      = 1U << 4;
  constexpr uint16_t STATUS_BIT_DSG_OCP      = 1U << 5;
  constexpr uint16_t STATUS_BIT_BMS_OTP      = 1U << 6;
  constexpr uint16_t STATUS_BIT_CELL_OTP     = 1U << 8;
  constexpr uint16_t STATUS_BIT_CELL_UTP     = 1U << 9;
  
  uint32_t mapStatusBits(uint16_t statusRaw)
  {
    uint32_t status = BMS_ERR_STATUS_OK;

    if(statusRaw & STATUS_BIT_CELL_OVP) status |= BMS_ERR_STATUS_CELL_OVP;
    if(statusRaw & STATUS_BIT_CELL_UVP) status |= BMS_ERR_STATUS_CELL_UVP;
    if(statusRaw & STATUS_BIT_CHG_OCP)  status |= BMS_ERR_STATUS_CHG_OCP;
    if(statusRaw & STATUS_BIT_DSG_OCP)  status |= BMS_ERR_STATUS_DSG_OCP;
    if(statusRaw & STATUS_BIT_BMS_OTP)  status |= (BMS_ERR_STATUS_CHG_OTP | BMS_ERR_STATUS_DSG_OTP);
    if(statusRaw & STATUS_BIT_CELL_OTP) status |= (BMS_ERR_STATUS_CHG_OTP | BMS_ERR_STATUS_DSG_OTP);
    if(statusRaw & STATUS_BIT_CELL_UTP) status |= (BMS_ERR_STATUS_CHG_UTP | BMS_ERR_STATUS_DSG_UTP);

    return status;
  }
}


bool FelicityLuxYBms_readBmsData(BscSerial *bscSerial, Stream *port, uint8_t devNr, serialDevData_s *devData)
{
  uint8_t response[FELICITY_LUXY_BMS_MAX_ANSWER_LEN];

  const uint8_t bmsAddress = devData->bmsAdresse;
  const uint8_t dataMappingNr = devData->dataMappingNr;

  if(dataMappingNr >= MUBER_OF_DATA_DEVICES) return false;
  modbusrtu::ModbusRTU modbus(port, devNr);

  if(!modbus.readData(bscSerial, bmsAddress, modbusrtu::ModbusRTU::fCode::READ_CMD_JK, REG_START, (REG_END - REG_START + 1), response))
  {
    BSC_LOGE(TAG, "Modbus read failed: addr=%u", bmsAddress);
    return false;
  }

  uint32_t cellVoltageSum = 0;
  uint16_t maxCellVoltage = 0;
  uint16_t minCellVoltage = 0xFFFF;
  uint8_t maxCellNumber = 0;
  uint8_t minCellNumber = 0;

  for(uint8_t i = 0; i < 16; i++)
  {
    const uint16_t cellVoltage = modbus.getU16Value(REG_CELL_VOLTAGE_1 + i); // 1mV
    setBmsCellVoltage(dataMappingNr, i, cellVoltage);

    cellVoltageSum += cellVoltage;
    if(cellVoltage > maxCellVoltage)
    {
      maxCellVoltage = cellVoltage;
      maxCellNumber = i;
    }
    if(cellVoltage < minCellVoltage)
    {
      minCellVoltage = cellVoltage;
      minCellNumber = i;
    }
  }

  setBmsAvgVoltage(dataMappingNr, static_cast<uint16_t>(cellVoltageSum / 16));
  setBmsMaxCellVoltage(dataMappingNr, maxCellVoltage);
  setBmsMinCellVoltage(dataMappingNr, minCellVoltage);
  setBmsMaxVoltageCellNumber(dataMappingNr, maxCellNumber);
  setBmsMinVoltageCellNumber(dataMappingNr, minCellNumber);
  setBmsMaxCellDifferenceVoltage(dataMappingNr, maxCellVoltage - minCellVoltage);

  // 0.01V => interner Speicher ist ebenfalls 0.01V
  setBmsTotalVoltage_int(dataMappingNr, static_cast<int16_t>(modbus.getU16Value(REG_PACK_VOLTAGE)));

  // 0.1A signed => 0.01A fuer den internen Speicher
  int32_t totalCurrent_001A = static_cast<int32_t>(modbus.getI16Value(REG_PACK_CURRENT)) * -10;
  if(totalCurrent_001A > 32767) totalCurrent_001A = 32767;
  else if(totalCurrent_001A < -32768) totalCurrent_001A = -32768;
  setBmsTotalCurrent_int(dataMappingNr, static_cast<int16_t>(totalCurrent_001A));

  // SOC in 1%
  uint16_t soc = modbus.getU16Value(REG_SOC);
  if(soc > 100) soc = 100;
  setBmsChargePercentage(dataMappingNr, static_cast<uint8_t>(soc));

  // 1C signed => internes Temperaturformat 0.1C
  setBmsTempatureI16(dataMappingNr, 0, static_cast<int16_t>(modbus.getI16Value(REG_AMBIENT_TEMP) * 100));
  for(uint8_t i = 1; i < NR_OF_BMS_TEMP_SENSORS && i <= (REG_CELL_TEMPERATURE_4 - REG_CELL_TEMPERATURE_1 + 1); i++)
  {
    setBmsTempatureI16(dataMappingNr, i, static_cast<int16_t>(modbus.getI16Value(REG_CELL_TEMPERATURE_1 + (i - 1)) * 100));
  }

  // Standard-MQTT Werte
  mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(dataMappingNr));
  mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(dataMappingNr));
  if(devData->bo_sendMqttMsg) mqttPublish(MQTT_TOPIC_DATA_DEVICE, dataMappingNr, MQTT_TOPIC2_CYCLE, -1, modbus.getU16Value(REG_CYCLE_COUNT));

  const uint16_t statusWordRaw = modbus.getU16Value(REG_STATUS_WORD);
  const uint16_t faultStatusRaw = modbus.getU16Value(REG_FAULT_STATUS);
  const uint32_t mappedErrors = mapStatusBits(faultStatusRaw);
  setBmsStateFETsCharge(dataMappingNr, (statusWordRaw & STATUS_WORD_BIT_CHARGE_ENABLE) > 0);
  setBmsStateFETsDischarge(dataMappingNr, (statusWordRaw & STATUS_WORD_BIT_DISCHARGE_ENABLE) > 0);
  setBmsErrors(dataMappingNr, mappedErrors);
  setBmsWarnings(dataMappingNr, BMS_ERR_STATUS_OK);

  return true;
}
