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

static void parsePackInfo(modbusrtu::ModbusRTU *modbus, uint8_t devNr);

//static void message2Log(uint8_t * t_message, uint8_t len, uint8_t address);

static serialDevData_s *mDevData;

bool JkInverterBms_readBmsData(Stream *port, uint8_t devNr, void (*callback)(uint8_t, uint8_t), serialDevData_s *devData)
{
  mDevData = devData;
  uint8_t u8_mCountOfPacks = devData->u8_NumberOfDevices;
  uint8_t response[JK_INVERTER_BMS_MAX_ANSWER_LEN];

  uint8_t jkInvBmsAdr = devData->u8_deviceNr;
  uint8_t jkInvBmsAdrBmsData = devData->u8_BmsDataAdr + BT_DEVICES_COUNT;

  if(u8_mCountOfPacks>1) jkInvBmsAdr += 1;
  
  #ifdef JK_INV_DEBUG
  BSC_LOGI(TAG,"JkInverterBms_readBmsData() devNr=%i, readFromAdr=%i, BmsDataAdr=%i, CountOfPacks=%i",u8_mDevNr,jkInvBmsAdr,jkInvBmsAdrBmsData,u8_mCountOfPacks);
  #endif

  modbusrtu::ModbusRTU modbus(port,callback,devNr);

  if(modbus.readData(jkInvBmsAdr, modbusrtu::ModbusRTU::fCode::READ_CMD_JK, 0x1200, 268, response))
  {
    //message2Log(response, 36, 0);
    parsePackInfo(&modbus, jkInvBmsAdrBmsData);

    // MQTT
    mqttPublish(MQTT_TOPIC_BMS_BT, jkInvBmsAdrBmsData, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, getBmsTotalVoltage(jkInvBmsAdrBmsData));
    mqttPublish(MQTT_TOPIC_BMS_BT, jkInvBmsAdrBmsData, MQTT_TOPIC2_TOTAL_CURRENT, -1, getBmsTotalCurrent(jkInvBmsAdrBmsData));

    vTaskDelay(pdMS_TO_TICKS(25));
  }
  else return false;


  return true;
}


static void parsePackInfo(modbusrtu::ModbusRTU *modbus, uint8_t devNr)
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

  // Total Voltage
  setBmsTotalVoltage_int(devNr, modbus->getU32ValueByOffset(144)/10);

  // Total Current
  setBmsTotalCurrent_int(devNr, modbus->getI32ValueByOffset(152)/10);

  // Temperature
  setBmsTempatureI16(devNr, 0, modbus->getI16ValueByOffset(156)*10);
  setBmsTempatureI16(devNr, 1, modbus->getI16ValueByOffset(158)*10);

  // Alarm
  // ToDo

  // Bal. Current
  setBmsBalancingCurrentI16(devNr, modbus->getI16ValueByOffset(164)/10);

  // Bal. State
  setBmsIsBalancingActive(devNr, modbus->getU8ValueByOffset(166));

  // SoC
  setBmsChargePercentage(devNr, modbus->getU8ValueByOffset(167));

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