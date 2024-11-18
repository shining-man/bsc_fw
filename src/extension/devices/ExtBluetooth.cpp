// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "extension/devices/ExtBluetooth.h"
#include "WebSettings.h"
#include "Utility.h"
#include "string.h"

static const char *TAG = "EXT_BT";

ExtBluetooth::ExtBluetooth(uint8_t address) : ExtInterface_I2C(address) { }

ExtBluetooth::~ExtBluetooth() {}


void ExtBluetooth::initialize()
{
  uint8_t u8_lErr = isDeviceAvailable(deviceAddress);
  if (u8_lErr == 0)
  {
    setEnabled(true);
    BSC_LOGI(TAG,"BT Ext. found");

    sendBtDeviceMACs();
  }
  else
  {
    setEnabled(false);
    BSC_LOGI(TAG,"BT Ext. not found (%i)",u8_lErr);
  }
}


void ExtBluetooth::getBtBmsData()
{
  uint8_t dataDeviceNr;
  uint8_t bytesReceived = 0;
  bool dataDevFound = false;

  struct  bmsData_s *p_lBmsData;
  p_lBmsData = getBmsData();

  for(uint8_t bmsNr = 0; bmsNr < BT_EXT_DEVICES_COUNT; bmsNr++)
  {
    for(uint8_t bmsDataTyp = (uint8_t)BMS_CELL_VOLTAGE; bmsDataTyp < (uint8_t)BMS_LAST_DATA_MILLIS + 1; bmsDataTyp++)
    {
      uint8_t rxBuf[50];
      if(getDataFromExtension(deviceAddress, BSC_GET_BT_EXTENSION_BMSDATA, bmsDataTyp, bmsNr, rxBuf, 50))
      {
        // Suche Data-mapping Device
        dataDevFound = false;
        for(uint8_t i = 0; i < MUBER_OF_DATA_DEVICES; i++)
        {
          uint8_t dataDeviceSchnittstelle = (uint8_t)WebSettings::getInt(ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE,i,DT_ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE);
          if(dataDeviceSchnittstelle == bmsNr) 
          {
            dataDevFound = true;
            dataDeviceNr = i;
          }
        }
        if(!dataDevFound) continue;

        switch(rxBuf[1])
        {
          case BMS_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsCellVoltage[dataDeviceNr], &rxBuf[2], 48);
            break;
          case BMS_TOTAL_VOLTAGE:
            memcpy(&p_lBmsData->bmsTotalVoltage[dataDeviceNr], &rxBuf[2], 4);
            break;
          case BMS_MAX_CELL_DIFFERENCE_VOLTAGE:
            memcpy(&p_lBmsData->bmsMaxCellDifferenceVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_AVG_VOLTAGE:
            memcpy(&p_lBmsData->bmsAvgVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_TOTAL_CURRENT:
            memcpy(&p_lBmsData->bmsTotalCurrent[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_MAX_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsMaxCellVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_MIN_CELL_VOLTAGE:
            memcpy(&p_lBmsData->bmsMinCellVoltage[dataDeviceNr], &rxBuf[2], 2);
            break;
          case BMS_MAX_VOLTAGE_CELL_NUMBER:
            memcpy(&p_lBmsData->bmsMaxVoltageCellNumber[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_MIN_VOLTAGE_CELL_NUMBER:
            memcpy(&p_lBmsData->bmsMinVoltageCellNumber[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_IS_BALANCING_ACTIVE:
            memcpy(&p_lBmsData->bmsIsBalancingActive[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_BALANCING_CURRENT:
            memcpy(&p_lBmsData->bmsBalancingCurrent[dataDeviceNr], &rxBuf[2], 4);
            break;
          case BMS_TEMPERATURE:
            memcpy(&p_lBmsData->bmsTempature[dataDeviceNr], &rxBuf[2], 12);
            break;
          case BMS_CHARGE_PERCENT:
            memcpy(&p_lBmsData->bmsChargePercentage[dataDeviceNr], &rxBuf[2], 1);
            break;
          case BMS_ERRORS:
            memcpy(&p_lBmsData->bmsErrors[dataDeviceNr], &rxBuf[2], 4);
            break;
          case BMS_LAST_DATA_MILLIS:
            if(rxBuf[2] == true) p_lBmsData->bmsLastDataMillis[dataDeviceNr] = millis();
            break;
          default:
            break;
        }
      }
      else I2cRxSemaphoreGive();
    }
  }
}


void ExtBluetooth::getNeeySettings()
{
  if(!isEnabled()) return;

  //BSC_LOGI(TAG, "getNeeySettings()");

  for(uint8_t bmsNr = 0; bmsNr < BT_EXT_DEVICES_COUNT; bmsNr++)
  {
    // ToDo: Üperprüfen ob NEEY

    uint8_t rxBuf[50];
    if(getDataFromExtension(deviceAddress, BSC_GET_BT_EXTENSION_DATA, BSC_BT_EXT_GET_SETTINGS, bmsNr, rxBuf, 50))
    {
      memcpy(getBmsSettingsReadback(bmsNr), &rxBuf[2], 32);

     //BSC_LOGI(TAG, "getDataFromExtension(); A dev=%i", bmsNr);
     //buffer2Log(rxBuf, 34);
    }
  }
}


void ExtBluetooth::startBtScan()
{
  uint8_t data[] = {BSC_SET_BT_EXTENSION_DATA, BSC_BT_START_SCAN, 0, 0};
  i2cWriteBytes(deviceAddress, data, sizeof(data) / sizeof(data[0]));
}


void ExtBluetooth::getScanBtMACs()
{
  if(!isEnabled()) return;

  //BSC_LOGI(TAG, "getScanBtMACs()");

  for(uint8_t macNr = 0; macNr < 20; macNr++)
  {
    uint8_t rxBuf[25];
    if(getDataFromExtension(deviceAddress, BSC_GET_BT_EXTENSION_DATA, BSC_BT_EXT_GET_FOUND_MAC, macNr, rxBuf, 25))
    {
      //BSC_LOGI(TAG, "getScanBtMACs(); A dev=%i", macNr);
      //buffer2Log(rxBuf, 20);

      if(rxBuf[2] == 0)
      {
        //BSC_LOGI(TAG, "getScanBtMACs(); Keine weiteren Devices gefunden (dev=%i)", macNr);
        memset(getBmsDeviceData(macNr)->macAdresse, 0, 6);
      }
      else
      {
        memcpy(getBmsDeviceData(macNr)->macAdresse, &rxBuf[3], 6);  // MAC
        memcpy(getBmsDeviceData(macNr)->devName, &rxBuf[9], 10);    // Name
      }
    }
  }
  
  //BtScanDevices2Log();
}


void ExtBluetooth::sendNeeySettings()
{
  if(!isEnabled()) return;

  NeeySettings_s settings;
  uint8_t data[4 + sizeof(NeeySettings_s)];

  for(uint8_t bmsNr = 0; bmsNr < BT_EXT_DEVICES_COUNT; bmsNr++)
  {
    // ToDo: Üperprüfen ob NEEY

    settings.balancerOn = (uint8_t)WebSettings::getIntFlash(ID_PARAM_NEEY_BALANCER_ON, bmsNr, DT_ID_PARAM_NEEY_BALANCER_ON);
    settings.cellCount = (uint8_t)WebSettings::getIntFlash(ID_PARAM_NEEY_CELLS, bmsNr, DT_ID_PARAM_NEEY_CELLS);
    settings.batteryType = (uint8_t)WebSettings::getIntFlash(ID_PARAM_NEEY_BAT_TYPE, bmsNr, DT_ID_PARAM_NEEY_BAT_TYPE);
    settings.batteryCapacity = (uint16_t)WebSettings::getIntFlash(ID_PARAM_NEEY_BAT_CAPACITY, bmsNr, DT_ID_PARAM_NEEY_BAT_CAPACITY);
    settings.startVoltage = WebSettings::getFloatFlash(ID_PARAM_NEEY_START_VOLTAGE, bmsNr);
    settings.maxBalanceCurrent = WebSettings::getFloatFlash(ID_PARAM_NEEY_MAX_BALANCE_CURRENT, bmsNr);
    settings.sleepVoltage = WebSettings::getFloatFlash(ID_PARAM_NEEY_SLEEP_VOLTAGE, bmsNr);
    settings.equalizationVoltage = WebSettings::getFloatFlash(ID_PARAM_NEEY_EQUALIZATION_VOLTAGE, bmsNr);

    data[0] = BSC_SET_BT_EXTENSION_DATA;
    data[1] = BSC_BT_NEEY_SETTINGS;
    data[2] = bmsNr;
    data[3] = 0;

    memcpy(&data[4], &settings, sizeof(NeeySettings_s));
    i2cWriteBytes(deviceAddress, data, sizeof(data) / sizeof(data[0]));
  }
}


void ExtBluetooth::sendBtDeviceMACs()
{
  if(!isEnabled()) return;

  struct  bmsData_s *p_lBmsData;
  p_lBmsData = getBmsData();

  for(uint8_t bmsNr = 0; bmsNr < BT_EXT_DEVICES_COUNT; bmsNr++)
  {
    uint8_t data[11];
    data[0] = BSC_SET_BT_EXTENSION_DATA;
    data[1] = BSC_BT_CONNECT_MACS;
    data[2] = bmsNr;
    data[3] = 0;

    data[4] = WebSettings::getInt(ID_PARAM_SS_BTDEV, bmsNr, DT_ID_PARAM_SS_BTDEV); //Device Typ

    //data[5-9] = MAC
    if(!parseMacAddress(WebSettings::getString(ID_PARAM_SS_BTDEVMAC,bmsNr).c_str(), &data[5]))
    {
      // Fehler beim Parsen der MAC
      BSC_LOGE(TAG, "Fehler beim Parsen der MAC");
    }

    i2cWriteBytes(deviceAddress, data, sizeof(data) / sizeof(data[0]));
  }
}


void ExtBluetooth::sendDataAfterParameterChange()
{
  sendBtDeviceMACs();
}


/* 
 * Ausgabe der gescannten BT-Geräte der BT-Extension
 */
void ExtBluetooth::BtScanDevices2Log()
{
  for (int i = 0; i < BT_DEVICES_COUNT; i++)
  {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
              getBmsDeviceData(i)->macAdresse[0], getBmsDeviceData(i)->macAdresse[1],
              getBmsDeviceData(i)->macAdresse[2], getBmsDeviceData(i)->macAdresse[3],
              getBmsDeviceData(i)->macAdresse[4], getBmsDeviceData(i)->macAdresse[5]);

    ESP_LOGI(TAG, "Device %d: MAC=%s, Name=%s", i, macStr, getBmsDeviceData(i)->devName);
  }
}


std::string ExtBluetooth::getBtScanResultAsHtmlTable()
{
  std::string btDevScanResult = "<table>";

  for (int i = 0; i < 20; i++)
  {
    if(getBmsDeviceData(i)->macAdresse[0] != 0 || getBmsDeviceData(i)->macAdresse[1] != 0)
    {
      char macStr[18];
      snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                getBmsDeviceData(i)->macAdresse[0], getBmsDeviceData(i)->macAdresse[1],
                getBmsDeviceData(i)->macAdresse[2], getBmsDeviceData(i)->macAdresse[3],
                getBmsDeviceData(i)->macAdresse[4], getBmsDeviceData(i)->macAdresse[5]);

      btDevScanResult += "<tr>";

      btDevScanResult += "<td>";
      btDevScanResult += macStr;
      btDevScanResult += "</td>";

      btDevScanResult += "<td>";
      btDevScanResult += getBmsDeviceData(i)->devName;
      btDevScanResult += "</td>";

      btDevScanResult += "<td>";
      btDevScanResult += "<button onclick='copyStringToClipboard(\"";
      btDevScanResult += macStr;
      btDevScanResult += "\")'>Copy</button>";
      btDevScanResult += "</td>";

      btDevScanResult += "</tr>";
    }
  }

  btDevScanResult += "<table>";

  return btDevScanResult;
}