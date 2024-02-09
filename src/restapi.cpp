#include <Arduino.h>
#include "restapi.h"
#include "BmsData.h"
#include "Ow.h"
#include "Json.h"
#include "WebSettings.h"
#include "dio.h"
#include "Canbus.h"

static const char* TAG = "REST";

enum genJsonTypes{arrStart,arrEnd,entrySingle,arrStart2,arrStart3,arrStart4,arrEnd2,entrySingle2};

bool handleRestArgs(WebServer * server);


void genJsonEntrySingle(String key, String value, String &str_retStr)
{
  //\"cells\":16,
  str_retStr += "\"";
  str_retStr += String(key);
  str_retStr += "\":\"";
  str_retStr += value;
  str_retStr += "\"";
}

void genJsonEntrySingle(String key, int32_t value, String &str_retStr)
{
  genJsonEntrySingle(key, String(value), str_retStr);
}

void genJsonEntryArray(genJsonTypes type, String key, String value, String &str_retStr, boolean lastEntry)
{
  switch(type)
  {
    case arrStart: //Start
      str_retStr += "\"";
      str_retStr += String(key);
      str_retStr += "\":[{";
      break;

    case arrStart2: //Start2
      str_retStr += "\"";
      str_retStr += String(key);
      str_retStr += "\":[";
      break;

    case arrStart3: //Start3
      str_retStr += "{";
      if(!lastEntry)str_retStr += ",";
      break;

    case arrStart4: //Start4
      str_retStr += "\"";
      str_retStr += String(key);
      str_retStr += "\":{";
      break;

    case entrySingle: //Entry
      genJsonEntrySingle(key, value, str_retStr);
      if(!lastEntry)str_retStr += ",";
      break;

    case entrySingle2: //Entry 2
      str_retStr += value;
      if(!lastEntry)str_retStr += ",";
      break;

    case arrEnd: //End
      str_retStr += "}";
      if(!lastEntry)str_retStr += ",";
      str_retStr += "\n";
      break;

    case arrEnd2: //End 2
      str_retStr += "]";
      if(!lastEntry)str_retStr += ",";
      str_retStr += "\n";
      break;
  }
}

void genJsonEntryArray(genJsonTypes type, String key, int32_t value, String &str_retStr, boolean lastEntry)
{
  genJsonEntryArray(type, key, String(value), str_retStr, lastEntry);
}



const char JSON_BMS_BT_1[] PROGMEM ="\"bms_bt\":[%s]";
const char JSON_BMS_BT_2[] PROGMEM ="\"{\"nr\":%i,\"cells\":%i,\"cell_voltage\":[%s],\"temperature\":[%s]}";


void buildJsonRest(WebServer * server)
{
  if(server->args()>0)
  {
    handleRestArgs(server);
    server->send(200, "application/json", F("{\"state\":1}"));
  }
  else
  {
    String str_htmlOut="";
    uint8_t u8_val=0;

    uint8_t u8_nrOfCells=WebSettings::getInt(ID_PARAM_SERIAL_NUMBER_OF_CELLS,0,DT_ID_PARAM_SERIAL_NUMBER_OF_CELLS);

    server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    server->send(200, "application/json", "");

    //Start
    genJsonEntryArray(arrStart3, "", "", str_htmlOut, true);

    // System
    //"bms_bt":{"en":"1","valid":"0","nr":"0","cells":"16"}
    genJsonEntryArray(arrStart4, F("system"), "", str_htmlOut, true);

    genJsonEntryArray(entrySingle, F("fw_version"), BSC_SW_VERSION, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("fw_add"), BSC_SW_SPEZIAL, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("hw_version"), getHwVersion(), str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("name"), WebSettings::getString(ID_PARAM_MQTT_DEVICE_NAME,0), str_htmlOut, true);

    genJsonEntryArray(arrEnd, "", "", str_htmlOut, false);
    server->sendContent(str_htmlOut);
    str_htmlOut="";

    // Inverter
    genJsonEntryArray(arrStart4, F("inverter"), "", str_htmlOut, true);

    inverterDataSemaphoreTake();
    inverterData_s *inverterData = getInverterData();
    int16_t inverterChargeCurrent = inverterData->inverterChargeCurrent;
    int16_t inverterDischargeCurrent = inverterData->inverterDischargeCurrent;
    int16_t inverterCurrent = inverterData->inverterCurrent;
    int16_t inverterVoltage = inverterData->inverterVoltage;
    uint16_t inverterSoc = inverterData->inverterSoc;

    int16_t calcChargeCurrentCellVoltage = inverterData->calcChargeCurrentCellVoltage;
    int16_t calcChargeCurrentSoc = inverterData->calcChargeCurrentSoc;
    int16_t calcChargeCurrentCelldrift = inverterData->calcChargeCurrentCelldrift;
    int16_t calcChargeCurrentCutOff = inverterData->calcChargeCurrentCutOff;

    int16_t calcDischargeCurrentCellVoltage = inverterData->calcDischargeCurrentCellVoltage;
    inverterDataSemaphoreGive();
    genJsonEntryArray(entrySingle, F("current"), inverterCurrent, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("voltage"), inverterVoltage, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("soc"), inverterSoc, str_htmlOut, false);

    genJsonEntryArray(entrySingle, F("setpoint_cc"), inverterChargeCurrent, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("setpoint_dcc"), inverterDischargeCurrent, str_htmlOut, false);

    genJsonEntryArray(entrySingle, F("cc_cellVoltage"), calcChargeCurrentCellVoltage, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("cc_soc"), calcChargeCurrentSoc, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("cc_cellDrift"), calcChargeCurrentCelldrift, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("cc_cutOff"), calcChargeCurrentCutOff, str_htmlOut, false);

    genJsonEntryArray(entrySingle, F("dcc_cellVoltage"), calcDischargeCurrentCellVoltage, str_htmlOut, true);

    genJsonEntryArray(arrEnd, "", "", str_htmlOut, false);
    server->sendContent(str_htmlOut);
    str_htmlOut="";

    // BMS Bluetooth
    genJsonEntryArray(arrStart, F("bms_bt"), "", str_htmlOut, false);
    for(uint8_t bmsDevNr=0;bmsDevNr<BT_DEVICES_COUNT;bmsDevNr++)
    {
      if(WebSettings::getInt(ID_PARAM_SS_BTDEV,bmsDevNr,DT_ID_PARAM_SS_BTDEV)!=0)
        genJsonEntryArray(entrySingle, F("en"), 1, str_htmlOut, false);
      else genJsonEntryArray(entrySingle, F("en"), 0, str_htmlOut, false);

      if((millis()-getBmsLastDataMillis(bmsDevNr)<5000)) u8_val=1;
      else u8_val=0;
      genJsonEntryArray(entrySingle, F("valid"), u8_val, str_htmlOut, false);

      genJsonEntryArray(entrySingle, F("nr"), bmsDevNr, str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("cells"), u8_nrOfCells, str_htmlOut, false);

      genJsonEntryArray(entrySingle, F("totalVolt"), String(getBmsTotalVoltage(bmsDevNr)), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("totalCurr"), String(getBmsTotalCurrent(bmsDevNr)), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("soc"), getBmsChargePercentage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("maxZellDiff"), getBmsMaxCellDifferenceVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("maxCellVolt"), getBmsMaxCellVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("minCellVolt"), getBmsMinCellVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("maxCellVoltNr"), getBmsMaxVoltageCellNumber(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("minCellVoltNr"), getBmsMinVoltageCellNumber(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("balOn"), getBmsIsBalancingActive(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("FetState"), getBmsStateFETs(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("bmsErr"), getBmsErrors(bmsDevNr), str_htmlOut, false);

      genJsonEntryArray(arrStart2, F("cell_voltage"), "", str_htmlOut, false);
      for(uint8_t i=0;i<(u8_nrOfCells-1);i++)
      {
        genJsonEntryArray(entrySingle2, "", getBmsCellVoltage(bmsDevNr,i), str_htmlOut, false);
      }
      genJsonEntryArray(entrySingle2, "", getBmsCellVoltage(bmsDevNr,u8_nrOfCells-1), str_htmlOut, true);
      genJsonEntryArray(arrEnd2, "", "", str_htmlOut, false);

      genJsonEntryArray(arrStart2, F("temperature"), "", str_htmlOut, false);
      genJsonEntryArray(entrySingle2, "", getBmsTempature(bmsDevNr,0), str_htmlOut, false);
      genJsonEntryArray(entrySingle2, "", getBmsTempature(bmsDevNr,1), str_htmlOut, false);
      genJsonEntryArray(entrySingle2, "", getBmsTempature(bmsDevNr,2), str_htmlOut, true);
      genJsonEntryArray(arrEnd2, "", "", str_htmlOut, true);

      if(bmsDevNr<BT_DEVICES_COUNT-1)
      {
        genJsonEntryArray(arrEnd, "", "", str_htmlOut, false);
        genJsonEntryArray(arrStart3, "", "", str_htmlOut, true);
      }
      else genJsonEntryArray(arrEnd, "", "", str_htmlOut, true);
    }
    genJsonEntryArray(arrEnd2, "", "", str_htmlOut, false);
    server->sendContent(str_htmlOut);
    str_htmlOut="";


    // BMS serial
    uint8_t u8_device, u8_deviceSerial2, u8_deviceSerial2NrOfBms;
    u8_deviceSerial2=WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,2,DT_ID_PARAM_SERIAL_CONNECT_DEVICE);
    u8_deviceSerial2NrOfBms=WebSettings::getInt(ID_PARAM_SERIAL2_CONNECT_TO_ID,0,DT_ID_PARAM_SERIAL2_CONNECT_TO_ID);

    genJsonEntryArray(arrStart, F("bms_serial"), "", str_htmlOut, false);
    for(uint8_t bmsDevNr=BT_DEVICES_COUNT;bmsDevNr<BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT;bmsDevNr++)
    {
      u8_device = WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,bmsDevNr-BT_DEVICES_COUNT,DT_ID_PARAM_SERIAL_CONNECT_DEVICE);
      //BSC_LOGI(TAG,"Restapi: u8_device=%i, u8_deviceSerial2=%i, bmsDevNr=%i, u8_deviceSerial2NrOfBms=%i",u8_device, u8_deviceSerial2, bmsDevNr, u8_deviceSerial2NrOfBms);
      if(u8_device!=0) genJsonEntryArray(entrySingle, F("en"), 1, str_htmlOut, false);
      else if(isMultiple485bms(u8_deviceSerial2) && bmsDevNr>BT_DEVICES_COUNT+2 && bmsDevNr<BT_DEVICES_COUNT+2+u8_deviceSerial2NrOfBms)
        genJsonEntryArray(entrySingle, F("en"), 1, str_htmlOut, false);
      else genJsonEntryArray(entrySingle, F("en"), 0, str_htmlOut, false);

      if((millis()-getBmsLastDataMillis(bmsDevNr)<5000)) u8_val=1;
      else u8_val=0;
      genJsonEntryArray(entrySingle, F("valid"), u8_val, str_htmlOut, false);

      genJsonEntryArray(entrySingle, F("nr"), bmsDevNr-BT_DEVICES_COUNT, str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("cells"), u8_nrOfCells, str_htmlOut, false);

      genJsonEntryArray(entrySingle, F("totalVolt"), String(getBmsTotalVoltage(bmsDevNr)), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("totalCurr"), String(getBmsTotalCurrent(bmsDevNr)), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("soc"), getBmsChargePercentage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("maxZellDiff"), getBmsMaxCellDifferenceVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("maxCellVolt"), getBmsMaxCellVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("minCellVolt"), getBmsMinCellVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("maxCellVoltNr"), getBmsMaxVoltageCellNumber(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("minCellVoltNr"), getBmsMinVoltageCellNumber(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("balOn"), getBmsIsBalancingActive(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("FetState"), getBmsStateFETs(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingle, F("bmsErr"), getBmsErrors(bmsDevNr), str_htmlOut, false);

      genJsonEntryArray(arrStart2, F("cell_voltage"), "", str_htmlOut, false);
      uint16_t u16_lZellVoltage=0;
      for(uint8_t i=0;i<(u8_nrOfCells-1);i++)
      {
        u16_lZellVoltage = getBmsCellVoltage(bmsDevNr,i);
        if(u16_lZellVoltage==0xFFFF)u16_lZellVoltage=0;
        genJsonEntryArray(entrySingle2, "", u16_lZellVoltage, str_htmlOut, false);
      }
      u16_lZellVoltage = getBmsCellVoltage(bmsDevNr,u8_nrOfCells-1);
      if(u16_lZellVoltage==0xFFFF)u16_lZellVoltage=0;
      genJsonEntryArray(entrySingle2, "", u16_lZellVoltage, str_htmlOut, true);
      genJsonEntryArray(arrEnd2, "", "", str_htmlOut, false);

      genJsonEntryArray(arrStart2, F("temperature"), "", str_htmlOut, false);
      genJsonEntryArray(entrySingle2, "", getBmsTempature(bmsDevNr,0), str_htmlOut, false);
      genJsonEntryArray(entrySingle2, "", getBmsTempature(bmsDevNr,1), str_htmlOut, false);
      genJsonEntryArray(entrySingle2, "", getBmsTempature(bmsDevNr,2), str_htmlOut, true);
      genJsonEntryArray(arrEnd2, "", "", str_htmlOut, true);

      if(bmsDevNr<BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT-1)
      {
        genJsonEntryArray(arrEnd, "", "", str_htmlOut, false);
        genJsonEntryArray(arrStart3, "", "", str_htmlOut, true);
      }
      else genJsonEntryArray(arrEnd, "", "", str_htmlOut, true);
    }
    genJsonEntryArray(arrEnd2, "", "", str_htmlOut, false);
    server->sendContent(str_htmlOut);
    str_htmlOut="";


    // onewire Temperature
    genJsonEntryArray(arrStart2, F("temperature"), "", str_htmlOut, false);
    for(uint8_t i=0;i<63;i++)
    {
      genJsonEntryArray(entrySingle2, "", String(owGetTemp(i)), str_htmlOut, false);
    }
    genJsonEntryArray(entrySingle2, "", String(owGetTemp(63)), str_htmlOut, true);
    genJsonEntryArray(arrEnd2, "", "", str_htmlOut, true);


    //Ende
    genJsonEntryArray(arrEnd, "", "", str_htmlOut, true);

    server->sendContent(str_htmlOut);
    str_htmlOut="";
  }
}

#ifdef UTEST_RESTAPI
uint8_t u8_activeBms=0;
uint8_t u8_activeCellNr=0;
#endif
bool handleRestArgs(WebServer * server)
{
  WebSettings ws;
  bool ret=true;
  String argName, argValue;

  for(uint8_t i=0;i<server->args();i++)
  {
    argName = server->argName(i);
    argValue = server->arg(i);

    #ifndef UTEST_RESTAPI
    BSC_LOGI(TAG,"%s=%s",argName.c_str(), argValue.c_str());
    #endif

    if(argName==F("save")) ws.writeConfig();
    else if(argName==F("setInvMaxChgCur")) ws.setParameter(ID_PARAM_BMS_MAX_CHARGE_CURRENT, 0, argValue, DT_ID_PARAM_BMS_MAX_CHARGE_CURRENT);
    else if(argName==F("setInvMaxDisChgCur")) ws.setParameter(ID_PARAM_BMS_MAX_DISCHARGE_CURRENT, 0, argValue, DT_ID_PARAM_BMS_MAX_DISCHARGE_CURRENT);


    #ifdef UTEST_RESTAPI
    else if(argName==F("setBms")) {u8_activeBms=(uint8_t)argValue.toInt();}
    else if(argName==F("cellNr")) {u8_activeCellNr=(uint8_t)argValue.toInt();}
    else if(argName==F("soc")) {setBmsChargePercentage(u8_activeBms,argValue.toInt()); setBmsLastDataMillis(u8_activeBms,millis());}
    else if(argName==F("cellV")) {setBmsCellVoltage(u8_activeBms,u8_activeCellNr,argValue.toInt()); setBmsLastDataMillis(u8_activeBms,millis());}
    #endif
    else ret=false;
  }

  return ret;
}


/*
void handle_setParameter(WebServer * server)
{
  if (server->hasArg("plain") == false) {
    server->send(200, "application/json", F("{\"state\":0}"));
    return;
  }

  Json json;
  const char *jsonData = server->arg("plain").c_str();

  uint16_t arrSize = json.getArraySize(jsonData, 0);

  uint32_t searchStartPos=0;
  String retName;
  String retVal;
  for(uint8_t i=0; i<arrSize;i++)
  {
    json.getValue(jsonData, i, "name", searchStartPos, retVal, searchStartPos);
    json.getValue(jsonData, i, "val", searchStartPos, retVal, searchStartPos);
    BSC_LOGI(TAG,"%s=%s",retName.c_str(), retVal.c_str());
  }



  server->send(200, "application/json", F("{\"state\":1}"));
}
*/
