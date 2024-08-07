#include <Arduino.h>
#include "restapi.h"
#include "inverter/Inverter.hpp"
#include "BmsData.h"
#include "Ow.h"
#include "Json.h"
#include "WebSettings.h"
#include "dio.h"
#include "AlarmRules.h"
#include "webUtility.h"

static const char* TAG = "REST";

enum genJsonTypes{arrStart,arrEnd,entrySingle,entrySingleNumber,arrStart2,arrStart3,arrStart4,arrEnd2,entrySingle2};

bool handleRestArgs(WebServer &server, WebSettings &ws);


void genJsonEntrySingle(String key, String value, String &str_retStr)
{
  //\"cells\":16,
  str_retStr += "\"";
  str_retStr += String(key);
  str_retStr += "\":\"";
  str_retStr += value;
  str_retStr += "\"";
}

void genJsonEntrySingleNumber(String key, String value, String &str_retStr)
{
  //\"cells\":16,
  str_retStr += "\"";
  str_retStr += String(key);
  str_retStr += "\":";
  str_retStr += value;
  str_retStr += "";
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

    case entrySingleNumber: //Entry
      genJsonEntrySingleNumber(key, value, str_retStr);
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

void genJsonEntryArray(genJsonTypes type, String key, uint32_t value, String &str_retStr, boolean lastEntry)
{
  genJsonEntryArray(type, key, String(value), str_retStr, lastEntry);
}

void genJsonEntryArray(genJsonTypes type, String key, float value, String &str_retStr, boolean lastEntry)
{
  //BSC_LOGI(TAG,"value=%f", value);
  genJsonEntryArray(type, key, String(value, 2), str_retStr, lastEntry);
}



//const char JSON_BMS_BT_1[] PROGMEM ="\"bms_bt\":[%s]";
//const char JSON_BMS_BT_2[] PROGMEM ="\"{\"nr\":%i,\"cells\":%i,\"cell_voltage\":[%s],\"temperature\":[%s]}";


void buildJsonRest(Inverter &inverter, WebServer &server, WebSettings &ws)
{
  if(server.args()>0)
  {
    if(!performAuthentication(server, ws))
    {
      server.send(200, "application/json", F("{\"state\":0}"));
      return;
    }

    handleRestArgs(server, ws);
    server.send(200, "application/json", F("{\"state\":1}"));
  }
  else
  {
    String str_htmlOut;
    str_htmlOut.reserve(2048);

    uint8_t u8_val=0;

    uint8_t u8_nrOfCells=WebSettings::getInt(ID_PARAM_SERIAL_NUMBER_OF_CELLS,0,DT_ID_PARAM_SERIAL_NUMBER_OF_CELLS);

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "application/json", "");

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
    server.sendContent(str_htmlOut);
    str_htmlOut="";

    // Trigger
    genJsonEntryArray(arrStart4, F("trigger"), "", str_htmlOut, true);

    for(uint8_t i=0;i<9;i++) genJsonEntryArray(entrySingleNumber, String(i), getAlarm(i), str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, String(9), getAlarm(9), str_htmlOut, true);

    genJsonEntryArray(arrEnd, "", "", str_htmlOut, false);
    server.sendContent(str_htmlOut);
    str_htmlOut="";

    // Inverter
    genJsonEntryArray(arrStart4, F("inverter"), "", str_htmlOut, true);

    inverter.inverterDataSemaphoreTake();
    Inverter::inverterData_s *inverterData = inverter.getInverterData();
    float inverterChargeVoltage = (float)inverterData->inverterChargeVoltage / 10.0f;
    float inverterChargeCurrent = (float)inverterData->inverterChargeCurrent / 10.0f;
    float inverterDischargeCurrent = (float)inverterData->inverterDischargeCurrent / 10.0f;
    float inverterCurrent = (float)inverterData->batteryCurrent / 10.0f;
    float inverterVoltage = (float)inverterData->batteryVoltage / 100.0f;
    uint16_t inverterSoc = inverterData->inverterSoc;

    float calcChargeCurrentCellVoltage = (float)inverterData->calcChargeCurrentCellVoltage / 10.0f;
    float calcChargeCurrentSoc = (float)inverterData->calcChargeCurrentSoc / 10.0f;
    float calcChargeCurrentCelldrift = (float)inverterData->calcChargeCurrentCelldrift / 10.0f;
    float calcChargeCurrentCutOff = (float)inverterData->calcChargeCurrentCutOff / 10.0f;

    float calcDischargeCurrentCellVoltage = (float)inverterData->calcDischargeCurrentCellVoltage / 10.0f;
    inverter.inverterDataSemaphoreGive();

    genJsonEntryArray(entrySingleNumber, F("current"), inverterCurrent, str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, F("voltage"), inverterVoltage, str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, F("soc"), inverterSoc, str_htmlOut, false);

    genJsonEntryArray(entrySingleNumber, F("setpoint_cv"), inverterChargeVoltage, str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, F("setpoint_cc"), inverterChargeCurrent, str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, F("setpoint_dcc"), inverterDischargeCurrent, str_htmlOut, false);

    genJsonEntryArray(entrySingleNumber, F("cc_cellVoltage"), calcChargeCurrentCellVoltage, str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, F("cc_soc"), calcChargeCurrentSoc, str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, F("cc_cellDrift"), calcChargeCurrentCelldrift, str_htmlOut, false);
    genJsonEntryArray(entrySingleNumber, F("cc_cutOff"), calcChargeCurrentCutOff, str_htmlOut, false);

    genJsonEntryArray(entrySingleNumber, F("dcc_cellVoltage"), calcDischargeCurrentCellVoltage, str_htmlOut, true);

    genJsonEntryArray(arrEnd, "", "", str_htmlOut, false);
    server.sendContent(str_htmlOut);
    str_htmlOut="";

    // Data devices
    uint8_t u8_device;

    genJsonEntryArray(arrStart, F("data_device"), "", str_htmlOut, false);
    for(uint8_t bmsDevNr=0; bmsDevNr < MUBER_OF_DATA_DEVICES; bmsDevNr++)
    {
      u8_device = (uint8_t)WebSettings::getInt(ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE, bmsDevNr, DT_ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE);

      genJsonEntryArray(entrySingle, F("name"), WebSettings::getString(ID_PARAM_DEVICE_MAPPING_NAME, bmsDevNr), str_htmlOut, false);

      if(u8_device < MUBER_OF_DATA_DEVICES) genJsonEntryArray(entrySingleNumber, F("en"), 1, str_htmlOut, false);
      else genJsonEntryArray(entrySingleNumber, F("en"), 0, str_htmlOut, false);

      if((millis()-getBmsLastDataMillis(bmsDevNr)<5000)) u8_val=1;
      else u8_val=0;
      genJsonEntryArray(entrySingleNumber, F("valid"), u8_val, str_htmlOut, false);

      genJsonEntryArray(entrySingleNumber, F("nr"), bmsDevNr, str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("cells"), u8_nrOfCells, str_htmlOut, false);

      genJsonEntryArray(entrySingleNumber, F("totalVolt"), String(getBmsTotalVoltage(bmsDevNr)), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("totalCurr"), String(getBmsTotalCurrent(bmsDevNr)), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("soc"), getBmsChargePercentage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("maxCellDiff"), getBmsMaxCellDifferenceVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("maxCellVolt"), getBmsMaxCellVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("minCellVolt"), getBmsMinCellVoltage(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("maxCellVoltNr"), getBmsMaxVoltageCellNumber(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("minCellVoltNr"), getBmsMinVoltageCellNumber(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("balOn"), getBmsIsBalancingActive(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("FetState"), getBmsStateFETs(bmsDevNr), str_htmlOut, false);
      genJsonEntryArray(entrySingleNumber, F("bmsErr"), getBmsErrors(bmsDevNr), str_htmlOut, false);

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

      if(bmsDevNr < MUBER_OF_DATA_DEVICES-1)
      {
        genJsonEntryArray(arrEnd, "", "", str_htmlOut, false);
        genJsonEntryArray(arrStart3, "", "", str_htmlOut, true);
      }
      else genJsonEntryArray(arrEnd, "", "", str_htmlOut, true);

      server.sendContent(str_htmlOut);
      str_htmlOut="";
    }
    genJsonEntryArray(arrEnd2, "", "", str_htmlOut, false);
    server.sendContent(str_htmlOut);
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

    server.sendContent(str_htmlOut);
    str_htmlOut="";
    str_htmlOut.clear();
  }
}

#ifdef UTEST_RESTAPI
uint8_t u8_activeBms=0;
uint8_t u8_activeCellNr=0;
uint16_t u16_paramNr=0;
uint8_t u8_groupNr=0;
uint8_t u8_dt=0;
#endif
bool handleRestArgs(WebServer &server, WebSettings &ws)
{
  bool ret=true;
  String argName, argValue;

  for(uint8_t i=0;i<server.args();i++)
  {
    argName = server.argName(i);
    argValue = server.arg(i);

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

    else if(argName==F("param")) {u16_paramNr=(uint16_t)argValue.toInt();}
    else if(argName==F("group")) {u8_groupNr=(uint8_t)argValue.toInt();}
    else if(argName==F("dt"))
    {
      if(argValue=="U8") u8_dt=PARAM_DT_U8;
      else if(argValue=="I8") u8_dt=PARAM_DT_I8;
      else if(argValue=="U16") u8_dt=PARAM_DT_U16;
      else if(argValue=="I16") u8_dt=PARAM_DT_I16;
      else if(argValue=="U32") u8_dt=PARAM_DT_U32;
      else if(argValue=="I32") u8_dt=PARAM_DT_I32;
      else if(argValue=="FL") u8_dt=PARAM_DT_FL;
      else if(argValue=="ST") u8_dt=PARAM_DT_ST;
      else if(argValue=="BO") u8_dt=PARAM_DT_BO;
    }
    else if(argName==F("val")) ws.setParameter(u16_paramNr, u8_groupNr, argValue, u8_dt);
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
