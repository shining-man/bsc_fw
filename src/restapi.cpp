#include <Arduino.h>
#include "restapi.h"
#include "BmsData.h"
#include "Ow.h"



enum genJsonTypes{arrStart,arrEnd,entrySingle,arrStart2,arrStart3,arrEnd2,entrySingle2};

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
  String str_htmlOut="";

  server->setContentLength(CONTENT_LENGTH_UNKNOWN);
  server->send(200, "application/json", "");

  //Start
  genJsonEntryArray(arrStart3, "", "", str_htmlOut, true);

  // BMS Bluetooth 
  genJsonEntryArray(arrStart, F("bms_bt"), "", str_htmlOut, false);
  for(uint8_t bmsDevNr=0;bmsDevNr<BT_DEVICES_COUNT;bmsDevNr++)
  {
    genJsonEntryArray(entrySingle, F("nr"), bmsDevNr, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("cells"), 20, str_htmlOut, false);

    genJsonEntryArray(arrStart2, F("cell_voltage"), "", str_htmlOut, false);
    for(uint8_t i=0;i<19;i++)
    {
      genJsonEntryArray(entrySingle2, "", getBmsCellVoltage(bmsDevNr,i), str_htmlOut, false);
    }
    genJsonEntryArray(entrySingle2, "", getBmsCellVoltage(bmsDevNr,20), str_htmlOut, true);
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
  genJsonEntryArray(arrStart, F("bms_serial"), "", str_htmlOut, false);
  for(uint8_t bmsDevNr=BT_DEVICES_COUNT;bmsDevNr<BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT;bmsDevNr++)
  {
    genJsonEntryArray(entrySingle, F("nr"), bmsDevNr-BT_DEVICES_COUNT, str_htmlOut, false);
    genJsonEntryArray(entrySingle, F("cells"), 20, str_htmlOut, false);

    genJsonEntryArray(arrStart2, F("cell_voltage"), "", str_htmlOut, false);
    uint16_t u16_lZellVoltage=0;
    for(uint8_t i=0;i<19;i++)
    {
      u16_lZellVoltage = getBmsCellVoltage(bmsDevNr,i);
      if(u16_lZellVoltage==0xFFFF)u16_lZellVoltage=0;
      genJsonEntryArray(entrySingle2, "", u16_lZellVoltage, str_htmlOut, false);
    }
    u16_lZellVoltage = getBmsCellVoltage(bmsDevNr,20);
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