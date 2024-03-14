// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "WebSettings.h"
#include "web/webSettings_web.h"
#include "defines.h"
#include <Arduino.h>
#include <FS.h>
#ifdef USE_LittleFS
  #define SPIFFS LittleFS
  #include <LittleFS.h>
#else
  #include <SPIFFS.h>
#endif
#include <WebServer.h>
#include <FS.h>
#include "log.h"
#include <sparsepp/spp.h>
#include <Preferences.h>
#include "crc.h"

using spp::sparse_hash_map;

static const char * TAG = "WEB_SETTINGS";

static SemaphoreHandle_t mParamMutex = NULL;

static sparse_hash_map<uint16_t, int8_t> settingValues_i8;
static sparse_hash_map<uint16_t, int16_t> settingValues_i16;
static sparse_hash_map<uint16_t, int32_t> settingValues_i32;
static sparse_hash_map<uint16_t, float> settingValues_fl;
static sparse_hash_map<uint16_t, bool> settingValues_bo;
static sparse_hash_map<uint16_t, std::string> settingValues_str;


static char _buf[2000];
static String st_mSendBuf = "";
static String str_lTmpGetString;
std::vector<String> mOptions;
std::vector<String> mOptionLabels;
bool bo_hasNewKeys=false;

uint8_t u8_mAktOptionGroupNr;
static Json json;

static Preferences prefs;

const char* CONST_options PROGMEM = "options";
const char* CONST_depVal PROGMEM = "depVal";


//HTML templates
const char HTML_START[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang='de'>
<head>
<meta http-equiv='Content-Type' content='text/html' charset='utf-8'/>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BSC</title>
)rawliteral";

const char HTML_START_2[] PROGMEM = R"rawliteral(
</head>
<body>
<div class="topnav">
  <span class='btnBack' onclick=location.href='../'>&#10094;</span>
  <span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>
  <span class='hl'>%s</span>
</div>
<div id='lc' class="loading-container"><div class="loading-spinner"></div></div>
<div class="content">
<form>
<table>
)rawliteral";

const char HTML_END_1[] PROGMEM = R"rawliteral(
</table>
</form>
)rawliteral";

const char HTML_END_2[] PROGMEM = R"rawliteral(
</div>
<br><div id='data_div'></div>
)rawliteral";

const char HTML_END_3[] PROGMEM = R"rawliteral(
</body>
</html>
)rawliteral";

const char HTML_ENTRY_TEXTFIELD[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><input type='%s' value='%s' name='%s'>&nbsp;%s</td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n";
const char HTML_ENTRY_AREA[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><textarea rows='%i' cols='%i' name='%s'>%s</textarea></td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n"; //
const char HTML_ENTRY_FLOAT[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><input type='number' step='%s' min='%i' max='%i' value='%s' name='%s'>&nbsp;%s</td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n";
const char HTML_ENTRY_FLOAT_X[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><input type='number' step='%s' min='%i' max='%i' value='%s' name='%s' class='%s'>&nbsp;%s</td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n";
const char HTML_ENTRY_NUMBER[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><input type='number' min='%i' max='%i' value='%s' name='%s'>&nbsp;%s</td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n";
const char HTML_ENTRY_RANGE[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'>%i&nbsp;<input type='range' min='%i' max='%i' value='%s' name='%s'>&nbsp;%i</td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n";
const char HTML_ENTRY_CHECKBOX[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td><td class='Ctd'><input type='checkbox' %s name='%s'></td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n";

const char HTML_ENTRY_SELECT_START[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><select name='%s'>\n";
const char HTML_ENTRY_SELECT_OPTION[] PROGMEM =
"<option value='%s' %s>%s</option>\n";
const char HTML_ENTRY_SELECT_END[] PROGMEM =
"</select></td><td class='t1'></td><td class='Ctd'><span class='secVal' id='s%s'></span></td></tr>\n";

const char HTML_ENTRY_MULTI_START[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><fieldset style='text-align:left;'>\n";
const char HTML_ENTRY_MULTI_OPTION[] PROGMEM =
"<input type='checkbox' name='%s' value='%i' %s>%s<br>\n";
const char HTML_ENTRY_MULTI_END[] PROGMEM =
"</fieldset></td><td class='t1'></td></tr>\n";  //<td class='Ctd'><span class='secVal' id='s%s'></span></td>

const char HTML_ENTRY_MULTI_COLLAPSIBLE_START[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'>\n"
"<input id='t%i' class='toggle' type='checkbox'>\n"
"<label for='t%i' class='lbl-toggle'>%s</label>\n"
"<div class='collapsible-content'>\n"
"<div class='content-inner'>\n"
"<fieldset style='text-align:left;'>\n";
const char HTML_ENTRY_MULTI_COLLAPSIBLE_END[] PROGMEM =
"</fieldset></div></div></td><td class='t1'></td></tr>\n";

const char HTML_GROUP_START[] PROGMEM = "<tr><td class='Ctd2' colspan='3'><b>%s</b></td></tr>\n";
const char HTML_GROUP_START_DETAILS[] PROGMEM = "</table><details><summary><b>%s</b></summary><table>\n"; //<details open>
const char HTML_GROUP_END_DETAILS[]   PROGMEM = "</table></details><table>\n";

const char HTML_ENTRY_SEPARATION[] PROGMEM = "<tr class='Ctr'><td class='sep' colspan='3'><b><u>%s</u></b></td></tr>\n";

const char HTML_BUTTON[] PROGMEM = "<div class='Ctd'><button id='%s' onclick='btnClick(\"%s\")'>%s</button></div>\n";


bool isNumber(const String& str)
{
  for (char const &c : str) {
    if(std::isdigit(c)==0 && c!='-') return false;
  }
  return true;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++)
  {
    if(data.charAt(i)==separator || i==maxIndex)
    {
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

WebSettings::WebSettings() {
  u8_mAktOptionGroupNr=0;
  mParamMutex = xSemaphoreCreateMutex();

  str_mConfName = "";
  str_mAjaxGetDataTimerHandlerName = "";
  u16_mAjaxGetDataTimerSec = 0;
};


void WebSettings::initWebSettings(const char *parameter, String confName, String configfile)
{
  static bool paramFileRead=false;
  u8_mJsonArraySize = 0;
  str_mConfName = confName.c_str();
  str_mConfigfile = configfile.c_str();

  parameterFile = parameter;

  if (!SPIFFS.begin())
  {
    BSC_LOGE(TAG,"Mount Failed");
    SPIFFS.format();
    SPIFFS.begin();
  }

  if (!prefs.begin("prefs"))
  {
    #ifdef WEBSET_DEBUG
    BSC_LOGE(TAG,"Fehler beim Oeffnen des NVS-Namespace");
    #endif
  }
  else
  {
    BSC_LOGI(TAG,"Free flash entries: %d", prefs.freeEntries());
  }

  if(paramFileRead==false) //Parameterfile nur einmal einlesen
  {
    paramFileRead=true;
    readConfig();
  }
  getDefaultValuesFromNewKeys(parameterFile, 8);

  if(bo_hasNewKeys)
  {
    writeConfig();
    bo_hasNewKeys = false;
  }

  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG, "initWebSettings() end");
  #endif
}

void WebSettings::setTimerHandlerName(String handlerName, uint16_t timerSec)
{
  str_mAjaxGetDataTimerHandlerName = handlerName;
  u16_mAjaxGetDataTimerSec = timerSec;
}

void WebSettings::sendContentHtml(WebServer *server, const char *buf, bool send)
{
  //send==true - > buf wird jetzt gesendet
  //send==false -> buf wird angehängt bis max. Größe erreicht ist

  if(strlen(buf)>500)
  {
    send=true;
  }
  else
  {
    st_mSendBuf += buf;
  }

  if(send==true || st_mSendBuf.length()>2000)
  {
    if(!st_mSendBuf.equals("")) server->sendContent(st_mSendBuf);
    st_mSendBuf="";
  }

  if(strlen(buf)>500)
  {
    server->sendContent(buf);
  }
}

//Function to response HTTP request from the form to save data
void WebSettings::handleHtmlFormRequest(WebServer * server)
{
  uint8_t ui8_optionsCnt;
  std::vector<String> options;
  std::vector<String> optionLabels;

  if(json.getArraySize(parameterFile, 8)==0)
  {
    BSC_LOGI(TAG,"Error read json");
  }
  else
  {
    boolean exit = false;
    if (server->hasArg(F("SAVE")))
    {
      for(uint8_t i=0; i<server->args(); i++)
      {
        String argName = server->argName(i);
        if(argName!="SAVE")
        {
          String argValue = server->arg(i);
          #ifdef WEBSET_DEBUG
          BSC_LOGD(TAG, "SAVE: name:%s, val:%s", argName, argValue);
          #endif

          if(isNumber(argName)) //Wenn keine Zahl, dann Fehler
          {
            char* end;
            uint64_t u64_argName=strtoull(argName.c_str(),&end,10);
            uint32_t u32_argName = (uint32_t)(u64_argName&0xFFFFFFFF);
            uint8_t  u8_datatype = ((u64_argName>>32)&0xff);

            if((u64_argName&((uint64_t)1<<40))==((uint64_t)1<<40)) //Store  in Flash
            {
              #ifdef WEBSET_DEBUG
              BSC_LOGD(TAG,"Store in Flash; u32_argName=%i, dt=%i",u32_argName,u8_datatype);
              #endif

              //Spezielle Parameter vor dem Speichern ändern
              uint16_t u16_lParamId=0;
              uint8_t u8_lParamGroup=0;
              getIdFromParamId(u32_argName,u16_lParamId,u8_lParamGroup);
              if(u16_lParamId==ID_PARAM_SS_BTDEVMAC) argValue.toLowerCase();

              uint8_t ret=0xFF;
              switch(u8_datatype)
              {
                case PARAM_DT_U8:
                  ret=prefs.putChar(String(u32_argName).c_str(),(uint8_t)argValue.toInt());
                  break;
                case PARAM_DT_I8:
                  ret=prefs.putChar(String(u32_argName).c_str(),(int8_t)argValue.toInt());
                  break;
                case PARAM_DT_U16:
                  ret=prefs.putInt(String(u32_argName).c_str(),(uint16_t)argValue.toInt());
                  break;
                case PARAM_DT_I16:
                  ret=prefs.putInt(String(u32_argName).c_str(),(int16_t)argValue.toInt());
                  break;
                case PARAM_DT_U32:
                  ret=prefs.putLong(String(u32_argName).c_str(),argValue.toInt());
                  break;
                case PARAM_DT_I32:
                  ret=prefs.putLong(String(u32_argName).c_str(),argValue.toInt());
                  break;
                case PARAM_DT_FL:
                  ret=prefs.putFloat(String(u32_argName).c_str(),argValue.toFloat());
                  break;
                case PARAM_DT_ST:
                  ret=prefs.putString(String(u32_argName).c_str(),argValue);
                  break;
                case PARAM_DT_BO:
                  ret=prefs.putBool(String(u32_argName).c_str(),argValue.toInt());
                  break;
              }
            }
            else //Store in RAM
            {
              #ifdef WEBSET_DEBUG
              BSC_LOGD(TAG,"Store in RAM; u32_argName=%i, dt=%i",u32_argName,u8_datatype);
              #endif

              setString(u32_argName, argValue, u8_datatype);
              writeConfig(); //Schreiben der Einstellungen in das Config file
            }
            server->send(200, "text/html", "OK"); //Sende ok an Website (Rückmeldung, dass gespeichert wurde)
          }
        }
      }

      //Lesen der Values von der Webseite
      //readWebValues(server, parameterFile, 0);

      if (fn_mOnButtonSave)
      {
        fn_mOnButtonSave();
        exit = true;
      }
      return;
    }
    if (server->hasArg(F("BTN1")) && fn_mOnButton1)
    {
      fn_mOnButton1();
      exit = true;
    }
    if (server->hasArg(F("BTN2")) && fn_mOnButton2)
    {
      fn_mOnButton2();
      exit = true;
    }
    if (server->hasArg(F("BTN3")) && fn_mOnButton3)
    {
      fn_mOnButton3();
      exit = true;
    }

    //Senden der HTML Seite
    if(!exit)
    {
      server->setContentLength(CONTENT_LENGTH_UNKNOWN);
      sprintf(_buf,HTML_START/*,BACKGROUND_COLOR*/);
      server->send(200, "text/html", _buf);

      sendContentHtml(server,webSettingsStyle,false);

      sprintf(_buf,HTML_START_2,str_mConfName.c_str());
      sendContentHtml(server,_buf,false);

      buildSendHtml(server, parameterFile, 8);

      sendContentHtml(server,HTML_END_1,false);

      //Buttons einfügen
      if ((u8_mButtons & 0x02) == 0x02) //Button 1
      {
        sprintf(_buf,HTML_BUTTON,"btn1","BTN1",str_mButton1Text.c_str());
        sendContentHtml(server,_buf,false);
      }
      if ((u8_mButtons & 0x04) == 0x04) //Button 2
      {
        sprintf(_buf,HTML_BUTTON,"btn2","BTN2",str_mButton2Text.c_str());
        sendContentHtml(server,_buf,false);
      }
      if ((u8_mButtons & 0x08) == 0x08) //Button 3
      {
        sprintf(_buf,HTML_BUTTON,"btn3","BTN3",str_mButton3Text.c_str());
        sendContentHtml(server,_buf,false);
      }

      sendContentHtml(server,HTML_END_2,false);
      sendContentHtml(server,webSettingsScript,false);

      //Timer bei bedarf einfügen
      if(!str_mAjaxGetDataTimerHandlerName.equals(""))
      {
        sprintf(_buf,webSettingsScript_Timer,str_mAjaxGetDataTimerHandlerName.c_str(),u16_mAjaxGetDataTimerSec);
        sendContentHtml(server,_buf,false);
      }

      sendContentHtml(server,HTML_END_3,true);
    }
  }
}

/*not use*/
#if 0
void WebSettings::readWebValues(WebServer * server, const String *parameter, uint32_t jsonStartPos)
{
  uint8_t g, optionGroupSize;
  String tmpStr = "";

  BSC_LOGD(tag,"SAVE readWebValues()");

  for (uint8_t a=0; a<json.getArraySize(parameter, jsonStartPos); a++)
  {
    uint8_t type, optionCnt, jsonSize;
    uint32_t jsonName, jsonNameBase;

    type = getJsonType(parameter, a, jsonStartPos);
    optionCnt = getJsonArrayCnt(parameter, CONST_options, a, jsonStartPos);
    jsonSize = getJsonSize(parameter, a, jsonStartPos);
    jsonNameBase = getJsonName(parameter, a, jsonStartPos);

    for(uint8_t n=0; n<jsonSize; n++)
    {
      tmpStr = "";
      jsonName = getParmId(jsonNameBase,u8_mSettingNr,u8_mAktOptionGroupNr,n);

      if (type == HTML_OPTIONGROUP)
      {
        optionGroupSize = getJsonGroupsize(parameter,a,jsonStartPos);
        for(g=0; g<optionGroupSize; g++)
        {
          u8_mAktOptionGroupNr=g;
          String retStr;
          uint32_t jsonArrayGroupStart = 0;
          bool ret = json.getValue(parameter, a, "group", jsonStartPos, retStr, jsonArrayGroupStart);
          readWebValues(server, parameter, jsonArrayGroupStart);
        }
        u8_mAktOptionGroupNr = 0;
        break;
      }
      else if (type == HTML_INPUTCHECKBOX)
      {
        if (server->hasArg(String(jsonName))){setString(jsonName,"1");}
        else{setString(jsonName,"0");}
      }
      else
      {
        tmpStr = server->arg(String(jsonName));
        if (server->hasArg(String(jsonName))) setString(jsonName,tmpStr);
      }
    }
  }
}
#endif

void WebSettings::buildSendHtml(WebServer * server, const char *parameter, uint32_t jsonStartPos)
{
  uint8_t g, optionsCnt, optionGroupSize, jsonSize, u8_dataType, u8_jsonLabelOffset;
  uint16_t u32_jsonName, jsonNameBase;
  uint64_t u64_jsonName;
  String jsonLabel, st_jsonLabelEntry, strlHelp;
  boolean bo_lIsGroup, bo_loadFromFlash;

  bo_lIsGroup=false;
  bo_loadFromFlash=false;
  u8_dataType=0;

  for (uint8_t a=0; a<json.getArraySize(parameter, jsonStartPos); a++)
  {
    u64_jsonName = 0;
    jsonLabel = getJsonLabel(parameter, a, jsonStartPos);
    jsonNameBase = getJsonName(parameter, a, jsonStartPos);

    bo_loadFromFlash=false;
    u8_dataType=0;
    u32_jsonName = getParmId(jsonNameBase,u8_mAktOptionGroupNr);
    u64_jsonName = u32_jsonName;

    //Load from RAM or Flash
    u8_dataType = (uint8_t)getJson_Key(parameter, "dt", a, jsonStartPos, "").toInt();
    u64_jsonName = u64_jsonName | ((uint64_t)u8_dataType<<32); // | (1ULL<<40);
    if(getJson_Key(parameter, "flash", a, jsonStartPos, "").equals("1"))
    {
      u64_jsonName |= (1ULL<<40);
      bo_loadFromFlash=true;
    }

    uint8_t u8_lJsonType = getJsonType(parameter, a, jsonStartPos);
    uint16_t u16_lDepId;
    uint8_t u8_lDepVal, u8_lDepDt;
    switch(u8_lJsonType)
    {
      case HTML_OPTIONGROUP:
      case HTML_OPTIONGROUP_COLLAPSIBLE:
      {
        //bo_lIsGroup=true;
        st_jsonLabelEntry = getJsonLabelEntry(parameter, a, jsonStartPos);
        u8_jsonLabelOffset = getJson_Key(parameter, "label_offset", a, jsonStartPos, "0").toInt();
        optionGroupSize = getJsonGroupsize(parameter, a, jsonStartPos);
        u16_lDepId = (uint16_t)getJson_Key(parameter, "depId", a, jsonStartPos, "0").toInt(); //depence
        u8_lDepDt = (uint8_t)getJson_Key(parameter, "depDt", a, jsonStartPos, "1").toInt();
        uint8_t nrOfDepts = getJsonArrayCnt(parameter, CONST_depVal, a, jsonStartPos);
        std::vector<String> deptsValues = getJsonArrayValues(parameter, CONST_depVal, a, jsonStartPos);

        if(optionGroupSize>1 && !jsonLabel.equals(""))
        {
          if(u8_lJsonType==HTML_OPTIONGROUP_COLLAPSIBLE) sprintf(_buf,HTML_GROUP_START_DETAILS,jsonLabel.c_str());
          else sprintf(_buf,HTML_GROUP_START,jsonLabel.c_str());
          sendContentHtml(server,_buf,false);
        }
        for(g=0; g<optionGroupSize; g++)
        {
          u8_mAktOptionGroupNr=g;
          if(nrOfDepts>0)
          {
            bool depOk = false;
            for(uint8_t nd = 0 ; nd<nrOfDepts; nd++)  //depence
            {
              uint8_t depVal = deptsValues.at(nd).toInt();
              if(getInt(u16_lDepId,g,u8_lDepDt)==depVal) depOk=true;
            }
            if(depOk==false) continue;
          }

          sprintf(_buf,"<tr><td colspan='3'><b>%s %i</b></td></tr>",st_jsonLabelEntry.c_str(), g+u8_jsonLabelOffset);
          sendContentHtml(server,_buf,false);

          String retStr;
          uint32_t jsonArrayGroupStart = 0;
          bool ret = json.getValue(parameter, a, "group", jsonStartPos, retStr, jsonArrayGroupStart);
          buildSendHtml(server, parameter, jsonArrayGroupStart);

          sprintf(_buf,"<tr><td colspan='3'><hr style='border:none; border-top:1px dashed black; height:1px; color:#000000; background:transparent'></td></tr>");
          sendContentHtml(server,_buf,false);
        }
        if(u8_lJsonType==HTML_OPTIONGROUP_COLLAPSIBLE && optionGroupSize>1 && !jsonLabel.equals(""))
        {
          sprintf(_buf,HTML_GROUP_END_DETAILS);
          sendContentHtml(server,_buf,false);
        }
        u8_mAktOptionGroupNr = 0;
        break;
      }
      case HTML_INPUTTEXT:
        createHtmlTextfield(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,"text",getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTTEXTAREA:
        createHtmlTextarea(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
          break;
      case HTML_INPUTPASSWORD:
        createHtmlTextfield(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,"password",getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTDATE:
        createHtmlTextfield(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,"date",getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTTIME:
        createHtmlTextfield(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,"time",getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTCOLOR:
        createHtmlTextfield(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,"color",getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTFLOAT:
        createHtmlFloat(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTFLOAT_1:
        if(bo_loadFromFlash) createHtmlFloatX(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,(int32_t)getIntFlash(u64_jsonName, u8_dataType),1);
        else createHtmlFloatX(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getInt(u64_jsonName, u8_dataType),1);
        break;
      case HTML_INPUTFLOAT_2:
        if(bo_loadFromFlash) createHtmlFloatX(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,(int32_t)getIntFlash(u64_jsonName, u8_dataType),2);
        else createHtmlFloatX(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getInt(u64_jsonName, u8_dataType),2);
        break;
      case HTML_INPUTFLOAT_3:
        if(bo_loadFromFlash) createHtmlFloatX(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,(int32_t)getIntFlash(u64_jsonName, u8_dataType),3);
        else createHtmlFloatX(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getInt(u64_jsonName, u8_dataType),3);
        break;
      case HTML_INPUTNUMBER:
        createHtmlNumber(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTRANGE:
        createHtmlRange(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTCHECKBOX:
        createHtmlCheckbox(_buf,&u32_jsonName,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
        break;
      case HTML_INPUTSELECT:
        createHtmlStartSelect(_buf,&u64_jsonName,&jsonLabel,parameter,a,jsonStartPos);
        optionsCnt = getJsonArrayCnt(parameter, CONST_options, a, jsonStartPos);
        mOptions = getJsonArrayValues(parameter, CONST_options, a, jsonStartPos);
        mOptionLabels = getJsonArrayLabels(parameter, CONST_options, a, jsonStartPos);
        for (uint8_t j = 0 ; j<optionsCnt; j++)
        {
          sendContentHtml(server,_buf,false);
          String str_lNewLabel = mOptionLabels[j];
          String str_lDes = getStringFlash(getJsonArrValue(parameter, CONST_options, "d", j, a, jsonStartPos));
          if(str_lDes.length()>0) str_lNewLabel += " ("+str_lDes+")";
          createHtmlAddSelectOption(_buf,mOptions.at(j),str_lNewLabel,getString(u32_jsonName,bo_loadFromFlash,u8_dataType));
        }
        mOptions.clear();
        mOptionLabels.clear();
        sendContentHtml(server,_buf,false);
        sprintf(_buf,HTML_ENTRY_SELECT_END,String(u32_jsonName));
        break;
      case HTML_INPUTMULTICHECK:
      case HTML_INPUTMULTICHECK_COLLAPSIBLE:
        createHtmlStartMulti(_buf,&jsonLabel,parameter,a,jsonStartPos, u8_lJsonType);
        optionsCnt = getJsonArrayCnt(parameter, CONST_options, a, jsonStartPos);
        mOptionLabels = getJsonArrayLabels(parameter, CONST_options, a, jsonStartPos);
        for (uint8_t j = 0 ; j<optionsCnt; j++)
        {
          sendContentHtml(server,_buf,false);
          String str_lNewLabel = mOptionLabels[j];
          String str_lDes = getStringFlash(getJsonArrValue(parameter, CONST_options, "d", j, a, jsonStartPos));
          if(str_lDes.length()>0) str_lNewLabel += " ("+str_lDes+")";
          createHtmlAddMultiOption(_buf,&u32_jsonName,&u64_jsonName,parameter,a,jsonStartPos,j,str_lNewLabel,(uint32_t)getInt(u32_jsonName,u8_dataType),u8_dataType);
        }
        sendContentHtml(server,_buf,false);
        if(u8_lJsonType==HTML_INPUTMULTICHECK_COLLAPSIBLE) strcpy_P(_buf,HTML_ENTRY_MULTI_COLLAPSIBLE_END);
        else strcpy_P(_buf,HTML_ENTRY_MULTI_END);;
        break;
      case HTML_SEPARATION:
        sprintf(_buf,HTML_ENTRY_SEPARATION,jsonLabel.c_str());
        break;
      default:
        _buf[0] = 0;
        break;
    }

    uint8_t u8_lType=getJsonType(parameter, a, jsonStartPos);
    if(u8_lType!=HTML_OPTIONGROUP && u8_lType!=HTML_OPTIONGROUP_COLLAPSIBLE)
    {
      sendContentHtml(server,_buf,false);
    }

    //Help einfügen
    if(!bo_lIsGroup)
    {
      strlHelp = getJsonHelp(parameter, a, jsonStartPos);
      if(!strlHelp.equals(""))
      {
        strlHelp.replace("\n","<br>");
        sprintf(_buf,"<tr><td colspan='3' class='td0'><div class='help'>%s</div></td></tr>",strlHelp.c_str());
        sendContentHtml(server,_buf,false);
      }
    }
  }
}

void WebSettings::getDefaultValuesFromNewKeys(const char *parameter, uint32_t jsonStartPos)
{
  uint8_t  g, optionGroupSize, u8_dataType;
  uint32_t u32_tmp;
  String   retStr_default = "";

  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG,"getDefaultValuesFromNewKeys: str_mConfName=%s, arraySize=%i",str_mConfName.c_str(), json.getArraySize(parameter, jsonStartPos));
  #endif
  for (uint8_t a=0; a<json.getArraySize(parameter, jsonStartPos); a++)
  {
    uint8_t type, optionCnt; //, jsonSize;
    uint16_t jsonName, jsonNameBase;

    type = getJsonType(parameter, a, jsonStartPos);
    optionCnt = getJsonArrayCnt(parameter, CONST_options, a, jsonStartPos);
    jsonNameBase = getJsonName(parameter, a, jsonStartPos);
    jsonName = getParmId(jsonNameBase,u8_mAktOptionGroupNr);

    retStr_default="";
    json.getValue(parameter, a, "dt", jsonStartPos, retStr_default, u32_tmp);
    u8_dataType = retStr_default.toInt();

    if (type==HTML_OPTIONGROUP || type==HTML_OPTIONGROUP_COLLAPSIBLE)
    {
      optionGroupSize = getJsonGroupsize(parameter,a,jsonStartPos);
      for(g=0; g<optionGroupSize; g++)
      {
        u8_mAktOptionGroupNr=g;
        String retStr;
        uint32_t jsonArrayGroupStart = 0;
        json.getValue(parameter, a, "group", jsonStartPos, retStr, jsonArrayGroupStart);
        getDefaultValuesFromNewKeys(parameter, jsonArrayGroupStart);
      }
      u8_mAktOptionGroupNr = 0;
      //break;
    }
    else
    {
      //Store in RAM
      if(getJson_Key(parameter, "flash", a, jsonStartPos, "").equals("1")==false)
      {
        retStr_default="";
        json.getValue(parameter, a, "default", jsonStartPos, retStr_default, u32_tmp);

        if((jsonNameBase!= 0) && (isKeyExist(jsonName, u8_dataType)==false))
        {
          bo_hasNewKeys=true;
          uint16_t id=0;
          uint8_t group=0;
          getIdFromParamId(jsonName,id,group);
          //BSC_LOGI(TAG,"newDefKeyInRam: key=%i, val=%s, dt=%i, id=%i, group=%i",jsonName,retStr_default.c_str(),u8_dataType,id,group);
          setString(jsonName, retStr_default, u8_dataType);
        }
      }
      else
      {
        if((jsonNameBase != 0) && (prefs.isKey(String(jsonName).c_str())==false))
        {
          bo_hasNewKeys=true;
          retStr_default="";
          json.getValue(parameter, a, "default", jsonStartPos, retStr_default, u32_tmp);
          uint16_t id=0;
          uint8_t group=0;
          getIdFromParamId(jsonName,id,group);
          //BSC_LOGI(TAG,"newDefKeyInFlash: key=%i, val=%s, dt=%i, id=%i, group=%i",jsonName,retStr_default.c_str(),u8_dataType,id,group);
          switch(u8_dataType)
          {
            case PARAM_DT_U8:
              prefs.putChar(String(jsonName).c_str(),(uint8_t)retStr_default.toInt());
              break;
            case PARAM_DT_I8:
              prefs.putChar(String(jsonName).c_str(),(int8_t)retStr_default.toInt());
              break;
            case PARAM_DT_U16:
              prefs.putInt(String(jsonName).c_str(),(uint16_t)retStr_default.toInt());
              break;
            case PARAM_DT_I16:
              prefs.putInt(String(jsonName).c_str(),(int16_t)retStr_default.toInt());
              break;
            case PARAM_DT_U32:
              prefs.putLong(String(jsonName).c_str(),retStr_default.toInt());
              break;
            case PARAM_DT_I32:
              prefs.putLong(String(jsonName).c_str(),retStr_default.toInt());
              break;
            case PARAM_DT_FL:
              prefs.putFloat(String(jsonName).c_str(),retStr_default.toFloat());
              break;
            case PARAM_DT_ST:
              prefs.putString(String(jsonName).c_str(),retStr_default);
              break;
            case PARAM_DT_BO:
              prefs.putBool(String(jsonName).c_str(),retStr_default.toInt());
              break;
          }
        }
      }
    }
  }
  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG,"getDefaultValuesFromNewKeys: finish");
  #endif
}


void WebSettings::createHtmlTextfield(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, const char * type, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_TEXTFIELD,label->c_str(),type,value.c_str(),String(*nameExt).c_str(),getJson_Key(parameter, "unit", idx, startPos, "").c_str(),String(*name));
}

void WebSettings::createHtmlTextarea(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_AREA,label->c_str(),getJsonOptionsMax(parameter, idx, startPos),getJsonOptionsMin(parameter, idx, startPos),String(*nameExt).c_str(), value.c_str(), String(*name));
}

void WebSettings::createHtmlNumber(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_NUMBER,label->c_str(),getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMax(parameter, idx, startPos), value.c_str(),String(*nameExt).c_str(),
    getJson_Key(parameter, "unit", idx, startPos, "").c_str(),String(*name));
}

void WebSettings::createHtmlFloat(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_FLOAT,label->c_str(),
    getJson_Key(parameter, "step", idx, startPos, "0.01").c_str(),
    getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMax(parameter, idx, startPos), value.c_str(),String(*nameExt).c_str(),
    getJson_Key(parameter, "unit", idx, startPos, "").c_str(),String(*name));
}

//HTML_INPUTFLOAT_1, HTML_INPUTFLOAT_2, HTML_INPUTFLOAT_3
void WebSettings::createHtmlFloatX(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, int32_t value, uint8_t precision)
{
  //if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  BSC_LOGI(TAG,"createHtmlFloat_int: precision=%i", precision);
  String str_className;
  String str_precision;
  float fl_lVal=0;

  if(precision==1)
  {
    str_className="fl1";
    str_precision="0.1";
    fl_lVal=(float)value/10;
  }
  else if(precision==2)
  {
    str_className="fl2";
    str_precision="0.01";
    fl_lVal=(float)value/100;
  }
  else if(precision==3)
  {
    str_className="fl3";
    str_precision="0.001";
    fl_lVal=(float)value/1000;
  }
  String valueStr = String(fl_lVal);
  BSC_LOGI(TAG,"createHtmlFloat_int: valStr=%s, val=%f", valueStr.c_str(), fl_lVal);
  sprintf(buf,HTML_ENTRY_FLOAT_X,label->c_str(),
    str_precision.c_str(),
    getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMax(parameter, idx, startPos), valueStr.c_str(),String(*nameExt).c_str(),str_className.c_str(),
    getJson_Key(parameter, "unit", idx, startPos, "").c_str(),String(*name));
}

void WebSettings::createHtmlRange(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_RANGE, label->c_str(), getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMin(parameter, idx, startPos), getJsonOptionsMax(parameter, idx, startPos),
    value.c_str(), nameExt,  getJsonOptionsMax(parameter, idx, startPos),String(*name));
}

void WebSettings::createHtmlCheckbox(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  String sValue = String(*name);
  if (!value.equals("0")) {
    sprintf(buf,HTML_ENTRY_CHECKBOX,label->c_str(),"checked",String(*nameExt).c_str(),String(*name));
  } else {
    sprintf(buf,HTML_ENTRY_CHECKBOX,label->c_str(),"",String(*nameExt).c_str(),sValue.c_str());
  }
}

void WebSettings::createHtmlStartSelect(char * buf, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos)
{
  sprintf(buf,HTML_ENTRY_SELECT_START,label->c_str(),String(*nameExt).c_str());
}

void WebSettings::createHtmlAddSelectOption(char * buf, String option, String label, String value)
{
  if (option.equals(value))
  {
    sprintf(buf,HTML_ENTRY_SELECT_OPTION,option.c_str(),"selected",label.c_str());
  } else {
    sprintf(buf,HTML_ENTRY_SELECT_OPTION,option.c_str(),"",label.c_str());
  }
}

void WebSettings::createHtmlStartMulti(char * buf, String *label, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t u8_jsonType)
{
  if(u8_jsonType==HTML_INPUTMULTICHECK_COLLAPSIBLE)
  {
    uint32_t u32_lMillis = millis();
    sprintf(buf,HTML_ENTRY_MULTI_COLLAPSIBLE_START,label->c_str(),u32_lMillis,u32_lMillis,label->c_str());
  }
  else
  {
    sprintf(buf,HTML_ENTRY_MULTI_START,label->c_str());
  }
}

void WebSettings::createHtmlAddMultiOption(char * buf, uint16_t *name, uint64_t *nameExt, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t option, String label, uint32_t value, uint8_t u8_dataType)
{
  #ifdef WEBSET_DEBUG
  BSC_LOGD(TAG,"createHtmlAddMultiOption: option=%i, value=%i",option,value);
  #endif

  if(!isKeyExist(*name,u8_dataType)) value = getJsonDefault(parameter, idx, startPos).toInt();

  if((value&(1<<option))==(1<<option))
  {
    sprintf(buf,HTML_ENTRY_MULTI_OPTION,String(*nameExt).c_str(),option,"checked",label.c_str());
  } else {
    sprintf(buf,HTML_ENTRY_MULTI_OPTION,String(*nameExt).c_str(),option,"",label.c_str());
  }
}


uint8_t WebSettings::getJsonSize(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  bool ret = json.getValue(parameter, idx, "size", startPos, retStr, retArrayStart);

  if(ret==true)
  {
    if (isNumber(retStr))
    {
      return atoi(retStr.c_str());
    }
    else
    {
      //Error; Sollte nicht vorkommen
      return 1;
    }
  }
  else
  {
    return 1;
  }
}

uint8_t WebSettings::getJsonGroupsize(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;

  bool ret = json.getValue(parameter, idx, "groupsize", startPos, retStr, retArrayStart);

  if (ret==true)
  {
    if (isNumber(retStr))
    {
      return atoi(retStr.c_str());
    }
    else
    { //Error; Sollte nicht vorkommen
      return 0;
    }
  }
  else
  {
    return 0;
  }
}

uint32_t WebSettings::getJsonName(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  uint32_t id = 0;
  bool ret = json.getValue(parameter, idx, "name", startPos, retStr, retArrayStart);
  if(ret==true){return retStr.toInt();}
  return 0;
}

String WebSettings::getJsonLabel(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  bool ret = json.getValue(parameter, idx, "label", startPos, retStr, retArrayStart);
  if(ret==true){return retStr;}
  return "";
}

String WebSettings::getJsonLabelEntry(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  bool ret = json.getValue(parameter, idx, "label_entry", startPos, retStr, retArrayStart);
  if(ret==true){return retStr;}
  return "";
}

String WebSettings::getJsonHelp(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  bool ret = json.getValue(parameter, idx, "help", startPos, retStr, retArrayStart);
  if(ret==true){return retStr;}
  return "";
}

uint8_t WebSettings::getJsonType(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;

  bool ret = json.getValue(parameter, idx, "type", startPos, retStr, retArrayStart);
  if (ret==true)
  {
    if (isNumber(retStr))
    {
      return atoi(retStr.c_str());
    }
    else
    { //Error; Sollte nicht vorkommen
      return HTML_INPUTTEXT;
    }
  }
  else
  {
    return HTML_INPUTTEXT;
  }
}

String WebSettings::getJsonDefault(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  bool ret = json.getValue(parameter, idx, "default", startPos, retStr, retArrayStart);
  if(ret==true){return retStr;}
  return "0";
}

std::vector<String> WebSettings::getJsonArrayValues(const char *parameter, String key, uint8_t idx, uint32_t startPos)
{
  std::vector<String> options;
  String retStr = "";
  uint32_t retArrayStart = 0;
  uint32_t retArrayStart2 = 0;
  bool ret = json.getValue(parameter, idx, key, startPos, retStr, retArrayStart);

  if(ret==true)
  {
    for (uint8_t i=0; i< json.getArraySize(parameter, retArrayStart); i++)
    {
      retStr = "";
      retArrayStart2 = 0;
      ret = json.getValue(parameter, i, "v", retArrayStart, retStr, retArrayStart2);
      if(ret==true){options.push_back(retStr);}
    }
  }
  return options;
}

std::vector<String> WebSettings::getJsonArrayLabels(const char *parameter, String key, uint8_t idx, uint32_t startPos)
{
  std::vector<String> labels;
  String retStr = "";
  uint32_t retArrayStart = 0;
  uint32_t retArrayStart2 = 0;
  bool ret = json.getValue(parameter, idx, key , startPos, retStr, retArrayStart);

  if(ret==true)
  {
    for (uint8_t i=0; i< json.getArraySize(parameter, retArrayStart); i++)
    {
      retStr = "";
      retArrayStart2 = 0;
      ret = json.getValue(parameter, i, "l", retArrayStart, retStr, retArrayStart2);
      if(ret==true){labels.push_back(retStr);}
    }
  }
  return labels;
}

String WebSettings::getJsonArrValue(const char *parameter, String str_key1, String str_key2, uint8_t u8_eCnt, uint8_t idx, uint32_t startPos)
{
  std::vector<String> options;
  String retStr = "";
  uint32_t retArrayStart = 0;
  uint32_t retArrayStart2 = 0;
  bool ret = json.getValue(parameter, idx, str_key1, startPos, retStr, retArrayStart);

  if(ret==true)
  {
    for (uint8_t i=0; i< json.getArraySize(parameter, retArrayStart); i++)
    {
      retStr = "";
      retArrayStart2 = 0;
      ret = json.getValue(parameter, i, str_key2, retArrayStart, retStr, retArrayStart2);
      if(i==u8_eCnt)return retStr;
    }
  }
  return retStr;
}

uint8_t WebSettings::getJsonArrayCnt(const char *parameter, String key, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  bool ret = json.getValue(parameter, idx, key, startPos, retStr, retArrayStart);
  if(ret==true){return json.getArraySize(parameter, retArrayStart);}
  return 0;
}

uint32_t WebSettings::getJsonOptionsMin(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;

  bool ret = json.getValue(parameter, idx, "min", startPos, retStr, retArrayStart);

  if (ret==true) {
    if (isNumber(retStr))
    {
      return atoi(retStr.c_str());
    }
    else
    { //Error; Sollte nicht vorkommen
      return 0;
    }
  }
  else
  {
    return 0;
  }
}

uint32_t WebSettings::getJsonOptionsMax(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0;

  bool ret = json.getValue(parameter, idx, "max", startPos, retStr, retArrayStart);
  if (ret==true)
  {
    if (isNumber(retStr))
    {
      return atoi(retStr.c_str());
    }
    else //Error; Sollte nicht vorkommen
    {
      return 100;
    }
  }
  else
  {
    return 100;
  }
}

//Universal
String WebSettings::getJson_Key(const char *parameter, String key, uint8_t idx, uint32_t startPos, String defaultValue)
{
  String retStr = "";
  uint32_t retArrayStart = 0;
  bool ret = json.getValue(parameter, idx, key, startPos, retStr, retArrayStart);
  if(ret==true){return retStr;}
  return defaultValue;
}



bool WebSettings::isKeyExist(uint16_t key, uint8_t u8_dataType)
{
  bool ret = false;
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  switch(u8_dataType)
  {
    case PARAM_DT_U8:
      if(settingValues_i8.count(key)) ret=true;
      break;
    case PARAM_DT_I8:
      if(settingValues_i8.count(key)) ret=true;
      break;
    case PARAM_DT_U16:
      if(settingValues_i16.count(key)) ret=true;
      break;
    case PARAM_DT_I16:
      if(settingValues_i16.count(key)) ret=true;
      break;
    case PARAM_DT_U32:
      if(settingValues_i32.count(key)) ret=true;
      break;
    case PARAM_DT_I32:
      if(settingValues_i32.count(key)) ret=true;
      break;
    case PARAM_DT_FL:
      if(settingValues_fl.count(key)) ret=true;
      break;
    case PARAM_DT_ST:
      if(settingValues_str.count(key)) ret=true;
      break;
    case PARAM_DT_BO:
      if(settingValues_bo.count(key)) ret=true;
      break;
  }
  xSemaphoreGive(mParamMutex);
  return ret;
}


void WebSettings::setParameter(uint16_t name, uint8_t group, String value, uint8_t u8_dataType)
{
  setString(getParmId(name, group), value, u8_dataType);
}


void WebSettings::setString(uint16_t name, String value, uint8_t u8_dataType)
{
  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG,"setString(): name=%i, value=%s, dataType=%i",name,value.c_str(),u8_dataType);
  #endif
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  switch(u8_dataType)
  {
    case PARAM_DT_U8:
      settingValues_i8[name] = (uint8_t)value.toInt();
      break;
    case PARAM_DT_I8:
      settingValues_i8[name] = (int8_t)value.toInt();
      break;
    case PARAM_DT_U16:
      settingValues_i16[name] = (uint16_t)value.toInt();
      break;
    case PARAM_DT_I16:
      settingValues_i16[name] = (int16_t)value.toInt();
      break;
    case PARAM_DT_U32:
      settingValues_i32[name] = (uint32_t)value.toInt();
      break;
    case PARAM_DT_I32:
      settingValues_i32[name] = (int32_t)value.toInt();
      break;
    case PARAM_DT_FL:
      settingValues_fl[name] = value.toFloat();
      break;
    case PARAM_DT_ST:
      settingValues_str[name] = value.c_str();
      break;
    case PARAM_DT_BO:
      settingValues_bo[name] = (bool)value.toInt();
      break;
  }
  xSemaphoreGive(mParamMutex);
}

String WebSettings::getString(uint16_t name, boolean fromFlash, uint8_t u8_dataType)
{
  std::string ret = "";
  String str_ret = "";

  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG,"getString(): name=%i, fromFlash=%d, dataType=%i",name,fromFlash,u8_dataType);
  #endif

  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  switch(u8_dataType)
  {
    case PARAM_DT_U8:
      if(fromFlash) ret = String(prefs.getChar(String(name).c_str())).c_str();
      else str_ret = String((uint8_t)settingValues_i8[name]);
      break;
    case PARAM_DT_I8:
      if(fromFlash) ret = String(prefs.getChar(String(name).c_str())).c_str();
      else str_ret = String(settingValues_i8[name]);
      break;
    case PARAM_DT_U16:
      if(fromFlash) ret = String(prefs.getInt(String(name).c_str())).c_str();
      else str_ret = String((uint16_t)settingValues_i16[name]);
      break;
    case PARAM_DT_I16:
      if(fromFlash) ret = String(prefs.getInt(String(name).c_str())).c_str();
      else str_ret = String(settingValues_i16[name]);
      break;
    case PARAM_DT_U32:
      if(fromFlash) ret = String(prefs.getLong(String(name).c_str())).c_str();
      else str_ret = String((uint32_t)settingValues_i32[name]);
      break;
    case PARAM_DT_I32:
      if(fromFlash) ret = String(prefs.getLong(String(name).c_str())).c_str();
      else str_ret = String(settingValues_i32[name]);
      break;
    case PARAM_DT_FL:
      if(fromFlash) ret = String((prefs.getFloat(String(name).c_str())),4).c_str();
      else str_ret = String(settingValues_fl[name]);
      break;
    case PARAM_DT_ST:
      if(fromFlash) ret = String(prefs.getString(String(name).c_str())).c_str();
      else str_ret = String(settingValues_str[name].c_str());
      break;
    case PARAM_DT_BO:
      if(fromFlash) ret = String(prefs.getBool(String(name).c_str())).c_str();
      else str_ret = String(settingValues_bo[name]);
      break;
  }
  xSemaphoreGive(mParamMutex);

  if(fromFlash) return String(ret.c_str());
  else return str_ret;
}

String WebSettings::getString(uint16_t name, uint8_t groupNr)
{
  std::string ret = "";
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  ret = settingValues_str[getParmId(name,groupNr)];
  xSemaphoreGive(mParamMutex);
  return String(ret.c_str());
}

int32_t WebSettings::getInt(uint16_t name, uint8_t u8_dataType)
{
  /*#ifdef WEBSET_DEBUG
  BSC_LOGI(TAG,"getInt(); name=%i",name);
  #endif*/

  uint32_t ret = 0;
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  switch(u8_dataType)
  {
    case PARAM_DT_U8:
      ret=(uint8_t)settingValues_i8[name];
      break;
    case PARAM_DT_I8:
      ret=settingValues_i8[name];
      break;
    case PARAM_DT_U16:
      ret=(uint16_t)settingValues_i16[name];
      break;
    case PARAM_DT_I16:
      ret=settingValues_i16[name];
      break;
    case PARAM_DT_U32:
      ret=(uint32_t)settingValues_i32[name];
      break;
    case PARAM_DT_I32:
      ret=settingValues_i32[name];
      break;
  }
  xSemaphoreGive(mParamMutex);
  return ret;
}

int32_t WebSettings::getInt(uint16_t name, uint8_t groupNr, uint8_t u8_dataType)
{
  return getInt(getParmId(name,groupNr),u8_dataType);
}

float WebSettings::getFloat(uint16_t name)
{
  return settingValues_fl[name];
}
float WebSettings::getFloat(uint16_t name, uint8_t groupNr)
{
  return getFloat(getParmId(name, groupNr));
}

bool WebSettings::getBool(uint16_t name)
{
  return settingValues_bo[name];
}
bool WebSettings::getBool(uint16_t name, uint8_t groupNr)
{
  return getBool(getParmId(name, groupNr));
}


//Load Data from Flash
uint32_t WebSettings::getIntFlash(uint16_t name, uint8_t groupNr, uint8_t u8_dataType)
{
  int ret=0;
  uint16_t u32_name = getParmId(name, groupNr);
  switch(u8_dataType)
  {
    case PARAM_DT_U8:
      ret = prefs.getChar(String(u32_name).c_str());
      break;
    case PARAM_DT_I8:
      ret = prefs.getChar(String(u32_name).c_str());
      break;
    case PARAM_DT_U16:
      ret = prefs.getInt(String(u32_name).c_str());
      break;
    case PARAM_DT_I16:
      ret = prefs.getInt(String(u32_name).c_str());
      break;
    case PARAM_DT_U32:
      ret = prefs.getLong(String(u32_name).c_str());
      break;
    case PARAM_DT_I32:
      ret = prefs.getLong(String(u32_name).c_str());
      break;
  }
  return ret;
}
uint32_t WebSettings::getIntFlash(uint16_t name, uint8_t u8_dataType)
{
  int ret=0;
  switch(u8_dataType)
  {
    case PARAM_DT_U8:
      ret = prefs.getChar(String(name).c_str());
      break;
    case PARAM_DT_I8:
      ret = prefs.getChar(String(name).c_str());
      break;
    case PARAM_DT_U16:
      ret = prefs.getInt(String(name).c_str());
      break;
    case PARAM_DT_I16:
      ret = prefs.getInt(String(name).c_str());
      break;
    case PARAM_DT_U32:
      ret = prefs.getLong(String(name).c_str());
      break;
    case PARAM_DT_I32:
      ret = prefs.getLong(String(name).c_str());
      break;
  }
  return ret;
}
float WebSettings::getFloatFlash(uint16_t name, uint8_t groupNr)
{
  uint16_t u32_name = getParmId(name, groupNr);
  return prefs.getFloat(String(u32_name).c_str());
}
float WebSettings::getFloatFlash(uint16_t name)
{
  return prefs.getFloat(String(name).c_str());
}
boolean WebSettings::getBoolFlash(uint16_t name, uint8_t groupNr)
{
  uint16_t u32_name = getParmId(name, groupNr);
  return prefs.getBool(String(u32_name).c_str());
}
boolean WebSettings::getBoolFlash(uint16_t name)
{
  return prefs.getBool(String(name).c_str());
}
String WebSettings::getStringFlash(uint16_t name, uint8_t groupNr)
{
  uint16_t u32_name = getParmId(name, groupNr);
  return prefs.getString(String(u32_name).c_str());
}
String WebSettings::getStringFlash(String name)
{
  if(name.equals(""))return "";
  return prefs.getString(name.c_str());
}
String WebSettings::getStringFlash(uint16_t name)
{
  return prefs.getString(String(name).c_str());
}



uint16_t WebSettings::getParmId(uint16_t id, uint8_t groupIdx)
{
  return (id<<6) | (groupIdx&0x3F);
}


void WebSettings::getIdFromParamId(uint16_t paramId, uint16_t &id, uint8_t &groupIdx)
{
  id = ((paramId>>6)&0x3FF);
  groupIdx = (paramId&0x3F);
}

//Lese Parameter aus Datei
boolean WebSettings::readConfig()
{
  String  str_line, str_value, str_dataType, confFile;
  uint32_t str_name;
  uint8_t ui8_pos;

  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG,"readConfig()");
  #endif

  confFile=str_mConfigfile;

  if (SPIFFS.exists(confFile.c_str()))
  {
    uint32_t crc = calcCrc(confFile.c_str());
    if(prefs.isKey("confCrc") && prefs.getULong("confCrc")!=crc) // Wrong CRC
    {
      if(SPIFFS.exists("/WebSettings.sich"))
      {
        BSC_LOGI(TAG,"Fehler beim lesen der Parameter (falsche CRC)! Lade Backup.");
        confFile="/WebSettings.sich";
      }
      else
      {
        BSC_LOGI(TAG,"Fehler beim lesen der Parameter (falsche CRC)!");
        writeConfig();
      }
    }
    else
    {
      if(!prefs.isKey("confCrc")) BSC_LOGI(TAG,"Noch keine CRC der Settings vorhanden");
      else BSC_LOGI(TAG,"Settings ok");

      //Wenn die CRC der Config ok ist, aber noch kein Backup-File exisitiert
      uint32_t crcBackup = copyFile(str_mConfigfile.c_str(), "/WebSettings.sich");
      if(crc!=crcBackup) //CRC des Backups falsch -> Bakup wieder löschen
      {
        BSC_LOGI(TAG,"Fehler beim erstellen des Backups (falsche CRC)");
        SPIFFS.remove("/WebSettings.sich");
      }
    }
  }
  else
  {
    //wenn settingfile nicht vorhanden, dann schreibe default Werte
    if(SPIFFS.exists("/WebSettings.sich"))
    {
      BSC_LOGI(TAG,"Kein Parameterfile vorhanden. Lade Backup.");
      confFile="/WebSettings.sich";
    }
    else
    {
      BSC_LOGI(TAG,"Kein Parameterfile vorhanden");
      writeConfig();
    }
  }

  File f = SPIFFS.open(confFile.c_str(),"r");
  if (f)
  {
    #ifdef WEBSET_DEBUG
    BSC_LOGI(TAG,"file open");
    #endif

    uint32_t size = f.size();
    uint32_t fPosOld=0xFFFFFFFF;
    while (f.position() < size)
    {
      #ifdef WEBSET_DEBUG
      BSC_LOGI(TAG,"readConfig size=%i, pos=%i", size, (uint32_t)f.position());
      #endif

      str_line = f.readStringUntil(10);
      ui8_pos = str_line.indexOf('=');
      str_name = str_line.substring(0,ui8_pos).toInt();
      str_dataType = str_line.substring(ui8_pos+1,ui8_pos+1+3).c_str();
      str_value = str_line.substring(ui8_pos+1+3).c_str();
      str_value.replace("~","\n");
      if(str_dataType.equals("U8_")) setString(str_name, str_value, PARAM_DT_U8);
      else if(str_dataType.equals("I8_")) setString(str_name, str_value, PARAM_DT_I8);
      else if(str_dataType.equals("U16")) setString(str_name, str_value, PARAM_DT_U16);
      else if(str_dataType.equals("I16")) setString(str_name, str_value, PARAM_DT_I16);
      else if(str_dataType.equals("U32")) setString(str_name, str_value, PARAM_DT_U32);
      else if(str_dataType.equals("I32")) setString(str_name, str_value, PARAM_DT_I32);
      else if(str_dataType.equals("FL_")) setString(str_name, str_value, PARAM_DT_FL);
      else if(str_dataType.equals("BO_")) setString(str_name, str_value, PARAM_DT_BO);
      else if(str_dataType.equals("STR")) setString(str_name, str_value, PARAM_DT_ST);

      #ifdef WEBSET_DEBUG
      BSC_LOGI(TAG,"readConfig key:%lu, val:%s",str_name, str_value.c_str());
      #endif
      if(fPosOld==f.position())
      {
        BSC_LOGE(TAG,"Read config break: pos=%i",(uint32_t)f.position());
        break;
      }
      fPosOld=f.position();
    }
    f.close();
    #ifdef WEBSET_DEBUG
    BSC_LOGI(TAG,"readConfig() end");
    #endif
    return true;
  }
  else
  {
    BSC_LOGI(TAG,"Cannot read configuration");
    return false;
  }
}

//Schreiben der Parameter in Datei
boolean WebSettings::writeConfig()
{
  //std::string val;
  String val2;
  uint32_t name;

  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG, "writeConfig()");
  #endif

  if (!SPIFFS.exists(str_mConfigfile.c_str()))
  {
    BSC_LOGI(TAG,"writeConfig(): file not exist");
  }

  File f = SPIFFS.open(str_mConfigfile.c_str(),"w");
  if (f)
  {
    xSemaphoreTake(mParamMutex, portMAX_DELAY);
    //String
    auto iter = settingValues_str.begin();
    for (const auto& n : settingValues_str)
    {
      name = n.first;
      std::string val = n.second;
      val2 = String(val.c_str());
      val2.replace("\n","~");
      f.printf("%lu=STR%s\n",name,val2.c_str());
    }

    //u8
    //I8
    iter = settingValues_i8.begin();
    for (const auto& n : settingValues_i8)
    {
      name = n.first;
      int8_t val = n.second;
      f.printf("%lu=I8_%i\n",name,val);
    }

    //u16
    //i16
    iter = settingValues_i16.begin();
    for (const auto& n : settingValues_i16)
    {
      name = n.first;
      int16_t val = n.second;
      f.printf("%lu=I16%i\n",name,val);
    }

    //u32
    //i32
    iter = settingValues_i32.begin();
    for (const auto& n : settingValues_i32)
    {
      name = n.first;
      int32_t val = n.second;
      f.printf("%lu=I32%i\n",name,val);
    }

    //bool
    iter = settingValues_bo.begin();
    for (const auto& n : settingValues_bo)
    {
      name = n.first;
      bool val = n.second;
      f.printf("%lu=BO_%d\n",name,val);
    }

    //float
    iter = settingValues_fl.begin();
    for (const auto& n : settingValues_fl)
    {
      name = n.first;
      float val = n.second;
      f.printf("%lu=FL_%f\n",name,val);
    }

    xSemaphoreGive(mParamMutex);
    f.flush();
    f.close();

    //CRC speichern und Backup der Config erstellen
    uint32_t crc = calcCrc(str_mConfigfile.c_str());
    prefs.putULong("confCrc",crc);
    uint32_t crcBackup = copyFile(str_mConfigfile.c_str(), "/WebSettings.sich");
    if(crc!=crcBackup)
    {
      BSC_LOGI(TAG,"Fehler beim erstellen des Backup (falsche CRC)");
      SPIFFS.remove("/WebSettings.sich");
    }

    return true;
  }
  else
  {
    BSC_LOGI(TAG,"Cannot write configuration");
    return false;
  }
  #ifdef WEBSET_DEBUG
  BSC_LOGI(TAG, "writeConfig() finish");
  #endif
}

//Löschen des Parameterfiles
boolean WebSettings::deleteConfig()
{
  //return LittleFS.remove(str_mConfigfile.c_str());
  return SPIFFS.remove(str_mConfigfile.c_str());
}

uint32_t WebSettings::copyFile(String fileSrc, String fileDst)
{
  char buf[64];
  uint32_t crc=0;

  if (SPIFFS.exists(fileDst) == true) SPIFFS.remove(fileDst);

  File fSrc = SPIFFS.open(fileSrc, "r");
  File fDst = SPIFFS.open(fileDst, "w");

  while (fSrc.available() > 0)
  {
    uint8_t readBytes = fSrc.readBytes(buf, 64);
    crc=calcCrc32(crc, (uint8_t*)buf, readBytes);
    fDst.write((uint8_t*)buf, readBytes);
  }

  fDst.close();
  fSrc.flush();
  fSrc.close();

  return crc;
}

uint32_t WebSettings::calcCrc(String fileSrc)
{
  char buf[64];
  uint32_t crc=0;
  File fSrc = SPIFFS.open(fileSrc, "r");
  while (fSrc.available() > 0)
  {
    uint8_t readBytes = fSrc.readBytes(buf, 64);
    crc=calcCrc32(crc, (uint8_t*)buf, readBytes);
  }
  return crc;
}

void WebSettings::setButtons(uint8_t buttons, String btnLabel)
{
  if(buttons==BUTTON_1)
  {
    str_mButton1Text = btnLabel;
    u8_mButtons |= (1<<BUTTON_1);
  }
  else if(buttons==BUTTON_2)
  {
    str_mButton2Text = btnLabel;
    u8_mButtons |= (1<<BUTTON_2);
  }
  else if(buttons==BUTTON_3)
  {
    str_mButton3Text = btnLabel;
    u8_mButtons |= (1<<BUTTON_3);
  }
}

//register callback for Buttons
void WebSettings::registerOnSave(void (*callback)())
{
  fn_mOnButtonSave = callback;
}
void WebSettings::registerOnButton1(void (*callback)())
{
  fn_mOnButton1 = callback;
}
void WebSettings::registerOnButton2(void (*callback)())
{
  fn_mOnButton2 = callback;
}
void WebSettings::registerOnButton3(void (*callback)())
{
  fn_mOnButton3 = callback;
}







#ifdef INSIDER_V1
void addJsonElem(char *buf, int id, bool val)
{
  uint16_t bufLen = strlen(buf);
  if (bufLen == 0) sprintf(&buf[bufLen], "[");
  else sprintf(&buf[bufLen], ",");
  bufLen++;
  sprintf(&buf[bufLen], "{\"id\":%i, \"val\":%u}",id,val);
}
void addJsonElem(char *buf, int id, int32_t val)
{
  uint16_t bufLen = strlen(buf);
  if (bufLen == 0) sprintf(&buf[bufLen], "[");
  else sprintf(&buf[bufLen], ",");
  bufLen++;
  sprintf(&buf[bufLen], "{\"id\":%i, \"val\":%i}",id,val);
}
void addJsonElem(char *buf, int id, float val)
{
  uint16_t bufLen = strlen(buf);
  if (bufLen == 0) sprintf(&buf[bufLen], "[");
  else sprintf(&buf[bufLen], ",");
  bufLen++;
  sprintf(&buf[bufLen], "{\"id\":%i, \"val\":%.3f}",id,val);
}
void addJsonElem(char *buf, int id, String val)
{
  uint16_t bufLen = strlen(buf);
  if (bufLen == 0) sprintf(&buf[bufLen], "[");
  else sprintf(&buf[bufLen], ",");
  bufLen++;
  sprintf(&buf[bufLen], "{\"id\":%i, \"val\":\"%s\"}",id,val.c_str());
}

void WebSettings::handleGetValues(WebServer *server)
{
  uint32_t argName;
  char data[4096] = { 0 };
  //BSC_LOGI(TAG,"handleGetValues: args=%i",server->args());
  for(uint8_t i=0; i<server->args(); i++)
  {
    //BSC_LOGI(TAG,"handleGetValues: argNr=%i",i);
    //BSC_LOGI(TAG,"handleGetValues: name=%s, arg=%s",server->argName(i).c_str(), server->arg(i).c_str());
    if(isNumber(server->argName(i))) //Wenn keine Zahl, dann Fehler
    {
      argName = server->argName(i).toInt();

      uint8_t u8_storeInFlash = ((argName>>16)&0xff);
      uint8_t u8_dataType = ((argName>>24)&0xff);

      if(u8_storeInFlash==0)
      {
        if(u8_dataType==PARAM_DT_BO) addJsonElem(data, argName, getBool((uint16_t)(argName&0xFFFF)));
        else if(u8_dataType==PARAM_DT_FL) addJsonElem(data, argName, getFloat((uint16_t)(argName&0xFFFF)));
        else if(u8_dataType==PARAM_DT_ST) addJsonElem(data, argName, getString((uint16_t)(argName&0xFFFF),false,PARAM_DT_ST));
        else addJsonElem(data, argName, getInt((uint16_t)(argName&0xFFFF), u8_dataType));
      }
      else if(u8_storeInFlash==1)
      {
        if(u8_dataType==PARAM_DT_BO) addJsonElem(data, argName, getBoolFlash((uint16_t)(argName&0xFFFF)));
        else if(u8_dataType==PARAM_DT_FL) addJsonElem(data, argName, getFloatFlash((uint16_t)(argName&0xFFFF)));
        else if(u8_dataType==PARAM_DT_ST) addJsonElem(data, argName, getStringFlash((uint16_t)(argName&0xFFFF)));
        else addJsonElem(data, argName, (int)getIntFlash((uint16_t)(argName&0xFFFF), u8_dataType));
      }
    }
    else
    {
      BSC_LOGE(TAG,"handleGetValues: not number");
    }
  }
  sprintf(&data[strlen(data)], "]");

  server->send(200, "application/json", data);
}

void WebSettings::handleSetValues(WebServer *server)
{
  //BSC_LOGI(TAG,"handleSetValues args=%i",server->args());

  uint8_t u8_storeInFlash,u8_datatype,ret,u8_lParamGroup;
  uint16_t u16_argName,u16_lParamId;
  uint32_t u32_argName;
  String argValue;

  //BSC_LOGI(TAG,"handleSetValues: args=%i",server->args());
  for(uint8_t i=0; i<server->args(); i++)
  {
    //BSC_LOGI(TAG,"handleSetValues: argNr=%i",i);
    //BSC_LOGI(TAG,"handleSetValues: name=%s, arg=%s",server->argName(i).c_str(), server->arg(i).c_str());
    if(isNumber(server->argName(i))) //Wenn keine Zahl, dann Fehler
    {
      //BSC_LOGI(TAG,"handleSetValues argNr=%i, argName=%s, val=%s", i, server->argName(i).c_str(), server->arg(i).c_str());

      u32_argName = server->argName(i).toInt();
      argValue = server->arg(i);

      u8_storeInFlash = ((u32_argName>>16)&0xff);
      u8_datatype = ((u32_argName>>24)&0xff);
      u16_argName = (u32_argName&0xFFFF);

      if(u8_storeInFlash==1) //Store  in Flash
      {
        #ifdef WEBSET_DEBUG
        BSC_LOGD(TAG,"Store in Flash; u16_argName=%i, dt=%i",u16_argName,u8_datatype);
        #endif

        //Spezielle Parameter vor dem Speichern ändern
        u16_lParamId=0;
        u8_lParamGroup=0;
        getIdFromParamId(u16_argName,u16_lParamId,u8_lParamGroup);
        if(u16_lParamId==ID_PARAM_SS_BTDEVMAC) argValue.toLowerCase();

        ret=0xFF;
        switch(u8_datatype)
        {
          case PARAM_DT_U8:
            ret=prefs.putChar(String(u16_argName).c_str(),(uint8_t)argValue.toInt());
            break;
          case PARAM_DT_I8:
            ret=prefs.putChar(String(u16_argName).c_str(),(int8_t)argValue.toInt());
            break;
          case PARAM_DT_U16:
            ret=prefs.putInt(String(u16_argName).c_str(),(uint16_t)argValue.toInt());
            break;
          case PARAM_DT_I16:
            ret=prefs.putInt(String(u16_argName).c_str(),(int16_t)argValue.toInt());
            break;
          case PARAM_DT_U32:
            ret=prefs.putLong(String(u16_argName).c_str(),argValue.toInt());
            break;
          case PARAM_DT_I32:
            ret=prefs.putLong(String(u16_argName).c_str(),argValue.toInt());
            break;
          case PARAM_DT_FL:
            ret=prefs.putFloat(String(u16_argName).c_str(),argValue.toFloat());
            break;
          case PARAM_DT_ST:
            ret=prefs.putString(String(u16_argName).c_str(),argValue);
            break;
          case PARAM_DT_BO:
            ret=prefs.putBool(String(u16_argName).c_str(),argValue.toInt());
            break;
        }
      }
      else //Store in RAM
      {
        #ifdef WEBSET_DEBUG
        BSC_LOGD(TAG,"Store in RAM; u16_argName=%i, dt=%i",u16_argName,u8_datatype);
        #endif

        setString(u16_argName, argValue, u8_datatype);
      }
    }
    /*else
    {
      BSC_LOGE(TAG,"handleSetValues: not number");
    }*/
  }
  server->send(200, "application/json", "{\"state\":1}");
}
#endif










