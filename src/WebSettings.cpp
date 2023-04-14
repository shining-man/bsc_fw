// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "WebSettings.h"
#include "web/webSettings_web.h"
#include "defines.h"
#include <Arduino.h>
#include "SPIFFS.h"
#include <WebServer.h>
#include <FS.h>
#include "log.h"
#include <sparsepp/spp.h>
#include <Preferences.h>

using spp::sparse_hash_map;

static const char * TAG = "WEB_SETTINGS";

static SemaphoreHandle_t mParamMutex = NULL;

sparse_hash_map<uint32_t, std::string> settingValues;
static char _buf[2000];
static String st_mSendBuf = "";
static String str_lTmpGetString;
std::vector<String> mOptions;
std::vector<String> mOptionLabels;

uint8_t u8_mAktOptionGroupNr;
static Json json;

static Preferences prefs;

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


void WebSettings::initWebSettings(const char *parameter, String confName, String configfile, uint8_t settingNr)
{
  u8_mJsonArraySize = 0;
  str_mConfName = confName.c_str();
  str_mConfigfile = configfile.c_str();
  u8_mSettingNr = settingNr;

  parameterFile = parameter;

  if (!SPIFFS.begin())
  {
    SPIFFS.format();
    SPIFFS.begin();
  }


  if (!prefs.begin("prefs"))
  {
    //ESP_LOGE(TAG,"Fehler beim Oeffnen des NVS-Namespace");
  }
  else
  {
    ESP_LOGI(TAG,"Anzahl freier Eintraege: %d", prefs.freeEntries());
  }
  
  readConfig();
  getDefaultValuesFromNewKeys(parameterFile, 0);
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
    //debugPrintf("sende sendBuf size=%i\n",st_mSendBuf.length());
    if(!st_mSendBuf.equals("")) server->sendContent(st_mSendBuf);
    //ESP_LOGI(TAG, "BUF_1:%s",st_mSendBuf.c_str());
    st_mSendBuf="";
  }

  if(strlen(buf)>500)
  {
    server->sendContent(buf);
    //ESP_LOGI(TAG, "BUF_2:%s",buf);
  }
}

//Function to response HTTP request from the form to save data
void WebSettings::handleHtmlFormRequest(WebServer * server)
{
  uint8_t ui8_optionsCnt;
  std::vector<String> options;
  std::vector<String> optionLabels;

  if(json.getArraySize(parameterFile,0)==0)
  {
    ESP_LOGI(TAG,"Error read json");
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
          ESP_LOGD(TAG, "SAVE: name:%s, val:%s", argName, argValue);
          #endif

          if(isNumber(argName)) //Wenn keine Zahl, dann Fehler
          {
            char* end;
            uint64_t u64_argName=strtoull(argName.c_str(),&end,10);
            uint32_t u32_argName = (uint32_t)(u64_argName&0xFFFFFFFF);

            if((u64_argName&((uint64_t)1<<40))==((uint64_t)1<<40)) //Store  in Flash
            {              
              uint8_t u8_datatype = ((u64_argName>>32)&0xff);
              #ifdef WEBSET_DEBUG
              ESP_LOGD(TAG,"Store in Flash; u32_argName=%i, dt=%i",u32_argName,u8_datatype);
              #endif

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
              //ESP_LOGD(TAG,"Store in RAM; u32_argName=%i",u32_argName);

              setString(u32_argName, argValue); 
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

      //sendContentHtml(server,HTML_STYLE_2,false);
      
      sendContentHtml(server,webSettingsStyle,false);

      sprintf(_buf,HTML_START_2,str_mConfName.c_str());
      sendContentHtml(server,_buf,false);

      buildSendHtml(server, parameterFile, 0);
      
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

  ESP_LOGD(tag,"SAVE readWebValues()");

  for (uint8_t a=0; a<json.getArraySize(parameter, jsonStartPos); a++)
  {
    uint8_t type, optionCnt, jsonSize;
    uint32_t jsonName, jsonNameBase;

    type = getJsonType(parameter, a, jsonStartPos);
    optionCnt = getJsonOptionsCnt(parameter, a, jsonStartPos);
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
  uint8_t g;
  uint8_t optionsCnt;
  uint8_t optionGroupSize;
  boolean bo_lIsGroup=false;
  
  uint32_t u32_jsonName;
  uint64_t u64_jsonName;
  String jsonLabel;
  String jsonLabelBase;
  String st_jsonLabelEntry;
  uint32_t jsonNameBase;
  uint8_t jsonSize;
  String strlHelp;

  boolean bo_loadFromFlash=false;
  uint8_t u8_dataType=0;

  for (uint8_t a=0; a<json.getArraySize(parameter, jsonStartPos); a++)
  {
    u64_jsonName = 0;
    jsonLabel = "";
    jsonLabelBase = getJsonLabel(parameter, a, jsonStartPos);
    jsonNameBase = getJsonName(parameter, a, jsonStartPos);
    jsonSize = getJsonSize(parameter, a, jsonStartPos);  


    for(uint8_t n=0; n<jsonSize; n++)
    {  
      bo_loadFromFlash=false;
      u8_dataType=0;

      u32_jsonName = getParmId(jsonNameBase,u8_mSettingNr,u8_mAktOptionGroupNr,n);
      u64_jsonName = u32_jsonName;
      jsonLabel = jsonLabelBase;
      if(jsonSize>1){jsonLabel += " (";}
      if(jsonSize>1){jsonLabel += n;}
      if(jsonSize>1){jsonLabel += ")";}


      //Load from RAM or Flash
      if(getJson_Key(parameter, "flash", a, jsonStartPos, "").equals("1"))
      {
        bo_loadFromFlash=true;
        u8_dataType = (uint8_t)getJson_Key(parameter, "dt", a, jsonStartPos, "").toInt();
        u64_jsonName = u64_jsonName | ((uint64_t)u8_dataType<<32) | (1ULL<<40);
      }

      uint8_t u8_lJsonType = getJsonType(parameter, a, jsonStartPos);
      switch(u8_lJsonType)
      {
        case HTML_OPTIONGROUP:
          //bo_lIsGroup=true;
          st_jsonLabelEntry = getJsonLabelEntry(parameter, a, jsonStartPos);
          optionGroupSize = getJsonGroupsize(parameter, a, jsonStartPos);
          if(optionGroupSize>1 && !jsonLabel.equals(""))
          {
            sprintf(_buf,HTML_GROUP_START,jsonLabel.c_str());
            sendContentHtml(server,_buf,false);
          }
          for(g=0; g<optionGroupSize; g++)
          {
            u8_mAktOptionGroupNr=g;

            sprintf(_buf,"<tr><td colspan='3'><b>%s %i</b></td></tr>",st_jsonLabelEntry.c_str(), g);
            sendContentHtml(server,_buf,false);

            String retStr;
            uint32_t jsonArrayGroupStart = 0;
            bool ret = json.getValue(parameter, a, "group", jsonStartPos, retStr, jsonArrayGroupStart);
            buildSendHtml(server, parameter, jsonArrayGroupStart);

            sprintf(_buf,"<tr><td colspan='3'><hr style='border:none; border-top:1px dashed black; height:1px; color:#000000; background:transparent'></td></tr>");
            sendContentHtml(server,_buf,false);
          }
          /*if(optionGroupSize>1 && !jsonLabel.equals(""))
          {
            sprintf(_buf,HTML_GROUP_END);
            sendContentHtml(server,_buf,false);
          }*/
          u8_mAktOptionGroupNr = 0;
          break;
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
          optionsCnt = getJsonOptionsCnt(parameter,a,jsonStartPos);
          mOptions = getJsonOptionValues(parameter,a,jsonStartPos);
          mOptionLabels = getJsonOptionLabels(parameter,a,jsonStartPos);
          for (uint8_t j = 0 ; j<optionsCnt; j++)
          {
            sendContentHtml(server,_buf,false);
            String str_lNewLabel = mOptionLabels[j];
            String str_lDes = getStringFlash(getJsonArrValue(parameter, "options", "d", j, a, jsonStartPos));
            if(str_lDes.length()>0) str_lNewLabel += " ("+str_lDes+")";
            createHtmlAddSelectOption(_buf,mOptions.at(j),str_lNewLabel,getString(u64_jsonName,bo_loadFromFlash,u8_dataType));
          }
          mOptions.clear();
          mOptionLabels.clear();
          sendContentHtml(server,_buf,false);
          sprintf(_buf,HTML_ENTRY_SELECT_END,String(u32_jsonName));
          break;
        case HTML_INPUTMULTICHECK:
        case HTML_INPUTMULTICHECK_COLLAPSIBLE:
          createHtmlStartMulti(_buf,&jsonLabel,parameter,a,jsonStartPos, u8_lJsonType);
          optionsCnt = getJsonOptionsCnt(parameter,a,jsonStartPos);
          mOptionLabels = getJsonOptionLabels(parameter,a,jsonStartPos);
          for (uint8_t j = 0 ; j<optionsCnt; j++)
          {
            sendContentHtml(server,_buf,false);
            String str_lNewLabel = mOptionLabels[j];
            String str_lDes = getStringFlash(getJsonArrValue(parameter, "options", "d", j, a, jsonStartPos));
            if(str_lDes.length()>0) str_lNewLabel += " ("+str_lDes+")";
            createHtmlAddMultiOption(_buf,&u32_jsonName,&u64_jsonName,parameter,a,jsonStartPos,j,str_lNewLabel,getInt(u32_jsonName));
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

      if(getJsonType(parameter, a, jsonStartPos) != HTML_OPTIONGROUP)
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
}

void WebSettings::getDefaultValuesFromNewKeys(const char *parameter, uint32_t jsonStartPos)
{
  uint8_t g, optionGroupSize, u8_dataType;
  uint32_t u32_tmp;
  //String retStr_label = "";
  String retStr_default = "";

  for (uint8_t a=0; a<json.getArraySize(parameter, jsonStartPos); a++)
  {
    uint8_t type, optionCnt, jsonSize;
    uint32_t jsonName, jsonNameBase;

    type = getJsonType(parameter, a, jsonStartPos);
    optionCnt = getJsonOptionsCnt(parameter, a, jsonStartPos);
    jsonSize = getJsonSize(parameter, a, jsonStartPos);
    jsonNameBase = getJsonName(parameter, a, jsonStartPos);

    for(uint8_t n=0; n<jsonSize; n++)
    {  
      jsonName = getParmId(jsonNameBase,u8_mSettingNr,u8_mAktOptionGroupNr,n);

      if (type == HTML_OPTIONGROUP)
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
        break;
      }
      else
      {
        //Store in RAM
        if(getJson_Key(parameter, "flash", a, jsonStartPos, "").equals("1")==false)
        {
          retStr_default="";
          json.getValue(parameter, a, "default", jsonStartPos, retStr_default, u32_tmp);

          if(isKeyExist(jsonName)==false)
          {
            setString(jsonName, retStr_default);
          }
        }
        else
        {
          if(prefs.isKey(String(jsonName).c_str())==false)
          {
            retStr_default="";
            json.getValue(parameter, a, "dt", jsonStartPos, retStr_default, u32_tmp);
            u8_dataType = retStr_default.toInt();

            //wirte defaultvalues
              retStr_default="";
              json.getValue(parameter, a, "default", jsonStartPos, retStr_default, u32_tmp);
              
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
  }
}


void WebSettings::createHtmlTextfield(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, const char * type, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_TEXTFIELD,label->c_str(),type,value.c_str(),String(*nameExt).c_str(),getJson_Key(parameter, "unit", idx, startPos, "").c_str(),String(*name));
}

void WebSettings::createHtmlTextarea(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_AREA,label->c_str(),getJsonOptionsMax(parameter, idx, startPos),getJsonOptionsMin(parameter, idx, startPos),String(*nameExt).c_str(), value.c_str(), String(*name));
}

void WebSettings::createHtmlNumber(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_NUMBER,label->c_str(),getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMax(parameter, idx, startPos), value.c_str(),String(*nameExt).c_str(),
    getJson_Key(parameter, "unit", idx, startPos, "").c_str(),String(*name));
}

void WebSettings::createHtmlFloat(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_FLOAT,label->c_str(),
    getJson_Key(parameter, "step", idx, startPos, "0.01").c_str(),
    getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMax(parameter, idx, startPos), value.c_str(),String(*nameExt).c_str(),
    getJson_Key(parameter, "unit", idx, startPos, "").c_str(),String(*name));
}

void WebSettings::createHtmlRange(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_RANGE, label->c_str(), getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMin(parameter, idx, startPos), getJsonOptionsMax(parameter, idx, startPos),
    value.c_str(), nameExt,  getJsonOptionsMax(parameter, idx, startPos),String(*name));
}

void WebSettings::createHtmlCheckbox(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
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
  if (option == value) {
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

void WebSettings::createHtmlAddMultiOption(char * buf, uint32_t *name, uint64_t *nameExt, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t option, String label, uint32_t value)
{
  #ifdef WEBSET_DEBUG
  ESP_LOGD(TAG,"createHtmlAddMultiOption: option=%i, value=%i",option,value);
  #endif

  if(!isKeyExist(*name)) value = getJsonDefault(parameter, idx, startPos).toInt();

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

std::vector<String> WebSettings::getJsonOptionValues(const char *parameter, uint8_t idx, uint32_t startPos)
{
  std::vector<String> options;
  String retStr = "";
  uint32_t retArrayStart = 0;
  uint32_t retArrayStart2 = 0;  
  bool ret = json.getValue(parameter, idx, "options", startPos, retStr, retArrayStart);

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

std::vector<String> WebSettings::getJsonOptionLabels(const char *parameter, uint8_t idx, uint32_t startPos)
{
  std::vector<String> labels;
  String retStr = "";
  uint32_t retArrayStart = 0; 
  uint32_t retArrayStart2 = 0;  
  bool ret = json.getValue(parameter, idx, "options", startPos, retStr, retArrayStart);

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

uint8_t WebSettings::getJsonOptionsCnt(const char *parameter, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0; 
  bool ret = json.getValue(parameter, idx, "options", startPos, retStr, retArrayStart);
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



bool WebSettings::isKeyExist(uint32_t key)
{
  bool ret = false;
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  if(settingValues.count(key)) ret=true;
  xSemaphoreGive(mParamMutex);
  return ret;
}

void WebSettings::setString(uint32_t name, String value)
{
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  settingValues[name] = value.c_str();
  xSemaphoreGive(mParamMutex);
}

String WebSettings::getString(uint32_t name, boolean fromFlash, uint8_t u8_dataType)
{
  std::string ret = "";

  if(fromFlash)
  {
    switch(u8_dataType)
    {
      case PARAM_DT_U8:
        ret = String(prefs.getChar(String(name).c_str())).c_str();
        break;
      case PARAM_DT_I8:
        ret = String(prefs.getChar(String(name).c_str())).c_str();
        break;
      case PARAM_DT_U16:
        ret = String(prefs.getInt(String(name).c_str())).c_str();
        break;
      case PARAM_DT_I16:
        ret = String(prefs.getInt(String(name).c_str())).c_str();
        break;
      case PARAM_DT_U32:
        ret = String(prefs.getLong(String(name).c_str())).c_str();
        break;
      case PARAM_DT_I32:
        ret = String(prefs.getLong(String(name).c_str())).c_str();
        break;
      case PARAM_DT_FL:
        ret = String((prefs.getFloat(String(name).c_str())),4).c_str();
        break;
      case PARAM_DT_ST:
        ret = String(prefs.getString(String(name).c_str())).c_str();
        break;
      case PARAM_DT_BO:
        ret = String(prefs.getBool(String(name).c_str())).c_str();
        break;
    }
  }
  else
  {
    xSemaphoreTake(mParamMutex, portMAX_DELAY);
    ret = settingValues[name];
    //debugPrintf("getString A:%lu, val:%s",name, ret);
    xSemaphoreGive(mParamMutex);
  }

  return String(ret.c_str());
}
String WebSettings::getString(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  std::string ret = "";
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  ret = settingValues[getParmId(name,settingNr, groupNr, listNr)];
  //debugPrintf("getString B:%lu, val:%s",name, ret);
  xSemaphoreGive(mParamMutex);
  return String(ret.c_str());
}

int WebSettings::getInt(uint32_t name)
{
  return getString(name,false,0).toInt();
}
int WebSettings::getInt(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  return getString(name, settingNr, groupNr, listNr).toInt();
}

float WebSettings::getFloat(uint32_t name)
{
  return getString(name,false,0).toFloat();
}
float WebSettings::getFloat(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  return getString(name, settingNr, groupNr, listNr).toFloat();
}

boolean WebSettings::getBool(uint32_t name)
{
  return (getString(name,false,0) != "0");
}
boolean WebSettings::getBool(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  return (getString(name, settingNr, groupNr, listNr) != "0");
}


//Load Data from Flash
int WebSettings::getIntFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr, uint8_t u8_dataType)
{
  int ret=0;
  uint32_t u32_name = getParmId(name,settingNr, groupNr, listNr);
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
float WebSettings::getFloatFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  uint32_t u32_name = getParmId(name,settingNr, groupNr, listNr);
  return prefs.getFloat(String(u32_name).c_str());
}
boolean WebSettings::getBoolFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  uint32_t u32_name = getParmId(name,settingNr, groupNr, listNr);
  return prefs.getBool(String(u32_name).c_str());
}
String WebSettings::getStringFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  uint32_t u32_name = getParmId(name,settingNr, groupNr, listNr);
  return prefs.getString(String(u32_name).c_str());
}
String WebSettings::getStringFlash(String name)
{
  if(name.equals(""))return "";
  return prefs.getString(name.c_str());
}


uint32_t WebSettings::getParmId(uint16_t id, uint8_t settingNr, uint8_t groupIdx, uint8_t listIdx)
{
  return (id<<20) | (settingNr<<14) | (groupIdx<<7) | (listIdx & 0x7F);
}


void WebSettings::getIdFromParamId(uint32_t paramId, uint16_t &id, uint8_t &settingNr, uint8_t &groupIdx, uint8_t &listIdx)
{
  id = (paramId>>20) & 0xFFF;
  settingNr = (paramId>>14) & 0x3F;
  groupIdx = (paramId>>7) & 0x7F;
  listIdx = paramId & 0x7F;
}

//Lese Parameter aus Datei
boolean WebSettings::readConfig()
{
  String  str_line, str_value;
  uint32_t str_name;
  uint8_t ui8_pos;

  if (!SPIFFS.exists(str_mConfigfile.c_str()))
  {
    //wenn settingfile nicht vorhanden, dann schreibe default Werte
    ESP_LOGI(TAG,"readConfig: file not exist");
    writeConfig();
  }

  File f = SPIFFS.open(str_mConfigfile.c_str(),"r");
  if (f)
  {
    uint32_t size = f.size();
    while (f.position() < size)
    {
      str_line = f.readStringUntil(10);
      ui8_pos = str_line.indexOf('=');
      str_name = str_line.substring(0,ui8_pos).toInt();
      str_value = str_line.substring(ui8_pos+1).c_str();
      str_value.replace("~","\n");
      setString(str_name, str_value);
      //ESP_LOGI(TAG,"readConfig key:%lu, val:%s\n",str_name, settingValues[str_name].c_str());
    }
    f.close();
    return true;
  }
  else
  {
    ESP_LOGI(TAG,"Cannot read configuration");
    return false;
  }
}

//Schreiben der Parameter in Datei
boolean WebSettings::writeConfig()
{
  std::string val;
  String val2;
  uint32_t name;

  //ESP_LOGD(TAG, "writeConfig()");

  if (!SPIFFS.exists(str_mConfigfile.c_str()))
  {
    ESP_LOGI(TAG,"writeConfig(): file not exist");
  }

  File f = SPIFFS.open(str_mConfigfile.c_str(),"w");
  if (f)
  {   
    xSemaphoreTake(mParamMutex, portMAX_DELAY);
    auto iter = settingValues.begin();
    for (const auto& n : settingValues) 
    {
      name = n.first;
      val = n.second;

      val2 = String(val.c_str());
      val2.replace("\n","~");
      f.printf("%lu=%s\n",name,val2.c_str());
      //fprintf(f,"%lu=%s\n",name,val2.c_str());
    }
    xSemaphoreGive(mParamMutex);
    f.flush();
    f.close();
    return true;
  }
  else
  {
    ESP_LOGI(TAG,"Cannot write configuration");
    return false;
  }
}

//Löschen des Parameterfiles
boolean WebSettings::deleteConfig()
{
  return SPIFFS.remove(str_mConfigfile.c_str());
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
