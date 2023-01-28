// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "WebSettings.h"
#include "defines.h"
#include <Arduino.h>
#include "SPIFFS.h"
#include <WebServer.h>
#include <FS.h>
#include "log.h"

static const char * TAG = "WEB_SETTINGS";

static SemaphoreHandle_t mParamMutex = NULL;

static std::map<uint32_t, String> settingValues;
static char _buf[2000];
static String st_mSendBuf = "";
static String str_lTmpGetString;
std::vector<String> mOptions;
std::vector<String> mOptionLabels;

uint8_t u8_mAktOptionGroupNr;
static Json json;

//HTML templates
const char HTML_START[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang='de'>
<head>
<meta http-equiv='Content-Type' content='text/html' charset='utf-8'/>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BSC</title>
<style>
html {font-family:Helvetica; display:inline-block; margin:0px auto; text-align:left;}
body {margin: 0; background-color: %s;}
.content {padding:20px; max-width:600px;}
.topnav {overflow: hidden;position:sticky;top:0;background-color:#6E6E6E;color:white;padding:5px;}
.topnav span {float: left; padding: 14px 16px; text-decoration: none; font-size:1.7rem;}
.btnBack:hover {background-color:#555555;color:white;}
.hl {flex:1;font-size:2rem;}
.titel {font-weight:bold; text-align:left; padding:5px;}
.Ctr {width:100%%; padding:10px 5px 0 5px; text-align: left;}
.Ctd {width:150; padding:10px 5px 0 5px; text-align: left; vertical-align: top;}
.tr0 {padding: 0 5px 0 5px;}
.help {border-left-width:2px; border-left-style:solid; padding:0 0 0 2px; margin:0 0 0 5px; text-align:left; vertical-align:top;}
button {font-size:100%%; width:150px; padding:0px; margin:10px 0 0 0;}
.sb {font-size:100%%; width:25px; height:25px; padding:0px; margin:5px 0 0 0;}
input:invalid {border:1px solid red;}
input:valid {border:1px solid green;}

</style>
</head>
<body>
<div class="topnav">
  <span class='btnBack' onclick=location.href='../'>&#10094;</span>
  <span class='hl'>%s</span>
</div>
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
<script>
function btnClick(btn) {
  var xhttp = new XMLHttpRequest();
  xhttp.open('POST','?'+btn+'=',true);
  xhttp.timeout=1000;
  xhttp.send();  
}

function urlencode(str) {
  str = (str + '').toString();
  return encodeURIComponent(str)
    .replace('!', '%21')
    .replace('\'', '%27')
    .replace('(', '%28')
    .replace(')', '%29')
    .replace('*', '%2A')
    .replace('%20', '+');
}

const collection = document.getElementsByClassName("t1");
for (let i = 0; i < collection.length; i++) {
  var button = document.createElement('button');
  button.type = 'button';
  button.innerHTML = 'S';
  button.className = 'sb';
  button.addEventListener("click", function(event){
    let t1=this.parentNode.previousSibling.firstChild;

    if (t1.checkValidity() === false) {
      alert('Fehler');
      return;
    }

    var name=t1.getAttribute("name");
    var val;
    if(t1.nodeName=='SELECT'){
      val=t1.options[t1.selectedIndex].value; 
    }else if(t1.nodeName=='INPUT'){
      if(t1.type=='checkbox'){
        if(t1.checked){val=1;}
        else{val=0;}
      }else{
        val=t1.value;
      }
    }else{
      val=t1.value;
    }

    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        alert(this.responseText);
      }
    };
    xhttp.open('GET','?SAVE=&'+name+'='+urlencode(val),true);
    xhttp.send();
  });
  collection[i].appendChild(button);
}
</script>
)rawliteral";

const char HTML_END_3[] PROGMEM = R"rawliteral(
</body>
</html>
)rawliteral";

const char HTML_SCRIPT[] PROGMEM = R"rawliteral(
<script>
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById('data_div').innerHTML = this.responseText;
    }
  };
  xhttp.open('GET', '%s', true);
  xhttp.timeout=1000;
  xhttp.send();
  var timer = window.setTimeout('getData()', %i);
}
getData();
</script>
)rawliteral";

const char HTML_ENTRY_TEXTFIELD[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><input type='%s' value='%s' name='%s'>&nbsp;%s</td><td class='t1'></td></tr>\n";
const char HTML_ENTRY_AREA[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><textarea rows='%i' cols='%i' name='%s'>%s</textarea></td><td class='t1'></td></tr>\n"; //
const char HTML_ENTRY_FLOAT[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><input type='number' step='0.01' min='%i' max='%i' value='%s' name='%s'>&nbsp;%s</td><td class='t1'></td></tr>\n";
const char HTML_ENTRY_NUMBER[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><input type='number' min='%i' max='%i' value='%s' name='%s'>&nbsp;%s</td><td class='t1'></td></tr>\n";
const char HTML_ENTRY_RANGE[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'>%i&nbsp;<input type='range' min='%i' max='%i' value='%s' name='%s'>&nbsp;%i</td><td class='t1'></td></tr>\n";
const char HTML_ENTRY_CHECKBOX[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td><td class='Ctd'><input type='checkbox' %s name='%s'></td><td class='t1'></td></tr>\n"; 
const char HTML_ENTRY_SELECT_START[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><select name='%s'>\n";
const char HTML_ENTRY_SELECT_OPTION[] PROGMEM =
"<option value='%s' %s>%s</option>\n"; //
const char HTML_ENTRY_SELECT_END[] PROGMEM =
"</select></td><td class='t1'></td></tr>\n";
const char HTML_ENTRY_MULTI_START[] PROGMEM =
"<tr class='Ctr'><td class='Ctd'><b>%s</b></td>\n"
"<td class='Ctd'><fieldset style='text-align:left;'>\n";
const char HTML_ENTRY_MULTI_OPTION[] PROGMEM =
"<input type='checkbox' name='%s', value='%i' %s>%s<br>\n";
const char HTML_ENTRY_MULTI_END[] PROGMEM =
"</fieldset></td></tr>\n";

const char HTML_GROUP_START[] PROGMEM = "</table><details open><summary><b>%s</b></summary><table>\n";
const char HTML_GROUP_END[]   PROGMEM = "</table></details><table>\n";

const char HTML_ENTRY_SEPARATION[] PROGMEM = "<tr class='Ctr'><td class='Ctd' colspan='3'><b><u>%s</u></b></td></tr>\n";

const char HTML_BUTTON[] PROGMEM = "<div class='Ctd'><button onclick='btnClick(\"%s\")'>%s</button></div>\n";


bool isNumber(const String& str)
{
  for (char const &c : str) {
    if(std::isdigit(c)==0 && c!='-') return false;
  }
  return true;
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
    server->sendContent(st_mSendBuf);
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

  if(json.getArraySize(parameterFile,0)==0)
  {
    ESP_LOGI(TAG,"Error read json");
  }
  else
  {
    boolean exit = false;

    if (server->hasArg(F("SAVE")))
    {
      //ESP_LOGD(TAG,"Arg: SAVE");
      for(uint8_t i=0; i<server->args(); i++)
      {
        String argName = server->argName(i);
        if(argName!="SAVE"){
          String argValue = server->arg(i);
          //ESP_LOGD(TAG, "SAVE: name:%s, val:%s", argName, argValue);

          if(isNumber(argName)) //Wenn keine Zahl, dann Fehler
          {
            setString((uint32_t)argName.toInt(), argValue); 
            writeConfig(); //Schreiben der Einstellungen in das Config file
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
    if (!exit){  
      server->setContentLength(CONTENT_LENGTH_UNKNOWN);
      sprintf(_buf,HTML_START,BACKGROUND_COLOR,str_mConfName.c_str()); //Titel der Seite
      server->send(200, "text/html", _buf);

      buildSendHtml(server, parameterFile, 0);
      
      sendContentHtml(server,HTML_END_1,false); //HTML_END_1

      //Buttons einfügen
      if ((u8_mButtons & 0x02) == 0x02) //Button 1
      {
        sprintf(_buf,HTML_BUTTON,"BTN1",str_mButton1Text.c_str());
        sendContentHtml(server,_buf,false);
      }
      if ((u8_mButtons & 0x04) == 0x04) //Button 2
      {
        sprintf(_buf,HTML_BUTTON,"BTN2",str_mButton2Text.c_str());
        sendContentHtml(server,_buf,false);
      }
      if ((u8_mButtons & 0x08) == 0x08) //Button 3
      {
        sprintf(_buf,HTML_BUTTON,"BTN3",str_mButton3Text.c_str());
        sendContentHtml(server,_buf,false);
      }

      sendContentHtml(server,HTML_END_2,false); //HTML_END_2

      //Timer bei bedarf einfügen
      if(!str_mAjaxGetDataTimerHandlerName.equals(""))
      {
        sprintf(_buf,HTML_SCRIPT,str_mAjaxGetDataTimerHandlerName.c_str(),u16_mAjaxGetDataTimerSec);
        sendContentHtml(server,_buf,false);
      }

      sendContentHtml(server,HTML_END_3,true); //HTML_END_3
    }
  }
}

/*not use*/
/*void WebSettings::readWebValues(WebServer * server, const String *parameter, uint32_t jsonStartPos)
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
}*/


void WebSettings::buildSendHtml(WebServer * server, const char *parameter, uint32_t jsonStartPos)
{
  uint8_t g;
  uint8_t optionsCnt;
  uint8_t optionGroupSize;
  boolean bo_lIsGroup=false;
  
  uint32_t jsonName;
  String jsonLabel;
  String jsonLabelBase;
  String st_jsonLabelEntry;
  uint32_t jsonNameBase;
  uint8_t jsonSize;
  String strlHelp;

  for (uint8_t a=0; a<json.getArraySize(parameter, jsonStartPos); a++)
  {
    jsonName = 0;
    jsonLabel = "";
    jsonLabelBase = getJsonLabel(parameter, a, jsonStartPos);
    jsonNameBase = getJsonName(parameter, a, jsonStartPos);
    jsonSize = getJsonSize(parameter, a, jsonStartPos);  


    for(uint8_t n=0; n<jsonSize; n++)
    {  
      jsonName = getParmId(jsonNameBase,u8_mSettingNr,u8_mAktOptionGroupNr,n);
      jsonLabel = jsonLabelBase;
      if(jsonSize>1){jsonLabel += " (";}
      if(jsonSize>1){jsonLabel += n;}
      if(jsonSize>1){jsonLabel += ")";}

      switch (getJsonType(parameter, a, jsonStartPos))
      {
        case HTML_OPTIONGROUP:
          bo_lIsGroup=true;
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

            sprintf(_buf,"<tr><td colspan='3'><hr></td></tr>");
            sendContentHtml(server,_buf,false);
          }
          if(optionGroupSize>1 && !jsonLabel.equals(""))
          {
            sprintf(_buf,HTML_GROUP_END);
            sendContentHtml(server,_buf,false);
          }
          u8_mAktOptionGroupNr = 0;
          break;
        case HTML_INPUTTEXT: 
          createHtmlTextfield(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,"text",getString(jsonName));
          break;
        case HTML_INPUTTEXTAREA: 
          createHtmlTextarea(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(jsonName));
            break;
        case HTML_INPUTPASSWORD: 
          createHtmlTextfield(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,"password",getString(jsonName));
          break;
        case HTML_INPUTDATE: 
          createHtmlTextfield(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,"date",getString(jsonName));
          break;
        case HTML_INPUTTIME: 
          createHtmlTextfield(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,"time",getString(jsonName));
          break;
        case HTML_INPUTCOLOR: 
          createHtmlTextfield(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,"color",getString(jsonName));
          break;      
        case HTML_INPUTFLOAT: 
          createHtmlFloat(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(jsonName));
          break;
        case HTML_INPUTNUMBER: 
          createHtmlNumber(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(jsonName));
          break;
        case HTML_INPUTRANGE: 
          createHtmlRange(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(jsonName));
          break;
        case HTML_INPUTCHECKBOX: 
          createHtmlCheckbox(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos,getString(jsonName));
          break;
        case HTML_INPUTSELECT: 
          createHtmlStartSelect(_buf,&jsonName,&jsonLabel,parameter,a,jsonStartPos);
          optionsCnt = getJsonOptionsCnt(parameter,a,jsonStartPos);
          mOptions = getJsonOptionValues(parameter,a,jsonStartPos);
          mOptionLabels = getJsonOptionLabels(parameter,a,jsonStartPos);
          for (uint8_t j = 0 ; j<optionsCnt; j++)
          {
            sendContentHtml(server,_buf,false);
            createHtmlAddSelectOption(_buf,mOptions.at(j),mOptionLabels[j],getString(jsonName));
          }
          sendContentHtml(server,_buf,false);
          strcpy_P(_buf,HTML_ENTRY_SELECT_END);
          break;
        case HTML_INPUTMULTICHECK: 
          createHtmlStartMulti(_buf,&jsonLabel,parameter,a,jsonStartPos);
          optionsCnt = getJsonOptionsCnt(parameter,a,jsonStartPos);
          mOptionLabels = getJsonOptionLabels(parameter,a,jsonStartPos);
          for (uint8_t j = 0 ; j<optionsCnt; j++)
          {
            sendContentHtml(server,_buf,false);
            createHtmlAddMultiOption(_buf,&jsonName,parameter,a,jsonStartPos,j,mOptionLabels[j],getString(jsonName));
          }
          server->sendContent(_buf);
          strcpy_P(_buf,HTML_ENTRY_MULTI_END);
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
        sprintf(_buf,"<tr class='tr0'><td colspan='3' class='tr0'><div class='help'>%s</div></td></tr>",strlHelp.c_str());
        sendContentHtml(server,_buf,false);
      }
      }
    }
  }
}

void WebSettings::getDefaultValuesFromNewKeys(const char *parameter, uint32_t jsonStartPos)
{
  uint8_t g, optionGroupSize;
  uint32_t u32_tmp;
  String retStr_label = "";
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
        retStr_default="";
        json.getValue(parameter, a, "default", jsonStartPos, retStr_default, u32_tmp);

        if(isKeyExist(jsonName)==false)
        {
          retStr_label="";
          json.getValue(parameter, a, "label", jsonStartPos, retStr_label, u32_tmp);
          //debugPrintf("New Parameter found: key=%i, default=%s, label=%s",jsonName,retStr_default.c_str(),retStr_label.c_str());
          setString(jsonName, retStr_default);
        }
      }
    }
  }
}


void WebSettings::createHtmlTextfield(char * buf, uint32_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos, const char * type, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_TEXTFIELD,label->c_str(),type,value.c_str(),String(*name).c_str(),getJson_Key(parameter, "unit", idx, startPos).c_str());
}

void WebSettings::createHtmlTextarea(char * buf, uint32_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_AREA,label->c_str(),getJsonOptionsMax(parameter, idx, startPos),getJsonOptionsMin(parameter, idx, startPos),String(*name).c_str(), value.c_str());
}

void WebSettings::createHtmlNumber(char * buf, uint32_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_NUMBER,label->c_str(),getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMax(parameter, idx, startPos), value.c_str(),String(*name).c_str(),
    getJson_Key(parameter, "unit", idx, startPos).c_str());
}

void WebSettings::createHtmlFloat(char * buf, uint32_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_FLOAT,label->c_str(),getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMax(parameter, idx, startPos), value.c_str(),String(*name).c_str(),
    getJson_Key(parameter, "unit", idx, startPos).c_str());
}

void WebSettings::createHtmlRange(char * buf, uint32_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  sprintf(buf,HTML_ENTRY_RANGE, label->c_str(), getJsonOptionsMin(parameter, idx, startPos),
    getJsonOptionsMin(parameter, idx, startPos), getJsonOptionsMax(parameter, idx, startPos),
    value.c_str(), name,  getJsonOptionsMax(parameter, idx, startPos));
}

void WebSettings::createHtmlCheckbox(char * buf, uint32_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  if (!value.equals("0")) {
    sprintf(buf,HTML_ENTRY_CHECKBOX,label->c_str(),"checked",String(*name).c_str());
  } else {
    sprintf(buf,HTML_ENTRY_CHECKBOX,label->c_str(),"",String(*name).c_str());
  }
}

void WebSettings::createHtmlStartSelect(char * buf, uint32_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos)
{
  sprintf(buf,HTML_ENTRY_SELECT_START,label->c_str(),String(*name).c_str());
}

void WebSettings::createHtmlAddSelectOption(char * buf, String option, String label, String value)
{
  if (option == value) {
    sprintf(buf,HTML_ENTRY_SELECT_OPTION,option.c_str(),"selected",label.c_str());
  } else {
    sprintf(buf,HTML_ENTRY_SELECT_OPTION,option.c_str(),"",label.c_str());
  }
}

void WebSettings::createHtmlStartMulti(char * buf, String *label, const char *parameter, uint8_t idx, uint32_t startPos)
{
  sprintf(buf,HTML_ENTRY_MULTI_START,label->c_str());
}

void WebSettings::createHtmlAddMultiOption(char * buf, uint32_t *name, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t option, String label, String value)
{
  if(value.equals("")){value = getJsonDefault(parameter, idx, startPos);}
  if ((value.length() > option) && (value[option] == '1')) {
    sprintf(buf,HTML_ENTRY_MULTI_OPTION,String(*name).c_str(),option,"checked",label.c_str());
  } else {
    sprintf(buf,HTML_ENTRY_MULTI_OPTION,String(*name).c_str(),option,"",label.c_str());
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
String WebSettings::getJson_Key(const char *parameter, String key, uint8_t idx, uint32_t startPos)
{
  String retStr = "";
  uint32_t retArrayStart = 0; 
  bool ret = json.getValue(parameter, idx, key, startPos, retStr, retArrayStart);
  if(ret==true){return retStr;}
  return "";
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

String WebSettings::getString(uint32_t name)
{
  String ret = "";
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  ret = settingValues[name];
  //debugPrintf("getString A:%lu, val:%s",name, ret);
  xSemaphoreGive(mParamMutex);
  return ret;
}
String WebSettings::getString(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  String ret = "";
  xSemaphoreTake(mParamMutex, portMAX_DELAY);
  ret = settingValues[getParmId(name,settingNr, groupNr, listNr)];
  //debugPrintf("getString B:%lu, val:%s",name, ret);
  xSemaphoreGive(mParamMutex);
  return ret;
}

int WebSettings::getInt(uint32_t name)
{
  return getString(name).toInt();
}
int WebSettings::getInt(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  return getString(name, settingNr, groupNr, listNr).toInt();
}

float WebSettings::getFloat(uint32_t name)
{
  return getString(name).toFloat();
}
float WebSettings::getFloat(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  return getString(name, settingNr, groupNr, listNr).toFloat();
}

boolean WebSettings::getBool(uint32_t name)
{
  return (getString(name) != "0");
}
boolean WebSettings::getBool(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr)
{
  return (getString(name, settingNr, groupNr, listNr) != "0");
}


uint32_t WebSettings::getParmId(uint16_t id, uint8_t settingNr, uint8_t groupIdx, uint8_t listIdx)
{
  return (id<<20) + (settingNr<<14) + (groupIdx<<7) + (listIdx & 0x7F);
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
      //debugPrintf("readConfig key:%lu, val:%s\n",str_name, settingValues[str_name].c_str());
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
  String val;
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
    while (iter != settingValues.end())
    {
      name = iter->first;
      val = iter->second;
      ++iter;

      val.replace("\n","~");
      f.printf("%lu=%s\n",name,val.c_str());
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
