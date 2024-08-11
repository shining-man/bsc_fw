// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Arduino.h"
#include "defines.h"

/* ***********************************************************************************
 * Wichtiger Hinweis!
 * webpages.h nicht manuell bearbeiten! Änderungen immer in der webpages_py.h vornehmen!
 * Die webpages.h wird beim Build automatisch erstellt!
 * ***********************************************************************************/

/*//PY_VAR_ANF
PYVAR_HTML_WEBPAGES_STYLE
"<style>"
  "html {font-family:Helvetica;display:inline-block;margin:0px auto;text-align:center;}"
  "body {margin:0;}"
  ".content {padding:20px;}"
  ".topnav {overflow: hidden;position:sticky;top:0;background-color:#6E6E6E;color:white;padding:5px;cursor:default;}"
  ".topnav span {float:left; padding: 14px 16px; text-decoration:none; font-size:1.7rem;}"
  ".btnBack:hover {background-color:#555555;color:white;}"
  ".hl {flex:1;font-size:2rem;}"
  ".cards {padding:20px;max-width:700px;margin:0 auto;display:grid;grid-gap:2rem;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));}"
  ".cardBtn {background-color:white;box-shadow:2px 2px 12px 1px rgba(140,140,140,.5);text-decoration:none;font-size:26px; cursor:default;}"
  ".cardDash {background-color:#F2F2F2;box-shadow:2px 2px 12px 1px rgba(140,140,140,.5);text-decoration:none;font-size:26px; cursor:default;}"
  ".cardDashBtn {background-color:#F2F2F2;box-shadow:2px 2px 12px 1px rgba(140,140,140,.5);text-decoration:none;font-size:26px; cursor:default;}"
  ".cardBtn:hover {background-color:#555555;color:white;}"
  ".cardDashBtn:hover {background-color:#555555;color:white;}"
  ".button {background-color:#BDBDBD;border:none;border-radius:10px;color:white;padding:16px 40px;text-decoration:none;font-size:26px;margin:2px;cursor:default;}"
  ".button:hover {background-color:#555555;color:white;}"
  ".button:active {color:#BDBDBD;}"
  "meter {margin: 0 20px 0 20px;}"
"</style>";
//PY_VAR_END*/


const char htmlPageRoot[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
 // "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
//  "<link rel='stylesheet' type='text/css' href='bsc.css'>"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='hl'>Battery Safety Controller&nbsp;&nbsp;&nbsp;__BSC_SW_VERSION__</span>"
  "</div>"
  "<div style='background-color: #BFEFFF; padding: 10px; margin-top: 10px;'>Gefällt dir das Projekt? Zeige deine Wertschätzung doch einfach mit einer Spende! Weitere Infos unter "
    "<a href='./support/'>Unterst&uuml;tzung</a></div>"
  "<div class='content'>"
    "<div class='cards'>"
      "<div class='cardDashBtn' onclick=\"location.href='./settings/system/'\"><p>WiFi<br><meter id='meter_wifiRssi' min='0' max='70' value='0'></meter></p></div>" //WiFi RSSI
      "<div class='cardDashBtn' onclick=\"location.href='./settings/system/'\"><p>System<br><span id='card_system'></span></p></div>"
      "<div class='cardDashBtn' onclick=\"location.href='./settings/system/'\"><p>MQTT<br><span id='card_mqtt'></span></p></div>"
      "<div class='cardDash' id='card_freeheap'></div>"
      "<div class='cardDashBtn' id='card_boottime' onclick=\"location.href='./log'\"></div>"
      "<div class='cardDashBtn' onclick=\"location.href='./settings/schnittstellen/bt/'\"><p>BT-Devices<center><table style='margin-top: -1em'>"
        "<tr><td>1..5</td><td><span id='card_bt1'></span></td></tr>"
        "<tr><td>6..7&nbsp;&nbsp;</td><td><span id='card_bt2'></span></td></tr></table></center></p></div>"
      "<div class='cardDash'><p>Trigger<center><table style='margin-top: -1em'>"
        "<tr><td>1..5</td><td><span id='card_alarm1'></span></td></tr>"
        "<tr><td>6..10&nbsp;&nbsp;</td><td><span id='card_alarm2'></span></td></tr></table></center></p></div>"
      "<div class='cardDashBtn' onclick=\"location.href='./livedata/bscDataLive/'\"><p>Serial BMS (0-2)<center><table style='margin-top: -1em; font-size:20px;'>"
        "<tr><td>0&nbsp;</td><td><span id='cardBms0V'></span>V</td><td><span id='cardBms0A'></span>A</td><td><span id='cardBms0P'></span>%</td></tr>"
        "<tr><td>1&nbsp;</td><td><span id='cardBms1V'></span>V</td><td><span id='cardBms1A'></span>A</td><td><span id='cardBms1P'></span>%</td></tr>"
        "<tr><td>2&nbsp;</td><td><span id='cardBms2V'></span>V</td><td><span id='cardBms2A'></span>A</td><td><span id='cardBms2P'></span>%</td></tr></table></center></p></div>"
      //"<div class='cardDash'><p>BSC SN<br><span id='card_bscsn'></span></p></div>"
      "<div class='cardBtn' onclick=\"location.href='./livedata/'\"><p>Livedaten</p></div>"
      "<div class='cardBtn' onclick=\"location.href='./settings/'\"><p>Einstellungen</p></div>"
      //"<div class='cardBtn' onclick=\"location.href='./log'\"><p>Log</p></div>"
      "<div class='cardBtn' onclick=\"window.open('https://github.com/shining-man/bsc_fw','_blank');\"><p>BSC on GitHub</p></div>"
      "<div class='cardBtn' onclick=\"location.href='./support/'\"><p>Unterst&uuml;tzung</p></div>"
    "</div>"
  "</div>"

 "<div id='sponsors' padding: 10px; margin-top: 10px;'></div>"

"</body>"
"<script>"
"function getData() {"
  "var xhttp = new XMLHttpRequest();"
  "xhttp.onreadystatechange = function() {"
    "if (this.readyState == 4 && this.status == 200) {"
      "var values = this.responseText.split('|');"
      "document.getElementById('card_freeheap').innerHTML = '<p>Free Heap<br>' + values[0] + '</p>';"
      "document.getElementById('card_alarm1').innerHTML = values[1];"
      "document.getElementById('card_alarm2').innerHTML = values[2];"
      "if(values[3]=='1') {"
        "document.getElementById('card_mqtt').innerHTML = 'connected';"
        "document.getElementById('card_mqtt').style.color = '#00FF00';"
      "}else{"
        "document.getElementById('card_mqtt').innerHTML = 'disconnected';"
        "document.getElementById('card_mqtt').style.color = '#FF0000';"
      "}"
      "if(values[4]=='0') {"
        "document.getElementById('card_system').innerHTML = 'running';"
        "document.getElementById('card_system').style.color = '#00FF00';"
      "}else{"
        "document.getElementById('card_system').innerHTML = 'failure (' + values[4] + ')';"
        "document.getElementById('card_system').style.color = '#FF0000';"
      "}"
      "document.getElementById('card_bt1').innerHTML = values[5];"
      "document.getElementById('card_bt2').innerHTML = values[6];"
      "document.getElementById('meter_wifiRssi').value = values[7];" // WiFi RSSI
      "document.getElementById('card_boottime').innerHTML = '<p>Boot Time<br>' + values[8] + '</p>';"
      //"var bscsn = document.getElementById('card_bscsn').innerHTML;"
      //"if(bscsn != values[13]){"
      //"document.getElementById('card_bscsn').innerHTML = values[13];" // BCS serialnumber
      //"}"

      "document.getElementById('cardBms0V').innerHTML = values[9].split(';')[0];"
      "document.getElementById('cardBms0A').innerHTML = values[9].split(';')[1];"
      "document.getElementById('cardBms0P').innerHTML = values[9].split(';')[2];"
      "document.getElementById('cardBms1V').innerHTML = values[10].split(';')[0];"
      "document.getElementById('cardBms1A').innerHTML = values[10].split(';')[1];"
      "document.getElementById('cardBms1P').innerHTML = values[10].split(';')[2];"
      "document.getElementById('cardBms2V').innerHTML = values[11].split(';')[0];"
      "document.getElementById('cardBms2A').innerHTML = values[11].split(';')[1];"
      "document.getElementById('cardBms2P').innerHTML = values[11].split(';')[2];"
    "}"
  "};"
  "xhttp.open('GET', 'getDashboardData', true);"
  "xhttp.timeout=1000;"
  "xhttp.send();"
  "var timer = window.setTimeout('getData()', 2000);"
"}"
"function getSponsors() {"
  "fetch('https://raw.githubusercontent.com/shining-man/bsc_fw/main/spons')"
    ".then(response => response.text())"
    ".then((data) => {"
	    "document.getElementById('sponsors').innerHTML = data;"
    "});"
"}"
"getData();"
"getSponsors();"
"</script>"
"</html>";


const char htmlPageSettings[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
  "<script>"
    "function reboot() {"
      "var check = confirm('Reboot?');"
      "if (check == true){location.href='../restart/';}"
    "}"
  "</script>"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Einstellungen</span>"
  "</div>"
  "<div class='content'>"
    "<div class='cards'>"
    "<div class='cardBtn' onclick=\"location.href='./system/'\"><p>System</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./schnittstellen/'\"><p>Schnittstellen</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./devices/'\"><p>Devices</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./alarm/'\"><p>Alarmregeln</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./inverter/'\"><p>Wechselrichter&nbsp;&<br>Laderegelung</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./webota/'\"><p>Update</p></div>"
    "<div class='cardBtn' onclick=\"location.href='/log'\"><p>Log</p></div>"
    "<div class='cardBtn' onclick='reboot()'><p>Reboot</p></div>"
    "</div>"
  "</div>"
"</body>"
"</html>";


const char htmlPageInverter[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Einstellungen - Inverter</span>"
  "</div>"
  "<div class='content'>"
    "<div class='cards'>"
    "<div class='cardBtn' onclick=\"location.href='./bms_can/'\"><p>Allgemein</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./can_charge/'\"><p>Laden</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./can_discharge/'\"><p>Entladen</p></div>"
    "</div>"
  "</div>"
"</body>"
"</html>";

const char htmlPageSchnittstellen[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Einstellungen - Schnittstellen</span>"
  "</div>"
  "<div class='content'>"
    "<div class='cards'>"
    "<div class='cardBtn' onclick=\"location.href='./dout/'\"><p>Relaisausg&auml;nge</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./din/'\"><p>Digitaleing&auml;nge</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./serial/'\"><p>Serial</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./ow/'\"><p>Onewire</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./ow2/'\"><p>Onewire II</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./bt/'\"><p>Bluetooth</p></div>"
    "</div>"
  "</div>"
"</body>"
"</html>";


const char htmlPageDevices[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Einstellungen - Devices</span>"
  "</div>"
  "<div class='content'>"
    "<div class='cards'>"
    "<div class='cardBtn' onclick=\"location.href='./neeyBalancer/'\"><p>NEEY Balancer</p></div>"
    //"<div class='cardBtn' onclick=\"location.href='./jbdBms/'\"><p>JBD BMS</p></div>"
    "</div>"
  "</div>"
"</body>"
"</html>";


const char htmlPageAlarm[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Einstellungen - Alarmregeln</span>"
  "</div>"
  "<div class='content'>"
    "<div class='cards'>"
    "<div class='cardBtn' onclick=\"location.href='./alarmBt/'\"><p>BMS</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./alarmTemp/'\"><p>Temperatur</p></div>"
    "</div>"
  "</div>"
"</body>"
"</html>";


const char htmlPageMenuLivedata[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Livedaten</span>"
  "</div>"
  "<div class='content'>"
    "<div class='cards'>"
    "<div class='cardBtn' onclick=\"location.href='./owTempLive/'\"><p>OW Temperaturen</p></div>"
    "<div class='cardBtn' onclick=\"location.href='./bscDataLive/'\"><p>BMS Daten</p></div>"
    "</div>"
  "</div>"
"</body>"
"</html>";


const char htmlPageBmsSpg[] PROGMEM = "<!doctype html>"
"<html>"
"<head>"
  "<meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "<title>BSC</title>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"

  "<style>"
  ".chart-wrap {margin-left: 50px; font-family: sans-serif; height: 650px; width: 300px;}"
  "//.chart-wrap .title {font-weight: bold; font-size: 1.62em; padding: 0.5em 0 1.8em 0; text-align: center; white-space: nowrap;}"
  ".chart-wrap.vertical .grid {transform: translateY(-175px) translateX(175px) rotate(-90deg);}"
  ".chart-wrap.vertical .grid .bar::after {transform: translateY(-50%); display: block;}"
  ".chart-wrap.vertical .grid::before,"
  ".chart-wrap.vertical .grid::after {transform: translateX(-0.2em) rotate(90deg);}"
  ".chart-wrap .grid {position: relative; padding: 5px 0 5px 0; height: 100%; width: 100%; border-left: 2px solid #aaa; "
    "background: repeating-linear-gradient(90deg, transparent, transparent 19.5%, rgba(170, 170, 170, 0.7) 20%);}"
  ".chart-wrap .grid::before {font-size: 0.8em; font-weight: bold; content: '2,5 V'; position: absolute; left: -0.5em; top: -1.5em;}"
  ".chart-wrap .grid::after {font-size: 0.8em; font-weight: bold; content: '4,0 V'; position: absolute; right: -1.5em; top: -1.5em;}"
  ".chart-wrap .bar {width: 0%; height: 20px; margin: 10px 0; background-color: #F16335; border-radius: 0 3px 3px 0;}"
  ".chart-wrap .bar:hover {opacity: 0.7;}"
  "</style>"
"</head>"
 "<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Zellspannungen</span>"
  "</div>"
  "<br><br>"
  "<div class='chart-wrap vertical'>"
  "<div class='grid'>"
  "<div id='b0' class='bar'></div>"
  "<div id='b1' class='bar'></div>"
  "<div id='b2' class='bar'></div>"
  "<div id='b3' class='bar'></div>"
  "<div id='b4' class='bar'></div>"
  "<div id='b5' class='bar'></div>"
  "<div id='b6' class='bar'></div>"
  "<div id='b7' class='bar'></div>"
  "<div id='b8' class='bar'></div>"
  "<div id='b9' class='bar'></div>"
  "<div id='b10' class='bar'></div>"
  "<div id='b11' class='bar'></div>"
  "<div id='b12' class='bar'></div>"
  "<div id='b13' class='bar'></div>"
  "<div id='b14' class='bar'></div>"
  "<div id='b15' class='bar'></div>"
  "</div>"
  "</div>"

  "<script>"
  "function getData() {"
    "var xhttp = new XMLHttpRequest();"
    "xhttp.onreadystatechange = function() {"
      "if (this.readyState == 4 && this.status == 200) {"
        "let values = this.responseText.split(';');"
        "let n = 0;"
        "for (let i=0; i<(values.length/2); i++) {"
          "let divId = 'b' + i;"
          "document.getElementById(divId).style.width = values[n] + '%';"
          "document.getElementById(divId).innerHTML = values[n+1] + '&nbsp;';"
          "n=n+2;"
        "}"
      "}"
    "};"
    "xhttp.open('GET', 'getBmsSpgData', true);"
    "xhttp.timeout=1000;"
    "xhttp.send();"
    "var timer = window.setTimeout('getData()', 1000);"
  "}"
  "getData();"
  "</script>"
"</body>"
"</html>";


const char htmlPageStatus[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='hl'>Battery Safety Controller - Status</span>"
  "</div>"
  "<div class='content'>"
    "<pre id='status'></pre>"
  "</div>"
"</body>"
"<script>"
"function getData() {"
  "var xhttp = new XMLHttpRequest();"
  "xhttp.onreadystatechange = function() {"
    "if (this.readyState == 4 && this.status == 200) {"
      "const myObj = JSON.parse(this.responseText);"
      "document.getElementById('status').innerHTML = JSON.stringify(myObj, undefined, 3);"
    "}"
  "};"
  "xhttp.open('GET', 'getDashboardData', true);"
  "xhttp.timeout=1000;"
  "xhttp.send();"
  "var timer = window.setTimeout('getStatusData()', 2000);"
"}"
"getData();"
"</script>"
"</html>";


const char htmlPageOwTempLive[] PROGMEM = "<!doctype html>"
"<html>"
"<head>"
"<meta charset='utf-8'/>"
"<title></title>"
"__PYVAR_HTML_WEBPAGES_STYLE__"

"<style>"
  ".bar {"
  "border: solid #808080 2px;"
  "border-radius: 5px;"
  "-moz-box-shadow: 3px 3px 3px #C0C0C0;"
  "-webkit-box-shadow: 3px 3px 3px #C0C0C0;"
  "box-shadow: 3px 3px 3px #C0C0C0;"
  "padding: 3px;"
  "width: 400px;"
  "height: 10px;"
  "}"
"</style>"

"<script>"
  "function addSensor(sensorNr){"
  "elem = document.createElement('span');"
  "elem.innerHTML ='<br>Sensor #' + sensorNr + ' - ';"
  "document.body.appendChild(elem);"

  "elem = document.createElement('span');"
  "elem.id = 'temps'+sensorNr;"
  "elem.innerHTML ='-- °C';"
  "document.body.appendChild(elem);"

  "elem = document.createElement('br');"
  "document.body.appendChild(elem);"

  "elem = document.createElement('progress');"
  "elem.id = 'temp'+sensorNr;"
  "elem.className = 'bar';"
  "elem.value = 0;"
  "elem.max = 50;"
  "document.body.appendChild(elem);"
  "} "

  "function setSensorVal(sensorNr, sensorValue){"
  "let elem = document.getElementById('temp'+sensorNr);"
  "elem.value = sensorValue;"

  "elem = document.getElementById('temps'+sensorNr);"
  "elem.innerHTML = '<b>'+sensorValue+' °C<b/';"
  "}"

  "function getData() {"
  "var xhttp = new XMLHttpRequest();"
  "xhttp.onreadystatechange = function(){"
  "  if (this.readyState == 4 && this.status == 200){"
  "    var values = this.responseText.split(';');"
  "    for(i=0;i<64;i++){"
  "	   if(values[i]<150){setSensorVal(i, values[i]);}"
  "    }"
  "  }"
  "};"
  "xhttp.open('GET', 'getOwTempData', true);"
  "xhttp.timeout=1000;"
  "xhttp.send();"
  "var timer = window.setTimeout('getData()', 10000);"
  "}"
"</script>"
"</head>"

"<body>"
"<div class='topnav'>"
  "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
  "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
  "<span class='hl'>Temperaturen (OW)</span>"
"</div>"

"<script>"
"for(i=0;i<64;i++){addSensor(i);} "
"getData();"
"</script>"
"</body>"
"</html>";





const char htmlPageBscDataLive[] PROGMEM = "<!doctype html>"
"<html>"
"<head>"
"<meta charset='utf-8'/>"
"<title></title>"
"__PYVAR_HTML_WEBPAGES_STYLE__"

"<Style>"
".cardsL {padding:20px;max-width:1300px;margin:0 auto;display:grid;grid-gap:2rem;grid-template-columns:repeat(auto-fill,minmax(500px,1fr));}"
"</Style>"

"<script>"
  "function addBmsCard(devName, devIdName, devNr){"
    "let temp = document.getElementsByTagName('template')[0];"
    "let newCard = temp.content.cloneNode(true);"
    "newCard.getElementById('cardHead').innerHTML=devName+' ('+devNr+')';"
    "for(i=0;i<20;i++){newCard.getElementById('bms0_zellVoltage'+i).id=devIdName+devNr+'_zellVoltage'+i;}"
    "newCard.getElementById('bms0_maxZellDiff').id=devIdName+devNr+'_maxZellDiff';"
    "newCard.getElementById('bms0_balOn').id=devIdName+devNr+'_balOn';"
    "newCard.getElementById('bms0_maxCellVolt').id=devIdName+devNr+'_maxCellVolt';"
    "newCard.getElementById('bms0_FetState').id=devIdName+devNr+'_FetState';"
    "newCard.getElementById('bms0_minCellVolt').id=devIdName+devNr+'_minCellVolt';"
    "newCard.getElementById('bms0_bmsErr').id=devIdName+devNr+'_bmsErr';"
    "newCard.getElementById('bms0_maxCellVoltNr').id=devIdName+devNr+'_maxCellVoltNr';"
    "newCard.getElementById('bms0_minCellVoltNr').id=devIdName+devNr+'_minCellVoltNr';"
    "newCard.getElementById('bms0_soc').id=devIdName+devNr+'_soc';"
    "newCard.getElementById('bms0_totalVolt').id=devIdName+devNr+'_totalVolt';"
    "newCard.getElementById('bms0_totalCurr').id=devIdName+devNr+'_totalCurr';"
    "document.getElementById('cards').appendChild(newCard);"
  "}"

  "function addAllBmsCards(){"
    "for(a=0;a<11;a++) addBmsCard('BMS','bms',a);"
    "for(b=0;b<7;b++) addBmsCard('BT BMS','btbms',b);"
  "}"

  "function getData() {"
  "var xhttp = new XMLHttpRequest();"
  "xhttp.onreadystatechange = function(){"
    "if (this.readyState == 4 && this.status == 200){"
      "const jsonObj = JSON.parse(this.responseText);\n"

      "for(k=0;k<11;k++){\n"
        "for(i=0;i<20;i++){\n"
          "if(jsonObj.bms_serial[k].cell_voltage[i] > 0){"
          "document.getElementById('bms'+k+'_zellVoltage'+i).innerHTML=jsonObj.bms_serial[k].cell_voltage[i];}\n"
          "else{document.getElementById('bms'+k+'_zellVoltage'+i).innerHTML=0;}"
        "}\n"
        "document.getElementById('bms'+k+'_totalVolt').innerHTML=jsonObj.bms_serial[k].totalVolt;\n"
        "document.getElementById('bms'+k+'_totalCurr').innerHTML=jsonObj.bms_serial[k].totalCurr;\n"
        "document.getElementById('bms'+k+'_soc').innerHTML=jsonObj.bms_serial[k].soc;\n"
        "document.getElementById('bms'+k+'_maxZellDiff').innerHTML=jsonObj.bms_serial[k].maxZellDiff;\n"
        "document.getElementById('bms'+k+'_maxCellVolt').innerHTML=jsonObj.bms_serial[k].maxCellVolt;\n"
        "document.getElementById('bms'+k+'_minCellVolt').innerHTML=jsonObj.bms_serial[k].minCellVolt;\n"
        "document.getElementById('bms'+k+'_maxCellVoltNr').innerHTML=jsonObj.bms_serial[k].maxCellVoltNr;\n"
        "document.getElementById('bms'+k+'_minCellVoltNr').innerHTML=jsonObj.bms_serial[k].minCellVoltNr;\n"
        "document.getElementById('bms'+k+'_balOn').innerHTML=jsonObj.bms_serial[k].balOn;\n"
        "document.getElementById('bms'+k+'_FetState').innerHTML=jsonObj.bms_serial[k].FetState;\n"
        "document.getElementById('bms'+k+'_bmsErr').innerHTML=jsonObj.bms_serial[k].bmsErr;\n"
      "}\n"

      "for(k=0;k<7;k++){\n"
        "for(i=0;i<20;i++){\n"
          "if(jsonObj.bms_bt[k].cell_voltage[i] > 0){"
          "document.getElementById('btbms'+k+'_zellVoltage'+i).innerHTML=jsonObj.bms_bt[k].cell_voltage[i];}\n"
          "else{document.getElementById('btbms'+k+'_zellVoltage'+i).innerHTML=0;}"
        "}\n"
        "document.getElementById('btbms'+k+'_totalVolt').innerHTML=jsonObj.bms_bt[k].totalVolt;\n"
        "document.getElementById('btbms'+k+'_totalCurr').innerHTML=jsonObj.bms_bt[k].totalCurr;\n"
        "document.getElementById('btbms'+k+'_soc').innerHTML=jsonObj.bms_bt[k].soc;\n"
        "document.getElementById('btbms'+k+'_maxZellDiff').innerHTML=jsonObj.bms_bt[k].maxZellDiff;\n"
        "document.getElementById('btbms'+k+'_maxCellVolt').innerHTML=jsonObj.bms_bt[k].maxCellVolt;\n"
        "document.getElementById('btbms'+k+'_minCellVolt').innerHTML=jsonObj.bms_bt[k].minCellVolt;\n"
        "document.getElementById('btbms'+k+'_maxCellVoltNr').innerHTML=jsonObj.bms_bt[k].maxCellVoltNr;\n"
        "document.getElementById('btbms'+k+'_minCellVoltNr').innerHTML=jsonObj.bms_bt[k].minCellVoltNr;\n"
        "document.getElementById('btbms'+k+'_balOn').innerHTML=jsonObj.bms_bt[k].balOn;\n"
        "document.getElementById('btbms'+k+'_FetState').innerHTML=jsonObj.bms_bt[k].FetState;\n"
        "document.getElementById('btbms'+k+'_bmsErr').innerHTML=jsonObj.bms_bt[k].bmsErr;\n"
      "}\n"

    "}\n"
  "};"
  "xhttp.open('GET', 'getBscLiveData', true);"
  "xhttp.timeout=6000;"
  "xhttp.send();"
  "var timer = window.setTimeout('getData()', 5000);"
  "}"
"</script>"

"</head>"
"<body>"

"<div class='topnav'>"
  "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
  "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
  "<span class='hl'>BSC Daten</span>"
"</div>"

"<div class='content'>"
  "<div id='cards' class='cardsL'></div>"
"</div>"

"<template>"
"<div class='cardDash' style='min-width: 500px'><p id='cardHead'>BMS serial 0</p><center>"
"<table style='font-size:16px;'><tr>"
  "<td>0: </td><td><span id='bms0_zellVoltage0'>----</span>&nbsp;mV</td>"
  "<td>4: </td><td><span id='bms0_zellVoltage4'>----</span>&nbsp;mV</td>"
  "<td>8: </td><td><span id='bms0_zellVoltage8'>----</span>&nbsp;mV</td>"
  "<td>12: </td><td><span id='bms0_zellVoltage12'>----</span>&nbsp;mV</td>"
  "<td>16: </td><td><span id='bms0_zellVoltage16'>----</span>&nbsp;mV</td>"
"</tr><tr>"
  "<td>1: </td><td><span id='bms0_zellVoltage1'>----</span>&nbsp;mV</td>"
  "<td>5: </td><td><span id='bms0_zellVoltage5'>----</span>&nbsp;mV</td>"
  "<td>9: </td><td><span id='bms0_zellVoltage9'>----</span>&nbsp;mV</td>"
  "<td>13: </td><td><span id='bms0_zellVoltage13'>----</span>&nbsp;mV</td>"
  "<td>17: </td><td><span id='bms0_zellVoltage17'>----</span>&nbsp;mV</td>"
"</tr><tr>"
  "<td>2: </td><td><span id='bms0_zellVoltage2'>----</span>&nbsp;mV</td>"
  "<td>6: </td><td><span id='bms0_zellVoltage6'>----</span>&nbsp;mV</td>"
  "<td>10: </td><td><span id='bms0_zellVoltage10'>----</span>&nbsp;mV</td>"
  "<td>14: </td><td><span id='bms0_zellVoltage14'>----</span>&nbsp;mV</td>"
  "<td>18: </td><td><span id='bms0_zellVoltage18'>----</span>&nbsp;mV</td>"
"</tr><tr>"
  "<td>3: </td><td><span id='bms0_zellVoltage3'>----</span>&nbsp;mV</td>"
  "<td>7: </td><td><span id='bms0_zellVoltage7'>----</span>&nbsp;mV</td>"
  "<td>11: </td><td><span id='bms0_zellVoltage11'>----</span>&nbsp;mV</td>"
  "<td>15: </td><td><span id='bms0_zellVoltage15'>----</span>&nbsp;mV</td>"
  "<td>19: </td><td><span id='bms0_zellVoltage19'>----</span>&nbsp;mV</td>"
"</tr></table>"

"<table style='font-size:16px;'><tr>"
  "<td>Max. Cell difference voltage</td><td><span id='bms0_maxZellDiff'></span>&nbsp;mV</td>"
  "<td>Balancing active</td><td><span id='bms0_balOn'></span>&nbsp;</td>"
"<tr><tr>"
  "<td>Max. cellvoltage</td><td><span id='bms0_maxCellVolt'></span>&nbsp;mV</td>"
  "<td>bmsStateFETs</td><td><span id='bms0_FetState'></span>&nbsp;</td>"
"</tr><tr>"
  "<td>Min. cellvoltage</td><td><span id='bms0_minCellVolt'></span>&nbsp;mV</td>"
  "<td>BMS errors</td><td><span id='bms0_bmsErr'></span>&nbsp;</td>"
"</tr><tr>"
  "<td>Max. voltage cellnumber</td><td><span id='bms0_maxCellVoltNr'></span>&nbsp;</td>"
  "<td></td><td><span></span></td>"
"</tr><tr>"
  "<td>Min. voltage cellnumber</td><td><span id='bms0_minCellVoltNr'></span>&nbsp;</td>"
  "<td>SoC</td><td><span id='bms0_soc'></span>&nbsp;%</td>"
"</tr><tr>"
  "<td>Total voltage</td><td><span id='bms0_totalVolt'></span>&nbsp;V</td>"
  "<td>Total current</td><td><span id='bms0_totalCurr'></span>&nbsp;A</td>"
"</tr></table>"
"</center></div>"
"</template>"

"<script>"
"addAllBmsCards();"
"getData();"
"</script>"

"</body>"
"</html>";


const char htmlPageSupport[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<head>"
  "<title>BSC</title>"
  "<meta http-equiv='Content-Type' content='text/html; charset=utf-8'>"
  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
  "<link rel='icon' href='data:,'>"
  "__PYVAR_HTML_WEBPAGES_STYLE__"
"</head>"
"<body>"
  "<div class='topnav'>"
    "<span class='btnBack' onclick=location.href='/'>&#10094;&#10094;</span>"
    "<span class='btnBack' onclick=\"location.href='../'\">&#10094;</span>"
    "<span class='hl'>Unterst&uuml;tzung</span>"
  "</div>"
  "<div class='content' style='max-width:500px'>"
    "<p>Habt ihr schon mal darüber nachgedacht, wie viele Stunden an Herzblut und Koffein in die Entwicklung von Open Source Projekten fließen? Nun, ich kann euch sagen, es ist eine ganze Menge!</p>"
    "<p>Aber hier ist die Sache: Ich liebe, was ich tue. Ich lieben es, innovative Lösungen zu entwickeln und ich lieben es, diese Lösungen der Welt frei zugänglich zu machen.</p>"
    "<p>Aber ich könnte ein bisschen Unterstützung gebrauchen, um weiterhin großartige Arbeit zu leisten. Wenn euch Battery Safety Controller geholfen hat, dann bitte ich euch herzlich, über eine Spende nachzudenken.</p>"
    "<p>Eure großzügige Unterstützung ermöglicht es mir, weiterhin neue Funktionen zu entwickeln und Bugs zu bekämpfen. Nur so kann das Projekt auch in der Zukunft erfolgreich sein.</p>"
    "<p>Vielen Dank im Voraus für eure Großzügigkeit und eure Unterstützung!</p>"
    "<p>Mit elektrischen Grüßen,<br>shiningman</p>"
    "<p>GitHub Sponsor: <a href=\"javascript:window.open('https://github.com/sponsors/shining-man');\">https://github.com/sponsors/shining-man</a></p>"

    "<p>Mit elektrischen Grüßen,<br>shiningman</p>"
  "</div>"
"</body>"
"</html>";


const char htmlPageUpload[] PROGMEM = "<!DOCTYPE HTML>"
"<html>"
"<form method='POST' action='/uploadPbnFw' enctype='multipart/form-data'>"
"<input type='file' name='[]' multiple><button>Upload</button></form>"
"</html>";

const char htmlFavicon[] PROGMEM = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
"<svg width=\"100mm\" height=\"100mm\" version=\"1.1\" viewBox=\"0 0 100 100\" xmlns=\"http://www.w3.org/2000/svg\">"
"<rect x=\"4.7268\" y=\"37.342\" width=\"11.975\" height=\"23.161\" fill=\"#0cf\"/>"
"<rect x=\"20.422\" y=\"14.18\" width=\"17.804\" height=\"70.902\" fill=\"#2ad4ff\"/>"
"<rect x=\"41.947\" y=\"14.105\" width=\"17.804\" height=\"70.902\" fill=\"#80e5ff\"/>"
"<rect x=\"63.472\" y=\"14.405\" width=\"17.804\" height=\"70.902\" fill=\"#aef\"/>"
"<text x=\"35.714909\" y=\"56.70203\" fill=\"#ffffff\" font-family=\"sans-serif\" font-size=\"19.756px\" stroke-width=\".26458\" text-align=\"end\" text-anchor=\"end\" style=\"line-height:1.25\" xml:space=\"preserve\"><tspan x=\"35.714909\" y=\"56.70203\" fill=\"#ffffff\" font-family=\"Arial\" font-size=\"19.756px\" stroke-width=\".26458\">B</tspan></text>"
"<text x=\"57.510044\" y=\"56.70203\" fill=\"#ffffff\" font-family=\"sans-serif\" font-size=\"19.756px\" stroke-width=\".26458\" text-align=\"end\" text-anchor=\"end\" style=\"line-height:1.25\" xml:space=\"preserve\"><tspan x=\"57.510044\" y=\"56.70203\" fill=\"#ffffff\" font-family=\"Arial\" font-size=\"19.756px\" stroke-width=\".26458\">S</tspan></text>"
"<text x=\"79.406464\" y=\"56.70203\" fill=\"#fffefc\" font-family=\"sans-serif\" font-size=\"19.756px\" stroke-width=\".26458\" text-align=\"end\" text-anchor=\"end\" style=\"line-height:1.25\" xml:space=\"preserve\"><tspan x=\"79.406464\" y=\"56.70203\" fill=\"#fffefc\" font-family=\"Arial\" font-size=\"19.756px\" stroke-width=\".26458\">C</tspan></text>"
"</svg>";