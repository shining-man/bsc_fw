// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "bscTime.h"
#include "defines.h"
#include "Arduino.h"
#include <NTP.h>
#include <WiFiUdp.h>
#include "WebSettings.h"

static const char *TAG = "TIME";

//I used the NTP lib from: https://github.com/sstaub/NTP

WiFiUDP ntpUDP;
NTP timeClient(ntpUDP);

bool bo_lNtpEnable=false;
uint8_t u8_mNtpTiemout=0;


void initTime()
{
  String str_lNtpServerName = WebSettings::getStringFlash(ID_PARAM_SYSTEM_NTP_SERVER_NAME,0);
  //uint16_t u16_lNtpServerPort = WebSettings::getIntFlash(ID_PARAM_SYSTEM_NTP_SERVER_PORT,0,0,0,DT_ID_PARAM_SYSTEM_NTP_SERVER_PORT);
  
  if(str_lNtpServerName.length()==0)return;
  bo_lNtpEnable=true;

  timeClient.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00,   timetone +120min (+1 GMT + 1h summertime offset)
  timeClient.ruleSTD("CET", Last, Sun, Oct, 3, 60);   // last sunday in october 3:00, timezone +60min (+1 GMT)
  timeClient.isDST(true);
  timeClient.updateInterval((uint32_t)600000); //All 10 minutes

  IPAddress ipAdr;
  if (ipAdr.fromString(str_lNtpServerName.c_str()))
  {
    BSC_LOGI(TAG,"Init NTP (IP): server=%s",ipAdr.toString().c_str());
    timeClient.begin(ipAdr);
  }
  else
  {
    BSC_LOGI(TAG,"Init NTP (NAME): server=%s",str_lNtpServerName.c_str());
    timeClient.begin(str_lNtpServerName.c_str());
  }
}


bool timeRunCyclic(bool bo_resetTimeout)
{
  if(bo_lNtpEnable==false) return false;
  if(bo_resetTimeout)u8_mNtpTiemout=0;
  if(u8_mNtpTiemout>=3) return false;

  int8_t i8_lNtpRet=timeClient.update();
  if(i8_lNtpRet==1)
  {
    u8_mNtpTiemout=0;
    setTimeFromNTP();
    return true;
  }
  else if(i8_lNtpRet==2) //timeout
  {
    u8_mNtpTiemout++;
  }

  return false;
}


String getBscTime()
{
  return String(timeClient.formattedTime("%H:%M:%S"));
}


String getBscDateTime()
{
  return String(timeClient.formattedTime("%Y-%m-%d %H:%M:%S"));
}

const char* getBscDateTimeCc()
{
  return timeClient.formattedTime("%Y-%m-%d %H:%M:%S");
}

const char* getBscDateTimeCc2()
{
  return timeClient.formattedTime("%Y%m%d%H%M%S");
}

uint32_t getEpoch()
{
  return (uint32_t) timeClient.epoch();
}

uint8_t getMinutes()
{
  return timeClient.minutes();
}

uint32_t getDayMinutes()
{
  return (timeClient.hours()*60) + timeClient.minutes();
}

void setTimeFromNTP()
{
  struct timeval tv;
  //struct timezone tz;
	uint32_t u32_lEpoch = (uint32_t)timeClient.localEpoch();

  if(u32_lEpoch>2082758399) tv.tv_sec=u32_lEpoch-2082758399;  // epoch time (seconds)
  else tv.tv_sec=u32_lEpoch;  // epoch time (seconds)
  tv.tv_usec=0;    // microseconds

  //tz.tz_dsttime = DST_MET;
  //tz.tz_minuteswest = 60;

  settimeofday(&tv, NULL);
}
