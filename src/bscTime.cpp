// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "bscTime.h"
#include "Arduino.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);

void initTime()
{
  timeClient.begin();
}

void timeRunCyclic()
{
  timeClient.update();
}

String getBscTime()
{
  return timeClient.getFormattedTime();
}

String getBscDateTime()
{
  const time_t old = (time_t)timeClient.getEpochTime();
  struct tm *oldt = gmtime(&old);
  return asctime(oldt);
}
