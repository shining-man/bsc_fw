// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Ow.h"
#include "WebSettings.h"
#include "defines.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "log.h"

static const char *TAG = "OW";

static SemaphoreHandle_t owMutex = NULL;

uint8_t owAddr[MAX_ANZAHL_OW_SENSOREN][8];
float owTempsC[MAX_ANZAHL_OW_SENSOREN];
float owTempsC_AvgCalc[MAX_ANZAHL_OW_SENSOREN];

uint8_t cycleCounter;
bool firstMeasurement;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(OW_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);


String findDevices();
void getTempC_allDevices(bool);


uint8_t getSensorAdrFromParams()
{
  String owAdr = "";
  uint8_t owAmount = 0;
  for(uint8_t i=0;i<MAX_ANZAHL_OW_SENSOREN;i++)
  {
    owAdr = WebSettings::getString(ID_PARAM_ONEWIRE_ADR,0,0,i);

    if (8 == sscanf(owAdr.c_str(), "%x:%x:%x:%x:%x:%x:%x:%x%*c",
        &owAddr[i][0], &owAddr[i][1], &owAddr[i][2], &owAddr[i][3],
        &owAddr[i][4], &owAddr[i][5], &owAddr[i][6], &owAddr[i][7]))
    {
      owAmount++;
      //debugPrint("owAdr ok: ");
      //debugPrintln(owAdr);
    }
    else
    {
      // invalid Adr 
      for(uint8_t n=0;n<8;n++)
      {
        owAddr[i][n]=0;
      }
    }
  }

  return owAmount-1;
}


 /*
Conversion time from precision
precision | time
  12 bit  | 750 ms
  11 bit  | 375 ms
  10 bit  | 187 ms
   9 bit  |  93 ms
*/
void owSetup()
{
  cycleCounter=0;
  firstMeasurement=true;
  owMutex = xSemaphoreCreateMutex();

  takeOwSensorAddress();
}


//Immer aufrufen , wenn sich Adressen geändert haben
void takeOwSensorAddress()
{
  firstMeasurement=true;
  xSemaphoreTake(owMutex, portMAX_DELAY);
  sensors.begin();
  getSensorAdrFromParams();
  sensors.setResolution(12);
  xSemaphoreGive(owMutex);
}



//Wird vom Task aus der main.c zyklisch aufgerufen
void owCyclicRun()
{
  //Wenn onewire aktiviert wurde
  if(WebSettings::getBool(ID_PARAM_ONWIRE_ENABLE,0,0,0))
  {
    xSemaphoreTake(owMutex, portMAX_DELAY);
    if(cycleCounter==0)
    { 
      sensors.requestTemperatures();
    }
    else if(cycleCounter>0 && cycleCounter<OW_TEMP_AVG_CYCELS)
    {
      getTempC_allDevices(false);
      sensors.requestTemperatures();
    }
    else if(cycleCounter==OW_TEMP_AVG_CYCELS)
    {
      getTempC_allDevices(true);
    }
    else if(cycleCounter==(OW_TEMP_AVG_CYCELS+3))
    {
      cycleCounter=-1;
    }
    xSemaphoreGive(owMutex);
  }

  cycleCounter++;
}


#define FILTER_WEIGHT  0.5 //Gewicht von 0.01 bis 1.0; Ein Gewicht von 1 glättet am wenigsten
//Exponential-Filter
void filter(uint8_t sensNr, float fNew) //fCurrent
{
  owTempsC[sensNr] = (float)FILTER_WEIGHT * fNew + (1.0 - (float)FILTER_WEIGHT) * owTempsC[sensNr];
}



void getTempC_allDevices(bool tempToOutBuffer)
{
  float tempC;
  for(uint8_t i=0;i<MAX_ANZAHL_OW_SENSOREN;i++)
  {
    if(owAddr[i][0]>0)
    {
      for(uint8_t r=0;r<3;r++)
      {
        tempC = sensors.getTempC(&owAddr[i][0]);
        if (tempC == DEVICE_DISCONNECTED_C)
        {
          //debugPrintf("Error: Could not read temperature data [%i] r=%i",i,r);
          owTempsC[i]=TEMP_IF_SENSOR_READ_ERROR;
        }
        else
        {
          //Offset abziehen
          tempC += WebSettings::getFloat(ID_PARAM_ONWIRE_TEMP_OFFSET,0,i,0);

          if(firstMeasurement){
            owTempsC_AvgCalc[i] = tempC;
          }else{
            owTempsC_AvgCalc[i] = (owTempsC_AvgCalc[i]+tempC)/2;
          }
          
          if(tempToOutBuffer==true)
          {
            if( (owTempsC_AvgCalc[i]>=owTempsC[i]+0.08) || (owTempsC_AvgCalc[i]<=owTempsC[i]-0.08) )
            {
              owTempsC[i]=owTempsC_AvgCalc[i];
            }
          }
          break;
        }
      }
    }
  }

  if(firstMeasurement){firstMeasurement=false;}
}


float owGetTemp(uint8_t sensNr)
{
  return owTempsC[sensNr];
}


String getSensorAdr() {
  return findDevices();
}


String findDevices()
{
  uint8_t address[8];
  uint8_t count = 0;
  String owAddresses = "<table>";
  String owAdr = "";
  bool newAdr=false;

  xSemaphoreTake(owMutex, portMAX_DELAY);
  if (oneWire.search(address))
  {
    do {
      count++;
      newAdr=true;
      owAdr="";

      for(uint8_t n=0; n<MAX_ANZAHL_OW_SENSOREN; n++)
      {
        if(memcmp(address, owAddr[n], 8)==0)
        {
          newAdr=false;
        }
      }

      owAddresses+="<tr><td>";
      if(newAdr){owAddresses+="<b>";}
      for (uint8_t i = 0; i < 8; i++)
      {
        owAdr+=String(address[i],HEX);
        if (i < 7) owAdr+=":";
      }
      owAddresses+=owAdr;
      if(newAdr){owAddresses+="</b>";}
      owAddresses+="</td><td>";
      owAddresses+="<button onclick='navigator.clipboard.writeText(\"";
      owAddresses+=owAdr;
      owAddresses+="\")'>Copy</button>";
      owAddresses+="</td></tr>";
    } while (oneWire.search(address));
  }
  xSemaphoreGive(owMutex);

  owAddresses+="</table>";

  return owAddresses;
}
