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

//enum enumOwSenprStae{owSensorOk, owSensorFailure};

uint8_t  owAddr[MAX_ANZAHL_OW_SENSOREN][8];
int16_t owTempsC[MAX_ANZAHL_OW_SENSOREN];
int16_t owTempsC_AvgCalc[MAX_ANZAHL_OW_SENSOREN];
//uint8_t owSensorState[MAX_ANZAHL_OW_SENSOREN]; //0=ok

uint8_t cycleCounter;
bool firstMeasurement;
uint32_t owReadErrorTimer;

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
    owAdr = WebSettings::getStringFlash(ID_PARAM_ONEWIRE_ADR,i);

    if (8 == sscanf(owAdr.c_str(), "%x:%x:%x:%x:%x:%x:%x:%x%*c",
        &owAddr[i][0], &owAddr[i][1], &owAddr[i][2], &owAddr[i][3],
        &owAddr[i][4], &owAddr[i][5], &owAddr[i][6], &owAddr[i][7]))
    {
      owAmount++;
    }
    else
    {
      // invalid Adr 
      for(uint8_t n=0;n<8;n++)
      {
        owAddr[i][n]=0;
      }

      //Wenn Adresse ungültig, dann Temp zurücksetzen
      owTempsC[i]=0;
      owTempsC_AvgCalc[i]=0;
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
  owReadErrorTimer=0;
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

void addErrorCounter()
{
  if(owReadErrorTimer==0)
  {
    owReadErrorTimer=millis();
    if(owReadErrorTimer==0)owReadErrorTimer=1;
  }
}

void resetErrorCounter()
{
  owReadErrorTimer=0;
}

uint8_t getErrorCounter()
{
  if(owReadErrorTimer==0) return 0;
  return (millis()-owReadErrorTimer)/1000;
}


//Wird vom Task aus der main.c zyklisch aufgerufen
void owCyclicRun()
{
  //Wenn onewire aktiviert wurde
  if(WebSettings::getBool(ID_PARAM_ONWIRE_ENABLE,0))
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


#if 0
#define FILTER_WEIGHT  0.5 //Gewicht von 0.01 bis 1.0; Ein Gewicht von 1 glättet am wenigsten
//Exponential-Filter
void filter(uint8_t sensNr, float fNew) //fCurrent
{
  owTempsC[sensNr] = (float)FILTER_WEIGHT * fNew + (1.0 - (float)FILTER_WEIGHT) * owTempsC[sensNr];
}
#endif


void getTempC_allDevices(bool tempToOutBuffer)
{
  float tempC;
  bool bo_lsensorHasError=false;
  for(uint8_t i=0;i<MAX_ANZAHL_OW_SENSOREN;i++)
  {
    if(owAddr[i][0]>0)
    {
      for(uint8_t r=0;r<3;r++) //Drei Versuche den Sensor auszulesen
      {
        tempC = (int16_t)(sensors.getTempC(&owAddr[i][0])*100);
        if (tempC == DEVICE_DISCONNECTED_C_U16)
        {
          //BSC_LOGI(TAG, "Error read temp: seonsor=%i, r=%i",i,r);
          if(r==2)
          {
            bo_lsensorHasError=true;
            //BSC_LOGI(TAG, "Error sensor %i nicht lesbar",i); //ToDo Fehlerzähler addieren
          }
        }
        else
        {
          //Offset abziehen
          tempC += (int16_t)(WebSettings::getFloat(ID_PARAM_ONWIRE_TEMP_OFFSET,i)*100);

          if(firstMeasurement) owTempsC_AvgCalc[i] = tempC;
          else owTempsC_AvgCalc[i] = (owTempsC_AvgCalc[i]+tempC)/2;
                    
          if(tempToOutBuffer==true)
          {
            if( (owTempsC_AvgCalc[i]>=owTempsC[i]+8) || (owTempsC_AvgCalc[i]<=owTempsC[i]-8) ) //0.08
            {
              owTempsC[i]=owTempsC_AvgCalc[i];
            }
          }
          break;
        }
      }
    }
  }

  if(bo_lsensorHasError) addErrorCounter();
  else resetErrorCounter();

  if(firstMeasurement){firstMeasurement=false;}
}


float owGetTemp(uint8_t sensNr)
{
  return (float)owTempsC[sensNr]/100;
}


uint8_t owGetAllSensorError()
{
  return getErrorCounter();
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
      owAddresses+="<button onclick='copyStringToClipboard(\"";
      owAddresses+=owAdr;
      owAddresses+="\")'>Copy</button>";
      owAddresses+="</td></tr>";
    } while (oneWire.search(address));
  }
  xSemaphoreGive(owMutex);

  owAddresses+="</table>";

  return owAddresses;
}
