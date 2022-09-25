// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "AlarmRules.h"
#include "dio.h"
#include "Ow.h"
#include "Canbus.h"
#include "BmsData.h"
#include "mqtt_t.h"


TimerHandle_t timer_doOffPulse;
SemaphoreHandle_t doMutex = NULL;


bool bo_Alarm[CNT_ALARMS];
bool bo_Alarm_old[CNT_ALARMS];

uint32_t bo_DoPulsOffCounter[CNT_DIGITALOUT];
uint8_t u8_DoVerzoegerungTimer[CNT_DIGITALOUT];

bool bo_alarmActivate[CNT_ALARMS]; //Merker ob ein Alarm in diesem 'run' gesetzt wurde
bool bo_timerPulseOffIsRunning;

uint8_t u8_mDoByte;


void rules_Bms();
void rules_Temperatur();
void rules_CanInverter();
bool temperatur_maxWertUeberwachung(uint8_t);
bool temperatur_maxWertUeberwachungReferenz(uint8_t);
bool temperatur_DifferenzUeberwachung(uint8_t);
void runDigitalAusgaenge();
void doOffPulse(TimerHandle_t xTimer);
void getDIs();
void setDOs();

void initAlarmRules()
{
  u8_mDoByte = 0;
  bo_timerPulseOffIsRunning = false;

  for(uint8_t i=0;i<CNT_ALARMS;i++)
  {
    bo_Alarm[i] = false;
    bo_Alarm_old[i] = false;

    //Initialwerte per mqqt senden
    if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
    {
      mqttPublish("alarm/"+String(i+1), bo_Alarm[i]);
    }  
  }

  for(uint8_t i=0;i<CNT_DIGITALOUT;i++)
  {
    bo_DoPulsOffCounter[i] = 0;
    u8_DoVerzoegerungTimer[i] = 0xFF;
  }

  doMutex = xSemaphoreCreateMutex();

  timer_doOffPulse = xTimerCreate("doPulse", pdMS_TO_TICKS(10), pdFALSE, (void *)1, &doOffPulse);
  assert(timer_doOffPulse);

}

bool getAlarm(uint8_t alarmNr)
{
  return bo_Alarm[alarmNr];
}

void setAlarm(uint8_t alarmNr, bool bo_lAlarm)
{
  alarmNr--;
  if(bo_lAlarm)
  {
    bo_Alarm[alarmNr]=true;
    bo_alarmActivate[alarmNr]=true;
  }
  else if(bo_alarmActivate[alarmNr]!=true)
  {
    bo_Alarm[alarmNr]=bo_lAlarm;
    if(bo_lAlarm)
    {
      bo_alarmActivate[alarmNr]=true;
    }
  }
}

//Wird vom Task aus der main.c zyklisch aufgerufen
void runAlarmRules()
{
  uint8_t i,tmp;

  //Toggle LED
  u8_mDoByte ^= (1 << 7); 

  //Merker vor jedem run auf false setzen
  for(i=0;i<CNT_ALARMS;i++){bo_alarmActivate[i]=false;}

  //Bluetooth+Serial BMS
  rules_Bms();

  //Temperatur Alarm
  rules_Temperatur();

  //Inverter (CAN)
  rules_CanInverter();

  //Digitaleingänge
  getDIs();

  //Bearbeiten der Alarme
  for(i=0; i<CNT_ALARMS; i++)
  {
    if(bo_Alarm[i]!=bo_Alarm_old[i]) //Flankenwechsel
    {
      //Serial.printf("Alarm (%i) Flanke, new State=%i",i,bo_Alarm[i]);
      bo_Alarm_old[i] = bo_Alarm[i];

      //Bei Statusänderung mqqt msg absetzen
      if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
      {
        mqttPublish("alarm/"+String(i+1), bo_Alarm[i]);
      }
      
      //Bearbeiten der 6 Relaisausgaenge + 1 OptoOut
      for(uint8_t o=0; o<CNT_DIGITALOUT; o++)
      {
        tmp=WebSettings::getInt(ID_PARAM_DO_AUSLOESUNG_BEI,0,o,0)-1;
        if(tmp==i)
        {
          if(bo_Alarm[i]==true)
          {     
            if(u8_DoVerzoegerungTimer[i]==0xFF) //Verzoegerungstimer nur starten wenn er noch nicht läuft
            {
              ESP_LOGD("AlarmRules", "Set DO VerzoegerungTimer (%i)", o);
              u8_DoVerzoegerungTimer[i] = WebSettings::getInt(ID_PARAM_DO_VERZOEGERUNG,0,o,0);
            }
          }
          else
          {
            uint8_t doPulseOrPermanent = WebSettings::getInt(ID_PARAM_DO_AUSLOESEVERHALTEN,0,o,0);
            if(doPulseOrPermanent==0) //Wenn Permanent
            {
              ESP_LOGD("AlarmRules", "Alarm geht (%i)", i);
              u8_mDoByte &= ~(1 << o); //bit loeschen
            }
          }
        }
      }
    }
  }

  setDOs();
}


void setDOs()
{
  //Bearbeiten der 6 Relaisausgaenge + 1 OptoOut
  for(uint8_t o=0; o<CNT_DIGITALOUT; o++)
  {
    if(u8_DoVerzoegerungTimer[o]==0)
    {
      u8_DoVerzoegerungTimer[o]=0xFF;

      ESP_LOGD("AlarmRules", "Set DO (%i)", o);
      u8_mDoByte |= (1 << o); //bit setzen

      uint8_t doPulseOrPermanent = WebSettings::getInt(ID_PARAM_DO_AUSLOESEVERHALTEN,0,o,0);
      if(doPulseOrPermanent==1) //Wenn Impuls
      {
        uint32_t pulseDuration = WebSettings::getInt(ID_PARAM_DO_IMPULSDAUER,0,o,0);
      ESP_LOGD("AlarmRules", "DO Impuls DO=%i Dauer=%i", o, pulseDuration);
              
        xSemaphoreTake(doMutex, portMAX_DELAY);
        bo_DoPulsOffCounter[o] = (pulseDuration/10);
        if(bo_timerPulseOffIsRunning==false)
        {
          bo_timerPulseOffIsRunning=true;
          if (xTimerStart(timer_doOffPulse, 10) != pdPASS){ /*Fehler beim Starten des Timers*/ }
        }
        xSemaphoreGive(doMutex);
      }
    }
    else
    {
      if(u8_DoVerzoegerungTimer[o]!=0xFF) u8_DoVerzoegerungTimer[o]--;
    }
  }


  //Setze Ausgänge, lese Eingänge
  xSemaphoreTake(doMutex, portMAX_DELAY);
  setDoData(u8_mDoByte);
  xSemaphoreGive(doMutex);
}


void getDIs()
{
  //Lese der Eingänge
  xSemaphoreTake(doMutex, portMAX_DELAY);
  uint8_t u8_lDiData = dioRwInOut();
  xSemaphoreGive(doMutex);

  //Bearbeiten der 4 Digitaleingänge
  for(uint8_t i=0; i<CNT_DIGITALIN; i++)
  {
    uint8_t u8_lAlarmNr  = WebSettings::getInt(ID_PARAM_DI_ALARM_NR,0,i,0);
    bool    bo_lDiInvert = WebSettings::getBool(ID_PARAM_DI_INVERTIERT,0,i,0);

    if((u8_lDiData & (1<<i)) == (1<<i))
    {
      if(!bo_lDiInvert) setAlarm(u8_lAlarmNr,true);
      else setAlarm(u8_lAlarmNr,false);
    }
    else
    {
      if(!bo_lDiInvert) setAlarm(u8_lAlarmNr,false);
      else setAlarm(u8_lAlarmNr,true);
    }
  }
}


void rules_Bms()
{
  uint8_t i,tmp;

  for(i=0; i<BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT; i++)
  {
    bool b_lBmsOnline=true;
    if((millis()-getBmsLastDataMillis(i))>2000) b_lBmsOnline=false; //Wenn 2000 ms keine Daten vom BMS kamen, dann ist es offline
    //Serial.printf("i=%i, b_lBmsOnline=%i\n",i,b_lBmsOnline);

    //Ist überhaupt ein Device parametriert?
    if((i<BT_DEVICES_COUNT && !WebSettings::getString(ID_PARAM_SS_BTDEV,0,i,0).equals(String(ID_BT_DEVICE_NB)) && !WebSettings::getString(ID_PARAM_SS_BTDEVMAC,0,i,0).equals("")) ||
      (i>=BT_DEVICES_COUNT && WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,i-BT_DEVICES_COUNT,0)!=0 ) )
    {
      //Wenn Alram für das Device aktiv ist
      if(WebSettings::getBool(ID_PARAM_ALARM_BTDEV_ALARM_ON,0,i,0)) 
      {      
        //Alarm wenn keine Daten mehr vom BT-Device kommen
        if((millis()-getBmsLastDataMillis(i))>(WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT,0,i,0)*1000))
        {
          //Alarm
          //Serial.printf("BT Alarm (%i)\n",i);
          tmp=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_AKTION,0,i,0);
          setAlarm(tmp,true);
        }
        else
        {
          tmp=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_AKTION,0,i,0);
          setAlarm(tmp,false);
        }
      }

      //Überwachung Zellspannung
      //Serial.printf("(i=%i) Zell Spg. Outside b_lBmsOnline=%i, enable=%i\n",i, b_lBmsOnline,WebSettings::getBool(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON,0,i,0));
      if(b_lBmsOnline==true && WebSettings::getBool(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON,0,i,0)) //BleHandler::bmsIsConnect(i)
      {
      //Serial.printf("Zell Spg. Outside cellCnt=%i\n",WebSettings::getInt(ID_PARAM_ALARM_BT_CNT_CELL_CTRL,0,i,0));
        for(uint8_t cc=0; cc<WebSettings::getInt(ID_PARAM_ALARM_BT_CNT_CELL_CTRL,0,i,0); cc++)
        {
          //Serial.printf("i=%i, cc=%i, cellspg=%i, min=%i, max=%i\n",i,cc,getBmsCellVoltage(i,cc),WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MIN,0,i,0),WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MAX,0,i,0));
          if(getBmsCellVoltage(i,cc) < WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MIN,0,i,0) || getBmsCellVoltage(i,cc) > WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MAX,0,i,0))
          {
            //Alarm
            tmp=WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,0,i,0);
            setAlarm(tmp,true);
            ESP_LOGD("AlarmRules", "Zell Spg. Outside (%i) alarmNr=%i\n", i,tmp);
            break; //Sobald eine Zelle Alarm meldet kann abgebrochen werden
          }
          else
          {
            tmp=WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,0,i,0);
            setAlarm(tmp,false);
          }
        }
      }

      //Überwachung Gesammtspannung
      uint8_t u8_lAlarm = WebSettings::getBool(ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION,0,i,0); //BleHandler::bmsIsConnect(i)
      if(b_lBmsOnline==true && u8_lAlarm>0) 
      {
        if(getBmsTotalVoltage(i) < WebSettings::getInt(ID_PARAM_ALARM_BT_GESAMT_SPG_MIN,0,i,0) || getBmsTotalVoltage(i) > WebSettings::getInt(ID_PARAM_ALARM_BT_GESAMT_SPG_MAX,0,i,0))
        {
          //Alarm
          setAlarm(u8_lAlarm,true);
        }
        else
        {
          setAlarm(u8_lAlarm,false);
        }
      }
    }
  }
}


void rules_Temperatur()
{
  uint8_t sensorVon, sensorBis, alarmNr;
  bool bo_lAlarm;

  for(uint8_t i=0;i<COUNT_TEMP_RULES;i++)
  {
    bo_lAlarm = false;
    sensorVon = WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,0,i,0);
    sensorBis = WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,0,i,0);
    
    if(sensorVon>=0 && sensorBis>=0 && sensorBis>=sensorVon)
    {
      switch (WebSettings::getInt(ID_PARAM_TEMP_ALARM_UEBERWACH_FUNKTION,0,i,0))
      {
        case ID_TEMP_ALARM_FUNKTION_NB:
          break;

        case ID_TEMP_ALARM_FUNKTION_MAXWERT:
          if(temperatur_maxWertUeberwachung(i)){bo_lAlarm=true;}
          break;

        case ID_TEMP_ALARM_FUNKTION_MAXWERT_REFERENZ:
          if(temperatur_maxWertUeberwachungReferenz(i)){bo_lAlarm=true;}
          break;

        case ID_TEMP_ALARM_FUNKTION_DIFFERENZ:
          if(temperatur_DifferenzUeberwachung(i)){bo_lAlarm=true;}
        
        default:
          break;
      }

      alarmNr=WebSettings::getInt(ID_PARAM_TEMP_ALARM_AKTION,0,i,0);
      setAlarm(alarmNr,bo_lAlarm);
    }
  }
}


void rules_CanInverter()
{
  //Ladeleistung bei Alarm auf 0 Regeln
  uint8_t alarmCurrentToZero = WebSettings::getInt(ID_PARAM_BMS_LADELEISTUNG_AUF_NULL,0,0,0);
  if(alarmCurrentToZero>0)
  {
      if(bo_Alarm[alarmCurrentToZero-1]==true){
        canSetChargeCurrentToZero(true);
      }else{
        canSetChargeCurrentToZero(false);
      }
  }

  //Entladeleistung bei Alarm auf 0 Regeln
  alarmCurrentToZero = WebSettings::getInt(ID_PARAM_BMS_ENTLADELEISTUNG_AUF_NULL,0,0,0);
  if(alarmCurrentToZero>0)
  {
      if(bo_Alarm[alarmCurrentToZero-1]==true){
        canSetDischargeCurrentToZero(true);
      }else{
        canSetDischargeCurrentToZero(false);
      }
  }

  //SOC bei Alarm auf 100 stellen
  alarmCurrentToZero = WebSettings::getInt(ID_PARAM_BMS_SOC_AUF_FULL,0,0,0);
  if(alarmCurrentToZero>0)
  {
      if(bo_Alarm[alarmCurrentToZero-1]==true){
        canSetSocToFull(true);
      }else{
        canSetSocToFull(false);
      }
  }

  
}


bool temperatur_maxWertUeberwachung(uint8_t i)
{
  float maxTemp = WebSettings::getInt(ID_PARAM_TEMP_ALARM_WERT1,0,i,0);

  if(maxTemp<=0) return false; //Wenn keine Max.Temp. angegeben ist

  for(uint8_t n=WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,0,i,0);n<WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,0,i,0)+1;n++)
  {
    if(owGetTemp(n)>maxTemp && owGetTemp(n)!=TEMP_IF_SENSOR_READ_ERROR)
    {
      return true;
    }
  }
  return false;
}


bool temperatur_maxWertUeberwachungReferenz(uint8_t i)
{
  float aktTemp = 0;
  float tempOffset = WebSettings::getInt(ID_PARAM_TEMP_ALARM_WERT1,0,i,0);

  aktTemp = tempOffset + owGetTemp(WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VERGLEICH,0,i,0));
  for(uint8_t n=WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,0,i,0);n<WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,0,i,0)+1;n++)
  {
    if(owGetTemp(n)>aktTemp && owGetTemp(n)!=TEMP_IF_SENSOR_READ_ERROR)
    {
      return true;
    }
  }
  return false;
}


bool temperatur_DifferenzUeberwachung(uint8_t i)
{
  float f_lTempMin = 0xFF;
  float f_lTempMax = 0;

  float f_lMaxTempDiff = WebSettings::getInt(ID_PARAM_TEMP_ALARM_WERT1,0,i,0);
  
  for(uint8_t n=WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,0,i,0);n<WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,0,i,0)+1;n++)
  {
    if(owGetTemp(n)>f_lTempMax) f_lTempMax=owGetTemp(n);
    if(owGetTemp(n)<f_lTempMin) f_lTempMin=owGetTemp(n);
  }

  if(f_lTempMax-f_lTempMin>=f_lMaxTempDiff)
  {
    return true;
  }
  return false;
}


//doOffPulse wird alle 10 ms aufgerufen solange der Timer läuft
void doOffPulse(TimerHandle_t xTimer)
{
  bool restartTimer = false;
  bool changeDoData = false;
  //doNr = ( uint32_t ) pvTimerGetTimerID( xTimer );

  xSemaphoreTake(doMutex, portMAX_DELAY);
  for(uint8_t i=0;i<CNT_DIGITALOUT;i++)
  {
    if(bo_DoPulsOffCounter[i] > 1)
    {
      bo_DoPulsOffCounter[i]--;
      restartTimer = true;
    }
    else if(bo_DoPulsOffCounter[i] == 1)
    {
      //Serial.printf("DO off - Impuls(%i)\n",i);

      bo_DoPulsOffCounter[i]=0;

      u8_mDoByte &= ~(1 << i); //bit loeschen
      changeDoData=true;
    }
  }

  //New DO Data
  if(changeDoData)
  {
    setDoData(u8_mDoByte);
    dioRwInOut();
  }

  //Timer neu starten solange noch nicht alle Relais abgeschalten sind
  if(restartTimer)
  {
    xTimerStart(timer_doOffPulse, 10);
  }
  else
  {
    bo_timerPulseOffIsRunning=false;
  }
  
  xSemaphoreGive(doMutex);
}