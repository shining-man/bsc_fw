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
#include "FreqCountESP.h"

static const char *TAG = "ALARM";

TimerHandle_t timer_doOffPulse;
static SemaphoreHandle_t doMutex = NULL;
static SemaphoreHandle_t alarmSettingsChangeMutex = NULL;


bool bo_Alarm[CNT_ALARMS];
bool bo_Alarm_old[CNT_ALARMS];

uint32_t bo_DoPulsOffCounter[CNT_DIGITALOUT];
uint8_t u8_DoVerzoegerungTimer[CNT_DIGITALOUT];

bool bo_alarmActivate[CNT_ALARMS]; //Merker ob ein Alarm in diesem 'run' gesetzt wurde
bool bo_timerPulseOffIsRunning;

bool bo_mChangeAlarmSettings;

uint8_t u8_mDoByte;
uint8_t u8_mTachoChannel;

void rules_Bms();
void rules_Temperatur();
void rules_CanInverter();
void rules_Tacho();
bool temperatur_maxWertUeberwachung(uint8_t);
bool temperatur_maxWertUeberwachungReferenz(uint8_t);
bool temperatur_DifferenzUeberwachung(uint8_t);
void runDigitalAusgaenge();
void doOffPulse(TimerHandle_t xTimer);
void getDIs();
void setDOs();
void tachoInit();
bool tachoRead(uint16_t &tachoRpm);
void tachoSetMux(uint8_t channel);


void initAlarmRules()
{
  u8_mDoByte = 0;
  bo_timerPulseOffIsRunning = false;
  bo_mChangeAlarmSettings=false;

  for(uint8_t i=0;i<CNT_ALARMS;i++)
  {
    bo_Alarm[i] = false;
    bo_Alarm_old[i] = false;

    //Initialwerte per mqqt senden
    if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
    {
      mqttPublish(MQTT_TOPIC_ALARM, i+1, -1, -1, bo_Alarm[i]);
      
    }  
  }

  for(uint8_t i=0;i<CNT_DIGITALOUT;i++)
  {
    bo_DoPulsOffCounter[i] = 0;
    u8_DoVerzoegerungTimer[i] = 0xFF;
  }

  doMutex = xSemaphoreCreateMutex();
  alarmSettingsChangeMutex = xSemaphoreCreateMutex();

  timer_doOffPulse = xTimerCreate("doPulse", pdMS_TO_TICKS(10), pdFALSE, (void *)1, &doOffPulse);
  assert(timer_doOffPulse);

  if(getHwVersion()>=1)
  {
    //tachoInit();
  }
}

bool getAlarm(uint8_t alarmNr)
{
  return bo_Alarm[alarmNr];
}

uint16_t getAlarm()
{
  uint16_t u16_lAlm=0;
  for(uint8_t i=0; i<10;i++)
  {
    u16_lAlm |= (bo_Alarm[i] << i);
  }
  return u16_lAlm;
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
  if(getHwVersion()==0)u8_mDoByte ^= (1 << 7);
  else digitalWrite(GPIO_LED1_HW1, !digitalRead(GPIO_LED1_HW1));

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

  //Tacho auswerten
  //rules_Tacho();

  //Bearbeiten der Alarme
  for(i=0; i<CNT_ALARMS; i++)
  {
    xSemaphoreTake(alarmSettingsChangeMutex, portMAX_DELAY);
    if(bo_mChangeAlarmSettings)
    {
      bo_Alarm[i]=false;
      bo_Alarm_old[i]=true;
      //setAlarm(i+1, false); //Alarm zurücksetzen
    }
    xSemaphoreGive(alarmSettingsChangeMutex);

    if(bo_Alarm[i]!=bo_Alarm_old[i]) //Flankenwechsel
    {
      //debugPrintf("Alarm (%i) Flanke, new State=%i",i,bo_Alarm[i]);
      bo_Alarm_old[i] = bo_Alarm[i];

      //Bei Statusänderung mqqt msg absetzen
      if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0,0,0))
      {
        mqttPublish(MQTT_TOPIC_ALARM, i+1, -1, -1, bo_Alarm[i]);
      }
      
      //Bearbeiten der 6 Relaisausgaenge + 1 OptoOut
      for(uint8_t o=0; o<CNT_DIGITALOUT; o++)
      {
        tmp=WebSettings::getInt(ID_PARAM_DO_AUSLOESUNG_BEI,0,o,0)-1;
        if(tmp==i)
        {
          if(bo_Alarm[i]==true)
          {     
            if(u8_DoVerzoegerungTimer[o]==0xFF) //Verzoegerungstimer nur starten wenn er noch nicht läuft
            {
              ESP_LOGI(TAG, "Set DO VerzoegerungTimer (DoNr=%i)", o);
              u8_DoVerzoegerungTimer[o] = WebSettings::getInt(ID_PARAM_DO_VERZOEGERUNG,0,o,0);
            }
          }
          else
          {
            uint8_t doPulseOrPermanent = WebSettings::getInt(ID_PARAM_DO_AUSLOESEVERHALTEN,0,o,0);
            if(doPulseOrPermanent==0) //Wenn Permanent
            {
              ESP_LOGI(TAG, "Alarm geht (AlarmNr=%i)", i);
              u8_mDoByte &= ~(1 << o); //bit loeschen
            }
          }
        }
      }
    }
  }
  xSemaphoreTake(alarmSettingsChangeMutex, portMAX_DELAY);
  bo_mChangeAlarmSettings=false;
  xSemaphoreGive(alarmSettingsChangeMutex);

  setDOs();
}


/* Wenn Einstellungen geändert werden, die auf einen Alarm auswirkung haben können, 
 * dann müssen die Alarme erst einmal zurück gesetzt werden
 */
void changeAlarmSettings()
{  
  xSemaphoreTake(alarmSettingsChangeMutex, portMAX_DELAY);
  bo_mChangeAlarmSettings=true;
  xSemaphoreGive(alarmSettingsChangeMutex);
}


void setDOs()
{
  //Bearbeiten der 6 Relaisausgaenge + 1 OptoOut
  for(uint8_t o=0; o<CNT_DIGITALOUT; o++)
  {
    if(u8_DoVerzoegerungTimer[o]==0)
    {
      u8_DoVerzoegerungTimer[o]=0xFF;

      ESP_LOGI(TAG, "Set DO (DoNr=%i)", o);
      u8_mDoByte |= (1 << o); //bit setzen

      uint8_t doPulseOrPermanent = WebSettings::getInt(ID_PARAM_DO_AUSLOESEVERHALTEN,0,o,0);
      if(doPulseOrPermanent==1) //Wenn Impuls
      {
        uint32_t pulseDuration = WebSettings::getInt(ID_PARAM_DO_IMPULSDAUER,0,o,0);
        ESP_LOGI(TAG, "DO Impuls DO=%i Dauer=%i", o, pulseDuration);
              
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
      if(!bo_lDiInvert)
      {
        setAlarm(u8_lAlarmNr,true); 
        //ESP_LOGD(TAG,"Alarm DI TRUE; Alarm %i", u8_lAlarmNr);
      }
      else
      {
        setAlarm(u8_lAlarmNr,false);
        //ESP_LOGD(TAG,"Alarm DI FALSE; Alarm %i", u8_lAlarmNr);
      }
    }
    else
    {
      if(!bo_lDiInvert)
      {
        setAlarm(u8_lAlarmNr,false);
        //ESP_LOGD(TAG,"Alarm DI FALSE; Alarm %i", u8_lAlarmNr);
      }
      else
      {
        setAlarm(u8_lAlarmNr,true);
        //ESP_LOGD(TAG,"Alarm DI TRUE; Alarm %i", u8_lAlarmNr);
      }
    }
  }
}

void tachoInit()
{
  u8_mTachoChannel=0;

  pinMode(TACHO_ADDR2, OUTPUT);
  digitalWrite(TACHO_ADDR2, LOW);

  u8_mDoByte &= ~(1 << TACHO_ADDR0); //bit loeschen
  u8_mDoByte &= ~(1 << TACHO_ADDR1); //bit loeschen

  FreqCountESP.begin(TACHO_GPIO, TACHO_MEAS_TIME);
  tachoSetMux(u8_mTachoChannel);
  FreqCountESP.runMeasure(); //Erste Messung starten
}

bool tachoRead(uint16_t &tachoRpm)
{
  bool ret=false;
  if (FreqCountESP.available())
  {
    tachoRpm = FreqCountESP.read() * (60000/TACHO_MEAS_TIME);
    ESP_LOGD(TAG,"chan=%i, RPM=%i",u8_mTachoChannel,tachoRpm);
    u8_mTachoChannel++;
    tachoSetMux(u8_mTachoChannel);
    FreqCountESP.runMeasure(); //nächste Messung starten
    ret=true;
  }
  return ret;
}

void tachoSetMux(uint8_t channel)
{
  if(channel>5)return;

  switch(channel)
  {
    case 0: //0 0 0
      u8_mDoByte &= ~(1 << TACHO_ADDR0); //bit loeschen
      u8_mDoByte &= ~(1 << TACHO_ADDR1); //bit loeschen
      digitalWrite(TACHO_ADDR2, LOW);
      break;
    case 1: //1 0 0
      u8_mDoByte |=  (1 << TACHO_ADDR0); //bit setzen
      u8_mDoByte &= ~(1 << TACHO_ADDR1); //bit loeschen
      digitalWrite(TACHO_ADDR2, LOW);
      break;
    case 2: //0 1 0
      u8_mDoByte &= ~(1 << TACHO_ADDR0); //bit loeschen
      u8_mDoByte |=  (1 << TACHO_ADDR1); //bit setzen
      digitalWrite(TACHO_ADDR2, LOW);
      break;
    case 3: //1 1 0
      u8_mDoByte |= (1 << TACHO_ADDR0); //bit setzen
      u8_mDoByte |= (1 << TACHO_ADDR1); //bit setzen
      digitalWrite(TACHO_ADDR2, LOW);
      break;
    case 4: //0 0 1
      u8_mDoByte &= ~(1 << TACHO_ADDR0); //bit loeschen
      u8_mDoByte &= ~(1 << TACHO_ADDR1); //bit loeschen
      digitalWrite(TACHO_ADDR2, HIGH);
      break;
    case 5: //1 0 1
      u8_mDoByte |=  (1 << TACHO_ADDR0); //bit setzen
      u8_mDoByte &= ~(1 << TACHO_ADDR1); //bit loeschen
      digitalWrite(TACHO_ADDR2, HIGH);
      break;
    default:
      break;
  }

  xSemaphoreTake(doMutex, portMAX_DELAY);
  setDoData(u8_mDoByte);
  xSemaphoreGive(doMutex);
}

void rules_Tacho()
{
  if(getHwVersion()==0) return;
  
  uint16_t u16_lTachoRpm=0;
  if(tachoRead(u16_lTachoRpm))
  {
    //ToDo: Settings laden; z.B. Alarmregeln/Lüfter
    //uint16_t u16_lTachoRpmUpperLimit;  //u8_mTachoChannel
    uint16_t u16_lTachoRpmLowerLimit;  //u8_mTachoChannel
    uint8_t u8_lTachAlarmNr;           //u16_lTachoRpm

    if(u16_lTachoRpm<u16_lTachoRpmLowerLimit /*|| u16_lTachoRpm>u16_lTachoRpmUpperLimit*/)
    {
      setAlarm(u8_lTachAlarmNr,true);
    }
    else
    {
      setAlarm(u8_lTachAlarmNr,false);
    }
  }
}


void rules_Bms()
{
  uint8_t i,tmp,u8_lAlarmruleBmsNr;

  for(i=0; i<CNT_BT_ALARMS_RULES; i++)
  {
    u8_lAlarmruleBmsNr=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_BMS_SELECT,0,i,0);

    if(u8_lAlarmruleBmsNr<=0) continue;
    u8_lAlarmruleBmsNr--;

    //bool b_lBmsOnline=true;
    //if((millis()-getBmsLastDataMillis(i))>10000) b_lBmsOnline=false; //Wenn 2000 ms keine Daten vom BMS kamen, dann ist es offline
    //debugPrintf("i=%i, b_lBmsOnline=%i",i,b_lBmsOnline);

    //Ist überhaupt ein Device parametriert?
    if((u8_lAlarmruleBmsNr<BT_DEVICES_COUNT && WebSettings::getInt(ID_PARAM_SS_BTDEV,0,u8_lAlarmruleBmsNr,0)>0 && !WebSettings::getString(ID_PARAM_SS_BTDEVMAC,0,u8_lAlarmruleBmsNr,0).equals("")) ||
      (u8_lAlarmruleBmsNr>=BT_DEVICES_COUNT && WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,u8_lAlarmruleBmsNr-BT_DEVICES_COUNT,0)!=0 ) )
    {
      //Wenn Alram für das Device aktiv ist
      if(WebSettings::getBool(ID_PARAM_ALARM_BTDEV_ALARM_ON,0,i,0)) 
      {      
        //Alarm wenn keine Daten mehr vom BT-Device kommen
        if((millis()-getBmsLastDataMillis(u8_lAlarmruleBmsNr))>((uint32_t)WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT,0,i,0)*1000))
        {
          //Alarm
          //debugPrintf("BT Alarm (%i)",u8_lAlarmruleBmsNr);
          tmp=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_AKTION,0,i,0);
          setAlarm(tmp,true);
          //ESP_LOGD(TAG,"Alarm BMS no data - TRUE; Alarm %i", tmp);
        }
        else
        {
          tmp=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_AKTION,0,i,0);
          setAlarm(tmp,false);
          //ESP_LOGD(TAG,"Alarm BMS no data - FALSE; Alarm %i", tmp);
        }
      }

      //Überwachung Zellspannung
      //debugPrintf("(i=%i) Zell Spg. Outside b_lBmsOnline=%i, enable=%i",i, b_lBmsOnline,WebSettings::getBool(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON,0,i,0));
      if(/*b_lBmsOnline==true &&*/ WebSettings::getBool(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON,0,i,0))
      {
      //debugPrintf("Zell Spg. Outside cellCnt=%i",WebSettings::getInt(ID_PARAM_ALARM_BT_CNT_CELL_CTRL,0,i,0));
        for(uint8_t cc=0; cc<WebSettings::getInt(ID_PARAM_ALARM_BT_CNT_CELL_CTRL,0,i,0); cc++)
        {
          //debugPrintf("i=%i, cc=%i, cellspg=%i, min=%i, max=%i",i,cc,getBmsCellVoltage(i,cc),WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MIN,0,i,0),WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MAX,0,i,0));
          if(getBmsCellVoltage(u8_lAlarmruleBmsNr,cc) < WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MIN,0,i,0) || getBmsCellVoltage(u8_lAlarmruleBmsNr,cc) > WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MAX,0,i,0))
          {
            //Alarm
            tmp=WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,0,i,0);
            setAlarm(tmp,true);
            //ESP_LOGD(TAG, "Zell Spg. Outside (%i) alarmNr=%i", i,tmp);
            break; //Sobald eine Zelle Alarm meldet kann abgebrochen werden
          }
          else
          {
            tmp=WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,0,i,0);
            setAlarm(tmp,false);
            //ESP_LOGD(TAG,"Alarm BMS Zell Spg. - FALSE; Alarm %i", tmp);
          }
        }
      }

      //Überwachung Gesamtspannung
      uint8_t u8_lAlarm = WebSettings::getInt(ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION,0,i,0); //BleHandler::bmsIsConnect(i)
      if(/*b_lBmsOnline==true &&*/ u8_lAlarm>0) 
      {
        if(getBmsTotalVoltage(u8_lAlarmruleBmsNr) < WebSettings::getInt(ID_PARAM_ALARM_BT_GESAMT_SPG_MIN,0,i,0) || getBmsTotalVoltage(u8_lAlarmruleBmsNr) > WebSettings::getInt(ID_PARAM_ALARM_BT_GESAMT_SPG_MAX,0,i,0))
        {
          //Alarm
          setAlarm(u8_lAlarm,true);
          //ESP_LOGD(TAG,"Alarm BMS Gesamtspannung - TRUE; Alarm %i", u8_lAlarm);
        }
        else
        {
          setAlarm(u8_lAlarm,false);
          //ESP_LOGD(TAG,"Alarm BMS Gesamtspannung - FALSE; Alarm %i", u8_lAlarm);
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
      //ESP_LOGD(TAG,"Alarm BMS Temperatur - %i; Alarm %i",bo_lAlarm,alarmNr);
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
      //debugPrintf("DO off - Impuls(%i)",i);

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