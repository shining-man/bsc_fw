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

uint16_t u16_DoPulsOffCounter[CNT_DIGITALOUT];
uint8_t u8_DoVerzoegerungTimer[CNT_DIGITALOUT];

bool bo_alarmActivate[CNT_ALARMS]; //Merker ob ein Alarm in diesem 'run' gesetzt wurde
bool bo_timerPulseOffIsRunning;

bool bo_mChangeAlarmSettings;

uint8_t u8_mDoByte;
uint8_t u8_mTachoChannel;

//Hysterese
uint32_t u32_hystereseTotalVoltageMin=0;
uint32_t u32_hystereseTotalVoltageMax=0;

void rules_Bms();
void rules_Temperatur();
void rules_CanInverter();
void rules_Tacho();
bool temperatur_maxWertUeberwachung(uint8_t);
bool temperatur_maxWertUeberwachungReferenz(uint8_t);
bool temperatur_DifferenzUeberwachung(uint8_t);
void temperatur_senorsErrors();
void runDigitalAusgaenge();
void doOffPulse(TimerHandle_t xTimer);
void getDIs();
void setDOs();
void tachoInit();
bool tachoRead(uint16_t &tachoRpm);
void tachoSetMux(uint8_t channel);
void setAlarmToBtDevices(uint8_t u8_AlarmNr, boolean bo_Alarm);


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
    if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0))
    {
      mqttPublish(MQTT_TOPIC_ALARM, i+1, -1, -1, bo_Alarm[i]);
      
    }  
  }

  for(uint8_t i=0;i<CNT_DIGITALOUT;i++)
  {
    u16_DoPulsOffCounter[i] = 0;
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

bool isTriggerSelected(uint16_t paramId, uint8_t groupNr, uint8_t dataType, uint8_t triggerNr)
{
  uint16_t u16_lValue = WebSettings::getInt(paramId,groupNr,dataType);
  if(u16_lValue>0)
  {
    if((u16_lValue>>triggerNr)&0x1)
    {
        return true;
    }

  }
  return false;
}

bool isTriggerActive(uint16_t paramId, uint8_t groupNr, uint8_t dataType)
{
  uint16_t u16_lValue = WebSettings::getInt(paramId,groupNr,dataType);
  if(u16_lValue>0)
  {
    for(uint16_t i=0;i<CNT_ALARMS;i++)
    {
      if((u16_lValue>>i)&0x1)
      {
        if(getAlarm(i))
        {
          return true;
        }
      }
    }
  }
  return false;
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
  if(alarmNr==0)return;

  alarmNr--;
  if(bo_lAlarm)
  {
    //BSC_LOGD(TAG,"setAlarm: alarmNr=%i TRUE",alarmNr);
    bo_Alarm[alarmNr]=true;
    bo_alarmActivate[alarmNr]=true;
  }
  else if(bo_alarmActivate[alarmNr]!=true)
  {
    //BSC_LOGD(TAG,"setAlarm: alarmNr=%i FALSE",alarmNr);
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
  uint8_t i;

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
      BSC_LOGD(TAG,"Reset Trigger (Nr=%i) - %s",i+1,WebSettings::getStringFlash(ID_PARAM_TRIGGER_NAMES,i).c_str());
      bo_Alarm[i]=false;
      bo_Alarm_old[i]=true;
      //setAlarm(i+1, false); //Alarm zurücksetzen
    }
    xSemaphoreGive(alarmSettingsChangeMutex);

    if(bo_Alarm[i]!=bo_Alarm_old[i]) //Flankenwechsel
    {
      BSC_LOGI(TAG, "Trigger %i, value=%i - %s",i+1,bo_Alarm[i],WebSettings::getStringFlash(ID_PARAM_TRIGGER_NAMES,i).c_str());
      bo_Alarm_old[i] = bo_Alarm[i];

      //Bei Statusänderung mqqt msg absetzen
      if(WebSettings::getBool(ID_PARAM_MQTT_SERVER_ENABLE,0))
      {
        BSC_LOGD(TAG, "Trigger %i: %i - %s",i+1,bo_Alarm[i],WebSettings::getStringFlash(ID_PARAM_TRIGGER_NAMES,i).c_str());
        mqttPublish(MQTT_TOPIC_ALARM, i+1, -1, -1, bo_Alarm[i]);
      }
      
      //Bearbeiten der 6 Relaisausgaenge
      uint8_t u8_lTriggerNrDo=0;
      for(uint8_t b=0; b<CNT_DIGITALOUT; b++)
      {
        if(isTriggerSelected(ID_PARAM_DO_AUSLOESUNG_BEI,b,DT_ID_PARAM_DO_AUSLOESUNG_BEI,i))
        {
          if(bo_Alarm[i]==true)
          {     
            if(u8_DoVerzoegerungTimer[b]==0xFF) //Verzoegerungstimer nur starten wenn er noch nicht läuft
            {
              BSC_LOGD(TAG, "Set DO VerzoegerungTimer (DoNr=%i)", b);
              u8_DoVerzoegerungTimer[b] = WebSettings::getInt(ID_PARAM_DO_VERZOEGERUNG,b,DT_ID_PARAM_DO_VERZOEGERUNG);
            }
          }
          else
          {
            uint8_t doPulseOrPermanent = WebSettings::getInt(ID_PARAM_DO_AUSLOESEVERHALTEN,b,DT_ID_PARAM_DO_AUSLOESEVERHALTEN);
            if(doPulseOrPermanent==0) //Wenn Permanent
            {
              BSC_LOGD(TAG, "Trigger geht (AlarmNr=%i) - %s", i+1,WebSettings::getStringFlash(ID_PARAM_TRIGGER_NAMES,i).c_str());
              u8_mDoByte &= ~(1 << b); //bit loeschen
            }
          }
        }
      }

      //Alarm an BT Device weiterleiten
      setAlarmToBtDevices(i, bo_Alarm[i]);
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
  for(uint8_t b=0; b<CNT_DIGITALOUT; b++)
  {
    if(u8_DoVerzoegerungTimer[b]==0)
    {
      u8_DoVerzoegerungTimer[b]=0xFF;

      BSC_LOGD(TAG, "Set DO (DoNr=%i)", b);
      u8_mDoByte |= (1 << b); //bit setzen

      uint8_t doPulseOrPermanent = WebSettings::getInt(ID_PARAM_DO_AUSLOESEVERHALTEN,b,DT_ID_PARAM_DO_AUSLOESEVERHALTEN);
      if(doPulseOrPermanent==1) //Wenn Impuls
      {
        uint16_t pulseDuration = (uint16_t)WebSettings::getInt(ID_PARAM_DO_IMPULSDAUER,b,DT_ID_PARAM_DO_IMPULSDAUER);
        BSC_LOGD(TAG, "DO Impuls DO=%i Dauer=%i", b, pulseDuration);
              
        xSemaphoreTake(doMutex, portMAX_DELAY);
        u16_DoPulsOffCounter[b] = (pulseDuration/10);
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
      if(u8_DoVerzoegerungTimer[b]!=0xFF) u8_DoVerzoegerungTimer[b]--;
    }
  }


  //Setze Ausgänge, lese Eingänge
  xSemaphoreTake(doMutex, portMAX_DELAY);
  setDoData(u8_mDoByte);
  xSemaphoreGive(doMutex);
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
    if(u16_DoPulsOffCounter[i] > 1)
    {
      u16_DoPulsOffCounter[i]--;
      restartTimer = true;
    }
    else if(u16_DoPulsOffCounter[i] == 1)
    {
      BSC_LOGD(TAG, "DO off - Impuls(%i)",i);

      u16_DoPulsOffCounter[i]=0;

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


void getDIs()
{
  //Lese der Eingänge
  xSemaphoreTake(doMutex, portMAX_DELAY);
  uint8_t u8_lDiData = dioRwInOut();
  xSemaphoreGive(doMutex);

  //Bearbeiten der 4 Digitaleingänge
  for(uint8_t i=0; i<CNT_DIGITALIN; i++)
  {
    uint8_t u8_lAlarmNr  = WebSettings::getInt(ID_PARAM_DI_ALARM_NR,i,DT_ID_PARAM_DI_ALARM_NR);
    bool    bo_lDiInvert = WebSettings::getBool(ID_PARAM_DI_INVERTIERT,i);

    if(u8_lAlarmNr==0) continue;

    if((u8_lDiData & (1<<i)) == (1<<i))
    {
      if(!bo_lDiInvert)
      {
        setAlarm(u8_lAlarmNr,true); 
        //BSC_LOGD(TAG,"Alarm (a) DI TRUE; Alarm %i", u8_lAlarmNr);
      }
      else
      {
        setAlarm(u8_lAlarmNr,false);
        //BSC_LOGD(TAG,"Alarm (b) DI FALSE; Alarm %i", u8_lAlarmNr);
      }
    }
    else
    {
      if(!bo_lDiInvert)
      {
        setAlarm(u8_lAlarmNr,false);
        //BSC_LOGD(TAG,"Alarm (c) DI FALSE; Alarm %i", u8_lAlarmNr);
      }
      else
      {
        setAlarm(u8_lAlarmNr,true);
        //BSC_LOGD(TAG,"Alarm (d) DI TRUE; Alarm %i", u8_lAlarmNr);
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
    BSC_LOGD(TAG,"chan=%i, RPM=%i",u8_mTachoChannel,tachoRpm);
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
  uint8_t u8_bmsSerial2=(uint8_t)WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,2,DT_ID_PARAM_SERIAL_CONNECT_DEVICE);
  uint8_t u8_numberOfBmsOnSerial2=(uint8_t)WebSettings::getInt(ID_PARAM_SERIAL2_CONNECT_TO_ID,0,DT_ID_PARAM_SERIAL2_CONNECT_TO_ID);

  for(i=0; i<CNT_BT_ALARMS_RULES; i++)
  {
    u8_lAlarmruleBmsNr=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_BMS_SELECT,i,DT_ID_PARAM_ALARM_BTDEV_BMS_SELECT);
    if(u8_lAlarmruleBmsNr==127) continue; //'AUS'
    //BSC_LOGE(TAG,"u8_lAlarmruleBmsNr=%i, u8_numberOfBmsOnSerial2=%i, u8_bmsSerial2=%i",u8_lAlarmruleBmsNr,u8_numberOfBmsOnSerial2,u8_bmsSerial2);
    //Ist überhaupt ein Device parametriert?
    if((u8_lAlarmruleBmsNr<BT_DEVICES_COUNT && WebSettings::getInt(ID_PARAM_SS_BTDEV,u8_lAlarmruleBmsNr,DT_ID_PARAM_SS_BTDEV)>0 && !WebSettings::getString(ID_PARAM_SS_BTDEVMAC,u8_lAlarmruleBmsNr).equals("")) ||
      (u8_lAlarmruleBmsNr>=BT_DEVICES_COUNT && WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,u8_lAlarmruleBmsNr-BT_DEVICES_COUNT,DT_ID_PARAM_SERIAL_CONNECT_DEVICE)!=0 ) ||
      (u8_lAlarmruleBmsNr>(BT_DEVICES_COUNT+2) && u8_lAlarmruleBmsNr<(u8_numberOfBmsOnSerial2+BT_DEVICES_COUNT+2) && (u8_bmsSerial2==ID_SERIAL_DEVICE_SEPLOSBMS || u8_bmsSerial2==ID_SERIAL_DEVICE_SYLCINBMS)) )
    {
      //Wenn Alram für das Device aktiv ist
      if(WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BTDEV_ALARM_AKTION)>0) 
      {      
        //Alarm wenn keine Daten mehr vom BT-Device kommen
        if((millis()-getBmsLastDataMillis(u8_lAlarmruleBmsNr))>((uint32_t)WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT,i,DT_ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT)*1000))
        {
          //Alarm
          //debugPrintf("BT Alarm (%i)",u8_lAlarmruleBmsNr);
          tmp=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BTDEV_ALARM_AKTION);
          setAlarm(tmp,true);
          //BSC_LOGD(TAG,"Alarm BMS no data - TRUE; Alarm %i", tmp);
        }
        else
        {
          tmp=WebSettings::getInt(ID_PARAM_ALARM_BTDEV_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BTDEV_ALARM_AKTION);
          setAlarm(tmp,false);
          //BSC_LOGD(TAG,"Alarm BMS no data - FALSE; Alarm %i", tmp);
        }
      }

      //Überwachung Zellspannung
      //BSC_LOGD(TAG, "(i=%i) Zell Spg. Outside, enable=%i",i, WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION));
      if(WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION)>0)
      {
      //debugPrintf("Zell Spg. Outside cellCnt=%i",WebSettings::getInt(ID_PARAM_ALARM_BT_CNT_CELL_CTRL,0,i,0));
        for(uint8_t cc=0; cc<WebSettings::getInt(ID_PARAM_ALARM_BT_CNT_CELL_CTRL,i,DT_ID_PARAM_ALARM_BT_CNT_CELL_CTRL); cc++)
        {
          //debugPrintf("i=%i, cc=%i, cellspg=%i, min=%i, max=%i",i,cc,getBmsCellVoltage(i,cc),WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MIN,0,i,0),WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MAX,0,i,0));
          if(getBmsCellVoltage(u8_lAlarmruleBmsNr,cc) < WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MIN,i,DT_ID_PARAM_ALARM_BT_CELL_SPG_MIN) 
            || getBmsCellVoltage(u8_lAlarmruleBmsNr,cc) > WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_MAX,i,DT_ID_PARAM_ALARM_BT_CELL_SPG_MAX))
          {
            //Alarm
            tmp=WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION);
            setAlarm(tmp,true);
            //BSC_LOGD(TAG, "Zell Spg. Outside (%i) alarmNr=%i", i,tmp);
            break; //Sobald eine Zelle Alarm meldet kann abgebrochen werden
          }
          else
          {
            tmp=WebSettings::getInt(ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION);
            setAlarm(tmp,false);
            //BSC_LOGD(TAG,"Alarm BMS Zell Spg. - FALSE; Alarm %i", tmp);
          }
        }
      }

      //Überwachung Gesamtspannung
      uint8_t u8_lAlarm = WebSettings::getInt(ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION,i,DT_ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION);
      if(u8_lAlarm>0) 
      {
        //Total voltage Min
        if(getBmsTotalVoltage(u8_lAlarmruleBmsNr) < WebSettings::getFloat(ID_PARAM_ALARM_BT_GESAMT_SPG_MIN,i))
        {
          //Alarm
          bitSet(u32_hystereseTotalVoltageMin,i);
          bitClear(u32_hystereseTotalVoltageMax,i);
          setAlarm(u8_lAlarm,true);
          //BSC_LOGD(TAG,"Alarm BMS Gesamtspannung - TRUE; Alarm %i", u8_lAlarm);
        }
        //Total voltage Max
        else if(getBmsTotalVoltage(u8_lAlarmruleBmsNr) > WebSettings::getFloat(ID_PARAM_ALARM_BT_GESAMT_SPG_MAX,i))
        {
          //Alarm
          bitSet(u32_hystereseTotalVoltageMax,i);
          bitClear(u32_hystereseTotalVoltageMin,i);
          setAlarm(u8_lAlarm,true);
          //BSC_LOGD(TAG,"Alarm BMS Gesamtspannung - TRUE; Alarm %i", u8_lAlarm);
        }
        else
        {
          float fl_totalVoltageHysterese = WebSettings::getFloat(ID_PARAM_ALARM_BT_GESAMT_SPG_HYSTERESE,i);
          if(isBitSet(u32_hystereseTotalVoltageMin,i))
          {
            if(getBmsTotalVoltage(u8_lAlarmruleBmsNr) > (WebSettings::getFloat(ID_PARAM_ALARM_BT_GESAMT_SPG_MIN,i)+fl_totalVoltageHysterese))
            {
              bitClear(u32_hystereseTotalVoltageMin,i);
              setAlarm(u8_lAlarm,false);
            }
            else setAlarm(u8_lAlarm,true);
          }
          else if(isBitSet(u32_hystereseTotalVoltageMax,i))
          {
            if(getBmsTotalVoltage(u8_lAlarmruleBmsNr) < (WebSettings::getFloat(ID_PARAM_ALARM_BT_GESAMT_SPG_MAX,i)-fl_totalVoltageHysterese))
            {
              bitClear(u32_hystereseTotalVoltageMax,i);
              setAlarm(u8_lAlarm,false);
            }
            else setAlarm(u8_lAlarm,true);
          }
          /*else
          {
            setAlarm(u8_lAlarm,false);
            //BSC_LOGD(TAG,"Alarm BMS Gesamtspannung - FALSE; Alarm %i", u8_lAlarm);
          }*/
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
    sensorVon = WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_VON);
    sensorBis = WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_BIS);
    
    if(sensorVon>=0 && sensorBis>=0 && sensorBis>=sensorVon)
    {
      switch (WebSettings::getInt(ID_PARAM_TEMP_ALARM_UEBERWACH_FUNKTION,i,DT_ID_PARAM_TEMP_ALARM_UEBERWACH_FUNKTION))
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
          break;

        default:
          break;
      }

      alarmNr=WebSettings::getInt(ID_PARAM_TEMP_ALARM_AKTION,i,DT_ID_PARAM_TEMP_ALARM_AKTION);
      setAlarm(alarmNr,bo_lAlarm);
      //BSC_LOGD(TAG,"Alarm BMS Temperatur - %i; Alarm %i",bo_lAlarm,alarmNr);
    }
  }

  temperatur_senorsErrors();
}


void rules_CanInverter()
{
  //Ladeleistung bei Alarm auf 0 Regeln
  if(isTriggerActive(ID_PARAM_BMS_LADELEISTUNG_AUF_NULL,0,DT_ID_PARAM_BMS_LADELEISTUNG_AUF_NULL)) canSetChargeCurrentToZero(true);
  else canSetChargeCurrentToZero(false);

  //Entladeleistung bei Alarm auf 0 Regeln
  if(isTriggerActive(ID_PARAM_BMS_ENTLADELEISTUNG_AUF_NULL,0,DT_ID_PARAM_BMS_ENTLADELEISTUNG_AUF_NULL)) canSetDischargeCurrentToZero(true);
  else canSetDischargeCurrentToZero(false);

  //SOC bei Alarm auf 100 stellen
  if(isTriggerActive(ID_PARAM_BMS_SOC_AUF_FULL,0,DT_ID_PARAM_BMS_SOC_AUF_FULL)) canSetSocToFull(true);
  else canSetSocToFull(false);
}

uint16_t u16_mMerkerTemperaturTrigger=0;
void setMerker(uint8_t merkerNr, bool val)
{
  if(val) u16_mMerkerTemperaturTrigger |= (1<<merkerNr);
  else u16_mMerkerTemperaturTrigger &= ~(1<<merkerNr); //bit loeschen
}

bool isMerker(uint8_t merkerNr)
{
  if((u16_mMerkerTemperaturTrigger>>merkerNr)&0x01) return true;
  else return false;
}

/*
 * Hohlt die Temperaturen von der entsprechenden Quelle (BMS, onewire) für die Alarmrules
 */
float getTemp_TempAlarmrule(uint8_t u8_ruleNr, uint8_t u8_sensorNr)
{
  uint8_t u8_lRuleTempQuelle=WebSettings::getInt(ID_PARAM_TEMP_ALARM_TEMP_QUELLE,u8_ruleNr,DT_ID_PARAM_TEMP_ALARM_TEMP_QUELLE);

  if(u8_lRuleTempQuelle==1) //BMS
  {
    uint8_t u8_lRuleBmsQuelle=WebSettings::getInt(ID_PARAM_TEMP_ALARM_BMS_QUELLE,u8_ruleNr,DT_ID_PARAM_TEMP_ALARM_BMS_QUELLE);
    return getBmsTempature(u8_lRuleBmsQuelle,u8_sensorNr);
  }
  else if(u8_lRuleTempQuelle==2) //Onewire
  {
    return owGetTemp(u8_sensorNr);
  }

  BSC_LOGE(TAG,"Error while reading the temperatures for the alarm rules");
  return 0; //Fehler
}

bool temperatur_maxWertUeberwachung(uint8_t i)
{
  bool bo_lHystMerker=true;
  float maxTemp = WebSettings::getFloat(ID_PARAM_TEMP_ALARM_WERT1,i);
  float hysterese = WebSettings::getFloat(ID_PARAM_TEMP_ALARM_WERT2,i);

  if(maxTemp<=0) return false; //Wenn keine Max.Temp. angegeben ist

  for(uint8_t n=WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_VON);n<WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_BIS)+1;n++)
  {
    if(getTemp_TempAlarmrule(i,n)>maxTemp && getTemp_TempAlarmrule(i,n)!=TEMP_IF_SENSOR_READ_ERROR)
    {
      setMerker(i,true);
      return true;
    }
    else if(isMerker(i) && getTemp_TempAlarmrule(i,n)>maxTemp-hysterese && getTemp_TempAlarmrule(i,n)!=TEMP_IF_SENSOR_READ_ERROR)
    {
      bo_lHystMerker=false;
    }
  }

  if(bo_lHystMerker)
  {
    setMerker(i,false); //Merker löschen, wenn alle Werte kleiner der Hysterese Waren
    return false;
  }
  else return true;
}


bool temperatur_maxWertUeberwachungReferenz(uint8_t i)
{
  bool bo_lHystMerker=true;
  float aktTemp = 0;
  float tempOffset = WebSettings::getFloat(ID_PARAM_TEMP_ALARM_WERT1,i);
  float hysterese = WebSettings::getFloat(ID_PARAM_TEMP_ALARM_WERT2,i);

  aktTemp = tempOffset + getTemp_TempAlarmrule(i,WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VERGLEICH,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_VERGLEICH));
  for(uint8_t n=WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_VON);n<WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_BIS)+1;n++)
  {
    if(getTemp_TempAlarmrule(i,n)>aktTemp && getTemp_TempAlarmrule(i,n)!=TEMP_IF_SENSOR_READ_ERROR)
    {
      setMerker(i,true);
      return true;
    }
    else if(isMerker(i) && getTemp_TempAlarmrule(i,n)>aktTemp-hysterese && getTemp_TempAlarmrule(i,n)!=TEMP_IF_SENSOR_READ_ERROR)
    {
      bo_lHystMerker=false;
    }
  }

  if(bo_lHystMerker)
  {
    setMerker(i,false); //Merker löschen, wenn alle Werte kleiner der Hysterese Waren
    return false;
  }
  else return true;
}


bool temperatur_DifferenzUeberwachung(uint8_t i)
{
  bool bo_lHystMerker=true;
  float f_lTempMin = 0xFF;
  float f_lTempMax = 0;

  float f_lMaxTempDiff = WebSettings::getFloat(ID_PARAM_TEMP_ALARM_WERT1,i);
  float hysterese = WebSettings::getFloat(ID_PARAM_TEMP_ALARM_WERT2,i);
  
  for(uint8_t n=WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_VON,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_VON);n<WebSettings::getInt(ID_PARAM_TEMP_ALARM_SENSOR_BIS,i,DT_ID_PARAM_TEMP_ALARM_SENSOR_BIS)+1;n++)
  {
    if(getTemp_TempAlarmrule(i,n)>f_lTempMax) f_lTempMax=getTemp_TempAlarmrule(i,n);
    if(getTemp_TempAlarmrule(i,n)<f_lTempMin) f_lTempMin=getTemp_TempAlarmrule(i,n);
  }

  if(f_lTempMax-f_lTempMin>=f_lMaxTempDiff)
  {
    setMerker(i,true);
    return true;
  }
  else if(isMerker(i) && f_lTempMax-f_lTempMin>=f_lMaxTempDiff-hysterese)
  {
    bo_lHystMerker=false;
  }

  if(bo_lHystMerker)
  {
    setMerker(i,false); //Merker löschen, wenn alle Werte kleiner der Hysterese Waren
    return false;
  }
  else return true;
}


void temperatur_senorsErrors()
{
  boolean bo_lAlarm = false;
  uint8_t u8_lOwTempSensorErrors = owGetAllSensorError();

  uint8_t u8_lSensorNoValueTime = WebSettings::getInt(ID_PARAM_TEMP_SENSOR_TIMEOUT_TIME,0,DT_ID_PARAM_TEMP_SENSOR_TIMEOUT_TIME); //Time in secounds
  uint8_t u8_lTrigger = WebSettings::getInt(ID_PARAM_TEMP_SENSOR_TIMEOUT_TRIGGER,0,DT_ID_PARAM_TEMP_SENSOR_TIMEOUT_TRIGGER);

  if(u8_lTrigger>0) //If enabled
  {
    if(u8_lOwTempSensorErrors>u8_lSensorNoValueTime) bo_lAlarm=true;
    setAlarm(u8_lTrigger,bo_lAlarm);
  }
}


//Alarm an BT Device weiterleiten
void setAlarmToBtDevices(uint8_t u8_AlarmNr, boolean bo_Alarm)
{
  for(uint8_t d=0;d<BT_DEVICES_COUNT;d++)
  {
    uint8_t u8_lTriggerNr = WebSettings::getIntFlash(ID_PARAM_NEEY_BALANCER_ON,0,DT_ID_PARAM_NEEY_BALANCER_ON);
    if(u8_AlarmNr==u8_lTriggerNr) BleHandler::setBalancerState(d,bo_Alarm);
  }
}
