// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Canbus.h"
#include "WebSettings.h"
#include "defines.h"
#include "BmsData.h"
#include "mqtt_t.h"
#include <CAN.h>
#include "log.h"

static const char *TAG = "CAN";

void sendBmsCanMessages();
void sendCanMsg_370_371();
void sendCanMsg_35e();
void sendCanMsg_351();
void sendCanMsg_355();
void sendCanMsg_356();
void sendCanMsg_359();
void sendCanMsg_35a();
void sendCanMsg_373();
void sendCanMsg(uint32_t identifier, uint8_t *buffer, uint8_t length);


static SemaphoreHandle_t mInverterDataMutex = NULL;
static struct inverterData_s inverterData;

char hostname[16] = {'B','S','C',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

uint8_t u8_mBmsDatasource;
uint8_t u8_mBmsDatasourceAdd;
uint8_t u8_mSelCanInverter;

bool alarmSetChargeCurrentToZero;
bool alarmSetDischargeCurrentToZero;
bool alarmSetSocToFull;

int16_t  i16_mMaxChargeCurrent=0;
int16_t  i16_mAktualChargeCurrentSoll;
uint8_t  u8_mTimerCalcMaxChareCurrent=0;
uint16_t u16_mTimerChargeOff=0;
uint16_t u16_mSperrzeitChargeOff;

//Variablen: Wenn Zellspannung kleiner x mV wird SoC auf x% setzen
enum SM_SocZellspgStates {STATE_MINCELLSPG_SOC_WAIT_OF_MIN=0, STATE_MINCELLSPG_SOC_BELOW_MIN, STATE_MINCELLSPG_SOC_LOCKTIMER};
uint8_t  u8_mSocZellspannungState;
uint16_t u16_mSocZellspannungSperrzeitTimer;

struct data351
{
  uint16_t chargevoltagelimit;   // CVL
  int16_t  maxchargecurrent;     // CCL
  int16_t  maxdischargecurrent;  // DCL
  uint16_t dischargevoltage;     // DVL
};

struct data355
{
  uint16_t soc; //state of charge
  uint16_t soh; //state of health
};

struct data356
{
  int16_t voltage;
  int16_t current;
  int16_t temperature;
};

struct data35a
{
  uint8_t u8_b0;
  uint8_t u8_b1;
  uint8_t u8_b2;
  uint8_t u8_b3;
  uint8_t u8_b4;
  uint8_t u8_b5;
  uint8_t u8_b6;
  uint8_t u8_b7;
};

struct data373
{
  uint16_t minCellColtage;
  uint16_t maxCellVoltage;
  uint16_t minCellTemp;
  uint16_t maxCellTemp;
};

void canSetup()
{
  mInverterDataMutex = xSemaphoreCreateMutex();

  u8_mBmsDatasource=0;
  alarmSetChargeCurrentToZero=false;
  alarmSetDischargeCurrentToZero=false;
  alarmSetSocToFull=false;

  u8_mTimerCalcMaxChareCurrent=0;
  u16_mTimerChargeOff=0;

  u16_mSperrzeitChargeOff=WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_SPERRZEIT,0,0,0); //Sperrzeit ab wann wieder geladen werden darf wenn auf 0 gereglt wird

  u16_mSocZellspannungSperrzeitTimer=0;
  u8_mSocZellspannungState=STATE_MINCELLSPG_SOC_WAIT_OF_MIN;

  loadCanSettings();

  // start the CAN bus at 500 kbps
  CAN.setPins(5,4);
  if (!CAN.begin(500000))
  {
    ESP_LOGI(TAG,"Init CAN failed!");
  }
  else
  {
    ESP_LOGI(TAG,"Init CAN ok");
  } 
}

void inverterDataSemaphoreTake()
{
  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
}

void inverterDataSemaphoreGive()
{
  xSemaphoreGive(mInverterDataMutex);
}

struct inverterData_s * getInverterData()
{
  return &inverterData;
}

void loadCanSettings()
{
  u8_mBmsDatasource = WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE,0,0,0);
  u8_mSelCanInverter = WebSettings::getInt(ID_PARAM_SS_CAN,0,0,0);
  u8_mBmsDatasourceAdd=0;
  
  if(WebSettings::getBool(ID_PARAM_BMS_CAN_DATASOURCE_SS1,0,0,0) && WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,0,0)!=0)
  {
    if(u8_mBmsDatasource!=BT_DEVICES_COUNT) u8_mBmsDatasourceAdd=1;
  }

  if(WebSettings::getBool(ID_PARAM_BMS_CAN_DATASOURCE_SS2,0,0,0) && WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,1,0)!=0)
  {
    if(u8_mBmsDatasource!=BT_DEVICES_COUNT+1) u8_mBmsDatasourceAdd |= (1<<1);
  }  

  if(WebSettings::getBool(ID_PARAM_BMS_CAN_DATASOURCE_SS3,0,0,0) && WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,0,2,0)!=0)
  {
    if(u8_mBmsDatasource!=BT_DEVICES_COUNT+2) u8_mBmsDatasourceAdd |= (1<<2);
  } 

  ESP_LOGI(TAG,"loadCanSettings(): u8_mBmsDatasource=%i",u8_mBmsDatasource);
}

//Ladeleistung auf 0 einstellen
void canSetChargeCurrentToZero(bool val)
{
  alarmSetChargeCurrentToZero = val;
}

//Entladeleistung auf 0 einstellen
void canSetDischargeCurrentToZero(bool val)
{
  alarmSetDischargeCurrentToZero = val;
}

//SOC auf 100 einstellen
void canSetSocToFull(bool val)
{
  alarmSetSocToFull = val;
}



uint16_t getAktualChargeCurrentSoll()
{
  return i16_mAktualChargeCurrentSoll;
}


//Wird vom Task aus der main.c zyklisch aufgerufen
void canTxCyclicRun()
{
  if(WebSettings::getBool(ID_PARAM_BMS_CAN_ENABLE,0,0,0))
  {
    sendBmsCanMessages();
  }
}


void sendCanMsg(uint32_t identifier, uint8_t *buffer, uint8_t length)
{
  uint16_t ret = CAN.beginPacket(identifier); //11 bit Id
  ret = CAN.write(buffer, length);
  ret = CAN.endPacket();
}


void sendBmsCanMessages()
{
  switch (u8_mSelCanInverter)
  {
    
    case ID_CAN_DEVICE_DEYE:
    case ID_CAN_DEVICE_SOLISRHI:
      sendCanMsg_351();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_355();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_356();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_35e();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_359();
      break;

    case ID_CAN_DEVICE_VICTRON:
      // CAN-IDs for core functionality: 0x351, 0x355, 0x356 and 0x35A.

      sendCanMsg_351();
      vTaskDelay(pdMS_TO_TICKS(5));

      sendCanMsg_370_371();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_35e();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_35a(); //Alarm Details
      vTaskDelay(pdMS_TO_TICKS(5));

      //372
      //35f

      sendCanMsg_355();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_356();
      vTaskDelay(pdMS_TO_TICKS(5));
      sendCanMsg_373();
      //374, 375, 376, 377

      //sendCanMsg_359();
      break;

    default:
      break;
  }
}


void calcMaximalenLadestromSprung(int16_t i16_pNewChargeCurrent)
{
    //Evtl. Sprünge im Batteriestrom und hohe Lastströme berücksichtigen

    if(u16_mTimerChargeOff>0)u16_mTimerChargeOff--;
    if(u16_mTimerChargeOff==0)
    {
      u8_mTimerCalcMaxChareCurrent++;

      /* Wird der neue Soll-Ladestrom kleiner, dann wird dieser sofort geändert 
      * um bei hoher Zellspannung schnell ausregeln zu können.
      * Ist der neue Soll-Ladestrom größer, dann wird dieser nur alle 30 Sekunden geändert. */
      if(i16_pNewChargeCurrent<i16_mMaxChargeCurrent) 
      {
        //debugPrintf("Sprung unten > 5A (a): i16_pNewChargeCurrent=%i, i16_mMaxChargeCurrent=%",i16_pNewChargeCurrent,i16_mMaxChargeCurrent);
        if(i16_mMaxChargeCurrent>=50){
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-10;
        }else if(i16_mMaxChargeCurrent>=25 && i16_mMaxChargeCurrent<50){
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-5;
        }else if(i16_mMaxChargeCurrent>=10 && i16_mMaxChargeCurrent<25){
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-3;
        }else if(i16_mMaxChargeCurrent<10){
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-1;
        }
        if(i16_pNewChargeCurrent<0)i16_pNewChargeCurrent=0;
        //debugPrintf("Sprung unten: i16_pNewChargeCurrent=%i, i16_mMaxChargeCurrent=%i",i16_pNewChargeCurrent,i16_mMaxChargeCurrent);

        u8_mTimerCalcMaxChareCurrent=0;
        if(i16_pNewChargeCurrent==0 && i16_mMaxChargeCurrent>0) //Strom wurde auf 0 geregelt -> Sperrzeit für Aufwärtsregelung starten
        {
          u16_mTimerChargeOff=u16_mSperrzeitChargeOff; //Sperrzeit ab wann wieder geladen werden darf wenn auf 0 gereglt wird
        }

        i16_mMaxChargeCurrent = i16_pNewChargeCurrent;
      }
      else
      {
        if(u8_mTimerCalcMaxChareCurrent==15)
        {
          u8_mTimerCalcMaxChareCurrent=0;

          if(i16_pNewChargeCurrent-i16_mMaxChargeCurrent>10) //Maximal 10A Sprünge nach oben erlauben
          {
            i16_pNewChargeCurrent=i16_mMaxChargeCurrent+10;
            //debugPrintf("Sprung oben: i16_pNewChargeCurrent=%i, i16_mMaxChargeCurrent=%i",i16_pNewChargeCurrent,i16_mMaxChargeCurrent);
          }

          i16_mMaxChargeCurrent = i16_pNewChargeCurrent;
        }
      }
    }
}


/* Berechnet der Maximalzulässigen Ladestrom anhand der eigestellten Zellspannungsparameter */
int16_t calcLadestromZellspanung(int16_t i16_pMaxChargeCurrent)
{
  if(WebSettings::getBool(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_EN,0,0,0)==true) //wenn enabled
  {
    uint16_t u16_lAktuelleMaxZellspg = getBmsMaxCellVoltage(u8_mBmsDatasource);
    uint16_t u16_lStartSpg = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_STARTSPG,0,0,0);
    if(u16_lStartSpg<=u16_lAktuelleMaxZellspg)
    {
      uint16_t u16_lEndSpg = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_ENDSPG,0,0,0);
      int16_t i16_lMindestChargeCurrent = (WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM,0,0,0));

      if(u16_lStartSpg>u16_lEndSpg) return i16_pMaxChargeCurrent; //Startspannung > Endspannung => Fehler
      if(i16_pMaxChargeCurrent<=i16_lMindestChargeCurrent) return i16_pMaxChargeCurrent; //Maximaler Ladestrom < Mindest-Ladestrom => Fehler

      if(u16_lAktuelleMaxZellspg>u16_lEndSpg) 
      {
        //Wenn die aktuelle Zellspannung bereits größer als der Endzellspannung ist, 
        //dann Ladestrom auf Mindest-Ladestrom einstellen
        return i16_lMindestChargeCurrent;
      }
      else
      {
        uint32_t u32_lAenderungProMv = ((u16_lEndSpg-u16_lStartSpg)*100)/(i16_pMaxChargeCurrent-i16_lMindestChargeCurrent); //Änderung pro mV
        uint32_t u32_lStromAenderung = ((u16_lAktuelleMaxZellspg-u16_lStartSpg)*100)/u32_lAenderungProMv; //Ladestrom um den theoretisch reduziert werden muss
        if(u32_lStromAenderung>(i16_pMaxChargeCurrent-i16_lMindestChargeCurrent))
        {
          return i16_lMindestChargeCurrent;
        }
        else
        {
          uint16_t u16_lNewChargeCurrent = i16_pMaxChargeCurrent-u32_lStromAenderung; //neuer Ladestrom
          return u16_lNewChargeCurrent;
        }

      }
    }
  }
  return i16_pMaxChargeCurrent;
}


/* */
int16_t calcLadestromBeiZelldrift(int16_t i16_pMaxChargeCurrent)
{
  int16_t i16_lMaxChargeCurrent = i16_pMaxChargeCurrent;

  if(WebSettings::getBool(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLDRIFT_EN,0,0,0)==true) //wenn enabled
  {
    //Maximalen Ladestrom berechnen
    uint16_t u32_lMaxCellDrift = getBmsMaxCellDifferenceVoltage(u8_mBmsDatasource);
    if(u32_lMaxCellDrift>0)
    {
      if(u32_lMaxCellDrift>=WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTABWEICHUNG,0,0,0)) //Wenn Drift groß genug ist
      {
        if(getBmsMaxCellVoltage(u8_mBmsDatasource)>=WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTSPG_ZELLE,0,0,0)) //Wenn höchste Zellspannung groß genug ist
        {
          i16_lMaxChargeCurrent = i16_lMaxChargeCurrent-(u32_lMaxCellDrift*WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_MV,0,0,0));
          if(i16_lMaxChargeCurrent<0) i16_lMaxChargeCurrent=0;
        }
      }
    }
  }

  return i16_lMaxChargeCurrent;
}


/* */
int16_t calcLadestromSocAbhaengig(int16_t i16_lMaxChargeCurrent, uint8_t u8_lSoc)
{
  if(WebSettings::getBool(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_SOC_EN,0,0,0)==true) //wenn enabled
  {
    uint8_t u8_lReduzierenAbSoc = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_AB_SOC,0,0,0);
    if(u8_lSoc>=u8_lReduzierenAbSoc && u8_lSoc<100)
    {
      uint8_t u8_lReduzierenUmA = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_PERCENT_SOC,0,0,0);

      if(i16_lMaxChargeCurrent-((u8_lSoc-u8_lReduzierenAbSoc+1)*u8_lReduzierenUmA)>=0)
      {
        return i16_lMaxChargeCurrent-((u8_lSoc-u8_lReduzierenAbSoc+1)*u8_lReduzierenUmA);
      }
      else
      {
        return 0;
      }
    }

    return i16_lMaxChargeCurrent;
  }

  return i16_lMaxChargeCurrent;
}


/* *******************************************************************************************
 * getNewSocByMinCellVoltage() 
 * Wenn die eingestellte mindest-Zellspannung unterschritten wird, dann kann ein belibiger SoC 
 * an den Wechselrichter gesendet werden. Hiermit kann ein Nachladen erzwungen werden. 
 * *******************************************************************************************/
uint8_t getNewSocByMinCellVoltage(uint8_t u8_lSoc)
{
  //Wenn Zellspannung unterschritten wird, dann SoC x% an Inverter senden
  switch(u8_mSocZellspannungState)
  {
    //Warte bis Zellspannung kleiner Mindestspannung
    case STATE_MINCELLSPG_SOC_WAIT_OF_MIN:
      if(getBmsMinCellVoltage(u8_mBmsDatasource)<=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG,0,0,0))
      {
        u8_mSocZellspannungState=STATE_MINCELLSPG_SOC_BELOW_MIN;
      }
      break;

    //Spannung war kleiner als Mindestspannung
    case STATE_MINCELLSPG_SOC_BELOW_MIN:
      uint16_t u16_lZellspgChargeEnd;
      u16_lZellspgChargeEnd = WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END,0,0,0);
      //Wenn Parameter ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END 0 ist, dann Ladestartspannung nehmen
      if(u16_lZellspgChargeEnd==0) 
      {
        u16_lZellspgChargeEnd=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG,0,0,0);
      }

      if(getBmsMinCellVoltage(u8_mBmsDatasource)>u16_lZellspgChargeEnd)
      {
        u16_mSocZellspannungSperrzeitTimer=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_TIME,0,0,0);
        u8_mSocZellspannungState=STATE_MINCELLSPG_SOC_LOCKTIMER;
      }
      u8_lSoc=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SOC,0,0,0);
      break;

    //Sperrzeit läuft, warte auf ablauf der Sperrezit
    case STATE_MINCELLSPG_SOC_LOCKTIMER:
      u16_mSocZellspannungSperrzeitTimer--;
      if(u16_mSocZellspannungSperrzeitTimer==0)
      {
        u8_mSocZellspannungState=STATE_MINCELLSPG_SOC_WAIT_OF_MIN;
      }
      break;

    //default:
    //  break;
  }

  return u8_lSoc;
}


// Transmit hostname
void sendCanMsg_370_371()
{
  sendCanMsg(0x370, (uint8_t *)&hostname, 8);
  sendCanMsg(0x371, (uint8_t *)&hostname[8], 8);
}

void sendCanMsg_35e()
{
  sendCanMsg(0x35e, (uint8_t *)&hostname, 6);
}

/*
 * Data 0 + 1:
 * CVL: Battery Charge Voltage (data type : 16bit unsigned int, byte order : little endian, scale factor : 0.1, unit : V) 
 * Data 2 + 3:
 * CCL: DC Charge Current Limitation (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : A) 
 * Data 4 + 5:
 * DCL: DC Discharge Current Limitation (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : A) 
*/
void sendCanMsg_351()
{
  data351 msgData;
  uint8_t errors = 0;
  //@ToDo: Fehler feststellen


  if (errors!=0) //wenn Fehler
  {    
    msgData.chargevoltagelimit  = (uint16_t)(WebSettings::getFloat(ID_PARAM_BMS_MAX_CHARGE_SPG,0,0,0)*10.0); // 55200/100;
    msgData.maxchargecurrent    = 0;
    msgData.maxdischargecurrent = 0;
    msgData.dischargevoltage    = 0; //not use
  }
  else
  {
    msgData.chargevoltagelimit  = (uint16_t)(WebSettings::getFloat(ID_PARAM_BMS_MAX_CHARGE_SPG,0,0,0)*10.0); //55200/100;
    msgData.dischargevoltage    = 0; //not use

    if(alarmSetChargeCurrentToZero)
    {
      msgData.maxchargecurrent=0;
    }
    else
    {

      int16_t i16_lMaxChargeCurrentOld=i16_mMaxChargeCurrent;
      int16_t i16_lMaxChargeCurrent = (int16_t)(WebSettings::getInt(ID_PARAM_BMS_MAX_CHARGE_CURRENT,0,0,0));

      int16_t i16_lMaxChargeCurrent1 = calcLadestromZellspanung(i16_lMaxChargeCurrent);
      int16_t i16_lMaxChargeCurrent2 = calcLadestromSocAbhaengig(i16_lMaxChargeCurrent, getBmsChargePercentage(u8_mBmsDatasource));

      i16_lMaxChargeCurrent = calcLadestromBeiZelldrift(i16_lMaxChargeCurrent);
      
      if(i16_lMaxChargeCurrent>i16_lMaxChargeCurrent1) i16_lMaxChargeCurrent=i16_lMaxChargeCurrent1;
      if(i16_lMaxChargeCurrent2<i16_lMaxChargeCurrent) i16_lMaxChargeCurrent=i16_lMaxChargeCurrent2;

      calcMaximalenLadestromSprung(i16_lMaxChargeCurrent); //calcMaximalenLadestromSprung schreibt den neuen Ausgangsstrom in i16_mMaxChargeCurrent
      //debugPrintf("Soll Ladestrom: %i, %i, %i",i16_lMaxChargeCurrent1, i16_lMaxChargeCurrent, i16_mMaxChargeCurrent);

      //Soll-Ladestrom in die Ausgangs-Msg. schreiben
      msgData.maxchargecurrent = i16_mMaxChargeCurrent*10;

      //Wenn sich der Wert geändert hat per mqqt senden
      if(i16_mMaxChargeCurrent!=i16_lMaxChargeCurrentOld)
      {
        mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_CURRENT_SOLL, -1, getAktualChargeCurrentSoll());
      }
    }

    //Entladestrom
    if(alarmSetDischargeCurrentToZero)
    {
      msgData.maxdischargecurrent=0;
    }
    else
    {
      msgData.maxdischargecurrent = (int16_t)(WebSettings::getInt(ID_PARAM_BMS_MAX_DISCHARGE_CURRENT,0,0,0)*10);
    }

  }

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  inverterData.inverterChargeCurrent = msgData.maxchargecurrent;
  inverterData.inverterDischargeCurrent = msgData.maxdischargecurrent;
  xSemaphoreGive(mInverterDataMutex);

  i16_mAktualChargeCurrentSoll=msgData.maxchargecurrent/10;
  sendCanMsg(0x351, (uint8_t *)&msgData, sizeof(msgData));
}


/* SOC
 * Data 0 + 1:
 * SOC Value (data type : 16bit unsigned int, byte order : little endian, scale factor : 1, unit : %) 
 * Data 2 + 3:
 * SOH Value (data type : 16bit unsigned int, byte order : little endian, scale factor : 1, unit : %) 
 */
void sendCanMsg_355()
{
  data355 msgData;

  if(alarmSetSocToFull)
  {
    //debugPrintln("SOC aufgrund von Alarm auf 100%");
    msgData.soc = 100;
  }
  else
  {
    msgData.soc = getBmsChargePercentage(u8_mBmsDatasource); // SOC, uint16 1 %

    //Wenn Zellspannung unterschritten wird, dann SoC x an Inverter senden
    msgData.soc = getNewSocByMinCellVoltage(msgData.soc);
  }

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  inverterData.inverterSoc = msgData.soc;
  xSemaphoreGive(mInverterDataMutex);

  msgData.soh = 100; // SOH, uint16 1 %
  sendCanMsg(0x355, (uint8_t *)&msgData, sizeof(data355));
}


/* Battery voltage
 * Data 0 + 1:
 * Battery Voltage (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.01, unit : V) 
 * Data 2 + 3:
 * Battery Current (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : A) 
 * Data 4 + 5:
 * Battery Temperature (data type : 16bit signed int, 2's complement, byte order : little endian, scale factor : 0.1, unit : degC) 
 */
void sendCanMsg_356()
{
  data356 msgData;

  msgData.current = (int16_t)(getBmsTotalCurrent(u8_mBmsDatasource)*10);
  msgData.temperature = (int16_t)(getBmsTempature(u8_mBmsDatasource,0)*10);
  msgData.voltage = (int16_t)(getBmsTotalVoltage(u8_mBmsDatasource)*100);

  //Wenn zusätzliche Datenquellen angegeben sind:
  if((u8_mBmsDatasourceAdd & 0x01) == 0x01) msgData.current += (int16_t)(getBmsTotalCurrent(BT_DEVICES_COUNT)*10);
  if((u8_mBmsDatasourceAdd & 0x02) == 0x02) msgData.current += (int16_t)(getBmsTotalCurrent(BT_DEVICES_COUNT+1)*10);
  if((u8_mBmsDatasourceAdd & 0x04) == 0x04) msgData.current += (int16_t)(getBmsTotalCurrent(BT_DEVICES_COUNT+2)*10);

  //debugPrintf("CAN: current=%i temperature=%i voltage=%i", msgData.current, msgData.temperature, msgData.voltage);

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  inverterData.inverterVoltage = msgData.voltage;
  inverterData.inverterCurrent = msgData.current;
  xSemaphoreGive(mInverterDataMutex);

  sendCanMsg(0x356, (uint8_t *)&msgData, sizeof(data356));
}


// Send alarm details
void sendCanMsg_359()
{
  data35a msgData;

  /*
  Pylontech V1.2

  Data 0 (Alarm)
  0: -
  1: Cell/module over voltage
  2: Cell/module under voltage
  3: Cell over temp
  4: Cell under temp
  5: -
  6: -
  7: Discharge over current 

  Data 1 (Alarm)
  0: Charge over current 
  1: -
  2: -
  3: System error
  4: -
  5: -
  6: -
  7: -

  Data 2 (Warning)
  0: -
  1: Cell/module high voltage
  2: Cell/module low voltage
  3: Cell high temp
  4: Cell low temp
  5: -
  6: -
  7: Discharg high current 

  Data 3 (Warning)
  0: Charge high current 
  1: -
  2: -
  3: System error
  4: -
  5: -
  6: -
  7: -

  Data 4: Pack number (data type : 8bit unsigned char)
  Data 5: 0x50
  Data 6: 0x4E
  Data 6: -
  */
 
  msgData.u8_b0=0;
  msgData.u8_b1=0;
  msgData.u8_b2=0;
  msgData.u8_b3=0;
  msgData.u8_b4=0;
  msgData.u8_b5=0;
  msgData.u8_b6=0;
  msgData.u8_b7=0;

  sendCanMsg(0x359, (uint8_t *)&msgData, sizeof(data35a));
}


// Send alarm details
void sendCanMsg_35a()
{

  // 0 (bit 0+1) General alarm (not implemented)
  // 0 (bit 2+3) Battery low voltage alarm
  // 0 (bit 4+5) Battery high voltage alarm
  // 0 (bit 6+7) Battery high temperature alarm

  // 1 (bit 0+1) Battery low temperature alarm
  // 1 (bit 2+3) Battery high temperature charge alarm
  // 1 (bit 4+5) Battery low temperature charge alarm
  // 1 (bit 6+7) Battery high current alarm
  
  // 2 (bit 0+1) Battery high charge current alarm
  // 2 (bit 2+3) Contactor Alarm (not implemented)
  // 2 (bit 4+5) Short circuit Alarm (not implemented)
  // 2 (bit 6+7) BMS internal alarm

  // 3 (bit 0+1) Cell imbalance alarm
  // 3 (bit 2+3) Reserved
  // 3 (bit 4+5) Reserved
  // 3 (bit 6+7) Reserved

  // 4 (bit 0+1) General warning (not implemented)
  // 4 (bit 2+3) Battery low voltage warning
  // 4 (bit 4+5) Battery high voltage warning
  // 4 (bit 6+7) Battery high temperature warning

  // 5 (bit 0+1) Battery low temperature warning
  // 5 (bit 2+3) Battery high temperature charge warning
  // 5 (bit 4+5) Battery low temperature charge warning
  // 5 (bit 6+7) Battery high current warning

  // 6 (bit 0+1) Battery high charge current warning
  // 6 (bit 2+3) Contactor warning (not implemented)
  // 6 (bit 4+5) Short circuit warning (not implemented)
  // 6 (bit 6+7) BMS internal warning

  // 7 (bit 0+1) Cell imbalance warning
  // 7 (bit 2+3) System status (online/offline) [1]
  // 7 (bit 4+5) Reserved
  // 7 (bit 6+7) Reserved


  data35a msgData;

  msgData.u8_b0=0;
  msgData.u8_b1=0;
  msgData.u8_b2=0;
  msgData.u8_b3=0;
  msgData.u8_b4=0;
  msgData.u8_b5=0;
  msgData.u8_b6=0;
  msgData.u8_b7=0;

  sendCanMsg(0x35a, (uint8_t *)&msgData, sizeof(data35a));
}


void sendCanMsg_373()
{
  data373 msgData;

  uint16_t u16_lCellVoltageMax=getBmsMaxCellVoltage(u8_mBmsDatasource);
  uint16_t u16_lCellVoltageMin=getBmsMinCellVoltage(u8_mBmsDatasource);

  //Wenn zusätzliche Datenquellen angegeben sind:
  if((u8_mBmsDatasourceAdd & 0x01) == 0x01)
  { 
    if(getBmsMaxCellVoltage(BT_DEVICES_COUNT)>u16_lCellVoltageMax) u16_lCellVoltageMax=getBmsMaxCellVoltage(BT_DEVICES_COUNT); 
  }
  if((u8_mBmsDatasourceAdd & 0x02) == 0x02)
  { 
    if(getBmsMaxCellVoltage(BT_DEVICES_COUNT+1)>u16_lCellVoltageMax) u16_lCellVoltageMax=getBmsMaxCellVoltage(BT_DEVICES_COUNT+1); 
  }
  if((u8_mBmsDatasourceAdd & 0x04) == 0x04)
  { 
    if(getBmsMaxCellVoltage(BT_DEVICES_COUNT+2)>u16_lCellVoltageMax) u16_lCellVoltageMax=getBmsMaxCellVoltage(BT_DEVICES_COUNT+2); 
  }

  if((u8_mBmsDatasourceAdd & 0x01) == 0x01)
  { 
    if(getBmsMinCellVoltage(BT_DEVICES_COUNT)<u16_lCellVoltageMin) u16_lCellVoltageMin=getBmsMinCellVoltage(BT_DEVICES_COUNT); 
  }
  if((u8_mBmsDatasourceAdd & 0x02) == 0x02)
  { 
    if(getBmsMinCellVoltage(BT_DEVICES_COUNT+1)<u16_lCellVoltageMin) u16_lCellVoltageMin=getBmsMinCellVoltage(BT_DEVICES_COUNT+1); 
  }
  if((u8_mBmsDatasourceAdd & 0x04) == 0x04)
  { 
    if(getBmsMinCellVoltage(BT_DEVICES_COUNT+2)<u16_lCellVoltageMin) u16_lCellVoltageMin=getBmsMinCellVoltage(BT_DEVICES_COUNT+2); 
  }


  msgData.maxCellVoltage = u16_lCellVoltageMax;
  msgData.minCellColtage = u16_lCellVoltageMin;
  msgData.minCellTemp = 273 + getBmsTempature(u8_mBmsDatasource,1);
  msgData.maxCellTemp = 273 + getBmsTempature(u8_mBmsDatasource,2);

  sendCanMsg(0x373, (uint8_t *)&msgData, sizeof(data373));
}