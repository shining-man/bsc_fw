// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Canbus.h"
#include "WebSettings.h"
#include "defines.h"
#include "BmsData.h"
#include "Ow.h"
#include "mqtt_t.h"
#include <CAN.h>
#include "log.h"
#include "AlarmRules.h"

static const char *TAG = "CAN";

void sendBmsCanMessages();
void sendCanMsg_370_371();
void sendCanMsg_35e();
void sendCanMsg_351();
void sendCanMsg_355();
void sendCanMsg_356();
void sendCanMsg_359();
void sendCanMsg_35a();
void sendCanMsg_373_376_377();
void sendCanMsgTemp();
void sendCanMsgBmsData();
void sendCanMsg_372();
void sendCanMsg_35f();
void sendCanMsg(uint32_t identifier, uint8_t *buffer, uint8_t length);

void onCanReceive(int packetSize);

static SemaphoreHandle_t mInverterDataMutex = NULL;
static struct inverterData_s inverterData;

char hostname[16] = {'B','S','C',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};

uint8_t u8_mMqttTxTimer=0;

uint8_t u8_mBmsDatasource;
uint16_t u8_mBmsDatasourceAdd;
uint8_t u8_mSelCanInverter;

bool alarmSetChargeCurrentToZero;
bool alarmSetDischargeCurrentToZero;
bool alarmSetSocToFull;

int16_t  i16_mMaxChargeCurrent=0;
int16_t  i16_mAktualChargeCurrentSoll;
//uint8_t  u8_mTimerCalcMaxChareCurrent=0;
//uint16_t u16_mTimerChargeOff=0;
//uint16_t u16_mSperrzeitChargeOff;
int16_t  i16_mAktualDischargeCurrentSoll=0;

//Variablen: Wenn Zellspannung kleiner x mV wird SoC auf x% setzen
enum SM_SocZellspgStates {STATE_MINCELLSPG_SOC_WAIT_OF_MIN=0, STATE_MINCELLSPG_SOC_BELOW_MIN, STATE_MINCELLSPG_SOC_LOCKTIMER};
uint8_t  u8_mSocZellspannungState;
uint16_t u16_mSocZellspannungSperrzeitTimer;

//uint8_t u8_mModulesCntCharge;
//uint8_t u8_mModulesCntDischarge;

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


//Test
uint32_t u32_lastCanId;
uint16_t can_batVolt=0;
int16_t can_batCurr=0;
uint8_t can_soc=0;

void canSetup()
{
  mInverterDataMutex = xSemaphoreCreateMutex();

  u8_mBmsDatasource=0;
  alarmSetChargeCurrentToZero=false;
  alarmSetDischargeCurrentToZero=false;
  alarmSetSocToFull=false;

  //u8_mTimerCalcMaxChareCurrent=0;
  //u16_mTimerChargeOff=0;

  //u16_mSperrzeitChargeOff=WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_SPERRZEIT,0,DT_ID_PARAM_INVERTER_LADESTROM_SPERRZEIT); //Sperrzeit ab wann wieder geladen werden darf wenn auf 0 gereglt wird

  u16_mSocZellspannungSperrzeitTimer=0;
  u8_mSocZellspannungState=STATE_MINCELLSPG_SOC_WAIT_OF_MIN;

  loadCanSettings();

  
  CAN.setPins(5,4);
  if (!CAN.begin(500000)) // start the CAN bus at 500 kbps
  //if (!CAN.begin(250E3)) // start the CAN bus at 250 kbps
  {
    BSC_LOGI(TAG,"Init CAN failed!");
  }
  else
  {
    BSC_LOGI(TAG,"Init CAN ok");
  } 

  CAN.onReceive(onCanReceive);
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
  u8_mBmsDatasource = WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE,0,DT_ID_PARAM_BMS_CAN_DATASOURCE);
  u8_mSelCanInverter = WebSettings::getInt(ID_PARAM_SS_CAN,0,DT_ID_PARAM_SS_CAN);
  
  uint32_t bmsConnectFilter=0;
  /*for(uint8_t i;i<BT_DEVICES_COUNT;i++)
  {
    if(WebSettings::getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV)!=0)
    {
      bmsConnectFilter |= (1<<i);
    }
  }*/
  for(uint8_t i;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    if(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,i,DT_ID_PARAM_SERIAL_CONNECT_DEVICE)!=0)
    {
      bmsConnectFilter |= (1<<i);
    }
  }

  u8_mBmsDatasourceAdd=(((uint32_t)WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE_SS1,0,DT_ID_PARAM_BMS_CAN_DATASOURCE_SS1))&bmsConnectFilter);

  BSC_LOGI(TAG,"loadCanSettings(): dataSrcAdd=%i, u8_mBmsDatasource=%i, bmsConnectFilter=%i, u8_mBmsDatasourceAdd=%i",WebSettings::getInt(ID_PARAM_BMS_CAN_DATASOURCE_SS1,0,DT_ID_PARAM_BMS_CAN_DATASOURCE_SS1),u8_mBmsDatasource,bmsConnectFilter, u8_mBmsDatasourceAdd);
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
  if(WebSettings::getBool(ID_PARAM_BMS_CAN_ENABLE,0))
  {
    u8_mMqttTxTimer++;
    sendBmsCanMessages();
    if(u8_mMqttTxTimer>=15)u8_mMqttTxTimer=0;
  }
}


void sendCanMsg(uint32_t identifier, uint8_t *buffer, uint8_t length)
{
  uint32_t err = CAN.beginPacket(identifier); //11 bit Id
  err = CAN.write(buffer, length);
  err = CAN.endPacket();
  if(err>0) BSC_LOGE(TAG,"TX error: REG_ECC=%i, REG_SR=%i", ((err>>8)&0xff), (err&0xff));
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
      sendCanMsg_359(); //Alarms
      break;

    case ID_CAN_DEVICE_VICTRON:
      // CAN-IDs for core functionality: 0x351, 0x355, 0x356 and 0x35A.
      sendCanMsg_351();
      sendCanMsg_370_371();
      sendCanMsg_35e();
      sendCanMsg_35a(); //Alarms

      sendCanMsg_372();
      sendCanMsg_35f();

      sendCanMsg_355();
      sendCanMsg_356();
      sendCanMsg_373_376_377();

      //Send extended data
      if(WebSettings::getBool(ID_PARAM_BMS_CAN_EXTENDED_DATA_ENABLE,0)==true)
      {
        sendCanMsgTemp();
        sendCanMsgBmsData();
      }

      //374, 359
      break;

    default:
      break;
  }
}


uint8_t getNumberOfBatteryModules()
{
  uint8_t u8_lModules=1;

  if(u8_mBmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u8_mBmsDatasourceAdd>>i)&0x01)
      {
        u8_lModules++;
      }
    }
  }
  return u8_lModules;
}

uint8_t getNumberOfBatteryModulesCharge()
{
  uint8_t u8_lModules=0;
  if(getBmsStateFETsCharge(u8_mBmsDatasource)) u8_lModules++;
        
  if(u8_mBmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u8_mBmsDatasourceAdd>>i)&0x01)
      {
        if(getBmsStateFETsCharge(BMSDATA_FIRST_DEV_SERIAL+i))
        {
          u8_lModules++;
        }
      }
    }
  }
  return u8_lModules;
}

uint8_t getNumberOfBatteryModulesDischarge()
{
  uint8_t u8_lModules=0;
  if(getBmsStateFETsDischarge(u8_mBmsDatasource)) u8_lModules++;
        
  if(u8_mBmsDatasourceAdd>0)
  {
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u8_mBmsDatasourceAdd>>i)&0x01)
      {
        if(getBmsStateFETsDischarge(BMSDATA_FIRST_DEV_SERIAL+i))
        {
          u8_lModules++;
        }
      }
    }
  }
  return u8_lModules;
}

void buildBatteryCellText(char *buffer, uint8_t batteryNr, uint8_t cellNr)
{
  memset(buffer, 0, 8); // Clear
  if(batteryNr<BMSDATA_FIRST_DEV_SERIAL) snprintf(buffer, 8, "B%d C%d", batteryNr, cellNr);
  else snprintf(buffer, 8, "S%d C%d", batteryNr-BMSDATA_FIRST_DEV_SERIAL, cellNr);
}



//Maximale Zellspannung von allen aktiven BMSen ermitteln
uint16_t getMaxCellSpannungFromBms()
{
  uint8_t u8_lBmsNr=0;
  uint8_t u8_lCellNr=0;


  uint16_t u16_lCellSpg = getBmsMaxCellVoltage(u8_mBmsDatasource);
  u8_lBmsNr=u8_mBmsDatasource; 
  u8_lCellNr=getBmsMaxVoltageCellNumber(u8_mBmsDatasource);

  if(u8_mBmsDatasourceAdd>0)
  {
    uint16_t u16_lMaxCellSpg=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u8_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<5000) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          u16_lMaxCellSpg=getBmsMaxCellVoltage(BMSDATA_FIRST_DEV_SERIAL+i);
          if(u16_lMaxCellSpg>u16_lCellSpg)
          {
            u16_lCellSpg=u16_lMaxCellSpg; 
            u8_lBmsNr=BMSDATA_FIRST_DEV_SERIAL+i;
            u8_lCellNr=getBmsMaxVoltageCellNumber(BMSDATA_FIRST_DEV_SERIAL+i);
          }
        }
      }
    }
    #ifdef CAN_DEBUG
    BSC_LOGD(TAG,"getMaxCellSpannung: MaxSpg=%i, bms=%i, cell=%i",u16_lCellSpg, u8_lBmsNr, u8_lCellNr); //nur zum Debug
    #endif
  }
  #ifdef CAN_DEBUG
  else
  {
    BSC_LOGD(TAG,"getMaxCellSpannung: MaxSpg=%i",u16_lCellSpg); //nur zum Debug
  }
  #endif

  char buf[8];
  buildBatteryCellText(buf,u8_lBmsNr,u8_lCellNr);
  sendCanMsg(0x375, (uint8_t *)&buf, 8);

  return u16_lCellSpg;
}


//Minimale Zellspannung von allen aktiven BMSen ermitteln
uint16_t getMinCellSpannungFromBms()
{
  uint8_t u8_lBmsNr=0;
  uint8_t u8_lCellNr=0;

  uint16_t u16_lCellSpg = getBmsMinCellVoltage(u8_mBmsDatasource);
  u8_lBmsNr=u8_mBmsDatasource; 
  u8_lCellNr=getBmsMinVoltageCellNumber(u8_mBmsDatasource);

  if(u8_mBmsDatasourceAdd>0)
  {
    uint16_t u16_lMinCellSpg=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u8_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<5000) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          u16_lMinCellSpg=getBmsMinCellVoltage(BMSDATA_FIRST_DEV_SERIAL+i);
          if(u16_lMinCellSpg<u16_lCellSpg)
          {
            u16_lCellSpg=u16_lMinCellSpg; 
            u8_lBmsNr=BMSDATA_FIRST_DEV_SERIAL+i;
            u8_lCellNr=getBmsMinVoltageCellNumber(BMSDATA_FIRST_DEV_SERIAL+i); 
          }
        }
      }
    }
    #ifdef CAN_DEBUG
    BSC_LOGD(TAG,"getMinCellSpannung: MinSpg=%i, bms=%i, cell=%i",u16_lCellSpg, u8_lBmsNr, u8_lCellNr); //nur zum Debug
    #endif
  }
  #ifdef CAN_DEBUG
  else
  {
    BSC_LOGD(TAG,"getMinCellSpannung: MinSpg=%i",u16_lCellSpg); //nur zum Debug
  }
  #endif

  char buf[8];
  buildBatteryCellText(buf,u8_lBmsNr,u8_lCellNr);
  sendCanMsg(0x374, (uint8_t *)&buf, 8);

  return u16_lCellSpg;
}


//Maximale Cell-Difference von allen aktiven BMSen ermitteln
uint16_t getMaxCellDifferenceFromBms()
{
  uint8_t u8_lBmsNr=0; //nur zum Debug
  uint8_t u8_lCellNr=0;  //nur zum Debug

  uint16_t u16_lCellDiff = getBmsMaxCellDifferenceVoltage(u8_mBmsDatasource);

  if(u8_mBmsDatasourceAdd>0)
  {
    uint16_t u16_lMaxCellDiff=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u8_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<5000) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          u16_lMaxCellDiff=getBmsMaxCellDifferenceVoltage(BMSDATA_FIRST_DEV_SERIAL+i);
          if(u16_lMaxCellDiff>u16_lCellDiff)
          {
            u16_lCellDiff=u16_lMaxCellDiff; 
            u8_lBmsNr=BMSDATA_FIRST_DEV_SERIAL+i;  //nur zum Debug
            u8_lCellNr=getBmsMaxVoltageCellNumber(BMSDATA_FIRST_DEV_SERIAL+i); //nur zum Debug
          }
        }
      }
    }
    #ifdef CAN_DEBUG
    BSC_LOGD(TAG,"getMaxCellDifference: MaxDiff=%i, bms=%i, cell=%i",u16_lCellDiff, u8_lBmsNr, u8_lCellNr); //nur zum Debug
    #endif
  }
  #ifdef CAN_DEBUG
  else
  {
    BSC_LOGD(TAG,"getMaxCellDifference: MaxDiff=%i",u16_lCellDiff); //nur zum Debug
  }
  #endif

  return u16_lCellDiff;
}


/*
 * Regelfunktionen
 */
void calcMaximalenLadestromSprung(int16_t i16_pNewChargeCurrent)
{
    //Evtl. Sprünge im Batteriestrom und hohe Lastströme berücksichtigen

    /*if(u16_mTimerChargeOff>0)u16_mTimerChargeOff--;
    if(u16_mTimerChargeOff==0)
    {
      u8_mTimerCalcMaxChareCurrent++;*/

      /* Wird der neue Soll-Ladestrom kleiner, dann wird dieser sofort geändert 
      * um bei hoher Zellspannung schnell ausregeln zu können.
      * Ist der neue Soll-Ladestrom größer, dann wird dieser nur alle 30 Sekunden geändert. */
      if(i16_pNewChargeCurrent<i16_mMaxChargeCurrent) 
      {
        #ifdef CAN_DEBUG
        BSC_LOGD(TAG,"Sprung unten > 5A (a): i16_pNewChargeCurrent=%i, i16_mMaxChargeCurrent=%",i16_pNewChargeCurrent,i16_mMaxChargeCurrent);
        #endif

        if(i16_mMaxChargeCurrent>=50)
        {
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-10;
        }
        else if(i16_mMaxChargeCurrent>=25 && i16_mMaxChargeCurrent<50)
        {
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-5;
        }
        else if(i16_mMaxChargeCurrent>=10 && i16_mMaxChargeCurrent<25)
        {
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-3;
        }
        else if(i16_mMaxChargeCurrent<10)
        {
          i16_pNewChargeCurrent=i16_mMaxChargeCurrent-1;
        }
        if(i16_pNewChargeCurrent<0)i16_pNewChargeCurrent=0;
        #ifdef CAN_DEBUG
        BSC_LOGD(TAG,"Sprung unten: i16_pNewChargeCurrent=%i, i16_mMaxChargeCurrent=%i",i16_pNewChargeCurrent,i16_mMaxChargeCurrent);
        #endif

        /*u8_mTimerCalcMaxChareCurrent=0;
        if(i16_pNewChargeCurrent==0 && i16_mMaxChargeCurrent>0) //Strom wurde auf 0 geregelt -> Sperrzeit für Aufwärtsregelung starten
        {
          u16_mTimerChargeOff=u16_mSperrzeitChargeOff; //Sperrzeit ab wann wieder geladen werden darf wenn auf 0 gereglt wird
        }*/

        i16_mMaxChargeCurrent = i16_pNewChargeCurrent;
      }
      else
      {
        //if(u8_mTimerCalcMaxChareCurrent==15)
        //{
          //u8_mTimerCalcMaxChareCurrent=0;

          if(i16_pNewChargeCurrent-i16_mMaxChargeCurrent>10) //Maximal 10A Sprünge nach oben erlauben
          {
            i16_pNewChargeCurrent=i16_mMaxChargeCurrent+10;
            #ifdef CAN_DEBUG
            BSC_LOGD(TAG,"Sprung oben: i16_pNewChargeCurrent=%i, i16_mMaxChargeCurrent=%i",i16_pNewChargeCurrent,i16_mMaxChargeCurrent);
            #endif
          }

          i16_mMaxChargeCurrent = i16_pNewChargeCurrent;
        //}
      }
    //}
}


/* Berechnet der Maximalzulässigen Ladestrom anhand der eigestellten Zellspannungsparameter */
int16_t calcLadestromZellspanung(int16_t i16_pMaxChargeCurrent)
{
  if(WebSettings::getBool(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_EN,0)==true) //wenn enabled
  {
    //Maximale Zellspannung von den aktiven BMSen ermitteln
    uint16_t u16_lAktuelleMaxZellspg = getMaxCellSpannungFromBms();

    uint16_t u16_lStartSpg = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_STARTSPG,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_STARTSPG);
    if(u16_lStartSpg<=u16_lAktuelleMaxZellspg)
    {
      uint16_t u16_lEndSpg = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_ENDSPG,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_ENDSPG);
      int16_t i16_lMindestChargeCurrent = (WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM));

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

  if(WebSettings::getBool(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLDRIFT_EN,0)==true) //wenn enabled
  {
    //Maximalen Ladestrom berechnen
    uint16_t u32_lMaxCellDrift = getMaxCellDifferenceFromBms();
    uint16_t u16_lstartDrift = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTABWEICHUNG,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTABWEICHUNG);
    if(u32_lMaxCellDrift>0)
    {
      if(u32_lMaxCellDrift>u16_lstartDrift) //Wenn Drift groß genug ist
      {
        if(getMaxCellSpannungFromBms()>=WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTSPG_ZELLE,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTSPG_ZELLE)) //Wenn höchste Zellspannung groß genug ist
        {
          i16_lMaxChargeCurrent = i16_lMaxChargeCurrent-((u32_lMaxCellDrift-u16_lstartDrift)*WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_MV,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_MV));
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
  if(WebSettings::getBool(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_SOC_EN,0)==true) //wenn enabled
  {
    uint8_t u8_lReduzierenAbSoc = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_AB_SOC,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_AB_SOC);
    if(u8_lSoc>=u8_lReduzierenAbSoc && u8_lSoc<100)
    {
      uint8_t u8_lReduzierenUmA = WebSettings::getInt(ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_PERCENT_SOC,0,DT_ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_PERCENT_SOC);

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


/*
 * Ladestrom herunterregeln, wenn von einem Pack die Stromgrenze überschritten wird
 */
int16_t calcChargecurrent_MaxCurrentPerPackToHigh(int16_t i16_pMaxChargeCurrent)
{
  if(u8_mBmsDatasourceAdd>0)
  {
    //uint16_t u16_lMaxCellDiff=0;
    for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
    {
      if((u8_mBmsDatasourceAdd>>i)&0x01)
      {
        if((millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i))<5000) //So lang die letzten 5000ms Daten kamen ist alles gut
        {
          float fl_lTotalCurrent=getBmsTotalCurrent(BMSDATA_FIRST_DEV_SERIAL+i);
          if(fl_lTotalCurrent>(float)WebSettings::getInt(ID_PARAM_BATTERY_PACK_CHARGE_CURRENT,i,DT_ID_PARAM_BATTERY_PACK_CHARGE_CURRENT))
          {
            //Ladestrom für einen Pack (BMS) wird zu groß -> Ladestrom herunterregeln
            #ifdef CAN_DEBUG
            BSC_LOGD(TAG,"MaxCurrentPerPackToHigh: current=%i",i16_pMaxChargeCurrent); //nur zum Debug
            #endif
            return i16_mAktualChargeCurrentSoll-10; 
          }
        }
      }
    }
  }

  return i16_pMaxChargeCurrent;
}


/********************************************************************************************
 * calcChargeCurrentCutOff(int16_t u16_lChargeCurrent)
 * Ladestrom auf 0 setzen, wenn längere Zeit mit einem geringen Ladestrom geladen wurde.
 * 
 ********************************************************************************************/
int16_t calcChargeCurrentCutOff(int16_t u16_lChargeCurrent)
{
  static uint16_t u16_mChargeCurrentCutOfTimer=0;
  float fl_lTotalCurrent=0;

  uint16_t u16_lCutOffTime = (uint16_t)WebSettings::getInt(ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_TIME,0,DT_ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_TIME);
  if(u16_lCutOffTime==0) return u16_lChargeCurrent;

  float fl_lCutOffCurrent = (uint16_t)WebSettings::getInt(ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_CURRENT,0,DT_ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_CURRENT);
  uint8_t u16_lCutOffSoc = (uint8_t)WebSettings::getInt(ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_SOC,0,DT_ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_SOC);

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  uint8_t u8_lSoc = (uint8_t)inverterData.inverterSoc;
  xSemaphoreGive(mInverterDataMutex);

  //Aktuellen Ladestrom berechnen
  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    if((u8_mBmsDatasource-BMSDATA_FIRST_DEV_SERIAL)==i || (u8_mBmsDatasourceAdd>>i)&0x01)
    {
      if(getBmsStateFETsCharge(BMSDATA_FIRST_DEV_SERIAL+i) || getBmsStateFETsDischarge(BMSDATA_FIRST_DEV_SERIAL+i))
      {
        fl_lTotalCurrent += getBmsTotalCurrent(BMSDATA_FIRST_DEV_SERIAL+i);
      }
    }
  }

  if(u16_mChargeCurrentCutOfTimer>=u16_lCutOffTime)
  {
    if(u8_lSoc < u16_lCutOffSoc) //Wenn SoC zur Freigabe wieder unterschritten
    {
      u16_mChargeCurrentCutOfTimer=0;
    }
    else return 0;
  }
  else
  {
    //Timer hochzählen, wenn Strom kleiner 
    if(fl_lTotalCurrent>=0 && fl_lTotalCurrent<fl_lCutOffCurrent) u16_mChargeCurrentCutOfTimer++;
    else u16_mChargeCurrentCutOfTimer=0;
  }

  return u16_lChargeCurrent;
}


/* */
uint16_t calcDynamicReduzeChargeVolltage(uint16_t u16_lChargeVoltage)
{
  static uint16_t u16_lDynamicChargeVoltage = u16_lChargeVoltage;

  if(WebSettings::getBool(ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_EN,0)==true) //wenn enabled
  {
    uint16_t u16_lStartZellVoltage = WebSettings::getInt(ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_ZELLSPG,0,DT_ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_ZELLSPG);
    uint16_t u16_lDeltaCellVoltage= WebSettings::getInt(ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_DELTA,0,DT_ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_DELTA);

    if(getMaxCellSpannungFromBms()>u16_lStartZellVoltage)
    {
      uint16_t u16_lMaxCellDiffVoltage = getMaxCellDifferenceFromBms();
      if(u16_lMaxCellDiffVoltage>u16_lDeltaCellVoltage)
      {
        u16_lDynamicChargeVoltage-=1; //1=100mV
        if(u16_lDynamicChargeVoltage<0)u16_lDynamicChargeVoltage=0;
        return u16_lDynamicChargeVoltage;
      }
      else if(u16_lMaxCellDiffVoltage<u16_lDeltaCellVoltage)
      {
        u16_lDynamicChargeVoltage+=1; //1=100mV
        if(u16_lDynamicChargeVoltage>u16_lChargeVoltage)u16_lDynamicChargeVoltage=u16_lChargeVoltage;
        return u16_lDynamicChargeVoltage;
      }
    }
  }
  return u16_lChargeVoltage;
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
      if(getMinCellSpannungFromBms()<=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG,0,DT_ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG))
      {
        u8_mSocZellspannungState=STATE_MINCELLSPG_SOC_BELOW_MIN;
      }
      break;

    //Spannung war kleiner als Mindestspannung
    case STATE_MINCELLSPG_SOC_BELOW_MIN:
      uint16_t u16_lZellspgChargeEnd;
      u16_lZellspgChargeEnd = WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END,0,DT_ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END);
      //Wenn Parameter ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END 0 ist, dann Ladestartspannung nehmen
      if(u16_lZellspgChargeEnd==0) 
      {
        u16_lZellspgChargeEnd=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG,0,DT_ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG);
      }

      if(getMinCellSpannungFromBms()>u16_lZellspgChargeEnd)
      {
        u16_mSocZellspannungSperrzeitTimer=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_TIME,0,DT_ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_TIME);
        u8_mSocZellspannungState=STATE_MINCELLSPG_SOC_LOCKTIMER;
      }
      u8_lSoc=WebSettings::getInt(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SOC,0,DT_ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SOC);
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
    msgData.chargevoltagelimit  = (uint16_t)(WebSettings::getFloat(ID_PARAM_BMS_MAX_CHARGE_SPG,0)*10.0);
    msgData.maxchargecurrent    = 0;
    msgData.maxdischargecurrent = 0;
    msgData.dischargevoltage    = 0; //not use
  }
  else
  {
    msgData.dischargevoltage    = 0; //not use

    //Ladespannung
    uint16_t u16_lChargeVoltage = (uint16_t)(WebSettings::getFloat(ID_PARAM_BMS_MAX_CHARGE_SPG,0)*10.0); 
    u16_lChargeVoltage = calcDynamicReduzeChargeVolltage(u16_lChargeVoltage);
    msgData.chargevoltagelimit = u16_lChargeVoltage;
    if(u8_mMqttTxTimer==15)
    {
      mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_INVERTER_CHARGE_VOLTAGE, -1, (float)(u16_lChargeVoltage/10.0));
    }

    //Ladestrom
    if(alarmSetChargeCurrentToZero)
    {
      msgData.maxchargecurrent=0;
    }
    else
    {
      int16_t i16_lMaxChargeCurrentOld=i16_mMaxChargeCurrent;
      int16_t i16_lMaxChargeCurrent = (int16_t)(WebSettings::getInt(ID_PARAM_BMS_MAX_CHARGE_CURRENT,0,DT_ID_PARAM_BMS_MAX_CHARGE_CURRENT));
      //u8_mModulesCntCharge=1;
      
      //Maximalen Ladestrom aus den einzelnen Packs errechnen
      if(u8_mBmsDatasourceAdd>0)
      {
        uint16_t u16_lMaxCurrent=0;
        for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
        {
          if((u8_mBmsDatasource-BMSDATA_FIRST_DEV_SERIAL)==i || (u8_mBmsDatasourceAdd>>i)&0x01)
          {
            if(getBmsErrors(BMSDATA_FIRST_DEV_SERIAL+i)==0 && (millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i)<5000) &&
              getBmsStateFETsCharge(BMSDATA_FIRST_DEV_SERIAL+i))
            {
              u16_lMaxCurrent+=WebSettings::getInt(ID_PARAM_BATTERY_PACK_CHARGE_CURRENT,i,DT_ID_PARAM_BATTERY_PACK_CHARGE_CURRENT);
              //u8_mModulesCntCharge++;
            }
          }
        }
        if(u16_lMaxCurrent<i16_lMaxChargeCurrent) i16_lMaxChargeCurrent=u16_lMaxCurrent;
      }
      #ifdef CAN_DEBUG
      BSC_LOGI(TAG,"chargeCurrent Pack:%i",i16_lMaxChargeCurrent);
      #endif

      int16_t i16_lMaxChargeCurrentList[4];
      i16_lMaxChargeCurrentList[0] = calcLadestromZellspanung(i16_lMaxChargeCurrent);
      i16_lMaxChargeCurrentList[1] = calcLadestromSocAbhaengig(i16_lMaxChargeCurrent, getBmsChargePercentage(u8_mBmsDatasource));
      i16_lMaxChargeCurrentList[2] = calcLadestromBeiZelldrift(i16_lMaxChargeCurrent);
      //i16_lMaxChargeCurrentList[3] = calcChargecurrent_MaxCurrentPerPackToHigh(i16_lMaxChargeCurrent); //ToDO: Ein/Aus
      i16_lMaxChargeCurrentList[3] = calcChargeCurrentCutOff(i16_lMaxChargeCurrent);

      #ifdef CAN_DEBUG
      BSC_LOGD(TAG,"chargeCurrent Zellspg.:%i, SoC:%i, Zelldrift:%i",i16_lMaxChargeCurrentList[0],i16_lMaxChargeCurrentList[1],i16_lMaxChargeCurrentList[2]);
      #endif

      //Bestimmt kleinsten Ladestrom aller Optionen
      for(uint8_t i=0;i<sizeof(i16_lMaxChargeCurrentList)/sizeof(i16_lMaxChargeCurrentList[0]);i++)
      {
        if(i16_lMaxChargeCurrentList[i] < i16_lMaxChargeCurrent)
        {
          i16_lMaxChargeCurrent = i16_lMaxChargeCurrentList[i];
        }
      }

      calcMaximalenLadestromSprung(i16_lMaxChargeCurrent); //calcMaximalenLadestromSprung schreibt den neuen Ausgangsstrom in i16_mMaxChargeCurrent

      #ifdef CAN_DEBUG
      BSC_LOGD(TAG, "Soll Ladestrom: %i, %i, %i",i16_lMaxChargeCurrentList[0], i16_lMaxChargeCurrentList[1], i16_lMaxChargeCurrentList[2]);
      BSC_LOGI(TAG,"New charge current: %i, %i",i16_lMaxChargeCurrent,i16_mMaxChargeCurrent);
      #endif

      //Soll-Ladestrom in die Ausgangs-Msg. schreiben
      msgData.maxchargecurrent = i16_mMaxChargeCurrent*10;

      //Wenn sich der Wert geändert hat per mqqt senden
      if(i16_mMaxChargeCurrent!=i16_lMaxChargeCurrentOld || u8_mMqttTxTimer==15)
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
      int16_t i16_lMaxDischargeCurrent = (int16_t)WebSettings::getInt(ID_PARAM_BMS_MAX_DISCHARGE_CURRENT,0,DT_ID_PARAM_BMS_MAX_DISCHARGE_CURRENT);
      //u8_mModulesCntDischarge=1;

      //Maximalen Entladestrom aus den einzelnen Packs errechnen
      if(u8_mBmsDatasourceAdd>0)
      {
        int16_t i16_lMaxCurrent=0;
        for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
        {
          if((u8_mBmsDatasource-BMSDATA_FIRST_DEV_SERIAL)==i || (u8_mBmsDatasourceAdd>>i)&0x01)
          {
            #ifdef CAN_DEBUG
            BSC_LOGD(TAG,"MaxDischargeCurrent Pack: i=%i, bmsErr=%i, FETdisState=%i, time=%i",
              i,getBmsErrors(BMSDATA_FIRST_DEV_SERIAL+i),getBmsStateFETsDischarge(BMSDATA_FIRST_DEV_SERIAL+i),millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i));
            #endif

            if(getBmsErrors(BMSDATA_FIRST_DEV_SERIAL+i)==0 && (millis()-getBmsLastDataMillis(BMSDATA_FIRST_DEV_SERIAL+i)<5000) &&
              getBmsStateFETsDischarge(BMSDATA_FIRST_DEV_SERIAL+i))
            {
              i16_lMaxCurrent+=WebSettings::getInt(ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT,i,DT_ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT);
              //u8_mModulesCntDischarge++;
            }
          }
        }
        if(i16_lMaxCurrent<i16_lMaxDischargeCurrent) i16_lMaxDischargeCurrent=i16_lMaxCurrent;
      }
      #ifdef CAN_DEBUG
      BSC_LOGI(TAG,"dischargeCurrent Pack:%i",i16_lMaxDischargeCurrent);
      #endif


      //Wenn sich der Wert geändert hat per mqqt senden
      if(i16_lMaxDischargeCurrent!=i16_mAktualDischargeCurrentSoll || u8_mMqttTxTimer==15)
      {
        mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_DISCHARGE_CURRENT_SOLL, -1, i16_lMaxDischargeCurrent);
      }

      msgData.maxdischargecurrent = i16_lMaxDischargeCurrent*10;
    }
  }

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  inverterData.inverterChargeCurrent = msgData.maxchargecurrent;
  inverterData.inverterDischargeCurrent = msgData.maxdischargecurrent;
  xSemaphoreGive(mInverterDataMutex);

  i16_mAktualDischargeCurrentSoll=msgData.maxdischargecurrent/10;
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
    #ifdef CAN_DEBUG
    BSC_LOGD(TAG,"SOC aufgrund von Alarm auf 100%");
    #endif
    msgData.soc = 100;
  }
  else
  {
    msgData.soc = getBmsChargePercentage(u8_mBmsDatasource); // SOC, uint16 1 %

    uint8_t u8_lMultiBmsSocHandling = WebSettings::getInt(ID_PARAM_INVERTER_MULTI_BMS_VALUE_SOC,0,DT_ID_PARAM_INVERTER_MULTI_BMS_VALUE_SOC);

    if(u8_mBmsDatasourceAdd>0)
    {
      for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
      {
        if((u8_mBmsDatasourceAdd>>i)&0x01)
        {
          if(u8_lMultiBmsSocHandling==OPTION_MULTI_BMS_SOC_AVG)
          {
            msgData.soc+=getBmsChargePercentage(BMSDATA_FIRST_DEV_SERIAL+i);
            msgData.soc/=2;
          }
          else if(u8_lMultiBmsSocHandling==OPTION_MULTI_BMS_SOC_MAX)
          {
            if(getBmsChargePercentage(BMSDATA_FIRST_DEV_SERIAL+i)>msgData.soc) msgData.soc=getBmsChargePercentage(BMSDATA_FIRST_DEV_SERIAL+i);
          }
        }
      }
    }

    if(WebSettings::getBool(ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_EN,0)==true)
    {
      //Wenn Zellspannung unterschritten wird, dann SoC x an Inverter senden
      msgData.soc = getNewSocByMinCellVoltage(msgData.soc);
    }
  }

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  inverterData.inverterSoc = msgData.soc;
  xSemaphoreGive(mInverterDataMutex);

  if(u8_mMqttTxTimer==15)
  {
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_CHARGE_PERCENT, -1, msgData.soc);
  }

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
  msgData.voltage = (int16_t)(getBmsTotalVoltage(u8_mBmsDatasource)*100);

  //Wenn zusätzliche Datenquellen angegeben sind:
  if((u8_mBmsDatasourceAdd & 0x01) == 0x01) msgData.current += (int16_t)(getBmsTotalCurrent(BT_DEVICES_COUNT)*10);
  if((u8_mBmsDatasourceAdd & 0x02) == 0x02) msgData.current += (int16_t)(getBmsTotalCurrent(BT_DEVICES_COUNT+1)*10);
  if((u8_mBmsDatasourceAdd & 0x04) == 0x04) msgData.current += (int16_t)(getBmsTotalCurrent(BT_DEVICES_COUNT+2)*10);

  //Temperatur
  uint8_t u8_lBmsTempQuelle=WebSettings::getInt(ID_PARAM_INVERTER_BATT_TEMP_QUELLE,0,DT_ID_PARAM_INVERTER_BATT_TEMP_QUELLE);
  uint8_t u8_lBmsTempSensorNr=WebSettings::getInt(ID_PARAM_INVERTER_BATT_TEMP_SENSOR,0,DT_ID_PARAM_INVERTER_BATT_TEMP_SENSOR);
  if(u8_lBmsTempQuelle==1)
  {
    if(u8_lBmsTempSensorNr<3)
    {
      msgData.temperature = (int16_t)(getBmsTempature(u8_mBmsDatasource,u8_lBmsTempQuelle)*10);
    }
    else
    {
      msgData.temperature = (int16_t)(getBmsTempature(u8_mBmsDatasource,0)*10); //Im Fehlerfall immer Senor 0 des BMS nehmen
    }
  }
  else if(u8_lBmsTempQuelle==2)
  {
    if(u8_lBmsTempSensorNr<MAX_ANZAHL_OW_SENSOREN)
    {
      msgData.temperature = (int16_t)(owGetTemp(u8_lBmsTempSensorNr)*10);
    }
    else
    {
      msgData.temperature = (int16_t)(getBmsTempature(u8_mBmsDatasource,0)*10); //Im Fehlerfall immer Senor 0 des BMS nehmen
    }
  }
  else
  {
    msgData.temperature = (int16_t)(getBmsTempature(u8_mBmsDatasource,0)*10); //Im Fehlerfall immer Senor 0 des BMS nehmen
  }


  #ifdef CAN_DEBUG
  BSC_LOGD(TAG, "CAN: current=%i temperature=%i voltage=%i", msgData.current, msgData.temperature, msgData.voltage);
  #endif

  xSemaphoreTake(mInverterDataMutex, portMAX_DELAY);
  inverterData.inverterVoltage = msgData.voltage;
  inverterData.inverterCurrent = msgData.current;
  xSemaphoreGive(mInverterDataMutex);

  if(u8_mMqttTxTimer==15)
  {
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TOTAL_VOLTAGE, -1, (float)(msgData.voltage/100));
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TOTAL_CURRENT, -1, (float)(msgData.current/10));
    mqttPublish(MQTT_TOPIC_INVERTER, -1, MQTT_TOPIC2_TEMPERATURE, -1, (float)(msgData.temperature/10));
  }

  sendCanMsg(0x356, (uint8_t *)&msgData, sizeof(data356));
}


// Send alarm details
void sendCanMsg_359()
{
  data35a msgData;
  uint8_t u8_lValue=0;

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x //bit0  single cell overvoltage protection 
  #define BMS_ERR_STATUS_CELL_UVP          2  x //bit1  single cell undervoltage protection    
  #define BMS_ERR_STATUS_BATTERY_OVP       4  x //bit2  whole pack overvoltage protection 
  #define BMS_ERR_STATUS_BATTERY_UVP       8  x //bit3  Whole pack undervoltage protection     
  #define BMS_ERR_STATUS_CHG_OTP          16  x //bit4  charging over temperature protection 
  #define BMS_ERR_STATUS_CHG_UTP          32  x //bit5  charging low temperature protection 
  #define BMS_ERR_STATUS_DSG_OTP          64  x //bit6  Discharge over temperature protection  
  #define BMS_ERR_STATUS_DSG_UTP         128  x //bit7  discharge low temperature protection   
  #define BMS_ERR_STATUS_CHG_OCP         256  x //bit8  charging overcurrent protection 
  #define BMS_ERR_STATUS_DSG_OCP         512  x //bit9  Discharge overcurrent protection       
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  x //bit10 short circuit protection              
  #define BMS_ERR_STATUS_AFE_ERROR      2048  x //bit11 Front-end detection IC error 
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  x //bit12 software lock MOS 
  #define BMS_ERR_STATUS_RESERVED1      8192  - //bit13 Reserved 
  #define BMS_ERR_STATUS_RESERVED2     16384  - //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  - //bit15 Reserved */


  /*
  Pylontech V1.2

  Data 0 (Alarm)
  0: -
  1: Battery high voltage
  2: Battery low voltage alarm
  3: Battery high temp
  4: Battery low temp
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
  1: Battery high voltage
  2: Battery low voltage alarm
  3: Battery high temp
  4: Battery low temp
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
 
  uint32_t u32_bmsErrors = getBmsErrors(u8_mBmsDatasource);

  msgData.u8_b0=0;
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_OVP)==BMS_ERR_STATUS_CELL_OVP) msgData.u8_b0 |= B00000010;    //1: Battery high voltage
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_UVP)==BMS_ERR_STATUS_CELL_UVP) msgData.u8_b0 |= B00000100;    //2: Battery low voltage alarm
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_OVP)==BMS_ERR_STATUS_BATTERY_OVP) msgData.u8_b0 |= B00000010; //1: Battery high voltage
  if((u32_bmsErrors&BMS_ERR_STATUS_CELL_UVP)==BMS_ERR_STATUS_BATTERY_UVP) msgData.u8_b0 |= B00000100; //2: Battery low voltage alarm

  if((u32_bmsErrors&BMS_ERR_STATUS_CHG_OTP)==BMS_ERR_STATUS_CHG_OTP) msgData.u8_b0 |= B00001000;      //3: Battery high temp
  if((u32_bmsErrors&BMS_ERR_STATUS_CHG_UTP)==BMS_ERR_STATUS_CHG_UTP) msgData.u8_b0 |= B00010000;      //4: Battery low temp
  if((u32_bmsErrors&BMS_ERR_STATUS_DSG_OTP)==BMS_ERR_STATUS_DSG_OTP) msgData.u8_b0 |= B00001000;      //3: Battery high temp
  if((u32_bmsErrors&BMS_ERR_STATUS_DSG_UTP)==BMS_ERR_STATUS_DSG_UTP) msgData.u8_b0 |= B00010000;      //4: Battery low temp

  if((u32_bmsErrors&BMS_ERR_STATUS_DSG_OCP)==BMS_ERR_STATUS_DSG_OCP) msgData.u8_b0 |= B10000000;      //7: Discharge over current 

  msgData.u8_b1=0;
  if((u32_bmsErrors&BMS_ERR_STATUS_CHG_OCP)==BMS_ERR_STATUS_CHG_OCP) msgData.u8_b1 |= B00000001;              //0: Charge high current 
  if((u32_bmsErrors&BMS_ERR_STATUS_SHORT_CIRCUIT)==BMS_ERR_STATUS_SHORT_CIRCUIT) msgData.u8_b1 |= B00001000;  //3: System error
  if((u32_bmsErrors&BMS_ERR_STATUS_AFE_ERROR)==BMS_ERR_STATUS_AFE_ERROR) msgData.u8_b1 |= B00001000;          //3: System error
  if((u32_bmsErrors&BMS_ERR_STATUS_SOFT_LOCK)==BMS_ERR_STATUS_SHORT_CIRCUIT) msgData.u8_b1 |= B00001000;      //3: System error


  //Alarme über Trigger einbinden
  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE)) msgData.u8_b0 |= B00000010;
  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE)) msgData.u8_b0 |= B00000100;
  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE)) msgData.u8_b0 |= B00001000;
  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOWTEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_LOWTEMPERATURE)) msgData.u8_b0 |= B00010000;


  msgData.u8_b2=0;
  msgData.u8_b3=0;
  msgData.u8_b4=0x01; //Pack number (data type : 8bit unsigned char)
  msgData.u8_b5=0x50;
  msgData.u8_b6=0x4E;
  msgData.u8_b7=0;

  sendCanMsg(0x359, (uint8_t *)&msgData, sizeof(data35a));
}


// Send alarm details
void sendCanMsg_35a()
{
  const uint8_t BB0_ALARM = B00000001;
  const uint8_t BB1_ALARM = B00000100;
  const uint8_t BB2_ALARM = B00010000;
  const uint8_t BB3_ALARM = B01000000;

  const uint8_t BB0_OK = B00000010;
  const uint8_t BB1_OK = B00001000;
  const uint8_t BB2_OK = B00100000;
  const uint8_t BB3_OK = B10000000;

  data35a msgData;
  msgData.u8_b0=0;
  msgData.u8_b1=0;
  msgData.u8_b2=0;
  msgData.u8_b3=0;
  msgData.u8_b4=0;
  msgData.u8_b5=0;
  msgData.u8_b6=0;
  msgData.u8_b7=0;

  uint8_t u8_lValue=0;

  /*bmsErrors
  #define BMS_ERR_STATUS_OK                0
  #define BMS_ERR_STATUS_CELL_OVP          1  x //bit0  single cell overvoltage protection 
  #define BMS_ERR_STATUS_CELL_UVP          2  x //bit1  single cell undervoltage protection    
  #define BMS_ERR_STATUS_BATTERY_OVP       4  x //bit2  whole pack overvoltage protection 
  #define BMS_ERR_STATUS_BATTERY_UVP       8  x //bit3  Whole pack undervoltage protection     
  #define BMS_ERR_STATUS_CHG_OTP          16  x //bit4  charging over temperature protection 
  #define BMS_ERR_STATUS_CHG_UTP          32  x //bit5  charging low temperature protection 
  #define BMS_ERR_STATUS_DSG_OTP          64  x //bit6  Discharge over temperature protection  
  #define BMS_ERR_STATUS_DSG_UTP         128  x //bit7  discharge low temperature protection   
  #define BMS_ERR_STATUS_CHG_OCP         256  x //bit8  charging overcurrent protection 
  #define BMS_ERR_STATUS_DSG_OCP         512  x //bit9  Discharge overcurrent protection       
  #define BMS_ERR_STATUS_SHORT_CIRCUIT  1024  x //bit10 short circuit protection              
  #define BMS_ERR_STATUS_AFE_ERROR      2048  x //bit11 Front-end detection IC error 
  #define BMS_ERR_STATUS_SOFT_LOCK      4096  x //bit12 software lock MOS 
  #define BMS_ERR_STATUS_RESERVED1      8192  - //bit13 Reserved 
  #define BMS_ERR_STATUS_RESERVED2     16384  - //bit14 Reserved
  #define BMS_ERR_STATUS_RESERVED3     32768  - //bit15 Reserved */


  //msgData.u8_b0 |= BB0_ALARM; //n.b.
  //msgData.u8_b0 |= BB1_ALARM; //High battery voltage
  //msgData.u8_b0 |= BB2_ALARM; //Low battery voltage
  //msgData.u8_b0 |= BB3_ALARM; //High Temperature

  //msgData.u8_b1 |= BB0_ALARM; //Low Temperature
  //msgData.u8_b1 |= BB1_ALARM; //High charge Temperature
  //msgData.u8_b1 |= BB2_ALARM; //Low charge Temperature
  //msgData.u8_b1 |= BB3_ALARM; //High discharge current

  //msgData.u8_b2 |= BB0_ALARM; //High charge current
  //msgData.u8_b2 |= BB1_ALARM; //n.b.
  //msgData.u8_b2 |= BB2_ALARM; //n.b.
  //msgData.u8_b2 |= BB3_ALARM; //Internal failure

  //msgData.u8_b3 |= BB0_ALARM; // Cell imbalance
  //msgData.u8_b3 |= BB1_ALARM; //n.b.
  //msgData.u8_b3 |= BB2_ALARM; //n.b.
  //msgData.u8_b3 |= BB3_ALARM; //n.b.

  uint32_t u32_bmsErrors = getBmsErrors(u8_mBmsDatasource);
  //BSC_LOGI(TAG,"u8_mBmsDatasource=%i, u32_bmsErrors=%i",u8_mBmsDatasource,u32_bmsErrors);

  // 0 (bit 0+1) n.b.
  msgData.u8_b0 |= BB0_OK;
    
  // 0 (bit 2+3) Battery high voltage alarm
  msgData.u8_b0 |= (((u32_bmsErrors&BMS_ERR_STATUS_BATTERY_OVP)==BMS_ERR_STATUS_BATTERY_OVP) ||
    ((u32_bmsErrors&BMS_ERR_STATUS_CELL_OVP)==BMS_ERR_STATUS_CELL_OVP))? BB1_ALARM : BB1_OK;

  // 0 (bit 4+5) Battery low voltage alarm
  msgData.u8_b0 |= (((u32_bmsErrors&BMS_ERR_STATUS_BATTERY_UVP)==BMS_ERR_STATUS_BATTERY_UVP) ||
    ((u32_bmsErrors&BMS_ERR_STATUS_CELL_UVP)==BMS_ERR_STATUS_CELL_UVP)) ? BB2_ALARM : BB2_OK;

  // 0 (bit 6+7) Battery high temperature alarm
  msgData.u8_b0 |= ((u32_bmsErrors&BMS_ERR_STATUS_DSG_OTP)==BMS_ERR_STATUS_DSG_OTP) ? BB3_ALARM : BB3_OK;

  // 1 (bit 0+1) Battery low temperature alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_DSG_UTP)==BMS_ERR_STATUS_DSG_UTP) ? BB0_ALARM : BB0_OK;

  // 1 (bit 2+3) Battery high temperature charge alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_CHG_OTP)==BMS_ERR_STATUS_CHG_OTP) ? BB1_ALARM : BB1_OK;

  // 1 (bit 4+5) Battery low temperature charge alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_CHG_UTP)==BMS_ERR_STATUS_CHG_UTP) ? BB2_ALARM : BB2_OK;

  // 1 (bit 6+7) Battery high discharge current alarm
  msgData.u8_b1 |= ((u32_bmsErrors&BMS_ERR_STATUS_DSG_OCP)==BMS_ERR_STATUS_DSG_OCP) ? BB3_ALARM : BB3_OK;
  
  // 2 (bit 0+1) Battery high charge current alarm
  msgData.u8_b2 |= ((u32_bmsErrors&BMS_ERR_STATUS_CHG_OCP)==BMS_ERR_STATUS_CHG_OCP) ? BB0_ALARM : BB0_OK;

  // 2 (bit 2+3) Contactor Alarm (not implemented)
  msgData.u8_b2 |= BB1_OK;

  // 2 (bit 4+5) Short circuit Alarm (not implemented)
  msgData.u8_b2 |= BB2_OK;

  // 2 (bit 6+7) BMS internal alarm
  msgData.u8_b2 |= (((u32_bmsErrors&BMS_ERR_STATUS_AFE_ERROR)==BMS_ERR_STATUS_AFE_ERROR) || 
    ((u32_bmsErrors&BMS_ERR_STATUS_SHORT_CIRCUIT)==BMS_ERR_STATUS_SHORT_CIRCUIT) || 
    ((u32_bmsErrors&BMS_ERR_STATUS_SOFT_LOCK)==BMS_ERR_STATUS_SOFT_LOCK)) ? BB3_ALARM : BB3_OK;

  // 3 (bit 0+1) Cell imbalance alarm
  // 3 (bit 2+3) n.b.
  // 3 (bit 4+5) n.b.
  // 3 (bit 6+7) n.b.


  //Warnings
  // 4 (bit 0+1) n.b.
  // 4 (bit 2+3) Battery high voltage warning
  // 4 (bit 4+5) Battery low voltage warning
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
  // 7 (bit 2+3) System status (online/offline)
  // 7 (bit 4+5) n.b.
  // 7 (bit 6+7) n.b.


  //Alarme über Trigger einbinden
  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE))
  {
    msgData.u8_b0 &= ~(BB1_OK);
    msgData.u8_b0 |= BB1_ALARM;
  }

  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE,0,DT_ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE))
  {
    msgData.u8_b0 &= ~(BB2_OK);
    msgData.u8_b0 |= BB2_ALARM;
  }

  if(isTriggerActive(ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE))
  {
    msgData.u8_b0 &= ~(BB3_OK);
    msgData.u8_b0 |= BB3_ALARM;
  }

  if(isTriggerActive(ID_PARAM_BMS_ALARM_LOWTEMPERATURE,0,DT_ID_PARAM_BMS_ALARM_LOWTEMPERATURE))
  {
    msgData.u8_b1 &= ~(BB0_OK);
    msgData.u8_b1 |= BB0_ALARM;
  }


  //BSC_LOGI(TAG,"0x35a=%i,%i,%i,%i,%i,%i,%i,%i",msgData.u8_b0,msgData.u8_b1,msgData.u8_b2,msgData.u8_b3,msgData.u8_b4,msgData.u8_b5,msgData.u8_b6,msgData.u8_b7);
  sendCanMsg(0x35a, (uint8_t *)&msgData, sizeof(data35a));
}


void sendCanMsg_35f()
{
  struct data35f
  {
    uint16_t BatteryModel;
    uint16_t Firmwareversion;
    uint16_t Onlinecapacity;
  };

  data35f msgData;

  msgData.BatteryModel = 0;
  msgData.Firmwareversion = 3;
  msgData.Onlinecapacity = 0;

  sendCanMsg(0x35f, (uint8_t *)&msgData, sizeof(data35f));
}


void sendCanMsg_372()
{
  struct data372
  {
    uint16_t numberofmodulesok;
    uint16_t numberofmodulesblockingcharge;
    uint16_t numberofmodulesblockingdischarge;
    // uint16_t numberofmodulesoffline;
  };
  data372 msgData;

  msgData.numberofmodulesok = getNumberOfBatteryModules();
  msgData.numberofmodulesblockingcharge = getNumberOfBatteryModules()-getNumberOfBatteryModulesCharge();
  msgData.numberofmodulesblockingdischarge = getNumberOfBatteryModules()-getNumberOfBatteryModulesDischarge();
  //msgData.numberofmodulesoffline = 0;

  sendCanMsg(0x372, (uint8_t *)&msgData, sizeof(data372));
}


void sendCanMsg_373_376_377()
{
  data373 msgData;

  msgData.maxCellVoltage = getMaxCellSpannungFromBms();
  msgData.minCellColtage = getMinCellSpannungFromBms();

  if(getBmsTempature(u8_mBmsDatasource,1)>getBmsTempature(u8_mBmsDatasource,2))
  {
    msgData.minCellTemp = 273 + getBmsTempature(u8_mBmsDatasource,2);
    msgData.maxCellTemp = 273 + getBmsTempature(u8_mBmsDatasource,1);
  }
  else
  {
    msgData.minCellTemp = 273 + getBmsTempature(u8_mBmsDatasource,1);
    msgData.maxCellTemp = 273 + getBmsTempature(u8_mBmsDatasource,2);
  }

  sendCanMsg(0x373, (uint8_t *)&msgData, sizeof(data373));

  //sendCanMsg(0x376, (uint8_t *)&msgData, 8); //lowestExternalTemp
  //sendCanMsg(0x377, (uint8_t *)&msgData, 8); //highestExternalTemp
}



void sendCanMsgTemp()
{
  uint32_t u16_lCanId = 0x380;

  struct dataTemp
  {
    uint16_t temperature[4];
  };
  dataTemp msgData;

  for(uint8_t i=0;i<64;i+=4)
  {
    for(uint8_t n=0;n<4;n++) msgData.temperature[n] = (uint16_t)(owGetTemp(i+n)*100);
    sendCanMsg(u16_lCanId, (uint8_t *)&msgData, sizeof(dataTemp));
    u16_lCanId++;
  }
}

void sendCanMsgBmsData()
{
  uint32_t u16_lBaseCanId = 0x400;
  uint32_t u16_lCanId=u16_lBaseCanId;
  static uint8_t u8_mCanSendDataBmsNumber=0;

  struct dataCellVoltage
  {
    uint16_t cellVoltage[4];
  };
  dataCellVoltage msgData;

  struct dataBms1
  {
    bool stateFETCharge;
    bool stateFETDischarge;
    bool stateBalancingActive;
  };
  dataBms1 msgDataBms1;

  struct dataBms2
  {
    int16_t value1;
  };
  dataBms2 msgDataBms2;


  u16_lCanId=u16_lBaseCanId+(0x32*u8_mCanSendDataBmsNumber);

  //Cellvoltage
  for(uint8_t n=0;n<16;n+=4)
  {
    for(uint8_t x=0;x<4;x++) msgData.cellVoltage[x] = getBmsCellVoltage(u8_mCanSendDataBmsNumber,n+x);
    sendCanMsg(u16_lCanId, (uint8_t *)&msgData, sizeof(dataCellVoltage));
    u16_lCanId++;
  }

  msgDataBms1.stateFETCharge = getBmsStateFETsCharge(u8_mCanSendDataBmsNumber);
  msgDataBms1.stateFETDischarge = getBmsStateFETsDischarge(u8_mCanSendDataBmsNumber);
  msgDataBms1.stateBalancingActive = getBmsIsBalancingActive(u8_mCanSendDataBmsNumber);
  sendCanMsg(u16_lCanId, (uint8_t *)&msgDataBms1, sizeof(dataBms1));
  u16_lCanId++;
  
  msgDataBms2.value1 = (int16_t)(getBmsBalancingCurrent(u8_mCanSendDataBmsNumber)*100);
  sendCanMsg(u16_lCanId, (uint8_t *)&msgDataBms2, sizeof(dataBms2));
  u16_lCanId++;

  u8_mCanSendDataBmsNumber++;
  if(u8_mCanSendDataBmsNumber==BMSDATA_NUMBER_ALLDEVICES)u8_mCanSendDataBmsNumber=0;
}


void onCanReceive(int packetSize)
{
  u32_lastCanId = CAN.packetId();

  if (CAN.packetRtr())
  {
    //requested length
  }
  else
  {
    uint8_t i=0;
    uint8_t u8_canPacket[8];
    while (CAN.available())
    {
      u8_canPacket[i]=(uint8_t)CAN.read();
      i++;
      if(i==8)break;
    }

    if(CAN.packetId()==0x02F4)
    {
      can_batVolt = (uint16_t)u8_canPacket[1]<<8 | u8_canPacket[0];

      can_batCurr = ((int16_t)u8_canPacket[3]<<8 | u8_canPacket[2])-4000;
      if (can_batCurr&0x8000){can_batCurr=(can_batCurr&0x7fff);}
      else {can_batCurr*=-1;} // Wenn negativ

      can_soc = u8_canPacket[4];
    }
    else if(CAN.packetId()==0x04F4)
    {

    }
    else if(CAN.packetId()==0x05F4)
    {

    }
  }
}

