// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BscSerial.h"
#include "defines.h"
#include "WebSettings.h"
#include "BmsData.h"
#include "log.h"
#include "dio.h"
#include "i2c.h"

//include Devices
#include "devices/serialDevData.h"
#include "devices/JbdBms.h"
#include "devices/JkBms.h"
#include "devices/SeplosBms.h"
#include "devices/DalyBms.h"

static const char *TAG = "BSC_SERIAL";

struct serialDeviceData_s
{
  bool (*readBms)(Stream*, uint8_t, void (*callback)(uint8_t, uint8_t), serialDevData_s*) = 0;
  Stream * stream_mPort;

  uint32_t u32_baudrate;
  uint8_t u8_mAddData;
  uint8_t u8_mFilterBmsCellVoltageMaxCount=0;
};

struct serialDeviceData_s serialDeviceData[SERIAL_BMS_DEVICES_COUNT];


#ifdef UTEST_BMS_FILTER
bool readBmsTestData(uint8_t devNr);
#endif


BscSerial::BscSerial()
{

}

void BscSerial::initSerial()
{
  mSerialMutex = xSemaphoreCreateMutex();

  pinMode(SERIAL3_PIN_TX_EN, OUTPUT);  //0
  pinMode(SERIAL1_PIN_TX_EN, OUTPUT);  //1
  pinMode(SERIAL2_PIN_TX_EN, OUTPUT);  //2
  pinMode(SERIAL2_PIN_RX_EN, OUTPUT);  //2

  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    serialDeviceData[i].u8_mAddData=0;
    serialDeviceData[i].u8_mFilterBmsCellVoltageMaxCount=0;
    serialDeviceData[i].u32_baudrate=9600;

    #ifdef DEBUG_ON_HW_SERIAL
    if(i>=2) setSoftSerial(i,serialDeviceData[i].u32_baudrate);
    #endif

    setSerialBaudrate(i);
    uint8_t funktionsTyp = WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,i,DT_ID_PARAM_SERIAL_CONNECT_DEVICE);
    BSC_LOGI(TAG, "initSerial SerialNr=%i, funktionsTyp=%i",i,funktionsTyp);
    setReadBmsFunktion(i, funktionsTyp);
  }
}


void BscSerial::setHwSerial(uint8_t u8_devNr, uint32_t baudrate)
{
  if(u8_devNr>=2) // Hw Serial 0
  {
    Serial.end();
    Serial.begin(baudrate,SERIAL_8N1,SERIAL3_PIN_RX,SERIAL3_PIN_TX);
    serialDeviceData[u8_devNr].stream_mPort=&Serial;
  }
  else if(u8_devNr==0) 
  {
    Serial1.end();
    Serial1.begin(baudrate,SERIAL_8N1,SERIAL1_PIN_RX,SERIAL1_PIN_TX);
    serialDeviceData[u8_devNr].stream_mPort=&Serial1;
  }
  else if(u8_devNr==1) // Hw Serial 2
  {
    Serial2.end();
    Serial2.begin(baudrate,SERIAL_8N1,SERIAL2_PIN_RX,SERIAL2_PIN_TX);
    serialDeviceData[u8_devNr].stream_mPort=&Serial2;
  }
}


void BscSerial::setSoftSerial(uint8_t u8_devNr, uint32_t baudrate) // Sw Serial
{
  static SoftwareSerial mySwSerial(SERIAL3_PIN_RX,SERIAL3_PIN_TX,false);
  serialDeviceData[u8_devNr].stream_mPort = &mySwSerial;
  static_cast<SoftwareSerial*>(serialDeviceData[u8_devNr].stream_mPort)->begin(baudrate);
}


void BscSerial::setSerialBaudrate(uint8_t u8_devNr, uint32_t baudrate)
{
  serialDeviceData[u8_devNr].u32_baudrate = baudrate;

  if(u8_devNr==0)
  {
    Serial1.end();
    setHwSerial(u8_devNr, baudrate);
  }
  else if(u8_devNr==1)
  {
    Serial2.end();
    setHwSerial(u8_devNr, baudrate);
  }
  //#ifndef DEBUG_ON_HW_SERIAL
  else if(u8_devNr>=2)
  {
    #ifndef DEBUG_ON_HW_SERIAL
    Serial.end();
    setHwSerial(u8_devNr, baudrate);
    #else
    static_cast<SoftwareSerial*>(serialDeviceData[u8_devNr].stream_mPort)->end();
    setSoftSerial(u8_devNr, baudrate);
    #endif
  }
  //#endif
}

void BscSerial::setSerialBaudrate(uint8_t u8_devNr)
{
  setSerialBaudrate(u8_devNr, serialDeviceData[u8_devNr].u32_baudrate);
}


void BscSerial::setReadBmsFunktion(uint8_t u8_devNr, uint8_t funktionsTyp)
{
  serialDeviceData[u8_devNr].u8_mFilterBmsCellVoltageMaxCount = WebSettings::getIntFlash(ID_PARAM_BMS_FILTER_RX_ERROR_COUNT,0,DT_ID_PARAM_BMS_FILTER_RX_ERROR_COUNT);

  xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  serialDeviceData[u8_devNr].u8_mAddData=0;

  switch (funktionsTyp)
  {
    case ID_SERIAL_DEVICE_NB:
      serialDeviceData[u8_devNr].readBms = 0;
      break;

    case ID_SERIAL_DEVICE_JBDBMS:
      BSC_LOGI(TAG,"setReadBmsFunktion JBD-BMS");
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &JbdBms_readBmsData;
      break;

    case ID_SERIAL_DEVICE_JKBMS:
      BSC_LOGI(TAG,"setReadBmsFunktion JK-BMS");
      setSerialBaudrate(u8_devNr, 115200);
      serialDeviceData[u8_devNr].readBms = &JkBms_readBmsData;
      break;
      
    case ID_SERIAL_DEVICE_SEPLOSBMS:
      BSC_LOGI(TAG,"setReadBmsFunktion SEPLOS");
      setSerialBaudrate(u8_devNr, 19200);
      serialDeviceData[u8_devNr].readBms = &SeplosBms_readBmsData;
      serialDeviceData[u8_devNr].u8_mAddData=1;
      BSC_LOGI(TAG,"adrData (A): devNr=%i, adr=%i",u8_devNr, serialDeviceData[u8_devNr].u8_mAddData);
      break;
      
    case ID_SERIAL_DEVICE_DALYBMS:
      BSC_LOGI(TAG,"setReadBmsFunktion DALY");
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &DalyBms_readBmsData;
      break;
   
    default:
      serialDeviceData[u8_devNr].readBms = 0;
  }

  if(u8_devNr==2)
  {
    if(WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,u8_devNr,DT_ID_PARAM_SERIAL_CONNECT_DEVICE)==ID_SERIAL_DEVICE_SEPLOSBMS)
    {
      serialDeviceData[u8_devNr].u8_mAddData=WebSettings::getInt(ID_PARAM_SERIAL_SEPLOS_CONNECT_TO_ID,0,DT_ID_PARAM_SERIAL_SEPLOS_CONNECT_TO_ID);
      BSC_LOGI(TAG,"adrData (B): devNr=%i, adr=%i",u8_devNr, serialDeviceData[u8_devNr].u8_mAddData);
    }
  }
  xSemaphoreGive(mSerialMutex);
}


/*
 * Callback
 */
//serialRxTxEn_e {serialRxTx_RxTxDisable, serialRxTx_TxEn, serialRxTx_RxEn};
void cbSetRxTxEn(uint8_t u8_devNr, uint8_t e_rw)
{
  //BSC_LOGI(TAG, "RxTxEnCallback dev=%i, rw=%i", u8_devNr, e_rw);

  if(u8_devNr==0)
  {
    if(e_rw==serialRxTx_TxEn)
    {
      digitalWrite(SERIAL3_PIN_TX_EN, HIGH); 
      usleep(20);
    }
    else if(e_rw==serialRxTx_RxEn) digitalWrite(SERIAL3_PIN_TX_EN, LOW); 
  }
  else if(u8_devNr==1) 
  {
    if(e_rw==serialRxTx_TxEn)
    {
      digitalWrite(SERIAL1_PIN_TX_EN, HIGH); 
      usleep(20);
    }
    else if(e_rw==serialRxTx_RxEn) digitalWrite(SERIAL1_PIN_TX_EN, LOW); 
  }
  else if(u8_devNr==2)
  {
    if(e_rw==serialRxTx_RxTxDisable) //RX + TX aus
    {
      if(getHwVersion()>=2) digitalWrite(SERIAL2_PIN_RX_EN, HIGH);
      digitalWrite(SERIAL2_PIN_TX_EN, LOW);
    }
    else if(e_rw==serialRxTx_TxEn) //RX
    {
      if(getHwVersion()>=2) digitalWrite(SERIAL2_PIN_RX_EN, LOW); 
      digitalWrite(SERIAL2_PIN_TX_EN, HIGH); 
      usleep(20);
    }
    else if(e_rw==serialRxTx_RxEn) //TX
    {
      if(getHwVersion()>=2) digitalWrite(SERIAL2_PIN_RX_EN, HIGH); 
      digitalWrite(SERIAL2_PIN_TX_EN, LOW); 
    }
  }
  else if(u8_devNr>2 && u8_devNr<=10 && getHwVersion()>=2)
  {
    i2cExtSerialSetEnable(u8_devNr-3, (serialRxTxEn_e)e_rw);
  }
}


void BscSerial::cyclicRun()
{
  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {  
    if(serialDeviceData[i].readBms==0) //Wenn nicht Initialisiert
    {
      setSerialBmsWriteData(i,false);
      continue;
    }

    bool    bo_lBmsReadOk=false;
    uint8_t u8_lReason=1;

    if(i>=2) setSerialBaudrate(i); //Baudrate wechslen

    uint8_t *u8_pBmsFilterErrorCounter = getBmsFilterErrorCounter(BT_DEVICES_COUNT+i);

    xSemaphoreTake(mSerialMutex, portMAX_DELAY);
    *u8_pBmsFilterErrorCounter &= ~(0x80); //Fehlermerker des aktuellen Durchgangs löschen (bit 0)
    #ifndef UTEST_BMS_FILTER
    serialDevData_s devData;
    devData.u8_addData=serialDeviceData[i].u8_mAddData;
    devData.bo_writeData=getSerialBmsWriteData(i); //Hier true setzen, wenn Daten gschrieben werden sollen
    BSC_LOGI(TAG,"adrData (C): devNr=%i, adr=%i, adr2=%i",i, serialDeviceData[i].u8_mAddData, devData.u8_addData);
    bo_lBmsReadOk=serialDeviceData[i].readBms(serialDeviceData[i].stream_mPort, i, &cbSetRxTxEn, &devData); //Wenn kein Fehler beim Holen der Daten vom BMS 
    setSerialBmsWriteData(i,false);
    #else
    bmsReadOk=readBmsTestData(BT_DEVICES_COUNT+u8_mSerialNr);
    BSC_LOGI(TAG,"Filter: RX serial Data; errCnt=%i",*u8_pBmsFilterErrorCounter);
    #endif
    if((*u8_pBmsFilterErrorCounter>>7)) //Wenn beim Empfang Fehler wahren
    {
      //BSC_LOGI(TAG,"Filter: RX Error; errCnt=%i",*u8_pBmsFilterErrorCounter);
      if((*u8_pBmsFilterErrorCounter&0x7F)>=serialDeviceData[i].u8_mFilterBmsCellVoltageMaxCount) //Wenn Fehler
      {
        //Zu häufig Fehler
        bo_lBmsReadOk=false;
        u8_lReason=2;
        //BSC_LOGI(TAG,"Filter: Zu viele Errors ist=%i, max=%i", (*u8_pBmsFilterErrorCounter&0x7F), u8_mFilterBmsCellVoltageMaxCount);
      }
      else
      {
        if((*u8_pBmsFilterErrorCounter&0x7F)<125) *u8_pBmsFilterErrorCounter=*u8_pBmsFilterErrorCounter+1;
        //BSC_LOGI(TAG,"Filter: ErrCount ist=%i, max=%i", (*u8_pBmsFilterErrorCounter&0x7F), u8_mFilterBmsCellVoltageMaxCount);
      }
    }
    else
    {
      if(*u8_pBmsFilterErrorCounter>0) BSC_LOGI(TAG,"Filter: Reset RX Error");
      *u8_pBmsFilterErrorCounter = 0; //Fehler Counter zurücksetzen
    }
    xSemaphoreGive(mSerialMutex);
    if(bo_lBmsReadOk)
    {
      setBmsLastDataMillis(BT_DEVICES_COUNT+i,millis());
    }
    else
    {
      String str_lReaseon="";
      if(u8_lReason=1) str_lReaseon=F("Cheksum wrong");
      else str_lReaseon=F("Filter");
      BSC_LOGE(TAG,"Error: device=%i, reason=%s",i,str_lReaseon.c_str());
    }
  }
}







  #ifdef UTEST_BMS_FILTER
  static uint16_t u16_filterTestErrCnt=0;
  bool readBmsTestData(uint8_t devNr)
  {
    BSC_LOGI(TAG,"readBmsTestData: u16_filterTestErrCnt=%i", u16_filterTestErrCnt);

    for(uint8_t i=0;i<24;i++)
    {
      setBmsCellVoltage(devNr,i,3450);
    }

    if(u16_filterTestErrCnt>5 && u16_filterTestErrCnt<30)
    {
      setBmsCellVoltage(devNr,3,6900);
    }


    u16_filterTestErrCnt++;
    return true;
  }
  #endif
