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
#include "crc.h"

//include Devices
#include "devices/serialDevData.h"
#include "devices/JbdBms.h"
#include "devices/JkBms.h"
#include "devices/SeplosBms.h"
#include "devices/SylcinBms.h"
#include "devices/DalyBms.h"
#include "devices/JkBmsV13.h"
#include "devices/GobelBms.h"
#include "devices/SmartShunt.h"
#include "devices/GobelBms_PC200.h"
#ifdef BPN
#include "devices/bpnSerial.h"
#endif

static const char *TAG = "BSC_SERIAL";

uint32_t serialMqttSendeTimer;

struct serialDeviceData_s
{
  bool (*readBms)(Stream*, uint8_t, void (*callback)(uint8_t, uint8_t), serialDevData_s*) = 0;
  Stream * stream_mPort;

  uint32_t u32_baudrate;
  uint8_t u8_mFilterBmsCellVoltageMaxCount=0;
};
struct serialDeviceData_s serialDeviceData[SERIAL_BMS_DEVICES_COUNT];

void cbSetRxTxEn(uint8_t u8_devNr, uint8_t e_rw);

#ifdef UTEST_BMS_FILTER
bool readBmsTestData(uint8_t devNr);
#endif


BscSerial::BscSerial()
{

}

void BscSerial::initSerial()
{
  mSerialMutex = xSemaphoreCreateMutex();

  pinMode(SERIAL1_PIN_TX_EN, OUTPUT);  //HW serial0
  pinMode(SERIAL2_PIN_TX_EN, OUTPUT);  //HW serial1
  if(getHwVersion()>=2) pinMode(SERIAL3_PIN_TX_EN, OUTPUT);  //HW serial2
  pinMode(SERIAL3_PIN_RX_EN, OUTPUT);  //HW serial2

  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    serialDeviceData[i].u8_mFilterBmsCellVoltageMaxCount=0;
    serialDeviceData[i].u32_baudrate=9600;

    #ifdef DEBUG_ON_HW_SERIAL
    if(i==0) setSoftSerial(i,serialDeviceData[i].u32_baudrate);
    #endif

    //setSerialBaudrate(i);
    uint8_t funktionsTyp = WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,i,DT_ID_PARAM_SERIAL_CONNECT_DEVICE);
    BSC_LOGI(TAG, "initSerial SerialNr=%i, funktionsTyp=%i",i,funktionsTyp);
    setReadBmsFunktion(i, funktionsTyp);
  }

  serialMqttSendeTimer=millis();
}

void BscSerial::stopCyclicRun(bool state)
{
  if(state==true) 
  {
    xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  }
  else 
  {
    xSemaphoreGive(mSerialMutex);
  }
}

void BscSerial::setHwSerial(uint8_t u8_devNr, uint32_t baudrate)
{
  //BSC_LOGI(TAG,"setHwSerial() devNr=%i, baudrate=%i",u8_devNr,baudrate);

  if(u8_devNr==0) // Hw Serial 1
  {
    Serial.end();
    Serial.begin(baudrate,SERIAL_8N1,SERIAL1_PIN_RX,SERIAL1_PIN_TX);
        serialDeviceData[u8_devNr].stream_mPort=&Serial;
  }
  else if(u8_devNr==1) // Hw Serial 2
  {
    Serial1.end();
    Serial1.begin(baudrate,SERIAL_8N1,SERIAL2_PIN_RX,SERIAL2_PIN_TX);
        serialDeviceData[u8_devNr].stream_mPort=&Serial1;
  }
  else if(u8_devNr==2) // Hw Serial 0
  {
    Serial2.end();
    Serial2.begin(baudrate,SERIAL_8N1,SERIAL3_PIN_RX,SERIAL3_PIN_TX);
        serialDeviceData[u8_devNr].stream_mPort=&Serial2;
  }
  else if(u8_devNr>2 && isSerialExtEnabled()) // Hw Serial 0
  {
    Serial2.end();
    Serial2.begin(baudrate,SERIAL_8N1,SERIAL3_PIN_RX,SERIAL3_PIN_TX);
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
    #ifndef DEBUG_ON_HW_SERIAL
    setHwSerial(u8_devNr, baudrate);
    #else
    static_cast<SoftwareSerial*>(serialDeviceData[u8_devNr].stream_mPort)->end();
    setSoftSerial(u8_devNr, baudrate);
    #endif
  }
  else if(u8_devNr==1)
  {
    setHwSerial(u8_devNr, baudrate);
  }
  else if(u8_devNr>=2)
  {
    setHwSerial(u8_devNr, baudrate);
  }
}

void BscSerial::setSerialBaudrate(uint8_t u8_devNr)
{
  setSerialBaudrate(u8_devNr, serialDeviceData[u8_devNr].u32_baudrate);
}


void BscSerial::setReadBmsFunktion(uint8_t u8_devNr, uint8_t funktionsTyp)
{
  serialDeviceData[u8_devNr].u8_mFilterBmsCellVoltageMaxCount = WebSettings::getIntFlash(ID_PARAM_BMS_FILTER_RX_ERROR_COUNT,0,DT_ID_PARAM_BMS_FILTER_RX_ERROR_COUNT);

  //xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  
  switch (funktionsTyp)
  {
    case ID_SERIAL_DEVICE_NB:
      if(u8_devNr==2) cbSetRxTxEn(u8_devNr, serialRxTx_RxTxDisable);
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = 0;
      break;

    case ID_SERIAL_DEVICE_JBDBMS:
      BSC_LOGI(TAG,"Set serial device %i: JBD-BMS",u8_devNr);
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &JbdBms_readBmsData;
      break;

    case ID_SERIAL_DEVICE_JKBMS:
      BSC_LOGI(TAG,"Set serial device %i: JK-BMS",u8_devNr);
      setSerialBaudrate(u8_devNr, 115200);
      serialDeviceData[u8_devNr].readBms = &JkBms_readBmsData;
      break;
      
    case ID_SERIAL_DEVICE_SEPLOSBMS:
      BSC_LOGI(TAG,"Set serial device %i: SEPLOS",u8_devNr);
      setSerialBaudrate(u8_devNr, 19200);
      serialDeviceData[u8_devNr].readBms = &SeplosBms_readBmsData;
      break;

    case ID_SERIAL_DEVICE_DALYBMS:
      BSC_LOGI(TAG,"Set serial device %i: DALY",u8_devNr);
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &DalyBms_readBmsData;
      break;

    case ID_SERIAL_DEVICE_SYLCINBMS:
      BSC_LOGI(TAG,"Set serial device %i: SYLCIN",u8_devNr);
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &SylcinBms_readBmsData;
      break;

    case ID_SERIAL_DEVICE_JKBMS_V13:
      BSC_LOGI(TAG,"Set serial device %i: JKBMS V1.3",u8_devNr);
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &JkBmsV13_readBmsData;
      break;      

    case ID_SERIAL_DEVICE_GOBELBMS:
      BSC_LOGI(TAG,"setReadBmsFunktion Gobel RN150");
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &GobelBms_readBmsData;
      break; 

    #ifdef BPN
    case ID_SERIAL_DEVICE_BPN:
      ESP_LOGI(TAG,"setReadBmsFunktion BPN");
      setSerialBaudrate(u8_devNr, 19200);
      serialDeviceData[u8_devNr].readBms = &bpn_readBmsData;
      break;
    #endif

    case ID_SERIAL_DEVICE_SMARTSHUNT_VEDIRECT:
      ESP_LOGI(TAG,"setReadBmsFunktion SmartShunt");
      setSerialBaudrate(u8_devNr, 19200);
      serialDeviceData[u8_devNr].readBms = &SmartShunt_readBmsData;
      break;

    case ID_SERIAL_DEVICE_GOBEL_PC200:
      BSC_LOGI(TAG,"setReadBmsFunktion Gobel PC200");
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &GobelBmsPC200_readBmsData;
      break;

    default:
      serialDeviceData[u8_devNr].readBms = 0;
  }

  //xSemaphoreGive(mSerialMutex);
}


/*
 * Callback
 */
//serialRxTxEn_e {serialRxTx_RxTxDisable, serialRxTx_TxEn, serialRxTx_RxEn};
void cbSetRxTxEn(uint8_t u8_devNr, uint8_t e_rw)
{
  if(u8_devNr==0)
  {
    if(e_rw==serialRxTx_TxEn)
    {
      digitalWrite(SERIAL1_PIN_TX_EN, HIGH); 
      usleep(20);
    }
    else if(e_rw==serialRxTx_RxEn) digitalWrite(SERIAL1_PIN_TX_EN, LOW); 
  }
  else if(u8_devNr==1) 
  {
    if(e_rw==serialRxTx_TxEn)
    {
      digitalWrite(SERIAL2_PIN_TX_EN, HIGH); 
      usleep(20);
    }
    else if(e_rw==serialRxTx_RxEn) digitalWrite(SERIAL2_PIN_TX_EN, LOW); 
  }
  else if(u8_devNr==2)
  {
    if(e_rw==serialRxTx_RxTxDisable) //RX + TX aus
    {
      if(getHwVersion()<2)
      {
        digitalWrite(SERIAL3_PIN_RX_EN, LOW);
      }
      else
      {
        digitalWrite(SERIAL3_PIN_RX_EN, HIGH);  //32=RXTX_EN; 3=TX_EN
        digitalWrite(SERIAL3_PIN_TX_EN, LOW);
      }
    }
    else if(e_rw==serialRxTx_TxEn) //TX
    {
      digitalWrite(SERIAL3_PIN_RX_EN, HIGH); //LOW
      if(getHwVersion()>=2) digitalWrite(SERIAL3_PIN_TX_EN, HIGH); //HIGH
      usleep(20);
    }
    else if(e_rw==serialRxTx_RxEn) //RX
    {
      digitalWrite(SERIAL3_PIN_RX_EN, LOW); 
      if(getHwVersion()>=2) digitalWrite(SERIAL3_PIN_TX_EN, LOW); //LOW
    }
  }
  else if(u8_devNr>2 && u8_devNr<=10 && getHwVersion()>=2)
  {
    i2cExtSerialSetEnable(u8_devNr-3, (serialRxTxEn_e)e_rw);
  }
}


void BscSerial::cyclicRun()
{
  xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  bool bo_lMqttSendMsg=false;
  uint8_t u8_lNumberOfSeplosBms = 0;
  uint8_t u8_lBmsOnSerial2 = (uint8_t)WebSettings::getInt(ID_PARAM_SERIAL_CONNECT_DEVICE,2,DT_ID_PARAM_SERIAL_CONNECT_DEVICE);
  if(u8_lBmsOnSerial2==ID_SERIAL_DEVICE_SEPLOSBMS || u8_lBmsOnSerial2==ID_SERIAL_DEVICE_SYLCINBMS || 
      u8_lBmsOnSerial2==ID_SERIAL_DEVICE_GOBELBMS || u8_lBmsOnSerial2==ID_SERIAL_DEVICE_GOBEL_PC200)
  {
    if(isSerialExtEnabled()) u8_lNumberOfSeplosBms=0;
    else u8_lNumberOfSeplosBms=WebSettings::getInt(ID_PARAM_SERIAL2_CONNECT_TO_ID,0,DT_ID_PARAM_SERIAL2_CONNECT_TO_ID);
  }

  if((millis()-serialMqttSendeTimer)>60000) 
  {
    serialMqttSendeTimer=millis();
    bo_lMqttSendMsg=true;
  }

  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {  
    if(serialDeviceData[i].readBms==0) //Wenn nicht Initialisiert
    {
      if(u8_lNumberOfSeplosBms==0 || i<2 || i>(u8_lNumberOfSeplosBms+1))
      {  
        continue;
      }
    }

    bool    bo_lBmsReadOk=false;
    uint8_t u8_lReason=1;
    uint8_t u8_serDeviceNr=i;

    //Workaround: Notwendig damit der Transceiver nicht in einen komischen Zustand geht, indem er den RX "flattern" lässt.
    //Unklar wo das Verhalten herkommt.
    if(i>2 && isSerialExtEnabled())
    {
      cbSetRxTxEn(2,serialRxTx_RxEn);
      usleep(50);
      cbSetRxTxEn(2,serialRxTx_RxTxDisable);
    }

    if(i>=2 && isSerialExtEnabled()) setSerialBaudrate(i); //Baudrate wechslen

    uint8_t *u8_pBmsFilterErrorCounter = getBmsFilterErrorCounter(BT_DEVICES_COUNT+i);

    //xSemaphoreTake(mSerialMutex, portMAX_DELAY);
    *u8_pBmsFilterErrorCounter &= ~(0x80); //Fehlermerker des aktuellen Durchgangs löschen (bit 0)
    #ifndef UTEST_BMS_FILTER
    serialDevData_s devData;
    devData.u8_NumberOfDevices=1;
    devData.u8_deviceNr=0;
    devData.u8_BmsDataAdr=i;
    devData.bo_writeData=false;
    devData.rwDataLen=0;
    devData.bo_sendMqttMsg=bo_lMqttSendMsg;

    //Überprüfen ob Daten an das BMS gesendet werden sollen
    bmsDataSemaphoreTake();
    uint8_t *lRwDataP=getSerialBmsWriteData(i,&devData.rwDataTyp,&devData.rwDataLen);
    uint8_t *lRwData;
    if(devData.rwDataLen>0)
    {
      devData.bo_writeData=true;

      lRwData = (uint8_t*)malloc(devData.rwDataLen);
      memcpy(lRwData, lRwDataP, devData.rwDataLen);
      devData.rwData = lRwData;

      clearSerialBmsWriteData(i);
    }
    bmsDataSemaphoreGive();

    //Wenn Spelos (o.ä.) an Serial 2 verbunden
    if(i>=2 && u8_lNumberOfSeplosBms>0)
    {
      u8_serDeviceNr=2;
      devData.u8_BmsDataAdr=i;
      devData.u8_NumberOfDevices=u8_lNumberOfSeplosBms;
      devData.u8_deviceNr=i-2;
    }

    //BSC_LOGI(TAG, "cyclicRun dev=%i, u8_BmsDataAdr=%i, u8_NumberOfDevices=%i, u8_deviceNr=%i", u8_serDeviceNr, devData.u8_BmsDataAdr,devData.u8_NumberOfDevices,devData.u8_deviceNr);
    if(serialDeviceData[u8_serDeviceNr].readBms!=NULL)
      bo_lBmsReadOk=serialDeviceData[u8_serDeviceNr].readBms(serialDeviceData[u8_serDeviceNr].stream_mPort, u8_serDeviceNr, &cbSetRxTxEn, &devData); //Wenn kein Fehler beim Holen der Daten vom BMS 
    else BSC_LOGE(TAG,"Error readBms nullptr, dev=%i",u8_serDeviceNr);

    if(devData.bo_writeData) free(lRwData);
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
    //xSemaphoreGive(mSerialMutex);
    if(bo_lBmsReadOk)
    {
      setBmsLastDataMillis(BT_DEVICES_COUNT+i,millis());
    }
    else
    {
      auto GetReasonStr = [u8_lReason]()
      {
        return (1==u8_lReason) ? "Checksum wrong" : "Filter";
      };
      BSC_LOGE(TAG,"ERROR: device=%i, reason=%s",i,GetReasonStr());
    }

    
    /* Überprüfen ob das BMS noch aktuelle Zellspannungen liefert, oder ob es "hängt" und nur noch die gleichen Werte liefert.
     * Es wird über alle Cellspannungen eine CRC16 gebildet. Die CRC muss sich in gewissen Abständen ändern, 
     * da nie alle Zellspannungen konstant sind. Es wird immer eine der Zellen um +/- ein mV schwanken.
    */
    uint8_t u8_lTriggerPlausibilityCeckCellVoltage = WebSettings::getInt(ID_PARAM_BMS_PLAUSIBILITY_CHECK_CELLVOLTAGE,0,DT_ID_PARAM_BMS_PLAUSIBILITY_CHECK_CELLVOLTAGE);
    if(u8_lTriggerPlausibilityCeckCellVoltage>0)
    {
      bmsDataSemaphoreTake();
      struct  bmsData_s *p_lBmsData = getBmsData();
      uint16_t crcNeu = crc16((uint8_t*)&p_lBmsData->bmsCellVoltage[BT_DEVICES_COUNT+i][0],24*2);
      uint8_t crcErrorCounter = p_lBmsData->bmsLastChangeCellVoltageCrc[BT_DEVICES_COUNT+i];
      bmsDataSemaphoreGive();

      uint16_t crcOld = getBmsCellVoltageCrc(BT_DEVICES_COUNT+i);
      //BSC_LOGI(TAG,"crcNeu=%i, crcOld=%i, crcErrorCounter=%i",crcNeu,crcOld,crcErrorCounter);
      if(crcNeu==crcOld)
      {
        if(crcErrorCounter>=CYCLES_BMS_VALUES_PLAUSIBILITY_CHECK) //Wenn sich der Wert x Zyklen nicht mehr geändert hat
        {
          if(crcErrorCounter==CYCLES_BMS_VALUES_PLAUSIBILITY_CHECK)
          {
            BSC_LOGE(TAG,"ERROR: device=%i, No change in cell voltage",i);
            //Der entsprechende Trigger wird in den Alarmrules gesetzt
          }
        }
        crcErrorCounter++;
        if(crcErrorCounter<0xFE)setBmsLastChangeCellVoltageCrc(BT_DEVICES_COUNT+i,crcErrorCounter);
      }
      else
      {
        if(crcErrorCounter!=0) setBmsLastChangeCellVoltageCrc(BT_DEVICES_COUNT+i,0);
        if(crcErrorCounter>CYCLES_BMS_VALUES_PLAUSIBILITY_CHECK) BSC_LOGI(TAG,"OK: device=%i, Change in cell voltage",i);
        setBmsCellVoltageCrc(BT_DEVICES_COUNT+i,crcNeu);
      }
    }

  }
  xSemaphoreGive(mSerialMutex);
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
