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
#include "devices/SeplosBmsV3.h"
#include "devices/NeeySerial.h"
#include "devices/JkInverterBms.h"
#ifdef BPN
#include "devices/bpnSerial.h"
#endif

static const char *TAG = "BSC_SERIAL";

uint32_t serialMqttSendeTimer;

struct serialDeviceData_s
{
  bool (*readBms)(BscSerial*, Stream*, uint8_t, serialDevData_s*) = 0;
  Stream * stream_mPort;

  uint32_t u32_baudrate;
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

  #ifdef LILYGO_TCAN485
  pinMode(TCAN485_RS485_EN_PIN, OUTPUT);
  digitalWrite(TCAN485_RS485_EN_PIN, HIGH);
  pinMode(TCAN485_RS485_SE_PIN, OUTPUT);
  digitalWrite(TCAN485_RS485_SE_PIN, HIGH);

  pinMode(TCAN485_PIN_5V_EN, OUTPUT);
  digitalWrite(TCAN485_PIN_5V_EN, HIGH);

  #else

  pinMode(SERIAL1_PIN_TX_EN, OUTPUT);  //HW serial0
  pinMode(SERIAL2_PIN_TX_EN, OUTPUT);  //HW serial1
  if(getHwVersion()>=2) pinMode(SERIAL3_PIN_TX_EN, OUTPUT);  //HW serial2
  pinMode(SERIAL3_PIN_RX_EN, OUTPUT);  //HW serial2
  #endif

  for(uint8_t i=0;i<SERIAL_BMS_DEVICES_COUNT;i++)
  {
    serialDeviceData[i].u8_mFilterBmsCellVoltageMaxCount=0;
    serialDeviceData[i].u32_baudrate=9600;

    #if defined(DEBUG_ON_HW_SERIAL) && !defined(LILYGO_TCAN485)
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

  #ifdef LILYGO_TCAN485
  if(u8_devNr==2) // Hw Serial 0
  {
    Serial2.end();
    Serial2.begin(baudrate,SERIAL_8N1,TCAN485_RS485_RX_PIN,TCAN485_RS485_TX_PIN);
    serialDeviceData[u8_devNr].stream_mPort=&Serial2;
  }

  #else

  if(u8_devNr==0) // Hw Serial 1
  {
    Serial.end();
    //Serial.setRxBufferSize(300);
    Serial.begin(baudrate,SERIAL_8N1,SERIAL1_PIN_RX,SERIAL1_PIN_TX);
    serialDeviceData[u8_devNr].stream_mPort=&Serial;
  }
  else if(u8_devNr==1) // Hw Serial 2
  {
    Serial1.end();
    //Serial1.setRxBufferSize(300);
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
    //Serial2.setRxBufferSize(300);
    Serial2.begin(baudrate,SERIAL_8N1,SERIAL3_PIN_RX,SERIAL3_PIN_TX);
    serialDeviceData[u8_devNr].stream_mPort=&Serial2;
  }
  #endif
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

  #ifdef LILYGO_TCAN485
  if(u8_devNr>=2)
  {
    setHwSerial(u8_devNr, baudrate);
  }
  #else
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
  #endif
}

void BscSerial::setSerialBaudrate(uint8_t u8_devNr)
{
  setSerialBaudrate(u8_devNr, serialDeviceData[u8_devNr].u32_baudrate);
}


void BscSerial::setReadBmsFunktion(uint8_t u8_devNr, uint8_t funktionsTyp)
{
  #ifdef LILYGO_TCAN485
  if(u8_devNr<2)
  {
    serialDeviceData[u8_devNr].readBms = 0;
    return;
  }
  #endif

  serialDeviceData[u8_devNr].u8_mFilterBmsCellVoltageMaxCount = (uint8_t)WebSettings::getIntFlash(ID_PARAM_BMS_FILTER_RX_ERROR_COUNT,0,DT_ID_PARAM_BMS_FILTER_RX_ERROR_COUNT);

  //xSemaphoreTake(mSerialMutex, portMAX_DELAY);

  switch (funktionsTyp)
  {
    case ID_SERIAL_DEVICE_NB:
      if(u8_devNr==2) setRxTxEnable(u8_devNr, serialRxTx_RxTxDisable);
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

    case ID_SERIAL_DEVICE_SEPLOSBMS_V3:
      BSC_LOGI(TAG,"setReadBmsFunktion Seplos V3");
      setSerialBaudrate(u8_devNr, 19200);
      serialDeviceData[u8_devNr].readBms = &SeplosBmsV3_readBmsData;
      break;

    case ID_SERIAL_DEVICE_NEEY_4A:
      BSC_LOGI(TAG,"setReadBmsFunktion NEEY 4A Serial");
      setSerialBaudrate(u8_devNr, 115200);
      serialDeviceData[u8_devNr].readBms = &NeeySerial_readBmsData;
      break;

    case ID_SERIAL_DEVICE_JKINVERTERBMS:
      BSC_LOGI(TAG,"setReadBmsFunktion JK Inverter");
      setSerialBaudrate(u8_devNr, 115200);
      serialDeviceData[u8_devNr].readBms = &JkInverterBms_readBmsData;
      break;

    case ID_SERIAL_DEVICE_PYLONTECH:
      BSC_LOGI(TAG,"Set serial device %i: Pylontech",u8_devNr);
      setSerialBaudrate(u8_devNr, 9600);
      serialDeviceData[u8_devNr].readBms = &SeplosBms_readBmsData;
      break;

    default:
      serialDeviceData[u8_devNr].readBms = 0;
  }

  //xSemaphoreGive(mSerialMutex);
}


/*
 * 
 */
//serialRxTxEn_e {serialRxTx_RxTxDisable, serialRxTx_TxEn, serialRxTx_RxEn};
void BscSerial::setRxTxEnable(uint8_t u8_devNr, serialRxTxEn_e e_rw)
{
  #ifndef LILYGO_TCAN485
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
    i2cExtSerialSetEnable(u8_devNr-3, e_rw);
  }
  #endif
}

void BscSerial::sendSerialData(Stream *port, uint8_t devNr, uint8_t *txBuffer, uint8_t txLen)
{
  setRxTxEnable(devNr,serialRxTx_TxEn);
  usleep(20);

  vTaskPrioritySet(task_handle_bscSerial, configMAX_PRIORITIES);
  
  port->write(txBuffer, txLen);
  port->flush();
  setRxTxEnable(devNr, serialRxTx_RxEn);

  vTaskPrioritySet(task_handle_bscSerial, TASK_PRIORITY_STD);
}


void BscSerial::cyclicRun()
{
  xSemaphoreTake(mSerialMutex, portMAX_DELAY);
  bool bo_lMqttSendMsg=false;

  if((millis()-serialMqttSendeTimer) > (WebSettings::getInt(ID_PARAM_MQTT_SEND_DELAY,0,DT_ID_PARAM_MQTT_SEND_DELAY)*1000))
  {
    serialMqttSendeTimer=millis();
    bo_lMqttSendMsg=true;
  }

  for(uint8_t i = 0; i < MUBER_OF_DATA_DEVICES; i++)
  {
    // 
    uint8_t dataDeviceSchnittstelle = (uint8_t)WebSettings::getInt(ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE,i,DT_ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE);
    uint8_t dataDeviceAdresse = (uint8_t)WebSettings::getInt(ID_PARAM_DEVICE_MAPPING_ADRESSE,i,DT_ID_PARAM_DEVICE_MAPPING_ADRESSE);
    
    uint8_t u8_serDeviceNr = dataDeviceSchnittstelle - BT_DEVICES_COUNT;

    // 
    if(dataDeviceSchnittstelle >= MUBER_OF_DATA_DEVICES) continue;

    // Wenn BT-Device eingestellt ist
    if(dataDeviceSchnittstelle < BT_DEVICES_COUNT) continue;
      
 
    bool    bo_lBmsReadOk = false;
    uint8_t u8_lReason = 1;

    //Workaround: Notwendig damit der Transceiver nicht in einen komischen Zustand geht, indem er den RX "flattern" lässt.
    //Unklar wo das Verhalten herkommt.
    if(i>2 && isSerialExtEnabled())
    {
      setRxTxEnable(2,serialRxTx_RxEn);
      usleep(50);
      setRxTxEnable(2,serialRxTx_RxTxDisable);
    }

    if(i>=2 && isSerialExtEnabled()) setSerialBaudrate(i); //Baudrate wechslen

    uint8_t *u8_pBmsFilterErrorCounter = getBmsFilterErrorCounter(BT_DEVICES_COUNT+i);

    //xSemaphoreTake(mSerialMutex, portMAX_DELAY);
    *u8_pBmsFilterErrorCounter &= ~(0x80); //Fehlermerker des aktuellen Durchgangs löschen (bit 0)
    #ifndef UTEST_BMS_FILTER
    serialDevData_s devData;
    devData.bmsAdresse = dataDeviceAdresse;
    devData.dataMappingNr = i;
    devData.bo_writeData = false;
    devData.rwDataLen = 0;
    devData.bo_sendMqttMsg = bo_lMqttSendMsg;

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


    //BSC_LOGI(TAG, "cyclicRun dev=%i, u8_BmsDataAdr=%i, u8_NumberOfDevices=%i, u8_deviceNr=%i", u8_serDeviceNr, devData.u8_BmsDataAdr,devData.u8_NumberOfDevices,devData.u8_deviceNr);
    if(serialDeviceData[u8_serDeviceNr].readBms != NULL)
    {
      bo_lBmsReadOk = serialDeviceData[u8_serDeviceNr].readBms(this, serialDeviceData[u8_serDeviceNr].stream_mPort, u8_serDeviceNr, &devData); //Wenn kein Fehler beim Holen der Daten vom BMS
      if(u8_serDeviceNr >= 2) setRxTxEnable(u8_serDeviceNr, serialRxTx_RxTxDisable);
    }
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
      setBmsLastDataMillis(i,millis());
    }
    else
    {
      //#ifndef UTEST_RESTAPI
      auto GetReasonStr = [u8_lReason]()
      {
        return (1==u8_lReason) ? "Checksum wrong" : "Filter";
      };
      BSC_LOGE(TAG,"ERROR: device=%i, reason=%s",i,GetReasonStr());
      //#else
      //setBmsLastDataMillis(i,millis());
      //#endif
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
      uint16_t crcNeu = crc16((uint8_t*)&p_lBmsData->bmsCellVoltage[i][0],24*2);
      uint8_t crcErrorCounter = p_lBmsData->bmsLastChangeCellVoltageCrc[i];
      bmsDataSemaphoreGive();

      uint16_t crcOld = getBmsCellVoltageCrc(i);
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
        if(crcErrorCounter<0xFE)setBmsLastChangeCellVoltageCrc(i,crcErrorCounter);
      }
      else
      {
        if(crcErrorCounter!=0) setBmsLastChangeCellVoltageCrc(i,0);
        if(crcErrorCounter>CYCLES_BMS_VALUES_PLAUSIBILITY_CHECK) BSC_LOGI(TAG,"OK: device=%i, Change in cell voltage",i);
        setBmsCellVoltageCrc(i,crcNeu);
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
