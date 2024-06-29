// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BleHandler.h"
#include "log.h"

#include "devices/JkBmsBt.h"
#include "devices/NeeyBalancer.h"
#include "AlarmRules.h"


static const char *TAG = "BLE_HANDLER";

bool bleNeeyBalancerConnect(uint8_t deviceNr);

static bleDevice bleDevices[BT_DEVICES_COUNT];
NimBLEScan* pBLEScan;
NimBLEAdvertisedDevice* advDevice;
uint8_t u8_mAdvDeviceNumber;

WebSettings webSettings;

static boolean bo_mBleHandlerRunning = false;
static boolean bo_mBtScanIsRunning = false;
static boolean bo_mBtNotAllDeviceConnectedOrScanRunning = false;
uint8_t u8_mScanAndNotConnectTimer;
uint8_t u8_mSendDataToNeey;

//NEEY
//NimBLEUUID NeyyBalancer4A_serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
//NimBLEUUID NeyyBalancer4A_charUUID   ("0000ffe1-0000-1000-8000-00805f9b34fb");
//byte NeeyBalancer_getInfo[20] PROGMEM = {0xaa, 0x55, 0x11, 0x01, 0x02, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf9, 0xff};
byte NeeyBalancer_getInfo[20] PROGMEM =  {0xaa, 0x55, 0x11, 0x01, 0x01, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA, 0xff};
byte NeeyBalancer_getInfo3[20] PROGMEM = {0xaa, 0x55, 0x11, 0x01, 0x02, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf9, 0xff};
//aa	55	11	1	1	0	14   0	0	0	0	0	0	0	0	0	0	0	fa	ff
//aa	55	11	1	4	0	14 	 0	0	0	0	0	0	0	0	0	0	0	ff	ff   // Lesen der Settings
//aa	55	11	1	2	0	 0  14	0	0	0	0	0	0	0	0	0	0	f9	ff

//JK-BMS
NimBLEUUID BtServiceUUID("ffe0");
NimBLEUUID BtCharUUID   ("ffe1");

class ClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient* pClient)
  {
    String devMacAdr = pClient->getPeerAddress().toString().c_str();
    BSC_LOGI(TAG, "onConnect() %s", devMacAdr.c_str());

    for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
    {
      if(bleDevices[i].macAdr.equals(devMacAdr))
      {
        switch(bleDevices[i].deviceTyp)
        {
          case ID_BT_DEVICE_NEEY4A:
          case ID_BT_DEVICE_NEEY8A:
            // interval 1,25ms; timeout 10ms
            //pClient->updateConnParams(BT_NEEY_POLL_INTERVAL,BT_NEEY_POLL_INTERVAL,0,300); //timeout 1500
            pClient->updateConnParams(20,20,0,60);
            break;
          case ID_BT_DEVICE_JKBMS_JK02:
          case ID_BT_DEVICE_JKBMS_JK02_32S:
            pClient->updateConnParams(120,120,0,60);
            jkBmsBtDevInit(i);
            break;
        }

        bleDevices[i].balancerOn=e_BalancerWaitForCmd;
        bleDevices[i].isConnect = true;
        bleDevices[i].doConnect = btDoConnectionWaitStart;
        setBmsLastDataMillis(i,millis());
      }
    }

    u8_mScanAndNotConnectTimer=BT_SCAN_AND_NOT_CONNECT_TIME;
  };

  void onDisconnect(NimBLEClient* pClient)
  {
    String devMacAdr = pClient->getPeerAddress().toString().c_str();
    #ifdef BT_DEBUG
    BSC_LOGI(TAG, "onDisconnect() %s", devMacAdr.c_str());
    #endif

    for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
    {
      if(bleDevices[i].macAdr.equals(devMacAdr.c_str()))
      {
        BSC_LOGI(TAG, "onDisconnect() dev=%i, mac=%s", i, devMacAdr.c_str());
        bleDevices[i].isConnect = false;
        bleDevices[i].doConnect = btDoConnectionIdle;
        bleDevices[i].deviceTyp = ID_BT_DEVICE_NB;
        bleDevices[i].macAdr = "";
      }
    }
  }

  bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params)
  {
    #ifdef BT_DEBUG
    BSC_LOGD(TAG, "onConnParamsUpdateRequest(): itvl_min=%i itvl_max=%i latency=%i supervision_timeout=%i",
      params->itvl_min,params->itvl_max,params->latency,params->supervision_timeout);
    #endif

    //BSC_LOGI(TAG, "onConnParamsUpdateRequest(): itvl_min=%i itvl_max=%i latency=%i supervision_timeout=%i",
    //  params->itvl_min,params->itvl_max,params->latency,params->supervision_timeout);

    //return true;
    return false; //Änderungen immer ablehnen

    #if 0
    if(params->itvl_min < 24) {
      return false;
    } else if(params->itvl_max > 40) {
      return false;
    } else if(params->latency > 2) {
      return false;
    } else if(params->supervision_timeout > 150) {
      return false;
    }

    #ifdef BT_DEBUG
    BSC_LOGD(TAG, "onConnParamsUpdateRequest(): true");
    #endif
    return true;
    #endif
  };
};


class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks
{
  std::string devMacAdr;

  void onResult(NimBLEAdvertisedDevice* advertisedDevice)
  {
    //Device gefunden
    devMacAdr = advertisedDevice->getAddress().toString();

    #ifdef BT_DEBUG
    BSC_LOGI(TAG, "onResult() dev found: %s",devMacAdr.c_str());
    #endif

    for(uint8_t i=0; i<BT_DEVICES_COUNT; i++)
    {
      #ifdef BT_DEBUG
      BSC_LOGD(TAG, "onResult() dev=%i, mac=%s", i, webSettings.getString(ID_PARAM_SS_BTDEVMAC,i).c_str());
      #endif

      if(!webSettings.getString(ID_PARAM_SS_BTDEVMAC,i).equals(""))
      {
        if(webSettings.getString(ID_PARAM_SS_BTDEVMAC,i).equals(devMacAdr.c_str()) && webSettings.getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV)!=ID_BT_DEVICE_NB)
        {
          BSC_LOGD(TAG, "Dev found: i=%i, mac=%s -> scan stop", i, webSettings.getString(ID_PARAM_SS_BTDEVMAC,i).c_str());

          NimBLEDevice::getScan()->stop();

          advDevice = advertisedDevice;
          u8_mAdvDeviceNumber = i;
          bleDevices[i].deviceTyp = (uint8_t)webSettings.getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV);
          bleDevices[i].macAdr = webSettings.getString(ID_PARAM_SS_BTDEVMAC,i);
          bleDevices[i].doConnect = btDoConnect;

          bo_mBtScanIsRunning = false; //Scan beendet
        }
      }
    }
  }
};






// Notification / Indication receiving handler callback
void notifyCB_NEEY(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  std::string notifyMacAdr = pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();
  //BSC_LOGI(TAG,"neey_cb mac=%s, len=%i",notifyMacAdr.c_str(),length);

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleDevices[i].macAdr.equals(notifyMacAdr.c_str()))
    {
      //Daten kopieren
      NeeyBalancer::neeyBalancerCopyData(i, pData, length);
    }
  }
}

//String temp="";
void notifyCB_JKBMS(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  std::string notifyMacAdr = pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleDevices[i].macAdr.equals(notifyMacAdr.c_str()))
    {
      //BSC_LOGI(TAG,"JKBMS RX len=%i", length);

      /*for(uint16_t i=0; i<length; i++)
      {
        temp += pData[i];
        temp+=" ";
      }
      BSC_LOGI(TAG,"JKBMS RX data=%s", temp.c_str());
      temp="";*/

      uint8_t u8_frameVersion=FRAME_VERSION_JK02;
      switch(bleDevices[i].deviceTyp)
      {
        case ID_BT_DEVICE_JKBMS_JK02_32S:
          u8_frameVersion=FRAME_VERSION_JK02_32S;
          break;
      }

      jkBmsBtCopyData(i, u8_frameVersion, pData, length);

      bleDevices[i].sendDataStep=0;
    }
  }
}


// Callback invoked when scanning has completed.
void scanCompleteCB(NimBLEScanResults scanResults)
{

}

// Create a single global instance of the callback class to be used by all clients
static ClientCallbacks clientCB;

bool btDeviceConnect()
{
  #ifdef BT_DEBUG
  BSC_LOGI(TAG, "btDeviceConnect()");
  #endif

  NimBLEClient* pClient = nullptr;
  NimBLEUUID serviceUUID;
  NimBLEUUID charUUID;

  uint8_t devNr = u8_mAdvDeviceNumber; //devNr ist letztes gefundenes Device!

  switch(bleDevices[devNr].deviceTyp)
  {
    case ID_BT_DEVICE_NEEY4A:
    case ID_BT_DEVICE_NEEY8A:
      serviceUUID=BtServiceUUID;
      charUUID=BtCharUUID;
      break;
    case ID_BT_DEVICE_JKBMS_JK02:
    case ID_BT_DEVICE_JKBMS_JK02_32S:
      serviceUUID=BtServiceUUID;
      charUUID=BtCharUUID;
      break;
    default:
      //Wenn Device-Typ nicht bekannt, dann abbrechen
      return false;
  }

  // Check if we have a client we should reuse first
  if(NimBLEDevice::getClientListSize())
  {
    /* Special case when we already know this device, we send false as the
     * second argument in connect() to prevent refreshing the service database.
     * This saves considerable time and power.
     */
    pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
    if(pClient)
    {
      //if(!pClient->connect(advDevice, false))
      if(!pClient->connect(advDevice, true))
      {
        BSC_LOGW(TAG, "Reconnect failed: mac=%s",advDevice->getAddress().toString().c_str());
        return false;
      }
      #ifdef BT_DEBUG
      BSC_LOGI(TAG, "Reconnect: dev=%s",advDevice->getAddress().toString().c_str());
      #endif
    }
    /* We don't already have a client that knows this device,
     * we will check for a client that is disconnected that we can use.
     */
    else
    {
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  // No client to reuse? Create a new one.
  if(!pClient)
  {
    if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
    {
      BSC_LOGW(TAG, "Max clients reached - no more connections available");
      return false;
    }

    pClient = NimBLEDevice::createClient();
    #ifdef BT_DEBUG
    BSC_LOGI(TAG, "New client created");
    #endif
    pClient->setClientCallbacks(&clientCB, false);

    // Interval *1.25ms; timeout *10ms
    pClient->setConnectionParams(12,12,0,51);

    // Set how long we are willing to wait for the connection to complete (seconds)
    pClient->setConnectTimeout(5);

    if (!pClient->connect(advDevice))
    {
      // Created a client but failed to connect, don't need to keep it as it has no data
      BSC_LOGW(TAG, "Failed to connect, deleted client; dev=%i",devNr);
      NimBLEDevice::deleteClient(pClient);
      return false;
    }
  }

  if(!pClient->isConnected())
  {
    if (!pClient->connect(advDevice))
    {
      BSC_LOGW(TAG, "Failed to connect; dev=%i", devNr);
      return false;
    }
  }
  #ifdef BT_DEBUG
  BSC_LOGI(TAG, "Connected to: %s, RSSI: %i",pClient->getPeerAddress().toString().c_str(),pClient->getRssi());
  #endif

  // Now we can read/write/subscribe the charateristics of the services we are interested in
  NimBLERemoteService* pSvc = nullptr;
  bleDevices[devNr].pChr = nullptr;
  NimBLERemoteDescriptor* pDsc = nullptr;

  pSvc = pClient->getService(serviceUUID);
  if(pSvc)
  {
    bleDevices[devNr].pChr = pSvc->getCharacteristic(charUUID);

    if(bleDevices[devNr].pChr)
    {
      if(bleDevices[devNr].pChr->canRead())
      {
        #ifdef BT_DEBUG
        BSC_LOGI(TAG, "can read: %s, Value: %s",bleDevices[devNr].pChr->getUUID().toString().c_str(),bleDevices[devNr].pChr->readValue().c_str());
        #endif
      }

      if(bleDevices[devNr].pChr->canWrite())
      {
        #ifdef BT_DEBUG
        BSC_LOGI(TAG, "can write");
        #endif
      }

      //Subscribe
      if(bleDevices[devNr].pChr->canNotify())
      {
        bool bo_lSubscribeRet=false;
        switch(bleDevices[devNr].deviceTyp)
        {
          case ID_BT_DEVICE_NEEY4A:
          case ID_BT_DEVICE_NEEY8A:
            bo_lSubscribeRet=bleDevices[devNr].pChr->subscribe(true, notifyCB_NEEY);
            break;
          case ID_BT_DEVICE_JKBMS_JK02:
          case ID_BT_DEVICE_JKBMS_JK02_32S:
            bo_lSubscribeRet=bleDevices[devNr].pChr->subscribe(true, notifyCB_JKBMS);
            break;
        }

        if(!bo_lSubscribeRet)
        {
          // Disconnect if subscribe failed
          pClient->disconnect();
          BSC_LOGW(TAG, "Device not connected; Can not subscribe; dev=%i",devNr);
          return false;
        }
        else
        {
          BSC_LOGI(TAG, "Device connected; dev=%i",devNr);
          return true;
        }
      }
      else
      {
        pClient->disconnect();
        BSC_LOGW(TAG, "Device not connected; Can not notify; dev=%i",devNr);
        return false;
      }
    }
    else
    {
      pClient->disconnect();
      BSC_LOGW(TAG, "Device not connected; Characteristic not found; dev=%i",devNr);
      return false;
    }
  }
  else
  {
    pClient->disconnect();
    BSC_LOGW(TAG, "Device not connected; Service not found; dev=%i",devNr);
    return false;
  }

  BSC_LOGW(TAG, "Device not connected; dev=%i",devNr);
  return false;
}


void btDeviceDisconnect(uint8_t devNr)
{
  //#ifdef BT_DEBUG
  //BSC_LOGI(TAG, "btDeviceDisconnect()");
  //#endif

  NimBLEClient* pClient = nullptr;

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(NimBLEDevice::getClientListSize())
    {
      std::string adrStr = std::string(bleDevices[i].macAdr.c_str());
      NimBLEAddress adr = NimBLEAddress(adrStr);

      pClient = NimBLEDevice::getClientByPeerAddress(adr);
      if(pClient)
      {
        pClient->disconnect();
      }
    }
    else
    {
      break;
    }
  }
}

void btDeviceDisconnectSingle(uint8_t devNr)
{
  //#ifdef BT_DEBUG
  //BSC_LOGI(TAG, "btDeviceDisconnectSingle() devNr=%i",devNr);
  //#endif

  if(devNr>=0 && devNr<BT_DEVICES_COUNT)
  {
    NimBLEClient* pClient = nullptr;
    if(NimBLEDevice::getClientListSize())
    {
      std::string adrStr = std::string(bleDevices[devNr].macAdr.c_str());
      NimBLEAddress adr = NimBLEAddress(adrStr);

      pClient = NimBLEDevice::getClientByPeerAddress(adr);
      if(pClient)
      {
        pClient->disconnect();
        BSC_LOGI(TAG, "Device disconnected single, dev=%s",bleDevices[devNr].macAdr.c_str());
      }
    }
  }
}

//#define BT_LATENCY 0
//#define BT_NEEY_POLL_MAX_INTERVAL 720
//#define BT_TIMEOUT (((1+BT_LATENCY)*BT_NEEY_POLL_MAX_INTERVAL)/4)+1

/*void changeBtConnParams(uint8_t devNr, uint8_t speed)
{
  BSC_LOGI(TAG,"changeBtConnParams: A");
  if(NimBLEDevice::getClientListSize())
  {
    BSC_LOGI(TAG,"changeBtConnParams: B");
    std::string adrStr = std::string(bleDevices[devNr].macAdr.c_str());
    NimBLEAddress adr = NimBLEAddress(adrStr);

    NimBLEClient* pClient = nullptr;
    pClient = NimBLEDevice::getClientByPeerAddress(adr);
    if(pClient)
    {
      BSC_LOGI(TAG,"changeBtConnParams: dev=%i, speed=%i",devNr, speed);
      if(speed==0) pClient->updateConnParams(8,16,0,500);
      else if(speed==10) pClient->updateConnParams(720,720,0,300);
    }
  }
}*/





/**
 * Class BleHandler
 */
BleHandler::BleHandler()
{
};


void BleHandler::init()
{
  if(bo_mBleHandlerRunning)return;

  u8_mScanAndNotConnectTimer=0;
  timer_startScan=0;
  bo_mStartManualScan=false;
  bo_mBtScanIsRunning=false;
  bo_mBtNotAllDeviceConnectedOrScanRunning=false;
  u8_mSendDataToNeey=0;

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    bleDevices[i].doConnect = btDoConnectionIdle;
    bleDevices[i].isConnect = false;
    bleDevices[i].macAdr = "";
    bleDevices[i].deviceTyp = ID_BT_DEVICE_NB;
    setBmsLastDataMillis(i,0);
    bleDevices[i].sendDataStep=0;
    bleDevices[i].balancerOn=e_BalancerWaitForCmd;

    for(uint8_t n=0;n<24;n++)
    {
      setBmsCellVoltage(i,n,0);
    }

  }

  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_N0); // ESP_PWR_LVL_P9

  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(45);
  pBLEScan->setWindow(15);
  pBLEScan->setActiveScan(true);
  pBLEScan->setMaxResults(BT_SCAN_RESULTS);
  pBLEScan->clearResults();
};

void BleHandler::start()
{
  bo_mBleHandlerRunning=true;
}

void BleHandler::stop()
{
  btDeviceDisconnect(0);

  bo_mBleHandlerRunning=false;
  bo_mBtNotAllDeviceConnectedOrScanRunning=false;
  u8_mSendDataToNeey=0;
}

void BleHandler::startScan()
{
  bo_mStartManualScan=true;
}

bool BleHandler::isScanFinish()
{
  if(bo_mBtScanIsRunning)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool BleHandler::isNotAllDeviceConnectedOrScanRunning()
{
  return bo_mBtNotAllDeviceConnectedOrScanRunning;
}

void BleHandler::sendDataToNeey()
{
  if(u8_mSendDataToNeey==0)
  {
    //BSC_LOGI(TAG,"sendDataToNeey()");
    u8_mSendDataToNeey=1;
  }
}

void BleHandler::readDataFromNeey()
{
  if(u8_mSendDataToNeey==0)
  {
    BSC_LOGI(TAG,"readDataFromNeey()");
    u8_mSendDataToNeey=2;
  }
}


void BleHandler::run()
{
  #ifdef BT_DEBUG
  BSC_LOGD(TAG, "run(a) bo_mBtScanIsRunning=%i, bo_mBleHandlerRunning=%i",bo_mBtScanIsRunning,bo_mBleHandlerRunning);
  #endif

  if(!bo_mBleHandlerRunning)return;

  #ifdef BT_DEBUG
  BSC_LOGD(TAG, "run(b) bo_mBtScanIsRunning=%i",bo_mBtScanIsRunning);
  #endif

  //Devices trennen wenn notwendig
  handleDisconnectionToDevices();

  //Überprüfe ob Devices verbunden werden müssen
  boolean bo_lDoStartBtScan=handleConnectionToDevices();

  //Scan abbrechen, wenn Daten an den NEEY gesendet werden müssen
  if(bo_mBtScanIsRunning && u8_mSendDataToNeey>0 && !bo_lDoStartBtScan)
  {
    BSC_LOGD(TAG, "Send Data to NEEY -> Stoppe Scan!");
    bo_lDoStartBtScan=false;
    pBLEScan->stop();
    bo_mBtNotAllDeviceConnectedOrScanRunning=false;
    bo_mBtScanIsRunning=false;
  }

  //Manuellen scan starten
  if(bo_mStartManualScan && u8_mSendDataToNeey==0)
  {
    bo_mStartManualScan=false;
    if(!bo_mBtScanIsRunning) bo_lDoStartBtScan=true;
  }

  //Wenn angefordert, dann starte neuen BT scan
  if(bo_lDoStartBtScan && !bo_mBtScanIsRunning)
  {
    BSC_LOGD(TAG, "Starte BT Scan");
    bo_mBtNotAllDeviceConnectedOrScanRunning=true;
    bo_lDoStartBtScan = false;
    bo_mBtScanIsRunning = true;
    if(pBLEScan->isScanning())
    {
      BSC_LOGD(TAG, "Scan läuft noch");
    }
    else
    {
      u8_mScanAndNotConnectTimer=BT_SCAN_AND_NOT_CONNECT_TIME;
      NimBLEDevice::getScan()->clearResults();
      NimBLEDevice::getScan()->start(0,scanCompleteCB);
    }
  }

  #ifdef BT_DEBUG
  BSC_LOGD(TAG, "run() u8_mScanAndNotConnectTimer=%i, bo_mBtScanIsRunning=%i, scanResCount=%i",u8_mScanAndNotConnectTimer,bo_mBtScanIsRunning,pBLEScan->getResults().getCount());
  #endif

  //Stop den Scan, wenn die Ergebnisliste voll ist und x Sekunden kein Device mehr verbunden wurde
  if(u8_mScanAndNotConnectTimer>0 && bo_mBtScanIsRunning /*&& pBLEScan->getResults().getCount()>=BT_SCAN_RESULTS*/)
  {
    u8_mScanAndNotConnectTimer--;
  }
  if(u8_mScanAndNotConnectTimer==1)
  {
    u8_mScanAndNotConnectTimer=0;
    BSC_LOGD(TAG, "Connect Timeout -> Stoppe Scan!");
    pBLEScan->stop();
    bo_mBtNotAllDeviceConnectedOrScanRunning=false;
    bo_mBtScanIsRunning=false;
  }
}


void BleHandler::setBalancerState(uint8_t u8_devNr, boolean bo_state)
{
  if(bo_state)
  {
    if(bleDevices[u8_devNr].balancerOn!=e_BalancerIsOn) bleDevices[u8_devNr].balancerOn=e_BalancerChangeToOn;
  }
  else
  {
    if(bleDevices[u8_devNr].balancerOn!=e_BalancerIsOff) bleDevices[u8_devNr].balancerOn=e_BalancerChangeToOff;
  }
}


/* handleConnectToDevices()
 * Die Funktion Handelt den Verbinungsaufbau und ob eine neue Suche nach BT Devices gestartet werden muss.
 * Rückgabewert: true, wenn eine neue Suche gestarten werden soll.
**/
bool BleHandler::handleConnectionToDevices()
{
  boolean bo_lDoStartBtScan=false;
  uint8_t u8_lBtConnStatus=0;

  //Nur wenn nicht nach BT Devices gesucht wird
  if(!bo_mBtScanIsRunning)
  {
    //BT Devices verbinden
    for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
    {
      uint8_t u8_lBtDevType = webSettings.getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV);

      uint8_t u8_devDeativateTriggerNr = webSettings.getInt(ID_PARAM_BTDEV_DEACTIVATE,i,DT_ID_PARAM_BTDEV_DEACTIVATE);
      bool bo_devDeactivateTrigger=false;
      if(u8_devDeativateTriggerNr>0) bo_devDeactivateTrigger=getAlarm(u8_devDeativateTriggerNr-1);

      //Überprüfen ob BT-Devices gesucht werden müssen; ggf. suche starten
      if(u8_lBtDevType>ID_BT_DEVICE_NB && !bo_devDeactivateTrigger) //Wenn ein Device parametriert ist
      {
        if(bleDevices[i].isConnect==false && bleDevices[i].doConnect==btDoConnectionIdle) //Wenn es nicht verbunden ist
        {
          //Suche starten
          BSC_LOGD(TAG, "Suche device %i, %s",i,bleDevices[i].macAdr.c_str());
          bo_lDoStartBtScan = true;
        }
      }

      if(bleDevices[i].doConnect == btDoConnect)
      {
        bleDevices[i].doConnect = btConnectionSetup; //Verbdinung wird hergestellt

        if(btDeviceConnect())
        {
          u8_lBtConnStatus|=(1<<i);
        }
        else //Wenn Verbingungsversuch fehgeschlagen
        {
          bleDevices[i].doConnect = btDoConnectionIdle;
        }
      }
      else //Wenn keine Verbindungsaufforderung besteht
      {
        if((u8_lBtDevType==ID_BT_DEVICE_NB) || (u8_lBtDevType>ID_BT_DEVICE_NB && bleDevices[i].isConnect) || bo_devDeactivateTrigger){u8_lBtConnStatus|=(1<<i);}
      }
    }

    //Wenn alle Geräte verbunden sind
    #ifdef BT_DEBUG
    BSC_LOGI(TAG,"u8_lBtConnStatus=%i",u8_lBtConnStatus);
    #endif
    if(u8_lBtConnStatus==0x7F)
    {
      bool bo_lAllDevWrite=false;
      for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
      {
        if(bleDevices[i].doConnect==btDoConnectionWaitStart)
        {
          BSC_LOGD(TAG,"BT write start (%i) typ=%i",i,bleDevices[i].deviceTyp);
          switch(bleDevices[i].deviceTyp)
          {
            case ID_BT_DEVICE_NEEY4A:
              BSC_LOGD(TAG,"BT write dev=%i",i);
              bleDevices[i].pChr->writeValue(NeeyBalancer_getInfo, 20);
              delay(200);
              NeeyBalancer::neeyWriteMsg2(ID_BT_DEVICE_NEEY4A, bleDevices[i].pChr);
              delay(200);
              bleDevices[i].pChr->writeValue(NeeyBalancer_getInfo3, 20);

              bleDevices[i].doConnect = btDoConnectionIdle;
              bleDevices[i].sendDataStep = 0;
              break;

            case ID_BT_DEVICE_NEEY8A:
              BSC_LOGD(TAG,"BT write dev=%i",i);
              NeeyBalancer::sendNeeyConnectMsg(ID_BT_DEVICE_NEEY8A, bleDevices[i].pChr);

              bleDevices[i].doConnect = btDoConnectionIdle;
              bleDevices[i].sendDataStep = 0;
              break;

            case ID_BT_DEVICE_JKBMS_JK02:
            case ID_BT_DEVICE_JKBMS_JK02_32S:
              //BSC_LOGI(TAG,"BT write (A) dev=%i",i);
              uint8_t frame[20];
              jkBmsBtBuildSendFrame(frame, JKBMS_BT_COMMAND_DEVICE_INFO, 0x00000000, 0x00);
              bleDevices[i].pChr->writeValue(frame, 20);

              bleDevices[i].doConnect = btDoConnectionIdle;
              break;
          }
          bo_lAllDevWrite=true;
        }
        else if(bleDevices[i].doConnect==btDoConnectionIdle)
        {
          if(bleDevices[i].deviceTyp==ID_BT_DEVICE_NEEY4A || bleDevices[i].deviceTyp==ID_BT_DEVICE_NEEY8A)
          {
            if(bleDevices[i].sendDataStep==0)
            {
              bleDevices[i].sendDataStep=1;
              //NeeyBalancer::neeyWriteMsg2(bleDevices[i].pChr);

              //changeBtConnParams(i, 10);
            }
            else if(bleDevices[i].sendDataStep==1)
            {
              bleDevices[i].sendDataStep=2;
              //bleDevices[i].pChr->writeValue(NeeyBalancer_getInfo3, 20);

              // Update connections parameter
              /*NimBLEClient* pClient = nullptr;
              if(NimBLEDevice::getClientListSize())
              {
                std::string adrStr = std::string(bleDevices[i].macAdr.c_str());
                NimBLEAddress adr = NimBLEAddress(adrStr);

                pClient = NimBLEDevice::getClientByPeerAddress(adr);
                if(pClient)
                {
                  pClient->updateConnParams(720,720,1,1500);
                }
              }*/
            }
            else if(bleDevices[i].sendDataStep==2)
            {
              if(u8_mSendDataToNeey>0)
              {
                if(u8_mSendDataToNeey==1)
                {
                  //changeBtConnParams(i, 0);
                  NeeyBalancer::neeyWriteData_GotoStartStep(1);
                  u8_mSendDataToNeey=10;
                }
                else if(u8_mSendDataToNeey==2) //Nur lesen der Settings
                {
                  //changeBtConnParams(i, 0);
                  NeeyBalancer::neeyWriteData_GotoStartStep(10);
                  u8_mSendDataToNeey=10;
                }
                NeeyBalancer::neeyWriteData(bleDevices[i].deviceTyp, i, bleDevices[i].pChr);

                //if(u8_mSendDataToNeey==10) changeBtConnParams(i, 10);
              }
              else //Hier befinden wir uns, wenn alle Devices verbunden sind und wir im normalen Betrieb sind
              {
                //Überprüfen ob Daten noch aktuell sind
                uint8_t u8_lBtDevType, u8_devDeativateTriggerNr;
                bool bo_devDeactivateTrigger=false;

                if((millis()-getBmsLastDataMillis(i))>5000)
                {
                  u8_lBtDevType = webSettings.getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV);
                  u8_devDeativateTriggerNr = webSettings.getInt(ID_PARAM_BTDEV_DEACTIVATE,i,DT_ID_PARAM_BTDEV_DEACTIVATE);
                  bo_devDeactivateTrigger=false;
                  if(u8_devDeativateTriggerNr>0) bo_devDeactivateTrigger=getAlarm(u8_devDeativateTriggerNr-1);

                  //Überprüfen ob BT-Devices verbunden sein sollte
                  if((u8_lBtDevType==ID_BT_DEVICE_NEEY4A || u8_lBtDevType==ID_BT_DEVICE_NEEY8A)&& !bo_devDeactivateTrigger)
                  {
                    BSC_LOGI(TAG,"No cyclical data from dev %i", i);
                    btDeviceDisconnectSingle(i);
                  }
                }

                if(bleDevices[i].balancerOn==e_BalancerChangeToOff)
                {
                  bleDevices[i].balancerOn=e_BalancerIsOff;
                  NeeyBalancer::neeySetBalancerOnOff(bleDevices[i].pChr, false);
                }
                else if(bleDevices[i].balancerOn==e_BalancerChangeToOn)
                {
                  bleDevices[i].balancerOn=e_BalancerIsOn;
                  NeeyBalancer::neeySetBalancerOnOff(bleDevices[i].pChr, true);
                }
              }
            }
          }
          else if(bleDevices[i].deviceTyp==ID_BT_DEVICE_JKBMS_JK02 || bleDevices[i].deviceTyp==ID_BT_DEVICE_JKBMS_JK02_32S)
          {
            bleDevices[i].sendDataStep++;
            if(bleDevices[i].sendDataStep>2)
            {
              //BSC_LOGI(TAG,"BT write (B) dev=%i",i);
              uint8_t frame[20];
              jkBmsBtBuildSendFrame(frame, JKBMS_BT_COMMAND_CELL_INFO, 0x00000000, 0x00);
              bleDevices[i].pChr->writeValue(frame, 20);

              bleDevices[i].sendDataStep=0;
            }
          }
        }
      }
      if(u8_mSendDataToNeey==10)
      {
        if(NeeyBalancer::neeyWriteData_GotoNextStep()) u8_mSendDataToNeey=0;
      }
      if(bo_lAllDevWrite)bo_mBtNotAllDeviceConnectedOrScanRunning=false;
    }
  }

  return bo_lDoStartBtScan;
}


void BleHandler::handleDisconnectionToDevices()
{
  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleDevices[i].isConnect)
    {
      uint8_t u8_devDeativateTriggerNr = webSettings.getInt(ID_PARAM_BTDEV_DEACTIVATE,i,DT_ID_PARAM_BTDEV_DEACTIVATE);
      bool bo_devDeactivateTrigger=false;
      if(u8_devDeativateTriggerNr>0) bo_devDeactivateTrigger=getAlarm(u8_devDeativateTriggerNr-1);

      if(webSettings.getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV)==ID_BT_DEVICE_NB || webSettings.getString(ID_PARAM_SS_BTDEVMAC,i).equals("") || bo_devDeactivateTrigger)
      {
        if(NimBLEDevice::getClientListSize())
        {
          NimBLEClient* pClient = NimBLEDevice::getClientByPeerAddress(NimBLEAddress(bleDevices[i].macAdr.c_str(), BLE_ADDR_PUBLIC));
          if(pClient)
          {
            if(!pClient->disconnect())
            {
              BSC_LOGI(TAG, "Device disconnected, dev=%s",bleDevices[i].macAdr.c_str());
            }
          }
        }
      }
    }
  }
}


std::string BleHandler::getBtScanResult()
{
  std::string btDevScanResult = "<table>";

  NimBLEScanResults results = NimBLEDevice::getScan()->getResults();

  for (int i = 0; i < results.getCount(); i++)
  {
    NimBLEAdvertisedDevice device = results.getDevice(i);

    btDevScanResult += "<tr>";

    btDevScanResult += "<td>";
    btDevScanResult += device.getAddress().toString();
    btDevScanResult += "</td>";

    btDevScanResult += "<td>";
    btDevScanResult += device.getName();
    btDevScanResult += "</td>";

    btDevScanResult += "<td>";
    btDevScanResult += device.getServiceUUID();
    btDevScanResult += "</td>";

    btDevScanResult += "<td>";
    btDevScanResult += device.getServiceDataUUID();
    btDevScanResult += "</td>";

    btDevScanResult += "<td>";
    btDevScanResult+="<button onclick='copyStringToClipboard(\"";
    btDevScanResult+=device.getAddress().toString();
    btDevScanResult+="\")'>Copy</button>";
    btDevScanResult += "</td>";

    btDevScanResult += "</tr>";
  }

  btDevScanResult += "<table>";

  return btDevScanResult;
}


uint8_t BleHandler::bmsIsConnect(uint8_t devNr)
{
  if(bleDevices[devNr].isConnect)
  {
    return 2;
  }
  else
  {
    if(!webSettings.getString(ID_PARAM_SS_BTDEVMAC,devNr).equals("") && webSettings.getInt(ID_PARAM_SS_BTDEV,devNr,DT_ID_PARAM_SS_BTDEV)!=ID_BT_DEVICE_NB) return 1;
  }
  return 0;
}
