// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BleHandler.h"
#include "log.h"

static const char *TAG = "BLE_HANDLER";

//void scanEndedCB(NimBLEScanResults results);
bool bleNeeyBalancerConnect(uint8_t deviceNr);

static bleDevice bleDevices[BT_DEVICES_COUNT];
NimBLEScan* pBLEScan;
NimBLEAdvertisedDevice* advDevice;

WebSettings webSettings;

//static boolean doStartBtScan   = false; 
static boolean btScanIsRunning = false; 

uint8_t u8_mScanAndNotConnectTimer;

NimBLEUUID NeyyBalancer4A_serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
NimBLEUUID NeyyBalancer4A_charUUID   ("0000ffe1-0000-1000-8000-00805f9b34fb");

byte NeeyBalancer_getInfo[20] PROGMEM = {0xaa, 0x55, 0x11, 0x01, 0x02, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf9, 0xff}; 

std::string notifyMacAdr;


class ClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient* pClient)
  {
    ESP_LOGI(TAG, "Connected");
    /** After connection we should change the parameters if we don't need fast response times.
     *  These settings are 150ms interval, 0 latency, 450ms timout.
     *  Timeout should be a multiple of the interval, minimum is 100ms.
     *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
     *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
     */
    pClient->updateConnParams(120,120,0,150);

    String devMacAdr = pClient->getPeerAddress().toString().c_str();
    ESP_LOGI(TAG, "%s", devMacAdr.c_str());

    for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
    {
      if(bleDevices[i].macAdr.equals(devMacAdr))
      {
        bleDevices[i].isConnect = true;
        bleDevices[i].doConnect = btDoConnectionIdle; 
        setBmsLastDataMillis(i,millis());
      }
    }

    u8_mScanAndNotConnectTimer=BT_SCAN_AND_NOT_CONNECT_TIME;
  };

  void onDisconnect(NimBLEClient* pClient)
  {
    String devMacAdr = pClient->getPeerAddress().toString().c_str();
    ESP_LOGI(TAG, "Disconnected: %s", devMacAdr.c_str());

    for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
    {
      if(bleDevices[i].macAdr.equals(devMacAdr.c_str()))
      {
        ESP_LOGI(TAG, "Disconnected Device %s", i);
        bleDevices[i].isConnect = false;
        bleDevices[i].doConnect = btDoConnectionIdle; 
        bleDevices[i].deviceTyp = ID_BT_DEVICE_NB;
        bleDevices[i].macAdr = "";
      }
    }
  }

  /** Called when the peripheral requests a change to the connection parameters.
   *  Return true to accept and apply them or false to reject and keep
   *  the currently used parameters. Default will return true.
   */
  bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params)
  {
    ESP_LOGD(TAG, "onConnParamsUpdateRequest(): itvl_min=%i itvl_max=%i latency=%i supervision_timeout=%i",
      params->itvl_min,params->itvl_max,params->latency,params->supervision_timeout);

    if(params->itvl_min < 24) { /** 1.25ms units */
      return false;
    } else if(params->itvl_max > 40) { /** 1.25ms units */
      return false;
    } else if(params->latency > 2) { /** Number of intervals allowed to skip */
      return false;
    } else if(params->supervision_timeout > 150) { /** 10ms units */
      return false;
    }

    ESP_LOGD(TAG, "onConnParamsUpdateRequest(): true");
    return true;
  };
};


class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks 
{
  std::string devMacAdr;

  void onResult(NimBLEAdvertisedDevice* advertisedDevice)
  {
    ESP_LOGI(TAG, "BT device found()");

    //Device gefunden
    devMacAdr = advertisedDevice->getAddress().toString();

    for(uint8_t i=0; i<BT_DEVICES_COUNT; i++)
    {    
      if(!webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0).equals(""))
      {
        if (webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0).equals(devMacAdr.c_str()) && webSettings.getString(ID_PARAM_SS_BTDEV,0,i,0).equals(String(ID_BT_DEVICE_NB))==false)
        {
          ESP_LOGI(TAG, "Gesuchtes Device gefunden: %s", webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0).c_str());
          ESP_LOGI(TAG, "Scan stop");

          NimBLEDevice::getScan()->stop();
        
          advDevice = advertisedDevice;   
          bleDevices[i].deviceTyp = webSettings.getInt(ID_PARAM_SS_BTDEV,0,i,0);
          bleDevices[i].macAdr = webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0);
          bleDevices[i].doConnect = btDoConnect;
     
          btScanIsRunning = false; //Scan beendet
        }
      }
    }
  }
};


/** Notification / Indication receiving handler callback */
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  notifyMacAdr = pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleDevices[i].macAdr.equals(notifyMacAdr.c_str()))
    {
      setBmsLastDataMillis(i,millis());

      if(bleDevices[i].deviceTyp==ID_BT_DEVICE_NEEY4A)
      {
        //Daten kopieren
        NeeyBalancer::neeyBalancerCopyData(i, pData, length);
      }
    }
  }

  /*debugPrint(": len=");
  debugPrintln(length);
  debugPrint("data=");
  //debugPrintln((char*)pData);
  for(uint16_t i=0; i<length; i++)
  {
    debugPrint(pData[i], HEX);
    debugPrint(" ");
  }
  debugPrintln("");*/
}


/**
 * Callback invoked when scanning has completed.
 */
void scanCompleteCB(NimBLEScanResults scanResults) {
	//debugPrintf("Scan complete! %i Devices found",scanResults.getCount());
} 


/** Create a single global instance of the callback class to be used by all clients */
static ClientCallbacks clientCB;

bool bleNeeyBalancerConnect(uint8_t devNr)
{
  ESP_LOGI(TAG, "bleNeeyBalancerConnect()");

  NimBLEClient* pClient = nullptr;

  /** Check if we have a client we should reuse first **/
  if(NimBLEDevice::getClientListSize())
  {
    /** Special case when we already know this device, we send false as the
     *  second argument in connect() to prevent refreshing the service database.
     *  This saves considerable time and power.
     */
    pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
    if(pClient)
    {
      //if(!pClient->connect(bleDevices[devNr].advDevice, false))
      if(!pClient->connect(advDevice, false))
      {
        ESP_LOGI(TAG, "Reconnect failed");
        return false;
      }
      ESP_LOGI(TAG, "Reconnect client");
    }
    /** We don't already have a client that knows this device,
     *  we will check for a client that is disconnected that we can use.
     */
    else
    {
      pClient = NimBLEDevice::getDisconnectedClient();
    }
  }

  /** No client to reuse? Create a new one. */
  if(!pClient)
  {
    if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
    {
      ESP_LOGI(TAG, "Max clients reached - no more connections available");
      return false;
    }

    pClient = NimBLEDevice::createClient();
    ESP_LOGI(TAG, "New client created");
    pClient->setClientCallbacks(&clientCB, false);
    /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
     *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
     *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
     *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
     */
    //pClient->setConnectionParams(12,12,0,51);
    pClient->setConnectionParams(12,12,0,150);
    /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
    pClient->setConnectTimeout(5);


    //if (!pClient->connect(bleDevices[devNr].advDevice))
    if (!pClient->connect(advDevice))
    {
      /** Created a client but failed to connect, don't need to keep it as it has no data */
      ESP_LOGI(TAG, "Failed to connect, deleted client");
      NimBLEDevice::deleteClient(pClient);
      return false;
    }
  }

  if(!pClient->isConnected()) {
    //if (!pClient->connect(bleDevices[devNr].advDevice))
    if (!pClient->connect(advDevice))
    {
      ESP_LOGI(TAG, "Failed to connect");
      return false;
    }
  }

  ESP_LOGI(TAG, "Connected to: %s, RSSI: %i",pClient->getPeerAddress().toString().c_str(),pClient->getRssi());

  /** Now we can read/write/subscribe the charateristics of the services we are interested in */
  NimBLERemoteService* pSvc = nullptr;
  //NimBLERemoteCharacteristic* pChr = nullptr;
  bleDevices[devNr].pChr = nullptr;
  NimBLERemoteDescriptor* pDsc = nullptr;

  pSvc = pClient->getService(NeyyBalancer4A_serviceUUID);
  if(pSvc)  /** make sure it's not null */
  {     
    bleDevices[devNr].pChr = pSvc->getCharacteristic(NeyyBalancer4A_charUUID);

    if(bleDevices[devNr].pChr)
    {     /** make sure it's not null */
      if(bleDevices[devNr].pChr->canRead())
      {
        ESP_LOGI(TAG, "%s, Value: %s",bleDevices[devNr].pChr->getUUID().toString().c_str(),bleDevices[devNr].pChr->readValue().c_str());
      }

      if(bleDevices[devNr].pChr->canWrite())
      {
        ESP_LOGI(TAG, "can write");
        /*if(pChr->writeValue("Tasty"))
        {
          debugPrint("Wrote new value to: ");
          debugPrintln(pChr->getUUID().toString().c_str());
        }
        else
        {
          // Disconnect if write failed 
          pClient->disconnect();
          return false;
        }

        if(pChr->canRead())
        {
          debugPrint("The value of: ");
          debugPrint(pChr->getUUID().toString().c_str());
          debugPrint(" is now: ");
          debugPrintln(pChr->readValue().c_str());
        }*/
      }

      /** registerForNotify() has been deprecated and replaced with subscribe() / unsubscribe().
       *  Subscribe parameter defaults are: notifications=true, notifyCallback=nullptr, response=false.
       *  Unsubscribe parameter defaults are: response=false.
       */
      if(bleDevices[devNr].pChr->canNotify())
      {
        //if(!pChr->registerForNotify(notifyCB))
        if(!bleDevices[devNr].pChr->subscribe(true, notifyCB))
        {
          /** Disconnect if subscribe failed */
          pClient->disconnect();
          return false;
        }
      }
      else if(bleDevices[devNr].pChr->canIndicate())
      {
        /** Send false as first argument to subscribe to indications instead of notifications */
        //if(!pChr->registerForNotify(notifyCB, false))
        if(!bleDevices[devNr].pChr->subscribe(false, notifyCB))
        {
          /** Disconnect if subscribe failed */
          pClient->disconnect();
          return false;
        }
      }
    }
  }
  else
  {
    ESP_LOGI(TAG, "Service not found.");
  }

  ESP_LOGI(TAG, "Done with this device!");
  return true;
}








/**
 * Class BleHandler
 */
BleHandler::BleHandler()
{
};

void BleHandler::init()
{
  u8_mScanAndNotConnectTimer=0;
  timer_startScan=0;
  startManualScan=false;

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    bleDevices[i].doConnect = btDoConnectionIdle;
    bleDevices[i].doDisconnect = false;
    bleDevices[i].isConnect = false;
    bleDevices[i].macAdr = "";
    bleDevices[i].deviceTyp = ID_BT_DEVICE_NB;
    setBmsLastDataMillis(i,0);

    for(uint8_t n=0;n<24;n++)
    {
      setBmsCellVoltage(i,n,0);
    }
    
  }

  NimBLEDevice::init("");
  //NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(45);
  pBLEScan->setWindow(15);
  pBLEScan->setActiveScan(true);
  pBLEScan->setMaxResults(BT_SCAN_RESULTS);

  pBLEScan->clearResults();
  //pBLEScan->start(1);   
};

void BleHandler::startScan()
{
  startManualScan=true;
}

bool BleHandler::isScanFinish()
{
  if(btScanIsRunning)
  {
    return false;
  }
  else
  {
    return true;
  }
}


void BleHandler::run()
{
  static uint8_t i;
  boolean doStartBtScan=false; 

  //BT Devices verbinden 
  for(i=0;i<BT_DEVICES_COUNT;i++)
  {
    //Nur wenn nicht nach BT Devices gesucht wird
    if(!btScanIsRunning) 
    {
      //Überprüfen ob BT-Devices gesucht werden müssen; ggf. suche starten
      if(webSettings.getInt(ID_PARAM_SS_BTDEV,0,i,0)>ID_BT_DEVICE_NB) //Wenn ein Device parametriert ist
      {
        if(bleDevices[i].isConnect==false && bleDevices[i].doConnect==btDoConnectionIdle) //Wenn es nicht verbunden ist
        {
          //Suche starten
          ESP_LOGD(TAG, "doStartBtScan -> Device %i",i);
          doStartBtScan = true;
        }
      }

      if(bleDevices[i].doConnect == btDoConnect)
      {
        if(bleDevices[i].deviceTyp==ID_BT_DEVICE_NEEY4A) //Wenn ein NEEY Balancer 4A konfiguriert ist
        {
          bleDevices[i].doConnect = btConnectionSetup; //Verbdinung wird hergestellt

          if(bleNeeyBalancerConnect(i))
          {
            bleDevices[i].pChr->writeValue(NeeyBalancer_getInfo, 20);
          }
          else //Wenn Verbingungsversuch fehgeschlagen
          {
            bleDevices[i].doConnect = btDoConnectionIdle; 
          }
        }
      }
    }
  }
  
  //Manuellen scan starten
  if(startManualScan)
  {
    startManualScan=false;
    if(!btScanIsRunning) doStartBtScan=true;
  }

  //Wenn angefordert, dann starte neuen BT scan
  if(doStartBtScan && !btScanIsRunning)
  {
    ESP_LOGI(TAG, "Starte BT Scan");
    doStartBtScan = false;
    btScanIsRunning = true;
    if(pBLEScan->isScanning())
    {
      ESP_LOGI(TAG, "scan läuft noch");
    }
    else
    {
      u8_mScanAndNotConnectTimer=BT_SCAN_AND_NOT_CONNECT_TIME;
      NimBLEDevice::getScan()->clearResults();
      NimBLEDevice::getScan()->start(0,scanCompleteCB);
    }
  }

  //Stop den Scan, wenn die Ergebnisliste voll ist und x Sekunden kein Device mehr verbunden wurde
  if(u8_mScanAndNotConnectTimer>0 && btScanIsRunning && pBLEScan->getResults().getCount()>=BT_SCAN_RESULTS)
  {
    u8_mScanAndNotConnectTimer--;
  }
  if(u8_mScanAndNotConnectTimer==1)
  {
    u8_mScanAndNotConnectTimer=0;
    ESP_LOGD(TAG, "Connect Timeout -> Stoppe Scan!");
    pBLEScan->stop(); 
    btScanIsRunning=false;
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
        
    btDevScanResult += "</tr>";
  }
  
  btDevScanResult += "<table>";

  return btDevScanResult;
}


bool BleHandler::bmsIsConnect(uint8_t devNr)
{
  return bleDevices[devNr].isConnect;
}
