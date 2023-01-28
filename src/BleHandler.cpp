// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "BleHandler.h"
#include "log.h"

static const char *TAG = "BLE_HANDLER";

bool bleNeeyBalancerConnect(uint8_t deviceNr);

static bleDevice bleDevices[BT_DEVICES_COUNT];
NimBLEScan* pBLEScan;
NimBLEAdvertisedDevice* advDevice;

WebSettings webSettings;

static boolean bo_mBleHandlerRunning = false; 
static boolean bo_mBtScanIsRunning = false; 
static boolean bo_mBtNotAllDeviceConnectedOrScanRunning = false;
uint8_t u8_mScanAndNotConnectTimer;


//NEEY
//#define NEEY_ZYCLIC_SEND_TIME 60   // 0=deaktiviert
byte NeeyBalancer_getInfo[20] PROGMEM = {0xaa, 0x55, 0x11, 0x01, 0x02, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf9, 0xff}; 
NimBLEUUID NeyyBalancer4A_serviceUUID("0000ffe0-0000-1000-8000-00805f9b34fb");
NimBLEUUID NeyyBalancer4A_charUUID   ("0000ffe1-0000-1000-8000-00805f9b34fb");


class ClientCallbacks : public NimBLEClientCallbacks
{
  void onConnect(NimBLEClient* pClient)
  {
    // interval 1,25ms; timeout 10ms
    pClient->updateConnParams(800,800,5,1500); //120,120,5,150

    String devMacAdr = pClient->getPeerAddress().toString().c_str();
    ESP_LOGI(TAG, "onConnect() %s", devMacAdr.c_str());

    for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
    {
      if(bleDevices[i].macAdr.equals(devMacAdr))
      {
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
    ESP_LOGI(TAG, "onDisconnect() %s", devMacAdr.c_str());
    #endif

    for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
    {
      if(bleDevices[i].macAdr.equals(devMacAdr.c_str()))
      {
        ESP_LOGI(TAG, "onDisconnect() dev=%i, mac=%s", i, devMacAdr.c_str());
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
    ESP_LOGD(TAG, "onConnParamsUpdateRequest(): itvl_min=%i itvl_max=%i latency=%i supervision_timeout=%i",
      params->itvl_min,params->itvl_max,params->latency,params->supervision_timeout);
    #endif

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
    ESP_LOGD(TAG, "onConnParamsUpdateRequest(): true");
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
    #ifdef BT_DEBUG
    ESP_LOGI(TAG, "BT device found()");
    #endif

    //Device gefunden
    devMacAdr = advertisedDevice->getAddress().toString();

    for(uint8_t i=0; i<BT_DEVICES_COUNT; i++)
    {    
      if(!webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0).equals(""))
      {
        if (webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0).equals(devMacAdr.c_str()) && webSettings.getString(ID_PARAM_SS_BTDEV,0,i,0).equals(String(ID_BT_DEVICE_NB))==false)
        {
          ESP_LOGI(TAG, "Dev found: i=%i, mac=%s -> scan stop", i, webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0).c_str());

          NimBLEDevice::getScan()->stop();
        
          advDevice = advertisedDevice;   
          bleDevices[i].deviceTyp = (uint8_t)webSettings.getInt(ID_PARAM_SS_BTDEV,0,i,0);
          bleDevices[i].macAdr = webSettings.getString(ID_PARAM_SS_BTDEVMAC,0,i,0);
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

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleDevices[i].macAdr.equals(notifyMacAdr.c_str()))
    {
      //ESP_LOGI(TAG,"NEEY RX len=%i", length);

      //Daten kopieren
      NeeyBalancer::neeyBalancerCopyData(i, pData, length);     
    }
  }
}


// Callback invoked when scanning has completed.
void scanCompleteCB(NimBLEScanResults scanResults)
{
	
} 

// Create a single global instance of the callback class to be used by all clients
static ClientCallbacks clientCB;

bool btDeviceConnect(uint8_t devNr)
{
  #ifdef BT_DEBUG
  ESP_LOGI(TAG, "btDeviceConnect()");
  #endif

  NimBLEClient* pClient = nullptr;
  NimBLEUUID serviceUUID;
  NimBLEUUID charUUID;

  switch(bleDevices[devNr].deviceTyp)
  {
    case ID_BT_DEVICE_NEEY4A:
      serviceUUID=NeyyBalancer4A_serviceUUID;
      charUUID=NeyyBalancer4A_charUUID;
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
      if(!pClient->connect(advDevice, false))
      {
        ESP_LOGW(TAG, "Reconnect failed: dev=%i",devNr);
        return false;
      }
      #ifdef BT_DEBUG
      ESP_LOGI(TAG, "Reconnect: dev=%i",devNr);
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
      ESP_LOGW(TAG, "Max clients reached - no more connections available");
      return false;
    }

    pClient = NimBLEDevice::createClient();
    #ifdef BT_DEBUG
    ESP_LOGI(TAG, "New client created");
    #endif
    pClient->setClientCallbacks(&clientCB, false);

    // Interval *1.25ms; timeout *10ms
    pClient->setConnectionParams(12,12,0,51);
    
    // Set how long we are willing to wait for the connection to complete (seconds)
    pClient->setConnectTimeout(5);

    if (!pClient->connect(advDevice))
    {
      // Created a client but failed to connect, don't need to keep it as it has no data
      ESP_LOGW(TAG, "Failed to connect, deleted client; dev=%i",devNr);
      NimBLEDevice::deleteClient(pClient);
      return false;
    }
  }

  if(!pClient->isConnected())
  {
    if (!pClient->connect(advDevice))
    {
      ESP_LOGW(TAG, "Failed to connect; dev=%i", devNr);
      return false;
    }
  }
  #ifdef BT_DEBUG
  ESP_LOGI(TAG, "Connected to: %s, RSSI: %i",pClient->getPeerAddress().toString().c_str(),pClient->getRssi());
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
        ESP_LOGI(TAG, "can read: %s, Value: %s",bleDevices[devNr].pChr->getUUID().toString().c_str(),bleDevices[devNr].pChr->readValue().c_str());
        #endif
      }

      if(bleDevices[devNr].pChr->canWrite())
      {
        #ifdef BT_DEBUG
        ESP_LOGI(TAG, "can write");
        #endif
      }

      //Subscribe
      if(bleDevices[devNr].pChr->canNotify())
      {
        bool bo_lSubscribeRet=false;
        switch(bleDevices[devNr].deviceTyp)
        {
          case ID_BT_DEVICE_NEEY4A:
            bo_lSubscribeRet=bleDevices[devNr].pChr->subscribe(true, notifyCB_NEEY);
            break;
        }

        if(!bo_lSubscribeRet)
        {
          // Disconnect if subscribe failed
          pClient->disconnect();
          return false;
        }
      }
      /*else if(bleDevices[devNr].pChr->canIndicate())
      {
        // Send false as first argument to subscribe to indications instead of notifications
        if(!bleDevices[devNr].pChr->subscribe(false, notifyCB_NEEY))
        {
          // Disconnect if subscribe failed
          pClient->disconnect();
          return false;
        }
      }*/
    }
  }
  else
  {
    ESP_LOGW(TAG, "Service not found.");
  }

  #ifdef BT_DEBUG
  ESP_LOGI(TAG, "Device connected; dev=%i",devNr);
  #endif
  return true;
}


void btDeviceDisconnect(uint8_t devNr)
{
  //#ifdef BT_DEBUG
  ESP_LOGI(TAG, "btDeviceDisconnect() devNr=%i",devNr);
  //#endif

  NimBLEClient* pClient = nullptr;

  for(uint8_t i=0;i<7;i++)
  {
    if(NimBLEDevice::getClientListSize())
    {
      //uint8_t address[6] = bleDevices[i].macAdr;
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







/**
 * Class BleHandler
 */
BleHandler::BleHandler()
{
};


//NimBLEAdvertisedDeviceCallbacks* myAdvertisedDeviceCallback;
void BleHandler::init()
{
  if(bo_mBleHandlerRunning)return;

  u8_mScanAndNotConnectTimer=0;
  timer_startScan=0;
  bo_mStartManualScan=false;
  bo_mBtScanIsRunning=false;
  bo_mBtNotAllDeviceConnectedOrScanRunning=false;

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    bleDevices[i].doConnect = btDoConnectionIdle;
    bleDevices[i].isConnect = false;
    //bleDevices[i].u16_zyclicWriteTimer=0;
    bleDevices[i].macAdr = "";
    bleDevices[i].deviceTyp = ID_BT_DEVICE_NB;
    setBmsLastDataMillis(i,0);

    for(uint8_t n=0;n<24;n++)
    {
      setBmsCellVoltage(i,n,0);
    }
    
  }

  NimBLEDevice::init("");
  //NimBLEDevice::setPower(ESP_PWR_LVL_P9); // +9db

  pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(45);
  pBLEScan->setWindow(15);
  pBLEScan->setActiveScan(true);
  pBLEScan->setMaxResults(BT_SCAN_RESULTS);
  pBLEScan->clearResults();

  //bo_mBleHandlerRunning=true;
};

void BleHandler::start()
{
  bo_mBleHandlerRunning=true;
}

void BleHandler::stop()
{
  btDeviceDisconnect(0);
  
  /*if(bo_mBleHandlerRunning)
  {
    pBLEScan->stop();
    pBLEScan->clearResults();
    pBLEScan->clearDuplicateCache();
    NimBLEDevice::deinit(false);
  }*/
  /*for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    bleDevices[i].pChr=nullptr;
  }*/

  bo_mBleHandlerRunning=false;
  bo_mBtNotAllDeviceConnectedOrScanRunning=false;
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


void BleHandler::run()
{
  if(!bo_mBleHandlerRunning)return;

  #ifdef BT_DEBUG
  ESP_LOGD(TAG, "run() bo_mBtScanIsRunning=%i",bo_mBtScanIsRunning);
  #endif

  //
  boolean bo_lDoStartBtScan=handleConnectionToDevices();

  //Manuellen scan starten
  if(bo_mStartManualScan)
  {
    bo_mStartManualScan=false;
    if(!bo_mBtScanIsRunning) bo_lDoStartBtScan=true;
  }

  //Wenn angefordert, dann starte neuen BT scan
  if(bo_lDoStartBtScan && !bo_mBtScanIsRunning)
  {
    ESP_LOGI(TAG, "Starte BT Scan");
    bo_mBtNotAllDeviceConnectedOrScanRunning=true;
    bo_lDoStartBtScan = false;
    bo_mBtScanIsRunning = true;
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

  #ifdef BT_DEBUG
  ESP_LOGD(TAG, "run() u8_mScanAndNotConnectTimer=%i, bo_mBtScanIsRunning=%i, scanResCount=%i",u8_mScanAndNotConnectTimer,bo_mBtScanIsRunning,pBLEScan->getResults().getCount());
  #endif

  //Stop den Scan, wenn die Ergebnisliste voll ist und x Sekunden kein Device mehr verbunden wurde
  if(u8_mScanAndNotConnectTimer>0 && bo_mBtScanIsRunning && pBLEScan->getResults().getCount()>=BT_SCAN_RESULTS)
  {
    u8_mScanAndNotConnectTimer--;
  }
  if(u8_mScanAndNotConnectTimer==1)
  {
    u8_mScanAndNotConnectTimer=0;
    ESP_LOGD(TAG, "Connect Timeout -> Stoppe Scan!");
    pBLEScan->stop(); 
    bo_mBtNotAllDeviceConnectedOrScanRunning=false;
    bo_mBtScanIsRunning=false;
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
      uint8_t u8_lBtDevType = webSettings.getInt(ID_PARAM_SS_BTDEV,0,i,0);
      //Überprüfen ob BT-Devices gesucht werden müssen; ggf. suche starten
      if(u8_lBtDevType>ID_BT_DEVICE_NB) //Wenn ein Device parametriert ist
      {
        if(bleDevices[i].isConnect==false && bleDevices[i].doConnect==btDoConnectionIdle) //Wenn es nicht verbunden ist
        {
          //Suche starten
          ESP_LOGI(TAG, "Suche device %i",i);
          bo_lDoStartBtScan = true;
        }
      }

      if(bleDevices[i].doConnect == btDoConnect)
      {
        bleDevices[i].doConnect = btConnectionSetup; //Verbdinung wird hergestellt

        if(btDeviceConnect(i))
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
        if((u8_lBtDevType==ID_BT_DEVICE_NB) || (u8_lBtDevType>ID_BT_DEVICE_NB && bleDevices[i].isConnect)){u8_lBtConnStatus|=(1<<i);}
      }
    }

    //Wenn alle Geräte verbunden sind
    #ifdef BT_DEBUG
    ESP_LOGI(TAG,"u8_lBtConnStatus=%i",u8_lBtConnStatus);
    #endif
    if(u8_lBtConnStatus==0x7F)
    {
      bool bo_lAllDevWrite=false;
      for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
      {
        //ESP_LOGI(TAG,"BT write (%i) doC=%i",i,bleDevices[i].doConnect);
        //if(bleDevices[i].u16_zyclicWriteTimer>1)bleDevices[i].u16_zyclicWriteTimer--;

        if(bleDevices[i].doConnect==btDoConnectionWaitStart)
        {
          //ESP_LOGI(TAG,"BT write (%i) typ=%i",i,bleDevices[i].deviceTyp);
          switch(bleDevices[i].deviceTyp)
          {
            case ID_BT_DEVICE_NEEY4A:
              ESP_LOGI(TAG,"BT write dev=%i",i);
              //bleDevices[i].u16_zyclicWriteTimer=NEEY_ZYCLIC_SEND_TIME;
              bleDevices[i].pChr->writeValue(NeeyBalancer_getInfo, 20);
              bleDevices[i].doConnect = btDoConnectionIdle; 
              break;
          }
          bo_lAllDevWrite=true; 
        }
      }
      if(bo_lAllDevWrite)bo_mBtNotAllDeviceConnectedOrScanRunning=false; 
    }
  }

  return bo_lDoStartBtScan;
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
