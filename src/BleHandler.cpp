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

static boolean bleHandlerRunning = false; 
static boolean btScanIsRunning = false; 
uint8_t u8_mScanAndNotConnectTimer;
std::string notifyMacAdr;


//NEEY
#define NEEY_ZYCLIC_SEND_TIME 60   // 0=deaktiviert
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
     
          btScanIsRunning = false; //Scan beendet
        }
      }
    }
  }
};


// Notification / Indication receiving handler callback
void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  notifyMacAdr = pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(bleDevices[i].macAdr.equals(notifyMacAdr.c_str()))
    {
      //ESP_LOGI(TAG,"RX len=%i", length);
      //setBmsLastDataMillis(i,millis());

      if(bleDevices[i].deviceTyp==ID_BT_DEVICE_NEEY4A)
      {
        //Daten kopieren
        NeeyBalancer::neeyBalancerCopyData(i, pData, length);
        //ESP_LOGI(TAG,"NEEY RX len=%i", length);
      }
    }
  }
}


/**
 * Callback invoked when scanning has completed.
 */
void scanCompleteCB(NimBLEScanResults scanResults)
{
	
} 

// Create a single global instance of the callback class to be used by all clients
static ClientCallbacks clientCB;

bool bleNeeyBalancerConnect(uint8_t devNr)
{
  #ifdef BT_DEBUG
  ESP_LOGI(TAG, "bleNeeyBalancerConnect()");
  #endif

  NimBLEClient* pClient = nullptr;

  // Check if we have a client we should reuse first 
  if(NimBLEDevice::getClientListSize())
  {
    /** Special case when we already know this device, we send false as the
     *  second argument in connect() to prevent refreshing the service database.
     *  This saves considerable time and power.
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
    /** We don't already have a client that knows this device,
     *  we will check for a client that is disconnected that we can use.
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

  pSvc = pClient->getService(NeyyBalancer4A_serviceUUID);
  if(pSvc)
  {     
    bleDevices[devNr].pChr = pSvc->getCharacteristic(NeyyBalancer4A_charUUID);

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
        if(!bleDevices[devNr].pChr->subscribe(true, notifyCB))
        {
          // Disconnect if subscribe failed
          pClient->disconnect();
          return false;
        }
      }
      /*else if(bleDevices[devNr].pChr->canIndicate())
      {
        // Send false as first argument to subscribe to indications instead of notifications
        if(!bleDevices[devNr].pChr->subscribe(false, notifyCB))
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








/**
 * Class BleHandler
 */
BleHandler::BleHandler()
{
};

void BleHandler::init()
{
  if(bleHandlerRunning)return;

  u8_mScanAndNotConnectTimer=0;
  timer_startScan=0;
  startManualScan=false;
  btScanIsRunning=false;

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

  bleHandlerRunning=true;
};

void BleHandler::stop()
{
  bleHandlerRunning=false;
  if(pBLEScan)
  {
    pBLEScan->stop();
    pBLEScan->clearResults();
  }
  NimBLEDevice::deinit(true);
}

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

bool BleHandler::isScanRuning()
{
  return btScanIsRunning;
}


void BleHandler::run()
{
  if(!bleHandlerRunning)return;

  static uint8_t i;
  boolean bo_lDoStartBtScan=false; 
  uint8_t u8_lBtConnStatus=0;

  #ifdef BT_DEBUG
  ESP_LOGD(TAG, "run() btScanIsRunning=%i",btScanIsRunning);
  #endif

  //Nur wenn nicht nach BT Devices gesucht wird
  if(!btScanIsRunning) 
  {
    //BT Devices verbinden 
    for(i=0;i<BT_DEVICES_COUNT;i++)
    {
      uint8_t u8_lBtDevType = webSettings.getInt(ID_PARAM_SS_BTDEV,0,i,0);
      //Überprüfen ob BT-Devices gesucht werden müssen; ggf. suche starten
      if(u8_lBtDevType>ID_BT_DEVICE_NB) //Wenn ein Device parametriert ist
      {
        if(bleDevices[i].isConnect==false && bleDevices[i].doConnect==btDoConnectionIdle) //Wenn es nicht verbunden ist
        {
          //Suche starten
          ESP_LOGD(TAG, "Suche device %i",i);
          bo_lDoStartBtScan = true;
        }
      }

      if(bleDevices[i].doConnect == btDoConnect)
      {
        bleDevices[i].doConnect = btConnectionSetup; //Verbdinung wird hergestellt

        if(bleNeeyBalancerConnect(i))
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
      for(i=0;i<BT_DEVICES_COUNT;i++)
      {
        //ESP_LOGI(TAG,"BT write (%i) doC=%i",i,bleDevices[i].doConnect);
        //if(bleDevices[i].u16_zyclicWriteTimer>1)bleDevices[i].u16_zyclicWriteTimer--;

        if(bleDevices[i].doConnect==btDoConnectionWaitStart)
        {
          //ESP_LOGI(TAG,"BT write (%i) typ=%i",i,bleDevices[i].deviceTyp);
          if(bleDevices[i].deviceTyp==ID_BT_DEVICE_NEEY4A)
          {
            //bleDevices[i].u16_zyclicWriteTimer=NEEY_ZYCLIC_SEND_TIME;
            ESP_LOGI(TAG,"BT write dev=%i",i);
            bleDevices[i].pChr->writeValue(NeeyBalancer_getInfo, 20);
            bleDevices[i].doConnect = btDoConnectionIdle; 
          }  
        }
        /*else if(bleDevices[i].isConnect && bleDevices[i].u16_zyclicWriteTimer==1)
        {
          if(bleDevices[i].deviceTyp==ID_BT_DEVICE_NEEY4A)
          {
            bleDevices[i].u16_zyclicWriteTimer=NEEY_ZYCLIC_SEND_TIME;
            #ifdef BT_DEBUG
            ESP_LOGI(TAG,"BT write zyclic dev=%i",i);
            #endif
            bleDevices[i].pChr->writeValue(NeeyBalancer_getInfo, 20);
          }
        }*/
      }
    }
  }


  //Manuellen scan starten
  if(startManualScan)
  {
    startManualScan=false;
    if(!btScanIsRunning) bo_lDoStartBtScan=true;
  }

  //Wenn angefordert, dann starte neuen BT scan
  if(bo_lDoStartBtScan && !btScanIsRunning)
  {
    ESP_LOGI(TAG, "Starte BT Scan");
    bo_lDoStartBtScan = false;
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

  #ifdef BT_DEBUG
  ESP_LOGD(TAG, "run() u8_mScanAndNotConnectTimer=%i, btScanIsRunning=%i, scanResCount=%i",u8_mScanAndNotConnectTimer,btScanIsRunning,pBLEScan->getResults().getCount());
  #endif

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
