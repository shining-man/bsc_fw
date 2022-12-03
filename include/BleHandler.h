// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef BleHandler_h
#define BleHandler_h

#include "Arduino.h"
#include "NimBLEDevice.h"
#include "WebSettings.h"
#include "devices/NeeyBalancer.h"
#include "defines.h"
#include "BmsData.h"


enum btDoConnectEnums {btDoConnect, btConnectionSetup, btDoConnectionIdle};

struct bleDevice {
  bool isConnect;
  btDoConnectEnums doConnect;
  bool doDisconnect;
  String macAdr;
  String deviceTyp;
  NimBLERemoteCharacteristic* pChr;
};


class BleHandler {
public:
  BleHandler();
  
  void init();
  void run();
  std::string getBtScanResult();
  static bool bmsIsConnect(uint8_t devNr);

  void startScan();
  bool isScanFinish();

private:
  uint8_t timer_startScan;
  bool startManualScan;
  
};

#endif
