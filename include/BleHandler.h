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


enum btDoConnectEnums {btDoConnect, btConnectionSetup, btDoConnectionIdle, btDoConnectionWaitStart};

struct bleDevice {
  bool isConnect;
  btDoConnectEnums doConnect;
  //uint16_t u16_zyclicWriteTimer;
  String macAdr;
  uint8_t deviceTyp;
  NimBLERemoteCharacteristic* pChr;
};


class BleHandler {
public:
  BleHandler();
  
  void init();
  void start();
  void stop();
  void run();
  std::string getBtScanResult();
  static bool bmsIsConnect(uint8_t devNr);

  void startScan();
  bool isScanFinish();
  static bool isNotAllDeviceConnectedOrScanRunning();

private:
  uint8_t timer_startScan;
  bool bo_mStartManualScan;
  
  bool handleConnectionToDevices();
};

#endif
