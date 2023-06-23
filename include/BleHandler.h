// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef BleHandler_h
#define BleHandler_h

#include "Arduino.h"
#include "NimBLEDevice.h"
#include "WebSettings.h"
#include "defines.h"
#include "BmsData.h"



enum btDoConnectEnums {btDoConnect, btConnectionSetup, btDoConnectionIdle, btDoConnectionWaitStart};
enum e_btBalancerOnOff {e_BalancerWaitForCmd, e_BalancerChangeToOff, e_BalancerIsOff, e_BalancerChangeToOn, e_BalancerIsOn};

struct bleDevice {
  bool isConnect;
  btDoConnectEnums doConnect;
  String macAdr;
  uint8_t deviceTyp;
  NimBLERemoteCharacteristic* pChr;
  uint8_t sendDataStep;
  e_btBalancerOnOff balancerOn;
};


class BleHandler {
public:
  BleHandler();
  
  void init();
  void start();
  void stop();
  void run();
  std::string getBtScanResult();
  static uint8_t bmsIsConnect(uint8_t devNr);

  void startScan();
  bool isScanFinish();
  static bool isNotAllDeviceConnectedOrScanRunning();
  void sendDataToNeey();
  void readDataFromNeey();
  static void setBalancerState(uint8_t u8_devNr, boolean bo_state);

private:
  uint8_t timer_startScan;
  bool bo_mStartManualScan;
  
  bool handleConnectionToDevices();
  void handleDisconnectionToDevices();
};

#endif
