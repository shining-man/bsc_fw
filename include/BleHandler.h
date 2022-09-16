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


extern float   bmsCellVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][24];
extern float   bmsCellResistance[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][24];
extern float   bmsTotalVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];
extern float   bmsSingleVoltage[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];
extern float   bmsTotalCurrent[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT]; //Kann von einem reinen Balancer nicht geliefert werden, was dann?
extern uint8_t bmsMaxVoltageCellNumber[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];
extern uint8_t bmsMinVoltageCellNumber[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];
extern uint8_t bmsIsBalancingActive[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];
extern float   bmsBalancingCurrent[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];
extern float   bmsTempature[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT][3];
extern uint8_t bmsErrorBitmask[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];
extern uint8_t bmsNtcWarnings[BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT];

//static const uint8_t bleDevicesCount = 5;


enum btDoConnectEnums {btDoConnect, btConnectionSetup, btDoConnectionIdle};

struct bleDevice {
  bool isConnect;
  btDoConnectEnums doConnect;
  bool doDisconnect;
  String macAdr;
  String deviceTyp;
  //unsigned long lastDataMillis;
  
  NimBLERemoteCharacteristic* pChr;

  //NimBLEAdvertisedDevice* advDevice;
};






class BleHandler {
public:
  BleHandler();
  

  void init();
  void run();
  std::string getBtScanResult();
  //static unsigned long getLastDataMillis(uint8_t devNr);
  static bool bmsIsConnect(uint8_t devNr);

  void startScan();
  bool isScanFinish();

private:
  uint8_t timer_startScan;
  bool startManualScan;
  

};

#endif
