// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "devices/NeeyBalancer.h"
#include "BmsData.h"
#include "WebSettings.h"

static const char* TAG = "NEEY";

byte NeeyBalancer_cmdBalanceOn[20] PROGMEM =  {0xaa, 0x55, 0x11, 0x00, 0x05, 0x0d, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf3, 0xff};
byte NeeyBalancer_cmdBalanceOff[20] PROGMEM = {0xaa, 0x55, 0x11, 0x00, 0x05, 0x0d, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf2, 0xff};


static uint8_t u8_neeySendStep=0;

NeeyBalancer::NeeyBalancer()
{
};

void NeeyBalancer::init()
{
}

void NeeyBalancer::neeyBalancerCopyData(uint8_t devNr, uint8_t* pData, size_t length)
{
  if((pData[0]==0x55 && pData[1]==0xAA && pData[2]==0x11 && pData[3]==0x01 && pData[4]==0x04 && pData[5]==0x00 && pData[6]==0x64))
  {
    //BSC_LOGI(TAG,"RX");
    //log_print_buf(pData, length);

    //String log="";
    //for(uint8_t i=0;i<length;i++){log+=String(pData[i], HEX); log+=" ";}
    //BSC_LOGI(TAG,"RX: %s",log.c_str()); log="";

    bmsDataSemaphoreTake();
    memcpy(getBmsSettingsReadback(devNr), &pData[8], 32);
    bmsDataSemaphoreGive();

    return;
  }

  if(length==241)
  {
    // 0     2   0x55 0xAA              Header
    // 2     1   0x11                   Device address
    // 3     1   0x01                   Function (read)
    // 4     2   0x02 0x00              Command (cell info)
    if(!(pData[0]==0x55 && pData[1]==0xAA && pData[2]==0x11 && pData[3]==0x01 && pData[4]==0x02 && pData[5]==0x00)) return;
    if((millis()-getBmsLastDataMillis(devNr))<500) return; //Nur alle 500ms eine neue Nachricht akzeptieren

    float    f_lTmpValue;
    uint32_t u32_lTmpValue;
    //Cellspannung
    for(uint8_t i=0;i<24;i++)
    {
      memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_CELVOLTAGE+(i*4), 4);
      setBmsCellVoltage(devNr,i,(uint16_t)(f_lTmpValue*1000));
    }

    //Avg. Voltage
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_AVERAGEVOLTAGE, 4);
    setBmsAvgVoltage(devNr,(uint16_t)(f_lTmpValue*1000));

    //Delta Cell Voltage
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_DELTACELLVOLTAGE, 4);
    setBmsMaxCellDifferenceVoltage(devNr,(uint16_t)(f_lTmpValue*1000));
   
    /*
    Errors:
    STATUS_OK                0
    STATUS_CELL_OVP          1   //bit0 single cell overvoltage protection 
    STATUS_CELL_UVP          2   //bit1 single cell undervoltage protection    
    STATUS_PACK_OVP          4   //bit2 whole pack overvoltage protection 
    STATUS_PACK_UVP          8   //bit3 Whole pack undervoltage protection     
    STATUS_CHG_OTP          16   //bit4 charging over-temperature protection 
    STATUS_CHG_UTP          32   //bit5 charging low temperature protection 
    STATUS_DSG_OTP          64   //bit6 Discharge over temperature protection  
    STATUS_DSG_UTP         128   //bit7 discharge low temperature protection   
    STATUS_CHG_OCP         256   //bit8 charging overcurrent protection 
    STATUS_DSG_OCP         512   //bit9 Discharge overcurrent protection       
    STATUS_SHORT_CIRCUIT  1024   //bit10 short circuit protection              
    STATUS_AFE_ERROR      2048   //bit11 Front-end detection IC error 
    STATUS_SOFT_LOCK      4096   //bit12 software lock MOS 
    */
    memcpy(&u32_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_ERRCELLOV, 4);
  

    setBmsErrors(devNr, u32_lTmpValue);
 
    struct bmsData_s *p_lBmsData = getBmsData();
    bmsDataSemaphoreTake();
    //memcpy(&p_lBmsData->bmsCellResistance[devNr][0], pData+OFFSET_NEEYBAL4A_DATA0x2_CELLRESISTANCE, 4*24); 
    memcpy(&p_lBmsData->bmsTotalVoltage[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_TOTALVOLTAGE, 4);
    memcpy(&p_lBmsData->bmsMaxVoltageCellNumber[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_MAXVOLTCELLNR, 1);
    memcpy(&p_lBmsData->bmsMinVoltageCellNumber[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_MINVOLTCELLNR, 1);
    memcpy(&p_lBmsData->bmsIsBalancingActive[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_BALANCING, 1);
    memcpy(&p_lBmsData->bmsBalancingCurrent[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_BALANCINGCUR, 4);
    memcpy(&p_lBmsData->bmsTempature[devNr][0], pData+OFFSET_NEEYBAL4A_DATA0x2_TEMPERATUR, 4*2);
    bmsDataSemaphoreGive();

    uint16_t f_lMacZellVoltage = getBmsCellVoltage(devNr,getBmsMaxVoltageCellNumber(devNr));
    uint16_t f_lMinZellVoltage = getBmsCellVoltage(devNr,getBmsMinVoltageCellNumber(devNr));
    setBmsMaxCellDifferenceVoltage(devNr, f_lMacZellVoltage-f_lMinZellVoltage);

    setBmsMaxCellVoltage(devNr, f_lMacZellVoltage);
    setBmsMinCellVoltage(devNr, f_lMinZellVoltage);

    setBmsLastDataMillis(devNr,millis());
  }
  else if(length==59)
  {

  }
  else
  {
    //String log="";
    //for(uint8_t i=0;i<length;i++){log+=String(pData[i], HEX); log+=" ";}
    //BSC_LOGI(TAG,"RX: %s",log.c_str()); log="";

    //BSC_LOGI(TAG,"RX");
    //log_print_buf(pData, length);
  }

}



uint8_t NeeyBalancer::neeyBtCrc(uint8_t* data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        crc = crc ^ data[i];
    }
    return crc;
}


void NeeyBalancer::neeyBtBuildSendData(uint8_t* frame, uint8_t byte3, uint8_t cmd, uint8_t func, uint32_t value)
{
    frame[0] = 0xAA;     // start sequence
    frame[1] = 0x55;     // start sequence
    frame[2] = 0x11;     // start sequence
    frame[3] = byte3;    // start sequence
    frame[4] = cmd;      // 
    frame[5] = func;     // 
    frame[6] = 0x14;     // length 
    frame[7] = 0x00;
    frame[8] = value >> 0;
    frame[9] = value >> 8;
    frame[10] = value >> 16;
    frame[11] = value >> 24;
    frame[12] = 0x00;
    frame[13] = 0x00;
    frame[14] = 0x00;
    frame[15] = 0x00;
    frame[16] = 0x00;
    frame[17] = 0x00;
    frame[18] = neeyBtCrc(frame, 18);
    frame[19] = 0xFF;

    //String log="";
    //for(uint8_t i=0;i<20;i++){log+=String(frame[i], HEX); log+=" ";}
    //BSC_LOGI(TAG,"TX: %s",log.c_str()); log="";

    //BSC_LOGI(TAG,"TX");
    //log_print_buf(frame, 20);
}

void NeeyBalancer::neeyBtBuildSendData(uint8_t* frame, uint8_t cmd, uint8_t func, uint32_t value)
{
    neeyBtBuildSendData(frame, 0x0, cmd, func, value);
}

#if 0
void NeeyBalancer::neeyWriteMsg1(NimBLERemoteCharacteristic* pChr)
{
  //{0xaa, 0x55, 0x11, 0x01, 0x01, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfa, 0xff};
  uint8_t  frame[20];
  neeyBtBuildSendData(frame, 0x01, 0x01, 0x0, (uint32_t)0x0); //msg1
  pChr->writeValue(frame, 20, true);
}
#endif

void NeeyBalancer::neeyWriteMsg2(NimBLERemoteCharacteristic* pChr)
{
  uint8_t  frame[20];
  neeyBtBuildSendData(frame, 0x01, 0x04, 0x0, (uint32_t)0x0); //msg2
  pChr->writeValue(frame, 20, true);
}

void NeeyBalancer::neeyBtBuildSendData(uint8_t* frame, uint8_t cmd, uint8_t func, float value)
{
  //{0xaa, 0x55, 0x11, 0x01, 0x04, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff};
  uint32_t ui32_lValue;
  memcpy(&ui32_lValue, &value, 4);

  neeyBtBuildSendData(frame, 0x0, cmd, func, ui32_lValue);
}

void NeeyBalancer::neeySetBalancerOnOff(NimBLERemoteCharacteristic* pChr, boolean u8_state)
{
  if(u8_state) pChr->writeValue(NeeyBalancer_cmdBalanceOn, 20);
  else pChr->writeValue(NeeyBalancer_cmdBalanceOff, 20);
}

bool NeeyBalancer::neeyWriteData(uint8_t btDevNr, NimBLERemoteCharacteristic* pChr)
{
  bool ret=false;

  uint8_t  frame[20];
  uint8_t  u8_lValue;
  uint16_t u16_lValue;
  float    f_lValue;

  //u8_neeySendStep++;
  
  switch(u8_neeySendStep)
  {
    case 1:
      neeyBtBuildSendData(frame, 0x04, 0x0, (uint32_t)0x0);
      pChr->writeValue(frame, 20, true);
      break;

    case 2:
      u8_lValue=WebSettings::getIntFlash(ID_PARAM_NEEY_BALANCER_ON,btDevNr,DT_ID_PARAM_NEEY_BALANCER_ON);
      if(u8_lValue==0) neeySetBalancerOnOff(pChr, false);
      else if(u8_lValue==110) neeySetBalancerOnOff(pChr, true);
      break;

    case 3:
      u8_lValue=WebSettings::getIntFlash(ID_PARAM_NEEY_BAT_TYPE,btDevNr,DT_ID_PARAM_NEEY_BAT_TYPE);
      neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_BAT_TYPE, (uint32_t)u8_lValue);
      pChr->writeValue(frame, 20, true);
      break;

    case 4:
      u8_lValue=WebSettings::getIntFlash(ID_PARAM_NEEY_CELLS,btDevNr,DT_ID_PARAM_NEEY_CELLS);
      neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_CELLS, (uint32_t)u8_lValue);
      pChr->writeValue(frame, 20, true);
      break;

    case 5:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_START_VOLTAGE,btDevNr);
      neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_START_VOL, f_lValue);
      pChr->writeValue(frame, 20, true);
      break;

    case 6:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_MAX_BALANCE_CURRENT,btDevNr);
      neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT, f_lValue);
      pChr->writeValue(frame, 20, true);
      break;

    case 7:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_EQUALIZATION_VOLTAGE,btDevNr);
      neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE, f_lValue);
      pChr->writeValue(frame, 20, true);
      break;

    case 8:
      f_lValue=WebSettings::getFloatFlash(ID_PARAM_NEEY_SLEEP_VOLTAGE,btDevNr);
      neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE, f_lValue);
      pChr->writeValue(frame, 20, true);
      break;

    case 9:
      u16_lValue=WebSettings::getIntFlash(ID_PARAM_NEEY_BAT_CAPACITY,btDevNr,DT_ID_PARAM_NEEY_BAT_CAPACITY);
      neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_BAT_CAP, (uint32_t)u16_lValue);
      pChr->writeValue(frame, 20, true);
      break;

    case 10:
      neeyBtBuildSendData(frame, 0x01, 0x04, 0x0, (uint32_t)0x0); //msg2
      pChr->writeValue(frame, 20, true);
      break;

    case 11:
      neeyBtBuildSendData(frame, 0x04, 0x0, (uint32_t)0x0);
      pChr->writeValue(frame, 20, true);

      //u8_neeySendStep=0;
      ret=true;
      break;
  }

  //NeeyBalancer::neeyBtBuildSendData(frame, NEEYBAL4A_CMD_WRITE, NEEYBAL4A_FUNC_SETTING_BUZZER_MODE, f_lTmpValue);

  return ret;
}

void NeeyBalancer::neeyWriteData_GotoStartStep()
{
  u8_neeySendStep=1;
}


bool NeeyBalancer::neeyWriteData_GotoNextStep()
{
  if(u8_neeySendStep==11)
  {
    u8_neeySendStep=0;
    return true;
  }
  else
  {
    u8_neeySendStep++;
    return false;
  }
}


float NeeyBalancer::neeyGetReadbackDataFloat(uint8_t devNr, uint8_t dataType)
{
  float retValue=0;
  bmsDataSemaphoreTake();

  switch(dataType)
  {
    case NEEYBAL4A_FUNC_SETTING_START_VOL:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[1], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[5], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[9], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[20], 4);
      break;
  }
  
  bmsDataSemaphoreGive();
  return retValue;
}


uint32_t NeeyBalancer::neeyGetReadbackDataInt(uint8_t devNr, uint8_t dataType)
{
  uint32_t retValue=0;
  bmsDataSemaphoreTake();

  switch(dataType)
  {
    case NEEYBAL4A_FUNC_SETTING_CELLS:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[0], 1);
      break;
    case NEEYBAL4A_FUNC_SETTING_BAT_TYPE:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[15], 1);
      break;
    case NEEYBAL4A_FUNC_SETTING_BAT_CAP:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[16], 4);
      break;
    case NEEYBAL4A_FUNC_SETTING_BUZZER_MODE:
      break;
    case NEEYBAL4A_FUNC_SETTING_BALLANCER_ON_OFF:
      memcpy(&retValue, &getBmsSettingsReadback(devNr)[13], 1);
      break;
  }
  
  bmsDataSemaphoreGive();
  return retValue;
}


void NeeyBalancer::getNeeyReadbackDataAsString(String &value)
{
  String  str_lText="";
  uint8_t u8_lValue=0;

  //ID_PARAM_NEEY_BUZZER //not use
  //ID_PARAM_NEEY_BALANCER_ON
  
  if(u8_neeySendStep>0) //Write Data, please wait
  {
    value += F("display;flex|");  //spinner visible on
    value += F("btn1;0");        //disable
    //Hier kein "|" anhängen, da dies im nächsten Schritte bei den BMS Daten erfolgt
  }
  else
  {
    value += F("display;none|");  //spinner visible off
    value += F("btn1;1");        //enable
    //Hier kein "|" anhängen, da dies im nächsten Schritte bei den BMS Daten erfolgt
  }

  for(uint8_t i=0;i<BT_DEVICES_COUNT;i++)
  {
    if(WebSettings::getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV)==ID_BT_DEVICE_NEEY4A)
    {
      value += "|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_START_VOLTAGE, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_START_VOL),3) + " V|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_MAX_BALANCE_CURRENT, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT),3) + " A|";
      
      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_SLEEP_VOLTAGE, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE),3) + " V|";
      
      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_EQUALIZATION_VOLTAGE, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE),3) + " V|";
      
      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_CELLS, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_CELLS)) + "|";    
      
      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_BAT_TYPE, i)) + ";";
      u8_lValue = NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BAT_TYPE);
      str_lText="";
      if(u8_lValue==1) str_lText=F("NCM");
      else if(u8_lValue==2) str_lText=F("LFP");
      else if(u8_lValue==3) str_lText=F("LTO");
      else if(u8_lValue==4) str_lText=F("PbAc");
      else String(u8_lValue);
      value += str_lText + "|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_BAT_CAPACITY, i)) + ";";
      value += String(NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BAT_CAP)) + " Ah|";

      value += "s" + String(WebSettings::getParmId(ID_PARAM_NEEY_BALANCER_ON, i)) + ";";
      u8_lValue = NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BALLANCER_ON_OFF);
      str_lText="";
      if(u8_lValue==0) str_lText=F("AUS");
      else if(u8_lValue==1) str_lText=F("EIN");
      value += str_lText;

    }
  }
}