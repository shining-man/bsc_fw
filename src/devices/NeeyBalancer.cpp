// Copyright (c) 2022 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "devices/NeeyBalancer.h"
#include "BmsData.h"
#include "WebSettings.h"
#include "Utility.h"

static const char* TAG = "NEEY";


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

constexpr const char* textNeeySet_displayFlex = "display;flex|";
constexpr const char* textNeeySet_displayNone = "display;none|";
constexpr const char* textNeeySet_btn0 = "btn1;0";
constexpr const char* textNeeySet_btn1 = "btn1;1";

constexpr const char* textNeeySet_NCM = "NCM";
constexpr const char* textNeeySet_LFP = "LFP";
constexpr const char* textNeeySet_LTO = "LTO";
constexpr const char* textNeeySet_PbAc = "PbAc";

constexpr const char* textNeeySet_EIN = "EIN";
constexpr const char* textNeeySet_AUS = "AUS";

void NeeyBalancer::getNeeyReadbackDataAsString(std::string &value)
{
  std::string  str_lText="";
  uint8_t u8_lValue=0;
  uint8_t devTyp;

  //ID_PARAM_NEEY_BUZZER //not use
  //ID_PARAM_NEEY_BALANCER_ON

  /*if(u8_neeySendStep>0) //Write Data, please wait
  {
    value += textNeeySet_displayFlex;  //spinner visible on
    value += textNeeySet_btn0;         //disable
    //Hier kein "|" anhängen, da dies im nächsten Schritte bei den BMS Daten erfolgt
  }
  else
  {
    value += textNeeySet_displayNone;  //spinner visible off
    value += textNeeySet_btn1);        //enable
    //Hier kein "|" anhängen, da dies im nächsten Schritte bei den BMS Daten erfolgt
  }*/

  value += textNeeySet_displayNone;  //spinner visible off
  value += textNeeySet_btn0;         //disable

  for(uint8_t i=0;i<5;i++)
  {
    devTyp = WebSettings::getInt(ID_PARAM_SS_BTDEV,i,DT_ID_PARAM_SS_BTDEV);
    if(devTyp==ID_BT_DEVICE_NEEY_GW_24S4EB || devTyp==ID_BT_DEVICE_NEEY_EK_24S4EB)
    {
      value += "|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_START_VOLTAGE, i)) + ";";
      value += floatToString(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_START_VOL),3) + " V|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_MAX_BALANCE_CURRENT, i)) + ";";
      value += floatToString(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_MAX_BAL_CURRENT),3) + " A|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_SLEEP_VOLTAGE, i)) + ";";
      value += floatToString(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_SLEEP_VOLTAGE),3) + " V|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_EQUALIZATION_VOLTAGE, i)) + ";";
      value += floatToString(NeeyBalancer::neeyGetReadbackDataFloat(i, NEEYBAL4A_FUNC_SETTING_EQUALIZATION_VOLTAGE),3) + " V|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_CELLS, i)) + ";";
      value += std::to_string(NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_CELLS)) + "|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_BAT_TYPE, i)) + ";";
      u8_lValue = NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BAT_TYPE);
      str_lText="";
      if(u8_lValue==1) str_lText = textNeeySet_NCM;
      else if(u8_lValue==2) str_lText = textNeeySet_LFP;
      else if(u8_lValue==3) str_lText = textNeeySet_LTO;
      else if(u8_lValue==4) str_lText = textNeeySet_PbAc;
      else std::to_string(u8_lValue);
      value += str_lText + "|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_BAT_CAPACITY, i)) + ";";
      value += std::to_string(NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BAT_CAP)) + " Ah|";

      value += "s" + std::to_string(WebSettings::getParmId(ID_PARAM_NEEY_BALANCER_ON, i)) + ";";
      u8_lValue = NeeyBalancer::neeyGetReadbackDataInt(i, NEEYBAL4A_FUNC_SETTING_BALLANCER_ON_OFF);
      str_lText="";
      if(u8_lValue==0) str_lText = textNeeySet_EIN;
      else if(u8_lValue==1) str_lText = textNeeySet_AUS;
      value += str_lText;

    }
  }
}