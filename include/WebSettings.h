// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef WebSettings_h
#define WebSettings_h

#include <Json.h>
#include <WebServer.h>


#define BUILD_PARAM_ID(id, settingNr, groupIdx, listIdx) (id << 20)\
|(settingNr << 14) \
|(groupIdx << 7) \
|(listIdx & 0x7F)


#define HTML_INPUTTEXT                    0
#define HTML_INPUTTEXTAREA                1
#define HTML_INPUTPASSWORD                2
#define HTML_INPUTNUMBER                  3
#define HTML_INPUTFLOAT                   4
#define HTML_INPUTRANGE                   5
#define HTML_INPUTDATE                    6
#define HTML_INPUTTIME                    7
#define HTML_INPUTCOLOR                   8
#define HTML_INPUTSELECT                  9
#define HTML_INPUTCHECKBOX                10
#define HTML_INPUTMULTICHECK              11
#define HTML_INPUTMULTICHECK_COLLAPSIBLE  14
#define HTML_OPTIONGROUP                  12
#define HTML_SEPARATION                   13

#define BACKGROUND_COLOR "#ffffff" 

#define BUTTON_SAVE  0
#define BUTTON_1     1
#define BUTTON_2     2
#define BUTTON_3     3

#define PARAM_DT_U8  1
#define PARAM_DT_I8  2
#define PARAM_DT_U16 3
#define PARAM_DT_I16 4
#define PARAM_DT_U32 5
#define PARAM_DT_I32 6
#define PARAM_DT_FL  7
#define PARAM_DT_ST  8
#define PARAM_DT_BO  9


class WebSettings {
public:
  WebSettings();
  void initWebSettings(const char *parameter, String confName, String configfile, uint8_t settingNr);

  //Parameterfile
  boolean deleteConfig();
  boolean writeConfig();
  
  static String   getString(uint32_t name, boolean fromFlash, uint8_t u8_dataType);
  static String   getString(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);
  static int      getInt(uint32_t name);
  static int      getInt(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);
  static float    getFloat(uint32_t name);
  static float    getFloat(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);
  static boolean  getBool(uint32_t name);
  static boolean  getBool(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);

  static int getIntFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr, uint8_t dataType);
  static float getFloatFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);
  static boolean getBoolFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);
  static String getStringFlash(uint32_t name, uint8_t settingNr, uint8_t groupNr, uint8_t listNr);
  String getStringFlash(String name);


  //setButtons
  void setButtons(uint8_t buttons, String btnLabel);

  //register onSave callback
  void registerOnSave(void (*callback)());
  void registerOnButton1(void (*callback)());
  void registerOnButton2(void (*callback)());
  void registerOnButton3(void (*callback)());
  
  void setTimerHandlerName(String handlerName, uint16_t timerSec=1000);
  void handleHtmlFormRequest(WebServer * server);

  static uint32_t getParmId(uint16_t id, uint8_t settingNr, uint8_t groupIdx, uint8_t listIdx);
  static void     getIdFromParamId(uint32_t paramId, uint16_t &id, uint8_t &settingNr, uint8_t &groupIdx, uint8_t &listIdx);

  
private:
  const char *parameterFile;

  String   str_mConfName;
  String   str_mConfigfile;
  uint8_t  u8_mJsonArraySize;
  uint8_t  u8_mSettingNr;
  String   str_mAjaxGetDataTimerHandlerName;
  uint16_t u16_mAjaxGetDataTimerSec;

  uint8_t  u8_mButtons = 0;
  String   str_mButton1Text;
  String   str_mButton2Text;
  String   str_mButton3Text;

  //Parameterfile
  boolean readConfig();

  void getDefaultValuesFromNewKeys(const char *parameter, uint32_t jsonStartPos);
  void buildSendHtml(WebServer * server, const char *parameter, uint32_t jsonStartPos);
  void sendContentHtml(WebServer *server, const char *buf, bool send);
  //void readWebValues(WebServer * server, const String *parameter, uint32_t jsonStartPos);

  bool isKeyExist(uint32_t key);
  void setString(uint32_t name, String value);

  String   getJson_Key(const char *parameter, String key, uint8_t idx, uint32_t startPos, String defaultValue); //Universal über parameter key
  uint8_t  getJsonSize(const char *parameter, uint8_t idx, uint32_t startPos);
  uint8_t  getJsonGroupsize(const char *parameter, uint8_t idx, uint32_t startPos);
  uint32_t getJsonName(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonLabel(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonLabelEntry(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonHelp(const char *parameter, uint8_t idx, uint32_t startPos);
  uint8_t  getJsonType(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonDefault(const char *parameter, uint8_t idx, uint32_t startPos);
  uint8_t  getJsonOptionsCnt(const char *parameter, uint8_t idx, uint32_t startPos);
  uint32_t getJsonOptionsMin(const char *parameter, uint8_t idx, uint32_t startPos);
  uint32_t getJsonOptionsMax(const char *parameter, uint8_t idx, uint32_t startPos);
  std::vector<String> getJsonOptionValues(const char *parameter, uint8_t idx, uint32_t startPos);
  std::vector<String> getJsonOptionLabels(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonArrValue(const char *parameter, String str_key1, String str_key2, uint8_t u8_eCnt, uint8_t idx, uint32_t startPos);

  void createHtmlTextfield(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, const char * type, String value);
  void createHtmlTextarea(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlNumber(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlFloat(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlRange(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlCheckbox(char * buf, uint32_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlStartSelect(char * buf, uint64_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos);
  void createHtmlAddSelectOption(char * buf, String option, String label, String value);
  void createHtmlStartMulti(char * buf, String *label, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t u8_jsonType);
  void createHtmlAddMultiOption(char * buf, uint32_t *name, uint64_t *nameExt, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t option, String label, uint32_t value);

  void (*fn_mOnButtonSave)() = NULL;
  void (*fn_mOnButton1)() = NULL;
  void (*fn_mOnButton2)() = NULL;
  void (*fn_mOnButton3)() = NULL;
};

#endif
