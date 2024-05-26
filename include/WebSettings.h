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
#define HTML_OPTIONGROUP                  12
#define HTML_SEPARATION                   13
#define HTML_INPUTMULTICHECK_COLLAPSIBLE  14
#define HTML_OPTIONGROUP_COLLAPSIBLE      15
#define HTML_INPUTFLOAT_1                 16
#define HTML_INPUTFLOAT_2                 17
#define HTML_INPUTFLOAT_3                 18
//Max Type = 18

#define BACKGROUND_COLOR "#ffffff"

#define BUTTON_SAVE  0
#define BUTTON_1     1
#define BUTTON_2     2
#define BUTTON_3     3


class WebSettings {
public:
  WebSettings();
  void initWebSettings(const char *parameter, String confName, String configfile);

  //Parameterfile
  boolean deleteConfig();
  boolean writeConfig();

  static String   getString(uint16_t name, boolean fromFlash, uint8_t u8_dataType);
  static String   getString(uint16_t name, uint8_t groupNr);
  static int32_t  getInt(uint16_t name, uint8_t u8_dataType);
  static int32_t  getInt(uint16_t name, uint8_t groupNr, uint8_t u8_dataType);
  static float    getFloat(uint16_t name);
  static float    getFloat(uint16_t name, uint8_t groupNr);
  static bool     getBool(uint16_t name);
  static bool     getBool(uint16_t name, uint8_t groupNr);

  static uint32_t getIntFlash(uint16_t name, uint8_t groupNr, uint8_t dataType);
  static uint32_t getIntFlash(uint16_t name, uint8_t groupNr);
  static float   getFloatFlash(uint16_t name, uint8_t groupNr);
  static float   getFloatFlash(uint16_t name);
  static boolean getBoolFlash(uint16_t name, uint8_t groupNr);
  static boolean getBoolFlash(uint16_t name);
  static String  getStringFlash(uint16_t name, uint8_t groupNr);
  String getStringFlash(String name);
  String getStringFlash(uint16_t name);


  //setButtons
  void setButtons(uint8_t buttons, String btnLabel);

  //register onSave callback
  void registerOnSave(void (*callback)());
  void registerOnButton1(void (*callback)());
  void registerOnButton2(void (*callback)());
  void registerOnButton3(void (*callback)());

  void setTimerHandlerName(String handlerName, uint16_t timerSec=1000);
  void handleHtmlFormRequest(WebServer * server);
  void handleGetValues(WebServer *server);
  void handleSetValues(WebServer *server);

  void setParameter(uint16_t name, uint8_t group, String value, uint8_t u8_dataType);

  static uint16_t getParmId(uint16_t id, uint8_t groupIdx);
  static void     getIdFromParamId(uint16_t paramId, uint16_t &id, uint8_t &groupIdx);

private:
  const char *parameterFile;
  String   str_mConfName;
  String   str_mConfigfile;
  uint8_t  u8_mJsonArraySize;
  String   str_mAjaxGetDataTimerHandlerName;
  uint16_t u16_mAjaxGetDataTimerSec;

  uint8_t  u8_mButtons = 0;
  String   str_mButton1Text;
  String   str_mButton2Text;
  String   str_mButton3Text;

  //Parameterfile
  boolean readConfig();

  uint32_t copyFile(String fileSrc, String fileDst);
  uint32_t calcCrc(String fileSrc);

  void getDefaultValuesFromNewKeys(const char *parameter, uint32_t jsonStartPos);
  void buildSendHtml(WebServer * server, const char *parameter, uint32_t jsonStartPos);
  void sendContentHtml(WebServer *server, const char *buf, bool send);
  //void readWebValues(WebServer * server, const String *parameter, uint32_t jsonStartPos);

  bool isKeyExist(uint16_t key, uint8_t u8_dataType);
  void setString(uint16_t name, String value, uint8_t u8_dataType);
  void setInt(uint16_t name, int32_t value);

  String   getJson_Key(const char *parameter, String key, uint8_t idx, uint32_t startPos, String defaultValue); //Universal über parameter key
  uint8_t  getJsonSize(const char *parameter, uint8_t idx, uint32_t startPos);
  uint8_t  getJsonGroupsize(const char *parameter, uint8_t idx, uint32_t startPos);
  uint32_t getJsonName(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonLabel(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonLabelEntry(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonHelp(const char *parameter, uint8_t idx, uint32_t startPos);
  uint8_t  getJsonType(const char *parameter, uint8_t idx, uint32_t startPos);
  String   getJsonDefault(const char *parameter, uint8_t idx, uint32_t startPos);
  uint8_t  getJsonArrayCnt(const char *parameter, String key, uint8_t idx, uint32_t startPos);
  uint32_t getJsonOptionsMin(const char *parameter, uint8_t idx, uint32_t startPos);
  uint32_t getJsonOptionsMax(const char *parameter, uint8_t idx, uint32_t startPos);
  std::vector<String> getJsonArrayValues(const char *parameter, String key, uint8_t idx, uint32_t startPos);
  std::vector<String> getJsonArrayLabels(const char *parameter, String key, uint8_t idx, uint32_t startPos);
  String   getJsonArrValue(const char *parameter, String str_key1, String str_key2, uint8_t u8_eCnt, uint8_t idx, uint32_t startPos);

  void createHtmlTextfield(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, const char * type, String value);
  void createHtmlTextarea(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlNumber(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlFloat(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlRange(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlCheckbox(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, String value);
  void createHtmlStartSelect(char * buf, uint64_t *name, String *label, const char *parameter, uint8_t idx, uint32_t startPos);
  void createHtmlAddSelectOption(char * buf, String option, String label, String value);
  void createHtmlStartMulti(char * buf, String *label, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t u8_jsonType);
  void createHtmlAddMultiOption(char * buf, uint16_t *name, uint64_t *nameExt, const char *parameter, uint8_t idx, uint32_t startPos, uint8_t option, String label, uint32_t value, uint8_t u8_dataType);
  void createHtmlFloatX(char * buf, uint16_t *name, uint64_t *nameExt, String *label, const char *parameter, uint8_t idx, uint32_t startPos, int32_t value, uint8_t precision);

  void (*fn_mOnButtonSave)() = NULL;
  void (*fn_mOnButton1)() = NULL;
  void (*fn_mOnButton2)() = NULL;
  void (*fn_mOnButton3)() = NULL;
};

#endif
