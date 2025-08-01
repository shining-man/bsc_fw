// Copyright (c) 2022 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DEFINES_H
#define DEFINES_H

#include "params_dt.h"
#include "bscTime.h"

#define BSC_SW_VERSION      "V0.7.6"

static const char COMPILE_DATE_TIME[] = "";


// TaskHandles
extern TaskHandle_t task_handle_bscSerial;

#define TASK_PRIORITY_STD 5
#define TASK_PRIORITY_ALARMRULES (configMAX_PRIORITIES - 6)
#define TASK_PRIORITY_CONNECT_WIFI 1
#define TASK_PRIORITY_SERIAL_MAX (TASK_PRIORITY_ALARMRULES + 1)


//#define USE_LittleFS
#define HTML_MINIFY

//Debug
//#define DEBUG_ON_FS        //wird in der platformio.ini gesetzt
//#define DEBUG_ON_HW_SERIAL //wird in der platformio.ini gesetzt
//#define LOG_TO_SERIAL
#define DEBUG_SW_BAUDRATE         19200

//#define DEBUG_JTAG
//#define BPN
//#define INSIDER_V1

#ifdef DEBUG_ON_FS
//Erweitertes Logging (zum debuggen)
//#define JK_DEBUG
//#define JK_BT_DEBUG
//#define SEPLOS_DEBUG
//#define SYLCIN_DEBUG
//#define NEEY_DEBUG
//#define NEEY_WRITE_DATA_DEBUG
//#define DALY_DEBUG
//#define BT_DEBUG        //Bluetooth
//#define MQTT_DEBUG
//#define GOBEL_DEBUG
//#define GOBELPC200_DEBUG
//#define WLAN_DEBUG
//#define WLAN_DEBUG2
//#define CAN_DEBUG
//#define CAN_DEBUG_STATUS
//#define WEBSET_DEBUG
//#define MAIN_DEBUG
//#define LOG_BMS_DATA
//#define JK_INV_DEBUG
#endif

#ifdef DEBUG_ON_HW_SERIAL
//Erweitertes Logging (zum debuggen)
//#define JK_DEBUG
//#define JK_BT_DEBUG
//#define SEPLOS_DEBUG
//#define SYLCIN_DEBUG
//#define NEEY_DEBUG
//#define NEEY_WRITE_DATA_DEBUG
//#define DALY_DEBUG
//#define BT_DEBUG        //Bluetooth
//#define MQTT_DEBUG
//#define GOBEL_DEBUG
#define GOBELPC200_DEBUG
#define WLAN_DEBUG
//#define WLAN_DEBUG2
//#define CAN_DEBUG
//#define CAN_DEBUG_STATUS
//#define WEBSET_DEBUG
#define MAIN_DEBUG
//#define LOG_BMS_DATA
//#define JK_INV_DEBUG
#endif

//Tests
//#define UTEST_BMS_FILTER
//#define UTEST_FS
//#define UTEST_RESTAPI


//WebSettings Datatypes
#define PARAM_DT_U8  1
#define PARAM_DT_I8  2
#define PARAM_DT_U16 3
#define PARAM_DT_I16 4
#define PARAM_DT_U32 5
#define PARAM_DT_I32 6
#define PARAM_DT_FL  7
#define PARAM_DT_ST  8
#define PARAM_DT_BO  9

//System
#define WEBSERVER_PORT               80

//Alarmrules
#define CNT_ALARMS                   10
#define CNT_BT_ALARMS_RULES          20
#define ANZAHL_RULES_TRIGGER_SOC      4

//DI/DO
#if !defined(LILYGO_TCAN485)
#define CNT_DIGITALOUT                6
#define CNT_DIGITALIN                 4
#endif

#if !defined(LILYGO_TCAN485) && !defined(TCONNECT)
#define H_CLK                        14
#define H_MOSI                       13
#define H_MISO                       12
#define IO_DO_PL                     26
#define GPIO_LED1_HW1                 0
#endif

#if defined(TCONNECT)
const uint8_t GPIO_IN[] = {47, 21, 45, 48};        // 0..3
const uint8_t GPIO_OUT[] = {40, 39, 42, 41, 1, 2}; // 0..5
#endif

//Tacho
#if !defined(LILYGO_TCAN485) && !defined(TCONNECT)
#define TACHO_ADDR0                   6
#define TACHO_ADDR1                   7
#define TACHO_ADDR2                  15
#define TACHO_GPIO                   27
#define TACHO_MEAS_TIME            3000
#endif

//Onewire
#define MAX_ANZAHL_OW_SENSOREN         64
#define TEMP_IF_SENSOR_READ_ERROR  0xFF00
#define OW_TEMP_AVG_CYCELS              3

//Bluetooth
#define BT_DEVICES_COUNT              7
#define BT_EXT_DEVICES_COUNT          5

//Serial
#define SERIAL_BMS_DEVICES_COUNT      11
enum serialRxTxEn_e {serialRxTx_RxTxDisable, serialRxTx_TxEn, serialRxTx_RxEn};

#ifdef TCONNECT
#define SERIAL1_PIN_RX                5
#define SERIAL1_PIN_TX                4
#define SERIAL2_PIN_RX                7
#define SERIAL2_PIN_TX                6
#define SERIAL3_PIN_RX               18
#define SERIAL3_PIN_TX               17
#else
#define SERIAL1_PIN_RX               16
#define SERIAL1_PIN_TX               17
#define SERIAL1_PIN_TX_EN            18
#define SERIAL2_PIN_RX               23
#define SERIAL2_PIN_TX               25
#define SERIAL2_PIN_TX_EN             2
#define SERIAL3_PIN_RX               35
#define SERIAL3_PIN_TX               33
#define SERIAL3_PIN_TX_EN             3
#define SERIAL3_PIN_RX_EN            32
#endif

//CAN
#if defined(LILYGO_TCAN485)
  #define CAN_TX_PIN GPIO_NUM_27
  #define CAN_RX_PIN GPIO_NUM_26
#elif defined(TCONNECT)
  #define CAN_TX_PIN GPIO_NUM_9
  #define CAN_RX_PIN GPIO_NUM_10
#else
  #define CAN_TX_PIN GPIO_NUM_4
  #define CAN_RX_PIN GPIO_NUM_5
#endif

//Onewire (Temperatur)
#if defined(LILYGO_TCAN485)
  #define OW_PIN                     25
#elif defined(TCONNECT)
  #define OW_PIN                     21
#else
  #define OW_PIN                     19
#endif
#define COUNT_TEMP_RULES             10

//I2C
#define ID_I2C_MASTER                  0 //Auswahl Master/Slave
#define I2C_DEV_ADDR_DISPLAY        0x55
#define I2C_DEV_ADDR_SLAVE1           16
#define I2C_DEV_ADDR_SLAVE2           17
#define I2C_DEV_ADDR_SERIAL_EXTENSION 32
#define I2C_DEV_ADDR_BT_EXTENSION_1 0x60
#define I2C_FREQUENCY           1000000U
#define I2C_CNT_SLAVES                2

#if defined(LILYGO_TCAN485)
  #define I2C_SDA_PIN                   32
  #define I2C_SCL_PIN                   33
#elif defined(TCONNECT)
  #define I2C_SDA_PIN                   11
  #define I2C_SCL_PIN                   12
#else
  #define I2C_SDA_PIN                   21
  #define I2C_SCL_PIN                   22
#endif

//LILYGO_TCAN485
#define TCAN485_PIN_5V_EN    16
#define TCAN485_CAN_SE_PIN   23
#define TCAN485_RS485_EN_PIN 17
#define TCAN485_RS485_TX_PIN 22
#define TCAN485_RS485_RX_PIN 21
#define TCAN485_RS485_SE_PIN 19


//BMS Data (mapping)
#define MUBER_OF_DATA_DEVICES       18
#define NR_OF_BMS_TEMP_SENSORS      6
#define SERIAL_BMS_EXT_COUNT        8


//Alarmrules
#define CYCLES_BMS_VALUES_PLAUSIBILITY_CHECK  15


// Inverter
#define CAN_BMS_COMMUNICATION_TIMEOUT 5000


//Send serial Data
enum serialDataRwTyp_e {BPN_NO_DATA, BPN_READ_SETTINGS, BPN_WRITE_READ_SETTINGS, BPN_START_FWUPDATE};
//#define SERIAL_DATA_RW_LEN__BPN_READ_SETTINGS 20


// Register
#define RTC_CNTL_SDIO_CONF_REG 0x3FF48074

// Register RTC_CNTL_SDIO_CONF_REG
#define RTC_CNTL_XPD_SDIO_VREG 0x80000000
#define RTC_CNTL_SDIO_TIEH     0x800000


/*********************************************
 * Parameter IDs
 *********************************************/
#define ID_PARAM_SERIAL_CONNECT_DEVICE               1
#define ID_PARAM_SERIAL_ALARM_AKTION                 6
// ID_PARAM_SERIAL_ALARM_AKTION: 0=AUS; 1-5=Alarm 1-5
#define ID_PARAM_SERIAL_ALARM_TIME_OUT               7

#define ID_PARAM_SS_CAN                              2
//#define ID_PARAM_SS_BT                             3
#define ID_PARAM_SS_BTDEV                            4
#define ID_PARAM_SS_BTDEVMAC                         5

#define ID_PARAM_ALARM_BTDEV_BMS_SELECT              9
//#define ID_PARAM_ALARM_BT                         10
//#define ID_PARAM_ALARM_BTDEV_ALARM_ON             11
#define ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT         12
//#define ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON       13
#define ID_PARAM_ALARM_BT_CNT_CELL_CTRL             14
#define ID_PARAM_ALARM_BT_CELL_SPG_MIN              15
#define ID_PARAM_ALARM_BT_CELL_SPG_MAX              16
#define ID_PARAM_ALARM_BTDEV_ALARM_AKTION           17
#define ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION     18
#define ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION   19
// ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION: 0=AUS; 1-5=Alarm 1-5

#define ID_PARAM_TEMP_ALARM                         20
#define ID_PARAM_TEMP_ALARM_SENSOR_VON              21
#define ID_PARAM_TEMP_ALARM_SENSOR_BIS              22
#define ID_PARAM_TEMP_ALARM_SENSOR_VERGLEICH        23
#define ID_PARAM_TEMP_ALARM_WERT1                   24
#define ID_PARAM_TEMP_ALARM_WERT2                   25
#define ID_PARAM_TEMP_ALARM_AKTION                  26
#define ID_PARAM_TEMP_ALARM_UEBERWACH_FUNKTION      27

#define ID_PARAM_DO_AUSLOESEVERHALTEN               30
#define ID_PARAM_DO_IMPULSDAUER                     31
#define ID_PARAM_DO_AUSLOESUNG_BEI                  32
#define ID_PARAM_DO_VERZOEGERUNG                    33

#define ID_PARAM_DI_INVERTIERT                      34
#define ID_PARAM_DI_ALARM_NR                        35

#define ID_PARAM_WLAN_IP_ADRESSE                    36
#define ID_PARAM_WLAN_GATEWAY                       37
#define ID_PARAM_WLAN_SUBNET                        38
#define ID_PARAM_WLAN_DNS                           39

#define ID_PARAM_WLAN_SSID                          40
#define ID_PARAM_WLAN_PWD                           41
#define ID_PARAM_MQTT_SERVER_IP                     42
#define ID_PARAM_MQTT_SERVER_PORT                   43
#define ID_PARAM_MQTT_SERVER_ENABLE                 44
#define ID_PARAM_MQTT_DEVICE_NAME                   45
#define ID_PARAM_MQTT_TOPIC_NAME                    46

#define ID_PARAM_ONWIRE_ENABLE                      50
#define ID_PARAM_ONEWIRE_ADR                        51
#define ID_PARAM_ONWIRE_TEMP_OFFSET                 52

#define ID_PARAM_BMS_CAN_ENABLE                                         60
#define ID_PARAM_BMS_CAN_DATASOURCE                                     61
#define ID_PARAM_BMS_MAX_CHARGE_SPG                                     62
#define ID_PARAM_BMS_MAX_CHARGE_CURRENT                                 64
#define ID_PARAM_BMS_MAX_DISCHARGE_CURRENT                              65
#define ID_PARAM_BMS_LADELEISTUNG_AUF_NULL                              66
#define ID_PARAM_BMS_ENTLADELEISTUNG_AUF_NULL                           67
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLDRIFT_EN             68
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTABWEICHUNG          69
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_MV                 70
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTSPG_ZELLE           71

#define ID_PARAM_ALARM_BT_GESAMT_SPG_MIN                                72
#define ID_PARAM_ALARM_BT_GESAMT_SPG_MAX                                73

#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_EN               74
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_STARTSPG         75
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_ENDSPG           76
#define ID_PARAM_BMS_SOC_AUF_FULL                                       77
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM    78
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_SOC_EN                   79
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_AB_SOC                   80
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_PERCENT_SOC        81
#define ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_TIME                   82
#define ID_PARAM_BMS_CAN_DATASOURCE_SS1                                 83
#define ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_CURRENT                84
#define ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_SOC                    85

#define ID_PARAM_MQTT_USERNAME                                          86
#define ID_PARAM_MQTT_PWD                                               87

#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_EN                     88
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG                    89
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SOC                    90
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_TIME                   91
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END                92

#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_EN              93
#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_ZELLSPG         94
#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_DELTA           95

#define ID_PARAM_WLAN_CONNECT_TIMEOUT             96

#define ID_PARAM_INVERTER_BATT_TEMP_QUELLE        97
#define ID_PARAM_INVERTER_BATT_TEMP_SENSOR        98

#define ID_PARAM_NEEY_CELLS                       99
#define ID_PARAM_NEEY_START_VOLTAGE              100
#define ID_PARAM_NEEY_MAX_BALANCE_CURRENT        101
#define ID_PARAM_NEEY_SLEEP_VOLTAGE              102
#define ID_PARAM_NEEY_EQUALIZATION_VOLTAGE       103
#define ID_PARAM_NEEY_BAT_CAPACITY               104
#define ID_PARAM_NEEY_BAT_TYPE                   105
#define ID_PARAM_NEEY_BUZZER                     106 //not use
#define ID_PARAM_NEEY_BALANCER_ON                107

//#define ID_PARAM_SERIAL2_CONNECT_TO_ID           108 // not use

#define ID_PARAM_TEMP_SENSOR_TIMEOUT_TRIGGER     109
#define ID_PARAM_TEMP_SENSOR_TIMEOUT_TIME        110

#define ID_PARAM_MASTER_SLAVE_TYP                111

#define ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE      112
#define ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE       113
#define ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE      114
#define ID_PARAM_BMS_ALARM_LOWTEMPERATURE        115

#define ID_PARAM_INVERTER_MULTI_BMS_VALUE_SOC    116

#define ID_PARAM_TRIGGER_NAMES                   117

#define ID_PARAM_BATTERY_PACK_CHARGE_CURRENT     118
#define ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT  119

#define ID_PARAM_BMS_FILTER_CELL_VOLTAGE_PERCENT 120
#define ID_PARAM_BMS_FILTER_RX_ERROR_COUNT       121

#define ID_PARAM_SYSTEM_NTP_SERVER_NAME          122
//#define ID_PARAM_SYSTEM_NTP_SERVER_PORT          123

#define ID_PARAM_JBD_CELL_VOLTAGE_100            124

#define ID_PARAM_BMS_CAN_EXTENDED_DATA_ENABLE    125

#define ID_PARAM_BTDEV_DEACTIVATE                126

#define ID_PARAM_BMS_BALUE_ADJUSTMENTS_SOC100_CELL_VOLTAGE 127

#define ID_PARAM_TEMP_ALARM_TEMP_QUELLE          128
#define ID_PARAM_TEMP_ALARM_BMS_QUELLE           129

#define ID_PARAM_BMS_BALUE_ADJUSTMENTS_SOC0_CELL_VOLTAGE 130

#define ID_PARAM_ALARM_BT_GESAMT_SPG_HYSTERESE   131

#define ID_PARAM_BMS_PLAUSIBILITY_CHECK_CELLVOLTAGE 132

#define ID_PARAM_MQTT_SEND_DELAY 133

#define ID_PARAM_TRIGGER_AT_SOC     134
#define ID_PARAM_TRIGGER_AT_SOC_ON  135
#define ID_PARAM_TRIGGER_AT_SOC_OFF 136

#define ID_PARAM_I_AM_A_SUPPORTER   137

#define ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_STARTSPG         138
#define ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_ENDSPG           139
#define ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM    140

#define ID_PARAM_SERIAL_NUMBER_OF_CELLS 141

#define ID_PARAM_INVERTER_BMS_QUELLE_SOC 142

#define ID_PARAM_SYSTEM_RECORD_VALUES_PERIODE 143

#define ID_PARAM_DISPLAY_TIMEOUT 144

#define ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_START_VOLTAGE 145

#define ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_CURRENT    146
#define ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_OFFSET_MAX 147
#define ID_PARAM_DYNAMIC_CHARGE_VOLTAGE_OFFSET_MIN 148

#define ID_PARAM_INVERTER_AUTOBALANCE_START_INTERVAL     149
#define ID_PARAM_INVERTER_AUTOBALANCE_START_CELLVOLTAGE  150
#define ID_PARAM_INVERTER_AUTOBALANCE_CHARGE_VOLTAGE     151
#define ID_PARAM_INVERTER_AUTOBALANCE_CELLDIF_FINISH     152
#define ID_PARAM_INVERTER_AUTOBALANCE_TIMEOUT            153
#define ID_PARAM_INVERTER_AUTOBALANCE_CHARGE_CELLVOLTAGE 154

#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_SOC_MINDEST_STROM 155
#define ID_PARAM_BMS_FLOAT_CHARGE_SPG                            156

#define ID_PARAM_INVERTER_AUTOBALANCE_ENABLE                     157
#define ID_PARAM_INVERTER_CHARGE_CURRENT_CUT_OFF_ENABLE          158
#define ID_PARAM_INVERTER_ENTLADESTROM_REDUZIEREN_ZELLSPG_EN     159

#define ID_PARAM_BSC_USERNAME 160
#define ID_PARAM_BSC_PASSWORD 161

#define ID_PARAM_INVERTER_AUTOBALANCE_MINDEST_TIME 162

#define ID_PARAM_BATTERY_PACK_CAPACITY 163

#define ID_PARAM_REL_INVERTIERT 164

#define ID_PARAM_DEVICE_MAPPING_SCHNITTSTELLE      163
#define ID_PARAM_DEVICE_MAPPING_ADRESSE            164
#define ID_PARAM_DEVICE_MAPPING_NAME               165

#define ID_PARAM_ALARM_BT_CELL_SPG_MAX_HYSTERESE   166

#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_BATTERYPACK 167

#define ID_PARAM_MQTT_VTRIGGER_REMANENT 168




//Auswahl Bluetooth Geräte
#define ID_BT_DEVICE_NB              0
#define ID_BT_DEVICE_NEEY_GW_24S4EB  1
#define ID_BT_DEVICE_JKBMS_JK02      2
#define ID_BT_DEVICE_JKBMS_JK02_32S  3
#define ID_BT_DEVICE_NEEY_EK_24S4EB  4

//Auswahl Serial Geräte
#define ID_SERIAL_DEVICE_NB                   0
#define ID_SERIAL_DEVICE_JBDBMS               1
#define ID_SERIAL_DEVICE_JKBMS                2
#define ID_SERIAL_DEVICE_SEPLOSBMS            3
#define ID_SERIAL_DEVICE_DALYBMS              4
#define ID_SERIAL_DEVICE_SYLCINBMS            5
#define ID_SERIAL_DEVICE_JKBMS_V13            6
#define ID_SERIAL_DEVICE_GOBELBMS             7
#define ID_SERIAL_DEVICE_JKBMS_CAN            8
#define ID_SERIAL_DEVICE_BPN                  9
#define ID_SERIAL_DEVICE_SMARTSHUNT_VEDIRECT 10
#define ID_SERIAL_DEVICE_GOBEL_PC200         11
#define ID_SERIAL_DEVICE_SEPLOSBMS_V3        12
#define ID_SERIAL_DEVICE_NEEY_4A             13
#define ID_SERIAL_DEVICE_JKINVERTERBMS       14
#define ID_SERIAL_DEVICE_PYLONTECH           15

//Auswahl CAN Geräte
#define ID_CAN_DEVICE_NB            0
#define ID_CAN_DEVICE_SOLISRHI      1
#define ID_CAN_DEVICE_PYLONTECH     2
#define ID_CAN_DEVICE_VICTRON       3
#define ID_CAN_DEVICE_VICTRON_250K  4
#define ID_CAN_DEVICE_BYD_PROTOCOL  5

//Auswahl Temp.Alarm Funktionen
#define ID_TEMP_ALARM_FUNKTION_NB               0
#define ID_TEMP_ALARM_FUNKTION_MAXWERT          1
#define ID_TEMP_ALARM_FUNKTION_MAXWERT_REFERENZ 2
#define ID_TEMP_ALARM_FUNKTION_DIFFERENZ        3
#define ID_TEMP_ALARM_FUNKTION_MINWERT          4

//Auswahl
#define OPTION_MULTI_BMS_SOC_AVG                1
#define OPTION_MULTI_BMS_SOC_MAX                2
#define OPTION_MULTI_BMS_SOC_BMS                3
#define OPTION_MULTI_BMS_SOC_MIN                4





/*********************************************
 * I2C
 *********************************************/
#define TXBUFF_OFFSET                     0x04

#define BMS_DATA                          0x01  //BMS-Daten
#define INVERTER_DATA                     0x02  //Inverter-Daten
#define BSC_DATA                          0x03  //
#define BSC_GET_BT_EXTENSION_DATA         0x0B  // Hole Daten von BT Extension
#define BSC_GET_BT_EXTENSION_BMSDATA      0x0C  // Hole Daten von BT Extension
#define BSC_SET_BT_EXTENSION_DATA         0x0D  // Sende Daten zur BT Extension

//BMS_DATA 0x01
#define BMS_CELL_VOLTAGE                  0x01
#define BMS_TOTAL_VOLTAGE                 0x02
#define BMS_MAX_CELL_DIFFERENCE_VOLTAGE   0x03
#define BMS_AVG_VOLTAGE                   0x04
#define BMS_TOTAL_CURRENT                 0x05
#define BMS_MAX_CELL_VOLTAGE              0x06
#define BMS_MIN_CELL_VOLTAGE              0x07
#define BMS_MAX_VOLTAGE_CELL_NUMBER       0x08
#define BMS_MIN_VOLTAGE_CELL_NUMBER       0x09
#define BMS_IS_BALANCING_ACTIVE           0x0A
#define BMS_BALANCING_CURRENT             0x0B
#define BMS_TEMPERATURE                   0x0C
#define BMS_CHARGE_PERCENT                0x0D
#define BMS_ERRORS                        0x0E
#define BMS_LAST_DATA_MILLIS              0x0F

//INVERTER_DATA 0x02
#define INVERTER_VOLTAGE                  0x01
#define INVERTER_CURRENT                  0x02
#define INVERTER_SOC                      0x03
#define INVERTER_CHARGE_CURRENT           0x04
#define INVERTER_DISCHARG_CURRENT         0x05

//BSC_DATA 0x03
#define BSC_ALARMS                        0x01
#define BSC_IP_ADDR                       0x02
#define BSC_RELAIS                        0x03
#define BSC_DISPLAY_TIMEOUT               0x04

// BSC_GET_BT_EXTENSION_DATA
#define BSC_BT_EXT_GET_FOUND_MAC          0x01
#define BSC_BT_EXT_GET_SETTINGS           0x02

// BSC_SET_BT_EXTENSION_DATA 
#define BSC_BT_CONNECT_MACS               0x01 // Zu verbindende MACs
#define BSC_BT_NEEY_SETTINGS              0x02
#define BSC_BT_NEEY_BALLANCER_STATE       0x03
#define BSC_BT_START_SCAN                 0x04


/*********************************************
 * MQTT
 *********************************************/
#define MQTT_TOPIC_DATA_DEVICE                   1
#define MQTT_TOPIC_TEMPERATUR                    2
#define MQTT_TOPIC_ALARM                         3
#define MQTT_TOPIC_INVERTER                      4
#define MQTT_TOPIC_SYS                           5
//#define MQTT_TOPIC_BMS_SERIAL                    6

#define MQTT_TOPIC2_CELL_VOLTAGE                11
#define MQTT_TOPIC2_CELL_VOLTAGE_MAX            12
#define MQTT_TOPIC2_CELL_VOLTAGE_MIN            13
#define MQTT_TOPIC2_TOTAL_VOLTAGE               14
#define MQTT_TOPIC2_MAXCELL_DIFFERENCE_VOLTAGE  15
#define MQTT_TOPIC2_BALANCING_ACTIVE            16
#define MQTT_TOPIC2_BALANCING_CURRENT           17
#define MQTT_TOPIC2_TEMPERATURE                 18
#define MQTT_TOPIC2_CHARGE_PERCENT              19
#define MQTT_TOPIC2_ERRORS                      20
#define MQTT_TOPIC2_BALANCE_CAPACITY            21
#define MQTT_TOPIC2_CYCLE                       22
#define MQTT_TOPIC2_TOTAL_CURRENT               23
#define MQTT_TOPIC2_FULL_CAPACITY               24
#define MQTT_TOPIC2_BALANCE_STATUS              25
#define MQTT_TOPIC2_DISCHARGE_CURRENT_SOLL      26
#define MQTT_TOPIC2_CHARGED_ENERGY              27
#define MQTT_TOPIC2_DISCHARGED_ENERGY           28
#define MQTT_TOPIC2_CHARGE_CURRENT_SOLL         29
#define MQTT_TOPIC2_ESP32_TEMP                  30
#define MQTT_TOPIC2_FREE_HEAP                   31
#define MQTT_TOPIC2_MIN_FREE_HEAP               32
#define MQTT_TOPIC2_HIGHWATER_TASK_BLE          33
#define MQTT_TOPIC2_HIGHWATER_TASK_ALARMRULES   34
#define MQTT_TOPIC2_HIGHWATER_TASK_OW           35
#define MQTT_TOPIC2_HIGHWATER_TASK_CAN          36
#define MQTT_TOPIC2_HIGHWATER_TASK_SERIAL       37
#define MQTT_TOPIC2_HIGHWATER_TASK_WIFICONN     38
#define MQTT_TOPIC2_FET_STATE_CHARGE            39
#define MQTT_TOPIC2_FET_STATE_DISCHARGE         40
#define MQTT_TOPIC2_INVERTER_CHARGE_VOLTAGE     41
#define MQTT_TOPIC2_BMS_DATA_VALID              42
#define MQTT_TOPIC2_CELL_RESISTANCE             43
#define MQTT_TOPIC2_CYCLE_CAPACITY              44
#define MQTT_TOPIC2_POWER                       45
#define MQTT_TOPIC2_TIME_TO_GO                  46
#define MQTT_TOPIC2_TOTAL_VOLT_MIN              47
#define MQTT_TOPIC2_TOTAL_VOLT_MAX              48
#define MQTT_TOPIC2_TIME_SINCE_FULL             49
#define MQTT_TOPIC2_SOC_SYNC_COUNT              50
#define MQTT_TOPIC2_TOTAL_VOLT_MIN_COUNT        51
#define MQTT_TOPIC2_TOTAL_VOLT_MAX_COUNT        52
#define MQTT_TOPIC2_AMOUNT_DCH_ENERGY           53
#define MQTT_TOPIC2_AMOUNT_CH_ENERGY            54
#define MQTT_TOPIC2_CUTOFF_VALUE                55
#define MQTT_TOPIC2_CUTOFF_TIMER                56
#define MQTT_TOPIC2_AUTOBAL_STATE               57
#define MQTT_TOPIC2_WARNINGS                    58
#define MQTT_TOPIC2_CELL_VOLTAGE_MAX_NR         59
#define MQTT_TOPIC2_CELL_VOLTAGE_MIN_NR         60
#define MQTT_TOPIC2_CHARGE_VOLTAGE_STATE        61


static const char* mqttTopics[] = {"", // 0
  "DataDevice",    // 1
  "temperatur",    // 2
  "trigger",       // 3
  "inverter",      // 4
  "sys",           // 5
  "",              // 6
  "", // 7
  "", // 8
  "", // 9
  "", // 10
  "cellVoltage",               // 11
  "cellVoltageMax",            // 12
  "cellVoltageMin",            // 13
  "totalVoltage",              // 14
  "maxCellDifferenceVoltage",  // 15
  "balancingActive",           // 16
  "balancingCurrent",          // 17
  "temperature",               // 18
  "SoC",                       // 19
  "errors",                    // 20
  "BalanceCapacity",           // 21
  "Cycle",                     // 22
  "totalCurrent",              // 23
  "FullCapacity",              // 24
  "BalanceStatus",             // 25
  "dischargeCurrentSoll",      // 26
  "ChargedEnergy",             // 27
  "DischargedEnergy",          // 28
  "chargeCurrentSoll",         // 29
  "esp32Temp",                 // 30
  "free_heap",                 // 31
  "min_free_heap",             // 32
  "highWater_task_ble",        // 33
  "highWater_task_alarmrules", // 34
  "highWater_task_ow",         // 35
  "highWater_task_can",        // 36
  "highWater_task_serial",     // 37
  "highWater_task_wifi",       // 38
  "stateCharge",               // 39
  "stateDischarge",            // 40
  "chargeVoltage",             // 41
  "valid",                     // 42
  "cellResistance",            // 43
  "CycleCapacity",             // 44
  "power",                     // 45
  "timeToGo",                  // 46
  "totalVoltMin",              // 47
  "totalVoltMax",              // 48
  "timeSinceFull",             // 49
  "SocSyncCount",              // 50
  "totalVoltMinCount",         // 51
  "totalVoltMaxCount",         // 52
  "amountDchEnergy",           // 53
  "amountChEnergy",            // 54
  "cutoffValue",               // 55  Debug
  "cutoffTimer",               // 56  Debug
  "autoBalState",              // 57  Debug
  "warnings",                  // 58
  "maxCell",                   // 59
  "minCell",                   // 60
  "cv_state",                  // 61
  "",                          // 62
  "",                          // 63
  };



/*
 * BPN
 */
// Allgemein
#define BPN_SETTINGS_NR_OF_CELLS 1

// Alarm
#define BPN_SETTINGS_LOW_CELL_VOLTAGE 2
#define BPN_SETTINGS_HIGH_CELL_VOLTAGE 3
#define BPN_SETTINGS_ALARM_DELAY_CELL_VOLTAGE 4
#define BPN_SETTINGS_ALARM_RELAIS_CELL_VOLTAGE 5

#define BPN_SETTINGS_LOW_BATTERY_VOLTAGE 6
#define BPN_SETTINGS_HIGH_BATTERY_VOLTAGE 7
#define BPN_SETTINGS_ALARM_DELAY_BATTERY_VOLTAGE 8
#define BPN_SETTINGS_ALARM_RELAIS_BATTERY_VOLTAGE 9

#define BPN_SETTINGS_MAX_CHARGE_CURRENT 10
#define BPN_SETTINGS_ALARM_DELAY_MAX_CHARGE_CURRENT 11
#define BPN_SETTINGS_ALARM_RELAIS_CHARGE_CURRENT 12

#define BPN_SETTINGS_MAX_DISCHARGE_CURRENT 13
#define BPN_SETTINGS_ALARM_DELAY_MAX_DISCHARGE_CURRENT 14
#define BPN_SETTINGS_ALARM_RELAIS_DISCHARGE_CURRENT 15

// Shunt
#define BPN_SETTINGS_NOMINAL_BAT_CAPACITY 16

// Ausgänge
#define BPN_SETTINGS_ALARM_RELAIS1_MODE 17
#define BPN_SETTINGS_ALARM_RELAIS2_MODE 18






/*
#define BSC_LOGE ESP_LOGE
#define BSC_LOGW ESP_LOGW
#define BSC_LOGI ESP_LOGI
#define BSC_LOGD ESP_LOGD
#define BSC_LOGV ESP_LOGV
*/

#define DISABLE_BSCx_LOG

extern uint8_t bscLogLevel;

#define BSC_LOGE( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC(ESP_LOG_ERROR,   tag, format, ##__VA_ARGS__)
#define BSC_LOGW( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC(ESP_LOG_WARN,    tag, format, ##__VA_ARGS__)
#define BSC_LOGI( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC(ESP_LOG_INFO,    tag, format, ##__VA_ARGS__)
#define BSC_LOGD( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC(ESP_LOG_DEBUG,   tag, format, ##__VA_ARGS__)
#define BSC_LOGV( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC(ESP_LOG_VERBOSE, tag, format, ##__VA_ARGS__)


#ifdef DISABLE_BSCx_LOG
#define BSC_LOGI1(tag, format, ...) do {} while (0)
#define BSC_LOGI2(tag, format, ...) do {} while (0)

#define BSC_LOGE1(tag, format, ...) do {} while (0)
#define BSC_LOGE2(tag, format, ...) do {} while (0)
#else
#define BSC_LOGI1( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC_2(ESP_LOG_INFO, 1, tag, format, ##__VA_ARGS__)
#define BSC_LOGI2( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC_2(ESP_LOG_INFO, 2, tag, format, ##__VA_ARGS__)

#define BSC_LOGE1( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC_2(ESP_LOG_ERROR, 1, tag, format, ##__VA_ARGS__)
#define BSC_LOGE2( tag, format, ... ) ESP_LOG_LEVEL_LOCAL_BSC_2(ESP_LOG_ERROR, 2, tag, format, ##__VA_ARGS__)
#endif

#define ESP_LOG_LEVEL_LOCAL_BSC(level, tag, format, ...) do {               \
        if ( LOG_LOCAL_LEVEL_BSC >= level) ESP_LOG_LEVEL_BSC(level, tag, format, ##__VA_ARGS__); \
    } while(0)

#define ESP_LOG_LEVEL_LOCAL_BSC_2(level, level2, tag, format, ...) do {               \
        if ( LOG_LOCAL_LEVEL_BSC >= level && level2 <= bscLogLevel) ESP_LOG_LEVEL_BSC(level, tag, format, ##__VA_ARGS__); \
    } while(0)

#define ESP_LOG_LEVEL_BSC(level, tag, format, ...) do {                     \
        if (level==ESP_LOG_ERROR )          { esp_log_write(ESP_LOG_ERROR,      tag, LOG_SYSTEM_TIME_FORMAT(E, format), getBscDateTimeCc(), tag, ##__VA_ARGS__); } \
        else if (level==ESP_LOG_WARN )      { esp_log_write(ESP_LOG_WARN,       tag, LOG_SYSTEM_TIME_FORMAT(W, format), getBscDateTimeCc(), tag, ##__VA_ARGS__); } \
        else if (level==ESP_LOG_DEBUG )     { esp_log_write(ESP_LOG_DEBUG,      tag, LOG_SYSTEM_TIME_FORMAT(D, format), getBscDateTimeCc(), tag, ##__VA_ARGS__); } \
        else if (level==ESP_LOG_VERBOSE )   { esp_log_write(ESP_LOG_VERBOSE,    tag, LOG_SYSTEM_TIME_FORMAT(V, format), getBscDateTimeCc(), tag, ##__VA_ARGS__); } \
        else                                { esp_log_write(ESP_LOG_INFO,       tag, LOG_SYSTEM_TIME_FORMAT(I, format), getBscDateTimeCc(), tag, ##__VA_ARGS__); } \
    } while(0)



#define isBitSet(byte,bit)   (((byte & (1 << bit)) != 0) ? 1 : 0)
#define ROUND(a, b) ( (a)/(b) + (((a)%(b) >= (b)/2)?1:0))

/**
 * @brief Setzt das Bit #bit in flags auf 1.
 * @param flags Referenz auf die 32‑Bit‑Variable, in der das Bit gesetzt wird.
 * @param bit   Nummer des Bits [0…31], das gesetzt werden soll.
 */
template<typename T>
inline void setBit(T &flags, uint8_t bit) {
    flags |= (T(1) << bit);
}

/**
 * @brief Löscht das Bit #bit in flags (setzt es auf 0).
 * @param flags Referenz auf die 32‑Bit‑Variable, in der das Bit gelöscht wird.
 * @param bit   Nummer des Bits [0…31], das gelöscht werden soll.
 */
template<typename T>
inline void clearBit(T &flags, uint8_t bit) {
    flags &= ~(T(1) << bit);
}

#endif