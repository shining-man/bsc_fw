// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DEFINES_H
#define DEFINES_H


#define BSC_SW_VERSION      "V0.3.10"
static const char COMPILE_DATE_TIME[] = "";

#define HTML_MINIFY

//Debug
#define DEBUG_ON_FS
//#define DEBUG_ON_HW_SERIAL
//#define LOG_TO_SERIAL
#define DEBUG_SW_BAUDRATE         19200

//Erweitertes Logging (zum debuggen)
//#define JK_DEBUG
//#define JK_BT_DEBUG
//#define SEPLOS_DEBUG
//#define NEEY_DEBUG
#define DALY_DEBUG
//#define BT_DEBUG        //Bluetooth
//#define MQTT_DEBUG        
#define WLAN_DEBUG        
//#define WLAN_DEBUG2
//#define CAN_DEBUG
//#define WEBSET_DEBUG

//Tests
//#define UTEST_BMS_FILTER

//System
#define WEBSERVER_PORT               80

//Alarmrules
#define CNT_ALARMS                   10
#define CNT_BT_ALARMS_RULES          10
//
//DI/DO
#define H_CLK                        14
#define H_MOSI                       13
#define H_MISO                       12
#define IO_DO_PL                     26
#define CNT_DIGITALOUT                6
#define CNT_DIGITALIN                 4
#define GPIO_LED1_HW1                 0

//Tacho
#define TACHO_ADDR0                   6
#define TACHO_ADDR1                   7
#define TACHO_ADDR2                  15
#define TACHO_GPIO                   27
#define TACHO_MEAS_TIME            3000

//Onewire
#define MAX_ANZAHL_OW_SENSOREN         64
#define TEMP_IF_SENSOR_READ_ERROR  0xFF00
#define OW_TEMP_AVG_CYCELS              3

//Bluetooth
#define BT_DEVICES_COUNT              7
#define BT_SCAN_RESULTS               5
#define BT_SCAN_AND_NOT_CONNECT_TIME 11 //secounds

//Serial
#define SERIAL_BMS_DEVICES_COUNT      3
#define SERIAL_BMS_SEPLOS_COUNT       2
enum serialRxTxEn_e {serialRxTx_RxTxDisable, serialRxTx_TxEn, serialRxTx_RxEn};

#define SERIAL1_PIN_RX               16
#define SERIAL1_PIN_TX               17
#define SERIAL1_PIN_TX_EN            18
#define SERIAL2_PIN_RX               23
#define SERIAL2_PIN_TX               25
#define SERIAL2_PIN_RX_EN             3
#define SERIAL2_PIN_TX_EN             2
#define SERIAL3_PIN_RX               35
#define SERIAL3_PIN_TX               33
#define SERIAL3_PIN_TX_EN            32

//Onewire (Temperatur)
#define OW_PIN                       19
#define COUNT_TEMP_RULES             10

//I2C
#define ID_I2C_MASTER                  0 //Auswahl Master/Slave
#define I2C_DEV_ADDR_DISPLAY        0x55
#define I2C_DEV_ADDR_SLAVE1           16
#define I2C_DEV_ADDR_SLAVE2           17
#define I2C_DEV_ADDR_SERIAL_EXTENSION 32
#define I2C_SDA_PIN                   21
#define I2C_SCL_PIN                   22
#define I2C_FREQUENCY           1000000U
//#define I2C_FREQUENCY           400000U
#define I2C_CNT_SLAVES                2


//BMS Data
#define SERIAL_BMS_EXT_COUNT        8

#define BMSDATA_LAST_DEV_BT         BT_DEVICES_COUNT-1 
#define BMSDATA_FIRST_DEV_SERIAL    BMSDATA_LAST_DEV_BT+1 
#define BMSDATA_LAST_DEV_SERIAL     BMSDATA_FIRST_DEV_SERIAL+SERIAL_BMS_DEVICES_COUNT-1 
#define BMSDATA_FIRST_DEV_EXT       BMSDATA_LAST_DEV_SERIAL+1 
#define BMSDATA_LAST_DEV_EXT        BMSDATA_FIRST_DEV_EXT+SERIAL_BMS_EXT_COUNT-1


#define BMSDATA_NUMBER_ALLDEVICES BT_DEVICES_COUNT+SERIAL_BMS_DEVICES_COUNT+SERIAL_BMS_EXT_COUNT


// Register
#define RTC_CNTL_SDIO_CONF_REG 0x3FF48074

// Rester RTC_CNTL_SDIO_CONF_REG
#define RTC_CNTL_XPD_SDIO_VREG 0x80000000
#define RTC_CNTL_SDIO_TIEH     0x800000


/*********************************************
 * Parameter IDs
 *********************************************/
#define ID_PARAM_SERIAL_CONNECT_DEVICE               1
#define ID_PARAM_SERIAL_ALARM_AKTION                 6          //0=AUS; 1-5=Alarm 1-5
#define ID_PARAM_SERIAL_ALARM_TIME_OUT               7

#define ID_PARAM_SS_CAN                              2
#define ID_PARAM_SS_BT                               3
#define ID_PARAM_SS_BTDEV                            4
#define ID_PARAM_SS_BTDEVMAC                         5

#define ID_PARAM_ALARM_BTDEV_BMS_SELECT              9
#define ID_PARAM_ALARM_BT                           10
//#define ID_PARAM_ALARM_BTDEV_ALARM_ON               11
#define ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT         12
//#define ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON         13
#define ID_PARAM_ALARM_BT_CNT_CELL_CTRL             14
#define ID_PARAM_ALARM_BT_CELL_SPG_MIN              15
#define ID_PARAM_ALARM_BT_CELL_SPG_MAX              16
#define ID_PARAM_ALARM_BTDEV_ALARM_AKTION           17
#define ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION     18
#define ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION   19          //0=AUS; 1-5=Alarm 1-5

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

#define ID_PARAM_WLAN_SSID                          40
#define ID_PARAM_WLAN_PWD                           41
#define ID_PARAM_MQTT_SERVER_IP                     42
#define ID_PARAM_MQTT_SERVER_PORT                   43
#define ID_PARAM_MQTT_SERVER_ENABLE                 44
#define ID_PARAM_MQTT_DEVICE_NAME                   45

#define ID_PARAM_ONWIRE_ENABLE                      50
#define ID_PARAM_ONEWIRE_ADR                        51
#define ID_PARAM_ONWIRE_TEMP_OFFSET                 52

#define ID_PARAM_BMS_CAN_ENABLE                     60
#define ID_PARAM_BMS_CAN_DATASOURCE                 61
#define ID_PARAM_BMS_MAX_CHARGE_SPG                 62
#define ID_PARAM_BMS_MAX_CHARGE_CURRENT             64
#define ID_PARAM_BMS_MAX_DISCHARGE_CURRENT          65
#define ID_PARAM_BMS_LADELEISTUNG_AUF_NULL          66
#define ID_PARAM_BMS_ENTLADELEISTUNG_AUF_NULL       67
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
#define ID_PARAM_INVERTER_LADESTROM_SPERRZEIT                           82
#define ID_PARAM_BMS_CAN_DATASOURCE_SS1                                 83
#define ID_PARAM_BMS_CAN_DATASOURCE_SS2                                 84
#define ID_PARAM_BMS_CAN_DATASOURCE_SS3                                 85

#define ID_PARAM_MQTT_USERNAME                                   86
#define ID_PARAM_MQTT_PWD                                        87

#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_EN              88 
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG             89
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SOC             90
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_TIME            91
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END         92

#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_EN       93
#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_ZELLSPG  94
#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_DELTA    95

#define ID_PARAM_WLAN_CONNECT_TIMEOUT         96

#define ID_PARAM_INVERTER_BATT_TEMP_QUELLE    97
#define ID_PARAM_INVERTER_BATT_TEMP_SENSOR    98

#define ID_PARAM_NEEY_CELLS                   99
#define ID_PARAM_NEEY_START_VOLTAGE          100
#define ID_PARAM_NEEY_MAX_BALANCE_CURRENT    101
#define ID_PARAM_NEEY_SLEEP_VOLTAGE          102
#define ID_PARAM_NEEY_EQUALIZATION_VOLTAGE   103
#define ID_PARAM_NEEY_BAT_CAPACITY           104
#define ID_PARAM_NEEY_BAT_TYPE               105
#define ID_PARAM_NEEY_BUZZER                 106 //not use
#define ID_PARAM_NEEY_BALANCER_ON            107

#define ID_PARAM_SERIAL_SEPLOS_CONNECT_TO_ID 108

#define ID_PARAM_TEMP_SENSOR_TIMEOUT_TRIGGER 109
#define ID_PARAM_TEMP_SENSOR_TIMEOUT_TIME    110

#define ID_PARAM_MASTER_SLAVE_TYP            111

#define ID_PARAM_BMS_ALARM_HIGH_BAT_VOLTAGE  112
#define ID_PARAM_BMS_ALARM_LOW_BAT_VOLTAGE   113
#define ID_PARAM_BMS_ALARM_HIGH_TEMPERATURE  114
#define ID_PARAM_BMS_ALARM_LOWTEMPERATURE    115

#define ID_PARAM_INVERTER_MULTI_BMS_VALUE_SOC 116

#define ID_PARAM_TRIGGER_NAMES                117

#define ID_PARAM_BATTERY_PACK_CHARGE_CURRENT    118
#define ID_PARAM_BATTERY_PACK_DISCHARGE_CURRENT 119

#define ID_PARAM_BMS_FILTER_CELL_VOLTAGE_PERCENT 120
#define ID_PARAM_BMS_FILTER_RX_ERROR_COUNT       121

#define ID_PARAM_SYSTEM_NTP_SERVER_NAME          122
#define ID_PARAM_SYSTEM_NTP_SERVER_PORT          123



//Auswahl Bluetooth Geräte
#define ID_BT_DEVICE_NB             0
#define ID_BT_DEVICE_NEEY4A         1
#define ID_BT_DEVICE_JKBMS_JK02     2
#define ID_BT_DEVICE_JKBMS_JK02_32S 3

//Auswahl Serial Geräte
#define ID_SERIAL_DEVICE_NB         0
#define ID_SERIAL_DEVICE_JBDBMS     1
#define ID_SERIAL_DEVICE_JKBMS      2
#define ID_SERIAL_DEVICE_SEPLOSBMS  3
#define ID_SERIAL_DEVICE_DALYBMS    4

//Auswahl CAN Geräte
#define ID_CAN_DEVICE_NB            0
#define ID_CAN_DEVICE_SOLISRHI      1
#define ID_CAN_DEVICE_DEYE          2
#define ID_CAN_DEVICE_VICTRON       3

//Auswahl Temp.Alarm Funktionen
#define ID_TEMP_ALARM_FUNKTION_NB               0
#define ID_TEMP_ALARM_FUNKTION_MAXWERT          1
#define ID_TEMP_ALARM_FUNKTION_MAXWERT_REFERENZ 2
#define ID_TEMP_ALARM_FUNKTION_DIFFERENZ        3

//Auswahl
#define OPTION_MULTI_BMS_SOC_AVG                1
#define OPTION_MULTI_BMS_SOC_MAX                2





/*********************************************
 * I2C
 *********************************************/
#define TXBUFF_OFFSET                     0x04

#define BMS_DATA                          0x01  //BMS-Daten
#define INVERTER_DATA                     0x02  //Inverter-Daten
#define BSC_DATA                          0x03  //
#define BSC_GET_SLAVE_DATA                0x0A  //

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

//BSC_GET_SLAVE_DATA 0x0A
//#define BMS_GET_ALL_DATA                  0x01


/*********************************************
 * MQTT
 *********************************************/
#define MQTT_TOPIC_BMS_BT                        1
#define MQTT_TOPIC_TEMPERATUR                    2
#define MQTT_TOPIC_ALARM                         3
#define MQTT_TOPIC_INVERTER                      4
#define MQTT_TOPIC_SYS                           5
#define MQTT_TOPIC_BMS_SERIAL                    6

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


static const char* mqttTopics[] PROGMEM = {"", // 0
  "bms/bt",        // 1
  "temperatur",    // 2
  "trigger",       // 3
  "inverter",      // 4
  "sys",           // 5
  "bms/serial",    // 6
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
  "dischargeCurrentSoll",      // 26  frei
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
  "",                          // 42
  "",                          // 43
  "",                          // 44
  "",                          // 45
  };

#endif