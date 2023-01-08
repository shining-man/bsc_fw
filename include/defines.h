// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DEFINES_H
#define DEFINES_H


#define BSC_SW_VERSION      "V0.2.7"

//Debug
#define DEBUG_ON_FS
//#define DEBUG_ON_HW_SERIAL
//#define LOG_TO_SERIAL
#define DEBUG_SW_BAUDRATE         19200

//BMS
//#define JK_DEBUG
//#define SEPLOS_DEBUG

//System
#define WEBSERVER_PORT               80

//Alarmrules
#define CNT_ALARMS                   10

//DI/DO
#define CNT_DIGITALOUT                6
#define CNT_DIGITALIN                 4

//Onewire
#define MAX_ANZAHL_OW_SENSOREN       64
#define TEMP_IF_SENSOR_READ_ERROR  0xFF
#define OW_TEMP_AVG_CYCELS            3

//Bluetooth
//#define BT_DEBUG
#define BT_DEVICES_COUNT              7
#define BT_SCAN_RESULTS               5
#define BT_SCAN_AND_NOT_CONNECT_TIME 11 //secounds

//Serial
#define SERIAL_BMS_DEVICES_COUNT      3

#define SERIAL1_PIN_RX               16
#define SERIAL1_PIN_TX               17
#define SERIAL1_PIN_TX_EN            18
#define SERIAL2_PIN_RX               23
#define SERIAL2_PIN_TX               25
#define SERIAL2_PIN_TX_EN             0
#define SERIAL3_PIN_RX               35
#define SERIAL3_PIN_TX               33
#define SERIAL3_PIN_TX_EN            32

//Onewire (Temperatur)
#define OW_PIN                       19
#define COUNT_TEMP_RULES             10

//I2C
#define I2C_DEV_ADDR_DISPLAY       0x55
#define I2C_SDA_PIN                  21
#define I2C_SCL_PIN                  22
#define I2C_FREQUENCY          1000000U



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

#define ID_PARAM_ALARM_BT                           10
#define ID_PARAM_ALARM_BTDEV_ALARM_ON               11
#define ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT         12
#define ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON         13
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

#define ID_PARAM_MQTT_USERNAME  86
#define ID_PARAM_MQTT_PWD       87

#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_EN      88 
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG     89
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SOC     90
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_TIME    91
#define ID_PARAM_INVERTER_SOC_BELOW_ZELLSPANNUNG_SPG_END 92

#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_EN       93
#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_ZELLSPG  94
#define ID_PARAM_INVERTER_CHARGE_VOLTAGE_DYNAMIC_REDUCE_DELTA    95




//Auswahl Bluetooth Geräte
#define ID_BT_DEVICE_NB           0
#define ID_BT_DEVICE_NEEY4A       1

//Auswahl Serial Geräte
#define ID_SERIAL_DEVICE_NB        0
#define ID_SERIAL_DEVICE_JBDBMS    1
#define ID_SERIAL_DEVICE_JKBMS     2
#define ID_SERIAL_DEVICE_SEPLOSBMS 3

//Auswahl CAN Geräte
#define ID_CAN_DEVICE_NB          0
#define ID_CAN_DEVICE_SOLISRHI    1
#define ID_CAN_DEVICE_DEYE        2
#define ID_CAN_DEVICE_VICTRON     3

//Auswahl Temp.Alarm Funktionen
#define ID_TEMP_ALARM_FUNKTION_NB               0
#define ID_TEMP_ALARM_FUNKTION_MAXWERT          1
#define ID_TEMP_ALARM_FUNKTION_MAXWERT_REFERENZ 2
#define ID_TEMP_ALARM_FUNKTION_DIFFERENZ        3



/*********************************************
 * I2C
 *********************************************/
#define TXBUFF_OFFSET                     0x04

#define BMS_DATA                          0x01  //BMS-Daten
#define INVERTER_DATA                     0x02  //Inverter-Daten
#define BSC_DATA                          0x03  //

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

//INVERTER_DATA 0x02
#define INVERTER_VOLTAGE                  0x01 
#define INVERTER_CURRENT                  0x02 
#define INVERTER_SOC                      0x03 
#define INVERTER_CHARGE_CURRENT           0x04 
#define INVERTER_DISCHARG_CURRENT         0x05 

//BSC_DATA 0x03
#define BSC_ALARMS                        0x01 
#define BSC_IP_ADDR                       0x02



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
#define MQTT_TOPIC2_FET_STATUS                  26
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


static const char* mqttTopics[] PROGMEM = {"", // 0
  "bms/bt",        // 1
  "temperatur",    // 2
  "alarm",         // 3
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
  "FetStatus",                 // 26
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
  "",                          // 38
  "",                          // 39
  "",                          // 40
  };

#endif