// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef DEFINES_H
#define DEFINES_H

#define BSC_SW_VERSION      "V0.1.1"

//System
#define WEBSERVER_PORT               80


//Onewire
#define MAX_ANZAHL_OW_SENSOREN       64
#define TEMP_IF_SENSOR_READ_ERROR  0xFF
#define OW_TEMP_AVG_CYCELS            3

//Bluetooth
//#define BT_SCAN_PASUSE_TIME       11
#define BT_DEVICES_COUNT             5

//Serial
#define SERIAL_BMS_DEVICES_COUNT     3

//Temperatur
#define COUNT_TEMP_RULES            10




//Parameter IDs
#define ID_PARAM_WLAN_SSID                      40
#define ID_PARAM_WLAN_PWD                       41
#define ID_PARAM_MQTT_SERVER_IP                 42
#define ID_PARAM_MQTT_SERVER_PORT               43
#define ID_PARAM_MQTT_SERVER_ENABLE             44
#define ID_PARAM_MQTT_DEVICE_NAME               45

#define ID_PARAM_SERIAL_CONNECT_DEVICE           1
#define ID_PARAM_SERIAL_ALARM_AKTION             6          //0=AUS; 1-5=Alarm 1-5
#define ID_PARAM_SERIAL_ALARM_TIME_OUT           7

#define ID_PARAM_SS_CAN                          2
#define ID_PARAM_SS_BT                           3
#define ID_PARAM_SS_BTDEV                        4
#define ID_PARAM_SS_BTDEVMAC                     5

#define ID_PARAM_ALARM_BT                       10
#define ID_PARAM_ALARM_BTDEV_ALARM_ON           11
#define ID_PARAM_ALARM_BTDEV_ALARM_TIME_OUT     12
#define ID_PARAM_ALARM_BT_CELL_SPG_ALARM_ON     13
#define ID_PARAM_ALARM_BT_CNT_CELL_CTRL         14
#define ID_PARAM_ALARM_BT_CELL_SPG_MIN          15
#define ID_PARAM_ALARM_BT_CELL_SPG_MAX          16
#define ID_PARAM_ALARM_BTDEV_ALARM_AKTION       17
#define ID_PARAM_ALARM_BT_CELL_SPG_ALARM_AKTION     18
#define ID_PARAM_ALARM_BT_GESAMT_SPG_ALARM_AKTION   19          //0=AUS; 1-5=Alarm 1-5
#define ID_PARAM_ALARM_BT_GESAMT_SPG_MIN            72
#define ID_PARAM_ALARM_BT_GESAMT_SPG_MAX            73

#define ID_PARAM_TEMP_ALARM                     20
#define ID_PARAM_TEMP_ALARM_SENSOR_VON          21
#define ID_PARAM_TEMP_ALARM_SENSOR_BIS          22
#define ID_PARAM_TEMP_ALARM_SENSOR_VERGLEICH    23
#define ID_PARAM_TEMP_ALARM_WERT1               24
#define ID_PARAM_TEMP_ALARM_WERT2               25
#define ID_PARAM_TEMP_ALARM_AKTION              26
#define ID_PARAM_TEMP_ALARM_UEBERWACH_FUNKTION  27

#define ID_PARAM_DO_AUSLOESEVERHALTEN           30
#define ID_PARAM_DO_IMPULSDAUER                 31
#define ID_PARAM_DO_AUSLOESUNG_BEI              32
#define ID_PARAM_DO_VERZOEGERUNG                33

#define ID_PARAM_DI_INVERTIERT                  34
#define ID_PARAM_DI_ALARM_NR                    35

#define ID_PARAM_ONWIRE_ENABLE                  50
#define ID_PARAM_ONEWIRE_ADR                    51
#define ID_PARAM_ONWIRE_TEMP_OFFSET             52

#define ID_PARAM_BMS_CAN_ENABLE                 60
#define ID_PARAM_BMS_CAN_DATASOURCE             61
#define ID_PARAM_BMS_MAX_CHARGE_SPG             62
//#define ID_PARAM_BMS_MIN_CHARGE_SPG             63
#define ID_PARAM_BMS_MAX_CHARGE_CURRENT         64
#define ID_PARAM_BMS_MAX_DISCHARGE_CURRENT      65
#define ID_PARAM_BMS_LADELEISTUNG_AUF_NULL      66
#define ID_PARAM_BMS_ENTLADELEISTUNG_AUF_NULL   67
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLDRIFT_EN      68
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTABWEICHUNG   69
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_A_PRO_MV          70
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_STARTSPG_ZELLE    71
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_EN        74
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_STARTSPG  75
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_ENDSPG    76
#define ID_PARAM_BMS_SOC_AUF_FULL                                77
#define ID_PARAM_INVERTER_LADESTROM_REDUZIEREN_ZELLSPG_MINDEST_STROM 78

//Auswahl Bluetooth Geräte
#define ID_BT_DEVICE_NB           0
#define ID_BT_DEVICE_NEEY4A       1

//Auswahl Serial Geräte
#define ID_SERIAL_DEVICE_NB       0
#define ID_SERIAL_DEVICE_JBDBMS   1

//Auswahl CAN Geräte
#define ID_CAN_DEVICE_NB          0
#define ID_CAN_DEVICE_SOLISRHI    1

//Auswahl Temp.Alarm Funktionen
#define ID_TEMP_ALARM_FUNKTION_NB  0
#define ID_TEMP_ALARM_FUNKTION_MAXWERT 1
#define ID_TEMP_ALARM_FUNKTION_MAXWERT_REFERENZ 2



#endif