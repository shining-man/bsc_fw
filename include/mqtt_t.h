// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef MQTT_T_H
#define MQTT_T_H

#include <Arduino.h>

void initMqtt();
bool mqttLoop();
  
bool mqttConnect();
void mqttDisconnect();
bool mqttConnected();
uint16_t getTxBufferSize();

void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, String value);
void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, uint32_t value);
void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, int32_t value);
void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, float value);
void mqttPublish(int8_t t1, int8_t t2, int8_t t3, int8_t t4, bool value);


#endif