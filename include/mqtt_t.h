// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef MQTT_T_H
#define MQTT_T_H

#include <Arduino.h>

static String mqttDeviceName;

void initMqtt();
void mqttLoop();
  
void mqttConnect();
void mqttDisconnect();


void mqttPublish(String topic, String value);
void mqttPublish(String topic, uint32_t value);
void mqttPublish(String topic, int32_t value);
void mqttPublish(String topic, uint8_t value);
void mqttPublish(String topic, int8_t value);
void mqttPublish(String topic, float value);
void mqttPublish(String topic, bool value);





#endif