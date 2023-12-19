// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef RESTAPI_H
#define RESTAPI_H

#include <Arduino.h>
#include <WebServer.h>


void buildJsonRest(WebServer * server);
void handle_setParameter(WebServer * server);

#endif