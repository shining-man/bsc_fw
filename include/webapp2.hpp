// Copyright (c) 2023 Tobias Himmler
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef WEBAPP2_H
#define WEBAPP2_H

#include <Arduino.h>
#include <WebServer.h>
#include <WebSettings.h>
#include <BleHandler.h>
#include <BscSerial.h>

void initWebApp2(WebServer *lserver, WebSettings *lwebSettings, BleHandler *lbleHandler, BscSerial *lBscSerial);


#endif