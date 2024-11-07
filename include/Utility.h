// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdint.h>
#include <string>

bool parseMacAddress(const std::string& macStr, uint8_t* mac);
std::string macAddressToString(const uint8_t* mac);

#endif